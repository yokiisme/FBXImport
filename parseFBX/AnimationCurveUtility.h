#pragma once
#include "AnimationImporter.h"


#define kDefaultWeight 1.0F / 3.0F

template<class T>
static T DefaultWeight() { return kDefaultWeight; }

template<>
Quaternionf DefaultWeight<Quaternionf>() { return Quaternionf(kDefaultWeight, kDefaultWeight, kDefaultWeight, kDefaultWeight); }

template<>
Vector3f DefaultWeight<Vector3f>() { return Vector3f(kDefaultWeight, kDefaultWeight, kDefaultWeight); }

#undef kDefaultWeight

template<class T>
static T Zero() { return T(); }

const float kCurveTimeEpsilon = 0.00001f;

template<class T>
T SafeDeltaDivide(T y, float x)
{
	if (Abs(x) > kCurveTimeEpsilon)
		return y / x;
	else
		return Zero<T>();
}

template<class T>
void RecalculateSplineSlopeT(FBXAnimationCurveTpl<T>& curve, int key, float bias = 0.0F);

template<class T>
void RecalculateSplineSlopeT(FBXAnimationCurveTpl<T>& curve, int key, float b)
{
	//Assert(key >= 0 && key < curve.GetKeyCount());
	if (curve.GetKeyCount() < 2)
		return;

	// First keyframe
	// in and out slope are set to be the slope from this to the right key
	if (key == 0)
	{
		float dx = curve.GetKey(1).time - curve.GetKey(0).time;
		T dy = curve.GetKey(1).value - curve.GetKey(0).value;
		T m = dy / dx;
		curve.GetKey(key).inSlope = m; curve.GetKey(key).outSlope = m;
		curve.GetKey(key).outWeight = DefaultWeight<T>();
	}
	// last keyframe
	// in and out slope are set to be the slope from this to the left key
	else if (key == curve.GetKeyCount() - 1)
	{
		float dx = curve.GetKey(key).time - curve.GetKey(key - 1).time;
		T dy = curve.GetKey(key).value - curve.GetKey(key - 1).value;
		T m = dy / dx;
		curve.GetKey(key).inSlope = m; curve.GetKey(key).outSlope = m;
		curve.GetKey(key).inWeight = DefaultWeight<T>();
	}
	// Keys are on the left and right
	// Calculates the slopes from this key to the left key and the right key.
	// Then blend between them using the bias
	// A bias of zero doesn't bend in any direction
	// a positive bias bends to the right
	else
	{
		float dx1 = curve.GetKey(key).time - curve.GetKey(key - 1).time;
		T dy1 = curve.GetKey(key).value - curve.GetKey(key - 1).value;

		float dx2 = curve.GetKey(key + 1).time - curve.GetKey(key).time;
		T dy2 = curve.GetKey(key + 1).value - curve.GetKey(key).value;

		T m1 = SafeDeltaDivide(dy1, dx1);
		T m2 = SafeDeltaDivide(dy2, dx2);

		T m = (1.0F + b) * 0.5F * m1 + (1.0F - b) * 0.5F * m2;
		curve.GetKey(key).inSlope = m; curve.GetKey(key).outSlope = m;

		curve.GetKey(key).inWeight = DefaultWeight<T>();
		curve.GetKey(key).outWeight = DefaultWeight<T>();
	}

	curve.InvalidateCache();
}

template<class T>
void RecalculateSplineSlope(FBXAnimationCurveTpl<T>& curve)
{
	for (int i = 0; i < curve.GetKeyCount(); i++)
		RecalculateSplineSlopeT(curve, i);
}

void RecalculateSplineSlope(FBXAnimationCurve& curve, int key, float bias)
{
	RecalculateSplineSlopeT<float>(curve, key, bias);
}

typedef FBXAnimationCurveTpl<float>        FBXAnimationCurve;
void EnsureQuaternionContinuity(FBXAnimationCurve** curves)
{
	if (!curves[0] || !curves[1] || !curves[2] || !curves[3])
		return;

	int keyCount = curves[0]->GetKeyCount();
	if (keyCount != curves[1]->GetKeyCount() || keyCount != curves[2]->GetKeyCount() || keyCount != curves[3]->GetKeyCount())
		return;

	if (keyCount == 0)
		return;

	Quaternionf last(curves[0]->GetKey(keyCount - 1).value, curves[1]->GetKey(keyCount - 1).value, curves[2]->GetKey(keyCount - 1).value, curves[3]->GetKey(keyCount - 1).value);
	for (int i = 0; i < keyCount; i++)
	{
		Quaternionf cur(curves[0]->GetKey(i).value, curves[1]->GetKey(i).value, curves[2]->GetKey(i).value, curves[3]->GetKey(i).value);
		if (Dot(cur, last) < 0.0F)
			cur = Quaternionf(-cur.x, -cur.y, -cur.z, -cur.w);
		last = cur;
		curves[0]->GetKey(i).value = cur.x;
		curves[1]->GetKey(i).value = cur.y;
		curves[2]->GetKey(i).value = cur.z;
		curves[3]->GetKey(i).value = cur.w;
	}

	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < keyCount; i++)
			RecalculateSplineSlopeT(*curves[j], i);
	}
}

template<class T>
void RecalculateSplineSlopeLinear(FBXAnimationCurveTpl<T>& curve)
{
	if (curve.GetKeyCount() < 2)
		return;

	for (int i = 0; i < curve.GetKeyCount() - 1; i++)
	{
		RecalculateSplineSlopeLinear(curve, i);
	}
}

template<class T>
void RecalculateSplineSlopeLinear(FBXAnimationCurveTpl<T>& curve, int key)
{
	//Assert(key >= 0 && key < curve.GetKeyCount() - 1);
	if (curve.GetKeyCount() < 2)
		return;

	float dx = curve.GetKey(key).time - curve.GetKey(key + 1).time;
	//Assert(dx != 0.0f);

	T dy = curve.GetKey(key).value - curve.GetKey(key + 1).value;
	T m = dy / dx;

	curve.GetKey(key).outSlope = m;
	curve.GetKey(key + 1).inSlope = m;
}


// Calculates Hermite curve coefficients
void HermiteCooficients(double t, double& a, double& b, double& c, double& d)
{
	double t2 = t * t;
	double t3 = t2 * t;

	a = 2.0F * t3 - 3.0F * t2 + 1.0F;
	b = t3 - 2.0F * t2 + t;
	c = t3 - t2;
	d = -2.0F * t3 + 3.0F * t2;
}

namespace TToArray
{
	template<class T> float& Index(T& value, int index) { return value[index]; }
	template<class T> float  Index(const T& value, int index) { return value[index]; }

	template<> float& Index<float>(float& value, int index)
	{
		//Assert(index == 0);
		return value;
	}

	template<> float  Index<float>(const float& value, int index)
	{
		//Assert(index == 0);
		return value;
	}

	template<class T> int CoordinateCount();
	template<> int CoordinateCount<float>() { return 1; }
	template<> int CoordinateCount<Vector3f>() { return 3; }
	template<> int CoordinateCount<Quaternionf>() { return 4; }
}



template<class T>
void FitTangents(FBXKeyFrameTpl<T>& key0, FBXKeyFrameTpl<T>& key1, float time1, float time2, const T& value1, const T& value2)
{
	//Assert(fabsf(time1) >= std::numeric_limits<float>::epsilon());
	//Assert(fabsf(time2) >= std::numeric_limits<float>::epsilon());

	const float dt = key1.time - key0.time;

	const int coordinateCount = TToArray::CoordinateCount<T>();

	if (fabsf(dt) < std::numeric_limits<float>::epsilon())
	{
		for (int i = 0; i < coordinateCount; ++i)
		{
			TToArray::Index(key0.outSlope, i) = 0;
			TToArray::Index(key1.inSlope, i) = 0;
		}
	}
	else
	{
		// p0 and p1 for Hermite curve interpolation equation
		const T p0 = key0.value;
		const T p1 = key1.value;

		// Hermite coefficients at points time1 and time2
		double a1, b1, c1, d1;
		double a2, b2, c2, d2;

		// TODO : try using doubles, because it doesn't work well when p0==p1==v0==v1
		HermiteCooficients(time1, a1, b1, c1, d1);
		HermiteCooficients(time2, a2, b2, c2, d2);

		for (int i = 0; i < coordinateCount; ++i)
		{
			// we need to solve these two equations in order to find m0 and m1
			// b1 * m0 + c1 * m1 = v0 - a1 * p0 - d1 * p1;
			// b2 * m0 + c2 * m1 = v1 - a2 * p0 - d2 * p1;

			// c1, c2 is never equal 0, because time1 and time2 not equal to 0

			// divide by c1 and c2
			// b1 / c1 * m0 + m1 = (v0 - a1 * p0 - d1 * p1) / c1;
			// b2 / c2 * m0 + m1 = (v1 - a2 * p0 - d2 * p1) / c2;

			// subtract one from another
			// b1 / c1 * m0 - b2 / c2 * m0 = (v0 - a1 * p0 - d1 * p1) / c1 - (v1 - a2 * p0 - d2 * p1) / c2;

			// solve for m0
			// (b1 / c1 - b2 / c2) * m0 = (v0 - a1 * p0 - d1 * p1) / c1 - (v1 - a2 * p0 - d2 * p1) / c2;

			const double v0 = TToArray::Index(value1, i);
			const double v1 = TToArray::Index(value2, i);
			const double pp0 = TToArray::Index(p0, i);
			const double pp1 = TToArray::Index(p1, i);

			// calculate m0
			const double m0 = ((v0 - a1 * pp0 - d1 * pp1) / c1 - (v1 - a2 * pp0 - d2 * pp1) / c2) / (b1 / c1 - b2 / c2);

			// solve for m1 using m0
			// c1 * m1 = p0 - a1 * p0 - d1 * p1 - b1 * m0;

			// calculate m1
			const double m1 = (v0 - a1 * pp0 - d1 * pp1 - b1 * m0) / c1;

			TToArray::Index(key0.outSlope, i) = static_cast<float>(m0 / dt);
			TToArray::Index(key1.inSlope, i) = static_cast<float>(m1 / dt);
		}
	}
}
