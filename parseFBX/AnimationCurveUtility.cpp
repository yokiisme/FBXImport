#include "AnimationCurveUtility.h"


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


void RecalculateSplineSlope(FBXAnimationCurve& curve, int key, float bias)
{
	RecalculateSplineSlopeT<float>(curve, key, bias);
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

void EnsureQuaternionContinuityAndRecalculateSlope(FBXAnimationCurveQuat& curve)
{
	if (!curve.IsValid())
		return;

	int keyCount = curve.GetKeyCount();

	Quaternionf last(curve.GetKey(keyCount - 1).value);
	for (int i = 0; i < keyCount; i++)
	{
		Quaternionf cur(curve.GetKey(i).value);
		if (Dot(cur, last) < 0.0F)
			cur = Quaternionf(-cur.x, -cur.y, -cur.z, -cur.w);
		last = cur;
		curve.GetKey(i).value = cur;
		DebugAssert(FBXAnimationCurveQuat::FBXKeyframe::IsValid(curve.GetKey(i)));
	}

	for (int i = 0; i < keyCount; i++)
		RecalculateSplineSlopeT(curve, i);
}

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

template void FitTangents<float>(FBXKeyFrameTpl<float>&, FBXKeyFrameTpl<float>&, float time1, float time2, const float& value1, const float& value2);