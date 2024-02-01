#pragma once
#include "AnimationImporter.h"
#include "AnimationCurve.h"
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
void RecalculateSplineSlope(FBXAnimationCurveTpl<T>& curve)
{
	for (int i = 0; i < curve.GetKeyCount(); i++)
		RecalculateSplineSlopeT(curve, i);
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

template<class T>
void FitTangents(FBXKeyFrameTpl<T>& key0, FBXKeyFrameTpl<T>& key1, float time1, float time2, const T& value1, const T& value2);


void HermiteCooficients(double t, double& a, double& b, double& c, double& d);
void EnsureQuaternionContinuityAndRecalculateSlope(FBXAnimationCurveQuat& curve);