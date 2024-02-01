#pragma once
#include "Utility.h"
#include "EnumFlags.h"

enum InternalWrapMode
{
	// Values are serialized in CompressedMesh and must be preserved
	kInternalWrapModePingPong = 0,
	kInternalWrapModeRepeat = 1,
	kInternalWrapModeClamp = 2,
	kInternalWrapModeDefault = 3
};


enum WeightedMode
{
	kNotWeighted = 0,
	kInWeighted = 1 << 0,
	kOutWeighted = 1 << 1,
	kBothWeighted = kInWeighted | kOutWeighted
};
ENUM_FLAGS(WeightedMode);


template<class T>
struct FBXKeyFrameTpl
{
	float time;
	int weightedMode;

	T value;
	T inSlope;
	T outSlope;
	T inWeight;
	T outWeight;

	inline static bool IsValid(const FBXKeyFrameTpl<T>& key) { return IsFinite(key.value) && IsFinite(key.time); }

};

template<class T>
struct FBXAnimationCurveTpl
{
	typedef FBXKeyFrameTpl<T> FBXKeyframe;


	std::vector<FBXKeyframe> m_Curve;
	InternalWrapMode preInfinity;
	InternalWrapMode postInfinity;
	RotationOrder rotationOrder;

	void SetRotationOrder(RotationOrder order) { rotationOrder = order; }
	RotationOrder GetRotationOrder() const { return (RotationOrder)rotationOrder; }
	int GetKeyCount() const { return (int)m_Curve.size(); }
	FBXKeyframe& GetKey(int index) { return m_Curve[index]; }

	int AddKey(const FBXKeyframe& key);
	/// Performs no error checking. And doesn't invalidate the cache!
	void AddKeyBackFast(const FBXKeyframe& key) { m_Curve.push_back(key); }
	const FBXKeyframe& GetKey(int index) const { return m_Curve[index]; }


	struct Cache
	{
		int index;
		float time;
		float timeEnd;
		T coeff[4];

		Cache() { time = std::numeric_limits<float>::infinity(); index = 0; timeEnd = 0.0f; memset(&coeff, 0, sizeof(coeff)); }
		void Invalidate() { time = std::numeric_limits<float>::infinity(); index = 0; }
	};
	mutable Cache m_Cache;
	mutable Cache m_ClampCache;
	void InvalidateCache();
	std::pair<float, float> GetRange() const;

	T Evaluate(float curveT, Cache* cache = NULL) const;
	bool IsValid() const { return m_Curve.size() >= 1 && IsFinite(GetRange().first) && IsFinite(GetRange().second); }


	void FindIndexForSampling(const Cache& cache, float curveT, int& lhs, int& rhs) const;
	void EvaluateWithoutCache(float curveT, T& output) const;
	void CalculateCacheData(Cache& cache, int lhsIndex, int rhsIndex, float timeOffset) const;
	float WrapTime(float curveT) const;
	static T InterpolateKeyframe(const FBXKeyframe& lhs, const FBXKeyframe& rhs, float curveT);
};

typedef FBXAnimationCurveTpl<float>        FBXAnimationCurveBase;
typedef FBXAnimationCurveTpl<float>        FBXAnimationCurve;
typedef FBXAnimationCurveTpl<Quaternionf>  FBXAnimationCurveQuat;
typedef FBXAnimationCurveTpl<Vector3f>     FBXAnimationCurveVec3;

void HandleSteppedCurve(const FBXKeyFrameTpl<float>& lhs, const FBXKeyFrameTpl<float>& rhs, float& value);
void HandleSteppedTangent(const FBXKeyFrameTpl<float>& lhs, const FBXKeyFrameTpl<float>& rhs, float& value);

void HandleSteppedCurve(const FBXKeyFrameTpl<Vector3f>& lhs, const FBXKeyFrameTpl<Vector3f>& rhs, Vector3f& value);
void HandleSteppedTangent(const FBXKeyFrameTpl<Vector3f>& lhs, const FBXKeyFrameTpl<Vector3f>& rhs, Vector3f& tangent);

void HandleSteppedCurve(const FBXKeyFrameTpl<Quaternionf>& lhs, const FBXKeyFrameTpl<Quaternionf>& rhs, Quaternionf& tangent);
void HandleSteppedTangent(const FBXKeyFrameTpl<Quaternionf>& lhs, const FBXKeyFrameTpl<Quaternionf>& rhs, Quaternionf& tangent);


template float FBXAnimationCurveTpl<float>::Evaluate(float curveT, Cache* cache) const;
template Quaternionf FBXAnimationCurveTpl<Quaternionf>::Evaluate(float curveT, Cache* cache) const;
template Vector3f FBXAnimationCurveTpl<Vector3f>::Evaluate(float curveT, Cache* cache) const;


template<class T>
void FBXAnimationCurveTpl<T>::InvalidateCache()
{
	m_Cache.time = std::numeric_limits<float>::infinity();
	m_Cache.index = 0;
	m_ClampCache.time = std::numeric_limits<float>::infinity();
	m_ClampCache.index = 0;
}

template<class T>
std::pair<float, float> FBXAnimationCurveTpl<T>::GetRange() const
{
	if (!m_Curve.empty())
		return std::make_pair(m_Curve[0].time, m_Curve.back().time);
	else
		return std::make_pair(std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
}

template<class T>
static T Zero() { return T(); }

template<>
Quaternionf Zero<Quaternionf>() { return Quaternionf(0.0F, 0.0F, 0.0F, 0.0F); }

template<>
Vector3f Zero<Vector3f>() { return Vector3f(0.0F, 0.0F, 0.0F); }

#define kDefaultWeight 1.0F / 3.0F

template<class T>
static T DefaultWeight() { return kDefaultWeight; }

template<>
Quaternionf DefaultWeight<Quaternionf>() { return Quaternionf(kDefaultWeight, kDefaultWeight, kDefaultWeight, kDefaultWeight); }

template<>
Vector3f DefaultWeight<Vector3f>() { return Vector3f(kDefaultWeight, kDefaultWeight, kDefaultWeight); }

#undef kDefaultWeight

template<class T>
T HermiteInterpolate(float curveT, const FBXKeyFrameTpl<T>& lhs, const FBXKeyFrameTpl<T>& rhs);

template<class T>
T BezierInterpolate(float t, T v1, T m1, T w1, T v2, T m2, T w2);

template<class T>
T BezierInterpolate(float curveT, const FBXKeyFrameTpl<T>& lhs, const FBXKeyFrameTpl<T>& rhs);

float BezierExtractU(float t, float w1, float w2);

inline float Repeat(float t, float begin, float end)
{
	return Repeat(t - begin, end - begin) + begin;
}

inline float PingPong(float t, float length)
{
	t = Repeat(t, length * 2.0F);
	t = length - Abs(t - length);
	return t;
}

inline float PingPong(float t, float begin, float end)
{
	return PingPong(t - begin, end - begin) + begin;
}
