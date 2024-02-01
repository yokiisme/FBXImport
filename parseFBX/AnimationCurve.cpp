#include "AnimationCurve.h"


template<class T>
inline void EvaluateCache(const typename FBXAnimationCurveTpl<T>::Cache& cache, float curveT, T& output)
{
	//  DebugAssert (curveT >= cache.time - kCurveTimeEpsilon && curveT <= cache.timeEnd + kCurveTimeEpsilon);
	float t = curveT - cache.time;
	output = (t * (t * (t * cache.coeff[0] + cache.coeff[1]) + cache.coeff[2])) + cache.coeff[3];
	DebugAssert(IsFinite(output));
}

void SetupStepped(float* coeff, const FBXKeyFrameTpl<float>& lhs, const FBXKeyFrameTpl<float>& rhs)
{
	// If either of the tangents in the segment are set to stepped, make the constant value equal the value of the left key
	if (lhs.outSlope == std::numeric_limits<float>::infinity() || rhs.inSlope == std::numeric_limits<float>::infinity())
	{
		coeff[0] = 0.0F;
		coeff[1] = 0.0F;
		coeff[2] = 0.0F;
		coeff[3] = lhs.value;
	}
}

void HandleSteppedCurve(const FBXKeyFrameTpl<float>& lhs, const FBXKeyFrameTpl<float>& rhs, float& value)
{
	if (lhs.outSlope == std::numeric_limits<float>::infinity() || rhs.inSlope == std::numeric_limits<float>::infinity())
		value = lhs.value;
}

void HandleSteppedTangent(const FBXKeyFrameTpl<float>& lhs, const FBXKeyFrameTpl<float>& rhs, float& tangent)
{
	if (lhs.outSlope == std::numeric_limits<float>::infinity() || rhs.inSlope == std::numeric_limits<float>::infinity())
		tangent = std::numeric_limits<float>::infinity();
}

void SetupStepped(Vector3f* coeff, const FBXKeyFrameTpl<Vector3f>& lhs, const FBXKeyFrameTpl<Vector3f>& rhs)
{
	for (int i = 0; i < 3; i++)
	{
		// If either of the tangents in the segment are set to stepped, make the constant value equal the value of the left key
		if (lhs.outSlope[i] == std::numeric_limits<float>::infinity() || rhs.inSlope[i] == std::numeric_limits<float>::infinity())
		{
			coeff[0][i] = 0.0F;
			coeff[1][i] = 0.0F;
			coeff[2][i] = 0.0F;
			coeff[3][i] = lhs.value[i];
		}
	}
}

void HandleSteppedCurve(const FBXKeyFrameTpl<Vector3f>& lhs, const FBXKeyFrameTpl<Vector3f>& rhs, Vector3f& value)
{
	for (int i = 0; i < 3; i++)
	{
		if (lhs.outSlope[i] == std::numeric_limits<float>::infinity() || rhs.inSlope[i] == std::numeric_limits<float>::infinity())
			value[i] = lhs.value[i];
	}
}

void HandleSteppedTangent(const FBXKeyFrameTpl<Vector3f>& lhs, const FBXKeyFrameTpl<Vector3f>& rhs, Vector3f& value)
{
	for (int i = 0; i < 3; i++)
	{
		if (lhs.outSlope[i] == std::numeric_limits<float>::infinity() || rhs.inSlope[i] == std::numeric_limits<float>::infinity())
			value[i] = std::numeric_limits<float>::infinity();
	}
}

void SetupStepped(Quaternionf* coeff, const FBXKeyFrameTpl<Quaternionf>& lhs, const FBXKeyFrameTpl<Quaternionf>& rhs)
{
	// If either of the tangents in the segment are set to stepped, make the constant value equal the value of the left key
	if (lhs.outSlope[0] == std::numeric_limits<float>::infinity() || rhs.inSlope[0] == std::numeric_limits<float>::infinity() ||
		lhs.outSlope[1] == std::numeric_limits<float>::infinity() || rhs.inSlope[1] == std::numeric_limits<float>::infinity() ||
		lhs.outSlope[2] == std::numeric_limits<float>::infinity() || rhs.inSlope[2] == std::numeric_limits<float>::infinity() ||
		lhs.outSlope[3] == std::numeric_limits<float>::infinity() || rhs.inSlope[3] == std::numeric_limits<float>::infinity())
	{
		for (int i = 0; i < 4; i++)
		{
			coeff[0][i] = 0.0F;
			coeff[1][i] = 0.0F;
			coeff[2][i] = 0.0F;
			coeff[3][i] = lhs.value[i];
		}
	}
}

void HandleSteppedCurve(const FBXKeyFrameTpl<Quaternionf>& lhs, const FBXKeyFrameTpl<Quaternionf>& rhs, Quaternionf& value)
{
	if (lhs.outSlope[0] == std::numeric_limits<float>::infinity() || rhs.inSlope[0] == std::numeric_limits<float>::infinity() ||
		lhs.outSlope[1] == std::numeric_limits<float>::infinity() || rhs.inSlope[1] == std::numeric_limits<float>::infinity() ||
		lhs.outSlope[2] == std::numeric_limits<float>::infinity() || rhs.inSlope[2] == std::numeric_limits<float>::infinity() ||
		lhs.outSlope[3] == std::numeric_limits<float>::infinity() || rhs.inSlope[3] == std::numeric_limits<float>::infinity())
	{
		value = lhs.value;
	}
}

void HandleSteppedTangent(const FBXKeyFrameTpl<Quaternionf>& lhs, const FBXKeyFrameTpl<Quaternionf>& rhs, Quaternionf& tangent)
{
	for (int i = 0; i < 4; i++)
	{
		if (lhs.outSlope[i] == std::numeric_limits<float>::infinity() || rhs.inSlope[i] == std::numeric_limits<float>::infinity())
			tangent[i] = std::numeric_limits<float>::infinity();
	}
}

template<class T>
T FBXAnimationCurveTpl<T>::Evaluate(float curveT, Cache* cache) const
{
	int lhsIndex, rhsIndex;
	T output;

	if (GetKeyCount() == 1)
		return m_Curve.begin()->value;

	// use custom cache for multi-threading support?
	if (!cache)
		cache = &m_Cache;

	if (curveT >= cache->time && curveT < cache->timeEnd)
	{
		//      Assert (CompareApproximately (EvaluateCache (*cache, curveT), EvaluateWithoutCache (curveT), 0.001F));
		EvaluateCache<T>(*cache, curveT, output);
		return output;
	}
	// @TODO: Optimize IsValid () away if by making the non-valid case always use the *cache codepath
	else if (IsValid())
	{
		float begTime = m_Curve[0].time;
		float endTime = m_Curve.back().time;
		float wrappedTime;

		if (curveT >= endTime)
		{
			if (postInfinity == kInternalWrapModeClamp)
			{
				cache->time = endTime;
				cache->timeEnd = std::numeric_limits<float>::infinity();
				cache->coeff[0] = cache->coeff[1] = cache->coeff[2] = Zero<T>();
				cache->coeff[3] = m_Curve.back().value;
			}
			else if (postInfinity == kInternalWrapModeRepeat)
			{
				wrappedTime = clamp(Repeat(curveT, begTime, endTime), begTime, endTime);

				FindIndexForSampling(*cache, wrappedTime, lhsIndex, rhsIndex);

				const FBXKeyframe& lhs = m_Curve[lhsIndex];
				const FBXKeyframe& rhs = m_Curve[rhsIndex];

				if (lhs.weightedMode & kOutWeighted || rhs.weightedMode & kInWeighted)
				{
					EvaluateWithoutCache(curveT, output);
					return output;
				}
				else
				{
					CalculateCacheData(*cache, lhsIndex, rhsIndex, curveT - wrappedTime);
				}
			}
			///@todo optimize pingpong by making it generate a cache too
			else
			{
				EvaluateWithoutCache(curveT, output);
				return output;
			}
		}
		else if (curveT < begTime)
		{
			if (preInfinity == kInternalWrapModeClamp)
			{
				cache->time = curveT - 1000.0F;
				cache->timeEnd = begTime;
				cache->coeff[0] = cache->coeff[1] = cache->coeff[2] = Zero<T>();
				cache->coeff[3] = m_Curve[0].value;
			}
			else if (preInfinity == kInternalWrapModeRepeat)
			{
				wrappedTime = Repeat(curveT, begTime, endTime);
				FindIndexForSampling(*cache, wrappedTime, lhsIndex, rhsIndex);

				const FBXKeyframe& lhs = m_Curve[lhsIndex];
				const FBXKeyframe& rhs = m_Curve[rhsIndex];

				if (lhs.weightedMode & kOutWeighted || rhs.weightedMode & kInWeighted)
				{
					EvaluateWithoutCache(curveT, output);
					return output;
				}
				else
				{
					CalculateCacheData(*cache, lhsIndex, rhsIndex, curveT - wrappedTime);
				}
			}
			///@todo optimize pingpong by making it generate a cache too
			else
			{
				EvaluateWithoutCache(curveT, output);
				return output;
			}
		}
		else
		{
			FindIndexForSampling(*cache, curveT, lhsIndex, rhsIndex);

			const FBXKeyframe& lhs = m_Curve[lhsIndex];
			const FBXKeyframe& rhs = m_Curve[rhsIndex];

			if (lhs.weightedMode & kOutWeighted || rhs.weightedMode & kInWeighted)
			{
				EvaluateWithoutCache(curveT, output);
				return output;
			}
			else
			{
				CalculateCacheData(*cache, lhsIndex, rhsIndex, 0.0F);
			}
		}

		//      Assert (CompareApproximately (EvaluateCache (*cache, curveT), EvaluateWithoutCache (curveT), 0.001F));
		EvaluateCache<T>(*cache, curveT, output);
		return output;
	}
	else
	{
		return Zero<T>();
	}
}

template<class T>
void FBXAnimationCurveTpl<T>::FindIndexForSampling(const Cache& cache, float curveT, int& lhs, int& rhs) const
{
	//Assert(curveT >= GetRange().first && curveT <= GetRange().second);
	int actualSize = (int)m_Curve.size();
	const FBXKeyframe* frames = &m_Curve[0];

	// Reference implementation:
	// (index is the last value that is equal to or smaller than curveT)
#if 0
	int foundIndex = 0;
	for (int i = 0; i < actualSize; i++)
	{
		if (frames[i].time <= curveT)
			foundIndex = i;
	}

	lhs = foundIndex;
	rhs = min<int>(lhs + 1, actualSize - 1);
	Assert(curveT >= m_Curve[lhs].time && curveT <= m_Curve[rhs].time);
	Assert(!(frames[rhs].time == curveT && frames[lhs].time != curveT));
	return;
#endif


#if SEARCH_AHEAD > 0
	int cacheIndex = cache.index;
	if (cacheIndex != -1)
	{
		// We can not use the cache time or time end since that is in unwrapped time space!
		float time = m_Curve[cacheIndex].time;

		if (curveT > time)
		{
			for (int i = 0; i < SEARCH_AHEAD; i++)
			{
				int index = cacheIndex + i;
				if (index + 1 < actualSize && frames[index + 1].time > curveT)
				{
					lhs = index;

					rhs = std::min<int>(lhs + 1, actualSize - 1);
					Assert(curveT >= frames[lhs].time && curveT <= frames[rhs].time);
					Assert(!(frames[rhs].time == curveT && frames[lhs].time != curveT));
					return;
				}
			}
		}
		else
		{
			for (int i = 0; i < SEARCH_AHEAD; i++)
			{
				int index = cacheIndex - i;
				if (index >= 0 && curveT >= frames[index].time)
				{
					lhs = index;
					rhs = std::min<int>(lhs + 1, actualSize - 1);
					Assert(curveT >= frames[lhs].time && curveT <= m_Curve[rhs].time);
					Assert(!(frames[rhs].time == curveT && frames[lhs].time != curveT));
					return;
				}
			}
		}
	}

#endif

	// Fall back to using binary search
	// upper bound (first value larger than curveT)
	int __len = actualSize;
	int __half;
	int __middle;
	int __first = 0;
	while (__len > 0)
	{
		__half = __len >> 1;
		__middle = __first + __half;

		if (curveT < frames[__middle].time)
			__len = __half;
		else
		{
			__first = __middle;
			++__first;
			__len = __len - __half - 1;
		}
	}

	// If not within range, we pick the last element twice
	lhs = __first - 1;
	rhs = min(actualSize - 1, __first);

	//Assert(lhs >= 0 && lhs < actualSize);
	//Assert(rhs >= 0 && rhs < actualSize);

	//Assert(curveT >= m_Curve[lhs].time && curveT <= m_Curve[rhs].time);
	//Assert(!(frames[rhs].time == curveT && frames[lhs].time != curveT));
}

template<class T>
void FBXAnimationCurveTpl<T>::CalculateCacheData(Cache& cache, int lhsIndex, int rhsIndex, float timeOffset) const
{
	const FBXKeyframe& lhs = m_Curve[lhsIndex];
	const FBXKeyframe& rhs = m_Curve[rhsIndex];
	//  DebugAssert (timeOffset >= -0.001F && timeOffset - 0.001F <= rhs.time - lhs.time);
	cache.index = lhsIndex;
	cache.time = lhs.time + timeOffset;
	cache.timeEnd = rhs.time + timeOffset;
	cache.index = lhsIndex;

	float dx, length;
	T dy;
	T m1, m2, d1, d2;

	dx = rhs.time - lhs.time;
	dx = max(dx, 0.0001F);
	dy = rhs.value - lhs.value;
	length = 1.0F / (dx * dx);

	m1 = lhs.outSlope;
	m2 = rhs.inSlope;
	d1 = m1 * dx;
	d2 = m2 * dx;

	cache.coeff[0] = (d1 + d2 - dy - dy) * length / dx;
	cache.coeff[1] = (dy + dy + dy - d1 - d1 - d2) * length;
	cache.coeff[2] = m1;
	cache.coeff[3] = lhs.value;
	SetupStepped(cache.coeff, lhs, rhs);

	DebugAssert(IsFinite(cache.coeff[0]));
	DebugAssert(IsFinite(cache.coeff[1]));
	DebugAssert(IsFinite(cache.coeff[2]));
	DebugAssert(IsFinite(cache.coeff[3]));
}

template<class T>
float FBXAnimationCurveTpl<T>::WrapTime(float curveT) const
{
	DebugAssert(IsValid());

	float begTime = m_Curve[0].time;
	float endTime = m_Curve.back().time;

	if (curveT < begTime)
	{
		if (preInfinity == kInternalWrapModeClamp)
			curveT = begTime;
		else if (preInfinity == kInternalWrapModePingPong)
			curveT = PingPong(curveT, begTime, endTime);
		else
			curveT = Repeat(curveT, begTime, endTime);
	}
	else if (curveT > endTime)
	{
		if (postInfinity == kInternalWrapModeClamp)
			curveT = endTime;
		else if (postInfinity == kInternalWrapModePingPong)
			curveT = PingPong(curveT, begTime, endTime);
		else
			curveT = Repeat(curveT, begTime, endTime);
	}
	return curveT;
}

#define FAST_CBRT_POSITIVE(x) (float)exp(log(x) / 3.0);
#define FAST_CBRT(x) (float)(((x) < 0) ? -exp(log(-(x)) / 3.0) : exp(log(x) / 3.0))

template<class T>
inline T HermiteInterpolate(float t, T p0, T m0, T m1, T p1)
{
	float t2 = t * t;
	float t3 = t2 * t;

	float a = 2.0F * t3 - 3.0F * t2 + 1.0F;
	float b = t3 - 2.0F * t2 + t;
	float c = t3 - t2;
	float d = -2.0F * t3 + 3.0F * t2;

	return a * p0 + b * m0 + c * m1 + d * p1;
}

template<class T>
T HermiteInterpolate(float curveT, const FBXKeyFrameTpl<T>& lhs, const FBXKeyFrameTpl<T>& rhs)
{
	float dx = rhs.time - lhs.time;
	T m1;
	T m2;
	float t;
	if (dx != 0.0F)
	{
		t = (curveT - lhs.time) / dx;
		m1 = lhs.outSlope * dx;
		m2 = rhs.inSlope * dx;
	}
	else
	{
		t = 0.0F;
		m1 = Zero<T>();
		m2 = Zero<T>();
	}

	return HermiteInterpolate(t, lhs.value, m1, m2, rhs.value);
}

template<class T>
inline T BezierInterpolate(float t, T p0, T p1, T p2, T p3)
{
	float t2 = t * t;
	float t3 = t2 * t;
	float omt = 1.0F - t;
	float omt2 = omt * omt;
	float omt3 = omt2 * omt;

	return omt3 * p0 + 3.0F * t * omt2 * p1 + 3.0F * t2 * omt * p2 + t3 * p3;
}

float BezierExtractU(float t, float w1, float w2)
{
	DebugAssert(t >= 0.0F && t <= 1.0F);

	float a = 3.0F * w1 - 3.0F * w2 + 1.0F;
	float b = -6.0F * w1 + 3.0F * w2;
	float c = 3.0F * w1;
	float d = -t;

	if (fabsf(a) > 1e-3f)
	{
		float p = -b / (3.0F * a);
		float p2 = p * p;
		float p3 = p2 * p;

		float q = p3 + (b * c - 3.0F * a * d) / (6.0F * a * a);
		float q2 = q * q;

		float r = c / (3.0F * a);
		float rmp2 = r - p2;

		float s = q2 + rmp2 * rmp2 * rmp2;

		if (s < 0.0F)
		{
			float ssi = sqrtf(-s);
			float r = sqrtf(-s + q2);
			float phi = atan2f(ssi, q);

			float r_3 = FAST_CBRT_POSITIVE(r);
			float phi_3 = phi / 3.0F;

			// Extract cubic roots.
			float u1 = 2.0F * r_3 * cosf(phi_3) + p;
			float u2 = 2.0F * r_3 * cosf(phi_3 + 2.0F * pi_over_three()) + p;
			float u3 = 2.0F * r_3 * cosf(phi_3 - 2.0F * pi_over_three()) + p;

			if (u1 >= 0.0F && u1 <= 1.0F)
				return u1;
			else if (u2 >= 0.0F && u2 <= 1.0F)
				return u2;
			else if (u3 >= 0.0F && u3 <= 1.0F)
				return u3;

			// Aiming at solving numerical imprecisions when u is outside [0,1].
			return (t < 0.5F) ? 0.0F : 1.0F;
		}
		else
		{
			float ss = sqrtf(s);
			float u = FAST_CBRT(q + ss) + FAST_CBRT(q - ss) + p;

			if (u >= 0.0F && u <= 1.0F)
				return u;

			// Aiming at solving numerical imprecisions when u is outside [0,1].
			return (t < 0.5F) ? 0.0F : 1.0F;
		}
	}
	if (fabsf(b) > 1e-3f)
	{
		float s = c * c - 4.0F * b * d;
		float ss = sqrtf(s);

		float u1 = (-c - ss) / (2.0F * b);
		float u2 = (-c + ss) / (2.0F * b);

		if (u1 >= 0.0F && u1 <= 1.0F)
			return u1;
		else if (u2 >= 0.0F && u2 <= 1.0F)
			return u2;

		// Aiming at solving numerical imprecisions when u is outside [0,1].
		return (t < 0.5F) ? 0.0F : 1.0F;
	}
	if (fabsf(c) > 1e-3f)
	{
		return (-d / c);
	}

	return 0.0F;
}

template<class T>
T BezierInterpolate(float t, T v1, T m1, T w1, T v2, T m2, T w2)
{
	float u = BezierExtractU(t, w1, 1.0F - w2);
	return BezierInterpolate(u, v1, w1 * m1 + v1, v2 - w2 * m2, v2);
}

template<>
Quaternionf BezierInterpolate(float t, Quaternionf v1, Quaternionf m1, Quaternionf w1, Quaternionf v2, Quaternionf m2, Quaternionf w2)
{
	Quaternionf ret;
	for (int i = 0; i < 4; ++i)
	{
		ret[i] = BezierInterpolate(t, v1[i], m1[i], w1[i], v2[i], m2[i], w2[i]);
	}

	return ret;
}

template<>
Vector3f BezierInterpolate(float t, Vector3f v1, Vector3f m1, Vector3f w1, Vector3f v2, Vector3f m2, Vector3f w2)
{
	Vector3f ret;
	for (int i = 0; i < 3; ++i)
	{
		ret[i] = BezierInterpolate(t, v1[i], m1[i], w1[i], v2[i], m2[i], w2[i]);
	}

	return ret;
}

template<class T>
T BezierInterpolate(float curveT, const FBXKeyFrameTpl<T>& lhs, const FBXKeyFrameTpl<T>& rhs)
{
	T lhsOutWeight = lhs.weightedMode & kOutWeighted ? lhs.outWeight : DefaultWeight<T>();
	T rhsInWeight = rhs.weightedMode & kInWeighted ? rhs.inWeight : DefaultWeight<T>();

	float dx = rhs.time - lhs.time;
	if (dx == 0.0F)
		return lhs.value;

	return BezierInterpolate((curveT - lhs.time) / dx, lhs.value, lhs.outSlope * dx, lhsOutWeight, rhs.value, rhs.inSlope * dx, rhsInWeight);
}

template<class T>
void FBXAnimationCurveTpl<T>::EvaluateWithoutCache(float curveT, T& output) const
{
	DebugAssert(IsValid());

	if (GetKeyCount() == 1)
	{
		output = m_Curve[0].value;
		return;
	}

	curveT = WrapTime(curveT);

	int lhsIndex, rhsIndex;
	FindIndexForSampling(m_Cache, curveT, lhsIndex, rhsIndex);
	const FBXKeyframe& lhs = m_Curve[lhsIndex];
	const FBXKeyframe& rhs = m_Curve[rhsIndex];

	output = InterpolateKeyframe(lhs, rhs, curveT);

	DebugAssert(IsFinite(output));
}

template<class T>
T FBXAnimationCurveTpl<T>::InterpolateKeyframe(const FBXKeyFrameTpl<T>& lhs, const FBXKeyFrameTpl<T>& rhs, float curveT)
{
	T output;

	if (lhs.weightedMode & kOutWeighted || rhs.weightedMode & kInWeighted)
		output = BezierInterpolate<T>(curveT, lhs, rhs);
	else
		output = HermiteInterpolate(curveT, lhs, rhs);

	HandleSteppedCurve(lhs, rhs, output);

	return output;
}

// Instanciate template for float, Vector3f, and Quaternionf.
template float FBXAnimationCurveTpl<float>::InterpolateKeyframe(const FBXKeyframe& lhs, const FBXKeyframe& rhs, float curveT);
template Vector3f FBXAnimationCurveTpl<Vector3f>::InterpolateKeyframe(const FBXKeyframe& lhs, const FBXKeyframe& rhs, float curveT);
template Quaternionf FBXAnimationCurveTpl<Quaternionf>::InterpolateKeyframe(const FBXKeyframe& lhs, const FBXKeyframe& rhs, float curveT);

