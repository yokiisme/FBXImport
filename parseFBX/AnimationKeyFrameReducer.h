#pragma once
//#include "AnimationCurveUtility.h"
#include "FBXImporterDef.h"
#include "AnimationCurve.h"

void EulerToQuaternionCurveBake(const FBXAnimationCurve& curveX, const FBXAnimationCurve& curveY, const FBXAnimationCurve& curveZ, FBXAnimationCurveQuat& collapsed, float sampleRate);

template<class T, class ErrorFunction>
bool CanReduceAtTime(FBXAnimationCurveTpl<T>& curve, const FBXKeyFrameTpl<T>& key0, const FBXKeyFrameTpl<T>& key1, float time, ErrorFunction canReduceFunction, const float allowedError)
{
    T value = curve.Evaluate(time);
    T reduced_value = FBXAnimationCurveTpl<T>::InterpolateKeyframe(key0, key1, time);

    return canReduceFunction(value, reduced_value, allowedError);
}

template<class T, class ErrorFunction>
bool CanReduce(FBXAnimationCurveTpl<T>& curve, const FBXKeyFrameTpl<T>& fromKey, const FBXKeyFrameTpl<T>& toKey, ErrorFunction canReduceFunction, const float allowedError, const float delta,
    int firstKey, int lastKey, bool useKeyframeLimit)
{
    const float beginTime = fromKey.time;
    const float endTime = toKey.time;

    // We simply sample every frame and compare the original curve against, if we simply removed one key.
    // If the error between the curves is not too big, we just remove the key
    bool canReduce = true;

    const int stepCount = static_cast<int>((endTime - beginTime) / delta) + 1;
    for (int i = 0; i < stepCount; ++i)
    {
        const float time = beginTime + i * delta;
        if (!(canReduce = CanReduceAtTime(curve, fromKey, toKey, time, canReduceFunction, allowedError)))
            break;
    }

    // we need to check that all keys can be reduced, because keys might be closer to each other than delta
    // this happens when we have steps in curve
    // TODO : we could skip the loop above if keyframes are close enough

    float lastTime = beginTime;

    for (int j = firstKey; canReduce && j < lastKey; ++j)
    {
        const float time = curve.GetKey(j).time;

        // validates point at keyframe (j) and point between (j) and and (j-1) keyframes
        // TODO : For checking point at "time" it could just use keys[j].value instead - that would be faster than sampling the curve
        canReduce =
            CanReduceAtTime(curve, fromKey, toKey, time, canReduceFunction, allowedError) &&
            CanReduceAtTime(curve, fromKey, toKey, (lastTime + time) / 2, canReduceFunction, allowedError);

        lastTime = time;
    }

    if (canReduce)
    {
        // validate point between last two keyframes
        float time = curve.GetKey(lastKey).time;
        canReduce = CanReduceAtTime(curve, fromKey, toKey, (lastTime + time) / 2, canReduceFunction, allowedError);
    }

    // Don't reduce if we are about to reduce more than 50 samples at
    // once to prevent n^2 performance impact
    canReduce = canReduce && (!useKeyframeLimit || (endTime - beginTime < 50.0F * delta));

    return canReduce;
}

template<class T, class ErrorFunction>
float ReduceKeyframes(FBXAnimationCurveTpl<T>& curve, float sampleRate, ErrorFunction canReduceFunction, const float allowedError, T zeroValue)
{
    //AssertFormatMsg(curve.GetKeyCount() >= 2, "Key count: %d", curve.GetKeyCount());

    if (curve.GetKeyCount() <= 2)
        return 100.0F;
    std::vector<FBXKeyFrameTpl<T>> output;
    output.reserve(curve.GetKeyCount());

    float delta = 1.f / sampleRate;

    // at first try to reduce to const curve
    typename FBXAnimationCurveTpl<T>::FBXKeyframe firstKey = curve.GetKey(0);
    typename FBXAnimationCurveTpl<T>::FBXKeyframe lastKey = curve.GetKey(curve.GetKeyCount() - 1);
    firstKey.inSlope = firstKey.outSlope = zeroValue;
    lastKey.inSlope = lastKey.outSlope = zeroValue;
    lastKey.value = firstKey.value;

    float reduction = 100.0f;
    const bool canReduceToConstCurve = CanReduce(curve, firstKey, lastKey, canReduceFunction, allowedError, delta, 0, curve.GetKeyCount() - 1, false);
    if (canReduceToConstCurve)
    {
        output.reserve(2);
        output.push_back(firstKey);
        output.push_back(lastKey);
        reduction = (float)output.size() / (float)curve.GetKeyCount() * 100.0F;
    }
    else
    {
        output.reserve(curve.GetKeyCount());
        // We always add the first key
        output.push_back(curve.GetKey(0));

        int lastUsedKey = 0;

        for (int i = 1; i < curve.GetKeyCount() - 1; i++)
        {
            typename FBXAnimationCurveTpl<T>::FBXKeyframe fromKey = curve.GetKey(lastUsedKey);
            typename FBXAnimationCurveTpl<T>::FBXKeyframe toKey = curve.GetKey(i + 1);

            const bool canReduce = CanReduce(curve, fromKey, toKey, canReduceFunction, allowedError, delta, lastUsedKey + 1, i + 1, true);

            if (!canReduce)
            {
                output.push_back(curve.GetKey(i));
                lastUsedKey = i;
            }
        }

        // We always add the last key
        output.push_back(curve.GetKey(curve.GetKeyCount() - 1));
        reduction = (float)output.size() / (float)curve.GetKeyCount() * 100.0F;
    }
    curve.m_Curve.swap(output);
    //curve.Swap(output);

    return reduction;
}

// Rotation error is defined as maximum angle deviation allowed in degrees
// For others it is defined as maximum distance/delta deviation allowed in percents
void ReduceKeyframes(FBXImportAnimationClip& clip, float samplerate ,float rotationError, float positionError, float scaleError, float floatError);

