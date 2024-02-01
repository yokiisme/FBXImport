#include <chrono>
#include "AnimationKeyFrameReducer.h"
#include "AnimationCurveUtility.h"


#define DEBUG_COMPRESSION 0

const float kPositionMinValue = 0.00001F;
const float kQuaternionNormalizationError = 0.001F;/// The sampled quaternion must be almost normalized.


/// - We allow reduction if the reduced magnitude doesn't go off very far
/// - And the angle between the two rotations is similar
static inline bool QuaternionDistanceError(Quaternionf value, Quaternionf reduced, const float quaternionDotError)
{
    float magnitude = Magnitude(reduced);
    if (!CompareApproximately(1.0F, magnitude, kQuaternionNormalizationError))
    {
        return false;
    }

    value = NormalizeSafe(value);
    reduced = reduced / magnitude;

    if (Dot(value, reduced) < quaternionDotError)
    {
        return false;
    }

    return true;
}

static inline bool EulerDistanceError(Vector3f value, Vector3f reduced, const float eulerDotError)
{
    Quaternionf quatValue = EulerToQuaternion(value);
    Quaternionf quatReduced = EulerToQuaternion(reduced);

    return QuaternionDistanceError(quatValue, quatReduced, eulerDotError);
}

static bool DeltaError(const float value, const float reducedValue, const float delta, const float percentage, const float minValue)
{
    const float absValue = Abs(value);
    // (absValue > minValue || Abs(reducedValue) > minValue) part is necessary for reducing values which have tiny fluctuations around 0
    return (absValue > minValue || Abs(reducedValue) > minValue) && (delta > absValue * percentage);
}

/// We allow reduction
// - the distance of the two vectors is low
// - the distance of each axis is low
static inline bool PositionDistanceError(Vector3f value, Vector3f reduced, const float distancePercentageError)
{
    const float percentage = distancePercentageError;
    const float minValue = kPositionMinValue * percentage;

    // Vector3 distance as a percentage
    float distance = SqrMagnitude(value - reduced);
    float length = SqrMagnitude(value);
    float lengthReduced = SqrMagnitude(reduced);
    //if (distance > length * Sqr(percentage))
    if (DeltaError(length, lengthReduced, distance, Sqr(percentage), Sqr(minValue)))
        return false;

    // Distance of each axis
    float distanceX = Abs(value.x - reduced.x);
    float distanceY = Abs(value.y - reduced.y);
    float distanceZ = Abs(value.z - reduced.z);

    //if (distanceX > Abs(value.x) * percentage)
    if (DeltaError(value.x, reduced.x, distanceX, percentage, minValue))
        return false;
    //if (distanceY > Abs(value.y) * percentage)
    if (DeltaError(value.y, reduced.y, distanceY, percentage, minValue))
        return false;
    //if (distanceZ > Abs(value.z) * percentage)
    if (DeltaError(value.z, reduced.z, distanceZ, percentage, minValue))
        return false;

    return true;
}

/// We allow reduction if the distance between the two values is low
static bool FloatDistanceError(float value, float reduced, const float distancePercentageError)
{
    const float percentage = distancePercentageError;
    const float minValue = kPositionMinValue * percentage;

    float distance = Abs(value - reduced);
    //if (distance > Abs(value) * percentage)
    if (DeltaError(value, reduced, distance, percentage, minValue))
        return false;

    return true;
}

template<class T, class U, typename ReductionFunction>
static float ReduceKeyframes(const float sampleRate, T& curves, ReductionFunction reductionFunction, const float allowedError, const U zeroValue)
{
    float totalRatio = 0;
    for (typename T::iterator it = curves.begin(), end = curves.end(); it != end; ++it)
    {
        float compressionRatio = ReduceKeyframes(it->curve, sampleRate, reductionFunction, allowedError, zeroValue);
#if DEBUG_COMPRESSION
        printf_console("Compression %f%% of rotation %s\n", compressionRatio, it->path.c_str());
#endif

        totalRatio += compressionRatio;
    }

    return totalRatio;
}

void ReduceKeyframes(FBXImportAnimationClip& clip, float sampleRate ,float rotationError, float positionError, float scaleError, float floatError)
{
    
    FBXImportNodeAnimations& nodeAnim = clip.nodeAnimations;
    FBXImportFloatAnimations& floatAnim = clip.floatAnimations;
    rotationError = cos(Deg2Rad(rotationError) / 2.0F);
    positionError = positionError / 100.0F;
    scaleError = scaleError / 100.0F;
    floatError = floatError / 100.0F;

    auto start_time = std::chrono::high_resolution_clock::now();
   
    float averageCompressionRatio = 0;
    //int totalFrame = 0;
    auto& rot = nodeAnim.begin()->rotation;
    for (auto it = nodeAnim.begin(); it != nodeAnim.end(); it++)
    {
        auto& rot = it->rotation;
        auto& scale = it->scale;
        auto& pos = it->translation;
        
        averageCompressionRatio += ReduceKeyframes(rot[0], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(rot[1], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(rot[2], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(rot[3], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(scale[0], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(scale[1], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(scale[2], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(pos[0], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(pos[1], sampleRate, FloatDistanceError, floatError, 0.f);
        averageCompressionRatio += ReduceKeyframes(pos[2], sampleRate, FloatDistanceError, floatError, 0.f);
    }
     
    // If all the curves are empty, we end up with zero reduction.
    if (averageCompressionRatio == 0)
        return;
    auto end_time = std::chrono::high_resolution_clock::now();
    //auto duration = end_time - start_time;
    //auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    averageCompressionRatio /= 10 * nodeAnim.size();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    {
        //core::StringBuilder sb(kMemTempAlloc);
        //std::cout << "Keyframe reduction: Ratio: " << averageCompressionRatio << "%; Time: " << duration << "s;\n";
        //printf_console("%s", sb.ToString().c_str());
    }
}

// TODO : use tangents from editor or perform curve fitting
// TODO : these should be removed eventually and custom settings should be used instead
const float kQuaternionAngleError = 0.5F;// The maximum angle deviation allowed in degrees
const float kQuaternionDotError = cos(Deg2Rad(kQuaternionAngleError) / 2.0F);

///@TODO: * Support step curves
///       * Improve keyframe reduction
///       * Make keyframe reduction faster
void EulerToQuaternionCurveBake(const FBXAnimationCurve& curveX, const FBXAnimationCurve& curveY, const FBXAnimationCurve& curveZ, FBXAnimationCurveQuat& collapsed, float sampleRate)
{
    float begin = std::numeric_limits<float>::infinity();
    float end = -std::numeric_limits<float>::infinity();

    float delta = 1.0F / sampleRate;

    const FBXAnimationCurve* curves[3] = { &curveX, &curveY, &curveZ };

    for (int i = 0; i < 3; i++)
    {
        if (curves[i]->GetKeyCount() >= 1)
        {
            begin = std::min<float>(curves[i]->GetKey(0).time, begin);
            end = std::min<float>(curves[i]->GetKey(curves[i]->GetKeyCount() - 1).time, end);
        }
    }
    if (!IsFinite(begin) || !IsFinite(end))
        return;

    // changed Deg2Rad(1.0F) for math::radians(1.0F) since it was not giving same value on win32 than the other platforms
    // @TODO: review impl and use of Deg2Rad(). We should use constants defined by math:: in general
    float deg2rad = radians(1.0F); // Deg2Rad(1.0F);
    for (float i = begin; i < end + delta; i += delta)
    {
        if (i + delta / 2.0F > end)
            i = end;

        Vector3f euler = Vector3f(curveX.Evaluate(i), curveY.Evaluate(i), curveZ.Evaluate(i)) * deg2rad;
        Quaternionf q = EulerToQuaternion(euler);
        //Vector3f eulerBack = QuaternionToEuler (q) * Rad2Deg(1.0F);
        //printf_console("%f : %f, %f, %f ----- %f, %f, %f\n", i, euler.x, euler.y, euler.z, eulerBack.x, eulerBack.y, eulerBack.z);

        FBXKeyFrameTpl<Quaternionf> key;
        key.time = i;
        key.value = q;
        key.inSlope = key.outSlope = Quaternionf(0, 0, 0, 0);
        collapsed.AddKeyBackFast(key);

        if (i == end)
            break;
    }

    EnsureQuaternionContinuityAndRecalculateSlope(collapsed);

    //@TODO: Keyframe reduction is disabled for now, with keyframe reduction enabled iteration time
    // in the animation window becomes unbearable
    // ReduceKeyframes(collapsed, sampleRate, QuaternionDistanceError, kQuaternionDotError, Quaternionf(0, 0, 0, 0), false);
}


