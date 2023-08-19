#include "Utility.h"
#include "AnimationImporter.h"
#include "floatConversion.h"
#include "AnimationCurveUtility.h"
#include "Components.h"
#include "Constraints.h"
#include <sstream>
#include <format>
#include <set>

namespace
{
	float EvaluateCurve(FbxAnimCurve* curve, FbxTime fbxtime, float defaultValue, bool remap, int coordIndex, int* lastCurveIndex, CurveFlags& flags)
	{
		float value = defaultValue;
		if (curve)
		{
			value = curve->Evaluate(fbxtime, lastCurveIndex);
			if (IsNAN(value))
			{
				value = defaultValue;
				flags |= kHasNaNs;
			}
		}

		if (remap)
			value = FBXCoordinateRemap(value, coordIndex);

		if (flags & kConvertEuler)
		{
			value = FBXEulerRemap(value, coordIndex);
		}
		return value;
	}

	float EvaluateCurve(FbxAnimCurve* curve, float time, float defaultValue, bool remap, int coordIndex, int* lastCurveIndex, CurveFlags& flags)
	{
		FbxTime fbxtime;
		fbxtime.SetSecondDouble(time);

		return EvaluateCurve(curve, fbxtime, defaultValue, remap, coordIndex, lastCurveIndex, flags);
	}

	struct KeyInfo
	{
		KeyInfo() : mode((FbxAnimCurveDef::EInterpolationType)-1), stepHelperKey(false) {}
		KeyInfo(FbxAnimCurveDef::EInterpolationType m, FbxAnimCurveDef::ETangentMode tM, float v, bool helperKey) : mode(m), tangentMode(tM), value(v), stepHelperKey(helperKey) {}

		FbxAnimCurveDef::EInterpolationType mode;
		FbxAnimCurveDef::ETangentMode tangentMode;
		float value;
		// use this to mark keys inserter as extra key for const interpolation
		bool stepHelperKey;
	};

	typedef std::vector<std::pair<float, KeyInfo>> KeyInfoVector;

	const float kStepKeyOffset = 1e-5f;
	const float kOverlappingKeyOffset = kStepKeyOffset;
	const float kOversamplingKeyOffset = 1e-4f;

	// curve should be const, but KFCurve is not implemented properly
	void GetKeyModes(FbxAnimCurve& curve, const float maxTime, KeyInfoVector& keyInfo)
	{
		keyInfo.clear();
		keyInfo.reserve(curve.KeyGetCount());

		float lastKeyTime = -1e10f;

		for (int i = 0, c = curve.KeyGetCount(); i < c; ++i)
		{
			FbxAnimCurveKey key = curve.KeyGet(i);
			const FbxAnimCurveDef::EInterpolationType mode = key.GetInterpolation();
			const FbxAnimCurveDef::ETangentMode tangentMode = key.GetTangentMode();
			const float value = key.GetValue();
			float time = static_cast<float>(key.GetTime().GetSecondDouble());

			// Separating overlapping keys
			if (time < lastKeyTime + kOverlappingKeyOffset)
				time = lastKeyTime + kOverlappingKeyOffset;

			// optimization - skip keys after max time
			// TODO : potential optimization: skip keys before minTime (it must keep at least one key before minTime)
			if (time > maxTime)
				break;

			keyInfo.push_back(KeyInfoVector::value_type(time, KeyInfo(mode, tangentMode, value, false)));
			lastKeyTime = time;

			if (FbxAnimCurveDef::eInterpolationConstant == mode && i < curve.KeyGetCount() - 1)
			{
				// we need to insert additional key, because there is no other way to do stepped curves

				FbxAnimCurveKey nextKey = curve.KeyGet(i + 1);
				const float nextValue = nextKey.GetValue();

				// we don't need extra key if values match
				if (nextValue != value)
				{
					if (FbxAnimCurveDef::eConstantNext == key.GetConstantMode())
					{
						// insert key right after this one, because const value is taken from next one

						// modify current key - it acts as a step helper key
						KeyInfo& thisKeyInfo = keyInfo.back().second;
						thisKeyInfo.stepHelperKey = true;
						thisKeyInfo.mode = FbxAnimCurveDef::eInterpolationLinear;

						const float time2 = time + kStepKeyOffset;
						keyInfo.push_back(KeyInfoVector::value_type(time2, KeyInfo(FbxAnimCurveDef::eInterpolationConstant, thisKeyInfo.tangentMode, nextValue, false)));
					}
					else
					{
						// insert helper key right before the next one
						const float nextKeyTime = static_cast<float>(nextKey.GetTime().GetSecondDouble());
						const float time2 = nextKeyTime - kStepKeyOffset - std::numeric_limits<float>::epsilon();
						keyInfo.push_back(KeyInfoVector::value_type(time2, KeyInfo(FbxAnimCurveDef::eInterpolationLinear, nextKey.GetTangentMode(), value, true)));
					}

					lastKeyTime = keyInfo.back().first;
				}
			}
		}
	}

	struct KeyInfoGroup
	{
		KeyInfo keys[3];

		bool HasStepHelperKey() const
		{
			return keys[0].stepHelperKey || keys[1].stepHelperKey || keys[1].stepHelperKey;
		}

		bool HasOnlyCubicInterpolation() const
		{
			return
				keys[0].mode == FbxAnimCurveDef::eInterpolationCubic &&
				keys[1].mode == FbxAnimCurveDef::eInterpolationCubic &&
				keys[2].mode == FbxAnimCurveDef::eInterpolationCubic;
		}
	};

	typedef std::map<float,KeyInfoGroup> KeyInfoMap;

	void AddKeyModes(const KeyInfoVector& keyModes, float minTime, float maxTime, int coordIndex, KeyInfoMap& timeMap, FbxAnimCurve* curve, float defaultValue, CurveFlags& flags)
	{
		int firstKeyIndex = -1;

		for (unsigned int i = 0; i < keyModes.size(); ++i)
		{
			const float time = keyModes[i].first;
			const KeyInfo& keyInfo = keyModes[i].second;

			// we need key right before or on minTime
			if (time <= minTime)
				firstKeyIndex = i;

			// skip keys which are out of range
			if (time > minTime && time <= maxTime)
			{
				KeyInfoMap::iterator it = timeMap.find(time);
				if (it == timeMap.end())
				{
					it = timeMap.insert(KeyInfoMap::value_type(time, KeyInfoGroup())).first;
				}

				it->second.keys[coordIndex] = keyInfo;
			}
		}

		if (firstKeyIndex >= 0)
		{
			KeyInfoGroup& firstKeyInfo = timeMap.begin()->second;

			firstKeyInfo.keys[coordIndex] = keyModes[firstKeyIndex].second;

			if (keyModes[firstKeyIndex].first != minTime)
			{
				// doesn't remap value - remapping will be done later
				firstKeyInfo.keys[coordIndex].value = EvaluateCurve(curve, minTime, defaultValue, false, coordIndex, NULL, flags);
			}
		}
	}

	void AddKeyModesInfo(float time, KeyInfoMap& timeMap)
	{
		std::pair<KeyInfoMap::iterator, bool> result = timeMap.insert(KeyInfoMap::value_type(time, KeyInfoGroup()));
		if (result.second)
		{
			const KeyInfoMap::iterator it = result.first;

			bool remove = false;

			if (it != timeMap.begin())
			{
				KeyInfoMap::iterator prev = it;
				--prev;

				if (time - prev->first <= kOversamplingKeyOffset)
					remove = true;
			}

			if (!remove)
			{
				KeyInfoMap::iterator next = it;
				++next;
				if (next != timeMap.end())
				{
					if (next->first - time <= kOversamplingKeyOffset)
						remove = true;
				}
			}

			if (remove)
				timeMap.erase(it);
		}
	}

	void GroupKeyModes(FbxAnimCurve** curve, float minTime, float maxTime, KeyInfoMap& timeMap, FbxVector4 defaultValue, CurveFlags& flags)
	{
		timeMap.clear();
		AddKeyModesInfo(minTime, timeMap);
		AddKeyModesInfo(maxTime, timeMap);

		KeyInfoVector keyModes;

		for (int i = 0; i < 3; ++i)
		{
			if (curve[i])
			{
				GetKeyModes(*curve[i], maxTime, keyModes);
				AddKeyModes(keyModes, minTime, maxTime, i, timeMap, curve[i], static_cast<float>(defaultValue[i]), flags);
			}
		}
	}

	void GenerateKeyModes(FbxAnimCurve** curve, float minTime, float maxTime, float sampleRate, int oversampling, KeyInfoMap& timeMap, FbxVector4 defaultValue, CurveFlags& flags)
	{
		// insert key modes from keyframes
		GroupKeyModes(curve, minTime, maxTime, timeMap, defaultValue, flags);

		// insert key modes from (over)sampling
		if (oversampling > 0)
		{
			//const float delta = sampleRate / oversampling;
			//for (float t = minTime; t < maxTime; t += delta)
			//  AddKeyModesInfo(t, timeMap);

			// TODO : switch to the code above as it much more simple
			// we are using this one, just to minimize differences between old and new code
			int firstFrame = RoundfToInt(minTime / sampleRate * oversampling);
			int lastFrame = RoundfToInt(maxTime / sampleRate * oversampling);
			lastFrame = max(firstFrame + 1, lastFrame);

			for (int f = firstFrame; f <= lastFrame; f++)
			{
				float t = f * sampleRate / oversampling;
				AddKeyModesInfo(t, timeMap);
			}
		}
	}

	void EnsureQuaternionContinuity(const Quaternionf& last, Quaternionf& current)
	{
		if (Dot(current, last) < 0.0F)
			current.Set(-current.x, -current.y, -current.z, -current.w);
	}

	template<class T>
	void AddValues(FBXAnimationCurve* outcurves, float time, const T& values, int coordinateCount, const float timeRangeOffset)
	{
		for (int j = 0; j < coordinateCount; ++j)
		{
            FBXAnimationCurve::FBXKeyframe key;
			key.time = time + timeRangeOffset;
			key.value = values[j];
			key.inSlope = key.outSlope = 0;
			key.inWeight = key.outWeight = DefaultWeight<float>();
			key.weightedMode = kNotWeighted;
			outcurves[j].AddKeyBackFast(key);
		}
	}

	// we need two points on the curve for fitting
	const float FitTime1 = 0.3f;
	const float FitTime2 = 1 - FitTime1;

	// Fits tangents (keys(keyIndex).outSlope and keys(keyIndex).inSlope) of outcurve to curve "curve"
	void FitCurve(FBXAnimationCurve& outcurve, float value1, float value2, int keyIndex)
	{
		FBXAnimationCurve::FBXKeyframe& key0 = outcurve.GetKey(keyIndex);
		FBXAnimationCurve::FBXKeyframe& key1 = outcurve.GetKey(keyIndex + 1);

		FitTangents(key0, key1, FitTime1, FitTime2, value1, value2);
	}

	template<class T>
	void AddKeyValues(FBXAnimationCurve* outcurves, const KeyInfoMap& timeMap, const std::vector<T>& values, const std::vector<T>& fitValues, int coordinateCount, const float timeRangeOffset)
	{
		int keyIndex = 0;
		for (KeyInfoMap::const_iterator it = timeMap.begin(), end = timeMap.end(); it != end; ++it, ++keyIndex)
		{
			const float time = it->first;

			AddValues(outcurves, time, values[keyIndex], coordinateCount, timeRangeOffset);
		}

		// perform curve fitting
		for (int j = 0; j < coordinateCount; ++j)
		{
			KeyInfoMap::const_iterator end = timeMap.end();
			if (end != timeMap.begin())
				--end;

			int keyIndex = 0;
			for (KeyInfoMap::const_iterator it = timeMap.begin(); it != end; ++it, ++keyIndex)
			{
				const KeyInfoGroup& keyInfo = it->second;
				//if there are 4 coordinates, it means we're converting eulers to Quaternions. The interpolation mode of w is irrelevant as it should all be Cubic anyway
				const size_t modeIndex = j > 2 ? 2 : j;

				const FbxAnimCurveDef::EInterpolationType mode = keyInfo.keys[modeIndex].mode;

				if (FbxAnimCurveDef::eInterpolationConstant == mode)
				{
					// we don't need to perform fitting because slopes are assigned to 0 already
				}
				else if (FbxAnimCurveDef::eInterpolationLinear == mode)
				{
					// linear curve
					RecalculateSplineSlopeLinear(outcurves[j], keyIndex);
				}
				else
				{
					// cubic curve - fit it
					FitCurve(outcurves[j], fitValues[keyIndex*2][j], fitValues[keyIndex*2+1][j], keyIndex);
				}
			}

			if (outcurves[j].GetKeyCount() > 0)
			{
				// make first key have matching in/out slopes
				outcurves[j].GetKey(0).inSlope = outcurves[j].GetKey(0).outSlope;

				// make last key have matching in/out slopes
				const int last = outcurves[j].GetKeyCount() - 1;
				outcurves[j].GetKey(last).outSlope = outcurves[j].GetKey(last).inSlope;
			}
		}
	}

	const float kMaxTimeRange = 100000;

	bool IsValidTimeRange(const char* const takeName, const char* const nodeName, const char* const curveName, const float minimum, const float maximum)
	{
		const float timeRange = maximum - minimum;
		if (timeRange > kMaxTimeRange)
		{
			std::ostringstream str;
			str << "Time range (" << static_cast<int>(timeRange) << ") for " << nodeName << " curve(s) on node '" << takeName << "' on take '" << static_cast<int>(kMaxTimeRange) << "' larger than maximum allowed (" << takeName << "). These curves won't be imported. "
				"Check your file - it most likely has keys in far negative or positive timeline.\n";
			ReportError(str.str());
			return false;
		}
		else if (timeRange < 0)
		{
			std::ostringstream str;
			str << "Invalid time range (" << static_cast<int>(timeRange) << ") for " << curveName << " curve on node '" << nodeName << "' on take '" << takeName << "'. These curves won't be imported. Check your file - it most likely has invalid curves.";
			ReportError(str.str());
			return false;
		}
		else
			return true;
	}
}



static void MergeVisibilityAnimCurve(
	FbxAnimCurve* reference,
	FbxAnimCurve* target)
{
	if (reference != NULL)
	{
		int i = 0, j = 0;
		int parentLastIndex = 0;
		int childLastIndex = 0;
		std::vector<float> keyvalues(target->KeyGetCount() + reference->KeyGetCount());
		// we first need to add as many key as necessary to the child curve and calculate the final value.
		while (i < target->KeyGetCount() && j < reference->KeyGetCount())
		{
			FbxTime targetTime = target->KeyGetTime(i);
			FbxTime referenceTime = reference->KeyGetTime(j);
			// we are flattening curves to avoid negative visibilities due to tangents.
			target->KeySetTangentMode(i, FbxAnimCurveDef::eTangentBreak);
			target->KeySetInterpolation(i, FbxAnimCurveDef::eInterpolationConstant);
			target->KeySetConstantMode(i, FbxAnimCurveDef::eConstantStandard);
			if (targetTime > referenceTime)
			{
				target->KeyInsert(referenceTime, &childLastIndex);
				target->KeySetValue(i, target->KeyGetValue(i - 1));
				target->KeySetTangentMode(i, FbxAnimCurveDef::eTangentBreak);
				target->KeySetInterpolation(i, FbxAnimCurveDef::eInterpolationConstant);
				target->KeySetConstantMode(i, FbxAnimCurveDef::eConstantStandard);
			}
			keyvalues[i] = target->KeyGetValue(i) * reference->Evaluate(targetTime, &parentLastIndex);
			++i;
			if (targetTime >= referenceTime)
				++j;
		}
		// add parent key at the end if there is still some to add.
		float lastValue = target->KeyGetValue(i - 1); // this is the last value of the curve
		while (j < reference->KeyGetCount())
		{
			int newKey = target->KeyAdd(reference->KeyGetTime(j), &childLastIndex);
			target->KeySetTangentMode(newKey, FbxAnimCurveDef::eTangentBreak);
			target->KeySetInterpolation(newKey, FbxAnimCurveDef::eInterpolationConstant);
			target->KeySetConstantMode(newKey, FbxAnimCurveDef::eConstantStandard);
			keyvalues[i] = reference->KeyGetValue(j) * lastValue;
			++i;
			++j;
		}
		// Set the final values in a separate loop because of KeyInsert
		// that interpolates values based on previous and next key
		// and would give wrong results if we already changed the values.
		for (int k = 0; k < i; ++k)
		{
			target->KeySetValue(k, keyvalues[k]);
		}
	}
	else
	{
		for (int i = 0; i < target->KeyGetCount(); ++i)
		{
			// we are flattening curves to avoid negative visibilities due to tangents.
			target->KeySetTangentMode(i, FbxAnimCurveDef::eTangentBreak);
			target->KeySetInterpolation(i, FbxAnimCurveDef::eInterpolationConstant);
			target->KeySetConstantMode(i, FbxAnimCurveDef::eConstantStandard);
		}
	}
}

static const Quaternionf FBXCameraRotationOffset = EulerToQuaternion(Vector3f(.0f, -90.0f, .0f) * kDeg2Rad, kOrderUnityDefault);
static const Quaternionf FBXLightRotationOffset = EulerToQuaternion(Vector3f(90.0f, .0f, .0f) * kDeg2Rad, kOrderUnityDefault);

bool EvaluateForEulerToQuat(FbxNode* node)
{
	const bool postRotation = node->GetPostRotation(FbxNode::eSourcePivot) == node->GetPostRotation(FbxNode::eDestinationPivot);
	const bool preRotation = node->GetPreRotation(FbxNode::eSourcePivot) == node->GetPreRotation(FbxNode::eDestinationPivot);
	const bool rotationOffset = node->GetRotationOffset(FbxNode::eSourcePivot) == node->GetRotationOffset(FbxNode::eDestinationPivot);
	return postRotation && preRotation && rotationOffset;
}

void SetupTimeRange(FbxScene& fbxScene, FbxAnimStack& animStack, const AnimationSettings& settings, AnimationTimeRange& output)
{
    {
        FbxTimeSpan bakeTimeSpan = animStack.GetLocalTimeSpan();
        // This is a special workaround for a change in FBXSDK 2012.2
        // For some COLLADA files animStack->GetLocalTimeSpan() is [0 0]
        // So we get animation range from keyframes themselves using GetAnimationInterval
        if (bakeTimeSpan.GetDuration().GetSecondDouble() < std::numeric_limits<float>::epsilon())
        {
            FbxTimeSpan tempbakeTimeSpan;
            const int animLayerCount = animStack.GetSrcObjectCount<FbxAnimLayer>();
            bool isRangeValid = false;
            for (int i = 0; i < animLayerCount; ++i)
                isRangeValid = fbxScene.GetRootNode()->GetAnimationInterval(tempbakeTimeSpan, &animStack, i);
            if (isRangeValid)
            {
                bakeTimeSpan = tempbakeTimeSpan;
            }
        }

        output.bakeStart = bakeTimeSpan.GetStart().GetSecondDouble();
        output.bakeStop = bakeTimeSpan.GetStop().GetSecondDouble();
    }

    if (settings.adjustClipsByTimeRange)
    {
        output.timeRangeStart = static_cast<float>(output.bakeStart);
        output.timeRangeEnd = static_cast<float>(output.bakeStop);
        output.timeRangeOffset = -output.timeRangeStart;
    }
    else
    {
        output.timeRangeStart = -std::numeric_limits<float>::infinity();
        output.timeRangeEnd = std::numeric_limits<float>::infinity();
        output.timeRangeOffset = 0;
    }
}

bool HasAnyCurveValues(const FbxAnimCurve* curve)
{
    return curve && curve->KeyGetCount() != 0;
}

bool HasAnyCurveValues(FbxAnimCurve** curves)
{
    for (int i = 0; i < 3; i++)
    {
        if (HasAnyCurveValues(curves[i]))
            return true;
    }
    return false;
}

CurveFlags FBXRotationOrderToCurveFlags(EFbxRotationOrder order)
{
    CurveFlags retValue = (CurveFlags)(1 << (2 + order));
    return retValue;
}


RotationOrder FlagsToRotationImportOrder(CurveFlags flags)
{
    int rotationFlags = (flags & kRotationFlags);

    for (int i = 0; i < kRotationOrderCount; ++i)
    {
        if (rotationFlags & (1 << (2 + i)))
            return (RotationOrder)i;
    }
    return kOrderUnityDefault;
}

EFbxRotationOrder FlagsToFBXRotationOrder(CurveFlags flags)
{
    return (EFbxRotationOrder)FlagsToRotationImportOrder(flags);
}


void EvaluateTimeRange(FbxAnimCurve** curve, const AnimationTimeRange& range, float& minimum, float& maximum)
{
    minimum = std::numeric_limits<float>::infinity();
    maximum = -std::numeric_limits<float>::infinity();

    for (int i = 0; i < 3; ++i)
    {
        if (curve[i])
        {
            if (curve[i]->KeyGetCount())
            {
                float curMin = static_cast<float>(curve[i]->KeyGet(0).GetTime().GetSecondDouble());
                float curMax = static_cast<float>(curve[i]->KeyGet(curve[i]->KeyGetCount() - 1).GetTime().GetSecondDouble());
                if (curMin > range.timeRangeEnd || curMax < range.timeRangeStart)
                    continue;

                minimum = std::min<float>(curMin, minimum);
                maximum = std::max<float>(curMax, maximum);
            }
        }
    }

    minimum = std::max<float>(minimum, range.timeRangeStart);
    maximum = std::min<float>(maximum, range.timeRangeEnd);
}

void GetCurveTimeSpans(std::vector<FbxTimeSpan>& curveTimeSpans, FbxAnimCurve** curve, float defaultStartTime, float defaultEndTime)
{
    FbxTime defaultFbxStartTime, defaultFbxEndTime;

    defaultFbxStartTime.SetSecondDouble(defaultStartTime);
    defaultFbxEndTime.SetSecondDouble(defaultEndTime);

    for (int i = 0; i < 3; i++)
    {
        if (curve[i])
        {
            if (!curve[i]->GetTimeInterval(curveTimeSpans[i]))
            {
                // can't retrieve curve timespan
                // use the default start/end
                curveTimeSpans[i].SetStart(defaultFbxStartTime);
                curveTimeSpans[i].SetStop(defaultFbxEndTime);
            }
        }
    }
}

void ClipFbxTimeAgainstTimeSpan(FbxTime& fbxtime, const FbxTimeSpan& timeSpan)
{
    if (!timeSpan.IsInside(fbxtime))
    {
        if (fbxtime < timeSpan.GetStart())
            fbxtime = timeSpan.GetStart();
        else if (fbxtime > timeSpan.GetStop())
            fbxtime = timeSpan.GetStop();
    }
}


void ConvertCurveOversampling(
    // for error reporting
    const char* const takeName, const char* const nodeName, const char* const curveName,
    FbxAnimCurve** curve, FBXAnimationCurve* outcurves, const AnimationSettings& animationSettings, FbxVector4 defaultValue, bool remap,
    RotationCurveImportOptions rotationOption, const AnimationTimeRange& range, CurveFlags& flags, FbxNode* node)
{
    //Assert(animationSettings.animationOversampling);

    float minimum, maximum;

    EvaluateTimeRange(curve, range, minimum, maximum);

    if (minimum == std::numeric_limits<float>::infinity())
        return;

    if (!IsValidTimeRange(takeName, nodeName, curveName, minimum, maximum))
    {
        // Error: time range is too big
        return;
    }
    float sampleRate = static_cast<float>(animationSettings.sampleRate);
    int firstFrame = RoundfToInt(minimum / sampleRate);
    int lastFrame = RoundfToInt(maximum / sampleRate);
    lastFrame = max(firstFrame + 1, lastFrame);   // generate at least two keys at a minimum

    // used for optimized sampling
    // TODO : use static array
    std::vector<int> lastCurveIndices(3, 0);

    // get start/end times per curve
    std::vector<FbxTimeSpan> curveTimeSpans(3);
    GetCurveTimeSpans(curveTimeSpans, curve, firstFrame * sampleRate, lastFrame * sampleRate);

    for (int f = firstFrame; f <= lastFrame; f++)
    {
        float time = f * sampleRate;
        FbxVector4 temp;

        for (int i = 0; i < 3; i++)
        {
            if (curve[i])
            {
                FbxTime fbxtime;
                fbxtime.SetSecondDouble(time);
                // ensure fbxtime is within the curve's timespan, otherwise FBX will crash
                ClipFbxTimeAgainstTimeSpan(fbxtime, curveTimeSpans[i]);

                temp[i] = curve[i]->Evaluate(fbxtime, &lastCurveIndices[i]);
                if (IsNAN(temp[i]))
                {
                    temp[i] = defaultValue[i];
                    flags |= kHasNaNs;
                }
            }
            else
                temp[i] = defaultValue[i];
        }

        if (rotationOption == RotationCurveImportOptions::kImportQuaternion)
        {
            Quaternionf values = ExtractQuaternionFromFBXEulerOld(temp);

            if (animationSettings.importCameras && node->GetCamera())
            {
                values *= FBXCameraRotationOffset;
            }

            if (animationSettings.importLights && node->GetLight())
            {
                values *= FBXLightRotationOffset;
            }

            AddValues(outcurves, time, values, 4, range.timeRangeOffset);
        }
        else if (rotationOption == RotationCurveImportOptions::kImportEulerRadians || rotationOption == RotationCurveImportOptions::kImportEulerDegrees)
        {
            FbxVector4 fbxEuler = rotationOption == RotationCurveImportOptions::kImportEulerRadians ? temp * (float)FBXSDK_RAD_TO_DEG : temp;

            Quaternionf quat = ExtractQuaternionFromFBXEuler(fbxEuler, FlagsToFBXRotationOrder(flags));

            Vector3f eulerAngles = QuaternionToEuler(quat) * kRad2Deg;

            AddValues(outcurves, time, eulerAngles, 3, range.timeRangeOffset);
        }
        else
        {
            Vector3f values = remap ? FBXPointToVector3Remap(temp) : FBXPointToVector3(temp);
            AddValues(outcurves, time, values, 3, range.timeRangeOffset);
        }
    }

    if (rotationOption == RotationCurveImportOptions::kImportQuaternion)
    {
        FBXAnimationCurve* temp[4] = { outcurves + 0, outcurves + 1, outcurves + 2, outcurves + 3 };
        EnsureQuaternionContinuity(temp);
    }
    else
    {
        // Check for step curves and adjust tangents
        for (int j = 0; j < 3; j++)
        {
            RecalculateSplineSlope(outcurves[j]);

            for (int i = 0; i < outcurves[j].GetKeyCount() - 1; i++)
            {
                if (curve[j])
                {
                    FbxTime fbxtime;
                    fbxtime.SetSecondDouble(outcurves[j].GetKey(i).time);
                    double dKeyIndex = curve[j]->KeyFind(fbxtime);
                    int keyIndex = static_cast<int>(dKeyIndex);
                    if (keyIndex > -1 && (curve[j]->KeyGetTangentMode(keyIndex) & FbxAnimCurveDef::eTangentGenericBreak))
                    {
                        // TODO : this doesn't work with oversampling
                        RecalculateSplineSlopeLinear(outcurves[j], i);
                    }
                }
            }
        }
    }
}





void ConvertCurve(
    // for error reporting
    const char* const takeName, const char* const nodeName, const char* const curveName,
    FbxAnimCurve** curve, FBXAnimationCurve* outcurves, const AnimationSettings& animationSettings, FbxVector4 defaultValue, bool remap, RotationCurveImportOptions rotationOption,
    const AnimationTimeRange& range, CurveFlags& flags, FbxNode* node)
{
    {
        // validating default value
        for (int i = 0; i < 3; i++)
        {
            if (IsNAN(defaultValue[i]))
            {
                defaultValue[i] = 0;
                flags |= kHasNaNs;
            }
        }
    }

    bool rotation = rotationOption != RotationCurveImportOptions::kNoRotation;

    // HACK : we know that !remap == scale, we will remove this once we have an answer from FBX support
    if ((rotation || !remap) && animationSettings.animationOversampling)
    {
        ConvertCurveOversampling(takeName, nodeName, curveName, curve, outcurves, animationSettings, defaultValue, remap, rotationOption, range, flags, node);
        return;
    }

    float minimum;
    float maximum;

    EvaluateTimeRange(curve, range, minimum, maximum);

    if (minimum == std::numeric_limits<float>::infinity())
        return;

    if (!IsValidTimeRange(takeName, nodeName, curveName, minimum, maximum))
    {
        // Error: time range is too big
        return;
    }

    float sampleRate = static_cast<float>(animationSettings.sampleRate);

    // generate at least two keys at a minimum
    if (minimum >= maximum)
        maximum = minimum + sampleRate;

    // get start/end times per curve
    std::vector<FbxTimeSpan> curveTimeSpans(3);
    GetCurveTimeSpans(curveTimeSpans, curve, minimum, maximum);

    KeyInfoMap timeMap;
    GenerateKeyModes(curve, minimum, maximum, sampleRate, animationSettings.animationOversampling, timeMap, defaultValue, flags);

    int tangentModesMask = 0; //to get the list of all tangent modes in the curves.
    for (KeyInfoMap::iterator it = timeMap.begin(), end = timeMap.end(); it != end; ++it)
        for (int i = 0; i < 3; ++i)
        {
            if (it->second.keys[i].tangentMode > 0)
            {
                tangentModesMask |= it->second.keys[i].tangentMode;
            }
        }

    bool euler = false;
    if (rotation && EvaluateForEulerToQuat(node) && !(animationSettings.importCameras && node->GetCamera()) && !(animationSettings.importLights && node->GetLight()))
    {
        euler = true;
        flags |= kConvertEuler;
    }

    // TODO : should carry over stepHelper flag too - just in case some sampling key falls in between?
    FbxAnimCurveDef::EInterpolationType lastModes[3];
    lastModes[0] = lastModes[1] = lastModes[2] = FbxAnimCurveDef::eInterpolationCubic;

    std::vector<Vector3f> fitValues(timeMap.size() * 2 - 2);

    // used for optimized sampling
    // TODO : use static array
    std::vector<int> lastCurveIndices(3, 0);

    int keyIndex = 0;
    for (KeyInfoMap::iterator it = timeMap.begin(), end = timeMap.end(); it != end; ++it, ++keyIndex)
    {
        const float time = it->first;

        KeyInfoGroup& keyInfo = it->second;

        bool last = false;
        float sampleTime1 = 0, sampleTime2 = 0;

        {
            float nextTime = maximum;
            KeyInfoMap::const_iterator nextIt = it;
            ++nextIt;

            if (nextIt != end)
            {
                nextTime = nextIt->first;

                const float dt = nextTime - time;

                sampleTime1 = time + dt * FitTime1;
                sampleTime2 = time + dt * FitTime2;
            }
            else
                last = true;
        }

        for (int i = 0; i < 3; ++i)
        {
            FbxTime fbxtime;
            fbxtime.SetSecondDouble(time);
            // ensure fbxtime is within the curve's timespan, otherwise FBX will crash
            ClipFbxTimeAgainstTimeSpan(fbxtime, curveTimeSpans[i]);

            FbxAnimCurveDef::EInterpolationType mode = keyInfo.keys[i].mode;
            // there is no linear interpolation for rotation curve, because it's in euler angles and we operate in quaternions,
            // so we need to perform full curve fitting
            if (rotation && FbxAnimCurveDef::eInterpolationLinear == mode && (flags & kConvertEuler) == 0)
                keyInfo.keys[i].mode = mode = FbxAnimCurveDef::eInterpolationCubic;


            if ((FbxAnimCurveDef::EInterpolationType)-1 == mode)
            {
                keyInfo.keys[i].value = EvaluateCurve(curve[i], fbxtime, static_cast<float>(defaultValue[i]), remap, i, &lastCurveIndices[i], flags);
                keyInfo.keys[i].mode = mode = lastModes[i];
            }
            else
            {
                if (rotation && euler)
                {
                    float angle = FBXEulerRemap(keyInfo.keys[i].value, i);
                    keyInfo.keys[i].value = rotationOption == RotationCurveImportOptions::kImportEulerRadians ? angle * kRad2Deg : angle;
                }
                else
                {
                    if (remap)
                    {
                        keyInfo.keys[i].value = FBXCoordinateRemap(keyInfo.keys[i].value, i);
                    }
                }
                lastModes[i] = mode;
            }

            lastModes[i] = mode;

            if (!last)
            {
                FbxTime sampleFbxtime1, sampleFbxtime2;
                sampleFbxtime1.SetSecondDouble(sampleTime1);
                sampleFbxtime2.SetSecondDouble(sampleTime2);
                // ensure fbxtimes are within the curve's timespan, otherwise FBX will crash
                ClipFbxTimeAgainstTimeSpan(sampleFbxtime1, curveTimeSpans[i]);
                ClipFbxTimeAgainstTimeSpan(sampleFbxtime2, curveTimeSpans[i]);

                // we need to evaluate all values for rotation, because we need it for conversion to Quaternions
                if (FbxAnimCurveDef::eInterpolationCubic == mode)
                {
                    const float f0 = EvaluateCurve(curve[i], sampleFbxtime1, static_cast<float>(defaultValue[i]), remap, i, &lastCurveIndices[i], flags);
                    const float f1 = EvaluateCurve(curve[i], sampleFbxtime2, static_cast<float>(defaultValue[i]), remap, i, &lastCurveIndices[i], flags);
                    fitValues[keyIndex * 2 + 0][i] = rotationOption == RotationCurveImportOptions::kImportEulerRadians ? f0 * kRad2Deg : f0;
                    fitValues[keyIndex * 2 + 1][i] = rotationOption == RotationCurveImportOptions::kImportEulerRadians ? f1 * kRad2Deg : f1;
                }
                else if (rotation)
                {
                    fitValues[keyIndex * 2 + 0][i] = fitValues[keyIndex * 2 + 1][i] = keyInfo.keys[i].value;
                }
                else
                {
                    // rotation should have its' fitValues assigned
                    //Assert(!rotation);
                }
            }
        }
    }


    if (rotation && !euler)
    {
        std::vector<Quaternionf> quatValues(timeMap.size());
        std::vector<Quaternionf> quatFitValues(fitValues.size());

        Quaternionf last;

        int keyIndex = 0;
        for (KeyInfoMap::const_iterator it = timeMap.begin(), end = timeMap.end(); it != end; ++it, ++keyIndex)
        {
            const KeyInfoGroup& keyInfo = it->second;

            Vector3f value(keyInfo.keys[0].value, keyInfo.keys[1].value, keyInfo.keys[2].value);
            quatValues[keyIndex] = ExtractQuaternionFromFBXEuler(Vector3ToFBXPoint(value), FlagsToFBXRotationOrder(flags));

            if ((animationSettings.importCameras && node->GetCamera()))
            {
                quatValues[keyIndex] *= FBXCameraRotationOffset;
            }
            if ((animationSettings.importLights && node->GetLight()))
            {
                quatValues[keyIndex] *= FBXLightRotationOffset;
            }

            if (keyIndex > 0)
                EnsureQuaternionContinuity(last, quatValues[keyIndex]);

            if (keyIndex < static_cast<int>(timeMap.size()) - 1)
            {
                const int i1 = keyIndex * 2 + 0;
                const int i2 = keyIndex * 2 + 1;

                quatFitValues[i1] = ExtractQuaternionFromFBXEuler(Vector3ToFBXPoint(fitValues[i1]), FlagsToFBXRotationOrder(flags));
                quatFitValues[i2] = ExtractQuaternionFromFBXEuler(Vector3ToFBXPoint(fitValues[i2]), FlagsToFBXRotationOrder(flags));
                if ((animationSettings.importCameras && node->GetCamera()))
                {
                    Quaternionf camRotOffset = FBXCameraRotationOffset;
                    quatFitValues[i1] *= camRotOffset;
                    quatFitValues[i2] *= camRotOffset;
                }
                if ((animationSettings.importLights && node->GetLight()))
                {
                    Quaternionf lightRotOffset = FBXLightRotationOffset;
                    quatFitValues[i1] *= lightRotOffset;
                    quatFitValues[i2] *= lightRotOffset;
                }

                EnsureQuaternionContinuity(quatValues[keyIndex], quatFitValues[i1]);
                EnsureQuaternionContinuity(quatFitValues[i1], quatFitValues[i2]);

                last = quatFitValues[i2];
            }
        }
        AddKeyValues(outcurves, timeMap, quatValues, quatFitValues, 4, range.timeRangeOffset);
    }
    else
    {
        std::vector<Vector3f> values(timeMap.size());

        int keyIndex = 0;
        for (KeyInfoMap::const_iterator it = timeMap.begin(), end = timeMap.end(); it != end; ++it, ++keyIndex)
        {
            const KeyInfoGroup& keyInfo = it->second;

            values[keyIndex] = Vector3f(keyInfo.keys[0].value, keyInfo.keys[1].value, keyInfo.keys[2].value);
        }
        AddKeyValues(outcurves, timeMap, values, fitValues, 3, range.timeRangeOffset);
    }
}

void ConvertCurveVec3(
    // for error reporting
    const char* const takeName, const char* const nodeName, const char* const curveName,
    FbxAnimCurve** curve, FBXAnimationCurve* outcurves, const AnimationSettings& animationSettings, FbxVector4 defaultValue, bool remap, const AnimationTimeRange& range, CurveFlags& flags, FbxNode* node)
{
    ConvertCurve(takeName, nodeName, curveName, curve, outcurves, animationSettings, defaultValue, remap, RotationCurveImportOptions::kNoRotation, range, flags, node);
}

void ConvertCurveRotation(
	// for error reporting
	const char* const takeName, const char* const nodeName, const char* const curveName, RotationCurveImportOptions rotationOption,
	FbxAnimCurve** curve, FBXAnimationCurve* outcurves, const AnimationSettings& animationSettings, FbxVector4 defaultValue, const AnimationTimeRange& range, CurveFlags& flags, FbxNode* node)
{
	ConvertCurve(takeName, nodeName, curveName, curve, outcurves, animationSettings, defaultValue, false, rotationOption, range, flags, node);
}


void ConvertCurveFloat(
	// for error reporting
	const char* const takeName, const char* const nodeName, const char* const curveName,
	FbxAnimCurve& curve, FBXAnimationCurve& outcurve, const AnimationSettings& animationSettings, double defaultValue, const AnimationTimeRange& range, CurveFlags& flags, FbxNode* node)
{
	// HACK : remove this ultimate hack - faking Vector3 curve in order to be able to use the same ConvertCurveVec3 code...
	FbxAnimCurve* fbxCurves[] = { &curve, &curve, &curve };
	FBXAnimationCurve curves[3];

	ConvertCurveVec3(takeName, nodeName, curveName, fbxCurves, curves, animationSettings, FbxVector4(defaultValue, defaultValue, defaultValue), false, range, flags, node);
	outcurve = curves[0];
}

void ConvertCurveEulerDegreesRotation(FbxAnimCurve** fbxCurve, FbxVector4 defaultValue, const char* className, const char* propertyName,
	FBXImportNode*& importNode, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	FBXAnimationCurve rotation[3];
	ConvertCurveRotation(clip.name.c_str(), importNode->name.c_str(), propertyName, RotationCurveImportOptions::kImportEulerDegrees,
		fbxCurve, rotation, animationSettings, defaultValue, range, flags, node);
	for (int i = 0; i < 3; ++i)
	{
		FBXImportFloatAnimation animation;
		animation.node = importNode;
		animation.className = className;
		animation.propertyName = Format("%s.%c", propertyName, 'x' + i);
		animation.curve = rotation[i];
		clip.floatAnimations.push_back(animation);
	}

	// The *flags* param is both an input and an output, so we must clear the input that we don't want (in this case, the kConvertEuler flag).
	// Also, we cannot clear this in ConvertCurveRotation, because other users of that function expect to read this flag.
	flags &= ~kConvertEuler;
}

void ConvertCurveEulerRadiansRotation(FbxAnimCurve** fbxCurve, FbxVector4 defaultValue, const char* className, const char* propertyName,
	FBXImportNode*& importNode, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	FBXAnimationCurve rotation[3];
	ConvertCurveRotation(clip.name.c_str(), importNode->name.c_str(), propertyName, RotationCurveImportOptions::kImportEulerRadians,
		fbxCurve, rotation, animationSettings, defaultValue, range, flags, node);
	for (int i = 0; i < 3; ++i)
	{
		FBXImportFloatAnimation animation;
		animation.node = importNode;
		animation.className = className;
		animation.propertyName = Format("%s.%c", propertyName, 'x' + i);
		animation.curve = rotation[i];
		clip.floatAnimations.push_back(animation);
	}

	// The *flags* param is both an input and an output, so we must clear the input that we don't want (in this case, the kConvertEuler flag).
	// Also, we cannot clear this in ConvertCurveRotation, because other users of that function expect to read this flag.
	flags &= ~kConvertEuler;
}



template<class ConverterData, class Converter>
void ConvertCurves(FBXAnimationCurve& curve, Converter converter, ConverterData& data)
{
	for (int i = 0, size = curve.GetKeyCount(); i < size; ++i)
	{
        FBXAnimationCurve::FBXKeyframe& key = curve.GetKey(i);
		float valueIn = key.value + key.inSlope;
		float value = key.value;
		float valueOut = key.value + key.outSlope;

		valueIn = converter(data, valueIn);
		value = converter(data, value);
		valueOut = converter(data, valueOut);

		key.inSlope = valueIn - value;
		key.value = value;
		key.outSlope = valueOut - value;

		key.inWeight = key.outWeight = DefaultWeight<float>();
		key.weightedMode = kNotWeighted;
	}
}



bool IsTakeEmpty(const FbxAnimStack& animStack)
{
    int layerCount = animStack.GetSrcObjectCount<FbxAnimLayer>();
    if (layerCount > 0)
    {
        for (int j = 0; j < layerCount; ++j)
        {
            FbxAnimLayer* layer = animStack.GetSrcObject<FbxAnimLayer>(j);
            if (layer->GetSrcObjectCount() > 0)
            {
                return false;
            }
        }
    }
    return true;
}

struct InvertTransparencyFactor
{
	float operator()(const FbxSurfaceMaterial&, float value)
	{
		return 1.0f - value;
	}
};

static FbxAnimCurve* GetVisibilityCurveWithInheritance(FbxNode* node, FbxAnimLayer& animLayer)
{
	FbxAnimCurve* visibility = node->Visibility.GetCurve(&animLayer);
	// get the first parent valid visibility curve if any
	FbxAnimCurve* parentVisibility = NULL;
	FbxNode* current = node;
	FbxNode* parent = node->GetParent();
	while (parent != NULL && !HasAnyCurveValues(parentVisibility) && !CompareApproximately(current->VisibilityInheritance.Get(), 0.0))
	{
		parentVisibility = parent->Visibility.GetCurve(&animLayer);
		current = parent;
		parent = current->GetParent();
	}
	if (HasAnyCurveValues(visibility))
	{
		// The merge process is destructive inside the FBXNode
		// thus we only need to merge the curve with the first valid parent.
		MergeVisibilityAnimCurve(HasAnyCurveValues(parentVisibility) ? parentVisibility : NULL, visibility);
		return visibility;
	}
	return parentVisibility;
}

FBXImportFloatAnimation* ConvertFloatAnimation(FbxAnimCurve& fbxCurve, const double defaultValue, const char* className, const char* propertyName,
	FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	if (!HasAnyCurveValues(&fbxCurve))
		return NULL;
	else
	{
		clip.floatAnimations.push_back(FBXImportFloatAnimation());
		FBXImportFloatAnimation& animation = clip.floatAnimations.back();
		animation.node = importNode;
		animation.className = className;
		animation.propertyName = propertyName;

		ConvertCurveFloat(clip.name.c_str(), animation.node->name.c_str(), propertyName, fbxCurve, animation.curve, animationSettings, defaultValue, range, flags, node);

		return &animation;
	}
}

FBXImportFloatAnimation* ConvertFloatAnimation(FbxPropertyT<FbxDouble>& property, const char* className, const char* propertyName,
	FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	FbxAnimCurve* fbxCurve = property.GetCurve(&animLayer);
	if (!fbxCurve)
		return NULL;

	const double defaultValue = property.Get();

	return ConvertFloatAnimation(*fbxCurve, defaultValue, className, propertyName,
		importNode, animLayer, clip, range, animationSettings, flags, node);
}

FBXImportFloatAnimation* ConvertFloatAnimation(FbxPropertyT<FbxDouble3>& property, const int valueId, const char* curveId, const char* className, const char* propertyName,
	FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	FbxAnimCurve* fbxCurve = property.GetCurve(&animLayer, curveId);
	if (!fbxCurve)
		return NULL;

	const double defaultValue = property.Get()[valueId];

	return ConvertFloatAnimation(*fbxCurve, defaultValue, className, propertyName,
		importNode, animLayer, clip, range, animationSettings, flags, node);
}

void ConvertCustomFloatAnimation(FbxPropertyT<FbxDouble>& property, const char* propertyName,
	FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	FbxAnimCurve* fbxCurve = property.GetCurve(&animLayer);
	const double defaultValue = property.Get();

	if (HasAnyCurveValues(fbxCurve))
	{
		importNode->animatedCustomProperties.push_back(FBXImportCustomFloatAnimation());
		FBXImportCustomFloatAnimation& animation = importNode->animatedCustomProperties.back();
		animation.node = importNode;
		animation.propertyName = propertyName;
		animation.clip = &clip;
		ConvertCurveFloat(clip.name.c_str(), animation.node->name.c_str(), propertyName, *fbxCurve, animation.curve, animationSettings, defaultValue, range, flags, node);
	}
}

// in Maya the weight is a value in the interval [0, 100], but we prefer to use the interval [0, 1]
static const float kPercentMultiplier = 0.01f;

template<typename TConstraint>
struct CurveScalePredicate
{
	CurveScalePredicate(float scale) : m_Scale(scale) {}

	float operator()(const TConstraint&, float value)
	{
		return value * m_Scale;
	}

	float m_Scale;
};

void ImportAnimationClipsInfo(FbxScene& scene, FBXImportScene& importScene)
{
    FbxArray<FbxString*> animStacks;

    int count = scene.GetSrcObjectCount<FbxAnimStack>();
    importScene.animationClips.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        FbxAnimStack* animStack = scene.GetSrcObject<FbxAnimStack>(i);

        if (!IsTakeEmpty(*animStack))
        {
            FBXImportAnimationClip clip;
            clip.name = animStack->GetName();
            importScene.animationClips.push_back(clip);
        }
    }
}

static void RecursiveImportAnimation(FbxNode* node,	FbxAnimLayer& animLayer,	FBXImportAnimationClip& clip,	FbxScene& fbxScene,	const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap,	const FBXMeshToInfoMap& fbxMeshToInfoMap,	const AnimationTimeRange& range,
	NANAnimationInfos& nanAnimationInfos,const AnimationSettings& animationSettings)
{
	ImportAnimationTake(node, animLayer, clip, fbxScene, fbxNodeMap, fbxMeshToInfoMap, range, nanAnimationInfos, animationSettings);

	int count = node->GetChildCount();
	for (int i = 0; i < count; i++)
	{
		FbxNode* curChild = node->GetChild(i);
		RecursiveImportAnimation(curChild, animLayer, clip, fbxScene, fbxNodeMap, fbxMeshToInfoMap, range, nanAnimationInfos, animationSettings);
	}
}

void ImportAllAnimations(FbxManager& sdkManager, FbxScene& fbxScene, FbxNode* root, FBXImportScene& unity, const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap,const FBXMeshToInfoMap& fbxMeshToInfoMap, const AnimationSettings& animationSettings)
{
    const float kMaxTimeRange = 100000;
    // The root node never has any animation, so only process it's children.
    int childCount = root->GetChildCount();

    for (int i = 0, c = fbxScene.GetSrcObjectCount<FbxAnimStack>(); i < c; ++i)
    {
        FbxAnimStack* animStack = fbxScene.GetSrcObject<FbxAnimStack>(i);

        const std::string takeName = animStack->GetName();

        AnimationTimeRange range;

        {
            SetupTimeRange(fbxScene, *animStack, animationSettings, range);
            float timeRange = range.timeRangeEnd - range.timeRangeStart;
            bool usesInfinity = range.timeRangeStart == -std::numeric_limits<float>::infinity() && range.timeRangeEnd == std::numeric_limits<float>::infinity();
            if (timeRange > kMaxTimeRange && !usesInfinity)
            {
                //std::string timeRangeString = Format("%d", static_cast<int>(timeRange));
                //std::string maxTimeRangeString = Format("%d", static_cast<int>(kMaxTimeRange));
                //std::string errorDisplayString = FormatOrdered("Time range ({0}) for take '{1}' is larger than maximum allowed ({2}). Check your file - it most likely has keys in far negative or positive timeline.\n", timeRangeString.c_str(), takeName.c_str(), maxTimeRangeString.c_str(), NULL);
                //ReportError(errorDisplayString.c_str());
                continue;
            }
        }

        int animLayerCount = animStack->GetSrcObjectCount<FbxAnimLayer>();
        if (animLayerCount > 1)
        {
            FbxTime bakeStartTime, bakeEndTime, fbxSampleRate;
            bakeStartTime.SetSecondDouble(range.bakeStart);
            bakeEndTime.SetSecondDouble(range.bakeStop);

            fbxSampleRate.SetSecondDouble(animationSettings.sampleRate);

            if (!animStack->BakeLayers(fbxScene.GetAnimationEvaluator(), bakeStartTime, bakeEndTime, fbxSampleRate))
            {
                ReportWarning("Failed to bake layers.\n");
            }


            animLayerCount = animStack->GetSrcObjectCount<FbxAnimLayer>();
            if (animLayerCount > 1)
            {
                ReportWarning("Expected to have 1 layer after BakeLayers.");
                //ReportWarning("Expected to have 1 layer after BakeLayers. It has %d now.\n", animLayerCount);
            }

        }

        if (animLayerCount > 0)
        {
            FBXImportAnimationClip* clip = 0;
            for (unsigned int j = 0; j < unity.animationClips.size(); ++j)
            {
                if (unity.animationClips[j].name == takeName)
                {
                    clip = &unity.animationClips[j];
                    break;
                }
            }

            if (clip)
            {
                FbxAnimLayer* animLayer = animStack->GetSrcObject<FbxAnimLayer>();

                clip->bakeStart = range.bakeStart;
                clip->bakeStop = range.bakeStop;

                NANAnimationInfos nanAnimationInfos;

                for (int i = 0; i < childCount; i++)
                {
                    RecursiveImportAnimation(root->GetChild(i),
                        *animLayer,
                        *clip,
                        fbxScene,
                        fbxNodeMap,
                        fbxMeshToInfoMap,
                        range,
                        nanAnimationInfos,
                        animationSettings);
                }

                if (!nanAnimationInfos.empty())
                {
                    NANAnimationInfo accumulated;
                    for (NANAnimationInfos::const_iterator it = nanAnimationInfos.begin(), end = nanAnimationInfos.end(); it != end; ++it)
                        accumulated.Accumulate(*it);

                    std::ostringstream str;
                    str << "Take/AnimStack '" << takeName << "' contains animation curves with invalid numbers (NANs) on these nodes"
                        << " (" << accumulated.GetHelperString() << "): ";
                    for (NANAnimationInfos::const_iterator it = nanAnimationInfos.begin(), begin = nanAnimationInfos.begin(), end = nanAnimationInfos.end(); it != end; ++it)
                    {
                        if (it != begin)
                            str << ", ";

                        str << "'" << it->nodeName << "' (" << it->GetString() << ")";
                    }
                    str << ". The invalid numbers will be replaced by default numbers from default pose.\n"
                        << "This error indicates corrupt FBX file. "
                        << "In some case this error can be caused by having 0 scales in animation. "
                        << "0 scales are not supported in FBXSDK. Use very small scales (like 1e-5) instead.";

                    ReportError(str.str().c_str());
                }
            }
        }
    }
}

void CollectCurvesFromAllShapeChannels(FBXImportNode& importNode, FbxMesh& fbxMesh, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, FbxNode* node)
{
	// collect curves from all channels
	const int blendShapeDeformerCount = fbxMesh.GetDeformerCount(FbxDeformer::eBlendShape);
	for (int i = 0; i < blendShapeDeformerCount; ++i)
	{
		FbxBlendShape* blendShapeDeformer = (FbxBlendShape*)fbxMesh.GetDeformer(i, FbxDeformer::eBlendShape);

		const int blendShapeChannelCount = blendShapeDeformer->GetBlendShapeChannelCount();
		for (int j = 0; j < blendShapeChannelCount; ++j)
		{
			FbxBlendShapeChannel* blendShapeChannel = blendShapeDeformer->GetBlendShapeChannel(j);

			FbxAnimCurve* curve = fbxMesh.GetShapeChannel(i, j, &animLayer);
			if (HasAnyCurveValues(curve))
			{
				clip.floatAnimations.push_back(FBXImportFloatAnimation());
                FBXImportFloatAnimation& animation = clip.floatAnimations.back();

				animation.node = &importNode;
				animation.className = "SkinnedMeshRenderer";
				animation.propertyName = GenerateBlendShapeAnimationAttribute(blendShapeChannel);

				CurveFlags curveflags = kNone;
				ConvertCurveFloat(clip.name.c_str(), importNode.name.c_str(), "shape", *curve, animation.curve, animationSettings, 0, range, curveflags, node);
				flags |= curveflags;
			}
		}
	}
}


void ImportShapesAnimation(FbxNode* node, FBXImportNode& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, const FBXMeshToInfoMap& fbxMeshToInfoMap)
{
	FbxMesh* fbxMesh = node->GetMesh();
	if (fbxMesh)
	{
		////@TODO: Remove if there are no channels...
		CollectCurvesFromAllShapeChannels(importNode, *fbxMesh, animLayer, clip, range, animationSettings, flags, node);
	}
}

void ImportMaterialPropertyAnimations(FbxScene& fbxScene,FbxNode* fbxNode,FBXImportNode*& importNode,FbxAnimLayer& animLayer,FBXImportAnimationClip& clip,const AnimationTimeRange& range,
	const AnimationSettings& animationSettings,	CurveFlags& flags)
{
	for (int i = 0, count = fbxScene.GetMaterialCount(); i < count; ++i)
	{
		if (const FbxSurfaceMaterial* material = fbxScene.GetMaterial(i))
		{
			FbxPropertyT<FbxDouble3> propColor = material->FindProperty(FbxSurfaceMaterial::sDiffuse);
			const FbxAnimCurve* diffuseColorCurve = propColor.GetCurve(&animLayer);
			if (HasAnyCurveValues(diffuseColorCurve))
			{
				ConvertFloatAnimation(propColor, 0, FBXSDK_CURVENODE_COLOR_RED, "MeshRenderer", "material._Color.r", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
				ConvertFloatAnimation(propColor, 1, FBXSDK_CURVENODE_COLOR_GREEN, "MeshRenderer", "material._Color.g", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
				ConvertFloatAnimation(propColor, 2, FBXSDK_CURVENODE_COLOR_BLUE, "MeshRenderer", "material._Color.b", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
			}

			// Max exports the alpha as TransparencyFactor
			FbxPropertyT<FbxDouble> propTransparencyFactor = material->FindProperty(FbxSurfaceMaterial::sTransparencyFactor);
			if (propTransparencyFactor.IsAnimated(&animLayer))
			{
				const FbxAnimCurve* transparencyFactorCurve = propTransparencyFactor.GetCurve(&animLayer);
				if (HasAnyCurveValues(transparencyFactorCurve))
				{
					ConvertFloatAnimation(propTransparencyFactor, "MeshRenderer", "material._Color.a", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
					ConvertCurves(clip.floatAnimations.back().curve, InvertTransparencyFactor(), *material);
				}
			}
			else
			{
				// Maya exports the alpha as TransparencyColor
				FbxPropertyT<FbxDouble> propTransparencyColor = material->FindProperty(FbxSurfaceMaterial::sTransparentColor);
				const FbxAnimCurve* transparencyColorCurve = propTransparencyColor.GetCurve(&animLayer);
				if (HasAnyCurveValues(transparencyColorCurve))
				{
					ConvertFloatAnimation(propTransparencyColor, "MeshRenderer", "material._Color.a", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
					ConvertCurves(clip.floatAnimations.back().curve, InvertTransparencyFactor(), *material);
				}
			}
		}
	}
}

void ImportCameraAnimations(FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags)
{
	FbxCamera* fbxCamera = fbxNode->GetCamera();
	if (fbxCamera)
	{
		FbxPropertyT<FbxDouble>* property;
		FOVConversionFunction* converter;

		if (fbxCamera->GetApertureMode() == FbxCamera::eFocalLength)
		{
			ConvertFloatAnimation(fbxCamera->FocalLength, "Camera", "m_FocalLength", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
		}
		else if (GetFieldOfViewProperty(*fbxCamera, property, converter))
		{
			FBXImportFloatAnimation* animation = ConvertFloatAnimation(*property, "Camera", "field of view", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);

			// XSI will never set ApertureMode to eFOCAL_LENGTH even when Focal Length is animated, but
			// Field Of View is not, so we just check for Focal Length animation and use that instead if it's available
			if (!animation)
			{
				GetFocalLengthProperty(*fbxCamera, property, converter);
				animation = ConvertFloatAnimation(*property, "Camera", "field of view", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
			}

			if (animation && converter)
			{
				ConvertCurves(animation->curve, converter, *fbxCamera);
			}
		}

		ConvertFloatAnimation(fbxCamera->Roll, "LookAtConstraint", "m_Roll", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
	}
}

void ImportLightAnimations(FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags)
{
	FbxLight* fbxLight = fbxNode->GetLight();
	if (fbxLight)
	{
		ConvertFloatAnimation(fbxLight->Color, 0, FBXSDK_CURVENODE_COLOR_RED, "Light", "m_Color.r", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
		ConvertFloatAnimation(fbxLight->Color, 1, FBXSDK_CURVENODE_COLOR_GREEN, "Light", "m_Color.g", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
		ConvertFloatAnimation(fbxLight->Color, 2, FBXSDK_CURVENODE_COLOR_BLUE, "Light", "m_Color.b", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);

		ConvertFloatAnimation(fbxLight->OuterAngle, "Light", "m_SpotAngle", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);

		FBXImportFloatAnimation* animation = ConvertFloatAnimation(fbxLight->Intensity, "Light", "m_Intensity", importNode, animLayer, clip, range, animationSettings, flags, fbxNode);

		if (animation)
			ConvertCurves(animation->curve, ScaleLightIntensity, *fbxLight);
	}
}

void ImportCustomPropertyAnimations(FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags)
{
	FbxProperty prop = fbxNode->GetFirstProperty();
	while (prop.IsValid())
	{
		if (prop.GetFlag(FbxPropertyFlags::eUserDefined))
		{
			FbxDataType propertyType = prop.GetPropertyDataType();
			// only support floats for now.
			if (propertyType.Is(FbxFloatDT) || propertyType.Is(FbxDoubleDT) || propertyType.Is(FbxRealDT))
			{
				FbxPropertyT<FbxDouble> propT = FbxPropertyT<FbxDouble>(prop);
				ConvertCustomFloatAnimation(propT, prop.GetNameAsCStr(), importNode, animLayer, clip, range, animationSettings, flags, fbxNode);
			}
		}
		prop = fbxNode->GetNextProperty(prop);
	}
}

template<typename TConstraint>
void ImportConstraintSourceWeightAnimCurves(TConstraint* constraint, const char* componentName, FbxNode* node, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip,
	const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags)
{
	for (int i = 0, c = constraint->GetConstraintSourceCount(); i < c; ++i)
	{
		FbxObject* source = constraint->GetConstraintSource(i);
		std::string weightPropStr = Format("%s.%s", source->GetNameWithoutNameSpacePrefix().Buffer(), "Weight");
		FbxProperty weightProp = constraint->FindProperty(weightPropStr.c_str());

		if (weightProp.IsValid())
		{
			FbxAnimCurve* weightCurve = weightProp.GetCurve(&animLayer);
			if (HasAnyCurveValues(weightCurve))
			{
				std::string propStr = Format("m_Sources.Array.data[%d].weight", i);
				ConvertFloatAnimation(*weightCurve, weightProp.Get<FbxFloat>(), componentName, propStr.c_str(), importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertCurves<TConstraint, CurveScalePredicate<TConstraint> >(clip.floatAnimations.back().curve, CurveScalePredicate<TConstraint>(kPercentMultiplier), *constraint);
			}
		}
	}
}

void ImportParentConstraintOffsets(FbxConstraintParent* constraint, FbxNode* node, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip,
	const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags)
{
	for (int sourceIdx = 0, c = constraint->GetConstraintSourceCount(); sourceIdx < c; ++sourceIdx)
	{
		FbxObject* source = constraint->GetConstraintSource(sourceIdx);

		std::string translationOffsetPropStr = Format("%s.%s", source->GetNameWithoutNameSpacePrefix().Buffer(), "Offset T");
		std::string rotationOffsetPropStr = Format("%s.%s", source->GetNameWithoutNameSpacePrefix().Buffer(), "Offset R");

		FbxPropertyT<FbxVector4> translationProp = static_cast<FbxPropertyT<FbxVector4>>(constraint->FindProperty(translationOffsetPropStr.c_str()));
		FbxPropertyT<FbxVector4> rotProp = static_cast<FbxPropertyT<FbxVector4>>(constraint->FindProperty(rotationOffsetPropStr.c_str()));

		if (translationProp.IsValid())
		{
			FbxAnimCurve* offset[3] =
			{
				translationProp.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				translationProp.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				translationProp.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(offset))
			{
				std::string propStr = Format("m_TranslationOffsets.Array.data[%d]", sourceIdx);

				FBXAnimationCurve offsetCurve[3];
				ConvertCurve(clip.name.c_str(), importNode->name.c_str(), propStr.c_str(),
					offset, offsetCurve, animationSettings, translationProp.EvaluateValue(0),
					true, RotationCurveImportOptions::kNoRotation, range, flags, node);
				for (int i = 0; i < 3; ++i)
				{
					FBXImportFloatAnimation animation;
					animation.node = importNode;
					animation.className = "ParentConstraint";
					animation.propertyName = Format("%s.%c", propStr.c_str(), 'x' + i);
					animation.curve = offsetCurve[i];
					clip.floatAnimations.push_back(animation);
				}
			}
		}

		if (rotProp.IsValid())
		{
			FbxAnimCurve* offset[3] =
			{
				rotProp.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				rotProp.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				rotProp.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(offset))
			{
				// for some reason, the parent constraint rotation offsets are stored in radians in the FBX file,
				// unlike the offsets of all other constraints
				std::string propStr = Format("m_RotationOffsets.Array.data[%d]", sourceIdx);
				ConvertCurveEulerRadiansRotation(offset, rotProp.EvaluateValue(0),
					"ParentConstraint", propStr.c_str(),
					importNode, clip, range, animationSettings, flags, node);
			}
		}
	}
}


void ImportConstraintPropertyAnimations(FbxScene& fbxScene, FbxNode* node, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip,
	const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags)
{
	for (int i = 0, c = fbxScene.GetSrcObjectCount<FbxConstraint>(); i < c; ++i)
	{
		FbxConstraint* constraint = fbxScene.GetSrcObject<FbxConstraint>(i);
		if (!constraint)
			continue;

		if (!IsConstraintTypeSupported(constraint->GetConstraintType()))
			continue;

		if (node != constraint->GetConstrainedObject())
			continue;

		ConstraintKind constraintType = static_cast<ConstraintKind>(constraint->GetConstraintType());
		switch (constraintType)
		{
		case kConstraintKindPosition:
		{
			FbxConstraintPosition* constraintPostion = static_cast<FbxConstraintPosition*>(constraint);

			FbxAnimCurve* weightCurve = constraintPostion->Weight.GetCurve(&animLayer);
			if (HasAnyCurveValues(weightCurve))
			{
				ConvertFloatAnimation(*weightCurve, constraintPostion->Weight.Get(), "PositionConstraint", "m_Weight", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertCurves<FbxConstraint, CurveScalePredicate<FbxConstraint> >(clip.floatAnimations.back().curve, CurveScalePredicate<FbxConstraint>(kPercentMultiplier), *constraint);
			}

			FbxAnimCurve* offset[3] =
			{
				constraintPostion->Translation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintPostion->Translation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintPostion->Translation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(offset))
			{
				FBXAnimationCurve offsetCurve[3];
				ConvertCurve(clip.name.c_str(), importNode->name.c_str(), "m_TranslationOffset",
					offset, offsetCurve, animationSettings, constraintPostion->Translation.EvaluateValue(0),
					true, RotationCurveImportOptions::kNoRotation, range, flags, node);
				for (int i = 0; i < 3; ++i)
				{
					FBXImportFloatAnimation animation;
					animation.node = importNode;
					animation.className = "PositionConstraint";
					animation.propertyName = Format("m_TranslationOffset.%c", 'x' + i);
					animation.curve = offsetCurve[i];
					clip.floatAnimations.push_back(animation);
				}
			}

			ImportConstraintSourceWeightAnimCurves(constraintPostion, "PositionConstraint", node, importNode, animLayer, clip, range, animationSettings, flags);
			break;
		}
		case kConstraintKindRotation:
		{
			FbxConstraintRotation* constraintRotation = static_cast<FbxConstraintRotation*>(constraint);

			FbxAnimCurve* weightCurve = constraintRotation->Weight.GetCurve(&animLayer);
			if (HasAnyCurveValues(weightCurve))
			{
				ConvertFloatAnimation(*weightCurve, constraintRotation->Weight.Get(), "RotationConstraint", "m_Weight", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertCurves<FbxConstraint, CurveScalePredicate<FbxConstraint> >(clip.floatAnimations.back().curve, CurveScalePredicate<FbxConstraint>(kPercentMultiplier), *constraint);
			}
			FbxAnimCurve* offset[3] =
			{
				constraintRotation->Rotation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintRotation->Rotation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintRotation->Rotation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(offset))
			{
				ConvertCurveEulerDegreesRotation(offset, constraintRotation->Rotation.EvaluateValue(0),
					"RotationConstraint", "m_RotationOffset",
					importNode, clip, range, animationSettings, flags, node);
			}
			ImportConstraintSourceWeightAnimCurves(constraintRotation, "RotationConstraint", node, importNode, animLayer, clip, range, animationSettings, flags);
			break;
		}
		case kConstraintKindScale:
		{
			FbxConstraintScale* constraintScale = static_cast<FbxConstraintScale*>(constraint);

			FbxAnimCurve* weightCurve = constraintScale->Weight.GetCurve(&animLayer);
			if (HasAnyCurveValues(weightCurve))
			{
				ConvertFloatAnimation(*weightCurve, constraintScale->Weight.Get(), "ScaleConstraint", "m_Weight", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertCurves<FbxConstraint, CurveScalePredicate<FbxConstraint> >(clip.floatAnimations.back().curve, CurveScalePredicate<FbxConstraint>(kPercentMultiplier), *constraint);
			}

			FbxAnimCurve* offset[3] =
			{
				constraintScale->Scaling.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintScale->Scaling.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintScale->Scaling.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(offset))
			{
				ConvertFloatAnimation(constraintScale->Scaling, 0, FBXSDK_CURVENODE_COMPONENT_X, "ScaleConstraint", "m_ScaleOffset.x", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintScale->Scaling, 1, FBXSDK_CURVENODE_COMPONENT_Y, "ScaleConstraint", "m_ScaleOffset.y", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintScale->Scaling, 2, FBXSDK_CURVENODE_COMPONENT_Z, "ScaleConstraint", "m_ScaleOffset.z", importNode, animLayer, clip, range, animationSettings, flags, node);
			}

			ImportConstraintSourceWeightAnimCurves(constraintScale, "ScaleConstraint", node, importNode, animLayer, clip, range, animationSettings, flags);
			break;
		}
		case kConstraintKindParent:
		{
			FbxConstraintParent* constraintParent = static_cast<FbxConstraintParent*>(constraint);

			FbxAnimCurve* weightCurve = constraintParent->Weight.GetCurve(&animLayer);
			if (HasAnyCurveValues(weightCurve))
			{
				ConvertFloatAnimation(*weightCurve, constraintParent->Weight.Get(), "ParentConstraint", "m_Weight", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertCurves<FbxConstraint, CurveScalePredicate<FbxConstraint> >(clip.floatAnimations.back().curve, CurveScalePredicate<FbxConstraint>(kPercentMultiplier), *constraint);
			}

			ImportParentConstraintOffsets(constraintParent, node, importNode, animLayer, clip, range, animationSettings, flags);
			ImportConstraintSourceWeightAnimCurves(constraintParent, "ParentConstraint", node, importNode, animLayer, clip, range, animationSettings, flags);
			break;
		}
		case kConstraintKindAim:
		{
			FbxConstraintAim* constraintAim = static_cast<FbxConstraintAim*>(constraint);

			FbxAnimCurve* weightCurve = constraintAim->Weight.GetCurve(&animLayer);
			if (HasAnyCurveValues(weightCurve))
			{
				ConvertFloatAnimation(*weightCurve, constraintAim->Weight.Get(), "AimConstraint", "m_Weight", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertCurves<FbxConstraint, CurveScalePredicate<FbxConstraint> >(clip.floatAnimations.back().curve, CurveScalePredicate<FbxConstraint>(kPercentMultiplier), *constraint);
			}

			FbxAnimCurve* offset[3] =
			{
				constraintAim->RotationOffset.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintAim->RotationOffset.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintAim->RotationOffset.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(offset))
			{
				ConvertCurveEulerDegreesRotation(offset, constraintAim->RotationOffset.EvaluateValue(0),
					"AimConstraint", "m_RotationOffset",
					importNode, clip, range, animationSettings, flags, node);
			}

			FbxAnimCurve* aimVector[3] =
			{
				constraintAim->AimVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintAim->AimVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintAim->AimVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(aimVector))
			{
				ConvertFloatAnimation(constraintAim->AimVector, 0, FBXSDK_CURVENODE_COMPONENT_X, "AimConstraint", "m_AimVector.x", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintAim->AimVector, 1, FBXSDK_CURVENODE_COMPONENT_Y, "AimConstraint", "m_AimVector.y", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintAim->AimVector, 2, FBXSDK_CURVENODE_COMPONENT_Z, "AimConstraint", "m_AimVector.z", importNode, animLayer, clip, range, animationSettings, flags, node);
			}

			FbxAnimCurve* upVector[3] =
			{
				constraintAim->UpVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintAim->UpVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintAim->UpVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(upVector))
			{
				ConvertFloatAnimation(constraintAim->UpVector, 0, FBXSDK_CURVENODE_COMPONENT_X, "AimConstraint", "m_UpVector.x", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintAim->UpVector, 1, FBXSDK_CURVENODE_COMPONENT_Y, "AimConstraint", "m_UpVector.y", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintAim->UpVector, 2, FBXSDK_CURVENODE_COMPONENT_Z, "AimConstraint", "m_UpVector.z", importNode, animLayer, clip, range, animationSettings, flags, node);
			}

			FbxAnimCurve* worldUpVector[3] =
			{
				constraintAim->WorldUpVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
				constraintAim->WorldUpVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
				constraintAim->WorldUpVector.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z),
			};
			if (HasAnyCurveValues(worldUpVector))
			{
				ConvertFloatAnimation(constraintAim->WorldUpVector, 0, FBXSDK_CURVENODE_COMPONENT_X, "AimConstraint", "m_WorldUpVector.x", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintAim->WorldUpVector, 1, FBXSDK_CURVENODE_COMPONENT_Y, "AimConstraint", "m_WorldUpVector.y", importNode, animLayer, clip, range, animationSettings, flags, node);
				ConvertFloatAnimation(constraintAim->WorldUpVector, 2, FBXSDK_CURVENODE_COMPONENT_Z, "AimConstraint", "m_WorldUpVector.z", importNode, animLayer, clip, range, animationSettings, flags, node);
			}
			ImportConstraintSourceWeightAnimCurves(constraintAim, "AimConstraint", node, importNode, animLayer, clip, range, animationSettings, flags);
			break;
		}
		default:
			break;
		}
	}
}


static void ImportAnimationTake(FbxNode* node, FbxAnimLayer& animLayer,  FBXImportAnimationClip& clip,   FbxScene& fbxScene,   const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap,    const FBXMeshToInfoMap& fbxMeshToInfoMap,    const AnimationTimeRange& range,
    NANAnimationInfos& nanAnimationInfos, const AnimationSettings& animationSettings)
{
    FbxAnimCurve* translation[3] =
    {
        node->LclTranslation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
        node->LclTranslation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
        node->LclTranslation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z)
    };
    FbxAnimCurve* scale[3] =
    {
        node->LclScaling.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
        node->LclScaling.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
        node->LclScaling.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z)
    };
    FbxAnimCurve* rotation[3] =
    {
        node->LclRotation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_X),
        node->LclRotation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Y),
        node->LclRotation.GetCurve(&animLayer, FBXSDK_CURVENODE_COMPONENT_Z)
    };

    std::map<FbxNode*, FBXImportNode*>::const_iterator it = fbxNodeMap.find(node);
    if (it == fbxNodeMap.end())
    {
        ReportError("Internal error: node for animation not found!");
        return;
    }
    
    FBXImportNode* importNode = it->second;
    NANAnimationInfo nanInfo;

    if (HasAnyCurveValues(translation) || HasAnyCurveValues(scale) || HasAnyCurveValues(rotation))
    {
        clip.nodeAnimations.push_back(FBXImportNodeAnimation());
        FBXImportNodeAnimation& animation = clip.nodeAnimations.back();

        EFbxRotationOrder order;
        node->GetRotationOrder(FbxNode::eSourcePivot, order);
        animation.rotationOrder = (RotationOrder)(int)order;

        animation.node = importNode;

        for (int i = 0; i < 4; ++i)
        {
            animation.rotation[i].SetRotationOrder(animation.rotationOrder);
        }

        CurveFlags flags = kNone;
        flags |= FBXRotationOrderToCurveFlags(order);

        const char* nodeName = animation.node->name.c_str();
        ConvertCurveVec3(clip.name.c_str(), nodeName, "translation", translation, animation.translation, animationSettings, node->LclTranslation.Get(), true, range, flags, node);
        nanInfo.hasNANsPos = ((flags & kHasNaNs) != 0);
        flags &= ~kHasNaNs;
        ConvertCurveVec3(clip.name.c_str(), nodeName, "scale", scale, animation.scale, animationSettings, node->LclScaling.Get(), false, range, flags, node);
        nanInfo.hasNANsScale = ((flags & kHasNaNs) != 0);
        flags &= ~kHasNaNs;
        ConvertCurveRotation(clip.name.c_str(), nodeName, "rotation", RotationCurveImportOptions::kImportQuaternion, rotation, animation.rotation, animationSettings, node->LclRotation.EvaluateValue(0), range, flags, node);
        nanInfo.hasNANsRot = ((flags & kHasNaNs) != 0);
        animation.rotationType = flags & kConvertEuler ? kRotationImportEuler : kRotationImportQuat;
        if (animation.rotationType == kRotationImportQuat)
        {
            animation.rotationOrder = kOrderUnityDefault;
        }
    }

    CurveFlags flags = kNone;
    if (animationSettings.importVisibility)
    {
        FbxAnimCurve* visibility = GetVisibilityCurveWithInheritance(node, animLayer);
        if (HasAnyCurveValues(visibility) && importNode->meshIndex != -1)
        {
            ConvertFloatAnimation(*visibility, node->Visibility.Get(), "Renderer", "m_Enabled", importNode, animLayer, clip, range, animationSettings, flags, node);
            flags = kNone;
        }
    }

    if (animationSettings.importBlendShapes)
    {
        ImportShapesAnimation(node, *importNode, animLayer, clip, range, animationSettings, flags, fbxMeshToInfoMap);
    }

    if (animationSettings.importCameras && importNode->cameraIndex >= 0)
    {
        ImportCameraAnimations(node, importNode, animLayer, clip, range, animationSettings, flags);
        nanInfo.hasNANsCamera = ((flags & kHasNaNs) != 0);
        flags = kNone;
    }

    if (animationSettings.importLights && importNode->lightIndex >= 0)
    {
        ImportLightAnimations(node, importNode, animLayer, clip, range, animationSettings, flags);
        nanInfo.hasNANsLight = ((flags & kHasNaNs) != 0);
        flags = kNone;
    }

    if (animationSettings.importAnimatedCustomProperties)
    {
        ImportCustomPropertyAnimations(node, importNode, animLayer, clip, range, animationSettings, flags);
    }

    if (animationSettings.importConstraints)
    {
        ImportConstraintPropertyAnimations(fbxScene, node, importNode, animLayer, clip, range, animationSettings, flags);
    }

    if (animationSettings.importMaterials)
    {
        ImportMaterialPropertyAnimations(fbxScene, node, importNode, animLayer, clip, range, animationSettings, flags);
    }

    if (nanInfo.HasNAN())
    {
        //Assert(importNode);
        nanInfo.nodeName = importNode->name;
        nanAnimationInfos.push_back(nanInfo);
    }
}

namespace {
	// checks if all indices and weigths are the same
	void ValidateIdenticalWeights(const std::vector<BoneWeights4>& boneWeights, const int boneCount, const char* meshName)
	{
		if (boneWeights.empty())
			return;

		const BoneWeights4& firstBW = boneWeights[0];

		// checking if weights are the same on first bone
		for (int j = 1; j < 4; ++j)
		{
			if (firstBW.weight[0] != firstBW.weight[j])
				return;
		}

		// checking if weights and indices on other bones match the ones on the first bone
		for (size_t i = 1, size = boneWeights.size(); i < size; ++i)
		{
			const BoneWeights4& bw = boneWeights[i];
			for (int j = 0; j < 4; ++j)
			{
				if (bw.boneIndex[j] != firstBW.boneIndex[j] || bw.weight[j] != firstBW.weight[j])
					return;
			}
		}
		std::string ss = Format("All vertices are affected by same bones (%d, %d, %d, %d) and same weights (%f, %f, %f, %f) on mesh '%s'. Bone count: %d; vertex count: %d.\n",
			firstBW.boneIndex[0], firstBW.boneIndex[1], firstBW.boneIndex[2], firstBW.boneIndex[3],
			firstBW.weight[0], firstBW.weight[1], firstBW.weight[2], firstBW.weight[3],
			meshName, boneCount, boneWeights.size());
		ReportWarning(ss);
	}

	// checks if all indices and weights are in range
	void ValidateBoneWeightRanges(const std::vector<BoneWeights4>& boneWeights, const int boneCount, const char* meshName)
	{
		const int kMaxInvalidCount = 10;
		int invalidCount = 0;
		std::ostringstream oss;

		// checking that weights and indices are in range
		for (size_t i = 0, size = boneWeights.size(); i < size; ++i)
		{
			const BoneWeights4& bw = boneWeights[i];

			std::ostringstream ossBW;
			for (int j = 0; j < 4; ++j)
			{
				if (!IsFinite(bw.weight[j]) ||
					bw.boneIndex[j] < 0 || bw.boneIndex[j] > boneCount ||
					bw.weight[j] < 0 || bw.weight[j] > 1)
				{
					if (j > 0)
						ossBW << ", ";

					ossBW << "(" << bw.boneIndex[j] << "; " << bw.weight[j] << ")";
				}
			}

			const std::string invalidBW = ossBW.str();
			if (!invalidBW.empty())
			{
				if (invalidCount < kMaxInvalidCount)
				{
					if (invalidCount > 0)
						oss << "\n";

					oss << invalidCount << ": (" << invalidBW << ")";
				}
				++invalidCount;
			}
		}

		if (invalidCount > 0)
		{
			if (invalidCount > kMaxInvalidCount)
				oss << "\nand so on..";

			std::string ss =Format( "Mesh '%s' (bone count: %d) has invalid %d (out of %d) BoneWeights (bone index; weight): \n%s.\n",
				meshName, boneCount, invalidCount, boneWeights.size(), oss.str().c_str());
			ReportWarning(ss);
		}
	}

	void ValidateSkin(const std::vector<BoneWeights4>& boneWeights, const int boneCount, const char* meshName)
	{
		ValidateIdenticalWeights(boneWeights, boneCount, meshName);
		ValidateBoneWeightRanges(boneWeights, boneCount, meshName);
	}

	bool IsValidVertex(const Vector3f& v)
	{
		return !IsNAN(v.x) && !IsNAN(v.y) && !IsNAN(v.z) && !IsPlusInf(v.x) && !IsPlusInf(v.y) && !IsPlusInf(v.z) && !IsMinusInf(v.x) && !IsMinusInf(v.y) && !IsMinusInf(v.z);
	}

	// referenceVertices are used for blendshapes
	void ImportVertices(const FbxGeometryBase& fbx, const std::vector<Vector3f>* referenceVertices, std::vector<Vector3f>& vertices, std::vector<int>& invalidVertices)
	{
		int vertexCount = fbx.GetControlPointsCount();
		vertices.resize(vertexCount);
		FbxVector4* controlPoints = fbx.GetControlPoints();
		for (int i = 0; i < vertexCount; i++)
		{
			vertices[i] = FBXPointToVector3Remap(controlPoints[i]);

			// case 370153: vertices in fbx can contain NaNs, so we set these vertices to (0,0,0) for meshes and to referenceVertices for blendShapes
			if (!IsValidVertex(vertices[i]))
			{
				vertices[i] = referenceVertices ? (*referenceVertices)[i] : Vector3f::zero;
				invalidVertices.push_back(i);
			}
		}
	}

	void PrintGenericInvalidList(const std::string& nameStr, const size_t maxIndexCount, const size_t totalCount, const std::vector<int>& indices, const std::string& messageText)
	{
		std::ostringstream oss;
		oss << nameStr << " has " << indices.size()
			<< " (out of " << totalCount << ") " << messageText << ". The list of vertices: ";

		for (size_t i = 0, size = std::min<size_t>(maxIndexCount, indices.size()); i < size; ++i)
		{
			if (i != 0)
				oss << ", ";
			oss << indices[i];
		}

		if (indices.size() > maxIndexCount)
			oss << " and so on...";

		oss << std::endl;

		ReportWarning(oss.str().c_str());
	}

	void PrintInvalidList(const std::string& meshName, const size_t maxIndexCount, const size_t totalCount, const std::vector<int>& indices, const std::string& messageText)
	{
		PrintGenericInvalidList("Mesh '" + meshName + "'", maxIndexCount, totalCount, indices, messageText);
	}

	class NormalCalculation
	{
	private:
		struct VertexNormalGroup
		{
			int smoothingGroup;
			Vector3f normal;
			int vertexCount;

			VertexNormalGroup() : smoothingGroup(0), normal(Vector3f::zero), vertexCount(0) {}

			void Add(int sg, const Vector3f& n, int vc)
			{
				smoothingGroup |= sg;
				normal += n;
				vertexCount += vc;
			}

			void Add(int sg, const Vector3f& n) { Add(sg, n, 1); }
			void Add(const VertexNormalGroup& g) { Add(g.smoothingGroup, g.normal, g.vertexCount); }

			void Normalize()
			{
				normal = NormalizeSafe(normal);
			}
		};

		struct SmoothVertex
		{
			std::vector<VertexNormalGroup> groups;

			void Add(int smoothingGroup, const Vector3f& normal)
			{
				bool found = false;

				for (size_t i = 0; i < groups.size(); ++i)
				{
					if (groups[i].smoothingGroup & smoothingGroup)
					{
						const bool hasExtraGroups = (smoothingGroup & ~groups[i].smoothingGroup) != 0;

						if (hasExtraGroups)
							for (size_t j = i + 1; j < groups.size(); ++j)
								if (groups[j].smoothingGroup & smoothingGroup)
								{
									groups[i].Add(groups[j]);
									groups.erase(groups.begin() + j);
									--j;
								}

						groups[i].Add(smoothingGroup, normal);

						found = true;
						break;
					}
				}

				if (!found)
				{
					VertexNormalGroup group;
					group.Add(smoothingGroup, normal);

					groups.push_back(group);
				}
			}

			void Normalize()
			{
				for (size_t i = 0; i < groups.size(); ++i)
					groups[i].Normalize();
			}

			Vector3f FindNormal(const int smoothingGroup)
			{
				for (size_t i = 0; i < groups.size(); ++i)
					if (groups[i].smoothingGroup & smoothingGroup)
					{
						//for (size_t j = i + 1; j < groups.size(); ++j)
						//{
						//	/*Assert(!(groups[j].smoothingGroup & smoothingGroup));*/

						//}


						return groups[i].normal;
					}
				std::string ss = Format("Couldn't find normal for smoothingGroup %d", smoothingGroup);
				ReportError(ss.c_str());
				//AssertFormatMsg(false, "Couldn't find normal for smoothingGroup %d", smoothingGroup);
				return Vector3f::zero;
			}
		};

	public:
		static void CalculateNormalsFromSmoothingGroups_Legacy(const std::vector<UInt32>& polygonSizes,
			const std::vector<UInt32>& polygons,
			const std::vector<int>& smoothingInfo,
			const std::vector<Vector3f>& vertices,
			std::vector<Vector3f>& normals)
		{
			//std::vector<Vector3f> normals;
			//normals.resize(mesh.polygons.size(), Vector3f::zero);

			std::vector<SmoothVertex> smoothNormals;
			smoothNormals.resize(vertices.size());

			normals.resize(polygons.size(), Vector3f::zero);

			int index = 0;
			for (size_t p = 0; p < polygonSizes.size(); ++p)
			{
				const int polygonSize = polygonSizes[p];
				for (int i = 0; i < polygonSize; ++i)
				{
					int prev = (i > 0 ? i - 1 : polygonSize - 1);
					int next = (i < polygonSize - 1 ? i + 1 : 0);

					int currentVertex = polygons[index + i];

					Vector3f v0 = vertices[polygons[index + prev]];
					Vector3f v1 = vertices[currentVertex];
					Vector3f v2 = vertices[polygons[index + next]];

					Vector3f normal = NormalizeSafe(Cross(v0 - v1, v2 - v1));
					smoothNormals[currentVertex].Add(smoothingInfo[index + i], normal);

					// we will use it if it has no smoothing group at all
					DebugAssert(index + i < normals.size());
					normals[index + i] = normal;
				}

				index += polygonSize;
			}

			for (size_t i = 0; i < smoothNormals.size(); ++i)
				smoothNormals[i].Normalize();

			index = 0;
			for (size_t p = 0; p < polygonSizes.size(); ++p)
			{
				const int polygonSize = polygonSizes[p];
				for (int i = 0; i < polygonSize; ++i)
					// if vertex has smoothing group, then find it
					if (smoothingInfo[index + i] != 0)
					{
						int currentVertex = polygons[index + i];

						normals[index + i] = smoothNormals[currentVertex].FindNormal(smoothingInfo[index + i]);
					}

				index += polygonSize;
			}
		}
	};



}



// Blend shape import
void ConvertFBXShape(const FbxMesh& fbxMesh, const FBXImportMeshSetting& meshSettings, const FBXImportMesh& mesh, const FbxShape& fbxShape, FBXImportBlendShape& shape)
{
	const int shapeVertexCount = fbxShape.GetControlPointsCount();
	const int meshVertexCount = fbxMesh.GetControlPointsCount();

	int polygonCount = fbxMesh.GetPolygonCount();
	const int* indices = fbxMesh.GetPolygonVertices();

	int polygonIndexCount = 0;
	for (unsigned int i = 0; i < mesh.polygonSizes.size(); i++)
		polygonIndexCount += mesh.polygonSizes[i];

	const std::string shapeName = static_cast<const char*>(fbxShape.GetNameWithoutNameSpacePrefix());
	shape.name = shapeName;

	if (shapeVertexCount == meshVertexCount)
	{
		std::vector<int> invalidVertices;
		ImportVertices(fbxShape, &mesh.vertices, shape.vertices, invalidVertices);

		if (!invalidVertices.empty())
		{
			PrintGenericInvalidList(
				Format("Blend shape '%s' on mesh '%s'", fbxShape.GetName(), mesh.name.c_str()),
				10, shape.vertices.size(), invalidVertices, "invalid vertices (NaNs). They will be assigned value (0,0,0)."
			);
		}

		if (meshSettings.legacyComputeAllNormalsFromSmoothingGroupsWhenMeshHasBlendShapes)
		{
			if (!mesh.smoothingGroups.empty())
			{
				NormalCalculation::CalculateNormalsFromSmoothingGroups_Legacy(mesh.polygonSizes, mesh.polygons,
					mesh.smoothingGroups,
					shape.vertices,
					shape.normals);
			}
			else
			{
				std::string ss = Format("Can't generate normals for blend shape '%s' on mesh '%s', mesh has no smoothing groups.",
					fbxShape.GetName(), mesh.name.c_str());
				ReportWarning(ss);
			}
		}
		else if (meshSettings.blendShapeNormalImportMode == kNormalOptionsImport)
		{
			const FbxGeometryElementNormal* elementNormal = fbxShape.GetElementNormal();
			if (elementNormal)
			{
				ExtractWedgeLayerData(elementNormal, shape.normals, indices, polygonIndexCount,
					mesh.polygonSizes,
					polygonCount, meshVertexCount, "normals", shapeName);
			}
			else
			{
				std::string ss = Format("Can't import normals, blend shape '%s' on mesh '%s' doesn't have any.",
					fbxShape.GetName(), mesh.name.c_str());
				ReportWarning(ss);
			}
		}
	}
	else
	{

		std::string ss = Format("Can't import blend shape '%s' on mesh '%s', because vertex count on shape (%d) doesn't match vertex count on mesh (%d).",
			fbxShape.GetName(), mesh.name.c_str(), shapeVertexCount, meshVertexCount);
		ReportError(ss);

		shape.vertices.resize(mesh.vertices.size(), Vector3f::zero);
		shape.normals.resize(mesh.normals.size(), Vector3f::zero);
	}
}




void ConvertFBXShapes(const FbxMesh& fbxMesh, const FBXImportMeshSetting& meshSettings, FBXImportMesh& mesh)
{
	//Assert(mesh.shapes.empty() && mesh.shapeChannels.empty());

	mesh.shapes.reserve(fbxMesh.GetShapeCount());
	mesh.shapeChannels.reserve(fbxMesh.GetShapeCount());

	const int blendShapeDeformerCount = fbxMesh.GetDeformerCount(FbxDeformer::eBlendShape);
	for (int i = 0; i < blendShapeDeformerCount; ++i)
	{
		const FbxBlendShape* blendShapeDeformer = (const FbxBlendShape*)fbxMesh.GetDeformer(i, FbxDeformer::eBlendShape);

		const int blendShapeChannelCount = blendShapeDeformer->GetBlendShapeChannelCount();
		for (int j = 0; j < blendShapeChannelCount; ++j)
		{
			const FbxBlendShapeChannel* blendShapeChannel = blendShapeDeformer->GetBlendShapeChannel(j);
			const int targetShapeCount = blendShapeChannel->GetTargetShapeCount();
			const double* weights = const_cast<FbxBlendShapeChannel*>(blendShapeChannel)->GetTargetShapeFullWeights();

			if (targetShapeCount == 0)
				continue;

			std::string name = GenerateBlendShapeName(blendShapeChannel);

			mesh.shapeChannels.push_back(FBXImportBlendShapeChannel());
			FBXImportBlendShapeChannel& channel = mesh.shapeChannels.back();

			channel.name = name;
			channel.frameIndex = static_cast<int>(mesh.shapes.size());
			channel.frameCount = targetShapeCount;

			for (int k = 0; k < targetShapeCount; ++k)
			{
				const FbxShape* shape = blendShapeChannel->GetTargetShape(k);

				mesh.shapes.push_back(FBXImportBlendShape());
				FBXImportBlendShape& frame = mesh.shapes.back();

				ConvertFBXShape(fbxMesh, meshSettings, mesh, *shape, frame);

				frame.targetWeight = (float)weights[k];
			}
		}
	}
}






void ImportAllBlendShapes(const FBXImportMeshSetting& meshSettings, FBXMeshToInfoMap& meshMap, FBXImportScene& scene)
{
	for (FBXMeshToInfoMap::iterator it = meshMap.begin(); it != meshMap.end(); ++it)
	{
		FbxMesh* fbxMesh = it->first;
		FBXImportMesh& mesh = scene.meshes[it->second.index];

		ConvertFBXShapes(*fbxMesh, meshSettings, mesh);
	}
}

void ImportSkin(FbxMesh* mesh, FBXImportMesh& importMesh, const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap, const FBXMeshToInfoMap& fbxMeshToInfoMap)
{
	FbxSkin* skin = NULL;
	int boneCount = 0;
	if (mesh)
	{
		int skinDeformerCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
		if (skinDeformerCount != 0)
		{
			skin = (FbxSkin*)mesh->GetDeformer(0, FbxDeformer::eSkin);
			boneCount = skin->GetClusterCount();
		}
	}

	if (boneCount == 0)
		skin = NULL;

	// Ignore the skin if the only bone links to the node itself
	if (boneCount == 1)
	{
		FBXMeshToInfoMap::const_iterator it = fbxMeshToInfoMap.find(mesh);
		//Assert(it != fbxMeshToInfoMap.end());

		const FBXSharedMeshInfo& smInfo = it->second;

		FbxCluster* cluster = skin->GetCluster(0);
		if (smInfo.usedByNodes.size() == 1 && cluster != NULL && cluster->GetLink() == smInfo.usedByNodes[0])
			skin = NULL;
	}

	if (skin != NULL)
	{
		importMesh.bones.resize(boneCount);
		importMesh.skin.resize(importMesh.vertices.size(), BoneWeights4());

		std::vector<BoneWeights4>& boneWeights = importMesh.skin;

		for (unsigned int i = 0; i < boneWeights.size(); i++)
		{
			for (int b = 0; b < 4; b++)
			{
				boneWeights[i].weight[b] = 0;
				boneWeights[i].boneIndex[b] = -1;
			}
		}

		std::set<FbxNode*> duplicates;

		int outBoneIndex = 0;
		for (int inbone = 0; inbone < boneCount; inbone++)
		{
			FbxCluster* cluster = skin->GetCluster(inbone);

			std::map<FbxNode*, FBXImportNode*>::const_iterator itFbxNodeMap = fbxNodeMap.find(cluster->GetLink());
			if (cluster == NULL || itFbxNodeMap == fbxNodeMap.end())
			{
				importMesh.bones.resize(importMesh.bones.size() - 1);
				continue;
			}

			bool isDuplicate = (duplicates.count(cluster->GetLink()) != 0);
			FbxString boneName = cluster->GetLink()->GetNameWithoutNameSpacePrefix();
			duplicates.insert(cluster->GetLink());
			importMesh.bones[outBoneIndex].node = itFbxNodeMap->second;

			// Weights and indices
			const double* weights = cluster->GetControlPointWeights();
			const int* indices = cluster->GetControlPointIndices();
			int controlPoints = cluster->GetControlPointIndicesCount();
			bool importWeights = true;

			if (isDuplicate)
			{
				std::string ss = Format("Skipping bone because it is duplicate %s\n", boneName.Buffer());
				ReportWarning(ss);
				importWeights = false;
			}

			/////TODO: we dont support eAdditive ( the mode used for deformers)
			FbxCluster::ELinkMode linkMode = cluster->GetLinkMode();
			if (linkMode == FbxCluster::eAdditive)
			{
				std::string ss = Format("Skipping bone weights for '%s' because it is marked as ADDITIVE. This most likely means that "
					"you are using a blend shape or custom deformer, which is not supported.\n",
					boneName.Buffer());
				ReportWarning(ss);
				importWeights = false;
			}

#if 0
			// AE: I am disabling this sanity check until I get more information from Autodesk about why nodes are
			// popping up w/ different attributes other than eSkeleton
			FbxNodeAttribute* lNodeAttribute = cluster->GetLink()->GetNodeAttribute();
			//          FbxNodeAttribute::EAttributeType parentAttrType = cluster->GetLink()->GetParent()->GetNodeAttribute()->GetAttributeType();
			FbxNodeAttribute::EAttributeType attrType = FbxNodeAttribute::eUNIDENTIFIED;
			if (lNodeAttribute)
				attrType = lNodeAttribute->GetAttributeType();
			switch (attrType)
			{
			case FbxNodeAttribute::eSkeleton:
				// Only import weights for proper bones, not custom deformers.
				break;

			case FbxNodeAttribute::eMesh:
				// I'm not actually sure that a node marked as mesh should be allowed to import as a bone,
				// however, Maya imports it properly, so I guess we should, too.
				break;

			default:
				printf_console("Not importing weights for '%s' because the node attribute type is not marked as eSkeleton, which "
					"means it is not a proper bone.\n", boneName.Buffer());
				//                  ReportWarning( "The attribute type is not a SKELETON!" );
				importWeights = false;
				break;
			}
#endif


			if (importWeights)
			{
				// Go through all control points
				// -  Find and replace the smallest weight with the current weight!
				for (int i = 0; i < controlPoints; i++)
				{
					int vertexIndex = indices[i];
					float weight = static_cast<float>(weights[i]);

					// Weights are sorted (most important weights first)
					for (int j = 0; j < 4; j++)
					{
						// Check out of range vertex index
						if (vertexIndex < 0 || vertexIndex >= static_cast<int>(boneWeights.size()))
							continue;

						if (weight >= boneWeights[vertexIndex].weight[j])
						{
							// Push back old weights!
							for (int k = 2; k >= j; k--)
							{
								boneWeights[vertexIndex].weight[k + 1] = boneWeights[vertexIndex].weight[k];
								boneWeights[vertexIndex].boneIndex[k + 1] = boneWeights[vertexIndex].boneIndex[k];
							}
							// Replace the weights!
							boneWeights[vertexIndex].weight[j] = weight;
							boneWeights[vertexIndex].boneIndex[j] = outBoneIndex;
							break;
						}
					}
				}
			}

			// Bindpose
			importMesh.bones[outBoneIndex].bindpose.SetIdentity();

			FbxAMatrix lClusterGlobalInitPosition, lReferenceGlobalInitPosition, bindPose;
			cluster->GetTransformMatrix(lReferenceGlobalInitPosition);
			cluster->GetTransformLinkMatrix(lClusterGlobalInitPosition);

			bindPose = lClusterGlobalInitPosition.Inverse();
			bindPose *= lReferenceGlobalInitPosition;

			importMesh.bones[outBoneIndex].bindpose = FBXMatrixToMatrix4x4(bindPose);

			outBoneIndex++;
		}

		// only invalid links
		if (importMesh.bones.empty())
		{
			importMesh.skin.clear();
			boneWeights.clear();
		}

		std::vector<int> invalidWeights, invalidBones;
		invalidWeights.reserve(boneWeights.size());
		invalidBones.reserve(boneWeights.size());

		///@TODO: NORMALIZE WEIGHTS IF WE SHOULD!
		/////TODO: we dont support eTOTAL1 - Is this true? It seems that it would already be normalized.
		// Normalize the skin weights!
		for (unsigned int i = 0; i < boneWeights.size(); i++)
		{
			float weightsum = 0.0F;

			for (int j = 0; j < 4; j++)
				weightsum += boneWeights[i].weight[j];

			const float kWeigthEpsilon = 0.00000001F;
			if (weightsum < kWeigthEpsilon)
			{
				for (int j = 0; j < 4; j++)
				{
					if (boneWeights[i].boneIndex[j] != -1)
						boneWeights[i].weight[j] = 1.0F;
				}

				weightsum = 0.0F;
				for (int j = 0; j < 4; j++)
					weightsum += boneWeights[i].weight[j];

				if (weightsum < kWeigthEpsilon)
				{
					invalidBones.push_back(i);

					boneWeights[i].weight[0] = weightsum = 1;
					boneWeights[i].boneIndex[0] = 0;
				}
				else
					invalidWeights.push_back(i);
			}

			weightsum = 1.0F / weightsum;
			for (int j = 0; j < 4; j++)
			{
				boneWeights[i].weight[j] *= weightsum;

				if (boneWeights[i].boneIndex[j] == -1)
					boneWeights[i].boneIndex[j] = 0;
			}
		}

		ValidateSkin(boneWeights, boneCount, importMesh.name.c_str());

		if (!invalidBones.empty())
			PrintInvalidList(importMesh.name, 10, boneWeights.size(), invalidBones, "vertices with no weight and bone assigned (they will be assigned to bone #0 with weight 1)");

		if (!invalidWeights.empty())
			PrintInvalidList(importMesh.name, 10, boneWeights.size(), invalidWeights, "vertices with bone, but no weight assigned (weight will be increased to 1)");
	}
}
