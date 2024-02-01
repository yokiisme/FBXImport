#pragma once
#include "fbxsdk/core/math/fbxmath.h"
#include "RotationOrder.h"
#include "Color.h"
#include "stringFormat.h"
#include "Matrix.h"


#define M_DEG_2_RAD  0.0174532925f
#define M_RAD_2_DEG  57.295779513f

inline FbxVector4 Vector3ToFBXPoint(const Vector3f& vec)
{
	return FbxVector4(vec[0], vec[1], vec[2]);
}

inline float FBXEulerRemap(const float value, int coordinateIndex)
{
	return coordinateIndex == 1 || coordinateIndex == 2 ? -value : value;
}

//The order of quaternion multiplications is different in Unity and FBX, so we need to invert the order
inline RotationOrder FbxOrderToUnityOrder(EFbxRotationOrder rotationOrder)
{
    return (RotationOrder)(int)rotationOrder;
}

inline std::string GenerateBlendShapeName(const FbxBlendShapeChannel* blendShapeChannel)
{
	const int targetShapeCount = blendShapeChannel->GetTargetShapeCount();
	std::string name = blendShapeChannel->GetName();
	if (name.empty() && targetShapeCount > 0)
	{
		// Choose name of first target shape if any
		const FbxShape* shape = blendShapeChannel->GetTargetShape(0);
		FbxString fbxName = shape->GetName();
		name = static_cast<const char*>(fbxName);

		// If name ends with "Mesh", suppress it
		size_t pos = name.find("Mesh");
		if (pos > 0 && pos == (name.length() - 4))
		{
			name = name.substr(0, pos);
		}
	}
	if (name.empty())
	{
		Format("No name (%d)", blendShapeChannel->GetUniqueID());
	}

	return name;
}

inline std::string GenerateBlendShapeAnimationAttribute(const FbxBlendShapeChannel* blendShapeChannel)
{
	return "blendShape." + GenerateBlendShapeName(blendShapeChannel);
}

inline bool IsSet(const FbxProperty& prop)
{
	return prop.IsValid() && prop.Modified();
}

inline bool ConvertFBXDoubleProperty(const FbxPropertyT<FbxDouble>& prop, float& value, const float defaultValue)
{
	const bool isSet = IsSet(prop);
	value = isSet ? static_cast<float>(prop.Get()) : defaultValue;
	return isSet;
}

inline bool ConvertFBXColor4Property(const FbxPropertyT<FbxDouble4>& prop, ColorRGBAf& color)
{
	if (!prop.IsValid())
		return false;
	else
	{
		FbxDouble4 value = prop.Get();
		color.Set(
			static_cast<float>(value[0]),
			static_cast<float>(value[1]),
			static_cast<float>(value[2]),
			static_cast<float>(value[3])
		);
		return true;
	}
}

inline bool ConvertFBXColor3Property(const FbxPropertyT<FbxDouble3>& prop, ColorRGBAf& color)
{
	if (!prop.IsValid())
		return false;
	else
	{
		FbxDouble3 value = prop.Get();
		color.Set(
			static_cast<float>(value[0]),
			static_cast<float>(value[1]),
			static_cast<float>(value[2]),
			1.f
		);
		return true;
	}
}

// FbxProperty agnositic way to convert to ColorRGBAf. It handles
// both Maya & 3DsMax different vector sizes (respectively internally represented
// as a FbxColor3 & a FbxColor4).
inline bool ConvertFBXColorProperty(const FbxProperty& prop, ColorRGBAf& color)
{
	if (FbxDouble3DT == prop.GetPropertyDataType() || FbxColor3DT == prop.GetPropertyDataType())
		return ConvertFBXColor3Property(prop, color);
	else if (FbxDouble4DT == prop.GetPropertyDataType() || FbxColor4DT == prop.GetPropertyDataType())
		return ConvertFBXColor4Property(prop, color);
	return false;
}

inline bool ConvertFBXColorProperty(const FbxProperty& prop, ColorRGBAf& color, const ColorRGBAf& defaultValue)
{
	if (!ConvertFBXColorProperty(prop, color))
	{
		///@TODO: This looks very strange...  Investigate...
		return true;
		color = defaultValue;
	}
	else
		return false;
}


inline bool ConvertFBXBoolProperty(const FbxPropertyT<FbxBool>& prop, bool& value, const bool defaultValue)
{
	const bool isSet = IsSet(prop);
	value = isSet ? prop.Get() : defaultValue;
	return isSet;
}

// Keep logic of this function the same as function above
inline float FBXCoordinateRemap(const float value, int coordinateIndex)
{
	return coordinateIndex == 0 ? -value : value;
}

inline Matrix4x4f FBXMatrixToMatrix4x4(FbxAMatrix before)
{
	Matrix4x4f converted;
	const double* beforePtr = before;
	converted[0] = static_cast<float>(beforePtr[0]);
	converted[1] = static_cast<float>(-beforePtr[1]);
	converted[2] = static_cast<float>(-beforePtr[2]);
	converted[3] = static_cast<float>(beforePtr[3]);

	converted[4] = static_cast<float>(-beforePtr[4]);
	converted[5] = static_cast<float>(beforePtr[5]);
	converted[6] = static_cast<float>(beforePtr[6]);
	converted[7] = static_cast<float>(beforePtr[7]);

	converted[8] = static_cast<float>(-beforePtr[8]);
	converted[9] = static_cast<float>(beforePtr[9]);
	converted[10] = static_cast<float>(beforePtr[10]);
	converted[11] = static_cast<float>(beforePtr[11]);

	converted[12] = static_cast<float>(-beforePtr[12]);
	converted[13] = static_cast<float>(beforePtr[13]);
	converted[14] = static_cast<float>(beforePtr[14]);
	converted[15] = static_cast<float>(beforePtr[15]);

	return converted;
}


inline float radians(float deg)
{
	return M_DEG_2_RAD * deg;
}

inline float degrees(float rad)
{
	return M_RAD_2_DEG * rad;
}