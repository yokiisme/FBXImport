#pragma once

#include <list>
#include <vector>

#include "Utility.h"
#include "EnumFlags.h"
#include "Lighting.h"
#include "Color.h"
#include "ConstrainEnums.h"
#include "ImportMeshDef.h"


struct FBXImportSettings;
struct FBXImportMeshSetting;
struct FBXImportMesh;
struct FBXImportNode;
struct FBXImportBone;
struct FBXImportBlendShape;
struct FBXImportBlendShapeChannel;
struct FBXImportBaseAnimation;
struct FBXImportNodeAnimation;
struct FBXImportFloatAnimation;
struct FBXImportAnimationClip;
struct FBXImportCustomFloatAnimation;
struct FBXImportSceneInfo;
struct FBXImportScene;
struct FBXImportMesh;
struct FBXMesh;
struct FBXGameObject;
struct ImportNodeUserData;
struct FBXTriangulationData;
struct TSpaceMeshInfo;
struct FBXImportNodeUserData;




enum WeightedMode
{
	kNotWeighted = 0,
	kInWeighted = 1 << 0,
	kOutWeighted = 1 << 1,
	kBothWeighted = kInWeighted | kOutWeighted
};
ENUM_FLAGS(WeightedMode);


enum InternalWrapMode
{
	// Values are serialized in CompressedMesh and must be preserved
	kInternalWrapModePingPong = 0,
	kInternalWrapModeRepeat = 1,
	kInternalWrapModeClamp = 2,
	kInternalWrapModeDefault = 3
};

enum WrapMode
{
	// Must be kept in sync with WrapMode in InternalUtility.bindings
	kWrapModeDefault = 0,
	kWrapModeClamp = 1 << 0,
	kWrapModeRepeat = 1 << 1,
	kWrapModePingPong = 1 << 2,
	kWrapModeClampForever = 1 << 3
};

enum AnimationCurveType
{
	kFloatCurve = 0,
	kVector3Curve = 1,
	kQuaternionCurve = 2
};

enum  RotationImportType
{
	kRotationImportQuat = 0,
	kRotationImportEuler
};

enum CurveFlags
{
	kNone = 0,
	kConvertEuler = 1 << 0,
	kHasNaNs = 1 << 1,
	kRotationOrderXYZ = 1 << (2 + eEulerXYZ),
	kRotationOrderXZY = 1 << (2 + eEulerXZY),
	kRotationOrderYZX = 1 << (2 + eEulerYZX),
	kRotationOrderYXZ = 1 << (2 + eEulerYXZ),
	kRotationOrderZXY = 1 << (2 + eEulerZXY),
	kRotationOrderZYX = 1 << (2 + eEulerZYX),
	kRotationOrderSphericXYZ = 1 << (2 + eSphericXYZ),
	kRotationFlags = kRotationOrderXYZ | kRotationOrderXZY | kRotationOrderYZX | kRotationOrderYXZ | kRotationOrderZXY | kRotationOrderZYX | kRotationOrderSphericXYZ,
};
ENUM_FLAGS(CurveFlags);

enum RotationCurveImportOptions
{
	kNoRotation,
	kImportQuaternion,
	kImportEulerRadians,
	kImportEulerDegrees
};

struct AnimationSettings
{
	bool adjustClipsByTimeRange;
	bool importBlendShapes;
	bool importAnimatedCustomProperties;
	bool importCameras;
	bool importLights;
	bool importVisibility;
	bool importConstraints;
	bool importMaterials;
	int animationOversampling;
	double sampleRate;
};

struct AnimationTimeRange
{
	float timeRangeStart, timeRangeEnd, timeRangeOffset;

	double bakeStart;
	double bakeStop;
};



struct FBXImportSettings
{
	FBXImportSettings()
		: importBlendShapes(true)
		, importNormals(false)
		, importTangents(false)
		, importVisibility(false)
		, importSkinMesh(false)
		, resampleCurves(false)
		, importCameras(false)
		, importLights(false)
		, importConstraints(false)
		, importMaterials(false)
	{}

	int importAnimations;
	int adjustClipsByTimeRange;
	const char* absolutePath;
	const char* originalExtension;
	std::string textureImportFolder;

	bool importBlendShapes;
	bool importCameras;
	bool importLights;
	bool importSkinMesh;
	bool importNormals;
	bool importTangents;
	bool importVisibility;
	bool importMaterials;
	bool resampleCurves;
	bool useFileScale;
	bool importAnimatedCustomProperties;
	bool importConstraints;
};

struct	UnwrapParam
{
	float   angleDistortionThreshold;
	float   areaDistortionThreshold;
	float   hardAngle;

	float   packMargin;

	// WARNING: set it only when providing data from Mesh (scripting access)
	int32_t recollectVertices;

	void    Reset()
	{
		angleDistortionThreshold = 0.08f;
		areaDistortionThreshold = 0.15f;
		hardAngle = 88.0f;
		packMargin = 1.0f / 256.0f;

		recollectVertices = 0;
	}

	UnwrapParam() { Reset(); }
};

struct FBXImportMeshSetting
{
	
	bool importNormal;
	bool importTangent;

	//
	bool    optimizeMesh; // default true;
	bool    weldVertices;// true (default)
	bool    invertWinding;// false (default)
	bool    swapUVChannels; // false
	bool    generateSecondaryUV; //false

	// padding goes here due to align fail
	float   secondaryUVAngleDistortion;
	float   secondaryUVAreaDistortion;
	float   secondaryUVHardAngle;
	float   secondaryUVPackMargin;

	// Tangent space settings
	NormalOptions normalImportMode;
	NormalCalculationOptions normalCalculationMode;
	TangentSpaceOptions tangentImportMode;
	NormalSmoothingSourceOptions normalSmoothingSource;
	float   normalSmoothAngle;  /// 60 (default)
	NormalOptions blendShapeNormalImportMode;

	ModelImporterIndexFormat    indexFormat;

	bool    keepQuads; // default false

	bool legacyComputeAllNormalsFromSmoothingGroupsWhenMeshHasBlendShapes;

	FBXImportMeshSetting():
		optimizeMesh(true),
		weldVertices(true),
		invertWinding(false),
		swapUVChannels(false),
		generateSecondaryUV(false),
		normalImportMode(NormalOptions::kNormalOptionsImport),
		normalCalculationMode(NormalCalculationOptions::kNormalCalculationOptionsAreaAndAngleWeighted),
		blendShapeNormalImportMode(NormalOptions::kNormalOptionsCalculate),
		normalSmoothingSource(NormalSmoothingSourceOptions::kNormalSmoothingSourceOptionsPreferSmoothingGroups),
		tangentImportMode(TangentSpaceOptions::kTangentSpaceOptionsCalculateMikk),
		normalSmoothAngle(60.0F),
		indexFormat(ModelImporterIndexFormat::kMIIndexFormatAuto),
		keepQuads(false),
		legacyComputeAllNormalsFromSmoothingGroupsWhenMeshHasBlendShapes(false)
	{
		UnwrapParam defaultUnwrapParam;
		defaultUnwrapParam.Reset();

		secondaryUVAngleDistortion = 100.0f * defaultUnwrapParam.angleDistortionThreshold;
		secondaryUVAreaDistortion = 100.0f * defaultUnwrapParam.areaDistortionThreshold;
		secondaryUVHardAngle = defaultUnwrapParam.hardAngle;
		secondaryUVPackMargin = 1024.0f * defaultUnwrapParam.packMargin;
	};


	bool ImportSmoothingGroups() const
	{
		return legacyComputeAllNormalsFromSmoothingGroupsWhenMeshHasBlendShapes ||
			((normalImportMode != kNormalOptionsNone || blendShapeNormalImportMode != kNormalOptionsNone)
				&& (normalSmoothingSource == kNormalSmoothingSourceOptionsFromSmoothingGroups || normalSmoothingSource == kNormalSmoothingSourceOptionsPreferSmoothingGroups));
	};
	bool SmoothingGroupsRequiredForImport() const
	{
		return (normalImportMode == kNormalOptionsCalculate || blendShapeNormalImportMode == kNormalOptionsCalculate)
			&& normalSmoothingSource == kNormalSmoothingSourceOptionsFromSmoothingGroups;
	}
};
struct BoneWeights4
{
	float weight[4];
	int   boneIndex[4];
};


struct FBXImportMesh
{
	enum { kMaxUVs = 2 }; // Unity-8
	std::vector<Vector3f>      vertices;
	std::vector<BoneWeights4>  skin; // per vertex skin info
	std::vector<Vector3f>      normals;
	std::vector<Vector4f>      tangents;
	std::vector<ColorRGBA32>   colors;
	std::vector<Vector2f>      uvs[kMaxUVs];
	std::vector<int>           smoothingGroups;

	std::vector<uint32_t>      polygons;
	// Per-polygon arrays.
	std::vector<uint32_t>      polygonSizes; // Size of each polygon
	std::vector<uint32_t>      materials; // Material index of each polygon

	std::string                name;
	std::vector<FBXImportBone>    bones;
	std::vector<FBXImportBlendShape>   shapes;
	std::vector<FBXImportBlendShapeChannel> shapeChannels;

	bool					hasAnyQuads;

	FBXImportMesh() : hasAnyQuads(false) {}
	void Reserve(int vertexCount, int faceCount, const FBXImportMesh* src)
	{
		if (src)
		{
			if (!src->polygons.empty())
				polygons.reserve(faceCount * 3);
			if (!src->polygonSizes.empty())
				polygonSizes.reserve(faceCount);
			if (!src->materials.empty())
				materials.reserve(faceCount);

			if (!src->vertices.empty())
				vertices.reserve(vertexCount);
			if (!src->normals.empty())
				normals.reserve(vertexCount);
			if (!src->tangents.empty())
				tangents.reserve(vertexCount);

			for (int uvIndex = 0; uvIndex < 2; uvIndex++)
			{
				if (!src->uvs[uvIndex].empty())
					uvs[uvIndex].reserve(vertexCount);
			}
		}
		else
		{
			polygons.reserve(faceCount * 3);
			polygonSizes.reserve(faceCount);
			materials.reserve(faceCount);
			vertices.reserve(vertexCount);
			normals.reserve(vertexCount);
			tangents.reserve(vertexCount);
			for (int uvIndex = 0; uvIndex < 2; uvIndex++)
				uvs[uvIndex].reserve(vertexCount);
		}

		if (src)
		{
			if (!src->polygons.empty())
				polygons.reserve(faceCount * 3);
			if (!src->polygonSizes.empty())
				polygonSizes.reserve(faceCount);
			if (!src->materials.empty())
				materials.reserve(faceCount);

			if (!src->vertices.empty())
				vertices.reserve(vertexCount);
			if (!src->skin.empty())
				skin.reserve(vertexCount);
			if (!src->normals.empty())
				normals.reserve(vertexCount);
			if (!src->tangents.empty())
				tangents.reserve(vertexCount);
			if (!src->colors.empty())
				colors.reserve(vertexCount);
			for (int uvIndex = 0; uvIndex < FBXImportMesh::kMaxUVs; uvIndex++)
			{
				if (!src->uvs[uvIndex].empty())
					uvs[uvIndex].reserve(vertexCount);
			}
		}
		else
		{
			polygons.reserve(faceCount * 3);
			polygonSizes.reserve(faceCount);
			materials.reserve(faceCount);

			vertices.reserve(vertexCount);
			skin.reserve(vertexCount);
			normals.reserve(vertexCount);
			tangents.reserve(vertexCount);
			colors.reserve(vertexCount);
			for (int uvIndex = 0; uvIndex < FBXImportMesh::kMaxUVs; uvIndex++)
				uvs[uvIndex].reserve(vertexCount);
		}
	}
};

struct FBXImportNode
{
	std::string name;
	Vector3f    position; // zero (default)
	Quaternionf rotation; // identity (default)
	Vector3f    scale; // identity (default)
	Matrix4x4f  meshTransform;// identity (default)
	int         cameraIndex;
	int         lightIndex;
	int         meshIndex;// -1 default
	bool        visibility;
	std::vector<FBXImportNode> children;

	std::vector<FBXImportNodeUserData> userData;
	std::vector<FBXImportCustomFloatAnimation> animatedCustomProperties;
};

struct FBXImportBone
{
	// The node this bone uses to deform the mesh!
	FBXImportNode* node;
	Matrix4x4f bindpose;

	FBXImportBone() { node = NULL; bindpose.SetIdentity(); }
};

struct FBXImportBlendShape
{
	float                 targetWeight;

	// vertices, normals and tangents are stored as non-delta data, i.e. plain object-space just as mesh data
	std::vector<Vector3f> vertices;
	std::vector<Vector3f> normals;
	std::vector<Vector3f> tangents;

	std::string          name;
};

struct FBXImportBlendShapeChannel
{
	std::string name;

	// index into shapes
	int frameIndex;
	int frameCount;
};

struct FBXImportBaseAnimation
{
	FBXImportNode* node;
};

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

};

template<class T>
void FBXAnimationCurveTpl<T>::InvalidateCache()
{
	m_Cache.time = std::numeric_limits<float>::infinity();
	m_Cache.index = 0;
	m_ClampCache.time = std::numeric_limits<float>::infinity();
	m_ClampCache.index = 0;
}


typedef FBXAnimationCurveTpl<float>        FBXAnimationCurveBase;
typedef FBXAnimationCurveTpl<float>        FBXAnimationCurve;
typedef FBXAnimationCurveTpl<Quaternionf>  FBXAnimationCurveQuat;
typedef FBXAnimationCurveTpl<Vector3f>     FBXAnimationCurveVec3;



struct FBXImportNodeAnimation : public FBXImportBaseAnimation
{
	RotationImportType rotationType;
	RotationOrder rotationOrder;
	FBXAnimationCurve rotation[4];
	FBXAnimationCurve translation[3];
	FBXAnimationCurve scale[3];
};
typedef std::list<FBXImportNodeAnimation> FBXImportNodeAnimations;

struct FBXImportFloatAnimation : public FBXImportBaseAnimation
{
	FBXAnimationCurve curve;
	std::string className;
	std::string propertyName;
};
typedef std::list<FBXImportFloatAnimation> FBXImportFloatAnimations;

struct FBXImportCustomFloatAnimation : public FBXImportFloatAnimation
{
	FBXImportAnimationClip* clip;
};

struct FBXImportAnimationClip
{
	std::string					name;
	double						bakeStart;
	double						bakeStop;

	FBXImportNodeAnimations		nodeAnimations;
	FBXImportFloatAnimations	floatAnimations;

	bool HasAnimations() { return !nodeAnimations.empty() || !floatAnimations.empty(); }
};

struct FBXImportCameraComponent
{
	bool orthographic;
	float orthographicSize;
	float fieldOfView;
	float nearPlane;
	float farPlane;
	int gateFitMode;
	FBXImportNode* target;
	FBXImportNode* up;
	FBXImportNode* owner;
	float roll;
	Vector2f sensorSize;
	Vector2f lensShift;
	float focalLength;
	bool usePhysicalProperties;
};

struct FBXImportLightComponent
{
	LightType type;
	float rangeStart;
	float rangeEnd;
	float spotAngle;
	float intensity;
	ColorRGBAf color;
	bool castShadows;
};

struct FBXImportConstraint
{
	FBXImportConstraint() : kind(kConstraintKindUndefined)
		, weight(0.0)
		, target(NULL)
		, translation(Vector3f::zero)
		, rotation(Vector3f::zero)
		, scaling(Vector3f::one)
		, aimVector(0.0f, 0.0f, 1.0f)
		, upVector(0.0f, 1.0f, 0.0f)
		, worldUpVector(0.0f, 1.0f, 0.0f)
		, upObject(NULL)
		, upType(kWorldUpTypeSceneUp)
		, affectTranslationX(true)
		, affectTranslationY(true)
		, affectTranslationZ(true)
		, affectRotationX(true)
		, affectRotationY(true)
		, affectRotationZ(true)
		, affectScalingX(true)
		, affectScalingY(true)
		, affectScalingZ(true)
		, isActive(true)
		, isLocked(false)
	{
	};

	std::string name; ///< The name of the constraint node, used for logging errors only

	ConstraintKind kind;
	float weight;

	const FBXImportNode* target;
	std::vector<const FBXImportNode*> sources;
	std::vector<float> sourceWeights;
	std::vector<Vector3f> sourceTranslationOffsets;
	std::vector<Vector3f> sourceRotationOffsets;

	Vector3f    translation;    ///< Translation offset
	Vector3f    rotation;       ///< Rotation offset
	Vector3f    scaling;        ///< Scaling factor

	Vector3f    aimVector;
	Vector3f    upVector;
	Vector3f    worldUpVector;
	const FBXImportNode* upObject;
	WorldUpType upType;

	bool affectTranslationX;
	bool affectTranslationY;
	bool affectTranslationZ;

	bool affectRotationX;
	bool affectRotationY;
	bool affectRotationZ;

	bool affectScalingX;
	bool affectScalingY;
	bool affectScalingZ;

	bool isActive;
	bool isLocked;
};

struct FBXImportSceneInfo
{
	std::string applicationName;
	std::string applicationDetailedName;
	std::string exporterInfo;
	bool hasApplicationName;
	bool hasEmbeddedTextures;
	bool hasSkeleton;
	float fileScale;
	std::string fileScaleUnit;
	float fileScaleFactor;
	float sampleRate;

	FBXImportSceneInfo()
		: fileScale(1.0f)
		, fileScaleUnit("")
		, fileScaleFactor(1.0f)
		, hasApplicationName(false)
		, hasSkeleton(false)
		, hasEmbeddedTextures(false)
	{}
};

struct FBXImportScene
{
	float														fileScaleFactor;
	float                                                       sampleRate;
	std::string													fileScaleUnit;
	std::vector<FBXImportNode>									nodes;
	std::vector<FBXImportMesh>									meshes;
	std::vector<std::string>									materials;
	std::vector<FBXImportAnimationClip>                         animationClips;
	std::vector<FBXImportCameraComponent>                       cameras;
	std::vector<FBXImportLightComponent>                        lights;
	std::vector<FBXImportConstraint>                            constraints;
	FBXImportSceneInfo                                          sceneInfo;
};

struct FBXMesh
{
public:
	FBXMesh() {}
	char* name;
	std::vector<Vector3f> vertices;
	std::vector<Vector3f> normals;
	std::vector<Vector4f> tangents;
	std::vector<Vector2f> uv1;
	std::vector<Vector2f> uv2;
	std::vector<uint32_t> indicesize;
	std::vector<uint32_t> topologytype;
	std::vector<uint32_t> indices;
	std::vector<std::string> materials;
	std::vector<uint32_t> materialindex;
};

struct FBXGameObject
{
public:
	int meshCount;
	std::vector<FBXMesh> meshList;
};

#pragma region mesh

struct FBXTriangulationData
{
	const FBXImportMesh* input;
	FBXImportMesh* output;
	const FBXImportMeshSetting* setting;
};

struct TSpaceMeshInfo
{
	uint32_t* FaceOffs;
	FBXImportMesh* ImpMesh;

	const Vector3f* vertices;
	const Vector3f* normals;
	Vector4f* outTangents;
};
#pragma endregion

struct FBXSharedMeshInfo
{
	int index;
	std::vector<FbxNode*> usedByNodes;
};
typedef std::map<FbxMesh*, FBXSharedMeshInfo> FBXMeshToInfoMap;


enum UserDataType
{
	kUserDataBool,
	kUserDataFloat,
	kUserDataColor,
	kUserDataInt,
	kUserDataVector,
	kUserDataString
};

struct FBXImportNodeUserData
{
	std::string name;

	UserDataType data_type_indicator;

	//data
	bool boolData;
	Vector4f vectorData;
	std::string stringData;
	ColorRGBA32 colorData;
	int intData;
	float floatData;
};

