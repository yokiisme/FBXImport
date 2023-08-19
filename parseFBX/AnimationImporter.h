#pragma once

#include "Utility.h"
#include "FBXImporterDef.h"
#include "Color.h"
#include "Lighting.h"
#include <map>
#include <string.h>
#include <sstream>

void ImportAnimationClipsInfo(FbxScene& scene, FBXImportScene& importScene);
void ImportAllAnimations(FbxManager& sdkManager,
    FbxScene& fbxScene,
    FbxNode* root,
    FBXImportScene& unity,
    const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap,
    const FBXMeshToInfoMap& fbxMeshToInfoMap,
    const AnimationSettings& animationSettings);
void SyncCurveNodeChannelValue(FbxNode* node);
void RemoveInvalidTransformCurves(FbxScene& fbxScene, FbxNode* root);

struct NANAnimationInfo
{
	NANAnimationInfo() : hasNANsPos(false), hasNANsRot(false), hasNANsScale(false), hasNANsBlendShape(false), hasNANsCamera(false), hasNANsLight(false) {}

	bool HasNAN() const { return hasNANsPos || hasNANsRot || hasNANsScale || hasNANsBlendShape || hasNANsCamera || hasNANsLight; }
	void Accumulate(const NANAnimationInfo& other)
	{
		hasNANsPos = hasNANsPos || other.hasNANsPos;
		hasNANsRot = hasNANsRot || other.hasNANsRot;
		hasNANsScale = hasNANsScale || other.hasNANsScale;
		hasNANsBlendShape = hasNANsBlendShape || other.hasNANsBlendShape;
		hasNANsCamera = hasNANsCamera || other.hasNANsCamera;
		hasNANsLight = hasNANsLight || other.hasNANsLight;
	}

	std::string GetHelperString() const
	{
		const std::string infos[] =
		{
			!hasNANsPos ? "" : "P - position",
			!hasNANsRot ? "" : "R - rotation",
			!hasNANsScale ? "" : "S - scale",
			!hasNANsBlendShape ? "" : "B - BlendShape curves",
			!hasNANsCamera ? "" : "C - Camera curves",
			!hasNANsLight ? "" : "L - Light curves",
		};

		std::ostringstream str;
		for (int i = 0; i < 5; ++i)
			if (!infos[i].empty())
			{
				if (str.tellp() > 0)
					str << ", ";
				str << infos[i];
			}

		return str.str();
	}

	std::string GetString() const
	{
		std::ostringstream str;
		if (hasNANsPos) str << "P";
		if (hasNANsRot) str << "R";
		if (hasNANsScale) str << "S";
		if (hasNANsBlendShape) str << "B";
		if (hasNANsCamera) str << "C";
		if (hasNANsLight) str << "L";

		return str.str();
	}

	std::string nodeName;
	bool hasNANsPos, hasNANsRot, hasNANsScale, hasNANsBlendShape, hasNANsCamera, hasNANsLight;
};

typedef std::vector<NANAnimationInfo> NANAnimationInfos;



//Declaration
void ImportAllAnimations(FbxManager& sdkManager, FbxScene& fbxScene, FbxNode* root,	FBXImportScene& unity, const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap,
	const FBXMeshToInfoMap& fbxMeshToInfoMap, const AnimationSettings& animationSettings);

static void RecursiveImportAnimation(FbxNode* node, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, FbxScene& fbxScene,
	const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap, const FBXMeshToInfoMap& fbxMeshToInfoMap, const AnimationTimeRange& range,
	NANAnimationInfos& nanAnimationInfos, const AnimationSettings& animationSettings);

static void ImportAnimationTake(FbxNode* node, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, FbxScene& fbxScene,
	const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap, const FBXMeshToInfoMap& fbxMeshToInfoMap, const AnimationTimeRange& range,
	NANAnimationInfos& nanAnimationInfos, const AnimationSettings& animationSettings);

void ImportAnimationClipsInfo(FbxScene& scene, FBXImportScene& importScene);

void ImportConstraintPropertyAnimations(FbxScene& fbxScene, FbxNode* node, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip,
	const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags);

void ImportShapesAnimation(FbxNode* node, FBXImportNode& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags, const FBXMeshToInfoMap& fbxMeshToInfoMap);

void ImportCameraAnimations(FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags);

void ImportLightAnimations(FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags);

void ImportCustomPropertyAnimations(FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags);

void ImportParentConstraintOffsets(FbxConstraintParent* constraint, FbxNode* node, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip,
	const AnimationTimeRange& range, const AnimationSettings& animationSettings, CurveFlags& flags);
void ImportMaterialPropertyAnimations(FbxScene& fbxScene, FbxNode* fbxNode, FBXImportNode*& importNode, FbxAnimLayer& animLayer, FBXImportAnimationClip& clip, const AnimationTimeRange& range,
	const AnimationSettings& animationSettings, CurveFlags& flags);

void ImportAllAnimations(FbxManager& sdkManager, FbxScene& fbxScene, FbxNode* root, FBXImportScene& unity, const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap, const FBXMeshToInfoMap& fbxMeshToInfoMap, const AnimationSettings& animationSettings);
void ImportAllBlendShapes(const FBXImportMeshSetting& meshSettings, FBXMeshToInfoMap& meshMap, FBXImportScene& scene);
void ImportSkin(FbxMesh* mesh, FBXImportMesh& importMesh, const std::map<FbxNode*, FBXImportNode*>& fbxNodeMap, const FBXMeshToInfoMap& fbxMeshToInfoMap);
void ImportAnimationClipsInfo(FbxScene& scene, FBXImportScene& importScene);