#include "Constraints.h"

static const FBXImportNode* FindNodeByName(const std::string& name, const std::vector<FBXImportNode>& nodes)
{
	for (std::vector<FBXImportNode>::const_iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt)
	{
		if (name == nodeIt->name)
		{
			return &(*nodeIt);
		}

		const FBXImportNode* node = FindNodeByName(name, nodeIt->children);
		if (node != NULL)
		{
			return node;
		}
	}

	return NULL;
}

template<typename TConstraint>
static void ImportSources(const TConstraint* fbxConstraint, const FBXImportScene& scene, FBXImportConstraint& unityConstraint)
{
	for (int i = 0, c = fbxConstraint->GetConstraintSourceCount(); i < c; ++i)
	{
		FbxObject* source = fbxConstraint->GetConstraintSource(i);
		std::string sourceName = source->GetName();

		const FBXImportNode* node = FindNodeByName(sourceName, scene.nodes);
		if (node != NULL)
		{
			unityConstraint.sources.push_back(node);
			unityConstraint.sourceWeights.push_back(fbxConstraint->GetSourceWeight(source));
		}
	}
}

template<>
void ImportSources<FbxConstraintParent>(const FbxConstraintParent* fbxConstraint, const FBXImportScene& scene, FBXImportConstraint& unityConstraint)
{
	for (int i = 0, c = fbxConstraint->GetConstraintSourceCount(); i < c; ++i)
	{
		FbxObject* source = fbxConstraint->GetConstraintSource(i);
		std::string sourceName = source->GetName();

		const FBXImportNode* node = FindNodeByName(sourceName, scene.nodes);
		if (node != NULL)
		{
			unityConstraint.sources.push_back(node);
			unityConstraint.sourceWeights.push_back(fbxConstraint->GetSourceWeight(source));
			unityConstraint.sourceTranslationOffsets.push_back(FBXPointToVector3Remap(fbxConstraint->GetTranslationOffset(source)));

			// for some reason we get the euler angles in a Vector4...
			FbxVector4 rotOffset = fbxConstraint->GetRotationOffset(source);
			unityConstraint.sourceRotationOffsets.push_back(Vector3f(rotOffset[0], rotOffset[1], rotOffset[2]));
		}
	}
}

template<typename TConstraint>
static void ImportCommonConstraintProperties(const TConstraint* fbxConstraint, const FBXImportScene& scene, FBXImportConstraint& unityConstraint)
{
	unityConstraint.name = fbxConstraint->GetName();

	if (FbxObject* targetObject = fbxConstraint->GetConstrainedObject())
	{
		std::string name = targetObject->GetName();
		unityConstraint.target = FindNodeByName(name, scene.nodes);

		ImportSources(fbxConstraint, scene, unityConstraint);
	}
}

bool IsConstraintTypeSupported(FbxConstraint::EType constraintType)
{
	switch (constraintType)
	{
	case FbxConstraint::eAim:
	case FbxConstraint::ePosition:
	case FbxConstraint::eRotation:
	case FbxConstraint::eScale:
	case FbxConstraint::eParent:
		return true;
	default:
		return false;
	}
}

void ImportConstraints(FbxManager& sdkManager, FbxScene& fbxScene, FbxNode* root, FBXImportScene& unityScene)
{
	for (int i = 0, c = fbxScene.GetSrcObjectCount<FbxConstraint>(); i < c; ++i)
	{
		FbxConstraint* constraint = fbxScene.GetSrcObject<FbxConstraint>(i);
		if (!constraint)
			continue;

		if (!IsConstraintTypeSupported(constraint->GetConstraintType()))
			continue;

		FBXImportConstraint unityConstraint;

		unityConstraint.kind = static_cast<ConstraintKind>(constraint->GetConstraintType());

		unityConstraint.weight = constraint->Weight;
		unityConstraint.isActive = constraint->Active;
		// The Locked property should not be imported, since it is irrelevant if the constraint is locked or not in the DCC.
		// Instead, we want to always default to isLocked == true in Unity in order to preserve the offsets of the imported asset
		unityConstraint.isLocked = true;

		switch (unityConstraint.kind)
		{
		case kConstraintKindAim:
		{
			FbxConstraintAim* constraintAim = static_cast<FbxConstraintAim*>(constraint);

			unityConstraint.affectRotationX = constraintAim->AffectX;
			unityConstraint.affectRotationY = constraintAim->AffectY;
			unityConstraint.affectRotationZ = constraintAim->AffectZ;

			unityConstraint.upType = static_cast<WorldUpType>((int)constraintAim->WorldUpType);
			FbxDouble3 aimVector = constraintAim->AimVector;
			unityConstraint.aimVector = FBXPointToVector3Remap(aimVector);
			FbxDouble3 upVector = constraintAim->UpVector;
			unityConstraint.upVector = Vector3f(upVector[0], upVector[1], upVector[2]);
			FbxDouble3 worldUpVector = constraintAim->WorldUpVector;
			unityConstraint.worldUpVector = Vector3f(worldUpVector[0], worldUpVector[1], worldUpVector[2]);

			FbxObject* worldUpObject = constraintAim->GetWorldUpObject();
			if (worldUpObject)
			{
				std::string objName = worldUpObject->GetName();
				unityConstraint.upObject = FindNodeByName(objName, unityScene.nodes);
			}

			FbxDouble3 fbxEuler = constraintAim->RotationOffset.EvaluateValue(0);
			unityConstraint.rotation = Vector3f(fbxEuler[0], fbxEuler[1], fbxEuler[2]);

			ImportCommonConstraintProperties(constraintAim, unityScene, unityConstraint);
		}
		break;
		case kConstraintKindPosition:
		{
			FbxConstraintPosition* constraintPostion = static_cast<FbxConstraintPosition*>(constraint);
			unityConstraint.translation = FBXPointToVector3Remap(constraintPostion->Translation.EvaluateValue(0));

			unityConstraint.affectTranslationX = constraintPostion->AffectX;
			unityConstraint.affectTranslationY = constraintPostion->AffectY;
			unityConstraint.affectTranslationZ = constraintPostion->AffectZ;

			ImportCommonConstraintProperties(constraintPostion, unityScene, unityConstraint);
		}
		break;
		case kConstraintKindRotation:
		{
			FbxConstraintRotation* constraintRotation = static_cast<FbxConstraintRotation*>(constraint);

			unityConstraint.affectRotationX = constraintRotation->AffectX;
			unityConstraint.affectRotationY = constraintRotation->AffectY;
			unityConstraint.affectRotationZ = constraintRotation->AffectZ;

			// pass the euler angle unchanged:
			// To convert it for Unity we need the target node, which we cannot get at this point (it has already been imported).
			// We pass the angle unchanged here so that it can be converted in ModelImporter, at the point where we have access to the target node
			FbxDouble3 fbxEuler = constraintRotation->Rotation.EvaluateValue(0);
			unityConstraint.rotation = Vector3f(fbxEuler[0], fbxEuler[1], fbxEuler[2]);
			ImportCommonConstraintProperties(constraintRotation, unityScene, unityConstraint);
		}
		break;
		case kConstraintKindScale:
		{
			FbxConstraintScale* constraintScale = static_cast<FbxConstraintScale*>(constraint);
			unityConstraint.scaling = FBXPointToVector3(constraintScale->Scaling.EvaluateValue(0));

			unityConstraint.affectScalingX = constraintScale->AffectX;
			unityConstraint.affectScalingY = constraintScale->AffectY;
			unityConstraint.affectScalingZ = constraintScale->AffectZ;

			ImportCommonConstraintProperties(constraintScale, unityScene, unityConstraint);
		}
		break;
		case kConstraintKindParent:
		{
			FbxConstraintParent* constraintParent = static_cast<FbxConstraintParent*>(constraint);

			unityConstraint.affectTranslationX = constraintParent->AffectTranslationX;
			unityConstraint.affectTranslationY = constraintParent->AffectTranslationY;
			unityConstraint.affectTranslationZ = constraintParent->AffectTranslationZ;

			unityConstraint.affectRotationX = constraintParent->AffectRotationX;
			unityConstraint.affectRotationY = constraintParent->AffectRotationY;
			unityConstraint.affectRotationZ = constraintParent->AffectRotationZ;

			unityConstraint.affectScalingX = constraintParent->AffectScalingX;
			unityConstraint.affectScalingY = constraintParent->AffectScalingY;
			unityConstraint.affectScalingZ = constraintParent->AffectScalingZ;

			ImportCommonConstraintProperties(constraintParent, unityScene, unityConstraint);
		}
		break;
		default:
			break;
		}

		unityScene.constraints.push_back(unityConstraint);
	}
}
