#pragma once
#include "FBXImporterDef.h"
struct FBXImportScene;

void ImportConstraints(FbxManager& sdkManager, FbxScene& fbxScene, FbxNode* root, FBXImportScene& unity);
bool IsConstraintTypeSupported(FbxConstraint::EType constraintType);