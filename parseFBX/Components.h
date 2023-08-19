#pragma once
#include "Utility.h"
#include "FBXImporterDef.h"
#include "AnimationImporter.h"
struct FBXImportNode;
struct FBXImportScene;
void ConvertComponents(FbxNode* fbxNode, FBXImportNode& node, FBXImportScene& scene, const FBXImportSettings& settings);

typedef float FOVConversionFunction(FbxCamera& fbxCamera, double focalLength);
bool GetFieldOfViewProperty(FbxCamera& fbxCamera, FbxPropertyT<FbxDouble>*& property, FOVConversionFunction*& converter);
void GetFocalLengthProperty(FbxCamera& fbxCamera, FbxPropertyT<FbxDouble>*& property, FOVConversionFunction*& converter);

float ScaleLightIntensity(FbxLight& fbxLight, float intensity);