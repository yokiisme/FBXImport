#pragma once
#include <fbxsdk.h>




void DetectPotential3dsMaxCrash(FbxNode* pNode, std::vector<const FbxNode*>& invalidNodes, int& nodeCount);
void DetectPotential3dsMaxCrash(FbxNode* node);

void ConvertNurbsAndPatch(FbxManager& sdkManager, FbxScene& fbxScene);
void SyncCurveNodeChannelValue(const std::string& nodeName, FbxAnimCurveNode* curveNode, const char* channelName);
bool IsValidTimeRange(const char* const takeName, const char* const nodeName, const char* const curveName, const float minimum, const float maximum);
void RemoveInvalidCurves(const char* const takeName, const char* const nodeName, FbxAnimLayer& animLayer, const char* const curveName, FbxPropertyT<FbxDouble3>& fbxProperty, const char* const* ids);
void RemoveInvalidTransformCurves(FbxScene& fbxScene, FbxNode* root);
void SyncCurveNodeChannelValue(FbxNode* node);
void Preprocess(FbxScene* scene, const float framerate);

void ProcessFBXImport(FbxManager* fbxManager, FbxScene* fbxScene, FBXImportScene& outputScene);


bool IsValidTimeRange(const char* const takeName, const char* const nodeName, const char* const curveName, const float minimum, const float maximum)
{
    const float kMaxTimeRange = 100000;
    const float timeRange = maximum - minimum;
    if (timeRange > kMaxTimeRange)
    {

        return false;
    }
    else if (timeRange < 0)
    {
        return false;
    }
    else
        return true;
}


void RemoveInvalidCurves(const char* const takeName, const char* const nodeName, FbxAnimLayer& animLayer, const char* const curveName, FbxPropertyT<FbxDouble3>& fbxProperty, const char* const* ids)
{
    FbxAnimCurve* curves[3] =
    {
        fbxProperty.GetCurve(&animLayer, ids[0]),
        fbxProperty.GetCurve(&animLayer, ids[1]),
        fbxProperty.GetCurve(&animLayer, ids[2])
    };

    bool valid = true;
    for (int i = 0; i < 3; ++i)
    {
        if (curves[i] && curves[i]->KeyGetCount())
        {
            const float curMin = static_cast<float>(curves[i]->KeyGet(0).GetTime().GetSecondDouble());
            const float curMax = static_cast<float>(curves[i]->KeyGet(curves[i]->KeyGetCount() - 1).GetTime().GetSecondDouble());

            if (!IsValidTimeRange(takeName, nodeName, curveName, curMin, curMax))
            {
                valid = false;
                break;
            }
        }
    }

    if (!valid)
    {
        for (int i = 0; i < 3; ++i)
            if (curves[i])
                curves[i]->KeyClear();
    }
}


static void ConvertNurbsAndPatchRecursive(FbxManager& sdkManager, FbxNode* pNode)
{
    FbxGeometryConverter lConverter(&sdkManager);
    for (int i = 0, c = pNode->GetNodeAttributeCount(); i < c; ++i)
    {
        FbxNodeAttribute* lNodeAttribute = pNode->GetNodeAttributeByIndex(i);
        if (lNodeAttribute)
        {
            // Only triangulate nurbs and patches. Polygon triangulation is done in unity!
            if (lNodeAttribute->GetAttributeType() == FbxNodeAttribute::eNurbs ||
                lNodeAttribute->GetAttributeType() == FbxNodeAttribute::ePatch)
            {
                lConverter.Triangulate(lNodeAttribute, true);
            }
        }
    }

    for (int i = 0, c = pNode->GetChildCount(); i < c; ++i)
    {
        ConvertNurbsAndPatchRecursive(sdkManager, pNode->GetChild(i));
    }
}

void ConvertNurbsAndPatch(FbxManager& sdkManager, FbxScene& fbxScene)
{
    ConvertNurbsAndPatchRecursive(sdkManager, fbxScene.GetRootNode());
}



static void RecursiveRemoveInvalidTransformCurves(const char* const takeName, FbxAnimLayer& animLayer, FbxNode* node)
{
    const char* const translationIds[3] = { FBXSDK_CURVENODE_COMPONENT_X, FBXSDK_CURVENODE_COMPONENT_Y, FBXSDK_CURVENODE_COMPONENT_Z };
    const char* const scaleIds[3] = { FBXSDK_CURVENODE_COMPONENT_X, FBXSDK_CURVENODE_COMPONENT_Y, FBXSDK_CURVENODE_COMPONENT_Z };
    const char* const rotationIds[3] = { FBXSDK_CURVENODE_COMPONENT_X, FBXSDK_CURVENODE_COMPONENT_Y, FBXSDK_CURVENODE_COMPONENT_Z };

    const char* const nodeName = node->GetName();
    RemoveInvalidCurves(takeName, nodeName, animLayer, "translation", node->LclTranslation, translationIds);
    RemoveInvalidCurves(takeName, nodeName, animLayer, "scale", node->LclScaling, scaleIds);
    RemoveInvalidCurves(takeName, nodeName, animLayer, "rotation", node->LclRotation, rotationIds);

    for (int i = 0, count = node->GetChildCount(); i < count; ++i)
    {
        FbxNode* curChild = node->GetChild(i);
        RecursiveRemoveInvalidTransformCurves(takeName, animLayer, curChild);
    }
}


void DetectPotential3dsMaxCrash(FbxNode* pNode, std::vector<const FbxNode*>& invalidNodes, int& nodeCount)
{
    ++nodeCount;

    if (pNode == NULL)
        return;

    FbxProperty lTMProperty[] =
    {
        pNode->LclTranslation,
        pNode->LclRotation,
        pNode->LclScaling
    };

    FbxTime lNever(FbxLongLong(-20663336074399200));  // 3ds max's NEVER

    bool isValid = true;

    // 3ds max export a time range {NEVER, NEVER} and collapse to one key
    // detect these keys and reset them to 0sec
    for (int i = 0; i < 3; i++)
    {
        FbxAnimCurveNode* lCurveNode = lTMProperty[i].GetCurveNode();
        if (lCurveNode == NULL)
            continue;

        for (unsigned int j = 0; j < 3; j++)
        {
            FbxAnimCurve* lCurve = lCurveNode->GetCurve(j);
            if (lCurve && lCurve->KeyGetCount() == 1 && lCurve->KeyGetTime(0) <= lNever)
            {
                lCurve->KeySetTime(0, FbxTime(0));
                isValid = false;
            }
        }
    }

    if (!isValid)
        invalidNodes.push_back(pNode);

    for (int i = 0; i < pNode->GetChildCount(); i++)
    {
        DetectPotential3dsMaxCrash(pNode->GetChild(i), invalidNodes, nodeCount);
    }
}

void DetectPotential3dsMaxCrash(FbxNode* node)
{
    std::vector<const FbxNode*> invalidNodes;
    int nodeCount = 0;
    DetectPotential3dsMaxCrash(node, invalidNodes, nodeCount);
}

void SyncCurveNodeChannelValue(const std::string& nodeName, FbxAnimCurveNode* curveNode, const char* channelName)
{
    if (curveNode && 3 == curveNode->GetChannelsCount())
    {
        // channels which have value (0, 0, 0) are considered invalid
        bool valid = false;
        for (unsigned int i = 0; i < curveNode->GetChannelsCount(); ++i)
        {
            const double channelValue = curveNode->GetChannelValue(i, -1.0);
            if (0 != channelValue)
            {
                valid = true;
                break;
            }
        }

        if (!valid)
        {
            for (unsigned int i = 0; i < curveNode->GetChannelsCount(); ++i)
            {
                double value = 1;
                FbxAnimCurve* curve = curveNode->GetCurve(i);
                if (curve && curve->KeyGetCount() > 0)
                    value = curve->KeyGetValue(0);

                curveNode->SetChannelValue(i, value);
            }
        }
    }
}


void SyncCurveNodeChannelValue(FbxNode* node)
{
    if (node == NULL)
        return;

    const std::string nodeName = static_cast<const char*>(node->GetNameWithoutNameSpacePrefix());

    SyncCurveNodeChannelValue(nodeName, node->LclScaling.GetCurveNode(), "Scale");

    for (int i = 0; i < node->GetChildCount(); ++i)
        SyncCurveNodeChannelValue(node->GetChild(i));
}

void RemoveInvalidTransformCurves(FbxScene& fbxScene, FbxNode* root)
{
    // The root node never has any animation, so only process it's children.
    const int childCount = root->GetChildCount();

    for (int i = 0, c = fbxScene.GetSrcObjectCount<FbxAnimStack>(); i < c; ++i)
    {
        FbxAnimStack* animStack = fbxScene.GetSrcObject<FbxAnimStack>(i);
        const std::string takeName = animStack->GetName();

        const int animLayerCount = animStack->GetSrcObjectCount<FbxAnimLayer>();
        for (int j = 0; j < animLayerCount; ++j)
        {
            FbxAnimLayer* animLayer = animStack->GetSrcObject<FbxAnimLayer>(j);

            for (int k = 0; k < childCount; ++k)
                RecursiveRemoveInvalidTransformCurves(takeName.c_str(), *animLayer, root->GetChild(k));
        }
    }
}


void Preprocess(FbxScene* scene, const float framerate)
{
    FbxNode* root = scene->GetRootNode();

    /// This function should transform control points and animation curves to match our simplified node system.
    /// * The origin of the node has to be where the rotation pivot is in maya.
    ///   Eg. Vertices have to be transformed so a vertex which has the same world-space position as the rotation pivot
    ///   in world space will end up be at (0,0,0) in local-space.
    /// * Animation curves have to be converted so that animations look the same but match the simplified pivot context.

    // Fixing corrupt scale
    SyncCurveNodeChannelValue(root);

    // Clear curves with keys out of range
    RemoveInvalidTransformCurves(*scene, root);

    root->ResetPivotSetAndConvertAnimation(framerate, false, false);
}



void ProcessFBXImport(FbxManager* fbxManager, FbxScene* fbxScene, FBXImportScene& outputScene)
{
    FbxGlobalSettings& fbxGlobalSettings = fbxScene->GetGlobalSettings();
    float sceneScaleFactor = (float)fbxGlobalSettings.GetSystemUnit().GetScaleFactor();
    const FbxString sceneUnit(fbxGlobalSettings.GetSystemUnit().GetScaleFactorAsString());
    if (sceneScaleFactor != 1.0)
    {
        FbxSystemUnit::cm.ConvertScene(fbxScene);
        FbxSystemUnit::ConversionOptions options;
        options.mConvertRrsNodes = false; // Do not convert nodes that don't inherit scale from their parent.
        FbxSystemUnit(sceneScaleFactor).ConvertScene(fbxScene, options);
    }
    outputScene.fileScaleFactor = sceneScaleFactor * 0.01f;
    outputScene.fileScaleUnit = sceneUnit.Buffer();

    FbxAxisSystem sceneAxisSystem = fbxGlobalSettings.GetAxisSystem();
    FbxAxisSystem unityAxisSystem(FbxAxisSystem::eYAxis, FbxAxisSystem::eParityOdd, FbxAxisSystem::eRightHanded);
    if (sceneAxisSystem != unityAxisSystem)
        unityAxisSystem.ConvertScene(fbxScene);

    DetectPotential3dsMaxCrash(fbxScene->GetRootNode());
    double frameRate;
    FbxTime::EMode timeMode = fbxGlobalSettings.GetTimeMode();
    if (timeMode != FbxTime::eCustom)
        frameRate = FbxTime::GetFrameRate(timeMode);
    else
        frameRate = fbxGlobalSettings.GetCustomFrameRate();

    if (frameRate <= 0.0)
    {
        frameRate = 1.0;
    }

    Preprocess(fbxScene, frameRate);

    ConvertNurbsAndPatch(*fbxManager, *fbxScene);

}