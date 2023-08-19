#pragma once
#include <fbxsdk.h>
#include "Utility.h"
#include "FBXImporterDef.h"
#include "ProcessFBX.h"

FbxAMatrix GetNodeLOCTransform(FbxNode* pNode, float scalefactor)
{
    FbxAMatrix matrixGeo;
    matrixGeo.SetIdentity();

    if (pNode->GetNodeAttribute())
    {
        const FbxDouble3 lT = pNode->LclTranslation;
        const FbxDouble3 lR = pNode->LclRotation;
        const FbxDouble3 lS = pNode->LclScaling;

        matrixGeo.SetT(lT);
        matrixGeo.SetR(lR);
        matrixGeo.SetS(lS);
    }
    return matrixGeo;
}

FbxAMatrix CalculateGlobalTransform(FbxNode* pNode)
{
    FbxAMatrix lTranlationM, lScalingM, lScalingPivotM, lScalingOffsetM, lRotationOffsetM, lRotationPivotM, lPreRotationM, lRotationM, lPostRotationM, lTransform;

    FbxAMatrix lParentGX, lGlobalT, lGlobalRS;

    if (!pNode)
    {
        lTransform.SetIdentity();
        return lTransform;
    }

    // Construct translation matrix
    FbxVector4 lTranslation = pNode->LclTranslation.Get();
    lTranlationM.SetT(lTranslation);

    // Construct rotation matrices
    FbxVector4 lRotation = pNode->LclRotation.Get();
    FbxVector4 lPreRotation = pNode->PreRotation.Get();
    FbxVector4 lPostRotation = pNode->PostRotation.Get();
    lRotationM.SetR(lRotation);
    lPreRotationM.SetR(lPreRotation);
    lPostRotationM.SetR(lPostRotation);

    // Construct scaling matrix
    FbxVector4 lScaling = pNode->LclScaling.Get();
    lScalingM.SetS(lScaling);

    // Construct offset and pivot matrices
    FbxVector4 lScalingOffset = pNode->ScalingOffset.Get();
    FbxVector4 lScalingPivot = pNode->ScalingPivot.Get();
    FbxVector4 lRotationOffset = pNode->RotationOffset.Get();
    FbxVector4 lRotationPivot = pNode->RotationPivot.Get();
    lScalingOffsetM.SetT(lScalingOffset);
    lScalingPivotM.SetT(lScalingPivot);
    lRotationOffsetM.SetT(lRotationOffset);
    lRotationPivotM.SetT(lRotationPivot);


    // Calculate the global transform matrix of the parent node
    FbxNode* lParentNode = pNode->GetParent();
    if (lParentNode)
    {
        lParentGX = CalculateGlobalTransform(lParentNode);
    }
    else
    {
        lParentGX.SetIdentity();
    }

    //Construct Global Rotation
    FbxAMatrix lLRM, lParentGRM;
    FbxVector4 lParentGR = lParentGX.GetR();
    lParentGRM.SetR(lParentGR);
    lLRM = lPreRotationM * lRotationM * lPostRotationM;

    //Construct Global Shear*Scaling
    //FBX SDK does not support shear, to patch this, we use:
    //Shear*Scaling = RotationMatrix.Inverse * TranslationMatrix.Inverse * WholeTranformMatrix
    FbxAMatrix lLSM, lParentGSM, lParentGRSM, lParentTM;
    FbxVector4 lParentGT = lParentGX.GetT();
    lParentTM.SetT(lParentGT);
    lParentGRSM = lParentTM.Inverse() * lParentGX;
    lParentGSM = lParentGRM.Inverse() * lParentGRSM;
    lLSM = lScalingM;

    //Do not consider translation now
    FbxTransform::EInheritType lInheritType = pNode->InheritType.Get();
    if (lInheritType == FbxTransform::eInheritRrSs)
    {
        lGlobalRS = lParentGRM * lLRM * lParentGSM * lLSM;
    }
    else if (lInheritType == FbxTransform::eInheritRSrs)
    {
        lGlobalRS = lParentGRM * lParentGSM * lLRM * lLSM;
    }
    else if (lInheritType == FbxTransform::eInheritRrs)
    {
        FbxAMatrix lParentLSM;
        FbxVector4 lParentLS = lParentNode->LclScaling.Get();
        lParentLSM.SetS(lParentLS);

        FbxAMatrix lParentGSM_noLocal = lParentGSM * lParentLSM.Inverse();
        lGlobalRS = lParentGRM * lLRM * lParentGSM_noLocal * lLSM;
    }
    else
    {
        FBXSDK_printf("error, unknown inherit type! \n");
    }

    // Construct translation matrix
    // Calculate the local transform matrix
    lTransform = lTranlationM * lRotationOffsetM * lRotationPivotM * lPreRotationM * lRotationM * lPostRotationM * lRotationPivotM.Inverse()\
        * lScalingOffsetM * lScalingPivotM * lScalingM * lScalingPivotM.Inverse();
    FbxVector4 lLocalTWithAllPivotAndOffsetInfo = lTransform.GetT();
    // Calculate global translation vector according to: 
    // GlobalTranslation = ParentGlobalTransform * LocalTranslationWithPivotAndOffsetInfo



    FbxVector4 lGlobalTranslation = lParentGX.MultT(lLocalTWithAllPivotAndOffsetInfo);
    lGlobalT.SetT(lGlobalTranslation);

    //Construct the whole global transform
    lTransform = lGlobalT * lGlobalRS;

    return lTransform;
}

FbxAMatrix GetNodeGeometryTransform(FbxNode* pNode)
{
    FbxAMatrix matrixGeo;
    matrixGeo.SetIdentity();

    if (pNode->GetNodeAttribute())
    {
        FbxVector4 lT = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
        FbxVector4 lR = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
        FbxVector4 lS = pNode->GetGeometricScaling(FbxNode::eSourcePivot);

        matrixGeo.SetT(lT);
        /*matrixGeo.SetR(lR);
        matrixGeo.SetS(lS);*/
    }

    return matrixGeo;
}

FbxAMatrix GetNodeLocalTransform(FbxNode* pNode, float scalefactor)
{
    FbxAMatrix matrixLoc = GetNodeLOCTransform(pNode, scalefactor);
    FbxAMatrix matrixGeo = GetNodeGeometryTransform(pNode);
    return matrixGeo;
}

FbxAMatrix GetTransform(FbxNode* pNode)
{
    FbxAMatrix matrixGeo;
    matrixGeo.SetIdentity();
    if (pNode->GetNodeAttribute())
    {
        const FbxVector4 lT = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
        const FbxVector4 lR = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
        const FbxVector4 lS = pNode->GetGeometricScaling(FbxNode::eSourcePivot);
        matrixGeo.SetT(lT);
        /* matrixGeo.SetR(lR);
         matrixGeo.SetS(lS);*/
    }
    FbxAMatrix localMatrix = pNode->EvaluateLocalTransform();

    FbxNode* pParentNode = pNode->GetParent();
    FbxAMatrix parentMatrix = pParentNode->EvaluateLocalTransform();
    while ((pParentNode = pParentNode->GetParent()) != NULL)
    {
        parentMatrix = pParentNode->EvaluateLocalTransform() * parentMatrix;
    }

    return parentMatrix * localMatrix * matrixGeo;

}

void ParseMesh(FbxNode* pNode, FBXGameObject* gameObject, float scalefactor)//原解析
{
    FbxMesh* pMesh = (FbxMesh*)pNode->GetNodeAttribute();
    FbxVector4* lControlPoints = pMesh->GetControlPoints();
    FBXMesh fbxMesh;

    int namelen = strlen(pNode->GetName()) + 1;
    fbxMesh.name = new char[namelen];
    strcpy_s(fbxMesh.name, namelen, pNode->GetName());

    int i, j, l, lPolygonCount = pMesh->GetPolygonCount();
    int polygon_cur = 0;
    int submeshcount = 0;
    int vertexId = 0;
    std::vector<std::vector<uint32_t>> meshindices;
    std::vector<uint32_t> materialindex;
    std::vector<std::vector<uint32_t>> indices;
    meshindices.resize(lPolygonCount);

    for (i = 0; i < lPolygonCount; i++)
    {
        int lPolygonSize = pMesh->GetPolygonSize(i);
        for (j = 0; j < lPolygonSize; j++)
        {
            int lControlPointIndex = pMesh->GetPolygonVertex(i, j);
            //转换过
            FbxVector4 post = FbxVector4(-lControlPoints[lControlPointIndex][0], lControlPoints[lControlPointIndex][2], -lControlPoints[lControlPointIndex][1]);
            fbxMesh.vertices.push_back(Vector3f(post[0] * scalefactor, post[1] * scalefactor, post[2] * scalefactor));

            for (l = 0; l < pMesh->GetElementNormalCount(); ++l)
            {
                FbxGeometryElementNormal* leNormal = pMesh->GetElementNormal(l);
                if (leNormal->GetMappingMode() == FbxGeometryElement::eByPolygonVertex)
                {
                    switch (leNormal->GetReferenceMode())
                    {
                    case FbxGeometryElement::eDirect:
                    {
                        FbxVector4 nort = FbxVector4(-leNormal->GetDirectArray().GetAt(vertexId)[0], leNormal->GetDirectArray().GetAt(vertexId)[2],
                            -leNormal->GetDirectArray().GetAt(vertexId)[1]);
                        fbxMesh.normals.push_back(Vector3f(nort[0] * scalefactor, nort[1] * scalefactor, nort[2] * scalefactor));

                        //FbxVector4 nort = FbxVector4(leNormal->GetDirectArray().GetAt(vertexId)[0], leNormal->GetDirectArray().GetAt(vertexId)[1],
                        //    leNormal->GetDirectArray().GetAt(vertexId)[2]);
                        //fbxMesh.normals.push_back(Vector3f(nort[0] * scalefactor, nort[1] * scalefactor, nort[2] * scalefactor));

                        /*FbxVector4 nort = FbxVector4(leNormal->GetDirectArray().GetAt(vertexId)[0], leNormal->GetDirectArray().GetAt(vertexId)[1],
                            leNormal->GetDirectArray().GetAt(vertexId)[2]);
                        nort = matrixA.MultT(nort);
                        fbxMesh.normals.push_back(Vector3f(-nort[0] * scalefactor, nort[2] * scalefactor, -nort[1] * scalefactor));*/
                    }
                    break;
                    case FbxGeometryElement::eIndexToDirect:
                    {
                        int id = leNormal->GetIndexArray().GetAt(lControlPointIndex);

                        FbxVector4 nort = FbxVector4(-leNormal->GetDirectArray().GetAt(id)[0], leNormal->GetDirectArray().GetAt(id)[2],
                            -leNormal->GetDirectArray().GetAt(id)[1]);
                        fbxMesh.normals.push_back(Vector3f(nort[0] * scalefactor, nort[1] * scalefactor, nort[2] * scalefactor));

                        //FbxVector4 nort = FbxVector4(leNormal->GetDirectArray().GetAt(id)[0], leNormal->GetDirectArray().GetAt(id)[1],
                        //    leNormal->GetDirectArray().GetAt(id)[2]);
                        //fbxMesh.normals.push_back(Vector3f(nort[0] * scalefactor, nort[1] * scalefactor, nort[2] * scalefactor));
                    }
                    break;
                    default:
                        break; // other reference modes not shown here!
                    }
                }
            }

            for (l = 0; l < pMesh->GetElementUVCount(); ++l)
            {
                FbxGeometryElementUV* leUV = pMesh->GetElementUV(l);
                switch (leUV->GetMappingMode())
                {
                default:
                    break;

                case FbxGeometryElement::eByPolygonVertex:
                {
                    int lTextureUVIndex = pMesh->GetTextureUVIndex(i, j);
                    switch (leUV->GetReferenceMode())
                    {
                    case FbxGeometryElement::eDirect:
                    {
                        if (l == 0)
                        {
                            float u = leUV->GetDirectArray().GetAt(lTextureUVIndex)[0];
                            float v = leUV->GetDirectArray().GetAt(lTextureUVIndex)[1];

                            float u1 = leUV->GetDirectArray().GetAt(vertexId)[0];
                            float v1 = leUV->GetDirectArray().GetAt(vertexId)[1];

                            fbxMesh.uv1.push_back(Vector2f(u, v));
                        }
                        else if (l == 1)
                        {
                            fbxMesh.uv2.push_back(Vector2f(leUV->GetDirectArray().GetAt(lTextureUVIndex)[0], leUV->GetDirectArray().GetAt(lTextureUVIndex)[1]));
                        }
                    }
                    break;
                    case FbxGeometryElement::eIndexToDirect:
                    {
                        if (l == 0)
                        {
                            float u = leUV->GetDirectArray().GetAt(lTextureUVIndex)[0];
                            float v = leUV->GetDirectArray().GetAt(lTextureUVIndex)[1];
                            fbxMesh.uv1.push_back(Vector2f(u, v));
                        }
                        else if (l == 1)
                        {
                            float u = leUV->GetDirectArray().GetAt(lTextureUVIndex)[0];
                            float v = leUV->GetDirectArray().GetAt(lTextureUVIndex)[1];
                            fbxMesh.uv2.push_back(Vector2f(leUV->GetDirectArray().GetAt(lTextureUVIndex)[0], leUV->GetDirectArray().GetAt(lTextureUVIndex)[1]));
                        }
                    }
                    break;
                    default:
                        break; // other reference modes not shown here!
                    }
                }
                break;

                case FbxGeometryElement::eByPolygon: // doesn't make much sense for UVs
                case FbxGeometryElement::eAllSame:   // doesn't make much sense for UVs
                case FbxGeometryElement::eNone:       // doesn't make much sense for UVs
                    break;
                }
            }

            vertexId++;
        }

        int materialCount = pMesh->GetElementMaterialCount();

        for (l = 0; l < materialCount; l++)
        {
            FbxGeometryElementMaterial* lMaterialElement = pMesh->GetElementMaterial(l);
            FbxSurfaceMaterial* lMaterial = NULL;
            int submeshindex = lMaterialElement->GetIndexArray().GetAt(i);
            lMaterial = pMesh->GetNode()->GetMaterial(submeshindex);

            std::string matname(lMaterial->GetName());
            if (fbxMesh.materials.size() == 0)
            {
                fbxMesh.materials.push_back(matname);
                fbxMesh.materialindex.push_back(submeshindex);
            }

            if (std::find(fbxMesh.materials.begin(), fbxMesh.materials.end(), matname) == fbxMesh.materials.end())
            {
                fbxMesh.materials.push_back(matname);
                fbxMesh.materialindex.push_back(submeshindex);
            }
            /*if (submeshcount < submeshindex)
            {
                submeshcount = submeshindex;
                std::string matname(lMaterial->GetName());
                fbxMesh.materials.push_back(matname);
            }*/

            if (submeshindex >= 0)
            {
                materialindex.push_back(submeshindex);
                if (std::find(fbxMesh.materialindex.begin(), fbxMesh.materialindex.end(), submeshindex) == fbxMesh.materialindex.end())
                {
                    fbxMesh.materials.push_back(matname);
                    fbxMesh.materialindex.push_back(submeshindex);
                }
                break;
            }
        }
        std::vector<uint32_t> polygonvec;
        for (l = 0; l < lPolygonSize - 2; l++)
        {
            polygonvec.push_back(polygon_cur);
            polygonvec.push_back(l + polygon_cur + 2);
            polygonvec.push_back(l + polygon_cur + 1);//转换过
        }
        meshindices[i] = polygonvec;
        polygon_cur += lPolygonSize;
    }

    indices.resize((fbxMesh.materials.size()));

    for (i = 0; i < materialindex.size(); i++)
    {
        indices[materialindex[i]].insert(indices[materialindex[i]].end(), meshindices[i].begin(), meshindices[i].end());
    }

    for (int i = 0; i < indices.size(); i++)
    {
        if (indices[i].size() > 0)
        {
            fbxMesh.indices.insert(fbxMesh.indices.end(), indices[i].begin(), indices[i].end());
            fbxMesh.indicesize.push_back(indices[i].size());
        }
    }

    gameObject->meshList.push_back(fbxMesh);
}

void ParseConentByNode(FbxNode* pNode, FBXGameObject* gameObject, float scalefactor)
{
    FbxNodeAttribute::EType lAttributeType;

    if (pNode->GetNodeAttribute() == NULL)
    {
        //FBXSDK_printf("NULL Node Attribute\n\n");
    }
    else
    {
        lAttributeType = (pNode->GetNodeAttribute()->GetAttributeType());

        switch (lAttributeType)
        {
        default:
            break;
        case FbxNodeAttribute::eMesh:
            gameObject->meshCount++;
            ParseMesh(pNode, gameObject, scalefactor);
            break;
        }
    }
    for (int i = 0; i < pNode->GetChildCount(); i++)
    {
        ParseConentByNode(pNode->GetChild(i), gameObject, scalefactor);
    }
}


void SetPivotStateRecursive(FbxNode* node)
{
    EFbxRotationOrder order;
    node->GetRotationOrder(FbxNode::eSourcePivot, order);
    if (order == eSphericXYZ)
    {
        order = eEulerXYZ;
    }
    node->SetRotationOrder(FbxNode::eDestinationPivot, order);
    node->SetPivotState(FbxNode::eSourcePivot, FbxNode::ePivotActive);

    for (int i = 0, size = node->GetChildCount(); i < size; ++i)
    {
        FbxNode* child = node->GetChild(i);
        SetPivotStateRecursive(child);
    }
}

FBXGameObject* ParseContent(FbxManager* lSdkManager, FbxScene* pScene)
{
    FbxGlobalSettings& fbxGlobalSettings = pScene->GetGlobalSettings();
    float sceneScaleFactor = (float)fbxGlobalSettings.GetSystemUnit().GetScaleFactor();
    FBXGameObject* gameObj = new FBXGameObject();
    FbxNode* lNode = pScene->GetRootNode();
    gameObj->meshCount = 0;
    lNode->ResetPivotSet(FbxNode::eDestinationPivot);
    SetPivotStateRecursive(lNode);
    ConvertNurbsAndPatch(*lSdkManager, *pScene);
    ParseConentByNode(lNode, gameObj, sceneScaleFactor * 0.01f);
    return gameObj;
}
