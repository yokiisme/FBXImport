#pragma once
#include "Utility.h"



int FindGridCell(const float fMin, const float fMax, const float fVal)
{
    const float fIndex = (fMin == fMax) ? 0 : g_iCells * ((fVal - fMin) / (fMax - fMin));
    const int iIndex = fIndex < 0 ? 0 : ((int)fIndex);
    return iIndex < g_iCells ? iIndex : (g_iCells - 1);
}

static int getNumFaces(const SMikkTSpaceContext* Context)
{
    TSpaceMeshInfo* MeshInf = reinterpret_cast<TSpaceMeshInfo*>(Context->m_pUserData);
    const FBXImportMesh* ImpMesh = MeshInf->ImpMesh;
    return ImpMesh->polygonSizes.size();
}

static int getNumVerticesOfFace(const SMikkTSpaceContext* Context, const int FaceIdx)
{
    TSpaceMeshInfo* MeshInf = reinterpret_cast<TSpaceMeshInfo*>(Context->m_pUserData);
    const FBXImportMesh* ImpMesh = MeshInf->ImpMesh;
    return ImpMesh->polygonSizes[FaceIdx];
}

static void getPosition(const SMikkTSpaceContext* Context, float PosOut[], const int FaceIdx, const int VertIdx)
{
    TSpaceMeshInfo* MeshInf = reinterpret_cast<TSpaceMeshInfo*>(Context->m_pUserData);
    const FBXImportMesh* ImpMesh = MeshInf->ImpMesh;
    const uint32_t* FaceOffs = MeshInf->FaceOffs;
    //const int iNrVertsOnFace = pImpMesh->polygonSizes[iFace];     // always 3 or 4
    const uint32_t* face = &ImpMesh->polygons[FaceOffs[FaceIdx]];
    const uint32_t idx = face[VertIdx];
    Vector3f vP = MeshInf->vertices[idx];   // shape position array
    PosOut[0] = vP.x; PosOut[1] = vP.y; PosOut[2] = vP.z;
}

static void getNormal(const SMikkTSpaceContext* Context, float NormOut[], const int FaceIdx, const int VertIdx)
{
    TSpaceMeshInfo* MeshInf = reinterpret_cast<TSpaceMeshInfo*>(Context->m_pUserData);
    const uint32_t* FaceOffs = MeshInf->FaceOffs;

    Vector3f vN = MeshInf->normals[FaceOffs[FaceIdx] + VertIdx];        // shape normals array
    NormOut[0] = vN.x; NormOut[1] = vN.y; NormOut[2] = vN.z;
}

static void setTSpace(const SMikkTSpaceContext* Context, const float Tangent[], const float BiTangent[], const float fMagS, const float fMagT,
    const bool IsOrientationPreserving, const int FaceIdx, const int VertIdx)
{
    TSpaceMeshInfo* MeshInf = reinterpret_cast<TSpaceMeshInfo*>(Context->m_pUserData);
    const uint32_t* FaceOffs = MeshInf->FaceOffs;
    Vector4f* outTangents = MeshInf->outTangents;          // shape output tangents

    // fSign * cross(vN, vT) <-- In that order!
    const float fSign = (IsOrientationPreserving != 0) ? 1.0f : -1.0f;
    outTangents[FaceOffs[FaceIdx] + VertIdx] = Vector4f(Tangent[0], Tangent[1], Tangent[2], fSign);
}

static void IndexToData(int* piFace, int* piVert, const int iIndexIn)
{
    piVert[0] = iIndexIn & 0x3;
    piFace[0] = iIndexIn >> 2;
}

static Vector3f GetPosition(const SMikkTSpaceContext* pContext, const int index)
{
    int iF, iI;
    Vector3f res; float pos[3];
    IndexToData(&iF, &iI, index);
    pContext->m_pInterface->m_getPosition(pContext, pos, iF, iI);
    res.x = pos[0]; res.y = pos[1]; res.z = pos[2];
    return res;
}

static Vector3f GetNormal(const SMikkTSpaceContext* pContext, const int index)
{
    int iF, iI;
    Vector3f res;
    float norm[3];
    IndexToData(&iF, &iI, index);
    pContext->m_pInterface->m_getNormal(pContext, norm, iF, iI);
    res.x = norm[0]; res.y = norm[1]; res.z = norm[2];
    return res;
}

static void getTexCoord(const SMikkTSpaceContext* Context, float TexcOut[], const int FaceIdx, const int VertIdx)
{
    TSpaceMeshInfo* MeshInf = reinterpret_cast<TSpaceMeshInfo*>(Context->m_pUserData);
    FBXImportMesh* ImpMesh = MeshInf->ImpMesh;

    float U = 0.0f, V = 0.0f;
    if (!ImpMesh->uvs[0].empty())
    {
        const uint32_t* FaceOffs = MeshInf->FaceOffs;

        const Vector2f* tex = &ImpMesh->uvs[0][0];
        const Vector2f TC = tex[FaceOffs[FaceIdx] + VertIdx];
        U = TC.x; V = TC.y;
    }

    TexcOut[0] = U; TexcOut[1] = V;
}


static Vector3f GetTexCoord(const SMikkTSpaceContext* pContext, const int index)
{
    int iF, iI;
    Vector3f res; float texc[2];
    IndexToData(&iF, &iI, index);
    pContext->m_pInterface->m_getTexCoord(pContext, texc, iF, iI);
    res.x = texc[0]; res.y = texc[1]; res.z = 1.0f;
    return res;
}


static int GenerateInitialVerticesIndexList(STriInfo pTriInfos[], int piTriList_out[], const SMikkTSpaceContext* pContext, const int iNrTrianglesIn)
{
    int iTSpacesOffs = 0, f = 0, t = 0;
    int iDstTriIndex = 0;
    for (f = 0; f < pContext->m_pInterface->m_getNumFaces(pContext); f++)
    {
        const int verts = pContext->m_pInterface->m_getNumVerticesOfFace(pContext, f);
        if (verts != 3 && verts != 4) continue;

        pTriInfos[iDstTriIndex].iOrgFaceNumber = f;
        pTriInfos[iDstTriIndex].iTSpacesOffs = iTSpacesOffs;

        if (verts == 3)
        {
            unsigned char* pVerts = pTriInfos[iDstTriIndex].vert_num;
            pVerts[0] = 0; pVerts[1] = 1; pVerts[2] = 2;
            piTriList_out[iDstTriIndex * 3 + 0] = MakeIndex(f, 0);
            piTriList_out[iDstTriIndex * 3 + 1] = MakeIndex(f, 1);
            piTriList_out[iDstTriIndex * 3 + 2] = MakeIndex(f, 2);
            ++iDstTriIndex;	// next
        }
        else
        {
            {
                pTriInfos[iDstTriIndex + 1].iOrgFaceNumber = f;
                pTriInfos[iDstTriIndex + 1].iTSpacesOffs = iTSpacesOffs;
            }

            {
                // need an order independent way to evaluate
                // tspace on quads. This is done by splitting
                // along the shortest diagonal.
                const int i0 = MakeIndex(f, 0);
                const int i1 = MakeIndex(f, 1);
                const int i2 = MakeIndex(f, 2);
                const int i3 = MakeIndex(f, 3);
                const Vector3f T0 = GetTexCoord(pContext, i0);
                const Vector3f T1 = GetTexCoord(pContext, i1);
                const Vector3f T2 = GetTexCoord(pContext, i2);
                const Vector3f T3 = GetTexCoord(pContext, i3);
                const float distSQ_02 = LengthSquared(vsub(T2, T0));
                const float distSQ_13 = LengthSquared(vsub(T3, T1));
                bool bQuadDiagIs_02;
                if (distSQ_02 < distSQ_13)
                    bQuadDiagIs_02 = true;
                else if (distSQ_13 < distSQ_02)
                    bQuadDiagIs_02 = false;
                else
                {
                    const Vector3f P0 = GetPosition(pContext, i0);
                    const Vector3f P1 = GetPosition(pContext, i1);
                    const Vector3f P2 = GetPosition(pContext, i2);
                    const Vector3f P3 = GetPosition(pContext, i3);
                    const float distSQ_02 = LengthSquared(vsub(P2, P0));
                    const float distSQ_13 = LengthSquared(vsub(P3, P1));

                    bQuadDiagIs_02 = distSQ_13 < distSQ_02 ? false : true;
                }

                if (bQuadDiagIs_02)
                {
                    {
                        unsigned char* pVerts_A = pTriInfos[iDstTriIndex].vert_num;
                        pVerts_A[0] = 0; pVerts_A[1] = 1; pVerts_A[2] = 2;
                    }
                    piTriList_out[iDstTriIndex * 3 + 0] = i0;
                    piTriList_out[iDstTriIndex * 3 + 1] = i1;
                    piTriList_out[iDstTriIndex * 3 + 2] = i2;
                    ++iDstTriIndex;	// next
                    {
                        unsigned char* pVerts_B = pTriInfos[iDstTriIndex].vert_num;
                        pVerts_B[0] = 0; pVerts_B[1] = 2; pVerts_B[2] = 3;
                    }
                    piTriList_out[iDstTriIndex * 3 + 0] = i0;
                    piTriList_out[iDstTriIndex * 3 + 1] = i2;
                    piTriList_out[iDstTriIndex * 3 + 2] = i3;
                    ++iDstTriIndex;	// next
                }
                else
                {
                    {
                        unsigned char* pVerts_A = pTriInfos[iDstTriIndex].vert_num;
                        pVerts_A[0] = 0; pVerts_A[1] = 1; pVerts_A[2] = 3;
                    }
                    piTriList_out[iDstTriIndex * 3 + 0] = i0;
                    piTriList_out[iDstTriIndex * 3 + 1] = i1;
                    piTriList_out[iDstTriIndex * 3 + 2] = i3;
                    ++iDstTriIndex;	// next
                    {
                        unsigned char* pVerts_B = pTriInfos[iDstTriIndex].vert_num;
                        pVerts_B[0] = 1; pVerts_B[1] = 2; pVerts_B[2] = 3;
                    }
                    piTriList_out[iDstTriIndex * 3 + 0] = i1;
                    piTriList_out[iDstTriIndex * 3 + 1] = i2;
                    piTriList_out[iDstTriIndex * 3 + 2] = i3;
                    ++iDstTriIndex;	// next
                }
            }
        }

        iTSpacesOffs += verts;
        //assert(iDstTriIndex <= iNrTrianglesIn);
    }

    for (t = 0; t < iNrTrianglesIn; t++)
        pTriInfos[t].iFlag = 0;

    // return total amount of tspaces
    return iTSpacesOffs;
}


static float CalcTexArea(const SMikkTSpaceContext* pContext, const int indices[])
{
    const Vector3f t1 = GetTexCoord(pContext, indices[0]);
    const Vector3f t2 = GetTexCoord(pContext, indices[1]);
    const Vector3f t3 = GetTexCoord(pContext, indices[2]);

    const float t21x = t2.x - t1.x;
    const float t21y = t2.y - t1.y;
    const float t31x = t3.x - t1.x;
    const float t31y = t3.y - t1.y;

    const float fSignedAreaSTx2 = t21x * t31y - t21y * t31x;

    return fSignedAreaSTx2 < 0 ? (-fSignedAreaSTx2) : fSignedAreaSTx2;
}

static void MergeVertsFast(int piTriList_in_and_out[], STmpVert pTmpVert[], const SMikkTSpaceContext* pContext, const int iL_in, const int iR_in)
{
    // make bbox
    int c = 0, l = 0, channel = 0;
    float fvMin[3], fvMax[3];
    float dx = 0, dy = 0, dz = 0, fSep = 0;
    for (c = 0; c < 3; c++)
    {
        fvMin[c] = pTmpVert[iL_in].vert[c]; fvMax[c] = fvMin[c];
    }
    for (l = (iL_in + 1); l <= iR_in; l++)
        for (c = 0; c < 3; c++)
            if (fvMin[c] > pTmpVert[l].vert[c]) fvMin[c] = pTmpVert[l].vert[c];
            else if (fvMax[c] < pTmpVert[l].vert[c]) fvMax[c] = pTmpVert[l].vert[c];

    dx = fvMax[0] - fvMin[0];
    dy = fvMax[1] - fvMin[1];
    dz = fvMax[2] - fvMin[2];

    channel = 0;
    if (dy > dx && dy > dz) channel = 1;
    else if (dz > dx) channel = 2;

    fSep = 0.5f * (fvMax[channel] + fvMin[channel]);

    // terminate recursion when the separation/average value
    // is no longer strictly between fMin and fMax values.
    if (fSep >= fvMax[channel] || fSep <= fvMin[channel])
    {
        // complete the weld
        for (l = iL_in; l <= iR_in; l++)
        {
            int i = pTmpVert[l].index;
            const int index = piTriList_in_and_out[i];
            const Vector3f vP = GetPosition(pContext, index);
            const Vector3f vN = GetNormal(pContext, index);
            const Vector3f vT = GetTexCoord(pContext, index);

            bool bNotFound = true;
            int l2 = iL_in, i2rec = -1;
            while (l2 < l && bNotFound)
            {
                const int i2 = pTmpVert[l2].index;
                const int index2 = piTriList_in_and_out[i2];
                const Vector3f vP2 = GetPosition(pContext, index2);
                const Vector3f vN2 = GetNormal(pContext, index2);
                const Vector3f vT2 = GetTexCoord(pContext, index2);
                i2rec = i2;

                //if(vP==vP2 && vN==vN2 && vT==vT2)
                if (vP.x == vP2.x && vP.y == vP2.y && vP.z == vP2.z &&
                    vN.x == vN2.x && vN.y == vN2.y && vN.z == vN2.z &&
                    vT.x == vT2.x && vT.y == vT2.y && vT.z == vT2.z)
                    bNotFound = false;
                else
                    ++l2;
            }

            // merge if previously found
            if (!bNotFound)
                piTriList_in_and_out[i] = piTriList_in_and_out[i2rec];
        }
    }
    else
    {
        int iL = iL_in, iR = iR_in;
        //assert((iR_in - iL_in) > 0);	// at least 2 entries

        // separate (by fSep) all points between iL_in and iR_in in pTmpVert[]
        while (iL < iR)
        {
            bool bReadyLeftSwap = false, bReadyRightSwap = false;
            while ((!bReadyLeftSwap) && iL < iR)
            {
                //assert(iL >= iL_in && iL <= iR_in);
                bReadyLeftSwap = !(pTmpVert[iL].vert[channel] < fSep);
                if (!bReadyLeftSwap) ++iL;
            }
            while ((!bReadyRightSwap) && iL < iR)
            {
                //assert(iR >= iL_in && iR <= iR_in);
                bReadyRightSwap = pTmpVert[iR].vert[channel] < fSep;
                if (!bReadyRightSwap) --iR;
            }
            //assert((iL < iR) || !(bReadyLeftSwap && bReadyRightSwap));

            if (bReadyLeftSwap && bReadyRightSwap)
            {
                const STmpVert sTmp = pTmpVert[iL];
                //assert(iL < iR);
                pTmpVert[iL] = pTmpVert[iR];
                pTmpVert[iR] = sTmp;
                ++iL; --iR;
            }
        }

        //assert(iL == (iR + 1) || (iL == iR));
        if (iL == iR)
        {
            const bool bReadyRightSwap = pTmpVert[iR].vert[channel] < fSep;
            if (bReadyRightSwap) ++iL;
            else --iR;
        }

        // only need to weld when there is more than 1 instance of the (x,y,z)
        if (iL_in < iR)
            MergeVertsFast(piTriList_in_and_out, pTmpVert, pContext, iL_in, iR);	// weld all left of fSep
        if (iL < iR_in)
            MergeVertsFast(piTriList_in_and_out, pTmpVert, pContext, iL, iR_in);	// weld all right of (or equal to) fSep
    }
}

static void MergeVertsSlow(int piTriList_in_and_out[], const SMikkTSpaceContext* pContext, const int pTable[], const int iEntries)
{
    // this can be optimized further using a tree structure or more hashing.
    int e = 0;
    for (e = 0; e < iEntries; e++)
    {
        int i = pTable[e];
        const int index = piTriList_in_and_out[i];
        const Vector3f vP = GetPosition(pContext, index);
        const Vector3f vN = GetNormal(pContext, index);
        const Vector3f vT = GetTexCoord(pContext, index);

        bool bNotFound = true;
        int e2 = 0, i2rec = -1;
        while (e2 < e && bNotFound)
        {
            const int i2 = pTable[e2];
            const int index2 = piTriList_in_and_out[i2];
            const Vector3f vP2 = GetPosition(pContext, index2);
            const Vector3f vN2 = GetNormal(pContext, index2);
            const Vector3f vT2 = GetTexCoord(pContext, index2);
            i2rec = i2;

            if (veq(vP, vP2) && veq(vN, vN2) && veq(vT, vT2))
                bNotFound = false;
            else
                ++e2;
        }

        // merge if previously found
        if (!bNotFound)
            piTriList_in_and_out[i] = piTriList_in_and_out[i2rec];
    }
}


static void GenerateSharedVerticesIndexListSlow(int piTriList_in_and_out[], const SMikkTSpaceContext* pContext, const int iNrTrianglesIn)
{
    int iNumUniqueVerts = 0, t = 0, i = 0;
    for (t = 0; t < iNrTrianglesIn; t++)
    {
        for (i = 0; i < 3; i++)
        {
            const int offs = t * 3 + i;
            const int index = piTriList_in_and_out[offs];

            const Vector3f vP = GetPosition(pContext, index);
            const Vector3f vN = GetNormal(pContext, index);
            const Vector3f vT = GetTexCoord(pContext, index);

            bool bFound = false;
            int t2 = 0, index2rec = -1;
            while (!bFound && t2 <= t)
            {
                int j = 0;
                while (!bFound && j < 3)
                {
                    const int index2 = piTriList_in_and_out[t2 * 3 + j];
                    const Vector3f vP2 = GetPosition(pContext, index2);
                    const Vector3f vN2 = GetNormal(pContext, index2);
                    const Vector3f vT2 = GetTexCoord(pContext, index2);

                    if (veq(vP, vP2) && veq(vN, vN2) && veq(vT, vT2))
                        bFound = true;
                    else
                        ++j;
                }
                if (!bFound) ++t2;
            }

            //assert(bFound);
            // if we found our own
            if (index2rec == index) { ++iNumUniqueVerts; }

            piTriList_in_and_out[offs] = index2rec;
        }
    }
}


static void GenerateSharedVerticesIndexList(int piTriList_in_and_out[], const SMikkTSpaceContext* pContext, const int iNrTrianglesIn)
{

    // Generate bounding box
    int* piHashTable = NULL, * piHashCount = NULL, * piHashOffsets = NULL, * piHashCount2 = NULL;
    STmpVert* pTmpVert = NULL;
    int i = 0, iChannel = 0, k = 0, e = 0;
    int iMaxCount = 0;
    Vector3f vMin = GetPosition(pContext, 0), vMax = vMin, vDim;
    float fMin, fMax;
    for (i = 1; i < (iNrTrianglesIn * 3); i++)
    {
        const int index = piTriList_in_and_out[i];

        const Vector3f vP = GetPosition(pContext, index);
        if (vMin.x > vP.x) vMin.x = vP.x;
        else if (vMax.x < vP.x) vMax.x = vP.x;
        if (vMin.y > vP.y) vMin.y = vP.y;
        else if (vMax.y < vP.y) vMax.y = vP.y;
        if (vMin.z > vP.z) vMin.z = vP.z;
        else if (vMax.z < vP.z) vMax.z = vP.z;
    }

    vDim = vsub(vMax, vMin);
    iChannel = 0;
    fMin = vMin.x; fMax = vMax.x;
    if (vDim.y > vDim.x && vDim.y > vDim.z)
    {
        iChannel = 1;
        fMin = vMin.y, fMax = vMax.y;
    }
    else if (vDim.z > vDim.x)
    {
        iChannel = 2;
        fMin = vMin.z, fMax = vMax.z;
    }

    // make allocations
    piHashTable = (int*)malloc(sizeof(int) * iNrTrianglesIn * 3);
    piHashCount = (int*)malloc(sizeof(int) * g_iCells);
    piHashOffsets = (int*)malloc(sizeof(int) * g_iCells);
    piHashCount2 = (int*)malloc(sizeof(int) * g_iCells);

    if (piHashTable == NULL || piHashCount == NULL || piHashOffsets == NULL || piHashCount2 == NULL)
    {
        if (piHashTable != NULL) free(piHashTable);
        if (piHashCount != NULL) free(piHashCount);
        if (piHashOffsets != NULL) free(piHashOffsets);
        if (piHashCount2 != NULL) free(piHashCount2);
        GenerateSharedVerticesIndexListSlow(piTriList_in_and_out, pContext, iNrTrianglesIn);
        return;
    }
    memset(piHashCount, 0, sizeof(int) * g_iCells);
    memset(piHashCount2, 0, sizeof(int) * g_iCells);

    // count amount of elements in each cell unit
    for (i = 0; i < (iNrTrianglesIn * 3); i++)
    {
        const int index = piTriList_in_and_out[i];
        const Vector3f vP = GetPosition(pContext, index);
        const float fVal = iChannel == 0 ? vP.x : (iChannel == 1 ? vP.y : vP.z);
        const int iCell = FindGridCell(fMin, fMax, fVal);
        ++piHashCount[iCell];
    }

    // evaluate start index of each cell.
    piHashOffsets[0] = 0;
    for (k = 1; k < g_iCells; k++)
        piHashOffsets[k] = piHashOffsets[k - 1] + piHashCount[k - 1];

    // insert vertices
    for (i = 0; i < (iNrTrianglesIn * 3); i++)
    {
        const int index = piTriList_in_and_out[i];
        const Vector3f vP = GetPosition(pContext, index);
        const float fVal = iChannel == 0 ? vP.x : (iChannel == 1 ? vP.y : vP.z);
        const int iCell = FindGridCell(fMin, fMax, fVal);
        int* pTable = NULL;

        //assert(piHashCount2[iCell] < piHashCount[iCell]);
        pTable = &piHashTable[piHashOffsets[iCell]];
        pTable[piHashCount2[iCell]] = i;	// vertex i has been inserted.
        ++piHashCount2[iCell];
    }
    //for (k = 0; k < g_iCells; k++)
        //assert(piHashCount2[k] == piHashCount[k]);	// verify the count
    free(piHashCount2);

    // find maximum amount of entries in any hash entry
    iMaxCount = piHashCount[0];
    for (k = 1; k < g_iCells; k++)
        if (iMaxCount < piHashCount[k])
            iMaxCount = piHashCount[k];
    pTmpVert = (STmpVert*)malloc(sizeof(STmpVert) * iMaxCount);


    // complete the merge
    for (k = 0; k < g_iCells; k++)
    {
        // extract table of cell k and amount of entries in it
        int* pTable = &piHashTable[piHashOffsets[k]];
        const int iEntries = piHashCount[k];
        if (iEntries < 2) continue;

        if (pTmpVert != NULL)
        {
            for (e = 0; e < iEntries; e++)
            {
                int i = pTable[e];
                const Vector3f vP = GetPosition(pContext, piTriList_in_and_out[i]);
                pTmpVert[e].vert[0] = vP.x; pTmpVert[e].vert[1] = vP.y;
                pTmpVert[e].vert[2] = vP.z; pTmpVert[e].index = i;
            }
            MergeVertsFast(piTriList_in_and_out, pTmpVert, pContext, 0, iEntries - 1);
        }
        else
            MergeVertsSlow(piTriList_in_and_out, pContext, pTable, iEntries);
    }

    if (pTmpVert != NULL) { free(pTmpVert); }
    free(piHashTable);
    free(piHashCount);
    free(piHashOffsets);
}

static void DegenPrologue(STriInfo pTriInfos[], int piTriList_out[], const int iNrTrianglesIn, const int iTotTris)
{
    int iNextGoodTriangleSearchIndex = -1;
    bool bStillFindingGoodOnes;

    // locate quads with only one good triangle
    int t = 0;
    while (t < (iTotTris - 1))
    {
        const int iFO_a = pTriInfos[t].iOrgFaceNumber;
        const int iFO_b = pTriInfos[t + 1].iOrgFaceNumber;
        if (iFO_a == iFO_b)	// this is a quad
        {
            const bool bIsDeg_a = (pTriInfos[t].iFlag & MARK_DEGENERATE) != 0 ? true : false;
            const bool bIsDeg_b = (pTriInfos[t + 1].iFlag & MARK_DEGENERATE) != 0 ? true : false;
            if ((bIsDeg_a ^ bIsDeg_b) != 0)
            {
                pTriInfos[t].iFlag |= QUAD_ONE_DEGEN_TRI;
                pTriInfos[t + 1].iFlag |= QUAD_ONE_DEGEN_TRI;
            }
            t += 2;
        }
        else
            ++t;
    }

    // reorder list so all degen triangles are moved to the back
    // without reordering the good triangles
    iNextGoodTriangleSearchIndex = 1;
    t = 0;
    bStillFindingGoodOnes = true;
    while (t < iNrTrianglesIn && bStillFindingGoodOnes)
    {
        const bool bIsGood = (pTriInfos[t].iFlag & MARK_DEGENERATE) == 0 ? true : false;
        if (bIsGood)
        {
            if (iNextGoodTriangleSearchIndex < (t + 2))
                iNextGoodTriangleSearchIndex = t + 2;
        }
        else
        {
            int t0, t1;
            // search for the first good triangle.
            bool bJustADegenerate = true;
            while (bJustADegenerate && iNextGoodTriangleSearchIndex < iTotTris)
            {
                const bool bIsGood = (pTriInfos[iNextGoodTriangleSearchIndex].iFlag & MARK_DEGENERATE) == 0 ? true : false;
                if (bIsGood) bJustADegenerate = false;
                else ++iNextGoodTriangleSearchIndex;
            }

            t0 = t;
            t1 = iNextGoodTriangleSearchIndex;
            ++iNextGoodTriangleSearchIndex;
            //assert(iNextGoodTriangleSearchIndex > (t + 1));

            // swap triangle t0 and t1
            if (!bJustADegenerate)
            {
                int i = 0;
                for (i = 0; i < 3; i++)
                {
                    const int index = piTriList_out[t0 * 3 + i];
                    piTriList_out[t0 * 3 + i] = piTriList_out[t1 * 3 + i];
                    piTriList_out[t1 * 3 + i] = index;
                }
                {
                    const STriInfo tri_info = pTriInfos[t0];
                    pTriInfos[t0] = pTriInfos[t1];
                    pTriInfos[t1] = tri_info;
                }
            }
            else
                bStillFindingGoodOnes = false;	// this is not supposed to happen
        }

        if (bStillFindingGoodOnes) ++t;
    }

    //assert(bStillFindingGoodOnes);	// code will still work.
    //assert(iNrTrianglesIn == t);
}

static void BuildNeighborsSlow(STriInfo pTriInfos[], const int piTriListIn[], const int iNrTrianglesIn)
{
    int f = 0, i = 0;
    for (f = 0; f < iNrTrianglesIn; f++)
    {
        for (i = 0; i < 3; i++)
        {
            // if unassigned
            if (pTriInfos[f].FaceNeighbors[i] == -1)
            {
                const int i0_A = piTriListIn[f * 3 + i];
                const int i1_A = piTriListIn[f * 3 + (i < 2 ? (i + 1) : 0)];

                // search for a neighbor
                bool bFound = false;
                int t = 0, j = 0;
                while (!bFound && t < iNrTrianglesIn)
                {
                    if (t != f)
                    {
                        j = 0;
                        while (!bFound && j < 3)
                        {
                            // in rev order
                            const int i1_B = piTriListIn[t * 3 + j];
                            const int i0_B = piTriListIn[t * 3 + (j < 2 ? (j + 1) : 0)];
                            //assert(!(i0_A==i1_B && i1_A==i0_B));
                            if (i0_A == i0_B && i1_A == i1_B)
                                bFound = true;
                            else
                                ++j;
                        }
                    }

                    if (!bFound) ++t;
                }

                // assign neighbors
                if (bFound)
                {
                    pTriInfos[f].FaceNeighbors[i] = t;
                    //assert(pTriInfos[t].FaceNeighbors[j]==-1);
                    pTriInfos[t].FaceNeighbors[j] = f;
                }
            }
        }
    }
}

static void QuickSortEdges(SEdge* pSortBuffer, int iLeft, int iRight, const int channel, unsigned int uSeed)
{
    unsigned int t;
    int iL, iR, n, index, iMid;

    // early out
    SEdge sTmp;
    const int iElems = iRight - iLeft + 1;
    if (iElems < 2) return;
    else if (iElems == 2)
    {
        if (pSortBuffer[iLeft].array[channel] > pSortBuffer[iRight].array[channel])
        {
            sTmp = pSortBuffer[iLeft];
            pSortBuffer[iLeft] = pSortBuffer[iRight];
            pSortBuffer[iRight] = sTmp;
        }
        return;
    }

    // Random
    t = uSeed & 31;
    t = (uSeed << t) | (uSeed >> (32 - t));
    uSeed = uSeed + t + 3;
    // Random end

    iL = iLeft, iR = iRight;
    n = (iR - iL) + 1;
    //assert(n >= 0);
    index = (int)(uSeed % n);

    iMid = pSortBuffer[index + iL].array[channel];

    do
    {
        while (pSortBuffer[iL].array[channel] < iMid)
            ++iL;
        while (pSortBuffer[iR].array[channel] > iMid)
            --iR;

        if (iL <= iR)
        {
            sTmp = pSortBuffer[iL];
            pSortBuffer[iL] = pSortBuffer[iR];
            pSortBuffer[iR] = sTmp;
            ++iL; --iR;
        }
    } while (iL <= iR);

    if (iLeft < iR)
        QuickSortEdges(pSortBuffer, iLeft, iR, channel, uSeed);
    if (iL < iRight)
        QuickSortEdges(pSortBuffer, iL, iRight, channel, uSeed);
}


static void GetEdge(int* i0_out, int* i1_out, int* edgenum_out, const int indices[], const int i0_in, const int i1_in)
{
    *edgenum_out = -1;

    // test if first index is on the edge
    if (indices[0] == i0_in || indices[0] == i1_in)
    {
        // test if second index is on the edge
        if (indices[1] == i0_in || indices[1] == i1_in)
        {
            edgenum_out[0] = 0;	// first edge
            i0_out[0] = indices[0];
            i1_out[0] = indices[1];
        }
        else
        {
            edgenum_out[0] = 2;	// third edge
            i0_out[0] = indices[2];
            i1_out[0] = indices[0];
        }
    }
    else
    {
        // only second and third index is on the edge
        edgenum_out[0] = 1;	// second edge
        i0_out[0] = indices[1];
        i1_out[0] = indices[2];
    }
}

static void BuildNeighborsFast(STriInfo pTriInfos[], SEdge* pEdges, const int piTriListIn[], const int iNrTrianglesIn)
{
    // build array of edges
    unsigned int uSeed = INTERNAL_RND_SORT_SEED;				// could replace with a random seed?
    int iEntries = 0, iCurStartIndex = -1, f = 0, i = 0;
    for (f = 0; f < iNrTrianglesIn; f++)
        for (i = 0; i < 3; i++)
        {
            const int i0 = piTriListIn[f * 3 + i];
            const int i1 = piTriListIn[f * 3 + (i < 2 ? (i + 1) : 0)];
            pEdges[f * 3 + i].i0 = i0 < i1 ? i0 : i1;			// put minimum index in i0
            pEdges[f * 3 + i].i1 = !(i0 < i1) ? i0 : i1;		// put maximum index in i1
            pEdges[f * 3 + i].f = f;							// record face number
        }

    // sort over all edges by i0, this is the pricy one.
    QuickSortEdges(pEdges, 0, iNrTrianglesIn * 3 - 1, 0, uSeed);	// sort channel 0 which is i0

    // sub sort over i1, should be fast.
    // could replace this with a 64 bit int sort over (i0,i1)
    // with i0 as msb in the quicksort call above.
    iEntries = iNrTrianglesIn * 3;
    iCurStartIndex = 0;
    for (i = 1; i < iEntries; i++)
    {
        if (pEdges[iCurStartIndex].i0 != pEdges[i].i0)
        {
            const int iL = iCurStartIndex;
            const int iR = i - 1;
            //const int iElems = i-iL;
            iCurStartIndex = i;
            QuickSortEdges(pEdges, iL, iR, 1, uSeed);	// sort channel 1 which is i1
        }
    }

    // sub sort over f, which should be fast.
    // this step is to remain compliant with BuildNeighborsSlow() when
    // more than 2 triangles use the same edge (such as a butterfly topology).
    iCurStartIndex = 0;
    for (i = 1; i < iEntries; i++)
    {
        if (pEdges[iCurStartIndex].i0 != pEdges[i].i0 || pEdges[iCurStartIndex].i1 != pEdges[i].i1)
        {
            const int iL = iCurStartIndex;
            const int iR = i - 1;
            //const int iElems = i-iL;
            iCurStartIndex = i;
            QuickSortEdges(pEdges, iL, iR, 2, uSeed);	// sort channel 2 which is f
        }
    }

    // pair up, adjacent triangles
    for (i = 0; i < iEntries; i++)
    {
        const int i0 = pEdges[i].i0;
        const int i1 = pEdges[i].i1;
        const int f = pEdges[i].f;
        bool bUnassigned_A;

        int i0_A, i1_A;
        int edgenum_A, edgenum_B = 0;	// 0,1 or 2
        GetEdge(&i0_A, &i1_A, &edgenum_A, &piTriListIn[f * 3], i0, i1);	// resolve index ordering and edge_num
        bUnassigned_A = pTriInfos[f].FaceNeighbors[edgenum_A] == -1 ? true : false;

        if (bUnassigned_A)
        {
            // get true index ordering
            int j = i + 1, t;
            bool bNotFound = true;
            while (j < iEntries && i0 == pEdges[j].i0 && i1 == pEdges[j].i1 && bNotFound)
            {
                bool bUnassigned_B;
                int i0_B, i1_B;
                t = pEdges[j].f;
                // flip i0_B and i1_B
                GetEdge(&i1_B, &i0_B, &edgenum_B, &piTriListIn[t * 3], pEdges[j].i0, pEdges[j].i1);	// resolve index ordering and edge_num
                //assert(!(i0_A==i1_B && i1_A==i0_B));
                bUnassigned_B = pTriInfos[t].FaceNeighbors[edgenum_B] == -1 ? true : false;
                if (i0_A == i0_B && i1_A == i1_B && bUnassigned_B)
                    bNotFound = false;
                else
                    ++j;
            }

            if (!bNotFound)
            {
                int t = pEdges[j].f;
                pTriInfos[f].FaceNeighbors[edgenum_A] = t;
                //assert(pTriInfos[t].FaceNeighbors[edgenum_B]==-1);
                pTriInfos[t].FaceNeighbors[edgenum_B] = f;
            }
        }
    }
}


static void InitTriInfo(STriInfo pTriInfos[], const int piTriListIn[], const SMikkTSpaceContext* pContext, const int iNrTrianglesIn)
{
    int f = 0, i = 0, t = 0;
    // pTriInfos[f].iFlag is cleared in GenerateInitialVerticesIndexList() which is called before this function.

    // generate neighbor info list
    for (f = 0; f < iNrTrianglesIn; f++)
        for (i = 0; i < 3; i++)
        {
            pTriInfos[f].FaceNeighbors[i] = -1;
            pTriInfos[f].AssignedGroup[i] = NULL;

            pTriInfos[f].vOs.x = 0.0f; pTriInfos[f].vOs.y = 0.0f; pTriInfos[f].vOs.z = 0.0f;
            pTriInfos[f].vOt.x = 0.0f; pTriInfos[f].vOt.y = 0.0f; pTriInfos[f].vOt.z = 0.0f;
            pTriInfos[f].fMagS = 0;
            pTriInfos[f].fMagT = 0;

            // assumed bad
            pTriInfos[f].iFlag |= GROUP_WITH_ANY;
        }

    // evaluate first order derivatives
    for (f = 0; f < iNrTrianglesIn; f++)
    {
        // initial values
        const Vector3f v1 = GetPosition(pContext, piTriListIn[f * 3 + 0]);
        const Vector3f v2 = GetPosition(pContext, piTriListIn[f * 3 + 1]);
        const Vector3f v3 = GetPosition(pContext, piTriListIn[f * 3 + 2]);
        const Vector3f t1 = GetTexCoord(pContext, piTriListIn[f * 3 + 0]);
        const Vector3f t2 = GetTexCoord(pContext, piTriListIn[f * 3 + 1]);
        const Vector3f t3 = GetTexCoord(pContext, piTriListIn[f * 3 + 2]);

        const float t21x = t2.x - t1.x;
        const float t21y = t2.y - t1.y;
        const float t31x = t3.x - t1.x;
        const float t31y = t3.y - t1.y;
        const Vector3f d1 = vsub(v2, v1);
        const Vector3f d2 = vsub(v3, v1);

        const float fSignedAreaSTx2 = t21x * t31y - t21y * t31x;
        //assert(fSignedAreaSTx2!=0);
        Vector3f vOs = vsub(vscale(t31y, d1), vscale(t21y, d2));	// eq 18
        Vector3f vOt = vadd(vscale(-t31x, d1), vscale(t21x, d2)); // eq 19

        pTriInfos[f].iFlag |= (fSignedAreaSTx2 > 0 ? ORIENT_PRESERVING : 0);

        if (NotZero(fSignedAreaSTx2))
        {
            const float fAbsArea = fabsf(fSignedAreaSTx2);
            const float fLenOs = Length(vOs);
            const float fLenOt = Length(vOt);
            const float fS = (pTriInfos[f].iFlag & ORIENT_PRESERVING) == 0 ? (-1.0f) : 1.0f;
            if (NotZero(fLenOs)) pTriInfos[f].vOs = vscale(fS / fLenOs, vOs);
            if (NotZero(fLenOt)) pTriInfos[f].vOt = vscale(fS / fLenOt, vOt);

            // evaluate magnitudes prior to normalization of vOs and vOt
            pTriInfos[f].fMagS = fLenOs / fAbsArea;
            pTriInfos[f].fMagT = fLenOt / fAbsArea;

            // if this is a good triangle
            if (NotZero(pTriInfos[f].fMagS) && NotZero(pTriInfos[f].fMagT))
                pTriInfos[f].iFlag &= (~GROUP_WITH_ANY);
        }
    }

    // force otherwise healthy quads to a fixed orientation
    while (t < (iNrTrianglesIn - 1))
    {
        const int iFO_a = pTriInfos[t].iOrgFaceNumber;
        const int iFO_b = pTriInfos[t + 1].iOrgFaceNumber;
        if (iFO_a == iFO_b)	// this is a quad
        {
            const bool bIsDeg_a = (pTriInfos[t].iFlag & MARK_DEGENERATE) != 0 ? true : false;
            const bool bIsDeg_b = (pTriInfos[t + 1].iFlag & MARK_DEGENERATE) != 0 ? true : false;

            // bad triangles should already have been removed by
            // DegenPrologue(), but just in case check bIsDeg_a and bIsDeg_a are false
            if ((bIsDeg_a || bIsDeg_b) == false)
            {
                const bool bOrientA = (pTriInfos[t].iFlag & ORIENT_PRESERVING) != 0 ? true : false;
                const bool bOrientB = (pTriInfos[t + 1].iFlag & ORIENT_PRESERVING) != 0 ? true : false;
                // if this happens the quad has extremely bad mapping!!
                if (bOrientA != bOrientB)
                {
                    //printf("found quad with bad mapping\n");
                    bool bChooseOrientFirstTri = false;
                    if ((pTriInfos[t + 1].iFlag & GROUP_WITH_ANY) != 0) bChooseOrientFirstTri = true;
                    else if (CalcTexArea(pContext, &piTriListIn[t * 3 + 0]) >= CalcTexArea(pContext, &piTriListIn[(t + 1) * 3 + 0]))
                        bChooseOrientFirstTri = true;

                    // force match
                    {
                        const int t0 = bChooseOrientFirstTri ? t : (t + 1);
                        const int t1 = bChooseOrientFirstTri ? (t + 1) : t;
                        pTriInfos[t1].iFlag &= (~ORIENT_PRESERVING);	// clear first
                        pTriInfos[t1].iFlag |= (pTriInfos[t0].iFlag & ORIENT_PRESERVING);	// copy bit
                    }
                }
            }
            t += 2;
        }
        else
            ++t;
    }

    // match up edge pairs
    {
        SEdge* pEdges = (SEdge*)malloc(sizeof(SEdge) * iNrTrianglesIn * 3);
        if (pEdges == NULL)
            BuildNeighborsSlow(pTriInfos, piTriListIn, iNrTrianglesIn);
        else
        {
            BuildNeighborsFast(pTriInfos, pEdges, piTriListIn, iNrTrianglesIn);

            free(pEdges);
        }
    }
}

static void AddTriToGroup(SGroup* pGroup, const int iTriIndex)
{
    pGroup->pFaceIndices[pGroup->iNrFaces] = iTriIndex;
    ++pGroup->iNrFaces;
}

static bool AssignRecur(const int piTriListIn[], STriInfo psTriInfos[], const int iMyTriIndex, SGroup* pGroup)
{
    STriInfo* pMyTriInfo = &psTriInfos[iMyTriIndex];

    // track down vertex
    const int iVertRep = pGroup->iVertexRepresentitive;
    const int* pVerts = &piTriListIn[3 * iMyTriIndex + 0];
    int i = -1;
    if (pVerts[0] == iVertRep) i = 0;
    else if (pVerts[1] == iVertRep) i = 1;
    else if (pVerts[2] == iVertRep) i = 2;
    //assert(i >= 0 && i < 3);

    // early out
    if (pMyTriInfo->AssignedGroup[i] == pGroup) return TRUE;
    else if (pMyTriInfo->AssignedGroup[i] != NULL) return FALSE;
    if ((pMyTriInfo->iFlag & GROUP_WITH_ANY) != 0)
    {
        // first to group with a group-with-anything triangle
        // determines it's orientation.
        // This is the only existing order dependency in the code!!
        if (pMyTriInfo->AssignedGroup[0] == NULL &&
            pMyTriInfo->AssignedGroup[1] == NULL &&
            pMyTriInfo->AssignedGroup[2] == NULL)
        {
            pMyTriInfo->iFlag &= (~ORIENT_PRESERVING);
            pMyTriInfo->iFlag |= (pGroup->bOrientPreservering ? ORIENT_PRESERVING : 0);
        }
    }
    {
        const bool bOrient = (pMyTriInfo->iFlag & ORIENT_PRESERVING) != 0 ? TRUE : FALSE;
        if (bOrient != pGroup->bOrientPreservering) return FALSE;
    }

    AddTriToGroup(pGroup, iMyTriIndex);
    pMyTriInfo->AssignedGroup[i] = pGroup;

    {
        const int neigh_indexL = pMyTriInfo->FaceNeighbors[i];
        const int neigh_indexR = pMyTriInfo->FaceNeighbors[i > 0 ? (i - 1) : 2];
        if (neigh_indexL >= 0)
            AssignRecur(piTriListIn, psTriInfos, neigh_indexL, pGroup);
        if (neigh_indexR >= 0)
            AssignRecur(piTriListIn, psTriInfos, neigh_indexR, pGroup);
    }



    return TRUE;
}

static int Build4RuleGroups(STriInfo pTriInfos[], SGroup pGroups[], int piGroupTrianglesBuffer[], const int piTriListIn[], const int iNrTrianglesIn)
{
    const int iNrMaxGroups = iNrTrianglesIn * 3;
    int iNrActiveGroups = 0;
    int iOffset = 0, f = 0, i = 0;
    for (f = 0; f < iNrTrianglesIn; f++)
    {
        for (i = 0; i < 3; i++)
        {
            // if not assigned to a group
            if ((pTriInfos[f].iFlag & GROUP_WITH_ANY) == 0 && pTriInfos[f].AssignedGroup[i] == NULL)
            {
                bool bOrPre;
                int neigh_indexL, neigh_indexR;
                const int vert_index = piTriListIn[f * 3 + i];
                //assert(iNrActiveGroups < iNrMaxGroups);
                pTriInfos[f].AssignedGroup[i] = &pGroups[iNrActiveGroups];
                pTriInfos[f].AssignedGroup[i]->iVertexRepresentitive = vert_index;
                pTriInfos[f].AssignedGroup[i]->bOrientPreservering = (pTriInfos[f].iFlag & ORIENT_PRESERVING) != 0;
                pTriInfos[f].AssignedGroup[i]->iNrFaces = 0;
                pTriInfos[f].AssignedGroup[i]->pFaceIndices = &piGroupTrianglesBuffer[iOffset];
                ++iNrActiveGroups;

                AddTriToGroup(pTriInfos[f].AssignedGroup[i], f);
                bOrPre = (pTriInfos[f].iFlag & ORIENT_PRESERVING) != 0 ? TRUE : FALSE;
                neigh_indexL = pTriInfos[f].FaceNeighbors[i];
                neigh_indexR = pTriInfos[f].FaceNeighbors[i > 0 ? (i - 1) : 2];
                if (neigh_indexL >= 0) // neighbor
                {
                    const bool bAnswer =
                        AssignRecur(piTriListIn, pTriInfos, neigh_indexL,
                            pTriInfos[f].AssignedGroup[i]);

                    const bool bOrPre2 = (pTriInfos[neigh_indexL].iFlag & ORIENT_PRESERVING) != 0 ? TRUE : FALSE;
                    const bool bDiff = bOrPre != bOrPre2 ? TRUE : FALSE;
                    //assert(bAnswer || bDiff);
                }
                if (neigh_indexR >= 0) // neighbor
                {
                    const bool bAnswer =
                        AssignRecur(piTriListIn, pTriInfos, neigh_indexR,
                            pTriInfos[f].AssignedGroup[i]);

                    const bool bOrPre2 = (pTriInfos[neigh_indexR].iFlag & ORIENT_PRESERVING) != 0 ? TRUE : FALSE;
                    const bool bDiff = bOrPre != bOrPre2 ? TRUE : FALSE;
                    //assert(bAnswer || bDiff);
                }

                // update offset
                iOffset += pTriInfos[f].AssignedGroup[i]->iNrFaces;
                // since the groups are disjoint a triangle can never
                // belong to more than 3 groups. Subsequently something
                // is completely screwed if this assertion ever hits.
                //assert(iOffset <= iNrMaxGroups);
            }
        }
    }

    return iNrActiveGroups;
}

static bool CompareSubGroups(const SSubGroup* pg1, const SSubGroup* pg2)
{
    bool bStillSame = TRUE;
    int i = 0;
    if (pg1->iNrFaces != pg2->iNrFaces) return FALSE;
    while (i < pg1->iNrFaces && bStillSame)
    {
        bStillSame = pg1->pTriMembers[i] == pg2->pTriMembers[i] ? TRUE : FALSE;
        if (bStillSame) ++i;
    }
    return bStillSame;
}


static void QuickSort(int* pSortBuffer, int iLeft, int iRight, unsigned int uSeed)
{
    int iL, iR, n, index, iMid, iTmp;

    // Random
    unsigned int t = uSeed & 31;
    t = (uSeed << t) | (uSeed >> (32 - t));
    uSeed = uSeed + t + 3;
    // Random end

    iL = iLeft; iR = iRight;
    n = (iR - iL) + 1;
    //assert(n >= 0);
    index = (int)(uSeed % n);

    iMid = pSortBuffer[index + iL];


    do
    {
        while (pSortBuffer[iL] < iMid)
            ++iL;
        while (pSortBuffer[iR] > iMid)
            --iR;

        if (iL <= iR)
        {
            iTmp = pSortBuffer[iL];
            pSortBuffer[iL] = pSortBuffer[iR];
            pSortBuffer[iR] = iTmp;
            ++iL; --iR;
        }
    } while (iL <= iR);

    if (iLeft < iR)
        QuickSort(pSortBuffer, iLeft, iR, uSeed);
    if (iL < iRight)
        QuickSort(pSortBuffer, iL, iRight, uSeed);
}

static STSpace EvalTspace(int face_indices[], const int iFaces, const int piTriListIn[], const STriInfo pTriInfos[],
    const SMikkTSpaceContext* pContext, const int iVertexRepresentitive)
{
    STSpace res;
    float fAngleSum = 0;
    int face = 0;
    res.vOs.x = 0.0f; res.vOs.y = 0.0f; res.vOs.z = 0.0f;
    res.vOt.x = 0.0f; res.vOt.y = 0.0f; res.vOt.z = 0.0f;
    res.fMagS = 0; res.fMagT = 0;

    for (face = 0; face < iFaces; face++)
    {
        const int f = face_indices[face];

        // only valid triangles get to add their contribution
        if ((pTriInfos[f].iFlag & GROUP_WITH_ANY) == 0)
        {
            Vector3f n, vOs, vOt, p0, p1, p2, v1, v2;
            float fCos, fAngle, fMagS, fMagT;
            int i = -1, index = -1, i0 = -1, i1 = -1, i2 = -1;
            if (piTriListIn[3 * f + 0] == iVertexRepresentitive) i = 0;
            else if (piTriListIn[3 * f + 1] == iVertexRepresentitive) i = 1;
            else if (piTriListIn[3 * f + 2] == iVertexRepresentitive) i = 2;
            //assert(i >= 0 && i < 3);

            // project
            index = piTriListIn[3 * f + i];
            n = GetNormal(pContext, index);
            vOs = vsub(pTriInfos[f].vOs, vscale(vdot(n, pTriInfos[f].vOs), n));
            vOt = vsub(pTriInfos[f].vOt, vscale(vdot(n, pTriInfos[f].vOt), n));
            if (VNotZero(vOs)) vOs = Normalize(vOs);
            if (VNotZero(vOt)) vOt = Normalize(vOt);

            i2 = piTriListIn[3 * f + (i < 2 ? (i + 1) : 0)];
            i1 = piTriListIn[3 * f + i];
            i0 = piTriListIn[3 * f + (i > 0 ? (i - 1) : 2)];

            p0 = GetPosition(pContext, i0);
            p1 = GetPosition(pContext, i1);
            p2 = GetPosition(pContext, i2);
            v1 = vsub(p0, p1);
            v2 = vsub(p2, p1);

            // project
            v1 = vsub(v1, vscale(vdot(n, v1), n)); if (VNotZero(v1)) v1 = Normalize(v1);
            v2 = vsub(v2, vscale(vdot(n, v2), n)); if (VNotZero(v2)) v2 = Normalize(v2);

            // weight contribution by the angle
            // between the two edge vectors
            fCos = vdot(v1, v2); fCos = fCos > 1 ? 1 : (fCos < (-1) ? (-1) : fCos);
            fAngle = (const float)acos(fCos);
            fMagS = pTriInfos[f].fMagS;
            fMagT = pTriInfos[f].fMagT;

            res.vOs = vadd(res.vOs, vscale(fAngle, vOs));
            res.vOt = vadd(res.vOt, vscale(fAngle, vOt));
            res.fMagS += (fAngle * fMagS);
            res.fMagT += (fAngle * fMagT);
            fAngleSum += fAngle;
        }
    }

    // normalize
    if (VNotZero(res.vOs)) res.vOs = Normalize(res.vOs);
    if (VNotZero(res.vOt)) res.vOt = Normalize(res.vOt);
    if (fAngleSum > 0)
    {
        res.fMagS /= fAngleSum;
        res.fMagT /= fAngleSum;
    }

    return res;
}

static STSpace AvgTSpace(const STSpace* pTS0, const STSpace* pTS1)
{
    STSpace ts_res;

    // this if is important. Due to floating point precision
    // averaging when ts0==ts1 will cause a slight difference
    // which results in tangent space splits later on
    if (pTS0->fMagS == pTS1->fMagS && pTS0->fMagT == pTS1->fMagT &&
        veq(pTS0->vOs, pTS1->vOs) && veq(pTS0->vOt, pTS1->vOt))
    {
        ts_res.fMagS = pTS0->fMagS;
        ts_res.fMagT = pTS0->fMagT;
        ts_res.vOs = pTS0->vOs;
        ts_res.vOt = pTS0->vOt;
    }
    else
    {
        ts_res.fMagS = 0.5f * (pTS0->fMagS + pTS1->fMagS);
        ts_res.fMagT = 0.5f * (pTS0->fMagT + pTS1->fMagT);
        ts_res.vOs = vadd(pTS0->vOs, pTS1->vOs);
        ts_res.vOt = vadd(pTS0->vOt, pTS1->vOt);
        if (VNotZero(ts_res.vOs)) ts_res.vOs = Normalize(ts_res.vOs);
        if (VNotZero(ts_res.vOt)) ts_res.vOt = Normalize(ts_res.vOt);
    }

    return ts_res;
}

static bool GenerateTSpaces(STSpace psTspace[], const STriInfo pTriInfos[], const SGroup pGroups[],
    const int iNrActiveGroups, const int piTriListIn[], const float fThresCos,
    const SMikkTSpaceContext* pContext)
{
    STSpace* pSubGroupTspace = NULL;
    SSubGroup* pUniSubGroups = NULL;
    int* pTmpMembers = NULL;
    int iMaxNrFaces = 0, iUniqueTspaces = 0, g = 0, i = 0;
    for (g = 0; g < iNrActiveGroups; g++)
        if (iMaxNrFaces < pGroups[g].iNrFaces)
            iMaxNrFaces = pGroups[g].iNrFaces;

    if (iMaxNrFaces == 0) return TRUE;

    // make initial allocations
    pSubGroupTspace = (STSpace*)malloc(sizeof(STSpace) * iMaxNrFaces);
    pUniSubGroups = (SSubGroup*)malloc(sizeof(SSubGroup) * iMaxNrFaces);
    pTmpMembers = (int*)malloc(sizeof(int) * iMaxNrFaces);
    if (pSubGroupTspace == NULL || pUniSubGroups == NULL || pTmpMembers == NULL)
    {
        if (pSubGroupTspace != NULL) free(pSubGroupTspace);
        if (pUniSubGroups != NULL) free(pUniSubGroups);
        if (pTmpMembers != NULL) free(pTmpMembers);
        return FALSE;
    }


    iUniqueTspaces = 0;
    for (g = 0; g < iNrActiveGroups; g++)
    {
        const SGroup* pGroup = &pGroups[g];
        int iUniqueSubGroups = 0, s = 0;

        for (i = 0; i < pGroup->iNrFaces; i++)	// triangles
        {
            const int f = pGroup->pFaceIndices[i];	// triangle number
            int index = -1, iVertIndex = -1, iOF_1 = -1, iMembers = 0, j = 0, l = 0;
            SSubGroup tmp_group;
            bool bFound;
            Vector3f n, vOs, vOt;
            if (pTriInfos[f].AssignedGroup[0] == pGroup) index = 0;
            else if (pTriInfos[f].AssignedGroup[1] == pGroup) index = 1;
            else if (pTriInfos[f].AssignedGroup[2] == pGroup) index = 2;
            //assert(index >= 0 && index < 3);

            iVertIndex = piTriListIn[f * 3 + index];
            //assert(iVertIndex == pGroup->iVertexRepresentitive);

            // is normalized already
            n = GetNormal(pContext, iVertIndex);

            // project
            vOs = vsub(pTriInfos[f].vOs, vscale(vdot(n, pTriInfos[f].vOs), n));
            vOt = vsub(pTriInfos[f].vOt, vscale(vdot(n, pTriInfos[f].vOt), n));
            if (VNotZero(vOs)) vOs = Normalize(vOs);
            if (VNotZero(vOt)) vOt = Normalize(vOt);

            // original face number
            iOF_1 = pTriInfos[f].iOrgFaceNumber;

            iMembers = 0;
            for (j = 0; j < pGroup->iNrFaces; j++)
            {
                const int t = pGroup->pFaceIndices[j];	// triangle number
                const int iOF_2 = pTriInfos[t].iOrgFaceNumber;

                // project
                Vector3f vOs2 = vsub(pTriInfos[t].vOs, vscale(vdot(n, pTriInfos[t].vOs), n));
                Vector3f vOt2 = vsub(pTriInfos[t].vOt, vscale(vdot(n, pTriInfos[t].vOt), n));
                if (VNotZero(vOs2)) vOs2 = Normalize(vOs2);
                if (VNotZero(vOt2)) vOt2 = Normalize(vOt2);

                {
                    const bool bAny = ((pTriInfos[f].iFlag | pTriInfos[t].iFlag) & GROUP_WITH_ANY) != 0 ? true : false;
                    // make sure triangles which belong to the same quad are joined.
                    const bool bSameOrgFace = iOF_1 == iOF_2 ? true : false;

                    const float fCosS = vdot(vOs, vOs2);
                    const float fCosT = vdot(vOt, vOt2);

                    //assert(f != t || bSameOrgFace);	// sanity check
                    if (bAny || bSameOrgFace || (fCosS > fThresCos && fCosT > fThresCos))
                        pTmpMembers[iMembers++] = t;
                }
            }

            // sort pTmpMembers
            tmp_group.iNrFaces = iMembers;
            tmp_group.pTriMembers = pTmpMembers;
            if (iMembers > 1)
            {
                unsigned int uSeed = INTERNAL_RND_SORT_SEED;	// could replace with a random seed?
                QuickSort(pTmpMembers, 0, iMembers - 1, uSeed);
            }

            // look for an existing match
            bFound = FALSE;
            l = 0;
            while (l < iUniqueSubGroups && !bFound)
            {
                bFound = CompareSubGroups(&tmp_group, &pUniSubGroups[l]);
                if (!bFound) ++l;
            }

            // assign tangent space index
            //assert(bFound || l == iUniqueSubGroups);
            //piTempTangIndices[f*3+index] = iUniqueTspaces+l;

            // if no match was found we allocate a new subgroup
            if (!bFound)
            {
                // insert new subgroup
                int* pIndices = (int*)malloc(sizeof(int) * iMembers);
                if (pIndices == NULL)
                {
                    // clean up and return false
                    int s = 0;
                    for (s = 0; s < iUniqueSubGroups; s++)
                        free(pUniSubGroups[s].pTriMembers);
                    free(pUniSubGroups);
                    free(pTmpMembers);
                    free(pSubGroupTspace);
                    return false;
                }
                pUniSubGroups[iUniqueSubGroups].iNrFaces = iMembers;
                pUniSubGroups[iUniqueSubGroups].pTriMembers = pIndices;
                memcpy(pIndices, tmp_group.pTriMembers, iMembers * sizeof(int));
                pSubGroupTspace[iUniqueSubGroups] =
                    EvalTspace(tmp_group.pTriMembers, iMembers, piTriListIn, pTriInfos, pContext, pGroup->iVertexRepresentitive);
                ++iUniqueSubGroups;
            }

            // output tspace
            {
                const int iOffs = pTriInfos[f].iTSpacesOffs;
                const int iVert = pTriInfos[f].vert_num[index];
                STSpace* pTS_out = &psTspace[iOffs + iVert];
                //assert(pTS_out->iCounter < 2);
                //assert(((pTriInfos[f].iFlag & ORIENT_PRESERVING) != 0) == pGroup->bOrientPreservering);
                if (pTS_out->iCounter == 1)
                {
                    *pTS_out = AvgTSpace(pTS_out, &pSubGroupTspace[l]);
                    pTS_out->iCounter = 2;	// update counter
                    pTS_out->bOrient = pGroup->bOrientPreservering;
                }
                else
                {
                    //assert(pTS_out->iCounter == 0);
                    *pTS_out = pSubGroupTspace[l];
                    pTS_out->iCounter = 1;	// update counter
                    pTS_out->bOrient = pGroup->bOrientPreservering;
                }
            }
        }

        // clean up and offset iUniqueTspaces
        for (s = 0; s < iUniqueSubGroups; s++)
            free(pUniSubGroups[s].pTriMembers);
        iUniqueTspaces += iUniqueSubGroups;
        iUniqueSubGroups = 0;
    }

    // clean up
    free(pUniSubGroups);
    free(pTmpMembers);
    free(pSubGroupTspace);

    return TRUE;
}

static void DegenEpilogue(STSpace psTspace[], STriInfo pTriInfos[], int piTriListIn[], const SMikkTSpaceContext* pContext, const int iNrTrianglesIn, const int iTotTris)
{
    int t = 0, i = 0;
    // deal with degenerate triangles
    // punishment for degenerate triangles is O(N^2)
    for (t = iNrTrianglesIn; t < iTotTris; t++)
    {
        // degenerate triangles on a quad with one good triangle are skipped
        // here but processed in the next loop
        const bool bSkip = (pTriInfos[t].iFlag & QUAD_ONE_DEGEN_TRI) != 0 ? TRUE : FALSE;

        if (!bSkip)
        {
            for (i = 0; i < 3; i++)
            {
                const int index1 = piTriListIn[t * 3 + i];
                // search through the good triangles
                bool bNotFound = TRUE;
                int j = 0;
                while (bNotFound && j < (3 * iNrTrianglesIn))
                {
                    const int index2 = piTriListIn[j];
                    if (index1 == index2) bNotFound = FALSE;
                    else ++j;
                }

                if (!bNotFound)
                {
                    const int iTri = j / 3;
                    const int iVert = j % 3;
                    const int iSrcVert = pTriInfos[iTri].vert_num[iVert];
                    const int iSrcOffs = pTriInfos[iTri].iTSpacesOffs;
                    const int iDstVert = pTriInfos[t].vert_num[i];
                    const int iDstOffs = pTriInfos[t].iTSpacesOffs;

                    // copy tspace
                    psTspace[iDstOffs + iDstVert] = psTspace[iSrcOffs + iSrcVert];
                }
            }
        }
    }

    // deal with degenerate quads with one good triangle
    for (t = 0; t < iNrTrianglesIn; t++)
    {
        // this triangle belongs to a quad where the
        // other triangle is degenerate
        if ((pTriInfos[t].iFlag & QUAD_ONE_DEGEN_TRI) != 0)
        {
            Vector3f vDstP;
            int iOrgF = -1, i = 0;
            bool bNotFound;
            unsigned char* pV = pTriInfos[t].vert_num;
            int iFlag = (1 << pV[0]) | (1 << pV[1]) | (1 << pV[2]);
            int iMissingIndex = 0;
            if ((iFlag & 2) == 0) iMissingIndex = 1;
            else if ((iFlag & 4) == 0) iMissingIndex = 2;
            else if ((iFlag & 8) == 0) iMissingIndex = 3;

            iOrgF = pTriInfos[t].iOrgFaceNumber;
            vDstP = GetPosition(pContext, MakeIndex(iOrgF, iMissingIndex));
            bNotFound = TRUE;
            i = 0;
            while (bNotFound && i < 3)
            {
                const int iVert = pV[i];
                const Vector3f vSrcP = GetPosition(pContext, MakeIndex(iOrgF, iVert));
                if (veq(vSrcP, vDstP) == TRUE)
                {
                    const int iOffs = pTriInfos[t].iTSpacesOffs;
                    psTspace[iOffs + iMissingIndex] = psTspace[iOffs + iVert];
                    bNotFound = FALSE;
                }
                else
                    ++i;
            }
            //assert(!bNotFound);
        }
    }
}

bool genTangSpace(const SMikkTSpaceContext* pContext, const float fAngularThreshold)
{
    // count nr_triangles
    int* piTriListIn = NULL, * piGroupTrianglesBuffer = NULL;
    STriInfo* pTriInfos = NULL;
    SGroup* pGroups = NULL;
    STSpace* psTspace = NULL;
    int iNrTrianglesIn = 0, f = 0, t = 0, i = 0;
    int iNrTSPaces = 0, iTotTris = 0, iDegenTriangles = 0, iNrMaxGroups = 0;
    int iNrActiveGroups = 0, index = 0;
    const int iNrFaces = pContext->m_pInterface->m_getNumFaces(pContext);
    bool bRes = false;
    const float fThresCos = (const float)cos((fAngularThreshold * M_PI) / 180);

    // verify all call-backs have been set
    if (pContext->m_pInterface->m_getNumFaces == NULL ||
        pContext->m_pInterface->m_getNumVerticesOfFace == NULL ||
        pContext->m_pInterface->m_getPosition == NULL ||
        pContext->m_pInterface->m_getNormal == NULL ||
        pContext->m_pInterface->m_getTexCoord == NULL)
        return false;

    // count triangles on supported faces
    for (f = 0; f < iNrFaces; f++)
    {
        const int verts = pContext->m_pInterface->m_getNumVerticesOfFace(pContext, f);
        if (verts == 3) ++iNrTrianglesIn;
        else if (verts == 4) iNrTrianglesIn += 2;
    }
    if (iNrTrianglesIn <= 0) return false;

    // allocate memory for an index list
    piTriListIn = (int*)malloc(sizeof(int) * 3 * iNrTrianglesIn);
    pTriInfos = (STriInfo*)malloc(sizeof(STriInfo) * iNrTrianglesIn);
    if (piTriListIn == NULL || pTriInfos == NULL)
    {
        if (piTriListIn != NULL) free(piTriListIn);
        if (pTriInfos != NULL) free(pTriInfos);
        return false;
    }

    // make an initial triangle --> face index list
    iNrTSPaces = GenerateInitialVerticesIndexList(pTriInfos, piTriListIn, pContext, iNrTrianglesIn);

    // make a welded index list of identical positions and attributes (pos, norm, texc)
    //printf("gen welded index list begin\n");
    GenerateSharedVerticesIndexList(piTriListIn, pContext, iNrTrianglesIn);
    //printf("gen welded index list end\n");

    // Mark all degenerate triangles
    iTotTris = iNrTrianglesIn;
    iNrTrianglesIn = 0;
    iDegenTriangles = 0;
    for (t = 0; t < iTotTris; t++)
    {
        const int i0 = piTriListIn[t * 3 + 0];
        const int i1 = piTriListIn[t * 3 + 1];
        const int i2 = piTriListIn[t * 3 + 2];
        const Vector3f p0 = GetPosition(pContext, i0);
        const Vector3f p1 = GetPosition(pContext, i1);
        const Vector3f p2 = GetPosition(pContext, i2);
        if (veq(p0, p1) || veq(p0, p2) || veq(p1, p2))	// degenerate
        {
            pTriInfos[t].iFlag |= MARK_DEGENERATE;
            ++iDegenTriangles;
        }
    }
    iNrTrianglesIn = iTotTris - iDegenTriangles;

    // mark all triangle pairs that belong to a quad with only one
    // good triangle. These need special treatment in DegenEpilogue().
    // Additionally, move all good triangles to the start of
    // pTriInfos[] and piTriListIn[] without changing order and
    // put the degenerate triangles last.
    DegenPrologue(pTriInfos, piTriListIn, iNrTrianglesIn, iTotTris);


    // evaluate triangle level attributes and neighbor list
    //printf("gen neighbors list begin\n");
    InitTriInfo(pTriInfos, piTriListIn, pContext, iNrTrianglesIn);
    //printf("gen neighbors list end\n");


    // based on the 4 rules, identify groups based on connectivity
    iNrMaxGroups = iNrTrianglesIn * 3;
    pGroups = (SGroup*)malloc(sizeof(SGroup) * iNrMaxGroups);
    piGroupTrianglesBuffer = (int*)malloc(sizeof(int) * iNrTrianglesIn * 3);
    if (pGroups == NULL || piGroupTrianglesBuffer == NULL)
    {
        if (pGroups != NULL) free(pGroups);
        if (piGroupTrianglesBuffer != NULL) free(piGroupTrianglesBuffer);
        free(piTriListIn);
        free(pTriInfos);
        return false;
    }
    //printf("gen 4rule groups begin\n");
    iNrActiveGroups =
        Build4RuleGroups(pTriInfos, pGroups, piGroupTrianglesBuffer, piTriListIn, iNrTrianglesIn);
    //printf("gen 4rule groups end\n");

    //

    psTspace = (STSpace*)malloc(sizeof(STSpace) * iNrTSPaces);
    if (psTspace == NULL)
    {
        free(piTriListIn);
        free(pTriInfos);
        free(pGroups);
        free(piGroupTrianglesBuffer);
        return false;
    }
    memset(psTspace, 0, sizeof(STSpace) * iNrTSPaces);
    for (t = 0; t < iNrTSPaces; t++)
    {
        psTspace[t].vOs.x = 1.0f; psTspace[t].vOs.y = 0.0f; psTspace[t].vOs.z = 0.0f; psTspace[t].fMagS = 1.0f;
        psTspace[t].vOt.x = 0.0f; psTspace[t].vOt.y = 1.0f; psTspace[t].vOt.z = 0.0f; psTspace[t].fMagT = 1.0f;
    }

    // make tspaces, each group is split up into subgroups if necessary
    // based on fAngularThreshold. Finally a tangent space is made for
    // every resulting subgroup
    //printf("gen tspaces begin\n");
    bRes = GenerateTSpaces(psTspace, pTriInfos, pGroups, iNrActiveGroups, piTriListIn, fThresCos, pContext);
    //printf("gen tspaces end\n");

    // clean up
    free(pGroups);
    free(piGroupTrianglesBuffer);

    if (!bRes)	// if an allocation in GenerateTSpaces() failed
    {
        // clean up and return false
        free(pTriInfos); free(piTriListIn); free(psTspace);
        return false;
    }


    // degenerate quads with one good triangle will be fixed by copying a space from
    // the good triangle to the coinciding vertex.
    // all other degenerate triangles will just copy a space from any good triangle
    // with the same welded index in piTriListIn[].
    DegenEpilogue(psTspace, pTriInfos, piTriListIn, pContext, iNrTrianglesIn, iTotTris);

    free(pTriInfos); free(piTriListIn);

    index = 0;
    for (f = 0; f < iNrFaces; f++)
    {
        const int verts = pContext->m_pInterface->m_getNumVerticesOfFace(pContext, f);
        if (verts != 3 && verts != 4) continue;


        // I've decided to let degenerate triangles and group-with-anythings
        // vary between left/right hand coordinate systems at the vertices.
        // All healthy triangles on the other hand are built to always be either or.

        /*// force the coordinate system orientation to be uniform for every face.
        // (this is already the case for good triangles but not for
        // degenerate ones and those with bGroupWithAnything==true)
        bool bOrient = psTspace[index].bOrient;
        if(psTspace[index].iCounter == 0)	// tspace was not derived from a group
        {
            // look for a space created in GenerateTSpaces() by iCounter>0
            bool bNotFound = true;
            int i=1;
            while(i<verts && bNotFound)
            {
                if(psTspace[index+i].iCounter > 0) bNotFound=false;
                else ++i;
            }
            if(!bNotFound) bOrient = psTspace[index+i].bOrient;
        }*/

        // set data
        for (i = 0; i < verts; i++)
        {
            const STSpace* pTSpace = &psTspace[index];
            float tang[] = { pTSpace->vOs.x, pTSpace->vOs.y, pTSpace->vOs.z };
            float bitang[] = { pTSpace->vOt.x, pTSpace->vOt.y, pTSpace->vOt.z };
            if (pContext->m_pInterface->m_setTSpace != NULL)
                pContext->m_pInterface->m_setTSpace(pContext, tang, bitang, pTSpace->fMagS, pTSpace->fMagT, pTSpace->bOrient, f, i);
            if (pContext->m_pInterface->m_setTSpaceBasic != NULL)
                pContext->m_pInterface->m_setTSpaceBasic(pContext, tang, pTSpace->bOrient == true ? 1.0f : (-1.0f), f, i);

            ++index;
        }
    }

    free(psTspace);


    return true;
}


bool genTangSpaceDefault(const SMikkTSpaceContext* pContext)
{
    return genTangSpace(pContext, 180.0f);
}


static void GenerateMikkTSpace(FBXImportMesh& mesh)
{
    const uint32_t uFC = mesh.polygonSizes.size();
    std::vector<uint32_t> faceOffsets(uFC);
    for (int i = 0, idx = 0; i < uFC; i++)
    {
        faceOffsets[i] = idx;
        idx += mesh.polygonSizes[i];
    }

    mesh.tangents.resize(mesh.polygons.size());

    TSpaceMeshInfo mesh_info;
    mesh_info.ImpMesh = &mesh;
    mesh_info.FaceOffs = &faceOffsets[0];

    // base mesh shape
    mesh_info.vertices = &mesh.vertices[0];
    mesh_info.normals = &mesh.normals[0];
    mesh_info.outTangents = &mesh.tangents[0];

    // setup the interface for mikktspace generator
    SMikkTSpaceInterface Interface;
    memset(reinterpret_cast<void*>(&Interface), 0, sizeof(SMikkTSpaceInterface));

    Interface.m_getNumFaces = getNumFaces;
    Interface.m_getNumVerticesOfFace = getNumVerticesOfFace;
    Interface.m_getPosition = getPosition;
    Interface.m_getNormal = getNormal;
    Interface.m_getTexCoord = getTexCoord;
    Interface.m_setTSpace = setTSpace;

    SMikkTSpaceContext Context;
    memset(reinterpret_cast<void*>(&Context), 0, sizeof(SMikkTSpaceContext));
    Context.m_pUserData = reinterpret_cast<void*>(&mesh_info);
    Context.m_pInterface = &Interface;

    // Return result to input Mesh format
    bool bRes = genTangSpaceDefault(&Context) != 0;

}
