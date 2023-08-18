#pragma once

#include "GL/glut.h"
#include "Utility.h"
#define TESS_FUNCTION_CALLCONV CALLBACK

static bool foundIntersectingEdge;

void TESS_FUNCTION_CALLCONV DummyEmitEdgeFlag(GLboolean flag);
void TESS_FUNCTION_CALLCONV EmitVertex(void* vertexData, void* userData);
void TESS_FUNCTION_CALLCONV CombineCallback(GLdouble coords[3], void* vertex_data[4], GLfloat weight[4], void** outData);


// This forces output to be triangles
void TESS_FUNCTION_CALLCONV DummyEmitEdgeFlag(GLboolean flag) {}
void TESS_FUNCTION_CALLCONV CombineCallback(GLdouble coords[3], void* d[4], GLfloat w[4], void** outData)
{
    foundIntersectingEdge = true;
}


void EmitVertex(void* vertexData, void* userData)
{
    intptr_t index = (intptr_t)vertexData;
    FBXTriangulationData* meshes = (FBXTriangulationData*)userData;
    const FBXImportMesh& input = *meshes->input;
    FBXImportMesh& output = *meshes->output;

    const bool emitNormals = meshes->setting->importNormal;
    const bool emitTangents = meshes->setting->importTargent;

    output.polygons.push_back(input.polygons[index]);

    // Add wedge data
    if (emitNormals && !input.normals.empty())
        output.normals.push_back(input.normals[index]);
    if (emitTangents && !input.tangents.empty())
        output.tangents.push_back(input.tangents[index]);
    for (int uvIndex = 0; uvIndex < 2; uvIndex++)
        if (!input.uvs[uvIndex].empty())
            output.uvs[uvIndex].push_back(input.uvs[uvIndex][index]);
    if (!input.colors.empty())
        output.colors.push_back(input.colors[index]);
}



bool Triangulate(const FBXImportMesh& input, FBXImportMesh& output, const ImportMeshSetting& settings, bool keepQuads)
{
    // We have a triangulated mesh.
    if (input.polygonSizes.empty())
    {
        output = input;
        output.polygonSizes.resize(input.polygons.size() / 3, 3);  // set all polygon sizes to 3
        output.hasAnyQuads = false;
        return output.polygons.size() % 3 == 0;
    }

    {
        // calculate and reserve as much as it actually will actually need
        int faceCount = 0;
        int vertexCount = 0;
        for (int i = 0, size = input.polygonSizes.size(); i < size; ++i)
        {
            // we will skip all polygons which have less than 3 vertices in the next loop
            int faceSize = input.polygonSizes[i];
            if (keepQuads && faceSize == 4)
            {
                ++faceCount;
                vertexCount += 4;
            }
            else if (faceSize >= 3)
            {
                faceCount += faceSize - 2;
                vertexCount += (faceSize - 2) * 3;
            }
        }

        output.polygons.reserve(vertexCount);
        if (settings.importNormal && !input.normals.empty())
            output.normals.reserve(vertexCount);
        if (settings.importTargent && !input.tangents.empty())
            output.tangents.reserve(vertexCount);
        for (int uvIndex = 0; uvIndex < 2; uvIndex++)
            if (!input.uvs[uvIndex].empty())
                output.uvs[uvIndex].reserve(vertexCount);
        if (!input.colors.empty())
            output.colors.reserve(vertexCount);
        if (!input.materials.empty())
            output.materials.reserve(faceCount);
        output.polygonSizes.reserve(faceCount);
        output.hasAnyQuads = false;

    }


    GLUtesselator* tesselator = gluNewTess();
    gluTessCallback(tesselator, GLU_TESS_EDGE_FLAG, reinterpret_cast<GLvoid(CALLBACK*)()>(DummyEmitEdgeFlag));
    gluTessCallback(tesselator, GLU_TESS_VERTEX_DATA, reinterpret_cast<GLvoid(CALLBACK*)()>(EmitVertex));
    gluTessCallback(tesselator, GLU_TESS_COMBINE, reinterpret_cast<GLvoid(CALLBACK*)()>(CombineCallback));


    FBXTriangulationData data;
    data.input = &input;
    data.output = &output;
    data.setting = &settings;

    int index = 0;
    int faceIndex = 0;
    for (int p = 0; p < input.polygonSizes.size(); p++)
    {
        int polygonSize = input.polygonSizes[p];
        if (index + polygonSize > input.polygons.size())
            return false;

        int nextFaceIndex = faceIndex;

        if (polygonSize == 3)
        {
            EmitVertex((void*)(intptr_t)index, &data); index++;
            EmitVertex((void*)(intptr_t)index, &data); index++;
            EmitVertex((void*)(intptr_t)index, &data); index++;
            nextFaceIndex += 1;
            output.polygonSizes.push_back(3);
        }
        else if (keepQuads && polygonSize == 4)
        {
            EmitVertex((void*)(intptr_t)index, &data); index++;
            EmitVertex((void*)(intptr_t)index, &data); index++;
            EmitVertex((void*)(intptr_t)index, &data); index++;
            EmitVertex((void*)(intptr_t)index, &data); index++;
            nextFaceIndex += 1;
            output.polygonSizes.push_back(4);
            output.hasAnyQuads = true;
        }
        else if (polygonSize == 4)
        {
            int curIndex = index;
            curIndex = index + 0;
            EmitVertex((void*)(intptr_t)curIndex, &data);
            curIndex = index + 1;
            EmitVertex((void*)(intptr_t)curIndex, &data);
            curIndex = index + 2;
            EmitVertex((void*)(intptr_t)curIndex, &data);

            curIndex = index + 0;
            EmitVertex((void*)(intptr_t)curIndex, &data);
            curIndex = index + 2;
            EmitVertex((void*)(intptr_t)curIndex, &data);
            curIndex = index + 3;
            EmitVertex((void*)(intptr_t)curIndex, &data);


            index += 4;
            nextFaceIndex += 2;
            output.polygonSizes.push_back(3);
            output.polygonSizes.push_back(3);
        }
        else
        {
            foundIntersectingEdge = false;

            size_t curPolySize = output.polygons.size();
            gluTessBeginPolygon(tesselator, &data);
            gluTessBeginContour(tesselator);

            for (int v = 0; v < polygonSize; v++)
            {
                int vertexIndex = input.polygons[index];
                Vector3f vertex = input.vertices[vertexIndex];
                double gluVertex[3] = { vertex.x, vertex.y, vertex.z };
                gluTessVertex(tesselator, gluVertex, (void*)(intptr_t)index);
                index++;
            }

            gluTessEndContour(tesselator);
            gluTessEndPolygon(tesselator);

            if (foundIntersectingEdge)
            {
                std::cout << " A polygon of %s is self-intersecting and has been discarded. " + input.name << std::endl;
            }

            int newTris = (output.polygons.size() - curPolySize) / 3;
            for (int i = 0; i < newTris; ++i)
                output.polygonSizes.push_back(3);
            nextFaceIndex += newTris;

        }

        // Fill up material
        if (!input.materials.empty())
        {
            for (int i = faceIndex; i < nextFaceIndex; ++i)
                output.materials.push_back(input.materials[p]);
        }

        faceIndex = nextFaceIndex;
    }

    gluDeleteTess(tesselator);

    output.vertices = input.vertices;
    output.name = input.name;

    return true;
}