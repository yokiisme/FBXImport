#pragma once
#include <fbxsdk.h>
#include "Utility.h"


void WriteMeshFile(FBXGameObject* gameObj, const char* outdir);
void WriteMeshFileNew(FBXGameObject* gameObj, FBXImportScene& importScene, const char* outdir);

//-----------------------------------Output test-----------------------------------
void WriteMeshOutputFiles(FBXMesh& mesh, std::vector<int>& lodMeshMaterials, char* outdir);
void WriteFBXMeshOutputFiles(const FBXImportMesh& mesh, char* outdir);
void WriteSceneOutputFiles(FBXImportScene& outputScene, char* outdir);
//-----------------------------------Output test-----------------------------------


void WriteMeshFile(FBXGameObject* gameObj, const char* outdir)
{
    if (_access(outdir, 0) == -1)
    {
        _mkdir(outdir);
    }

    uint64_t meshcount = gameObj->meshCount;
    std::vector<std::string> materialList;
    for (int i = 0; i < meshcount; i++)
    {
        std::vector<int> materialID;
        materialID.assign(gameObj->meshList[i].materialindex.begin(), gameObj->meshList[i].materialindex.end());
        for (int j = 0; j < gameObj->meshList[i].materials.size(); j++)
        {
            if (std::find(materialList.begin(), materialList.end(), gameObj->meshList[i].materials[j]) == materialList.end())
            {
                materialList.push_back(gameObj->meshList[i].materials[j]);
            }

            for (int matindex = 0; matindex < materialList.size(); matindex++)
            {
                if (materialList[matindex] == gameObj->meshList[i].materials[j])
                {
                    materialID[gameObj->meshList[i].materialindex[j]] = matindex;
                }
            }
        }

        gameObj->meshList[i].materialindex.clear();
        gameObj->meshList[i].materialindex.assign(materialID.begin(), materialID.end());
    }

    std::string materialname = "";
    for (int i = 0; i < materialList.size(); i++)
    {
        materialname += materialList[i] + "\n";
    }

    std::string meshname = "";
    std::vector<int32_t> meshtomatindex;
    std::string directory(outdir);

    for (int i = 0; i < meshcount; i++)
    {
        std::string meshfilename(gameObj->meshList[i].name);
        meshfilename = directory + "/" + meshfilename + ".~@FFFUB";
        meshname += meshfilename + "\n";
        std::ofstream osData(meshfilename, std::ios_base::out | std::ios_base::binary);
        uint64_t namecount = strlen(gameObj->meshList[i].name) + 1;
        osData.write(reinterpret_cast<char*>(&namecount), 8);
        osData.write(gameObj->meshList[i].name, namecount);
        uint64_t vertexcount = gameObj->meshList[i].vertices.size();
        osData.write(reinterpret_cast<char*>(&vertexcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].vertices.data()), vertexcount * sizeof(Vector3f));
        uint64_t normalcount = gameObj->meshList[i].normals.size();
        osData.write(reinterpret_cast<char*>(&normalcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].normals.data()), normalcount * sizeof(Vector3f));
        uint64_t uv1count = gameObj->meshList[i].uv1.size();
        osData.write(reinterpret_cast<char*>(&uv1count), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].uv1.data()), uv1count * sizeof(Vector2f));
        uint64_t uv2count = gameObj->meshList[i].uv2.size();
        osData.write(reinterpret_cast<char*>(&uv2count), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].uv2.data()), uv2count * sizeof(Vector2f));
        uint64_t indicesizecount = gameObj->meshList[i].indicesize.size();
        osData.write(reinterpret_cast<char*>(&indicesizecount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].indicesize.data()), indicesizecount * sizeof(uint32_t));
        uint64_t indicecount = gameObj->meshList[i].indices.size();
        osData.write(reinterpret_cast<char*>(&indicecount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].indices.data()), indicecount * sizeof(uint32_t));
        uint64_t matcount = gameObj->meshList[i].materialindex.size();
        osData.write(reinterpret_cast<char*>(&matcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].materialindex.data()), matcount * sizeof(uint32_t));
        meshtomatindex.insert(meshtomatindex.end(), gameObj->meshList[i].materialindex.begin(), gameObj->meshList[i].materialindex.end());
        meshtomatindex.push_back(-1);
        osData.close();
    }

    std::cout << meshname << std::endl;
    std::cout << materialname << std::endl;
    int meshtoindexlen = meshtomatindex.size();
    for (int i = 0; i < meshtoindexlen; i++)
    {
        std::cout << meshtomatindex[i] << std::endl;
    }

    if (meshcount > 0)
    {
        std::cout << std::endl;
        std::cout << "import fbx success" << std::endl;
    }
}

void WriteMeshFileNew(FBXGameObject* gameObj, FBXImportScene& importScene, const char* outdir)
{
    if (_access(outdir, 0) == -1)
    {
        _mkdir(outdir);
    }

    uint64_t meshcount = gameObj->meshCount;
    std::string materialname = "";

    std::vector < std::string > materialnamelist;

    for (int i = 0; i < importScene.materials.size(); i++)
    {
        if (std::find(materialnamelist.begin(), materialnamelist.end(), importScene.materials[i]) == materialnamelist.end())
        {
            materialnamelist.push_back(importScene.materials[i]);
            materialname += importScene.materials[i] + "\n";
        }
    }

    for (int i = 0; i < meshcount; i++)
    {
        gameObj->meshList[i].materialindex.clear();
        for (int j = 0; j < gameObj->meshList[i].materials.size(); j++)
        {
            auto it = std::find(materialnamelist.begin(), materialnamelist.end(), gameObj->meshList[i].materials[j]);
            if (it != materialnamelist.end())
            {
                gameObj->meshList[i].materialindex.push_back(it - materialnamelist.begin());
            }
            else
            {
                gameObj->meshList[i].materialindex.push_back(0);
            }
        }
    }

    std::string meshname = "";
    std::vector<int32_t> meshtomatindex;
    std::string directory(outdir);

    for (int i = 0; i < meshcount; i++)
    {
        std::string meshfilename(gameObj->meshList[i].name);
        meshfilename = directory + "/" + meshfilename + ".~@FFFUB";
        meshname += meshfilename + "\n";
        std::ofstream osData(meshfilename, std::ios_base::out | std::ios_base::binary);
        std::string version = "vernum";
        uint32_t versioncode = 1;
        osData.write(version.c_str(), version.length());
        osData.write(reinterpret_cast<char*>(&versioncode), sizeof(versioncode));
        uint64_t namecount = strlen(gameObj->meshList[i].name) + 1;
        osData.write(reinterpret_cast<char*>(&namecount), 8);
        osData.write(gameObj->meshList[i].name, namecount);
        uint64_t vertexcount = gameObj->meshList[i].vertices.size();
        osData.write(reinterpret_cast<char*>(&vertexcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].vertices.data()), vertexcount * sizeof(Vector3f));
        uint64_t colorcount = gameObj->meshList[i].colors.size();
        osData.write(reinterpret_cast<char*>(&colorcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].colors.data()), colorcount * sizeof(ColorRGBA32));
        uint64_t normalcount = gameObj->meshList[i].normals.size();
        osData.write(reinterpret_cast<char*>(&normalcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].normals.data()), normalcount * sizeof(Vector3f));
        uint64_t uv1count = gameObj->meshList[i].uv1.size();
        osData.write(reinterpret_cast<char*>(&uv1count), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].uv1.data()), uv1count * sizeof(Vector2f));
        uint64_t uv2count = gameObj->meshList[i].uv2.size();
        osData.write(reinterpret_cast<char*>(&uv2count), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].uv2.data()), uv2count * sizeof(Vector2f));
        uint64_t indicesizecount = gameObj->meshList[i].indicesize.size();
        osData.write(reinterpret_cast<char*>(&indicesizecount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].indicesize.data()), indicesizecount * sizeof(uint32_t));
        uint64_t indicecount = gameObj->meshList[i].indices.size();
        osData.write(reinterpret_cast<char*>(&indicecount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].indices.data()), indicecount * sizeof(uint32_t));
        uint64_t matcount = gameObj->meshList[i].materialindex.size();
        osData.write(reinterpret_cast<char*>(&matcount), 8);
        osData.write(reinterpret_cast<char*>(gameObj->meshList[i].materialindex.data()), matcount * sizeof(uint32_t));
        meshtomatindex.insert(meshtomatindex.end(), gameObj->meshList[i].materialindex.begin(), gameObj->meshList[i].materialindex.end());
        meshtomatindex.push_back(-1);
        osData.close();
    }

    std::cout << meshname << std::endl;
    std::cout << materialname << std::endl;
    int meshtoindexlen = meshtomatindex.size();
    for (int i = 0; i < meshtoindexlen; i++)
    {
        std::cout << meshtomatindex[i] << std::endl;
    }

    if (meshcount > 0)
    {
        std::cout << std::endl;
        std::cout << "import fbx success" << std::endl;
    }

}


//-----------------------------------Output test-----------------------------------
void WriteMeshOutputFiles(FBXMesh& mesh, std::vector<int>& lodMeshMaterials, char* outdir)
{
    if (_access(outdir, 0) == -1)
    {
        _mkdir(outdir);
    }

    std::string directory(outdir);

    std::string meshfilename(mesh.name);
    meshfilename = directory + "/" + meshfilename + ".txt";
    std::ofstream osData(meshfilename, std::ios_base::out | std::ios_base::trunc);
    osData << "vertices " << std::endl;
    for (int j = 0; j < mesh.vertices.size(); j++)
    {
        osData << mesh.vertices[j].x << ", " << mesh.vertices[j].y << ", " << mesh.vertices[j].z << std::endl;
    }
    osData << " normals " << std::endl;
    for (int j = 0; j < mesh.normals.size(); j++)
    {
        osData << mesh.normals[j].x << ", " << mesh.normals[j].y << ", " << mesh.normals[j].z << std::endl;
    }
    osData << " tangents " << std::endl;
    for (int j = 0; j < mesh.tangents.size(); j++)
    {
        osData << mesh.tangents[j].x << ", " << mesh.tangents[j].y << ", " << mesh.tangents[j].z << ", " << mesh.tangents[j].w << std::endl;
    }
    osData << " uv 0 " << std::endl;
    for (int j = 0; j < mesh.uv1.size(); j++)
    {
        osData << mesh.uv1[j].x << ", " << mesh.uv1[j].y << std::endl;
    }

    osData << " uv 1 " << std::endl;
    if (mesh.uv2.size() > 1)
    {
        for (int j = 0; j < mesh.uv2.size(); j++)
        {
            osData << mesh.uv2[j].x << ", " << mesh.uv2[j].y << std::endl;
        }
    }

    osData << " indices " << std::endl;
    for (int j = 0; j < mesh.indices.size(); j++)
    {
        osData << mesh.indices[j] << "," << std::flush;
    }
    osData << "\n indicesize " << std::endl;
    for (int j = 0; j < mesh.indicesize.size(); j++)
    {
        osData << mesh.indicesize[j] << "," << std::flush;
    }
    osData << "\n materials " << std::endl;
    for (int j = 0; j < mesh.materials.size(); j++)
    {
        osData << mesh.materials[j] << "," << std::flush;
    }

    osData << "\n lodMeshMaterials " << std::endl;
    for (int j = 0; j < lodMeshMaterials.size(); j++)
    {
        osData << lodMeshMaterials[j] << "," << std::flush;
    }


    osData.close();
}

void WriteFBXMeshOutputFiles(const FBXImportMesh& mesh, char* outdir)
{
    if (_access(outdir, 0) == -1)
    {
        _mkdir(outdir);
    }

    std::string directory(outdir);

    std::string meshfilename(mesh.name);
    meshfilename = directory + "/" + meshfilename + ".txt";
    std::ofstream osData(meshfilename, std::ios_base::out | std::ios_base::trunc);
    osData << "vertices " << std::endl;
    for (int j = 0; j < mesh.vertices.size(); j++)
    {
        osData << mesh.vertices[j].x << ", " << mesh.vertices[j].y << ", " << mesh.vertices[j].z << std::endl;
    }
    osData << " normals " << std::endl;
    for (int j = 0; j < mesh.normals.size(); j++)
    {
        osData << mesh.normals[j].x << ", " << mesh.normals[j].y << ", " << mesh.normals[j].z << std::endl;
    }
    osData << " tangents " << std::endl;
    for (int j = 0; j < mesh.tangents.size(); j++)
    {
        osData << mesh.tangents[j].x << ", " << mesh.tangents[j].y << ", " << mesh.tangents[j].z << ", " << mesh.tangents[j].w << std::endl;
    }
    osData << " uv 0 " << std::endl;
    for (int j = 0; j < mesh.uvs[0].size(); j++)
    {
        osData << mesh.uvs[0][j].x << ", " << mesh.uvs[0][j].y << std::endl;
    }

    osData << " uv 1 " << std::endl;
    if (mesh.uvs->size() > 1)
    {
        for (int j = 0; j < mesh.uvs[0].size(); j++)
        {
            osData << mesh.uvs[0][j].x << ", " << mesh.uvs[0][j].y << std::endl;
        }

    }

    osData << " polygons " << std::endl;
    for (int j = 0; j < mesh.polygons.size(); j++)
    {
        osData << mesh.polygons[j] << "," << std::flush;
    }
    osData << "\n polygonSizes " << std::endl;
    for (int j = 0; j < mesh.polygonSizes.size(); j++)
    {
        osData << mesh.polygonSizes[j] << "," << std::flush;
    }
    osData << "\n materials " << std::endl;
    for (int j = 0; j < mesh.materials.size(); j++)
    {
        osData << mesh.materials[j] << "," << std::flush;
    }
    osData.close();
}

void WriteSceneOutputFiles(FBXImportScene& outputScene, char* outdir)
{
    if (_access(outdir, 0) == -1)
    {
        _mkdir(outdir);
    }

    std::string directory(outdir);

    for (int i = 0; i < outputScene.meshes.size(); i++)
    {
        std::string meshfilename(outputScene.meshes[i].name);
        meshfilename = directory + "/" + meshfilename + ".txt";
        std::ofstream osData(meshfilename, std::ios_base::out | std::ios_base::trunc);
        osData << "vertices " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].vertices.size(); j++)
        {
            osData << outputScene.meshes[i].vertices[j].x << ", " << outputScene.meshes[i].vertices[j].y << ", " << outputScene.meshes[i].vertices[j].z << std::endl;
        }
        osData << " normals " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].normals.size(); j++)
        {
            osData << outputScene.meshes[i].normals[j].x << ", " << outputScene.meshes[i].normals[j].y << ", " << outputScene.meshes[i].normals[j].z << std::endl;
        }
        osData << " tangents " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].tangents.size(); j++)
        {
            osData << outputScene.meshes[i].tangents[j].x << ", " << outputScene.meshes[i].tangents[j].y << ", " << outputScene.meshes[i].tangents[j].z << ", " << outputScene.meshes[i].tangents[j].w << std::endl;
        }
        osData << " uv 0 " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].uvs[0].size(); j++)
        {
            osData << outputScene.meshes[i].uvs[0][j].x << ", " << outputScene.meshes[i].uvs[0][j].y << std::endl;
        }
        osData << " polygons " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].polygons.size(); j++)
        {
            osData << outputScene.meshes[i].polygons[j] << "," << std::flush;
        }
        osData << "\n polygonSizes " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].polygonSizes.size(); j++)
        {
            osData << outputScene.meshes[i].polygonSizes[j] << "," << std::flush;
        }
        osData << "\n materials " << std::endl;
        for (int j = 0; j < outputScene.meshes[i].materials.size(); j++)
        {
            osData << outputScene.meshes[i].materials[j] << "," << std::flush;
        }
        osData.close();
    }
}
//-----------------------------------Output test-----------------------------------