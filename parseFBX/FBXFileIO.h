#pragma once
#include <fbxsdk.h>
#include "Utility.h"
#include "FBXImporterDef.h"
#include "DataGenerate.h"
#include <fstream>

#include "proto/ProtoBuffUGCResource.pb.h"
#include <google/protobuf/text_format.h>

void WriteMeshFile(FBXGameObject* gameObj, const char* outdir);
void WriteMeshFileNew(FBXGameObject* gameObj, FBXImportScene& importScene, const char* outdir);

//Write All Mesh Data
void WriteMeshAllFile(FBXGameObject* gameObj, FBXImportScene& importScene, const char* outdir, std::string filename);
//Write Manifest File
void WriteManifest(FBXGameObject* gameObj, std::vector<std::string>& materials, FBXImportScene& importScene, const char* outdir, std::string filename);
//Write Sk File
void WriteSkeletonProtoBuf(FBXImportScene& scene, const char* outdir, const char* filename);
//Write Anim Clip File
void WriteAnimClipProtoBuf(FBXImportScene& scene, const char* outdir);
//-----------------------------------Output test-----------------------------------
void WriteMeshOutputFiles(FBXMesh& mesh, std::vector<int>& lodMeshMaterials, char* outdir);
void WriteFBXMeshOutputFiles(const FBXImportMesh& mesh, char* outdir);
void WriteSceneOutputFiles(FBXImportScene& outputScene, char* outdir);
void WriteSkeletonFileTest(FBXImportScene& scene, const char* outdir);
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
		osData.precision(8);
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
void WriteMeshAllFile(FBXGameObject* gameObj, FBXImportScene& importScene, const char* outdir,std::string filename)
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
		std::string meshfilename;
		BuildSingleMesh(gameObj->meshList[i], importScene,meshfilename, outdir);
		meshname += meshfilename + "\n";
		meshtomatindex.insert(meshtomatindex.end(), gameObj->meshList[i].materialindex.begin(), gameObj->meshList[i].materialindex.end());
		meshtomatindex.push_back(-1);
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
	WriteManifest(gameObj, materialnamelist, importScene, outdir, filename);
}


bool findNodeByName(const FBXImportNode& node, const std::string& targetName) {
	return node.name == targetName;
}

const FBXImportNode* findMeshNodeRecursive(const FBXImportNode& currentNode, const std::string& name) {
	if (findNodeByName(currentNode, name)) {
		return &currentNode;
	}
	for (const auto& child : currentNode.children) {
		const FBXImportNode* result = findMeshNodeRecursive(child, name);
		if (result != nullptr) {
			return result;
		}
	}
	return nullptr;
}

const FBXImportNode* findFirstMatchingMeshNode(const std::vector<FBXImportNode>& rootNodes, const std::string& name) {
	for (const auto& rootNode : rootNodes) {
		const FBXImportNode* result = findMeshNodeRecursive(rootNode, name);
		if (result != nullptr) {
			return result;
		}
	}
	return nullptr;
}


void WriteManifest(FBXGameObject* gameObj, std::vector<std::string>& materials, FBXImportScene& importScene, const char* outdir, std::string filename)
{
	if (_access(outdir, 0) == -1)
	{
		_mkdir(outdir);
	}
	std::string directory(outdir);
	auto outputfilename = directory + "/" + filename + ".json";
	std::ofstream osData(outputfilename, std::ios_base::out | std::ios::binary);
	osData << "{" << std::endl;

	osData << "    \"FBXName\" : \"" << filename << "\"," << std::endl;
	osData << "    \"ImportSucceeded\" : "  << "true," << std::endl;
	//Material Details
	osData << "    \"MaterialCount\" : " << materials.size() << "," << std::endl;
	osData << "    \"MaterialDetail\" : [" << std::endl;

	for (int i = 0; i < materials.size(); i++)
	{
		osData << "    {" << std::endl;
		osData << "        \"Index\" : " << i << "," << std::endl;
		osData << "        \"MaterialName\" :  \"" << materials[i] << "\"" << std::endl;
		osData << "    }";
		if (i != (materials.size() - 1))
			osData << ",";
		osData << std::endl;
	}
	osData <<"    ]," << std::endl;
	//Mesh Details
	osData << "    \"MeshCount\" : " << gameObj->meshCount << "," << std::endl;
	osData << "    \"MeshDetail\" : [" << std::endl;
	auto nodes = importScene.nodes;
	for (int i = 0; i < gameObj->meshCount; i++)
	{
		auto name = gameObj->meshList[i].name;
		osData << "    {" << std::endl;
		osData << "        \"MeshName\" : \"" << name << "\"," << std::endl;
		osData << "        \"MaterialIndex\" : ";

		auto index = gameObj->meshList[i].materialindex;
		if (index.empty()) {
			osData << "[]" << std::endl;
		}
		else {
			osData << "[";
			for (size_t j = 0; j < index.size(); ++j) {
				osData << index[j];
				if (j < index.size() - 1) {
					osData << ",";
				}
			}
			osData << "]";
		}
		
		const FBXImportNode* result = findFirstMatchingMeshNode(nodes, name);
		if (result != nullptr) {

			osData << "," << std::endl;
			osData << "        \"Transform\" : {" << std::endl;
			osData << "          \"Position\" : ["
				<< result->position.x * importScene.fileScaleFactor << ", "
				<< result->position.y * importScene.fileScaleFactor << ", "
				<< result->position.z * importScene.fileScaleFactor << "]," << std::endl;
			osData << "          \"Rotation\" : ["
				<< result->rotation.x << ", "
				<< result->rotation.y << ", "
				<< result->rotation.z << ", "
				<< result->rotation.w <<
				"]," << std::endl;
			osData << "          \"Scale\" : ["
				<< result->scale.x << ", "
				<< result->scale.y << ", "
				<< result->scale.z << "]" << std::endl;
			osData << "        }" << std::endl;
		}
		else
			osData << std::endl;
		osData << "    }";


		if (i != (gameObj->meshCount - 1))
			osData << ",";
		osData << std::endl;
	}
	osData << "    ]" << std::endl;
	osData << "}" << std::endl;
	osData.close();

	//Rename To Support Chinese		
	std::wstring outputfilenameW = ConvertUTF8ToWide(outputfilename);
	RenameFileToWide(outputfilename, outputfilenameW);
}
void WriteManifestErrorInput(const char* outdir, std::string filename,std::vector<std::string> errormesh)
{
	if (_access(outdir, 0) == -1)
	{
		_mkdir(outdir);
	}
	std::string directory(outdir);
	auto outputfilename = directory + "/" + filename + ".json";
	std::ofstream osData(outputfilename, std::ios_base::out);
	osData << "{" << std::endl;

	osData << "    \"FBXName\" : \"" << filename << "\"," << std::endl;
	osData << "    \"ImportSucceeded\" : " << "false," << std::endl;
	//Material Details
	osData << "    \"MaterialCount\" : " << "0," << std::endl;
	osData << "    \"MaterialDetail\" : []" << std::endl;

	//Mesh Details
	osData << "    \"MeshCount\" : " <<  "0," << std::endl;
	osData << "    \"MeshDetail\" : []" << std::endl;

	//Error Mesh
	osData << "    \"ErrorMeshCount\" : " << errormesh.size() << "," << std::endl;
	osData << "    \"ErrorMeshDetail\" : [" << std::endl;

	for (int i = 0; i < errormesh.size(); i++)
	{
		osData << "        \"" << errormesh[i] << "\"" ;
		
		if (i != (errormesh.size() - 1))
			osData << ",";
		osData << std::endl;
	}
	osData << "    ]" << std::endl;
	osData << "}" << std::endl;


	osData << "}" << std::endl;
	osData.close();


	//Rename To Support Chinese		
	std::wstring outputfilenameW = ConvertUTF8ToWide(outputfilename);
	RenameFileToWide(outputfilename, outputfilenameW);

}
void WriteSkeletonProtoBuf(FBXImportScene& scene, const char* outdir, const char* filename)
{
	if (_access(outdir, 0) == -1)
	{
		_mkdir(outdir);
	}
	message::UGCResSkeletonData* sk = new message::UGCResSkeletonData();

	message::UGCResBoneNodeData* root = new message::UGCResBoneNodeData();
	message::UGCResBoneNodeCapsuleData* root_capsule = new message::UGCResBoneNodeCapsuleData();
	root->set_bonename("root");
	//root->set_allocated_capsuleinfo(root_capsule);
	sk->set_allocated_rootbone(root);
	message::ProtoBuffVector3* pos = new message::ProtoBuffVector3();
	message::ProtoBuffVector3* scale = new message::ProtoBuffVector3();
	message::ProtoBuffQuaternion* quat = new message::ProtoBuffQuaternion();
	auto scaleFactor = scene.fileScaleFactor;
	pos->set_x(0.0f); pos->set_y(0.0f); pos->set_z(0.0f);
	scale->set_x(1.0f); scale->set_y(1.0f); scale->set_z(1.0f);
	quat->set_x(0.0f); quat->set_y(0.0f); quat->set_z(0.0f); quat->set_w(1.0f);
	root->set_allocated_localposition(pos);
	root->set_allocated_localscale(scale);
	root->set_allocated_localrotation(quat);

	auto allnodes = scene.nodes;
	if (allnodes.size() > 1)
	{
		// multi root - add an "root" node to ensure a single root tree
		for (auto i = 0; i < allnodes.size(); i++)
		{
			BuildBoneNodeData(scene, allnodes[i], root);
		}
	}
	else if (allnodes.size() == 1 )
	{
		// single root- rename the single name as root
		auto rootnode = allnodes[0];
		pos->set_x(rootnode.position.x * scaleFactor); pos->set_y(rootnode.position.y * scaleFactor); pos->set_z(rootnode.position.z * scaleFactor);
		scale->set_x(rootnode.scale.x); scale->set_y(rootnode.scale.y); scale->set_z(rootnode.scale.z);
		quat->set_x(rootnode.rotation.x); quat->set_y(rootnode.rotation.y); quat->set_z(rootnode.rotation.z); quat->set_w(rootnode.rotation.w);

		auto childnodes = rootnode.children;
		for (auto i = 0; i < childnodes.size(); i++)
		{
			BuildBoneNodeData(scene, childnodes[i], root);
		}
	}
	//else
	//{
	//	// no root, create dummy root
	//}


	std::string directory(outdir);

	auto OutputFileName = directory + "/" + filename + ".sk";
	//Serialize
	std::ofstream output(OutputFileName, std::ios::out | std::ios::trunc | std::ios::binary);
	bool flag = sk->SerializePartialToOstream(&output);
	if (!flag)
	{
		std::cout << "Error when Serializing" << std::endl;
	}
	output.close();
	//Rename To Support Chinese		
	std::wstring OutputFileNameW = ConvertUTF8ToWide(OutputFileName);
	RenameFileToWide(OutputFileName, OutputFileNameW);


#if DebugMeshInfoOutput
	std::string text_string;
	google::protobuf::TextFormat::PrintToString(*sk, &text_string);
	std::ofstream output_file(directory + "/" + filename + "_sk.txt");
	if (!output_file.is_open()) {
		std::cerr << "Failed to open file for writing." << std::endl;
	}
	output_file << text_string;
	output_file.close();
#endif
	google::protobuf::ShutdownProtobufLibrary();
}
void WriteAnimClipProtoBuf(FBXImportScene& scene, const char* outdir)
{
	if (_access(outdir, 0) == -1)
	{
		_mkdir(outdir);
	}
	auto& anim = scene.animationClips;

	for (auto i = 0; i < anim.size(); i++)
	{
		if (anim[i].HasAnimations())
		{
			//BuildSingleAnimProtoFile(scene, anim[i], outdir);
			BuildSingleAnimBinaryFile(scene, anim[i], outdir);
#if DebugMeshInfoOutput
			WriteNodeAnimationsToText(scene, anim[i], outdir);
#endif
		}

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

void WriteAnimClipFileTest(FBXImportScene& scene, const char* outdir)
{
	auto anim = scene.animationClips;
	for (auto i = 0; i < anim.size(); i++)
	{
		auto clip = anim[i];
		auto floatcurves = clip.floatAnimations;
		auto nodecurves = clip.nodeAnimations;

		std::string directory(outdir);
		std::string filename(clip.name);
		filename = directory + "/" + filename + ".txt";
		std::ofstream osData(filename, std::ios_base::out | std::ios_base::trunc);
		osData << "Curve Count: " << nodecurves.size() << std::endl;
		for (auto it = nodecurves.begin(); it != nodecurves.end(); it++)
		{
			osData << "    propName: " << it->node->name << std::endl;
			auto rotation_curve = it->rotation;
			auto translation_curve = it->translation;
			auto scale_curve = it->scale;
			auto rot_count = rotation_curve[0].m_Curve.size();
			if (rot_count > 0)
			{
				osData << "      Rotation Curve Count: " << rot_count << std::endl;
				for (auto k = 0; k < rot_count; k++)
				{
					auto keyframes_x = rotation_curve[0].m_Curve[k];
					auto keyframes_y = rotation_curve[1].m_Curve[k];
					auto keyframes_z = rotation_curve[2].m_Curve[k];
					auto keyframes_w = rotation_curve[3].m_Curve[k];

					osData << "        Rotation Quaternion Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << " ," << keyframes_w.value << std::endl;

				}
			}

			auto trans_count = translation_curve[0].m_Curve.size();
			if (trans_count > 0)
			{
				osData << "      Translation Curve Count: " << trans_count << std::endl;
				for (auto k = 0; k < trans_count; k++)
				{
					auto keyframes_x = translation_curve[0].m_Curve[k];
					auto keyframes_y = translation_curve[1].m_Curve[k];
					auto keyframes_z = translation_curve[2].m_Curve[k];


					osData << "        Translation Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << std::endl;

				}
			}

			auto scale_count = scale_curve[0].m_Curve.size();
			if (scale_count > 0)
			{
				osData << "      Scale Curve Count: " << scale_count << std::endl;
				for (auto k = 0; k < scale_count; k++)
				{
					auto keyframes_x = scale_curve[0].m_Curve[k];
					auto keyframes_y = scale_curve[1].m_Curve[k];
					auto keyframes_z = scale_curve[2].m_Curve[k];
					osData << "        Scale Quaternion Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << std::endl;

				}
			}

		}
	}
}
void PrintBone(const message::UGCResBoneNodeData& node,std::string parent)
{
	auto name = node.bonename();
	auto pos = node.localposition();
	auto scale = node.localscale();
	auto quat = node.localrotation();
	auto childsize = node.childbones_size();
	parent = parent + name + "/";
	std::cout << " Bone Name: " << parent << " Has " << childsize << " Children" << std::endl;
	std::cout << "      Pos:" << pos.x() << " , " << pos.y() << " , " << pos.z() << std::endl;
	std::cout << "      Scale:" << scale.x() << " , " << scale.y() << " , " << scale.z() << std::endl;
	std::cout << "      Quat:" << quat.x() << " , " << quat.y() << " , " << quat.z() << " , " << quat.w() << std::endl;

	for (auto i = 0; i < childsize; i++)
	{
		const message::UGCResBoneNodeData& next = node.childbones(i);
		PrintBone(next, parent);
	}
}
void PrintAnimFile(FBXImportScene& importScene, const char* outdir)
{
	if (_access(outdir, 0) == -1)
	{
		_mkdir(outdir);
	}

	auto anims = importScene.animationClips;
	for (auto i = 0; i < anims.size(); i++)
	{
		auto clip = anims[i];
		auto floatcurves = clip.floatAnimations;
		auto nodecurves = clip.nodeAnimations;

		std::string directory(outdir);
		std::string filename(clip.name);
		filename = directory + "/" + filename + ".anim";

		std::ofstream osData(filename, std::ios_base::out | std::ios_base::binary);
		osData.precision(8);

		uint64_t CurveCount = nodecurves.size();
		osData.write(reinterpret_cast<char*>(&CurveCount), 8);

		for (auto it = nodecurves.begin(); it != nodecurves.end(); it++)
		{
			osData << "    propName: " << it->node->name << std::endl;
			auto rotation_curve = it->rotation;
			auto translation_curve = it->translation;
			auto scale_curve = it->scale;
			auto rot_count = rotation_curve[0].m_Curve.size();
			if (rot_count > 0)
			{
				osData << "      Rotation Curve Count: " << rot_count << std::endl;
				for (auto k = 0; k < rot_count; k++)
				{
					auto keyframes_x = rotation_curve[0].m_Curve[k];
					auto keyframes_y = rotation_curve[1].m_Curve[k];
					auto keyframes_z = rotation_curve[2].m_Curve[k];
					auto keyframes_w = rotation_curve[3].m_Curve[k];

					osData << "        Rotation Quaternion Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << " ," << keyframes_w.value << std::endl;

				}
			}

			auto trans_count = translation_curve[0].m_Curve.size();
			if (trans_count > 0)
			{
				osData << "      Translation Curve Count: " << trans_count << std::endl;
				for (auto k = 0; k < trans_count; k++)
				{
					auto keyframes_x = translation_curve[0].m_Curve[k];
					auto keyframes_y = translation_curve[1].m_Curve[k];
					auto keyframes_z = translation_curve[2].m_Curve[k];


					osData << "        Translation Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << std::endl;

				}
			}

			auto scale_count = scale_curve[0].m_Curve.size();
			if (scale_count > 0)
			{
				osData << "      Scale Curve Count: " << scale_count << std::endl;
				for (auto k = 0; k < scale_count; k++)
				{
					auto keyframes_x = scale_curve[0].m_Curve[k];
					auto keyframes_y = scale_curve[1].m_Curve[k];
					auto keyframes_z = scale_curve[2].m_Curve[k];
					osData << "        Scale Quaternion Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << std::endl;

				}
			}

		}
		osData << "Curve Count: " << nodecurves.size() << std::endl;
		for (auto it = nodecurves.begin(); it != nodecurves.end(); it++)
		{
			osData << "    propName: " << it->node->name << std::endl;
			auto rotation_curve = it->rotation;
			auto translation_curve = it->translation;
			auto scale_curve = it->scale;
			auto rot_count = rotation_curve[0].m_Curve.size();
			if (rot_count > 0)
			{
				osData << "      Rotation Curve Count: " << rot_count << std::endl;
				for (auto k = 0; k < rot_count; k++)
				{
					auto keyframes_x = rotation_curve[0].m_Curve[k];
					auto keyframes_y = rotation_curve[1].m_Curve[k];
					auto keyframes_z = rotation_curve[2].m_Curve[k];
					auto keyframes_w = rotation_curve[3].m_Curve[k];

					osData << "        Rotation Quaternion Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << " ," << keyframes_w.value << std::endl;

				}
			}

			auto trans_count = translation_curve[0].m_Curve.size();
			if (trans_count > 0)
			{
				osData << "      Translation Curve Count: " << trans_count << std::endl;
				for (auto k = 0; k < trans_count; k++)
				{
					auto keyframes_x = translation_curve[0].m_Curve[k];
					auto keyframes_y = translation_curve[1].m_Curve[k];
					auto keyframes_z = translation_curve[2].m_Curve[k];


					osData << "        Translation Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << std::endl;

				}
			}

			auto scale_count = scale_curve[0].m_Curve.size();
			if (scale_count > 0)
			{
				osData << "      Scale Curve Count: " << scale_count << std::endl;
				for (auto k = 0; k < scale_count; k++)
				{
					auto keyframes_x = scale_curve[0].m_Curve[k];
					auto keyframes_y = scale_curve[1].m_Curve[k];
					auto keyframes_z = scale_curve[2].m_Curve[k];
					osData << "        Scale Quaternion Curve [" << k << "]: Time: " << keyframes_x.time << " , Value: "
						<< keyframes_x.value << " ," << keyframes_y.value << " ," << keyframes_z.value << std::endl;

				}
			}

		}
	}
}

//-----------------------------------Output test-----------------------------------