#pragma once
#include "Utility.h"
#include "AnimationKeyFrameReducer.h"
#include "FBXImporterDef.h"
#include "proto/ProtoBuffUGCResource.pb.h"
#include <locale>
#include <codecvt>

#define DebugMeshInfoOutput 0
static std::map<std::string, std::string> gNodePath2Name;
static std::map<std::string, std::vector<std::string>> gNodeName2BoneName;
static std::map<std::string, std::vector<Matrix4x4f>> gNodeName2BoneBindePose;
static std::map<std::string, std::string> gBlendShapeMesh2Bone;

#pragma region FileProcess 
std::string CodeTUTF8(const char* str, int t)
{
	std::string result;
	WCHAR* strSrc;
	LPSTR szRes;

	int i = MultiByteToWideChar(t, 0, str, -1, NULL, 0);
	strSrc = new WCHAR[i + 1];
	MultiByteToWideChar(t, 0, str, -1, strSrc, i);

	i = WideCharToMultiByte(CP_UTF8, 0, strSrc, -1, NULL, 0, NULL, NULL);
	szRes = new CHAR[i + 1];
	WideCharToMultiByte(CP_UTF8, 0, strSrc, -1, szRes, i, NULL, NULL);

	result = szRes;
	delete[]strSrc;
	delete[]szRes;

	return result; 
}
std::wstring ConvertToWide(const char* utf8Str) {
	if (utf8Str == nullptr) {
		return L"";
	}

	int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, utf8Str, -1, NULL, 0);
	if (sizeNeeded <= 0) {
		//std::wcerr << L"MultiByteToWideChar 转换失败！" << std::endl;
		return L"";
	}

	std::wstring wideStr(sizeNeeded - 1, 0);
	MultiByteToWideChar(CP_UTF8, 0, utf8Str, -1, &wideStr[0], sizeNeeded);

	return wideStr;
}
std::wstring ConvertUTF8ToWide(const std::string& utf8Str) {
	return ConvertToWide(utf8Str.c_str());
}
std::wstring MultiByteToWide(const std::string& multiByteStr, UINT codePage) {
	int sizeNeeded = MultiByteToWideChar(codePage, 0, multiByteStr.c_str(), -1, NULL, 0);
	if (sizeNeeded <= 0) {
		//throw std::runtime_error("多字节字符串转宽字符失败");
	}
	std::wstring wideStr(sizeNeeded - 1, 0); 
	MultiByteToWideChar(codePage, 0, multiByteStr.c_str(), -1, &wideStr[0], sizeNeeded);
	return wideStr;
}
void EnsureDirectoryExists(const std::wstring& directoryPath) {
	DWORD fileAttributes = GetFileAttributesW(directoryPath.c_str());

	if (fileAttributes != INVALID_FILE_ATTRIBUTES) {
		if (fileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			return;
		}
		else {
			//std::wcerr << L"路径存在，但不是目录: " << directoryPath << std::endl;
			return;
		}
	}

	std::size_t pos = directoryPath.find_last_of(L"\\/");
	if (pos != std::wstring::npos) {
		EnsureDirectoryExists(directoryPath.substr(0, pos));
	}

	if (CreateDirectoryW(directoryPath.c_str(), NULL)) {
		//std::wcout << L"成功创建目录: " << directoryPath << std::endl;
	}
	else {
		DWORD error = GetLastError();
		if (error == ERROR_ALREADY_EXISTS) {
			// 忽略目录已存在的错误
			return;
		}
		else {
			//std::wcerr << L"无法创建目录: " << directoryPath << L"，错误代码: " << error << std::endl;
		}
	}
}
void RenameFileToWide(const std::string& originalName, const std::wstring& newName) {
	try {
		std::wstring wideOriginalName = MultiByteToWide(originalName, CP_ACP);

		std::size_t pos = newName.find_last_of(L"\\/");
		if (pos != std::wstring::npos) {
			std::wstring targetDirectory = newName.substr(0, pos);
			EnsureDirectoryExists(targetDirectory);
		}

		if (MoveFileExW(wideOriginalName.c_str(), newName.c_str(), MOVEFILE_REPLACE_EXISTING)) {
			//std::wcout << L"文件重命名成功！从 " << wideOriginalName << L" 到 " << newName << std::endl;
		}
		else {
			//DWORD error = GetLastError();
			//std::wcerr << L"文件重命名失败，错误代码: " << error << std::endl;
		}
	}
	catch (const std::exception& e) {
		//std::cerr << "错误: " << e.what() << std::endl;
	}
}
bool IsDirectoryEmpty(const std::wstring& directoryPath) {
	WIN32_FIND_DATAW findFileData;
	std::wstring searchPath = directoryPath + L"\\*";

	HANDLE hFind = FindFirstFileW(searchPath.c_str(), &findFileData);
	if (hFind == INVALID_HANDLE_VALUE) {
		//std::wcerr << L"无法打开目录: " << directoryPath << L"，错误代码: " << GetLastError() << std::endl;
		return false;
	}

	bool isEmpty = true;
	do {
		if (wcscmp(findFileData.cFileName, L".") != 0 && wcscmp(findFileData.cFileName, L"..") != 0) {
			isEmpty = false;
			break;
		}
	} while (FindNextFileW(hFind, &findFileData));

	FindClose(hFind);
	return isEmpty;
}
bool IsSubPath(const std::wstring& sourcePath, const std::wstring& targetPath) {
	if (targetPath.find(sourcePath) == 0 && (targetPath[sourcePath.length()] == L'\\' || targetPath[sourcePath.length()] == L'/')) {
		return true;
	}
	return false;
}
void DeleteEmptyFolders(const std::string& sourcePath, const std::wstring& targetPath) {
	
	std::wstring currentPath = MultiByteToWide(sourcePath, CP_ACP);
	while (!currentPath.empty()) {
		if (IsSubPath(currentPath, targetPath)) {
			//std::cout << "Finish Del at : " << currentPath.c_str() << std::endl;
			break;
		}

		// 检查当前路径是否为空
		if (IsDirectoryEmpty(currentPath)) {
			if (RemoveDirectoryW(currentPath.c_str())) {
				//std::cout << "Del: " << currentPath.c_str() << std::endl;
			}
			else {
				//std::wcerr << L"无法删除文件夹: " << currentPath << L"，错误代码: " << GetLastError() << std::endl;
				break;
			}
		}
		else {
			//std::wcout << L"文件夹不为空，停止删除: " << currentPath << std::endl;
			break;
		}

		std::size_t pos = currentPath.find_last_of(L"\\/");
		if (pos != std::wstring::npos) {
			currentPath = currentPath.substr(0, pos);
		}
		else {
			currentPath.clear();
		}
	}
}
#pragma endregion

//Build Ref Maps
void BuildBoneNameMap(FBXImportNode& node, std::map<std::string, std::string>& path2name, std::string parentpath = "");
void BuildAllBoneNameMap(FBXImportScene& scene);
void BuildMeshBoneRefMap(FBXImportScene& scene);

//Build Mesh
struct MeshHead
{
	byte MagicNumber1;
	byte MagicNumber2;
	byte MagicNumber3;
	byte MagicNumber4;

	int Version;
	int MeshType;
	int MeshDataStartPos;
	int MeshDataSize;
	int MeshDataExtPos;
	int MeshDataExtSize;
	MeshHead()
	{
		MagicNumber1 = 0;
		MagicNumber2 = 0;
		MagicNumber3 = 0;
		MagicNumber4 = 0;

		Version = 0;
		MeshType = 0;
		MeshDataStartPos = 0;
		MeshDataSize = 0;
		MeshDataExtPos = 0;
		MeshDataExtSize = 0;
	}
};
struct MeshBody
{
	int NamePos;
	int NameHeadLength;//8
	uint64_t NameLength;
	int VerticesPos;
	int VerticesHeadLength;//8
	uint64_t VerticesLength;
	int ColorPos;
	int ColorHeadLength;//8
	uint64_t ColorLength;
	int NormalPos;
	int NormalHeadLength;//8
	uint64_t NormalLength;
	int UV1Pos;
	int UV1HeadLength;//8
	uint64_t UV1Length;
	int UV2Pos;
	int UV2HeadLength;//8
	uint64_t UV2Length;
	int IndexSizePos;
	int IndexSizeHeadLength;//8
	uint64_t IndexSizeLength;
	int IndexPos;
	int IndexHeadLength;//8
	uint64_t IndexLength;
	int MatPos;
	int MatHeadLength;//8
	uint64_t MatLength;
	int BindPosesPos;
	int BindPosesHeadLength;//8
	uint64_t BindPosesLength;
	int BoneWeightPos;
	int BoneWeightHeadLength;//8
	uint64_t BoneWeightLength;

	MeshBody() :NamePos(0),
		NameHeadLength(0),
		NameLength(0),
		VerticesPos(0),
		VerticesHeadLength(0),
		VerticesLength(0),
		ColorPos(0),
		ColorHeadLength(0),
		ColorLength(0),
		NormalPos(0),
		NormalHeadLength(0),
		NormalLength(0),
		UV1Pos(0),
		UV1HeadLength(0),
		UV1Length(0),
		UV2Pos(0),
		UV2HeadLength(0),
		UV2Length(0),
		IndexSizePos(0),
		IndexSizeHeadLength(0),
		IndexSizeLength(0),
		IndexPos(0),
		IndexHeadLength(0),
		IndexLength(0),
		MatPos(0),
		MatHeadLength(0),
		MatLength(0),
		BindPosesPos(0),
		BindPosesHeadLength(0),
		BindPosesLength(0),
		BoneWeightPos(0),
		BoneWeightHeadLength(0),
		BoneWeightLength(0)
	{}



};
void BuildMeshHead(FBXMesh& meshData, message::UGCResSkinnedMeshExtData& extData, MeshBody& body, std::ofstream& osData);
message::UGCResSkinnedMeshExtData BuildMeshExtData(FBXMesh& meshData);
void BuildMeshBody(FBXMesh& meshData, MeshBody& bodyinfo, std::ofstream& osData);
//Build Anim
void BuildSingleAnimProtoFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir);
//Build Bones
void BuildBoneNodeData(FBXImportScene& scene, FBXImportNode& node, message::UGCResBoneNodeData* parent);

//Debug Only
void ParseSingleMesh(std::string meshfile);
void DebugMeshInfo(FBXMesh& meshData, MeshBody& bodyinfo);
void ParseAnimProto(message::UGCResAnimClipData& msg);
void ParseSingleAnim(std::string animfile);

void BuildBoneNameMap(FBXImportNode& node, std::map<std::string, std::string>& path2name, std::string parentpath)
{
	std::string NodeName = node.name;
	std::string NodePath = NodeName;
	if(parentpath != "")
		NodePath = parentpath + "/" + NodeName;
	//std::cout << NodeName << " and path :: " << NodePath << std::endl;
	path2name.insert(std::pair<std::string, std::string>(NodePath, NodeName));
	for (auto i = 0; i < node.children.size(); i++)
	{
		BuildBoneNameMap(node.children[i], path2name, NodePath);
	}
}
void BuildAllBoneNameMap(FBXImportScene& scene)
{
	gNodePath2Name.clear();
	auto allnodes = scene.nodes;
	// when it has extra roots then put an "root" to make sure a single root object
	// when it has only one root then rename the root as "root"
	// when it has no node then create at least one dummy "root"
	if (allnodes.size() > 1)
	{
		for (auto i = 0; i < allnodes.size(); i++)
		{
			BuildBoneNameMap(allnodes[i], gNodePath2Name);
		}
	}
	else if (allnodes.size() == 1)
	{
		auto children = allnodes[0].children;
		for (auto i = 0; i < children.size(); i++)
		{
			BuildBoneNameMap(children[i], gNodePath2Name);
		}
	}
}
void BuildMeshBoneRefMap(FBXImportScene& scene)
{
	gNodeName2BoneName.clear();
	gNodeName2BoneBindePose.clear();
	auto meshes = scene.meshes;
	auto scale = scene.fileScaleFactor;
	for (auto i = 0; i < meshes.size(); i++)
	{
		auto singleMeshBones = meshes[i].bones;
		std::vector<std::string> boneNames;
		std::vector<Matrix4x4f> boneBindPoses;

		boneNames.clear();
		boneBindPoses.clear();
		for (auto j = 0; j < singleMeshBones.size(); j++)
		{
			std::string name = singleMeshBones[j].node->name;
			Matrix4x4f& bindpos = singleMeshBones[j].bindpose;
			//Apply FileScale to Transfrom
			Matrix4x4f newPose = bindpos;
			newPose[12] *= scale;
			newPose[13] *= scale;
			newPose[14] *= scale;
			boneNames.push_back(name);
			boneBindPoses.push_back(newPose);
		}
		if (boneNames.size() > 0)
			gNodeName2BoneName.insert(std::pair<std::string, std::vector<std::string>>(meshes[i].name, boneNames));

		if (boneBindPoses.size() > 0)
			gNodeName2BoneBindePose.insert(std::pair<std::string, std::vector<Matrix4x4f>>(meshes[i].name, boneBindPoses));
	}
}
void BuildBlendShapeBoneMap(FBXImportScene& scene)
{
	auto allMesh = scene.meshes;
	for (auto it = allMesh.begin(); it != allMesh.end(); it++)
	{
		if (it->shapes.size() > 0 && it->vertices.size() > 0)
		{
			auto blendShapeMeshName = it->name;
			for (const auto& pair : gNodePath2Name) {
				if (pair.second == blendShapeMeshName) {
					auto bonename = pair.first;
					size_t pos = bonename.rfind('/');
					if (pos != std::string::npos) 
					{
						bonename = bonename.substr(pos + 1);
					}
					gBlendShapeMesh2Bone.insert(std::pair<std::string, std::string>(pair.second, bonename));
					break; 
				}
			}
		}
	}


}
void PrebuildFBXMeshForBlendShape(FBXMesh& meshData)
{
	if (meshData.vertices.size() > 0)
	{
		//add bone weight
		auto allBoneWeights = meshData.boneWeights;
		BoneWeights4 boneWeights = { {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0} };
		allBoneWeights.resize(meshData.vertices.size(), boneWeights);
		//add bindpose
		Matrix4x4f ident;
		ident.SetIdentity();
		meshData.bindPoses.push_back(ident);
	}
}

void BuildMeshTxt(FBXMesh& meshData, FBXImportScene& importScene, const char* outdir)
{
	std::string meshfilename(meshData.name);
	std::string directory(outdir);
	meshfilename = directory + "/" + meshfilename + ".txt";
	std::ofstream osData(meshfilename);
	osData.precision(8);
	osData << "MeshName: " << meshData.name << std::endl;

	osData << "*********************************************************" << std::endl;
	auto& bw = meshData.boneWeights;
	osData << "Mesh boneweight Count: " << bw.size() << std::endl;
	for (auto i = 0; i < bw.size(); i++)
	{
		auto currentbw = bw[i];
		osData << "BoneWeight No. " << i << " weight [ " << currentbw.weight[0] << ", " << currentbw.weight[1] << ", " << currentbw.weight[2] << ", " << currentbw.weight[3] << "]" << std::endl;
		osData << "BoneWeight No. " << i << " boneIndex [ " << currentbw.boneIndex[0] << ", " << currentbw.boneIndex[1] << ", " << currentbw.boneIndex[2] << ", " << currentbw.boneIndex[3] << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "Mesh bindPoses Count: " << meshData.bindPoses.size() << std::endl;
	for (auto i = 0; i < meshData.bindPoses.size(); i++)
	{
		auto mat = meshData.bindPoses[i];
		osData << "*********************************************************" << std::endl;
		osData << "[ " << mat[0] << " , " << mat[1] << " , " << mat[2]<< " , " << mat[3] <<" ]" << std::endl;
		osData << "[ " << mat[4] << " , " << mat[5] << " , " << mat[6] << " , " << mat[7] << " ]" << std::endl;
		osData << "[ " << mat[8] << " , " << mat[9] << " , " << mat[10] << " , " << mat[11] << " ]" << std::endl;
		osData << "[ " << mat[12] << " , " << mat[13] << " , " << mat[14] << " , " << mat[15] << " ]" << std::endl;
		osData << "*********************************************************" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "Vert Count : " << meshData.vertices.size() << std::endl;
	for (auto i = 0; i < meshData.vertices.size(); i++)
	{
		osData << "Vertex No. " << i << "[ " << meshData.vertices[i].x << ", " << meshData.vertices[i].y << ", " << meshData.vertices[i].z << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "Norl Count : " << meshData.normals.size() << std::endl;
	for (auto i = 0; i < meshData.normals.size(); i++)
	{
		osData << "Norl No. " << i << "[ " << meshData.normals[i].x << ", " << meshData.normals[i].y << ", " << meshData.normals[i].z << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "uv Count : " << meshData.uv1.size() << std::endl;
	for (auto i = 0; i < meshData.uv1.size(); i++)
	{
		osData << "UV1 No. " << i << "[ " << meshData.uv1[i].x << ", " << meshData.uv1[i].y << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "index Count : " << meshData.indices.size() << std::endl;
	for (auto i = 0; i < meshData.indices.size(); i++)
	{
		osData << "Index No. " << i << "[ " << meshData.indices[i] << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData.close();
	//osData << "Mesh boneWeights Count: " << meshData.bindPoses.size() << std::endl;

	//osData.write(reinterpret_cast<char*>(&bodyinfo.BindPosesLength), 8);
	//osData.write(reinterpret_cast<char*>(meshData.bindPoses.data()), bodyinfo.BindPosesLength * sizeof(Matrix4x4f));

	////BoneWeights4
	//osData.write(reinterpret_cast<char*>(&bodyinfo.BoneWeightLength), 8);
	//osData.write(reinterpret_cast<char*>(meshData.boneWeights.data()), bodyinfo.BoneWeightLength * sizeof(BoneWeights4));


	std::wstring meshfilenameW = ConvertUTF8ToWide(meshfilename);
	RenameFileToWide(meshfilename, meshfilenameW);

}


void BuildSingleMesh(FBXMesh& meshData, FBXImportScene& importScene, std::string& filename, const char* outdir)
{
	if (_access(outdir, 0) == -1)
	{
		_mkdir(outdir);
	}
	bool isSkinnedMesh = false;
	message::UGCResSkinnedMeshExtData extData = BuildMeshExtData(meshData);
	if (extData.bonenames_size() > 0)
	{
		isSkinnedMesh = true;
	}
	std::string meshfilename(meshData.name);
	std::string directory(outdir);
	if (isSkinnedMesh)
		meshfilename = directory + "/" + meshfilename + ".~@FFSKIN";
	else
		meshfilename = directory + "/" + meshfilename + ".~@FFFUB";
	filename = meshfilename;

	std::ofstream osData(meshfilename, std::ios_base::out | std::ios_base::binary);

	if (!osData.is_open()) {
		std::cout << "Error On Create File at " << meshfilename << std::endl;
		return;
	}
	osData.precision(8);
	MeshBody body;
	BuildMeshHead(meshData, extData, body, osData);
	BuildMeshBody(meshData, body, osData);
	if (isSkinnedMesh)
		extData.SerializePartialToOstream(&osData);
	osData.close();
	//Rename To Support Chinese		
	std::wstring meshfilenameW = ConvertUTF8ToWide(meshfilename);
	RenameFileToWide(meshfilename, meshfilenameW);
#if DebugMeshInfoOutput
	BuildMeshTxt(meshData, importScene, outdir);
	ParseSingleMesh(meshfilename);
#endif
}
void BuildMeshHead(FBXMesh& meshData, message::UGCResSkinnedMeshExtData& extData, MeshBody& body, std::ofstream& osData)
{
	MeshHead head;

	body.NamePos = sizeof(MeshHead);
	body.NameHeadLength = 8;
	body.NameLength = strlen(meshData.name) + 1;

	body.VerticesPos += body.NamePos + body.NameHeadLength + body.NameLength;
	body.VerticesHeadLength = 8;
	body.VerticesLength = meshData.vertices.size();

	body.ColorPos += body.VerticesPos + body.VerticesHeadLength + body.VerticesLength * sizeof(Vector3f);
	body.ColorHeadLength = 8;
	body.ColorLength = meshData.colors.size();

	body.NormalPos += body.ColorPos + body.ColorHeadLength + body.ColorLength * sizeof(ColorRGBA32);
	body.NormalHeadLength = 8;
	body.NormalLength = meshData.normals.size();

	body.UV1Pos += body.NormalPos + body.NormalHeadLength + body.NormalLength * sizeof(Vector3f);
	body.UV1HeadLength = 8;
	body.UV1Length = meshData.uv1.size();

	body.UV2Pos += body.UV1Pos + body.UV1HeadLength + body.UV1Length * sizeof(Vector2f);
	body.UV2HeadLength = 8;
	body.UV2Length = meshData.uv2.size();

	body.IndexSizePos += body.UV2Pos + body.UV2HeadLength + body.UV2Length * sizeof(Vector2f);
	body.IndexSizeHeadLength = 8;
	body.IndexSizeLength = meshData.indicesize.size();

	body.IndexPos += body.IndexSizePos + body.IndexSizeHeadLength + body.IndexSizeLength * sizeof(uint32_t);
	body.IndexHeadLength = 8;
	body.IndexLength = meshData.indices.size();

	body.MatPos += body.IndexPos + body.IndexHeadLength + body.IndexLength * sizeof(uint32_t);
	body.MatHeadLength = 8;
	body.MatLength = meshData.materialindex.size();

	body.BindPosesPos += body.MatPos + body.MatHeadLength + body.MatLength * sizeof(uint32_t);
	body.BindPosesHeadLength = 8;
	body.BindPosesLength = meshData.bindPoses.size();

	body.BoneWeightPos += body.BindPosesPos + body.BindPosesHeadLength + body.BindPosesLength * sizeof(Matrix4x4f);
	body.BoneWeightHeadLength = 8;
	body.BoneWeightLength = meshData.boneWeights.size();

	head.MeshDataStartPos = body.NamePos;
	head.MeshDataExtPos = body.BoneWeightPos + body.BoneWeightHeadLength + body.BoneWeightLength * sizeof(BoneWeights4);
	head.MeshDataSize = head.MeshDataExtPos - head.MeshDataStartPos;

	if (extData.bonenames_size() > 0)
	{
		head.MeshDataExtSize = extData.ByteSizeLong();
		head.MeshType = 1;
	}


#if DebugMeshInfoOutput
	std::cout << "////////////////////////////////////////////////////////////////////////" << std::endl;
	std::cout << "Mesh Name:" << meshData.name << std::endl;
	std::cout << "MeshDataStartPos:" << head.MeshDataStartPos << std::endl;
	std::cout << "MeshDataExtPos:" << head.MeshDataExtPos << std::endl;
	std::cout << "MeshDataSize:" << head.MeshDataSize << std::endl;
	std::cout << "MeshDataExtSize:" << head.MeshDataExtSize << std::endl;
	std::cout << "////////////////////////////////////////////////////////////////////////" << std::endl;
#endif // DebugMeshInfoOutput
	head.MagicNumber1 = 100;
	head.MagicNumber2 = 97;
	head.MagicNumber3 = 157;
	head.MagicNumber4 = 136;
	head.Version = 1009715;
	//WriteHead
	osData.write(reinterpret_cast<char*>(&head.MagicNumber1), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.MagicNumber2), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.MagicNumber3), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.MagicNumber4), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.Version), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshType), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataStartPos), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataSize), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataExtPos), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataExtSize), sizeof(int));

	//BuildMeshBody(meshData, body, osData);
	//extData.SerializePartialToOstream(&osData);
	//osData.close();
	//LoadMeshHead(filename);
}
void BuildMeshBody(FBXMesh& meshData, MeshBody& bodyinfo, std::ofstream& osData)
{
#if DebugMeshInfoOutput
	DebugMeshInfo(meshData, bodyinfo);
#endif // DebugMeshInfoOutput

	osData.write(reinterpret_cast<char*>(&bodyinfo.NameLength), 8);
	osData.write(meshData.name, bodyinfo.NameLength);

	osData.write(reinterpret_cast<char*>(&bodyinfo.VerticesLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.vertices.data()), bodyinfo.VerticesLength * sizeof(Vector3f));

	//color	
	osData.write(reinterpret_cast<char*>(&bodyinfo.ColorLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.colors.data()), bodyinfo.ColorLength * sizeof(ColorRGBA32));

	osData.write(reinterpret_cast<char*>(&bodyinfo.NormalLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.normals.data()), bodyinfo.NormalLength * sizeof(Vector3f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.UV1Length), 8);
	osData.write(reinterpret_cast<char*>(meshData.uv1.data()), bodyinfo.UV1Length * sizeof(Vector2f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.UV2Length), 8);
	osData.write(reinterpret_cast<char*>(meshData.uv2.data()), bodyinfo.UV2Length * sizeof(Vector2f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.IndexSizeLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.indicesize.data()), bodyinfo.IndexSizeLength * sizeof(uint32_t));

	osData.write(reinterpret_cast<char*>(&bodyinfo.IndexLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.indices.data()), bodyinfo.IndexLength * sizeof(uint32_t));

	osData.write(reinterpret_cast<char*>(&bodyinfo.MatLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.materialindex.data()), bodyinfo.MatLength * sizeof(uint32_t));

	//bindPose
	osData.write(reinterpret_cast<char*>(&bodyinfo.BindPosesLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.bindPoses.data()), bodyinfo.BindPosesLength * sizeof(Matrix4x4f));

	//BoneWeights4
	osData.write(reinterpret_cast<char*>(&bodyinfo.BoneWeightLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.boneWeights.data()), bodyinfo.BoneWeightLength * sizeof(BoneWeights4));

}
message::UGCResSkinnedMeshExtData BuildMeshExtData(FBXMesh& meshData)
{
	std::string meshName = meshData.name;
	message::UGCResSkinnedMeshExtData ext;
	if (gNodeName2BoneName.find(meshName) != gNodeName2BoneName.end())
	{
		auto bones = gNodeName2BoneName[meshName];
		for (auto i = 0; i < bones.size(); i++)
		{
			ext.add_bonenames(bones[i]);
		}
	}
	// For BlendShape Mesh Add bone
	if (gBlendShapeMesh2Bone.find(meshName) != gBlendShapeMesh2Bone.end())
	{
		auto bone = gBlendShapeMesh2Bone[meshName];
		ext.add_bonenames(bone);
		PrebuildFBXMeshForBlendShape(meshData);
	}
	return ext;
}

//Build Anim
void BuildSingleAnimProtoFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	auto& floatCurves = clip.floatAnimations;
	auto& nodeCurves = clip.nodeAnimations;
	message::UGCResAnimClipData AnimClipProto;

	auto sampleRate = scene.sceneInfo.sampleRate;
	auto fileScale = scene.fileScaleFactor;
	//::ReduceKeyframes(clip, 0.5f, 0.5f, 0.5f, 0.5f);
	ReduceKeyframes(clip, sampleRate, 0.5f, 0.5f, 0.5f, 0.5f);
	AnimClipProto.set_name(clip.name);
	AnimClipProto.set_bakestart(clip.bakeStart);
	AnimClipProto.set_bakestop(clip.bakeStop);
	AnimClipProto.set_samplerate(sampleRate);

	////Add Float Animation
	//for (auto it = floatCurves.begin(); it != floatCurves.end(); it++)
	//{
	//	message::UGCResAnimFloatCurves* curFloatProto = AnimClipProto.add_floatanim();
	//	curFloatProto->set_classname(it->className);
	//	curFloatProto->set_propertyname(it->propertyName);
	//	auto& curve = it->curve;
	//	auto& keyFrames = curve.m_Curve;
	//	message::FBXAnimationCurve* floatAnimCurveProto = curFloatProto->add_curve();
	//	for (auto i = 0; i < keyFrames.size(); i++)
	//	{
	//		auto& curKeyFrame = keyFrames[i];
	//		message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
	//		keyframeProto->set_time(curKeyFrame.time);
	//		keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
	//		keyframeProto->set_value(curKeyFrame.value);
	//		keyframeProto->set_inslope(curKeyFrame.inSlope);
	//		keyframeProto->set_outslope(curKeyFrame.outSlope);
	//		keyframeProto->set_inweight(curKeyFrame.inWeight);
	//		keyframeProto->set_outweight(curKeyFrame.outWeight);
	//	}
	//}
	//Add Node Animation
	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++)
	{
		message::UGCResAnimNodeCurves* curNodeProto = AnimClipProto.add_nodeanim();
		auto nodeName = it->node->name;

		for (auto it = gNodePath2Name.begin(); it != gNodePath2Name.end(); it++)
		{
			if (it->second == nodeName)
			{
				nodeName = it->first;
				break;
			}
		}
		curNodeProto->set_name(nodeName);

		auto& rotCurves = it->rotation;
		for (auto i = 0; i < 4; i++)
		{
			auto& curve = rotCurves[i];
			auto& keyFrames = curve.m_Curve;
			message::FBXAnimationCurve* floatAnimCurveProto = curNodeProto->add_rotation();
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
				keyframeProto->set_time(curKeyFrame.time);
				keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
				keyframeProto->set_value(curKeyFrame.value);
				keyframeProto->set_inslope(curKeyFrame.inSlope);
				keyframeProto->set_outslope(curKeyFrame.outSlope);
				keyframeProto->set_inweight(curKeyFrame.inWeight);
				keyframeProto->set_outweight(curKeyFrame.outWeight);
			}
		}
		auto& transCurves = it->translation;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = transCurves[i];
			auto& keyFrames = curve.m_Curve;
			message::FBXAnimationCurve* floatAnimCurveProto = curNodeProto->add_translation();
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
				keyframeProto->set_time(curKeyFrame.time);
				keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
				keyframeProto->set_value(curKeyFrame.value * fileScale);
				keyframeProto->set_inslope(curKeyFrame.inSlope);
				keyframeProto->set_outslope(curKeyFrame.outSlope);
				keyframeProto->set_inweight(curKeyFrame.inWeight);
				keyframeProto->set_outweight(curKeyFrame.outWeight);
			}
		}

		auto& scaleCurves = it->scale;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = scaleCurves[i];
			auto& keyFrames = curve.m_Curve;
			message::FBXAnimationCurve* floatAnimCurveProto = curNodeProto->add_scale();
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
				keyframeProto->set_time(curKeyFrame.time);
				keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
				keyframeProto->set_value(curKeyFrame.value);
				keyframeProto->set_inslope(curKeyFrame.inSlope);
				keyframeProto->set_outslope(curKeyFrame.outSlope);
				keyframeProto->set_inweight(curKeyFrame.inWeight);
				keyframeProto->set_outweight(curKeyFrame.outWeight);
			}
		}
	}
	std::string directory(outdir);
	std::string filename(clip.name);

	filename = directory + "/" + filename + ".Anim";
	std::fstream output(filename, std::ios::out | std::ios::trunc | std::ios::binary);
	bool flag = AnimClipProto.SerializePartialToOstream(&output);
	if (!flag)
	{
		std::cout << "Error when Serializing Anim File" << std::endl;
	}
	output.close();

	//Rename To Support Chinese		
	std::wstring meshfilenameW = ConvertUTF8ToWide(filename);
	RenameFileToWide(filename, meshfilenameW);

#if DebugMeshInfoOutput
	ParseAnimProto(AnimClipProto);
#endif
}
void BuildSingleAnimBinaryFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	// create ostream
	std::string Animfilename(clip.name);
	std::string directory(outdir);
	Animfilename = directory + "/" + Animfilename + ".Anim";
	std::ofstream osData(Animfilename, std::ios_base::out | std::ios_base::binary);
	osData.precision(8);


	//build head
	byte MagicNumber1 = 100;
	byte MagicNumber2 = 98;
	byte MagicNumber3 = 158;
	byte MagicNumber4 = 134;
	int Version = 1009715;

	float BakeStart = clip.bakeStart;
	float BakeStop = clip.bakeStop;
	float SampleRate = scene.sceneInfo.sampleRate;
	auto fileScale = scene.fileScaleFactor;
	int NameLength = clip.name.size();
	std::string Name = clip.name;

	//WriteHead
	osData.write(reinterpret_cast<char*>(&MagicNumber1), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&MagicNumber2), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&MagicNumber3), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&MagicNumber4), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&Version), sizeof(int));
	osData.write(reinterpret_cast<char*>(&BakeStart), sizeof(float));
	osData.write(reinterpret_cast<char*>(&BakeStop), sizeof(float));
	osData.write(reinterpret_cast<char*>(&SampleRate), sizeof(float));
	osData.write(reinterpret_cast<char*>(&NameLength), sizeof(int));
	osData.write(Name.data(),NameLength);
	
	ReduceKeyframes(clip, SampleRate, 0.5f, 0.5f, 0.5f, 0.5f);

	//Node Anim
	auto& nodeCurves = clip.nodeAnimations;
	int NodeAnimCurveCount = clip.nodeAnimations.size();
	osData.write(reinterpret_cast<char*>(&NodeAnimCurveCount), sizeof(int));

	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++)
	{
		auto nodeName = it->node->name;

		for (auto it = gNodePath2Name.begin(); it != gNodePath2Name.end(); it++)
		{
			if (it->second == nodeName)
			{
				nodeName = it->first;
				break;
			}
		}
		int NodeNameLength = nodeName.size();
		osData.write(reinterpret_cast<char*>(&NodeNameLength), sizeof(int));
		osData.write(nodeName.data(), NodeNameLength);


		auto& rotCurves = it->rotation;
		for (auto i = 0; i < 4; i++)
		{
			auto& curve = rotCurves[i];
			auto& keyFrames = curve.m_Curve;
			int RotKeyFrameCount = keyFrames.size();
			osData.write(reinterpret_cast<char*>(&RotKeyFrameCount), sizeof(int));
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				osData.write(reinterpret_cast<char*>(&curKeyFrame.weightedMode), sizeof(int));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.time), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.value), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inWeight), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outWeight), sizeof(float));

			}
		}
		auto& scaleCurves = it->scale;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = scaleCurves[i];
			auto& keyFrames = curve.m_Curve;
			int ScaleKeyFrameCount = keyFrames.size();
			osData.write(reinterpret_cast<char*>(&ScaleKeyFrameCount), sizeof(int));
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				osData.write(reinterpret_cast<char*>(&curKeyFrame.weightedMode), sizeof(int));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.time), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.value), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inWeight), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outWeight), sizeof(float));
			}
		}
		auto& transCurves = it->translation;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = transCurves[i];
			auto& keyFrames = curve.m_Curve;
			int TransKeyFrameCount = keyFrames.size();
			osData.write(reinterpret_cast<char*>(&TransKeyFrameCount), sizeof(int));
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				//apply scale
				curKeyFrame.value = curKeyFrame.value * fileScale;
				osData.write(reinterpret_cast<char*>(&curKeyFrame.weightedMode), sizeof(int));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.time), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.value), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inWeight), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outWeight), sizeof(float));

			}
		}
	}

	osData.close();
	//Rename To Support Chinese		
	std::wstring AnimfilenameW = ConvertUTF8ToWide(Animfilename);
	RenameFileToWide(Animfilename, AnimfilenameW);
	//ParseSingleAnim(Animfilename);

}

void WriteNodeAnimationsToText(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	std::string Animfilename(clip.name);
	std::string directory(outdir);
	Animfilename = directory + "/" + Animfilename + "_anim.txt";
	std::ofstream osData(Animfilename);


	float fileScale = scene.fileScaleFactor;
	auto& nodeCurves = clip.nodeAnimations;
	int NodeAnimCurveCount = nodeCurves.size();


	//build head
	byte MagicNumber1 = 100;
	byte MagicNumber2 = 98;
	byte MagicNumber3 = 158;
	byte MagicNumber4 = 134;
	int Version = 1009715;

	float BakeStart = clip.bakeStart;
	float BakeStop = clip.bakeStop;
	float SampleRate = scene.sceneInfo.sampleRate;
	int NameLength = clip.name.size();
	std::string Name = clip.name;

	osData << MagicNumber1 << std::endl;
	osData << MagicNumber2 << std::endl;
	osData << MagicNumber3 << std::endl;
	osData << MagicNumber4 << std::endl;
	osData << Version << std::endl;
	osData << BakeStart << std::endl;
	osData << BakeStop << std::endl;
	osData << SampleRate << std::endl;
	osData << NameLength << std::endl;
	osData << Name.data() << std::endl;
	osData << "**********************************************" << std::endl;
	osData << "Node Anim Count: " << NodeAnimCurveCount << std::endl;

	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++) {
		auto nodeName = it->node->name;
		for (auto it = gNodePath2Name.begin(); it != gNodePath2Name.end(); it++) {
			if (it->second == nodeName) {
				nodeName = it->first;
				break;
			}
		}
		int NodeNameLength = nodeName.size();
		osData << "Node Anim NodeNameLength: " << NodeNameLength << std::endl;
		osData << "Node Anim NodeName: " << nodeName.data() << std::endl;

		auto& rotCurves = it->rotation;
		for (auto i = 0; i < 4; i++) {
			auto& curve = rotCurves[i];
			auto& keyFrames = curve.m_Curve;
			int RotKeyFrameCount = keyFrames.size();
			osData << "Node Anim RotKeyFrameCount: " << RotKeyFrameCount << std::endl;
			for (auto j = 0; j < keyFrames.size(); j++) {
				auto& curKeyFrame = keyFrames[j];

				osData << "    Channel [" << i << " ] Num [ " << j << "] weightMode " << curKeyFrame.weightedMode << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] time " << curKeyFrame.time << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] value " << curKeyFrame.value << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inSlope " << curKeyFrame.inSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outSlope " << curKeyFrame.outSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inWeight " << curKeyFrame.inWeight << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outWeight " << curKeyFrame.outWeight << std::endl;
			}
		}

		auto& scaleCurves = it->scale;
		for (auto i = 0; i < 3; i++) {
			auto& curve = scaleCurves[i];
			auto& keyFrames = curve.m_Curve;
			int ScaleKeyFrameCount = keyFrames.size();
			osData << "Node Anim ScaleKeyFrameCount: " << ScaleKeyFrameCount << std::endl;
			for (auto j = 0; j < keyFrames.size(); j++) {
				auto& curKeyFrame = keyFrames[j];
				osData << "    Channel [" << i << " ] Num [ " << j << "] weightMode " << curKeyFrame.weightedMode << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] time " << curKeyFrame.time << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] value " << curKeyFrame.value << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inSlope " << curKeyFrame.inSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outSlope " << curKeyFrame.outSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inWeight " << curKeyFrame.inWeight << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outWeight " << curKeyFrame.outWeight << std::endl;
			}
		}

		auto& transCurves = it->translation;
		for (auto i = 0; i < 3; i++) {
			auto& curve = transCurves[i];
			auto& keyFrames = curve.m_Curve;
			int TransKeyFrameCount = keyFrames.size();
			osData << "Node Anim TransKeyFrameCount: " << TransKeyFrameCount << std::endl;
			for (auto j = 0; j < keyFrames.size(); j++) {
				auto& curKeyFrame = keyFrames[j];
				// apply scale
				curKeyFrame.value = curKeyFrame.value * fileScale;
				osData << "    Channel [" << i << " ] Num [ " << j << "] weightMode " << curKeyFrame.weightedMode << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] time " << curKeyFrame.time << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] value " << curKeyFrame.value << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inSlope " << curKeyFrame.inSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outSlope " << curKeyFrame.outSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inWeight " << curKeyFrame.inWeight << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outWeight " << curKeyFrame.outWeight << std::endl;
			}
		}

		osData << "***************************************" << std::endl;
	}

	osData.close();
	//Rename To Support Chinese		
	std::wstring AnimfilenameW = ConvertUTF8ToWide(Animfilename);
	RenameFileToWide(Animfilename, AnimfilenameW);
}


//Build Bones
void BuildBoneNodeData(FBXImportScene& scene, FBXImportNode& node, message::UGCResBoneNodeData* parent)
{
	message::UGCResBoneNodeData* msg_node = parent->add_childbones();
	message::UGCResBoneNodeCapsuleData root_capsule;

	message::ProtoBuffVector3* msg_pos = new message::ProtoBuffVector3();
	message::ProtoBuffVector3* msg_scale = new message::ProtoBuffVector3();
	message::ProtoBuffQuaternion* msg_quat = new message::ProtoBuffQuaternion();
	auto scale = scene.fileScaleFactor;
	msg_pos->set_x(node.position.x * scale); msg_pos->set_y(node.position.y * scale); msg_pos->set_z(node.position.z * scale);
	msg_scale->set_x(node.scale.x); msg_scale->set_y(node.scale.y); msg_scale->set_z(node.scale.z);
	msg_quat->set_x(node.rotation.x); msg_quat->set_y(node.rotation.y); msg_quat->set_z(node.rotation.z); msg_quat->set_w(node.rotation.w);

	msg_node->set_bonename(node.name);
	//std::cout << " Set Bone :" << node.name << std::endl;
	//msg_node->set_allocated_capsuleinfo(&root_capsule);
	msg_node->set_allocated_localposition(msg_pos);
	msg_node->set_allocated_localrotation(msg_quat);
	msg_node->set_allocated_localscale(msg_scale);
	for (auto i = 0; i < node.children.size(); i++)
	{
		auto nextNode = node.children[i];
		BuildBoneNodeData(scene, nextNode, msg_node);
	}
}

#pragma region DebugInfo
void ParseSingleMesh(std::string meshfile)
{
	std::cout << "*******************************Begin Parse Single Mesh**************************" << std::endl;
	std::ifstream istream(meshfile, std::ios_base::in | std::ios_base::binary);

	MeshHead head;
	istream.read(reinterpret_cast<char*>(&head.MagicNumber1), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.MagicNumber2), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.MagicNumber3), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.MagicNumber4), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.Version), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshType), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataStartPos), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataSize), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataExtPos), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataExtSize), sizeof(int));

	std::cout << "filename: " << meshfile << std::endl;
	std::cout << "MagicNumber: " << (int)head.MagicNumber1 << "," << (int)head.MagicNumber2 << "," << (int)head.MagicNumber3 << "," << (int)head.MagicNumber4 << std::endl;
	std::cout << "Version: " << head.Version << std::endl;
	std::cout << "MeshType: " << head.MeshType << std::endl;
	std::cout << "Version: " << head.Version << std::endl;
	std::cout << "MeshDataStartPos: " << head.MeshDataStartPos << std::endl;
	std::cout << "MeshDataSize: " << head.MeshDataSize << std::endl;
	std::cout << "MeshDataExtPos: " << head.MeshDataExtPos << std::endl;
	std::cout << "MeshDataExtSize: " << head.MeshDataExtSize << std::endl;

	MeshBody body;

	//Name
	{
		istream.read(reinterpret_cast<char*>(&body.NameLength), 8);
		std::string name;
		istream.read(reinterpret_cast<char*>(&name), body.NameLength);
		std::cout << "[BODY]Name: " << name.c_str() << std::endl;
	}
	//vertices
	{
		istream.read(reinterpret_cast<char*>(&body.VerticesLength), 8);
		std::vector<Vector3f> verts;
		verts.clear();
		for (auto i = 0; i < body.VerticesLength; i++)
		{
			Vector3f vert;
			istream.read(reinterpret_cast<char*>(&vert.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&vert.y), sizeof(float));
			istream.read(reinterpret_cast<char*>(&vert.z), sizeof(float));
			verts.push_back(vert);
		}
		std::cout << "[BODY]Vertices: " << verts.size() << std::endl;
	}

	//normals
	{
		istream.read(reinterpret_cast<char*>(&body.NormalLength), 8);
		std::vector<Vector3f> normals;
		normals.clear();
		for (auto i = 0; i < body.NormalLength; i++)
		{
			Vector3f norm;
			istream.read(reinterpret_cast<char*>(&norm.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&norm.y), sizeof(float));
			istream.read(reinterpret_cast<char*>(&norm.z), sizeof(float));
			normals.push_back(norm);
		}
		std::cout << "[BODY]Normal: " << normals.size() << std::endl;
	}

	//uv1
	{
		istream.read(reinterpret_cast<char*>(&body.UV1Length), 8);
		std::vector<Vector2f> uv1s;
		uv1s.clear();
		for (auto i = 0; i < body.UV1Length; i++)
		{
			Vector2f uv;
			istream.read(reinterpret_cast<char*>(&uv.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&uv.y), sizeof(float));
			uv1s.push_back(uv);
		}
		std::cout << "[BODY]UV1: " << uv1s.size() << std::endl;
	}

	//uv2
	{
		istream.read(reinterpret_cast<char*>(&body.UV2Length), 8);
		std::vector<Vector2f> uv2s;
		uv2s.clear();
		for (auto i = 0; i < body.UV2Length; i++)
		{
			Vector2f uv;
			istream.read(reinterpret_cast<char*>(&uv.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&uv.y), sizeof(float));
			uv2s.push_back(uv);
		}
		std::cout << "[BODY]UV2: " << uv2s.size() << std::endl;
	}

	//IndexSize
	{
		istream.read(reinterpret_cast<char*>(&body.IndexSizeLength), 8);
		std::vector<uint32_t> indexSizes;
		indexSizes.clear();
		for (auto i = 0; i < body.IndexSizeLength; i++)
		{
			uint32_t idxsize;
			istream.read(reinterpret_cast<char*>(&idxsize), sizeof(uint32_t));
			indexSizes.push_back(idxsize);
		}
		std::cout << "[BODY]IndexSize: " << indexSizes.size() << std::endl;
	}

	//Index
	{
		istream.read(reinterpret_cast<char*>(&body.IndexLength), 8);
		std::vector<uint32_t> indexes;
		indexes.clear();
		for (auto i = 0; i < body.IndexLength; i++)
		{
			uint32_t idxsize;
			istream.read(reinterpret_cast<char*>(&idxsize), sizeof(uint32_t));
			indexes.push_back(idxsize);
		}
		std::cout << "[BODY]Index: " << indexes.size() << std::endl;
	}

	//Material
	{
		istream.read(reinterpret_cast<char*>(&body.MatLength), 8);
		std::vector<uint32_t> mats;
		mats.clear();
		for (auto i = 0; i < body.MatLength; i++)
		{
			uint32_t mat;
			istream.read(reinterpret_cast<char*>(&mat), sizeof(uint32_t));
			mats.push_back(mat);
		}
		std::cout << "[BODY]Index: " << mats.size() << std::endl;
	}

	message::UGCResSkinnedMeshExtData ext1;
	bool flag = ext1.ParseFromIstream(&istream);
	if (flag)
	{
		std::cout << "Byte Size of ext new " << ext1.ByteSizeLong() << std::endl;
		if (ext1.ByteSizeLong() > 0)
		{
			for (auto i = 0; i < ext1.bonenames_size(); i++)
			{
				std::cout << "the [" << i << "] Bone is " << ext1.bonenames(i) << std::endl;
			}
		}
	}
	std::cout << "*******************************End Parse Single Mesh**************************" << std::endl;

}
void DebugMeshInfo(FBXMesh& meshData, MeshBody& bodyinfo)
{

	uint64_t namecount = strlen(meshData.name) + 1;
	uint64_t vertexcount = meshData.vertices.size();
	uint64_t normalcount = meshData.normals.size();
	uint64_t uv1count = meshData.uv1.size();
	uint64_t uv2count = meshData.uv2.size();
	uint64_t indicesizecount = meshData.indicesize.size();
	uint64_t indicecount = meshData.indices.size();
	uint64_t matcount = meshData.materialindex.size();
	std::cout << "////////////////////////////////////////////////////////////////////////" << std::endl;
	std::cout << "name: " << meshData.name << std::endl;
	std::cout << "namecount: " << namecount << std::endl;
	std::cout << "vertexcount: " << vertexcount << std::endl;
	std::cout << "normalcount: " << normalcount << std::endl;
	std::cout << "uv1count: " << uv1count << std::endl;
	std::cout << "uv2count: " << uv2count << std::endl;
	std::cout << "indicesizecount: " << indicesizecount << std::endl;
	std::cout << "indicecount: " << indicecount << std::endl;

	std::cout << "************************************************************************" << std::endl;
	std::cout << "bodyinfo.NamePos :" << bodyinfo.NamePos << std::endl;
	std::cout << "bodyinfo.NameHeadLength :" << bodyinfo.NameHeadLength << std::endl;
	std::cout << "bodyinfo.NameLength :" << bodyinfo.NameLength << std::endl;
	std::cout << "bodyinfo.VerticesPos :" << bodyinfo.VerticesPos << std::endl;
	std::cout << "bodyinfo.VerticesHeadLength :" << bodyinfo.VerticesHeadLength << std::endl;
	std::cout << "bodyinfo.VerticesLength :" << bodyinfo.VerticesLength << std::endl;
	std::cout << "bodyinfo.NormalPos :" << bodyinfo.NormalPos << std::endl;
	std::cout << "bodyinfo.NormalHeadLength :" << bodyinfo.NormalHeadLength << std::endl;
	std::cout << "bodyinfo.NormalLength :" << bodyinfo.NormalLength << std::endl;
	std::cout << "bodyinfo.UV1Pos :" << bodyinfo.UV1Pos << std::endl;
	std::cout << "bodyinfo.UV1HeadLength :" << bodyinfo.UV1HeadLength << std::endl;
	std::cout << "bodyinfo.UV1Length :" << bodyinfo.UV1Length << std::endl;
	std::cout << "bodyinfo.UV2Pos :" << bodyinfo.UV2Pos << std::endl;
	std::cout << "bodyinfo.UV2HeadLength :" << bodyinfo.UV2HeadLength << std::endl;
	std::cout << "bodyinfo.UV2Length :" << bodyinfo.UV2Length << std::endl;
	std::cout << "bodyinfo.IndexSizePos :" << bodyinfo.IndexSizePos << std::endl;
	std::cout << "bodyinfo.IndexSizeHeadLength :" << bodyinfo.IndexSizeHeadLength << std::endl;
	std::cout << "bodyinfo.IndexSizeLength :" << bodyinfo.IndexSizeLength << std::endl;
	std::cout << "bodyinfo.IndexPos :" << bodyinfo.IndexPos << std::endl;
	std::cout << "bodyinfo.IndexHeadLength :" << bodyinfo.IndexHeadLength << std::endl;
	std::cout << "bodyinfo.IndexLength :" << bodyinfo.IndexLength << std::endl;
	std::cout << "bodyinfo.MatPos :" << bodyinfo.MatPos << std::endl;
	std::cout << "bodyinfo.MatHeadLength :" << bodyinfo.MatHeadLength << std::endl;
	std::cout << "bodyinfo.MatLength :" << bodyinfo.MatLength << std::endl;
	std::cout << "bodyinfo.BindPosesPos :" << bodyinfo.BindPosesPos << std::endl;
	std::cout << "bodyinfo.BindPosesHeadLength :" << bodyinfo.BindPosesHeadLength << std::endl;
	std::cout << "bodyinfo.BindPosesLength :" << bodyinfo.BindPosesLength << std::endl;
	std::cout << "bodyinfo.BoneWeightPos :" << bodyinfo.BoneWeightPos << std::endl;
	std::cout << "bodyinfo.BoneWeightHeadLength :" << bodyinfo.BoneWeightHeadLength << std::endl;
	std::cout << "bodyinfo.BoneWeightLength :" << bodyinfo.BoneWeightLength << std::endl;

}
void ParseAnimProto(message::UGCResAnimClipData& msg)
{
	std::cout << "*****************************Begin Parse Anim Proto!*********************************" << std::endl;
	//name
	auto name = msg.name();
	std::cout << "The Name is :" << name.c_str() << std::endl;
	//BakeStart,End
	auto start = msg.bakestart();
	auto stop = msg.bakestop();
	std::cout << "The bake start stop are : " << start << " , " << stop << std::endl;
	auto samplerate = msg.samplerate();
	std::cout << "The Sample Rate is : " << samplerate << std::endl;
	//Float Anim
	auto floatAnimSize = msg.floatanim_size();
	for (auto i = 0; i < floatAnimSize; i++)
	{
		auto floatAnim = msg.floatanim(i);
		std::cout << "The [" << i << "] th Float Anim are list below: " << std::endl;
		std::cout << "		The ClassName is : " << floatAnim.classname().c_str() << std::endl;
		std::cout << "		The PropertyName is : " << floatAnim.propertyname().c_str() << std::endl;
		std::cout << "		The CurveCount is : " << floatAnim.curve_size() << std::endl;
		for (int j = 0; j < floatAnim.curve_size(); j++)
		{
			auto curve = floatAnim.curve(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << curve.keyframes_size() << std::endl;
		}
	}

	//Node Anim
	auto nodeAnimSize = msg.nodeanim_size();
	for (auto i = 0; i < nodeAnimSize; i++)
	{
		auto nodeAnim = msg.nodeanim(i);
		auto name = nodeAnim.name();
		std::cout << "The [" << i << "] th Node Anim:" << name << " are list below: " << std::endl;
		std::cout << "		The Rotation CurveChannel is : " << nodeAnim.rotation_size() << std::endl;
		for (int j = 0; j < nodeAnim.rotation_size(); j++)
		{
			auto rot = nodeAnim.rotation(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << rot.keyframes_size() << std::endl;
		}

		std::cout << "		The Scale CurveChannel is : " << nodeAnim.scale_size() << std::endl;
		for (int j = 0; j < nodeAnim.scale_size(); j++)
		{
			auto scale = nodeAnim.scale(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << scale.keyframes_size() << std::endl;
		}
		std::cout << "		The Transition CurveChannel is : " << nodeAnim.translation_size() << std::endl;
		for (int j = 0; j < nodeAnim.translation_size(); j++)
		{
			auto trans = nodeAnim.translation(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << trans.keyframes_size() << std::endl;
		}
	}

	std::cout << "*****************************End Parse Anim Proto!*********************************" << std::endl;
}
void ParseSingleAnim(std::string animfile)
{
	std::cout << "*******************************Begin Parse Single Anim**************************" << std::endl;
	std::ifstream istream(animfile, std::ios_base::in | std::ios_base::binary);

	byte MagicNum[4] = { 0,0,0,0 };
	int Version = -1;
	float BakeStart = 0;
	float BakeStop = 0;
	float SampleRate = 0;
	int NameLength = 0;
	std::string Name;



	istream.read(reinterpret_cast<char*>(&MagicNum[0]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&MagicNum[1]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&MagicNum[2]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&MagicNum[3]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&Version), sizeof(int));
	istream.read(reinterpret_cast<char*>(&BakeStart), sizeof(float));
	istream.read(reinterpret_cast<char*>(&BakeStop), sizeof(float));
	istream.read(reinterpret_cast<char*>(&SampleRate), sizeof(float));
	istream.read(reinterpret_cast<char*>(&NameLength), sizeof(int));
	Name.resize(NameLength);
	istream.read(&Name[0], NameLength);



	std::cout << "MagicNumber: " << (int)MagicNum[0] << "," << (int)MagicNum[1] << "," << (int)MagicNum[2] << "," << (int)MagicNum[3] << std::endl;
	std::cout << "Version: " << Version << std::endl;
	std::cout << "BakeStart: " << BakeStart << std::endl;
	std::cout << "BakeStop: " << BakeStop << std::endl;
	std::cout << "SampleRate: " << SampleRate << std::endl;
	std::cout << "NameLength: " << NameLength << std::endl;
	std::cout << "filename: " << Name << std::endl;
	int NodeCount = 0;
	istream.read(reinterpret_cast<char*>(&NodeCount), sizeof(int));
	std::cout << "NodeCount: " << NodeCount << std::endl;
	for (auto i = 0; i < NodeCount; i++)
	{
		int NodeNameLength = -1;
		std::string NodeName;
		istream.read(reinterpret_cast<char*>(&NodeNameLength), sizeof(int));
		NodeName.resize(NodeNameLength);
		istream.read(&NodeName[0], NodeNameLength);

		//std::cout << "Node NodeNameLength: " << NodeNameLength << std::endl;
		std::cout << "Node Name: " << NodeName << std::endl;
	

		int weightedMode = -1;
		float time, value, inSlope, outSlope, inWeight, outWeight;

		//Rot
		for (auto channel = 0; channel < 4; channel++)
		{
			int RotKeyFrameCount = 0;
			istream.read(reinterpret_cast<char*>(&RotKeyFrameCount), sizeof(int));
			if (RotKeyFrameCount != 0)
			{
				std::cout << "Node RotKeyFrameCount channel : " << (int)channel << "Node RotKeyFrameCount: " << RotKeyFrameCount<<std::endl;
			}
			for (auto id = 0; id < RotKeyFrameCount; id++)
			{
				istream.read(reinterpret_cast<char*>(&weightedMode), sizeof(int));
				istream.read(reinterpret_cast<char*>(&time), sizeof(float));
				istream.read(reinterpret_cast<char*>(&value), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inWeight), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outWeight), sizeof(float));
				//std::cout << "	Rot :["<< i<<" ] channel: " << (int)channel << std::endl;
				//std::cout << "		weightedMode: " << (int)weightedMode << std::endl;
				std::cout << "		time: " << time << std::endl;
				std::cout << "		value: " << value << std::endl;
				//std::cout << "		inSlope: " << inSlope << std::endl;
				//std::cout << "		outSlope: " << outSlope << std::endl;
				//std::cout << "		inWeight: " << inWeight << std::endl;
				//std::cout << "		outWeight: " << outWeight << std::endl;
			}
		}
		//Scale
		for (auto channel = 0; channel < 3; channel++)
		{
			int ScaleKeyFrameCount = 0;
			istream.read(reinterpret_cast<char*>(&ScaleKeyFrameCount), sizeof(int));
			if (ScaleKeyFrameCount != 0)
			{
				std::cout << "Node ScaleKeyFrameCount channel : " << (int)channel << "Node ScaleKeyFrameCount: " << ScaleKeyFrameCount << std::endl;
			}


			for (auto id = 0; id < ScaleKeyFrameCount; id++)
			{
				istream.read(reinterpret_cast<char*>(&weightedMode), sizeof(int));
				istream.read(reinterpret_cast<char*>(&time), sizeof(float));
				istream.read(reinterpret_cast<char*>(&value), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inWeight), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outWeight), sizeof(float));
				//std::cout << "	Scale :[" << i << " ] channel: " << (int)channel << std::endl;
				//std::cout << "		weightedMode: " << (int)weightedMode << std::endl;
				std::cout << "		time: " << time << std::endl;
				std::cout << "		value: " << value << std::endl;
				//std::cout << "		inSlope: " << inSlope << std::endl;
				//std::cout << "		outSlope: " << outSlope << std::endl;
				//std::cout << "		inWeight: " << inWeight << std::endl;
				//std::cout << "		outWeight: " << outWeight << std::endl;
			}
		}
		//Trans
		for (auto channel = 0; channel < 3; channel++)
		{
			int TransKeyFrameCount = 0;
			istream.read(reinterpret_cast<char*>(&TransKeyFrameCount), sizeof(int));
			if (TransKeyFrameCount != 0)
			{
				std::cout << "Node TransKeyFrameCount channel : " << (int)channel << "Node TransKeyFrameCount: " << TransKeyFrameCount << std::endl;
			}
			for (auto id = 0; id < TransKeyFrameCount; id++)
			{
				istream.read(reinterpret_cast<char*>(&weightedMode), sizeof(int));
				istream.read(reinterpret_cast<char*>(&time), sizeof(float));
				istream.read(reinterpret_cast<char*>(&value), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inWeight), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outWeight), sizeof(float));
				//std::cout << "	Trans :[" << i << " ] channel: " << (int)channel << std::endl;
				//std::cout << "		weightedMode: " << (int)weightedMode << std::endl;
				std::cout << "		time: " << time << std::endl;
				std::cout << "		value: " << value << std::endl;
				//std::cout << "		inSlope: " << inSlope << std::endl;
				//std::cout << "		outSlope: " << outSlope << std::endl;
				//std::cout << "		inWeight: " << inWeight << std::endl;
				//std::cout << "		outWeight: " << outWeight << std::endl;
			}
		}

	}



	std::cout << "*******************************End Parse Single Mesh**************************" << std::endl;

}
#pragma endregion
