#pragma once

#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <atlbase.h>
#include <atlsafe.h>
#include <atlstr.h>
#include <fbxsdk.h>
#include <io.h>
#include <direct.h>
#include <cmath>
#include "MathBasic.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Conversion.h"
#include "floatConversion.h"
#include "RotationOrder.h"
//#include "stringFormat.h"
#ifndef M_PI
#define M_PI	3.1415926535897932384626433832795
#endif

#define MARK_DEGENERATE				1
#define QUAD_ONE_DEGEN_TRI			2
#define GROUP_WITH_ANY				4
#define ORIENT_PRESERVING			8

#define INTERNAL_RND_SORT_SEED		39871946


#define MAT(m, r, c) (m)[(c)*4+(r)]

#define RETURN_ZERO PP_WRAP_CODE(\
    for (int i=0;i<16;i++) \
        out[i] = 0.0F; \
    return false;\
)

#define PP_WRAP_CODE(CODE_) \
    do { CODE_; } while (0)

#define SWAP_ROWS(a, b) PP_WRAP_CODE(float *_tmp = a; (a)=(b); (b)=_tmp;)

typedef __m128 Simd128;
#   define V4LoadUnaligned(base, offset)            _mm_loadu_ps((base)+(offset))

#define _MM_SHUFFLE(fp3,fp2,fp1,fp0) (((fp3) << 6) | ((fp2) << 4) | \
                                     ((fp1) << 2) | ((fp0)))

#   define V4Mul(v0, v1)        _mm_mul_ps((v0), (v1))
#   define V4MulAdd(v0, v1, v2) _mm_add_ps(_mm_mul_ps((v0), (v1)), (v2))
#   define V4Splat(v0, i)           _mm_shuffle_ps((v0), (v0), _MM_SHUFFLE(i,i,i,i))
#   define V4StoreUnaligned(value, base, offset)    _mm_storeu_ps((base)+(offset), value)


const int g_iCells = 2048;


class Matrix3x3f;
class Matrix4x4f;
class Quaternionf;

typedef std::map<FbxSurfaceMaterial*, int> FBXMaterialLookup;



inline bool InvertMatrix4x4_Full(const float* m, float* out);


inline Vector3f FBXPointToVector3Remap(const FbxVector4& vec)
{
	//return FBXPointToVector3( vec );
	return Vector3f(
		static_cast<float>(-vec.mData[0]),
		static_cast<float>(vec.mData[1]),
		static_cast<float>(vec.mData[2])
	);
}

inline Vector3f FBXPointToVector3RemapWith(const FbxVector4& vec)
{
	//return FBXPointToVector3( vec );
	return Vector3f(
		static_cast<float>(-vec.mData[0]),
		static_cast<float>(vec.mData[1]),
		static_cast<float>(vec.mData[2])
	);
}

inline Vector3f FBXPointToVector3(const FbxVector4& vec)
{
	return Vector3f(
		static_cast<float>(vec.mData[0]),
		static_cast<float>(vec.mData[1]),
		static_cast<float>(vec.mData[2])
	);
}


inline Vector2f FBXToUnityBasicType(const FbxVector2& t)
{
	return Vector2f(
		static_cast<float>(t.mData[0]),
		static_cast<float>(t.mData[1])
	);
}

inline ColorRGBAf FBXColorToColorRGBA(const FbxColor& vec)
{
	return ColorRGBAf(
		static_cast<float>(vec.mRed),
		static_cast<float>(vec.mGreen),
		static_cast<float>(vec.mBlue),
		static_cast<float>(vec.mAlpha)
	);
}

inline ColorRGBA32 FBXToUnityBasicType(const FbxColor& t)
{
	return FBXColorToColorRGBA(t);
}


inline Vector3f FBXToUnityBasicType(const FbxVector4& t)
{
	return FBXPointToVector3Remap(t);
}

enum DegenFace { kDegenNone, kDegenFull, kDegenToTri };
static inline DegenFace IsDegenerateFace(const uint32_t* f, const int faceSize)
{
	if (faceSize == 3)
	{
		// triangle
		return (f[0] == f[1] || f[0] == f[2] || f[1] == f[2]) ? kDegenFull : kDegenNone;
	}
	else if (faceSize == 4)
	{
		// quad

		// non-neighbor indices are the same: quad is fully degenerate (produces no output)
		if (f[0] == f[2] || f[1] == f[3])
			return kDegenFull;
		// two opposing sides are collapsed: fully degenerate
		if ((f[0] == f[1] && f[2] == f[3]) || (f[1] == f[2] && f[3] == f[0]))
			return kDegenFull;
		// just one side is collapsed: degenerate to triangle
		if (f[0] == f[1] || f[1] == f[2] || f[2] == f[3] || f[3] == f[0])
			return kDegenToTri;
		// otherwise, regular quad
		return kDegenNone;
	}
	else
	{
		return kDegenFull;
	}
}

inline uint32_t GetVector3HashValue(const Vector3f& value)
{
	const uint32_t* h = (const uint32_t*)(&value);
	uint32_t f = (h[0] + h[1] * 11 - (h[2] * 17)) & 0x7fffffff; // avoid problems with +-0
	return (f >> 22) ^ (f >> 12) ^ (f);
}

inline int nextPowerOfTwo(uint32_t v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v + (v == 0);
}

template<typename T>
static void AddFace(const typename std::vector<T>& src, typename std::vector<T>& dst, int idx, const uint32_t* indices, int faceSize)
{
	if (src.empty())
		return;
	for (int i = 0; i < faceSize; ++i)
		dst.push_back(src[idx + indices[i]]);
}


template<class T, class T2>
class GetValueAtDirect
{
public:
	GetValueAtDirect(const FbxLayerElementTemplate<T>* element)
		: input(&element->GetDirectArray()),
		directArraySize(input->GetCount())
	{
	}

	bool GetValue(int index, T2& value) const
	{
		if (index < 0 || index >= directArraySize)
			return false;

		value = FBXToUnityBasicType(input->GetAt(index));
		return true;
	}

	int GetDirectArraySize() const { return directArraySize; }

	bool IsValid() const { return GetSize() > 0; }
	int GetSize() const { return directArraySize; }

private:
	const FbxLayerElementArrayTemplate<T>* input;
	int directArraySize;
};

// Wrapper for getting data from FbxLayerElementTemplate when reference mode is FbxLayerElement::eIndexToDirect
template<class T, class T2>
class GetValueAtIndex : public GetValueAtDirect<T, T2>
{
public:
	GetValueAtIndex(const FbxLayerElementTemplate<T>* element)
		: GetValueAtDirect<T, T2>(element),
		layerIndices(&element->GetIndexArray()),
		indexArraySize(layerIndices->GetCount())
	{
	}

	bool GetValue(int index, T2& value) const
	{
		if (index < 0 || index >= indexArraySize)
			return false;

		const int index2 = layerIndices->GetAt(index);
		return GetValueAtDirect<T, T2>::GetValue(index2, value);
	}

	bool IsValid() const { return GetValueAtDirect<T, T2>::IsValid() && GetSize() > 0; }
	int GetSize() const { return indexArraySize; }

private:
	const FbxLayerElementArrayTemplate<int>* layerIndices;
	int indexArraySize;
};

template<class T, class T2, class GetValueAt>
void ExtractWedgeLayerData1(const FbxLayerElementTemplate<T>* element, std::vector<T2>& output, const int* indices, const int indexCount, 
	const std::vector<uint32_t>& polygonSizes, const int polygonCount, const int vertexCount, const char* layerName, const std::string& meshName, const T2& defaultValue)
{
	GetValueAt input(element);
	if (!input.IsValid())
		return;

	output.resize(indexCount, defaultValue);

	const FbxLayerElement::EMappingMode mappingMode = element->GetMappingMode();

	if (mappingMode == FbxLayerElement::eByControlPoint)
	{
		// TODO : I'm not sure this makes sense when eIndexToDirect is used
		if (input.GetDirectArraySize() != vertexCount)
		{
			output.clear();
			std::cout << "Try cleaning and triangulating in your 3D modeller before importing it in unity" << std::endl;
			//ReportError("The mesh %s has invalid %s. Try cleaning and triangulating %s in your 3D modeller before importing it in unity.\nThis is a failure in the fbx exporter of the tool which exported the fbx file.", meshName.c_str(), layerName, meshName.c_str(), layerName);
			return;
		}

		for (int f = 0; f < indexCount; f++)
		{
			int index = indices[f];
			input.GetValue(index, output[f]);
		}
	}
	else if (mappingMode == FbxLayerElement::eByPolygonVertex)
	{
		for (int f = 0; f < indexCount; ++f)
			input.GetValue(f, output[f]);
	}
	else if (mappingMode == FbxLayerElement::eByPolygon)
	{
		int wedgeIndex = 0;
		for (int f = 0; f < polygonCount; f++)
			for (uint32_t e = 0; e < polygonSizes[f]; e++, wedgeIndex++)
				input.GetValue(f, output[wedgeIndex]);
	}
	else if (mappingMode == FbxLayerElement::eAllSame)
	{
		T2 value;
		if (input.GetValue(0, value))
		{
			for (int f = 0; f < indexCount; f++)
				output[f] = value;
		}
	}
	else
	{
		std::cout << "Unsupported wedge mapping mode type.Please report this bug" << std::endl;
		//printf_console("Unsupported wedge mapping %d", (int)mappingMode);
		//ReportError("Unsupported wedge mapping mode type. Please report this bug.\n");
	}
}

template<class T, class T2>
void ExtractWedgeLayerData(const FbxLayerElementTemplate<T>* element, std::vector<T2>& output, const int* indices, const int indexCount, const std::vector<uint32_t>& polygonSizes, const int polygonCount, int vertexCount, const char* layerName, const std::string& meshName, const T2& defaultValue = T2())
{
	const FbxLayerElement::EReferenceMode referenceMode = element->GetReferenceMode();

	if (referenceMode == FbxLayerElement::eDirect)
		ExtractWedgeLayerData1<T, T2, GetValueAtDirect<T, T2> >(element, output, indices, indexCount, polygonSizes, polygonCount, vertexCount, layerName, meshName, defaultValue);
	else if (referenceMode == FbxLayerElement::eIndexToDirect)
		ExtractWedgeLayerData1<T, T2, GetValueAtIndex<T, T2> >(element, output, indices, indexCount, polygonSizes, polygonCount, vertexCount, layerName, meshName, defaultValue);
	else
		std::cout<< "Unsupported wedge reference mode type. Please report this bug." << std::endl; 
}

template<typename T>
static void InvertFaceWindings(typename std::vector<T>& x, const std::vector<uint32_t>& polygonSizes)
{
	if (x.empty())
		return;
	const size_t faceCount = polygonSizes.size();
	for (size_t i = 0, idx = 0; i < faceCount; ++i)
	{
		int ps = polygonSizes[i];

		for (int v = 0; v < (ps / 2); v++)
			std::swap(x[idx + v], x[idx + (ps - 1) - v]);

		// try to match more closely to the unit tests
		T last_entry = x[idx + (ps - 1)];
		for (int v = ps - 1; v >= 0; v--)
			x[idx + v] = v > 0 ? x[idx + v - 1] : last_entry;

		idx += ps;
	}
}


typedef struct SMikkTSpaceContext SMikkTSpaceContext;

typedef struct
{
	// Returns the number of faces (triangles/quads) on the mesh to be processed.
	int (*m_getNumFaces)(const SMikkTSpaceContext* pContext);

	// Returns the number of vertices on face number iFace
	// iFace is a number in the range {0, 1, ..., getNumFaces()-1}
	int (*m_getNumVerticesOfFace)(const SMikkTSpaceContext* pContext, const int iFace);

	// returns the position/normal/texcoord of the referenced face of vertex number iVert.
	// iVert is in the range {0,1,2} for triangles and {0,1,2,3} for quads.
	void (*m_getPosition)(const SMikkTSpaceContext* pContext, float fvPosOut[], const int iFace, const int iVert);
	void (*m_getNormal)(const SMikkTSpaceContext* pContext, float fvNormOut[], const int iFace, const int iVert);
	void (*m_getTexCoord)(const SMikkTSpaceContext* pContext, float fvTexcOut[], const int iFace, const int iVert);

	// either (or both) of the two setTSpace callbacks can be set.
	// The call-back m_setTSpaceBasic() is sufficient for basic normal mapping.

	// This function is used to return the tangent and fSign to the application.
	// fvTangent is a unit length vector.
	// For normal maps it is sufficient to use the following simplified version of the bitangent which is generated at pixel/vertex level.
	// bitangent = fSign * cross(vN, tangent);
	// Note that the results are returned unindexed. It is possible to generate a new index list
	// But averaging/overwriting tangent spaces by using an already existing index list WILL produce INCRORRECT results.
	// DO NOT! use an already existing index list.
	void (*m_setTSpaceBasic)(const SMikkTSpaceContext* pContext, const float fvTangent[], const float fSign, const int iFace, const int iVert);

	// This function is used to return tangent space results to the application.
	// fvTangent and fvBiTangent are unit length vectors and fMagS and fMagT are their
	// true magnitudes which can be used for relief mapping effects.
	// fvBiTangent is the "real" bitangent and thus may not be perpendicular to fvTangent.
	// However, both are perpendicular to the vertex normal.
	// For normal maps it is sufficient to use the following simplified version of the bitangent which is generated at pixel/vertex level.
	// fSign = bIsOrientationPreserving ? 1.0f : (-1.0f);
	// bitangent = fSign * cross(vN, tangent);
	// Note that the results are returned unindexed. It is possible to generate a new index list
	// But averaging/overwriting tangent spaces by using an already existing index list WILL produce INCRORRECT results.
	// DO NOT! use an already existing index list.
	void (*m_setTSpace)(const SMikkTSpaceContext* pContext, const float fvTangent[], const float fvBiTangent[], const float fMagS, const float fMagT,
		const bool bIsOrientationPreserving, const int iFace, const int iVert);
} SMikkTSpaceInterface;

struct SMikkTSpaceContext
{
	SMikkTSpaceInterface* m_pInterface;	// initialized with callback functions
	void* m_pUserData;						// pointer to client side mesh data etc. (passed as the first parameter with every interface call)
};

typedef struct
{
	int iNrFaces;
	int* pTriMembers;
} SSubGroup;

typedef struct
{
	int iNrFaces;
	int* pFaceIndices;
	int iVertexRepresentitive;
	bool bOrientPreservering;
} SGroup;

typedef struct
{
	int FaceNeighbors[3];
	SGroup* AssignedGroup[3];

	// normalized first order face derivatives
	Vector3f vOs, vOt;
	float fMagS, fMagT;	// original magnitudes

	// determines if the current and the next triangle are a quad.
	int iOrgFaceNumber;
	int iFlag, iTSpacesOffs;
	unsigned char vert_num[4];
} STriInfo;

typedef struct
{
	Vector3f vOs;
	float fMagS;
	Vector3f vOt;
	float fMagT;
	int iCounter;	// this is to average back into quads.
	bool bOrient;
} STSpace;

static bool	veq(const Vector3f v1, const Vector3f v2)
{
	return (v1.x == v2.x) && (v1.y == v2.y) && (v1.z == v2.z);
}

static int MakeIndex(const int iFace, const int iVert)
{
	return (iFace << 2) | (iVert & 0x3);
}

static Vector3f		vsub(const Vector3f v1, const Vector3f v2)
{
	Vector3f vRes;

	vRes.x = v1.x - v2.x;
	vRes.y = v1.y - v2.y;
	vRes.z = v1.z - v2.z;

	return vRes;
}

typedef struct
{
	float vert[3];
	int index;
} STmpVert;


static float			LengthSquared(const Vector3f v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

static Vector3f		vscale(const float fS, const Vector3f v)
{
	Vector3f vRes;

	vRes.x = fS * v.x;
	vRes.y = fS * v.y;
	vRes.z = fS * v.z;

	return vRes;
}

static Vector3f		vadd(const Vector3f v1, const Vector3f v2)
{
	Vector3f vRes;

	vRes.x = v1.x + v2.x;
	vRes.y = v1.y + v2.y;
	vRes.z = v1.z + v2.z;

	return vRes;
}

static float		vdot(const Vector3f v1, const Vector3f v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}


static bool NotZero(const float fX)
{
	// could possibly use FLT_EPSILON instead
	return fabsf(fX) > FLT_MIN;
}

static float			Length(const Vector3f v)
{
	return sqrtf(LengthSquared(v));
}

static bool VNotZero(const Vector3f v)
{
	// might change this to an epsilon based test
	return NotZero(v.x) || NotZero(v.y) || NotZero(v.z);
}

static Vector3f		Normalize(const Vector3f v)
{
	return vscale(1 / Length(v), v);
}

typedef union
{
	struct
	{
		int i0, i1, f;
	};
	int array[3];
} SEdge;

inline float SqrMagnitude(const Vector2f& inV) { return Dot(inV, inV); }

inline bool CompareApproximately(const Vector2f& inV0, const Vector2f& inV1, float inMaxDist)
{
	return SqrMagnitude(inV1 - inV0) <= inMaxDist * inMaxDist;
}
