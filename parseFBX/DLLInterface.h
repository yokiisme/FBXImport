#pragma once
#include "Utility.h"
#include <wingdi.h>
#include <GL/glu.h>
#define TESS_FUNCTION_CALLCONV CALLBACK

#define EXPORT_DLL extern "C" __declspec(dllexport) 


enum GfxPrimitiveType
{
    kPrimitiveInvalid = -1,

    kPrimitiveTriangles = 0, kPrimitiveTypeFirst = kPrimitiveTriangles,
    kPrimitiveTriangleStrip,
    kPrimitiveQuads,
    kPrimitiveLines,
    kPrimitiveLineStrip,
    kPrimitivePoints, kPrimitiveTypeLast = kPrimitivePoints,

    kPrimitiveForce32BitInt = 0x7fffffff // force 32 bit enum size
};

typedef std::pair<int, GfxPrimitiveType> SubsetKey;
typedef std::map<SubsetKey, int> SubsetLookup;


EXPORT_DLL void ParseFBX(char* fbxpath, char* outdir);


