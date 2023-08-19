#pragma once

enum NormalOptions
{
	kNormalOptionsImport = 0,
	kNormalOptionsCalculate = 1,
	kNormalOptionsNone = 2
};

enum NormalCalculationOptions
{
	kNormalCalculationOptionsUnweighted_Legacy = 0,
	kNormalCalculationOptionsUnweighted = 1,
	kNormalCalculationOptionsAreaWeighted = 2,
	kNormalCalculationOptionsAngleWeighted = 3,
	kNormalCalculationOptionsAreaAndAngleWeighted = 4
};

enum NormalSmoothingSourceOptions
{
	kNormalSmoothingSourceOptionsPreferSmoothingGroups = 0,
	kNormalSmoothingSourceOptionsFromSmoothingGroups = 1,
	kNormalSmoothingSourceOptionsFromAngle = 2,
	kNormalSmoothingSourceOptionsNone = 3
};

enum TangentSpaceOptions
{
	kTangentSpaceOptionsImport = 0,
	kTangentSpaceOptionsCalculateLegacy = 1,
	kTangentSpaceOptionsCalculateLegacySplitAcross = 4,
	kTangentSpaceOptionsCalculateMikk = 3,
	kTangentSpaceOptionsNone = 2
};

enum ModelImporterIndexFormat
{
	kMIIndexFormatAuto = 0,
	kMIIndexFormatUInt16 = 1,
	kMIIndexFormatUInt32 = 2,
};