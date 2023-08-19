#pragma once
enum ConstraintKind
{
	kConstraintKindUndefined,
	kConstraintKindPosition,
	kConstraintKindRotation,
	kConstraintKindScale,
	kConstraintKindParent,
	kConstraintKindSingleChainIK,
	kConstraintKindAim,
	kConstraintKindCharacter,

	kConstraintKindCount
};

enum Axis
{
	kAxisNone = 0,
	kAxisX = 1,
	kAxisY = 2,
	kAxisZ = 4
};

enum WorldUpType
{
	kWorldUpTypeSceneUp,
	kWorldUpTypeObjectUp,
	kWorldUpTypeObjectRotationUp,
	kWorldUpTypeVector,
	kWorldUpTypeNone,

	kWorldUpTypeCount
};


enum ActivationType
{
	kActivationNone,
	kActivationPreserveOffset,
	kActivationZeroOffset,
	kUserUpdateOffset,
};
