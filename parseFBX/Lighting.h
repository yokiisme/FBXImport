#pragma once

// Light type
enum LightType
{
	kLightSpot,
	kLightDirectional,
	kLightPoint,
	kLightRectangle,
	kLightDisc,
	kLightTypeLast = kLightDisc // keep this last
};

const int kLightTypeCount = kLightTypeLast + 1;

// How to render a light
enum LightRenderMode
{
	kLightRenderModeAuto,
	kLightRenderModeImportant,
	kLightRenderModeNotImportant,
	kLightRenderModeLast = kLightRenderModeNotImportant// keep this last!
};

const int kLightRenderModeCount = kLightRenderModeLast + 1;

enum MixedLightingMode
{
	kBakeIndirectOnly = 0,
	kBakeShadowmask = 2,
	kBakeSubtractive = 1
};

enum LightmapBakeType
{
	kLightRealtime = 1 << 2,   // Light is realtime
	kLightBaked = 1 << 1,   // light will always be fully baked
	kLightMixed = 1 << 0,   // depends on selected LightmapMixedBakeMode
};

enum LightShadowCasterMode
{
	kLightShadowCasterModeDefault,
	kLightShadowCasterModeNonLightmappedOnly,
	kLightShadowCasterModeEverything
};

struct LightmapBakeMode
{
	LightmapBakeMode() : lightmapBakeType(kLightRealtime), mixedLightingMode(kBakeShadowmask) {}
	LightmapBakeMode(LightmapBakeType _lightmapBakeType, MixedLightingMode _mixedLightingMode) : lightmapBakeType(_lightmapBakeType), mixedLightingMode(_mixedLightingMode) {}

	LightmapBakeType lightmapBakeType;  ///< enum { Realtime=4, Mixed=1, Baked=2 }
	MixedLightingMode mixedLightingMode;///< enum { IndirectOnly=0, Shadowmask=2, Subtractive=1  }

		//In case of a Mixed light baking is based on lightmapMixedBakeMode:
		//IndirectOnly
		//  Lightmaps
		//  - direct:    no
		//  - occlusion: no
		//  Light probes
		//  - direct:    no
		//  - occlusion: no
		//
		//Shadowmask
		//  Lightmaps
		//  - direct:    no
		//  - occlusion: yes
		//  Light probes
		//  - direct:    no
		//  - occlusion: yes
		//
		//Subtractive
		//  Lightmaps
		//  - direct:    yes
		//  - occlusion: no
		//  Light probes
		//  - direct:    no
		//  - occlusion: yes
		//
		//Note: indirect is baked in all the mixed modes.

		// Indirect
		bool IsIndirectPurelyBaked() const
	{
		return (lightmapBakeType != kLightRealtime);
	}

	// Direct
	bool IsDirectPurelyBaked()   const
	{
		return (lightmapBakeType == kLightBaked);
	}

	bool IsDirectUsingSubtractiveShadows() const
	{
		return (lightmapBakeType == kLightMixed) && (mixedLightingMode == kBakeSubtractive);
	}

	bool IsDirectUsingShadowmasks() const
	{
		return (lightmapBakeType == kLightMixed) && (mixedLightingMode == kBakeShadowmask);
	}

	bool IsDirectPurelyRealtime() const
	{
		return (lightmapBakeType == kLightRealtime) || ((lightmapBakeType == kLightMixed) && (mixedLightingMode == kBakeIndirectOnly));
	}

	//Helpers
	bool IsDirectUsingBakedOcclusion() const
	{
		return IsDirectUsingSubtractiveShadows() || IsDirectUsingShadowmasks();
	}

	bool IsPurelyBaked() const
	{
		return IsDirectPurelyBaked() && IsIndirectPurelyBaked();
	}

	bool IsPurelyRealtime() const
	{
		return IsDirectPurelyRealtime() && !IsIndirectPurelyBaked();
	}

	//Hash128 GetHash() const
	//{
	//	Hash128 hash;
	//	HashValue<UInt32>(lightmapBakeType, hash);
	//	HashValue<UInt32>(mixedLightingMode, hash);
	//	return hash;
	//}

	//template<class TransferFunction>
	//inline void Transfer(TransferFunction& transfer)
	//{
	//	TRANSFER_ENUM(lightmapBakeType);
	//	TRANSFER_ENUM(mixedLightingMode);
	//}
};

enum LightmapModeForRender
{
	// Light should be rendered realtime
	// (Enlighten may or may not have dynamic gi for this light, but this never impacts any renderloops in regards to lights)
	kLightmapModeRealtime = 0,

	// dynamic (non-lightmapped) renderers should receive this light & should cast shadows.
	kLightmapModeRealtimeDynamicObjectsOnly = 1,

	// The light has been baked into lightmaps or for some other reason should not be rendered.
	kLightmapModeDontRenderLight = 2
};

enum ShadowType
{
	kShadowNone = 0,
	kShadowHard,
	kShadowSoft,
	kShadowTypeCount
};

// Match LightShadowResolution enum on C# side
enum LightShadowResolution
{
	kLightShadowResolutionFromQualitySettings = -1,
	kLightShadowResolutionLow = 0,
	kLightShadowResolutionMedium = 1,
	kLightShadowResolutionHigh = 2,
	kLightShadowResolutionVeryHigh = 3,
	kLightShadowResolutionCount = 4
};

inline bool IsSoftShadow(ShadowType shadowType)
{
	return (shadowType == kShadowSoft);
}
