#pragma once

#include "CoreMinimal.h"

/** One footprint stamp, e.g. a tire contact patch. Units are cm / world space. */
struct FTerrainDeformStamp
{
	/** World-space XY centre of the footprint. */
	FVector2f Center = FVector2f::ZeroVector;
	/** Unit direction of the footprint's long axis (wheel rolling direction) in world XY. */
	FVector2f Forward = FVector2f(1.f, 0.f);
	/** Half extents along Forward / across it. */
	float HalfLength = 20.f;
	float HalfWidth = 15.f;
	/** Absolute depth below the original surface at the footprint centre (positive = down). */
	float Sinkage = 0.f;
	/** Half length of the straight path swept along Forward, so one stamp covers last-frame to this-frame travel. */
	float SegmentHalf = 0.f;
	/** Distance the source had travelled when the swept segment started; anchors the tread pattern to the path. */
	float PathStart = 0.f;
};

/** One toroidal height-delta window. */
struct FTerrainDeformLayerConfig
{
	/** Texels per side. Must be a multiple of 8. */
	int32 WindowSize = 2048;
	/** World size of one texel in cm. */
	float TexelSize = 1.f;
	/** Stamp tile size in texels (covers ~2 m). Multiple of 8. */
	int32 StampTileSize = 200;
	/** Whether tread lugs are pressed into this layer. */
	bool bTread = false;
};

/**
 * Two layers: a coarse window for long memory (~200 m) and a fine window around the vehicle for tread detail.
 * Away from the vehicle the track falls back to the coarse shape; the fine window is seeded from the coarse
 * layer whenever it scrolls onto old tracks, so nothing disappears.
 */
struct FTerrainDeformSettings
{
	static constexpr int32 NumLayers = 2;
	static constexpr int32 CoarseLayer = 0;
	static constexpr int32 FineLayer = 1;

	FTerrainDeformLayerConfig Layers[NumLayers];

	/** Fraction of displaced volume that is compacted away. 0 keeps volume exactly. */
	float Compression = 0.4f;
	/** 1 = berm material goes to the nearest point of the footprint contour (can pile up ahead); 0 = straight out to the sides. */
	float ForwardBias = 0.f;
	/** Angle of repose for the slope relaxation pass. */
	float ReposeAngleDeg = 35.f;
	/** Relaxation passes per stamp per frame. 0 disables. */
	int32 RelaxIterations = 8;
	/** How far tread lugs press below the track floor, in cm. 0 disables tread. */
	float TreadDepthCm = 2.f;
	/** Physical size of the baked tread texture: width across the tire and length of one repeat along the path (cm). */
	float TreadWidthCm = 24.11f;
	float TreadRepeatCm = 83.6f;

	FTerrainDeformSettings()
	{
		// Coarse: 4096 x 5 cm = ~205 m of memory.
		Layers[CoarseLayer].WindowSize = 4096;
		Layers[CoarseLayer].TexelSize = 5.f;
		Layers[CoarseLayer].StampTileSize = 40;
		Layers[CoarseLayer].bTread = false;
		// Fine: 4096 x 0.75 cm = ~30.7 m around the vehicle, with tread. Full detail extends to 40% of the window
		// width from the vehicle (~12 m / 40 ft) and fades out by 45% (~14 m). 0.75 cm keeps a 2.8 cm lug about
		// 4 texels long. Memory: 3 textures x 64 MB (delta R32, height R32F, normal RGBA8).
		Layers[FineLayer].WindowSize = 4096;
		Layers[FineLayer].TexelSize = 0.75f;
		Layers[FineLayer].StampTileSize = 272;
		Layers[FineLayer].bTread = true;
	}
};
