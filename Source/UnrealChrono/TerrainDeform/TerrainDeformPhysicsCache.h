#pragma once

#include "CoreMinimal.h"

/** A CPU copy of the fine deformation layer around the vehicle, as read back from the GPU. */
struct FTerrainDeformPhysicsSnapshot
{
	/** Absolute fine-layer texel coordinate of Data[0]. */
	FIntPoint OriginTexel = FIntPoint::ZeroValue;
	/** Texels per side (square). */
	int32 Size = 0;
	float TexelSizeCm = 1.f;
	/** Height deltas in 0.1 mm units, row-major, Data[Y * Size + X]. */
	TArray<int32> Data;
};

/**
 * Read-only view of the deformation for the physics side. The render thread publishes a fresh snapshot every frame
 * (a few frames behind the GPU); any thread can sample it. Heights are relative to the original ground surface.
 */
class UNREALCHRONO_API FTerrainDeformPhysicsCache
{
public:
	/**
	 * Thread-safe. Returns false outside the cached region (or when nothing has been published), in which case the
	 * outputs are zero.
	 * @param OutHeightCm  deformation height in cm (negative = rut, positive = berm).
	 * @param OutSlope     dHeight/dX and dHeight/dY (cm per cm), for tilting the surface normal.
	 */
	static bool Sample(const FVector2D& WorldXYCm, float& OutHeightCm, FVector2D& OutSlope);

	/**
	 * The part of a sampled deformation the physics should treat as ground. Berms (positive) are solid ground the
	 * tire must climb. Ruts (negative) are NOT fed back by default: a rut is the result of the sinkage the soil model
	 * already applies, and lowering the ground by it again removes the tire's penetration, so the load support and
	 * traction vanish and the car digs itself in. TerrainDeform.PhysicsRutGain (0..1) scales how much of a rut to
	 * feed back. Both the terrain query and the stamp's sinkage computation must use this same function.
	 */
	static float ToPhysicsHeightCm(float DeformCm);

	/** Render thread. */
	static void Publish(TSharedPtr<const FTerrainDeformPhysicsSnapshot, ESPMode::ThreadSafe> Snapshot);
	static void Clear();
};
