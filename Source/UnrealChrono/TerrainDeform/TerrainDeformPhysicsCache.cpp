#include "TerrainDeformPhysicsCache.h"

namespace
{
	FCriticalSection GCacheLock;
	TSharedPtr<const FTerrainDeformPhysicsSnapshot, ESPMode::ThreadSafe> GSnapshot;

	// 0.01 mm per unit -> cm (DELTA_PER_CM in TerrainDeform.usf).
	constexpr float UnitsToCm = 0.001f;
}

static TAutoConsoleVariable<float> CVarPhysicsRutGain(TEXT("TerrainDeform.PhysicsRutGain"), 0.f,
	TEXT("Fraction (0..1) of a rut's depth that tire physics sees as lowered ground. 0 = only berms feed back (stable). "
		"Values near 1 make the ground follow the tire down and the car digs itself in."), ECVF_Default);

float FTerrainDeformPhysicsCache::ToPhysicsHeightCm(float DeformCm)
{
	if (DeformCm >= 0.f)
	{
		return DeformCm;
	}
	return DeformCm * FMath::Clamp(CVarPhysicsRutGain.GetValueOnAnyThread(), 0.f, 1.f);
}

void FTerrainDeformPhysicsCache::Publish(TSharedPtr<const FTerrainDeformPhysicsSnapshot, ESPMode::ThreadSafe> Snapshot)
{
	FScopeLock Lock(&GCacheLock);
	GSnapshot = MoveTemp(Snapshot);
}

void FTerrainDeformPhysicsCache::Clear()
{
	FScopeLock Lock(&GCacheLock);
	GSnapshot.Reset();
}

bool FTerrainDeformPhysicsCache::Sample(const FVector2D& WorldXYCm, float& OutHeightCm, FVector2D& OutSlope)
{
	OutHeightCm = 0.f;
	OutSlope = FVector2D::ZeroVector;

	TSharedPtr<const FTerrainDeformPhysicsSnapshot, ESPMode::ThreadSafe> Snap;
	{
		FScopeLock Lock(&GCacheLock);
		Snap = GSnapshot;
	}
	if (!Snap.IsValid() || Snap->Size < 2)
	{
		return false;
	}

	// Texel centres sit at (index + 0.5) * texel size.
	const float Inv = 1.f / Snap->TexelSizeCm;
	const float Fx = (float)WorldXYCm.X * Inv - 0.5f - (float)Snap->OriginTexel.X;
	const float Fy = (float)WorldXYCm.Y * Inv - 0.5f - (float)Snap->OriginTexel.Y;
	const int32 X0 = FMath::FloorToInt(Fx);
	const int32 Y0 = FMath::FloorToInt(Fy);
	if (X0 < 0 || Y0 < 0 || X0 >= Snap->Size - 1 || Y0 >= Snap->Size - 1)
	{
		return false;
	}

	const float Tx = Fx - (float)X0;
	const float Ty = Fy - (float)Y0;
	const int32* Row0 = Snap->Data.GetData() + (int64)Y0 * Snap->Size + X0;
	const int32* Row1 = Row0 + Snap->Size;
	const float H00 = Row0[0] * UnitsToCm;
	const float H10 = Row0[1] * UnitsToCm;
	const float H01 = Row1[0] * UnitsToCm;
	const float H11 = Row1[1] * UnitsToCm;

	OutHeightCm = FMath::Lerp(FMath::Lerp(H00, H10, Tx), FMath::Lerp(H01, H11, Tx), Ty);
	OutSlope.X = FMath::Lerp(H10 - H00, H11 - H01, Ty) * Inv;
	OutSlope.Y = FMath::Lerp(H01 - H00, H11 - H10, Tx) * Inv;
	return true;
}
