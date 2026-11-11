#include "ChaosTerrain.h"

#include "ChronoChaosCollision.h"
#include "TerrainDeformPhysicsCache.h"
#include "Tools/Conversions.h"

using namespace chrono;
using namespace chrono::vehicle;

ChaosTerrain::ChaosTerrain(ChSystem* System, std::vector<FTerrainMaterialEntry> MaterialsInit)
	: ChronoSystem(System), Materials(std::move(MaterialsInit))
{
}

bool ChaosTerrain::QueryTerrain(const ChVector3d& Loc, FTerrainQueryResult& Out) const
{
	Out = FTerrainQueryResult();
	if (!ChronoSystem)
	{
		return false;
	}

	//Vertical ray through the query point: from RayUpMetres above (so a slightly buried point still finds the surface
	//it is in) down to RayDownMetres below. Only the Unreal level is tested, never the Chrono proxies (the vehicle
	//must not sense its own chassis).
	const ChVector3d Vertical = ChWorldFrame::Vertical();
	const FVector Start = Conversions::ToUnrealPos(Loc + Vertical * RayUpMetres);
	const FVector End = Conversions::ToUnrealPos(Loc - Vertical * RayDownMetres);

	FChronoChaosTerrainHit Hit;
	FChronoChaosTerrainQueryParams Params;
	Params.bIncludeChronoBodies = false;
	if (!FChronoChaosCollision::RaycastTerrain(*ChronoSystem, Start, End, Hit, Params) || !Hit.bHit)
	{
		return false;
	}

	Out.Friction = Hit.Friction;
	Out.TerrainType = INextGenTerrain::ResolveTerrainType(Materials, Hit.MaterialUserData);

	//Deformable ground (TerrainDeform plugin): the ray hits the original collision surface, so add the ruts and berms
	//the vehicles have pressed into it. Only soft surfaces deform; a road or bridge mesh above a rutted landscape must
	//not inherit its ruts. The cache is a few frames old and zero where nothing was deformed.
	FVector HitPoint = Hit.Point;
	FVector HitNormal = Hit.Normal;
	switch (Out.TerrainType)
	{
	case ETerrainType::Sand:
	case ETerrainType::Dirt:
	case ETerrainType::Mud:
	case ETerrainType::Snow:
	case ETerrainType::Grass:
	case ETerrainType::Gravel:
	{
		float DeformCm = 0.f;
		FVector2D Slope = FVector2D::ZeroVector;
		if (FTerrainDeformPhysicsCache::Sample(FVector2D(HitPoint.X, HitPoint.Y), DeformCm, Slope))
		{
			// Berms are ground; ruts are not fed back (see ToPhysicsHeightCm) or the car digs itself in.
			HitPoint.Z += FTerrainDeformPhysicsCache::ToPhysicsHeightCm(DeformCm);
			if (DeformCm > 0.f)
			{
				// Tilt the normal by the berm's local slope (flat deformation leaves it untouched).
				HitNormal = (HitNormal - FVector(Slope.X, Slope.Y, 0.0)).GetSafeNormal();
			}
		}
		break;
	}
	default:
		break;
	}

	Out.bHit = true;
	Out.Point = Conversions::ToChronoVec(HitPoint);
	Out.Normal = Conversions::ToChronoNormal(HitNormal);
	Out.Normal.Normalize();
	//Terrain queries are about the driving surface: a downward facing hit (ceiling / inside of a shape) is flipped
	if (Vdot(Out.Normal, Vertical) < 0)
	{
		Out.Normal = -Out.Normal;
	}
	return true;
}

double ChaosTerrain::GetHeight(const ChVector3d& loc) const
{
	FTerrainQueryResult Result;
	return QueryTerrain(loc, Result) ? ChWorldFrame::Height(Result.Point) : OffMapHeight;
}

ChVector3d ChaosTerrain::GetPoint(const ChVector3d& loc) const
{
	FTerrainQueryResult Result;
	if (QueryTerrain(loc, Result))
	{
		return Result.Point;
	}
	ChVector3d Point = loc;
	ChWorldFrame::Project(Point);
	return Point + ChWorldFrame::Vertical() * OffMapHeight;
}

ChVector3d ChaosTerrain::GetNormal(const ChVector3d& loc) const
{
	FTerrainQueryResult Result;
	return QueryTerrain(loc, Result) ? Result.Normal : ChWorldFrame::Vertical();
}

float ChaosTerrain::GetCoefficientFriction(const ChVector3d& loc) const
{
	FTerrainQueryResult Result;
	return QueryTerrain(loc, Result) ? Result.Friction : 0.8f;
}

void ChaosTerrain::GetProperties(const ChVector3d& loc, ChVector3d& point, double& height, ChVector3d& normal, float& friction) const
{
	//One ray for everything (the default ChTerrain implementation would cast three)
	FTerrainQueryResult Result;
	if (QueryTerrain(loc, Result))
	{
		point = Result.Point;
		height = ChWorldFrame::Height(Result.Point);
		normal = Result.Normal;
		friction = Result.Friction;
		return;
	}

	point = GetPoint(loc);
	height = OffMapHeight;
	normal = ChWorldFrame::Vertical();
	friction = 0.8f;
}

double ChaosTerrain::GetHeight(const ChVector3d& loc, const WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const
{
	return GetHeight(loc);
}

ChVector3d ChaosTerrain::GetNormal(const ChVector3d& loc, const WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const
{
	return GetNormal(loc);
}

float ChaosTerrain::GetCoefficientFriction(const ChVector3d& loc, const WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const
{
	return GetCoefficientFriction(loc);
}

void ChaosTerrain::GetProperties(const ChVector3d& loc, double& height, ChVector3d& normal, float& friction, const WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const
{
	ChVector3d Point;
	GetProperties(loc, Point, height, normal, friction);
}
