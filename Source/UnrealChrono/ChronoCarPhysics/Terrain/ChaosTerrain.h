#pragma once

#include "ChronoTypes.h"
#include "TerrainTypes.h"
#include <vector>

//Terrain of the Chaos collision backend.
//
//The vehicle's tires (TMeasy, Pacejka, ...) sense the ground through ChTerrain::GetHeight / GetNormal /
//GetCoefficientFriction. On the Chaos backend the ground IS the Unreal level, so every query is a physics-thread ray
//cast through the ChronoCore plugin (FChronoChaosCollision::RaycastTerrain) against the same collidable Unreal shapes
//Chrono collides with: landscape, static meshes, rigid bodies. Nothing is tagged, exported or streamed.
//
//Friction comes from the Unreal physical material of the hit shape; the surface type is resolved by comparing the
//material identity with the UVehiclePhysicalMaterial table captured on the game thread (no UObject access here).
//Rigid (contact based) tires do not need any of this: their collision shapes collide with the level directly.
class ChaosTerrain : public chrono::vehicle::ChTerrain, public INextGenTerrain
{
public:
	ChaosTerrain(chrono::ChSystem* System, std::vector<FTerrainMaterialEntry> MaterialsInit);
	virtual ~ChaosTerrain() override {}

	// chrono::vehicle::ChTerrain
	virtual void Synchronize(double time) override {}
	virtual void Advance(double step) override {}
	virtual double GetHeight(const ChVector3d& loc) const override;
	virtual ChVector3d GetPoint(const ChVector3d& loc) const override;
	virtual ChVector3d GetNormal(const ChVector3d& loc) const override;
	virtual float GetCoefficientFriction(const ChVector3d& loc) const override;
	virtual void GetProperties(const ChVector3d& loc, ChVector3d& point, double& height, ChVector3d& normal, float& friction) const override;

	//NextGenCars extensions (per tire queries, same ray cast)
	virtual double GetHeight(const ChVector3d& loc, const chrono::vehicle::WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const override;
	virtual ChVector3d GetNormal(const ChVector3d& loc, const chrono::vehicle::WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const override;
	virtual float GetCoefficientFriction(const ChVector3d& loc, const chrono::vehicle::WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const override;
	virtual void GetProperties(const ChVector3d& loc, double& height, ChVector3d& normal, float& friction, const chrono::vehicle::WheelState& wheelState, const double TireRadius, uintptr_t TirePtr) const override;

	// INextGenTerrain
	virtual bool QueryTerrain(const ChVector3d& Loc, FTerrainQueryResult& Out) const override;

	//Height reported when the ray hits nothing (vehicle over the void): far below so the tires lose contact
	static constexpr double OffMapHeight = -1000.0;

	//Ray extents around the query point (metres)
	double RayUpMetres = 2.0;
	double RayDownMetres = 6.0;

private:
	chrono::ChSystem* ChronoSystem = nullptr;
	std::vector<FTerrainMaterialEntry> Materials;
};
