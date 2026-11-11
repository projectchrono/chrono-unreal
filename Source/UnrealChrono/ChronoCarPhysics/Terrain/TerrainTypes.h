#pragma once

#include "CoreMinimal.h"
#include "ChronoTypes.h"
#include "VehicleCommon.h"
#include <vector>

// Plain data shared by the NextGenCars terrain implementations (ChaosTerrain / ChronoTerrain) and the vehicle world.
// Everything here is physics-thread safe: no UObject access, pointer identities only.

struct TireKey
{
public:
	TireKey() {}
	TireKey(uintptr_t KeyInit) : Tire(KeyInit) {}
	uintptr_t Tire = 0;
};

FORCEINLINE bool operator==(const TireKey& A, const TireKey& B)
{
	return A.Tire == B.Tire;
}

FORCEINLINE uint32 GetTypeHash(const TireKey& Key)
{
	return ::GetTypeHash(Key.Tire); 
}

struct TireTerrainData
{
public:
	ChVector3<> Normal;
	ChVector3<> ImpactPoint;
	chrono::ChCoordsys<> TireFrame;
	float Height = 0;
	float Friction = 0.7;
	double Depth = 0;
	bool Hit = false;
	bool InContact = false;
	chrono::vehicle::WheelState State;
	double DT;
	ETerrainType Terrain = ETerrainType::Asphalt;	
};

//Identity of an Unreal physical material as seen from the physics thread. Captured on the game thread at init
//(ACarPhysicsWorld::MakeVehicleSetup): MaterialUserData is &UPhysicalMaterial::UserData, the pointer Chaos hands
//back in FChaosPhysicsMaterial::UserData / FChronoChaosTerrainHit::MaterialUserData. Never dereferenced.
struct FTerrainMaterialEntry
{
	const void* MaterialUserData = nullptr;
	ETerrainType TerrainType = ETerrainType::Asphalt;
	float Friction = 0.7f;
	float RollingFriction = 0.001f;
	float SpinningFriction = 0.001f;
};

//Result of a terrain query below / around a point (chrono frame, metres)
struct FTerrainQueryResult
{
	bool bHit = false;
	ChVector3d Point = ChVector3d(0, 0, 0);
	ChVector3d Normal = ChVector3d(0, 0, 1);
	float Friction = 0.7f;
	ETerrainType TerrainType = ETerrainType::Asphalt;
};

//Common physics-thread interface of the NextGenCars terrains on top of chrono::vehicle::ChTerrain: a full query
//(point / normal / friction / surface type) used by the snapshot builder for the per tire telemetry.
class INextGenTerrain
{
public:
	virtual ~INextGenTerrain() = default;

	//Terrain below 'Loc' (chrono metres). Returns false when nothing is hit within the search range
	virtual bool QueryTerrain(const ChVector3d& Loc, FTerrainQueryResult& Out) const = 0;

	//Resolves an Unreal physical material identity to a NextGenCars terrain type (Asphalt when unknown)
	static ETerrainType ResolveTerrainType(const std::vector<FTerrainMaterialEntry>& Materials, const void* MaterialUserData)
	{
		if (MaterialUserData)
		{
			for (const FTerrainMaterialEntry& Entry : Materials)
			{
				if (Entry.MaterialUserData == MaterialUserData)
				{
					return Entry.TerrainType;
				}
			}
		}
		return ETerrainType::Asphalt;
	}
};
