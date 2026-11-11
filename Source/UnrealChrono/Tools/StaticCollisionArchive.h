#pragma once

#include "CoreMinimal.h"
#include "Misc/Paths.h"

//Binary archive format used to cache the static (ChronoStatic tagged) collision geometry of a level.
//The editor utility (UNextGenCarsUtilities) writes the archive, ACarPhysicsWorld reads it back at runtime
//so we don't have to walk the whole level and re-cook every body setup on every launch.
namespace StaticCollisionArchive
{
	//'NGCC' - NextGenCars Collision Cache
	static const uint32 ArchiveMagic = 0x4E474343u;
	//v3: drop ActorPosition (bucket by mesh world Position); add TriangleMesh landscape patches
	static const uint32 ArchiveVersion = 3;

	//Same location the rest of the chrono map data lives in (see ACarPhysicsWorld::Init)
	static inline FString GetArchiveDirectory(const FString& MapDirectory)
	{
		FString Directory = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() + TEXT("Binaries/Win64/data/vehicle/MapData/"));
		if (!MapDirectory.IsEmpty())
		{
			Directory += MapDirectory;
			if (!Directory.EndsWith(TEXT("/")))
			{
				Directory += TEXT("/");
			}
		}
		return Directory;
	}

	static inline FString GetArchiveFilePath(const FString& MapDirectory, const FString& StaticCollisionArchiveTag)
	{
		return GetArchiveDirectory(MapDirectory) + StaticCollisionArchiveTag;
	}
}

//Mirrors PhysicsDataTypes::ShapeType, but kept separate so the archive layout is stable
enum class EStaticCollisionShapeKind : uint8
{
	Box,
	Sphere,
	Capsule,
	Convex,
	TriangleMesh
};

//Plain (chrono free) mirror of ChColliderInfo. Everything is already stored in chrono space and chrono units
struct FStaticColliderRecord
{
	float Mass = 0.f;
	float Restitution = 0.f;
	float Friction = 0.f;
	float KineticFriction = 0.f;
	float Density = 0.f;
	float YoungsModulus = 0.f;
	float PoissonRatio = 0.f;
	float RollingFriction = 0.f;
	float SpinningFriction = 0.f;
	bool StaticObject = true;

	//World-space mesh/shape transform (chrono space). Also used to bucket into the streaming grid
	double PositionX = 0.0;
	double PositionY = 0.0;
	double PositionZ = 0.0;

	//Chrono quaternion components (e0 = w)
	double RotationE0 = 1.0;
	double RotationE1 = 0.0;
	double RotationE2 = 0.0;
	double RotationE3 = 0.0;

	friend FArchive& operator<<(FArchive& Ar, FStaticColliderRecord& Record)
	{
		Ar << Record.Mass;
		Ar << Record.Restitution;
		Ar << Record.Friction;
		Ar << Record.KineticFriction;
		Ar << Record.Density;
		Ar << Record.YoungsModulus;
		Ar << Record.PoissonRatio;
		Ar << Record.RollingFriction;
		Ar << Record.SpinningFriction;
		Ar << Record.StaticObject;
		Ar << Record.PositionX;
		Ar << Record.PositionY;
		Ar << Record.PositionZ;
		Ar << Record.RotationE0;
		Ar << Record.RotationE1;
		Ar << Record.RotationE2;
		Ar << Record.RotationE3;
		return Ar;
	}
};

struct FStaticCollisionShapeRecord
{
	uint8 ShapeKind = (uint8)EStaticCollisionShapeKind::Box;
	//PhysicsDataTypes::MeshType
	uint8 MeshKind = 0;
	//PhysicsDataTypes::CollisionType
	uint8 CollisionKind = 2;

	FStaticColliderRecord Collider;

	//Box: half extents. Sphere: radius in X. Capsule: radius in X, half length in Y
	double ExtentX = 0.0;
	double ExtentY = 0.0;
	double ExtentZ = 0.0;

	//Convex hull points, already in chrono space/units (body-local)
	TArray<FVector> HullPoints;

	//Triangle mesh (landscape patches etc.), already in chrono space/units (body-local)
	TArray<FVector> MeshVertices;
	//Packed triangle indices (i0,i1,i2) repeated
	TArray<int32> MeshIndices;

	FBox AABB = FBox(ForceInit);

	//Purely informational, handy when debugging a cache
	FString SourceName;

	friend FArchive& operator<<(FArchive& Ar, FStaticCollisionShapeRecord& Record)
	{
		Ar << Record.ShapeKind;
		Ar << Record.MeshKind;
		Ar << Record.CollisionKind;
		Ar << Record.Collider;
		Ar << Record.ExtentX;
		Ar << Record.ExtentY;
		Ar << Record.ExtentZ;
		Ar << Record.HullPoints;
		Ar << Record.MeshVertices;
		Ar << Record.MeshIndices;
		Ar << Record.AABB;
		Ar << Record.SourceName;
		return Ar;
	}
};

struct FStaticCollisionArchiveData
{
	uint32 Magic = StaticCollisionArchive::ArchiveMagic;
	uint32 Version = StaticCollisionArchive::ArchiveVersion;
	FString LevelName;
	TArray<FStaticCollisionShapeRecord> Shapes;

	friend FArchive& operator<<(FArchive& Ar, FStaticCollisionArchiveData& Data)
	{
		Ar << Data.Magic;
		Ar << Data.Version;
		Ar << Data.LevelName;
		Ar << Data.Shapes;
		return Ar;
	}
};
