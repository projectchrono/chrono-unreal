#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "StaticCollisionArchive.h"
#include <memory>
#include <vector>

class AActor;
class UWorld;
class UBodySetup;
class UStaticMesh;
class UPrimitiveComponent;
class USceneComponent;
class UShapeComponent;
class UStaticMeshComponent;
class USkeletalMeshComponent;
class UInstancedStaticMeshComponent;
class ULandscapeHeightfieldCollisionComponent;
class UVehiclePhysicalMaterial;
class APhysicsVehicle;
enum class UETerrainResolution : uint8;

using namespace PhysicsDataTypes;

//Everything the extractor reads from ACarPhysicsWorld, captured by value so the extractor can be built on demand
//(also from the CallInEditor SerializeCollisionActors path that runs without Init())
struct FUnrealCollisionExtractorSettings
{
	UWorld* World = nullptr;
	//Name of the owning actor, only used in log messages
	FString OwnerName;
	bool EnableChronoVisual = true;
	UETerrainResolution DesiredTerrainResolution = (UETerrainResolution)0;
	FString MapDirectory;
	FString StaticCollisionArchiveTag;
	//Collision streaming tuning (see the ACarPhysicsWorld properties of the same name)
	bool EnableCollisionStreaming = true;
	float CollisionCellSizeMetres = 64.f;
	float CollisionActivationRadiusMetres = 250.f;
	float CollisionCellHysteresisMetres = 50.f;
	//Start location of the streaming evaluation (InitializeCollisionStreaming)
	APhysicsVehicle* PhysicsVehicle = nullptr;
};

//Bullet backend only: game thread helper that walks the Unreal level (tagged actors / components, landscape
//patches, the static collision archive) and turns the geometry into detached Chrono bodies
//(FBulletStaticCollision::Create*), collected as FCollisionModel entries in the CollisionData vector passed
//to the constructor. Also buckets the gathered colliders into the FStaticCollisionGrid.
class FUnrealCollisionExtractor
{
public:
	FUnrealCollisionExtractor(const FUnrealCollisionExtractorSettings& InSettings, std::vector<std::shared_ptr<FCollisionModel>>& InCollisionData);

	const FUnrealCollisionExtractorSettings& GetSettings() const { return Settings; }
	const std::vector<std::shared_ptr<FCollisionModel>>& GetCollisionData() const { return CollisionData; }

	void GatherUnrealCollision(bool bSkipStaticTag = false);
	//Builds the streaming grid from the gathered colliders and applies the cells that should be active at start
	void InitializeCollisionStreaming();

	//Returns the full path of the binary static collision cache for this level. The static overload only needs the
	//settings, so a const owner can resolve the path without a CollisionData reference
	static FString GetStaticCollisionArchivePath(const FUnrealCollisionExtractorSettings& InSettings);
	FString GetStaticCollisionArchivePath() const { return GetStaticCollisionArchivePath(Settings); }
	//Looks for the StaticCollisionArchiveTag file and, if present, rebuilds the static collision from it
	//instead of walking the level again. Returns true when the cache was used
	bool LoadSerializedCollision();

#if WITH_EDITOR
	//Gathers the chrono/bullet collision of every selected actor (recursively including attached children)
	//whose static mesh components are tagged 'ChronoStatic' and writes it to the static collision archive
	void SerializeCollisionActors();
#endif //WITH_EDITOR

	void ExtractPhysicsGeometry(AActor* Actor, CollisionType Collision, const FName& Tag);
	void ExtractPhysicsGeometry(USkeletalMeshComponent* SKMeshComp, CollisionType Collision);
	void ExtractPhysicsGeometry(UStaticMeshComponent* StaticMeshComp, CollisionType Collision);
	void ExtractPhysicsGeometry(UInstancedStaticMeshComponent* InstancedStaticMeshComp, CollisionType Collision);
	void ExtractPhysicsGeometry(UShapeComponent* ShapeComp, CollisionType Collision);
	void ExtractPhysicsGeometry(UPrimitiveComponent * Component, CollisionType Collision, UBodySetup* BodySetup, UVehiclePhysicalMaterial * PhysMaterial, const float Mass, MeshType Mesh);
	void ExtractPhysicsGeometry(UPrimitiveComponent * Component, const FTransform& ComponentTx, CollisionType Collision, UBodySetup* BodySetup, UVehiclePhysicalMaterial * PhysMaterial, const float Mass, MeshType Mesh);
	void ExtractLandscapeCollision(ULandscapeHeightfieldCollisionComponent* CollisionComp, CollisionType Collision);
	bool BuildLandscapePatchTriangleMesh(ULandscapeHeightfieldCollisionComponent* CollisionComp, TArray<FVector>& OutVertices, TArray<int32>& OutIndices, FBox& OutAABB, ChronoTransform& OutChTransform) const;
	bool BuildStaticMeshTriangleMesh(UStaticMesh* StaticMesh, const FTransform& Transform, TArray<FVector>& OutVertices, TArray<int32>& OutIndices, FBox& OutAABB, ChronoTransform& OutChTransform) const;
	void ExtractStaticMeshTriangleCollision(UStaticMeshComponent* StaticMeshComp, CollisionType Collision, bool bIncludeSubComponents = true);
	void ExtractInstancedStaticMeshTriangleCollision(UInstancedStaticMeshComponent* InstancedStaticMeshComp, CollisionType Collision);
	void ExtractStaticMeshTriangleCollision(UStaticMesh* StaticMesh, const FTransform& Transform, CollisionType Collision, UVehiclePhysicalMaterial* PhysMaterial = nullptr);
	void ExtractTriangleMeshCollision(UStaticMeshComponent* StaticMeshComp, CollisionType Collision, bool bIncludeSubComponents = true);
	void ExtractTriangleMeshCollision(UInstancedStaticMeshComponent* InstancedStaticMeshComp, CollisionType Collision);

private:
#if WITH_EDITOR
	void GatherActorRecursive(AActor* Actor, TArray<FStaticCollisionShapeRecord>& OutRecords);
	void GatherStaticMeshComponent(UStaticMeshComponent* StaticMeshComp, TArray<FStaticCollisionShapeRecord>& OutRecords);
	void GatherLandscapeCollisionComponent(ULandscapeHeightfieldCollisionComponent* CollisionComp, TArray<FStaticCollisionShapeRecord>& OutRecords);
#endif //WITH_EDITOR

	FUnrealCollisionExtractorSettings Settings;
	//Owned by ACarPhysicsWorld (SetupBulletStaticCollision iterates it, EndPlay clears it)
	std::vector<std::shared_ptr<FCollisionModel>>& CollisionData;
};
