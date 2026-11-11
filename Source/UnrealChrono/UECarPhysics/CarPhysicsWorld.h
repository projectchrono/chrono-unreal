#pragma once

#include "CoreMinimal.h"
#include "../Tools/DataTypes.h"
#include "../Tools/StaticCollisionArchive.h"
#include "../Tools/StaticCollisionGrid.h"
#include "../Tools/VehicleSnapshot.h"
#include "../Benchmark/BackendComparisonTypes.h"
#include "PhysicsEngine/BodySetup.h"
#include "Components/ArrowComponent.h"
#include "Components/ShapeComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Conversions.h"
#include "Engine/StaticMesh.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "VehiclePhysicalMaterial.h"
#include "PhysicsVehicle.h"
#include "GameFramework/Actor.h"
#include <atomic>
#include "CarPhysicsWorld.generated.h"

class FChronoSimulation;
class FChronoVehicleWorld;
struct FChronoVehicleSetup;
struct FChronoTickInputs;
struct FUnrealCollisionExtractorSettings;

using namespace PhysicsDataTypes;

UENUM(BlueprintType)
enum class UETerrainResolution : uint8
{
	Centimetres,
	Inches,
	Decimetres,
	Feet
};

//Collision detection backend of the Chrono system (mirrors EChronoCollisionBackend of the ChronoCore plugin)
UENUM(BlueprintType)
enum class EVehicleCollisionBackend : uint8
{
	//Chrono bodies are mirrored into the Chaos scene and collide with the Unreal level directly (landscape, static
	//meshes, rigid bodies). No level geometry is exported to Chrono.
	Chaos			UMETA(DisplayName = "Chaos"),
	//Chrono's bundled Bullet backend. Only sees Chrono bodies: MapData terrain patches plus the tagged level
	//geometry gathered by GatherUnrealCollision / the static collision archive.
	Bullet			UMETA(DisplayName = "Bullet")
};

//Hosts the Chrono vehicle simulation on Unreal's physics thread through the ChronoCore plugin (FChronoSimulation).
//
//Threading: the game thread only pushes driver inputs, enqueues one-shot commands and reads immutable per step
//snapshots (FVehicleSnapshot). The Chrono step itself (FChronoVehicleWorld) runs inside the Chaos sim callback:
//  * with Project Settings > Physics > Tick Physics Async enabled (Config/DefaultEngine.ini: bTickPhysicsAsync=True,
//    AsyncFixedTimeStepSize=1/120) the physics thread runs at a fixed rate fully decoupled from the frame rate and
//    Chrono performs ChronoSubStepsPerPhysicsTick substeps per physics tick;
//  * with async physics off the callback still runs on the physics task thread but the game thread waits for it at
//    the end of the frame; Chrono then substeps at ~1/360 s capped by MaxChronoSubStepsPerTick.
//Neither thread ever blocks on the other during the simulation loop.
UCLASS()
class UNREALCHRONO_API ACarPhysicsWorld : public AActor
{
	GENERATED_BODY()
	
public:
	// Sets default values for this actor's properties
	ACarPhysicsWorld();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void UpdateVehicles();	
	
	//Debug drawing of the Bullet static colliders (editor only)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		bool EnableChronoVisual = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		APhysicsVehicle * PhysicsVehicle;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		UArrowComponent * Arrow;	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		UStaticMeshComponent * MapDimensions;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		USceneComponent * MapMinHeight;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		USceneComponent * MapMaxHeight;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		UETerrainResolution DesiredTerrainResolution = UETerrainResolution::Decimetres;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		FString StaticCollisionArchiveTag = "StaticCollision.cache";

	//Collision detection backend of the Chrono system. Chaos (default) collides with the Unreal level directly,
	//Bullet only sees the Chrono side geometry (MapData patches + gathered/archived static collision)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation")
		EVehicleCollisionBackend CollisionBackend = EVehicleCollisionBackend::Chaos;
	//Chrono substeps per Chaos physics tick when Tick Physics Async is enabled. The Chrono step size is
	//AsyncFixedTimeStepSize / ChronoSubStepsPerPhysicsTick (3 x 1/120 s = 2.78 ms, close to the previous 3 ms)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation", meta = (ClampMin = "1", ClampMax = "32"))
		int32 ChronoSubStepsPerPhysicsTick = 3;
	//Chrono step size (seconds) used when Tick Physics Async is disabled (variable Chaos step)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation", meta = (ClampMin = "0.0005", ClampMax = "0.02"))
		float SyncChronoStepSize = 1.f / 360.f;
	//Upper bound of Chrono substeps in a single physics tick, the backlog beyond it is dropped
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation", meta = (ClampMin = "1", ClampMax = "128"))
		int32 MaxChronoSubStepsPerTick = 32;
	//Maximum iterations of Chrono's iterative NSC solver
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation", meta = (ClampMin = "10", ClampMax = "1000"))
		int32 SolverMaxIterations = 140;
	//Bullet only: threads used by the Bullet collision system
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation", meta = (ClampMin = "1", ClampMax = "64"))
		int32 BulletCollisionThreads = 8;
	//Capture the Chrono contact points in every snapshot for debug drawing (costs one contact report per step)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation")
		bool bCaptureContactsForDebug = false;

	//Chaos only: collision object channel of the Chrono proxy shapes. Every Unreal shape that blocks this channel
	//(landscape, static meshes, rigid bodies, ...) collides with the vehicle and is sensed by its tires; the visual
	//vehicle / trailer meshes must NOT block it (or have no physics collision) to avoid self collision
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation")
		TEnumAsByte<ECollisionChannel> ChronoCollisionChannel = ECC_Vehicle;
	//Chaos only: persistent contact points whose separation exceeds this distance (cm) are dropped
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Simulation", meta = (ClampMin = "0.1", ClampMax = "20.0"))
		float ContactBreakingThresholdCm = 2.f;

	//Bullet only. Streams the static collision in and out of the physics system in grid cells around the vehicle.
	//Without this every single static collider of the map stays in the solver and the collision world
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Collision Streaming")
		bool EnableCollisionStreaming = true;
	//Size of a single grid cell in metres. Smaller cells stream more precisely but cost more cells to test
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Collision Streaming", meta = (ClampMin = "8.0"))
		float CollisionCellSizeMetres = 64.f;
	//Colliders within this distance (metres) of the vehicle are active
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Collision Streaming", meta = (ClampMin = "0.0"))
		float CollisionActivationRadiusMetres = 250.f;
	//Extra distance a cell has to travel away before it is deactivated again, stops cells thrashing on borders
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Collision Streaming", meta = (ClampMin = "0.0"))
		float CollisionCellHysteresisMetres = 50.f;
	//How often the worker thread re-evaluates the cells
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Collision Streaming", meta = (ClampMin = "0.01"))
		float CollisionStreamingIntervalSeconds = 0.1f;
	//Maximum amount of colliders the physics thread may attach/detach in a single step
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld|Collision Streaming", meta = (ClampMin = "1"))
		int32 CollisionStreamingBudgetPerStep = 128;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
	FString  MapDirectory = "";	

	//Calls Init() from BeginPlay
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CarPhysicsWorld")
		bool bAutoInit = true;

	UFUNCTION(BlueprintCallable, Category = "Car Physics World")
		void Init();
	UFUNCTION()
		void AddPhysicsVehicle(APhysicsVehicle* NewPhysicsVehicle);
	
	//Bullet only. The level geometry extraction itself lives in FUnrealCollisionExtractor (Tools/UnrealCollisionExtractor.h),
	//the entry points below are thin forwarders that keep the Blueprint / editor bindings unchanged

	//Returns the full path of the binary static collision cache for this level
	UFUNCTION(BlueprintCallable, Category = "Car Physics World")
		FString GetStaticCollisionArchivePath() const;
	//Looks for the StaticCollisionArchiveTag file and, if present, rebuilds the static collision from it
	//instead of walking the level again. Returns true when the cache was used (Bullet backend only)
	UFUNCTION(BlueprintCallable, Category = "Car Physics World")
		bool LoadSerializedCollision();

#if WITH_EDITOR
	//Gathers the chrono/bullet collision of every selected actor (recursively including attached children)
	//whose static mesh components are tagged 'ChronoStatic' and writes it to the static collision archive.
	//Returns the number of collision shapes written, or -1 if the archive could not be written
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "NextGenCars|Collision")
		void SerializeCollisionActors();
#endif //WITH_EDITOR

	//Builds the streaming grid from the gathered colliders and applies the cells that should be active at start
	void InitializeCollisionStreaming();

	//Number of static colliders currently attached to the physics system (Bullet backend)
	UFUNCTION(BlueprintPure, Category = "Car Physics World")
		int32 GetActiveStaticColliderCount() const { return FStaticCollisionGrid::Get().GetActiveBodyCount(); }

	//Whether the Chrono step is currently paused (mirrors the game pause state)
	UFUNCTION(BlueprintPure, Category = "Car Physics World")
		bool IsSimulationPaused() const { return bSimulationPaused; }

	//One-shot commands for the physics thread
	void RequestShift(int32 Direction);
	void ToggleAxleDifferentialLock(bool FrontAxle, bool Locked);
	void ToggleCenterDifferentialLock(bool Locked);

	//Newest vehicle snapshot pumped from the physics thread (never null, default constructed before the first step)
	const FVehicleSnapshot& GetVehicleSnapshot() const { return *VehicleSnapshot; }
	TSharedPtr<FChronoSimulation> GetSimulation() const { return Simulation; }
	EVehicleCollisionBackend GetCollisionBackend() const { return CollisionBackend; }

	//Finds the ACarPhysicsWorld of a world (there is at most one per level)
	static ACarPhysicsWorld* Get(UWorld* World);

	//Walks the tagged level actors and creates the Chrono collision bodies (Bullet backend only)
	void GatherUnrealCollision(bool bSkipStaticTag = false);

	//Backend comparison (set before BeginPlay): scripted driver inputs + recording of the vehicle state, see FChronoVehicleSetup
	TArray<BackendCompare::FScriptKey> InputScript;
	TSharedPtr<BackendCompare::FRunRecording> Recording;

	TArray<APhysicsVehicle*> PhysicsVehicles;

	std::vector<std::shared_ptr<FCollisionModel>> CollisionData;

private:
	//Builds the plain data setup of the Chrono vehicle world from the Unreal actors (game thread)
	TSharedPtr<FChronoVehicleSetup> MakeVehicleSetup() const;
	//Creates the FChronoSimulation hosting the Chrono system on the physics thread
	bool CreateSimulation();
	//Bullet only: gathers the level geometry, hands the bodies to the physics thread and starts the streaming worker
	void SetupBulletStaticCollision();
	void BuildTickInputs(FChronoTickInputs& OutInputs) const;
	void DistributeSnapshot();
	//Captures the actor settings the FUnrealCollisionExtractor reads (built on demand, also outside Init())
	FUnrealCollisionExtractorSettings MakeExtractorSettings() const;

	TSharedPtr<FChronoSimulation> Simulation;
	TSharedPtr<FChronoVehicleWorld> World;
	TSharedPtr<const FVehicleSnapshot> VehicleSnapshot;
	FStaticCollisionGridThread * CollisionGridThread = nullptr;
	bool bSimulationPaused = false;
};
