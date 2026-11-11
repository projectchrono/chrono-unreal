#include "CarPhysicsWorld.h"
#include "../ChronoCarPhysics/ChronoVehicleWorld.h"
#include "../ChronoCarPhysics/BulletStaticCollision.h"
#include "../Tools/UnrealCollisionExtractor.h"
#include "ChronoSimulation.h"
#include "EngineUtils.h"
#include "Misc/Paths.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include <Kismet/GameplayStatics.h>

#if WITH_EDITOR
#include "Chaos/DebugDrawQueue.h"
#endif

#define PrintToScreen(x, ...) if(GEngine){GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, FString::Printf(TEXT(x), __VA_ARGS__));}

#if WITH_EDITOR
#pragma optimize("", off)
#endif

namespace
{
	EChronoCollisionBackend ToPluginBackend(EVehicleCollisionBackend Backend)
	{
		return Backend == EVehicleCollisionBackend::Bullet ? EChronoCollisionBackend::Bullet : EChronoCollisionBackend::Chaos;
	}
}

// Sets default values
ACarPhysicsWorld::ACarPhysicsWorld()
{
	// Set this actor to call Tick() every frame. You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false;
	//Keep pushing inputs (with Flag_Paused) and pumping snapshots while the game is paused
	PrimaryActorTick.bTickEvenWhenPaused = true;
	Arrow = CreateDefaultSubobject<UArrowComponent>(TEXT("WorldMarker"));	
	MapDimensions = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MapDimensions"));
	MapMinHeight = CreateDefaultSubobject<USceneComponent>(TEXT("MapMinHeight"));
	MapMaxHeight = CreateDefaultSubobject<USceneComponent>(TEXT("MapMaxHeight"));
	VehicleSnapshot = MakeShared<FVehicleSnapshot>();
}

ACarPhysicsWorld* ACarPhysicsWorld::Get(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	for (TActorIterator<ACarPhysicsWorld> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

void ACarPhysicsWorld::EndPlay(const EEndPlayReason::Type EndPlayReason)
{	
	//Stop the streaming worker before the simulation, it must never queue work for a dead system
	if (CollisionGridThread)
	{
		CollisionGridThread->Stop();
		CollisionGridThread->Exit();

		while (CollisionGridThread->Running) {}
		delete CollisionGridThread;
		CollisionGridThread = nullptr;
	}

	//Destroy the simulation BEFORE the physics scene goes away: this unregisters the sim callback and Chaos destroys
	//the stepper (FChronoVehicleWorld) and the ChSystem on the physics thread while the solver is still alive
	World.Reset();
	Simulation.Reset();

	FStaticCollisionGrid::Get().Reset();
	CollisionData.clear();
	VehicleSnapshot = MakeShared<FVehicleSnapshot>();

	Super::EndPlay(EndPlayReason);
}

// Called when the game starts or when spawned
void ACarPhysicsWorld::BeginPlay()
{
	Super::BeginPlay();

	if (!PhysicsVehicle)
	{
		for (TActorIterator<APhysicsVehicle> It(GetWorld()); It; ++It)
		{
			PhysicsVehicle = *It;
			break;
		}
	}
	if (bAutoInit)
	{
		Init();
	}
}

TSharedPtr<FChronoVehicleSetup> ACarPhysicsWorld::MakeVehicleSetup() const
{
	FString ProjectDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() + "Binaries/Win64/data/vehicle/");
	std::string ProjectDirFull(TCHAR_TO_UTF8(*ProjectDir));
	std::string TerrainDirectory = ProjectDirFull + "MapData/" + std::string(TCHAR_TO_UTF8(*MapDirectory));

	TSharedPtr<FChronoVehicleSetup> Setup = MakeShared<FChronoVehicleSetup>();
	Setup->Backend = ToPluginBackend(CollisionBackend);
	Setup->Vehicle = std::make_shared<FChVehicleSetup>(PhysicsVehicle->ChVehicleSetup);
	Setup->StartPose = ChCoordsysd(Conversions::ToChronoVec(PhysicsVehicle->GetActorLocation()), Conversions::ToChronoQuat(PhysicsVehicle->GetActorQuat()));
	Setup->DriverMassLbs = 180.0;
	Setup->MapDataDir = TerrainDirectory;
	Setup->bStreamStaticColliders = CollisionBackend == EVehicleCollisionBackend::Bullet && EnableCollisionStreaming;
	Setup->StreamingBudgetPerStep = CollisionStreamingBudgetPerStep;
	Setup->CollisionThreads = BulletCollisionThreads;
	Setup->InputScript = InputScript;
	Setup->Recording = Recording;
	//Physical material table for the physics-thread terrain type lookup: every loaded UVehiclePhysicalMaterial,
	//identified by the user data pointer Chaos attaches to its FChaosPhysicsMaterial (&UPhysicalMaterial::UserData).
	//Only the pointer identity crosses to the physics thread.
	for (TObjectIterator<UVehiclePhysicalMaterial> It; It; ++It)
	{
		UVehiclePhysicalMaterial* Material = *It;
		if (!Material || Material->HasAnyFlags(RF_ClassDefaultObject))
		{
			continue;
		}
		FTerrainMaterialEntry Entry;
		Entry.MaterialUserData = &Material->UserData;
		Entry.TerrainType = Material->MaterialType;
		Entry.Friction = Material->Friction;
		Entry.RollingFriction = Material->RollingFriction;
		Entry.SpinningFriction = Material->SpinningFriction;
		Setup->Materials.push_back(Entry);
	}

	return Setup;
}

bool ACarPhysicsWorld::CreateSimulation()
{
	FChronoSimulationParams Params;
	Params.Scene = GetWorld() ? GetWorld()->GetPhysicsScene() : nullptr;
	Params.Backend = ToPluginBackend(CollisionBackend);
	Params.bSMC = false;
	Params.MaxSubStepsPerTick = MaxChronoSubStepsPerTick;
	Params.SolverMaxIterations = SolverMaxIterations;
	Params.bCaptureContacts = EnableChronoVisual && bCaptureContactsForDebug;

	//Chaos backend: the proxies live on the vehicle channel and collide with everything that blocks it
	Params.Collision.Filter.CollisionChannel = (uint8)ChronoCollisionChannel.GetValue();
	Params.Collision.Filter.BlockChannels = ~0ull & ~(1ull << ChronoCollisionChannel.GetValue());
	Params.Collision.bCollideWithUnrealGeometry = true;
	Params.Collision.ContactBreakingThresholdCm = ContactBreakingThresholdCm;

	//Derive the Chrono step from the fixed physics tick so the vehicle always sees whole substeps per Chaos step
	const UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get();
	if (PhysicsSettings && PhysicsSettings->bTickPhysicsAsync && PhysicsSettings->AsyncFixedTimeStepSize > 0.f)
	{
		Params.StepSizeS = (double)PhysicsSettings->AsyncFixedTimeStepSize / FMath::Max(ChronoSubStepsPerPhysicsTick, 1);
	}
	else
	{
		Params.StepSizeS = SyncChronoStepSize;
	}

	Simulation = FChronoSimulation::Create(Params);
	if (!Simulation)
	{
		UE_LOG(LogTemp, Error, TEXT("ACarPhysicsWorld::Init: could not create the Chrono simulation (no physics scene?)"));
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("ACarPhysicsWorld: Chrono simulation created (%s backend, step %.4f ms, async physics %s)"),
		CollisionBackend == EVehicleCollisionBackend::Chaos ? TEXT("Chaos") : TEXT("Bullet"), Params.StepSizeS * 1000.0,
		(PhysicsSettings && PhysicsSettings->bTickPhysicsAsync) ? TEXT("on") : TEXT("off"));
	return true;
}

void ACarPhysicsWorld::Init()
{	
	FString ProjectDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() + "Binaries/Win64/data/vehicle/");
	std::string ProjectDirFull(TCHAR_TO_UTF8(*ProjectDir));
	chrono::vehicle::SetVehicleDataPath(ProjectDirFull);	

#if !WITH_EDITOR
	EnableChronoVisual = false;
#endif

	if (!PhysicsVehicle)
	{
		return;
	}

	PhysicsVehicle->Build();
	PhysicsVehicles.AddUnique(PhysicsVehicle);

	if (!CreateSimulation())
	{
		return;
	}

	//The vehicle world is built on the physics thread inside the first command and installed as the stepper
	TSharedPtr<FChronoVehicleSetup> Setup = MakeVehicleSetup();
	World = MakeShared<FChronoVehicleWorld>();
	{
		TSharedPtr<FChronoVehicleWorld> WorldRef = World;
		Simulation->Enqueue([WorldRef, Setup](FChronoCommandContext& Ctx)
		{
			if (WorldRef->Build(Ctx.System, *Setup))
			{
				Ctx.SetStepper(WorldRef);
			}
		});
	}

	//Bullet only: the level geometry has to be exported to Chrono. Chaos collides with the level directly
	if (CollisionBackend == EVehicleCollisionBackend::Bullet)
	{
		SetupBulletStaticCollision();
	}


	PhysicsVehicle->SetActorTickEnabled(true);
	SetActorTickEnabled(true);
		
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (PC)
	{
		PC->Possess(PhysicsVehicle);
	}

}

void ACarPhysicsWorld::SetupBulletStaticCollision()
{
	//If the level's static collision was baked out by SerializeCollisionActors we just
	//load it back instead of walking every actor in the level again
	const bool bLoadedStaticCollision = LoadSerializedCollision();
	GatherUnrealCollision(bLoadedStaticCollision);
	InitializeCollisionStreaming();

	if (EnableCollisionStreaming && FStaticCollisionGrid::Get().IsBuilt())
	{
		//The grid owns the bodies and streams them in/out per cell from the stepper. Attach the initial set of cells
		//on the physics thread, then start the worker that decides which cells should be active
		Simulation->Enqueue([](FChronoCommandContext& Ctx)
		{
			FStaticCollisionGrid::Get().ApplyAllPending(&Ctx.System);
		});
		CollisionGridThread = new FStaticCollisionGridThread("StaticCollisionGridThread", CollisionStreamingIntervalSeconds);
	}
	else
	{
		//No streaming: every static body lives in the system for the whole session
		std::vector<std::shared_ptr<chrono::ChBody>> Bodies;
		Bodies.reserve(CollisionData.size());
		for (auto&& Model : CollisionData)
		{
			if (Model && Model->Shape)
			{
				Bodies.push_back(Model->Shape);
			}
		}
		Simulation->Enqueue([Bodies = std::move(Bodies)](FChronoCommandContext& Ctx)
		{
			FBulletStaticCollision::AddBodies(Ctx.System, Bodies);
		});
	}
}

void ACarPhysicsWorld::BuildTickInputs(FChronoTickInputs& OutInputs) const
{
	OutInputs = FChronoTickInputs();
	if (bSimulationPaused)
	{
		OutInputs.Flags |= FChronoTickInputs::Flag_Paused;
	}

	if (PhysicsVehicle && PhysicsVehicle->IsPlayerControlled())
	{
		const FDriverInputs& Inputs = PhysicsVehicle->GetInputs();
		OutInputs.Throttle = Inputs.Throttle;
		OutInputs.Steering = Inputs.Steering;
		OutInputs.Braking = Inputs.Brake;
		if (Inputs.Handbrake > 0.f)
		{
			OutInputs.Flags |= VehicleInputFlags::Handbrake;
		}
	}
}

// Called every frame
void ACarPhysicsWorld::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Simulation)
	{
		return;
	}

	bSimulationPaused = UGameplayStatics::IsGamePaused(GetWorld());

	for (auto&& Veh : PhysicsVehicles)
	{
		if (!Veh || !Veh->IsPlayerControlled())
		{
			continue;
		}

		//Gear shifts are events: hand them over as one-shot commands, the idempotent state goes through PushInputs
		const FDriverInputs& Inputs = Veh->GetInputs();
		if (Inputs.Shift != 0)
		{
			RequestShift(Inputs.Shift);
		}
		Veh->ConsumeInput();
	}

	FChronoTickInputs TickInputs;
	BuildTickInputs(TickInputs);
	Simulation->PushInputs(TickInputs);

	Simulation->PumpOutputs();
	DistributeSnapshot();

	if (EnableChronoVisual != FStaticCollisionGrid::Get().GetEnableChronoVisual())
	{
		FStaticCollisionGrid::Get().SetEnableChronoVisual(EnableChronoVisual);
	}

}

void ACarPhysicsWorld::DistributeSnapshot()
{
	const FChronoSnapshot& Snapshot = Simulation->GetLatestSnapshot();
	TSharedPtr<const FVehicleSnapshot> Payload = StaticCastSharedPtr<const FVehicleSnapshot>(Snapshot.Payload);
	if (!Payload)
	{
		return;
	}
	VehicleSnapshot = Payload;

	for (auto&& Veh : PhysicsVehicles)
	{
		if (Veh)
		{
			Veh->ApplySnapshot(*VehicleSnapshot);
		}
	}

}

void ACarPhysicsWorld::RequestShift(int32 Direction)
{
	if (Simulation && World)
	{
		TSharedPtr<FChronoVehicleWorld> WorldRef = World;
		Simulation->Enqueue([WorldRef, Direction](FChronoCommandContext&) { WorldRef->RequestShift(Direction); });
	}
}

void ACarPhysicsWorld::ToggleAxleDifferentialLock(bool FrontAxle, bool Locked)
{
	if (Simulation && World)
	{
		TSharedPtr<FChronoVehicleWorld> WorldRef = World;
		Simulation->Enqueue([WorldRef, FrontAxle, Locked](FChronoCommandContext&) { WorldRef->ToggleAxleDiffLock(FrontAxle, Locked); });
	}
}

void ACarPhysicsWorld::ToggleCenterDifferentialLock(bool Locked)
{
	if (Simulation && World)
	{
		TSharedPtr<FChronoVehicleWorld> WorldRef = World;
		Simulation->Enqueue([WorldRef, Locked](FChronoCommandContext&) { WorldRef->ToggleCenterDiffLock(Locked); });
	}
}

void ACarPhysicsWorld::AddPhysicsVehicle(APhysicsVehicle* NewPhysicsVehicle)
{
	PhysicsVehicles.Add(NewPhysicsVehicle);	
}

//The level geometry extraction lives in FUnrealCollisionExtractor (Tools/UnrealCollisionExtractor.cpp). The extractor
//is stateless apart from the captured settings and the reference to CollisionData, so it is built on demand per call;
//this also works for the CallInEditor SerializeCollisionActors path that runs without Init()
FUnrealCollisionExtractorSettings ACarPhysicsWorld::MakeExtractorSettings() const
{
	FUnrealCollisionExtractorSettings Settings;
	Settings.World = GetWorld();
	Settings.OwnerName = GetName();
	Settings.EnableChronoVisual = EnableChronoVisual;
	Settings.DesiredTerrainResolution = DesiredTerrainResolution;
	Settings.MapDirectory = MapDirectory;
	Settings.StaticCollisionArchiveTag = StaticCollisionArchiveTag;
	Settings.EnableCollisionStreaming = EnableCollisionStreaming;
	Settings.CollisionCellSizeMetres = CollisionCellSizeMetres;
	Settings.CollisionActivationRadiusMetres = CollisionActivationRadiusMetres;
	Settings.CollisionCellHysteresisMetres = CollisionCellHysteresisMetres;
	Settings.PhysicsVehicle = PhysicsVehicle;
	return Settings;
}

void ACarPhysicsWorld::GatherUnrealCollision(bool bSkipStaticTag)
{
	FUnrealCollisionExtractor(MakeExtractorSettings(), CollisionData).GatherUnrealCollision(bSkipStaticTag);
}

void ACarPhysicsWorld::InitializeCollisionStreaming()
{
	FUnrealCollisionExtractor(MakeExtractorSettings(), CollisionData).InitializeCollisionStreaming();
}

FString ACarPhysicsWorld::GetStaticCollisionArchivePath() const
{
	return FUnrealCollisionExtractor::GetStaticCollisionArchivePath(MakeExtractorSettings());
}

bool ACarPhysicsWorld::LoadSerializedCollision()
{
	return FUnrealCollisionExtractor(MakeExtractorSettings(), CollisionData).LoadSerializedCollision();
}

#if WITH_EDITOR

//Editor only tool. Bakes all ChronoStatic tagged geometry into the
//StaticCollisionArchiveTag file so the runtime can load it back instead of walking the level
void ACarPhysicsWorld::SerializeCollisionActors()
{
	FUnrealCollisionExtractor(MakeExtractorSettings(), CollisionData).SerializeCollisionActors();
}

#endif //WITH_EDITOR

#pragma optimize("", on)
