#include "BackendComparison.h"

#include "CarPhysicsWorld.h"
#include "PhysicsVehicle.h"
#include "Conversions.h"
#include "Camera/CameraActor.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/DateTime.h"
#include "Misc/EngineVersion.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

DEFINE_LOG_CATEGORY_STATIC(LogBackendCompare, Log, All);

using namespace BackendCompare;

namespace
{
	const TCHAR* BackendName(EChronoCollisionBackend B) { return B == EChronoCollisionBackend::Chaos ? TEXT("Chaos") : TEXT("Bullet"); }

	const TCHAR* TireName(ETireModelType T)
	{
		switch (T)
		{
		case ETireModelType::RIGID: return TEXT("RIGID");
		case ETireModelType::TMEASY: return TEXT("TMEASY");
		default: return TEXT("DISCRETE");
		}
	}

	// Driver script of the vehicle runs (seconds since the vehicle was built): settle at rest, accelerate, steer left and
	// right, then brake. Everything is a function of simulation time.
	TArray<FScriptKey> MakeVehicleScript(double Duration)
	{
		TArray<FScriptKey> Keys;
		const auto Key = [&](float T, float Throttle, float Steering, float Braking) { FScriptKey K; K.Time = T; K.Throttle = Throttle; K.Steering = Steering; K.Braking = Braking; Keys.Add(K); };
		Key(0.0f, 0.0f, 0.0f, 0.0f);
		Key(1.5f, 0.0f, 0.0f, 0.0f);     // settle on the suspension
		Key(2.0f, 0.6f, 0.0f, 0.0f);     // accelerate
		Key(4.5f, 0.6f, 0.0f, 0.0f);
		Key(5.5f, 0.6f, 0.35f, 0.0f);    // steer left
		Key(6.5f, 0.6f, -0.35f, 0.0f);   // and right
		Key(7.0f, 0.0f, 0.0f, 0.0f);     // lift
		Key(7.2f, 0.0f, 0.0f, 0.5f);     // brake
		Key((float)Duration + 1.f, 0.0f, 0.0f, 0.5f);
		return Keys;
	}

	FString Sci(double V) { return FString::Printf(TEXT("%.3g"), V); }

	void DrawBenchShape(UWorld* World, EBenchShape Shape, const FVector& Size, const FVector& C, const FQuat& Q, const FColor& Color)
	{
		switch (Shape)
		{
		case EBenchShape::Box:
			DrawDebugBox(World, C, Size * 50.0, Q, Color, false, -1.f, 0, 1.5f);
			break;
		case EBenchShape::Sphere:
			DrawDebugSphere(World, C, Size.X * 100.0, 10, Color, false, -1.f, 0, 1.f);
			break;
		case EBenchShape::Capsule:
			DrawDebugCapsule(World, C, (Size.Y * 0.5 + Size.X) * 100.0, Size.X * 100.0, Q, Color, false, -1.f, 0, 1.5f);
			break;
		case EBenchShape::Cylinder:
		{
			const FVector Axis = Q.RotateVector(FVector::UpVector) * Size.Y * 50.0;
			DrawDebugCylinder(World, C - Axis, C + Axis, Size.X * 100.0, 12, Color, false, -1.f, 0, 1.5f);
			break;
		}
		case EBenchShape::Hull:
		{
			const FVector V[6] = { FVector(Size.X, 0, 0), FVector(-Size.X, 0, 0), FVector(0, Size.Y, 0), FVector(0, -Size.Y, 0), FVector(0, 0, Size.Z), FVector(0, 0, -Size.Z) };
			FVector P[6];
			for (int32 k = 0; k < 6; ++k) { P[k] = C + Q.RotateVector(V[k] * 100.0); }
			for (int32 a = 0; a < 6; ++a) { for (int32 b = a + 1; b < 6; ++b) { if ((a ^ 1) != b) { DrawDebugLine(World, P[a], P[b], Color, false, -1.f, 0, 1.5f); } } }
			break;
		}
		}
	}
}

AChronoBackendComparison::AChronoBackendComparison()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AChronoBackendComparison::BuildRunList()
{
	Runs.Reset();
	// Command line shortcuts for quick iterations / CI
	if (FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareVehicleOnly"))) { bRunRigidBodyTests = false; }
	if (FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareRigidOnly"))) { bRunVehicleTests = false; }
	if (FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareNoRepeat"))) { bMeasureRepeatability = false; }
	if (FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareColliders"))) { bRunColliderTests = true; }
	if (FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareCollidersOnly"))) { bRunColliderTests = true; bRunVehicleTests = false; bRunRigidBodyTests = false; }
	if (FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareSerialChaos"))) { ChaosParallelThreshold = MAX_int32; }
	FString TireArg;
	if (FParse::Value(FCommandLine::Get(), TEXT("ChronoCompareTire="), TireArg))
	{
		TireModels.Reset();
		if (TireArg.Contains(TEXT("D"), ESearchCase::IgnoreCase)) { TireModels.Add(ETireModelType::DISCRETE); }
		if (TireArg.Contains(TEXT("T"), ESearchCase::IgnoreCase)) { TireModels.Add(ETireModelType::TMEASY); }
		if (TireArg.Contains(TEXT("R"), ESearchCase::IgnoreCase)) { TireModels.Add(ETireModelType::RIGID); }
	}
	const int32 NumRepeats = bMeasureRepeatability ? 2 : 1;

	FParse::Value(FCommandLine::Get(), TEXT("ChronoCompareOnly="), ScenarioFilter);
	const auto AddScenario = [&](EKind Kind, ETireModelType Tire, const FString& Name)
	{
		if (!ScenarioFilter.IsEmpty() && !Name.Contains(ScenarioFilter, ESearchCase::IgnoreCase))
		{
			return;
		}
		// Order: Chaos, Bullet, then the repeats (Chaos, Bullet)
		for (int32 Repeat = 0; Repeat < NumRepeats; ++Repeat)
		{
			for (EChronoCollisionBackend Backend : { EChronoCollisionBackend::Chaos, EChronoCollisionBackend::Bullet })
			{
				FRunSpec Run;
				Run.Kind = Kind;
				Run.Tire = Tire;
				Run.Backend = Backend;
				Run.Repeat = Repeat;
				Run.Scenario = Name;
				Runs.Add(MoveTemp(Run));
			}
		}
	};

	if (bRunVehicleTests)
	{
		for (ETireModelType Tire : TireModels)
		{
			AddScenario(EKind::Vehicle, Tire, FString::Printf(TEXT("vehicle_%s"), TireName(Tire)));
		}
	}
	if (bRunRigidBodyTests)
	{
		AddScenario(EKind::Roll, ETireModelType::DISCRETE, TEXT("rigid_roll"));
		AddScenario(EKind::Sparse, ETireModelType::DISCRETE, TEXT("rigid_sparse"));
		AddScenario(EKind::Pile, ETireModelType::DISCRETE, TEXT("rigid_pile"));
	}
	if (bRunColliderTests)
	{
		AddScenario(EKind::Pairs, ETireModelType::DISCRETE, TEXT("col_pairs"));
		AddScenario(EKind::Stack, ETireModelType::DISCRETE, TEXT("col_stack"));
		AddScenario(EKind::Slide, ETireModelType::DISCRETE, TEXT("col_slide"));
		AddScenario(EKind::Bounce, ETireModelType::DISCRETE, TEXT("col_bounce"));
	}
}

void AChronoBackendComparison::BeginPlay()
{
	Super::BeginPlay();

	OutputDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("BackendComparison") / FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
	IFileManager::Get().MakeDirectory(*OutputDir, true);

	BuildRunList();
	UE_LOG(LogBackendCompare, Display, TEXT("Backend comparison: %d runs, output in %s"), Runs.Num(), *OutputDir);

	CurrentRun = -1;
	CooldownFrames = 90;   // let the level settle before the first run
	Phase = EPhase::Cooldown;
}

void AChronoBackendComparison::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopRun();
	Super::EndPlay(EndPlayReason);
}

void AChronoBackendComparison::ShowStatus(const FString& Text, const FColor& Color) const
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(7701, 5.f, Color, Text);
	}
}

void AChronoBackendComparison::StartRun(FRunSpec& Run)
{
	UWorld* World = GetWorld();
	Run.Recording = MakeShared<FRunRecording>();
	Run.WallSeconds = 0.0;
	RunWallStart = FPlatformTime::Seconds();

	const UPhysicsSettings* PhysicsSettings = UPhysicsSettings::Get();
	// Same Chrono step the vehicle world uses (one third of the fixed physics step)
	const double StepSize = (PhysicsSettings && PhysicsSettings->bTickPhysicsAsync && PhysicsSettings->AsyncFixedTimeStepSize > 0.f)
		? (double)PhysicsSettings->AsyncFixedTimeStepSize / 3.0 : 1.0 / 360.0;

	if (Run.Kind == EKind::Vehicle)
	{
		Run.Recording->Label = FString::Printf(TEXT("%s %s"), *Run.Scenario, BackendName(Run.Backend));
		Run.Recording->Duration = VehicleDuration;

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Vehicle = World->SpawnActor<APhysicsVehicle>(APhysicsVehicle::StaticClass(), FTransform(FRotator::ZeroRotator, VehicleStart), SpawnParams);
		Vehicle->TireModel = Run.Tire;

		// The physics world initializes itself in BeginPlay, so everything is set before it is finished spawning
		PhysicsWorld = World->SpawnActorDeferred<ACarPhysicsWorld>(ACarPhysicsWorld::StaticClass(), FTransform::Identity, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		PhysicsWorld->PhysicsVehicle = Vehicle;
		PhysicsWorld->CollisionBackend = Run.Backend == EChronoCollisionBackend::Chaos ? EVehicleCollisionBackend::Chaos : EVehicleCollisionBackend::Bullet;
		PhysicsWorld->InputScript = MakeVehicleScript(VehicleDuration);
		PhysicsWorld->Recording = Run.Recording;
		PhysicsWorld->bAutoInit = true;
		PhysicsWorld->FinishSpawning(FTransform::Identity);
	}
	else
	{
		switch (Run.Kind)
		{
		case EKind::Pile:   RigidScene = MakePileScene(Seed); break;
		case EKind::Roll:   RigidScene = MakeRollScene(Seed); break;
		case EKind::Pairs:  RigidScene = MakePairScene(); break;
		case EKind::Stack:  RigidScene = MakeStackScene(); break;
		case EKind::Slide:  RigidScene = MakeSlideScene(); break;
		case EKind::Bounce: RigidScene = MakeBounceScene(); break;
		default:            RigidScene = MakeSparseScene(Seed); break;
		}
		Run.Recording->Label = FString::Printf(TEXT("%s %s"), *Run.Scenario, BackendName(Run.Backend));
		Run.Recording->Duration = RigidScene.Duration;

		FChronoSimulationParams Params;
		Params.Scene = World->GetPhysicsScene();
		Params.Backend = Run.Backend;
		Params.bSMC = false;
		Params.StepSizeS = StepSize;
		Params.MaxSubStepsPerTick = 32;
		Params.SolverMaxIterations = 140;
		// A pure Chrono scene: the Unreal level (the vehicle ground) must not take part
		Params.Collision.bCollideWithUnrealGeometry = false;
		Params.Collision.ParallelThreshold = ChaosParallelThreshold;
		RigidSimulation = FChronoSimulation::Create(Params);
		if (!RigidSimulation)
		{
			Run.bTimedOut = true;
			Run.Recording->bDone = true;
			return;
		}

		RigidBodyIds.Reset();
		for (int32 i = 0; i < RigidScene.Bodies.Num(); ++i)
		{
			RigidBodyIds.Add(RigidSimulation->AllocateBodyId());
		}

		const ChVector3d Origin = Conversions::ToChronoVec(ArenaCenter);
		const FVector OriginM(Origin.x(), Origin.y(), Origin.z());
		TSharedPtr<FBenchScene> Scene = MakeShared<FBenchScene>(RigidScene);
		TSharedPtr<FRunRecording> Recording = Run.Recording;
		TArray<FChronoBodyId> Ids = RigidBodyIds;
		RigidSimulation->Enqueue([Scene, Recording, OriginM, Ids](FChronoCommandContext& Ctx)
		{
			TArray<std::shared_ptr<chrono::ChBody>> Bodies;
			BuildBenchScene(Ctx.System, *Scene, OriginM, *Recording, Bodies);
			for (int32 i = 0; i < Bodies.Num(); ++i)
			{
				Ctx.TrackBody(Ids[i], Bodies[i]);
			}
			Ctx.SetStepper(MakeShared<FRigidBenchStepper>(Recording, Bodies, 3));
		});

		// Look at the arena (framed per scene; scene coordinates are Chrono metres, y mirrored)
		{
			if (!ArenaCamera)
			{
				FActorSpawnParameters SpawnParams;
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
				ArenaCamera = World->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), FTransform::Identity, SpawnParams);
			}
			const FVector Target = ArenaCenter + FVector(RigidScene.ViewTarget.X, -RigidScene.ViewTarget.Y, RigidScene.ViewTarget.Z) * 100.0;
			const FVector Location = Target + FVector(-RigidScene.ViewDistance * 100.0, 0, RigidScene.ViewDistance * 25.0);
			ArenaCamera->SetActorLocationAndRotation(Location, FRotationMatrix::MakeFromX(Target - Location).Rotator());
		}
		if (APlayerController* PC = World->GetFirstPlayerController())
		{
			PC->SetViewTarget(ArenaCamera);
		}
	}
}

void AChronoBackendComparison::StopRun()
{
	if (PhysicsWorld)
	{
		PhysicsWorld->Destroy();
		PhysicsWorld = nullptr;
	}
	if (Vehicle)
	{
		Vehicle->Destroy();
		Vehicle = nullptr;
	}
	RigidSimulation.Reset();
}

void AChronoBackendComparison::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	switch (Phase)
	{
	case EPhase::Cooldown:
	{
		if (--CooldownFrames > 0)
		{
			break;
		}
		++CurrentRun;
		if (!Runs.IsValidIndex(CurrentRun))
		{
			Phase = EPhase::Finished;
			Analyze();
			WriteReport();
			break;
		}
		FRunSpec& Run = Runs[CurrentRun];
		UE_LOG(LogBackendCompare, Display, TEXT("Run %d/%d: %s, %s backend%s"), CurrentRun + 1, Runs.Num(), *Run.Scenario, BackendName(Run.Backend), Run.Repeat ? TEXT(" (repeat)") : TEXT(""));
		StartRun(Run);
		Phase = EPhase::Running;
		break;
	}
	case EPhase::Running:
	{
		FRunSpec& Run = Runs[CurrentRun];
		Run.WallSeconds = FPlatformTime::Seconds() - RunWallStart;
		double SimTime = 0.0;
		{
			FScopeLock ScopeLock(&Run.Recording->Lock);
			SimTime = Run.Recording->Times.Num() > 0 ? Run.Recording->Times.Last() : 0.0;
		}
		ShowStatus(FString::Printf(TEXT("Backend comparison: run %d / %d  %s  %s%s  t = %.1f / %.1f s"), CurrentRun + 1, Runs.Num(), *Run.Scenario, BackendName(Run.Backend),
			Run.Repeat ? TEXT(" (repeat)") : TEXT(""), SimTime, Run.Recording->Duration), Run.Backend == EChronoCollisionBackend::Chaos ? FColor::Green : FColor::Orange);

		if (Run.Kind != EKind::Vehicle && RigidSimulation)
		{
			RigidSimulation->PumpOutputs();
			DrawRigidBodies();
		}

		if (!Run.Recording->bDone && Run.WallSeconds > Run.Recording->Duration * 4.0 + 30.0)
		{
			Run.bTimedOut = true;
			UE_LOG(LogBackendCompare, Error, TEXT("Run %s %s timed out after %.0f s (sim time %.2f)"), *Run.Scenario, BackendName(Run.Backend), Run.WallSeconds, SimTime);
		}
		if (Run.Recording->bDone || Run.bTimedOut)
		{
			StopRun();
			CooldownFrames = 45;
			Phase = EPhase::Cooldown;
		}
		break;
	}
	default:
		break;
	}
}

void AChronoBackendComparison::DrawRigidBodies() const
{
	UWorld* World = GetWorld();
	if (!World || !RigidSimulation)
	{
		return;
	}
	const FRunSpec& Run = Runs[CurrentRun];
	const FColor Color = Run.Backend == EChronoCollisionBackend::Chaos ? FColor(60, 220, 60) : FColor(255, 150, 30);
	const FChronoSnapshot& Snapshot = RigidSimulation->GetLatestSnapshot();

	// Ground and container outline
	DrawDebugBox(World, ArenaCenter + FVector(0, 0, -2), FVector(1000, 1000, 2), FColor(90, 90, 90), false, -1.f, 0, 2.f);
	for (const FBenchFixedBox& Wall : RigidScene.FixedBoxes)
	{
		// Scene coordinates are Chrono (y mirrored) metres relative to the arena centre
		DrawDebugBox(World, ArenaCenter + FVector(Wall.Center.X, -Wall.Center.Y, Wall.Center.Z) * 100.0, FVector(Wall.Size.X, Wall.Size.Y, Wall.Size.Z) * 50.0, FColor(150, 150, 150), false, -1.f, 0, 2.f);
	}

	for (const FBenchFixedShape& F : RigidScene.FixedShapes)
	{
		const FQuat Q(-F.Rotation.X, F.Rotation.Y, -F.Rotation.Z, F.Rotation.W);   // y mirrored
		DrawBenchShape(World, F.Shape, F.Size, ArenaCenter + FVector(F.Position.X, -F.Position.Y, F.Position.Z) * 100.0, Q, FColor(150, 150, 150));
	}

	for (int32 i = 0; i < RigidScene.Bodies.Num() && i < RigidBodyIds.Num(); ++i)
	{
		const FChronoBodyState* State = Snapshot.FindBody(RigidBodyIds[i]);
		if (!State)
		{
			continue;
		}
		const FBenchBodySpec& S = RigidScene.Bodies[i];
		DrawBenchShape(World, S.Shape, S.Size, State->Transform.GetLocation(), State->Transform.GetRotation(), Color);
	}
}

bool AChronoBackendComparison::HasScenario(const FString& Scenario) const
{
	for (const FRunSpec& Run : Runs)
	{
		if (Run.Scenario == Scenario)
		{
			return true;
		}
	}
	return false;
}

AChronoBackendComparison::FRunSpec* AChronoBackendComparison::FindRun(const FString& Scenario, EChronoCollisionBackend Backend, int32 Repeat)
{
	for (FRunSpec& Run : Runs)
	{
		if (Run.Scenario == Scenario && Run.Backend == Backend && Run.Repeat == Repeat)
		{
			return &Run;
		}
	}
	return nullptr;
}

void AChronoBackendComparison::AddCheck(FScenarioResult& R, const FString& Name, double Value, double Limit, const TCHAR* Unit, bool bRequired)
{
	FCheck C;
	C.Name = Name;
	C.Value = FString::Printf(TEXT("%s %s"), *Sci(Value), Unit);
	C.Limit = FString::Printf(TEXT("<= %s %s"), *Sci(Limit), Unit);
	C.bRequired = bRequired;
	C.bPass = Value <= Limit;
	R.Checks.Add(C);
}

AChronoBackendComparison::FScenarioResult AChronoBackendComparison::EvaluateVehicle(const FString& Scenario, const FString& Title)
{
	FScenarioResult R;
	R.Name = Title;

	FRunSpec* Chaos = FindRun(Scenario, EChronoCollisionBackend::Chaos, 0);
	FRunSpec* Bullet = FindRun(Scenario, EChronoCollisionBackend::Bullet, 0);
	FRunSpec* Chaos2 = FindRun(Scenario, EChronoCollisionBackend::Chaos, 1);
	FRunSpec* Bullet2 = FindRun(Scenario, EChronoCollisionBackend::Bullet, 1);

	const auto Completed = [&](FRunSpec* Run, const TCHAR* Name) -> bool
	{
		const bool bOk = Run && Run->Recording && Run->Recording->bDone && !Run->bTimedOut && Run->Recording->NumSamples() > 10;
		FCheck C;
		C.Name = FString::Printf(TEXT("%s run completed"), Name);
		C.Value = bOk ? FString::Printf(TEXT("%d samples, %.1f s wall"), Run->Recording->NumSamples(), Run->WallSeconds) : TEXT("incomplete");
		C.Limit = TEXT("completes");
		C.bPass = bOk;
		R.Checks.Add(C);
		return bOk;
	};
	if (!Completed(Chaos, TEXT("Chaos")) | !Completed(Bullet, TEXT("Bullet")))
	{
		return R;
	}

	const FTrajectoryComparison Cross = CompareTrajectories(Title + TEXT(": Chaos vs Bullet"), *Chaos->Recording, *Bullet->Recording);
	Trajectories.Add(Cross);
	if (!Cross.bValid)
	{
		R.Notes.Add(TEXT("comparison failed: ") + Cross.Error);
		FCheck C; C.Name = TEXT("recordings comparable"); C.Value = Cross.Error; C.Limit = TEXT("same time grid"); C.bPass = false; R.Checks.Add(C);
		return R;
	}

	const double PositionLimit = Criteria.VehiclePositionBaseM + Criteria.VehiclePositionPathFraction * Cross.PathLengthA;
	double WheelMax = 0.0;
	for (int32 b = 1; b < Cross.BodyPosition.Num(); ++b)
	{
		WheelMax = FMath::Max(WheelMax, Cross.BodyPosition[b].Max);
	}
	AddCheck(R, TEXT("chassis position, max error"), Cross.BodyPosition[0].Max, PositionLimit, TEXT("m"));
	AddCheck(R, TEXT("wheel (spindle) positions, max error"), WheelMax, PositionLimit, TEXT("m"));
	AddCheck(R, TEXT("chassis orientation, max error"), Cross.BodyRotation[0].Max, Criteria.VehicleRotationMaxDeg, TEXT("deg"));
	// Terrain sensing at fixed points (flat asphalt, ramp, bumps): the same ray the tires use, independent of the vehicle
	const FTerrainProbeComparison Probe = CompareTerrainProbes(*Chaos->Recording, *Bullet->Recording);
	if (Probe.bValid)
	{
		AddCheck(R, TEXT("terrain query: friction, max difference"), Probe.FrictionMax, Criteria.TerrainFrictionMax, TEXT(""));
		AddCheck(R, TEXT("terrain query: height, max difference"), Probe.HeightMax, Criteria.TerrainHeightMaxM, TEXT("m"));
		AddCheck(R, TEXT("terrain query: normal, max difference"), Probe.NormalMaxDeg, Criteria.TerrainNormalMaxDeg, TEXT("deg"));
		AddCheck(R, TEXT("terrain query: points hit by only one backend"), Probe.HitMismatches, 0.0, TEXT("points"));
		AddCheck(R, TEXT("terrain query: surface type mismatches"), Probe.TypeMismatches, 0.0, TEXT("points"));
		R.Notes.Add(FString::Printf(TEXT("terrain probe: %d points (3 lines along the track, 0.25 m apart); worst height difference at x=%.2f y=%.2f; worst friction %.2f (Chaos) vs %.2f (Bullet)"),
			Probe.NumPoints, Probe.HeightMaxX, Probe.HeightMaxY, Probe.FrictionA, Probe.FrictionB));
	}
	if (Cross.Extra.Num() > 0)
	{
		const double Peak = FMath::Max(Cross.ExtraPeakA[0], 1e-6);
		AddCheck(R, TEXT("speed, RMS error / peak speed"), Cross.Extra[0].Rms / Peak, Criteria.VehicleSpeedRmsFraction, TEXT(""));
	}

	R.Notes.Add(FString::Printf(TEXT("path length of the chassis %.1f m, peak speed %.1f m/s (%.0f km/h), limit = %.2f m + %.0f%% of the path"), Cross.PathLengthA,
		Cross.ExtraPeakA.IsValidIndex(0) ? Cross.ExtraPeakA[0] : 0.0, Cross.ExtraPeakA.IsValidIndex(0) ? Cross.ExtraPeakA[0] * 3.6 : 0.0,
		Criteria.VehiclePositionBaseM, Criteria.VehiclePositionPathFraction * 100.0));
	R.Notes.Add(FString::Printf(TEXT("chassis position error: RMS %s m, final %s m; first 5 cm divergence at %s s"), *Sci(Cross.BodyPosition[0].Rms), *Sci(Cross.BodyPosition[0].Final),
		Cross.TimeToDivergence5cm < 0 ? TEXT("never") : *FString::Printf(TEXT("%.2f"), Cross.TimeToDivergence5cm)));
	if (Cross.Extra.Num() > 2)
	{
		R.Notes.Add(FString::Printf(TEXT("engine speed difference: RMS %.0f rpm (peak %.0f rpm); gear difference RMS %.2f"), Cross.Extra[1].Rms, Cross.ExtraPeakA[1], Cross.Extra[2].Rms));
	}

	// Noise floor
	const auto Repeatability = [&](FRunSpec* A, FRunSpec* B, const TCHAR* Name)
	{
		if (!A || !B || !A->Recording->bDone || !B->Recording->bDone)
		{
			return;
		}
		const FTrajectoryComparison Cmp = CompareTrajectories(FString::Printf(TEXT("%s: %s repeat"), *Title, Name), *A->Recording, *B->Recording);
		Trajectories.Add(Cmp);
		if (Cmp.bValid)
		{
			AddCheck(R, FString::Printf(TEXT("%s repeatability, max position difference"), Name), Cmp.Position.Max, Criteria.RepeatabilityMaxPositionM, TEXT("m"), false);
		}
	};
	Repeatability(Chaos, Chaos2, TEXT("Chaos"));
	Repeatability(Bullet, Bullet2, TEXT("Bullet"));
	return R;
}

AChronoBackendComparison::FScenarioResult AChronoBackendComparison::EvaluateRigid(const FString& Scenario, const FString& Title, EKind Kind)
{
	FScenarioResult R;
	R.Name = Title;

	FRunSpec* Chaos = FindRun(Scenario, EChronoCollisionBackend::Chaos, 0);
	FRunSpec* Bullet = FindRun(Scenario, EChronoCollisionBackend::Bullet, 0);
	FRunSpec* Chaos2 = FindRun(Scenario, EChronoCollisionBackend::Chaos, 1);
	FRunSpec* Bullet2 = FindRun(Scenario, EChronoCollisionBackend::Bullet, 1);

	const auto Completed = [&](FRunSpec* Run, const TCHAR* Name) -> bool
	{
		const bool bOk = Run && Run->Recording && Run->Recording->bDone && !Run->bTimedOut && Run->Recording->NumSamples() > 10;
		FCheck C;
		C.Name = FString::Printf(TEXT("%s run completed"), Name);
		C.Value = bOk ? FString::Printf(TEXT("%d samples, %.1f s wall"), Run->Recording->NumSamples(), Run->WallSeconds) : TEXT("incomplete");
		C.Limit = TEXT("completes");
		C.bPass = bOk;
		R.Checks.Add(C);
		return bOk;
	};
	if (!Completed(Chaos, TEXT("Chaos")) | !Completed(Bullet, TEXT("Bullet")))
	{
		return R;
	}

	const FRunRecording& RA = *Chaos->Recording;
	const FRunRecording& RB = *Bullet->Recording;
	const FTrajectoryComparison Cross = CompareTrajectories(Title + TEXT(": Chaos vs Bullet"), RA, RB);
	Trajectories.Add(Cross);
	if (!Cross.bValid)
	{
		FCheck C; C.Name = TEXT("recordings comparable"); C.Value = Cross.Error; C.Limit = TEXT("same time grid"); C.bPass = false; R.Checks.Add(C);
		return R;
	}

	const FAggregateSeries AggA = ComputeAggregate(RA), AggB = ComputeAggregate(RB);
	const FAggregateComparison Agg = CompareAggregates(AggA, AggB, RA.NumBodies(), RA, RB);

	const bool bPile = Kind == EKind::Pile;
	if (Kind == EKind::Roll)
	{
		// Distance rolled by every body (start to end, Chrono metres) and the relative difference between the backends
		double CylinderMax = 0.0, SphereMax = 0.0;
		const int32 Last = FMath::Min(RA.NumSamples(), RB.NumSamples()) - 1;
		for (int32 b = 0; b < RA.NumBodies(); ++b)
		{
			const auto Travel = [&](const FRunRecording& Rec)
			{
				const double* P0 = Rec.GetBody(0, b);
				const double* P1 = Rec.GetBody(Last, b);
				return FMath::Sqrt(FMath::Square(P1[0] - P0[0]) + FMath::Square(P1[1] - P0[1]) + FMath::Square(P1[2] - P0[2]));
			};
			const double DA = Travel(RA), DB = Travel(RB);
			const double Rel = FMath::Abs(DA - DB) / FMath::Max(DA, 1e-6);
			const bool bCylinder = RA.BodyNames[b].StartsWith(TEXT("cylinder"));
			double& Max = bCylinder ? CylinderMax : SphereMax;
			Max = FMath::Max(Max, Rel);
			R.Notes.Add(FString::Printf(TEXT("%s: rolled %.3f m (Chaos) vs %.3f m (Bullet), %.2f%% difference"), *RA.BodyNames[b], DA, DB, Rel * 100.0));
		}
		AddCheck(R, TEXT("cylinders: distance rolled, max relative difference"), CylinderMax, Criteria.RollDistanceFraction, TEXT(""));
		AddCheck(R, TEXT("spheres: distance rolled, max relative difference"), SphereMax, Criteria.RollDistanceFraction, TEXT(""));
	}
	else if (Kind == EKind::Pairs || Kind == EKind::Stack || Kind == EKind::Slide || Kind == EKind::Bounce)
	{
		double FinalMax = 0.0;
		for (int32 b = 0; b < Cross.BodyPosition.Num(); ++b) { FinalMax = FMath::Max(FinalMax, Cross.BodyPosition[b].Final); }
		AddCheck(R, TEXT("body positions, max error (any body, any time)"), Cross.Position.Max, Criteria.ColliderPositionMaxM, TEXT("m"));
		AddCheck(R, TEXT("final resting position, max error"), FinalMax, Criteria.ColliderFinalPositionM, TEXT("m"));
		AddCheck(R, TEXT("kinetic energy, RMS error / peak"), Agg.KineticEnergyRmsFraction, Criteria.ColliderEnergyRmsFraction, TEXT(""));
		WriteBodyTable(Scenario);
	}
	else if (!bPile)
	{
		AddCheck(R, TEXT("body positions, max error (any body, any time)"), Cross.Position.Max, Criteria.SparsePositionMaxM, TEXT("m"));
		AddCheck(R, TEXT("kinetic energy, RMS error / peak"), Agg.KineticEnergyRmsFraction, Criteria.SparseEnergyRmsFraction, TEXT(""));
	}
	else
	{
		AddCheck(R, TEXT("kinetic energy, RMS error / peak"), Agg.KineticEnergyRmsFraction, Criteria.PileEnergyRmsFraction, TEXT(""));
		AddCheck(R, TEXT("centre of mass height, max difference"), Agg.CenterOfMassZMax, Criteria.PileCenterOfMassMaxM, TEXT("m"));
		AddCheck(R, TEXT("pile height at the end (90th percentile body), difference"), FMath::Abs(Agg.FinalHeightP90Diff), Criteria.PileFinalHeightM, TEXT("m"));
		AddCheck(R, TEXT("bodies still moving at the end, difference / all bodies"), FMath::Abs(Agg.FinalMovingA - Agg.FinalMovingB) / FMath::Max(RA.NumBodies(), 1), Criteria.PileMovingFraction, TEXT(""));
	}
	AddCheck(R, TEXT("bodies that left the arena (Chaos - Bullet)"), FMath::Abs((double)Agg.EscapedA - (double)Agg.EscapedB), 0.0, TEXT("bodies"));

	R.Notes.Add(FString::Printf(TEXT("%d bodies, %.1f s. Position error over all bodies: RMS %s m, max %s m, 5 cm divergence at %s s"), RA.NumBodies(), Cross.Duration,
		*Sci(Cross.Position.Rms), *Sci(Cross.Position.Max), Cross.TimeToDivergence5cm < 0 ? TEXT("never") : *FString::Printf(TEXT("%.2f"), Cross.TimeToDivergence5cm)));
	R.Notes.Add(FString::Printf(TEXT("settle time (fewer than 5%% of the bodies moving): Chaos %s s, Bullet %s s; escaped: Chaos %d, Bullet %d"),
		Agg.SettleTimeA < 0 ? TEXT("not settled") : *FString::Printf(TEXT("%.2f"), Agg.SettleTimeA),
		Agg.SettleTimeB < 0 ? TEXT("not settled") : *FString::Printf(TEXT("%.2f"), Agg.SettleTimeB), Agg.EscapedA, Agg.EscapedB));

	// Per shape type position error
	static const TCHAR* Shapes[] = { TEXT("box"), TEXT("sphere"), TEXT("capsule"), TEXT("cylinder"), TEXT("hull") };
	for (const TCHAR* Shape : Shapes)
	{
		double Max = 0.0, SumSq = 0.0;
		int32 Count = 0;
		for (int32 b = 0; b < Cross.BodyNames.Num(); ++b)
		{
			if (Cross.BodyNames[b].StartsWith(Shape))
			{
				Max = FMath::Max(Max, Cross.BodyPosition[b].Max);
				SumSq += Cross.BodyPosition[b].Rms * Cross.BodyPosition[b].Rms;
				++Count;
			}
		}
		if (Count > 0)
		{
			R.Notes.Add(FString::Printf(TEXT("%s (%d bodies): RMS %s m, worst max %s m"), Shape, Count, *Sci(FMath::Sqrt(SumSq / Count)), *Sci(Max)));
		}
	}

	// The bodies that differ most: where to look first
	{
		TArray<int32> Order;
		for (int32 b = 0; b < Cross.BodyNames.Num(); ++b) { Order.Add(b); }
		Order.Sort([&](int32 A, int32 B) { return Cross.BodyPosition[A].Max > Cross.BodyPosition[B].Max; });
		FString Worst = TEXT("worst bodies (max position error): ");
		for (int32 k = 0; k < FMath::Min(5, Order.Num()); ++k)
		{
			Worst += FString::Printf(TEXT("%s %s m%s"), *Cross.BodyNames[Order[k]], *Sci(Cross.BodyPosition[Order[k]].Max), k + 1 < 5 ? TEXT(", ") : TEXT(""));
		}
		R.Notes.Add(Worst);
	}

	// Aggregate time series for plotting
	{
		FString Csv = TEXT("time,KE_chaos,KE_bullet,comZ_chaos,comZ_bullet,heightP90_chaos,heightP90_bullet,moving_chaos,moving_bullet\n");
		const int32 N = FMath::Min(AggA.Times.Num(), AggB.Times.Num());
		for (int32 s = 0; s < N; ++s)
		{
			Csv += FString::Printf(TEXT("%.6f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.0f,%.0f\n"), AggA.Times[s], AggA.KineticEnergy[s], AggB.KineticEnergy[s], AggA.CenterOfMassZ[s], AggB.CenterOfMassZ[s],
				AggA.HeightP90[s], AggB.HeightP90[s], AggA.NumMoving[s], AggB.NumMoving[s]);
		}
		FFileHelper::SaveStringToFile(Csv, *(OutputDir / (Scenario + TEXT("_aggregate.csv"))));
	}

	const auto Repeatability = [&](FRunSpec* A, FRunSpec* B, const TCHAR* Name)
	{
		if (!A || !B || !A->Recording->bDone || !B->Recording->bDone)
		{
			return;
		}
		const FTrajectoryComparison Cmp = CompareTrajectories(FString::Printf(TEXT("%s: %s repeat"), *Title, Name), *A->Recording, *B->Recording);
		Trajectories.Add(Cmp);
		if (Cmp.bValid)
		{
			AddCheck(R, FString::Printf(TEXT("%s repeatability, max position difference"), Name), Cmp.Position.Max, Criteria.RepeatabilityMaxPositionM, TEXT("m"), false);
		}
	};
	Repeatability(Chaos, Chaos2, TEXT("Chaos"));
	Repeatability(Bullet, Bullet2, TEXT("Bullet"));
	return R;
}

void AChronoBackendComparison::Analyze()
{
	Results.Reset();
	Trajectories.Reset();

	// Raw recordings of the vehicle and sparse runs (a pile is 200 bodies x 13 columns: only its aggregate series is written)
	for (const FRunSpec& Run : Runs)
	{
		if (Run.Kind != EKind::Pile && Run.Recording && Run.Recording->NumSamples() > 0)
		{
			WriteRecordingCsv(OutputDir / FString::Printf(TEXT("%s_%s_%s.csv"), *Run.Scenario, BackendName(Run.Backend), Run.Repeat ? TEXT("repeat") : TEXT("run")), *Run.Recording);
		}
	}

	if (bRunVehicleTests)
	{
		for (ETireModelType Tire : TireModels)
		{
			const FString Scenario = FString::Printf(TEXT("vehicle_%s"), TireName(Tire));
			if (!HasScenario(Scenario))
			{
				continue;
			}
			Results.Add(EvaluateVehicle(Scenario, FString::Printf(TEXT("Vehicle, %s tire"), TireName(Tire))));
			FRunSpec* A = FindRun(Scenario, EChronoCollisionBackend::Chaos, 0);
			FRunSpec* B = FindRun(Scenario, EChronoCollisionBackend::Bullet, 0);
			if (A && B && A->Recording->NumSamples() > 0 && B->Recording->NumSamples() > 0)
			{
				WriteComparisonCsv(OutputDir / (Scenario + TEXT("_error.csv")), *A->Recording, *B->Recording);
			}
			if (A && B && A->Recording->TerrainProbe.Num() == B->Recording->TerrainProbe.Num())
			{
				FString Csv = TEXT("x,y,hit_chaos,hit_bullet,height_chaos,height_bullet,normal_chaos,normal_bullet,friction_chaos,friction_bullet,type_chaos,type_bullet\n");
				for (int32 i = 0; i < A->Recording->TerrainProbe.Num() / TerrainProbeStride; ++i)
				{
					const double* PA = &A->Recording->TerrainProbe[i * TerrainProbeStride];
					const double* PB = &B->Recording->TerrainProbe[i * TerrainProbeStride];
					Csv += FString::Printf(TEXT("%.3f,%.3f,%.0f,%.0f,%.5f,%.5f,\"%.3f %.3f %.3f\",\"%.3f %.3f %.3f\",%.3f,%.3f,%.0f,%.0f\n"), PA[0], PA[1], PA[2], PB[2], PA[3], PB[3],
						PA[4], PA[5], PA[6], PB[4], PB[5], PB[6], PA[7], PB[7], PA[8], PB[8]);
				}
				FFileHelper::SaveStringToFile(Csv, *(OutputDir / (Scenario + TEXT("_terrain_probe.csv"))));
			}
		}
	}
	if (bRunRigidBodyTests)
	{
			if (HasScenario(TEXT("rigid_roll"))) { Results.Add(EvaluateRigid(TEXT("rigid_roll"), TEXT("Rigid bodies, rolling cylinders and spheres (10 bodies)"), EKind::Roll)); }
			if (HasScenario(TEXT("rigid_sparse"))) { Results.Add(EvaluateRigid(TEXT("rigid_sparse"), TEXT("Rigid bodies, sparse drop (36 bodies)"), EKind::Sparse)); }
			if (HasScenario(TEXT("rigid_pile"))) { Results.Add(EvaluateRigid(TEXT("rigid_pile"), TEXT("Rigid bodies, pile (200 bodies)"), EKind::Pile)); }
			for (const FString& Scenario : { FString(TEXT("rigid_roll")), FString(TEXT("rigid_sparse")), FString(TEXT("rigid_pile")) })
			{
				if (!HasScenario(Scenario))
				{
					continue;
				}
				FRunSpec* A = FindRun(Scenario, EChronoCollisionBackend::Chaos, 0);
				FRunSpec* B = FindRun(Scenario, EChronoCollisionBackend::Bullet, 0);
				if (A && B && A->Recording->NumSamples() > 0 && B->Recording->NumSamples() > 0)
				{
					WriteComparisonCsv(OutputDir / (Scenario + TEXT("_error.csv")), *A->Recording, *B->Recording);
				}
			}
		}
	if (bRunColliderTests)
	{
		struct FCol { const TCHAR* Scenario; const TCHAR* Title; EKind Kind; };
		const FCol Cols[] = {
			{ TEXT("col_pairs"),  TEXT("Colliders, every shape on every collider (30 bodies)"), EKind::Pairs },
			{ TEXT("col_stack"),  TEXT("Colliders, stacks of boxes and cylinders (18 bodies)"), EKind::Stack },
			{ TEXT("col_slide"),  TEXT("Colliders, sliding and rolling down a ramp (5 bodies)"), EKind::Slide },
			{ TEXT("col_bounce"), TEXT("Colliders, bounce with restitution 0.6 (7 bodies)"), EKind::Bounce },
		};
		for (const FCol& Col : Cols)
		{
			if (!HasScenario(Col.Scenario))
			{
				continue;
			}
			Results.Add(EvaluateRigid(Col.Scenario, Col.Title, Col.Kind));
			FRunSpec* A = FindRun(Col.Scenario, EChronoCollisionBackend::Chaos, 0);
			FRunSpec* B = FindRun(Col.Scenario, EChronoCollisionBackend::Bullet, 0);
			if (A && B && A->Recording->NumSamples() > 0 && B->Recording->NumSamples() > 0)
			{
				WriteComparisonCsv(OutputDir / (Col.Scenario + FString(TEXT("_error.csv"))), *A->Recording, *B->Recording);
			}
		}
	}

	bAllPassed = true;
	for (const FScenarioResult& R : Results)
	{
		bAllPassed &= R.bPass();
	}
}

void AChronoBackendComparison::WriteReport()
{
	FString Md;
	Md += TEXT("# Bullet vs Chaos backend comparison\n\n");
	Md += FString::Printf(TEXT("Run on %s, engine %s. Chrono step %.4f ms, solver iterations 140, NSC contacts, async physics %s.\n\n"), *FDateTime::Now().ToString(), *FEngineVersion::Current().ToString(),
		1000.0 / 360.0, UPhysicsSettings::Get()->bTickPhysicsAsync ? TEXT("on") : TEXT("off"));
	Md += TEXT("Method: every scenario is simulated with identical inputs on the Chaos and on the Bullet collision backend (inputs are a function of simulation time, applied on the physics thread; the rigid body scenes are generated from a fixed seed). The state of every body is recorded at each Chrono substep and the two runs are compared on the same time grid. Each backend is also run twice to measure its repeatability (the noise floor of the comparison).\n\n");
	Md += FString::Printf(TEXT("**Overall: %s**\n\n"), bAllPassed ? TEXT("PASS") : TEXT("FAIL"));

	Md += TEXT("| Scenario | Result |\n|---|---|\n");
	for (const FScenarioResult& R : Results)
	{
		Md += FString::Printf(TEXT("| %s | %s |\n"), *R.Name, R.bPass() ? TEXT("PASS") : TEXT("FAIL"));
	}
	Md += TEXT("\n");

	FString Json = TEXT("{\n  \"overall\": ") + FString(bAllPassed ? TEXT("\"pass\"") : TEXT("\"fail\"")) + TEXT(",\n  \"scenarios\": [\n");
	for (int32 i = 0; i < Results.Num(); ++i)
	{
		const FScenarioResult& R = Results[i];
		Md += FString::Printf(TEXT("## %s: %s\n\n| Check | Value | Limit | Result |\n|---|---|---|---|\n"), *R.Name, R.bPass() ? TEXT("PASS") : TEXT("FAIL"));
		Json += FString::Printf(TEXT("    { \"name\": \"%s\", \"pass\": %s, \"checks\": [\n"), *R.Name, R.bPass() ? TEXT("true") : TEXT("false"));
		for (int32 c = 0; c < R.Checks.Num(); ++c)
		{
			const FCheck& C = R.Checks[c];
			Md += FString::Printf(TEXT("| %s%s | %s | %s | %s |\n"), *C.Name, C.bRequired ? TEXT("") : TEXT(" (info)"), *C.Value, *C.Limit, C.bPass ? TEXT("pass") : (C.bRequired ? TEXT("**FAIL**") : TEXT("warn")));
			Json += FString::Printf(TEXT("      { \"check\": \"%s\", \"value\": \"%s\", \"limit\": \"%s\", \"required\": %s, \"pass\": %s }%s\n"), *C.Name, *C.Value, *C.Limit,
				C.bRequired ? TEXT("true") : TEXT("false"), C.bPass ? TEXT("true") : TEXT("false"), c + 1 < R.Checks.Num() ? TEXT(",") : TEXT(""));
		}
		Json += FString::Printf(TEXT("    ] }%s\n"), i + 1 < Results.Num() ? TEXT(",") : TEXT(""));
		Md += TEXT("\n");
		for (const FString& Note : R.Notes)
		{
			Md += TEXT("- ") + Note + TEXT("\n");
		}
		Md += TEXT("\n");
	}
	Json += TEXT("  ]\n}\n");

	Md += TEXT("## Reading the results\n\n- *Repeatability* rows are the noise floor: the same backend run twice. A backend that is bit-for-bit repeatable shows 0; anything larger means differences below that size cannot be attributed to the backend.\n");
	Md += TEXT("- Trajectories of a dense pile diverge chaotically for any two solvers that differ in the last bit, so the pile is judged on scene level quantities (kinetic energy, centre of mass, height, settling), not on individual positions.\n");
	Md += TEXT("- Known modelling differences (CHAOS_COVERAGE.md): Bullet inflates shapes by the collision envelope and uses an analytic cylinder, the Chaos backend a 24 segment prism; the Bullet terrain reports every surface as asphalt.\n");
	Md += FString::Printf(TEXT("\nFiles: `%s`\n"), *OutputDir);

	ReportText = Md;
	FFileHelper::SaveStringToFile(Md, *(OutputDir / TEXT("report.md")));
	FFileHelper::SaveStringToFile(Json, *(OutputDir / TEXT("summary.json")));

	UE_LOG(LogBackendCompare, Display, TEXT("\n%s"), *Md);
	UE_LOG(LogBackendCompare, Display, TEXT("BACKEND COMPARISON %s (report: %s)"), bAllPassed ? TEXT("PASSED") : TEXT("FAILED"), *(OutputDir / TEXT("report.md")));

	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(7701, 600.f, bAllPassed ? FColor::Green : FColor::Red, FString::Printf(TEXT("Backend comparison finished: %s"), bAllPassed ? TEXT("PASS") : TEXT("FAIL")));
		int32 Key = 7702;
		for (const FScenarioResult& R : Results)
		{
			GEngine->AddOnScreenDebugMessage(Key++, 600.f, R.bPass() ? FColor::Green : FColor::Red, FString::Printf(TEXT("  %s: %s"), *R.Name, R.bPass() ? TEXT("PASS") : TEXT("FAIL")));
		}
		GEngine->AddOnScreenDebugMessage(Key, 600.f, FColor::White, FString::Printf(TEXT("  report: %s"), *(OutputDir / TEXT("report.md"))));
	}

	if (bQuitWhenDone || FParse::Param(FCommandLine::Get(), TEXT("ChronoCompareQuit")))
	{
		FPlatformMisc::RequestExitWithStatus(false, bAllPassed ? 0 : 1);
	}
}

/// One row per body: final pose, highest point after the start and distance travelled on both backends
void AChronoBackendComparison::WriteBodyTable(const FString& Scenario)
{
	FRunSpec* A = FindRun(Scenario, EChronoCollisionBackend::Chaos, 0);
	FRunSpec* B = FindRun(Scenario, EChronoCollisionBackend::Bullet, 0);
	if (!A || !B || !A->Recording || !B->Recording || A->Recording->NumSamples() < 2 || B->Recording->NumSamples() < 2)
	{
		return;
	}
	const FRunRecording& RA = *A->Recording;
	const FRunRecording& RB = *B->Recording;
	const int32 LastA = RA.NumSamples() - 1, LastB = RB.NumSamples() - 1;
	FString Csv = TEXT("body,final_chaos_x,final_chaos_y,final_chaos_z,final_bullet_x,final_bullet_y,final_bullet_z,final_error,apex_z_chaos,apex_z_bullet,travel_chaos,travel_bullet\n");
	for (int32 b = 0; b < RA.NumBodies(); ++b)
	{
		const auto Apex = [b](const FRunRecording& R)
		{
			double Max = -1e9;
			for (int32 s = 0; s < R.NumSamples(); ++s) { Max = FMath::Max(Max, R.GetBody(s, b)[2]); }
			return Max;
		};
		const auto Travel = [b](const FRunRecording& R)
		{
			const double* P0 = R.GetBody(0, b);
			const double* P1 = R.GetBody(R.NumSamples() - 1, b);
			return FMath::Sqrt(FMath::Square(P1[0] - P0[0]) + FMath::Square(P1[1] - P0[1]) + FMath::Square(P1[2] - P0[2]));
		};
		const double* FA = RA.GetBody(LastA, b);
		const double* FB = RB.GetBody(LastB, b);
		const double Err = FMath::Sqrt(FMath::Square(FA[0] - FB[0]) + FMath::Square(FA[1] - FB[1]) + FMath::Square(FA[2] - FB[2]));
		Csv += FString::Printf(TEXT("%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"), *RA.BodyNames[b], FA[0], FA[1], FA[2], FB[0], FB[1], FB[2], Err, Apex(RA), Apex(RB), Travel(RA), Travel(RB));
	}
	FFileHelper::SaveStringToFile(Csv, *(OutputDir / (Scenario + TEXT("_bodies.csv"))));
}
