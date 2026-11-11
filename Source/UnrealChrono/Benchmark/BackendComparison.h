#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ChronoSimulation.h"
#include "VehicleCommon.h"
#include "BackendComparisonTypes.h"
#include "RigidBodyBenchmark.h"
#include "BackendComparison.generated.h"

class ACarPhysicsWorld;
class APhysicsVehicle;
class ACameraActor;

/// Tolerances of the comparison. They are deliberately written down in one place: a result is only meaningful against
/// an explicit pass criterion. The defaults come from the noise floor the test measures itself (the repeat runs) plus a
/// margin for the genuine modelling differences between the backends (see README, "Backend comparison").
USTRUCT(BlueprintType)
struct FBackendComparisonCriteria
{
	GENERATED_BODY()

	/// Same backend, same inputs, run twice: the largest allowed position difference (m). A deterministic backend gives 0.
	UPROPERTY(EditAnywhere, Category = "Determinism")
	double RepeatabilityMaxPositionM = 1e-3;

	/// Vehicle: chassis / wheel position error limit = Base + Fraction * path length of the chassis
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double VehiclePositionBaseM = 0.05;
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double VehiclePositionPathFraction = 0.02;
	/// Vehicle: largest chassis orientation difference (degrees)
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double VehicleRotationMaxDeg = 3.0;
	/// Vehicle: RMS speed difference as a fraction of the peak speed
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double VehicleSpeedRmsFraction = 0.03;

	/// Terrain query of the tires: largest friction / height difference between the backends
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double TerrainFrictionMax = 0.05;
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double TerrainHeightMaxM = 0.01;
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	double TerrainNormalMaxDeg = 2.0;

	/// Sparse bodies (independent bounces): largest position difference of any body at any time (m)
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double SparsePositionMaxM = 0.15;
	/// Rolling cylinders / spheres: largest relative difference of the distance rolled
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double RollDistanceFraction = 0.02;
	/// Kinetic energy RMS difference as a fraction of the peak energy
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double SparseEnergyRmsFraction = 0.10;
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double PileEnergyRmsFraction = 0.35;
	/// Pile: largest centre of mass height difference (m) and final pile height difference (m, 90th percentile body: a single
	/// perched body would make a maximum meaningless in a chaotic stack)
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double PileCenterOfMassMaxM = 0.10;
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double PileFinalHeightM = 0.15;
	/// Pile: difference of the number of bodies still moving at the end, as a fraction of all bodies
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	double PileMovingFraction = 0.10;

	/// Collider scenes: largest position difference of any body at any time (m), largest difference of the final resting
	/// position (m), kinetic energy RMS difference as a fraction of the peak
	UPROPERTY(EditAnywhere, Category = "Colliders")
	double ColliderPositionMaxM = 0.15;
	UPROPERTY(EditAnywhere, Category = "Colliders")
	double ColliderFinalPositionM = 0.05;
	UPROPERTY(EditAnywhere, Category = "Colliders")
	double ColliderEnergyRmsFraction = 0.10;
};

/// Runs the same scenarios on the Bullet and on the Chaos collision backend and compares the outputs.
///
/// Place one in a level (Maps/BackendComparison) and press Play. The actor runs, one after the other:
///   * the test car driven by a script (a function of simulation time, applied on the physics thread) over identical level
///     geometry, once per tire model;
///   * Chrono-only rigid body scenes: a sparse drop of 36 mixed shapes and a pile of 200 bodies in a walled container.
/// Every scenario is simulated with Chaos, with Bullet and, to measure the noise floor, a second time with each backend.
/// Each run records the state of its bodies at every Chrono substep; the runs are then compared on the same time grid.
/// Results go to the screen, the log and Saved/BackendComparison/<timestamp>/ (report.md, summary.json, csv).
UCLASS()
class UNREALCHRONO_API AChronoBackendComparison : public AActor
{
	GENERATED_BODY()

public:
	AChronoBackendComparison();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	UPROPERTY(EditAnywhere, Category = "Comparison")
	bool bRunVehicleTests = true;
	UPROPERTY(EditAnywhere, Category = "Comparison")
	bool bRunRigidBodyTests = true;
	/// Collider scenes (shape pairs, stacks, ramp slide, bounce): Chrono-only rigid shapes against every kind of collider.
	/// Used by the RigidColliders level; command line: -ChronoCompareColliders (alone: -ChronoCompareCollidersOnly)
	UPROPERTY(EditAnywhere, Category = "Comparison")
	bool bRunColliderTests = false;
	/// Also run every scenario a second time per backend to measure repeatability (doubles the run time)
	UPROPERTY(EditAnywhere, Category = "Comparison")
	bool bMeasureRepeatability = true;
	/// Exit the game when finished; the exit code is 0 when every required check passed, 1 otherwise
	UPROPERTY(EditAnywhere, Category = "Comparison")
	bool bQuitWhenDone = false;
	/// Tire models of the vehicle scenarios
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	TArray<ETireModelType> TireModels = { ETireModelType::DISCRETE, ETireModelType::TMEASY, ETireModelType::RIGID };
	/// Length of a vehicle run (simulation seconds)
	UPROPERTY(EditAnywhere, Category = "Vehicle", meta = (ClampMin = "3.0"))
	float VehicleDuration = 9.5f;
	/// Where the vehicle starts (cm); the level geometry it drives over is placed around it
	UPROPERTY(EditAnywhere, Category = "Vehicle")
	FVector VehicleStart = FVector(0.0, 0.0, 15.0);
	/// Centre of the rigid body arena (cm). Chrono only: the level geometry is ignored there.
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	FVector ArenaCenter = FVector(0.0, 12000.0, 0.0);
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	int32 Seed = 12345;
	/// Chaos backend: proxies are processed in parallel above this many proxies (plugin default 8). Set very high for a serial
	/// narrowphase, e.g. to test whether the contact order is the source of run to run differences.
	UPROPERTY(EditAnywhere, Category = "Rigid bodies")
	int32 ChaosParallelThreshold = 8;
	UPROPERTY(EditAnywhere, Category = "Comparison")
	FBackendComparisonCriteria Criteria;

private:
	enum class EKind : uint8 { Vehicle, Sparse, Roll, Pile, Pairs, Stack, Slide, Bounce };
	enum class EPhase : uint8 { Idle, Running, Cooldown, Finished };

	struct FRunSpec
	{
		EKind Kind = EKind::Vehicle;
		ETireModelType Tire = ETireModelType::DISCRETE;
		EChronoCollisionBackend Backend = EChronoCollisionBackend::Chaos;
		int32 Repeat = 0;
		FString Scenario;
		TSharedPtr<BackendCompare::FRunRecording> Recording;
		double WallSeconds = 0.0;
		bool bTimedOut = false;
	};

	struct FCheck
	{
		FString Name;
		FString Value;
		FString Limit;
		bool bRequired = true;
		bool bPass = false;
	};

	struct FScenarioResult
	{
		FString Name;
		TArray<FCheck> Checks;
		TArray<FString> Notes;
		bool bPass() const
		{
			for (const FCheck& C : Checks) { if (C.bRequired && !C.bPass) return false; }
			return true;
		}
	};

	void BuildRunList();
	void StartRun(FRunSpec& Run);
	void StopRun();
	void Analyze();
	void WriteReport();
	void ShowStatus(const FString& Text, const FColor& Color) const;
	void DrawRigidBodies() const;
	void WriteBodyTable(const FString& Scenario);

	FRunSpec* FindRun(const FString& Scenario, EChronoCollisionBackend Backend, int32 Repeat);
	void AddCheck(FScenarioResult& R, const FString& Name, double Value, double Limit, const TCHAR* Unit, bool bRequired = true);
	FScenarioResult EvaluateVehicle(const FString& Scenario, const FString& Title);
	FScenarioResult EvaluateRigid(const FString& Scenario, const FString& Title, EKind Kind);

	/// Only scenarios whose name contains this text run (command line: -ChronoCompareOnly=roll); empty = all
	FString ScenarioFilter;
	bool HasScenario(const FString& Scenario) const;

	TArray<FRunSpec> Runs;
	int32 CurrentRun = -1;
	EPhase Phase = EPhase::Idle;
	int32 CooldownFrames = 0;
	double RunWallStart = 0.0;
	FString OutputDir;

	// Current run
	UPROPERTY(Transient)
	TObjectPtr<ACarPhysicsWorld> PhysicsWorld;
	UPROPERTY(Transient)
	TObjectPtr<APhysicsVehicle> Vehicle;
	UPROPERTY(Transient)
	TObjectPtr<ACameraActor> ArenaCamera;
	TSharedPtr<FChronoSimulation> RigidSimulation;
	TArray<FChronoBodyId> RigidBodyIds;
	BackendCompare::FBenchScene RigidScene;

	TArray<FScenarioResult> Results;
	TArray<BackendCompare::FTrajectoryComparison> Trajectories;
	FString ReportText;
	bool bAllPassed = false;
};
