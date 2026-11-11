// FChronoSimulation - hosts a chrono::ChSystem on Unreal's Chaos physics thread.
//
// The whole Chrono system (bodies, solver, collision detection) lives inside a Chaos sim callback
// (Chaos::TSimCallbackObject) registered on the solver of a FChaosScene, exactly like Epic's ChaosVehicles plugin.
// The game thread never touches the ChSystem; it talks to it through this facade:
//
//   commands in : Enqueue(TUniqueFunction<void(FChronoCommandContext&)>) - one-shot, run on the physics thread before
//                 the next physics step (create bodies, apply forces, query the collision system, ...);
//   inputs in   : PushInputs(FChronoTickInputs) - idempotent per-frame values (driver controls) marshalled through
//                 Chaos' FSimCallbackInput double buffering;
//   snapshots out: PumpOutputs() drains the physics thread's FSimCallbackOutputs and keeps the newest FChronoSnapshot
//                 (per-body transforms / velocities in Unreal units, sim time, contact count, optional contact debug).
//
// THREADING CONTRACT
//   * Create / Enqueue / PushInputs / PumpOutputs / GetLatestSnapshot / destructor: game thread only.
//   * Commands, FChronoSimulationParams::ConfigureSystem and everything inside the ChSystem: physics thread. They must
//     not touch UObjects, actors or Unreal scene APIs - only Chrono. Exceptions thrown by commands are caught and
//     logged; the simulation continues.
//   * The game thread never blocks on Chrono (no mutex around the ChSystem). With Project Settings > Physics >
//     "Tick Physics Async" the physics thread runs at a fixed rate independent of the frame rate; without it the sim
//     callback still runs on the physics task thread while the game thread waits at the end of the frame.
//   * Destroy the FChronoSimulation (Reset the shared pointer) BEFORE the world's physics scene goes away (e.g. in
//     AActor::EndPlay). The destructor unregisters the callback; Chaos then destroys the ChSystem on the physics
//     thread while the solver is still alive.
//
// Every fixed physics step the callback: drains the command queue -> copies the consumed inputs -> advances Chrono by
// GetDeltaTime_Internal() in fixed StepSizeS substeps (accumulator, capped by MaxSubStepsPerTick) -> pushes the
// Chrono bodies' poses to their Chaos proxy particles (CHAOS backend) -> writes one snapshot.
//
// STEPPER HOOK
//   By default every substep is a plain chrono::ChSystem::DoStepDynamics(StepSizeS). Systems that need per-substep
//   work around the integration (e.g. chrono::vehicle - driver / terrain / vehicle Synchronize + Advance) install an
//   IChronoStepper from a command (FChronoCommandContext::SetStepper). The stepper then owns the substep and may attach
//   a typed, immutable payload (FChronoSnapshotPayload) to every snapshot so game code can publish its own data without
//   touching this struct.
//
// This header only forward-declares Chaos / Chrono types so that it can be included from any module.

#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "Templates/SharedPointer.h"

#include "ChronoChaosCollision.h"

#include <atomic>
#include <memory>

namespace chrono
{
	class ChSystem;
	class ChBody;
}

namespace Chaos
{
	class FPhysicsSolverBase;
}

class FChaosScene;
class FChronoSimCallback;

/// Collision detection backend of the hosted ChSystem.
enum class EChronoCollisionBackend : uint8
{
	/// Chrono's bundled Bullet backend. Chrono bodies collide only with other Chrono bodies.
	Bullet,
	/// Unreal Chaos as external collision provider (chrono::ChCollisionSystem::Type::CHAOS). Chrono bodies are
	/// mirrored into the Chaos scene and collide with each other and with the Unreal level.
	Chaos,
};

/// Identifies a tracked Chrono body across the game / physics thread boundary. Allocate with
/// FChronoSimulation::AllocateBodyId(), hand it to FChronoCommandContext::TrackBody() on the physics thread and look
/// the body up in snapshots with FChronoSnapshot::FindBody().
struct CHRONOCORE_API FChronoBodyId
{
	uint32 Value = 0;

	bool IsValid() const { return Value != 0; }

	friend bool operator==(const FChronoBodyId& A, const FChronoBodyId& B) { return A.Value == B.Value; }
	friend bool operator!=(const FChronoBodyId& A, const FChronoBodyId& B) { return A.Value != B.Value; }
	friend bool operator<(const FChronoBodyId& A, const FChronoBodyId& B) { return A.Value < B.Value; }
	friend uint32 GetTypeHash(const FChronoBodyId& Id) { return ::GetTypeHash(Id.Value); }
};

/// Creation parameters of a simulation.
struct CHRONOCORE_API FChronoSimulationParams
{
	/// Chaos scene whose solver hosts the simulation (World->GetPhysicsScene()). Required.
	FChaosScene* Scene = nullptr;

	/// Collision detection backend.
	EChronoCollisionBackend Backend = EChronoCollisionBackend::Chaos;

	/// Create a chrono::ChSystemSMC (penalty contacts) instead of a chrono::ChSystemNSC (complementarity contacts).
	bool bSMC = false;

	/// Fixed Chrono integration step in seconds.
	double StepSizeS = 1e-3;

	/// Upper bound on the number of Chrono substeps per physics step. Backlog beyond this is dropped (and logged
	/// once) instead of spiralling.
	int32 MaxSubStepsPerTick = 32;

	/// Gravity in Unreal units (cm/s^2, Unreal frame); converted to Chrono.
	FVector GravityUE = FVector(0.0, 0.0, -981.0);

	/// Maximum iterations of Chrono's iterative solver (PSOR).
	int32 SolverMaxIterations = 50;

	/// Fill FChronoSnapshot::Contacts every step (debug drawing). Costs one contact report per step.
	bool bCaptureContacts = false;

	/// Chaos backend binding parameters (ignored for Bullet).
	FChronoChaosBindParams Collision;

	/// Optional hook run once on the physics thread right after the ChSystem is constructed and its collision
	/// system type is set (before any body exists). Use it for solver / timestepper settings. Must not touch UObjects.
	TFunction<void(chrono::ChSystem&)> ConfigureSystem;
};

/// Idempotent per-frame inputs pushed by the game thread and visible to commands as FChronoCommandContext::Inputs.
/// Chaos may hand the same inputs to several physics steps (or none, when the game thread is slower than the fixed
/// physics rate), so they must describe a state ("throttle is 0.7"), never an event ("throttle was pressed").
struct CHRONOCORE_API FChronoTickInputs
{
	/// Flags bit: the simulation is paused. The callback skips stepping (and drops the accumulated time so resuming
	/// does not jump) but still publishes a snapshot every physics step.
	static constexpr uint32 Flag_Paused = 1u << 0;
	/// First flag bit free for user-defined flags.
	static constexpr uint32 Flag_UserFirst = 1u << 8;

	float Throttle = 0.0f;
	float Steering = 0.0f;
	float Braking = 0.0f;
	float Clutch = 0.0f;
	uint32 Flags = 0;
	TArray<float> Custom;

	bool IsPaused() const { return (Flags & Flag_Paused) != 0; }
};

/// Base of the typed data an IChronoStepper attaches to a snapshot (FChronoSnapshot::Payload). Payloads are built on
/// the physics thread, published as immutable shared objects and read on the game thread.
struct CHRONOCORE_API FChronoSnapshotPayload
{
	virtual ~FChronoSnapshotPayload() = default;
};

/// Per-substep context handed to an IChronoStepper (physics thread).
struct CHRONOCORE_API FChronoStepContext
{
	/// The hosted Chrono system.
	chrono::ChSystem& System;
	/// Latest inputs consumed by the physics thread.
	const FChronoTickInputs& Inputs;
	/// Chrono simulation time at the beginning of the substep (seconds).
	double Time = 0.0;
	/// Fixed substep size (FChronoSimulationParams::StepSizeS).
	double Dt = 0.0;
	/// Index of the substep within the current physics step (0 based).
	int32 SubStep = 0;
	/// Chaos solver time of the current physics step (seconds).
	double ChaosSimTime = 0.0;
};

/// Pluggable substep of the hosted system. Installed from a command through FChronoCommandContext::SetStepper and
/// invoked on the physics thread only. Must not touch UObjects.
class CHRONOCORE_API IChronoStepper
{
public:
	virtual ~IChronoStepper() = default;

	/// Called once per physics step before the first substep (only when at least one substep will run).
	virtual void PreStep(FChronoStepContext& Context) {}

	/// Advance the system by Context.Dt. Implementations must step the system themselves (System.DoStepDynamics(Dt)
	/// or an equivalent that steps it exactly once).
	virtual void SubStep(FChronoStepContext& Context) = 0;

	/// Called once per physics step (even when paused) after the substeps; the returned payload is attached to the
	/// snapshot of this step. Return null for no payload.
	virtual TSharedPtr<const FChronoSnapshotPayload> MakePayload(const chrono::ChSystem& System) { return nullptr; }
};

/// State of one tracked body in Unreal units / frame.
struct CHRONOCORE_API FChronoBodyState
{
	FChronoBodyId Id;
	/// Body reference frame (chrono::ChBody::GetFrameRefToAbs), Unreal cm / left-handed.
	FTransform Transform = FTransform::Identity;
	/// Linear velocity of the body reference frame origin (cm/s).
	FVector LinearVelocity = FVector::ZeroVector;
	/// Angular velocity expressed in the parent (world) frame (rad/s, Unreal frame).
	FVector AngularVelocity = FVector::ZeroVector;
};

/// One contact point for debug drawing (FChronoSimulationParams::bCaptureContacts).
struct CHRONOCORE_API FChronoContactDebug
{
	/// Contact point on body A (Unreal cm).
	FVector Point = FVector::ZeroVector;
	/// Contact normal (Unreal frame, unit length).
	FVector Normal = FVector::UpVector;
	/// Signed distance in metres (negative = penetration).
	double Distance = 0.0;
};

/// Result of one physics step of the hosted ChSystem.
struct CHRONOCORE_API FChronoSnapshot
{
	/// Chrono simulation time (chrono::ChSystem::GetChTime) in seconds.
	double SimTime = 0.0;

	/// Chaos solver time at which the step began (Chaos::ISimCallbackObject::GetSimTime_Internal).
	double ChaosSimTime = 0.0;

	/// Total number of Chrono substeps performed so far (monotonic).
	int32 StepIndex = 0;

	/// Number of Chrono substeps performed in the physics step that produced this snapshot.
	int32 SubStepsThisTick = 0;

	/// Total simulated time dropped so far because MaxSubStepsPerTick was exceeded (seconds).
	double DroppedTimeS = 0.0;

	/// Number of active contacts after the last substep.
	int32 NumContacts = 0;

	/// Collision detection timers of the last substep (seconds).
	double CollisionBroadTimeS = 0.0;
	double CollisionNarrowTimeS = 0.0;

	/// Tracked bodies, sorted by Id.
	TArray<FChronoBodyState> Bodies;

	/// Contact points after the last substep (only with bCaptureContacts).
	TArray<FChronoContactDebug> Contacts;

	/// Typed data published by the installed IChronoStepper (null without a stepper / payload).
	TSharedPtr<const FChronoSnapshotPayload> Payload;

	/// Binary search for a tracked body; null if it is not part of this snapshot.
	const FChronoBodyState* FindBody(FChronoBodyId Id) const;

	/// Payload cast helper: the payload as T, or null if absent / of another type.
	template <typename T>
	const T* GetPayload() const { return static_cast<const T*>(Payload.Get()); }
};

/// Physics-thread context handed to commands (see FChronoSimulation::Enqueue).
struct CHRONOCORE_API FChronoCommandContext
{
	/// The hosted Chrono system.
	chrono::ChSystem& System;

	/// Latest inputs consumed by the physics thread.
	const FChronoTickInputs& Inputs;

	/// Chaos solver time of the current physics step (seconds).
	const double ChaosSimTime;

	/// Publish the state of 'Body' under 'Id' in every subsequent snapshot. The body should already be part of
	/// System. Re-tracking an Id replaces the body.
	void TrackBody(FChronoBodyId Id, std::shared_ptr<chrono::ChBody> Body);

	/// Stop publishing 'Id'.
	void UntrackBody(FChronoBodyId Id);

	/// Find a tracked body (null if unknown).
	std::shared_ptr<chrono::ChBody> FindTrackedBody(FChronoBodyId Id) const;

	/// Install (or, with null, remove) the stepper that owns the substeps from now on. The stepper is destroyed on the
	/// physics thread together with the system, so it may hold Chrono objects.
	void SetStepper(TSharedPtr<IChronoStepper> Stepper);

	/// Currently installed stepper (null = plain DoStepDynamics).
	TSharedPtr<IChronoStepper> GetStepper() const;

private:
	friend class FChronoSimCallback;
	FChronoCommandContext(FChronoSimCallback& InCallback, chrono::ChSystem& InSystem, const FChronoTickInputs& InInputs, double InChaosSimTime)
		: System(InSystem), Inputs(InInputs), ChaosSimTime(InChaosSimTime), Callback(InCallback)
	{
	}

	FChronoSimCallback& Callback;
};

/// Game-thread facade of a Chrono system hosted on the Chaos physics thread. See the file comment for the threading
/// contract.
class CHRONOCORE_API FChronoSimulation
{
public:
	/// Create the simulation and register its sim callback on Params.Scene's solver. The ChSystem itself is
	/// constructed on the physics thread during the next physics step. Returns null (and logs) if Params.Scene is
	/// null or has no solver. Game thread only.
	static TSharedPtr<FChronoSimulation> Create(const FChronoSimulationParams& Params);

	/// Unregisters the sim callback; Chaos destroys the callback and the ChSystem on the physics thread during the
	/// next physics step (or synchronously when the solver shuts down). Game thread only.
	~FChronoSimulation();

	FChronoSimulation(const FChronoSimulation&) = delete;
	FChronoSimulation& operator=(const FChronoSimulation&) = delete;

	/// Allocate a unique body id (any thread).
	FChronoBodyId AllocateBodyId();

	/// Queue a command for execution on the physics thread before the next physics step. Commands run in FIFO
	/// order, once, and must only touch Chrono (never UObjects). Dropped (with a log) once the simulation is no
	/// longer running. Game thread only.
	void Enqueue(TUniqueFunction<void(FChronoCommandContext&)>&& Command);

	/// Publish the inputs for the next physics step(s). Call once per frame. Game thread only.
	void PushInputs(const FChronoTickInputs& Inputs);

	/// Drain all snapshots produced since the last call and keep the newest. Call once per frame before reading
	/// GetLatestSnapshot(). Game thread only.
	void PumpOutputs();

	/// Newest snapshot pumped so far (an empty snapshot before the first physics step). Game thread only.
	const FChronoSnapshot& GetLatestSnapshot() const { return Latest; }

	/// True while the sim callback is registered and its solver is alive.
	bool IsRunning() const;

	/// Solver hosting the simulation (null after destruction of the scene).
	Chaos::FPhysicsSolverBase* GetSolver() const;

	/// Creation parameters.
	const FChronoSimulationParams& GetParams() const { return Params; }

private:
	FChronoSimulation() = default;

	bool IsSolverAlive() const;

	FChronoSimulationParams Params;
	FChronoSimCallback* Callback = nullptr;   ///< owned by the Chaos solver
	Chaos::FPhysicsSolverBase* Solver = nullptr;
	FChronoSnapshot Latest;
	std::atomic<uint32> NextBodyId{1};
	bool bWarnedDroppedCommand = false;
};
