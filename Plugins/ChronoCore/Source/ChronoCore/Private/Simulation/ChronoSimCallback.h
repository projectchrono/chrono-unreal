// FChronoSimCallback - the Chaos sim callback that owns and steps a chrono::ChSystem on the physics thread.
// Private to ChronoCore; game code uses FChronoSimulation (Public/ChronoSimulation.h).

#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "Containers/SortedMap.h"

#include "ChronoSimulation.h"

#include "Chaos/SimCallbackObject.h"
#include "Chaos/SimCallbackInput.h"

#include <memory>

namespace chrono
{
	class ChSystem;
	class ChBody;
}

/// Per-physics-step inputs marshalled from the game thread (FChronoSimulation::PushInputs).
struct FChronoSimInput : public Chaos::FSimCallbackInput
{
	FChronoTickInputs Inputs;
	bool bHasInputs = false;

	void Reset()
	{
		Inputs = FChronoTickInputs();
		bHasInputs = false;
	}
};

/// Per-physics-step output handed back to the game thread (FChronoSimulation::PumpOutputs).
struct FChronoSimOutput : public Chaos::FSimCallbackOutput
{
	FChronoSnapshot Snapshot;

	void Reset()
	{
		Snapshot = FChronoSnapshot();
	}
};

/// Physics-thread host of the Chrono system. Registered on the solver by FChronoSimulation::Create and destroyed by
/// Chaos (on the physics thread) after FChronoSimulation unregisters it.
class FChronoSimCallback : public Chaos::TSimCallbackObject<FChronoSimInput, FChronoSimOutput, Chaos::ESimCallbackOptions::Presimulate>
{
public:
	using FCommand = TUniqueFunction<void(FChronoCommandContext&)>;

	explicit FChronoSimCallback(const FChronoSimulationParams& InParams);
	virtual ~FChronoSimCallback() override;

	/// Game thread: queue a command for the next physics step (MPSC queue, lock free).
	void EnqueueCommand_External(FCommand&& Command);

	// Physics thread (used by FChronoCommandContext).
	void TrackBody(FChronoBodyId Id, std::shared_ptr<chrono::ChBody> Body);
	void UntrackBody(FChronoBodyId Id);
	std::shared_ptr<chrono::ChBody> FindTrackedBody(FChronoBodyId Id) const;
	void SetStepper(TSharedPtr<IChronoStepper> InStepper) { Stepper = MoveTemp(InStepper); }
	TSharedPtr<IChronoStepper> GetStepper() const { return Stepper; }

	virtual FName GetFNameForStatId() const override;

private:
	virtual void OnPostInitialize_Internal() override;
	virtual void OnPreSimulate_Internal() override;

	void CreateSystem_Internal();
	void DestroySystem_Internal();
	void DrainCommands_Internal();
	int32 StepChrono_Internal(double DeltaTime);
	void WriteSnapshot_Internal(int32 SubStepsThisTick);

	FChronoSimulationParams Params;

	/// Commands pushed by the game thread, consumed (once) by the physics thread.
	TQueue<FCommand, EQueueMode::Mpsc> Commands;

	// --- physics-thread state -------------------------------------------------------------------------------------
	std::unique_ptr<chrono::ChSystem> System;
	TSortedMap<uint32, std::shared_ptr<chrono::ChBody>> Tracked;
	TSharedPtr<IChronoStepper> Stepper;
	FChronoTickInputs LatestInputs;
	double Accumulator = 0.0;
	double DroppedTimeS = 0.0;
	int32 StepIndex = 0;
	bool bWarnedBacklog = false;
	bool bSystemCreated = false;
};
