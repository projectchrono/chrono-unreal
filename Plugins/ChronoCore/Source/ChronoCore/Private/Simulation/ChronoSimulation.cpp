// FChronoSimulation - game-thread facade of a Chrono system hosted on the Chaos physics thread.

#include "ChronoSimulation.h"

#include "Simulation/ChronoSimCallback.h"
#include "ChronoCoreModule.h"

#include "Algo/BinarySearch.h"
#include "Chaos/ChaosScene.h"
#include "ChaosSolversModule.h"
#include "PBDRigidsSolver.h"

// ---------------------------------------------------------------------------------------------------------------------
// FChronoSnapshot
// ---------------------------------------------------------------------------------------------------------------------

const FChronoBodyState* FChronoSnapshot::FindBody(FChronoBodyId Id) const
{
	const int32 Index = Algo::BinarySearchBy(Bodies, Id.Value, [](const FChronoBodyState& State) { return State.Id.Value; });
	return Index != INDEX_NONE ? &Bodies[Index] : nullptr;
}

// ---------------------------------------------------------------------------------------------------------------------
// FChronoSimulation
// ---------------------------------------------------------------------------------------------------------------------

TSharedPtr<FChronoSimulation> FChronoSimulation::Create(const FChronoSimulationParams& Params)
{
	check(IsInGameThread());

	if (!Params.Scene)
	{
		UE_LOG(LogChronoCore, Error, TEXT("FChronoSimulation::Create: FChronoSimulationParams::Scene is null."));
		return nullptr;
	}

	Chaos::FPhysicsSolverBase* Solver = Params.Scene->GetSolver();
	if (!Solver)
	{
		UE_LOG(LogChronoCore, Error, TEXT("FChronoSimulation::Create: the Chaos scene has no solver."));
		return nullptr;
	}

	TSharedPtr<FChronoSimulation> Simulation = MakeShareable(new FChronoSimulation());
	Simulation->Params = Params;
	Simulation->Solver = Solver;
	Simulation->Callback = Solver->CreateAndRegisterSimCallbackObject_External<FChronoSimCallback>(Params);

	UE_LOG(LogChronoCore, Log, TEXT("FChronoSimulation: registered Chrono sim callback (%s backend, step %.4g s, async physics %s)."),
	       Params.Backend == EChronoCollisionBackend::Chaos ? TEXT("Chaos") : TEXT("Bullet"), Params.StepSizeS,
	       Solver->IsUsingAsyncResults() ? TEXT("on") : TEXT("off"));
	return Simulation;
}

FChronoSimulation::~FChronoSimulation()
{
	check(IsInGameThread());

	if (!Callback)
	{
		return;
	}

	if (IsSolverAlive())
	{
		// Chaos destroys the callback (and with it the ChSystem) on the physics thread during the next step.
		Solver->UnregisterAndFreeSimCallbackObject_External(Callback);
	}
	else
	{
		UE_LOG(LogChronoCore, Warning,
		       TEXT("FChronoSimulation destroyed after its Chaos solver: the sim callback cannot be unregistered and is leaked. ")
		       TEXT("Destroy the simulation before the world's physics scene (e.g. in EndPlay)."));
	}
	Callback = nullptr;
	Solver = nullptr;
}

FChronoBodyId FChronoSimulation::AllocateBodyId()
{
	FChronoBodyId Id;
	Id.Value = NextBodyId.fetch_add(1);
	if (Id.Value == 0)   // wrapped around
	{
		Id.Value = NextBodyId.fetch_add(1);
	}
	return Id;
}

void FChronoSimulation::Enqueue(TUniqueFunction<void(FChronoCommandContext&)>&& Command)
{
	check(IsInGameThread());

	if (!IsRunning())
	{
		if (!bWarnedDroppedCommand)
		{
			bWarnedDroppedCommand = true;
			UE_LOG(LogChronoCore, Warning, TEXT("FChronoSimulation::Enqueue: the simulation is not running - command dropped."));
		}
		return;
	}
	Callback->EnqueueCommand_External(MoveTemp(Command));
}

void FChronoSimulation::PushInputs(const FChronoTickInputs& Inputs)
{
	check(IsInGameThread());

	if (!IsRunning())
	{
		return;
	}
	if (FChronoSimInput* Input = Callback->GetProducerInputData_External())
	{
		Input->Inputs = Inputs;
		Input->bHasInputs = true;
	}
}

void FChronoSimulation::PumpOutputs()
{
	check(IsInGameThread());

	if (!IsRunning())
	{
		return;
	}

	Chaos::TSimCallbackOutputHandle<FChronoSimOutput> Output;
	while ((Output = Callback->PopFutureOutputData_External()))
	{
		// The handle recycles the output (Reset) when it goes out of scope - steal the snapshot first.
		Latest = MoveTemp(Output->Snapshot);
	}
}

bool FChronoSimulation::IsRunning() const
{
	return Callback != nullptr && IsSolverAlive();
}

Chaos::FPhysicsSolverBase* FChronoSimulation::GetSolver() const
{
	return IsSolverAlive() ? Solver : nullptr;
}

bool FChronoSimulation::IsSolverAlive() const
{
	if (!Solver)
	{
		return false;
	}
	const FChaosSolversModule* Module = FChaosSolversModule::GetModule();
	return Module && Module->GetAllSolvers().Contains(Solver);
}
