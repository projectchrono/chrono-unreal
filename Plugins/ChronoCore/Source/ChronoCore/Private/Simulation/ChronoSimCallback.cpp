// FChronoSimCallback - physics-thread host of a chrono::ChSystem.

#include "Simulation/ChronoSimCallback.h"

#include "ChronoChaosCollision.h"
#include "ChronoConversions.h"
#include "ChronoCoreModule.h"

#include "Framework/Threading.h"
#include "PBDRigidsSolver.h"

#include "ChronoIncludeBegin.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "ChronoIncludeEnd.h"

#include <exception>

namespace
{
	/// Collects contact points for FChronoSnapshot::Contacts.
	class FSnapshotContactCollector : public chrono::ChContactContainer::ReportContactCallback
	{
	public:
		explicit FSnapshotContactCollector(TArray<FChronoContactDebug>& InContacts) : Contacts(InContacts) {}

		virtual bool OnReportContact(const chrono::ChVector3d& pA,
		                             const chrono::ChVector3d& pB,
		                             const chrono::ChMatrix33<>& plane_coord,
		                             double distance,
		                             double eff_radius,
		                             const chrono::ChVector3d& react_forces,
		                             const chrono::ChVector3d& react_torques,
		                             chrono::ChContactable* contactobjA,
		                             chrono::ChContactable* contactobjB,
		                             int constraint_offset) override
		{
			FChronoContactDebug& C = Contacts.AddDefaulted_GetRef();
			C.Point = FChronoConversions::ToUnrealPos(pA);
			C.Normal = FChronoConversions::ToUnrealDir(plane_coord.GetAxisX());
			C.Distance = distance;
			return true;
		}

	private:
		TArray<FChronoContactDebug>& Contacts;
	};
}

// ---------------------------------------------------------------------------------------------------------------------
// FChronoCommandContext
// ---------------------------------------------------------------------------------------------------------------------

void FChronoCommandContext::TrackBody(FChronoBodyId Id, std::shared_ptr<chrono::ChBody> Body)
{
	Callback.TrackBody(Id, MoveTemp(Body));
}

void FChronoCommandContext::UntrackBody(FChronoBodyId Id)
{
	Callback.UntrackBody(Id);
}

std::shared_ptr<chrono::ChBody> FChronoCommandContext::FindTrackedBody(FChronoBodyId Id) const
{
	return Callback.FindTrackedBody(Id);
}

void FChronoCommandContext::SetStepper(TSharedPtr<IChronoStepper> Stepper)
{
	Callback.SetStepper(MoveTemp(Stepper));
}

TSharedPtr<IChronoStepper> FChronoCommandContext::GetStepper() const
{
	return Callback.GetStepper();
}

// ---------------------------------------------------------------------------------------------------------------------
// FChronoSimCallback
// ---------------------------------------------------------------------------------------------------------------------

FChronoSimCallback::FChronoSimCallback(const FChronoSimulationParams& InParams)
	: Params(InParams)
{
}

FChronoSimCallback::~FChronoSimCallback()
{
	// Chaos deletes callbacks on the physics thread (FPushPhysicsData::Reset) while the solver is still alive.
	DestroySystem_Internal();
}

FName FChronoSimCallback::GetFNameForStatId() const
{
	const static FLazyName StaticName("FChronoSimCallback");
	return StaticName;
}

void FChronoSimCallback::EnqueueCommand_External(FCommand&& Command)
{
	Commands.Enqueue(MoveTemp(Command));
}

void FChronoSimCallback::TrackBody(FChronoBodyId Id, std::shared_ptr<chrono::ChBody> Body)
{
	if (!Id.IsValid() || !Body)
	{
		return;
	}
	Tracked.Add(Id.Value, MoveTemp(Body));
}

void FChronoSimCallback::UntrackBody(FChronoBodyId Id)
{
	Tracked.Remove(Id.Value);
}

std::shared_ptr<chrono::ChBody> FChronoSimCallback::FindTrackedBody(FChronoBodyId Id) const
{
	const std::shared_ptr<chrono::ChBody>* Found = Tracked.Find(Id.Value);
	return Found ? *Found : nullptr;
}

void FChronoSimCallback::OnPostInitialize_Internal()
{
	CreateSystem_Internal();
}

void FChronoSimCallback::CreateSystem_Internal()
{
	if (bSystemCreated)
	{
		return;
	}
	bSystemCreated = true;

	try
	{
		if (Params.bSMC)
		{
			System = std::make_unique<chrono::ChSystemSMC>();
		}
		else
		{
			System = std::make_unique<chrono::ChSystemNSC>();
		}

		System->SetCollisionSystemType(Params.Backend == EChronoCollisionBackend::Chaos
			                               ? chrono::ChCollisionSystem::Type::CHAOS
			                               : chrono::ChCollisionSystem::Type::BULLET);
		System->SetGravitationalAcceleration(FChronoConversions::ToChronoPos(Params.GravityUE));
		System->SetSolverType(chrono::ChSolver::Type::PSOR);
		if (chrono::ChIterativeSolver* Iterative = System->GetSolver()->AsIterative())
		{
			Iterative->SetMaxIterations(FMath::Max(1, Params.SolverMaxIterations));
		}

		if (Params.ConfigureSystem)
		{
			Params.ConfigureSystem(*System);
		}

		if (Params.Backend == EChronoCollisionBackend::Chaos)
		{
			Chaos::FPBDRigidsSolver* RigidsSolver = static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
			if (RigidsSolver)
			{
				FChronoChaosCollision::BindSolver(*System, *RigidsSolver, Params.Collision);
			}
		}
	}
	catch (const std::exception& Ex)
	{
		UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: exception while creating the Chrono system: %hs"), Ex.what());
		System.reset();
	}

	UE_LOG(LogChronoCore, Log, TEXT("FChronoSimCallback: Chrono system created on the physics thread (%s backend, step %.4g s)."),
	       Params.Backend == EChronoCollisionBackend::Chaos ? TEXT("Chaos") : TEXT("Bullet"), Params.StepSizeS);
}

void FChronoSimCallback::DestroySystem_Internal()
{
	if (!System)
	{
		Tracked.Empty();
		Stepper.Reset();
		return;
	}

	try
	{
		if (Params.Backend == EChronoCollisionBackend::Chaos)
		{
			FChronoChaosCollision::UnbindSolver(*System);
		}
		Tracked.Empty();
		// The stepper may own items of the system (vehicle subsystems remove themselves from the system on
		// destruction), so it goes first while the system is still alive.
		Stepper.Reset();
		System.reset();
	}
	catch (const std::exception& Ex)
	{
		UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: exception while destroying the Chrono system: %hs"), Ex.what());
	}
}

void FChronoSimCallback::OnPreSimulate_Internal()
{
	if (!System)
	{
		// Creation failed (logged) - keep draining commands so the game thread's queue does not grow forever.
		FCommand Dropped;
		while (Commands.Dequeue(Dropped)) {}
		return;
	}

	// 1. Inputs (may be absent when the game thread did not push any for this step).
	if (const FChronoSimInput* Input = GetConsumerInput_Internal())
	{
		if (Input->bHasInputs)
		{
			LatestInputs = Input->Inputs;
		}
	}

	// 2. One-shot commands.
	DrainCommands_Internal();

	// 3. Fixed-step Chrono integration. While paused nothing is stepped and the accumulated time is dropped so that
	//    resuming continues from the current state without a time jump.
	int32 SubSteps = 0;
	if (LatestInputs.IsPaused())
	{
		Accumulator = 0.0;
	}
	else
	{
		if (Params.Backend == EChronoCollisionBackend::Chaos)
		{
			FChronoChaosCollision::BeginChaosStep(*System, GetDeltaTime_Internal());
		}
		SubSteps = StepChrono_Internal(GetDeltaTime_Internal());
		if (Params.Backend == EChronoCollisionBackend::Chaos)
		{
			FChronoChaosCollision::CommitKinematicTargets(*System);
		}
	}

	// 4. Snapshot.
	WriteSnapshot_Internal(SubSteps);
}

void FChronoSimCallback::DrainCommands_Internal()
{
	FChronoCommandContext Context(*this, *System, LatestInputs, GetSimTime_Internal());

	bool bRanAny = false;
	FCommand Command;
	while (Commands.Dequeue(Command))
	{
		if (!Command)
		{
			continue;
		}
		bRanAny = true;
		try
		{
			Command(Context);
		}
		catch (const std::exception& Ex)
		{
			UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: exception in a Chrono command (ignored): %hs"), Ex.what());
		}
		catch (...)
		{
			UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: unknown exception in a Chrono command (ignored)."));
		}
	}

	// Chrono binds collision models to the collision system only in ChSystem::Initialize() (first step); items added
	// by later commands would never collide unless bound explicitly. Both backends make Add() idempotent, so binding
	// everything after a command batch is cheap and safe.
	if (bRanAny)
	{
		if (const std::shared_ptr<chrono::ChCollisionSystem>& CollisionSystem = System->GetCollisionSystem())
		{
			if (CollisionSystem->IsInitialized())
			{
				try
				{
					CollisionSystem->BindAll();
				}
				catch (const std::exception& Ex)
				{
					UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: exception while binding collision models: %hs"), Ex.what());
				}
			}
		}
	}
}

int32 FChronoSimCallback::StepChrono_Internal(double DeltaTime)
{
	const double Step = FMath::Max(Params.StepSizeS, 1e-6);
	const int32 MaxSubSteps = FMath::Max(Params.MaxSubStepsPerTick, 1);

	Accumulator += FMath::Max(DeltaTime, 0.0);

	int32 SubSteps = 0;
	try
	{
		// Copy the stepper pointer: a stepper may replace itself from within SubStep.
		const TSharedPtr<IChronoStepper> ActiveStepper = Stepper;
		FChronoStepContext Context{*System, LatestInputs};
		Context.Dt = Step;
		Context.ChaosSimTime = GetSimTime_Internal();

		if (ActiveStepper && Accumulator >= Step)
		{
			Context.Time = System->GetChTime();
			ActiveStepper->PreStep(Context);
		}

		while (Accumulator >= Step && SubSteps < MaxSubSteps)
		{
			if (ActiveStepper)
			{
				Context.Time = System->GetChTime();
				Context.SubStep = SubSteps;
				ActiveStepper->SubStep(Context);
			}
			else
			{
				System->DoStepDynamics(Step);
			}
			Accumulator -= Step;
			++SubSteps;
		}
	}
	catch (const std::exception& Ex)
	{
		UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: exception in the Chrono substep: %hs"), Ex.what());
	}
	StepIndex += SubSteps;

	// Drop the backlog instead of trying to catch up forever.
	if (Accumulator >= Step)
	{
		const double Dropped = FMath::Floor(Accumulator / Step) * Step;
		Accumulator -= Dropped;
		DroppedTimeS += Dropped;
		if (!bWarnedBacklog)
		{
			bWarnedBacklog = true;
			UE_LOG(LogChronoCore, Warning,
			       TEXT("FChronoSimCallback: Chrono cannot keep up with the physics step (dt %.4g s, %d substeps of %.4g s allowed) - dropped %.4g s of simulation time. ")
			       TEXT("Increase MaxSubStepsPerTick or StepSizeS (further drops are not logged)."),
			       DeltaTime, MaxSubSteps, Step, Dropped);
		}
	}

	return SubSteps;
}

void FChronoSimCallback::WriteSnapshot_Internal(int32 SubStepsThisTick)
{
	FChronoSnapshot& Snapshot = GetProducerOutputData_Internal().Snapshot;

	Snapshot.SimTime = System->GetChTime();
	Snapshot.ChaosSimTime = GetSimTime_Internal();
	Snapshot.StepIndex = StepIndex;
	Snapshot.SubStepsThisTick = SubStepsThisTick;
	Snapshot.DroppedTimeS = DroppedTimeS;
	Snapshot.NumContacts = static_cast<int32>(System->GetNumContacts());
	if (const std::shared_ptr<chrono::ChCollisionSystem>& CollisionSystem = System->GetCollisionSystem())
	{
		Snapshot.CollisionBroadTimeS = CollisionSystem->GetTimerCollisionBroad();
		Snapshot.CollisionNarrowTimeS = CollisionSystem->GetTimerCollisionNarrow();
	}

	Snapshot.Bodies.Reset(Tracked.Num());
	for (const TPair<uint32, std::shared_ptr<chrono::ChBody>>& Pair : Tracked)   // TSortedMap iterates in key order
	{
		const chrono::ChBody& Body = *Pair.Value;
		FChronoBodyState& State = Snapshot.Bodies.AddDefaulted_GetRef();
		State.Id.Value = Pair.Key;
		State.Transform = FChronoConversions::ToUnrealTransform(Body.GetFrameRefToAbs());
		State.LinearVelocity = FChronoConversions::ToUnrealPos(Body.GetFrameRefToAbs().GetPosDt());
		State.AngularVelocity = FChronoConversions::ToUnrealDir(Body.GetAngVelParent());
	}

	Snapshot.Contacts.Reset();
	if (Params.bCaptureContacts && System->GetContactContainer())
	{
		auto Collector = chrono_types::make_shared<FSnapshotContactCollector>(Snapshot.Contacts);
		System->GetContactContainer()->ReportAllContacts(Collector);
	}

	Snapshot.Payload.Reset();
	if (Stepper)
	{
		try
		{
			Snapshot.Payload = Stepper->MakePayload(*System);
		}
		catch (const std::exception& Ex)
		{
			UE_LOG(LogChronoCore, Error, TEXT("FChronoSimCallback: exception in IChronoStepper::MakePayload (ignored): %hs"), Ex.what());
		}
	}
}
