// ChronoChaosSceneProxy - physics-thread kinematic proxy particles for Chrono collision models.

#include "ChaosCollision/ChronoChaosSceneProxy.h"

#include "ChronoConversions.h"
#include "ChronoCoreModule.h"

#include "Chaos/ImplicitObject.h"
#include "Chaos/KinematicTargets.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/ShapeInstance.h"
#include "Framework/Threading.h"
#include "PBDRigidsSolver.h"

namespace ChronoChaosSceneProxy
{
	using namespace Chaos;

	chrono::ChAABB ToChronoAABB(const FAABB3& Bounds)
	{
		if (Bounds.IsEmpty())
		{
			return chrono::ChAABB();
		}
		// Unreal -> Chrono is a uniform scale plus a mirror of Y, so an AABB maps to an AABB with min/max Y swapped.
		const chrono::ChVector3d A = FChronoConversions::ToChronoPos(FVector(Bounds.Min()));
		const chrono::ChVector3d B = FChronoConversions::ToChronoPos(FVector(Bounds.Max()));
		return chrono::ChAABB(chrono::ChVector3d(FMath::Min(A.x(), B.x()), FMath::Min(A.y(), B.y()), FMath::Min(A.z(), B.z())),
		                      chrono::ChVector3d(FMath::Max(A.x(), B.x()), FMath::Max(A.y(), B.y()), FMath::Max(A.z(), B.z())));
	}

	void UpdateWorldBounds(chrono::ChCollisionModelChaos::Data& Data)
	{
		if (Data.Geometry.IsValid() && !Data.Geometry.LocalBounds.IsEmpty())
		{
			Data.WorldBounds = Data.Geometry.LocalBounds.TransformedAABB(FRigidTransform3(Data.PoseUE));
		}
		else
		{
			Data.WorldBounds = FAABB3::EmptyAABB();
		}
	}

	bool CreateProxy(chrono::ChCollisionSystemChaos::Impl& Impl, chrono::ChCollisionModelChaos& Model)
	{
		chrono::ChCollisionModelChaos::Data* Data = Model.GetData();
		if (!Impl.IsBound() || !Data || Data->Particle)
		{
			return Data && Data->Particle != nullptr;
		}

		EnsureIsInPhysicsThreadContext();

		if (!Data->bGeometryBuilt)
		{
			Data->bGeometryBuilt = true;
			if (chrono::ChCollisionModel* ChronoModel = Model.GetModel())
			{
				ChronoChaosShapeFactory::BuildGeometry(*ChronoModel, Data->Geometry);
			}
		}
		if (!Data->Geometry.IsValid())
		{
			UE_LOG(LogChronoCore, Warning, TEXT("ChCollisionSystemChaos: collision model has no shape supported by the Chaos backend - no proxy particle created."));
			return false;
		}

		// Current pose of the contactable.
		Model.SyncPosition();

		FPBDRigidsSolver& Solver = *Impl.Solver;
		FPBDRigidsEvolution* Evolution = Solver.GetEvolution();

		// A rigid particle in the Kinematic object state - the same representation Unreal uses for its own kinematic
		// bodies. Pure FKinematicGeometryParticles are never integrated by kinematic targets (only rigid particles are
		// tracked as "moving kinematics"), so they could not push Unreal dynamics.
		FPBDRigidParticleHandle* Particle = Solver.GetParticles().CreateDynamicParticles(1)[0];
		Particle->SetGeometry(Data->Geometry.Root);
		Particle->SetHasCollision(true);
		Particle->SetM(0);
		Particle->SetInvM(0);
		Particle->SetI(TVec3<FRealSingle>(0));
		Particle->SetInvI(TVec3<FRealSingle>(0));
		Particle->SetGravityEnabled(false);
		Particle->SetCCDEnabled(false);
#if CHAOS_DEBUG_NAME
		Particle->SetDebugName(MakeShared<FString, ESPMode::ThreadSafe>(TEXT("ChronoProxy")));
#endif

		// One FPerShapeData per union child: filter + sim/query flags from the bind params.
		const Filter::FCombinedShapeFilterData CombinedFilter(Impl.ShapeFilter, Filter::FInstanceData());
		for (const TUniquePtr<FPerShapeData>& Shape : Particle->ShapesArray())
		{
			Shape->SetCombinedShapeFilterData(CombinedFilter);
			Shape->SetSimEnabled(Impl.Params.bProxySimEnabled);
			Shape->SetQueryEnabled(Impl.Params.bProxyQueryEnabled);
		}

		// Kinematic state, initial pose (X/R and the predicted P/Q), world-space shape bounds and registration with the
		// evolution / acceleration structure (same sequence as FSingleParticlePhysicsProxy::PushToPhysicsState).
		// Kinematics are not added to the constraint graph until a constraint references them.
		Evolution->SetParticleObjectState(Particle, EObjectStateType::Kinematic);
		const FRigidTransform3 Pose(Data->PoseUE);
		Evolution->SetParticleTransform(Particle, Pose.GetLocation(), Pose.GetRotation(), /*bIsTeleport*/ true, /*bWakeUp*/ false);
		Particle->SetP(Pose.GetLocation());
		Particle->SetQ(Pose.GetRotation());
		Particle->SetV(FVec3(0));
		Particle->SetW(FVec3(0));
		Particle->UpdateWorldSpaceState(Pose, FVec3(0));
		Evolution->EnableParticle(Particle);

		Data->Particle = Particle;
		Data->CommittedPoseUE = Data->PoseUE;
		Data->bPoseCommitted = true;
		UpdateWorldBounds(*Data);

		Impl.Proxies.Add(&Model);
		Impl.ParticleToModel.Add(Particle, &Model);
		return true;
	}

	void DestroyProxy(chrono::ChCollisionSystemChaos::Impl& Impl, chrono::ChCollisionModelChaos& Model)
	{
		chrono::ChCollisionModelChaos::Data* Data = Model.GetData();
		if (!Data || !Data->Particle)
		{
			return;
		}

		FPBDRigidParticleHandle* Particle = Data->Particle;
		Data->Particle = nullptr;
		Data->bPoseCommitted = false;
		Data->WorldBounds = FAABB3::EmptyAABB();

		Impl.Proxies.Remove(&Model);
		Impl.ParticleToModel.Remove(Particle);

		if (Impl.IsBound())
		{
			EnsureIsInPhysicsThreadContext();
			Impl.Solver->GetEvolution()->DestroyParticle(Particle);
		}
	}

	void CommitKinematicTargets(chrono::ChCollisionSystemChaos::Impl& Impl)
	{
		if (!Impl.IsBound() || Impl.Proxies.Num() == 0)
		{
			return;
		}

		EnsureIsInPhysicsThreadContext();
		FPBDRigidsEvolution* Evolution = Impl.Solver->GetEvolution();

		for (chrono::ChCollisionModelChaos* Model : Impl.Proxies)
		{
			chrono::ChCollisionModelChaos::Data* Data = Model->GetData();
			if (!Data || !Data->Particle)
			{
				continue;
			}
			if (Data->bPoseCommitted && Data->PoseUE.Equals(Data->CommittedPoseUE, UE_KINDA_SMALL_NUMBER))
			{
				continue;
			}

			// Chaos moves the particle to the target during this step (ApplyKinematicTargets), derives V/W from the
			// delta (so Unreal dynamics get pushed with the right momentum), refreshes the world-space shape bounds and
			// queues the acceleration-structure update.
			Evolution->SetParticleKinematicTarget(Data->Particle, FKinematicTarget::MakePositionTarget(FRigidTransform3(Data->PoseUE)));
			Data->CommittedPoseUE = Data->PoseUE;
			Data->bPoseCommitted = true;
		}
	}
}
