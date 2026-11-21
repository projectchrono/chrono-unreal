// ChronoChaosDynamicMirror - two-way coupling between Chrono bodies and dynamic Unreal (Chaos) rigid bodies.

#include "ChaosCollision/ChronoChaosDynamicMirror.h"

#include "ChronoConversions.h"

#include "Chaos/ParticleHandle.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "PBDRigidsSolver.h"

#include "ChronoIncludeBegin.h"
#include "chrono/physics/ChSystem.h"
#include "ChronoIncludeEnd.h"

namespace ChronoChaosDynamicMirror
{
	using namespace Chaos;
	using chrono::ChVector3d;
	using chrono::FChronoDynamicMirror;

	namespace
	{
		/// Angular velocity is an axial vector: mirroring Y negates X and Z (same mapping as the quaternion axis).
		ChVector3d ToChronoAngVel(const FVector& W) { return ChVector3d(-W.X, W.Y, -W.Z); }
		FVector ToUnrealAngVel(const ChVector3d& W) { return FVector(-W.x(), W.y(), -W.z()); }

		/// Copy mass properties, pose and velocity of the particle into the mirror body.
		void SyncFromParticle(FChronoDynamicMirror& Mirror, const FPBDRigidParticleHandle& Rigid, const chrono::ChSystem& System)
		{
			chrono::ChBody& Body = *Mirror.Body;

			// Chaos inertia is the diagonal in the centre-of-mass frame (kg cm^2); the mirror body's frame is that frame.
			const TVec3<FRealSingle> Inertia = Rigid.I();
			Body.SetMass(FMath::Max(static_cast<double>(Rigid.M()), 1e-3));
			Body.SetInertiaXX(ChVector3d(FMath::Max(Inertia.X * 1e-4, 1e-6), FMath::Max(Inertia.Y * 1e-4, 1e-6), FMath::Max(Inertia.Z * 1e-4, 1e-6)));

			Mirror.ComLocalUE = FTransform(FQuat(Rigid.RotationOfMass()), FVector(Rigid.CenterOfMass()));
			Mirror.ParticlePoseUE = FTransform(FQuat(Rigid.GetR()), FVector(Rigid.GetX()));
			const FTransform ComWorld = Mirror.ComLocalUE * Mirror.ParticlePoseUE;

			Body.SetPos(FChronoConversions::ToChronoPos(ComWorld.GetLocation()));
			Body.SetRot(FChronoConversions::ToChronoQuat(ComWorld.GetRotation()));
			Mirror.StartLinVel = FChronoConversions::ToChronoPos(FVector(Rigid.GetV()));
			Mirror.StartAngVel = ToChronoAngVel(FVector(Rigid.GetW()));
			Body.SetPosDt(Mirror.StartLinVel);
			Body.SetAngVelParent(Mirror.StartAngVel);
			Mirror.SyncTime = System.GetChTime();
		}
	}

	bool IsTwoWayParticle(const chrono::ChCollisionSystemChaos::Impl& Impl, const FGeometryParticleHandle* Particle)
	{
		if (!Impl.Params.bTwoWayDynamics)
		{
			return false;
		}
		const FPBDRigidParticleHandle* Rigid = Particle->CastToRigidParticle();
		if (!Rigid || Rigid->InvM() <= 0)
		{
			return false;
		}
		const EObjectStateType State = Rigid->ObjectState();
		return State == EObjectStateType::Dynamic || State == EObjectStateType::Sleeping;
	}

	FChronoDynamicMirror* Touch(chrono::ChCollisionSystemChaos::Impl& Impl, const FGeometryParticleHandle* Particle)
	{
		chrono::ChSystem* System = Impl.Owner ? Impl.Owner->GetSystem() : nullptr;
		const FPBDRigidParticleHandle* Rigid = Particle->CastToRigidParticle();
		if (!System || !Rigid)
		{
			return nullptr;
		}

		FChronoDynamicMirror* Mirror = nullptr;
		if (FChronoDynamicMirror** Found = Impl.Mirrors.Find(Particle))
		{
			Mirror = *Found;
			if (Mirror->ChaosStepSerial == Impl.ChaosStepSerial)
			{
				return Mirror;   // already synchronised in this Chaos step
			}
		}
		else
		{
			if (Impl.FreeMirrors.Num() > 0)
			{
				Mirror = Impl.FreeMirrors.Pop(EAllowShrinking::No);
			}
			else
			{
				Mirror = Impl.MirrorPool.Add_GetRef(MakeUnique<FChronoDynamicMirror>()).Get();
				Mirror->Body = chrono_types::make_shared<chrono::ChBody>();
				Mirror->Body->SetName("ChronoChaosMirror");
				// Collision stays disabled: the model only gives Chrono contacts a ChContactable, the geometry is the
				// Unreal particle's. Chaos integrates the particle's own gyroscopic term.
				Mirror->Body->EnableCollision(false);
				Mirror->Body->SetUseGyroTorque(false);
				Mirror->Model = chrono_types::make_shared<chrono::ChCollisionModel>();
				Mirror->Body->AddCollisionModel(Mirror->Model);
			}
			System->AddBody(Mirror->Body);
			Impl.Mirrors.Add(Particle, Mirror);
		}

		Mirror->Particle = const_cast<FPBDRigidParticleHandle*>(Rigid);
		Mirror->ChaosStepSerial = Impl.ChaosStepSerial;
		SyncFromParticle(*Mirror, *Rigid, *System);
		return Mirror;
	}

	void ReleaseStale(chrono::ChCollisionSystemChaos::Impl& Impl)
	{
		if (Impl.Mirrors.Num() == 0)
		{
			return;
		}
		chrono::ChSystem* System = Impl.Owner ? Impl.Owner->GetSystem() : nullptr;
		for (auto It = Impl.Mirrors.CreateIterator(); It; ++It)
		{
			FChronoDynamicMirror* Mirror = It.Value();
			if (Mirror->ChaosStepSerial == Impl.ChaosStepSerial)
			{
				continue;
			}
			// The particle may already be destroyed (despawned / pooled actor): only the key is used.
			Impl.Manifolds.RemoveObject(It.Key());
			if (System && Mirror->Body->GetSystem() == System)
			{
				System->RemoveBody(Mirror->Body);
			}
			Mirror->Particle = nullptr;
			Impl.FreeMirrors.Add(Mirror);
			It.RemoveCurrent();
		}
	}

	void UpdatePoses(chrono::ChCollisionSystemChaos::Impl& Impl)
	{
		for (const TPair<const FGeometryParticleHandle*, FChronoDynamicMirror*>& Pair : Impl.Mirrors)
		{
			FChronoDynamicMirror& Mirror = *Pair.Value;
			const FTransform ComWorld(FChronoConversions::ToUnrealQuat(Mirror.Body->GetRot()), FChronoConversions::ToUnrealPos(Mirror.Body->GetPos()));
			Mirror.ParticlePoseUE = Mirror.ComLocalUE.Inverse() * ComWorld;
		}
	}

	void ApplyImpulses(chrono::ChCollisionSystemChaos::Impl& Impl)
	{
		chrono::ChSystem* System = Impl.Owner ? Impl.Owner->GetSystem() : nullptr;
		if (!System || !Impl.IsBound() || Impl.Mirrors.Num() == 0)
		{
			return;
		}
		FPBDRigidsEvolution* Evolution = Impl.Solver->GetEvolution();
		const ChVector3d Gravity = System->GetGravitationalAcceleration();

		for (const TPair<const FGeometryParticleHandle*, FChronoDynamicMirror*>& Pair : Impl.Mirrors)
		{
			FChronoDynamicMirror& Mirror = *Pair.Value;
			// Only mirrors validated by this step's candidate gather: an older one may point at a destroyed particle.
			if (Mirror.ChaosStepSerial != Impl.ChaosStepSerial || !Mirror.Particle)
			{
				continue;
			}
			const double Elapsed = System->GetChTime() - Mirror.SyncTime;
			if (Elapsed <= 0.0)
			{
				continue;
			}

			// Chrono applied gravity to the free-floating mirror; Chaos applies its own, so only contacts are transferred.
			const ChVector3d DeltaLin = Mirror.Body->GetPosDt() - Mirror.StartLinVel - Gravity * Elapsed;
			const ChVector3d DeltaAng = Mirror.Body->GetAngVelParent() - Mirror.StartAngVel;
			if (DeltaLin.Length2() < 1e-10 && DeltaAng.Length2() < 1e-10)
			{
				continue;   // untouched in this step
			}

			FPBDRigidParticleHandle* Rigid = Mirror.Particle;
			if (Rigid->ObjectState() == EObjectStateType::Sleeping)
			{
				Evolution->WakeParticle(Rigid);
			}
			Rigid->SetV(Rigid->GetV() + FVec3(FChronoConversions::ToUnrealPos(DeltaLin)));
			Rigid->SetW(Rigid->GetW() + FVec3(ToUnrealAngVel(DeltaAng)));

			// The transferred part is consumed (several calls per step must not apply it twice).
			Mirror.StartLinVel = Mirror.Body->GetPosDt();
			Mirror.StartAngVel = Mirror.Body->GetAngVelParent();
			Mirror.SyncTime = System->GetChTime();
		}
	}

	void Reset(chrono::ChCollisionSystemChaos::Impl& Impl, bool bRemoveFromSystem)
	{
		chrono::ChSystem* System = (bRemoveFromSystem && Impl.Owner) ? Impl.Owner->GetSystem() : nullptr;
		if (System)
		{
			for (const TPair<const FGeometryParticleHandle*, FChronoDynamicMirror*>& Pair : Impl.Mirrors)
			{
				if (Pair.Value->Body->GetSystem() == System)
				{
					System->RemoveBody(Pair.Value->Body);
				}
			}
		}
		Impl.Mirrors.Reset();
		Impl.FreeMirrors.Reset();
		Impl.MirrorPool.Reset();
	}
}
