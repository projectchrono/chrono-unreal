// ChronoChaosUnrealPhase - contacts between Chrono proxies and the non-Chrono geometry of the bound Chaos solver.

#include "ChaosCollision/ChronoChaosUnrealPhase.h"

#include "ChaosCollision/ChronoChaosCollisionPhase.h"
#include "ChaosCollision/ChronoChaosNarrowphase.h"
#include "ChaosCollision/ChronoChaosSceneQuery.h"
#include "ChronoConversions.h"
#include "ChronoCoreModule.h"

#include "Chaos/Defines.h"
#include "Chaos/Framework/Parallel.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ISpatialAcceleration.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/PhysicalMaterials.h"
#include "Chaos/ShapeInstance.h"
#include "PBDRigidsSolver.h"

#include "ChronoIncludeBegin.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChSystem.h"
#include "ChronoIncludeEnd.h"

namespace ChronoChaosUnrealPhase
{
	using namespace Chaos;
	using chrono::ChCollisionModelChaos;
	using chrono::ChCollisionSystemChaos;
	using chrono::FChronoUnrealCandidate;

	namespace
	{
		/// Base inflation of the candidate query bounds (cm) on top of envelope and per-step displacement.
		constexpr double BaseInflationCm = 10.0;

		/// Collects the non-Chrono, collidable shapes overlapping a query box.
		class FCandidateVisitor : public ISpatialVisitor<FAccelerationStructureHandle, FReal>
		{
		public:
			FCandidateVisitor(const ChCollisionSystemChaos::Impl& InImpl, const FAABB3& InQueryBounds, TArray<FChronoUnrealCandidate>& InOut)
				: Impl(InImpl), QueryBounds(InQueryBounds), Out(InOut)
			{
			}

			virtual bool Overlap(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance) override
			{
				const FGeometryParticleHandle* Particle = Instance.Payload.GetGeometryParticleHandle_PhysicsThread();
				if (!Particle || Impl.ParticleToModel.Contains(Particle))
				{
					return true;   // Chrono proxies are handled by the Chrono <-> Chrono phase
				}

				const FShapesArray& Shapes = Particle->ShapesArray();
				for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
				{
					const FPerShapeData* Shape = Shapes[ShapeIndex].Get();
					if (!Shape || !ChronoChaosSceneQuery::IsUnrealShapeCollidable(Impl, *Shape))
					{
						continue;
					}
					// Per-shape bounds are world space and kept up to date by the solver.
					if (Shape->GetLeafGeometry() && Shape->GetLeafGeometry()->HasBoundingBox() && !Shape->GetWorldSpaceShapeBounds().Intersects(QueryBounds))
					{
						continue;
					}
					FChronoUnrealCandidate& Candidate = Out.AddDefaulted_GetRef();
					Candidate.Particle = Particle;
					Candidate.ShapeIndex = ShapeIndex;
				}
				return true;
			}

			virtual bool Raycast(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance, FQueryFastData& CurData) override { return true; }
			virtual bool Sweep(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance, FQueryFastData& CurData) override { return true; }

		private:
			const ChCollisionSystemChaos::Impl& Impl;
			FAABB3 QueryBounds;
			TArray<FChronoUnrealCandidate>& Out;
		};

		/// Gather the candidates of one proxy.
		void GatherProxyCandidates(const ChCollisionSystemChaos::Impl& Impl, ChCollisionModelChaos& Model)
		{
			ChCollisionModelChaos::Data* Data = Model.GetData();
			Data->UnrealCandidates.Reset();
			if (!Data->Particle || Data->WorldBounds.IsEmpty())
			{
				return;
			}

			// Inflate by the envelope, a base margin and the distance the proxy can travel in one Chaos step.
			double Inflation = BaseInflationCm + ChronoChaosCollisionPhase::EnvelopeCm(Model);
			if (chrono::ChContactable* Contactable = Model.GetModel() ? Model.GetModel()->GetContactable() : nullptr)
			{
				const chrono::ChVector3d Speed = Contactable->GetContactPointSpeed(Model.GetAbsFrame().GetPos());
				Inflation += FChronoConversions::ToUnrealSize(Speed.Length()) * FMath::Max(Impl.ChaosStepDtS, 0.0) * 1.5;
			}

			FAABB3 QueryBounds = Data->WorldBounds;
			QueryBounds.Thicken(static_cast<FReal>(Inflation));
			Data->CandidateGatherLocation = Data->PoseUE.GetLocation();
			Data->CandidateInflationCm = Inflation;

			GatherCandidatesInBounds(Impl, QueryBounds, Data->UnrealCandidates);
		}

		/// A proxy that moved more than its inflation since the gather needs a fresh candidate list.
		bool NeedsRegather(const ChCollisionModelChaos::Data& Data)
		{
			return FVector::Dist(Data.PoseUE.GetLocation(), Data.CandidateGatherLocation) > 0.5 * Data.CandidateInflationCm;
		}

		/// Narrowphase of one proxy against its candidates.
		void NarrowphaseProxy(const ChCollisionSystemChaos::Impl& Impl, ChCollisionModelChaos& Model, TArray<ChCollisionSystemChaos::Impl::FPairContacts>& Out)
		{
			const ChCollisionModelChaos::Data& Data = *Model.GetData();
			const double CullDistance = ChronoChaosCollisionPhase::EnvelopeCm(Model);

			for (const FChronoUnrealCandidate& Candidate : Data.UnrealCandidates)
			{
				const FPerShapeData* Shape = GetCandidateShape(Candidate);
				if (!Shape)
				{
					continue;
				}
				const FImplicitObject* LeafB = Shape->GetLeafGeometry();
				if (!LeafB)
				{
					continue;
				}
				// Current particle pose (integrated for this Chaos step) - fresher than the acceleration structure.
				const FTransform LeafBTM(Shape->GetLeafWorldTransform(Candidate.Particle));
				const FAABB3 BoundsB = LeafB->HasBoundingBox() ? LeafB->BoundingBox().TransformedAABB(LeafBTM) : FAABB3::FullAABB();

				for (int32 SlotIndexA = 0; SlotIndexA < Data.Geometry.Slots.Num(); ++SlotIndexA)
				{
					const FChronoShapeSlot& SlotA = Data.Geometry.Slots[SlotIndexA];
					FAABB3 BoundsA = ChronoChaosCollisionPhase::LeafWorldBounds(Data, SlotA);
					BoundsA.Thicken(static_cast<FReal>(CullDistance));
					if (!BoundsA.Intersects(BoundsB))
					{
						continue;
					}

					const FTransform LeafATM = ChronoChaosCollisionPhase::LeafWorldTransform(Data, SlotA);
					ChronoChaosNarrowphase::FLeafParams ParamsA;
					ParamsA.SweptRadiusCm = SlotA.SweptRadiusCm;
					ParamsA.CylinderRadiusCm = SlotA.CylinderRadiusCm;
					ParamsA.CylinderHalfHeightCm = SlotA.CylinderHalfHeightCm;
					ChronoChaosNarrowphase::FContactArray Contacts;
					ChronoChaosNarrowphase::GenerateContacts(*SlotA.Leaf, LeafATM, *LeafB, LeafBTM, CullDistance, Contacts, ParamsA);
					if (Contacts.Num() == 0)
					{
						continue;
					}
					ChronoChaosNarrowphase::ReduceContacts(Contacts, 4);

					ChCollisionSystemChaos::Impl::FPairContacts& Pair = Out.AddDefaulted_GetRef();
					Pair.Key.ObjectA = &Model;
					Pair.Key.ShapeA = SlotIndexA;
					Pair.Key.ObjectB = Candidate.Particle;
					Pair.Key.ShapeB = Candidate.ShapeIndex;
					Pair.Key.bUnrealB = true;
					Pair.Key.SerialA = Data.SerialId;
					Pair.Key.SerialB = static_cast<uint32>(Candidate.Particle->UniqueIdx().Idx);
					Pair.LeafATM = LeafATM;
					Pair.Contacts = MoveTemp(Contacts);
				}
			}
		}
	}

	const FPerShapeData* GetCandidateShape(const FChronoUnrealCandidate& Candidate)
	{
		if (!Candidate.Particle)
		{
			return nullptr;
		}
		const FShapesArray& Shapes = Candidate.Particle->ShapesArray();
		return Shapes.IsValidIndex(Candidate.ShapeIndex) ? Shapes[Candidate.ShapeIndex].Get() : nullptr;
	}

	void GatherCandidatesInBounds(const ChCollisionSystemChaos::Impl& Impl, const FAABB3& QueryBounds, TArray<FChronoUnrealCandidate>& Out)
	{
		if (!Impl.IsBound())
		{
			return;
		}
		const ISpatialAccelerationCollection<FAccelerationStructureHandle, FReal, 3>* Acceleration = Impl.Solver->GetInternalAccelerationStructure_Internal();
		if (!Acceleration)
		{
			return;
		}
		FCandidateVisitor Visitor(Impl, QueryBounds, Out);
		Acceleration->Overlap(QueryBounds, Visitor);
	}

	void GatherCandidates(ChCollisionSystemChaos::Impl& Impl)
	{
		if (!Impl.IsBound() || !Impl.Params.bCollideWithUnrealGeometry)
		{
			return;
		}
		Impl.TimerBroad.start();
		for (ChCollisionModelChaos* Model : Impl.Proxies)
		{
			GatherProxyCandidates(Impl, *Model);
		}
		Impl.TimerBroad.stop();
	}

	void RunUnrealPairs(ChCollisionSystemChaos::Impl& Impl)
	{
		if (!Impl.IsBound() || !Impl.Params.bCollideWithUnrealGeometry)
		{
			return;
		}

		TArray<ChCollisionModelChaos*>& Proxies = Impl.Proxies;
		const int32 NumProxies = Proxies.Num();
		if (NumProxies == 0)
		{
			return;
		}

		// Fast movers that left their gathered region get a fresh candidate list (rare in the steady state).
		Impl.TimerBroad.start();
		for (ChCollisionModelChaos* Model : Proxies)
		{
			if (Model->GetData()->Particle && NeedsRegather(*Model->GetData()))
			{
				GatherProxyCandidates(Impl, *Model);
			}
		}
		Impl.TimerBroad.stop();

		Impl.TimerNarrow.start();
		TArray<TArray<ChCollisionSystemChaos::Impl::FPairContacts>>& ProxyContacts = Impl.ScratchUnrealContacts;
		if (ProxyContacts.Num() < NumProxies)
		{
			ProxyContacts.SetNum(NumProxies);
		}
		for (int32 i = 0; i < NumProxies; ++i)
		{
			ProxyContacts[i].Reset();
		}

		auto ProcessProxy = [&Impl, &Proxies, &ProxyContacts](int32 Index)
		{
			ChCollisionModelChaos& Model = *Proxies[Index];
			if (Model.GetData()->Particle && Model.GetData()->UnrealCandidates.Num() > 0)
			{
				NarrowphaseProxy(Impl, Model, ProxyContacts[Index]);
			}
		};

		const bool bParallel = NumProxies >= FMath::Max(Impl.Params.ParallelThreshold, 2);
		PhysicsParallelFor(NumProxies, ProcessProxy, /*bForceSingleThreaded*/ !bParallel);

		for (int32 i = 0; i < NumProxies; ++i)
		{
			for (const ChCollisionSystemChaos::Impl::FPairContacts& Pair : ProxyContacts[i])
			{
				Impl.Manifolds.Update(Pair.Key, Pair.Contacts, Pair.LeafATM, Impl.Params.ContactBreakingThresholdCm, Impl.RunSerial);
			}
		}
		Impl.TimerNarrow.stop();
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Material cache (Impl member, defined here to keep the material includes in one place)
// ---------------------------------------------------------------------------------------------------------------------

chrono::ChCollisionShape* chrono::ChCollisionSystemChaos::Impl::GetMaterialShape(const Chaos::FPerShapeData& Shape)
{
	const Chaos::FChaosPhysicsMaterial* Material = nullptr;
	if (Solver && Shape.NumMaterials() > 0)
	{
		// Per-face material masks (triangle meshes) are not resolved: the first material stands for the whole shape.
		Material = Shape.GetMaterial(0).GetInternal(&Solver->GetSimMaterials());
	}

	if (const std::shared_ptr<ChCollisionShape>* Found = MaterialShapes.Find(Material))
	{
		return Found->get();
	}

	// Friction / restitution from the Unreal physical material; Chrono-only parameters keep their defaults.
	const float Friction = Material ? static_cast<float>(Material->Friction) : 0.5f;
	const float Restitution = Material ? static_cast<float>(Material->Restitution) : 0.1f;

	std::shared_ptr<ChContactMaterial> ChronoMaterial;
	const ChContactMethod Method = (Owner && Owner->GetSystem()) ? Owner->GetSystem()->GetContactMethod() : ChContactMethod::NSC;
	if (Method == ChContactMethod::SMC)
	{
		auto SMC = chrono_types::make_shared<ChContactMaterialSMC>();
		SMC->SetFriction(Friction);
		SMC->SetRestitution(Restitution);
		ChronoMaterial = SMC;
	}
	else
	{
		auto NSC = chrono_types::make_shared<ChContactMaterialNSC>();
		NSC->SetFriction(Friction);
		NSC->SetRestitution(Restitution);
		ChronoMaterial = NSC;
	}

	std::shared_ptr<ChCollisionShape> ChronoShape = chrono_types::make_shared<ChCollisionShape>(ChCollisionShape::Type::UNKNOWN_SHAPE, ChronoMaterial);
	MaterialShapes.Add(Material, ChronoShape);
	return ChronoShape.get();
}
