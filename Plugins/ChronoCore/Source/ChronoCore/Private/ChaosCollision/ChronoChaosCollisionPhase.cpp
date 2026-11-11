// ChronoChaosCollisionPhase - Chrono <-> Chrono pair detection, narrowphase dispatch and contact reporting.

#include "ChaosCollision/ChronoChaosCollisionPhase.h"

#include "ChaosCollision/ChronoChaosNarrowphase.h"
#include "ChaosCollision/ChronoChaosUnrealPhase.h"
#include "ChronoConversions.h"

#include "Chaos/Framework/Parallel.h"
#include "Chaos/ImplicitObject.h"

#include "ChronoIncludeBegin.h"
#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactable.h"
#include "ChronoIncludeEnd.h"

namespace ChronoChaosCollisionPhase
{
	using namespace Chaos;
	using chrono::ChCollisionModelChaos;
	using chrono::ChCollisionSystemChaos;

	FTransform LeafWorldTransform(const ChCollisionModelChaos::Data& Data, const FChronoShapeSlot& Slot)
	{
		return Slot.LocalTransform * Data.PoseUE;
	}

	FAABB3 LeafWorldBounds(const ChCollisionModelChaos::Data& Data, const FChronoShapeSlot& Slot)
	{
		// Slot.Root is the TImplicitObjectTransformed wrapper: its bounds are already in model space.
		return Slot.Root->BoundingBox().TransformedAABB(Data.PoseUE);
	}

	double EnvelopeCm(const ChCollisionModelChaos& Model)
	{
		return Model.GetModel() ? FChronoConversions::ToUnrealSize(Model.GetModel()->GetEnvelope()) : 0.0;
	}

	namespace
	{
		/// Chrono-side filters shared by the Bullet backend: same contactable, both inactive, collision families.
		bool PassesChronoFilters(chrono::ChCollisionModel& A, chrono::ChCollisionModel& B)
		{
			chrono::ChContactable* ContactableA = A.GetContactable();
			chrono::ChContactable* ContactableB = B.GetContactable();
			if (!ContactableA || !ContactableB || ContactableA == ContactableB)
			{
				return false;
			}
			if (!ContactableA->IsContactActive() && !ContactableB->IsContactActive())
			{
				return false;   // two fixed / inactive objects never generate contacts
			}
			const bool bFamiliesCollide = (A.GetFamilyGroup() & B.GetFamilyMask()) != 0 && (B.GetFamilyGroup() & A.GetFamilyMask()) != 0;
			return bFamiliesCollide;
		}

		/// Narrowphase of all leaf pairs of two proxies; appends one FPairContacts per leaf pair with contacts.
		void NarrowphaseProxyPair(ChCollisionModelChaos& ModelA, ChCollisionModelChaos& ModelB, double CullDistanceCm, TArray<ChCollisionSystemChaos::Impl::FPairContacts>& Out)
		{
			const ChCollisionModelChaos::Data& DataA = *ModelA.GetData();
			const ChCollisionModelChaos::Data& DataB = *ModelB.GetData();

			for (int32 SlotIndexA = 0; SlotIndexA < DataA.Geometry.Slots.Num(); ++SlotIndexA)
			{
				const FChronoShapeSlot& SlotA = DataA.Geometry.Slots[SlotIndexA];
				FAABB3 BoundsA = LeafWorldBounds(DataA, SlotA);
				BoundsA.Thicken(static_cast<FReal>(CullDistanceCm));
				const FTransform LeafATM = LeafWorldTransform(DataA, SlotA);

				for (int32 SlotIndexB = 0; SlotIndexB < DataB.Geometry.Slots.Num(); ++SlotIndexB)
				{
					const FChronoShapeSlot& SlotB = DataB.Geometry.Slots[SlotIndexB];
					if (!BoundsA.Intersects(LeafWorldBounds(DataB, SlotB)))
					{
						continue;
					}

					ChronoChaosNarrowphase::FLeafParams ParamsA, ParamsB;
					ParamsA.SweptRadiusCm = SlotA.SweptRadiusCm;
					ParamsB.SweptRadiusCm = SlotB.SweptRadiusCm;
					ParamsA.CylinderRadiusCm = SlotA.CylinderRadiusCm;
					ParamsA.CylinderHalfHeightCm = SlotA.CylinderHalfHeightCm;
					ParamsB.CylinderRadiusCm = SlotB.CylinderRadiusCm;
					ParamsB.CylinderHalfHeightCm = SlotB.CylinderHalfHeightCm;
					ChronoChaosNarrowphase::FContactArray Contacts;
					ChronoChaosNarrowphase::GenerateContacts(*SlotA.Leaf, LeafATM, *SlotB.Leaf, LeafWorldTransform(DataB, SlotB), CullDistanceCm, Contacts, ParamsA, ParamsB);
					if (Contacts.Num() == 0)
					{
						continue;
					}
					ChronoChaosNarrowphase::ReduceContacts(Contacts, 4);

					ChCollisionSystemChaos::Impl::FPairContacts& Pair = Out.AddDefaulted_GetRef();
					Pair.Key.ObjectA = &ModelA;
					Pair.Key.ShapeA = SlotIndexA;
					Pair.Key.ObjectB = &ModelB;
					Pair.Key.ShapeB = SlotIndexB;
					Pair.Key.bUnrealB = false;
					Pair.Key.SerialA = DataA.SerialId;
					Pair.Key.SerialB = DataB.SerialId;
					Pair.LeafATM = LeafATM;
					Pair.Contacts = MoveTemp(Contacts);
				}
			}
		}
	}

	void RunChronoPairs(ChCollisionSystemChaos::Impl& Impl, chrono::ChCollisionSystem::BroadphaseCallback* BroadCallback)
	{
		TArray<ChCollisionModelChaos*>& Proxies = Impl.Proxies;
		const int32 NumProxies = Proxies.Num();
		Impl.ScratchPairs.Reset();
		if (NumProxies < 2)
		{
			return;
		}

		// ---- Broadphase: sort-and-sweep on the current (this substep) world bounds inflated by the envelope ----------
		Impl.TimerBroad.start();

		struct FSweepEntry
		{
			FAABB3 Bounds;
			int32 Proxy;
			bool bValid;
		};
		TArray<FSweepEntry, TInlineAllocator<64>> Entries;
		Entries.SetNumUninitialized(NumProxies);
		for (int32 i = 0; i < NumProxies; ++i)
		{
			const ChCollisionModelChaos::Data* Data = Proxies[i]->GetData();
			FSweepEntry& Entry = Entries[i];
			Entry.Proxy = i;
			Entry.bValid = Data && Data->Particle && !Data->WorldBounds.IsEmpty();
			Entry.Bounds = Entry.bValid ? Data->WorldBounds : FAABB3::EmptyAABB();
			if (Entry.bValid)
			{
				Entry.Bounds.Thicken(static_cast<FReal>(EnvelopeCm(*Proxies[i])));
			}
		}

		TArray<int32>& Sorted = Impl.ScratchSortedProxies;
		Sorted.Reset(NumProxies);
		for (int32 i = 0; i < NumProxies; ++i)
		{
			if (Entries[i].bValid)
			{
				Sorted.Add(i);
			}
		}
		Sorted.Sort([&Entries](int32 L, int32 R)
		{
			const FReal XL = Entries[L].Bounds.Min().X, XR = Entries[R].Bounds.Min().X;
			return XL != XR ? XL < XR : L < R;   // ties by proxy index: Sort is not stable
		});

		for (int32 si = 0; si < Sorted.Num(); ++si)
		{
			const FSweepEntry& EntryA = Entries[Sorted[si]];
			for (int32 sj = si + 1; sj < Sorted.Num(); ++sj)
			{
				const FSweepEntry& EntryB = Entries[Sorted[sj]];
				if (EntryB.Bounds.Min().X > EntryA.Bounds.Max().X)
				{
					break;
				}
				if (!EntryA.Bounds.Intersects(EntryB.Bounds))
				{
					continue;
				}

				ChCollisionModelChaos* ModelA = Proxies[EntryA.Proxy];
				ChCollisionModelChaos* ModelB = Proxies[EntryB.Proxy];
				if (!ModelA->GetModel() || !ModelB->GetModel() || !PassesChronoFilters(*ModelA->GetModel(), *ModelB->GetModel()))
				{
					continue;
				}
				if (BroadCallback && !BroadCallback->OnBroadphase(ModelA->GetModel(), ModelB->GetModel()))
				{
					continue;
				}

				// Stable ordering (by creation order, not by address: addresses change from run to run and with them the
				// contact order and the solver result) so a pair keeps its manifold identity when the proxy list is reordered.
				const bool bAFirst = ModelA->GetData()->SerialId < ModelB->GetData()->SerialId;
				ChCollisionSystemChaos::Impl::FProxyPair& Pair = Impl.ScratchPairs.AddDefaulted_GetRef();
				Pair.A = bAFirst ? EntryA.Proxy : EntryB.Proxy;
				Pair.B = bAFirst ? EntryB.Proxy : EntryA.Proxy;
			}
		}
		Impl.TimerBroad.stop();

		// ---- Narrowphase: per pair (parallel above the threshold), results merged serially ----------------------------
		Impl.TimerNarrow.start();

		const int32 NumPairs = Impl.ScratchPairs.Num();
		TArray<TArray<ChCollisionSystemChaos::Impl::FPairContacts>>& PairContacts = Impl.ScratchPairContacts;
		if (PairContacts.Num() < NumPairs)
		{
			PairContacts.SetNum(NumPairs);
		}
		for (int32 i = 0; i < NumPairs; ++i)
		{
			PairContacts[i].Reset();
		}

		auto ProcessPair = [&Impl, &Proxies, &PairContacts](int32 PairIndex)
		{
			const ChCollisionSystemChaos::Impl::FProxyPair& Pair = Impl.ScratchPairs[PairIndex];
			ChCollisionModelChaos& ModelA = *Proxies[Pair.A];
			ChCollisionModelChaos& ModelB = *Proxies[Pair.B];
			const double CullDistance = EnvelopeCm(ModelA) + EnvelopeCm(ModelB);
			NarrowphaseProxyPair(ModelA, ModelB, CullDistance, PairContacts[PairIndex]);
		};

		const bool bParallel = NumPairs >= FMath::Max(Impl.Params.ParallelThreshold, 2);
		PhysicsParallelFor(NumPairs, ProcessPair, /*bForceSingleThreaded*/ !bParallel);

		for (int32 i = 0; i < NumPairs; ++i)
		{
			for (const ChCollisionSystemChaos::Impl::FPairContacts& Pair : PairContacts[i])
			{
				Impl.Manifolds.Update(Pair.Key, Pair.Contacts, Pair.LeafATM, Impl.Params.ContactBreakingThresholdCm, Impl.RunSerial);
			}
		}
		Impl.TimerNarrow.stop();
	}

	void ReportManifolds(ChCollisionSystemChaos::Impl& Impl, chrono::ChContactContainer& Container, chrono::ChCollisionSystem::NarrowphaseCallback* NarrowCallback)
	{
		chrono::ChCollisionInfo Info;

		Impl.Manifolds.ForEach([&](const FChronoManifoldKey& Key, FChronoManifold& Manifold)
		{
			const ChCollisionModelChaos* ProxyA = static_cast<const ChCollisionModelChaos*>(Key.ObjectA);
			chrono::ChCollisionModel* ModelA = ProxyA ? ProxyA->GetModel() : nullptr;
			if (!ModelA || !ProxyA->GetData() || !ProxyA->GetData()->Geometry.Slots.IsValidIndex(Key.ShapeA))
			{
				return;
			}
			chrono::ChCollisionShape* ShapeA = ProxyA->GetData()->Geometry.Slots[Key.ShapeA].Shape;

			chrono::ChCollisionModel* ModelB = nullptr;
			chrono::ChCollisionShape* ShapeB = nullptr;
			if (Key.bUnrealB)
			{
				// Unreal geometry: the world proxy body stands in as the contactable, the shape carries the Unreal
				// physical material (friction / restitution) of the hit shape.
				chrono::FChronoUnrealCandidate Candidate;
				Candidate.Particle = static_cast<const Chaos::FGeometryParticleHandle*>(Key.ObjectB);
				Candidate.ShapeIndex = Key.ShapeB;
				const Chaos::FPerShapeData* UnrealShape = ChronoChaosUnrealPhase::GetCandidateShape(Candidate);
				if (!UnrealShape || !Impl.WorldModel)
				{
					return;
				}
				ModelB = Impl.WorldModel.get();
				ShapeB = Impl.GetMaterialShape(*UnrealShape);
			}
			else
			{
				const ChCollisionModelChaos* ProxyB = static_cast<const ChCollisionModelChaos*>(Key.ObjectB);
				ModelB = ProxyB ? ProxyB->GetModel() : nullptr;
				if (!ModelB || !ProxyB->GetData() || !ProxyB->GetData()->Geometry.Slots.IsValidIndex(Key.ShapeB))
				{
					return;
				}
				ShapeB = ProxyB->GetData()->Geometry.Slots[Key.ShapeB].Shape;
			}
			if (!ShapeA || !ShapeB)
			{
				return;
			}

			for (FChronoManifoldPoint& Point : Manifold.Points)
			{
				Info.modelA = ModelA;
				Info.modelB = ModelB;
				Info.shapeA = ShapeA;
				Info.shapeB = ShapeB;
				Info.vpA = FChronoConversions::ToChronoPos(Point.PointA);
				Info.vpB = FChronoConversions::ToChronoPos(Point.PointB);
				Info.vN = FChronoConversions::ToChronoDir(Point.Normal);
				Info.vN.Normalize();
				Info.distance = FChronoConversions::ToChronoSize(Point.Separation);
				Info.eff_radius = chrono::ChCollisionInfo::GetDefaultEffectiveCurvatureRadius();
				Info.reaction_cache = Point.ReactionCache;

				if (NarrowCallback && !NarrowCallback->OnNarrowphase(Info))
				{
					continue;
				}
				Container.AddContact(Info);
			}
		});
	}
}
