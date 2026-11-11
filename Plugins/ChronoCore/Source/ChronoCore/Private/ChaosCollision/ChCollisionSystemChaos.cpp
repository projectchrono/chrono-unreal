// chrono::ChCollisionSystemChaos - Unreal-side implementation of the Chaos collision backend.
//
// Physics-thread code (see ChCollisionSystemChaosImpl.h). While no solver is bound the system is inert: every
// query returns "nothing" and a single warning is logged.

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"
#include "ChaosCollision/ChronoChaosCollisionPhase.h"
#include "ChaosCollision/ChronoChaosSceneProxy.h"
#include "ChaosCollision/ChronoChaosSceneQuery.h"
#include "ChaosCollision/ChronoChaosUnrealPhase.h"

#include "ChaosCollision/ChronoChaosNarrowphase.h"
#include "ChronoConversions.h"
#include "ChronoCoreModule.h"

#include "Chaos/Box.h"
#include "Chaos/Capsule.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/PhysicalMaterials.h"
#include "Chaos/ShapeInstance.h"
#include "Chaos/Sphere.h"
#include "Framework/Threading.h"
#include "PBDRigidsSolver.h"

#include "ChronoIncludeBegin.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChSystem.h"
#include "ChronoIncludeEnd.h"

namespace chrono
{
	// -----------------------------------------------------------------------------------------------------------------
	// Impl helpers
	// -----------------------------------------------------------------------------------------------------------------

	void ChCollisionSystemChaos::Impl::WarnUnboundOnce(const TCHAR* Operation)
	{
		if (bWarnedUnbound)
		{
			return;
		}
		bWarnedUnbound = true;
		UE_LOG(LogChronoCore, Warning,
		       TEXT("ChCollisionSystemChaos::%s: no Chaos solver is bound - collision detection is inert (no contacts, no ray hits). ")
		       TEXT("Bind the system with FChronoChaosCollision::BindSolver() (done automatically by FChronoSimulation)."),
		       Operation);
	}

	ChCollisionModelChaos* ChCollisionSystemChaos::Impl::FindModel(const ChCollisionModel* Model) const
	{
		for (const std::shared_ptr<ChCollisionModelChaos>& Candidate : Models)
		{
			if (Candidate->GetModel() == Model)
			{
				return Candidate.get();
			}
		}
		return nullptr;
	}

	ChCollisionModelChaos* ChCollisionSystemChaos::Impl::FindModelForParticle(const Chaos::FGeometryParticleHandle* Particle) const
	{
		ChCollisionModelChaos* const* Found = ParticleToModel.Find(Particle);
		return Found ? *Found : nullptr;
	}

	void ChCollisionSystemChaos::Impl::BindModel(ChCollisionModelChaos& Model)
	{
		ChronoChaosSceneProxy::CreateProxy(*this, Model);
	}

	void ChCollisionSystemChaos::Impl::UnbindModel(ChCollisionModelChaos& Model)
	{
		Manifolds.RemoveObject(&Model);
		ChronoChaosSceneProxy::DestroyProxy(*this, Model);
	}

	// -----------------------------------------------------------------------------------------------------------------
	// ChCollisionSystemChaos
	// -----------------------------------------------------------------------------------------------------------------

	ChCollisionSystemChaos::ChCollisionSystemChaos() : m_impl(std::make_unique<Impl>())
	{
		m_impl->Owner = this;
	}

	ChCollisionSystemChaos::~ChCollisionSystemChaos()
	{
		Clear();
	}

	void ChCollisionSystemChaos::Clear()
	{
		// Like ChCollisionSystemBullet::Clear(): release the Chaos-side data and forget the models. The
		// ChCollisionModels themselves are never dereferenced here - ChSystem::~ChSystem() destroys the bodies (and
		// their collision models) before the collision system, so they may already be gone.
		for (const std::shared_ptr<ChCollisionModelChaos>& Model : m_impl->Models)
		{
			m_impl->UnbindModel(*Model);
		}
		m_impl->Models.Reset();
		m_impl->Proxies.Reset();
		m_impl->ParticleToModel.Reset();
		m_impl->Manifolds.Reset();
	}

	void ChCollisionSystemChaos::Add(std::shared_ptr<ChCollisionModel> model)
	{
		if (!model || model->HasImplementation())
		{
			return;
		}

		auto ChaosModel = std::make_shared<ChCollisionModelChaos>(model.get(), this);
		m_impl->Models.Add(ChaosModel);
		m_impl->BindModel(*ChaosModel);
	}

	void ChCollisionSystemChaos::Remove(std::shared_ptr<ChCollisionModel> model)
	{
		if (!model || !model->HasImplementation())
		{
			return;
		}

		const int32 Index = m_impl->Models.IndexOfByPredicate(
			[&model](const std::shared_ptr<ChCollisionModelChaos>& Candidate) { return Candidate->GetModel() == model.get(); });
		if (Index == INDEX_NONE)
		{
			// Implemented by another collision system; not ours to remove.
			return;
		}

		m_impl->UnbindModel(*m_impl->Models[Index]);
		m_impl->Models.RemoveAt(Index);
		model->RemoveImplementation();
	}

	void ChCollisionSystemChaos::PreProcess()
	{
	}

	void ChCollisionSystemChaos::Run()
	{
		m_impl->TimerBroad.reset();
		m_impl->TimerNarrow.reset();

		if (!m_impl->IsBound())
		{
			m_impl->WarnUnboundOnce(TEXT("Run"));
			return;
		}

		Chaos::EnsureIsInPhysicsThreadContext();
		++m_impl->RunSerial;

		// Phase A (once per Chaos step): candidates against Unreal geometry from the internal acceleration structure.
		if (m_impl->LastProcessedChaosStepSerial != m_impl->ChaosStepSerial)
		{
			m_impl->LastProcessedChaosStepSerial = m_impl->ChaosStepSerial;
			ChronoChaosUnrealPhase::GatherCandidates(*m_impl);
		}

		// Phase B (every substep): Chrono <-> Chrono exact sweep on the current poses, Chrono <-> Unreal narrowphase.
		ChronoChaosCollisionPhase::RunChronoPairs(*m_impl, broad_callback.get());
		ChronoChaosUnrealPhase::RunUnrealPairs(*m_impl);

		// Manifolds of pairs that produced nothing this substep are gone.
		m_impl->Manifolds.Purge(m_impl->RunSerial);
	}

	ChAABB ChCollisionSystemChaos::GetBoundingBox() const
	{
		ChAABB Box;   // inverted (empty) box
		for (const ChCollisionModelChaos* Model : m_impl->Proxies)
		{
			Box += Model->GetBoundingBox();
		}
		return Box;
	}

	void ChCollisionSystemChaos::ResetTimers()
	{
		m_impl->TimerBroad.reset();
		m_impl->TimerNarrow.reset();
	}

	double ChCollisionSystemChaos::GetTimerCollisionBroad() const
	{
		return m_impl->TimerBroad();
	}

	double ChCollisionSystemChaos::GetTimerCollisionNarrow() const
	{
		return m_impl->TimerNarrow();
	}

	void ChCollisionSystemChaos::ReportContacts(ChContactContainer* contact_container)
	{
		if (!contact_container)
		{
			return;
		}

		// Always bracket with Begin/EndAddContact so the container purges stale contacts even while inert.
		contact_container->BeginAddContact();
		if (m_impl->IsBound())
		{
			ChronoChaosCollisionPhase::ReportManifolds(*m_impl, *contact_container, narrow_callback.get());
		}
		contact_container->EndAddContact();
	}

	void ChCollisionSystemChaos::ReportProximities(ChProximityContainer* proximity_container)
	{
		// Not supported by the Chaos backend (see CHAOS_COVERAGE.md).
	}

	namespace
	{
		/// Shared ray-hit implementation: closest hit among Chrono proxies (optionally a single model) and, for
		/// scene-wide queries, the non-Chrono geometry of the solver. Chaos shapes are not inflated by the Chrono
		/// envelope (unlike Bullet's), so the hit point lies on the actual surface and needs no envelope shift.
		bool RayHitImpl(const ChCollisionSystemChaos::Impl& Impl,
		                const ChVector3d& from,
		                const ChVector3d& to,
		                const ChCollisionModel* OnlyModel,
		                ChCollisionSystem::ChRayhitResult& result)
		{
			result.hit = false;
			result.hitModel = nullptr;
			result.dist_factor = 0.0;

			const FVector Start = FChronoConversions::ToUnrealPos(from);
			const FVector End = FChronoConversions::ToUnrealPos(to);
			FVector Dir = End - Start;
			const double Length = Dir.Size();
			if (Length <= UE_DOUBLE_SMALL_NUMBER)
			{
				return false;
			}
			Dir /= Length;

			ChronoChaosSceneQuery::FRayHit Best;
			Best.Time = Length;
			ChronoChaosSceneQuery::RaycastChronoProxies(Impl, Start, Dir, Length, OnlyModel, Best);
			if (!OnlyModel && Impl.Params.bCollideWithUnrealGeometry)
			{
				ChronoChaosSceneQuery::RaycastUnrealGeometry(Impl, Start, Dir, Length, Best);
			}

			if (!Best.bHit)
			{
				return false;
			}

			result.hit = true;
			result.abs_hitPoint = FChronoConversions::ToChronoPos(Best.Point);
			result.abs_hitNormal = FChronoConversions::ToChronoDir(Best.Normal);
			result.abs_hitNormal.Normalize();
			result.dist_factor = FMath::Clamp(Best.Time / Length, 0.0, 1.0);
			result.hitModel = Best.Model ? Best.Model->GetModel() : Impl.WorldModel.get();
			return true;
		}
	}

	bool ChCollisionSystemChaos::RayHit(const ChVector3d& from, const ChVector3d& to, ChRayhitResult& result) const
	{
		if (!m_impl->IsBound())
		{
			m_impl->WarnUnboundOnce(TEXT("RayHit"));
			result.hit = false;
			result.hitModel = nullptr;
			result.dist_factor = 0.0;
			return false;
		}
		return RayHitImpl(*m_impl, from, to, nullptr, result);
	}

	bool ChCollisionSystemChaos::RayHit(const ChVector3d& from,
	                                    const ChVector3d& to,
	                                    ChCollisionModel* model,
	                                    ChRayhitResult& result) const
	{
		if (!m_impl->IsBound())
		{
			m_impl->WarnUnboundOnce(TEXT("RayHit"));
			result.hit = false;
			result.hitModel = nullptr;
			result.dist_factor = 0.0;
			return false;
		}
		if (!model)
		{
			result.hit = false;
			result.hitModel = nullptr;
			result.dist_factor = 0.0;
			return false;
		}
		return RayHitImpl(*m_impl, from, to, model, result);
	}

	bool ChCollisionSystemChaos::ShapeContactTest(ChCollisionModel& query_model,
	                                              const ChFrame<>& frame,
	                                              std::vector<ChContactQueryHit>& hits,
	                                              short int filter_group,
	                                              short int filter_mask) const
	{
		Impl& I = *m_impl;
		if (!I.IsBound())
		{
			I.WarnUnboundOnce(TEXT("ShapeContactTest"));
			return false;
		}
		Chaos::EnsureIsInPhysicsThreadContext();

		// Chaos geometry of the query model (cached, rebuilt when its shape count changes).
		Impl::FQueryGeometry& Query = I.QueryGeometries.FindOrAdd(&query_model);
		if (!Query.Geometry.IsValid() || Query.NumShapes != static_cast<int32>(query_model.GetNumShapes()))
		{
			ChronoChaosShapeFactory::BuildGeometry(query_model, Query.Geometry);
			Query.NumShapes = static_cast<int32>(query_model.GetNumShapes());
		}
		if (!Query.Geometry.IsValid())
		{
			return false;
		}

		const FTransform QueryTM = FChronoConversions::ToUnrealTransform(frame);
		const double CullDistance = FChronoConversions::ToUnrealSize(query_model.GetEnvelope());
		Chaos::FAABB3 QueryBounds = Query.Geometry.LocalBounds.TransformedAABB(QueryTM);
		QueryBounds.Thicken(static_cast<Chaos::FReal>(CullDistance));
		ChContactable* QueryContactable = query_model.GetContactable();

		const size_t NumBefore = hits.size();

		// Contacts of one query leaf against one other leaf (Unreal frame) -> hits (Chrono frame). Normals are
		// reported from the other object towards the query shape; only penetrating points count.
		auto AppendHits = [&](const FChronoShapeSlot& QuerySlot, const Chaos::FImplicitObject& OtherLeaf, const FTransform& OtherTM,
		                      ChCollisionModel* OtherModel, ChCollisionShape* OtherShape, float Mu, const void* OtherId, double OtherSweptRadiusCm = 0.0)
		{
			const FTransform QueryLeafTM = QuerySlot.LocalTransform * QueryTM;
			ChronoChaosNarrowphase::FLeafParams QueryParams, OtherParams;
			QueryParams.SweptRadiusCm = QuerySlot.SweptRadiusCm;
			QueryParams.CylinderRadiusCm = QuerySlot.CylinderRadiusCm;
			QueryParams.CylinderHalfHeightCm = QuerySlot.CylinderHalfHeightCm;
			OtherParams.SweptRadiusCm = OtherSweptRadiusCm;
			ChronoChaosNarrowphase::FContactArray Contacts;
			ChronoChaosNarrowphase::GenerateContacts(*QuerySlot.Leaf, QueryLeafTM, OtherLeaf, OtherTM, CullDistance, Contacts, QueryParams, OtherParams);
			for (const ChronoChaosNarrowphase::FContact& C : Contacts)
			{
				if (C.Separation >= 0.0)
				{
					continue;
				}
				ChContactQueryHit Hit;
				Hit.model = OtherModel;
				Hit.shape = OtherShape;
				Hit.point = FChronoConversions::ToChronoPos(C.PointB);
				Hit.normal = FChronoConversions::ToChronoDir(-C.Normal);
				Hit.normal.Normalize();
				Hit.depth = FChronoConversions::ToChronoSize(-C.Separation);
				Hit.mu = Mu;
				Hit.other_id = OtherId;
				hits.push_back(Hit);
			}
		};

		// Chrono proxies (current Chrono poses), with the same filters as regular pairs.
		for (const ChCollisionModelChaos* Proxy : I.Proxies)
		{
			const ChCollisionModelChaos::Data* Data = Proxy->GetData();
			ChCollisionModel* OtherModel = Proxy->GetModel();
			if (!Data || !Data->Particle || !OtherModel || Data->WorldBounds.IsEmpty() || !Data->WorldBounds.Intersects(QueryBounds))
			{
				continue;
			}
			if (QueryContactable && OtherModel->GetContactable() == QueryContactable)
			{
				continue;
			}
			if ((filter_group & OtherModel->GetFamilyMask()) == 0 || (OtherModel->GetFamilyGroup() & filter_mask) == 0)
			{
				continue;
			}

			for (const FChronoShapeSlot& OtherSlot : Data->Geometry.Slots)
			{
				float Mu = 0.8f;
				if (OtherSlot.Shape && OtherSlot.Shape->GetMaterial())
				{
					Mu = OtherSlot.Shape->GetMaterial()->GetSlidingFriction();
				}
				const FTransform OtherTM = ChronoChaosCollisionPhase::LeafWorldTransform(*Data, OtherSlot);
				for (const FChronoShapeSlot& QuerySlot : Query.Geometry.Slots)
				{
					AppendHits(QuerySlot, *OtherSlot.Leaf, OtherTM, OtherModel, OtherSlot.Shape, Mu, Proxy, OtherSlot.SweptRadiusCm);
				}
			}
		}

		// Unreal geometry through the internal acceleration structure.
		if (I.Params.bCollideWithUnrealGeometry && I.WorldModel)
		{
			TArray<FChronoUnrealCandidate> Candidates;
			ChronoChaosUnrealPhase::GatherCandidatesInBounds(I, QueryBounds, Candidates);
			for (const FChronoUnrealCandidate& Candidate : Candidates)
			{
				const Chaos::FPerShapeData* Shape = ChronoChaosUnrealPhase::GetCandidateShape(Candidate);
				const Chaos::FImplicitObject* Leaf = Shape ? Shape->GetLeafGeometry() : nullptr;
				if (!Leaf)
				{
					continue;
				}
				ChCollisionShape* MaterialShape = I.GetMaterialShape(*Shape);
				const float Mu = MaterialShape && MaterialShape->GetMaterial() ? MaterialShape->GetMaterial()->GetSlidingFriction() : 0.8f;
				const FTransform OtherTM(Shape->GetLeafWorldTransform(Candidate.Particle));
				for (const FChronoShapeSlot& QuerySlot : Query.Geometry.Slots)
				{
					AppendHits(QuerySlot, *Leaf, OtherTM, I.WorldModel.get(), MaterialShape, Mu, Candidate.Particle);
				}
			}
		}

		return hits.size() > NumBefore;
	}

	namespace
	{
		/// Draw a wireframe box (Unreal frame, cm) through the Chrono visualization callback.
		void DrawWireBox(ChCollisionSystem::VisualizationCallback& Vis, const FTransform& TM, const FVector& Min, const FVector& Max, const ChColor& Color)
		{
			FVector Corners[8];
			for (int32 i = 0; i < 8; ++i)
			{
				Corners[i] = TM.TransformPosition(FVector((i & 1) ? Max.X : Min.X, (i & 2) ? Max.Y : Min.Y, (i & 4) ? Max.Z : Min.Z));
			}
			static const int32 Edges[12][2] = { {0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7} };
			for (const auto& Edge : Edges)
			{
				Vis.DrawLine(FChronoConversions::ToChronoPos(Corners[Edge[0]]), FChronoConversions::ToChronoPos(Corners[Edge[1]]), Color);
			}
		}

		/// Draw three great circles of a sphere.
		void DrawWireSphere(ChCollisionSystem::VisualizationCallback& Vis, const FVector& Centre, double Radius, const ChColor& Color)
		{
			constexpr int32 Segments = 16;
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				FVector Prev;
				for (int32 i = 0; i <= Segments; ++i)
				{
					const double A = 2.0 * PI * i / Segments;
					FVector P(0.0);
					P[(Axis + 1) % 3] = Radius * FMath::Cos(A);
					P[(Axis + 2) % 3] = Radius * FMath::Sin(A);
					P += Centre;
					if (i > 0)
					{
						Vis.DrawLine(FChronoConversions::ToChronoPos(Prev), FChronoConversions::ToChronoPos(P), Color);
					}
					Prev = P;
				}
			}
		}
	}

	void ChCollisionSystemChaos::Visualize(int flags)
	{
		if (!vis_callback || !m_impl->IsBound())
		{
			return;
		}
		VisualizationCallback& Vis = *vis_callback;
		const ChColor ShapeColor(0.0f, 1.0f, 0.0f);
		const ChColor AabbColor(1.0f, 1.0f, 0.0f);
		const ChColor ContactColor(1.0f, 0.0f, 0.0f);

		for (const ChCollisionModelChaos* Proxy : m_impl->Proxies)
		{
			const ChCollisionModelChaos::Data* Data = Proxy->GetData();
			if (!Data || !Data->Particle)
			{
				continue;
			}

			if ((flags & VIS_Aabb) && !Data->WorldBounds.IsEmpty())
			{
				DrawWireBox(Vis, FTransform::Identity, FVector(Data->WorldBounds.Min()), FVector(Data->WorldBounds.Max()), AabbColor);
			}

			if (flags & VIS_Shapes)
			{
				for (const FChronoShapeSlot& Slot : Data->Geometry.Slots)
				{
					const FTransform LeafTM = ChronoChaosCollisionPhase::LeafWorldTransform(*Data, Slot);
					if (const Chaos::FImplicitBox3* Box = Slot.Leaf->template GetObject<Chaos::FImplicitBox3>())
					{
						DrawWireBox(Vis, LeafTM, FVector(Box->Min()), FVector(Box->Max()), ShapeColor);
					}
					else if (const Chaos::FImplicitSphere3* Sphere = Slot.Leaf->template GetObject<Chaos::FImplicitSphere3>())
					{
						DrawWireSphere(Vis, LeafTM.TransformPosition(FVector(Sphere->GetCenterf())), Sphere->GetRadiusf(), ShapeColor);
					}
					else if (const Chaos::FImplicitCapsule3* Capsule = Slot.Leaf->template GetObject<Chaos::FImplicitCapsule3>())
					{
						const FVector X1 = LeafTM.TransformPosition(FVector(Capsule->GetX1f()));
						const FVector X2 = LeafTM.TransformPosition(FVector(Capsule->GetX2f()));
						DrawWireSphere(Vis, X1, Capsule->GetRadiusf(), ShapeColor);
						DrawWireSphere(Vis, X2, Capsule->GetRadiusf(), ShapeColor);
						Vis.DrawLine(FChronoConversions::ToChronoPos(X1), FChronoConversions::ToChronoPos(X2), ShapeColor);
					}
					else
					{
						// Convex hulls / meshes: their local bounds.
						const Chaos::FAABB3 Bounds = Slot.Leaf->BoundingBox();
						DrawWireBox(Vis, LeafTM, FVector(Bounds.Min()), FVector(Bounds.Max()), ShapeColor);
					}
				}
			}
		}

		if (flags & VIS_Contacts)
		{
			const double NormalScale = Vis.GetNormalScale();
			m_impl->Manifolds.ForEach([&](const FChronoManifoldKey&, const FChronoManifold& Manifold)
			{
				for (const FChronoManifoldPoint& Point : Manifold.Points)
				{
					const ChVector3d PA = FChronoConversions::ToChronoPos(Point.PointA);
					const ChVector3d PB = FChronoConversions::ToChronoPos(Point.PointB);
					ChVector3d N = FChronoConversions::ToChronoDir(Point.Normal);
					N.Normalize();
					Vis.DrawLine(PA, PB, ContactColor);
					Vis.DrawLine(PB, PB - N * NormalScale, ContactColor);
				}
			});
		}
	}

	void ChCollisionSystemChaos::ArchiveOut(ChArchiveOut& archive_out)
	{
		// Chaos state is not serializable (see CHAOS_COVERAGE.md).
	}

	void ChCollisionSystemChaos::ArchiveIn(ChArchiveIn& archive_in)
	{
		// Chaos state is not serializable (see CHAOS_COVERAGE.md).
	}

	bool ChCollisionSystemChaos::IsSceneBound() const
	{
		return m_impl->IsBound();
	}

	ChCollisionModel* ChCollisionSystemChaos::GetWorldProxyModel() const
	{
		return m_impl->WorldModel.get();
	}

	unsigned int ChCollisionSystemChaos::GetNumModels() const
	{
		return static_cast<unsigned int>(m_impl->Models.Num());
	}
}
