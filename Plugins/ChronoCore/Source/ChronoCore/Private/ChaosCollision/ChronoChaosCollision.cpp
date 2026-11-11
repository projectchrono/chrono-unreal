// FChronoChaosCollision - physics-thread binding of a chrono::ChSystem (CHAOS collision system) to a Chaos solver.

#include "ChronoChaosCollision.h"

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"
#include "ChaosCollision/ChronoChaosSceneProxy.h"
#include "ChaosCollision/ChronoChaosSceneQuery.h"
#include "ChronoCoreModule.h"

#include "Chaos/Capsule.h"
#include "Chaos/CollisionFilterData.h"
#include "Chaos/PhysicalMaterials.h"
#include "Chaos/Sphere.h"
#include "Framework/Threading.h"
#include "PBDRigidsSolver.h"

#include "ChronoIncludeBegin.h"
#include "chrono/physics/ChSystem.h"
#include "ChronoIncludeEnd.h"

namespace
{
	chrono::ChCollisionSystemChaos* GetChaosCollisionSystem(const chrono::ChSystem& System)
	{
		return dynamic_cast<chrono::ChCollisionSystemChaos*>(System.GetCollisionSystem().get());
	}

	/// Chaos shape filter from the Engine-free description in the bind params.
	Chaos::Filter::FShapeFilterData BuildShapeFilter(const FChronoChaosShapeFilter& In)
	{
		Chaos::EFilterFlags Flags = Chaos::EFilterFlags::None;
		if (In.bSimpleCollision)
		{
			Flags |= Chaos::EFilterFlags::SimpleCollision;
		}
		if (In.bComplexCollision)
		{
			Flags |= Chaos::EFilterFlags::ComplexCollision;
		}

		PRAGMA_DISABLE_INTERNAL_WARNINGS
		return Chaos::Filter::FShapeFilterBuilder()
			.SetCollisionChannelIndex(In.CollisionChannel)
			.SetBlockChannelMask(In.BlockChannels)
			.SetOverlapChannelMask(In.OverlapChannels)
			.SetMaskFilter(In.MaskFilter)
			.SetFilterFlags(Flags)
			.Build();
		PRAGMA_ENABLE_INTERNAL_WARNINGS
	}
}

bool FChronoChaosCollision::BindSolver(chrono::ChSystem& System, Chaos::FPBDRigidsSolver& Solver, const FChronoChaosBindParams& Params)
{
	Chaos::EnsureIsInPhysicsThreadContext();

	chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System);
	if (!CollisionSystem)
	{
		UE_LOG(LogChronoCore, Error,
		       TEXT("FChronoChaosCollision::BindSolver: the ChSystem does not use the CHAOS collision system ")
		       TEXT("(call System.SetCollisionSystemType(chrono::ChCollisionSystem::Type::CHAOS) first)."));
		return false;
	}

	chrono::ChCollisionSystemChaos::Impl& Impl = CollisionSystem->GetImpl();
	if (Impl.IsBound())
	{
		if (Impl.Solver != &Solver)
		{
			UE_LOG(LogChronoCore, Warning, TEXT("FChronoChaosCollision::BindSolver: rebinding a ChSystem to a different solver."));
			UnbindSolver(System);
		}
		else
		{
			Impl.Params = Params;
			return true;
		}
	}

	Impl.Solver = &Solver;
	Impl.Params = Params;
	Impl.ShapeFilter = BuildShapeFilter(Params.Filter);
	Impl.ChaosStepSerial = 0;
	Impl.LastProcessedChaosStepSerial = -1;

	// World proxy body: a fixed, collision-disabled ChBody whose collision model is reported as the "other" model
	// for contacts / ray hits against non-Chrono geometry. Collision is disabled so it is never mirrored into Chaos
	// itself; the model only exists to give Chrono contacts a valid ChContactable.
	{
		auto WorldBody = chrono_types::make_shared<chrono::ChBody>();
		WorldBody->SetName("ChronoChaosWorld");
		WorldBody->SetFixed(true);
		WorldBody->EnableCollision(false);

		auto WorldModel = chrono_types::make_shared<chrono::ChCollisionModel>();
		WorldBody->AddCollisionModel(WorldModel);

		System.AddBody(WorldBody);

		Impl.WorldBody = WorldBody;
		Impl.WorldModel = WorldModel;
	}

	// Models added while the system was inert are bound retroactively.
	for (const std::shared_ptr<chrono::ChCollisionModelChaos>& Model : Impl.Models)
	{
		Impl.BindModel(*Model);
	}

	UE_LOG(LogChronoCore, Log, TEXT("FChronoChaosCollision::BindSolver: bound ChSystem to Chaos solver (%d collision models, %d proxy particles)."),
	       Impl.Models.Num(), Impl.Proxies.Num());
	return true;
}

void FChronoChaosCollision::UnbindSolver(chrono::ChSystem& System)
{
	chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System);
	if (!CollisionSystem)
	{
		return;
	}

	chrono::ChCollisionSystemChaos::Impl& Impl = CollisionSystem->GetImpl();
	if (!Impl.IsBound())
	{
		return;
	}

	Chaos::EnsureIsInPhysicsThreadContext();

	for (const std::shared_ptr<chrono::ChCollisionModelChaos>& Model : Impl.Models)
	{
		Impl.UnbindModel(*Model);
	}

	if (Impl.WorldBody)
	{
		if (Impl.WorldBody->GetSystem() == &System)
		{
			System.RemoveBody(Impl.WorldBody);
		}
		Impl.WorldBody.reset();
		Impl.WorldModel.reset();
	}

	Impl.Manifolds.Reset();
	Impl.MaterialShapes.Reset();
	Impl.Solver = nullptr;
}

bool FChronoChaosCollision::IsBound(const chrono::ChSystem& System)
{
	const chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System);
	return CollisionSystem && CollisionSystem->IsSceneBound();
}

void FChronoChaosCollision::BeginChaosStep(chrono::ChSystem& System, double ChaosDtS)
{
	if (chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System))
	{
		chrono::ChCollisionSystemChaos::Impl& Impl = CollisionSystem->GetImpl();
		++Impl.ChaosStepSerial;
		Impl.ChaosStepDtS = ChaosDtS;
	}
}

void FChronoChaosCollision::CommitKinematicTargets(chrono::ChSystem& System)
{
	if (chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System))
	{
		ChronoChaosSceneProxy::CommitKinematicTargets(CollisionSystem->GetImpl());
	}
}

int32 FChronoChaosCollision::GetNumProxyParticles(const chrono::ChSystem& System)
{
	if (const chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System))
	{
		return CollisionSystem->GetImpl().Proxies.Num();
	}
	return 0;
}

// ---------------------------------------------------------------------------------------------------------------------
// Terrain queries
// ---------------------------------------------------------------------------------------------------------------------

namespace
{
	const chrono::ChCollisionSystemChaos::Impl* GetBoundImpl(const chrono::ChSystem& System)
	{
		const chrono::ChCollisionSystemChaos* CollisionSystem = GetChaosCollisionSystem(System);
		if (!CollisionSystem || !CollisionSystem->GetImpl().IsBound())
		{
			return nullptr;
		}
		return &CollisionSystem->GetImpl();
	}

	/// Material fields of a hit from the Unreal shape (defaults for Chrono proxies / shapes without material).
	void FillMaterial(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FPerShapeData* Shape, FChronoChaosTerrainHit& Hit)
	{
		if (!Shape)
		{
			return;
		}
		if (const Chaos::FChaosPhysicsMaterial* Material = ChronoChaosSceneQuery::GetShapeMaterial(Impl, *Shape))
		{
			Hit.Friction = static_cast<float>(Material->Friction);
			Hit.Restitution = static_cast<float>(Material->Restitution);
			Hit.MaterialUserData = Material->UserData;
		}
	}

	void FillFromSweepHit(const chrono::ChCollisionSystemChaos::Impl& Impl, const ChronoChaosSceneQuery::FSweepHit& In, FChronoChaosTerrainHit& Out)
	{
		Out.bHit = true;
		Out.Point = In.Point;
		Out.Normal = In.Normal;
		Out.Distance = In.Time;
		Out.bStartPenetrating = In.bStartPenetrating;
		Out.bIsChronoBody = In.Model != nullptr;
		Out.ChronoModel = In.Model ? In.Model->GetModel() : nullptr;
		FillMaterial(Impl, In.ParticleShape, Out);
	}

	/// Direction / length of a segment; a degenerate segment sweeps "down" by zero length (pure overlap test).
	void SegmentDirLength(const FVector& Start, const FVector& End, FVector& OutDir, double& OutLength)
	{
		const FVector Delta = End - Start;
		OutLength = Delta.Size();
		OutDir = OutLength > UE_SMALL_NUMBER ? Delta / OutLength : -FVector::UpVector;
		if (OutLength <= UE_SMALL_NUMBER)
		{
			OutLength = 0.0;
		}
	}

	bool SweepShapeTerrain(const chrono::ChSystem& System, const Chaos::FImplicitObject& Swept, const FTransform& SweptPose, const FVector& EndUE,
	                       TArray<FChronoChaosTerrainHit>& OutHits, const FChronoChaosTerrainQueryParams& Params)
	{
		const chrono::ChCollisionSystemChaos::Impl* Impl = GetBoundImpl(System);
		if (!Impl)
		{
			return false;
		}

		FVector Dir;
		double Length;
		SegmentDirLength(SweptPose.GetLocation(), EndUE, Dir, Length);

		TArray<ChronoChaosSceneQuery::FSweepHit> Hits;
		ChronoChaosSceneQuery::SweepUnrealGeometry(*Impl, Swept, SweptPose, Dir, Length, Hits);
		if (Params.bIncludeChronoBodies)
		{
			ChronoChaosSceneQuery::SweepChronoProxies(*Impl, Swept, SweptPose, Dir, Length, Params.IgnoreModel, Hits);
		}
		if (Hits.Num() == 0)
		{
			return false;
		}

		Hits.Sort([](const ChronoChaosSceneQuery::FSweepHit& A, const ChronoChaosSceneQuery::FSweepHit& B) { return A.Time < B.Time; });
		OutHits.Reserve(OutHits.Num() + Hits.Num());
		for (const ChronoChaosSceneQuery::FSweepHit& Hit : Hits)
		{
			FillFromSweepHit(*Impl, Hit, OutHits.AddDefaulted_GetRef());
		}
		return true;
	}
}

bool FChronoChaosCollision::RaycastTerrain(const chrono::ChSystem& System, const FVector& StartUE, const FVector& EndUE,
                                           FChronoChaosTerrainHit& Out, const FChronoChaosTerrainQueryParams& Params)
{
	Out = FChronoChaosTerrainHit();

	const chrono::ChCollisionSystemChaos::Impl* Impl = GetBoundImpl(System);
	if (!Impl)
	{
		return false;
	}

	FVector Dir;
	double Length;
	SegmentDirLength(StartUE, EndUE, Dir, Length);
	if (Length <= 0.0)
	{
		return false;
	}

	ChronoChaosSceneQuery::FRayHit Best;
	ChronoChaosSceneQuery::RaycastUnrealGeometry(*Impl, StartUE, Dir, Length, Best);
	if (Params.bIncludeChronoBodies)
	{
		ChronoChaosSceneQuery::FRayHit ProxyBest;
		ProxyBest.Time = Best.Time;
		if (ChronoChaosSceneQuery::RaycastChronoProxies(*Impl, StartUE, Dir, Length, nullptr, ProxyBest)
			&& ProxyBest.Time < Best.Time
			&& !(Params.IgnoreModel && ProxyBest.Model && ProxyBest.Model->GetModel() == Params.IgnoreModel))
		{
			Best = ProxyBest;
		}
	}

	if (!Best.bHit)
	{
		return false;
	}

	Out.bHit = true;
	Out.Point = Best.Point;
	Out.Normal = Best.Normal;
	Out.Distance = Best.Time;
	Out.bIsChronoBody = Best.Model != nullptr;
	Out.ChronoModel = Best.Model ? Best.Model->GetModel() : nullptr;
	FillMaterial(*Impl, Best.ParticleShape, Out);
	return true;
}

bool FChronoChaosCollision::SweepCapsuleTerrain(const chrono::ChSystem& System, const FVector& StartUE, const FVector& EndUE, const FQuat& RotUE,
                                                float RadiusCm, float HalfHeightCm, TArray<FChronoChaosTerrainHit>& OutHits,
                                                const FChronoChaosTerrainQueryParams& Params)
{
	// Same convention as FCollisionShape::MakeCapsule: HalfHeight includes the caps, axis along local Z.
	const Chaos::FReal Radius = FMath::Max(static_cast<Chaos::FReal>(RadiusCm), static_cast<Chaos::FReal>(UE_KINDA_SMALL_NUMBER));
	const Chaos::FReal SegmentHalf = FMath::Max(static_cast<Chaos::FReal>(HalfHeightCm) - Radius, static_cast<Chaos::FReal>(0));
	const Chaos::FCapsule Capsule(Chaos::FVec3(0, 0, -SegmentHalf), Chaos::FVec3(0, 0, SegmentHalf), Radius);
	return SweepShapeTerrain(System, Capsule, FTransform(RotUE, StartUE), EndUE, OutHits, Params);
}

bool FChronoChaosCollision::SweepSphereTerrain(const chrono::ChSystem& System, const FVector& StartUE, const FVector& EndUE, float RadiusCm,
                                               TArray<FChronoChaosTerrainHit>& OutHits, const FChronoChaosTerrainQueryParams& Params)
{
	const Chaos::FSphere Sphere(Chaos::FVec3(0), FMath::Max(static_cast<Chaos::FReal>(RadiusCm), static_cast<Chaos::FReal>(UE_KINDA_SMALL_NUMBER)));
	return SweepShapeTerrain(System, Sphere, FTransform(StartUE), EndUE, OutHits, Params);
}
