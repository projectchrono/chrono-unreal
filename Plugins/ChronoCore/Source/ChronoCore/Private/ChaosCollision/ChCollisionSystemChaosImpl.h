// Private, Unreal-side state of the Chaos collision backend (chrono::ChCollisionSystemChaos::Impl and
// chrono::ChCollisionModelChaos::Data). Only the ChronoCore plugin sources include this header.
//
// Everything here is physics-thread data: it is created/mutated by FChronoChaosCollision::BindSolver() and by the
// collision system callbacks (Run, ReportContacts, RayHit, ...), which FChronoSimulation invokes from a Chaos sim
// callback. The one exception is the "unbound" state, in which the collision system may be stepped on any thread
// (it simply does nothing).

#pragma once

#include "CoreMinimal.h"

#include "ChronoChaosCollision.h"
#include "ChaosCollision/ChronoChaosManifold.h"
#include "ChaosCollision/ChronoChaosShapeFactory.h"

#include "Chaos/AABB.h"
#include "Chaos/CollisionFilterData.h"
#include "Chaos/ParticleHandleFwd.h"

#include "ChronoIncludeBegin.h"
#include "chrono/collision/chaos/ChCollisionSystemChaos.h"
#include "chrono/collision/chaos/ChCollisionModelChaos.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBody.h"
#include "ChronoIncludeEnd.h"

#include <atomic>
#include <memory>

namespace Chaos
{
	class FPBDRigidsSolver;
	class FPerShapeData;
	class FChaosPhysicsMaterial;
}

namespace chrono
{
	class ChContactMaterial;

	/// A non-Chrono (Unreal) shape found near a proxy by the per-Chaos-step broadphase.
	struct FChronoUnrealCandidate
	{
		const Chaos::FGeometryParticleHandle* Particle = nullptr;
		int32 ShapeIndex = INDEX_NONE;
	};

	/// Unreal-side data of one Chrono collision model: its Chaos geometry and its kinematic proxy particle.
	/// Creation order of collision models (deterministic, unlike their addresses).
	inline uint32 NextModelSerial()
	{
		static std::atomic<uint32> Counter{0};
		return ++Counter;
	}

	struct ChCollisionModelChaos::Data
	{
		/// Creation order of this model; orders pairs and contacts reproducibly.
		uint32 SerialId = NextModelSerial();

		/// Absolute pose of the model in Unreal units / frame, refreshed by SyncPosition() every Chrono substep.
		FTransform PoseUE = FTransform::Identity;

		/// World-space bounds of the geometry at PoseUE (Unreal cm), refreshed by SyncPosition().
		Chaos::FAABB3 WorldBounds = Chaos::FAABB3::EmptyAABB();

		/// Chaos geometry built from the model's shapes (may be empty when no shape is supported).
		FChronoChaosGeometry Geometry;
		bool bGeometryBuilt = false;

		/// Proxy particle in the bound solver: a rigid particle in the Kinematic object state (like Unreal's own
		/// kinematic bodies). Null while unbound or when the geometry is empty.
		Chaos::FPBDRigidParticleHandle* Particle = nullptr;

		/// Pose last pushed to the solver (kinematic target), to skip redundant updates.
		FTransform CommittedPoseUE = FTransform::Identity;
		bool bPoseCommitted = false;

		/// Non-Chrono shapes near this proxy, gathered once per Chaos step from the solver's internal acceleration
		/// structure and reused by every Chrono substep of that step.
		TArray<FChronoUnrealCandidate> UnrealCandidates;

		/// Pose at which UnrealCandidates was gathered (to detect proxies that moved farther than the inflation).
		FVector CandidateGatherLocation = FVector::ZeroVector;
		double CandidateInflationCm = 0.0;
	};

	/// Unreal-side state of the Chaos collision system.
	struct ChCollisionSystemChaos::Impl
	{
		/// Owning collision system.
		ChCollisionSystemChaos* Owner = nullptr;

		/// Bound Chaos solver (null while inert).
		Chaos::FPBDRigidsSolver* Solver = nullptr;

		/// Parameters of the current binding.
		FChronoChaosBindParams Params;

		/// Shape filter built from Params.Filter at bind time.
		Chaos::Filter::FShapeFilterData ShapeFilter;

		/// Every collision model added to the system (with or without a proxy particle).
		TArray<std::shared_ptr<ChCollisionModelChaos>> Models;

		/// Models that currently own a proxy particle (subset of Models, stable order).
		TArray<ChCollisionModelChaos*> Proxies;

		/// Particle handles owned by this backend, for O(1) "is this one of ours" tests in scene queries.
		TMap<const Chaos::FGeometryParticleHandle*, ChCollisionModelChaos*> ParticleToModel;

		/// Fixed, collision-disabled body owned by the binding; its collision model is reported as 'modelB' /
		/// 'hitModel' for contacts and ray hits against non-Chrono geometry.
		std::shared_ptr<ChBody> WorldBody;
		std::shared_ptr<ChCollisionModel> WorldModel;

		/// Bumped by FChronoChaosCollision::BeginChaosStep(); Run() refreshes its Unreal candidate caches when it
		/// observes a new value.
		int32 ChaosStepSerial = 0;
		int32 LastProcessedChaosStepSerial = -1;

		/// Bumped by every Run(); manifolds not refreshed in the current run are purged.
		int32 RunSerial = 0;

		/// Persistent contact manifolds (Chrono <-> Chrono and Chrono <-> Unreal shape pairs).
		FChronoManifoldCache Manifolds;

		/// Per-Chaos-material Chrono shapes (UNKNOWN_SHAPE carrying a ChContactMaterial of the system's contact method)
		/// reported as 'shapeB' for contacts against Unreal geometry. Keyed by the solver-side material pointer; the
		/// null key is the default material.
		TMap<const Chaos::FChaosPhysicsMaterial*, std::shared_ptr<ChCollisionShape>> MaterialShapes;

		/// Shape (with material) to report for a non-Chrono shape. Creates the cache entry on first use.
		ChCollisionShape* GetMaterialShape(const Chaos::FPerShapeData& Shape);

		/// Chaos geometry of stand-alone query models (ShapeContactTest), rebuilt when their shape count changes.
		struct FQueryGeometry
		{
			FChronoChaosGeometry Geometry;
			int32 NumShapes = 0;
		};
		TMap<const ChCollisionModel*, FQueryGeometry> QueryGeometries;

		/// Speed-based inflation of the candidate query bounds (cm) for the current Chaos step.
		double ChaosStepDtS = 0.0;

		/// Proxy pair found by the broadphase (indices into Proxies, A < B).
		struct FProxyPair
		{
			int32 A;
			int32 B;
		};

		/// Fresh contacts of one leaf pair, produced by the (possibly parallel) narrowphase and merged serially.
		struct FPairContacts
		{
			FChronoManifoldKey Key;
			FTransform LeafATM;
			ChronoChaosNarrowphase::FContactArray Contacts;
		};

		// Scratch buffers reused across Run() calls (no allocations in the steady state).
		TArray<FProxyPair> ScratchPairs;
		TArray<TArray<FPairContacts>> ScratchPairContacts;
		TArray<TArray<FPairContacts>> ScratchUnrealContacts;
		TArray<int32> ScratchSortedProxies;

		ChTimer TimerBroad;
		ChTimer TimerNarrow;

		/// The "collision system is inert" warning is logged once per system instance.
		bool bWarnedUnbound = false;

		bool IsBound() const { return Solver != nullptr; }

		/// Log (once) that the system is stepped without a bound solver.
		void WarnUnboundOnce(const TCHAR* Operation);

		/// Find the implementation of 'Model' in Models (null if not managed by this system).
		ChCollisionModelChaos* FindModel(const ChCollisionModel* Model) const;

		/// Model owning a solver particle, or null for non-Chrono (Unreal) particles.
		ChCollisionModelChaos* FindModelForParticle(const Chaos::FGeometryParticleHandle* Particle) const;

		/// Create the Chaos geometry / proxy particle for a model (no-op while unbound or when it already has one).
		void BindModel(ChCollisionModelChaos& Model);

		/// Destroy the proxy particle of a model (no-op when it has none).
		void UnbindModel(ChCollisionModelChaos& Model);
	};
}
