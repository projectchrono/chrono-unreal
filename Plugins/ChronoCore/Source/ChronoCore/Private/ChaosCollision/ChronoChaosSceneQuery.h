// Physics-thread scene queries of the Chaos collision backend: ray casts against Chrono proxies (with their current
// Chrono poses) and against the non-Chrono geometry of the bound solver (through its internal spatial acceleration
// structure). All in Unreal units / frame; the collision system converts to Chrono.

#pragma once

#include "CoreMinimal.h"

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"

namespace Chaos
{
	class FPerShapeData;
	class FImplicitObject;
	class FChaosPhysicsMaterial;
}

namespace ChronoChaosSceneQuery
{
	/// Result of a ray cast (Unreal cm / frame).
	struct FRayHit
	{
		bool bHit = false;
		/// Distance along the ray (cm).
		double Time = TNumericLimits<double>::Max();
		FVector Point = FVector::ZeroVector;
		FVector Normal = FVector::UpVector;
		/// Chrono model hit (proxy owner) or null for non-Chrono geometry.
		chrono::ChCollisionModelChaos* Model = nullptr;
		/// Chrono shape hit (proxy leaf) or null for non-Chrono geometry.
		chrono::ChCollisionShape* Shape = nullptr;
		/// Non-Chrono particle hit (null for Chrono proxies).
		const Chaos::FGeometryParticleHandle* Particle = nullptr;
		/// Non-Chrono shape hit (null for Chrono proxies).
		const Chaos::FPerShapeData* ParticleShape = nullptr;
	};

	/// True if Chrono should collide with / be blocked by this non-Chrono shape: sim enabled, valid filter, blocking
	/// response to the proxy filter, and (for triangle meshes) complex-as-simple collision.
	bool IsUnrealShapeCollidable(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FPerShapeData& Shape);

	/// Ray cast against one implicit object placed at 'Pose'. 'Dir' must be unit length.
	bool RaycastImplicit(const Chaos::FImplicitObject& Geometry, const FTransform& Pose, const FVector& Start, const FVector& Dir, double Length,
	                     double& OutTime, FVector& OutPoint, FVector& OutNormal);

	/// Closest hit among the Chrono proxies (all of them, or only 'OnlyModel' when non-null), using the models' current
	/// Chrono poses (fresher than the solver particles, which lag by one Chaos step).
	bool RaycastChronoProxies(const chrono::ChCollisionSystemChaos::Impl& Impl, const FVector& Start, const FVector& Dir, double Length,
	                          const chrono::ChCollisionModel* OnlyModel, FRayHit& InOutBest);

	/// Closest hit among non-Chrono particles of the bound solver (internal acceleration structure).
	bool RaycastUnrealGeometry(const chrono::ChCollisionSystemChaos::Impl& Impl, const FVector& Start, const FVector& Dir, double Length, FRayHit& InOutBest);

	/// Solver-side physical material of a non-Chrono shape (first material; null when the shape has none).
	const Chaos::FChaosPhysicsMaterial* GetShapeMaterial(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FPerShapeData& Shape);

	/// One hit of a shape sweep (Unreal cm / frame). Same attribution fields as FRayHit.
	struct FSweepHit
	{
		/// Distance along the sweep (cm); 0 for an initial overlap.
		double Time = 0.0;
		bool bStartPenetrating = false;
		FVector Point = FVector::ZeroVector;
		FVector Normal = FVector::UpVector;
		chrono::ChCollisionModelChaos* Model = nullptr;
		chrono::ChCollisionShape* Shape = nullptr;
		const Chaos::FGeometryParticleHandle* Particle = nullptr;
		const Chaos::FPerShapeData* ParticleShape = nullptr;
	};

	/// Sweep 'Swept' (posed at SweptPose, moving along the unit 'Dir' by 'Length') against every collidable non-Chrono
	/// shape of the bound solver, appending one hit per touched shape (with MTD point / normal for initial overlaps).
	bool SweepUnrealGeometry(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FImplicitObject& Swept, const FTransform& SweptPose,
	                         const FVector& Dir, double Length, TArray<FSweepHit>& OutHits);

	/// Same against the Chrono proxies (with their current Chrono poses), skipping 'IgnoreModel' when non-null.
	bool SweepChronoProxies(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FImplicitObject& Swept, const FTransform& SweptPose,
	                        const FVector& Dir, double Length, const chrono::ChCollisionModel* IgnoreModel, TArray<FSweepHit>& OutHits);
}
