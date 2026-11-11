// Unreal-side entry point of the Chaos collision backend for Project Chrono (chrono::ChCollisionSystem::Type::CHAOS).
//
// A chrono::ChSystem whose collision system is a ChCollisionSystemChaos is inert (no contacts, no ray hits) until
// it is bound to a Chaos rigid-body solver with FChronoChaosCollision::BindSolver(). Once bound, every Chrono
// collision model is mirrored as a kinematic proxy particle in that solver, so Chrono bodies collide with each other
// and with the Unreal level (static meshes, landscapes, rigid bodies).
//
// THREADING: everything in this file runs on the Chaos physics thread (inside a Chaos::ISimCallbackObject).
// FChronoSimulation (ChronoSimulation.h) hosts a ChSystem there and calls these functions for you; use them directly
// only when writing your own sim callback. Never call them from the game thread.
//
// This header only forward-declares Chaos / Chrono types so that it can be included from any module.

#pragma once

#include "CoreMinimal.h"

namespace chrono
{
	class ChSystem;
	class ChCollisionModel;
}

namespace Chaos
{
	class FPBDRigidsSolver;
}

/// One hit of a physics-thread terrain query (FChronoChaosCollision::RaycastTerrain / SweepCapsuleTerrain).
/// Everything in Unreal units / frame.
struct CHRONOCORE_API FChronoChaosTerrainHit
{
	bool bHit = false;

	/// Impact point on the hit surface (cm).
	FVector Point = FVector::ZeroVector;

	/// Surface normal at the impact point (unit length).
	FVector Normal = FVector::UpVector;

	/// Distance travelled along the ray / sweep until the hit (cm). 0 for an initial overlap of a sweep.
	double Distance = 0.0;

	/// True when the swept shape already overlapped the surface at the start (Point / Normal then come from the
	/// minimum translation direction, like Unreal's bStartPenetrating hits).
	bool bStartPenetrating = false;

	/// Sliding friction of the hit shape's physical material (Unreal UPhysicalMaterial::Friction; 0.8 if unknown).
	float Friction = 0.8f;

	/// Restitution of the hit shape's physical material (0.1 if unknown).
	float Restitution = 0.1f;

	/// Chaos::FChaosPhysicsMaterial::UserData of the hit shape's material. For materials created by the engine this
	/// is the address of UPhysicalMaterial::UserData, so game code can identify the physical material by pointer
	/// comparison against values captured on the game thread - without dereferencing any UObject on the physics
	/// thread. Null for Chrono bodies and shapes without a material.
	const void* MaterialUserData = nullptr;

	/// The hit belongs to a Chrono body (proxy) instead of Unreal geometry.
	bool bIsChronoBody = false;

	/// Chrono collision model hit (only for bIsChronoBody).
	const chrono::ChCollisionModel* ChronoModel = nullptr;
};

/// Options of the physics-thread terrain queries.
struct CHRONOCORE_API FChronoChaosTerrainQueryParams
{
	/// Also test the Chrono bodies mirrored into the solver. Off by default: tires querying the ground must not hit
	/// their own chassis.
	bool bIncludeChronoBodies = false;

	/// Ignore this Chrono model even when bIncludeChronoBodies is set (e.g. the querying body).
	const chrono::ChCollisionModel* IgnoreModel = nullptr;
};

/// Collision filter applied to the shapes of every Chrono proxy particle. Mirrors the fields of Unreal's
/// FPhysicsFilterBuilder / Chaos::Filter::FShapeFilterData without depending on Engine or Chaos headers.
struct CHRONOCORE_API FChronoChaosShapeFilter
{
	/// Collision channel the proxies belong to (ECollisionChannel index; 5 = ECC_PhysicsBody).
	uint8 CollisionChannel = 5;

	/// Channels the proxies block (bit i = channel i). Default: everything.
	uint64 BlockChannels = ~0ull;

	/// Channels the proxies overlap (query only). Default: none.
	uint64 OverlapChannels = 0;

	/// Extra mask filter (FMaskFilter). Default: none.
	uint8 MaskFilter = 0;

	/// Shapes take part in simple collision.
	bool bSimpleCollision = true;

	/// Shapes take part in complex (per-triangle) collision.
	bool bComplexCollision = true;
};

/// Parameters of a solver binding.
struct CHRONOCORE_API FChronoChaosBindParams
{
	/// Filter of the proxy shapes (what Chaos pairs the proxies with, and which traces see them).
	FChronoChaosShapeFilter Filter;

	/// Proxy shapes collide with Unreal rigid bodies inside Chaos (kinematic proxies push dynamics).
	bool bProxySimEnabled = true;

	/// Proxy shapes are visible to Chaos scene queries issued on the physics thread.
	bool bProxyQueryEnabled = true;

	/// Persistent manifold points whose separation exceeds this distance (Unreal cm) are dropped.
	float ContactBreakingThresholdCm = 2.0f;

	/// Generate Chrono contacts against non-Chrono geometry found in the Chaos scene.
	bool bCollideWithUnrealGeometry = true;

	/// Process proxies in parallel (PhysicsParallelFor) when at least this many exist.
	int32 ParallelThreshold = 8;
};

/// Physics-thread helpers that connect a chrono::ChSystem (with a CHAOS collision system) to a Chaos solver.
struct CHRONOCORE_API FChronoChaosCollision
{
	/// Bind the CHAOS collision system of 'System' to 'Solver'. Creates the fixed, collision-disabled world proxy
	/// body used to report contacts against Unreal geometry and mirrors models added before the binding.
	/// Returns false (and logs an error) if the system's collision system is not a ChCollisionSystemChaos.
	/// Physics thread only.
	static bool BindSolver(chrono::ChSystem& System, Chaos::FPBDRigidsSolver& Solver, const FChronoChaosBindParams& Params);

	/// Destroy every proxy particle in the solver and remove the world proxy body. Safe to call when not bound.
	/// Physics thread only.
	static void UnbindSolver(chrono::ChSystem& System);

	/// True if the system's collision system is a ChCollisionSystemChaos bound to a solver.
	static bool IsBound(const chrono::ChSystem& System);

	/// Notify the backend that a new Chaos step of 'ChaosDtS' seconds begins (candidate caches against Unreal
	/// geometry are refreshed on the next Run(); the step length sizes their velocity inflation). Call once per Chaos
	/// step before the Chrono substeps. Physics thread only.
	static void BeginChaosStep(chrono::ChSystem& System, double ChaosDtS = 0.0);

	/// Push the current poses of all proxies to the solver as kinematic targets. Call once per Chaos step after
	/// the last Chrono substep. Physics thread only.
	static void CommitKinematicTargets(chrono::ChSystem& System);

	/// Number of proxy particles currently owned by the system's collision backend (0 when unbound).
	static int32 GetNumProxyParticles(const chrono::ChSystem& System);

	// --- terrain queries (physics thread, system must be bound) -----------------------------------------------------
	//
	// These look at the same geometry Chrono collides with: every non-Chrono shape of the bound solver that blocks the
	// proxy filter (landscapes, static meshes, rigid bodies), optionally plus the Chrono proxies. They are meant for
	// terrain sensing (chrono::vehicle tire / terrain functors) and never touch UObjects.

	/// Closest hit along the segment StartUE -> EndUE (cm). Returns false (Out.bHit = false) when nothing is hit or
	/// the system is not bound.
	static bool RaycastTerrain(const chrono::ChSystem& System, const FVector& StartUE, const FVector& EndUE,
	                           FChronoChaosTerrainHit& Out, const FChronoChaosTerrainQueryParams& Params = FChronoChaosTerrainQueryParams());

	/// Sweep a capsule (radius / half height in cm, axis along RotUE's Z like FCollisionShape::MakeCapsule) from StartUE
	/// to EndUE and report one hit per shape touched, sorted by distance (initial overlaps first). StartUE == EndUE
	/// degenerates to an overlap test whose hits carry the minimum-translation point / normal. Appends to OutHits and
	/// returns true if anything was hit.
	static bool SweepCapsuleTerrain(const chrono::ChSystem& System, const FVector& StartUE, const FVector& EndUE, const FQuat& RotUE,
	                                float RadiusCm, float HalfHeightCm, TArray<FChronoChaosTerrainHit>& OutHits,
	                                const FChronoChaosTerrainQueryParams& Params = FChronoChaosTerrainQueryParams());

	/// Same as SweepCapsuleTerrain with a sphere.
	static bool SweepSphereTerrain(const chrono::ChSystem& System, const FVector& StartUE, const FVector& EndUE, float RadiusCm,
	                               TArray<FChronoChaosTerrainHit>& OutHits,
	                               const FChronoChaosTerrainQueryParams& Params = FChronoChaosTerrainQueryParams());
};
