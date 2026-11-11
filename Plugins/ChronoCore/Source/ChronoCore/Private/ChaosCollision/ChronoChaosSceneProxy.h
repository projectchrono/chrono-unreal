// Physics-thread kinematic proxy particles that mirror Chrono collision models into the bound Chaos solver.
//
// Proxies are created directly in the solver's particle SOAs (no game-thread FSingleParticlePhysicsProxy): Unreal game
// code never sees them, but Chaos' own pipeline does - Unreal dynamics are pushed by them like by any other kinematic,
// and they are part of the solver's internal spatial acceleration structure used for scene queries on the physics
// thread. All functions here are physics-thread only.

#pragma once

#include "CoreMinimal.h"

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"

namespace ChronoChaosSceneProxy
{
	/// Build the geometry of 'Model' (once) and create its kinematic particle in Impl.Solver at the model's current
	/// pose. Returns false when the model has no supported shape (no particle is created).
	bool CreateProxy(chrono::ChCollisionSystemChaos::Impl& Impl, chrono::ChCollisionModelChaos& Model);

	/// Destroy the particle of 'Model' (no-op when it has none). Geometry is kept for re-binding.
	void DestroyProxy(chrono::ChCollisionSystemChaos::Impl& Impl, chrono::ChCollisionModelChaos& Model);

	/// Push the current pose of every proxy whose pose changed as a kinematic target (Chaos moves the particle,
	/// derives its velocity and refreshes the acceleration structure during the step). Once per Chaos step.
	void CommitKinematicTargets(chrono::ChCollisionSystemChaos::Impl& Impl);

	/// Convert an Unreal-space AABB (cm) to a Chrono AABB (m).
	chrono::ChAABB ToChronoAABB(const Chaos::FAABB3& Bounds);

	/// Refresh Data.WorldBounds from Data.PoseUE and the geometry's local bounds.
	void UpdateWorldBounds(chrono::ChCollisionModelChaos::Data& Data);
}
