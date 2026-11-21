// ChronoChaosDynamicMirror - two-way coupling between Chrono bodies and dynamic Unreal (Chaos) rigid bodies.
//
// By default an Unreal rigid body is immovable for Chrono's solver (contacts are reported against the fixed world
// proxy body). With FChronoChaosBindParams::bTwoWayDynamics the dynamic particles the proxies collide with are instead
// mirrored by a collision-less chrono::ChBody carrying the particle's mass, inertia, pose and velocity:
//
//   * the mirror exists only while the particle is a broadphase candidate of some proxy (created / released by the
//     per-Chaos-step candidate gather, so particles that spawn, despawn or get pooled need no registration);
//   * it is re-synchronised from the particle once per Chaos step, integrated by Chrono during the substeps (so the
//     narrowphase sees the body where Chrono moved it) and solved with its real mass;
//   * after the last substep the velocity change the contacts produced is applied to the Chaos particle.
//
// Physics thread only.

#pragma once

#include "CoreMinimal.h"

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"

namespace ChronoChaosDynamicMirror
{
	/// True if 'Particle' is a dynamic (or sleeping) rigid body that may be coupled two-way.
	bool IsTwoWayParticle(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FGeometryParticleHandle* Particle);

	/// Mirror of 'Particle' for the current Chaos step: created (and synchronised from the particle) on first use.
	chrono::FChronoDynamicMirror* Touch(chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FGeometryParticleHandle* Particle);

	/// Release every mirror that was not touched in the current Chaos step. Never dereferences their particles.
	void ReleaseStale(chrono::ChCollisionSystemChaos::Impl& Impl);

	/// Refresh FChronoDynamicMirror::ParticlePoseUE from the mirror bodies (call before the narrowphase).
	void UpdatePoses(chrono::ChCollisionSystemChaos::Impl& Impl);

	/// Apply the velocity change of every mirror touched in the current Chaos step to its Chaos particle.
	void ApplyImpulses(chrono::ChCollisionSystemChaos::Impl& Impl);

	/// Drop all mirrors. 'bRemoveFromSystem' is false when the ChSystem is being destroyed.
	void Reset(chrono::ChCollisionSystemChaos::Impl& Impl, bool bRemoveFromSystem);
}
