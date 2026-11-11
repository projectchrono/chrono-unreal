// Chrono <-> Unreal geometry phase of ChCollisionSystemChaos::Run() (physics thread).
//
// Phase A (once per Chaos step, triggered by FChronoChaosCollision::BeginChaosStep): every proxy overlaps its
// velocity-inflated bounds against the solver's internal spatial acceleration structure and caches the non-Chrono
// shapes it finds (O(log N) in the number of level colliders). Phase B (every Chrono substep): the narrowphase runs
// between the proxy's leaves (current Chrono pose) and the cached shapes (current particle pose) and updates the
// manifold cache. Statics cannot move between Chaos steps; Unreal dynamics are at most one Chaos step stale.

#pragma once

#include "CoreMinimal.h"

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"

namespace ChronoChaosUnrealPhase
{
	/// Phase A: refresh every proxy's UnrealCandidates from the internal acceleration structure.
	void GatherCandidates(chrono::ChCollisionSystemChaos::Impl& Impl);

	/// Phase B: narrowphase of every proxy against its cached candidates; updates Impl.Manifolds and the timers.
	void RunUnrealPairs(chrono::ChCollisionSystemChaos::Impl& Impl);

	/// Shape of a candidate (null if the index is stale).
	const Chaos::FPerShapeData* GetCandidateShape(const chrono::FChronoUnrealCandidate& Candidate);

	/// Collect the collidable non-Chrono shapes whose bounds overlap 'QueryBounds' (world, cm) from the solver's
	/// internal acceleration structure.
	void GatherCandidatesInBounds(const chrono::ChCollisionSystemChaos::Impl& Impl, const Chaos::FAABB3& QueryBounds, TArray<chrono::FChronoUnrealCandidate>& Out);
}
