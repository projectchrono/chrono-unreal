// Collision phases of ChCollisionSystemChaos::Run() (physics thread):
//   * Chrono <-> Chrono : exact sort-and-sweep over the proxies' current Chrono poses every substep, then the
//                         narrowphase per overlapping leaf pair (in parallel above the configured threshold);
//   * Chrono <-> Unreal : candidates gathered once per Chaos step from the solver's internal acceleration structure
//                         (see ChronoChaosUnrealPhase), narrowphase every substep.
// Both feed the persistent manifold cache that ReportContacts() hands to Chrono.

#pragma once

#include "CoreMinimal.h"

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"

#include "ChronoIncludeBegin.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "ChronoIncludeEnd.h"

namespace ChronoChaosCollisionPhase
{
	/// World-space leaf transform of a proxy shape slot (model pose composed with the slot's local frame).
	FTransform LeafWorldTransform(const chrono::ChCollisionModelChaos::Data& Data, const FChronoShapeSlot& Slot);

	/// World-space bounds of a proxy shape slot at the model's current pose.
	Chaos::FAABB3 LeafWorldBounds(const chrono::ChCollisionModelChaos::Data& Data, const FChronoShapeSlot& Slot);

	/// Chrono collision envelope of a model in Unreal cm.
	double EnvelopeCm(const chrono::ChCollisionModelChaos& Model);

	/// Chrono <-> Chrono pairs: broadphase, narrowphase and manifold update. Updates Impl.TimerBroad / TimerNarrow.
	void RunChronoPairs(chrono::ChCollisionSystemChaos::Impl& Impl, chrono::ChCollisionSystem::BroadphaseCallback* BroadCallback);

	/// Hand every manifold point to the contact container as a ChCollisionInfo (Chrono units / frame).
	void ReportManifolds(chrono::ChCollisionSystemChaos::Impl& Impl,
	                     chrono::ChContactContainer& Container,
	                     chrono::ChCollisionSystem::NarrowphaseCallback* NarrowCallback);
}
