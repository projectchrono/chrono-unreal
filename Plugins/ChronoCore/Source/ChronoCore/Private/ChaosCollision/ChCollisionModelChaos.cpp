// chrono::ChCollisionModelChaos - Unreal-side implementation of a Chrono collision model mirrored into Chaos.

#include "ChaosCollision/ChCollisionSystemChaosImpl.h"
#include "ChaosCollision/ChronoChaosSceneProxy.h"

#include "ChronoConversions.h"

#include "ChronoIncludeBegin.h"
#include "chrono/physics/ChContactable.h"
#include "ChronoIncludeEnd.h"

namespace chrono
{
	ChCollisionModelChaos::ChCollisionModelChaos(ChCollisionModel* collision_model, ChCollisionSystemChaos* collision_system)
		: ChCollisionModelImpl(collision_model)
		, m_system(collision_system)
		, m_data(std::make_unique<Data>())
	{
	}

	ChCollisionModelChaos::~ChCollisionModelChaos()
	{
		// 'model' may already be destroyed (see ChCollisionSystemChaos::Clear) - never dereference it here. The proxy
		// particle (if any) was destroyed by the owning system before this point.
	}

	void ChCollisionModelChaos::SyncPosition()
	{
		ChContactable* Contactable = GetContactable();
		if (!Contactable)
		{
			return;
		}

		m_abs_frame = Contactable->GetCollisionModelFrame();
		m_data->PoseUE = FChronoConversions::ToUnrealTransform(m_abs_frame);
		ChronoChaosSceneProxy::UpdateWorldBounds(*m_data);
	}

	void ChCollisionModelChaos::OnFamilyChange(short int family_group, short int family_mask)
	{
		// Chrono collision families only filter Chrono <-> Chrono pairs and are evaluated by the backend itself in
		// Run(); the Chaos proxy filter (Unreal channels) is independent and comes from FChronoChaosBindParams.
	}

	ChAABB ChCollisionModelChaos::GetBoundingBox() const
	{
		return ChronoChaosSceneProxy::ToChronoAABB(m_data->WorldBounds);
	}

	bool ChCollisionModelChaos::HasProxy() const
	{
		return m_data && m_data->Particle != nullptr;
	}
}
