// =============================================================================
// PROJECT CHRONO - Unreal Engine Chaos collision backend
//
// This file is NOT part of upstream Project Chrono. It is the Chrono-facing half of the Chaos collision backend
// shipped with the ChronoCore Unreal plugin (see Plugins/ChronoCore/PATCHES.md and CHAOS_COVERAGE.md).
//
// This header is pure Chrono C++: it must stay includable from any Chrono translation unit without Unreal headers.
// Everything that touches Unreal / Chaos lives behind the opaque 'Data' struct implemented in
// Plugins/ChronoCore/Source/ChronoCore/Private/ChaosCollision/.
// =============================================================================

#ifndef CH_COLLISION_MODEL_CHAOS_H
#define CH_COLLISION_MODEL_CHAOS_H

#include <memory>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/geometry/ChGeometry.h"

namespace chrono {

class ChCollisionSystemChaos;

/// @addtogroup collision_chaos
/// @{

/// Concrete collision model for the Unreal Chaos collision backend.
/// Mirrors a ChCollisionModel as a kinematic particle (with Chaos implicit geometry) in the Chaos scene of the
/// solver bound to the owning ChCollisionSystemChaos.
class ChApi ChCollisionModelChaos : public ChCollisionModelImpl {
  public:
    ChCollisionModelChaos(ChCollisionModel* collision_model, ChCollisionSystemChaos* collision_system);
    virtual ~ChCollisionModelChaos();

    /// Return the associated Chrono collision model.
    ChCollisionModel* GetModel() const { return model; }

    /// Return the owning collision system.
    ChCollisionSystemChaos* GetSystem() const { return m_system; }

    /// Synchronize the position and orientation of the collision model to the associated contactable.
    virtual void SyncPosition() override;

    /// Additional operations to be performed on a change in collision family.
    virtual void OnFamilyChange(short int family_group, short int family_mask) override;

    /// Return the current axis aligned bounding box (AABB) of the collision model.
    /// Note that SyncPosition() should be invoked before calling this.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the absolute frame of this model as recorded by the last SyncPosition() call.
    const ChFrame<>& GetAbsFrame() const { return m_abs_frame; }

    /// Return true if this model has a Chaos geometry / scene proxy.
    bool HasProxy() const;

    /// Opaque Unreal-side data (Chaos geometry, scene proxy). Defined in the ChronoCore private sources.
    struct Data;

    /// Direct access to the Unreal-side data (only for the implementation of the Chaos collision system).
    Data* GetData() const { return m_data.get(); }

  private:
    ChCollisionSystemChaos* m_system;  ///< owning collision system
    ChFrame<> m_abs_frame;             ///< absolute frame of the model (updated by SyncPosition)
    std::unique_ptr<Data> m_data;      ///< Unreal-side data

    friend class ChCollisionSystemChaos;
};

/// @} collision_chaos

}  // end namespace chrono

#endif
