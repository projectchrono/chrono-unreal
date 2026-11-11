// =============================================================================
// PROJECT CHRONO - Unreal Engine Chaos collision backend
//
// This file is NOT part of upstream Project Chrono. It is the Chrono-facing half of the Chaos collision backend
// shipped with the ChronoCore Unreal plugin (see Plugins/ChronoCore/PATCHES.md and CHAOS_COVERAGE.md).
//
// This header is pure Chrono C++: it must stay includable from any Chrono translation unit (e.g. ChSystem.cpp)
// without Unreal headers. Everything that touches Unreal / Chaos lives behind the opaque 'Impl' struct implemented
// in Plugins/ChronoCore/Source/ChronoCore/Private/ChaosCollision/.
//
// Usage from Unreal code: the collision system is inert (no contacts, no ray hits) until it is bound to a Chaos
// rigid-body solver with FChronoChaosCollision::BindSolver() (Plugins/ChronoCore/Source/ChronoCore/Public/
// ChronoChaosCollision.h). Binding and everything that follows (Run, ReportContacts, RayHit, ...) must happen on
// the Chaos physics thread; FChronoSimulation takes care of this.
// =============================================================================

#ifndef CH_COLLISION_SYSTEM_CHAOS_H
#define CH_COLLISION_SYSTEM_CHAOS_H

#include <memory>

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/chaos/ChCollisionModelChaos.h"

namespace chrono {

/// @addtogroup collision_chaos
/// @{

/// Collision engine that uses the Unreal Engine Chaos physics scene as an external collision provider.
/// Chrono keeps its own solvers, integrators, contact containers and materials; broadphase, narrowphase and ray /
/// shape queries are delegated to the Chaos scene the system is bound to. Every Chrono collision model becomes a
/// kinematic proxy particle in that scene, so Chrono bodies collide with each other as well as with Unreal static
/// geometry, landscapes and rigid bodies.
class ChApi ChCollisionSystemChaos : public ChCollisionSystem {
  public:
    ChCollisionSystemChaos();
    virtual ~ChCollisionSystemChaos();

    /// Clears all data instanced by this algorithm (persistent contact manifolds, proxy particles).
    virtual void Clear() override;

    /// Add the specified collision model to the collision engine.
    /// If no Chaos solver is bound yet, the model is kept pending and bound retroactively.
    virtual void Add(std::shared_ptr<ChCollisionModel> model) override;

    /// Remove the specified collision model from the collision engine.
    virtual void Remove(std::shared_ptr<ChCollisionModel> model) override;

    /// Synchronization operations invoked before running the collision detection.
    virtual void PreProcess() override;

    /// Run the collision detection and find the contacts.
    virtual void Run() override;

    /// Return an AABB bounding all collision shapes in the system.
    virtual ChAABB GetBoundingBox() const override;

    /// Reset timers for collision detection.
    virtual void ResetTimers() override;

    /// Return the time (in seconds) for broadphase collision detection.
    virtual double GetTimerCollisionBroad() const override;

    /// Return the time (in seconds) for narrowphase collision detection.
    virtual double GetTimerCollisionNarrow() const override;

    /// Report contacts (fill the provided 'contact container').
    virtual void ReportContacts(ChContactContainer* contact_container) override;

    /// Report proximities. Not supported by the Chaos backend (no-op).
    virtual void ReportProximities(ChProximityContainer* proximity_container) override;

    /// Perform a ray-hit test with all collision models (Chrono proxies and Unreal geometry).
    /// Hits on non-Chrono geometry report the backend's world proxy model as 'hitModel'.
    virtual bool RayHit(const ChVector3d& from, const ChVector3d& to, ChRayhitResult& result) const override;

    /// Perform a ray-hit test with the specified collision model.
    virtual bool RayHit(const ChVector3d& from,
                        const ChVector3d& to,
                        ChCollisionModel* model,
                        ChRayhitResult& result) const override;

    /// Discrete contact test of a stand-alone collision model against everything in the collision world.
    virtual bool ShapeContactTest(ChCollisionModel& query_model,
                                  const ChFrame<>& frame,
                                  std::vector<ChContactQueryHit>& hits,
                                  short int filter_group = 1,
                                  short int filter_mask = -1) const override;

    /// Method to trigger debug visualization of collision shapes (through the registered visualization callback).
    virtual void Visualize(int flags) override;

    /// Serialization of Chaos state is not supported (no-op).
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// De-serialization of Chaos state is not supported (no-op).
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// Return true if this collision system is bound to a Chaos solver (see FChronoChaosCollision::BindSolver).
    bool IsSceneBound() const;

    /// Return the world proxy collision model used to report contacts against non-Chrono (Unreal) geometry.
    /// Null while no solver is bound.
    ChCollisionModel* GetWorldProxyModel() const;

    /// Return the number of collision models currently managed by this system (bound or pending).
    unsigned int GetNumModels() const;

    /// Return the associated Chrono system (null until the system is attached to a ChSystem).
    ChSystem* GetSystem() const { return m_system; }

    /// Opaque Unreal-side implementation. Defined in the ChronoCore private sources.
    struct Impl;

    /// Direct access to the Unreal-side implementation (only for the ChronoCore plugin code).
    Impl& GetImpl() { return *m_impl; }
    const Impl& GetImpl() const { return *m_impl; }

  private:
    std::unique_ptr<Impl> m_impl;

    friend class ChCollisionModelChaos;
};

/// @} collision_chaos

}  // end namespace chrono

#endif
