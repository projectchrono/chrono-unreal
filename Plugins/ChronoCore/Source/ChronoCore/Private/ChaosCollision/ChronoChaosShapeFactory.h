// Converts the collision shapes of a chrono::ChCollisionModel into Chaos implicit geometry.
//
// Every supported ChCollisionShape becomes a Chaos FImplicitObject (in Unreal units / handedness, see
// FChronoConversions) wrapped in a TImplicitObjectTransformed carrying the shape's frame in the model, and all of them
// are unioned in a FImplicitObjectUnion that can be assigned to a Chaos particle. FShapeSlot records which Chrono shape
// produced which union child so that contact / ray hits on a leaf can be mapped back to the ChCollisionShape (and
// therefore to its contact material). Unsupported shape types are skipped and logged once per type - see
// Plugins/ChronoCore/CHAOS_COVERAGE.md for the matrix.

#pragma once

#include "CoreMinimal.h"

#include "Chaos/AABB.h"
#include "Chaos/ImplicitFwd.h"
#include "Chaos/ImplicitObject.h"   // FImplicitObjectPtr (TRefCountPtr) needs the complete type wherever it is destroyed

#include "ChronoIncludeBegin.h"
#include "chrono/collision/ChCollisionModel.h"
#include "ChronoIncludeEnd.h"

/// One union child of a proxy's geometry <-> one Chrono collision shape instance.
struct FChronoShapeSlot
{
	/// Index of the shape instance in ChCollisionModel::GetShapeInstances().
	int32 ChronoShapeIndex = INDEX_NONE;

	/// The Chrono shape (owned by the model).
	chrono::ChCollisionShape* Shape = nullptr;

	/// Union child (TImplicitObjectTransformed wrapper).
	const Chaos::FImplicitObject* Root = nullptr;

	/// Leaf implicit (sphere, box, capsule, convex, triangle mesh) inside the wrapper.
	const Chaos::FImplicitObject* Leaf = nullptr;

	/// Shape frame relative to the model, Unreal units / frame.
	FTransform LocalTransform = FTransform::Identity;

	/// Sphere-swept radius of triangle mesh shapes (ChCollisionShapeTriangleMesh::GetRadius, Unreal cm); 0 otherwise.
	/// Convex-vs-mesh pairs ignore it (Chaos meshes are thin); mesh-vs-mesh pairs inflate the triangles by it.
	double SweptRadiusCm = 0.0;

	/// Exact size (Unreal cm) of a plain Chrono cylinder (axis = local Z of the leaf); 0 for every other shape. The leaf is then
	/// a circumscribed prism used for overlap detection only, the narrowphase generates the contacts on the true circle.
	double CylinderRadiusCm = 0.0;
	double CylinderHalfHeightCm = 0.0;
};

/// Chaos geometry built from a Chrono collision model.
struct FChronoChaosGeometry
{
	/// FImplicitObjectUnion of all supported shapes (null when the model has no supported shape).
	Chaos::FImplicitObjectPtr Root;

	/// One entry per union child, same order as FImplicitObjectUnion::GetObjects() and the particle's ShapesArray().
	TArray<FChronoShapeSlot> Slots;

	/// Model-space bounds of Root (Unreal cm).
	Chaos::FAABB3 LocalBounds = Chaos::FAABB3::EmptyAABB();

	bool IsValid() const { return Root.IsValid() && Slots.Num() > 0; }

	/// Slot whose leaf (or root wrapper) is 'Implicit', or null.
	const FChronoShapeSlot* FindSlot(const Chaos::FImplicitObject* Implicit) const;
};

namespace ChronoChaosShapeFactory
{
	/// Build the Chaos geometry of 'Model'. Returns false when no shape could be converted (Out.Root is null).
	bool BuildGeometry(const chrono::ChCollisionModel& Model, FChronoChaosGeometry& Out);

	/// True if the shape type is converted by BuildGeometry.
	bool IsShapeTypeSupported(chrono::ChCollisionShape::Type Type);

	/// Human readable name of a Chrono shape type (for logs / coverage).
	const TCHAR* ShapeTypeName(chrono::ChCollisionShape::Type Type);

	/// Number of segments used when tessellating cylinders / cones / rounded cylinders into convex hulls.
	constexpr int32 RoundShapeSegments = 24;
}
