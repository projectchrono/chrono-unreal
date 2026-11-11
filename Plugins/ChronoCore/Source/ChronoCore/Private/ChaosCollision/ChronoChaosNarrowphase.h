// Narrowphase of the Chaos collision backend: contact generation between two Chaos implicit leaves using Chaos'
// public geometric primitives (GJK/EPA for quadric pairs, one-shot triangle manifolds for polyhedra and meshes).
//
// Everything is in Unreal units / frame. Contact normals point from A to B (Chrono's ChCollisionInfo::vN convention:
// "normal with respect to A"); separation is positive when the shapes are apart and negative when they penetrate.

#pragma once

#include "CoreMinimal.h"

#include "Chaos/ImplicitFwd.h"

namespace Chaos
{
	class FImplicitObject;
}

namespace ChronoChaosNarrowphase
{
	/// One contact between leaf A and leaf B (world space, cm).
	struct FContact
	{
		FVector PointA = FVector::ZeroVector;   ///< on A's surface
		FVector PointB = FVector::ZeroVector;   ///< on B's surface
		FVector Normal = FVector::UpVector;     ///< unit, from A to B
		double Separation = 0.0;                ///< < 0 penetration
		int32 FaceIndexB = INDEX_NONE;          ///< triangle index on B for mesh / heightfield leaves
	};

	using FContactArray = TArray<FContact, TInlineAllocator<8>>;

	/// Leaf categories handled by the narrowphase (wrappers such as scaled/instanced are looked through).
	enum class ELeafKind : uint8
	{
		Unsupported,
		Sphere,
		Capsule,
		Box,
		Convex,
		TriangleMesh,
		HeightField,
	};

	ELeafKind ClassifyLeaf(const Chaos::FImplicitObject& Leaf);

	/// True if the narrowphase can produce contacts for this pair of leaf kinds.
	bool IsPairSupported(ELeafKind A, ELeafKind B);

	/// Per-leaf parameters that are not part of the Chaos implicit itself.
	struct FLeafParams
	{
		/// Sphere-swept radius (cm) of a triangle mesh leaf (Chrono's trimesh "sweep sphere" / Bullet margin). Only
		/// used by mesh-vs-mesh and mesh-vs-heightfield pairs, where both surfaces are thin; convex-vs-mesh pairs
		/// treat meshes as infinitely thin like Chaos itself.
		double SweptRadiusCm = 0.0;

		/// Exact cylinder (axis = local Z of the leaf transform): the leaf is a circumscribed prism that only detects the overlap,
		/// the manifold is built on the true circle so that wheels roll without facet impacts. 0 = not a cylinder.
		double CylinderRadiusCm = 0.0;
		double CylinderHalfHeightCm = 0.0;
		bool IsCylinder() const { return CylinderRadiusCm > 0.0; }
	};

	/// Generate contacts between leaf A placed at ATM and leaf B placed at BTM (leaf world transforms, no scale;
	/// scaled implicits carry their own scale). Contacts farther apart than CullDistance (cm) are dropped.
	/// Returns false when the pair kind is unsupported (nothing appended).
	bool GenerateContacts(const Chaos::FImplicitObject& LeafA, const FTransform& ATM,
	                      const Chaos::FImplicitObject& LeafB, const FTransform& BTM,
	                      double CullDistance, FContactArray& Out,
	                      const FLeafParams& ParamsA = FLeafParams(), const FLeafParams& ParamsB = FLeafParams());

	/// Keep at most MaxPoints contacts: the deepest, the farthest from it, then the ones that maximise the covered
	/// area (Bullet / Chaos style manifold reduction). Order of the survivors is unspecified.
	void ReduceContacts(FContactArray& Contacts, int32 MaxPoints = 4);
}
