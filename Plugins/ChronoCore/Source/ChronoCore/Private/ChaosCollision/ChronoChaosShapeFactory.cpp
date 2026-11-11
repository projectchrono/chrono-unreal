// ChronoChaosShapeFactory - chrono::ChCollisionShape -> Chaos::FImplicitObject.

#include "ChaosCollision/ChronoChaosShapeFactory.h"

#include "ChronoConversions.h"
#include "ChronoCoreModule.h"

#include "Chaos/Box.h"
#include "Chaos/Capsule.h"
#include "Chaos/Convex.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ImplicitObjectTransformed.h"
#include "Chaos/ImplicitObjectUnion.h"
#include "Chaos/Particles.h"
#include "Chaos/Sphere.h"
#include "Chaos/TriangleMeshImplicitObject.h"

#include "ChronoIncludeBegin.h"
#include "chrono/collision/ChCollisionShapes.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "ChronoIncludeEnd.h"

// Chrono cylinders are collided analytically: the prism below only detects the overlap and gives the separating axis, the
// contact points are generated on the exact circle (see CylinderFaceManifold in ChronoChaosNarrowphase.cpp). A prism alone
// makes a rolling wheel hit a facet edge every few degrees and lose a large part of its speed.
static TAutoConsoleVariable<int32> CVarAnalyticCylinder(TEXT("ChronoChaos.AnalyticCylinder"), 1,
	TEXT("1: contacts of Chrono cylinders are generated on the exact circle (default). 0: contacts come from the faceted prism."));

namespace
{
	using namespace Chaos;

	/// Warn once per unsupported shape type (per process).
	void WarnUnsupportedOnce(chrono::ChCollisionShape::Type Type)
	{
		static bool bWarned[chrono::ChCollisionShape::UNKNOWN_SHAPE + 1] = {};
		const int32 Index = FMath::Clamp(static_cast<int32>(Type), 0, static_cast<int32>(chrono::ChCollisionShape::UNKNOWN_SHAPE));
		if (!bWarned[Index])
		{
			bWarned[Index] = true;
			UE_LOG(LogChronoCore, Warning,
			       TEXT("ChCollisionSystemChaos: collision shape type %s is not supported by the Chaos backend and is ignored (see Plugins/ChronoCore/CHAOS_COVERAGE.md)."),
			       ChronoChaosShapeFactory::ShapeTypeName(Type));
		}
	}

	FORCEINLINE FReal Cm(double Metres)
	{
		return static_cast<FReal>(FChronoConversions::ToUnrealSize(Metres));
	}

	FImplicitObjectPtr MakeSphere(double RadiusM)
	{
		return FImplicitObjectPtr(new FImplicitSphere3(FVec3(0), Cm(RadiusM)));
	}

	FImplicitObjectPtr MakeBox(const chrono::ChVector3d& HalfLengthsM, double Margin = 0.0)
	{
		const FVec3 Half(Cm(HalfLengthsM.x()), Cm(HalfLengthsM.y()), Cm(HalfLengthsM.z()));
		return FImplicitObjectPtr(new TBox<FReal, 3>(-Half, Half, Margin));
	}

	FImplicitObjectPtr MakeCapsule(double RadiusM, double HeightM)
	{
		// Chrono capsules are aligned with the local Z axis; 'height' is the length of the cylindrical part.
		const FReal HalfHeight = Cm(0.5 * HeightM);
		return FImplicitObjectPtr(new FCapsule(FVec3(0, 0, -HalfHeight), FVec3(0, 0, HalfHeight), Cm(RadiusM)));
	}

	FImplicitObjectPtr MakeConvex(TArray<FConvex::FVec3Type>&& Points, FReal Margin)
	{
		if (Points.Num() < 4)
		{
			return nullptr;
		}
		FConvex* Convex = new FConvex(Points, Margin);
		if (Convex->NumVertices() < 4)
		{
			delete Convex;
			return nullptr;
		}
		return FImplicitObjectPtr(Convex);
	}

	/// Points on the two rims of a Z-aligned cylinder of the given radius / height (Unreal cm).
	void AppendCylinderRims(TArray<FConvex::FVec3Type>& Points, FReal RadiusCm, FReal HalfHeightCm, int32 Segments)
	{
		for (int32 i = 0; i < Segments; ++i)
		{
			const double Angle = 2.0 * PI * static_cast<double>(i) / Segments;
			const FRealSingle X = static_cast<FRealSingle>(RadiusCm * FMath::Cos(Angle));
			const FRealSingle Y = static_cast<FRealSingle>(RadiusCm * FMath::Sin(Angle));
			Points.Add(FConvex::FVec3Type(X, Y, static_cast<FRealSingle>(-HalfHeightCm)));
			Points.Add(FConvex::FVec3Type(X, Y, static_cast<FRealSingle>(HalfHeightCm)));
		}
	}

	FImplicitObjectPtr MakeCylinder(double RadiusM, double HeightM, double RoundingRadiusM = 0.0, bool bCircumscribed = false)
	{
		// Rounded cylinders: hull of the rounded outline with a Chaos margin equal to the rounding radius (the
		// margin shrinks the core and adds it back as a sphere radius, which is exactly a rounded shape).
		// A circumscribed prism (vertices outside the circle) never misses an overlap of the true cylinder.
		const FReal Rounding = Cm(RoundingRadiusM);
		const int32 Segments = ChronoChaosShapeFactory::RoundShapeSegments;
		FReal Radius = Cm(RadiusM) + Rounding;
		if (bCircumscribed)
		{
			Radius /= FMath::Cos(PI / Segments);
		}
		TArray<FConvex::FVec3Type> Points;
		Points.Reserve(2 * Segments);
		AppendCylinderRims(Points, Radius, Cm(0.5 * HeightM) + Rounding, Segments);
		return MakeConvex(MoveTemp(Points), Rounding);
	}

	FImplicitObjectPtr MakeCone(double RadiusM, double HeightM)
	{
		// Chrono cones are aligned with Z, origin at the axis centre, apex towards +Z.
		const FReal HalfHeight = Cm(0.5 * HeightM);
		TArray<FConvex::FVec3Type> Points;
		Points.Reserve(ChronoChaosShapeFactory::RoundShapeSegments + 1);
		for (int32 i = 0; i < ChronoChaosShapeFactory::RoundShapeSegments; ++i)
		{
			const double Angle = 2.0 * PI * static_cast<double>(i) / ChronoChaosShapeFactory::RoundShapeSegments;
			Points.Add(FConvex::FVec3Type(static_cast<FRealSingle>(Cm(RadiusM) * FMath::Cos(Angle)),
			                              static_cast<FRealSingle>(Cm(RadiusM) * FMath::Sin(Angle)),
			                              static_cast<FRealSingle>(-HalfHeight)));
		}
		Points.Add(FConvex::FVec3Type(0, 0, static_cast<FRealSingle>(HalfHeight)));
		return MakeConvex(MoveTemp(Points), 0);
	}

	FImplicitObjectPtr MakeEllipsoid(const chrono::ChVector3d& SemiaxesM)
	{
		constexpr int32 Rings = 12;
		constexpr int32 Sectors = 24;
		const FVec3 Semi(Cm(SemiaxesM.x()), Cm(SemiaxesM.y()), Cm(SemiaxesM.z()));

		TArray<FConvex::FVec3Type> Points;
		Points.Reserve((Rings - 1) * Sectors + 2);
		Points.Add(FConvex::FVec3Type(0, 0, static_cast<FRealSingle>(Semi.Z)));
		Points.Add(FConvex::FVec3Type(0, 0, static_cast<FRealSingle>(-Semi.Z)));
		for (int32 r = 1; r < Rings; ++r)
		{
			const double Theta = PI * static_cast<double>(r) / Rings;
			const double SinT = FMath::Sin(Theta), CosT = FMath::Cos(Theta);
			for (int32 s = 0; s < Sectors; ++s)
			{
				const double Phi = 2.0 * PI * static_cast<double>(s) / Sectors;
				Points.Add(FConvex::FVec3Type(static_cast<FRealSingle>(Semi.X * SinT * FMath::Cos(Phi)),
				                              static_cast<FRealSingle>(Semi.Y * SinT * FMath::Sin(Phi)),
				                              static_cast<FRealSingle>(Semi.Z * CosT)));
			}
		}
		return MakeConvex(MoveTemp(Points), 0);
	}

	FImplicitObjectPtr MakeRoundedBox(const chrono::ChVector3d& HalfLengthsM, double RoundingRadiusM)
	{
		// Box corners pushed out by the rounding radius + a Chaos margin of the same radius = rounded box.
		const FReal Rounding = Cm(RoundingRadiusM);
		const FVec3 Half(Cm(HalfLengthsM.x()) + Rounding, Cm(HalfLengthsM.y()) + Rounding, Cm(HalfLengthsM.z()) + Rounding);
		TArray<FConvex::FVec3Type> Points;
		Points.Reserve(8);
		for (int32 i = 0; i < 8; ++i)
		{
			Points.Add(FConvex::FVec3Type(static_cast<FRealSingle>((i & 1) ? Half.X : -Half.X),
			                              static_cast<FRealSingle>((i & 2) ? Half.Y : -Half.Y),
			                              static_cast<FRealSingle>((i & 4) ? Half.Z : -Half.Z)));
		}
		return MakeConvex(MoveTemp(Points), Rounding);
	}

	FImplicitObjectPtr MakeConvexHull(const std::vector<chrono::ChVector3d>& PointsM)
	{
		TArray<FConvex::FVec3Type> Points;
		Points.Reserve(static_cast<int32>(PointsM.size()));
		for (const chrono::ChVector3d& P : PointsM)
		{
			// Positions inside the shape frame: scale and mirror like any position.
			const FVector V = FChronoConversions::ToUnrealPos(P);
			Points.Add(FConvex::FVec3Type(static_cast<FRealSingle>(V.X), static_cast<FRealSingle>(V.Y), static_cast<FRealSingle>(V.Z)));
		}
		return MakeConvex(MoveTemp(Points), 0);
	}

	FImplicitObjectPtr MakeTriangleMesh(chrono::ChTriangleMesh& Mesh)
	{
		FTriangleMeshImplicitObject::ParticlesType Particles;
		TArray<TVec3<int32>> Elements;
		TArray<uint16> MaterialIndices;

		auto AddVertex = [&Particles](const chrono::ChVector3d& P)
		{
			const int32 Index = static_cast<int32>(Particles.Size());
			Particles.AddParticles(1);
			const FVector V = FChronoConversions::ToUnrealPos(P);
			Particles.SetX(Index, TVec3<FRealSingle>(static_cast<FRealSingle>(V.X), static_cast<FRealSingle>(V.Y), static_cast<FRealSingle>(V.Z)));
			return Index;
		};

		if (auto* Connected = dynamic_cast<chrono::ChTriangleMeshConnected*>(&Mesh))
		{
			const std::vector<chrono::ChVector3d>& Vertices = Connected->GetCoordsVertices();
			const std::vector<chrono::ChVector3i>& Indices = Connected->GetIndicesVertexes();
			for (const chrono::ChVector3d& V : Vertices)
			{
				AddVertex(V);
			}
			Elements.Reserve(static_cast<int32>(Indices.size()));
			for (const chrono::ChVector3i& Tri : Indices)
			{
				// Mirroring Y flips the handedness: swap two indices to keep the outward winding.
				Elements.Add(TVec3<int32>(Tri.x(), Tri.z(), Tri.y()));
			}
		}
		else
		{
			const unsigned int NumTriangles = Mesh.GetNumTriangles();
			Elements.Reserve(static_cast<int32>(NumTriangles));
			for (unsigned int i = 0; i < NumTriangles; ++i)
			{
				const chrono::ChTriangle Tri = Mesh.GetTriangle(i);
				const int32 I0 = AddVertex(Tri.p1);
				const int32 I1 = AddVertex(Tri.p2);
				const int32 I2 = AddVertex(Tri.p3);
				Elements.Add(TVec3<int32>(I0, I2, I1));
			}
		}

		if (Particles.Size() < 3 || Elements.Num() == 0)
		{
			return nullptr;
		}
		MaterialIndices.Init(0, Elements.Num());
		return FImplicitObjectPtr(new FTriangleMeshImplicitObject(MoveTemp(Particles), MoveTemp(Elements), MoveTemp(MaterialIndices)));
	}

	/// Convert one shape instance. Returns the leaf implicit (null when unsupported / degenerate) and the local
	/// offset to apply in addition to the instance frame (used by POINT).
	FImplicitObjectPtr MakeLeaf(chrono::ChCollisionShape& Shape, FVector& OutExtraOffset)
	{
		OutExtraOffset = FVector::ZeroVector;
		using chrono::ChCollisionShape;

		switch (Shape.GetType())
		{
			case ChCollisionShape::SPHERE:
				return MakeSphere(static_cast<chrono::ChCollisionShapeSphere&>(Shape).GetRadius());

			case ChCollisionShape::BOX:
				return MakeBox(static_cast<chrono::ChCollisionShapeBox&>(Shape).GetHalflengths());

			case ChCollisionShape::CAPSULE:
			{
				auto& Capsule = static_cast<chrono::ChCollisionShapeCapsule&>(Shape);
				return MakeCapsule(Capsule.GetRadius(), Capsule.GetHeight());
			}

			case ChCollisionShape::CYLINDER:
			{
				auto& Cylinder = static_cast<chrono::ChCollisionShapeCylinder&>(Shape);
				return MakeCylinder(Cylinder.GetRadius(), Cylinder.GetHeight(), 0.0, CVarAnalyticCylinder.GetValueOnAnyThread() != 0);
			}

			case ChCollisionShape::CONE:
			{
				auto& Cone = static_cast<chrono::ChCollisionShapeCone&>(Shape);
				return MakeCone(Cone.GetRadius(), Cone.GetHeight());
			}

			case ChCollisionShape::ELLIPSOID:
				return MakeEllipsoid(static_cast<chrono::ChCollisionShapeEllipsoid&>(Shape).GetSemiaxes());

			case ChCollisionShape::ROUNDEDBOX:
			{
				auto& Box = static_cast<chrono::ChCollisionShapeRoundedBox&>(Shape);
				return MakeRoundedBox(Box.GetHalflengths(), Box.GetSRadius());
			}

			case ChCollisionShape::ROUNDEDCYL:
			{
				auto& Cylinder = static_cast<chrono::ChCollisionShapeRoundedCylinder&>(Shape);
				return MakeCylinder(Cylinder.GetRadius(), Cylinder.GetHeight(), Cylinder.GetSRadius());
			}

			case ChCollisionShape::CONVEXHULL:
				return MakeConvexHull(static_cast<chrono::ChCollisionShapeConvexHull&>(Shape).GetPoints());

			case ChCollisionShape::TRIANGLEMESH:
			{
				auto& TriMesh = static_cast<chrono::ChCollisionShapeTriangleMesh&>(Shape);
				if (std::shared_ptr<chrono::ChTriangleMesh> Mesh = TriMesh.GetMesh())
				{
					return MakeTriangleMesh(*Mesh);
				}
				return nullptr;
			}

			case ChCollisionShape::POINT:
			{
				auto& Point = static_cast<chrono::ChCollisionShapePoint&>(Shape);
				OutExtraOffset = FChronoConversions::ToUnrealPos(Point.GetPoint());
				return MakeSphere(FMath::Max(Point.GetRadius(), 1e-4));
			}

			default:
				WarnUnsupportedOnce(Shape.GetType());
				return nullptr;
		}
	}
}

const FChronoShapeSlot* FChronoChaosGeometry::FindSlot(const Chaos::FImplicitObject* Implicit) const
{
	for (const FChronoShapeSlot& Slot : Slots)
	{
		if (Slot.Leaf == Implicit || Slot.Root == Implicit)
		{
			return &Slot;
		}
	}
	return nullptr;
}

namespace ChronoChaosShapeFactory
{
	bool IsShapeTypeSupported(chrono::ChCollisionShape::Type Type)
	{
		using chrono::ChCollisionShape;
		switch (Type)
		{
			case ChCollisionShape::SPHERE:
			case ChCollisionShape::BOX:
			case ChCollisionShape::CAPSULE:
			case ChCollisionShape::CYLINDER:
			case ChCollisionShape::CONE:
			case ChCollisionShape::ELLIPSOID:
			case ChCollisionShape::ROUNDEDBOX:
			case ChCollisionShape::ROUNDEDCYL:
			case ChCollisionShape::CONVEXHULL:
			case ChCollisionShape::TRIANGLEMESH:
			case ChCollisionShape::POINT:
				return true;
			default:
				return false;
		}
	}

	const TCHAR* ShapeTypeName(chrono::ChCollisionShape::Type Type)
	{
		using chrono::ChCollisionShape;
		switch (Type)
		{
			case ChCollisionShape::SPHERE: return TEXT("SPHERE");
			case ChCollisionShape::ELLIPSOID: return TEXT("ELLIPSOID");
			case ChCollisionShape::BOX: return TEXT("BOX");
			case ChCollisionShape::CYLINDER: return TEXT("CYLINDER");
			case ChCollisionShape::CYLSHELL: return TEXT("CYLSHELL");
			case ChCollisionShape::CONVEXHULL: return TEXT("CONVEXHULL");
			case ChCollisionShape::TRIANGLEMESH: return TEXT("TRIANGLEMESH");
			case ChCollisionShape::BARREL: return TEXT("BARREL");
			case ChCollisionShape::POINT: return TEXT("POINT");
			case ChCollisionShape::SEGMENT: return TEXT("SEGMENT");
			case ChCollisionShape::TRIANGLE: return TEXT("TRIANGLE");
			case ChCollisionShape::MESHTRIANGLE: return TEXT("MESHTRIANGLE");
			case ChCollisionShape::CAPSULE: return TEXT("CAPSULE");
			case ChCollisionShape::CONE: return TEXT("CONE");
			case ChCollisionShape::ROUNDEDBOX: return TEXT("ROUNDEDBOX");
			case ChCollisionShape::ROUNDEDCYL: return TEXT("ROUNDEDCYL");
			case ChCollisionShape::TETRAHEDRON: return TEXT("TETRAHEDRON");
			case ChCollisionShape::PATH2D: return TEXT("PATH2D");
			case ChCollisionShape::SEGMENT2D: return TEXT("SEGMENT2D");
			case ChCollisionShape::ARC2D: return TEXT("ARC2D");
			default: return TEXT("UNKNOWN_SHAPE");
		}
	}

	bool BuildGeometry(const chrono::ChCollisionModel& Model, FChronoChaosGeometry& Out)
	{
		Out = FChronoChaosGeometry();

		const std::vector<chrono::ChCollisionShapeInstance>& Instances = Model.GetShapeInstances();
		TArray<FImplicitObjectPtr> Children;
		Children.Reserve(static_cast<int32>(Instances.size()));
		Out.Slots.Reserve(static_cast<int32>(Instances.size()));

		for (int32 Index = 0; Index < static_cast<int32>(Instances.size()); ++Index)
		{
			const chrono::ChCollisionShapeInstance& Instance = Instances[Index];
			if (!Instance.shape)
			{
				continue;
			}

			FVector ExtraOffset;
			FImplicitObjectPtr Leaf = MakeLeaf(*Instance.shape, ExtraOffset);
			if (!Leaf.IsValid())
			{
				continue;
			}

			FTransform LocalTransform = FChronoConversions::ToUnrealTransform(Instance.frame);
			LocalTransform.SetLocation(LocalTransform.TransformPosition(ExtraOffset));

			const FImplicitObject* LeafPtr = Leaf.GetReference();
			FImplicitObjectPtr Wrapped(new TImplicitObjectTransformed<FReal, 3>(MoveTemp(Leaf), FRigidTransform3(LocalTransform)));

			FChronoShapeSlot& Slot = Out.Slots.AddDefaulted_GetRef();
			Slot.ChronoShapeIndex = Index;
			Slot.Shape = Instance.shape.get();
			Slot.Root = Wrapped.GetReference();
			Slot.Leaf = LeafPtr;
			Slot.LocalTransform = LocalTransform;
			if (Instance.shape->GetType() == chrono::ChCollisionShape::CYLINDER && CVarAnalyticCylinder.GetValueOnAnyThread() != 0)
			{
				const auto& Cylinder = static_cast<const chrono::ChCollisionShapeCylinder&>(*Instance.shape);
				Slot.CylinderRadiusCm = Cm(Cylinder.GetRadius());
				Slot.CylinderHalfHeightCm = Cm(0.5 * Cylinder.GetHeight());
			}
			if (Instance.shape->GetType() == chrono::ChCollisionShape::TRIANGLEMESH)
			{
				// Sphere-swept triangles (Bullet's cbtCEtriangleShape margin): used by the mesh-vs-mesh narrowphase.
				Slot.SweptRadiusCm = Cm(static_cast<chrono::ChCollisionShapeTriangleMesh&>(*Instance.shape).GetRadius());
			}

			Out.LocalBounds.GrowToInclude(Wrapped->BoundingBox());
			Children.Add(MoveTemp(Wrapped));
		}

		if (Children.Num() == 0)
		{
			Out.Slots.Reset();
			Out.LocalBounds = FAABB3::EmptyAABB();
			return false;
		}

		Out.Root = FImplicitObjectPtr(new FImplicitObjectUnion(MoveTemp(Children)));
		return true;
	}
}
