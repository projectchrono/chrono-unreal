// ChronoChaosNarrowphase - contact generation between Chaos implicit leaves for the Chrono collision backend.
//
// Pair strategy (only exported / header-only Chaos entry points are used - the templated
// ConstructConvexTriangleOneShotManifold2 instantiations and TBox's static face tables are not exported from the
// Chaos DLL, so polyhedron manifolds are built here):
//   * sphere vs anything convex / triangle          : GJKPenetration (header-only), one exact contact point;
//   * capsule vs polyhedron / triangle              : ConstructCapsuleTriangleOneShotManifold2 (exported) against the
//                                                     polyhedron's face triangles (2-point manifolds);
//   * capsule vs capsule                            : GJKPenetration;
//   * polyhedron (box / convex) vs polyhedron / tri : GJKPenetration for the separating axis + Bullet-style reference /
//                                                     incident face clipping (FPolyView) -> up to 4 planar points;
//   * anything convex vs triangle mesh / heightfield: FTriangleMeshImplicitObject::VisitTriangles /
//                                                     FHeightField::VisitTriangles + the per-triangle rules above;
//   * triangle mesh vs triangle mesh / heightfield  : AABB-culled triangle pairs, GJK/EPA with the Chrono sphere-swept
//                                                     radius of the mesh triangles as GJK thickness (Bullet's
//                                                     cbtCEtriangleShape margin), one contact per touching pair;
//   * heightfield vs heightfield                    : unsupported (recorded in CHAOS_COVERAGE.md).

#include "ChaosCollision/ChronoChaosNarrowphase.h"

#include "ChronoCoreModule.h"

#include "Chaos/Box.h"
#include "Chaos/Capsule.h"
#include "Chaos/Collision/CapsuleTriangleContactPoint.h"
#include "Chaos/Collision/ContactPoint.h"
#include "Chaos/Collision/SphereTriangleContactPoint.h"
#include "Chaos/CollisionResolutionTypes.h"
#include "Chaos/Convex.h"
#include "Chaos/GJK.h"
#include "Chaos/HeightField.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ImplicitObjectScaled.h"
#include "Chaos/ImplicitObjectType.h"
#include "Chaos/Sphere.h"
#include "Chaos/Triangle.h"
#include "Chaos/TriangleMeshImplicitObject.h"

namespace ChronoChaosNarrowphase
{
	using namespace Chaos;

	namespace
	{
		FORCEINLINE bool IsPolyhedral(ELeafKind Kind) { return Kind == ELeafKind::Box || Kind == ELeafKind::Convex; }
		FORCEINLINE bool IsQuadric(ELeafKind Kind) { return Kind == ELeafKind::Sphere || Kind == ELeafKind::Capsule; }
		FORCEINLINE bool IsConvexKind(ELeafKind Kind) { return IsPolyhedral(Kind) || IsQuadric(Kind); }
		FORCEINLINE bool IsMeshKind(ELeafKind Kind) { return Kind == ELeafKind::TriangleMesh || Kind == ELeafKind::HeightField; }

		using FFaceVertices = TArray<FVector, TInlineAllocator<8>>;

		void SwapContactSides(FContactArray& Contacts, int32 FirstIndex);

		// -------------------------------------------------------------------------------------------------------------
		// FPolyView: world-space face access for boxes (from min/max), convexes (FConvex API) and triangles
		// -------------------------------------------------------------------------------------------------------------

		class FPolyView
		{
		public:
			/// Build a view over a box / convex leaf (optionally scaled or instanced). Returns false for other types.
			static bool Make(const FImplicitObject& Leaf, const FTransform& TM, FPolyView& Out)
			{
				Out.TM = TM;
				Out.Scale = FVector(1.0);
				if (const FImplicitBox3* Box = Leaf.template GetObject<FImplicitBox3>())
				{
					Out.Kind = EKind::Box;
					Out.BoxMin = FVector(Box->Min());
					Out.BoxMax = FVector(Box->Max());
					return true;
				}
				if (const TImplicitObjectScaled<FImplicitBox3>* Scaled = Leaf.template GetObject<TImplicitObjectScaled<FImplicitBox3>>())
				{
					Out.Kind = EKind::Box;
					Out.Scale = FVector(Scaled->GetScale());
					Out.BoxMin = FVector(Scaled->GetUnscaledObject()->Min()) * Out.Scale;
					Out.BoxMax = FVector(Scaled->GetUnscaledObject()->Max()) * Out.Scale;
					// Negative scale components swap min / max.
					for (int32 Axis = 0; Axis < 3; ++Axis)
					{
						if (Out.BoxMin[Axis] > Out.BoxMax[Axis])
						{
							Swap(Out.BoxMin[Axis], Out.BoxMax[Axis]);
						}
					}
					return true;
				}
				if (const FImplicitConvex3* Convex = Leaf.template GetObject<FImplicitConvex3>())
				{
					Out.Kind = EKind::Convex;
					Out.Convex = Convex;
					return true;
				}
				if (const TImplicitObjectScaled<FImplicitConvex3>* Scaled = Leaf.template GetObject<TImplicitObjectScaled<FImplicitConvex3>>())
				{
					Out.Kind = EKind::Convex;
					Out.Convex = Scaled->GetUnscaledObject();
					Out.Scale = FVector(Scaled->GetScale());
					return true;
				}
				if (const TImplicitObjectInstanced<FImplicitConvex3>* Instanced = Leaf.template GetObject<TImplicitObjectInstanced<FImplicitConvex3>>())
				{
					Out.Kind = EKind::Convex;
					Out.Convex = Instanced->GetInstancedObject();
					return true;
				}
				return false;
			}

			/// View over a world-space triangle (single face).
			static FPolyView MakeTriangle(const FVector& V0, const FVector& V1, const FVector& V2)
			{
				FPolyView View;
				View.Kind = EKind::Triangle;
				View.TriVerts[0] = V0;
				View.TriVerts[1] = V1;
				View.TriVerts[2] = V2;
				View.TriNormal = FVector::CrossProduct(V1 - V0, V2 - V0).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
				return View;
			}

			int32 NumFaces() const
			{
				switch (Kind)
				{
					case EKind::Box: return 6;
					case EKind::Convex: return Convex->NumPlanes();
					default: return 1;
				}
			}

			/// Outward world-space normal of a face.
			FVector FaceNormal(int32 Face) const
			{
				switch (Kind)
				{
					case EKind::Box:
					{
						FVector Local(0.0);
						Local[Face / 2] = (Face % 2 == 0) ? 1.0 : -1.0;
						return TM.TransformVectorNoScale(Local);
					}
					case EKind::Convex:
					{
						FVec3 N, X;
						Convex->GetPlaneNX(Face, N, X);
						// Inverse-transpose for non-uniform scale.
						const FVector ScaledN = (FVector(N) / Scale).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
						return TM.TransformVectorNoScale(ScaledN);
					}
					default:
						return TriNormal;
				}
			}

			/// World-space vertices of a face, counter-clockwise about its outward normal.
			void FaceVertices(int32 Face, FFaceVertices& Out) const
			{
				Out.Reset();
				switch (Kind)
				{
					case EKind::Box:
					{
						const int32 A = Face / 2;
						const int32 B = (A + 1) % 3;
						const int32 C = (A + 2) % 3;
						const bool bPositive = (Face % 2 == 0);
						FVector V[4];
						for (int32 i = 0; i < 4; ++i)
						{
							V[i][A] = bPositive ? BoxMax[A] : BoxMin[A];
						}
						if (bPositive)
						{
							V[0][B] = BoxMin[B]; V[0][C] = BoxMin[C];
							V[1][B] = BoxMax[B]; V[1][C] = BoxMin[C];
							V[2][B] = BoxMax[B]; V[2][C] = BoxMax[C];
							V[3][B] = BoxMin[B]; V[3][C] = BoxMax[C];
						}
						else
						{
							V[0][B] = BoxMin[B]; V[0][C] = BoxMin[C];
							V[1][B] = BoxMin[B]; V[1][C] = BoxMax[C];
							V[2][B] = BoxMax[B]; V[2][C] = BoxMax[C];
							V[3][B] = BoxMax[B]; V[3][C] = BoxMin[C];
						}
						for (int32 i = 0; i < 4; ++i)
						{
							Out.Add(TM.TransformPositionNoScale(V[i]));
						}
						break;
					}
					case EKind::Convex:
					{
						const int32 Num = Convex->NumPlaneVertices(Face);
						for (int32 i = 0; i < Num; ++i)
						{
							const FVector Local = FVector(Convex->GetVertex(Convex->GetPlaneVertex(Face, i))) * Scale;
							Out.Add(TM.TransformPositionNoScale(Local));
						}
						// Negative scale flips the winding.
						if (Out.Num() >= 3)
						{
							const FVector Cross = FVector::CrossProduct(Out[1] - Out[0], Out[2] - Out[0]);
							if (FVector::DotProduct(Cross, FaceNormal(Face)) < 0.0)
							{
								Algo::Reverse(Out);
							}
						}
						break;
					}
					default:
						Out.Add(TriVerts[0]);
						Out.Add(TriVerts[1]);
						Out.Add(TriVerts[2]);
						break;
				}
			}

			/// Face whose outward normal is most aligned with 'Dir'; returns the alignment (dot product).
			int32 FindMostAlignedFace(const FVector& Dir, double& OutDot) const
			{
				int32 Best = 0;
				OutDot = -2.0;
				const int32 Num = NumFaces();
				for (int32 i = 0; i < Num; ++i)
				{
					const double Dot = FVector::DotProduct(FaceNormal(i), Dir);
					if (Dot > OutDot)
					{
						OutDot = Dot;
						Best = i;
					}
				}
				return Best;
			}

		private:
			enum class EKind : uint8 { Box, Convex, Triangle };
			EKind Kind = EKind::Box;
			FTransform TM = FTransform::Identity;
			FVector Scale = FVector(1.0);
			FVector BoxMin = FVector::ZeroVector;
			FVector BoxMax = FVector::ZeroVector;
			const FImplicitConvex3* Convex = nullptr;
			FVector TriVerts[3];
			FVector TriNormal = FVector::UpVector;
		};

		// -------------------------------------------------------------------------------------------------------------
		// Face clipping manifold (Bullet cbtPolyhedralContactClipping style)
		// -------------------------------------------------------------------------------------------------------------

		/// Sutherland-Hodgman clip of 'Polygon' against the half-space { p : dot(p - PlaneX, PlaneN) <= 0 }.
		void ClipPolygonAgainstPlane(const FFaceVertices& Polygon, const FVector& PlaneN, const FVector& PlaneX, FFaceVertices& Out)
		{
			Out.Reset();
			const int32 Num = Polygon.Num();
			if (Num == 0)
			{
				return;
			}
			FVector Prev = Polygon[Num - 1];
			double PrevDist = FVector::DotProduct(Prev - PlaneX, PlaneN);
			for (int32 i = 0; i < Num; ++i)
			{
				const FVector& Cur = Polygon[i];
				const double CurDist = FVector::DotProduct(Cur - PlaneX, PlaneN);
				if (PrevDist <= 0.0)
				{
					if (CurDist <= 0.0)
					{
						Out.Add(Cur);
					}
					else
					{
						Out.Add(FMath::Lerp(Prev, Cur, PrevDist / (PrevDist - CurDist)));
					}
				}
				else if (CurDist <= 0.0)
				{
					Out.Add(FMath::Lerp(Prev, Cur, PrevDist / (PrevDist - CurDist)));
					Out.Add(Cur);
				}
				Prev = Cur;
				PrevDist = CurDist;
			}
		}

		/// Build a face manifold between two polyhedra given the separating axis (unit, from A to B) found by GJK/EPA.
		/// Returns the number of contacts appended (0 when the clipped polygon is empty, e.g. edge-edge cases).
		int32 ClipFaceManifold(const FPolyView& A, const FPolyView& B, const FVector& AxisAToB, double CullDistance, FContactArray& Out)
		{
			double DotA, DotB;
			const int32 FaceA = A.FindMostAlignedFace(AxisAToB, DotA);
			const int32 FaceB = B.FindMostAlignedFace(-AxisAToB, DotB);

			// The reference face is the one most parallel to the separating axis (slight bias towards A for stability).
			const bool bRefIsA = DotA + 1e-3 >= DotB;
			const FPolyView& Ref = bRefIsA ? A : B;
			const FPolyView& Inc = bRefIsA ? B : A;
			const int32 RefFace = bRefIsA ? FaceA : FaceB;
			const int32 IncFace = bRefIsA ? FaceB : FaceA;

			const FVector RefN = Ref.FaceNormal(RefFace);
			FFaceVertices RefVerts, Polygon, Clipped;
			Ref.FaceVertices(RefFace, RefVerts);
			Inc.FaceVertices(IncFace, Polygon);
			if (RefVerts.Num() < 3 || Polygon.Num() < 3)
			{
				return 0;
			}

			// Clip the incident face against the side planes of the reference face.
			for (int32 i = 0; i < RefVerts.Num() && Polygon.Num() > 0; ++i)
			{
				const FVector& E0 = RefVerts[i];
				const FVector& E1 = RefVerts[(i + 1) % RefVerts.Num()];
				const FVector SideN = FVector::CrossProduct(E1 - E0, RefN).GetSafeNormal();
				if (SideN.IsNearlyZero())
				{
					continue;
				}
				ClipPolygonAgainstPlane(Polygon, SideN, E0, Clipped);
				Swap(Polygon, Clipped);
			}

			int32 Added = 0;
			const FVector& RefX = RefVerts[0];
			for (const FVector& P : Polygon)
			{
				const double Separation = FVector::DotProduct(P - RefX, RefN);
				if (Separation > CullDistance)
				{
					continue;
				}
				const FVector OnRef = P - RefN * Separation;

				FContact& C = Out.AddDefaulted_GetRef();
				C.Separation = Separation;
				if (bRefIsA)
				{
					C.PointA = OnRef;
					C.PointB = P;
					C.Normal = RefN;
				}
				else
				{
					C.PointA = P;
					C.PointB = OnRef;
					C.Normal = -RefN;
				}
				++Added;
			}
			return Added;
		}

		/// Manifold of an exact cylinder against the reference face of a polyhedron / triangle. The cylinder is a circle on each end
		/// cap: the lowest point of a rim (towards the face) is a sampled vertex by construction, so a rolling cylinder always touches
		/// at its true lowest point (no facet impacts), a cap lying flat gets a ring of points and a cylinder on its side the two
		/// end points of its contact line. AxisCylToRef: unit separating axis from the cylinder towards the face's body (from GJK).
		/// Contacts have the cylinder as A. Returns the number of contacts appended (0: nothing within reach, use the GJK point).
		int32 CylinderFaceManifold(const FLeafParams& Cyl, const FTransform& CylTM, const FPolyView& Ref, const FVector& AxisCylToRef, double CullDistance, FContactArray& Out)
		{
			double DotRef;
			const int32 RefFace = Ref.FindMostAlignedFace(-AxisCylToRef, DotRef);
			const FVector RefN = Ref.FaceNormal(RefFace);
			FFaceVertices RefVerts;
			Ref.FaceVertices(RefFace, RefVerts);
			if (RefVerts.Num() < 3)
			{
				return 0;
			}

			const FVector Axis = CylTM.TransformVectorNoScale(FVector::ZAxisVector);
			const FVector Centre = CylTM.GetLocation();
			// Radial direction of the lowest rim point; for a cap lying flat it is undefined and follows the body's own X axis
			// (stable from frame to frame, so the contact points stay put).
			FVector U = -RefN - Axis * FVector::DotProduct(-RefN, Axis);
			if (U.SizeSquared() < 1e-6)
			{
				U = CylTM.TransformVectorNoScale(FVector::XAxisVector);
			}
			U = U.GetSafeNormal(UE_SMALL_NUMBER, FVector::XAxisVector);
			const FVector W = FVector::CrossProduct(Axis, U);

			constexpr int32 RimPoints = 16;
			const FVector& RefX = RefVerts[0];
			TArray<FVector, TInlineAllocator<32>> Candidates;
			double Deepest = TNumericLimits<double>::Max();
			for (int32 Cap = 0; Cap < 2; ++Cap)
			{
				const FVector CapCentre = Centre + Axis * (Cap == 0 ? Cyl.CylinderHalfHeightCm : -Cyl.CylinderHalfHeightCm);
				FFaceVertices Polygon, Clipped;
				for (int32 k = 0; k < RimPoints; ++k)
				{
					const double Angle = 2.0 * PI * static_cast<double>(k) / RimPoints;
					Polygon.Add(CapCentre + (U * FMath::Cos(Angle) + W * FMath::Sin(Angle)) * Cyl.CylinderRadiusCm);
				}
				// Clip against the side planes of the reference face (as ClipFaceManifold does for the incident face)
				for (int32 i = 0; i < RefVerts.Num() && Polygon.Num() > 0; ++i)
				{
					const FVector& E0 = RefVerts[i];
					const FVector& E1 = RefVerts[(i + 1) % RefVerts.Num()];
					const FVector SideN = FVector::CrossProduct(E1 - E0, RefN).GetSafeNormal();
					if (SideN.IsNearlyZero())
					{
						continue;
					}
					ClipPolygonAgainstPlane(Polygon, SideN, E0, Clipped);
					Swap(Polygon, Clipped);
				}
				for (const FVector& P : Polygon)
				{
					Candidates.Add(P);
					Deepest = FMath::Min(Deepest, FVector::DotProduct(P - RefX, RefN));
				}
			}

			// Keep the points near the deepest one: a flat cap (all points level) keeps its ring, a cylinder on its side the two end
			// points of its contact line; the rim points beyond that (a facet away from the lowest point, mm above the face) would only
			// give an asymmetric selection after the manifold reduction.
			const double MaxSeparation = FMath::Min(CullDistance, Deepest + 0.04 * Cyl.CylinderRadiusCm);
			int32 Added = 0;
			for (const FVector& P : Candidates)
			{
				const double Separation = FVector::DotProduct(P - RefX, RefN);
				if (Separation > MaxSeparation)
				{
					continue;
				}
				FContact& C = Out.AddDefaulted_GetRef();
				C.Separation = Separation;
				C.PointA = P;
				C.PointB = P - RefN * Separation;
				C.Normal = -RefN;
				++Added;
			}
			return Added;
		}

		// -------------------------------------------------------------------------------------------------------------
		// GJK helpers
		// -------------------------------------------------------------------------------------------------------------

		struct FGJKResult
		{
			FVector PointA;   ///< world, on A
			FVector PointB;   ///< world, on B
			FVector Normal;   ///< world, unit, from A to B
			double Separation;
		};

		/// GJK/EPA between two convex objects placed at ATM / BTM. Returns false when they are farther than CullDistance.
		/// ThicknessA / ThicknessB inflate the shapes by a sphere-swept radius (cm).
		template <typename TypeA, typename TypeB>
		bool GJKQuery(const TypeA& A, const FTransform& ATM, const TypeB& B, const FTransform& BTM, double CullDistance, FGJKResult& Out,
		              double ThicknessA = 0.0, double ThicknessB = 0.0)
		{
			const FRigidTransform3 BToATM = FRigidTransform3(BTM).GetRelativeTransform(FRigidTransform3(ATM));

			FReal Penetration = 0;
			FVec3 ClosestA, ClosestB, Normal;
			int32 VertexA = INDEX_NONE, VertexB = INDEX_NONE;
			FVec3 InitialDir = BToATM.GetLocation().GetSafeNormal(UE_SMALL_NUMBER, FVec3(1, 0, 0));
			if (InitialDir.IsNearlyZero())
			{
				InitialDir = FVec3(1, 0, 0);
			}
			if (!GJKPenetration<true>(A, B, BToATM, Penetration, ClosestA, ClosestB, Normal, VertexA, VertexB, static_cast<FReal>(ThicknessA), static_cast<FReal>(ThicknessB), InitialDir))
			{
				return false;
			}
			if (-Penetration > CullDistance)
			{
				return false;
			}

			// GJKPenetration (unlike GJKPenetrationCore) returns the closest points on the shape surfaces, margins included.
			Out.PointA = ATM.TransformPositionNoScale(FVector(ClosestA));
			Out.PointB = ATM.TransformPositionNoScale(FVector(ClosestB));
			Out.Normal = ATM.TransformVectorNoScale(FVector(Normal)).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
			Out.Separation = -static_cast<double>(Penetration);
			return true;
		}

		void AppendGJKContact(const FGJKResult& R, FContactArray& Out)
		{
			FContact& C = Out.AddDefaulted_GetRef();
			C.PointA = R.PointA;
			C.PointB = R.PointB;
			C.Normal = R.Normal;
			C.Separation = R.Separation;
		}

		/// Dispatch a leaf to one of the concrete convex types the narrowphase supports.
		template <typename Func>
		bool VisitConvex(const FImplicitObject& Leaf, const Func& F)
		{
			if (const FImplicitSphere3* Sphere = Leaf.template GetObject<FImplicitSphere3>()) { F(*Sphere); return true; }
			if (const FImplicitCapsule3* Capsule = Leaf.template GetObject<FImplicitCapsule3>()) { F(*Capsule); return true; }
			if (const FImplicitBox3* Box = Leaf.template GetObject<FImplicitBox3>()) { F(*Box); return true; }
			if (const FImplicitConvex3* Convex = Leaf.template GetObject<FImplicitConvex3>()) { F(*Convex); return true; }
			if (const TImplicitObjectScaled<FImplicitBox3>* Scaled = Leaf.template GetObject<TImplicitObjectScaled<FImplicitBox3>>()) { F(*Scaled); return true; }
			if (const TImplicitObjectScaled<FImplicitConvex3>* Scaled = Leaf.template GetObject<TImplicitObjectScaled<FImplicitConvex3>>()) { F(*Scaled); return true; }
			if (const TImplicitObjectInstanced<FImplicitConvex3>* Instanced = Leaf.template GetObject<TImplicitObjectInstanced<FImplicitConvex3>>()) { F(*Instanced); return true; }
			return false;
		}

		/// GJK between two arbitrary supported convex leaves.
		bool GJKLeaves(const FImplicitObject& LeafA, const FTransform& ATM, const FImplicitObject& LeafB, const FTransform& BTM, double CullDistance, FGJKResult& Out)
		{
			bool bHit = false;
			VisitConvex(LeafA, [&](const auto& A)
			{
				VisitConvex(LeafB, [&](const auto& B)
				{
					bHit = GJKQuery(A, ATM, B, BTM, CullDistance, Out);
				});
			});
			return bHit;
		}

		/// GJK between a supported convex leaf and a world-space triangle.
		bool GJKLeafTriangle(const FImplicitObject& LeafA, const FTransform& ATM, const FVector& V0, const FVector& V1, const FVector& V2, double CullDistance, FGJKResult& Out)
		{
			// Triangle in A's space so the query has a single transform.
			const FTriangle LocalTri(FVec3(ATM.InverseTransformPositionNoScale(V0)), FVec3(ATM.InverseTransformPositionNoScale(V1)), FVec3(ATM.InverseTransformPositionNoScale(V2)));
			bool bHit = false;
			VisitConvex(LeafA, [&](const auto& A)
			{
				bHit = GJKQuery(A, ATM, LocalTri, ATM, CullDistance, Out);
			});
			return bHit;
		}

		// -------------------------------------------------------------------------------------------------------------
		// Pair routines
		// -------------------------------------------------------------------------------------------------------------

		/// Polyhedron A vs polyhedron B: GJK axis + face clipping (fallback: the GJK point).
		void PolyVsPoly(const FImplicitObject& LeafA, const FTransform& ATM, const FImplicitObject& LeafB, const FTransform& BTM, double CullDistance, FContactArray& Out,
		                const FLeafParams& ParamsA = FLeafParams(), const FLeafParams& ParamsB = FLeafParams())
		{
			FGJKResult GJK;
			if (!GJKLeaves(LeafA, ATM, LeafB, BTM, CullDistance, GJK))
			{
				return;
			}

			FPolyView ViewA, ViewB;
			if (FPolyView::Make(LeafA, ATM, ViewA) && FPolyView::Make(LeafB, BTM, ViewB))
			{
				const int32 First = Out.Num();
				int32 NumClipped = 0;
				bool bAnalytic = true;
				if (ParamsA.IsCylinder())
				{
					NumClipped = CylinderFaceManifold(ParamsA, ATM, ViewB, GJK.Normal, CullDistance, Out);
				}
				else if (ParamsB.IsCylinder())
				{
					NumClipped = CylinderFaceManifold(ParamsB, BTM, ViewA, -GJK.Normal, CullDistance, Out);
					SwapContactSides(Out, First);
				}
				else
				{
					bAnalytic = false;
					NumClipped = ClipFaceManifold(ViewA, ViewB, GJK.Normal, CullDistance, Out);
				}
				if (NumClipped > 0)
				{
					if (bAnalytic)
					{
						return;
					}
					// Make sure the deepest GJK point is represented (clipping can miss it for edge contacts).
					bool bHasDeep = false;
					for (int32 i = First; i < Out.Num(); ++i)
					{
						if (Out[i].Separation <= GJK.Separation + 0.05)
						{
							bHasDeep = true;
							break;
						}
					}
					if (!bHasDeep)
					{
						AppendGJKContact(GJK, Out);
					}
					return;
				}
			}
			AppendGJKContact(GJK, Out);
		}

		/// Polyhedron A vs world-space triangle (part of a mesh, heightfield or another polyhedron's face).
		void PolyVsTriangle(const FImplicitObject& LeafA, const FTransform& ATM, const FVector& V0, const FVector& V1, const FVector& V2, int32 FaceIndex, double CullDistance, FContactArray& Out,
		                    const FLeafParams& ParamsA = FLeafParams())
		{
			FGJKResult GJK;
			if (!GJKLeafTriangle(LeafA, ATM, V0, V1, V2, CullDistance, GJK))
			{
				return;
			}

			FPolyView ViewA;
			const int32 First = Out.Num();
			if (FPolyView::Make(LeafA, ATM, ViewA))
			{
				const FPolyView ViewTri = FPolyView::MakeTriangle(V0, V1, V2);
				const int32 NumClipped = ParamsA.IsCylinder() ? CylinderFaceManifold(ParamsA, ATM, ViewTri, GJK.Normal, CullDistance, Out)
				                                              : ClipFaceManifold(ViewA, ViewTri, GJK.Normal, CullDistance, Out);
				if (NumClipped == 0)
				{
					AppendGJKContact(GJK, Out);
				}
			}
			else
			{
				AppendGJKContact(GJK, Out);
			}
			for (int32 i = First; i < Out.Num(); ++i)
			{
				Out[i].FaceIndexB = FaceIndex;
			}
		}

		/// Sphere A vs world-space triangle.
		void SphereVsTriangle(const FImplicitSphere3& Sphere, const FTransform& ATM, const FVector& V0, const FVector& V1, const FVector& V2, int32 FaceIndex, double CullDistance, FContactArray& Out)
		{
			const FTriangle LocalTri(FVec3(ATM.InverseTransformPositionNoScale(V0)), FVec3(ATM.InverseTransformPositionNoScale(V1)), FVec3(ATM.InverseTransformPositionNoScale(V2)));
			FContactPointManifold Points;
			ConstructSphereTriangleOneShotManifold(Sphere, LocalTri, static_cast<FReal>(CullDistance), Points);
			for (int32 i = 0; i < Points.Num(); ++i)
			{
				const FContactPoint& P = Points[i];
				FContact& C = Out.AddDefaulted_GetRef();
				C.PointA = ATM.TransformPositionNoScale(FVector(P.ShapeContactPoints[0]));
				C.PointB = ATM.TransformPositionNoScale(FVector(P.ShapeContactPoints[1]));
				C.Normal = -ATM.TransformVectorNoScale(FVector(P.ShapeContactNormal)).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
				C.Separation = static_cast<double>(P.Phi);
				C.FaceIndexB = FaceIndex;
			}
		}

		/// Capsule A vs world-space triangle.
		void CapsuleVsTriangle(const FImplicitCapsule3& Capsule, const FTransform& ATM, const FVector& V0, const FVector& V1, const FVector& V2, int32 FaceIndex, double CullDistance, FContactArray& Out)
		{
			const FTriangle LocalTri(FVec3(ATM.InverseTransformPositionNoScale(V0)), FVec3(ATM.InverseTransformPositionNoScale(V1)), FVec3(ATM.InverseTransformPositionNoScale(V2)));
			FContactPointManifold Points;
			ConstructCapsuleTriangleOneShotManifold2(Capsule, LocalTri, static_cast<FReal>(CullDistance), Points);
			for (int32 i = 0; i < Points.Num(); ++i)
			{
				const FContactPoint& P = Points[i];
				FContact& C = Out.AddDefaulted_GetRef();
				C.PointA = ATM.TransformPositionNoScale(FVector(P.ShapeContactPoints[0]));
				C.PointB = ATM.TransformPositionNoScale(FVector(P.ShapeContactPoints[1]));
				C.Normal = -ATM.TransformVectorNoScale(FVector(P.ShapeContactNormal)).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
				C.Separation = static_cast<double>(P.Phi);
				C.FaceIndexB = FaceIndex;
			}
		}

		/// Convex A (any kind) vs one world-space triangle - dispatches on A's kind.
		void ConvexVsTriangle(ELeafKind KindA, const FImplicitObject& LeafA, const FTransform& ATM, const FVector& V0, const FVector& V1, const FVector& V2, int32 FaceIndex, double CullDistance, FContactArray& Out,
		                      const FLeafParams& ParamsA = FLeafParams())
		{
			switch (KindA)
			{
				case ELeafKind::Sphere:
					if (const FImplicitSphere3* Sphere = LeafA.template GetObject<FImplicitSphere3>())
					{
						SphereVsTriangle(*Sphere, ATM, V0, V1, V2, FaceIndex, CullDistance, Out);
						return;
					}
					break;
				case ELeafKind::Capsule:
					if (const FImplicitCapsule3* Capsule = LeafA.template GetObject<FImplicitCapsule3>())
					{
						CapsuleVsTriangle(*Capsule, ATM, V0, V1, V2, FaceIndex, CullDistance, Out);
						return;
					}
					break;
				default:
					PolyVsTriangle(LeafA, ATM, V0, V1, V2, FaceIndex, CullDistance, Out, ParamsA);
					return;
			}
			// Scaled sphere / capsule: GJK point.
			FGJKResult GJK;
			if (GJKLeafTriangle(LeafA, ATM, V0, V1, V2, CullDistance, GJK))
			{
				AppendGJKContact(GJK, Out);
				Out.Last().FaceIndexB = FaceIndex;
			}
		}

		/// World-space bounds of a leaf at a transform, grown by the cull distance.
		FAABB3 CulledWorldBounds(const FImplicitObject& Leaf, const FTransform& TM, double CullDistance)
		{
			FAABB3 Bounds = Leaf.BoundingBox().TransformedAABB(TM);
			Bounds.Thicken(static_cast<FReal>(CullDistance));
			return Bounds;
		}

		/// Convex A vs the triangles of a mesh / heightfield B, enumerated in world space by 'ForEachTriangle'.
		template <typename TriangleSource>
		void ConvexVsTriangles(ELeafKind KindA, const FImplicitObject& LeafA, const FTransform& ATM, double CullDistance, const TriangleSource& ForEachTriangle, FContactArray& Out,
		                       const FLeafParams& ParamsA = FLeafParams())
		{
			const FAABB3 BoundsA = CulledWorldBounds(LeafA, ATM, CullDistance);
			const FVector CentreA = FVector(BoundsA.Center());

			ForEachTriangle([&](const FVector& V0, const FVector& V1, const FVector& V2, int32 FaceIndex)
			{
				// Single-sided: A must be on the front side of the triangle.
				const FVector TriN = FVector::CrossProduct(V1 - V0, V2 - V0);
				if (FVector::DotProduct(TriN, CentreA - V0) <= 0.0)
				{
					return;
				}
				const FVec3 P0(V0);
				FAABB3 TriBounds(P0, P0);
				TriBounds.GrowToInclude(FVec3(V1));
				TriBounds.GrowToInclude(FVec3(V2));
				if (!TriBounds.Intersects(BoundsA))
				{
					return;
				}
				ConvexVsTriangle(KindA, LeafA, ATM, V0, V1, V2, FaceIndex, CullDistance, Out, ParamsA);
				});
		}

		/// Convex A vs polyhedron B through B's face triangles (used for capsules, which need the exported
		/// capsule-triangle manifold for stable 2-point contacts).
		void ConvexVsPolyFaces(ELeafKind KindA, const FImplicitObject& LeafA, const FTransform& ATM, const FImplicitObject& LeafB, const FTransform& BTM, double CullDistance, FContactArray& Out)
		{
			FPolyView ViewB;
			if (!FPolyView::Make(LeafB, BTM, ViewB))
			{
				FGJKResult GJK;
				if (GJKLeaves(LeafA, ATM, LeafB, BTM, CullDistance, GJK))
				{
					AppendGJKContact(GJK, Out);
				}
				return;
			}

			const int32 First = Out.Num();
			ConvexVsTriangles(KindA, LeafA, ATM, CullDistance, [&ViewB](const auto& Visit)
			{
				FFaceVertices Verts;
				const int32 NumFaces = ViewB.NumFaces();
				for (int32 Face = 0; Face < NumFaces; ++Face)
				{
					ViewB.FaceVertices(Face, Verts);
					for (int32 i = 1; i + 1 < Verts.Num(); ++i)
					{
						Visit(Verts[0], Verts[i], Verts[i + 1], Face);
					}
				}
			}, Out);
			// Face-triangle indices are meaningless for polyhedra.
			for (int32 i = First; i < Out.Num(); ++i)
			{
				Out[i].FaceIndexB = INDEX_NONE;
			}
		}

		/// Convex A vs a triangle mesh (possibly scaled / instanced).
		void ConvexVsTriangleMesh(ELeafKind KindA, const FImplicitObject& LeafA, const FTransform& ATM, const FImplicitObject& MeshLeaf, const FTransform& MeshTM, double CullDistance, FContactArray& Out,
		                          const FLeafParams& ParamsA = FLeafParams())
		{
			const FTriangleMeshImplicitObject* Mesh = nullptr;
			FVector MeshScale(1.0);
			if (const TImplicitObjectScaled<FTriangleMeshImplicitObject>* Scaled = MeshLeaf.template GetObject<TImplicitObjectScaled<FTriangleMeshImplicitObject>>())
			{
				Mesh = Scaled->GetUnscaledObject();
				MeshScale = FVector(Scaled->GetScale());
			}
			else if (const TImplicitObjectInstanced<FTriangleMeshImplicitObject>* Instanced = MeshLeaf.template GetObject<TImplicitObjectInstanced<FTriangleMeshImplicitObject>>())
			{
				Mesh = Instanced->GetInstancedObject();
			}
			else
			{
				Mesh = MeshLeaf.template GetObject<FTriangleMeshImplicitObject>();
			}
			if (!Mesh)
			{
				return;
			}

			// Query bounds in unscaled mesh space; triangles come out in world space (the query transform carries the
			// mesh scale, as FTriangleMeshImplicitObject::VisitTriangles expects).
			const FTransform ScaledMeshTM(MeshTM.GetRotation(), MeshTM.GetLocation(), MeshScale);
			const FAABB3 QueryBounds = CulledWorldBounds(LeafA, ATM, CullDistance).TransformedAABB(ScaledMeshTM.Inverse());

			ConvexVsTriangles(KindA, LeafA, ATM, CullDistance, [&](const auto& Visit)
			{
				Mesh->VisitTriangles(QueryBounds, FRigidTransform3(ScaledMeshTM), [&](const FTriangle& Tri, int32 TriangleIndex, int32, int32, int32)
				{
					Visit(FVector(Tri.GetVertex(0)), FVector(Tri.GetVertex(1)), FVector(Tri.GetVertex(2)), TriangleIndex);
				});
			}, Out, ParamsA);
		}

		/// Convex A vs a heightfield (landscape). FHeightField bakes its scale into the produced triangles.
		void ConvexVsHeightField(ELeafKind KindA, const FImplicitObject& LeafA, const FTransform& ATM, const FImplicitObject& FieldLeaf, const FTransform& FieldTM, double CullDistance, FContactArray& Out,
		                         const FLeafParams& ParamsA = FLeafParams())
		{
			const FHeightField* Field = FieldLeaf.template GetObject<FHeightField>();
			if (!Field)
			{
				if (const TImplicitObjectScaled<FHeightField>* Scaled = FieldLeaf.template GetObject<TImplicitObjectScaled<FHeightField>>())
				{
					Field = Scaled->GetUnscaledObject();
				}
			}
			if (!Field)
			{
				return;
			}

			const FAABB3 QueryBounds = CulledWorldBounds(LeafA, ATM, CullDistance).TransformedAABB(FieldTM.Inverse());
			ConvexVsTriangles(KindA, LeafA, ATM, CullDistance, [&](const auto& Visit)
			{
				Field->VisitTriangles(QueryBounds, FRigidTransform3(FieldTM), [&](const FTriangle& Tri, int32 TriangleIndex, int32, int32, int32)
				{
					Visit(FVector(Tri.GetVertex(0)), FVector(Tri.GetVertex(1)), FVector(Tri.GetVertex(2)), TriangleIndex);
				});
			}, Out, ParamsA);
		}

		/// Swap the roles of A and B in contacts appended after 'FirstIndex'.
		void SwapContactSides(FContactArray& Contacts, int32 FirstIndex)
		{
			for (int32 i = FirstIndex; i < Contacts.Num(); ++i)
			{
				FContact& C = Contacts[i];
				Swap(C.PointA, C.PointB);
				C.Normal = -C.Normal;
				C.FaceIndexB = INDEX_NONE;
			}
		}

		// -------------------------------------------------------------------------------------------------------------
		// Mesh vs triangle source (mesh / heightfield)
		// -------------------------------------------------------------------------------------------------------------

		/// Unwrap a (possibly scaled / instanced) triangle mesh leaf. Returns null for other leaves.
		const FTriangleMeshImplicitObject* ResolveTriangleMesh(const FImplicitObject& MeshLeaf, FVector& OutScale)
		{
			OutScale = FVector(1.0);
			if (const TImplicitObjectScaled<FTriangleMeshImplicitObject>* Scaled = MeshLeaf.template GetObject<TImplicitObjectScaled<FTriangleMeshImplicitObject>>())
			{
				OutScale = FVector(Scaled->GetScale());
				return Scaled->GetUnscaledObject();
			}
			if (const TImplicitObjectInstanced<FTriangleMeshImplicitObject>* Instanced = MeshLeaf.template GetObject<TImplicitObjectInstanced<FTriangleMeshImplicitObject>>())
			{
				return Instanced->GetInstancedObject();
			}
			return MeshLeaf.template GetObject<FTriangleMeshImplicitObject>();
		}

		/// Unwrap a (possibly scaled) heightfield leaf. Returns null for other leaves.
		const FHeightField* ResolveHeightField(const FImplicitObject& FieldLeaf)
		{
			if (const FHeightField* Field = FieldLeaf.template GetObject<FHeightField>())
			{
				return Field;
			}
			if (const TImplicitObjectScaled<FHeightField>* Scaled = FieldLeaf.template GetObject<TImplicitObjectScaled<FHeightField>>())
			{
				return Scaled->GetUnscaledObject();
			}
			return nullptr;
		}

		/// Enumerate the world-space triangles of a mesh / heightfield leaf that overlap 'WorldBounds'.
		template <typename Func>
		bool ForEachTriangleInBounds(ELeafKind Kind, const FImplicitObject& Leaf, const FTransform& TM, const FAABB3& WorldBounds, const Func& Visit)
		{
			if (Kind == ELeafKind::TriangleMesh)
			{
				FVector Scale;
				const FTriangleMeshImplicitObject* Mesh = ResolveTriangleMesh(Leaf, Scale);
				if (!Mesh)
				{
					return false;
				}
				const FTransform ScaledTM(TM.GetRotation(), TM.GetLocation(), Scale);
				const FAABB3 QueryBounds = WorldBounds.TransformedAABB(ScaledTM.Inverse());
				Mesh->VisitTriangles(QueryBounds, FRigidTransform3(ScaledTM), [&](const FTriangle& Tri, int32 TriangleIndex, int32, int32, int32)
				{
					Visit(FVector(Tri.GetVertex(0)), FVector(Tri.GetVertex(1)), FVector(Tri.GetVertex(2)), TriangleIndex);
				});
				return true;
			}
			if (Kind == ELeafKind::HeightField)
			{
				const FHeightField* Field = ResolveHeightField(Leaf);
				if (!Field)
				{
					return false;
				}
				const FAABB3 QueryBounds = WorldBounds.TransformedAABB(TM.Inverse());
				Field->VisitTriangles(QueryBounds, FRigidTransform3(TM), [&](const FTriangle& Tri, int32 TriangleIndex, int32, int32, int32)
				{
					Visit(FVector(Tri.GetVertex(0)), FVector(Tri.GetVertex(1)), FVector(Tri.GetVertex(2)), TriangleIndex);
				});
				return true;
			}
			return false;
		}

		/// World-space triangle with cached derived data.
		struct FWorldTriangle
		{
			FVector V[3];
			FVector Normal;     ///< unit, winding order
			FVector Centroid;
			FAABB3 Bounds;
			int32 FaceIndex = INDEX_NONE;

			FWorldTriangle(const FVector& V0, const FVector& V1, const FVector& V2, int32 InFaceIndex)
				: Bounds(FVec3(V0), FVec3(V0)), FaceIndex(InFaceIndex)
			{
				V[0] = V0;
				V[1] = V1;
				V[2] = V2;
				Normal = FVector::CrossProduct(V1 - V0, V2 - V0).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
				Centroid = (V0 + V1 + V2) / 3.0;
				Bounds.GrowToInclude(FVec3(V1));
				Bounds.GrowToInclude(FVec3(V2));
			}
		};

		/// Triangle mesh A (sphere-swept by RadiusA) vs the triangles of mesh / heightfield B (swept by RadiusB).
		/// Every AABB-overlapping, mutually front-facing triangle pair goes through GJK/EPA with the swept radii as GJK
		/// thickness, which reproduces Bullet's cbtCEtriangleShape margin semantics: a mesh tire sinks into the ground by
		/// (RadiusA - distance) before the contact flips. One contact per touching pair; callers reduce the manifold.
		void MeshVsTriangleSource(const FImplicitObject& LeafA, const FTransform& ATM, double RadiusA,
		                          ELeafKind KindB, const FImplicitObject& LeafB, const FTransform& BTM, double RadiusB,
		                          double CullDistance, FContactArray& Out)
		{
			FVector ScaleA;
			const FTriangleMeshImplicitObject* MeshA = ResolveTriangleMesh(LeafA, ScaleA);
			if (!MeshA)
			{
				return;
			}

			// Region of interest: A's culled bounds intersected with B's bounds (both inflated by the swept radii).
			const double Inflation = CullDistance + RadiusA + RadiusB;
			FAABB3 Region = CulledWorldBounds(LeafA, ATM, Inflation);
			FAABB3 BoundsB = LeafB.BoundingBox().TransformedAABB(BTM);
			BoundsB.Thicken(static_cast<FReal>(Inflation));
			if (!Region.Intersects(BoundsB))
			{
				return;
			}
			Region = Region.GetIntersection(BoundsB);

			// Triangles of A inside the region (usually the few facing the ground).
			TArray<FWorldTriangle, TInlineAllocator<64>> TrianglesA;
			{
				const FTransform ScaledATM(ATM.GetRotation(), ATM.GetLocation(), ScaleA);
				const FAABB3 QueryBoundsA = Region.TransformedAABB(ScaledATM.Inverse());
				MeshA->VisitTriangles(QueryBoundsA, FRigidTransform3(ScaledATM), [&](const FTriangle& Tri, int32 TriangleIndex, int32, int32, int32)
				{
					TrianglesA.Emplace(FVector(Tri.GetVertex(0)), FVector(Tri.GetVertex(1)), FVector(Tri.GetVertex(2)), TriangleIndex);
				});
			}
			if (TrianglesA.Num() == 0)
			{
				return;
			}

			// A triangle whose centroid sank deeper than the swept radii behind the other surface is treated as
			// having tunnelled (no contact), like Bullet's per-triangle one-sidedness.
			const double BackTolerance = RadiusA + RadiusB;
			const FTransform Identity = FTransform::Identity;

			ForEachTriangleInBounds(KindB, LeafB, BTM, Region, [&](const FVector& W0, const FVector& W1, const FVector& W2, int32 FaceIndexB)
			{
				const FWorldTriangle TriB(W0, W1, W2, FaceIndexB);
				FAABB3 TriBBounds = TriB.Bounds;
				TriBBounds.Thicken(static_cast<FReal>(Inflation));
				const FTriangle ChaosTriB{FVec3(W0), FVec3(W1), FVec3(W2)};

				for (const FWorldTriangle& TriA : TrianglesA)
				{
					if (!TriA.Bounds.Intersects(TriBBounds))
					{
						continue;
					}
					// Mutually front-facing (with the swept tolerance).
					if (FVector::DotProduct(TriB.Normal, TriA.Centroid - TriB.V[0]) < -BackTolerance ||
						FVector::DotProduct(TriA.Normal, TriB.Centroid - TriA.V[0]) < -BackTolerance)
					{
						continue;
					}

					const FTriangle ChaosTriA{FVec3(TriA.V[0]), FVec3(TriA.V[1]), FVec3(TriA.V[2])};
					FGJKResult GJK;
					if (!GJKQuery(ChaosTriA, Identity, ChaosTriB, Identity, CullDistance, GJK, RadiusA, RadiusB))
					{
						continue;
					}
					// The separating axis must not point back through B's surface (coplanar / edge cases).
					if (FVector::DotProduct(GJK.Normal, TriB.Normal) > 0.0)
					{
						GJK.Normal = -TriB.Normal;
					}
					AppendGJKContact(GJK, Out);
					Out.Last().FaceIndexB = FaceIndexB;
				}
			});
		}
	}

	ELeafKind ClassifyLeaf(const FImplicitObject& Leaf)
	{
		switch (GetInnerType(Leaf.GetType()))
		{
			case ImplicitObjectType::Sphere: return ELeafKind::Sphere;
			case ImplicitObjectType::Capsule: return ELeafKind::Capsule;
			case ImplicitObjectType::Box: return ELeafKind::Box;
			case ImplicitObjectType::Convex: return ELeafKind::Convex;
			case ImplicitObjectType::TriangleMesh: return ELeafKind::TriangleMesh;
			case ImplicitObjectType::HeightField: return ELeafKind::HeightField;
			default: return ELeafKind::Unsupported;
		}
	}

	bool IsPairSupported(ELeafKind A, ELeafKind B)
	{
		if (A == ELeafKind::Unsupported || B == ELeafKind::Unsupported)
		{
			return false;
		}
		// Heightfield vs heightfield has no meaning (two static landscapes); everything else is handled.
		return !(A == ELeafKind::HeightField && B == ELeafKind::HeightField);
	}

	bool GenerateContacts(const FImplicitObject& LeafA, const FTransform& ATM, const FImplicitObject& LeafB, const FTransform& BTM, double CullDistance, FContactArray& Out,
	                      const FLeafParams& ParamsA, const FLeafParams& ParamsB)
	{
		const ELeafKind KindA = ClassifyLeaf(LeafA);
		const ELeafKind KindB = ClassifyLeaf(LeafB);
		if (!IsPairSupported(KindA, KindB))
		{
			return false;
		}

		// Mesh vs mesh / heightfield: the triangle mesh side is A (a heightfield never moves, so it is always B).
		if (IsMeshKind(KindA) && IsMeshKind(KindB))
		{
			if (KindA == ELeafKind::HeightField)
			{
				const int32 First = Out.Num();
				MeshVsTriangleSource(LeafB, BTM, ParamsB.SweptRadiusCm, KindA, LeafA, ATM, ParamsA.SweptRadiusCm, CullDistance, Out);
				SwapContactSides(Out, First);
				return true;
			}
			MeshVsTriangleSource(LeafA, ATM, ParamsA.SweptRadiusCm, KindB, LeafB, BTM, ParamsB.SweptRadiusCm, CullDistance, Out);
			return true;
		}

		// Make sure the convex side is A for the convex-vs-mesh cases (swap back afterwards).
		if (IsMeshKind(KindA))
		{
			const int32 First = Out.Num();
			const bool bResult = GenerateContacts(LeafB, BTM, LeafA, ATM, CullDistance, Out, ParamsB, ParamsA);
			SwapContactSides(Out, First);
			return bResult;
		}

		// A is convex here.
		if (KindB == ELeafKind::TriangleMesh)
		{
			ConvexVsTriangleMesh(KindA, LeafA, ATM, LeafB, BTM, CullDistance, Out, ParamsA);
			return true;
		}
		if (KindB == ELeafKind::HeightField)
		{
			ConvexVsHeightField(KindA, LeafA, ATM, LeafB, BTM, CullDistance, Out, ParamsA);
			return true;
		}

		// Both convex.
		if (KindA == ELeafKind::Sphere || KindB == ELeafKind::Sphere || (KindA == ELeafKind::Capsule && KindB == ELeafKind::Capsule))
		{
			// One exact contact point is all a sphere (or a capsule pair) needs.
			FGJKResult GJK;
			if (GJKLeaves(LeafA, ATM, LeafB, BTM, CullDistance, GJK))
			{
				AppendGJKContact(GJK, Out);
			}
			return true;
		}

		if (KindA == ELeafKind::Capsule)
		{
			ConvexVsPolyFaces(KindA, LeafA, ATM, LeafB, BTM, CullDistance, Out);
			return true;
		}
		if (KindB == ELeafKind::Capsule)
		{
			const int32 First = Out.Num();
			ConvexVsPolyFaces(KindB, LeafB, BTM, LeafA, ATM, CullDistance, Out);
			SwapContactSides(Out, First);
			return true;
		}

		// Polyhedron vs polyhedron.
		PolyVsPoly(LeafA, ATM, LeafB, BTM, CullDistance, Out, ParamsA, ParamsB);
		return true;
	}

	void ReduceContacts(FContactArray& Contacts, int32 MaxPoints)
	{
		if (Contacts.Num() <= MaxPoints || MaxPoints <= 0)
		{
			return;
		}

		FContactArray Kept;
		Kept.Reserve(MaxPoints);
		TArray<bool, TInlineAllocator<32>> Used;
		Used.Init(false, Contacts.Num());

		auto Take = [&](int32 Index)
		{
			Used[Index] = true;
			Kept.Add(Contacts[Index]);
		};

		// 1. Deepest point.
		{
			int32 Best = 0;
			for (int32 i = 1; i < Contacts.Num(); ++i)
			{
				if (Contacts[i].Separation < Contacts[Best].Separation)
				{
					Best = i;
				}
			}
			Take(Best);
		}

		// 2. Farthest from the deepest point.
		if (Kept.Num() < MaxPoints)
		{
			int32 Best = INDEX_NONE;
			double BestDist = -1.0;
			for (int32 i = 0; i < Contacts.Num(); ++i)
			{
				if (Used[i]) continue;
				const double Dist = FVector::DistSquared(Contacts[i].PointA, Kept[0].PointA);
				if (Dist > BestDist)
				{
					BestDist = Dist;
					Best = i;
				}
			}
			if (Best != INDEX_NONE)
			{
				Take(Best);
			}
		}

		// 3./4. Maximise the covered area: farthest from the segment, then from the kept points.
		while (Kept.Num() < MaxPoints)
		{
			int32 Best = INDEX_NONE;
			double BestScore = -1.0;
			for (int32 i = 0; i < Contacts.Num(); ++i)
			{
				if (Used[i]) continue;
				double Score = 0.0;
				if (Kept.Num() == 2)
				{
					Score = FMath::PointDistToSegmentSquared(Contacts[i].PointA, Kept[0].PointA, Kept[1].PointA);
				}
				else
				{
					Score = TNumericLimits<double>::Max();
					for (const FContact& K : Kept)
					{
						Score = FMath::Min(Score, FVector::DistSquared(Contacts[i].PointA, K.PointA));
					}
				}
				if (Score > BestScore)
				{
					BestScore = Score;
					Best = i;
				}
			}
			if (Best == INDEX_NONE)
			{
				break;
			}
			Take(Best);
		}

		Contacts = MoveTemp(Kept);
	}
}
