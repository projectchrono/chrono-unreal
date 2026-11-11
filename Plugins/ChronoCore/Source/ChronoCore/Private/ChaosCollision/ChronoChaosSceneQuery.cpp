// ChronoChaosSceneQuery - physics-thread ray casts against Chrono proxies and the bound solver's geometry.

#include "ChaosCollision/ChronoChaosSceneQuery.h"

#include "Chaos/Collision/CollisionFilter.h"
#include "Chaos/GeometryQueries.h"
#include "Chaos/ImplicitObject.h"
#include "Chaos/ImplicitObjectType.h"
#include "Chaos/ISpatialAcceleration.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/PhysicalMaterials.h"
#include "Chaos/ShapeInstance.h"
#include "PBDRigidsSolver.h"

namespace ChronoChaosSceneQuery
{
	using namespace Chaos;

	bool IsUnrealShapeCollidable(const chrono::ChCollisionSystemChaos::Impl& Impl, const FPerShapeData& Shape)
	{
		const FImplicitObject* Leaf = Shape.GetLeafGeometry();
		if (!Leaf)
		{
			return false;
		}
		// Same rules Chaos applies to sim pairs: sim-enabled, valid filter, triangle meshes only with
		// complex-as-simple, and a blocking response between the proxy filter and the shape filter.
		if (!DoCollide(GetInnerType(Leaf->GetType()), &Shape))
		{
			return false;
		}
		return Impl.ShapeFilter.NarrowFilter(Shape.GetShapeFilterData()) == Filter::ENarrowFilterResult::Block;
	}

	bool RaycastImplicit(const FImplicitObject& Geometry, const FTransform& Pose, const FVector& Start, const FVector& Dir, double Length,
	                     double& OutTime, FVector& OutPoint, FVector& OutNormal)
	{
		const FVec3 LocalStart = Pose.InverseTransformPositionNoScale(Start);
		const FVec3 LocalDir = Pose.InverseTransformVectorNoScale(Dir);

		FReal Time = 0;
		FVec3 LocalPoint, LocalNormal;
		int32 FaceIndex = INDEX_NONE;
		if (!Geometry.Raycast(LocalStart, LocalDir, static_cast<FReal>(Length), /*Thickness*/ 0, Time, LocalPoint, LocalNormal, FaceIndex))
		{
			return false;
		}

		OutTime = static_cast<double>(Time);
		OutPoint = Pose.TransformPositionNoScale(FVector(LocalPoint));
		OutNormal = Pose.TransformVectorNoScale(FVector(LocalNormal)).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
		return true;
	}

	bool RaycastChronoProxies(const chrono::ChCollisionSystemChaos::Impl& Impl, const FVector& Start, const FVector& Dir, double Length,
	                          const chrono::ChCollisionModel* OnlyModel, FRayHit& InOutBest)
	{
		bool bAnyHit = false;
		for (chrono::ChCollisionModelChaos* Model : Impl.Proxies)
		{
			if (OnlyModel && Model->GetModel() != OnlyModel)
			{
				continue;
			}
			const chrono::ChCollisionModelChaos::Data* Data = Model->GetData();
			if (!Data || !Data->Geometry.IsValid())
			{
				continue;
			}

			// Coarse reject on the world bounds (fresh Chrono pose) before touching the geometry.
			if (!Data->WorldBounds.IsEmpty())
			{
				const FVec3 RayStart(Start);
				FAABB3 RayBounds(RayStart, RayStart);
				RayBounds.GrowToInclude(FVec3(Start + Dir * Length));
				if (!Data->WorldBounds.Intersects(RayBounds))
				{
					continue;
				}
			}

			// Per union child so the hit can be attributed to a Chrono shape.
			for (const FChronoShapeSlot& Slot : Data->Geometry.Slots)
			{
				double Time;
				FVector Point, Normal;
				if (Slot.Root && RaycastImplicit(*Slot.Root, Data->PoseUE, Start, Dir, FMath::Min(Length, InOutBest.Time), Time, Point, Normal) && Time < InOutBest.Time)
				{
					InOutBest.bHit = true;
					InOutBest.Time = Time;
					InOutBest.Point = Point;
					InOutBest.Normal = Normal;
					InOutBest.Model = Model;
					InOutBest.Shape = Slot.Shape;
					InOutBest.Particle = nullptr;
					InOutBest.ParticleShape = nullptr;
					bAnyHit = true;
				}
			}
		}
		return bAnyHit;
	}

	namespace
	{
		/// Visits the solver's internal acceleration structure and keeps the closest hit on non-Chrono geometry.
		class FUnrealRayVisitor : public ISpatialVisitor<FAccelerationStructureHandle, FReal>
		{
		public:
			FUnrealRayVisitor(const chrono::ChCollisionSystemChaos::Impl& InImpl, const FVector& InStart, const FVector& InDir, FRayHit& InBest)
				: Impl(InImpl), Start(InStart), Dir(InDir), Best(InBest)
			{
			}

			bool bAnyHit = false;

			virtual bool Overlap(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance) override { return true; }
			virtual bool Sweep(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance, FQueryFastData& CurData) override { return true; }

			virtual bool Raycast(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance, FQueryFastData& CurData) override
			{
				const FGeometryParticleHandle* Particle = Instance.Payload.GetGeometryParticleHandle_PhysicsThread();
				if (!Particle || Impl.ParticleToModel.Contains(Particle))
				{
					return true;   // Chrono proxies are handled with their fresh poses by RaycastChronoProxies
				}

				const FTransform ParticlePose(Particle->GetR(), Particle->GetX());
				for (const TUniquePtr<FPerShapeData>& Shape : Particle->ShapesArray())
				{
					if (!Shape || !IsUnrealShapeCollidable(Impl, *Shape))
					{
						continue;
					}
					const FImplicitObject* Geometry = Shape->GetGeometry();
					if (!Geometry)
					{
						continue;
					}

					double Time;
					FVector Point, Normal;
					if (RaycastImplicit(*Geometry, ParticlePose, Start, Dir, static_cast<double>(CurData.CurrentLength), Time, Point, Normal) && Time < Best.Time)
					{
						Best.bHit = true;
						Best.Time = Time;
						Best.Point = Point;
						Best.Normal = Normal;
						Best.Model = nullptr;
						Best.Shape = nullptr;
						Best.Particle = Particle;
						Best.ParticleShape = Shape.Get();
						bAnyHit = true;
						CurData.SetLength(FMath::Max(static_cast<FReal>(Time), static_cast<FReal>(UE_SMALL_NUMBER)));
					}
				}
				return true;
			}

		private:
			const chrono::ChCollisionSystemChaos::Impl& Impl;
			FVector Start;
			FVector Dir;
			FRayHit& Best;
		};
	}

	bool RaycastUnrealGeometry(const chrono::ChCollisionSystemChaos::Impl& Impl, const FVector& Start, const FVector& Dir, double Length, FRayHit& InOutBest)
	{
		if (!Impl.IsBound() || Length <= 0.0)
		{
			return false;
		}
		const ISpatialAccelerationCollection<FAccelerationStructureHandle, FReal, 3>* Acceleration = Impl.Solver->GetInternalAccelerationStructure_Internal();
		if (!Acceleration)
		{
			return false;
		}

		FUnrealRayVisitor Visitor(Impl, Start, Dir, InOutBest);
		Acceleration->Raycast(FVec3(Start), FVec3(Dir), static_cast<FReal>(FMath::Min(Length, InOutBest.Time)), Visitor);
		return Visitor.bAnyHit;
	}

	const FChaosPhysicsMaterial* GetShapeMaterial(const chrono::ChCollisionSystemChaos::Impl& Impl, const FPerShapeData& Shape)
	{
		if (!Impl.Solver || Shape.NumMaterials() == 0)
		{
			return nullptr;
		}
		// Per-face material masks (triangle meshes) are not resolved: the first material stands for the whole shape.
		return Shape.GetMaterial(0).GetInternal(&Impl.Solver->GetSimMaterials());
	}

	namespace
	{
		/// Sweep one implicit object (scene geometry 'A' at 'APose') with the swept shape; fills a hit on success.
		bool SweepAgainstImplicit(const FImplicitObject& A, const FTransform& APose, const FImplicitObject& Swept, const FTransform& SweptPose,
		                          const FVector& Dir, double Length, FSweepHit& OutHit)
		{
			FReal Time = 0;
			FVec3 Position, Normal, FaceNormal;
			int32 FaceIndex = INDEX_NONE;
			if (!SweepQuery(A, FRigidTransform3(APose), Swept, FRigidTransform3(SweptPose), FVec3(Dir), static_cast<FReal>(Length),
			                Time, Position, Normal, FaceIndex, FaceNormal, /*Thickness*/ 0, /*bComputeMTD*/ true))
			{
				return false;
			}

			OutHit.Time = FMath::Max(static_cast<double>(Time), 0.0);
			OutHit.bStartPenetrating = Time <= 0;
			OutHit.Point = FVector(Position);
			OutHit.Normal = FVector(Normal).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
			return true;
		}

		/// World bounds of the swept volume (start pose bounds grown along the sweep).
		FAABB3 SweptBounds(const FImplicitObject& Swept, const FTransform& SweptPose, const FVector& Dir, double Length)
		{
			FAABB3 Bounds = Swept.BoundingBox().TransformedAABB(FRigidTransform3(SweptPose));
			const FVec3 Offset(Dir * Length);
			FAABB3 EndBounds = Bounds;
			EndBounds.MoveByVector(Offset);
			Bounds.GrowToInclude(EndBounds);
			Bounds.Thicken(static_cast<FReal>(UE_KINDA_SMALL_NUMBER));
			return Bounds;
		}

		/// Overlap visitor that sweeps the swept shape against every collidable non-Chrono shape in the query bounds.
		class FUnrealSweepVisitor : public ISpatialVisitor<FAccelerationStructureHandle, FReal>
		{
		public:
			FUnrealSweepVisitor(const chrono::ChCollisionSystemChaos::Impl& InImpl, const FImplicitObject& InSwept, const FTransform& InSweptPose,
			                    const FVector& InDir, double InLength, const FAABB3& InQueryBounds, TArray<FSweepHit>& InOut)
				: Impl(InImpl), Swept(InSwept), SweptPose(InSweptPose), Dir(InDir), Length(InLength), QueryBounds(InQueryBounds), Out(InOut)
			{
			}

			bool bAnyHit = false;

			virtual bool Overlap(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance) override
			{
				const FGeometryParticleHandle* Particle = Instance.Payload.GetGeometryParticleHandle_PhysicsThread();
				if (!Particle || Impl.ParticleToModel.Contains(Particle))
				{
					return true;   // Chrono proxies are handled with their fresh poses by SweepChronoProxies
				}

				const FTransform ParticlePose(Particle->GetR(), Particle->GetX());
				for (const TUniquePtr<FPerShapeData>& Shape : Particle->ShapesArray())
				{
					if (!Shape || !IsUnrealShapeCollidable(Impl, *Shape))
					{
						continue;
					}
					const FImplicitObject* Geometry = Shape->GetGeometry();
					if (!Geometry)
					{
						continue;
					}
					if (Shape->GetLeafGeometry() && Shape->GetLeafGeometry()->HasBoundingBox() && !Shape->GetWorldSpaceShapeBounds().Intersects(QueryBounds))
					{
						continue;
					}

					FSweepHit Hit;
					if (SweepAgainstImplicit(*Geometry, ParticlePose, Swept, SweptPose, Dir, Length, Hit))
					{
						Hit.Particle = Particle;
						Hit.ParticleShape = Shape.Get();
						Out.Add(Hit);
						bAnyHit = true;
					}
				}
				return true;
			}

			virtual bool Raycast(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance, FQueryFastData& CurData) override { return true; }
			virtual bool Sweep(const TSpatialVisitorData<FAccelerationStructureHandle>& Instance, FQueryFastData& CurData) override { return true; }

		private:
			const chrono::ChCollisionSystemChaos::Impl& Impl;
			const FImplicitObject& Swept;
			FTransform SweptPose;
			FVector Dir;
			double Length;
			FAABB3 QueryBounds;
			TArray<FSweepHit>& Out;
		};
	}

	bool SweepUnrealGeometry(const chrono::ChCollisionSystemChaos::Impl& Impl, const FImplicitObject& Swept, const FTransform& SweptPose,
	                         const FVector& Dir, double Length, TArray<FSweepHit>& OutHits)
	{
		if (!Impl.IsBound() || Length < 0.0)
		{
			return false;
		}
		const ISpatialAccelerationCollection<FAccelerationStructureHandle, FReal, 3>* Acceleration = Impl.Solver->GetInternalAccelerationStructure_Internal();
		if (!Acceleration)
		{
			return false;
		}

		const FAABB3 QueryBounds = SweptBounds(Swept, SweptPose, Dir, Length);
		FUnrealSweepVisitor Visitor(Impl, Swept, SweptPose, Dir, Length, QueryBounds, OutHits);
		Acceleration->Overlap(QueryBounds, Visitor);
		return Visitor.bAnyHit;
	}

	bool SweepChronoProxies(const chrono::ChCollisionSystemChaos::Impl& Impl, const FImplicitObject& Swept, const FTransform& SweptPose,
	                        const FVector& Dir, double Length, const chrono::ChCollisionModel* IgnoreModel, TArray<FSweepHit>& OutHits)
	{
		if (Length < 0.0)
		{
			return false;
		}
		const FAABB3 QueryBounds = SweptBounds(Swept, SweptPose, Dir, Length);

		bool bAnyHit = false;
		for (chrono::ChCollisionModelChaos* Model : Impl.Proxies)
		{
			if (IgnoreModel && Model->GetModel() == IgnoreModel)
			{
				continue;
			}
			const chrono::ChCollisionModelChaos::Data* Data = Model->GetData();
			if (!Data || !Data->Geometry.IsValid())
			{
				continue;
			}
			if (!Data->WorldBounds.IsEmpty() && !Data->WorldBounds.Intersects(QueryBounds))
			{
				continue;
			}

			for (const FChronoShapeSlot& Slot : Data->Geometry.Slots)
			{
				FSweepHit Hit;
				if (Slot.Root && SweepAgainstImplicit(*Slot.Root, Data->PoseUE, Swept, SweptPose, Dir, Length, Hit))
				{
					Hit.Model = Model;
					Hit.Shape = Slot.Shape;
					OutHits.Add(Hit);
					bAnyHit = true;
				}
			}
		}
		return bAnyHit;
	}
}
