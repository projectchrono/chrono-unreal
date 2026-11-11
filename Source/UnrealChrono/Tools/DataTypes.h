#pragma once

#include "CoreMinimal.h"
#include "ChronoTypes.h"
#include <Common.h>
#include "Components/ShapeComponent.h"
#include <PhysicsEngine/BoxElem.h>
#include <PhysicsEngine/SphereElem.h>
#include <PhysicsEngine/SphylElem.h>
#include <PhysicsEngine/ConvexElem.h>

using namespace chrono;

namespace PhysicsDataTypes
{
enum CollisionType { Moveable, Simulating, Static };
enum ShapeType {Box, Sphere, Capsule, Convex, TriMesh};
enum MeshType { StaticMesh, SkeletalMesh, PrimitiveMesh};

struct FShapeData
{
	ShapeType Type;
};

struct FBoxData : public FShapeData
{
	FKBoxElem* BoxElem;
};

struct FSphereData : public FShapeData
{
	FKSphereElem* SphereElem;
};

struct FCapsuleData : public FShapeData
{
	FKSphylElem* CapsuleElem;
};

struct FConvexData : public FShapeData
{
	FKConvexElem* ConvexElem;
};

struct FConvexHullShapeHolder : public FShapeData
{
	int HullIndex;
	FVector Scale;
};

struct FCachedDynamicShapeData
{
	FName ClassName; // class name for cache
	ChBody* Shape;
	CollisionType Collision;
	bool bIsCompound; // if true, this is a compound shape and so must be deleted
	float Mass;
	ChVector3d Inertia; // because we like to precalc this
};

struct FCollisionModel
{
	FCollisionModel() {}
	FCollisionModel(AActor* OwningActorInit, const ChColliderInfo& ChColliderInfoInit,  const FBox& AABBInit, UBodySetup * BodySetupInit, std::shared_ptr<ChBody> ShapeInit, const CollisionType CollisionTypeInit, const FShapeData& ShapeDataInit) :
					 ColliderInfo(ChColliderInfoInit), AABB(AABBInit), Shape(ShapeInit), CollideType(CollisionTypeInit), ShapeData(ShapeDataInit) {}	
	
	ChColliderInfo ColliderInfo;
	FBox AABB;	
	std::shared_ptr<ChBody> Shape;
	CollisionType CollideType = CollisionType::Static;
	FShapeData ShapeData;
};

struct FStaticMeshCollisionInfo : public FCollisionModel
{
	FStaticMeshCollisionInfo() {}
	FStaticMeshCollisionInfo(AActor* OwningActorInit, const ChColliderInfo& ChColliderInfoInit, const FBox& AABBInit, UBodySetup* BodySetupInit, std::shared_ptr<ChBody> ChBodyInit,
		const CollisionType CollisionTypeInit, const FShapeData& ShapeDataInit, UStaticMeshComponent* StaticMeshInit) :
		FCollisionModel(OwningActorInit, ChColliderInfoInit, AABBInit, BodySetupInit, ChBodyInit, CollisionTypeInit, ShapeDataInit),
		StaticMesh(StaticMeshInit) {}

	UStaticMeshComponent* StaticMesh;
};

struct FSkeletalMeshCollisionInfo : public FCollisionModel
{
	FSkeletalMeshCollisionInfo() {}
	FSkeletalMeshCollisionInfo(AActor* OwningActorInit, const ChColliderInfo& ChColliderInfoInit, const CollisionType CollisionTypeInit, const FShapeData& ShapeDataInit, const FBox& AABBInit,
		UBodySetup* BodySetupInit, std::shared_ptr<ChBody> ChBodyInit, USkeletalMeshComponent* SkeletalMeshInit) :
		FCollisionModel(OwningActorInit, ChColliderInfoInit, AABBInit, BodySetupInit, ChBodyInit, CollisionTypeInit, ShapeDataInit),
		SkeletalMesh(SkeletalMeshInit) {}

	USkeletalMeshComponent* SkeletalMesh;
};

struct FShapeCompCollisionInfo : public FCollisionModel
{
	FShapeCompCollisionInfo() {}
	FShapeCompCollisionInfo(AActor* OwningActorInit, const ChColliderInfo& ChColliderInfoInit, const CollisionType CollisionTypeInit, const FShapeData& ShapeDataInit, const FBox& AABBInit,
		UBodySetup* BodySetupInit, std::shared_ptr<ChBody> ChBodyInit, UShapeComponent* ShapeCompInit) :
		FCollisionModel(OwningActorInit, ChColliderInfoInit, AABBInit, BodySetupInit, ChBodyInit, CollisionTypeInit, ShapeDataInit),
		ShapeComp(ShapeCompInit) {}

	UShapeComponent* ShapeComp;
};

class CollisionModelFactory
{
public:
	std::shared_ptr<FCollisionModel> ConstructCollisionModel(MeshType NewType)
	{
		switch (NewType)
		{
		case MeshType::StaticMesh:
			return std::make_shared<FStaticMeshCollisionInfo>();// std::shared_ptr<FStaticMeshCollisionInfo>();
		case MeshType::SkeletalMesh:
			return std::make_shared<FSkeletalMeshCollisionInfo>();// std::shared_ptr<FSkeletalMeshCollisionInfo>();
		case MeshType::PrimitiveMesh:
			return std::make_shared<FShapeCompCollisionInfo>(); //std::shared_ptr<FShapeCompCollisionInfo>();
			default:
			return std::make_shared<FStaticMeshCollisionInfo>(); //std::shared_ptr<FStaticMeshCollisionInfo>();
		}
	}
};
}


