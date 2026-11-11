#include "UnrealCollisionExtractor.h"
#include "../ChronoCarPhysics/BulletStaticCollision.h"
#include "../UECarPhysics/PhysicsVehicle.h"
#include "StaticCollisionGrid.h"
#include "Conversions.h"
#include "VehiclePhysicalMaterial.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Components/ShapeComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/FileHelper.h"
#include "Serialization/MemoryReader.h"
#include <Kismet/GameplayStatics.h>

#include "LandscapeProxy.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "StaticMeshResources.h"

#if WITH_EDITOR
#include "Serialization/BufferArchive.h"
#endif

#if WITH_EDITOR
#pragma optimize("", off)
#endif

namespace
{
	//Contact method of the hosted system (FChronoSimulationParams::bSMC == false)
	constexpr chrono::ChContactMethod StaticCollisionContactMethod = chrono::ChContactMethod::NSC;
}

FUnrealCollisionExtractor::FUnrealCollisionExtractor(const FUnrealCollisionExtractorSettings& InSettings, std::vector<std::shared_ptr<FCollisionModel>>& InCollisionData)
	: Settings(InSettings)
	, CollisionData(InCollisionData)
{
}

void FUnrealCollisionExtractor::GatherUnrealCollision(bool bSkipStaticTag)
{
	TArray<AActor*> Actors;
	FName Tag = "ChronoTerrain";
	UGameplayStatics::GetAllActorsWithTag(Settings.World, Tag, Actors);
	CollisionType DesiredCollision = CollisionType::Static;

	for (auto Actor : Actors)
	{
		if (Actor)
		{			
			ExtractPhysicsGeometry(Actor, DesiredCollision, Tag);
		}
	}
	
	if (!bSkipStaticTag)
	{
		Tag = "ChronoStatic";
		UGameplayStatics::GetAllActorsWithTag(Settings.World, Tag, Actors);
		DesiredCollision = CollisionType::Static;

		for (auto Actor : Actors)
		{
			if (Actor)
			{
				ExtractPhysicsGeometry(Actor, DesiredCollision, Tag);
			}
		}
	}

	Tag = "ChronoMoveable";
	UGameplayStatics::GetAllActorsWithTag(Settings.World, Tag, Actors);
	DesiredCollision = CollisionType::Static;

	for (auto Actor : Actors)
	{
		if (Actor)
		{
			ExtractPhysicsGeometry(Actor, DesiredCollision, Tag);
		}
	}

	Tag = "ChronoSimulating";
	UGameplayStatics::GetAllActorsWithTag(Settings.World, Tag, Actors);
	DesiredCollision = CollisionType::Static;

	for (auto Actor : Actors)
	{
		if (Actor)
		{
			ExtractPhysicsGeometry(Actor, DesiredCollision, Tag);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Extracted %d collisions"), CollisionData.size());
}

//Buckets the gathered static collision into the streaming grid and attaches the cells that are already
//in range, so the very first physics step starts with a correct set of active colliders
void FUnrealCollisionExtractor::InitializeCollisionStreaming()
{
	FStaticCollisionGrid& Grid = FStaticCollisionGrid::Get();

	if (!Settings.EnableCollisionStreaming || CollisionData.empty())
	{
		Grid.Reset();
		return;
	}

	Grid.SetTuning(Settings.CollisionCellSizeMetres, Settings.CollisionActivationRadiusMetres, Settings.CollisionCellHysteresisMetres);
	Grid.SetEnableChronoVisual(Settings.EnableChronoVisual);
	Grid.Build(CollisionData);

	//Initial evaluation on begin play. The cells in range are queued here and attached to the system by the
	//ApplyAllPending command SetupBulletStaticCollision enqueues right after (physics thread)
	const ChVector3d StartLocation = Conversions::ToChronoVec(Settings.PhysicsVehicle->GetActorLocation());
	Grid.SetVehicleLocation(StartLocation.x(), StartLocation.y(), StartLocation.z());
	Grid.Evaluate();

	UE_LOG(LogTemp, Log, TEXT("InitializeCollisionStreaming: %d static colliders in %d cells, initial cells queued for the physics thread"),
		Grid.GetBodyCount(), Grid.GetCellCount());
}

FString FUnrealCollisionExtractor::GetStaticCollisionArchivePath(const FUnrealCollisionExtractorSettings& InSettings)
{
	return StaticCollisionArchive::GetArchiveFilePath(InSettings.MapDirectory, InSettings.StaticCollisionArchiveTag);
}

//Runtime counterpart of UNextGenCarsUtilities::SerializeCollisionActors. If the baked archive exists we
//rebuild the chrono/bullet bodies straight from it, which saves walking the level and re-cooking body setups
bool FUnrealCollisionExtractor::LoadSerializedCollision()
{
	if (Settings.StaticCollisionArchiveTag.IsEmpty()) return false;

	const FString FilePath = GetStaticCollisionArchivePath();
	if (!FPlatformFileManager::Get().GetPlatformFile().FileExists(*FilePath))
	{
		return false;
	}

	TArray<uint8> FileData;
	if (!FFileHelper::LoadFileToArray(FileData, *FilePath) || FileData.Num() == 0)
	{
		return false;
	}

	FStaticCollisionArchiveData ArchiveData;
	FMemoryReader Reader(FileData, true);
	Reader << ArchiveData;

	if (ArchiveData.Magic != StaticCollisionArchive::ArchiveMagic || ArchiveData.Version != StaticCollisionArchive::ArchiveVersion)
	{
		UE_LOG(LogTemp, Warning, TEXT("LoadSerializedCollision: %s is not a valid static collision archive, falling back to gathering the level"), *FilePath);
		return false;
	}

	CollisionModelFactory CollisionFactory;
	int32 BuiltShapes = 0;

	for (auto&& Record : ArchiveData.Shapes)
	{
		std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel((MeshType)Record.MeshKind);
		CollisionModel->ColliderInfo.Mass = Record.Collider.Mass;
		CollisionModel->ColliderInfo.Friction = Record.Collider.Friction;
		CollisionModel->ColliderInfo.Restitution = Record.Collider.Restitution;
		CollisionModel->ColliderInfo.PoissonRatio = Record.Collider.PoissonRatio;
		CollisionModel->ColliderInfo.Density = Record.Collider.Density;
		CollisionModel->ColliderInfo.RollingFriction = Record.Collider.RollingFriction;
		CollisionModel->ColliderInfo.SpinningFriction = Record.Collider.SpinningFriction;
		CollisionModel->ColliderInfo.YoungsModulus = Record.Collider.YoungsModulus;
		CollisionModel->ColliderInfo.KineticFriction = Record.Collider.KineticFriction;
		CollisionModel->ColliderInfo.StaticObject = Record.Collider.StaticObject;
		CollisionModel->ColliderInfo.Position = ChVector3d(Record.Collider.PositionX, Record.Collider.PositionY, Record.Collider.PositionZ);
		CollisionModel->ColliderInfo.Rotation = ChQuaterniond(Record.Collider.RotationE0, Record.Collider.RotationE1, Record.Collider.RotationE2, Record.Collider.RotationE3);
		CollisionModel->CollideType = (CollisionType)Record.CollisionKind;
		CollisionModel->AABB = Record.AABB;

		std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);

		switch ((EStaticCollisionShapeKind)Record.ShapeKind)
		{
		case EStaticCollisionShapeKind::Box:
			CollisionModel->Shape = FBulletStaticCollision::CreateBoxCollision(StaticCollisionContactMethod, ChVector3d(Record.ExtentX, Record.ExtentY, Record.ExtentZ), ColliderInfo, Settings.EnableChronoVisual);
			break;
		case EStaticCollisionShapeKind::Sphere:
			CollisionModel->Shape = FBulletStaticCollision::CreateSphereCollision(StaticCollisionContactMethod, Record.ExtentX, ColliderInfo, Settings.EnableChronoVisual);
			break;
		case EStaticCollisionShapeKind::Capsule:
			CollisionModel->Shape = FBulletStaticCollision::CreateCapsuleCollision(StaticCollisionContactMethod, Record.ExtentX, Record.ExtentY, ColliderInfo, Settings.EnableChronoVisual);
			break;
		case EStaticCollisionShapeKind::Convex:
		{
			std::vector<ChVector3d> Hull;
			Hull.reserve(Record.HullPoints.Num());
			for (auto&& Point : Record.HullPoints)
			{
				Hull.push_back(ChVector3d(Point.X, Point.Y, Point.Z));
			}
			CollisionModel->Shape = FBulletStaticCollision::CreateConvexCollision(StaticCollisionContactMethod, Hull, ColliderInfo, Settings.EnableChronoVisual);
			break;
		}
		case EStaticCollisionShapeKind::TriangleMesh:
		{
			if (Record.MeshVertices.Num() < 3 || Record.MeshIndices.Num() < 3)
			{
				continue;
			}
			auto TriMesh = chrono_types::make_shared<ChTriangleMeshConnected>();
			auto& Verts = TriMesh->GetCoordsVertices();
			auto& Indices = TriMesh->GetIndicesVertexes();
			Verts.reserve(Record.MeshVertices.Num());
			for (auto&& Point : Record.MeshVertices)
			{
				Verts.push_back(ChVector3d(Point.X, Point.Y, Point.Z));
			}
			Indices.reserve(Record.MeshIndices.Num() / 3);
			for (int32 i = 0; i + 2 < Record.MeshIndices.Num(); i += 3)
			{
				Indices.emplace_back(Record.MeshIndices[i], Record.MeshIndices[i + 1], Record.MeshIndices[i + 2]);
			}
			CollisionModel->Shape = FBulletStaticCollision::CreateTriangleMeshCollision(StaticCollisionContactMethod, TriMesh, ColliderInfo, Settings.EnableChronoVisual);
			break;
		}
		default:
			continue;
		}

		CollisionData.push_back(CollisionModel);
		++BuiltShapes;
	}

	UE_LOG(LogTemp, Log, TEXT("LoadSerializedCollision: restored %d static collision shapes from %s"), BuiltShapes, *FilePath);
	return BuiltShapes > 0;
}

#if WITH_EDITOR

//Editor only tool. Bakes all ChronoStatic tagged geometry into the
//StaticCollisionArchiveTag file so the runtime can load it back instead of walking the level
void FUnrealCollisionExtractor::SerializeCollisionActors()
{
	if (Settings.StaticCollisionArchiveTag.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("SerializeCollisionActors: StaticCollisionArchiveTag on %s is empty"), *Settings.OwnerName);
		return;
	}

	//Gather all actors with the actor tag 'ChronoStatic'
	TArray<AActor*> Actors;
	UGameplayStatics::GetAllActorsWithTag(Settings.World, FName("ChronoStatic"), Actors);

	TArray<FStaticCollisionShapeRecord> Records;
	int32 ActorCount = 0;
	for (auto Actor : Actors)
	{
		if (Actor)
		{
			++ActorCount;
			//Bucket each shape by its own world-space mesh transform (instance / patch position)
			GatherActorRecursive(Actor, Records);
		}
	}

	if (ActorCount == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("SerializeCollisionActors: no actors with tag 'ChronoStatic' found in the level"));
		return;
	}

	//Serialize everything we gathered into the tagged binary file
	FStaticCollisionArchiveData ArchiveData;
	ArchiveData.LevelName = Settings.World ? Settings.World->GetMapName() : FString();
	ArchiveData.Shapes = Records;

	FBufferArchive Buffer;
	Buffer << ArchiveData;

	const FString Directory = StaticCollisionArchive::GetArchiveDirectory(Settings.MapDirectory);
	const FString FilePath = GetStaticCollisionArchivePath();
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!PlatformFile.DirectoryExists(*Directory))
	{
		PlatformFile.CreateDirectoryTree(*Directory);
	}

	const bool bSaved = FFileHelper::SaveArrayToFile(Buffer, *FilePath);
	Buffer.FlushCache();
	Buffer.Empty();

	if (!bSaved)
	{
		UE_LOG(LogTemp, Error, TEXT("SerializeCollisionActors: failed to write %s"), *FilePath);
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("SerializeCollisionActors: wrote %d collision shapes from %d actors with tag 'ChronoStatic' to %s"),
		Records.Num(), ActorCount, *FilePath);
}

void FUnrealCollisionExtractor::GatherActorRecursive(AActor* Actor, TArray<FStaticCollisionShapeRecord>& OutRecords)
{
	if (!Actor) return;

	//Landscape actors expose one heightfield collision component per patch; bake each as a triangle mesh
	if (ALandscapeProxy* Landscape = Cast<ALandscapeProxy>(Actor))
	{	
		for (ULandscapeHeightfieldCollisionComponent* CollisionComp : Landscape->CollisionComponents)
		{
			if (CollisionComp)
			{
				GatherLandscapeCollisionComponent(CollisionComp, OutRecords);
			}
		}
	}

	TInlineComponentArray<UActorComponent*, 3000> Components;
	Actor->GetComponents(UStaticMeshComponent::StaticClass(), Components);
	for (auto&& Comp : Components)
	{
		if (Comp && Comp->ComponentHasTag("ChronoStatic"))
		{
			GatherStaticMeshComponent(Cast<UStaticMeshComponent>(Comp), OutRecords);
		}
	}

	//Recurse into attached child actors so whole prefab like hierarchies can be baked in one go
	TArray<AActor*> ChildActors;
	Actor->GetAttachedActors(ChildActors);
	for (auto&& Child : ChildActors)
	{
		if (Child && Child != Actor)
		{
			GatherActorRecursive(Child, OutRecords);
		}
	}
}

//Mirrors ExtractPhysicsGeometry, but instead of creating chrono bodies we only record
//the data needed to re-create them at runtime
void FUnrealCollisionExtractor::GatherStaticMeshComponent(UStaticMeshComponent* StaticMeshComp, TArray<FStaticCollisionShapeRecord>& OutRecords)
{
	if (!StaticMeshComp) return;

	UStaticMesh* Mesh = StaticMeshComp->GetStaticMesh();
	if (!Mesh) return;

	UBodySetup* BodySetup = Mesh->GetBodySetup();
	if (!BodySetup) return;

	UPhysicalMaterial* PhMat = StaticMeshComp->BodyInstance.GetSimplePhysicalMaterial();
	UVehiclePhysicalMaterial* VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
	if (!VehPhysMat)
	{
		UE_LOG(LogTemp, Warning, TEXT("SerializeCollisionActors: %s has no UVehiclePhysicalMaterial, skipping"), *StaticMeshComp->GetPathName());
		return;
	}

	const float Mass = BodySetup->CalculateMass();
	const FString SourceName = StaticMeshComp->GetPathName();

	auto MakeRecordsForTransform = [&](const FTransform& ComponentTx, UVehiclePhysicalMaterial* CurrentVehPhysMat)
	{
		if (!CurrentVehPhysMat) return;

		auto MakeRecord = [&](const EStaticCollisionShapeKind Kind, const ChronoTransform& ChTransform) -> FStaticCollisionShapeRecord
		{
			FStaticCollisionShapeRecord Record;
			Record.ShapeKind = (uint8)Kind;
			Record.MeshKind = (uint8)MeshType::StaticMesh;
			Record.CollisionKind = (uint8)CollisionType::Static;
			Record.SourceName = SourceName;
			Record.Collider.Mass = Mass;
			Record.Collider.Friction = CurrentVehPhysMat->Friction;
			Record.Collider.Restitution = CurrentVehPhysMat->Restitution;
			Record.Collider.PoissonRatio = CurrentVehPhysMat->PoissonRatio;
			Record.Collider.Density = CurrentVehPhysMat->Density;
			Record.Collider.RollingFriction = CurrentVehPhysMat->RollingFriction;
			Record.Collider.SpinningFriction = CurrentVehPhysMat->SpinningFriction;
			Record.Collider.YoungsModulus = CurrentVehPhysMat->YoungsModulus;
			Record.Collider.KineticFriction = CurrentVehPhysMat->StaticFriction;
			Record.Collider.StaticObject = true;
			//World-space mesh transform used both for chrono body placement and streaming grid buckets
			Record.Collider.PositionX = ChTransform.Location.x();
			Record.Collider.PositionY = ChTransform.Location.y();
			Record.Collider.PositionZ = ChTransform.Location.z();
			Record.Collider.RotationE0 = ChTransform.Rotation.e0();
			Record.Collider.RotationE1 = ChTransform.Rotation.e1();
			Record.Collider.RotationE2 = ChTransform.Rotation.e2();
			Record.Collider.RotationE3 = ChTransform.Rotation.e3();
			return Record;
		};

		const bool bUseComplexCollision = (BodySetup->GetCollisionTraceFlag() == ECollisionTraceFlag::CTF_UseComplexAsSimple);
		if (bUseComplexCollision || BodySetup->AggGeom.GetElementCount() == 0)
		{
			TArray<FVector> MeshVertices;
			TArray<int32> MeshIndices;
			FBox AABB(ForceInit);
			ChronoTransform ChTransform;
			if (BuildStaticMeshTriangleMesh(Mesh, ComponentTx, MeshVertices, MeshIndices, AABB, ChTransform))
			{
				FStaticCollisionShapeRecord Record = MakeRecord(EStaticCollisionShapeKind::TriangleMesh, ChTransform);
				Record.MeshVertices = MoveTemp(MeshVertices);
				Record.MeshIndices = MoveTemp(MeshIndices);
				Record.AABB = AABB;
				OutRecords.Add(Record);
				return;
			}
		}

		for (auto&& Box : BodySetup->AggGeom.BoxElems)
		{
			const FTransform ShapeXform = Box.GetTransform();
			const FTransform FinalTransformForm = ShapeXform * ComponentTx;
			const ChronoTransform ChTransform = Conversions::ToChronoTransform(FinalTransformForm);
			const FVector Dimensions = FVector(Box.X, Box.Y, Box.Z) * 0.01 * FinalTransformForm.GetScale3D().GetAbs();

			FStaticCollisionShapeRecord Record = MakeRecord(EStaticCollisionShapeKind::Box, ChTransform);
			Record.ExtentX = Dimensions.X;
			Record.ExtentY = Dimensions.Y;
			Record.ExtentZ = Dimensions.Z;
			Record.AABB = Box.CalcAABB(FinalTransformForm, 1.f);
			OutRecords.Add(Record);
		}

		for (auto&& Sphere : BodySetup->AggGeom.SphereElems)
		{
			const FTransform ShapeXform(FRotator::ZeroRotator, Sphere.Center);
			const FTransform XForm = ShapeXform * ComponentTx;
			const ChronoTransform ChTransform = Conversions::ToChronoTransform(XForm);
			const FVector Scale = XForm.GetScale3D().GetAbs();

			FStaticCollisionShapeRecord Record = MakeRecord(EStaticCollisionShapeKind::Sphere, ChTransform);
			Record.ExtentX = Conversions::ToChronoSize(Sphere.Radius * FMath::Max3(Scale.X, Scale.Y, Scale.Z));
			Record.AABB = Sphere.CalcAABB(XForm, 1.f);
			OutRecords.Add(Record);
		}

		for (auto&& Capsule : BodySetup->AggGeom.SphylElems)
		{
			const FTransform ShapeXForm(Capsule.Rotation, Capsule.Center);
			const FTransform FinalTransformForm = ShapeXForm * ComponentTx;
			const ChronoTransform ChTransform = Conversions::ToChronoTransform(FinalTransformForm);
			const FVector Scale = FinalTransformForm.GetScale3D().GetAbs();

			FStaticCollisionShapeRecord Record = MakeRecord(EStaticCollisionShapeKind::Capsule, ChTransform);
			//Chrono capsules are Y up, unreal capsules are Z up, so bake in the same correction the runtime path uses
			const chrono::ChQuaterniond Corrected = ChTransform.Rotation * chrono::QuatFromAngleAxis(PI, Conversions::ToChronoNormal(FVector(1, 0, 0)));
			Record.Collider.RotationE0 = Corrected.e0();
			Record.Collider.RotationE1 = Corrected.e1();
			Record.Collider.RotationE2 = Corrected.e2();
			Record.Collider.RotationE3 = Corrected.e3();
			Record.ExtentX = Conversions::ToChronoSize(Capsule.GetScaledRadius(Scale));
			Record.ExtentY = Conversions::ToChronoSize(Capsule.GetScaledCylinderLength(Scale) / 2.0f);
			Record.AABB = Capsule.CalcAABB(FinalTransformForm, 1.f);
			OutRecords.Add(Record);
		}

		for (int i = 0; i < BodySetup->AggGeom.ConvexElems.Num(); ++i)
		{
			const FTransform ShapeXForm = BodySetup->AggGeom.ConvexElems[i].GetTransform();
			const FTransform FinalTransformForm = ShapeXForm * ComponentTx;
			const ChronoTransform ChTransform = Conversions::ToChronoTransform(FinalTransformForm);
			const FVector Scale = FinalTransformForm.GetScale3D();

			FStaticCollisionShapeRecord Record = MakeRecord(EStaticCollisionShapeKind::Convex, ChTransform);
			Record.AABB = BodySetup->AggGeom.ConvexElems[i].CalcAABB(FinalTransformForm, FVector::OneVector);
			Record.HullPoints.Reserve(BodySetup->AggGeom.ConvexElems[i].VertexData.Num());
			for (int x = 0; x < BodySetup->AggGeom.ConvexElems[i].VertexData.Num(); ++x)
			{
				const FVector ScaledPoint = BodySetup->AggGeom.ConvexElems[i].VertexData[x] * Scale;
				const chrono::ChVector3d Point = Conversions::ToChronoVec(ScaledPoint);
				Record.HullPoints.Add(FVector(Point.x(), Point.y(), Point.z()));
			}
			OutRecords.Add(Record);
		}
	};

	if (UInstancedStaticMeshComponent* InstancedComp = Cast<UInstancedStaticMeshComponent>(StaticMeshComp))
	{
		//Each instance is its own world-space mesh for grid bucketing and body placement
		const int32 NumInstances = InstancedComp->GetInstanceCount();
		const TArray<FBodyInstance*>& InstanceBodies = InstancedComp->GetInstanceBodies();
		for (int32 i = 0; i < NumInstances; ++i)
		{
			FTransform InstanceTx;
			if (InstancedComp->GetInstanceTransform(i, InstanceTx, true))
			{
				UVehiclePhysicalMaterial* InstanceVehPhysMat = VehPhysMat;
				if (InstanceBodies.IsValidIndex(i) && InstanceBodies[i])
				{
					if (InstanceBodies[i]->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
					{
						continue;
					}
					if (UPhysicalMaterial* BodyPhMat = InstanceBodies[i]->GetSimplePhysicalMaterial())
					{
						if (UVehiclePhysicalMaterial* BodyVehPhysMat = Cast<UVehiclePhysicalMaterial>(BodyPhMat))
						{
							InstanceVehPhysMat = BodyVehPhysMat;
						}
					}
				}
				MakeRecordsForTransform(InstanceTx, InstanceVehPhysMat);
			}
		}
	}
	else
	{
		MakeRecordsForTransform(StaticMeshComp->GetComponentTransform(), VehPhysMat);
	}
}

void FUnrealCollisionExtractor::GatherLandscapeCollisionComponent(ULandscapeHeightfieldCollisionComponent* CollisionComp, TArray<FStaticCollisionShapeRecord>& OutRecords)
{
	if (!CollisionComp) return;

	TArray<FVector> MeshVertices;
	TArray<int32> MeshIndices;
	FBox AABB(ForceInit);
	ChronoTransform ChTransform;
	if (!BuildLandscapePatchTriangleMesh(CollisionComp, MeshVertices, MeshIndices, AABB, ChTransform))
	{
		return;
	}

	UVehiclePhysicalMaterial* VehPhysMat = nullptr;
	if (UPhysicalMaterial* PhMat = CollisionComp->BodyInstance.GetSimplePhysicalMaterial())
	{
		VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
	}

	FStaticCollisionShapeRecord Record;
	Record.ShapeKind = (uint8)EStaticCollisionShapeKind::TriangleMesh;
	Record.MeshKind = (uint8)MeshType::StaticMesh;
	Record.CollisionKind = (uint8)CollisionType::Static;
	Record.SourceName = CollisionComp->GetPathName();
	Record.Collider.Mass = 0.f;
	Record.Collider.Friction = VehPhysMat ? VehPhysMat->Friction : 0.9f;
	Record.Collider.Restitution = VehPhysMat ? VehPhysMat->Restitution : 0.f;
	Record.Collider.PoissonRatio = VehPhysMat ? VehPhysMat->PoissonRatio : 0.3f;
	Record.Collider.Density = VehPhysMat ? VehPhysMat->Density : 1000.f;
	Record.Collider.RollingFriction = VehPhysMat ? VehPhysMat->RollingFriction : 0.02f;
	Record.Collider.SpinningFriction = VehPhysMat ? VehPhysMat->SpinningFriction : 0.01f;
	Record.Collider.YoungsModulus = VehPhysMat ? VehPhysMat->YoungsModulus : 1e5f;
	Record.Collider.KineticFriction = VehPhysMat ? VehPhysMat->StaticFriction : 0.9f;
	Record.Collider.StaticObject = true;
	Record.Collider.PositionX = ChTransform.Location.x();
	Record.Collider.PositionY = ChTransform.Location.y();
	Record.Collider.PositionZ = ChTransform.Location.z();
	Record.Collider.RotationE0 = ChTransform.Rotation.e0();
	Record.Collider.RotationE1 = ChTransform.Rotation.e1();
	Record.Collider.RotationE2 = ChTransform.Rotation.e2();
	Record.Collider.RotationE3 = ChTransform.Rotation.e3();
	Record.MeshVertices = MoveTemp(MeshVertices);
	Record.MeshIndices = MoveTemp(MeshIndices);
	Record.AABB = AABB;
	OutRecords.Add(Record);
}

#endif //WITH_EDITOR

void FUnrealCollisionExtractor::ExtractPhysicsGeometry(AActor* Actor, CollisionType Collision, const FName& Tag)
{
	if (!Actor) return;

	//ChronoStatic landscapes: one triangle mesh body per heightfield collision patch, bucketed by patch location
	if (ALandscapeProxy* Landscape = Cast<ALandscapeProxy>(Actor))
	{
		for (ULandscapeHeightfieldCollisionComponent* CollisionComp : Landscape->CollisionComponents)
		{
			if (CollisionComp)
			{
				ExtractLandscapeCollision(CollisionComp, Collision);
			}
		}
	}

	TInlineComponentArray<UActorComponent*, 3000> Components;

	//Collisions from static meshes
	Actor->GetComponents(UStaticMeshComponent::StaticClass(), Components);
	for (auto&& Comp : Components)
	{
		if (Comp->ComponentHasTag(Tag))
		{
			ExtractPhysicsGeometry(Cast<UStaticMeshComponent>(Comp),  Collision);
		}
	}

	//Collisions from skeletal physics bodies
	TInlineComponentArray<UActorComponent*, 3000> SKComponents;
	Actor->GetComponents(USkeletalMeshComponent::StaticClass(), SKComponents);
	for (auto&& Comp : SKComponents)
	{
		if (Comp->ComponentHasTag(Tag))
		{
			ExtractPhysicsGeometry(Cast<USkeletalMeshComponent>(Comp), Collision);
		}
	}

	//Collisions from separate collision components
	Actor->GetComponents(UShapeComponent::StaticClass(), Components);
	for (auto&& Comp : Components)
	{
		if (Comp->ComponentHasTag(Tag))
		{
			ExtractPhysicsGeometry(Cast<UShapeComponent>(Comp),  Collision);
		}
	}		
	UE_LOG(LogTemp, Log, TEXT("Extracted %d collisions"), CollisionData.size());
}


//Instanced static meshes
void FUnrealCollisionExtractor::ExtractPhysicsGeometry(UInstancedStaticMeshComponent* InstancedStaticMeshComp, CollisionType Collision)
{
	if (!InstancedStaticMeshComp) return;

	UStaticMesh* Mesh = InstancedStaticMeshComp->GetStaticMesh();
	if (!Mesh) return;

	UBodySetup* BodySetup = Mesh->GetBodySetup();
	if (!BodySetup) return;

	UPhysicalMaterial* PhMat = InstancedStaticMeshComp->BodyInstance.GetSimplePhysicalMaterial();
	UVehiclePhysicalMaterial* VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
	if (!VehPhysMat) return;

	const float Mass = BodySetup->CalculateMass();
	const int32 NumInstances = InstancedStaticMeshComp->GetInstanceCount();
	const TArray<FBodyInstance*>& InstanceBodies = InstancedStaticMeshComp->GetInstanceBodies();

	for (int32 i = 0; i < NumInstances; ++i)
	{
		FTransform InstanceTx;
		if (!InstancedStaticMeshComp->GetInstanceTransform(i, InstanceTx, true))
		{
			continue;
		}

		UVehiclePhysicalMaterial* InstanceVehPhysMat = VehPhysMat;
		if (InstanceBodies.IsValidIndex(i) && InstanceBodies[i])
		{
			if (InstanceBodies[i]->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
			{
				continue;
			}
			if (UPhysicalMaterial* BodyPhMat = InstanceBodies[i]->GetSimplePhysicalMaterial())
			{
				if (UVehiclePhysicalMaterial* BodyVehPhysMat = Cast<UVehiclePhysicalMaterial>(BodyPhMat))
				{
					InstanceVehPhysMat = BodyVehPhysMat;
				}
			}
		}

		ExtractPhysicsGeometry(InstancedStaticMeshComp, InstanceTx, Collision, BodySetup, InstanceVehPhysMat, Mass, MeshType::StaticMesh);
	}
}

//Skeletal meshes
void FUnrealCollisionExtractor::ExtractPhysicsGeometry(USkeletalMeshComponent* SKMeshComp, CollisionType Collision)
{
	USkeletalMesh* Mesh = SKMeshComp->GetSkeletalMeshAsset();

	if (!Mesh) return;

	UPhysicsAsset* PhysicsAsset = SKMeshComp->GetPhysicsAsset();
	if (!PhysicsAsset) return;

	for (auto Body : SKMeshComp->Bodies)
	{
		if (Body->GetCollisionEnabled() != ECollisionEnabled::NoCollision)
		{
			UBodySetup* BodySetup = Body->GetBodySetup();
			if (BodySetup)
			{				
				UPhysicalMaterial* PhMat = Body->GetSimplePhysicalMaterial();
				UVehiclePhysicalMaterial * VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
				check(VehPhysMat);	
				ExtractPhysicsGeometry(SKMeshComp, Body->GetUnrealWorldTransform(), Collision, BodySetup, VehPhysMat, BodySetup->CalculateMass(), MeshType::SkeletalMesh);
			}
		}
	}
}

//Static meshes
void FUnrealCollisionExtractor::ExtractPhysicsGeometry(UStaticMeshComponent* StaticMeshComp,  CollisionType Collision)
{
	if (UInstancedStaticMeshComponent* InstancedComp = Cast<UInstancedStaticMeshComponent>(StaticMeshComp))
	{
		ExtractPhysicsGeometry(InstancedComp, Collision);
		return;
	}

	UStaticMesh* Mesh = StaticMeshComp->GetStaticMesh();
	if (!Mesh) return;	

	UPhysicalMaterial * PhMat = StaticMeshComp->BodyInstance.GetSimplePhysicalMaterial();	
	UVehiclePhysicalMaterial * VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
	if(!VehPhysMat) return;
	
	ExtractPhysicsGeometry(StaticMeshComp, StaticMeshComp->GetComponentTransform(), Collision, Mesh->GetBodySetup(), VehPhysMat, Mesh->GetBodySetup()->CalculateMass(), MeshType::StaticMesh);
}

//Shape meshes (Sphere collision, capsule collision and other simple collision)
void FUnrealCollisionExtractor::ExtractPhysicsGeometry(UShapeComponent* ShapeComp, CollisionType Collision)
{
	UPhysicalMaterial* PhMat = ShapeComp->BodyInstance.GetSimplePhysicalMaterial();
	UVehiclePhysicalMaterial * VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
	check(VehPhysMat);	
	ExtractPhysicsGeometry(ShapeComp, ShapeComp->GetComponentTransform(), Collision, ShapeComp->ShapeBodySetup, VehPhysMat, ShapeComp->GetMass(), MeshType::PrimitiveMesh);
}

void FUnrealCollisionExtractor::ExtractPhysicsGeometry(UPrimitiveComponent * Component, CollisionType Collision, UBodySetup* BodySetup, UVehiclePhysicalMaterial * PhysMaterial, const float Mass, MeshType Mesh)
{
	ExtractPhysicsGeometry(Component, Component->GetComponentTransform(), Collision, BodySetup, PhysMaterial, Mass, Mesh);
}

//We re-use the same physics extract functions for all types of collisions. the AABB calculations have not yet been verified and are probably wrong
void FUnrealCollisionExtractor::ExtractPhysicsGeometry(UPrimitiveComponent * Component, const FTransform& ComponentTx, CollisionType Collision, UBodySetup* BodySetup, UVehiclePhysicalMaterial * PhysMaterial, const float Mass, MeshType Mesh)
{	
	if (!BodySetup || !PhysMaterial) return;

	CollisionModelFactory CollisionFactory;
	//ComponentTx is already the world-space mesh transform (instance world transform for ISMs).
	//ColliderInfo.Position/Rotation is both the chrono body pose and the streaming grid bucket key.
	// Iterate over the simple collision shapes
	for (auto&& Box : BodySetup->AggGeom.BoxElems)
	{					
		const FTransform ShapeXform = Box.GetTransform();
		const FTransform FinalTransformForm = ShapeXform * ComponentTx;		
		ChronoTransform ChTransform = Conversions::ToChronoTransform(FinalTransformForm);		
		const FVector Dimensions = FVector(Box.X, Box.Y, Box.Z) * 0.01 * FinalTransformForm.GetScale3D().GetAbs(); // convert to meters and then divide by two as it expects the half height			
		std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel(Mesh);
		CollisionModel->ColliderInfo.Friction = PhysMaterial->Friction;
		CollisionModel->ColliderInfo.Restitution = PhysMaterial->Restitution;
		CollisionModel->ColliderInfo.PoissonRatio = PhysMaterial->PoissonRatio;		
		CollisionModel->ColliderInfo.Density = PhysMaterial->Density;
		CollisionModel->ColliderInfo.RollingFriction = PhysMaterial->RollingFriction;
		CollisionModel->ColliderInfo.SpinningFriction = PhysMaterial->SpinningFriction;		
		CollisionModel->ColliderInfo.YoungsModulus = PhysMaterial->YoungsModulus;
		CollisionModel->ColliderInfo.KineticFriction = PhysMaterial->StaticFriction;
		CollisionModel->ColliderInfo.Mass = Mass;
		CollisionModel->ColliderInfo.Position = ChTransform.Location;
		CollisionModel->ColliderInfo.Rotation = ChTransform.Rotation;
		CollisionModel->ColliderInfo.StaticObject = Collision == CollisionType::Static;
		CollisionModel->AABB = Box.CalcAABB(FinalTransformForm, 1.f);
		//CollisionModel->BodySetup = BodySetup;		
		std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);
		CollisionModel->Shape = FBulletStaticCollision::CreateBoxCollision(StaticCollisionContactMethod, ChVector3d(Dimensions.X, Dimensions.Y, Dimensions.Z), ColliderInfo, Settings.EnableChronoVisual);		
		
		CollisionData.push_back(CollisionModel);
	}
	for (auto&& Sphere : BodySetup->AggGeom.SphereElems)
	{			
		const FTransform ShapeXform(FRotator::ZeroRotator, Sphere.Center);		
		const FTransform XForm = ShapeXform * ComponentTx;
		const ChronoTransform ChTransform = Conversions::ToChronoTransform(XForm);
		std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel(Mesh);
		const FVector Scale = XForm.GetScale3D().GetAbs();		
		const float Radius = Conversions::ToChronoSize(Sphere.Radius * FMath::Max3(Scale.X, Scale.Y, Scale.Z));
		CollisionModel->ColliderInfo.Friction = PhysMaterial->Friction;
		CollisionModel->ColliderInfo.Restitution = PhysMaterial->Restitution;
		CollisionModel->ColliderInfo.PoissonRatio = PhysMaterial->PoissonRatio;		
		CollisionModel->ColliderInfo.Density = PhysMaterial->Density;
		CollisionModel->ColliderInfo.RollingFriction = PhysMaterial->RollingFriction;
		CollisionModel->ColliderInfo.SpinningFriction = PhysMaterial->SpinningFriction;		
		CollisionModel->ColliderInfo.YoungsModulus = PhysMaterial->YoungsModulus;
		CollisionModel->ColliderInfo.KineticFriction = PhysMaterial->StaticFriction;
		CollisionModel->ColliderInfo.Mass = Mass;
		CollisionModel->ColliderInfo.Position = ChTransform.Location;
		CollisionModel->ColliderInfo.Rotation = ChTransform.Rotation;
		CollisionModel->ColliderInfo.StaticObject = Collision == CollisionType::Static;
		CollisionModel->AABB = Sphere.CalcAABB(XForm, 1.f);
		std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);
		CollisionModel->Shape = FBulletStaticCollision::CreateSphereCollision(StaticCollisionContactMethod, Radius, ColliderInfo, Settings.EnableChronoVisual);//Ch_CreateSphereCollision(Sphere.Radius, ColliderInfo);
		CollisionData.push_back(CollisionModel);
	}
	for (auto& Capsule : BodySetup->AggGeom.SphylElems)
	{		
		const FTransform ShapeXForm(Capsule.Rotation, Capsule.Center);
		const FTransform FinalTransformForm = ShapeXForm * ComponentTx;
		const ChronoTransform ChTransform = Conversions::ToChronoTransform(FinalTransformForm);
		const FVector Scale = FinalTransformForm.GetScale3D().GetAbs();
		const float ScaledRadius = Conversions::ToChronoSize(Capsule.GetScaledRadius(Scale));
		const float ScaledHalfLength = Conversions::ToChronoSize(Capsule.GetScaledCylinderLength(Scale) / 2.0f);

		std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel(Mesh);
		CollisionModel->ColliderInfo.Friction = PhysMaterial->Friction;
		CollisionModel->ColliderInfo.Restitution = PhysMaterial->Restitution;
		CollisionModel->ColliderInfo.PoissonRatio = PhysMaterial->PoissonRatio;		
		CollisionModel->ColliderInfo.Density = PhysMaterial->Density;
		CollisionModel->ColliderInfo.RollingFriction = PhysMaterial->RollingFriction;
		CollisionModel->ColliderInfo.SpinningFriction = PhysMaterial->SpinningFriction;		
		CollisionModel->ColliderInfo.YoungsModulus = PhysMaterial->YoungsModulus;
		CollisionModel->ColliderInfo.KineticFriction = PhysMaterial->StaticFriction;
		CollisionModel->ColliderInfo.Mass = Mass;
		CollisionModel->ColliderInfo.Position = ChTransform.Location;
		const double angle_rad = PI;//PI / 2.0;  // 90 degrees in radians
		const auto axis = Conversions::ToChronoNormal(FVector(1,0,0));				
		CollisionModel->ColliderInfo.Rotation = ChTransform.Rotation * chrono::QuatFromAngleAxis(angle_rad, axis);
		CollisionModel->ColliderInfo.StaticObject = Collision == CollisionType::Static;
		CollisionModel->AABB = Capsule.CalcAABB(FinalTransformForm, 1.f);
		std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);

		CollisionModel->Shape = FBulletStaticCollision::CreateCapsuleCollision(StaticCollisionContactMethod, ScaledRadius,
									ScaledHalfLength, ColliderInfo, Settings.EnableChronoVisual);
		CollisionData.push_back(CollisionModel);
	}	
	for (int i = 0; i < BodySetup->AggGeom.ConvexElems.Num(); ++i)
	{
		const FTransform ShapeXForm = BodySetup->AggGeom.ConvexElems[i].GetTransform();
		const FTransform FinalTransformForm = ShapeXForm * ComponentTx;
		const ChronoTransform ChTransform = Conversions::ToChronoTransform(FinalTransformForm);
		const FVector Scale = FinalTransformForm.GetScale3D();
		std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel(Mesh);
		CollisionModel->ColliderInfo.Friction = PhysMaterial->Friction;
		CollisionModel->ColliderInfo.Restitution = PhysMaterial->Restitution;
		CollisionModel->ColliderInfo.PoissonRatio = PhysMaterial->PoissonRatio;		
		CollisionModel->ColliderInfo.Density = PhysMaterial->Density;
		CollisionModel->ColliderInfo.RollingFriction = PhysMaterial->RollingFriction;
		CollisionModel->ColliderInfo.SpinningFriction = PhysMaterial->SpinningFriction;		
		CollisionModel->ColliderInfo.YoungsModulus = PhysMaterial->YoungsModulus;
		CollisionModel->ColliderInfo.KineticFriction = PhysMaterial->StaticFriction;
		CollisionModel->ColliderInfo.Mass = Mass;
		CollisionModel->ColliderInfo.Position = ChTransform.Location;
		CollisionModel->ColliderInfo.Rotation = ChTransform.Rotation;
		CollisionModel->ColliderInfo.StaticObject = Collision == CollisionType::Static;
		CollisionModel->AABB = BodySetup->AggGeom.ConvexElems[i].CalcAABB(FinalTransformForm, FVector::OneVector);
		std::vector<chrono::ChVector3d> Hull;
		for (int x = 0; x < BodySetup->AggGeom.ConvexElems[i].VertexData.Num(); ++x)
		{
			const FVector ScaledPoint = BodySetup->AggGeom.ConvexElems[i].VertexData[x] * Scale;		
			Hull.push_back(Conversions::ToChronoVec(ScaledPoint));
		}
		std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);
		CollisionModel->Shape = FBulletStaticCollision::CreateConvexCollision(StaticCollisionContactMethod, Hull, ColliderInfo, Settings.EnableChronoVisual); 
		CollisionData.push_back(CollisionModel);
	}
}

bool FUnrealCollisionExtractor::BuildLandscapePatchTriangleMesh(
	ULandscapeHeightfieldCollisionComponent* CollisionComp,
	TArray<FVector>& OutVertices,
	TArray<int32>& OutIndices,
	FBox& OutAABB,
	ChronoTransform& OutChTransform) const
{
	OutVertices.Reset();
	OutIndices.Reset();
	OutAABB = FBox(ForceInit);

	if (!CollisionComp) return false;

	// GetCollisionSampleInfo() is not LANDSCAPE_API-exported (MinimalAPI class); mirror its formula from public UPROPERTY data
	const int32 NumVerts = CollisionComp->CollisionSizeQuads + 1;
	if (NumVerts < 2) return false;

	TArray<float> WorldHeights;
	WorldHeights.SetNumUninitialized(NumVerts * NumVerts);
	if (!CollisionComp->FillHeightTile(WorldHeights, 0, NumVerts))
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildLandscapePatchTriangleMesh: FillHeightTile failed for %s"), *CollisionComp->GetPathName());
		return false;
	}
	const FTransform CompTx = CollisionComp->GetComponentTransform();
	
	const float Step = CollisionComp->CollisionScale;	

	OutVertices.Reserve(NumVerts * NumVerts);
	for (int32 Y = 0; Y < NumVerts; ++Y)
	{
		for (int32 X = 0; X < NumVerts; ++X)
		{
			const FVector LocalXY(X * Step, Y * Step, 0.f);
			const FVector WorldXY = CompTx.TransformPosition(LocalXY);
			const FVector WorldPos = FVector(WorldXY.X, WorldXY.Y, WorldHeights[Y * NumVerts + X]);				
			OutVertices.Add(FVector(WorldPos.X * UE2CH_SCALE, -WorldPos.Y * UE2CH_SCALE, WorldPos.Z * UE2CH_SCALE));		
			OutAABB += WorldPos;			
		}
	}

	OutChTransform = ChronoTransform(Conversions::ToChronoVec(OutAABB.GetCenter()), chrono::ChQuaterniond(1, 0, 0, 0));

	const int32 NumQuads = NumVerts - 1;
	OutIndices.Reserve(NumQuads * NumQuads * 6);
	const bool bHasQuadFlags = CollisionComp->CollisionQuadFlags.Num() >= NumQuads * NumQuads;

	for (int32 Y = 0; Y < NumQuads; ++Y)
	{
		for (int32 X = 0; X < NumQuads; ++X)
		{
			if (bHasQuadFlags)
			{
				const uint8 Flags = CollisionComp->CollisionQuadFlags[Y * NumQuads + X];
				if (Flags & ULandscapeHeightfieldCollisionComponent::QF_NoCollision)
				{
					continue;
				}

				const int32 I00 = Y * NumVerts + X;
				const int32 I10 = I00 + 1;
				const int32 I01 = I00 + NumVerts;
				const int32 I11 = I01 + 1;

				if (Flags & ULandscapeHeightfieldCollisionComponent::QF_EdgeTurned)
				{
					//Turned diagonal: (0,0)-(1,1)
					OutIndices.Add(I00); OutIndices.Add(I11); OutIndices.Add(I01);
					OutIndices.Add(I00); OutIndices.Add(I10); OutIndices.Add(I11);
				}
				else
				{
					OutIndices.Add(I00); OutIndices.Add(I10); OutIndices.Add(I01);
					OutIndices.Add(I10); OutIndices.Add(I11); OutIndices.Add(I01);
				}
			}
			else
			{
				const int32 I00 = Y * NumVerts + X;
				const int32 I10 = I00 + 1;
				const int32 I01 = I00 + NumVerts;
				const int32 I11 = I01 + 1;
				OutIndices.Add(I00); OutIndices.Add(I10); OutIndices.Add(I01);
				OutIndices.Add(I10); OutIndices.Add(I11); OutIndices.Add(I01);
			}
		}
	}

	return OutVertices.Num() >= 3 && OutIndices.Num() >= 3;
}

void FUnrealCollisionExtractor::ExtractLandscapeCollision(ULandscapeHeightfieldCollisionComponent* CollisionComp, CollisionType Collision)
{
	if (!CollisionComp) return;
	if (CollisionComp->GetCollisionEnabled() == ECollisionEnabled::NoCollision) return;

	TArray<FVector> MeshVertices;
	TArray<int32> MeshIndices;
	FBox AABB(ForceInit);
	ChronoTransform ChTransform;
	if (!BuildLandscapePatchTriangleMesh(CollisionComp, MeshVertices, MeshIndices, AABB, ChTransform))
	{
		return;
	}

	UVehiclePhysicalMaterial* VehPhysMat = nullptr;
	if (UPhysicalMaterial* PhMat = CollisionComp->BodyInstance.GetSimplePhysicalMaterial())
	{
		VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);
	}

	CollisionModelFactory CollisionFactory;
	std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel(MeshType::StaticMesh);
	CollisionModel->ColliderInfo.Friction = VehPhysMat ? VehPhysMat->Friction : 0.9f;
	CollisionModel->ColliderInfo.Restitution = VehPhysMat ? VehPhysMat->Restitution : 0.f;
	CollisionModel->ColliderInfo.PoissonRatio = VehPhysMat ? VehPhysMat->PoissonRatio : 0.3f;
	CollisionModel->ColliderInfo.Density = VehPhysMat ? VehPhysMat->Density : 1000.f;
	CollisionModel->ColliderInfo.RollingFriction = VehPhysMat ? VehPhysMat->RollingFriction : 0.02f;
	CollisionModel->ColliderInfo.SpinningFriction = VehPhysMat ? VehPhysMat->SpinningFriction : 0.01f;
	CollisionModel->ColliderInfo.YoungsModulus = VehPhysMat ? VehPhysMat->YoungsModulus : 1e5f;
	CollisionModel->ColliderInfo.KineticFriction = VehPhysMat ? VehPhysMat->StaticFriction : 0.9f;
	CollisionModel->ColliderInfo.Mass = 0.f;
	CollisionModel->ColliderInfo.Position = ChTransform.Location;
	CollisionModel->ColliderInfo.Rotation = ChTransform.Rotation;
	CollisionModel->ColliderInfo.StaticObject = Collision == CollisionType::Static;
	CollisionModel->AABB = AABB;
	CollisionModel->CollideType = Collision;

	auto TriMesh = chrono_types::make_shared<ChTriangleMeshConnected>();
	auto& Verts = TriMesh->GetCoordsVertices();
	auto& Indices = TriMesh->GetIndicesVertexes();
	Verts.reserve(MeshVertices.Num());
	for (const FVector& V : MeshVertices)
	{
		Verts.emplace_back(V.X, V.Y, V.Z);
	}
	Indices.reserve(MeshIndices.Num() / 3);
	for (int32 i = 0; i + 2 < MeshIndices.Num(); i += 3)
	{
		Indices.emplace_back(MeshIndices[i], MeshIndices[i + 1], MeshIndices[i + 2]);
	}

	std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);
	CollisionModel->Shape = FBulletStaticCollision::CreateTriangleMeshCollision(StaticCollisionContactMethod, TriMesh, ColliderInfo, Settings.EnableChronoVisual);
	CollisionData.push_back(CollisionModel);
}

bool FUnrealCollisionExtractor::BuildStaticMeshTriangleMesh(
	UStaticMesh* StaticMesh,
	const FTransform& Transform,
	TArray<FVector>& OutVertices,
	TArray<int32>& OutIndices,
	FBox& OutAABB,
	ChronoTransform& OutChTransform) const
{
	OutVertices.Reset();
	OutIndices.Reset();
	OutAABB = FBox(ForceInit);

	if (!StaticMesh) return false;

	// First try GetPhysicsTriMeshData (uses collision LOD / complex collision data)
	FTriMeshCollisionData TriMeshData;
	if (StaticMesh->GetPhysicsTriMeshData(&TriMeshData, true) && TriMeshData.Vertices.Num() >= 3 && TriMeshData.Indices.Num() > 0)
	{
		OutVertices.Reserve(TriMeshData.Vertices.Num());
		for (const FVector3f& LocalVert : TriMeshData.Vertices)
		{
			const FVector WorldPos = Transform.TransformPosition(FVector(LocalVert));
			OutVertices.Add(FVector(WorldPos.X * UE2CH_SCALE, -WorldPos.Y * UE2CH_SCALE, WorldPos.Z * UE2CH_SCALE));
			OutAABB += WorldPos;
		}

		OutIndices.Reserve(TriMeshData.Indices.Num() * 3);
		for (const FTriIndices& Tri : TriMeshData.Indices)
		{
			OutIndices.Add((int32)Tri.v0);
			OutIndices.Add((int32)Tri.v1);
			OutIndices.Add((int32)Tri.v2);
		}
	}
	else if (FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData())
	{
		// Fallback to LOD0 render data if physics tri mesh data is unavailable
		if (RenderData->LODResources.Num() > 0)
		{
			const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
			const FPositionVertexBuffer& PosBuffer = LOD.VertexBuffers.PositionVertexBuffer;
			const uint32 NumVerts = PosBuffer.GetNumVertices();
			FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();

			if (NumVerts >= 3 && Indices.Num() >= 3)
			{
				OutVertices.Reserve(NumVerts);
				for (uint32 i = 0; i < NumVerts; ++i)
				{
					const FVector WorldPos = Transform.TransformPosition(FVector(PosBuffer.VertexPosition(i)));
					OutVertices.Add(FVector(WorldPos.X * UE2CH_SCALE, -WorldPos.Y * UE2CH_SCALE, WorldPos.Z * UE2CH_SCALE));
					OutAABB += WorldPos;
				}

				OutIndices.Reserve(Indices.Num());
				for (int32 i = 0; i < Indices.Num(); ++i)
				{
					OutIndices.Add((int32)Indices[i]);
				}
			}
		}
	}

	if (OutVertices.Num() < 3 || OutIndices.Num() < 3)
	{
		return false;
	}

	OutChTransform = ChronoTransform(Conversions::ToChronoVec(OutAABB.GetCenter()), chrono::ChQuaterniond(1, 0, 0, 0));
	return true;
}

void FUnrealCollisionExtractor::ExtractStaticMeshTriangleCollision(UStaticMesh* StaticMesh, const FTransform& Transform, CollisionType Collision, UVehiclePhysicalMaterial* PhysMaterial)
{
	if (!StaticMesh) return;

	TArray<FVector> MeshVertices;
	TArray<int32> MeshIndices;
	FBox AABB(ForceInit);
	ChronoTransform ChTransform;
	if (!BuildStaticMeshTriangleMesh(StaticMesh, Transform, MeshVertices, MeshIndices, AABB, ChTransform))
	{
		return;
	}

	UVehiclePhysicalMaterial* VehPhysMat = PhysMaterial;
	UBodySetup* BodySetup = StaticMesh->GetBodySetup();
	const float Mass = (Collision == CollisionType::Static || !BodySetup) ? 0.f : BodySetup->CalculateMass();

	CollisionModelFactory CollisionFactory;
	std::shared_ptr<FCollisionModel> CollisionModel = CollisionFactory.ConstructCollisionModel(MeshType::StaticMesh);
	CollisionModel->ColliderInfo.Friction = VehPhysMat ? VehPhysMat->Friction : 0.9f;
	CollisionModel->ColliderInfo.Restitution = VehPhysMat ? VehPhysMat->Restitution : 0.f;
	CollisionModel->ColliderInfo.PoissonRatio = VehPhysMat ? VehPhysMat->PoissonRatio : 0.3f;
	CollisionModel->ColliderInfo.Density = VehPhysMat ? VehPhysMat->Density : 1000.f;
	CollisionModel->ColliderInfo.RollingFriction = VehPhysMat ? VehPhysMat->RollingFriction : 0.02f;
	CollisionModel->ColliderInfo.SpinningFriction = VehPhysMat ? VehPhysMat->SpinningFriction : 0.01f;
	CollisionModel->ColliderInfo.YoungsModulus = VehPhysMat ? VehPhysMat->YoungsModulus : 1e5f;
	CollisionModel->ColliderInfo.KineticFriction = VehPhysMat ? VehPhysMat->StaticFriction : 0.9f;
	CollisionModel->ColliderInfo.Mass = Mass;
	CollisionModel->ColliderInfo.Position = ChTransform.Location;
	CollisionModel->ColliderInfo.Rotation = ChTransform.Rotation;
	CollisionModel->ColliderInfo.StaticObject = Collision == CollisionType::Static;
	CollisionModel->AABB = AABB;
	CollisionModel->CollideType = Collision;

	auto TriMesh = chrono_types::make_shared<ChTriangleMeshConnected>();
	auto& Verts = TriMesh->GetCoordsVertices();
	auto& Indices = TriMesh->GetIndicesVertexes();
	Verts.reserve(MeshVertices.Num());
	for (const FVector& V : MeshVertices)
	{
		Verts.emplace_back(V.X, V.Y, V.Z);
	}
	Indices.reserve(MeshIndices.Num() / 3);
	for (int32 i = 0; i + 2 < MeshIndices.Num(); i += 3)
	{
		Indices.emplace_back(MeshIndices[i], MeshIndices[i + 1], MeshIndices[i + 2]);
	}

	std::shared_ptr<ChColliderInfo> ColliderInfo = std::make_shared<ChColliderInfo>(CollisionModel->ColliderInfo);
	CollisionModel->Shape = FBulletStaticCollision::CreateTriangleMeshCollision(StaticCollisionContactMethod, TriMesh, ColliderInfo, Settings.EnableChronoVisual);
	CollisionData.push_back(CollisionModel);
}

void FUnrealCollisionExtractor::ExtractInstancedStaticMeshTriangleCollision(UInstancedStaticMeshComponent* InstancedStaticMeshComp, CollisionType Collision)
{
	if (!InstancedStaticMeshComp) return;
	if (InstancedStaticMeshComp->GetCollisionEnabled() == ECollisionEnabled::NoCollision) return;

	UStaticMesh* Mesh = InstancedStaticMeshComp->GetStaticMesh();
	if (!Mesh) return;

	UPhysicalMaterial* PhMat = InstancedStaticMeshComp->BodyInstance.GetSimplePhysicalMaterial();
	UVehiclePhysicalMaterial* VehPhysMat = Cast<UVehiclePhysicalMaterial>(PhMat);

	const int32 NumInstances = InstancedStaticMeshComp->GetInstanceCount();
	const TArray<FBodyInstance*>& InstanceBodies = InstancedStaticMeshComp->GetInstanceBodies();

	for (int32 i = 0; i < NumInstances; ++i)
	{
		FTransform InstanceTx;
		if (!InstancedStaticMeshComp->GetInstanceTransform(i, InstanceTx, true))
		{
			continue;
		}

		UVehiclePhysicalMaterial* InstanceVehPhysMat = VehPhysMat;
		if (InstanceBodies.IsValidIndex(i) && InstanceBodies[i])
		{
			if (InstanceBodies[i]->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
			{
				continue;
			}
			if (UPhysicalMaterial* BodyPhMat = InstanceBodies[i]->GetSimplePhysicalMaterial())
			{
				if (UVehiclePhysicalMaterial* BodyVehPhysMat = Cast<UVehiclePhysicalMaterial>(BodyPhMat))
				{
					InstanceVehPhysMat = BodyVehPhysMat;
				}
			}
		}

		ExtractStaticMeshTriangleCollision(Mesh, InstanceTx, Collision, InstanceVehPhysMat);
	}
}

void FUnrealCollisionExtractor::ExtractStaticMeshTriangleCollision(UStaticMeshComponent* StaticMeshComp, CollisionType Collision, bool bIncludeSubComponents)
{
	if (!StaticMeshComp) return;

	if (UInstancedStaticMeshComponent* InstancedComp = Cast<UInstancedStaticMeshComponent>(StaticMeshComp))
	{
		ExtractInstancedStaticMeshTriangleCollision(InstancedComp, Collision);
	}
	else if (StaticMeshComp->GetCollisionEnabled() != ECollisionEnabled::NoCollision)
	{
		UStaticMesh* Mesh = StaticMeshComp->GetStaticMesh();
		if (Mesh)
		{
			UVehiclePhysicalMaterial* VehPhysMat = Cast<UVehiclePhysicalMaterial>(StaticMeshComp->BodyInstance.GetSimplePhysicalMaterial());
			ExtractStaticMeshTriangleCollision(Mesh, StaticMeshComp->GetComponentTransform(), Collision, VehPhysMat);
		}
	}

	if (bIncludeSubComponents)
	{
		TArray<USceneComponent*> ChildrenComps;
		StaticMeshComp->GetChildrenComponents(true, ChildrenComps);
		for (USceneComponent* Child : ChildrenComps)
		{
			if (UStaticMeshComponent* ChildMeshComp = Cast<UStaticMeshComponent>(Child))
			{
				ExtractStaticMeshTriangleCollision(ChildMeshComp, Collision, false);
			}
		}
	}
}

void FUnrealCollisionExtractor::ExtractTriangleMeshCollision(UStaticMeshComponent* StaticMeshComp, CollisionType Collision, bool bIncludeSubComponents)
{
	ExtractStaticMeshTriangleCollision(StaticMeshComp, Collision, bIncludeSubComponents);
}

void FUnrealCollisionExtractor::ExtractTriangleMeshCollision(UInstancedStaticMeshComponent* InstancedStaticMeshComp, CollisionType Collision)
{
	ExtractInstancedStaticMeshTriangleCollision(InstancedStaticMeshComp, Collision);
}

#pragma optimize("", on)
