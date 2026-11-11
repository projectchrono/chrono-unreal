#include "BulletStaticCollision.h"

#include "ChronoIncludeBegin.h"
#include "chrono/assets/ChVisualShapeRoundedCylinder.h"
#include "ChronoIncludeEnd.h"

using namespace chrono;

const ChColor FBulletStaticCollision::STATIC_VIS_COLOR = ChColor(0.0f, 1.0f, 0.0f);
const ChColor FBulletStaticCollision::DYNAMIC_VIS_COLOR = ChColor(0.5f, 0.5f, 0.0f);
const ChColor FBulletStaticCollision::SKELETAL_VIS_COLOR = ChColor(1.0f, 0.0f, 0.0f);

std::shared_ptr<ChContactMaterial> FBulletStaticCollision::MakeMaterial(ChContactMethod ContactMethod, const ChColliderInfo& CollisionInfo)
{
	auto Material = ChContactMaterial::DefaultMaterial(ContactMethod);
	Material->SetRestitution(CollisionInfo.Restitution);
	Material->SetStaticFriction(CollisionInfo.Friction);
	//KineticFriction comes from UPhysicalMaterial::StaticFriction, which Unreal leaves at 0 unless it is authored. The tire
	//models read the sliding friction of the ground as their friction coefficient, so a 0 would mean no traction on Bullet
	//while Chaos (which uses UPhysicalMaterial::Friction) gives full grip: fall back to the same coefficient.
	Material->SetSlidingFriction(CollisionInfo.KineticFriction > 0.f ? CollisionInfo.KineticFriction : CollisionInfo.Friction);
	Material->SetRollingFriction(CollisionInfo.RollingFriction);
	Material->SetSpinningFriction(CollisionInfo.SpinningFriction);
	return Material;
}

void FBulletStaticCollision::SetStaticFamily(const std::shared_ptr<ChBody>& Body)
{
	if (const std::shared_ptr<ChCollisionModel>& Model = Body->GetCollisionModel())
	{
		Model->SetFamilyGroup(STATIC_GROUP);
		Model->DisallowCollisionsWith(STATIC_GROUP);
		Model->SetFamilyMask(STATIC_MASK);
	}
}

void FBulletStaticCollision::AddBodies(ChSystem& System, const std::vector<std::shared_ptr<ChBody>>& Bodies)
{
	for (const std::shared_ptr<ChBody>& Body : Bodies)
	{
		if (Body && Body->GetSystem() == nullptr)
		{
			System.AddBody(Body);
		}
	}
}

std::shared_ptr<ChBody> FBulletStaticCollision::CreateBoxCollision(ChContactMethod ContactMethod, const ChVector3d& BoxSize, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);
	auto Box = chrono_types::make_shared<ChBodyEasyBox>(BoxSize.x(), BoxSize.y(), BoxSize.z(), CollisionInfo->Density, Visualize, true, Material);
	Box->SetPos(CollisionInfo->Position);
	Box->SetRot(CollisionInfo->Rotation);
	Box->SetFixed(CollisionInfo->StaticObject);
	SetStaticFamily(Box);
	if (Visualize)
		Box->GetVisualShape(0)->SetColor(STATIC_VIS_COLOR);
	return Box;
}

std::shared_ptr<ChBody> FBulletStaticCollision::CreateEllipsoidCollision(ChContactMethod ContactMethod, const ChVector3d& Size, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);
	auto Ellipsoid = chrono_types::make_shared<ChBodyEasyEllipsoid>(ChVector3d(Size.x(), Size.y(), Size.z()), CollisionInfo->Density, Visualize, true, Material);
	Ellipsoid->SetPos(CollisionInfo->Position);
	Ellipsoid->SetRot(CollisionInfo->Rotation);
	Ellipsoid->SetFixed(CollisionInfo->StaticObject);
	SetStaticFamily(Ellipsoid);
	if (Visualize)
		Ellipsoid->GetVisualShape(0)->SetColor(STATIC_VIS_COLOR);
	return Ellipsoid;
}

std::shared_ptr<ChBody> FBulletStaticCollision::CreateSphereCollision(ChContactMethod ContactMethod, double Radius, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);
	auto Sphere = chrono_types::make_shared<ChBodyEasySphere>(Radius, CollisionInfo->Density, Visualize, true, Material);
	Sphere->SetPos(CollisionInfo->Position);
	Sphere->SetRot(CollisionInfo->Rotation);
	Sphere->SetFixed(CollisionInfo->StaticObject);
	SetStaticFamily(Sphere);
	if (Visualize)
		Sphere->GetVisualShape(0)->SetColor(STATIC_VIS_COLOR);
	return Sphere;
}

std::shared_ptr<ChBody> FBulletStaticCollision::CreateCapsuleCollision(ChContactMethod ContactMethod, double Radius, double HalfLength, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);

	auto Capsule = chrono_types::make_shared<ChBody>();
	Capsule->SetPos(CollisionInfo->Position);
	Capsule->SetRot(CollisionInfo->Rotation);
	Capsule->SetFixed(CollisionInfo->StaticObject);

	auto CollisionShape = chrono_types::make_shared<ChCollisionShapeCapsule>(Material, Radius, HalfLength * 2.0);
	Capsule->AddCollisionShape(CollisionShape);
	SetStaticFamily(Capsule);
	Capsule->EnableCollision(true);

	if (Visualize)
	{
		auto VisualShape = chrono_types::make_shared<ChVisualShapeRoundedCylinder>(Radius, HalfLength * 2.0, Radius);
		VisualShape->SetColor(STATIC_VIS_COLOR);
		Capsule->AddVisualShape(VisualShape);
	}
	return Capsule;
}

std::shared_ptr<ChBody> FBulletStaticCollision::CreateCylinderCollision(ChContactMethod ContactMethod, double Radius, double HalfLength, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);
	//Axis direction always Z??? or pass in the collision info rot?
	auto Cylinder = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, Radius, HalfLength * 2, CollisionInfo->Density, Visualize, true, Material);
	Cylinder->SetPos(CollisionInfo->Position);
	Cylinder->SetRot(CollisionInfo->Rotation);
	if (Visualize)
		Cylinder->GetVisualShape(0)->SetColor(STATIC_VIS_COLOR);
	Cylinder->SetFixed(CollisionInfo->StaticObject);
	SetStaticFamily(Cylinder);
	return Cylinder;
}

//this assumes WORLD space mesh verts for the landscape. Needs work to support component space verts!!
std::shared_ptr<ChBody> FBulletStaticCollision::CreateTriangleMeshCollision(ChContactMethod ContactMethod, std::shared_ptr<ChTriangleMeshConnected> TriMesh, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);
	auto Mesh = chrono_types::make_shared<ChBodyEasyMesh>(TriMesh, CollisionInfo->Density, true, Visualize, true, Material, 0.001);
	// ChBodyEasyMesh derives from ChBodyAuxRef: the collision/visual mesh is defined
	// relative to the auxiliary REF frame, while SetPos()/SetRot() operate on the COM
	// frame. Because the constructor computes a (non-zero) COM offset with a
	// principal-inertia rotation, using SetPos/SetRot would drag the REF frame - and
	// therefore the mesh geometry - into a rotated and mis-translated pose. Since the
	// mesh vertices are baked directly in Chrono world space, position the REF frame
	// (not the COM) at the world origin via SetFrameRefToAbs so the geometry stays where it was built.
	Mesh->SetFrameRefToAbs(ChFramed(ChVector3d(0, 0, 0), ChQuaterniond(1, 0, 0, 0)));
	Mesh->SetFixed(CollisionInfo->StaticObject);
	if (Visualize)
		Mesh->GetVisualShape(0)->SetColor(STATIC_VIS_COLOR);
	SetStaticFamily(Mesh);
	return Mesh;
}

std::shared_ptr<ChBody> FBulletStaticCollision::CreateConvexCollision(ChContactMethod ContactMethod, std::vector<ChVector3d>& Hulls, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize)
{
	auto Material = MakeMaterial(ContactMethod, *CollisionInfo);
	auto ConvexBody = chrono_types::make_shared<ChBodyEasyConvexHull>(Hulls, CollisionInfo->Density, Visualize, true, Material);
	ConvexBody->SetFixed(CollisionInfo->StaticObject);
	ConvexBody->SetPos(CollisionInfo->Position);
	ConvexBody->SetRot(CollisionInfo->Rotation);
	if (Visualize)
		ConvexBody->GetVisualShape(0)->SetColor(STATIC_VIS_COLOR);
	SetStaticFamily(ConvexBody);
	return ConvexBody;
}
