#pragma once

#include "ChronoTypes.h"
#include <Common.h>
#include <memory>
#include <vector>

// Bullet backend only: Chrono rigid bodies for the level geometry gathered from Unreal (the Chaos backend collides
// with the Unreal level directly, so it never creates these).
//
// The factories only build detached ChBody objects (no ChSystem involved), so they can run on any thread. Attaching
// the bodies to the system (AddBodies) mutates the system and must run on the physics thread (from an
// FChronoSimulation command or the stepper).
class FBulletStaticCollision
{
public:
	static const ChColor STATIC_VIS_COLOR;
	static const ChColor DYNAMIC_VIS_COLOR;
	static const ChColor SKELETAL_VIS_COLOR;

	static std::shared_ptr<chrono::ChBody> CreateBoxCollision(chrono::ChContactMethod ContactMethod, const ChVector3d& BoxSize, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);
	static std::shared_ptr<chrono::ChBody> CreateEllipsoidCollision(chrono::ChContactMethod ContactMethod, const ChVector3d& Size, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);
	static std::shared_ptr<chrono::ChBody> CreateSphereCollision(chrono::ChContactMethod ContactMethod, double Radius, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);
	static std::shared_ptr<chrono::ChBody> CreateCapsuleCollision(chrono::ChContactMethod ContactMethod, double Radius, double HalfLength, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);
	static std::shared_ptr<chrono::ChBody> CreateCylinderCollision(chrono::ChContactMethod ContactMethod, double Radius, double HalfLength, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);
	static std::shared_ptr<chrono::ChBody> CreateTriangleMeshCollision(chrono::ChContactMethod ContactMethod, std::shared_ptr<chrono::ChTriangleMeshConnected> TriMesh, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);
	static std::shared_ptr<chrono::ChBody> CreateConvexCollision(chrono::ChContactMethod ContactMethod, std::vector<ChVector3d>& Hulls, std::shared_ptr<ChColliderInfo> CollisionInfo, bool Visualize);

	//Attaches every body to the system (physics thread only). Used when collision streaming is off; with streaming
	//the FStaticCollisionGrid owns the bodies and adds/removes them per cell.
	static void AddBodies(chrono::ChSystem& System, const std::vector<std::shared_ptr<chrono::ChBody>>& Bodies);

private:
	static std::shared_ptr<chrono::ChContactMaterial> MakeMaterial(chrono::ChContactMethod ContactMethod, const ChColliderInfo& CollisionInfo);
	static void SetStaticFamily(const std::shared_ptr<chrono::ChBody>& Body);
};
