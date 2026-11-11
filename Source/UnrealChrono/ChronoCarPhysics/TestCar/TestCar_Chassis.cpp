#include "TestCar_Chassis.h"

#include <filesystem>


const double TestCarChassis::Mass_kg = 2322.847;
const float TestCarChassis::Wheelbase_metres = 2.997;
const ChVector3d TestCarChassis::m_inertiaXX(800, 2400, 3000);
const ChVector3d TestCarChassis::m_inertiaXY(0, 0, 0);
ChVector3d TestCarChassis::m_COM_loc(0.5, 0, 0.483); 
const ChVector3d TestCarChassis::m_connector_rear_loc(-2.65, 0, 0.413);
const chrono::ChCoordsys<> TestCarChassis::DriverCsys(ChVector3d(1, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


TestCarChassis::TestCarChassis(const std::string& name, bool fixed, PhysicsCollisionType chassis_collision_type, const std::vector<std::vector<chrono::ChVector3d>>& ChassisHulls) : chrono::vehicle::ChRigidChassis(name, fixed)
{				
	chrono::ChContactMaterialData minfo;
	minfo.cr = 0.9;
	minfo.Y = 2e7;
	minfo.gn = 0.5f;
	minfo.mu = 0.8;	
	
	m_geometry.materials.push_back(minfo);	

	if (chassis_collision_type == PhysicsCollisionType::HULLS)
	{
		chrono::utils::ChBodyGeometry::ConvexHullsShape hullBody;
		if (!ChassisHulls.empty())
		{
			hullBody.hulls = ChassisHulls;
		}
		else
		{
			//Primitive body: a single box hull (chassis frame, metres) that sits between the wheels
			const double X0 = -2.05, X1 = 2.35, Y = 0.74, Z0 = 0.48, Z1 = 1.45;
			std::vector<chrono::ChVector3d> Box;
			for (double X : { X0, X1 }) for (double Yy : { -Y, Y }) for (double Z : { Z0, Z1 }) Box.emplace_back(X, Yy, Z);
			hullBody.hulls.push_back(Box);
		}
		hullBody.matID = 0;
		hullBody.is_mutable = false;
		m_geometry.coll_hulls.push_back(hullBody);
	}
	
	Inertia(0, 0) = m_inertiaXX.x();
	Inertia(1, 1) = m_inertiaXX.y();
	Inertia(2, 2) = m_inertiaXX.z();

	Inertia(0, 1) = m_inertiaXY.x();
	Inertia(0, 2) = m_inertiaXY.y();
	Inertia(1, 2) = m_inertiaXY.z();
	Inertia(1, 0) = m_inertiaXY.x();
	Inertia(2, 0) = m_inertiaXY.y();
	Inertia(2, 1) = m_inertiaXY.z();
}


void TestCarChassis::Construct(chrono::vehicle::ChVehicle* Vehicle, const chrono::ChCoordsys<>& chassisPos,  double chassisFwdVel, int collision_family)
{
	chrono::vehicle::ChRigidChassis::Construct(Vehicle, chassisPos, chassisFwdVel, collision_family);

	//
	// std::shared_ptr<ChCollisionModelBullet> btModel = std::static_pointer_cast<ChCollisionModelBullet>(GetBody()->GetCollisionModel());
	//
	// if (btModel)
	// {
	// 	cbtCollisionObject* bt= btModel->GetBulletModel(); 
	// 	bt->setCcdMotionThreshold(0.5);
	// 	bt->setCcdSweptSphereRadius(0.2);	
	// }
}

