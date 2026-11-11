#pragma once

#include "ChronoTypes.h"
#include "TestCar_Wheel.h"


class TestCarChassis : public chrono::vehicle::ChRigidChassis
{
public:	
	//ChassisHulls: convex hulls (chassis frame, Chrono metres) from the Unreal ChassisCollision mesh, used for
	//PhysicsCollisionType::HULLS. When empty the generated collision .obj is loaded instead.
	TestCarChassis(const std::string& name, bool fixed, PhysicsCollisionType chassis_collision_type,
	                       const std::vector<std::vector<chrono::ChVector3d>>& ChassisHulls = {});
	virtual ~TestCarChassis() {}

	/// Get the name of the vehicle subsystem template.
	virtual std::string GetTemplateName() const override { return "TestCarChassis"; }
	
	std::shared_ptr<chrono::vehicle::ChWheel> FrontDriverWheel;
	std::shared_ptr<chrono::vehicle::ChWheel> FrontPassengerWheel;
	std::shared_ptr<chrono::vehicle::ChWheel> RearPassengerWheel;
	std::shared_ptr<chrono::vehicle::ChWheel> RearDriverWheel;	

	virtual void Construct(chrono::vehicle::ChVehicle* Vehicle,                ///< [in] containing vehicle
	                       const chrono::ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
	                       double chassisFwdVel,            ///< [in] initial chassis forward velocity
	                       int collision_family = 0         ///< [in] chassis collision family
	) override;

	/// Get the location (in the local frame of this chassis) of the connection to a rear chassis.
	virtual const ChVector3d GetLocalPosRearConnector() const override { return m_connector_rear_loc; }
	/// Get the local driver position and orientation.
	/// This is a coordinate system relative to the chassis reference frame.
	virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const override { return DriverCsys; }
	virtual double GetBodyMass() const override { return Mass_kg; }
protected:

	/// Return the mass of the chassis body.
	
	/// Return the inertia tensor of the chassis body.
	virtual chrono::ChMatrix33<> GetBodyInertia() const override { return Inertia; }
	/// Get the location of the center of mass in the chassis frame.
	virtual chrono::ChFrame<> GetBodyCOMFrame() const override { return chrono::ChFrame<>(m_COM_loc, ChQuaternion<double>(1.,0,0,0)); }

	//void CreateContactMaterials(ChContactMethod ContactMethodInit);
	PhysicsCollisionType m_chassis_collision_type;
	chrono::ChMatrix33<> Inertia;
	static const ChVector3d m_inertiaXX;
	static const ChVector3d m_inertiaXY;
	static ChVector3d m_COM_loc;
	static const ChVector3d m_connector_rear_loc;
	static const chrono::ChCoordsys<> DriverCsys;
	static const float Wheelbase_metres;
	static const double Mass_kg;	
public:	
	
};