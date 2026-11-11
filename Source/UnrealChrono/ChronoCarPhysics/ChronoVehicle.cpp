#include "ChronoVehicle.h"

//The vehicle is attached to an externally created and owned Chrono system (hosted by FChronoSimulation).
//ChVehicle::Advance does not step the system, the stepper calls DoStepDynamics itself. The owner also
//configures the collision system type (Bullet or Chaos) on the system; SetCollisionSystemType only records
//it for the vehicle.
ChronoVehicle::ChronoVehicle(const std::string& NameInit, chrono::ChSystem* SystemInit)
    : m_contactMethod(SystemInit->GetContactMethod()),
    m_collsysType(DetectCollisionSystemType(SystemInit)),
    m_chassisCollisionType(PhysicsCollisionType::NONE),
    m_fixed(false),
    m_brake_locking(false),
    m_brake_type(chrono::vehicle::BrakeType::SIMPLE),
    m_tireType(ETireModelType::TMEASY),
	m_powertrainType(PowertrainModelType::SIMPLE_MAP),
    m_tire_collision_type(chrono::vehicle::ChTire::CollisionType::SINGLE_POINT),
    m_initFwdVel(0),
    m_initPos(chrono::ChCoordsys<>(Vector(0, 0, 0), ChQuaternion<double>(1.,0.,0.,0.))),
    m_initOmega({ 0, 0, 0, 0 }),
    m_air_density(0.01),
	m_tire_mass(37), //lbs
    m_steeringType(chrono::vehicle::SteeringTypeWV::RACK_PINION),
    m_usetierod_bodies(false),
    m_driver_mass(81.65),
    ChWheeledVehicle(NameInit, SystemInit),
	m_apply_drag(false),
    m_driveType(chrono::vehicle::DrivelineTypeWV::AWD),
    m_area(0),
    m_Cd(0){}
//                                                                                      std::shared_ptr<chrono::ChBody> AnchorSpindle)
// {
// }
//
// std::shared_ptr<chrono::ChCollisionShapeCylinder> ChronoVehicle::CreateCollisionTire(const float Radius, const float Height, const Vector& StartingLocation, const ChQuaternion<>& StartingRotation,
//     std::shared_ptr<chrono::ChBody> AnchorSpindle)
// {
// }

chrono::ChCollisionSystem::Type ChronoVehicle::DetectCollisionSystemType(const chrono::ChSystem* System)
{
    if (System && dynamic_cast<chrono::ChCollisionSystemChaos*>(System->GetCollisionSystem().get()))
    {
        return chrono::ChCollisionSystem::Type::CHAOS;
    }
    return chrono::ChCollisionSystem::Type::BULLET;
    
}

std::shared_ptr<chrono::ChTriangleMeshConnected> ChronoVehicle::GetTriangleDebugMesh()
{
    return std::shared_ptr<chrono::ChTriangleMeshConnected>();
}

std::shared_ptr<SuspensionGeometry> ChronoVehicle::GetSuspensionGeometry(const bool Front, ESuspensionType& SuspOut) const
{
    return nullptr; //overriden in children
}

ChVehicleData* ChronoVehicle::UpdateVehicleData()
{
    return nullptr;
}

void ChronoVehicle::SetTerrain(std::shared_ptr<chrono::vehicle::ChTerrain> TerrainInit)
{
    m_Terrain = TerrainInit;
}

void ChronoVehicle::ToggleCenterDiffLock(bool Locked)
{
}

void ChronoVehicle::ToggleAxleDiffLock(bool FrontAxle, bool Locked)
{
}

//ChronoVehicle::ChronoVehicle(ChSystem* system, const std::string& NameInit)
//    : m_contactMethod(system->GetContactMethod()),
//    m_collsysType(collision::ChCollisionSystemType::BULLET),
//    m_chassisCollisionType(PhysicsCollisionType::NONE),
//    m_fixed(false),
//    m_brake_locking(false),
//    m_brake_type(BrakeType::SHAFTS),
//    m_driveType(DrivelineTypeWV::AWD),
//    m_powertrainType(PowertrainModelType::SIMPLE_MAP),
//    m_tireType(TireModelType::TMEASY),
//    m_tire_collision_type(ChTire::CollisionType::ENVELOPE),
//    m_tire_step_size(0.001),
//    m_initFwdVel(0),
//    m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), chrono::Quaternion(1,0,0,0))),
//    m_initOmega({ 0, 0, 0, 0 }),
//    m_tire_mass(24.f), //lbs
//    m_air_density(0.01),
//    m_steeringType(chrono::vehicle::SteeringTypeWV::RACK_PINION),
//    m_usetierod_bodies(false),
//    m_area(0),
//    m_Cd(0),
//    m_driver_mass(81.65),
//    m_apply_drag(false),
//    ChWheeledVehicle(NameInit, system) {}

// -----------------------------------------------------------------------------
void ChronoVehicle::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;
    m_apply_drag = true;
}

void ChronoVehicle::AssignDriverMass(double mass)
{
    m_driver_mass = mass;
}

// -----------------------------------------------------------------------------
void ChronoVehicle::SetTireVisualizationType(chrono::VisualizationType vis)
{
    for (auto& axle : GetAxles()) {
        for (auto& wheel : axle->GetWheels())
        {
            wheel->GetTire()->SetVisualizationType(vis);
        }
    }
}
//
//void ChronoVehicle::GetWheelLocations(FVector& FL_Location, FVector& FR_Location, FVector& RR_Location, FVector& RL_Location)
//{
//   /* Conversions::ChronoVecToUnigine(FL_Wheel->GetPos(), FL_Location);
//    Conversions::ChronoVecToUnigine(FR_Wheel->GetPos(), FR_Location);
//    Conversions::ChronoVecToUnigine(RR_Wheel->GetPos(), RR_Location);
//    Conversions::ChronoVecToUnigine(RL_Wheel->GetPos(), RL_Location);*/
//}

// -----------------------------------------------------------------------------
double ChronoVehicle::GetTotalMass() const
{
    return GetMass() + m_driver_mass;
}

void ChronoVehicle::Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel)
{
    //Overriden in child
}
