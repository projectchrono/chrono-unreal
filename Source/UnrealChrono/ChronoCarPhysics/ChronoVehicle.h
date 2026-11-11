#pragma once
#include <vector>
#include <string>
#include <utility>
#include "ChronoTypes.h"
#include "PhysicsDefs.h"
#include <Common.h>
#include "VehicleCommon.h"
#include "ChronoWheelSetup.h"

//Reduced double wishbone suspension parameters (masses in kg, inertias in kg m^2, spring/damping in SI units, length in m)
struct FDoubleWishboneParams
{
    double SpringCoefficient  = 180000.0;
    double DampingCoefficient = 18000.0;
    double SpringRestLength   = 0.49;
    double SpindleMass        = 4.0;
    double UprightMass        = 8.0;
    chrono::ChVector3d SpindleInertia = chrono::ChVector3d(0.015, 0.015, 0.015);
    chrono::ChVector3d UprightInertia = chrono::ChVector3d(0.08, 0.08, 0.08);
    double AxleInertia        = 0.08;
};

//Everything the Chrono vehicle model needs, filled on the game thread (plain data).
struct FChVehicleSetup
{
public:
    //Hardpoints of the front and rear suspension (chassis relative, metres), keyed by ChDoubleWishboneReduced::PointId
    std::map<int, Vector> FrontSuspensionLocations, RearSuspensionLocations;
    ETireModelType TireModel = ETireModelType::DISCRETE;
    chrono::vehicle::ChTire::CollisionType TireCollisionType = chrono::vehicle::ChTire::CollisionType::ENVELOPE;

    // Wheel (rim) and tire data
    FChWheelSetup Wheels;

    FDoubleWishboneParams Front;
    FDoubleWishboneParams Rear;

    // Chassis collision hulls (chassis frame, Chrono metres). When empty the chassis uses a built in box hull.
    std::vector<std::vector<chrono::ChVector3d>> ChassisCollisionHulls;

    // Engine parameters
    double MaxEngineSpeedRPM = 6700.0;
    std::vector<std::pair<double, double>> EngineZeroThrottleMap;
    std::vector<std::pair<double, double>> EngineFullThrottleMap;
};

class ChronoVehicle : public chrono::vehicle::ChWheeledVehicle
{
public:			
    
    //Construct the vehicle on top of an already existing Chrono system (the vehicle does not own it).
    //The system is the one hosted by FChronoSimulation on the physics thread: ChVehicle::Advance does not step
    //it, the stepper (FChronoVehicleWorld::SubStep) calls DoStepDynamics once per substep. The owner also
    //configures the collision system type (Bullet or Chaos) on the system.
    ChronoVehicle(const std::string& NameInit, chrono::ChSystem* SystemInit);

    virtual ~ChronoVehicle() { }

    void SetContactMethod(chrono::ChContactMethod val) { m_contactMethod = val; }
    void SetCollisionSystemType(chrono::ChCollisionSystem::Type collsys_type) { m_collsysType = collsys_type; }
    //Collision backend of the system the vehicle is attached to (CHAOS when the system uses ChCollisionSystemChaos)
    static chrono::ChCollisionSystem::Type DetectCollisionSystemType(const chrono::ChSystem* System);

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(PhysicsCollisionType val) { m_chassisCollisionType = val; }
    void SetSteeringType(chrono::vehicle::SteeringTypeWV SteeringType) { m_steeringType = SteeringType; }
    void SetDriveType(chrono::vehicle::DrivelineTypeWV val) { m_driveType = val; }
    void SetBrakeType(chrono::vehicle::BrakeType brake_type) { m_brake_type = brake_type; }
    void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }
    void SetTireType(ETireModelType val) { m_tireType = val; }
    // std::shared_ptr<chrono::ChCollisionShapeCylinder> CreateCollisionTire(const float Radius, const float Height, const Vector& StartingLocation,
    //                                                                       const ChQuaternion<>& StartingRotation,  std::shared_ptr<chrono::ChBody> AnchorSpindle);
    virtual std::shared_ptr<chrono::ChTriangleMeshConnected> GetTriangleDebugMesh();

    virtual std::shared_ptr<SuspensionGeometry> GetSuspensionGeometry(const bool Front, ESuspensionType& SuspOut) const;

    /// Return the global transform (name + coordinate system) of every suspension body.
    /// Used for runtime debug visualization of the suspension components. The returned
    /// coordinate systems are expressed in the Chrono global frame (meters) and can be
    /// converted to Unreal transforms via Conversions::ToUETransform.
    /// The base implementation returns an empty list; concrete vehicles override this.
    virtual std::vector<std::pair<std::string, chrono::ChCoordsysd>> GetSuspensionDebugTransforms() const { return {}; }

    virtual ChVehicleData * UpdateVehicleData();

    void SetTerrain(std::shared_ptr<chrono::vehicle::ChTerrain> TerrainInit);
    std::shared_ptr<chrono::vehicle::ChTerrain> GetTerrain() const { return m_Terrain; }
    virtual void ToggleCenterDiffLock(bool Locked);
    virtual void ToggleAxleDiffLock(bool FrontAxle, bool Locked);

    void SetTireCollisionType(chrono::vehicle::ChTire::CollisionType collision_type) { m_tire_collision_type = collision_type; }
    void SetInitPosition(const chrono::ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }  
    void EnableBrakeLocking(bool lock) { m_brake_locking = lock; }

    std::shared_ptr<FChVehicleSetup> VehicleSetup;
   
    chrono::ChSystem* GetSystem() const { return m_system; }  

    std::shared_ptr<chrono::vehicle::ChChassis> GetChassis() const { return m_chassis; }
    std::shared_ptr<chrono::ChBodyAuxRef> GetChassisBody() const { return m_chassis_body; }
    std::shared_ptr<ChPowertrainAssembly> GetPowertrain() const { return m_powertrain_assembly; }
    double GetTotalMass() const;   
    virtual void Setup() {}    
    virtual void Create() {};

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    void LockAxleDifferential(int axle, bool lock) { ChWheeledVehicle::LockAxleDifferential(axle, lock); }
    void LockCentralDifferential(int which, bool lock) { ChWheeledVehicle::LockCentralDifferential(which, lock); }   

    virtual unsigned int GetNumberAxles() const override { return 2; }
    virtual double GetWheelbase() const override { return m_wheelbase; }
    virtual double GetMaxSteeringAngle() const override { return m_max_steering_angle * chrono::CH_DEG_TO_RAD; }

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void AssignDriverMass(double mass);

    void SetChassisVisualizationType(chrono::VisualizationType vis) { ChVehicle::SetChassisVisualizationType(vis); }
    void SetSuspensionVisualizationType(chrono::VisualizationType vis) { ChWheeledVehicle::SetSuspensionVisualizationType(vis); }
    void SetSteeringVisualizationType(chrono::VisualizationType vis) { ChWheeledVehicle::SetSteeringVisualizationType(vis); }
    void SetWheelVisualizationType(chrono::VisualizationType vis) { ChWheeledVehicle::SetWheelVisualizationType(vis); }
    void SetTireVisualizationType(chrono::VisualizationType vis); 
    //void GetWheelLocations(FVector& FL_Location, FVector& FR_Location, FVector& RR_Location, FVector& RL_Location);
    
    ESuspensionType SuspensionType;
    chrono::ChContactMethod m_contactMethod;
    chrono::ChCollisionSystem::Type m_collsysType = chrono::ChCollisionSystem::Type::BULLET;
    PhysicsCollisionType m_chassisCollisionType;
    bool m_fixed;
    bool m_brake_locking;
    chrono::vehicle::BrakeType m_brake_type;
    ETireModelType m_tireType;
    chrono::vehicle::DrivelineTypeWV m_driveType;
    PowertrainModelType m_powertrainType;
    std::shared_ptr<chrono::ChBodyAuxRef> m_chassis_body;
    chrono::vehicle::ChTire::CollisionType m_tire_collision_type;
    chrono::VisualizationType m_tire_visualization_type;
    chrono::vehicle::SteeringTypeWV m_steeringType;

    ChVehicleData m_VehicleData;
    std::shared_ptr<chrono::vehicle::ChTerrain> m_Terrain;

    uintptr_t FR_TirePtr;
    uintptr_t FL_TirePtr;
    uintptr_t RR_TirePtr;
    uintptr_t RL_TirePtr;

    std::shared_ptr<chrono::vehicle::ChTire> FR_Tire;
    std::shared_ptr<chrono::vehicle::ChTire> FL_Tire;
    std::shared_ptr<chrono::vehicle::ChTire> RR_Tire;
    std::shared_ptr<chrono::vehicle::ChTire> RL_Tire;

    std::shared_ptr<chrono::vehicle::ChWheel> FR_Wheel;
    std::shared_ptr<chrono::vehicle::ChWheel> FL_Wheel;
    std::shared_ptr<chrono::vehicle::ChWheel> RR_Wheel;
    std::shared_ptr<chrono::vehicle::ChWheel> RL_Wheel;

    double m_tire_step_size = 0.003;
    double m_driver_mass;    
    double m_initFwdVel;

    chrono::ChCoordsys<> m_initPos;
    std::vector<double> m_initOmega;

    bool m_usetierod_bodies;
    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;   
    double m_max_steering_angle;
    double m_wheelbase;
    double m_tire_mass;

private:
	std::vector<double> Omega;
};