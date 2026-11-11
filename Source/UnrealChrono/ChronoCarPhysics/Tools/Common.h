#pragma once
#include "ChronoTypes.h"

#ifndef CH_C_DEG_TO_RAD
#define CH_C_DEG_TO_RAD CH_DEG_TO_RAD
#endif


//typedef unsigned int uint32;



const double rpm2rads = PI / 30;
const double rads2rpm = PI * 30;
const double ftlbs2nm = 1.3558179483;
const double nm2ftlbs = 0.7375621493;

//Bullet collision families. Static level geometry collides with the vehicle and is visible to the terrain sensing
//ray casts (ChronoTerrain::QueryTerrain); the vehicle only collides with the static geometry.
const int STATIC_GROUP = 1<<0;
const int VEHICLE_GROUP = 1<<1;
const int TERRAIN_QUERY_GROUP = 1<<2;
const int STATIC_MASK = VEHICLE_GROUP | TERRAIN_QUERY_GROUP;
const int VEHICLE_MASK = STATIC_GROUP;

struct FDriverInputs
{	
	FDriverInputs() {}
	FDriverInputs(const FDriverInputs& other) : Throttle(other.Throttle), Brake(other.Brake), Steering(other.Steering), Handbrake(other.Handbrake), Shift(other.Shift) {}
	FDriverInputs(float ThrottleInput, float BrakeInput, float SteeringInput, float HandbrakeInput, short ShiftInput) : Throttle(ThrottleInput), Brake(BrakeInput), Steering(SteeringInput), Handbrake(HandbrakeInput), Shift(ShiftInput){}
	float Throttle = 0; 
	float Brake = 0;
	float Steering = 0;
	float Handbrake = 0;
	short Shift = 0;
};

enum TerrainResolution
{
	Centimetres,
	Decimetres,
	Inches,
	Feet
};


class ChronoTransform
{
public:
	ChronoTransform() {}
	ChronoTransform(const ChronoTransform& A) : Location(A.Location), Rotation(A.Rotation){}	
	ChronoTransform(const chrono::ChVector3d& LocationInit, const chrono::ChQuaterniond& RotationInit) : Location(LocationInit), Rotation(RotationInit) {}
	chrono::ChVector3d Location = {0,0,0};
	chrono::ChQuaterniond Rotation = {};

	 //Convert quaternion to Euler angles (in degrees)
	chrono::ChVector3f QuaternionToEuler() const
	{
		double w = Rotation.e0();
		double x = Rotation.e1();
		double y = Rotation.e2();
		double z = Rotation.e3();

		// Roll (X-axis rotation)
		double sinr_cosp = 2.0 * (w * x + y * z);
		double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
		double roll = std::atan2(sinr_cosp, cosr_cosp);

		// Pitch (Y-axis rotation)
		double sinp = 2.0 * (w * y - z * x);
		double pitch;
		if (std::abs(sinp) >= 1)
			pitch = std::copysign(3.1415926535 / 2, sinp);  // Handle gimbal lock
		else
			pitch = std::asin(sinp);

		// Yaw (Z-axis rotation)
		double siny_cosp = 2.0 * (w * z + x * y);
		double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
		double yaw = std::atan2(siny_cosp, cosy_cosp);

		// Convert from radians to degrees
		constexpr double RAD_TO_DEG = 180.0f / 3.1415926535;
		return chrono::ChVector3f(roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);
	}

	// Log Location and Euler rotation to a file
	void LogToFile(const std::string& FilePath) const
	{
		std::ofstream OutFile(FilePath, std::ios::app); // Append mode

		if (!OutFile.is_open())
		{
			std::cerr << "Error: Could not open file " << FilePath << " for logging.\n";
			return;
		}
					
		chrono::ChVector3f EulerAngles = QuaternionToEuler();
		OutFile << "ChronoTransform Log:\n";
		OutFile << "Location: (" << Location.x() << ", " << Location.y() << ", " << Location.z() << ")\n";
		OutFile << "Rotation (Euler Angles): Roll: " << EulerAngles.x()
			<< "ï¿½, Pitch: " << EulerAngles.y()
			<< "ï¿½, Yaw: " << EulerAngles.z() << "ï¿½\n";
		OutFile << "---------------------------------------------\n";	
		

		OutFile.close();
	}
};

enum ESuspensionType : int
{
	MultiLink = 0,
	DoubleWishbone = 1,		
	MacPherson = 2,		
	Solid3LinkAxle = 3,
	HMMWV_DoubleWishbone = 4
};

//Physics-thread side vehicle telemetry gathered by ChronoVehicle::UpdateVehicleData every step. It is copied into the
//immutable FVehicleSnapshot payload (Tools/VehicleSnapshot.h) that the game thread reads; never shared across threads.
struct ChVehicleData
{
public:
	ChVehicleData() {}
	ChVehicleData(const ChVehicleData&& A) : RPM(A.RPM), Torque(A.Torque), Load(A.Load), BrakeInput(A.BrakeInput), DriveshaftTorque(A.DriveshaftTorque),  
	FR_SlipAngle(A.FR_SlipAngle), FL_SlipAngle(A.FL_SlipAngle), RR_SlipAngle(A.RR_SlipAngle), RL_SlipAngle(A.RL_SlipAngle),
	FR_LongitudinalSlip(A.FR_LongitudinalSlip), FL_LongitudinalSlip(A.FL_LongitudinalSlip), RR_LongitudinalSlip(A.RR_LongitudinalSlip), RL_LongitudinalSlip(A.RL_LongitudinalSlip),
	FR_CamberAngle(A.FR_CamberAngle), FL_CamberAngle(A.FL_CamberAngle), RR_CamberAngle(A.RR_CamberAngle), RL_CamberAngle(A.RL_CamberAngle),
	FR_TireDeflection(A.FR_TireDeflection), FL_TireDeflection(A.FL_TireDeflection), RR_TireDeflection(A.RR_TireDeflection), RL_TireDeflection(A.RL_TireDeflection), 
	FR_ContactPoint(A.FR_ContactPoint), FL_ContactPoint(A.FL_ContactPoint), RR_ContactPoint(A.RR_ContactPoint), RL_ContactPoint(A.RL_ContactPoint), CurrentGear(A.CurrentGear), Speed(A.Speed){}
	//FR_SpindleTorque(std::move(A.FR_SpindleTorque)), FL_SpindleTorque(std::move(A.FL_SpindleTorque)), RR_SpindleTorque(std::move(A.RR_SpindleTorque)), RL_SpindleTorque(std::move(A.RL_SpindleTorque)),
	//FR_BrakeLocked(std::move(A.FR_BrakeLocked)), FL_BrakeLocked(std::move(A.FL_BrakeLocked)), RR_BrakeLocked(std::move(A.RR_BrakeLocked)), RL_BrakeLocked(std::move(A.RL_BrakeLocked)) {}
	
	float RPM, Torque, Load, BrakeInput, DriveshaftTorque, FR_SlipAngle, FL_SlipAngle, RR_SlipAngle, RL_SlipAngle, FR_LongitudinalSlip, FL_LongitudinalSlip, RR_LongitudinalSlip, RL_LongitudinalSlip = 0;
	float FR_CamberAngle, FL_CamberAngle, RR_CamberAngle, RL_CamberAngle = 0;
	float FR_TireDeflection, FL_TireDeflection, RR_TireDeflection, RL_TireDeflection = 0;
	chrono::ChVector3f FR_ContactPoint, FL_ContactPoint, RR_ContactPoint, RL_ContactPoint;
	int CurrentGear = 1; //0 == Reverse, 1+ == forward gears
	float Speed;
	//float FR_SpindleTorque, FL_SpindleTorque, RR_SpindleTorque, RL_SpindleTorque = 0;
	//bool FR_BrakeLocked, FL_BrakeLocked, RR_BrakeLocked, RL_BrakeLocked = false;
};

struct VehicleSpecs
{
	VehicleSpecs() {}
	
	float Mass;
	float MaxSteeringAngle;
	float MinTurningRadius;	
	int TotalGears;
};

struct ChColliderInfo
{
    // Default constructor
    ChColliderInfo()
        : Mass(0.0f), Restitution(0.0f), Friction(0.0f), KineticFriction(0.8f), Density(0.0f),
          YoungsModulus(1e5f), PoissonRatio(0.3f), RollingFriction(0.02f), SpinningFriction(0.01f),
          StaticObject(false),
          Position(0, 0, 0), Rotation(1, 0, 0, 0),
          InertiaXX(0, 0, 0), InertiaXY(0, 0, 0)
    {}

    // Copy constructor
    ChColliderInfo(const ChColliderInfo& ChInfo)
        : Mass(ChInfo.Mass), Restitution(ChInfo.Restitution), Friction(ChInfo.Friction),
          KineticFriction(ChInfo.KineticFriction), Density(ChInfo.Density),
          YoungsModulus(ChInfo.YoungsModulus), PoissonRatio(ChInfo.PoissonRatio),
          RollingFriction(ChInfo.RollingFriction), SpinningFriction(ChInfo.SpinningFriction),
          StaticObject(ChInfo.StaticObject),
          Position(ChInfo.Position), Rotation(ChInfo.Rotation),
          InertiaXX(ChInfo.InertiaXX), InertiaXY(ChInfo.InertiaXY)
    {}

    // Parameterized constructor
    ChColliderInfo(float MassInit, float RestitutionInit, float FrictionInit, float KineticFrictionInit, bool StaticObjectInit,
                   chrono::ChVector3d PositionInit, chrono::ChQuaterniond RotationInit,
                   chrono::ChVector3d InertiaXXInit, chrono::ChVector3d InertiaXYInit,
                   float DensityInit = 0.0f,
                   float YoungsModulusInit = 1e5f,
                   float PoissonRatioInit = 0.3f,          
                   float RollingFrictionInit = 0.02f,
                   float SpinningFrictionInit = 0.01f)
        : Mass(MassInit), Restitution(RestitutionInit), Friction(FrictionInit), KineticFriction(KineticFrictionInit),
          Density(DensityInit), YoungsModulus(YoungsModulusInit), PoissonRatio(PoissonRatioInit),    
          RollingFriction(RollingFrictionInit), SpinningFriction(SpinningFrictionInit),
          StaticObject(StaticObjectInit),
          Position(PositionInit), Rotation(RotationInit),
          InertiaXX(InertiaXXInit), InertiaXY(InertiaXYInit)
    {}

    // Move constructor
    ChColliderInfo(ChColliderInfo&& other) noexcept
        : Mass(other.Mass), Restitution(other.Restitution), Friction(other.Friction),
          KineticFriction(other.KineticFriction), Density(other.Density),
          YoungsModulus(other.YoungsModulus), PoissonRatio(other.PoissonRatio),         
          RollingFriction(other.RollingFriction), SpinningFriction(other.SpinningFriction),
          StaticObject(other.StaticObject),
          Position(std::move(other.Position)), Rotation(std::move(other.Rotation)),
          InertiaXX(std::move(other.InertiaXX)), InertiaXY(std::move(other.InertiaXY))
    {
        other.Mass = 0.0f;
        other.Restitution = 0.0f;
        other.Friction = 0.0f;
        other.KineticFriction = 0.8f;
        other.Density = 0.0f;
        other.YoungsModulus = 1e5f;
        other.PoissonRatio = 0.3f;
        other.RollingFriction = 0.02f;
        other.SpinningFriction = 0.01f;
        other.StaticObject = false;
        other.Position = chrono::ChVector3d(0, 0, 0);
        other.Rotation = chrono::ChQuaterniond(1, 0, 0, 0);
        other.InertiaXX = chrono::ChVector3d(0, 0, 0);
        other.InertiaXY = chrono::ChVector3d(0, 0, 0);
    }

    // Member variables
    float Mass;
    float Restitution;
    float Friction;
    float KineticFriction;    // Newly added
    float Density;
    float YoungsModulus;
    float PoissonRatio;      
    float RollingFriction;
    float SpinningFriction;	
    bool StaticObject;
    // World-space mesh/shape transform (chrono space). Used both to place the body and to
    // bucket it into the static collision streaming grid (per instance / per landscape patch).
    chrono::ChVector3d Position;
    chrono::ChQuaterniond Rotation;
    chrono::ChVector3d InertiaXX;
    chrono::ChVector3d InertiaXY;	
};

struct SuspensionGeometry
{
public:
	SuspensionGeometry() {}
	chrono::ChVector3d ORIGIN, SPINDLE, UPRIGHT, SHOCKC, SHOCKU, SHOCKA, SPRINGC, SPRINGA;
};

struct DoubleWishboneGeom : public SuspensionGeometry
{
public:
	chrono::ChVector3d UCA_F, UCA_B, UCA_U, UCA_COM, LCA_F, LCA_B, LCA_U, LCA_CM, TIEROD_C, TIEROD_U;
};

struct MultiLinkGeometry : public SuspensionGeometry
{
public:
	chrono::ChVector3d UA_F, UA_B, UA_U, UA_CM, LAT_C, LAT_U, LAT_CM, TL_C, TL_U, TL_CM, TIEROD_C, TIEROD_U;
};

struct MacPhersonGeometry : public SuspensionGeometry
{
public:
	chrono::ChVector3d UCA_F, UCA_B, UCA_U, UCA_COM, LCA_F, LCA_B, LCA_U, LCA_CM, TIEROD_C, TIEROD_U;
};

struct Solid3LinkAxleGeom : public SuspensionGeometry
{
public:
	chrono::ChVector3d TRI_A, TRI_C, LINK_A, LINK_C;
};
