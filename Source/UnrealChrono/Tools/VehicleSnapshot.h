#pragma once

#include "CoreMinimal.h"
#include "ChronoSimulation.h"
#include "../UECarPhysics/VehicleCommon.h"

// Immutable per physics step state of the Chrono vehicle world, published by FChronoVehicleWorld::MakePayload
// (physics thread) as FChronoSnapshot::Payload and read by the Unreal actors on the game thread.
//
// Plain Unreal types only: cm, left handed, world space unless stated otherwise. No Chrono header is needed to
// consume this struct.

/// Game specific bits of FChronoTickInputs::Flags (the plugin reserves the bits below Flag_UserFirst).
namespace VehicleInputFlags
{
	constexpr uint32 Handbrake = FChronoTickInputs::Flag_UserFirst << 0;
}

/// Per tire state.
struct FTireSnapshot
{
	float SlipAngle = 0.f;
	float LongitudinalSlip = 0.f;
	float CamberAngle = 0.f;
	float Deflection = 0.f;
	/// Tire/terrain contact point (world, cm).
	FVector ContactPoint = FVector::ZeroVector;
	/// Last terrain query of the tire (Bullet: UE ray/sweep functors, Chaos: ChaosTerrain).
	bool bInContact = false;
	double TerrainHeight = 0.0;
	float TerrainFriction = 0.7f;
	FVector TerrainNormal = FVector::UpVector;
	FVector TerrainImpactPoint = FVector::ZeroVector;
	ETerrainType TerrainType = ETerrainType::Asphalt;

	/// Ground heightfield of the discrete tire for the visual deformation (empty for the other tire models, or airborne).
	/// Layout: GroundHeightsCm[Column * GroundStations + Station], heights along GroundAxisZ relative to the wheel centre,
	/// station x along GroundAxisX in [-GroundReachCm, +GroundReachCm], column y along GroundAxisY (axle) in
	/// [-GroundHalfWidthCm, +GroundHalfWidthCm]. The three axes are unit vectors in the UE world frame.
	bool bHasGround = false;
	int32 GroundColumns = 0;
	int32 GroundStations = 0;
	float GroundReachCm = 0.f;
	float GroundHalfWidthCm = 0.f;
	FVector GroundAxisX = FVector::ForwardVector;
	FVector GroundAxisY = FVector::RightVector;
	FVector GroundAxisZ = FVector::UpVector;
	TArray<float> GroundHeightsCm;
};

/// Wheel index convention shared by the arrays below.
namespace VehicleWheel
{
	constexpr int32 FL = 0;
	constexpr int32 FR = 1;
	constexpr int32 RL = 2;
	constexpr int32 RR = 3;
	constexpr int32 Num = 4;
}

struct FVehicleSnapshot : public FChronoSnapshotPayload
{
	/// False until the vehicle has been built on the physics thread (or when the state is NaN).
	bool bValid = false;

	/// Chrono simulation time of this state (seconds).
	double SimTime = 0.0;

	/// Chassis reference frame (world).
	FTransform Body = FTransform::Identity;
	/// Wheel spindle frames relative to the chassis (see VehicleWheel indices).
	FTransform Wheels[VehicleWheel::Num] = { FTransform::Identity, FTransform::Identity, FTransform::Identity, FTransform::Identity };

	// --- powertrain / driver -----------------------------------------------------------------------------------
	/// Vehicle speed (m/s, chrono::vehicle::ChVehicle::GetSpeed).
	float SpeedMS = 0.f;
	/// Engine speed (rad/s, ChEngine::GetMotorSpeed).
	float EngineSpeed = 0.f;
	/// Engine output torque (Nm).
	float EngineTorque = 0.f;
	/// Engine load estimate (Nm, see TestCar::UpdateVehicleData).
	float EngineLoad = 0.f;
	/// Brake torque of the front left brake (Nm).
	float BrakeTorque = 0.f;
	/// 0 == reverse, 1+ == forward gears.
	int32 CurrentGear = 1;
	/// Driver inputs actually applied by the Chrono driver this step.
	float Throttle = 0.f;
	float Steering = 0.f;
	float Braking = 0.f;

	FTireSnapshot Tires[VehicleWheel::Num];

	const FTireSnapshot& GetTire(EWheelLocation Which) const
	{
		switch (Which)
		{
		case EWheelLocation::FrontLeft: return Tires[VehicleWheel::FL];
		case EWheelLocation::FrontRight: return Tires[VehicleWheel::FR];
		case EWheelLocation::RearLeft: return Tires[VehicleWheel::RL];
		case EWheelLocation::RearRight: default: return Tires[VehicleWheel::RR];
		}
	}
};
