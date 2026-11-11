#pragma once

#include "ChronoVehicle.h"
#include "Common.h"
#include "PhysicsDefs.h"
#include "ChronoSimulation.h"
#include "../Tools/VehicleSnapshot.h"

using namespace chrono;
using namespace vehicle;

//Chrono driver fed from the idempotent per frame inputs pushed by the game thread (FChronoSimulation::PushInputs).
//Throttle / steering / braking / handbrake are states and come through SetInputs every physics step; gear shifts
//are events and arrive as one-shot commands (ShiftUp / ShiftDown, see FChronoVehicleWorld::RequestShift).
class Driver : public ChDriver
{

public:	
	Driver(ChronoVehicle* VehicleInit, double DriversMassInit) : ChDriver(*VehicleInit), DriversMass(DriversMassInit)
	{
		Vehicle = VehicleInit;
	}
	void SetSteeringInput(double SteeringInput);
	void SetThrottleInput(double ThrottleInput);
	void SetBrakingInput(double BrakeInput);
	void SetHandbrakeInput(double HandbrakeInput);
	void SetSteeringDelta(double SteeringDelta) { m_SteeringDelta = SteeringDelta; }
	void SetThrottleDelta(double ThrottleDelta) { m_ThrottleDelta = ThrottleDelta; }
	void SetBrakingDelta(double BrakingDelta) { m_BrakingDelta = BrakingDelta; }
	void GearUp();
	void GearDown();
	//Chrono gear numbering: -1 reverse, 0 neutral, 1+ forward
	void SetReverseDrive();
	void SetForwardDrive();
	void Init();	
	void SetInputs(const FChronoTickInputs& NewInputs);
	//Shift request from the game thread: +1 up (or out of reverse), -1 down (or into reverse from first)
	void RequestShift(int Direction);
	void Advance(double Step) override;	

	double DriversMass = 180; //lbs
	double m_ThrottleDelta = 0;
	double m_BrakingDelta = 0;
	double m_SteeringDelta = 0;
	//Optional smoothing towards a target (Advance). Zero gains: the inputs pushed every step ARE the state
	double m_ThrottleGain = 0;
	double m_SteeringGain = 0;
	double m_BrakingGain = 0;
	double m_BrakingTarget = 0;
	double m_SteeringTarget = 0;
	double m_ThrottleTarget = 0;

	float m_handbrake = 0.f;	

	int m_StepSize = 4;

	ChronoVehicle * Vehicle;	
};
