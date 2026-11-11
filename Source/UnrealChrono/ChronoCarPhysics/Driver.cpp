#include "Driver.h"

void Driver::SetSteeringInput(double SteeringInput)
{
	m_steering = SteeringInput;
}

void Driver::SetThrottleInput(double ThrottleInput)
{
	m_throttle = ThrottleInput;
}

void Driver::SetBrakingInput(double BrakeInput)
{
	m_braking = BrakeInput;	
}

void Driver::SetHandbrakeInput(double HandbrakeInput)
{
	Vehicle->ApplyParkingBrake((bool)HandbrakeInput);
}

void Driver::SetInputs(const FChronoTickInputs& NewInputs)
{
	m_throttle = NewInputs.Throttle;
	m_steering = NewInputs.Steering;
	m_braking = NewInputs.Braking;
	m_handbrake = (NewInputs.Flags & VehicleInputFlags::Handbrake) != 0 ? 1.f : 0.f;
}

void Driver::RequestShift(int Direction)
{
	if (!Vehicle->GetTransmission())
	{
		return;
	}

	//Chrono numbering: -1 = reverse, 0 = neutral (ratio 1e20, no torque), 1+ = forward gears. (The older Chrono used
	//0 for reverse; calling SetGear(0) for "reverse" leaves the car in neutral.)
	const int CurrentGear = Vehicle->GetTransmission()->GetCurrentGear();

	if (Direction > 0)
	{
		//Currently in reverse (or neutral)? Put back into drive gear. Otherwise shift up
		if (CurrentGear <= 0)
		{
			SetForwardDrive();
		}
		else
		{
			GearUp();
		}
	}
	else if (Direction < 0)
	{
		//In lowest gear? Put into reverse. Otherwise shift down
		if (CurrentGear == 1)
		{
			SetReverseDrive();
		}
		else
		{
			GearDown();
		}			
	}	
}

void Driver::SetReverseDrive()
{
	//Automatic transmissions keep a drive mode next to the gear, keep the two consistent
	if (auto Automatic = std::dynamic_pointer_cast<chrono::vehicle::ChAutomaticTransmission>(Vehicle->GetTransmission()))
	{
		Automatic->SetDriveMode(chrono::vehicle::ChAutomaticTransmission::DriveMode::REVERSE);
	}
	else
	{
		Vehicle->GetTransmission()->SetGear(-1);
	}
}

void Driver::SetForwardDrive()
{
	if (auto Automatic = std::dynamic_pointer_cast<chrono::vehicle::ChAutomaticTransmission>(Vehicle->GetTransmission()))
	{
		Automatic->SetDriveMode(chrono::vehicle::ChAutomaticTransmission::DriveMode::FORWARD);
	}
	else
	{
		Vehicle->GetTransmission()->SetGear(1);
	}
}

void Driver::GearUp()
{
	Vehicle->GetTransmission()->ShiftUp();
}

void Driver::GearDown()
{
	Vehicle->GetTransmission()->ShiftDown();
}

void Driver::Init()
{
	Initialize();
}

void Driver::Advance(double Step)
{
	//Integrate taking as many step as required to reach the value "step"
	double t = 0;
	while (t < Step)
	{
		const double h = std::min<>((double)m_StepSize, Step - t);

		const double throttle_deriv = m_ThrottleGain * (m_ThrottleTarget - m_throttle);
		const double steering_deriv = m_SteeringGain * (m_SteeringTarget - m_steering);
		const double braking_deriv = m_BrakingGain * (m_BrakingTarget - m_braking);

		SetThrottle(m_throttle + h * throttle_deriv);
		SetSteering(m_steering + h * steering_deriv);
		SetBraking(m_braking + h * braking_deriv);

		t += h;
	}
}
