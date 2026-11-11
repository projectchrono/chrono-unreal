#include "TestCar_EngineMap.h"

TestCar_EngineMap::TestCar_EngineMap(const std::string& name, FChVehicleSetup* VehicleSetup) 
	: chrono::vehicle::ChEngineSimpleMap(name), m_VehicleSetup(VehicleSetup)
{
}

double TestCar_EngineMap::GetMaxEngineSpeed()
{
	if (m_VehicleSetup)
	{
		return m_VehicleSetup->MaxEngineSpeedRPM * rpm2rads;
	}
	return 6700 * rpm2rads;
}

void TestCar_EngineMap::SetEngineTorqueMaps(chrono::ChFunctionInterp& map0, chrono::ChFunctionInterp& mapF)
{
	//Fall back to the built in maps when the setup carries none (no UEngineMapComponent on the vehicle / tests)
	if (m_VehicleSetup && !m_VehicleSetup->EngineZeroThrottleMap.empty() && !m_VehicleSetup->EngineFullThrottleMap.empty())
	{
		for (const auto& Point : m_VehicleSetup->EngineZeroThrottleMap)
		{
			map0.AddPoint(Point.first * rpm2rads, Point.second);
		}

		for (const auto& Point : m_VehicleSetup->EngineFullThrottleMap)
		{
			mapF.AddPoint(Point.first * rpm2rads, Point.second);
		}

		return;
	}
	
	map0.AddPoint(-10.0, 0.0);
	map0.AddPoint(rpm2rads * 1000, -30.0);
	map0.AddPoint(rpm2rads * 2000, -50.0);
	map0.AddPoint(rpm2rads * 3000, -70.0);
	map0.AddPoint(rpm2rads * 4000, -100.0);
	map0.AddPoint(rpm2rads * 5000, -120.0);
	map0.AddPoint(rpm2rads * 6000, -140.0);
	map0.AddPoint(rpm2rads * 6500, -160.0);

	// The engine speed is tied directly to the wheel speed (no clutch / torque converter in the simple map model), so at a
	// standstill the engine sits at 0 rpm: give it full torque there or the car cannot pull away from rest.
	mapF.AddPoint(-10.0, 450.0);
	mapF.AddPoint(rpm2rads * 800, 450.0);
	mapF.AddPoint(rpm2rads * 1000, 520.0);
	mapF.AddPoint(rpm2rads * 1500, 570.0);
	mapF.AddPoint(rpm2rads * 2000, 610.0);
	mapF.AddPoint(rpm2rads * 2500, 624.0);
	mapF.AddPoint(rpm2rads * 3000, 624.0);
	mapF.AddPoint(rpm2rads * 4000, 610.0);
	mapF.AddPoint(rpm2rads * 5000, 570.0);
	mapF.AddPoint(rpm2rads * 5500, 540.0);
	mapF.AddPoint(rpm2rads * 6000, 500.0);
	mapF.AddPoint(rpm2rads * 6300, 350.0);
	mapF.AddPoint(rpm2rads * 6400, 100.0);
	mapF.AddPoint(rpm2rads * 6500, -200.0);
	
}
