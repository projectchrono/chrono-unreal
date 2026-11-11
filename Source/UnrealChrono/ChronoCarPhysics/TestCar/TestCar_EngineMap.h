#pragma once
#include "ChronoVehicle.h"

class TestCar_EngineMap : public chrono::vehicle::ChEngineSimpleMap 
{
public:
	TestCar_EngineMap(const std::string& name, FChVehicleSetup* VehicleSetup);

protected:
	FChVehicleSetup* m_VehicleSetup;
	/// Specify maximum engine speed.
	virtual double GetMaxEngineSpeed() override;

	/// Set the engine speed-torque maps.
	/// A concrete class must add the speed-torque points to the provided maps,
	/// using the ChFunctionInterp::AddPoint() function.
	virtual void SetEngineTorqueMaps(chrono::ChFunctionInterp& map0,  ///< [out] engine map at zero throttle
	                                 chrono::ChFunctionInterp& mapF   ///< [out] engine map at full throttle
	) override;
};
