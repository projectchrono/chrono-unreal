#pragma once
#include "ChronoTypes.h"

class TestCar_MapTransmission : public chrono::vehicle::ChAutomaticTransmissionSimpleMap
{
public:
	TestCar_MapTransmission(const std::string& name);

	/// Engine speed demanded by the transmission. The base class divides |driveshaft speed| by the gear ratio, which
	/// is negative in reverse: the engine then received a negative speed, clamped it to 0 rpm and delivered almost no
	/// torque in reverse. The engine turns the same way in every gear, so report the magnitude.
	virtual double GetOutputMotorshaftSpeed() const override
	{
		return std::abs(chrono::vehicle::ChAutomaticTransmissionSimpleMap::GetOutputMotorshaftSpeed());
	}

protected:
	/// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
	virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

	/// Set the ideal shift points for automatic gear shifting.
	/// For each forward gear, specify the min and max engine speed for shifting (down and up, respectively).
	virtual void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
		) override;

};
