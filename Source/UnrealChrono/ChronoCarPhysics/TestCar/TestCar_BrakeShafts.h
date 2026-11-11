#pragma once
#include "ChronoTypes.h"


class TestCar_BrakeShafts : public chrono::vehicle::ChBrakeShafts
{
public:
	TestCar_BrakeShafts(const std::string& NameInit);
	~TestCar_BrakeShafts() {}

protected:
	virtual double GetMaxBrakingTorque() override { return Maxtorque; }
	virtual double GetShaftInertia() override { return ShaftInertia; }

private:
	static const double Maxtorque;
	static const double ShaftInertia;
};

