#pragma once
#include "ChronoTypes.h"
#include "ChronoWheelSetup.h"

//Test car rim. The size, mass and inertia come from the FChWheelSetup.
class TestCar_Wheel : public chrono::vehicle::ChWheel
{
public:
	TestCar_Wheel(const std::string& NameInit, const FChWheelSetup& SetupInit);
	~TestCar_Wheel() {}
	virtual double GetRadius() const override { return Radius; }
	virtual double GetWidth() const override { return Width; }

protected:
	virtual double GetWheelMass() const override { return Mass; }
	virtual const ChVector3<>& GetWheelInertia() const override { return Inertia; }

	double Mass;
	double Radius;
	double Width;
	ChVector3<> Inertia;
};
