#include "TestCar_Wheel.h"

TestCar_Wheel::TestCar_Wheel(const std::string& NameInit, const FChWheelSetup& SetupInit) : ChWheel(NameInit),
    Mass(SetupInit.RimMass),
    Radius(SetupInit.RimRadius),
    Width(SetupInit.RimWidth),
    Inertia(SetupInit.RimInertia[0], SetupInit.RimInertia[1], SetupInit.RimInertia[2])
{
}
