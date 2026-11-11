#include "TestCar_BrakeShafts.h"

#include <string>

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------


const double TestCar_BrakeShafts::Maxtorque = 4000;
const double TestCar_BrakeShafts::ShaftInertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TestCar_BrakeShafts::TestCar_BrakeShafts(const std::string& NameInit) : ChBrakeShafts(NameInit) {}