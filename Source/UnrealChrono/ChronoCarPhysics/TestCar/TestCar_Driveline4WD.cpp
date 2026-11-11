#include "TestCar_Driveline4WD.h"

using namespace chrono;
using namespace vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double TestCar_Driveline4WD::CentralDifferentialboxInertia = 0.4;
const double TestCar_Driveline4WD::FrontDifferentialboxInertia = 0.3;
const double TestCar_Driveline4WD::RearDifferentialboxInertia = 0.3;
const double TestCar_Driveline4WD::DriveshaftInertia = 0.2;
const double TestCar_Driveline4WD::FrontshaftInertia = 0.2;
const double TestCar_Driveline4WD::RearshaftInertia = 0.2;

const double TestCar_Driveline4WD::FrontConicalgearRatio = 1/3.31;
const double TestCar_Driveline4WD::RearConicalgearRatio = 1/3.31;

const double TestCar_Driveline4WD::AxleDifferentialLockingLimit = 500;
const double TestCar_Driveline4WD::CentralDifferentialLockingLimit = 100;

// -----------------------------------------------------------------------------
// Constructor of the TestCar_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
TestCar_Driveline4WD::TestCar_Driveline4WD(const std::string& NameInit) : ChShaftsDriveline4WD(NameInit) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}