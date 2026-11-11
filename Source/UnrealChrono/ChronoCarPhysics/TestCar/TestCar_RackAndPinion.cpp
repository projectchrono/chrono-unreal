#include "TestCar_RackAndPinion.h"

const double TestCar_RackAndPinion::SteeringLinkMass = 9.072;
const ChVector3d TestCar_RackAndPinion::SteeringLinkInertia(1, 1, 1);
const double TestCar_RackAndPinion::SteeringLinkCOM = 0;
const double TestCar_RackAndPinion::SteeringLinkLength = 0.6;
const double TestCar_RackAndPinion::SteeringLinkRadius = 0.03;

const double TestCar_RackAndPinion::PinionRadius = 0.1;

const double TestCar_RackAndPinion::MaxAngle = 35.0 * (PI / 180);


TestCar_RackAndPinion::TestCar_RackAndPinion(const std::string& NameInit) : ChRackPinion(NameInit) {}