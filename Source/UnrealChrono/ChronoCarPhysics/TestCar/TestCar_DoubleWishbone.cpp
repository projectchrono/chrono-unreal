#include "TestCar_DoubleWishbone.h"

using namespace chrono;
using namespace vehicle;

TestCar_DoubleWishbone::TestCar_DoubleWishbone(const std::string& NameInit, const std::map<int, Vector>& HardpointsIn, const FDoubleWishboneParams& ParamsIn)
    : ChDoubleWishboneReduced(NameInit), Hardpoints(HardpointsIn), Params(ParamsIn)
{
    ShockForce = chrono_types::make_shared<utils::LinearSpringDamperForce>(Params.SpringCoefficient, Params.DampingCoefficient);
}

const ChVector3d TestCar_DoubleWishbone::getLocation(PointId WhichPoint)
{
    const auto It = Hardpoints.find((int)WhichPoint);
    return It != Hardpoints.end() ? It->second : ChVector3d(0, 0, 0);
}
