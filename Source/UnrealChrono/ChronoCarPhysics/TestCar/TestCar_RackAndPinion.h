#pragma once

#include "ChronoTypes.h"

class TestCar_RackAndPinion : public chrono::vehicle::ChRackPinion
{
public:
    TestCar_RackAndPinion(const std::string& name);
    ~TestCar_RackAndPinion() {}

protected:
    virtual double GetSteeringLinkMass() const override { return SteeringLinkMass; }
    virtual ChVector3d GetSteeringLinkInertia() const override { return SteeringLinkInertia; }
    virtual double GetSteeringLinkCOM() const override { return SteeringLinkCOM; }
    virtual double GetSteeringLinkRadius() const override { return SteeringLinkRadius; }
    virtual double GetSteeringLinkLength() const override { return SteeringLinkLength; }

    virtual double GetPinionRadius() const override { return PinionRadius; }

    virtual double GetMaxAngle() const override { return MaxAngle; }

private:
    static const double SteeringLinkMass;
    static const ChVector3d SteeringLinkInertia;
    static const double SteeringLinkCOM;
    static const double SteeringLinkRadius;
    static const double SteeringLinkLength;

    static const double PinionRadius;

    static const double MaxAngle;
};

