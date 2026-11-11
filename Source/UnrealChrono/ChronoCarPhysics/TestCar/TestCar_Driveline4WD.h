#pragma once

#include "ChronoTypes.h"

class TestCar_Driveline4WD : public chrono::vehicle::ChShaftsDriveline4WD
{
public:
    TestCar_Driveline4WD(const std::string& name);
    ~TestCar_Driveline4WD() {}

protected:
    virtual double GetCentralDifferentialBoxInertia() const override { return CentralDifferentialboxInertia; }
    virtual double GetFrontDifferentialBoxInertia() const override { return FrontDifferentialboxInertia; }
    virtual double GetRearDifferentialBoxInertia() const override { return RearDifferentialboxInertia; }
    virtual double GetDriveshaftInertia() const override { return DriveshaftInertia; }
    virtual double GetToFrontDiffShaftInertia() const override { return FrontshaftInertia; }
    virtual double GetToRearDiffShaftInertia() const override { return RearshaftInertia; }

    virtual double GetFrontConicalGearRatio() const override { return FrontConicalgearRatio; }
    virtual double GetRearConicalGearRatio() const override { return RearConicalgearRatio; }

    virtual double GetAxleDifferentialLockingLimit() const override { return AxleDifferentialLockingLimit; }
    virtual double GetCentralDifferentialLockingLimit() const override { return CentralDifferentialLockingLimit; }

private:
    // Shaft inertias.
    static const double CentralDifferentialboxInertia;
    static const double FrontDifferentialboxInertia;
    static const double RearDifferentialboxInertia;
    static const double DriveshaftInertia;
    static const double FrontshaftInertia;
    static const double RearshaftInertia;

    // Gear ratios.
    static const double FrontConicalgearRatio;
    static const double RearConicalgearRatio;

    // Differential locking torque limits.
    static const double AxleDifferentialLockingLimit;
    static const double CentralDifferentialLockingLimit;
};

