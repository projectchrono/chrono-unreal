#pragma once

#include "../ChronoVehicle.h"
#include "../PhysicsDefs.h"
#include "TestCar_BrakeShafts.h"
#include "TestCar_Chassis.h"
#include "TestCar_DiscreteTire.h"
#include "TestCar_DoubleWishbone.h"
#include "TestCar_Driveline4WD.h"
#include "TestCar_EngineMap.h"
#include "TestCar_MapTransmission.h"
#include "TestCar_RackAndPinion.h"
#include "TestCar_RigidTire.h"
#include "TestCar_TMeasyTire.h"
#include "TestCar_Wheel.h"
#include <Common.h>

// Minimal four wheel drive test car: double wishbone suspension on both axles, rack and pinion steering, 4WD
// driveline, simple map engine + automatic transmission, discrete (terramechanics) tires. Everything is built from
// primitives: no mesh or data files are loaded.
class TestCar : public ChronoVehicle
{
public:
    // Attach the vehicle to an externally created system (the one hosted by FChronoSimulation)
    TestCar(const std::string& NameInit, chrono::ChSystem* SystemInit) : ChronoVehicle(NameInit, SystemInit) {}
    virtual ~TestCar() {}

    void Create() override;
    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;
    ChVehicleData* UpdateVehicleData() override;

    virtual unsigned int GetNumberAxles() const override { return 2; }
    virtual double GetWheelbase() const override { return m_wheelbase; }
    virtual double GetMinTurningRadius() const override { return 9.1; }
    virtual double GetMaxSteeringAngle() const override { return m_max_steering_angle * chrono::CH_DEG_TO_RAD; }

    std::vector<double> m_omega{ 0,0,0,0 };
};
