#pragma once

#include "ChronoTypes.h"
#include "ChronoWheelSetup.h"
#include "Vehicle/ChDiscreteTire.h"
#include <Common.h>

//Off-road tire: discrete tread element model with enveloping of terrain and rigid objects, brush
//type friction on hard surfaces and Bekker / Wong / Janosi-Hanamoto terramechanics on deformable ground (see
//ChDiscreteTire / DiscreteTireCore and L. Mancardi, "Implementation of an off-road tyre model for real-time dynamic
//driving simulator", Politecnico di Torino 2020).
//
//Takes an FChWheelSetup (size, mass, inflation, grip).
class TestCar_DiscreteTire : public ChDiscreteTire
{
public:
    TestCar_DiscreteTire(const std::string& name, const FChWheelSetup& SetupInit);
    ~TestCar_DiscreteTire() {}

    virtual double GetTireMass() const override { return m_mass; }
    virtual Vector GetTireInertia() const override { return m_inertia; }

protected:
    virtual void SetDiscreteTireParams() override;

private:
    FChWheelSetup m_setup;
    double m_mass;
    Vector m_inertia;

};
