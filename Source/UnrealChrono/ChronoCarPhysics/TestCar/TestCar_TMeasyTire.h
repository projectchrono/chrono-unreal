#pragma once

#include "ChronoTypes.h"
#include "ChronoWheelSetup.h"
#include <Common.h>

// TMeasy tire. Size, mass, inflation and grip come from the FChWheelSetup.
class TestCar_TMeasyTire : public chrono::vehicle::ChTMeasyTire
{
public:
    TestCar_TMeasyTire(const std::string& name, const FChWheelSetup& SetupInit);
    ~TestCar_TMeasyTire() {}

    virtual double GetVisualizationWidth() const override { return m_setup.TireWidth; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual Vector GetTireInertia() const override { return m_inertia; }

protected:
    virtual void SetTMeasyParams() override;

private:
    FChWheelSetup m_setup;
    double m_mass;
    Vector m_inertia;
};
