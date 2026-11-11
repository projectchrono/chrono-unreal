#include "TestCar_TMeasyTire.h"
#include <algorithm>

TestCar_TMeasyTire::TestCar_TMeasyTire(const std::string& name, const FChWheelSetup& SetupInit) : ChTMeasyTire(name),
    m_setup(SetupInit),
    m_mass(SetupInit.TireMass),
    m_inertia(SetupInit.TireInertia[0], SetupInit.TireInertia[1], SetupInit.TireInertia[2])
{
    SetTMeasyParams();
}

void TestCar_TMeasyTire::SetTMeasyParams()
{
    const double rimdia = 2.0 * m_setup.RimRadius;                       // rim diameter [m]
    const double r = m_setup.TireSectionHeight / m_setup.TireWidth;      // aspect ratio

    GuessPassCar70Par(m_setup.RatedLoad, m_setup.TireWidth, r, rimdia, m_setup.RatedPressure, m_setup.InflationPressure, m_setup.DampingRatio);

    // The guess is for a tire with a friction coefficient of 0.8; the force levels scale with mu_road / mu_0, so a
    // stickier tire (GripScale > 1) is the same tire with a smaller reference friction
    SetFrictionCoefficient(0.8 / std::max(m_setup.GripScale, 0.1));
    SetRollingResistanceCoefficient(m_setup.RollingResistance);
}
