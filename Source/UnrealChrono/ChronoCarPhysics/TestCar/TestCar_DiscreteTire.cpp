#include "TestCar_DiscreteTire.h"
#include <algorithm>
#include <cmath>

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TestCar_DiscreteTire::TestCar_DiscreteTire(const std::string& name, const FChWheelSetup& SetupInit) : ChDiscreteTire(name),
    m_setup(SetupInit),
    m_mass(SetupInit.TireMass),
    m_inertia(SetupInit.TireInertia[0], SetupInit.TireInertia[1], SetupInit.TireInertia[2])
{
    SetDiscreteTireParams();
}

void TestCar_DiscreteTire::SetDiscreteTireParams()
{
    const double rimdia = 2.0 * m_setup.RimRadius;                         // rim diameter [m]
    const double ratio = m_setup.TireSectionHeight / m_setup.TireWidth;    // aspect ratio

    // Radius, width, vertical stiffness / damping and slip stiffnesses from the tire size (same guess as TMeasy)
    GuessTireData(m_setup.RatedLoad, m_setup.TireWidth, ratio, rimdia, m_setup.RatedPressure, m_setup.InflationPressure, m_setup.DampingRatio);

    // Discretization. Ring resolution 3 degrees (120 elements) keeps the ripple of the vertical force under about
    // 2 %, five strips across the tread give the camber / overturning response, three columns of ground samples
    // are interpolated to the strips, fifteen stations cover the tread radius plus a margin (a face in front of
    // the tire is seen). Terrain queries per step and tire: up to numColumns * numStations = 45, one coarse gate
    // of 5 when airborne.
    ChDiscreteTire::AccessParams().numTread = m_setup.NumTread;
    ChDiscreteTire::AccessParams().numStrips = m_setup.NumStrips;
    ChDiscreteTire::AccessParams().numColumns = m_setup.NumColumns;
    ChDiscreteTire::AccessParams().numStations = m_setup.NumStations;

    // Tread friction: peak friction relative to the terrain friction value. With the terrain friction of asphalt
    // (0.8) the peak of the slip curve is about 1.0 (TMeasy: 1.13), the sliding friction about 0.9 of it. 1.70 is
    // the street tire, GripScale makes the rubber stickier (race) or less sticky (offroad).
    ChDiscreteTire::AccessParams().gripScale = 1.70 * m_setup.GripScale;

    // Standstill damping. The bristle damper (5 ms) is the only tangential damping of the tire, which leaves a car on
    // the shear stiffness of its tires ringing back and forth for seconds after a hard stop (damping ratio ~0.02,
    // measured ~1.3 Hz on sand). 60 ms gives a damping ratio of roughly 0.12 to 0.25 near rest and fades out by a few
    // m/s of rolling speed. See DiscreteTireCore.h (tauDampStandstill).
    ChDiscreteTire::AccessParams().tauDampStandstill = 0.06;
}
