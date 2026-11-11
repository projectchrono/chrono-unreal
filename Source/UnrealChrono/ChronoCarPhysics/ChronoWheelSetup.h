#pragma once

#include <array>
#include <string>

// Wheel (rim) and tire description handed to the Chrono vehicle at construction time.
//
// Filled by UWheelsComponent (Unreal, edit time) and consumed by the TestCar_Wheel / _TMeasyTire /
// _RigidTire / _DiscreteTire classes, so none of the tire data is hardcoded in those classes any more. Plain
// data only (SI units: m, kg, kg m^2, N, Pa) so it can be included from Unreal headers and Chrono sources alike.
//
// Which fields are used depends on the tire model, see the group comments. The defaults are the stock street
// wheel of the test car.
struct FChWheelSetup
{
	// ---- Rim (ChWheel, all tire models) ----
	double RimRadius = 0.2794;                                   // [m]
	double RimWidth = 0.2413;                                    // [m]
	double RimMass = 12.42;                                      // [kg]
	std::array<double, 3> RimInertia{ 0.24, 0.42, 0.24 };        // [kg m^2]

	// ---- Grip, all tire models. 1 = the street tire, > 1 stickier rubber ----
	double GripScale = 1.0;

	// ---- Tire carcass (TMeasy and Discrete) ----
	double TireWidth = 0.2667;                                   // [m]
	double TireSectionHeight = 0.1143;                           // [m] sidewall height above the rim
	double TireMass = 37.0;                                      // [kg]
	std::array<double, 3> TireInertia{ 3.84, 6.68, 3.84 };       // [kg m^2]

	// Size based parameter guess (TMeasy and Discrete)
	double RatedLoad = 9071.17;                                  // [N] tire load at the load index
	double RatedPressure = 290000.0;                             // [Pa] inflation pressure at the load index
	double InflationPressure = 358500.0;                         // [Pa] inflation pressure in use
	double DampingRatio = 0.5;                                   // []

	// ---- TMeasy only ----
	double RollingResistance = 0.015;                            // []

	// ---- Discrete only: tread discretization ----
	int NumTread = 120;                                          // tread elements around the circumference
	int NumStrips = 5;                                           // lateral strips across the tread
	int NumColumns = 3;                                          // lateral columns of ground samples
	int NumStations = 15;                                        // ground samples along the rolling direction (odd)

	// ---- Rigid only (cylinder or mesh contact) ----
	double RigidRadius = 0.38;                                   // [m]
	double RigidWidth = 0.2413;                                  // [m]
	double RigidMass = 33.06;                                    // [kg]
	std::array<double, 3> RigidInertia{ 3.84, 6.69, 3.84 };      // [kg m^2]
	double RigidFriction = 0.9;                                  // [] before GripScale
	double RigidRestitution = 0.1;                               // []
	double RigidYoungModulus = 2.0e7;                            // [Pa]

	// Unloaded tire radius implied by the size description [m]
	double GetTireRadius() const { return RimRadius + TireSectionHeight; }
};
