#pragma once

// Common Project Chrono types used by the NextGenCars vehicle model.
//
// Chrono is compiled from source by the ChronoCore / ChronoVehicle plugins. Their umbrella headers are already
// bracketed with ChronoIncludeBegin.h / ChronoIncludeEnd.h, so this header can be included from any Unreal
// translation unit without leaking Windows / Bullet macros (min, max, PlaySound, ...) into Engine headers.
//
// Only the Chrono headers that are part of the plugin umbrellas are available through this file. A translation
// unit that needs another Chrono header wraps it itself:
//
//   #include "ChronoIncludeBegin.h"
//   #include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"
//   #include "ChronoIncludeEnd.h"
//
// Unreal headers that also include a *.generated.h must NOT include this file; forward declare the Chrono side
// and keep the Chrono includes in the .cpp.

#include "ChronoVehicleIncludes.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ChVector3d, ChVector3f, ChQuaterniond, ChQuaternionf, ChCoordsysd, ChFramed, ChMatrix33d and ChColor are already
// aliased at global scope by the plugin (ChronoCoreIncludes.h).
using chrono::ChVector3;
using chrono::ChVector2;
using chrono::ChQuaternion;
using chrono::ChMatrix44;
using chrono::ChVisualShapeCylinder;
using chrono::ChVisualShapeTriangleMesh;
using chrono::vehicle::ChPowertrainAssembly;

using Vector = chrono::ChVector3d;
using VectorF = chrono::ChVector3f;
using Quaternion = chrono::ChQuaterniond;
using QuaternionF = chrono::ChQuaternionf;

/// Enumeration of collision model types.
enum class PhysicsCollisionType
{
	NONE,        ///< no contact shapes
	PRIMITIVES,  ///< contact model composed of primitives
	HULLS,       ///< contact model composed of convex hulls
	MESH         ///< contact model composed of trimeshes
};

enum class SystemType
{
	PenaltyContacts,
	ComplementaryContacts
};

enum class PowertrainModelType
{
	SHAFTS,
	SIMPLE_MAP
};
