// Umbrella header for the Project Chrono vehicle module.
//
// Includes ChronoCoreIncludes.h and the commonly used chrono_vehicle headers, bracketed with
// ChronoIncludeBegin.h / ChronoIncludeEnd.h so it can be included from any Unreal translation unit.
//
// For headers that are not part of this umbrella, wrap them yourself:
//   #include "ChronoIncludeBegin.h"
//   #include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"
//   #include "ChronoIncludeEnd.h"

#pragma once

#include "ChronoCoreIncludes.h"

#include "ChronoIncludeBegin.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

// Vehicle, chassis, powertrain
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/powertrain/ChEngineSimple.h"
#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

// Drivers
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

// Terrain
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

// Wheeled vehicles
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChMultiLink.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"

// JSON-specified vehicle and subsystems
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

// Tracked vehicles
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "ChronoIncludeEnd.h"
