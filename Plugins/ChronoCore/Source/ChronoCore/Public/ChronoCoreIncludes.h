// Umbrella header for the Project Chrono core module.
//
// Includes the commonly used Chrono core headers, safely bracketed with ChronoIncludeBegin.h / ChronoIncludeEnd.h
// so that it can be included from any Unreal translation unit - before or after Engine headers.
//
// For headers that are not part of this umbrella, wrap them yourself:
//   #include "ChronoIncludeBegin.h"
//   #include "chrono/physics/ChLinkMotorRotationSpeed.h"
//   #include "ChronoIncludeEnd.h"

#pragma once

#include "CoreMinimal.h"

#include "ChronoIncludeBegin.h"

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/core/ChApiCE.h"

// Math / kinematics
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChRotation.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChConstants.h"
#include "chrono/utils/ChUtils.h"

// Systems, bodies, links
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"

// Functions
#include "chrono/functions/ChFunction.h"
#include "chrono/functions/ChFunctionConst.h"

// Collision
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionShapes.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/collision/chaos/ChCollisionSystemChaos.h"   // Unreal Chaos backend (Type::CHAOS), see ChronoChaosCollision.h

// Visual assets (used for shape/colour metadata; Chrono does no rendering here)
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

// Solvers / integrators
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

// Utilities
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "ChronoIncludeEnd.h"

// Convenience aliases so game code does not need `using namespace chrono`.
// (chrono::ChVector3d etc. are already typedefs inside namespace chrono; these mirror them at global scope.)
using ChVector3d    = chrono::ChVector3d;
using ChVector3f    = chrono::ChVector3f;
using ChQuaterniond = chrono::ChQuaterniond;
using ChQuaternionf = chrono::ChQuaternionf;
using ChCoordsysd   = chrono::ChCoordsysd;
using ChFramed      = chrono::ChFramed;
using ChMatrix33d   = chrono::ChMatrix33d;
using ChColor       = chrono::ChColor;
