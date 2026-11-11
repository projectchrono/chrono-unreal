// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono-Vehicle configuration header file
//
// NOTE (Unreal integration): this file is the hand-written equivalent of the
// CMake configure_file() output of ChConfigVehicle.h.in for Chrono 10.0.0 with
// OpenCRG support disabled (CH_ENABLE_OPENCRG=OFF). See Plugins/ChronoCore/README.md.
//
// =============================================================================

#ifndef CH_CONFIG_VEHICLE_H
#define CH_CONFIG_VEHICLE_H

// Include main Chrono configuration header
#include "chrono/ChConfig.h"

// -----------------------------------------------------------------------------

// If OpenCRG support was enabled, define CHRONO_HAS_OPENCRG
#undef CHRONO_HAS_OPENCRG

#endif
