/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Gnss-definitions.h
 * @brief  Definitions for Gnss.h
 * @author Igor Lovets
 */

#pragma once

#include <glog/logging.h>
#include <gtsam/base/Matrix.h>

#include <Eigen/Dense>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/Gnss.h"

namespace VIO {

// Gnss containers.
using GnssStamp = Timestamp;
using GnssStampS = Eigen::Matrix<GnssStamp, 1, Eigen::Dynamic>;

struct GnssMeasurement {
  Gnss nav_data;
  GnssStamp timestamp;
};

// Multiple Imu measurements, bundled in dynamic matrices.
struct GnssMeasurements {};

}  // namespace VIO
