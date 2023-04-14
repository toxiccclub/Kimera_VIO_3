/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssParams.cpp
 * @brief  Params for GnssFrontend.
 * @author Igor Lovets
 */

#pragma once

#include <gtsam/base/Vector.h>

#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct GnssParams : public PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssParams();
  virtual ~GnssParams() = default;

 public:
  bool parseYAML(const std::string& filepath) override;
  void print() const override;

 protected:
  bool equals(const PipelineParams& obj) const override;

 public:
  double noise_ = 0.0;

  double nominal_sampling_time_s_ = 0.0;
  gtsam::Pose3 b_pose_ = gtsam::Pose3::identity();
};

}  // namespace VIO
