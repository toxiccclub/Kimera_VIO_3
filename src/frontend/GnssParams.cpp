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

#include "kimera-vio/frontend/GnssParams.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

GnssParams::GnssParams() : PipelineParams("GNSS params") {}

bool GnssParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  std::vector<double> vector_pose;
  yaml_parser.getNestedYamlParam("Pose", "data", &vector_pose);
  b_pose_ = UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);

  double rate_hz = 0.0;
  yaml_parser.getYamlParam("rate_hz", &rate_hz);
  CHECK_GT(rate_hz, 0.0);
  nominal_sampling_time_s_ = 1.0 / rate_hz;
  yaml_parser.getYamlParam("noise", &noise_);

  return true;
}

void GnssParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "b_pose_: ",
                        b_pose_,
                        "nominal_sampling_time_s_: ",
                        nominal_sampling_time_s_,
                        "noise_: ",
                        noise_);
  LOG(INFO) << out.str();
}

bool GnssParams::equals(const PipelineParams& obj) const {
  const auto& rhs = static_cast<const GnssParams&>(obj);
  // clang-format off
  return b_pose_.equals(rhs.b_pose_) &&
      noise_ == rhs.noise_ &&
      nominal_sampling_time_s_ == rhs.nominal_sampling_time_s_;
  // clang-format on
}

}  // namespace VIO
