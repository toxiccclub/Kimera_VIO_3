/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend.cpp
 * @brief  Class describing a stereo tracker
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/frontend/GnssStereoVisionImuFrontend.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

GnssStereoVisionImuFrontend::GnssStereoVisionImuFrontend(
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const FrontendParams& frontend_params,
    const StereoCamera::ConstPtr& stereo_camera,
    const GnssParams& gnss_params,
    DisplayQueue* display_queue,
    bool log_output)
    : StereoVisionImuFrontend(imu_params,
                              imu_initial_bias,
                              frontend_params,
                              stereo_camera,
                              display_queue,
                              log_output) {}

GnssStereoVisionImuFrontend::~GnssStereoVisionImuFrontend() {
  LOG(INFO) << "GnssStereoVisionImuFrontend destructor called.";
}

FrontendOutputPacketBase::UniquePtr GnssStereoVisionImuFrontend::bootstrapSpin(
    FrontendInputPacketBase::UniquePtr&& input) {
  CHECK(frontend_state_ == FrontendState::Bootstrap);
  CHECK(input);
  GnssStereoFrontendInputPayload::UniquePtr gnss_vio_input =
      VIO::safeCast<FrontendInputPacketBase, GnssStereoFrontendInputPayload>(
          std::move(input));
  auto gnss_nav_data = gnss_vio_input->getGnssData();
  auto vio_output_base =
      StereoVisionImuFrontend::bootstrapSpin(std::move(gnss_vio_input));
  StereoFrontendOutput::UniquePtr vio_output =
      VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
          std::move(vio_output_base));
  return VIO::make_unique<GnssStereoFrontendOutput>(std::move(vio_output),
                                                    gnss_nav_data);
}

FrontendOutputPacketBase::UniquePtr GnssStereoVisionImuFrontend::nominalSpin(
    FrontendInputPacketBase::UniquePtr&& input) {
  CHECK(frontend_state_ == FrontendState::Nominal);
  CHECK(input);
  GnssStereoFrontendInputPayload::UniquePtr gnss_vio_input =
      VIO::safeCast<FrontendInputPacketBase, GnssStereoFrontendInputPayload>(
          std::move(input));
  auto gnss_nav_data = gnss_vio_input->getGnssData();
  auto vio_output_base =
      StereoVisionImuFrontend::nominalSpin(std::move(gnss_vio_input));
  StereoFrontendOutput::UniquePtr vio_output =
      VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(
          std::move(vio_output_base));
  return VIO::make_unique<GnssStereoFrontendOutput>(std::move(vio_output),
                                                    gnss_nav_data);
}

}  // namespace VIO
