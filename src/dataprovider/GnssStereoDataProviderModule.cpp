/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoDataProviderModule.cpp
 * @brief  Pipeline Module that takes care of providing data to the VIO
 * pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"

#include <glog/logging.h>

#include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"

namespace VIO {

GnssStereoDataProviderModule::GnssStereoDataProviderModule(
    OutputQueue* output_queue,
    const std::string& name_id,
    const bool& parallel_run,
    const StereoMatchingParams& stereo_matching_params)
    : StereoDataProviderModule(output_queue,
                               name_id,
                               parallel_run,
                               stereo_matching_params),
      gnss_queue_("data_provider_gnss_data") {}

GnssStereoDataProviderModule::InputUniquePtr
GnssStereoDataProviderModule::getInputPacket() {
  //! Get left image + IMU data
  send_packet_ = false;
  StereoDataProviderModule::InputUniquePtr packet =
      StereoDataProviderModule::getInputPacket();

  Gnss::UniquePtr gnss_data_payload = nullptr;
  if (!MISO::syncQueue(packet->timestamp_, &gnss_queue_, &gnss_data_payload)) {
    gnss_data_payload =
        VIO::make_unique<Gnss>(packet->timestamp_, gtsam::Vector3(), true);
  }
  CHECK(gnss_data_payload);

  Gnss data(gnss_data_payload->timestamp_,
            gnss_data_payload->nav_pos_,
            gnss_data_payload->undefined_);

  StereoImuSyncPacket::UniquePtr vio_packet =
      VIO::safeCast<FrontendInputPacketBase, StereoImuSyncPacket>(
          std::move(packet));

  auto gnss_packet =
      VIO::make_unique<GnssStereoImuSyncPacket>(std::move(vio_packet), data);

  if (!shutdown_) {
    CHECK(vio_pipeline_callback_);
    vio_pipeline_callback_(std::move(gnss_packet));
  }

  return nullptr;
}

void GnssStereoDataProviderModule::shutdownQueues() {
  gnss_queue_.shutdown();
  StereoDataProviderModule::shutdownQueues();
}

}  // namespace VIO
