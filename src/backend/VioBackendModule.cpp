/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackendModule.cpp
 * @brief  Pipeline module for the Backend.
 *
 * @author Antoni Rosinol
 */

#include "kimera-vio/backend/VioBackendModule.h"

#include "kimera-vio/backend/GnssVioBackend-definitions.h"
#include "kimera-vio/backend/GnssVioBackend.h"

namespace VIO {

VioBackendModule::VioBackendModule(InputQueue* input_queue,
                                   bool parallel_run,
                                   VioBackend::UniquePtr vio_backend)
    : SIMO(input_queue, "VioBackend", parallel_run),
      vio_backend_(std::move(vio_backend)) {
  CHECK(vio_backend_);
}

VioBackendModule::OutputUniquePtr VioBackendModule::spinOnce(
    BackendInput::UniquePtr input) {
  CHECK(input);
  CHECK(vio_backend_);
  LOG(INFO) << "VioBackendModule::spinOnce";
  OutputUniquePtr output;
  output = vio_backend_->spinOnce(*input);
  //  switch (vio_backend_->getBackendType()) {
  //    case BackendType::kStereoImu:
  //    case BackendType::kStructuralRegularities:
  //      output = vio_backend_->spinOnce(*input);
  //      break;
  //    case BackendType::kGnssStereoImu: {
  //      //      GnssBackendInput::UniquePtr gnss_input =
  //      //          VIO::safeCast<BackendInput,
  //      //          GnssBackendInput>(std::move(input));
  //      GnssVioBackend::UniquePtr gnss_vo_backend =
  //          VIO::safeCast<VioBackend,
  //          GnssVioBackend>(std::move(vio_backend_));
  //      CHECK(gnss_vo_backend);
  //      output = gnss_vo_backend->spinOnce(*input);
  //      vio_backend_ = std::move(gnss_vo_backend);
  //    } break;
  //  }
  if (!output) {
    LOG(ERROR) << "Backend did not return an output: shutting down Backend.";
    shutdown();
  }
  return output;
}

void VioBackendModule::registerImuBiasUpdateCallback(
    const VioBackend::ImuBiasCallback& imu_bias_update_callback) {
  CHECK(vio_backend_);
  vio_backend_->registerImuBiasUpdateCallback(imu_bias_update_callback);
}

}  // namespace VIO
