/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontendModule.h
 * @brief  Pipeline module for the vision Frontend.
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/frontend/VisionImuFrontendModule.h"
#include "kimera-vio/pipeline/PipelineModule.h"

namespace VIO {

class GnssVisionImuFrontendModule : public VisionImuFrontendModule {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssVisionImuFrontendModule);
  KIMERA_POINTER_TYPEDEFS(GnssVisionImuFrontendModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief VisionImuFrontendModule
   * @param input_queue
   * @param output_queue
   * @param parallel_run
   * @param vio_frontend
   */
  explicit GnssVisionImuFrontendModule(
      InputQueue* input_queue,
      bool parallel_run,
      VisionImuFrontend::UniquePtr vio_frontend);

  virtual ~GnssVisionImuFrontendModule() = default;

 private:
  VisionImuFrontend::UniquePtr vio_frontend_;
};

}  // namespace VIO
