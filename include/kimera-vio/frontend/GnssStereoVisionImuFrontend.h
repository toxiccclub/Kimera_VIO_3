/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend.h
 * @brief  Class describing a stereo tracker
 * @author Antoni Rosinol, Luca Carlone
 */

#pragma once

#include <opencv2/highgui/highgui_c.h>

#include <atomic>
#include <boost/shared_ptr.hpp>  // used for opengv
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//#include "kimera-vio/frontend/VisionImuFrontend.h"
#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/GnssParams.h"
#include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"
#include "kimera-vio/frontend/GnssStereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {

using GnssStereoFrontendInputPayload = GnssStereoImuSyncPacket;

class GnssStereoVisionImuFrontend : public StereoVisionImuFrontend {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoVisionImuFrontend);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoVisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  GnssStereoVisionImuFrontend(const ImuParams& imu_params,
                              const ImuBias& imu_initial_bias,
                              const FrontendParams& tracker_params,
                              const StereoCamera::ConstPtr& stereo_camera,
                              const GnssParams& gnss_params,
                              DisplayQueue* display_queue = nullptr,
                              bool log_output = false);
  virtual ~GnssStereoVisionImuFrontend();

 public:
  /**
   * @brief bootstrapSpin SpinOnce used when initializing the Frontend.
   * @param input
   * @return
   */
  FrontendOutputPacketBase::UniquePtr bootstrapSpin(
      FrontendInputPacketBase::UniquePtr&& input) override;

  /**
   * @brief nominalSpin SpinOnce used when in nominal mode after initialization
   * (bootstrap)
   * @param input
   * @return
   */
  FrontendOutputPacketBase::UniquePtr nominalSpin(
      FrontendInputPacketBase::UniquePtr&& input) override;
};

}  // namespace VIO
