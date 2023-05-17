/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackend.h
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial
 * Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying
 * Perspective based on Smart Factors. In IEEE Intl. Conf. on Robotics and
 * Automation (ICRA), 2014.
 *
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#pragma once

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "kimera-vio/backend/VioBackend.h"

//#include "kimera-vio/backend/VioBackend-definitions.h"
//#include "kimera-vio/backend/VioBackendParams.h"
//#include "kimera-vio/factors/PointPlaneFactor.h"
//#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
//#include "kimera-vio/imu-frontend/ImuFrontend.h"
//#include "kimera-vio/initial/InitializationFromImu.h"
//#include "kimera-vio/logging/Logger.h"
//#include "kimera-vio/utils/Macros.h"
//#include "kimera-vio/utils/ThreadsafeQueue.h"
//#include "kimera-vio/utils/UtilsGTSAM.h"
//#include "kimera-vio/utils/UtilsOpenCV.h"

#include "kimera-vio/backend/GnssVioBackend-definitions.h"
#include "kimera-vio/frontend/GnssParams.h"

namespace VIO {

// Forward-declarations
class VioNavState;

// class UnaryFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
//  gtsam::Pose3 pose_;  ///< X and Y measurements

// public:
//  UnaryFactor(Key j, gtsam::Pose3 pose, const gtsam::SharedNoiseModel& model)
//      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), pose_(pose) {}

//  gtsam::Vector evaluateError(
//      const gtsam::Pose3& q,
//      boost::optional<gtsam::Matrix&> H = boost::none) const {
//    const gtsam::Rot3& R = q.rotation();
//    if (H)
//      (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0)
//                 .finished();
//    return (gtsam::Vector(2) << q.x() - mx_, q.y() - my_).finished();
//  }
//};

class GnssVioBackend : public VioBackend {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssVioBackend);
  KIMERA_POINTER_TYPEDEFS(GnssVioBackend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief GnssVioBackend Constructor. Initialization must be done separately.
   * @param B_Pose_leftCam Pose from IMU Body (Backend's reference pose) to
   * left camera.
   * @param stereo_calibration Stereo Calibration data for stereo factors.
   * @param backend_params Parameters for Backend.
   * @param log_output Whether to log to CSV files the Backend output.
   */
  GnssVioBackend(const BackendType& bk_type,
                 const Pose3& B_Pose_leftCam,
                 const StereoCalibPtr& stereo_calibration,
                 const BackendParams& backend_params,
                 const ImuParams& imu_params,
                 const BackendOutputParams& backend_output_params,
                 const GnssParams& gnss_params,
                 bool log_output);

  virtual ~GnssVioBackend() { LOG(INFO) << "Backend destructor called."; }

 public:
  virtual BackendOutput::UniquePtr spinOnce(const BackendInput& input) override;
  // Add initial prior factors.
  virtual void addInitialPriorFactors(const FrameId& frame_id) override;
  virtual bool addVisualInertialStateAndOptimize(
      const BackendInput& input) override;
  //  virtual bool addVisualInertialStateAndOptimize(BackendInput &input)
  //  override;

 protected:
  GnssParams gnss_params_;
};

}  // namespace VIO
