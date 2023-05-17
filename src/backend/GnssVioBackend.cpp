/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackend.cpp
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * A. Rosinol, M. Abate, Y. Chang, L. Carlone.
 * Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization
 * and Mapping. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2019.
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

#include "kimera-vio/backend/GnssVioBackend.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/navigation/GPSFactor.h>

#include <limits>  // for numeric_limits<>
#include <map>
#include <string>
#include <utility>  // for make_pair
#include <vector>

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"  // for safeCast
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

// DEFINE_bool(debug_graph_before_opt,
//            false,
//            "Store factor graph before optimization for later printing if the
//            " "optimization fails.");
// DEFINE_bool(process_cheirality,
//            false,
//            "Handle cheirality exception by removing problematic landmarks and
//            " "re-running optimization.");
// DEFINE_int32(max_number_of_cheirality_exceptions,
//             5,
//             "Sets the maximum number of times we process a cheirality "
//             "exception for a given optimization problem. This is to avoid too
//             " "many recursive calls to update the smoother");
// DEFINE_bool(compute_state_covariance,
//            false,
//            "Flag to compute state covariance from optimization Backend");

namespace VIO {

/* -------------------------------------------------------------------------- */
GnssVioBackend::GnssVioBackend(const BackendType& bk_type,
                               const Pose3& B_Pose_leftCam,
                               const StereoCalibPtr& stereo_calibration,
                               const BackendParams& backend_params,
                               const ImuParams& imu_params,
                               const BackendOutputParams& backend_output_params,
                               const GnssParams& gnss_params,
                               bool log_output)
    : VioBackend(bk_type,
                 B_Pose_leftCam,
                 stereo_calibration,
                 backend_params,
                 imu_params,
                 backend_output_params,
                 log_output),
      gnss_params_(gnss_params) {
  LOG(INFO) << "Create GNSS VioBackend";
}

BackendOutput::UniquePtr GnssVioBackend::spinOnce(const BackendInput& input) {
  LOG(INFO) << "GNSS spinOnce";

  return VioBackend::spinOnce(input);
}

void GnssVioBackend::addInitialPriorFactors(const FrameId& frame_id) {
  LOG(INFO) << "GNSS addInitialPriorFactors";
  VioBackend::addInitialPriorFactors(frame_id);
// Add initial gnss priors
#if (0)
  gtsam::Pose3 priorMean = gtsam::Pose3();  // prior at origin
  // Create noise models
  gtsam::SharedNoiseModel GPS_NOISE =
      gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::SharedNoiseModel PRIOR_NOISE =
      gtsam::noiseModel::Isotropic::Sigma(6, 0.25);
  new_imu_prior_and_other_factors_.push_back(
      boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          1, priorMean, PRIOR_NOISE));
#endif

  VLOG(2) << "Added initial priors for frame " << frame_id;
}

bool GnssVioBackend::addVisualInertialStateAndOptimize(
    const BackendInput& input) {
  LOG(INFO) << "GNSS addVisualInertialStateAndOptimize";
  const GnssBackendInput& gnss_input =
      static_cast<const GnssBackendInput&>(input);
  bool use_stereo_btw_factor =
      backend_params_.addBetweenStereoFactors_ &&
      input.stereo_tracking_status_ == TrackingStatus::VALID;

  LOG(INFO) << "VioBackend::addVisualInertialStateAndOptimize(const "
               "BackendInput& input)";
  VLOG(10) << "Add visual inertial state and optimize.";
  VLOG_IF(10, use_stereo_btw_factor) << "Using stereo between factor.";
  LOG_IF(WARNING,
         use_stereo_btw_factor && input.stereo_ransac_body_pose_ == boost::none)
      << "User set useStereoBetweenFactor = true, but stereo_ransac_body_pose_ "
         "not available!";
  CHECK(input.status_stereo_measurements_kf_);
  CHECK(input.pim_);
  gtsam::FactorIndices extra_factor_slots_to_delete;
  addVisualInertialState(
      input.timestamp_,  // Current time for fixed lag smoother.
      *input.status_stereo_measurements_kf_,  // Vision data.
      *input.pim_,                            // Imu preintegrated data.
      extra_factor_slots_to_delete,
      use_stereo_btw_factor
          ? input.stereo_ransac_body_pose_
          : boost::none);  // optional: pose estimate from stereo ransac

  // add gps factor last_kf_id_ curr_kf_id_
  LOG(INFO) << "Gnss data pos=" << gnss_input.gnss_nav_data_->nav_pos_
            << " defined="
            << (!gnss_input.gnss_nav_data_->undefined_ ? "yes" : "no");
  if (!gnss_input.gnss_nav_data_->undefined_) {
    last_kf_id_ = curr_kf_id_;
    ++curr_kf_id_;
    LOG(INFO) << "add GNSS factor";
    gtsam::Point3 p3 = {gnss_input.gnss_nav_data_->nav_pos_.x(),
                        gnss_input.gnss_nav_data_->nav_pos_.y(),
                        gnss_input.gnss_nav_data_->nav_pos_.z()};
    gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::identity(), p3);
    if (new_values_.find(gtsam::Symbol(kPoseSymbolChar, last_kf_id_)) !=
        new_values_.end()) {
      gtsam::Pose3 p =
          new_values_.at<Pose3>(gtsam::Symbol(kPoseSymbolChar, last_kf_id_));
      pose = gtsam::Pose3(p.rotation(), p3);
      LOG(INFO) << "Gnss data rot=" << p.rotation();
    }
    gtsam::Vector6 noise_sigmas;
    noise_sigmas.setConstant(0.25);
    gtsam::SharedNoiseModel GNSS_NOISE =
        gtsam::noiseModel::Diagonal::Sigmas(noise_sigmas);
    new_imu_prior_and_other_factors_.addPrior(
        gtsam::Symbol(kPoseSymbolChar, curr_kf_id_), pose, GNSS_NOISE);
    new_values_.insert(gtsam::Symbol(kPoseSymbolChar, curr_kf_id_), pose);

    if (new_values_.find(gtsam::Symbol(kVelocitySymbolChar, last_kf_id_)) !=
        new_values_.end()) {
      Vector3 vel = new_values_.at<Vector3>(
          gtsam::Symbol(kVelocitySymbolChar, last_kf_id_));
      new_values_.insert(gtsam::Symbol(kVelocitySymbolChar, curr_kf_id_), vel);
    }
    if (new_values_.find(gtsam::Symbol(kImuBiasSymbolChar, last_kf_id_)) !=
        new_values_.end()) {
      ImuBias imu = new_values_.at<gtsam::imuBias::ConstantBias>(
          gtsam::Symbol(kImuBiasSymbolChar, last_kf_id_));
      new_values_.insert(gtsam::Symbol(kImuBiasSymbolChar, curr_kf_id_), imu);
    }
  }

  bool is_smoother_ok = optimize(input.timestamp_,
                                 curr_kf_id_,
                                 backend_params_.numOptimize_,
                                 extra_factor_slots_to_delete);
  postOptimize(is_smoother_ok);
  // Bookkeeping
  timestamp_lkf_ = input.timestamp_;
  return is_smoother_ok;
}

}  // namespace VIO.
