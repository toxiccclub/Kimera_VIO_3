/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssVioBackend-definitions.h
 * @brief  Definitions for GnssVioBackend.
 * @author Igor Lovets
 */

#pragma once

#include "VioBackend-definitions.h"
#include "kimera-vio/frontend/Gnss.h"

namespace VIO {

////////////////////////////////////////////////////////////////////////////////
struct GnssBackendInput : public BackendInput {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssBackendInput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssBackendInput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GnssBackendInput(
      const Timestamp& timestamp_kf_nsec,
      const StatusStereoMeasurementsPtr& status_stereo_measurements_kf,
      const TrackingStatus& stereo_tracking_status,
      const ImuFrontend::PimPtr& pim,
      //! Raw imu msgs for Backend init only
      const ImuAccGyrS& imu_acc_gyrs,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none,
      std::optional<Gnss> gnss_nav_data = std::nullopt)
      : BackendInput(timestamp_kf_nsec,
                     status_stereo_measurements_kf,
                     stereo_tracking_status,
                     pim,
                     imu_acc_gyrs,
                     stereo_ransac_body_pose),
        gnss_nav_data_(gnss_nav_data) {}

 public:
  std::optional<Gnss> gnss_nav_data_;
};

}  // namespace VIO
