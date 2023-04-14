/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Stereo Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/frontend/Gnss.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"

namespace VIO {

class GnssStereoImuSyncPacket : public StereoImuSyncPacket {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoImuSyncPacket);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoImuSyncPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GnssStereoImuSyncPacket() = delete;
  GnssStereoImuSyncPacket(const StereoFrame& stereo_frame,
                          const ImuStampS& imu_stamps,
                          const ImuAccGyrS& imu_accgyr,
                          const Gnss& gnss_data,
                          const ReinitPacket& reinit_packet = ReinitPacket());
  GnssStereoImuSyncPacket(const StereoImuSyncPacket::UniquePtr&& sis_packet,
                          const Gnss& gnss_data);
  ~GnssStereoImuSyncPacket() = default;

  inline const Gnss& getGnssData() const { return gnss_data_; }

 private:
  const Gnss gnss_data_;
};

}  // namespace VIO
