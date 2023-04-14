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

#include "kimera-vio/frontend/GnssStereoImuSyncPacket.h"

#include <utility>

namespace VIO {

GnssStereoImuSyncPacket::GnssStereoImuSyncPacket(
    const StereoFrame& stereo_frame,
    const ImuStampS& imu_stamps,
    const ImuAccGyrS& imu_accgyrs,
    const Gnss& gnss_data,
    const ReinitPacket& reinit_packet)
    : StereoImuSyncPacket(stereo_frame, imu_stamps, imu_accgyrs, reinit_packet),
      gnss_data_(gnss_data) {
  // The timestamp of the last IMU measurement must correspond to the timestamp
  // of the stereo frame. In case there is no IMU measurement with exactly
  // the same timestamp as the stereo frame, the user should interpolate
  // IMU measurements to get a value at the time of the stereo_frame.
}

GnssStereoImuSyncPacket::GnssStereoImuSyncPacket(
    const StereoImuSyncPacket::UniquePtr&& sis_packet,
    const Gnss& gnss_data)
    : StereoImuSyncPacket(sis_packet->getStereoFrame(),
                          sis_packet->getImuStamps(),
                          sis_packet->getImuAccGyrs(),
                          sis_packet->getReinitPacket()),
      gnss_data_(gnss_data) {}

}  // namespace VIO
