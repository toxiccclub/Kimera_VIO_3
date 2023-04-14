/* ----------------------------------------------------------------------------
 * Copyright 2023, SPb CVS,
 * -------------------------------------------------------------------------- */

/**
 * @file   GNSSVIODataProvider.h
 * @brief  Parse EUROC dataset format with additional info about GNSS position.
 * @author Igor Lovets
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include <map>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
//#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class GNSSVIODataProvider : public EurocDataProvider {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(GNSSVIODataProvider);
  KIMERA_POINTER_TYPEDEFS(GNSSVIODataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  GNSSVIODataProvider(const std::string& dataset_path,
                      const int& initial_k,
                      const int& final_k,
                      const VioParams& vio_params)
      : EurocDataProvider(dataset_path, initial_k, final_k, vio_params) {}
  //! Ctor from gflags
  explicit GNSSVIODataProvider(const VioParams& vio_params)
      : EurocDataProvider(vio_params) {}

  virtual ~GNSSVIODataProvider() {}

 public:
  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin() override;

 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  //  virtual bool spinOnce();
 public:
  const std::string kGnssName = "gnss0";
};

}  // namespace VIO
