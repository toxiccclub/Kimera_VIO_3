/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssStereoImuPipeline.h
 * @brief  Implements Gnss StereoVIO pipeline workflow.
 * @author Igor Lovets
 */

#pragma once

//#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
//#include "kimera-vio/frontend/StereoCamera.h"
//#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"

namespace VIO {

class GnssStereoImuPipeline : public StereoImuPipeline {
 public:
  KIMERA_POINTER_TYPEDEFS(GnssStereoImuPipeline);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GnssStereoImuPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief StereoImuPipeline
   * @param params Vio parameters
   * @param visualizer Optional visualizer for visualizing 3D results
   * @param displayer Optional displayer for visualizing 2D results
   */
  GnssStereoImuPipeline(const VioParams& params,
                        Visualizer3D::UniquePtr&& visualizer = nullptr,
                        DisplayBase::UniquePtr&& displayer = nullptr);

  ~GnssStereoImuPipeline() = default;

 public:
};

}  // namespace VIO
