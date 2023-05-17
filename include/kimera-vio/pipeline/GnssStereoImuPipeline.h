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

//#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/dataprovider/GnssStereoDataProviderModule.h"
#include "kimera-vio/frontend/Gnss.h"
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
  inline void fillGnssQueue(Gnss::UniquePtr gnss_nav_data) {
    CHECK(data_provider_module_);
    CHECK(gnss_nav_data);
    // StereoDataProviderModule::UniquePtr stereo_dataprovider =
    //     VIO::safeCast<MonoDataProviderModule, StereoDataProviderModule>(
    //         std::move(data_provider_module_));
    // stereo_dataprovider->fillRightFrameQueue(std::move(right_frame));
    // data_provider_module_ =
    //     VIO::safeCast<StereoDataProviderModule, MonoDataProviderModule>(
    //         std::move(stereo_dataprovider));

    // TODO(marcus): this is not a good solution. The problem is the above code
    // doesn't work in online/parallel because other threads are accessing
    // data_provider_module_ when it's been temporarily released to the stereo
    // version. Checks fail for that reason.
    // This fix is really bad because it totally bypasses the rules of
    // unique_ptr
    dynamic_cast<GnssStereoDataProviderModule*>(data_provider_module_.get())
        ->fillGnssQueue(std::move(gnss_nav_data));
  }
  inline void fillGnssQueueBlockingIfFull(Gnss::UniquePtr gnss_nav_data) {
    CHECK(data_provider_module_);
    CHECK(gnss_nav_data);
    // StereoDataProviderModule::UniquePtr stereo_dataprovider =
    //     VIO::safeCast<MonoDataProviderModule, StereoDataProviderModule>(
    //         std::move(data_provider_module_));
    // stereo_dataprovider->fillRightFrameQueueBlockingIfFull(
    //     std::move(right_frame));
    // data_provider_module_ =
    //     VIO::safeCast<StereoDataProviderModule, MonoDataProviderModule>(
    //         std::move(stereo_dataprovider));

    // TODO(marcus): this is not a good solution. The problem is the above code
    // doesn't work in online/parallel because other threads are accessing
    // data_provider_module_ when it's been temporarily released to the stereo
    // version. Checks fail for that reason.
    // This fix is really bad because it totally bypasses the rules of
    // unique_ptr
    dynamic_cast<GnssStereoDataProviderModule*>(data_provider_module_.get())
        ->fillGnssQueueBlockingIfFull(std::move(gnss_nav_data));
  }
};

}  // namespace VIO
