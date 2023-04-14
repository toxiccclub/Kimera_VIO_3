/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a single image
 * @author Luca Carlone
 */

#pragma once

#include <glog/logging.h>
#include <gtsam/base/Matrix.h>
#include <stdio.h>
#include <stdlib.h>

#include <boost/foreach.hpp>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {

////////////////////////////////////////////////////////////////////////////
// Class for storing/processing a single image
class Gnss : public PipelinePayload {
 public:
  // KIMERA_DELETE_COPY_CONSTRUCTORS(Gnss);
  KIMERA_POINTER_TYPEDEFS(Gnss);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructors.
  /// @param img: does a shallow copy of the image by defaults,
  ///  if Frame should have ownership of the image, clone it.
  Gnss(const Timestamp& timestamp, const gtsam::Vector3& pos)
      : PipelinePayload(timestamp), nav_pos_(pos) {}

  Gnss(const Timestamp& timestamp, const gtsam::Vector3& pos, bool undefined)
      : PipelinePayload(timestamp), nav_pos_(pos), undefined_(undefined_) {}

  // TODO(TONI): delete all copy constructors!!
  // Look at the waste of time this is :O
  Gnss(const Gnss& gnss)
      : PipelinePayload(gnss.timestamp_),
        nav_pos_(gnss.nav_pos_),
        undefined_(gnss.undefined_) {}

 public:
  /* ------------------------------------------------------------------------ */
  void print() const {
    LOG(INFO) << "Gnss position at timestamp: " << timestamp_ << "\n"
              << "pos_: " << nav_pos_ << "\n"
              << "undefined_: " << undefined_;
  }

 public:
  const gtsam::Vector3 nav_pos_;
  bool undefined_ = true;
};

}  // namespace VIO
