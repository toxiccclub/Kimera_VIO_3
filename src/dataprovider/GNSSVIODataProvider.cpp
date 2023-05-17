/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   EurocDataProvider.h
 * @brief  Parse EUROC dataset.
 * @author Antoni Rosinol
 * @author Yun Chang
 * @author Luca Carlone
 */

#include "kimera-vio/dataprovider/GNSSVIODataProvider.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <nmeaparse/nmea.h>

#include <algorithm>  // for max
#include <fstream>
#include <map>
#include <regex>
#include <string>
#include <utility>  // for pair<>
#include <vector>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/YamlParser.h"

DEFINE_int32(gnss_data_type,
             0,
             "0 - Data from local file in cartesian\n 1 - Data from local file "
             "in nmea format");

enum GnssInputDataType { GIDT_LOCAL = 0, GIDT_NMEA = 1 };

using namespace nmea;

namespace VIO {

GNSSVIODataProvider::GNSSVIODataProvider(const std::string& dataset_path,
                                         const int& initial_k,
                                         const int& final_k,
                                         const VioParams& vio_params)
    : EurocDataProvider(dataset_path, initial_k, final_k, vio_params) {}

GNSSVIODataProvider::GNSSVIODataProvider(const VioParams& vio_params)
    : EurocDataProvider(vio_params) {
  parseGnss(dataset_path_, "");
}

bool GNSSVIODataProvider::spin() {
  if (gnss_callback_) {
    for (auto& m : gnss_measurements_) {
      gnss_callback_(VIO::make_unique<Gnss>(m.nav_data));
    }
  }
  return EurocDataProvider::spin();
}

void GNSSVIODataProvider::parseNMEAFile(const std::string& input_dataset_path,
                                        const std::string& gnssName) {
  std::string filename_data = input_dataset_path + "/mav0/gnss0/data.csv";
  std::ifstream fin(filename_data.c_str());
  LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << filename_data;

  NMEAParser parser;
  GPSService gps(parser);
  // (optional) Called when a sentence is valid syntax
  parser.onSentence += [](const NMEASentence& nmea) {
    std::cout << "Received $" << nmea.name << std::endl;
  };
  // (optional) Called when data is read/changed
  gps.onUpdate += [&]() {
    // There are *tons* of GPSFix properties
    if (gps.fix.locked()) {
      std::cout << " # Position: " << gps.fix.latitude << ", "
                << gps.fix.longitude << std::endl;
    } else {
      std::cout << " # Searching..." << std::endl;
    }
  };
  // Send in a log file or a byte stream
  while (fin.good()) {
    uint8_t buf[1024];
    memset(buf, 0, 1024);
    fin.getline((char*)buf, 1023);
    try {
      parser.readBuffer(buf, 1024);
    } catch (NMEAParseError&) {
      // Syntax error, skip
    }
  }
}

void GNSSVIODataProvider::parseLocalFile(const std::string& input_dataset_path,
                                         const std::string& gnssName) {
  std::string filename_data = input_dataset_path + "/mav0/gnss0/data.csv";
  std::ifstream fin(filename_data.c_str());
  LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << filename_data;

  const std::regex r(
      R"((\d+),(-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+),-?\d+.\d+,-?\d+.\d+,-?\d+.\d+,-?\d+.\d+,-?\d+.\d+,-?\d+.\d+,-?\d+.\d+,-?\d+.\d+,-?\d+.\d+)");
  while (fin.good() && !fin.eof()) {
    char buf[1024];
    memset(buf, 0, 1024);
    fin.getline(buf, 1023);
    std::smatch match;
    std::string str(buf);
    if (std::regex_search(str, match, r)) {
      Gnss nav_data(atol(match[1].str().c_str()),
                    {atof(match[2].str().c_str()),
                     atof(match[3].str().c_str()),
                     atof(match[4].str().c_str())},
                    false);
      GnssMeasurement meas = {nav_data, atol(match[1].str().c_str())};
      gnss_measurements_.push_back(meas);
    }
  }
}

void GNSSVIODataProvider::parseGnss(const std::string& input_dataset_path,
                                    const std::string& gnssName) {
  switch (FLAGS_gnss_data_type) {
    case GIDT_LOCAL: {
      parseLocalFile(input_dataset_path, gnssName);
    } break;
    case GIDT_NMEA: {
      parseNMEAFile(input_dataset_path, gnssName);
    } break;
    default: {
    }
  }
}

}  // namespace VIO
