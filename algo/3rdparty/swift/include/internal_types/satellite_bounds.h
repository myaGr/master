/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_SATELLITE_BOUNDS_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_SATELLITE_BOUNDS_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <internal_types/common.h>

namespace swift {
namespace internal {

struct SsrOrbitClockBoundsSat {
  SatelliteDescription sat_id;
  MeanAndStdDev radial_bound_m;
  MeanAndStdDev along_bound_m;
  MeanAndStdDev cross_bound_m;
  MeanAndStdDev clock_bound_m;
};

struct SsrOrbitClockBoundsCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;

  std::size_t satellites_count = 0;
  std::array<SsrOrbitClockBoundsSat, cMaxSats> satellites;
};

struct SsrOrbitClockDegradationParameters {
  uint64_t bitmask = 0;
  MeanAndStdDev radial_degradation_bound_m;
  MeanAndStdDev along_degradation_bound_m;
  MeanAndStdDev cross_degradation_bound_m;
  MeanAndStdDev clock_degradation_bound_m;
};

struct SsrOrbitClockDegradation {
  GnssId constellation = GnssId::GPS;
  std::size_t parameters_count = 0;
  std::array<SsrOrbitClockDegradationParameters, cMaxSatsPerConstellation>
      parameters;
};

struct SsrOrbitClockBoundsDegradationCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;

  std::size_t degradations_count = 0;
  std::array<SsrOrbitClockDegradation, cMaxConstellations> degradations;
};

struct SsrCodeAndPhaseBiasBound {
  uint8_t signal_code = 255;
  MeanAndStdDev code_bias_m;
  MeanAndStdDev phase_bias_m;
};

struct SsrCodeAndPhaseBiasBoundsSat {
  SatelliteDescription sat_id;
  std::size_t bounds_count = 0;
  std::array<SsrCodeAndPhaseBiasBound, cMaxSignals> bounds;
};

struct SsrCodeAndPhaseBiasBoundsCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;

  std::size_t satellites_count = 0;
  std::array<SsrCodeAndPhaseBiasBoundsSat, cMaxSats> satellites;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_SATELLITE_BOUNDS_H
