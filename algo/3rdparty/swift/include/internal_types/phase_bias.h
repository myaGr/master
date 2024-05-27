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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_PHASE_BIAS_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_PHASE_BIAS_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <internal_types/common.h>

namespace swift {
namespace internal {

struct SsrPhaseBias {
  uint8_t signal_code = 255;
  float bias_m = 0.0;
  bool int_indicator = false;
  uint8_t widelane_int_indicator = 0;
  uint8_t discontinuity_counter = 0;
};

struct SsrPhaseBiasesSat {
  SatelliteDescription sat_id;
  float yaw_rad = 0.0;
  float yaw_rate_rad = 0.0;  // radians per second
  std::size_t biases_count = 0;
  std::array<SsrPhaseBias, cMaxSignals> biases;
};

struct SsrPhaseBiasesCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;
  bool dispersive_bias_consistency = false;
  bool mw_consistency = false;

  std::size_t satellites_count = 0;
  std::array<SsrPhaseBiasesSat, cMaxSats> satellites;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_PHASE_BIAS_H
