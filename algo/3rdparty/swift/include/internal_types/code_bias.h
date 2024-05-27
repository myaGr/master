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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CODE_BIAS_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CODE_BIAS_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <internal_types/common.h>

namespace swift {
namespace internal {

struct SsrCodeBias {
  uint8_t signal_code = 255;
  float bias_m = 0.0;
};

struct SsrCodeBiasesSat {
  SatelliteDescription sat_id;
  std::size_t biases_count = 0;
  std::array<SsrCodeBias, cMaxSignals> biases;
};

struct SsrCodeBiasesCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;

  std::size_t satellites_count = 0;
  std::array<SsrCodeBiasesSat, cMaxSats> satellites;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CODE_BIAS_H
