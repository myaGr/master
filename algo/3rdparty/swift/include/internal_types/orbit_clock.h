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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_ORBIT_CLOCK_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_ORBIT_CLOCK_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <internal_types/common.h>

namespace swift {
namespace internal {

struct SsrOrbitClockSat {
  SatelliteDescription sat_id;
  uint16_t iode = 0;
  float radial = 0.0;
  float along = 0.0;
  float cross = 0.0;
  float delta_radial = 0.0;
  float delta_along = 0.0;
  float delta_cross = 0.0;
  float delta_clock_c0 = 0.0;
  float delta_clock_c1 = 0.0;
  float delta_clock_c2 = 0.0;
};

struct SsrOrbitClockCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;

  std::size_t satellites_count = 0;
  std::array<SsrOrbitClockSat, cMaxSats> satellites;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_ORBIT_CLOCK_H
