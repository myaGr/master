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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_COMMON_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_COMMON_H

#include <cstdint>
#include <limits>

#include <swift/inputs.h>

// This solves the rule cpp:S813 that requests to replace the use of doubles
// with a typedef
static_assert(std::numeric_limits<float>::is_iec559,
              "IEC 559 standard not supported for float type");
static_assert(std::numeric_limits<double>::is_iec559,
              "IEC 559 standard not supported for double type");

namespace swift {
namespace internal {

struct Timestamp {
  uint32_t tow_s = 0;
  uint16_t wn = 0;

  bool operator==(const Timestamp &other) const {
    return ((this->tow_s == other.tow_s) && (this->wn == other.wn));
  }

  bool operator!=(const Timestamp &other) const { return !(*this == other); }
};

struct TimestampMs {
  uint32_t tow_ms = 0;
  uint16_t wn = 0;

  bool operator==(const TimestampMs &other) const {
    return ((this->tow_ms == other.tow_ms) && (this->wn == other.wn));
  }
};

struct MeanAndStdDev {
  float mean = 0.0;
  float std_dev = 0.0;
};

struct CollectionIdentifier {
  uint16_t provider_id = 0;
  uint8_t solution_id = 0;

  bool operator==(const CollectionIdentifier &other) const {
    return ((this->provider_id == other.provider_id) &&
            (this->solution_id == other.solution_id));
  }
};

struct TileId {
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
};

struct GridId {
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint8_t solution_id = 0;
  uint8_t iod_atmo = 0;

  bool operator==(const GridId &other) const {
    return ((this->tile_set_id == other.tile_set_id) &&
            (this->tile_id == other.tile_id) &&
            (this->solution_id == other.solution_id) &&
            (this->iod_atmo == other.iod_atmo));
  }
};

enum class CollectionType : uint8_t {
  ORBIT_CLOCK,
  ORBIT_CLOCK_BOUNDS,
  ORBIT_CLOCK_DEGRADATION,
  CODE_BIAS,
  PHASE_BIAS,
  CODE_AND_PHASE_BOUNDS,
  SATELLITE_APC,
  TILE_DEFINITION,
  ATMO_GRID,
  STEC_POLYNOMIAL,
  COLLECTION_TYPE_COUNT,
};

struct CollectionIndexMap {
  CollectionType type = CollectionType::COLLECTION_TYPE_COUNT;
  std::size_t storage_index = 0;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_COMMON_H
