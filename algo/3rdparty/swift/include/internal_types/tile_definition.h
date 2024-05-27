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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_TILE_DEFINITION_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_TILE_DEFINITION_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <internal_types/common.h>

namespace swift {
namespace internal {

struct TileFrame {
  double corner_nw_lat = 0.0;
  double corner_nw_lon = 0.0;

  double spacing_lat = 0.0;
  double spacing_lon = 0.0;

  uint16_t rows = 0;
  uint16_t cols = 0;
  uint64_t bitmask = 0;
};

struct TileDefinitionCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_atmo = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint16_t update_interval_s = 0;
  TileFrame frame;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_TILE_DEFINITION_H
