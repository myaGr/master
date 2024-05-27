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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_FLAGS_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_FLAGS_H

#include <swift/config.h>
#include <swift/inputs.h>

#include <internal_types/common.h>
#include <internal_types/static_buffer.h>

#include <array>
#include <cstdint>

namespace swift {
namespace internal {

enum class UsageFlag : uint8_t { NOMINAL, WARNING, ALERT, NOT_MONITORED };

enum class LowLvlFlagIdx : uint8_t {
  GPS = 0,
  GAL,
  BDS,
  TROPO_GRID,
  IONO_GRID,
  IONO_TILE_LOS,
  IONO_GRID_LOS,
  LOW_LVL_FLAGS_COUNT
};

using LowLvlFlagsArray =
    std::array<UsageFlag,
               static_cast<std::size_t>(LowLvlFlagIdx::LOW_LVL_FLAGS_COUNT)>;

struct FlagsHighLevel {
  Timestamp obs_timestamp;
  Timestamp corr_timestamp;

  uint8_t solution_id = 0;

  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint8_t chain_id = 0;

  LowLvlFlagsArray use_flags{
      {UsageFlag::NOT_MONITORED, UsageFlag::NOT_MONITORED,
       UsageFlag::NOT_MONITORED, UsageFlag::NOT_MONITORED,
       UsageFlag::NOT_MONITORED, UsageFlag::NOT_MONITORED,
       UsageFlag::NOT_MONITORED}};
};

struct FlagsSatellites {
  bool complete = false;

  Timestamp obs_timestamp;

  uint8_t solution_id = 0;
  uint8_t chain_id = 0;

  GnssId constellation = GnssId::GPS;
  std::size_t faulty_satellites_count = 0;
  std::array<uint8_t, cMaxSats> faulty_satellite_ids{};
};

struct FlagsGridPoints {
  bool complete = false;

  Timestamp obs_timestamp;
  uint8_t solution_id = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint8_t chain_id = 0;
  std::size_t faulty_grid_points_count = 0;
  std::array<uint16_t, cMaxGridPoints> faulty_grid_point_indices{};
};

struct FlagsIonoTileLos {
  bool complete = false;

  Timestamp obs_timestamp;

  uint8_t solution_id = 0;
  uint8_t chain_id = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;

  uint8_t faulty_los_count = 0;
  std::array<SatelliteDescription, cMaxSats> faulty_los{};
};

struct FlagsIonoGridLos {
  bool complete = false;

  Timestamp obs_timestamp;

  uint8_t solution_id = 0;
  uint8_t chain_id = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;

  std::array<StaticBuffer<SatelliteDescription, cMaxSats>, cMaxGridPoints>
      faulty_grid_point_los{};
};

struct FlagsIonoGridPointLos {
  bool complete = false;

  Timestamp obs_timestamp;

  uint8_t solution_id = 0;
  uint8_t chain_id = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint16_t grid_point_idx = 0;

  std::size_t faulty_los_count = 0;
  std::array<SatelliteDescription, cMaxSats> faulty_los{};
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_FLAGS_H
