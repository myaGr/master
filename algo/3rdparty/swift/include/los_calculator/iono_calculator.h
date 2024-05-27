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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_IONO_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_IONO_CALCULATOR_H

#include <swift/config.h>
#include <swift/inputs.h>
#include <swift/outputs.h>

#include <integrity/integrity_provider.h>
#include <internal_types/atmo.h>
#include <internal_types/common.h>
#include <internal_types/flags.h>
#include <internal_types/static_buffer.h>
#include <los_calculator/common.h>
#include <storage_areas/active_area.h>

#include <cstdint>

namespace swift {
namespace internal {

class IonoCalculator {
 public:
  explicit IonoCalculator(const IntegrityConfig &integrity_config);

  /**
   * @brief Calculate LoS ionospheric corrections from SSR correction input.
   *
   * @param[in] timestamp Timestamp for which the corrections are calculated
   * for.
   * @param[in] integrity_provider Integrity provider to validate that flags for
   * the SSR corrections are nominal.
   * @param[in] sat_id Satellite constellation and number.
   * @param[in] grid_locality_set Grid locality set.
   * @param[in] iono_set Ionospheric parameters set from SSR service.
   * @param[out] component_result Output iono corrections.
   * @param[out] tile_id Tile id of the tile used to calculate the corrections.
   *
   * @return ReturnCode.
   */
  ReturnCode calculate_corrections(
      const TimestampMs &timestamp_ms,
      const IntegrityProvider &integrity_provider,
      const SatelliteDescription &sat_id,
      const StaticBuffer<GridLocality, cMaxTiles> &grid_locality_set,
      const StaticBuffer<SsrCoherentSet::Iono, cMaxTiles> &iono_set,
      LosComponentResult *component_result, TileId *tile_id) const;

 private:
  IntegrityConfig integrity_config_;
};

/**
 * @brief Calculate the Iono protection level
 * @param integrity_config Integrity configuration
 * @param timestamp_now_ms Current timestamp on milliseconds
 * @param iono Iono object
 * @param grid_locality Grid locality
 * @param iono_protection_level Calculated protection level
 * @return SUCCESS if the protection level is calculated, INTERNAL_ERROR
 * otherwise
 */
ReturnCode calculate_iono_protection_level(
    const IntegrityConfig &integrity_config,
    const TimestampMs &timestamp_now_ms, const SsrCoherentSet::Iono &iono,
    const GridLocality &grid_locality, double *const iono_protection_level);

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_IONO_CALCULATOR_H
