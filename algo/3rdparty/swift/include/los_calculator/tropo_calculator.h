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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_TROPO_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_TROPO_CALCULATOR_H

#include <swift/outputs.h>

#include <internal_types/common.h>
#include <internal_types/flags.h>
#include <los_calculator/common.h>
#include <storage_areas/active_area.h>

#include <array>

namespace swift {
namespace internal {

struct TropoCorrectionComponents {
  double M_s_dry = 0.0;
  double M_s_wet = 0.0;
  double apriori_dry_zenith_delay = 0.0;
  double apriori_wet_zenith_delay = 0.0;
  double dry_zenith_delay = 0.0;
  double wet_zenith_delay = 0.0;
};

class TropoCalculator {
 public:
  explicit TropoCalculator(const IntegrityConfig &integrity_config);

  /**
   * @brief Calculate LOS tropospheric corrections from SSR correction input.
   *
   * @param[in] timestamp_ms Timestamp for which the corrections are calculated
   * for.
   * @param[in] integrity_provider Integrity provider to validate that flags for
   * the SSR corrections are nominal.
   * @param[in] rover_pos_ecef Rover position in geodetic coordinates.
   * @param[in] sat_pva Satellite states.
   * @param[in] grid_locality_set Grid locality set.
   * @param[in] tropo_set Tropospheric parameters set from SSR service.
   *
   * @param[out] component_result Output tropospheric corrections.
   * @param[out] tile_id Tile id of the tile used to calculate the corrections.
   *
   * @return ReturnCode
   */
  ReturnCode calculate_corrections(
      const TimestampMs &timestamp_ms,
      const IntegrityProvider &integrity_provider,
      const std::array<double, 3> &rover_pos_ecef, const SatellitePva &sat_pva,
      const StaticBuffer<GridLocality, cMaxTiles> &grid_locality_set,
      const StaticBuffer<SsrCoherentSet::Tropo, cMaxTiles> &tropo_set,
      LosComponentResult *component_result, TileId *tile_id) const;

 private:
  /**
   * @brief Check the tropo correction against the apriori tropo correction and
   * complete the integrity flag and bounds accordingly
   * @param[in] to_check_zenith_correction The tropo correction to check at
   * zenith
   * @param[in] apriori_zenith_correction The apriori tropo correction at zenith
   * @param[in] elevation_rad The elevation of the satellite
   * @param[out] component_result Output tropospheric corrections updated
   * with the integrity flag and bounds
   */
  void check_integrity(const double to_check_zenith_correction,
                       const double apriori_zenith_correction,
                       const double elevation_rad,
                       LosComponentResult *const component_result) const;

  ReturnCode calculate_tropo_correction_components(
      const TimestampMs &timestamp_ms,
      const std::array<double, 3> &rover_pos_ecef,
      const double sat_elevation_rad, const SsrCoherentSet::Tropo &tropo,
      const GridLocality &grid_locality,
      TropoCorrectionComponents *const tropo_correction) const;

  IntegrityConfig integrity_config_;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_TROPO_CALCULATOR_H
