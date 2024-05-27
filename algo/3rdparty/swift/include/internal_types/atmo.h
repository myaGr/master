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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_ATMO_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_ATMO_H

#include <array>
#include <cstdint>

#include <swift/config.h>
#include <swift/inputs.h>

#include <helper_functions/coord_system.h>
#include <internal_types/common.h>

namespace swift {
namespace internal {

struct TropoPoint {
  float dry_delay_m = 0.0;
  float wet_delay_m = 0.0;
  float std_dev_m = 0.0;
  MeanAndStdDev dry_bound_m;
  MeanAndStdDev wet_bound_m;
};

struct StecResiduals {
  bool valid = false;
  float bias = 0.0;
  float std_dev_m = 0.0;
  MeanAndStdDev bound_m;
  MeanAndStdDev degradation_bound_m;
};

struct StecResidualSat {
  SatelliteDescription sat_id;
  StecResiduals residuals;
};

struct AtmoGridPoint {
  uint16_t grid_point_id = 0;
  TropoPoint tropo_point;
  std::size_t satellites_count = 0;
  std::array<StecResidualSat, cMaxSats> satellites;
};

/**
 * Holding the contents of SBP MSG_SSR_GRIDDED_CORRECTION (1532)
 * Not combined with STEC for the tile as a whole
 */
struct AtmoGrid {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_atmo = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint16_t update_interval_s = 0;
  float tropo_quality_indicator_m = 0;

  std::size_t grid_points_count = 0;
  std::array<AtmoGridPoint, cMaxGridPoints> grid_points;
};

struct StecPolynomial {
  float c00 = 0.0;  // [TECU]
  float c01 = 0.0;  // [TECU/deg]
  float c10 = 0.0;  // [TECU/deg]
  float c11 = 0.0;  // [TECU/deg^2]
};

struct StecPolynomialSat {
  SatelliteDescription sat_id;
  float stec_quality_indicator_tecu = 0.0;
  StecPolynomial stec_coefficients;
};

struct StecPolynomialSatCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_atmo = 0;
  uint16_t tile_set_id = 0;
  uint16_t tile_id = 0;
  uint16_t update_interval_s = 0;

  std::size_t polynomials_count = 0;
  std::array<StecPolynomialSat, cMaxSats> polynomials;
};

struct GridLocality {
  Timestamp timestamp;
  GridId grid_id;
  uint16_t update_interval_s = 0;

  // Latitude and Longitude of the most north-west grid point in the tile
  LatLon grid_origin;

  // Four closest grid points to unit coordinate
  LatLon corner_pos_nw;
  LatLon corner_pos_ne;
  LatLon corner_pos_sw;
  LatLon corner_pos_se;

  // Mapped indexes to iono and tropo residual arrays
  std::size_t corner_idx_nw = 0;
  std::size_t corner_idx_ne = 0;
  std::size_t corner_idx_sw = 0;
  std::size_t corner_idx_se = 0;

  // Unit coordinates within above corners relative to north-west corner
  double unit_coord_east = 0.0;
  double unit_coord_south = 0.0;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_ATMO_H
