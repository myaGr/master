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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_GRID_LOCALITY_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_GRID_LOCALITY_CALCULATOR_H

#include <swift/outputs.h>

#include <internal_types/common.h>
#include <internal_types/flags.h>
#include <internal_types/tile_definition.h>
#include <los_calculator/iono_calculator.h>
#include <storage_areas/active_area.h>

#include <array>

namespace swift {
namespace internal {

class GridLocalityCalculator {
 public:
  GridLocalityCalculator() = default;

  /**
   * @brief Calculate a grid locality from a tile definition and rover position.
   *
   * The grid locality contains information about the position and index of the
   * surrounding grid points. And also information of how the tile and rover
   * position are related to them.
   *
   * @param rover_pos Rover position in ECEF meters.
   * @param tile_definition Tile definition to be used. Must use squared
   * spacing.
   * @param grid_locality Created grid locality output.
   *
   * @return ReturnCode.
   */
  ReturnCode calculate_grid_locality(
      const std::array<double, 3> &rover_pos_ecef,
      const TileDefinition &tile_definition, GridLocality *grid_locality) const;

  /**
   * @brief Get the index of the corner closest to the unit_coord
   * in a grid locality object
   *
   * The unit_coord_east and unit_coord_south describes the location
   * between the four surrounding points in the grid.
   * This returns the index of the closest corner
   * (either corner_idx_nw, corner_idx_ne, corner_idx_sw, corner_idx_se).
   *
   * @param grid_locality Object describing the grid locality
   * @return Index of the grid point closest to the unit coord.
   */
  std::size_t get_closest_corner_idx(const GridLocality &grid_locality) const;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_GRID_LOCALITY_CALCULATOR_H
