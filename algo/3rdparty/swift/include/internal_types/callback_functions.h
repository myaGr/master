/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CALLBACK_FUNCTIONS_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CALLBACK_FUNCTIONS_H

#include <functional>

#include <swift/outputs.h>

#include <internal_types/atmo.h>
#include <internal_types/code_bias.h>
#include <internal_types/flags.h>
#include <internal_types/orbit_clock.h>
#include <internal_types/phase_bias.h>
#include <internal_types/satellite_apc.h>
#include <internal_types/satellite_bounds.h>
#include <internal_types/tile_definition.h>

namespace swift {
namespace internal {

struct InternalCallbackFunctions {
  std::function<void(const SsrCodeBiasesCollection &code_bias)> put_code_bias =
      nullptr;
  std::function<void(const SsrPhaseBiasesCollection &phase_bias)>
      put_phase_bias = nullptr;
  std::function<void(const SsrOrbitClockCollection &orbit_clock)>
      put_orbit_clock = nullptr;
  std::function<void(const AtmoGrid &atmo_grid)> put_atmo_grid = nullptr;
  std::function<void(const AtmoGrid &atmo_grid)> put_atmo_grid_bounds = nullptr;
  std::function<void(const StecPolynomialSatCollection &stec_polynomial)>
      put_stec_polynomial = nullptr;
  std::function<void(const TileDefinitionCollection &tile_definition)>
      put_tile_definition = nullptr;
  std::function<void(const SsrSatelliteApcCollection &sat_apc)> put_sat_apc =
      nullptr;
  std::function<void(const SsrOrbitClockBoundsDegradationCollection
                         &sat_orbit_clock_bounds_degradation)>
      put_orbit_clock_bounds_degradation = nullptr;
  std::function<void(const SsrOrbitClockBoundsCollection &orbit_clock_bounds)>
      put_orbit_clock_bounds = nullptr;
  std::function<void(
      const SsrCodeAndPhaseBiasBoundsCollection &code_phase_bounds)>
      put_code_and_phase_biases_bounds = nullptr;
  std::function<void(const FlagsHighLevel &flags_high_level)>
      put_flags_high_level = nullptr;
  std::function<void(const FlagsGridPoints &flag_iono_grid_points)>
      put_flag_iono_grid_points = nullptr;
  std::function<void(const FlagsIonoGridPointLos &)>
      put_flag_iono_grid_point_los = nullptr;
  std::function<void(const FlagsIonoTileLos &flag_iono_tile_sat_los)>
      put_flag_iono_tile_sat_los = nullptr;
  std::function<void(const FlagsGridPoints &flag_tropo_grid_points)>
      put_flag_tropo_grid_points = nullptr;
  std::function<void(const FlagsSatellites &flag_satellites)>
      put_flag_satellites = nullptr;
};
}  // namespace internal

struct RtcmCallbackFunctions {
  DecoderCallbacks callbacks;
  internal::InternalCallbackFunctions internal_callbacks;
};
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CALLBACK_FUNCTIONS_H
