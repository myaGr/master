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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_LOS_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_LOS_CALCULATOR_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <internal_types/common.h>
#include <internal_types/flags.h>
#include <internal_types/satellite_apc.h>
#include <internal_types/static_buffer.h>
#include <internal_types/tile_definition.h>
#include <los_calculator/clock_calculator.h>
#include <los_calculator/code_bias_calculator.h>
#include <los_calculator/grid_locality_calculator.h>
#include <los_calculator/iono_calculator.h>
#include <los_calculator/orbit_calculator.h>
#include <los_calculator/phase_bias_calculator.h>
#include <los_calculator/tropo_calculator.h>
#include <storage_areas/active_area.h>
#include <storage_areas/holding_area.h>

#include <array>

namespace swift {
namespace internal {

struct LosCalculatorResults {
  LosCorrections los_corrections{};
  TileId iono_tile_id{};
  TileId tropo_tile_id{};
};

struct SolutionSource {
  SatelliteDescription sat_id;
  CorrectionId correction_id;
};

class LosCalculator {
 public:
  LosCalculator();
  explicit LosCalculator(const IntegrityConfig &integrity_config);

  ReturnCode calculate_corrections(
      const std::array<double, 3> &rover_position,
      const TimestampMs &timestamp_ms,
      const std::array<SatellitePva, cMaxSats> &sat_states,
      const ActiveArea &active_area, LosCalculatorResults *results);

 private:
  GridLocalityCalculator grid_locality_calculator_;
  ClockCalculator clock_calculator_;
  CodeBiasCalculator code_bias_calculator_;
  IonoCalculator iono_calculator_;
  OrbitCalculator orbit_calculator_;
  PhaseBiasCalculator phase_bias_calculator_;
  TropoCalculator tropo_calculator_;

  ExpectedCorrectionTypes expected_correction_types_;
  CorrectionTimeContributors correction_time_contributors_;
  StaticBuffer<SolutionSource, cMaxSats> previous_solution_source_;
  bool detect_solution_source_change(
      const SolutionSource &current_solution_source);
  StaticBuffer<GridLocality, cMaxTiles> compute_grid_locality_set(
      const std::array<double, 3> &rover_position,
      const TimestampMs &timestamp_ms, const ActiveArea &active_area);

  /**
   * @brief CalculateCorrectionInputs contains references to common inputs to
   * the internal calculate functions
   */
  struct CalculateCorrectionCommonInputs {
    const TimestampMs &timestamp_ms;
    const ActiveArea &active_area;
    const SsrCoherentSet &sat_set;
    const SatellitePva &sat_pva;
  };

  void calculate_orbit(const CalculateCorrectionCommonInputs &inputs,
                       const std::array<double, 3> &rover_position,
                       bool *const any_computed_correction,
                       Timestamp *const latest_correction_time,
                       SatelliteCorrection *const satellite_correction);

  void calculate_clock(const CalculateCorrectionCommonInputs &inputs,
                       bool *const any_computed_correction,
                       Timestamp *const latest_correction_time,
                       SatelliteCorrection *const satellite_correction);

  void calculate_code_bias(const CalculateCorrectionCommonInputs &inputs,
                           const std::array<double, 3> &rover_position,
                           bool *const any_computed_correction,
                           Timestamp *const latest_correction_time,
                           SatelliteCorrection *const satellite_correction);

  void calculate_phase_bias(const CalculateCorrectionCommonInputs &inputs,
                            const std::array<double, 3> &rover_position,
                            bool *const any_computed_correction,
                            Timestamp *const latest_correction_time,
                            SatelliteCorrection *const satellite_correction);

  void calculate_iono(
      const CalculateCorrectionCommonInputs &inputs,
      const StaticBuffer<GridLocality, cMaxTiles> &grid_locality_set,
      bool *const any_computed_correction,
      Timestamp *const latest_correction_time,
      SatelliteCorrection *const satellite_correction, TileId *const tile_id);

  void calculate_tropo(
      const CalculateCorrectionCommonInputs &inputs,
      const std::array<double, 3> &rover_position,
      const StaticBuffer<GridLocality, cMaxTiles> &grid_locality_set,
      bool *const any_computed_correction,
      Timestamp *const latest_correction_time,
      SatelliteCorrection *const satellite_correction, TileId *const tile_id);
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_LOS_CALCULATOR_H
