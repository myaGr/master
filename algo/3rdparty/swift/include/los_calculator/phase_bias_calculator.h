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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_PHASE_BIAS_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_PHASE_BIAS_CALCULATOR_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <internal_types/common.h>
#include <internal_types/flags.h>
#include <internal_types/satellite_apc.h>
#include <storage_areas/active_area.h>

#include <array>

namespace swift {
namespace internal {

struct PhaseBiasCorrections {
  std::array<SsrSignalCorrection, cMaxSignals> correction{};
  Timestamp latest_corr_timestamp{};
  bool integrity_flags_valid = false;
  bool discontinuity = false;
};

// Data structures to keep track of changes in phase bias discontinuity
// satellite_phase_bias_states:
struct DiscontinuityCounter {
  uint8_t signal_code = 0;
  uint8_t counter = 0;
};

struct SatellitePhaseBiasState {
  uint8_t sat_id = 0;
  StaticBuffer<DiscontinuityCounter, cMaxSignals> counters;
  double windup_cycles = 0;
};

struct ConstellationPhaseBiasState {
  std::array<StaticBuffer<SatellitePhaseBiasState, cMaxSatsPerConstellation>,
             cMaxConstellations>
      satellite_phase_bias_states;
};

class PhaseBiasCalculator {
 public:
  explicit PhaseBiasCalculator(const IntegrityConfig &integrity_config);

  /**
   * @brief Calculate phase bias corrections from SSR correction input.
   *
   * The LoS correction output from this function must be added to the
   * measurements of the corresponding signal in order to get a corrected
   * measurement, such that:
   *
   * meas_corrected = meas + correction.corr
   *
   * @param[in] timestamp_ms Timestamp for which the corrections are calculated.
   * @param[in] solution_id Solution ID of the SSR corrections.
   * @param[in] integrity_provider Integrity provider to validate that flags for
   * the SSR corrections are nominal.
   * @param[in] rover_position Rover position in ECEF.
   * @param[in] sat_pva Satellite Position Velocity and Acceleration states.
   * @param[in] sat_apcs Antenna Phase Center corrections from SSR service.
   * @param[in] phase_bias Phase bias parameters from SSR service.
   *
   * @param[out] correction Output phase corrections.
   *
   * @return ReturnCode
   */
  ReturnCode calculate_corrections(const TimestampMs &timestamp_ms,
                                   const uint8_t solution_id,
                                   const IntegrityProvider &integrity_provider,
                                   const std::array<double, 3> &rover_position,
                                   const SatellitePva &sat_pva,
                                   const SsrCoherentSet::SatelliteApc &apc,
                                   const SsrCoherentSet::PhaseBias &phase_bias,
                                   PhaseBiasCorrections *corrections);

 private:
  IntegrityConfig integrity_config_;
  ConstellationPhaseBiasState most_recent_constellation_state_;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_PHASE_BIAS_CALCULATOR_H
