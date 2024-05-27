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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_CODE_BIAS_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_CODE_BIAS_CALCULATOR_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <integrity/integrity_provider.h>
#include <internal_types/common.h>
#include <internal_types/flags.h>
#include <internal_types/satellite_apc.h>
#include <storage_areas/active_area.h>

#include <array>
#include <cstdint>

namespace swift {
namespace internal {

struct CodeBiasCorrections {
  std::array<SsrSignalCorrection, cMaxSignals> correction{};
  Timestamp latest_corr_timestamp{};
  bool integrity_flags_valid = false;
};

class CodeBiasCalculator {
 public:
  explicit CodeBiasCalculator(const IntegrityConfig &integrity_config);

  /**
   *
   * @param[in] timestamp_ms Timestamp for which the corrections are calculated.
   * @param[in] solution_id Solution ID of the SSR corrections.
   * @param[in] integrity_provider Integrity provider to validate that flags for
   * the SSR corrections are nominal.
   * @param[in] rover_position Rover position in ECEF.
   * @param[in] sat_pva Satellite Position Velocity and Acceleration states.
   * @param[in] satellite_yaw_rad Satellite yaw in radians.
   * @param[in] sat_apcs Antenna Phase Center corrections from SSR service.
   * @param[in] code_bias Code bias parameters from SSR service.
   *
   * @param[out] component_result Output code bias corrections.
   *
   * @return ReturnCode.
   */
  ReturnCode calculate_corrections(
      const TimestampMs &timestamp_ms, const uint8_t solution_id,
      const IntegrityProvider &integrity_provider,
      const std::array<double, 3> &rover_position, const SatellitePva &sat_pva,
      const double satellite_yaw_rad, const SsrCoherentSet::SatelliteApc &apc,
      const SsrCoherentSet::CodeBias &code_bias,
      CodeBiasCorrections *const corrections) const;

 private:
  IntegrityConfig integrity_config_;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_CODE_BIAS_CALCULATOR_H
