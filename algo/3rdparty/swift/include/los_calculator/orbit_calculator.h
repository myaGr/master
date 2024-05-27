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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_ORBIT_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_ORBIT_CALCULATOR_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <integrity/integrity_provider.h>
#include <internal_types/common.h>
#include <los_calculator/common.h>
#include <storage_areas/active_area.h>

#include <array>
#include <cstdint>

namespace swift {
namespace internal {

enum class RswIndices : uint8_t { R_RADIAL = 0, S_ALONG, W_CROSS };

class OrbitCalculator {
 public:
  explicit OrbitCalculator(const IntegrityConfig &integrity_config);

  /**
   * @brief Calculate LoS orbit corrections from SSR correction input.
   *
   * The LoS correction output from this function must be subtracted from the
   * measurements of the corresponding signal in order to get a corrected
   * measurement, such that:
   *
   * meas_corrected = meas - correction.corr
   *
   * @param[in] timestamp Timestamp for which the corrections are calculated
   * for.
   * @param[in] solution_id Solution ID of the SSR corrections.
   * @param[in] integrity_provider Integrity provider to validate that flags for
   * the SSR corrections are nominal.
   * @param[in] rover_position Rover position in ECEF.
   * @param[in] sat_pva Satellite Position Velocity and Acceleration states.
   * @param[in] orbit Orbit parameters from SSR service.
   *
   * @param[out] component_result Output orbit corrections.
   *
   * @return ReturnCode
   */
  ReturnCode calculate_corrections(const TimestampMs &timestamp_ms,
                                   const uint8_t solution_id,
                                   const IntegrityProvider &integrity_provider,
                                   const std::array<double, 3> &rover_position,
                                   const SatellitePva &sat_pva,
                                   const SsrCoherentSet::Orbit &orbit,
                                   LosComponentResult *component_result) const;

 private:
  IntegrityConfig integrity_config_;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_ORBIT_CALCULATOR_H
