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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_CLOCK_CALCULATOR_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_CLOCK_CALCULATOR_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <integrity/integrity_provider.h>
#include <internal_types/common.h>
#include <los_calculator/common.h>
#include <storage_areas/active_area.h>

#include <cstdint>

namespace swift {
namespace internal {

class ClockCalculator {
 public:
  explicit ClockCalculator(const IntegrityConfig &integrity_config);

  /**
   * @brief Calculate LOS clock corrections from SSR correction input.
   *
   * @param[in] timestamp Timestamp for which the corrections are calculated
   * for.
   * @param[in] solution_id Solution ID of the SSR corrections.
   * @param[in] integrity_provider Integrity provider to validate that flags for
   * the SSR corrections are nominal.
   * @param[in] sat_id Satellite constellation and number.
   * @param[in] clock Clock parameters from SSR service.
   *
   * @param[out] component_result Output clock corrections.
   *
   * @return ReturnCode
   */
  ReturnCode calculate_corrections(const TimestampMs &timestamp_ms,
                                   const uint8_t solution_id,
                                   const IntegrityProvider &integrity_provider,
                                   const SatelliteDescription &sat_id,
                                   const SsrCoherentSet::Clock &clock,
                                   LosComponentResult *component_result) const;

 private:
  IntegrityConfig integrity_config_;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_CLOCK_CALCULATOR_H
