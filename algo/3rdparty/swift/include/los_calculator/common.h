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

#ifndef SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_COMMON_H
#define SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_COMMON_H

#pragma warning(disable:4244)

#include <swift/config.h>
#include <swift/inputs.h>
#include <swift/outputs.h>

#include <helper_functions/cappers.h>
#include <internal_types/common.h>

#include <array>
#include <limits>

namespace swift {
namespace internal {

static constexpr double cAvgYearLen = 365.25;

constexpr struct {
  uint32_t l1_hz = 1575420e3;
  uint32_t l2_hz = 1227600e3;
  uint32_t l5_hz = 1176450e3;
} GpsFrequencies;

constexpr struct {
  uint32_t e1_hz = 1575420e3;
  uint32_t e5a_hz = 1176450e3;
  uint32_t e5b_hz = 1207140e3;
  uint32_t e5_hz = 1191795e3;
  uint32_t e6_hz = 1278750e3;

} GalFrequencies;

constexpr struct {
  uint32_t b1_hz = 1561098e3;
  uint32_t b1a_hz = 1575420e3;
  uint32_t b1c_hz = 1575420e3;
  uint32_t b2_hz = 1207140e3;
  uint32_t b2b_hz = 1207140e3;
  uint32_t b2a_hz = 1176450e3;
  uint32_t b2a_plus_b2b_hz = 1191795e3;
  uint32_t b3_hz = 1268520e3;
  uint32_t b3a_hz = 1268520e3;
} BdsFrequencies;

constexpr double cGpsIcdSpeedOfLight = 2.99792458e8;

// Struct holding output from a calculator for one component, per satellite
struct LosComponentResult {
  SsrCorrection correction{};
  Timestamp latest_corr_timestamp{};
  bool integrity_flags_valid = false;
};

/**
 * @brief Returns the carrier frequency corresponding to DF380 and DF382 signal
 * identifier fields given from RTCM 3.3.
 *
 * @param constellation The constellation of the satellite emitting the signal.
 * @param signal_code The signal identifier tracking code.
 * @param frequency_Hz The carrier frequency output.
 * @return false if the frequency belonging to the given constellation/signal_id
 * is not known. true if a matching constellation and signal code was found, in
 * which case the frequency_Hz gets updated.
 */
inline bool rtcm_signal_code_to_frequency(const GnssId constellation,
                                          const uint8_t signal_code,
                                          uint32_t *const frequency_Hz) {
  bool found_signal_frequency = false;
  if (constellation == GnssId::GPS) {
    if (is_within_interval<uint8_t>(signal_code, 0, 2)) {
      *frequency_Hz = GpsFrequencies.l1_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 5, 11)) {
      *frequency_Hz = GpsFrequencies.l2_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 14, 15)) {
      *frequency_Hz = GpsFrequencies.l5_hz;
      found_signal_frequency = true;
    }
  }

  if (constellation == GnssId::GAL) {
    if (is_within_interval<uint8_t>(signal_code, 0, 2)) {
      *frequency_Hz = GalFrequencies.e1_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 5, 6)) {
      *frequency_Hz = GalFrequencies.e5a_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 8, 9)) {
      *frequency_Hz = GalFrequencies.e5b_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 11, 12)) {
      *frequency_Hz = GalFrequencies.e5_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 14, 16)) {
      *frequency_Hz = GalFrequencies.e6_hz;
      found_signal_frequency = true;
    }
  }

  return found_signal_frequency;
}

/**
 * @brief Returns the carrier frequency corresponding to Swift RTCM signal
 * identifier fields.
 *
 * @details Swift RTCM signal identifier fields share the signal identifier
 * fields defined in RTCM 3.3 as well as define signals not yet added to the
 * RTCM standard. See https://swift-nav.atlassian.net/l/cp/X5h7wBR1
 *
 * @param constellation The constellation of the satellite emitting the signal.
 * @param signal_code The signal and tracking mode identifier.
 * @param frequency_Hz The carrier frequency output.
 * @return false if the frequency belonging to the given constellation/signal_id
 * is not known. true if a matching constellation and signal code was found, in
 * which case the frequency_Hz argument gets updated.
 */
inline bool swift_rtcm_signal_code_to_frequency(const GnssId constellation,
                                                const uint8_t signal_code,
                                                uint32_t *const frequency_Hz) {
  bool found_signal_frequency = false;
  if (constellation == GnssId::GPS) {
    if (signal_code == 16) {  // L5 Data + Pilot channel
      *frequency_Hz = GpsFrequencies.l5_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 17, 19)) {  // L1C
      *frequency_Hz = GpsFrequencies.l1_hz;
      found_signal_frequency = true;
    }
  }

  if (constellation == GnssId::GAL) {
    if (is_within_interval<uint8_t>(signal_code, 3, 4)) {
      *frequency_Hz = GalFrequencies.e1_hz;
      found_signal_frequency = true;
    }
    if (signal_code == 7) {  // E5a Data + Pilot
      *frequency_Hz = GalFrequencies.e5a_hz;
      found_signal_frequency = true;
    }
    if (signal_code == 10) {  // E5b Data + Pilot
      *frequency_Hz = GalFrequencies.e5b_hz;
      found_signal_frequency = true;
    }
    if (signal_code == 13) {  // E5a+b Data + Pilot
      *frequency_Hz = GalFrequencies.e5_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 17, 18)) {
      *frequency_Hz = GalFrequencies.e6_hz;
      found_signal_frequency = true;
    }
  }

  if (constellation == GnssId::BDS) {
    if (is_within_interval<uint8_t>(signal_code, 0, 2)) {
      *frequency_Hz = BdsFrequencies.b1_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 3, 5)) {
      *frequency_Hz = BdsFrequencies.b3_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 6, 8)) {
      *frequency_Hz = BdsFrequencies.b2_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 9, 11)) {
      *frequency_Hz = BdsFrequencies.b1a_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 12, 14)) {
      *frequency_Hz = BdsFrequencies.b2a_hz;
      found_signal_frequency = true;
    }
    if (is_within_interval<uint8_t>(signal_code, 15, 17)) {
      *frequency_Hz = BdsFrequencies.b1c_hz;
      found_signal_frequency = true;
    }
  }

  if (!found_signal_frequency) {
    return rtcm_signal_code_to_frequency(constellation, signal_code,
                                         frequency_Hz);
  }
  return found_signal_frequency;
}

/**
 * @brief Returns the carrier frequency corresponding to an SBP signal code.
 * Supports GPS, GAL, BDS codes
 *
 * @param[in] signal_code The SBP signal code of the signal in question.
 * @param[out] frequency_Hz The signal's carrier frequency.
 * @return false if the frequency belonging to the given signal code
 *               is not known.
 *         true if a carrier frequency for the given signal code is known. In
 *              this case the frequency_Hz argument is updated.
 */
inline bool sbp_signal_code_to_frequency(const uint8_t signal_code,
                                         uint32_t *const frequency_Hz) {
  bool found_signal_frequency = true;
  // GPS signals:
  if ((signal_code == 0) || (signal_code == 5) ||
      is_within_interval<uint8_t>(signal_code, 56, 58)) {
    *frequency_Hz = GpsFrequencies.l1_hz;
  } else if ((signal_code == 1) ||
             is_within_interval<uint8_t>(signal_code, 6, 8)) {
    *frequency_Hz = GpsFrequencies.l2_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 9, 11)) {
    *frequency_Hz = GpsFrequencies.l5_hz;
  }
  // GAL signals:
  else if (is_within_interval<uint8_t>(signal_code, 14, 16)) {
    *frequency_Hz = GalFrequencies.e1_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 17, 19)) {
    *frequency_Hz = GalFrequencies.e6_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 20, 22)) {
    *frequency_Hz = GalFrequencies.e5b_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 23, 25)) {
    *frequency_Hz = GalFrequencies.e5_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 26, 28)) {
    *frequency_Hz = GalFrequencies.e5a_hz;
  }
  // BDS signals:
  else if (signal_code == 12) {
    *frequency_Hz = BdsFrequencies.b1_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 44, 46)) {
    *frequency_Hz = BdsFrequencies.b1c_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 47, 49)) {
    *frequency_Hz = BdsFrequencies.b2a_hz;
  } else if (signal_code == 13) {
    *frequency_Hz = BdsFrequencies.b2_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 50, 52)) {
    *frequency_Hz = BdsFrequencies.b2b_hz;
  } else if (is_within_interval<uint8_t>(signal_code, 53, 55)) {
    *frequency_Hz = BdsFrequencies.b3_hz;
  } else {
    found_signal_frequency = false;
  }
  return found_signal_frequency;
}

/**
 * @brief Finds the index of a given ID in a given array
 */
template <typename CandidateArrayType, typename IdType>
static inline bool find_match(const IdType &id,
                              const CandidateArrayType &candidates,
                              const std::size_t number_of_candidates,
                              std::size_t *const matching_index) {
  const std::size_t candidates_size = candidates.size();
  for (std::size_t idx = 0;
       (idx < number_of_candidates) && (idx < candidates_size); ++idx) {
    *matching_index = idx;
    if (id == candidates[*matching_index]) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Finds the index of a given satellite in a given array.
 */
template <typename CandidateArrayType, typename SatIdType>
static inline bool find_matching_sat(const SatIdType &sat_id,
                                     const CandidateArrayType &candidates,
                                     const std::size_t number_of_candidates,
                                     std::size_t *const matching_index) {
  const std::size_t candidates_size = candidates.size();
  for (std::size_t idx = 0;
       (idx < number_of_candidates) && (idx < candidates_size); ++idx) {
    *matching_index = idx;
    if (sat_id == candidates[*matching_index].sat_id) {
      return true;
    }
  }
  return false;
}

template <typename CandidateArrayType, typename SatIdType>
static inline bool find_matching_sat(const SatIdType &sat_id,
                                     const CandidateArrayType &candidates,
                                     std::size_t *const matching_index) {
  const std::size_t candidates_size = candidates.size();
  for (std::size_t idx = 0; idx < candidates_size; ++idx) {
    *matching_index = idx;
    if (sat_id == candidates[*matching_index].sat_id) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Finds the index of a given signal in a given array.
 */
template <typename CandidateArrayType>
static inline bool find_matching_signal(const uint8_t signal_code,
                                        const CandidateArrayType &candidates,
                                        std::size_t *const index) {
  const std::size_t candidates_size = candidates.size();
  for (std::size_t idx = 0; idx < candidates_size; ++idx) {
    *index = idx;
    if (candidates[*index].signal_code == signal_code) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Finds the index of the first signal in a given array that matches the
 * given frequency.
 *
 * @param[in] frequency_hz Desired carrier frequency
 * @param[in] candidates Array of candidate signals carrying SBP signal codes
 * @param[out] index If a match is found, its array index is written to this
 * argument.
 *
 * @return True if a signal with the right frequency could be found, false
 * otherwise.
 */
template <typename CandidateArrayType>
static inline bool find_matching_signal_by_frequency(
    const uint32_t frequency_hz, const CandidateArrayType &candidates,
    std::size_t *const index) {
  const std::size_t candidates_size = candidates.size();
  for (std::size_t idx = 0; idx < candidates_size; ++idx) {
    *index = idx;
    uint32_t candidate_frequency_hz = 0;
    if (!sbp_signal_code_to_frequency(candidates[*index].signal_code,
                                      &candidate_frequency_hz)) {
      // We should be able to look up the frequency of any signal, so
      // conservatively return false here:
      return false;
    }
    if (candidate_frequency_hz == frequency_hz) {
      return true;
    }
  }
  return false;
}

inline bool has_one_valid_signal_correction(
    const std::array<SsrSignalCorrection, cMaxSignals> &signal_correction) {
  for (const auto &ssr_signal : signal_correction) {
    if (ssr_signal.correction.valid) {
      return true;
    }
  }
  return false;
}

inline bool has_expected_correction_types(
    const SatelliteCorrection &satellite_correction,
    const ExpectedCorrectionTypes &expected_correction_types) {
  if (expected_correction_types.size() == 0) {
    return true;
  }
  if (expected_correction_types.contains(CorrectionType::CLOCK) &&
      (!satellite_correction.clock.valid)) {
    return false;
  }
  if (expected_correction_types.contains(CorrectionType::IONO) &&
      (!satellite_correction.iono.valid)) {
    return false;
  }
  if (expected_correction_types.contains(CorrectionType::TROPO) &&
      (!satellite_correction.tropo.valid)) {
    return false;
  }
  if (expected_correction_types.contains(CorrectionType::ORBIT) &&
      (!satellite_correction.orbit.valid)) {
    return false;
  }
  if (expected_correction_types.contains(CorrectionType::PHASE_BIAS) &&
      (!has_one_valid_signal_correction(satellite_correction.phase_bias))) {
    return false;
  }
  if (expected_correction_types.contains(CorrectionType::CODE_BIAS) &&
      (!has_one_valid_signal_correction(satellite_correction.code_bias))) {
    return false;
  }
  return true;
}

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_LOS_CALCULATOR_COMMON_H
