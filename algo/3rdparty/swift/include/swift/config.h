/*
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

#ifndef SWIFT_SSR_CLIENT_CONFIG_H
#define SWIFT_SSR_CLIENT_CONFIG_H

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>

namespace swift {

// Compile time configuration
static constexpr std::size_t cMaxGridPoints = 64;
static constexpr std::size_t cMaxTiles = 2;

struct ChainIds {
  uint8_t chain_1 = 0;
  uint8_t chain_2 = 1;
};

enum class CorrectionType : uint8_t {
  CODE_BIAS = 0,
  PHASE_BIAS,
  ORBIT,
  CLOCK,
  IONO,
  TROPO,
  COUNT
};

class CorrectionTypesSet {
 public:
  CorrectionTypesSet() = default;

  explicit CorrectionTypesSet(
      const std::initializer_list<CorrectionType> &correction_types) {
    for (const auto &correction_type : correction_types) {
      insert(correction_type);
    }
  }

  bool contains(const CorrectionType correction_type) const {
    if (correction_type >= CorrectionType::COUNT) {
      return false;
    }
    return correction_types_[static_cast<std::size_t>(correction_type)];
  };

  void insert(const CorrectionType correction_type) {
    if (correction_type < CorrectionType::COUNT) {
      correction_types_[static_cast<std::size_t>(correction_type)] = true;
    }
  }

  void erase(const CorrectionType correction_type) {
    if (correction_type < CorrectionType::COUNT) {
      correction_types_[static_cast<std::size_t>(correction_type)] = false;
    }
  }

  std::size_t size() const { return correction_types_.count(); }

 private:
  std::bitset<static_cast<std::size_t>(CorrectionType::COUNT)>
      correction_types_;
};

/**
 * @brief Configuration of the tropo correction integrity check to update its
 * integrity flag when tropo is not monitored by the correction service
 * integrity chains.
 */
struct TropoCheckConfig {
  TropoCheckConfig() { set_adaptive_bound_approach_default_values(); }

  /**
   * @brief Factor between 0 and 1 to be multiplied with the separation
   */
  double alpha;
  /**
   * @brief Threshold to be compared with the separation in meters
   */
  double threshold_m;
  /**
   * @brief Minimum bound that we allow at zenith in meters
   */
  double gamma_zenith_m;
  /**
   * @brief Bound mean at zenith in meters
   */
  double mean_beta_zenith_m;
  /**
   * @brief Bound sigma at zenith in meters
   */
  double sigma_beta_zenith_m;

  void set_adaptive_bound_approach_default_values() {
    // Set 1 to add the separation to the bound.
    alpha = 1.0;
    // Arbitrary large value not to flag.
    threshold_m = 5.0;
    // No need to have a minimal bound.
    gamma_zenith_m = 0.0;
    // Minimum bias paired-bound from MOPS model error analysis (see the
    // following article
    // https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Lai_ION_ITM_2023_Tropo.pdf)
    mean_beta_zenith_m = 0.1526;
    // Paired-bound standard deviation from MOPS model error analysis (see the
    // following article
    // https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Lai_ION_ITM_2023_Tropo.pdf)
    sigma_beta_zenith_m = 0.0624;
  }

  void set_fixed_bound_approach_default_values() {
    // Set 0 to have fixed bound.
    alpha = 0.0;
    // Threshold to respect the target Probability of False Alarm derived from
    // an analysis of the separation Cumulative Distribution Function. See the
    // following Confluence page for more details:
    // https://swift-nav.atlassian.net/wiki/spaces/ENG/pages/2858876935/Tropospheric+Monitoring+-+ssr2los+-+Parameters+and+Performance
    threshold_m = 0.1710;
    // Minimum fault free bound in meters derived from an analysis of the
    // tropospheric residual error Cumulative Distribution Function.
    // See the following Confluence page for more details:
    // https://swift-nav.atlassian.net/wiki/spaces/ENG/pages/2858876935/Tropospheric+Monitoring+-+ssr2los+-+Parameters+and+Performance
    gamma_zenith_m = 0.2704;
    // Threshold + Minimum bias paired-bound from MOPS model error analysis (see
    // the following article
    // https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Lai_ION_ITM_2023_Tropo.pdf)
    mean_beta_zenith_m = 0.3236;
    // Paired-bound standard deviation from MOPS model error analysis multiplied
    // by K_MD and divided by K_TIR. With K_MD the K related to the probability
    // of misdetection and K_TIR the K related to the target integrity risk (see
    // the following article
    // https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Lai_ION_ITM_2023_Tropo.pdf)
    sigma_beta_zenith_m = 0.0518;
  }
};

using CorrectionTimeContributors = CorrectionTypesSet;
using ExpectedCorrectionTypes = CorrectionTypesSet;

struct IntegrityConfig {
  bool enable_flags_checking = false;
  bool enable_bounds_calculation = true;
  // If true then enable_bounds_calculation should always be true as we use the
  // bound parameters to compute the protection level
  bool enable_protection_level_vs_threshold_comparison = false;
  double integrity_risk = 1e-5;
  double code_bias_integrity_threshold_m = 1.0;
  double iono_integrity_threshold_m = 1.0;
  double orbit_integrity_threshold_m = 1.0;
  double phase_bias_integrity_threshold_m = 1.0;
  double tropo_integrity_threshold_m = 1.0;
  double clock_integrity_threshold_m = 1.0;
  CorrectionTimeContributors correction_time_contributors{
      CorrectionType::CODE_BIAS, CorrectionType::PHASE_BIAS,
      CorrectionType::ORBIT,     CorrectionType::CLOCK,
      CorrectionType::IONO,      CorrectionType::TROPO};
  ExpectedCorrectionTypes expected_correction_types;
  /**
   * @brief boolean that allows to validate the tropo correction integrity
   * thanks to an apriori model (MOPS) when set to true
   */
  bool enable_local_tropo_integrity_check = true;
  TropoCheckConfig tropo_check;
};

struct InitConfig {
  IntegrityConfig integrity;
  ChainIds chain_ids;
};

}  // namespace swift

#endif  // SWIFT_SSR_CLIENT_CONFIG_H
