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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CAPPERS_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CAPPERS_H

#include <cstdint>
#include <limits>
#include <type_traits>

/**
 *  @brief Saturates a given value within the range limits of a given data type.
 *
 *
 * @param value
 * @return The clamped value
 */
template <typename TargetType, typename SourceType>
static inline TargetType cap_to_type(const SourceType value) {
  static_assert(!(std::is_signed<TargetType>::value &&
                  (!std::is_signed<SourceType>::value)),
                "If target type is signed, source type must be signed");

  if (value > std::numeric_limits<TargetType>::max()) {
    return std::numeric_limits<TargetType>::max();
  } else if (value < std::numeric_limits<TargetType>::min()) {
    return std::numeric_limits<TargetType>::min();
  } else {
    return static_cast<TargetType>(value);
  }
}

/**
 *  @brief Saturates a given value within a given closed interval.
 *
 *  @details No checks are performed on bounds, so the caller needs to make sure
 *  that lower < upper.
 *
 * @param value
 * @param lower_bound
 * @param upper_bound
 * @return The clamped value
 */
template <typename ValueType>
static inline ValueType clamp(const ValueType value,
                              const ValueType lower_bound,
                              const ValueType upper_bound) {
  if (value > upper_bound) {
    return upper_bound;
  }
  if (value < lower_bound) {
    return lower_bound;
  }
  return value;
}

/**
 *  @brief Does check whether a given value values is between or equal to given
 *  bounds, i.e. within a given closed interval.
 *
 *  @details No checks are performed on bounds, so the caller needs to make sure
 *  that lower < upper.
 *
 * @param value
 * @param lower_bound
 * @param upper_bound
 * @return true if value \in [lower, upper], false otherwise
 */
template <typename ValueType>
inline bool is_within_interval(const ValueType value,
                               const ValueType lower_bound,
                               const ValueType upper_bound) {
  if ((value < lower_bound) || (value > upper_bound)) {
    return false;
  }
  return true;
}

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_CAPPERS_H
