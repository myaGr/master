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

#ifndef AUK_COMPILE_CONSTANTS_H
#define AUK_COMPILE_CONSTANTS_H

// !!! NOTE !!!
//   This file is populated from the template at
//   ssr2los/header_templates/compile_constants.h.in,
//   do not edit the header file directly.
// !!! NOTE !!!

#include <cstddef>

namespace swift {

// Dictates the maximum number of signals associated with a single satellite
static constexpr std::size_t cMaxSignals = 8;
// Dictates the maximum number of satellites we can handle
static constexpr std::size_t cMaxGpsSats = 14;
static constexpr std::size_t cMaxGalSats = 14;
static constexpr std::size_t cMaxBdsSats = 20;
// Dictates how many APC messages (one per satellite) we can store
static constexpr std::size_t cApcSlots = 108;

}  // namespace swift

#endif  // AUK_COMPILE_CONSTANTS_H
