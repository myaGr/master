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

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_SATELLITE_APC_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_SATELLITE_APC_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <internal_types/common.h>
#include <internal_types/static_buffer.h>

namespace swift {
namespace internal {

struct SsrSignalApc {
  // Signal band and code
  uint8_t signal_code = 255;
  // Mean phase center offset,X Y and Z axes.
  std::array<float, 3> phase_center_offset_m{};
  // Elevation dependent phase center variations. First element is 0 degrees
  // separation from the Z axis, subsequent elements represent elevation
  // variations in 1 degree increments.
  std::array<float, 21> phase_center_variations_m{};
};

struct SsrSatelliteApc {
  // Constellation-speciﬁc satellite identiﬁer
  SatelliteDescription sat_id;
  // Additional satellite information
  uint8_t sat_info = 0;
  // Satellite Code, as deﬁned by IGS. Typically the space vehicle number.
  uint16_t svn = 0;
  // Phase APCs and PCVs for frequencies emitted by this satellite
  StaticBuffer<SsrSignalApc, cMaxSignals> signal_apcs;
};

struct SsrSatelliteApcCollection {
  bool complete = false;
  Timestamp timestamp;
  CollectionIdentifier collection_id;
  uint8_t iod_ssr = 0;
  uint16_t update_interval_s = 0;

  StaticBuffer<SsrSatelliteApc, cApcSlots> satellites;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_SATELLITE_APC_H
