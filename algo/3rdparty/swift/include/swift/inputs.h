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

#ifndef SWIFT_SSR_CLIENT_INPUTS_H
#define SWIFT_SSR_CLIENT_INPUTS_H

#include <swift/compile_constants.h>

#include <array>
#include <cstdint>
#include <tuple>

namespace swift {

static constexpr std::size_t cMaxSats = cMaxGpsSats + cMaxGalSats + cMaxBdsSats;
static constexpr std::size_t cMaxSatsPerConstellation = cMaxBdsSats;
static_assert((cMaxSatsPerConstellation >= cMaxGpsSats) &&
                  (cMaxSatsPerConstellation >= cMaxGalSats) &&
                  (cMaxSatsPerConstellation >= cMaxBdsSats),
              "cMax_<constellation>_Sats exceeds cMaxSatsPerConstellation");

enum class GnssId : uint8_t {
  GPS = 0,
  GAL = 1,
  BDS = 2,
};
static constexpr std::size_t cMaxConstellations = 3;
static_assert((cMaxConstellations - 1) == static_cast<std::size_t>(GnssId::BDS),
              "cMaxConstellations does not match number of constellations in "
              "enum GnssId");

struct SatelliteDescription {
  GnssId constellation = GnssId::GPS;
  uint8_t number = 0;

  bool operator==(const SatelliteDescription &other) const {
    return ((this->number == other.number) &&
            (this->constellation == other.constellation));
  }
  bool operator!=(const SatelliteDescription &other) const {
    return !(*this == other);
  }
  bool operator<(const SatelliteDescription &sat) const {
    return std::tie(this->constellation, this->number) <
           std::tie(sat.constellation, sat.number);
  }
};

struct SatellitePva {
  // Signals if fields in this instance are set or not
  bool valid = false;
  // ECEF, in m
  std::array<double, 3> position = {{0.0, 0.0, 0.0}};
  // ECEF, in m/s
  std::array<double, 3> velocity = {{0.0, 0.0, 0.0}};
  // ECEF, in m/s^2
  std::array<double, 3> acceleration = {{0.0, 0.0, 0.0}};
  // in m
  double clock = 0.0;
  // in m/s
  double clock_rate = 0.0;
  // IODE value used to calculate the satellite state
  uint16_t iode = 0;
  // Satellite constellation and number
  SatelliteDescription sat_id;
};

}  // namespace swift

#endif  // SWIFT_SSR_CLIENT_INPUTS_H
