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

#ifndef SWIFT_SSR2LOS_SRC_STORAGE_AREAS_ACTIVE_AREA_H
#define SWIFT_SSR2LOS_SRC_STORAGE_AREAS_ACTIVE_AREA_H

#include <array>
#include <cstdint>

#include <swift/inputs.h>

#include <integrity/integrity_provider.h>
#include <internal_types/atmo.h>
#include <internal_types/code_bias.h>
#include <internal_types/common.h>
#include <internal_types/orbit_clock.h>
#include <internal_types/phase_bias.h>
#include <internal_types/satellite_apc.h>
#include <internal_types/static_buffer.h>
#include "internal_types/tile_definition.h"

namespace swift {
namespace internal {

struct CorrectionId {
  uint8_t iod_ssr = 0;
  uint8_t iod_atmo = 0;
  uint8_t solution_id = 0;

  bool operator==(const CorrectionId &other) const {
    return ((this->iod_ssr == other.iod_ssr) &&
            (this->iod_atmo == other.iod_atmo) &&
            (this->solution_id == other.solution_id));
  }
};

struct Header {
  bool valid = false;
  Timestamp timestamp;
  uint32_t validity_duration_s = 0;
};

struct SsrCoherentSet {
  SatelliteDescription sat_id;
  CorrectionId correction_id;
  uint16_t iode = 0;

  struct OrbitCorrection {
    Header header;
    float update_interval_s = 0;
    float radial = 0.0;
    float along = 0.0;
    float cross = 0.0;
    float delta_radial = 0.0;
    float delta_along = 0.0;
    float delta_cross = 0.0;
  };
  struct OrbitBounds {
    Header header;
    MeanAndStdDev radial;
    MeanAndStdDev along;
    MeanAndStdDev cross;
  };
  struct OrbitBoundsDegradation {
    Header header;
    MeanAndStdDev radial;
    MeanAndStdDev along;
    MeanAndStdDev cross;
  };
  struct Orbit {
    OrbitCorrection corr;
    OrbitBounds bounds;
    OrbitBoundsDegradation degradation;
  };
  Orbit orbit;

  struct ClockCorrection {
    Header header;
    float update_interval_s = 0.0;
    float delta_clock_c0 = 0.0;
    float delta_clock_c1 = 0.0;
    float delta_clock_c2 = 0.0;
  };
  struct ClockBounds {
    Header header;
    MeanAndStdDev value;
  };
  struct ClockBoundsDegradation {
    Header header;
    MeanAndStdDev value;
  };
  struct Clock {
    ClockCorrection corr;
    ClockBounds bounds;
    ClockBoundsDegradation degradation;
  };
  Clock clock;

  struct BiasAndBound {
    bool valid = false;
    uint8_t signal_code = 255;
    float bias = 0.0;
    MeanAndStdDev bounds_m;
  };

  struct CodeBias {
    Header biases_header;
    Header bounds_header;
    std::size_t biases_count = 0;
    std::array<BiasAndBound, cMaxSignals> biases;
  };
  CodeBias code_bias;

  struct PhaseBias {
    Header biases_header;
    Header bounds_header;
    float yaw_rad = 0.0;
    float yaw_rate_radps = 0.0;
    std::size_t biases_count = 0;
    std::array<BiasAndBound, cMaxSignals> biases;
    std::array<uint8_t, cMaxSignals> bias_discontinuity_counters;
  };
  PhaseBias phase_bias;

  struct SatelliteApc {
    Header header;
    StaticBuffer<SsrSignalApc, cMaxSignals> signal_apcs;
  };
  SatelliteApc apc;

  struct Tropo {
    Header header;
    GridId grid_id;
    uint16_t update_interval_s = 0;
    float tropo_quality_indicator_m = 0;
    std::size_t grid_points_count = 0;
    std::array<TropoPoint, cMaxGridPoints> grid_points;
  };
  StaticBuffer<Tropo, cMaxTiles> tropo;

  struct Iono {
    Header header;
    GridId grid_id;
    uint16_t update_interval_s = 0;
    float stec_quality_indicator_tecu = 0.0;
    StecPolynomial stec_coefficients;
    std::array<StecResiduals, cMaxGridPoints> grid_points;
  };
  StaticBuffer<Iono, cMaxTiles> iono;
};

struct TileDefinition {
  Header header;
  GridId grid_id;
  uint16_t update_interval_s = 0;
  TileFrame frame;
};

struct ActiveArea {
  uint16_t provider_id = 0;
  StaticBuffer<SsrCoherentSet, cMaxSats> satellites;
  IntegrityProvider integrity_provider;
  StaticBuffer<TileDefinition, cMaxTiles> tile_definitions;
};

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_STORAGE_AREAS_ACTIVE_AREA_H
