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

#ifndef SWIFT_SSR_CLIENT_H
#define SWIFT_SSR_CLIENT_H

#include <swift/config.h>
#include <swift/inputs.h>
#include <swift/outputs.h>

#include <internal_types/callback_functions.h>
#include <los_calculator/los_calculator.h>
#include <rtcm_decoder/rtcm_decoder.h>
#include <storage_areas/active_area.h>
#include <storage_areas/area_converter.h>
#include <storage_areas/flags_area.h>
#include <storage_areas/holding_area.h>

namespace swift {

// Main interface for the Swift SSR client
//
// This client performs two tasks:
//  1. Decoding and storing SSR corrections from RTCM messages.
//  2. Calculating the line of sight component for all SSR corrections at a
//      particular time and location for a set of satellites.
//
// The client is not thread safe, so access to it from across multiple threads
// must be externally synchronized.
class SsrClient {
 public:
  SsrClient() = default;
  virtual ~SsrClient() = default;

  // This function is used to initialize the library
  //
  // The user of the library initializes it with callback functions.
  // These callback functions are allowed to be `nullptr` if the user
  // is not interested in the corresponding data.
  virtual ReturnCode init(const InitConfig &config,
                          const CallbackFunctions &callbacks);

  // This function is used to reset the library
  //
  // After calling the function the state is the same as after init() was called
  virtual ReturnCode reset();

  // This function is the primary input to the SSR client.
  //
  // It takes in a single RTCM payload, represented as an array of bytes along
  // with a message length. This function assumes that the message has
  // already been fully read out of a stream, and all authentication and
  // error checking has been completed successfully. It decodes data from the
  // supported messages and stores the relevant information internally.
  virtual ReturnCode handle_rtcm_message(const uint8_t *msg_payload,
                                         const std::size_t msg_length);

  // This function is the primary way to request SSR corrections from the
  // client.
  //
  // The requester provides a rover position (in ECEF coordinates), a time,
  // and a set of satellites to get corrections for. The list of satellites
  // contains not just the satellite identification but also the satellite
  // position, velocity, acceleration, and clock states calculated from the
  // broadcast ephemeris. This function uses this input and the internally
  // stored correction data to calculate the line of sight components for the
  // corrections for each satellite.
  //
  // Corrections components for each satellite are calculated and returned
  // in the output parameter `corrections`.
  //
  // An error code returned from this function will likely mean that SSR
  // is unavailable or inapplicable. In these cases the user must request
  // new SSR data from what source is providing it and feed it in to the
  // client via #handle_rtcm_message
  virtual ReturnCode calculate_corrections(
      const std::array<double, 3> &rover_position, const uint32_t tow_ms,
      const uint16_t wn, LosCorrections *corrections);

  // Don't allow assignment, move or copy
  void operator=(const SsrClient &) = delete;
  SsrClient(const SsrClient &) = delete;
  SsrClient(SsrClient &&) = delete;

 private:
  InitConfig init_config_{};
  CallbackFunctions initial_callbacks_{};

  // Contains complete and current SSR data to be consumed during calls
  // to #calc_corrections.
  internal::ActiveArea active_area_;

  // A holding area where incoming RTCM messages can be staged until the
  // entire sequence of corrections has been received. Once the sequence
  // is complete and verified the data here is copied over to #active_area_
  // to be used in future calculations.
  internal::HoldingArea holding_area_;

  internal::FlagsArea flags_area_chain_1_;
  internal::FlagsArea flags_area_chain_2_;

  // Converts compatible collections stored in holding area into active area.
  internal::AreaConverter area_converter_;

  // Calculator object for doing the transformations from SSR to line-of-sight.
  internal::LosCalculator los_calculator_;

  // Decoder object for decoding the incoming messages into internal structures.
  rtcm_decoder::RtcmDecoder rtcm_decoder_;

  // Integrity provider to handle flags and integrity.
  internal::IntegrityProvider integrity_provider_;

  // Internal state to signal an unexpected error.
  ReturnCode error_state_ = ReturnCode::SUCCESS;

  // Attached callback functions for each incoming rtcm message decoded in the
  // rtcm decoder.
  void decoder_callback_orbit_clock(
      const internal::SsrOrbitClockCollection &collection);
  void decoder_callback_orbit_clock_bounds(
      const internal::SsrOrbitClockBoundsCollection &collection);
  void decoder_callback_code_biases(
      const internal::SsrCodeBiasesCollection &collection);
  void decoder_callback_phase_biases(
      const internal::SsrPhaseBiasesCollection &collection);
  void decoder_callback_tile_definition(
      const internal::TileDefinitionCollection &collection);
  void decoder_callback_stec_polynomials(
      const internal::StecPolynomialSatCollection &collection);
  void decoder_callback_atmo_grid(const internal::AtmoGrid &collection);
  void decoder_callback_satellite_apc(
      const internal::SsrSatelliteApcCollection &collection);
  void decoder_callback_code_and_phase_biases_bounds(
      const internal::SsrCodeAndPhaseBiasBoundsCollection &collection);
  void decoder_callback_orbit_clock_bounds_degradation(
      const internal::SsrOrbitClockBoundsDegradationCollection &collection);
  // Flags decoders
  void decoder_callback_flag_high_level(
      const internal::FlagsHighLevel &incoming_flags);
  void decoder_callback_flag_satellites(
      const internal::FlagsSatellites &incoming_flags);
  void decoder_callback_flag_tropo_grid_points(
      const internal::FlagsGridPoints &incoming_flags);
  void decoder_callback_flag_iono_grid_points(
      const internal::FlagsGridPoints &incoming_flags);
  void decoder_callback_flag_iono_tile_sat_los(
      const internal::FlagsIonoTileLos &incoming_flags);
  void decoder_callback_flag_iono_grid_point_sat_los(
      const internal::FlagsIonoGridPointLos &incoming_flags);
};

}  // namespace swift

#endif /* SWIFT_SSR_CLIENT_H */
