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

#ifndef SWIFT_SSR2LOS_SRC_RTCM_DECODER_SBP_DECODER_H
#define SWIFT_SSR2LOS_SRC_RTCM_DECODER_SBP_DECODER_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <rtcm_decoder/bits/sbp_msg.h>
#include <rtcm_decoder/constants.h>

#include <internal_types/atmo.h>
#include <internal_types/callback_functions.h>
#include <internal_types/flags.h>
#include <internal_types/satellite_apc.h>
#include <internal_types/satellite_bounds.h>
#include <internal_types/tile_definition.h>

namespace swift {
namespace rtcm_decoder {

struct MultiSbpPosTracker {
  std::array<uint16_t, cChainIdCount> msg_counter{};
  std::array<internal::Timestamp, cChainIdCount> last_timestamp = {};
};

class SbpDecoder {
 public:
  SbpDecoder() = default;
  explicit SbpDecoder(const RtcmCallbackFunctions &callbacks);
  void reset(const RtcmCallbackFunctions &callbacks);
  ReturnCode handle_sbp_message(SbpMsg *buff, MessageInfo *message_info);
  bool get_last_seen_timestamp(internal::Timestamp *timestamp) const;
  void reset_last_seen_timestamp();
  static bool decode_header(SbpMsg *buff, internal::Timestamp *timestamp,
                            uint8_t *num_msgs, uint8_t *position);

 private:
  ReturnCode parse_itrf_msg(SbpMsg *buff, ItrfModel *itrf_model_msg);
  ReturnCode parse_gridded_correction_msg(SbpMsg *buff,
                                          internal::AtmoGrid *atmo_grid);
  ReturnCode parse_gridded_correction_bounds_msg(SbpMsg *buff,
                                                 internal::AtmoGrid *atmo_grid);
  ReturnCode parse_stec_polynomial_msg(
      SbpMsg *buff, internal::StecPolynomialSatCollection *stec_correction);
  ReturnCode parse_stec_polynomial_dep_msg(
      SbpMsg *buff, internal::StecPolynomialSatCollection *stec_correction);
  ReturnCode parse_tile_definition_depA_msg(
      SbpMsg *buff, internal::TileDefinitionCollection *tile_definition);
  ReturnCode parse_tile_definition_depB_msg(
      SbpMsg *buff, internal::TileDefinitionCollection *tile_definition);
  ReturnCode parse_tile_definition_msg(
      SbpMsg *buff, internal::TileDefinitionCollection *tile_definition);
  ReturnCode parse_satellite_apc_dep_msg(
      SbpMsg *buff,
      internal::SsrSatelliteApcCollection *satellite_apc_collection);
  ReturnCode parse_satellite_apc_msg(
      SbpMsg *buff,
      internal::SsrSatelliteApcCollection *satellite_apc_collection);
  ReturnCode parse_orbit_clock_bounds_degradation_msg(
      SbpMsg *buff, internal::SsrOrbitClockBoundsDegradationCollection
                        *orbit_clock_bounds_degradation_collection);
  ReturnCode parse_orbit_clock_bounds_msg(
      SbpMsg *buff,
      internal::SsrOrbitClockBoundsCollection *orbit_clock_bounds_collection);
  ReturnCode parse_code_and_phase_biases_bounds(
      SbpMsg *buff, internal::SsrCodeAndPhaseBiasBoundsCollection
                        *code_and_phase_biases_bounds);
  ReturnCode parse_flags_high_level_msg(
      SbpMsg *buff, internal::FlagsHighLevel *flags_high_level);
  ReturnCode parse_flag_iono_grid_points(
      SbpMsg *buff, internal::FlagsGridPoints *flag_iono_grid_points);
  ReturnCode parse_flag_iono_grid_point_los(
      SbpMsg *buff, internal::FlagsIonoGridPointLos *flag_iono_grid_point_los);
  ReturnCode parse_flag_iono_tile_sat_los_msg(
      SbpMsg *buff, internal::FlagsIonoTileLos *flag_iono_tile_sat_los);
  ReturnCode parse_flag_tropo_grid_points(
      SbpMsg *buff, internal::FlagsGridPoints *flag_tropo_grid_points);
  ReturnCode parse_flag_satellites(SbpMsg *buff,
                                   internal::FlagsSatellites *flags_satellites);
  ReturnCode parse_leap_seconds(SbpMsg *buff, LeapSeconds *leap_seconds);
  ReturnCode parse_acknowledge_msg(SbpMsg *buff,
                                   AcknowledgeMessage *acknowledge_msg) const;

  void reset_multi_msg_trackers();
  void reset_last_timestamp_trackers();

  RtcmCallbackFunctions callbacks_;

  // Multi msg trackers
  MultiSbpPosTracker grid_correction_tracker_;
  MultiSbpPosTracker grid_correction_bounds_tracker_;
  MultiSbpPosTracker stec_correction_dep_tracker_;
  MultiSbpPosTracker stec_correction_tracker_;
  MultiSbpPosTracker clock_orbit_bounds_degradation_tracker_;
  MultiSbpPosTracker clock_orbit_bounds_tracker_;
  MultiSbpPosTracker code_phase_biases_tracker_;
  MultiSbpPosTracker flag_iono_grid_points_tracker_;
  MultiSbpPosTracker flag_iono_grid_points_los_tracker_;
  MultiSbpPosTracker flag_iono_tile_sat_los_tracker_;
  MultiSbpPosTracker flag_tropo_grid_points_tracker_;
  MultiSbpPosTracker flag_satellites_tracker_;

  // Week number setter
  internal::Timestamp last_seen_timestamp_;
  bool last_seen_timestamp_is_set_ = false;

  struct IodInfo {
    uint8_t iod = 0;
    uint8_t solution_id = 0;
  };

  IodInfo last_sat_iod_;
  bool last_sat_iod_is_set_ = false;

  IodInfo last_atmo_iod_;
  bool last_atmo_iod_is_set_ = false;
};

bool df389_decode(SbpMsg *const buff, float *const value);

/*
 * This function helps to decode the coefficients in stec polynomial message, by
 * decoding the rtcm message and multiply with scale factor
 */
bool stec_polynomial_read_coefficient(SbpMsg *const buff,
                                      const float scale_factor,
                                      float *const out_coefficient);

}  // namespace rtcm_decoder
}  // namespace swift

#endif  //  SWIFT_SSR2LOS_SRC_RTCM_DECODER_SBP_DECODER_H
