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

#ifndef SWIFT_SSR2LOS_SRC_RTCM_DECODER_RTCM_DECODER_H
#define SWIFT_SSR2LOS_SRC_RTCM_DECODER_RTCM_DECODER_H

#include <swift/inputs.h>
#include <swift/outputs.h>

#include <rtcm_decoder/bits/bitstream.h>

#include <internal_types/callback_functions.h>
#include <internal_types/code_bias.h>
#include <internal_types/orbit_clock.h>
#include <internal_types/phase_bias.h>

#include <rtcm_decoder/sbp_decoder.h>

namespace swift {
namespace rtcm_decoder {

struct RtcmHeader {
  uint16_t message_num = 0;
  uint32_t epoch_time_s = 0;
  GnssId constellation = GnssId::GPS;
  uint8_t update_interval = 0;
  bool multi_message = false;
  bool satellite_reference_datum = false;
  uint8_t iod = 0;
  uint16_t provider_id = 0;
  uint8_t solution_id = 0;
  bool dispersive_bias_consistency = false;
  bool mw_consistency = false;
  uint8_t num_sats = 0;
};

ProtocolType get_protocol_type(const uint8_t protocol_version);
SwiftRtcmType get_swift_rtcm_type(const uint16_t message_number);

class RtcmDecoder {
 public:
  RtcmDecoder() = default;
  explicit RtcmDecoder(const RtcmCallbackFunctions &callbacks);
  void reset_last_seen_timestamp();

  void reset(const RtcmCallbackFunctions &callbacks);
  ReturnCode handle_rtcm_message(const uint8_t *payload,
                                 const std::size_t payload_length);

 private:
  RtcmCallbackFunctions callbacks_;
  SbpDecoder sbp_decoder_;

  ReturnCode parse_rtcm_header(BitStream *buff, RtcmHeader *header) const;
  ReturnCode parse_orbit_and_clock_msg(
      BitStream *const buff,
      internal::SsrOrbitClockCollection *ssr_orbit_clock_collection) const;
  ReturnCode parse_code_bias_msg(
      BitStream *const buff,
      internal::SsrCodeBiasesCollection *ssr_code_bias_collection) const;
  ReturnCode parse_phase_bias_msg(
      BitStream *const buff,
      internal::SsrPhaseBiasesCollection *ssr_phase_bias_collection) const;

  bool set_week_number(internal::Timestamp *rtcm_timestamp);
  bool get_constellation(const uint16_t &msg_num, GnssId *constellation) const;

  /**
   * @brief Parse, unpack and decode RTCM3.3 GPS Ephemeris message number
   * 1019.
   *
   * @param buff Raw RTCM bitstream for GPS Ephemeris message.
   * @param gps_eph_msg Decoded GPS Ephemeris message.
   * @return ReturnCode SUCCESS, raw message was decoded. DECODE_ERROR, issue
   * with decoding, incorrect length, message id did not match. OUT_OF_BOUNDS,
   * at least one data field is out of range.
   */
  ReturnCode parse_gps_ephemeris_msg(BitStream *buff,
                                     EphemerisGps *gps_eph_msg) const;

  /**
   * @brief Parse, unpack and decode RTCM3.3 GAL Ephemeris message number
   * 1046
   *
   * @param buff Raw RTCM bitstream buffer
   * @param gps_eph_msg Will be used to store decoded galileo ephemeris message.
   * @return ReturnCode SUCCESS, raw message was decoded. DECODE_ERROR, issue
   * with decoding, incorrect length, message id did not match. OUT_OF_BOUNDS,
   * at least one data field is out of range.
   */
  ReturnCode parse_gal_ephemeris_msg(BitStream *buff,
                                     EphemerisGal *gal_eph_msg) const;

  /**
   * @brief Parse, unpack and decode RTCM3.3 BDS Ephemeris message number
   * 1042.
   *
   * @param buff Raw RTCM bitstream for BDS Ephemeris message.
   * @param gps_eph_msg Decoded BDS Ephemeris message.
   * @return ReturnCode SUCCESS, raw message was decoded. DECODE_ERROR, issue
   * with decoding, incorrect length, message id did not match. OUT_OF_BOUNDS,
   * at least one data field is out of range.
   */
  ReturnCode parse_bds_ephemeris_msg(BitStream *buff,
                                     EphemerisBds *bds_eph_msg) const;

  /**
   * @brief Parse, unpack and decode a SBP message.
   * @param buff Raw RTCM bitstream.
   * @param message_info Output parameter containing metadata about the parsed
   * message
   *
   * @return ReturnCode SUCCESS: Raw message was decoded. DECODE_ERROR: Issue
   * with decoding. INVALID_CALLBACK: Callback has not been set.
   */
  ReturnCode parse_wrapped_sbp(BitStream *buff, MessageInfo *message_info);

  /**
   * @brief Helper function which puts a decoded orbit/clock message into a
   * callback, if active.
   * @param collection Internal data struct to be broadcast to subscriber of the
   * callback.
   * @param message_info Metadata object where timestamp of message will
   * be populated if message is transferred.
   *
   * @return ReturnCode SUCCESS: Message was transferred. UNKNOWN_WEEK_NUMBER:
   * Could not set correct week number.
   */
  ReturnCode broadcast_orbit_clock(
      internal::SsrOrbitClockCollection *collection, MessageInfo *message_info);
  /**
   * @brief Helper function which puts a decoded code bias message into a
   * callback, if active.
   * @param collection Internal data struct to be broadcast to subscriber of the
   * callback.
   * @param message_info Metadata object where timestamp of message will
   * be populated if message is transferred.
   *
   * @return ReturnCode SUCCESS: Message was transferred. UNKNOWN_WEEK_NUMBER:
   * Could not set correct week number.
   */
  ReturnCode broadcast_code_bias(internal::SsrCodeBiasesCollection *collection,
                                 MessageInfo *message_info);

  /**
   * @brief Helper function which puts a decoded phase bias message into a
   * callback, if active.
   * @param collection Internal data struct to be broadcast to subscriber of the
   * callback.
   * @param message_info Metadata object where timestamp of message will
   * be populated if message is transferred.
   *
   * @return ReturnCode SUCCESS: Message was transferred. UNKNOWN_WEEK_NUMBER:
   * Could not set correct week number.
   */
  ReturnCode broadcast_phase_bias(
      internal::SsrPhaseBiasesCollection *collection,
      MessageInfo *message_info);

  /**
   * @brief Parse, unpack and decode a Swift RTCM message.
   * @param bitstream Raw RTCM bitstream.
   * @param message_info Output parameter containing metadata about the parsed
   * message
   *
   * @return ReturnCode SUCCESS: Raw message was decoded. DECODE_ERROR: Issue
   * with decoding. UNKNOWN_WEEK_NUMBER: Could not set correct week number.
   */
  ReturnCode parse_wrapped_swift_rtcm(BitStream *bitstream,
                                      MessageInfo *message_info);

  /**
   * @brief Parse, unpack and decode a Swift RTCM header message.
   * @param buff Raw RTCM bitstream.
   * @param header Output data struct.
   *
   * @return ReturnCode SUCCESS: Raw message was decoded. DECODE_ERROR: Issue
   * with decoding.
   */
  ReturnCode parse_swift_rtcm_header(BitStream *buff, RtcmHeader *header) const;

  /**
   * @brief Parse, unpack and decode a Swift RTCM orbit/clock message.
   * @param buff Raw RTCM bitstream.
   * @param swift_rtcm_header Decoded Swift RTCM header.
   * @param ssr_orbit_clock_collection Output data struct.
   *
   * @return ReturnCode SUCCESS: Raw message was decoded. DECODE_ERROR: Issue
   * with decoding.
   */
  static ReturnCode parse_swift_rtcm_orbit_clock_msg(
      BitStream *buff, const RtcmHeader *swift_rtcm_header,
      internal::SsrOrbitClockCollection *ssr_orbit_clock_collection);

  /**
   * @brief Parse, unpack and decode a Swift RTCM code bias message.
   * @param buff Raw RTCM bitstream.
   * @param swift_rtcm_header Decoded Swift RTCM header.
   * @param ssr_code_bias_collection Output data struct.
   *
   * @return ReturnCode SUCCESS: Raw message was decoded. DECODE_ERROR: Issue
   * with decoding.
   */
  static ReturnCode parse_swift_rtcm_code_bias_msg(
      BitStream *buff, const RtcmHeader *swift_rtcm_header,
      internal::SsrCodeBiasesCollection *ssr_code_bias_collection);

  /**
   * @brief Parse, unpack and decode a Swift RTCM phase bias message.
   * @param buff Raw RTCM bitstream.
   * @param swift_rtcm_header Decoded Swift RTCM header.
   * @param ssr_phase_bias_collection Output data struct.
   *
   * @return ReturnCode SUCCESS: Raw message was decoded. DECODE_ERROR: Issue
   * with decoding.
   */
  static ReturnCode parse_swift_rtcm_phase_bias_msg(
      BitStream *buff, const RtcmHeader *swift_rtcm_header,
      internal::SsrPhaseBiasesCollection *ssr_phase_bias_collection);
};

}  // namespace rtcm_decoder
}  // namespace swift

#endif  //  SWIFT_SSR2LOS_SRC_RTCM_DECODER_RTCM_DECODER_H
