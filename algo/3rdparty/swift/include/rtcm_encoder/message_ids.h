/**
 * Copyright (C) 2023 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef RTCM_ENCODER_MESSAGE_IDS_H
#define RTCM_ENCODER_MESSAGE_IDS_H

#include <array>
#include <cmath>
#include <cstdint>

namespace swift {
namespace rtcm_decoder {

enum class ProtocolType {
  WRAPPED_UNKNOWN = -1,
  WRAPPED_SBP,
  WRAPPED_SWIFT_RTCM,
};

enum class SwiftRtcmType {
  SWIFT_RTCM_UNKNOWN = -1,
  SWIFT_RTCM_ORBIT_CLOCK,
  SWIFT_RTCM_CODE_BIAS,
  SWIFT_RTCM_PHASE_BIAS,
};

static constexpr uint16_t cRtcmMaxPayloadSize = 1023;

// Combined Orbit and Clock Correction message types
static constexpr uint16_t cRtcmGpsOrbitClockMsgType = 1060;
static constexpr uint16_t cRtcmGalOrbitClockMsgType = 1243;
static constexpr uint16_t cRtcmBdsOrbitClockMsgType = 1261;

// Code bias message types
static constexpr uint16_t cRtcmGpsCodeBiasMsgType = 1059;
static constexpr uint16_t cRtcmGalCodeBiasMsgType = 1242;
static constexpr uint16_t cRtcmBdsCodeBiasMsgType = 1260;

// Phase Bias message types
static constexpr uint16_t cRtcmGpsPhaseBiasMsgType = 1265;
static constexpr uint16_t cRtcmGalPhaseBiasMsgType = 1267;
static constexpr uint16_t cRtcmBdsPhaseBiasMsgType = 1270;

// Swift proprietary message
static constexpr uint16_t cRtcmSwiftProprietaryMsgType = 4062;

static constexpr uint16_t cSwiftRtcmGalOrbitClockMsgType = 3243;
static constexpr uint16_t cSwiftRtcmBdsOrbitClockMsgType = 3261;
static constexpr uint16_t cSwiftRtcmGalCodeBiasMsgType = 3242;
static constexpr uint16_t cSwiftRtcmBdsCodeBiasMsgType = 3260;
static constexpr uint16_t cSwiftRtcmGpsPhaseBiasMsgType = 3265;
static constexpr uint16_t cSwiftRtcmGalPhaseBiasMsgType = 3267;
static constexpr uint16_t cSwiftRtcmBdsPhaseBiasMsgType = 3270;

// Satellite APC message
static constexpr uint16_t cSbpSatelliteApcDepMsgType = 1540;
static constexpr uint16_t cSbpSatelliteApcMsgType = 1541;

// STEC correction dep message
static constexpr uint16_t cSbpStecCorrectionDepMsgType = 1531;
// STEC correction message
static constexpr uint16_t cSbpStecCorrectionMsgType = 1533;

// Tile definition
static constexpr uint16_t cSbpTileDefinitionDepAMsgType = 1526;
static constexpr uint16_t cSbpTileDefinitionDepBMsgType = 1527;
static constexpr uint16_t cSbpTileDefinitionMsgType = 1528;

// Gridded correction message
static constexpr uint16_t cSbpGriddedCorrectionMsgType = 1532;

// Gridded correction and bounds message
static constexpr uint16_t cSbpGriddedCorrectionBoundsMsgType = 1534;

// High Level Flags
static constexpr uint16_t cSbpFlagsHighLevelMsgType = 3001;

// Flag Grid message
static constexpr uint16_t cSbpFlagTropoGridPointMsgType = 3011;
static constexpr uint16_t cSbpFlagIonoGridPointMsgType = 3015;
static constexpr uint16_t cSbpFlagIonoGridPointSatLosMsgType = 3025;

// Ephemeris message types
static const uint16_t cRtcmGpsEphemerisMsgType = 1019;
static const uint16_t cRtcmGalEphemerisMsgType = 1046;
static const uint16_t cRtcmBdsEphemerisMsgType = 1042;

// ITRF message
static constexpr uint16_t cSbpItrfMsgType = 580;

// Orbit Clock Bounds
static constexpr uint16_t cSbpOrbitClockBoundsMsgType = 1502;
// Orbit Clock Bounds Degradation
static constexpr uint16_t cSbpOrbitClockBoundsDegradationMsgType = 1503;

// Code and Phase Biases Bounds
static constexpr uint16_t cSbpCodeAndPhaseBiasesBoundsMsgType = 1516;

// Leap Seconds
static const uint16_t cLeapSecondsMsgType = 570;

// Flag Iono Tile Sat LOS
static constexpr uint16_t cSbpFlagIonoTileSatLosMsgType = 3021;

// Flag Satellites
static constexpr uint16_t cSbpFlagSatellitesMsgType = 3005;

// Acknowledge message
static const uint16_t cAcknowledgeMsgType = 3026;

}  // namespace rtcm_decoder
}  // namespace swift

#endif  // RTCM_ENCODER_MESSAGE_IDS_H
