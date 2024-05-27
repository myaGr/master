/**
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
#ifndef SWIFT_SSR2LOS_RTCM_DECODER_CONSTANTS_H
#define SWIFT_SSR2LOS_RTCM_DECODER_CONSTANTS_H

#include <array>
#include <cmath>
#include <cstdint>

#include <rtcm_encoder/message_ids.h>

namespace swift {
namespace rtcm_decoder {

static constexpr uint8_t cBitsPerByte = 8;

static constexpr uint8_t cChainIdCount = 2;

static constexpr float cMillimeterToMeter = 0.001F;

// BDS-GPS Time Offset
static constexpr uint16_t cBdsTimeToGpsTimeSecond = 14;

static constexpr std::array<uint16_t, 16> cUpdateIntervalLookupTable = {
    {1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200,
     10800}};

// Biases scale factors
static constexpr float cCodeBiasScaleFactor = 0.01F;
static constexpr float cPhaseBiasScaleFactor = 0.0001F;
static constexpr float cYawScaleFactor = 256.F;       // DF480
static constexpr float cYawRateScaleFactor = 8192.F;  // DF481

// Rtcm Wrapped Sbp Header size
static constexpr uint8_t cWrappedSbpHeaderSize = 7;

// Satellite APC message scale factors
static constexpr float cPcoScaleFactor = 0.001F;
static constexpr float cPcvScaleFactor = 0.001F;
static constexpr uint16_t cSbpSatelliteApcArrayLength = 32U;

// STEC coefficient scale factors
static constexpr float cStecC00ScaleFactor = 0.05F;
static constexpr float cStecC01ScaleFactor = 0.02F;
static constexpr float cStecC10ScaleFactor = 0.02F;
static constexpr float cStecC11ScaleFactor = 0.02F;

// Tile coefficient scale factor
static constexpr double cTileLatCoefficient = (90.0 / 16384.0);
static constexpr double cTileLonCoefficient = (180.0 / 32768.0);
static constexpr double cTileLatSpacingCoefficient = 0.01;
static constexpr double cTileLonSpacingCoefficient = 0.01;

// Gridded correction scale factors
static constexpr float cStecResidualScaleFactor = 0.04F;
static constexpr float cStecResidualStdScaleFactor = 0.1F;
static constexpr float cAtmoGridDryScaleFactor = 0.004F;
static constexpr float cAtmoGridDryDelay = 2.3F;
static constexpr float cAtmoGridWetScaleFactor = 0.004F;
static constexpr float cAtmoGridWetDelay = 0.252F;

// Gridded correction and bounds scale factors
static constexpr float cSbpTropoPointDryBoundMeanScaleFactor = 0.005F;
static constexpr float cSbpTropoPointDryBoundStdDevScaleFactor = 0.005F;
static constexpr float cSbpTropoPointWetBoundMeanScaleFactor = 0.005F;
static constexpr float cSbpTropoPointWetBoundStdDevScaleFactor = 0.005F;
static constexpr float cSbpDegradationBoundMeanScaleFactor = 0.00005F;
static constexpr float cSbpDegradationBoundStdDevScaleFactor = 0.00005F;

// SBP constellation IDs
static constexpr uint8_t cSbpGpsConstellationId = 0;
static constexpr uint8_t cSbpGalConstellationId = 5;
static constexpr uint8_t cSbpBdsConstellationId = 3;

// Constants used for ephemeris conversions
static constexpr double cPow2_5 = 0.03125;                  // 2^-5
static constexpr double cPow2_6 = 0.015625;                 // 2^-6
static constexpr double cPow2_19 = 1.9073486328125E-06;     // 2^-19
static constexpr double cPow2_29 = 1.862645149230957E-09;   // 2^-29
static constexpr double cPow2_31 = 4.656612873077393E-10;   // 2^-31
static constexpr double cPow2_32 = 2.328306436538696E-10;   // 2^-32
static constexpr double cPow2_33 = 1.1641532182693481E-10;  // 2^-33
static constexpr double cPow2_34 = 5.82076609134674E-11;    // 2^-34
static constexpr double cPow2_43 = 1.1368683772161603E-13;  // 2^-43
static constexpr double cPow2_46 = 1.4210854715202E-14;     // 2^-46
static constexpr double cPow2_50 = 8.881784197001252E-16;   // 2^-50
static constexpr double cPow2_55 = 2.7755575615628914E-17;  // 2^-55
static constexpr double cPow2_59 = 1.73472347597681E-18;    // 2^-59
static constexpr double cPow2_66 = 1.35525271560688E-20;    // 2^-66

static constexpr double cEphemerisPi =
    3.1415926535898;  // The official value of Pi for Ephemeris messages
static constexpr double cBdsTgd2Seconds = 1e-10F;  // .1 nano seconds to seconds

// Maximum update interval we allow for any message
static const uint32_t cRtcmMaxAllowedUpdateInterval = 11;  // DF391: 900 seconds

// GPS Ephemeris valid DF range -- pre scaling
static const uint8_t cRtcmGpsMaxSatNumber = 32U;  // DF009
static const uint32_t cRtcmGpsMaxToc = 37799U;    // DF081 (604784 / 2^4)
static const uint32_t cRtcmGpsMaxToe = 37799U;    // DF093 (604784 / 2^4)

// ITRF message scale factors
static constexpr float cSbpItrfTranslationScaleFactor = 0.001F;        // [m]
static constexpr float cSbpItrfRotationScaleFactor = 0.00002F;         // ["]
static constexpr float cSbpItrfScaleCorrectionScaleFactor = 0.00001F;  // [ppm]
static constexpr float cSbpItrfRateTranslationScaleFactor = 0.00002F;  // [m/yr]
static constexpr float cSbpItrfRateRotationScaleFactor = 0.0000004F;   // ["/yr]
static constexpr float cSbpItrfRateScaleCorrectionScaleFactor =
    0.0000002F;  // [ppm/yr]

// Code and Phase Biases Bounds scale factors
static constexpr float cCodeAndPhaseBoundsScaleFactor = .005f;

// GAL Ephemeris valid DF range -- pre scaling
static const uint8_t cRtcmGalMaxSatNumber = 36U;  // DF252
static const uint32_t cRtcmGalMaxToc = 10079U;    // DF293 (604740/60)
static const uint32_t cRtcmGalMaxToe = 10079U;    // DF304 (604740/60)

// BDS Ephemeris valid DF range -- pre scaling
static const uint8_t cRtcmBdsMaxSatNumber =
    63U;  // DF488 range (we still allow IDs > 37)
static const uint32_t cRtcmBdsMaxToc = 75599U;  // DF488 (604792/2^3)
static const uint32_t cRtcmBdsMaxToe = 75599U;  // DF488 (604792/2^3)

// Leap Seconds scale factors
constexpr double cBiasScale = 2.910383045673370E-11;       // 2^-35
constexpr double cDriftScale = 4.440892098500626E-16;      // 2^-51
constexpr double cDriftRateScale = 3.388131789017201E-21;  // 2^-68

}  // namespace rtcm_decoder
}  // namespace swift

#endif  // SWIFT_SSR2LOS_RTCM_DECODER_CONSTANTS_H
