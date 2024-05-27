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

#ifndef SWIFT_SSR_CLIENT_OUTPUTS_H
#define SWIFT_SSR_CLIENT_OUTPUTS_H

#include <array>
#include <cstdint>
#include <functional>

#include <swift/inputs.h>

namespace swift {

enum class ReturnCode : uint8_t {
  // Operation completed successfully
  SUCCESS,
  // RTCM decoding error
  DECODE_ERROR,
  // Incoming RTCM sequence not consistent
  BROKEN_SEQUENCE,
  // Internal error
  INTERNAL_ERROR,
  // Provided rover position is out of bounds of the currently
  // available SSR data
  OUT_OF_BOUNDS,
  // SSR data is not available or incomplete. Corrections can't
  // be calculated
  CORR_UNAVAILABLE,
  // Invalid callback set
  INVALID_CALLBACK,
  // Unsupported chain ID present
  UNSUPPORTED_CHAIN_ID,
  // Week number not set
  UNKNOWN_WEEK_NUMBER,
};

/// CONSTANTS
// ITRF message
static constexpr std::size_t cRtcmItrfMaxSourceNameSize = 32;
static constexpr std::size_t cRtcmItrfMaxTargetNameSize = 32;
// Satellite flags
constexpr static uint16_t cSatIntegrityFailedBit = 0x1000;  // The twelfth bit
constexpr static uint16_t cAtmoIntegrityFailedBit =
    0x2000;  // The thirteenth bit

struct CorrectionStdDev {
  float value = 0.0;
  bool valid = false;
};

struct SsrCorrection {
  float corr = 0.0;
  float bound_mean = 0.0;
  float bound_std_dev = 0.0;
  int32_t age_ms = 0;
  int32_t age_of_integrity_ms = 0;
  bool valid = false;
  CorrectionStdDev corr_std_dev;
};

struct SsrSignalCorrection {
  uint8_t signal_code = 255;
  SsrCorrection correction;
};

struct SatelliteCorrection {
  SatelliteDescription sv_desc;
  uint32_t tow_ms = 0;
  uint16_t wn = 0;
  uint16_t flags = 0;
  // Indicates validity of fields that do not have their own validity flags
  bool valid = false;

  SsrCorrection orbit;
  SsrCorrection clock;
  std::array<SsrSignalCorrection, cMaxSignals> code_bias;
  std::array<SsrSignalCorrection, cMaxSignals> phase_bias;
  SsrCorrection iono;
  SsrCorrection tropo;
};

struct LosCorrections {
  uint16_t ssr_provider_id = 0;
  uint16_t flags = 0;
  uint32_t latest_corr_tow_ms = 0;
  uint16_t latest_corr_wn = 0;
  std::array<SatelliteCorrection, cMaxSats> sat_correction_block;
};

/**
 * @brief GPS Ephemeris RTCM message number 1019.
 * based on RTCM 10403.3 Version 3 table 3.5-21.
 * This struct is used for decoded/scaled message. All data field with units are
 * in standard SI units: meters, seconds, radians.
 */
struct EphemerisGps {
  bool valid = false;
  uint8_t sat_id = 0;  /* DF009 Satellite ID */
  uint16_t wn = 0;     /* DF076 Week number*/
  uint8_t ura = 0;     /* DF077 User range accuracy */
  uint8_t code_l2 = 0; /* DF078 Code on L2 */
  double idot = 0;     /* DF079 Rate of Inclination Angle */
  uint8_t iode = 0;    /* DF071 Idex of Data, Ephemeris */
  uint32_t toc = 0;    /* DF081 Clock  data reference time */
  double af2 = 0;      /* DF082 Clock  drift rate */
  double af1 = 0;      /* DF083 Clock  drift */
  double af0 = 0;      /* DF084 Clock  bias */
  uint16_t iodc = 0;   /* DF085 Issue of Data, Clock */
  double crs = 0;      /* DF086 Sine harmonic correction to orbit radius */
  double dn = 0;       /* DF087 Mean motion difference from computer value */
  double m0 = 0;       /* DF088 Mean Anomaly at reference time*/
  double cuc = 0;      /* DF089 Amplitude of the Cosine Harmonic Correction
                          Term to the Argument of Latitude */
  double ecc = 0;      /* DF090 Eccentricity */
  double cus = 0;      /* DF091 Amplitude of the Sine Harmonic Correction
                          Term to the Argument of Latitude */
  double sqrt_a = 0;   /* DF092 Square root of the semi-major axis */
  uint32_t toe = 0;    /* DF093 Reference time of ephemeris */
  double cic = 0; /* DF094 Cosine harmonic correction to angle of inclination */
  double omega0 = 0; /* DF095 Longitude of Ascending Node of Orbit Plane
                       at Weekly Epoch */
  double cis = 0;   /* DF096 Sine harmonic correction to angle of inclination */
  double i0 = 0;    /* DF097 Inclination Angle at Reference Time */
  double crc = 0;   /* DF098 Amplitude of the Cosine Harmonic Correction
                     Term to the Orbit Radius */
  double omega = 0; /* DF099 Argument of perigee */
  double omega_dot = 0;      /* DF100 Rate of right ascension */
  double tgd = 0;            /* DF101 Group delay */
  uint8_t health = 0;        /* DF102 Satellite health */
  bool l2_data_bit = false;  /* DF103 L2 P data flag */
  bool fit_interval = false; /* DF137 Fit interval */
};

/**
 * @brief Galileo Ephemeris RTCM message number 1046.
 * Based on RTCM 10403.3 Version 3 table 3.5-111.
 * This struct is used for decoded/scaled message. All data field with units are
 * in standard SI units: meters, seconds, radians.
 */
struct EphemerisGal {
  bool valid = false;
  uint8_t sat_id = 0;     /* DF252 Satellite ID */
  uint16_t wn = 0;        /* DF289 Week number*/
  uint16_t iodn = 0;      /* DF290 Issue of Data for navigation data uint10*/
  uint8_t sisa_e1e5b = 0; /* DF286 Signal In Space Accuracy */
  double idot = 0;        /* DF292 Rate of inclination int14 */
  uint32_t toc = 0;       /* DF293 Clock data reference time */
  double af2 = 0;         /* DF294 Clock correction drift rate */
  double af1 = 0;         /* DF295 Clock correction drift */
  double af0 = 0;         /* DF296 Clock correction bias */
  double crs = 0;        /* DF297 Amplitude of the sine harmonic correction term
                             to the orbit radius */
  double dn = 0;         /* DF298 Mean motion difference from computed value */
  double m0 = 0;         /* DF299 Mean anomaly at reference time*/
  double cuc = 0;        /* DF300 Amplitude of the cosine harmonic correction
                             term to the argument of latitude */
  double ecc = 0;        /* DF301 Eccentricity */
  double cus = 0;        /* DF302 Amplitude of the sine harmonic correction term
                             to the argument of latitude */
  double sqrt_a = 0;     /* DF303 Square root of the semi-major axis */
  uint32_t toe = 0;      /* DF304 Reference time of ephemeris */
  double cic = 0;        /* DF305 Amplitude of the cosine harmonic correction
                             term to the angle of inclination */
  double omega0 = 0;     /* DF306 Longitude of ascending node of orbital
                             plane at weekly epoch */
  double cis = 0;        /* DF307 Amplitude of the sine harmonic correction term
                             to the angle of inclination */
  double i0 = 0;         /* DF308 Inclination angle at reference time */
  double crc = 0;        /* DF309 Amplitude of the cosine harmonic correction
                             term to the orbit radius */
  double omega = 0;      /* DF310 Argument of perigee */
  double omega_dot = 0;  /* DF311 Rate of right ascension */
  double bgd_e5a_e1 = 0; /* DF312 Broadcast Group Delay E1/E5a */
  double bgd_e5b_e1 = 0; /* DF313 Broadcast Group Delay E5b/E1 */
  uint8_t sh_e5b = 0;    /* DF316 E5b Signal Health Status */
  bool dv_e5b =
      false; /* DF317 Navigation data validity status transmitted on E5b */
  uint8_t sh_e1b = 0; /* DF287 E1b Signal Health Status */
  bool dv_e1b =
      false; /* DF288 Navigation data validity status transmitted on E1b */
  uint8_t reserved = 0; /* DF001 Reserved*/
};

/**
 * @brief BDS Ephemeris RTCM message number 1042,
 * based on RTCM 10403.3 Version 3 table 3.5-113
 * This struct is used for decoded/scaled message. All data field with units are
 * in standard SI units: meters, seconds, radians.
 */
struct EphemerisBds {
  bool valid = false;
  uint8_t sat_id = 0;   /* DF488 Satellite ID */
  uint16_t wn = 0;      /* DF489 Week number*/
  uint8_t urai = 0;     /* DF490 User range accuracy */
  double idot = 0;      /* DF491 Rate of inclination angle */
  uint8_t aode = 0;     /* DF492 Age of data, Ephemeris */
  uint32_t toc = 0;     /* DF493 The reference time of clock parameters */
  double af2 = 0;       /* DF494 Clock correction parameter drift rate */
  double af1 = 0;       /* DF495 Clock correction parameter drift */
  double af0 = 0;       /* DF496 Clock correction parameter bias */
  uint8_t aodc = 0;     /* DF497 Age of Data Clock */
  double crs = 0;       /* DF498 Amplitude of sine harmonic correction term to
                            the orbit radius */
  double dn = 0;        /* DF499 Mean motion difference from computed value */
  double m0 = 0;        /* DF500 Mean Anomaly at reference time*/
  double cuc = 0;       /* DF501 Amplitude of cosine harmonic correction term
                            to the argument of latitude */
  double ecc = 0;       /* DF502 Eccentricity */
  double cus = 0;       /* DF503 Amplitude of sine harmonic correction term to
                            the argument of latitude */
  double sqrt_a = 0;    /* DF504 Square root of semi-major axis */
  uint32_t toe = 0;     /* DF505 Ephemeris reference time */
  double cic = 0;       /* DF506 Amplitude of cosine harmonic correction term
                            to the angle of inclination */
  double omega0 = 0;    /* DF507 Longitude of Ascending Node of Orbit Plane
                            at Weekly Epoch */
  double cis = 0;       /* DF508 Amplitude of sine harmonic correction term to
                            the angle of inclination */
  double i0 = 0;        /* DF509 Inclination angle at reference time */
  double crc = 0;       /* DF510 Amplitude of cosine harmonic correction term
                            to the orbit radius */
  double omega = 0;     /* DF511 Argument of perigee */
  double omega_dot = 0; /* DF512 Rate of right ascension */
  double tgd1 = 0;      /* DF513 Equipment Group Delay Differential */
  double tgd2 = 0;      /* DF514 Equipment Group Delay Differential */
  bool health = false;  /* DF515 Satellite health */
};

/**
 * @brief ITRF RTCM message number 580.
 * based on Swift Navigation Binary Protocol table 7.5-84.
 * This struct is used for decoded/scaled message.
 */
struct ItrfModel {
  uint8_t ssr_iod = 0;                           // SSR IOD parameter
  char source_name[cRtcmItrfMaxSourceNameSize];  // Source-Name
  char target_name[cRtcmItrfMaxTargetNameSize];  // Target-Name
  uint8_t system_id_number = 0;              // System Identiﬁcation Number
  uint16_t utilized_transformation_msg = 0;  // Utilized Transformation Message
  uint16_t reference_epoch_t0 =
      0;              // Reference Epoch t0 for transformation parameter set
                      // given as Modiﬁed Julian Day (MDJ)
  double dx0 = 0;     // Translation in X for Reference Epoch t0 [m]
  double dy0 = 0;     // Translation in Y for Reference Epoch t0
  double dz0 = 0;     // Translation in Z for Reference Epoch t0
  double ro1 = 0;     // Rotation around the X-axis for Reference Epoch t0 [”]
  double ro2 = 0;     // Rotation around the Y-axis for Reference Epoch t0
  double ro3 = 0;     // Rotation around the Z-axis for Reference Epoch t0
  double ds0 = 0;     // Scale correction for Reference Epoch t0 [ppm]
  double dot_dx = 0;  // Rate of change of translation in X [m/yr]
  double dot_dy = 0;  // Rate of change of translation in Y
  double dot_dz = 0;  // Rate of change of translation in Z
  double dot_r1 = 0;  // Rate of change of rotation around the X-axis [”/yr]
  double dot_r2 = 0;  // Rate of change of rotation around the Y-axis
  double dot_r3 = 0;  // Rate of change of rotation around the Z-axis
  double dot_ds = 0;  // Rate of change of scale correction [ppm/yr]
};

struct LeapSeconds {
  double bias_coeff = 0;
  double drift_coeff = 0;
  double drift_rate_coeff = 0;
  int8_t count_before = 0;
  uint16_t tow_s = 0;
  uint16_t wn = 0;
  uint16_t ref_wn = 0;
  uint8_t ref_dn = 0;
  int8_t count_after = 0;
};

struct AcknowledgeMessage {
  uint8_t request_id = 0;
  uint32_t area_id = 0;
  uint8_t response_code = 0;
  uint16_t correction_mask_on_demand = 0;
  uint16_t correction_mask_stream = 0;
  uint8_t solution_id = 0;
};

struct SatelliteIode {
  SatelliteDescription sat_id;
  uint16_t iode = 0;

  bool operator==(const SatelliteIode &other) const {
    return ((this->sat_id == other.sat_id) && (this->iode == other.iode));
  }
};

struct MessageInfo {
  enum class WrappedContentType { NONE, SBP, SWIFT_RTCM };

  uint16_t rtcm_message_type = 0;
  WrappedContentType wrapped_type = WrappedContentType::NONE;
  // If the wrapped_type is set to either SBP or SWIFT_RTCM,
  // then the field wrapped_message_type is set to the type of the inner message
  uint16_t wrapped_message_type = 0;
  // The below timestamp values are only to be read if timestamp_valid is set to
  // true
  bool timestamp_valid = false;
  uint32_t tow_s = 0;
  uint16_t wn = 0;

  bool update_interval_valid = false;
  uint16_t update_interval_s = 0;
};

struct DecoderCallbacks {
  std::function<void(const ItrfModel &itrf_model_msg)> itrf_model_callback =
      nullptr;
  std::function<void(const EphemerisGps &gps_eph_msg)> gps_eph_msg_callback =
      nullptr;
  std::function<void(const EphemerisGal &gal_eph_msg)> gal_eph_msg_callback =
      nullptr;
  std::function<void(const EphemerisBds &bds_eph_msg)> bds_eph_msg_callback =
      nullptr;
  std::function<void(const LeapSeconds &leap_seconds)> leap_seconds_callback =
      nullptr;
  std::function<void(const AcknowledgeMessage &acknowledge_msg)>
      acknowledge_msg_callback = nullptr;
  std::function<void(const MessageInfo &message_info)> message_info_callback =
      nullptr;
};

struct CorrectionCallbacks {
  std::function<std::size_t(
      const uint32_t tow_ms, const uint16_t wn,
      const std::size_t satellite_iodes_count,
      const std::array<SatelliteIode, cMaxSats> &satellite_iodes,
      std::array<SatellitePva, cMaxSats> *sat_states)>
      satellite_states_callback = nullptr;
};

struct CallbackFunctions {
  DecoderCallbacks decoder_callbacks;
  CorrectionCallbacks correction_callbacks;
};

}  // namespace swift

#endif  // SWIFT_SSR_CLIENT_OUTPUTS_H
