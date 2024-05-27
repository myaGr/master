///////////////////////////////////////////////////////////////////////////////
//
// COPYRIGHT NovAtel Inc, 2023. All rights reserved.
//
// No part of this software may be reproduced or modified in any form
// or by any means - electronic, mechanical, photocopying, recording,
// or otherwise - without the prior written consent of NovAtel Inc.
//
///////////////////////////////////////////////////////////////////////////////
//                            DESCRIPTION
//
//! \file tsxadapter_corrections_types.h
//! \brief This file contains type definitions for of corrections-related
//!  data structures for use with the TSX adapter library.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TSXADAPTER_CORRECTIONS_TYPES_H_
#define TSXADAPTER_CORRECTIONS_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

#include "tsxadapter_types.h"

//! \brief TSXCorrectionsSource enumerates the type of corrections sources
typedef enum TSXCorrectionsSource
{
   COR_SRC_NONE = 0,    //!< No corrections source
   COR_SRC_TSXQM,       //!< TerraStar-X QM corrections
   COR_SRC_TSXI         //!< TerraStar-Xi corrections
} TSXCorrectionsSource;

//! \brief TSXIntegrityDNUFlag provides a use / don't use indicator for integrity checks
typedef enum TSXIntegrityDNUFlag
{
   INTEGRITY_NO_INFO = 0,   //!< Data integrity information is not available.
   INTEGRITY_SAFE_FOR_USE,  //!< Integrity was monitored and data is safe for use.
   INTEGRITY_DO_NOT_USE     //!< Integrity was monitored and data is not safe for use.
} TSXIntegrityDNUFlag;

//! The SVID mask is a bit-field indicating which SVIDs are present. The mapping
//! of bit positions varies per constellation.
//! GPS: Bit 0 (LSB) = PRN 1, up to Bit 62 = PRN 63 (bit 63 (MSB) is reserved)
//! QZSS: Bit 0 (LSB) = PRN 193, up to Bit 9 = PRN 202 (the rest are reserved)
//! Galileo: Bit 0 (LSB) = SVID 1, up to Bit 35 = SVID 36 (the rest are reserved)
//! BDS: Bit 0 (LSB) = SVID 1, up to Bit 62 = SVID 63 (bit 63 (MSB) is reserved)
typedef uint64_t TSXSVIDMask;

//! A structure containing masks indicating which satellites are present for
//! each constellation.
typedef struct TSXSatellitesPresent
{
   uint32_t    total;    //!< Total number of satellites present across all constellations
   TSXSVIDMask gps;      //!< Bit-mask indicating which GPS satellites are present
   TSXSVIDMask galileo;  //!< Bit-mask indicating which Galileo satellites are present
   TSXSVIDMask bds;      //!< Bit-mask indicating which BeiDou satellites are present
   TSXSVIDMask qzss;     //!< Bit-mask indicating which QZSS satellites are present
} TSXSatellitesPresent;

//! TSX Corrections Metadata
typedef struct TSXCorrectionsMetadata
{
   //! TSX corrections source
   TSXCorrectionsSource source_type;
   //! Global server source ID (0..7)
   uint8_t source_id;
   //! Unique ID indicating the current ionosphere tile in use (0..8191)
   uint16_t tile_id;
   //! ITRF datum realization consisting of a year offset from 2014, indicating
   //! ITRF2014 to ITRF2045 (0..31)
   uint8_t itrf_realization;
} TSXCorrectionsMetadata;

//! General (non satellite-specific) TSX corrections integrity data
typedef struct TSXGeneralCorrectionsIntegrityData
{
   ///////////////////////////////////////////////////////
   //                        NOTE                       //
   // This list is currently incomplete and is still in //
   // the process of being fully defined. The content   //
   // can change in future releases.                    //
   //                                                   //
   ///////////////////////////////////////////////////////

   // ## OVERALL CORRECTIONS VALIDITY ##
   //! Flag to indicate if any corrections data in the message can be used.
   //! If set to "do not use" the entire correction message should not be used.
   TSXIntegrityDNUFlag overall_usage_flag;

   // ## OVERALL FAULT / DISCONTINUITY INDICATORS ##
   //! Flag indicating if fault / discontinuity indicators are available. This
   //! flag should be checked before using any of the fault / discontinuity
   //! indicators.
   bool have_fault_indicators;
   //! This IOD is used to indicate if the integrity monitor has detected a
   //! fault or series of faults that would required a user to reset the states
   //! of their positioning filter in order to regain full integrity.
   //! If this IOD changes relative to the value in the previously decoded
   //! correction dataset, a positioning filter reset should be carried out
   //! with all filter history being discarded.
   uint8_t filter_reset_iod;
} TSXGeneralCorrectionsIntegrityData;

//! General TSX corrections data
typedef struct TSXGeneralCorrectionsData
{
   //! Corrections Reference Time - General reference time indicating the
   //! newest reference time contained inside this corrections data set.
   TSXGNSSTime reference_time;
   //! List of satellite for which satellite-specifc corrections are available
   TSXSatellitesPresent sv_present_list;
} TSXGeneralCorrectionsData;

//! Data related to a TSX corrections dataset
typedef struct TSXCorrectionsDataset
{
   TSXCorrectionsMetadata metadata;              //!< Metadata for the corrections dataset
   TSXGeneralCorrectionsData corrections;        //!< General corrections data
   TSXGeneralCorrectionsIntegrityData integrity; //!< General corrections integrity data
} TSXCorrectionsDataset;

//! Maximum code biases for a satellite
#define TSX_MAX_CODE_BIASES (8U)
//! Maximum phase biases for a satellite
#define TSX_MAX_PHASE_BIASES (8U)

//! Code bias data for a single satellite signal
typedef struct TSXCodeBias
{
   //! Signal type for this code bias (set to TSX_SIGNAL_INVALID) if invalid
   TSXSignalType signal_type;
   //! Code bias (m)
   double bias;
} TSXCodeBias;

//! Phase bias data for a single satellite signal
typedef struct TSXPhaseBias
{
   //! Signal type for this phase bias (set to TSX_SIGNAL_INVALID) if invalid
   TSXSignalType signal_type;
   //! Phase bias (m)
   double bias;
   //! Flag to indicate whether the phase bias can be used for integer
   //! ambiguity resolution (AR) or is limited to float ambiguity estimation
   //! only.
   bool is_integer_ar_capable;
   //! This IOD indicates whether there has been a discontinuity in the phase
   //! bias estimation process. If the IOD value has changed compared to the
   //! the value in the previously decoded correction for the same phase bias,
   //! the corresponding ambiguity for this phase observable (or ambiguities
   //! of the linear combinations for which it is used) has to be reset.
   uint8_t discontinuity_iod;
} TSXPhaseBias;

//! Maximum PCV frequency corrections per satellite
#define TSX_MAX_PCV_FREQ (8U)

//! Phase centre variation data
typedef struct TSXPhaseCentreVariation
{
   //! Signal frequency to which this phase centre variation applies
   TSXFrequency frequency;
   //! Antenna phase center variation value (m)
   //! Positive when the phase center is further from the earth
   double variation;
} TSXPhaseCentreVariation;

//! Satellite-specific corrections data
typedef struct TSXSatelliteCorrectionsData
{
   // ## SATELLITE IDENTIFICATION ##
   //! GNSS constellation of this satellite
   TSXConstellationType constellation;
   //! SVID / PRN of this satellite
   uint8_t svid;

   // ## CLOCK BIAS ##
   //! Flag indicating clock bias correction data is available for this satellite
   bool have_clock_bias;
   //! Full precise satellite clock bias at the clock bias reference time (m)
   double clock_bias;
   //! Clock bias reference time
   TSXGNSSTime clock_bias_reference_time;
   //! IOD to indicate whether there has been a discontinuity in clock
   //! estimation. If the IOD value has changed compared to the value in
   //! the previously decoded clock correction, the corresponding ambiguities
   //! for this satellite have to be reset.
   uint8_t clock_bias_discontinuity_iod;

   // ## ORBIT ##
   //! Flag indicating orbit correction data is available for this satellite
   bool have_orbit;
   //! Orbit IODE (Issue of Data Ephemeris)
   uint16_t iode;
   //! Orbit position correction at the orbit reference time (m)
   //! Correction is provided as deltas from the broadcast ephemeris with IODE
   //! indicated by the "iode" data field.
   TSXVectorECEF delta_position;
   //! Orbit position correction rate (m/s)
   //! Rate of change of the "delta_position" corrections.
   TSXVectorECEF delta_position_dot;
   //! Orbit reference time
   TSXGNSSTime orbit_reference_time;

   // ## CODE BIASES ##
   //! Amount of code biases stored in the code_bias array (0..TSX_MAX_CODE_BIASES)
   uint8_t number_of_code_biases;
   //! Code biases for various satellite signals at the code bias reference time
   TSXCodeBias code_bias[TSX_MAX_CODE_BIASES];
   //! Code bias reference time
   TSXGNSSTime code_bias_reference_time;

   // ## PHASE BIASES ##
   //! Amount of phase biases stored in the phase_bias array (0..TSX_MAX_PHASE_BIASES)
   uint8_t number_of_phase_biases;
   //! Phase biases for various satellite signals at the phase bias reference time
   TSXPhaseBias phase_bias[TSX_MAX_PHASE_BIASES];
   //! Phase bias reference time
   TSXGNSSTime phase_bias_reference_time;

   // ## SLANT IONOSPHERE CORRECTIONS ##
   //! Flag indicating slant ionosphere correction data is available for this satellite
   bool have_iono;
   //! Slant ionosphere delay for the GPS L1 (1.57542GHz) frequency computed at
   //! the ionosphere reference time (m)
   double slant_iono_delay;
   //! Slant ionosphere delay standard deviation (m)
   double slant_iono_delay_std_dev;
   //! Ionosphere delay reference time
   TSXGNSSTime iono_reference_time;

   // ## PHASE CENTRE VARIATIONS ##
   //! Amount of PCV corrections stored in the pcv array (0..TSX_MAX_PCV_FREQ)
   uint8_t number_of_pcv_corrections;
   //! Phase centre variation data for various signal frequencies
   TSXPhaseCentreVariation pcv[TSX_MAX_PCV_FREQ];
   //! PCV reference time
   TSXGNSSTime pcv_reference_time;

} TSXSatelliteCorrectionsData;

//! Satellite-specific corrections integrity data
typedef struct TSXSatelliteCorrectionsIntegrityData
{
   ///////////////////////////////////////////////////////
   //                        NOTE                       //
   // This list is currently incomplete and is still in //
   // the process of being fully defined. The content   //
   // can change in future releases.                    //
   //                                                   //
   ///////////////////////////////////////////////////////

   // ## EPHEMERIS INTEGRITY ##
   //! Indicates the usability of the broadcast ephemeris for this satellite
   TSXIntegrityDNUFlag ephemeris_health;

   // ## OCB INTEGRITY ##
   //! Indicates the usability of the entire set of orbit, clock and bias
   //! corrections for this satellite.
   //! Note: All other OCB integrity metrics are not valid if the health is
   //! reported as "no info".
   TSXIntegrityDNUFlag ocb_health;

   // TBD: Orbit and clock integrity metrics

   // TBD: Code bias integrity metrics

   // TBD: Phase bias integrity metrics and missed discontinuity indicator

   // ## IONOSPHERE INTEGRITY ##
   //! Indicates the usability of the ionosphere corrections for this
   //! satellite.
   //! Note: All other ionosphere integrity metrics are not valid if the
   //! health is reported as "no info".
   TSXIntegrityDNUFlag iono_health;

   // TBD: Ionosphere integrity metrics

} TSXSatelliteCorrectionsIntegrityData;

//! Satellite-specific corrections and integrity data
typedef struct TSXSatelliteCorrectionsEntry
{
   TSXSatelliteCorrectionsData corrections;        //!< Satellite corrections data
   TSXSatelliteCorrectionsIntegrityData integrity; //!< Satellite corrections integrity data
} TSXSatelliteCorrectionsEntry;


#endif // TSXADAPTER_CORRECTIONS_TYPES_H_

