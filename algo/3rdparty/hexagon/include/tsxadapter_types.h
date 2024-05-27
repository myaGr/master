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
//! \file tsxadapter_types.h
//! \brief This file contains generic type definitions for use with the TSX
//!  adapter library.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TSXADAPTER_TYPES_H_
#define TSXADAPTER_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

//! A handle to the TSX library
typedef void* TSXHandle;

//! \enum TSXAdapterResult
//! Result code returned by TSX Adapter API functions.
typedef enum TSXAdapterResult
{
   TSXADAPTER_SUCCESS = 0,          //!< The operation was successful
   TSXADAPTER_DATA_UNAVAILABLE = 1, //!< The requested data is not available
   TSXADAPTER_PARTIAL = 2,          //!< Partial/incomplete data was supplied and validity not yet verified
   TSXADAPTER_NO_DATA = 3,          //!< Provided data did not contain any desired messages
   TSXADAPTER_ERR_CALL = 256,       //!< An error was made calling the function
   TSXADAPTER_ERR_FAILED,           //!< The operation failed to execute
   TSXADAPTER_ERR_INTEGRITY,        //!< An integrity error was detected
   TSXADAPTER_ERR_NOMEMORY,         //!< Insufficient memory to perform the operation
   TSXADAPTER_ERR_UNSUPPORTED,      //!< The input data or action is not supported
   TSXADAPTER_ERR_BADCONFIG,        //!< Configuration not valid
   TSXADAPTER_ERR_INVALID,          //!< One or more input values are invalid.
   TSXADAPTER_ERR_DECRYPT_KEY_MISMATCH, //!< Failure to decrypt corrections due to key mismatch
   TSXADAPTER_ERR_GEOGRAPHICAL_MISMATCH //!< User position doesn't fall under correction region
} TSXAdapterResult;

//! GNSS Satellite constellation types
typedef enum TSXConstellationType
{
   TSX_CONSTELLATION_GPS     =  0,  //!< GPS (Navstar)
   TSX_CONSTELLATION_GALILEO =  5,  //!< Galileo
   TSX_CONSTELLATION_BEIDOU  =  6,  //!< BeiDou
   TSX_CONSTELLATION_QZSS    =  7,  //!< QZSS
   TSX_CONSTELLATION_UNKNOWN = 30   //!< Unknown constellation
} TSXConstellationType;

//! GNSS satellite signal types
typedef enum TSXSignalType
{
   TSX_SIGNAL_INVALID    = 0x0000,  //!< Invalid/unknown signal type
   TSX_SIGNAL_GPSL1CA    = 0x0021,  //!< GPS L1 C/A
   TSX_SIGNAL_GPSL1P     = 0x0022,  //!< GPS L1 P
   TSX_SIGNAL_GPSL2Y     = 0x0044,  //!< GPS L2 Y
   TSX_SIGNAL_GPSL2CM    = 0x0045,  //!< GPS L2C M
   TSX_SIGNAL_GPSL5Q     = 0x0067,  //!< GPS L5 Q
   TSX_SIGNAL_GALE1C     = 0x28C1,  //!< GAL E1 C
   TSX_SIGNAL_GALE1B     = 0x28CB,  //!< GAL E1 B   (Note: Not for TSX - Only used for navigation data)
   TSX_SIGNAL_GALE5AQ    = 0x28E2,  //!< GAL E5a Q
   TSX_SIGNAL_GALE5BQ    = 0x2903,  //!< GAL E5b Q
   TSX_SIGNAL_GALALTBOCQ = 0x2924,  //!< GAL ALTB Q
   TSX_SIGNAL_GALE6B     = 0x294C,  //!< GAL E6 B
   TSX_SIGNAL_BDSB1I     = 0x3181,  //!< BDS B1I - BDS-2/3 signal - Rinex C2I/L2I
   TSX_SIGNAL_BDSB2I     = 0x3203,  //!< BDS B2I - BDS-2 signal - Rinex C7I/L7I
   TSX_SIGNAL_BDSB3I     = 0x324D,  //!< BDS B3I - BDS-2/3 signal - Rinex C6I/L6I
   TSX_SIGNAL_BDSB2AP    = 0x32D4,  //!< BDS B2a (P) - BDS-3 signal - Rinex C5P/L5P
   TSX_SIGNAL_BDSB1CP    = 0x32B3,  //!< BDS B1C (P) - BDS-3 signal - Rinex C1P/L1P
   TSX_SIGNAL_BDSB2BI    = 0x3315,  //!< BDS B2b (I) - BDS-3 signal - Rinex C7D/L7D
   TSX_SIGNAL_QZSSL1CA   = 0x39A1,  //!< QZSS L1 C/A
   TSX_SIGNAL_QZSSL1S    = 0x39AA,  //!< QZSS L1 S
   TSX_SIGNAL_QZSSL2CM   = 0x39C3,  //!< QZSS L2C M
   TSX_SIGNAL_QZSSL2CL   = 0x39CD,  //!< QZSS L2C L
   TSX_SIGNAL_QZSSL5Q    = 0x39E4,  //!< QZSS L5 Q
   TSX_SIGNAL_QZSSL6P    = 0x3A2B   //!< QZSS L6 P
} TSXSignalType;

//! GNSS signal frequencies
typedef enum TSXFrequency
{
   TSX_FREQ_INVALID = 0x00,  //!< Not a valid frequency
   TSX_FREQ_L1      = 0x10,  //!< L1 frequency
   TSX_FREQ_L2,              //!< L2 frequency
   TSX_FREQ_L5,              //!< L5 frequency
   TSX_FREQ_E1      = 0x20,  //!< E1 frequency
   TSX_FREQ_E5A,             //!< E5a frequency
   TSX_FREQ_E5B,             //!< E5b frequency
   TSX_FREQ_ALTBOC,          //!< ALT BOC frequency
   TSX_FREQ_E6,              //!< E6 frequency
   TSX_FREQ_B1      = 0x30,  //!< B1 frequency
   TSX_FREQ_B1C,             //!< B1C frequency
   TSX_FREQ_B2A,             //!< B2a frequency
   TSX_FREQ_B2B,             //!< B2b frequency
   TSX_FREQ_B3,              //!< B3 frequency
   TSX_FREQ_ALL     = 0xFF   //!< All frequencies (i.e. frequency independent)
} TSXFrequency;

//! GNSS time
typedef struct TSXGNSSTime
{
   //! The absolute GPS week accounting for rollover since the start of the GPS
   //! epoch (NOT modulo 1024)
   uint32_t week;
   //! The number of seconds into the GPS week (valid range: 0 to 604799)
   uint32_t seconds;
} TSXGNSSTime;

//! ECEF vector in the reference frame of the corrections service
typedef struct TSXVectorECEF
{
   double x;  //!< X coordinate
   double y;  //!< Y coordinate
   double z;  //!< Z coordinate
} TSXVectorECEF;

//! User position in the reference frame of the corrections service
typedef struct TSXUserPosition
{
   TSXVectorECEF coordinates;    //!< User position coordinates (m)
   TSXGNSSTime time;             //!< Time at which this position was computed
} TSXUserPosition;

#endif // TSXADAPTER_TYPES_H_

