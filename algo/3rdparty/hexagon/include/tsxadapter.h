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
//! \file tsxadapter.h
//! \brief This file provides a decoder interface for TSX-i corrections.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TSXADAPTER_H_
#define TSXADAPTER_H_

#include <stdint.h>
#include <stdbool.h>

#include "tsxadapter_types.h"
#include "tsxadapter_corrections_types.h"

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

//-----------------------------------------------------------------------------
//! \brief Fetch the TSX adapter API version that the library implements.
//!
//! \param[out] api_major_version - The API major version
//! \param[out] api_minor_version - The API minor version
//!
//! \return true - The version was successfully retrieved. Output arguments
//!   are updated.
//! \return false - The version could not be retrieved. Output arguments may
//!   not be updated.
//-----------------------------------------------------------------------------
bool TSX_GetAPIVersion(
   uint16_t* api_major_version,
   uint16_t* api_minor_version);

//-----------------------------------------------------------------------------
//! \brief Fetch the TSX adapter library version.
//!
//! \param[out] product_id - The product identifier
//! \param[out] feature_version - The feature release version of the library
//! \param[out] maintenance_version - The maintenance release version of the
//!  library
//! \param[out] build_type - A character indicating the build type
//! \param[out] sequence - The sequence number of the build
//!
//! \return true - The version was successfully retrieved. Output arguments
//!   are updated.
//! \return false - The version could not be retrieved. Output arguments may
//!   not be updated.
//-----------------------------------------------------------------------------
bool TSX_GetLibraryVersion(
      uint32_t* product_id,
      uint16_t* feature_version,
      uint16_t* maintenance_version,
      char* build_type,
      uint32_t* sequence);

//-----------------------------------------------------------------------------
//! \brief Initialize the TSX adapter library. This function should be called
//!  before using the library and provides the user with a library handle that
//!  can be used with the other library calls.
//!
//! \remark This function performs dynamic memory allocation.
//!
//! \remark NOTE: This function's parameters may change in future if
//!  corrections key storage and retrieval is done by the library user.
//!
//! \param[in] configuration_buf - A pointer to a buffer containing the library
//!  configuration.
//! \param[in] configuration_size - The size of the buffer provided in
//!  configuration_buf.
//! \param[out] handle - A handle to the new library instance. This handle
//!  should be provided when making subsequent calls to the library.
//!
//! \return TSXADAPTER_SUCCESS - TSX adapter has been successfully
//!  initialized. Returned handle is valid.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_FAILED - TSX adapter initialization failed.
//! \return TSXADAPTER_ERR_NOMEMORY - Dynamic memory allocation failed during
//!  initialization.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_Initialize(
   const uint8_t* configuration_buf,
   uint32_t configuration_size,
   TSXHandle* handle);

//-----------------------------------------------------------------------------
//! \brief Update the library with the current GNSS time.
//!
//! This function should be called every time a new GNSS time becomes available
//!  from the receiver.
//!
//! \remark This time is typically obtained from a GNSS receiver and consists
//!  of the absolute GPS week number and seconds of week.
//!
//! \pre The GNSS time must be valid.
//!
//! \param[in] handle - A handle to a valid library instance.
//! \param[in] time - The current GNSS time
//!
//! \return TSXADAPTER_SUCCESS - The time has been successfully updated.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_FAILED - Update of the time failed.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_UpdateGNSSTime(
   TSXHandle handle,
   TSXGNSSTime time);

//-----------------------------------------------------------------------------
//! \brief Update the TSX corrections user position.
//!
//! This function should be called every time a new position has been computed
//! for a new GNSS epoch.
//!
//! \remark All future decoded TSX corrections data will be provided for this
//!  user position.
//!
//! \pre The user position must contain a valid solution. If at any epoch a
//!  user is unable to compute a position solution this function call should
//!  be skipped for that epoch.
//!
//! \param[in] handle - A handle to a valid library instance.
//! \param[in] position - The current user position
//
//! \return TSXADAPTER_SUCCESS - Position has been successfully updated.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_FAILED - Update of the user position failed.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_UpdateUserPosition(
   TSXHandle handle,
   TSXUserPosition position);

//-----------------------------------------------------------------------------
//! \brief Process raw navigation subframe data.
//!
//! The raw subframe includes all sync bytes, parity and CRCs as applicable.
//! The subframe content is continuous with padding added only at the end if
//! necessary to make the length an even multiple of 8 bits.
//!
//!  The following navigation data formats are supported:
//!  Constellation | Signal Type | Nav Msg Type   | Navigation Data Length
//!  --------------------------------------------------------------------
//!   GPS          | GPS L1CA    | NAV            | 38 bytes (300 bits)
//!   QZSS         | QZSS L1CA   | NAV            | 38 bytes (300 bits)
//!   Galileo      | GAL E1B     | INAV page part | 15 bytes (120 bits)
//!                |             |                |
//!                | GAL E1B     | INAV page      | 30 bytes (240 bits)
//!                |             |                |
//!
//!  The message formats are explained below. The first bit of each
//!  message will correspond to the most significant bit (MSB) of the
//!  first byte in the navigation message buffer.
//!  Bit number:   | 1      8 | 9     16 | ...
//!  Input buffer: |  Byte 1  |  Byte 2  | ...
//!                | MSB  LSB | MSB  LSB | ...
//!
//!  # GPS & QZSS - NAV Message # (300 bit subframe)
//!  | 1                | 31               | ... | 271           300 |
//!  | Word 1 (30 bits) | Word 2 (30 bits) | ... | Word 10 (30 bits) |
//!  Each 30-bit word in the subframe consists of 24 data bits followed
//!  by 6 parity bits.
//!  See IS-GPS-200K for more information.
//!
//!  # Galileo - INAV Page Part # (120 bit page part)
//!  | 1                                   | 115       120 |
//!  |     I/NAV Page Part (114 bits)      | Tail (6 bits) |
//!  | EO | PT | Page Part data (112 bits) |               |
//!     |    |
//!     |    |- Page Type (1 bit)
//!     |- Even / Odd Page (1 bit)
//!  See Galileo-OS-SIS-ICD Specification Issue 1.3 for more information.
//!
//!  # Galileo - INAV Page # (240 bit page - even and odd page parts)
//!  | 1                           120 | 121                        240 |
//!  | I/NAV Even Page Part (120 bits) | I/NAV Odd Part Part (120 bits) |
//!  This convenience format supports an even and odd page part fed into
//!  the library as one message. See detailed definition of page parts
//!  in the 120 bit I/NAV page part format shown above.
//!  See Galileo-OS-SIS-ICD Specification Issue 1.3 for more information.
//!
//! \param[in] handle - A handle to a valid library instance.
//! \param[in] constellation - The GNSS constellation that the subframe applies
//!  to.
//! \param[in] signal_type - The GNSS signal type that the subframe applies to.
//! \param[in] svid - The satellite SVID or PRN (as appropriate for the
//!  constellation).
//! \param[in] subframe_buffer - A buffer containing the raw ephemeris
//!  subframe.
//! \param[in] subframe_buffer_length - The length in bytes of the
//!  subframe_buffer. Should not exceed 64 bytes and will depend on the
//!  constellation and navigation data type.
//!
//! \return TSXADAPTER_SUCCESS - Navigation subframe data has been
//!  successfully processed and a new ephemeris dataset was processed.
//! \return TSXADAPTER_PARTIAL - A valid ephemeris subframe was received but
//!  does not yet form a complete ephemeris dataset for this satellite.
//! \return TSXADAPTER_ERR_UNSUPPORTED - The provided ephemeris data is not
//!  supported.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_INVALID - The provided ephemeris data is not valid.
//! \return TSXADAPTER_ERR_FAILED - Processing of the navigation data failed.
//! \return TSXADAPTER_ERR_BADCONFIG - The ephemeris could not be processed
//!  because the library configuration is invalid or expired.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_ProcessNavigationData(
   TSXHandle handle,
   TSXConstellationType constellation,
   TSXSignalType signal_type,
   uint8_t svid,
   const uint8_t* subframe_buffer,
   uint32_t subframe_buffer_length);

//-----------------------------------------------------------------------------
//! \brief Process received TSX corrections data.
//!
//! Decodes and processes TSX corrections data. If a new corrections dataset
//! was decoded that is usable at the current user position the function
//! returns TSXADAPTER_SUCCESS and provides  corrections dataset information
//! including a list of satellites for which corrections are available via the
//! output "corrections" parameter.
//! In all other cases the "corrections" output parameter is not modified.
//!
//! Satellite-specific corrections from the latest dataset can be retrieved
//! using the TSX_GetSatelliteCorrectionsData() function.
//!
//! \remark Note that when a new corrections dataset is successfully decoded
//!  and TSXADAPTER_SUCCESS is returned, all previously stored corrections
//!  data is cleared and overwritten with the data from the new dataset.
//!
//! \param[in] handle - A handle to a valid library instance.
//! \param[in] encoded_corrections_data_buffer - A buffer containing
//!  corrections data. Must contain at least the number of bytes indicated by
//!  data_buffer_length.
//! \param[in] data_buffer_length - The number of bytes contained in the buffer
//!  pointed to by corrections_data_buffer.
//! \param[out] decoded_corrections - General corrections and integrity data
//!  obtained from the newly decoded corrections dataset. This includes a list
//!  of satellites for which new satellite-specific corrections are available.
//!  This structure is only populated when TSXADAPTER_SUCCESS is returned.
//! \param[out] user_key_updated - A flag to indicate if a new corrections
//!  encryption key was decoded and processed. [RESERVED - NOT IMPLEMENTED]
//!
//! \return TSXADAPTER_SUCCESS - The provided data was processed and at
//!  least one full message was successfully decoded. The new data was written
//!  to the "decoded_corrections" output parameter. The dataset has also been
//!  verified to be usable for the current user position.
//! \return TSXADAPTER_PARTIAL - The provided data contains a partial message
//!  but it is still incomplete. Note that the message contents has not yet
//!  been checked.
//! \return TSXADAPTER_NO_DATA - The provided data did not contain any
//!  desired corrections message data.
//! \return TSXADAPTER_ERR_INTEGRITY - The provided data contained at least
//!  one valid framed message but failed a data integrity check.
//! \return TSXADAPTER_ERR_DECRYPT_KEY_MISMATCH - Corrections decryption
//!  failed due to an invalid user key. If the issue persists please connect
//!  to a user key update stream and use TSX_ProcessTSXCorrectionsKeyUpdateData().
//! \return TSXADAPTER_ERR_GEOGRAPHICAL_MISMATCH - A corrections dataset was
//!  decoded but the corrections data is not valid for the current user
//!  position. The "decoded_corrections" output parameter was not updated.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_BADCONFIG - The corrections data could not be
//!  processed because the library configuration is invalid or expired.
//! \return TSXADAPTER_ERR_FAILED - Processing of the corrections data failed.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_ProcessCorrectionsData(
   TSXHandle handle,
   const uint8_t* encoded_corrections_data_buffer,
   uint32_t data_buffer_length,
   TSXCorrectionsDataset* decoded_corrections,
   bool* user_key_updated);

//-----------------------------------------------------------------------------
//! \brief Process a secondary user key update data stream in order to update
//!  the corrections user key used for decrypting the main corrections data
//!  stream.
//!
//! This function should be fed with data received from the correction service
//! key update stream. This should only be done in cases where a key update is
//! required as indicated by receiving TSXADAPTER_ERR_DECRYPT_KEY_MISMATCH
//! from TSX_ProcessTSXCorrectionsData(). When a new key was received the
//! user_key_updated flag is set and TSXADAPTER_SUCCESS is returned. The user
//! can then stop providing data from the key update stream.
//!
//! \param[in] handle - A handle to a valid library instance.
//! \param[in] key_update_data_buffer - A buffer containing key update data
//!  received on the key update stream. Must contain at least the number of
//!  bytes indicated by data_buffer_length.
//! \param[in] data_buffer_length - The number of bytes contained in the buffer
//!  pointed to by key_update_data_buffer.
//! \param[out] user_key_updated - A flag to indicate if a new corrections
//!  encryption key was decoded and processed. If this flag is set the library
//!  user can disconnect from the key update stream and stop providing data to
//!  this interface. [RESERVED - NOT IMPLEMENTED]
//!
//! \return TSXADAPTER_SUCCESS - The provided data was processed successfully
//!  and a new corrections user key was obtained.
//! \return TSXADAPTER_PARTIAL - The provided data contains a partial message
//!  but it is still incomplete. Note that the message contents has not yet
//!  been checked.
//! \return TSXADAPTER_NO_DATA - The provided data did not contain any
//!  desired key update message data.
//! \return TSXADAPTER_ERR_INTEGRITY - The provided data contained at least
//!  one valid framed message but failed a data integrity check.
//! \return TSXADAPTER_ERR_DECRYPT_KEY_MISMATCH - Key update message
//!  decryption failed due to an invalid user key. This typically should not
//!  happen since old user keys should be supported for a long time on the key
//!  update stream.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_FAILED - Processing of the key update data failed.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_ProcessCorrectionsKeyUpdateData(
   TSXHandle handle,
   const uint8_t* key_update_data_buffer,
   uint32_t data_buffer_length,
   bool* user_key_updated);

//-----------------------------------------------------------------------------
//! \brief Get decoded corrections data for a specified satellite.
//!
//! When a new corrections dataset was decoded using
//! TSX_ProcessTSXCorrectionsData(), this function is used to retrieve
//! corrections from this dataset for a specific satellite.
//!
//! \param[in] handle - A handle to a valid library instance.
//! \param[in] constellation - The GNSS constellation of the satellite.
//! \param[in] svid - The satellite SVID or PRN (as appropriate for the
//!  constellation) for which corrections data is required.
//! \param[out] sv_corrections_entry - A structure containing all the
//!  corrections and integrity data applicable to the requested satellite.
//!
//! \return TSXADAPTER_SUCCESS - Corrections data was retrieved for the
//!  requested satellite and written to the sv_corrections_entry parameter.
//! \return TSXADAPTER_DATA_UNAVAILABLE - Corrections data is not available
//!  for the requested satellite.
//! \return TSXADAPTER_ERR_CALL - One or more arguments to this
//!  call was invalid.
//! \return TSXADAPTER_ERR_FAILED - Retrieval of the corrections entry failed.
//-----------------------------------------------------------------------------
TSXAdapterResult TSX_GetSatelliteCorrectionsEntry(
   const TSXHandle handle,
   TSXConstellationType constellation,
   uint8_t svid,
   TSXSatelliteCorrectionsEntry* sv_corrections_entry);


#ifdef __cplusplus
}
#endif //__cplusplus


#endif // TSXADAPTER_H_
