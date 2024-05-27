///////////////////////////////////////////////////////////////////////////////
//
// COPYRIGHT NovAtel Inc, 2019. All rights reserved.
//
// No part of this software may be reproduced or modified in any form
// or by any means - electronic, mechanical, photocopying, recording,
// or otherwise - without the prior written consent of NovAtel Inc.
//
///////////////////////////////////////////////////////////////////////////////
//                            DESCRIPTION
//
//! \file correctionsclient.h
//! \brief A library that implements a corrections client for use with NTRIP and
//!  raw corrections sources.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CORRECTIONSCLIENT_H
#define CORRECTIONSCLIENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

typedef void* CorrectionsHandle;

typedef enum
{
   CORRECTION_SERVER_NONE,
   CORRECTION_SERVER_RAW,
   CORRECTION_SERVER_NTRIP,
   CORRECTION_SERVER_TSX,
   CORRECTION_SERVER_MQTT
}CorrectionServerType;

typedef enum
{
   CONNECTION_SUCCESS,
   CONNECTION_ERR_ADDRESS,
   CONNECTION_ERR_NOTFOUND,
   CONNECTION_ERR_UNAUTHORIZED,
   CONNECTION_ERR_FAILED
}CorrectionsConnectionResult;

typedef enum
{
   CORRECTIONS_DISCONNECTED,
   CORRECTIONS_CONNECTED,
   CORRECTIONS_RECONNECTING
}CorrectionsConnectionStatus;

typedef struct
{
   const char* current_mountpoint;     //! NTRIP connection:
                                       //!    Shows the mountpoint currently in use (may be NULL)
                                       //! TSX connection:
                                       //!    Shows the full topic path for the currently subscribed tile ID.
                                       //!    The tile ID will be set to 0 if there is no active tile subscription.
                                       //!    Examples:  "TSX-QM__/001/NAM/0/6169" -> Subscribed to tile ID 6169
                                       //!               "TSX-QM__/001/NAM/0/0"    -> No active tile subscription
                                       //! MQTT connection:
                                       //!    Shows the topic that is currently subscribed.
   float data_rate_kbps;               //! The current data rate in kilobits per second
   uint32_t current_outage_time_sec;   //! The length of the current connection outage
   uint32_t current_uptime_sec;        //! The length of time that corrections have been continuously connected.
   float total_uptime_percent;         //! The percentage of time that the connection has been available
   uint64_t total_bytes_received;      //! The total number of bytes received since connect.
   uint32_t total_bytes_dropped;       //! The total number of bytes dropped since connect.
}CorrectionStatistics;

#define CORRECTIONSCLIENT_POSITION_SCALE_FACTOR (16)

//----------------------------------------------------------------------------
//! \brief Fetch the CorrectionsClient API version that the library implements.
//!
//! \param[out] api_major_version - The API major version
//! \param[out] api_minor_version - The API minor version
//
//! \return true - The version was successfully retrieved. Output arguments are
//!   updated.
//! \return false - The version could not be retrieved. Output arguments may not
//!   be updated.
//----------------------------------------------------------------------------
bool CorrectionsClient_GetAPIVersion(uint16_t* api_major_version, uint16_t* api_minor_version);

//----------------------------------------------------------------------------
//! \brief Fetch the CorrectionsClient library version
//!
//! \param[out] product_id - The product identifier
//! \param[out] feature_version - The feature release version of the library
//! \param[out] maintenance_version - The maintenance release version of the library
//! \param[out] build_type - A character indicating the build type
//! \param[out] sequence - The sequence number of the build
//
//! \return true - The version was successfully retrieved. Output arguments are
//!   updated.
//! \return false - The version could not be retrieved. Output arguments may not
//!   be updated.
//----------------------------------------------------------------------------
bool CorrectionsClient_GetLibraryVersion(
      uint32_t* product_id,
      uint16_t* feature_version,
      uint16_t* maintenance_version,
      char* build_type,
      uint32_t* sequence);

//-----------------------------------------------------------------------------
//! \brief Initialize a corrections client
//
//! \remark This function performs dynamic memory allocation.
//
//! \param[in] hostname - A null-terminated string containing the server hostname
//!   to connect to.
//! \param[in] port - A null-terminated string containing the server port to connect to
//! \param[in] server_type - The type of correction server being connected to.
//! \param[out] handle - A handle to the data corrections client instance. This should
//! be used for subsequent calls.
//
//! \return true - The initialization is successful and the handle is set.
//! \return false - The initialization failed. The handle should not be used.
//-----------------------------------------------------------------------------
bool CorrectionsClient_Init(
      const char* hostname,
      const char* port,
      CorrectionServerType server_type,
      CorrectionsHandle* handle);

//-----------------------------------------------------------------------------
//! \brief Destroy the corrections client
//
//! \remark This function performs memory deallocation.
//
//! \param[in] handle - A handle to the client to destroy.
//-----------------------------------------------------------------------------
void CorrectionsClient_Destroy(CorrectionsHandle handle);

//--------------------------------------------------------------------------
//! \brief Set the NTRIP mountpoint to stream corrections from for NTRIP
//!    connections, or set the subscription topic for MQTT connections.
//
//! This is only applicable to NTRIP, MQTT and TSX correction servers.
//! Calling this function will have no effect on other client types.
//
//! \remark If the mountpoint is not set prior to connecting, then the client
//!   will attempt to return the server source table rather than streaming
//!   corrections for NTRIP connections.
//
//! \param[in] handle - A handle to the client for an
//!    NTRIP, MQTT or TSX connection.
//! \param[in] mount_point
//!    NTRIP connection:
//!       The mountpoint to stream corrections from.
//!    TSX connection:
//!       A string representing the coverage field.
//!       Examples: "NAM", "EUR"
//!       The default is "NAM" for TSX connections if this is not set
//!       by the user.
//!    MQTT connection:
//!      The topic to subscribe to.
//!
//! \retval CONNECTION_SUCCESS
//!    The operation was successful.
//! \retval CONNECTION_ERR_FAILED
//!    The operation failed.  The input parameter may be badly formed.
//--------------------------------------------------------------------------
CorrectionsConnectionResult CorrectionsClient_SetMountpoint(
      CorrectionsHandle handle,
      const char* mount_point);

//--------------------------------------------------------------------------
//! \brief Set the username and password credentials for
//!    NTRIP, MQTT and TSX connections.
//
//! This is only applicable to NTRIP, MQTT and TSX correction servers.
//! Calling this function will have no effect on other client types.
//
//! \remark If the credentials are not set prior to connecting, then the client
//!   will attempt to connect anonymously.
//
//! \param[in] handle - A handle to the client to assign the credentials to.
//! \param[in] szUsername_ - The username to use for corrections streaming.
//! \param[in] szPassword_ - The password to use for corrections streaming.
//--------------------------------------------------------------------------
void CorrectionsClient_SetCredentials(
      CorrectionsHandle handle,
      const char* username,
      const char* password);

//--------------------------------------------------------------------------
//! \brief Set the path to a directory containing the PEM encoded
//!    trusted CA certificate files.
//!
//! The default path for the certificate files are as follows:
//!    Windows:      "C:\tsx_certfiles"
//!    Non-windows:  "/etc/ssl/certs"
//!
//! This call will override the default path.
//!
//! \remark This is relevant for TSX and MQTT connections.
//--------------------------------------------------------------------------
void CorrectionsClient_SetCertificatePath(
      CorrectionsHandle handle,
      const char* certificate_path );

//--------------------------------------------------------------------------
//! \brief Select the best tile to subscribe to for data corrections
//!    based on the provided position.
//!
//! This is only valid for TSX connections.  Corrections will not be
//! provided until this function has been called in order to determine
//! which tile to subscribe to.
//!
//! This function can result in no tile subscriptions if the provided position
//! falls outside of the coverage area indicated by the tile list of
//! the corrections server.
//!
//! \param[in] scaled_latitude
//!    The position latitude in decimal degrees multiplied by
//!    CORRECTIONSCLIENT_POSITION_SCALE_FACTOR.
//! \param[in] scaled_longitude
//!    The position longitude in decimal degrees multiplied by
//!    CORRECTIONSCLIENT_POSITION_SCALE_FACTOR.
//! \param[out] selected_tile
//!    Optional parameter to indicate the tile number selected to provide
//!    corrections based on the input position.
//!    Set this parameter to NULL to ignore the selected tile value.
//!    When not NULL, this parameter is set to the selected tile that
//!    encompasses the input position, or is set to 0 if no tile has
//!    a coverage area based on the input position.
//!
//! \retval true
//!    A tile with coverage encompassing the provided position was found.
//!    The selected_tile is set to the tile ID when selected_tile
//!    is not NULL.
//! \retval false
//!    A tile with coverage encompassing the provided position was not found.
//!    This can occur if a tile list has not been received from the corrections
//!    server yet.
//!    The selected_tile is set to 0 when selected_tile is not NULL.
//--------------------------------------------------------------------------
bool CorrectionsClient_UpdateTileSelection(
      CorrectionsHandle handle,
      int16_t scaled_latitude,
      int16_t scaled_longitude,
      uint16_t* selected_tile );

//--------------------------------------------------------------------------
//! \brief Connect to the correction source
//
//! Once successfully connected, the client will automatically attempt to
//! re-establish a connection if it is lost. If the connection is not successful
//! the client will not attempt to re-connect, and this function must be explicitly
//! called again.
//
//! \param[in] handle - A handle to the client to connect.
//
//! \return CONNECTION_SUCCESS - The connection was successful. Reconnect will be
//!   attempted if the connection is lost.
//! \return CONNECTION_ERR_NOTFOUND - The specified mountpoint was not found.
//! \return CONNECTION_ERR_UNAUTHORIZED - The specified credentials are invalid.
//! \return CONNECTION_ERR_ADDRESS - The address could not be resolved. The connection
//!   will not be automatically re-tried.
//! \return CONNECTION_ERR_FAILED - The connection failed
//--------------------------------------------------------------------------
CorrectionsConnectionResult CorrectionsClient_Connect(CorrectionsHandle handle);

//--------------------------------------------------------------------------
//! \brief Disconnect from the correction source
//
//! Once this function is called, the client will not attempt to re-establish
//! the connection until Connect() is called.
//
//! \param[in] handle - A handle to the client to disconnect.
//--------------------------------------------------------------------------
void CorrectionsClient_Disconnect(CorrectionsHandle handle);

//--------------------------------------------------------------------------
//! \brief Get the current status of a correction source
//
//! \param[in] handle - A handle to the client to fetch the status of.
//! \param[out] connection_state - The current state of the connection. Pass NULL
//!   if not needed.
//! \param[out] connection_stats - Various statistics of the connection. Pass NULL
//!   if not needed.
//
//! \return - The number of bytes currently waiting to be read. May be zero if
//!   no data is waiting, or negative if an error occurred attempting to get status.
//--------------------------------------------------------------------------
int32_t CorrectionsClient_GetStatus(
      CorrectionsHandle handle,
      CorrectionsConnectionStatus* connection_state,
      CorrectionStatistics* connection_stats);

//--------------------------------------------------------------------------
//! \brief Receive correction data
//
//! This function will not block. If no data is available, it will immediately
//! return zero bytes.
//
//! \param[in] handle - A handle to the client to receive from.
//! \param[in] buffer - A buffer to store the received correction data in
//! \param[in] buffer_size - The size of the buffer provided in buffer
//
//! \return The number of bytes received and placed in the buffer. This will
//!   not exceed buffer_size. This value will be negative if an error occurred.
//--------------------------------------------------------------------------
int32_t CorrectionsClient_Receive(
      CorrectionsHandle handle,
      uint8_t* buffer,
      uint32_t buffer_size);


#ifdef __cplusplus
}
#endif //__cplusplus


#endif // CORRECTIONSCLIENT_H
