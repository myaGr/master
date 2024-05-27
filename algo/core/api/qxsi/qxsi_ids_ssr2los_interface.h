/*******************************************************************************
 * Copyright (c) 2021 Qianxun SI Inc. All rights reserved.
 *
 * @file    qxsi_ids_ssr2los_interface.h
 *
 *                            FILE DESCRIPTION
 *-----------------------------------------------------------------------------*
 *  This file is the header file of the Adapter Manager module.
 *-----------------------------------------------------------------------------*

 *-----------------------------------------------------------------------------*
 *                            CHANGE HISTORY                                   *
 *-----------------------------------------------------------------------------*
 *   <Date>   | <Version> | <Author> | <Description>
 *------------------------------------------------------------------------------
 * 2021/7/29  | 0.1.1     | xu7        | Newly create.
 * 2021/12/15 | 1.0.1     | binbin     | Modify.
 * 2022/03/17 | 2.0.2     | hailong    | Modify for functional safety.
 ******************************************************************************/

#ifndef QXSI_IDS_SSR2LOS_INTERFACE_H
#define QXSI_IDS_SSR2LOS_INTERFACE_H

#include <stddef.h>
#include "qxsi_ssr2los_type_def.h"

#ifdef __cplusplus
extern "C" {
#endif


/* SSR2LOS IDS status list */
typedef enum {
    /* IDS STATE CODE */
    QXWZ_IDS_STATUS_NOT_INITIALIZED                      = 0,       /* terminal adapter is unnitialized */
    QXWZ_IDS_STATUS_INIT_SUCCESS                         = 1,        /* terminal adapter is initialized */
    QXWZ_IDS_STATUS_CLEANUP_SUCCESS                      = 2,        /*terminal adapter clean up is success */

    /* SSR2LOS STATE CODE */
    QXWZ_SSR2LOS_STATUS_UNKNOWN                          = 100,

    /* SSR Decoder STATE CODE*/
    QXWZ_SSRDECODER_STATUS_INVALID_PARAMETER             = 401,        /* invalid parameters, maybe NULL pointer, invalid length, etc. */
    QXWZ_SSRDECODER_STATUS_INCOMPLETE_DATA               = 402,        /* not a whole frame, need input more data */
    QXWZ_SSRDECODER_STATUS_INVALID_FRAME_LEN             = 403,        /* invalid transport layer frame length */
    QXWZ_SSRDECODER_STATUS_INVALID_FRAME_OFFSET          = 404,        /* invalid frame offset */
    QXWZ_SSRDECODER_STATUS_INVALID_DATA_SOURCE           = 405,        /* Invalid data source, neither NSSR nor LSSR */
    QXWZ_SSRDECODER_STATUS_FRAME_DECODE_FAIL             = 406,        /* frame decode fail */
    QXWZ_SSRDECODER_STATUS_INVALID_DATA_CALLBACK         = 407,        /* invalid data callback */
    QXWZ_SSRDECODER_STATUS_INVALID_STAT_CALLBACK         = 408,        /* invalid status callback */
    QXWZ_SSRDECODER_STATUS_NOT_INITIALIZED               = 413,        /* decode without configuration */
    QXWZ_SSRDECODER_STATUS_UNSUPPORT_NSSR_TYPE           = 451,        /* unsupported NSSR type */
    QXWZ_SSRDECODER_STATUS_CRC_CALCULATE_FAIL            = 453,        /* crc function executed fail */
    QXWZ_SSRDECODER_STATUS_CRC_COMPARE_FAIL              = 454,        /* crc compare failed */
    QXWZ_SSRDECODER_STATUS_INVALID_SUBFRAME_LEN          = 456,        /* invalid sub-frame length */
    QXWZ_SSRDECODER_STATUS_XOR_CHECK_FAIL                = 457,        /* xor check fail */
    QXWZ_SSRDECODER_STATUS_CRC_MEMORY_CHECK_FAIL         = 490,
} qxsi_ids_ssr2los_status_e;

/* Configuration of SSR2LOS IDS  engine */
//#ifndef QXIDS_CONFIG_T
//#define QXIDS_CONFIG_T
//typedef struct {
//    unsigned int enable;              /* 1 to enable record, 0 to disable record */
//    qxsi_log_level_e level;
//} qxsi_ids_ssr2los_config_t;
//#endif

typedef struct {
  unsigned int enable;              /* 1 to enable record, 0 to disable record */
  qxsi_log_level_e level;
} qxsi_ids_ssr2los_config_t;

/*
 * Represent SSR2LOS callbacks.
 */
typedef struct {

    /*
     * Logging function.
     *
     * @param qxsi_log_level_e logging level @see #qxsi_log_level_e.
     * @param buf the pointer of the buffer that stores the logging data.
     * @param len the length of the buffer pointed to by the buf parameter.
     */
    void (*log_callback)(const char * const buf, const int len);
    /*
     * Callback to report status code.
     *
     * @param qxsi_ids_ssr2los_status_e @see #qxsi_ids_ssr2los_status_e.
     */
    void (*status_callback)(const qxsi_ids_ssr2los_status_e code);
    
} qxsi_ids_ssr2los_callbacks_t;

/*
 * Represent the SSR2LOS IDS interface.
 */
typedef struct {
    /*
     * Set to sizeof(qxsi_SSR2LOS_interface).
     */
    size_t size;

    /*
     * Initialize the SSR2LOS engine, to provide configuration and callbacks.
     *
     * @param config pointer to the config of engine, @see #qxsi_ssr2los_config_t.
     * @param callbacks pointer to the callbacks @see #qxsi_SSR2LOS_callbacks_t.
     *
     * @return 0 on success, -1 on failure.
     */
    int (*init)(const qxsi_ids_ssr2los_config_t *const config, const qxsi_ids_ssr2los_callbacks_t *const callbacks);

    /*
     * Push GPS broadcast ephemeris to SSR2LOS engine.
     *
     * @param eph pointer to GPS broadcast ephemeris.
     *
     * @return 0 on success, -1 on failure
     */
    int (*push_eph_gps)(const qxsi_gps_nav_t* const eph);

    /*
     * Push BeiDou broadcast ephemeris to SSR2LOS engine.
     *
     * @param eph pointer to Beidou broadcast ephemeris.
     *
     * @return 0 on success, -1 on failure
     */
    int (*push_eph_bds)(const qxsi_bds_nav_t* const eph);

    /*
     * Push Galileo broadcast ephemeris to SSR2LOS engine.
     *
     * @param eph pointer to GAL broadcast ephemeris.
     *
     * @return 0 on success, -1 on failure
     */

    int (*push_eph_gal)(const qxsi_gal_nav_t* const eph);

    /*
     * Push SSR data to SSR2LOS engine.
     *
     * @param type type of SSR data, @see #qxwz_ssr_data_type_e.
     * @param ssr_data pointer to decoded ssr data, @see #qxwz_ssr_data_t.
     *
     * @return 0 on success, -1 on failure
     */
    int (*push_ssr)(const unsigned char* const ssr_data, const unsigned int len);

    /*
     * Pull LOS data from SSR2LOS engine.
     *
     * @param station contains station's observation and position info, @see #qxsi_station_info_t.
     * @param los calculated LOS result, @see #qxsi_station_los_t.
     *
     * @return 0 on success, -1 on failure.
     */
    int (*pull_los_data_by_obs)(const qxsi_station_info_t *const station, qxsi_station_los_t *const los);

    /*
     * Coordinate transformation from SSR service's framework to targeted framework.
     *
     * @param pos input position with timestamp, velocity is not necessary.
     * @param trans_pos output position result for targeted coordinate framework.
     *
     * @return 0 on success.
     *         -1 indicates invalid parameters.
     *         -2 indicates itrf info not ready.
     *         -3 indicates time validity check failed.
     *         -4 indicates transformation failure.
     *
     */
    int (*coordinate_transform)(const qxsi_sta_pos_t *const pos, qxsi_coor_xyz_t *const trans_pos);
    /*
     * Cleanup SSR2LOS engine.
     *
     * @param void.
     *
     * @return 0 on success, -1 on failure.
     */
    int (*cleanup)(void);

    /*
     * Get version of SSR2LOS engine.
     *
     * @param void.
     *
     * @return the string of version on success, NULL on failure.
     */
    const char* (*get_version)(void);

} qxsi_ids_ssr2los_interface_t;

/*
 * Get SSR2LOS IDS interface.
 *
 * @param void.
 * @return a pointer of qxsi_ids_get_SSR2LOS_interface type that contains algorithm's interfaces.
 */
const qxsi_ids_ssr2los_interface_t* qxsi_ids_get_ssr2los_interface(void);


#ifdef __cplusplus
}
#endif

#endif /*QXSI_IDS_SSR2LOS_INTERFACE_H */