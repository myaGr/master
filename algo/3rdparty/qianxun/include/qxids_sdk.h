#ifndef QXIDS_SDK_H
#define QXIDS_SDK_H

#include "qxids_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************
 * @brief Definition for SDK status.
 ********************************************************/
#ifndef QXIDS_STATUS_LIST
#define QXIDS_STATUS_LIST

#define QXIDS_STATUS_OUT_OF_SERVICE_AREA   1001
#define QXIDS_STATUS_INVALID_GGA           1002
#define QXIDS_STATUS_CAP_START_SUCCESS     2000
#define QXIDS_STATUS_AUTH_SUCCESS          2001
#define QXIDS_STATUS_NETWORK_RECOVER       2002
#define QXIDS_STATUS_CFG_COORD_SYS_SUCCESS 2003
#define QXIDS_STATUS_NETWORK_ERROR         3001
#define QXIDS_STATUS_ACCT_ERROR            3002
#define QXIDS_STATUS_ACCT_EXPIRED          3003
#define QXIDS_STATUS_ACCT_NEED_BIND        3004
#define QXIDS_STATUS_ACCT_NEED_ACTIVE      3005
#define QXIDS_STATUS_NO_AVAILABLE_ACCT     3006
#define QXIDS_STATUS_SERVICE_STOP          3007
#define QXIDS_STATUS_NULL_ACCT             3100
#define QXIDS_STATUS_INVALID_PARA          3101
#define QXIDS_STATUS_SERVER_ABORT          5000
#define QXIDS_STATUS_CAP_START_FAILED      5001
#define QXIDS_STATUS_UNKNOWN_ERROR         9999

#endif  // QXIDS_STATUS_LIST

/*********************************************************
 * @brief log level
 ********************************************************/
#ifndef QXIDS_LOG_LEVEL_E
#define QXIDS_LOG_LEVEL_E
typedef enum
{
    QXIDS_LOG_E = 0,
    QXIDS_LOG_W = 1,
    QXIDS_LOG_I = 2,
    QXIDS_LOG_D = 3,
    QXIDS_LOG_V = 4,
} qxids_log_level_e;
#endif  // QXIDS_LOG_LEVEL_E

/*********************************************************
 * @brief log mode
 ********************************************************/
#ifndef QXIDS_LOG_MODE_E
#define QXIDS_LOG_MODE_E
typedef enum
{
    QXIDS_LOG_DISABLED = 0,
    QXIDS_LOG_CONSOLE  = 1 << 0,
    QXIDS_LOG_FILE     = 1 << 1,
} qxids_log_mode_e;
#endif  // QXIDS_LOG_MODE_E

/*********************************************************
 * @brief  SDK key type
 ********************************************************/
#ifndef QXIDS_KEY_TYPE
#define QXIDS_KEY_TYPE
typedef enum
{
    QXIDS_KEY_TYPE_AK  = 0,
    QXIDS_KEY_TYPE_DSK = 1,
} qxids_key_type_e;
#endif  // QXIDS_KEY_TYPE

#ifndef NAME_MAX
#define NAME_MAX 255 /* # chars in a file name */
#endif

/*********************************************************
 * @brief SDK configuration.
 ********************************************************/
#ifndef QXIDS_CONFIG_T
#define QXIDS_CONFIG_T

#define QXIDS_SDK_MAX_KEY_LEN      128
#define QXIDS_SDK_MAX_SECRET_LEN   128
#define QXIDS_SDK_MAX_DEV_ID_LEN   128
#define QXIDS_SDK_MAX_DEV_TYPE_LEN 128

typedef struct
{
    /**
     * @brief Set to sizeof(qxids_config_t)
     */
    qxids_size_t size;
    /**
     * SDK key type
     **/
    qxids_key_type_e key_type;
    /**
     * Application unique identity.
     */
    qxids_char_t app_key[QXIDS_SDK_MAX_KEY_LEN];
    /**
     * Secret for corresponding application key.
     */
    qxids_char_t app_secret[QXIDS_SDK_MAX_SECRET_LEN];
    /**
     * Device id, UUID.
     */
    qxids_char_t device_id[QXIDS_SDK_MAX_DEV_ID_LEN];
    /**
     * Device type.
     */
    qxids_char_t device_type[QXIDS_SDK_MAX_DEV_TYPE_LEN];
    /**
     * SDK working directoy.
     */
    qxids_char_t log_dir[NAME_MAX];
    /**
     * @brief LOG mode
     */
    qxids_uint32_t log_mode;
    /**
     * LOG enable flag.
     */
    qxids_log_level_e log_level;
    /**
      * LOG space size
      */
     qxids_uint32_t log_size;
} qxids_config_t;

#endif  // QXIDS_CONFIG_T

/****************************************************************
 * SDK callbacks.
 ****************************************************************/
#ifndef QXIDS_CALLBACKS_T
#define QXIDS_CALLBACKS_T

typedef struct
{
    /**
     * @brief Set to sizeof(qxids_callbacks_t)
     */
    qxids_size_t size;
    /**
     * @brief Callback to report NMEA data
     */
    qxids_void_t (*fill_nmea_info)(qxids_uint64_t time, qxids_char_t const* nmea, qxids_size_t len);
    /**
     * @brief Callback to report RTCM data
     */
    qxids_void_t (*fill_rtcm_data)(qxids_char_t const* buf, qxids_size_t len);
    /**
     * @brief Callback to report NSSR data
     */
    qxids_void_t (*fill_nssr_data)(qxids_char_t const* buf, qxids_size_t len);
    /*
     * Callback to report sdk status code
     */
    qxids_void_t (*status_response)(qxids_int32_t status);
    /**
     * @brief Callback to report AGNSS data
     */
    qxids_void_t (*fill_agnss_data)(qxids_char_t const* buf, qxids_size_t len);
} qxids_callbacks_t;

#endif  // QXIDS_CALLBACKS_T

/****************************************************************
 * @brief Satelite System
 ****************************************************************/
#ifndef QXIDS_SAT_SYS_E
#define QXIDS_SAT_SYS_E
typedef enum
{
    QXWZ_IDS_SAT_SYS_GPS = 0x01, /** navigation system: GPS */
    QXWZ_IDS_SAT_SYS_SBS = 0x02, /** navigation system: SBAS */
    QXWZ_IDS_SAT_SYS_GLO = 0x04, /** navigation system: GLONASS */
    QXWZ_IDS_SAT_SYS_GAL = 0x08, /** navigation system: Galileo */
    QXWZ_IDS_SAT_SYS_QZS = 0x10, /** navigation system: QZSS */
    QXWZ_IDS_SAT_SYS_BDS = 0x20, /** navigation system: BeiDou */
    QXWZ_IDS_SAT_SYS_ALL = 0xFF, /** navigation system: all */
} qxids_sat_sys_e;
#endif  // QXIDS_SAT_SYS_E

/****************************************************************
 * @brief QXWZ position struct
 *     Used for uploading position information instead of GGA
 ****************************************************************/
#ifndef QXIDS_POS_STRUCT_T
#define QXIDS_POS_STRUCT_T

typedef struct /* lat | lon | alt | sec | utc time | age | solq | ns */
{
    qxids_float64_t lat;      /* latitude (degree) */
    qxids_float64_t lon;      /* longitude (degree) */
    qxids_float64_t alt;      /* altitude (meter) */
    qxids_float64_t sec;      /* time of week in gps time (s) */
    qxids_gtime_t   utc_time; /* UTC time in gtime struct */
    qxids_float32_t age;      /* age of differential (s) */
    qxids_uint8_t   solq;     /* fix quality 0-8 */
    qxids_uint8_t   ns;       /* number of satellites being tracked */
} qxids_pos_struct_t;

#endif  // QXIDS_POS_STRUCT_T

#ifndef QXIDS_CAPS_ID_E
#define QXIDS_CAPS_ID_E

typedef enum
{
    QXIDS_CAPS_ID_NOSR = 0x0001,
    QXIDS_CAPS_ID_NSSR = 0x0002,
    QXIDS_CAPS_ID_LSSR = 0x0004,
    QXIDS_CAPS_ID_EPH  = 0x0020,
} qxids_caps_id_e;

#endif  // QXIDS_CAPS_ID_E

#ifndef QXIDS_CAPS_STATE_E
#define QXIDS_CAPS_STATE_E

typedef enum
{
    QXIDS_CAPS_STATE_INSERVICE = 0,
    QXIDS_CAPS_STATE_INACTIVE  = 1,
    QXIDS_CAPS_STATE_SUSPENDED = 2,
    QXIDS_CAPS_STATE_EXPIRED   = 3,
    QXIDS_CAPS_STATE_DISABLE   = 9,
} qxids_caps_state_e;

#endif  // QXIDS_CAPS_STATE_E

#ifndef QXIDS_CAPS_INFO_T
#define QXIDS_CAPS_INFO_T

#define QXIDS_CAPS_NUM_MAX 4

typedef struct
{
    qxids_uint32_t caps_num;
    struct
    {
        qxids_uint32_t id;
        qxids_uint32_t state;
        qxids_uint64_t expire_time;
    } caps[QXIDS_CAPS_NUM_MAX];
} qxids_caps_info_t;

#endif  // QXIDS_CAPS_INFO_T

#ifndef QXIDS_SDK_CONF_E
#define QXIDS_SDK_CONF_E

typedef enum
{
    QXIDS_SDK_CONF_DNS = 0,
} qxids_sdk_conf_e;

#endif // QXIDS_SDK_CONF_E

/****************************************************************
 * SDK interfaces.
 ****************************************************************/
#ifndef QXIDS_INTERFACE_T
#define QXIDS_INTERFACE_T

typedef struct
{
    /**
     * Set to sizeof(QXWZSdkInterface).
     */
    qxids_size_t size;
    qxids_int32_t (*config)(qxids_sdk_conf_e type, qxids_void_t*conf);
    qxids_int32_t (*init)(qxids_callbacks_t* callbacks, qxids_config_t* config);
    qxids_int32_t (*start)();
    qxids_int32_t (*stop)();
    qxids_int32_t (*cleanup)();
    const qxids_char_t* (*get_ver_info)();
    qxids_int32_t (*get_caps_info)(qxids_caps_info_t* caps_info);
    qxids_int32_t (*inject_pos_struct)(qxids_pos_struct_t* pos_struct);
    qxids_int32_t (*inject_gnss_data)(qxids_char_t* buffer, qxids_size_t len);
    /**
     * sat must make using QXWZIdsSatSys types with logic operator "or"
     * must called after Authentication succeeds, i.e. after status 2001 received
     */
    qxids_int32_t (*assist_eph_request)(qxids_uint32_t sat);
} qxids_interface_t;

/**
 * Get the SDK interfaces.
 * @param qxids_void_t
 * @return a pointer of QXWZSdkInterface type that contains the SDK's interfaces
 */
qxids_interface_t const* get_qxids_interface(qxids_size_t size);

#endif  // QXIDS_INTERFACE_T

#ifdef __cplusplus
}
#endif

#endif /* QXIDS_SDK_H */
