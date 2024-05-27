/**@file        loc_core_api.h
 * @brief       Location engine core api header file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * <tr><td>2022/12/28  <td>0.2      <td>caizhijie   <td>Add INS Interface
 * </table>
 *
 **********************************************************************************
 */

#ifndef __LOC_CORE_API_H__
#define __LOC_CORE_API_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SSR Service Source */
typedef uint8_t gnss_ssrServiceSource;
#define GNSS_SSR_SERVICE_SOURCE_NONE          (uint8_t)0x00
#define GNSS_SSR_SERVICE_SOURCE_QX            (uint8_t)0x01
#define GNSS_SSR_SERVICE_SOURCE_Gee           (uint8_t)0x02
#define GNSS_SSR_SERVICE_SOURCE_AG_SRR_LOS    (uint8_t)0x03

/** Location Engine Memory type */
typedef uint8_t loc_api_MemoryRegisterType;
#define LOC_API_MEMORY_NONE                   (uint8_t)0x00
#define LOC_API_MEMORY_OS                     (uint8_t)0x01
#define LOC_API_MEMORY_EXT_POOL               (uint8_t)0x02
#define LOC_API_MEMORY_EXT_API                (uint8_t)0x03

typedef uint8_t gnss_DcpType;

#define LOC_API_MEM_POOL_COUNT_MAX (8)
/** Location Engine Memory Register Structure */
typedef struct {
  loc_api_MemoryRegisterType u_type;
  /** External memory alloc callback, if unavaliable, set NULL */
  void* (*alloc)(uint32_t sz, const char* func, uint32_t line, char* file);
  /** External memory free callback, if unavaliable, set NULL */
  void (*free)(void* ptr);
  uint32_t pool_count;
  uint8_t* pool_addr[LOC_API_MEM_POOL_COUNT_MAX];
  uint32_t pool_size[LOC_API_MEM_POOL_COUNT_MAX];
} loc_api_MemoryRegister_t;

typedef struct
{
  uint64_t utc_timestamp; /* ms */
  uint16_t utc_year;
  uint16_t utc_month;
  uint16_t utc_day;
  uint16_t utc_hour;
  uint16_t utc_minute;
  double utc_second;

  uint16_t week; /* GPS week number */
  double tow; /* time of week */
  uint8_t leapsec; /* leap second */

  uint8_t fix_quality; /* 0/1/2/4/5 */

  double lon; /* Longitude (deg) */
  double lat; /* Latitude (deg) */
  float alt;

  float lonstd; /* unit: m */
  float latstd; /* unit: m */
  float altstd; /* unit: m */

  float vel_n; /* NED north velocity (m/s) */
  float vel_e; /* NED east velocity (m/s) */
  float vel_d; /* NED down velocity (m/s) */

  float vel_n_std; /* unit: m/s */
  float vel_e_std; /* unit: m/s */
  float vel_d_std; /* unit: m/s */

  float age; /* diff age (s) */

  uint16_t sv_used;
  uint16_t sv_trked;
  uint16_t sv_fixed;

  float gdop;
  float pdop;
  float hdop;
  float vdop;
  float tdop;

  double quasi_geoid_h;//add new field to represent quasi-geoid height
  float avg_CN0;//add new field to represent average CN0
  uint8_t CN040;
  gnss_DcpType dcp_pos_type;
} loc_api_location_report_t;

/* IMU Inject Data */
typedef struct {
  uint64_t t_timestamp;
  uint16_t w_status;
  float    f_gyro[3];
  float    f_accl[3];
  float    f_temperature[1];
} loc_api_imu_data_t;

/* PPS Inject Data */
typedef struct {
  uint64_t t_timestamp;
  uint16_t w_valid;
} loc_api_pps_t;

/** Register Callback function definition */
typedef struct {
  /** Get millisecond tick */
  uint64_t (*get_tick_ms)();
  /** Report loc engine log */
  void (*report_log)(uint8_t* buf, uint32_t len);
  /** Report location information */
  void (*report_location)(loc_api_location_report_t* info);
} loc_api_callback_t;

/** Configurate parameter definition */
typedef struct {
  uint64_t t_mask;
  uint8_t  u_field[10];
  float    f_field[10];
} loc_api_config_para_t;

typedef struct {
  loc_api_config_para_t para[8];
} loc_api_config_para_group_t;

/**
 * @brief     Location Engine API for Register callback functions
 * @param[in] cb - the callback function package
 * @return    None
 */
int32_t loc_api_Register_Callback(loc_api_callback_t* cb);

/**
 * @brief     Location Engine API for Register Memory pool
 * @param[in] pool_mem   - Memory pools pointer array
 * @param[in] pool_size  - Memory pools size array
 * @param[in] pool_count - Memory pools count
 * @return    None
 */
int32_t loc_api_Register_MemoryPool(loc_api_MemoryRegister_t* pz_MemoryRegister);

/**
 * @brief Location Engine API for Register configurations
 * @return      None
 */
int32_t loc_api_Register_Config(loc_api_config_para_group_t* pz_api_config_para_grp);

/**
 * @brief Location Engine API for start
 * @return      None
 */
int32_t loc_api_Initialize();

/**
 * @brief Location Engine API for release
 * @return      None
 */
int32_t loc_api_Release();

/**
 * @brief     Inject GNSS Receiver measurement by RTCM format
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 */
void loc_api_InjectRcvMeasRTCM(uint8_t* data, uint32_t len);

/**
 * @brief     Inject GNSS Reference station observed Correction measurement
              using RTCM format
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 * @note      The correction measurement is from NRTK service
 */
void loc_api_InjectRefCorrRTCM(uint8_t* data, uint32_t len);

/**
 * @brief     Inject GNSS Reference station observed Correction measurement
              using measurement block
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 * @note      The correction measurement is from NRTK service as measurement block
 */
void loc_api_InjectRefCorrMeasBlk(uint8_t* data, uint32_t len);

/**
 * @brief     Inject SSR Correction stream
 * @param[in] data - stream data buffer
 * @param[in] len - stream data length
 * @param[in] source - stream data service source
 * @return    None
 */
void loc_api_InjectSsrStream(uint8_t* data, uint32_t len, gnss_ssrServiceSource source);

/**
 * @brief     Inject IMU data
 * @param[in] pz_imu - pointer to loc_api_imu_data_t
 * @return    None
 */
void loc_api_InjectImuData(loc_api_imu_data_t* pz_imu);

/**
 * @brief     Inject PPS time data
 * @param[in] pz_pps - pointer to loc_api_pps_t
 * @return    None
 */
void loc_api_InjectPPS(loc_api_pps_t* pz_pps);

/**
* @brief Location Engine API for get version
* @return      Gnss version string
*/
const char* loc_api_GetVersion();

#ifdef __cplusplus
}
#endif

#endif
