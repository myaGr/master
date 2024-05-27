/**@file        mw_log.h
 * @brief       log module header file
 * @Detail      VLT log module contain three kinds of log formats.
 *              1. String Log  : for Text output
                2. Message Log : for IPC Message binary format output
                3. Package Log : for Runtime struct binary format output
 * @author      caizhijie
 * @date        2022/02/12
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/02/12  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __MW_LOG_H__
#define __MW_LOG_H__

#include "cmn_def.h"
#include "mw_ipctask.h"
#include "loc_core_cfg.h"

BEGIN_DECL

/** The max cache fifo size before output buffer */
#define LOG_CACHE_MAX_LENGTH (1024*5)

/** The max length of a text string */
#define LOG_TEXT_MAX_LENGTH  (1024*3)

/** The max length of a bddb string */
#define LOG_BDDB_MAX_LENGTH  (256)

/** Log header sync code */
#define LOG_SYNC0 (uint8_t)0x41 // 'A'
#define LOG_SYNC1 (uint8_t)0x53 // 'S'

/* Log option */
typedef enum
{
  LOG_OPT_DONT_TIME_PREFIX = 0x01,
  LOG_OPT_DONT_TIME_COST   = 0x02
}LogOption_EnumVal;
typedef uint8_t LogOption;

/** Log Type enum */
typedef enum {
  LOG_TYPE_NONE    = (uint8_t)0x00, //< None
  LOG_TYPE_TEXT    = (uint8_t)0x01, //< Text string log
  LOG_TYPE_IPC     = (uint8_t)0x02, //< IPC record log
  LOG_TYPE_PACKAGE = (uint8_t)0x03, //< Struct information record log
  LOG_TYPE_CMPRS   = (uint8_t)0x04, //< Text string log compressed
  LOG_TYPE_CRC_TEXT    = (uint8_t)0x05, //< Text string log using CRC24
  LOG_TYPE_CRC_IPC     = (uint8_t)0x06, //< IPC record log using CRC24
  LOG_TYPE_CRC_PACKAGE = (uint8_t)0x07, //< Struct information record log using CRC24
  LOG_TYPE_CRC_CMPRS   = (uint8_t)0x08, //< Text string log compressed using CRC24
  LOG_TYPE_MAX     = (uint8_t)LOG_TYPE_CRC_CMPRS
} LogType_EnumVal;
typedef uint8_t LogType_e;

typedef enum {
  LOG_PACKAGE_ID_START      = (uint16_t)0x0000, /* Package Start id   */
  LOG_PACKAGE_ID_MONITOR    = (uint16_t)0x0001, /* Loc engine monitor */
  LOG_PACKAGE_ID_LOC_CONFIG = (uint16_t)0x0002, /* Loc engine config  */
  LOG_PACKAGE_ID_INS_RESULT = (uint16_t)0x0003, /* Ins Result         */
  LOG_PACKAGE_ID_TSX_CORR   = (uint16_t)0x0004, /* TSX adapter        */
  LOG_PACKAGE_ID_END,                           /* End of values      */
} LogPackageIdTypeVal;
typedef uint16_t LogPackageIdType;

/** Log level */
typedef enum {
  LOG_LEVEL_NONE,
  LOG_LEVEL_E,
  LOG_LEVEL_W,
  LOG_LEVEL_I,
  LOG_LEVEL_D
} LogLevelEnumVal;
typedef uint8_t LogLevelEnum;

/** Log level */
typedef enum {
  LOG_OUTPUT_IPC,
  LOG_OUTPUT_RAW,
} LogOutputTypeEnumVal;
typedef uint8_t LogOutputTypeEnum;

/** Log module */
typedef enum {
  TAG_SM = (uint8_t)0x00,
  TAG_HPP,
  TAG_DCP,
  TAG_PPP,
  TAG_SD,
  TAG_VDR,
  LOG_TAG_MAX,
} LogTag_EnumVal;
typedef uint8_t LogTag_e;

void log_api_Init(log_ConfigParam_t* pz_log_config);

void log_api_Release();

/**
 * @brief Set log level
 *        Log level list:
          LOG_LEVEL_NONE ： 0
 *        LOG_LEVEL_E : 1
 *        LOG_LEVEL_W : 2
 *        LOG_LEVEL_I : 3
 *        LOG_LEVEL_D : 4
 * @return: None
 */
void log_SetLogLevel(LogLevelEnum level);

/**
 * @brief Get log level
 *        Log level list:
          LOG_LEVEL_NONE : 0
 *        LOG_LEVEL_E : 1
 *        LOG_LEVEL_W : 2
 *        LOG_LEVEL_I : 3
 *        LOG_LEVEL_D : 4
 * @return: Log Level
 */
LogLevelEnum log_GetLogLevel();

/**
 * @brief Set log output type
 * @return: None
 */
void log_SetLogOutputType(LogOutputTypeEnum type);

/**
 * @brief GNSS IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* log_task(void* args);

/**
 * @brief Log a IPC data
 *    Log format
      |  1Byte      1Byte  | 2Byte    4Byte |    sizeof(ipc_t)    |       ipc length    | 2Byte   |
      |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      IPC_HEADER     |        ipc data     | CHECKSUM|
      |                    | -----------------------Check Sum Compute data--------------|         |
 * @param[in] ipc
 * @return: None
 */
void log_ipc(ipc_t* ipc);

/**
 * @brief Write a text string into log
 * 
 *    Text string format
 * | 1Byte 1Byte | 2Byte    4Byte |  Log Text String length | 2Byte   |
 * | SYNC0 SYNC1 | LOG_TYPE LENGTH|  Log Text String        | CHECKSUM|
 * |             | ---------Check Sum Compute data--------- |         |
 *
 * @param[in] module : log module
 * @param[in] level  : log level
 * @param[in] line   : code line
 * @return: None
 */
void log_WriteText(LogTag_e module, LogLevelEnum level, uint32_t line, const char* fmt, ...);
#define LOGE(m, ...) log_WriteText(m, LOG_LEVEL_E, __LINE__, ##__VA_ARGS__);
#define LOGW(m, ...) log_WriteText(m, LOG_LEVEL_W, __LINE__, ##__VA_ARGS__);
#define LOGI(m, ...) log_WriteText(m, LOG_LEVEL_I, __LINE__, ##__VA_ARGS__);
#define LOGD(m, ...) log_WriteText(m, LOG_LEVEL_D, __LINE__, ##__VA_ARGS__);

/**
 * @brief Write a package into log
 *
 *    Package format
 * | 1Byte 1Byte | 2Byte    4Byte |  2Byte | Package data length | 2Byte   |
 * | SYNC0 SYNC1 | LOG_TYPE LENGTH| Pkg ID | Package data        | CHECKSUM|
 * |             | ---------Check Sum Compute data-------------- |         |
 *
 * @param[in] w_PackageId : Package id
 * @param[in] p_data      : data buffer
 * @param[in] q_length    : data length
 * @return: None
 */
void log_Package(LogPackageIdType w_PackageId, uint8_t* p_data, uint32_t q_length);

/**
 * @brief 
 log
 * @param[in] p_in :
 * @param[in] q_length_in :
 * @param[out] p_out :
 * @return: q_length_out : output length, if >0, compress success
 */
uint32_t log_compress(uint8_t* p_in, uint32_t q_length_in, uint8_t* p_out);

/**
 * @brief Write data into fifo
 * @param[in] data :
 * @param[in] len :
 * @return: None
 */
BOOL log_write_fifo(uint8_t* data, uint32_t len);

/**
 * @brief fflush log fifo
 * @return: None
 */
void log_fflush();

/** Data Log module */
typedef enum {
  DLOG_SM = (uint8_t)0x00,
  DLOG_HPP,
  DLOG_DCP,
  DLOG_PPP,
  DLOG_SD,
  DLOG_VDR,
  DLOG_SAT_POS,
  DLOG_GNSS_PL,       /** GNSS protection level */
  DLOG_GNSS_ML_PL,    /** GNSS machine learning data to determine PL */
  DLOG_GNSS_ML_SCENE, /** GNSS machine learing data to determine SCENE */
  DLOG_GNSS_SCENE,    /** result of scene type prediction */
  DLOG_GNSS_STD,      /** result of scene type prediction */
  DLOG_VDR_BDDB,
  DLOG_MAX,
} DataLogTagEnumVal;
typedef uint8_t DataLogTagEnum;

/** Data Log Manager structure */
typedef struct {
  uint8_t u_Enable;
  uint8_t u_Fileheader[DLOG_MAX];   /* status of file header(columns names) */
  FILE* p_DataLogHander[DLOG_MAX];
} DataLogManager_t;

typedef struct {
  uint8_t u_Enable;
  char    pb_output_dir[256];
  char    p_DataLogFilepath[DLOG_MAX][256];
} DataLogManagerConfig_t;

/**
 * @brief Release datalog file pinter
 * 
 */
void log_DataLogManager_Release();

/**
 * @brief Initilize Data log manager
 *        Only use in playback mode
 * @param[in] pz_DataLogManagerConfig : Data log module manager config structure
 * @return: None
 */
void log_DataLogManager_Initialize(DataLogManagerConfig_t* pz_DataLogManagerConfig);

/**
 * @brief Check Data log manager is enable
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
BOOL log_DataLogManager_IsEnable();
#define DATA_LOG_ENABLE() log_DataLogManager_IsEnable()

/**
 * @brief Print Data log string
 * @param[in] datalog_tag
 * @param[in] fmt
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
void log_DataLogManager_Print(DataLogTagEnum datalog_tag, const char* fmt, ...);
#define DATA_LOG(m, ...) log_DataLogManager_Print(m, ##__VA_ARGS__)

BOOL log_DataLogHeader_IsEnable(DataLogTagEnum datalog_tag);
#define DATA_LOG_HEADER_ENABLE(m) log_DataLogHeader_IsEnable(m);

/**
 * @brief Print Data log string in file header
 * @param[in] datalog_tag
 * @param[in] fmt
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
void log_DataLogManager_Print_Header(DataLogTagEnum datalog_tag, const char* fmt, ...);
#define DATA_LOG_HEADER(m, ...) log_DataLogManager_Print_Header(m, ##__VA_ARGS__)

/**
 * @brief Print BDDB log string
 * @param[in] datalog_tag
 * @param[in] fmt
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
void log_BDDBLogManager_Print(DataLogTagEnum datalog_tag, const unsigned char* str, uint32_t len);
#define BDDB_LOG(m, str, len) log_BDDBLogManager_Print(m, str, len)

/**
 * @brief File pointer close safely
 * @param[in] fp file pointer
 */
void log_fclose_safe(FILE* fp);

int32_t log_DataLogHeaderSprintf(DataLogTagEnum datalog_tag, char *buffer, const char *format, ...);

END_DECL

#endif
