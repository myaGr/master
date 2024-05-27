/**@file        mw_log.cpp
 * @brief       log Implement
 * @Detail      VLT log module contain three kinds of log formats.
 *              1. Text Log    : for Text output
                2. Message Log : for IPC Message binary format output
                3. Package Log : for Runtime struct binary format output
 * @author      caizhijie
 * @date        2022/02/01
 * @version     V0.1
 * @copyright   Copyright (c) 2022-2022 Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/02/01  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include "mw_alloc.h"
#include "cmn_utils.h"
#include "mw_log.h"
#include "mw_ipctask.h"
#include "loc_core_report.h"
#include "mw_logcmprs.h"

#define C_COMPRESS_CAPACITY  (25*1024)

extern uint64_t timepro_get_now();

/** Log Manager structure */
typedef struct {
  uint8_t           u_isInit;
  LogLevelEnum      u_log_level;
  LogOutputTypeEnum u_log_output_type;
  uint32_t          q_cache_size;
  /* Log compress module */
  uint8_t           u_cmprs_enable;
  uint32_t          q_cmprs_capacity;
  uint32_t          q_cmprs_output_count;
  uint8_t*          u_cmprs_output_buffer;
  uint32_t          q_cmprs_internal_count;
  uint8_t*          u_cmprs_internal_buffer;
} LogManager_t;

static struct {
  LogTag_e type;
  const char* tag;
} gz_aLogTag[LOG_TAG_MAX] = {
  {TAG_SM,  "SM "},
  {TAG_HPP, "HPP"},
  {TAG_DCP, "DCP"},
  {TAG_PPP, "PPP"},
  {TAG_SD,  "SD "},
  {TAG_VDR, "VDR"},
};

static LogManager_t gz_logManager = {
  FALSE,
  LOG_LEVEL_I,
  LOG_OUTPUT_IPC,
  LOG_CACHE_MAX_LENGTH
};

void log_api_Init(log_ConfigParam_t* pz_log_config)
{
  gz_logManager.u_isInit = TRUE;

  /* Compress */
  gz_logManager.u_cmprs_enable = pz_log_config->u_logCompressEnable;
  if (TRUE == gz_logManager.u_cmprs_enable)
  {
    gz_logManager.q_cmprs_capacity = C_COMPRESS_CAPACITY;

    gz_logManager.q_cmprs_output_count = 0;
    gz_logManager.u_cmprs_output_buffer = (uint8_t*)OS_MALLOC(gz_logManager.q_cmprs_capacity);
    if (NULL == gz_logManager.u_cmprs_output_buffer)
    {
      gz_logManager.u_cmprs_enable = FALSE;
    }

    gz_logManager.q_cmprs_internal_count = 0;
    gz_logManager.u_cmprs_internal_buffer = (uint8_t*)OS_MALLOC(gz_logManager.q_cmprs_capacity);
    if (NULL == gz_logManager.u_cmprs_internal_buffer)
    {
      gz_logManager.u_cmprs_enable = FALSE;
    }
  }

  return;
}

void log_api_Release()
{
  /* Compress */
  if (TRUE == gz_logManager.u_cmprs_enable)
  {
    mw_CompressRelease();
    OS_FREE(gz_logManager.u_cmprs_output_buffer);
    OS_FREE(gz_logManager.u_cmprs_internal_buffer);
  }
}

/**
 * @brief Set log level
 *        Log level list:
          LOG_LEVEL_NONE : 0
 *        LOG_LEVEL_E : 1
 *        LOG_LEVEL_W : 2
 *        LOG_LEVEL_I : 3
 *        LOG_LEVEL_D : 4
 * @return: None
 */
void log_SetLogLevel(LogLevelEnum level)
{
  gz_logManager.u_log_level = level;
  return;
}

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
LogLevelEnum log_GetLogLevel()
{
  return gz_logManager.u_log_level;
}

/**
 * @brief Set log output type
 * @return: None
 */
void log_SetLogOutputType(LogOutputTypeEnum type)
{
  gz_logManager.u_log_output_type = type;
  return;
}

/**
 * @brief GNSS IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* log_task(void* args)
{
  uint8_t* log_buffer = NULL;
  uint32_t log_fifo_used = 0;
  TaskIndex_e taskIndex = TASK_INDEX_LOG;
  ipctask_t* t = ipctask_GetInstance(taskIndex);
  t->e_status = TASK_STATUS_ENABLE;
  if(NULL != t->float_hard_enable)
  {
    t->float_hard_enable();
  }

  while (t->e_status == TASK_STATUS_ENABLE)
  {
    mutex_lock(&t->z_mtx);
    log_fifo_used = (uint32_t)fifo_used(t->pz_fifo);
    if (0 == log_fifo_used)
    {
      signal_wait(&t->z_sgnl, &t->z_mtx, 0);
      mutex_unlock(&t->z_mtx);
      continue;
    }
    log_buffer = (uint8_t*)OS_MALLOC(log_fifo_used);

    fifo_pull(t->pz_fifo, log_buffer, log_fifo_used);
    mutex_unlock(&t->z_mtx);

    if (TRUE == gz_logManager.u_cmprs_enable)
    {
      uint32_t q_compressed_length = log_compress(log_buffer, log_fifo_used, gz_logManager.u_cmprs_output_buffer);
      if (q_compressed_length > 0)
      {
        loc_core_report_log(gz_logManager.u_cmprs_output_buffer, q_compressed_length);
      }
    }
    else
    {
      loc_core_report_log(log_buffer, log_fifo_used);
    }

    OS_FREE(log_buffer);
  }

  return NULL;
}

/**
 * @brief Log a IPC data
 *    Log format
      |  1Byte      1Byte  | 2Byte    4Byte |    sizeof(ipc_t)    |       ipc length    | 2Byte   |
      |LOG_SYNC0 LOG_SYNC1 | LOG_TYPE LENGTH|      IPC_HEADER     |        ipc data     | CHECKSUM|
      |                    | -----------------------Check Sum Compute data--------------|         |
 * @param[in] ipc
 * @return: None
 */
void log_ipc(ipc_t* ipc)
{
  if (FALSE == ipctask_ipc_validity_check(ipc->u_dst_id, ipc->u_src_id, ipc->q_ipc_id, IPC_VALIDITY_TYPE_LOG))
  {
    return;
  }

  uint8_t* ipc_body = ipc->p_data;
  uint32_t ipc_body_length = ipc->q_length;
  uint32_t payload_length = sizeof(ipc_t) + ipc_body_length;
  uint8_t ck[2] = { 0, 0 };

  /* buffer length:  8 -> header
                     payload_length
                     4 -> Crc24 */
  uint8_t* buffer = (uint8_t*)OS_MALLOC(8 + payload_length + 4);

  if (NULL == buffer)
  {
    return;
  }
  buffer[0] = LOG_SYNC0;
  buffer[1] = LOG_SYNC1;
  buffer[2] = LOG_TYPE_CRC_IPC;
  buffer[3] = 0;

  memcpy(buffer + 4, &payload_length, sizeof(uint32_t));

  memcpy(buffer + 8, (uint8_t*)ipc, sizeof(ipc_t));
  memcpy(buffer + 8 + sizeof(ipc_t), ipc_body, ipc_body_length);

  uint32_t* p_Crc24q = (uint32_t*)(&buffer[8 + payload_length]);
  *p_Crc24q = loc_crc24q((uint8_t*)buffer + 2, 6 + payload_length);

  log_write_fifo(buffer, payload_length + 12);

  IPC_FREE(buffer);
  buffer = NULL;

  return;
}

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
void log_WriteText(LogTag_e module, LogLevelEnum level, uint32_t line, const char *fmt, ...)
{
  if ((gz_logManager.u_isInit == FALSE) || (level > gz_logManager.u_log_level))
  {
    return;
  }

  uint32_t n = 8;
  char buffer[LOG_TEXT_MAX_LENGTH] = { 0 };

  char level_c = 'E';
  switch (level)
  {
  case LOG_LEVEL_E:
    level_c = 'E';
    break;
  case LOG_LEVEL_W:
    level_c = 'W';
    break;
  case LOG_LEVEL_I:
    level_c = 'I';
    break;
  case LOG_LEVEL_D:
    level_c = 'D';
    break;
  default:
    break;
  }
  /* prefix time log */
  loc_ConfigParamGroup_t* pz_conf = loc_cfg_getConfigParamGroup();
  if(!M_IS_SET_MASK(pz_conf->log_cfg.u_logTimeOption, LOG_OPT_DONT_TIME_PREFIX))
  {
    struct timespec ts = { 0 };
    uint64_t t = timepro_get_now();
    ts.tv_sec = t / 1000;
    n += (uint32_t)strftime(buffer + n, LOG_TEXT_MAX_LENGTH - 1 - n, "%F %T", gmtime(&ts.tv_sec));
    n += snprintf(buffer + n, LOG_TEXT_MAX_LENGTH - 1 - n, ".%03u", (uint32_t)(t % 1000));
  }

  n += snprintf(buffer + n, LOG_TEXT_MAX_LENGTH - 1 - n, "[%-3s:%c:%4u]", gz_aLogTag[module].tag, level_c, line);

  va_list ap;
  va_start(ap, fmt);
  if (n < LOG_TEXT_MAX_LENGTH)
  {
    n += vsnprintf(buffer + n, LOG_TEXT_MAX_LENGTH - 1 - n, fmt, ap);
  }
  va_end(ap);

  buffer[0] = LOG_SYNC0;
  buffer[1] = LOG_SYNC1;

  buffer[2] = LOG_TYPE_CRC_TEXT;
  buffer[3] = 0x00;

  uint32_t payload = n - 8;
  memcpy(buffer + 4, &payload, sizeof(uint32_t));

  if (LOG_OUTPUT_IPC == gz_logManager.u_log_output_type)
  {
    uint32_t* p_Crc24q = (uint32_t*)(&buffer[n]);
    *p_Crc24q = loc_crc24q((uint8_t*)buffer + 2, n - 2);
    log_write_fifo((uint8_t*)buffer, n + 4);
  }
  else if (LOG_OUTPUT_RAW == gz_logManager.u_log_output_type)
  {
    log_write_fifo((uint8_t*)buffer + 8, n - 8);
  }

  return;
}

/**
 * @brief compress log
 * @param[in] p_in :
 * @param[in] q_length_in :
 * @param[out] p_out :
 * @return: q_length_out : output length, if >0, compress success
 */
uint32_t log_compress(uint8_t* p_in, uint32_t q_length_in, uint8_t* p_out)
{
  uint32_t q_length_out = 0;
  uint32_t q_buffer_space = gz_logManager.q_cmprs_capacity - gz_logManager.q_cmprs_internal_count;

  if (NULL == gz_logManager.u_cmprs_internal_buffer)
  {
    return 0;
  }

  if (q_length_in < q_buffer_space)
  {
    memcpy(gz_logManager.u_cmprs_internal_buffer + gz_logManager.q_cmprs_internal_count, p_in, q_length_in);
    gz_logManager.q_cmprs_internal_count += q_length_in;
    return q_length_out;
  }

  p_out[0] = LOG_SYNC0;
  p_out[1] = LOG_SYNC1;
  p_out[2] = LOG_TYPE_CRC_CMPRS;
  p_out[3] = 0x00;

  q_length_out = mw_Compress(gz_logManager.u_cmprs_internal_buffer, gz_logManager.q_cmprs_internal_count, p_out + 8);
  if (q_length_out > 0)
  {
    memcpy(p_out + 4, &q_length_out, sizeof(uint32_t));
    uint32_t* p_Crc24q = (uint32_t*)(&p_out[q_length_out + 8]);
    *p_Crc24q = loc_crc24q((uint8_t*)p_out + 2, q_length_out + 6);
    q_length_out += 12;
  }

  gz_logManager.q_cmprs_internal_count = 0;
  memcpy(gz_logManager.u_cmprs_internal_buffer + gz_logManager.q_cmprs_internal_count, p_in, q_length_in);
  gz_logManager.q_cmprs_internal_count += q_length_in;

  return q_length_out;
}

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
static uint8_t g_PackageWriteBuffer[LOG_TEXT_MAX_LENGTH] = { 0 };
void log_Package(LogPackageIdType w_PackageId, uint8_t* p_data, uint32_t q_length)
{
  if (gz_logManager.u_isInit == FALSE)
  {
    return;
  }

  uint32_t n = 8; // 8 bytes package header
  n += 2;         // 2 bytes package id
  n += q_length;  // q_length package data

  if (n >= (LOG_TEXT_MAX_LENGTH - 2))
  {
    return;
  }

  memset(g_PackageWriteBuffer, 0, LOG_TEXT_MAX_LENGTH);
  g_PackageWriteBuffer[0] = LOG_SYNC0;
  g_PackageWriteBuffer[1] = LOG_SYNC1;

  g_PackageWriteBuffer[2] = LOG_TYPE_CRC_PACKAGE;
  g_PackageWriteBuffer[3] = 0x00;

  uint32_t payload = q_length + 2;
  memcpy(g_PackageWriteBuffer + 4, &payload, sizeof(uint32_t));
  memcpy(g_PackageWriteBuffer + 8, &w_PackageId, sizeof(uint16_t));
  memcpy(g_PackageWriteBuffer + 10, p_data, q_length);

  uint32_t* p_Crc24q = (uint32_t*)(&g_PackageWriteBuffer[n]);
  *p_Crc24q = loc_crc24q((uint8_t*)g_PackageWriteBuffer + 2, n - 2);
  log_write_fifo(g_PackageWriteBuffer, n + 4);

  return;
}


/**
 * @brief Write data into fifo
 * @param[in] data :
 * @param[in] len :
 * @return: None
 */
BOOL log_write_fifo(uint8_t* data, uint32_t len)
{
  if (FALSE == gz_logManager.u_isInit)
  {
    return FALSE;
  }

  size_t sz = 0;
  ipctask_t* t = ipctask_GetInstance(TASK_INDEX_LOG);

  if (t->e_status == TASK_STATUS_ENABLE)
  {
    mutex_lock(&t->z_mtx);
    sz = fifo_push(t->pz_fifo, data, len);
    if (sz != len)
    {
      mutex_unlock(&t->z_mtx);
      return FALSE;
    }
    sz = fifo_used(t->pz_fifo);
    if (sz > gz_logManager.q_cache_size)
    {
      log_fflush();
    }
	mutex_unlock(&t->z_mtx);
  } else {
    if (TRUE == gz_logManager.u_cmprs_enable)
    {
      uint32_t q_compressed_length = log_compress(data, len, gz_logManager.u_cmprs_output_buffer);
      if (q_compressed_length > 0)
      {
        loc_core_report_log(gz_logManager.u_cmprs_output_buffer, q_compressed_length);
        memset(gz_logManager.u_cmprs_output_buffer, 0, gz_logManager.q_cmprs_capacity);
      }
    }
    else
    {
      loc_core_report_log(data, len);
    }
  }

  return TRUE;
}

/**
 * @brief fflush log fifo
 * @return: None
 */
void log_fflush()
{
  ipctask_t* t = ipctask_GetInstance(TASK_INDEX_LOG);
  signal_trigger(&t->z_sgnl);
}

static DataLogManager_t z_DataLogManager = { 0 };


/**
 * @brief Release datalog file pinter
 * 
 */
void log_DataLogManager_Release()
{
  for (uint8_t i = 0; i < DLOG_MAX; i++)
  {
    log_fclose_safe(z_DataLogManager.p_DataLogHander[i]);
  }
}

/**
 * @brief Initilize Data log manager
 *        Only use in playback mode
 * @param[in] pz_DataLogManagerConfig : Data log module manager config structure
 * @return: None
 */
void log_DataLogManager_Initialize(DataLogManagerConfig_t* pz_DataLogManagerConfig)
{
  if (NULL == pz_DataLogManagerConfig)
  {
    return;
  }

  z_DataLogManager.u_Enable = pz_DataLogManagerConfig->u_Enable;
  if (FALSE == z_DataLogManager.u_Enable)
  {
    return;
  }

  char filename[256] = { 0 };
  for (uint8_t u_i = 0; u_i < DLOG_MAX; u_i++)
  {
    if (strlen((const char*)pz_DataLogManagerConfig->p_DataLogFilepath[u_i]) > 0)
    {
      snprintf(filename, sizeof(filename), "%s%s", pz_DataLogManagerConfig->pb_output_dir, pz_DataLogManagerConfig->p_DataLogFilepath[u_i]);
      z_DataLogManager.p_DataLogHander[u_i] = fopen((const char*)filename, "wb+");
    }
  }

  return;
}

/**
 * @brief Check Data log manager is enable
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
BOOL log_DataLogManager_IsEnable()
{
  return z_DataLogManager.u_Enable;
}

/**
 * @brief Print Data log string
 * @param[in] datalog_tag
 * @param[in] fmt
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
void log_DataLogManager_Print(DataLogTagEnum datalog_tag, const char* fmt, ...)
{
  if ((z_DataLogManager.u_Enable == FALSE) ||
     (datalog_tag >= DLOG_MAX))
  {
    return;
  }

  char buffer[LOG_TEXT_MAX_LENGTH] = { 0 };
  uint32_t n = 0;
  va_list ap;
  va_start(ap, fmt);

  n += vsnprintf(buffer, LOG_TEXT_MAX_LENGTH, fmt, ap);

  va_end(ap);

  if (z_DataLogManager.p_DataLogHander[datalog_tag])
  {
    fwrite(buffer, n, 1, z_DataLogManager.p_DataLogHander[datalog_tag]);
  }

  return;
}

BOOL log_DataLogHeader_IsEnable(DataLogTagEnum datalog_tag)
{
  if ((z_DataLogManager.u_Enable == FALSE) || (datalog_tag >= DLOG_MAX)
    || (TRUE == z_DataLogManager.u_Fileheader[datalog_tag]))
  {
    return FALSE;
  }
  return TRUE;
}

/**
 * @brief Print Data log string in file header
 * @param[in] datalog_tag
 * @param[in] fmt
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
void log_DataLogManager_Print_Header(DataLogTagEnum datalog_tag, const char* fmt, ...)
{
  if(!log_DataLogHeader_IsEnable(datalog_tag))
  {
    return;
  }

  char buffer[LOG_TEXT_MAX_LENGTH] = { 0 };
  uint32_t n = 0;
  va_list ap;
  va_start(ap, fmt);

  n += vsnprintf(buffer, LOG_TEXT_MAX_LENGTH, fmt, ap);

  va_end(ap);

  if (z_DataLogManager.p_DataLogHander[datalog_tag])
  {
    fwrite(buffer, n, 1, z_DataLogManager.p_DataLogHander[datalog_tag]);
  }
  z_DataLogManager.u_Fileheader[datalog_tag] = TRUE;
}

/**
 * @brief Print BDDB log string
 * @param[in] datalog_tag
 * @param[in] str
 * @return: TRUE - Enabled
 *          FALSE - Disbled
 */
void log_BDDBLogManager_Print(DataLogTagEnum datalog_tag, const unsigned char* str, uint32_t len)
{
  unsigned char buffer[LOG_BDDB_MAX_LENGTH] = { 0 };

  if ((z_DataLogManager.u_Enable == FALSE) ||
      (datalog_tag >= DLOG_MAX))
  {
    return;
  }
  if (datalog_tag != DLOG_VDR_BDDB)
  {
    return;
  }
  if (len > LOG_BDDB_MAX_LENGTH)
  {
    return;
  }

  memcpy(buffer, str, sizeof(unsigned char) * len);

  if (z_DataLogManager.p_DataLogHander[datalog_tag])
  {
    fwrite(buffer, len, 1, z_DataLogManager.p_DataLogHander[datalog_tag]);
  }
}

/**
 * @brief File pointer close safely
 * @param[in] fp file pointer
 */
void log_fclose_safe(FILE* fp)
{
  if (fp != NULL)
  {
    fclose(fp);
    fp = NULL;
  }
}

int32_t log_DataLogHeaderSprintf(DataLogTagEnum datalog_tag, char *buffer, const char *format, ...)
{
  if(!log_DataLogHeader_IsEnable(datalog_tag))
  {
    return 0;
  }
  va_list args;
  va_start(args, format);
  int32_t size = vsprintf(buffer, format, args);
  va_end(args);
  return size;
}
