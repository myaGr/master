/**@file        mw_ipctask.h
 * @brief       Inter-Process-Commucation Task Implement
 * @author      caizhijie
 * @date        2022/04/25
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __MW_IPC_TASK_H__
#define __MW_IPC_TASK_H__

#include "os_thread.h"
#include "os_mutex.h"
#include "mw_fifo.h"

BEGIN_DECL

/* To enable task hard float feature */
typedef void (*v_float_hard_enable_func_p)(void);

typedef enum {
  TASK_LOW_LEVEL = 0,
  TASK_NORMAL_LEVEL,
  TASK_MID_LEVEL,
  TASK_HIGH_LEVEL,
  TASK_INVALID_LEVEL = 0xFF
} TaskRunning_Level;

typedef enum {
  TASK_INDEX_SM,
  TASK_INDEX_HPP,
  TASK_INDEX_DCP,
  TASK_INDEX_PPP,
  TASK_INDEX_SD,
  TASK_INDEX_VDR,
  TASK_INDEX_EKF,
  TASK_INDEX_ORT,
  TASK_INDEX_LOG,
  TASK_INDEX_MAX,
  TASK_INDEX_INVALID = 0xFF
} TaskIndex_EnumVal;
typedef uint8_t TaskIndex_e;

#define TASK_INDEX_USER TASK_INDEX_MAX

/** Inter Process Commucation Message type list */
typedef enum {
  C_M_IPC_BEGINNING = (uint32_t)0x00000000,
  
  /* List Core CTRL Receive IPCs */
  C_M_SM_IPC_BEGINNING = (uint32_t)0x00000000,
  C_M_SM_INIT,
  C_M_SM_START,
  C_M_SM_STOP,
  C_M_SM_RELEASE,
  C_M_SM_RCV_OBS_RTCM,
  C_M_SM_REF_CORR_RTCM,
  C_M_SM_REPORT_POS_FIX,
  C_M_SM_SSR_LOS_BLK,
  C_M_SM_RCV_MEAS_PUT,
  C_M_SM_SSR_STREAM_QX,
  C_M_SM_SSR_STREAM_GEE,
  C_M_SM_SSR_AG_LOS,
  C_M_SM_TWIN_RCV_OBS_RTCM,
  C_M_SM_REF_CORR_MEAS_BLK,
  C_M_SM_REPORT_GNSS_FB_INS,
  C_M_SM_REPORT_GNSS_EPH,
  C_M_SM_ORT_MEAS_PUT,
  C_M_SM_REPORT_ORT_FIX,
  C_M_SM_REPORT_RCV_OBS_MEAS_BLK,
  C_M_SM_REPORT_RCV_OBS_MEAS_SAT_BLK,
  C_M_SM_REPORT_NAV_DATA_FRAME,
  C_M_SM_IPC_END,

  /* List HPP Receive IPCs */
  C_M_HPP_IPC_BEGINNING = (uint32_t)0x00010000,
  C_M_HPP_INIT,
  C_M_HPP_START,
  C_M_HPP_STOP,
  C_M_HPP_RELEASE,
  C_M_HPP_RCV_MEAS_1HZ_PUT,
  C_M_HPP_REF_CORR_PUT,
  C_M_HPP_EPHEMERIS_PUT,
  C_M_HPP_RCV_MEAS_RTCM,
  C_M_HPP_REF_CORR_RTCM,
  C_M_HPP_RCV_MEAS_TWIN_PUT,
  C_M_HPP_IPC_END,
  
  C_M_DCP_IPC_BEGINNING = (uint32_t)0x00020000,
  C_M_DCP_INIT,
  C_M_DCP_START,
  C_M_DCP_STOP,
  C_M_DCP_RELEASE,
  C_M_DCP_RCV_MEAS_NHZ_PUT,
  C_M_DCP_RCV_ORT_MEAS_NHZ_PUT,
  C_M_DCP_TDCP_MEAS_PUT,
  C_M_DCP_MAIN_ANT_DCP_MEAS_PUT,
  C_M_DCP_AUXI_ANT_DCP_MEAS_PUT,
  C_M_DCP_ORT_BACK_GROUND_PUT,
  C_M_DCP_IPC_END,
  
  C_M_PPP_IPC_BEGINNING = (uint32_t)0x00030000,
  C_M_PPP_INIT,
  C_M_PPP_START,
  C_M_PPP_STOP,
  C_M_PPP_RELEASE,
  C_M_PPP_RCV_MEAS_1HZ_PUT,
  C_M_PPP_SSR_LOS_BLK,
  C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT,
  C_M_PPP_ALGO_START,
  C_M_PPP_SSR_STREAM_QX,
  C_M_PPP_SSR_STREAM_GEE,
  C_M_PPP_SSR_STREAM_AG_LOS,
  C_M_PPP_IPC_END,
  
  C_M_SD_IPC_BEGINNING = (uint32_t)0x00040000,
  C_M_SD_INIT,
  C_M_SD_START,
  C_M_SD_STOP,
  C_M_SD_RELEASE,
  C_M_SD_EPHEMERIS_PUT,
  C_M_SD_SAT_PVT_POLY_REQUEST,
  C_M_SD_IPC_END,

  C_M_VDR_IPC_BEGINNING = (uint32_t)0x00050000,
  C_M_VDR_INIT,
  C_M_VDR_START,
  C_M_VDR_STOP,
  C_M_VDR_RELEASE,
  C_M_VDR_IMU_DATA_PUT,
  C_M_VDR_GNSS_FIX_PUT,
  C_M_VDR_WHEEL_DATA_PUT,
  C_M_VDR_EKF_UPDATE_PUT,
  C_M_VDR_GNSS_MEAS_PUT,
  C_M_VDR_IPC_END,

  C_M_ORT_IPC_BEGINNING = (uint32_t)0x00060000,
  C_M_ORT_INIT,
  C_M_ORT_START,
  C_M_ORT_STOP,
  C_M_ORT_RELEASE,
  C_M_ORT_RCV_MEAS_PUT,
  C_M_ORT_ORT_MEAS_PUT,
  C_M_ORT_MAIN_ANT_PVT_INFO,
  C_M_ORT_IPC_END,

  C_M_LOG_IPC_BEGINNING = (uint32_t)0x00070000,
  C_M_LOG_IPC_END,

  C_M_EKF_IPC_BEGINNING = (uint32_t)0x00080000,
  C_M_EKF_INIT,
  C_M_EKF_START,
  C_M_EKF_STOP,
  C_M_EKF_RELEASE,
  C_M_EKF_SEM_DATA_PUT,
  C_M_EKF_IPC_END,

  C_M_IPC_END           = (uint32_t)0x000F0000,
} IPC_Id_EnumVal;
typedef uint32_t IPC_Id_Enum;

typedef enum {
  IPC_VALIDITY_TYPE_LOG = 0,
  IPC_VALIDITY_TYPE_PLAYBACK,
  IPC_VALIDITY_TYPE_LIVE
} IpcValidity_EnumVal;
typedef uint8_t IpcValidityEnum;

#define IPC_ID_NUMBER(id) (id&0x0000FFFF)
#define IPC_ID_GROUP(id)  ((id%0xFFFF0000)>>16)

typedef enum {
  TASK_STATUS_DISABLE = 0,
  TASK_STATUS_ENABLE,
  TASK_STATUS_SINGLE
} TaskStatus_EnumVal;
typedef uint8_t TaskStatus_e;

typedef struct {
  uint8_t   u_version;
  uint8_t   u_src_id;
  uint8_t   u_dst_id;
  uint8_t   u_use_heap    : 1;
  uint8_t   u_need_record : 1;
  uint32_t  q_ipc_id;
  uint64_t  t_timestamp_ms;
  uint32_t  q_reserve;
  uint32_t  q_length;
  uint8_t*  p_data;
} ipc_t;

typedef struct {
  uint8_t   task_enable_list[TASK_INDEX_MAX];
  uint32_t  filter_pass_log_ipc[16];
  uint32_t  filter_pass_live_ipc[16];
  uint32_t  filter_need_saved_ipc_count;
  uint32_t  filter_need_saved_ipc[64];
} ipc_filter_t;

typedef struct {
  TaskStatus_e    e_status;
  TaskIndex_e     e_index;
  const char*     name;
  uint32_t        stack_size;
  uint32_t        capacity;
  mutex_t         z_mtx;
  signal_t        z_sgnl;
  thread_id_t     z_thread_id;
  fifo_t*         pz_fifo;
  void*           stack;
  uint32_t        priority;

  v_float_hard_enable_func_p float_hard_enable;
} ipctask_t;

/**
 * @brief Convert ipc id to string
 * @return: string
 */
const char* ipctask_get_string_from_id(uint32_t q_ipc_id);

/**
 * @brief Convert string to ipc id
 * @return: string
 */
uint32_t ipctask_get_id_from_string(const char* p_ipc_string);

/**
 * @brief IPC filter instance set, the filter input is from configuration
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_ipc_filter_set(ipc_filter_t* pz_ipc_filter);

/**
 * @brief IPC validity check
 * @return: TRUE - IPC is valid
 *          FALSE - IPC is invalid and should be filtered
 */
BOOL ipctask_ipc_validity_check(uint8_t u_dst_id,
  uint8_t u_src_id, uint32_t q_ipc_id, IpcValidityEnum u_ipc_validity_type);

/**
 * @brief All IPC tasks initialize
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Init();

/**
 * @brief Start a specific ipctask
 * @param[in]   task_id
 * @param[in]   taskProc - message process function
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Start(TaskIndex_e id, thread_proc_t taskProc, TaskStatus_e status);

/**
 * @brief Stop a specific ipctask
 * @param[in]   task_id
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Stop(TaskIndex_e id);

/**
 * @brief All IPC tasks Release
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Release();

/**
 * @brief Get a ipctask instance
 * @param[in]   task_id
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
ipctask_t* ipctask_GetInstance(TaskIndex_e id);

/**
 * @brief Set thread id to task id
 * @param[in]   thread id from get_thread_id function
 * @param[in]   task id
 * @return:
 */
void ipctask_SetThreadId2TaskId(uint32_t thread_id, TaskIndex_e task_id);

/**
 * @brief Get thread id to task id
 * @param[in]   thread id from get_thread_id function
 * @return: task id
 */
TaskIndex_e ipctask_GetThreadId2TaskId(uint32_t thread_id);

/**
 * @brief Send a IPC message with related data to destination ipctask
 * @param[in] dst
 * @param[in] ipc_id
 * @param[in] data
 * @param[in] size
 * @return: None
 * @note In this function, it will call malloc and alloc a new memory to store
 *       input data. And this memory must free after process the IPC.
 */
void ipctask_SendMessage(TaskIndex_e dst, uint32_t ipc_id,
                     uint8_t* data, uint32_t size);

/**
 * @brief Receive a IPC message with related data, if
 * @param[in] dst
 * @param[in] pz_ipc
 * @return: None
 */
BOOL ipctask_ReceiveMessage(TaskIndex_e dst, ipc_t* ipc);

/**
 * @brief Release IPC message
 * @param[in] dst
 * @param[in] pz_ipc
 * @return: None
 */
void ipctask_ReleaseMessage(ipc_t* ipc);

/**
 * @brief Convert a data to ipc structure
 * @return: None
 */
void ipctask_PackDataToIpc(ipc_t* pz_ipc, TaskIndex_e dst, uint32_t ipc_id,
  uint8_t* data, uint32_t size);

END_DECL

#endif
