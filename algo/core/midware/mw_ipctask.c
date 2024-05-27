/**@file        mw_ipctask.cpp
 * @brief       Inter-Process-Commucation Task Implement
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
#include <time.h>

#include "mw_alloc.h"
#include "mw_ipctask.h"
#include "mw_log.h"

#include "sm_api.h"
#include "hpp_api.h"
#include "dcp_api.h"
#include "ppp_api.h"
#include "sd_api.h"
#include "vdr_api.h"
#include "ekf_api.h"
#include "ort_api.h"

#include "cmn_utils.h"

#define IPC_VERSION (1)
#ifdef ENABLE_LARGE_LOG_CACHE
#define LOG_CACHE_MULTIPLE  10
#else
#define LOG_CACHE_MULTIPLE  4
#endif

extern uint64_t timepro_get_now();

static ipctask_t    ipctask_pool[TASK_INDEX_MAX];
static uint32_t     ipctask_thread_ids[TASK_INDEX_MAX];
static ipc_filter_t gz_ipc_filter;

static mutex_t log_ipc_mtx;

static struct {
  uint32_t ipc_id;
  const char* ipc_name;
} ipc_id_names[] = {
  {C_M_SM_INIT,                         "C_M_SM_INIT"                           },
  {C_M_SM_START,                        "C_M_SM_START"                          },
  {C_M_SM_STOP,                         "C_M_SM_STOP"                           },
  {C_M_SM_RELEASE,                      "C_M_SM_RELEASE"                        },
  {C_M_SM_RCV_OBS_RTCM,                 "C_M_SM_RCV_OBS_RTCM"                   },
  {C_M_SM_REF_CORR_RTCM,                "C_M_SM_REF_CORR_RTCM"                  },
  {C_M_SM_REPORT_POS_FIX,               "C_M_SM_REPORT_POS_FIX"                 },
  {C_M_SM_SSR_LOS_BLK,                  "C_M_SM_SSR_LOS_BLK"                    },
  {C_M_SM_RCV_MEAS_PUT,                 "C_M_SM_RCV_MEAS_PUT"                   },
  {C_M_SM_SSR_STREAM_QX,                "C_M_SM_SSR_STREAM_QX"                  },
  {C_M_SM_SSR_STREAM_GEE,               "C_M_SM_SSR_STREAM_GEE"                 },
  {C_M_SM_TWIN_RCV_OBS_RTCM,            "C_M_SM_TWIN_RCV_OBS_RTCM"              },
  {C_M_SM_REF_CORR_MEAS_BLK,            "C_M_SM_REF_CORR_MEAS_BLK"              },
  {C_M_SM_REPORT_GNSS_FB_INS,           "C_M_SM_REPORT_GNSS_FB_INS"             },
  {C_M_SM_REPORT_GNSS_EPH,              "C_M_SM_REPORT_GNSS_EPH"                },
  {C_M_HPP_INIT,                        "C_M_HPP_INIT"                          },
  {C_M_HPP_START,                       "C_M_HPP_START"                         },
  {C_M_HPP_STOP,                        "C_M_HPP_STOP"                          },
  {C_M_HPP_RELEASE,                     "C_M_HPP_RELEASE"                       },
  {C_M_HPP_RCV_MEAS_1HZ_PUT,            "C_M_HPP_RCV_MEAS_1HZ_PUT"              },
  {C_M_HPP_REF_CORR_PUT,                "C_M_HPP_REF_CORR_PUT"                  },
  {C_M_HPP_EPHEMERIS_PUT,               "C_M_HPP_EPHEMERIS_PUT"                 },
  {C_M_HPP_RCV_MEAS_RTCM,               "C_M_HPP_RCV_MEAS_RTCM"                 },
  {C_M_HPP_REF_CORR_RTCM,               "C_M_HPP_REF_CORR_RTCM"                 },
  {C_M_HPP_RCV_MEAS_TWIN_PUT,           "C_M_HPP_RCV_MEAS_TWIN_PUT"             },
  {C_M_DCP_INIT,                        "C_M_DCP_INIT"                          },
  {C_M_DCP_START,                       "C_M_DCP_START"                         },
  {C_M_DCP_STOP,                        "C_M_DCP_STOP"                          },
  {C_M_DCP_RELEASE,                     "C_M_DCP_RELEASE"                       },
  {C_M_DCP_RCV_MEAS_NHZ_PUT,            "C_M_DCP_RCV_MEAS_NHZ_PUT"              },
  {C_M_DCP_TDCP_MEAS_PUT,               "C_M_DCP_TDCP_MEAS_PUT"                 },
  {C_M_PPP_INIT,                        "C_M_PPP_INIT"                          },
  {C_M_PPP_START,                       "C_M_PPP_START"                         },
  {C_M_PPP_STOP,                        "C_M_PPP_STOP"                          },
  {C_M_PPP_RELEASE,                     "C_M_PPP_RELEASE"                       },
  {C_M_PPP_RCV_MEAS_1HZ_PUT,            "C_M_PPP_RCV_MEAS_1HZ_PUT"              },
  {C_M_PPP_SSR_LOS_BLK,                 "C_M_PPP_SSR_LOS_BLK"                   },
  {C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT,   "C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT"     },
  {C_M_PPP_ALGO_START,                  "C_M_PPP_ALGO_START"                    },
  {C_M_PPP_SSR_STREAM_QX,               "C_M_PPP_SSR_STREAM_QX"                 },
  {C_M_PPP_SSR_STREAM_GEE,              "C_M_PPP_SSR_STREAM_GEE"                },
  {C_M_SD_INIT,                         "C_M_SD_INIT"                           },
  {C_M_SD_START,                        "C_M_SD_START"                          },
  {C_M_SD_STOP,                         "C_M_SD_STOP"                           },
  {C_M_SD_RELEASE,                      "C_M_SD_RELEASE"                        },
  {C_M_SD_EPHEMERIS_PUT,                "C_M_SD_EPHEMERIS_PUT"                  },
  {C_M_SD_SAT_PVT_POLY_REQUEST,         "C_M_SD_SAT_PVT_POLY_REQUEST"           },
  {C_M_VDR_INIT,                        "C_M_VDR_INIT"                          },
  {C_M_VDR_START,                       "C_M_VDR_START"                         },
  {C_M_VDR_STOP,                        "C_M_VDR_STOP"                          },
  {C_M_VDR_RELEASE,                     "C_M_VDR_RELEASE"                       },
  {C_M_VDR_IMU_DATA_PUT,                "C_M_VDR_IMU_DATA_PUT"                  },
  {C_M_VDR_GNSS_FIX_PUT,                "C_M_VDR_GNSS_FIX_PUT"                  },
  {C_M_VDR_WHEEL_DATA_PUT,				"C_M_VDR_WHEEL_DATA_PUT"				},
  {C_M_VDR_EKF_UPDATE_PUT,              "C_M_VDR_EKF_UPDATE_PUT"                },
  {C_M_VDR_GNSS_MEAS_PUT,               "C_M_VDR_GNSS_MEAS_PUT"                 },
  {C_M_EKF_INIT,						"C_M_EKF_INIT"						    },
  {C_M_EKF_START, 					    "C_M_EKF_START"						    },
  {C_M_EKF_STOP,						"C_M_EKF_STOP"						    },
  {C_M_EKF_RELEASE,					    "C_M_EKF_RELEASE" 					    },
  {C_M_EKF_SEM_DATA_PUT,				"C_M_EKF_SEM_DATA_PUT"				    },
  {C_M_SM_REPORT_ORT_FIX,				"C_M_SM_REPORT_ORT_FIX"				    }
};

/**
 * @brief Convert ipc id to string
 * @return: string
 */
const char* ipctask_get_string_from_id(uint32_t q_ipc_id)
{
  uint32_t q_arr_count = sizeof(ipc_id_names) / sizeof(ipc_id_names[0]);
  for (uint32_t i = 0; i < q_arr_count; i++)
  {
    if (ipc_id_names[i].ipc_id == q_ipc_id)
    {
      return ipc_id_names[i].ipc_name;
    }
  }

  return "N/A";
}

/**
 * @brief Convert string to ipc id
 * @return: string
 */
uint32_t ipctask_get_id_from_string(const char* p_ipc_string)
{
  uint32_t q_arr_count = sizeof(ipc_id_names) / sizeof(ipc_id_names[0]);
  for (uint32_t i = 0; i < q_arr_count; i++)
  {
    if (0 == strcmp(ipc_id_names[i].ipc_name, p_ipc_string))
    {
      return ipc_id_names[i].ipc_id;
    }
  }

  return C_M_IPC_BEGINNING;
}

/**
 * @brief IPC filter instance set, the filter input is from configuration
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_ipc_filter_set(ipc_filter_t* pz_ipc_filter)
{
  if (NULL == pz_ipc_filter)
  {
    return FALSE;
  }

  uint32_t len1 = 0;
  uint32_t len2 = 0;
  uint32_t len = 0;

  for (uint8_t u_i = 0; u_i < TASK_INDEX_MAX; u_i++)
  {
    gz_ipc_filter.task_enable_list[u_i] = pz_ipc_filter->task_enable_list[u_i];
  }
  
  len1 = M_ARRAY_SIZE(gz_ipc_filter.filter_pass_log_ipc);
  len2 = M_ARRAY_SIZE(pz_ipc_filter->filter_pass_log_ipc);
  len = len1 < len2 ? len1 : len2;
  for (uint8_t u_i = 0; u_i < len; u_i++)
  {
    gz_ipc_filter.filter_pass_log_ipc[u_i] = pz_ipc_filter->filter_pass_log_ipc[u_i];
  }

  len1 = M_ARRAY_SIZE(gz_ipc_filter.filter_pass_live_ipc);
  len2 = M_ARRAY_SIZE(pz_ipc_filter->filter_pass_live_ipc);
  len = len1 < len2 ? len1 : len2;
  for (uint8_t u_i = 0; u_i < len; u_i++)
  {
    gz_ipc_filter.filter_pass_live_ipc[u_i] = pz_ipc_filter->filter_pass_live_ipc[u_i];
  }

  len1 = M_ARRAY_SIZE(gz_ipc_filter.filter_need_saved_ipc);
  len2 = M_ARRAY_SIZE(pz_ipc_filter->filter_need_saved_ipc);
  len = len1 < len2 ? len1 : len2;
  for (uint8_t u_i = 0; u_i < len; u_i++)
  {
    if (0 == pz_ipc_filter->filter_need_saved_ipc[u_i])
    {
      break;
    }
    gz_ipc_filter.filter_need_saved_ipc[gz_ipc_filter.filter_need_saved_ipc_count++] = 
      pz_ipc_filter->filter_need_saved_ipc[u_i];
  }

  return TRUE;
}

/**
 * @brief IPC validity check
 * @return: TRUE - IPC is valid
 *          FALSE - IPC is invalid and should be filtered
 */
BOOL ipctask_ipc_validity_check(uint8_t u_dst_id, 
  uint8_t u_src_id, uint32_t q_ipc_id, IpcValidityEnum u_ipc_validity_type)
{
  if (q_ipc_id >= C_M_IPC_END || u_dst_id >= TASK_INDEX_MAX)
  {
    return FALSE;
  }

  if (u_ipc_validity_type == IPC_VALIDITY_TYPE_LOG)
  {
    for (uint32_t i = 0; i < gz_ipc_filter.filter_need_saved_ipc_count; i++)
    {
      if (q_ipc_id == gz_ipc_filter.filter_need_saved_ipc[i])
      {
        return TRUE;
      }
    }
    return FALSE;
  }
  else if (u_ipc_validity_type == IPC_VALIDITY_TYPE_PLAYBACK)
  {
    if (FALSE == gz_ipc_filter.task_enable_list[u_dst_id])
    {
      return FALSE;
    }

    for (uint32_t i = 0; i < M_ARRAY_SIZE(gz_ipc_filter.filter_pass_log_ipc); i++)
    {
      if (q_ipc_id == gz_ipc_filter.filter_pass_log_ipc[i])
      {
        return FALSE;
      }
    }
  }
  else if (u_ipc_validity_type == IPC_VALIDITY_TYPE_LIVE)
  {
    for (uint32_t i = 0; i < M_ARRAY_SIZE(gz_ipc_filter.filter_pass_live_ipc); i++)
    {
      if (q_ipc_id == gz_ipc_filter.filter_pass_live_ipc[i])
      {
        return FALSE;
      }
    }
  }

  return TRUE;
}

/**
 * @brief All IPC tasks initialize
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Init()
{
  for (uint8_t i = 0; i < TASK_INDEX_MAX; i++)
  {
    ipctask_t* t = &ipctask_pool[i];
    t->e_status = TASK_STATUS_DISABLE;
    t->pz_fifo = NULL;
    t->e_index = TASK_INDEX_MAX;

    switch (i)
    {
      case TASK_INDEX_SM:
        t->e_index = TASK_INDEX_SM;
        t->name = "SM Task";
        t->stack_size = 8 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
		t->priority = TASK_MID_LEVEL;
        break;
      case TASK_INDEX_HPP:
        t->e_index = TASK_INDEX_HPP;
        t->name = "HPP Task";
        t->stack_size = 60 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
		t->priority = TASK_NORMAL_LEVEL;
        break;
      case TASK_INDEX_DCP:
        t->e_index = TASK_INDEX_DCP;
        t->name = "DCP Task";
        t->stack_size = 48 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
		t->priority = TASK_MID_LEVEL;
        break;
      case TASK_INDEX_PPP:
        t->e_index = TASK_INDEX_PPP;
        t->name = "PPP Task";
        t->stack_size = 8 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
		t->priority = TASK_NORMAL_LEVEL;
        break;
      case TASK_INDEX_SD:
        t->e_index = TASK_INDEX_SD;
        t->name = "SD Task";
        t->stack_size = 6 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
		t->priority = TASK_NORMAL_LEVEL;
        break;
      case TASK_INDEX_VDR:
        t->e_index = TASK_INDEX_VDR;
        t->name = "VDR Task";
        t->stack_size = 64 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
        t->priority = TASK_HIGH_LEVEL;
        break;
      case TASK_INDEX_EKF:
        t->e_index = TASK_INDEX_EKF;
        t->name = "EKF Task";
        t->stack_size = 8 * 1024;
        t->capacity = 64 * sizeof(ipc_t);
        t->priority = TASK_MID_LEVEL;
        break;
      case TASK_INDEX_ORT:
        t->e_index = TASK_INDEX_ORT;
        t->name = "ORT Task";
        t->stack_size = 90 * 1024;
        t->capacity = 128 * sizeof(ipc_t);
		t->priority = TASK_NORMAL_LEVEL;
        break;
      case TASK_INDEX_LOG:
        t->e_index = TASK_INDEX_LOG;
        t->name = "Log Task";
        t->stack_size = 8 * 1024;
        t->capacity = LOG_CACHE_MAX_LENGTH * LOG_CACHE_MULTIPLE;
        break;
      default:
        break;
    }
  }

  mutex_init(&log_ipc_mtx, NULL);
  
  return TRUE;
}

/**
 * @brief Start a specific ipctask
 * @param[in]   task_id 
 * @param[in]   ipc_capacity - message receive fifo store max ipc number
 * @param[in]   taskProc - message process function
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Start(TaskIndex_e task_id, thread_proc_t taskProc, TaskStatus_e status)
{
  if (task_id >= TASK_INDEX_MAX)
  {
    return FALSE;
  }

  ipctask_t* t = ipctask_GetInstance(task_id);
  if ((NULL == t) || 
      (TASK_INDEX_MAX == t->e_index) ||
      (0 == t->stack_size) || 
      (0 == t->capacity) ||
      (status == TASK_STATUS_DISABLE))
  {
    return FALSE;
  }

  if (status == TASK_STATUS_SINGLE)
  {
    t->e_status = status;
    return TRUE;
  }

  mutex_init(&t->z_mtx, NULL);
  signal_init(&t->z_sgnl);
  t->pz_fifo = fifo_create(1, t->capacity);
#ifdef ENABLE_TASK_STACK
  t->stack = OS_MALLOC(t->stack_size);
#endif
  thread_create(&t->z_thread_id, t->name, taskProc, &t->float_hard_enable, (thread_stk_t*)t->stack, t->stack_size, t->priority);

  uint32_t count = 0;
  while (t->e_status != TASK_STATUS_ENABLE)
  {
    thread_sleep(5);
    count++;
    if (count == 200)
    {
      break;
    }
  }

  return TRUE;
}

/**
 * @brief Stop a specific ipctask
 * @param[in]   task_id
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Stop(TaskIndex_e task_id)
{
  /* Core ctrl task couldn't stop */
  if ((task_id >= TASK_INDEX_MAX) || (task_id == TASK_INDEX_SM))
  {
    return FALSE;
  }

  ipctask_t* t = &ipctask_pool[task_id];
  if (TASK_STATUS_DISABLE == t->e_status)
  {
    return TRUE;
  }

  t->e_status = TASK_STATUS_DISABLE;

  /* Send a last message to avoid task is still waitting for signal after 
     IPC status is set as STOP */
  ipctask_SendMessage(task_id, C_M_IPC_END, NULL, 0);

  thread_join(&t->z_thread_id, 1000);
#ifdef ENABLE_TASK_STACK
  OS_FREE(t->stack);
#endif
  mutex_destroy(&t->z_mtx);
  signal_destroy(&t->z_sgnl);
  fifo_destroy(t->pz_fifo);

  return TRUE;
}

/**
 * @brief All IPC tasks Release
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
BOOL ipctask_Release()
{
  mutex_destroy(&log_ipc_mtx);

  return TRUE;
}

/**
 * @brief Get a ipctask instance
 * @param[in]   task_id
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
ipctask_t* ipctask_GetInstance(TaskIndex_e id)
{
  if (id >= TASK_INDEX_MAX)
  {
    return NULL;
  }

  return &ipctask_pool[id];
}

/**
 * @brief Set thread id to task id 
 * @param[in]   thread id from get_thread_id function
 * @param[in]   task id
 * @return:
 */
void ipctask_SetThreadId2TaskId(uint32_t thread_id, TaskIndex_e task_id)
{
  if (task_id >= TASK_INDEX_MAX)
  {
    return;
  }

  ipctask_thread_ids[task_id] = thread_id;
}

/**
 * @brief Get thread id to task id 
 * @param[in]   thread id from get_thread_id function
 * @return: task id
 */
TaskIndex_e ipctask_GetThreadId2TaskId(uint32_t thread_id)
{
  TaskIndex_e task_id = 0;
  for (task_id = 0; task_id < TASK_INDEX_MAX; task_id++)
  {
    if (ipctask_thread_ids[task_id] == thread_id)
    {
      break;
    }
  }

  return task_id;
}

/**
 * @brief Prpccess a message if Task isn't started.
 */
void ipctask_ProcMessage_Single(TaskIndex_e dst, uint32_t ipc_id,
  uint8_t* data, uint32_t size)
{
  if (FALSE == ipctask_ipc_validity_check(dst, TASK_INDEX_INVALID, ipc_id, IPC_VALIDITY_TYPE_LIVE))
  {
    return;
  }

  ipc_t ipc = { 0 };
  ipctask_PackDataToIpc(&ipc, dst, ipc_id, data, size);
  log_ipc(&ipc);

  switch (dst)
  {
  case TASK_INDEX_SM:
    sm_Proc(&ipc);
    break;
  case TASK_INDEX_HPP:
    hpp_Proc(&ipc);
    break;
  case TASK_INDEX_DCP:
    dcp_Proc(&ipc);
    break;
  case TASK_INDEX_PPP:
    ppp_Proc(&ipc);
    break;
  case TASK_INDEX_SD:
    sd_Proc(&ipc);
    break;
  case TASK_INDEX_VDR:
    vdr_Proc(&ipc);
    break;
  case TASK_INDEX_EKF:
    ekf_Proc(&ipc);
    break;
  case TASK_INDEX_ORT:
    ort_Proc(&ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief Send a IPC message with related data to destination ipctask
 * @param[in] dst
 * @param[in] src
 * @param[in] ipc_id
 * @param[in] data
 * @param[in] size
 * @return: None
 * @note In this function, it will call malloc and alloc a new memory to store 
 *       input data. And this memory must free after process the IPC.
 */
void ipctask_SendMessage(TaskIndex_e dst, uint32_t ipc_id,
                     uint8_t* data, uint32_t size)
{
  size_t ret = 0;
  ipc_t ipc = { 0 };
  ipctask_t* t = &ipctask_pool[dst];

  if (TASK_STATUS_SINGLE == t->e_status)
  {
    ipctask_ProcMessage_Single(dst, ipc_id, data, size);
    return;
  }

  if (TASK_STATUS_ENABLE != t->e_status)
  {
    return;
  }

  ipc.u_version = IPC_VERSION;
  ipc.u_src_id = ipctask_GetThreadId2TaskId(get_thread_id());;
  ipc.u_dst_id = dst;
  ipc.q_ipc_id = ipc_id;
  ipc.t_timestamp_ms = timepro_get_now();

  if ((size > 0) && (NULL != data))
  {
    ipc.q_length = size;
    ipc.p_data = (uint8_t*)IPC_MALLOC(size, ipc.q_ipc_id);
    if (ipc.p_data)
    {
      memcpy(ipc.p_data, data, size);
    }
    else
    {
      return;
    }
  }
  else
  {
    ipc.q_length = 0;
    ipc.p_data = NULL;
  }

  mutex_lock(&t->z_mtx);
  ret = fifo_push(t->pz_fifo, &ipc, sizeof(ipc_t));
  mutex_unlock(&t->z_mtx);
  if(0 == ret){
      if (NULL != ipc.p_data)
      {
         OS_FREE(ipc.p_data);
	     ipc.p_data = NULL;
	     ipc.q_length = 0;
      }
  }
  signal_trigger(&t->z_sgnl);
}

/**
 * @brief Receive a IPC message with related data, if
 * @param[in] dst
 * @param[in] pz_ipc
 * @return: None
 */
BOOL ipctask_ReceiveMessage(TaskIndex_e dst, ipc_t* pz_ipc)
{
  size_t ret = 0;
  ipctask_t* t = ipctask_GetInstance(dst);

  mutex_lock(&t->z_mtx);
  if (0 == fifo_used(t->pz_fifo))
  {
    signal_wait(&t->z_sgnl, &t->z_mtx, 0);
    mutex_unlock(&t->z_mtx);
    return FALSE;
  }
  ret = fifo_pull(t->pz_fifo, pz_ipc, sizeof(ipc_t));
  mutex_unlock(&t->z_mtx);
  if (ret != sizeof(ipc_t))
  {    
    return FALSE;
  }

  log_ipc(pz_ipc);
  
  return TRUE;
}

/**
 * @brief Release IPC message
 * @param[in] dst
 * @param[in] pz_ipc
 * @return: None
 */
void ipctask_ReleaseMessage(ipc_t* ipc)
{
  if (NULL != ipc->p_data)
  {
    OS_FREE(ipc->p_data);
    ipc->p_data = NULL;
    ipc->q_length = 0;
  }
  return;
}

/**
 * @brief Convert a data to ipc structure
 * @return: None
 */
void ipctask_PackDataToIpc(ipc_t* pz_ipc, TaskIndex_e dst, uint32_t ipc_id,
  uint8_t* data, uint32_t size)
{
  //struct timespec ts;
  if (NULL == pz_ipc)
  {
    return;
  }
  pz_ipc->u_version = IPC_VERSION;
  pz_ipc->u_src_id = ipctask_GetThreadId2TaskId(get_thread_id());;
  pz_ipc->u_dst_id = dst;
  pz_ipc->q_ipc_id = ipc_id;
  pz_ipc->t_timestamp_ms = timepro_get_now();

  if ((size > 0) && (NULL != data))
  {
    pz_ipc->q_length = size;
    pz_ipc->p_data = data;
  }
  else
  {
    pz_ipc->q_length = 0;
    pz_ipc->p_data = NULL;
  }
}
