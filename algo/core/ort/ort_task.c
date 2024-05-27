/**@file        ort_task.c
 * @brief       Double Antenna Orient Calculate Task
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/12  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "ort_api.h"
#include "ort_proc.h"
#include "sm_api.h"
#include "mw_alloc.h"
#include "dcp_api.h"
#include "gnss_common.h"

/**
 * @brief ORT module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ort_Init(ipc_t* p_ipc)
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParam = loc_cfg_getConfigParamGroup();
  rtk_alg_opt_t z_opt;
  ort_loadDefaultOption(&z_opt);
  if (pz_loc_ConfigParam->ort_cfg.b_enable)
  {
    if ((pz_loc_ConfigParam->sm_cfg.f_ele_mask) >= FABS_ZEROS && (pz_loc_ConfigParam->sm_cfg.f_ele_mask) <= 90.0f)
    {
      z_opt.d_elmin = (double)(pz_loc_ConfigParam->sm_cfg.f_ele_mask * DEG2RAD);
      ort_init(&z_opt);
    }
  }
  return;
}

/**
 * @brief ORT module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ort_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief ORT module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ort_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief ORT module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ort_Release(ipc_t* p_ipc)
{
  ort_deInit();
  return;
}

/**
 * @brief ORT module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ort_RcvMeasBlk_Put(ipc_t* p_ipc)
{
  if (sizeof(GnssMeasBlock_t) != p_ipc->q_length)
  {
    return;
  }

  GnssMeasBlock_t* pz_mainMeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;
  ort_pushMainAntMeas(pz_mainMeasBlock);
  return;
}

/**
 * @brief ORT module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ort_OrtMeasBlk_Put(ipc_t* p_ipc)
{
  GpsTime_t z_obsTime = { 0 };
  ort_solStatus_t u_status = 0;
  gnss_OrientFix_t z_attitudeResult = { 0 };
  gnss_TdcpMeasBlock_t* pz_mainAntTdcpMeasBlock = NULL;
  gnss_TdcpMeasBlock_t* pz_auxiAntTdcpMeasBlock = NULL;
  if (sizeof(GnssMeasBlock_t) != p_ipc->q_length)
  {
    return;
  }

  GnssMeasBlock_t* pz_auxiMeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;
  ort_pushAuxiAntMeas(pz_auxiMeasBlock);
  z_obsTime = pz_auxiMeasBlock->z_Clock.z_gpsTime;
  /* Fill week into measurement */
  gnss_AdjustMeasBlockWeek(pz_auxiMeasBlock->w_measCount, pz_auxiMeasBlock->z_meas, &z_obsTime);
  pz_mainAntTdcpMeasBlock = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  pz_auxiAntTdcpMeasBlock = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  u_status = ort_startProcess(z_obsTime, &z_attitudeResult, pz_mainAntTdcpMeasBlock, pz_auxiAntTdcpMeasBlock);
  if (ORT_SUCC == u_status || ORT_GET_MAIN_PVT_FAIL == u_status || ORT_GET_AUXI_ANT_PVT_FAIL == u_status || ORT_FILTER_FAIL == u_status)
  {
    sm_api_OrientFix_Report(&z_attitudeResult);
  }
  if (ORT_SUCC == u_status && NULL != pz_mainAntTdcpMeasBlock && NULL != pz_auxiAntTdcpMeasBlock && 0 != z_attitudeResult.u_ortFixFlag)
  {
    dcp_api_MainAntTdcpMeasBlk_Put(pz_mainAntTdcpMeasBlock);
    dcp_api_AuxiAntTdcpMeasBlk_Put(pz_auxiAntTdcpMeasBlock);
    dcp_api_orientBackGround_Put(&z_attitudeResult);
  }
  OS_FREE(pz_mainAntTdcpMeasBlock);
  OS_FREE(pz_auxiAntTdcpMeasBlock);
  return;
}

/**
 * @brief Sanity Check IPC
 * @param[in]   p_ipc - pointer to IPC message
 * @return: true - success
 *          false - fail for any reason
 */
static BOOL ort_IpcCheck(ipc_t* p_ipc)
{
  if (TASK_INDEX_ORT != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_ORT_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_ORT_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}
/**
 * @brief ORT module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ort_mainAntPvtInfo_Put(ipc_t* p_ipc)
{
  ort_solStatus_t u_status = 0;
  gnss_OrientFix_t z_attitudeResult = { 0 };
  gnss_TdcpMeasBlock_t* pz_mainAntTdcpMeasBlock = NULL;
  gnss_TdcpMeasBlock_t* pz_auxiAntTdcpMeasBlock = NULL;
  const gnss_PVTResult_t* pz_mainAntPvtInfo = NULL;
  if (sizeof(gnss_PVTResult_t) != p_ipc->q_length)
  {
    return;
  }

  memset(&z_attitudeResult, 0, sizeof(gnss_OrientFix_t));
  pz_mainAntPvtInfo = (const gnss_PVTResult_t*)p_ipc->p_data;
  ort_pushMainAntPosition(pz_mainAntPvtInfo);
  pz_mainAntTdcpMeasBlock = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  pz_auxiAntTdcpMeasBlock = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  u_status = ort_startProcess(pz_mainAntPvtInfo->z_positionFix.z_gpsTime, &z_attitudeResult, pz_mainAntTdcpMeasBlock, pz_auxiAntTdcpMeasBlock);
  if (ORT_SUCC == u_status || ORT_GET_MAIN_PVT_FAIL == u_status || ORT_GET_AUXI_ANT_PVT_FAIL == u_status || ORT_FILTER_FAIL == u_status)
  {
    sm_api_OrientFix_Report(&z_attitudeResult);
  }
  if (ORT_SUCC == u_status && NULL != pz_mainAntTdcpMeasBlock && NULL != pz_auxiAntTdcpMeasBlock)
  {
    dcp_api_MainAntTdcpMeasBlk_Put(pz_mainAntTdcpMeasBlock);
    dcp_api_AuxiAntTdcpMeasBlk_Put(pz_auxiAntTdcpMeasBlock);
    dcp_api_orientBackGround_Put(&z_attitudeResult);
  }
  OS_FREE(pz_mainAntTdcpMeasBlock);
  OS_FREE(pz_auxiAntTdcpMeasBlock);
  return;
}
/**
 * @brief ORT IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ort_Proc(ipc_t* p_ipc)
{
  if (FALSE == ort_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_ORT_INIT:
    ort_Init(p_ipc);
    break;
  case C_M_ORT_START:
    ort_Start(p_ipc);
    break;
  case C_M_ORT_STOP:
    ort_Stop(p_ipc);
    break;
  case C_M_ORT_RELEASE:
    ort_Release(p_ipc);
    break;
  case C_M_ORT_RCV_MEAS_PUT:
    ort_RcvMeasBlk_Put(p_ipc);
    break;
  case C_M_ORT_ORT_MEAS_PUT:
    ort_OrtMeasBlk_Put(p_ipc);
    break;
  case C_M_ORT_MAIN_ANT_PVT_INFO:
    ort_mainAntPvtInfo_Put(p_ipc);
    break;
  default:
    break;
  }
}

/**
 * @brief ORT IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ort_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_ORT;

  ipctask_t* t = ipctask_GetInstance(taskIndex);
  t->e_status = TASK_STATUS_ENABLE;
  ipctask_SetThreadId2TaskId(get_thread_id(), taskIndex);
  if (NULL != t->float_hard_enable)
  {
    t->float_hard_enable();
  }
  while (t->e_status == TASK_STATUS_ENABLE)
  {
    if (ipctask_ReceiveMessage(taskIndex, &z_ipc))
    {
      ort_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief ORT API for initilization
 * @return      None
 */
BOOL ort_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief ORT API for start
 * @return      None
 */
BOOL ort_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_START, NULL, 0);
  return TRUE;
}

/**
 * @brief ORT API for stop
 * @return      None
 */
BOOL ort_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief ORT API for release
 * @return      None
 */
BOOL ort_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief ORT API to Put Rcv Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ort_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas)
{
  meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_RCV_MEAS_PUT,
    (uint8_t*)meas, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief ORT API to Put Orient Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ort_api_OrtMeasBlk_Put(GnssMeasBlock_t* meas)
{
  meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_ORT_MEAS_PUT,
    (uint8_t*)meas, sizeof(GnssMeasBlock_t));
  return;
}
/**
 * @brief ORT API to Put PVT information of main antenna from SM or other task
 * @param[in]   pz_pvtInfo - the PVT information of main antenna
 * @return None
 */
void ort_api_mainAntennaPVTinfo_put(gnss_PVTResult_t* pz_pvtInfo)
{
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_MAIN_ANT_PVT_INFO,
    (uint8_t*)pz_pvtInfo, sizeof(gnss_PVTResult_t));
  return;
}
