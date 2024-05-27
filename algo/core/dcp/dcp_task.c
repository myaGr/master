/**@file        dcp_task.cpp
 * @brief       Differential Carrier Phase 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "mw_ipctask.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "dcp_task.h"
#include "dcp_api.h"
#include "hpp_api.h"
#include "ppp_api.h"
#include "dcp_solution.h"
#include "gnss_common.h"
#include "sm_api.h"
#include "dcp_ort_sol.h"

typedef struct {
  uint32_t      q_preSecondAlignMsec;
} dcp_task_ctrl_t;

static dcp_task_ctrl_t gz_dcp_task_ctrl;

dcp_solInfo_t gz_dcpSolInfo = { 0 };

/**
 * @brief DCP module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_Init(ipc_t* p_ipc)
{
  gnss_DcpConfigOpt_t z_dcpOpt = { 0 };
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  DCP_loadDefaultOption(&z_dcpOpt);
  if ((pz_loc_ConfigParamGroup->sm_cfg.f_ele_mask) >= FABS_ZEROS && (pz_loc_ConfigParamGroup->sm_cfg.f_ele_mask) <= 90.0f)
  {
    z_dcpOpt.f_eleCutOff = pz_loc_ConfigParamGroup->sm_cfg.f_ele_mask;
  }
  z_dcpOpt.u_enableFeedbackMeasToINS = 1;
  DCP_init(&z_dcpOpt, &gz_dcpSolInfo);
  if (pz_loc_ConfigParamGroup->ort_cfg.b_enable)
  {
    z_dcpOpt.u_enableFeedbackMeasToINS = 0;
    DCP_ortInit(&z_dcpOpt);
  }
  return;
}

/**
 * @brief DCP module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief DCP module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief DCP module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_Release(ipc_t* p_ipc)
{
  DCP_deinit(&gz_dcpSolInfo);
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  if (pz_loc_ConfigParamGroup->ort_cfg.b_enable)
  {
    DCP_ortDeinit();
  }
  return;
}

/**
 * @brief DCP module put 1Hz or 10Hz measurement
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_RcvMeasBlk_NHz_Put(const ipc_t* p_ipc)
{
  uint8_t u_ortStatus = 0;
  uint8_t u_i = 0;
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  gnss_OrientFix_t z_ortResult = { 0 };
  if (sizeof(GnssMeasBlock_t) != p_ipc->q_length)
  {
    return;
  }

  GnssMeasBlock_t* pz_MeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;
  gnss_TdcpMeasBlock_t* pz_CurentTdcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  if ((NULL == pz_CurentTdcpMeas))
  {
    return;
  }

  if (!gnss_AdjustMeasBlockWeek(pz_MeasBlock->w_measCount, pz_MeasBlock->z_meas, &pz_MeasBlock->z_Clock.z_gpsTime)) // GPS WEEK
  {
    /* report location */
    dcp_CreateEmptyPos(pz_MeasBlock, pz_CurentTdcpMeas, &gz_dcpSolInfo);
    sm_api_PositionFix_Report(&pz_CurentTdcpMeas->z_posSol);
    OS_FREE(pz_CurentTdcpMeas);
    LOGW(TAG_DCP, "Adjust meas GPS week failed %.0f\n", pz_MeasBlock->z_Clock.z_gpsTime.q_towMsec * TIME_MSEC_INV);
    return;
  }

  /* Convrert measurement block to tdcp measurement */
  gnss_cvt_Meas2TdcpMeas(pz_MeasBlock, pz_CurentTdcpMeas);
  /* Dcp position solution */
  uint8_t status = dcp_SolveRealtimePos(pz_CurentTdcpMeas, &gz_dcpSolInfo);
  if (pz_loc_ConfigParamGroup->ort_cfg.b_enable)
  {
    DCP_PushRcvMainMeasBlk_NHz(pz_CurentTdcpMeas);
  }
  if (1 == status)
  {
    sm_api_PositionFix_Report(&pz_CurentTdcpMeas->z_posSol);
    if (NULL != gz_dcpSolInfo.pz_GnssFeedbackInsMeas)
    {
      sm_api_GnssFeedbackInsMeasBlock_Report(gz_dcpSolInfo.pz_GnssFeedbackInsMeas);
    }
  }
  else
  {
    sm_api_PositionFix_Report(&pz_CurentTdcpMeas->z_posSol);
  }
  if (pz_loc_ConfigParamGroup->ort_cfg.b_enable)
  {
    u_ortStatus = DCP_SolveRealtimeOrt(pz_CurentTdcpMeas->z_obsTime, &z_ortResult);
    if (ORT_DCP_SUCC == u_ortStatus)
    {
      sm_api_OrientFix_Report(&z_ortResult);
    }
  }
  LOGI(TAG_DCP, "DCP measurement put %d\n", pz_CurentTdcpMeas->z_obsTime.q_towMsec);
  OS_FREE(pz_CurentTdcpMeas);
  return;
}

/**
 * @brief DCP module put 1Hz or 10Hz orient measurement
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_RcvAuxiMeasBlk_NHz_Put(ipc_t* p_ipc)
{
  uint8_t u_ortStatus = 0;
  gnss_OrientFix_t z_ortResult = { 0 };
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  if (sizeof(GnssMeasBlock_t) != p_ipc->q_length)
  {
    return;
  }
  GnssMeasBlock_t* pz_MeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;
  gnss_TdcpMeasBlock_t* pz_CurentTdcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));

  if ((NULL == pz_CurentTdcpMeas))
  {
    return;
  }
  if (!gnss_AdjustMeasBlockWeek(pz_MeasBlock->w_measCount, pz_MeasBlock->z_meas, &pz_MeasBlock->z_Clock.z_gpsTime)) // GPS WEEK
  {
    OS_FREE(pz_CurentTdcpMeas);
    LOGW(TAG_DCP, "Ort adjust meas GPS week failed %.0f\n", pz_MeasBlock->z_Clock.z_gpsTime.q_towMsec * TIME_MSEC_INV);
    return;
  }
  /* Convrert measurement block to tdcp measurement */
  gnss_cvt_Meas2TdcpMeas(pz_MeasBlock, pz_CurentTdcpMeas);
  if (pz_loc_ConfigParamGroup->ort_cfg.b_enable)
  {
    DCP_PushRcvAuxiMeasBlk_NHz(pz_CurentTdcpMeas);
    u_ortStatus = DCP_SolveRealtimeOrt(pz_CurentTdcpMeas->z_obsTime, &z_ortResult);
    if (ORT_DCP_SUCC == u_ortStatus)
    {
      sm_api_OrientFix_Report(&z_ortResult);
    }
  }
  OS_FREE(pz_CurentTdcpMeas);
  return;
}

/**
 * @brief DCP Process TdcpMeasuremt @1Hz
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_TdcpMeas_Put(const ipc_t* p_ipc)
{
  gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock = (gnss_TdcpMeasBlock_t*)p_ipc->p_data;
  dcp_updateBaseMeasBlock(pz_tdcpMeasBlock, &gz_dcpSolInfo);
  LOGI(TAG_DCP,"DCP measurement base put %d\n", pz_tdcpMeasBlock->z_obsTime.q_towMsec);
  return;
}

/**
 * @brief DCP Process TDCP measuremt for main antenna
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_mainAntTdcpMeas_Put(const ipc_t* p_ipc)
{
  const gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock = (gnss_TdcpMeasBlock_t*)p_ipc->p_data;
  DCP_updateMainAntBackGroundResult(pz_tdcpMeasBlock);
  return;
}

/**
 * @brief DCP Process TDCP measuremt for auxi antenna
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_auxiAntTdcpMeas_Put(const ipc_t* p_ipc)
{
  const gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock = (gnss_TdcpMeasBlock_t*)p_ipc->p_data;
  DCP_updateAuxiAntBackGroundResult(pz_tdcpMeasBlock);
  return;
}

/**
 * @brief DCP Process orient result
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void dcp_orientBackGround_Put(const ipc_t* p_ipc)
{
  const gnss_OrientFix_t* pz_ortResult = (gnss_OrientFix_t*)p_ipc->p_data;
  DCP_updatOrtBackGroundResult(pz_ortResult);
  return;
}

/**
 * @brief Sanity Check IPC 
 * @param[in]   p_ipc - pointer to IPC message
 * @return: true - success
 *          false - fail for any reason
 */
static BOOL dcp_IpcCheck(ipc_t* p_ipc)
{
  if (TASK_INDEX_DCP != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_DCP_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_DCP_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief DCP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void dcp_Proc(ipc_t* p_ipc)
{
  if (FALSE == dcp_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_DCP_INIT:
    dcp_Init(p_ipc);
    break;
  case C_M_DCP_START:
    dcp_Start(p_ipc);
    break;
  case C_M_DCP_STOP:
    dcp_Stop(p_ipc);
    break;
  case C_M_DCP_RELEASE:
    dcp_Release(p_ipc);
    break;
  case C_M_DCP_RCV_MEAS_NHZ_PUT:
    dcp_RcvMeasBlk_NHz_Put(p_ipc);
    break;
  case C_M_DCP_RCV_ORT_MEAS_NHZ_PUT:
    dcp_RcvAuxiMeasBlk_NHz_Put(p_ipc);
    break;
  case C_M_DCP_TDCP_MEAS_PUT:
    dcp_TdcpMeas_Put(p_ipc);
    break;
  case C_M_DCP_MAIN_ANT_DCP_MEAS_PUT:
    dcp_mainAntTdcpMeas_Put(p_ipc);
    break;
  case C_M_DCP_AUXI_ANT_DCP_MEAS_PUT:
    dcp_auxiAntTdcpMeas_Put(p_ipc);
    break;
  case C_M_DCP_ORT_BACK_GROUND_PUT:
    dcp_orientBackGround_Put(p_ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief DCP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* dcp_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_DCP;
  
  ipctask_t* t = ipctask_GetInstance(taskIndex);
  t->e_status = TASK_STATUS_ENABLE;
  ipctask_SetThreadId2TaskId(get_thread_id(), taskIndex);
  if(NULL != t->float_hard_enable){
    t->float_hard_enable();
  }
  while (t->e_status == TASK_STATUS_ENABLE)
  {
    if (ipctask_ReceiveMessage(taskIndex, &z_ipc))
    {
      dcp_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief DCP API for initilization
 * @return      None
 */
BOOL dcp_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief DCP API for start
 * @return      None
 */
BOOL dcp_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_START, NULL, 0);
  return TRUE;
}

/**
 * @brief DCP API for stop
 * @return      None
 */
BOOL dcp_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief DCP API for release
 * @return      None
 */
BOOL dcp_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief HPP API to Put Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void dcp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas)
{
  meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_RCV_MEAS_NHZ_PUT, 
    (uint8_t*)meas, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief DCP API to Put Orient Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void dcp_api_RcvOrtMeasBlk_Put(GnssMeasBlock_t* meas)
{
  meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_RCV_ORT_MEAS_NHZ_PUT,
    (uint8_t*)meas, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief HPP API to Put Tdcp Measurement @1Hz from HPP Task
 * @param[in]   tdcpMeas - Tdcp Measurement @1Hz from HPP Task
 * @return None
 */
void dcp_api_TdcpMeasBlk_Put(gnss_TdcpMeasBlock_t* tdcpMeas)
{
  tdcpMeas->u_version = VERSION_TDCP_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_TDCP_MEAS_PUT, 
    (uint8_t*)tdcpMeas, sizeof(gnss_TdcpMeasBlock_t));
  return;
}

/**
 * @brief DCP API to Put Tdcp Measurement of main antenna from ORT Task
 * @param[in]   pz_tdcpMeas - Tdcp Measurement @1Hz from ORT Task
 * @return None
 */
void dcp_api_MainAntTdcpMeasBlk_Put(gnss_TdcpMeasBlock_t* pz_tdcpMeas)
{
  pz_tdcpMeas->u_version = VERSION_TDCP_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_MAIN_ANT_DCP_MEAS_PUT, (uint8_t*)pz_tdcpMeas, sizeof(gnss_TdcpMeasBlock_t));
  return;
}

/**
 * @brief DCP API to Put Tdcp Measurement of auxi antenna from ORT Task
 * @param[in]   pz_tdcpMeas - Tdcp Measurement @1Hz from ORT Task
 * @return None
 */
void dcp_api_AuxiAntTdcpMeasBlk_Put(gnss_TdcpMeasBlock_t* pz_tdcpMeas)
{
  pz_tdcpMeas->u_version = VERSION_TDCP_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_AUXI_ANT_DCP_MEAS_PUT, (uint8_t*)pz_tdcpMeas, sizeof(gnss_TdcpMeasBlock_t));
  return;
}

/**
 * @brief DCP API to Put orient result from ORT Task
 * @param[in]   pz_attitudeResult - orient result from ORT Task
 * @return None
 */
void dcp_api_orientBackGround_Put(gnss_OrientFix_t* pz_attitudeResult)
{
  pz_attitudeResult->u_version = VERSION_GNSS_ORT_FIX;
  ipctask_SendMessage(TASK_INDEX_DCP, C_M_DCP_ORT_BACK_GROUND_PUT, (uint8_t*)pz_attitudeResult, sizeof(gnss_OrientFix_t));
  return;
}
