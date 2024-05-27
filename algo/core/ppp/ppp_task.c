/**@file        ppp_task.cpp
 * @brief       Precision Point Positioning
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
#include "windup_correction.h"
#include "gnss_common.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "ppp_api.h"
#include "ppp_task.h"
#include "ppp_process.h"
#include "sm_api.h"
#include "dcp_api.h"
#include "loc_core_report.h"
#include "qxsi_ids_ssr2los_interface.h"
#include "service_ssr_cvt.h"
#include "ppp_integrity.h"

static gnss_SatSigMeasCollect_t gz_ppp_SatSigMeasCollect;
static gnss_SatSigMeasCollect_t gz_ppp_SatSigMeasCollect_prev;
static gnss_ssrLosBlock_t*      gpz_ppp_ssrLosBlock;

static const qxsi_ids_ssr2los_interface_t* p_qx_ssr2los_interace = NULL;

/**
 * @brief QianXun Ssr2Los adapter log callback
 * @param[in]   buf
 * @param[in]   len
 * @return      None
 */
static void ppp_QianXun_cb_log(const char* const buf, const int len)
{
  LOGD(TAG_PPP, "QXSSR: %s\n", buf);
  return;
}

/**
 * @brief QianXun Ssr2Los status log callback
 * @param[in]   code
 * @return      None
 */
static void ppp_QianXun_cb_status(qxsi_ids_ssr2los_status_e code)
{
  LOGD(TAG_PPP, "QXSSR status: %d\n", code);
  return;
}

/**
 * @brief Init QianXun Ssr2Los adapter, depands on qxsi_ids_get_ssr2los_interface
 * @param[in]   None
 * @return      return TRUE on success, FALSE on failure
 */
static BOOL ppp_SsrQianXunAdapter_init()
{
  int32_t ret = -1;
  qxsi_ids_ssr2los_config_t config = { 0 };
  qxsi_ids_ssr2los_callbacks_t cbs = { 0 };
  config.enable = 1;
  config.level = QXSI_LOG_LEVEL_DEBUG;

  cbs.log_callback = ppp_QianXun_cb_log;
  cbs.status_callback = ppp_QianXun_cb_status;
#ifdef FEATURE_USE_QXWZ_SSR
  p_qx_ssr2los_interace = qxsi_ids_get_ssr2los_interface();
#endif
  if (NULL == p_qx_ssr2los_interace)
  {
    LOGE(TAG_PPP, "get ssr2los interface fail\n");
    return FALSE;
  }

  ret = p_qx_ssr2los_interace->init(&config, &cbs);
  if (-1 == ret)
  {
    LOGE(TAG_PPP, "init ssr2los interface fail\n");
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief release QianXun Ssr2Los adapter
 * @param[in]   None
 * @return      return 0 on success, -1 on failure
 */
static BOOL ppp_SsrQianXunAdapter_Release()
{
  int32_t ret = -1;
  if (NULL == p_qx_ssr2los_interace)
  {
    return FALSE;
  }

  ret = p_qx_ssr2los_interace->cleanup();
  if (-1 == ret)
  {
    return FALSE;
  }

  p_qx_ssr2los_interace = NULL;
  return TRUE;
}

/**
 * @brief PPP module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_Init(ipc_t* p_ipc)
{
  gpz_ppp_ssrLosBlock = OS_MALLOC(sizeof(gnss_ssrLosBlock_t));

  ppp_SsrQianXunAdapter_init();

  ppp_Algorithm_Init(NULL);

  ppp_IntegInit();

  return;
}

/**
 * @brief PPP module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief PPP module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief PPP module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_Release(ipc_t* p_ipc)
{
  OS_FREE(gpz_ppp_ssrLosBlock);

  ppp_SsrQianXunAdapter_Release();
  ppp_Algorithm_Deinit();
  ppp_IntegDeInit();
  wup_deinit();

  /* destruct sat sig meas */
  gnss_SatSigMeasCollect_Clear(&gz_ppp_SatSigMeasCollect);
  gnss_SatSigMeasCollect_Clear(&gz_ppp_SatSigMeasCollect_prev);

  return;
}


/**
 * @brief Send out the tdcp analyzed satellite and signal measurement collect.
 * @param pz_satSigMeasCollect satellite signal observation information
 * @param pz_PositionFix position
 */
static void
ppp_TdcpMeasBlk_SendOut(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_PositionFix_t* pz_PositionFix)
{
  gnss_TdcpMeasBlock_t* pz_tdcpMeas = OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  if (NULL == pz_tdcpMeas)
  {
    return;
  }
  /* convert pack */
  gnss_cvt_SatSigMeas2TdcpMeas(pz_satSigMeasCollect, pz_PositionFix, pz_tdcpMeas);

  /* tdcp measurement inject */
  dcp_api_TdcpMeasBlk_Put(pz_tdcpMeas);

  OS_FREE(pz_tdcpMeas);
}

static void decimal2binary(uint32_t status, char* c_status)
{
  int8_t i = 0;
  uint8_t u_max = 0;
  uint32_t temp = 0;
  uint32_t sum = 0;

  for (i = 0; i < 32; i++)
  {
    temp = 1 << i;
    if (status & temp)
    {
      sum += temp;
    }
    if (sum >= status)
    {
      u_max = i;
      break;
    }
  }

  for (i = u_max; i >=0; i--)
  {
    c_status[i] = (status & 1) + '0';
    status = status >> 1;
  }
  c_status[u_max + 1] = '\0';
  return;
}

/**
 * Init pz_PositionFix
 * @param[in,out] pz_PositionFix
 */
static void ppp_InitPosfix(gnss_PositionFix_t* pz_PositionFix)
{
  *pz_PositionFix = gz_ppp_SatSigMeasCollect.z_positionFix;
  pz_PositionFix->u_fixSource = FIX_SOURCE_PPP;
  gnss_InitIntegrityStruct(&pz_PositionFix->z_integrity);
}
/**
 * @brief ppp algorithm start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_Algorithm_Start(ipc_t* p_ipc)
{
  char c_algstatus[33] = "";
  uint32_t q_pppSoluteStatus = 0;

  /* skip the same epoch as previous */
  if (fabs(tm_GpsTimeDiff(&gz_ppp_SatSigMeasCollect.z_tor, &gz_ppp_SatSigMeasCollect_prev.z_tor)) < FABS_ZEROS)
  {
    LOGI(TAG_PPP, "Skip the same epoch as previous %d\n", gz_ppp_SatSigMeasCollect.z_tor.q_towMsec);
    return;
  }

  /* reset integrity info */
  if(ppp_IntegValid())
  {
    ppp_IntegClear(&gz_ppp_SatSigMeasCollect.z_tor);
  }

  gnss_PositionFix_t* pz_PositionFix = (gnss_PositionFix_t*)OS_MALLOC_FAST(sizeof(gnss_PositionFix_t));
  ppp_InitPosfix(pz_PositionFix);

  /* PPP */
  uint64_t t_tstart = timepro_get_now();
  q_pppSoluteStatus = ppp_Algorithm_Prcocess(gpz_ppp_ssrLosBlock, &gz_ppp_SatSigMeasCollect, pz_PositionFix);
  LOGW(TAG_PPP, "Time cost ppp_Algorithm_Prcocess %d, %llu ms\n", gz_ppp_SatSigMeasCollect.z_tor.q_towMsec, time_cost(t_tstart));

  /* Report location */
  sm_api_PositionFix_Report(pz_PositionFix);

  /* Sendout to TDCP */
  ppp_TdcpMeasBlk_SendOut(&gz_ppp_SatSigMeasCollect, pz_PositionFix);

  OS_FREE(pz_PositionFix);

  decimal2binary(q_pppSoluteStatus, c_algstatus);
  LOGW(TAG_PPP, "ppp_Algorithm_Start Finish tow %d, status= %d, bina=%s\n", gz_ppp_SatSigMeasCollect.z_tor.q_towMsec,
       q_pppSoluteStatus, c_algstatus);

  return;
}

/**
 * @brief PPP Receiver measurement put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_RcvMeas_Put(ipc_t* p_ipc)
{
  GnssMeasBlock_t* pz_MeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;
  // TODO: Inject Measurement to PPP Engine
  return;
}

/**
 * @brief PPP Put SSR Loc Block Data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_SsrLocBlock_Put(ipc_t* p_ipc)
{
  gnss_ssrLosBlock_t* pz_SsrLocBlk = (gnss_ssrLosBlock_t*)p_ipc->p_data;

  *gpz_ppp_ssrLosBlock = *pz_SsrLocBlk;

  LOGI(TAG_PPP, "Ssr loc bloc put %d\n", pz_SsrLocBlk->z_epochLosInfo.z_tor.q_towMsec);

  return;
}

/**
 * @brief Put tight satellite signal measurement collect and convert it
          as a sparse satellite signal measurement collect stored in PPP
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_TightSatSigMeasCollect_Put(ipc_t* p_ipc)
{
  /* get tight sat sig meas */
  gnss_TightSatSigMeasCollect_t* pz_TightSatSigMeasCollect = (gnss_TightSatSigMeasCollect_t*)p_ipc->p_data;

  LOGW(TAG_PPP, "ppp put sat signal meas count %d\n",
   pz_TightSatSigMeasCollect->u_satMeasCount);

  /* unpack tight sats sig meas */
  gnss_SatSigMeasCollect_Unpack(pz_TightSatSigMeasCollect, &gz_ppp_SatSigMeasCollect);

  /* ST 20ms jump fix */
  gnss_SatSigMeasTimeJumpST(&gz_ppp_SatSigMeasCollect);

  qxsi_station_los_t* p_qxsi_los = NULL;
  gnss_ssrLosBlock_t* p_SsrLocBlk = NULL;

  /* Align SSR LOS message generate at 1Hz */
  static uint64_t q_preSecondAlignMsec = 0;
  if (gnss_CheckMeasSecondOverrun(pz_TightSatSigMeasCollect->z_tor.t_fullMsec,
          &q_preSecondAlignMsec, 1.0f))
  {
      if ((GNSS_FIX_FLAG_INVALID != gz_ppp_SatSigMeasCollect.z_positionFix.u_fixFlag) &&
          (NULL != p_qx_ssr2los_interace))
      {
          qxsi_station_info_t* p_qxsi_station_info = (qxsi_station_info_t*)OS_MALLOC_FAST(sizeof(qxsi_station_info_t));
          p_qxsi_los = (qxsi_station_los_t*)OS_MALLOC_FAST(sizeof(qxsi_station_los_t));
          p_SsrLocBlk = (gnss_ssrLosBlock_t*)OS_MALLOC_FAST(sizeof(gnss_ssrLosBlock_t));
          cvt_SatSigMeasCollect_Ag2Qxwz(&gz_ppp_SatSigMeasCollect.z_positionFix, &gz_ppp_SatSigMeasCollect, p_qxsi_station_info);

          uint64_t t_tstart = timepro_get_now();
          p_qx_ssr2los_interace->pull_los_data_by_obs(p_qxsi_station_info, p_qxsi_los);
          LOGW(TAG_PPP, "Time cost pull_los_data_by_obs %d, %llu ms\n",  gz_ppp_SatSigMeasCollect.z_tor.q_towMsec, time_cost(t_tstart));
          OS_FREE(p_qxsi_station_info);

          /* los qsi to ag */
          ssr_cvt_SsrLosBlk_Qxwz2Ag((const qxsi_station_los_t*)p_qxsi_los, p_SsrLocBlk);
          /* put */
          ppp_api_SsrLocBlock_Put(p_SsrLocBlk);
      }
  }

  OS_FREE(p_qxsi_los);
  OS_FREE(p_SsrLocBlk);

  ppp_api_Algorithm_Start();

  return;
}

/**
 * @brief Session Manager Put SSR Stream Data from QianXun
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_SsrQianXunStream_Put(ipc_t* p_ipc)
{
  int ret = 0;
  const uint8_t* const buffer = p_ipc->p_data;
  const uint32_t length = p_ipc->q_length;

  if (p_qx_ssr2los_interace != NULL) {
    ret = p_qx_ssr2los_interace->push_ssr(buffer, length);
    if (0 == ret)
    {
      LOGW(TAG_PPP, "SSR push string length %d succ\n", length);
    }
    else
    {
      LOGE(TAG_PPP, "SSR push string length %d fail\n", length);
    }
  }

  return;
}

/**
 * @brief Session Manager Put SSR Data by AG los Structure
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ppp_SsrAgLos_Put(ipc_t* p_ipc)
{
#if 0
  const uint8_t* const buffer = p_ipc->p_data;
  const uint32_t length = p_ipc->q_length;

  gnss_ssrLosBlock_t* pz_SsrLocBlk = (gnss_ssrLosBlock_t*)OS_MALLOC(sizeof(gnss_ssrLosBlock_t));
  if (NULL == pz_SsrLocBlk)
  {
    LOGE(TAG_PPP, "gnss_ssrLosBlock_t Alloc fail\n");
    return;
  }

  BOOL ret = loc_report_cvt_ParseBufferToSsrLosBlock(pz_SsrLocBlk, (uint8_t*)buffer, length);
  if (FALSE == ret)
  {
	 OS_FREE(pz_SsrLocBlk);
    LOGE(TAG_PPP, "SSR AG Los put fail\n");
    return;
  }

  LOGI(TAG_PPP, "SSR AG Los put Success, week %d tow %d\n",
    pz_SsrLocBlk->z_epochLosInfo.z_tor.w_week,
    pz_SsrLocBlk->z_epochLosInfo.z_tor.q_towMsec);

  memcpy(gpz_ppp_ssrLosBlock, pz_SsrLocBlk, sizeof(gnss_ssrLosBlock_t));
  OS_FREE(pz_SsrLocBlk);
#endif
  return;
}

/**
 * @brief Sanity Check IPC
 * @param[in]   p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL ppp_IpcCheck(ipc_t* p_ipc)
{
  if (TASK_INDEX_PPP != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_PPP_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_PPP_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief PPP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_Proc(ipc_t* p_ipc)
{
  if (FALSE == ppp_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_PPP_INIT:
    ppp_Init(p_ipc);
    break;
  case C_M_PPP_START:
    ppp_Start(p_ipc);
    break;
  case C_M_PPP_STOP:
    ppp_Stop(p_ipc);
    break;
  case C_M_PPP_RELEASE:
    ppp_Release(p_ipc);
    break;
  case C_M_PPP_RCV_MEAS_1HZ_PUT:
    ppp_RcvMeas_Put(p_ipc);
    break;
  case C_M_PPP_SSR_LOS_BLK:
    ppp_SsrLocBlock_Put(p_ipc);
    break;
  case C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT:
    ppp_TightSatSigMeasCollect_Put(p_ipc);
    break;
  case C_M_PPP_ALGO_START:
    ppp_Algorithm_Start(p_ipc);
    break;
  case C_M_PPP_SSR_STREAM_QX:
    ppp_SsrQianXunStream_Put(p_ipc);
    break;
  case C_M_PPP_SSR_STREAM_AG_LOS:
    ppp_SsrAgLos_Put(p_ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief Get current in use Satellite Signal Measurement Collect
 * @return The pointer to current structure
 */
gnss_SatSigMeasCollect_t* ppp_GetCurrentSatSigMeasCollect()
{
  return &gz_ppp_SatSigMeasCollect;
}

/**
 * @brief Get previous in use Satellite Signal Measurement Collect
 * @return The pointer to previous structure
 */
gnss_SatSigMeasCollect_t* ppp_GetPreviousSatSigMeasCollect()
{
  return &gz_ppp_SatSigMeasCollect_prev;
}

/**
 * @brief PPP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ppp_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_PPP;

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
      ppp_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief PPP API for initilization
 * @return      None
 */
BOOL ppp_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief PPP API for start
 * @return      None
 */
BOOL ppp_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_START, NULL, 0);
  return TRUE;
}

/**
 * @brief PPP API for stop
 * @return      None
 */
BOOL ppp_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief PPP API for release
 * @return      None
 */
BOOL ppp_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief PPP API to Put Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ppp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas)
{
  meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_RCV_MEAS_1HZ_PUT,
    (uint8_t*)meas, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief PPP API to Put SSR Loc Block Data
 * @param[in]   pz_ssrLocBlk - SSR Loc Block Data
 * @return      None
 */
void ppp_api_SsrLocBlock_Put(gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  pz_SsrLocBlk->u_version = VERSION_GNSS_SSR_LOS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_SSR_LOS_BLK,
    (uint8_t*)pz_SsrLocBlk, sizeof(gnss_ssrLosBlock_t));
  return;
}

/**
 * @brief PPP API to Put tight satellite signal measurement collect
          and convert it as a sparse satellite signal measurement
          collect stored in PPP
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_TightSatSigMeasCollect_Put(gnss_TightSatSigMeasCollect_t* pz_TightSatSigMeasCollect)
{
  pz_TightSatSigMeasCollect->u_version = VERSION_GNSS_TIGHT_SAT_SIG_MEAS_COLLECT;
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT,
    (uint8_t*)pz_TightSatSigMeasCollect, sizeof(gnss_TightSatSigMeasCollect_t));
  return;
}

/**
 * @brief PPP API to start run algorithm
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_Algorithm_Start()
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_ALGO_START, NULL, 0);
  return;
}

/**
 * @brief PPP API to inject Qianxun SSR stream
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_SsrQianXunStream_Put(uint8_t* data, uint32_t len)
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_SSR_STREAM_QX,
    (uint8_t*)data, len);
  return;
}

/**
 * @brief PPP API to inject AG SSR Los data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_SsrAgLos_Put(uint8_t* data, uint32_t len)
{
  ipctask_SendMessage(TASK_INDEX_PPP, C_M_PPP_SSR_STREAM_AG_LOS,
    (uint8_t*)data, len);
  return;
}
