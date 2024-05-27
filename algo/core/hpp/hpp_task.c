/**@file        hpp_task.c
 * @brief       High Precision Position(HPP) message process task file
 * @details     HPP task handle data injection from other module(SM, SD, etc.)
 * @author      caizhijie
 * @date        2022/04/26
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/26  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "cmn_utils.h"
#include "gnss_common.h"
#include "mw_alloc.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "hpp_task.h"
#include "hpp_api.h"
#include "sm_api.h"
#include "sd_api.h"
#include "dcp_api.h"
#include "ppp_api.h"
#include "loc_core_api.h"
#include "gnss_engine_api.h"
#include "rtcm_dec.h"
#include "gnss_prep.h"
#include "pvt_proc.h"
#include "rtk_proc.h"
#include "loc_core_report.h"
#include "rtk_integrity.h"
#include "ort_api.h"


typedef struct {
  uint8_t        u_isInit;
  hpp_ConfigParam_t* pz_hppConfigParam;
  RtcmDecoder_t* pz_hppRcvRtcmDecoder;
  RtcmDecoder_t* pz_hppRefRtcmDecoder;
  gnss_PVTResult_t z_PvtSolution;
} hpp_task_ctrl_t;
static hpp_task_ctrl_t gz_hpp_task_ctrl;

static gnss_SatSigMeasCollect_t gz_SatSigMeasCollect = { 0 };
static gnss_SatSigMeasCollect_t gz_SatSigMeasCollectPre = { 0 };

static void gnss_engine_callback_report_location(gnss_NavSolution_t* location_info)
{
  sm_api_PositionFix_Report(&location_info->z_positionFix);
}

typedef enum
{
  LOG_NONE = 0,
  LOG_DEBUG,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
} Log_Level_t;

static void gnss_engine_callback_report_log(int level, char* log, int length)
{
  if (any_Ptrs_Null(1, log) || (0 == length))
  {
    return;
  }

  if (LOG_DEBUG == level)
  {
    LOGD(TAG_HPP, "%s\n", log);
  }
  else if (LOG_INFO == level)
  {
    LOGI(TAG_HPP, "%s\n", log);
  }
  else if (LOG_WARNING == level)
  {
    LOGW(TAG_HPP, "%s\n", log);
  }
  else if (LOG_ERROR == level)
  {
    LOGE(TAG_HPP, "%s\n", log);
  }
}

/**
 * @brief HPP module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_Init(ipc_t* p_ipc)
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParam = loc_cfg_getConfigParamGroup();
  rtk_alg_opt_t z_opt;
  if (NULL == pz_loc_ConfigParam)
  {
    return;
  }

  if (TRUE == gz_hpp_task_ctrl.u_isInit)
  {
    return;
  }

  gz_hpp_task_ctrl.pz_hppConfigParam = &(pz_loc_ConfigParam->hpp_cfg);

  GnssEngineCallback_t z_callback = { 0 };
  z_callback.report_location = gnss_engine_callback_report_location;
  z_callback.report_log = gnss_engine_callback_report_log;
  gnss_engine_set_callback(&z_callback);
  gnss_engine_init(pz_loc_ConfigParam->sm_cfg.f_ele_mask);

  /* Receiver raw rtcm stream decoder enable */
  if (gz_hpp_task_ctrl.pz_hppConfigParam->b_enable_rcv_rtcm_dec)
  {
    gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder = (RtcmDecoder_t*)OS_MALLOC(sizeof(RtcmDecoder_t));
    memset(gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder, 0, sizeof(RtcmDecoder_t));
  }
  else
  {
    gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder = NULL;
  }

  /* Reference raw correction rtcm stream decoder enable */
  if (gz_hpp_task_ctrl.pz_hppConfigParam->b_enable_ref_rtcm_dec)
  {
    gz_hpp_task_ctrl.pz_hppRefRtcmDecoder = (RtcmDecoder_t*)OS_MALLOC(sizeof(RtcmDecoder_t));
    memset(gz_hpp_task_ctrl.pz_hppRefRtcmDecoder, 0, sizeof(RtcmDecoder_t));
  }
  else
  {
    gz_hpp_task_ctrl.pz_hppRefRtcmDecoder = NULL;
  }
  
  pvt_init();
  
  if (gz_hpp_task_ctrl.pz_hppConfigParam->b_enable_rtk)
  {
    RTK_loadDefaultOption(&z_opt);
    if(0 != pz_loc_ConfigParam->hpp_cfg.f_max_age)
    {
        z_opt.d_maxDiffAge = (double)pz_loc_ConfigParam->hpp_cfg.f_max_age;
    }
    if (FABS_ZEROS < pz_loc_ConfigParam->hpp_cfg.f_max_prefix_age)
    {
      if (30.0 > pz_loc_ConfigParam->hpp_cfg.f_max_prefix_age)
      {
        z_opt.d_maxPrefixAge = 30.0;
      }
      else if (600.1 < pz_loc_ConfigParam->hpp_cfg.f_max_prefix_age)
      {
        z_opt.d_maxPrefixAge = 600.0;
      }
      else
      {
        z_opt.d_maxPrefixAge = (double)pz_loc_ConfigParam->hpp_cfg.f_max_prefix_age;
      }
    }
    else
    {
      z_opt.d_maxPrefixAge = 60.0;
    }
    if ((pz_loc_ConfigParam->sm_cfg.f_ele_mask)>= FABS_ZEROS && (pz_loc_ConfigParam->sm_cfg.f_ele_mask)<=90.0f)
    {
      z_opt.d_elmin = (double)(pz_loc_ConfigParam->sm_cfg.f_ele_mask * DEG2RAD);
    }
    rtk_init(&z_opt, pz_loc_ConfigParam->sm_cfg.f_meas_update_interval);
  }

  gz_hpp_task_ctrl.u_isInit = TRUE;

  return;
}

/**
 * @brief HPP module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief HPP module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief HPP module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_Release(ipc_t* p_ipc)
{
  if (FALSE == gz_hpp_task_ctrl.u_isInit)
  {
    return;
  }

  if (gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder)
  {
    OS_FREE(gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder);
    gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder = NULL;
  }

  if (gz_hpp_task_ctrl.pz_hppRefRtcmDecoder)
  {
    OS_FREE(gz_hpp_task_ctrl.pz_hppRefRtcmDecoder);
    gz_hpp_task_ctrl.pz_hppRefRtcmDecoder = NULL;
  }

  gnss_engine_release();
  pvt_deinit();
  rtk_deInit();
  gnss_SatSigMeasCollect_Clear(&gz_SatSigMeasCollect);
  gnss_SatSigMeasCollect_Clear(&gz_SatSigMeasCollectPre);

  gz_hpp_task_ctrl.u_isInit = FALSE;

  return;
}

/**
 * @brief Send out the hpp analyzed satellite and signal measurement collect.
          1. satellite position/velocity/clock
          2. raw measurement PR/DR/CP
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void hpp_SatSigMeasCollect_Sendout(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, const gnss_PVTResult_t* pz_pvt_result)
{
  gnss_TightSatSigMeasCollect_t* pz_TightSatSigMeasCollect = (gnss_TightSatSigMeasCollect_t*) OS_MALLOC(sizeof(gnss_TightSatSigMeasCollect_t));

  if (NULL == pz_TightSatSigMeasCollect)
  {
    return;
  }

  gnss_SatSigMeasCollect_Pack(pz_SatSigMeasCollect, pz_pvt_result, pz_TightSatSigMeasCollect);

  ppp_api_TightSatSigMeasCollect_Put(pz_TightSatSigMeasCollect);

  OS_FREE(pz_TightSatSigMeasCollect);
}
/**
 * @brief Send out the tdcp analyzed satellite and signal measurement collect.
 * @param pz_satSigMeasCollect satellite signal observation information
 * @param pz_PositionFix position
 */
static void hpp_TdcpMeasBlkSendOut(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_PositionFix_t* pz_PositionFix)
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
/**
 * @brief get the index in gnss_MeasQos_t array
 * @param[in]     u_constellation  - constellation
 * @param[in]     u_prn            - PRN
 * @param[in]     u_signal         - signal
 * @param[in]     pz_measQOS       - gnss_MeasQos_t struct
 * @param[out]    u_measNum        - the rnumber of gnss_MeasQos_t
 * @return        the index in gnss_MeasQos_t array
 */
static uint8_t hpp_getIndexOfMeasQos(uint8_t u_constellation, uint8_t u_prn, uint8_t u_signal, const gnss_MeasQos_t* pz_measQOS, uint8_t u_measNum)
{
  uint8_t u_index = u_measNum;
  uint8_t u_i = 0;
  for (u_i = 0; u_i < u_measNum; ++u_i)
  {
    if ((pz_measQOS[u_i].u_constellation == u_constellation) && (pz_measQOS[u_i].u_svid == u_prn) && (pz_measQOS[u_i].u_signal == u_signal))
    {
      u_index = u_i;
      break;
    }
  }
  return u_index;
}
/**
 * @brief gain the receiver clock initial value of the pseudo-range observations used the SPP result
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys  - the target constellation
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pd_siteCoor  - site coordinate
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[out]    pd_rcvInitClk - the receiver clock initial value of the pseudo-range observations used the SPP result
 * @return        TRUE represent success
 */
static BOOL hpp_gainInitRcvClkPseudoRangeNotUsedByPvt(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const double pd_siteCoor[3], 
  const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, double* pd_rcvInitClk)
{
  BOOL q_isSuccess = FALSE;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_satNum = 0;
  uint8_t u_status = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_roverDist = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const double* pd_satPosClk = NULL;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_clkSet[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  *pd_rcvInitClk = 0.0;
  for (u_satNum = 0; u_satNum < MAX_GNSS_ACTIVE_SAT_NUMBER; ++u_satNum)
  {
    pd_clkSet[u_satNum] = 0.0;
  }
  u_satNum = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_SatSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetSys != u_constellation)
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    if (fabs(pd_satPosClk[0]) < FABS_ZEROS || fabs(pd_satPosClk[1]) < FABS_ZEROS
      || fabs(pd_satPosClk[2]) < FABS_ZEROS || fabs(pd_satPosClk[3]) < FABS_ZEROS)
    {
      continue;
    }
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (u_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || (pz_sigMeas->d_pseudoRange) < FABS_ZEROS)
      {
        continue;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqType != u_targetFreq)
      {
        continue;
      }
      pd_clkSet[u_satNum] = (pz_sigMeas->d_pseudoRange) - (d_roverDist - pd_satPosClk[3]);
      ++u_satNum;
    }
  }
  u_status = gnss_ascSortMedianDouble(pd_clkSet, u_satNum, pd_rcvInitClk);
  if (1 == u_status)
  {
    q_isSuccess = TRUE;
  }
  return q_isSuccess;
}
/**
 * @brief gain the receiver drift initial value of the doppler observations used the SPP result
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys  - the target constellation
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pd_sitePosVel  - site position and velocity,0~2 represent position and 3~5 represent velocity
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[out]    pf_rcvInitDrift - the receiver drift initial value of the doppler observations used the SPP result
 * @return        TRUE represent success
 */
static BOOL hpp_gainInitRcvClkDriftDopplerNotUsedByPvt(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const double pd_sitePosVel[6], const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, float* pf_rcvInitDrift)
{
  BOOL q_isSuccess = FALSE;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_satNum = 0;
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_roverDist = 0.0;
  double d_calculatedValue = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_satVelClkDrift = NULL;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  float pf_driftSet[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  *pf_rcvInitDrift = 0.0;
  for (u_satNum = 0; u_satNum < MAX_GNSS_ACTIVE_SAT_NUMBER; ++u_satNum)
  {
    pf_driftSet[u_satNum] = 0.0;
  }
  u_satNum = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_SatSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetSys != u_constellation)
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    if (fabs(pd_satPosClk[0]) < FABS_ZEROS || fabs(pd_satPosClk[1]) < FABS_ZEROS
      || fabs(pd_satPosClk[2]) < FABS_ZEROS || fabs(pd_satPosClk[3]) < FABS_ZEROS)
    {
      continue;
    }
    pd_satVelClkDrift = pz_satMeas->z_satPosVelClk.d_satVelClk;
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    gnss_satRot(pd_sitePosVel, pd_satPosClk, pd_satPosRot);
    /*d_roverDist =*/ gnss_unitVector(pd_sitePosVel, pd_satPosRot, pd_unitVector);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (u_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || fabs(pz_sigMeas->d_doppler) < FABS_ZEROS)
      {
        continue;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqType != u_targetFreq)
      {
        continue;
      }
      d_calculatedValue = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        d_calculatedValue += (-pd_unitVector[u_i]) * (pd_sitePosVel[3 + u_i] - pd_satVelClkDrift[u_i]);
      }
      d_calculatedValue -= pd_satVelClkDrift[3];
      pf_driftSet[u_satNum] = (float)((pz_sigMeas->d_doppler) - d_calculatedValue);
      ++u_satNum;
    }
  }
  u_status = gnss_ascSortMedianFloat(pf_driftSet, u_satNum, pf_rcvInitDrift);
  if (1 == u_status)
  {
    q_isSuccess = TRUE;
  }
  return q_isSuccess;
}
/**
 * @brief mask the pseudo-range observations used the SPP result
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys - the target constellation
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[in]     d_rcvInitClk - receiver clock initial value
 * @param[in/out] pz_pvtResult - pvt reslut
 * @return      None
 */
static void hpp_maskPseudoRangeNotUsedByPvt(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect,
  double d_rcvInitClk, gnss_PVTResult_t* pz_pvtResult)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_status = 0;
  uint8_t u_measQosIndex = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_roverDist = 0.0;
  double d_omcAbs = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_siteCoor = pz_pvtResult->z_positionFix.d_xyz;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_SatSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetSys != u_constellation)
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    if (fabs(pd_satPosClk[0]) < FABS_ZEROS || fabs(pd_satPosClk[1]) < FABS_ZEROS
      || fabs(pd_satPosClk[2]) < FABS_ZEROS || fabs(pd_satPosClk[3]) < FABS_ZEROS)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqType != u_targetFreq)
      {
        continue;
      }
      u_measQosIndex = hpp_getIndexOfMeasQos(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_pvtResult->pz_meas_qos, pz_pvtResult->u_meas_count);
      if (u_measQosIndex >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      if (u_measQosIndex == (pz_pvtResult->u_meas_count))
      {
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_constellation = pz_sigMeas->u_constellation;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_svid = pz_sigMeas->u_svid;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_signal = pz_sigMeas->u_signal;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_freq_idx = u_signalIndex;
        if (fabs(pz_sigMeas->d_carrierPhase) > FABS_ZEROS)
        {
          pz_pvtResult->pz_meas_qos[u_measQosIndex].u_status |= 0x8;
        }
        ++(pz_pvtResult->u_meas_count);
      }
      gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
      d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
      d_omcAbs = fabs((pz_sigMeas->d_pseudoRange) - (d_roverDist - pd_satPosClk[3]) - d_rcvInitClk);
      pz_pvtResult->pz_meas_qos[u_measQosIndex].z_tor = pz_SatSigMeasCollect->z_tor;
      pz_pvtResult->pz_meas_qos[u_measQosIndex].f_pr_residual_post = (float)d_omcAbs;
      if (d_omcAbs < 25.0)
      {
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_status |= 0x1;
      }
    }
  }
  return;
}
/**
 * @brief mask the doppler observations used the SPP result
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys - the target constellation
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[in]     f_rcvInitClkDrift - receiver clock drift initial value
 * @param[in/out] pz_pvtResult - pvt reslut
 * @return      None
 */
static void hpp_maskDopplerNotUsedByPvt(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, float f_rcvInitClkDrift, gnss_PVTResult_t* pz_pvtResult)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_status = 0;
  uint8_t u_measQosIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_roverDist = 0.0;
  float f_omcAbs = 0.0;
  double d_calculatedValue = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_satVelClkDrift = NULL;
  const double* pd_siteCoor = pz_pvtResult->z_positionFix.d_xyz;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_SatSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetSys != u_constellation)
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    if (fabs(pd_satPosClk[0]) < FABS_ZEROS || fabs(pd_satPosClk[1]) < FABS_ZEROS
      || fabs(pd_satPosClk[2]) < FABS_ZEROS || fabs(pd_satPosClk[3]) < FABS_ZEROS)
    {
      continue;
    }
    pd_satVelClkDrift = pz_satMeas->z_satPosVelClk.d_satVelClk;
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqType != u_targetFreq)
      {
        continue;
      }
      u_measQosIndex = hpp_getIndexOfMeasQos(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_pvtResult->pz_meas_qos, pz_pvtResult->u_meas_count);
      if (u_measQosIndex >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      if (u_measQosIndex == (pz_pvtResult->u_meas_count))
      {
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_constellation = pz_sigMeas->u_constellation;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_svid = pz_sigMeas->u_svid;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_signal = pz_sigMeas->u_signal;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_freq_idx = u_signalIndex;
        if (fabs(pz_sigMeas->d_carrierPhase) > FABS_ZEROS)
        {
          pz_pvtResult->pz_meas_qos[u_measQosIndex].u_status |= 0x8;
        }
        ++(pz_pvtResult->u_meas_count);
      }
      gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
      /*d_roverDist =*/ gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
      d_calculatedValue = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        d_calculatedValue += (-pd_unitVector[u_i]) * (pz_pvtResult->z_positionFix.f_velXyz[u_i] - pd_satVelClkDrift[u_i]);
      }
      d_calculatedValue -= pd_satVelClkDrift[3];
      f_omcAbs = (float)fabs((pz_sigMeas->d_doppler) - d_calculatedValue - f_rcvInitClkDrift);
      pz_pvtResult->pz_meas_qos[u_measQosIndex].z_tor = pz_SatSigMeasCollect->z_tor;
      pz_pvtResult->pz_meas_qos[u_measQosIndex].f_dr_residual_post = f_omcAbs;
      if (f_omcAbs < 1.0)
      {
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_status |= 0x2;
      }
    }
  }
  return;
}
/**
 * @brief deal the pseudo-range observations used the SPP result
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys - the target constellation
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[in/out] pz_pvtResult - pvt reslut
 * @return      None
 */
static void hpp_dealPseudoRangeNotUsedByPvt(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_PVTResult_t* pz_pvtResult)
{
  double d_rcvInitClk = 0.0;
  if (TRUE == hpp_gainInitRcvClkPseudoRangeNotUsedByPvt(q_isSperateBDS2And3, u_targetSys, u_targetFreq, pz_pvtResult->z_positionFix.d_xyz, pz_SatSigMeasCollect, &d_rcvInitClk))
  {
    hpp_maskPseudoRangeNotUsedByPvt(q_isSperateBDS2And3, u_targetSys, u_targetFreq, pz_SatSigMeasCollect, d_rcvInitClk, pz_pvtResult);
  }
  return;
}
/**
 * @brief deal the doppler observations used the SPP result
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys - the target constellation
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[in/out] pz_pvtResult - pvt reslut
 * @return      None
 */
static void hpp_dealDopplerNotUsedByPvt(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_PVTResult_t* pz_pvtResult)
{
  uint8_t u_i = 0;
  float f_rcvInitDrift = 0.0;
  double pd_sitePosVel[6] = { 0.0 };
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_sitePosVel[u_i] = pz_pvtResult->z_positionFix.d_xyz[u_i];
    pd_sitePosVel[3 + u_i] = pz_pvtResult->z_positionFix.f_velXyz[u_i];
  }
  if (TRUE == hpp_gainInitRcvClkDriftDopplerNotUsedByPvt(q_isSperateBDS2And3, u_targetSys, u_targetFreq, pd_sitePosVel, pz_SatSigMeasCollect, &f_rcvInitDrift))
  {
    hpp_maskDopplerNotUsedByPvt(q_isSperateBDS2And3, u_targetSys, u_targetFreq, pz_SatSigMeasCollect, f_rcvInitDrift, pz_pvtResult);
  }
  return;
}
/**
 * @brief deal the carrier-phase observations not be push to QOS struct
 * @param[in]     u_targetFreq - the target frequency
 * @param[in]     pz_SatSigMeasCollect - satellite signal measurement collect
 * @param[out]    pz_pvtResult - pvt reslut
 * @return      None
 */
static void hpp_dealCarrierNotIncludeQos(gnss_FreqType u_targetFreq, const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_PVTResult_t* pz_pvtResult)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_measQosIndex = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_SatSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqType != u_targetFreq)
      {
        continue;
      }
      u_measQosIndex = hpp_getIndexOfMeasQos(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_pvtResult->pz_meas_qos, pz_pvtResult->u_meas_count);
      if (u_measQosIndex >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      if (u_measQosIndex == (pz_pvtResult->u_meas_count))
      {
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_constellation = pz_sigMeas->u_constellation;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_svid = pz_sigMeas->u_svid;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_signal = pz_sigMeas->u_signal;
        pz_pvtResult->pz_meas_qos[u_measQosIndex].u_freq_idx = u_signalIndex;
        if (fabs(pz_sigMeas->d_carrierPhase) > FABS_ZEROS)
        {
          pz_pvtResult->pz_meas_qos[u_measQosIndex].u_status |= 0x8;
        }
        ++(pz_pvtResult->u_meas_count);
      }
    }
  }
  return;
}

/**
 * @brief report meas and satellite block collect
 * @param[in] pz_MeasBlock
 * @param[in] pz_SatSigMeasCollect
 */

static void hpp_makeAndReportMeasSatBlockCollect(const GnssMeasBlock_t *pz_MeasBlock,
                                                 const gnss_SatSigMeasCollect_t *pz_SatSigMeasCollect)
{
  static uint64_t q_preSecondAlignMsec = 0;

  /* Interval is 1s */
  if (gnss_CheckMeasSecondAlign(pz_MeasBlock->z_Clock.z_gpsTime.t_fullMsec,
                                &q_preSecondAlignMsec, 1.0))
  {
    GnssMeasSatBlockCollect_t *pz_measSatColl = (GnssMeasSatBlockCollect_t *)OS_MALLOC(sizeof(GnssMeasSatBlockCollect_t));
    if (NULL != pz_measSatColl)
    {
      /* Meas block */
      memcpy(&pz_measSatColl->z_measBlock, pz_MeasBlock, sizeof(GnssMeasBlock_t));
      pz_measSatColl->d_lla[0] = pz_SatSigMeasCollect->z_positionFix.d_lla[0];
      pz_measSatColl->d_lla[1] = pz_SatSigMeasCollect->z_positionFix.d_lla[1];
      pz_measSatColl->d_lla[2] = pz_SatSigMeasCollect->z_positionFix.d_lla[2];

      /* Satellite block */
      for (uint8_t i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++)
      {
        gnss_SatelliteMeas_t* pz_satMeas = pz_SatSigMeasCollect->pz_satMeas[i];
        if(pz_satMeas != NULL && pz_satMeas->z_satPosVelClk.u_valid)
        {
          memcpy(&pz_measSatColl->z_measBlock.z_satPosVelClk[pz_measSatColl->z_measBlock.u_satCount],
           &pz_satMeas->z_satPosVelClk, sizeof(gnss_SatPosVelClk_t));
          ++pz_measSatColl->z_measBlock.u_satCount;
        }
        if(pz_measSatColl->z_measBlock.u_satCount >= MAX_GNSS_ACTIVE_SAT_NUMBER)
        {
          break;
        }
      }

      /* Report */
      sm_api_GnssMeasSatBlockCollect_Report(pz_measSatColl);

      OS_FREE(pz_measSatColl);
    }
  }
}

/**
 * @brief HPP measurement block process
 * @param[in]   pz_MeasBlock - measurement block
 * @return      None
 */
static void hpp_MeasBlock_Proc(GnssMeasBlock_t* pz_MeasBlock)
{
  uint8_t u_i = 0;
  BOOL q_isMultiFreqMeas = FALSE;
  BOOL q_isSperateBDS2And3 = FALSE;

  gnss_AppendMeasBlkToSatSigMeasCollect(pz_MeasBlock, &gz_SatSigMeasCollect);

  gnss_ComputeSatPosVelClk(&gz_SatSigMeasCollect);

//  hpp_Pvt_Process(&gz_SatSigMeasCollect);

  /* inject pvt solution */
  memset(gz_hpp_task_ctrl.z_PvtSolution.pz_meas_qos, 0, sizeof(gz_hpp_task_ctrl.z_PvtSolution.pz_meas_qos));
  gnss_engine_inject_pvt_solution(&gz_SatSigMeasCollect, &gz_hpp_task_ctrl.z_PvtSolution);

  q_isMultiFreqMeas = gnss_IdentifyMultiFreq(pz_MeasBlock->z_meas, pz_MeasBlock->w_measCount);
  if (TRUE == q_isMultiFreqMeas)
  {
    q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(C_GNSS_FREQ_TYPE_L5, &gz_SatSigMeasCollect);
    hpp_dealCarrierNotIncludeQos(C_GNSS_FREQ_TYPE_L5, &gz_SatSigMeasCollect, &gz_hpp_task_ctrl.z_PvtSolution);
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(q_isSperateBDS2And3, u_i))
      {
        continue;
      }
      hpp_dealPseudoRangeNotUsedByPvt(q_isSperateBDS2And3, u_i, C_GNSS_FREQ_TYPE_L5, &gz_SatSigMeasCollect, &gz_hpp_task_ctrl.z_PvtSolution);
      hpp_dealDopplerNotUsedByPvt(q_isSperateBDS2And3, u_i, C_GNSS_FREQ_TYPE_L5, &gz_SatSigMeasCollect, &gz_hpp_task_ctrl.z_PvtSolution);
    }
  }

  /* satellite ele */
  pp_UpdateSatEleAzi(&gz_SatSigMeasCollect);

  /* report meas and satellite block*/
  hpp_makeAndReportMeasSatBlockCollect(pz_MeasBlock, &gz_SatSigMeasCollect);

  /* empirical value of troposphere */
  pp_UpdateTroposphericModel(&gz_SatSigMeasCollect);

  /* klb empirical value of ionosphere */
  pp_UpdateIonosphericModel(&gz_SatSigMeasCollect);

  ort_api_mainAntennaPVTinfo_put(&gz_hpp_task_ctrl.z_PvtSolution);

  const loc_ConfigParamGroup_t* pz_ConfigParamGrp = loc_cfg_getConfigParamGroup();

  /* PVT */
  gnss_PositionFix_t z_PositionFix = gz_SatSigMeasCollect.z_positionFix;
  if (pz_ConfigParamGrp->hpp_cfg.b_enable &&
      !pz_ConfigParamGrp->hpp_cfg.b_enable_rtk &&
      !pz_ConfigParamGrp->ppp_cfg.b_enable)
  {
    hpp_TdcpMeasBlkSendOut(&gz_SatSigMeasCollect, &z_PositionFix);
  }
  /* RTK */
  BOOL b_RtkSucc = FALSE;
  if(pz_ConfigParamGrp->hpp_cfg.b_enable_rtk)
  {
    memset(&z_PositionFix, 0, sizeof(gnss_PositionFix_t));
    b_RtkSucc = hpp_Rtk_Process(&(gz_hpp_task_ctrl.z_PvtSolution), &gz_SatSigMeasCollect, &gz_SatSigMeasCollectPre, &z_PositionFix);
    sm_api_PositionFix_Report(&z_PositionFix);
    hpp_TdcpMeasBlkSendOut(&gz_SatSigMeasCollect, &z_PositionFix);
  }
  if (b_RtkSucc)
  {
    LOGI(TAG_HPP, " Hpp_Rtk_Process success\n");
  }
  /* PPP */
  if(pz_ConfigParamGrp->ppp_cfg.b_enable)
  {
    hpp_SatSigMeasCollect_Sendout(&gz_SatSigMeasCollect, &gz_hpp_task_ctrl.z_PvtSolution);
  }
  return;
}

/**
 * @brief report meas block collect
 * @param[in] pz_MeasBlock
 * @param[in] pz_PositionFix
 */
static void hpp_makeAndReportMeasBlockCollect(const GnssMeasBlock_t* pz_MeasBlock, const gnss_PositionFix_t* pz_PositionFix)
{
	static uint64_t q_preSecondAlignMsec = 0;
	/* interval is 1s */
  if (gnss_CheckMeasSecondOverrun(pz_MeasBlock->z_Clock.z_gpsTime.t_fullMsec,
          &q_preSecondAlignMsec, 1.0f))
  {
		GnssMeasBlockCollect_t* pz_measBlockkCollect = OS_MALLOC(sizeof(GnssMeasBlockCollect_t));
		if (NULL != pz_measBlockkCollect)
		{
			memcpy(&pz_measBlockkCollect->z_measBlock, pz_MeasBlock, sizeof(GnssMeasBlock_t));
			pz_measBlockkCollect->d_lla[0] = pz_PositionFix->d_lla[0];
			pz_measBlockkCollect->d_lla[1] = pz_PositionFix->d_lla[1];
			pz_measBlockkCollect->d_lla[2] = pz_PositionFix->d_lla[2];
			/* Report */
			sm_api_GnssMeasBlockCollect_Report(pz_measBlockkCollect);

			OS_FREE(pz_measBlockkCollect);
		}
	}
}

/**
 * @brief HPP Receiver measurement put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_RcvMeas_Put(ipc_t* p_ipc)
{
  if (sizeof(GnssMeasBlock_t) != p_ipc->q_length)
  {
    LOGE(TAG_HPP, "Ipc 0x08x length %d mismatch payload %d\n",
      p_ipc->q_ipc_id, p_ipc->q_length, sizeof(GnssMeasBlock_t));
    return;
  }

  GnssMeasBlock_t* pz_MeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;

  /* Fill week into measurement */
  if (!gnss_AdjustMeasBlockWeek(pz_MeasBlock->w_measCount, pz_MeasBlock->z_meas, &pz_MeasBlock->z_Clock.z_gpsTime))
  {
    LOGW(TAG_HPP, "Adjust meas GSP week failed %.0f\n", 
      pz_MeasBlock->z_Clock.z_gpsTime.q_towMsec * TIME_MSEC_INV);
    /* Report 1.0 PVT position */
    gnss_PositionFix_t* pz_PositionFix = (gnss_PositionFix_t*)OS_MALLOC(sizeof(gnss_PositionFix_t));
    gnss_engine_create_empty_pos(pz_MeasBlock, pz_PositionFix);
    sm_api_PositionFix_Report(pz_PositionFix);
    OS_FREE(pz_PositionFix);

    return;
  }

  gnss_FeedbackInsMeasBlock_t* pz_CurrentFeedbackInsMeas = (gnss_FeedbackInsMeasBlock_t*)OS_MALLOC(sizeof(gnss_FeedbackInsMeasBlock_t));
  if (NULL == pz_CurrentFeedbackInsMeas)
  {
    LOGW(TAG_HPP, "Feedback Ins Meas fail!\n");
  }

  /** Version 1.0 PVT + RTK */
  uint64_t t_tstart = timepro_get_now();
  gnss_engine_inject_meas(pz_MeasBlock, pz_CurrentFeedbackInsMeas);
  LOGW(TAG_HPP, "Time cost SPP %llu ms\n", time_cost(t_tstart));

  /* Report 1.0 PVT position */
  gnss_PositionFix_t* pz_PositionFix = (gnss_PositionFix_t*)OS_MALLOC(sizeof(gnss_PositionFix_t));
  gnss_engine_get_position_fix(pz_PositionFix);
  sm_api_PositionFix_Report(pz_PositionFix);

  /** Report psr meas*/
  pz_CurrentFeedbackInsMeas->z_GpsTime = pz_MeasBlock->z_Clock.z_gpsTime;
  sm_api_GnssFeedbackInsMeasBlock_Report(pz_CurrentFeedbackInsMeas);
  OS_FREE(pz_CurrentFeedbackInsMeas);

	/** Report meas block collect */
	hpp_makeAndReportMeasBlockCollect(pz_MeasBlock, pz_PositionFix);
	OS_FREE(pz_PositionFix);

	/** Version 2.0 PVT + RTK */
  {
    t_tstart = timepro_get_now();
    hpp_MeasBlock_Proc(pz_MeasBlock);
    LOGW(TAG_HPP, "Time cost HPP %llu ms\n", time_cost(t_tstart));
  }

  return;
}

/**
 * @brief HPP Reference station correction put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_RefCorr_Put(ipc_t* p_ipc)
{
  GnssCorrBlock_t* corr = (GnssCorrBlock_t*)p_ipc->p_data;

  {
    GnssCorrBlock_t z_GnssCorr = { 0 };
    z_GnssCorr = *corr;

    for (int meas_idx = 0; meas_idx < z_GnssCorr.w_measCount; meas_idx++)
    {
      GnssMeas_t* m = &z_GnssCorr.z_meas[meas_idx];
      m->u_constellation = gnss_CvtConstellation(m->u_constellation, m->u_svid);
    }

    gnss_engine_inject_correct(&z_GnssCorr);
  }

  gnss_AdjustMeasBlockWeek(corr->w_measCount,corr->z_meas,& corr->z_Clock.z_gpsTime);
  rtk_corrUpdate(corr);
  return;
}

/**
 * @brief HPP Ephemeris data put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_Ephemeris_Put(ipc_t* p_ipc)
{
  gnss_Ephemeris_t* eph = (gnss_Ephemeris_t*)p_ipc->p_data;

  gnss_engine_inject_ephemeris(eph);
  return;
}

/**
 * @brief HPP Receiver Measurement Put Using RTCM
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_RcvMeasRTCM_Put(ipc_t* p_ipc)
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParam = loc_cfg_getConfigParamGroup();
  if (NULL == pz_loc_ConfigParam)
  {
    return;
  }

  static uint64_t q_preSecondAlignMsec = 0;

  LOGD(TAG_HPP, "HPP receive rover rtcm data length %d\n", p_ipc->q_length);

  if ((FALSE == pz_loc_ConfigParam->hpp_cfg.b_enable_rcv_rtcm_dec) ||
    (NULL == gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder))
  {
    return;
  }
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();

  int8_t ret = 0;
  // Combined data of previous stored and ipc
  uint32_t length = 0;
  uint8_t* buffer = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder->e2emsglen + p_ipc->q_length + 1));
  if (!rtcmE2eSkip(gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder, p_ipc->p_data, p_ipc->q_length, buffer, &length))
  {
    OS_FREE(buffer);
    return;
  }

  for (uint32_t i = 0; i < length; i++)
  {
    ret = rtcm_dec_input(gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder, buffer[i]);
    if (ret <= 0)
    {
      continue;
    }

    if (1 == ret)
    {
      if (gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder->is_complete)
      {
        GnssMeasBlock_t z_MeasBlock = { 0 };
        rtcm_ExtractMeasBlk(gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder, &z_MeasBlock);
        if (gnss_CheckMeasSecondAlign(z_MeasBlock.z_Clock.z_gpsTime.t_fullMsec, 
            &q_preSecondAlignMsec, pz_loc_ConfigParamGroup->sm_cfg.f_meas_update_interval))
        {
          gnss_engine_inject_meas(&z_MeasBlock,NULL);
        }
      }
    }

    if (2 == ret)
    {
      gnss_engine_inject_ephemeris(&gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder->ephemeris);
    }
  }
  OS_FREE(buffer);
  return;
}

/**
 * @brief HPP Reference Correction Put Using RTCM
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_RefCorrRTCM_Put(ipc_t* p_ipc)
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParam = loc_cfg_getConfigParamGroup();
  if (NULL == pz_loc_ConfigParam)
  {
    return;
  }

  LOGI(TAG_HPP, "Receive base RTCM data length %d\n", p_ipc->q_length);

  if ((FALSE == pz_loc_ConfigParam->hpp_cfg.b_enable_ref_rtcm_dec) ||
    (NULL == gz_hpp_task_ctrl.pz_hppRefRtcmDecoder))
  {
    return;
  }
  
  int8_t ret = 0;
  // Combined data of previous stored and ipc
  uint32_t length = 0;
  uint8_t* buffer = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder->e2emsglen + p_ipc->q_length + 1));
  if (!rtcmE2eSkip(gz_hpp_task_ctrl.pz_hppRcvRtcmDecoder, p_ipc->p_data, p_ipc->q_length, buffer, &length))
  {
    OS_FREE(buffer);
    return;
  }
  for (uint32_t i = 0; i < length; i++)
  {
    ret = rtcm_dec_input(gz_hpp_task_ctrl.pz_hppRefRtcmDecoder, buffer[i]);
    if (ret <= 0)
    {
      continue;
    }

    if (1 == ret)
    {
      if (gz_hpp_task_ctrl.pz_hppRefRtcmDecoder->is_complete)
      {
        GnssCorrBlock_t z_CorrectBlock = { 0 };
        rtcm_ExtractCorrBlk(gz_hpp_task_ctrl.pz_hppRefRtcmDecoder, &z_CorrectBlock);
        gnss_engine_inject_correct(&z_CorrectBlock);
      }
    }
  }
  OS_FREE(buffer);
  return;
}

/**
 * @brief HPP Receive twin measurement Put for dual-ant orienter
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void hpp_RcvMeasTwin_Put(ipc_t* p_ipc)
{
  return;
}


/**
 * @brief Sanity Check IPC 
 * @param[in]   p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL hpp_IpcCheck(ipc_t* p_ipc)
{
  if (TASK_INDEX_HPP != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_HPP_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_HPP_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief HPP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void hpp_Proc(ipc_t* p_ipc)
{
  if (FALSE == hpp_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_HPP_INIT:
    hpp_Init(p_ipc);
    break;
  case C_M_HPP_START:
    hpp_Start(p_ipc);
    break;
  case C_M_HPP_STOP:
    hpp_Stop(p_ipc);
    break;
  case C_M_HPP_RELEASE:
    hpp_Release(p_ipc);
    break;
  case C_M_HPP_RCV_MEAS_1HZ_PUT:
    hpp_RcvMeas_Put(p_ipc);
    break;
  case C_M_HPP_REF_CORR_PUT:
    hpp_RefCorr_Put(p_ipc);
    break;
  case C_M_HPP_EPHEMERIS_PUT:
    hpp_Ephemeris_Put(p_ipc);
    break;
  case C_M_HPP_RCV_MEAS_RTCM:
    hpp_RcvMeasRTCM_Put(p_ipc);
    break;
  case C_M_HPP_REF_CORR_RTCM:
    hpp_RefCorrRTCM_Put(p_ipc);
    break;
  case C_M_HPP_RCV_MEAS_TWIN_PUT:
    hpp_RcvMeasTwin_Put(p_ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief HPP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* hpp_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_HPP;
  
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
      hpp_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief HPP API for initilization
 * @return      None
 */
BOOL hpp_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief HPP API for start
 * @return      None
 */
BOOL hpp_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_START, NULL, 0);
  return TRUE;
}

/**
 * @brief HPP API for stop
 * @return      None
 */
BOOL hpp_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief HPP API for release
 * @return      None
 */
BOOL hpp_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief HPP API to Put Receiver Measurement Block
 * @param[in] meas - Receiver Measurement Block
 * @return None
 */
void hpp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas)
{
  meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_RCV_MEAS_1HZ_PUT, (uint8_t*)meas, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief HPP API to Put Reference Correction Block
 * @param[in] corr - Reference Correction Block
 * @return    None
 */
void hpp_api_RefCorrBlk_Put(GnssCorrBlock_t* corr)
{
  corr->u_version = VERSION_GNSS_CORR_BLOCK;
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_REF_CORR_PUT, (uint8_t*)corr, sizeof(GnssCorrBlock_t));
  return;
}

/**
 * @brief HPP API to Put Ephemeris Structure
 * @param[in] eph - Ephemeris Structure
 * @return    None
 */
void hpp_api_Ephemeris_Put(gnss_Ephemeris_t* eph)
{
  eph->u_version = VERSION_GNSS_EPHEMERIS;
  ipctask_SendMessage(TASK_INDEX_HPP, C_M_HPP_EPHEMERIS_PUT, (uint8_t*)eph, sizeof(gnss_Ephemeris_t));
  return;
}

/**
 * @brief HPP API to Put receiver twin measurement
 * @param[in] twin_meas
 * @return    None
 */
void hpp_api_RcvMeasTwin_Put(GnssMeasBlock_t* twin_meas)
{
  twin_meas->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_ORT, C_M_ORT_ORT_MEAS_PUT, (uint8_t*)twin_meas, sizeof(GnssMeasBlock_t));
  return;
}
