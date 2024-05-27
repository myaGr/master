/**@file        rtk_proc.c
 * @brief       Real-time kinematic(RTK) Main Process module
 * @details
 * @author      caizhijie
 * @date        2022/06/21
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "rtk_proc.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "gnss_filter_type.h"
#include "gnss_type.h"
#include "gnss_common.h"
#include "rtk_filter_sol.h"
#include "rtk_amb_fix.h"
#include "rtk_pre_fix.h"
#include "rtk_corr_check.h"
#include "rtk_pos_check.h"
#include "mw_log.h"
#include "cmn_utils.h"
#include "gnss_prep.h"
#include "cmn_turboEdit.h"
#include "cmn_multipathDetect.h"
#include "cmn_multiFreq_detectCS.h"
#include "cmn_dopplerDetectCS.h"
#include "gnss_tdcp_detectCS.h"
#include "gnss_identify_scene.h"
#include "rtd_sol.h"
#include "rtk_integrity.h"
#include "gnss_engine_api.h"

/* tag of whether date is 3 freq or not */
uint8_t gu_sigMeasCount[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX];

/*used for RTK float solute*/
rtk_filterInfo_t gz_RTKfilterInfo;
gnss_EKFstateRepresentPool_t gz_RTK_EKFstateRepPool;

/*used for RTD solute*/
rtd_filterInfo_t* gpz_RTDfilterInfo;
rtd_EKFstateRepresentPool_t gz_RTD_EKFstateRepPool;

gnss_rtkAmbFixInputInfo_t gz_RTKambFixInputInfo;
GnssCorrBlock_t* gpz_RtkCorrBlock[RTK_CORR_MAX_BLOCK] = { NULL };/*[0] saves latest valid corr,
                                                                     [1] & [1+] save latest succ updated corr in float filter*/
cmn_turboEditFilterSet gz_RTKgfDetectFilter;
cmn_turboEditFilterSet gz_RTKmwDetectFilter;
cmn_multipathDetectSet gz_rtkMPdetectFilter;
cmn_multiFreqDetectSet gz_RTKMultFreqDetectFilter;
gnss_dopplerDetectPairingBlock_t gz_dopplerDetectPairingBlockRTK;
gnss_TdcpMeasBlock_t* gpz_preTdcpMeasRTK = NULL;

cmn_turboEditFilterSet gz_refRTKgfDetectFilter;
GnssCorrSlipFlag_t gz_RTKcorrSlipFlag;

 /**
 *@brief load the default option for the RTK algorithm
 * @param[out]  pz_opt represent the algorithm optimization for the RTK
 * @return     void
 */
void RTK_loadDefaultOption(rtk_alg_opt_t* pz_opt)
{
	pz_opt->z_usedSys = (ALGO_GPS_SYS | ALGO_BDS_SYS | ALGO_GLO_SYS | ALGO_GAL_SYS | ALGO_QZS_SYS);
	pz_opt->z_usedFreq = (ALGO_L1_FREQ | ALGO_L2_FREQ | ALGO_L5_FREQ);
	pz_opt->q_isSperateBDS2And3 = TRUE;
	pz_opt->d_elmin = 7.0 * DEG2RAD;
	pz_opt->d_maxinno = 25.0;
	pz_opt->d_maxDiffAge = 150.0;
  pz_opt->d_maxPrefixAge = 60.0;
  pz_opt->f_sample = 1.0f;
	return;
}

/**
 * @brief Initialize RTK module
 * @param[in]  pz_opt represent the algorithm option for the RTK
 * @param[in]  f_sample background process sample
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t rtk_init(const rtk_alg_opt_t* pz_opt, float f_sample)
{
  uint8_t u_status = 0;
  uint16_t w_i = 0;
  uint16_t w_j = 0;

  for (w_i = C_GNSS_GPS; w_i < C_GNSS_MAX; w_i++)
  {
    for (w_j = 0; w_j < MAX_GNSS_SIGNAL_FREQ; w_j++)
    {
      gu_sigMeasCount[w_i][w_j] = 0;
    }
  }

  cmn_initTEfilterSet(&gz_RTKgfDetectFilter);
  cmn_initTEfilterSet(&gz_RTKmwDetectFilter);
  cmn_initMPdetectorSet(&gz_rtkMPdetectFilter, f_sample);
  cmn_initMultiFreqDetectCyleSlip(&gz_RTKMultFreqDetectFilter);
  gz_dopplerDetectPairingBlockRTK.pz_curBlock = NULL;
  gz_dopplerDetectPairingBlockRTK.pz_preBlock = NULL;
  cmn_initDopplerDetectCyleSlip(&gz_dopplerDetectPairingBlockRTK);

  //initilize the information of RTK filter
  gz_RTKfilterInfo.w_n = 0;
  gz_RTKfilterInfo.u_start = 0;
  gz_RTKfilterInfo.u_threeFreqTag = FALSE;
  gz_RTKfilterInfo.w_nmax = GNSS_MAX_FILTER_STATE_NUM;
  gz_RTKfilterInfo.pd_deltaX = (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  gz_RTKfilterInfo.pd_X      = (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  gz_RTKfilterInfo.pd_Q      = (double*)OS_MALLOC_FAST(NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
  gz_RTKfilterInfo.pz_filterSol = (rtk_filter_sol_t*)OS_MALLOC_FAST(sizeof(rtk_filter_sol_t));
  gz_RTKfilterInfo.pq_paraValid = (BOOL*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(BOOL));
  gz_RTKfilterInfo.pz_rcvInitClkInfo = (rtk_rcv_init_clk_info_t*)OS_MALLOC_FAST(sizeof(rtk_rcv_init_clk_info_t));
  gz_RTKfilterInfo.pz_historyPos_t = (rtk_history_pos_t*)OS_MALLOC_FAST(sizeof(rtk_history_pos_t));
  gpz_RTDfilterInfo= (rtd_filterInfo_t*)OS_MALLOC_FAST(sizeof(rtd_filterInfo_t));
  if (any_Ptrs_Null(7, gz_RTKfilterInfo.pd_deltaX, gz_RTKfilterInfo.pd_X, 
    gz_RTKfilterInfo.pd_Q, gz_RTKfilterInfo.pz_filterSol, gz_RTKfilterInfo.pq_paraValid, gz_RTKfilterInfo.pz_rcvInitClkInfo, gpz_RTDfilterInfo))
  {
    LOGW(TAG_HPP, "rtk_init X or Q malloc failed");
    OS_FREE(gz_RTKfilterInfo.pd_deltaX);
    OS_FREE(gz_RTKfilterInfo.pd_X);
    OS_FREE(gz_RTKfilterInfo.pd_Q);
    OS_FREE(gz_RTKfilterInfo.pz_filterSol);
    OS_FREE(gz_RTKfilterInfo.pq_paraValid);
    OS_FREE(gz_RTKfilterInfo.pz_rcvInitClkInfo);
    OS_FREE(gpz_RTDfilterInfo);
    u_status = 1;
  }
  else
  {
    //initilize the information of EKF status
    tm_initGpsTime(&(gz_RTK_EKFstateRepPool.z_gpsTime));
    gz_RTK_EKFstateRepPool.d_continueFilterTime = 0.0;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM && !u_status; ++w_i)
    {
      gz_RTK_EKFstateRepPool.pz_satPool[w_i] = (gnss_EKFstateRepresent_t*)OS_MALLOC_FAST(1 * sizeof(gnss_EKFstateRepresent_t));
      if (NULL == gz_RTK_EKFstateRepPool.pz_satPool[w_i])
      {
        u_status = 2;
        break;
      }
      gz_RTK_EKFstateRepPool.pz_satPool[w_i]->w_index = INVALID_INDEX;
      initEKFstateRepresent(gz_RTK_EKFstateRepPool.pz_satPool[w_i], gz_RTKfilterInfo.w_nmax, gz_RTKfilterInfo.pd_X,
          gz_RTKfilterInfo.pd_Q);
    }

    for (w_i = 0; w_i < MAX_FILTER_POS_STATUS; w_i++)
    {
      gz_RTKfilterInfo.pz_filterSol->pq_diffFlagWithSPP[w_i] = 0x0;
      gz_RTKfilterInfo.pz_filterSol->pq_diffFlagWithPredict[w_i] = 0x0;
    }
    rtk_initHistoryPos(gz_RTKfilterInfo.pz_historyPos_t);

    gz_RTKfilterInfo.z_kfStatus = KF_INIT;
  }

  //option
  if (NULL != pz_opt)
  {
    gz_RTKfilterInfo.z_opt = *(pz_opt);
    gz_RTKfilterInfo.z_opt.f_sample = f_sample;
  }
  else
  {
    RTK_loadDefaultOption(&(gz_RTKfilterInfo.z_opt));
  }
  gz_RTKfilterInfo.z_opt.b_enableStaticConstraints = TRUE;
  gz_RTKfilterInfo.z_opt.b_enableL1onlyFix = FALSE;

  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    gz_RTKfilterInfo.pq_paraValid[w_i] = FALSE;
  }
  gz_RTKfilterInfo.f_sample = f_sample;
  gz_RTKfilterInfo.t_openSkyCount = 0;
  gz_RTKfilterInfo.t_closeSkyCount = 0;
  gz_RTKfilterInfo.w_floatCount = 0;
  gz_RTKfilterInfo.w_rtdCount = 0;
  gz_RTKfilterInfo.w_filterWithPhaseCount = 0;
  gz_RTKfilterInfo.d_resetPercent = 0.0;
  gz_RTKfilterInfo.u_goodSceneStatusFlag = 0;
  gz_RTKfilterInfo.pz_pvtResult = NULL;
  for (w_i = 0; w_i < RTK_CORR_MAX_BLOCK; ++w_i)
  {
    gpz_RtkCorrBlock[w_i] = (GnssCorrBlock_t*)OS_MALLOC_FAST(sizeof(GnssCorrBlock_t));
  }

  gz_RTKcorrSlipFlag.u_changeStationFlag = 0;
  for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    for (w_j = 0; w_j < MAX_GNSS_SIGNAL_FREQ; ++w_j)
    {
      gz_RTKcorrSlipFlag.u_slip[w_i][w_j] =  0 ;
    }
  }
  for (w_i = C_GNSS_GPS; w_i < C_GNSS_MAX; ++w_i)
  {
    for (w_j = 0; w_j < C_GNSS_FREQ_TYPE_MAX; ++w_j)
    {
      gz_RTKfilterInfo.pd_phaseCodeClkDiff[w_i][w_j] = 0.0;
    }
  }
  gpz_preTdcpMeasRTK = (gnss_TdcpMeasBlock_t*)OS_MALLOC_FAST(sizeof(gnss_TdcpMeasBlock_t));
  rtd_init(&(gz_RTKfilterInfo.z_opt), gpz_RTDfilterInfo, &gz_RTD_EKFstateRepPool);
  gz_RTKfilterInfo.pz_integ = rtk_integ_init();
  memset(&gz_RTKfilterInfo.z_timeLastHold, 0, sizeof(GpsTime_t));
  gz_RTKfilterInfo.b_staticFlag = FALSE;
  return u_status;
}

static void rtk_DeinitAmbInfo()
{
  for (uint8_t i = 0; i < GNSS_MAX_AMB_FIXED_TYPE; ++i)
  {
    free_fixedSignalAmbPool(&gz_RTKambFixInputInfo.pz_preFixedAmbPool[i]);
  }
}

/**
 * @brief deinitilize the RTK algorithm
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void rtk_deInit()
{
  uint16_t w_i = 0;
  //deinitilize the information of PPP filter
  cmn_deinitTEfilterSet(&gz_RTKgfDetectFilter);
  cmn_deinitTEfilterSet(&gz_RTKmwDetectFilter);
  cmn_deinitTEfilterSet(&gz_refRTKgfDetectFilter);
  cmn_deinitMPdetectorSet(&gz_rtkMPdetectFilter);
  cmn_deinitmultiFreqDetectCyleSlip(&gz_RTKMultFreqDetectFilter);
  cmn_deinitDopplerDetectCyleSlip(&gz_dopplerDetectPairingBlockRTK);
  gz_RTKfilterInfo.w_n = 0;
  gz_RTKfilterInfo.w_nmax = 0;
  gz_RTKfilterInfo.u_start = 0;
  OS_FREE(gz_RTKfilterInfo.pd_deltaX);
  OS_FREE(gz_RTKfilterInfo.pd_X);
  OS_FREE(gz_RTKfilterInfo.pd_Q);
  OS_FREE(gz_RTKfilterInfo.pq_paraValid);
  OS_FREE(gz_RTKfilterInfo.pz_filterSol);
  OS_FREE(gz_RTKfilterInfo.pz_rcvInitClkInfo);
  OS_FREE(gz_RTKfilterInfo.pz_historyPos_t);
  //deinitilize the information of EKF status
  tm_initGpsTime(&(gz_RTK_EKFstateRepPool.z_gpsTime));
  gz_RTK_EKFstateRepPool.d_continueFilterTime = 0.0;
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    if (NULL != gz_RTK_EKFstateRepPool.pz_satPool[w_i])
    {
      OS_FREE(gz_RTK_EKFstateRepPool.pz_satPool[w_i]);
    }
  }
  for (w_i = 0; w_i < RTK_CORR_MAX_BLOCK; ++w_i)
  {
    OS_FREE(gpz_RtkCorrBlock[w_i]);
  }
  if (NULL != gpz_preTdcpMeasRTK)
  {
    OS_FREE(gpz_preTdcpMeasRTK);
  }
  rtd_deinit(gpz_RTDfilterInfo, &gz_RTD_EKFstateRepPool);
  rtk_DeinitAmbInfo();
  rtk_integ_deinit(gz_RTKfilterInfo.pz_integ);
  gz_RTKfilterInfo.pz_integ = NULL;
  return;
}
/**
 * @brief update the Ref correct data fit Ref position or not
 * @param[in]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[out] void
 * @return     true for update success; false for other case
 */
BOOL rtk_corrUpdate(const GnssCorrBlock_t * pz_GnssCorrBlockUpdate)
{
  GnssCorrBlock_t* pz_RTKcorrBlockUpdate = NULL;/* copy updated ref corr & pos*/
  uint8_t u_i = 0;
  double d_deltaTime = 0.0;

  // 1. check Ref position availability
  if (NULL == pz_GnssCorrBlockUpdate || pz_GnssCorrBlockUpdate->w_refStationId == 0
    || gnss_Dot(pz_GnssCorrBlockUpdate->d_refPosEcef,
        pz_GnssCorrBlockUpdate->d_refPosEcef, 3) < FABS_ZEROS * FABS_ZEROS)
  {
    LOGI(TAG_HPP, "--CorrUpdate-- Fail:Ref id/position isn't valid\n");
    return FALSE;
  }

  if (NULL == gpz_RtkCorrBlock[0])
  {
    return FALSE;
  }

  // 2. check the time (tm_GpsTimeDiff
  if(gpz_RtkCorrBlock[0]->w_refStationId != 0)
  {
    d_deltaTime = tm_GpsTimeDiff(&pz_GnssCorrBlockUpdate->z_Clock.z_gpsTime,
      &gpz_RtkCorrBlock[0]->z_Clock.z_gpsTime);
    if (d_deltaTime < FABS_ZEROS)
    {
      LOGI(TAG_HPP, "--CorrUpdate-- Fail:Ref time isn't correct, dt=%.2f\n", -d_deltaTime);
      return FALSE;
    }
  }

  pz_RTKcorrBlockUpdate = (GnssCorrBlock_t*)OS_MALLOC_FAST(sizeof(GnssCorrBlock_t));
  memcpy(pz_RTKcorrBlockUpdate, pz_GnssCorrBlockUpdate, sizeof(GnssCorrBlock_t));
  rtk_initCorrSatPosClk(pz_RTKcorrBlockUpdate);
  rtk_calCorrDataSatPosClk(pz_RTKcorrBlockUpdate);
  
  // 3. check the Ref correct data availability
  if (!rtk_checkCorrValid(pz_RTKcorrBlockUpdate))
  {
    LOGI(TAG_HPP, "--CorrUpdate-- Fail:Ref correct satnum too less/lock of system\n");
    OS_FREE(pz_RTKcorrBlockUpdate);
    return FALSE;
  }

  // 4.1. check the Ref correct data fit Ref position or not
  if (rtk_checkCorrPos(pz_RTKcorrBlockUpdate, pz_RTKcorrBlockUpdate->d_refPosEcef))
  {
    //updata Ref correct data struct
    pz_RTKcorrBlockUpdate->w_switchStationId = pz_RTKcorrBlockUpdate->w_refStationId;
    LOGI(TAG_HPP, "--CorrUpdate-- Succ:Ref correct data fit Ref position\n");
  }
  else
  {
    // 4.2. check the Ref correct data fit previous Ref position or not
    if (fabs(gpz_RtkCorrBlock[0]->d_refPosEcef[0] -
      pz_RTKcorrBlockUpdate->d_refPosEcef[0]) < FABS_ZEROS &&
      fabs(gpz_RtkCorrBlock[0]->d_refPosEcef[1] -
        pz_RTKcorrBlockUpdate->d_refPosEcef[1]) < FABS_ZEROS &&
      fabs(gpz_RtkCorrBlock[0]->d_refPosEcef[2] -
        pz_RTKcorrBlockUpdate->d_refPosEcef[2]) < FABS_ZEROS)
    {
      LOGI(TAG_HPP, "--CorrUpdate-- Fail:Ref correct data don't fit updated/previous Ref position\n");
      OS_FREE(pz_RTKcorrBlockUpdate);
      return FALSE;
    }
    if (gnss_Dot(gpz_RtkCorrBlock[0]->d_refPosEcef,
      gpz_RtkCorrBlock[0]->d_refPosEcef, 3) > FABS_ZEROS * FABS_ZEROS &&
      rtk_checkCorrPos(pz_RTKcorrBlockUpdate, gpz_RtkCorrBlock[0]->d_refPosEcef))
    {
      //Ref position update before Ref correct data change ref station
      //updata Ref correct data struct besides Ref position 
      for (u_i = 0; u_i < 3; u_i++)
      {
        pz_RTKcorrBlockUpdate->d_refPosEcef[u_i] = gpz_RtkCorrBlock[0]->d_refPosEcef[u_i];
      }
      pz_RTKcorrBlockUpdate->w_refStationId = gpz_RtkCorrBlock[0]->w_refStationId;
      pz_RTKcorrBlockUpdate->w_switchStationId = gpz_RtkCorrBlock[0]->w_refStationId;
      LOGI(TAG_HPP, "--CorrUpdate-- Succ:Ref correct data fit previous Ref position\n");
    }
    else
    {
      LOGI(TAG_HPP, "--CorrUpdate-- Fail:lost Ref position\n");
      OS_FREE(pz_RTKcorrBlockUpdate);
      return FALSE;
    }
  }

  //5. detect corr slip & save date
  rtk_corrDetectCycleSlip(&gz_refRTKgfDetectFilter, &gz_RTKcorrSlipFlag,
    pz_RTKcorrBlockUpdate, gpz_RtkCorrBlock[0]);
  memcpy(gpz_RtkCorrBlock[0], pz_RTKcorrBlockUpdate, sizeof(GnssCorrBlock_t));  
  OS_FREE(pz_RTKcorrBlockUpdate);
  return TRUE;
}

/**
 * @brief      save succ updated corr in float filter, for possibly base change
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void rtk_refCorrSave()
{
  if(tm_GpsTimeDiff(&gpz_RtkCorrBlock[0]->z_Clock.z_gpsTime,
    &gpz_RtkCorrBlock[1]->z_Clock.z_gpsTime) > FABS_ZEROS)
  {
    for (int u_i = RTK_CORR_MAX_BLOCK - 1; u_i >= 1; u_i--)
    {
      memcpy(gpz_RtkCorrBlock[u_i], gpz_RtkCorrBlock[u_i - 1], sizeof(GnssCorrBlock_t));
    }
  }
}

/**
 * @brief extract covariance from filter
 * @param[in] p_Q covariance
 * @param[in] pd_index index
 * @param[in] n number of index
 * @param[in] p_Qout output
 */
void RTK_recoverQ(const double* pd_Q, const int16_t* pw_index, uint8_t u_n, double* pd_Qout)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  for (u_i = 0; u_i < u_n; ++u_i)
  {
    for (u_j = 0; u_j < u_n; ++u_j)
    {
      pd_Qout[u_i * u_n + u_j] = pd_Q[IUTM(pw_index[u_i], pw_index[u_j])];
    }
  }
  return;
}

/**
 * @brief      save pvt rlt in filterInfo's filter pos
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     void
 */
void RTK_savePVTinFilterPos(const gnss_PVTResult_t* pz_PVTresult)
{
  uint8_t u_i = 0;
  if (NULL != gz_RTKfilterInfo.pz_filterSol && NULL != pz_PVTresult
    && gnss_Dot(pz_PVTresult->z_positionFix.d_xyz, pz_PVTresult->z_positionFix.d_xyz, 3) > 1e-3
    && GNSS_FIX_FLAG_INVALID != pz_PVTresult->z_positionFix.u_fixFlag)
  {
    for (u_i = 0; u_i < 3; u_i++)
    {
      gz_RTKfilterInfo.pz_filterSol->pd_filterPos[RTK_FILTER_POS_SPP][u_i] = pz_PVTresult->z_positionFix.d_xyz[u_i];
      gz_RTKfilterInfo.pz_filterSol->pd_filterVel[RTK_FILTER_POS_SPP][u_i] = pz_PVTresult->z_positionFix.f_velXyz[u_i];
    }
    gz_RTKfilterInfo.pz_filterSol->pu_posValid[RTK_FILTER_POS_SPP] = RTK_FILTER_SOL_STAT_SAVE;
  }
}

/**
 * @brief RTK print measure
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @return     void
 */
void RTK_printMeasInfo(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const rtk_filterInfo_t* pz_RTKfilterInfo)
{
  float* pf_dat = NULL;
  uint8_t* pu_datValid = NULL;
  rtk_measSNRprint_t* pz_SNRprint = NULL;
  do
  {
    LogLevelEnum z_logLevel;
    z_logLevel = log_GetLogLevel();
    if(z_logLevel < LOG_LEVEL_I)
    {
      continue;
    }

    char char_sat[4] = "";
    uint8_t u_signalIndex = 0;
    uint32_t q_satIndex = 0;
    const uint8_t u_size = 8;
    const uint8_t u_datPack = (MAX_GNSS_SIGNAL_FREQ + 2);
    const gnss_SatelliteMeas_t* pz_satMeas = NULL;
    const gnss_SignalMeas_t* pz_sigMeas = NULL;
    pf_dat = OS_MALLOC(ALL_GNSS_SYS_SV_NUMBER * u_datPack * sizeof(float));
    pu_datValid = OS_MALLOC(ALL_GNSS_SYS_SV_NUMBER * sizeof(uint8_t));
    pz_SNRprint = RTK_malloc_SNRprint();
    if (pf_dat == NULL || pu_datValid == NULL || pz_SNRprint == NULL)
    {
      continue;
    }
    memset(pf_dat, 0, ALL_GNSS_SYS_SV_NUMBER * u_datPack * sizeof(float));
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_satMeas || pz_satMeas->z_satPosVelClk.q_iode <= 0)
      {
        continue;
      }
      pu_datValid[q_satIndex] = FALSE;
      for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
      {
        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid))
        {
          continue;
        }
        if (pz_satMeas->z_satPosVelClk.f_elevation > FABS_ZEROS)
        {
          pf_dat[q_satIndex * u_datPack + 0] = (float)(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG);
          pf_dat[q_satIndex * u_datPack + 1] = (float)(pz_satMeas->z_satPosVelClk.f_azimuth);
        }
        if (pz_sigMeas->f_cn0 > FABS_ZEROS)
        {
          pf_dat[q_satIndex * u_datPack + 2 + u_signalIndex] = pz_sigMeas->f_cn0;
          pu_datValid[q_satIndex] = TRUE;
        }
      }
    }
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      if (TRUE == pu_datValid[q_satIndex])
      {
        satidx_SatString(q_satIndex, char_sat);
        if ((pz_SNRprint->pu_prn - pz_SNRprint->pu_prn_sat) < AMB_PRINT_SIZE - u_size)
        {
          pz_SNRprint->pu_prn += snprintf(pz_SNRprint->pu_prn, u_size, "  %4s", char_sat);
          pz_SNRprint->pu_ele += snprintf(pz_SNRprint->pu_ele, u_size, "%6.1f", pf_dat[q_satIndex * u_datPack + 0]);
          pz_SNRprint->pu_azi += snprintf(pz_SNRprint->pu_azi, u_size, "%6.1f", pf_dat[q_satIndex * u_datPack + 1]);
          for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
          {
            if (pf_dat[q_satIndex * u_datPack + 2 + u_signalIndex] > FABS_ZEROS)
            {
              pz_SNRprint->pu_snr[u_signalIndex] += snprintf(pz_SNRprint->pu_snr[u_signalIndex], u_size, "%6.1f",
                pf_dat[q_satIndex * u_datPack + 2 + u_signalIndex]);
            }
            else
            {
              pz_SNRprint->pu_snr[u_signalIndex] += snprintf(pz_SNRprint->pu_snr[u_signalIndex], u_size, "      ");
            }
          }
        }
      }
    }
    if ((pz_SNRprint->pu_prn - pz_SNRprint->pu_prn_sat) > 0)
    {
      LOGI(TAG_HPP, "............Measment..SNR..print..start...........\n");
      LOGI(TAG_HPP, "PRN  : %s\n", pz_SNRprint->pu_prn_sat);
      LOGI(TAG_HPP, "ele  : %s\n", pz_SNRprint->pu_ele_sat);
      LOGI(TAG_HPP, "azi_r: %s\n", pz_SNRprint->pu_azi_sat);
      for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
      {
        LOGI(TAG_HPP, "SNR_%d: %s\n", u_signalIndex, pz_SNRprint->pu_snr_sat[u_signalIndex]);
      }
      LOGI(TAG_HPP, "............Measment..SNR..print..end.............\n");
    }
  } while (0);
  OS_FREE(pf_dat);
  OS_FREE(pu_datValid);
  OS_FREE(pz_SNRprint);
  return;
}
/**
 * @brief RTK count measure and print measure & time 
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @return     void
 */
void RTK_printEpochAndCountMeas(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const rtk_filterInfo_t* pz_RTKfilterInfo)
{
  uint8_t u_signalIndex = 0;
  uint8_t u_PRnum = 0;
  uint8_t u_CPnum = 0;
  uint8_t u_constellation = 0;
  uint8_t u_SNRnum[MAX_GNSS_SIGNAL_FREQ] = { 0 };
  uint32_t q_satIndex = 0;
  double d_avgSNR[MAX_GNSS_SIGNAL_FREQ] = { 0.0 };
  double d_tow;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  EpochTime_t z_epochT;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;

  d_tow = pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV;
  tm_cvt_GpstToEpoch(&pz_satSigMeasCollect->z_tor, &z_epochT);

  //LOGW for realtime print out
  //LOGW(TAG_HPP, "\n");
  LOGW(TAG_HPP, "==========hpp start filter %.1f, %4d/%02d/%02d %02d:%02d:%05.2f==========\n",
    d_tow, z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour, z_epochT.min, z_epochT.second);

  for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
  {
    u_SNRnum[u_signalIndex] = 0;
    d_avgSNR[u_signalIndex] = 0.0;
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_satMeas)
      {
        continue;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid))
      {
        continue;
      }
      u_constellation = pz_sigMeas->u_constellation;
      if (C_GNSS_BDS2 == u_constellation || C_GNSS_BDS3 == u_constellation)
      {
        u_constellation = C_GNSS_BDS3;
      }
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      if (pz_sigMeas->z_measStatusFlag.b_prValid && pz_sigMeas->d_pseudoRange > 1.0e-2)
      {
        u_PRnum++;
        gu_sigMeasCount[u_constellation][u_signalIndex]++;
      }
      if (fabs(pz_sigMeas->d_carrierPhase) > FABS_ZEROS)
      {
        u_CPnum++;
      }
      if (pz_sigMeas->f_cn0 > FABS_ZEROS)
      {
        u_SNRnum[u_signalIndex]++;
        d_avgSNR[u_signalIndex] += pz_sigMeas->f_cn0;
      }
    }
    if (d_avgSNR[u_signalIndex] > FABS_ZEROS && u_SNRnum[u_signalIndex] > 0)
    {
      d_avgSNR[u_signalIndex] = d_avgSNR[u_signalIndex] / u_SNRnum[u_signalIndex];
    }
  }
  //LOGW for realtime print out
  LOGW(TAG_HPP, "--MEAS--| CP=%d PR=%d | SNR_1=%3.1f 2=%3.1f 3=%3.1f\n", u_CPnum, u_PRnum,
    d_avgSNR[0], d_avgSNR[1], d_avgSNR[2]);

  return;
}
/**
 * @brief fillup the t_openSkyCount and t_closeSkyCount field in the struct rtk_filterInfo_t
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     void
 */
void RTK_recordSceneInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  gnss_SceneType z_sceneType = OPEN_SKY_SCENE;
  gz_RTKfilterInfo.u_goodSceneStatusFlag = 0;
  gz_RTKfilterInfo.t_openSkyCount <<= 1;
  gz_RTKfilterInfo.t_closeSkyCount <<= 1;
  z_sceneType = gnss_usingObsIdentifyScene(pz_satSigMeasCollect);
  if (OPEN_SKY_SCENE == z_sceneType)
  {
    gz_RTKfilterInfo.t_openSkyCount |= 1;
  }
  else if (CLOSE_SKY_SCENE == z_sceneType)
  {
    gz_RTKfilterInfo.t_closeSkyCount |= 1;
  }

  if ((gz_RTKfilterInfo.t_openSkyCount & 0x3) == 0x3)
  {
    gz_RTKfilterInfo.u_goodSceneStatusFlag = 1;
  }
  return;
}
/**
 * @brief      some of gz_RTKfilterInfo's value need to init every epoch
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     void
 */
void RTK_EpochFilterInfoInit(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;

  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
  {
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      gu_sigMeasCount[u_i][u_j] = 0;
    }
  }

  gz_RTKfilterInfo.q_filterPosValid = FALSE;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    gz_RTKfilterInfo.pu_measUpdatePseudoNum[u_i] = 0;
    gz_RTKfilterInfo.pu_measUpdateCarrierNum[u_i] = 0;
  }

  gz_RTKfilterInfo.d_resetPercent = 0.0;
  gz_RTKfilterInfo.f_age = 0.0;
  gz_RTKfilterInfo.q_StationID = 0;
  for (u_i = 0; u_i < 3; u_i++)
  {
    gz_RTKfilterInfo.pd_StationCoordinate[u_i] = 0.0;
  }

  if (NULL != gz_RTKfilterInfo.pz_filterSol)
  {
    for (u_i = 0; u_i < MAX_FILTER_POS_STATUS; u_i++)
    {
      gz_RTKfilterInfo.pz_filterSol->pu_posValid[u_i] = RTK_FILTER_SOL_STAT_NONE;
      for (u_j = 0; u_j < 3; u_j++)
      {
        gz_RTKfilterInfo.pz_filterSol->pd_filterPos[u_i][u_j] = 0.0;
        gz_RTKfilterInfo.pz_filterSol->pd_filterVel[u_i][u_j] = 0.0;
      }
      for (u_j = 0; u_j < 6; u_j++)
      {
        gz_RTKfilterInfo.pz_filterSol->pd_filterPosQ[u_i][u_j] = 0.0;
      }
    }
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      gz_RTKfilterInfo.pd_phaseCodeClkDiff[u_i][u_j] = 0.0;
    }
  }

  cmn_epochUpdateMPdetectorInfo(&gz_rtkMPdetectFilter);

  return;
}
/**
 * @brief RTK diff age calculate
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]      pz_rtkCorrBlock is the VRS observation information
 * @return         0 represent normal and other abnormal
 */
static uint8_t RTK_diffAgeCal(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,rtk_filterInfo_t* pz_RTKfilterInfo, const GnssCorrBlock_t* pz_rtkCorrBlock)
{
  uint8_t u_status = 0;

  // update age of VRS
  if (pz_rtkCorrBlock->z_Clock.z_gpsTime.w_week > 0)
  {
    pz_RTKfilterInfo->f_age = (float)tm_GpsTimeDiff(&(pz_satSigMeasCollect->z_tor), &(pz_rtkCorrBlock->z_Clock.z_gpsTime));
    if (fabs(pz_RTKfilterInfo->f_age) > pz_RTKfilterInfo->z_opt.d_maxDiffAge && pz_RTKfilterInfo->z_opt.d_maxDiffAge > 1e-3)
    {
      u_status = 1;
      LOGI(TAG_HPP, "Diff Age error (age=%.1f)\n", pz_RTKfilterInfo->f_age);
    }
  }
  else
  {
    u_status = 1;
    pz_RTKfilterInfo->f_age = 999.f;
    LOGI(TAG_HPP, "Base Time not init\n");
  }
  
  return u_status;
}

/**
 * @brief RTK Main Process
 * @param[in]      pz_pvt_result is information of pvt
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in/out]  pz_satSigMeasCollectPre is the observation information of previous epoch
 * @param[out]     pz_PositionFix is RTK position Fix report
 * @return         true represent success
 */
BOOL hpp_Rtk_Process(const gnss_PVTResult_t* pz_pvt_result, 
  gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, 
  gnss_SatSigMeasCollect_t* pz_satSigMeasCollectPre, 
  gnss_PositionFix_t* pz_PositionFix)
{
  uint8_t u_common_sat = 0;
  uint8_t u_check = 0;
  uint8_t u_refIndex = 0;
  int32_t q_status = 0;
  uint8_t u_i = 0;
  uint8_t u_nv = 0;
  uint8_t u_diffStatus = 0;
  rtk_EpochFilterObs_t* pz_filterObs = NULL;
  gnss_fixedSignalAmbPool_t* pz_fixedRTKambPool = NULL;
  gnss_TdcpMeasBlock_t* pz_curDcpMeas = NULL;
  gnss_TdcpMeasPair_t z_tdcpMeasPointerPair = { NULL,NULL };
  gz_RTKfilterInfo.pz_pvtResult = pz_pvt_result;
  rtk_positiveDefiniteCheck(&gz_RTKfilterInfo, &gz_RTK_EKFstateRepPool);
  RTK_recordSceneInfo(pz_satSigMeasCollect);

  RTK_EpochFilterInfoInit(pz_satSigMeasCollect);

  /* print Debug time */
  RTK_printEpochAndCountMeas(pz_satSigMeasCollect, &gz_RTKfilterInfo);
  /* print measure */
  RTK_printMeasInfo(pz_satSigMeasCollect, &gz_RTKfilterInfo);

  RTK_savePVTinFilterPos(pz_pvt_result);
  gnss_fillUpObsMask(pz_pvt_result, pz_satSigMeasCollect);
  rtk_reCalculateCorrDataSatPosClkByRoverIode(pz_satSigMeasCollect, gpz_RtkCorrBlock[0]);
  
  gnss_SatSigMeasTimeJumpST(pz_satSigMeasCollect);
  u_common_sat = RTK_intersectionSatNumObsAndVRS(pz_satSigMeasCollect, gpz_RtkCorrBlock[0]);
  u_diffStatus = RTK_diffAgeCal(pz_satSigMeasCollect ,&gz_RTKfilterInfo, gpz_RtkCorrBlock[0]);
  if (((KF_INIT == (gz_RTKfilterInfo.z_kfStatus) || KF_RESET == (gz_RTKfilterInfo.z_kfStatus)
    || tm_GpsTimeDiff(&(pz_satSigMeasCollect->z_tor), &(gz_RTK_EKFstateRepPool.z_gpsTime)) > 3.0) && u_common_sat< 5)
    || GNSS_FIX_FLAG_INVALID == pz_pvt_result->z_positionFix.u_fixFlag || u_diffStatus)
  {
    LOGI(TAG_HPP, "RTK Start fail!\n");
    rtk_CreatePosSolution(pz_satSigMeasCollect, &gz_RTKfilterInfo, pz_fixedRTKambPool,
      pz_PositionFix, gz_RTKambFixInputInfo.pz_preFixedAmbPool[1]);
    return FALSE;
  }

  pz_filterObs = malloc_pz_RTKfilterObs(pz_satSigMeasCollect);

  /* use CA dynamic & history pos to predict current pos */
  rtk_getPredictPosByHistoryPos(&gz_RTKfilterInfo, pz_satSigMeasCollect->z_tor);
  /* Cycle slip && receiver clock jump */
  cmn_gfDetectCycleSlip(pz_satSigMeasCollect, &gz_RTKgfDetectFilter);
  pp_ClkJumpRepaire(pz_satSigMeasCollectPre, pz_satSigMeasCollect);
  cmn_mwDetectCycleSlip(pz_satSigMeasCollect, &gz_RTKmwDetectFilter);
  cmn_detectCycleSlipUsingMultiFreqObs(pz_satSigMeasCollect, &gz_RTKMultFreqDetectFilter);
  gz_RTKfilterInfo.d_deltaTimeDopplerDetect = cmn_dopplerDetectCycleSlip((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, &gz_dopplerDetectPairingBlockRTK);
  pz_curDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC_FAST(sizeof(gnss_TdcpMeasBlock_t));
  cmn_convertSigMeasToDcpStructNonSSR(pz_satSigMeasCollect, pz_curDcpMeas);
  z_tdcpMeasPointerPair.pz_curTdcpMeas = pz_curDcpMeas;
  z_tdcpMeasPointerPair.pz_preTdcpMeas = gpz_preTdcpMeasRTK;
  gz_RTKfilterInfo.u_tdcpMethodValid = cmn_detectCycleSlipByTDCP(z_tdcpMeasPointerPair, pz_satSigMeasCollect);
  /* multipath detect in tri-freq */
  cmn_detectMultiPath(pz_satSigMeasCollect, &gz_rtkMPdetectFilter, &(pz_filterObs->pz_MPflag), gu_sigMeasCount, &gz_RTKfilterInfo.u_threeFreqTag);
  /* ref station change check & update AMB */
  u_refIndex = 0;
  u_check = rtk_refChangeCheckAndUpdateAmb(gpz_RtkCorrBlock, &gz_RTKfilterInfo,
    &gz_RTK_EKFstateRepPool, pz_satSigMeasCollect,&gz_RTKcorrSlipFlag, gz_RTKambFixInputInfo.pz_preFixedAmbPool[0]);
  if(u_check == REF_AMB_CHANGE_POSTPONE)
  {
    u_refIndex = 1;
  }
  else if (u_check == REF_AMB_CHANGE_ERROR)
  {
    removeAMBEKFstatus(&gz_RTK_EKFstateRepPool, gz_RTKfilterInfo.w_nmax, gz_RTKfilterInfo.pd_X, 
      gz_RTKfilterInfo.pd_Q, gz_RTKfilterInfo.pq_paraValid);//reset AMB
  }

  /* Integrity prepare work */
  rtk_integ_start(gz_RTKfilterInfo.pz_integ, &gz_RTKfilterInfo, pz_pvt_result, u_common_sat);

  if (0 != RTD_solute(pz_pvt_result, pz_satSigMeasCollect, gpz_RtkCorrBlock[u_refIndex], gpz_RTDfilterInfo, &gz_RTD_EKFstateRepPool))
  {
    gz_RTKfilterInfo.q_QRcheckStatus = FALSE;
  }
  else
  {
    gz_RTKfilterInfo.q_QRcheckStatus = TRUE;
  }

  /* RTK float resolution */
  q_status = RTK_filterSolute((gnss_SatSigMeasCollect_t*)pz_satSigMeasCollect, gpz_RtkCorrBlock[u_refIndex],
    &gz_RTKfilterInfo, &gz_RTK_EKFstateRepPool, pz_filterObs, &gz_RTKcorrSlipFlag);
  *gpz_preTdcpMeasRTK = *pz_curDcpMeas;
  gpz_preTdcpMeasRTK->z_posSol = pz_satSigMeasCollect->z_positionFix;
  
  OS_FREE(pz_curDcpMeas);

  if (!q_status && 0 == u_refIndex)
  {
    /* save succ updated corr in float filter */
    rtk_refCorrSave();
  }
  if (rtk_filterResetCheck(&gz_RTKfilterInfo,&gz_RTK_EKFstateRepPool) && !q_status)
  {
    q_status = 1;
  }

  rtk_setAmbFixInputInfo(pz_satSigMeasCollect, pz_filterObs, gpz_RtkCorrBlock[u_refIndex], &gz_RTKfilterInfo, &gz_RTK_EKFstateRepPool, &gz_RTKambFixInputInfo);

  /* pre fix logic */
  u_nv = RTK_preFixSolute(pz_satSigMeasCollect, gpz_RtkCorrBlock[u_refIndex], &gz_RTKfilterInfo, &gz_RTK_EKFstateRepPool,
    gz_RTKambFixInputInfo.pz_preFixedAmbPool[0], pz_filterObs);
  
  /* clean detected half cycle flag */
  RTK_preFixCleanRejectLabel(pz_satSigMeasCollect);

  if (gz_RTKfilterInfo.u_prefixStatus)
  {
    RTK_calculateResidual(gz_RTKfilterInfo.pd_X, pz_satSigMeasCollect, gpz_RtkCorrBlock[u_refIndex], &gz_RTKfilterInfo,
                          &gz_RTK_EKFstateRepPool, gz_RTKambFixInputInfo.pz_preFixedAmbPool[0], pz_filterObs,
                          RES_PREFIX);

    /* use prefix to detect half cycle */
    if (u_nv > 6)
    {
      rtk_preFixSol_availabilityCheck(&gz_RTKambFixInputInfo, gz_RTKambFixInputInfo.pz_preFixedAmbPool[0], u_nv);

      RTK_preFixHalfCycleDetect(pz_satSigMeasCollect, gpz_RtkCorrBlock[u_refIndex], &gz_RTKfilterInfo, &gz_RTK_EKFstateRepPool,
        gz_RTKambFixInputInfo.pz_preFixedAmbPool[0]);
    }
  }

  /* RTK amb resolution */
  malloc_fixedSignalAmbPool(&pz_fixedRTKambPool, pz_satSigMeasCollect);
  if (!q_status && gz_RTKfilterInfo.pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_WARN)
  {
    RTK_AmbResolution(&gz_RTKambFixInputInfo, pz_fixedRTKambPool);
  }

  rtk_PreFixAmbCalculate(u_nv, pz_satSigMeasCollect, gpz_RtkCorrBlock[u_refIndex], &gz_RTKfilterInfo, &gz_RTK_EKFstateRepPool,
    gz_RTKambFixInputInfo.pz_preFixedAmbPool[0]);

  rtk_CreatePosSolution(pz_satSigMeasCollect, &gz_RTKfilterInfo, pz_fixedRTKambPool,
    pz_PositionFix, gz_RTKambFixInputInfo.pz_preFixedAmbPool[1]);

  /* Integrity Factor */
  if (rtk_integ_valid(gz_RTKfilterInfo.pz_integ))
  {
    if(!gz_RTKfilterInfo.q_filterPosValid)
    {
      rtk_integ_clearInfo(gz_RTKfilterInfo.pz_integ);
    }
    else
    {
      rtk_integ_save_KFSTD(&gz_RTK_EKFstateRepPool, pz_PositionFix, &gz_RTKfilterInfo);
      rtk_integ_save_statusInfo(&gz_RTKfilterInfo, pz_PositionFix, gz_RTKfilterInfo.pz_integ);
      integ_PLResult_t z_pl = {0};
      /* STD by DT model */
      rtk_integ_calSTDByDecisionTree(&gz_RTKfilterInfo, pz_PositionFix);
      /* Log PL STD feature  */
      gnss_log_ProtectionLevel(gz_RTKfilterInfo.u_solTag, &z_pl, pz_PositionFix);
      gnss_log_STD(gz_RTKfilterInfo.u_solTag, pz_PositionFix);
      rkt_integ_logFeature(&gz_RTKfilterInfo);
    }
  }

  free_pz_RTKFilterObs(&pz_filterObs);
  free_fixedSignalAmbPool(&pz_fixedRTKambPool);

  gnss_SatSigMeasCollect_Copy(pz_satSigMeasCollect, pz_satSigMeasCollectPre);
  pz_fixedRTKambPool = NULL;

  LOGW(TAG_HPP, "========================== hpp end filter ==========================\n");
  //LOGW(TAG_HPP, "\n");

  return TRUE;
}
