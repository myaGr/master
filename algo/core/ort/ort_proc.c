/**@file        ort_proc.c
 * @brief       Orientation Main Process module
 * @details
 * @author      chenjinhe
 * @date        2023/08/13
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/13  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include <math.h>
#include "ort_proc.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "cmn_utils.h"
#include "rtk_pos_check.h"
#include "gnss_common.h"
#include "ort_type.h"
#include "gnss_prep.h"
#include "gnss_tdcp_detectCS.h"
#include "rtk_filter_sol.h"
#include "rtk_pre_fix.h"
#include "cmn_CSmask_combine.h"
#include "rtd_sol.h"
#include "rtk_amb_fix.h"
#include "rtk_integrity.h"

#define AUXI_ANT_MAX_BLOCK             (2)                                              /*the maximum number of auxiliary antenna block, default > 1*/
#define MAIN_ANT_MAX_BLOCK             (2)                                              /*the maximum number of main antenna block*/
#define ORT_OBS_POS_TIME_DIFF_THRES    (0.001)                                          /*the maximum difference between main antenna measure time and postion result*/
 
BOOL gz_isMemoryAllocSucc = TRUE;
/*used for Orientation float solute*/
rtk_filterInfo_t                       gz_ortFilterInfo;
gnss_EKFstateRepresentPool_t           gz_ORT_EKFstateRepPool;
GnssMeasBlock_t*                       gpz_mainMeasBlock[MAIN_ANT_MAX_BLOCK] = { NULL };/*[0] saves latest valid main antenna measure block,
                                                                                          [1] & [1+] save backward main antenna measure block*/
GnssCorrBlock_t*                       gpz_auxiMeasBlock[AUXI_ANT_MAX_BLOCK] = { NULL };/*[0] saves latest valid auxiliary antenna measure block,
                                                                                          [1] & [1+] save backward auxiliary antenna measure block*/
ort_cycleSlipDetectObj                 gz_mainAntSlipDetectObj;                         /*the cycle slip detect object of main antenna measure*/
ort_cycleSlipDetectObj                 gz_auxiAntSlipDetectObj;                         /*the cycle slip detect object of auxiliary antenna measure*/
GnssCorrSlipFlag_t                     gz_auxiCorrSlipFlag;                             /*the cycle slip flag for the  auxiliary antenna measure*/

gnss_SatSigMeasCollect_t               gz_mainSatSigMeasCollectPre = { 0 };
gnss_SatSigMeasCollect_t               gz_auxiSatSigMeasCollectPre = { 0 };

/*used for RTD solute*/
rtd_filterInfo_t*                      gpz_ortRTDfilterInfo;
rtd_EKFstateRepresentPool_t            gz_ort_RTD_EKFstateRepPool;

gnss_rtkAmbFixInputInfo_t              gz_ortAmbFixInputInfo;

gnss_PositionFix_t*                    gpz_positionFix = NULL;

/* rtk integrity, call ver1.0`s uncertainty */
integ_RtkIntegrity_t*                  gpz_baseLineIntegrity = NULL;

gnss_PVTResult_t* gpz_mainPvtInfo[MAIN_ANT_MAX_BLOCK] = { NULL };                       /*[0] saves latest valid main antenna pvt result,
                                                                                          [1] & [1+] save backward main antenna pvt result*/

GpsTime_t gz_ortLastCalTime = { 0 };

/**
 *@brief load the default option for the Orientation algorithm
 * @param[out]  pz_opt represent the algorithm optimization for the Orientation
 * @return     void
 */
void ort_loadDefaultOption(rtk_alg_opt_t* pz_opt)
{
  pz_opt->z_usedSys = (ALGO_GPS_SYS | ALGO_BDS_SYS | ALGO_GLO_SYS | ALGO_GAL_SYS | ALGO_QZS_SYS);
  pz_opt->z_usedFreq = (ALGO_L1_FREQ | ALGO_L2_FREQ | ALGO_L5_FREQ);
  pz_opt->q_isSperateBDS2And3 = TRUE;
  pz_opt->d_elmin = 7.0 * DEG2RAD;
  pz_opt->d_maxinno = 25.0;
  return;
}
 /**
  * @brief Initialze Orientation module
  * @param[in]  pz_opt represent the algorithm option for the Orientation
  * @return     status -- 0: initilize success, other: fail
  */
uint8_t ort_init(const rtk_alg_opt_t* pz_opt)
{
  uint8_t u_status = 0;
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  memset(&gz_ortLastCalTime, 0, sizeof(GpsTime_t));
  //initilize the information of RTK filter
  gz_ortFilterInfo.w_n = 0;
  gz_ortFilterInfo.u_start = 0;
  gz_ortFilterInfo.w_nmax = GNSS_MAX_FILTER_STATE_NUM;
  gz_ortFilterInfo.pd_deltaX = (double*)OS_MALLOC(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  gz_ortFilterInfo.pd_X = (double*)OS_MALLOC(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  gz_ortFilterInfo.pd_Q = (double*)OS_MALLOC(NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
  gz_ortFilterInfo.pz_filterSol = (rtk_filter_sol_t*)OS_MALLOC(sizeof(rtk_filter_sol_t));
  gz_ortFilterInfo.pq_paraValid = (BOOL*)OS_MALLOC(GNSS_MAX_FILTER_STATE_NUM * sizeof(BOOL));
  gz_ortFilterInfo.pz_rcvInitClkInfo = (rtk_rcv_init_clk_info_t*)OS_MALLOC(sizeof(rtk_rcv_init_clk_info_t));
  gz_ortFilterInfo.pz_historyPos_t = (rtk_history_pos_t*)OS_MALLOC(sizeof(rtk_history_pos_t));
  gpz_ortRTDfilterInfo = (rtd_filterInfo_t*)OS_MALLOC(sizeof(rtd_filterInfo_t));
  gpz_positionFix = (gnss_PositionFix_t*)OS_MALLOC(sizeof(gnss_PositionFix_t));
  if (any_Ptrs_Null(9, gz_ortFilterInfo.pd_deltaX, gz_ortFilterInfo.pd_X,
    gz_ortFilterInfo.pd_Q, gz_ortFilterInfo.pz_filterSol, gz_ortFilterInfo.pq_paraValid, gz_ortFilterInfo.pz_rcvInitClkInfo, gz_ortFilterInfo.pz_historyPos_t, gpz_ortRTDfilterInfo, gpz_positionFix))
  {
    LOGW(TAG_HPP, "ort_init X or Q malloc failed");
    OS_FREE(gz_ortFilterInfo.pd_deltaX);
    OS_FREE(gz_ortFilterInfo.pd_X);
    OS_FREE(gz_ortFilterInfo.pd_Q);
    OS_FREE(gz_ortFilterInfo.pz_filterSol);
    OS_FREE(gz_ortFilterInfo.pq_paraValid);
    OS_FREE(gz_ortFilterInfo.pz_rcvInitClkInfo);
    OS_FREE(gz_ortFilterInfo.pz_historyPos_t);
    OS_FREE(gpz_ortRTDfilterInfo);
    OS_FREE(gpz_positionFix);
    u_status = 1;
  }
  else
  {
    //initilize the information of EKF status
    tm_initGpsTime(&(gz_ORT_EKFstateRepPool.z_gpsTime));
    gz_ORT_EKFstateRepPool.d_continueFilterTime = 0.0;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM && !u_status; ++w_i)
    {
      gz_ORT_EKFstateRepPool.pz_satPool[w_i] = (gnss_EKFstateRepresent_t*)OS_MALLOC(1 * sizeof(gnss_EKFstateRepresent_t));
      if (NULL == gz_ORT_EKFstateRepPool.pz_satPool[w_i])
      {
        u_status = 2;
        break;
      }
      gz_ORT_EKFstateRepPool.pz_satPool[w_i]->w_index = INVALID_INDEX;
      initEKFstateRepresent(gz_ORT_EKFstateRepPool.pz_satPool[w_i], gz_ortFilterInfo.w_nmax, gz_ortFilterInfo.pd_X,
        gz_ortFilterInfo.pd_Q);
    }

    for (w_i = 0; w_i < MAX_FILTER_POS_STATUS; w_i++)
    {
      gz_ortFilterInfo.pz_filterSol->pq_diffFlagWithSPP[w_i] = 0x0;
      gz_ortFilterInfo.pz_filterSol->pq_diffFlagWithPredict[w_i] = 0x0;
    }
    rtk_initHistoryPos(gz_ortFilterInfo.pz_historyPos_t);

    gz_ortFilterInfo.z_kfStatus = KF_INIT;
  }

  //option
  if (NULL != pz_opt)
  {
    gz_ortFilterInfo.z_opt = *(pz_opt);
  }
  else
  {
    ort_loadDefaultOption(&(gz_ortFilterInfo.z_opt));
  }
  gz_ortFilterInfo.z_opt.b_enableStaticConstraints = FALSE;
  gz_ortFilterInfo.z_opt.b_enableL1onlyFix = TRUE;
  if (NULL != gz_ortFilterInfo.pq_paraValid)
  {
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
    {
      gz_ortFilterInfo.pq_paraValid[w_i] = FALSE;
    }
  }
  //gz_ortFilterInfo.f_sample = f_sample;
  gz_ortFilterInfo.f_sample = 1;
  gz_ortFilterInfo.t_openSkyCount = 0;
  gz_ortFilterInfo.t_closeSkyCount = 0;
  gz_ortFilterInfo.u_goodSceneStatusFlag = 0;
  gz_ortFilterInfo.pz_pvtResult = NULL;
  //initilize the auxiliary antenna measure block
  for (w_i = 0; w_i < AUXI_ANT_MAX_BLOCK; ++w_i)
  {
    gpz_auxiMeasBlock[w_i] = (GnssCorrBlock_t*)OS_MALLOC(sizeof(GnssCorrBlock_t));
    if (NULL == gpz_auxiMeasBlock[w_i])
    {
      u_status = 3;
      break;
    }
  }
  //initilize the main antenna measure block
  for (w_i = 0; w_i < MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    gpz_mainMeasBlock[w_i] = (GnssMeasBlock_t*)OS_MALLOC(sizeof(GnssMeasBlock_t));
    gpz_mainPvtInfo[w_i] = (gnss_PVTResult_t*)OS_MALLOC(sizeof(gnss_PVTResult_t));
    if (NULL == gpz_mainMeasBlock[w_i] || NULL== gpz_mainPvtInfo[w_i])
    {
      u_status = 4;
      break;
    }
  }
  ort_initCycleSlipDetectObject(&gz_mainAntSlipDetectObj);
  ort_initCycleSlipDetectObject(&gz_auxiAntSlipDetectObj);
  gz_auxiCorrSlipFlag.u_changeStationFlag = 0;
  for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    for (w_j = 0; w_j < MAX_GNSS_SIGNAL_FREQ; ++w_j)
    {
      gz_auxiCorrSlipFlag.u_slip[w_i][w_j] = 0;
    }
  }
  rtd_init(&(gz_ortFilterInfo.z_opt), gpz_ortRTDfilterInfo, &gz_ort_RTD_EKFstateRepPool);
  if (0 != u_status)
  {
    gz_isMemoryAllocSucc = FALSE;
  }
  gpz_baseLineIntegrity = rtk_integ_init();
  gz_ortFilterInfo.pz_integ = gpz_baseLineIntegrity;
  memset(&gz_ortFilterInfo.z_timeLastHold, 0, sizeof(GpsTime_t));
  gz_ortFilterInfo.b_staticFlag = FALSE;
  return u_status;
}
void ort_DeinitAmbInfo()
{
  uint8_t u_i = 0;
  for (u_i = 0; u_i < GNSS_MAX_AMB_FIXED_TYPE; ++u_i)
  {
    free_fixedSignalAmbPool(&gz_ortAmbFixInputInfo.pz_preFixedAmbPool[u_i]);
  }
}
/**
 * @brief deinitilize the Orientation algorithm
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void ort_deInit()
{
  uint16_t w_i = 0;
  gz_ortFilterInfo.w_n = 0;
  gz_ortFilterInfo.w_nmax = 0;
  gz_ortFilterInfo.u_start = 0;
  OS_FREE(gz_ortFilterInfo.pd_deltaX);
  OS_FREE(gz_ortFilterInfo.pd_X);
  OS_FREE(gz_ortFilterInfo.pd_Q);
  OS_FREE(gz_ortFilterInfo.pq_paraValid);
  OS_FREE(gz_ortFilterInfo.pz_filterSol);
  OS_FREE(gz_ortFilterInfo.pz_rcvInitClkInfo);
  OS_FREE(gz_ortFilterInfo.pz_historyPos_t);
  //deinitilize the information of EKF status
  tm_initGpsTime(&(gz_ORT_EKFstateRepPool.z_gpsTime));
  gz_ORT_EKFstateRepPool.d_continueFilterTime = 0.0;
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    OS_FREE(gz_ORT_EKFstateRepPool.pz_satPool[w_i]);
  }
  for (w_i = 0; w_i < AUXI_ANT_MAX_BLOCK; ++w_i)
  {
    OS_FREE(gpz_auxiMeasBlock[w_i]);
  }
  for (w_i = 0; w_i < MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    OS_FREE(gpz_mainMeasBlock[w_i]);
  }
  ort_deInitCycleSlipDetectObject(&gz_mainAntSlipDetectObj);
  ort_deInitCycleSlipDetectObject(&gz_auxiAntSlipDetectObj);
  gnss_SatSigMeasCollect_Clear(&gz_mainSatSigMeasCollectPre);
  gnss_SatSigMeasCollect_Clear(&gz_auxiSatSigMeasCollectPre);

  rtd_deinit(gpz_ortRTDfilterInfo, &gz_ort_RTD_EKFstateRepPool);
  ort_DeinitAmbInfo();
  rtk_integ_deinit(gpz_baseLineIntegrity);
  gz_ortFilterInfo.pz_integ = NULL;
  return;
}
/**
 * @brief convert struct from GnssMeasBlock_t to GnssCorrBlock_t
 * @param[in]  pz_measBlock is the GNSS measure block
 * @param[out] pz_corrBlock is the GNSS correction block
 * @return     void
 */
void ort_convertMeasToCorrBlock(const GnssMeasBlock_t* pz_measBlock, GnssCorrBlock_t* pz_corrBlock)
{
  uint16_t w_i = 0;
  pz_corrBlock->u_version = pz_measBlock->u_version;
  pz_corrBlock->w_size = sizeof(GnssCorrBlock_t);
  pz_corrBlock->w_refStationId = 0;
  pz_corrBlock->w_switchStationId = 0;
  for (w_i = 0; w_i < 3; ++w_i)
  {
    pz_corrBlock->d_refPosEcef[w_i] = 0.0;
  }
  pz_corrBlock->z_Clock = pz_measBlock->z_Clock;
  pz_corrBlock->z_Clock.d_clockBias = 0.0;
  pz_corrBlock->z_Clock.d_clockDrift = 0.0;
  pz_corrBlock->w_measCount = pz_measBlock->w_measCount;
  pz_corrBlock->w_satCount = 0;
  memcpy(pz_corrBlock->z_meas, pz_measBlock->z_meas, MAX_GNSS_TRK_MEAS_NUMBER * sizeof(GnssMeas_t));
  for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    pz_corrBlock->d_trop[w_i] = 0.0;
  }
  for (w_i = 0; w_i < MAX_GNSS_ACTIVE_SAT_NUMBER; ++w_i)
  {
    memset(&(pz_corrBlock->pz_satPosClk[w_i]), 0, sizeof(gnss_SatPosClkInfo_t));
  }
  /* Fill week into measurement */
  gnss_AdjustMeasBlockWeek(pz_corrBlock->w_measCount, pz_corrBlock->z_meas, &pz_corrBlock->z_Clock.z_gpsTime);
  return;
}
/**
 * @brief push the main antenna measure block
 * @param[in]  pz_mainMeasBlock is the measure of main antenna
 * @param[out] void
 * @return     void
 */
void ort_pushMainAntMeas(const GnssMeasBlock_t* pz_mainMeasBlock)
{
  uint8_t u_i = 0;
  if (TRUE == gz_isMemoryAllocSucc)
  {
    for (u_i = MAIN_ANT_MAX_BLOCK - 1; u_i >= 1; --u_i)
    {
      memcpy(gpz_mainMeasBlock[u_i], gpz_mainMeasBlock[u_i - 1], sizeof(GnssMeasBlock_t));
    }
    memcpy(gpz_mainMeasBlock[0], pz_mainMeasBlock, sizeof(GnssMeasBlock_t));
    /* Fill week into measurement */
    gnss_AdjustMeasBlockWeek(gpz_mainMeasBlock[0]->w_measCount, gpz_mainMeasBlock[0]->z_meas, &(gpz_mainMeasBlock[0]->z_Clock.z_gpsTime));
  }
  return;
}
/**
 * @brief push the auxiliary antenna measure block
 * @param[in]  pz_auxiMeasBlock is the measure of auxiliary antenna
 * @param[out] void
 * @return     void
 */
void ort_pushAuxiAntMeas(const GnssMeasBlock_t* pz_auxiMeasBlock)
{
  uint8_t u_i = 0;
  if (TRUE == gz_isMemoryAllocSucc)
  {
    for (u_i = AUXI_ANT_MAX_BLOCK - 1; u_i >= 1; --u_i)
    {
      memcpy(gpz_auxiMeasBlock[u_i], gpz_auxiMeasBlock[u_i - 1], sizeof(GnssCorrBlock_t));
    }
    ort_convertMeasToCorrBlock(pz_auxiMeasBlock, gpz_auxiMeasBlock[0]);
  }
  return;
}
/**
 * @brief push the main antenna position
 * @param[in]  pz_pvtInfo is the position result of main antenna
 * @param[out] void
 * @return     void
 */
void ort_pushMainAntPosition(const gnss_PVTResult_t* pz_pvtInfo)
{
  uint8_t u_i = 0;
  if (TRUE == gz_isMemoryAllocSucc)
  {
    for (u_i = MAIN_ANT_MAX_BLOCK - 1; u_i >= 1; --u_i)
    {
      memcpy(gpz_mainPvtInfo[u_i], gpz_mainPvtInfo[u_i - 1], sizeof(gnss_PVTResult_t));
    }
    memcpy(gpz_mainPvtInfo[0], pz_pvtInfo, sizeof(gnss_PVTResult_t));
  }
  return;
}
/**
 * @brief get the index of main antenna measure block according to PVT time
 * @param[in]  pz_pvtTime is the PVT time of main antenna
 * @return     the index of main antenna measure block, -1 represent matched failure
 */
int16_t ort_getMainAntMeasIndexAccordPvtInfo(const GpsTime_t* pz_pvtTime)
{
  int16_t w_index = -1;
  int16_t w_i = 0;
  double d_timeDiff = -1.0;
  for (w_i = 0; w_i < MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    d_timeDiff = tm_GpsTimeDiff(pz_pvtTime, &(gpz_mainMeasBlock[w_i]->z_Clock.z_gpsTime));
    if (fabs(d_timeDiff) < ORT_OBS_POS_TIME_DIFF_THRES)
    {
      w_index = w_i;
      break;
    }
  }
  return w_index;
}
/**
 * @brief get the index of main antenna pvt result according to observation time
 * @param[in]  pz_obsTime is the observation time of main antenna
 * @return     the index of pvt result, -1 represent matched failure
 */
int16_t ort_getMainAntPvtAccordTime(const GpsTime_t* pz_obsTime)
{
  int16_t w_index = -1;
  int16_t w_i = 0;
  double d_timeDiff = -1.0;
  for (w_i = 0; w_i < MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    d_timeDiff = tm_GpsTimeDiff(pz_obsTime, &(gpz_mainPvtInfo[w_i]->z_positionFix.z_gpsTime));
    if (fabs(d_timeDiff) < ORT_OBS_POS_TIME_DIFF_THRES)
    {
      w_index = w_i;
      break;
    }
  }
  return w_index;
}
/**
 * @brief get the index of auxiliary antenna measure block according to main antenna measure time
 * @param[in]  pz_mainAntTime is the main antenna
 * @return     the index of auxiliary antenna measure block, -1 represent matched failure
 */
int16_t ort_getAuxiliaryAntMeasIndexAccordPvtInfo(const GpsTime_t* pz_mainAntTime)
{
  int16_t w_index = -1;
  int16_t w_i = 0;
  double d_timeDiff = -1.0;
  double d_minTimeDiff = 0.5;
  for (w_i = 0; w_i < AUXI_ANT_MAX_BLOCK; ++w_i)
  {
    d_timeDiff = tm_GpsTimeDiff(pz_mainAntTime, &(gpz_auxiMeasBlock[w_i]->z_Clock.z_gpsTime));
    if (fabs(d_timeDiff) < d_minTimeDiff)
    {
      w_index = w_i;
      d_minTimeDiff = fabs(d_timeDiff);
    }
  }
  if (d_minTimeDiff > ORT_MAIN_AUXI_TIME_DIFF_THRES)
  {
    w_index = -1;
  }
  return w_index;
}
/**
 * @brief calcluate the position of auxiliary antenna
 * @param[in]  pz_pvtInfo is the PVT information of main antenna
 * @param[out] pz_auxiMeas is the auxiliary antenna measure block
 * @return     0 represent success and other failure
 */
uint8_t ort_fillUpAuxiliaryAntPos(const gnss_PVTResult_t* pz_pvtInfo, GnssCorrBlock_t* pz_auxiMeas)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  //use the position of main antenna to fill up the position of auxiliary antenna
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_auxiMeas->d_refPosEcef[u_i] = pz_pvtInfo->z_positionFix.d_xyz[u_i];
  }
  if (GNSS_FIX_FLAG_INVALID == (pz_pvtInfo->z_positionFix.u_fixFlag))
  {
    u_status = 1;
  }
  return u_status;
}
/**
 * @brief mask the cycle slip of auxiliary antenna measure
 * @param[in]  pz_corrMeasCollect is the auxiliary antenna satellite signal collect
 * @param[out] pz_auxiCorrSlipFlag is the cycle slip flag of auxiliary antenna measure
 * @return     void
 */
void ort_maskAuxiAntCycleSlip(const gnss_SatSigMeasCollect_t* pz_corrMeasCollect, GnssCorrSlipFlag_t* pz_auxiCorrSlipFlag)
{
  uint32_t q_satIndex = 0;
  uint8_t u_sigNum = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_cycleSlipMask = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_corrMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_sigNum = RTK_signalNum(pz_satMeas);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      u_cycleSlipMask = cmn_combineCycleSlipMask(gz_auxiAntSlipDetectObj.u_tdcpMethodValid, u_sigNum,
        pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, gz_auxiAntSlipDetectObj.d_deltaTimeDopplerDetect, 0, pz_sigMeas->u_slipMethodValid, NULL);
      if (1 == u_cycleSlipMask)
      {
        pz_auxiCorrSlipFlag->u_slip[q_satIndex][u_signalIndex] = HAS_CORR_SLIP_BY_TD;//to match the correction data cycle slip flag in RTK, there set the auxiliary antenna cycle slip mask to HAS_CORR_SLIP_BY_TD
      }
    }
  }
  return;
}
/**
 * @brief gain the median value of doppler observations minus pseudo-range observations for target frequency and target constellation
 * @param[in]      e_TargetSysType is the target constellation
 * @param[in]      z_targetFreq is the target frequency
 * @param[in]      pz_curSatSigMeasCollect is the current epoch of auxiliary antenna measure block
 * @param[in]      pz_preSatSigMeasCollect is the previous epoch of auxiliary antenna measure block
 * @param[out]     pf_corseRcvDrift,the result of corse receiver clock drift
 * @return         TRUE represent success and False represent failure
 */
BOOL ort_gainMedianAuxiAntDopplerMinusPseudo(gnss_ConstellationType e_TargetSysType, gnss_FreqType z_targetFreq,
  const gnss_SatSigMeasCollect_t* pz_curSatSigMeasCollect, const gnss_SatSigMeasCollect_t* pz_preSatSigMeasCollect, float* pf_corseRcvDrift)
{
  BOOL q_isSuccess = FALSE;
  uint32_t q_satIndex = 0;
  uint8_t u_constellation = 0;
  uint8_t u_obsNum = 0;
  float f_avgDoppler = 0.0;
  float f_deltaTime = 0.0;
  double d_distVar = 0.0;
  double d_distVarPseudo = 0.0;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  const gnss_SatelliteMeas_t* pz_curSatMeas = NULL;
  const gnss_SignalMeas_t* pz_curSigMeas = NULL;
  const gnss_SatelliteMeas_t* pz_preSatMeas = NULL;
  const gnss_SignalMeas_t* pz_preSigMeas = NULL;
  *pf_corseRcvDrift = 0.0;
  f_deltaTime = (float)(tm_GpsTimeDiff(&(pz_curSatSigMeasCollect->z_tor), &(pz_preSatSigMeasCollect->z_tor)));
  if (z_targetFreq < MAX_GNSS_SIGNAL_FREQ)
  {
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      pz_curSatMeas = pz_curSatSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_curSatMeas)
      {
        continue;
      }
      pz_curSigMeas = pz_curSatMeas->pz_signalMeas[z_targetFreq];
      if (NULL == pz_curSigMeas)
      {
        continue;
      }
      gnss_cvt_SvIndex2Svid((uint8_t)q_satIndex, &u_constellation);
      if (u_constellation != e_TargetSysType)
      {
        continue;
      }
      pz_preSatMeas= pz_preSatSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_preSatMeas)
      {
        continue;
      }
      pz_preSigMeas = pz_preSatMeas->pz_signalMeas[z_targetFreq];
      if (NULL == pz_preSigMeas)
      {
        continue;
      }
      if ((pz_curSigMeas->d_pseudoRange) < 1.0e-3 || (pz_preSigMeas->d_pseudoRange) < 1.0e-3)
      {
        continue;
      }
      if (fabs(pz_curSigMeas->d_doppler) < 1.0e-3 && fabs(pz_preSigMeas->d_doppler) < 1.0e-3)
      {
        continue;
      }
      if (fabs(pz_curSigMeas->d_doppler) > 0.0 && fabs(pz_preSigMeas->d_doppler) > 0.0)
      {
        f_avgDoppler = (float)((pz_curSigMeas->d_doppler + pz_preSigMeas->d_doppler) * 0.5);
      }
      else if (fabs(pz_curSigMeas->d_doppler) > 0.0)
      {
        f_avgDoppler = (float)(pz_curSigMeas->d_doppler);
      }
      else if (fabs(pz_preSigMeas->d_doppler) > 0.0)
      {
        f_avgDoppler = (float)(pz_preSigMeas->d_doppler);
      }
      d_distVar = ((double)f_avgDoppler * f_deltaTime);
      d_distVarPseudo = (pz_curSigMeas->d_pseudoRange - pz_preSigMeas->d_pseudoRange);
      pf_diff[u_obsNum] = (float)(d_distVar - d_distVarPseudo);
      ++u_obsNum;
    }
  }
  if (gnss_ascSortMedianFloat(pf_diff, (uint32_t)u_obsNum, pf_corseRcvDrift))
  {
    q_isSuccess = TRUE;
  }
  return q_isSuccess;
}
/**
 * @brief gain the median value of doppler observations minus pseudo-range observations for target frequency and target constellation
 * @param[in]      e_TargetSysType is the target constellation
 * @param[in]      z_targetFreq is the target frequency
 * @param[in/out]  pz_curSatSigMeasCollect is the current epoch of auxiliary antenna measure block
 * @param[in]      pz_preSatSigMeasCollect is the previous epoch of auxiliary antenna measure block
 * @param[in]      f_corseRcvDrift,the corse receiver clock drift
 * @return         void
 */
void ort_markAuxiAntTargetFreqSysByrDrConsistCheck(gnss_ConstellationType e_TargetSysType, gnss_FreqType z_targetFreq,
  gnss_SatSigMeasCollect_t* pz_curSatSigMeasCollect, const gnss_SatSigMeasCollect_t* pz_preSatSigMeasCollect, float f_corseRcvDrift)
{
  uint32_t q_satIndex = 0;
  uint8_t u_constellation = 0;
  float f_avgDoppler = 0.0;
  float f_deltaTime = 0.0;
  double d_distVar = 0.0;
  double d_distVarPseudo = 0.0;
  float f_diff = 0.0;
  gnss_SatelliteMeas_t* pz_curSatMeas = NULL;
  gnss_SignalMeas_t* pz_curSigMeas = NULL;
  const gnss_SatelliteMeas_t* pz_preSatMeas = NULL;
  const gnss_SignalMeas_t* pz_preSigMeas = NULL;
  f_deltaTime = (float)(tm_GpsTimeDiff(&(pz_curSatSigMeasCollect->z_tor), &(pz_preSatSigMeasCollect->z_tor)));
  if (z_targetFreq < MAX_GNSS_SIGNAL_FREQ)
  {
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      pz_curSatMeas = pz_curSatSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_curSatMeas)
      {
        continue;
      }
      pz_curSigMeas = pz_curSatMeas->pz_signalMeas[z_targetFreq];
      if (NULL == pz_curSigMeas)
      {
        continue;
      }
      gnss_cvt_SvIndex2Svid((uint8_t)q_satIndex, &u_constellation);
      if (u_constellation != e_TargetSysType)
      {
        continue;
      }
      pz_preSatMeas = pz_preSatSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_preSatMeas)
      {
        continue;
      }
      pz_preSigMeas = pz_preSatMeas->pz_signalMeas[z_targetFreq];
      if (NULL == pz_preSigMeas)
      {
        continue;
      }
      if ((pz_curSigMeas->d_pseudoRange) < 1.0e-3 || (pz_preSigMeas->d_pseudoRange) < 1.0e-3)
      {
        continue;
      }
      if (fabs(pz_curSigMeas->d_doppler) < 1.0e-3 && fabs(pz_preSigMeas->d_doppler) < 1.0e-3)
      {
        continue;
      }
      if (fabs(pz_curSigMeas->d_doppler) > 0.0 && fabs(pz_preSigMeas->d_doppler) > 0.0)
      {
        f_avgDoppler = (float)((pz_curSigMeas->d_doppler + pz_preSigMeas->d_doppler) * 0.5);
      }
      else if (fabs(pz_curSigMeas->d_doppler) > 0.0)
      {
        f_avgDoppler = (float)(pz_curSigMeas->d_doppler);
      }
      else if (fabs(pz_preSigMeas->d_doppler) > 0.0)
      {
        f_avgDoppler = (float)(pz_preSigMeas->d_doppler);
      }
      d_distVar = ((double)f_avgDoppler * f_deltaTime);
      d_distVarPseudo = (pz_curSigMeas->d_pseudoRange - pz_preSigMeas->d_pseudoRange);
      f_diff = (float)(d_distVar - d_distVarPseudo- f_corseRcvDrift);
      if (fabs(f_diff) > 10.0)
      {
        pz_curSigMeas->z_measStatusFlag.b_prValid = 0;
      }
    }
  }
  return;
}
/**
 * @brief check the consistence of target frequency and target constellation's pseudo-range and doppler for the auxiliary antenna measure block
 * @param[in]      e_TargetSysType is the target constellation
 * @param[in]      z_targetFreq is the target frequency
 * @param[in/out]  pz_curSatSigMeasCollect is the current epoch of auxiliary antenna measure block
 * @param[in]      pz_preSatSigMeasCollect is the previous epoch of auxiliary antenna measure block
 * @return         void
 */
void ort_auxiAntTargetFreqSysPrDrConsistCheck(gnss_ConstellationType e_TargetSysType, gnss_FreqType z_targetFreq,
  gnss_SatSigMeasCollect_t* pz_curSatSigMeasCollect, const gnss_SatSigMeasCollect_t* pz_preSatSigMeasCollect)
{
  float f_corseRcvDrift = 0.0;
  if (TRUE == ort_gainMedianAuxiAntDopplerMinusPseudo(e_TargetSysType, z_targetFreq, pz_curSatSigMeasCollect, pz_preSatSigMeasCollect, &f_corseRcvDrift))
  {
    ort_markAuxiAntTargetFreqSysByrDrConsistCheck(e_TargetSysType, z_targetFreq, pz_curSatSigMeasCollect, pz_preSatSigMeasCollect, f_corseRcvDrift);
  }
  return;
}
/**
 * @brief check the consistence of pseudo-range and doppler for the auxiliary antenna measure block
 * @param[in/out]  pz_curSatSigMeasCollect is the current epoch of auxiliary antenna measure block
 * @param[in]      pz_preSatSigMeasCollect is the previous epoch of auxiliary antenna measure block
 * @return         void
 */
void ort_auxiAntPseudoDopplerConsistCheck(gnss_SatSigMeasCollect_t* pz_curSatSigMeasCollect, const gnss_SatSigMeasCollect_t* pz_preSatSigMeasCollect)
{
  uint8_t u_constellation = 0;
  uint8_t u_iFreq = 0;
  double d_deltaTime = 999.9;
  d_deltaTime = tm_GpsTimeDiff(&(pz_curSatSigMeasCollect->z_tor), &(pz_preSatSigMeasCollect->z_tor));
  if (fabs(d_deltaTime) < 10.0)
  {
    for (u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_MAX; ++u_constellation)
    {
      for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
      {
        ort_auxiAntTargetFreqSysPrDrConsistCheck((gnss_ConstellationType)u_constellation, (gnss_FreqType)u_iFreq, pz_curSatSigMeasCollect, pz_preSatSigMeasCollect);
      }
    }
  }
  return;
}
/**
 * @brief deal with the gross error in pseudo-range of auxiliary antenna
 * @param[in/out]  pz_curSatSigMeasCollect is the current epoch of auxiliary antenna measure block
 * @param[out]     pz_preSatSigMeasCollect is the previous epoch of auxiliary antenna measure block
 * @return         void
 */
void ort_dealWithAuxiAntPseudoRange(gnss_SatSigMeasCollect_t* pz_curSatSigMeasCollect, const gnss_SatSigMeasCollect_t* pz_preSatSigMeasCollect)
{
  if (NULL != pz_curSatSigMeasCollect && NULL != pz_preSatSigMeasCollect)
  {
    ort_auxiAntPseudoDopplerConsistCheck(pz_curSatSigMeasCollect, pz_preSatSigMeasCollect);
  }
  return;
}
/**
 * @brief find the index of observations in the GNSS corrections block
 * @param[in]      u_constellation is the constellation type
 * @param[in]      u_prn is prn
 * @param[in]      u_targetFreqType is the target frequency type
 * @param[in]      pz_auxiMeasBlock is the current epoch of auxiliary antenna measure represent by GNSS correction block
 * @return         the index of observations in the GNSS corrections block, -1 represent gained index failure
 */
int16_t ort_findObsIndexInGnssCorrBlock(uint8_t u_constellation, uint8_t u_prn, uint8_t u_targetFreqType, const GnssCorrBlock_t* pz_auxiMeasBlock)
{
  int16_t w_index = -1;
  uint16_t w_iMeas = 0;
  uint8_t u_freqType = 0;
  const GnssMeas_t* pz_meas = NULL;
  for (w_iMeas = 0; w_iMeas < (pz_auxiMeasBlock->w_measCount); ++w_iMeas)
  {
    pz_meas = &(pz_auxiMeasBlock->z_meas[w_iMeas]);
    if (u_constellation != (pz_meas->u_constellation) || u_prn != (pz_meas->u_svid))
    {
      continue;
    }
    u_freqType = gnss_Signal2FreqIdx(pz_meas->u_signal);
    if (C_GNSS_FREQ_TYPE_MAX == u_freqType)
    {
      continue;
    }
    if (u_freqType == u_targetFreqType)
    {
      w_index = (int16_t)w_iMeas;
      break;
    }
  }
  return w_index;
}
/**
 * @brief copy the mask and observations to the GNSS correction block of auxiliary antenna
 * @param[in]      pz_corrMeasCollect is the current epoch of auxiliary antenna measure block
 * @param[out]     pz_auxiMeasBlock is the current epoch of auxiliary antenna measure represent by GNSS correction block
 * @return         void
 */
void ort_copyAuxiAntMaskObsToGnssCorrBlock(const gnss_SatSigMeasCollect_t* pz_corrMeasCollect, GnssCorrBlock_t* pz_auxiMeasBlock)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_constellation = 0;
  uint8_t u_prn = 0;
  int16_t w_index = -1;
  const gnss_SatelliteMeas_t* pz_curSatMeas = NULL;
  const gnss_SignalMeas_t* pz_curSigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_curSatMeas = pz_corrMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_curSatMeas)
    {
      continue;
    }
    u_prn = gnss_cvt_SvIndex2Svid((uint8_t)q_satIndex, &u_constellation);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_curSigMeas = pz_curSatMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_curSigMeas)
      {
        continue;
      }
      w_index = ort_findObsIndexInGnssCorrBlock(u_constellation, u_prn, u_signalIndex, pz_auxiMeasBlock);
      if (w_index < 0)
      {
        continue;
      }
      if (0 == (pz_curSigMeas->z_measStatusFlag.b_prValid))
      {
        pz_auxiMeasBlock->z_meas[w_index].z_measStatusFlag.b_prValid = 0;
      }
      pz_auxiMeasBlock->z_meas[w_index].d_pseudoRange = pz_curSigMeas->d_pseudoRange;
      if (0 != (pz_corrMeasCollect->q_clk_jump))
      {
        pz_auxiMeasBlock->z_meas[w_index].d_carrierPhase = pz_curSigMeas->d_carrierPhase;
      }
    }
  }
  return;
}
/**
 * @brief do pre-process of auxiliary antenna measure,include detecting the cycle slip and mask the pseudo-range
 * @param[in]      pz_pvtPosition is the PVT position
 * @param[in/out]  pz_auxiMeasBlock is the auxiliary antenna measure block
 * @param[out]     pz_auxiCorrSlipFlag is the cycle slip flag of auxiliary antenna measure
 * @return     void
 */
void ort_auxiAntPreProcess(const gnss_PositionFix_t* pz_pvtPosition, GnssCorrBlock_t* pz_auxiMeasBlock, GnssCorrSlipFlag_t* pz_auxiCorrSlipFlag)
{
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  gnss_SatSigMeasCollect_t z_corrMeasCollect = { 0 };
  gnss_TdcpMeasBlock_t* pz_curDcpMeas = NULL;
  gnss_TdcpMeasPair_t z_tdcpMeasPointerPair = { NULL,NULL };
  //clean Corr Slip Flag
  pz_auxiCorrSlipFlag->u_changeStationFlag = 0;
  for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    for (w_j = 0; w_j < MAX_GNSS_SIGNAL_FREQ; ++w_j)
    {
      pz_auxiCorrSlipFlag->u_slip[w_i][w_j] = 0;
    }
  }
  gz_auxiAntSlipDetectObj.u_tdcpMethodValid = 0;
  gz_auxiAntSlipDetectObj.d_deltaTimeDopplerDetect = 0.0;
  z_corrMeasCollect.z_positionFix = *pz_pvtPosition;
  gnss_AppendCorrBlkToSatSigMeasCollect(pz_auxiMeasBlock, &z_corrMeasCollect);
  pp_UpdateSatEleAzi(&z_corrMeasCollect);
  gnss_SatSigMeasTimeJumpST(&z_corrMeasCollect);
  /*cycle slip detect and clock jump repair*/
  cmn_gfDetectCycleSlip(&z_corrMeasCollect, &(gz_auxiAntSlipDetectObj.z_ortGfDetectFilter));
  pp_ClkJumpRepaire(&gz_auxiSatSigMeasCollectPre, &z_corrMeasCollect);
  cmn_mwDetectCycleSlip(&z_corrMeasCollect, &(gz_auxiAntSlipDetectObj.z_ortMwDetectFilter));
  gz_auxiAntSlipDetectObj.d_deltaTimeDopplerDetect = cmn_dopplerDetectCycleSlip(&z_corrMeasCollect, &(gz_auxiAntSlipDetectObj.z_dopplerDetectPairingBlockOrt));
  pz_curDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  cmn_convertSigMeasToDcpStructNonSSR(&z_corrMeasCollect, pz_curDcpMeas);
  z_tdcpMeasPointerPair.pz_curTdcpMeas = pz_curDcpMeas;
  z_tdcpMeasPointerPair.pz_preTdcpMeas = (gz_auxiAntSlipDetectObj.pz_preTdcpMeasOrt);
  gz_auxiAntSlipDetectObj.u_tdcpMethodValid = cmn_detectCycleSlipByTDCP(z_tdcpMeasPointerPair, &z_corrMeasCollect);
  ort_maskAuxiAntCycleSlip(&z_corrMeasCollect, pz_auxiCorrSlipFlag);
  *(gz_auxiAntSlipDetectObj.pz_preTdcpMeasOrt) = *pz_curDcpMeas;
  (gz_auxiAntSlipDetectObj.pz_preTdcpMeasOrt)->z_posSol = z_corrMeasCollect.z_positionFix;
  OS_FREE(pz_curDcpMeas);
  /*end of *cycle slip detect and clock jump repair*/
  ort_dealWithAuxiAntPseudoRange(&z_corrMeasCollect, &gz_auxiSatSigMeasCollectPre);//deal with the gross error in the pseudo-range
  ort_copyAuxiAntMaskObsToGnssCorrBlock(&z_corrMeasCollect, pz_auxiMeasBlock);
  gnss_SatSigMeasCollect_Copy(&z_corrMeasCollect, &gz_auxiSatSigMeasCollectPre);
  gnss_SatSigMeasCollect_Clear(&z_corrMeasCollect);
  return;
}
/**
 * @brief detect the cycle slip of main antenna measure
 * @param[in/out]  pz_satSigMeasCollect is the main antenna measure block
 * @return     void
 */
void ort_mainAntCycleSlipDetect(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  gnss_TdcpMeasBlock_t* pz_curDcpMeas = NULL;
  gnss_TdcpMeasPair_t z_tdcpMeasPointerPair = { NULL,NULL };
  gz_mainAntSlipDetectObj.u_tdcpMethodValid = 0;
  gz_mainAntSlipDetectObj.d_deltaTimeDopplerDetect = 0.0;
  cmn_gfDetectCycleSlip(pz_satSigMeasCollect, &(gz_mainAntSlipDetectObj.z_ortGfDetectFilter));
  pp_ClkJumpRepaire(&gz_mainSatSigMeasCollectPre, pz_satSigMeasCollect);
  cmn_mwDetectCycleSlip(pz_satSigMeasCollect, &(gz_mainAntSlipDetectObj.z_ortMwDetectFilter));
  gz_mainAntSlipDetectObj.d_deltaTimeDopplerDetect = cmn_dopplerDetectCycleSlip(pz_satSigMeasCollect, &(gz_mainAntSlipDetectObj.z_dopplerDetectPairingBlockOrt));
  pz_curDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  cmn_convertSigMeasToDcpStructNonSSR(pz_satSigMeasCollect, pz_curDcpMeas);
  z_tdcpMeasPointerPair.pz_curTdcpMeas = pz_curDcpMeas;
  z_tdcpMeasPointerPair.pz_preTdcpMeas = (gz_mainAntSlipDetectObj.pz_preTdcpMeasOrt);
  gz_mainAntSlipDetectObj.u_tdcpMethodValid = cmn_detectCycleSlipByTDCP(z_tdcpMeasPointerPair, pz_satSigMeasCollect);
  gz_ortFilterInfo.d_deltaTimeDopplerDetect = gz_mainAntSlipDetectObj.d_deltaTimeDopplerDetect;
  gz_ortFilterInfo.u_tdcpMethodValid = gz_mainAntSlipDetectObj.u_tdcpMethodValid;
  *(gz_mainAntSlipDetectObj.pz_preTdcpMeasOrt) = *pz_curDcpMeas;
  (gz_mainAntSlipDetectObj.pz_preTdcpMeasOrt)->z_posSol = pz_satSigMeasCollect->z_positionFix;
  OS_FREE(pz_curDcpMeas);
  return;
}

/**
 * @brief      some of gz_ortfilterInfo's value need to init every epoch
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     void
 */
void ort_EpochFilterInfoInit(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  
  gz_ortFilterInfo.q_filterPosValid = FALSE;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    gz_ortFilterInfo.pu_measUpdatePseudoNum[u_i] = 0;
    gz_ortFilterInfo.pu_measUpdateCarrierNum[u_i] = 0;
  }
  gz_ortFilterInfo.f_age = 0.0;
  if ((gpz_auxiMeasBlock[0]->w_measCount) > 0)
  {
    gz_ortFilterInfo.f_age = (float)tm_GpsTimeDiff(&(pz_satSigMeasCollect->z_tor), &(gpz_auxiMeasBlock[0]->z_Clock.z_gpsTime));
  }

  if (NULL != gz_ortFilterInfo.pz_filterSol)
  {
    for (u_i = 0; u_i < MAX_FILTER_POS_STATUS; u_i++)
    {
      gz_ortFilterInfo.pz_filterSol->pu_posValid[u_i] = RTK_FILTER_SOL_STAT_NONE;
      for (u_j = 0; u_j < 3; u_j++)
      {
        gz_ortFilterInfo.pz_filterSol->pd_filterPos[u_i][u_j] = 0.0;
        gz_ortFilterInfo.pz_filterSol->pd_filterVel[u_i][u_j] = 0.0;
      }
      for (u_j = 0; u_j < 6; u_j++)
      {
        gz_ortFilterInfo.pz_filterSol->pd_filterPosQ[u_i][u_j] = 0.0;
      }
    }
  }
  for (u_i = 0; u_i < C_GNSS_MAX; ++u_i)
  {
    for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      gz_ortFilterInfo.pd_phaseCodeClkDiff[u_i][u_j] = 0.0;
    }
  }
  return;
}
/**
 * @brief ort count measure and print measure & time
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @return     void
 */
void ort_printEpochAndCountMeas(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const rtk_filterInfo_t* pz_RTKfilterInfo)
{
  uint8_t u_signalIndex = 0;
  uint8_t u_PRnum = 0;
  uint8_t u_CPnum = 0;
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
  LOGW(TAG_HPP, "\n");
  LOGW(TAG_HPP, "==========ort start filter %.1f, %4d/%02d/%02d %02d:%02d:%05.2f==========\n",
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
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      if (pz_sigMeas->z_measStatusFlag.b_prValid && pz_sigMeas->d_pseudoRange > 1.0e-2)
      {
        u_PRnum++;
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
 * @brief initilize the gnss_OrientFix_t struct
 * @param[in]  pz_pvtInfo is the PVT information of main antenna
 * @param[out] pz_ortResult is the result of heading .etc
 * @return     void
 */
void ort_initOrientStruct(const gnss_PVTResult_t* pz_pvtInfo, gnss_OrientFix_t* pz_ortResult)
{
  uint8_t u_i = 0;
  if (NULL != pz_pvtInfo && NULL != pz_ortResult)
  {
    pz_ortResult->u_version = VERSION_GNSS_ORT_FIX;
    pz_ortResult->w_size = (uint16_t)sizeof(gnss_OrientFix_t);
    pz_ortResult->z_gpsTime = pz_pvtInfo->z_positionFix.z_gpsTime;
    pz_ortResult->z_epoch = pz_pvtInfo->z_positionFix.z_epoch;
    pz_ortResult->u_leapsec = pz_pvtInfo->z_positionFix.u_leapsec;
    pz_ortResult->u_fixSource = FIX_SOURCE_INVALID;
    pz_ortResult->u_DcpPosType = GNSS_DCP_POS_INVALID;
    pz_ortResult->u_ortFixFlag = GNSS_FIX_FLAG_INVALID;
    pz_ortResult->u_SvTrackCount = 0;
    pz_ortResult->u_SvTrackCountEleThres = 0;
    pz_ortResult->u_SvInUseCount = 0;
    pz_ortResult->u_SvInUseCountMuliFreq = 0;
    pz_ortResult->u_ortSolStatus = C_GNSS_ORT_SOL_STATUS_UNAUTHORIZED;
    pz_ortResult->u_ortPosVelType = C_GNSS_ORT_POS_VEL_TYPE_NONE;
    pz_ortResult->u_GPSsignalUsedMask = 0;
    pz_ortResult->u_GLOsignalUsedMask = 0;
    pz_ortResult->u_GalSignalUsedMask = 0;
    pz_ortResult->u_BDSsignalUsedMask = 0;
    pz_ortResult->f_age = 0.0;
    pz_ortResult->z_ortResult.f_heading = 0.0;
    pz_ortResult->z_ortResult.f_pitch = 0.0;
    pz_ortResult->z_ortResult.f_roll = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_ortResult->z_ortResult.f_ENU[u_i] = 0.0;
      pz_ortResult->z_ortResult.f_deltaXYZ[u_i] = 0.0;
      pz_ortResult->z_mainAntInfo.d_xyz[u_i] = pz_pvtInfo->z_positionFix.d_xyz[u_i];
      pz_ortResult->z_mainAntInfo.d_lla[u_i] = pz_pvtInfo->z_positionFix.d_lla[u_i];
      pz_ortResult->z_mainAntInfo.f_velXyz[u_i] = pz_pvtInfo->z_positionFix.f_velXyz[u_i];
      pz_ortResult->z_mainAntInfo.f_velEnu[u_i] = pz_pvtInfo->z_positionFix.f_velEnu[u_i];
      pz_ortResult->z_ortResult.f_velXyz[u_i] = pz_pvtInfo->z_positionFix.f_velXyz[u_i];
      pz_ortResult->z_ortResult.f_velEnu[u_i] = pz_pvtInfo->z_positionFix.f_velEnu[u_i];
      pz_ortResult->z_mainAntInfo.pd_refStationCoordinate[u_i] = 0.0;
    }
    pz_ortResult->z_ortResult.f_headingStd = 0.0;
    pz_ortResult->z_ortResult.f_pitchStd = 0.0;
    pz_ortResult->z_ortResult.f_rollStd = 0.0;
    pz_ortResult->z_mainAntInfo.u_SvInUseCount = pz_pvtInfo->z_positionFix.z_SvStatus.u_SvInUseCount;
    pz_ortResult->z_mainAntInfo.u_posFixFlag = pz_pvtInfo->z_positionFix.u_fixFlag;
  }
  return;
}
/**
 * @brief fill up the sv track information into gnss_OrientFix_t struct
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_auxiMeasBlock is the auxiliary antenna measure block
 * @param[in]  pz_algoConfig is the configuration of orientation
 * @param[out] pz_ortResult is the result of heading .etc
 * @return     void
 */
void ort_fillupSvTrackInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_auxiMeasBlock, const rtk_alg_opt_t* pz_algoConfig, gnss_OrientFix_t* pz_ortResult)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_iSat = 0;
  BOOL z_isValid = FALSE;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  pz_ortResult->u_SvTrackCount = 0;
  pz_ortResult->u_SvTrackCountEleThres = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    z_isValid = FALSE;
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid))
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_auxiMeasBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        continue;
      }
      z_isValid = TRUE;
    }
    if (TRUE == z_isValid)
    {
      ++(pz_ortResult->u_SvTrackCount);
      if ((pz_satMeas->z_satPosVelClk.f_elevation) >= (pz_algoConfig->d_elmin))
      {
        ++(pz_ortResult->u_SvTrackCountEleThres);
      }
    }
  }
  return;
}
/**
 * @brief fill up the constellation mask of orient into gnss_OrientFix_t struct
 * @param[in]  u_constellation is constellation type
 * @param[in]  u_freqType frequency type
 * @param[out] pz_ortResult is the result of heading .etc
 * @return     void
 */
void ort_fillupOrientSysMask(gnss_ConstellationType u_constellation, gnss_FreqType u_freqType, gnss_OrientFix_t* pz_ortResult)
{
  if (C_GNSS_GPS == u_constellation)
  {
    if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
    {
      (pz_ortResult->u_GPSsignalUsedMask) |= C_GNSS_ORT_GPS_L1_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == u_freqType)
    {
      (pz_ortResult->u_GPSsignalUsedMask) |= C_GNSS_ORT_GPS_L2_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == u_freqType)
    {
      (pz_ortResult->u_GPSsignalUsedMask) |= C_GNSS_ORT_GPS_L5_USED;
    }
  }
  else if (C_GNSS_GLO == u_constellation)
  {
    if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
    {
      (pz_ortResult->u_GLOsignalUsedMask) |= C_GNSS_ORT_GLO_L1_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == u_freqType)
    {
      (pz_ortResult->u_GLOsignalUsedMask) |= C_GNSS_ORT_GLO_L2_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == u_freqType)
    {
      (pz_ortResult->u_GLOsignalUsedMask) |= C_GNSS_ORT_GLO_L3_USED;
    }
  }
  else if (C_GNSS_BDS3 == u_constellation)
  {
    if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
    {
      (pz_ortResult->u_BDSsignalUsedMask) |= C_GNSS_ORT_BDS_L1_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == u_freqType)
    {
      (pz_ortResult->u_BDSsignalUsedMask) |= C_GNSS_ORT_BDS_L3_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == u_freqType)
    {
      (pz_ortResult->u_BDSsignalUsedMask) |= C_GNSS_ORT_BDS_L5_USED;
    }
  }
  else if (C_GNSS_GAL == u_constellation)
  {
    if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
    {
      (pz_ortResult->u_GalSignalUsedMask) |= C_GNSS_ORT_GAL_L1_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == u_freqType)
    {
      (pz_ortResult->u_GalSignalUsedMask) |= C_GNSS_ORT_GAL_L2_USED;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == u_freqType)
    {
      (pz_ortResult->u_GalSignalUsedMask) |= C_GNSS_ORT_GAL_L5_USED;
    }
  }
  else if (C_GNSS_QZS == u_constellation)//to be extended
  {
    if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
    {
    }
    else if (C_GNSS_FREQ_TYPE_L2 == u_freqType)
    {
    }
    else if (C_GNSS_FREQ_TYPE_L5 == u_freqType)
    {
    }
  }
  return;
}
/**
 * @brief fill up the sv in use information into gnss_OrientFix_t struct
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_ortResult is the result of heading .etc
 * @return     void
 */
void ort_fillupSvInUseInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const rtk_EpochFilterObs_t* pz_filterObs, gnss_OrientFix_t* pz_ortResult)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_signalNumInUse = 0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const rtk_SatFilterObs_t* pz_satFilterObs = NULL;
  pz_ortResult->u_SvInUseCount = 0;
  pz_ortResult->u_SvInUseCountMuliFreq = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_signalNumInUse = 0;
    pz_satFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      if (1 == (pz_satFilterObs->z_measUpdateFlag[u_signalIndex]->b_prValid) || 1 == (pz_satFilterObs->z_measUpdateFlag[u_signalIndex]->b_cpValid))
      {
        ++u_signalNumInUse;
        z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
        ort_fillupOrientSysMask(pz_sigMeas->u_constellation, z_freqType, pz_ortResult);
      }
    }
    if (u_signalNumInUse > 0)
    {
      ++(pz_ortResult->u_SvInUseCount);
    }
    if (u_signalNumInUse > 1)
    {
      ++(pz_ortResult->u_SvInUseCountMuliFreq);
    }
  }
  return;
}
/**
 * @brief get the position variance
 * @param[in]  pz_ortFilterInfo is the orientation information of filter
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pd_posSigma is the sigma of position x,y,z
 * @return     TRUE represent success and FALSE failure
 */
BOOL ort_getPosSigma(const rtk_filterInfo_t* pz_ortFilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, double pd_posSigma[3])
{
  BOOL q_isValid = TRUE;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  uint8_t u_i = 0;
  double d_cov = 0.0;
  const double d_enfactor = 2.0;//CEP95
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    pw_PVAindex[u_i] = -1;
  }
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_posSigma[u_i] = 0.0;
    if (pw_PVAindex[u_i] < 0)
    {
      q_isValid = FALSE;
      break;
    }
    d_cov = pz_ortFilterInfo->pd_Q[IUTM(u_i, u_i)];
    if (d_cov <= 0.0)
    {
      q_isValid = FALSE;
      break;
    }
    pd_posSigma[u_i] = sqrt(d_cov) * d_enfactor;
    
  }
  return q_isValid;
}
/**
 * @brief convert the GnssCorrBlock_t struct to gnss_TdcpMeasBlock_t
 * @param[in]  pz_corrBlock is correction data
 * @param[in]  pz_positionFix is position result
 * @param[out] pz_tdcpMeasBlock the measure block expressed by TDCP struct
 * @return     void
 */
void ort_cvtCorrMeas2TdcpMeas(const GnssCorrBlock_t* pz_corrBlock, const gnss_PositionFix_t* pz_positionFix, gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock)
{
  uint16_t w_i = 0;
  uint8_t u_j = 0;
  float pf_azi[2] = { 0 };
  double pd_unitVector[3] = { 0 };
  double pd_satEcefRot[3] = { 0 };
  const double* pd_siteXyz = pz_positionFix->d_xyz;
  const double* pd_siteLLA = pz_positionFix->d_lla;
  gnss_DcpSatInfo_t* pz_satInfo = NULL;
  pz_tdcpMeasBlock->u_version = VERSION_TDCP_MEAS_BLOCK;
  pz_tdcpMeasBlock->w_size = sizeof(gnss_TdcpMeasBlock_t);
  pz_tdcpMeasBlock->z_obsTime = pz_corrBlock->z_Clock.z_gpsTime;
  if (NULL != pz_positionFix)
  {
    pz_tdcpMeasBlock->z_posSol = *pz_positionFix;
  }
  pz_tdcpMeasBlock->w_satNum = pz_corrBlock->w_satCount;
  pz_tdcpMeasBlock->w_measNum = pz_corrBlock->w_measCount;
  for (w_i = 0; w_i < (pz_corrBlock->w_measCount); ++w_i)
  {
    if (w_i >= MAX_GNSS_TRK_MEAS_NUMBER)
    {
      break;
    }
    pz_tdcpMeasBlock->pz_meas[w_i] = pz_corrBlock->z_meas[w_i];
    pz_tdcpMeasBlock->pz_measExt[w_i].u_multiPath = 0;
  }

  for (w_i = 0; w_i < (pz_corrBlock->w_satCount); ++w_i)
  {
    if (w_i >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satInfo = &(pz_tdcpMeasBlock->pz_satInfo[w_i]);
    pz_satInfo->u_constellation = pz_corrBlock->pz_satPosClk[w_i].u_constellation;
    pz_satInfo->u_svid = pz_corrBlock->pz_satPosClk[w_i].u_svid;
    pz_satInfo->q_iode = pz_corrBlock->pz_satPosClk[w_i].q_iode;
    pz_satInfo->u_valid = FALSE;
    pz_satInfo->u_isCalculateDist = FALSE;
    for (u_j = 0; u_j < 4; ++u_j)
    {
      pz_satInfo->pd_satPosClk[u_j] = pz_corrBlock->pz_satPosClk[w_i].d_satPosClk[u_j];
      pz_satInfo->pd_satVelClk[u_j] = 0.0;
    }
    gnss_satRot(pd_siteXyz, pz_satInfo->pd_satPosClk, pd_satEcefRot);
    pz_satInfo->d_sat2SiteDist = gnss_unitVector(pd_siteXyz, pd_satEcefRot, pd_unitVector);
    gnss_Satazel(pd_siteLLA, pd_unitVector, pf_azi);
    pz_satInfo->f_elevation = pf_azi[1] * (float)RAD2DEG;
    pz_satInfo->f_azimuth = pf_azi[0] * (float)RAD2DEG;
    for (u_j = 0; u_j < 3; ++u_j)
    {
      pz_satInfo->pd_site2SatUnit[u_j] = -pd_unitVector[u_j];
    }
    if ((pz_satInfo->q_iode) >= 0 && (pz_satInfo->f_elevation > FABS_ZEROS) && (pz_satInfo->f_azimuth >= FABS_ZEROS))
    {
      pz_satInfo->u_valid = TRUE;
      pz_satInfo->u_isCalculateDist = TRUE;
    }
  }
  return;
}
/**
 * @brief process the main antenna and auxiliary antenna measure block to get heading .etc
 * @param[in]  z_obsTime is observation time
 * @param[out] pz_ortResult is the result of heading .etc
 * @param[out] pz_mainAntTdcpMeasBlock the main measure block expressed by TDCP struct
 * @param[out] pz_auxiAntTdcpMeasBlock the auxiliary measure block expressed by TDCP struct
 * @return     0 represent success and other failure
 */
ort_solStatus ort_startProcess(GpsTime_t z_obsTime, gnss_OrientFix_t* pz_ortResult, gnss_TdcpMeasBlock_t* pz_mainAntTdcpMeasBlock, gnss_TdcpMeasBlock_t* pz_auxiAntTdcpMeasBlock)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_nv = 0;
  int32_t q_floatSolStatus = 0;
  int16_t w_mainAntIndex = -1;
  int16_t w_auxiAntIndex = -1;
  int16_t w_mainPvtIndex = -1;
  uint8_t u_commonSat = 0;
  double pd_posSigma[3] = { 0.0 };
  gnss_SatSigMeasCollect_t z_satSigMeasCollect = { 0 };
  rtk_EpochFilterObs_t* pz_filterObs = NULL;
  gnss_fixedSignalAmbPool_t* pz_fixedOrtAmbPool = NULL;
  const gnss_PVTResult_t* pz_pvtInfo = NULL;
  memset(gpz_positionFix, 0, sizeof(gnss_PositionFix_t));
  memset(pz_ortResult, 0, sizeof(gnss_OrientFix_t));
  pz_ortResult->u_fixSource = FIX_SOURCE_RTK;
  if (FALSE == gz_isMemoryAllocSucc)
  {
    return ORT_MEM_FAIL;
  }
  if (fabs(tm_GpsTimeDiff(&z_obsTime, &gz_ortLastCalTime)) < 1.0e-3)
  {
    return ORT_TIME_REPEAT;
  }
  w_mainPvtIndex = ort_getMainAntPvtAccordTime(&z_obsTime);
  if (w_mainPvtIndex < 0)
  {
    return ORT_GET_MAIN_ANT_PVT_FAIL;
  }
  pz_pvtInfo = gpz_mainPvtInfo[w_mainPvtIndex];
  if (GNSS_FIX_FLAG_INVALID == (pz_pvtInfo->z_positionFix.u_fixFlag))
  {
    return ORT_GET_MAIN_PVT_FAIL;
  }
  gz_ortFilterInfo.pz_pvtResult = pz_pvtInfo;
  ort_initOrientStruct(pz_pvtInfo, pz_ortResult);
  w_mainAntIndex = ort_getMainAntMeasIndexAccordPvtInfo(&(pz_pvtInfo->z_positionFix.z_gpsTime));
  if (w_mainAntIndex < 0)
  {
    return ORT_GET_MAIN_ANT_OBS_FAIL;
  }
  w_auxiAntIndex = ort_getAuxiliaryAntMeasIndexAccordPvtInfo(&(gpz_mainMeasBlock[w_mainAntIndex]->z_Clock.z_gpsTime));
  if (w_auxiAntIndex < 0)
  {
    return ORT_GET_AUXI_ANT_OBS_FAIL;
  }
  if (0 != ort_fillUpAuxiliaryAntPos(pz_pvtInfo, gpz_auxiMeasBlock[w_auxiAntIndex]))
  {
    return ORT_GET_AUXI_ANT_PVT_FAIL;
  }
  gz_ortLastCalTime = z_obsTime;
  pz_ortResult->f_age = (float)(tm_GpsTimeDiff(&(gpz_mainMeasBlock[w_mainAntIndex]->z_Clock.z_gpsTime), &(gpz_auxiMeasBlock[w_auxiAntIndex]->z_Clock.z_gpsTime)));
  rtk_positiveDefiniteCheck(&gz_ortFilterInfo, &gz_ORT_EKFstateRepPool);
  gnss_AppendMeasBlkToSatSigMeasCollect(gpz_mainMeasBlock[w_mainAntIndex], &z_satSigMeasCollect);//if the function exited midway,must call the function gnss_SatSigMeasCollect_Clear before exited
  ort_EpochFilterInfoInit(&z_satSigMeasCollect);
  gnss_fillUpObsMask(pz_pvtInfo, &z_satSigMeasCollect);
  z_satSigMeasCollect.z_positionFix = pz_pvtInfo->z_positionFix;
  gnss_ComputeSatPosVelClk(&z_satSigMeasCollect);
  gnss_SatSigMeasTimeJumpST(&z_satSigMeasCollect);

  /* satellite ele */
  pp_UpdateSatEleAzi(&z_satSigMeasCollect);
  rtk_initCorrSatPosClk(gpz_auxiMeasBlock[w_auxiAntIndex]);
  rtk_calCorrDataSatPosClk(gpz_auxiMeasBlock[w_auxiAntIndex]);
  rtk_reCalculateCorrDataSatPosClkByRoverIode(&z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex]);
  u_commonSat = RTK_intersectionSatNumObsAndVRS(&z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex]);
  ort_fillupSvTrackInfo(&z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex], &(gz_ortFilterInfo.z_opt), pz_ortResult);
  if (((KF_INIT == (gz_ortFilterInfo.z_kfStatus) || KF_RESET == (gz_ortFilterInfo.z_kfStatus)
    || tm_GpsTimeDiff(&(z_satSigMeasCollect.z_tor), &(gz_ORT_EKFstateRepPool.z_gpsTime)) > 3.0) && u_commonSat < 5)
    || GNSS_FIX_FLAG_INVALID == pz_pvtInfo->z_positionFix.u_fixFlag)
  {
    LOGI(TAG_HPP, "ORT Start fail!\n");
    rtk_CreatePosSolution(&(z_satSigMeasCollect), &gz_ortFilterInfo, pz_fixedOrtAmbPool,
      gpz_positionFix, gz_ortAmbFixInputInfo.pz_preFixedAmbPool[1]);
    gnss_SatSigMeasCollect_Clear(&z_satSigMeasCollect);
    return ORT_FILTER_FAIL;
  }

  /* print Debug time */
  ort_printEpochAndCountMeas(&z_satSigMeasCollect, &gz_ortFilterInfo);

  ort_auxiAntPreProcess(&(pz_pvtInfo->z_positionFix), gpz_auxiMeasBlock[w_auxiAntIndex], &gz_auxiCorrSlipFlag);
  ort_mainAntCycleSlipDetect(&z_satSigMeasCollect);

  if (0 != RTD_solute(pz_pvtInfo, &z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex], gpz_ortRTDfilterInfo, &gz_ort_RTD_EKFstateRepPool))
  {
    gz_ortFilterInfo.q_QRcheckStatus = FALSE;
  }
  else
  {
    gz_ortFilterInfo.q_QRcheckStatus = TRUE;
  }
  pz_filterObs = malloc_pz_RTKfilterObs(&z_satSigMeasCollect);
  /* Orientation float resolution */
  q_floatSolStatus = RTK_filterSolute(&z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex],
    &gz_ortFilterInfo, &gz_ORT_EKFstateRepPool, pz_filterObs, &gz_auxiCorrSlipFlag);
  ort_fillupSvInUseInfo(&z_satSigMeasCollect, pz_filterObs, pz_ortResult);
  if (rtk_filterResetCheck(&gz_ortFilterInfo, &gz_ORT_EKFstateRepPool) && !q_floatSolStatus)
  {
    q_floatSolStatus = 1;
  }
  rtk_setAmbFixInputInfo(&z_satSigMeasCollect, pz_filterObs, gpz_auxiMeasBlock[w_auxiAntIndex], &gz_ortFilterInfo,
    &gz_ORT_EKFstateRepPool, &gz_ortAmbFixInputInfo);


  /* pre fix logic */
  u_nv = RTK_preFixSolute(&z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex], &gz_ortFilterInfo, 
    &gz_ORT_EKFstateRepPool, gz_ortAmbFixInputInfo.pz_preFixedAmbPool[0], pz_filterObs);
  /* clean detected half cycle flag */
  RTK_preFixCleanRejectLabel(&z_satSigMeasCollect);
  if (gz_ortFilterInfo.u_prefixStatus)
  {
    RTK_calculateResidual(gz_ortFilterInfo.pd_X, &z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex], &gz_ortFilterInfo,
      &gz_ORT_EKFstateRepPool, gz_ortAmbFixInputInfo.pz_preFixedAmbPool[0], pz_filterObs, RES_PREFIX);
    /* use prefix to detect half cycle */
    if (u_nv > 6)
    {
      rtk_preFixSol_availabilityCheck(&gz_ortAmbFixInputInfo, gz_ortAmbFixInputInfo.pz_preFixedAmbPool[0], u_nv);

      RTK_preFixHalfCycleDetect(&z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex], &gz_ortFilterInfo, &gz_ORT_EKFstateRepPool,
        gz_ortAmbFixInputInfo.pz_preFixedAmbPool[0]);
    }
  }

  /* Orientation ambiguity resolution */
  malloc_fixedSignalAmbPool(&pz_fixedOrtAmbPool, &z_satSigMeasCollect);
  if (!q_floatSolStatus && gz_ortFilterInfo.pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] >= RTK_FILTER_SOL_STAT_VALID)
  {
    RTK_AmbResolution(&gz_ortAmbFixInputInfo, pz_fixedOrtAmbPool);
  }

  rtk_PreFixAmbCalculate(u_nv, &z_satSigMeasCollect, gpz_auxiMeasBlock[w_auxiAntIndex], &gz_ortFilterInfo, &gz_ORT_EKFstateRepPool,
    gz_ortAmbFixInputInfo.pz_preFixedAmbPool[0]);

  rtk_CreatePosSolution(&(z_satSigMeasCollect), &gz_ortFilterInfo, pz_fixedOrtAmbPool,
    gpz_positionFix, gz_ortAmbFixInputInfo.pz_preFixedAmbPool[1]);

  if (TRUE == ort_getPosSigma(&gz_ortFilterInfo, &gz_ORT_EKFstateRepPool, pd_posSigma))
  {
    gnss_getOrtHeadingPitch(gpz_positionFix->d_xyz, gpz_auxiMeasBlock[w_auxiAntIndex]->d_refPosEcef, pd_posSigma, &(pz_ortResult->z_ortResult));
  }
  pz_ortResult->u_ortFixFlag = gpz_positionFix->u_fixFlag;
  if (GNSS_FIX_FLAG_INVALID != (gpz_positionFix->u_fixFlag))
  {
    pz_ortResult->u_ortSolStatus = C_GNSS_ORT_SOL_STATUS_SOL_COMPUTED;
  }
  pz_ortResult->u_ortPosVelType = gnss_cvt_ortFixFlag2PosVelType(gpz_positionFix->u_fixFlag);
  if (GNSS_FIX_FLAG_FIXED == (pz_ortResult->u_ortFixFlag))
  {
    if (pz_ortResult->z_mainAntInfo.u_SvInUseCount > 20 && pz_ortResult->u_SvInUseCount > 20
      && (pz_ortResult->z_ortResult.f_headingStd) < 3.0f && (pz_ortResult->z_ortResult.f_pitchStd) < 3.0f)
    {
      pz_ortResult->z_ortResult.f_headingStd = M_MIN(0.9f, pz_ortResult->z_ortResult.f_headingStd);
      pz_ortResult->z_ortResult.f_pitchStd = M_MIN(0.9f, pz_ortResult->z_ortResult.f_pitchStd);
    }
  }
  //fill up the TDCP measure block of main antenna
  if (NULL != pz_mainAntTdcpMeasBlock)
  {
    gnss_cvt_SatSigMeas2TdcpMeas(&(z_satSigMeasCollect), gpz_positionFix, pz_mainAntTdcpMeasBlock);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_mainAntTdcpMeasBlock->z_posSol.f_posXyzUnc[u_i] = (float)(pd_posSigma[u_i]);
    }
  }
  //fill up the TDCP measure block of auxi antenna
  if (NULL != pz_auxiAntTdcpMeasBlock)
  {
    ort_cvtCorrMeas2TdcpMeas(gpz_auxiMeasBlock[w_auxiAntIndex], &(pz_pvtInfo->z_positionFix), pz_auxiAntTdcpMeasBlock);
    pz_auxiAntTdcpMeasBlock->z_posSol.u_fixFlag = gpz_positionFix->u_fixFlag;//there will used to be decided fix lag in DCP
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_auxiAntTdcpMeasBlock->z_posSol.f_posXyzUnc[u_i] = (float)(pd_posSigma[u_i]);
    }
  }

  free_pz_RTKFilterObs(&pz_filterObs);
  free_fixedSignalAmbPool(&pz_fixedOrtAmbPool);
  gnss_SatSigMeasCollect_Copy(&z_satSigMeasCollect, &gz_mainSatSigMeasCollectPre);
  gnss_SatSigMeasCollect_Clear(&z_satSigMeasCollect);
  pz_fixedOrtAmbPool = NULL;

  LOGW(TAG_HPP, "============================= ort end filter =============================\n");
  LOGW(TAG_HPP, "\n");

  return u_status;
}