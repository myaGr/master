#include "rtk_filter_sol.h"
#include "gnss_common.h"
#include "gnss_engine_api.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "cmn_CSmask_combine.h"
#include "cmn_highPrecision.h"
#include "seq_kalman.h"
#include "rtk_pos_check.h"
#include "rtk_integrity.h"
#include "cmn_utils.h"
#include <math.h>

typedef struct
{
  const double*                   pd_X;                /*the value of parameter in filter*/
  const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect;/*the observations*/
  const GnssCorrBlock_t*          pz_rtkCorrBlock;     /*the VRS observations*/
  rtk_filterInfo_t*               pz_RTKfilterInfo;    /*the filter information of RTK*/
  gnss_EKFstateRepresentPool_t*   pz_EKFstateRepPool;  /*the filter state pool of RTK*/
  gnss_fixedSignalAmbPool_t*      pz_preFixedAmbPool;  /*the pre-fixed pool of RTK*/
  rtk_EpochFilterObs_t*           pz_filterObs;        /*the observation information that entered in filter*/
  rtk_ResidualType                u_residualType;      /*the residual type*/
}RTK_resCalInputInfo_t;

typedef struct
{
  double d_lockSum;
  double d_allCn0;
  uint8_t u_validSat[ALL_GNSS_SYS_SV_NUMBER];
  uint8_t u_resNumCodeArray[MAX_GNSS_SIGNAL_FREQ];
  uint8_t u_resNumPhaseArray[MAX_GNSS_SIGNAL_FREQ];
}RTK_resStatisticInfo_t;


/**
 * @brief sat meas`s singal num, filter NULL
 * @param pz_satMeas
 * @return sat meas`s singal num 0~3
 */
uint8_t RTK_signalNum(const gnss_SatelliteMeas_t* pz_satMeas)
{
  uint8_t u_n = 0;
  uint8_t u_i = 0;
  for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; ++u_i)
  {
    if (NULL != pz_satMeas->pz_signalMeas[u_i] && fabs(pz_satMeas->pz_signalMeas[u_i]->d_carrierPhase) > FABS_ZEROS)
    {
      ++u_n;
    }
  }
  return u_n;
}
/**
 * @brief      get flag whether vehicle is parking in close sky
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @return     1 represent parking in close sky and 0 represent not
 */
uint8_t RTK_getParkFlag(rtk_filterInfo_t* pz_RTKfilterInfo)
{
  BOOL b_closeSky = FALSE;
  BOOL b_staticFlag = FALSE;
  BOOL b_status = FALSE;

  double d_tow = pz_RTKfilterInfo->pz_pvtResult->z_positionFix.z_gpsTime.q_towMsec * TIME_MSEC_INV;

  b_closeSky = !((pz_RTKfilterInfo->t_openSkyCount & 0x3) == 0x3);

  b_staticFlag = gnss_engine_get_peMode_stastic_flag();

  LOGI(TAG_HPP, "CloseSkyParkFlag: tow=%.1f, static=%d, close=%d\n", d_tow, b_staticFlag, b_closeSky);

  return b_staticFlag;
}
/**
 * @brief add and time update the PVA parameter in the EKF
 * @param[in]  z_gpsTime is the observation time for current epoch
 * @param[in]  pd_siteCoor is the initilization value of site coordination
 * @param[in]  pd_siteVel is the initilization value of site velocity
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @return     1 represent success and 0 represent failure
 */
uint8_t RTK_addSiteParaPVA(GpsTime_t z_gpsTime, const double pd_siteCoor[3], const double pd_siteVel[3],
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  BOOL b_closeSky = FALSE;
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_filterInitInfo_t z_initInfo;
  int16_t pw_pvaIndex[PVA_NUM] = { -1 };
  double d_staticNoice = 0.0;
  double d_tow = 0.0;
  z_initInfo.z_paraTime = z_gpsTime;

  if (NULL == pz_RTKfilterInfo)
  {
    return 0;
  }
  d_tow = pz_RTKfilterInfo->pz_pvtResult->z_positionFix.z_gpsTime.q_towMsec * TIME_MSEC_INV;
  b_closeSky = !(pz_RTKfilterInfo->u_goodSceneStatusFlag);
  pz_RTKfilterInfo->b_staticFlag = RTK_getParkFlag(pz_RTKfilterInfo);

  //initialize position,velocity and acceleration for first epoch
  if (KF_INIT == (pz_RTKfilterInfo->z_kfStatus) || KF_RESET == (pz_RTKfilterInfo->z_kfStatus)
    || tm_GpsTimeDiff(&z_gpsTime, &(pz_EKFstateRepPool->z_gpsTime)) > 3.0)
  {
    z_initInfo.u_resetPara = 1;
    //initialize position
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = pd_siteCoor[u_i];
      z_initInfo.d_sigma0 = POS_SIGMA;
      z_initInfo.d_noise = POS_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_POS;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    //initialize velocity
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = pd_siteVel[u_i];
      z_initInfo.d_sigma0 = VEL_SIGMA;
      z_initInfo.d_noise = VEL_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_VEL;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    //initialize acceleration
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = 0.0;
      z_initInfo.d_sigma0 = ACC_SIGMA;
      z_initInfo.d_noise = ACC_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_ACC;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    (pz_RTKfilterInfo->z_kfStatus) = KF_RUN;
  }
  else if (TRUE == pz_RTKfilterInfo->z_opt.b_enableStaticConstraints && pz_RTKfilterInfo->b_staticFlag)
  {
    LOGI(TAG_HPP, "Recognize as Park: tow=%.1f, use P mode replase PVA mode\n", d_tow);
    if (b_closeSky)
    {
      //z_initInfo.u_resetPara = 1;
      d_staticNoice = 1e-3;
    }
    else
    {
      d_staticNoice = 1e-3;
    }
    z_initInfo.u_resetPara = 0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = pd_siteCoor[u_i];
      z_initInfo.d_sigma0 = POS_SIGMA;
      z_initInfo.d_noise = d_staticNoice;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_POS;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    z_initInfo.u_resetPara = 1;
    //initialize velocity
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = pd_siteVel[u_i];
      z_initInfo.d_sigma0 = VEL_SIGMA;
      z_initInfo.d_noise = VEL_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_VEL;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    //initialize acceleration
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = 0.0;
      z_initInfo.d_sigma0 = ACC_SIGMA;
      z_initInfo.d_noise = ACC_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_ACC;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    getPvaParaIndex(pz_EKFstateRepPool, pw_pvaIndex);
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
      pz_RTKfilterInfo->pq_paraValid[pw_pvaIndex[u_i]] = TRUE;
    }
  }
  else
  {
    dynamicStateTransfer(pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_EKFstateRepPool, z_gpsTime, 0.2, 1.0, 0);
    getPvaParaIndex(pz_EKFstateRepPool, pw_pvaIndex);
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
      pz_RTKfilterInfo->pq_paraValid[pw_pvaIndex[u_i]] = TRUE;
    }
  }
  return u_status;
}
/**
 * @brief get the satellite number used by the initialition value of the receiver clock parameter
 * @param[in]  q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]  z_targetSys is the target constellation
 * @param[in]  z_targetFreq is the target frequency
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR product
 * @return     the satellite number used by the initialition value of the receiver clock parameter
 */
uint32_t RTK_getSatNumUsedByInitRcvClkCal(BOOL q_isSperateBDS2And3, gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq,
  const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock)
{
  uint32_t q_satNum = 0;
  uint32_t q_satIndex = 0;
  uint16_t w_iSat = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_prValid)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
      if (u_constellation != z_targetSys || (pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != z_targetFreq)
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount))
      {
        continue;
      }
      ++q_satNum;
    }
  }
  return q_satNum;
}
/**
 * @brief check the valid of inital value of receiver clock
 * @param[in]      pd_rcvClkSet is the value of receiver clock for target observations
 * @param[in]      q_num is the number of observations that used by calculating receiver clock
 * @param[in]      d_initRcvClk is the initilization value of reiceiver clock
 * @return         1 represent valid and 0 represent invalid
 */
uint8_t RTK_CheckValidOfInitRcvClk(const double* pd_rcvClkSet, uint32_t q_num, double d_initRcvClk)
{
  uint8_t u_valid = 0;
  uint32_t q_maxIterNum = q_num;
  uint32_t q_normNum = 0;
  uint32_t q_i = 0;
  uint32_t q_k = 0;
  uint8_t* pu_mask = NULL;
  double d_rcvClkStd = 0.0;
  double d_maxClk = 0.0;
  int32_t q_maxClkIndex = -1;
  if (q_num > 0)
  {
    pu_mask = (uint8_t*)OS_MALLOC(q_num * sizeof(uint8_t));
    for (q_i = 0; q_i < q_num; ++q_i)
    {
      pu_mask[q_i] = 0;
    }
  }
  //check the valid of inital value of receiver clock
  if (q_num <= 1)
  {
    u_valid = 1;
    if (fabs(pd_rcvClkSet[0]) > 20.0)
    {
      u_valid = 0;
    }
  }
  else
  {
    //iterative processing to filter max value
    for (q_k = 0; q_k < q_maxIterNum; ++q_k)
    {
      u_valid = 0;
      d_rcvClkStd = 0.0;
      q_normNum = 0;
      for (q_i = 0; q_i < q_num; ++q_i)
      {
        if (1 == pu_mask[q_i])
        {
          continue;
        }
        d_rcvClkStd += (pd_rcvClkSet[q_i] - d_initRcvClk) * (pd_rcvClkSet[q_i] - d_initRcvClk);
        ++q_normNum;
      }
      if (q_normNum < 2)
      {
        break;
      }
      d_rcvClkStd = sqrt(d_rcvClkStd / (q_normNum - 1));
      if (d_rcvClkStd < 20.0)
      {
        u_valid = 1;
        break;
      }
      else
      {
        d_maxClk = 0.0;
        q_maxClkIndex = -1;
        for (q_i = 0; q_i < q_num; ++q_i)
        {
          if (1 == pu_mask[q_i])
          {
            continue;
          }
          if (fabs(pd_rcvClkSet[q_i] - d_initRcvClk) > d_maxClk)
          {
            d_maxClk = fabs(pd_rcvClkSet[q_i] - d_initRcvClk);
            q_maxClkIndex = q_i;
          }
        }
        if (q_maxClkIndex >=0)
        {
          pu_mask[q_maxClkIndex] = 1;
        }
      }
    }
  }
  if (NULL != pu_mask)
  {
    OS_FREE(pu_mask);
  }
  return u_valid;
}
/**
 * @brief using pseudo-range to calculate the initialition value of the receiver clock parameter
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]      z_targetSys is the target constellation
 * @param[in]      z_targetFreq is the target frequency
 * @param[in]      pd_siteCoor is the initilization value of site coordination
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR product
 * @param[out]     pd_initRcvClk is the initialition value of the receiver clock parameter
 * @param[out]     pq_obsNum is the observations number for calculating receiver clock value
 * @return         1 represent success and 0 represent failure
 */
rtk_RcvInitClkStatus RTK_UsePseudoCalcInitRcvClk(BOOL q_isSperateBDS2And3, gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq,
  const double pd_siteCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, double* pd_initRcvClk, uint32_t* pq_obsNum)
{
  uint32_t q_satNum = 0;
  uint32_t q_satIndex = 0;
  uint32_t q_i = 0;
  uint32_t q_num = 0;
  uint16_t w_iSat = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_status = 0;
  uint8_t u_constellation = 0;
  rtk_RcvInitClkStatus u_clkValid = RTK_RCV_INIT_CLK_INVALID;
  uint8_t u_i = 0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_pseduoSatDiff = 0.0;
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double pd_s2r[3] = { 0.0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  double* pd_rcvClkSet = NULL;
  uint32_t* pq_satIndexSet = NULL;
  uint8_t* pu_signalIndexSet = NULL;
  q_satNum = RTK_getSatNumUsedByInitRcvClkCal(q_isSperateBDS2And3, z_targetSys, z_targetFreq, pz_satSigMeasCollect, pz_rtkCorrBlock);
  *pq_obsNum = q_satNum;

  if ((C_GNSS_QZS != z_targetSys && q_satNum <= 2) ||
    (C_GNSS_QZS == z_targetSys && q_satNum < 2))
  {
    return RTK_RCV_INIT_CLK_INVALID;
  }

  pd_rcvClkSet = (double*)OS_MALLOC(q_satNum * sizeof(double));
  memset(pd_rcvClkSet, 0, q_satNum * sizeof(double));
  pq_satIndexSet = (uint32_t*)OS_MALLOC(q_satNum * sizeof(uint32_t));
  memset(pq_satIndexSet, 0, q_satNum * sizeof(uint32_t));
  pu_signalIndexSet = (uint8_t*)OS_MALLOC(q_satNum * sizeof(uint8_t));
  memset(pu_signalIndexSet, 0, q_satNum * sizeof(uint8_t));
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (q_num >= q_satNum)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_prValid)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
      if (u_constellation != z_targetSys || (pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != z_targetFreq)
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount))
      {
        continue;
      }
      d_pseduoSatDiff = (pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_satMeas->z_satPosVelClk.d_satPosClk[u_i] - pd_siteCoor[u_i];
      }
      d_roverDist = gnss_Norm(pd_s2r, 3);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      pd_rcvClkSet[q_num] = d_pseduoSatDiff - (d_roverDist - d_baseDist);
      pq_satIndexSet[q_num] = q_satIndex;
      pu_signalIndexSet[q_num] = u_signalIndex;
      ++q_num;
    }
  }
  u_status = gnss_ascSortMedianDouble(pd_rcvClkSet, q_num, pd_initRcvClk);
  if (0 == RTK_CheckValidOfInitRcvClk(pd_rcvClkSet, q_num, *pd_initRcvClk))
  {
    u_status = 0;
  }
  if (u_status)
  {
    u_clkValid = RTK_RCV_INIT_CLK_VALID;
  }
  if (NULL != pd_rcvClkSet)
  {
    OS_FREE(pd_rcvClkSet);
  }
  if (NULL != pq_satIndexSet)
  {
    OS_FREE(pq_satIndexSet);
  }
  if (NULL != pu_signalIndexSet)
  {
    OS_FREE(pu_signalIndexSet);
  }
  return u_clkValid;
}
/**
 * @brief verify the valid of receiver clock for the BDS2 and BDS3 constellation
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[in]     pz_rtkCorrBlock is the OSR product
 * @param[out]    pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]    pu_BDS2clkStatus is the valid of receiver clock for BDS2 constellation
 * @param[out]    pu_BDS3clkStatus is the valid of receiver clock for BDS3 constellation
 * @return        void
 */
void RTK_VerifyBDSclkValid(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock, const rtk_filterInfo_t* pz_RTKfilterInfo,
  uint8_t pu_BDS2clkStatus[C_GNSS_FREQ_TYPE_MAX], uint8_t pu_BDS3clkStatus[C_GNSS_FREQ_TYPE_MAX])
{
  uint8_t u_i = 0;
  uint8_t u_signalIndex = 0;
  uint32_t q_satIndex = 0;
  uint16_t w_iSat = 0;
  BOOL q_isBDS3Sat = FALSE;
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  uint8_t pu_BDS2clkNum[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint8_t pu_BDS3clkNum[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_BDS2clkStatus[u_i] = 0;
    pu_BDS3clkStatus[u_i] = 0;
    pu_BDS2clkNum[u_i] = 0;
    pu_BDS3clkNum[u_i] = 0;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas || (C_GNSS_BDS3 !=(pz_satMeas->u_constellation)&& C_GNSS_BDS2 != (pz_satMeas->u_constellation)))
    {
      continue;
    }
    q_isBDS3Sat = gnss_isBDS3Sat(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }
      if ((pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid) || (pz_corrMeasReslut->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      //z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      if (TRUE == q_isBDS3Sat)
      {
        ++pu_BDS3clkNum[z_freqType];
      }
      else
      {
        ++pu_BDS2clkNum[z_freqType];
      }
    }
  }
  if (TRUE == (pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      if (pu_BDS2clkNum[u_i] >= 2)
      {
        pu_BDS2clkStatus[u_i] = 1;
      }
      if (pu_BDS3clkNum[u_i] >= 2)
      {
        pu_BDS3clkStatus[u_i] = 1;
      }
    }
  }
  else
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      if ((pu_BDS2clkNum[u_i]+ pu_BDS3clkNum[u_i]) >= 2)
      {
        pu_BDS2clkStatus[u_i] = 1;
        pu_BDS3clkStatus[u_i] = 1;
      }
    }
  }
  return;
}
/**
 * @brief add and time update the receiver clock parameter in the EKF
 * @param[in]     pd_siteCoor is the initilization value of site coordination
 * @param[in/out] pz_satSigMeasCollect is the observation information
 * @param[in]     pz_rtkCorrBlock is the OSR product
 * @param[out]    pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]    pz_EKFstateRepPool is the pool of EKF state represent
 * @return        void
 */
void RTK_addRcvClkPara(const double pd_siteCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  gnss_filterInitInfo_t z_initInfo = { 0 };
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
  int16_t w_paraIndex = -1;
  double d_diff = 0.0;
  const rtk_rcv_init_clk_info_t* pz_rcvInitClkInfo = (const rtk_rcv_init_clk_info_t*)pz_RTKfilterInfo->pz_rcvInitClkInfo;
  //initialize reciver clock
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 1;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    /*if BDS2 and BDS3 are treated as one constellation,there only use C_GNSS_BDS3 to represent receiver clock of BDS */
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_i))
    {
      continue;
    }
    if (RTK_RCV_INIT_CLK_INVALID == pz_rcvInitClkInfo->pu_clkStatus[u_i])
    {
      continue;
    }
    z_initInfo.d_initValue = pz_rcvInitClkInfo->pd_initRcvClockValue[u_i];
    z_initInfo.d_sigma0 = RCV_CLK_SIGMA;
    z_initInfo.d_noise = RCV_CLK_SIGMA;
    w_id[0] = 0;
    w_id[1] = u_i;
    w_id[2] = GNSS_FILTER_STATE_CLK;
    addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
  }
  //initialize reciver DCB
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    /*if BDS2 and BDS3 are treated as one constellation,there only use C_GNSS_BDS3 to represent receiver clock of BDS */
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_i))
    {
      continue;
    }
    if (RTK_RCV_INIT_CLK_INVALID == pz_rcvInitClkInfo->pu_clkStatus[u_i])
    {
      continue;
    }
    for (u_j = 1; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      if (RTK_RCV_INIT_CLK_INVALID == pz_rcvInitClkInfo->pu_clkStatus[u_j * C_GNSS_MAX + u_i])
      {
        continue;
      }
      z_filterType = convertFreqDCB2FilterType((gnss_FreqType)u_j);
      if (GNSS_FILTER_STATE_NUM == z_filterType)
      {
        continue;
      }
      //the logic to be modified
      z_initInfo.u_resetPara = 1;
      z_initInfo.d_initValue = pz_rcvInitClkInfo->pd_initRcvClockValue[u_j * C_GNSS_MAX + u_i] - pz_rcvInitClkInfo->pd_initRcvClockValue[u_i];
      z_initInfo.d_sigma0 = RCV_DCB_SIGMA;
      z_initInfo.d_noise = RCV_DCB_SIGMA;
      w_id[0] = u_j;
      w_id[1] = u_i;
      w_id[2] = z_filterType;
      pz_ekfState = getEKF_status(w_id, (const gnss_EKFstateRepresentPool_t*)pz_EKFstateRepPool);
      if (NULL != pz_ekfState)
      {
        w_paraIndex = pz_ekfState->w_index;
        if (w_paraIndex >= 0)
        {
          d_diff = fabs(z_initInfo.d_initValue - (pz_RTKfilterInfo->pd_X[w_paraIndex]));
          if (d_diff < 20.0)
          {
            z_initInfo.u_resetPara = 0;
          }
        }
      }
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
  }
  return;
}
/**
 * @brief add and time update the receiver clock drift parameter in the EKF
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[out]    pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]    pz_EKFstateRepPool is the pool of EKF state represent
 * @return        void
 */
void RTK_addRcvClkDriftPara(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  gnss_filterInitInfo_t z_initInfo = { 0 };
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  //initialize reciver clock drift parameter
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 1;
  z_initInfo.d_initValue = 0.0;
  z_initInfo.d_sigma0 = RCV_CLK_DRIFT_SIGMA;
  z_initInfo.d_noise = RCV_CLK_DRIFT_SIGMA;
  w_id[0] = 0;
  w_id[1] = 0;
  w_id[2] = GNSS_FILTER_STATE_DRIFT;
  addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
  return;
}
/**
 * @brief calculate satellite signal carrier initial ambiguity
 * @param[in]     pd_roverSiteCoor is the initilization value of site coordination
 * @param[in]     pd_baseSiteCoor is the VRS coordination
 * @param[in]     pz_satMeas is satellite observation information
 * @param[in]     pz_sigMeas is signal observation information for target satellite
 * @param[in]     pz_corrMeas is signal correction observation for target satellite
 * @param[in]     pz_satPosClk is satellite position and clock of signal correction observation for target satellite
 * @param[in]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]    pd_initAmb is the initial ambiguity of target signal carrier phase
 * @return        TRUE represent successful and FALSE represent failure
 */
BOOL RTK_calInitAmb(const double pd_roverSiteCoor[3], const double pd_baseSiteCoor[3], const gnss_SatelliteMeas_t* pz_satMeas, const gnss_SignalMeas_t* pz_sigMeas,
  const GnssMeas_t* pz_corrMeas, const gnss_SatPosClkInfo_t* pz_satPosClk, const rtk_filterInfo_t* pz_RTKfilterInfo,
  const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, double* pd_initAmb)
{
  BOOL z_status = FALSE;
  BOOL z_obsValid = TRUE;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  double d_pseduoSatDiff = 0.0;
  double d_carrierSatDiff = 0.0;
  double pd_s2r[3] = { 0.0 };
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double d_wave = wavelength(pz_sigMeas->u_signal);
  double d_prInitAmb = 0.0;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  gnss_FreqType z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  if (fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS || fabs(pz_corrMeas->d_carrierPhase) < FABS_ZEROS
    || (pz_sigMeas->d_pseudoRange) < FABS_ZEROS || (pz_corrMeas->d_pseudoRange) < FABS_ZEROS
    || !(pz_sigMeas->z_measStatusFlag.b_prValid) || !(pz_corrMeas->z_measStatusFlag.b_valid))
  {
    z_obsValid = FALSE;
  }
  *pd_initAmb = 0.0;
  d_carrierSatDiff = (pz_sigMeas->d_carrierPhase) - (pz_corrMeas->d_carrierPhase);
  if (TRUE == z_obsValid)
  {
    d_pseduoSatDiff = (pz_sigMeas->d_pseudoRange) - (pz_corrMeas->d_pseudoRange);
    *pd_initAmb = d_carrierSatDiff - d_pseduoSatDiff / d_wave;
    z_status = TRUE;
  }
  else
  {
    getRcvClkParaIndex(pz_EKFstateRepPool, pw_rcvClkIndex);
    u_constellation = pz_sigMeas->u_constellation;
    w_rcvClkFilterIndex = pw_rcvClkIndex[u_constellation];
    w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + u_constellation];
    if (INVALID_INDEX != w_rcvClkFilterIndex && INVALID_INDEX != w_rcvDcbFilterIndex
      && fabs(pz_sigMeas->d_carrierPhase) > FABS_ZEROS && fabs(pz_corrMeas->d_carrierPhase) > FABS_ZEROS)
    {
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_satMeas->z_satPosVelClk.d_satPosClk[u_i] - pd_roverSiteCoor[u_i];
      }
      d_roverDist = gnss_Norm(pd_s2r, 3);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_satPosClk->d_satPosClk[u_i] - pd_baseSiteCoor[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      *pd_initAmb = d_carrierSatDiff - (d_roverDist - d_baseDist + pz_RTKfilterInfo->pd_X[w_rcvClkFilterIndex]) / d_wave;
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        *pd_initAmb -= pz_RTKfilterInfo->pd_X[w_rcvDcbFilterIndex] / d_wave;
      }
      z_status = TRUE;
      if ((pz_sigMeas->d_pseudoRange) > FABS_ZEROS && (pz_corrMeas->d_pseudoRange) > FABS_ZEROS
        && (pz_sigMeas->z_measStatusFlag.b_prValid) && (pz_corrMeas->z_measStatusFlag.b_valid))
      {
        d_pseduoSatDiff = (pz_sigMeas->d_pseudoRange) - (pz_corrMeas->d_pseudoRange);
        d_prInitAmb = d_carrierSatDiff - d_pseduoSatDiff / d_wave;
        if (fabs(*pd_initAmb - d_prInitAmb) > 500.0 && fabs(pz_sigMeas->d_pseudoRange) > 1e+7)
        {
          *pd_initAmb = 0.0;
          z_status = FALSE;
        }
      }
    }
  }
  return z_status;
}
/**
 * @brief malloc rtk_AMBupdatePrint_t and init
 * @param[in]  rtk_AMBupdatePrint_t is the zero AMB update print struct
 * @return     void
 */
void RTK_AMBupdatePrint(const rtk_AMBupdatePrint_t* pz_AMBprint, double d_tow, const uint8_t u_size,
  uint8_t u_allNum, uint8_t u_stableNum, uint8_t u_resetNum, uint8_t u_newNum)
{
  uint8_t u_absentNum = 0;

  if ((pz_AMBprint->pu_LackOfCP - pz_AMBprint->pu_LackOfCP_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] Lack of CP        :%s\n", pz_AMBprint->pu_LackOfCP_sat);
  }
  if ((pz_AMBprint->pu_LackOfCorr - pz_AMBprint->pu_LackOfCorr_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] Lack of Corr      :%s\n", pz_AMBprint->pu_LackOfCorr_sat);
  }
  if ((pz_AMBprint->pu_CorrSlip - pz_AMBprint->pu_CorrSlip_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] Corr Slip         :%s\n", pz_AMBprint->pu_CorrSlip_sat);
  }
  if ((pz_AMBprint->pu_InitFail - pz_AMBprint->pu_InitFail_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] AMB Init Fail     :%s\n", pz_AMBprint->pu_InitFail_sat);
  }
  if ((pz_AMBprint->pu_DCBout - pz_AMBprint->pu_DCBout_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] DCB Large         :%s\n", pz_AMBprint->pu_DCBout_sat);
  }
  if ((pz_AMBprint->pu_LLI - pz_AMBprint->pu_LLI_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] SLIP! LLI         :%s\n", pz_AMBprint->pu_LLI_sat);
  }
  if ((pz_AMBprint->pu_GF - pz_AMBprint->pu_GF_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] SLIP! GF          :%s\n", pz_AMBprint->pu_GF_sat);
  }
  if ((pz_AMBprint->pu_TDCP - pz_AMBprint->pu_TDCP_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] SLIP! TDCP        :%s\n", pz_AMBprint->pu_TDCP_sat);
  }
  if ((pz_AMBprint->pu_LackOfDetect - pz_AMBprint->pu_LackOfDetect_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] SLIP! Lack detect :%s\n", pz_AMBprint->pu_LackOfDetect_sat);
  }
  if ((pz_AMBprint->pu_Doppler - pz_AMBprint->pu_Doppler_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] SLIP! Doppler     :%s\n", pz_AMBprint->pu_Doppler_sat);
  }
  if ((pz_AMBprint->pu_newLock - pz_AMBprint->pu_newLock_sat) >= u_size)
  {
    LOGI(TAG_HPP, "[AMB UPDATE] -- NEW LOCK --    :%s\n", pz_AMBprint->pu_newLock_sat);
  }
  if (u_allNum >= (u_stableNum + u_resetNum))
  {
    u_absentNum = u_allNum - u_stableNum - u_resetNum;
  }
  LOGI(TAG_HPP, "[AMB UPDATE] OverView Num %6.1f : All=%d  Stable=%d  Reset=%d Absent=%d  New=%d \n",
    d_tow, u_allNum, u_stableNum, u_resetNum, u_absentNum, u_newNum);
  if (u_allNum > 5)
  {
    uint8_t u_i = 0;
    uint8_t u_stablePercent = 0;
    uint8_t u_resetPercent = 0;
    uint8_t u_absentPercent = 0;
    uint8_t u_newPercent = 0;
    char pu_bar[40 + 2] = { 0 };
    if (u_newNum >= u_allNum)
    {
      u_newPercent = 20;
    }
    else
    {
      u_newPercent = (uint8_t)(u_newNum / (double)u_allNum * 20 + 0.5);
    }
    u_resetPercent = (uint8_t)(u_resetNum / (double)u_allNum * 20 + 0.5);
    u_absentPercent = (uint8_t)(u_absentNum / (double)u_allNum * 20.0 + 0.5);
    if (u_absentPercent + u_resetPercent > 20)
    {
      u_stablePercent = 0;
      /*u_absentPercent = 20 - u_resetPercent;*/
    }
    else
    {
      u_stablePercent = 20 - u_absentPercent - u_resetPercent;
    }

    for (u_i = 0; u_i < u_stablePercent; u_i++)
    {
      pu_bar[u_i] = '=';
    }
    for (u_i = u_stablePercent; u_i < u_stablePercent + u_resetPercent; u_i++)
    {
      pu_bar[u_i] = '#';
    }
    for (u_i = u_stablePercent + u_resetPercent; u_i < 20; u_i++)
    {
      pu_bar[u_i] = '-';
    }
    for (u_i = 20; u_i < 20 + u_newPercent; u_i++)
    {
      pu_bar[u_i] = '+';
    }
    pu_bar[u_i] = '\0';
    LOGI(TAG_HPP, "[AMB UPDATE] OverView Bar %6.1f : %s \n", d_tow, pu_bar);
  }
}
/**
 * @brief malloc rtk_AMBupdatePrint_t and init
 * @return     rtk_AMBupdatePrint_t
 */
rtk_AMBupdatePrint_t* RTK_malloc_AMBupdatePrint()
{
  rtk_AMBupdatePrint_t* pz_AMBprint = (rtk_AMBupdatePrint_t*)OS_MALLOC(sizeof(rtk_AMBupdatePrint_t));
  memset(pz_AMBprint->pu_LLI_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_GF_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_Doppler_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_TDCP_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_LackOfDetect_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_CorrSlip_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_LackOfCP_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_LackOfCorr_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_InitFail_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_DCBout_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_AMBprint->pu_newLock_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  pz_AMBprint->pu_LLI = &pz_AMBprint->pu_LLI_sat[0];
  pz_AMBprint->pu_GF = &pz_AMBprint->pu_GF_sat[0];
  pz_AMBprint->pu_Doppler = &pz_AMBprint->pu_Doppler_sat[0];
  pz_AMBprint->pu_TDCP = &pz_AMBprint->pu_TDCP_sat[0];
  pz_AMBprint->pu_LackOfDetect = &pz_AMBprint->pu_LackOfDetect_sat[0];
  pz_AMBprint->pu_CorrSlip = &pz_AMBprint->pu_CorrSlip_sat[0];
  pz_AMBprint->pu_LackOfCP = &pz_AMBprint->pu_LackOfCP_sat[0];
  pz_AMBprint->pu_LackOfCorr = &pz_AMBprint->pu_LackOfCorr_sat[0];
  pz_AMBprint->pu_InitFail = &pz_AMBprint->pu_InitFail_sat[0];
  pz_AMBprint->pu_DCBout = &pz_AMBprint->pu_DCBout_sat[0];
  pz_AMBprint->pu_newLock = &pz_AMBprint->pu_newLock_sat[0];
  return pz_AMBprint;
}
/**
 * @brief malloc rtk_measSNRprint_t and init
 * @return     rtk_measSNRprint_t
 */
rtk_measSNRprint_t* RTK_malloc_SNRprint()
{
  rtk_measSNRprint_t* pz_SNRprint = (rtk_measSNRprint_t*)OS_MALLOC(sizeof(rtk_measSNRprint_t));
  if (pz_SNRprint == NULL)
  {
    return NULL;
  }
  memset(pz_SNRprint->pu_prn_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_SNRprint->pu_ele_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  memset(pz_SNRprint->pu_azi_sat, 0, AMB_PRINT_SIZE * sizeof(char));
  pz_SNRprint->pu_prn = pz_SNRprint->pu_prn_sat;
  pz_SNRprint->pu_ele = pz_SNRprint->pu_ele_sat;
  pz_SNRprint->pu_azi = pz_SNRprint->pu_azi_sat;
  for (uint8_t u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; u_i++)
  {
    memset(pz_SNRprint->pu_snr_sat[u_i], 0, AMB_PRINT_SIZE * sizeof(char));
    pz_SNRprint->pu_snr[u_i] = pz_SNRprint->pu_snr_sat[u_i];
  }
  return pz_SNRprint;
}
/**
 * @brief add and time update the ambiguity parameter in the EKF
 * @param[in]  pd_roverSiteCoor is the initilization value of site coordination
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR product
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void RTK_addZeroCombineSatAmbPara(const double pd_roverSiteCoor[3], const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, GnssCorrSlipFlag_t* pz_corrSlipFlag)
{
  LogLevelEnum z_logLevel = 0;
  uint8_t u_size = 15;
  uint8_t u_constellation = 0;
  uint8_t u_allNum = 0;
  uint8_t u_stableNum = 0;
  uint8_t u_resetNum = 0;
  uint8_t u_newNum = 0;
  uint8_t u_slipFlag = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_isCurrentCloseSky = 0;
  char pu_charSat[4] = { '\0' };
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  uint16_t w_i = 0;
  uint16_t w_iSat = 0;
  double d_wave = 0.0;
  double d_ambNoise = RTK_AMB_NOISE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  gnss_filterInitInfo_t z_initInfo = { 0 };
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  BOOL z_initAmbStatus = FALSE;
  double d_initAmbValue = 0.0;
  double d_tow = pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV;
  uint8_t u_sigNum = 0;
  uint8_t u_cycleSlipMask = 0;
  gnss_EKFstateRepresent_t* pz_ekfStateRep = NULL;
  rtk_AMBupdatePrint_t* pz_AMBprint = NULL;
  if (0x1 == (pz_RTKfilterInfo->t_closeSkyCount & 0x1))
  {
    d_ambNoise *= 0.1;
    u_isCurrentCloseSky = 1;
  }
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    if ((NULL == pz_EKFstateRepPool->pz_satPool[w_i]) || INVALID_INDEX == (pz_EKFstateRepPool->pz_satPool[w_i]->w_index))
    {
      continue;
    }
    if (pz_EKFstateRepPool->pz_satPool[w_i]->w_id[2] >= GNSS_FILTER_STATE_AMB_L1 &&
      pz_EKFstateRepPool->pz_satPool[w_i]->w_id[2] <= GNSS_FILTER_STATE_AMB_L5)
    {
      u_allNum++;
    }
  }
  z_logLevel = log_GetLogLevel();
  if (z_logLevel >= LOG_LEVEL_I)
  {
    pz_AMBprint = RTK_malloc_AMBupdatePrint();
  }
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    u_sigNum = RTK_signalNum(pz_satMeas);
    satidx_SatString(q_satIndex, pu_charSat);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_valid)
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      d_wave = wavelength(pz_sigMeas->u_signal);
      if (d_wave >= 1.0) // wave error
      {
        continue;
      }
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation)) // sys filter
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq)) // frq filter
      {
        continue;
      }
      z_filterType = convertFreqAmb2FilterType(z_freqType);
      if (GNSS_FILTER_STATE_NUM == z_filterType)  // AMB type
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = z_filterType;
      pz_ekfStateRep = getEKFstatusModify(w_id, pz_EKFstateRepPool);
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        if ((NULL != pz_AMBprint) && (NULL != pz_ekfStateRep) 
          && ((pz_AMBprint->pu_LackOfCorr - pz_AMBprint->pu_LackOfCorr_sat) < AMB_PRINT_SIZE - u_size))
        {
          pz_AMBprint->pu_LackOfCorr += snprintf(pz_AMBprint->pu_LackOfCorr, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
        }
        continue;
      }
      if (fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS) // carrier == 0
      {
        if ((NULL != pz_AMBprint) && (NULL != pz_ekfStateRep)
          && ((pz_AMBprint->pu_LackOfCP - pz_AMBprint->pu_LackOfCP_sat) < AMB_PRINT_SIZE - u_size))
        {
          pz_AMBprint->pu_LackOfCP += snprintf(pz_AMBprint->pu_LackOfCP, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
        }
        continue;
      }
      z_initInfo.u_resetPara = 0;
      u_slipFlag = 0;
      u_cycleSlipMask = cmn_combineCycleSlipMask(pz_RTKfilterInfo->u_tdcpMethodValid, u_sigNum,
        pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, pz_RTKfilterInfo->d_deltaTimeDopplerDetect, u_isCurrentCloseSky, pz_sigMeas->u_slipMethodValid, &u_slipFlag);
      if (1 == u_cycleSlipMask)
      {
        z_initInfo.u_resetPara = 1;
        if ((NULL != pz_AMBprint) && (NULL != pz_ekfStateRep))
        {
          if ((u_slipFlag & SLIP_FLAG_LLI)
            && ((pz_AMBprint->pu_LLI - pz_AMBprint->pu_LLI_sat) < AMB_PRINT_SIZE - u_size))
          {
            pz_AMBprint->pu_LLI += snprintf(pz_AMBprint->pu_LLI, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
          }
          if ((u_slipFlag & SLIP_FLAG_GF)
            && ((pz_AMBprint->pu_GF - pz_AMBprint->pu_GF_sat) < AMB_PRINT_SIZE - u_size))
          {
            pz_AMBprint->pu_GF += snprintf(pz_AMBprint->pu_GF, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
          }
          if ((u_slipFlag & SLIP_FLAG_DOP)
            && ((pz_AMBprint->pu_Doppler - pz_AMBprint->pu_Doppler_sat) < AMB_PRINT_SIZE - u_size))
          {
            pz_AMBprint->pu_Doppler += snprintf(pz_AMBprint->pu_Doppler, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
          }
          if ((u_slipFlag & SLIP_FLAG_TDCP)
            && ((pz_AMBprint->pu_TDCP - pz_AMBprint->pu_TDCP_sat) < AMB_PRINT_SIZE - u_size))
          {
            pz_AMBprint->pu_TDCP += snprintf(pz_AMBprint->pu_TDCP, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
          }
          if ((u_slipFlag & SLIP_FLAG_LACK)
            && ((pz_AMBprint->pu_LackOfDetect - pz_AMBprint->pu_LackOfDetect_sat) < AMB_PRINT_SIZE - u_size))
          {
            pz_AMBprint->pu_LackOfDetect += snprintf(pz_AMBprint->pu_LackOfDetect, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
          }
        }
      }
      if ((pz_corrSlipFlag->u_changeStationFlag == 0 &&
          (pz_corrSlipFlag->u_slip[q_satIndex][u_signalIndex] & (HAS_CORR_SLIP_BY_GF | HAS_CORR_SLIP_BY_TD)) != 0))
      {
        z_initInfo.u_resetPara = 1;
        if ((NULL != pz_AMBprint) && (NULL != pz_ekfStateRep)
          && ((pz_AMBprint->pu_CorrSlip - pz_AMBprint->pu_CorrSlip_sat) < AMB_PRINT_SIZE - u_size))
        {
          pz_AMBprint->pu_CorrSlip += snprintf(pz_AMBprint->pu_CorrSlip, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
        }
      }
      /*if (1 == z_initInfo.u_resetPara && !(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        LOGI(TAG_HPP, "Reset amb failed due to PR gross sat=%s, freq=%d\n", pu_charSat, z_freqType);
        continue;
      }*/
      z_initAmbStatus = RTK_calInitAmb(pd_roverSiteCoor, pz_rtkCorrBlock->d_refPosEcef, pz_satMeas, pz_sigMeas, pz_corrMeasReslut,
        &(pz_rtkCorrBlock->pz_satPosClk[w_iSat]), pz_RTKfilterInfo, pz_EKFstateRepPool, &d_initAmbValue);
      if (FALSE == z_initAmbStatus)
      {
        if ((NULL != pz_AMBprint) && (NULL != pz_ekfStateRep)
          && ((pz_AMBprint->pu_InitFail - pz_AMBprint->pu_InitFail_sat) < AMB_PRINT_SIZE - u_size))
        {
          pz_AMBprint->pu_InitFail += snprintf(pz_AMBprint->pu_InitFail, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
        }
        continue;
      }
      u_constellation = pz_sigMeas->u_constellation;
      if (fabs(pz_RTKfilterInfo->pd_phaseCodeClkDiff[u_constellation][u_signalIndex]) > GNSS_PHASE_CODE_CLK_MAX_BIAS)
      {
        z_initInfo.u_resetPara = 1;
        if ((NULL != pz_AMBprint) && (NULL != pz_ekfStateRep)
          && ((pz_AMBprint->pu_DCBout - pz_AMBprint->pu_DCBout_sat) < AMB_PRINT_SIZE - u_size))
        {
          pz_AMBprint->pu_DCBout += snprintf(pz_AMBprint->pu_DCBout, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
        }
      }
      if (NULL == pz_ekfStateRep)
      {
        u_newNum++;
        if ((NULL != pz_AMBprint) && ((pz_AMBprint->pu_newLock - pz_AMBprint->pu_newLock_sat) < AMB_PRINT_SIZE - u_size))
        {
          pz_AMBprint->pu_newLock += snprintf(pz_AMBprint->pu_newLock, u_size, "%4s.%-2d", pu_charSat, u_signalIndex);
        }
      }
      else if (1 == z_initInfo.u_resetPara)
      {
        u_resetNum++;
      }
      else
      {
        u_stableNum++;
      }
      z_initInfo.d_initValue = d_initAmbValue;
      z_initInfo.d_sigma0 = AMB_SIGMA;
      z_initInfo.d_noise = d_ambNoise;
      addState2EKFfilter(w_id, pz_RTKfilterInfo->w_nmax, &z_initInfo, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
  }
  if (u_allNum > 0)
  {
    pz_RTKfilterInfo->d_resetPercent = (double)u_resetNum / u_allNum;
  }
  if (NULL != pz_AMBprint)
  {
    RTK_AMBupdatePrint(pz_AMBprint, d_tow, 5, u_allNum, u_stableNum, u_resetNum, u_newNum);
    OS_FREE(pz_AMBprint);
  }
  return;
}
/**
 * @brief calculate the bias between carrier phase and pseudo-range
 * @param[in]     q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]     u_targetSys is the target constellation
 * @param[in]     u_targetFreq is the target frequency
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[in]     pz_rtkCorrBlock is the OSR product
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @return        the bias between carrier phase and pseudo-range
 */
double RTK_calBiasBetweenPhaseCode(BOOL q_isSperateBDS2And3, uint8_t u_targetSys, uint8_t u_targetFreq, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const rtk_filterInfo_t* pz_RTKfilterInfo)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_obsNum = 0;
  uint8_t u_status = 0;
  uint8_t u_constellation = 0;
  uint8_t u_cycleSlipMask = 0;
  uint8_t u_sigNum = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint16_t w_iSat = 0;
  int16_t w_ambIndex = -1;
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  double d_wave = 0.0;
  double pd_offset[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  double d_median = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const double* pd_X = pz_RTKfilterInfo->pd_X;
  if (0x1 == (pz_RTKfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetSys != u_constellation)
    {
      continue;
    }
    u_sigNum = RTK_signalNum(pz_satMeas);
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }
      if (fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS)
      {
        continue;
      }
      //z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != u_targetFreq)
      {
        continue;
      }
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
        || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      d_wave = wavelength(pz_sigMeas->u_signal);
      if (d_wave >= 1.0)
      {
        continue;
      }
      u_cycleSlipMask = cmn_combineCycleSlipMask(pz_RTKfilterInfo->u_tdcpMethodValid, u_sigNum,
        pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, pz_RTKfilterInfo->d_deltaTimeDopplerDetect, u_isCurrentCloseSky, pz_sigMeas->u_slipMethodValid, NULL);
      if (1 == u_cycleSlipMask)
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (w_ambIndex < 0)
      {
        continue;
      }
      pd_offset[u_obsNum] = ((pz_sigMeas->d_carrierPhase) - (pz_corrMeasReslut->d_carrierPhase) - pd_X[w_ambIndex]) - ((pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange)) / d_wave;
      ++u_obsNum;
    }
  }
  if (u_obsNum <= 1)
  {
    d_median = pd_offset[0];
  }
  else
  {
    u_status = gnss_ascSortMedianDouble(pd_offset, u_obsNum, &d_median);
  }
  //when the number of observation is too few and the difference is too large,ther will reset the bias to zero
  if (u_status)
  {
    if (u_obsNum < 5 && d_median>1000.0)
    {
      d_median = 0.0;
    }
    //for 0.2 meter satellite orbit error
    if (d_median > (5.0e-5 * CLIGHT))
    {
      d_median = 0.0;
    }
  }
  return d_median;
}
/**
 * @brief add and time update the PVA, receiver clock,ambiguity in the EKF
 * @param[in]     pd_siteCoor is the initilization value of site coordination
 * @param[in]     pd_siteVel is the initilization value of site velocity
 * @param[in/out] pz_satSigMeasCollect is the observation information
 * @param[in]     pz_rtkCorrBlock is the OSR product
 * @param[out]    pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void RTK_timeUpdateFilterPara(const double pd_siteCoor[3], const double pd_siteVel[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, GnssCorrSlipFlag_t* pz_corrSlipFlag)
{
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  BOOL q_isSperateBDS2And3 = FALSE;
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    pz_RTKfilterInfo->pq_paraValid[w_i] = FALSE;
  }
  RTK_addSiteParaPVA(pz_satSigMeasCollect->z_tor, pd_siteCoor, pd_siteVel, pz_RTKfilterInfo, pz_EKFstateRepPool);
  RTK_addRcvClkPara(pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool);
  if (1 != (pz_RTKfilterInfo->u_goodSceneStatusFlag))
  {
    RTK_addRcvClkDriftPara(pz_satSigMeasCollect, pz_RTKfilterInfo, pz_EKFstateRepPool);
  }
  for (w_j = 0; w_j < C_GNSS_FREQ_TYPE_MAX; ++w_j)
  {
    q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect((gnss_FreqType)w_j, pz_satSigMeasCollect);
    for (w_i = C_GNSS_GPS; w_i < C_GNSS_MAX; ++w_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(q_isSperateBDS2And3, (gnss_ConstellationType)w_i))
      {
        continue;
      }
      pz_RTKfilterInfo->pd_phaseCodeClkDiff[w_i][w_j] = RTK_calBiasBetweenPhaseCode(q_isSperateBDS2And3, (uint8_t)w_i, (uint8_t)w_j, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_EKFstateRepPool, pz_RTKfilterInfo);
    }
    if (FALSE == q_isSperateBDS2And3)
    {
      pz_RTKfilterInfo->pd_phaseCodeClkDiff[C_GNSS_BDS2][w_j] = pz_RTKfilterInfo->pd_phaseCodeClkDiff[C_GNSS_BDS3][w_j];
    }
  }
  RTK_addZeroCombineSatAmbPara(pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_corrSlipFlag);
  pz_EKFstateRepPool->z_gpsTime = pz_satSigMeasCollect->z_tor;
  removeEKFstatus(pz_EKFstateRepPool, pz_RTKfilterInfo->w_nmax, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q,
    pz_RTKfilterInfo->pq_paraValid);

  //clean Corr Slip Flag
  pz_corrSlipFlag->u_changeStationFlag = 0;
  for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    for (w_j = 0; w_j < MAX_GNSS_SIGNAL_FREQ; ++w_j)
    {
      pz_corrSlipFlag->u_slip[w_i][w_j] = 0;
    }
  }

  return;
}
/**
 * @brief determine the variance of observation
 * @param[in]  u_constellation is the type of constellation
 * @param[in]  z_obsType is the type of observations
 * @param[in]  f_eleRad is the elevation of satellite, unit in radius
 * @param[in]  f_age is difference age
 * @return     the variance of observation
 */
double RTK_obsVariance(uint8_t u_constellation, gnss_ObsType z_obsType, float f_eleRad, float f_age)
{
  double d_obsVariance = -1.0;
  double d_sinEle = sin((double)f_eleRad);
  double d_carrierVariance = RTK_PHASE_FACTOR_A * RTK_PHASE_FACTOR_A + RTK_PHASE_FACTOR_B * RTK_PHASE_FACTOR_B / (d_sinEle * d_sinEle + 1.0e-12);
  double d_dFactor = RTK_SAT_CLOCK_VEL_SECOND_ERR * CLIGHT * f_age;
  if (C_GNSS_OBS_TYPE_PR == z_obsType)
  {
    d_obsVariance = RTK_FACTOR_PR_RELATIVE_CR * RTK_FACTOR_PR_RELATIVE_CR * d_carrierVariance + d_dFactor * d_dFactor;
  }
  else if (C_GNSS_OBS_TYPE_CR == z_obsType)
  {
    d_obsVariance = d_carrierVariance + d_dFactor * d_dFactor;
  }
  return d_obsVariance;
}
/**
 * @brief using the PVT information to get the variance of pseudo-range
 * @param[in]  pz_pvtResult is the result of PVT
 * @param[in]  u_constellation is the type of constellation
 * @param[in]  u_svid is the satellite index of the system
 * @param[in]  u_signal is the type of signal
 * @return     the variance of pseudo-range used by PVT information
 */
double RTK_getPrVarianceUsedByPVT(const gnss_PVTResult_t* pz_pvtResult, uint8_t u_constellation, uint8_t u_svid, uint8_t u_signal)
{
  double d_prVar = -1.0;
  const gnss_MeasQos_t* pz_QOS = NULL;
  uint8_t u_iQos = 0;
  for (u_iQos = 0; u_iQos < (pz_pvtResult->u_meas_count); ++u_iQos)
  {
    pz_QOS = &(pz_pvtResult->pz_meas_qos[u_iQos]);
    if ((pz_QOS->u_constellation) == u_constellation && (pz_QOS->u_svid) == u_svid && (pz_QOS->u_signal) == u_signal)
    {
      d_prVar = (pz_QOS->d_pr_unc);
      break;
    }
  }
  return d_prVar;
}
/**
 * @brief using the PVT information to get the variance of doppler observations
 * @param[in]  pz_pvtResult is the result of PVT
 * @param[in]  u_constellation is the type of constellation
 * @param[in]  u_svid is the satellite index of the system
 * @param[in]  u_signal is the type of signal
 * @return     the variance of doppler observations used by PVT information
 */
double RTK_getDopplerVarianceUsedByPVT(const gnss_PVTResult_t* pz_pvtResult, uint8_t u_constellation, uint8_t u_svid, uint8_t u_signal)
{
  double d_drVar = -1.0;
  const gnss_MeasQos_t* pz_QOS = NULL;
  uint8_t u_iQos = 0;
  for (u_iQos = 0; u_iQos < (pz_pvtResult->u_meas_count); ++u_iQos)
  {
    pz_QOS = &(pz_pvtResult->pz_meas_qos[u_iQos]);
    if ((pz_QOS->u_constellation) == u_constellation && (pz_QOS->u_svid) == u_svid && (pz_QOS->u_signal) == u_signal)
    {
      d_drVar = (pz_QOS->d_dr_unc);
      break;
    }
  }
  return d_drVar;
}
/**
 * @brief get the index of PVA ZTD and receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_PVAindex is the index of PVA parameter in the EKF
 * @param[out] pw_rcvClkIndex is the index of receiver clock parameter in the EKF
 * @return     1 represent success and 0 represent failure
 */
uint8_t RTK_getSiteRelativeParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  int16_t pw_PVAindex[PVA_NUM], int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX])
{
  uint8_t u_i = 0;
  uint8_t u_status = 1;
  if (NULL == pw_PVAindex || NULL == pw_rcvClkIndex)
  {
    return 0;
  }
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  getRcvClkParaIndex(pz_EKFstateRepPool, pw_rcvClkIndex);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    if (pw_PVAindex[u_i] < 0)
    {
      u_status = 0;
      break;
    }
  }
  return u_status;
}
/**
 * @brief print omc residual
 * @param[in]  pz_filterObs is the filter obs information
 * @param[in]  z_residualType is the residual type, see RES_...
 * @return     void
 */
void RTK_printResidual(rtk_EpochFilterObs_t* pz_filterObs,uint16_t w_resRefCode[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ],
  uint16_t w_resRefPhase[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ], rtk_ResidualType z_residualType)
{
  uint8_t u_size = 15;
  char char_sat[4] = "";
  char buff[BUFF_SIZE] = { 0 };
  char buff_res[BUFF_SIZE] = { 0 };
  char* p = buff;
  char* p_res = buff_res;
  uint16_t w_svid = 0;
  uint32_t q_satIndex = 0;
  double d_refRes = 0.0;
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_ConstellationType z_iconstell = C_GNSS_NONE + 1;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  rtk_SatFilterObs_t* pz_SatFilterObs;

  //1.print code res
  p = &buff[0];
  p_res = &buff_res[0];
  for (z_constell = C_GNSS_GPS; z_constell < C_GNSS_MAX; z_constell++)
  {
    for (z_freqType = 0; z_freqType < MAX_GNSS_SIGNAL_FREQ; z_freqType++)
    {
      if (w_resRefCode[z_constell][z_freqType] == ALL_GNSS_SYS_SV_NUMBER ||
        NULL == pz_filterObs->pz_SatFilterObs[w_resRefCode[z_constell][z_freqType]])
      {
        continue;
      }
      else
      {
        if ((p - buff) >= BUFF_SIZE - u_size || (p_res - buff_res) >= BUFF_SIZE - u_size)
        {
          continue;
        }
        satidx_SatString((uint8_t)w_resRefCode[z_constell][z_freqType], char_sat);
        p += snprintf(p, u_size, " | %4s.%-2d", char_sat, z_freqType);
        d_refRes = pz_filterObs->pz_SatFilterObs[w_resRefCode[z_constell][z_freqType]]->pf_codeResidual[z_freqType];
        p_res += snprintf(p_res, u_size, "  |%7.2f", d_refRes);
      }
      for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
      {
        pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
        if (NULL == pz_SatFilterObs)
        {
          continue;
        }
        /*w_svid =*/ gnss_cvt_SvIndex2Svid(q_satIndex, &z_iconstell);
        if (z_iconstell != z_constell)
        {
          continue;
        }
        if (w_resRefCode[z_constell][z_freqType] != q_satIndex &&
          fabs(pz_SatFilterObs->pf_codeResidual[z_freqType]) > FABS_ZEROS)
        {
          satidx_SatString((uint8_t)q_satIndex, char_sat);
          if ((p - buff) >= BUFF_SIZE - u_size || (p_res - buff_res) >= BUFF_SIZE - u_size)
          {
            continue;
          }
          p += snprintf(p, u_size, "%4s.%-2d", char_sat, z_freqType);
          p_res += snprintf(p_res, u_size, "%7.2f", pz_SatFilterObs->pf_codeResidual[z_freqType] - d_refRes);
        }
      }
    }
  }

  LOGI(TAG_HPP, " Sat    %s\n", buff);
  LOGI(TAG_HPP, " Code  %s\n", buff_res);

  if (z_residualType == RES_RTD)
  {
    return;
  }

  //2.print phase res
  p = &buff[0];
  p_res = &buff_res[0];
  for (z_constell = C_GNSS_GPS; z_constell < C_GNSS_MAX; z_constell++)
  {
    for (z_freqType = 0; z_freqType < MAX_GNSS_SIGNAL_FREQ; z_freqType++)
    {
      if (w_resRefPhase[z_constell][z_freqType] == ALL_GNSS_SYS_SV_NUMBER ||
        NULL == pz_filterObs->pz_SatFilterObs[w_resRefPhase[z_constell][z_freqType]])
      {
        continue;
      }
      else
      {
        if ((p - buff) >= BUFF_SIZE - u_size || (p_res - buff_res) >= BUFF_SIZE - u_size)
        {
          continue;
        }
        satidx_SatString((uint8_t)w_resRefPhase[z_constell][z_freqType], char_sat);
        if (z_residualType == RES_POST &&
          FALSE == pz_filterObs->pz_SatFilterObs[w_resRefPhase[z_constell][z_freqType]]->pu_notNewLock[z_freqType])
        {
          p += snprintf(p, u_size, " | %4s*%-2d", char_sat, z_freqType);
        }
        else
        {
          p += snprintf(p, u_size, " | %4s.%-2d", char_sat, z_freqType);
        }
        d_refRes = pz_filterObs->pz_SatFilterObs[w_resRefPhase[z_constell][z_freqType]]->pf_phaseResidual[z_freqType];
        p_res += snprintf(p_res, u_size, "  |%7.2f", d_refRes);
      }
      for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
      {
        pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
        if (NULL == pz_SatFilterObs)
        {
          continue;
        }
        /*w_svid =*/ gnss_cvt_SvIndex2Svid(q_satIndex, &z_iconstell);
        if (z_iconstell != z_constell)
        {
          continue;
        }
        if (w_resRefPhase[z_constell][z_freqType] != q_satIndex &&
          fabs(pz_SatFilterObs->pf_phaseResidual[z_freqType]) > FABS_ZEROS)
        {
          satidx_SatString((uint8_t)q_satIndex, char_sat);
          if ((p - buff) >= BUFF_SIZE - u_size || (p_res - buff_res) >= BUFF_SIZE - u_size)
          {
            continue;
          } 
          if (z_residualType == RES_POST &&
            FALSE == pz_SatFilterObs->pu_notNewLock[z_freqType])
          {
            p += snprintf(p, u_size, "%4s*%-2d", char_sat, z_freqType);
          }
          else
          {
            p += snprintf(p, u_size, "%4s.%-2d", char_sat, z_freqType);
          }
          p_res += snprintf(p_res, u_size, "%7.2f", pz_SatFilterObs->pf_phaseResidual[z_freqType] - d_refRes);
        }
      }
    }
  }

  LOGI(TAG_HPP, " Sat    %s\n", buff);
  LOGI(TAG_HPP, " Phase %s\n", buff_res);
}
/**
 * @brief fillup omc residual
 * @param[in]  pz_resCalInput is input information of calculated reidual
 * @param[out] pw_resRefCode is code reference satellite of residual
 * @param[out] pw_resRefPhase is phase reference satellite of residual
 * @return    TRUE represent success, FALSE represent failture
 */
BOOL RTK_fillupPseudoPhaseObsResidual(RTK_resCalInputInfo_t* pz_resCalInput, uint16_t pw_resRefCode[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ], uint16_t pw_resRefPhase[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ])
{
  uint8_t u_i = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_iSat = 0;
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  int16_t w_ambIndex = -1;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  double pd_siteCoor[3] = { 0.0 };
  rtk_SatFilterObs_t* pz_SatFilterObs = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const double* pd_satPosClk = NULL;
  char* pu_resType[RES_MAX] = { "RES_PRIOR","RES_POST","RES_FIXED","RES_PREFIX","RES_RTD" };
  double d_refEleCode[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ] = { {0} };
  double d_refElePhase[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ] = { {0} };
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double d_baseEarthRot = 0.0;
  double d_wave = 0.0;
  double d_omc = 0.0;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  const double d_timeLock = 0.1; //in second
  const double* pd_baseSatPosClk = NULL;
  double pd_s2r[3] = { 0.0 };
  if (!RTK_getSiteRelativeParaIndex(pz_resCalInput->pz_EKFstateRepPool, pw_PVAindex, pw_rcvClkIndex))
  {
    return FALSE;
  }
  if (FALSE == (pz_resCalInput->pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  /* site coordinate */
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pz_resCalInput->pd_X[pw_PVAindex[u_i]];
  }
  for (z_constell = C_GNSS_GPS; z_constell < C_GNSS_MAX; ++z_constell)
  {
    for (z_freqType = 0; z_freqType < MAX_GNSS_SIGNAL_FREQ; ++z_freqType)
    {
      pw_resRefCode[z_constell][z_freqType] = ALL_GNSS_SYS_SV_NUMBER;
      pw_resRefPhase[z_constell][z_freqType] = ALL_GNSS_SYS_SV_NUMBER;
    }
  }
  if (pz_resCalInput->u_residualType == RES_PREFIX)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_siteCoor[u_i] = pz_resCalInput->pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_PREFIX][u_i];
    }
  }
  else if (pz_resCalInput->u_residualType == RES_FIXED)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_siteCoor[u_i] = pz_resCalInput->pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_FIX][u_i];
    }
  }
  if (gnss_Dot(pd_siteCoor, pd_siteCoor, 3) < FABS_ZEROS * FABS_ZEROS)
  {
    LOGI(TAG_HPP, "-- %s -- tow=%.2f | coor is NULL\n", pu_resType[pz_resCalInput->u_residualType],
      pz_resCalInput->pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV);
    return FALSE;
  }
  else
  {
    LOGI(TAG_HPP, "-- %s -- tow=%.2f | coor=%.2f,%.2f,%.2f\n", pu_resType[pz_resCalInput->u_residualType],
      pz_resCalInput->pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV, pd_siteCoor[0], pd_siteCoor[1], pd_siteCoor[2]);
  }
  //1.2 clean res
  pz_resCalInput->pz_filterObs->w_numPhaseRes = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_SatFilterObs = pz_resCalInput->pz_filterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      //clean
      pz_SatFilterObs->pf_phaseResidual[u_signalIndex] = 0.0;
      pz_SatFilterObs->pf_codeResidual[u_signalIndex] = 0.0;
    }
  }
  pz_resCalInput->pz_filterObs->z_resType = pz_resCalInput->u_residualType;
  //2. traverse all observations
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_resCalInput->pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_resCalInput->pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    pz_SatFilterObs = pz_resCalInput->pz_filterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      //meas valid check
      if ((pz_resCalInput->u_residualType == RES_POST || pz_resCalInput->u_residualType == RES_RTD)
        && (NULL == pz_SatFilterObs->z_measUpdateFlag[u_signalIndex] ||
          !pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_valid))
      {
        continue;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid))
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_resCalInput->pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_resCalInput->pz_rtkCorrBlock->pz_satPosClk, pz_resCalInput->pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_resCalInput->pz_rtkCorrBlock->w_satCount) || (pz_resCalInput->pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_resCalInput->pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_resCalInput->pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_resCalInput->pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
      if (INVALID_INDEX == w_rcvDcbFilterIndex) // no clk
      {
        continue;
      }
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
        continue;
      }
      if (pz_resCalInput->u_residualType == RES_PREFIX && pz_SatFilterObs->pu_preFixUpdateFlag[u_signalIndex] == 0)
      {
        continue;
      }

      //2.1 calcu geo dist
      pd_baseSatPosClk = pz_resCalInput->pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk;
      for (u_i = 0; u_i < 3; u_i++)
      {
        pd_s2r[u_i] = pd_baseSatPosClk[u_i] - pz_resCalInput->pz_rtkCorrBlock->d_refPosEcef[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      d_baseEarthRot = (pd_baseSatPosClk[0] * pz_resCalInput->pz_rtkCorrBlock->d_refPosEcef[1] -
        pd_baseSatPosClk[1] * pz_resCalInput->pz_rtkCorrBlock->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;

      d_wave = wavelength(pz_sigMeas->u_signal);
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_resCalInput->pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (pz_resCalInput->u_residualType == RES_POST &&
        ((!pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_cpValid) ||
          (pz_SatFilterObs->pf_phaseVar[u_signalIndex] <= 0.0)))
      {
        //continue;
      }
      else if (!(pz_resCalInput->u_residualType == RES_RTD ||
        fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS || d_wave >= 1.0 || w_ambIndex < 0 ||
        fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS))
      {
        /* OMC */
        d_omc = ((pz_sigMeas->d_carrierPhase) - (pz_corrMeasReslut->d_carrierPhase)) * d_wave - (d_roverDist - (d_baseDist + d_baseEarthRot));
        d_omc += (pd_satPosClk[3] - pd_baseSatPosClk[3]);

        if (pz_resCalInput->u_residualType == RES_FIXED || pz_resCalInput->u_residualType == RES_PREFIX)
        {
          pz_fixedAmbSet = pz_resCalInput->pz_preFixedAmbPool->pz_fixedAmbSet[q_satIndex];
          if (NULL == pz_fixedAmbSet)
          {
            continue;
          }
          if (pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_NONE || pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_FLOAT)
          {
            continue;
          }
          d_omc -= d_wave * pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex]; // amb
        }
        else
        {
          d_omc -= d_wave * pz_resCalInput->pd_X[w_ambIndex]; // amb
        }

        d_omc -= pz_resCalInput->pd_X[w_rcvClkFilterIndex]; // clock
        if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
        {
          d_omc -= pz_resCalInput->pd_X[w_rcvDcbFilterIndex]; // dcb
        }
        //update sat filter info
        pz_SatFilterObs->pf_phaseResidual[u_signalIndex] = (float)d_omc;
        pz_satPool = getEKFstatusModify(w_id, pz_resCalInput->pz_EKFstateRepPool);
        if (pz_resCalInput->u_residualType == RES_POST && NULL != pz_satPool &&
          tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) > d_timeLock)
        {
          pz_SatFilterObs->pu_notNewLock[u_signalIndex] = TRUE;
        }
        if (pw_resRefPhase[z_constell][z_freqType] == ALL_GNSS_SYS_SV_NUMBER ||
          pz_satMeas->z_satPosVelClk.f_elevation > d_refElePhase[z_constell][z_freqType])
        {
          d_refElePhase[z_constell][z_freqType] = pz_satMeas->z_satPosVelClk.f_elevation;
          pw_resRefPhase[z_constell][z_freqType] = q_satIndex;
        }
      }

      //2.3 calcu code v=l-hx
      if ((pz_resCalInput->u_residualType == RES_POST || pz_resCalInput->u_residualType == RES_RTD) &&
        ((!pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_prValid) ||
          (pz_SatFilterObs->pf_codeVar[u_signalIndex] <= 0.0)))
      {
        continue;
      }
      if (!(fabs(pz_sigMeas->d_pseudoRange) < FABS_ZEROS || fabs(pz_corrMeasReslut->d_pseudoRange) < FABS_ZEROS))
      {
        if (!(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
        {
          continue;
        }
        if (!(pz_corrMeasReslut->z_measStatusFlag.b_valid) || (pz_corrMeasReslut->d_pseudoRange) < 1.0e-2)
        {
          continue;
        }
        d_omc = ((pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange)) - (d_roverDist - (d_baseDist + d_baseEarthRot));
        d_omc += (pd_satPosClk[3] - pd_baseSatPosClk[3]);
        d_omc -= pz_resCalInput->pd_X[w_rcvClkFilterIndex]; // clock
        if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
        {
          d_omc -= pz_resCalInput->pd_X[w_rcvDcbFilterIndex]; // dcb
        }
        pz_SatFilterObs->pf_codeResidual[u_signalIndex] = (float)d_omc;
        if (pw_resRefCode[z_constell][z_freqType] == ALL_GNSS_SYS_SV_NUMBER ||
          pz_satMeas->z_satPosVelClk.f_elevation > d_refEleCode[z_constell][z_freqType])
        {
          d_refEleCode[z_constell][z_freqType] = pz_satMeas->z_satPosVelClk.f_elevation;
          pw_resRefCode[z_constell][z_freqType] = q_satIndex;
        }
      }
    }
  }
  return TRUE;
}
/**
 * @brief statistic residual std
 * @param[in]      pw_resRefCode is code reference satellite of residual
 * @param[in]      pw_resRefPhase is phase reference satellite of residual
 * @param[in/out]  pz_resCalInput is input information of calculated reidual
 * @param[out]     pz_resStatistic is statistic information of calculated reidual
 * @return    the number of residual for phase observation
 */
uint16_t RTK_statisticResStd(const uint16_t pw_resRefCode[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ], const uint16_t pw_resRefPhase[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ],
  RTK_resCalInputInfo_t* pz_resCalInput, RTK_resStatisticInfo_t* pz_resStatistic)
{
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  uint8_t u_systemNumCode = 0;
  uint8_t u_systemNumPhase = 0;
  uint16_t w_resNumCode = 0;
  uint16_t w_resNumPhase = 0;
  uint16_t w_resNumCode_temp = 0;
  uint16_t w_resNumPhase_temp = 0;
  uint16_t w_numOverQuarter = 0;
  uint32_t q_satIndex = 0;
  uint16_t w_svid = 0;
  uint16_t w_i = 0;
  gnss_ConstellationType z_iconstell = C_GNSS_NONE;
  double d_stdCode_temp = 0.0;
  double d_stdPhase_temp = 0.0;
  double d_omc = 0.0;
  double d_maxCode = 0.0;
  double d_maxPhase = 0.0;
  double d_stdCode = 0.0;
  double d_stdPhase = 0.0;
  rtk_SatFilterObs_t* pz_SatFilterObs = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  char* pu_resType[RES_MAX] = { "RES_PRIOR","RES_POST","RES_FIXED","RES_PREFIX","RES_RTD" };
  pz_resStatistic->d_lockSum = 0.0;
  pz_resStatistic->d_allCn0 = 0.0;
  for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
  {
    pz_resStatistic->u_validSat[w_i] = 0;
  }
  for (w_i = 0; w_i < MAX_GNSS_SIGNAL_FREQ; ++w_i)
  {
    pz_resStatistic->u_resNumCodeArray[w_i] = 0;
    pz_resStatistic->u_resNumPhaseArray[w_i] = 0;
  }
  for (z_constell = C_GNSS_GPS; z_constell < C_GNSS_MAX; z_constell++)
  {
    for (z_freqType = 0; z_freqType < MAX_GNSS_SIGNAL_FREQ; z_freqType++)
    {
      w_resNumCode_temp = 0;
      w_resNumPhase_temp = 0;
      d_stdCode_temp = 0.0;
      d_stdPhase_temp = 0.0;
      for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
      {
        pz_SatFilterObs = pz_resCalInput->pz_filterObs->pz_SatFilterObs[q_satIndex];
        if (NULL == pz_SatFilterObs)
        {
          continue;
        }
        /*w_svid =*/ gnss_cvt_SvIndex2Svid(q_satIndex, &z_iconstell);
        if (z_iconstell != z_constell)
        {
          continue;
        }
        if (pw_resRefCode[z_constell][z_freqType] != ALL_GNSS_SYS_SV_NUMBER)
        {
          if (pw_resRefCode[z_constell][z_freqType] != q_satIndex && fabs(pz_SatFilterObs->pf_codeResidual[z_freqType]) > FABS_ZEROS)
          {
            d_omc = pz_resCalInput->pz_filterObs->pz_SatFilterObs[pw_resRefCode[z_constell][z_freqType]]->pf_codeResidual[z_freqType];
            pz_resStatistic->d_allCn0 += (double)pz_resCalInput->pz_satSigMeasCollect->pz_satMeas[q_satIndex]->pz_signalMeas[z_freqType]->f_cn0;
            d_stdCode_temp += (double)(pz_SatFilterObs->pf_codeResidual[z_freqType] - d_omc) * (pz_SatFilterObs->pf_codeResidual[z_freqType] - d_omc);
            if (fabs(pz_SatFilterObs->pf_codeResidual[z_freqType] - d_omc) > d_maxCode)
            {
              d_maxCode = fabs(pz_SatFilterObs->pf_codeResidual[z_freqType] - d_omc);
            }
            pz_resStatistic->u_validSat[q_satIndex] = 1;
            w_resNumCode_temp++;
            pz_resStatistic->u_resNumCodeArray[z_freqType]++;
          }
        }
        if (pw_resRefPhase[z_constell][z_freqType] != ALL_GNSS_SYS_SV_NUMBER)
        {
          if (pw_resRefPhase[z_constell][z_freqType] != q_satIndex && fabs(pz_SatFilterObs->pf_phaseResidual[z_freqType]) > FABS_ZEROS)
          {
            d_omc = pz_resCalInput->pz_filterObs->pz_SatFilterObs[pw_resRefPhase[z_constell][z_freqType]]->pf_phaseResidual[z_freqType];
            d_stdPhase_temp += (double)(pz_SatFilterObs->pf_phaseResidual[z_freqType] - d_omc) * (pz_SatFilterObs->pf_phaseResidual[z_freqType] - d_omc);
            if (fabs(pz_SatFilterObs->pf_phaseResidual[z_freqType] - d_omc) > d_maxPhase)
            {
              d_maxPhase = fabs(pz_SatFilterObs->pf_phaseResidual[z_freqType] - d_omc);
            }
            if (fabs(pz_SatFilterObs->pf_phaseResidual[z_freqType] - d_omc) > 0.055)
            {
              w_numOverQuarter++;
            }
            pz_resStatistic->d_allCn0 += (double)pz_resCalInput->pz_satSigMeasCollect->pz_satMeas[q_satIndex]->pz_signalMeas[z_freqType]->f_cn0;
            pz_resStatistic->u_validSat[q_satIndex] = 1;
            /* Lock time & cn0, rtk uncertainty factor*/
            w_id[0] = 0;
            w_id[1] = q_satIndex;
            w_id[2] = convertFreqAmb2FilterType(z_freqType);
            pz_satPool = getEKF_status(w_id, pz_resCalInput->pz_EKFstateRepPool);
            if (pz_satPool != NULL)
            {
              double d_lock = tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime);
              pz_resStatistic->d_lockSum += d_lock > 20.0 ? 20.0 : d_lock;
            }
            w_resNumPhase_temp++;
            pz_resStatistic->u_resNumPhaseArray[z_freqType]++;
          }
        }
      }
      w_resNumCode += w_resNumCode_temp;
      w_resNumPhase += w_resNumPhase_temp;
      d_stdCode += d_stdCode_temp;
      d_stdPhase += d_stdPhase_temp;
      if (pw_resRefPhase[z_constell][z_freqType] != ALL_GNSS_SYS_SV_NUMBER)
      {
        u_systemNumPhase++;
      }
      if (pw_resRefCode[z_constell][z_freqType] != ALL_GNSS_SYS_SV_NUMBER)
      {
        u_systemNumCode++;
      }
      if (pz_resCalInput->u_residualType == RES_POST && w_resNumPhase_temp > 1)
      {
        pz_resCalInput->pz_filterObs->w_groupNUM[z_constell][z_freqType] = w_resNumPhase_temp;
        pz_resCalInput->pz_filterObs->d_groupSTD[z_constell][z_freqType] = sqrt(d_stdPhase_temp / (w_resNumPhase_temp - 1));
      }
    }
  }
  //3.1 save & print std
  if (w_resNumCode > 1)
  {
    d_stdCode = sqrt(d_stdCode / (w_resNumCode - 1));
    switch (pz_resCalInput->u_residualType)
    {
    case(RES_POST):
      pz_resCalInput->pz_filterObs->d_postCodeSTD = d_stdCode; break;
    case(RES_PRIOR):
      pz_resCalInput->pz_filterObs->d_priorCodeSTD = d_stdCode; break;
    case(RES_FIXED):
      pz_resCalInput->pz_filterObs->d_fixedCodeSTD = d_stdCode; break;
    case(RES_PREFIX):
      pz_resCalInput->pz_filterObs->d_prefixCodeSTD = d_stdCode; break;
    case(RES_RTD):
      pz_resCalInput->pz_filterObs->d_rtdCodeSTD = d_stdCode; break;
    }
    pz_resCalInput->pz_filterObs->d_maxCodeBias = d_maxCode;
    LOGI(TAG_HPP, "-- %s -- num=%d sys=%d | code  std=%.3f max=%.3f\n", pu_resType[pz_resCalInput->u_residualType], w_resNumCode, u_systemNumCode, d_stdCode, d_maxCode);
  }
  else
  {
    LOGI(TAG_HPP, "-- %s -- code res is NULL\n", pu_resType[pz_resCalInput->u_residualType]);
  }
  if (w_resNumPhase > 1)
  {
    pz_resCalInput->pz_filterObs->w_numPhaseRes = w_resNumPhase;
    pz_resCalInput->pz_filterObs->w_numOverQuarter = w_numOverQuarter;
    d_stdPhase = sqrt(d_stdPhase / (w_resNumPhase - 1));
    switch (pz_resCalInput->u_residualType)
    {
    case(RES_POST):
      pz_resCalInput->pz_filterObs->d_postPhaseSTD = d_stdPhase; break;
    case(RES_PRIOR):
      pz_resCalInput->pz_filterObs->d_priorPhaseSTD = d_stdPhase; break;
    case(RES_FIXED):
      pz_resCalInput->pz_filterObs->d_fixedPhaseSTD = d_stdPhase; break;
    case(RES_PREFIX):
      pz_resCalInput->pz_filterObs->d_prefixPhaseSTD = d_stdPhase; break;
    }
    pz_resCalInput->pz_filterObs->d_maxPhaseBias = d_maxPhase;
    LOGI(TAG_HPP, "-- %s -- num=%d sys=%d | phase std=%.3f max=%.3f\n", pu_resType[pz_resCalInput->u_residualType], w_resNumPhase, u_systemNumPhase, d_stdPhase, d_maxPhase);
  }
  else if (pz_resCalInput->u_residualType != RES_RTD)
  {
    LOGI(TAG_HPP, "-- %s -- phase res is NULL\n", pu_resType[pz_resCalInput->u_residualType]);
  }
  return w_resNumPhase;
}
/**
 * @brief calculate omc residual
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pz_preFixedAmbPool is the FIXED AMB, could be NULL in float case
 * @param[out] pz_filterObs is the filter obs information
 * @param[in]  z_residualType is the residual type, see RES_...
 * @return    the number of Residuals
 */
uint8_t RTK_calculateResidual(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs, rtk_ResidualType z_residualType)
{
  uint16_t w_resNumPhase = 0;
  uint16_t w_resRefCode[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ];
  uint16_t w_resRefPhase[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ];
  LogLevelEnum z_logLevel;
  RTK_resCalInputInfo_t z_resCalInputInfo = { 0 };
  RTK_resStatisticInfo_t z_resStatistic = { 0 };
  z_resCalInputInfo.pd_X = pd_X;
  z_resCalInputInfo.pz_satSigMeasCollect = pz_satSigMeasCollect;
  z_resCalInputInfo.pz_rtkCorrBlock = pz_rtkCorrBlock;
  z_resCalInputInfo.pz_RTKfilterInfo = pz_RTKfilterInfo;
  z_resCalInputInfo.pz_EKFstateRepPool = pz_EKFstateRepPool;
  z_resCalInputInfo.pz_preFixedAmbPool = pz_preFixedAmbPool;
  z_resCalInputInfo.pz_filterObs = pz_filterObs;
  z_resCalInputInfo.u_residualType = z_residualType;
  //1.1 get the site state index in the filter & init
  if ((z_residualType == RES_FIXED || z_residualType == RES_PREFIX) && NULL == pz_preFixedAmbPool)
  {
    return 0;
  }
  pz_filterObs->w_numOverQuarter = 0;
  if (FALSE == RTK_fillupPseudoPhaseObsResidual(&z_resCalInputInfo, w_resRefCode, w_resRefPhase))
  {
    return 0;
  }
  //3.1 print Residual 
  z_logLevel = log_GetLogLevel();
  if (z_logLevel >= LOG_LEVEL_I)
  {
    RTK_printResidual(pz_filterObs, w_resRefCode, w_resRefPhase, z_residualType);
  }
  //3.2 statistic std of post res
  w_resNumPhase = RTK_statisticResStd(w_resRefCode, w_resRefPhase, &z_resCalInputInfo, &z_resStatistic);

  /* Uncertainty factor */
  rtk_integ_save_residualPost(z_residualType, z_resStatistic.d_lockSum, z_resStatistic.d_allCn0, z_resStatistic.u_resNumCodeArray, z_resStatistic.u_resNumPhaseArray,
    z_resStatistic.u_validSat, pz_RTKfilterInfo, pz_filterObs, pz_preFixedAmbPool, pz_RTKfilterInfo->pz_integ);

  return (uint8_t)w_resNumPhase;
}

/**
 * @brief calculate omc residual
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pz_preFixedAmbPool is the FIXED AMB, could be NULL in float case
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of unreset Residuals
 */
uint8_t RTK_priorResidualCheck(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  uint8_t u_resetNum = 0;
  uint16_t w_i = 0;
  uint16_t w_resNumPhase = 0;
  double pd_HPREF[3] = { 0 };
  /* threshold define begin */
  uint8_t u_resNumPhaseThres = 10;
  double d_resThresholdFoctor = 1.0;
  /* threshold define end */
#ifdef HPREF
  if (cmn_highPrecision_search(pz_satSigMeasCollect->z_tor, pd_HPREF))
  {
    double *pd_Xx = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
    if (NULL == pd_Xx)
    {
      LOGW(TAG_HPP, "%s filter pd_X cant malloc %d bytes\n", __FUNCTION__, pz_RTKfilterInfo->w_nmax * sizeof(double));
    }
    else
    {
      for (w_i = 0; w_i < (pz_RTKfilterInfo->w_nmax); ++w_i)
      {
        pd_Xx[w_i] = (pd_X[w_i]);
      }
      for (w_i = 0; w_i < 3; w_i++)
      {
        pd_Xx[w_i] = (pd_HPREF[w_i]);
      }
    }
    LOGI(TAG_HPP, "______ HPREF prior start ______\n");
    /* Calculate prior residual */
    w_resNumPhase = RTK_calculateResidual(pd_Xx, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
                                          pz_EKFstateRepPool, pz_preFixedAmbPool, pz_filterObs, RES_PRIOR);
    LOGI(TAG_HPP, "______ HPREF prior  end  ______\n");
    OS_FREE(pd_Xx);
  }
#endif
  /* Calculate prior residual */
  w_resNumPhase = RTK_calculateResidual(pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
                                        pz_EKFstateRepPool, pz_preFixedAmbPool, pz_filterObs, RES_PRIOR);

  /* Check every phase's res, AMB corresponding to out-threshold phase res will be reset */
  /* need position accuracy (update in futrue) */
  /*if (w_resNumPhase > u_resNumPhaseThres)
  {
  }*/
  return w_resNumPhase - u_resetNum;
}

/**
 * @brief calculate omc residual
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pz_preFixedAmbPool is the FIXED AMB, could be NULL in float case
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of un_reset AMB
 */
uint8_t RTK_postResidualCheck(double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  uint8_t u_i = 0;
  uint8_t u_resetNum = 0;
  uint8_t u_resetNumTemp = 0;
  uint8_t u_resNum = 0;
  char char_sat[4] = "";
  char char_sys[4] = "";
  uint16_t w_i = 0;
  uint16_t w_resNumPhase = 0;
  uint16_t w_svid = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  uint32_t q_satIndex = 0;
  uint32_t q_res_satIndex[MAX_SV_NUM_IN_ONE_SYS] = { 0 };
  double d_wave = 0.0;
  double d_eleFactor = 0.0;
  double d_resMedian = 0.0;
  double d_resMedianAbs = 0.0;
  double pd_HPREF[3] = { 0.0 };
  double d_res[MAX_SV_NUM_IN_ONE_SYS] = { 0.0 };
  double d_resMAD[MAX_SV_NUM_IN_ONE_SYS] = { 0.0 };
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_ConstellationType z_iconstell = C_GNSS_NONE + 1;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  rtk_SatFilterObs_t* pz_SatFilterObs;
  gnss_EKFstateRepresent_t* pz_EKFstateRepresent;
  /* threshold define begin */
  uint8_t u_resNumPhaseThres = 12;
  double d_timeLock = 0.1; //in second
  double d_resMADthreshold = 0.32; //in cycle
  double d_resThreshold = 0.65; //in cycle
  double d_resMADthreshold_reset = 0.6; //in cycle
  double d_resThreshold_reset = 1.2; //in cycle
  /* threshold define end */
#ifdef HPREF
  if (cmn_highPrecision_search(pz_satSigMeasCollect->z_tor, pd_HPREF))
  {
    double* pd_Xx = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
    if (NULL == pd_Xx)
    {
      LOGW(TAG_HPP, "%s filter pd_X cant malloc %d bytes\n", __FUNCTION__, pz_RTKfilterInfo->w_nmax * sizeof(double));
    }
    else
    {
      for (w_i = 0; w_i < (pz_RTKfilterInfo->w_nmax); ++w_i)
      {
        pd_Xx[w_i] = (pd_X[w_i]);
      }
      for (w_i = 0; w_i < 3; w_i++)
      {
        pd_Xx[w_i] = (pd_HPREF[w_i]);
      }
    }
    /* Calculate post residual */
    LOGI(TAG_HPP, "______ HPREF post start ______\n");
    w_resNumPhase = RTK_calculateResidual(pd_Xx, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
                                        pz_EKFstateRepPool, pz_preFixedAmbPool, pz_filterObs, RES_POST);
    LOGI(TAG_HPP, "______ HPREF post  end  ______\n");
    OS_FREE(pd_Xx);
  }
#endif
  /* Calculate post residual */
  w_resNumPhase = RTK_calculateResidual(pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
                                        pz_EKFstateRepPool, pz_preFixedAmbPool, pz_filterObs, RES_POST);

  /* Check every phase's res, AMB corresponding to out-threshold phase res will be reset */
  if (w_resNumPhase < u_resNumPhaseThres)
  {
    return (uint8_t)w_resNumPhase;
  } 
  for (z_constell = C_GNSS_GPS; z_constell < C_GNSS_MAX; z_constell++)
  {
    for (z_freqType = 0; z_freqType < MAX_GNSS_SIGNAL_FREQ; z_freqType++)
    {
      u_resNum = 0;
      d_wave = wavelength(gnss_FreqIdx2Signal(z_freqType, z_constell));
      if (d_wave >= 1.0)
      {
        continue;
      }
      for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
      {
        pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
        if (NULL == pz_SatFilterObs)
        {
          continue;
        }
        /*w_svid =*/ gnss_cvt_SvIndex2Svid(q_satIndex, &z_iconstell);
        if (z_iconstell != z_constell)
        {
          continue;
        }
        w_id[0] = 0;
        w_id[1] = q_satIndex;
        w_id[2] = convertFreqAmb2FilterType(z_freqType);
        pz_EKFstateRepresent = getEKFstatusModify(w_id, pz_EKFstateRepPool);
        if (NULL == pz_EKFstateRepresent ||
          tm_GpsTimeDiff(&pz_EKFstateRepresent->z_endTime, &pz_EKFstateRepresent->z_beginTime) < d_timeLock)
        {
          continue;
        }
        if (fabs(pz_SatFilterObs->pf_phaseResidual[z_freqType]) > FABS_ZEROS)
        {
          d_res[u_resNum] = pz_SatFilterObs->pf_phaseResidual[z_freqType];
          q_res_satIndex[u_resNum] = q_satIndex;
          u_resNum++;
        }
      }
      if (u_resNum > 1)
      {
        gnss_MadDouble(d_res, u_resNum, d_resMAD, &d_resMedian, &d_resMedianAbs);
        if (d_resMedianAbs > d_resMADthreshold_reset * d_wave)
        {
          u_resetNumTemp = 0;
          for (u_i = 0; u_i < u_resNum; u_i++)
          {
            w_id[0] = 0;
            w_id[1] = q_res_satIndex[u_i];
            w_id[2] = convertFreqAmb2FilterType(z_freqType);
            removeTargetEKFstatus(w_id, pz_EKFstateRepPool, pz_RTKfilterInfo->w_nmax,
              pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid);
            u_resetNumTemp++;
          }
          satidx_SatString((uint8_t)q_res_satIndex[0], char_sat);
          LOGI(TAG_HPP, "-\\Post Res Exceed Thres/- -\\SYS MAD Reject: Reset/- ref=%s freq=%d nsat=%d\n",
            char_sat, z_freqType, u_resetNumTemp);
          u_resetNum += u_resetNumTemp;
        }
        else if (d_resMedianAbs > d_resMADthreshold * d_wave)
        {
          u_resetNumTemp = 0;
          for (u_i = 0; u_i < u_resNum; u_i++)
          {
            w_id[0] = 0;
            w_id[1] = q_res_satIndex[u_i];
            w_id[2] = convertFreqAmb2FilterType(z_freqType);
            pz_EKFstateRepresent = getEKFstatusModify(w_id, pz_EKFstateRepPool);
            if (NULL != pz_EKFstateRepresent)
            {
              memcpy(&pz_EKFstateRepresent->z_beginTime, &pz_EKFstateRepresent->z_endTime, sizeof(GpsTime_t));
            }
            u_resetNumTemp++;
          }
          satidx_SatString((uint8_t)q_res_satIndex[0], char_sat);
          LOGI(TAG_HPP, "-\\Post Res Exceed Thres/- -\\SYS MAD Reject: Exclude fix sol./- ref=%s freq=%d nsat=%d\n",
            char_sat, z_freqType, u_resetNumTemp);
          u_resetNum += u_resetNumTemp;
        }
        else
        {
          for (u_i = 0; u_i < u_resNum; u_i++)
          {
            if (d_resMAD[u_i] > d_resThreshold_reset * d_wave)
            {
              w_id[0] = 0;
              w_id[1] = q_res_satIndex[u_i];
              w_id[2] = convertFreqAmb2FilterType(z_freqType);
              removeTargetEKFstatus(w_id, pz_EKFstateRepPool, pz_RTKfilterInfo->w_nmax,
                pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid);
              u_resetNum++;
              satidx_SatString((uint8_t)q_res_satIndex[u_i], char_sat);
              LOGI(TAG_HPP, "-\\Post Res Exceed Thres/- -\\Sat Reject: Reset/- sat=%s.%d res=%lf\n",
                char_sat, z_freqType, d_resMAD[u_i]);
            }
            else if (d_resMAD[u_i] > d_resThreshold * d_wave)
            {
              w_id[0] = 0;
              w_id[1] = q_res_satIndex[u_i];
              w_id[2] = convertFreqAmb2FilterType(z_freqType);
              pz_EKFstateRepresent = getEKFstatusModify(w_id, pz_EKFstateRepPool);
              if (NULL != pz_EKFstateRepresent)
              {
                memcpy(&pz_EKFstateRepresent->z_beginTime, &pz_EKFstateRepresent->z_endTime, sizeof(GpsTime_t));
              }
              u_resetNum++;
              satidx_SatString((uint8_t)q_res_satIndex[u_i], char_sat);
              LOGI(TAG_HPP, "-\\Post Res Exceed Thres/- -\\Sat Reject: Exclude fix sol./- sat=%s.%d res=%lf\n", 
                char_sat, z_freqType, d_resMAD[u_i]);
            }
          }
        }
      }
    }
  }
  return w_resNumPhase - u_resetNum;
}

/**
 * @brief using the zero-combine phase observations to update the EKF
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of updated carrier observations
 */
uint8_t RTK_zeroCombinePhaseMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  uint8_t u_measUpdateCarrierNum = 0;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  uint8_t u_i = 0;
  uint8_t u_MPflag = 0;
  uint8_t u_seqStatus = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  int16_t w_ambIndex = -1;
  uint16_t w_iSat = 0;
  rtk_SatFilterObs_t* pz_SatFilterObs;
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  GnssMultiPathFlag_t* pz_MPflag = NULL;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  BOOL q_isBDS3Sat = FALSE;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_s2r[3] = { 0.0 };
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double d_baseEarthRot = 0.0;
  double d_wave = 0.0;
  double d_omc = 0.0;
  double d_var = 0.0;
  double d_pdop = 0.0;
  double pd_HtH[16] = { 0.0 };
  double pd_HtHinv[16] = { 0.0 };
  SeqKalmanVar_t z_seqVar = { 0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_baseSatPosClk = NULL;
  double* pd_deltaX = pz_RTKfilterInfo->pd_deltaX;
  uint8_t pu_BDS2clkStatus[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint8_t pu_BDS3clkStatus[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  matrix_t  z_HtH;
  matrix_t  z_HtHinv;

  RTK_VerifyBDSclkValid(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pu_BDS2clkStatus, pu_BDS3clkStatus);
  pz_MPflag = pz_filterObs->pz_MPflag;

  //get the  site state index in the filter
  if (!RTK_getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, pw_rcvClkIndex))
  {
    return 0;
  }
  if (FALSE == (pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  /* site coordinate */
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
  }
  /* init sequential filter */
  u_seqStatus = hpp_seq_init(&z_seqVar, pz_RTKfilterInfo->w_nmax);
  if (0 != u_seqStatus)
  {
    return 0;
  }
  /* begin filter*/
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    q_isBDS3Sat = gnss_isBDS3Sat(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid))
      {
        continue;
      }
      if (fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS)
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
        || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      if (TRUE == q_isBDS3Sat)
      {
        if (0 == pu_BDS3clkStatus[z_freqType])
        {
          continue;
        }
      }
      else if (C_GNSS_BDS2 == z_constell)
      {
        if (0 == pu_BDS2clkStatus[z_freqType])
        {
          continue;
        }
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
      if (INVALID_INDEX == w_rcvDcbFilterIndex) // no clk
      {
        continue;
      }
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
        continue;
      }
      d_wave = wavelength(pz_sigMeas->u_signal);
      if (d_wave >= 1.0)
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (w_ambIndex < 0)
      {
        continue;
      }
      d_var = RTK_obsVariance(pz_sigMeas->u_constellation, C_GNSS_OBS_TYPE_CR, pz_satMeas->z_satPosVelClk.f_elevation, pz_RTKfilterInfo->f_age);
      if (d_var <= 0.0)
      {
        continue;
      }
      if ((pz_RTKfilterInfo->t_closeSkyCount & 0x1) == 0x1)
      {
        if ((pz_satMeas->z_satPosVelClk.f_elevation) >= 30.0 * DEG2RAD)
        {
          d_var *= 9.0;
        }
        else
        {
          d_var *= 1000.0;
        }
      }
      if (NULL != pz_MPflag && pz_satSigMeasCollect->u_satMeasCount >= 30)
      {
        u_MPflag = pz_MPflag->pu_MPflag[q_satIndex][z_freqType] & MULTIPATH_FLAG_PR;
        
        if (u_MPflag & MULTIPATH_FLAG_PR_HIGH)
        {
          d_var *= SQR(5.0);
        }
        else if (u_MPflag & MULTIPATH_FLAG_PR_LOW)
        {
          d_var *= SQR(1.5);
        }
        else if (u_MPflag == MULTIPATH_FLAG_PR_NORMAL)
        {
          d_var *= SQR(0.8);
        }
      }
      hpp_seq_ClearH(&z_seqVar);
      /* H */
      for (u_i = 0; u_i < 3; ++u_i)
      {
        hpp_seq_AddH(pw_PVAindex[u_i], -pd_unitVector[u_i], &z_seqVar); // pos
      }
      hpp_seq_AddH(w_rcvClkFilterIndex, 1.0, &z_seqVar); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        hpp_seq_AddH(w_rcvDcbFilterIndex, 1.0, &z_seqVar); // dcb
      }
      hpp_seq_AddH(w_ambIndex, d_wave, &z_seqVar); // amb
      /* OMC */
      pd_baseSatPosClk = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk;
      for (u_i = 0; u_i < 3; u_i++)
      {
        pd_s2r[u_i] = pd_baseSatPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      d_baseEarthRot = (pd_baseSatPosClk[0] * pz_rtkCorrBlock->d_refPosEcef[1] -
        pd_baseSatPosClk[1] * pz_rtkCorrBlock->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;
      d_omc = ((pz_sigMeas->d_carrierPhase) - (pz_corrMeasReslut->d_carrierPhase)) * d_wave - (d_roverDist - (d_baseDist + d_baseEarthRot));
      d_omc += (pd_satPosClk[3] - pd_baseSatPosClk[3]);
      d_omc -= d_wave * pd_X[w_ambIndex]; // amb
      d_omc -= (pd_X[w_rcvClkFilterIndex]); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        d_omc -= (pd_X[w_rcvDcbFilterIndex]); // dcb
      }
      /*if (fabs(pz_RTKfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][z_freqType])> GNSS_PHASE_CODE_CLK_MAX_BIAS)
      {
        d_omc -= pz_RTKfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][z_freqType];
      }*/
      hpp_seq_SetOMC(d_omc, d_var, &z_seqVar);
      hpp_seq_PredictStep(pd_deltaX, pz_RTKfilterInfo->pd_Q, &z_seqVar);
      hpp_seq_measUpdate(pz_RTKfilterInfo->pq_paraValid, pz_RTKfilterInfo->pd_X, pd_deltaX, pz_RTKfilterInfo->pd_Q, &z_seqVar, 0.0);
      ++u_measUpdateCarrierNum;
      ++(pz_RTKfilterInfo->pu_measUpdateCarrierNum[z_freqType]);

      /*PDOP*/
      if (gnss_Dot(pd_unitVector, pd_unitVector, 3) > FABS_ZEROS)
      {
        pd_HtH[0] += pd_unitVector[0] * pd_unitVector[0];
        pd_HtH[1 * 4 + 1] += pd_unitVector[1] * pd_unitVector[1];
        pd_HtH[2 * 4 + 2] += pd_unitVector[2] * pd_unitVector[2];
        pd_HtH[3 * 4 + 3] += 1 * 1;
        pd_HtH[1] = pd_HtH[4] = pd_HtH[4] + pd_unitVector[0] * pd_unitVector[1];
        pd_HtH[2] = pd_HtH[8] = pd_HtH[8] + pd_unitVector[0] * pd_unitVector[2];
        pd_HtH[6] = pd_HtH[9] = pd_HtH[9] + pd_unitVector[1] * pd_unitVector[2];
        pd_HtH[3] = pd_HtH[12] = pd_HtH[12] + pd_unitVector[0] * 1;
        pd_HtH[7] = pd_HtH[13] = pd_HtH[13] + pd_unitVector[1] * 1;
        pd_HtH[11] = pd_HtH[14] = pd_HtH[14] + pd_unitVector[2] * 1;
      }

      //update sat filter info
      if (NULL != pz_SatFilterObs && NULL != pz_SatFilterObs->z_measUpdateFlag[u_signalIndex])
      {
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_valid = 1;
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_cpValid = 1;
        //pz_SatFilterObs->pf_phaseResidual[u_signalIndex] = (float)d_omc;
        pz_SatFilterObs->pf_phaseVar[u_signalIndex] = (float)d_var;
      }
    }
  }

  /* calcu & save pdop */
  z_HtH.col = 4;
  z_HtH.row = 4;
  z_HtH.data = pd_HtH;
  z_HtHinv.col = 4;
  z_HtHinv.row = 4;
  z_HtHinv.data = pd_HtHinv;
  if (matrix_inverse(&z_HtH, &z_HtHinv))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      d_pdop += pd_HtHinv[u_i + u_i * 4];
    }
    if (d_pdop > 0.0)
    {
      pz_filterObs->f_pdop = (float)sqrt(d_pdop);
    }
  }

  /* free */
  hpp_seq_deinit(&z_seqVar);
  return u_measUpdateCarrierNum;
}
/**
 * @brief using the zero-combine code observations to update the EKF
 * @param[in]  pz_pseudoUpdateMask is the mask of pseudo-range to do measure update
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of updated pseudo-range observations
 */
uint8_t RTK_zeroCombineCodeMeasUpdate(const gnss_FreqType pz_pseudoUpdateMask[C_GNSS_FREQ_TYPE_MAX], const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, 
  const GnssCorrBlock_t* pz_rtkCorrBlock,rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  uint8_t u_measUpdatePseudoNum = 0;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  uint8_t u_i = 0;
  uint8_t u_seqStatus = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_MPflag = 0;
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  uint16_t w_iSat = 0;
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  GnssMultiPathFlag_t* pz_MPflag = NULL;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  BOOL q_isBDS3Sat = FALSE;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_s2r[3] = { 0.0 };
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double d_baseEarthRot = 0.0;
  double d_omc = 0.0;
  double d_var = 0.0;
  double d_pdop = 0.0;
  double pd_HtH[16] = { 0.0 };
  double pd_HtHinv[16] = { 0.0 };
  rtk_SatFilterObs_t* pz_SatFilterObs;
  SeqKalmanVar_t z_seqVar = { 0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_baseSatPosClk = NULL;
  double* pd_deltaX = pz_RTKfilterInfo->pd_deltaX;
  uint8_t pu_BDS2clkStatus[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint8_t pu_BDS3clkStatus[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  matrix_t  z_HtH;
  matrix_t  z_HtHinv;

  RTK_VerifyBDSclkValid(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pu_BDS2clkStatus, pu_BDS3clkStatus);
  //get the  site state index in the filter
  if (!RTK_getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, pw_rcvClkIndex))
  {
    return 0;
  }
  if (FALSE == (pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  /* site coordinate */
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
  }
  /* init sequential filter */
  u_seqStatus = hpp_seq_init(&z_seqVar, pz_RTKfilterInfo->w_nmax);
  if (0 != u_seqStatus)
  {
    return 0;
  }
  pz_MPflag = pz_filterObs->pz_MPflag;
  /* begin filter*/
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    q_isBDS3Sat = gnss_isBDS3Sat(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }
      if ((pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (0 == pz_pseudoUpdateMask[z_freqType])
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid) || !(pz_corrMeasReslut->z_measStatusFlag.b_prValid) || (pz_corrMeasReslut->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      if (TRUE == q_isBDS3Sat)
      {
        if (0 == pu_BDS3clkStatus[z_freqType])
        {
          continue;
        }
      }
      else if (C_GNSS_BDS2 == z_constell)
      {
        if (0 == pu_BDS2clkStatus[z_freqType])
        {
          continue;
        }
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
      if (INVALID_INDEX == w_rcvDcbFilterIndex) // no clk
      {
        continue;
      }
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
        continue;
      }
      //w_id[0] = 0;
      //w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      //w_id[2] = convertFreqAmb2FilterType(z_freqType);
      if (0 == (pz_RTKfilterInfo->u_goodSceneStatusFlag))
      {
        d_var = RTK_getPrVarianceUsedByPVT(pz_RTKfilterInfo->pz_pvtResult, pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal);
      }
      else
      {
        d_var = RTK_obsVariance(pz_sigMeas->u_constellation, C_GNSS_OBS_TYPE_PR, pz_satMeas->z_satPosVelClk.f_elevation, pz_RTKfilterInfo->f_age);
      }
      if (d_var <= 0.0)
      {
        continue;
      }
      if (NULL != pz_MPflag && pz_satSigMeasCollect->u_satMeasCount >= 30)
      {
        u_MPflag = pz_MPflag->pu_MPflag[q_satIndex][z_freqType] & MULTIPATH_FLAG_PR;
        /*if (u_MPflag == MULTIPATH_FLAG_PR_NORMAL)
        {
          d_var *= SQR(0.8);
        }*/
        if (u_MPflag == MULTIPATH_FLAG_PR_LOW)
        {
          d_var *= SQR(1.5);
        }
        else if (u_MPflag == MULTIPATH_FLAG_PR_HIGH)
        {
          d_var *= SQR(5.0);
        }
      }

      hpp_seq_ClearH(&z_seqVar);
      /* H */
      for (u_i = 0; u_i < 3; ++u_i)
      {
        hpp_seq_AddH(pw_PVAindex[u_i], -pd_unitVector[u_i], &z_seqVar); // pos
      }
      hpp_seq_AddH(w_rcvClkFilterIndex, 1.0, &z_seqVar); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        hpp_seq_AddH(w_rcvDcbFilterIndex, 1.0, &z_seqVar); // dcb
      }
      /* OMC */
      pd_baseSatPosClk = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk;
      for (u_i = 0; u_i < 3; u_i++)
      {
        pd_s2r[u_i] = pd_baseSatPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      d_baseEarthRot = (pd_baseSatPosClk[0] * pz_rtkCorrBlock->d_refPosEcef[1] -
        pd_baseSatPosClk[1] * pz_rtkCorrBlock->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;
      d_omc = ((pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange)) - (d_roverDist - (d_baseDist + d_baseEarthRot));
      d_omc += (pd_satPosClk[3] - pd_baseSatPosClk[3]);
      d_omc -= (pd_X[w_rcvClkFilterIndex]); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        d_omc -= (pd_X[w_rcvDcbFilterIndex]); // dcb
      }
      hpp_seq_SetOMC(d_omc, d_var, &z_seqVar);
      hpp_seq_PredictStep(pd_deltaX, pz_RTKfilterInfo->pd_Q, &z_seqVar);
      hpp_seq_measUpdate(pz_RTKfilterInfo->pq_paraValid, pz_RTKfilterInfo->pd_X, pd_deltaX, pz_RTKfilterInfo->pd_Q, &z_seqVar, 0.0);
      ++u_measUpdatePseudoNum;
      ++(pz_RTKfilterInfo->pu_measUpdatePseudoNum[z_freqType]);

      /*PDOP*/
      if (gnss_Dot(pd_unitVector, pd_unitVector, 3) > FABS_ZEROS)
      {
        pd_HtH[0] += pd_unitVector[0] * pd_unitVector[0];
        pd_HtH[1 * 4 + 1] += pd_unitVector[1] * pd_unitVector[1];
        pd_HtH[2 * 4 + 2] += pd_unitVector[2] * pd_unitVector[2];
        pd_HtH[3 * 4 + 3] += 1 * 1;
        pd_HtH[1] = pd_HtH[4] = pd_HtH[4] + pd_unitVector[0] * pd_unitVector[1];
        pd_HtH[2] = pd_HtH[8] = pd_HtH[8] + pd_unitVector[0] * pd_unitVector[2];
        pd_HtH[6] = pd_HtH[9] = pd_HtH[9] + pd_unitVector[1] * pd_unitVector[2];
        pd_HtH[3] = pd_HtH[12] = pd_HtH[12] + pd_unitVector[0] * 1;
        pd_HtH[7] = pd_HtH[13] = pd_HtH[13] + pd_unitVector[1] * 1;
        pd_HtH[11] = pd_HtH[14] = pd_HtH[14] + pd_unitVector[2] * 1;
      }

      //update sat filter info
      if (NULL != pz_SatFilterObs && NULL != pz_SatFilterObs->z_measUpdateFlag[u_signalIndex])
      {
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_valid = 1;
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_prValid = 1;
        //pz_SatFilterObs->pf_codeResidual[u_signalIndex] = (float)d_omc;
        pz_SatFilterObs->pf_codeVar[u_signalIndex] = (float)d_var;
      }
    }
  }

  /* calcu & save pdop */
  z_HtH.col = 4;
  z_HtH.row = 4;
  z_HtH.data = pd_HtH;
  z_HtHinv.col = 4;
  z_HtHinv.row = 4;
  z_HtHinv.data = pd_HtHinv;
  if (matrix_inverse(&z_HtH, &z_HtHinv))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      d_pdop += pd_HtHinv[u_i + u_i * 4];
    }
    if (d_pdop > 0.0)
    {
      pz_filterObs->f_pdop = (float)sqrt(d_pdop);
    }
  }

  /* free */
  hpp_seq_deinit(&z_seqVar);
  return u_measUpdatePseudoNum;
}
/**
 * @brief using the zero-combine doppler observations to update the EKF
 * @param[in]  z_optFreq is the optimal frequency of doppler used by measure update
 * @param[in]  pd_X is value for the state
 * @param[in]  pf_eleSection is section of elevation,[ele0,ele1),ele0 is pf_eleSection[0] and ele1 is pf_eleSection[1]
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of updated doppler observations
 */
uint8_t RTK_zeroCombineDopplerMeasUpdate(gnss_FreqType z_optFreq, const double* pd_X, const float pf_eleSection[2], const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,rtk_filterInfo_t* pz_RTKfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  uint8_t u_measUpdatDopplerNum = 0;
  uint8_t u_i = 0;
  uint8_t u_seqStatus = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_rcvClkDriftIndex = -1;
  BOOL q_paraValid = TRUE;
  BOOL q_satUpdated = FALSE;
  double* pd_deltaX = pz_RTKfilterInfo->pd_deltaX;
  double pd_siteCoor[3] = { 0.0 };
  double pd_siteVel[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double d_roverDist = 0.0;
  SeqKalmanVar_t z_seqVar = { 0 };
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_satVelClkDrift = NULL;
  double d_dopplerVar = 0.0;
  double d_omc = 0.0;
  double d_calculatedValue = 0.0;
  double d_ustd = 0.0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;

  //velocity parameter
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    if (pw_PVAindex[u_i] < 0)
    {
      q_paraValid = FALSE;
      break;
    }
  }
  //receiver clock drift parameter
  w_id[0] = 0;
  w_id[1] = 0;
  w_id[2] = GNSS_FILTER_STATE_DRIFT;
  pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
  if (NULL != pz_ekfState)
  {
    w_rcvClkDriftIndex = pz_ekfState->w_index;
  }
  if (w_rcvClkDriftIndex < 0 || FALSE == q_paraValid)
  {
    return 0;
  }
  //using doppler observations to do measure update
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
    pd_siteVel[u_i] = pd_X[pw_PVAindex[3 + u_i]];
  }
  /* init sequential filter */
  u_seqStatus = hpp_seq_init(&z_seqVar, pz_RTKfilterInfo->w_nmax);
  if (0 != u_seqStatus)
  {
    return 0;
  }
  /* begin filter*/
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < pf_eleSection[0] || (pz_satMeas->z_satPosVelClk.f_elevation) >= pf_eleSection[1])
    {
      continue;
    }
    pd_satVelClkDrift = pz_satMeas->z_satPosVelClk.d_satVelClk;
    pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    /*d_roverDist =*/ gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    q_satUpdated = FALSE;
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (TRUE == q_satUpdated || NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_drValid))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != z_optFreq)
      {
        continue;
      }
      d_dopplerVar = RTK_getDopplerVarianceUsedByPVT(pz_RTKfilterInfo->pz_pvtResult, pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal);
      if (d_dopplerVar <= 0.0)
      {
        continue;
      }
      hpp_seq_ClearH(&z_seqVar);
      /* H */
      for (u_i = 0; u_i < 3; ++u_i)
      {
        hpp_seq_AddH(pw_PVAindex[3 + u_i], -pd_unitVector[u_i], &z_seqVar); // vel
      }
      hpp_seq_AddH(w_rcvClkDriftIndex, 1.0, &z_seqVar); // clk
      d_calculatedValue = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        d_calculatedValue += (-pd_unitVector[u_i]) * (pd_X[pw_PVAindex[3 + u_i]] - pd_satVelClkDrift[u_i]);
      }
      d_calculatedValue += (pd_X[w_rcvClkDriftIndex] - pd_satVelClkDrift[3]);
      d_omc = (pz_sigMeas->d_doppler) - d_calculatedValue;
      hpp_seq_SetOMC(d_omc, d_dopplerVar, &z_seqVar);
      hpp_seq_PredictStep(pd_deltaX, pz_RTKfilterInfo->pd_Q, &z_seqVar);
      d_ustd += d_omc * d_omc / d_dopplerVar;
      hpp_seq_measUpdate(pz_RTKfilterInfo->pq_paraValid, pz_RTKfilterInfo->pd_X, pd_deltaX, pz_RTKfilterInfo->pd_Q, &z_seqVar, 0.0);
      ++u_measUpdatDopplerNum;
      q_satUpdated = TRUE;
    }
  }

  /* Save std unit weight of doppler */
  if(rtk_integ_valid(pz_RTKfilterInfo->pz_integ))
  {
    d_ustd = sqrt(d_ustd / u_measUpdatDopplerNum);
    rtk_integ_save_residualDr(d_ustd, u_measUpdatDopplerNum, pz_RTKfilterInfo->pz_integ);
  }

  /* free */
  hpp_seq_deinit(&z_seqVar);
  return u_measUpdatDopplerNum;
}
/**
 * @brief select the optimal frequency of pseudo-range used by measure update
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[in]  pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_pseudoUpdateMask is the mask of pseudo-range to do measure update
 * @return     void
 */
void RTK_selectPseudoOptFreq(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  const rtk_filterInfo_t* pz_RTKfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_FreqType pz_pseudoUpdateMask[C_GNSS_FREQ_TYPE_MAX])
{
  gnss_FreqType z_optFreq = C_GNSS_FREQ_TYPE_L1;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint16_t w_iSat = 0;
  BOOL q_isBDS3Sat = FALSE;
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  gnss_ConstellationType z_constell = C_GNSS_NONE;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  uint8_t pu_BDS2clkStatus[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint8_t pu_BDS3clkStatus[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  uint8_t pu_satNumPerFreq[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint8_t u_maxSatNum = 0;
  double d_var = 0.0;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_satNumPerFreq[u_i] = 0;
    pz_pseudoUpdateMask[u_i] = 1;
  }
  RTK_VerifyBDSclkValid(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pu_BDS2clkStatus, pu_BDS3clkStatus);
  //get the  site state index in the filter
  if (!RTK_getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, pw_rcvClkIndex))
  {
    return;
  }
  if (FALSE == (pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    q_isBDS3Sat = gnss_isBDS3Sat(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }
      if ((pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid) || !(pz_corrMeasReslut->z_measStatusFlag.b_prValid) || (pz_corrMeasReslut->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        continue;
      }
      z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_RTKfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      if (TRUE == q_isBDS3Sat)
      {
        if (0 == pu_BDS3clkStatus[z_freqType])
        {
          continue;
        }
      }
      else if (C_GNSS_BDS2 == z_constell)
      {
        if (0 == pu_BDS2clkStatus[z_freqType])
        {
          continue;
        }
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
      if (INVALID_INDEX == w_rcvDcbFilterIndex) // no clk
      {
        continue;
      }
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
        continue;
      }
      if (0 == (pz_RTKfilterInfo->u_goodSceneStatusFlag))
      {
        d_var = RTK_getPrVarianceUsedByPVT(pz_RTKfilterInfo->pz_pvtResult, pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal);
      }
      else
      {
        d_var = RTK_obsVariance(pz_sigMeas->u_constellation, C_GNSS_OBS_TYPE_PR, pz_satMeas->z_satPosVelClk.f_elevation, pz_RTKfilterInfo->f_age);
      }
      if (d_var <= 0.0)
      {
        continue;
      }
      ++pu_satNumPerFreq[z_freqType];
    }
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (pu_satNumPerFreq[u_i] > u_maxSatNum)
    {
      u_maxSatNum = pu_satNumPerFreq[u_i];
      z_optFreq = (gnss_FreqType)u_i;
    }
  }
  if (u_maxSatNum >= 20)
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      if (u_i != z_optFreq)
      {
        pz_pseudoUpdateMask[u_i] = 0;//disable the other frequency that exclude the optimal frequeceny
      }
    }
  }
  return;
}
/**
 * @brief select the optimal frequency of doppler used by measure update
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @return     the optimal frequency of doppler used by measure update
 */
gnss_FreqType RTK_selectDopplerOptFreq(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const rtk_filterInfo_t* pz_RTKfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  gnss_FreqType u_optFreq = C_GNSS_FREQ_TYPE_L1;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  uint8_t u_i = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  double d_dopplerVar = 0.0;
  uint8_t u_maxSatNum = 0;
  uint8_t pu_satNumPerFreq[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (u_i = C_GNSS_FREQ_TYPE_L1; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_satNumPerFreq[u_i] = 0;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (float)(pz_RTKfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_drValid))
      {
        continue;
      }
      u_freqType= gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqType >= C_GNSS_FREQ_TYPE_MAX)
      {
        continue;
      }
      d_dopplerVar = RTK_getDopplerVarianceUsedByPVT(pz_RTKfilterInfo->pz_pvtResult, pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal);
      if (d_dopplerVar <= 0.0)
      {
        continue;
      }
      ++pu_satNumPerFreq[u_freqType];
    }
  }
  if (pu_satNumPerFreq[0] >= 20)//to avoid switched other frequecny observations too frequently that when L1 frequency had enough valid doppler observation to do measure update,there not been switched
  {
    u_optFreq = (gnss_FreqType)0;
  }
  else
  {
    for (u_i = C_GNSS_FREQ_TYPE_L1; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      if (pu_satNumPerFreq[u_i] > u_maxSatNum)
      {
        u_maxSatNum = pu_satNumPerFreq[u_i];
        u_optFreq = u_i;
      }
    }
  }
  return u_optFreq;
}
/**
 * @brief          RTK RTD solution availability check
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]      pz_filterObs information of observation after filter
 * @return         void
 */
void rtk_RTDsol_availabilityCheck(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  BOOL b_openSky = FALSE;
  uint8_t u_i = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_sumUpdatePhaseNum = 0;
  uint16_t w_sumUpdateCodeNum = 0;
  uint16_t w_rejCodeNum = 0;
  uint16_t w_acceptCodeNum = 0;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  uint32_t q_satIndex = 0;
  float f_pdop = 0.0;
  /* threshold define begin */
  const uint8_t u_acceptNum = 6; //min accept Code Num 
  const double d_rejectPecent = 0.6; //max reject pecent
  const double d_pdopThres = 30.0; //max pdop
  const double d_floatSTDCodeThres = 10.0; //max STD code threshold
  const double d_CodeThresOpen = 5.0; //max code res value in open sky (m)
  const double d_CodeThresClose = 10.0; //max code res value in close sky (m)
  const double d_rtdCount = 5.0; //max filter epoch continuous (s)
  /* threshold define end */
  double d_floatSTDCode;
  rtk_filter_sol_t* pz_filterSol = NULL;
  rtk_SatFilterObs_t* pz_SatFilterObs = NULL;

  if (pz_RTKfilterInfo == NULL || pz_filterObs == NULL || pz_EKFstateRepPool == NULL)
  {
    return;
  }
  if (!(RTK_getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, pw_rcvClkIndex)))
  {
    return;
  }

  /* save result */
  for (u_i = 0; u_i < 3; u_i++)
  {
    pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_RTD][u_i] = pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i]];
    pz_RTKfilterInfo->pz_filterSol->pd_filterVel[RTK_FILTER_POS_RTD][u_i] = pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i + 3]];
  }
  pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] = RTK_FILTER_SOL_STAT_SAVE;

  /* residual calculation */
  RTK_calculateResidual(pz_RTKfilterInfo->pd_X, pz_satSigMeasCollect,
                        pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool, NULL, pz_filterObs, RES_RTD);

  /* availability check */
  d_floatSTDCode = pz_filterObs->d_rtdCodeSTD;
  pz_filterSol = pz_RTKfilterInfo->pz_filterSol;
  f_pdop = pz_filterObs->f_pdop;
  if (pz_filterSol == NULL)
  {
    return;
  }
  if (d_floatSTDCode < FABS_ZEROS || f_pdop < FABS_ZEROS ||
    gnss_Dot(pz_filterSol->pd_filterPos[RTK_FILTER_POS_RTD],
      pz_filterSol->pd_filterPos[RTK_FILTER_POS_RTD], 3) < FABS_ZEROS * FABS_ZEROS)
  {
    return;
  }

  // statistic updated PR num
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if ((pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]) > 0)
    {
      w_sumUpdateCodeNum += pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i];
    }
  }

  // statistic updated outline num
  b_openSky = ((pz_RTKfilterInfo->t_closeSkyCount & 0x3) == 0);
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (b_openSky)
      {
        if (pz_SatFilterObs->pf_codeResidual[u_signalIndex] > d_CodeThresOpen)
        {
          w_rejCodeNum++;
        }
      }
      else
      {
        if (pz_SatFilterObs->pf_codeResidual[u_signalIndex] > d_CodeThresClose)
        {
          w_rejCodeNum++;
        }
      }
    }
  }
  w_acceptCodeNum = w_sumUpdateCodeNum - w_rejCodeNum;

  if ((w_acceptCodeNum >= u_acceptNum) && (w_rejCodeNum <= w_sumUpdateCodeNum * 0.60)
    && (d_floatSTDCode < d_floatSTDCodeThres) && (f_pdop < d_pdopThres))
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] = RTK_FILTER_SOL_STAT_VALID;
  }
  
  // continuous rtd,  might reset
  if ((double)pz_RTKfilterInfo->w_rtdCount * pz_RTKfilterInfo->f_sample >= d_rtdCount && d_floatSTDCode > d_CodeThresOpen)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] = RTK_FILTER_SOL_STAT_RESET;
    LOGI(TAG_HPP, "** RTD Sol Check ** continuous rtd, might need to reset | continuous second:%f, d_floatSTDCode:%f |\n",
      d_rtdCount, d_floatSTDCode);
  }

  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] == RTK_FILTER_SOL_STAT_VALID)
  {
    LOGI(TAG_HPP, "** RTD Sol Check ** <VALID>  | opensky=%d | reject=%d accept=%d STD=%.2f pdop=%.2f\n",
      b_openSky, w_rejCodeNum, w_acceptCodeNum, d_floatSTDCode, f_pdop);
  }
  else
  {
    LOGI(TAG_HPP, "** RTD Sol Check ** < NOT >  | opensky=%d | reject=%d accept=%d STD=%.2f pdop=%.2f\n",
      b_openSky, w_rejCodeNum, w_acceptCodeNum, d_floatSTDCode, f_pdop);
  }

  return;
}
/**
 * @brief using the zero-combine phase observations to update the EKF
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pz_filterObs information of observation after filter
 * @return     TRUE represent successful, otherwise, failure
 */
BOOL RTK_obsMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  BOOL q_status = TRUE;
  uint8_t u_measUpdateDopplerNum = 0;
  uint8_t u_measUpdatePseudoNum = 0;
  uint8_t u_measUpdateCarrierNum = 0;
  uint8_t u_maxUpdateCarrierNum = 0;
  uint8_t u_maxUpdatePseudoNum = 0;
  uint8_t u_i = 0;
  gnss_FreqType pz_pseudoUpdateMask[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  gnss_FreqType u_dopplerOptFreq = C_GNSS_FREQ_TYPE_L1;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  float pf_eleSection[2] = { (float)(60.0 * DEG2RAD),(float)(91.0 * DEG2RAD) };

  if (1 != (pz_RTKfilterInfo->u_goodSceneStatusFlag))
  {
    u_dopplerOptFreq = RTK_selectDopplerOptFreq(pz_satSigMeasCollect, pz_RTKfilterInfo, pz_EKFstateRepPool);
    u_measUpdateDopplerNum += RTK_zeroCombineDopplerMeasUpdate(u_dopplerOptFreq, pd_X, pf_eleSection, pz_satSigMeasCollect, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);
    if (u_measUpdateDopplerNum < 18)
    {
      pf_eleSection[1] = pf_eleSection[0];
      pf_eleSection[0] = (float)(45.0 * DEG2RAD);
      u_measUpdateDopplerNum += RTK_zeroCombineDopplerMeasUpdate(u_dopplerOptFreq, pd_X, pf_eleSection, pz_satSigMeasCollect, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);
      if (u_measUpdateDopplerNum < 18)
      {
        pf_eleSection[1] = pf_eleSection[0];
        pf_eleSection[0] = (float)(pz_RTKfilterInfo->z_opt.d_elmin);
        u_measUpdateDopplerNum += RTK_zeroCombineDopplerMeasUpdate(u_dopplerOptFreq, pd_X, pf_eleSection, pz_satSigMeasCollect, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);
      }
    }
    LOGI(TAG_HPP, "Doppler Meas Update in filter: %d\n", u_measUpdateDopplerNum);
  }
  RTK_selectPseudoOptFreq(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_pseudoUpdateMask);
  u_measUpdatePseudoNum = RTK_zeroCombineCodeMeasUpdate(pz_pseudoUpdateMask, pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock,
    pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);

  LOGI(TAG_HPP, "=============== RTD UPDATE PR=%d ===============\n", u_measUpdatePseudoNum);

  //save RTD rlt
  if (u_measUpdatePseudoNum > 4)
  {
    rtk_RTDsol_availabilityCheck(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);
  }

  u_measUpdateCarrierNum = RTK_zeroCombinePhaseMeasUpdate(pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock,
    pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);

  //save float rlt
  if (u_measUpdateCarrierNum > 0 && RTK_getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, pw_rcvClkIndex))
  {
    for (u_i = 0; u_i < 3; u_i++)
    {
      pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_FLOAT][u_i] = pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i]];
      pz_RTKfilterInfo->pz_filterSol->pd_filterVel[RTK_FILTER_POS_FLOAT][u_i] = pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i + 3]];
    }
    pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_SAVE;
  }

  LOGI(TAG_HPP, "=============== FLOAT UPDATE PR=%d CP=%d ===============\n", u_measUpdatePseudoNum, u_measUpdateCarrierNum);

  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if ((pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]) > u_maxUpdateCarrierNum)
    {
      u_maxUpdateCarrierNum = (pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]);
    }
    if ((pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]) > u_maxUpdatePseudoNum)
    {
      u_maxUpdatePseudoNum = (pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]);
    }
  }
  if (u_maxUpdateCarrierNum < 5 && u_maxUpdatePseudoNum < 5)
  {
    q_status = FALSE;
  }
  return q_status;
}

void RTK_PriorQualityControlPseodoRange(BOOL q_isSperateBDS2And3, gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq,
  const double pd_siteCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  const rtk_filterInfo_t* pz_RTKfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint32_t q_satNum = 0;
  uint32_t q_satIndex = 0;
  uint32_t q_i = 0;
  uint32_t q_num = 0;
  uint16_t w_iSat = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_pseduoSatDiff = 0.0;
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double pd_s2r[3] = { 0.0 };
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  double* pd_rcvClkSet = NULL;
  uint32_t* pq_satIndexSet = NULL;
  uint8_t* pu_signalIndexSet = NULL;
  double d_initRcvClk = 0.0;
  q_satNum = RTK_getSatNumUsedByInitRcvClkCal(q_isSperateBDS2And3, z_targetSys, z_targetFreq, pz_satSigMeasCollect, pz_rtkCorrBlock);
  if (q_satNum < 1)
  {
    return;
  }
  pd_rcvClkSet = (double*)OS_MALLOC(q_satNum * sizeof(double));
  memset(pd_rcvClkSet, 0, q_satNum * sizeof(double));
  pq_satIndexSet = (uint32_t*)OS_MALLOC(q_satNum * sizeof(uint32_t));
  memset(pq_satIndexSet, 0, q_satNum * sizeof(uint32_t));
  pu_signalIndexSet = (uint8_t*)OS_MALLOC(q_satNum * sizeof(uint8_t));
  memset(pu_signalIndexSet, 0, q_satNum * sizeof(uint8_t));
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (q_num >= q_satNum)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_prValid)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
      if (u_constellation != z_targetSys || (pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != z_targetFreq)
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount))
      {
        continue;
      }
      d_pseduoSatDiff = (pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_satMeas->z_satPosVelClk.d_satPosClk[u_i] - pd_siteCoor[u_i];
      }
      d_roverDist = gnss_Norm(pd_s2r, 3);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      pd_rcvClkSet[q_num] = d_pseduoSatDiff - (d_roverDist - d_baseDist);
      pq_satIndexSet[q_num] = q_satIndex;
      pu_signalIndexSet[q_num] = u_signalIndex;
      ++q_num;
    }
  }
  u_status = gnss_ascSortMedianDouble(pd_rcvClkSet, q_num, &d_initRcvClk);
  if (0 == u_status)
  {
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
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
        u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
        if (u_constellation != z_targetSys)
        {
          continue;
        }
        z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
        if (z_freqType != z_targetFreq)
        {
          continue;
        }
        pz_sigMeas->z_measStatusFlag.b_prValid = 0;
      }
    }
  }
  else
  {
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_satMeas)
      {
        continue;
      }
      for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
      {
        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_prValid)
        {
          continue;
        }
        u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
        if (u_constellation != z_targetSys || (pz_sigMeas->d_pseudoRange) < 1.0e-2)
        {
          continue;
        }
        z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
        if (z_freqType != z_targetFreq)
        {
          continue;
        }
        pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
        if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
        {
          continue;
        }
        w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
        if (w_iSat >= (pz_rtkCorrBlock->w_satCount))
        {
          continue;
        }
        d_pseduoSatDiff = (pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange);
        for (u_i = 0; u_i < 3; ++u_i)
        {
          pd_s2r[u_i] = pz_satMeas->z_satPosVelClk.d_satPosClk[u_i] - pd_siteCoor[u_i];
        }
        d_roverDist = gnss_Norm(pd_s2r, 3);
        for (u_i = 0; u_i < 3; ++u_i)
        {
          pd_s2r[u_i] = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
        }
        d_baseDist = gnss_Norm(pd_s2r, 3);
        if (fabs(d_pseduoSatDiff - (d_roverDist - d_baseDist) - d_initRcvClk) > 10.0)
        {
          pz_sigMeas->z_measStatusFlag.b_prValid = 0;
        }
      }
    }
  }

  if (NULL != pd_rcvClkSet)
  {
    OS_FREE(pd_rcvClkSet);
  }
  if (NULL != pq_satIndexSet)
  {
    OS_FREE(pq_satIndexSet);
  }
  if (NULL != pu_signalIndexSet)
  {
    OS_FREE(pu_signalIndexSet);
  }
  return;
}
void RTK_PseodoRangeQualityControl(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  const rtk_filterInfo_t* pz_RTKfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  double pd_pvtCoor[3] = { 0.0 };
  double pd_PredictCoor[3] = { 0.0 };
  double d_deltaTime = 0.0;
  float pf_pvtVel[3] = { 0.0 };
  float pf_predictVel[3] = { 0.0 };
  float pf_predictAcc[3] = { 0.0 };
  float f_velDiff = 0.0;
  double* pd_siteCoor = pd_pvtCoor;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  BOOL z_isPVAvalid = TRUE;
  BOOL q_isSperateBDS2And3 = FALSE;
  const gnss_PositionFix_t* pz_positionFix = &(pz_RTKfilterInfo->pz_pvtResult->z_positionFix);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_pvtCoor[u_i] = (pz_positionFix->d_xyz[u_i]);
    pf_pvtVel[u_i] = (pz_positionFix->f_velXyz[u_i]);
  }
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    if (pw_PVAindex[u_i] < 0)
    {
      z_isPVAvalid = FALSE;
      break;
    }
  }
  if (TRUE == z_isPVAvalid)
  {
    d_deltaTime = tm_GpsTimeDiff(&(pz_satSigMeasCollect->z_tor), &(pz_EKFstateRepPool->z_gpsTime));
    if (fabs(d_deltaTime) > 1.5)
    {
      z_isPVAvalid = FALSE;
    }
  }
  if (TRUE == z_isPVAvalid)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_PredictCoor[u_i] = pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i]];
      pf_predictVel[u_i] = (float)(pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i + 3]]);
      pf_predictAcc[u_i] = (float)(pz_RTKfilterInfo->pd_X[pw_PVAindex[u_i + 6]]);
    }
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_PredictCoor[u_i] += pf_predictVel[u_i] * d_deltaTime;
      pf_predictVel[u_i] += (float)(pf_predictAcc[u_i] * d_deltaTime);
      f_velDiff += (pf_predictVel[u_i] - pf_pvtVel[u_i]) * (pf_predictVel[u_i] - pf_pvtVel[u_i]);
    }
    f_velDiff = sqrtf(f_velDiff);
    if (f_velDiff < 1.0)
    {
      pd_siteCoor = pd_PredictCoor;
    }
  }
  for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
  {
    q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(u_j, pz_satSigMeasCollect);
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(q_isSperateBDS2And3, u_i))
      {
        continue;
      }
      RTK_PriorQualityControlPseodoRange(q_isSperateBDS2And3, u_i, u_j, pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool);
    }
  }
  return;
}
/**
 * @brief          Using the result of QR check to mark the pseudo-range observations
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @return         void
 */
void RTK_markPseudoRangeUsingQRresult(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
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
      if (PR_QUALITY_GROSS_BY_QR_PARITY == (pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality))
      {
        pz_sigMeas->z_measStatusFlag.b_prValid = 0;
      }
    }
  }
  return;
}
/**
 * @brief          RTK float solution availability check
 * @param[in/out]  pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]      pz_filterObs information of observation after filter
 * @return         void
 */
void rtk_floatSol_availabilityCheck(rtk_filterInfo_t* pz_RTKfilterInfo, rtk_EpochFilterObs_t* pz_filterObs)
{
  BOOL b_openSky = FALSE;
  uint8_t u_i = 0;
  uint8_t u_diffFlagSPP = 0;
  uint8_t u_diffFlagPredict = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_maxUpdatePhaseNum = 0;
  uint8_t u_sumDiffSppEpoch = 0;
  uint16_t w_floatEpoch = 0;
  uint16_t w_filterWithPhaseCount = 0;
  uint16_t w_sumUpdatePhaseNum = 0;
  uint16_t w_sumUpdateCodeNum = 0;
  uint16_t w_rejCodeNum = 0;
  uint16_t w_rejPhaseNum = 0;
  uint16_t w_acceptCodeNum = 0;
  uint16_t w_acceptPhaseNum = 0;
  uint32_t q_satIndex = 0;
  float f_pdop = 0.0;
  /* threshold define begin */
  const double d_pdopThres = 50.0; //max pdop
  const double d_pdopMultiStdThres = 30.0; //max possible error, get form pdop mulitiply post res STD (m)
  const double d_CodeThresOpen = 5.0; //max code res value in open sky (m)
  const double d_PhaseThresOpen = 1.0; //max phase res value in open sky (m)
  const double d_CodeThresClose = 10.0; //max code res value in close sky (m)
  const double d_PhaseThresClose = 2.0; //max phase res value in close sky (m)
  const double d_PhaseThresNotFix = 0.02; //max phase res value in long time not fix (m)
  const double d_resetPencentThres = 0.2; //max Proportion of reset AMB
  const uint16_t w_floatEpochThres = 30; //max filter epoch not fix (epoch)
  const double d_elevationVelThres = 2.5; //max velocity in elevation (m/s)
  const double d_horizontalVelThres = 2.0; //max diff-velocity in Horizontal (m/s)
  /* threshold define end */
  double d_floatSTDCode;
  double d_floatSTDPhase;
  double d_resetPencent;
  rtk_filter_sol_t* pz_filterSol = NULL;
  rtk_SatFilterObs_t* pz_SatFilterObs = NULL;

  if (pz_RTKfilterInfo == NULL|| pz_filterObs==NULL)
  {
    return;
  }
  d_floatSTDCode = pz_filterObs->d_postCodeSTD;
  d_floatSTDPhase = pz_filterObs->d_postPhaseSTD;
  pz_filterSol = pz_RTKfilterInfo->pz_filterSol;
  f_pdop = pz_filterObs->f_pdop;
  w_floatEpoch = pz_RTKfilterInfo->w_floatCount;
  w_filterWithPhaseCount = pz_RTKfilterInfo->w_filterWithPhaseCount;
  d_resetPencent = pz_RTKfilterInfo->d_resetPercent;
  if (pz_filterSol == NULL || RTK_FILTER_SOL_STAT_NONE == (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT]))
  {
    return;
  }
  if (d_floatSTDCode < FABS_ZEROS || d_floatSTDPhase < FABS_ZEROS || f_pdop < FABS_ZEROS ||
    gnss_Dot(pz_filterSol->pd_filterPos[RTK_FILTER_POS_FLOAT],
    pz_filterSol->pd_filterPos[RTK_FILTER_POS_FLOAT], 3) < FABS_ZEROS * FABS_ZEROS)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_ERR;
    LOGI(TAG_HPP, "** Float Sol Check ** <ERROR>  | float STD Code:%f Phase:%f f_pdop:%f |\n",
      d_floatSTDCode, d_floatSTDPhase, f_pdop);
    return;
  }

  // statistic updated carrier num
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if ((pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]) > 0)
    {
      w_sumUpdatePhaseNum += pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i];
    }
    if ((pz_RTKfilterInfo->pu_measUpdatePseudoNum [u_i]) > 0)
    {
      w_sumUpdateCodeNum += pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i];
    }
    if ((pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]) > u_maxUpdatePhaseNum)
    {
      u_maxUpdatePhaseNum = (pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]);
    }
  }
  if (u_maxUpdatePhaseNum >= 5)
  {
    pz_RTKfilterInfo->q_filterPosValid = TRUE;
  }
  else
  {
    pz_RTKfilterInfo->q_filterPosValid = FALSE;
    return;
  }

  // statistic updated outline num
  b_openSky = ((pz_RTKfilterInfo->t_closeSkyCount & 0x3) == 0);
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_SatFilterObs = pz_filterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (b_openSky)
      {
        if (pz_SatFilterObs->pf_phaseResidual[u_signalIndex] > d_PhaseThresOpen)
        {
          w_rejPhaseNum++;
        }
        if (pz_SatFilterObs->pf_codeResidual[u_signalIndex] > d_CodeThresOpen)
        {
          w_rejCodeNum++;
        }
      }
      else
      {
        if (pz_SatFilterObs->pf_phaseResidual[u_signalIndex] > d_PhaseThresClose)
        {
          w_rejPhaseNum++;
        }
        if (pz_SatFilterObs->pf_codeResidual[u_signalIndex] > d_CodeThresClose)
        {
          w_rejCodeNum++;
        }
      }
    }
  }
  w_acceptCodeNum = w_sumUpdateCodeNum - w_rejCodeNum;
  w_acceptPhaseNum = w_sumUpdatePhaseNum - w_rejPhaseNum;

  /* judge with
  *  a. % of outage res
  *  b. std of code/phase res
  */
  if (w_acceptCodeNum < 6 && w_acceptPhaseNum < 6)
  {
    if (w_rejCodeNum >= w_sumUpdateCodeNum * 0.50 || w_rejPhaseNum >= w_sumUpdatePhaseNum * 0.50)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
    }
    else
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_VALID;
    }
  }
  else if (w_rejCodeNum >= w_sumUpdateCodeNum * 0.80 || w_rejPhaseNum >= w_sumUpdatePhaseNum * 0.67 ||
    d_floatSTDCode > 20.0 || d_floatSTDPhase > 2.0)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
  }
  else if (w_rejCodeNum >= w_sumUpdateCodeNum * 0.60 || w_rejPhaseNum >= w_sumUpdatePhaseNum * 0.50 ||
    d_floatSTDCode > 10.0 || d_floatSTDPhase > 1.5)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_WARN;
  }
  else
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_VALID;
  }

  /* judge with
  *  c. check with spp
  */
  //u_diffFlagSPP = rtk_getDiffFlagBewteen_spp_rtk(pz_RTKfilterInfo->pz_filterSol,
  //  &pz_RTKfilterInfo->pz_pvtResult->z_positionFix, RTK_FILTER_POS_FLOAT);
  //pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] <<= 2;
  //if (u_diffFlagSPP == RTK_DIFF_BETWEEN_POS_LARGE)
  //{
  //  pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] |= 0x3;
  //}
  //else if (u_diffFlagSPP == RTK_DIFF_BETWEEN_POS_SMALL)
  //{
  //  pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] |= 0x1;
  //}
  //if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_VALID)
  //{
  //  for (u_i = 0; u_i < 5 * 2; u_i++)//every epoch occupy 2 bit
  //  {
  //    if ((pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] >> u_i) & 1)
  //    {
  //      u_sumDiffSppEpoch++;
  //    }
  //  }
  //  if ((u_sumDiffSppEpoch >= 6 && u_diffFlagSPP == RTK_DIFF_BETWEEN_POS_SMALL) 
  //    || u_diffFlagSPP == RTK_DIFF_BETWEEN_POS_LARGE)
  //  {
  //    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_WARN;
  //  }
  //  //spp accuracy is not good now, so won't use it to reset filter (update in future)
  //  /*if (u_sumDiffSppEpoch >= 8 && u_diffFlagSPP == RTK_DIFF_BETWEEN_POS_LARGE)
  //  {
  //    pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
  //  }*/
  //}

  /* judge with
  *  d. check with predict
  */
  u_diffFlagPredict = rtk_getDiffFlagBewteen_history_current(pz_RTKfilterInfo->pz_filterSol, RTK_FILTER_POS_FLOAT);
  pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] <<= 2;
  if (u_diffFlagPredict == RTK_DIFF_BETWEEN_POS_LARGE)
  {
    pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] |= 0x3;
  }
  else if (u_diffFlagPredict == RTK_DIFF_BETWEEN_POS_SMALL)
  {
    pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] |= 0x1;
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_VALID)
  {
    for (u_i = 0; u_i < 5 * 2; u_i++)//every epoch occupy 2 bit
    {
      if ((pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[RTK_FILTER_POS_FLOAT] >> u_i) & 1)
      {
        u_sumDiffSppEpoch++;
      }
    }
    if ((u_sumDiffSppEpoch >= 5 && u_diffFlagPredict == RTK_DIFF_BETWEEN_POS_SMALL) 
      || u_diffFlagPredict == RTK_DIFF_BETWEEN_POS_LARGE)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
    }
    else if (u_sumDiffSppEpoch >= 2 && u_diffFlagPredict == RTK_DIFF_BETWEEN_POS_SMALL)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_WARN;
    }
  }

  /* judge with
  *  e. check with pdop
  */
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_VALID)
  {
    if (f_pdop > d_pdopThres || d_floatSTDPhase * f_pdop > d_pdopMultiStdThres)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_WARN;
    }
  }

  /* judge with
  *  f. check with long time not fix
  *  g. check with vehicle velocity in elevation
  */
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] != RTK_FILTER_SOL_STAT_RESET)
  {
    if (w_floatEpoch > w_floatEpochThres && d_floatSTDPhase > d_PhaseThresNotFix && d_resetPencent > d_resetPencentThres)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
      LOGI(TAG_HPP, "** Float Sol Check ** <RESET>  | Long time not fix | float epoch:%d, reset pencent:%f |\n",
        w_floatEpoch, d_resetPencent);
    }
    else if ((NULL != pz_RTKfilterInfo->pz_historyPos_t) && pz_RTKfilterInfo->pz_historyPos_t->u_num >= MAX_NUM_HISTORY_POS && (w_filterWithPhaseCount >= w_floatEpochThres))
    {
      double d_diffTime = 0.0;
      double d_deltaTime = 0.0;
      double pd_enu[3] = { 0.0 };
      double pd_lla[3] = { 0.0 };
      double pd_delta_pos[3] = { 0.0 };
      double pd_delta_pos_next[3] = { 0.0 };
      double pd_v[3] = { 0.0 };
      double pd_v_t[3] = { 0.0 };
      double pd_v_next[3] = { 0.0 };
      double pd_v_pvt[3] = { 0.0 };
      double pd_v_pvt_next[3] = { 0.0 };
      d_diffTime = fabs(tm_GpsTimeDiff(&(pz_filterObs->z_tor), &(pz_RTKfilterInfo->pz_historyPos_t->z_tor)));
      d_deltaTime = d_diffTime + pz_RTKfilterInfo->pz_historyPos_t->pd_deltaTime[0];
      if (d_diffTime <= 1.1 && d_deltaTime <= 2.1)
      {
        for (u_i = 0; u_i < 3; u_i++)
        {
          pd_delta_pos[u_i] = pz_filterSol->pd_filterPos[RTK_FILTER_POS_FLOAT][u_i] -
            pz_RTKfilterInfo->pz_historyPos_t->pd_historyPos[0][u_i];
          pd_delta_pos_next[u_i] = pz_filterSol->pd_filterPos[RTK_FILTER_POS_FLOAT][u_i] - 
            pz_RTKfilterInfo->pz_historyPos_t->pd_historyPos[1][u_i];
        }
        gnss_Ecef2Lla(pz_filterSol->pd_filterPos[RTK_FILTER_POS_FLOAT], pd_lla);
        gnss_Ecef2Enu(pd_lla, pd_delta_pos, pd_enu);
        memcpy(pd_delta_pos, pd_enu, sizeof(double) * 3);
        gnss_Ecef2Enu(pd_lla, pd_delta_pos_next, pd_enu);
        memcpy(pd_delta_pos_next, pd_enu, sizeof(double) * 3);

        for (u_i = 0; u_i < 3; u_i++)
        {
          pd_v[u_i] = (pd_delta_pos[u_i] / d_diffTime);
          pd_v_t[u_i] = ((pd_delta_pos_next[u_i] - pd_delta_pos[u_i]) / pz_RTKfilterInfo->pz_historyPos_t->pd_deltaTime[0]);
          pd_v_next[u_i] = (pd_delta_pos_next[u_i] / d_deltaTime);
          pd_v_pvt[u_i] = pz_RTKfilterInfo->pz_historyPos_t->pd_SPPvelENU[0][u_i];
          pd_v_pvt_next[u_i] = pz_RTKfilterInfo->pz_historyPos_t->pd_SPPvelENU[1][u_i];
        }
        //elevation
        if (fabs(pd_v[2] - pd_v_pvt[2]) > d_elevationVelThres && fabs(pd_v_next[2] - pd_v_pvt[2]) > d_elevationVelThres
          && fabs(pd_v_t[2] - pd_v_pvt[2]) > d_elevationVelThres)
        {
          pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
          LOGI(TAG_HPP, "** Float Sol Check ** <RESET>  | elevation velocity out threshold | v_pvt=%f | v_h:%f, v_h_next:%f, v_h_t:%f |\n",
            pd_v_pvt[2], pd_v[2], pd_v_next[2], pd_v_t[2]);
        }
        //Horizontal
        if (sqrt(SQR(pd_v[0] - pd_v_pvt[0]) + SQR(pd_v[1] - pd_v_pvt[1])) > d_horizontalVelThres
          && sqrt(SQR(pd_v_next[0] - pd_v_pvt_next[0]) + SQR(pd_v_next[1] - pd_v_pvt_next[1])) > d_horizontalVelThres
          && sqrt(SQR(pd_v_t[0] - pd_v_pvt_next[0]) + SQR(pd_v_t[1] - pd_v_pvt_next[1])) > d_horizontalVelThres)
        {
          pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] = RTK_FILTER_SOL_STAT_RESET;
          LOGI(TAG_HPP, "** Float Sol Check ** <RESET>  | horizontal velocity out threshold | v_pvt=%f,%f | v_h:%f,%f, v_h_next:%f,%f, v_h_t:%f,%f |\n",
            pd_v_pvt[0], pd_v_pvt[1], pd_v[0], pd_v[1], pd_v_next[0], pd_v_next[1], pd_v_t[0], pd_v_t[1]);
        }
        if (sqrt(SQR(pd_v_t[2] - pd_v_pvt_next[2])) > d_horizontalVelThres)
        {
          //update in future
        }
      }
    }
  }

  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_WARN)
  {
    LOGI(TAG_HPP, "** Float Sol Check ** <WARN!>  | opensky=%d | reject obs Code:%d Phase:%d | accept obs Code:%d Phase:%d\n",
      b_openSky, w_rejCodeNum, w_rejPhaseNum, w_acceptCodeNum, w_acceptPhaseNum);
    LOGI(TAG_HPP, "** Float Sol Check ** <WARN!>  | float STD Code:%f Phase:%f |\n",
      d_floatSTDCode, d_floatSTDPhase);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_VALID)
  {
    LOGI(TAG_HPP, "** Float Sol Check ** <VALID>  | opensky=%d | reject obs Code:%d Phase:%d | accept obs Code:%d Phase:%d\n",
      b_openSky, w_rejCodeNum, w_rejPhaseNum, w_acceptCodeNum, w_acceptPhaseNum);
    LOGI(TAG_HPP, "** Float Sol Check ** <VALID>  | float STD Code:%f Phase:%f |\n",
      d_floatSTDCode, d_floatSTDPhase);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_RESET)
  {
    LOGI(TAG_HPP, "** Float Sol Check ** <RESET>  | opensky=%d | reject obs Code:%d Phase:%d | accept obs Code:%d Phase:%d\n",
      b_openSky, w_rejCodeNum, w_rejPhaseNum, w_acceptCodeNum, w_acceptPhaseNum);
    LOGI(TAG_HPP, "** Float Sol Check ** <RESET>  | float STD Code:%f Phase:%f |\n",
      d_floatSTDCode, d_floatSTDPhase);
  }
}
/**
 * @brief check the valid of receiver clock in the filter
 * @param[in]      u_targetSys is the target constellation
 * @param[in]      pd_siteCoor is the initilization value of site coordination
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR product
 * @param[in]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pu_freqType is the valid frequency type
 * @param[out]     pd_rcvClkValue is the receiver clock value
 * @return         TRUE represent finded valid receiver clock in filter
 */
BOOL RTK_checkValidRcvClkInFilter(gnss_ConstellationType u_targetSys, const double pd_siteCoor[3], const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, const rtk_filterInfo_t* pz_RTKfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  gnss_FreqType* pu_freqType, double* pd_rcvClkReturnValue, uint8_t* pu_NormReturnNum)
{
  BOOL q_rcvClkInFilterValid = FALSE;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_iSat = 0;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  double d_pseduoSatDiff = 0.0;
  double d_roverDist = 0.0;
  double d_baseDist = 0.0;
  double d_omc = 0.0;
  double pd_s2r[3] = { 0.0 };
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  gnss_FreqType u_freqTypeTemp = C_GNSS_FREQ_TYPE_MAX;
  BOOL q_isBDS3Sat = FALSE;
  int16_t w_freq1ClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  double d_clkValue = 0.0;
  uint8_t pu_normNum[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint8_t pu_BDS3normNum[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  double pd_RcvClkValue[C_GNSS_FREQ_TYPE_MAX] = { 0.0 };
  double pd_BDS3RcvClkValue[C_GNSS_FREQ_TYPE_MAX] = { 0.0 };
  *pu_freqType = C_GNSS_FREQ_TYPE_MAX;
  *pd_rcvClkReturnValue = 0.0;
  *pu_NormReturnNum = 0;
  getRcvClkParaIndex(pz_EKFstateRepPool, pw_rcvClkIndex);
  if (FALSE == (pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_normNum[u_i] = 0;
    pu_BDS3normNum[u_i] = 0;
    pd_RcvClkValue[u_i] = 0.0;
    pd_BDS3RcvClkValue[u_i] = 0.0;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = pz_satMeas->u_constellation;
    if (C_GNSS_BDS2 == u_constellation)
    {
      u_constellation = C_GNSS_BDS3;
    }
    if (u_constellation != u_targetSys)
    {
      continue;
    }
    q_isBDS3Sat = gnss_isBDS3Sat(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    w_freq1ClkFilterIndex = pw_rcvClkIndex[pz_satMeas->u_constellation];
    if (INVALID_INDEX == w_freq1ClkFilterIndex)
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
      u_freqTypeTemp = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (u_freqTypeTemp >= C_GNSS_FREQ_TYPE_MAX)
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[u_freqTypeTemp * C_GNSS_MAX + pz_sigMeas->u_constellation];
      if (INVALID_INDEX == w_rcvDcbFilterIndex) // no clk
      {
        continue;
      }
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid))
      {
        continue;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount))
      {
        continue;
      }
      d_pseduoSatDiff = (pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_satMeas->z_satPosVelClk.d_satPosClk[u_i] - pd_siteCoor[u_i];
      }
      d_roverDist = gnss_Norm(pd_s2r, 3);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_s2r[u_i] = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
      }
      d_baseDist = gnss_Norm(pd_s2r, 3);
      d_clkValue = pz_RTKfilterInfo->pd_X[w_freq1ClkFilterIndex];
      if (C_GNSS_FREQ_TYPE_L1 != u_freqTypeTemp)
      {
        d_clkValue += pz_RTKfilterInfo->pd_X[w_rcvDcbFilterIndex];
      }
      d_omc = d_pseduoSatDiff - (d_roverDist - d_baseDist) - d_clkValue;
      if (fabs(d_omc) > 100.0)
      {
        continue;
      }
      if (TRUE == q_isBDS3Sat)
      {
        ++pu_BDS3normNum[u_freqTypeTemp];
        pd_BDS3RcvClkValue[u_freqTypeTemp] = d_clkValue;
      }
      else
      {
        ++pu_normNum[u_freqTypeTemp];
        pd_RcvClkValue[u_freqTypeTemp] = d_clkValue;
      }
    }
  }

  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (pu_normNum[u_i] > 1)
    {
      *pu_freqType = u_i;
      *pd_rcvClkReturnValue = pd_RcvClkValue[u_i];
      *pu_NormReturnNum = pu_normNum[u_i];
      q_rcvClkInFilterValid = TRUE;
      break;
    }
  }

  if (FALSE == q_rcvClkInFilterValid && C_GNSS_BDS3 == u_targetSys)
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      if (pu_BDS3normNum[u_i] > 1)
      {
        *pu_freqType = u_i;
        *pd_rcvClkReturnValue = pd_BDS3RcvClkValue[u_i];
        *pu_NormReturnNum = pu_BDS3normNum[u_i];
        q_rcvClkInFilterValid = TRUE;
        break;
      }
    }
  }
  return q_rcvClkInFilterValid;
}
/**
 * @brief calculate the initialition value of the receiver clock parameter
 * @param[in]      pd_siteCoor is the initilization value of site coordination
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR product
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @return         void
 */
void RTK_getRcvInitClk(const double pd_siteCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_index = 0;
  rtk_rcv_init_clk_info_t* pz_rcvInitClkInfo = pz_RTKfilterInfo->pz_rcvInitClkInfo;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  double d_rcvClkValue = 0.0;
  uint8_t u_obsNum = 0;
  BOOL q_rcvClkValid = FALSE;
  BOOL q_isSperateBDS2And3 = FALSE;
  for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
  {
    q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(u_j, pz_satSigMeasCollect);
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(q_isSperateBDS2And3, u_i))
      {
        continue;
      }
      u_index = u_j * C_GNSS_MAX + u_i;
      pz_rcvInitClkInfo->pu_clkStatus[u_index] = RTK_UsePseudoCalcInitRcvClk(q_isSperateBDS2And3, (gnss_ConstellationType)u_i, (gnss_FreqType)u_j,
        pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, &(pz_rcvInitClkInfo->pd_initRcvClockValue[u_index]), &(pz_rcvInitClkInfo->pq_usedObsNum[u_index]));
    }
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    if (C_GNSS_BDS2 == u_i)/*there only used C_GNSS_BDS3 to maintain consistent performance*/
    {
      continue;
    }
    q_rcvClkValid = FALSE;
    for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      u_index = u_j * C_GNSS_MAX + u_i;
      if (RTK_RCV_INIT_CLK_INVALID != pz_rcvInitClkInfo->pu_clkStatus[u_index])
      {
        q_rcvClkValid = TRUE;
      }
    }

    if (FALSE == q_rcvClkValid && TRUE == RTK_checkValidRcvClkInFilter(u_i, pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool,
      &u_freqType, &d_rcvClkValue, &u_obsNum))
    {
      u_index = u_freqType * C_GNSS_MAX + u_i;
      pz_rcvInitClkInfo->pu_clkStatus[u_index] = RTK_RCV_INIT_CLK_VALID;
      pz_rcvInitClkInfo->pd_initRcvClockValue[u_index] = d_rcvClkValue;
      pz_rcvInitClkInfo->pq_usedObsNum[u_index] = u_obsNum;
    }
  }

  for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
  {
    q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(u_j, pz_satSigMeasCollect);
    if (FALSE == q_isSperateBDS2And3)
    {
      /*copy the BDS3 initial value of receiver clock to BDS2*/
      pz_rcvInitClkInfo->pu_clkStatus[u_j * C_GNSS_MAX + C_GNSS_BDS2] = pz_rcvInitClkInfo->pu_clkStatus[u_j * C_GNSS_MAX + C_GNSS_BDS3];
      pz_rcvInitClkInfo->pq_usedObsNum[u_j * C_GNSS_MAX + C_GNSS_BDS2] = pz_rcvInitClkInfo->pq_usedObsNum[u_j * C_GNSS_MAX + C_GNSS_BDS3];
      pz_rcvInitClkInfo->pd_initRcvClockValue[u_j * C_GNSS_MAX + C_GNSS_BDS2] = pz_rcvInitClkInfo->pd_initRcvClockValue[u_j * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  //if receiver clock failure,used other frequency value replaced
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      u_index = u_j * C_GNSS_MAX + u_i;
      if (RTK_RCV_INIT_CLK_VALID == pz_rcvInitClkInfo->pu_clkStatus[u_index] || pz_rcvInitClkInfo->pq_usedObsNum[u_index] < 1)
      {
        continue;
      }
      for (u_k = 0; u_k < C_GNSS_FREQ_TYPE_MAX; ++u_k)
      {
        if (RTK_RCV_INIT_CLK_VALID == pz_rcvInitClkInfo->pu_clkStatus[u_k * C_GNSS_MAX + u_i])
        {
          pz_rcvInitClkInfo->pd_initRcvClockValue[u_index] = pz_rcvInitClkInfo->pd_initRcvClockValue[u_k * C_GNSS_MAX + u_i];
          pz_rcvInitClkInfo->pu_clkStatus[u_index] = RTK_RCV_INIT_CLK_VALID;
          break;
        }
      }
    }
  }
  return;
}

/**
 * @brief print out infomation of RTK float solution
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]      pz_filterObs is the filter obs information
 * @return         void
 */
void rtk_PrintInfoOfFilterSolute(rtk_filterInfo_t* pz_RTKfilterInfo, rtk_EpochFilterObs_t* pz_filterObs)
{
  uint8_t u_i = 0;
  uint8_t u_UpdateCarrierNum = 0;
  uint8_t u_UpdatePseudoNum = 0;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if ((pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]) > 0)
    {
      u_UpdateCarrierNum += (pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]);
    }
    if ((pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]) > 0)
    {
      u_UpdatePseudoNum += (pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]);
    }
  }

  //LOGW for realtime print out
  //LOGW is not encoded and it's exposed to clients, so could not be too comprehensive in writing
  LOGW(TAG_HPP, "--INFO--| CP=%.2f %d %.2f | PR=%.2f %d %.2f |\n",
    pz_filterObs->d_priorPhaseSTD, u_UpdateCarrierNum, pz_filterObs->d_postPhaseSTD,
    pz_filterObs->d_priorCodeSTD, u_UpdatePseudoNum, pz_filterObs->d_postCodeSTD);
}

/**
 * @brief the inteface of RTK float algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return         0 represent success and other failed
 */
int32_t RTK_filterSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  rtk_EpochFilterObs_t* pz_filterObs, GnssCorrSlipFlag_t* pz_corrSlipFlag)
{
  int32_t q_status = 0;
  BOOL z_measUpdateStatus = TRUE;
  uint8_t u_i = 0;
  uint16_t w_i = 0;
  double pd_siteCoor[3] = { 0.0 };
  double pd_siteVel[3] = { 0.0 };
  double pd_refCoor[3] = { 0.0 };
  double pd_s2r[3] = { 0.0 };
  double d_baseline = 0.0;
  double* pd_X = NULL;
  pz_RTKfilterInfo->q_filterPosValid = FALSE;
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pz_satSigMeasCollect->z_positionFix.d_xyz[u_i];
    pd_siteVel[u_i] = (double)(pz_satSigMeasCollect->z_positionFix.f_velXyz[u_i]);
    if (NULL != pz_rtkCorrBlock)
    {
      pd_refCoor[u_i] = pz_rtkCorrBlock->d_refPosEcef[u_i];
    }
  }
  RTK_getRcvInitClk(pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool);
  //update obs time
  if (NULL != pz_filterObs)
  {
    pz_filterObs->z_tor = pz_satSigMeasCollect->z_tor;
  }
  if (TRUE == (pz_RTKfilterInfo->q_QRcheckStatus))
  {
    RTK_markPseudoRangeUsingQRresult(pz_satSigMeasCollect);
  }
  else
  {
    RTK_PseodoRangeQualityControl(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool);
  }
  RTK_timeUpdateFilterPara(pd_siteCoor, pd_siteVel, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_corrSlipFlag);
  pd_X = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
  if (NULL == pd_X)
  {
    LOGW(TAG_HPP, "%s filter pd_X cant malloc %d bytes\n", __FUNCTION__, pz_RTKfilterInfo->w_nmax * sizeof(double));
    q_status = 1;
  }
  else
  {
    for (w_i = 0; w_i < (pz_RTKfilterInfo->w_nmax); ++w_i)
    {
      pd_X[w_i] = (pz_RTKfilterInfo->pd_X[w_i]);
      pz_RTKfilterInfo->pd_deltaX[w_i] = 0.0;
    }
  }

  //calculate prior residual, check & reset AMB corresponding to out-threshold phase
  RTK_priorResidualCheck(pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
    pz_EKFstateRepPool, NULL, pz_filterObs);

  z_measUpdateStatus = RTK_obsMeasUpdate(pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo, pz_EKFstateRepPool, pz_filterObs);

  //calculate posterior residual, check & reset AMB corresponding to out-threshold phase
  RTK_postResidualCheck(pz_RTKfilterInfo->pd_X, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
    pz_EKFstateRepPool, NULL, pz_filterObs);

  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_s2r[u_i] = pz_RTKfilterInfo->pd_X[u_i] - pd_refCoor[u_i];
    pz_RTKfilterInfo->pd_StationCoordinate[u_i] = pd_refCoor[u_i];
  }
  d_baseline = gnss_Norm(pd_s2r, 3);
  LOGI(TAG_HPP, "baseline=%.3f m\n", d_baseline);

  // update stationID of VRS
  pz_RTKfilterInfo->q_StationID = pz_rtkCorrBlock->w_refStationId;

  rtk_PrintInfoOfFilterSolute(pz_RTKfilterInfo, pz_filterObs);

  if (FALSE == z_measUpdateStatus)
  {
    q_status = 2;
  }
  else
  {
    rtk_floatSol_availabilityCheck(pz_RTKfilterInfo, pz_filterObs);
  }
  OS_FREE(pd_X);
  return q_status;
}

/**
 * @brief      check filter need to reset or not
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0x0: No need to reset;
                         0x1: close sky;
                         0x2: float filter thres outage;
 */
uint8_t rtk_filterResetCheck(rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i = 0;
  uint8_t u_status = 0;
  uint8_t u_sumCloseEpoch = 0;
  uint8_t u_maxUpdateCarrierNum = 0;
  uint8_t u_maxUpdatePseudoNum = 0;

  // close sky check
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if ((pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]) > u_maxUpdateCarrierNum)
    {
      u_maxUpdateCarrierNum = (pz_RTKfilterInfo->pu_measUpdateCarrierNum[u_i]);
    }
    if ((pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]) > u_maxUpdatePseudoNum)
    {
      u_maxUpdatePseudoNum = (pz_RTKfilterInfo->pu_measUpdatePseudoNum[u_i]);
    }
  }
  u_sumCloseEpoch = 0;
  for (u_i = 0; u_i < 32; u_i++)
  {
    if ((pz_RTKfilterInfo->t_closeSkyCount >> u_i) & 1)
    {
      u_sumCloseEpoch++;
    }
  }
  if (u_maxUpdateCarrierNum < 5 && u_maxUpdatePseudoNum < 5 &&
    u_sumCloseEpoch > 24 && (pz_RTKfilterInfo->t_closeSkyCount & 0xFF) == 0xFF)
  {
    u_status |= 0x1;
  }

  //float filter thres outage
  if (pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] == RTK_FILTER_SOL_STAT_RESET)
  {
    u_status |= 0x2;
  }

  //float filter thres outage
  if (pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_RTD] == RTK_FILTER_SOL_STAT_RESET &&
    pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_FLOAT] < RTK_FILTER_SOL_STAT_WARN)
  {
    u_status |= 0x4;
  }

  //reset
  if (u_status)
  {
    if (u_status & 0x1)
    {
      LOGW(TAG_HPP, " -- Reset Check : close sky , need to reset\n");
      rtk_resetFilter(pz_RTKfilterInfo, pz_EKFstateRepPool);
    }
    else if (u_status & 0x2)
    {
      LOGW(TAG_HPP, " -- Reset Check : float filter thres outage, need to reset\n");
      rtk_resetFilter(pz_RTKfilterInfo, pz_EKFstateRepPool);
    }
    else if (u_status & 0x4)
    {
      LOGW(TAG_HPP, " -- Reset Check : continuous rtd, need to reset\n");
      rtk_resetFilter(pz_RTKfilterInfo, pz_EKFstateRepPool);
      //removeAMBEKFstatus(pz_EKFstateRepPool, pz_RTKfilterInfo->w_nmax, pz_RTKfilterInfo->pd_X,
      //  pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pq_paraValid);//reset AMB
    }
  }

  return u_status;
}

/**
 * @brief      reset RTK filter module
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t rtk_resetFilter(rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_status = 0;
  uint16_t w_i = 0;
  pz_RTKfilterInfo->w_n = 0;
  pz_RTKfilterInfo->u_start = 0;
  if (any_Ptrs_Null(5, pz_RTKfilterInfo->pd_deltaX, pz_RTKfilterInfo->pd_X,
    pz_RTKfilterInfo->pd_Q, pz_RTKfilterInfo->pz_filterSol, pz_RTKfilterInfo->pq_paraValid))
  {
    u_status = 1;
  }
  else
  {
    //initilize the information of EKF status
    tm_initGpsTime(&(pz_EKFstateRepPool->z_gpsTime));
    pz_EKFstateRepPool->d_continueFilterTime = 0.0;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM && !u_status; ++w_i)
    {
      if (NULL == pz_EKFstateRepPool->pz_satPool[w_i])
      {
        u_status = 2;
        continue;
      }
      pz_EKFstateRepPool->pz_satPool[w_i]->w_index = INVALID_INDEX;
      initEKFstateRepresent(pz_EKFstateRepPool->pz_satPool[w_i], pz_RTKfilterInfo->w_nmax, pz_RTKfilterInfo->pd_X,
        pz_RTKfilterInfo->pd_Q);
    }
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
    {
      pz_RTKfilterInfo->pq_paraValid[w_i] = FALSE;
    }
    pz_RTKfilterInfo->z_kfStatus = KF_INIT;
  }

  for (w_i = 0; w_i < MAX_FILTER_POS_STATUS; w_i++)
  {
    pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithSPP[w_i] = 0x0;
    pz_RTKfilterInfo->pz_filterSol->pq_diffFlagWithPredict[w_i] = 0x0;
  }

  pz_RTKfilterInfo->w_floatCount = 0;
  pz_RTKfilterInfo->w_rtdCount = 0;
  pz_RTKfilterInfo->w_filterWithPhaseCount = 0;
  pz_RTKfilterInfo->t_openSkyCount = 0;
  pz_RTKfilterInfo->t_closeSkyCount = 0;

  if (u_status)
  {
    LOGW(TAG_HPP, " -- RTK filter reset failed\n");
  }
  else
  {
    LOGW(TAG_HPP, " -- RTK filter reset success\n");
  }

  return u_status;
}

/**
 * @brief      check positive definite of gz_RTKfilterInfo's P mat module
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0: Positive Definite; other: error case
 */
uint8_t rtk_positiveDefiniteCheck(rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i = 0;
  uint8_t u_status = 0;
  int16_t w_index = 0;
  int16_t w_i = 0;

  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    if (NULL != pz_EKFstateRepPool->pz_satPool[w_i])
    {
      w_index = (pz_EKFstateRepPool->pz_satPool[w_i]->w_index);
      if (w_index >= 0 && pz_RTKfilterInfo->pd_Q[IUTM(w_index, w_index)] < -FABS_ZEROS)
      {
        u_status = 2;
        break;
      }
    }
  }

  if (u_status)
  {
    LOGW(TAG_HPP, " -- Reset Check : [Q] mat of rtk is not Positive Definite, need to reset\n");
    rtk_resetFilter(pz_RTKfilterInfo, pz_EKFstateRepPool);
  }

  return u_status;
}