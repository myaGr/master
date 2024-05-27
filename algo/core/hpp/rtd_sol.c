#include "rtd_sol.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include "cmn_qr_parity_check.h"
#include <math.h>

#define SOL_DIFF_INVALID_VALUE  (999.9)
#define SOL_CONSISTENCE_THRES  (10.0)
typedef struct
{
  uint32_t q_satIndex;
  uint8_t u_signalIndex;
}RTD_obsIndex_t;

typedef struct
{
  matrix_t* pz_H;
  matrix_t* pz_P;
  matrix_t* pz_L;
  matrix_t* pz_deltaX;
  RTD_obsIndex_t* pz_obsIndex;
  int8_t ps_sysIndex[C_GNSS_MAX];
  float pf_rcvClockInit[C_GNSS_MAX];
}RTD_obsLinearInfo_t;

typedef struct
{
  double pd_unitVector[3];
  double d_omc;
  double d_weight;
  double d_roverDist;
}RTD_sat2siteInfo_t;
/**
 * @brief Initialize RTD module
 * @param[in]  pz_opt represent the algorithm option for the RTD
 * @param[out] pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t rtd_init(const rtk_alg_opt_t* pz_opt, rtd_filterInfo_t* pz_RTDfilterInfo, rtd_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint16_t w_i = 0;
  if (NULL == pz_RTDfilterInfo)
  {
    LOGE(TAG_HPP, "pz_RTDfilterInfo is NULL\n");
    u_status = 1;
  }
  else
  {
    pz_RTDfilterInfo->z_kfStatus = KF_INIT;
    pz_RTDfilterInfo->w_nmax = 3 + C_GNSS_MAX;
    pz_RTDfilterInfo->w_n = 0;
    pz_RTDfilterInfo->q_filterPosValid = FALSE;
    pz_RTDfilterInfo->f_age = 0.0;
    pz_RTDfilterInfo->pd_deltaX = (double*)OS_MALLOC((pz_RTDfilterInfo->w_nmax) * sizeof(double));
    pz_RTDfilterInfo->pd_X = (double*)OS_MALLOC((pz_RTDfilterInfo->w_nmax) * sizeof(double));
    pz_RTDfilterInfo->pd_Q = (double*)OS_MALLOC(NUTM(pz_RTDfilterInfo->w_nmax) * sizeof(double));
    pz_RTDfilterInfo->pq_paraValid = (BOOL*)OS_MALLOC((pz_RTDfilterInfo->w_nmax) * sizeof(BOOL));
    pz_RTDfilterInfo->z_opt = *pz_opt;
    pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo = (rtd_QRcheckSiteCoorInfo_t*)OS_MALLOC_FAST(C_GNSS_FREQ_TYPE_MAX * sizeof(rtd_QRcheckSiteCoorInfo_t));
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pz_RTDfilterInfo->pu_measUpdatePseudoNum[u_i] = 0;
    }
    if (any_Ptrs_Null(5, pz_RTDfilterInfo->pd_deltaX, pz_RTDfilterInfo->pd_X,
      pz_RTDfilterInfo->pd_Q, pz_RTDfilterInfo->pq_paraValid, pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo))
    {
      LOGW(TAG_HPP, "rtd_init X or Q malloc failed");
      OS_FREE(pz_RTDfilterInfo->pd_deltaX);
      OS_FREE(pz_RTDfilterInfo->pd_X);
      OS_FREE(pz_RTDfilterInfo->pd_Q);
      OS_FREE(pz_RTDfilterInfo->pq_paraValid);
      OS_FREE(pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo);
      u_status = 1;
    }
    else
    {
      //initilize the information of EKF status
      tm_initGpsTime(&(pz_EKFstateRepPool->z_gpsTime));
      pz_EKFstateRepPool->d_continueFilterTime = 0.0;
      for (w_i = 0; w_i < (pz_RTDfilterInfo->w_nmax) && !u_status; ++w_i)
      {
        pz_EKFstateRepPool->pz_satPool[w_i] = (gnss_EKFstateRepresent_t*)OS_MALLOC(1 * sizeof(gnss_EKFstateRepresent_t));
        if (NULL == pz_EKFstateRepPool->pz_satPool[w_i])
        {
          u_status = 2;
          break;
        }
        pz_EKFstateRepPool->pz_satPool[w_i]->w_index = INVALID_INDEX;
        initEKFstateRepresent(pz_EKFstateRepPool->pz_satPool[w_i], pz_RTDfilterInfo->w_nmax, pz_RTDfilterInfo->pd_X,
          pz_RTDfilterInfo->pd_Q);
      }
      pz_RTDfilterInfo->z_kfStatus = KF_INIT;
      memset(pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo, 0, C_GNSS_FREQ_TYPE_MAX * sizeof(rtd_QRcheckSiteCoorInfo_t));
    }
  }
  return u_status;
}
/**
 * @brief de-initialize RTD module
 * @param[out] pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void rtd_deinit(rtd_filterInfo_t* pz_RTDfilterInfo, rtd_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i;
  if (NULL != pz_RTDfilterInfo)
  {
    OS_FREE(pz_RTDfilterInfo->pd_deltaX);
    OS_FREE(pz_RTDfilterInfo->pd_X);
    OS_FREE(pz_RTDfilterInfo->pd_Q);
    OS_FREE(pz_RTDfilterInfo->pq_paraValid);
    OS_FREE(pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo);
    if (NULL != pz_EKFstateRepPool)
    {
      for (u_i = 0; u_i < (pz_RTDfilterInfo->w_nmax); ++u_i)
      {
        OS_FREE(pz_EKFstateRepPool->pz_satPool[u_i]);
      }
    }
    OS_FREE(pz_RTDfilterInfo);
  }
  return;
}
/**
 * @brief initilize the information of QR check struct
* @param[out]     pz_QRcheckSiteCoorInfo is the information of QR check struct
 */
void rtd_initQRcheckSiteCoorInfo(rtd_QRcheckSiteCoorInfo_t* pz_QRcheckSiteCoorInfo)
{
  uint8_t u_i;
  pz_QRcheckSiteCoorInfo->q_QRsuccess = FALSE;
  pz_QRcheckSiteCoorInfo->u_PRtotalNum = 0;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_QRcheckSiteCoorInfo->pu_PRnum[u_i] = 0;
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_QRcheckSiteCoorInfo->pd_QRcheckSiteCoor[u_i] = 0.0;
  }
  return;
}
/**
 * @brief determine the valid of pseudo-range
* @param[in]      pz_sigMeas is the observation information for one signal
* @param[in]      pz_rtkCorrBlock is the OSR correction
* @param[in]      pz_satMeas is the satellite measure
* @return         TRUE represent valid, FALSE represent invalid
 */
BOOL RTD_determineSignalPRvalid(const gnss_SignalMeas_t* pz_sigMeas, const GnssCorrBlock_t* pz_rtkCorrBlock, const gnss_SatelliteMeas_t* pz_satMeas)
{
  BOOL q_signalPRvalid = TRUE;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  uint16_t w_iSat = 0;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  if (NULL != pz_sigMeas)
  {
    e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
    if ((pz_sigMeas->d_pseudoRange) < 1.0e-2 || !(pz_sigMeas->z_measStatusFlag.b_prValid))
    {
      q_signalPRvalid = FALSE;
    }
    else if (e_freqType >= C_GNSS_FREQ_TYPE_MAX)
    {
      q_signalPRvalid = FALSE;
    }
    else if (PR_QUALITY_NORM != (pz_satMeas->pz_signalQuality[e_freqType].u_PRquality))
    {
      q_signalPRvalid = FALSE;
    }
    else
    {
      pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
      if (NULL == pz_corrMeasReslut || (pz_corrMeasReslut->d_pseudoRange) < 1.0e-2)
      {
        q_signalPRvalid = FALSE;
      }
      w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
      if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
        || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
      {
        q_signalPRvalid = FALSE;
      }
    }
  }
  else
  {
    q_signalPRvalid = FALSE;
  }
  return q_signalPRvalid;
}

/**
 * @brief count the number of pseudo-range observations for the target frequency
* @param[in]      q_isSperateBDS2And3 whether sperate for BDS2 and BDS3
* @param[in]      e_targetFreq is the target frequecny
* @param[in]      pz_satSigMeasCollect is the observation information
* @param[in]      pz_rtkCorrBlock is the OSR correction
* @param[in]      d_elmin is elevation mask angle (rad)
* @param[out]     pu_PRnum is the number of pseudo-range observations for each frequency
* @return         the number of pseudo-range observations for the target frequency
 */
uint8_t RTD_countTargetFreqObsNum(BOOL q_isSperateBDS2And3, gnss_FreqType e_targetFreq, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  double d_elmin, uint8_t pu_PRnum[C_GNSS_MAX])
{
  uint8_t u_PRtotalNum = 0;
  uint8_t u_i = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_constellation = 0;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pu_PRnum[u_i] = 0;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < d_elmin)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (FALSE == RTD_determineSignalPRvalid(pz_sigMeas, pz_rtkCorrBlock, pz_satMeas))
      {
        continue;
      }
      e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_freqType >= C_GNSS_FREQ_TYPE_MAX || e_freqType != e_targetFreq)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_satMeas->u_constellation);
      ++pu_PRnum[u_constellation];
    }
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    u_PRtotalNum += pu_PRnum[u_i];
  }
  return u_PRtotalNum;
}

/**
 * @brief select the optimal frequency to do solute RTD
* @param[in]      pz_satSigMeasCollect is the observation information
* @param[in]      pz_rtkCorrBlock is the OSR correction
* @param[in]      d_elmin is elevation mask angle (rad)
* @return         the optimal frequency to do solute RTD
 */
gnss_FreqType RTD_selectOptimalFreq(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock, double d_elmin)
{
  gnss_FreqType e_selectedFreq = C_GNSS_FREQ_TYPE_L1;
  uint8_t u_PRmaxNum = 0;
  uint8_t u_i = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  uint8_t pu_PRnum[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_PRnum[u_i] = 0;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < d_elmin)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (FALSE == RTD_determineSignalPRvalid(pz_sigMeas, pz_rtkCorrBlock, pz_satMeas))
      {
        continue;
      }
      e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_freqType >= C_GNSS_FREQ_TYPE_MAX)
      {
        continue;
      }
      ++pu_PRnum[e_freqType];
    }
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (pu_PRnum[u_i] > u_PRmaxNum)
    {
      u_PRmaxNum = pu_PRnum[u_i];
      e_selectedFreq = (gnss_FreqType)u_i;
    }
  }
  return e_selectedFreq;
}
/**
 * @brief calculating the number of parameter of RTD solution
 * @param[in]      pu_PRnum[C_GNSS_MAX] is the number of pseudo-range observations
 * @param[out]     ps_sysIndex[C_GNSS_MAX] is the index of constellation
 * @return         the number of parameter used by RTD
 */
uint8_t RTD_calParaNum(const uint8_t pu_PRnum[C_GNSS_MAX], int8_t ps_sysIndex[C_GNSS_MAX])
{
  uint8_t u_paraNum = 0;
  uint8_t u_sysValidNum = 0;
  uint8_t u_i = 0;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    ps_sysIndex[u_i] = -1;
    if (pu_PRnum[u_i] > 1)
    {
      ps_sysIndex[u_i] = 3 + u_sysValidNum;
      ++u_sysValidNum;
    }
  }
  u_paraNum = 3 + u_sysValidNum;
  return u_paraNum;
}
/**
* @brief obtain the value of observations minus calculation for pseudo-range
* @param[in]      pd_siteCoor is the site coordinate
* @param[in]      pz_sigMeas is the observation information for one signal
* @param[in]      pz_rtkCorrBlock is the OSR correction
* @param[in]      pz_satMeas is the satellite measure
* @param[out]     pz_sat2siteInfo is the information of satellite to site
* @return         TRUE represent valid, FALSE represent invalid
 */
BOOL RTD_obsMinusCalculation(const double pd_siteCoor[3], const gnss_SignalMeas_t* pz_sigMeas, const GnssCorrBlock_t* pz_rtkCorrBlock, 
  const gnss_SatelliteMeas_t* pz_satMeas, RTD_sat2siteInfo_t* pz_sat2siteInfo)
{
  uint8_t u_i = 0;
  BOOL q_signalPRvalid = TRUE;
  double pd_satPosRot[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  uint16_t w_iSat = 0;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const double* pd_baseSatPosClk = NULL;
  double pd_s2r[3] = { 0.0 };
  double d_baseDist = 0.0;
  double d_baseEarthRot = 0.0;
  double d_sinEle = 0.0;
  pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
  gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
  pz_sat2siteInfo->d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pz_sat2siteInfo->pd_unitVector);
  pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
  if (NULL == pz_corrMeasReslut || (pz_corrMeasReslut->d_pseudoRange) < 1.0e-2)
  {
    q_signalPRvalid = FALSE;
  }
  w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
  if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
    || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
  {
    q_signalPRvalid = FALSE;
  }
  if (TRUE == q_signalPRvalid)
  {
    pd_baseSatPosClk = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk;
    for (u_i = 0; u_i < 3; u_i++)
    {
      pd_s2r[u_i] = pd_baseSatPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
    }
    d_baseDist = gnss_Norm(pd_s2r, 3);
    d_baseEarthRot = (pd_baseSatPosClk[0] * pz_rtkCorrBlock->d_refPosEcef[1] -
      pd_baseSatPosClk[1] * pz_rtkCorrBlock->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;
    (pz_sat2siteInfo->d_omc) = ((pz_sigMeas->d_pseudoRange) - (pz_corrMeasReslut->d_pseudoRange)) - ((pz_sat2siteInfo->d_roverDist) - (d_baseDist + d_baseEarthRot));
    (pz_sat2siteInfo->d_omc) += (pd_satPosClk[3] - pd_baseSatPosClk[3]);
    (pz_sat2siteInfo->d_weight) = 1.0;
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < 30.0 * DEG2RAD)
    {
      d_sinEle = sin((pz_satMeas->z_satPosVelClk.f_elevation));
      (pz_sat2siteInfo->d_weight) = 4.0 * d_sinEle * d_sinEle;
    }
  }
  return q_signalPRvalid;
}
/**
 * @brief get initial value of receiver clock  for the target constellation and target frequency
 * @param[in] e_TargetSysType is the target constellation
 * @param[in] e_targetFreqType is the target frequency
 * @param[in] pd_siteCoor is the site coordinate
 * @param[in] pz_satSigMeasCollect is the observation information
 * @param[in] pz_rtkCorrBlock is the OSR correction
 * @param[in] pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out]pf_rcvDriftClk is the value of receiver clock drift for the target frequency
 * @return TRUE represent successful and FALSE represent failure
 */
BOOL RTD_calTargetFreqRcvDriftClock(gnss_ConstellationType e_TargetSysType, gnss_FreqType e_targetFreqType, const double pd_siteCoor[3], const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, 
  const GnssCorrBlock_t* pz_rtkCorrBlock, const rtd_filterInfo_t* pz_RTDfilterInfo, float* pf_rcvDriftClk)
{
  BOOL q_success = TRUE;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_num = 0;
  uint8_t u_normNum = 0;
  uint8_t u_i = 0;
  uint8_t u_flag = 0;
  uint8_t u_constellation = 0;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_normDiff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_diffAbs[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_mean = 0.0;
  float f_rejectThres = 0.0;
  float f_rejectK2 = 8.0;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  RTD_sat2siteInfo_t z_sat2siteInfo = { 0 };
  memset(&z_sat2siteInfo, 0, sizeof(RTD_sat2siteInfo_t));
  *pf_rcvDriftClk = 0.0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (e_TargetSysType != u_constellation)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTDfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (FALSE == RTD_determineSignalPRvalid(pz_sigMeas, pz_rtkCorrBlock, pz_satMeas))
      {
        continue;
      }
      e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_freqType >= C_GNSS_FREQ_TYPE_MAX || e_freqType != e_targetFreqType)
      {
        continue;
      }
      z_sat2siteInfo.d_omc = 0.0;
      z_sat2siteInfo.d_roverDist = 0.0;
      z_sat2siteInfo.d_weight = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        z_sat2siteInfo.pd_unitVector[u_i] = 0.0;
      }
      if (FALSE == RTD_obsMinusCalculation(pd_siteCoor, pz_sigMeas, pz_rtkCorrBlock, pz_satMeas, &z_sat2siteInfo))
      {
        continue;
      }
      pf_diff[u_num] = (float)z_sat2siteInfo.d_omc;
      ++u_num;
    }
  }
  u_flag = gnss_MadFloat(pf_diff, (uint32_t)u_num, pf_diffAbs, &f_mean);
  if (1 == u_flag)
  {
    f_rejectThres = (float)(f_rejectK2 * (f_mean / 0.6745f));
    u_normNum = 0;
    for (u_i = 0; u_i < u_num; ++u_i)
    {
      if (pf_diffAbs[u_i] > f_rejectThres && pf_diffAbs[u_i] > 10.0)
      {
        continue;
      }
      pf_normDiff[u_normNum++] = pf_diff[u_i];
    }
    u_flag = gnss_ascSortMedianFloat(pf_normDiff, (uint32_t)u_normNum, &f_mean);
  }
  if (!u_flag || u_normNum<2)
  {
    q_success = FALSE;
  }
  else
  {
    *pf_rcvDriftClk = f_mean;
  }
  return q_success;
}
/**
 * @brief Linear the pseudo-range observations
 * @param[in]      pz_pvtResult is information of pvt
 * @param[in]      e_selectedFreq is the target frequency
 * @param[in]      u_PRtotalNum is the total number of pseudo-range observations
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[in]      pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out]     pz_obsLinearInfo is the information of pseudo-range observations lined
 * @return         void
 */
void RTD_obsLinear(const gnss_PVTResult_t* pz_pvtResult, gnss_FreqType e_selectedFreq, uint8_t u_PRtotalNum, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, 
  const GnssCorrBlock_t* pz_rtkCorrBlock, const rtd_filterInfo_t* pz_RTDfilterInfo, RTD_obsLinearInfo_t* pz_obsLinearInfo)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_index = 0;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  int8_t s_sysIndex = -1;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  RTD_sat2siteInfo_t z_sat2siteInfo = { 0};
  memset(&z_sat2siteInfo, 0, sizeof(RTD_sat2siteInfo_t));
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_index >= u_PRtotalNum)
    {
      break;
    }
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    s_sysIndex = (pz_obsLinearInfo->ps_sysIndex[u_constellation]);
    if (s_sysIndex < 0 || (pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTDfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (u_index >= u_PRtotalNum)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (FALSE == RTD_determineSignalPRvalid(pz_sigMeas, pz_rtkCorrBlock, pz_satMeas))
      {
        continue;
      }
      e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_freqType >= C_GNSS_FREQ_TYPE_MAX || e_freqType != e_selectedFreq)
      {
        continue;
      }
      z_sat2siteInfo.d_omc = 0.0;
      z_sat2siteInfo.d_roverDist = 0.0;
      z_sat2siteInfo.d_weight = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        z_sat2siteInfo.pd_unitVector[u_i] = 0.0;
      }
      if (FALSE == RTD_obsMinusCalculation(pz_pvtResult->z_positionFix.d_xyz, pz_sigMeas, pz_rtkCorrBlock, pz_satMeas, &z_sat2siteInfo))
      {
        continue;
      }
      for (u_i = 0; u_i < 3; ++u_i)
      {
        MAT(pz_obsLinearInfo->pz_H, u_index, u_i) = z_sat2siteInfo.pd_unitVector[u_i];
      }
      MAT(pz_obsLinearInfo->pz_H, u_index, s_sysIndex) = 1.0;
      MAT(pz_obsLinearInfo->pz_L, u_index, 0) = z_sat2siteInfo.d_omc - (pz_obsLinearInfo->pf_rcvClockInit[u_constellation]);
      MAT(pz_obsLinearInfo->pz_P, u_index, 0) = z_sat2siteInfo.d_weight;
      pz_obsLinearInfo->pz_obsIndex[u_index].q_satIndex = q_satIndex;
      pz_obsLinearInfo->pz_obsIndex[u_index].u_signalIndex = u_signalIndex;
      ++u_index;
    }
  }
  return;
}
/**
 * @brief Using QR parity method to detect the gross error in pseudo-range
 * @param[in]      pz_pvtResult is information of pvt
 * @param[in]      e_selectedFreq is the target frequency
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTDfilterInfo the filter for the RTD algorithm
 * @return         void
 */
void RTD_checkPseudoRangeUsingQR(const gnss_PVTResult_t* pz_pvtResult, gnss_FreqType e_selectedFreq, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,rtd_filterInfo_t* pz_RTDfilterInfo)
{
  BOOL q_success = FALSE;
  QR_check_status u_checkStatus = QR_CHECK_UNKNOW;
  uint8_t u_i = 0;
  uint8_t u_PRtotalNum = 0;
  uint8_t pu_PRnum[C_GNSS_MAX] = { 0 };
  uint8_t u_maxPRsysNum = 0;
  uint8_t u_paraNum = 0;
  uint8_t u_maxIterNum = 10;
  uint8_t u_iterNum = 0;
  int32_t q_grossIndex = -1;
  RTD_obsLinearInfo_t z_obsLinearInfo;
  RTD_obsIndex_t* pz_obsIndex = NULL;
  rtd_QRcheckSiteCoorInfo_t* pz_QRcheckSiteCoorInfo= &(pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo[e_selectedFreq]);
  rtd_initQRcheckSiteCoorInfo(pz_QRcheckSiteCoorInfo);
  z_obsLinearInfo.pz_H = NULL;
  z_obsLinearInfo.pz_P = NULL;
  z_obsLinearInfo.pz_L = NULL;
  z_obsLinearInfo.pz_obsIndex = NULL;
  u_PRtotalNum = RTD_countTargetFreqObsNum(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, e_selectedFreq, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTDfilterInfo->z_opt.d_elmin, pu_PRnum);
  u_paraNum = RTD_calParaNum(pu_PRnum, z_obsLinearInfo.ps_sysIndex);
  while (u_PRtotalNum > (u_paraNum + 2))
  {
    q_success = FALSE;
    if (u_iterNum > u_maxIterNum)
    {
      break;
    }
    z_obsLinearInfo.pz_H = matrix_new((uint32_t)u_PRtotalNum, (uint32_t)u_paraNum);
    z_obsLinearInfo.pz_P = matrix_new((uint32_t)u_PRtotalNum, 1);
    z_obsLinearInfo.pz_L = matrix_new((uint32_t)u_PRtotalNum, 1);
    z_obsLinearInfo.pz_deltaX = matrix_new((uint32_t)u_paraNum, 1);
    z_obsLinearInfo.pz_obsIndex = OS_MALLOC_FAST(u_PRtotalNum * sizeof(RTD_obsIndex_t));
    for (u_i = 0; u_i < u_PRtotalNum; ++u_i)
    {
      z_obsLinearInfo.pz_obsIndex[u_i].q_satIndex = 0;
      z_obsLinearInfo.pz_obsIndex[u_i].u_signalIndex = 0;
    }
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, u_i))
      {
        continue;
      }
      RTD_calTargetFreqRcvDriftClock(u_i, e_selectedFreq, pz_pvtResult->z_positionFix.d_xyz, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTDfilterInfo, &(z_obsLinearInfo.pf_rcvClockInit[u_i]));
    }
    if (FALSE == pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3)
    {
      z_obsLinearInfo.pf_rcvClockInit[C_GNSS_BDS2] = z_obsLinearInfo.pf_rcvClockInit[C_GNSS_BDS3];
    }
    RTD_obsLinear(pz_pvtResult, e_selectedFreq, u_PRtotalNum, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTDfilterInfo, &z_obsLinearInfo);
    q_grossIndex = -1;
    u_checkStatus = cmn_qrParityCheckAcoordingToPostRes(3.0, z_obsLinearInfo.pz_H, z_obsLinearInfo.pz_P, z_obsLinearInfo.pz_L, &q_grossIndex, z_obsLinearInfo.pz_deltaX);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_QRcheckSiteCoorInfo->pd_QRcheckSiteCoor[u_i] = pz_pvtResult->z_positionFix.d_xyz[u_i] + MAT(z_obsLinearInfo.pz_deltaX, u_i, 0);
    }
    matrix_free(&z_obsLinearInfo.pz_H);
    matrix_free(&z_obsLinearInfo.pz_P);
    matrix_free(&z_obsLinearInfo.pz_L);
    matrix_free(&z_obsLinearInfo.pz_deltaX);
    if (QR_CHECK_OBS_NORMAL == u_checkStatus)
    {
      q_success = TRUE;
      OS_FREE(z_obsLinearInfo.pz_obsIndex);
      break;
    }
    if (QR_CHECK_IDETIFY_OBS_SUCC == u_checkStatus)
    {
      pz_obsIndex = &(z_obsLinearInfo.pz_obsIndex[q_grossIndex]);
      pz_satSigMeasCollect->pz_satMeas[pz_obsIndex->q_satIndex]->pz_signalQuality[pz_obsIndex->u_signalIndex].u_PRquality |= PR_QUALITY_GROSS_BY_QR_PARITY;
      u_PRtotalNum = RTD_countTargetFreqObsNum(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, e_selectedFreq, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTDfilterInfo->z_opt.d_elmin, pu_PRnum);
      u_paraNum = RTD_calParaNum(pu_PRnum, z_obsLinearInfo.ps_sysIndex);
      ++u_iterNum;
    }
    /*if (QR_CHECK_UNKNOW == u_checkStatus || QR_CHECK_OBS_Insufficient == u_checkStatus || QR_CHECK_IDETIFY_OBS_FAIL == u_checkStatus)
    {
      OS_FREE(z_obsLinearInfo.pz_obsIndex);
      break;
    }*/
    OS_FREE(z_obsLinearInfo.pz_obsIndex);
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_QRcheckSiteCoorInfo->pu_PRnum[u_i] = pu_PRnum[u_i];
    if (pu_PRnum[u_i] > u_maxPRsysNum)
    {
      u_maxPRsysNum = pu_PRnum[u_i];
    }
  }
  if (u_PRtotalNum < (u_paraNum + 2) || u_maxPRsysNum < (3 + 1 + 2))
  {
    q_success = FALSE;
  }
  pz_QRcheckSiteCoorInfo->q_QRsuccess = q_success;
  pz_QRcheckSiteCoorInfo->u_PRtotalNum = u_PRtotalNum;
  return;
}
/**
 * @brief mark the gross error of pseudo-range  for the target constellation and target frequency
 * @param[in] e_TargetSysType is the target constellation
 * @param[in] e_targetFreqType is the target frequency
 * @param[in] pd_siteCoor is the site coordinate
 * @param[in] pz_rtkCorrBlock is the OSR correction
 * @param[in] pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out] pz_satSigMeasCollect is the observation information
 * @return void
 */
void RTD_markPseudoRangeGross(gnss_ConstellationType e_TargetSysType, gnss_FreqType e_targetFreqType, const double pd_siteCoor[3], const GnssCorrBlock_t* pz_rtkCorrBlock,
  const rtd_filterInfo_t* pz_RTDfilterInfo, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_constellation = 0;
  float f_rcvClk = 0.0;
  BOOL q_getRcvClksuccess = TRUE;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  RTD_sat2siteInfo_t z_sat2siteInfo = { 0 };
  memset(&z_sat2siteInfo, 0, sizeof(RTD_sat2siteInfo_t));
  q_getRcvClksuccess = RTD_calTargetFreqRcvDriftClock(e_TargetSysType, e_targetFreqType, pd_siteCoor, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTDfilterInfo, &f_rcvClk);
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_constellation != e_TargetSysType)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_RTDfilterInfo->z_opt.d_elmin))
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
      e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_freqType >= C_GNSS_FREQ_TYPE_MAX || e_freqType != e_targetFreqType)
      {
        continue;
      }
      pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality = PR_QUALITY_NORM;
      if (FALSE == q_getRcvClksuccess)
      {
        pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality |= PR_QUALITY_GROSS_BY_QR_PARITY;
        continue;
      }
      if (FALSE == RTD_determineSignalPRvalid(pz_sigMeas, pz_rtkCorrBlock, pz_satMeas))
      {
        continue;
      }
      z_sat2siteInfo.d_omc = 0.0;
      z_sat2siteInfo.d_roverDist = 0.0;
      z_sat2siteInfo.d_weight = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        z_sat2siteInfo.pd_unitVector[u_i] = 0.0;
      }
      if (FALSE == RTD_obsMinusCalculation(pd_siteCoor, pz_sigMeas, pz_rtkCorrBlock, pz_satMeas, &z_sat2siteInfo))
      {
        continue;
      }
      z_sat2siteInfo.d_omc -= f_rcvClk;
      if (fabs(z_sat2siteInfo.d_omc) > 5.0)
      {
        pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality |= PR_QUALITY_GROSS_BY_QR_PARITY;
      }
    }
  }
  return;
}
/**
 * @brief determine the valid of each frequency solution
 * @param[in]      pz_QRcheckSiteCoorInfo is information of site coordinate calculated by QR check
 * @param[out]     pq_freqSolValid is the valid of each frequency solution
 * @return         TRUE represent success and other failed
 */
BOOL rtd_determineSolutionValid(const rtd_QRcheckSiteCoorInfo_t* pz_QRcheckSiteCoorInfo, BOOL pq_freqSolValid[C_GNSS_FREQ_TYPE_MAX])
{
  BOOL q_status = FALSE;
  double d_L1L2soluteDist = SOL_DIFF_INVALID_VALUE;
  double d_L1L5soluteDist = SOL_DIFF_INVALID_VALUE;
  double d_L2L5soluteDist = SOL_DIFF_INVALID_VALUE;
  double d_delta = 0.0;
  const double* pd_firstFreqSiteCoor = NULL;
  const double* pd_secondFreqSiteCoor = NULL;
  uint8_t u_i = 0;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pq_freqSolValid[u_i] = FALSE;
  }
  //the distance between L1 and L2
  if (TRUE == pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L1].q_QRsuccess && TRUE == pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L2].q_QRsuccess)
  {
    d_delta = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_firstFreqSiteCoor = pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L1].pd_QRcheckSiteCoor;
      pd_secondFreqSiteCoor = pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L2].pd_QRcheckSiteCoor;
      d_delta += (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]) * (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]);
    }
    d_L1L2soluteDist = sqrt(d_delta);
  }
  //the distance between L1 and L5
  if (TRUE == pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L1].q_QRsuccess && TRUE == pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L5].q_QRsuccess)
  {
    d_delta = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_firstFreqSiteCoor = pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L1].pd_QRcheckSiteCoor;
      pd_secondFreqSiteCoor = pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L5].pd_QRcheckSiteCoor;
      d_delta += (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]) * (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]);
    }
    d_L1L5soluteDist = sqrt(d_delta);
  }

  //the distance between L2 and L5
  if (TRUE == pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L2].q_QRsuccess && TRUE == pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L5].q_QRsuccess)
  {
    d_delta = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_firstFreqSiteCoor = pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L2].pd_QRcheckSiteCoor;
      pd_secondFreqSiteCoor = pz_QRcheckSiteCoorInfo[C_GNSS_FREQ_TYPE_L5].pd_QRcheckSiteCoor;
      d_delta += (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]) * (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]);
    }
    d_L2L5soluteDist = sqrt(d_delta);
  }
  //determine the valid ofsolution
  if (d_L1L2soluteDist < SOL_CONSISTENCE_THRES)
  {
    pq_freqSolValid[C_GNSS_FREQ_TYPE_L1] = TRUE;
    pq_freqSolValid[C_GNSS_FREQ_TYPE_L2] = TRUE;
    if (d_L1L5soluteDist < SOL_CONSISTENCE_THRES || d_L2L5soluteDist < SOL_CONSISTENCE_THRES)
    {
      pq_freqSolValid[C_GNSS_FREQ_TYPE_L5] = TRUE;
    }
  }
  else if (d_L1L5soluteDist < SOL_CONSISTENCE_THRES)
  {
    pq_freqSolValid[C_GNSS_FREQ_TYPE_L1] = TRUE;
    pq_freqSolValid[C_GNSS_FREQ_TYPE_L5] = TRUE;
    if (d_L1L2soluteDist < SOL_CONSISTENCE_THRES || d_L2L5soluteDist < SOL_CONSISTENCE_THRES)
    {
      pq_freqSolValid[C_GNSS_FREQ_TYPE_L2] = TRUE;
    }
  }
  else if (d_L2L5soluteDist < SOL_CONSISTENCE_THRES)
  {
    pq_freqSolValid[C_GNSS_FREQ_TYPE_L2] = TRUE;
    pq_freqSolValid[C_GNSS_FREQ_TYPE_L5] = TRUE;
    //If-condition below always evaluates to false.
    /*if (d_L1L2soluteDist < SOL_CONSISTENCE_THRES || d_L1L5soluteDist < SOL_CONSISTENCE_THRES)
    {
      pq_freqSolValid[C_GNSS_FREQ_TYPE_L1] = TRUE;
    }*/
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (TRUE == pq_freqSolValid[u_i])
    {
      q_status = TRUE;
      break;
    }
  }
  return q_status;
}
/**
 * @brief the inteface of RTD solute algorithm
 * @param[in]      pz_pvtResult is information of pvt
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return         0 represent success and other failed
 */
int32_t RTD_solute(const gnss_PVTResult_t* pz_pvtResult, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtd_filterInfo_t* pz_RTDfilterInfo, rtd_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  int32_t q_status = 0;
  uint8_t u_iFreq = 0;
  uint8_t u_iSys = 0;
  uint8_t u_maxPseudoRangeNum = 0;
  uint8_t u_optSolutionIndex = 0;
  BOOL pq_freqSolValid[C_GNSS_FREQ_TYPE_MAX] = { FALSE };
  for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
  {
    pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(u_iFreq, pz_satSigMeasCollect);
    RTD_checkPseudoRangeUsingQR(pz_pvtResult, u_iFreq, pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTDfilterInfo);
  }
  if (FALSE == rtd_determineSolutionValid(pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo, pq_freqSolValid))
  {
    LOGI(TAG_HPP, "QR check failure\n");
    q_status = 1;
  }
  else
  {
    for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
    {
      if (FALSE == pq_freqSolValid[u_iFreq])
      {
        continue;
      }
      if ((pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo[u_iFreq].u_PRtotalNum) > u_maxPseudoRangeNum)
      {
        u_maxPseudoRangeNum = (pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo[u_iFreq].u_PRtotalNum);
        u_optSolutionIndex = u_iFreq;
      }
    }
    for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
    {
      if (TRUE == pq_freqSolValid[u_iFreq])
      {
        continue;
      }
      pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(u_iFreq, pz_satSigMeasCollect);
      for (u_iSys = C_GNSS_GPS; u_iSys < C_GNSS_MAX; ++u_iSys)
      {
        if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTDfilterInfo->z_opt.q_isSperateBDS2And3, u_iSys))
        {
          continue;
        }
        RTD_markPseudoRangeGross(u_iSys, u_iFreq, pz_RTDfilterInfo->pz_QRcheckSiteCoorInfo[u_optSolutionIndex].pd_QRcheckSiteCoor, pz_rtkCorrBlock, pz_RTDfilterInfo, pz_satSigMeasCollect);
      }
    }
  }
  return q_status;
}