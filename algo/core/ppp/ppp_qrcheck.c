#include "ppp_qrcheck.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include "cmn_qr_parity_check.h"
#include "ppp_type.h" 
#include <math.h>

typedef struct
{
  uint32_t        q_satIndex;
  uint8_t         u_signalIndex;
}QR_obsIndex_t;

typedef struct
{
  matrix_t*       pz_H;
  matrix_t*       pz_P;
  matrix_t*       pz_L;
  matrix_t*       pz_deltaX;
  QR_obsIndex_t*  pz_obsIndex;
  int8_t          ps_sysIndex[C_GNSS_MAX];
  int8_t          ps_ztdIndex;
  float           pf_rcvClockInit[C_GNSS_MAX];
}QR_obsLinearInfo_t;

typedef struct
{
  double          pd_unitVector[3];
  double          d_omc;
  double          d_weight;
  double          d_roverDist;
}QR_sat2siteInfo_t;

typedef struct
{
  BOOL            q_QRsuccess;         /*the status of QR check,TRUE represent successful and FALSE represent failure*/
  BOOL            q_ionUseFlag;        /*FALSE represent do not use stec of los and TRUE represent use stec of los*/
  uint8_t         u_PRtotalNum;        /*the total number of norm pseudo-range observations*/
  uint8_t         pu_PRnum[C_GNSS_MAX];/*the number of norm pseudo-range observations for per constellation*/
  double          pd_QRcheckSiteCoor[3];/*the site coordinate calculated by QR check*/
}ppp_QRcheckInfo_t;


/**
* @brief initilize the information of QR parity check struct
* @param[out]   pz_QRcheckSiteCoorInfo is the information of QR check struct
* @return     void
*/
void qr_initQRcheckSiteCoorInfo(ppp_QRcheckInfo_t* pz_QRcheckSiteCoorInfo)
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
* @brief determine the valid of pseudo-range signal
* @param[in]      pz_sigMeas is the observation information for one signal
* @param[in]      pz_SsrLocBlock is the SSR product
* @param[in]      pz_satMeas is the satellite measure
* @return         TRUE represent valid, FALSE represent invalid
 */
BOOL QR_determineSignalPRvalid(const gnss_SignalMeas_t* pz_sigMeas, const gnss_ssrLosBlock_t* pz_SsrLocBlk, const gnss_SatelliteMeas_t* pz_satMeas)
{
  BOOL q_signalPRvalid = TRUE;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;

  if (NULL != pz_sigMeas)
  {
    e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
    if((pz_sigMeas->d_pseudoRange) < 1.0e-2)
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
    else if (!pz_sigMeas->z_measStatusFlag.b_prValid)
    {
      q_signalPRvalid = FALSE;
    }
    else
    {
      const gnss_satSsrLos_t* pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
      if (NULL == pz_satLos)
      {
        q_signalPRvalid = FALSE;
        return q_signalPRvalid;
      }
      /* ssr brdc satellite posclk */
      if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH) || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
        || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
      {
        q_signalPRvalid = FALSE;
      }
      if (integrity_check(pz_satLos->z_orbClk.u_postItg, pz_satLos->z_orbClk.u_preItg) >= 1.0)
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
* @param[in]      e_targetFreq is the target frequecny
* @param[in]      d_elmin is the min elevation mask
* @param[in]      pz_satSigMeasCollect is the observation information
* @param[in]      pz_SsrLocBlk  is the SSR product
* @param[out]      pz_QRcheckInfo is the information of QRcheck
* @param[out]     pu_PRnum is the number of pseudo-range observations for each frequency
* @return         the number of pseudo-range observations for the target frequency
 */
uint8_t QR_countTargetFreqObsNum(gnss_FreqType e_targetFreq, double d_elmin, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect
  , const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_QRcheckInfo_t* pz_QRcheckInfo, uint8_t pu_PRnum[C_GNSS_MAX])
{
  uint8_t u_PRtotalNum = 0;
  uint8_t u_i = 0;
  uint8_t u_sys = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_noSsrIonNum = 0;
  uint32_t q_satIndex = 0;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  double d_dt = 0;
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
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }
    u_sys = pz_satMeas->u_constellation;
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      const gnss_satSsrLos_t* pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
      if (NULL == pz_satLos)
      {
        continue;
      }
      d_dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime);
      if (fabs(d_dt) > PPP_MAX_SSR_AGE)
      {
        continue;
      }
      if (FALSE == QR_determineSignalPRvalid(pz_sigMeas, pz_SsrLocBlk, pz_satMeas))
      {
        continue;
      }  
      e_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_freqType >= C_GNSS_FREQ_TYPE_MAX || e_freqType != e_targetFreq)
      {
        continue;
      }   
      /*Calculate the number of no ionospheric corrections of SSR*/
      if (!(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR) || (fabsf(pz_satLos->z_stec.f_qi) > 10.0))
      {
        ++u_noSsrIonNum;
      }
      ++pu_PRnum[u_sys];
    }
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    u_PRtotalNum += pu_PRnum[u_i];
  }
  pz_QRcheckInfo->q_ionUseFlag = TRUE;
  /* Use PPPfilter value to modify ionosphere delay when the ionospheric correction of SSR is missing by more than 10% */
  if (u_PRtotalNum > 0)
  {
    if ( (1.0*u_noSsrIonNum / u_PRtotalNum)> 0.1 )
    {
      pz_QRcheckInfo->q_ionUseFlag = FALSE;
    }
  }
  return u_PRtotalNum;
}
/**
 * @brief calculating the number of parameter of QR parity check module
 * @param[in]      e_selectedFreq is the target frequecny
 * @param[in]      pu_PRnum[C_GNSS_MAX] is the number of pseudo-range observations
 * @return         the number of parameter to be estimated used by QR parity check
 */
uint8_t QR_calParaNum(gnss_FreqType e_selectedFreq,const uint8_t pu_PRnum[C_GNSS_MAX], QR_obsLinearInfo_t* pz_obsLinearInfo)
{
  uint8_t u_paraNum = 0;
  uint8_t u_sysValidNum = 0;
  uint8_t u_i = 0;
  pz_obsLinearInfo->ps_ztdIndex = -1;

   for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_obsLinearInfo->ps_sysIndex[u_i] = -1;
    if (pu_PRnum[u_i] > 1)
    {
      pz_obsLinearInfo->ps_sysIndex[u_i] = 3 + u_sysValidNum;
      ++u_sysValidNum;
    }
  }
  //pz_obsLinearInfo->ps_ztdIndex = 3 + u_sysValidNum ;
  /*xyz + rcvclk */
  u_paraNum = 3 + u_sysValidNum;

  return u_paraNum;
}
/**
* @brief obtain the value of observations minus calculation for pseudo-range
* @param[in]      q_ionUseFlag is whether to use stec of los (TRUE:use)
* @param[in]      pd_siteCoor is the site coordinate
* @param[in]      pz_sigMeas is the observation information for one signal
* @param[in]      pz_SsrLocBlk is the SSR product
* @param[in]      pz_satMeas is the satellite measure
* @param[in]      pz_satSigMeasCollect  is the observation information
* @param[in]      pz_PPPfilterInfo is the filter for the PPP-RTK algorithm 
* @param[in]      pz_EKFstateRepPool is the pool of EKF state represent 
* @param[out]     pz_sat2siteInfo is the information of satellite to site
* @return         TRUE represent valid, FALSE represent invalid
 */
BOOL QR_obsMinusCalculation(BOOL q_ionUseFlag,const double pd_siteCoor[3], const gnss_SignalMeas_t* pz_sigMeas, const gnss_ssrLosBlock_t* pz_SsrLocBlk, const gnss_SatelliteMeas_t* pz_satMeas,
  const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, QR_sat2siteInfo_t* pz_sat2siteInfo)
{
  uint8_t u_i = 0;
  BOOL q_signalPRvalid = TRUE;
  double pd_satPosRot[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  double d_sinEle = 0.0;
  const gnss_SignalCorr_t* pz_signal_corr = NULL;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  double pw_siteCoor[3] = { 0.0 };
  double d_dist = 0.0;
  double pd_unitVector[3] = { 0.0 };
  double d_beta = 0.0;
  double ssr_singal_dcb = 0.0;
  double ssr_sat_orclk = 0.0;
  double d_dt = 0.0;
  double d_stecFact = 0.0;
  double d_ionoCorrection = 0.0;
  
  const gnss_satSsrLos_t* pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
  if (NULL == pz_satLos)
  {
    q_signalPRvalid = FALSE;
    return q_signalPRvalid;
  }
  d_dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime);
  if (fabs(d_dt) > PPP_MAX_SSR_AGE)
  {
    q_signalPRvalid = FALSE;
  }
  ssr_sat_orclk = pz_satLos->z_orbClk.q_corr * 0.001;
  pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
  gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
  pz_sat2siteInfo->d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pz_sat2siteInfo->pd_unitVector);
  if (fabs(ssr_sat_orclk) > 30.0||!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    q_signalPRvalid = FALSE;
    return q_signalPRvalid;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  if (TRUE == q_signalPRvalid)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pw_siteCoor[u_i] = pz_PPPfilterInfo->pd_X[pw_PVAindex[u_i]];
    }
    if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_valid)
    {
      q_signalPRvalid = FALSE;
      return q_signalPRvalid;
    }
    if (w_ZTDindex < 0)
    {
      q_signalPRvalid = FALSE;
      return q_signalPRvalid;
    }
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector); 
    /* ion */
    d_stecFact = 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(pz_sigMeas->u_constellation));
    d_ionoCorrection = 0.001 * (pz_satLos->z_stec.q_corr) * d_stecFact;
    d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
    w_id[0] = 0;
    w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
    w_id[2] = GNSS_FILTER_STATE_IONO;
    getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
    /* SSR code bias */
    ssr_singal_dcb = 9999.999;
    pz_signal_corr = getSSR_Bias(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_SsrLocBlk);
    if (NULL != pz_signal_corr)
    {
      ssr_singal_dcb = pz_signal_corr->z_codeBias.q_corr * 0.001;
    }
    /* OMC*/
    pz_sat2siteInfo->d_omc = (pz_sigMeas->d_pseudoRange) - (d_dist - pd_satPosClk[3]);
    pz_sat2siteInfo->d_omc -= ((pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap) + (pz_PPPfilterInfo->pd_X[w_ZTDindex] * pz_satMeas->d_wetMap));
    if (!q_ionUseFlag)
    {
      if (w_ionoIndex < 0)
      {
        q_signalPRvalid = FALSE;
        return q_signalPRvalid;
      }
      pz_sat2siteInfo->d_omc -= (d_beta * pz_PPPfilterInfo->pd_X[w_ionoIndex]);
    }
    else 
    {
      if (!(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR))
      {
        q_signalPRvalid = FALSE;
        return q_signalPRvalid;
      }
      pz_sat2siteInfo->d_omc -= (d_beta * d_ionoCorrection);
    }

    pz_sat2siteInfo->d_omc -= ssr_sat_orclk;
    if (fabs(ssr_singal_dcb) < 100.0)
    {
      (pz_sat2siteInfo->d_omc) += ssr_singal_dcb;
    }
    /* weight */
    (pz_sat2siteInfo->d_weight) = 1.0;
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < 30.0 * DEG2RAD)
    {
      d_sinEle = sin((pz_satMeas->z_satPosVelClk.f_elevation));
      (pz_sat2siteInfo->d_weight) = 4.0 * d_sinEle * d_sinEle;
    }
    pz_sat2siteInfo->d_roverDist = d_dist;
  }
  return q_signalPRvalid;
}

/**
 * @brief get initial value of receiver clock  for the target constellation and target frequency
 * @param[in]      e_TargetSysType is the target constellation
 * @param[in]      e_targetFreqType is the target frequency
 * @param[in]      pd_siteCoor is the site coordinate
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR procduct
 * @param[in]      pz_QRcheckInfo is the information of QRcheck
 * @param[in]      pz_PPPfilterInfo is the filter for the PPP algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pf_rcvDriftClk is the value of receiver clock drift for the target frequency
 * @return TRUE represent successful and FALSE represent failure
 */
BOOL QR_calTargetFreqRcvDriftClock(gnss_ConstellationType e_TargetSysType, gnss_FreqType e_targetFreqType, const double pd_siteCoor[3], const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_QRcheckInfo_t* pz_QRcheckInfo, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, float* pf_rcvDriftClk)
{
  BOOL q_success = TRUE;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_num = 0;
  uint8_t u_normNum = 0;
  uint8_t u_i = 0;
  uint8_t u_flag = 0;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_normDiff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_diffAbs[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_mean = 0.0;
  float f_rejectThres = 0.0;
  float f_rejectK2 = 8.0;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  QR_sat2siteInfo_t z_sat2siteInfo = { 0.0 };

  *pf_rcvDriftClk = 0.0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas || (pz_satMeas->u_constellation) != e_TargetSysType)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_PPPfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
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
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      if (FALSE == QR_determineSignalPRvalid(pz_sigMeas, pz_SsrLocBlk, pz_satMeas))
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
      if (FALSE == QR_obsMinusCalculation(pz_QRcheckInfo->q_ionUseFlag,pd_siteCoor, pz_sigMeas, pz_SsrLocBlk, pz_satMeas, pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, &z_sat2siteInfo))
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
  if (!u_flag || u_normNum < 2)
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
 * @param[in]      e_selectedFreq is the target frequency
 * @param[in]      u_PRtotalNum is the total number of pseudo-range observations
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR correction
 * @param[in]      pz_QRcheckInfo is the information of QRcheck
 * @param[out]     pz_PPPfilterInfo the filter for the PPP algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pz_obsLinearInfo is the information of pseudo-range observations lined
 * @return         void
 */
uint8_t QR_obsLinear(gnss_FreqType e_selectedFreq, uint8_t u_PRtotalNum, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_QRcheckInfo_t* pz_QRcheckInfo, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, QR_obsLinearInfo_t* pz_obsLinearInfo)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_index = 0;
  uint8_t u_i = 0;
  int8_t s_sysIndex = -1;
  gnss_ConstellationType u_sys = C_GNSS_MAX;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  double d_dt = 0;
  QR_sat2siteInfo_t z_sat2siteInfo = { 0.0 };
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
    u_sys = pz_satMeas->u_constellation;
    const gnss_satSsrLos_t* pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    d_dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime);
    if (fabs(d_dt) > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    s_sysIndex = (pz_obsLinearInfo->ps_sysIndex[u_sys]);
    if (s_sysIndex < 0 || (pz_satMeas->z_satPosVelClk.f_elevation) < (pz_PPPfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
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
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      if (FALSE == QR_determineSignalPRvalid(pz_sigMeas, pz_SsrLocBlk, pz_satMeas))
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
      if (FALSE == QR_obsMinusCalculation(pz_QRcheckInfo->q_ionUseFlag,pz_satSigMeasCollect->z_positionFix.d_xyz, pz_sigMeas, pz_SsrLocBlk, pz_satMeas, pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, &z_sat2siteInfo))
      { 
        continue;
      }
      for (u_i = 0; u_i < 3; ++u_i)
      {
        MAT(pz_obsLinearInfo->pz_H, u_index, u_i) = z_sat2siteInfo.pd_unitVector[u_i];
      }
      MAT(pz_obsLinearInfo->pz_H, u_index, s_sysIndex) = 1.0;  
      MAT(pz_obsLinearInfo->pz_L, u_index, 0) = z_sat2siteInfo.d_omc - (pz_obsLinearInfo->pf_rcvClockInit[u_sys]);
      MAT(pz_obsLinearInfo->pz_P, u_index, 0) = z_sat2siteInfo.d_weight;
      pz_obsLinearInfo->pz_obsIndex[u_index].q_satIndex = q_satIndex;
      pz_obsLinearInfo->pz_obsIndex[u_index].u_signalIndex = u_signalIndex;
      ++u_index;
    }
  } 
  return u_index;
}
/**
 * @brief mark the gross error of pseudo-range  for the target constellation and target frequency
 * @param[in]   e_TargetSysType is the target constellation
 * @param[in]   e_targetFreqType is the target frequency
 * @param[in]   pd_siteCoor is the site coordinate
 * @param[in]   pz_SsrLocBlk is the SSR correction
 * @param[in]   pz_QRfilterInfo the filter for the QR parity check algorithm
 * @param[out]  pz_PPPfilterInfo the filter for the PPP algorithm
 * @param[in]   pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]  pz_satSigMeasCollect is the observation information
 * @return void
 */
void QR_markPseudoRangeGross(gnss_ConstellationType e_TargetSysType, gnss_FreqType e_targetFreqType, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  ppp_QRcheckInfo_t* pz_QRcheckInfo, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_nb = 0;
  char c_buff[256]="";
  char c_sat[4] = "";
  float f_rcvClk = 0.0;
  BOOL q_getRcvClksuccess = TRUE;
  gnss_ConstellationType u_sys = C_GNSS_MAX;
  gnss_FreqType e_freqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  QR_sat2siteInfo_t z_sat2siteInfo = { 0.0 };
  q_getRcvClksuccess = QR_calTargetFreqRcvDriftClock(e_TargetSysType, e_targetFreqType, pz_satSigMeasCollect->z_positionFix.d_xyz
    , pz_satSigMeasCollect, pz_SsrLocBlk, pz_QRcheckInfo, pz_PPPfilterInfo, pz_EKFstateRepPool, &f_rcvClk);
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_sys = pz_satMeas->u_constellation;
    if (u_sys != e_TargetSysType)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_PPPfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
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
      if (FALSE == QR_determineSignalPRvalid(pz_sigMeas, pz_SsrLocBlk, pz_satMeas))
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
      if (FALSE == QR_obsMinusCalculation(pz_QRcheckInfo->q_ionUseFlag, pz_satSigMeasCollect->z_positionFix.d_xyz,
        pz_sigMeas, pz_SsrLocBlk, pz_satMeas, pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, &z_sat2siteInfo))
      {
        continue;
      }
      z_sat2siteInfo.d_omc -= f_rcvClk;
      
      if (fabs(z_sat2siteInfo.d_omc) > 5.0)
      {
        pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality |= PR_QUALITY_GROSS_BY_QR_PARITY;

        if (log_GetLogLevel() >= LOG_LEVEL_I)
        {
          satidx_SatString(q_satIndex, c_sat);
          if (u_nb <= 0) u_nb += snprintf(&c_buff[u_nb], 17, "QR_check res-pr:");
          u_nb += snprintf(&c_buff[u_nb], 21, " %3s f=%01d v=%8.3lf;", c_sat, u_signalIndex, z_sat2siteInfo.d_omc);
          if (u_nb > 220)
          {
            LOGI(TAG_PPP, "%s\n", c_buff);
            strcpy(c_buff, ""); u_nb = 0;
          }
        }
      }
    }
  }
  if (u_nb > 0)
  {
    LOGI(TAG_PPP, "%s\n", c_buff);
    strcpy(c_buff, "");
  }
  return;
}

/**
 * @brief determine the valid of each frequency solution
 * @param[in]      pz_QRcheckInfo is information of site coordinate calculated by QR parity check
 * @param[out]     pq_freqSolValid is the valid of each frequency solution
 * @return         TRUE represent successs and other failed
 */
BOOL qr_determineSolutionValid(const ppp_QRcheckInfo_t* pz_QRcheckInfo, BOOL pq_freqSolValid[C_GNSS_FREQ_TYPE_MAX])
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
  if (TRUE == pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L1].q_QRsuccess && TRUE == pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L2].q_QRsuccess)
  {
    d_delta = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_firstFreqSiteCoor = pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L1].pd_QRcheckSiteCoor;
      pd_secondFreqSiteCoor = pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L2].pd_QRcheckSiteCoor;
      d_delta += (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]) * (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]);
    }
    d_L1L2soluteDist = sqrt(d_delta);
  }
  //the distance between L1 and L5
  if (TRUE == pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L1].q_QRsuccess && TRUE == pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L5].q_QRsuccess)
  {
    d_delta = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_firstFreqSiteCoor = pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L1].pd_QRcheckSiteCoor;
      pd_secondFreqSiteCoor = pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L5].pd_QRcheckSiteCoor;
      d_delta += (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]) * (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]);
    }
    d_L1L5soluteDist = sqrt(d_delta); 
  }

  //the distance between L2 and L5
  if (TRUE == pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L2].q_QRsuccess && TRUE == pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L5].q_QRsuccess)
  {
    d_delta = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_firstFreqSiteCoor = pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L2].pd_QRcheckSiteCoor;
      pd_secondFreqSiteCoor = pz_QRcheckInfo[C_GNSS_FREQ_TYPE_L5].pd_QRcheckSiteCoor;
      d_delta += (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]) * (pd_firstFreqSiteCoor[u_i] - pd_secondFreqSiteCoor[u_i]);
    }
    d_L2L5soluteDist = sqrt(d_delta);  }
  //determine the valid of solution
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
    if (d_L1L2soluteDist < SOL_CONSISTENCE_THRES || d_L1L5soluteDist < SOL_CONSISTENCE_THRES)
    {
      pq_freqSolValid[C_GNSS_FREQ_TYPE_L1] = TRUE;
    }
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (TRUE == pq_freqSolValid[u_i])
    {
      q_status = TRUE;
      break;
    }
  }
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    if (FALSE == pq_freqSolValid[u_i]&& pz_QRcheckInfo[u_i].q_QRsuccess)
    {
      LOGI(TAG_PPP, "QR pos info:L1L2_dist=%.3lf L1L5_dist=%.3lf L2L5_dist=%.3lf\n", d_L1L2soluteDist, d_L1L5soluteDist, d_L2L5soluteDist);
      break;
    }
  }
  
  return q_status;
}

/**
 * @brief Using QR parity check to detect the gross error in pseudo-range
 * @param[in]      e_selectedFreq is the target frequency
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      pz_PPPfilterInfo is the filter for the PPP algorithm 
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent 
 * @param[out]     pz_QRcheckInfo is the information of QRcheck
 * @return         void
 */
void ppp_checkPseudoRangeUsingQR(gnss_FreqType e_selectedFreq, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_QRcheckInfo_t* pz_QRcheckInfo)
{
  BOOL q_success = FALSE;
  QR_check_status u_checkStatus = QR_CHECK_UNKNOW;
  uint8_t u_i = 0;
  uint8_t u_nb = 0;
  uint8_t u_PRtotalNum = 0;
  uint8_t pu_PRnum[C_GNSS_MAX] = { 0 };
  uint8_t u_maxPRsysNum = 0;
  uint8_t u_paraNum = 0;
  uint8_t u_maxIterNum = 10;
  uint8_t u_iterNum = 0;
  int32_t q_grossIndex = -1;
  char c_sat[4] = "";
  char c_buff[256]="";
  QR_obsLinearInfo_t z_obsLinearInfo;
  QR_obsIndex_t* pz_obsIndex = NULL;
  qr_initQRcheckSiteCoorInfo(pz_QRcheckInfo);
  z_obsLinearInfo.pz_H = NULL;
  z_obsLinearInfo.pz_P = NULL;
  z_obsLinearInfo.pz_L = NULL;
  z_obsLinearInfo.pz_obsIndex = NULL;
  u_PRtotalNum = QR_countTargetFreqObsNum(e_selectedFreq, pz_PPPfilterInfo->z_opt.d_elmin, pz_satSigMeasCollect,
    pz_SsrLocBlk, pz_QRcheckInfo, pu_PRnum);
  u_paraNum = QR_calParaNum(e_selectedFreq,pu_PRnum, &z_obsLinearInfo);

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
    z_obsLinearInfo.pz_obsIndex = OS_MALLOC_FAST(u_PRtotalNum * sizeof(QR_obsIndex_t));
    for (u_i = 0; u_i < u_PRtotalNum; ++u_i)
    {
      z_obsLinearInfo.pz_obsIndex[u_i].q_satIndex = 0;
      z_obsLinearInfo.pz_obsIndex[u_i].u_signalIndex = 0;
    }

    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      QR_calTargetFreqRcvDriftClock(u_i, e_selectedFreq, pz_satSigMeasCollect->z_positionFix.d_xyz, pz_satSigMeasCollect, pz_SsrLocBlk, pz_QRcheckInfo, pz_PPPfilterInfo, pz_EKFstateRepPool, &(z_obsLinearInfo.pf_rcvClockInit[u_i]));
    }
    u_PRtotalNum = QR_obsLinear(e_selectedFreq, u_PRtotalNum, pz_satSigMeasCollect, pz_SsrLocBlk, pz_QRcheckInfo, pz_PPPfilterInfo, pz_EKFstateRepPool, &z_obsLinearInfo);
    if (u_PRtotalNum <= (u_paraNum + 2))
    {
      matrix_free(&z_obsLinearInfo.pz_H);
      matrix_free(&z_obsLinearInfo.pz_P);
      matrix_free(&z_obsLinearInfo.pz_L);
      matrix_free(&z_obsLinearInfo.pz_deltaX);
      OS_FREE(z_obsLinearInfo.pz_obsIndex);
      break;
    }
    q_grossIndex = -1;
    u_checkStatus = cmn_qrParityCheckAcoordingToPostRes(3,z_obsLinearInfo.pz_H, z_obsLinearInfo.pz_P, z_obsLinearInfo.pz_L, &q_grossIndex, z_obsLinearInfo.pz_deltaX);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_QRcheckInfo->pd_QRcheckSiteCoor[u_i] = pz_satSigMeasCollect->z_positionFix.d_xyz[u_i] + MAT(z_obsLinearInfo.pz_deltaX, u_i, 0);
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
    if (QR_CHECK_UNKNOW == u_checkStatus || QR_CHECK_OBS_Insufficient == u_checkStatus || QR_CHECK_IDETIFY_OBS_FAIL == u_checkStatus)
    {
      OS_FREE(z_obsLinearInfo.pz_obsIndex);
      break;
    }
    if (QR_CHECK_IDETIFY_OBS_SUCC == u_checkStatus)
    {
      pz_obsIndex = &(z_obsLinearInfo.pz_obsIndex[q_grossIndex]);
      pz_satSigMeasCollect->pz_satMeas[pz_obsIndex->q_satIndex]->pz_signalQuality[pz_obsIndex->u_signalIndex].u_PRquality |= PR_QUALITY_GROSS_BY_QR_PARITY;          
      u_PRtotalNum = QR_countTargetFreqObsNum(e_selectedFreq, pz_PPPfilterInfo->z_opt.d_elmin,pz_satSigMeasCollect, pz_SsrLocBlk, pz_QRcheckInfo, pu_PRnum);
      u_paraNum = QR_calParaNum(e_selectedFreq,pu_PRnum, &z_obsLinearInfo);

      if (log_GetLogLevel() >= LOG_LEVEL_I)
      {
        satidx_SatString(pz_obsIndex->q_satIndex, c_sat);
        if(u_nb<=0) u_nb+=snprintf(&c_buff[u_nb], 13, "QR_check pr:");
        u_nb += snprintf(&c_buff[u_nb], 10, " %3s f=%01d;", c_sat, pz_obsIndex->u_signalIndex);
        if (u_nb > 220)
        {
          LOGI(TAG_PPP, "%s\n", c_buff);
          strcpy(c_buff, ""); u_nb = 0;
        }
      }
      
    }
    OS_FREE(z_obsLinearInfo.pz_obsIndex);
    ++u_iterNum;
  }
  if (u_nb > 0)
  {
    LOGI(TAG_PPP, "%s\n", c_buff);
    strcpy(c_buff, "");
  }

  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    pz_QRcheckInfo->pu_PRnum[u_i] = pu_PRnum[u_i];
    if (pu_PRnum[u_i] > u_maxPRsysNum)
    {
      u_maxPRsysNum = pu_PRnum[u_i];
    }
  }
  if (u_PRtotalNum < (u_paraNum + 2) || u_maxPRsysNum < (3 + 1))
  {
    q_success = FALSE;
  }
  pz_QRcheckInfo->q_QRsuccess = q_success;
  pz_QRcheckInfo->u_PRtotalNum = u_PRtotalNum;
  return;
}
  
/**
 * @brief the interface of QR parity check algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[out]     pz_PPPfilterInfo the filter for the PPP algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @return         0 represent successs and other failed
 */
int32_t ppp_QRcheck(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  int32_t q_status = 0;
  uint8_t u_iFreq = 0;
  uint8_t u_iSys = 0;
  uint8_t u_maxPseudoRangeNum = 0;
  //uint8_t u_optSolutionIndex = 0;
  BOOL pq_freqSolValid[C_GNSS_FREQ_TYPE_MAX] = { FALSE };
  ppp_QRcheckInfo_t* pz_QRcheckInfo = (ppp_QRcheckInfo_t*)OS_MALLOC_FAST(sizeof(ppp_QRcheckInfo_t) * C_GNSS_FREQ_TYPE_MAX);
  if (NULL == pz_QRcheckInfo)
  {
    return 1;
  }

  for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
  {
    ppp_checkPseudoRangeUsingQR(u_iFreq, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, &pz_QRcheckInfo[u_iFreq]);
  }
  if (FALSE == qr_determineSolutionValid(pz_QRcheckInfo, pq_freqSolValid))
  {
    LOGI(TAG_PPP, "QR check failure\n");
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
      if ((pz_QRcheckInfo[u_iFreq].u_PRtotalNum) > u_maxPseudoRangeNum)
      {
        u_maxPseudoRangeNum = (pz_QRcheckInfo[u_iFreq].u_PRtotalNum);
        //u_optSolutionIndex = u_iFreq;
      }
    }
    for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
    {
      if (TRUE == pq_freqSolValid[u_iFreq])
      {
        continue;
      }

      for (u_iSys = C_GNSS_GPS; u_iSys < C_GNSS_MAX; ++u_iSys)
      {
        QR_markPseudoRangeGross(u_iSys, u_iFreq, pz_SsrLocBlk, &pz_QRcheckInfo[u_iFreq], pz_PPPfilterInfo, pz_EKFstateRepPool, pz_satSigMeasCollect);
      }
    }
  }

  OS_FREE(pz_QRcheckInfo);
  return q_status;
}

