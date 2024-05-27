/**@file        ppp_amb_fix.c
 * @brief       integer ambiguity resolution
 * @details
 * @author      liuguo
 * @date        2022/05/12
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/12  <td>0.1      <td>liuguo      <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "ppp_amb_fix.h"
#include "ppp_filter_sol.h"
#include "gnss_common.h"
#include "seq_kalman.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "cmn_utils.h"
#include "cmn_CSmask_combine.h"

 /* constants/macros ----------------------------------------------------------*/

#define LOG_PI      (1.14472988584940017) /* log(pi) */
#define MIN_SUC_RATE   (0.95)
#define MIN_SAT_AR (5)
#define PPPAR_MAX_LOS_QI   (0.2)         /* max age of ssr */ 

typedef uint8_t PPP_AmbFixDeleteConstellationType;    /* define the needed to delete constellation in PPP-RTK ambiguity algorithm */
#define PPP_AMB_FIX_DELETE_NONE_SYS   ((uint8_t)0x00) /* delete NONE constellation in PPP-RTK ambiguity algorithm */
#define PPP_AMB_FIX_DELETE_GPS_SYS    ((uint8_t)0x01) /* delete GPS constellation in PPP-RTK ambiguity algorithm */
#define PPP_AMB_FIX_DELETE_BDS2_SYS   ((uint8_t)0x02) /* delete BDS2 constellation in PPP-RTK ambiguity algorithm  */
#define PPP_AMB_FIX_DELETE_BDS3_SYS   ((uint8_t)0x04) /* delete BDS3 constellation in PPP-RTK ambiguity algorithm  */
#define PPP_AMB_FIX_DELETE_GLO_SYS    ((uint8_t)0x08) /* delete GLONASS constellation in PPP-RTK ambiguity algorithm  */
#define PPP_AMB_FIX_DELETE_GAL_SYS    ((uint8_t)0x10) /* delete Gsalileo constellation in PPP-RTK ambiguity algorithm  */
#define PPP_AMB_FIX_DELETE_QZS_SYS    ((uint8_t)0x20) /* delete QZSS constellation in PPP-RTK ambiguity algorithm */
#define PPP_AMB_FIX_DELETE_NAVIC_SYS  ((uint8_t)0x40) /* delete NAVIC constellation in PPP-RTK ambiguity algorithm */

static uint8_t ambFixTypeIdx(uint8_t a)
{
  uint8_t idx = 0;

  while (a)
  {
    a = a / 2;
    if (a == 0) break;
    idx++;
  }
  return idx;
}
static float minLockDecide(const uint8_t* rtk, uint8_t sys, uint8_t flag, float ele)
{
  float minlock = 30.0;

  if (NULL != rtk) /* first fix epoch */
  {
    if ((*rtk) > 0) /* ppprtk */
    {
      minlock = 3.0;
    }
    else minlock = 900;  /* pppar */
    return minlock;
  }

  /* every satellite */
  if (flag == 0)
  {
    minlock = 600.0;
    if (ele < 20.0 * DEG2RAD) minlock = 1800.0;
    else if (ele < 30.0 * DEG2RAD) minlock = 1200.0;
  }
  else
  {
    minlock = 0.0;
    if (ele < 20.0 * DEG2RAD) minlock = 5.0;
  }

  return minlock;
}

/**
 * @brief get pdop of ambfix satellites
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in/out]    pz_fixedAmbPool
 * @return status  none
 */
static void getFixDops(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_ns = 0;
  float pf_dop[4] = { 0.0 };
  float* pf_azel = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  pz_fixedAmbPool->f_pdop = 999.0f;
  pf_azel = (float*)OS_MALLOC_FAST(sizeof(float) * MAX_GNSS_ACTIVE_SAT_NUMBER * 2);
  if (NULL == pf_azel)
  {
    return;
  }
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL == pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_i]
      || NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_i])
    {
      continue;
    }
    if (GNSS_NONE_AMB_FIXED==pz_fixedAmbPool->pz_fixedAmbSet[u_i]->u_ambFixType)
    {
      continue;
    }
    if (u_ns >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_i];
    pf_azel[0 + u_ns * 2] = pz_satMeas->z_satPosVelClk.f_azimuth;
    pf_azel[1 + u_ns * 2] = pz_satMeas->z_satPosVelClk.f_elevation;
    u_ns++;
  }
  if (u_ns >= 4)
  {
    gnss_dops(u_ns, pf_azel, pf_dop);
    pz_fixedAmbPool->f_pdop = pf_dop[1]; // PDOP
  }
  LOGI(TAG_PPP, "PDOP of amb-fix sat is %.1f\n", pz_fixedAmbPool->f_pdop);

  OS_FREE(pf_azel);
}

/**
 * @brief keep previous refsat
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_satMeas_pre
 * @param[in]    u_sys
 * @param[in]    u_pre_refsat
 * @param[in]    z_freqType
 * @return status -   0: keep, other: do not keep
 */
static uint8_t keepPreRefsat(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  const gnss_SatelliteMeas_t* pz_satMeas_pre, uint8_t u_sys, uint8_t u_pre_refsat,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  uint8_t u_sat_idx = 0;
  float f_minlock = 0.0;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const GpsTime_t* pz_curtime = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const ppp_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal2 = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  gnss_fixedSignalAmb_t* pz_fixedAmb = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  /* get information of previous refsat */
  u_sat_idx = gnss_cvt_Svid2SvIndex(u_pre_refsat, u_sys);
  pz_curtime = &pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor;
  pz_corrStat = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_sat_idx];
  
  if (NULL != pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx])
  {
    pz_fixedAmb = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]->pz_fixedAmbSet[u_sat_idx];
  }
  if (NULL == pz_fixedAmb|| NULL == pz_corrStat||
    NULL == pz_satMeas_pre->pz_signalMeas[z_freqType1]
    || NULL == pz_satMeas_pre->pz_signalMeas[z_freqType2]
    || NULL == pz_corrStat->pf_phaseResidual[z_freqType1]
    || NULL == pz_corrStat->pf_phaseResidual[z_freqType2])
  {
    u_status = 1;
    return u_status;
  }
  if (NULL == pz_corrStat->z_measUpdateFlag[z_freqType1] || NULL == pz_corrStat->z_measUpdateFlag[z_freqType2]
    || !pz_corrStat->z_measUpdateFlag[z_freqType1]->b_cpValid || !pz_corrStat->z_measUpdateFlag[z_freqType2]->b_cpValid)
  {
    u_status = 2;
    return u_status;
  }

  w_x_id[0] =  0;
  w_x_id[1] = u_sat_idx;
  w_x_id[2] = z_filterType1;
  pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
  pz_signal1 = getSSR_Bias(pz_satMeas_pre->u_constellation, pz_satMeas_pre->u_svid, pz_satMeas_pre->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
  if (z_filterType1 == z_filterType2)
  {
    pz_satPool2 = pz_satPool1;
    pz_signal2 = pz_signal1;
  }
  else
  {
    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    pz_signal2 = getSSR_Bias(pz_satMeas_pre->u_constellation, pz_satMeas_pre->u_svid, pz_satMeas_pre->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
  }
  
  if ((NULL == pz_satPool1) ||(NULL == pz_satPool2))
  {
    u_status = 3;
    return u_status;
  }
  
  if (NULL == pz_signal1|| NULL == pz_signal2||
    !(pz_signal1->u_biasMask&GNSS_SSR_SAT_BIAS_PHASE_CORR)
    || !(pz_signal2->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
  {
    u_status = 4;
    return u_status;
  }
  pz_satLos = getSSR_constLos(pz_satMeas_pre->u_constellation, pz_satMeas_pre->u_svid, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
  if ((NULL == pz_satLos) || pz_satLos->z_orbClk.f_qi * 0.001 > PPPAR_MAX_LOS_QI
    || !(GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID & pz_satLos->u_windUpMask))
  {
    u_status = 5;
    return u_status;
  }
  if (pz_satMeas_pre->pz_signalMeas[z_freqType1]->u_LLI > 0 || pz_satMeas_pre->pz_signalMeas[z_freqType2]->u_LLI > 0
    ||pz_satMeas_pre->pz_signalMeas[z_freqType1]->f_cn0 < 30.0 || pz_satMeas_pre->pz_signalMeas[z_freqType2]->f_cn0 < 30.0)
  {
    u_status = 6;
    return u_status;
  }
  if (fabs(*pz_corrStat->pf_phaseResidual[z_freqType1]) > 0.05f || fabs(*pz_corrStat->pf_phaseResidual[z_freqType2]) > 0.05f
    || fabs(pz_corrStat->f_ionResidual) > 0.15f)
  {
    u_status = 7;
    return u_status;
  }

  /* decision of keep previous refsat */
  if (!(pz_corrStat->u_IonoCorrStatus & PPP_SSR_CORRECTED))
  {
    f_minlock = 900.0;
  }
  else f_minlock = 5.0;
  if (NULL != pz_fixedAmb && pz_fixedAmb->u_continue_fix[z_freqType2] > 50 && pz_satMeas_pre->z_satPosVelClk.f_elevation * RAD2DEG > 30.0
    && tm_GpsTimeDiff(pz_curtime, &pz_satPool1->z_endTime) < 1e-4 && tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime) > f_minlock
    && tm_GpsTimeDiff(pz_curtime, &pz_satPool2->z_endTime) < 1e-4 && tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime) > f_minlock)
  {
    u_status = 0;
    return u_status;
  }
  else
  {
    u_status = 8;
  }

  return u_status;
}

/**
 * @brief refsat select
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @param[out]   pz_curfixedAmbPool
 * @return status -   1: success, 0: fail
 */
static uint8_t refsatSelect(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2
  , gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_isys = 0;
  uint8_t u_sys = 0;
  uint8_t u_sat_idx = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  uint8_t u_pre_refsat = 0;
  uint8_t u_refsat[C_GNSS_MAX] = { 0 };
  char char_refsat[4] = "";
  char char_fin_refsat[4] = "";
  float f_minlock = 0.0;
  float f_mindt = 0.0;
  float f_dt1 = 0.0;
  float f_dt2 = 0.0;
  double d_max_weight = 0.0;
  double* pd_sat_weight = NULL;
  gnss_fixedAmbType u_fix_type;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const GpsTime_t* curtime = NULL;
  const gnss_fixedSignalAmbPool_t* pz_prefixedAmbPool = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas_prefsat = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const ppp_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal2 = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);
  curtime = &pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor;
  pz_prefixedAmbPool = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx];
  pd_sat_weight = OS_MALLOC_FAST(ALL_GNSS_SYS_SV_NUMBER * sizeof(double));
  if (NULL == pd_sat_weight)
  {
    LOGI(TAG_PPP, "refsatSelect MALLOC fai\n");
    return 0;
  }
  memset(pd_sat_weight, 0, ALL_GNSS_SYS_SV_NUMBER * sizeof(double));
  if (z_freqType1 == z_freqType2)
  {
    u_fix_type = GNSS_NL_AMB_FIXED;
  }
  else u_fix_type = GNSS_WL_AMB_FIXED;

  /* get weight of satellites from check loop all satellites */
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    pd_sat_weight[u_i] = 0.0;
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_i];
    pz_corrStat = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_i];
    if (NULL == pz_satMeas || NULL == pz_corrStat)
    {
      continue;
    }
    if (NULL == pz_corrStat->z_measUpdateFlag[z_freqType1] || NULL == pz_corrStat->z_measUpdateFlag[z_freqType2]
      || NULL == pz_satMeas->pz_signalMeas[z_freqType1] || NULL == pz_satMeas->pz_signalMeas[z_freqType2]
      || NULL == pz_corrStat->pf_phaseResidual[z_freqType1] || NULL == pz_corrStat->pf_phaseResidual[z_freqType2])
    {
      continue;
    }
    if (!pz_corrStat->z_measUpdateFlag[z_freqType1]->b_cpValid || !pz_corrStat->z_measUpdateFlag[z_freqType2]->b_cpValid
      || pz_satMeas->pz_signalMeas[z_freqType1]->u_LLI >0 || pz_satMeas->pz_signalMeas[z_freqType2]->u_LLI >0)
    {
      continue;
    }
    if (fabs(*pz_corrStat->pf_phaseResidual[z_freqType1]) > 0.05f || fabs(*pz_corrStat->pf_phaseResidual[z_freqType2]) > 0.05f
      || fabs(pz_corrStat->f_ionResidual) > 0.15f)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }

    /* get status from EKF */
    w_x_id[0] = 0;
    w_x_id[1] = u_i;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (z_filterType1 == z_filterType2)
    {
      pz_satPool2 = pz_satPool1;
      pz_signal2 = pz_signal1;
    }
    else
    {
      w_x_id[0] = 0;
      w_x_id[1] = u_i;
      w_x_id[2] = z_filterType2;
      pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
      pz_signal2 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    }
    
    if (NULL == pz_satPool1 || NULL == pz_satPool2)
    {
      continue;
    }
    if (NULL == pz_signal1 || NULL == pz_signal2 ||
      !(pz_signal1->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR)
      || !(pz_signal2->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
    {
      continue;
    }
    if (fabs(tm_GpsTimeDiff(curtime, &pz_satPool1->z_endTime)) > 1e-4
         || fabs(tm_GpsTimeDiff(curtime, &pz_satPool2->z_endTime)) > 1e-4) /* unhealth,current epoch */
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if ((NULL == pz_satLos) || pz_satLos->z_orbClk.f_qi * 0.001 > PPPAR_MAX_LOS_QI
      || !(GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID & pz_satLos->u_windUpMask))
    {
      continue;
    }

    /* check code bias and phase bias */
    f_minlock = minLockDecide(NULL, pz_satMeas->u_constellation, pz_corrStat->u_IonoCorrStatus & PPP_SSR_CORRECTED, pz_satMeas->z_satPosVelClk.f_elevation);
    if (pz_satMeas->z_satPosVelClk.f_elevation < 20.0 * DEG2RAD|| pz_satMeas->pz_signalMeas[z_freqType1]->f_cn0 < 30.0
      || pz_satMeas->pz_signalMeas[z_freqType2]->f_cn0 < 30.0)
    {
      continue;
    }
    f_dt1 = (float)tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime);
    f_dt2 = (float)tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime);
    f_mindt = f_dt1 < f_dt2 ? f_dt1 : f_dt2;
    f_minlock = f_minlock < 0.5f ? 0.5f : f_minlock;
    if (f_mindt < f_minlock)
    {
      continue;
    }
    if (f_mindt < 180.0) f_mindt = 180.0f;
    f_mindt = f_mindt * 0.1f;
    u_isys = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    /* computation of weight */
    pd_sat_weight[u_i] = pow(10, (pz_satMeas->pz_signalMeas[z_freqType1]->f_cn0*1.0 + pz_satMeas->pz_signalMeas[z_freqType2]->f_cn0) *0.05)
      * pow(10, (pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG)*0.06) * f_mindt;
    if (pz_corrStat->u_IonoCorrStatus & PPP_SSR_CORRECTED)
    {
      pd_sat_weight[u_i] *= 20 * 60.0;
    }
    if (C_SAT_TYPE_IGSO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      pd_sat_weight[u_i] = pd_sat_weight[u_i] / pow(10, 3);
    }
    /* switch ref-sat, if fix fail in previous epoch */
    if (NULL != pz_prefixedAmbPool && !(pz_prefixedAmbPool->u_fixStatus & u_fix_type) &&
      u_i == gnss_cvt_Svid2SvIndex(pz_prefixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2], pz_satMeas->u_constellation))
    {
      pd_sat_weight[u_i] = pd_sat_weight[u_i] / pow(10, 6);
    }
    else if (NULL != pz_prefixedAmbPool && (pz_prefixedAmbPool->u_fixStatus & u_fix_type) &&(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG>30.0)&&
      u_i == gnss_cvt_Svid2SvIndex(pz_prefixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2], pz_satMeas->u_constellation))
    {
      pd_sat_weight[u_i] = pd_sat_weight[u_i] * pow(10, 2);
    }
  }

  /* select refsat by weight */
  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    d_max_weight = 0.0;
    u_pre_refsat = 0;
    pz_satMeas_prefsat = NULL;
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    if (NULL != pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx])
    {
      u_pre_refsat = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    }
    
    for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
    {
      pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_i];
      if (NULL == pz_satMeas)
      {
        continue;
      }
      u_sys = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      if (u_sys != u_isys || pd_sat_weight[u_i] <= 0.0)
      {
        continue;
      }
      u_sat_idx = gnss_cvt_Svid2SvIndex(u_pre_refsat, u_sys);
      if (u_sat_idx < ALL_GNSS_SYS_SV_NUMBER)
      {
        pz_satMeas_prefsat = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
      }
      else
      {
        pz_satMeas_prefsat = NULL;
      }

      if (pd_sat_weight[u_i] > d_max_weight)
      {
        d_max_weight = pd_sat_weight[u_i]; 
        u_refsat[u_isys] = pz_satMeas->u_svid;
      }
    }
    if (u_refsat[u_isys] <= 0)
    {
      continue;
    }
    /* keep previous reference satellites */
    if (u_refsat[u_isys] > 0 && u_pre_refsat > 0 && u_refsat[u_isys] != u_pre_refsat && NULL != pz_satMeas_prefsat
      && (!keepPreRefsat(pz_pppAmbFixInputInfo, pz_satMeas_prefsat, u_isys, u_pre_refsat, z_freqType1, z_freqType2)))
    {
      LOGD(TAG_PPP, "Pre-refsat keep\n");
      pz_curfixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2] = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    }
    else
    {
      pz_curfixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2] = u_refsat[u_isys];
    }

    svid_SatString(u_refsat[u_isys], u_isys, char_refsat);
    svid_SatString(pz_curfixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2], u_isys, char_fin_refsat);
    LOGI(TAG_PPP, "Final refsat=%3s, selected sat=%3s, freq=%d\n", char_fin_refsat, char_refsat, z_freqType2);
  }

  OS_FREE(pd_sat_weight);
  pd_sat_weight = NULL;

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (pz_curfixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2] > 0)
    {
      return 1;
    }
  }

  return 0;
}

/**
 * @brief variance of satellite ephemeris
 * @param[in]  f_ure
 * @return     variance
 */
static double varianceEphSSR(float f_ure)
{
  double var = 0.0;
  const double DEFURASSR = 5.0E-3;

  if (f_ure < 0.04)
  {
    var = SQR(DEFURASSR * 0.5);
  }
  else if (f_ure < 0.08)
  {
    var = SQR(DEFURASSR * 1.0);
  }
  else if (f_ure < 0.12)
  {
    var = SQR(DEFURASSR * 2.0);
  }
  else if (f_ure < 0.16)
  {
    var = SQR(DEFURASSR * 4.0);
  }
  else if (f_ure < 0.20)
  {
    var = SQR(DEFURASSR * 6.0);
  }
  else if (f_ure < 0.30)
  {
    var = SQR((double)f_ure);
  }
  else
  {
    var = SQR(9999.99);
  }

  return var;
}
/**
 * @brief residual of AMB
 * @param[in]    u_iter
 * @param[in]    NX
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @param[in]    pd_X
 * @param[out]    pd_V
 * @param[out]    pd_H
 * @param[out]    pd_R
 * @return number of residual
 */
static uint8_t resAmb(uint8_t u_iter, const uint8_t NX, gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, matrix_t* pd_X, matrix_t* pd_V, matrix_t* pd_H, matrix_t* pd_R)
{
  uint8_t u_sig_num = 0;
  uint32_t q_obsNum = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_grossflag = 0;
  uint8_t u_maxsat = 0;
  uint8_t u_maxfre = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint8_t* u_mask = NULL;
  char char_sat[4] = "";
  gnss_ConstellationType z_constell = C_GNSS_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_fixedSignalAmb_t* pz_fixedAmbSat = NULL;
  ppp_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t w_ambIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  double vart = 0.0;
  double vari = 0.0;
  double var_rs = 0.0;
  double dt = 0.0;
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double d_dist = 0.0;
  double pd_unitVector[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  double d_wave = 0.0;
  double d_beta = 0.0;
  double d_windup = 0.0;
  double d_fact = 1.0;
  double ssr_sat_orclk = 0.0;
  double d_maxP = 0.0;
  double d_vp = 0.0;

  u_mask = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (NX - 3));
  if (0x1 == (pz_pppAmbFixInputInfo->pz_pppFilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }

  //get the  site state index in the filter
  if (NULL == u_mask || !getSiteRelativeParaIndex(pz_pppAmbFixInputInfo->pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    OS_FREE(u_mask);
    return 0;
  }
  if (FALSE == (pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  /* site coordinate */
  if ((fabs(pd_X->data[u_i]) + fabs(pd_X->data[u_i]) + fabs(pd_X->data[u_i])) <= 0.0)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_X->data[u_i] = pz_fixedAmbPool->pd_x_fix[pw_PVAindex[u_i]];
    }
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X->data[u_i];
  }

  /* begain filter*/
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[q_satIndex];
    pz_fixedAmbSat = pz_fixedAmbPool->pz_fixedAmbSet[q_satIndex];
    pz_corrStat = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_satMeas || NULL == pz_fixedAmbSat || NULL == pz_corrStat)
    {
      continue;
    }
    if (GNSS_NONE_AMB_FIXED == pz_fixedAmbSat->u_ambFixType)
    {
      continue;
    }
    u_sig_num += PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR);
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }

    if (pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID)
    {
      d_windup = pz_satLos->d_windUp;
    }
    else
    {
      continue;
    }
    ssr_sat_orclk = pz_satLos->z_orbClk.q_corr * 0.001;
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);

    vart = 0.0;
    vari = 0.0;
    var_rs = varianceEphSSR(pz_satLos->z_orbClk.f_qi * 0.001f);
    uint8_t sig_num = PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      if (pz_corrStat->u_lsqFlag[u_signalIndex] || 1 == cmn_combineCycleSlipMask(pz_pppAmbFixInputInfo->pz_pppFilterInfo->u_tdcpMethodValid, sig_num,
        pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, pz_pppAmbFixInputInfo->pz_pppFilterInfo->d_deltaTimeDopplerDetect,u_isCurrentCloseSky, pz_sigMeas->u_slipMethodValid, NULL))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType == C_GNSS_FREQ_TYPE_MAX || C_GNSS_SIG_MAX == pz_fixedAmbSat->u_freqType[gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal)]
        || pz_fixedAmbSat->u_freqType[gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal)] != pz_sigMeas->u_signal)
      {
        continue;
      }

      z_constell = pz_sigMeas->u_constellation;
      if (w_ZTDindex < 0)
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + pz_sigMeas->u_constellation];
      w_rcvClkFilterIndex = pw_rcvClkIndex[pz_sigMeas->u_constellation];
      if (INVALID_INDEX == w_rcvDcbFilterIndex) // no clk
      {
        continue;
      }
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
        continue;
      }
      d_wave = wavelength(pz_sigMeas->u_signal);
      if (d_wave == 1.0)
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = GNSS_FILTER_STATE_IONO;
      getTargetParaIndex(pz_pppAmbFixInputInfo->pz_EKFstateRepPool, w_id, &w_ionoIndex);
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_pppAmbFixInputInfo->pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (w_ionoIndex < 0 || w_ambIndex < 0)
      {
        continue;
      }
      d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);

      /* H */
      for (u_i = 0; u_i < NX; ++u_i)
      {
        pd_H->data[q_obsNum * NX + u_i] = 0.0;
      }
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_H->data[q_obsNum * NX + u_i] = -pd_unitVector[u_i];// pos
      }
      pd_H->data[q_obsNum * NX + 3 + z_constell] = 1.0; //clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        pd_H->data[q_obsNum * NX + 3 + z_freqType * (C_GNSS_MAX + 1) + z_constell] = 1.0; // dcb
      }

      /* OMC */
      pd_V->data[q_obsNum] = (pz_sigMeas->d_carrierPhase - d_windup) * d_wave - (d_dist - pd_satPosClk[3]);
      if (fabs(pd_X->data[3 + z_constell]) <= 0.0)
      {
        pd_X->data[3 + z_constell] = pz_fixedAmbPool->pd_x_fix[w_rcvClkFilterIndex];
      }
      pd_V->data[q_obsNum] -= pd_X->data[3 + z_constell]; // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        if (fabs(pd_X->data[3 + z_freqType * (C_GNSS_MAX + 1) + z_constell]) <= 0.0)
        {
          pd_X->data[3 + z_freqType * (C_GNSS_MAX + 1) + z_constell] = pz_fixedAmbPool->pd_x_fix[w_rcvDcbFilterIndex];
        }
        pd_V->data[q_obsNum] -= pd_X->data[3 + z_freqType * (C_GNSS_MAX + 1) + z_constell]; // dcb
      }

      pd_V->data[q_obsNum] -= ((pz_pppAmbFixInputInfo->pz_satSignalPool->d_zhd_emp * pz_satMeas->d_dryMap) + (pz_fixedAmbPool->pd_x_fix[w_ZTDindex] * pz_satMeas->d_wetMap));
      pd_V->data[q_obsNum] -= d_wave * pz_fixedAmbPool->pd_x_fix[w_ambIndex]; // amb
      pd_V->data[q_obsNum] -= (-d_beta * pz_fixedAmbPool->pd_x_fix[w_ionoIndex]); // ion
      pd_V->data[q_obsNum] -= ssr_sat_orclk;

      d_fact = 1.0;
      if (C_GNSS_BDS3 == pz_sigMeas->u_constellation || C_GNSS_BDS2 == pz_sigMeas->u_constellation)
      {
        d_fact = 2.0;
      }
      pd_R->data[q_obsNum] = SQR(d_fact * 0.003) * pow(10.0, MAX_A_B((40.0 - pz_sigMeas->f_cn0) / 10.0, 0.0));;
      pd_R->data[q_obsNum] += vart + vari + var_rs;

      d_vp = fabs(pd_V->data[q_obsNum]) / sqrt(pd_R->data[q_obsNum]);

      satidx_SatString(q_satIndex, char_sat);
      LOGI(TAG_PPP, "iter=%02d, sat=%s, f=%d, V=%8.3lf  R=%7.3lf V/R=%7.3lf\n", u_iter, char_sat,
        z_freqType + 1, pd_V->data[q_obsNum], sqrt(pd_R->data[q_obsNum]), d_vp);

      if (d_vp > d_maxP)
      {
        d_maxP = d_vp;
        u_maxsat = q_satIndex;
        u_maxfre = u_signalIndex;
      }
      if (!u_iter && (fabs(pd_V->data[q_obsNum]) > 0.1 || fabs(pd_V->data[q_obsNum] / sqrt(pd_R->data[q_obsNum])) > 3.0))
      {
        LOGI(TAG_PPP, "LSQ delete, sat=%s, f=%d, V/R=%8.3lf\n", char_sat, z_freqType + 1, d_vp);
        pz_corrStat->u_lsqFlag[u_signalIndex] = 1;
        u_grossflag = 1;
      }
      u_mask[z_freqType * (C_GNSS_MAX + 1) + z_constell] = 1;
      ++q_obsNum;

    }
  }
  if (!u_iter && !u_grossflag)
  {
    satidx_SatString(u_maxsat, char_sat);
    LOGI(TAG_PPP, "LSQ delete, sat=%s, f=%d, V/R=%8.3lf\n", char_sat, u_maxfre + 1, d_maxP);
    pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_maxsat]->u_lsqFlag[u_maxfre] = 1;
  }
  /* constraint to avoid rank-deficient */
  for (u_i = 0; u_i < (NX - 3) && q_obsNum>0; u_i++)
  {
    if (u_mask[u_i] > 0.0) continue;
    pd_V->data[q_obsNum] = 0.0;
    for (u_j = 0; u_j < NX; u_j++) pd_H->data[q_obsNum * NX + u_j] = u_j == u_i + 3 ? 1.0 : 0.0;
    pd_R->data[q_obsNum] = 0.01;
    ++q_obsNum;
  }
  OS_FREE(u_mask);
  return q_obsNum;
}
/**
 * @brief AMB checking by LSQ
 * @param[in/out]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @return none
 */
static void PPP_AmbcheckLSQ(gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  const uint8_t NX = 3 + (C_GNSS_MAX + 1) * MAX_GNSS_SIGNAL_FREQ;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_nv = 0;
  double d_sig = 0.0;
  double d_vv = 0.0;
  double d_dxyz[3] = { 0.0 };
  const double chisqr[100] = {      /* chi-sqr(n) (alpha=0.001) */
    10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
    31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
    46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
    61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
    74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
    88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
    101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
    113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
    126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
    138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149
  };
  matrix_t* pd_X = NULL;
  matrix_t* pd_V = NULL;
  matrix_t* pd_H = NULL;
  matrix_t* pd_R = NULL;
  matrix_t* pd_dX = NULL;
  matrix_t* pd_Q = NULL;

  if (NULL == pz_fixedAmbPool)
  {
    return;
  }

  pd_X = matrix_new(NX, 1);
  pd_Q = matrix_new(NX, NX);
  pd_dX = matrix_new(NX, 1);
  pd_V = matrix_new(MAX_GNSS_ACTIVE_SAT_NUMBER * MAX_GNSS_SIGNAL_FREQ, 1);
  pd_H = matrix_new(MAX_GNSS_ACTIVE_SAT_NUMBER * MAX_GNSS_SIGNAL_FREQ, NX);
  pd_R = matrix_new(MAX_GNSS_ACTIVE_SAT_NUMBER * MAX_GNSS_SIGNAL_FREQ, 1);

  if (NULL == pd_X || NULL == pd_V || NULL == pd_H || NULL == pd_R || NULL == pd_Q || NULL == pd_dX)
  {
    matrix_free(&pd_X); matrix_free(&pd_V); matrix_free(&pd_H); matrix_free(&pd_R); matrix_free(&pd_Q); matrix_free(&pd_dX);
    return;
  }
  for (u_i = 1; u_i <= 10; u_i++)
  {
    u_nv = resAmb(u_i, NX, pz_pppAmbFixInputInfo, pz_fixedAmbPool, pd_X, pd_V, pd_H, pd_R);

    if (u_nv < NX)
    {
      LOGI(TAG_PPP, "lack of valid sats ns=%d\n", u_nv);
      break;
    }
    pd_V->row = u_nv;
    pd_H->row = u_nv;
    pd_R->row = u_nv;
    /* weighted by Std */
    for (u_j = 0; u_j < u_nv; u_j++)
    {
      d_sig = sqrt(pd_R->data[u_j]);
      pd_V->data[u_j] /= d_sig;
      for (u_k = 0; u_k < NX; u_k++) pd_H->data[u_k + u_j * NX] /= d_sig;
    }
    /* least square estimation */
    if (!lsqMat(pd_H, pd_V, pd_dX, pd_Q))
    {
      LOGI(TAG_PPP, "lsq error\n", u_nv);
      break;
    }
    for (u_j = 0; u_j < NX; u_j++)
    {
      pd_X->data[u_j] += pd_dX->data[u_j];
    }
    d_vv = gnss_Norm(pd_dX->data, 3);
    if (d_vv < 1E-3)
    {
      for (u_j = 0; u_j < 3; u_j++)
      {
        d_dxyz[u_j] = pz_fixedAmbPool->pd_x_fix[u_j] - pd_X->data[u_j];
      }
      LOGI(TAG_PPP, "PPP_AmbcheckLSQ dx=%10.3lf\n", gnss_Norm(d_dxyz, 3));
      /* validate solution */
      d_vv = gnss_Dot(pd_V->data, pd_V->data, u_nv);
      if (u_nv > NX && d_vv > chisqr[u_nv - NX - 1])
      {
        LOGI(TAG_PPP, "chi-square error nv=%d vv=%.1f cs=%.1f\n", u_nv, d_vv, chisqr[u_nv - NX - 1]);
        resAmb(0, NX, pz_pppAmbFixInputInfo, pz_fixedAmbPool, pd_X, pd_V, pd_H, pd_R);
        continue;
      }
      break;
    }
  }
  matrix_free(&pd_X); matrix_free(&pd_V); matrix_free(&pd_H); matrix_free(&pd_R); matrix_free(&pd_Q); matrix_free(&pd_dX);

  return;
}

/**
 * @brief get partial AR quality list by covariance
 * @param[in]    u_n
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pu_satlist
 * @param[in]    pd_Q
 * @param[out]   pd_Qii
 * @return status - 0: success, other:fail
 */
static uint8_t parCovariance(uint8_t u_n, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const uint8_t* pu_satlist, const double* pd_Q, double* pd_Qii)
{
  uint8_t u_i = 0;
  int16_t w_sat_idx = 0;
  float f_fact = 1.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_corr = NULL;

  for (u_i = 0; u_i < u_n; u_i++)
  {
    w_sat_idx = pu_satlist[u_i] - 1;
    if (w_sat_idx < 0|| w_sat_idx >= ALL_GNSS_SYS_SV_NUMBER)
    {
      return 1;
    }
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[w_sat_idx];
    if (NULL == pz_satMeas)
    {
      return 2;
    }
    f_fact = 1.0;
    pz_corr = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[w_sat_idx];
    if (NULL == pz_corr || !(pz_corr->u_IonoCorrStatus & PPP_SSR_CORRECTED))
    {
      f_fact *= 100.0;
    }
    pd_Qii[u_i] = f_fact * pd_Q[u_i * (u_n + 1)];
  }
  return 0;
}

/**
 * @brief get partial AR quality list by snr
 * @param[in]    u_n
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pu_satlist
 * @param[out]   pd_Qii
 * @return status - 0: success, other:fail
 */
static uint8_t parSnr(uint8_t u_n, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const uint8_t* pu_satlist, double* pd_Qii)
{
    uint8_t u_i = 0;
    int16_t w_sat_idx = 0;
    float f_fact = 1.0;
    const gnss_SatelliteMeas_t* pz_satMeas = NULL;
    const ppp_SatFilterObs_t* pz_corr = NULL;

    for (u_i = 0; u_i < u_n; u_i++)
    {
      w_sat_idx = pu_satlist[u_i] - 1;
      if (w_sat_idx < 0||w_sat_idx >= ALL_GNSS_SYS_SV_NUMBER)
      {
        return 1;
      }
      pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[w_sat_idx];
      if (NULL == pz_satMeas)
      {
        return 2;
      }
      f_fact = 1.0;
      pz_corr = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[w_sat_idx];
      if (NULL == pz_corr || !(pz_corr->u_IonoCorrStatus & PPP_SSR_CORRECTED))
      {
        f_fact *= 100.0;
      }
      if (NULL != pz_satMeas->pz_signalMeas[0] && pz_satMeas->pz_signalMeas[0]->f_cn0 > 0.0)
      {
        pd_Qii[u_i] = f_fact / (pz_satMeas->pz_signalMeas[0]->f_cn0);
      }
      else
      {
        pd_Qii[u_i] = f_fact / 0.01;;
      }
      
    }
  return 0;
}
/**
 * @brief get partial AR quality list by lock
 * @param[in]    u_n
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pu_satlist
 * @param[out]   pd_Qii
 * @return status - 0: success, other:fail
 */
static uint8_t parLock(uint8_t u_n, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const uint8_t* pu_satlist, double* pd_Qii)
{
  uint8_t u_i = 0;
  int16_t w_sat_idx = 0;
  float f_fact = 1.0;
  float f_dt = 0.0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_corr = NULL;

  for (u_i = 0; u_i < u_n; u_i++)
  {
    w_sat_idx = pu_satlist[u_i] - 1;
    if (w_sat_idx < 0 || w_sat_idx >= ALL_GNSS_SYS_SV_NUMBER)
    {
      return 1;
    }
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[w_sat_idx];
    if (NULL == pz_satMeas)
    {
      return 2;
    }
    f_fact = 1.0;
    pz_corr = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[w_sat_idx];
    if (NULL == pz_corr || !(pz_corr->u_IonoCorrStatus & PPP_SSR_CORRECTED))
    {
      f_fact *= 100.0;
    }
    w_x_id[0] = 0;
    w_x_id[1] = gnss_cvt_Svid2SvIndex(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    w_x_id[2] = GNSS_FILTER_STATE_AMB_L1;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1)
    {
      return 3;
    }
    f_dt = (float)tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime);
    if (f_dt >= 0.0f)
    {
      pd_Qii[u_i] = f_fact / (f_dt + 1e-4);
    }
    else
    {
      pd_Qii[u_i] = f_fact / 1e-4;;
    }

  }
  return 0;
}
/**
 * @brief get partial AR quality list by elevation
 * @param[in]    u_n
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pu_satlist
 * @param[out]   pd_Qii
 * @return status - 0: success, other:fail
 */
static uint8_t parElevation(uint8_t u_n, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const uint8_t* pu_satlist, double* pd_Qii)
{
  uint8_t u_i = 0;
  int16_t w_sat_idx = 0;
  float f_fact = 1.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_corr = NULL;

  for (u_i = 0; u_i < u_n; u_i++)
  {
    w_sat_idx = pu_satlist[u_i] - 1;
    if (w_sat_idx < 0 || w_sat_idx >= ALL_GNSS_SYS_SV_NUMBER)
    {
      return 1;
    }
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[w_sat_idx];
    if (NULL == pz_satMeas)
    {
      return 2;
    }
    f_fact = 1.0;
    pz_corr = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[w_sat_idx];
    if (NULL == pz_corr || !(pz_corr->u_IonoCorrStatus & PPP_SSR_CORRECTED))
    {
      f_fact *= 100.0;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation > 0.0)
    {
      pd_Qii[u_i] = f_fact / (pz_satMeas->z_satPosVelClk.f_elevation);
    }
    else
    {
      pd_Qii[u_i] = f_fact / 0.01;;
    }

  }
  return 0;
}
/**
 * @brief get adop
 * @param[in]    pdQ
 * @param[in]    u_n
 * @param[out]   Qadop
 * @return status - 0: success, other:fail
 */
static uint8_t adop(const double* pdQ, uint8_t u_n, double* Qadop)
{
  uint8_t u_i = 0;
  double* L = NULL;
  double* D = NULL;

  L = (double*)OS_MALLOC_FAST((u_n * u_n) * sizeof(double));
  D = (double*)OS_MALLOC_FAST((u_n) * sizeof(double));
  if (NULL == L || NULL == D)
  {
    if (NULL != L) OS_FREE(L);
    if (NULL != D) OS_FREE(D);
    return 1;
  }
  /* LD factorization */
  if (LD(u_n, pdQ, L, D)) 
  {
    OS_FREE(L);
    OS_FREE(D);
    return 2;
  }
  (*Qadop) = 1.0;
  for (u_i = 0; u_i < u_n; u_i++)
  {
    (*Qadop) *= D[u_i];
  }
  OS_FREE(L);
  OS_FREE(D);
  return 0;
}

/**
 * @brief get partial AR quality list by adop
 * @param[in]    u_n
 * @param[in]    pu_satlist
 * @param[in]    pd_Q
 * @param[out]   pd_Qii
 * @return status - 0: success, other:fail
 */
static uint8_t parAdop(uint8_t u_n, const uint8_t* pu_satlist, const double* pd_Q, double* pd_Qii)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint32_t u_l = 0;
  int16_t w_sat_idx = 0;
  double* pd_Qtemp = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  if (u_n <= 1) return 1;
  pd_Qtemp = (double*)OS_MALLOC_FAST((u_n - 1)*(u_n - 1) * sizeof(double));
  if (NULL == pd_Qtemp)
  {
    return 2;
  }

  for (u_i = 0; u_i < u_n; u_i++)
  {
    u_l = 0;
    w_sat_idx = pu_satlist[u_i] - 1;
    if (w_sat_idx < 0)
    {
      OS_FREE(pd_Qtemp);
      return 3;
    }
    for (u_j = 0; u_j < u_n; u_j++)
    {
      if (u_j == u_i)
      {
        continue;
      }
      for (u_k = 0; u_k < u_n; u_k++)
      {
        if (u_k == u_i)
        {
          continue;
        }
        pd_Qtemp[u_l++] = pd_Q[u_j * u_n + u_k];
      }
    }
    if (adop(pd_Qtemp, u_n - 1, &pd_Qii[u_i]))
    {
      OS_FREE(pd_Qtemp);
      return 4;
    }
    pd_Qii[u_i] = 1.0 / pow((pd_Qii[u_i]), 1.0 / (2.0 * u_n));
    
  }
  OS_FREE(pd_Qtemp);
  return 0;
}
/**
 * @brief get partial AR quality list by Reduction
 * @param[in]    u_n
 * @param[in]    pd_Q
 * @param[out]   pd_Qii
 * @return status - 0: success, other:fail
 */
static uint8_t parReduction(uint8_t u_n, const double* pd_Q, double* pd_Qii)
{
  int32_t i = 0;
  int32_t j = 0;
  int32_t k = 0;
  uint8_t u_temp = 0;
  double del = 0.0;
  double* L = NULL;
  double* D = NULL;
  double* Z = NULL;
  uint8_t* pu_index = NULL;

  L = (double*)OS_MALLOC_FAST((u_n * u_n) * sizeof(double));
  D = (double*)OS_MALLOC_FAST((u_n) * sizeof(double));
  Z = (double*)OS_MALLOC_FAST((u_n * u_n) * sizeof(double));
  pu_index = (uint8_t*)OS_MALLOC_FAST((u_n) * sizeof(uint8_t));
  if (NULL == L || NULL == D||NULL==Z||NULL== pu_index)
  {
    if (NULL != L) OS_FREE(L);
    if (NULL != D) OS_FREE(D);
    if (NULL != Z) OS_FREE(Z);
    if (NULL != pu_index) OS_FREE(pu_index);
    return 1;
  }
  /* LD factorization */
  if (LD(u_n, pd_Q, L, D))
  {
    OS_FREE(L);
    OS_FREE(D);
    OS_FREE(Z);
    OS_FREE(pu_index);
    return 2;
  }
  for (i = 0; i < u_n; i++)
  {
    Z[i * (u_n + 1)] = 1.0;
    pu_index[i] = i;
  }
  j = u_n - 2; k = u_n - 2;
  while (j >= 0) {
    if (j <= k) for (i = j + 1; i < u_n; i++) gauss(u_n, L, Z, i, j);
    del = D[j] + L[j + 1 + j * u_n] * L[j + 1 + j * u_n] * D[j + 1];
    if (del + 1E-6 < D[j + 1]) { /* compared considering numerical error */
      perm(u_n, L, D, j, del, Z);
      u_temp = pu_index[j]; pu_index[j] = pu_index[j + 1]; pu_index[j + 1] = u_temp;
      k = j; j = u_n - 2;
    }
    else j--;
  }
  for (i = 0; i < u_n; i++)
  {
    pd_Qii[pu_index[i]] = pow(10, u_n*1.0-i);
  }

  OS_FREE(L);
  OS_FREE(D);
  OS_FREE(Z);
  OS_FREE(pu_index);
  return 0;
}
/**
 * @brief get partial AR sorted list
 * @param[in]    u_method
 * @param[in]    u_n
 * @param[out]   pu_index
 * @param[out]   pd_Qii
 * @param[in]    pu_satlist
 * @param[in]    pd_Q
 * @param[in]    pz_pppAmbFixInputInfo
 * @return status - 0: success, other:fail
 */
static uint8_t getQindex(uint8_t u_method, uint8_t u_n, uint8_t* pu_index, double* pd_Qii, const uint8_t* pu_satlist
  , const double* pd_Q, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_tempi = 0;
  double tempQ = 0.0;

  LOGI(TAG_PPP, "Par-AR method is %d\n", u_method);
  for (u_i = 0; u_i < u_n; u_i++)
  {
    pu_index[u_i] = u_i;
  }
  switch (u_method)
  {
  case PPP_PAR_AR_METHOD_COV:    /* covariance method */
  {
    if (parCovariance(u_n, pz_pppAmbFixInputInfo, pu_satlist, pd_Q, pd_Qii))
    {
      u_status = 1;
    }
    break;  /* Q */
  }
  case PPP_PAR_AR_METHOD_SNR:   /* snr method */
  {
    if (parSnr(u_n, pz_pppAmbFixInputInfo, pu_satlist, pd_Qii))
    {
      u_status = 2;
    }
    break;
  }
  case PPP_PAR_AR_METHOD_LOCK:  /* lock method */
  {
    if (parLock(u_n, pz_pppAmbFixInputInfo, pu_satlist, pd_Qii))
    {
      u_status = 3;
    }
    break;
  }
  case PPP_PAR_AR_METHOD_ELE:  /* elevation method */
  {
    if (parElevation(u_n, pz_pppAmbFixInputInfo, pu_satlist, pd_Qii))
    {
      u_status = 4;
    }
    break;
  }
  case PPP_PAR_AR_METHOD_ADOP:  /* ADOP method */
  {
    if (parAdop(u_n, pu_satlist, pd_Q, pd_Qii))
    {
      u_status = 5;
    }
    break;
  }
  case PPP_PAR_AR_METHOD_RED:  /* Reduction method */
  {
    if (parReduction(u_n, pd_Q, pd_Qii))
    {
      u_status = 6;
    }
    break;
  }
  default:
    u_status = 7;
    break;
  }
  /* sort Qii */
  for (u_i = 0; u_i < u_n && !u_status; u_i++)
  {
    for (u_j = u_i + 1; u_j < u_n; u_j++)
    {
      if (pd_Qii[u_j] > pd_Qii[u_i])
      {
        tempQ = pd_Qii[u_j]; pd_Qii[u_j] = pd_Qii[u_i]; pd_Qii[u_i] = tempQ;
        u_tempi = pu_index[u_j]; pu_index[u_j] = pu_index[u_i]; pu_index[u_i] = u_tempi;
      }
    }
  }
  return u_status;
}

/**
 * @brief partial ambiguity resolution
 * @param[in]    pd_X
 * @param[in]    pd_Q
 * @param[in]    pu_index
 * @param[out]    pd_X_new
 * @param[out]    pd_Q_new
 * @param[in]    u_d
 * @param[in]    u_ns
 * @param[out]    pu_satlist_new
 * @param[in]    pu_satlist
 * @return status - 0: success, other:fail
 */
static uint8_t rebuildXQ(const double* pd_X, const double* pd_Q, const uint8_t* pu_index, double* pd_X_new
  , double* pd_Q_new, uint8_t u_d, uint8_t u_ns, uint8_t* pu_satlist_new, const uint8_t* pu_satlist)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_l = 0;
  uint8_t u_m = 0;
  int16_t* puw_Xindex = NULL;
  int16_t* puw_Qindex = NULL;
  puw_Xindex = OS_MALLOC_FAST(u_ns * sizeof(int16_t));
  puw_Qindex = OS_MALLOC_FAST(u_ns * u_ns * sizeof(int16_t));
  if (NULL == puw_Xindex || NULL == puw_Qindex)
  {
    if (NULL != puw_Xindex)
    {
      OS_FREE(puw_Xindex);
      puw_Xindex = NULL;
    }
    if (NULL != puw_Qindex)
    {
      OS_FREE(puw_Qindex);
      puw_Qindex = NULL;
    }
    u_status = 1;
    return u_status;
  }

  for (u_i = 0; u_i < u_d; u_i++)
  {
    u_k = pu_index[u_i];
    puw_Xindex[u_k] = -1;
    for (u_j = 0; u_j < u_ns; u_j++)
    {
      puw_Qindex[u_k + u_j * u_ns] = -1;
      puw_Qindex[u_k * u_ns + u_j] = -1;
    }
  }
  for (u_i = 0, u_l = 0, u_m = 0; u_i < u_ns; u_i++) {
    if (puw_Xindex[u_i] < 0) continue;
    pd_X_new[u_l] = pd_X[u_i];
    if (NULL != pu_satlist_new && NULL != pu_satlist )
    {
      pu_satlist_new[u_l] = pu_satlist[u_i];
    }
    u_l++;
    for (u_j = 0; u_j < u_ns; u_j++)
    {
      if (puw_Qindex[u_j + u_i * u_ns] < 0) continue;
      pd_Q_new[u_m] = pd_Q[u_j + u_i * u_ns];
      if (u_i == u_j)
      {
        pd_Q_new[u_m] += 1e-6;
      }
      u_m++;
    }
  }
  OS_FREE(puw_Xindex);
  puw_Xindex = NULL;
  OS_FREE(puw_Qindex);
  puw_Qindex = NULL;
  return u_status;
}
/**
 * @brief partial ambiguity resolution from history amb
 * @param[in]    u_n
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_curfixedAmbPool
 * @param[in]    z_freqType
 * @param[out]    pu_namb
 * @param[in/out]    pu_satlist
 * @param[in/out]    pd_amb_int
 * @param[out]    pu_index
 * @param[out]   pd_amb_int_his
 * @param[in]     MINAR
 * @return status - 0: success, other:fail
 */
static uint8_t historyAmbTrans(uint8_t u_n, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
  gnss_FreqType z_freqType, uint8_t* pu_namb, uint8_t* pu_satlist, double* pd_X, uint8_t* pu_index, double* pd_amb_int_his, uint8_t MINAR)
{
  char c_sat[4] = "";
  uint8_t u_i = 0;
  uint8_t u_flag = 0;
  uint8_t u_namb = 0;
  uint8_t u_svid = 0;
  uint8_t u_sys = 0;
  uint8_t u_isys = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_refsat_idx = 0;
  uint8_t u_ref_svid = 0;
  uint8_t* pu_satlist_new = NULL;
  double   d_sdamb = 0.0;
  double* pd_X_new = NULL;
  const gnss_fixedSignalAmbPool_t* pz_prefixedAmbPool = NULL;

  if (u_n < 1)
  {
    return 1;
  }

  pu_satlist_new = (uint8_t*)OS_MALLOC_FAST(u_n * sizeof(uint8_t));
  pd_X_new = (double*)OS_MALLOC_FAST(u_n * sizeof(double));
  if (NULL == pu_satlist_new || NULL == pd_X_new)
  {
    if (NULL != pu_satlist_new) OS_FREE(pu_satlist_new);
    if (NULL != pd_X_new) OS_FREE(pd_X_new);
    return 2;
  }

  pz_prefixedAmbPool = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx];
  if (NULL == pz_prefixedAmbPool || pz_prefixedAmbPool->u_fixStatus == GNSS_NONE_AMB_FIXED
    || tm_GpsTimeDiff(&pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor, &pz_prefixedAmbPool->z_time)>60.0)
  {
    OS_FREE(pu_satlist_new);
    OS_FREE(pd_X_new);
    LOGI(TAG_PPP, "History amb trans fail by previous info\n");
    return 3;
  }

  for (u_i = 0; u_i < u_n; u_i++) 
  {
    u_sat_idx = pu_satlist[u_i] - 1;
    if (u_sat_idx < 0|| NULL==pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx])
    {
      continue;
    }
    
    u_svid=gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_isys = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, u_sys);
    svid_SatString(u_svid, u_sys, c_sat);
    u_ref_svid = pz_curfixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType];
    if (u_ref_svid < 1)
    {
      continue;
    }
    u_refsat_idx = gnss_cvt_Svid2SvIndex(u_ref_svid, u_sys);

    if (u_ref_svid == pz_prefixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType])
    {
      d_sdamb = pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue;
    }
    else 
    {
      if (NULL == pz_prefixedAmbPool->pz_fixedAmbSet[u_refsat_idx])
      {
        continue;
      }
      if (pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_continue_fix[z_freqType]>0 && 
        pz_prefixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->u_continue_fix[z_freqType] > 0)
      {
        d_sdamb = pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue - pz_prefixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->d_fixedValue;
      }
      else
      {
        d_sdamb = pd_X[u_i] + 100; // prevent default amb is 0
      }
    }
    u_flag = 0;
    if (pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_continue_fix[z_freqType] >= 3) 
    {
      if (fabs(pd_X[u_i] - d_sdamb)<0.4&& fabs(round(pd_X[u_i]) - round(d_sdamb))<FABS_ZEROS)
      {
        pu_satlist_new[u_namb] = pu_satlist[u_i];
        pd_X_new[u_namb] = pd_X[u_i];
        pu_index[u_namb] = u_i;
        pd_amb_int_his[u_namb] = d_sdamb;
        u_namb++;
        u_flag = 1;
      }
      else if(d_sdamb&&fabs(pd_X[u_i]- d_sdamb)>30) //if diff > 30��the history amb is error, reset
      {
        pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_continue_fix[z_freqType] = 0;
        pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue = 0;
      }
    }
    LOGI(TAG_PPP, "His-amb info %s %8.2lf %8.2lf %4d  %u\n", c_sat, pd_X[u_i], d_sdamb, pz_prefixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_continue_fix[z_freqType], u_flag);
  }
  if (u_namb >= MINAR &&((u_namb*1.0/u_n)>0.6|| u_namb>6))
  {
    *pu_namb = u_namb;
    LOGI(TAG_PPP, "History amb trans success,n=%u\n", u_namb);
    for (u_i = 0; u_i < u_namb; u_i++)
    {
      pu_satlist[u_i] = pu_satlist_new[u_i];
      pd_X[u_i] = pd_X_new[u_i];
    }
    OS_FREE(pu_satlist_new);
    OS_FREE(pd_X_new);
    return 0;
  }
  else
  {
    LOGI(TAG_PPP, "History amb trans fail,n=%u %u\n", u_namb, u_n);
  }
  
  OS_FREE(pu_satlist_new);
  OS_FREE(pd_X_new);
  return 4;
}

 /**
  * @brief          ppp ambiguity solution availability check
  * @param[in]      pz_pppAmbFixInputInfo
  * @param[out]     pz_fixedAmbPool is result of amb resolution
  * @return         0: success, other: ar fail
  */
static uint32_t PPP_AmbResolutionCheck(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_sys = 0;
  uint8_t u_wl = 0;
  uint8_t u_gps = 0;
  uint8_t u_gps_thres = 0;
  uint32_t u_status = 0;
  float f_floatSTDCode;
  float f_floatSTDPhase;
  float f_fixedSTDCode;
  float f_fixedSTDPhase;
  float f_codeSTDthres = 1.0f;
  float f_phaseSTDthres = 0.01f;
  float f_PDOPthres = 15.0f;
  double d_horVel = 0.0;
  double d_enu_vel[3] = { 0.0 };
  const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool_UW = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[GNSS_UW_AMB_FIXED];
  const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool_W1 = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[GNSS_W1_AMB_FIXED];
  const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool_W2 = pz_pppAmbFixInputInfo->pz_preFixedAmbPool[GNSS_W2_AMB_FIXED];
  
  getFixDops(pz_pppAmbFixInputInfo, pz_curfixedAmbPool);
  f_floatSTDCode = pz_pppAmbFixInputInfo->pz_FilterObs->f_postCodeSTD;
  f_floatSTDPhase = pz_pppAmbFixInputInfo->pz_FilterObs->f_postPhaseSTD;
  f_fixedSTDCode = pz_pppAmbFixInputInfo->pz_FilterObs->f_fixedCodeSTD;
  f_fixedSTDPhase = pz_pppAmbFixInputInfo->pz_FilterObs->f_fixedPhaseSTD;

  if (pz_curfixedAmbPool->u_fixStatus == GNSS_NONE_AMB_FIXED)
  {
    return u_status;
  }

  getEnuVel(pz_pppAmbFixInputInfo->pz_EKFstateRepPool, pz_pppAmbFixInputInfo->pz_pppFilterInfo->pd_X, d_enu_vel);
  d_horVel = sqrt(d_enu_vel[0] * d_enu_vel[0] + d_enu_vel[1] * d_enu_vel[1]);
  if (d_horVel > 60 || fabs(d_enu_vel[2]) > 3.0)
  {
    u_status = PPP_STATUS_AR_CheckVel;
    pz_curfixedAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;
    LOGW(TAG_PPP, "Fix velocity is bad: H=%.3lf   V=%.3lf\n", d_horVel, d_enu_vel[2]);
  }

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; ++u_i)
  {
    if (NULL == pz_curfixedAmbPool->pz_fixedAmbSet[u_i])
    {
      continue;
    }
    if (GNSS_WL_AMB_FIXED & pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_ambFixType)
    {
      u_wl++;
      gnss_cvt_SvIndex2Svid(u_i, &u_sys);
      if (C_GNSS_GPS == u_sys)
      {
        u_gps++;
      }
    }
  }
  if (u_gps > 0)
  {
    f_codeSTDthres = 1.5f;
    f_phaseSTDthres = 0.02f;
  }
  if ((NULL != pz_curfixedAmbPool_UW && pz_curfixedAmbPool_UW->f_ratio > 3.0 && pz_curfixedAmbPool_UW->u_nb >= 8)
    || (NULL != pz_curfixedAmbPool_W1 && pz_curfixedAmbPool_W1->f_ratio > 3.0 && pz_curfixedAmbPool_W1->u_nb >= 8)
    || (NULL != pz_curfixedAmbPool_W2 && pz_curfixedAmbPool_W2->f_ratio > 3.0 && pz_curfixedAmbPool_W2->u_nb >= 8))
  {
    u_gps_thres = 0;
  }
  else u_gps_thres = 3;

  if (pz_curfixedAmbPool->f_pdop > f_PDOPthres || f_floatSTDCode < FABS_ZEROS || f_floatSTDPhase < FABS_ZEROS
    || f_fixedSTDCode < FABS_ZEROS || f_fixedSTDPhase < FABS_ZEROS)
  {
    u_status = PPP_STATUS_AR_CheckDop;
    pz_curfixedAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;
    LOGI(TAG_PPP, "PPP-AR checking fail - f_pdop\n");
  }
  else if ((!(pz_curfixedAmbPool->u_fixStatus & GNSS_NL_AMB_FIXED) && (pz_curfixedAmbPool->u_fixStatus & GNSS_WL_AMB_FIXED))
    && (u_wl < 8 || u_gps < u_gps_thres)) // WL fix and NL do not fix
  {
    u_status = PPP_STATUS_AR_CheckWLnoNL;
    pz_curfixedAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;
    LOGI(TAG_PPP, "PPP-AR checking fail - WL\n");
  }
  else if (((pz_curfixedAmbPool->u_fixStatus & GNSS_NL_AMB_FIXED) && (!(pz_curfixedAmbPool->u_fixStatus & GNSS_WL_AMB_FIXED)))
    && pz_curfixedAmbPool->f_ratio < 2.0) // WL do not fix and NL fix
  {
    u_status = PPP_STATUS_AR_CheckNLnoWL;
    pz_curfixedAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;
    LOGI(TAG_PPP, "PPP-AR checking fail - NL\n");
  }

  if (pz_curfixedAmbPool->u_fixStatus != GNSS_NONE_AMB_FIXED)
  {
    if ((f_fixedSTDPhase > f_phaseSTDthres && f_fixedSTDCode > f_codeSTDthres) || ((f_fixedSTDPhase / f_floatSTDPhase) > 3.0 && f_fixedSTDPhase > 0.03)
      || f_fixedSTDPhase > 0.04 || f_fixedSTDCode > 3.0)
    {
      u_status = PPP_STATUS_AR_CheckResidual;
      pz_curfixedAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;
      LOGI(TAG_PPP, "PPP-AR checking fail - STD\n");
    }
  }

  return u_status;
}
PPP_AmbFixDeleteConstellationType PPP_convertConstellationEnumToAmbDeleteType(gnss_ConstellationType u_excludeSYS)
{
  PPP_AmbFixDeleteConstellationType u_deleteSysType = PPP_AMB_FIX_DELETE_NONE_SYS;
  if (C_GNSS_GPS == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_GPS_SYS;
  }
  else if (C_GNSS_GLO == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_GLO_SYS;
  }
  else if (C_GNSS_BDS2 == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_BDS2_SYS;
  }
  else if (C_GNSS_BDS3 == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_BDS3_SYS;
  }
  else if (C_GNSS_GAL == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_GAL_SYS;
  }
  else if (C_GNSS_QZS == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_QZS_SYS;
  }
  else if (C_GNSS_NAVIC == u_excludeSYS)
  {
    u_deleteSysType = PPP_AMB_FIX_DELETE_NAVIC_SYS;
  }
  return u_deleteSysType;
}
/**
 * @brief select ten best amb list
 * @param[in]      u_n
 * @param[in]      pu_satlist
 * @param[in/out]  pu_index
 * @param[in]      excludeSYS
 * @param[out]     u_ndeleteThres
 * @return status - u_deleteIDX: first delete index
 */
static uint8_t selectFirstTenAmb(uint8_t u_n, uint8_t* pu_satlist, uint8_t* pu_index, uint8_t excludeSYS,uint8_t* u_ndeleteThres)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_isys = 0;
  uint8_t u_jsys = 0;
  uint8_t u_temp = 0;
  uint8_t u_nbds = 0;
  uint8_t u_exc = 0;
  uint8_t u_deleteIDX = 0;
  PPP_AmbFixDeleteConstellationType u_iDeleteSysType = PPP_AMB_FIX_DELETE_NONE_SYS;
  PPP_AmbFixDeleteConstellationType u_jDeleteSysType = PPP_AMB_FIX_DELETE_NONE_SYS;

  /* exclude amb of SYS */
  for (u_i = 0; u_i < u_n ; u_i++)
  {
    gnss_cvt_SvIndex2Svid(pu_satlist[pu_index[u_i]] - 1, &u_isys);
    u_iDeleteSysType = PPP_convertConstellationEnumToAmbDeleteType(u_isys);
    for (u_j = u_i + 1; u_j < u_n; u_j++) // delete exclude-SYS first 
    {
      gnss_cvt_SvIndex2Svid(pu_satlist[pu_index[u_j]] - 1, &u_jsys);
      u_jDeleteSysType = PPP_convertConstellationEnumToAmbDeleteType(u_jsys);
      if (0 == (excludeSYS & u_iDeleteSysType) && 0 != (excludeSYS & u_jDeleteSysType))
      {
        u_temp = pu_index[u_i];
        pu_index[u_i] = pu_index[u_j];
        pu_index[u_j] = u_temp;
      }
    }
  }
  for (u_i = 0; u_i < u_n; u_i++)
  {
    gnss_cvt_SvIndex2Svid(pu_satlist[u_i] - 1, &u_isys);
    u_iDeleteSysType = PPP_convertConstellationEnumToAmbDeleteType(u_isys);
    if (C_GNSS_BDS3 == u_isys || C_GNSS_BDS2 == u_isys)
    {
      u_nbds++;
    }
    if (0 != (excludeSYS & u_iDeleteSysType))
    {
      u_exc++;
    }
  }

  /* delete some of satellites */
  if (u_n > 10)
  {
    u_deleteIDX = u_n - 10;
  }
  else
  {
    u_deleteIDX = 0;
  }

  if (u_exc > u_deleteIDX)
  {
    u_deleteIDX = u_exc;
  }
  else
  {
    u_deleteIDX = u_deleteIDX;
  }
  LOGI(TAG_PPP, "par_amb delete SYS nexc=%d\n", u_exc);
  *u_ndeleteThres = (u_deleteIDX + 4);
  if (*u_ndeleteThres > u_n)
  {
    *u_ndeleteThres = u_n;
  }
  return u_deleteIDX;
}
/**
 * @brief partial ambiguity resolution
 * @param[in]    method
 * @param[in]    exBDS
 * @param[in]    u_n
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[out]    pz_curfixedAmbPool
 * @param[out]    z_freqType
 * @param[out]    pu_namb
 * @param[out]    pu_satlist
 * @param[out]    pd_X
 * @param[out]    pd_Q
 * @param[out]    pd_amb_int
 * @param[out]    pf_ratio
 * @param[in]    MINAR
 * @param[in]    f_minratio
 * @return status - 0: success, other:fail
 */
static uint8_t parAmbResolution(uint8_t method, uint8_t excludeSYS, uint8_t u_n, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, 
  gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool, gnss_FreqType z_freqType, uint8_t* pu_namb, uint8_t* pu_satlist
  , double* pd_X, double* pd_Q, double* pd_amb_int, uint8_t MINAR, float f_minratio, float ave_frac, float max_frac)
{
  const uint8_t u_candidate = 20;
  uint8_t u_status = 10;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_sys = 0;
  uint8_t u_jsys = 0;
  uint8_t u_nbds = 0;
  uint8_t u_iter = 0;
  uint8_t u_m = 0;
  uint8_t u_nhis = 0;
  uint8_t u_deleteIDX = 0;
  uint8_t u_ndeleteThres = 0;
  uint8_t* pu_index = NULL;
  uint8_t* pu_index_his = NULL;
  uint8_t u_sat_idx = 0;
  float f_thre_rate = 0.0;
  float f_frac = 0.0;
  float f_frac_ave = 0.0;
  float f_frac_max = 0.0;
  float f_ratio = 0.0;
  float f_boot_rate = 0.0;
  char char_sat[4] = "";
  double* pd_Qii = NULL;
  double* pd_X_new = NULL;
  double* pd_X_his = NULL;
  double* pd_Q_new = NULL;
  double* pd_Q_his = NULL;
  double* pd_sigma = NULL;
  double* pd_amb_int_his = NULL;
  uint8_t* pu_satlist_new = NULL;
  uint8_t* pu_satlist_his = NULL;

  pz_curfixedAmbPool->f_ratio = 0.0;
  pd_Qii = OS_MALLOC_FAST(u_n * sizeof(double));
  pu_index = OS_MALLOC_FAST(u_n * sizeof(uint8_t));
  pu_index_his = OS_MALLOC_FAST(u_n * sizeof(uint8_t));
  pd_X_new = OS_MALLOC_FAST(u_n * sizeof(double));
  pd_X_his = OS_MALLOC_FAST(u_n * sizeof(double));
  pd_Q_new = OS_MALLOC_FAST(u_n * u_n * sizeof(double));
  pd_Q_his = OS_MALLOC_FAST(u_n * u_n * sizeof(double));
  pu_satlist_new = OS_MALLOC_FAST(u_n * sizeof(uint8_t));
  pu_satlist_his = OS_MALLOC_FAST(u_n * sizeof(uint8_t));
  pd_amb_int_his = OS_MALLOC_FAST(u_n * sizeof(double));
  pd_sigma = OS_MALLOC_FAST(u_candidate * sizeof(double));

  if (NULL == pd_Qii || NULL == pu_index || NULL == pd_X_new || NULL == pd_X_his || NULL == pd_Q_new || NULL == pd_Q_his
    || NULL == pu_satlist_new || NULL == pu_satlist_his || NULL == pd_amb_int || NULL == pd_sigma||NULL== pu_index_his
    ||NULL== pd_amb_int_his)
  {
    u_status = 1;
  }
  else
  {
    /* get partial AR sorted list */
    u_status = getQindex(method, u_n, pu_index, pd_Qii, pu_satlist, pd_Q, pz_pppAmbFixInputInfo);

    /* select ten best amb list */
    u_deleteIDX = selectFirstTenAmb(u_n, pu_satlist, pu_index, excludeSYS, &u_ndeleteThres);

    for (u_i = u_deleteIDX; u_i < u_ndeleteThres; u_i++)
    {
      u_m = u_n - u_i; u_iter++;
      if (u_m < MINAR || (u_m * 1.0 / (u_n - u_deleteIDX)) < 0.5)
      {
        u_status = 2;
        break;
      }

      /* delete worst amb and rebuild X and Q */
      memset(pd_X_new, 0, sizeof(double) * u_n);
      memset(pd_Q_new, 0, sizeof(double) * u_n * u_n);
      for (u_j = 0; u_j < u_n; u_j++) pu_satlist_new[u_j] = 0;
      if (rebuildXQ(pd_X, pd_Q, pu_index, pd_X_new, pd_Q_new, u_i, u_n, pu_satlist_new, pu_satlist))
      {
        u_status = 3;
        break;
      }

      /*search amb by BIE */
      if (lambdaBie(u_m, u_candidate, pd_X_new, pd_Q_new, pd_amb_int, pd_sigma, &f_boot_rate, &pz_curfixedAmbPool->f_adop))
      {
        u_status = 4;
        break;
      }

      /* average of Fractional part */
      f_frac_ave = 0.0;
      f_frac_max = 0.0;
      for (u_j = 0; u_j < u_m; u_j++)
      {
        f_frac = (float)fabs(pd_amb_int[u_j] - round(pd_amb_int[u_j]));
        f_frac_ave += f_frac;
        if (f_frac > f_frac_max)
        {
          f_frac_max = f_frac;
          satidx_SatString(pu_satlist_new[u_j] - 1, char_sat);
        }
      }
      f_frac_ave = f_frac_ave / u_m;
      LOGI(TAG_PPP, "parAmbResolution frac_ave=%.3f  frac_max=%.3f,sat=%s\n", f_frac_ave, f_frac_max, char_sat);

      f_ratio = (float)(pd_sigma[0] > 0 ? (float)(pd_sigma[1] / pd_sigma[0]) : 0.0);
      if (f_ratio > 999.99)
      {
        f_ratio = 999.99f;
      }
      pz_curfixedAmbPool->f_lambda_sigma[0] = pd_sigma[0];
      pz_curfixedAmbPool->f_lambda_sigma[1] = pd_sigma[1];
      pz_curfixedAmbPool->f_ratio = f_ratio;
      if (u_m <= 4 && f_ratio < 2.0f) /* ratio */
      {
        u_status = 5;
        break;
      }

      u_nbds = 0;
      for (u_j = 0; u_j < u_m; u_j++)
      {
        u_sat_idx = pu_satlist_new[u_j] - 1;
        gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
        if (C_GNSS_BDS3 == u_sys || C_GNSS_BDS2 == u_sys)
        {
          u_nbds++;
        }
      }
      f_thre_rate = 0.95f;
      if (u_i > 0)
      {
        satidx_SatString(pu_satlist[pu_index[u_i - 1]] - 1, char_sat);
      }
      else
      {
        strcpy(char_sat, "   ");
      }

      if (u_m <= 5)
      {
        f_minratio = 2.0;
      }
      LOGI(TAG_PPP, "par_amb delete, sat=%s ratio=%.1lf bootrate=%.2lf Q=%.3lf thre_rate=%.2lf adop=%7.3lf\n",
        char_sat, f_ratio, f_boot_rate, u_i > 0 ? pd_Qii[u_i - 1] : 0.0, f_thre_rate, pz_curfixedAmbPool->f_adop);
      if (f_ratio > f_minratio && f_boot_rate > f_thre_rate && f_frac_ave <= ave_frac
        && pz_curfixedAmbPool->f_adop >= 0.0f&& f_frac_max<=max_frac) /* AR success */
      {
        u_status = 0;
        break;
      }
      u_status = 6;
    }
    /* fix amb according to history ambiguity */
    if (u_m <= 6 || f_ratio < 2.0)
    {
      memcpy(pd_X_his, pd_X, u_n * sizeof(double));
      memcpy(pd_Q_his, pd_Q, u_n * u_n * sizeof(double));
      memcpy(pu_satlist_his, pu_satlist, u_n * sizeof(uint8_t));
      historyAmbTrans(u_n, pz_pppAmbFixInputInfo, pz_curfixedAmbPool, z_freqType, &u_nhis, pu_satlist_his, pd_X_his, pu_index_his, pd_amb_int_his,MINAR);
    }

    if (u_nhis > u_m && u_nhis >= MINAR)
    {
      LOGI(TAG_PPP, "His-Amb used,u_nhis=%02d, nbie=%02d\n", u_nhis, u_m);
      for (u_i = 0; u_i < u_nhis; u_i++)
      {
        pd_X[u_i] = pd_X_his[u_i];
        pd_amb_int[u_i] = pd_amb_int_his[u_i];
        pu_satlist[u_i] = pu_satlist_his[u_i];
        for (u_j = 0; u_j < u_nhis; u_j++)
        {
          pd_Q[u_j + u_i * u_nhis] = pd_Q_his[pu_index_his[u_j] + pu_index_his[u_i] * u_n];
        }
      }
      (*pu_namb) = u_nhis; u_status = 0;
    }
    else if (!u_status)
    {
      for (u_i = 0; u_i < u_m; u_i++)
      {
        pd_X[u_i] = pd_X_new[u_i]; pu_satlist[u_i] = pu_satlist_new[u_i];
        for (u_j = 0; u_j < u_m; u_j++)
        {
          pd_Q[u_j + u_i * u_m] = pd_Q_new[u_j + u_i * u_m];
        }
      }
      (*pu_namb) = u_m;
      LOGI(TAG_PPP, "lambda used,u_nhis=%02d, nbie=%02d\n", u_nhis, (*pu_namb));
    }
  }
  
  OS_FREE(pd_Qii);
  OS_FREE(pu_index);
  OS_FREE(pu_index_his);
  OS_FREE(pd_X_new);
  OS_FREE(pd_X_his);
  OS_FREE(pd_Q_new);
  OS_FREE(pd_Q_his);
  OS_FREE(pu_satlist_new);
  OS_FREE(pu_satlist_his);
  OS_FREE(pd_sigma);
  OS_FREE(pd_amb_int_his);
  
  return u_status;
}

static double q_gamma(double a, double x, double log_gamma_a);
static double p_gamma(double a, double x, double log_gamma_a)
{
  double y = 0.0;
  double w = 0.0;
  uint8_t i = 0;

  if (x == 0.0)
  {
    return 0.0;
  }
  if (x >= a + 1.0)
  {
    return 1.0 - q_gamma(a, x, log_gamma_a);
  }

  y = w = exp(a * log(x) - x - log_gamma_a) / a;

  for (i = 1; i < 100; i++) {
    w *= x / (a + i);
    y += w;
    if (fabs(w) < 1E-15)
    {
      break;
    }
  }
  return y;
}
static double q_gamma(double a, double x, double log_gamma_a)
{
  double y = 0.0;
  double w = 0.0;
  double la = 1.0;
  double lb = x + 1.0 - a, lc;
  uint8_t i = 0;

  if (x < a + 1.0)
  {
    return 1.0 - p_gamma(a, x, log_gamma_a);
  }
  w = exp(-x + a * log(x) - log_gamma_a);
  y = w / lb;
  for (i = 2; i < 100; i++) {
    lc = ((i - 1 - a) * (lb - la) + (i + x) * lb) / i;
    la = lb;
    lb = lc;
    w *= (i - 1 - a) / i;
    y += w / la / lb;
    if (fabs(w / la / lb) < 1E-15)
    {
      break;
    }
  }
  return y;
}

static double f_erfc(double x)
{
  return x >= 0.0 ? q_gamma(0.5, x * x, LOG_PI / 2.0) : 1.0 + p_gamma(0.5, x * x, LOG_PI / 2.0);
}
/* confidence function of integer ambiguity ----------------------------------*/
static  double confidence(double N, double B, double sig)
{
  double x = 0.0;
  double p = 1.0;
  uint8_t i = 0;;

  if (fabs(sig) < FABS_ZEROS)
  {
    return p;
  }
  x = fabs(B - N);
  for (i = 1; i < 8; i++) {
    p -= f_erfc((i - x) / (sqrt(2) * sig)) - f_erfc((i + x) / (sqrt(2) * sig));
  }
  return p;
}

/**
 * @brief success rate computation
 * @param[in]    pd_Q
 * @param[in]    i1,i2,j1,j2
 * @param[in]    WL_d
 * @param[in]    WL_f
 * @return success rate
 */
static float computeSuccess(const double* pd_Q, uint16_t i1, uint16_t j1, uint16_t i2, uint16_t j2, double WL_d, double WL_f)
{
  float int_rate = 0.0;
  double Qii = 0.0;

  Qii  = pd_Q[IUTM(i1, i1)] + pd_Q[IUTM(j1, j1)];
  Qii -= pd_Q[IUTM(i1, j1)] + pd_Q[IUTM(j1, i1)];

  Qii += pd_Q[IUTM(i2, i2)] + pd_Q[IUTM(j2, j2)];
  Qii -= pd_Q[IUTM(i2, j2)] + pd_Q[IUTM(j2, i2)];

  Qii += pd_Q[IUTM(j2, i1)] + pd_Q[IUTM(i1, j2)];
  Qii -= pd_Q[IUTM(i2, i1)] + pd_Q[IUTM(i1, i2)];

  Qii += pd_Q[IUTM(i2, j1)] + pd_Q[IUTM(j1, i2)];
  Qii -= pd_Q[IUTM(j1, j2)] + pd_Q[IUTM(j2, j1)];

  int_rate = (float)confidence(WL_d, WL_f, sqrt(Qii));

  return int_rate;
}

/**
 * @brief get list of satellites preliminarily
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @param[out]    pu_satlist
 * @return status - 0: success, other:fail
 */
static uint8_t getWlSatlist(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t* pu_satlist)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_ns = 0;
  uint8_t u_ns_all = 0;
  uint8_t u_sat_flag = 0;
  uint8_t u_sys_ns[C_GNSS_MAX] = { 0 };
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_refsat_svid = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_refsat_idx = 0;
  float f_minlock = 0.0;
  float f_upd = 0.0;
  float f_code = 0.0;
  float f_wl_amb = 0.0;
  float f_mw_amb = 0.0;
  float f_dfrac = 0.0;
  float f_rate_s1 = 0.0;
  float f_rate_s2 = 0.0;
  double pd_lambdaCoef = 0.0;
  double pd_pseudoCoef1 = 0.0;
  double pd_pseudoCoef2 = 0.0;
  char c_flag = ' ';
  char char_sat[4] = "";
  char char_refsat[4] = "";
  char c_buff[64] = "";
  char* p = c_buff;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const GpsTime_t* pz_curtime = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal2 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal2_ref = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  /* check loop all of satellites */
  pz_curtime = &pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor;
  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    c_flag = 'O';
    u_sat_flag = 0;
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    pz_corrStat = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_sat_idx];
    if (NULL == pz_satMeas || NULL == pz_corrStat)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if ((NULL == pz_satLos) || pz_satLos->z_orbClk.f_qi * 0.001 > PPPAR_MAX_LOS_QI
      || !(GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID & pz_satLos->u_windUpMask))
    {
      continue;
    }
    if (NULL == pz_satMeas->pz_signalMeas[z_freqType1] || NULL == pz_satMeas->pz_signalMeas[z_freqType2]
      || pz_satMeas->pz_signalMeas[z_freqType1]->u_LLI&2 || pz_satMeas->pz_signalMeas[z_freqType2]->u_LLI&2)
    {
      continue;
    }
    if (NULL== pz_corrStat->z_measUpdateFlag[z_freqType1]|| NULL == pz_corrStat->z_measUpdateFlag[z_freqType2]
      ||!pz_corrStat->z_measUpdateFlag[z_freqType1]->b_cpValid || !pz_corrStat->z_measUpdateFlag[z_freqType2]->b_cpValid
      ||!pz_corrStat->pf_phaseResidual[z_freqType1]||!pz_corrStat->pf_phaseResidual[z_freqType2])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    if (pz_satMeas->u_svid == u_refsat_svid || u_refsat_svid <= 0)  /* the same as refsat */
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))  /* do not select GEO */
    {
      continue;
    }

    /* get status from EKF */
    w_x_id[0] =  0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[0] = 0;
    u_refsat_idx = gnss_cvt_Svid2SvIndex(u_refsat_svid, pz_satMeas->u_constellation);;
    w_x_id[1] = u_refsat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPool1_ref || NULL == pz_satPool2_ref)
    {
      continue;
    }

    if (fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool1->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool2->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool1_ref->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool2_ref->z_endTime)) > 1e-4) /* unhealth,current epoch */
    {
      continue;
    }
    
    /* get code bias and phase bias */
    pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal2 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
    pz_signal2_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (NULL == pz_signal1 || NULL == pz_signal2 || NULL == pz_signal1_ref || NULL == pz_signal2_ref
      || !(pz_signal1->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR) || !(pz_signal2->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR)
      || !(pz_signal1_ref->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR) || !(pz_signal2_ref->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
    {
      c_flag = 'P'; u_sat_flag = 1; f_upd = 0.0; f_code = 0.0;  /* bias is unhealty */
    }
    else
    {
      f_upd = (float)((pz_signal1->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal) - pz_signal2->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal)
        - (pz_signal1_ref->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal) - pz_signal2_ref->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal))) * 0.001);

      pd_lambdaCoef = (wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal) - wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal)) /
        (wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal) + wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal));
      pd_pseudoCoef1 = pd_lambdaCoef / wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal);
      pd_pseudoCoef2 = pd_lambdaCoef / wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal);
      f_code = (float)((pz_signal1->z_codeBias.q_corr * pd_pseudoCoef1 + pz_signal2->z_codeBias.q_corr * pd_pseudoCoef2
        - (pz_signal1_ref->z_codeBias.q_corr * pd_pseudoCoef1 + pz_signal2_ref->z_codeBias.q_corr * pd_pseudoCoef2)) * 0.001);
    }
    f_minlock = minLockDecide(NULL, pz_satMeas->u_constellation, pz_corrStat->u_IonoCorrStatus & PPP_SSR_CORRECTED, pz_satMeas->z_satPosVelClk.f_elevation);
    if (tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime) < f_minlock ||
      tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime) < f_minlock)
    {
      c_flag = 'L'; u_sat_flag = 2;  /* lock is not enough */
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < 15.0 * DEG2RAD|| pz_satMeas->pz_signalMeas[z_freqType1]->f_cn0<25.0 || pz_satMeas->pz_signalMeas[z_freqType2]->f_cn0<25.0)
    {
      c_flag = 'E'; u_sat_flag = 3; /* elevation is too low */
    }
    if (C_GNSS_FREQ_TYPE_L1 == z_freqType1)
    {
      f_mw_amb = (float)((pz_satMeas->f_mw[z_freqType2] - pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refsat_idx]->f_mw[z_freqType2]) + f_upd - f_code);
    }
    else if (C_GNSS_FREQ_TYPE_L2 == z_freqType1 && C_GNSS_FREQ_TYPE_L5 == z_freqType2)
    {
      f_mw_amb = (float)(((pz_satMeas->f_mw[z_freqType2] - pz_satMeas->f_mw[z_freqType1]) -
        (pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refsat_idx]->f_mw[z_freqType2] -
          pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refsat_idx]->f_mw[z_freqType1])) + f_upd - f_code);
    }
    f_wl_amb = (float)(pz_fixedAmbPool->pd_x_fix[pz_satPool1->w_index] - pz_fixedAmbPool->pd_x_fix[pz_satPool2->w_index]
      - (pz_fixedAmbPool->pd_x_fix[pz_satPool1_ref->w_index] - pz_fixedAmbPool->pd_x_fix[pz_satPool2_ref->w_index]) + f_upd);

    f_rate_s1 = computeSuccess(pz_fixedAmbPool->pd_q_fix, pz_satPool1->w_index, pz_satPool1_ref->w_index, pz_satPool2->w_index, pz_satPool2_ref->w_index, floor(f_wl_amb), f_wl_amb);
    f_rate_s2 = computeSuccess(pz_fixedAmbPool->pd_q_fix, pz_satPool1->w_index, pz_satPool1_ref->w_index, pz_satPool2->w_index, pz_satPool2_ref->w_index, ceil(f_wl_amb), f_wl_amb);
    if (f_rate_s1 < MIN_SUC_RATE && f_rate_s2 < MIN_SUC_RATE)
    {
      c_flag = 'R'; u_sat_flag = 4; /* int_rate is not enough */
    }
    if (fabs(round(f_mw_amb) - round(f_wl_amb)) >= 5.0 && tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime) > 10.0)
    {
      c_flag = 'W'; u_sat_flag = 5;  /* large difference between  MW and WL */
    }
    f_dfrac = (float)(f_wl_amb - round(f_wl_amb));
    if (fabs(f_dfrac) > 0.4&& pz_satMeas->z_satPosVelClk.f_elevation < 25.0 * DEG2RAD)
    {
      c_flag = 'F'; u_sat_flag = 6;
    }
    if (fabs(*pz_corrStat->pf_phaseResidual[z_freqType1])>0.05f|| fabs(*pz_corrStat->pf_phaseResidual[z_freqType2]) > 0.05f
      ||fabs(pz_corrStat->f_ionResidual)>0.15f)
    {
      c_flag = 'V'; u_sat_flag = 7;
    }

    svid_SatString(pz_satMeas->u_svid, pz_satMeas->u_constellation, char_sat);
    svid_SatString(u_refsat_svid, pz_satMeas->u_constellation, char_refsat);
    LOGI(TAG_PPP, "WL %3s--%3s   ele=%4.1lf lock_t=%7.1lf mw=%10.2lf wl=%10.2lf dfrac=%6.2lf upd=%7.3lf %c\n",
      char_sat, char_refsat, pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime),
      f_mw_amb, f_wl_amb, f_dfrac, f_upd, c_flag);

    u_ns_all++;
    if (u_sat_flag || u_ns >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      continue;
    }
    pu_satlist[u_ns++] = u_sat_idx + 1;
    u_sys_ns[pz_satMeas->u_constellation]++;
  }

  /* print log */
  if (log_GetLogLevel() >= LOG_LEVEL_I)
  {
    p += snprintf(p, 12, "Fix WL num:");
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
    {
      p += snprintf(p, 4, " %02d", u_sys_ns[u_i]);
    }
    p += snprintf(p, 3, "\n");
    LOGI(TAG_PPP, c_buff);
  }

  if (u_ns_all <= 0 || ((u_ns * 1.0 / u_ns_all) < 0.5&& u_ns<7))
  {
    LOGI(TAG_PPP, "Too much bad wl ns=%d ns_all=%d\n", u_ns, u_ns_all);
    return 0;
  }
  return u_ns;
}

/**
 * @brief get X and Q of parameter
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @param[in]    ns
 * @param[out]   pd_X
 * @param[out]   pd_Q
 * @param[in]    pu_satlist
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @return status - 0: success, other:fail
 */
static uint8_t getFloatXQ_WL(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool
  , uint8_t ns, double* pd_X, double* pd_Q, uint8_t* pu_satlist, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_sys = 0;
  uint8_t u_sat_idx = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_refsat_svid = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  uint16_t uw_i1 = 0;
  uint16_t uw_i2 = 0;
  uint16_t uw_ri1 = 0;
  uint16_t uw_ri2 = 0;
  uint16_t uw_j1 = 0;
  uint16_t uw_j2 = 0;
  uint16_t uw_rj1 = 0;
  uint16_t uw_rj2 = 0;
  float f_upd = 0.0;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal2 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal2_ref = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  for (u_i = 0; u_i < ns; u_i++)
  {
    if (pu_satlist[u_i] < 1)
    {
      u_status = 1;
      return u_status;
    }
    u_sat_idx = pu_satlist[u_i] - 1;

    /* get ekf status of sat and refsat */
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    w_x_id[0] =  0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);

    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    w_x_id[0] = 0;
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
    w_x_id[2] = z_filterType1;
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPool1_ref || NULL == pz_satPool2_ref || NULL == pz_satMeas)
    {
      u_status = 2;
      return u_status;
    }
    
    /* get ssr bias of sat and refsat */
    pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal2 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
    pz_signal2_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (NULL == pz_signal1 || NULL == pz_signal2 || NULL == pz_signal1_ref || NULL == pz_signal2_ref)
    {
      u_status = 3;
      return u_status;
    }
    else
    {
      f_upd = (float)((pz_signal1->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal) - pz_signal2->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal)
        - (pz_signal1_ref->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal) - pz_signal2_ref->z_phaseBias.q_corr/ wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal))) * 0.001);
    }

    uw_i1 = pz_satPool1->w_index;
    uw_i2 = pz_satPool2->w_index;
    uw_ri1 = pz_satPool1_ref->w_index;
    uw_ri2 = pz_satPool2_ref->w_index;
    pd_X[u_i] = pz_fixedAmbPool->pd_x_fix[uw_i1] - pz_fixedAmbPool->pd_x_fix[uw_i2]
      - (pz_fixedAmbPool->pd_x_fix[uw_ri1] - pz_fixedAmbPool->pd_x_fix[uw_ri2]) + f_upd;
    for (u_j = 0; u_j < ns; u_j++)
    {
      if (pu_satlist[u_j] < 1)
      {
        u_status = 4;
        return u_status;
      }
      u_sat_idx = pu_satlist[u_j] - 1;
      /* get ekf status of sat and refsat */
      pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
      gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
      w_x_id[0] = 0;
      w_x_id[1] = u_sat_idx;
      w_x_id[2] = z_filterType1;
      pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
      w_x_id[2] = z_filterType2;
      pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);

      u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
      w_x_id[0] = 0;
      w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
      w_x_id[2] = z_filterType1;
      pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
      w_x_id[2] = z_filterType2;
      pz_satPool2_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
      if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPool1_ref || NULL == pz_satPool2_ref || NULL == pz_satMeas)
      {
        u_status = 5;
        return u_status;
      }
      /* get ssr bias of sat and refsat */
      pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
      pz_signal2 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
      pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
      pz_signal2_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
      if (NULL == pz_signal1 || NULL == pz_signal2 || NULL == pz_signal1_ref || NULL == pz_signal2_ref)
      {
        u_status = 6;
        return u_status;
      }

      uw_j1 = pz_satPool1->w_index;
      uw_j2 = pz_satPool2->w_index;
      uw_rj1 = pz_satPool1_ref->w_index;
      uw_rj2 = pz_satPool2_ref->w_index;

      pd_Q[u_j + u_i * ns] = pz_fixedAmbPool->pd_q_fix[IUTM(uw_i1,uw_j1)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri1,uw_rj1)];
      pd_Q[u_j + u_i * ns] -= pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri1,uw_j1)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_i1,uw_rj1)];

      pd_Q[u_j + u_i * ns] += pz_fixedAmbPool->pd_q_fix[IUTM(uw_i2,uw_j2)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri2,uw_rj2)];
      pd_Q[u_j + u_i * ns] -= pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri2,uw_j2)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_i2,uw_rj2)];


      pd_Q[u_j + u_i * ns] += pz_fixedAmbPool->pd_q_fix[IUTM(uw_i1,uw_rj2)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri2,uw_j1)];
      pd_Q[u_j + u_i * ns] -= pz_fixedAmbPool->pd_q_fix[IUTM(uw_i1,uw_j2)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_i2,uw_j1)];

      pd_Q[u_j + u_i * ns] += pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri1,uw_j2)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_i2,uw_rj1)];
      pd_Q[u_j + u_i * ns] -= pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri2,uw_rj1)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri1,uw_rj2)];


      pd_Q[u_i + u_j * ns] = pd_Q[u_j + u_i * ns];
    }
  }
  return u_status;
}

/**
 * @brief recheck result of widelane ambiguity
 * @param[in]    u_ns
 * @param[in/out]    pu_satlist
 * @param[in]    pd_X
 * @param[in]    pd_Q
 * @param[in]    pd_amb_int
 * @param[in]    max_dv
 * @param[in]    max_frac
 * @param[in]    min_rate
 * @return status - 0: success, other:fail
 */
static uint8_t recheckAmb(uint8_t u_ns, uint8_t* pu_satlist, const double* pd_X, const double* pd_Q, const double* pd_amb_int
  , uint8_t u_all_ns, float max_dv, float max_frac, float min_rate)
{
  uint8_t u_n = 0;
  uint8_t u_i = 0;
  char char_sat[4] = "";
  float f_dv = 0.0;
  float f_dfrac = 0.0;
  float f_int_rate = 0.0;

  for (u_i = 0; u_i < u_ns; u_i++)
  {
    f_dv = (float)(pd_amb_int[u_i] - pd_X[u_i]);
    f_dfrac = (float)(pd_amb_int[u_i] - round(pd_amb_int[u_i]));
    f_int_rate = (float)confidence(pd_amb_int[u_i], pd_X[u_i], sqrt(pd_Q[u_i * (u_ns + 1)]));

    if ((fabs(f_dv) > max_dv || fabs(f_dfrac) > max_frac) || f_int_rate < min_rate)
    {
      if (u_ns <= 4) return 0;
      satidx_SatString(pu_satlist[u_i] - 1, char_sat);
      LOGW(TAG_PPP, "recheckAmb delete sat=%3s dv=%4.2f dfrac=%4.2f int_rate=%5.2lf\n", char_sat, f_dv, f_dfrac, f_int_rate);
      pu_satlist[u_i] = 0;
      continue;
    }
    u_n++;
  }
  u_all_ns = (u_all_ns > 10 ? 10 : u_all_ns);
  if (((1.0 * u_n) / u_all_ns) < 0.6 && u_n < 5)
  {
    LOGW(TAG_PPP, "Too much unhealth amb,n=%d ns=%d\n", u_n, u_all_ns);
    return 0;
  }

  return u_n;
}

/**
 * @brief set result information of AR
 * @param[in]    u_fix_type
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    u_ns
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @param[in]    pu_satlist
 * @param[in]    pd_amb_int
 * @param[out]    pz_fixedAmbPool
 * @return status - 0: success, other:fail
 */
static void setResultAR(const gnss_fixedAmbType u_fix_type, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, uint8_t u_ns, gnss_FreqType z_freqType1,
  gnss_FreqType z_freqType2, const uint8_t* pu_satlist, const double* pd_amb_int, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_svid = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_refsvid = 0;
  uint8_t u_refidx = 0;
  uint8_t u_sys = 0;
  uint8_t u_sat_idx = 0;

  gnss_fixedSignalAmb_t* pz_fix_amb = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  pz_fixedAmbPool->w_continuefixedEpoch++;
  for (u_i = 0; u_i < u_ns; u_i++)
  {
    if (pu_satlist[u_i] <= 0)
    {
      continue;
    }
    u_sat_idx = pu_satlist[u_i] - 1;
    pz_fix_amb = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx];
    if (NULL == pz_fix_amb)
    {
      continue;
    }
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    if (NULL == pz_satMeas || NULL == pz_satMeas->pz_signalMeas[z_freqType1]|| NULL == pz_satMeas->pz_signalMeas[z_freqType2])
    {
      continue;
    }
    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    u_refsvid = pz_fixedAmbPool->pw_refSat[MAX_GNSS_SIGNAL_FREQ * u_constellation + z_freqType2];
    u_refidx = gnss_cvt_Svid2SvIndex(u_refsvid, u_sys);
    pz_fixedAmbPool->u_fixStatus |= u_fix_type;

    if (NULL != pd_amb_int) pz_fix_amb->d_fixedValue = pd_amb_int[u_i];
    pz_fix_amb->u_constellation = u_constellation;
    pz_fix_amb->u_svid = u_svid;
    pz_fix_amb->u_continue_fix[z_freqType2]++;
    pz_fix_amb->u_freqType[z_freqType1] = pz_satMeas->pz_signalMeas[z_freqType1]->u_signal;
    pz_fix_amb->u_freqType[z_freqType2] = pz_satMeas->pz_signalMeas[z_freqType2]->u_signal;
    pz_fix_amb->u_ambFixType |= u_fix_type;

    pz_fixedAmbPool->u_fixedns[u_constellation]++;
    pz_fixedAmbPool->u_nb++;
    if (1 == pz_fixedAmbPool->u_fixedns[u_constellation] &&NULL!= pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]) /* reference satellite */
    {
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->d_fixedValue = 0.0;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_continue_fix[z_freqType2]++;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_freqType[z_freqType1] = pz_satMeas->pz_signalMeas[z_freqType1]->u_signal;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_freqType[z_freqType2] = pz_satMeas->pz_signalMeas[z_freqType2]->u_signal;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_ambFixType |= u_fix_type;
    }
  }

  pz_fixedAmbPool->u_fixedTotalSatCount = 0;
  for (u_i = 0; u_i < C_GNSS_MAX; u_i++)
  {
    pz_fixedAmbPool->u_fixedTotalSatCount += pz_fixedAmbPool->u_fixedns[u_i];
  }
}

/**
 * @brief get list of satellites preliminarily
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @param[out]    pu_satlist
 * @param[out]    pd_amb_int
 * @return status - 0: success, other:fail
 */
static uint8_t widelaneRound(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool
  , gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t* pu_satlist, double* pd_amb_int)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_ns = 0;
  uint8_t u_ns_all = 0;
  uint8_t u_sat_flag = 0;
  uint8_t u_sys_ns[C_GNSS_MAX] = { 0 };
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_refsat_svid = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_refsat_idx = 0;
  float f_minlock = 0.0;
  float f_upd = 0.0;
  float f_code = 0.0;
  float f_wl_amb = 0.0;
  float f_mw_amb = 0.0;
  float f_dfrac = 0.0;
  float f_dfracMean = 0.0;
  float f_rate_s1 = 0.0;
  float f_rate_s2 = 0.0;
  double pd_lambdaCoef = 0.0;
  double pd_pseudoCoef1 = 0.0;
  double pd_pseudoCoef2 = 0.0;
  char c_flag = ' ';
  char char_sat[4] = "";
  char char_refsat[4] = "";
  char c_buff[64] = "";
  char* p = c_buff;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const GpsTime_t* pz_curtime = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal2 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal2_ref = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  /* check loop all of satellites */
  pz_curtime = &pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor;
  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    c_flag = 'O';
    u_sat_flag = 0;
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    pz_corrStat = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_sat_idx];
    if (NULL == pz_satMeas || NULL == pz_corrStat)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if ((NULL == pz_satLos) || pz_satLos->z_orbClk.f_qi * 0.001 > PPPAR_MAX_LOS_QI
      || !(GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID & pz_satLos->u_windUpMask))
    {
      continue;
    }
    if (NULL == pz_satMeas->pz_signalMeas[z_freqType1] || NULL == pz_satMeas->pz_signalMeas[z_freqType2]
      || pz_satMeas->pz_signalMeas[z_freqType1]->u_LLI & 2 || pz_satMeas->pz_signalMeas[z_freqType2]->u_LLI & 2)
    {
      continue;
    }
    if (NULL == pz_corrStat->z_measUpdateFlag[z_freqType1] || NULL == pz_corrStat->z_measUpdateFlag[z_freqType2]
      || !pz_corrStat->z_measUpdateFlag[z_freqType1]->b_cpValid || !pz_corrStat->z_measUpdateFlag[z_freqType2]->b_cpValid
      || !pz_corrStat->pf_phaseResidual[z_freqType1] || !pz_corrStat->pf_phaseResidual[z_freqType2])
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    if (pz_satMeas->u_svid == u_refsat_svid || u_refsat_svid <= 0)  /* the same as refsat */
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid)
      || C_SAT_TYPE_IGSO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid)
      || (pz_satMeas->u_constellation == C_GNSS_BDS2))  /* do not select GEO */
    {
      continue;
    }

    /* get status from EKF */
    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[0] = 0;
    u_refsat_idx = gnss_cvt_Svid2SvIndex(u_refsat_svid, pz_satMeas->u_constellation);;
    w_x_id[1] = u_refsat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPool1_ref || NULL == pz_satPool2_ref)
    {
      continue;
    }

    if (fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool1->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool2->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool1_ref->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool2_ref->z_endTime)) > 1e-4) /* unhealth,current epoch */
    {
      continue;
    }

    /* get code bias and phase bias */
    pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal2 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType1]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
    pz_signal2_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType2]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (NULL == pz_signal1 || NULL == pz_signal2 || NULL == pz_signal1_ref || NULL == pz_signal2_ref
      || !(pz_signal1->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR) || !(pz_signal2->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR)
      || !(pz_signal1_ref->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR) || !(pz_signal2_ref->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
    {
      c_flag = 'P'; u_sat_flag = 1; f_upd = 0.0; f_code = 0.0;  /* bias is unhealty */
    }
    else
    {
      f_upd = (float)((pz_signal1->z_phaseBias.q_corr / wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal) - pz_signal2->z_phaseBias.q_corr / wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal)
        - (pz_signal1_ref->z_phaseBias.q_corr / wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal) - pz_signal2_ref->z_phaseBias.q_corr / wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal))) * 0.001);

      pd_lambdaCoef = (wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal) - wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal)) /
        (wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal) + wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal));
      pd_pseudoCoef1 = pd_lambdaCoef / wavelength(pz_satMeas->pz_signalMeas[z_freqType1]->u_signal);
      pd_pseudoCoef2 = pd_lambdaCoef / wavelength(pz_satMeas->pz_signalMeas[z_freqType2]->u_signal);
      f_code = (float)((pz_signal1->z_codeBias.q_corr * pd_pseudoCoef1 + pz_signal2->z_codeBias.q_corr * pd_pseudoCoef2
        - (pz_signal1_ref->z_codeBias.q_corr * pd_pseudoCoef1 + pz_signal2_ref->z_codeBias.q_corr * pd_pseudoCoef2)) * 0.001);
    }
    f_minlock = minLockDecide(NULL, pz_satMeas->u_constellation, pz_corrStat->u_IonoCorrStatus & PPP_SSR_CORRECTED, pz_satMeas->z_satPosVelClk.f_elevation);
    if (tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime) < f_minlock ||
      tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime) < f_minlock)
    {
      c_flag = 'L'; u_sat_flag = 2;  /* lock is not enough */
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < 15.0 * DEG2RAD || pz_satMeas->pz_signalMeas[z_freqType1]->f_cn0 < 25.0 || pz_satMeas->pz_signalMeas[z_freqType2]->f_cn0 < 25.0)
    {
      c_flag = 'E'; u_sat_flag = 3; /* elevation is too low */
    }
    if (C_GNSS_FREQ_TYPE_L1 == z_freqType1)
    {
      f_mw_amb = (float)((pz_satMeas->f_mw[z_freqType2] - pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refsat_idx]->f_mw[z_freqType2]) + f_upd - f_code);
    }
    else if (C_GNSS_FREQ_TYPE_L2 == z_freqType1 && C_GNSS_FREQ_TYPE_L5 == z_freqType2)
    {
      f_mw_amb = (float)(((pz_satMeas->f_mw[z_freqType2] - pz_satMeas->f_mw[z_freqType1]) -
        (pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refsat_idx]->f_mw[z_freqType2] -
          pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refsat_idx]->f_mw[z_freqType1])) + f_upd - f_code);
    }
    f_wl_amb = (float)(pz_fixedAmbPool->pd_x_fix[pz_satPool1->w_index] - pz_fixedAmbPool->pd_x_fix[pz_satPool2->w_index]
      - (pz_fixedAmbPool->pd_x_fix[pz_satPool1_ref->w_index] - pz_fixedAmbPool->pd_x_fix[pz_satPool2_ref->w_index]) + f_upd);

    f_rate_s1 = computeSuccess(pz_fixedAmbPool->pd_q_fix, pz_satPool1->w_index, pz_satPool1_ref->w_index, pz_satPool2->w_index, pz_satPool2_ref->w_index, floor(f_wl_amb), f_wl_amb);
    f_rate_s2 = computeSuccess(pz_fixedAmbPool->pd_q_fix, pz_satPool1->w_index, pz_satPool1_ref->w_index, pz_satPool2->w_index, pz_satPool2_ref->w_index, ceil(f_wl_amb), f_wl_amb);
    if (f_rate_s1 < MIN_SUC_RATE && f_rate_s2 < MIN_SUC_RATE)
    {
      c_flag = 'R'; u_sat_flag = 4; /* int_rate is not enough */
    }
    if (fabs(round(f_mw_amb) - round(f_wl_amb)) >= 5.0 && tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime) > 10.0)
    {
      c_flag = 'W'; u_sat_flag = 5;  /* large difference between  MW and WL */
    }
    f_dfrac = (float)(f_wl_amb - round(f_wl_amb));
    if (fabs(f_dfrac) > 0.275)
    {
      c_flag = 'F'; u_sat_flag = 6;
    }
    if (fabs(*pz_corrStat->pf_phaseResidual[z_freqType1]) > 0.05f || fabs(*pz_corrStat->pf_phaseResidual[z_freqType2]) > 0.05f
      || fabs(pz_corrStat->f_ionResidual) > 0.15f)
    {
      c_flag = 'V'; u_sat_flag = 7;
    }

    svid_SatString(pz_satMeas->u_svid, pz_satMeas->u_constellation, char_sat);
    svid_SatString(u_refsat_svid, pz_satMeas->u_constellation, char_refsat);
    LOGI(TAG_PPP, "RoundWL %3s--%3s   ele=%4.1lf lock_t=%7.1lf mw=%10.2lf wl=%10.2lf dfrac=%6.2lf upd=%7.3lf %c\n",
      char_sat, char_refsat, pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime),
      f_mw_amb, f_wl_amb, f_dfrac, f_upd, c_flag);

    u_ns_all++;
    if (u_sat_flag || u_ns >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      continue;
    }
    f_dfracMean += (float)fabs(f_dfrac);
    pu_satlist[u_ns] = u_sat_idx + 1;
    pd_amb_int[u_ns++] = round(f_wl_amb);
    u_sys_ns[pz_satMeas->u_constellation]++;
  }

  /* print log */
  if (log_GetLogLevel() >= LOG_LEVEL_I)
  {
    p += snprintf(p, 12, "Fix WL num:");
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
    {
      p += snprintf(p, 4, " %02d", u_sys_ns[u_i]);
    }
    p += snprintf(p, 3, "\n");
    LOGI(TAG_PPP, c_buff);
  }
  if (u_ns > 3)
  {
    f_dfracMean = f_dfracMean / u_ns;
    if (f_dfracMean < 0.05) pz_fixedAmbPool->f_ratio = 5.0;
    if (f_dfracMean < 0.1) pz_fixedAmbPool->f_ratio = 3.0;
    else if (f_dfracMean < 0.2) pz_fixedAmbPool->f_ratio = 1.5;
    LOGI(TAG_PPP, "Round_wl ns=%d f_dfracMean=%.2lf f_ratio=%.1lf\n", u_ns, f_dfracMean, pz_fixedAmbPool->f_ratio);
  }
  return u_ns;
}

/**
 * @brief wide-lane ambiguity resolution
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @param[in]    MINAR
 * @return status - 0: success, other:fail
 */
static uint8_t widelaneArRound(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t MINAR)
{
  uint8_t u_status = AMB_AR_FAIL_UnknowError;
  uint8_t u_ns = 0;

  uint8_t* pu_satlist = NULL;
  double* pd_amb_int = NULL;
  gnss_fixedAmbType u_fix_type;

  pu_satlist = OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(uint8_t));
  pd_amb_int = OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(double));
  if (NULL == pu_satlist || NULL == pd_amb_int)
  {
    u_status = AMB_AR_FAIL_MemError;
    OS_FREE(pu_satlist);
    OS_FREE(pd_amb_int);
    return u_status;
  }

  /* get list of satellites preliminarily */
  u_ns = widelaneRound(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2, pu_satlist, pd_amb_int);
  if (u_ns > MINAR)
  {
    /* set result information of AR */
    u_fix_type = convertFreqType2AmbType(z_freqType1, z_freqType2);
    setResultAR(u_fix_type, pz_pppAmbFixInputInfo, u_ns, z_freqType1, z_freqType2, pu_satlist, pd_amb_int, pz_fixedAmbPool);
    u_status = AMB_AR_SUCCESS;
  }

  OS_FREE(pu_satlist);
  OS_FREE(pd_amb_int);

  return u_status;
}

/**
 * @brief wide-lane ambiguity resolution
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]    pz_fixedAmbPool
 * @param[in]    z_freqType1
 * @param[in]    z_freqType2
 * @param[in]    MINAR
 * @return status - 0: success, other:fail
 */
static uint8_t widelaneArLambda(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t MINAR)
{
  const uint8_t u_candidate = 20;
  uint8_t u_status = AMB_AR_FAIL_UnknowError;
  uint8_t u_i = 0;
  uint8_t u_ns = 0;
  uint8_t u_nwl = 0;

  uint8_t* pu_satlist = NULL;
  float f_boot_rate = 0.0;
  double* pd_X = NULL;
  double* pd_Q = NULL;
  double* pd_amb_int = NULL;
  gnss_fixedAmbType u_fix_type;

  pu_satlist = OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(uint8_t));
  if (NULL == pu_satlist)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }

  /* get list of satellites preliminarily */
  u_ns = getWlSatlist(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2, pu_satlist);
  pz_fixedAmbPool->u_ppp_use_sat_num_wideLane = u_ns;

  while (u_ns > MINAR)
  {
    pd_X = OS_MALLOC_FAST(u_ns * sizeof(double));
    pd_Q = OS_MALLOC_FAST(u_ns * u_ns * sizeof(double));
    pd_amb_int = OS_MALLOC_FAST(u_ns * u_candidate * sizeof(double));
    if (NULL == pd_X || NULL == pd_Q || NULL == pd_amb_int)
    {
      u_status = AMB_AR_FAIL_MemError;
      break;
    }

    /* get X and Q of parameter*/
    if (getFloatXQ_WL(pz_pppAmbFixInputInfo, pz_fixedAmbPool, u_ns, pd_X, pd_Q, pu_satlist, z_freqType1, z_freqType2))
    {
      u_status = AMB_AR_FAIL_DataError;
      break;
    }

    /* partial ambiguity resolution */
    while (TRUE)
    {
      if (!(u_status = parAmbResolution(PPP_PAR_AR_METHOD_COV, PPP_AMB_FIX_DELETE_NONE_SYS, u_ns, pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, &u_nwl, pu_satlist,
        pd_X, pd_Q, pd_amb_int, 3, 1.0, 0.1f, 0.4f)))
      {
        break;
      }
      if (!(u_status = parAmbResolution(PPP_PAR_AR_METHOD_ADOP, PPP_AMB_FIX_DELETE_NONE_SYS, u_ns, pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, &u_nwl, pu_satlist,
        pd_X, pd_Q, pd_amb_int, 3, 1.0, 0.1f, 0.4f)))
      {
        break;
      }
      /* delete BDS */
      u_status = parAmbResolution(PPP_PAR_AR_METHOD_COV, PPP_AMB_FIX_DELETE_BDS2_SYS | PPP_AMB_FIX_DELETE_BDS3_SYS, u_ns, pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, &u_nwl, pu_satlist,
        pd_X, pd_Q, pd_amb_int, MIN_SAT_AR, 1.0, 0.1f, 0.4f);
      break;
    }

    if (u_status)
    {
      LOGI(TAG_PPP, "WL parAmbResolution fail,status=%d\n", u_status);
      u_status = AMB_AR_FAIL_ParFail;
      break;
    }

    /* recheck result of widelane ambiguity */
    if (recheckAmb(u_nwl, pu_satlist, pd_X, pd_Q, pd_amb_int, u_ns, 1.0f, 0.2f, (float)MIN_SUC_RATE) > MINAR)
    {
      /* set result information of AR */
      u_fix_type = convertFreqType2AmbType(z_freqType1, z_freqType2);
      setResultAR(u_fix_type, pz_pppAmbFixInputInfo, u_nwl, z_freqType1, z_freqType2, pu_satlist, pd_amb_int, pz_fixedAmbPool);
      u_status = AMB_AR_SUCCESS;
      break;
    }
    else
    {
      u_status = AMB_AR_FAIL_LessNS;
      break;
    }
  }

  OS_FREE(pu_satlist);
  pu_satlist = NULL;
  if (NULL != pd_X)
  {
    OS_FREE(pd_X);
    pd_X = NULL;
  }
  if (NULL != pd_Q)
  {
    OS_FREE(pd_Q);
    pd_Q = NULL;
  }
  if (NULL != pd_amb_int)
  {
    OS_FREE(pd_amb_int);
    pd_amb_int = NULL;
  }

  return u_status;
}
/**
 * @brief check accuracy of float position
 * @param[in]    pz_pppAmbFixInputInfo
 * @return status - 0: OK, other:not OK
 */
static uint8_t floatForArCheck(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_stat = 0;
  double d_enu[3] = { 0.0 };
  double d_var[3] = { 0.0 };
  double d_xyz[3] = { 0.0 };
  double d_pos[3] = { 0.0 };
  double d_P[9] = { 0.0 };
  double d_Q[9] = { 0.0 };
  double d_horVel = 0.0;
  double d_enu_vel[3] = { 0.0 };

  if (pz_pppAmbFixInputInfo->pz_FilterObs->f_postCodeSTD > 4.0
    || pz_pppAmbFixInputInfo->pz_FilterObs->f_postPhaseSTD > 0.04)
  {
    return 1;
  }
  getEnuVel(pz_pppAmbFixInputInfo->pz_EKFstateRepPool, pz_pppAmbFixInputInfo->pz_pppFilterInfo->pd_X, d_enu_vel);
  d_horVel = sqrt(d_enu_vel[0] * d_enu_vel[0] + d_enu_vel[1] * d_enu_vel[1]);
  if (d_horVel > 60 || fabs(d_enu_vel[2]) > 3.0)
  {
    LOGW(TAG_PPP, "Float velocity is not benefit for AR: H=%.3lf   V=%.3lf\n", d_horVel, d_enu_vel[2]);
    return 2;
  }

  for (u_i = 0; u_i < 3; u_i++)
  {
    d_xyz[u_i] = pz_fixedAmbPool->pd_x_fix[u_i];
    d_var[u_i] = pz_fixedAmbPool->pd_q_fix[IUTM(u_i, u_i)];
  }
  gnss_Ecef2Lla(d_xyz, d_pos);

  for (u_i = 0; u_i < 9; u_i++) d_P[u_i] = 0.0;
  d_P[0 + 0 * 3] = d_var[0];
  d_P[1 + 1 * 3] = d_var[1];
  d_P[2 + 2 * 3] = d_var[2];
  gnss_CovEcef2Enu(d_pos, d_P, d_Q);

  d_enu[0] = sqrt(d_Q[0]);
  d_enu[1] = sqrt(d_Q[4]);
  d_enu[2] = sqrt(d_Q[8]);

  if (d_enu[0] > 0.8 || d_enu[1] > 0.8 || d_enu[2] > 0.8)
  {
    LOGW(TAG_PPP, "Float Accuracy is not OK!  e=%10.4lf,n=%10.4lf,u=%10.4lf,time=%d\n",
      d_enu[0], d_enu[1], d_enu[2], pz_pppAmbFixInputInfo->pz_EKFstateRepPool->d_continueFilterTime);
    u_stat = 3;
  }
  return u_stat;
}

/**
 * @brief get satlist of narrow-lane amb
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]   pz_fixedAmbPool
 * @param[in]    z_freqType
 * @param[out]    pu_satlist
 * @return u_ns -- number of satlist
 */
static uint8_t getNlSatlist(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType z_freqType, uint8_t* pu_satlist)
{
  uint8_t u_ns = 0;
  uint8_t u_ns_all = 0;
  uint8_t u_svid = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_isys = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_sat_flag = 0;
  uint8_t u_refsat_svid = 0;
  uint16_t w_x_id[3] = { 0 };
  char c_flag = ' ';
  char char_sat[4] = "";
  char char_refsat[4] = "";
  float f_minlock = 0.0;
  float f_upd = 0.0;
  float f_amb = 0.0;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const GpsTime_t* pz_curtime = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;
  const gnss_fixedSignalAmb_t* pz_fixedAmb = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  z_filterType = convertFreqAmb2FilterType(z_freqType);

  /* check loop all of satellites */
  pz_curtime = &pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor;
  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    c_flag = 'O';
    u_sat_flag = 0;
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    pz_corrStat = pz_pppAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_sat_idx];
    if (NULL == pz_satMeas || NULL == pz_corrStat)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if ((NULL == pz_satLos) || pz_satLos->z_orbClk.f_qi * 0.001 > PPPAR_MAX_LOS_QI
      || !(GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID & pz_satLos->u_windUpMask))
    {
      continue;
    }
    if (NULL == pz_corrStat->z_measUpdateFlag[z_freqType] || !pz_corrStat->z_measUpdateFlag[z_freqType]->b_cpValid
      || !pz_corrStat->pf_phaseResidual[z_freqType])
    {
      continue;
    }
    if (NULL == pz_satMeas->pz_signalMeas[z_freqType] || pz_satMeas->pz_signalMeas[z_freqType]->u_LLI&2)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType];
    if (pz_satMeas->u_svid == u_refsat_svid || u_refsat_svid <= 0)  /* the same as refsat */
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))  /* do not select GEO */
    {
      continue;
    }

    /* get status from EKF */
    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, pz_satMeas->u_constellation);
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool1_ref)
    {
      continue;
    }

    if (fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool1->z_endTime)) > 1e-4 ||
      fabs(tm_GpsTimeDiff(pz_curtime, &pz_satPool1_ref->z_endTime)) > 1e-4) /* unhealth,current epoch */
    {
      continue;
    }
    pz_fixedAmb = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx];
    if (NULL == pz_fixedAmb)
    {
      continue;
    }

    /* get code bias and phase bias */
    pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
    if (NULL == pz_signal1 || NULL == pz_signal1_ref
      || !(pz_signal1->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR)
      || !(pz_signal1_ref->u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
    {
      f_upd = 0.0; c_flag = 'P'; u_sat_flag = 1;  /* bias is unhealty */
    }
    else
    {
      f_upd = (float)((pz_signal1->z_phaseBias.q_corr - pz_signal1_ref->z_phaseBias.q_corr) * 0.001);
      f_upd = (float)(f_upd / wavelength(pz_satMeas->pz_signalMeas[z_freqType]->u_signal));
    }
    
    f_minlock = minLockDecide(NULL, pz_satMeas->u_constellation, pz_corrStat->u_IonoCorrStatus & PPP_SSR_CORRECTED, pz_satMeas->z_satPosVelClk.f_elevation);
    if (f_minlock < 3.0 && !(pz_fixedAmb->u_ambFixType & GNSS_WL_AMB_FIXED))
    {
      f_minlock = 3.0;
    }
    if (tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime) < f_minlock)
    {
      c_flag = 'L'; u_sat_flag = 2;  /* lock is not enough */
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < 15.0 * DEG2RAD|| pz_satMeas->pz_signalMeas[z_freqType]->f_cn0 < 25.0)
    {
      c_flag = 'E'; u_sat_flag = 3; /* elevation is too low */
    }
    if (fabs(*pz_corrStat->pf_phaseResidual[z_freqType]) > 0.05f|| fabs(pz_corrStat->f_ionResidual) > 0.15f)
    {
      c_flag = 'V'; u_sat_flag = 4;
    }
    f_amb = (float)((pz_fixedAmbPool->pd_x_fix[pz_satPool1->w_index] - pz_fixedAmbPool->pd_x_fix[pz_satPool1_ref->w_index]) + f_upd);
    svid_SatString(pz_satMeas->u_svid, pz_satMeas->u_constellation, char_sat);
    svid_SatString(u_refsat_svid, pz_satMeas->u_constellation, char_refsat);
    LOGI(TAG_PPP, "%3s--%3s   ele=%4.1lf lock_t=%7.1lf amb=%7.2f upd=%6.2f %c\n",
      char_sat, char_refsat, pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime), f_amb, f_upd, c_flag);

    u_ns_all++;
    if (u_sat_flag || u_ns >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      continue;
    }
    pu_satlist[u_ns++] = gnss_cvt_Svid2SvIndex(pz_satMeas->u_svid, pz_satMeas->u_constellation) + 1;
  }
  if (u_ns_all <= 0 || ((u_ns * 1.0 / u_ns_all) < 0.5&& u_ns<7))
  {
    LOGI(TAG_PPP, "Too much bad NL ns=%d ns_all=%d\n", u_ns, u_ns_all);
    return 0;
  }

  return u_ns;
}

/**
 * @brief get X and Q of narrow-lane amb
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[in]   pz_fixedAmbPool
 * @param[in]    pu_satlist
 * @param[in]    u_freq
 * @param[in]    u_ns
 * @param[out]    pd_X
 * @param[out]    pd_Q
 * @return 0: OK, other: fail
 */
static uint8_t getFloatXQ_NL(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool
  , const uint8_t* pu_satlist, gnss_FreqType z_freqType, uint8_t u_ns, double* pd_X, double* pd_Q)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_sys = 0;
  uint8_t u_sat_idx = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_refsat_svid = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  uint16_t uw_i1 = 0;
  uint16_t uw_ri1 = 0;
  uint16_t uw_j1 = 0;
  uint16_t uw_rj1 = 0;
  float f_upd = 0.0;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;

  z_filterType = convertFreqAmb2FilterType(z_freqType);

  for (u_i = 0; u_i < u_ns; u_i++)
  {
    if (pu_satlist[u_i] < 1)
    {
      u_status = 1;
      return u_status;
    }
    u_sat_idx = pu_satlist[u_i] - 1;

    /* get ekf status of sat and refsat */
    pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);

    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType];
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool1_ref || NULL == pz_satMeas
      || u_sat_idx == gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys))
    {
      u_status = 2;
      return u_status;
    }

    /* get ssr bias of sat and refsat */
    pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
    if (NULL == pz_signal1 || NULL == pz_signal1_ref)
    {
      u_status = 3;
      return u_status;
    }
    else
    {
      f_upd = (float)((pz_signal1->z_phaseBias.q_corr - pz_signal1_ref->z_phaseBias.q_corr) * 0.001);
      f_upd = (float)(f_upd / wavelength(pz_satMeas->pz_signalMeas[z_freqType]->u_signal));
    }

    uw_i1 = pz_satPool1->w_index;
    uw_ri1 = pz_satPool1_ref->w_index;
    pd_X[u_i] = pz_fixedAmbPool->pd_x_fix[uw_i1]
      - pz_fixedAmbPool->pd_x_fix[uw_ri1] + f_upd;
    for (u_j = 0; u_j < u_ns; u_j++)
    {
      if (pu_satlist[u_j] < 1)
      {
        u_status = 4;
        return u_status;
      }
      u_sat_idx = pu_satlist[u_j] - 1;
      /* get ekf status of sat and refsat */
      pz_satMeas = pz_pppAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
      gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
      w_x_id[0] = 0;
      w_x_id[1] = u_sat_idx;
      w_x_id[2] = z_filterType;
      pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);

      u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType];
      w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
      pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
      if (NULL == pz_satPool1 || NULL == pz_satPool1_ref || NULL == pz_satMeas
        || u_sat_idx == gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys))
      {
        u_status = 5;
        return u_status;
      }
      /* get ssr bias of sat and refsat */
      pz_signal1 = getSSR_Bias(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_satMeas->pz_signalMeas[z_freqType]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk);
      pz_signal1_ref = getSSR_Bias(pz_satMeas->u_constellation, u_refsat_svid, pz_satMeas->pz_signalMeas[z_freqType]->u_signal, pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */

      if (NULL == pz_signal1 || NULL == pz_signal1_ref)
      {
        u_status = 6;
        return u_status;
      }

      uw_j1 = pz_satPool1->w_index;
      uw_rj1 = pz_satPool1_ref->w_index;

      pd_Q[u_j + u_i * u_ns] = pz_fixedAmbPool->pd_q_fix[IUTM(uw_i1,uw_j1)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri1,uw_rj1)];

      pd_Q[u_j + u_i * u_ns] -= pz_fixedAmbPool->pd_q_fix[IUTM(uw_ri1,uw_j1)]
        + pz_fixedAmbPool->pd_q_fix[IUTM(uw_i1,uw_rj1)];

      pd_Q[u_i + u_j * u_ns] = pd_Q[u_j + u_i * u_ns];
    }
  }
  return u_status;
}

/**
 * @brief narrow-lane amb fix
 * @param[in]    pz_pppAmbFixInputInfo
 * @param[out]   pz_fixedAmbPool
 * @param[in]    z_freqType
 * @return status -   0: success, other: fail
 */
static uint8_t narrowlaneAR(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType z_freqType)
{
  uint8_t u_candidate = 20;
  uint8_t u_status = AMB_AR_SUCCESS;
  uint8_t u_ns = 0;
  uint8_t u_nl = 0;
  uint8_t* pu_satlist = NULL;
  float f_boot_rate = 0.0;
  double* pd_X = NULL;
  double* pd_Q = NULL;
  double* pd_amb_int = NULL;
  gnss_fixedAmbType z_ambType;

  pu_satlist = OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(uint8_t));
  if (NULL == pu_satlist)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }
  /* get satlist of narrow-lane */
  u_ns = getNlSatlist(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType, pu_satlist);
  pz_fixedAmbPool->u_ppp_use_sat_num_fix = u_ns;
  if (u_ns < MIN_SAT_AR)
  {
    u_status = AMB_AR_FAIL_LessNS;
  }
  while (u_ns >= MIN_SAT_AR)
  {
    pd_X = OS_MALLOC_FAST(u_ns * sizeof(double));
    pd_Q = OS_MALLOC_FAST(u_ns * u_ns * sizeof(double));
    pd_amb_int = OS_MALLOC_FAST(u_ns * u_candidate * sizeof(double));
    if (NULL == pd_X || NULL == pd_Q || NULL == pd_amb_int)
    {
      u_status = AMB_AR_FAIL_MemError;
      break;
    }
    /* get X and Q of parameter*/
    if (getFloatXQ_NL(pz_pppAmbFixInputInfo, pz_fixedAmbPool, pu_satlist, z_freqType, u_ns, pd_X, pd_Q))
    {
      u_status = AMB_AR_FAIL_DataError;
      break;
    }
    /* partial ambiguity resolution */
    while (TRUE)
    {
      if (!(u_status = parAmbResolution(PPP_PAR_AR_METHOD_COV, PPP_AMB_FIX_DELETE_NONE_SYS, u_ns, pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType,
        &u_nl, pu_satlist, pd_X, pd_Q, pd_amb_int, MIN_SAT_AR, 1.0, 0.1f, 0.5f)))
      {
        break;
      }
      if (!(u_status = parAmbResolution(PPP_PAR_AR_METHOD_ADOP, PPP_AMB_FIX_DELETE_NONE_SYS, u_ns, pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType,
        &u_nl, pu_satlist, pd_X, pd_Q, pd_amb_int, MIN_SAT_AR, 1.0, 0.1f, 0.5f)))
      {
        break;
      }
      /* delete BDS */
      u_status = parAmbResolution(PPP_PAR_AR_METHOD_COV, PPP_AMB_FIX_DELETE_BDS2_SYS| PPP_AMB_FIX_DELETE_BDS3_SYS, u_ns, pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType,
        &u_nl, pu_satlist, pd_X, pd_Q, pd_amb_int, MIN_SAT_AR, 1.0, 0.1f, 0.5f);
      break;
    }
    if (u_status)
    {
      LOGI(TAG_PPP, "NL parAmbResolution fail,status=%d\n", u_status);
      u_status = AMB_AR_FAIL_ParFail;
      break;
    }
    break;
  }
  if (AMB_AR_SUCCESS == u_status)
  {
    /* recheck result of narrow-lane ambiguity */
    if (recheckAmb(u_nl, pu_satlist, pd_X, pd_Q, pd_amb_int, u_ns, 0.9f, 0.2f, 0.0f) >= MIN_SAT_AR)
    {
      /* set result information of AR */
      z_ambType = gnss_cvt_FreqType2AmbType(z_freqType);
      setResultAR(z_ambType, pz_pppAmbFixInputInfo, u_nl, z_freqType, z_freqType, pu_satlist, pd_amb_int, pz_fixedAmbPool);
      u_status = AMB_AR_SUCCESS;
    }
    else
    {
      u_status = AMB_AR_FAIL_LessNS;
    }
  }

  OS_FREE(pu_satlist);
  pu_satlist = NULL;
  if (NULL != pd_X)
  {
    OS_FREE(pd_X);
    pd_X = NULL;
  }
  if (NULL != pd_Q)
  {
    OS_FREE(pd_Q);
    pd_Q = NULL;
  }
  if (NULL != pd_amb_int)
  {
    OS_FREE(pd_amb_int);
    pd_amb_int = NULL;
  }
  
  return u_status;
}

/**
 * @brief ambiguity fix and hold
 * @param[in]    pz_curfixedAmbPool
 * @param[in]    pz_prefixedAmbPool
 * @param[out]   pz_pppFilterInfo
 * @return none
 */
static void ambFixHold(const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
                       const gnss_fixedSignalAmbPool_t* pz_prefixedAmbPool, ppp_filterInfo_t* pz_pppFilterInfo)
{
  uint8_t ns = 0;
  uint8_t u_i = 0;

  if (NULL == pz_curfixedAmbPool || NULL == pz_prefixedAmbPool || NULL == pz_pppFilterInfo)
  {
    return;
  }

  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
  {
    ns += pz_curfixedAmbPool->u_fixedns[u_i];
  }
  if (pz_curfixedAmbPool->f_ratio > 3.0 && pz_prefixedAmbPool->w_continuefixedEpoch >= 10
    && ns>=6&&pz_curfixedAmbPool->u_fixedns[C_GNSS_GPS]>=2)
  {
    LOGI(TAG_PPP, "Fix and hold working, ns=%d  ratio=%.1lf\n", ns, pz_curfixedAmbPool->f_ratio);
    memcpy(pz_pppFilterInfo->pd_X, pz_curfixedAmbPool->pd_x_fix, pz_pppFilterInfo->w_nmax * sizeof(double));
    memcpy(pz_pppFilterInfo->pd_Q, pz_curfixedAmbPool->pd_q_fix, NUTM(pz_pppFilterInfo->w_nmax) * sizeof(double));
  }

}

static void resetFixedAmbPool(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;

  memset(pz_fixedAmbPool->pw_refSat, 0, MAX_GNSS_SIGNAL_FREQ * C_GNSS_MAX * sizeof(uint8_t));
  memset(pz_fixedAmbPool->u_fixedns, 0, C_GNSS_MAX * sizeof(uint8_t));
  pz_fixedAmbPool->w_continuefixedEpoch = 0;
  pz_fixedAmbPool->u_fixedTotalSatCount = 0;
  pz_fixedAmbPool->f_ratio = 0.0f;
  pz_fixedAmbPool->u_nb = 0;
  pz_fixedAmbPool->u_ppp_use_sat_num_fix = 0;
  pz_fixedAmbPool->u_ppp_use_sat_num_wideLane = 0;

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_i])
    {
      continue;
    }
    pz_fixedAmbPool->pz_fixedAmbSet[u_i]->d_fixedValue = 0.0;
    memset(pz_fixedAmbPool->pz_fixedAmbSet[u_i]->u_continue_fix, 0, MAX_GNSS_SIGNAL_FREQ*sizeof(uint32_t));
    //memset(pz_fixedAmbPool->pz_fixedAmbSet[u_i]->u_freqType, 0, MAX_GNSS_SIGNAL_FREQ * sizeof(gnss_SignalType));
  }
}
/**
 * @brief set pz_prefixedAmbPool from pz_curfixedAmbPool
 * @param[in]    pz_curfixedAmbPool
 * @param[out]    pz_prefixedAmbPool
 * @return none
 */
static void updateFixedAmbPool(const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
  gnss_fixedSignalAmbPool_t** pz_prefixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;

  if (NULL == (*pz_prefixedAmbPool))
  {
    (*pz_prefixedAmbPool) = (gnss_fixedSignalAmbPool_t*)OS_MALLOC_FAST(sizeof(gnss_fixedSignalAmbPool_t));
  }
  if (NULL == (*pz_prefixedAmbPool))
  {
    return;
  }
  
  (*pz_prefixedAmbPool)->z_time = pz_curfixedAmbPool->z_time;
  (*pz_prefixedAmbPool)->u_fixStatus = pz_curfixedAmbPool->u_fixStatus;
  for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ * C_GNSS_MAX; u_i++)
  {
    (*pz_prefixedAmbPool)->pw_refSat[u_i] = pz_curfixedAmbPool->pw_refSat[u_i];
  }
  if (pz_curfixedAmbPool->u_fixStatus == GNSS_NONE_AMB_FIXED)
  {
    (*pz_prefixedAmbPool)->w_continuefixedEpoch = 0;
  }
  else
  {
    (*pz_prefixedAmbPool)->w_continuefixedEpoch++;
  }
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
  {
    (*pz_prefixedAmbPool)->u_fixedns[u_i] = pz_curfixedAmbPool->u_fixedns[u_i];
  }
 
  (*pz_prefixedAmbPool)->u_nb = pz_curfixedAmbPool->u_nb;
  (*pz_prefixedAmbPool)->u_gap = pz_curfixedAmbPool->u_gap;
  (*pz_prefixedAmbPool)->f_ratio = pz_curfixedAmbPool->f_ratio;
  (*pz_prefixedAmbPool)->f_adop = pz_curfixedAmbPool->f_adop;
  (*pz_prefixedAmbPool)->f_pdop = pz_curfixedAmbPool->f_pdop;
  (*pz_prefixedAmbPool)->f_cn0Thres = pz_curfixedAmbPool->f_cn0Thres;
  (*pz_prefixedAmbPool)->f_eleThres = pz_curfixedAmbPool->f_eleThres;
  if (NULL == (*pz_prefixedAmbPool)->pd_x_fix)
  {
    (*pz_prefixedAmbPool)->pd_x_fix= (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  }
  if (NULL != (*pz_prefixedAmbPool)->pd_q_fix)
  {
    OS_FREE((*pz_prefixedAmbPool)->pd_q_fix);
  }
  if (NULL == (*pz_prefixedAmbPool)->pd_x_fix 
    || NULL == pz_curfixedAmbPool->pd_x_fix || NULL == pz_curfixedAmbPool->pd_q_fix)
  {
    return;
  }

  memcpy((*pz_prefixedAmbPool)->pd_x_fix, pz_curfixedAmbPool->pd_x_fix, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL != (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i] && NULL != pz_curfixedAmbPool->pz_fixedAmbSet[u_i])
    {
      for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
      {
        if (pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_continue_fix[u_j] > 0)
        {
          (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_continue_fix[u_j]++;
        }
        else (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_continue_fix[u_j] = 0;
        (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_freqType[u_j] = pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_freqType[u_j];
      }
      (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_ambFixType = pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_ambFixType;
      (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->d_fixedValue = pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->d_fixedValue;
    }
    else if (NULL == (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i] && NULL != pz_curfixedAmbPool->pz_fixedAmbSet[u_i])
    {
      (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i] = (gnss_fixedSignalAmb_t*)OS_MALLOC_FAST(sizeof(gnss_fixedSignalAmb_t));
      if (NULL == (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i])
      {
        continue;
      }
      memcpy((*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i], pz_curfixedAmbPool->pz_fixedAmbSet[u_i], sizeof(gnss_fixedSignalAmb_t));
    }
    else
    {
      OS_FREE((*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]);
    }
  }
}

/**
 * @brief zero-combine ambiguity update
 * @param[in]    z_freqType - the freq of amb
 * @param[in]    pz_pppAmbFixInputInfo - input information for ppp-ar
 * @param[out]   pz_fixedAmbPool - result of ppp-ar
 * @return none
 */
static void zeroCombineAmbUpdate(gnss_FreqType z_freqType, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_refsat_svid = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_sys = 0;
  uint8_t u_svid = 0;
  gnss_ConstellationType u_constellation = 0;
  char c_sat[4] = "";
  float f_upd = 0.0;
  float f_amb = 0.0;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double* pd_deltaX = NULL;
  double* pd_X = NULL;
  uint16_t w_i=0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  SeqKalmanVar_t z_seqVar = { 0 };
  gnss_fixedAmbType z_ambType;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;

  z_ambType = gnss_cvt_FreqType2AmbType(z_freqType);
  if (NULL == pz_fixedAmbPool || !(pz_fixedAmbPool->u_fixStatus & z_ambType))
  {
    return;
  }
  if (hpp_seq_init(&z_seqVar, pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax))
  {
    return;
  }
  pd_X = (double*)OS_MALLOC_FAST(pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax * sizeof(double));
  pd_deltaX = (double*)OS_MALLOC_FAST(pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax * sizeof(double));
  if (NULL == pd_X|| NULL==pd_deltaX)
  {
    if (NULL != pd_X)
    {
      OS_FREE(pd_X);
    }
    if (NULL != pd_deltaX)
    {
      OS_FREE(pd_deltaX);
    }
    return;
  }
  for (w_i = 0; w_i < pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax; w_i++)
  {
    pd_X[w_i] = pz_fixedAmbPool->pd_x_fix[w_i];
  }
  
  z_filterType = convertFreqAmb2FilterType(z_freqType);
  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx])
    {
      continue;
    }
    if (!(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_ambFixType & z_ambType))
    {
      continue;
    }
    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, u_sys);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType];
    if (u_refsat_svid < 0 || u_refsat_svid == u_svid)
    {
      continue;
    }

    /* get status from EKF */
    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[0] = 0;
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
    w_x_id[2] = z_filterType;
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool1_ref)
    {
      continue;
    }
    /* get code bias and phase bias */
    pz_signal1 = getSSR_Bias(u_sys, u_svid, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType], pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(u_sys, u_refsat_svid, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType], pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (NULL == pz_signal1 || NULL == pz_signal1_ref)
    {
      continue;
    }
    f_upd = (float)((pz_signal1->z_phaseBias.q_corr - pz_signal1_ref->z_phaseBias.q_corr) * 0.001);
    f_upd = (float)(f_upd / wavelength(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType]));
    f_amb = (float)(pd_X[pz_satPool1->w_index] - pd_X[pz_satPool1_ref->w_index] + f_upd);

    hpp_seq_ClearH(&z_seqVar);
    /* OMC */
    d_omc = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue - f_amb;
    d_curR = SQR(1.0e-8);

    /* H */
    hpp_seq_AddH(pz_satPool1->w_index, 1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPool1_ref->w_index, -1.0, &z_seqVar);

    hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
    hpp_seq_PredictStep(pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar);
    hpp_seq_measUpdate(pz_pppAmbFixInputInfo->pz_pppFilterInfo->pq_paraValid,pz_fixedAmbPool->pd_x_fix, pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar, 0.0);
    
    /* print log */
    satidx_SatString(u_sat_idx, c_sat);
    LOGI(TAG_PPP, "L%d amb update sat=%s, V=%5.2lf floatX=%7.2lf fixX=%7.2lf dx=%7.3lf\n",
      z_freqType + 1, c_sat, d_omc, f_amb, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue, pd_deltaX[0]);
  }

  hpp_seq_deinit(&z_seqVar);
  OS_FREE(pd_deltaX);
  OS_FREE(pd_X);
}

/**
 * @brief wide-lane ambiguity update
 * @param[in]    z_freqType1 - the first freq of wl
 * @param[in]    z_freqType2 - the second freq of wl
 * @param[in]    pz_pppAmbFixInputInfo - input information for ppp-ar
 * @param[out]   pz_fixedAmbPool - result of ppp-ar
 * @return none
 */
static void wideLaneAmbUpdate(gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_refsat_svid = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_sys = 0;
  uint8_t u_svid = 0;
  gnss_ConstellationType u_constellation = 0;
  char c_sat[4] = "";
  float f_upd = 0.0;
  float f_wl_amb = 0.0;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double* pd_X = NULL;
  double* pd_deltaX = NULL;
  uint16_t w_i = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  SeqKalmanVar_t z_seqVar = { 0 };
  gnss_fixedAmbType u_fix_type;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal2 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal2_ref = NULL;

  if (NULL == pz_fixedAmbPool || !(pz_fixedAmbPool->u_fixStatus& GNSS_WL_AMB_FIXED))
  {
    return;
  }
  if (hpp_seq_init(&z_seqVar, pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax))
  {
    return;
  }

  pd_X = (double*)OS_MALLOC_FAST(pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax * sizeof(double));
  pd_deltaX = (double*)OS_MALLOC_FAST(pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax * sizeof(double));
  if (NULL == pd_X || NULL == pd_deltaX)
  {
    if (NULL != pd_X) OS_FREE(pd_X); 
    if (NULL != pd_deltaX) OS_FREE(pd_deltaX);
    hpp_seq_deinit(&z_seqVar);
    return;
  }
  for (w_i = 0; w_i < pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_nmax; w_i++)
  {
    pd_X[w_i] = pz_fixedAmbPool->pd_x_fix[w_i];
  }
  u_fix_type = convertFreqType2AmbType(z_freqType1, z_freqType2);
  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);
  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx])
    {
      continue;
    }
    if (!(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_ambFixType & u_fix_type))
    {
      continue;
    }
    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_pppAmbFixInputInfo->pz_pppFilterInfo->z_opt.q_isSperateBDS2And3, u_sys);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    if (u_refsat_svid < 0 || u_refsat_svid == u_svid)
    {
      continue;
    }
   
    /* get status from EKF */
    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[0] = 0;
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
    w_x_id[2] = z_filterType1;
    pz_satPool1_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2_ref = getEKF_status(w_x_id, pz_pppAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPool1_ref || NULL == pz_satPool2_ref)
    {
      continue;
    }
    /* get code bias and phase bias */
    pz_signal1 = getSSR_Bias(u_sys, u_svid, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType1], pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal2 = getSSR_Bias(u_sys, u_svid, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType2], pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    pz_signal1_ref = getSSR_Bias(u_sys, u_refsat_svid, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType1], pz_pppAmbFixInputInfo->pz_ssrLocBlk); /* type is the same as rover satellite */
    pz_signal2_ref = getSSR_Bias(u_sys, u_refsat_svid, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType2], pz_pppAmbFixInputInfo->pz_ssrLocBlk);
    if (NULL == pz_signal1 || NULL == pz_signal2 || NULL == pz_signal1_ref || NULL == pz_signal2_ref)
    {
      continue;
    }
    
    f_upd = (float)((pz_signal1->z_phaseBias.q_corr/ wavelength(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType1]) - pz_signal2->z_phaseBias.q_corr/ wavelength(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType2])
      - (pz_signal1_ref->z_phaseBias.q_corr/ wavelength(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType1]) - pz_signal2_ref->z_phaseBias.q_corr/ wavelength(pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_freqType[z_freqType2]))) * 0.001);
    f_wl_amb = (float)(pd_X[pz_satPool1->w_index] - pd_X[pz_satPool2->w_index]
      - (pd_X[pz_satPool1_ref->w_index] - pd_X[pz_satPool2_ref->w_index]) + f_upd);

    hpp_seq_ClearH(&z_seqVar);
    /* OMC */
    d_omc = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue - f_wl_amb;
    d_curR = SQR(1.0e-8);

    /* H */
    hpp_seq_AddH(pz_satPool1->w_index, 1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPool2->w_index, -1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPool1_ref->w_index, -1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPool2_ref->w_index, 1.0, &z_seqVar);

    hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
    hpp_seq_PredictStep(pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar);
    hpp_seq_measUpdate(pz_pppAmbFixInputInfo->pz_pppFilterInfo->pq_paraValid,pz_fixedAmbPool->pd_x_fix, pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar, 0.0);

    /* print log */
    satidx_SatString(u_sat_idx, c_sat);
    LOGI(TAG_PPP, "wl amb update sat=%s, V=%5.2lf floatX=%7.2lf fixX=%7.2lf dx=%7.3lf\n",
      c_sat, d_omc, f_wl_amb, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValue, pd_deltaX[0]);
  }

  hpp_seq_deinit(&z_seqVar);
  OS_FREE(pd_deltaX);
  OS_FREE(pd_X);
}

/**
 * @brief wide-lane ambiguity resolution for ppp
 * @param[in]    pz_pppAmbFixInputInfo - input information for ppp-ar
 * @param[out]   pz_fixedAmbPool - result of ppp-ar
 * @param[out]   z_freqType - common frequency for narrow-lane amb fix
 * @return status -   1: pppar success, 0: pppar fail
 */
extern uint8_t ppp_wideLaneAmbFix(gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType* z_freqType)
{
  uint8_t q_fix_status = AMB_AR_SUCCESS;
  gnss_FreqType z_freqType1 = C_GNSS_FREQ_TYPE_L1;
  gnss_FreqType z_freqType2 = C_GNSS_FREQ_TYPE_L2;
  gnss_FreqType z_freqType3 = C_GNSS_FREQ_TYPE_L5;

  memcpy(pz_fixedAmbPool->pd_x_fix, pz_pppAmbFixInputInfo->pz_pppFilterInfo->pd_X, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  memcpy(pz_fixedAmbPool->pd_q_fix, pz_pppAmbFixInputInfo->pz_pppFilterInfo->pd_Q, NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));

  /* L2-L5 UW mode */
  pz_pppAmbFixInputInfo->u_fixTypeIdx = convertFreqType2AmbType(z_freqType2, z_freqType3);
  if (refsatSelect(pz_pppAmbFixInputInfo, z_freqType2, z_freqType3, pz_fixedAmbPool))
  {
    LOGI(TAG_PPP, "PPPAR-Widelane(L2-L5) starting...\n");
    if (!(q_fix_status= widelaneArLambda(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, z_freqType3, 1)))
    {
      // wide-lane amb update
      wideLaneAmbUpdate(z_freqType2, z_freqType3, pz_pppAmbFixInputInfo, pz_fixedAmbPool);
      ambFixHold(pz_fixedAmbPool, pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx], pz_pppAmbFixInputInfo->pz_pppFilterInfo);
      *z_freqType = z_freqType2;
    }
    else
    {
      LOGI(TAG_PPP, "PPPAR-Widelane(L2-L5) fail, status=%d\n", q_fix_status);
    }
    updateFixedAmbPool(pz_fixedAmbPool, &pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]);
  }

  /* L1-L2 WL mode */
  pz_pppAmbFixInputInfo->u_fixTypeIdx = convertFreqType2AmbType(z_freqType1, z_freqType2);
  resetFixedAmbPool(pz_fixedAmbPool);
  if (refsatSelect(pz_pppAmbFixInputInfo, z_freqType1, z_freqType2, pz_fixedAmbPool))
  {
    LOGI(TAG_PPP, "PPPAR-Widelane(L1-L2) starting...\n");
    if (!(q_fix_status = widelaneArLambda(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2, 1)))
    {
      // wide-lane amb update
      wideLaneAmbUpdate(z_freqType1, z_freqType2, pz_pppAmbFixInputInfo, pz_fixedAmbPool);
      ambFixHold(pz_fixedAmbPool, pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx], pz_pppAmbFixInputInfo->pz_pppFilterInfo);
      if (!(pz_fixedAmbPool->u_fixStatus& GNSS_UW_AMB_FIXED)) *z_freqType = z_freqType1;
    }
    else LOGI(TAG_PPP, "PPPAR-Widelane(L1-L2) fail, status=%d\n", q_fix_status);
    updateFixedAmbPool(pz_fixedAmbPool, &pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]);
  }
  if (convertFreqType2AmbType(z_freqType1, z_freqType2) & pz_fixedAmbPool->u_fixStatus)
  {
    return 1;
  }

  /* L1-L5 WL mode */
  pz_pppAmbFixInputInfo->u_fixTypeIdx = convertFreqType2AmbType(z_freqType1, z_freqType3);
  resetFixedAmbPool(pz_fixedAmbPool);
  if (refsatSelect(pz_pppAmbFixInputInfo, z_freqType1, z_freqType3, pz_fixedAmbPool))
  {
    LOGI(TAG_PPP, "PPPAR-Widelane(L1-L5) starting...\n");
    if (!(q_fix_status = widelaneArLambda(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType3, 1)))
    {
      // wide-lane amb update
      wideLaneAmbUpdate(z_freqType1, z_freqType3, pz_pppAmbFixInputInfo, pz_fixedAmbPool);
      ambFixHold(pz_fixedAmbPool, pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx], pz_pppAmbFixInputInfo->pz_pppFilterInfo);
      if (!(pz_fixedAmbPool->u_fixStatus & GNSS_UW_AMB_FIXED)) *z_freqType = z_freqType1;
    }
    else LOGI(TAG_PPP, "PPPAR-Widelane(L1-L5) fail, status=%d\n", q_fix_status);
    updateFixedAmbPool(pz_fixedAmbPool, &pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]);
  }
  
  if (GNSS_NONE_AMB_FIXED != pz_fixedAmbPool->u_fixStatus)
  {
    return 1;
  }

  pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_algStatus |= PPP_STATUS_AR_WLFail;
  return 0;
}

/**
 * @brief narrow-lane ambiguity resolution for ppp
 * @param[in]    z_freqType - frequency type of ppp-ar
 * @param[in]    pz_pppAmbFixInputInfo - input information for ppp-ar
 * @param[out]   pz_fixedAmbPool - result of ppp-ar
 * @return  status   --   0: pppar success, other: pppar fail
 */
extern uint8_t ppp_zeroCombineAmbFix(gnss_FreqType z_freqType, gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  LOGI(TAG_PPP, "PPPAR-Narrowlane starting(z_freqType=L%d)...\n", z_freqType+1);
  uint8_t q_fix_status = AMB_AR_SUCCESS;
  pz_pppAmbFixInputInfo->u_fixTypeIdx = convertFreqType2AmbType(z_freqType, z_freqType);

  if (C_GNSS_FREQ_TYPE_MAX == z_freqType)
  {
    LOGW(TAG_PPP, "frequency of nl amb error\n");
    q_fix_status = AMB_AR_FAIL_Freq;
    return q_fix_status;
  }

  /* ref-sat select */
  if (!(q_fix_status = refsatSelect(pz_pppAmbFixInputInfo, z_freqType, z_freqType,pz_fixedAmbPool)))
  {
    /* refsat select fail */
    LOGW(TAG_PPP, "nl refsat select fail,status=%d\n", q_fix_status);
    q_fix_status = AMB_AR_FAIL_RefsatFail;
    return q_fix_status;
  }

  /* narrow-lane ambiguity resolution L1 */
  if ((q_fix_status = narrowlaneAR(pz_pppAmbFixInputInfo, pz_fixedAmbPool, z_freqType)))
  {
    LOGW(TAG_PPP, "Narrowlane ar fail,status=%d\n", q_fix_status);
    return q_fix_status;
  }

  return q_fix_status;
}

/**
 * @brief PPP ambiguity resolve
 * @param[in/out]  pz_pppAmbFixInputInfo is input information for pppar
 * @param[out]  pz_fixedAmbPool is result of amb resolution
 * @return    status of PPP-AR
 */
extern uint8_t PPP_AmbResolution(gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_status = 0;
  uint32_t w_flag = 0;
  uint16_t w_nx = 0;
  
  gnss_FreqType z_freqType_zero = C_GNSS_FREQ_TYPE_L1;
  if (NULL == pz_fixedAmbPool)
  {
    pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_algStatus |= PPP_STATUS_MEMORY_ERROR;
    return AMB_AR_FAIL_MemError;
  }

  /* accuracy of float checking */
  if (floatForArCheck(pz_pppAmbFixInputInfo, pz_fixedAmbPool))
  {
    pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_algStatus |= PPP_STATUS_FLOAT_AR_Bad;
    return AMB_AR_FAIL_FloatError;
  }
  
  pz_fixedAmbPool->z_time = pz_pppAmbFixInputInfo->pz_satSignalPool->z_tor;
  pz_fixedAmbPool->u_ppp_use_sat_num_fix = 0;
  pz_fixedAmbPool->u_ppp_use_sat_num_wideLane = 0;
  pz_fixedAmbPool->u_fixedTotalSatCount = 0;

  // wide-lane amb fix
  ppp_wideLaneAmbFix(pz_pppAmbFixInputInfo, pz_fixedAmbPool, &z_freqType_zero);

  // L1 amb fix
  resetFixedAmbPool(pz_fixedAmbPool);
  u_status = ppp_zeroCombineAmbFix(z_freqType_zero, pz_pppAmbFixInputInfo, pz_fixedAmbPool);

  if (!u_status)
  {
    // L1 amb update
    zeroCombineAmbUpdate(z_freqType_zero, pz_pppAmbFixInputInfo, pz_fixedAmbPool);
  }
  else
  {
    pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_algStatus |= PPP_STATUS_AR_NLFail;
  }
  updateFixedAmbPool(pz_fixedAmbPool, &pz_pppAmbFixInputInfo->pz_preFixedAmbPool[pz_pppAmbFixInputInfo->u_fixTypeIdx]);

  /* post residual */
  PPP_zeroCombinePhaseMeasResidual(pz_fixedAmbPool->pd_x_fix, pz_pppAmbFixInputInfo->pz_satSignalPool, pz_pppAmbFixInputInfo->pz_ssrLocBlk,
    pz_pppAmbFixInputInfo->pz_pppFilterInfo, pz_pppAmbFixInputInfo->pz_EKFstateRepPool, pz_fixedAmbPool, pz_pppAmbFixInputInfo->pz_FilterObs, RES_FIXED);
  PPP_zeroCombineCodeMeasResidual(pz_fixedAmbPool->pd_x_fix, pz_pppAmbFixInputInfo->pz_satSignalPool, pz_pppAmbFixInputInfo->pz_ssrLocBlk,
    pz_pppAmbFixInputInfo->pz_pppFilterInfo, pz_pppAmbFixInputInfo->pz_EKFstateRepPool, pz_fixedAmbPool, pz_pppAmbFixInputInfo->pz_FilterObs, RES_FIXED);

  /* check quality of ambiguity resolution */
  if ((w_flag=PPP_AmbResolutionCheck(pz_pppAmbFixInputInfo, pz_fixedAmbPool)))
  {
    pz_pppAmbFixInputInfo->pz_pppFilterInfo->w_algStatus |= w_flag;
  }

  return u_status;
}
