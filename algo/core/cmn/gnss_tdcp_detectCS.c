#include <mw_log.h>
#include "gnss_tdcp_detectCS.h"
#include "dcp_solution.h"
#include "mw_alloc.h"
#include "gnss_def.h"
#include "gnss_common.h"
#include "dcp_common.h"

/**
 * @brief convert the observation information to TDCP observations
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_curDcpMeas is the current epoch TDCP observations
 * @return the number of TDCP measure
 */
uint16_t cmn_convertSigMeasToDcpStructNonSSR(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_j = 0;
  uint16_t w_measNum = 0;
  uint16_t w_satNum = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  GnssMeas_t* pz_dcpSigMeas = NULL;
  gnss_DcpSatInfo_t* pz_dcpSatInfo = NULL;
  pz_curDcpMeas->z_obsTime = pz_satSigMeasCollect->z_tor;
  pz_curDcpMeas->z_posSol = pz_satSigMeasCollect->z_positionFix;
  pz_curDcpMeas->w_size = sizeof(gnss_TdcpMeasBlock_t);
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (w_measNum >= MAX_GNSS_TRK_MEAS_NUMBER)
    {
      break;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (w_measNum >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      pz_dcpSigMeas = (pz_curDcpMeas->pz_meas) + w_measNum;
      pz_dcpSigMeas->z_measStatusFlag = pz_sigMeas->z_measStatusFlag;
      pz_dcpSigMeas->u_constellation = pz_sigMeas->u_constellation;
      pz_dcpSigMeas->u_svid = pz_sigMeas->u_svid;
      pz_dcpSigMeas->u_signal = pz_sigMeas->u_signal;
      pz_dcpSigMeas->u_LLI = pz_sigMeas->u_LLI;
      pz_dcpSigMeas->f_cn0 = pz_sigMeas->f_cn0;
      pz_dcpSigMeas->d_pseudoRange = pz_sigMeas->d_pseudoRange;
      pz_dcpSigMeas->d_doppler = pz_sigMeas->d_doppler;
      pz_dcpSigMeas->d_carrierPhase = pz_sigMeas->d_carrierPhase;
      ++w_measNum;
    }
  }
  pz_curDcpMeas->w_measNum = w_measNum;
  w_satNum = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (w_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_dcpSatInfo = (pz_curDcpMeas->pz_satInfo) + w_satNum;
    memset(pz_dcpSatInfo, 0, sizeof(gnss_DcpSatInfo_t));
    pz_dcpSatInfo->u_constellation = pz_satMeas->u_constellation;
    pz_dcpSatInfo->u_svid = pz_satMeas->u_svid;
    for (u_j = 0; u_j < 4; ++u_j)
    {
      pz_dcpSatInfo->pd_satPosClk[u_j] = pz_satMeas->z_satPosVelClk.d_satPosClk[u_j];
      pz_dcpSatInfo->pd_satVelClk[u_j] = pz_satMeas->z_satPosVelClk.d_satVelClk[u_j];
    }
    pz_dcpSatInfo->q_iode = pz_satMeas->z_satPosVelClk.q_iode;
    pz_dcpSatInfo->f_elevation = (float)(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG);
    pz_dcpSatInfo->f_azimuth = (float)(pz_satMeas->z_satPosVelClk.f_azimuth * RAD2DEG);
    pz_dcpSatInfo->u_valid = TRUE;
    ++w_satNum;
  }
  pz_curDcpMeas->w_satNum = w_satNum;
  return w_measNum;
}
/**
 * @brief convert the observation information to TDCP observations
 * @param[in] pz_SsrLocBlk is the SSR product
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_curDcpMeas is the current epoch TDCP observations
 * @return the number of TDCP measure
 */
uint16_t cmn_convertSigMeasToDcpStruct(const gnss_ssrLosBlock_t* pz_SsrLocBlk, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  gnss_ConstellationType u_constellation = 0;
  uint16_t w_measNum = 0;
  uint16_t w_satNum = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_epochSsrLos_t* pz_epochLos = &(pz_SsrLocBlk->z_epochLosInfo);
  const gnss_satSsrLos_t* pz_satLos = NULL;
  GnssMeas_t* pz_dcpSigMeas = NULL;
  gnss_DcpSatInfo_t* pz_dcpSatInfo = NULL;
  pz_curDcpMeas->z_obsTime = pz_satSigMeasCollect->z_tor;
  pz_curDcpMeas->z_posSol = pz_satSigMeasCollect->z_positionFix;
  pz_curDcpMeas->w_size = sizeof(gnss_TdcpMeasBlock_t);
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (w_measNum >= MAX_GNSS_TRK_MEAS_NUMBER)
    {
      break;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (w_measNum >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      pz_dcpSigMeas = (pz_curDcpMeas->pz_meas) + w_measNum;
      pz_dcpSigMeas->z_measStatusFlag = pz_sigMeas->z_measStatusFlag;
      pz_dcpSigMeas->u_constellation = pz_sigMeas->u_constellation;
      pz_dcpSigMeas->u_svid = pz_sigMeas->u_svid;
      pz_dcpSigMeas->u_signal = pz_sigMeas->u_signal;
      pz_dcpSigMeas->u_LLI = pz_sigMeas->u_LLI;
      pz_dcpSigMeas->f_cn0 = pz_sigMeas->f_cn0;
      pz_dcpSigMeas->d_pseudoRange = pz_sigMeas->d_pseudoRange;
      pz_dcpSigMeas->d_doppler = pz_sigMeas->d_doppler;
      pz_dcpSigMeas->d_carrierPhase = pz_sigMeas->d_carrierPhase;
      ++w_measNum;
    }
  }
  for (u_i = 0; u_i < (pz_epochLos->u_satCount); ++u_i)
  {
    pz_satLos = &(pz_epochLos->z_satLosCorr[u_i]);
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH))
    {
      continue;
    }
    pz_dcpSatInfo = (pz_curDcpMeas->pz_satInfo) + w_satNum;
    memset(pz_dcpSatInfo, 0, sizeof(gnss_DcpSatInfo_t));
    pz_dcpSatInfo->u_constellation = pz_satLos->u_constellation;
    pz_dcpSatInfo->u_svid = pz_satLos->u_svid;
    for (u_j = 0; u_j < 4; ++u_j)
    {
      pz_dcpSatInfo->pd_satPosClk[u_j] = pz_satLos->z_satPosVelClkBrdc.d_satPosClk[u_j];
      pz_dcpSatInfo->pd_satVelClk[u_j] = pz_satLos->z_satPosVelClkBrdc.d_satVelClk[u_j];
    }
    pz_dcpSatInfo->q_iode = pz_satLos->q_iode;
    pz_dcpSatInfo->u_valid = TRUE;
    ++w_satNum;
  }
  for (u_i = 0; u_i < w_satNum; ++u_i)
  {
    pz_dcpSatInfo = (pz_curDcpMeas->pz_satInfo) + u_i;
    for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
      if (NULL == pz_satMeas)
      {
        continue;
      }
      u_constellation = pz_satMeas->u_constellation;
      if (u_constellation == (pz_dcpSatInfo->u_constellation) && (pz_satMeas->u_svid) == (pz_dcpSatInfo->u_svid))
      {
        break;
      }
    }
    if (q_satIndex >= ALL_GNSS_SYS_SV_NUMBER)
    {
      continue;
    }
    pz_dcpSatInfo->f_elevation = (float)(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG);
    pz_dcpSatInfo->f_azimuth = (float)(pz_satMeas->z_satPosVelClk.f_azimuth * RAD2DEG);
  }
  pz_curDcpMeas->w_measNum = w_measNum;
  pz_curDcpMeas->w_satNum = w_satNum;

  return w_measNum;
}
/**
 * @brief find the index of signal observation in the struct gnss_SatSigMeasCollect_t
 * @param[in] u_constellation constellation belong to observation
 * @param[in] u_svid is the prn
 * @param[in] e_targetFreqType is the target frequency
 * @param[in] pz_satSigMeasCollect is the observation information
 * @param[out]pq_satIndexFinded the satllite index in the struct gnss_SatSigMeasCollect_t
 * @param[out]pu_sigIndexFinded the signal index in the struct gnss_SatSigMeasCollect_t
 * @return 1 represent had finded and 0 represent finded failure
 */
uint8_t cmn_findSatSigMeasTDCP(uint8_t u_constellation, uint8_t u_svid, gnss_FreqType e_targetFreqType, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  uint32_t* pq_satIndexFinded, uint8_t* pu_sigIndexFinded)
{
  uint32_t q_satIndex = 0;
  uint8_t u_isFind = 0;
  uint8_t u_signalIndex = 0;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  *pq_satIndexFinded = 0;
  *pu_sigIndexFinded = 0;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->u_constellation != u_constellation || (pz_satMeas->u_svid) != u_svid)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_sigMeas->u_signal);
      if (e_curFreqType == e_targetFreqType)
      {
        u_isFind = 1;
        *pq_satIndexFinded = q_satIndex;
        *pu_sigIndexFinded = u_signalIndex;
        break;
      }
    }
    if (1 == u_isFind)
    {
      break;
    }
  }
  return u_isFind;
}
/**
 * @brief mark the cycle slip using the post residual of TDCP
 * @param[in] z_usedSys constellation used in the DCP algorithm
 * @param[in] e_targetFreqType is the target frequency
 * @param[in] pz_curDcpMeas the current epoch TDCP observations
 * @param[in] pz_preDcpMeas the previous epoch TDCP observations
 * @param[in] pd_X the value of site coordinate and receiver clock drift parameters
 * @param[out]pz_satSigMeasCollect is the observation information
 * @return the number of non cycle slip observations
 */
uint8_t cmn_markCycleSlipTargetFreq(algo_useConstellation z_usedSys, gnss_FreqType e_targetFreqType, const gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_preDcpMeas, const double pd_X[TD_MAX_PARA], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t u_preIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_nonCycleSlipObsNum = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint16_t w_iMeas = 0;
  uint32_t q_satIndex = 0;
  const GnssMeas_t* pz_meas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  const GnssMeas_t* pz_preMeas = NULL;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  double d_wave = 0.0;
  double f_res = 0.0;
  const double* pre_siteEcefCoor = pz_preDcpMeas->z_posSol.d_xyz;
  //check carrier phase according to the residual
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation))
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == (z_usedSys & z_algoConstellation))
    {
      continue;
    }
    u_constellation = (pz_meas->u_constellation);
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if (e_targetFreqType != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_res = ((pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase) * d_wave);
    f_res -= (pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i] * (pd_X[u_i] - pre_siteEcefCoor[u_i]));
    }
    //the satellite clock correction to be tested
    f_res += (pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);

    //f_res -= (pd_X[3 + u_constellation]);
    if (fabs(pd_X[3 + u_constellation]) < 1e-3 && C_GNSS_QZS == u_constellation)
    {
        f_res -= (pd_X[3 + C_GNSS_GPS]);//use GPS to replace QZSS
    }
    else
    {
        f_res -= (pd_X[3 + u_constellation]);
    }
    if (0 == cmn_findSatSigMeasTDCP(pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, pz_satSigMeasCollect, &q_satIndex, &u_signalIndex))
    {
      continue;
    }
    (pz_satSigMeasCollect->pz_satMeas[q_satIndex]->pz_signalMeas[u_signalIndex]->u_slipMethodValid) |= TDCP_DETECT_VALID;
    if (fabs(f_res) > 0.10)//the threhold to be optimizite
    {
      char sid[8] = { 0 };
      svid_SatString(pz_meas->u_svid, pz_meas->u_constellation, sid);
      LOGI(TAG_DCP, "slip dcp: sat=%s, frq=%d, res=%8.3f\n", sid, e_targetFreqType, f_res);
      continue;
    }
    (pz_satSigMeasCollect->pz_satMeas[q_satIndex]->pz_signalMeas[u_signalIndex]->u_slipMask) |= NON_CYCLE_SLIP_BY_TDCP;
    ++u_nonCycleSlipObsNum;
  }
  return u_nonCycleSlipObsNum;
}
/**
 * @brief get the receiver clock drift for the target frequency
 * @param[in] q_isSeparateBDS2And3 whether BDS2 and BDS3 are separate
 * @param[in] e_TargetSysType is the target constellation
 * @param[in] e_targetFreqType is the target frequency
 * @param[in] pz_curDcpMeas the current epoch TDCP observations
 * @param[in] pz_preDcpMeas the previous epoch TDCP observations
 * @param[in] pd_X the value of site coordinate and receiver clock drift parameters
 * @param[out]pf_rcvDriftClk is the value of receiver clock drift for the target frequency
 * @return the satellite number of used by calculated receiver clock drift for the target frequency
 */
uint8_t cmn_filterTargetFreqRcvDriftClock(BOOL q_isSeparateBDS2And3, gnss_ConstellationType e_TargetSysType, gnss_FreqType e_targetFreqType, const gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  const gnss_TdcpMeasBlock_t* pz_preDcpMeas, const double pd_X[TD_MAX_PARA], float* pf_rcvDriftClk)
{
  uint8_t u_normNum = 0;
  uint8_t u_num = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_i = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_flag = 0;
  uint16_t w_iMeas = 0;
  uint32_t q_satIndex = 0;
  const GnssMeas_t* pz_meas = NULL;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  const GnssMeas_t* pz_preMeas = NULL;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  double d_wave = 0.0;
  float f_res = 0.0;
  const double* pre_siteEcefCoor = pz_preDcpMeas->z_posSol.d_xyz;
  uint8_t pu_obsTagIndex[MAX_GNSS_TRK_MEAS_NUMBER] = { 0 };
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_normDiff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float pf_diffAbs[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  float f_mean = 0.0;
  float f_rejectThres = 0.0;
  float f_rejectK2 = 8.0;
  *pf_rcvDriftClk = 0.0;
  //check carrier phase according to the residual
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (u_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(q_isSeparateBDS2And3, pz_meas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || e_TargetSysType != u_constellation)
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    //because of L1 frequency relative to other frequency, there will only deal with the L1 frequency
    if (e_targetFreqType != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeas, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeas->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeas);
    if (w_curSatIndex < 0 || w_preSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_preDcpMeas->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_res = (float)((pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase) * d_wave);
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_preDcpMeas->pz_satInfo[w_preSatIndex].d_sat2SiteDist);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i] * (pd_X[u_i] - pre_siteEcefCoor[u_i]));
    }
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_preDcpMeas->pz_satInfo[w_preSatIndex].pd_satPosClk[3]);
    f_res -= (float)(pd_X[3 + e_TargetSysType]);
    pf_diff[u_num] = f_res;
    pu_obsTagIndex[u_num] = (uint8_t)w_iMeas;
    ++u_num;
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
  if (!u_flag)
  {
    u_normNum = 0;
  }
  else
  {
    *pf_rcvDriftClk = f_mean;
  }
  return u_normNum;
}
/**
 * @brief the interface of using the previous epoch and current observations to detect cycle slip
 * @param[in/out] pz_dcpConfig the configuration for the DCP model
 * @param[in/out] pz_preDcpMeas the previous epoch TDCP observations
 * @param[out]    pz_curDcpMeas the current epoch TDCP observations
 * @param[out]    pd_X the value of site coordinate and receiver clock drift parameters
 * @return 1 represent success and 0 represent failure
 */
uint8_t tdcp_detectCycleSlip(gnss_DcpConfigOpt_t* pz_dcpConfig, gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas, double pd_X[TD_MAX_PARA])
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  dcp_interRecordInfo_t z_dcpRecordInfo = { 0 };
  gnss_PositionFix_t* pz_curPosSol = &(pz_curDcpMeas->z_posSol);
  const gnss_PositionFix_t* pz_prePosSol = NULL;
  pz_dcpConfig->z_optimalFreqType = C_GNSS_FREQ_TYPE_L1;
  for (u_i = 0; u_i < TD_MAX_PARA; ++u_i)
  {
    pd_X[u_i] = 0.0;
  }
  if (NULL == pz_preDcpMeas || NULL == pz_curDcpMeas)
  {
    return 0;
  }
  pz_prePosSol = &(pz_preDcpMeas->z_posSol);
  dcp_initDcpInterInfo(&z_dcpRecordInfo);
  z_dcpRecordInfo.f_deltaTime = (float)(tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_preDcpMeas->z_obsTime)));
  if (GNSS_FIX_FLAG_INVALID != (pz_preDcpMeas->z_posSol.u_fixFlag))
  {
    dcp_fillupPosSolUsingPreEpoch(pz_prePosSol, pz_curPosSol);
    pz_curPosSol->z_gpsTime = pz_curDcpMeas->z_obsTime;
    if ((pz_prePosSol->z_rtk.f_age) > 0.0)
    {
      pz_curPosSol->z_rtk.f_age = (pz_prePosSol->z_rtk.f_age) + z_dcpRecordInfo.f_deltaTime;
    }
  }
  if (dcp_checkGoingTDroute(&(pz_prePosSol->z_SvStatus.u_SvInUseCount), &(pz_prePosSol->u_fixFlag), pz_prePosSol->d_xyz, &(z_dcpRecordInfo.f_deltaTime)))
  {
    u_status = dcp_gainTDpos(pz_dcpConfig, pz_preDcpMeas, pz_curDcpMeas, &z_dcpRecordInfo);
    if (1 == u_status)
    {
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_X[u_i] = z_dcpRecordInfo.pd_curXyz[u_i];
      }
      for (u_i = 3; u_i < TD_MAX_PARA; ++u_i)
      {
        pd_X[u_i] = (double)(z_dcpRecordInfo.pf_rcvClkDrift[u_i - 3] + z_dcpRecordInfo.pf_deltaX[u_i]);
      }
    }
  }
  return  u_status;
}
/**
 * @brief using TDCP method to detect cycle slip
 * @param[in]     z_tdcpMeasPointerPair is the pointer pair of previous epoch and current epoch to TDCP observations
 * @param[in/out] pz_satSigMeasCollect is the observation information
 * @return 1 represent successful and 0 represent failure
 */
uint8_t cmn_detectCycleSlipByTDCP(gnss_TdcpMeasPair_t z_tdcpMeasPointerPair, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_iFreq = 0;
  uint8_t u_nonCycleSlipMaxObsNum = 0;
  uint8_t u_nonCycleSlipObsNum = 0;
  gnss_DcpConfigOpt_t z_dcpOpt = { 0 };
  double pd_X[TD_MAX_PARA] = { 0.0 };
  double pd_Xcopied[TD_MAX_PARA] = { 0.0 };
  float f_rcvDriftClk = 0.0;
  z_dcpOpt.f_eleCutOff = 7.0;
  z_dcpOpt.u_enableCalCurrentSatPos = 0;
  z_dcpOpt.u_enableDcpPosSmooth = 0;
  z_dcpOpt.u_enableTDDRorTDPR = 0;
  z_dcpOpt.z_usedSys = (ALGO_GPS_SYS | ALGO_BDS_SYS | ALGO_GAL_SYS | ALGO_QZS_SYS);
  z_dcpOpt.z_optimalFreqType = C_GNSS_FREQ_TYPE_L1;
  u_status = tdcp_detectCycleSlip(&z_dcpOpt, z_tdcpMeasPointerPair.pz_preTdcpMeas, z_tdcpMeasPointerPair.pz_curTdcpMeas, pd_X);
  if (FALSE == z_dcpOpt.pq_isSeparateBDS2And3[z_dcpOpt.z_optimalFreqType])
  {
    pd_X[3 + C_GNSS_BDS2] = pd_X[3 + C_GNSS_BDS3];
  }
  for (u_i = 0; u_i < TD_MAX_PARA; ++u_i)
  {
    pd_Xcopied[u_i] = pd_X[u_i];
  }
  if (1 == u_status)
  {
    u_nonCycleSlipObsNum = cmn_markCycleSlipTargetFreq(z_dcpOpt.z_usedSys, z_dcpOpt.z_optimalFreqType, z_tdcpMeasPointerPair.pz_curTdcpMeas, z_tdcpMeasPointerPair.pz_preTdcpMeas, pd_X, pz_satSigMeasCollect);
    u_nonCycleSlipMaxObsNum = u_nonCycleSlipObsNum;
    for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
    {
      if (u_iFreq == z_dcpOpt.z_optimalFreqType)
      {
        continue;
      }
      for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
      {
        if (TRUE == gnss_WhetherSkipBDS2VarInLoop(z_dcpOpt.pq_isSeparateBDS2And3[u_iFreq], u_i))
        {
          continue;
        }
        if (cmn_filterTargetFreqRcvDriftClock(z_dcpOpt.pq_isSeparateBDS2And3[u_iFreq], (gnss_ConstellationType)u_i, (gnss_FreqType)u_iFreq, z_tdcpMeasPointerPair.pz_curTdcpMeas,
          z_tdcpMeasPointerPair.pz_preTdcpMeas, pd_X, &f_rcvDriftClk) > 2)
        {
          pd_Xcopied[3 + u_i] = pd_X[3 + u_i] + f_rcvDriftClk;
        }
        else
        {
          pd_Xcopied[3 + u_i] = pd_X[3 + u_i];
        }
      }
      if (FALSE == z_dcpOpt.pq_isSeparateBDS2And3[u_iFreq])
      {
        pd_Xcopied[3 + C_GNSS_BDS2] = pd_Xcopied[3 + C_GNSS_BDS3];
      }
      u_nonCycleSlipObsNum = cmn_markCycleSlipTargetFreq(z_dcpOpt.z_usedSys, (gnss_FreqType)u_iFreq, z_tdcpMeasPointerPair.pz_curTdcpMeas, z_tdcpMeasPointerPair.pz_preTdcpMeas, pd_Xcopied, pz_satSigMeasCollect);
      if (u_nonCycleSlipObsNum >= u_nonCycleSlipMaxObsNum)
      {
        u_nonCycleSlipMaxObsNum = u_nonCycleSlipObsNum;
      }
    }
  }
  if (u_nonCycleSlipMaxObsNum < 10)//only for multi-concellation
  {
    u_status = 0;
  }
  return u_status;
}