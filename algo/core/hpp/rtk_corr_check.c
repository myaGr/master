#include "gnss_common.h"
#include "gnss_type.h"
#include "gnss_def.h"
#include "cmn_turboEdit.h"
#include "rtk_corr_check.h"
#include "rtk_proc.h"
#include "sd_api.h"
#include "mw_alloc.h"
#include "mw_log.h"

#define RTK_CORR_MAX_REF_SAT_NUM   (MAX_SV_NUM_IN_ONE_SYS + 1)

uint8_t pu_multiSysFlag = FALSE;

/**
 * @brief the inteface of using turbo edit method to detect ref corr's slip
 * @param[in]       pz_RTKcorrBlockCur is the current RTK VRS observations and coordinate
 * @param[in/out]   pz_gfFilterSet is the filter information used by GF method
 * @param[out]      pz_RTKcorrSlipFlag saves Detect result
 * @return     void
 */
void rtk_gfDetectCycleSlip(const GnssCorrBlock_t* pz_RTKcorrBlockCur, cmn_turboEditFilterSet* pz_gfFilterSet,
  GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag)
{
  uint16_t w_i = 0;
  uint8_t u_j = 0;
  uint16_t w_svIndex = 0;
  uint16_t w_filterIndex = 0;
  uint16_t w_invalidIndex = C_TE_COMBINE_MAX * ALL_GNSS_SYS_SV_NUMBER;
  uint16_t w_gfCount = 0;
  uint16_t w_isat = 0;
  uint16_t w_1stFreqCount = 0;
  uint16_t w_1stFreqIndex = 0;
  char char_sat[4] = "";
  float f_azel[2] = { 0.0 };
  double d_wave1 = 0.0;
  double d_wave2 = 0.0;
  double d_gfValue = 0.0;
  double d_gfValuePre = 0.0;
  double pd_unitVector[3] = { 0.0 };
  double pd_lla[3] = { 0.0 };
  gnss_FreqType z_freqType1 = C_GNSS_FREQ_TYPE_MAX;
  gnss_FreqType z_freqType2 = C_GNSS_FREQ_TYPE_MAX;
  const GnssMeas_t* pz_1stFreqMeas = NULL;
  const GnssMeas_t* pz_2rdFreqMeas = NULL;

  for (w_i = 0; w_i < pz_RTKcorrBlockCur->w_measCount; w_i++)
  {
    if (w_1stFreqCount >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      continue;
    }
    pz_1stFreqMeas = &pz_RTKcorrBlockCur->z_meas[w_i];
    if (!pz_1stFreqMeas->z_measStatusFlag.b_valid
      || !pz_1stFreqMeas->z_measStatusFlag.b_cpValid)
    {
      continue;
    }
    z_freqType1 = gnss_cvt_Sig2FreqType(pz_1stFreqMeas->u_signal);
    if (C_GNSS_FREQ_TYPE_L1 != z_freqType1)
    {
      continue;
    }
    if (fabs(pz_1stFreqMeas->d_carrierPhase) < 1.0e-3)
    {
      continue;
    }
    d_wave1 = wavelength(pz_1stFreqMeas->u_signal);
    if (d_wave1 >= 1.0)
    {
      continue;
    }
    w_1stFreqCount++;
    for (u_j = C_GNSS_FREQ_TYPE_L1 + 1; u_j < C_GNSS_FREQ_TYPE_MAX; u_j++)
    {
      pz_2rdFreqMeas = getRtkMeasCorr(pz_1stFreqMeas->u_constellation,
        pz_1stFreqMeas->u_svid, u_j, pz_RTKcorrBlockCur);
      if (NULL == pz_2rdFreqMeas)
      {
        continue;
      }
      if (fabs(pz_2rdFreqMeas->d_carrierPhase) < 1.0e-3)
      {
        continue;
      }
      d_wave2 = wavelength(pz_2rdFreqMeas->u_signal);
      if (d_wave2 >= 1.0)
      {
        continue;
      }
      w_svIndex = gnss_cvt_Svid2SvIndex(pz_1stFreqMeas->u_svid, pz_1stFreqMeas->u_constellation);

      d_gfValue = d_wave1 * (pz_1stFreqMeas->d_carrierPhase) - d_wave2 * (pz_2rdFreqMeas->d_carrierPhase);
      d_gfValue /= d_wave1;
      z_freqType2 = gnss_cvt_Sig2FreqType(pz_2rdFreqMeas->u_signal);
      w_filterIndex = cmn_getKeyValue(w_svIndex, z_freqType1, z_freqType2);
      if (w_filterIndex >= w_invalidIndex)
      {
        continue;
      }
      if (NULL == pz_gfFilterSet->pz_turboEditSet[w_filterIndex])
      {
        pz_gfFilterSet->pz_turboEditSet[w_filterIndex] = (cmn_oneObsTurboEditFilterInfo*)OS_MALLOC(sizeof(cmn_oneObsTurboEditFilterInfo));
        cmn_initTurboEditFilterInfo(pz_gfFilterSet->pz_turboEditSet[w_filterIndex]);
      }
      if (NULL == pz_gfFilterSet->pz_turboEditSet[w_filterIndex])
      {
        continue;
      }
      //get ele
      w_isat = rtk_getIndexInCorrSatPosClk(pz_1stFreqMeas->u_constellation,
        pz_1stFreqMeas->u_svid, pz_RTKcorrBlockCur->pz_satPosClk, pz_RTKcorrBlockCur->w_satCount);
      if (gnss_Dot(pz_RTKcorrBlockCur->pz_satPosClk[w_isat].d_satPosClk,
        pz_RTKcorrBlockCur->pz_satPosClk[w_isat].d_satPosClk, 3) < FABS_ZEROS)
      {
        continue;
      }
      gnss_unitVector(pz_RTKcorrBlockCur->d_refPosEcef, 
        pz_RTKcorrBlockCur->pz_satPosClk[w_isat].d_satPosClk, pd_unitVector);
      gnss_Ecef2Lla(pz_RTKcorrBlockCur->d_refPosEcef, pd_lla);
      gnss_Satazel(pd_lla, pd_unitVector, f_azel);

      d_gfValuePre = pz_gfFilterSet->pz_turboEditSet[w_filterIndex]->d_value;
      if (1 == cmn_gfFilterDetect(&(pz_RTKcorrBlockCur->z_Clock.z_gpsTime), d_gfValue, 
        f_azel[1], pz_gfFilterSet->pz_turboEditSet[w_filterIndex]))
      {
        pz_RTKcorrSlipFlag->u_slip[w_svIndex][z_freqType1] |= NON_CORR_SLIP_BY_GF;
        pz_RTKcorrSlipFlag->u_slip[w_svIndex][z_freqType2] |= NON_CORR_SLIP_BY_GF;
      }
      else
      {
        pz_RTKcorrSlipFlag->u_slip[w_svIndex][z_freqType1] |= HAS_CORR_SLIP_BY_GF;
        pz_RTKcorrSlipFlag->u_slip[w_svIndex][z_freqType2] |= HAS_CORR_SLIP_BY_GF;
        satidx_SatString((uint8_t)w_svIndex, char_sat);
        LOGI(TAG_HPP, " * Corr Slip * GF * sat=%s Freq0 & Freq%d gf0=%6.2f gf1=%6.2f\n", char_sat, z_freqType2,
          d_gfValuePre, d_gfValue);
      }

    }
  }
}

/**
 * @brief using tri-difference method to mask target constellation and frequency of ref corr's slip
 * @param[in]       z_targetSys is the target constellation
 * @param[in]       z_targetFreq is the target frequency
 * @param[in]       u_epochDiffNum is the number of time-difference between VRS observations
 * @param[in]       pd_epochDiff is the value of time-difference between VRS observations
 * @param[in]       pu_epochDiffIndex is the satellite index in VRS observations
 * @param[in]       u_refIdx is the selected reference index
 * @param[out]      pz_RTKcorrSlipFlag saves Detect result
 * @return     void
 */
void rtk_tdMaskTargetSysFreqCycleSlip(gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq, uint8_t u_epochDiffNum,
  const  double* pd_epochDiff, const uint8_t* pu_epochDiffIndex, uint8_t u_refIdx, GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag)
{
  /* threshold define begin */
  const double d_maxDDvalue = 0.5; //max time-sat-difference value (cycle)
  /* threshold define end */
  uint8_t u_epochDiffErrNum = 0;
  uint8_t u_i = 0;
  uint8_t u_isSuccess = 0;
  char char_sat[4] = "";
  char char_sat_ref[4] = "";
  double d_median = 0.0;
  double* pd_epochDiffMedianUsed = NULL;
  if (u_epochDiffNum > 2 && u_refIdx < RTK_CORR_MAX_REF_SAT_NUM)
  {
    u_epochDiffErrNum = 0;
    for (u_i = 0; u_i < u_epochDiffNum; u_i++)
    {
      if (u_i == u_refIdx)
      {
        continue;
      }
      if (fabs(pd_epochDiff[u_i] - pd_epochDiff[u_refIdx]) > d_maxDDvalue)
      {
        pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] |= HAS_CORR_SLIP_BY_TD;
        satidx_SatString((uint8_t)pu_epochDiffIndex[u_i], char_sat);
        satidx_SatString((uint8_t)pu_epochDiffIndex[u_refIdx], char_sat_ref);
        LOGI(TAG_HPP, " * Corr Slip * TD * sat=%s ref=%s freq=%d td=%6.2f\n", char_sat, char_sat_ref,
          z_targetFreq, pd_epochDiff[u_i] - pd_epochDiff[u_refIdx]);
        u_epochDiffErrNum++;
      }
      else
      {
        pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] |= NON_CORR_SLIP_BY_TD;
      }
    }
    if (u_epochDiffErrNum > u_epochDiffNum * 0.5)
    {
      pd_epochDiffMedianUsed = (double*)OS_MALLOC(sizeof(double) * u_epochDiffNum);
      memcpy(pd_epochDiffMedianUsed, pd_epochDiff, sizeof(double) * u_epochDiffNum);
      d_median = 0.0;
      u_isSuccess = gnss_ascSortMedianDouble(pd_epochDiffMedianUsed, u_epochDiffNum, &d_median);
      OS_FREE(pd_epochDiffMedianUsed);
      u_epochDiffErrNum = 0;
      for (u_i = 0; u_i < u_epochDiffNum; ++u_i)
      {
        pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] = CORR_SLIP_DEFAULT;
        if (fabs(pd_epochDiff[u_i] - d_median) > d_maxDDvalue)
        {
          pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] |= HAS_CORR_SLIP_BY_TD;
          satidx_SatString((uint8_t)pu_epochDiffIndex[u_i], char_sat);
          satidx_SatString((uint8_t)pu_epochDiffIndex[u_refIdx], char_sat_ref);
          LOGI(TAG_HPP, " * Corr Slip * TD * sat=%s ref=%s freq=%d td=%6.2f\n", char_sat, char_sat_ref,
            z_targetFreq, pd_epochDiff[u_i] - pd_epochDiff[u_refIdx]);
          u_epochDiffErrNum++;
        }
        else
        {
          pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] |= NON_CORR_SLIP_BY_TD;
        }
      }
      if (u_epochDiffErrNum > u_epochDiffNum * 0.5)
      {
        u_isSuccess = 0;
      }
      if (0 == u_isSuccess)
      {
        for (u_i = 0; u_i < u_epochDiffNum; u_i++)
        {
          pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] = CORR_SLIP_DEFAULT;
          pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_i]][z_targetFreq] |= HAS_CORR_SLIP_BY_TD;
        }
        if (C_GNSS_BDS3 == z_targetSys)
        {
          LOGI(TAG_HPP, " * Corr Slip * TD * sys=BDS3 freq=%d : half of sat slip, reset all\n", z_targetFreq);
        }
        else
        {
          LOGI(TAG_HPP, " * Corr Slip * TD * sys=%s freq=%d : half of sat slip, reset all\n", system_char[z_targetSys], z_targetFreq);
        }
      }
    }
    else
    {
      pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[u_refIdx]][z_targetFreq] |= NON_CORR_SLIP_BY_TD;
    }
  }
  else if (2 == u_epochDiffNum)
  {
    if (fabs(pd_epochDiff[0] - pd_epochDiff[1]) > d_maxDDvalue)
    {
      pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[0]][z_targetFreq] |= HAS_CORR_SLIP_BY_TD;
      pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[1]][z_targetFreq] |= HAS_CORR_SLIP_BY_TD;
      satidx_SatString((uint8_t)pu_epochDiffIndex[0], char_sat);
      satidx_SatString((uint8_t)pu_epochDiffIndex[1], char_sat_ref);
      LOGI(TAG_HPP, " * Corr Slip * TD * sat0=%s sat1=%s freq=%d td=%5.2f\n", char_sat, char_sat_ref,
        z_targetFreq, pd_epochDiff[0] - pd_epochDiff[1]);
    }
    else
    {
      pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[0]][z_targetFreq] |= NON_CORR_SLIP_BY_TD;
      pz_RTKcorrSlipFlag->u_slip[pu_epochDiffIndex[1]][z_targetFreq] |= NON_CORR_SLIP_BY_TD;
    }
  }
  return;
}
/**
 * @brief using tri-difference method to detect target constellation and frequency of ref corr's slip
 * @param[in]       z_targetSys is the target constellation
 * @param[in]       z_targetFreq is the target frequency
 * @param[in]       pz_RTKcorrBlockCur is the current RTK VRS observations and coordinate
 * @param[in]       pz_RTKcorrBlockPre is the previous RTK VRS observations and coordinate
 * @param[out]      pz_RTKcorrSlipFlag saves Detect result
 * @return     void
 */
void rtk_tdDetectTargetSysFreqCycleSlip(gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq, const GnssCorrBlock_t* pz_RTKcorrBlockCur,
  const GnssCorrBlock_t* pz_RTKcorrBlockPre, GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag)
{
  uint8_t u_epochDiffNum = 0;
  uint8_t u_i = 0;
  uint8_t u_constell = 0;
  uint8_t u_svid = 0;
  uint8_t u_refIdx = RTK_CORR_MAX_REF_SAT_NUM;
  gnss_FreqType z_FreqType = C_GNSS_FREQ_TYPE_MAX;
  uint16_t w_iMeas = 0;
  uint16_t w_iSatCur = 0;
  uint16_t w_iSatPre = 0;
  float f_refSNR = 0.0;
  double d_wave = 0.0;
  double d_curDist = 0.0;
  double d_preDist = 0.0;
  double d_baseEarthRotCur = 0.0;
  double d_baseEarthRotPre = 0.0;
  const GnssMeas_t* pz_corrMeasPre = NULL;
  const GnssMeas_t* pz_corrMeasCur = NULL;
  const double* pd_baseSatPosClkCur = NULL;
  const double* pd_baseSatPosClkPre = NULL;
  double* pd_epochDiff = NULL;
  uint8_t* pu_epochDiffIndex = NULL;
  double pd_s2r[3] = { 0.0 };

  u_epochDiffNum = 0;
  f_refSNR = 0.0;
  pd_epochDiff = (double*)OS_MALLOC(sizeof(double) * RTK_CORR_MAX_REF_SAT_NUM);
  pu_epochDiffIndex = (uint8_t*)OS_MALLOC(sizeof(uint8_t) * RTK_CORR_MAX_REF_SAT_NUM);
  memset(pd_epochDiff, 0, RTK_CORR_MAX_REF_SAT_NUM * sizeof(double));
  memset(pu_epochDiffIndex, 0, RTK_CORR_MAX_REF_SAT_NUM * sizeof(uint8_t));
  for (w_iMeas = 0; w_iMeas < (pz_RTKcorrBlockCur->w_measCount); ++w_iMeas)
  {
    // meas availability check
    pz_corrMeasCur = &(pz_RTKcorrBlockCur->z_meas[w_iMeas]);
    if (NULL == pz_corrMeasCur || !(pz_corrMeasCur->z_measStatusFlag.b_valid) ||
      !(pz_corrMeasCur->z_measStatusFlag.b_cpValid) || pz_corrMeasCur->d_carrierPhase < FABS_ZEROS)
    {
      continue;
    }
    z_FreqType = gnss_cvt_Sig2FreqType(pz_corrMeasCur->u_signal);
    if (z_targetFreq != z_FreqType)
    {
      continue;
    }
    u_constell = pz_corrMeasCur->u_constellation;
    u_svid = pz_corrMeasCur->u_svid;
    if (u_constell != z_targetSys)
    {
      continue;
    }
    // match target corr meas previous
    pz_corrMeasPre = getRtkMeasCorr(u_constell, u_svid, z_FreqType, pz_RTKcorrBlockPre);
    if (NULL == pz_corrMeasPre || !(pz_corrMeasPre->z_measStatusFlag.b_valid) ||
      !(pz_corrMeasPre->z_measStatusFlag.b_cpValid) || pz_corrMeasPre->d_carrierPhase < FABS_ZEROS)
    {
      continue;
    }
    // iode check
    w_iSatCur = rtk_getIndexInCorrSatPosClk(u_constell, u_svid, pz_RTKcorrBlockCur->pz_satPosClk, pz_RTKcorrBlockCur->w_satCount);
    if (w_iSatCur >= (pz_RTKcorrBlockCur->w_satCount) || (pz_RTKcorrBlockCur->pz_satPosClk[w_iSatCur].q_iode) < 0)
    {
      continue;
    }
    w_iSatPre = rtk_getIndexInCorrSatPosClk(u_constell, pz_corrMeasPre->u_svid, pz_RTKcorrBlockPre->pz_satPosClk, pz_RTKcorrBlockPre->w_satCount);
    if (w_iSatPre >= (pz_RTKcorrBlockPre->w_satCount) || (pz_RTKcorrBlockPre->pz_satPosClk[w_iSatPre].q_iode) < 0
      || (pz_RTKcorrBlockCur->pz_satPosClk[w_iSatCur].q_iode) != (pz_RTKcorrBlockPre->pz_satPosClk[w_iSatPre].q_iode))
    {
      continue;
    }
    // calcu geo dist
    d_wave = wavelength(pz_corrMeasCur->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    pd_baseSatPosClkCur = pz_RTKcorrBlockCur->pz_satPosClk[w_iSatCur].d_satPosClk;
    for (u_i = 0; u_i < 3; u_i++)
    {
      pd_s2r[u_i] = pd_baseSatPosClkCur[u_i] - pz_RTKcorrBlockCur->d_refPosEcef[u_i];
    }
    d_curDist = gnss_Norm(pd_s2r, 3);
    d_baseEarthRotCur = (pd_baseSatPosClkCur[0] * pz_RTKcorrBlockCur->d_refPosEcef[1] -
      pd_baseSatPosClkCur[1] * pz_RTKcorrBlockCur->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;
    pd_baseSatPosClkPre = pz_RTKcorrBlockPre->pz_satPosClk[w_iSatPre].d_satPosClk;
    for (u_i = 0; u_i < 3; u_i++)
    {
      pd_s2r[u_i] = pd_baseSatPosClkPre[u_i] - pz_RTKcorrBlockPre->d_refPosEcef[u_i];
    }
    d_preDist = gnss_Norm(pd_s2r, 3);
    d_baseEarthRotPre = (pd_baseSatPosClkPre[0] * pz_RTKcorrBlockPre->d_refPosEcef[1] -
      pd_baseSatPosClkPre[1] * pz_RTKcorrBlockPre->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;

    if (u_refIdx == RTK_CORR_MAX_REF_SAT_NUM || pz_corrMeasCur->f_cn0 > f_refSNR)
    {
      u_refIdx = u_epochDiffNum;
      f_refSNR = pz_corrMeasCur->f_cn0;
    }

    pd_epochDiff[u_epochDiffNum] =
      ((pz_corrMeasCur->d_carrierPhase * d_wave - d_curDist - d_baseEarthRotCur + pd_baseSatPosClkCur[3]) -
        (pz_corrMeasPre->d_carrierPhase * d_wave - d_preDist - d_baseEarthRotPre + pd_baseSatPosClkPre[3])) / d_wave;
    pu_epochDiffIndex[u_epochDiffNum] = gnss_cvt_Svid2SvIndex(u_svid, u_constell);
    ++u_epochDiffNum;
  }
  rtk_tdMaskTargetSysFreqCycleSlip(z_targetSys, z_targetFreq, u_epochDiffNum, pd_epochDiff, pu_epochDiffIndex, u_refIdx, pz_RTKcorrSlipFlag);
  OS_FREE(pd_epochDiff);
  OS_FREE(pu_epochDiffIndex);
  return;
}
/**
 * @brief the inteface of using tri-difference method to detect ref corr's slip
 * @param[in]       pz_RTKcorrBlockCur is the current RTK VRS observations and coordinate
 * @param[in]       pz_RTKcorrBlockPre is the previous RTK VRS observations and coordinate
 * @param[out]      pz_RTKcorrSlipFlag saves Detect result
 * @return     void
 */
void rtk_tdDetectCycleSlip(const GnssCorrBlock_t* pz_RTKcorrBlockCur, const GnssCorrBlock_t* pz_RTKcorrBlockPre,
  GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag)
{
  uint8_t u_iconstell = 0;
  gnss_FreqType z_iFreq = C_GNSS_FREQ_TYPE_MAX;
  for (u_iconstell = C_GNSS_GPS; u_iconstell < C_GNSS_MAX; u_iconstell++)
  {
    for (z_iFreq = 0; z_iFreq < C_GNSS_FREQ_TYPE_MAX; z_iFreq++)
    {
      rtk_tdDetectTargetSysFreqCycleSlip(u_iconstell, z_iFreq, pz_RTKcorrBlockCur, pz_RTKcorrBlockPre, pz_RTKcorrSlipFlag);
    }
  }
  return;
}
/**
 * @brief detect the slip of RTK VRS observations
 * @param[in]       pz_RTKcorrBlockCur is the current RTK VRS observations and coordinate
 * @param[in]       pz_RTKcorrBlockPre is the previous RTK VRS observations and coordinate
 * @param[in/out]   pz_gfFilterSet is the filter information used by GF method
 * @param[out]      pz_RTKcorrSlipFlag saves Detect result
 * @return   void
 */
void rtk_corrDetectCycleSlip(cmn_turboEditFilterSet* pz_gfFilterSet, GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag,
  const GnssCorrBlock_t* pz_RTKcorrBlockCur, const GnssCorrBlock_t* pz_RTKcorrBlockPre)
{
  if (0 != pz_RTKcorrBlockPre->w_refStationId)
  {
    rtk_gfDetectCycleSlip(pz_RTKcorrBlockCur, pz_gfFilterSet, pz_RTKcorrSlipFlag);

    if (fabs(pz_RTKcorrBlockCur->d_refPosEcef[0] - pz_RTKcorrBlockPre->d_refPosEcef[0]) < FABS_ZEROS &&
      fabs(pz_RTKcorrBlockCur->d_refPosEcef[1] - pz_RTKcorrBlockPre->d_refPosEcef[1]) < FABS_ZEROS &&
      fabs(pz_RTKcorrBlockCur->d_refPosEcef[2] - pz_RTKcorrBlockPre->d_refPosEcef[2]) < FABS_ZEROS)
    {
      if (tm_GpsTimeDiff(&(pz_RTKcorrBlockCur->z_Clock.z_gpsTime), &(pz_RTKcorrBlockPre->z_Clock.z_gpsTime)) < 30.0 &&
        tm_GpsTimeDiff(&(pz_RTKcorrBlockCur->z_Clock.z_gpsTime), &(pz_RTKcorrBlockPre->z_Clock.z_gpsTime)) > FABS_ZEROS)
      {
        rtk_tdDetectCycleSlip(pz_RTKcorrBlockCur, pz_RTKcorrBlockPre, pz_RTKcorrSlipFlag);
      }
    }

  }

}

 /**
  * @brief check the Ref correct data availability
  * @param[in]  pz_GnssCorrBlockUpdate represent Ref correct data
  * @param[out] void
  * @return     true for data is valid; false for other case
  */
BOOL rtk_checkCorrValid(const GnssCorrBlock_t * pz_GnssCorrBlockUpdate)
{
  uint8_t u_i = 0;
  uint8_t u_state = FALSE;
  uint8_t u_gpsMeasNum = 0;
  uint8_t u_bdsMeasNum = 0;
  uint8_t u_galMeasNum = 0;

  //check ref position
  if (gnss_Dot(pz_GnssCorrBlockUpdate->d_refPosEcef,
    pz_GnssCorrBlockUpdate->d_refPosEcef, 3) < FABS_ZEROS * FABS_ZEROS)
  {
    return FALSE;
  }

  //check data count
  if (pz_GnssCorrBlockUpdate->w_measCount <= 6)
  {
    return FALSE;
  }

  //check satellite system integrity
  for (u_i = 0; u_i < pz_GnssCorrBlockUpdate->w_measCount; u_i++)
  {
    if (C_GNSS_GPS == pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation
      && pz_GnssCorrBlockUpdate->z_meas[u_i].z_measStatusFlag.b_valid)
    {
      u_gpsMeasNum += 1;
    }
    else if (C_GNSS_GAL == pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation
      && pz_GnssCorrBlockUpdate->z_meas[u_i].z_measStatusFlag.b_valid)
    {
      u_galMeasNum += 1;
    }
    else if ((C_GNSS_BDS3 == pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation || C_GNSS_BDS2 == pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation)
      && pz_GnssCorrBlockUpdate->z_meas[u_i].z_measStatusFlag.b_valid)
    {
      u_bdsMeasNum += 1;
    }
  }
  //(update in future)if ((0 != ((pz_RTKfilterInfo->z_opt.z_usedSys) & ALGO_GPS_SYS) && d_gpsMeasNum < 4)
  //	|| (0 != ((pz_RTKfilterInfo->z_opt.z_usedSys) & ALGO_BDS_SYS) && d_bdsMeasNum < 4)
  //	|| (0 != ((pz_RTKfilterInfo->z_opt.z_usedSys) & ALGO_GAL_SYS) && d_galMeasNum < 4))
  if (u_gpsMeasNum >= 3 && u_bdsMeasNum >= 3 /*|| u_galMeasNum < 3*/)
  {
    u_state = TRUE;
    pu_multiSysFlag = TRUE;
  }
  else if ((FALSE == pu_multiSysFlag) && (u_gpsMeasNum > 7 || u_bdsMeasNum > 7))
  {
    u_state = TRUE;
  }

  return u_state;
}
/**
 * @brief check the Ref correct data fit Ref position or not
 * @param[in]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[in]  pd_refpos2Check represent Ref position which need to be checked
 * @param[out] pz_GnssCorrBlockUpdate system availability
 * @return     true for data fit position; false for other case
 */
BOOL rtk_checkCorrPos(GnssCorrBlock_t* pz_GnssCorrBlockUpdate, const double pd_refpos2Check[3])
{
  uint8_t u_sum = 0;
  uint8_t u_errSum = 0;
  uint8_t u_constellation = 0;
  uint8_t u_constellationCorr = 0;
  uint8_t u_svid = 0;
  uint8_t u_sys = 0;
  uint8_t u_freq = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint16_t w_iSat = 0;
  BOOL b_valid = FALSE;
  double d_sec = 0.0;
  double d_dist = 0.0;
  double d_earthRot = 0.0;
  double d_median = 0.0;
  double d_std = 0.0;
  double pd_s2r[3] = { 0.0 };
  double pd_diff[MAX_GNSS_TRK_MEAS_NUMBER] = { 0 };
  const gnss_SatPosClkInfo_t* pz_satPosClk = NULL;
  for (u_sys = C_GNSS_GPS; u_sys < C_GNSS_MAX; u_sys++)
  {
    if (C_GNSS_BDS2 == u_sys)
    {
      continue;
    }
    for (u_freq = 0; u_freq < C_GNSS_FREQ_TYPE_MAX; ++u_freq)
    {
      u_sum = 0;
      for (u_i = 0; u_i < pz_GnssCorrBlockUpdate->w_measCount; u_i++)
      {
        u_constellationCorr = pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation;
        if (C_GNSS_BDS2 == u_constellationCorr)
        {
          u_constellationCorr = C_GNSS_BDS3;
        }
        if (u_sys != u_constellationCorr
          //|| (update in future)(0 == ((gz_RTKfilterInfo->z_opt.z_usedSys) & u_sys))
          || u_freq != gnss_cvt_Sig2FreqType(pz_GnssCorrBlockUpdate->z_meas[u_i].u_signal)
          //|| (update in future)(0 == ((gz_RTKfilterInfo.z_opt.z_usedFreq) & u_freq))
          || !pz_GnssCorrBlockUpdate->z_meas[u_i].z_measStatusFlag.b_valid
          || !pz_GnssCorrBlockUpdate->z_meas[u_i].z_measStatusFlag.b_prValid)
        {
          continue;
        }
        if (u_sum >= MAX_GNSS_TRK_MEAS_NUMBER)
        {
          break;
        }
        // 1. use pseudoRange calculate rough tot
        u_constellation = pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation;
        u_svid = pz_GnssCorrBlockUpdate->z_meas[u_i].u_svid;
        w_iSat = rtk_getIndexInCorrSatPosClk(u_constellation, u_svid, pz_GnssCorrBlockUpdate->pz_satPosClk, pz_GnssCorrBlockUpdate->w_satCount);
        if (w_iSat >= (pz_GnssCorrBlockUpdate->w_satCount))
        {
          continue;
        }
        pz_satPosClk = &(pz_GnssCorrBlockUpdate->pz_satPosClk[w_iSat]);  
        // 2. calculate pseudoRange residual error (contain Atmospheric error and clock bias)
        for (u_j = 0; u_j < 3; u_j++)
        {
          pd_s2r[u_j] = pz_satPosClk->d_satPosClk[u_j] - pd_refpos2Check[u_j];
        }
        d_dist = gnss_Norm(pd_s2r, 3);
        d_earthRot = (pz_satPosClk->d_satPosClk[0] * pd_refpos2Check[1] -
          pz_satPosClk->d_satPosClk[1] * pd_refpos2Check[0]) * OMGE_GPS / CLIGHT;
        pd_diff[u_sum++] = pz_GnssCorrBlockUpdate->z_meas[u_i].d_pseudoRange -
          (d_dist + d_earthRot - pz_satPosClk->d_satPosClk[3]);
      }
      // 3. if there has one system that pr residual error std < 50, ref correct data fit position
      if (u_sum < 4)
      {
        continue;
      }
      d_median = 0.0;
      if (FALSE == gnss_ascSortMedianDouble(pd_diff, u_sum, &d_median))
      {
        continue;
      }
      d_std = 0.0; u_errSum = 0;
      for (u_j = 0; u_j < u_sum; u_j++)
      {
        d_std += (pd_diff[u_j] - d_median) * (pd_diff[u_j] - d_median);
        if (fabs(pd_diff[u_j] - d_median) > 100.0)
        {
          u_errSum++;
        }
      }
      d_std = sqrt(d_std / (u_sum - 1));
      if (d_std < 50.0)
      {
        b_valid |= TRUE;
      }
      else if (u_errSum > u_sum / 4.0)
      {
        //4. if one system's pr residual error > 25%, set system invalid
        LOGW(TAG_HPP, "--CorrUpdate-- WARN: A large number of abnormal obs occur in %s\n", system_char[u_sys]);
        for (u_i = 0; u_i < pz_GnssCorrBlockUpdate->w_measCount; u_i++)
        {
          u_constellationCorr = pz_GnssCorrBlockUpdate->z_meas[u_i].u_constellation;
          if (C_GNSS_BDS2 == u_constellationCorr)
          {
            u_constellationCorr = C_GNSS_BDS3;
          }
          if (u_sys != u_constellationCorr)
          {
            continue;
          }
          pz_GnssCorrBlockUpdate->z_meas[u_i].z_measStatusFlag.b_valid = 0;
        }
      }
    }
  }

  return b_valid;
}

/**
 * @brief      check ref station's position changes or not
 * @param[in]      pz_rtkCorrBlock[2] is the OSR correction
 * @return     REF_AMB_CHANGE_POSTPONE = 2  for enter next step
               REF_AMB_CHANGE_SUCC     = 1  for update success or no need to update;
               REF_AMB_CHANGE_ERROR    = 0  for error case, need to reset AMB;
 */
uint8_t rtk_refPosChangeCheck(GnssCorrBlock_t* pz_RTKcorrBlock[2])
{
  BOOL b_valid = FALSE;
  BOOL b_posChange = FALSE;
  BOOL b_idChange = FALSE;

  if (pz_RTKcorrBlock[0]->w_refStationId != 0 )
  {
    if (gnss_Dot(pz_RTKcorrBlock[0]->d_refPosEcef,
      pz_RTKcorrBlock[0]->d_refPosEcef, 3) > FABS_ZEROS * FABS_ZEROS)
    {
      b_valid = TRUE;
    }
  }
  if (!b_valid)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Ref0 id/pos invalid\n");
    return REF_AMB_CHANGE_ERROR;
  }
  b_valid = FALSE;
  if (pz_RTKcorrBlock[1]->w_refStationId != 0)
  {
    if (gnss_Dot(pz_RTKcorrBlock[1]->d_refPosEcef,
      pz_RTKcorrBlock[1]->d_refPosEcef, 3) > FABS_ZEROS * FABS_ZEROS)
    {
      b_valid = TRUE;
    }
  }
  if (!b_valid)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Ref1 id/pos invalid\n");
    return REF_AMB_CHANGE_SUCC;
  }
  if (pz_RTKcorrBlock[0]->w_refStationId != pz_RTKcorrBlock[1]->w_refStationId)
  {
    b_idChange = TRUE;
  }
  if ((fabs(pz_RTKcorrBlock[0]->d_refPosEcef[0] - pz_RTKcorrBlock[1]->d_refPosEcef[0]) > FABS_ZEROS ||
    fabs(pz_RTKcorrBlock[0]->d_refPosEcef[1] - pz_RTKcorrBlock[1]->d_refPosEcef[1]) > FABS_ZEROS ||
    fabs(pz_RTKcorrBlock[0]->d_refPosEcef[2] - pz_RTKcorrBlock[1]->d_refPosEcef[2]) > FABS_ZEROS))
  {
    b_posChange = TRUE;
  }

  if (!b_posChange && !b_idChange)
  {
    //station doesn't change
    LOGI(TAG_HPP, "--RefAmbUpdate-- INFO: Station dosen't change\n");
    return REF_AMB_CHANGE_SUCC;
  }
  else if (!b_posChange && b_idChange)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Ref id changes but position not\n");
    return REF_AMB_CHANGE_SUCC;
  }
  else if (b_posChange && !b_idChange)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Ref position changes but id not\n");
  }

  return REF_AMB_CHANGE_POSTPONE;
}
/**
 * @brief      while ref station changes, update amb
 * @param[in/out]  pz_rtkCorrBlock[2] is the OSR correction
 * @param[in]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pz_preFixedAmbPool is the pool of prefix info
 * @return     sum of updated AMB;
 */
uint8_t rtk_updateAmbForRefChange(GnssCorrBlock_t* pz_RTKcorrBlock[2], rtk_filterInfo_t* pz_RTKfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_rtkEKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool)
{
  uint8_t u_isys = 0;
  uint8_t u_freq = 0;
  uint8_t u_iref = 0;
  uint8_t u_DD_iref = 0;
  uint8_t u_svid = 0;
  uint8_t u_constellation = 0;
  uint8_t u_sys = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_satIndex = 0;
  uint8_t u_sum = 0;
  uint8_t u_sumALL = 0;
  uint8_t u_matchNum = 0;
  char c_sat[4] = "";
  char c_sat_ref[4] = "";
  uint16_t w_i = 0;
  uint16_t w_isat = 0;
  int16_t w_index = 0;
  uint16_t pw_diffIndex[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0 };
  float f_ele = 0.0; 
  float f_azel[2] = { 0.0 };
  double d_dist = 0.0;
  double d_earthRot = 0.0;
  double d_wave = 0.0;
  double d_diff0 = 0.0;
  double d_diff1 = 0.0;
  double d_median = 0.0;
  double pd_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_s2r[3] = { 0.0 };
  double pd_lla[3] = { 0.0 };
  gnss_SatPosClkInfo_t* pz_satPosClk = NULL;
  gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;

  //0. set prefix amb switch flag
  if (NULL != pz_preFixedAmbPool)
  {
    for (u_freq = 0; u_freq < MAX_GNSS_SIGNAL_FREQ; u_freq++)
    {
      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
        if (NULL == pz_fixedAmbSet)
        {
          continue;
        }
        if (pz_fixedAmbSet->u_fix[u_freq] == GNSS_AMB_FLAG_NONE ||
          pz_fixedAmbSet->u_fix[u_freq] == GNSS_AMB_FLAG_FLOAT)
        {
          continue;
        }
        pz_fixedAmbSet->u_fix[u_freq] = GNSS_AMB_FLAG_TOSWITCH;
      }
    }
  }

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    for (u_freq = 0; u_freq < C_GNSS_FREQ_TYPE_MAX; ++u_freq)
    {
      u_sum = 0; u_iref = MAX_GNSS_TRK_MEAS_NUMBER; f_ele = 0.0;
      for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
      {
        // 1. iterate through RTK_EKFstateRepPool's AMB paramete
        if (NULL == pz_rtkEKFstateRepPool->pz_satPool[w_i])
        {
          continue;
        }
        w_index = pz_rtkEKFstateRepPool->pz_satPool[w_i]->w_index;
        if (w_index < 0 ||
          u_freq != convertFilterType2FreqAmb((gnss_FilterStateEnumType)pz_rtkEKFstateRepPool->pz_satPool[w_i]->w_id[2]))
        {
          continue;
        }
        u_svid = gnss_cvt_SvIndex2Svid((uint8_t)pz_rtkEKFstateRepPool->pz_satPool[w_i]->w_id[1], &u_constellation);
        if (u_svid == ALL_GNSS_SYS_SV_NUMBER || u_constellation == C_GNSS_MAX)
        {
          continue;
        }
        if (u_constellation != u_isys)
        {
          continue;
        }

        // 2. find AMB paramete's Corresponding meas in gpz_RtkCorrBlock[0]
        for (u_i = 0; u_i < pz_RTKcorrBlock[0]->w_measCount; u_i++)
        {
          if (u_constellation != pz_RTKcorrBlock[0]->z_meas[u_i].u_constellation)
            //|| (update in future)(0 == ((gz_RTKfilterInfo->z_opt.z_usedSys) & u_isys))
          {
            continue;
          }
          if (!pz_RTKcorrBlock[0]->z_meas[u_i].z_measStatusFlag.b_valid
            || !pz_RTKcorrBlock[0]->z_meas[u_i].z_measStatusFlag.b_prValid
            || !pz_RTKcorrBlock[0]->z_meas[u_i].z_measStatusFlag.b_cpValid)
          {
            continue;
          }
          if (u_svid == pz_RTKcorrBlock[0]->z_meas[u_i].u_svid
            && u_freq == gnss_cvt_Sig2FreqType(pz_RTKcorrBlock[0]->z_meas[u_i].u_signal))
          {
            break;
          }
        }
        if (u_i == pz_RTKcorrBlock[0]->w_measCount)
        {
          initEKFstateRepresent(pz_rtkEKFstateRepPool->pz_satPool[w_i], pz_RTKfilterInfo->w_nmax,
            pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q);
          pz_RTKfilterInfo->pq_paraValid[w_i] = FALSE;
          continue;
        }

        // 3. find AMB paramete's Corresponding meas in gpz_RtkCorrBlock[1]
        for (u_j = 0; u_j < pz_RTKcorrBlock[1]->w_measCount; u_j++)
        {
          if (u_constellation != pz_RTKcorrBlock[1]->z_meas[u_j].u_constellation)
            //|| (update in future)(0 == ((gz_RTKfilterInfo->z_opt.z_usedSys) & u_isys))
          {
            continue;
          }
          if (!pz_RTKcorrBlock[1]->z_meas[u_j].z_measStatusFlag.b_valid
            || !pz_RTKcorrBlock[1]->z_meas[u_j].z_measStatusFlag.b_prValid
            || !pz_RTKcorrBlock[1]->z_meas[u_j].z_measStatusFlag.b_cpValid)
          {
            continue;
          }
          if (u_svid == pz_RTKcorrBlock[1]->z_meas[u_j].u_svid
            && u_freq == gnss_cvt_Sig2FreqType(pz_RTKcorrBlock[1]->z_meas[u_j].u_signal))
          {
            break;
          }
        }
        if (u_j == pz_RTKcorrBlock[1]->w_measCount)
        {
          initEKFstateRepresent(pz_rtkEKFstateRepPool->pz_satPool[w_i], pz_RTKfilterInfo->w_nmax,
            pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q);
          pz_RTKfilterInfo->pq_paraValid[w_i] = FALSE;
          continue;
        }

        // 4   use pseudoRange calculate rough tot of ref0,
        //       then calculate phase residual error (contain Atmospheric error and clock bias)
        w_isat = rtk_getIndexInCorrSatPosClk(u_constellation, u_svid, pz_RTKcorrBlock[0]->pz_satPosClk,
          pz_RTKcorrBlock[0]->w_satCount);
        if (w_isat >= (pz_RTKcorrBlock[0]->w_satCount))
        {
          continue;
        }
        pz_satPosClk = &(pz_RTKcorrBlock[0]->pz_satPosClk[w_isat]);
        if (gnss_Dot(pz_satPosClk->d_satPosClk, pz_satPosClk->d_satPosClk, 3) < FABS_ZEROS * FABS_ZEROS)
        {
          continue;
        }
        for (u_k = 0; u_k < 3; u_k++)
        {
          pd_s2r[u_k] = pz_satPosClk->d_satPosClk[u_k] - pz_RTKcorrBlock[0]->d_refPosEcef[u_k];
        }
        d_dist = gnss_Norm(pd_s2r, 3);
        d_earthRot = (pz_satPosClk->d_satPosClk[0] * pz_RTKcorrBlock[0]->d_refPosEcef[1] -
          pz_satPosClk->d_satPosClk[1] * pz_RTKcorrBlock[0]->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;
        d_wave = wavelength(pz_RTKcorrBlock[0]->z_meas[u_i].u_signal);
        if (d_wave >= 1.0)
        {
          continue;
        }
        d_diff0 = pz_RTKcorrBlock[0]->z_meas[u_i].d_carrierPhase * d_wave -
          (d_dist + d_earthRot - pz_satPosClk->d_satPosClk[3]);

        // 5   use pseudoRange calculate rough tot of ref1,
        //       then calculate phase residual error (contain Atmospheric error and clock bias)
        w_isat = rtk_getIndexInCorrSatPosClk(u_constellation, u_svid, pz_RTKcorrBlock[1]->pz_satPosClk,
          pz_RTKcorrBlock[1]->w_satCount);
        if (w_isat >= (pz_RTKcorrBlock[1]->w_satCount))
        {
          continue;
        }
        pz_satPosClk = &(pz_RTKcorrBlock[1]->pz_satPosClk[w_isat]);
        if (gnss_Dot(pz_satPosClk->d_satPosClk, pz_satPosClk->d_satPosClk, 3) < FABS_ZEROS * FABS_ZEROS)
        {
          continue;
        }
        for (u_k = 0; u_k < 3; u_k++)
        {
          pd_s2r[u_k] = pz_satPosClk->d_satPosClk[u_k] - pz_RTKcorrBlock[1]->d_refPosEcef[u_k];
        }
        d_dist = gnss_Norm(pd_s2r, 3);
        d_earthRot = (pz_satPosClk->d_satPosClk[0] * pz_RTKcorrBlock[1]->d_refPosEcef[1] -
          pz_satPosClk->d_satPosClk[1] * pz_RTKcorrBlock[1]->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;
        d_diff1 = pz_RTKcorrBlock[1]->z_meas[u_j].d_carrierPhase * d_wave -
          (d_dist + d_earthRot - pz_satPosClk->d_satPosClk[3]);

        //6 select ref sat
        gnss_unitVector(pz_RTKcorrBlock[1]->d_refPosEcef, pz_satPosClk->d_satPosClk, pd_unitVector);
        gnss_Ecef2Lla(pz_RTKcorrBlock[1]->d_refPosEcef, pd_lla);
        gnss_Satazel(pd_lla, pd_unitVector, f_azel);
        if (f_azel[1] < 10.0 * DEG2RAD)
        {
          initEKFstateRepresent(pz_rtkEKFstateRepPool->pz_satPool[w_i], pz_RTKfilterInfo->w_nmax,
            pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q);
          pz_RTKfilterInfo->pq_paraValid[w_i] = FALSE;
          continue;
        }
        else if (f_azel[1] > f_ele || u_iref == MAX_GNSS_TRK_MEAS_NUMBER)
        {
          f_ele = f_azel[1];
          u_iref = u_sum;
        }
        pw_diffIndex[u_sum] = w_i;
        pd_diff[u_sum++] = (d_diff1 - d_diff0) / d_wave;

      }
      //(update in future) trop deley calcu
      // 7.1 update prefix amb with double difference
      if (NULL != pz_preFixedAmbPool)
      {
        u_matchNum = 0;
        for (u_k = 0; u_k < u_sum; u_k++)
        {
          w_i = pw_diffIndex[u_k];
          u_satIndex = (uint8_t)pz_rtkEKFstateRepPool->pz_satPool[w_i]->w_id[1];
          pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
          if (NULL == pz_fixedAmbSet)
          {
            continue;
          }
          if (pz_fixedAmbSet->u_fix[u_freq] != GNSS_AMB_FLAG_TOSWITCH)
          {
            continue;
          }
          u_DD_iref = u_k;
          u_matchNum++;
        }
        if (u_matchNum > 1)
        {
          w_i = pw_diffIndex[u_DD_iref];
          u_satIndex = (uint8_t)pz_rtkEKFstateRepPool->pz_satPool[w_i]->w_id[1];
          pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
          pz_fixedAmbSet->u_fix[u_freq] = GNSS_AMB_FLAG_FIXED;
          satidx_SatString(u_satIndex, c_sat_ref);
          for (u_k = 0; u_k < u_sum; u_k++)
          {
            w_i = pw_diffIndex[u_k];
            u_satIndex = (uint8_t)pz_rtkEKFstateRepPool->pz_satPool[w_i]->w_id[1];
            pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
            if (u_k == u_DD_iref || NULL == pz_fixedAmbSet ||
              pz_fixedAmbSet->u_fix[u_freq] != GNSS_AMB_FLAG_TOSWITCH ||
              fabs(pd_diff[u_k] - pd_diff[u_DD_iref] - round(pd_diff[u_k] - pd_diff[u_DD_iref])) > 0.3)
            {
              continue;
            }
            if (fabs(round(pd_diff[u_k] - pd_diff[u_DD_iref])) > 1e-5)
            {
              satidx_SatString(u_satIndex, c_sat);
              LOGI(TAG_HPP, " -RefAmbUpdate-  Update prefix AMB | ref=%s freq=%d | sat=%s amb=%.1f\n",
                c_sat_ref, u_freq, c_sat, round(pd_diff[u_k] - pd_diff[u_DD_iref]));
            }
            pz_fixedAmbSet->d_fixedValueRTK[u_freq] += round(pd_diff[u_k] - pd_diff[u_DD_iref]);
            pz_fixedAmbSet->u_fix[u_freq] = GNSS_AMB_FLAG_FIXED;
          }
        }
      }
      
      // 7.2 update float amb with double difference
      if (u_sum < 3)
      {
        for (u_k = 0; u_k < u_sum; u_k++)
        {
          w_i = pw_diffIndex[u_k];
          initEKFstateRepresent(pz_rtkEKFstateRepPool->pz_satPool[w_i], pz_RTKfilterInfo->w_nmax,
            pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->pd_Q);
          pz_RTKfilterInfo->pq_paraValid[w_i] = FALSE;
        }
        continue;
      }
      u_sumALL += u_sum;
      d_median = 0.0;
      gnss_ascSortMedianDouble(pd_diff, u_sum, &d_median);
      satidx_SatString((uint8_t)pz_rtkEKFstateRepPool->pz_satPool[pw_diffIndex[u_iref]]->w_id[1], c_sat_ref);
      for (u_k = 0; u_k < u_sum; u_k++)
      {
        w_index = pz_rtkEKFstateRepPool->pz_satPool[pw_diffIndex[u_k]]->w_index;
        if (w_index < 0)
        {
          continue;
        }
        if (fabs(d_median) > 10.0)
        {
          //(update in future) take std in consideration, then patch clock bias, or error will occur
          pz_RTKfilterInfo->pd_X[w_index] += d_median;
        }
        if (u_k == u_iref)
        {
          continue;
        }
        /*(update in future) amb update of prefix module
        if (fabs(round(pd_diff[u_k] - pd_diff[u_iref])) > 0.7)
        {
          pz_RTKfilterInfo->pd_X[pw_diffIndex[u_k]] += round(pd_diff[u_k] - pd_diff[u_iref]);
        }*/
        satidx_SatString((uint8_t)pz_rtkEKFstateRepPool->pz_satPool[pw_diffIndex[u_k]]->w_id[1], c_sat);
        LOGI(TAG_HPP, " -RefAmbUpdate-  Update float AMB | ref=%s freq=%d | sat=%s amb=%.1f\n",
          c_sat_ref, u_freq, c_sat, (pd_diff[u_k] - pd_diff[u_iref]));
        pz_RTKfilterInfo->pd_X[w_index] += (pd_diff[u_k] - pd_diff[u_iref]);
      }
    }
  }

  //8. clean prefix amb that don't switch after station switch
  if (NULL != pz_preFixedAmbPool)
  {
    for (u_freq = 0; u_freq < MAX_GNSS_SIGNAL_FREQ; u_freq++)
    {
      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
        if (NULL == pz_fixedAmbSet)
        {
          continue;
        }
        if (GNSS_AMB_FLAG_TOSWITCH == pz_fixedAmbSet->u_fix[u_freq])
        {
          satidx_SatString(u_satIndex, c_sat);
          LOGI(TAG_HPP, " -RefAmbUpdate-  Reset Prefix AMB for None obs match | sat=%s freq=%d \n", c_sat, u_freq);
          pz_fixedAmbSet->u_fix[u_freq] = GNSS_AMB_FLAG_NONE;
          pz_fixedAmbSet->u_continue_fix[u_freq] = 0;
          pz_fixedAmbSet->d_fixedValueRTK[u_freq] = 0;
        }
      }
    }
  }

  return u_sumALL;
}
/**
 * @brief      while ref station changes, check amb offset and update it
 * @param[in/out]  pz_rtkCorrBlock[2] is the OSR correction
 * @param[in]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[out]     pz_RTKcorrSlipFlag is Correction satellite SlipFlag 
 * @param[out]     pz_preFixedAmbPool is the pool of prefix info
 * @return     REF_AMB_CHANGE_POSTPONE = 2  for it's not a good time to update amb, Postpone the update
               REF_AMB_CHANGE_SUCC     = 1  for update success or no need to update;
               REF_AMB_CHANGE_ERROR    = 0  for error case, need to reset AMB;
 */
uint8_t rtk_refChangeCheckAndUpdateAmb(GnssCorrBlock_t* pz_RTKcorrBlock[2], rtk_filterInfo_t* pz_RTKfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_rtkEKFstateRepPool, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool)
{
  uint8_t u_sum = 0;
  uint8_t u_constellation = 0;
  uint8_t u_svid = 0;
  /*uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_sys = 0;
  uint8_t u_k = 0;
  uint8_t u_iref = 0;
  gnss_FreqType u_freq = 0;
  uint16_t w_iMeas = 0;
  uint16_t w_index = 0;
  uint16_t pw_diffIndex[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0 };
  uint16_t w_isat = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  int16_t w_ambIndex = -1;*/
  BOOL b_valid = 0;
  /*float f_azel[2] = { 0.0 };
  float f_ele =  0.0;
  double d_sec = 0.0;
  double d_dist = 0.0;
  double d_wave = 0.0;
  double d_earthRot = 0.0;*/
  double d_deltaTime = 0.0;
  /*double d_diff0 = 0.0;
  double d_diff1 = 0.0;
  double pd_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_s2r[3] = { 0.0 };
  const gnss_SatPosClkInfo_t* pz_satPosClk = NULL;*/

  // 1.check ref position changes or not
  b_valid = rtk_refPosChangeCheck(pz_RTKcorrBlock);
  if (b_valid == REF_AMB_CHANGE_SUCC)
  {
    return REF_AMB_CHANGE_SUCC;
  }
  else if (b_valid == REF_AMB_CHANGE_ERROR)
  {
    return REF_AMB_CHANGE_ERROR;
  }
  
  // 2. check the time (tm_GpsTimeDiff
  d_deltaTime = tm_GpsTimeDiff(&pz_RTKcorrBlock[0]->z_Clock.z_gpsTime,
    &pz_RTKcorrBlock[1]->z_Clock.z_gpsTime);
  if (d_deltaTime < FABS_ZEROS)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Ref time incorrect, dt=%.2f, use old ref corr\n", d_deltaTime);
    rtk_reCalculateCorrDataSatPosClkByRoverIode(pz_satSigMeasCollect, pz_RTKcorrBlock[1]);
    return REF_AMB_CHANGE_POSTPONE;
  }
  if (d_deltaTime > 10.0)//(update in future)(config.max_delta_t)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Too long interval while station changes, dt=%.2f\n", d_deltaTime);
    return REF_AMB_CHANGE_ERROR;
  }

  // 3. check that AMB already updated or not
  if (pz_RTKcorrBlock[0]->w_switchStationId == 0 && pz_RTKcorrBlock[1]->w_refStationId == 0)
  {
    LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: Ref switch flag is tampered\n");
    return REF_AMB_CHANGE_ERROR;
  }
  else if (pz_RTKcorrBlock[1]->w_refStationId != pz_RTKcorrBlock[1]->w_switchStationId)
  {
    if (pz_RTKcorrBlock[1]->w_switchStationId == pz_RTKcorrBlock[0]->w_refStationId)
    {
      //already updated
      LOGI(TAG_HPP, "--RefAmbUpdate-- INFO: AMB already update\n");
      return REF_AMB_CHANGE_SUCC;
    }
    else
    {
      LOGW(TAG_HPP, "--RefAmbUpdate-- WARN: station change more than once before enter float filter\n");
      return REF_AMB_CHANGE_ERROR;
    }
  }

  // 4. check that sat num
  if (pz_RTKcorrBlock[1]->w_measCount <= 0)
  {
    LOGI(TAG_HPP, "--RefAmbUpdate-- WARN: no date in ref1\n");
    return REF_AMB_CHANGE_ERROR;
  }
  if (pz_RTKcorrBlock[0]->w_measCount < 6 ||
    pz_RTKcorrBlock[0]->w_measCount < pz_RTKcorrBlock[1]->w_measCount / 2)
  {
    if (d_deltaTime < 5.0)//(update in future)(config.max_delta_t/2)
    {
      LOGI(TAG_HPP, "--RefAmbUpdate-- WARN: too few sat num, postpone AMB update, ref0=%d, ref1=%d\n",
        pz_RTKcorrBlock[0]->w_measCount, pz_RTKcorrBlock[1]->w_measCount);
      rtk_reCalculateCorrDataSatPosClkByRoverIode(pz_satSigMeasCollect, pz_RTKcorrBlock[1]);
      return REF_AMB_CHANGE_POSTPONE;
    }
  }

  // 5. sync nav and update AMB
  rtk_reCalculateCorrDataSatPosClkByRoverIode(pz_satSigMeasCollect, pz_RTKcorrBlock[1]);
  u_sum = rtk_updateAmbForRefChange(pz_RTKcorrBlock, pz_RTKfilterInfo, pz_rtkEKFstateRepPool,pz_preFixedAmbPool);

  pz_RTKcorrBlock[1]->w_switchStationId = pz_RTKcorrBlock[0]->w_refStationId;

  pz_RTKcorrSlipFlag->u_changeStationFlag = 1;
  LOGI(TAG_HPP, "--RefAmbUpdate-- INFO: %d float AMB updated in filter\n", u_sum);
  return REF_AMB_CHANGE_SUCC;
}
