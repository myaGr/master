#include "cmn_turboEdit.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "gnss_common.h"
#include "gnss_def.h"
#include <math.h>

/**
 * @brief the inteface of initilizing the filter information used by GF or MW method
 * @param[out]  pz_TEfilterSet is the filter information used by GF or MW method
 * @return     void
 */
void cmn_initTEfilterSet(cmn_turboEditFilterSet* pz_TEfilterSet)
{
  uint16_t w_i = 0;
  uint16_t w_totalNum = C_TE_COMBINE_MAX * ALL_GNSS_SYS_SV_NUMBER;
  if (NULL != pz_TEfilterSet)
  {
    for (w_i = 0; w_i < w_totalNum; ++w_i)
    {
      pz_TEfilterSet->pz_turboEditSet[w_i] = NULL;
    }
  }
  return;
}
/**
 * @brief the inteface of deinitilizing the filter information used by GF or MW method
 * @param[out]  pz_TEfilterSet is the filter information used by GF or MW method
 * @return     void
 */
void cmn_deinitTEfilterSet(cmn_turboEditFilterSet* pz_TEfilterSet)
{
  uint16_t w_i = 0;
  uint16_t w_totalNum = C_TE_COMBINE_MAX * ALL_GNSS_SYS_SV_NUMBER;
  if (NULL != pz_TEfilterSet)
  {
    for (w_i = 0; w_i < w_totalNum; ++w_i)
    {
      if (NULL != pz_TEfilterSet->pz_turboEditSet[w_i])
      {
        OS_FREE(pz_TEfilterSet->pz_turboEditSet[w_i]);
        pz_TEfilterSet->pz_turboEditSet[w_i] = NULL;
      }
    }
  }
  return;
}
/**
 * @brief initilize the GF filter
 * @param[in]   d_gfValue is GF combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_gfFilterInfo is the filter information used by GF method
 * @return      void
 */
void initGfFilterInfo(double d_gfValue, float f_ele, cmn_oneObsTurboEditFilterInfo* pz_gfFilterInfo)
{
  pz_gfFilterInfo->d_value = d_gfValue;
  pz_gfFilterInfo->f_eleA = f_ele;
  pz_gfFilterInfo->d_valueD = 1.0;
  pz_gfFilterInfo->d_valueQ = 0.02;
  pz_gfFilterInfo->d_velocity = 0.0;
  return;
}
/**
 * @brief initilize the MW filter
 * @param[in]   d_mwValue is MW combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_mwFilterInfo is the filter information used by MW method
 * @return      void
 */
void initMwFilterInfo(double d_mwValue, float f_ele, cmn_oneObsTurboEditFilterInfo* pz_mwFilterInfo)
{
  pz_mwFilterInfo->d_value = d_mwValue;
  pz_mwFilterInfo->f_eleA = f_ele;
  pz_mwFilterInfo->d_valueD = 1.0;
  pz_mwFilterInfo->d_valueQ = 0.21;
  pz_mwFilterInfo->d_velocity = 0.0;
  return;
}
/**
 * @brief initilize the turbo edit filter
 * @param[out]  pz_turboEditFilterInfo is the filter information used by turbo edit method
 * @return      void
 */
void cmn_initTurboEditFilterInfo(cmn_oneObsTurboEditFilterInfo* pz_turboEditFilterInfo)
{
  if (NULL != pz_turboEditFilterInfo)
  {
    pz_turboEditFilterInfo->u_isFirstEpoch = 1;
    pz_turboEditFilterInfo->u_predictFlag = 0;
    pz_turboEditFilterInfo->f_eleA = 0.0;
    pz_turboEditFilterInfo->d_value = 0.0;
    pz_turboEditFilterInfo->d_velocity = 0.0;
    pz_turboEditFilterInfo->d_valueD = 0.0;
    pz_turboEditFilterInfo->d_valueQ = 0.0;
    tm_initGpsTime(&(pz_turboEditFilterInfo->z_beginTime));
    tm_initGpsTime(&(pz_turboEditFilterInfo->z_endTime));
  }
  return;
}
/**
 * @brief filter the GF value
 * @param[in]   pz_tor is observation time
 * @param[in]   d_gfValue is GF combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_gfFilterInfo is the filter information used by GF method
 * @return      void
 */
void predictGFvalue(const GpsTime_t* pz_tor, double d_gfValue, float f_ele, cmn_oneObsTurboEditFilterInfo* pz_gfFilterInfo)
{
  double d_noiseCoeff = 0.005;
  double pd_qCoeff[2] = { 49.0,1.0 };
  double d_intv = tm_GpsTimeDiff(pz_tor, &(pz_gfFilterInfo->z_endTime));
  double d_noise = fabs(d_intv) * d_noiseCoeff;
  double d_errorValue = 0.0;
  double d_gfVelocity = 0.0;
  double d_deltaGfVelocity = 0.0;
  double d_k = 0.0;
  double d_qPre = 0.0;
  if (0 == (pz_gfFilterInfo->u_predictFlag))
  {
    initGfFilterInfo(d_gfValue, f_ele, pz_gfFilterInfo);
  }
  else
  {
    if (fabs(pz_gfFilterInfo->d_velocity) > 1.0e-12)
    {
      d_gfVelocity = (d_gfValue - pz_gfFilterInfo->d_value) / d_intv;
      d_deltaGfVelocity = d_gfVelocity - pz_gfFilterInfo->d_velocity;
      d_k = pz_gfFilterInfo->d_valueD / (d_noise + pz_gfFilterInfo->d_valueD);
      pz_gfFilterInfo->d_valueD = (1.0 - d_k) * pz_gfFilterInfo->d_valueD;
      pz_gfFilterInfo->d_velocity += (d_k * d_deltaGfVelocity);
      d_errorValue = pz_gfFilterInfo->d_velocity - d_gfVelocity;
      d_qPre = pz_gfFilterInfo->d_valueQ;
      pz_gfFilterInfo->d_valueQ = sqrt((d_qPre * d_qPre * pd_qCoeff[0] + d_errorValue * d_errorValue * pd_qCoeff[1]) / (pd_qCoeff[0] + pd_qCoeff[1]));
    }
    pz_gfFilterInfo->d_value = d_gfValue;
  }
  return;
}
/**
 * @brief filter the MW value
 * @param[in]   pz_tor is observation time
 * @param[in]   d_mwValue is MW combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_mwFilterInfo is the filter information used by MW method
 * @return      void
 */
void predictMWvalue(const GpsTime_t* pz_tor, double d_mwValue, float f_ele, cmn_oneObsTurboEditFilterInfo* pz_mwFilterInfo)
{
  double d_noiseCoeff = 0.001;
  double pd_qCoeff[2] = { 10.0,1.0 };
  double d_intv = tm_GpsTimeDiff(pz_tor, &(pz_mwFilterInfo->z_endTime));
  double d_noise = fabs(d_intv) * d_noiseCoeff;
  double d_deltaValue = 0.0;
  double d_k = 0.0;
  double d_qPre = 0.0;
  if (0 == (pz_mwFilterInfo->u_predictFlag))
  {
    initMwFilterInfo(d_mwValue, f_ele, pz_mwFilterInfo);
  }
  else
  {
    d_deltaValue = d_mwValue - pz_mwFilterInfo->d_value;
    d_k = pz_mwFilterInfo->d_valueD / (d_noise + pz_mwFilterInfo->d_valueD);
    pz_mwFilterInfo->d_value += (d_k * d_deltaValue);
    pz_mwFilterInfo->d_valueD = (1.0 - d_k) * pz_mwFilterInfo->d_valueD;
    d_qPre = pz_mwFilterInfo->d_valueQ;
    pz_mwFilterInfo->d_valueQ = sqrt((d_qPre * d_qPre * pd_qCoeff[0] + d_deltaValue * d_deltaValue * pd_qCoeff[1]) / (pd_qCoeff[0] + pd_qCoeff[1]));
  }
  return;
}
/**
 * @brief filter the GF value and detect the cycle slip
 * @param[in]   pz_tor is observation time
 * @param[in]   d_gfValue is GF combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_gfFilterInfo is the filter information used by GF method
 * @return      0 represent non-detect, 1 represent non cycle slip, 2 represent cycle slip has happened
 */
uint8_t cmn_gfFilterDetect(const GpsTime_t* pz_tor, double d_gfValue, float f_ele, cmn_oneObsTurboEditFilterInfo* pz_gfFilterInfo)
{
  uint8_t u_cycleSlipStatus = 0;
  double d_intv = 0.0;
  double d_gfThreshold = 0.0;
  double d_deltaGf = 0.0;
  double d_deltaDeltaGf = 0.0;
  double d_tempThres = 0.0;
  d_intv = fabs(tm_GpsTimeDiff(pz_tor, &(pz_gfFilterInfo->z_endTime)));
  if (1 == (pz_gfFilterInfo->u_isFirstEpoch) || d_intv > 30.0 || d_intv <= 0.0)
  {
    initGfFilterInfo(d_gfValue, f_ele, pz_gfFilterInfo);
    pz_gfFilterInfo->z_endTime = *pz_tor;
    pz_gfFilterInfo->z_beginTime = *pz_tor;
    u_cycleSlipStatus = 2;
    pz_gfFilterInfo->u_isFirstEpoch = 0;
    return u_cycleSlipStatus;
  }
  //The threshold is determined based on the sampling interval
  if (d_intv > 0.0 && d_intv <= 1.0)
  {
    d_gfThreshold = 0.6;
  }
  else if (d_intv > 1.0 && d_intv <= 15.0)
  {
    d_gfThreshold = 0.8;
  }
  // d_intv > 15.0 && d_intv <= 30.0
  else
  {
    d_gfThreshold = 1.0;
  }

  pz_gfFilterInfo->u_predictFlag = 1;
  if (fabs(pz_gfFilterInfo->d_velocity) < 1.0e-12)
  {
    if (fabs(d_gfValue - pz_gfFilterInfo->d_value) > d_gfThreshold)
    {
      u_cycleSlipStatus = 2;
      pz_gfFilterInfo->u_predictFlag = 0;
    }
    else
    {
      pz_gfFilterInfo->d_velocity = (d_gfValue - pz_gfFilterInfo->d_value) / d_intv;
      u_cycleSlipStatus = 1;
    }
  }
  else
  {
    d_deltaGf = d_gfValue - pz_gfFilterInfo->d_value;
    d_deltaDeltaGf = fabs(d_deltaGf - pz_gfFilterInfo->d_velocity * d_intv);
    d_tempThres = 3.0 * pz_gfFilterInfo->d_valueQ * d_intv;
    d_tempThres = d_tempThres > 0.9 ? 0.9 : d_tempThres;
    if (d_deltaDeltaGf >= d_tempThres && (d_deltaDeltaGf >= d_gfThreshold + 0.003 * d_intv || fabs(d_deltaGf) > d_gfThreshold))
    {
      u_cycleSlipStatus = 2;
      pz_gfFilterInfo->u_predictFlag = 0;
    }
    else
    {
      u_cycleSlipStatus = 1;
    }
  }
  predictGFvalue(pz_tor, d_gfValue, f_ele, pz_gfFilterInfo);
  pz_gfFilterInfo->z_endTime = *pz_tor;
  if (2 == u_cycleSlipStatus)
  {
    pz_gfFilterInfo->z_beginTime = *pz_tor;
  }
  pz_gfFilterInfo->f_eleA = f_ele;
  return u_cycleSlipStatus;
}
/**
 * @brief filter the MW value and detect the cycle slip
 * @param[in]   pz_tor is observation time
 * @param[in]   d_mwValue is MW combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_mwFilterInfo is the filter information used by MW method
 * @return      0 represent non-detect, 1 represent non cycle slip, 2 represent cycle slip has happened
 */
uint8_t cmn_mwFilterDetect(const GpsTime_t* pz_tor, double d_mwValue, float f_ele, cmn_oneObsTurboEditFilterInfo* pz_mwFilterInfo)
{
  uint8_t u_cycleSlipStatus = 0;
  double d_intv = 0.0;
  double d_mwThreshold = 0.7;
  double d_deltaMw = 0.0;
  d_intv = fabs(tm_GpsTimeDiff(pz_tor, &(pz_mwFilterInfo->z_endTime)));
  if (1 == (pz_mwFilterInfo->u_isFirstEpoch) || d_intv > 30.0 || d_intv <= 0.0)
  {
    initMwFilterInfo(d_mwValue, f_ele, pz_mwFilterInfo);
    pz_mwFilterInfo->z_endTime = *pz_tor;
    pz_mwFilterInfo->z_beginTime = *pz_tor;
    u_cycleSlipStatus = 2;
    pz_mwFilterInfo->u_isFirstEpoch = 0;
    return u_cycleSlipStatus;
  }
  pz_mwFilterInfo->u_predictFlag = 1;
  d_deltaMw = fabs(d_mwValue - pz_mwFilterInfo->d_value);
  if (d_deltaMw > d_mwThreshold)
  {
    u_cycleSlipStatus = 2;
    pz_mwFilterInfo->u_predictFlag = 0;
  }
  else
  {
    u_cycleSlipStatus = 1;
  }
  predictMWvalue(pz_tor, d_mwValue, f_ele, pz_mwFilterInfo);
  pz_mwFilterInfo->z_endTime = *pz_tor;
  if (2 == u_cycleSlipStatus)
  {
    pz_mwFilterInfo->z_beginTime = *pz_tor;
  }
  pz_mwFilterInfo->f_eleA = f_ele;
  return u_cycleSlipStatus;
}
/**
 * @brief get the index of turbo edit filter
 * @param[in]  w_iSat is satellite id
 * @param[in]  z_freqType is the first frequency type
 * @param[in]  z_twinFreqType is the second frequency type
 * @return     the index of turbo edit filter
 */
uint16_t cmn_getKeyValue(uint16_t w_iSat, gnss_FreqType z_freqType, gnss_FreqType z_twinFreqType)
{
  uint16_t w_index = C_TE_COMBINE_MAX;
  if ((C_GNSS_FREQ_TYPE_L1 == z_freqType && C_GNSS_FREQ_TYPE_L2 == z_twinFreqType) || (C_GNSS_FREQ_TYPE_L2 == z_freqType && C_GNSS_FREQ_TYPE_L1 == z_twinFreqType))
  {
    w_index = C_TE_L1L2;
  }
  else if ((C_GNSS_FREQ_TYPE_L1 == z_freqType && C_GNSS_FREQ_TYPE_L5 == z_twinFreqType) || (C_GNSS_FREQ_TYPE_L5 == z_freqType && C_GNSS_FREQ_TYPE_L1 == z_twinFreqType))
  {
    w_index = C_TE_L1L5;
  }
  else if ((C_GNSS_FREQ_TYPE_L2 == z_freqType && C_GNSS_FREQ_TYPE_L5 == z_twinFreqType) || (C_GNSS_FREQ_TYPE_L5 == z_freqType && C_GNSS_FREQ_TYPE_L2 == z_twinFreqType))
  {
    w_index = C_TE_L2L5;
  }
  return (w_index * ALL_GNSS_SYS_SV_NUMBER + w_iSat);
}
/**
 * @brief the inteface of using turbo edit method to detect cycle slip
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_gfFilterSet is the filter information used by GF method
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_gfDetectCycleSlip(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_turboEditFilterSet* pz_gfFilterSet)
{
  uint8_t u_status = 0;
  uint16_t w_iSat = 0;
  char char_sat[4] = "";
  uint16_t w_filterIndex = 0;
  uint16_t w_invalidIndex = C_TE_COMBINE_MAX * ALL_GNSS_SYS_SV_NUMBER;
  gnss_SignalType u_firstSignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_secondSignalType = C_GNSS_SIG_MAX;
  uint8_t u_iFreq = 0;
  uint8_t u_jFreq = 0;
  gnss_FreqType z_firstFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_FreqType z_twinFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  double d_wave1 = 0.0;
  double d_wave2 = 0.0;
  double d_gfValue = 0.0;//the wave used L1
  double d_arcLen = 0.0;
  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[w_iSat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; ++u_iFreq)
    {
      if (NULL == pz_satMeas->pz_signalMeas[u_iFreq])
      {
        continue;
      }
      u_firstSignalType = pz_satMeas->pz_signalMeas[u_iFreq]->u_signal;
      z_firstFreqType = gnss_cvt_Sig2FreqType(u_firstSignalType);
      if (fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase) < 1.0e-3)
      {
        continue;
      }
      d_wave1 = wavelength(u_firstSignalType);
      if (d_wave1 >= 1.0)
      {
        continue;
      }
      for (u_jFreq = u_iFreq+1; u_jFreq < MAX_GNSS_SIGNAL_FREQ; ++u_jFreq)
      {
        if (NULL == pz_satMeas->pz_signalMeas[u_jFreq])
        {
          continue;
        }
        if (fabs(pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase) < 1.0e-3)
        {
          continue;
        }
        u_secondSignalType = (gnss_SignalType)(pz_satMeas->pz_signalMeas[u_jFreq]->u_signal);
        z_twinFreqType = gnss_cvt_Sig2FreqType(u_secondSignalType);
        d_wave2 = wavelength(u_secondSignalType);
        if (d_wave2 >= 1.0)
        {
          continue;
        }
        d_gfValue = d_wave1 * (pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase) - d_wave2 * (pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase);
        d_gfValue /= d_wave1;
        w_filterIndex = cmn_getKeyValue(w_iSat, z_firstFreqType, z_twinFreqType);
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
        if (1 == cmn_gfFilterDetect(&(pz_satSignalCur->z_tor), d_gfValue, pz_satMeas->z_satPosVelClk.f_elevation, pz_gfFilterSet->pz_turboEditSet[w_filterIndex]))
        {
          pz_satMeas->pz_signalMeas[u_iFreq]->u_slipMask |= NON_CYCLE_SLIP_BY_GF;
          pz_satMeas->pz_signalMeas[u_jFreq]->u_slipMask |= NON_CYCLE_SLIP_BY_GF;
        }
        else
        {
          satidx_SatString((uint8_t)w_iSat, char_sat);
          LOGD(TAG_PPP, "Cycle Slip GF sat=%s\n", char_sat);
        }
        pz_satMeas->pz_signalMeas[u_iFreq]->u_slipMethodValid |= GF_DETECT_VALID;
        pz_satMeas->pz_signalMeas[u_jFreq]->u_slipMethodValid |= GF_DETECT_VALID;
        if (C_GNSS_FREQ_TYPE_L1 == u_iFreq)
        {
          pz_satMeas->f_gf[u_jFreq] = (float)(pz_gfFilterSet->pz_turboEditSet[w_filterIndex]->d_value);
          d_arcLen = fabs(tm_GpsTimeDiff(&(pz_gfFilterSet->pz_turboEditSet[w_filterIndex]->z_endTime), &(pz_gfFilterSet->pz_turboEditSet[w_filterIndex]->z_beginTime)));
          pz_satMeas->d_gfArcLen[u_jFreq] = d_arcLen;
          if (d_arcLen > (pz_satMeas->d_gfArcLen[u_iFreq]))
          {
            pz_satMeas->d_gfArcLen[u_iFreq] = d_arcLen;
          }
        }
      }
    }
  }
  return u_status;
}
void cmn_calculateMWcombination(double d_pseudoRange1, double d_carrierPhase1, double d_freqValue1,
  double d_pseudoRange2, double d_carrierPhase2, double d_freqValue2, double* pd_mwValue)
{
  double pd_pseudoCoef[2] = { 0.0 };
  double pd_carrierCoef[2] = { 0.0 };
  double d_wave = 0.0;
  pd_pseudoCoef[0] = -d_freqValue1 / (d_freqValue1 + d_freqValue2);
  pd_pseudoCoef[1] = -d_freqValue2 / (d_freqValue1 + d_freqValue2);
  pd_carrierCoef[0] = CLIGHT / (d_freqValue1 - d_freqValue2);
  pd_carrierCoef[1] = -CLIGHT / (d_freqValue1 - d_freqValue2);
  *pd_mwValue = d_carrierPhase1 * pd_carrierCoef[0] + d_carrierPhase2 * pd_carrierCoef[1]
    + pd_pseudoCoef[0] * d_pseudoRange1 + pd_pseudoCoef[1] * d_pseudoRange2;
  d_wave = CLIGHT / (d_freqValue1 - d_freqValue2);
  *pd_mwValue /= d_wave;
  return;
}
/**
 * @brief the inteface of using MW method to detect cycle slip
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_mwFilterSet is the filter information used by MW method
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_mwDetectCycleSlip(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_turboEditFilterSet* pz_mwFilterSet)
{
  uint8_t u_status = 0;
  uint16_t w_iSat = 0;
  char char_sat[4] = "";
  uint16_t w_filterIndex = 0;
  uint16_t w_invalidIndex = C_TE_COMBINE_MAX * ALL_GNSS_SYS_SV_NUMBER;
  gnss_SignalType u_firstSignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_secondSignalType = C_GNSS_SIG_MAX;
  uint8_t u_iFreq = 0;
  uint8_t u_jFreq = 0;
  gnss_FreqType z_firstFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_FreqType z_twinFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  double d_freqValue1 = 0.0;
  double d_freqValue2 = 0.0;
  double d_mwValue = 0.0;
  double d_arcLen = 0.0;
  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[w_iSat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; ++u_iFreq)
    {
      if (NULL == pz_satMeas->pz_signalMeas[u_iFreq])
      {
        continue;
      }
      u_firstSignalType = pz_satMeas->pz_signalMeas[u_iFreq]->u_signal;
      z_firstFreqType = gnss_cvt_Sig2FreqType(u_firstSignalType);
      if (fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase) < 1.0e-3
        || (pz_satMeas->pz_signalMeas[u_iFreq]->d_pseudoRange) < 1.0e-3)
      {
        continue;
      }
      d_freqValue1 = CLIGHT / wavelength(u_firstSignalType);
      if (fabs(d_freqValue1 - CLIGHT) < 1.0e-6)
      {
        continue;
      }
      for (u_jFreq = u_iFreq+1; u_jFreq < MAX_GNSS_SIGNAL_FREQ; ++u_jFreq)
      {
        if (NULL == pz_satMeas->pz_signalMeas[u_jFreq])
        {
          continue;
        }
        if (fabs(pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase) < 1.0e-3
          || (pz_satMeas->pz_signalMeas[u_jFreq]->d_pseudoRange) < 1.0e-3)
        {
          continue;
        }
        u_secondSignalType = (gnss_SignalType)(pz_satMeas->pz_signalMeas[u_jFreq]->u_signal);
        z_twinFreqType = gnss_cvt_Sig2FreqType(u_secondSignalType);
        d_freqValue2 = CLIGHT / wavelength(u_secondSignalType);
        if (fabs(d_freqValue2 - CLIGHT) < 1.0e-6)
        {
          continue;
        }
        cmn_calculateMWcombination(pz_satMeas->pz_signalMeas[u_iFreq]->d_pseudoRange, pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase, d_freqValue1,
          pz_satMeas->pz_signalMeas[u_jFreq]->d_pseudoRange, pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase, d_freqValue2, &d_mwValue);
        w_filterIndex = cmn_getKeyValue(w_iSat, z_firstFreqType, z_twinFreqType);
        if (w_filterIndex >= w_invalidIndex)
        {
          continue;
        }
        if (NULL == pz_mwFilterSet->pz_turboEditSet[w_filterIndex])
        {
          pz_mwFilterSet->pz_turboEditSet[w_filterIndex] = (cmn_oneObsTurboEditFilterInfo*)OS_MALLOC(sizeof(cmn_oneObsTurboEditFilterInfo));
          cmn_initTurboEditFilterInfo(pz_mwFilterSet->pz_turboEditSet[w_filterIndex]);
        }
        if (NULL == pz_mwFilterSet->pz_turboEditSet[w_filterIndex])
        {
          continue;
        }
        if (1 == cmn_mwFilterDetect(&(pz_satSignalCur->z_tor), d_mwValue, pz_satMeas->z_satPosVelClk.f_elevation, pz_mwFilterSet->pz_turboEditSet[w_filterIndex]))
        {
          pz_satMeas->pz_signalMeas[u_iFreq]->u_slipMask |= NON_CYCLE_SLIP_BY_MW;
          pz_satMeas->pz_signalMeas[u_jFreq]->u_slipMask |= NON_CYCLE_SLIP_BY_MW;
        }
        else
        {
          satidx_SatString((uint8_t)w_iSat, char_sat);
          LOGD(TAG_PPP, "Cycle Slip MW sat=%s\n", char_sat);
        }
        pz_satMeas->pz_signalMeas[u_iFreq]->u_slipMethodValid |= MW_DETECT_VALID;
        pz_satMeas->pz_signalMeas[u_jFreq]->u_slipMethodValid |= MW_DETECT_VALID;
        if (C_GNSS_FREQ_TYPE_L1 == u_iFreq)
        {
          d_arcLen = fabs(tm_GpsTimeDiff(&(pz_mwFilterSet->pz_turboEditSet[w_filterIndex]->z_endTime), &(pz_mwFilterSet->pz_turboEditSet[w_filterIndex]->z_beginTime)));
          pz_satMeas->f_mw[u_jFreq] = (float)(pz_mwFilterSet->pz_turboEditSet[w_filterIndex]->d_value);
          pz_satMeas->d_mwArcLen[u_jFreq] = d_arcLen;
          if (d_arcLen > (pz_satMeas->d_mwArcLen[u_iFreq]))
          {
            pz_satMeas->d_mwArcLen[u_iFreq] = d_arcLen;
          }
        }
      }
    }
  }
  return u_status;
}