#include "cmn_dopplerDetectCS.h"
#include "mw_alloc.h"
#include "gnss_common.h"
/**
 * @brief initialize the module of using doppler observations to detect cycle slip
 * @param[in/out] pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
 * @return void
 */
void cmn_initDopplerDetectCyleSlip(gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock)
{
  if (NULL != pz_dopplerDetectPairingBlock)
  {
    if (NULL != (pz_dopplerDetectPairingBlock->pz_curBlock))
    {
      OS_FREE((pz_dopplerDetectPairingBlock->pz_curBlock));
      (pz_dopplerDetectPairingBlock->pz_curBlock) = NULL;
    }
    pz_dopplerDetectPairingBlock->pz_curBlock = (gnss_dopplerDetectBlock_t*)OS_MALLOC(sizeof(gnss_dopplerDetectBlock_t));
    if (NULL != (pz_dopplerDetectPairingBlock->pz_curBlock))
    {
      memset(pz_dopplerDetectPairingBlock->pz_curBlock, 0, sizeof(gnss_dopplerDetectBlock_t));
    }

    if (NULL != (pz_dopplerDetectPairingBlock->pz_preBlock))
    {
      OS_FREE((pz_dopplerDetectPairingBlock->pz_preBlock));
      (pz_dopplerDetectPairingBlock->pz_preBlock) = NULL;
    }
    pz_dopplerDetectPairingBlock->pz_preBlock = (gnss_dopplerDetectBlock_t*)OS_MALLOC(sizeof(gnss_dopplerDetectBlock_t));
    if (NULL != (pz_dopplerDetectPairingBlock->pz_preBlock))
    {
      memset(pz_dopplerDetectPairingBlock->pz_preBlock, 0, sizeof(gnss_dopplerDetectBlock_t));
    }
    pz_dopplerDetectPairingBlock->q_isSeparateBDS2And3 = FALSE;
  }
  return;
}
/**
 * @brief de-initialize the module of using doppler observations to detect cycle slip
 * @param[in/out] pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
 * @return void
 */
void cmn_deinitDopplerDetectCyleSlip(gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock)
{
  if (NULL != pz_dopplerDetectPairingBlock)
  {
    OS_FREE((pz_dopplerDetectPairingBlock->pz_curBlock));
    (pz_dopplerDetectPairingBlock->pz_curBlock) = NULL;

    OS_FREE((pz_dopplerDetectPairingBlock->pz_preBlock));
    (pz_dopplerDetectPairingBlock->pz_preBlock) = NULL;
    pz_dopplerDetectPairingBlock->q_isSeparateBDS2And3 = FALSE;
  }
  return;
}
/**
 * @brief find the index observation in gnss_dopplerDetectBlock_t struct
 * @param[in]  pz_detectBlock the observation information used by doppler detect cycle slip for previous or current epoch
 * @param[in] u_curSys, GNSS constellation
 * @param[in] u_curPrn, satellite index of the system
 * @param[in] e_curFreqType, frequency type of satellite
 * @param[out] pu_preIndex, the index of observation in previous epoch
 * @return 1 represent find target observations success, otherwise, 0 represent find target observations failure
 */
uint8_t cmn_findTargetIndexOfDopplerDetectBlock(const gnss_dopplerDetectBlock_t* pz_detectBlock, uint8_t u_curSys, uint8_t u_curPrn,
    gnss_FreqType e_curFreqType, uint8_t* pu_preIndex)
{
  uint8_t u_status = 0;
  uint16_t w_imeas = 0;
  gnss_FreqType e_preFreqType = C_GNSS_FREQ_TYPE_MAX;
  const GnssMeas_t* pz_meas = NULL;
  *pu_preIndex = 0;
  for (w_imeas = 0; w_imeas < pz_detectBlock->w_measNum; ++w_imeas)
  {
      pz_meas = pz_detectBlock->pz_meas + w_imeas;
      if ((pz_meas->u_constellation) != u_curSys || (pz_meas->u_svid) != u_curPrn)
      {
          continue;
      }
      e_preFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)(pz_meas->u_signal));
      if (e_preFreqType == e_curFreqType)
      {
          *pu_preIndex = (uint8_t)w_imeas;
          u_status = 1;
          break;
      }
  }
  return u_status;
}
/**
* @brief gain median of clock for carrier phase minus doppler observations
* @param[in]  e_TargetSysType, target constellation
* @param[in]  z_targetFreq is the target frequency
* @param[in]  pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
* @param[out] pf_corseRcvDrift,the result of corse receiver clock drift
* @return uint8_t,0 represent failure and 1 represent success
*/
uint8_t cmn_gainMedianDopplerMinusCarrier(gnss_ConstellationType e_TargetSysType, gnss_FreqType z_targetFreq, const gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock,
  float* pf_corseRcvDrift)
{
  uint8_t u_status = 0;
  uint8_t u_obsNum = 0;
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  uint16_t w_iMeasCur = 0;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_deltaTime = 0.0;
  double d_wave = 0.0;
  double d_distVar = 0.0;
  double d_distVarCarrier = 0.0;
  float f_avgDoppler = 0.0;
  float pf_diff[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  const GnssMeas_t* pz_curMeas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const gnss_dopplerDetectBlock_t* pz_preBlock = (pz_dopplerDetectPairingBlock->pz_preBlock);
  const gnss_dopplerDetectBlock_t* pz_curBlock = (pz_dopplerDetectPairingBlock->pz_curBlock);

  /* threshold define begin */
  const double d_minCN0 = 27.5; //min of CN0 to gain median of clock
  /* threshold define end */

  d_deltaTime = tm_GpsTimeDiff(&(pz_curBlock->z_obsTime), &(pz_preBlock->z_obsTime));
  for (w_iMeasCur = 0; w_iMeasCur < (pz_curBlock->w_measNum); ++w_iMeasCur)
  {
    pz_curMeas = pz_curBlock->pz_meas + w_iMeasCur;
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dopplerDetectPairingBlock->q_isSeparateBDS2And3, pz_curMeas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == u_constellation || e_TargetSysType!= u_constellation)
    {
      continue;
    }
    if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_curMeas->u_signal);
    if (e_curFreqType != z_targetFreq)
    {
      continue;
    }
    if (!cmn_findTargetIndexOfDopplerDetectBlock(pz_preBlock, pz_curMeas->u_constellation, pz_curMeas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preBlock->pz_meas + u_preIndex;
    if (fabs(pz_curMeas->d_doppler) < 1.0e-3 && fabs(pz_preMeas->d_doppler) < 1.0e-3)
    {
      continue;
    }
    if (fabs(pz_curMeas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    if (pz_curMeas->f_cn0 < d_minCN0 || pz_preMeas->f_cn0 < d_minCN0)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_curMeas->u_signal);
    if (fabs(pz_curMeas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
        f_avgDoppler = (float)(pz_curMeas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_curMeas->d_doppler) > 0.0)
    {
        f_avgDoppler = (float)pz_curMeas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
        f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    d_distVar = f_avgDoppler * d_deltaTime / d_wave;
    d_distVarCarrier = (pz_curMeas->d_carrierPhase - pz_preMeas->d_carrierPhase);
    pf_diff[u_obsNum++] = (float)(d_distVarCarrier - d_distVar);
  }
  u_status = gnss_ascSortMedianFloat(pf_diff, (uint32_t)u_obsNum, pf_corseRcvDrift);
  return u_status;
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
uint8_t cmn_findSatSigMeas(uint8_t u_constellation, uint8_t u_svid, gnss_FreqType e_targetFreqType, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
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
    if ((pz_satMeas->u_constellation) != u_constellation || (pz_satMeas->u_svid) != u_svid)
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
  * @brief mark cycle slip of target frequency
  * @param[in]     e_TargetSysType, target constellation
  * @param[in]     z_targetFreq is the target frequency
  * @param[in/out] pz_satSignalCur is the observation information
  * @param[in]     pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
  * @param[in]     f_corseRcvDrift,the result of corse receiver clock drift
  * @return void
  */
void cmn_markTargetFreqCycleSlipByDopplerDetectModel(gnss_ConstellationType e_TargetSysType, gnss_FreqType z_targetFreq, gnss_SatSigMeasCollect_t* pz_satSignalCur, 
  const gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock, float f_corseRcvDrift)
{
  uint8_t u_preIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_iMeasCur = 0;
  uint32_t q_satIndex = 0;
  BOOL q_isBDS3 = FALSE;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_deltaTime = 0.0;
  double d_wave = 0.0;
  double d_distVar = 0.0;
  double d_distVarCarrier = 0.0;
  float f_avgDoppler = 0.0;
  float f_diff = 0.0;
  const GnssMeas_t* pz_curMeas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const gnss_dopplerDetectBlock_t* pz_preBlock = (pz_dopplerDetectPairingBlock->pz_preBlock);
  const gnss_dopplerDetectBlock_t* pz_curBlock = (pz_dopplerDetectPairingBlock->pz_curBlock);

  /* threshold define begin */
  const double d_minCN0 = 27.5; //min of CN0 to gain median of clock
  /* threshold define end */

  d_deltaTime = tm_GpsTimeDiff(&(pz_curBlock->z_obsTime), &(pz_preBlock->z_obsTime));
  for (w_iMeasCur = 0; w_iMeasCur < (pz_curBlock->w_measNum); ++w_iMeasCur)
  {
    pz_curMeas = pz_curBlock->pz_meas + w_iMeasCur;
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_dopplerDetectPairingBlock->q_isSeparateBDS2And3, pz_curMeas->u_constellation);
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == u_constellation || e_TargetSysType != u_constellation)
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_curMeas->u_signal);
    if (e_curFreqType != z_targetFreq)
    {
      continue;
    }
    if (!cmn_findTargetIndexOfDopplerDetectBlock(pz_preBlock, pz_curMeas->u_constellation, pz_curMeas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preBlock->pz_meas + u_preIndex;
    if (fabs(pz_curMeas->d_doppler) < 1.0e-3 && fabs(pz_preMeas->d_doppler) < 1.0e-3)
    {
      continue;
    }
    if (fabs(pz_curMeas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_curMeas->u_signal);
    if (fabs(pz_curMeas->d_doppler) > 0.0 && fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_curMeas->d_doppler + pz_preMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_curMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_curMeas->d_doppler;
    }
    else if (fabs(pz_preMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_preMeas->d_doppler;
    }
    d_distVar = f_avgDoppler * d_deltaTime / d_wave;
    d_distVarCarrier = (pz_curMeas->d_carrierPhase - pz_preMeas->d_carrierPhase);
    f_diff = (float)(d_distVarCarrier - d_distVar) - f_corseRcvDrift;
    if (fabs(f_diff) < 5.0)
    {
      if (0 == cmn_findSatSigMeas(pz_curMeas->u_constellation, pz_curMeas->u_svid, e_curFreqType, pz_satSignalCur, &q_satIndex, &u_signalIndex))
      {
        continue;
      }
      if ((fabs(f_diff) < 1.0) && (pz_curMeas->f_cn0 > d_minCN0) && (pz_preMeas->f_cn0 > d_minCN0))
      {
        (pz_satSignalCur->pz_satMeas[q_satIndex]->pz_signalMeas[u_signalIndex]->u_slipMask) |= NON_1CYCLE_SLIP_BY_DOPPLER;
      }
      if (fabs(f_diff) < 5.0)
      {
        (pz_satSignalCur->pz_satMeas[q_satIndex]->pz_signalMeas[u_signalIndex]->u_slipMask) |= NON_5CYCLE_SLIP_BY_DOPPLER;
      }
      (pz_satSignalCur->pz_satMeas[q_satIndex]->pz_signalMeas[u_signalIndex]->u_slipMethodValid) |= DOPPLER_DETECT_VALID;
    }
  }
  return;
}
/**
  * @brief using doppler observations of target frequency to detect cycle slip
  * @param[in]     e_TargetSysType, target constellation
  * @param[in]     z_targetFreq is the target frequency
  * @param[in/out] pz_satSignalCur is the observation information
  * @param[in]     pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
  * @return void
  */
void cmn_targetFreqDopplerDetectCS(gnss_ConstellationType e_TargetSysType, gnss_FreqType z_targetFreq, gnss_SatSigMeasCollect_t* pz_satSignalCur, 
  const gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock)
{
    uint16_t w_iMeasCur = 0;
    double d_deltaTime = 0.0;
    float f_corseRcvDrift = 0.0;
    const gnss_dopplerDetectBlock_t* pz_preBlock = (pz_dopplerDetectPairingBlock->pz_preBlock);
    const gnss_dopplerDetectBlock_t* pz_curBlock = (pz_dopplerDetectPairingBlock->pz_curBlock);
    d_deltaTime = tm_GpsTimeDiff(&(pz_curBlock->z_obsTime), &(pz_preBlock->z_obsTime));
    if ((pz_dopplerDetectPairingBlock->pz_curBlock->w_measNum) > 2 && (pz_dopplerDetectPairingBlock->pz_preBlock->w_measNum) > 2 && fabs(d_deltaTime)<=10.0)
    {
      if (cmn_gainMedianDopplerMinusCarrier(e_TargetSysType, z_targetFreq, pz_dopplerDetectPairingBlock, &f_corseRcvDrift))
      {
        cmn_markTargetFreqCycleSlipByDopplerDetectModel(e_TargetSysType, z_targetFreq, pz_satSignalCur, pz_dopplerDetectPairingBlock, f_corseRcvDrift);
      }
    }
	return;
}
/**
 * @brief convert the observation information to TDCP observations
 * @param[in]  pz_satSignalCur is the observation information
 * @param[out] pz_curDopplerDetectBlock is the current epoch observations used by doppler detecting cycle slip
 * @return the number of the current epoch observations used by doppler detecting cycle slip
 */
uint16_t convertSigMeasToDopplerDetectStruct(const gnss_SatSigMeasCollect_t* pz_satSignalCur, gnss_dopplerDetectBlock_t* pz_curDopplerDetectBlock)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_measNum = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  GnssMeas_t* pz_dopplerDetectMeas = NULL;
  pz_curDopplerDetectBlock->z_obsTime = pz_satSignalCur->z_tor;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[q_satIndex];
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
      pz_dopplerDetectMeas = (pz_curDopplerDetectBlock->pz_meas) + w_measNum;
      pz_dopplerDetectMeas->z_measStatusFlag = pz_sigMeas->z_measStatusFlag;
      pz_dopplerDetectMeas->u_constellation = pz_sigMeas->u_constellation;
      pz_dopplerDetectMeas->u_svid = pz_sigMeas->u_svid;
      pz_dopplerDetectMeas->u_signal = pz_sigMeas->u_signal;
      pz_dopplerDetectMeas->u_LLI = pz_sigMeas->u_LLI;
      pz_dopplerDetectMeas->f_cn0 = pz_sigMeas->f_cn0;
      pz_dopplerDetectMeas->d_pseudoRange = pz_sigMeas->d_pseudoRange;
      if (!(pz_sigMeas->z_measStatusFlag.b_drValid))
      {
        pz_dopplerDetectMeas->d_doppler = 0.0;
      }
      else
      {
        pz_dopplerDetectMeas->d_doppler = pz_sigMeas->d_doppler;
      }
      pz_dopplerDetectMeas->d_carrierPhase = pz_sigMeas->d_carrierPhase;
      ++w_measNum;
    }
  }
  pz_curDopplerDetectBlock->w_measNum = w_measNum;
  return w_measNum;
}
/**
  * @brief according the signal type to determine whether BDS2 and BDS3 are separate
  * @param[in]     z_targetFreq is the target frequency
  * @param[in]     pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
  * @return TRUE represent BDS2 and BDS3 will be separated and FALSE represent BDS2 and BDS3 will not be separated
  */
BOOL cmn_isSeparateBDS2And3InDopplerCycleSlip(gnss_FreqType z_targetFreq, const gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock)
{
  BOOL q_isSeparateBDS2And3 = FALSE;
  BOOL q_isBDS3 = FALSE;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SignalType u_BDS2SignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_BDS3SignalType = C_GNSS_SIG_MAX;
  const GnssMeas_t* pz_curMeas = NULL;
  const gnss_dopplerDetectBlock_t* pz_curBlock = (pz_dopplerDetectPairingBlock->pz_curBlock);
  uint16_t w_iMeasCur = 0;
  for (w_iMeasCur = 0; w_iMeasCur < (pz_curBlock->w_measNum); ++w_iMeasCur)
  {
    pz_curMeas = pz_curBlock->pz_meas + w_iMeasCur;
    //only for BDS constellation
    if (C_GNSS_BDS2 != (pz_curMeas->u_constellation) && C_GNSS_BDS3 != (pz_curMeas->u_constellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_curMeas->u_signal);
    if (e_curFreqType != z_targetFreq)
    {
      continue;
    }
    if (C_GNSS_SIG_MAX != u_BDS2SignalType && C_GNSS_SIG_MAX != u_BDS3SignalType)
    {
      break;
    }
    q_isBDS3 = gnss_isBDS3Sat(pz_curMeas->u_svid, pz_curMeas->u_constellation);
    if (TRUE == q_isBDS3)
    {
      u_BDS3SignalType = pz_curMeas->u_signal;
    }
    else
    {
      u_BDS2SignalType = pz_curMeas->u_signal;
    }
  }
  if (u_BDS2SignalType != u_BDS3SignalType)
  {
    q_isSeparateBDS2And3 = TRUE;
  }
  return q_isSeparateBDS2And3;
}
/**
  * @brief using doppler observations to detect cycle slip
  * @param[in/out] pz_satSignalCur is the observation information
  * @param[in/out] pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
  * @return the delta time between current epoch and previous epoch 
  */
double cmn_dopplerDetectCycleSlip(gnss_SatSigMeasCollect_t* pz_satSignalCur, gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock)
{
  double d_deltaTime = 999.9;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_iFreq = 0;
  if (NULL != pz_dopplerDetectPairingBlock && NULL != (pz_dopplerDetectPairingBlock->pz_curBlock) && NULL!=pz_satSignalCur)
  {
    convertSigMeasToDopplerDetectStruct(pz_satSignalCur, pz_dopplerDetectPairingBlock->pz_curBlock);
  }
  
  if (NULL != pz_satSignalCur && NULL != pz_dopplerDetectPairingBlock 
      && NULL != (pz_dopplerDetectPairingBlock->pz_curBlock) && NULL != (pz_dopplerDetectPairingBlock->pz_preBlock))
  {
    d_deltaTime = tm_GpsTimeDiff(&(pz_dopplerDetectPairingBlock->pz_curBlock->z_obsTime), &(pz_dopplerDetectPairingBlock->pz_preBlock->z_obsTime));
    for (u_iFreq = 0; u_iFreq < C_GNSS_FREQ_TYPE_MAX; ++u_iFreq)
    {
      pz_dopplerDetectPairingBlock->q_isSeparateBDS2And3 = cmn_isSeparateBDS2And3InDopplerCycleSlip(u_iFreq, pz_dopplerDetectPairingBlock);
      for (u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_MAX; ++u_constellation)
      {
        if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_dopplerDetectPairingBlock->q_isSeparateBDS2And3, u_constellation))
        {
          continue;
        }
        cmn_targetFreqDopplerDetectCS((gnss_ConstellationType)u_constellation, (gnss_FreqType)u_iFreq, pz_satSignalCur, pz_dopplerDetectPairingBlock);
      }
    }
    memcpy(pz_dopplerDetectPairingBlock->pz_preBlock, pz_dopplerDetectPairingBlock->pz_curBlock, sizeof(gnss_dopplerDetectBlock_t));
  }
  return d_deltaTime;
}