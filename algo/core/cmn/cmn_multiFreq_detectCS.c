#include "cmn_multiFreq_detectCS.h"
#include "mw_alloc.h"
#include "gnss_common.h"
#include "mw_log.h"
#include <math.h>
#include "cmn_utils.h"

/**
 * @brief  initilizing the inteface which inculde the information of using multi-frequency  to detect cycle slip 
 * @param[in] pz_MLFreqDetectSet is the  multi-frequencyobservation information used to  detect cycle slip
 * @return
 */
void cmn_initMultiFreqDetectCyleSlip(cmn_multiFreqDetectSet* pz_MLFreqDetectSet)
{ 
	if (NULL != pz_MLFreqDetectSet) 
  {
    for (uint16_t w_i = 0 ; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
    {
      pz_MLFreqDetectSet->pz_amultiFreqSet[w_i] = NULL;
    }     
	}
}
/**
 * @brief de-initilizing the inteface which inculde the information of using multi-frequency  to detect cycle slip
 * @param[in] pz_MLFreqDetectSet is the  multi-frequencyobservation information used to  detect cycle slip
 * @return none
 */
void cmn_deinitmultiFreqDetectCyleSlip(cmn_multiFreqDetectSet* pz_MLFreqDetectSet) 
{
  if (NULL != pz_MLFreqDetectSet) 
  {
    for (uint16_t w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i) 
    {
      if (NULL != pz_MLFreqDetectSet->pz_amultiFreqSet[w_i]) 
      {
        OS_FREE(pz_MLFreqDetectSet->pz_amultiFreqSet[w_i]);
        pz_MLFreqDetectSet->pz_amultiFreqSet[w_i] = NULL;
      }
    }
  }
}
/**
 * @brief initilize the Multi-Frequency Detect Information
 * @param[in]  pz_amultiFreqInfo is the  information used by multi frequency detect cycle slip
 * @return none
 */
void cmn_initMultiFreqDetectInfo(cmn_oneObsmultiFreqDetectInfo* pz_amultiFreqInfo) 
{
  if (NULL != pz_amultiFreqInfo)
  {
    pz_amultiFreqInfo->d_GIFValue = 0.0;
    for (int i = 0; i < 2; ++i)
    {
      pz_amultiFreqInfo->d_GF_L1L2Value[i] = 0.0;
      pz_amultiFreqInfo->d_GF_L2L5Value[i] = 0.0;
    }
    pz_amultiFreqInfo->u_countEpoch = 0;
    tm_initGpsTime(&(pz_amultiFreqInfo->z_obsTimeBeg));
    tm_initGpsTime(&(pz_amultiFreqInfo->z_obsTimePre));
  }
}
/**
 * @brief update the Multi-Frequency Detect Information
 * @param[in]   pz_amultiFreqInfo is the  information used by multi frequency detect cycle slip
 * @param[in]  d_GIFvalue is the value of GIF combination observations
 * @param[in]  d_GFL1L2value is the value of GF_L2_L5 combination observations 
 * @param[in]  d_GFL2L5value is the value of GF_L1_L2 combination observations
 * @return void
 */
void cmn_updateMultiFreqDetecetInfo(double d_GIFvalue, double d_GFL1L2value, double d_GFL2L5value, cmn_oneObsmultiFreqDetectInfo* pz_amultiFreqInfo) 
{
  pz_amultiFreqInfo->d_GIFValue = d_GIFvalue;
  pz_amultiFreqInfo->d_GF_L1L2Value[1] = pz_amultiFreqInfo->d_GF_L1L2Value[0];
  pz_amultiFreqInfo->d_GF_L2L5Value[1] = pz_amultiFreqInfo->d_GF_L2L5Value[0];
  pz_amultiFreqInfo->d_GF_L1L2Value[0] = d_GFL1L2value;
  pz_amultiFreqInfo->d_GF_L2L5Value[0] = d_GFL2L5value;
}
/**
 * @brief calculation multi-frequency carrier phase combination observations and thresold for the current epoch
 * @param[in]  pz_satMeas is the observation information
 * @param[in]  pu_aFreqIndex is the index of L1,L2,L5
 * @param[in]  pz_combineResult is the result used by the current ephemeris in the multi-frequency slip detection
 * @param[in]  pd_wave is the wavelength of L1,L2,L5
 * @param[in]  pd_freq is the frequency of L1,L2,L5
 * @return none
 */
void cmn_calMultiFreqcombination(gnss_SatelliteMeas_t* pz_satMeas,uint8_t* pu_aFreqIndex, CalMultiFreqCombineResultInfo_t* pz_combineResult,double* pd_wave, double* pd_freq)
{
  double d_freq1_2 = pd_freq[0] * pd_freq[0], d_freq2_2 = pd_freq[1] * pd_freq[1];
  double d_freq5_2 = pd_freq[2] * pd_freq[2], d_sigama_phi = 0.01 * sqrt(0.5 / sin(pz_satMeas->z_satPosVelClk.f_elevation));
  pz_combineResult->d_beta = (-d_freq2_2 / (d_freq1_2 - d_freq2_2)) * pd_wave[1];
  pz_combineResult->d_alpha = (d_freq2_2 / (d_freq1_2 - d_freq2_2) - d_freq5_2 / (d_freq1_2 - d_freq5_2)) * pd_wave[0];
  pz_combineResult->d_gamma = (d_freq5_2 / (d_freq1_2 - d_freq5_2)) * pd_wave[2];
  pz_combineResult->d_GIFvalue = pz_combineResult->d_alpha * pz_satMeas->pz_signalMeas[pu_aFreqIndex[0]]->d_carrierPhase + pz_combineResult->d_beta * pz_satMeas->pz_signalMeas[pu_aFreqIndex[1]]->d_carrierPhase + pz_combineResult->d_gamma * pz_satMeas->pz_signalMeas[pu_aFreqIndex[2]]->d_carrierPhase;
  pz_combineResult->d_GFL1L2value = pd_wave[0] * pz_satMeas->pz_signalMeas[pu_aFreqIndex[0]]->d_carrierPhase - pd_wave[1] * pz_satMeas->pz_signalMeas[pu_aFreqIndex[1]]->d_carrierPhase;
  pz_combineResult->d_GFL2L5value = pd_wave[1] * pz_satMeas->pz_signalMeas[pu_aFreqIndex[1]]->d_carrierPhase - pd_wave[2] * pz_satMeas->pz_signalMeas[pu_aFreqIndex[2]]->d_carrierPhase;
  pz_combineResult->d_GIFsigma = sqrt(2 * (pz_combineResult->d_alpha * pz_combineResult->d_alpha + pz_combineResult->d_beta * pz_combineResult->d_beta + pz_combineResult->d_gamma * pz_combineResult->d_gamma)) * d_sigama_phi;
  pz_combineResult->d_GFL2L5sigma = sqrt(2 * ((pd_wave[2] * pd_wave[2] + pd_wave[1] * pd_wave[1]))) * d_sigama_phi * 5;
  pz_combineResult->d_GFL1L2sigma = sqrt(2 * ((pd_wave[0] * pd_wave[0] + pd_wave[1] * pd_wave[1]))) * d_sigama_phi * 5;
}
/**
 * @brief Determine and identify whether the frequency has a cycle slip
 * @param[in]  pz_tor is observation time
 * @param[in]  pz_amultiFreqInfo is the  multi - frequency observation information used to  detect cycle slip
 * @param[in]  pz_combineResult is the result used by the current ephemeris in the multi-frequency slip detection
 * @return     1 represent success and 0 failure
 */
//int32_t* cmn_DetermineFreqCycle(CalMultiFreqCombineResultInfo_t* pz_combineResult,double* d_wave,double d_deltaGIF, double d_deltaL1L2, double d_deltaL2L5, int32_t* pu_aCycle[3])
//{
//  //uint8_t pu_aCycle[3] = { 0 };
//  matrix_t* m_A = matrix_new(3, 3);
//  matrix_t* m_L = matrix_new(3, 1);
//  matrix_t* m_X = matrix_new(3, 1);
//  matrix_t* m_invA = matrix_new(3, 3);
//  double pd_aA[9] = { ((int)(1000*pz_combineResult->d_alpha/d_wave[0]+0.5))/1000.0,((int)(10000 * pz_combineResult->d_beta / d_wave[1] + 0.5)) / 10000.0,((int)(10000 * pz_combineResult->d_gamma / d_wave[2] + 0.5)) / 10000.0,0,1,-1,1,-1,0 };
//  double pd_aL[3] = {d_deltaGIF,d_deltaL2L5,d_deltaL1L2};
//  
//  for (int i = 0; i < m_A->row; ++i) 
//  {
//    for (int j = 0; j < m_A->col; ++j) 
//    {
//      MAT(m_A, i, j) = pd_aA[i * m_A->col + j];
//    }
//    MAT(m_L, i, 0) = pd_aL[i];
//  }
//  if (matrix_inverse(m_A, m_invA)) 
//  {
//    matrix_mul(m_invA, m_L, m_X);
//    for (int i = 0; i < m_X->row; ++i) 
//    {
//      pu_aCycle[i] = (int)(m_X->data[i]/d_wave[i]+0.5);
//    }
//  }
//  else
//  {
//    for (int i = 0; i < 3; ++i) 
//    {
//      pu_aCycle[i] = 1;
//    }
//  }
//  return pu_aCycle;
//}
/**
 * @brief the inteface of using Multi-frequency combination method to detect cycle slip
 * @param[in]  pz_tor is observation time
 * @param[in]  pz_combineResult is the result used by the current ephemeris in the multi-frequency slip detection
 * @param[in]  pz_amultiFreqInfo is the  multi - frequency observation information used to  detect cycle slip 
 * @param[in]  d_wave is the wavelength of three frequencies
 * @return     0 represent non-detect, 1 represent non cycle slip, 2 represent cycle slip has happened
 */
uint8_t cmn_MuLtiFreqDetectCycleSlip(const GpsTime_t* pz_tor, CalMultiFreqCombineResultInfo_t* pz_combineResult, cmn_oneObsmultiFreqDetectInfo* pz_amultiFreqInfo, double* d_wave, int32_t* pu_aCycle)
{
  uint8_t u_cycleSlipStatus = 0;
  uint8_t u_multithresold = 4;
  double d_deltaGIF = 0.0;
  double d_deltaGFL1L2 = 0.0;
  double d_deltaGFL2L5 = 0.0;
  double d_deltaSTRIPL2L5 = 0.0;
  double d_deltaSTRIPL1L2 = 0.0;
  double d_intv = fabs(tm_GpsTimeDiff(pz_tor, &(pz_amultiFreqInfo->z_obsTimePre)));
  double d_intv1 = fabs(tm_GpsTimeDiff(&(pz_amultiFreqInfo->z_obsTimeBeg), &(pz_amultiFreqInfo->z_obsTimePre)));
  double d_thresoldGIF = d_intv * u_multithresold * (pz_combineResult->d_GIFsigma);
  double d_thresoldGFL1L2 = u_multithresold * (pz_combineResult->d_GFL1L2sigma);
  double d_thresoldGFL2L5 = u_multithresold * (pz_combineResult->d_GFL2L5sigma);
  if (0 == (pz_amultiFreqInfo->u_countEpoch) || d_intv > 30.0 || d_intv <= 0.0 || d_intv1 > 30.0 || d_intv1 <= 0.0)
  {
    /*for (int i = 0; i < C_GNSS_FREQ_TYPE_MAX; ++i)
    {
      pu_aCycle[i] = 1;
    }*/
    cmn_updateMultiFreqDetecetInfo(pz_combineResult->d_GIFvalue, pz_combineResult->d_GFL1L2value, pz_combineResult->d_GFL2L5value, pz_amultiFreqInfo);
    pz_amultiFreqInfo->z_obsTimeBeg = pz_amultiFreqInfo->z_obsTimePre;
    pz_amultiFreqInfo->z_obsTimePre = *pz_tor;
    pz_amultiFreqInfo->u_countEpoch = 1;
    return u_cycleSlipStatus;
  }

  if (1 == (pz_amultiFreqInfo->u_countEpoch))
  {
    d_deltaGIF = fabs(pz_combineResult->d_GIFvalue - pz_amultiFreqInfo->d_GIFValue);
    d_deltaGFL1L2 = fabs(pz_combineResult->d_GFL1L2value - pz_amultiFreqInfo->d_GF_L1L2Value[0]);
    d_deltaGFL2L5 = fabs(pz_combineResult->d_GFL2L5value - pz_amultiFreqInfo->d_GF_L2L5Value[0]);
    d_thresoldGFL1L2 = sqrt(SQR(d_intv) + SQR(d_intv1)) * d_thresoldGFL1L2;
    d_thresoldGFL2L5 = sqrt(SQR(d_intv) + SQR(d_intv1)) * d_thresoldGFL2L5;
    d_deltaSTRIPL1L2 = fabs(d_deltaGFL1L2 - fabs(pz_amultiFreqInfo->d_GF_L1L2Value[0] - pz_amultiFreqInfo->d_GF_L1L2Value[1]));
    d_deltaSTRIPL2L5 = fabs(d_deltaGFL2L5 - fabs(pz_amultiFreqInfo->d_GF_L2L5Value[0] - pz_amultiFreqInfo->d_GF_L2L5Value[1]));
    if (d_deltaGIF <= d_thresoldGIF && d_deltaSTRIPL1L2 <= d_thresoldGFL1L2 && d_deltaSTRIPL2L5 <= d_thresoldGFL2L5)
    {
      cmn_updateMultiFreqDetecetInfo(pz_combineResult->d_GIFvalue, pz_combineResult->d_GFL1L2value, pz_combineResult->d_GFL2L5value, pz_amultiFreqInfo);
      u_cycleSlipStatus = 1;
      pz_amultiFreqInfo->z_obsTimeBeg = pz_amultiFreqInfo->z_obsTimePre;
      pz_amultiFreqInfo->z_obsTimePre = *pz_tor;
      return u_cycleSlipStatus;
    }
    else
    {
      u_cycleSlipStatus = 2;
      cmn_initMultiFreqDetectInfo(pz_amultiFreqInfo);
      return u_cycleSlipStatus;
    }
  }

  return u_cycleSlipStatus;
}
/**
 * @brief the inteface of using multi-frequency method to detect cycle slip
 * @param[in]  pz_satSignalCur is the observation information
 * @param[in] pz_MLFreqDetectSet is the  multi-frequency observation information used to  detect cycle slip
 * @return 
 */
uint8_t cmn_detectCycleSlipUsingMultiFreqObs(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multiFreqDetectSet* pz_MLFreqDetectSet)
{
  uint16_t w_iSat = 0;
  char pu_sat[4] = "";
  uint8_t u_continueflag = 0;
  uint8_t u_status = 0;
  uint8_t u_iFreq = 0;
  uint8_t u_isCycleSlipHappen = 0;
  uint8_t pu_aFreqIndex[C_GNSS_FREQ_TYPE_MAX];
  int32_t pu_aCycle[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  double pd_wave[C_GNSS_FREQ_TYPE_MAX] = { 0.0 };
  double pd_freq[C_GNSS_FREQ_TYPE_MAX] = { 0.0 };
  gnss_SignalType u_FreqSignalType = C_GNSS_SIG_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  CalMultiFreqCombineResultInfo_t* pz_combineResult = NULL;
  pz_combineResult = (CalMultiFreqCombineResultInfo_t*)OS_MALLOC(sizeof(CalMultiFreqCombineResultInfo_t));
  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat) 
  {
    if (NULL == pz_satSignalCur->pz_satMeas[w_iSat])
    {
      continue;
    }
    pz_satMeas = pz_satSignalCur->pz_satMeas[w_iSat];
    u_continueflag = 0;
    //memset(pu_aCycle, 0, sizeof(pu_aCycle));
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; ++u_iFreq)
    {
      if (NULL == pz_satMeas->pz_signalMeas[u_iFreq])
      {
        u_continueflag = 1;
        break;
      }
      if (fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase) < 1.0e-3)
      {
        u_continueflag = 1;
        break;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_satMeas->pz_signalMeas[u_iFreq]->u_signal);
      if (C_GNSS_FREQ_TYPE_L1 == z_freqType)
      {
        pu_aFreqIndex[0] = u_iFreq;
        u_FreqSignalType = pz_satMeas->pz_signalMeas[u_iFreq]->u_signal;
        pd_wave[0] = wavelength(u_FreqSignalType);
        pd_freq[0] = CLIGHT / pd_wave[0];
        if (pd_wave[0] >= 1.0 || fabs(pd_freq[0] - CLIGHT) < 1.0e-6)
        {
          u_continueflag = 1;
          break;
        }
      }
      else if (C_GNSS_FREQ_TYPE_L2 == z_freqType)
      {
        pu_aFreqIndex[1] = u_iFreq;
        u_FreqSignalType = pz_satMeas->pz_signalMeas[u_iFreq]->u_signal;
        pd_wave[1] = wavelength(u_FreqSignalType);
        pd_freq[1] = CLIGHT / pd_wave[1];
        if (pd_wave[1] >= 1.0 || fabs(pd_freq[1] - CLIGHT) < 1.0e-6)
        {
          u_continueflag = 1;
          break;
        }
      }
      else if (C_GNSS_FREQ_TYPE_L5 == z_freqType) 
      {
        pu_aFreqIndex[2] = u_iFreq;
        u_FreqSignalType = pz_satMeas->pz_signalMeas[u_iFreq]->u_signal;
        pd_wave[2] = wavelength(u_FreqSignalType);
        pd_freq[2] = CLIGHT / pd_wave[2];
        if (pd_wave[2] >= 1.0 || fabs(pd_freq[2] - CLIGHT) < 1.0e-6)
        {
          u_continueflag = 1;
          break;
        }
      }
    }
    if (u_continueflag == 1)
    {
      continue;
    }
    if (pz_satMeas->pz_signalMeas[0]->d_carrierPhase == 0 || pz_satMeas->pz_signalMeas[1]->d_carrierPhase == 0 || pz_satMeas->pz_signalMeas[2]->d_carrierPhase == 0)
    {
      continue;
    }
    cmn_calMultiFreqcombination(pz_satMeas, pu_aFreqIndex, pz_combineResult, pd_wave, pd_freq);
    if (NULL == pz_MLFreqDetectSet->pz_amultiFreqSet[w_iSat])
    {
      pz_MLFreqDetectSet->pz_amultiFreqSet[w_iSat] = (cmn_oneObsmultiFreqDetectInfo*)OS_MALLOC(sizeof(cmn_oneObsmultiFreqDetectInfo));
      cmn_initMultiFreqDetectInfo(pz_MLFreqDetectSet->pz_amultiFreqSet[w_iSat]);
    }
    if (NULL == pz_MLFreqDetectSet->pz_amultiFreqSet[w_iSat])
    {
      continue;
    }
    u_isCycleSlipHappen = cmn_MuLtiFreqDetectCycleSlip(&(pz_satSignalCur->z_tor), pz_combineResult, pz_MLFreqDetectSet->pz_amultiFreqSet[w_iSat], pd_wave, pu_aCycle);
    if (1 == u_isCycleSlipHappen)
    {
      for (int i = 0; i < C_GNSS_FREQ_TYPE_MAX; ++i)
      {
        pz_satMeas->pz_signalMeas[i]->u_slipMask |= NON_CYCLE_SLIP_BY_MULTIFREQUENCY;
      }
    }
    else if (2 == u_isCycleSlipHappen)
    {
      satidx_SatString((uint8_t)w_iSat, pu_sat);
      LOGI(TAG_PPP, "Cycle Slip Multi-Freq sat=%s \n", pu_sat);
      /*for (int i = 0; i < C_GNSS_FREQ_TYPE_MAX; ++i) 
      {
        if (pu_aCycle[i] != 0) 
        {
          satidx_SatString((uint8_t)w_iSat, char_sat);
          LOGI(TAG_PPP, "Cycle Slip Multi-Freq sat=%s Freq=%d\n", char_sat, i);
        }
        else 
        {
          pz_satMeas->pz_signalMeas[i]->u_slipMask |= NON_CYCLE_SLIP_BY_MULTIFREQUENCY;
        }
      }*/
    }
    if (u_isCycleSlipHappen != 0)
    {
      for (int i = 0; i < C_GNSS_FREQ_TYPE_MAX; ++i)
      {
        pz_satMeas->pz_signalMeas[i]->u_slipMethodValid |= MULTIFREQUENCY_DETECT_VALID;
      }
    }
  }
  if (NULL != pz_combineResult)
  {
    OS_FREE(pz_combineResult);
  }
  return u_status;
}
   
