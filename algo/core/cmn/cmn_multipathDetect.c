/**********************************************************************************
* @note
* @par History :
*<table>
* <tr><th>Date        <th>Version  <th>Author       <th>Description
* <tr><td>2023/06/01  <td> 0.1     <td>ZhangXuecheng<td>Init version
* < / table>
*
**********************************************************************************
*/
#include <cmn_multipathDetect.h>
#include <mw_alloc.h>
#include <gnss_common.h>
#include <mw_log.h>

float gf_sample = 1.0;
#define NumOfSegments 5 //Segments in 15/30/45/60/90 degree
#define MP_BUFF_SIZE 512 //buffer size of MP detector

#ifdef ENABLE_MULTI_FREQ

/**
 * @brief      get ele-snr bound index
 * @param[in]   u_constellation is index of constellation
 * @param[in]   u_targetFreq is index of freq
 * @param[in]   u_index is index of segments
 * @return     void
 */
uint16_t cmn_getEleSnrBoundIndex(gnss_ConstellationType u_constellation, uint8_t u_freq, uint8_t u_index)
{
  uint16_t w_rlt = 0;
  uint16_t u_sys = u_constellation;
  if (u_freq < C_GNSS_FREQ_TYPE_MAX && u_index < NumOfSegments)
  {
    if (C_GNSS_QZS == u_constellation)
    {
      u_sys = C_GNSS_GPS;
    }
    /*if (C_GNSS_BDS3 == u_constellation)
    {
      u_sys = C_GNSS_BDS2;
    }*/
    if (C_GNSS_GPS == u_sys || C_GNSS_BDS2 == u_sys || C_GNSS_BDS3 == u_sys || C_GNSS_GAL == u_sys)
    {
      w_rlt = u_sys * C_GNSS_FREQ_TYPE_MAX * NumOfSegments + u_freq * NumOfSegments + u_index;
    }
  }
  return w_rlt;
}
/**
 * @brief the inteface of initilizing the MP detector information
 * @param[out]  pz_MPdetectorSet is the obs multipath detector set
 * @param[in]  f_sample background process sample
 * @return     void
 */
void cmn_initMPdetectorSet(cmn_multipathDetectSet* pz_MPdetectorSet, float f_sample)
{
  uint16_t w_i = 0;
  if (NULL != pz_MPdetectorSet)
  {
    pz_MPdetectorSet->f_sample = f_sample;
    gf_sample = f_sample;
    for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
    {
      if (NULL != pz_MPdetectorSet->pz_multiPathSet[w_i])
      {
        OS_FREE(pz_MPdetectorSet->pz_multiPathSet[w_i]);
        pz_MPdetectorSet->pz_multiPathSet[w_i] = NULL;
      }
    }

    //set snr-ele mask
    pz_MPdetectorSet->pd_ele_snr_bound_upper =
      (double*)OS_MALLOC(sizeof(double) * C_GNSS_MAX * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
    pz_MPdetectorSet->pd_ele_snr_bound_lower =
      (double*)OS_MALLOC(sizeof(double) * C_GNSS_MAX * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
    // GPS & QZS
    {
      double pd_GPS_bound_upper[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 42, 45, 47, 48, 48, 39, 41, 45, 47, 48, 44, 46, 48, 50, 52 };
      double pd_GPS_bound_lower[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 32, 36, 40, 41, 42, 30, 32, 35, 39, 41, 35, 39, 42, 43, 45 };
      w_i = cmn_getEleSnrBoundIndex(C_GNSS_GPS, C_GNSS_FREQ_TYPE_L1, 0);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_upper[w_i]), pd_GPS_bound_upper,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_lower[w_i]), pd_GPS_bound_lower,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
    }
    // BDS2
    {
      double pd_BDS_bound_upper[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 41, 43, 44, 45, 45, 39, 40, 42, 43, 45, 43, 45, 46, 47, 48 };
      double pd_BDS_bound_lower[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 30, 33, 35, 36, 36, 30, 32, 34, 36, 38, 30, 36, 39, 40, 41 };
      w_i = cmn_getEleSnrBoundIndex(C_GNSS_BDS2, C_GNSS_FREQ_TYPE_L1, 0);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_upper[w_i]), pd_BDS_bound_upper,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_lower[w_i]), pd_BDS_bound_lower,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
    }
    // BDS3
    {
      double pd_BDS_bound_upper[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 43, 45, 46, 48, 46, 42, 43, 46, 47, 49, 43, 46, 48, 49, 51 };
      double pd_BDS_bound_lower[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 32, 35, 36, 39, 39, 32, 34, 36, 38, 41, 32, 39, 42, 43, 44 };
      w_i = cmn_getEleSnrBoundIndex(C_GNSS_BDS3, C_GNSS_FREQ_TYPE_L1, 0);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_upper[w_i]), pd_BDS_bound_upper,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_lower[w_i]), pd_BDS_bound_lower,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
    }
    // GAL
    {
      double pd_GAL_bound_upper[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 41, 42, 44, 45, 47, 43, 45, 47, 49, 50, 43, 45, 47, 49, 50 };
      double pd_GAL_bound_lower[C_GNSS_FREQ_TYPE_MAX * NumOfSegments] =
      { 31, 33, 35, 36, 37, 32, 34, 38, 39, 41, 32, 34, 38, 39, 41 };
      w_i = cmn_getEleSnrBoundIndex(C_GNSS_GAL, C_GNSS_FREQ_TYPE_L1, 0);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_upper[w_i]), pd_GAL_bound_upper,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
      memcpy(&(pz_MPdetectorSet->pd_ele_snr_bound_lower[w_i]), pd_GAL_bound_lower,
        sizeof(double) * C_GNSS_FREQ_TYPE_MAX * NumOfSegments);
    }
  }
  return;
}
/**
 * @brief the inteface of deinitilizing the MP detector information
 * @param[out]  pz_MPdetectorSet is the obs multipath detector set
 * @return     void
 */
void cmn_deinitMPdetectorSet(cmn_multipathDetectSet* pz_MPdetectorSet)
{
  uint16_t w_i = 0;
  if (NULL != pz_MPdetectorSet)
  {
    for (w_i = 0; w_i < ALL_GNSS_SYS_SV_NUMBER; ++w_i)
    {
      if (NULL != pz_MPdetectorSet->pz_multiPathSet[w_i])
      {
        OS_FREE(pz_MPdetectorSet->pz_multiPathSet[w_i]);
      }
    }
    if (NULL != pz_MPdetectorSet->pd_ele_snr_bound_upper)
    {
      OS_FREE(pz_MPdetectorSet->pd_ele_snr_bound_upper);
    }
    if (NULL != pz_MPdetectorSet->pd_ele_snr_bound_lower)
    {
      OS_FREE(pz_MPdetectorSet->pd_ele_snr_bound_lower);
    }
  }
  return;
}

/**
 * @brief initilize the multipath detector
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_initMPdetectorInfo(cmn_oneSatMultipathDetectInfo* pz_multiPathDetector)
{
  uint8_t u_i = 0;
  if (NULL != pz_multiPathDetector)
  {
    pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_UNKNOWEN;
    pz_multiPathDetector->u_GIFslipCount = 0x0;
    pz_multiPathDetector->u_SNRslipCount = 0x0;
    pz_multiPathDetector->w_GIFepochNum = 0;
    pz_multiPathDetector->w_SNRepochNum = 0;
    pz_multiPathDetector->f_eleA = 0.0;
    pz_multiPathDetector->d_GIFvalue = 0.0;
    pz_multiPathDetector->d_GIFvalueSTD = 0.0;
    pz_multiPathDetector->d_SNRcombineValue = 0.0;
    pz_multiPathDetector->d_SNRcombineValueSTD = 0.0;
    for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; u_i++)
    {
      pz_multiPathDetector->pw_CMCepochNum[u_i] = 0;
      pz_multiPathDetector->pu_SNRmultipathLevel[u_i] = OBS_MP_LEVEL_UNKNOWEN;
      pz_multiPathDetector->pu_CMCmultipathLevel[u_i] = OBS_MP_LEVEL_UNKNOWEN;
      pz_multiPathDetector->pu_CMCslipCount[u_i] = 0x0;
      pz_multiPathDetector->pd_CMCvalue[u_i] = 0.0;
      pz_multiPathDetector->pd_CMCvalueSTD[u_i] = 0.0;
      tm_initGpsTime(&(pz_multiPathDetector->pz_CMCendTime[u_i]));
    }
    tm_initGpsTime(&(pz_multiPathDetector->z_SNRendTime));
    tm_initGpsTime(&(pz_multiPathDetector->z_GIFendTime));
  }
  return;
}
/**
 * @brief update flag of the multipath detector between epoch
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_epochUpdateMPdetectorInfo(cmn_multipathDetectSet* pz_MPdetectorSet)
{
  uint8_t u_i = 0;
  uint8_t w_iSat = 0;
  cmn_oneSatMultipathDetectInfo* pz_multiPathDetector;
  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat)
  {
    pz_multiPathDetector = pz_MPdetectorSet->pz_multiPathSet[w_iSat];
    if (NULL != pz_multiPathDetector)
    {
      pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_UNKNOWEN;
      for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; u_i++)
      {
        pz_multiPathDetector->pu_SNRmultipathLevel[u_i] = OBS_MP_LEVEL_UNKNOWEN;
        pz_multiPathDetector->pu_CMCmultipathLevel[u_i] = OBS_MP_LEVEL_UNKNOWEN;
      }
    }
  }
  return;
}
/**
 * @brief initilize the multipath detector
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_cleanMultiPathDetector_CMC(cmn_oneSatMultipathDetectInfo* pz_multiPathDetector)
{

  uint8_t u_i = 0;
  if (NULL != pz_multiPathDetector)
  {
    for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; u_i++)
    {
      pz_multiPathDetector->pw_CMCepochNum[u_i] = 0;
      pz_multiPathDetector->pu_CMCslipCount[u_i] = 0x0;
      pz_multiPathDetector->pu_CMCmultipathLevel[u_i] = 0;
      pz_multiPathDetector->pd_CMCvalue[u_i] = 0.0;
      pz_multiPathDetector->pd_CMCvalueSTD[u_i] = 0.0;
      tm_initGpsTime(&(pz_multiPathDetector->pz_CMCendTime[u_i]));
    }
  }
  return;
}
/**
 * @brief initilize the GIF multipath detector
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_cleanMultiPathDetector_GIF(cmn_oneSatMultipathDetectInfo* pz_multiPathDetector)
{

  uint8_t u_i = 0;
  if (NULL != pz_multiPathDetector)
  {
    pz_multiPathDetector->w_GIFepochNum = 0;
    pz_multiPathDetector->u_GIFslipCount = 0x0;
    pz_multiPathDetector->u_GIFmultipathLevel = 0;
    pz_multiPathDetector->d_GIFvalue = 0.0;
    pz_multiPathDetector->d_GIFvalueSTD = 0.0;
    tm_initGpsTime(&(pz_multiPathDetector->z_GIFendTime));
  }
  return;
}
/**
 * @brief       use CMC combine detect the multipath of one PR
 * @param[in]   pz_tor is observation time
 * @param[in]   u_firstFreq is index of first freq
 * @param[in]   pd_CMCvalue is CMC combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_CMCdetectMP_onePR(const GpsTime_t* pz_tor, uint8_t u_firstFreq, double pd_CMCvalue, float f_ele,
  cmn_oneSatMultipathDetectInfo* pz_multiPathDetector)
{
  uint8_t u_i = 0;
  uint8_t u_sumSlipEpoch = 0;
  uint8_t u_MPlevel = OBS_MP_LEVEL_UNKNOWEN;
  uint16_t w_epochNum = 0;
  double d_CMCvalue_pre = 0.0;
  double d_CMCvalueSTD_pre = 0.0;
  double d_ave_est = 0;
  double d_std_est = 0;
  double d_intv = 0.0;

  /* threshold define begin */
  const uint8_t u_maxSlipCount = 3;   //max slip epoch num in u_maxEpochCount epochs
  const uint8_t u_maxEpochCount = 4;   //stastic epoch num of slip
  const double d_minEpochNum = 60.0 / gf_sample;   //min epoch num to detect MP
  const double d_maxEpochArc = 300.0 / gf_sample;   //min epoch num to detect MP
  const double d_maxDeltaTime = 60.0; //max time diff
  const double d_maxDeltaAve = 5.0; //max ave diff between epoch
  const double d_maxSlip = 2.0; //max diff between pd_CMCvalue & ave
  const double d_lowThres_STD = 0.8; //threshold of std in low MP level
  const double d_highThres_STD = 1.5; //threshold of std in high MP level
  const double d_lowThres_AVE = 1.0; //threshold of ave in low MP level
  const double d_highThres_AVE = 2.0; //threshold of ave in high MP level
  /* threshold define end */

  d_intv = fabs(tm_GpsTimeDiff(pz_tor, &(pz_multiPathDetector->pz_CMCendTime[u_firstFreq])));
  pz_multiPathDetector->f_eleA = f_ele;
  if (0 == (pz_multiPathDetector->pw_CMCepochNum[u_firstFreq]) || d_intv > d_maxDeltaTime || d_intv <= 0.0)
  {
    pz_multiPathDetector->pw_CMCepochNum[u_firstFreq] = 1;
    pz_multiPathDetector->pu_CMCmultipathLevel[u_firstFreq] = OBS_MP_LEVEL_UNKNOWEN;
    pz_multiPathDetector->pd_CMCvalue[u_firstFreq] = pd_CMCvalue;
    pz_multiPathDetector->pd_CMCvalueSTD[u_firstFreq] = 0.0;
    pz_multiPathDetector->pz_CMCendTime[u_firstFreq] = *pz_tor;
    return;
  }
  w_epochNum = pz_multiPathDetector->pw_CMCepochNum[u_firstFreq];
  d_CMCvalue_pre = pz_multiPathDetector->pd_CMCvalue[u_firstFreq];
  d_CMCvalueSTD_pre = pz_multiPathDetector->pd_CMCvalueSTD[u_firstFreq];
  d_ave_est = (d_CMCvalue_pre * w_epochNum + pd_CMCvalue) / (w_epochNum + 1.0);
  d_std_est = d_CMCvalueSTD_pre * (w_epochNum - 1.0) / (double)w_epochNum
    + SQR(d_ave_est - d_CMCvalue_pre) + SQR(d_ave_est - pd_CMCvalue) / (double)w_epochNum;

  if (w_epochNum < d_maxEpochArc)
  {
    pz_multiPathDetector->pw_CMCepochNum[u_firstFreq] += 1;
  }
  pz_multiPathDetector->pz_CMCendTime[u_firstFreq] = *pz_tor;
  pz_multiPathDetector->pu_CMCslipCount[u_firstFreq] <<= 1;
  if (fabs(d_ave_est - pd_CMCvalue) > d_maxSlip)
  {
    pz_multiPathDetector->pu_CMCslipCount[u_firstFreq] |= 1;
    for (u_i = 0; u_i < u_maxEpochCount; u_i++)
    {
      if ((pz_multiPathDetector->pu_CMCslipCount[u_firstFreq] >> u_i) & 1)
      {
        u_sumSlipEpoch++;
      }
    }
    if (u_sumSlipEpoch >= u_maxSlipCount)
    {
      cmn_cleanMultiPathDetector_CMC(pz_multiPathDetector);
    }
  }
  else if (fabs(d_ave_est - d_CMCvalue_pre) < d_maxDeltaAve)
  {
    pz_multiPathDetector->pd_CMCvalue[u_firstFreq] = d_ave_est;
    pz_multiPathDetector->pd_CMCvalueSTD[u_firstFreq] = d_std_est;
    if (w_epochNum > d_minEpochNum)
    {
      if (d_std_est > d_highThres_STD || fabs(d_ave_est - pd_CMCvalue) > d_highThres_AVE)
      {
        pz_multiPathDetector->pu_CMCmultipathLevel[u_firstFreq] = OBS_MP_LEVEL_HIGH;
      }
      else if (d_std_est > d_lowThres_STD || fabs(d_ave_est - pd_CMCvalue) > d_lowThres_AVE)
      {
        pz_multiPathDetector->pu_CMCmultipathLevel[u_firstFreq] = OBS_MP_LEVEL_LOW;
      }
      else
      {
        pz_multiPathDetector->pu_CMCmultipathLevel[u_firstFreq] = OBS_MP_LEVEL_NORMAL;
      }
    }
    else
    {
      pz_multiPathDetector->pu_CMCmultipathLevel[u_firstFreq] = OBS_MP_LEVEL_UNKNOWEN;
    }
  }
  else
  {
    cmn_cleanMultiPathDetector_CMC(pz_multiPathDetector);
  }


}

/**
 * @brief       use GIF combine detect the multipath of one PR
 * @param[in]   pz_tor is observation time
 * @param[in]   pd_GIFvalue is GIF combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_GIFdetectMP_onePR(const GpsTime_t* pz_tor, double pd_GIFvalue, float f_ele,
  cmn_oneSatMultipathDetectInfo* pz_multiPathDetector)
{
  uint8_t u_i = 0;
  uint8_t u_sumSlipEpoch = 0;
  uint8_t u_MPlevel = OBS_MP_LEVEL_UNKNOWEN;
  uint16_t w_epochNum = 0;
  double d_GIFvalue_pre = 0.0;
  double d_GIFvalueSTD_pre = 0.0;
  double d_ave_est = 0;
  double d_std_est = 0;
  double d_intv = 0.0;

  /* threshold define begin */
  const uint8_t u_maxSlipCount = 2;   //max slip epoch num in u_maxEpochCount epochs
  const uint8_t u_maxEpochCount = 3;   //stastic epoch num of slip
  const double d_minEpochNum = 5.0 / gf_sample;   //min epoch num to detect MP
  const double d_maxEpochArc = 300.0 / gf_sample;   //min epoch num to detect MP
  const double d_maxDeltaTime = 60.0; //max time diff
  const double d_maxDeltaAve = 1.0; //max ave diff between epoch
  const double d_maxSlip = 1.0; //max diff between pd_CMCvalue & ave
  const double d_lowThres_STD = 0.1; //threshold of std in low MP level
  const double d_highThres_STD = 0.2; //threshold of std in high MP level
  const double d_lowThres_AVE = 0.25; //threshold of ave in low MP level
  const double d_highThres_AVE = 0.50; //threshold of ave in high MP level
  /* threshold define end */

  d_intv = fabs(tm_GpsTimeDiff(pz_tor, &(pz_multiPathDetector->z_GIFendTime)));
  pz_multiPathDetector->f_eleA = f_ele;
  if (0 == (pz_multiPathDetector->w_GIFepochNum) || d_intv > d_maxDeltaTime || d_intv <= 0.0)
  {
    pz_multiPathDetector->w_GIFepochNum = 1;
    pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_UNKNOWEN;
    pz_multiPathDetector->d_GIFvalue = pd_GIFvalue;
    pz_multiPathDetector->d_GIFvalueSTD = 0.0;
    pz_multiPathDetector->z_GIFendTime = *pz_tor;
    return;
  }
  w_epochNum = pz_multiPathDetector->w_GIFepochNum;
  d_GIFvalue_pre = pz_multiPathDetector->d_GIFvalue;
  d_GIFvalueSTD_pre = pz_multiPathDetector->d_GIFvalueSTD;
  d_ave_est = (d_GIFvalue_pre * w_epochNum + pd_GIFvalue) / (w_epochNum + 1.0);
  d_std_est = d_GIFvalueSTD_pre * (w_epochNum - 1.0) / (double)w_epochNum
    + SQR(d_ave_est - d_GIFvalue_pre) + SQR(d_ave_est - pd_GIFvalue) / (double)w_epochNum;

  if (w_epochNum < d_maxEpochArc)
  {
    pz_multiPathDetector->w_GIFepochNum += 1;
  }
  pz_multiPathDetector->z_GIFendTime = *pz_tor;
  pz_multiPathDetector->u_GIFslipCount <<= 1;
  if (fabs(d_GIFvalue_pre - pd_GIFvalue) > d_maxSlip)
  {
    pz_multiPathDetector->u_GIFslipCount |= 1;
    for (u_i = 0; u_i < u_maxEpochCount; u_i++)
    {
      if ((pz_multiPathDetector->u_GIFslipCount >> u_i) & 1)
      {
        u_sumSlipEpoch++;
      }
    }
    if (u_sumSlipEpoch >= u_maxSlipCount)
    {
      cmn_cleanMultiPathDetector_GIF(pz_multiPathDetector);
    }
  }
  else if (fabs(d_ave_est - d_GIFvalue_pre) < d_maxDeltaAve)
  {
    pz_multiPathDetector->d_GIFvalue = d_ave_est;
    pz_multiPathDetector->d_GIFvalueSTD = d_std_est;
    if (w_epochNum > d_minEpochNum)
    {
      if (d_std_est > d_highThres_STD || fabs(d_ave_est - pd_GIFvalue) > d_highThres_AVE)
      {
        pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_HIGH;
      }
      else if (d_std_est > d_lowThres_STD || fabs(d_ave_est - pd_GIFvalue) > d_lowThres_AVE)
      {
        pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_LOW;
      }
      else
      {
        pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_NORMAL;
      }
    }
    else
    {
      pz_multiPathDetector->u_GIFmultipathLevel = OBS_MP_LEVEL_UNKNOWEN;
    }
  }
  else
  {
    cmn_cleanMultiPathDetector_GIF(pz_multiPathDetector);
  }


}
/**
 * @brief       use CMC combine detect the multipath of one PR
 * @param[in]   u_constellation is index of constellation
 * @param[in]   u_targetFreq is index of freq
 * @param[in]   d_SNR is SNR value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_SNRdetectMP_onePR(uint8_t u_constellation, uint8_t u_targetFreq, double d_SNR, float f_ele,
  cmn_oneSatMultipathDetectInfo* pz_multiPathDetector, cmn_multipathDetectSet* pz_MPdetectorSet)
{
  uint8_t u_status = FALSE;
  uint8_t u_x = 0;
  uint8_t u_i = 0;
  uint16_t w_index_front = 0;
  uint16_t w_index_behind = 0;
  double pd_ele_x[NumOfSegments] = { 15.0, 30.0, 45.0, 60.0, 90.0 };
  double d_scale = 0.0;
  double d_snr_upper_front = 0.0;
  double d_snr_upper_behind = 0.0;
  double d_snr_lower_front = 0.0;
  double d_snr_lower_behind = 0.0;
  double d_snr_upper = 0.0;
  double d_snr_lower = 0.0;

  /* threshold define begin */
  const double d_dSNR_MP_low = 3.0;   //delta SNR threshold for low multipath
  const double d_dSNR_MP_high = 6.0;   //delta SNR threshold for high multipath
  /* threshold define end */

  if (f_ele <= 15.0 || f_ele >= 90.0 || u_targetFreq >= C_GNSS_FREQ_TYPE_MAX)
  {
    return;
  }
  if (d_SNR < FABS_ZEROS || d_SNR > 65.0)
  {
    pz_multiPathDetector->pu_SNRmultipathLevel[u_targetFreq] = OBS_MP_LEVEL_HIGH;
    return;
  }

  for (u_i = 0; u_i < NumOfSegments - 1; u_i++)
  {
    if (f_ele > pd_ele_x[u_i] && f_ele < pd_ele_x[u_i + 1])
    {
      break;
    }
  }

  if (u_i >= NumOfSegments || u_targetFreq >= C_GNSS_FREQ_TYPE_MAX || u_constellation >= C_GNSS_MAX)
  {
    return;
  }

  d_scale = (f_ele - pd_ele_x[u_i]) / (pd_ele_x[u_i + 1] - pd_ele_x[u_i]);

  w_index_front = cmn_getEleSnrBoundIndex(u_constellation, u_targetFreq, u_i);
  w_index_behind = cmn_getEleSnrBoundIndex(u_constellation, u_targetFreq, u_i + 1);
  d_snr_upper_front = pz_MPdetectorSet->pd_ele_snr_bound_upper[w_index_front];
  d_snr_upper_behind = pz_MPdetectorSet->pd_ele_snr_bound_upper[w_index_behind];
  d_snr_lower_front = pz_MPdetectorSet->pd_ele_snr_bound_lower[w_index_front];
  d_snr_lower_behind = pz_MPdetectorSet->pd_ele_snr_bound_lower[w_index_behind];
  if (d_snr_upper_front > FABS_ZEROS && d_snr_upper_behind > FABS_ZEROS &&
    d_snr_lower_front > FABS_ZEROS && d_snr_lower_behind > FABS_ZEROS)
  {
    d_snr_upper = (1 - d_scale) * d_snr_upper_front + d_scale * d_snr_upper_behind;
    d_snr_lower = (1 - d_scale) * d_snr_lower_front + d_scale * d_snr_lower_behind;
  }
  if (d_snr_upper > FABS_ZEROS && d_snr_lower > FABS_ZEROS)
  {
    if (d_SNR - d_snr_upper > d_dSNR_MP_high || d_snr_lower - d_SNR > d_dSNR_MP_high)
    {
      pz_multiPathDetector->pu_SNRmultipathLevel[u_targetFreq] = OBS_MP_LEVEL_HIGH;
    }
    else if (d_SNR - d_snr_upper > d_dSNR_MP_low || d_snr_lower - d_SNR > d_dSNR_MP_low)
    {
      pz_multiPathDetector->pu_SNRmultipathLevel[u_targetFreq] = OBS_MP_LEVEL_LOW;
    }
    else
    {
      pz_multiPathDetector->pu_SNRmultipathLevel[u_targetFreq] = OBS_MP_LEVEL_NORMAL;
    }
  }
  else
  {
    pz_multiPathDetector->pu_SNRmultipathLevel[u_targetFreq] = OBS_MP_LEVEL_UNKNOWEN;
  }
}
/**
 * @brief the inteface of using SNR to detect multipath
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_SNRdetectMP(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetectorSet)
{
  uint8_t u_status = 1;
  uint16_t w_iSat = 0;
  char char_sat[4] = "";
  uint8_t u_iFreq = 0;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  double d_wave = 0.0;

  /* threshold define begin */
  double d_minEle = 15.0 * DEG2RAD; //min elevation
  /* threshold define end */

  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[w_iSat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < d_minEle)
    {
      continue;
    }
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; ++u_iFreq)
    {
      if (NULL == pz_satMeas->pz_signalMeas[u_iFreq])
      {
        continue;
      }
      if (!pz_satMeas->pz_signalMeas[u_iFreq]->z_measStatusFlag.b_valid ||
        fabs(pz_satMeas->pz_signalMeas[u_iFreq]->f_cn0) < 1.0e-3)
      {
        continue;
      }
      d_wave = wavelength(pz_satMeas->pz_signalMeas[u_iFreq]->u_signal);
      if (d_wave >= 1.0)
      {
        continue;
      }
      if (NULL == pz_MPdetectorSet->pz_multiPathSet[w_iSat])
      {
        pz_MPdetectorSet->pz_multiPathSet[w_iSat] =
          (cmn_oneSatMultipathDetectInfo*)OS_MALLOC(sizeof(cmn_oneSatMultipathDetectInfo));
        cmn_initMPdetectorInfo(pz_MPdetectorSet->pz_multiPathSet[w_iSat]);
      }
      if (NULL == pz_MPdetectorSet->pz_multiPathSet[w_iSat])
      {
        continue;
      }
      /*if (C_GNSS_BDS2 == pz_satMeas->u_constellation && C_GNSS_FREQ_TYPE_L5 == u_iFreq)
      {
        continue;
      }*/
      cmn_SNRdetectMP_onePR(pz_satMeas->u_constellation, u_iFreq, pz_satMeas->pz_signalMeas[u_iFreq]->f_cn0,
        (float)(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG), pz_MPdetectorSet->pz_multiPathSet[w_iSat], pz_MPdetectorSet);
    }
  }

  return u_status;
}
/**
 * @brief the inteface of using code minus carrier method to detect multipath
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_CMCdetectMP(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetectorSet)
{

  uint8_t u_status = 1;
  uint16_t w_iSat = 0;
  char char_sat[4] = "";
  gnss_SignalType u_firstSignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_secondSignalType = C_GNSS_SIG_MAX;
  uint8_t u_iFreq = 0;
  uint8_t u_jFreq = 0;
  gnss_FreqType z_firstFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_FreqType z_twinFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  double d_wave1 = 0.0;
  double d_wave2 = 0.0;
  double d_wave_denominator = 0.0;
  double pd_CMCvalue = 0.0;

  /* threshold define begin */
  double d_minEle = 15.0 * DEG2RAD; //min elevation
  /* threshold define end */

  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[w_iSat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < d_minEle)
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
      if (!pz_satMeas->pz_signalMeas[u_iFreq]->z_measStatusFlag.b_valid ||
        !pz_satMeas->pz_signalMeas[u_iFreq]->z_measStatusFlag.b_prValid ||
        fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase) < 1.0e-3 ||
        fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_pseudoRange) < 1.0e-3)
      {
        continue;
      }
      d_wave1 = wavelength(u_firstSignalType);
      if (d_wave1 >= 1.0)
      {
        continue;
      }
      switch (u_iFreq)
      {
      case(C_GNSS_FREQ_TYPE_L1):
        u_jFreq = C_GNSS_FREQ_TYPE_L2; break;
      case(C_GNSS_FREQ_TYPE_L2):
        u_jFreq = C_GNSS_FREQ_TYPE_L1; break;
      case(C_GNSS_FREQ_TYPE_L5):
        u_jFreq = C_GNSS_FREQ_TYPE_L1; break;
      default:
        continue; break;
      }

      if (NULL == pz_satMeas->pz_signalMeas[u_jFreq])
      {
        continue;
      }
      if (!pz_satMeas->pz_signalMeas[u_jFreq]->z_measStatusFlag.b_valid ||
        !pz_satMeas->pz_signalMeas[u_jFreq]->z_measStatusFlag.b_prValid ||
        fabs(pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase) < 1.0e-3 ||
        fabs(pz_satMeas->pz_signalMeas[u_jFreq]->d_pseudoRange) < 1.0e-3)
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
      if (NULL == pz_MPdetectorSet->pz_multiPathSet[w_iSat])
      {
        pz_MPdetectorSet->pz_multiPathSet[w_iSat] =
          (cmn_oneSatMultipathDetectInfo*)OS_MALLOC(sizeof(cmn_oneSatMultipathDetectInfo));
        cmn_initMPdetectorInfo(pz_MPdetectorSet->pz_multiPathSet[w_iSat]);
      }
      if (NULL == pz_MPdetectorSet->pz_multiPathSet[w_iSat])
      {
        continue;
      }
      d_wave_denominator = 1 / (d_wave2 * d_wave2 - d_wave1 * d_wave1);
      pd_CMCvalue = pz_satMeas->pz_signalMeas[u_iFreq]->d_pseudoRange
        - (d_wave1 * d_wave1 + d_wave2 * d_wave2) * d_wave_denominator * d_wave1 * pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase
        + 2 * d_wave1 * d_wave1 * d_wave_denominator * d_wave2 * pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase;

      cmn_CMCdetectMP_onePR(&(pz_satSignalCur->z_tor), u_iFreq, pd_CMCvalue, pz_satMeas->z_satPosVelClk.f_elevation,
        pz_MPdetectorSet->pz_multiPathSet[w_iSat]);

      /*if (pz_MPdetectorSet->pz_multiPathSet[w_iSat]->pw_CMCepochNum[u_iFreq] > 5)
      {
        LOGI(TAG_HPP, "-mp=,%d%d,%lf,%lf,%d\n", u_iFreq, w_iSat,
          pz_MPdetectorSet->pz_multiPathSet[w_iSat]->pd_CMCvalue[u_iFreq],
          pz_MPdetectorSet->pz_multiPathSet[w_iSat]->pd_CMCvalueSTD[u_iFreq],
          pz_MPdetectorSet->pz_multiPathSet[w_iSat]->pw_CMCepochNum[u_iFreq]);
      }*/

    }
  }

  return u_status;
}

/**
 * @brief the inteface of using geo-inno-free method to detect multipath
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_GIFdetectMP(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetectorSet)
{

  uint8_t u_status = 1;
  uint16_t w_iSat = 0;
  char char_sat[4] = "";
  gnss_SignalType u_firstSignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_secondSignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_thirdSignalType = C_GNSS_SIG_MAX;
  uint8_t u_iFreq = C_GNSS_FREQ_TYPE_L1;
  uint8_t u_jFreq = C_GNSS_FREQ_TYPE_L2;
  uint8_t u_kFreq = C_GNSS_FREQ_TYPE_L5;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  double d_wave1 = 0.0;
  double d_wave2 = 0.0;
  double d_wave3 = 0.0;
  double d_wave_denominator = 0.0;
  double pd_GIFvalue = 0.0;

  /* threshold define begin */
  double d_minEle = 15.0 * DEG2RAD; //min elevation
  /* threshold define end */

  for (w_iSat = 0; w_iSat < ALL_GNSS_SYS_SV_NUMBER; ++w_iSat)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[w_iSat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < d_minEle)
    {
      continue;
    }
    if (NULL == pz_satMeas->pz_signalMeas[u_iFreq] ||
      NULL == pz_satMeas->pz_signalMeas[u_jFreq] ||
      NULL == pz_satMeas->pz_signalMeas[u_kFreq])
    {
      continue;
    }
    if (!pz_satMeas->pz_signalMeas[u_iFreq]->z_measStatusFlag.b_valid ||
      !pz_satMeas->pz_signalMeas[u_iFreq]->z_measStatusFlag.b_prValid ||
      fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase) < 1.0e-3 ||
      fabs(pz_satMeas->pz_signalMeas[u_iFreq]->d_pseudoRange) < 1.0e-3)
    {
      continue;
    }
    u_firstSignalType = pz_satMeas->pz_signalMeas[u_iFreq]->u_signal;
    d_wave1 = wavelength(u_firstSignalType);
    if (d_wave1 >= 1.0)
    {
      continue;
    }

    if (!pz_satMeas->pz_signalMeas[u_jFreq]->z_measStatusFlag.b_valid ||
      !pz_satMeas->pz_signalMeas[u_jFreq]->z_measStatusFlag.b_prValid ||
      fabs(pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase) < 1.0e-3 ||
      fabs(pz_satMeas->pz_signalMeas[u_jFreq]->d_pseudoRange) < 1.0e-3)
    {
      continue;
    }
    u_secondSignalType = (gnss_SignalType)(pz_satMeas->pz_signalMeas[u_jFreq]->u_signal);
    d_wave2 = wavelength(u_secondSignalType);
    if (d_wave2 >= 1.0)
    {
      continue;
    }

    if (!pz_satMeas->pz_signalMeas[u_kFreq]->z_measStatusFlag.b_valid ||
      !pz_satMeas->pz_signalMeas[u_kFreq]->z_measStatusFlag.b_prValid ||
      fabs(pz_satMeas->pz_signalMeas[u_kFreq]->d_carrierPhase) < 1.0e-3 ||
      fabs(pz_satMeas->pz_signalMeas[u_kFreq]->d_pseudoRange) < 1.0e-3)
    {
      continue;
    }
    u_thirdSignalType = (gnss_SignalType)(pz_satMeas->pz_signalMeas[u_kFreq]->u_signal);
    d_wave3 = wavelength(u_thirdSignalType);
    if (d_wave3 >= 1.0)
    {
      continue;
    }

    if (NULL == pz_MPdetectorSet->pz_multiPathSet[w_iSat])
    {
      pz_MPdetectorSet->pz_multiPathSet[w_iSat] =
        (cmn_oneSatMultipathDetectInfo*)OS_MALLOC(sizeof(cmn_oneSatMultipathDetectInfo));
      cmn_initMPdetectorInfo(pz_MPdetectorSet->pz_multiPathSet[w_iSat]);
    }
    if (NULL == pz_MPdetectorSet->pz_multiPathSet[w_iSat])
    {
      continue;
    }

    d_wave_denominator = 1 / SQR(SQR(d_wave1) - SQR(d_wave2)) + SQR(SQR(d_wave2) - SQR(d_wave3)) + SQR(SQR(d_wave3) - SQR(d_wave1));
    pd_GIFvalue = (SQR(d_wave3) - SQR(d_wave2)) * d_wave_denominator * d_wave1 * pz_satMeas->pz_signalMeas[u_iFreq]->d_carrierPhase
      + (SQR(d_wave1) - SQR(d_wave3)) * d_wave_denominator * d_wave2 * pz_satMeas->pz_signalMeas[u_jFreq]->d_carrierPhase
      + (SQR(d_wave2) - SQR(d_wave1)) * d_wave_denominator * d_wave3 * pz_satMeas->pz_signalMeas[u_kFreq]->d_carrierPhase;

    cmn_GIFdetectMP_onePR(&(pz_satSignalCur->z_tor), pd_GIFvalue, pz_satMeas->z_satPosVelClk.f_elevation,
      pz_MPdetectorSet->pz_multiPathSet[w_iSat]);

    /*if (pz_MPdetectorSet->pz_multiPathSet[w_iSat]->w_GIFepochNum > 2)
    {
      LOGI(TAG_HPP, "-GIF=,%d,%lf,%lf,%d\n", w_iSat,
        pz_MPdetectorSet->pz_multiPathSet[w_iSat]->d_GIFvalue,
        pz_MPdetectorSet->pz_multiPathSet[w_iSat]->d_GIFvalueSTD,
        pz_MPdetectorSet->pz_multiPathSet[w_iSat]->w_GIFepochNum);
    }*/

  }

  return u_status;
}

/**
 * @brief the inteface of print multipath detecting flag
 * @param[in]  pz_satSignalCur is the observation information
 * @param[in]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @param[in]  pz_RTKmultiPathFlag the Multipath flag, see MULTIPATH_FLAG_...
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_MPflagPrint(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetectorSet, GnssMultiPathFlag_t* pz_RTKmultiPathFlag)
{
  uint8_t u_iSat = 0;
  uint8_t u_defalutFlag = 0;
  uint8_t u_iFreq = 0;
  uint8_t u_minSize = 5;
  uint8_t u_size = 30;
  char char_sat[4] = "";
  char u_state;
  char pu_CMC_sat[MP_BUFF_SIZE];
  char pu_GIF_sat[MP_BUFF_SIZE];
  char pu_SNR_sat[MP_BUFF_SIZE];
  char* pu_CMC = pu_CMC_sat;
  char* pu_GIF = pu_GIF_sat;
  char* pu_SNR = pu_SNR_sat;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  cmn_oneSatMultipathDetectInfo* pz_multiPathDetector = NULL;

  /* threshold define begin */
  double d_minEle = 15.0 * DEG2RAD; //min elevation
  /* threshold define end */

  if (NULL == pz_RTKmultiPathFlag)
  {
    return FALSE;
  }

  memset(pu_CMC_sat, 0, MP_BUFF_SIZE * sizeof(char));
  memset(pu_GIF_sat, 0, MP_BUFF_SIZE * sizeof(char));
  memset(pu_SNR_sat, 0, MP_BUFF_SIZE * sizeof(char));
  for (u_iSat = 0; u_iSat < ALL_GNSS_SYS_SV_NUMBER; u_iSat++)
  {
    pz_satMeas = pz_satSignalCur->pz_satMeas[u_iSat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation < d_minEle)
    {
      continue;
    }
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; u_iFreq++)
    {
      if (NULL == pz_satMeas->pz_signalMeas[u_iFreq])
      {
        continue;
      }
      /*if (C_GNSS_BDS2 == pz_satMeas->u_constellation && C_GNSS_FREQ_TYPE_L5 == u_iFreq)
      {
        continue;
      }*/
      pz_multiPathDetector = pz_MPdetectorSet->pz_multiPathSet[u_iSat];
      if (NULL == pz_multiPathDetector)
      {
        continue;
      }
      satidx_SatString(u_iSat, char_sat);

      if ((pu_CMC - pu_CMC_sat) < MP_BUFF_SIZE - u_size * 2)
      {
        u_defalutFlag = FALSE;
        switch (pz_multiPathDetector->pu_CMCmultipathLevel[u_iFreq])
        {
        case(OBS_MP_LEVEL_LOW):
          u_state = 'L'; break;
        case(OBS_MP_LEVEL_HIGH):
          u_state = 'H'; break;
        default:
          u_defalutFlag = TRUE; break;
        }
        if (!u_defalutFlag)
        {
          pu_CMC += snprintf(pu_CMC, u_size, " %4s.%1d-%c |", char_sat, u_iFreq, u_state);
        }
      }

      if ((pu_GIF - pu_GIF_sat) < MP_BUFF_SIZE - u_size * 2)
      {
        u_defalutFlag = FALSE;
        switch (pz_multiPathDetector->u_GIFmultipathLevel)
        {
        case(OBS_MP_LEVEL_LOW):
          u_state = 'L'; break;
        case(OBS_MP_LEVEL_HIGH):
          u_state = 'H'; break;
        default:
          u_defalutFlag = TRUE; break;
        }
        if (!u_defalutFlag)
        {
          pu_GIF += snprintf(pu_GIF, u_size, " %4s.%1d-%c |", char_sat, u_iFreq, u_state);
        }
      }

      if ((pu_SNR - pu_SNR_sat) < MP_BUFF_SIZE - u_size * 2)
      {
        u_defalutFlag = FALSE;
        switch (pz_multiPathDetector->pu_SNRmultipathLevel[u_iFreq])
        {
        case(OBS_MP_LEVEL_UNKNOWEN):
          u_state = 'U'; break;
        case(OBS_MP_LEVEL_LOW):
          u_state = 'L'; break;
        case(OBS_MP_LEVEL_HIGH):
          u_state = 'H'; break;
        default:
          u_defalutFlag = TRUE; break;
        }
        if (!u_defalutFlag)
        {
          pu_SNR += snprintf(pu_SNR, u_size, " %4s.%1d-%c |", char_sat, u_iFreq, u_state);
        }
      }
    }
  }
  if ((pu_CMC - pu_CMC_sat) > u_minSize)
  {
    LOGI(TAG_HPP, "...MP detect...CMC...: |%s\n", pu_CMC_sat);
  }
  if ((pu_GIF - pu_GIF_sat) > u_minSize)
  {
    LOGI(TAG_HPP, "...MP detect...GIF...: |%s\n", pu_GIF_sat);
  }
  if ((pu_SNR - pu_SNR_sat) > u_minSize)
  {
    LOGI(TAG_HPP, "...MP detect...SNR...: |%s\n", pu_SNR_sat);
  }
  return 1;
}
/**
 * @brief the inteface of detect multipath
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @param[out]  pz_RTKmultiPathFlag the Multipath flag, see MULTIPATH_FLAG_...
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_MPflagDetermine(cmn_multipathDetectSet* pz_MPdetectorSet, GnssMultiPathFlag_t* pz_RTKmultiPathFlag)
{
  uint8_t u_iSat = 0;
  uint8_t u_iFreq = 0;
  cmn_oneSatMultipathDetectInfo* pz_multiPathDetector;

  if (NULL == pz_RTKmultiPathFlag)
  {
    return FALSE;
  }

  for (u_iSat = 0; u_iSat < ALL_GNSS_SYS_SV_NUMBER; u_iSat++)
  {
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; u_iFreq++)
    {
      pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] = 0x0;
    }
  }
  for (u_iSat = 0; u_iSat < ALL_GNSS_SYS_SV_NUMBER; u_iSat++)
  {
    for (u_iFreq = 0; u_iFreq < MAX_GNSS_SIGNAL_FREQ; u_iFreq++)
    {
      pz_multiPathDetector = pz_MPdetectorSet->pz_multiPathSet[u_iSat];
      if (NULL == pz_multiPathDetector)
      {
        continue;
      }

      switch (pz_multiPathDetector->pu_CMCmultipathLevel[u_iFreq])
      {
      case(OBS_MP_LEVEL_UNKNOWEN):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_PR_UNKNOW; break;
      case(OBS_MP_LEVEL_NORMAL):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_PR_NORMAL; break;
      case(OBS_MP_LEVEL_LOW):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_PR_LOW; break;
      case(OBS_MP_LEVEL_HIGH):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_PR_HIGH; break;
      default:
        continue; break;
      }

      switch (pz_multiPathDetector->u_GIFmultipathLevel)
      {
      case(OBS_MP_LEVEL_UNKNOWEN):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_CP_UNKNOW; break;
      case(OBS_MP_LEVEL_NORMAL):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_CP_NORMAL; break;
      case(OBS_MP_LEVEL_LOW):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_CP_LOW; break;
      case(OBS_MP_LEVEL_HIGH):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= MULTIPATH_FLAG_CP_HIGH; break;
      default:
        continue; break;
      }

      switch (pz_multiPathDetector->pu_SNRmultipathLevel[u_iFreq])
      {
      case(OBS_MP_LEVEL_UNKNOWEN):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= (MULTIPATH_FLAG_CP_UNKNOW | MULTIPATH_FLAG_PR_UNKNOW); break;
      case(OBS_MP_LEVEL_NORMAL):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= (MULTIPATH_FLAG_CP_NORMAL | MULTIPATH_FLAG_PR_NORMAL); break;
      case(OBS_MP_LEVEL_LOW):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= (MULTIPATH_FLAG_CP_LOW | MULTIPATH_FLAG_PR_LOW); break;
      case(OBS_MP_LEVEL_HIGH):
        pz_RTKmultiPathFlag->pu_MPflag[u_iSat][u_iFreq] |= (MULTIPATH_FLAG_CP_HIGH | MULTIPATH_FLAG_PR_HIGH); break;
      default:
        continue; break;
      }

    }
  }
  return 0;
}
/**
 * @brief the inteface of detect multipath
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @param[out]  pz_RTKmultiPathFlag the Multipath flag, see MULTIPATH_FLAG_...
 * @param[out]  pu_threeFreqTag filterInfo's flag of whether date have three freq
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_detectMultiPath(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetectorSet,
  GnssMultiPathFlag_t** pz_RTKmultiPathFlag, uint8_t pu_sigMeasCount[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX], uint8_t* pu_threeFreqTag)
{

  uint8_t u_status = 1;
  uint8_t u_GPSthreeFreqTag = FALSE;
  uint8_t u_BDSthreeFreqTag = FALSE;
  LogLevelEnum z_logLevel;

  if (NULL != pu_threeFreqTag && FALSE == *pu_threeFreqTag)
  {
    if (pu_sigMeasCount[C_GNSS_BDS3][C_GNSS_FREQ_TYPE_L1] > 3 &&
      pu_sigMeasCount[C_GNSS_BDS3][C_GNSS_FREQ_TYPE_L2] > 3 &&
      pu_sigMeasCount[C_GNSS_BDS3][C_GNSS_FREQ_TYPE_L5] > 1)
    {
      u_BDSthreeFreqTag = TRUE;
    }
    if (pu_sigMeasCount[C_GNSS_GPS][C_GNSS_FREQ_TYPE_L1] > 3 &&
      pu_sigMeasCount[C_GNSS_GPS][C_GNSS_FREQ_TYPE_L2] > 3 &&
      pu_sigMeasCount[C_GNSS_GPS][C_GNSS_FREQ_TYPE_L5] > 3)
    {
      u_GPSthreeFreqTag = TRUE;
    }
    if (u_GPSthreeFreqTag && u_BDSthreeFreqTag)
    {
      *pu_threeFreqTag = TRUE;
    }
  }
  if (NULL != pu_threeFreqTag && *pu_threeFreqTag)
  {

    if (NULL == *pz_RTKmultiPathFlag)
    {
      *pz_RTKmultiPathFlag = (GnssMultiPathFlag_t*)OS_MALLOC(sizeof(GnssMultiPathFlag_t));
    }

    if (NULL == *pz_RTKmultiPathFlag)
    {
      LOGW(TAG_HPP, "%s pz_SatFilterObs malloc failed", __FUNCTION__);
      u_status = 0;
    }
    else
    {
      cmn_CMCdetectMP(pz_satSignalCur, pz_MPdetectorSet);
      cmn_GIFdetectMP(pz_satSignalCur, pz_MPdetectorSet);
      cmn_SNRdetectMP(pz_satSignalCur, pz_MPdetectorSet);
      cmn_MPflagDetermine(pz_MPdetectorSet, *pz_RTKmultiPathFlag);
      z_logLevel = log_GetLogLevel();
      if (z_logLevel >= LOG_LEVEL_I)
      {
        cmn_MPflagPrint(pz_satSignalCur, pz_MPdetectorSet, *pz_RTKmultiPathFlag);
      }
    }
  }
  return u_status;
}
#else
/**
 * @brief the inteface of initilizing the MP detector information
 * @param[out]  pz_MPdetectorSet is the obs multipath detector set
 * @param[in]  f_sample background process sample
 * @return     void
 */
void cmn_initMPdetectorSet(cmn_multipathDetectSet* pz_MPdetectorSet, float f_sample)
{
  return;
}
/**
 * @brief the inteface of deinitilizing the MP detector information
 * @param[out]  pz_MPdetectorSet is the obs multipath detector set
 * @return     void
 */
void cmn_deinitMPdetectorSet(cmn_multipathDetectSet* pz_MPdetectorSet)
{
  return;
}
/**
 * @brief initilize the multipath detector
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_initMPdetectorInfo(cmn_oneSatMultipathDetectInfo* pz_multiPathDetector)
{
  return;
}
/**
 * @brief the inteface of detect multipath
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_detectMultiPath(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetectorSet,
  GnssMultiPathFlag_t** pz_RTKmultiPathFlag, uint8_t pu_sigMeasCount[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX], uint8_t* pu_threeFreqTag)
{
  return 0;
}
/**
 * @brief update flag of the multipath detector between epoch
 * @param[out]  pz_multiPathDetector the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_epochUpdateMPdetectorInfo(cmn_multipathDetectSet* pz_MPdetectorSet)
{
  return;
}
#endif