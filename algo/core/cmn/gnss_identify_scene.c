#include "gnss_identify_scene.h"
#include "gnss_common.h"
/**
 * @brief using the GNSS observations to identify the scene of the current epoch
 * @param[in] pz_satSigMeasCollect - the struct of satellite measure clooect
 * @return OPEN_SKY_SCENE represent the open sky, SEMI_SKY_SCENE represent the semi sky and CLOSE_SKY_SCENE is the complex scene
 */
gnss_SceneType gnss_usingObsIdentifyScene(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  gnss_SceneType z_sceneType = OPEN_SKY_SCENE;
  uint16_t w_satNum = 0;
  uint16_t w_cn0Num = 0;
  uint16_t w_LLInum = 0;
  uint16_t w_noLLInum = 0;
  uint8_t u_signalType = 0;
  uint8_t u_signalIndex = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_L1;
  uint32_t q_satIndex = 0;
  uint32_t u_svIdx = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_constellationSceneUsed = 0;
  uint8_t u_svid = 0;
  uint8_t u_sysNum = 0;
  uint8_t u_nSatCloseThres = 10;
  uint8_t u_nSatOpenThres = 10;
  uint8_t u_nSatNoLLIthres = 7;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_SignalMeas_t* pz_L1SigMeas = NULL;
  float f_avgCN0 = 0.0;
  float f_maxCN0 = 0.0;
  float f_avgCN0Thres = 35.0;
  float f_CN0_35Pct = 0.5;
  uint8_t pu_LLIsatNum[C_GNSS_MAX] = { 0 };
  uint8_t pu_noLLIsatNum[C_GNSS_MAX] = { 0 };
  uint8_t pu_satNumPerSys[C_GNSS_MAX] = { 0 };
  uint8_t pu_CN0greater35NumPerSys[C_GNSS_MAX] = { 0 };
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = (const gnss_SatelliteMeas_t*)(pz_satSigMeasCollect->pz_satMeas[q_satIndex]);
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = pz_satMeas->u_constellation;
    u_constellationSceneUsed = u_constellation;
    if (C_GNSS_BDS2 == u_constellation || C_GNSS_BDS3 == u_constellation)
    {
      u_constellationSceneUsed = C_GNSS_BDS3;
    }
    u_svid = pz_satMeas->u_svid;
    u_svIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
    if (ALL_GNSS_SYS_SV_NUMBER == u_svIdx)
    {
      continue;
    }
    pz_L1SigMeas = NULL;
    /* loop for signal */
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = (const gnss_SignalMeas_t*)(pz_satMeas->pz_signalMeas[u_signalIndex]);
      if (NULL == pz_sigMeas || (pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      u_signalType = pz_sigMeas->u_signal;
      u_freqType = gnss_cvt_Sig2FreqType(u_signalType);
      if (C_GNSS_FREQ_TYPE_L1 == u_freqType)
      {
          pz_L1SigMeas = pz_sigMeas;
          break;
      }
    }
    if (NULL == pz_L1SigMeas)
    {
      continue;
    }
    ++pu_satNumPerSys[u_constellationSceneUsed];
    ++w_satNum;
    //start for CN0
    if ((pz_L1SigMeas->f_cn0) > 0.0)
    {
        f_avgCN0 += (pz_L1SigMeas->f_cn0);
        ++w_cn0Num;
        if ((pz_L1SigMeas->f_cn0) > f_maxCN0)
        {
          f_maxCN0 = (pz_L1SigMeas->f_cn0);
        }
        if ((pz_L1SigMeas->f_cn0) >= 35.0)
        {
          ++pu_CN0greater35NumPerSys[u_constellationSceneUsed];
        }
    }
    //end for CN0

    //start for LLI
    if((pz_L1SigMeas->u_LLI) > 0)
    {
      ++pu_LLIsatNum[u_constellationSceneUsed];
      ++w_LLInum;
    }
    else
    {
      ++pu_noLLIsatNum[u_constellationSceneUsed];
      ++w_noLLInum;
    }
    //end for LLI
  }
  if (w_cn0Num > 0)
  {
    f_avgCN0 /= w_cn0Num;
  }
  //calculating the number of constellation
  for (u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_MAX; ++u_constellation)
  {
    if (pu_satNumPerSys[u_constellation] > 0)
    {
      ++u_sysNum;
    }
  }

  if (1 == u_sysNum)
  {
    u_nSatCloseThres = 4;
    u_nSatOpenThres = 6;
  }
  else if (2 == u_sysNum)
  {
    u_nSatCloseThres = 8;
    u_nSatOpenThres = 12;
  }
  else if (u_sysNum >= 3)
  {
    u_nSatCloseThres = 12;
    u_nSatOpenThres = 18;
    u_nSatNoLLIthres = 10;
  }

  if (w_satNum <= u_nSatCloseThres || (pu_satNumPerSys[C_GNSS_GPS] < 4 && pu_satNumPerSys[C_GNSS_BDS3] < 4)
      || ((f_maxCN0 < 40 || f_avgCN0 < 30 || pu_CN0greater35NumPerSys[C_GNSS_GPS] < pu_satNumPerSys[C_GNSS_GPS] * 0.25) && w_LLInum > w_satNum * 0.5)
      || w_LLInum > w_satNum * 0.7
      || (pu_LLIsatNum[C_GNSS_GPS] > pu_satNumPerSys[C_GNSS_GPS] * 0.5 && pu_noLLIsatNum[C_GNSS_GPS] <= 4 && w_noLLInum < u_nSatNoLLIthres)
      || (f_avgCN0 < 35 && w_LLInum > w_satNum * 0.5 && pu_LLIsatNum[C_GNSS_GPS] > pu_satNumPerSys[C_GNSS_GPS] * 0.5))
  {
    z_sceneType = CLOSE_SKY_SCENE;
  }
  else if ((w_satNum > u_nSatOpenThres && (pu_satNumPerSys[C_GNSS_GPS] > 5 || pu_satNumPerSys[C_GNSS_BDS3] > 5) && f_avgCN0 >= f_avgCN0Thres
      && (pu_CN0greater35NumPerSys[C_GNSS_GPS] >= pu_satNumPerSys[C_GNSS_GPS] * f_CN0_35Pct || pu_CN0greater35NumPerSys[C_GNSS_BDS3] >= pu_satNumPerSys[C_GNSS_BDS3] * f_CN0_35Pct)
      && w_LLInum < w_satNum * 0.4 && (pu_LLIsatNum[C_GNSS_GPS] < pu_satNumPerSys[C_GNSS_GPS] * 0.4 || pu_LLIsatNum[C_GNSS_BDS3] < pu_satNumPerSys[C_GNSS_BDS3] * 0.4)
      && (pu_noLLIsatNum[C_GNSS_GPS] > 4 || pu_noLLIsatNum[C_GNSS_BDS3] > 4) && w_noLLInum > u_nSatNoLLIthres)
      ||(pu_satNumPerSys[C_GNSS_GPS] >= 8 && pu_CN0greater35NumPerSys[C_GNSS_GPS] >= 8 && pu_LLIsatNum[C_GNSS_GPS] <= 2 && pu_CN0greater35NumPerSys[C_GNSS_GPS] >= 0.75 * pu_satNumPerSys[C_GNSS_GPS]))
  {
    z_sceneType = OPEN_SKY_SCENE;
  }
  if (OPEN_SKY_SCENE == z_sceneType && u_sysNum <= 1 && w_LLInum > 2)
  {
    z_sceneType = SEMI_SKY_SCENE;
  }
  return z_sceneType;
}