#include "rtk_amb_fix.h"
#include "rtk_filter_sol.h"
#include "gnss_engine_api.h"
#include "rtk_integrity.h"

/**
 * @brief initialize the information for the RTK ambiguity resolution
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_filterObs is the information of observation after filter
 * @param[in]  pz_OSRblock is the OSR correction
 * @param[in]  pz_RTKfilterInfo is the EKF filter information of RTK
 * @param[in]  pz_EKFstateRepPool is the EKF state represent pool for RTK
 * @param[out] pz_RTKambFixInputInfo is the input information for RTK ambiguity fix
 * @return     void
 */
void rtk_setAmbFixInputInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, rtk_EpochFilterObs_t* pz_filterObs, const GnssCorrBlock_t* pz_OSRblock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_rtkAmbFixInputInfo_t* pz_RTKambFixInputInfo)
{
  uint8_t u_i = 0;
  pz_RTKambFixInputInfo->pz_satSignalPool = pz_satSigMeasCollect;
  pz_RTKambFixInputInfo->pz_OSRblock = pz_OSRblock;
  pz_RTKambFixInputInfo->pz_rtkFilterInfo = pz_RTKfilterInfo;
  pz_RTKambFixInputInfo->pz_EKFstateRepPool = pz_EKFstateRepPool;
  pz_RTKambFixInputInfo->pz_FilterObs = pz_filterObs;
  for (u_i = 0; u_i < MAX_SEARCH_LOOP; ++u_i)
  {
    pz_RTKambFixInputInfo->pu_searchLoopNb[u_i] = 0;
  }
  return;
}
/**
 * @brief calculate satellite cn0 and ele threshold
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @return		  u_status=0 fail, else success
 */
static uint8_t rtk_ambCalSatQos(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i;
  uint8_t u_j;
  uint8_t u_k;
  uint8_t u_status = 0;
  uint8_t u_thresIdx;
  uint8_t u_satIndex = 0;
  gnss_FreqType u_signalIndex;
  uint8_t u_count = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  float f_scale = 0.8f;
  float* f_cn0;
  float* f_ele;
  float* f_cn0Back;
  float* f_eleBack;
  float f_cn0Threshold = 0.0f;
  float f_eleThreshold = 0.0f;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const rtk_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const GpsTime_t* curtime = NULL;
  const gnss_MeasStatusFlag_t* pz_measUpdateFlag = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  float f_sample = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.f_sample;
  double d_timeLock = 0.0;

  f_cn0 = (float*)OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(float));
  f_ele = (float*)OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(float));
  f_cn0Back = (float*)OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(float));
  f_eleBack = (float*)OS_MALLOC_FAST(MAX_GNSS_ACTIVE_SAT_NUMBER * sizeof(float));

  if (any_Ptrs_Null(4, f_cn0, f_ele, f_cn0Back, f_eleBack))
  {
    if (NULL != f_cn0)
    {
      OS_FREE(f_cn0);
    }
    if (NULL != f_ele)
    {
      OS_FREE(f_ele);
    }
    if (NULL != f_cn0Back)
    {
      OS_FREE(f_cn0Back);
    }
    if (NULL != f_eleBack)
    {
      OS_FREE(f_eleBack);
    }
    return u_status;
  }

  curtime = &pz_rtkAmbFixInputInfo->pz_satSignalPool->z_tor;

  /* deal time lock */
  if (f_sample < 1e-3)
  {
    f_sample = 1.0f;
  }
  d_timeLock = (double)f_sample * GNSS_MIN_LOCK;

  for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
  {
    z_filterType = convertFreqAmb2FilterType(u_signalIndex);

    for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
    {
      pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
      pz_corrStat = pz_rtkAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_satIndex];
      if (NULL == pz_satMeas || NULL == pz_corrStat)
      {
        continue;
      }

      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      pz_measUpdateFlag = pz_corrStat->z_measUpdateFlag[u_signalIndex];
      if (NULL == pz_sigMeas || NULL == pz_measUpdateFlag)
      {
        continue;
      }

      if ((pz_sigMeas->u_LLI & 2) || !pz_measUpdateFlag->b_cpValid ||
        pz_satMeas->z_satPosVelClk.f_elevation < 20.0 * DEG2RAD)
      {
        continue;
      }

      w_x_id[0] = 0;
      w_x_id[1] = u_satIndex;
      w_x_id[2] = z_filterType;
      pz_satPool = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
      if (NULL == pz_satPool)
      {
        continue;
      }

      if (fabs(tm_GpsTimeDiff(curtime, &pz_satPool->z_endTime)) > 1e-4) /* unhealth,current epoch */
      {
        continue;
      }

      if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < d_timeLock)
      {
        continue;
      }

      if (u_count >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }

      f_cn0[u_count] = pz_sigMeas->f_cn0;
      f_ele[u_count++] = pz_satMeas->z_satPosVelClk.f_elevation;
    }
  }

  if (u_count > 10)
  {
    for (u_i = 0; u_i < u_count; u_i++)
    {
      f_cn0Back[u_i] = f_cn0[u_i];
      f_eleBack[u_i] = f_ele[u_i];
    }

    gnss_descSortFloat(f_cn0, u_count);
    gnss_descSortFloat(f_ele, u_count);

    for (u_j = 0; u_j < 5; u_j++)
    {
      u_thresIdx = (uint8_t)(u_count * (f_scale + 0.1 * u_j));

      if (u_thresIdx >= u_count)
      {
        break;
      }

      f_cn0Threshold = f_cn0[u_thresIdx];
      f_eleThreshold = f_ele[u_thresIdx];

      for (u_k = 0, u_i = 0; u_i < u_count; u_i++)
      {
        if (f_cn0Back[u_i] > f_cn0Threshold && f_eleBack[u_i] > f_eleThreshold)
        {
          u_k++;
        }
      }

      if (u_k > u_count * f_scale || u_k > 10)
      {
        break;
      }
    }

    u_status = 1;

    pz_fixedAmbPool->f_cn0Thres = (f_cn0Threshold > 38.0f) ? 38.0f : f_cn0Threshold;
    pz_fixedAmbPool->f_eleThres = (float)((f_eleThreshold > 40.0 * DEG2RAD) ? 40.0f * DEG2RAD : f_eleThreshold);

    LOGI(TAG_HPP, "zerocombine cn0 Threshold=%.1f, ele Threshold=%.1f\n", f_cn0Threshold, f_eleThreshold * RAD2DEG);
  }

  OS_FREE(f_cn0);
  OS_FREE(f_ele);
  OS_FREE(f_cn0Back);
  OS_FREE(f_eleBack);

  return u_status;
}

/**
 * @brief get reference satellite single frequence
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     u_targetConstellation
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     q_isArcLenLimit that TRUE limit the arc len and FALSE do not consider
 * @return svid
 */
static uint8_t rtk_ambFixRefSatSelectSf(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  uint8_t u_targetConstellation, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, BOOL q_isArcLenLimit)
{
  uint8_t u_satIndex = 0;
  uint8_t u_refsat = 0;
  uint8_t u_constellation = 0;
  uint8_t u_PAR_flag = pz_fixedAmbPool->u_ambfix_mode;// u_PAR_flag is PAR mode flag
  int16_t s_refsatIndex = -1;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const rtk_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const GpsTime_t* curtime = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  float f_sample = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.f_sample;
  double d_timeLock = 0.0;
  double d_ambArc = 0.0;
  double d_ambArc2 = 0.0;
  double d_refAmbLen = -1.0;
  float f_refEle = -1.0;
  BOOL q_isUpdateRef = FALSE;
  const double d_refSatMinEleLimit = 60.0 * DEG2RAD;

  curtime = &pz_rtkAmbFixInputInfo->pz_satSignalPool->z_tor;

  /* deal time lock */
  if (f_sample < 1e-3)
  {
    f_sample = 1.0f;
  }
  d_timeLock = (double)f_sample * GNSS_MIN_LOCK;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
    pz_corrStat = pz_rtkAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_satIndex];
    if (NULL == pz_satMeas || NULL == pz_corrStat)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetConstellation != u_constellation)
    {
      continue;
    }
    if (NULL == pz_satMeas->pz_signalMeas[z_freqType1] || NULL == pz_satMeas->pz_signalMeas[z_freqType2] ||
      NULL == pz_corrStat->z_measUpdateFlag[z_freqType1] || NULL == pz_corrStat->z_measUpdateFlag[z_freqType2])
    {
      continue;
    }

    if ((pz_satMeas->pz_signalMeas[z_freqType1]->u_LLI & 2) || !pz_corrStat->z_measUpdateFlag[z_freqType1]->b_cpValid ||
      (pz_satMeas->pz_signalMeas[z_freqType2]->u_LLI & 2) || !pz_corrStat->z_measUpdateFlag[z_freqType2]->b_cpValid ||
      pz_satMeas->z_satPosVelClk.f_elevation < 20.0 * DEG2RAD)
    {
      continue;
    }

    if (pz_satMeas->pz_signalMeas[z_freqType1]->f_cn0 < pz_fixedAmbPool->f_cn0Thres ||
      pz_satMeas->pz_signalMeas[z_freqType2]->f_cn0 < pz_fixedAmbPool->f_cn0Thres)
    {
      continue;
    }

    if (pz_satMeas->z_satPosVelClk.f_elevation < pz_fixedAmbPool->f_eleThres)
    {
      continue;
    }

    w_x_id[0] = 0;
    w_x_id[1] = u_satIndex;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);

    if (NULL == pz_satPool1 || NULL == pz_satPool2)
    {
      continue;
    }
    d_ambArc = fabs(tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime));
    d_ambArc2 = fabs(tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime));
    if (d_ambArc2 < d_ambArc)
    {
      d_ambArc = d_ambArc2;
    }
    if (TRUE == q_isArcLenLimit)
    {
      if (fabs(tm_GpsTimeDiff(curtime, &pz_satPool1->z_endTime)) > 1e-4 ||
        fabs(tm_GpsTimeDiff(curtime, &pz_satPool2->z_endTime)) > 1e-4) /* unhealth,current epoch */
      {
        continue;
      }
      if (d_ambArc < d_timeLock)
      {
        continue;
      }
    }

    if (u_PAR_flag == GNSS_PART_LAMBDA_MP && NULL != pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag)
    {
      if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_PR)
        != MULTIPATH_FLAG_PR_NORMAL)
      {
        continue;
      }
      if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_CP_HIGH) ||
        (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_CP_LOW))
      {
        continue;
      }
    }

    if (s_refsatIndex < 0)
    {
      d_refAmbLen = d_ambArc;
      f_refEle = pz_satMeas->z_satPosVelClk.f_elevation;
      s_refsatIndex = u_satIndex;
      continue;
    }
    q_isUpdateRef = FALSE;
    if ((pz_satMeas->z_satPosVelClk.f_elevation) > f_refEle)
    {
      if ((TRUE == pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->b_staticFlag))
      {
        q_isUpdateRef = TRUE;
      }
      else if (f_refEle > d_refSatMinEleLimit && d_ambArc >= d_refAmbLen)
      {
        q_isUpdateRef = TRUE;
      }
      else if (f_refEle <= d_refSatMinEleLimit)
      {
        q_isUpdateRef = TRUE;
      }
    }

    if (TRUE== q_isUpdateRef)
    {
      d_refAmbLen = d_ambArc;
      f_refEle = pz_satMeas->z_satPosVelClk.f_elevation;
      s_refsatIndex = u_satIndex;
    }
  }

  if (s_refsatIndex > 0)
  {
    u_refsat = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[s_refsatIndex]->u_svid;
  }

  return u_refsat;
}

/**
 * @brief print zero combine satellite information
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     u_nb number of fixed satellite
 * @return        void
 */
void rtk_zeroCombinePrintSat(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_AmbFix_t* pz_ambFix, uint8_t u_nb)
{
  char c_sat[4] = "";
  uint8_t u_i = 0;
  uint8_t u_isys;
  uint8_t u_size = 60;
  uint8_t u_sysIndex = 0;
  uint8_t u_constellation = 0;
  uint8_t u_satIndex = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  char satInfo[BUFF_SIZE] = { 0 };
  char* p_satInfo = satInfo;

  LOGI(TAG_HPP, " ===|====|==============L1==============|====|===============L2=============|====|==============L5=============|\n");
  LOGI(TAG_HPP, "      sat    lock     ele     cn0  fix    sat    lock     ele     cn0  fix    sat    lock     ele     cn0  fix\n");

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    memset(satInfo, 0, BUFF_SIZE * sizeof(char));
    p_satInfo = &satInfo[0];

    u_sysIndex = u_isys * MAX_GNSS_SIGNAL_FREQ;
    if (pz_fixedAmbPool->pw_refSat[u_sysIndex] == 0 &&
      (pz_fixedAmbPool->pw_refSat[u_sysIndex + 1] == 0) && (pz_fixedAmbPool->pw_refSat[u_sysIndex + 2] == 0))
    {
      continue;/* no ref sat */
    }

    if (pz_fixedAmbPool->pw_refSat[u_sysIndex] != 0)
    {
      svid_SatString(pz_fixedAmbPool->pw_refSat[u_sysIndex], u_isys, c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, " ref: %s           ", c_sat);
    }
    else
    {
      p_satInfo += snprintf(p_satInfo, u_size, " ref:               ");
    }

    if (pz_fixedAmbPool->pw_refSat[u_sysIndex + 1] != 0)
    {
      svid_SatString(pz_fixedAmbPool->pw_refSat[u_sysIndex + 1], u_isys, c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, "                    | %s", c_sat);
    }
    else
    {
      p_satInfo += snprintf(p_satInfo, u_size, "                         ");
    }

    if (pz_fixedAmbPool->pw_refSat[u_sysIndex + 2] != 0)
    {
      svid_SatString(pz_fixedAmbPool->pw_refSat[u_sysIndex + 2], u_isys, c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, "                               | %s", c_sat);
    }
    else
    {
      p_satInfo += snprintf(p_satInfo, u_size, "                                    ");
    }
    LOGI(TAG_HPP, "%s\n", satInfo);

    for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
    {
      pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
      if (NULL == pz_satMeas)
      {
        continue;
      }

      u_constellation = gnss_getConstellationEnumValueInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      if (u_isys != u_constellation)
      {
        continue;
      }

      // find satellite
      for (u_i = 0; u_i < u_nb; u_i++)
      {
        if ((pz_ambFix[u_i].u_satIndex - 1) == u_satIndex)
        {
          break;
        }
      }

      if (u_i == u_nb)
      {
        continue;
      }

      memset(satInfo, 0, BUFF_SIZE * sizeof(char));
      p_satInfo = &satInfo[0];

      satidx_SatString(u_satIndex, c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, "      %s", c_sat);

      for (u_i = 0; u_i < u_nb; u_i++)
      {
        if ((pz_ambFix[u_i].u_satIndex - 1) == u_satIndex && pz_ambFix[u_i].u_signal == 0)
        {
          break;
        }
      }

      if (u_i < u_nb)
      {
        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = convertFreqAmb2FilterType(C_GNSS_FREQ_TYPE_L1);
        pz_satPool = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
        if (NULL == pz_satPool)
        {
          continue;
        }
        p_satInfo += snprintf(p_satInfo, u_size, " %7.1f %7.1f %7.1f     ", tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime),
          pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, pz_satMeas->pz_signalMeas[0]->f_cn0);
      }
      else
      {
        p_satInfo += snprintf(p_satInfo, u_size, "                             ");
      }

      // L2
      for (u_i = 0; u_i < u_nb; u_i++)
      {
        if ((pz_ambFix[u_i].u_satIndex - 1) == u_satIndex && pz_ambFix[u_i].u_signal == 1)
        {
          break;
        }
      }

      if (u_i < u_nb)
      {
        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = convertFreqAmb2FilterType(C_GNSS_FREQ_TYPE_L2);
        pz_satPool = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
        if (NULL == pz_satPool)
        {
          continue;
        }
        p_satInfo += snprintf(p_satInfo, u_size, "        %7.1f %7.1f %7.1f      ", tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime),
          pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, pz_satMeas->pz_signalMeas[1]->f_cn0);
      }
      else
      {
        p_satInfo += snprintf(p_satInfo, u_size, "                                     ");
      }

      // L5
      for (u_i = 0; u_i < u_nb; u_i++)
      {
        if ((pz_ambFix[u_i].u_satIndex - 1) == u_satIndex && pz_ambFix[u_i].u_signal == 2)
        {
          break;
        }
      }

      if (u_i < u_nb)
      {
        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = convertFreqAmb2FilterType(C_GNSS_FREQ_TYPE_L5);
        pz_satPool = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
        if (NULL == pz_satPool)
        {
          continue;
        }
        p_satInfo += snprintf(p_satInfo, u_size, "       %7.1f %7.1f %7.1f     ", tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime),
          pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, pz_satMeas->pz_signalMeas[2]->f_cn0);
      }
      else
      {
        p_satInfo += snprintf(p_satInfo, u_size, "                                   ");
      }

      LOGI(TAG_HPP, "%s\n", satInfo);
    }
  }

  LOGI(TAG_HPP, " ===|====|==============L1==============|====|===============L2=============|====|==============L5=============|\n");
  return;
}

/**
 * @brief get rover satellite amb list
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     pu_toBeFixSatNum the number of satellite to be fixed
 * @return 0: success, other: fail
 */
static uint8_t rtk_ambFixGetSatList(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_AmbFix_t* pz_ambFix, uint8_t pu_toBeFixSatNum[C_GNSS_MAX])
{
  uint8_t u_nb = 0;
  uint8_t u_nb_f = 0;
  uint8_t u_isys = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_constellation = 0;
  uint8_t u_PAR_flag = pz_fixedAmbPool->u_ambfix_mode;
  char c_sat[4] = "";
  uint8_t u_satIndex = 0;
  uint8_t u_satIndexPre = 0;
  uint8_t u_refsat_svid = 0;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  LogLevelEnum z_logLevel = LOG_LEVEL_NONE;
  uint8_t pu_obsFixNum[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const GpsTime_t* curtime = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const rtk_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_MeasStatusFlag_t* pz_measUpdateFlag = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef = NULL;
  float f_sample = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.f_sample;
  double d_timeLock = 0.0;
  
  for (u_isys = 0; u_isys < C_GNSS_MAX; ++u_isys)
  {
    pu_toBeFixSatNum[u_isys] = 0;
    for (u_signalIndex = 0; u_signalIndex < C_GNSS_FREQ_TYPE_MAX; ++u_signalIndex)
    {
      pu_obsFixNum[u_isys][u_signalIndex] = 0;
    }
  }
  curtime = &pz_rtkAmbFixInputInfo->pz_satSignalPool->z_tor;

  /* deal time lock */
  if (f_sample < 1e-3)
  {
    f_sample = 1.0f;
  }
  d_timeLock = (double)f_sample * GNSS_MIN_LOCK;

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }

    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      u_nb_f = 0;

      u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex];

      if (0 == u_refsat_svid)
      {
        continue;
      }
      z_algoFreq = gnss_freqType2Algo((gnss_FreqType)u_signalIndex);
      if (0 == ((pz_rtkAmbFixInputInfo->z_fixUsedFreq) & z_algoFreq))
      {
        continue;
      }
#if 0
      svid_SatString(u_refsat_svid, u_constellation, c_sat);
      LOGI(TAG_HPP, "L%d ddmat ref sat=%s \n", (u_signalIndex + 1) > 2 ? 5 : (u_signalIndex + 1), c_sat);
#endif

      z_filterType = convertFreqAmb2FilterType(u_signalIndex);

      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
        pz_corrStat = pz_rtkAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_satIndex];
        if (NULL == pz_satMeas || NULL == pz_corrStat)
        {
          continue;
        }
        u_constellation = gnss_getConstellationEnumValueInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
        if (u_constellation != u_isys)
        {
          continue;
        }
        if (pz_satMeas->u_svid == u_refsat_svid || pz_satMeas->u_rejectLabel[u_signalIndex] == 1)
        {
          continue;
        }

        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        pz_measUpdateFlag = pz_corrStat->z_measUpdateFlag[u_signalIndex];
        if (NULL == pz_sigMeas || NULL == pz_measUpdateFlag)
        {
          continue;
        }

        if ((pz_sigMeas->u_LLI & 2) || !pz_measUpdateFlag->b_cpValid ||
          pz_satMeas->z_satPosVelClk.f_elevation < 20.0 * DEG2RAD)
        {
          continue;
        }

        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = z_filterType;
        pz_satPool = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
        if (NULL == pz_satPool)
        {
          continue;
        }

        if (fabs(tm_GpsTimeDiff(curtime, &pz_satPool->z_endTime)) > 1e-4) /* unhealth,current epoch */
        {
          continue;
        }

        if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < d_timeLock)
        {
          continue;
        }

        if (u_PAR_flag == GNSS_PART_LAMBDA_MP && NULL != pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag)
        {
          if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][u_signalIndex] & MULTIPATH_FLAG_PR)
            != MULTIPATH_FLAG_PR_NORMAL)
          {
            continue;
          }
          if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][u_signalIndex] & MULTIPATH_FLAG_CP_HIGH) || 
            (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][u_signalIndex] & MULTIPATH_FLAG_CP_LOW))
          {
            continue;
          }
        }

        if (pz_sigMeas->f_cn0 < pz_fixedAmbPool->f_cn0Thres || pz_satMeas->z_satPosVelClk.f_elevation < pz_fixedAmbPool->f_eleThres)
        {
#if 0
          /* print second lambda search info */
          satidx_SatString(u_satIndex, c_sat);
          LOGI(TAG_HPP, "L%d %s cn0=%4.1f ele=%5.1f lock=%6.1f fix=1\n", (u_signalIndex + 1) > 2 ? 5 : (u_signalIndex + 1), c_sat, pz_sigMeas->f_cn0,
            pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime));
#endif
          continue;
        }

        w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, pz_satMeas->u_constellation);
        pz_satPoolRef = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);

        if (u_nb >= MAX_GNSS_TRK_MEAS_NUMBER)
        {
          break;
        }

        pz_ambFix[u_nb].u_satIndex = u_satIndex + 1;
        pz_ambFix[u_nb].u_signal = u_signalIndex;
        pz_ambFix[u_nb].u_amb_index = pz_satPool->w_index;
        pz_ambFix[u_nb++].u_ref_amb_index = pz_satPoolRef->w_index;

        u_nb_f++;
        u_satIndexPre = u_satIndex;//update u_satIndexPre
        ++pu_obsFixNum[u_isys][u_signalIndex];
#if 0
        /* print log */
        satidx_SatString(u_satIndex, c_sat);
        LOGI(TAG_HPP, "L%d %s cn0=%4.1f ele=%5.1f lock=%6.1f fix=2\n", (u_signalIndex + 1) > 2 ? 5 : (u_signalIndex + 1), c_sat, pz_sigMeas->f_cn0,
          pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG, tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime));
#endif
      }

      if (0 == u_nb_f)
      {
        /* clean ref sat */
        pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 0;
        pu_obsFixNum[u_isys][u_signalIndex] = 0;
      }
      else if (1 == u_nb_f && pz_fixedAmbPool->u_ambfix_mode == GNSS_PART_LAMBDA_QOS && pz_fixedAmbPool->u_nb >= 25)
      {
        u_nb--;
        --pu_obsFixNum[u_isys][u_signalIndex];
        pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 0;
        /* print log */
        satidx_SatString(u_satIndexPre, c_sat);
        LOGI(TAG_HPP, "remove in second lambda search for too less ddmat:L%d %s\n", (u_signalIndex + 1) > 2 ? 5 : (u_signalIndex + 1), c_sat);
      }
    }
  }

  for (u_isys = 0; u_isys < C_GNSS_MAX; ++u_isys)
  {
    for (u_signalIndex = 0; u_signalIndex < C_GNSS_FREQ_TYPE_MAX; ++u_signalIndex)
    {
      if (pu_obsFixNum[u_isys][u_signalIndex] > pu_toBeFixSatNum[u_isys])
      {
        pu_toBeFixSatNum[u_isys] = pu_obsFixNum[u_isys][u_signalIndex];
      }
    }
  }
  // print zero combine info 
  z_logLevel = log_GetLogLevel();
  if (z_logLevel >= LOG_LEVEL_I)
  {
    rtk_zeroCombinePrintSat(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, pz_ambFix, u_nb);
  }

  return u_nb;
}

/**
 * @brief get reference satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @return 0: success, other: fail
 */
static RTKAmbFixFailInfoType rtk_ambFixRefSatSelect(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_status;
  uint8_t u_refsat;
  uint8_t u_i = 0;
  uint8_t u_isys = 0;
  uint8_t u_icoll = 0;
  uint8_t u_ifreq = 0;
  uint8_t u_refsatcount = 0;
  uint8_t u_PAR_flag = pz_fixedAmbPool->u_ambfix_mode;
  uint8_t u_groupSTDcount = 0;
  uint8_t u_groupSelect[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ] = { 0 };
  int16_t u_groupSelectIndex[C_GNSS_MAX * MAX_GNSS_SIGNAL_FREQ] = { 0 };
  const uint8_t u_groupSTDLength = C_GNSS_MAX * MAX_GNSS_SIGNAL_FREQ;
  double d_STD_BDS2[MAX_GNSS_SIGNAL_FREQ] = { 0.0 };
  double d_STD_BDS3[MAX_GNSS_SIGNAL_FREQ] = { 0.0 };
  double d_groupSTD[C_GNSS_MAX * MAX_GNSS_SIGNAL_FREQ] = { 0.0 };
  double d_groupSTD_sysMAX[C_GNSS_MAX] = { 0.0 };
  gnss_FreqType u_signalIndex = 0;
  RTKAmbFixFailInfoType u_ambStatus = AMB_AR_FAIL_UnknowError;
  rtk_EpochFilterObs_t* pz_filterObs = NULL;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  /* threshold define begin */
  const uint8_t u_nb_threshold = 2;
  const uint8_t u_numGroupSelect = 6;
  float f_minEle = (float)(20.0 * DEG2RAD);
  /* threshold define end */
  if (0x1 == (pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->t_closeSkyCount & 0x1))
  {
    f_minEle = (float)(30.0 * DEG2RAD);
  }

  if (GNSS_PART_LAMBDA_QOS == pz_fixedAmbPool->u_ambfix_mode)
  {
    u_status = rtk_ambCalSatQos(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);

    if (0 == u_status)
    {
      u_ambStatus = AMB_AR_FAIL_UnknowError;
      return u_ambStatus;
    }

    if (pz_fixedAmbPool->f_cn0Thres < 30.0f)
    {
      pz_fixedAmbPool->f_cn0Thres = 30.0f;
    }

    if (pz_fixedAmbPool->f_eleThres < f_minEle)
    {
      pz_fixedAmbPool->f_eleThres = f_minEle;
    }
  }
  else if (GNSS_PART_LAMBDA_GROUP == pz_fixedAmbPool->u_ambfix_mode)
  {
    pz_filterObs = pz_rtkAmbFixInputInfo->pz_FilterObs;
    if (NULL == pz_filterObs)
    {
      return AMB_AR_FAIL_UnknowError;
    }
    for (u_isys = C_GNSS_NONE; u_isys < C_GNSS_MAX; u_isys++)
    {
      for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
      {
        d_groupSTD[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 1e10;
        z_algoFreq = gnss_freqType2Algo((gnss_FreqType)u_signalIndex);
        if (0 == ((pz_rtkAmbFixInputInfo->z_fixUsedFreq) & z_algoFreq))
        {
          continue;
        }
        if ((C_GNSS_GLO == u_isys) || (C_GNSS_QZS == u_isys) || (C_GNSS_NONE == u_isys))
        {
          continue;
        }
        if (pz_filterObs->w_groupNUM[u_isys][u_signalIndex] >= u_nb_threshold)
        {
          d_groupSTD[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = pz_filterObs->d_groupSTD[u_isys][u_signalIndex];
          u_groupSTDcount++;
          /*if (C_GNSS_BDS3 == u_isys)
          {
            d_STD_BDS2[u_signalIndex] = pz_filterObs->d_groupSTD[u_isys][u_signalIndex];
          }
          else if (C_GNSS_MAX == u_isys)
          {
            d_STD_BDS3[u_signalIndex] = pz_filterObs->d_groupSTD[u_isys][u_signalIndex];
          }*/
          if (pz_filterObs->d_groupSTD[u_isys][u_signalIndex] > d_groupSTD_sysMAX[u_isys])
          {
            d_groupSTD_sysMAX[u_isys] = pz_filterObs->d_groupSTD[u_isys][u_signalIndex];
          }
        }
      }
    }
    for (u_isys = C_GNSS_NONE; u_isys < C_GNSS_MAX; u_isys++)
    {
      LOGD(TAG_HPP, "GNSS_PART_LAMBDA_GROUP d_groupSTD_sysMAX[%d] = %lf\n", u_isys, d_groupSTD_sysMAX[u_isys]);
    }
    if (u_groupSTDcount <= u_numGroupSelect)
    {
      return AMB_AR_FAIL_LessNS;
    }
    /* strategy: BDS2 or BDS3 */
    /*for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (d_STD_BDS2[u_signalIndex] > FABS_ZEROS && d_STD_BDS3[u_signalIndex] > FABS_ZEROS &&
        d_STD_BDS2[u_signalIndex] < 1e6 && d_STD_BDS3[u_signalIndex] < 1e6)
      {
        if (d_STD_BDS2[u_signalIndex] > d_STD_BDS3[u_signalIndex])
        {
          d_groupSTD[C_GNSS_BDS3 * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 1e10;
        }
        else
        {
          d_groupSTD[C_GNSS_MAX * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 1e10;
        }
      }
    }*/
    /* strategy: BDS2 or BDS3 */
    if ((d_groupSTD_sysMAX[C_GNSS_BDS2] > FABS_ZEROS) && (d_groupSTD_sysMAX[C_GNSS_BDS3] > FABS_ZEROS))
    {
      if (d_groupSTD_sysMAX[C_GNSS_BDS2] > d_groupSTD_sysMAX[C_GNSS_BDS3])
      {
        for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
        {
          if (d_groupSTD[C_GNSS_BDS2 * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] > FABS_ZEROS)
          {
            d_groupSTD[C_GNSS_BDS2 * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 1e10;
          }
        }
      }
      else
      {
        for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
        {
          if (d_groupSTD[C_GNSS_BDS3 * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] > FABS_ZEROS)
          {
            d_groupSTD[C_GNSS_BDS3 * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = 1e10;
          }
        }
      }
    }
    if (0 == cmn_find_min_indices(d_groupSTD, u_groupSTDLength, u_groupSelectIndex, u_numGroupSelect))
    {
      for (u_i = 0; u_i < u_numGroupSelect; u_i++)
      {
        if (u_groupSelectIndex[u_i] >= 0 && u_groupSelectIndex[u_i] < C_GNSS_MAX * MAX_GNSS_SIGNAL_FREQ)
        {
          u_icoll = (uint8_t)(u_groupSelectIndex[u_i] / MAX_GNSS_SIGNAL_FREQ);
          u_ifreq = (uint8_t)(u_groupSelectIndex[u_i] % MAX_GNSS_SIGNAL_FREQ);
          u_groupSelect[u_icoll][u_ifreq] = TRUE;
        }
      }
    }
    else
    {
      return AMB_AR_FAIL_LessNS;
    }
  }
  else
  {
    pz_fixedAmbPool->f_cn0Thres = 0.0f;
    pz_fixedAmbPool->f_eleThres = 0.0f;
  }

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      //u_refsat = 0;
      z_algoFreq = gnss_freqType2Algo((gnss_FreqType)u_signalIndex);
      if (0 == ((pz_rtkAmbFixInputInfo->z_fixUsedFreq) & z_algoFreq))
      {
        continue;
      }

      if (GNSS_PART_LAMBDA_GROUP == pz_fixedAmbPool->u_ambfix_mode && FALSE == u_groupSelect[u_isys][u_signalIndex])
      {
        continue;
      }

      // select ref sat in single frequence
      u_refsat = rtk_ambFixRefSatSelectSf(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, u_isys, u_signalIndex, u_signalIndex, TRUE);

      if (u_refsat <= 0)
      {
        continue;
      }
      else
      {
        pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = u_refsat;
        u_refsatcount++;
      }
    }
  }

  if (u_refsatcount > 0)
  {
    u_ambStatus = AMB_AR_SUCCESS;
  }

  return u_ambStatus;
}

/**
 * @brief get X and Q of zero combine ambiguity
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     u_nb
 * @param[out]    pd_X
 * @param[out]    pd_Q
 * @return 0: OK, other: fail
 */
static uint8_t rtk_ambFixGetXQ(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_AmbFix_t* pz_ambFix, uint8_t u_nb, double* pd_X, double* pd_Q)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_sat_idx = 0;
  uint16_t uw_i1 = 0;
  uint16_t uw_ri1 = 0;
  uint16_t uw_j1 = 0;
  uint16_t uw_rj1 = 0;

  for (u_i = 0; u_i < u_nb; u_i++)
  {
    if (0 == pz_ambFix[u_i].u_satIndex)
    {
      u_status = 1;
      return u_status;
    }
    //u_sat_idx = pz_ambFix[u_i].u_satIndex - 1;

    uw_i1 = pz_ambFix[u_i].u_amb_index;
    uw_ri1 = pz_ambFix[u_i].u_ref_amb_index;
    pd_X[u_i] = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[uw_i1] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[uw_ri1];

    for (u_j = 0; u_j < u_nb; u_j++)
    {
      if (0 == pz_ambFix[u_j].u_satIndex)
      {
        u_status = 1;
        return u_status;
      }
      //u_sat_idx = pz_ambFix[u_j].u_satIndex - 1;

      uw_j1 = pz_ambFix[u_j].u_amb_index;
      uw_rj1 = pz_ambFix[u_j].u_ref_amb_index;

      pd_Q[u_j + u_i * u_nb] = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i1, uw_j1)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri1, uw_rj1)];

      pd_Q[u_j + u_i * u_nb] -= pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri1, uw_j1)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i1, uw_rj1)];

      pd_Q[u_i + u_j * u_nb] = pd_Q[u_j + u_i * u_nb];
    }
  }

  return u_status;
}

/**
 * @brief RTK zero combine ambiguity result save
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static void rtk_ambFixSaveResult(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  uint8_t u_nb, gnss_AmbFix_t* pz_ambFix, const double* pd_amb_int)
{
  uint8_t u_i = 0;
  uint8_t u_svid = 0;
  uint8_t u_constellation = 0;
  uint8_t u_refsvid = 0;
  uint8_t u_refidx = 0;
  uint8_t u_sat_idx = 0;
  uint8_t z_freqType = 0;
  uint8_t u_sys = 0;
  uint8_t u_fixedTotalSatCount = 0;
  uint8_t pu_fixedsat[ALL_GNSS_SYS_SV_NUMBER] = { 0 };
  gnss_fixedSignalAmb_t* pz_fix_amb = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  pz_fixedAmbPool->w_continuefixedEpoch++;
  for (u_i = 0; u_i < u_nb; u_i++)
  {
    if (0 == pz_ambFix[u_i].u_satIndex)
    {
      continue;
    }
    u_sat_idx = pz_ambFix[u_i].u_satIndex - 1;

    pz_fix_amb = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx];
    if (NULL == pz_fix_amb)
    {
      continue;
    }

    z_freqType = pz_ambFix[u_i].u_signal;
    pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    if (NULL == pz_satMeas || NULL == pz_satMeas->pz_signalMeas[z_freqType])
    {
      continue;
    }

    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_constellation = pz_satMeas->u_constellation;

    u_refsvid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType];
    u_refidx = gnss_cvt_Svid2SvIndex(u_refsvid, u_sys);
    pz_fixedAmbPool->u_fixStatus |= GNSS_L1_AMB_FIXED;

    if (NULL != pd_amb_int)
    {
      pz_fix_amb->d_fixedValueRTK[z_freqType] = pd_amb_int[u_i];
    }
    pz_fix_amb->u_constellation = u_constellation;
    pz_fix_amb->u_svid = u_svid;
    pz_fix_amb->u_continue_fix[z_freqType]++;
    pz_fix_amb->u_freqType[z_freqType] = pz_satMeas->pz_signalMeas[z_freqType]->u_signal;
    pz_fix_amb->u_fix[z_freqType] = 2;

    pz_fixedAmbPool->u_fixedns[u_sys]++;
    pu_fixedsat[u_sat_idx] = 1;
    if (NULL != pz_fixedAmbPool->pz_fixedAmbSet[u_refidx] &&
      2 != pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_fix[z_freqType])
    {
      /* reference satellite */
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->d_fixedValueRTK[z_freqType] = 0.0;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_continue_fix[z_freqType]++;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_freqType[z_freqType] = pz_satMeas->pz_signalMeas[z_freqType]->u_signal;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_fix[z_freqType] = 2;
    }
  }
  pz_fixedAmbPool->u_fixedns[C_GNSS_BDS3] = pz_fixedAmbPool->u_fixedns[C_GNSS_BDS2] + pz_fixedAmbPool->u_fixedns[C_GNSS_BDS3];
  pz_fixedAmbPool->u_fixedns[C_GNSS_BDS2] = pz_fixedAmbPool->u_fixedns[C_GNSS_BDS3];
  /* cal joined in fix sat number*/
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (1 == pu_fixedsat[u_i])
    {
      u_fixedTotalSatCount++;
    }
  }
  pz_fixedAmbPool->u_fixedTotalSatCount = u_fixedTotalSatCount;

  return;
}

/**
 * @brief RTK zerocombine ambiguity confirm
 * @param[in]  u_nb is number of bias
 * @param[in]  f_ratio is lambda ratio
 * @param[in]  d_adop is lambda adop
 * @return     TRUE success; else fail
 */
static uint8_t rtk_ambConfirm(uint8_t u_nb, float f_ratio, double d_adop)
{
  uint8_t u_result = TRUE;

  if (u_nb <= 8 && d_adop > 10.0)
  {
    u_result = FALSE;
  }
  else if (u_nb < 10 && d_adop >= 15.0)
  {
    u_result = FALSE;
  }
  else if (u_nb < 6)
  {
    u_result = FALSE;
  }

  return u_result;
}

/**
 * @brief          deal second best amb and best amb
 * @param[in]      pz_rtkAmbFixInputInfo is rtk ambiguity information
 * @param[in]      pz_ambFix is fixed ambiguity list
 * @param[in]      u_nb is num of amb
 * @return         TRUE fot there is gap between fix sols, FALSE for not
 */
void rtk_dealGapBetweenAmb(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_AmbFix_t* pz_ambFix,
  double* pd_amb_int, uint8_t u_nb)
{
  uint8_t u_i = 0;
  uint8_t u_count = 0;
  uint8_t u_amb_sat[5] = { 0 };
  uint8_t u_amb_freq[5] = { 0 };
  char c_sat[4] = "";

  for (u_i = 0; u_i < u_nb; u_i++)
  {
    if (round(pd_amb_int[u_i]) != round(pd_amb_int[u_i + u_nb]) && u_count < 5)
    {
      u_amb_sat[u_count] = pz_ambFix[u_i].u_satIndex;
      u_amb_freq[u_count] = pz_ambFix[u_i].u_signal;
      LOGD(TAG_HPP, "amb not equal to round: %d L%d\n", u_amb_sat[u_count], u_amb_freq[u_count]);
      u_count++;
    }
#if 1
    satidx_SatString(pz_ambFix[u_i].u_satIndex - 1, c_sat);
    LOGD(TAG_HPP, "amb info: %s %d %8.3f %8.3f\n", c_sat, pz_ambFix[u_i].u_signal, pd_amb_int[u_i], pd_amb_int[u_i + u_nb]);
#endif
  }

  if (u_count < 5 && u_count < u_nb * 0.35)
  {
    for (u_i = 0; u_i < u_count; u_i++)
    {
      pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_amb_sat[u_i] - 1]->u_rejectLabel[u_amb_freq[u_i]] = 1;
      satidx_SatString(u_amb_sat[u_i] - 1, c_sat);
      LOGI(TAG_HPP, "amb not join in second lambda search: %s %d\n", c_sat, u_amb_freq[u_i]);
    }
  }
  else
  {
    LOGI(TAG_HPP, "number of fixed amb not equal to round >=5 or >=35%\n");
  }

  return;
}

/**
 * @brief          get whether second best fix sol is close to best fix sol or not
 * @param[in]      pd_amb_int is fixed ambiguity resolution
 * @param[in]      u_nb is num of amb
 * @return         TRUE fot there is gap between fix sols, 0 for not,1 for small, 2 for large
 */
uint8_t rtk_getGapBetweenFixSols(double* pd_amb_int, uint8_t u_nb)
{
  uint8_t u_flag = 0;
  uint8_t u_i;
  uint8_t u_gapLevel1 = 0;
  uint8_t u_gapLevel2 = 0;
  uint8_t u_gapLevel3 = 0;

  for (u_i = 0; u_i < u_nb; u_i++)
  {
    if (fabs(pd_amb_int[u_i] - pd_amb_int[u_i + u_nb]) >= 5) u_gapLevel1++;
    if (fabs(pd_amb_int[u_i] - pd_amb_int[u_i + u_nb]) >= 10) u_gapLevel2++;
    if (fabs(pd_amb_int[u_i] - pd_amb_int[u_i + u_nb]) >= 15) u_gapLevel3++;
  }

  if (u_gapLevel1 > 0.5 * u_nb && u_gapLevel2 > 0.2 * u_nb)
  {
    u_flag = 2;
  }
  else if (u_gapLevel1 > 0.5 * u_nb || (u_gapLevel2 > 0.3 * u_nb && u_gapLevel2 > 1))
  {
    u_flag = 1;
  }
  else
  {
    u_flag = 0;
  }

  LOGI(TAG_HPP, " AMB_GAP: Level1=%d, Level2=%d, nb=%d, gap=%d\n", u_gapLevel1, u_gapLevel2, u_nb, u_flag);
  return u_flag;
}
/**
 * @brief RTK zero combine ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static RTKAmbFixFailInfoType rtk_ambFixAR(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_candidate = 2;
  uint8_t u_nb;
  float f_boot_rate = 0.0;
  float f_ratio = 0.0;
  double d_adop = 0.0;
  double* pd_X = NULL;
  double* pd_Q = NULL;
  double* pd_amb_int = NULL;
  double* pd_sigma = NULL;
  gnss_AmbFix_t* pz_ambFix = NULL;
  RTKAmbFixFailInfoType u_status;
  /* threshold define begin */
  uint8_t u_nb_threshold = 4;
  uint8_t u_nb_threshold_PAR_MP = 6;
  uint8_t u_iSys = 0;
  uint8_t u_beFixMaxSatNum = 0;
  uint8_t u_beFixTotalSatNum = 0;
  uint8_t pu_toBeFixSatNum[C_GNSS_MAX] = { 0 };
  /* threshold define end */

  if (pz_fixedAmbPool->u_ambfix_mode == GNSS_PART_LAMBDA_MP)
  {
    u_nb_threshold = u_nb_threshold_PAR_MP;
  }

  pz_ambFix = (gnss_AmbFix_t*)OS_MALLOC_FAST(MAX_GNSS_TRK_MEAS_NUMBER * sizeof(gnss_AmbFix_t));
  if (NULL == pz_ambFix)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }

  // select not-ref satellite
  u_nb = rtk_ambFixGetSatList(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, pz_ambFix, pu_toBeFixSatNum);
  for (u_iSys = 0; u_iSys < C_GNSS_MAX; ++u_iSys)
  {
    u_beFixTotalSatNum += pu_toBeFixSatNum[u_iSys];
    if (pu_toBeFixSatNum[u_iSys] > u_beFixMaxSatNum)
    {
      u_beFixMaxSatNum = pu_toBeFixSatNum[u_iSys];
    }
  }

  // update nb
  pz_fixedAmbPool->u_nb = u_nb;

  if (u_nb < u_nb_threshold)
  {
    u_status = AMB_AR_FAIL_LessNS;
  }

  while (u_nb >= u_nb_threshold)
  {
    pd_X = (double*)OS_MALLOC_FAST(u_nb * sizeof(double));
    pd_Q = (double*)OS_MALLOC_FAST(u_nb * u_nb * sizeof(double));
    pd_amb_int = (double*)OS_MALLOC_FAST(u_nb * u_candidate * sizeof(double));
    pd_sigma = (double*)OS_MALLOC_FAST(u_candidate * sizeof(double));
    if (NULL == pd_X || NULL == pd_Q || NULL == pd_amb_int || NULL == pd_sigma)
    {
      u_status = AMB_AR_FAIL_MemError;
      break;
    }

    if (rtk_ambFixGetXQ(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, pz_ambFix, u_nb, pd_X, pd_Q))
    {
      u_status = AMB_AR_FAIL_DataError;
      break;
    }

    /* search ambiguity by lambda */
    if (rtk_lambda(u_nb, u_candidate, pd_X, pd_Q, pd_amb_int, pd_sigma))
    {
      u_status = AMB_AR_FAIL_LambdaError;
      break;
    }
    else
    {
      f_ratio = (float)(pd_sigma[0] > 0 ? (float)(pd_sigma[1] / pd_sigma[0]) : 0.0);
      f_ratio = f_ratio > 999.99f ? 999.99f : f_ratio;

      // update ratio
      pz_fixedAmbPool->f_ratio = f_ratio;
      pz_fixedAmbPool->f_lambda_sigma[0] = pd_sigma[0];
      pz_fixedAmbPool->f_lambda_sigma[1] = pd_sigma[1];

      if (f_ratio >= 2.5)
      {
        d_adop = rtk_adop_get();
        pz_fixedAmbPool->f_adop = (float)d_adop;
        pz_fixedAmbPool->u_gap = 0;
        /*update in future*/
        //if (u_candidate >= 2)
        {
          pz_fixedAmbPool->u_gap = rtk_getGapBetweenFixSols(pd_amb_int, u_nb);
        }

        LOGI(TAG_HPP, "zerocombine amb validation ok (nb=%d ratio=%.2f s=%.2f/%.2f adop=%.2f)\n", u_nb, f_ratio, pd_sigma[1], pd_sigma[0], d_adop);

        if (rtk_ambConfirm(u_nb, f_ratio, d_adop))
        {
          if (FALSE == pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->b_staticFlag
            || (TRUE == pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->b_staticFlag && (u_beFixMaxSatNum > 4 || u_beFixTotalSatNum > 10)))
          {
            LOGI(TAG_HPP, "zerocombine amb confirm success!\n");
            rtk_ambFixSaveResult(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, u_nb, pz_ambFix, pd_amb_int);
            u_status = AMB_AR_SUCCESS;
          }
          else
          {
            LOGI(TAG_HPP, "zerocombine amb confirm fail!\n");
            u_status = AMB_AR_FAIL_ParFail;
          }
          break;
        }
        else
        {
          LOGI(TAG_HPP, "zerocombine amb confirm fail!\n");
          u_status = AMB_AR_FAIL_ParFail;
          break;
        }
      }
      else
      {
        if (pz_fixedAmbPool->u_ambfix_mode == GNSS_PART_LAMBDA_FULL)
        {
          rtk_dealGapBetweenAmb(pz_rtkAmbFixInputInfo, pz_ambFix, pd_amb_int, u_nb);
        }
        LOGI(TAG_HPP, "zerocombine amb validation fail (nb=%d ratio=%.2f s=%.2f/%.2f)\n", u_nb, f_ratio, pd_sigma[1], pd_sigma[0]);
        u_status = AMB_AR_FAIL_ParFail;
      }
    }

    break;
  }

  if (pd_X) OS_FREE(pd_X);
  if (pd_Q) OS_FREE(pd_Q);
  if (pd_amb_int) OS_FREE(pd_amb_int);
  if (pd_sigma) OS_FREE(pd_sigma);
  if (pz_ambFix) OS_FREE(pz_ambFix);

  return u_status;
}

/**
 * @brief RTK zero combine ambiguity update
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static void rtk_ambZeroCombineUpdate(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_refsat_svid = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_sys = 0;
  uint8_t u_svid = 0;
  uint8_t u_isys;
  uint8_t u_constellation = 0;
  char c_sat[4] = "";
  double d_amb = 0.0;
  uint16_t w_i = 0;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double d_pdop = 0.0;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_HtH[16] = { 0.0 };
  double pd_HtHinv[16] = { 0.0 };
  double* pd_X = NULL;
  double* pd_deltaX = NULL;
  uint8_t u_signalIndex;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  SeqKalmanVar_t z_seqVar = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1_ref = NULL;
  const gnss_SignalCorr_t* pz_signal1 = NULL;
  const gnss_SignalCorr_t* pz_signal1_ref = NULL;
  matrix_t  z_HtH;
  matrix_t  z_HtHinv;
  const double* pd_satPosClk = NULL;

  if (NULL == pz_fixedAmbPool)
  {
    return;
  }
  if (hpp_seq_init(&z_seqVar, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax))
  {
    return;
  }

  pd_X = (double*)OS_MALLOC_FAST(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax * sizeof(double));
  pd_deltaX = (double*)OS_MALLOC_FAST(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax * sizeof(double));
  if (any_Ptrs_Null(2, pd_X, pd_deltaX))
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

  // update x and q
  memcpy(pz_fixedAmbPool->pd_x_fix, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  memcpy(pz_fixedAmbPool->pd_q_fix, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q, NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
  memcpy(pd_X, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  memset(pd_deltaX, 0, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax * sizeof(double));

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      // get refsat svid
      u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex];
      if (u_refsat_svid <= 0)
      {
        continue;
      }

#ifdef HPP_LOG_FULL
      svid_SatString(u_refsat_svid, u_constellation, c_sat);
      LOGI(TAG_HPP, "L%d amb ref sat=%s \n", (u_signalIndex + 1) > 2 ? 5 : (u_signalIndex + 1), c_sat);
#endif // HPP_LOG_FULL

      z_filterType = convertFreqAmb2FilterType(u_signalIndex);

      for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
      {
        if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx])
        {
          continue;
        }

        u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
        u_constellation= gnss_getConstellationEnumValueInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_sys);
        if (u_isys != u_constellation)
        {
          continue;
        }
        if (u_refsat_svid == u_svid)
        {
          continue;
        }

        if (pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_fix[u_signalIndex] != 2)
        {
          continue;
        }

        /* get status from EKF */
        w_x_id[0] = 0;
        w_x_id[1] = u_sat_idx;
        w_x_id[2] = z_filterType;
        pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
        w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
        pz_satPool1_ref = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
        if (NULL == pz_satPool1 || NULL == pz_satPool1_ref)
        {
          continue;
        }

        d_amb = (pd_X[pz_satPool1->w_index] - pd_X[pz_satPool1_ref->w_index]);

        hpp_seq_ClearH(&z_seqVar);

        /*PDOP*/
        pd_satPosClk = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx]->z_satPosVelClk.d_satPosClk;
        if (gnss_Dot(pd_satPosClk, pd_satPosClk, 3) > FABS_ZEROS)
        {
          gnss_satRot(pd_X, pd_satPosClk, pd_satPosRot);
          gnss_unitVector(pd_X, pd_satPosRot, pd_unitVector);
          pd_HtH[0] += pd_unitVector[0] * pd_unitVector[0];
          pd_HtH[1 * 4 + 1] += pd_unitVector[1] * pd_unitVector[1];
          pd_HtH[2 * 4 + 2] += pd_unitVector[2] * pd_unitVector[2];
          pd_HtH[3 * 4 + 3] += 1 * 1;
          pd_HtH[1] = pd_HtH[4] = pd_HtH[4] + pd_unitVector[0] * pd_unitVector[1];
          pd_HtH[2] = pd_HtH[8] = pd_HtH[8] + pd_unitVector[0] * pd_unitVector[2];
          pd_HtH[6] = pd_HtH[9] = pd_HtH[9] + pd_unitVector[1] * pd_unitVector[2];
          pd_HtH[3] = pd_HtH[12] = pd_HtH[12] + pd_unitVector[0] * 1;
          pd_HtH[7] = pd_HtH[13] = pd_HtH[13] + pd_unitVector[1] * 1;
          pd_HtH[11] = pd_HtH[14] = pd_HtH[14] + pd_unitVector[2] * 1;
        }

        /* OMC */
        d_omc = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValueRTK[u_signalIndex] - d_amb;
        d_curR = GNSS_VAR_HOLDAMB;

        /* H */
        hpp_seq_AddH(pz_satPool1->w_index, 1.0, &z_seqVar);
        hpp_seq_AddH(pz_satPool1_ref->w_index, -1.0, &z_seqVar);

        hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
        hpp_seq_PredictStep(pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar);
        hpp_seq_measUpdate(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pq_paraValid, pz_fixedAmbPool->pd_x_fix, pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar, 0.0);

#ifdef HPP_LOG_FULL
        /* print log */
        satidx_SatString(u_sat_idx, c_sat);
        LOGI(TAG_HPP, "L%d amb update sat=%s, V=%11.4f floatX=%9.2lf fixX=%9.2lf\n", (u_signalIndex + 1) > 2 ? 5 : (u_signalIndex + 1),
          c_sat, d_omc, f_amb, pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValueRTK[u_signalIndex]);
#endif
      }
    }
  }

  /* calcu & save pdop */
  z_HtH.col = 4;
  z_HtH.row = 4;
  z_HtH.data = pd_HtH;
  z_HtHinv.col = 4;
  z_HtHinv.row = 4;
  z_HtHinv.data = pd_HtHinv;
  if (matrix_inverse(&z_HtH, &z_HtHinv))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      d_pdop += pd_HtHinv[u_i + u_i * 4];
    }
    if (d_pdop > 0.0)
    {
      pz_fixedAmbPool->f_pdop = (float)sqrt(d_pdop);
    }
  }

  for (w_i = 0; w_i < 3; w_i++)
  {
    pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_FIX][w_i]
      = pz_fixedAmbPool->pd_x_fix[w_i];
  }

  hpp_seq_deinit(&z_seqVar);

  OS_FREE(pd_deltaX);
  OS_FREE(pd_X);

  return;
}

/**
 * @brief clear ambiguity info
 * @param[in/out]  pz_fixedAmbPool is result of amb resolution
 * @return         void
 */
static void RTK_ResetFixedAmbPool(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;

  memset(pz_fixedAmbPool->pw_refSat, 0, MAX_GNSS_SIGNAL_FREQ * C_GNSS_MAX * sizeof(uint8_t));
  memset(pz_fixedAmbPool->u_fixedns, 0, C_GNSS_MAX * sizeof(uint8_t));
  pz_fixedAmbPool->u_fixedTotalSatCount = 0;
  pz_fixedAmbPool->w_continuefixedEpoch = 0;
  pz_fixedAmbPool->f_ratio = 0.0f;
  pz_fixedAmbPool->f_adop = 0.0f;
  pz_fixedAmbPool->u_fixStatus = GNSS_NONE_AMB_FIXED;

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_i])
    {
      continue;
    }
    memset(pz_fixedAmbPool->pz_fixedAmbSet[u_i]->d_fixedValueRTK, 0, MAX_GNSS_SIGNAL_FREQ * sizeof(double));
    memset(pz_fixedAmbPool->pz_fixedAmbSet[u_i]->u_continue_fix, 0, MAX_GNSS_SIGNAL_FREQ * sizeof(uint32_t));
    memset(pz_fixedAmbPool->pz_fixedAmbSet[u_i]->u_freqType, 0, MAX_GNSS_SIGNAL_FREQ * sizeof(gnss_SignalType));
    memset(pz_fixedAmbPool->pz_fixedAmbSet[u_i]->u_fix, 0, MAX_GNSS_SIGNAL_FREQ * sizeof(uint8_t));
  }

  return;
}

/**
 * @brief        get number of bias from pz_fixedAmbPool
 * @param[in]    pz_fixedAmbPool is result of amb resolution
 * @return       number of bias
 */
uint16_t rtk_getNumberOfBias(const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_satIndex = 0;
  uint8_t u_icoll = 0;
  uint8_t u_system = 0;
  uint8_t pu_groupValid[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX] = { 0 };
  uint16_t w_nBias = 0;
  uint16_t w_nGroup = 0;
  uint8_t w_svid = 0;
  gnss_FreqType z_freqType;
  const gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;
  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    for (z_freqType = 0; z_freqType < C_GNSS_FREQ_TYPE_MAX; z_freqType++)
    {
      for (u_icoll = C_GNSS_GPS; u_icoll < C_GNSS_MAX; u_icoll++)
      {
        /*w_svid =*/ gnss_cvt_SvIndex2Svid(u_satIndex, &u_system);
        if (u_system == C_GNSS_NONE)
        {
          continue;
        }
        if (u_system != u_icoll)
        {
          continue;
        }
        pz_fixedAmbSet = pz_fixedAmbPool->pz_fixedAmbSet[u_satIndex];
        if (NULL == pz_fixedAmbSet || GNSS_AMB_FLAG_NONE == pz_fixedAmbSet->u_fix[z_freqType] ||
          GNSS_AMB_FLAG_FLOAT == pz_fixedAmbSet->u_fix[z_freqType])
        {
          continue;
        }
        pu_groupValid[u_icoll][z_freqType] |= TRUE;
        w_nBias++;
      }
    }
  }
  for (z_freqType = 0; z_freqType < C_GNSS_FREQ_TYPE_MAX; z_freqType++)
  {
    for (u_icoll = C_GNSS_GPS; u_icoll < C_GNSS_MAX; u_icoll++)
    {
      if (pu_groupValid[u_icoll][z_freqType])
      {
        w_nGroup++;
      }
    }
  }
  return (w_nBias - w_nGroup);
}

/**
 * @brief set pz_prefixedAmbPool from pz_curfixedAmbPool
 * @param[in]    pz_curfixedAmbPool
 * @param[out]    pz_prefixedAmbPool
 * @return none
 */
static void rtk_ambUpdateFixedAmbPool(const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
  gnss_fixedSignalAmbPool_t** pz_prefixedAmbPool)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_initEpoch = FALSE;
  uint16_t w_nBiasCur = 0;
  uint16_t w_nBiasPre = 0;
  double d_timeDiff = 0.0;
  const double d_timeClose = 40.0;

  //init pz_prefixedAmbPool
  if (NULL == (*pz_prefixedAmbPool))
  {
    (*pz_prefixedAmbPool) = (gnss_fixedSignalAmbPool_t*)OS_MALLOC_FAST(sizeof(gnss_fixedSignalAmbPool_t));
    u_initEpoch = TRUE;
  }
  if (NULL == (*pz_prefixedAmbPool))
  {
    return;
  }

  if (GNSS_NONE_AMB_FIXED == pz_curfixedAmbPool->u_fixStatus)
  {
    (*pz_prefixedAmbPool)->w_continuefixedEpoch = 0;
    return;
  }
  else
  {
    (*pz_prefixedAmbPool)->w_continuefixedEpoch++;
  }

  w_nBiasCur = rtk_getNumberOfBias(pz_curfixedAmbPool);
  w_nBiasPre = rtk_getNumberOfBias((*pz_prefixedAmbPool));
  d_timeDiff = tm_GpsTimeDiff(&pz_curfixedAmbPool->z_time, &(*pz_prefixedAmbPool)->z_time);
  if ((!u_initEpoch) && (2.0 * w_nBiasCur < w_nBiasPre) && (d_timeDiff < d_timeClose) && (d_timeDiff > FABS_ZEROS))
  {
    return;
  }

  (*pz_prefixedAmbPool)->z_time = pz_curfixedAmbPool->z_time;
  (*pz_prefixedAmbPool)->u_fixStatus = pz_curfixedAmbPool->u_fixStatus;

  for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ * C_GNSS_MAX; ++u_i)
  {
    (*pz_prefixedAmbPool)->pw_refSat[u_i] = pz_curfixedAmbPool->pw_refSat[u_i];
  }

  // update fixed staellite count
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
  {
    (*pz_prefixedAmbPool)->u_fixedns[u_i] = pz_curfixedAmbPool->u_fixedns[u_i];
  }

  // update ratio and nb
  (*pz_prefixedAmbPool)->f_ratio = pz_curfixedAmbPool->f_ratio;
  (*pz_prefixedAmbPool)->u_nb = pz_curfixedAmbPool->u_nb;
  (*pz_prefixedAmbPool)->u_fixedTotalSatCount = pz_curfixedAmbPool->u_fixedTotalSatCount;

  if (NULL == (*pz_prefixedAmbPool)->pd_x_fix)
  {
    (*pz_prefixedAmbPool)->pd_x_fix = (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
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
        else
        {
          (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_continue_fix[u_j] = 0;
        }

        /*if ((*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_fix[u_j] >= 2 && pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_fix[u_j] != 2)
        {
          continue;
        }*/

        (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_freqType[u_j] = pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_freqType[u_j];
        (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->d_fixedValueRTK[u_j] = pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->d_fixedValueRTK[u_j];
        (*pz_prefixedAmbPool)->pz_fixedAmbSet[u_i]->u_fix[u_j] = pz_curfixedAmbPool->pz_fixedAmbSet[u_i]->u_fix[u_j];
      }
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

  return;
}

/**
 * @brief RTK zero combine ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static RTKAmbFixFailInfoType RTK_ZeroCombineAmbFix(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  RTKAmbFixFailInfoType q_fix_status = AMB_AR_SUCCESS;

  RTK_ResetFixedAmbPool(pz_fixedAmbPool);

  q_fix_status = rtk_ambFixRefSatSelect(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
  if (q_fix_status)
  {
    /* refsat select fail */
    LOGI(TAG_HPP, "RTK refsat select fail,status=%d\n", q_fix_status);
    q_fix_status = AMB_AR_FAIL_RefsatFail;
  }

  if (!q_fix_status)
  {
    q_fix_status = rtk_ambFixAR(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    if (q_fix_status)
    {
      LOGI(TAG_HPP, "RTK AR fail,status=%d\n", q_fix_status);
    }
  }

  if (!q_fix_status)
  {
    /* obtain fixed solution */
    rtk_ambZeroCombineUpdate(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
  }

  return q_fix_status;
}
/**
 * @brief juge the satellite status whether meet the condition of wide lane fix
 * @param[in]     u_targetConstellation
 * @param[in]     u_satIndex sat id
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     q_isArcLenLimit that TRUE limit the arc len and FALSE do not consider
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool the pool of fixed ambiguity
 * @param[out]    pd_ambArc the arc of ambiguity
 * @return        TRUE  meet the condition of wide lane fix, FALSE not meet
 */
BOOL rtk_SatMeetWideLaneFixCondition(uint8_t u_targetConstellation, uint8_t u_satIndex, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, BOOL q_isArcLenLimit,
  const gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, double* pd_ambArc)
{
  BOOL q_meetCondition = FALSE;
  uint8_t u_constellation = 0;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  float f_sample = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.f_sample;
  double d_ambArc2 = 0.0;
  double d_timeLock = 0.0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  const GpsTime_t* curtime = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const rtk_SatFilterObs_t* pz_corrStat = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);
  curtime = &pz_rtkAmbFixInputInfo->pz_satSignalPool->z_tor;
  *pd_ambArc = 0.0;
  /* deal time lock */
  if (f_sample < 1e-3)
  {
    f_sample = 1.0f;
  }
  d_timeLock = (double)f_sample * GNSS_MIN_LOCK;
  do 
  {
    pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
    pz_corrStat = pz_rtkAmbFixInputInfo->pz_FilterObs->pz_SatFilterObs[u_satIndex];
    if (NULL == pz_satMeas || NULL == pz_corrStat)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetConstellation != u_constellation)
    {
      continue;
    }
    if (NULL == pz_satMeas->pz_signalMeas[z_freqType1] || NULL == pz_satMeas->pz_signalMeas[z_freqType2] ||
      NULL == pz_corrStat->z_measUpdateFlag[z_freqType1] || NULL == pz_corrStat->z_measUpdateFlag[z_freqType2])
    {
      continue;
    }

    if ((pz_satMeas->pz_signalMeas[z_freqType1]->u_LLI & 2) || !pz_corrStat->z_measUpdateFlag[z_freqType1]->b_cpValid ||
      (pz_satMeas->pz_signalMeas[z_freqType2]->u_LLI & 2) || !pz_corrStat->z_measUpdateFlag[z_freqType2]->b_cpValid ||
      pz_satMeas->z_satPosVelClk.f_elevation < 20.0 * DEG2RAD)
    {
      continue;
    }

    if (GNSS_PART_LAMBDA_MP == (pz_fixedAmbPool->u_ambfix_mode) && NULL != pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag)
    {
      if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_PR) != MULTIPATH_FLAG_PR_NORMAL
        || (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType2] & MULTIPATH_FLAG_PR) != MULTIPATH_FLAG_PR_NORMAL)
      {
        continue;
      }
      /*only filter the high level multi-path observations*/
      if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_HIGH
        /*|| (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_LOW*/
        || (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType2] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_HIGH
        /*|| (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType2] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_LOW*/)
      {
        continue;
      }
    }

    w_x_id[0] = 0;
    w_x_id[1] = u_satIndex;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2)
    {
      continue;
    }
    *pd_ambArc = fabs(tm_GpsTimeDiff(&pz_satPool1->z_endTime, &pz_satPool1->z_beginTime));
    d_ambArc2 = fabs(tm_GpsTimeDiff(&pz_satPool2->z_endTime, &pz_satPool2->z_beginTime));
    if (d_ambArc2 < *pd_ambArc)
    {
      *pd_ambArc = d_ambArc2;
    }
    if (TRUE == q_isArcLenLimit)
    {
      if (fabs(tm_GpsTimeDiff(curtime, &pz_satPool1->z_endTime)) > 1e-4 ||
        fabs(tm_GpsTimeDiff(curtime, &pz_satPool2->z_endTime)) > 1e-4) /* unhealth,current epoch */
      {
        continue;
      }
      if (*pd_ambArc < d_timeLock)
      {
        continue;
      }
    }
    q_meetCondition = TRUE;
  } while (0);
  
  return q_meetCondition;
}
/**
 * @brief statistics the number of good quality satellite for the target reference satellite
 * @param[in]     u_targetConstellation
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]	    u_refSatIndex is the index of reference satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool the pool of fixed ambiguity
 * @param[in]     q_isArcLenLimit that TRUE limit the arc len and FALSE do not consider
 * @param[in]     d_fractThres the threshold of judged good quality satellite for the target reference satellite
 * @return the number of good quality satellite for the target reference satellite
 */
uint8_t rtk_ambFixStatisticsGoodQualitySatNum(uint8_t u_targetConstellation, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t u_refSatIndex,
  const gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, BOOL q_isArcLenLimit, double d_fractThres)
{
  uint8_t u_goodQualitySatNum = 0;
  uint8_t u_satIndex = 0;
  uint8_t u_refSatSvId = 0;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  double d_ambArc = 0.0;
  double d_DDambFloatValue = 0.0;
  double d_DDAmbFract = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatelliteMeas_t* pz_refSatMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef2 = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);
  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    pz_refSatMeas= pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_refSatIndex];
    if (FALSE == rtk_SatMeetWideLaneFixCondition(u_targetConstellation, u_satIndex, z_freqType1, z_freqType2, q_isArcLenLimit, pz_rtkAmbFixInputInfo, pz_fixedAmbPool, &d_ambArc))
    {
      continue;
    }
    pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
    if (NULL == pz_refSatMeas || NULL == pz_satMeas)
    {
      continue;
    }
    u_refSatSvId = pz_refSatMeas->u_svid;
    if (0 == u_refSatSvId || pz_satMeas->u_svid == u_refSatSvId)
    {
      continue;
    }
    w_x_id[0] = 0;
    w_x_id[1] = u_satIndex;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2)
    {
      continue;
    }
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refSatSvId, pz_refSatMeas->u_constellation);
    w_x_id[2] = z_filterType1;
    pz_satPoolRef1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPoolRef2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPoolRef1 || NULL == pz_satPoolRef2)
    {
      continue;
    }
    d_DDambFloatValue = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPool1->w_index] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPool2->w_index]
      - (pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPoolRef1->w_index] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPoolRef1->w_index]);

    d_DDAmbFract = fabs(d_DDambFloatValue - floor(d_DDambFloatValue + 0.5));
    if (d_DDAmbFract <= d_fractThres)
    {
      ++u_goodQualitySatNum;
    }
  }
  return u_goodQualitySatNum;
}
/**
 * @brief get wide lane reference satellite single frequence
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool the pool of fixed ambiguity
 * @param[in]     u_targetConstellation
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @return svid
 */
int8_t rtk_ambFixWideLaneRefSatSelect(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  uint8_t u_targetConstellation, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_satIndex = 0;
  uint8_t u_refsat = 0;
  uint8_t u_PAR_flag = pz_fixedAmbPool->u_ambfix_mode;// u_PAR_flag is PAR mode flag
  uint8_t u_goodQualitySatNum = 0;
  uint8_t u_refGoodQualityMaxSatNum = 0;
  int16_t s_refsatIndex = -1;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  double d_ambArc = 0.0;
  double d_refAmbLen = 0.0;
  float f_refEle = -1.0;
  BOOL q_isUpdateRef = FALSE;
  const double d_refSatMinEleLimit = 45.0 * DEG2RAD;
  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
    if (FALSE == rtk_SatMeetWideLaneFixCondition(u_targetConstellation, u_satIndex, z_freqType1, z_freqType2, FALSE, pz_rtkAmbFixInputInfo, pz_fixedAmbPool,&d_ambArc))
    {
      continue;
    }
    if (pz_satMeas->pz_signalMeas[z_freqType1]->f_cn0 < pz_fixedAmbPool->f_cn0Thres ||
      pz_satMeas->pz_signalMeas[z_freqType2]->f_cn0 < pz_fixedAmbPool->f_cn0Thres)
    {
      continue;
    }

    if (pz_satMeas->z_satPosVelClk.f_elevation < pz_fixedAmbPool->f_eleThres)
    {
      continue;
    }
    if (u_PAR_flag == GNSS_PART_LAMBDA_MP && NULL != pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag)
    {
      if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_PR) != MULTIPATH_FLAG_PR_NORMAL
        || (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType2] & MULTIPATH_FLAG_PR) != MULTIPATH_FLAG_PR_NORMAL)
      {
        continue;
      }
      if ((pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_HIGH 
        || (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType1] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_LOW
        || (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType2] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_HIGH
        || (pz_rtkAmbFixInputInfo->pz_FilterObs->pz_MPflag->pu_MPflag[u_satIndex][z_freqType2] & MULTIPATH_FLAG_CP) == MULTIPATH_FLAG_CP_LOW)
      {
        continue;
      }
    }
    u_goodQualitySatNum = rtk_ambFixStatisticsGoodQualitySatNum(u_targetConstellation, z_freqType1, z_freqType2, u_satIndex, pz_rtkAmbFixInputInfo, pz_fixedAmbPool, FALSE, 0.25);
    q_isUpdateRef = FALSE;
    if (s_refsatIndex < 0)
    {
      q_isUpdateRef = TRUE;
    }
    else if ((pz_satMeas->z_satPosVelClk.f_elevation) > f_refEle)
    {
      if ((TRUE == pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->b_staticFlag))
      {
        q_isUpdateRef = TRUE;
      }
      else if (f_refEle > d_refSatMinEleLimit && u_goodQualitySatNum >= u_refGoodQualityMaxSatNum)
      {
        q_isUpdateRef = TRUE;
      }
      else if (f_refEle > d_refSatMinEleLimit && d_ambArc >= d_refAmbLen)
      {
        q_isUpdateRef = TRUE;
      }
      else if (f_refEle <= d_refSatMinEleLimit)
      {
        q_isUpdateRef = TRUE;
      }
    }

    if (TRUE == q_isUpdateRef)
    {
      d_refAmbLen = d_ambArc;
      f_refEle = pz_satMeas->z_satPosVelClk.f_elevation;
      u_refGoodQualityMaxSatNum = u_goodQualitySatNum;
      s_refsatIndex = u_satIndex;
    }
  }

  if (s_refsatIndex > 0)
  {
    u_refsat = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[s_refsatIndex]->u_svid;
  }

  return u_refsat;
}
/**
 * @brief get widelane reference satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     pz_ambFix  ambiguity information
 * @return 0: success, other: fail
 */
static RTKAmbFixFailInfoType rtk_widelaneFixRefSatSelect(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_refsat;
  uint8_t u_isys = 0;
  uint8_t u_refsatcount = 0;
  RTKAmbFixFailInfoType u_ambStatus = AMB_AR_FAIL_UnknowError;

  pz_fixedAmbPool->f_cn0Thres = 30.0f;
  pz_fixedAmbPool->f_eleThres = (float)(15.0 * DEG2RAD);

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    u_refsat = rtk_ambFixWideLaneRefSatSelect(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, u_isys, z_freqType1, z_freqType2);
    if (u_refsat <= 0)
    {
      continue;
    }
    else
    {
      pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2] = u_refsat;
      u_refsatcount++;
    }
  }

  if (u_refsatcount > 0)
  {
    u_ambStatus = AMB_AR_SUCCESS;
  }

  return u_ambStatus;
}

/**
 * @brief get widelane satellite list
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     q_isArcLenLimit that TRUE limit the arc len and FALSE do not consider
 * @return 0: success, other: fail
 */
static uint8_t rtk_widelaneGetSatList(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, gnss_WideLandAmbFix_t* pz_ambFix, BOOL q_isArcLenLimit)
{
  uint8_t u_nb = 0;
  uint8_t u_nb_f = 0;
  uint8_t u_isys = 0;
  uint8_t u_constellation = 0;
  uint8_t u_satIndex = 0;
  uint8_t u_refsat_svid = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef2 = NULL;
  double d_ambArc = 0.0;
  double d_DDambFloatValue = 0.0;
  double d_DDAmbFract = 0.0;
  double d_fractThres = 0.4;/*unit in cycle*/

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }

    u_nb_f = 0;
    for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
    {
      if (u_nb >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      if (FALSE == rtk_SatMeetWideLaneFixCondition(u_isys, u_satIndex, z_freqType1, z_freqType2, q_isArcLenLimit, pz_rtkAmbFixInputInfo, pz_fixedAmbPool, &d_ambArc))
      {
        continue;
      }
      pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_satIndex];
      u_constellation = gnss_getConstellationEnumValueInLoop(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
      if (pz_satMeas->u_svid == u_refsat_svid || u_refsat_svid == 0)
      {
        continue;
      }
      w_x_id[0] = 0;
      w_x_id[1] = u_satIndex;
      w_x_id[2] = z_filterType1;
      pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
      w_x_id[2] = z_filterType2;
      pz_satPool2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
      if (NULL == pz_satPool1 || NULL == pz_satPool2)
      {
        continue;
      }
      w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, pz_satMeas->u_constellation);
      w_x_id[2] = z_filterType1;
      pz_satPoolRef1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
      w_x_id[2] = z_filterType2;
      pz_satPoolRef2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
      if (NULL == pz_satPoolRef1 || NULL == pz_satPoolRef2)
      {
        continue;
      }
      d_DDambFloatValue = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPool1->w_index] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPool2->w_index]
        - (pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPoolRef1->w_index] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[pz_satPoolRef1->w_index]);

      d_DDAmbFract = fabs(d_DDambFloatValue - floor(d_DDambFloatValue + 0.5));
      if (d_DDAmbFract > d_fractThres)
      {
        continue;
      }

      pz_ambFix[u_nb].u_satIndex = u_satIndex + 1;
      pz_ambFix[u_nb].u_amb_index_L1 = pz_satPool1->w_index;
      pz_ambFix[u_nb].u_amb_index_L2 = pz_satPool2->w_index;
      pz_ambFix[u_nb].u_ref_amb_index_L1 = pz_satPoolRef1->w_index;
      pz_ambFix[u_nb++].u_ref_amb_index_L2 = pz_satPoolRef2->w_index;

      u_nb_f++; // each frequence number count
    }

    if (0 == u_nb_f)
    {
      pz_fixedAmbPool->pw_refSat[u_isys * MAX_GNSS_SIGNAL_FREQ + z_freqType2] = 0; //clean refsat information
    }
  }

  return u_nb;
}

/**
 * @brief RTK widelane ambiguity result save
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @param[in]	     z_freqType1 is first frequence
 * @param[in]	     z_freqType2 is second frequence
 * @param[in]      u_nb is number of ambiguity
 * @param[in]      pz_ambFix is satellite information
 * @param[in]      pd_amb_int is widelane integer ambiguity
 * @return         void
 */
static void rtk_widelaneSaveResult(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t u_nb, gnss_WideLandAmbFix_t* pz_ambFix, const double* pd_amb_int)
{
  uint8_t u_i = 0;
  uint8_t u_svid = 0;
  uint8_t u_constellation = 0;
  uint8_t u_refsvid = 0;
  uint8_t u_refidx = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_sys = 0;
  gnss_fixedSignalAmb_t* pz_fix_amb = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  pz_fixedAmbPool->w_continuefixedEpoch++;
  for (u_i = 0; u_i < u_nb; u_i++)
  {
    if (0 == pz_ambFix[u_i].u_satIndex)
    {
      continue;
    }
    u_sat_idx = pz_ambFix[u_i].u_satIndex - 1;

    pz_fix_amb = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx];
    if (NULL == pz_fix_amb)
    {
      continue;
    }

    pz_satMeas = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx];
    if (NULL == pz_satMeas || NULL == pz_satMeas->pz_signalMeas[z_freqType1] || NULL == pz_satMeas->pz_signalMeas[z_freqType2])
    {
      continue;
    }

    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_constellation = pz_satMeas->u_constellation;

    u_refsvid = pz_fixedAmbPool->pw_refSat[u_constellation * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    u_refidx = gnss_cvt_Svid2SvIndex(u_refsvid, u_sys);
    pz_fixedAmbPool->u_fixStatus |= GNSS_W1_AMB_FIXED;

    if (NULL != pd_amb_int)
    {
      pz_fix_amb->d_fixedValueRTK[z_freqType2] = pd_amb_int[u_i];
    }
    pz_fix_amb->u_constellation = u_constellation;
    pz_fix_amb->u_svid = u_svid;
    pz_fix_amb->u_continue_fix[z_freqType2]++;
    pz_fix_amb->u_freqType[z_freqType1] = z_freqType1;
    pz_fix_amb->u_freqType[z_freqType2] = z_freqType2;

    pz_fix_amb->u_fix[z_freqType2] = 2;

    pz_fixedAmbPool->u_fixedns[u_sys]++;
    if (1 == pz_fixedAmbPool->u_fixedns[u_sys] && NULL != pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]) /* reference satellite */
    {
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->d_fixedValueRTK[z_freqType2] = 0.0;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_continue_fix[z_freqType1]++;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_continue_fix[z_freqType2]++;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_freqType[z_freqType1] = z_freqType1;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_freqType[z_freqType2] = z_freqType2;
      pz_fixedAmbPool->pz_fixedAmbSet[u_refidx]->u_fix[z_freqType2] = 2;
    }
  }
  pz_fixedAmbPool->u_fixedns[C_GNSS_BDS3] = pz_fixedAmbPool->u_fixedns[C_GNSS_BDS2] + pz_fixedAmbPool->u_fixedns[C_GNSS_BDS3];
  pz_fixedAmbPool->u_fixedns[C_GNSS_BDS2] = pz_fixedAmbPool->u_fixedns[C_GNSS_BDS3];
  return;
}

/**
 * @brief get X and Q of widelane ambiguity
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix is satellite information
 * @param[in]     u_nb is number of ambiguity
 * @param[out]    pd_X
 * @param[out]    pd_Q
 * @return 0: OK, 1: fail
 */
static uint8_t rtk_widelaneGetXQ(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_WideLandAmbFix_t* pz_ambFix, uint8_t u_nb, double* pd_X, double* pd_Q)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_sat_idx = 0;
  uint16_t uw_i1 = 0;
  uint16_t uw_i2 = 0;
  uint16_t uw_ri1 = 0;
  uint16_t uw_ri2 = 0;
  uint16_t uw_j1 = 0;
  uint16_t uw_j2 = 0;
  uint16_t uw_rj1 = 0;
  uint16_t uw_rj2 = 0;

  for (u_i = 0; u_i < u_nb; u_i++)
  {
    if (0 == pz_ambFix[u_i].u_satIndex)
    {
      u_status = 1;
      return u_status;
    }
    //u_sat_idx = pz_ambFix[u_i].u_satIndex - 1;

    uw_i1 = pz_ambFix[u_i].u_amb_index_L1;
    uw_i2 = pz_ambFix[u_i].u_amb_index_L2;
    uw_ri1 = pz_ambFix[u_i].u_ref_amb_index_L1;
    uw_ri2 = pz_ambFix[u_i].u_ref_amb_index_L2;

    pd_X[u_i] = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[uw_i1] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[uw_i2]
      - (pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[uw_ri1] - pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X[uw_ri2]);

    for (u_j = 0; u_j < u_nb; u_j++)
    {
      if (0 == pz_ambFix[u_j].u_satIndex)
      {
        u_status = 1;
        return u_status;
      }
      //u_sat_idx = pz_ambFix[u_j].u_satIndex - 1;

      uw_j1 = pz_ambFix[u_j].u_amb_index_L1;
      uw_j2 = pz_ambFix[u_j].u_amb_index_L2;
      uw_rj1 = pz_ambFix[u_j].u_ref_amb_index_L1;
      uw_rj2 = pz_ambFix[u_j].u_ref_amb_index_L2;

      pd_Q[u_j + u_i * u_nb] = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i1, uw_j1)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri1, uw_rj1)];
      pd_Q[u_j + u_i * u_nb] -= pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri1, uw_j1)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i1, uw_rj1)];

      pd_Q[u_j + u_i * u_nb] += pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i2, uw_j2)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri2, uw_rj2)];
      pd_Q[u_j + u_i * u_nb] -= pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri2, uw_j2)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i2, uw_rj2)];

      pd_Q[u_j + u_i * u_nb] += pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i1, uw_rj2)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri2, uw_j1)];
      pd_Q[u_j + u_i * u_nb] -= pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i1, uw_j2)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i2, uw_j1)];

      pd_Q[u_j + u_i * u_nb] += pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri1, uw_j2)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_i2, uw_rj1)];
      pd_Q[u_j + u_i * u_nb] -= pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri2, uw_rj1)]
        + pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q[IUTM(uw_ri1, uw_rj2)];

      pd_Q[u_i + u_j * u_nb] = pd_Q[u_j + u_i * u_nb];
    }
  }

  return u_status;
}

/**
 * @brief RTK widelane ambiguity confirm
 * @param[in/out]  u_nb is number of bias
 * @return         TRUE success; else fail
 */
static uint8_t rtk_widelaneConfirm(uint8_t u_nb, float f_ratio, double d_adop)
{
  uint8_t u_result = TRUE;

  if (d_adop > 10.0)
  {
    u_result = FALSE;
  }
  else
  {
    if (u_nb <= 3)
    {
      if (f_ratio <= 10.0)
      {
        u_result = FALSE;
      }
    }
    else if (u_nb <= 8)
    {
      if (f_ratio <= 3.0)
      {
        u_result = FALSE;
      }

      if ((f_ratio / d_adop) < 1.5)
      {
        u_result = FALSE;
      }
    }
    else
    {
      if ((f_ratio / d_adop) < 1.5)
      {
        u_result = FALSE;
      }
    }
  }

  return u_result;
}

/**
 * @brief RTK widelane ambiguity resolution
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of ambigutiy resolution
 * @param[in]	     z_freqType1 is first frequence
 * @param[in]	     z_freqType2 is second frequence
 * @return         u_status 0 success; else fail
 */
static uint8_t rtk_widelaneFix(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_nb;
  uint8_t u_candidate = 2;
  float f_boot_rate = 0.0;
  float f_ratio = 0.0;
  double d_adop = 0.0;
  double* pd_X = NULL;
  double* pd_Q = NULL;
  double* pd_amb_int = NULL;
  double* pd_sigma = NULL;
  gnss_WideLandAmbFix_t* pz_ambFix = NULL;
  RTKAmbFixFailInfoType u_status;

  pz_ambFix = (gnss_WideLandAmbFix_t*)OS_MALLOC_FAST(MAX_GNSS_TRK_MEAS_NUMBER * sizeof(gnss_WideLandAmbFix_t));
  if (NULL == pz_ambFix)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }

  // get widelane satellite list
  u_nb = rtk_widelaneGetSatList(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2, pz_ambFix, FALSE);

  // update nb
  pz_fixedAmbPool->u_nb = u_nb;

  if (u_nb <= 3)
  {
    u_status = AMB_AR_FAIL_LessNS;
  }

  while (u_nb > 3)
  {
    pd_X = (double*)OS_MALLOC_FAST(u_nb * sizeof(double));
    pd_Q = (double*)OS_MALLOC_FAST(u_nb * u_nb * sizeof(double));
    pd_amb_int = (double*)OS_MALLOC_FAST(u_nb * u_candidate * sizeof(double));
    pd_sigma = (double*)OS_MALLOC_FAST(u_candidate * sizeof(double));
    if (NULL == pd_X || NULL == pd_Q || NULL == pd_amb_int || NULL == pd_sigma)
    {
      u_status = AMB_AR_FAIL_MemError;
      break;
    }

    if (rtk_widelaneGetXQ(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, pz_ambFix, u_nb, pd_X, pd_Q))
    {
      u_status = AMB_AR_FAIL_DataError;
      break;
    }

    /* search ambiguity by lambda */
    if (rtk_lambda(u_nb, u_candidate, pd_X, pd_Q, pd_amb_int, pd_sigma))
    {
      u_status = AMB_AR_FAIL_LambdaError;
      break;
    }
    else
    {
      f_ratio = (float)(pd_sigma[0] > 0 ? (float)(pd_sigma[1] / pd_sigma[0]) : 0.0);
      f_ratio = f_ratio > 999.99f ? 999.99f : f_ratio;

      // update ratio
      pz_fixedAmbPool->f_ratio = f_ratio;
      pz_fixedAmbPool->f_lambda_sigma[0] = pd_sigma[0];
      pz_fixedAmbPool->f_lambda_sigma[1] = pd_sigma[1];

      if (f_ratio >= 2.5)
      {
        d_adop = rtk_adop_get();
        pz_fixedAmbPool->f_adop = (float)d_adop;

        LOGI(TAG_HPP, " widelane amb validation ok (nb=%d ratio=%.2f s=%.2f/%.2f adop=%.2f)\n", u_nb, f_ratio, pd_sigma[1], pd_sigma[0], d_adop);

        rtk_widelaneSaveResult(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2, u_nb, pz_ambFix, pd_amb_int);
        u_status = AMB_AR_SUCCESS;
        break;
      }
      else
      {
        LOGI(TAG_HPP, " widelane amb validation fail (nb=%d ratio=%.2f s=%.2f/%.2f)\n", u_nb, f_ratio, pd_sigma[1], pd_sigma[0]);
        u_status = AMB_AR_FAIL_ParFail;
      }
    }

    break;
  }

  if (pd_X) OS_FREE(pd_X);
  if (pd_Q) OS_FREE(pd_Q);
  if (pd_amb_int) OS_FREE(pd_amb_int);
  if (pd_sigma) OS_FREE(pd_sigma);
  if (pz_ambFix) OS_FREE(pz_ambFix);

  return u_status;
}

/**
 * @brief RTK widelane ambiguity output
 * @param[in/out] pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[in]     pz_fixedAmbPool is result of amb resolution
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @return        void
 */
static void rtk_widelanePrintSat(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_sat_idx = 0;
  uint8_t u_refsat_svid = 0;
  uint8_t u_sys = 0;
  uint8_t u_svid = 0;
  uint8_t u_satSys = 255;
  uint8_t u_size = 10;
  uint8_t u_refsat_idx = 0;
  char c_sat[4] = "";
  char buffSat[BUFF_SIZE] = { 0 };
  char buffOmc[BUFF_SIZE] = { 0 };
  char buffFloat[BUFF_SIZE] = { 0 };
  char buffFix[BUFF_SIZE] = { 0 };
  char* p_buffSat = buffSat;
  char* p_buffOmc = buffOmc;
  char* p_buffFloat = buffFloat;
  char* p_buffFix = buffFix;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  float f_amb = 0.0f;
  double d_omc = 0.0;
  double d_curR = 0.0;
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  double* pd_X = NULL;
  SeqKalmanVar_t z_seqVar = { 0 };
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef2 = NULL;

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  pd_X = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X;

  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx])
    {
      continue;
    }

    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_sys * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    if (u_refsat_svid <= 0 || u_refsat_svid == u_svid)
    {
      continue;
    }

    if (pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_fix[z_freqType2] != 2)
    {
      continue;
    }

    u_refsat_idx = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);

    if (u_satSys != u_refsat_idx)
    {
      u_satSys = u_refsat_idx;
      if ((p_buffSat - buffSat) >= BUFF_SIZE - u_size)
      {
        continue;
      }
      /* print log */
      satidx_SatString(u_sat_idx, c_sat);
      svid_SatString(u_refsat_svid, u_sys, c_sat);
      p_buffSat += snprintf(p_buffSat, u_size, " | %s", c_sat);
      p_buffFix += snprintf(p_buffFix, u_size, " |    ");
      p_buffFloat += snprintf(p_buffFloat, u_size, " |    ");
      p_buffOmc += snprintf(p_buffOmc, u_size, " |    ");
    }

    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
    w_x_id[2] = z_filterType1;
    pz_satPoolRef1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPoolRef2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPoolRef1 || NULL == pz_satPoolRef2)
    {
      continue;
    }

    f_amb = (float)(pd_X[pz_satPool1->w_index] - pd_X[pz_satPool2->w_index] -
      (pd_X[pz_satPoolRef1->w_index] - pd_X[pz_satPoolRef2->w_index]));

    /*OMC*/
    d_omc = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValueRTK[z_freqType2] - f_amb;

    if ((p_buffSat - buffSat) >= BUFF_SIZE - u_size)
    {
      continue;
    }

    /* print log */
    satidx_SatString(u_sat_idx, c_sat);
    p_buffSat += snprintf(p_buffSat, u_size, "      %s", c_sat);
    p_buffFix += snprintf(p_buffFix, u_size, "%9.2f", pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValueRTK[z_freqType2]);
    p_buffFloat += snprintf(p_buffFloat, u_size, "%9.2f", f_amb);
    p_buffOmc += snprintf(p_buffOmc, u_size, "%9.2f", d_omc);
  }

  LOGI(TAG_HPP, " Sat  %s\n", buffSat);
  LOGI(TAG_HPP, " Float%s\n", buffFloat);
  LOGI(TAG_HPP, " Fix  %s\n", buffFix);
  LOGI(TAG_HPP, " Omc  %s\n", buffOmc);
}

/**
 * @brief RTK widelane ambiguity update
 * @param[in/out] pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[in]     pz_fixedAmbPool is result of amb resolution
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @return        void
 */
static void rtk_widelaneFixUpdate(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2)
{
  uint8_t u_i = 0;
  uint8_t u_sat_idx = 0;
  uint8_t u_refsat_svid = 0;
  uint8_t u_sys = 0;
  uint8_t u_svid = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  double d_amb = 0.0;
  double d_pdop = 0.0;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double pd_HtH[16] = { 0.0 };
  double pd_HtHinv[16] = { 0.0 };
  gnss_FilterStateEnumType z_filterType1 = GNSS_FILTER_STATE_NUM;
  gnss_FilterStateEnumType z_filterType2 = GNSS_FILTER_STATE_NUM;
  double* pd_X = NULL;
  double* pd_deltaX = NULL;
  matrix_t  z_HtH;
  matrix_t  z_HtHinv;
  LogLevelEnum z_logLevel;
  SeqKalmanVar_t z_seqVar = { 0 };
  const gnss_EKFstateRepresent_t* pz_satPool1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool2 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef1 = NULL;
  const gnss_EKFstateRepresent_t* pz_satPoolRef2 = NULL;
  const double* pd_satPosClk = NULL;

  if (NULL == pz_fixedAmbPool)
  {
    return;
  }

  if (hpp_seq_init(&z_seqVar, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax))
  {
    return;
  }

  pd_X = (double*)OS_MALLOC_FAST(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax * sizeof(double));
  pd_deltaX = (double*)OS_MALLOC_FAST(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax * sizeof(double));
  if (any_Ptrs_Null(2, pd_X, pd_deltaX))
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

  memcpy(pz_fixedAmbPool->pd_x_fix, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  memcpy(pz_fixedAmbPool->pd_q_fix, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_Q, NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
  memcpy(pd_X, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X, GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
  memset(pd_deltaX, 0, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->w_nmax * sizeof(double));

  z_filterType1 = convertFreqAmb2FilterType(z_freqType1);
  z_filterType2 = convertFreqAmb2FilterType(z_freqType2);

  for (u_sat_idx = 0; u_sat_idx < ALL_GNSS_SYS_SV_NUMBER; u_sat_idx++)
  {
    if (NULL == pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx])
    {
      continue;
    }

    u_svid = gnss_cvt_SvIndex2Svid(u_sat_idx, &u_sys);
    u_refsat_svid = pz_fixedAmbPool->pw_refSat[u_sys * MAX_GNSS_SIGNAL_FREQ + z_freqType2];
    if (u_refsat_svid <= 0 || u_refsat_svid == u_svid)
    {
      continue;
    }

    if (pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->u_fix[z_freqType2] != 2)
    {
      continue;
    }

    w_x_id[0] = 0;
    w_x_id[1] = u_sat_idx;
    w_x_id[2] = z_filterType1;
    pz_satPool1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPool2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[1] = gnss_cvt_Svid2SvIndex(u_refsat_svid, u_sys);
    w_x_id[2] = z_filterType1;
    pz_satPoolRef1 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    w_x_id[2] = z_filterType2;
    pz_satPoolRef2 = getEKF_status(w_x_id, pz_rtkAmbFixInputInfo->pz_EKFstateRepPool);
    if (NULL == pz_satPool1 || NULL == pz_satPool2 || NULL == pz_satPoolRef1 || NULL == pz_satPoolRef2)
    {
      continue;
    }

    d_amb = (pd_X[pz_satPool1->w_index] - pd_X[pz_satPool2->w_index] -
      (pd_X[pz_satPoolRef1->w_index] - pd_X[pz_satPoolRef2->w_index]));

    hpp_seq_ClearH(&z_seqVar);

    /*PDOP*/
    pd_satPosClk = pz_rtkAmbFixInputInfo->pz_satSignalPool->pz_satMeas[u_sat_idx]->z_satPosVelClk.d_satPosClk;
    if (gnss_Dot(pd_satPosClk, pd_satPosClk, 3) > FABS_ZEROS)
    {
      gnss_satRot(pd_X, pd_satPosClk, pd_satPosRot);
      gnss_unitVector(pd_X, pd_satPosRot, pd_unitVector);
      pd_HtH[0] += pd_unitVector[0] * pd_unitVector[0];
      pd_HtH[1 * 4 + 1] += pd_unitVector[1] * pd_unitVector[1];
      pd_HtH[2 * 4 + 2] += pd_unitVector[2] * pd_unitVector[2];
      pd_HtH[3 * 4 + 3] += 1 * 1;
      pd_HtH[1] = pd_HtH[4] = pd_HtH[4] + pd_unitVector[0] * pd_unitVector[1];
      pd_HtH[2] = pd_HtH[8] = pd_HtH[8] + pd_unitVector[0] * pd_unitVector[2];
      pd_HtH[6] = pd_HtH[9] = pd_HtH[9] + pd_unitVector[1] * pd_unitVector[2];
      pd_HtH[3] = pd_HtH[12] = pd_HtH[12] + pd_unitVector[0] * 1;
      pd_HtH[7] = pd_HtH[13] = pd_HtH[13] + pd_unitVector[1] * 1;
      pd_HtH[11] = pd_HtH[14] = pd_HtH[14] + pd_unitVector[2] * 1;
    }

    /*OMC*/
    d_omc = pz_fixedAmbPool->pz_fixedAmbSet[u_sat_idx]->d_fixedValueRTK[z_freqType2] - d_amb;
    d_curR = GNSS_VAR_HOLDAMB;

    /*H*/
    hpp_seq_AddH(pz_satPool1->w_index, 1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPool2->w_index, -1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPoolRef1->w_index, -1.0, &z_seqVar);
    hpp_seq_AddH(pz_satPoolRef2->w_index, 1.0, &z_seqVar);

    hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
    hpp_seq_PredictStep(pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar);
    hpp_seq_measUpdate(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pq_paraValid, pz_fixedAmbPool->pd_x_fix, pd_deltaX, pz_fixedAmbPool->pd_q_fix, &z_seqVar, 0.0);
  }

  // print widelane info 
  z_logLevel = log_GetLogLevel();
  if (z_logLevel >= LOG_LEVEL_I)
  {
    LOGI(TAG_HPP, " widelane start updating...\n");
    rtk_widelanePrintSat(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2);
    LOGI(TAG_HPP, " widelane updating success...\n");
  }

  /* calcu & save pdop */
  z_HtH.col = 4;
  z_HtH.row = 4;
  z_HtH.data = pd_HtH;
  z_HtHinv.col = 4;
  z_HtHinv.row = 4;
  z_HtHinv.data = pd_HtHinv;
  if (matrix_inverse(&z_HtH, &z_HtHinv))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      d_pdop += pd_HtHinv[u_i + u_i * 4];
    }
    if (d_pdop > 0.0)
    {
      pz_fixedAmbPool->f_pdop = (float)sqrt(d_pdop);
    }
  }

  hpp_seq_deinit(&z_seqVar);

  OS_FREE(pd_deltaX);
  OS_FREE(pd_X);

  return;
}

/**
 * @brief          RTK widelane combine solution availability check
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @param[in]      u_rtkFilterPosStatus is input information of fix type
 * @return         0 for success, other for failure
 */
static uint8_t rtk_wideLaneCombineSol_availabilityCheck(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, uint8_t u_rtkFilterPosStatus)
{
  uint16_t u_nb;
  uint8_t u_status = AMB_AR_FAIL_UnknowError;
  uint8_t u_i;
  BOOL b_openSky;
  float f_pdop;
  float f_adop;
  float f_ratio;
  rtk_filterInfo_t* pz_filterInfo;
  rtk_filter_sol_t* pz_filterSol;

  pz_filterInfo = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo;
  if (NULL == pz_filterInfo || NULL == pz_fixedAmbPool)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }
  pz_filterSol = pz_filterInfo->pz_filterSol;
  if (NULL == pz_filterSol)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }

  // save ewl pos
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_filterSol->pd_filterPos[u_rtkFilterPosStatus][u_i]
      = pz_fixedAmbPool->pd_x_fix[u_i];
  }
  pz_filterSol->pu_posValid[u_rtkFilterPosStatus] = RTK_FILTER_SOL_STAT_SAVE;

  u_nb = pz_fixedAmbPool->u_nb;
  f_pdop = pz_fixedAmbPool->f_pdop;
  f_adop = pz_fixedAmbPool->f_adop;
  f_ratio = pz_fixedAmbPool->f_ratio;

  b_openSky = FALSE;
  if ((pz_filterInfo->t_openSkyCount & 0x3) == 0x3)
  {
    b_openSky = TRUE;
  }

  if (f_pdop < FABS_ZEROS || f_ratio < 1.0)
  {
    pz_filterSol->pu_posValid[u_rtkFilterPosStatus] = RTK_FILTER_SOL_STAT_ERR;
  }
  if (u_nb >= 8 && f_pdop < 3.8 && f_ratio > 2.5 && f_ratio / f_adop > 1.5)
  {
    pz_filterSol->pu_posValid[u_rtkFilterPosStatus] = RTK_FILTER_SOL_STAT_VALID;
  }
  else
  {
    pz_filterSol->pu_posValid[u_rtkFilterPosStatus] = RTK_FILTER_SOL_STAT_WARN;
  }

  if (pz_filterSol->pu_posValid[u_rtkFilterPosStatus] == RTK_FILTER_SOL_STAT_ERR)
  {
    u_status = AMB_AR_FAIL_FixCheck;
    LOGI(TAG_HPP, "** WideLane FixSol Check ** <ERROR> nb=%d opensky=%d ratio=%.2f pdop=%.2f adop=%.2f\n",
      u_nb, b_openSky, f_ratio, f_pdop, f_adop);
  }
  else if (pz_filterSol->pu_posValid[u_rtkFilterPosStatus] == RTK_FILTER_SOL_STAT_WARN)
  {
    u_status = AMB_AR_FAIL_FixCheck;
    LOGI(TAG_HPP, "** WideLane FixSol Check ** <WARN!> nb=%d opensky=%d ratio=%.2f pdop=%.2f adop=%.2f\n",
      u_nb, b_openSky, f_ratio, f_pdop, f_adop);
  }
  else if (pz_filterSol->pu_posValid[u_rtkFilterPosStatus] == RTK_FILTER_SOL_STAT_VALID)
  {
    u_status = AMB_AR_SUCCESS;
    LOGI(TAG_HPP, "** WideLane FixSol Check ** <VALID> nb=%d opensky=%d ratio=%.2f pdop=%.2f adop=%.2f\n",
      u_nb, b_openSky, f_ratio, f_pdop, f_adop);
  }

  return u_status;
}
/**
 * @brief RTK widelane ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         void
 */
static void rtk_widelaneAmbResolution(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_i = 0;
  rtk_filterInfo_t* pz_filterInfo = NULL;
  rtk_filter_sol_t* pz_filterSol = NULL;
  gnss_FreqType z_freqType1 = C_GNSS_FREQ_TYPE_L1;
  gnss_FreqType z_freqType2 = C_GNSS_FREQ_TYPE_L2;
  gnss_FreqType z_freqType3 = C_GNSS_FREQ_TYPE_L5;
  RTKAmbFixFailInfoType q_fix_status = AMB_AR_FAIL_UnknowError;
  RTKAmbFixFailInfoType q_check_status = AMB_AR_FAIL_UnknowError;

  pz_filterInfo = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo;
  if (NULL == pz_filterInfo || NULL == pz_fixedAmbPool)
  {
    return;
  }
  pz_filterSol = pz_filterInfo->pz_filterSol;
  if (NULL == pz_filterSol)
  {
    return;
  }

  // init u_fixTypeIdx
  pz_rtkAmbFixInputInfo->u_fixTypeIdx = RTK_FIXTYPE_EXTRA_WIDELANE; // EWL

  RTK_ResetFixedAmbPool(pz_fixedAmbPool);
  pz_fixedAmbPool->u_ambfix_mode = GNSS_PART_LAMBDA_MP;

  if (!(rtk_widelaneFixRefSatSelect(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, z_freqType3)))
  {
    // EWL mode
    LOGI(TAG_HPP, " extra-widelane mode\n");

    /* EWL ambiguity resolution */
    if ((q_fix_status = rtk_widelaneFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, z_freqType3)))
    {
      LOGI(TAG_HPP, " extra-widelane ar fail, status=%d\n", q_fix_status);
    }

    pz_rtkAmbFixInputInfo->pu_searchLoopNb[FIRST_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

    if (!q_fix_status)
    {
      rtk_widelaneFixUpdate(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType2, z_freqType3);

      // ewl fix availability check
      q_check_status = rtk_wideLaneCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, RTK_FILTER_POS_FIX_EWL);
      if (q_check_status == AMB_AR_SUCCESS)
      {
        rtk_integ_save_wideLaneInfo(pz_fixedAmbPool, RTK_FILTER_POS_FIX_EWL, pz_rtkAmbFixInputInfo);
      }
    }
  }

  // init u_fixTypeIdx
  pz_rtkAmbFixInputInfo->u_fixTypeIdx = RTK_FIXTYPE_WIDELANE;

  RTK_ResetFixedAmbPool(pz_fixedAmbPool);

  if (!(q_fix_status = rtk_widelaneFixRefSatSelect(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2)))
  {
    // L1L2 mode
    LOGI(TAG_HPP, " widelane L1L2 mode\n");

    /* widelane ambiguity resolution */
    if ((q_fix_status = rtk_widelaneFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2)))
    {
      LOGI(TAG_HPP, " widelane ar fail, status=%d\n", q_fix_status);
    }

    pz_rtkAmbFixInputInfo->pu_searchLoopNb[SECOND_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

    if (!q_fix_status)
    {
      rtk_widelaneFixUpdate(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType2);
      // save wl pos
      if (pz_fixedAmbPool->u_fixStatus == GNSS_W1_AMB_FIXED)
      {
        //wl fix availability check
        q_check_status = rtk_wideLaneCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, RTK_FILTER_POS_FIX_WL);
        if (q_check_status == AMB_AR_SUCCESS)
        {
          rtk_integ_save_wideLaneInfo(pz_fixedAmbPool, RTK_FILTER_POS_FIX_WL, pz_rtkAmbFixInputInfo);
        }
      }
    }
  }
  if ((AMB_AR_SUCCESS != q_check_status) &&
    !(q_fix_status = rtk_widelaneFixRefSatSelect(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType3)))
  {
    // L1L5 mode
    LOGI(TAG_HPP, " widelane L1L5 mode\n");

    /* widelane ambiguity resolution */
    if ((q_fix_status = rtk_widelaneFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType3)))
    {
      LOGI(TAG_HPP, " widelane ar fail, status=%d\n", q_fix_status);
    }

    pz_rtkAmbFixInputInfo->pu_searchLoopNb[THIRD_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

    if (!q_fix_status)
    {
      rtk_widelaneFixUpdate(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, z_freqType1, z_freqType3);
      // save wl pos
      if (pz_fixedAmbPool->u_fixStatus == GNSS_W1_AMB_FIXED)
      {
        //wl fix availability check
        q_check_status = rtk_wideLaneCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool, RTK_FILTER_POS_FIX_WL);
        if (q_check_status == AMB_AR_SUCCESS)
        {
          rtk_integ_save_wideLaneInfo(pz_fixedAmbPool, RTK_FILTER_POS_FIX_WL, pz_rtkAmbFixInputInfo);
        }
      }
    }
  }

  if (AMB_AR_SUCCESS != q_check_status)
  {
    LOGI(TAG_HPP, " widelane ar fail: not valid\n");
  }
  else
  {
    // update widelane fixed amb pool 
    rtk_ambUpdateFixedAmbPool(pz_fixedAmbPool, &pz_rtkAmbFixInputInfo->pz_preFixedAmbPool[pz_rtkAmbFixInputInfo->u_fixTypeIdx]);

  }
  return;
}

/**
 * @brief          RTK zero combine solution availability check
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         0 represent success, other failure
 */
static uint8_t rtk_zeroCombineSol_availabilityCheck(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  BOOL b_openSky = FALSE;
  uint8_t u_status = AMB_AR_FAIL_UnknowError;
  uint8_t u_staticFlag = gnss_engine_get_peMode_historyStastic_flag();
  uint8_t u_i = 0;
  uint8_t u_gap = 0;
  uint16_t w_nb = 0;
  uint16_t w_resNumPhase = 0;
  uint16_t w_numOverQuarter = 0;
  uint8_t u_fixedTotalSatCount = 0;
  float f_adop = 0.0;
  float f_pdop = 0.0;
  float f_ratio = 0.0;
  double d_floatSTDCode = 0.0;
  double d_floatSTDPhase = 0.0;
  double d_fixedSTDCode = 0.0;
  double d_fixedSTDPhase = 0.0;
  double d_fixedMaxPhase = 0.0;
  rtk_filterInfo_t* pz_filterInfo = NULL;
  rtk_filter_sol_t* pz_filterSol = NULL;
  /* threshold define begin */
  const uint8_t d_minNv = 6; //min num of var
  const double d_OverQuarterPercent = 0.45; //max ratio over quarter residual exist
  const double d_pdopThres = 50.0; //max pdop
  const double d_STDphaseThres_STDgain = 0.02; //threshold of phase's STD(m) for STD gain up after fix case
  const double d_STDcodeThres_STDgain = 3.0; //threshold of code's STD(m) for STD gain up after fix case
  const double d_STDphaseThres_STDgain_2 = 0.05; //threshold of phase's STD(m) for STD gain up after fix case
  const double d_STDcodeThres_STDgain_2 = 2.0; //threshold of code's STD(m) for STD gain up after fix case
  const double d_STDphaseAmplifyThres = 1.8; //max code's STD magnification times
  const double d_STDcodeAmplifyThres = 1.5; //max code's STD magnification times
  const double d_pdopMultiStdThres = 1.5; //max possible error, get form pdop mulitiply post res STD (m)
  const double d_pdopMultiMaxThres = 2.0; //max possible error, get form pdop mulitiply post res MAX (m)
  /* threshold define end */

  pz_filterInfo = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo;
  if (NULL == pz_filterInfo || NULL == pz_fixedAmbPool || NULL == pz_rtkAmbFixInputInfo->pz_FilterObs)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }
  pz_filterSol = pz_filterInfo->pz_filterSol;
  if (NULL == pz_filterSol)
  {
    u_status = AMB_AR_FAIL_MemError;
    return u_status;
  }
  if (pz_fixedAmbPool->u_fixStatus != GNSS_L1_AMB_FIXED)
  {
    u_status = AMB_AR_FAIL_LambdaError;
    return u_status;
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_filterSol->pd_filterPos[RTK_FILTER_POS_FIX][u_i]
      = pz_fixedAmbPool->pd_x_fix[u_i];
  }
  pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_SAVE;

  //calculate fixed solution residual
  RTK_calculateResidual(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pd_X, pz_rtkAmbFixInputInfo->pz_satSignalPool,
                        pz_rtkAmbFixInputInfo->pz_OSRblock, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo,
                        pz_rtkAmbFixInputInfo->pz_EKFstateRepPool, pz_fixedAmbPool, pz_rtkAmbFixInputInfo->pz_FilterObs,
                        RES_FIXED);

  w_nb = pz_fixedAmbPool->u_nb;
  u_gap = pz_fixedAmbPool->u_gap;
  f_adop = pz_fixedAmbPool->f_adop;
  f_pdop = pz_fixedAmbPool->f_pdop;
  f_ratio = pz_fixedAmbPool->f_ratio;
  u_fixedTotalSatCount = pz_fixedAmbPool->u_fixedTotalSatCount;
  w_resNumPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->w_numPhaseRes;
  w_numOverQuarter = pz_rtkAmbFixInputInfo->pz_FilterObs->w_numOverQuarter;
  d_floatSTDCode = pz_rtkAmbFixInputInfo->pz_FilterObs->d_postCodeSTD;
  d_floatSTDPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->d_postPhaseSTD;
  d_fixedSTDCode = pz_rtkAmbFixInputInfo->pz_FilterObs->d_fixedCodeSTD;
  d_fixedSTDPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->d_fixedPhaseSTD;
  d_fixedMaxPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->d_maxPhaseBias;

  if ((pz_filterInfo->t_openSkyCount & 0x3) == 0x3)
  {
    b_openSky = TRUE;
  }
  else
  {
    b_openSky = FALSE;
  }

  if (f_adop < FABS_ZEROS || f_pdop < FABS_ZEROS || f_ratio < 1.0 ||
    d_floatSTDCode < FABS_ZEROS || d_floatSTDPhase < FABS_ZEROS ||
    d_fixedSTDCode < FABS_ZEROS || d_fixedSTDPhase < FABS_ZEROS)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_ERR;
  }
  //about std
  else if (d_fixedSTDPhase > d_STDphaseThres_STDgain && d_fixedSTDCode > d_STDcodeThres_STDgain &&
    d_fixedSTDPhase / d_floatSTDPhase > d_STDphaseAmplifyThres)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  //about std
  else if (d_fixedSTDPhase > d_STDphaseThres_STDgain_2 && d_fixedSTDCode > d_STDcodeThres_STDgain_2 &&
    (d_fixedSTDPhase > d_floatSTDPhase || d_fixedSTDPhase / d_floatSTDPhase > d_STDcodeAmplifyThres))
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  //about pdop
  else if (f_pdop > d_pdopThres || f_pdop * d_fixedSTDPhase > d_pdopMultiStdThres
    || f_pdop * d_fixedMaxPhase > d_pdopMultiMaxThres)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  //residuals over 1/4 wave are too many
  else if (w_numOverQuarter > d_OverQuarterPercent * w_resNumPhase)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  //temp condition for park in close sky(update in future)
  else if (2 == u_gap && u_staticFlag && d_fixedSTDCode > 3.0 && f_adop > 2.0 && f_pdop > 2.0 && f_ratio < 1.5 * f_adop)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  else if ((pz_filterInfo->t_openSkyCount & 0x3) != 0x3 && d_fixedSTDCode > 1.0 && f_adop > 3.0 &&
    GNSS_PART_LAMBDA_FULL != pz_fixedAmbPool->u_ambfix_mode)
  {
    if (u_staticFlag && w_nb < 8)/* static mode check logic */
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
    }
    else if (w_nb < 10 && f_ratio < f_adop && u_fixedTotalSatCount < 8)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
    }
    else if (f_adop < 6.0 && f_ratio > 3.0 && d_floatSTDPhase < 0.3 && d_fixedSTDPhase < 0.3)
    {
      pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_VALID;
    }
  }
  else if (w_nb >= d_minNv)
  {
    if ((pz_filterInfo->t_openSkyCount & 0x3) == 0x3)
    {
      if (!u_gap && f_adop < 3.5 && f_ratio > 2.2 && d_floatSTDPhase < 0.6 && d_fixedSTDPhase < 0.6)
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_VALID;
      }
      else if (u_gap && f_adop < 3.0 && f_ratio > 2.5 && d_floatSTDPhase < 0.5 && d_fixedSTDPhase < 0.5)
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_VALID;
      }
      else
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
      }
    }
    else
    {
      if (!u_gap && f_adop < 10.0 && f_ratio > 1.8 && d_floatSTDPhase < 1.2 && d_fixedSTDPhase < 1.2 && d_fixedSTDCode < 3.0)
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_VALID;
      }
      else if (u_gap && f_adop < 10.0 && f_ratio > 2.0 && d_floatSTDPhase < 1.0 && d_fixedSTDPhase < 1.0 && d_fixedSTDCode < 2.0)
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_VALID;
      }
      else if (f_adop < 3.0 && f_ratio > 2.5 && d_floatSTDPhase < 0.6 && d_fixedSTDPhase < 0.6)
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_VALID;
      }
      else
      {
        pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
      }
    }
  }
  else
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] = RTK_FILTER_SOL_STAT_WARN;
  }

  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_ERR)
  {
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <ERROR> nb=%d gap=%d (>1/4)=%d opensky=%d ratio=%f adop=%f pdop=%f nsat=%d\n",
      w_nb, u_gap, w_numOverQuarter, b_openSky, f_ratio, f_adop, f_pdop, u_fixedTotalSatCount);
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <ERROR> | static:%d | fix STD Code:%f Phase:%f | Max Phase:%f | float STD Code:%f Phase:%f |\n",
      u_staticFlag, d_fixedSTDCode, d_fixedSTDPhase, d_fixedMaxPhase, d_floatSTDCode, d_floatSTDPhase);
    u_status = AMB_AR_FAIL_FixCheck;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_WARN)
  {
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <WARN!> nb=%d gap=%d (>1/4)=%d opensky=%d ratio=%f adop=%f pdop=%f nsat=%d\n",
      w_nb, u_gap, w_numOverQuarter, b_openSky, f_ratio, f_adop, f_pdop, u_fixedTotalSatCount);
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <WARN!> | static:%d | fix STD Code:%f Phase:%f | Max Phase:%f | float STD Code:%f Phase:%f |\n",
      u_staticFlag, d_fixedSTDCode, d_fixedSTDPhase, d_fixedMaxPhase, d_floatSTDCode, d_floatSTDPhase);
    u_status = AMB_AR_FAIL_FixCheck;
  }
  else if (pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_VALID)
  {
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <VALID> nb=%d gap=%d (>1/4)=%d opensky=%d ratio=%f adop=%f pdop=%f nsat=%d\n",
      w_nb, u_gap, w_numOverQuarter, b_openSky, f_ratio, f_adop, f_pdop, u_fixedTotalSatCount);
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <VALID> | static:%d | fix STD Code:%f Phase:%f | Max Phase:%f | float STD Code:%f Phase:%f |\n",
      u_staticFlag, d_fixedSTDCode, d_fixedSTDPhase, d_fixedMaxPhase, d_floatSTDCode, d_floatSTDPhase);
    u_status = AMB_AR_SUCCESS;
  }
  return u_status;
}

/**
 * @brief      check positive definite of fix solution's P mat module
 * @param[out] pz_curfixedAmbPool is input information for RTK ambiguity resolution
 * @param[out] pz_rtkFilterInfo is filter information
 * @return     status -- 0: Positive Definite; other: normal case
 */
uint8_t rtk_ambHoldPositiveDefiniteCheck(gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool, rtk_filterInfo_t* pz_rtkFilterInfo)
{
  int16_t w_i = 0;
  uint8_t u_status = 1;

  for (w_i = 0; w_i < pz_rtkFilterInfo->w_nmax; ++w_i)
  {
    if ((TRUE == pz_rtkFilterInfo->pq_paraValid[w_i] && pz_curfixedAmbPool->pd_q_fix[IUTM(w_i, w_i)] <= 0.0))
    {
      u_status = 0;
      break;
    }
  }

  if (!u_status)
  {
    LOGW(TAG_HPP, " -- hold Check : [Q] mat of fix solution is not Positive Definite, not update\n");
  }

  return u_status;
}
/**
 * @brief RTK zerocombine fix and hold
 * @param[in/out]  pz_curfixedAmbPool is input information for RTK ambiguity resolution
 * @param[out]     pz_preFixedAmbPool is result of amb resolution
 * @param[out]     pz_rtkFilterInfo is filter information
 * @return         void
 */
static void rtk_zerocombineAmbHold(gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_filterInfo_t* pz_rtkFilterInfo)
{

#if 1
  if (pz_preFixedAmbPool->w_continuefixedEpoch > 2 && pz_curfixedAmbPool->f_ratio > 3.0 &&
    pz_curfixedAmbPool->u_nb > 9)
  {
    LOGI(TAG_HPP, "fix and hold working, ns=%d, ratio=%6.2f\n", pz_curfixedAmbPool->u_nb, pz_curfixedAmbPool->f_ratio);
    pz_rtkFilterInfo->z_timeLastHold = pz_curfixedAmbPool->z_time;
    memcpy(pz_rtkFilterInfo->pd_X, pz_curfixedAmbPool->pd_x_fix, pz_rtkFilterInfo->w_nmax * sizeof(double));
    memcpy(pz_rtkFilterInfo->pd_Q, pz_curfixedAmbPool->pd_q_fix, NUTM(pz_rtkFilterInfo->w_nmax) * sizeof(double));
  }
#endif

  return;
}
/**
 * @brief RTK zero combine ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static uint8_t rtk_zeroCombineAmbResolution(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  rtk_filter_sol_t* pz_filterSol = NULL;

  pz_rtkAmbFixInputInfo->u_fixTypeIdx = RTK_FIXTYPE_UNCOMBINED;

  // zero combine
  pz_fixedAmbPool->u_ambfix_mode = GNSS_PART_LAMBDA_FULL;
  u_status = RTK_ZeroCombineAmbFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
  pz_rtkAmbFixInputInfo->pu_searchLoopNb[FOURTH_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

  //fixed solution availability check
  if (!u_status)
  {
    u_status = rtk_zeroCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
  }

  if (u_status && pz_fixedAmbPool->u_nb > 9)
  {
    LOGI(TAG_HPP, "second lambda search...\n");

    //second lambda search
    pz_fixedAmbPool->u_ambfix_mode = GNSS_PART_LAMBDA_QOS;
    u_status = RTK_ZeroCombineAmbFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    pz_rtkAmbFixInputInfo->pu_searchLoopNb[FIFTH_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

    //fixed solution availability check
    if (!u_status)
    {
      u_status = rtk_zeroCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    }
  }

  if (u_status && pz_fixedAmbPool->u_nb > 9)
  {
    LOGI(TAG_HPP, "third lambda search...\n");

    //third lambda search
    pz_fixedAmbPool->u_ambfix_mode = GNSS_PART_LAMBDA_GROUP;
    u_status = RTK_ZeroCombineAmbFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    pz_rtkAmbFixInputInfo->pu_searchLoopNb[SIXTH_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

    //fixed solution availability check
    if (!u_status)
    {
      u_status = rtk_zeroCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    }
  }

  if (u_status && pz_fixedAmbPool->u_nb > 9 && pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->u_threeFreqTag)
  {
    LOGI(TAG_HPP, "fourth lambda search...\n");

    //fourth lambda search
    pz_fixedAmbPool->u_ambfix_mode = GNSS_PART_LAMBDA_MP;
    u_status = RTK_ZeroCombineAmbFix(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    pz_rtkAmbFixInputInfo->pu_searchLoopNb[SEVENTH_SEARCH_LOOP] = pz_fixedAmbPool->u_nb;

    //fixed solution availability check
    if (!u_status)
    {
      u_status = rtk_zeroCombineSol_availabilityCheck(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    }
  }

  if (!u_status)
  {
    // save non-combine pos
    pz_filterSol = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->pz_filterSol;
    if (NULL != pz_filterSol && pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_VALID)
    {
      //update fixed amb pool, save fixed ambiguity
      rtk_ambUpdateFixedAmbPool(pz_fixedAmbPool, &pz_rtkAmbFixInputInfo->pz_preFixedAmbPool[pz_rtkAmbFixInputInfo->u_fixTypeIdx]);

      //fix and hold
      if (rtk_ambHoldPositiveDefiniteCheck(pz_fixedAmbPool, pz_rtkAmbFixInputInfo->pz_rtkFilterInfo) &&
        fabs(tm_GpsTimeDiff(&(pz_fixedAmbPool->z_time), &(pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_timeLastHold)) >= 5.0))
      {
        rtk_zerocombineAmbHold(pz_fixedAmbPool, pz_rtkAmbFixInputInfo->pz_preFixedAmbPool[pz_rtkAmbFixInputInfo->u_fixTypeIdx], pz_rtkAmbFixInputInfo->pz_rtkFilterInfo);
      }
    }
  }

  return u_status;
}

/**
 * @brief RTK ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
uint8_t RTK_AmbResolution(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;

  if (NULL != pz_fixedAmbPool && NULL != pz_rtkAmbFixInputInfo->pz_rtkFilterInfo)
  {
    // update time
    pz_fixedAmbPool->z_time = pz_rtkAmbFixInputInfo->pz_satSignalPool->z_tor;

    pz_rtkAmbFixInputInfo->z_fixUsedFreq = (ALGO_L1_FREQ | ALGO_L2_FREQ | ALGO_L5_FREQ);
    for (u_i = 0; u_i < MAX_SEARCH_LOOP; u_i++)
    {
      pz_rtkAmbFixInputInfo->pu_searchLoopNb[u_i] = 0;
    }

    // widelane AR
    rtk_widelaneAmbResolution(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);

    // zero combine AR
    u_status = rtk_zeroCombineAmbResolution(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    if (0 != u_status && TRUE == (pz_rtkAmbFixInputInfo->pz_rtkFilterInfo->z_opt.b_enableL1onlyFix))/* for L1 fix only*/
    {
      pz_rtkAmbFixInputInfo->z_fixUsedFreq = ALGO_L1_FREQ;
      for (u_i = 0; u_i < MAX_SEARCH_LOOP; u_i++)
      {
        pz_rtkAmbFixInputInfo->pu_searchLoopNb[u_i] = 0;
      }
      u_status = rtk_zeroCombineAmbResolution(pz_rtkAmbFixInputInfo, pz_fixedAmbPool);
    }

    //LOGW for realtime print out
    //LOGW is not encoded and it's exposed to clients, so could not be too comprehensive in writing
    //see ..._SEARCH_LOOP;
    LOGW(TAG_HPP, "-- nb --| %d %d %d | %d %d %d %d |\n", pz_rtkAmbFixInputInfo->pu_searchLoopNb[FIRST_SEARCH_LOOP],
      pz_rtkAmbFixInputInfo->pu_searchLoopNb[SECOND_SEARCH_LOOP], pz_rtkAmbFixInputInfo->pu_searchLoopNb[THIRD_SEARCH_LOOP],
      pz_rtkAmbFixInputInfo->pu_searchLoopNb[FOURTH_SEARCH_LOOP], pz_rtkAmbFixInputInfo->pu_searchLoopNb[FIFTH_SEARCH_LOOP],
      pz_rtkAmbFixInputInfo->pu_searchLoopNb[SIXTH_SEARCH_LOOP], pz_rtkAmbFixInputInfo->pu_searchLoopNb[SEVENTH_SEARCH_LOOP]);
  }

  return u_status;
}