#include "rtk_pre_fix.h"

//#define HPREF  // high precision reference coordinate

/**
* @brief select ref sat of prefix
* @param[in/out]  pz_satSigMeasCollect is the observation information
* @param[in]      pz_rtkCorrBlock is the OSR correction
* @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
* @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
* @param[in]     pz_preFixedAmbPool is the pool of prefix info
* @param[out]     u_targetConstellation is target Constellation
* @param[out]     z_freqType is frequence type
* @return        -1 fail; else success
*/
int16_t RTK_preFixRefSatSelect(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool,
  uint8_t u_targetConstellation, gnss_FreqType z_freqType)
{
  uint8_t u_satIndex = 0;
  uint8_t u_constellation = 0;
  int16_t s_refsatIndex = -1;
  uint16_t w_iSat = 0;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const GpsTime_t* curtime = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_fixedSignalAmb_t* pz_fixedAmbSet;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;

  // obtain time
  //curtime = &pz_satSigMeasCollect->z_tor;

  z_filterType = convertFreqAmb2FilterType(z_freqType);
  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
    if (u_targetConstellation != u_constellation)
    {
      continue;
    }

    pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
    if ( NULL == pz_fixedAmbSet || GNSS_AMB_FLAG_NONE == pz_fixedAmbSet->u_fix[z_freqType] ||
      GNSS_AMB_FLAG_FLOAT == pz_fixedAmbSet->u_fix[z_freqType])
    {
      continue;
    }

    pz_sigMeas = pz_satMeas->pz_signalMeas[z_freqType];
    if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) ||
      fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS || pz_sigMeas->f_cn0 < 35.0f)
    {
      continue;
    }

    pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
    if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
      || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
    {
      continue;
    }

    w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
    if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
      || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
    {
      continue;
    }

    w_x_id[0] = 0;
    w_x_id[1] = u_satIndex;
    w_x_id[2] = z_filterType;
    pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);

    if (NULL == pz_satPool)
    {
      continue;
    }

    if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < 1.0)
    {
      continue;
    }

    if (s_refsatIndex < 0 ||
      pz_satMeas->z_satPosVelClk.f_elevation > pz_satSigMeasCollect->pz_satMeas[s_refsatIndex]->z_satPosVelClk.f_elevation)
    {
      s_refsatIndex = u_satIndex;
    }
  }

  return s_refsatIndex;
}

/**
* @brief cal omc
* @param[in/out]  pz_satSigMeasCollect is the observation information
* @param[in]      pz_rtkCorrBlock is the OSR correction
* @param[in]     pd_siteCoor is the site coordinate
* @param[in]     pd_refunitVector is vector of los
* @param[in]     z_freqType is frequence type
* @param[out]     d_omc is omc
* @param[out]     d_wave is wave length
* @return        0 fail; 1 success
*/
uint8_t RTK_preFixCalOMC(const gnss_SatelliteMeas_t* pz_satMeas, const GnssCorrBlock_t* pz_rtkCorrBlock,
  double* pd_siteCoor, double* pd_refunitVector, gnss_FreqType z_freqType, double* d_omc, double* d_wave)
{
  uint8_t   u_i;
  uint16_t  w_iSat = 0;
  double    pd_satPosRot[3] = { 0.0 };
  double    pd_s2r[3] = { 0.0 };
  double    d_baseDist = 0.0;
  double    d_baseEarthRot = 0.0;
  double    d_wave_temp = 0.0;
  double    d_roverDist = 0.0;
  const double* pd_satPosClk = NULL;
  const double* pd_baseSatPosClk = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;

  // rover postion
  pd_satPosClk = pz_satMeas->z_satPosVelClk.d_satPosClk;
  gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
  d_roverDist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_refunitVector);

  pz_sigMeas = pz_satMeas->pz_signalMeas[z_freqType];
  pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);

  w_iSat = rtk_getIndexInCorrSatPosClk(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
  pd_baseSatPosClk = pz_rtkCorrBlock->pz_satPosClk[w_iSat].d_satPosClk;
  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_s2r[u_i] = pd_baseSatPosClk[u_i] - pz_rtkCorrBlock->d_refPosEcef[u_i];
  }
  d_baseDist = gnss_Norm(pd_s2r, 3);
  d_baseEarthRot = (pd_baseSatPosClk[0] * pz_rtkCorrBlock->d_refPosEcef[1] -
    pd_baseSatPosClk[1] * pz_rtkCorrBlock->d_refPosEcef[0]) * OMGE_GPS / CLIGHT;

  d_wave_temp = wavelength(pz_sigMeas->u_signal);
  if (d_wave_temp >= 1.0)
  {
    return 0;
  }
  // cal omc
  *d_omc = (pz_sigMeas->d_carrierPhase - pz_corrMeasReslut->d_carrierPhase) * d_wave_temp - (d_roverDist - (d_baseDist + d_baseEarthRot));
  *d_omc += (pd_satPosClk[3] - pd_baseSatPosClk[3]);

  if (d_wave) *d_wave = d_wave_temp;

  return 1;
}

/**
 * @brief the judge whether need to clean prefix info
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return        void
 */
void RTK_preFixCleanSat(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool,
  const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  char      c_sat[4] = "";
  uint8_t   u_signalIndex;
  uint8_t   u_satIndex = 0;
  uint16_t  w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;

  if (NULL == pz_preFixedAmbPool || 0 == pz_preFixedAmbPool->u_fixStatus)
  {
    return;
  }

  // loop freq
  for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
  {
    z_filterType = convertFreqAmb2FilterType(u_signalIndex);

    for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
      pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
      if (NULL == pz_satMeas || NULL == pz_fixedAmbSet)
      {
        continue;
      }

      if (pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_NONE ||
        pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_FLOAT)
      {
        continue;
      }

      w_x_id[0] = 0;
      w_x_id[1] = u_satIndex;
      w_x_id[2] = z_filterType;
      pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);

      if (NULL == pz_satPool)
      {
        continue;
      }

      if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < 1e-3)
      {
        /* detect cycle slip */
        pz_fixedAmbSet->u_fix[u_signalIndex] = GNSS_AMB_FLAG_NONE;
        pz_fixedAmbSet->u_continue_fix[u_signalIndex] = 0;
        pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex] = 0;
        satidx_SatString(u_satIndex, c_sat);
        LOGI(TAG_HPP, "clean prefix: %s L%d\n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1);
      }
    }
  }

  return;
}

/**
 * @brief the interface of clean RTK prefix
 * @param[in/out] pz_preFixedAmbPool is the pool of prefix info
 * @return        void
 */
void RTK_preFixCleanPool(gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool)
{
  uint8_t u_isys;
  uint8_t   u_satIndex = 0;
  uint8_t   u_signalIndex;
  gnss_fixedSignalAmb_t* pz_fixedAmbSet;

  /* clean ratio and nb */
  pz_preFixedAmbPool->f_ratio = 0.0f;
  pz_preFixedAmbPool->u_nb = 0;

  for (u_isys = C_GNSS_NONE; u_isys < C_GNSS_MAX; u_isys++)
  {
    pz_preFixedAmbPool->u_fixedns[u_isys] = 0;
  }

  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
    if (NULL == pz_fixedAmbSet)
    {
      continue;
    }

    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_fixedAmbSet->u_fix[u_signalIndex] = GNSS_AMB_FLAG_NONE;
      pz_fixedAmbSet->u_continue_fix[u_signalIndex] = 0;
      pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex] = 0.0f;
    }
  }

  return;
}

/**
 * @brief the judge whether need to do prefix
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_filterInfo the filter for the RTK algorithm
 * @param[in]     pw_PVAindex is index of pva
 * @return        0 fail; 1 success
 */
uint8_t RTK_preFixJudge(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool,
  const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const rtk_filterInfo_t* pz_filterInfo, int16_t pw_PVAindex[PVA_NUM])
{
  uint8_t u_i;
  double d_dt;
  double d_preFixAge = pz_filterInfo->z_opt.d_maxPrefixAge;
  const GpsTime_t* curtime = NULL;

  if (NULL == pz_preFixedAmbPool || 0 == pz_preFixedAmbPool->u_fixStatus)
  {
    return 0;
  }

  /* obtain time */
  curtime = &pz_satSigMeasCollect->z_tor;

  d_dt = tm_GpsTimeDiff(curtime, &pz_preFixedAmbPool->z_time);
  LOGI(TAG_HPP, "preFix dt=%8.1f, prenb=%d, preRatio=%.1f\n", d_dt, pz_preFixedAmbPool->u_nb, pz_preFixedAmbPool->f_ratio);

  /* dt check fail */
  if (d_dt > d_preFixAge || pz_preFixedAmbPool->u_nb < 6)
  {
    return 0;
  }

  // filter check
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  for (u_i = 0; u_i < PVA_NUM; u_i++)
  {
    if (pw_PVAindex[u_i] < 0)
    {
      return 0;
    }
  }

  return 1;
}

/**
 * @brief the prefix double difference algorithm, not used now
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @param[in]     pd_siteCoor is site coordinate
 * @param[in]     pw_PVAindex is index of pva
 * @param[in]     u_nv is number of dd
 * @param[in]     u_satInuse is number of satellite
 * @return         void
 */
void RTK_preFixDDSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool,
  double* pd_siteCoor, int16_t pw_PVAindex[PVA_NUM], uint8_t* u_nv, uint8_t* u_satInuse)
{
  char      c_sat[4] = "";
  uint8_t   u_i;
  uint8_t   u_isys;
  uint8_t   u_constellation = 0;
  uint8_t   u_signalIndex;
  int16_t   u_refsat_idx = 0;
  double    d_wave = 0.0;
  double    d_omc = 0.0;
  double    d_refomc = 0.0;
  double    d_curR;
  double    pd_unitVector[3] = { 0.0 };
  double    pd_refunitVector[3] = { 0.0 };
  uint16_t  w_iSat = 0;
  uint8_t   u_rejectFlag = 0;
  uint8_t   u_satIndex = 0;
  uint8_t   u_seqStatus = 0;
  SeqKalmanVar_t z_seqVar = { 0 };
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatelliteMeas_t* pz_refsatMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  double* pd_pX = NULL;
  double* pd_deltaX = NULL;
  double* pd_Q = NULL;
  uint8_t* u_satList;

  /* init sequential filter */
  u_seqStatus = hpp_seq_init(&z_seqVar, pz_RTKfilterInfo->w_nmax);
  if (0 != u_seqStatus)
  {
    return;
  }

  pd_pX = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
  pd_deltaX = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
  pd_Q = (double*)OS_MALLOC(NUTM(pz_RTKfilterInfo->w_nmax) * sizeof(double));
  u_satList = (uint8_t*)OS_MALLOC(ALL_GNSS_SYS_SV_NUMBER * sizeof(uint8_t));

  if (any_Ptrs_Null(4, pd_pX, pd_deltaX, pd_Q, u_satList))
  {
    if (NULL != pd_pX)
    {
      OS_FREE(pd_pX);
    }
    if (NULL != pd_deltaX)
    {
      OS_FREE(pd_deltaX);
    }
    if (NULL != pd_Q)
    {
      OS_FREE(pd_Q);
    }
    if (NULL != u_satList)
    {
      OS_FREE(u_satList);
    }
    return;
  }
  else
  {
    memcpy(pd_pX, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->w_nmax * sizeof(double));
    memcpy(pd_Q, pz_RTKfilterInfo->pd_Q, NUTM(pz_RTKfilterInfo->w_nmax) * sizeof(double));

    //init sat info
    for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
    {
      u_satList[u_i] = 0;
    }

    for (u_i = 0; u_i < pz_RTKfilterInfo->w_nmax; u_i++)
    {
      pd_deltaX[u_i] = 0;
    }
  }

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    
    if (pz_preFixedAmbPool->u_fixedns[u_isys] < 2)
    {
      continue;
    }

    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      // select refsat
      u_refsat_idx = RTK_preFixRefSatSelect(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
        pz_EKFstateRepPool, pz_preFixedAmbPool, u_isys, u_signalIndex);

      if (u_refsat_idx < 0)
      {
        continue;
      }

      pz_refsatMeas = pz_satSigMeasCollect->pz_satMeas[u_refsat_idx];

      if (!RTK_preFixCalOMC(pz_refsatMeas, pz_rtkCorrBlock, pd_siteCoor, pd_refunitVector, u_signalIndex, &d_refomc, NULL))
      {
        continue;
      }

      satidx_SatString((uint8_t)u_refsat_idx, c_sat);
      LOGI(TAG_HPP, "preRefSat=%s L%d\n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1);

      z_filterType = convertFreqAmb2FilterType(u_signalIndex);

      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        // init flag
        u_rejectFlag = 0;

        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
        pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
        if (NULL == pz_satMeas || NULL == pz_fixedAmbSet)
        {
          continue;
        }

        if (u_refsat_idx == u_satIndex)
        {
          continue;
        }
        u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
        if (u_constellation != u_isys || pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_NONE
          || pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_FLOAT)
        {
          continue;
        }

        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) ||
          fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
        if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
          || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
        if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
          || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
        {
          continue;
        }

        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = z_filterType;
        pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);

        if (NULL == pz_satPool)
        {
          continue;
        }

        if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < 1.0)
        {
          continue;
        }

        // no-ref info
        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];

        if (!RTK_preFixCalOMC(pz_satMeas, pz_rtkCorrBlock, pd_siteCoor, pd_unitVector, u_signalIndex, &d_omc, &d_wave))
        {
          continue;
        }

        // double difference omc
        d_omc = (d_refomc - d_omc) -
          (pz_preFixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->d_fixedValueRTK[u_signalIndex] - pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex]) * d_wave;

        /* test innovation */
        if (pz_RTKfilterInfo->z_opt.d_maxinno > 1e-3 && fabs(d_omc) > pz_RTKfilterInfo->z_opt.d_maxinno)
        {
          u_rejectFlag = 1;
        }
        else
        {
          d_curR = SQR(1.0e-4);

          hpp_seq_ClearH(&z_seqVar);
          /* H */
          for (u_i = 0; u_i < 3; ++u_i)
          {
            hpp_seq_AddH(pw_PVAindex[u_i], -pd_refunitVector[u_i] + pd_unitVector[u_i], &z_seqVar);
          }

          hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
          hpp_seq_PredictStep(pd_deltaX, pd_Q, &z_seqVar);
          hpp_seq_measUpdate(pz_RTKfilterInfo->pq_paraValid, pd_pX, pd_deltaX, pd_Q, &z_seqVar, 0.0);
          (*u_nv)++;

          u_satList[u_satIndex] = 1;
        }

        satidx_SatString(u_satIndex, c_sat);
        LOGI(TAG_HPP, "preFix=%s L%d v=%10.3f ddamb=%8.1f reject=%d\n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1,
          d_omc, (pz_preFixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->d_fixedValueRTK[u_signalIndex] - pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex]), u_rejectFlag);
      }
    }
  }

  hpp_seq_deinit(&z_seqVar);

  OS_FREE(pd_pX);
  OS_FREE(pd_deltaX);
  OS_FREE(pd_Q);
  OS_FREE(u_satList);
}


/**
 * @brief          RTK prefix solution availability check
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[in]      pz_fixedAmbPool is result of amb resolution
 * @param[in]      w_nv is num of prefix used amb
 * @return         void
 */
void rtk_preFixSol_availabilityCheck(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo,
  gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, uint16_t w_nv)
{
  BOOL b_openSky = FALSE;
  float f_pdop = 0.0;
  double d_floatSTDCode;
  double d_floatSTDPhase;
  double d_prefixSTDCode;
  double d_prefixSTDPhase;
  double d_prefixMaxPhase;
  rtk_filterInfo_t* pz_filterInfo;
  rtk_filter_sol_t* pz_filterSol;
  /* threshold define begin */
  const uint8_t d_minNv = 6; //min num of var
  const double d_pdopThres = 30.0; //max pdop
  const double d_STDphaseThres = 0.05; //max phase's STD(m)
  const double d_STDcodeThres = 3.0; //max code's STD(m) 
  const double d_STDcodeAmplifyThres = 1.8; //max code's STD magnification times
  const double d_pdopMultiStdThres = 1.5; //max possible error, get form pdop mulitiply post res STD (m)
  const double d_pdopMultiMaxThres = 2.0; //max possible error, get form pdop mulitiply post res MAX (m)
  /* threshold define end */

  if (pz_fixedAmbPool == NULL)
  {
    return;
  }
  f_pdop = pz_fixedAmbPool->f_pdop;
  pz_filterInfo = pz_rtkAmbFixInputInfo->pz_rtkFilterInfo;
  if (pz_filterInfo == NULL)
  {
    return;
  }
  pz_filterSol = pz_filterInfo->pz_filterSol;
  if (pz_filterSol == NULL)
  {
    return;
  }
  if (pz_rtkAmbFixInputInfo->pz_FilterObs == NULL)
  {
    return;
  }
  d_floatSTDCode = pz_rtkAmbFixInputInfo->pz_FilterObs->d_postCodeSTD;
  d_floatSTDPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->d_postPhaseSTD;
  d_prefixSTDCode = pz_rtkAmbFixInputInfo->pz_FilterObs->d_prefixCodeSTD;
  d_prefixSTDPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->d_prefixPhaseSTD;
  d_prefixMaxPhase = pz_rtkAmbFixInputInfo->pz_FilterObs->d_maxPhaseBias;

  if (f_pdop < FABS_ZEROS || w_nv < 1 ||
    d_prefixSTDCode < FABS_ZEROS || d_prefixSTDPhase < FABS_ZEROS || d_prefixMaxPhase < FABS_ZEROS)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_ERR;
  }
  else if (w_nv < d_minNv || f_pdop > d_pdopThres || d_prefixSTDPhase > d_STDphaseThres || !pz_filterInfo->u_prefixStatus)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  else if (d_prefixSTDCode > d_STDcodeThres && d_prefixSTDPhase / d_floatSTDPhase > d_STDcodeAmplifyThres)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  else if (f_pdop * d_prefixSTDPhase > d_pdopMultiStdThres || f_pdop * d_prefixMaxPhase > d_pdopMultiMaxThres)
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_WARN;
  }
  else
  {
    pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_VALID;
  }

  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] == RTK_FILTER_SOL_STAT_ERR)
  {
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <ERROR> nv=%d opensky=%d pdop=%f\n",
      w_nv, b_openSky, f_pdop);
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <ERROR> | prefix STD Code:%f Phase:%f | float STD Code:%f Phase:%f |\n",
      d_prefixSTDCode, d_prefixSTDPhase, d_floatSTDCode, d_floatSTDPhase);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] == RTK_FILTER_SOL_STAT_WARN)
  {
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <WARN!> nv=%d opensky=%d pdop=%f\n",
      w_nv, b_openSky, f_pdop);
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <WARN!> | prefix STD Code:%f Phase:%f | float STD Code:%f Phase:%f |\n",
      d_prefixSTDCode, d_prefixSTDPhase, d_floatSTDCode, d_floatSTDPhase);
  }
  if (pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] == RTK_FILTER_SOL_STAT_VALID)
  {
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <VALID> nv=%d opensky=%d pdop=%f\n",
      w_nv, b_openSky, f_pdop);
    LOGI(TAG_HPP, "** Non-combine Sol Check ** <VALID> | prefix STD Code:%f Phase:%f | float STD Code:%f Phase:%f |\n",
      d_prefixSTDCode, d_prefixSTDPhase, d_floatSTDCode, d_floatSTDPhase);
  }

}

/**
 * @brief the interface of print prefix information
 * @param[in]     u_refsatList is the reference satellite list
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[in]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @param[in]     pz_filterObs is filter observation
 * @return         void
 */
void rtk_preFixPrintSat(uint8_t* u_refsatList, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  char      c_sat[4] = "";
  uint8_t   u_nv = 0;
  uint8_t   u_isys;
  uint8_t   u_sysIndex = 0;
  uint8_t   u_satIndex = 0;
  uint8_t   u_constellation = 0;
  uint8_t u_size = 60;
  const gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  char satInfo[BUFF_SIZE] = { 0 };
  char* p_satInfo = satInfo;

  LOGI(TAG_HPP, " |=============L1============|==========L2===========|==========L5===========|\n");
  LOGI(TAG_HPP, "           cno      amb             cno       amb            cno       amb\n");

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    memset(satInfo, 0, BUFF_SIZE * sizeof(char));
    p_satInfo = &satInfo[0];

    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    if (pz_preFixedAmbPool->u_fixedns[u_isys] < 2)
    {
      continue;
    }

    u_sysIndex = u_isys * MAX_GNSS_SIGNAL_FREQ;
    if (u_refsatList[u_sysIndex] == 0 && (u_refsatList[u_sysIndex + 1] == 0) && (u_refsatList[u_sysIndex + 2] == 0))
    {
      continue;/* no ref sat */
    }

    if (u_refsatList[u_sysIndex] != 0)
    {
      satidx_SatString(u_refsatList[u_sysIndex], c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, " ref: %s ", c_sat);
    }
    else
    {
      p_satInfo += snprintf(p_satInfo, u_size, " ref:     ");
    }

    if (u_refsatList[u_sysIndex + 1] != 0)
    {
      satidx_SatString(u_refsatList[u_sysIndex + 1], c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, "                   | %s", c_sat);
    }
    else
    {
      p_satInfo += snprintf(p_satInfo, u_size, "                        ");
    }

    if (u_refsatList[u_sysIndex + 2] != 0)
    {
      satidx_SatString(u_refsatList[u_sysIndex + 2], c_sat);
      p_satInfo += snprintf(p_satInfo, u_size, "                   | %s", c_sat);
    }
    else
    {
      p_satInfo += snprintf(p_satInfo, u_size, "                        ");
    }
    LOGI(TAG_HPP, "%s\n", satInfo);

    for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
    {
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
      pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
      if (NULL == pz_satMeas || NULL == pz_fixedAmbSet)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      if (u_constellation != u_isys)
      {
        continue;
      }

      if (NULL != pz_filterObs->pz_SatFilterObs[u_satIndex])
      {
        if (pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[0] != 1 &&
          pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[1] != 1 &&
          pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[2] != 1)
        {
          continue;
        }

        memset(satInfo, 0, BUFF_SIZE * sizeof(char));
        p_satInfo = &satInfo[0];

        satidx_SatString(u_satIndex, c_sat);
        p_satInfo += snprintf(p_satInfo, u_size, "      %s", c_sat);
        /* L1 */
        if (pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[0] == 1)
        {
          p_satInfo += snprintf(p_satInfo, u_size, " %4.1f %8.1f", pz_satMeas->pz_signalMeas[0]->f_cn0,
            pz_preFixedAmbPool->pz_fixedAmbSet[u_refsatList[u_sysIndex]]->d_fixedValueRTK[0] - pz_fixedAmbSet->d_fixedValueRTK[0]);
        }
        else
        {
          p_satInfo += snprintf(p_satInfo, u_size, "              ");
        }

        /* L2 */
        if (pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[1] == 1)
        {
          p_satInfo += snprintf(p_satInfo, u_size, "            %4.1f  %8.1f", pz_satMeas->pz_signalMeas[1]->f_cn0,
            pz_preFixedAmbPool->pz_fixedAmbSet[u_refsatList[u_sysIndex + 1]]->d_fixedValueRTK[1] - pz_fixedAmbSet->d_fixedValueRTK[1]);
        }
        else
        {
          p_satInfo += snprintf(p_satInfo, u_size, "                          ");
        }

        /* L5 */
        if (pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[2] == 1)
        {
          p_satInfo += snprintf(p_satInfo, u_size, "           %4.1f  %8.1f", pz_satMeas->pz_signalMeas[2]->f_cn0,
            pz_preFixedAmbPool->pz_fixedAmbSet[u_refsatList[u_sysIndex + 2]]->d_fixedValueRTK[2] - pz_fixedAmbSet->d_fixedValueRTK[2]);
        }
        else
        {
          p_satInfo += snprintf(p_satInfo, u_size, "                         ");
        }
      }
      LOGI(TAG_HPP, "%s\n", satInfo);
    }
  }

  LOGI(TAG_HPP, " |=============L1============|==========L2===========|==========L5===========|\n");

  return;
}

/**
 * @brief the interface of RTK prefix algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @return         void
 */
uint8_t RTK_preFixSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs)
{
  char      c_sat[4] = "";
  uint8_t   u_i;
  uint8_t   u_nv = 0;
  uint8_t   u_isys;
  uint8_t   u_rejectFlag = 0;
  uint8_t   u_updateSuccess = 0;
  uint8_t   u_signalIndex;
  int16_t   u_refsat_idx = 0;
  uint8_t   u_constellation = 0;
  uint8_t   u_seqStatus = 0;
  uint8_t   u_satIndex = 0;
  uint8_t   u_satInuse = 0;
  uint8_t   u_outlierCount = 0;
  uint8_t   u_satList[ALL_GNSS_SYS_SV_NUMBER];
  uint8_t   u_refSatList[C_GNSS_MAX * MAX_GNSS_SIGNAL_FREQ] = { 0 };
  int16_t   pw_PVAindex[PVA_NUM] = { 0 };
  uint16_t  w_j = 0;
  uint16_t  w_iSat = 0;
  uint8_t   u_nb[C_GNSS_MAX] = { 0 };
  double    pd_siteCoor[3] = { 0.0 };
  double    pd_unitVector[3] = { 0.0 };
  double    pd_refunitVector[3] = { 0.0 };
  double    d_wave = 0.0;
  double    d_omc = 0.0;
  double    d_refomc = 0.0;
  double    d_curR;
  double    d_pdop = 0.0;
  double    pd_H_oneline[3] = { 0.0 };
  double    pd_HtH[16] = { 0.0 };
  double    pd_HtHinv[16] = { 0.0 };
  matrix_t  z_HtH;
  matrix_t  z_HtHinv;
  LogLevelEnum z_logLevel;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  SeqKalmanVar_t z_seqVar = { 0 };
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatelliteMeas_t* pz_refsatMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const gnss_fixedSignalAmb_t* pz_fixedAmbSet = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  double* pd_pX = NULL;
  double* pd_deltaX = NULL;
  double* pd_Q = NULL;

  // init pre fix status
  pz_RTKfilterInfo->u_prefixStatus = 0;

  /* clean cycle slip satellite */
  RTK_preFixCleanSat(pz_satSigMeasCollect, pz_preFixedAmbPool, pz_EKFstateRepPool);

  if (!RTK_preFixJudge(pz_satSigMeasCollect, pz_preFixedAmbPool, pz_EKFstateRepPool, pz_RTKfilterInfo, pw_PVAindex))
  {
    LOGI(TAG_HPP, "prefix fail!\n");
    return 0;
  }

  // init pre fix AmbPool status
  pz_preFixedAmbPool->f_pdop = 0.0f;

  /* init sequential filter */
  u_seqStatus = hpp_seq_init(&z_seqVar, pz_RTKfilterInfo->w_nmax);
  if (0 != u_seqStatus)
  {
    return 0;
  }

  pd_pX = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
  pd_deltaX = (double*)OS_MALLOC(pz_RTKfilterInfo->w_nmax * sizeof(double));
  pd_Q = (double*)OS_MALLOC(NUTM(pz_RTKfilterInfo->w_nmax) * sizeof(double));

  if (any_Ptrs_Null(3, pd_pX, pd_deltaX, pd_Q))
  {
    if (NULL != pd_pX)
    {
      OS_FREE(pd_pX);
    }
    if (NULL != pd_deltaX)
    {
      OS_FREE(pd_deltaX);
    }
    if (NULL != pd_Q)
    {
      OS_FREE(pd_Q);
    }
    return 0;
  }
  else
  {
    memcpy(pd_pX, pz_RTKfilterInfo->pd_X, pz_RTKfilterInfo->w_nmax * sizeof(double));
    memcpy(pd_Q, pz_RTKfilterInfo->pd_Q, NUTM(pz_RTKfilterInfo->w_nmax) * sizeof(double));

    //init sat info
    for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
    {
      u_satList[u_i] = 0;
    }

    for (u_i = 0; u_i < pz_RTKfilterInfo->w_nmax; u_i++)
    {
      pd_deltaX[u_i] = 0;
    }
  }

  /* site coordinate */
  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_siteCoor[u_i] = pd_pX[pw_PVAindex[u_i]];
  }

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    if (pz_preFixedAmbPool->u_fixedns[u_isys] < 2)
    {
      continue;
    }

    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      // select refsat
      u_refsat_idx = RTK_preFixRefSatSelect(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
        pz_EKFstateRepPool, pz_preFixedAmbPool, u_isys, u_signalIndex);

      if (u_refsat_idx < 0)
      {
        continue;
      }

      /* save ref sat */
      u_refSatList[u_isys * MAX_GNSS_SIGNAL_FREQ + u_signalIndex] = (uint8_t)u_refsat_idx;

      pz_refsatMeas = pz_satSigMeasCollect->pz_satMeas[u_refsat_idx];

      if (!RTK_preFixCalOMC(pz_refsatMeas, pz_rtkCorrBlock, pd_siteCoor, pd_refunitVector, u_signalIndex, &d_refomc, NULL))
      {
        continue;
      }

#if 0
      satidx_SatString(u_refsat_idx, c_sat);
      LOGI(TAG_HPP, "preRefSat=%s L%d CN0=%3.1f\n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1, pz_refsatMeas->pz_signalMeas[u_signalIndex]->f_cn0);
#endif

      z_filterType = convertFreqAmb2FilterType(u_signalIndex);

      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        // init flag
        u_rejectFlag = 0;

        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
        pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
        if (NULL == pz_satMeas || NULL == pz_fixedAmbSet)
        {
          continue;
        }

        if (u_refsat_idx == u_satIndex)
        {
          continue;
        }
        u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
        if (u_constellation != u_isys || pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_NONE
          || pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_FLOAT)
        {
          continue;
        }

        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) ||
          fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
        if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
          || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
        if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
          || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
        {
          continue;
        }

        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = z_filterType;
        pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);

        if (NULL == pz_satPool)
        {
          continue;
        }

        if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < 1.0)
        {
          continue;
        }

        // no-ref info
        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];

        if (!RTK_preFixCalOMC(pz_satMeas, pz_rtkCorrBlock, pd_siteCoor, pd_unitVector, u_signalIndex, &d_omc, &d_wave))
        {
          continue;
        }

        // double difference omc
        d_omc = (d_refomc - d_omc) -
          (pz_preFixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->d_fixedValueRTK[u_signalIndex] - pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex]) * d_wave;

        /* test innovation */
        if (pz_RTKfilterInfo->z_opt.d_maxinno > 1e-3 && fabs(d_omc) > pz_RTKfilterInfo->z_opt.d_maxinno)
        {
          u_rejectFlag = 1;
        }
        else
        {
          d_curR = SQR(1.0e-4);

          hpp_seq_ClearH(&z_seqVar);
          /* H */
          for (u_i = 0; u_i < 3; ++u_i)
          {
            hpp_seq_AddH(pw_PVAindex[u_i], -pd_refunitVector[u_i] + pd_unitVector[u_i], &z_seqVar);
          }

          /* for dop calcu */
          for (u_i = 0; u_i < 3; ++u_i)
          {
            pd_H_oneline[u_i] = pd_unitVector[u_i];
          }
          pd_HtH[0 * 4 + 0] += pd_H_oneline[0] * pd_H_oneline[0];
          pd_HtH[1 * 4 + 1] += pd_H_oneline[1] * pd_H_oneline[1];
          pd_HtH[2 * 4 + 2] += pd_H_oneline[2] * pd_H_oneline[2];
          pd_HtH[3 * 4 + 3] += 1 * 1;
          pd_HtH[1] = pd_HtH[4] = pd_HtH[4] + pd_H_oneline[0] * pd_H_oneline[1];
          pd_HtH[2] = pd_HtH[8] = pd_HtH[8] + pd_H_oneline[0] * pd_H_oneline[2];
          pd_HtH[6] = pd_HtH[9] = pd_HtH[9] + pd_H_oneline[1] * pd_H_oneline[2];
          pd_HtH[3] = pd_HtH[12] = pd_HtH[12] + pd_H_oneline[0] * 1;
          pd_HtH[7] = pd_HtH[13] = pd_HtH[13] + pd_H_oneline[1] * 1;
          pd_HtH[11] = pd_HtH[14] = pd_HtH[14] + pd_H_oneline[2] * 1;

          /* filter */
          hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
          hpp_seq_PredictStep(pd_deltaX, pd_Q, &z_seqVar);
          hpp_seq_measUpdate(pz_RTKfilterInfo->pq_paraValid, pd_pX, pd_deltaX, pd_Q, &z_seqVar, 0.0);
          u_nv++;

          u_nb[u_constellation]++;/* sys sat update*/

          u_satList[u_satIndex] = 1;

          /* update pre fix flag */
          if (NULL != pz_filterObs->pz_SatFilterObs[u_satIndex])
          {
            pz_filterObs->pz_SatFilterObs[u_satIndex]->pu_preFixUpdateFlag[u_signalIndex] = 1;
          }
          if (NULL != pz_filterObs->pz_SatFilterObs[u_refsat_idx])
          {
            pz_filterObs->pz_SatFilterObs[u_refsat_idx]->pu_preFixUpdateFlag[u_signalIndex] = 1;
          }

        }

        if (fabs(d_omc) > 1.0)
        {
          u_outlierCount++;
        }
#if 1
        satidx_SatString(u_satIndex, c_sat);
        LOGD(TAG_HPP, "preFix=%s L%d CN0=%3.1f  v=%8.3f ddamb=%8.1f reject=%d\n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1, pz_sigMeas->f_cn0,
          d_omc, (pz_preFixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->d_fixedValueRTK[u_signalIndex] - pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex]), u_rejectFlag);
#endif
      }
    }
  }
  u_nb[C_GNSS_BDS3] = u_nb[C_GNSS_BDS2] + u_nb[C_GNSS_BDS3];
  u_nb[C_GNSS_BDS2] = 0;

  // print prefix info 
  z_logLevel = log_GetLogLevel();
  if (z_logLevel >= LOG_LEVEL_I && u_nv > 3)
  {
    LOGI(TAG_HPP, " prefix start updating...\n");
    rtk_preFixPrintSat(u_refSatList, pz_satSigMeasCollect, pz_RTKfilterInfo, pz_preFixedAmbPool, pz_filterObs);
    LOGI(TAG_HPP, " prefix updating success...\n");
  }

  //calculate used sat count
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    if (u_satList[u_i] == 1)
    {
      u_satInuse++;
    }
  }

  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; u_i++)
  {
    if (u_nb[u_i] > 3)
    {
      u_updateSuccess = 1;
      break;
    }
  }

  u_rejectFlag = 0;
  if (u_outlierCount > u_nv * 0.6 || !u_updateSuccess)
  {
    u_rejectFlag = 1;
    //RTK_preFixCleanPool(pz_preFixedAmbPool);
    LOGI(TAG_HPP, "preFix clean: nreject=%d, nv=%d, %d,%d,%d,%d,%d\n",
      u_outlierCount, u_nv, u_nb[C_GNSS_GPS], u_nb[C_GNSS_GLO], u_nb[C_GNSS_BDS3], u_nb[C_GNSS_GAL], u_nb[C_GNSS_QZS]);
  }

  if (u_nv >= 4 && u_satInuse >= 4 && !u_rejectFlag)
  {
    // update pre fix status
    pz_RTKfilterInfo->u_prefixStatus = 1;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_PREFIX][u_i] = pd_pX[pw_PVAindex[u_i]];
    }
    pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] = RTK_FILTER_SOL_STAT_SAVE;
    LOGI(TAG_HPP, "preFix output:%.3f,%.3f,%.3f, nv=%d, satnum=%d\n",
      pd_pX[pw_PVAindex[0]], pd_pX[pw_PVAindex[1]], pd_pX[pw_PVAindex[2]], u_nv, u_satInuse);

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
        pz_preFixedAmbPool->f_pdop = (float)sqrt(d_pdop);
      }
    }

  }
  hpp_seq_deinit(&z_seqVar);

  if (pz_RTKfilterInfo->u_prefixStatus == 1)
  {
    float f_factor = 4.42f;
    double* d_QEcefPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
    double* d_QenuPos = OS_MALLOC_FAST(sizeof(double) * 3 * 3);
    double d_lla[3] = {0.0};
    pz_RTKfilterInfo->pz_integ->z_rtk_feature.f_previous_ratio = pz_preFixedAmbPool->f_ratio;
    pz_RTKfilterInfo->pz_integ->z_rtk_feature.u_previous_fix_sat = pz_preFixedAmbPool->u_fixedTotalSatCount;
    pz_RTKfilterInfo->pz_integ->z_rtk_feature.f_prefix_age = (float)tm_GpsTimeDiff(&pz_filterObs->z_tor, &pz_preFixedAmbPool->z_time);
    pz_RTKfilterInfo->pz_integ->z_rtk_feature.u_prefix_sat_number = u_satInuse;
    pz_RTKfilterInfo->pz_integ->z_rtk_feature.u_prefix_sig_number = u_nv;
    pz_RTKfilterInfo->pz_integ->z_rtk_feature.f_prefix_pdop = pz_preFixedAmbPool->f_pdop;
    if(getPVAQ(pd_Q, pw_PVAindex, 3, d_QEcefPos))
    {
      gnss_Ecef2Lla(pd_siteCoor, d_lla);
      gnss_CovEcef2Enu(d_lla, d_QEcefPos, d_QenuPos);
      pz_RTKfilterInfo->pz_integ->z_rtk_feature.f_prefix_ls_std[0] = (float)sqrt(d_QenuPos[0] + d_QenuPos[4]) * f_factor;
      pz_RTKfilterInfo->pz_integ->z_rtk_feature.f_prefix_ls_std[1] = (float)sqrt(d_QenuPos[8]) * f_factor;
    }
    OS_FREE(d_QEcefPos);
    OS_FREE(d_QenuPos);
  }


  OS_FREE(pd_pX);
  OS_FREE(pd_deltaX);
  OS_FREE(pd_Q);

  // avoid Partial read warning: u_nb[0] is never read.
  for (u_i = C_GNSS_NONE; u_i < C_GNSS_MAX; u_i++)
  {
    LOGD(TAG_HPP, "prefix u_nb[%d] = %d\n", u_i, u_nb[u_i]);
  }

  return u_nv;
}

/**
 * @brief the interface of clean prefix reject label
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @return         void
 */
void RTK_preFixCleanRejectLabel(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t   u_satIndex = 0;
  uint8_t   u_signalIndex;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;

  for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }

    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (1 == pz_satMeas->u_rejectLabel[u_signalIndex])
      {
        pz_satMeas->u_rejectLabel[u_signalIndex] = 0;
      }
    }
  }
}
/**
 * @brief the interface of RTK prefix algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @return         void
 */
void RTK_preFixHalfCycleDetect(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool)
{
  char      c_sat[4] = "";
  uint8_t   u_i;
  uint8_t   u_isys;
  uint8_t   u_signalIndex;
  int16_t   u_refsat_idx = 0;
  uint8_t   u_constellation = 0;
  uint8_t   u_satIndex = 0;
  uint8_t   u_satNormal = 0;
  uint8_t   u_satAbnormal = 0;
  uint8_t   u_cn038 = 0;
  uint16_t  w_iSat = 0;
  double    pd_siteCoor[3] = { 0.0 };
  double    pd_unitVector[3] = { 0.0 };
  double    pd_refunitVector[3] = { 0.0 };
  double    d_wave = 0.0;
  double    d_omc = 0.0;
  double    d_refomc = 0.0;
  double    d_bias = 0.0;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatelliteMeas_t* pz_refsatMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;

  if (pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] <= RTK_FILTER_SOL_STAT_ERR)
  {
    return;
  }

  /* site coordinate */
  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_siteCoor[u_i] = pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_PREFIX][u_i];
  }

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    
    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      // select refsat
      u_refsat_idx = RTK_preFixRefSatSelect(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
        pz_EKFstateRepPool, pz_preFixedAmbPool, u_isys, u_signalIndex);

      if (u_refsat_idx < 0)
      {
        continue;
      }

      pz_refsatMeas = pz_satSigMeasCollect->pz_satMeas[u_refsat_idx];

      if (!RTK_preFixCalOMC(pz_refsatMeas, pz_rtkCorrBlock, pd_siteCoor, pd_refunitVector, u_signalIndex, &d_refomc, NULL))
      {
        continue;
      }

      z_filterType = convertFreqAmb2FilterType(u_signalIndex);

      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
        if (NULL == pz_satMeas || u_refsat_idx == u_satIndex)
        {
          continue;
        }
        u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
        if (u_constellation != u_isys)
        {
          continue;
        }

        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) ||
          fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
        if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
          || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
        if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
          || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
        {
          continue;
        }

        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = z_filterType;
        pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);

        if (NULL == pz_satPool)
        {
          continue;
        }

        // no-ref info
        if (!RTK_preFixCalOMC(pz_satMeas, pz_rtkCorrBlock, pd_siteCoor, pd_unitVector, u_signalIndex, &d_omc, &d_wave))
        {
          continue;
        }

        // double difference omc
        d_omc = (d_refomc - d_omc) / d_wave;
        d_bias = d_omc - round(d_omc);

        u_satNormal++;

        if (fabs(d_bias) > 0.4)
        {
          if ((pz_sigMeas->u_LLI & 0x2) != 0x2) // half cycle do not join in count
          {
            u_satAbnormal++;
            pz_satMeas->u_rejectLabel[u_signalIndex] = 1;
          }

          if (pz_sigMeas->f_cn0 >= 38.0f)
          {
            u_cn038++; //good quality count
          }
          satidx_SatString(u_satIndex, c_sat);
          LOGI(TAG_HPP, " detect half cycle: %s L%d CN0=%3.1f LLI=%d v=%8.3f dot=%.2f \n", c_sat,
            u_signalIndex > 1 ? 5 : u_signalIndex + 1, pz_sigMeas->f_cn0, pz_sigMeas->u_LLI, d_omc, d_bias);
        }
      }
    }
  }

  if (u_satAbnormal > 0)
  {
    LOGI(TAG_HPP, " abnormal sat=%d, total=%d\n", u_satAbnormal, u_satNormal);

    if ((u_satAbnormal >= 4 && u_satAbnormal > 0.2 * u_satNormal) ||
      (u_satAbnormal >= 5 && u_cn038  > 0.6 * u_satAbnormal))
    {
      //pz_preFixedAmbPool->u_nb = 0;
      LOGI(TAG_HPP, " abnormal prefix and init\n");
      RTK_preFixCleanRejectLabel(pz_satSigMeasCollect);
    }
  }

  return;
}

/**
 * @brief the interface of RTK prefix amb calculate
 * @param[in]     u_nv is number of prefix satellite
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[in]     pz_rtkCorrBlock is the OSR correction
 * @param[in]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @return         void
 */
void rtk_PreFixAmbCalculate(uint8_t u_nv, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool)
{
  char      c_sat[4] = "";
  uint8_t   u_i;
  uint8_t   u_isys;
  uint8_t   u_signalIndex;
  int16_t   u_refsat_idx = 0;
  uint8_t   u_constellation = 0;
  uint8_t   u_satIndex = 0;
  uint16_t  w_iSat = 0;
  double    pd_siteCoor[3] = { 0.0 };
  double    pd_unitVector[3] = { 0.0 };
  double    pd_refunitVector[3] = { 0.0 };
  double    d_wave = 0.0;
  double    d_omc = 0.0;
  double    d_refomc = 0.0;
  double    d_bias = 0.0;
  double    d_timeDiff = 0.0;
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  uint16_t w_x_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SatelliteMeas_t* pz_refsatMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const GnssMeas_t* pz_corrMeasReslut = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  gnss_fixedSignalAmb_t* pz_fixedAmbSet;

  if (NULL == pz_satSigMeasCollect || NULL == pz_preFixedAmbPool)
  {
    return;
  }
  d_timeDiff = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_preFixedAmbPool->z_time);
  if (fabs(d_timeDiff) < FABS_ZEROS)
  {
    return;
  }

  if (pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_FIX] == RTK_FILTER_SOL_STAT_VALID)
  {
    /* get fix site coordinate */
    for (u_i = 0; u_i < 3; u_i++)
    {
      pd_siteCoor[u_i] = pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_FIX][u_i];
    }
  }
  else if (pz_RTKfilterInfo->pz_filterSol->pu_posValid[RTK_FILTER_POS_PREFIX] >= RTK_FILTER_SOL_STAT_VALID)
  {
    /* get prefix site coordinate */
    for (u_i = 0; u_i < 3; u_i++)
    {
      pd_siteCoor[u_i] = pz_RTKfilterInfo->pz_filterSol->pd_filterPos[RTK_FILTER_POS_PREFIX][u_i];
    }
  }
  else
  {
    return;
  }
#ifdef HPREF
  /* high precision reference coordinate */
  /*pd_siteCoor[0] = -2837282.923;
  pd_siteCoor[1] = 4664340.437;
  pd_siteCoor[2] = 3286607.496;*/
#endif // HPREF

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      continue;
    }
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }

    // loop freq
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      // select refsat
      u_refsat_idx = RTK_preFixRefSatSelect(pz_satSigMeasCollect, pz_rtkCorrBlock, pz_RTKfilterInfo,
        pz_EKFstateRepPool, pz_preFixedAmbPool, u_isys, u_signalIndex);

      if (u_refsat_idx < 0)
      {
        continue;
      }

      pz_refsatMeas = pz_satSigMeasCollect->pz_satMeas[u_refsat_idx];

      if (!RTK_preFixCalOMC(pz_refsatMeas, pz_rtkCorrBlock, pd_siteCoor, pd_refunitVector, u_signalIndex, &d_refomc, NULL))
      {
        continue;
      }

      z_filterType = convertFreqAmb2FilterType(u_signalIndex);

      for (u_satIndex = 0; u_satIndex < ALL_GNSS_SYS_SV_NUMBER; u_satIndex++)
      {
        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_satIndex];
        if (NULL == pz_satMeas || u_refsat_idx == u_satIndex)
        {
          continue;
        }
        u_constellation = gnss_getConstellationEnumValueInLoop(pz_RTKfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
        if (u_constellation != u_isys)
        {
          continue;
        }

        pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
        if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || pz_sigMeas->f_cn0 < 35 || pz_satMeas->z_satPosVelClk.f_elevation < 35.0 * DEG2RAD ||
          fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS || (pz_sigMeas->u_LLI & 0x3) != 0)
        {
          continue;
        }

        pz_corrMeasReslut = getRtkMeasCorr(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_rtkCorrBlock);
        if (NULL == pz_corrMeasReslut || !(pz_corrMeasReslut->z_measStatusFlag.b_valid)
          || fabs(pz_corrMeasReslut->d_carrierPhase) < FABS_ZEROS)
        {
          continue;
        }

        w_iSat = rtk_getIndexInCorrSatPosClk(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_rtkCorrBlock->pz_satPosClk, pz_rtkCorrBlock->w_satCount);
        if (w_iSat >= (pz_rtkCorrBlock->w_satCount) || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) < 0
          || (pz_rtkCorrBlock->pz_satPosClk[w_iSat].q_iode) != (pz_satMeas->z_satPosVelClk.q_iode))
        {
          continue;
        }

        w_x_id[0] = 0;
        w_x_id[1] = u_satIndex;
        w_x_id[2] = z_filterType;
        pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);

        if (NULL == pz_satPool)
        {
          continue;
        }

        if (tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime) < 1.0)
        {
          continue;
        }

        // no-ref info
        if (!RTK_preFixCalOMC(pz_satMeas, pz_rtkCorrBlock, pd_siteCoor, pd_unitVector, u_signalIndex, &d_omc, &d_wave))
        {
          continue;
        }

        // double difference omc
        d_omc = (d_refomc - d_omc) / d_wave;
        d_bias = d_omc - round(d_omc);

#ifdef HPREF
        //satidx_SatString(u_satIndex, c_sat);
        //LOGI(TAG_HPP, " ddamb info: %s L%d %10.3f %8.2f \n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1, d_omc, pz_sigMeas->f_cn0);
#else
        pz_fixedAmbSet = pz_preFixedAmbPool->pz_fixedAmbSet[u_satIndex];
        if (NULL != pz_fixedAmbSet && 
          (pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_FLOAT || pz_fixedAmbSet->u_fix[u_signalIndex] == GNSS_AMB_FLAG_NONE))
        {
          if (fabs(d_bias) < 0.1)
          {
            pz_fixedAmbSet->d_fixedValueRTK[u_signalIndex] = pz_preFixedAmbPool->pz_fixedAmbSet[u_refsat_idx]->d_fixedValueRTK[u_signalIndex] - round(d_omc);
            pz_fixedAmbSet->u_fix[u_signalIndex] = GNSS_AMB_FLAG_PREFIXED;

            satidx_SatString(u_satIndex, c_sat);
            LOGI(TAG_HPP, "joined in fix: %s L%d %7.3f %8.2f  %10.6f\n", c_sat, u_signalIndex > 1 ? 5 : u_signalIndex + 1, d_bias, round(d_omc),
              pz_RTKfilterInfo->pd_Q[IUTM(pz_satPool->w_index, pz_satPool->w_index)]);
          }
        }
#endif
      }
    }
  }

  return;
}