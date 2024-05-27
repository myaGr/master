#include "gnss_prep.h"
#include "ppp_filter_sol.h"
#include "gnss_common.h"
#include "seq_kalman.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "cmn_utils.h"
#include "cmn_CSmask_combine.h"
#include "ppp_integrity.h"

/**
 * @brief variance of satellite ephemeris
 * @param[in]  f_ure
 * @return     variance
 */
double varianceEphSSR(float f_ure)
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
 * @brief calculate satellite signal carrier initial ambiguity
 * @param pz_satSigMeasCollect all satellite observation information coolection
 * @param pz_SsrLocBlk SSR product
 * @param pz_PPPfilterInfo  filter for the PPP-RTK algorithm
 * @param pz_EKFstateRepPool the pool of EKF state represent
 * @param pz_satMeas satellite observation information
 * @param pz_sigMeas signal observation information
 * @param d_wave signal wavelength
 * @return
 */
double PPP_calInitAMB(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
                      const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
                      const gnss_SatelliteMeas_t* pz_satMeas, const gnss_SignalMeas_t* pz_sigMeas)
{
  gnss_ConstellationType u_constellation = pz_sigMeas->u_constellation;
  double pd_satPosRot[3] = {0.0};
  double pd_siteCoor[3] = {0.0};
  double pd_unitVector[3] = {0.0};
  double d_init_value = 0.0;
  double d_prAmb = 0.0;
  double d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
  double d_wave = wavelength(pz_sigMeas->u_signal);
  const gnss_satSsrLos_t* pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
  if ((d_wave < 1.0) && (pz_satLos != NULL) && (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH))
  {

    for (int i = 0; i < 3; ++i)
    {
      pd_siteCoor[i] = pz_satSigMeasCollect->z_positionFix.d_xyz[i];
    }
    const double* pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    double d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX];
    getRcvClkParaIndex(pz_EKFstateRepPool, pw_rcvClkIndex);
    gnss_FreqType z_freqTypee = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
    int16_t w_rcvClkFilterIndex = pw_rcvClkIndex[u_constellation];
    int16_t w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqTypee * C_GNSS_MAX + u_constellation];
    if (INVALID_INDEX != w_rcvClkFilterIndex && INVALID_INDEX != w_rcvDcbFilterIndex)
    {
      d_init_value =
              (pz_sigMeas->d_carrierPhase) - (d_dist + pz_PPPfilterInfo->pd_X[w_rcvClkFilterIndex] - pd_satPosClk[3] +
                                              pz_satMeas->d_Tropo - d_beta * pz_satMeas->d_Iono) / d_wave;
      if (C_GNSS_FREQ_TYPE_L1 != z_freqTypee)
      {
        d_init_value -= pz_PPPfilterInfo->pd_X[w_rcvDcbFilterIndex] / d_wave;
      }
      d_prAmb = pz_sigMeas->d_carrierPhase - (pz_sigMeas->d_pseudoRange) / d_wave +
          (2.0 * d_beta * pz_satMeas->d_Iono) / d_wave;
      if (fabs(pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][z_freqTypee]) > GNSS_PHASE_CODE_CLK_MAX_BIAS)
      {
        d_prAmb -= pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][z_freqTypee];
      }
      if (fabs(d_init_value - d_prAmb) > 500.0 && fabs(pz_sigMeas->d_pseudoRange) > 1e+7)
      {
          d_init_value = 0.0;
      }
    }
  }
  if ((0.0 == d_init_value) && (pz_sigMeas->z_measStatusFlag.b_prValid) && (pz_sigMeas->d_pseudoRange > FABS_ZEROS))
  {
    d_init_value = pz_sigMeas->d_carrierPhase - (pz_sigMeas->d_pseudoRange) / d_wave +
                   (2.0 * d_beta * pz_satMeas->d_Iono) / d_wave;
  }
  return d_init_value;
}

/**
 * @brief add and time update the PVA parameter in the EKF
 * @param[in]  z_gpsTime is the observation time for current epoch
 * @param[in]  pd_siteCoor is the initilization value of site coordination
 * @param[in]  pd_siteVel is the initilization value of site velocity
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @return     1 represent success and 0 represent failure
 */
uint8_t PPP_addSiteParaPVA(GpsTime_t z_gpsTime, const double pd_siteCoor[3], const double pd_siteVel[3],
                           ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_filterInitInfo_t z_initInfo;
  int16_t pw_pvaIndex[PVA_NUM] = { -1 };
  double d_alph = 0.2;
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    d_alph = 0.01;
  }
  z_initInfo.z_paraTime = z_gpsTime;
  //initialize position,velocity and acceleration for first epoch
  if (KF_INIT == (pz_PPPfilterInfo->z_kfStatus) || KF_RESET == (pz_PPPfilterInfo->z_kfStatus)
    || tm_GpsTimeDiff(&z_gpsTime, &(pz_EKFstateRepPool->z_gpsTime))>3.0)
  {
    z_initInfo.u_resetPara = 1;
    //initialize position
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = pd_siteCoor[u_i];
      z_initInfo.d_sigma0 = POS_SIGMA;
      z_initInfo.d_noise = POS_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_POS;
      addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    //initialize velocity
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = pd_siteVel[u_i];
      z_initInfo.d_sigma0 = VEL_SIGMA;
      z_initInfo.d_noise = VEL_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_VEL;
      addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
    //initialize acceleration
    for (u_i = 0; u_i < 3; ++u_i)
    {
      z_initInfo.d_initValue = 0.0;
      z_initInfo.d_sigma0 = ACC_SIGMA;
      z_initInfo.d_noise = ACC_SIGMA;
      w_id[0] = 0;
      w_id[1] = u_i;
      w_id[2] = GNSS_FILTER_STATE_ACC;
      addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
  }
  else
  {
    dynamicStateTransfer(pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_EKFstateRepPool, z_gpsTime, d_alph, 1.0,
                         0);
    getPvaParaIndex(pz_EKFstateRepPool, pw_pvaIndex);
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
        pz_PPPfilterInfo->pq_paraValid[pw_pvaIndex[u_i]] = TRUE;
    }
  }
  return u_status;
}
/**
 * @brief get the satellite number used by the initialition value of the receiver clock parameter
 * @param[in]  q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]  z_targetSys is the target constellation
 * @param[in]  z_targetFreq is the target frequency
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @return     the satellite number used by the initialition value of the receiver clock parameter
 */
uint32_t PPP_getSatNumUsedByInitRcvClkCal(BOOL q_isSperateBDS2And3, gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq,
  const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  uint32_t q_satNum = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas|| !pz_sigMeas->z_measStatusFlag.b_prValid)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
      if (u_constellation != z_targetSys)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != z_targetFreq)
      {
        continue;
      }
      if ((pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      ++q_satNum;
    }
  }
  return q_satNum;
}
/**
 * @brief calculate the initialition value of the receiver clock parameter
 * @param[in]      q_isSperateBDS2And3  whether BDS2 and BDS3 are separate
 * @param[in]      z_targetSys is the target constellation
 * @param[in]      z_targetFreq is the target frequency
 * @param[in]      pd_siteCoor is the initilization value of site coordination
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[out]     pd_initRcvClk is the initialition value of the receiver clock parameter
 * @param[out]     pq_obsNum is the observations number for calculating receiver clock value
 * @return         1 represent success and 0 represent failure
 */
uint8_t ppp_CalcInitRcvClk(BOOL q_isSperateBDS2And3, gnss_ConstellationType z_targetSys, gnss_FreqType z_targetFreq,
  const double pd_siteCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, double* pd_initRcvClk, uint32_t* pq_obsNum)
{
  uint8_t u_status = 0;
  uint32_t q_satIndex = 0;
  uint32_t* pq_satIndexSet = NULL;
  uint32_t q_satNum = 0;
  uint32_t q_num = 0;
  uint32_t q_i = 0;
  uint8_t u_signalIndex = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t* pu_signalIndexSet = NULL;
  uint8_t u_i = 0;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  double* pd_rcvClkSet = NULL;
  const double* pd_satPosClk = NULL;
  double pd_satPosRot[3] = { 0.0 };
  double pd_deltaPos[3] = { 0.0 };
  double pd_unitVector[3] = {0.0};
  double d_dist = 0.0;
  double d_beta = 0.0;
  q_satNum = PPP_getSatNumUsedByInitRcvClkCal(q_isSperateBDS2And3, z_targetSys, z_targetFreq, pz_satSigMeasCollect, pz_SsrLocBlk);
  *pq_obsNum = q_satNum;
  if (q_satNum <= 2)
  {
    return 0;
  }
  pd_rcvClkSet = (double*)OS_MALLOC_FAST(q_satNum * sizeof(double));
  memset(pd_rcvClkSet, 0, q_satNum * sizeof(double));
  pq_satIndexSet = (uint32_t*)OS_MALLOC_FAST(q_satNum * sizeof(uint32_t));
  memset(pq_satIndexSet, 0, q_satNum * sizeof(uint32_t));
  pu_signalIndexSet = (uint8_t*)OS_MALLOC_FAST(q_satNum * sizeof(uint8_t));
  memset(pu_signalIndexSet, 0, q_satNum * sizeof(uint8_t));
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    const gnss_satSsrLos_t* pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    /* ssr brdc satellite posclk */
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH))
    {
      continue;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(q_isSperateBDS2And3, pz_sigMeas->u_constellation);
      if (u_constellation != z_targetSys)
      {
        continue;
      }
      if (!pz_sigMeas->z_measStatusFlag.b_prValid)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != z_targetFreq)
      {
        continue;
      }
      if ((pz_sigMeas->d_pseudoRange) < 1.0e-2)
      {
        continue;
      }
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_deltaPos[u_i] = pd_satPosRot[u_i] - pd_siteCoor[u_i];
      }
      d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
      pd_rcvClkSet[q_num] = (pz_sigMeas->d_pseudoRange) - (d_dist - pd_satPosClk[3] +
                                                           pz_satMeas->d_Tropo +
                                                           d_beta * pz_satMeas->d_Iono);
      pq_satIndexSet[q_num] = q_satIndex;
      pu_signalIndexSet[q_num] = u_signalIndex;
      ++q_num;
    }
  }
  // TODO(houxiaowei): check clk std with different frqs
  u_status = gnss_ascSortMedianDouble(pd_rcvClkSet, q_num, pd_initRcvClk);
  for (q_i = 0; q_i < q_num; ++q_i)
  {
      if (fabs(pd_rcvClkSet[q_i] - (*pd_initRcvClk)) > 1000.0)
      {
          pz_satSigMeasCollect->pz_satMeas[pq_satIndexSet[q_i]]->pz_signalMeas[pu_signalIndexSet[q_i]]->z_measStatusFlag.b_prValid = 0;
      }
  }
  if (NULL != pd_rcvClkSet)
  {
    OS_FREE(pd_rcvClkSet);
  }
  if (NULL != pq_satIndexSet)
  {
      OS_FREE(pq_satIndexSet);
  }
  if (NULL != pu_signalIndexSet)
  {
      OS_FREE(pu_signalIndexSet);
  }
  return u_status;
}
/**
 * @brief add and time update the receiver clock parameter in the EKF
 * @param[in]     pd_siteCoor is the initilization value of site coordination
 * @param[in/out] pz_satSigMeasCollect is the observation information
 * @param[in]     pz_SsrLocBlk is the SSR product
 * @param[out]    pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out]    pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void PPP_addRcvClkPara(const double pd_siteCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  gnss_filterInitInfo_t z_initInfo;
  double pd_rcvClkSet[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0.0 };
  uint8_t pu_clkStatus[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  uint32_t pq_obsNum[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  BOOL q_isSeparateBDS2And3 = FALSE;
  for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
  {
    q_isSeparateBDS2And3 = gnss_isSeparateBDS2And3ForSatSignalCollect(u_j, pz_satSigMeasCollect);
    for (u_i = 0; u_i < C_GNSS_MAX; ++u_i)
    {
      if (TRUE == gnss_WhetherSkipBDS2VarInLoop(q_isSeparateBDS2And3, u_i))
      {
        continue;
      }
      pu_clkStatus[u_j * C_GNSS_MAX + u_i] = ppp_CalcInitRcvClk(q_isSeparateBDS2And3, (gnss_ConstellationType)u_i, (gnss_FreqType)u_j,
        pd_siteCoor, pz_satSigMeasCollect, pz_SsrLocBlk, &(pd_rcvClkSet[u_j * C_GNSS_MAX + u_i]), &(pq_obsNum[u_j * C_GNSS_MAX + u_i]));
    }
    if (FALSE == q_isSeparateBDS2And3)
    {
      pu_clkStatus[u_j * C_GNSS_MAX + C_GNSS_BDS2] = pu_clkStatus[u_j * C_GNSS_MAX + C_GNSS_BDS3];
      pd_rcvClkSet[u_j * C_GNSS_MAX + C_GNSS_BDS2] = pd_rcvClkSet[u_j * C_GNSS_MAX + C_GNSS_BDS3];
      pq_obsNum[u_j * C_GNSS_MAX + C_GNSS_BDS2] = pq_obsNum[u_j * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }

  for (u_i = 0; u_i < C_GNSS_MAX; ++u_i)
  {
    for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      if (1 == pu_clkStatus[u_j * C_GNSS_MAX + u_i] || pq_obsNum[u_j * C_GNSS_MAX + u_i] < 1)
      {
        continue;
      }
      for (u_k = 0; u_k < C_GNSS_FREQ_TYPE_MAX; ++u_k)
      {
        if (1 == pu_clkStatus[u_k * C_GNSS_MAX + u_i])
        {
          pd_rcvClkSet[u_j * C_GNSS_MAX + u_i] = pd_rcvClkSet[u_k * C_GNSS_MAX + u_i];
          pu_clkStatus[u_j * C_GNSS_MAX + u_i] = 1;
          break;
        }
      }
    }
  }
  //initialize reciver clock
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 1;
  for (u_i = 0; u_i < C_GNSS_MAX; ++u_i)
  {
    if (0 == pu_clkStatus[u_i])
    {
      continue;
    }
    z_initInfo.d_initValue = pd_rcvClkSet[u_i];
    z_initInfo.d_sigma0 = RCV_CLK_SIGMA;
    z_initInfo.d_noise = RCV_CLK_SIGMA;
    w_id[0] = 0;
    w_id[1] = u_i;
    w_id[2] = GNSS_FILTER_STATE_CLK;
    addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
  }
  //initialize reciver DCB
  for (u_i = 0; u_i < C_GNSS_MAX; ++u_i)
  {
    if (0 == pu_clkStatus[u_i])
    {
      continue;
    }
    for (u_j = 1; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      if (0 == pu_clkStatus[u_j * C_GNSS_MAX + u_i])
      {
        continue;
      }
      z_filterType = convertFreqDCB2FilterType((gnss_FreqType)u_j);
      if (GNSS_FILTER_STATE_NUM == z_filterType)
      {
        continue;
      }
      //the logic to be modified
      z_initInfo.d_initValue = pd_rcvClkSet[u_j * C_GNSS_MAX + u_i] - pd_rcvClkSet[u_i];
      z_initInfo.d_sigma0 = RCV_DCB_SIGMA;
      z_initInfo.d_noise = RCV_DCB_SIGMA;
      w_id[0] = u_j;
      w_id[1] = u_i;
      w_id[2] = z_filterType;
      addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
  }
  return;
}
/**
 * @brief add and time update the receiver clock drift parameter in the EKF
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[out]    pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out]    pz_EKFstateRepPool is the pool of EKF state represent
 * @return        void
 */
void PPP_addRcvClkDriftPara(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  gnss_filterInitInfo_t z_initInfo = { 0 };
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  //initialize reciver clock drift parameter
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 1;
  z_initInfo.d_initValue = 0.0;
  z_initInfo.d_sigma0 = RCV_CLK_DRIFT_SIGMA;
  z_initInfo.d_noise = RCV_CLK_DRIFT_SIGMA;
  w_id[0] = 0;
  w_id[1] = 0;
  w_id[2] = GNSS_FILTER_STATE_DRIFT;
  addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
  return;
}
static uint8_t checkIodContinueFlag(gnss_SignalType u_signal,const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  const gnss_satSsrLos_t* pz_satLos, const ppp_iod_count_t* pz_iod_count)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  char char_sat[4];
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SignalCorr_t* pz_signal = NULL;

  if (NULL == pz_satLos || NULL == pz_SsrLocBlk)
  {
    return 0;
  }

  for (u_i = 0; u_i < GNSS_SSR_SAT_NUM_LIMIT; u_i++)
  {
    if (pz_satLos->u_constellation != pz_iod_count->iod_array[u_i].u_constellation
      || pz_iod_count->iod_array[u_i].u_svid != pz_satLos->u_svid)
    {
      continue;
    }
    satidx_SatString(gnss_cvt_Svid2SvIndex(pz_satLos->u_svid, pz_satLos->u_constellation), char_sat);
    z_freqType = gnss_cvt_Sig2FreqType(u_signal);
    if (pz_satLos->u_clockContinuityIod != pz_iod_count->iod_array[u_i].u_clockContinuityIod)
    {
      LOGI(TAG_PPP, "Reset amb by clock IOD sat=%s, freq=%d iod=%d %d\n", char_sat, z_freqType,
        pz_iod_count->iod_array[u_i].u_clockContinuityIod, pz_satLos->u_clockContinuityIod);
      return 1;
    }
    pz_signal = getSSR_Bias(pz_satLos->u_constellation, pz_satLos->u_svid, u_signal, pz_SsrLocBlk);
    if (pz_signal == NULL)
    {
      continue;
    }
    for (u_j = 0; u_j < pz_iod_count->iod_array[u_i].u_signalNum; u_j++)
    {
      if (pz_signal->u_signalType != pz_iod_count->iod_array[u_i].u_signalType[u_j])
      {
        continue;
      }
      if (pz_signal->u_discontinuityIod != pz_iod_count->iod_array[u_i].u_discontinuityIod[u_j])
      {
        LOGI(TAG_PPP, "Reset amb by bias IOD sat=%s, freq=%d iod=%d %d\n", char_sat, z_freqType,
          pz_iod_count->iod_array[u_i].u_discontinuityIod[u_j], pz_signal->u_discontinuityIod);
        return 2;
      }
    }
  }

  return 0;
}
static void setIodContinueFlag(const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_iod_count_t* pz_iod_count)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_n = 0;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalCorr_t* pz_signal = NULL;

  if (tm_GpsTimeDiff(&pz_SsrLocBlk->z_epochLosInfo.z_tor, &pz_iod_count->z_time) < FABS_ZEROS)
  {
    return;
  }

  memset(pz_iod_count, 0, sizeof(ppp_iod_count_t));
  pz_iod_count->z_time = pz_SsrLocBlk->z_epochLosInfo.z_tor;
  pz_iod_count->u_satnum = pz_SsrLocBlk->z_epochLosInfo.u_satCount;
  for (u_i = 0; u_i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; u_i++)
  {
    pz_satLos = &pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr[u_i];
    pz_iod_count->iod_array[u_i].u_constellation = pz_satLos->u_constellation;
    pz_iod_count->iod_array[u_i].u_svid = pz_satLos->u_svid;
    pz_iod_count->iod_array[u_i].u_clockContinuityIod = pz_satLos->u_clockContinuityIod;

    u_n = 0;
    for (u_j = 0; u_j < GNSS_SSR_SAT_CHL_NUM_LIMIT; u_j++)
    {
      if (!(pz_satLos->z_signalBiasCorr[u_j].u_biasMask & GNSS_SSR_SAT_BIAS_PHASE_CORR))
      {
        continue;
      }
      pz_signal = &pz_satLos->z_signalBiasCorr[u_j];
      pz_iod_count->iod_array[u_i].u_signalType[u_n] = pz_signal->u_signalType;
      pz_iod_count->iod_array[u_i].u_discontinuityIod[u_n] = pz_signal->u_discontinuityIod;
      u_n++;
    }
    pz_iod_count->iod_array[u_i].u_signalNum = u_n;
  }
}
/**
 * @brief add and time update the ambiguity parameter in the EKF
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void PPP_addZeroCombineSatAmbPara(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_idx = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint8_t u_allsignal = 0;
  uint8_t u_nslip = 0;
  char char_sat[4];
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  gnss_filterInitInfo_t z_initInfo = {0};
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  double d_wave = 0.0;
  double d_beta = 0.0;

  gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 0;
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }

  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
        || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR) 
        ||!(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
        continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation*RAD2DEG < 10.0||
      integrity_check(pz_satLos->z_orbClk.u_postItg, pz_satLos->z_orbClk.u_preItg) >= 1.0)
    {
      continue;
    }
    if (tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime) > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    if (varianceEphSSR(pz_satLos->z_orbClk.f_qi * 0.001f) > SQR(1.0) || (pz_satLos->z_orbClk.q_corr * 0.001) > 30.0)
    {
      continue;
    }
    satidx_SatString(q_satIndex, char_sat);
    /* only one frq, reset */
    uint8_t sig_num = PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR);
    if (sig_num < 1)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      z_initInfo.u_resetPara = 0;
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas|| !pz_sigMeas->z_measStatusFlag.b_valid) // sig meas
      {
        continue;
      }
      if (!pz_sigMeas->z_measStatusFlag.b_prValid && pz_sigMeas->f_cn0 < 25.0)
      {
        continue;
      }
      d_wave = wavelength(pz_sigMeas->u_signal);
      if (d_wave >= 1.0) // wave error
      {
        continue;
      }
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedSys) & z_algoConstellation)) // sys filter
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & z_algoFreq)) // frq filter
      {
        continue;
      }
      z_filterType = convertFreqAmb2FilterType(z_freqType);
      if (GNSS_FILTER_STATE_NUM == z_filterType)  // AMB type
      {
        continue;
      }
      if (fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS) // carrier == 0
      {
        LOGI(TAG_PPP, "carrier phase < 0.01, sat=%s, freq=%d, %f\n",char_sat, z_freqType, pz_sigMeas->d_carrierPhase);
        continue;
      }
      z_initInfo.d_initValue = PPP_calInitAMB(pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo,
          pz_EKFstateRepPool, pz_satMeas, pz_sigMeas);
      if (0.0 == z_initInfo.d_initValue)
      {
          LOGI(TAG_PPP, "Reset amb failed sat=%s, freq=%d\n", char_sat, z_freqType);
          continue;
      }
      u_allsignal++;
      if (1 == cmn_combineCycleSlipMask(pz_PPPfilterInfo->u_tdcpMethodValid, sig_num, pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, pz_PPPfilterInfo->d_deltaTimeDopplerDetect,u_isCurrentCloseSky, pz_sigMeas->u_slipMethodValid, NULL))
      {
        u_nslip++;
        z_initInfo.u_resetPara = 1;
        LOGI(TAG_PPP, "Reset amb by slip sat=%s, freq=%d, LLI=%d\n", char_sat, z_freqType, pz_sigMeas->u_LLI);
      }
      if (KF_RESET == (pz_PPPfilterInfo->z_kfStatus) || KF_RESET_AMB == (pz_PPPfilterInfo->z_kfStatus))
      {
        z_initInfo.u_resetPara = 1;
        LOGI(TAG_PPP, "Reset amb by kfStatus sat=%s, freq=%d status=%d\n", char_sat, z_freqType, pz_PPPfilterInfo->z_kfStatus);
      }

      if (checkIodContinueFlag(pz_sigMeas->u_signal, pz_SsrLocBlk, pz_satLos, &pz_PPPfilterInfo->z_iod_count))
      {
        z_initInfo.u_resetPara = 1;
      }

      z_initInfo.d_sigma0 = AMB_SIGMA;
      z_initInfo.d_noise = AMB_NOISE;
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = z_filterType;
      addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
    }
  }
  setIodContinueFlag(pz_SsrLocBlk, &pz_PPPfilterInfo->z_iod_count);
  if (u_allsignal > 0)
  {
    if (((u_nslip * 1.0) / u_allsignal) > 0.68)
    {
      pz_PPPfilterInfo->w_algStatus |= PPP_STATUS_SLIP_WARNING;
    }
  }
  return;
}
/**
 * @brief add and time update the ionospheric parameter in the EKF
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void PPP_addIonoPara(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_idx = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_isCurrentCloseSky = 0;
  char char_sat[4];
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  gnss_filterInitInfo_t z_initInfo;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  uint8_t u_isValid = 0;
  uint8_t u_resetPara = 0;
  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  z_initInfo.u_resetPara = 0;
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
        || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR) || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
        continue;
    }
    satidx_SatString(q_satIndex, char_sat);
    /* only one frq, skip */
    uint8_t sig_num = PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR);
    if (sig_num < 1)
    {
      continue;
    }
    u_isValid = 0;
    u_resetPara = 1;
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == (pz_PPPfilterInfo->z_opt.z_usedSys & z_algoConstellation)) // sys
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & z_algoFreq)) // frq
      {
        continue;
      }
      if (pz_sigMeas->z_measStatusFlag.b_valid) // signal use
      {
        u_isValid = 1;
      }
      //the condition to be optimizited
      if (0 == cmn_combineCycleSlipMask(pz_PPPfilterInfo->u_tdcpMethodValid, sig_num, pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, pz_PPPfilterInfo->d_deltaTimeDopplerDetect,u_isCurrentCloseSky, pz_sigMeas->u_slipMethodValid, NULL))
      {
        u_resetPara = 0;
      }
    }
    if (0 == u_isValid)
    {
      continue;
    }
    z_initInfo.u_resetPara = u_resetPara;
    z_initInfo.d_initValue = pz_satMeas->d_Iono;
    z_initInfo.d_sigma0 = IONO_SIGMA;
    z_initInfo.d_noise = IONO_NOISE;
    w_id[0] = 0;
    w_id[1] = gnss_cvt_Svid2SvIndex(pz_satMeas->u_svid, pz_satMeas->u_constellation);
    w_id[2] = GNSS_FILTER_STATE_IONO;
    if (KF_RESET == (pz_PPPfilterInfo->z_kfStatus))
    {
      z_initInfo.u_resetPara = 1;
    }
    if (z_initInfo.u_resetPara)
    {
      LOGI(TAG_PPP, "Reset ION sat=%s\n", char_sat);
    }
    addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid,
                       pz_EKFstateRepPool);
  }
  return;
}
/**
 * @brief add and time update the ZTD parameter in the EKF
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void PPP_addZTDpara(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
                    ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  gnss_filterInitInfo_t z_initInfo = {0};
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };

  z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
  
  z_initInfo.u_resetPara = 0;
  z_initInfo.d_initValue = (pz_satSigMeasCollect->d_zwd_emp);
  z_initInfo.d_sigma0 = ZTD_SIGMA;
  z_initInfo.d_noise = ZTD_NOISE;
  w_id[0] = 0;
  w_id[1] = 0;
  w_id[2] = GNSS_FILTER_STATE_ZTD;
  if (KF_RESET == (pz_PPPfilterInfo->z_kfStatus))
  {
    z_initInfo.u_resetPara = 1;
  }
  addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);
  
  return;
}
/**
 * @brief get the prior site coordinate that it will be used to do pseudo-range quality control
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pd_sitePriorCoor is the prior site coordinate
 * @return         void
 */
void PPP_getPriorSiteCoorUsedByPseudoRangeQC(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, double pd_sitePriorCoor[3])
{
  uint8_t u_i = 0;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  BOOL z_isPVAvalid = TRUE;
  double d_deltaTime = 0.0;
  double pd_PredictCoor[3] = { 0.0 };
  float f_velDiff = 0.0;
  float pf_predictVel[3] = { 0.0 };
  float pf_predictAcc[3] = { 0.0 };
  const gnss_PositionFix_t* pz_pvtPos = &(pz_satSigMeasCollect->z_positionFix);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_sitePriorCoor[u_i] = pz_pvtPos->d_xyz[u_i];
    pd_PredictCoor[u_i] = 0.0;
    pf_predictVel[u_i] = 0.0;
    pf_predictAcc[u_i] = 0.0;
  }
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    if (pw_PVAindex[u_i] < 0)
    {
      z_isPVAvalid = FALSE;
      break;
    }
  }
  if (TRUE == z_isPVAvalid)
  {
    d_deltaTime = tm_GpsTimeDiff(&(pz_satSigMeasCollect->z_tor), &(pz_EKFstateRepPool->z_gpsTime));
    if (fabs(d_deltaTime) > 1.5)
    {
      z_isPVAvalid = FALSE;
    }
  }
  if (TRUE == z_isPVAvalid)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_PredictCoor[u_i] = pz_PPPfilterInfo->pd_X[pw_PVAindex[u_i]];
      pf_predictVel[u_i] = (float)(pz_PPPfilterInfo->pd_X[pw_PVAindex[u_i + 3]]);
      pf_predictAcc[u_i] = (float)(pz_PPPfilterInfo->pd_X[pw_PVAindex[u_i + 6]]);
    }
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_PredictCoor[u_i] += pf_predictVel[u_i] * d_deltaTime;
      pf_predictVel[u_i] += (float)(pf_predictAcc[u_i] * d_deltaTime);
      f_velDiff += (pf_predictVel[u_i] - pz_pvtPos->f_velXyz[u_i]) * (pf_predictVel[u_i] - pz_pvtPos->f_velXyz[u_i]);
    }
    f_velDiff = (float)sqrt(f_velDiff);
    if (f_velDiff < 1.0)
    {
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_sitePriorCoor[u_i] = pd_PredictCoor[u_i];
      }
    }
  }
  return;
}
/**
 * @brief use the prior site coordinate to calculate the initial receiver clock of target constellation and target frequency pseudo-range
 * @param[in]      u_targetSys is the target constellation
 * @param[in]      u_targetFreq is the target frequency
 * @param[in]      pd_sitePriorCoor is the prior site coordinate
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pd_initRcvClk is the initial receiver clock of target constellation and target frequency pseudo-range
 * @return         void
 */
BOOL PPP_getInitRcvClkUsedPriorQCpseodoRange(gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const double pd_sitePriorCoor[3], const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, double* pd_initRcvClk)
{
  BOOL q_isSuccess = FALSE;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalCorr_t* pz_signalCorr = NULL;
  const double* pd_satPosClk = NULL;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_satNum = 0;
  uint8_t u_status = 0;
  int16_t w_ionoIndex = -1;
  int16_t w_ZTDindex = -1;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  algo_useConstellation u_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useFreq u_algoFreq = ALGO_NON_FREQ;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double d_codeBias = 0.0;
  double d_ionoCorrection = 0.0;
  double d_stecFact = 0.0;
  double d_beta = 0.0;
  double d_dist = 0.0;
  double d_omc = 0.0;
  double d_LosCorr = 0.0;
  double d_ztdValue = 0.0;
  double pd_clkSet[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  *pd_initRcvClk = 0.0;
  for (u_satNum = 0; u_satNum < MAX_GNSS_ACTIVE_SAT_NUMBER; ++u_satNum)
  {
    pd_clkSet[u_satNum] = 0.0;
  }

  u_satNum = 0;
  getZtdParaIndex(pz_EKFstateRepPool, &w_ZTDindex);
  if (w_ZTDindex >= 0)
  {
    d_ztdValue = pz_PPPfilterInfo->pd_X[w_ZTDindex];
    if (fabs(d_ztdValue - (pz_satSigMeasCollect->d_zwd_emp)) > 0.5)
    {
      d_ztdValue = (pz_satSigMeasCollect->d_zwd_emp);
    }
  }
  /* loop for sat */
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (u_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    /* sys */
    u_algoConstellation = gnss_satType2Algo(pz_satMeas->u_constellation);
    if (0 == ((pz_PPPfilterInfo->z_opt.z_usedSys) & u_algoConstellation)|| u_targetSys != pz_satMeas->u_constellation)
    {
      continue;
    }

    /* ssr block */
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    /* ssr brdc satellite posclk */
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH) || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      continue;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_sitePriorCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_sitePriorCoor, pd_satPosRot, pd_unitVector);
    /* loop for signal */
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_valid)
      {
        continue;
      }
      if (u_satNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      u_algoFreq = gnss_freqType2Algo(u_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & u_algoFreq) || u_freqType != u_targetFreq)
      {
        continue;
      }
      if (!(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }

      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = GNSS_FILTER_STATE_IONO;
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);

      d_LosCorr= pz_satLos->z_orbClk.q_corr * 0.001;
      /* SSR code bias */
      pz_signalCorr = getSSR_Bias(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_SsrLocBlk);
      if (NULL == pz_signalCorr)
      {
        continue;
      }
      d_codeBias = pz_signalCorr->z_codeBias.q_corr * 0.001;

      d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
      if (pz_PPPfilterInfo->w_algStatus& PPP_STATUS_a_few_SSR3)
      {
        if (w_ionoIndex < 0)
        {
          continue;
        }
        d_ionoCorrection = (d_beta * pz_PPPfilterInfo->pd_X[w_ionoIndex]);
      }
      else
      {
        if (!(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR))
        {
          continue;
        }
        d_stecFact = 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(pz_sigMeas->u_constellation));
        d_ionoCorrection = 0.001 * (pz_satLos->z_stec.q_corr) * d_stecFact;
        d_ionoCorrection = (d_beta * d_ionoCorrection);
      }

      d_omc = (pz_sigMeas->d_pseudoRange) - (d_dist - pd_satPosClk[3]);
      d_omc += d_codeBias;
      d_omc -= d_ionoCorrection;
      d_omc -= d_LosCorr;
      d_omc -= (pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap + d_ztdValue * pz_satMeas->d_wetMap);
      pd_clkSet[u_satNum] = d_omc;
      ++u_satNum;
    }
  }
  u_status = gnss_ascSortMedianDouble(pd_clkSet, u_satNum, pd_initRcvClk);
  if (1 == u_status)
  {
    q_isSuccess = TRUE;
  }
  return q_isSuccess;
}
/**
 * @brief use the prior site coordinate to calculate the initial receiver clock of target constellation and target frequency pseudo-range
 * @param[in]      u_targetSys is the target constellation
 * @param[in]      u_targetFreq is the target frequency
 * @param[in]      d_initRcvClk is the initial receiver clock of target constellation and target frequency pseudo-range
 * @param[in]      pd_sitePriorCoor is the prior site coordinate
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @return         void
 */
void PPP_maskPseudoRangeByPriorQC(gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, double d_initRcvClk, const double pd_sitePriorCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalCorr_t* pz_signalCorr = NULL;
  const double* pd_satPosClk = NULL;
  uint32_t q_satIndex = 0;
  uint8_t u_nb = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_status = 0;
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  char c_sat[4] = "";
  char c_buff[256] = "";
  algo_useConstellation u_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useFreq u_algoFreq = ALGO_NON_FREQ;
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double d_codeBias = 0.0;
  double d_ionoCorrection = 0.0;
  double d_stecFact = 0.0;
  double d_beta = 0.0;
  double d_dist = 0.0;
  double d_omc = 0.0;
  double d_LosCorr = 0.0;
  double d_ztdValue = 0.0;
  getZtdParaIndex(pz_EKFstateRepPool, &w_ZTDindex);
  if (w_ZTDindex >= 0)
  {
    d_ztdValue = pz_PPPfilterInfo->pd_X[w_ZTDindex];
    if (fabs(d_ztdValue-(pz_satSigMeasCollect->d_zwd_emp)) > 0.5)
    {
      d_ztdValue = (pz_satSigMeasCollect->d_zwd_emp);
    }
  }
  /* loop for sat */
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    /* sys */
    u_algoConstellation = gnss_satType2Algo(pz_satMeas->u_constellation);
    if (0 == ((pz_PPPfilterInfo->z_opt.z_usedSys) & u_algoConstellation)|| u_targetSys != pz_satMeas->u_constellation)
    {
      continue;
    }
    /* ssr block */
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    /* ssr brdc satellite posclk */
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH) || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      continue;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_sitePriorCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_sitePriorCoor, pd_satPosRot, pd_unitVector);
    /* loop for signal */
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_valid)
      {
        continue;
      }
      u_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      u_algoFreq = gnss_freqType2Algo(u_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & u_algoFreq) || u_freqType != u_targetFreq)
      {
        continue;
      }
      if (!(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }
      d_LosCorr = pz_satLos->z_orbClk.q_corr * 0.001;
      /* SSR code bias */
      pz_signalCorr = getSSR_Bias(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_SsrLocBlk);
      if (NULL == pz_signalCorr)
      {
        continue;
      }
      d_codeBias = pz_signalCorr->z_codeBias.q_corr * 0.001;
      
      d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
      if (pz_PPPfilterInfo->w_algStatus & PPP_STATUS_a_few_SSR3)
      {
        w_id[0] = 0;
        w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
        w_id[2] = GNSS_FILTER_STATE_IONO;
        getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
        if (w_ionoIndex < 0)
        {
          continue;
        }
        d_ionoCorrection = (d_beta * pz_PPPfilterInfo->pd_X[w_ionoIndex]);
      }
      else
      {
        if (!(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR))
        {
          continue;
        }
        d_stecFact = 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(pz_sigMeas->u_constellation));
        d_ionoCorrection = 0.001 * (pz_satLos->z_stec.q_corr) * d_stecFact;
        d_ionoCorrection = (d_beta * d_ionoCorrection);
      }

      d_omc = (pz_sigMeas->d_pseudoRange) - (d_dist - pd_satPosClk[3]);
      d_omc += d_codeBias;
      d_omc -= d_ionoCorrection;
      d_omc -= d_LosCorr;
      d_omc -= (pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap + d_ztdValue * pz_satMeas->d_wetMap);
      d_omc-= d_initRcvClk;
      if (fabs(d_omc) > 3.0)
      {
        pz_sigMeas->z_measStatusFlag.b_prValid = 0;

        if (log_GetLogLevel() >= LOG_LEVEL_I)
        {
          satidx_SatString(q_satIndex, c_sat);
          if (u_nb <= 0) u_nb += snprintf(&c_buff[u_nb], 20, "Prior_check res-pr:");
          u_nb += snprintf(&c_buff[u_nb], 21, " %3s f=%01d v=%8.3lf;", c_sat, u_signalIndex, d_omc);
          if (u_nb > 220)
          {
            LOGI(TAG_PPP, "%s\n", c_buff);
            strcpy(c_buff, ""); u_nb = 0;
          }
        }
      }
    }
  }
  if (u_nb > 0)
  {
    LOGI(TAG_PPP, "%s\n", c_buff);
    strcpy(c_buff, "");
  }
  return;
}

/**
 * @brief          Using the result of QR parity check to mark the pseudo-range observations
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @return         void
 */
void PPP_markPseudoRangeUsingQRresult(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      if (PR_QUALITY_GROSS_BY_QR_PARITY == (pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality))
      {
        pz_sigMeas->z_measStatusFlag.b_prValid = 0;    
      }
    }
  }
  return;
}

/**
 * @brief use the prior site coordinate to do target constellation and target frequency pseudo-range quality control
 * @param[in]      u_targetSys is the target constellation
 * @param[in]      u_targetFreq is the target frequency
 * @param[in]      pd_sitePriorCoor is the prior site coordinate
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @return         void
 */
void PPP_PriorQualityControlPseodoRange(gnss_ConstellationType u_targetSys, gnss_FreqType u_targetFreq, const double pd_sitePriorCoor[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  double d_initRcvClk = 0.0;
  if (TRUE == PPP_getInitRcvClkUsedPriorQCpseodoRange(u_targetSys, u_targetFreq, pd_sitePriorCoor, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, &d_initRcvClk))
  {
    PPP_maskPseudoRangeByPriorQC(u_targetSys, u_targetFreq, d_initRcvClk, pd_sitePriorCoor, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
  }
  
  return;
}
/**
 * @brief use the prior residual to do quality control
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[in]      pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @return         void
 */
void PPP_priorResPseudoRangeQualityControl(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  double pd_sitePriorCoor[3] = { 0.0 };
  gnss_ConstellationType u_sys = C_GNSS_GPS;
  gnss_FreqType u_freq = C_GNSS_FREQ_TYPE_L1;
  PPP_getPriorSiteCoorUsedByPseudoRangeQC(pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, pd_sitePriorCoor);

  for (u_sys = C_GNSS_GPS; u_sys < C_GNSS_MAX; ++u_sys)
  {
    for (u_freq = C_GNSS_FREQ_TYPE_L1; u_freq < C_GNSS_FREQ_TYPE_MAX; ++u_freq)
    {
      PPP_PriorQualityControlPseodoRange(u_sys, u_freq, pd_sitePriorCoor, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
    }
  }
  return;
}
/**
 * @brief calculate the bias between carrier phase and pseudo-range
 * @param[in]     u_targetSys is the target constellation
 * @param[in]     u_targetFreq is the target frequency
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @return        the bias between carrier phase and pseudo-range
 */
double PPP_calBiasBetweenPhaseCode(uint8_t u_targetSys, uint8_t u_targetFreq, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const ppp_filterInfo_t* pz_PPPfilterInfo)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_obsNum = 0;
  uint8_t u_cycleSlipMask = 0;
  uint8_t u_sigNum = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint16_t w_iSat = 0;
  int16_t w_ambIndex = -1;
  int16_t w_ionIndex = -1;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  double d_wave = 0.0;
  double d_ion = 0.0;
  double pd_offset[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0 };
  double d_median = 0.0;
  double d_std = 0.0;
  double d_maxdiff = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const double* pd_X = pz_PPPfilterInfo->pd_X;
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas || u_targetSys != (pz_satMeas->u_constellation))
    {
      continue;
    }
    u_sigNum = PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR);
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < (pz_PPPfilterInfo->z_opt.d_elmin))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (u_obsNum >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
        continue;
      }
      if (fabs(pz_sigMeas->d_carrierPhase) < FABS_ZEROS)
      {
        continue;
      }
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (z_freqType != u_targetFreq)
      {
        continue;
      }
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }

      d_wave = wavelength(pz_sigMeas->u_signal);
      if (d_wave >= 1.0)
      {
        continue;
      }
      u_cycleSlipMask = cmn_combineCycleSlipMask(pz_PPPfilterInfo->u_tdcpMethodValid, u_sigNum,
        pz_sigMeas->u_slipMask, pz_sigMeas->u_LLI, pz_PPPfilterInfo->d_deltaTimeDopplerDetect,u_isCurrentCloseSky, pz_sigMeas->u_slipMethodValid, NULL);
      if (1 == u_cycleSlipMask)
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (w_ambIndex < 0)
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_satMeas->u_svid, pz_satMeas->u_constellation);
      w_id[2] = GNSS_FILTER_STATE_IONO;
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionIndex);
      if (w_ionIndex < 0)
      {
        d_ion = pz_satMeas->d_Iono;
      }
      else d_ion = pd_X[w_ionIndex];

      pd_offset[u_obsNum] = (pz_sigMeas->d_carrierPhase - pd_X[w_ambIndex]) - (pz_sigMeas->d_pseudoRange - 2 * d_ion) / d_wave;
      if (fabs(pd_offset[u_obsNum]) > d_maxdiff)
      {
        d_maxdiff = pd_offset[u_obsNum];
      }
      ++u_obsNum;
    }
  }
  if (u_obsNum <= 1)
  {
    d_median = pd_offset[0];
  }
  else
  {
    gnss_ascSortMedianStdDouble(pd_offset, u_obsNum, &d_median, &d_std);
    LOGD(TAG_PPP, "phaseCodeClkDiff=%10.1lf std=%10.1lf isys=%d   freq=%d maxdiff=%10.1lf\n", d_median, d_std, u_targetSys, u_targetFreq, d_maxdiff);
  }

  if (u_obsNum <= 1 || (u_obsNum < 5 && d_std>150) || d_std > 500.0)
  {
    d_median = 0.0;
  }
  return d_median;
}
/**
 * @brief add and time update the PVA, receiver clock, ZTD,ambiguity and ionospheric parameter in the EKF
 * @param[in]     pd_siteCoor is the initilization value of site coordination
 * @param[in]     pd_siteVel is the initilization value of site velocity
 * @param[in/out] pz_satSigMeasCollect is the observation information
 * @param[in]     pz_SsrLocBlk is the SSR product
 * @param[out]    pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void PPP_timeUpdateFilterPara(const double pd_siteCoor[3], const double pd_siteVel[3], gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint16_t w_i = 0;
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    pz_PPPfilterInfo->pq_paraValid[w_i] = FALSE;
  }
  PPP_addSiteParaPVA(pz_satSigMeasCollect->z_tor, pd_siteCoor, pd_siteVel, pz_PPPfilterInfo, pz_EKFstateRepPool);
  PPP_addRcvClkPara(pd_siteCoor, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
  if (1 != (pz_PPPfilterInfo->u_goodSceneStatusFlag))
  {
    PPP_addRcvClkDriftPara(pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool);
  }
  PPP_addZTDpara(pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool);
  PPP_addZeroCombineSatAmbPara(pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
  PPP_addIonoPara(pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
  pz_EKFstateRepPool->z_gpsTime = pz_satSigMeasCollect->z_tor;
  removeEKFstatus(pz_EKFstateRepPool, pz_PPPfilterInfo->w_nmax, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q,
      pz_PPPfilterInfo->pq_paraValid);

  pz_PPPfilterInfo->z_kfStatus = KF_RUN;
  return;
}
/**
 * @brief fprint covariance martix
 * @param[in] P covariance martix
 * @param[in] n n*n
 * @param[in] fp FILE pointer
 */
static void log_FprintP(double* P, uint16_t n, FILE* fp, char* note)
{
  uint16_t i = 0;
  uint16_t j = 0;
  if(NULL==fp){
    return;
  }
  fprintf(fp,"%s\n", note);
  for (i = 0; i < n; ++i)
  {
    for (j = 0; j <=i; ++j)
    {
      fprintf(fp," %8.2f ", P[IUTM(i,j)]);
    }
    fprintf(fp,"\n");
  }
  fflush(fp);
}

static void log_FilterSatInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
 , const ppp_EpochFilterObs_t* pz_FilterObs, const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_flag = 0;
  uint16_t w_x_id[3] = { 0 };
  char c_buff[256] = "";
  char c_sat[4] = "";
  char* p = c_buff;
  double dt = 0.0;
  double d_dcb = 0.0;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_signalMeas = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  const ppp_SatFilterObs_t* pz_SatFilterObs = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  if (log_GetLogLevel() < LOG_LEVEL_I)
  {
    return;
  }

  //get the  site state index in the filter
  if (!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    return;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  p += snprintf(p, 5, "DCB:");
  for (u_i = 0; u_i < (C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX); u_i++)
  {
    if (pw_rcvClkIndex[u_i] < 0)
    {
      d_dcb = 0.0;
    }
    else
    {
      d_dcb = pz_PPPfilterInfo->pd_X[pw_rcvClkIndex[u_i]];
    }
    p += snprintf(p, 10, "%8.3lf ", d_dcb);
    if (0 == (u_i + 1) % C_GNSS_MAX)
    {
      p += snprintf(p, 3, "\n");
      LOGD(TAG_PPP, "%s", c_buff);
      strcpy(c_buff, "");
      p = c_buff;
    }
  }

  p = c_buff;
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_i];
    if (NULL == pz_satMeas||C_SAT_TYPE_GEO==svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid)
      ||C_GNSS_QZS== pz_satMeas->u_constellation)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[u_i];
    satidx_SatString(u_i, c_sat);
    p += snprintf(p,17, "%s ele:%4.1lf   ", c_sat, pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG);
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      pz_signalMeas = pz_satMeas->pz_signalMeas[u_j];
      if (NULL == pz_signalMeas|| pz_satMeas->z_satPosVelClk.f_elevation<=0.0)
      {
        continue;
      }
      if (NULL == pz_SatFilterObs || NULL == pz_SatFilterObs->z_measUpdateFlag[u_j]
        ||!pz_SatFilterObs->z_measUpdateFlag[u_j]->b_cpValid)
      {
        u_flag = 0u;
      }
      else
      {
        u_flag = (uint8_t)(pz_SatFilterObs->z_measUpdateFlag[u_j]->b_cpValid);
      }
      
      w_x_id[0] = 0;
      w_x_id[1] = u_i;
      w_x_id[2] = convertFreqAmb2FilterType(u_j);
      pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);
      if (NULL == pz_satPool)
      {
        dt = -1.0;
      }
      else
      {
        dt = tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime);
      }
      p += snprintf(p, 23, "Lock%1d:%7.1lf %3.1f %d%d ", u_j, dt, pz_signalMeas->f_cn0, (uint8_t)(pz_signalMeas->z_measStatusFlag.b_prValid), u_flag);
    }
    p += snprintf(p,3, "\n");
    LOGI(TAG_PPP, "%s", c_buff);
    strcpy(c_buff, "");
    p = c_buff;
  }
}

static void log_FilterInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, ppp_filterInfo_t* pz_PPPfilterInfo,
                           gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, char* note)
{
  uint8_t u_i = 0;
  FILE* fp = pz_PPPfilterInfo->fp_log;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t w_ambIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  char satid[4] = {0};
  if (NULL == pz_PPPfilterInfo->fp_log)
  {
    return;
  }

  if (!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    return;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  fprintf(fp, "\nLOG-%s, %.1f\n",note, pz_satSigMeasCollect->z_tor.q_towMsec * TIME_MSEC_INV);

  /* PVA */
  fprintf(fp, "  PVA X  : ");
  for (int i = 0; i < PVA_NUM; ++i)
  {
    double value = 0;
    int idx = pw_PVAindex[i];
    if (idx != INVALID_INDEX)
    {
      value = pz_PPPfilterInfo->pd_X[idx];
    }
    fprintf(fp, "%11.2f", value);
  }
  fprintf(fp, "\n");
  fprintf(fp, "  PVA P  : ");
  for (int i = 0; i < PVA_NUM; ++i)
  {
    double value = 0;
    int idx = pw_PVAindex[i];
    if (idx != INVALID_INDEX)
    {
      value = pz_PPPfilterInfo->pd_Q[IUTM(idx, idx)];
    }
    fprintf(fp, "%11.2f", value);
  }
  fprintf(fp, "\n");

  /* ZTD */
  fprintf(fp, "  ZWD XP: %8.2f %8.2f\n",  pz_PPPfilterInfo->pd_X[w_ZTDindex], pz_PPPfilterInfo->pd_Q[IUTM(w_ZTDindex, w_ZTDindex)]);

  /* CLK */
  fprintf(fp, "  CLK X  : ");
  for (int i = 0; i < C_GNSS_MAX * C_GNSS_FREQ_TYPE_MAX; ++i)
  {
    double value = 0;
    int idx = pw_rcvClkIndex[i];
    if (idx != INVALID_INDEX)
    {
      value = pz_PPPfilterInfo->pd_X[idx];
    }
    fprintf(fp, "%8.2f", value);
  }
  fprintf(fp, "\n");
  fprintf(fp, "  CLK P  : ");
  for (int i = 0; i < C_GNSS_MAX * C_GNSS_FREQ_TYPE_MAX; ++i)
  {
    double value = 0;
    int idx = pw_rcvClkIndex[i];
    if (idx != INVALID_INDEX)
    {
      value = pz_PPPfilterInfo->pd_Q[IUTM(idx, idx)];
    }
    fprintf(fp, "%8.2f", value);
  }
  fprintf(fp, "\n");

  /* AMB */
  for (int i = 0; i < pz_satSigMeasCollect->u_satMeasCount; ++i)
  {
    int sati = pz_satSigMeasCollect->u_satMeasIdxTable[i];
    const gnss_SatelliteMeas_t* pz_meas =  pz_satSigMeasCollect->pz_satMeas[sati];
    if (NULL == pz_meas)
    {
      continue;
    }
    double xx[3] = {0.0};
    double pp[3] = {0.0};
    satidx_SatString(sati,satid);
    for (int j = 0; j < MAX_GNSS_SIGNAL_FREQ; ++j)
    {
      gnss_SignalMeas_t* pz_signalMeas = pz_meas->pz_signalMeas[j];
      if (NULL != pz_signalMeas)
      {
        w_id[0] = 0;
        w_id[1] = sati;
        w_id[2] = GNSS_FILTER_STATE_AMB_L1 + j;
        getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ambIndex);
        if (w_ambIndex != INVALID_INDEX)
        {
          xx[j] = pz_PPPfilterInfo->pd_X[w_ambIndex];
          pp[j] = pz_PPPfilterInfo->pd_Q[IUTM(w_ambIndex, w_ambIndex)];
        }
      }
    }
    fprintf(fp, "  AMB %s: %8.2f, %8.2f, %8.2f, %8.2f, %8.2f, %8.2f\n", satid, xx[0], xx[1], xx[2], pp[0], pp[1], pp[2]);
  }
  /* ION */
  for (int i = 0; i < pz_satSigMeasCollect->u_satMeasCount; ++i)
  {
    int sati = pz_satSigMeasCollect->u_satMeasIdxTable[i];
    const gnss_SatelliteMeas_t* pz_meas = pz_satSigMeasCollect->pz_satMeas[sati];
    double xx = 0.0;
    double pp = 0.0;
    satidx_SatString(sati, satid);

    w_id[0] = 0;
    w_id[1] = sati;
    w_id[2] = GNSS_FILTER_STATE_IONO;
    getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
    if (w_ionoIndex != INVALID_INDEX)
    {
      xx = pz_PPPfilterInfo->pd_X[w_ionoIndex];
      pp = pz_PPPfilterInfo->pd_Q[IUTM(w_ionoIndex, w_ionoIndex)];
    }
    fprintf(fp, "  ION %s: %8.2f, %8.2f\n", satid, xx, pp);
  }

  fflush(fp);
}

/**
 * @brief model of stochastic
 * @param[in]  model
 * @param[in]  u_sys
 * @param[in]  u_svid
 * @param[in]  f_ele
 * @param[in]  u_type
 * @param[in]  f_snr
 * @return     variance
 */
double varianceObs(PPPStochasticModelType model, uint8_t u_sys, uint8_t u_svid, gnss_ObsType u_type, float f_ele, float f_snr)
{
  double fact = 1.0;
  double var = 0.0;
  double var_snr = 0.0;
  double var_ele = 0.0;
  const float EFACT_GPS = 1.0;   /* factor of GPS */
  const float EFACT_GLO = 1.5;   /* factor of GLONASS */
  const float EFACT_GAL = 1.0;   /* factor of Galileo */
  const float EFACT_CMP = 3.0;   /* factor of Compass */
  const float EFACT_BDS = 2.0;   /* factor of BDS */
  const float EFACT_QZS = 1.0;   /* factor of QZSS */

  if (C_GNSS_OBS_TYPE_PR == u_type)  /* pseudorange */
  {
    fact *= 200.0;
  }
  if (C_GNSS_GPS == u_sys)
  {
    fact *= EFACT_GPS;
  }
  else if (C_GNSS_GLO == u_sys)
  {
    fact *= EFACT_GLO;
  }
  else if (C_GNSS_GAL == u_sys)
  {
    fact *= EFACT_GAL;
  }
  else if (C_GNSS_BDS3 == u_sys || C_GNSS_BDS2 == u_sys)
  {
    fact *= EFACT_BDS;
    if (!gnss_isBDS3Sat(u_svid, u_sys) || C_SAT_TYPE_IGSO == svid2SatType(u_sys, u_svid))
    {
      fact *= 5.0;
    }
  }
  else if (C_GNSS_QZS == u_sys)
  {
    fact *= EFACT_QZS;
  }
  else
  {
    fact *= 10000.0;
  }

  switch (model)
  {
    case STOCHASTIC_ELE:
    {
      var = SQR(fact * 0.008) + SQR(fact * 0.008 / sin(f_ele));
      break;
    }
    case STOCHASTIC_SNR:
    {
      var = SQR(fact * 0.008) * pow(10.0, MAX_A_B((40.0 - f_snr) / 10.0, 0.0));
      break;
    }
    case STOCHASTIC_ELE_SNR:
    {
      var_ele = SQR(fact * 0.008) + SQR(fact * 0.008 / sin(f_ele));
      var_snr = SQR(fact * 0.008)* pow(10.0, MAX_A_B((40.0 - f_snr) / 10.0, 0.0));
      if (1 == u_type)  /* pseudorange */
      {
        var = var_ele * 0.8 + var_snr * 0.2;
      }
      else
      {
        var = var_ele * 0.2 + var_snr * 0.8;
      }
      break;
    }
    default:
      return SQR(10000.0);
      break;
  }

  return var;
}
/**
 * @brief select reference satellite of stec
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pz_PPPfilterInfo is information of filter
 * @param[in]  pz_ssrStatus
 * @param[out]  refsat
 * @return    none
 */
void stecRefsatSelect(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,const gnss_ssrLosBlock_t* pz_SsrLocBlk
  ,gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_filterInfo_t* pz_PPPfilterInfo, ppp_EpochFilterObs_t* pz_FilterObs, uint8_t* u_refsat)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_isys = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_ns = 0 ;
  uint16_t u_sat = 0;
  uint16_t w_temp = 0;
  uint16_t w_x_id[3] = { 0 };
  uint16_t* pw_satlist = NULL;
  float f_coef = 1.0;
  float f_minvar = 0.0;
  float f_temp = 0;
  float* pf_weight = NULL;
  const GpsTime_t* curtime = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  const ppp_SatFilterObs_t* pz_SatFilterObs = NULL;

  curtime = &pz_satSigMeasCollect->z_tor;
  pw_satlist = OS_MALLOC_FAST(GNSS_SSR_SAT_NUM_LIMIT * sizeof(uint16_t));
  pf_weight = OS_MALLOC_FAST(GNSS_SSR_SAT_NUM_LIMIT * sizeof(float));
  if (NULL == pw_satlist || NULL == pf_weight)
  {
    u_status = 1;
  }
  for (u_isys = 0; u_isys < C_GNSS_MAX && (!u_status); u_isys++)
  {
    u_ns = 0;
    u_refsat[u_isys] = 0u;
    if (TRUE == gnss_WhetherSkipBDS2VarInLoop(pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3, u_isys))
    {
      continue;
    }
    for (u_sat = 0; u_sat < ALL_GNSS_SYS_SV_NUMBER; u_sat++)
    {
      f_coef = 1.0;
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_sat];
      if (NULL == pz_satMeas)
      {
        continue;
      }
      u_constellation = gnss_getConstellationEnumValueInLoop(pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3, pz_satMeas->u_constellation);
      if (u_isys != u_constellation || pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 20.0)
      {
        continue;
      }
      if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
      {
        continue;
      }
      
      pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[u_sat];
      if (NULL == pz_SatFilterObs)
      {
        continue;
      }
      for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
      {
        if (NULL != pz_SatFilterObs->z_measUpdateFlag[u_j] && pz_SatFilterObs->z_measUpdateFlag[u_j]->b_cpValid
          && pz_SatFilterObs->z_measUpdateFlag[u_j]->b_prValid)
        {
          break;
        }
      }
      if (u_j >= MAX_GNSS_SIGNAL_FREQ)  // no carrier meas update
      {
        continue;
      }

      pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
      if ((NULL == pz_satLos) || !(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR)
        || !(GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID & pz_satLos->u_windUpMask)) // no stec correction
      {
        continue;
      }
      if (fabs(tm_GpsTimeDiff(curtime, &pz_satLos->z_STECtime)) > PPP_MAX_SSR_AGE)
      {
        continue;
      }
      if (integrity_check(pz_satLos->z_stec.u_postItg, pz_satLos->z_stec.u_preItg) >= 1.0)
      {
        continue;
      }
      if (pz_satLos->z_orbClk.f_qi * 0.001 > 0.3 || fabs(pz_satLos->z_stec.f_qi) > 5.0)
      {
        continue;
      }
      w_x_id[0] = 0;
      w_x_id[1] = u_sat;
      w_x_id[2] = GNSS_FILTER_STATE_IONO;

      /* get status from EKF */
      pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);
      if (NULL == pz_satPool || fabs(tm_GpsTimeDiff(curtime, &pz_satPool->z_endTime)) > 3.0)
      {
        continue;
      }

      if (C_SAT_TYPE_IGSO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
      {
        f_coef *= 50.0;
      }
      if (fabs(tm_GpsTimeDiff(&pz_satPool->z_beginTime, &pz_satPool->z_endTime)) < 1.0)
      {
        f_coef *= 100.0f;
      }
      pf_weight[u_ns] = pz_satLos->z_stec.f_qi * f_coef;
      pw_satlist[u_ns] = u_sat;
      u_ns++;
    }

    /* weight sort*/
    for (u_i = 0; u_i < u_ns; u_i++)
    {
      for (u_j = u_i + 1; u_j < u_ns; u_j++)
      {
        if (fabsf(pf_weight[u_j]) < fabsf(pf_weight[u_i]))
        {
          f_temp = pf_weight[u_j]; pf_weight[u_j] = pf_weight[u_i]; pf_weight[u_i] = f_temp;
          w_temp = pw_satlist[u_j]; pw_satlist[u_j] = pw_satlist[u_i]; pw_satlist[u_i] = w_temp;
        }
      }
    }

    /* select refsat from three */
    f_minvar = 0.0f;
    for (u_i = 0; u_i < u_ns && u_i < 3; u_i++)
    {
      u_sat = pw_satlist[u_i];
      w_x_id[0] = 0;
      w_x_id[1] = u_sat;
      w_x_id[2] = GNSS_FILTER_STATE_IONO;
      pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);
      pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_sat];
      f_coef = (float)fabs(tm_GpsTimeDiff(&pz_satPool->z_beginTime, &pz_satPool->z_endTime));
      if (f_coef > 600.0f) f_coef = 600.0f;
      else if (f_coef <= 0) f_coef = 1.0f;
      f_coef *= (float)(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG);
      for (u_j = 1; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
      {
        if (NULL != pz_satMeas->pz_signalMeas[u_j] && fabs(pz_satMeas->pz_signalMeas[u_j]->d_carrierPhase) > 1000.0
          && pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG > 30.0)
        {
          f_coef *= 600.0f;
          break;
        }
      }
      if (f_coef > f_minvar)
      {
        f_minvar = f_coef;
        u_refsat[u_isys] = pw_satlist[u_i] + 1;
      }
    }
  }

  if (NULL != pf_weight)
  {
    OS_FREE(pf_weight);
  }
  if (NULL != pw_satlist)
  {
    OS_FREE(pw_satlist);
  }
  
}
uint8_t PPP_stecMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_j = 0;
  uint8_t u_idx = 0;
  uint8_t u_sys = 0;
  uint8_t u_sat = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_svid = 0;
  uint8_t u_nobs = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint8_t u_refsat[C_GNSS_MAX ] = { 0 };
  uint16_t w_x_id[3] = { 0 };
  int16_t w_ionoIndex = 0;
  int16_t w_ionoIndex_R = 0;
  int32_t q_singeDiffStec = 0;
  char c_sat[4] = "";
  char c_refsat[4] = "";
  double d_omc = 0.0;
  double d_curR = 0.0;
  double stec_fact = 1.0;
  double dt = 0.0;
  double d_lock_t = 0.0;
  double* pd_deltaX = pz_PPPfilterInfo->pd_deltaX;
  SeqKalmanVar_t z_seqVar = { 0 };
  const GpsTime_t* curtime = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_satSsrLos_t* pz_satLos_R[C_GNSS_MAX ] = { NULL };
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  ppp_SatFilterObs_t* pz_SatFilterObs = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  uint8_t seq_rc = hpp_seq_init(&z_seqVar, pz_PPPfilterInfo->w_nmax);
  if (seq_rc != 0)
  {
    return 0;
  }
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  /* stec refsat */
  stecRefsatSelect(pz_satSigMeasCollect, pz_SsrLocBlk, pz_EKFstateRepPool, pz_PPPfilterInfo, pz_FilterObs, u_refsat);

  for (u_sys = 0; u_sys < C_GNSS_MAX ; u_sys++)
  {
    if (u_refsat[u_sys] <= 0)
    {
      continue;
    }
    u_svid = gnss_cvt_SvIndex2Svid(u_refsat[u_sys] - 1, &u_constellation);
    pz_satLos_R[u_sys] = getSSR_constLos(u_constellation, u_svid, pz_SsrLocBlk);
  }
  /* begain stec filter*/
  curtime = &pz_satSigMeasCollect->z_tor;
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    u_sat = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_SatFilterObs = NULL;
    u_svid = gnss_cvt_SvIndex2Svid(u_sat, &u_constellation);
    u_sys = gnss_getConstellationEnumValueInLoop(pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3, u_constellation);
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_sat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (u_refsat[u_sys] <= 0 || (u_sat + 1) == u_refsat[u_sys])
    {
      continue;
    }
    /* get sat status and ssr */
    w_x_id[0] = 0;
    w_x_id[1] = u_sat;
    w_x_id[2] = GNSS_FILTER_STATE_IONO;
    pz_satPool = getEKF_status(w_x_id, pz_EKFstateRepPool);
    if (NULL == pz_satPool || fabs(tm_GpsTimeDiff(curtime, &pz_satPool->z_endTime)) > 3.0)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(u_constellation, u_svid, pz_SsrLocBlk);
    if ((NULL == pz_satLos) || !(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR) || (fabsf(pz_satLos->z_stec.f_qi) > 10.0))
    {
      continue;
    }
    if (integrity_check(pz_satLos->z_stec.u_postItg, pz_satLos->z_stec.u_preItg) >= 1.0)
    {
      continue;
    }
    dt = fabs(tm_GpsTimeDiff(curtime, &pz_satLos->z_STECtime));
    if (dt > PPP_MAX_SSR_AGE)
    {
      continue;
    }

    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[u_sat];
    if (NULL == pz_SatFilterObs || NULL == pz_FilterObs->pz_SatFilterObs[u_refsat[u_sys] - 1])
    {
      continue;
    }
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      if (NULL != pz_SatFilterObs->z_measUpdateFlag[u_j] && (pz_SatFilterObs->z_measUpdateFlag[u_j]->b_cpValid
        || pz_SatFilterObs->z_measUpdateFlag[u_j]->b_prValid))
      {
        break;
      }
    }
    if (u_j >= MAX_GNSS_SIGNAL_FREQ)  // no meas update
    {
      continue;
    }
    pz_SatFilterObs->u_IonoCorrStatus = PPP_SSR_CORRECTED;
    pz_FilterObs->pz_SatFilterObs[u_refsat[u_sys] - 1]->u_IonoCorrStatus = PPP_SSR_CORRECTED;  // reference-sat

    w_x_id[0] = 0;
    w_x_id[1] = u_sat;
    w_x_id[2] = GNSS_FILTER_STATE_IONO;
    getTargetParaIndex(pz_EKFstateRepPool, w_x_id, &w_ionoIndex);
    w_x_id[1] = u_refsat[u_sys] - 1;
    getTargetParaIndex(pz_EKFstateRepPool, w_x_id, &w_ionoIndex_R);
    if (w_ionoIndex < 0 || w_ionoIndex_R < 0)
    {
      continue;
    }
    hpp_seq_ClearH(&z_seqVar);

    /* H */
    hpp_seq_AddH(w_ionoIndex, 1.0, &z_seqVar); // iono
    hpp_seq_AddH(w_ionoIndex_R, -1.0, &z_seqVar); // iono

    /* OMC */
    stec_fact = 40.31 * (1.0e+16) / SQR(gnss_BaseL1Freq(u_constellation));
    q_singeDiffStec = pz_satLos->z_stec.q_corr - pz_satLos_R[u_sys]->z_stec.q_corr;
    d_omc = 0.001 * ((double)q_singeDiffStec) * stec_fact - (pd_X[w_ionoIndex] - pd_X[w_ionoIndex_R]);

    d_curR = SQR(0.02);
    if (fabsf(pz_satLos->z_stec.f_qi) > 0.0)
    {
      d_curR = SQR(fabsf(pz_satLos->z_stec.f_qi) * stec_fact * 0.5);
    }
    if (d_curR > SQR(0.5))
    {
      continue;
    }
    if (dt > 30.0)
    {
      d_curR *= SQR(dt / 30.0);
    }
    if (PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_PR) > 1 || PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR) > 1)
    {
      d_lock_t = tm_GpsTimeDiff(&pz_satPool->z_endTime, &pz_satPool->z_beginTime);
      if (d_lock_t > 1800.0) d_curR *= SQR(10.0);
      else if (d_lock_t > 1200.0) d_curR *= SQR(5.0);
      else if (d_lock_t > 600.0) d_curR *= SQR(3.0);
      else if (d_lock_t > 300.0) d_curR *= SQR(2.0);
    }
    if (d_curR < 0.0025) d_curR = 0.0025;
    if (1 == u_isCurrentCloseSky && d_curR > 1.0e-4)
    {
      d_curR = 1.0e-4;
    }
    hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
    hpp_seq_PredictStep(pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar);
    hpp_seq_measUpdate(pz_PPPfilterInfo->pq_paraValid, pz_PPPfilterInfo->pd_X, pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar, 0.0);
    ++u_nobs;

    if (NULL != pz_FilterObs->pz_SatFilterObs[u_sat])
    {
      pz_FilterObs->pz_SatFilterObs[u_sat]->f_ionResidual = (float)d_omc;
    }

    /* print log */
    satidx_SatString(u_sat, c_sat);
    satidx_SatString(u_refsat[u_sys] - 1, c_refsat);
    LOGI(TAG_PPP, "stec sat=%s,refsat=%s, V=%7.3lf R=%9.3lf qi=%9.3lf dt=%4.1lf\n", c_sat, c_refsat, d_omc, sqrt(d_curR), pz_satLos->z_stec.f_qi * stec_fact, dt);
  }

  /* free */
  hpp_seq_deinit(&z_seqVar);

  return u_nobs;
}
uint8_t PPP_ztdMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_nobs = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint16_t w_x_id[3] = { 0 };
  int16_t w_tropIndex = 0;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double dt = 0.0;
  double* pd_deltaX = pz_PPPfilterInfo->pd_deltaX;
  SeqKalmanVar_t z_seqVar = { 0 };
  const gnss_satSsrLos_t* pz_satLos = NULL;

  /* begain ztd trop filter*/ // no trop correction
  if (!(pz_SsrLocBlk->z_epochLosInfo.u_ZTDmask & GNSS_SSR_ATMO_ZTD_CORR) ||
    integrity_check(pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.u_postItg, pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.u_preItg) >= 1.0)
  {
    return 0;
  }
  uint8_t seq_rc = hpp_seq_init(&z_seqVar, pz_PPPfilterInfo->w_nmax);
  if (seq_rc != 0)
  {
    return 0;
  }
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  dt = fabs(tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_SsrLocBlk->z_epochLosInfo.z_ZTDtime));
  w_x_id[0] = 0;
  w_x_id[1] = 0;
  w_x_id[2] = GNSS_FILTER_STATE_ZTD;
  getTargetParaIndex(pz_EKFstateRepPool, w_x_id, &w_tropIndex);
  if (dt < PPP_MAX_SSR_AGE && w_tropIndex >= 0)
  {
    hpp_seq_ClearH(&z_seqVar);
    hpp_seq_AddH(w_tropIndex, 1.0, &z_seqVar); // trop

    d_omc = pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.d_wetCorr - pd_X[w_tropIndex];
    d_curR = SQR(0.02);
    if (fabs(pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.f_qi) < 999.0)
    {
      d_curR = SQR(pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.f_qi * 0.5);
    }
    if (dt > 20.0)
    {
      d_curR *= SQR(dt / 20.0);
    }
    if (d_curR < 0.0004) d_curR = 0.0004;
    if (1 == u_isCurrentCloseSky && d_curR > 1.0e-6)
    {
      d_curR = 1.0e-6;
    }
    hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
    hpp_seq_PredictStep(pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar);
    hpp_seq_measUpdate(pz_PPPfilterInfo->pq_paraValid, pz_PPPfilterInfo->pd_X, pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar, 0.0);
    ++u_nobs;

    LOGI(TAG_PPP, "trop:ztd=%7.3lf,ztd_x=%7.3lf R=%9.3lf qi=%9.3lf dt=%4.1lf\n", pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.d_wetCorr, pd_X[w_tropIndex], sqrt(d_curR), pz_SsrLocBlk->z_epochLosInfo.z_ZTDcorr.f_qi, dt);
  }
  else
  {
    LOGI(TAG_PPP, "trop: ssr no ztd,mask=\n", pz_SsrLocBlk->z_epochLosInfo.u_ZTDmask);
  }

  /* free */
  hpp_seq_deinit(&z_seqVar);

  return u_nobs;
}

uint32_t PPP_slantTropMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool)
{
  uint8_t u_idx = 0;
  uint8_t u_sat = 0;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_svid = 0;
  uint8_t u_nobs = 0;
  uint8_t u_isCurrentCloseSky = 0;
  uint16_t w_x_id[3] = { 0 };
  int16_t w_tropIndex = 0;
  char c_sat[4] = "";
  double d_omc = 0.0;
  double d_curR = 0.0;
  double dt = 0.0;
  double* pd_deltaX = pz_PPPfilterInfo->pd_deltaX;
  SeqKalmanVar_t z_seqVar = { 0 };
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_EKFstateRepresent_t* pz_satPool = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;

  if (pz_SsrLocBlk->z_epochLosInfo.u_ZTDmask & GNSS_SSR_ATMO_ZTD_CORR)
  {
    return 0;
  }
  uint8_t seq_rc = hpp_seq_init(&z_seqVar, pz_PPPfilterInfo->w_nmax);
  if (seq_rc != 0)
  {
    return 0;
  }
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }

  w_x_id[0] = 0;
  w_x_id[1] = 0;
  w_x_id[2] = GNSS_FILTER_STATE_ZTD;
  getTargetParaIndex(pz_EKFstateRepPool, w_x_id, &w_tropIndex);
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount && w_tropIndex >= 0; ++u_idx)
  {
    u_sat = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_sat];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 20.0)
    {
      continue;
    }
    u_svid = pz_satMeas->u_svid;
    u_constellation = pz_satMeas->u_constellation;
    /* get sat status and ssr */
    pz_satLos = getSSR_constLos(u_constellation, u_svid, pz_SsrLocBlk);
    if ((NULL == pz_satLos) || !(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STD_CORR) || (fabsf(pz_satLos->z_std.f_qi) > 3.0))
    {
      continue;
    }
    if (integrity_check(pz_satLos->z_std.u_postItg, pz_satLos->z_std.u_preItg) >= 1.0)
    {
      continue;
    }
    dt = fabs(tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_STDtime));
    if (dt > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    hpp_seq_ClearH(&z_seqVar);

    /* H */
    hpp_seq_AddH(w_tropIndex, pz_satMeas->d_wetMap, &z_seqVar); // slant trop

    /* OMC */
    d_omc = pz_satLos->z_std.q_corr * 0.001 - ((pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap)
      + (pd_X[w_tropIndex] * pz_satMeas->d_wetMap));

    d_curR = SQR(0.02);
    if (fabsf(pz_satLos->z_std.f_qi) > 0.0)
    {
      d_curR = SQR((double)pz_satLos->z_std.f_qi);
    }
    if (d_curR > SQR(0.5))
    {
      continue;
    }
    if (dt > 60.0)
    {
      d_curR *= SQR(dt / 60.0);
    }
    if (d_curR < SQR(0.02)) d_curR = SQR(0.02);
    if (1 == u_isCurrentCloseSky && d_curR > 1.0e-6)
    {
      d_curR = 1.0e-6;
    }
    hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
    hpp_seq_PredictStep(pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar);
    hpp_seq_measUpdate(pz_PPPfilterInfo->pq_paraValid, pz_PPPfilterInfo->pd_X, pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar, 0.0);
    ++u_nobs;
    /* print log */
    satidx_SatString(u_sat, c_sat);
    LOGD(TAG_PPP, "Slant trop sat=%s, V=%7.3lf R=%9.3lf qi=%9.3lf dt=%4.1lf\n", c_sat, d_omc, sqrt(d_curR), pz_satLos->z_std.f_qi, dt);
  }
  LOGI(TAG_PPP, "Slant trop ns=%d\n", u_nobs);
  /* free */
  hpp_seq_deinit(&z_seqVar);

  return u_nobs;
}
/**
 * @brief using product of atmosphere to update the EKF
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_ssrStatus is the status of ssr correction
 * @return    the number of updated observations
 */
uint32_t PPP_AtmosMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint32_t w_nbos = 0;

  // update stec measurements
  w_nbos += PPP_stecMeasUpdate(pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs);

  // update ztd measurements
  w_nbos += PPP_ztdMeasUpdate(pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);

  // update slant trop measurements
  w_nbos += PPP_slantTropMeasUpdate(pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);

  return w_nbos;
}
/**
 * @brief using the zero-combine phase observations to update the EKF
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_ssrStatus is the status of ssr correction
 * @return    the number of updated observations
 */
uint32_t PPP_zeroCombinePhaseMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_sig_num = 0;
  uint32_t q_obsNum = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_idx = 0;
  uint8_t u_svh = 0;
  uint8_t u_isCurrentCloseSky = 0;
  char char_sat[4] = "";
  gnss_ConstellationType z_constell = C_GNSS_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  ppp_SatFilterObs_t* pz_SatFilterObs = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t w_ambIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  //int16_t w_clk
  SeqKalmanVar_t z_seqVar = { 0 };
  double vart = 0.0;
  double vari = 0.0;
  double var_rs = 0.0;
  double dt = 0.0;
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double d_dist = 0.0;
  double pd_unitVector[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double d_wave = 0.0;
  double d_beta = 0.0;
  double d_windup = 0.0;
  double ssr_sat_orclk = 0.0;
  double d_gravitation = 0.0;
  double d_minage = 999.99;
  double* pd_deltaX = pz_PPPfilterInfo->pd_deltaX;
  if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
  {
    u_isCurrentCloseSky = 1;
  }
  //get the  site state index in the filter
  if (!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    return 0;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  /* site coordinate */
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
    if (0 == pz_SsrLocBlk->z_epochLosInfo.u_errorModelMask)
    {
      pd_siteCoor[u_i] += pz_satSigMeasCollect->f_earthTide[u_i];
    }
  }
  /* init sequentai filter */
  uint8_t seq_rc = hpp_seq_init(&z_seqVar, pz_PPPfilterInfo->w_nmax);
  if (seq_rc != 0)
  {
    return 0;
  }
  /* begain filter*/
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    u_svh = 0;
    pz_SatFilterObs = NULL;
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_sig_num += PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR);
    if (pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 10.0) // ELE < 10deg
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    /* ssr brdc satellite posclk */
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH) || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
        || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
        continue;
    }
    dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime);
    if (fabs(dt) > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    if (fabs(dt) < d_minage)
    {
      d_minage = fabs(dt);
    }

    if (pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID)
    {
      d_windup = pz_satLos->d_windUp;
    }
    else
    {
      d_windup = 0.0;
    }
    ssr_sat_orclk = pz_satLos->z_orbClk.q_corr * 0.001;
    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[q_satIndex];
    if (NULL != pz_SatFilterObs)
    {
      pz_SatFilterObs->u_orbitCorrStatus = PPP_SSR_CORRECTED;
      pz_SatFilterObs->u_clkCorrStatus = PPP_SSR_CORRECTED;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);

    if (!(pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR))
    {
      d_gravitation = pz_satMeas->d_Gravitation;
    }
    else d_gravitation = 0.0;

    vart = 0.0;
    vari = 0.0;
    var_rs = varianceEphSSR(pz_satLos->z_orbClk.f_qi * 0.001f);
    
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_valid)
      {
        continue;
      }
      z_constell = pz_sigMeas->u_constellation;
      z_algoConstellation = gnss_satType2Algo(pz_sigMeas->u_constellation);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }

      if (w_ZTDindex < 0)
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
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
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (w_ionoIndex < 0 || w_ambIndex < 0)
      {
        continue;
      }
      d_beta = gnss_ionoCoefBaseL1Freq(z_constell, pz_sigMeas->u_signal);
      hpp_seq_ClearH(&z_seqVar);

      /* H */
      for (u_i = 0; u_i < 3; ++u_i)
      {
        hpp_seq_AddH(pw_PVAindex[u_i], -pd_unitVector[u_i], &z_seqVar); // pos
      }
      hpp_seq_AddH(w_rcvClkFilterIndex, 1.0, &z_seqVar); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
          hpp_seq_AddH(w_rcvDcbFilterIndex, 1.0, &z_seqVar); // dcb
      }
      hpp_seq_AddH(w_ZTDindex, pz_satMeas->d_wetMap, &z_seqVar); // trop
      hpp_seq_AddH(w_ionoIndex, -d_beta, &z_seqVar); // iono
      hpp_seq_AddH(w_ambIndex, d_wave, &z_seqVar); // amb

      /* OMC */
      d_omc = (pz_sigMeas->d_carrierPhase - d_windup) * d_wave - (d_dist - pd_satPosClk[3]-d_gravitation);
      d_omc -= (pd_X[w_rcvClkFilterIndex]); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
          d_omc -= (pd_X[w_rcvDcbFilterIndex]); // dcb
      }
      d_omc -= ((pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap) + (pd_X[w_ZTDindex] * pz_satMeas->d_wetMap));
      d_omc -= d_wave * pd_X[w_ambIndex]; // amb
      d_omc -= (-d_beta * pd_X[w_ionoIndex]); // ion
      d_omc -= ssr_sat_orclk;
      d_curR = varianceObs(STOCHASTIC_SNR, pz_satMeas->u_constellation, pz_satMeas->u_svid, C_GNSS_OBS_TYPE_CR, pz_satMeas->z_satPosVelClk.f_elevation,
                           pz_sigMeas->f_cn0);

      if (1 == u_isCurrentCloseSky)
      {
        if ((pz_satMeas->z_satPosVelClk.f_elevation) >= 30.0 * DEG2RAD)
        {
          d_curR *= 4.0;
        }
        else
        {
          d_curR *= 9.0;
        }
      }
      d_curR += vart + vari + var_rs;

      if (NULL != pz_SatFilterObs&& NULL!=pz_SatFilterObs->z_measUpdateFlag[u_signalIndex])
      {
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_valid = 1;
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_cpValid = 1;
      }

      hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
      hpp_seq_PredictStep(pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar);
      hpp_seq_measUpdate(pz_PPPfilterInfo->pq_paraValid, pz_PPPfilterInfo->pd_X, pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar, 0.0);
      ++q_obsNum;

      /* Add integrity information, H, K */
      if (ppp_IntegValid() && ppp_IntegGetIntegrityType() == C_INTEGRITY_FUN_RAIM)
      {
        integ_SignalInfo_t z_si = {0};
        ppp_IntegExtractInfoFromSeqKalmanFilter(z_freqType, &z_seqVar, pw_PVAindex, &z_si);
        ppp_IntegAddSignalInfo(pz_satMeas->u_constellation, pz_satMeas->u_svid, z_freqType, &z_si);
        ppp_IntegAddObsRes(FALSE, pz_satMeas->u_constellation, pz_satMeas->u_svid, z_freqType, d_omc);
      }

      if (0 == u_svh)
      {
        pz_PPPfilterInfo->u_ns++;
        u_svh = 1;
      }
      satidx_SatString(q_satIndex, char_sat);
      LOGD(TAG_PPP, "Lres, sat=%s, ele=%4.1lf, f=%d, V=%8.3lf  R=%7.3lf  QI=%7.3lf, d_pz:%7.3lf\n", char_sat,
        (double)(pz_satMeas->z_satPosVelClk.f_elevation)* RAD2DEG, z_freqType + 1, d_omc, sqrt(d_curR),
        pz_satLos->z_orbClk.f_qi * 0.001, z_seqVar.d_pZ);
    }
  }
  pz_PPPfilterInfo->f_age = (float)d_minage;

  /* free */
  hpp_seq_deinit(&z_seqVar);

  LOGI(TAG_PPP, "phase meas update, all:%d, use:%d, sat:%d, age:%.1f\n", u_sig_num, q_obsNum, pz_PPPfilterInfo->u_ns,
       pz_PPPfilterInfo->f_age);
  pz_FilterObs->u_ppp_use_cp_num_float = q_obsNum;
  return q_obsNum;
}
/**
 * @brief using the zero-combine doppler observations to update the EKF
 * @param[in]  pd_X is value for the state
 * @param[in]  pf_eleSection is section of elevation,[ele0,ele1),ele0 is pf_eleSection[0] and ele1 is pf_eleSection[1]
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_PPPfilterInfo the filter for the PPP algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of updated doppler observations
 */
uint8_t PPP_zeroCombineDopplerMeasUpdate(const double* pd_X, const float pf_eleSection[2], const gnss_ssrLosBlock_t* pz_SsrLocBlk, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, ppp_filterInfo_t* pz_PPPfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_measUpdatDopplerNum = 0;
  uint8_t u_i = 0;
  uint8_t u_idx = 0;
  uint8_t u_seqStatus = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_rcvClkDriftIndex = -1;
  BOOL q_paraValid = TRUE;
  BOOL q_satUpdated = FALSE;
  BOOL q_satInfoValid = TRUE;
  double* pd_deltaX = pz_PPPfilterInfo->pd_deltaX;
  double pd_siteCoor[3] = { 0.0 };
  double pd_siteVel[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  double d_rcvDriftValue = 0.0;
  SeqKalmanVar_t z_seqVar = { 0 };
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const double* pd_satPosClk = NULL;
  const double* pd_satVelClkDrift = NULL;
  double d_omc = 0.0;
  double d_calculatedValue = 0.0;

  //velocity parameter
  getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    if (pw_PVAindex[u_i] < 0)
    {
      q_paraValid = FALSE;
      break;
    }
  }
  //receiver clock drift parameter
  w_id[0] = 0;
  w_id[1] = 0;
  w_id[2] = GNSS_FILTER_STATE_DRIFT;
  pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
  if (NULL != pz_ekfState)
  {
    w_rcvClkDriftIndex = pz_ekfState->w_index;
  }
  if (w_rcvClkDriftIndex < 0 || FALSE == q_paraValid)
  {
    return 0;
  }
  d_rcvDriftValue = pd_X[w_rcvClkDriftIndex];
  //using doppler observations to do measure update
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
    pd_siteVel[u_i] = pd_X[pw_PVAindex[3 + u_i]];
  }
  /* init sequential filter */
  u_seqStatus = hpp_seq_init(&z_seqVar, pz_PPPfilterInfo->w_nmax);
  if (0 != u_seqStatus)
  {
    return 0;
  }
  /* begin filter*/
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if ((pz_satMeas->z_satPosVelClk.f_elevation) < pf_eleSection[0] || (pz_satMeas->z_satPosVelClk.f_elevation) >= pf_eleSection[1])
    {
      continue;
    }
    /* ssr block */
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    /* ssr brdc satellite posclk */
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH))
    {
      continue;
    }
    pd_satVelClkDrift = pz_satLos->z_satPosVelClkBrdc.d_satVelClk;
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    q_satInfoValid = TRUE;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      if (fabs(pd_satPosClk[u_i]) < 1.0e-3 || fabs(pd_satVelClkDrift[u_i]) < 1.0e-3)
      {
        q_satInfoValid = FALSE;
        break;
      }
    }
    if (FALSE == q_satInfoValid)
    {
      continue;
    }
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    q_satUpdated = FALSE;
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (TRUE == q_satUpdated || NULL == pz_sigMeas || !(pz_sigMeas->z_measStatusFlag.b_valid) || !(pz_sigMeas->z_measStatusFlag.b_drValid))
      {
        continue;
      }
      if ((pz_sigMeas->d_dr_unc) <= 0.0)
      {
        continue;
      }
      hpp_seq_ClearH(&z_seqVar);
      /* H */
      for (u_i = 0; u_i < 3; ++u_i)
      {
        hpp_seq_AddH(pw_PVAindex[3 + u_i], -pd_unitVector[u_i], &z_seqVar); // vel
      }
      hpp_seq_AddH(w_rcvClkDriftIndex, 1.0, &z_seqVar); // clk
      d_calculatedValue = 0.0;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        d_calculatedValue += (-pd_unitVector[u_i]) * (pd_siteVel[u_i] - pd_satVelClkDrift[u_i]);
      }
      d_calculatedValue += (d_rcvDriftValue - pd_satVelClkDrift[3]);
      d_omc = (pz_sigMeas->d_doppler) - d_calculatedValue;
      hpp_seq_SetOMC(d_omc, (pz_sigMeas->d_dr_unc), &z_seqVar);
      hpp_seq_PredictStep(pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar);
      hpp_seq_measUpdate(pz_PPPfilterInfo->pq_paraValid, pz_PPPfilterInfo->pd_X, pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar, 0.0);
      ++u_measUpdatDopplerNum;
      q_satUpdated = TRUE;
    }
  }
  /* free */
  hpp_seq_deinit(&z_seqVar);
  return u_measUpdatDopplerNum;
}
/**
 * @brief using the zero-combine code observations to update the EKF
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[out] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_ssrStatus is the status of ssr correction
 * @return    the number of updated observations
 */
uint32_t PPP_zeroCombineCodeMeasUpdate(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_sig_num = 0;
  uint32_t q_obsNum = 0;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_idx = 0;
  uint8_t u_svh = 0;
  uint8_t u_ns = 0;
  char char_sat[4] = "";
  gnss_ConstellationType z_constell = C_GNSS_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalCorr_t* pz_signal_corr = NULL;
  ppp_SatFilterObs_t* pz_SatFilterObs = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  algo_useFreq z_algoFreq = ALGO_NON_FREQ;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  SeqKalmanVar_t z_seqVar = { 0 };
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double d_dist = 0.0;
  double pd_unitVector[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double d_pseudoVarByPvt = 0.0;
  double d_beta = 0.0;
  double ssr_singal_dcb = 0.0;
  double ssr_sat_orclk = 0.0;
  double vart = 0.0;
  double vari = 0.0;
  double var_rs = 0.0;
  double dt = 0.0;
  double d_gravitation = 0.0;
  double* pd_deltaX = pz_PPPfilterInfo->pd_deltaX;
  //get the  site state index in the filter
  if (!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    return 0;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
    if (0 == pz_SsrLocBlk->z_epochLosInfo.u_errorModelMask)
    {
      pd_siteCoor[u_i] += pz_satSigMeasCollect->f_earthTide[u_i];
    }
  }
  /* init sequentai filter */
  uint8_t seq_rc = hpp_seq_init(&z_seqVar, pz_PPPfilterInfo->w_nmax);
  if (seq_rc != 0)
  {
    return 0;
  }
  
  /* loop for sat */
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    u_svh = 0;
    pz_SatFilterObs = NULL;
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    u_sig_num += PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_PR);
    if (pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 10.0)
    {
      continue;
    }
    /* sys */
    z_constell = pz_satMeas->u_constellation;
    z_algoConstellation = gnss_satType2Algo(pz_satMeas->u_constellation);
    if (0 == ((pz_PPPfilterInfo->z_opt.z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    /* ssr block */
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    dt = tm_GpsTimeDiff(&pz_satSigMeasCollect->z_tor, &pz_satLos->z_orbClkTime);
    if (fabs(dt) > PPP_MAX_SSR_AGE)
    {
      continue;
    }
    /* ssr brdc satellite posclk */
    if (0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH) || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
        || 0 == (pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
        continue;
    }
    if (integrity_check(pz_satLos->z_orbClk.u_postItg, pz_satLos->z_orbClk.u_preItg) >= 1.0)
    {
      continue;
    }
    ssr_sat_orclk = pz_satLos->z_orbClk.q_corr * 0.001;
    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[q_satIndex];
    if (NULL != pz_SatFilterObs)
    {
      pz_SatFilterObs->u_orbitCorrStatus = PPP_SSR_CORRECTED;
      pz_SatFilterObs->u_clkCorrStatus = PPP_SSR_CORRECTED;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);

    if (!(pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR))
    {
      d_gravitation = pz_satMeas->d_Gravitation;
    }
    else d_gravitation = 0.0;
    
    vart = 0.0;
    vari = 0.0;
    var_rs = varianceEphSSR(pz_satLos->z_orbClk.f_qi * 0.001f);
    if (var_rs > SQR(1.0) || fabs(ssr_sat_orclk) > 30.0)
    {
      continue;
    }
    satidx_SatString(q_satIndex, char_sat);
    /* loop for signal */
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || !pz_sigMeas->z_measStatusFlag.b_valid)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      z_algoFreq = gnss_freqType2Algo(z_freqType);
      if (0 == ((pz_PPPfilterInfo->z_opt.z_usedFreq) & z_algoFreq))
      {
        continue;
      }
      if (!(pz_sigMeas->z_measStatusFlag.b_prValid))
      {
          continue;
      }
      if (w_ZTDindex < 0)
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
      if (INVALID_INDEX == w_rcvDcbFilterIndex)
      {
        continue;
      }
       if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
          continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = GNSS_FILTER_STATE_IONO;
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
      if (w_ionoIndex < 0)
      {
        continue;
      }
      /* SSR code bias */
      ssr_singal_dcb = 9999.999;
      pz_signal_corr = getSSR_Bias(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_SsrLocBlk);
      if (NULL != pz_signal_corr)
      {
        ssr_singal_dcb = pz_signal_corr->z_codeBias.q_corr * 0.001;
        if (NULL != pz_SatFilterObs&&
          NULL!= pz_SatFilterObs->pz_signalBiasCorrStatus[u_signalIndex])
        {
          pz_SatFilterObs->pz_signalBiasCorrStatus[u_signalIndex]->u_codeBiasCorrStatus = PPP_SSR_CORRECTED;
        }
      }
      /* H */
      hpp_seq_ClearH(&z_seqVar);
      for (u_i = 0; u_i < 3; ++u_i)
      {
        hpp_seq_AddH(pw_PVAindex[u_i], -pd_unitVector[u_i], &z_seqVar);
      }
      hpp_seq_AddH(w_rcvClkFilterIndex, 1.0, &z_seqVar);
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
          hpp_seq_AddH(w_rcvDcbFilterIndex, 1.0, &z_seqVar);
      }
      hpp_seq_AddH(w_ZTDindex, pz_satMeas->d_wetMap, &z_seqVar);
      d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
      hpp_seq_AddH(w_ionoIndex, d_beta, &z_seqVar);
      /* OMC */
      d_omc = (pz_sigMeas->d_pseudoRange) - (d_dist - pd_satPosClk[3]- d_gravitation);
      d_omc -= (pd_X[w_rcvClkFilterIndex]);
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
          d_omc -= (pd_X[w_rcvDcbFilterIndex]); // dcb
      }
      if (fabs(pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][u_signalIndex]) > GNSS_PHASE_CODE_CLK_MAX_BIAS)
      {
        d_omc += pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][u_signalIndex] * wavelength(pz_sigMeas->u_signal);
        LOGI(TAG_PPP, "Clk diff between code and phase , sat=%s, f=%d, diff=%10.3lf\n", char_sat
          , z_freqType + 1, pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][u_signalIndex]);
      }
      d_omc -= ((pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap) + (pd_X[w_ZTDindex] * pz_satMeas->d_wetMap));
      d_omc -= (d_beta * pd_X[w_ionoIndex]);
      d_omc -= ssr_sat_orclk;
      d_curR = varianceObs(STOCHASTIC_SNR, pz_satMeas->u_constellation, pz_satMeas->u_svid, C_GNSS_OBS_TYPE_PR, pz_satMeas->z_satPosVelClk.f_elevation, pz_sigMeas->f_cn0);
      if (fabs(ssr_singal_dcb) < 100.0)
      {
        d_omc += ssr_singal_dcb;
      }
      else
      {
        d_curR *= SQR(3.0);
      }
      d_curR += vart + vari + var_rs;
      /*if (u_signalIndex > 1)
      {
        d_curR *= SQR(1.5);
      }*/
      d_pseudoVarByPvt = 0.0;
      if (0 == (pz_PPPfilterInfo->u_goodSceneStatusFlag))
      {
        d_pseudoVarByPvt = pz_sigMeas->d_pr_unc;
      }
      if (d_pseudoVarByPvt > d_curR)
      {
        d_curR = d_pseudoVarByPvt;
      }

      if (NULL != pz_SatFilterObs && NULL != pz_SatFilterObs->z_measUpdateFlag[u_signalIndex])
      {
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_valid = 1;
        pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_prValid = 1;
      }
      hpp_seq_SetOMC(d_omc, d_curR, &z_seqVar);
      hpp_seq_PredictStep(pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar);
      hpp_seq_measUpdate(pz_PPPfilterInfo->pq_paraValid, pz_PPPfilterInfo->pd_X, pd_deltaX, pz_PPPfilterInfo->pd_Q, &z_seqVar, 0.0);
      ++q_obsNum;

      if (0 == u_svh)
      {
        u_ns++;
        u_svh = 1;
      }

      LOGD(TAG_PPP, "Pres, sat=%s, ele=%4.1lf, f=%d, V=%8.3lf R=%7.3lf\n", char_sat, pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG
        , z_freqType + 1, d_omc, sqrt(d_curR));
    }
  }
  /* free */
  hpp_seq_deinit(&z_seqVar);

  LOGI(TAG_PPP, "code meas update, all:%d, use:%d, sat:%d\n", u_sig_num, q_obsNum, u_ns);
  pz_FilterObs->u_ppp_use_pr_num_float = q_obsNum;
  return q_obsNum;
}

/**
 * @brief ppp start judge
 * @param[in] pz_satSigMeasCollect
 * @param[in] pz_PPPfilterInfo
 * @return 0:sucess, other: failed
 */
static uint8_t ppp_StartJudge(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,const ppp_filterInfo_t* pz_PPPfilterInfo)
{
  /* pvt xyz>0 */
  if (gnss_Norm(pz_satSigMeasCollect->z_positionFix.d_xyz, 3) < FABS_ZEROS
    || pz_PPPfilterInfo->u_start)
  {
    return 1;
  }
  return 0;
}
/**
 * @brief reset value of ppp float
 * @param[in]  pz_satSigMeasCollect
 * @param[out] pz_PPPfilterInfo
 * @param[out] pz_FilterObs
 * @param[out] pd_siteCoor
 * @param[out] pd_siteVel
 * @return     none
 */
static void PPP_EpochFilterInfoInit(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, ppp_filterInfo_t* pz_PPPfilterInfo,
  ppp_EpochFilterObs_t* pz_FilterObs, double* pd_siteCoor, double* pd_siteVel)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;

  pz_FilterObs->u_ppp_use_cp_num_float = 0;
  pz_FilterObs->u_ppp_use_dr_num_float = 0;
  pz_FilterObs->u_ppp_use_pr_num_float = 0;

  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pz_satSigMeasCollect->z_positionFix.d_xyz[u_i];
    pd_siteVel[u_i] = pz_satSigMeasCollect->z_positionFix.f_velXyz[u_i];
  }

  pz_FilterObs->z_tor = pz_satSigMeasCollect->z_tor;

  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; ++u_i)
  {
    if (NULL == pz_FilterObs->pz_SatFilterObs[u_i])
    {
      continue;
    }
    pz_FilterObs->pz_SatFilterObs[u_i]->u_constellation = C_GNSS_MAX;
    pz_FilterObs->pz_SatFilterObs[u_i]->u_svid = 0;
    pz_FilterObs->pz_SatFilterObs[u_i]->u_orbitCorrStatus = PPP_NO_SSR_CORRECTED;
    pz_FilterObs->pz_SatFilterObs[u_i]->u_clkCorrStatus = PPP_NO_SSR_CORRECTED;
    pz_FilterObs->pz_SatFilterObs[u_i]->u_IonoCorrStatus = PPP_NO_SSR_CORRECTED;
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; ++u_j)
    {
      if (NULL == pz_FilterObs->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j])
      {
        continue;
      }
      pz_FilterObs->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j]->u_codeBiasCorrStatus = PPP_NO_SSR_CORRECTED;
      pz_FilterObs->pz_SatFilterObs[u_i]->pz_signalBiasCorrStatus[u_j]->u_phaseBiasCorrStatus = PPP_NO_SSR_CORRECTED;
    }
  }
  for (u_i = 0; u_i < C_GNSS_MAX; ++u_i)
  {
    for (u_j = 0; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
      pz_PPPfilterInfo->pd_phaseCodeClkDiff[u_i][u_j] = 0.0;
    }
  }
}

/**
 * @brief computing residual of zero-combine phase observations
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in] pz_fixedAmbPool is the information of amb fix
 * @param[out] pz_FilterObs is the information of observation after filter
 * @param[in] z_residualType is the type of residual
 * @return    the number of updated observations
 */
uint32_t PPP_zeroCombinePhaseMeasResidual(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, ppp_EpochFilterObs_t* pz_FilterObs, PPP_ResidualType z_residualType)
{
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_idx = 0;
  uint16_t w_resNumPhase = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  uint32_t q_satIndex = 0;
  char char_sat[4] = "";
  char* u_resType[RES_MAX] = { "RES_PRIOR","RES_POST","RES_FIXED","RES_PREFIX" };
  gnss_ConstellationType z_constell = C_GNSS_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  ppp_SatFilterObs_t* pz_SatFilterObs = NULL;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t w_ambIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  //int16_t w_clk
  double vart = 0.0;
  double vari = 0.0;
  double var_rs = 0.0;
  double dt = 0.0;
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double d_dist = 0.0;
  double pd_unitVector[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double d_wave = 0.0;
  double d_beta = 0.0;
  double d_windup = 0.0;
  double ssr_sat_orclk = 0.0;
  double d_stdPhase = 0.0;
  double d_gravitation = 0.0;

  //get the  site state index in the filter
  if (!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    return 0;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  /* site coordinate */
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
    if (0 == pz_SsrLocBlk->z_epochLosInfo.u_errorModelMask)
    {
      pd_siteCoor[u_i] += pz_satSigMeasCollect->f_earthTide[u_i];
    }
  }
  
  /* begain filter*/
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_SatFilterObs = NULL;
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 10.0)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
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
      d_windup = 0.0;
    }
    ssr_sat_orclk = pz_satLos->z_orbClk.q_corr * 0.001;
    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);

    if (!(pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR))
    {
      d_gravitation = pz_satMeas->d_Gravitation;
    }
    else d_gravitation = 0.0;

    vart = 0.0;
    vari = 0.0;
    var_rs = varianceEphSSR(pz_satLos->z_orbClk.f_qi * 0.001f);
    if ((RES_FIXED == z_residualType || RES_PREFIX == z_residualType) &&
      (NULL == pz_fixedAmbPool || NULL == pz_fixedAmbPool->pz_fixedAmbSet[q_satIndex]
        || GNSS_NONE_AMB_FIXED == pz_fixedAmbPool->pz_fixedAmbSet[q_satIndex]->u_ambFixType))
    {
      continue;
    }
    satidx_SatString(q_satIndex, char_sat);
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || NULL== pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]
        ||NULL== pz_SatFilterObs->pf_phaseResidual[u_signalIndex]|| NULL == pz_SatFilterObs->pf_phaseVar[u_signalIndex])
      {
        continue;
      }
      if (!pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_cpValid)
      {
        continue;
      }
      z_constell = pz_sigMeas->u_constellation;
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);

      if (w_ZTDindex < 0)
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
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
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = convertFreqAmb2FilterType(z_freqType);
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ambIndex);
      if (w_ionoIndex < 0 || w_ambIndex < 0)
      {
        continue;
      }
      d_beta = gnss_ionoCoefBaseL1Freq(z_constell, pz_sigMeas->u_signal);

      /* OMC */
      d_omc = (pz_sigMeas->d_carrierPhase - d_windup) * d_wave - (d_dist - pd_satPosClk[3]- d_gravitation);
      d_omc -= (pd_X[w_rcvClkFilterIndex]); // clk
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        d_omc -= (pd_X[w_rcvDcbFilterIndex]); // dcb
      }
      d_omc -= ((pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap) + (pd_X[w_ZTDindex] * pz_satMeas->d_wetMap));
      d_omc -= d_wave * pd_X[w_ambIndex]; // amb
      d_omc -= (-d_beta * pd_X[w_ionoIndex]); // ion
      d_omc -= ssr_sat_orclk;
      d_curR = varianceObs(STOCHASTIC_SNR, pz_satMeas->u_constellation, pz_satMeas->u_svid, C_GNSS_OBS_TYPE_CR, pz_satMeas->z_satPosVelClk.f_elevation,
        pz_sigMeas->f_cn0);
      d_curR += vart + vari + var_rs;

      *(pz_SatFilterObs->pf_phaseResidual[u_signalIndex]) = (float)d_omc;
      *(pz_SatFilterObs->pf_phaseVar[u_signalIndex]) = (float)sqrt(d_curR);

      /* integrity: save pos residual */
      if (ppp_IntegGetIntegrityType() == C_INTEGRITY_FUN_RAIM)
      {
        ppp_IntegAddObsRes(FALSE, pz_satMeas->u_constellation, pz_satMeas->u_svid, z_freqType, d_omc);
      }
      if (z_residualType == RES_POST && fabs(d_omc) > 0.1 && fabs(d_omc / sqrt(d_curR)) > 2.0)
      {
        gnss_filterInitInfo_t z_initInfo = { 0 };
        z_initInfo.u_resetPara = 1;
        z_initInfo.d_initValue = pd_X[w_ambIndex];
        z_initInfo.z_paraTime = pz_satSigMeasCollect->z_tor;
        z_initInfo.d_sigma0 = AMB_SIGMA;
        z_initInfo.d_noise = AMB_NOISE;
        w_id[0] = 0;
        w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
        w_id[2] = convertFreqAmb2FilterType(z_freqType);
        addState2EKFfilter(w_id, pz_PPPfilterInfo->w_nmax, &z_initInfo, pz_PPPfilterInfo->pd_X, pz_PPPfilterInfo->pd_Q, pz_PPPfilterInfo->pq_paraValid, pz_EKFstateRepPool);

        LOGI(TAG_PPP, "Ret amb by post residual, sat=%s, f=%d, V=%7.3lf  R=%7.3lf\n", char_sat, z_freqType + 1, d_omc, sqrt(d_curR));
      }

      LOGD(TAG_PPP, "Post-Lres, sat=%s, f=%d, V=%7.3lf  R=%7.3lf\n", char_sat, z_freqType + 1, d_omc*1000.0, sqrt(d_curR));

      d_stdPhase += d_omc * d_omc;
      w_resNumPhase++;
    }
  }
  if (w_resNumPhase > 1)
  {
    d_stdPhase = sqrt(d_stdPhase / (w_resNumPhase - 1));
    switch (z_residualType)
    {
    case(RES_POST):
      pz_FilterObs->f_postPhaseSTD = (float)d_stdPhase; break;
    case(RES_PRIOR):
      pz_FilterObs->f_priorPhaseSTD = (float)d_stdPhase; break;
    case(RES_FIXED):
      pz_FilterObs->f_fixedPhaseSTD = (float)d_stdPhase; break;
    case(RES_PREFIX):
      pz_FilterObs->f_prefixPhaseSTD = (float)d_stdPhase; break;
    }
    LOGI(TAG_PPP, "-- %s -- num=%d | phase std=%.3f\n", u_resType[z_residualType],
      w_resNumPhase, d_stdPhase);
  }
  else
  {
    LOGI(TAG_HPP, "-- %s -- phase res is NULL\n", u_resType[z_residualType]);
  }
  return w_resNumPhase;
}
/**
 * @brief computing std of data,delete the largest one
 * @return    std of code residual
 */
static double normCodeResStd(double* pd_moc, uint16_t w_n)
{
  uint16_t w_i = 0;
  uint16_t w_norm = 0;
  double d_std = 0.0f;
  double d_mean = 0.0;
  double* pd_norm_moc = NULL;

  if (w_n <= 1) return 0.0;

  for (w_i = 0; w_i < w_n; w_i++)
  {
    d_std += pd_moc[w_i] * pd_moc[w_i];
    d_mean += fabs(pd_moc[w_i]);
  }
  d_std = (sqrt(d_std / (w_n - 1)) + 1e-4);  //1e-4, avoid zero
  d_mean = (d_mean / w_n + 1e-4);
  if (w_n <= 10)
  {
    return d_std;
  }
  for (w_i = 0; w_i < w_n; w_i++) pd_moc[w_i] = fabs(pd_moc[w_i]);
  gnss_ascSortDouble(pd_moc, w_n);

  pd_norm_moc = (double*)OS_MALLOC_FAST(sizeof(double) * w_n);
  for (w_i = 0; w_i < w_n; w_i++) // delete the largest one
  {
    if (w_i >= (w_n - 1) && ((fabs(pd_moc[w_i] / d_std) > 3.0 && fabs(pd_moc[w_i]) > 5.0)
      || (fabs(pd_moc[w_i] / d_std) > 2.0 && fabs(pd_moc[w_i]) > 10.0)
      || (fabs(pd_moc[w_i] / d_mean) > 3.0 && fabs(pd_moc[w_i]) > 8.0)))
    {
      continue;
    }
    pd_norm_moc[w_norm++] = pd_moc[w_i];
  }
  
  for (w_i = 0; w_i < w_norm; w_i++)
  {
    d_std += pd_norm_moc[w_i] * pd_norm_moc[w_i];
  }
  d_std = sqrt(d_std / (w_norm - 1));

  OS_FREE(pd_norm_moc);
  return d_std;
}

/**
 * @brief computing std of code residual
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in] pz_FilterObs is the status of ssr correction
 * @return    std of code residual
 */
static double codeResidualStd(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_i = 0;
  uint8_t u_isys = 0;
  uint8_t u_freq = 0;
  uint8_t u_nv = 0;
  uint8_t u_nf = 0;
  double d_std = 0.0;
  double d_mean = 0.0;
  double d_std_fre = 0.0;
  double* pd_dmoc_all = NULL;
  double* pd_dmoc_fre = NULL;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const ppp_SatFilterObs_t* pz_SatFilterObs = NULL;

  pd_dmoc_all = (double*)OS_MALLOC_FAST(sizeof(double) * pz_satSigMeasCollect->u_satMeasCount * MAX_GNSS_SIGNAL_FREQ);
  pd_dmoc_fre = (double*)OS_MALLOC_FAST(sizeof(double) * pz_satSigMeasCollect->u_satMeasCount);

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    for (u_freq = 0; u_freq < MAX_GNSS_SIGNAL_FREQ; u_freq++)
    {
      u_nf = 0; d_mean = 0.0; d_std_fre = 0.0;
      for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
      {
        pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_i];
        if (NULL == pz_satMeas || u_isys != pz_satMeas->u_constellation)
        {
          continue;
        }
        pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[u_i];
        if (NULL == pz_SatFilterObs)
        {
          continue;
        }
        if (NULL == pz_SatFilterObs->z_measUpdateFlag[u_freq]
          || NULL == pz_SatFilterObs->pf_codeResidual[u_freq])
        {
          continue;
        }
        if (!pz_SatFilterObs->z_measUpdateFlag[u_freq]->b_prValid || fabs(*(pz_SatFilterObs->pf_codeResidual[u_freq])) < FABS_ZEROS)
        {
          continue;
        }
        pd_dmoc_fre[u_nf++] = *(pz_SatFilterObs->pf_codeResidual[u_freq]);
      }
      gnss_ascSortMedianStdDouble(pd_dmoc_fre, u_nf, &d_mean, &d_std_fre);
      if (d_std_fre > 1.0) d_mean = 0.0;
      for (u_i = 0; u_i < u_nf; u_i++)
      {
        pd_dmoc_all[u_nv++] = pd_dmoc_fre[u_i] - d_mean;
      }
    }
  }
  d_std = normCodeResStd(pd_dmoc_all, u_nv);

  OS_FREE(pd_dmoc_all);
  OS_FREE(pd_dmoc_fre);

  return d_std;
}
/**
 * @brief computing residual of  the zero-combine code observations
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in] pz_fixedAmbPool is the information of amb fix
 * @param[out] pz_FilterObs is the status of ssr correction
 * @param[in] z_residualType is the type of residual
 * @return    the number of updated observations
 */
uint32_t PPP_zeroCombineCodeMeasResidual(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, ppp_EpochFilterObs_t* pz_FilterObs, PPP_ResidualType z_residualType)
{
  
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  uint8_t u_idx = 0;
  uint16_t w_resNumCode = 0;
  uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
  uint32_t q_satIndex = 0;
  char char_sat[4] = "";
  char* u_resType[RES_MAX] = { "RES_PRIOR","RES_POST","RES_FIXED","RES_PREFIX" };
  gnss_ConstellationType z_constell = C_GNSS_MAX;
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;
  const gnss_SignalCorr_t* pz_signal_corr = NULL;
  ppp_SatFilterObs_t* pz_SatFilterObs = NULL;
  int16_t pw_PVAindex[PVA_NUM] = { 0 };
  int16_t w_ZTDindex = 0;
  int16_t w_ionoIndex = -1;
  int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX] = { 0 };
  int16_t w_rcvClkFilterIndex = -1;
  int16_t w_rcvDcbFilterIndex = -1;
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double d_dist = 0.0;
  double pd_unitVector[3] = { 0.0 };
  const double* pd_satPosClk = NULL;
  double d_omc = 0.0;
  double d_curR = 0.0;
  double d_beta = 0.0;
  double ssr_singal_dcb = 0.0;
  double ssr_sat_orclk = 0.0;
  double d_gravitation = 0.0;
  double vart = 0.0;
  double vari = 0.0;
  double var_rs = 0.0;
  double d_stdCode = 0.0;

  //get the  site state index in the filter
  if (!getSiteRelativeParaIndex(pz_EKFstateRepPool, pw_PVAindex, &w_ZTDindex, pw_rcvClkIndex))
  {
    return 0;
  }
  if (FALSE == (pz_PPPfilterInfo->z_opt.q_isSperateBDS2And3))
  {
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
      pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS2] = pw_rcvClkIndex[u_i * C_GNSS_MAX + C_GNSS_BDS3];
    }
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pd_X[pw_PVAindex[u_i]];
    if (0 == pz_SsrLocBlk->z_epochLosInfo.u_errorModelMask)
    {
      pd_siteCoor[u_i] += pz_satSigMeasCollect->f_earthTide[u_i];
    }
  }
 
  /* loop for sat */
  for (u_idx = 0; u_idx < pz_satSigMeasCollect->u_satMeasCount; ++u_idx)
  {
    q_satIndex = pz_satSigMeasCollect->u_satMeasIdxTable[u_idx];
    pz_SatFilterObs = NULL;
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 10.0)
    {
      continue;
    }
    /* sys */
    z_constell = pz_satMeas->u_constellation;
    /* ssr block */
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos)
    {
      continue;
    }
    ssr_sat_orclk = pz_satLos->z_orbClk.q_corr * 0.001;
    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    pd_satPosClk = pz_satLos->z_satPosVelClkBrdc.d_satPosClk;
    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);

    if (!(pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR))
    {
      d_gravitation = pz_satMeas->d_Gravitation;
    }
    else d_gravitation = 0.0;

    vart = 0.0;
    vari = 0.0;
    var_rs = varianceEphSSR(pz_satLos->z_orbClk.f_qi * 0.001f);
    satidx_SatString(q_satIndex, char_sat);
    if ((RES_FIXED == z_residualType || RES_PREFIX == z_residualType) &&
      (NULL == pz_fixedAmbPool || NULL == pz_fixedAmbPool->pz_fixedAmbSet[q_satIndex]
        || GNSS_NONE_AMB_FIXED == pz_fixedAmbPool->pz_fixedAmbSet[q_satIndex]->u_ambFixType))
    {
      continue;
    }

    /* loop for signal */
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas || NULL == pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]
        || NULL == pz_SatFilterObs->pf_codeResidual[u_signalIndex] || NULL == pz_SatFilterObs->pf_codeVar[u_signalIndex])
      {
        continue;
      }
      if (!pz_SatFilterObs->z_measUpdateFlag[u_signalIndex]->b_prValid)
      {
        continue;
      }
      z_freqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (w_ZTDindex < 0)
      {
        continue;
      }
      w_rcvDcbFilterIndex = pw_rcvClkIndex[z_freqType * C_GNSS_MAX + z_constell];
      w_rcvClkFilterIndex = pw_rcvClkIndex[z_constell];
      if (INVALID_INDEX == w_rcvDcbFilterIndex)
      {
        continue;
      }
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType && INVALID_INDEX == w_rcvClkFilterIndex)
      {
        continue;
      }
      w_id[0] = 0;
      w_id[1] = gnss_cvt_Svid2SvIndex(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      w_id[2] = GNSS_FILTER_STATE_IONO;
      getTargetParaIndex(pz_EKFstateRepPool, w_id, &w_ionoIndex);
      if (w_ionoIndex < 0)
      {
        continue;
      }
      /* SSR code bias */
      ssr_singal_dcb = 9999.999;
      pz_signal_corr = getSSR_Bias(pz_sigMeas->u_constellation, pz_sigMeas->u_svid, pz_sigMeas->u_signal, pz_SsrLocBlk);
      if (NULL != pz_signal_corr)
      {
        ssr_singal_dcb = pz_signal_corr->z_codeBias.q_corr * 0.001;
      }
      d_beta = gnss_ionoCoefBaseL1Freq(pz_sigMeas->u_constellation, pz_sigMeas->u_signal);
      /* OMC */
      d_omc = (pz_sigMeas->d_pseudoRange) - (d_dist - pd_satPosClk[3]-d_gravitation);
      d_omc -= (pd_X[w_rcvClkFilterIndex]);
      if (C_GNSS_FREQ_TYPE_L1 != z_freqType)
      {
        d_omc -= (pd_X[w_rcvDcbFilterIndex]); // dcb
      }
      if (fabs(pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][u_signalIndex]) > GNSS_PHASE_CODE_CLK_MAX_BIAS)
      {
        d_omc += pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][u_signalIndex] * wavelength(pz_sigMeas->u_signal);
        LOGI(TAG_PPP, "Clk diff between code and phase , sat=%s, f=%d, diff=%10.3lf\n", char_sat
          , z_freqType + 1, pz_PPPfilterInfo->pd_phaseCodeClkDiff[pz_sigMeas->u_constellation][u_signalIndex]);
      }
      d_omc -= ((pz_satSigMeasCollect->d_zhd_emp * pz_satMeas->d_dryMap) + (pd_X[w_ZTDindex] * pz_satMeas->d_wetMap));
      d_omc -= (d_beta * pd_X[w_ionoIndex]);
      d_omc -= ssr_sat_orclk;
      d_curR = varianceObs(STOCHASTIC_SNR, pz_satMeas->u_constellation, pz_satMeas->u_svid, C_GNSS_OBS_TYPE_PR, pz_satMeas->z_satPosVelClk.f_elevation, pz_sigMeas->f_cn0);
      if (fabs(ssr_singal_dcb) < 100.0)
      {
        d_omc += ssr_singal_dcb;
      }
      else
      {
        d_curR *= SQR(3.0);
      }
      d_curR += vart + vari + var_rs;

      *(pz_SatFilterObs->pf_codeResidual[u_signalIndex]) = (float)d_omc;
      *(pz_SatFilterObs->pf_codeVar[u_signalIndex]) = (float)sqrt(d_curR);

      LOGD(TAG_PPP, "Post-Pres, sat=%s, f=%d, V=%7.3lf  R=%7.3lf\n", char_sat, z_freqType + 1, d_omc, sqrt(d_curR));

      d_stdCode += d_omc * d_omc;
      w_resNumCode++;
    }
  }
  
  if (w_resNumCode > 1)
  {
    if (0x1 == (pz_PPPfilterInfo->t_closeSkyCount & 0x1))
    {
      d_stdCode = sqrt(d_stdCode / (w_resNumCode - 1));
    }
    else
    {
      d_stdCode = codeResidualStd(pz_satSigMeasCollect, pz_FilterObs);
    }
    switch (z_residualType)
    {
    case(RES_POST):
      pz_FilterObs->f_postCodeSTD = (float)d_stdCode; break;
    case(RES_PRIOR):
      pz_FilterObs->f_priorCodeSTD = (float)d_stdCode; break;
    case(RES_FIXED):
      pz_FilterObs->f_fixedCodeSTD = (float)d_stdCode; break;
    case(RES_PREFIX):
      pz_FilterObs->f_prefixCodeSTD = (float)d_stdCode; break;
    }
    LOGI(TAG_PPP, "-- %s -- num=%d | code std=%.3f\n", u_resType[z_residualType],
      w_resNumCode, d_stdCode);
  }
  else
  {
    LOGI(TAG_HPP, "-- %s -- code res is NULL\n", u_resType[z_residualType]);
  }

  return w_resNumCode;
}

/**
 * @brief          PPP float solution availability check
 * @param[in]      u_phaseNum
 * @param[in]      u_codeNum
 * @param[in]      pz_FilterObs
 * @param[in]      pz_EKFstateRepPool
 * @param[out]     pz_PPPfilterInfo
 * @return         1: OK 0: fail
 */
static uint8_t PPP_floatSolCheck(uint8_t u_phaseNum, uint8_t u_codeNum, const ppp_EpochFilterObs_t* pz_FilterObs,
  const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_filterInfo_t* pz_PPPfilterInfo)
{
  BOOL b_openSky = FALSE;
  uint8_t u_i = 0;
  uint8_t u_signalIndex = 0;
  uint16_t w_rejCodeNum = 0;
  uint16_t w_rejPhaseNum = 0;
  uint16_t w_acceptCodeNum = 0;
  uint16_t w_acceptPhaseNum = 0;
  uint32_t q_satIndex = 0;
  double d_horVel = 0.0;
  double d_enu_vel[3] = { 0.0 };

  /* threshold define begin */
  float f_CodeThresOpen = 5.0; //max code res value in open sky (m)
  float f_PhaseThresOpen = 1.0; //max phase res value in open sky (m)
  float f_CodeThresClose = 10.0; //max code res value in close sky (m)
  float f_PhaseThresClose = 2.0; //max phase res value in close sky (m)
  /* threshold define end */
  const ppp_SatFilterObs_t* pz_SatFilterObs = NULL;

  if (pz_PPPfilterInfo == NULL || pz_FilterObs == NULL)
  {
    return 0;
  }
  if (u_phaseNum <= 5|| u_codeNum <= 5)
  {
    pz_PPPfilterInfo->w_algStatus |= PPP_STATUS_LITTLE_SAT_PPP;
    return 0;
  }

  getEnuVel(pz_EKFstateRepPool, pz_PPPfilterInfo->pd_X, d_enu_vel);
  d_horVel = sqrt(d_enu_vel[0] * d_enu_vel[0] + d_enu_vel[1] * d_enu_vel[1]);
  LOGI(TAG_PPP, "Float velocity Info: H=%.3lf   V=%.3lf\n", d_horVel, d_enu_vel[2]);
  if (d_horVel > 300 || fabs(d_enu_vel[2]) > 15.0)
  {
    pz_PPPfilterInfo->z_kfStatus = KF_RESET;
    LOGW(TAG_PPP, "Float velocity is bad: H=%.3lf   V=%.3lf\n", d_horVel, d_enu_vel[2]);
    return 0;
  }

  // statistic updated outline num
  if (1 == pz_PPPfilterInfo->u_goodSceneStatusFlag)
  {
    b_openSky = TRUE;
  }
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_SatFilterObs = pz_FilterObs->pz_SatFilterObs[q_satIndex];
    if (NULL == pz_SatFilterObs)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
    {
      if (NULL == pz_SatFilterObs->pf_phaseResidual[u_signalIndex]
        || NULL == pz_SatFilterObs->pf_codeResidual[u_signalIndex])
      {
        continue;
      }
      if (b_openSky)
      {
        if (*pz_SatFilterObs->pf_phaseResidual[u_signalIndex] > f_PhaseThresOpen)
        {
          w_rejPhaseNum++;
        }
        if (*pz_SatFilterObs->pf_codeResidual[u_signalIndex] > f_CodeThresOpen)
        {
          w_rejCodeNum++;
        }
      }
      else
      {
        if (*pz_SatFilterObs->pf_phaseResidual[u_signalIndex] > f_PhaseThresClose)
        {
          w_rejPhaseNum++;
        }
        if (*pz_SatFilterObs->pf_codeResidual[u_signalIndex] > f_CodeThresClose)
        {
          w_rejCodeNum++;
        }
      }
    }
  }
  w_acceptCodeNum = u_codeNum - w_rejCodeNum;
  w_acceptPhaseNum = u_phaseNum - w_rejPhaseNum;

  /* judge with
  *  a. % of outage res
  *  b. std of code/phase res
  */
  if (w_acceptCodeNum < 6 && w_acceptPhaseNum < 6)
  {
    if (w_rejCodeNum >= u_codeNum * 0.50 || w_rejPhaseNum >= u_phaseNum * 0.50)
    {
      pz_PPPfilterInfo->z_kfStatus = KF_RESET;
    }
  }
  else if (w_rejCodeNum >= u_codeNum * 0.80 || w_rejPhaseNum >= u_phaseNum * 0.67 ||
    pz_FilterObs->f_postCodeSTD > 20.0 || pz_FilterObs->f_postPhaseSTD > 2.0)
  {
    pz_PPPfilterInfo->z_kfStatus = KF_RESET;
  }
  
  
  if (KF_RESET == pz_PPPfilterInfo->z_kfStatus)
  {
    LOGW(TAG_PPP, "PPP filter is needed to reset, b_openSky=%u, ncode=%d %d     nphase=%d %d    codeSTD=%.3lf  phaseSTD=%.3lf\n"
      , b_openSky, w_rejCodeNum, u_codeNum, w_rejPhaseNum, u_phaseNum, pz_FilterObs->f_postCodeSTD, pz_FilterObs->f_postPhaseSTD);
  }
  
  return 1;
}
/**
 * @brief          get sort quality list for every sys
 * @param[in]      pz_satSigMeasCollect
 * @param[in]      pz_SsrLocBlk
 * @param[in]      u_constellation
 * @param[out]     u_satidx
 * @return         u_ns
 */
static uint8_t PPP_sortSysMeasList(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk
  , gnss_ConstellationType u_constellation, uint8_t* u_satidx)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_temp = 0;
  uint8_t u_ns = 0;
  gnss_ConstellationType u_constellationObs = 0;
  float f_quality[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0.0f };
  float f_temp = 0.0f;
  float f_minsnr = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  /* get list of good quality */
  u_ns = 0;
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; ++u_i)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_i];
    if (NULL == pz_satMeas || u_ns >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      continue;
    }
    u_constellationObs = pz_satMeas->u_constellation;
    if (C_GNSS_BDS2 == u_constellationObs)
    {
      u_constellationObs = C_GNSS_BDS3;
    }
    if (u_constellation != u_constellationObs
      || pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG < 20.0)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }
    f_minsnr = 100.0;
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      if (NULL != pz_satMeas->pz_signalMeas[u_j] && pz_satMeas->pz_signalMeas[u_j]->f_cn0 > 0.0)
      {
        if (pz_satMeas->pz_signalMeas[u_j]->f_cn0 < f_minsnr) f_minsnr = pz_satMeas->pz_signalMeas[u_j]->f_cn0;
      }
    }
    if (f_minsnr < 25.0)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
      || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR)
      || !(pz_satLos->u_windUpMask & GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID))
    {
      continue;
    }
    if ((PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_PR) <= 1 || PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR) <= 1)
      && (!(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR) || fabsf(pz_satLos->z_stec.f_qi) > 10.0))
    {
      continue;
    }
    u_satidx[u_ns] = u_i;
    f_quality[u_ns] = (float)(pz_satMeas->z_satPosVelClk.f_elevation * RAD2DEG);
    u_ns++;
  }
  /* sort list */
  for (u_i = 0; u_i < u_ns; ++u_i)
  {
    for (u_j = u_i + 1; u_j < u_ns; ++u_j)
    {
      if (f_quality[u_j] > f_quality[u_i])
      {
        f_temp = f_quality[u_i]; f_quality[u_i] = f_quality[u_j]; f_quality[u_j] = f_temp;
        u_temp = u_satidx[u_i]; u_satidx[u_i] = u_satidx[u_j]; u_satidx[u_j] = u_temp;
      }
    }
  }

  return u_ns;
}
/**
 * @brief          get sort quality list all of satellites
 * @param[in/out]  pz_satSigMeasCollect
 * @param[in]      pz_SsrLocBlk
 * @return         none
 */
static void PPP_sortMeasQuality(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_ns = 0;
  uint8_t u_ns_good = 0;
  uint8_t u_satidx[MAX_GNSS_ACTIVE_SAT_NUMBER] = { 0 };
  float f_minsnr = 0.0;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_satSsrLos_t* pz_satLos = NULL;

  for (u_i = 0; u_i < MAX_GNSS_ACTIVE_SAT_NUMBER; ++u_i)
  {
    pz_satSigMeasCollect->u_satMeasIdxTable[u_i] = 0;
  }

  u_ns = PPP_sortSysMeasList(pz_satSigMeasCollect, pz_SsrLocBlk, C_GNSS_GPS, u_satidx); // GPS
  for (u_i = 0; u_i < u_ns; ++u_i)
  {
    if (u_ns_good >= MAX_GNSS_ACTIVE_SAT_NUMBER) continue;
    pz_satSigMeasCollect->u_satMeasIdxTable[u_ns_good++] = u_satidx[u_i];
  }
  u_ns = PPP_sortSysMeasList(pz_satSigMeasCollect, pz_SsrLocBlk, C_GNSS_GAL, u_satidx); // GAL
  for (u_i = 0; u_i < u_ns; ++u_i)
  {
    if (u_ns_good >= MAX_GNSS_ACTIVE_SAT_NUMBER) continue;
    pz_satSigMeasCollect->u_satMeasIdxTable[u_ns_good++] = u_satidx[u_i];
  }
  u_ns = PPP_sortSysMeasList(pz_satSigMeasCollect, pz_SsrLocBlk, C_GNSS_BDS3, u_satidx); // BDS
  for (u_i = 0; u_i < u_ns; ++u_i)
  {
    if (u_ns_good >= MAX_GNSS_ACTIVE_SAT_NUMBER) continue;
    pz_satSigMeasCollect->u_satMeasIdxTable[u_ns_good++] = u_satidx[u_i];
  }
  pz_satSigMeasCollect->u_satMeasCount = u_ns_good;

  if (pz_satSigMeasCollect->u_satMeasCount >= 25) return;

  /* other satellites added */
  for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; ++u_i)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_i];
    if (NULL == pz_satMeas || pz_satSigMeasCollect->u_satMeasCount >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      continue;
    }
    if (C_SAT_TYPE_GEO == svid2SatType(pz_satMeas->u_constellation, pz_satMeas->u_svid))
    {
      continue;
    }
    if (pz_satSigMeasCollect->u_satMeasCount >= 25)
    {
      break;
    }
    f_minsnr = 100.0;
    for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
    {
      if (NULL != pz_satMeas->pz_signalMeas[u_j] && pz_satMeas->pz_signalMeas[u_j]->f_cn0 > 0.0)
      {
        if (pz_satMeas->pz_signalMeas[u_j]->f_cn0 < f_minsnr) f_minsnr = pz_satMeas->pz_signalMeas[u_j]->f_cn0;
      }
    }
    if (f_minsnr < 20.0)
    {
      continue;
    }
    pz_satLos = getSSR_constLos(pz_satMeas->u_constellation, pz_satMeas->u_svid, pz_SsrLocBlk);
    if (NULL == pz_satLos || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_EPH)
      || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR)
      || !(pz_satLos->u_orbClkMask & GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR))
    {
      continue;
    }
    if (pz_satSigMeasCollect->u_satMeasCount >= 15 &&
      (PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_PR) <= 1 || PPP_signalNum(pz_satMeas, C_GNSS_OBS_TYPE_CR) <= 1)
      && (!(pz_satLos->u_atmoMask & GNSS_SSR_ATMO_STEC_CORR) || fabsf(pz_satLos->z_stec.f_qi) > 10.0))
    {
      continue;
    }

    for (u_j = 0; u_j < pz_satSigMeasCollect->u_satMeasCount; u_j++)
    {
      if (u_i == pz_satSigMeasCollect->u_satMeasIdxTable[u_j]) break;
    }
    if (u_j >= pz_satSigMeasCollect->u_satMeasCount)
    {
      pz_satSigMeasCollect->u_satMeasIdxTable[pz_satSigMeasCollect->u_satMeasCount++] = u_i;
    }
  }
}
/**
 * @brief the inteface of PPP-RTK float algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[out]     pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pz_ssrStatus is status of ssr correction
 * @return         0 represent success and other failed
 */
uint8_t PPP_filterSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_EpochFilterObs_t* pz_FilterObs)
{
  uint8_t u_status = 0;
  uint16_t w_i = 0;
  uint8_t u_nL = 0;
  uint8_t u_nP = 0;
  uint8_t u_nD = 0;
  double pd_siteCoor[3] = { 0.0 };//to be filled
  double pd_siteVel[3] = { 0.0 };//to be filled
  float pf_eleSection[2] = { (float)(30.0 * DEG2RAD),(float)(91.0 * DEG2RAD) };

  /* ppp start */
  pz_PPPfilterInfo->u_ns = 0;
  pz_PPPfilterInfo->f_age = 0.0f;
  if (any_Ptrs_Null(5, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs)
    || ppp_StartJudge(pz_satSigMeasCollect, pz_PPPfilterInfo))
  {
    LOGW(TAG_PPP, "ppp start failed!\n");
    pz_PPPfilterInfo->w_algStatus |= PPP_STATUS_MEMORY_ERROR;
    return 1;
  }
  /* sort measurement by good quality */
  PPP_sortMeasQuality(pz_satSigMeasCollect, pz_SsrLocBlk);

  if (TRUE == pz_PPPfilterInfo->q_QRcheckStatus)
  {
    PPP_markPseudoRangeUsingQRresult(pz_satSigMeasCollect);
  }
  else
  {
    PPP_priorResPseudoRangeQualityControl(pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
  }

  /* reset value of ppp float */
  PPP_EpochFilterInfoInit(pz_satSigMeasCollect, pz_PPPfilterInfo, pz_FilterObs, pd_siteCoor, pd_siteVel);

  /* time update */
  PPP_timeUpdateFilterPara(pd_siteCoor, pd_siteVel, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool);
  log_FilterInfo(pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, "filter before");

  double* pd_X = (double*)OS_MALLOC_FAST(pz_PPPfilterInfo->w_nmax * sizeof(double));
  if (NULL == pd_X)
  {
    pz_PPPfilterInfo->w_algStatus |= PPP_STATUS_MEMORY_ERROR;
    LOGW(TAG_PPP, "%s filter pd_X cant malloc %d bytes\n", __FUNCTION__, pz_PPPfilterInfo->w_nmax * sizeof(double));
    u_status = 1;
  }
  else
  {
    for (w_i = 0; w_i < (pz_PPPfilterInfo->w_nmax); ++w_i)
    {
      pd_X[w_i] = (pz_PPPfilterInfo->pd_X[w_i]);
      pz_PPPfilterInfo->pd_deltaX[w_i] = 0.0;
    }

    /* meas update */
    if (1 != (pz_PPPfilterInfo->u_goodSceneStatusFlag))
    {
      u_nD = PPP_zeroCombineDopplerMeasUpdate(pd_X, pf_eleSection, pz_SsrLocBlk, pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs);
      if (u_nD < 15)
      {
        pf_eleSection[1] = pf_eleSection[0];
        pf_eleSection[0] = (float)(pz_PPPfilterInfo->z_opt.d_elmin);
        PPP_zeroCombineDopplerMeasUpdate(pd_X, pf_eleSection, pz_SsrLocBlk, pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs);
      }
      pz_FilterObs->u_ppp_use_dr_num_float = u_nD;
    }
    u_nL = PPP_zeroCombinePhaseMeasUpdate(pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs);
    u_nP = PPP_zeroCombineCodeMeasUpdate(pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs);

    PPP_AtmosMeasUpdate(pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool, pz_FilterObs);

    /* post residual */
    PPP_zeroCombinePhaseMeasResidual(pz_PPPfilterInfo->pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool,NULL, pz_FilterObs, RES_POST);
    PPP_zeroCombineCodeMeasResidual(pz_PPPfilterInfo->pd_X, pz_satSigMeasCollect, pz_SsrLocBlk, pz_PPPfilterInfo, pz_EKFstateRepPool,NULL, pz_FilterObs, RES_POST);

    // log
    log_FilterInfo(pz_satSigMeasCollect, pz_PPPfilterInfo, pz_EKFstateRepPool, "filter after");
    log_FilterSatInfo(pz_satSigMeasCollect, pz_EKFstateRepPool, pz_FilterObs, pz_PPPfilterInfo, pz_SsrLocBlk);

    /* PPP float checking */
    if (PPP_floatSolCheck(u_nL, u_nP, pz_FilterObs, pz_EKFstateRepPool,pz_PPPfilterInfo))
    {
      pz_PPPfilterInfo->q_filterPosValid = TRUE;
    }
    if (pz_PPPfilterInfo->z_kfStatus != KF_RUN||!pz_PPPfilterInfo->q_filterPosValid)
    {
      u_status = 2;
    }
  }
  if (pz_PPPfilterInfo->q_filterPosValid)
  {
    pz_PPPfilterInfo->w_algStatus |= PPP_STATUS_SUCCESS;
  }
  if (pz_PPPfilterInfo->z_kfStatus == KF_RESET)
  {
    pz_PPPfilterInfo->w_algStatus |= PPP_STATUS_EKF_RESET;
  }

  OS_FREE(pd_X);
  return u_status;
}
