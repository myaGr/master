#include <math.h>
#include "dcp_solution.h"
#include "mw_alloc.h"
#include "gnss_common.h"
#include "sd_api.h"
#include "cmn_utils.h"
#include "seq_kalman_float.h"
#include "gnss_filter_type.h"
#include "sm_api.h"
#include "mw_log.h"

/* the input information used by TDCP or TDPR postion smooth algorithm */
typedef struct
{
  gnss_DcpType      u_posObainType;    /*position type of obtained method */
  GpsTime_t         z_obsTime;         /* observation time */
  double            pd_curEcefCoor[3]; /*the coordinate of site in ECEF*/
  double            pd_curEcefVel[3];  /*the velocity of site in ECEF*/
} dcp_posSmoothInterInfo_t;

/**
*@brief load the default option for the DCP algorithm
* @param[in]  pz_opt represent the algorithm optimization for the DCP
* @return
*/
void DCP_loadDefaultOption(gnss_DcpConfigOpt_t* pz_opt)
{
  uint8_t u_i = 0;
  pz_opt->z_usedSys = (ALGO_GPS_SYS | ALGO_BDS_SYS | ALGO_GAL_SYS | ALGO_QZS_SYS);
  pz_opt->u_enableDcpPosSmooth = 1;
  pz_opt->u_enableTDDRorTDPR = 1;
  pz_opt->u_enableCalCurrentSatPos = 1;
  pz_opt->u_enableFeedbackMeasToINS = 0;
  pz_opt->f_eleCutOff = 7.0f;
  pz_opt->z_optimalFreqType = C_GNSS_FREQ_TYPE_L1;
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pz_opt->pq_isSeparateBDS2And3[u_i] = FALSE;
  }
}

uint8_t DCP_sumofbit1(uint64_t t_marklist, uint8_t u_lsbwide)
{
  uint8_t u_i;
  uint8_t u_count = 0;

  for (u_i = 0; u_i < u_lsbwide; u_i++)
  {
    if ((t_marklist >> u_i) & 1)  u_count++;
  }

  return u_count;
}

/**
 * @brief initialize DCP model
 * @param[in]  pz_dcpConfig the configuration for the DCP model
 * @param[out] pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void DCP_init(const gnss_DcpConfigOpt_t* pz_dcpConfig, dcp_solInfo_t* pz_dcpSolInfo)
{
  if (NULL != pz_dcpConfig && NULL!= pz_dcpSolInfo)
  {
    pz_dcpSolInfo->z_dcpOpt = *pz_dcpConfig;
  }
  else if(NULL != pz_dcpSolInfo)
  {
    DCP_loadDefaultOption(&(pz_dcpSolInfo->z_dcpOpt));
  }

  if (NULL != pz_dcpSolInfo)
  {
    pz_dcpSolInfo->pz_baseDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
    if (1 == (pz_dcpSolInfo->z_dcpOpt.u_enableFeedbackMeasToINS))
    {
      pz_dcpSolInfo->pz_preDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
      pz_dcpSolInfo->pz_GnssFeedbackInsMeas = (gnss_FeedbackInsMeasBlock_t*)OS_MALLOC(sizeof(gnss_FeedbackInsMeasBlock_t));
    }
    else
    {
      pz_dcpSolInfo->pz_preDcpMeas = NULL;
      pz_dcpSolInfo->pz_GnssFeedbackInsMeas = NULL;
    }
    pz_dcpSolInfo->pz_dcpPosSeq = (gnss_DcpPosSeq_t*)OS_MALLOC(sizeof(gnss_DcpPosSeq_t));
    pz_dcpSolInfo->pz_dcpPosSeq->u_kfStatus = DCP_KF_INIT;

    memset(&(pz_dcpSolInfo->z_gpsTimePreEmpty), 0, sizeof(GpsTime_t));
  }
  return;
}
/**
*@brief deinitilize the DCP algorithm
* @param[in/out]  pz_dcpSolInfo the solution information for the DCP model
* @param[out]     void
* @return         void
*/
void DCP_deinit(dcp_solInfo_t* pz_dcpSolInfo)
{
  if (NULL != pz_dcpSolInfo)
  {
    if (NULL != (pz_dcpSolInfo->pz_preDcpMeas))
    {
      OS_FREE(pz_dcpSolInfo->pz_preDcpMeas);
    }

    if (NULL != (pz_dcpSolInfo->pz_baseDcpMeas))
    {
      OS_FREE(pz_dcpSolInfo->pz_baseDcpMeas);
    }

    if (NULL != (pz_dcpSolInfo->pz_dcpPosSeq))
    {
      OS_FREE(pz_dcpSolInfo->pz_dcpPosSeq);
    }

    if (NULL != (pz_dcpSolInfo->pz_GnssFeedbackInsMeas))
    {
      OS_FREE(pz_dcpSolInfo->pz_GnssFeedbackInsMeas);
    }
  }
  return;
}
/**
 * @brief update the TDCP measure block of previous epoch used by HPP or RTK or PPP-RTK or orient
 * @param[in] pz_baseDcpMeas
 * @param[in] pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void dcp_updateBaseMeasBlock(const gnss_TdcpMeasBlock_t* pz_baseDcpMeas, dcp_solInfo_t* pz_dcpSolInfo)
{
  if (NULL != pz_baseDcpMeas && NULL != pz_dcpSolInfo && NULL != (pz_dcpSolInfo->pz_baseDcpMeas))
  {
    *(pz_dcpSolInfo->pz_baseDcpMeas) = *pz_baseDcpMeas;
  }
  return;
}

/**
 * @brief update the initial smooth DCP position to use float to calculate
 * @param[in]  pd_curXyz the coordinate of site in ECEF of current epoch
 * @param[out] pz_dcpPosSeq the smooth information
 * @return void
 */
void dcp_updateInitSmoothPos(const double pd_curXyz[3], gnss_DcpPosSeq_t* pz_dcpPosSeq)
{
  uint8_t u_i = 0;
  uint8_t u_isInitPosValid = 1;
  uint8_t u_isUpdateInitPos = 0;
  double pd_filterPos[3] = { 0.0 };
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_filterPos[u_i] = pz_dcpPosSeq->pd_initPos[u_i] + pz_dcpPosSeq->pf_X[u_i];
    if (fabs(pz_dcpPosSeq->pf_X[u_i]) > 1000.0)
    {
      u_isUpdateInitPos = 1;
    }

    if (fabs(pz_dcpPosSeq->pd_initPos[u_i]) < 1.0e-3)
    {
      u_isInitPosValid = 0;
    }
  }

  if (0 == u_isInitPosValid)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_dcpPosSeq->pd_initPos[u_i] = pd_curXyz[u_i];
    }
  }

  if (u_isUpdateInitPos)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_dcpPosSeq->pf_X[u_i] = 0.0;
      pz_dcpPosSeq->pd_initPos[u_i] = pd_filterPos[u_i];
    }
  }
  return;
}
/**
 * @brief get the state transition matrix of PVA model
 * @param[in]    f_deltaT represent time difference between current and the filter time
 * @param[in]    u_reverse whether or not be reversed model
 * @param[out]   pf_F the state transition matrix of PVA model
 * @return  void
 */
void dcp_InitF(float f_deltaT, uint8_t u_reverse, float pf_F[])
{
  uint8_t u_i = 0;
  uint8_t u_recal = 0;
  float f_h1 = 0.0f;
  float f_h2 = 0.f;
  float f_h3 = 0.0f;
  float f_a = 0.2f;
  float f_expAT = 0.0f;
  if (u_reverse)
  {
    f_a = -f_a;
  }
  /* initialize A matrix */
  if (!u_reverse)
  {
    if (fabs(f_deltaT - 1.000) < 0.001)
    {
      f_h1 = DYN_H1_1s;
      f_h2 = DYN_H2_1s;
      f_h3 = DYN_H3_1s;
    }
    else if (fabs(f_deltaT - 0.200) < 0.001)
    {
      f_h1 = DYN_H1_200ms;
      f_h2 = DYN_H2_200ms;
      f_h3 = DYN_H3_200ms;
    }
    else if (fabs(f_deltaT - 0.100) < 0.001)
    {
      f_h1 = DYN_H1_100ms;
      f_h2 = DYN_H2_100ms;
      f_h3 = DYN_H3_100ms;
    }
    else
    {
      u_recal = 1;
    }
  }
  else //reverse
  {
    if (fabs(f_deltaT + 1.000) < 0.001)
    {
      f_h1 = DYN_H1_1s_revs;
      f_h2 = DYN_H2_1s_revs;
      f_h3 = DYN_H3_1s_revs;
    }
    else if (fabs(f_deltaT + 0.200) < 0.001)
    {
      f_h1 = DYN_H1_200ms_revs;
      f_h2 = DYN_H2_200ms_revs;
      f_h3 = DYN_H3_200ms_revs;
    }
    else if (fabs(f_deltaT + 0.100) < 0.001)
    {
      f_h1 = DYN_H1_100ms_revs;
      f_h2 = DYN_H2_100ms_revs;
      f_h3 = DYN_H3_100ms_revs;
    }
    else
    {
      u_recal = 1;
    }
  }

  if (u_recal)
  {
    f_expAT = expf(-f_a * f_deltaT);
    f_h1 = 1 / (f_a * f_a) * (-1 + f_a * f_deltaT + f_expAT);
    f_h2 = 1 / f_a * (1 - f_expAT);
    f_h3 = f_expAT;
  }
  for (u_i = 0; u_i < 3; u_i++)
  {
    pf_F[u_i + (u_i + 3) * PVA_NUM] = f_deltaT;
    pf_F[u_i + (u_i + 6) * PVA_NUM] = f_h1;
    pf_F[(u_i + 3) + (u_i + 6) * PVA_NUM] = f_h2;
    pf_F[(u_i + 6) + (u_i + 6) * PVA_NUM] = f_h3;
  }
  return;
}
/**
 * @brief fillup the noise of PVA model
 * @param[in]    f_deltaT represent time difference between current and the filter time
 * @param[in]    u_reverse whether or not be reversed model
 * @param[out]   pf_Q the noise of PVA model
 * @return  void
 */
void dcp_QmatSet(float f_deltaT, uint8_t u_reverse, float* pf_Q)
{
  float f_a = 0.2f;
  float f_T = f_deltaT;
  float f_exp_2aT = 0.0f;
  float f_exp_aT = 0.0f;
  float f_a2 = 0.0f;
  float f_a3 = 0.0f;
  float f_a4 = 0.0f;
  float f_T2 = 0.0f;
  float f_T3 = 0.0f;
  uint8_t u_recal = 0;

  f_deltaT = (float)fabs(f_T);
  if (u_reverse)
  {
    f_a = -f_a;
  }
  if (fabs(f_deltaT - 1.000) < 0.001)
  {
    pf_Q[0] = DYN_Q11_1s;
    pf_Q[1] = DYN_Q12_1s;
    pf_Q[2] = DYN_Q13_1s;
    pf_Q[3] = DYN_Q22_1s;
    pf_Q[4] = DYN_Q23_1s;
    pf_Q[5] = DYN_Q33_1s;
    if (u_reverse)
    {
      pf_Q[1] = -pf_Q[1];
      pf_Q[4] = -pf_Q[4];
    }
  }
  else if (fabs(f_deltaT - 0.200) < 0.001)
  {
    pf_Q[0] = DYN_Q11_200ms;
    pf_Q[1] = DYN_Q12_200ms;
    pf_Q[2] = DYN_Q13_200ms;
    pf_Q[3] = DYN_Q22_200ms;
    pf_Q[4] = DYN_Q23_200ms;
    pf_Q[5] = DYN_Q33_200ms;
    if (u_reverse)
    {
      pf_Q[1] = -pf_Q[1];
      pf_Q[4] = -pf_Q[4];
    }
  }
  else if (fabs(f_deltaT - 0.100) < 0.001)
  {
    pf_Q[0] = DYN_Q11_100ms;
    pf_Q[1] = DYN_Q12_100ms;
    pf_Q[2] = DYN_Q13_100ms;
    pf_Q[3] = DYN_Q22_100ms;
    pf_Q[4] = DYN_Q23_100ms;
    pf_Q[5] = DYN_Q33_100ms;
    if (u_reverse)
    {
      pf_Q[1] = -pf_Q[1];
      pf_Q[4] = -pf_Q[4];
    }
  }
  else
  {
    f_exp_2aT = expf(-2 * f_a * f_T);
    f_exp_aT = expf(-f_a * f_T);
    f_a2 = f_a * f_a;
    f_a3 = f_a2 * f_a;
    f_a4 = f_a3 * f_a;
    f_T2 = f_T * f_T;
    f_T3 = f_T2 * f_T;
    pf_Q[0] = 1 / f_a4 * (1 - f_exp_2aT + 2 * f_a * f_T + 2 * f_a3 * f_T3 / 3 - 2 * f_a2 * f_T2 - 4 * f_a * f_T * f_exp_aT);
    pf_Q[1] = 1 / f_a3 * (f_exp_2aT + 1 - 2 * f_exp_aT + 2 * f_a * f_T * f_exp_aT - 2 * f_a * f_T + f_a2 * f_T2);
    pf_Q[2] = 1 / f_a2 * (1 - f_exp_2aT - 2 * f_a * f_T * f_exp_aT);
    pf_Q[3] = 1 / f_a2 * (4 * f_exp_aT - 3 - f_exp_2aT + 2 * f_a * f_T);
    pf_Q[4] = 1 / f_a * (f_exp_2aT + 1 - 2 * f_exp_aT);
    pf_Q[5] = (1 - f_exp_2aT);
    if (u_reverse)
    {
      pf_Q[1] = -pf_Q[1];
      pf_Q[4] = -pf_Q[4];
    }
  }
  return;
}
/**
 * @brief get the noise of PVA model
 * @param[in]    f_deltaT represent time difference between current and the filter time
 * @param[in]    pf_accSigmaInit represent the enlarged factor for the PVA model
 * @param[in]    u_reverse whether or not be reversed model
 * @param[out]   pf_Q the noise of PVA model
 * @return  void
 */
void dcp_initQ(float f_deltaT, const float* pf_accSigmaInit, uint8_t u_reverse, float pf_Q[])
{
  uint8_t u_i = 0;
  float f_q11 = 0.0;
  float f_q12 = 0.0;
  float f_q13 = 0.0;
  float f_q22 = 0.0;
  float f_q23 = 0.0;
  float f_q33 = 0.0;
  float f_accSigma = 1.0;
  float pf_q[6] = { 0.0 };
  uint8_t u_trstCloCnt = 0;

  if (pf_accSigmaInit)
  {
    f_accSigma = *pf_accSigmaInit;
  }
  dcp_QmatSet(f_deltaT, 0, pf_q);
  f_q11 = pf_q[0] * f_accSigma;
  f_q12 = pf_q[1] * f_accSigma;
  f_q13 = pf_q[2] * f_accSigma;
  f_q22 = pf_q[3] * f_accSigma;
  f_q23 = pf_q[4] * f_accSigma;
  f_q33 = pf_q[5] * f_accSigma;
  for (u_i = 0; u_i < 3; u_i++)
  {
    pf_Q[u_i + u_i * PVA_NUM] = f_q11;
    pf_Q[(u_i + 3) + (u_i + 3) * PVA_NUM] = f_q22;
    pf_Q[(u_i)+(u_i + 3) * PVA_NUM] = f_q12;
    pf_Q[(u_i + 3) + (u_i)*PVA_NUM] = f_q12;

    pf_Q[(u_i + 6) + (u_i + 6) * PVA_NUM] = f_q33;
    pf_Q[(u_i)+(u_i + 6) * PVA_NUM] = f_q13;
    pf_Q[(u_i + 6) + (u_i)*PVA_NUM] = f_q13;
    pf_Q[(u_i + 3) + (u_i + 6) * PVA_NUM] = f_q23;
    pf_Q[(u_i + 6) + (u_i + 3) * PVA_NUM] = f_q23;
  }
  return;
}
/**
 * @brief update state transfer for PVA model
 * @param[in]    f_deltaT represent time difference between current and the filter time
 * @param[in]    pf_F the state transition matrix of PVA model
 * @param[out]   pz_dcpPosSeq the smooth information
 * @return void
 */
void dcp_PVAfilterPredict(float f_deltaT, const float* pf_F, gnss_DcpPosSeq_t* pz_dcpPosSeq)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_m = 0;
  float* pf_FP = NULL;
  float pf_FX[PVA_NUM] = { 0.0 };
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    pf_FX[u_i] = 0.0;
    for (u_j = 0; u_j < PVA_NUM; ++u_j)
    {
      pf_FX[u_i] += pf_F[u_j * PVA_NUM + u_i] * (pz_dcpPosSeq->pf_X[u_j]);
    }
  }

  for (u_i = 0; u_i < PVA_NUM; u_i++)
  {
    pz_dcpPosSeq->pf_X[u_i] = pf_FX[u_i];
  }

  //get the F*P
  pf_FP = (float*)OS_MALLOC(sizeof(float) * PVA_NUM * PVA_NUM);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    for (u_j = 0; u_j < PVA_NUM; ++u_j)
    {
      pf_FP[u_i * PVA_NUM + u_j] = 0.0;
      for (u_m = 0; u_m < PVA_NUM; ++u_m)
      {
        pf_FP[u_i * PVA_NUM + u_j] += pf_F[u_m * PVA_NUM + u_i] * pz_dcpPosSeq->pf_P[IUTM(u_j, u_m)];
      }
    }
  }
  //get the FP*Ft
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    for (u_j = 0; u_j < PVA_NUM; ++u_j)
    {
      pz_dcpPosSeq->pf_P[IUTM(u_i, u_j)] = 0.0;
      for (u_m = 0; u_m < PVA_NUM; ++u_m)
      {
        pz_dcpPosSeq->pf_P[IUTM(u_i, u_j)] += pf_FP[u_i * PVA_NUM + u_m] * pf_F[u_m * PVA_NUM + u_j];
      }
    }
  }
  OS_FREE(pf_FP);
  return;
}
/**
 * @brief update the state of smooth DCP position filter
 * @param[in]  pz_curTime the time of current observation
 * @param[in]  pf_deltaXyz the position that minus the initial value to enable to use float calculate
 * @param[in]  pd_velEcef the velocity in ECEF
 * @param[out] pz_dcpPosSeq the smooth information
 * @return void
 */
void dcp_updateSmoothPosFilterState(const GpsTime_t* pz_curTime, const float pf_deltaXyz[3], const double pd_velEcef[3], gnss_DcpPosSeq_t* pz_dcpPosSeq)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_rowIndex = 0;
  float f_accSigmaInit = 1.0;
  float* pf_F = NULL;/* state transition matrix of PVA model */
  float* pf_Q = NULL;/* the noise of PVA model */
  if (DCP_KF_INIT == (pz_dcpPosSeq->u_kfStatus) || DCP_KF_RESET == (pz_dcpPosSeq->u_kfStatus))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_dcpPosSeq->pf_X[u_i] = pf_deltaXyz[u_i];
      pz_dcpPosSeq->pf_X[u_i + 3] = (float)pd_velEcef[u_i];
      pz_dcpPosSeq->pf_X[u_i + 6] = 0.0;
    }
    for (u_i = 0; u_i < PVA_NUM; u_i++)
    {
      for (u_j = 0; u_j < PVA_NUM; ++u_j)
      {
        if (u_i == u_j)
        {
          pz_dcpPosSeq->pf_P[IUTM(u_i, u_i)] = SQR(50.0);
        }
        else
        {
          pz_dcpPosSeq->pf_P[IUTM(u_i, u_j)] = 0.0;
        }
      }
    }
    pz_dcpPosSeq->u_kfStatus = DCP_KF_RUN;
    pz_dcpPosSeq->z_filterTime = (*pz_curTime);
    return;
  }
  /* state transition of position/velocity/acceleration */
  pf_F = (float*)OS_MALLOC(sizeof(float) * PVA_NUM * PVA_NUM);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    u_rowIndex = u_i * PVA_NUM;
    for (u_j = 0; u_j < PVA_NUM; ++u_j)
    {
      if (u_i == u_j)
      {
        pf_F[u_rowIndex + u_j] = 1.0;
      }
      else
      {
        pf_F[u_rowIndex + u_j] = 0.0;
      }
    }
  }
  pf_Q = (float*)OS_MALLOC(sizeof(float) * PVA_NUM * PVA_NUM);
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    u_rowIndex = u_i * PVA_NUM;
    for (u_j = 0; u_j < PVA_NUM; ++u_j)
    {
      pf_Q[u_rowIndex + u_j] = 0.0;
    }
  }
  dcp_InitF(pz_dcpPosSeq->f_deltaT, 0, pf_F);
  dcp_initQ(pz_dcpPosSeq->f_deltaT, &f_accSigmaInit, 0, pf_Q);
  dcp_PVAfilterPredict(pz_dcpPosSeq->f_deltaT, pf_F, pz_dcpPosSeq);
  for (u_i = 0; u_i < 3; u_i++)
  {
    pz_dcpPosSeq->pf_P[IUTM(u_i, u_i)] += pf_Q[u_i + u_i * PVA_NUM];						 //q11;
    pz_dcpPosSeq->pf_P[IUTM((u_i + 3), (u_i + 3))] += pf_Q[(u_i + 3) + (u_i + 3) * PVA_NUM]; //q22;
    pz_dcpPosSeq->pf_P[IUTM(u_i, (u_i + 3))] += pf_Q[(u_i)+(u_i + 3) * PVA_NUM];			 //q12;
    pz_dcpPosSeq->pf_P[IUTM((u_i + 6), (u_i + 6))] += pf_Q[(u_i + 6) + (u_i + 6) * PVA_NUM]; //q33;
    pz_dcpPosSeq->pf_P[IUTM(u_i, (u_i + 6))] += pf_Q[(u_i)+(u_i + 6) * PVA_NUM];			 //q13;
    pz_dcpPosSeq->pf_P[IUTM((u_i + 3), (u_i + 6))] += pf_Q[(u_i + 3) + (u_i + 6) * PVA_NUM]; //q23;
  }
  OS_FREE(pf_F);
  OS_FREE(pf_Q);
  return;
}
/**
 * @brief get the position variance according to the fix quality of previous epoch and obtained position type by TDCP, TDDR or TDPR of current epoch
 * @param[in]  u_baseFixQuality fix quality of previous epoch
 * @param[in]  z_posType obtained position type by TDCP, TDDR or TDPR of current epoch
 * @return the position variance
 */
float dcp_getPosVar(uint8_t u_baseFixQuality, gnss_DcpType z_posType)
{
  float f_var = 0.1f * 0.1f;//the variance of fix solution
  switch (u_baseFixQuality)
  {
    case GNSS_FIX_FLAG_SPS:
      f_var *= 1.0e3f;
      break;
    case GNSS_FIX_FLAG_DGNSS:
      f_var *= 1.0e2f;
      break;
    case GNSS_FIX_FLAG_FLOATING:
      f_var *= 4.0f;
      break;
    case GNSS_FIX_FLAG_FIXED:
    default:
      break;
  }

  switch (z_posType)
  {
    case GNSS_DCP_POS_DOPPLER:
    f_var *= 4.0f;
      break;
    case GNSS_DCP_POS_PSEUDO:
    f_var *= 16.0f;
      break;
    default:
      break;
  }
  return f_var;
}
/**
 * @brief using the position of current epoch to update the kalman filter
 * @param[in]  pz_posSmoothInfo the input information used by TDCP or TDPR postion smooth algorithm
 * @param[out] pz_dcpPosSeq the smooth information
 * @return 0 represent failure and 1 represent successfull
 */
uint8_t dcp_posMeasUpdateFilter(const dcp_posSmoothInterInfo_t* pz_posSmoothInfo, gnss_DcpPosSeq_t* pz_dcpPosSeq)
{
  uint8_t u_posFilterStatus = 1;
  uint8_t u_i = 0;
  float pf_X[PVA_NUM] = { 0.0f };
  float pf_deltaX[PVA_NUM] = { 0.0f };
  float pf_P[PVA_NUM * (PVA_NUM + 1) / 2] = { 0.f };
  float f_omc = 0.0f;
  float f_var = 0.0f;
  uint16_t w_totalNum = PVA_NUM * (PVA_NUM + 1) / 2;
  SeqKalmanFloatVar_t z_seqKalmanFloatVar = { 0 };
  if (DCP_KF_RUN != pz_dcpPosSeq->u_kfStatus)
  {
    return 1;
  }
  if (0 == dcp_initSeqKalmanFloatVar(PVA_NUM, &z_seqKalmanFloatVar))
  {
    return 0;
  }
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    pf_X[u_i] = pz_dcpPosSeq->pf_X[u_i];
  }
  for (u_i = 0; u_i < w_totalNum; ++u_i)
  {
    pf_P[u_i] = pz_dcpPosSeq->pf_P[u_i];
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    hpp_seqCleanHfloat(&z_seqKalmanFloatVar);
    hpp_seqAddHfloat((uint16_t)u_i, 1.0, &z_seqKalmanFloatVar);
    f_omc =(float)(pz_posSmoothInfo->pd_curEcefCoor[u_i] - (pz_dcpPosSeq->pd_initPos[u_i] + pz_dcpPosSeq->pf_X[u_i]));
    f_var = dcp_getPosVar(pz_dcpPosSeq->u_fixFlag, pz_posSmoothInfo->u_posObainType);
    hpp_seqSetOmcFloat((float)f_omc, f_var, &z_seqKalmanFloatVar);
    hpp_seqPredictStepFloat(pf_deltaX, pf_P, &z_seqKalmanFloatVar);
    hpp_seqMeasUpdateFloat(pf_X, pf_deltaX, pf_P, &z_seqKalmanFloatVar, 0.0f);
  }
  for (u_i = 0; u_i < PVA_NUM; ++u_i)
  {
    if (pf_P[IUTM(u_i, u_i)] <= 0.0)
    {
      u_posFilterStatus = 0;
      break;
    }
  }
  if (u_posFilterStatus)
  {
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
      pz_dcpPosSeq->pf_X[u_i] = pf_X[u_i];
    }
    for (u_i = 0; u_i < w_totalNum; ++u_i)
    {
      pz_dcpPosSeq->pf_P[u_i] = pf_P[u_i];
    }
  }
  dcp_deinitSeqKalmanFloatVar(&z_seqKalmanFloatVar);
  return u_posFilterStatus;
}

/**
 * @brief print data log in DCP, for smooth filter X & P
 * @param pz_dcpPosSeq the smooth information
 */
void dcp_logPosSmooth(const gnss_DcpPosSeq_t* pz_dcpPosSeq)
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  DATA_LOG(DLOG_DCP, "%d\n", pz_dcpPosSeq->z_filterTime.q_towMsec);
  DATA_LOG(DLOG_DCP, " %f %f %f %f %f %f %f %f %f\n", pz_dcpPosSeq->pf_X[0],pz_dcpPosSeq->pf_X[1],pz_dcpPosSeq->pf_X[2],
           pz_dcpPosSeq->pf_X[3],pz_dcpPosSeq->pf_X[4],pz_dcpPosSeq->pf_X[5],
           pz_dcpPosSeq->pf_X[6],pz_dcpPosSeq->pf_X[7],pz_dcpPosSeq->pf_X[8]);

  for (int i = 0; i < PVA_NUM; ++i)
  {
    for (int j = 0; j <= i; ++j)
    {
      DATA_LOG(DLOG_DCP, " %f ", pz_dcpPosSeq->pf_P[IUTM(i, j)]);
    }
    DATA_LOG(DLOG_DCP, "\n");
  }
}

/**
 * @brief using the dynamic model to smooth DCP position
 * @param[in]  pz_posSmoothInfo the input information used by TDCP or TDPR postion smooth algorithm
 * @param[out] pz_dcpPosSeq the smooth information
 * @param[out] pd_smootedPos the smoothed DCP position
 * @return 0 represent failure and 1 represent successfull
 */
uint8_t dcp_smoothPosFilter(uint8_t u_fixFlag, const dcp_posSmoothInterInfo_t* pz_posSmoothInfo, gnss_DcpPosSeq_t* pz_dcpPosSeq, double pd_smootedPos[3])
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  float f_deltaTimeFilter = 0.0;
  float pf_deltaPos[3] = { 0.0 };
  float f_diff = 0.0;
  float f_distDiffPredict = 0.0;
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_smootedPos[u_i] = 0.0;
  }
  if (DCP_KF_INIT != (pz_dcpPosSeq->u_kfStatus))
  {
    f_deltaTimeFilter = (float)(tm_GpsTimeDiff(&(pz_posSmoothInfo->z_obsTime), &(pz_dcpPosSeq->z_filterTime)));
    pz_dcpPosSeq->f_deltaT = f_deltaTimeFilter;
  }
  if (fabs(pz_dcpPosSeq->f_deltaT) > 1.0)
  {
    pz_dcpPosSeq->u_kfStatus = DCP_KF_RESET;
  }
  dcp_updateInitSmoothPos(pz_posSmoothInfo->pd_curEcefCoor, pz_dcpPosSeq);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pf_deltaPos[u_i] = (float)(pz_posSmoothInfo->pd_curEcefCoor[u_i] - pz_dcpPosSeq->pd_initPos[u_i]);
  }
  /* time update */
  dcp_updateSmoothPosFilterState(&(pz_posSmoothInfo->z_obsTime), pf_deltaPos, pz_posSmoothInfo->pd_curEcefVel, pz_dcpPosSeq);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    f_diff = (float)(pz_posSmoothInfo->pd_curEcefCoor[u_i] - (pz_dcpPosSeq->pd_initPos[u_i] + pz_dcpPosSeq->pf_X[u_i]));
    f_distDiffPredict += (f_diff * f_diff);
  }
  LOGI(TAG_DCP, "Smooth d t upd %f\n", sqrtf(f_distDiffPredict));
  if (f_distDiffPredict >= 100.0 && fabsf(pz_dcpPosSeq->f_deltaT) <= 0.25 && u_fixFlag == (pz_dcpPosSeq->u_fixFlag))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_smootedPos[u_i] = pz_dcpPosSeq->pd_initPos[u_i] + pz_dcpPosSeq->pf_X[u_i];
    }
    return 1;
  }
  (pz_dcpPosSeq->u_fixFlag) = u_fixFlag;

  /* meas update */
  u_status = dcp_posMeasUpdateFilter(pz_posSmoothInfo, pz_dcpPosSeq);

  if (GNSS_DCP_POS_CARRIER != (pz_posSmoothInfo->u_posObainType))
  {
    if (GNSS_FIX_FLAG_FIXED == (pz_dcpPosSeq->u_fixFlag))
    {
      (pz_dcpPosSeq->u_fixFlag) = GNSS_FIX_FLAG_FLOATING;
    }
    else if (GNSS_FIX_FLAG_FLOATING == (pz_dcpPosSeq->u_fixFlag))
    {
      (pz_dcpPosSeq->u_fixFlag) = GNSS_FIX_FLAG_DGNSS;
    }
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    f_diff = (float)(pz_posSmoothInfo->pd_curEcefCoor[u_i] - (pz_dcpPosSeq->pd_initPos[u_i] + pz_dcpPosSeq->pf_X[u_i]));
    f_distDiffPredict += (f_diff * f_diff);
  }
  LOGI(TAG_DCP, "Smooth d m upd %f\n", sqrtf(f_distDiffPredict));

  /* copy result */
  if (u_status)
  {
    pz_dcpPosSeq->z_filterTime = pz_posSmoothInfo->z_obsTime;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pd_smootedPos[u_i] = pz_dcpPosSeq->pd_initPos[u_i] + pz_dcpPosSeq->pf_X[u_i];
    }
  }
  else
  {
    pz_dcpPosSeq->u_kfStatus = DCP_KF_RESET;
  }
  return u_status;
}
/**
 * @brief clean the position information for the DCP model
 * @param[out] pz_posSol the position information for the DCP model
 * @return void
 */
void dcp_cleanPos(gnss_PositionFix_t* pz_posSol)
{
  uint8_t u_i = 0;
  if (NULL != pz_posSol)
  {
    pz_posSol->u_fixSource = FIX_SOURCE_DCP; // DCP SOURCE solution
    pz_posSol->u_fixFlag = GNSS_FIX_FLAG_INVALID;
    pz_posSol->z_dops.f_gdop = 0.0;
    pz_posSol->z_dops.f_pdop = 0.0;
    pz_posSol->z_dops.f_hdop = 0.0;
    pz_posSol->z_dops.f_vdop = 0.0;
    pz_posSol->z_dops.f_tdop = 0.0;
    pz_posSol->z_rtk.f_age = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_posSol->f_velXyz[u_i] = 0.0;
      pz_posSol->f_velEnu[u_i] = 0.0;
      pz_posSol->f_posXyzUnc[u_i] = 0.0;
      pz_posSol->f_velEnuUnc[u_i] = 0.0;
      pz_posSol->d_xyz[u_i] = 0.0;
      pz_posSol->d_lla[u_i] = 0.0;
    }
  }
  return;
}
/**
 * @brief using the DCP postion information to fillup the struct of dcp_posSmoothInterInfo_t
 * @param[in]  pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]  pz_dcpInfo, the calculated information used by TDCP or TDPR algorithm
 * @param[out] pz_posSmoothInfo, the input information used by TDCP or TDPR postion smooth algorithm
 * @return void
 */
void dcp_fillupPosSmoothInterInfo(const gnss_TdcpMeasBlock_t* pz_curDcpMeas, const dcp_interRecordInfo_t* pz_dcpInfo,
  dcp_posSmoothInterInfo_t* pz_posSmoothInfo)
{
  uint8_t u_i = 0;
  pz_posSmoothInfo->u_posObainType = pz_dcpInfo->u_posObainType;
  pz_posSmoothInfo->z_obsTime = pz_curDcpMeas->z_obsTime;
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_posSmoothInfo->pd_curEcefCoor[u_i] = pz_curDcpMeas->z_posSol.d_xyz[u_i];
    pz_posSmoothInfo->pd_curEcefVel[u_i] = pz_curDcpMeas->z_posSol.f_velXyz[u_i];
  }
  return;
}

/**
 * @brief fill up the position and velocity to the TDCP measure block
 * @param[in] u_enableDcpPosSmooth whether enable position smooth function
 * @param[in] u_baseFixFlag the fix flag of previous epoch gained by HPP or RTK or PPP-RTK or orient
 * @param[in] p_dcpRecordInfo the calculated information used by TDCP or TDPR algorithm
 * @param[out] pz_dcpSolInfo the solution information for the DCP model
 * @return 1 represent do position smooth and 0 represent not do position smooth
 */
uint8_t dcp_fillPosVelToTDCPblock(uint8_t u_enableDcpPosSmooth, uint8_t u_baseFixFlag, const dcp_interRecordInfo_t* p_dcpRecordInfo, dcp_solInfo_t* pz_dcpSolInfo,  gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  uint8_t u_smoothPosStatus = 0;
  uint8_t u_i = 0;
  dcp_posSmoothInterInfo_t z_posSmoothInfo = { 0 };
  double pd_smoothedPos[3] = { 0.0 };
  double pd_velXyz[3] = { 0 };
  double pd_velEnu[3] = { 0 };
  gnss_DcpPosSeq_t* pz_dcpPosSeqInfo = pz_dcpSolInfo->pz_dcpPosSeq;
  /* Smooth coordinate */
  if (1 == u_enableDcpPosSmooth)
  {
    dcp_fillupPosSmoothInterInfo(pz_curDcpMeas, p_dcpRecordInfo, &z_posSmoothInfo);
    u_smoothPosStatus = dcp_smoothPosFilter(u_baseFixFlag, &z_posSmoothInfo, pz_dcpPosSeqInfo, pd_smoothedPos);
  }
  if (1 == u_smoothPosStatus)
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_curDcpMeas->z_posSol.d_xyz[u_i] = pd_smoothedPos[u_i];
      pz_curDcpMeas->z_posSol.f_velXyz[u_i] = pz_dcpPosSeqInfo->pf_X[u_i + 3];
    }
  }
  gnss_Ecef2Lla(pz_curDcpMeas->z_posSol.d_xyz, pz_curDcpMeas->z_posSol.d_lla);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_velXyz[u_i] = (double)(pz_curDcpMeas->z_posSol.f_velXyz[u_i]);
  }
  gnss_Ecef2Enu((const double*)pz_curDcpMeas->z_posSol.d_lla, (const double*)pd_velXyz, pd_velEnu);
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_curDcpMeas->z_posSol.f_velEnu[u_i] = (float)pd_velEnu[u_i];
  }
  return u_smoothPosStatus;
}

/**
 * @brief fill up the uncertainty of position and velocity to the TDCP measure block
 * @param[in] u_dcpCheckStatus whether going on DCP
 * @param[in] u_smoothPosStatus do position smooth sunccess flag
 * @param[in] pz_prePosSol the position information of previous
 * @param[in] pz_dcpPosSeqInfo the position smooth information
 * @param[out] pz_curPosSol the position information of current epoch
 * @return void
 */
void dcp_fillupPosVelUncertainty(uint8_t u_dcpCheckStatus, uint8_t u_smoothPosStatus, const gnss_PositionFix_t* pz_prePosSol, const gnss_DcpPosSeq_t* pz_dcpPosSeqInfo, gnss_PositionFix_t* pz_curPosSol)
{
  if ((pz_curPosSol->u_fixFlag != pz_prePosSol->u_fixFlag))
  {
    gnss_PositionUncertaintyMajic(pz_curPosSol);
  }
  else
  {
    /* DCP + SMOOTH */
    if (1 == u_dcpCheckStatus && 1 == u_smoothPosStatus)
    {
      pz_curPosSol->f_posXyzUnc[0] = sqrtf(pz_prePosSol->f_posXyzUnc[0] * pz_prePosSol->f_posXyzUnc[0] +
        pz_dcpPosSeqInfo->pf_P[IUTM(0, 0)] * pz_dcpPosSeqInfo->pf_P[IUTM(0, 0)]);
      pz_curPosSol->f_posXyzUnc[1] = sqrtf(pz_prePosSol->f_posXyzUnc[1] * pz_prePosSol->f_posXyzUnc[1] +
        pz_dcpPosSeqInfo->pf_P[IUTM(1, 1)] * pz_dcpPosSeqInfo->pf_P[IUTM(1, 1)]);
      pz_curPosSol->f_posXyzUnc[2] = sqrtf(pz_prePosSol->f_posXyzUnc[2] * pz_prePosSol->f_posXyzUnc[2] +
        pz_dcpPosSeqInfo->pf_P[IUTM(2, 2)] * pz_dcpPosSeqInfo->pf_P[IUTM(2, 2)]);
      double pd_Qecef[9] = { pz_curPosSol->f_posXyzUnc[0], 0, 0,
                            0, pz_curPosSol->f_posXyzUnc[1], 0,
                            0, 0, pz_curPosSol->f_posXyzUnc[2] };
      double pd_Qenu[9] = { 0.0 };
      gnss_CovEcef2Enu(pz_curPosSol->d_lla, pd_Qecef, pd_Qenu);
      pz_curPosSol->f_posLlaUnc[0] = (float)pd_Qenu[0];
      pz_curPosSol->f_posLlaUnc[1] = (float)pd_Qenu[4];
      pz_curPosSol->f_posLlaUnc[2] = (float)pd_Qenu[8];
      pz_curPosSol->f_velEnuUnc[0] = pz_prePosSol->f_velEnuUnc[0];
      pz_curPosSol->f_velEnuUnc[1] = pz_prePosSol->f_velEnuUnc[1];
      pz_curPosSol->f_velEnuUnc[2] = pz_prePosSol->f_velEnuUnc[2];
    }
  }
  return;
}
/**
 * @brief use the position smooth information to predict position and velocity
 * @param[in]  u_enableDcpPosSmooth whether enable position smooth function
 * @param[in]  pz_dcpPosSeqInfo the position smooth information
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @return 1 represent predict successful and 0 represent failure
 */
uint8_t dcp_useSmoothInfoPredictPosVel(uint8_t u_enableDcpPosSmooth,const gnss_DcpPosSeq_t* pz_dcpPosSeqInfo, gnss_TdcpMeasBlock_t* pz_curDcpMeas)
{
  uint8_t u_status = 0;
  uint8_t u_i = 0;
  float f_deltaTimeFilter = 0.0;
  double pd_velXyz[3] = { 0 };
  double pd_velEnu[3] = { 0 };
  if (1 == u_enableDcpPosSmooth)
  {
    f_deltaTimeFilter = (float)(tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_dcpPosSeqInfo->z_filterTime)));
    if (DCP_KF_RUN == (pz_dcpPosSeqInfo->u_kfStatus) && fabsf(f_deltaTimeFilter) <= 0.25)
    {
      u_status = 1;
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pz_curDcpMeas->z_posSol.d_xyz[u_i] = (pz_dcpPosSeqInfo->pf_X[u_i] + pz_dcpPosSeqInfo->pd_initPos[u_i])
          + pz_dcpPosSeqInfo->pf_X[u_i + 3] * (double)f_deltaTimeFilter
          + pz_dcpPosSeqInfo->pf_X[u_i + 6] * (double)f_deltaTimeFilter * f_deltaTimeFilter * 0.5;
        pz_curDcpMeas->z_posSol.f_velXyz[u_i] = pz_dcpPosSeqInfo->pf_X[u_i + 3] + pz_dcpPosSeqInfo->pf_X[u_i + 6] * f_deltaTimeFilter;
      }
      if (GNSS_FIX_FLAG_FIXED == (pz_curDcpMeas->z_posSol.u_fixFlag))
      {
        pz_curDcpMeas->z_posSol.u_fixFlag = GNSS_FIX_FLAG_FLOATING;
      }
      gnss_Ecef2Lla(pz_curDcpMeas->z_posSol.d_xyz, pz_curDcpMeas->z_posSol.d_lla);
      pd_velXyz[0] = (double)(pz_curDcpMeas->z_posSol.f_velXyz[0]);
      pd_velXyz[1] = (double)(pz_curDcpMeas->z_posSol.f_velXyz[1]);
      pd_velXyz[2] = (double)(pz_curDcpMeas->z_posSol.f_velXyz[2]);
      gnss_Ecef2Enu((const double*)pz_curDcpMeas->z_posSol.d_lla, (const double*)pd_velXyz, pd_velEnu);
      pz_curDcpMeas->z_posSol.f_velEnu[0] = (float)pd_velEnu[0];
      pz_curDcpMeas->z_posSol.f_velEnu[1] = (float)pd_velEnu[1];
      pz_curDcpMeas->z_posSol.f_velEnu[2] = (float)pd_velEnu[2];
    }
    else
    {
      dcp_cleanPos(&(pz_curDcpMeas->z_posSol));
    }
  }
  else
  {
    dcp_cleanPos(&(pz_curDcpMeas->z_posSol));
  }
  return u_status;
}
/**
 * @brief feedback the carrier phase measure to INS
 * @param[in]   f_priorSigma0 the prior sigma
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]   pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @param[out]  pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void dcp_feedbackCarrierPhaseToINS(float f_priorSigma0, const gnss_TdcpMeasBlock_t* pz_curDcpMeas, const dcp_interRecordInfo_t* pz_dcpInfo, dcp_solInfo_t* pz_dcpSolInfo)
{
  uint8_t u_i = 0;
  uint16_t w_iMeas = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_baseIndex = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_baseSatIndex = -1;
  int16_t w_preSatIndex = -1;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_baseMeas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const gnss_DcpConfigOpt_t* pz_dcpConfig = &(pz_dcpSolInfo->z_dcpOpt);
  const gnss_TdcpMeasBlock_t* pz_preDcpMeasBlock = pz_dcpSolInfo->pz_preDcpMeas;
  const gnss_TdcpMeasBlock_t* pz_baseDcpMeasBlock = pz_dcpSolInfo->pz_baseDcpMeas;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  double d_wave = 0.0;
  float f_ele = 0.0f;
  float f_res = 0.0f;
  float f_preRes = 0.0f;
  gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock = pz_dcpSolInfo->pz_GnssFeedbackInsMeas;
  if (NULL != pz_GnssFeedbackInsMeasBlock)
  {
    cmn_InitGnssMeasFeedbackInsBlock(pz_GnssFeedbackInsMeasBlock);
    pz_GnssFeedbackInsMeasBlock->u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_CP;
  }
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    if (NULL == pz_GnssFeedbackInsMeasBlock || (pz_GnssFeedbackInsMeasBlock->u_MeasCount) >= MAX_FEEDBACK_INS_MEAS_NUM || NULL == pz_preDcpMeasBlock)
    {
      continue;
    }
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (0x2 == ((pz_meas->u_LLI) & 0x2))
    {
      continue;
    }
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[pz_meas->u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_baseDcpMeasBlock, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_baseIndex))
    {
      continue;
    }
    pz_baseMeas = pz_baseDcpMeasBlock->pz_meas + u_baseIndex;
    if (!dcp_findMeasPreEpoch(pz_preDcpMeasBlock, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_preMeas = pz_preDcpMeasBlock->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_carrierPhase) < 1.0e-6 || fabs(pz_baseMeas->d_carrierPhase) < 1.0e-6 || fabs(pz_preMeas->d_carrierPhase) < 1.0e-6)
    {
      continue;
    }
    d_wave = wavelength((gnss_SignalType)pz_meas->u_signal);
    if (d_wave >= 1.0)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_baseSatIndex = dcp_findSatDataIndex(pz_baseMeas->u_constellation, pz_baseMeas->u_svid, pz_baseDcpMeasBlock);
    if (w_curSatIndex < 0 || w_baseSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].u_isCalculateDist)
    {
      continue;
    }
    w_preSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_preDcpMeasBlock);
    if (w_preSatIndex < 0 || FALSE == pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_res = (float)((pz_meas->d_carrierPhase - pz_baseMeas->d_carrierPhase) * d_wave);
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].pd_satPosClk[3]);
    f_res -= (pz_dcpInfo->pf_rcvClkDrift[pz_meas->u_constellation]);

    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].pd_site2SatUnit[u_i] * pz_dcpInfo->pf_deltaX[u_i]);
    }
    f_res -= (pz_dcpInfo->pf_deltaX[3 + (pz_meas->u_constellation)]);
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].u_constellation = (pz_meas->u_constellation);
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].u_svid = (pz_meas->u_svid);
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].u_cn0 = (uint8_t)(pz_meas->f_cn0 + 0.5);
    f_preRes = (float)((pz_meas->d_carrierPhase - pz_preMeas->d_carrierPhase) * d_wave);
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].f_epoch_diff_obs = f_preRes;
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].f_elevation = (float)(f_ele * DEG2RAD);
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].f_azimuth = (float)(pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].f_azimuth * DEG2RAD);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].f_unit_dir_vect[u_i] = (float)(pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i]);
      pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].d_sat_pos[u_i] = (pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[u_i]);
      pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].d_pre_sat_pos[u_i] = (pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].pd_satPosClk[u_i]);
    }
    if (fabs(f_res) <= f_priorSigma0)
    {
      pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].u_obs_valid = TRUE;
    }
    pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_GnssFeedbackInsMeasBlock->u_MeasCount].f_epoch_diff_res = f_res;
    ++(pz_GnssFeedbackInsMeasBlock->u_MeasCount);
  }
  return;
}

/**
 * @brief feedback the doppler measure to INS
 * @param[in]   f_priorSigma0 the prior sigma
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]   pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @param[out]  pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void dcp_feedbackDopplerToINS(float f_priorSigma0, const gnss_TdcpMeasBlock_t* pz_curDcpMeas, const dcp_interRecordInfo_t* pz_dcpInfo, dcp_solInfo_t* pz_dcpSolInfo)
{
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint8_t u_satValidNum = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint16_t w_iMeas = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_baseIndex = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  int16_t w_baseSatIndex = -1;
  const gnss_DcpConfigOpt_t* pz_dcpConfig = &(pz_dcpSolInfo->z_dcpOpt);
  const gnss_TdcpMeasBlock_t* pz_preDcpMeasBlock = pz_dcpSolInfo->pz_preDcpMeas;
  const gnss_TdcpMeasBlock_t* pz_baseDcpMeasBlock = pz_dcpSolInfo->pz_baseDcpMeas;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_baseMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_avgDoppler = 0.0;
  float f_ele = 0.0;
  float f_res = 0.0;
  float f_preRes = 0.0f;
  double d_deltaTime2Ins = 0.0;
  gnss_FeedbackInsMeasBlock_t* pz_feedbackInsMeasBlock = (pz_dcpSolInfo->pz_GnssFeedbackInsMeas);
  if (NULL != pz_feedbackInsMeasBlock)
  {
    cmn_InitGnssMeasFeedbackInsBlock(pz_feedbackInsMeasBlock);
    pz_feedbackInsMeasBlock->u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_DR;
  }
  if (NULL != pz_preDcpMeasBlock)
  {
    d_deltaTime2Ins = tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_preDcpMeasBlock->z_obsTime));
  }
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    if (NULL == pz_feedbackInsMeasBlock || (pz_feedbackInsMeasBlock->u_MeasCount) >= MAX_FEEDBACK_INS_MEAS_NUM || NULL == pz_preDcpMeasBlock)
    {
      continue;
    }
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[pz_meas->u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    u_constellation = (pz_meas->u_constellation);
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_baseDcpMeasBlock, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_baseIndex))
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeasBlock, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_baseMeas = pz_baseDcpMeasBlock->pz_meas + u_baseIndex;
    if (fabs(pz_meas->d_doppler) < 1.0e-6 && fabs(pz_baseMeas->d_doppler) < 1.0e-6)
    {
      continue;
    }
    if (fabs(pz_meas->d_doppler) > 0.0 && fabs(pz_baseMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)(pz_meas->d_doppler + pz_baseMeas->d_doppler) * 0.5f;
    }
    else if (fabs(pz_meas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_meas->d_doppler;
    }
    else if (fabs(pz_baseMeas->d_doppler) > 0.0)
    {
      f_avgDoppler = (float)pz_baseMeas->d_doppler;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_baseSatIndex = dcp_findSatDataIndex(pz_baseMeas->u_constellation, pz_baseMeas->u_svid, pz_baseDcpMeasBlock);
    if (w_curSatIndex < 0 || w_baseSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].u_isCalculateDist)
    {
      continue;
    }
    w_preSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_preDcpMeasBlock);
    if (w_preSatIndex < 0 || FALSE == pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_res = (float)(f_avgDoppler * (pz_dcpInfo->f_deltaTime));
    f_preRes = (float)(f_avgDoppler * d_deltaTime2Ins);
    
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].pd_satPosClk[3]);
    f_res -= (float)(pz_dcpInfo->pf_rcvClkDrift[u_constellation]);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].pd_site2SatUnit[u_i] * pz_dcpInfo->pf_deltaX[u_i]);
    }
    f_res -= (pz_dcpInfo->pf_deltaX[3 + u_constellation]);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_constellation = (pz_meas->u_constellation);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_svid = (pz_meas->u_svid);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_cn0 = (uint8_t)(pz_meas->f_cn0 + 0.5);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_epoch_diff_obs = f_preRes;
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_elevation = f_ele;
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_azimuth = (float)(pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].f_azimuth * DEG2RAD);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_unit_dir_vect[u_i] = (float)(pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i]);
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].d_sat_pos[u_i] = (pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[u_i]);
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].d_pre_sat_pos[u_i] = (pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].pd_satPosClk[u_i]);
    }
    if (fabs(f_res) <= f_priorSigma0)
    {
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_obs_valid = TRUE;
    }
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_epoch_diff_res = f_res;
    ++(pz_feedbackInsMeasBlock->u_MeasCount);
  }
  return;
}

/**
 * @brief feedback the pseudo-range measure to INS
 * @param[in]   f_priorSigma0 the prior sigma
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]   pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @param[out]  pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void dcp_feedbackPseudoRangeToINS(float f_priorSigma0, const gnss_TdcpMeasBlock_t* pz_curDcpMeas, const dcp_interRecordInfo_t* pz_dcpInfo, dcp_solInfo_t* pz_dcpSolInfo)
{
  uint8_t u_status = 1;
  uint8_t u_i = 0;
  uint8_t u_satValidNum = 0;
  gnss_ConstellationType u_constellation = C_GNSS_GPS;
  uint16_t w_iMeas = 0;
  uint8_t u_preIndex = 0;
  uint8_t u_baseIndex = 0;
  int16_t w_curSatIndex = -1;
  int16_t w_preSatIndex = -1;
  int16_t w_baseSatIndex = -1;
  const gnss_DcpConfigOpt_t* pz_dcpConfig = &(pz_dcpSolInfo->z_dcpOpt);
  const gnss_TdcpMeasBlock_t* pz_preDcpMeasBlock = pz_dcpSolInfo->pz_preDcpMeas;
  const gnss_TdcpMeasBlock_t* pz_baseDcpMeasBlock = pz_dcpSolInfo->pz_baseDcpMeas;
  const GnssMeas_t* pz_meas = NULL;
  const GnssMeas_t* pz_preMeas = NULL;
  const GnssMeas_t* pz_baseMeas = NULL;
  algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  float f_ele = 0.0f;
  float f_res = 0.0f;
  float f_preRes = 0.0f;
  gnss_FeedbackInsMeasBlock_t* pz_feedbackInsMeasBlock = (pz_dcpSolInfo->pz_GnssFeedbackInsMeas);
  if (NULL != pz_feedbackInsMeasBlock)
  {
    cmn_InitGnssMeasFeedbackInsBlock(pz_feedbackInsMeasBlock);
    pz_feedbackInsMeasBlock->u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_PR;
  }
  for (w_iMeas = 0; w_iMeas < (pz_curDcpMeas->w_measNum); ++w_iMeas)
  {
    if (NULL == pz_feedbackInsMeasBlock || (pz_feedbackInsMeasBlock->u_MeasCount) >= MAX_FEEDBACK_INS_MEAS_NUM || NULL == pz_preDcpMeasBlock)
    {
      continue;
    }
    pz_meas = pz_curDcpMeas->pz_meas + w_iMeas;
    if (DCP_NON_DETECT == pz_dcpInfo->pu_slipTag[w_iMeas])
    {
      continue;
    }
    //for GLONASS, there will be not do this routeline
    if (C_GNSS_GLO == (pz_meas->u_constellation) || (pz_dcpInfo->u_enterFilterNum[pz_meas->u_constellation]) < 2)
    {
      continue;
    }
    z_algoConstellation = gnss_satType2Algo(pz_meas->u_constellation);
    if (0 == ((pz_dcpConfig->z_usedSys) & z_algoConstellation))
    {
      continue;
    }
    u_constellation = (pz_meas->u_constellation);
    e_curFreqType = gnss_cvt_Sig2FreqType((gnss_SignalType)pz_meas->u_signal);
    if ((pz_dcpConfig->z_optimalFreqType) != e_curFreqType)
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_baseDcpMeasBlock, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_baseIndex))
    {
      continue;
    }
    if (!dcp_findMeasPreEpoch(pz_preDcpMeasBlock, pz_meas->u_constellation, pz_meas->u_svid, e_curFreqType, &u_preIndex))
    {
      continue;
    }
    pz_baseMeas = pz_baseDcpMeasBlock->pz_meas + u_baseIndex;
    pz_preMeas = pz_preDcpMeasBlock->pz_meas + u_preIndex;
    if (fabs(pz_meas->d_pseudoRange) < 1.0e-6 || fabs(pz_baseMeas->d_pseudoRange) < 1.0e-6 || fabs(pz_preMeas->d_pseudoRange) < 1.0e-6)
    {
      continue;
    }
    w_curSatIndex = dcp_findSatDataIndex(pz_meas->u_constellation, pz_meas->u_svid, pz_curDcpMeas);
    w_baseSatIndex = dcp_findSatDataIndex(pz_preMeas->u_constellation, pz_preMeas->u_svid, pz_baseDcpMeasBlock);
    if (w_curSatIndex < 0 || w_baseSatIndex < 0
      || FALSE == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_valid
      || FALSE == pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].u_valid
      || 0 == pz_curDcpMeas->pz_satInfo[w_curSatIndex].u_isCalculateDist
      || 0 == pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].u_isCalculateDist)
    {
      continue;
    }
    w_preSatIndex = dcp_findSatDataIndex(pz_baseMeas->u_constellation, pz_baseMeas->u_svid, pz_preDcpMeasBlock);
    if (w_preSatIndex < 0 || FALSE == pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].u_valid
      || 0 == pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].u_isCalculateDist)
    {
      continue;
    }
    f_ele = (pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].f_elevation);
    if (f_ele < (pz_dcpConfig->f_eleCutOff))
    {
      continue;
    }
    f_res = (float)(pz_meas->d_pseudoRange - pz_preMeas->d_pseudoRange);
    f_res -= (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].d_sat2SiteDist - pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].d_sat2SiteDist);
    //the satellite clock correction to be tested
    f_res += (float)(pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[3] - pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].pd_satPosClk[3]);
    f_res -= (pz_dcpInfo->pf_rcvClkDrift[u_constellation]);

    for (u_i = 0; u_i < 3; ++u_i)
    {
      f_res -= (float)(pz_baseDcpMeasBlock->pz_satInfo[w_baseSatIndex].pd_site2SatUnit[u_i] * pz_dcpInfo->pf_deltaX[u_i]);
    }
    f_res -= (pz_dcpInfo->pf_deltaX[3 + u_constellation]);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_constellation = (pz_meas->u_constellation);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_svid = (pz_meas->u_svid);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_cn0 = (uint8_t)(pz_meas->f_cn0 + 0.5);
    f_preRes = (float)(pz_meas->d_pseudoRange - pz_baseMeas->d_pseudoRange);
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_epoch_diff_obs = f_preRes;
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_elevation = f_ele;
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_azimuth = (float)(pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].f_azimuth * DEG2RAD);
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_unit_dir_vect[u_i] = (float)(pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].pd_site2SatUnit[u_i]);
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].d_sat_pos[u_i] = (pz_curDcpMeas->pz_satInfo[w_curSatIndex].pd_satPosClk[u_i]);
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].d_pre_sat_pos[u_i] = (pz_preDcpMeasBlock->pz_satInfo[w_preSatIndex].pd_satPosClk[u_i]);
    }
    if (fabsf(f_res) <= f_priorSigma0)
    {
      pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].u_obs_valid = TRUE;
    }
    pz_feedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[pz_feedbackInsMeasBlock->u_MeasCount].f_epoch_diff_res = f_res;
    ++(pz_feedbackInsMeasBlock->u_MeasCount);
  }
  return;
}
/**
 * @brief feedback the GNSS measure to INS
 * @param[in]   pz_curDcpMeas TDCP measure block of current epoch
 * @param[in]   pz_dcpInfo,   the calculated information used by TDCP or TDPR algorithm
 * @param[out]  pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void dcp_feedbackTimeDiffObsToINS(const gnss_TdcpMeasBlock_t* pz_curDcpMeas, const dcp_interRecordInfo_t* pz_dcpInfo, dcp_solInfo_t* pz_dcpSolInfo)
{
  uint8_t u_i = 0;
  double d_deltaTime2Ins = 0.0;
  gnss_TdcpMeasBlock_t* pz_preDcpMeas = pz_dcpSolInfo->pz_preDcpMeas;
  gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeas = NULL;
  d_deltaTime2Ins= tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_preDcpMeas->z_obsTime));
  if (GNSS_DCP_POS_CARRIER == (pz_dcpInfo->u_posObainType))
  {
    dcp_feedbackCarrierPhaseToINS(DCP_CARRIER_PRIOR_SIGMA0, pz_curDcpMeas, pz_dcpInfo, pz_dcpSolInfo);
  }
  else if (GNSS_DCP_POS_DOPPLER == (pz_dcpInfo->u_posObainType))
  {
    dcp_feedbackDopplerToINS(DCP_DOPPLE_PRIOR_SIGMA0, pz_curDcpMeas, pz_dcpInfo, pz_dcpSolInfo);
  }
  else if (GNSS_DCP_POS_PSEUDO == (pz_dcpInfo->u_posObainType))
  {
    dcp_feedbackPseudoRangeToINS(DCP_PSEUDO_PRIOR_SIGMA0, pz_curDcpMeas, pz_dcpInfo, pz_dcpSolInfo);
  }
  pz_GnssFeedbackInsMeas = pz_dcpSolInfo->pz_GnssFeedbackInsMeas;
  memcpy(&pz_GnssFeedbackInsMeas->z_GpsTime, &pz_curDcpMeas->z_obsTime, sizeof(GpsTime_t));

  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_GnssFeedbackInsMeas->d_CurLLA[u_i] = pz_curDcpMeas->z_posSol.d_lla[u_i];
    pz_GnssFeedbackInsMeas->d_PreLLA[u_i] = pz_preDcpMeas->z_posSol.d_lla[u_i];
    if (u_i <= 1)
    {
      (pz_GnssFeedbackInsMeas->d_CurLLA[u_i]) *= RAD2DEG;
      (pz_GnssFeedbackInsMeas->d_PreLLA[u_i]) *= RAD2DEG;
    }
  }

  pz_GnssFeedbackInsMeas->f_interval = (float)d_deltaTime2Ins;
  pz_GnssFeedbackInsMeas->f_QuasiGeoidHeight = (float)pz_curDcpMeas->z_posSol.d_quasiGeoidHeight;
  pz_GnssFeedbackInsMeas->u_CurFixFlag = pz_curDcpMeas->z_posSol.u_fixFlag;
  pz_GnssFeedbackInsMeas->u_PreFixFlag = pz_preDcpMeas->z_posSol.u_fixFlag;
  pz_GnssFeedbackInsMeas->u_FreqType = pz_dcpSolInfo->z_dcpOpt.z_optimalFreqType;
  memcpy(pz_preDcpMeas, pz_curDcpMeas, sizeof(gnss_TdcpMeasBlock_t));//update base dcp meas
  return;
}

/**
* @brief clean time if only have Glonass
* @param[in] pz_curDcpMeas TDCP measure block of current epoch
**/
uint8_t dcp_checkObsOfGLo(const GnssMeasBlock_t* pz_MeasBlock)
{
  uint16_t w_iMeas = 0;
  uint8_t u_status = 0;

  if (pz_MeasBlock->w_measCount > 0)
  {
    for (w_iMeas = 0; w_iMeas < (pz_MeasBlock->w_measCount); ++w_iMeas)
    {
      if (C_GNSS_GLO != pz_MeasBlock->z_meas[w_iMeas].u_constellation)
      {
        u_status = 1;
        break;
      }
    }
  }
  else
  {
    u_status = 1;
  }

  return u_status;
}

/**
 * @brief create gsv status
 * @param[int] pz_MeasBlock
 * @param[out] z_SvStatus is gsv status
 */
void dcp_CreateSvStatus(const GnssMeasBlock_t* pz_MeasBlock, gnss_PositionFix_SvStatus_t* z_SvStatus)
{
  uint16_t u_i = 0;
  uint8_t u_isChange = 0;
  uint8_t u_MeasCount = 0;
  gnss_PositionFixSV_t z_SVtemp = { 0 };

  for (; u_i < (pz_MeasBlock->w_measCount); ++u_i)
  {
    if (u_i >= MAX_GNSS_TRK_MEAS_NUMBER)
    {
      break;
    }

    const GnssMeas_t* pz_meas = &(pz_MeasBlock->z_meas[u_i]);

    if (pz_meas->u_svid == 0)
    {
      continue;
    }

    switch (pz_meas->u_constellation)
    {
    case C_GNSS_GPS:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GPS;
      z_SvStatus->z_SV[u_i].u_svid = pz_meas->u_svid;
      z_SvStatus->z_SV[u_i].u_signal = pz_meas->u_signal;
      z_SvStatus->t_SvTrackMask[C_GNSS_GPS] |= ((uint64_t)1 << (pz_meas->u_svid - 1));
      break;
    case C_GNSS_GLO:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GLO;
      z_SvStatus->z_SV[u_i].u_svid = pz_meas->u_svid;
      z_SvStatus->z_SV[u_i].u_signal = pz_meas->u_signal;
      z_SvStatus->t_SvTrackMask[C_GNSS_GLO] |= ((uint64_t)1 << (pz_meas->u_svid - 1));
      break;
    case C_GNSS_BDS2:
    case C_GNSS_BDS3:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_BDS3;
      z_SvStatus->z_SV[u_i].u_svid = pz_meas->u_svid;
      z_SvStatus->z_SV[u_i].u_signal = pz_meas->u_signal;
      z_SvStatus->t_SvTrackMask[C_GNSS_BDS3] |= ((uint64_t)1 << (pz_meas->u_svid - 1));
      break;
    case C_GNSS_GAL:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GAL;
      z_SvStatus->z_SV[u_i].u_svid = pz_meas->u_svid;
      z_SvStatus->z_SV[u_i].u_signal = pz_meas->u_signal;
      z_SvStatus->t_SvTrackMask[C_GNSS_GAL] |= ((uint64_t)1 << (pz_meas->u_svid - 1));
      break;
    case C_GNSS_QZS:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_QZS;
      z_SvStatus->z_SV[u_i].u_svid = pz_meas->u_svid;
      z_SvStatus->z_SV[u_i].u_signal = pz_meas->u_signal;
      z_SvStatus->t_SvTrackMask[C_GNSS_QZS] |= ((uint64_t)1 << (pz_meas->u_svid - 1));
      break;
    default:break;
    }
  }

  uint8_t u_gpsTrack = DCP_sumofbit1(z_SvStatus->t_SvTrackMask[C_GNSS_GPS], 64);
  uint8_t u_gloTrack = DCP_sumofbit1(z_SvStatus->t_SvTrackMask[C_GNSS_GLO], 64);
  uint8_t u_bdsTrack = DCP_sumofbit1(z_SvStatus->t_SvTrackMask[C_GNSS_BDS3], 64);
  uint8_t u_galTrack = DCP_sumofbit1(z_SvStatus->t_SvTrackMask[C_GNSS_GAL], 64);
  uint8_t u_qzssTrack = DCP_sumofbit1(z_SvStatus->t_SvTrackMask[C_GNSS_QZS], 64);

  z_SvStatus->u_MeasTrackCount = (uint8_t)u_i;
  z_SvStatus->u_SvTrackCount = u_gpsTrack + u_gloTrack + u_bdsTrack + u_galTrack + u_qzssTrack;

  if (u_i < 1)
  {
    return;
  }

  for (uint8_t u_j = 0; u_j < u_i - 1; u_j++)
  {
    u_isChange = 0;

    for (uint8_t u_k = 0; u_k < u_i - u_j - 1; u_k++)
    {
      int charge = 0;

      if (z_SvStatus->z_SV[u_k].u_constellation > z_SvStatus->z_SV[u_k + 1].u_constellation)
      {
        charge = 1;
      }
      else if (z_SvStatus->z_SV[u_k].u_constellation == z_SvStatus->z_SV[u_k + 1].u_constellation)
      {
        if (z_SvStatus->z_SV[u_k].u_svid > z_SvStatus->z_SV[u_k + 1].u_svid)
        {
          charge = 1;
        }
        else if (z_SvStatus->z_SV[u_k].u_svid == z_SvStatus->z_SV[u_k + 1].u_svid)
        {
          if (z_SvStatus->z_SV[u_k].u_signal > z_SvStatus->z_SV[u_k + 1].u_signal)
          {
            charge = 1;
          }
        }
      }

      if (charge)
      {
        memcpy(&z_SVtemp, &z_SvStatus->z_SV[u_k], sizeof(gnss_PositionFixSV_t));
        memcpy(&z_SvStatus->z_SV[u_k], &z_SvStatus->z_SV[u_k + 1], sizeof(gnss_PositionFixSV_t));
        memcpy(&z_SvStatus->z_SV[u_k + 1], &z_SVtemp, sizeof(gnss_PositionFixSV_t));
        u_isChange = 1;
      }
    }

    if (!u_isChange)
    {
      break;
    }
  }

  return;
}

/**
 * @brief create empty pos to report
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[in] pz_dcpSolInfo the solution information for the DCP model
 * @return None
 */
void dcp_CreateEmptyPos(const GnssMeasBlock_t* pz_MeasBlock, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  dcp_solInfo_t* pz_dcpSolInfo)
{
  if (NULL != pz_curDcpMeas && NULL != pz_dcpSolInfo)
  {
    memset(&pz_curDcpMeas->z_posSol, 0, sizeof(pz_curDcpMeas->z_posSol));

    pz_curDcpMeas->z_posSol.u_fixSource = FIX_SOURCE_DCP; // DCP SOURCE solution
    pz_curDcpMeas->z_posSol.u_fixFlag = GNSS_FIX_FLAG_INVALID;

    if (dcp_checkObsOfGLo(pz_MeasBlock) &&
      (fabs(tm_GpsTimeDiff(&pz_MeasBlock->z_Clock.z_gpsTime, &pz_dcpSolInfo->z_gpsTimePreEmpty)) > 1e-3))
    {
      pz_curDcpMeas->z_posSol.z_gpsTime = pz_MeasBlock->z_Clock.z_gpsTime;
    }

    if (NULL != pz_dcpSolInfo && pz_dcpSolInfo->pz_preDcpMeas->z_obsTime.w_week != 0 &&
      pz_curDcpMeas->z_posSol.z_gpsTime.q_towMsec != 0)
    {
      // use pre week
      pz_curDcpMeas->z_posSol.z_gpsTime.w_week = pz_dcpSolInfo->pz_preDcpMeas->z_obsTime.w_week;
    }

    // update pre time
    pz_dcpSolInfo->z_gpsTimePreEmpty = pz_MeasBlock->z_Clock.z_gpsTime;

    /* fill up sv status */
    dcp_CreateSvStatus(pz_MeasBlock, &pz_curDcpMeas->z_posSol.z_SvStatus);
  }

  return;
}

/**
 * @brief calculate the current position using the previous epoch and current observations by TDCP or TDPR
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[out] pz_dcpSolInfo the solution information for the DCP model
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_SolveRealtimePos(gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_solInfo_t* pz_dcpSolInfo)
{
  uint8_t u_status = 0;
  uint8_t u_dcpCheckStatus = 0;
  uint8_t u_smoothPosStatus = 0;
  uint8_t u_i = 0;
  gnss_TdcpMeasBlock_t* pz_baseDcpMeas = NULL;
  dcp_interRecordInfo_t* p_dcpRecordInfo = NULL;
  gnss_PositionFix_t* pz_curPosSol = &(pz_curDcpMeas->z_posSol);
  const gnss_PositionFix_t* pz_prePosSol = NULL;
  gnss_DcpPosSeq_t* pz_dcpPosSeqInfo = NULL;

  if (NULL != pz_dcpSolInfo)
  {
    if (NULL != (pz_dcpSolInfo->pz_GnssFeedbackInsMeas))
    {
      cmn_InitGnssMeasFeedbackInsBlock(pz_dcpSolInfo->pz_GnssFeedbackInsMeas);
    }
    pz_dcpSolInfo->z_dcpOpt.z_optimalFreqType = C_GNSS_FREQ_TYPE_L1;
    pz_baseDcpMeas = pz_dcpSolInfo->pz_baseDcpMeas;
    pz_dcpPosSeqInfo = pz_dcpSolInfo->pz_dcpPosSeq;
  }
  pz_curPosSol->z_gpsTime = pz_curDcpMeas->z_obsTime;

  if (NULL == pz_baseDcpMeas || NULL == pz_curDcpMeas)
  {
    return 0;
  }
  if (1 == (pz_dcpSolInfo->z_dcpOpt.u_enableDcpPosSmooth) && NULL == pz_dcpPosSeqInfo)
  {
    return 0;
  }
  p_dcpRecordInfo = (dcp_interRecordInfo_t*)OS_MALLOC(sizeof(dcp_interRecordInfo_t));
  if (NULL == p_dcpRecordInfo)
  {
    return 0;
  }
  pz_prePosSol = &(pz_baseDcpMeas->z_posSol);
  dcp_initDcpInterInfo(p_dcpRecordInfo);
  p_dcpRecordInfo->f_deltaTime = (float)(tm_GpsTimeDiff(&(pz_curDcpMeas->z_obsTime), &(pz_baseDcpMeas->z_obsTime)));
  /* Currect solution fullup */
  pz_curPosSol->u_fixSource = FIX_SOURCE_DCP;
  if (GNSS_FIX_FLAG_INVALID != (pz_baseDcpMeas->z_posSol.u_fixFlag))
  {
    dcp_fillupPosSolUsingPreEpoch(pz_prePosSol, pz_curPosSol);
    pz_curPosSol->z_gpsTime = pz_curDcpMeas->z_obsTime;
    tm_cvt_GpstToEpoch(&pz_curPosSol->z_gpsTime, &pz_curPosSol->z_epoch);
    if (GNSS_FIX_FLAG_SPS == pz_baseDcpMeas->z_posSol.u_fixFlag)
    {
      pz_curPosSol->z_rtk.f_age = 0.0; // clean age in spp mode
    }
    else if ((pz_prePosSol->z_rtk.f_age) > 0.0)
    {
      pz_curPosSol->z_rtk.f_age = (pz_prePosSol->z_rtk.f_age) + p_dcpRecordInfo->f_deltaTime;
    }
    // update stationID
    pz_curPosSol->z_rtk.q_StationID = pz_prePosSol->z_rtk.q_StationID;
  }
  else
  {
    /* update sv status */
    memcpy(&pz_curPosSol->z_SvStatus, &pz_prePosSol->z_SvStatus, sizeof(gnss_PositionFix_SvStatus_t));

    /* clean gsa status */
    pz_curPosSol->z_SvStatus.u_MeasInUseCount = 0;
    pz_curPosSol->z_SvStatus.u_SvInUseCount = 0;
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
      memset(&pz_curPosSol->z_SvStatus.t_SvInUseMask[u_i], 0, sizeof(uint64_t));
    }
  }

  /* Time difference CP/DR/PR */
  u_dcpCheckStatus = dcp_checkGoingTDroute(&(pz_prePosSol->z_SvStatus.u_SvInUseCount), &(pz_prePosSol->u_fixFlag), pz_prePosSol->d_xyz, &p_dcpRecordInfo->f_deltaTime);
  if (u_dcpCheckStatus)
  {
    LOGI(TAG_DCP, "Dcp start %d, base=%d, dt=%.1fs. flag=%d, age=%.1f\n", pz_curDcpMeas->z_obsTime.q_towMsec, pz_baseDcpMeas->z_obsTime.q_towMsec,
      p_dcpRecordInfo->f_deltaTime, pz_prePosSol->u_fixFlag, pz_prePosSol->z_rtk.f_age);
    u_status = dcp_gainTDpos(&(pz_dcpSolInfo->z_dcpOpt), pz_baseDcpMeas, pz_curDcpMeas, p_dcpRecordInfo);
    if (1 == u_status)
    {
      u_smoothPosStatus = dcp_fillPosVelToTDCPblock(pz_dcpSolInfo->z_dcpOpt.u_enableDcpPosSmooth, pz_baseDcpMeas->z_posSol.u_fixFlag, p_dcpRecordInfo, pz_dcpSolInfo, pz_curDcpMeas);
      if (1 == (pz_dcpSolInfo->z_dcpOpt.u_enableFeedbackMeasToINS) && NULL != (pz_dcpSolInfo->pz_GnssFeedbackInsMeas) && NULL != (pz_dcpSolInfo->pz_preDcpMeas) && NULL != (pz_dcpSolInfo->pz_baseDcpMeas))
      {
        dcp_feedbackTimeDiffObsToINS(pz_curDcpMeas, p_dcpRecordInfo, pz_dcpSolInfo);
      }
    }
    else
    {
      dcp_cleanPos(&(pz_curDcpMeas->z_posSol));
    }
  }
  else
  {
    if (1 == dcp_useSmoothInfoPredictPosVel(pz_dcpSolInfo->z_dcpOpt.u_enableDcpPosSmooth, pz_dcpPosSeqInfo, pz_curDcpMeas))
    {
      u_status = 1;
    }
  }
  if (u_status)
  {
    /* uncertainty */
    dcp_fillupPosVelUncertainty(u_dcpCheckStatus, u_smoothPosStatus, pz_prePosSol, pz_dcpPosSeqInfo, pz_curPosSol);
  }
  else
  {
    if (NULL != (pz_dcpSolInfo->pz_GnssFeedbackInsMeas))
    {
      pz_dcpSolInfo->pz_GnssFeedbackInsMeas->u_CurFixFlag = GNSS_FIX_FLAG_INVALID;
    }
  }
  OS_FREE(p_dcpRecordInfo);
  return u_status;
}