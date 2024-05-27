#include "gnss_filter_type.h"
#include "gnss_common.h"
#include "mw_alloc.h"
#include "cmn_sparse_matrix_multiply.h"
#include <math.h>
/**
 * @brief map the dcb type to filter type
 * @param[in]   z_f frequency type
 * @return gnss_FilterStateEnumType is the filter type
 */
gnss_FilterStateEnumType convertFreqDCB2FilterType(gnss_FreqType z_f)
{
    gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
    if (C_GNSS_FREQ_TYPE_L2 == z_f)
    {
        z_filterType = GNSS_FILTER_STATE_DCB12;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == z_f)
    {
        z_filterType = GNSS_FILTER_STATE_DCB15;
    }
    return z_filterType;
}
/**
 * @brief map the frequency type to filter type
 * @param[in]   z_f frequency type
 * @return gnss_FilterStateEnumType is the filter type
 */
gnss_FilterStateEnumType convertFreqAmb2FilterType(gnss_FreqType z_f)
{
    gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
    if (C_GNSS_FREQ_TYPE_L1 == z_f)
    {
        z_filterType = GNSS_FILTER_STATE_AMB_L1;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == z_f)
    {
        z_filterType = GNSS_FILTER_STATE_AMB_L2;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == z_f)
    {
        z_filterType = GNSS_FILTER_STATE_AMB_L5;
    }
    return z_filterType;
}
/**
 * @brief map the frequency type to amb type
 * @param[in]   z_f1 frequency type
 * @param[in]   z_f2 frequency type
 * @return gnss_fixedAmbType is the amb type
 */
gnss_fixedAmbType convertFreqType2AmbType(gnss_FreqType z_f1, gnss_FreqType z_f2)
{
  gnss_fixedAmbType z_ambType = GNSS_NONE_AMB_FIXED;
    if (C_GNSS_FREQ_TYPE_L1 == z_f1&& C_GNSS_FREQ_TYPE_L1 == z_f2)
    {
      z_ambType = GNSS_L1_AMB_FIXED;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == z_f1 && C_GNSS_FREQ_TYPE_L2 == z_f2)
    {
      z_ambType = GNSS_L2_AMB_FIXED;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == z_f1 && C_GNSS_FREQ_TYPE_L5 == z_f2)
    {
      z_ambType = GNSS_L3_AMB_FIXED;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == z_f1 && C_GNSS_FREQ_TYPE_L5 == z_f2)
    {
      z_ambType = GNSS_UW_AMB_FIXED;
    }
    else if (C_GNSS_FREQ_TYPE_L1 == z_f1 && C_GNSS_FREQ_TYPE_L2 == z_f2)
    {
      z_ambType = GNSS_W1_AMB_FIXED;
    }
    else if (C_GNSS_FREQ_TYPE_L1 == z_f1 && C_GNSS_FREQ_TYPE_L5 == z_f2)
    {
      z_ambType = GNSS_W2_AMB_FIXED;
    }
    return z_ambType;
}
/**
 * @brief map the filter type to frequency type
 * @param[in]   z_filterType filter type
 * @return gnss_FreqType is the frequency type
 */
gnss_FreqType convertFilterType2FreqAmb(gnss_FilterStateEnumType z_filterType)
{
  gnss_FreqType z_freqType = C_GNSS_FREQ_TYPE_MAX;
  if (GNSS_FILTER_STATE_AMB_L1 == z_filterType)
  {
    z_freqType = C_GNSS_FREQ_TYPE_L1;
  }
  else if (GNSS_FILTER_STATE_AMB_L2 == z_filterType)
  {
    z_freqType = C_GNSS_FREQ_TYPE_L2;
  }
  else if (GNSS_FILTER_STATE_AMB_L5 == z_filterType)
  {
    z_freqType = C_GNSS_FREQ_TYPE_L5;
  }
  return z_freqType;
}
/**
 * @brief the initilization information of parameters in the filter
 * @param[in]    gnss_filterInitInfo_t* pz_filterInitInfo
 * @return void
 */
void initFilterInitInfo(gnss_filterInitInfo_t* pz_filterInitInfo)
{
    if (pz_filterInitInfo)
    {
        memset(pz_filterInitInfo, 0, sizeof(gnss_filterInitInfo_t));
    }
    return;
}
/**
 * @brief initilize the ekf state represent
 * @param[out]   gnss_EKFstateRepresent_t* pz_EKFstateRep
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @return void
 */
void initEKFstateRepresent(gnss_EKFstateRepresent_t* pz_EKFstateRep, uint16_t w_nmax,
    double* pd_X, double* pd_Q)
{
    uint8_t u_i = 0;
    int16_t w_index = 0;
    int16_t w_i = 0;
    if (NULL != pz_EKFstateRep)
    {
        for (u_i = 0; u_i < GNSS_MAX_STATE_REP; ++u_i)
        {
            pz_EKFstateRep->w_id[u_i] = 0;
        }
        w_index = (pz_EKFstateRep->w_index);
        if (w_index >= 0)
        {
            pd_X[w_index] = 0.0;
            for (w_i = 0; w_i < w_nmax; w_i++)
            {
                pd_Q[IUTM(w_index, w_i)] = 0.0;
            }
        }
        pz_EKFstateRep->w_index = -1;
        tm_initGpsTime(&(pz_EKFstateRep->z_beginTime));
        tm_initGpsTime(&(pz_EKFstateRep->z_endTime));
    }
    return;
}
/**
 * @brief get ekf state pointer
 * @param[in]    w_id
 * @param[in]    pz_statusPool
 * @return pointer
 */
gnss_EKFstateRepresent_t* getEKFstatusModify(const uint16_t w_id[GNSS_MAX_STATE_REP], gnss_EKFstateRepresentPool_t* pz_statusPool)
{
    uint8_t q_i = 0;
    uint8_t q_type_index = 0;
    gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
    for (q_i = 0; q_i < GNSS_MAX_FILTER_STATE_NUM; ++q_i)
    {
        if (NULL == pz_statusPool->pz_satPool[q_i])
        {
            continue;
        }
        if (INVALID_INDEX == pz_statusPool->pz_satPool[q_i]->w_index)
        {
            continue;
        }
        for (q_type_index = 0; q_type_index < GNSS_MAX_STATE_REP; ++q_type_index)
        {
          if (pz_statusPool->pz_satPool[q_i]->w_id[q_type_index] != w_id[q_type_index])
          {
            break;
          }
        }
        if (q_type_index >= GNSS_MAX_STATE_REP)
        {
            pz_EKFstatus = pz_statusPool->pz_satPool[q_i];
            break;
        }
    }
    return pz_EKFstatus;
}
/**
 * @brief get ekf state pointer
 * @param[in]    w_id
 * @param[in]    pz_statusPool
 * @return pointer
 */
extern const gnss_EKFstateRepresent_t* getEKF_status(const uint16_t w_id[GNSS_MAX_STATE_REP], const gnss_EKFstateRepresentPool_t* pz_statusPool)
{
 uint8_t q_i = 0;
 uint8_t q_type_index = 0;
 const gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
 for (q_i = 0; q_i < GNSS_MAX_FILTER_STATE_NUM; ++q_i)
 {
   if (NULL == pz_statusPool->pz_satPool[q_i])
   {
     continue;
   }
  for (q_type_index = 0; q_type_index < GNSS_MAX_STATE_REP; ++q_type_index)
  {
  	if (pz_statusPool->pz_satPool[q_i]->w_id[q_type_index] != w_id[q_type_index])
  	{
  		break;
  	}
  }
  if (q_type_index >= GNSS_MAX_STATE_REP)
  {
    if(pz_statusPool->pz_satPool[q_i]->w_index == INVALID_INDEX){
      printf("a");
    }
    pz_EKFstatus = pz_statusPool->pz_satPool[q_i];
    break;
  }
 }
 return pz_EKFstatus;
}
/**
 * @brief remove the ekf state that long time not be updated
 * @param[in]    pz_statusPool
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @return       void
 */
void removeEKFstatus(gnss_EKFstateRepresentPool_t* pz_statusPool, uint16_t w_nmax,
    double* pd_X, double* pd_Q, BOOL* pq_paraValid)
{
    double d_timeDiff = 0.0;
    double d_maxDt = 0.01;
    uint16_t w_i = 0;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
    {
        if (NULL == pz_statusPool->pz_satPool[w_i])
        {
            continue;
        }
        d_timeDiff = tm_GpsTimeDiff(&(pz_statusPool->pz_satPool[w_i]->z_endTime),
            &(pz_statusPool->z_gpsTime));
        if (fabs(d_timeDiff) >= d_maxDt)
        {
            initEKFstateRepresent(pz_statusPool->pz_satPool[w_i], w_nmax, pd_X, pd_Q);
            pq_paraValid[w_i] = FALSE;
        }
    }
    return;
}
/**
 * @brief remove the target ekf state that long time not be updated
 * @param[in]    w_id represent the status of EKF
 * @param[in]    pz_statusPool
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @return       void
 */
void removeTargetEKFstatus(const uint16_t w_id[GNSS_MAX_STATE_REP], gnss_EKFstateRepresentPool_t* pz_statusPool,
    uint16_t w_nmax, double* pd_X, double* pd_Q, BOOL* pq_paraValid)
{
    uint16_t w_i = 0;
    uint8_t q_type_index = 0;
    gnss_EKFstateRepresent_t* pz_EKFstatus = getEKFstatusModify(w_id, pz_statusPool);
    if (NULL != pz_EKFstatus)
    {
        if (NULL != pq_paraValid)
        {
          w_i = (pz_EKFstatus->w_index);
          pq_paraValid[w_i] = FALSE;
        }
        initEKFstateRepresent(pz_EKFstatus, w_nmax, pd_X, pd_Q);
    }
    return;
}
/**
 * @brief remove the AMB ekf state in error case
 * @param[in]    pz_statusPool
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @return       void
 */
void removeAMBEKFstatus(gnss_EKFstateRepresentPool_t* pz_statusPool, uint16_t w_nmax,
  double* pd_X, double* pd_Q, BOOL* pq_paraValid)
{
  uint16_t w_i = 0;
  for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
  {
    if (NULL == pz_statusPool->pz_satPool[w_i])
    {
      continue;
    }
    if (pz_statusPool->pz_satPool[w_i]->w_id[2] >= GNSS_FILTER_STATE_AMB_L1 &&
      pz_statusPool->pz_satPool[w_i]->w_id[2] <= GNSS_FILTER_STATE_AMB_L5)
    {
      initEKFstateRepresent(pz_statusPool->pz_satPool[w_i], w_nmax, pd_X, pd_Q);
      pq_paraValid[w_i] = FALSE;
    }
  }
  return;
}
/**
 * @brief count number of the ekf state
 * @param[in]    pz_statusPool
 * @return       the number of the ekf state
 */
uint16_t countEKFstatus(const gnss_EKFstateRepresentPool_t* pz_statusPool)
{
    uint16_t w_stateNum = 0;
    uint16_t w_i = 0;
    gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
    {
        if (NULL == pz_statusPool->pz_satPool[w_i])
        {
            continue;
        }
        if ((pz_statusPool->pz_satPool[w_i]->w_index) < 0)
        {
            continue;
        }
        ++w_stateNum;
    }
    return w_stateNum;
}
/**
 * @brief get the indexs of all EKF state
 * @param[in]    pz_statusPool
 * @param[in]    pw_paraIndex the indexs of all EKF state
 * @return       the number of the ekf state
 */
uint16_t getAllStatusIndexEKF(const gnss_EKFstateRepresentPool_t* pz_statusPool, uint16_t pw_paraIndex[GNSS_MAX_FILTER_STATE_NUM])
{
    uint16_t w_stateNum = 0;
    uint16_t w_i = 0;
    gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
    {
        if (NULL == pz_statusPool->pz_satPool[w_i])
        {
            continue;
        }
        if ((pz_statusPool->pz_satPool[w_i]->w_index) < 0)
        {
            continue;
        }
        pw_paraIndex[w_stateNum] = w_i;
        ++w_stateNum;
    }
    return w_stateNum;
}
/**
 * @brief add the target ekf state
 * @param[in]    w_id represent the status of EKF
 * @param[in]    pz_obsTime represent the observation time of parameter
 * @param[in]    pz_statusPool
 * @return       gnss_EKFstateRepresent_t* the pointer of ekf state represent
 */
gnss_EKFstateRepresent_t* addTargetEKFstatus(const uint16_t w_id[GNSS_MAX_STATE_REP],
    const GpsTime_t* pz_obsTime, gnss_EKFstateRepresentPool_t* pz_statusPool)
{
    uint16_t w_i = 0;
    uint8_t q_type_index = 0;
    gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
    for (w_i = 0; w_i < GNSS_MAX_FILTER_STATE_NUM; ++w_i)
    {
        if (NULL == pz_statusPool->pz_satPool[w_i])
        {
            continue;
        }
        if (INVALID_INDEX == (pz_statusPool->pz_satPool[w_i]->w_index))
        {
            pz_EKFstatus = pz_statusPool->pz_satPool[w_i];
            break;
        }
    }

    if (NULL != pz_EKFstatus)
    {
        for (q_type_index = 0; q_type_index < GNSS_MAX_STATE_REP; ++q_type_index)
        {
            pz_EKFstatus->w_id[q_type_index] = w_id[q_type_index];
        }
        pz_EKFstatus->w_index = w_i;
        pz_EKFstatus->z_beginTime = *pz_obsTime;
        pz_EKFstatus->z_endTime = *pz_obsTime;
    }
    return pz_EKFstatus;
}
/**
 * @brief add the parameter to the EKF filter
 * @param[in]    w_id represent the status of EKF
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[in]    pz_initInfo represent the initilization infrmation of parameter
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @param[out]   pz_statusPool represet the state poll in the filter
 * @return       0 represent failure and other represent success
 */
uint8_t addState2EKFfilter(const uint16_t w_id[GNSS_MAX_STATE_REP], uint16_t w_nmax,
    const gnss_filterInitInfo_t* pz_initInfo, double* pd_X, double* pd_Q, BOOL* pq_paraValid, 
    gnss_EKFstateRepresentPool_t* pz_statusPool)
{
    uint8_t u_status = 1;
    int16_t w_index = -1;
    int16_t w_i = 0;
    uint8_t u_resetPara = (pz_initInfo->u_resetPara);
    double dt = 0.0;
    gnss_EKFstateRepresent_t* pz_ekfStateRep = NULL;
    pz_ekfStateRep = getEKFstatusModify(w_id, pz_statusPool);
    if (NULL == pz_ekfStateRep)
    {
        pz_ekfStateRep = addTargetEKFstatus(w_id, &(pz_initInfo->z_paraTime), pz_statusPool);
        u_resetPara = 1;
    }
    if (NULL != pz_ekfStateRep && (pz_ekfStateRep->w_index) >= 0)
    {
        w_index = (pz_ekfStateRep->w_index);
        pq_paraValid[w_index] = TRUE;
        if (u_resetPara)
        {
            pd_X[w_index] = pz_initInfo->d_initValue;
            for (w_i = 0; w_i < w_nmax; w_i++)
            {
                if (w_i == w_index)
                {
                    pd_Q[IUTM(w_index, w_i)] = (pz_initInfo->d_sigma0) * (pz_initInfo->d_sigma0);
                }
                else
                {
                    pd_Q[IUTM(w_index, w_i)] = 0.0;
                }
            }
            pz_ekfStateRep->z_beginTime = (pz_initInfo->z_paraTime);
            pz_ekfStateRep->z_endTime = (pz_initInfo->z_paraTime);
        }
        else
        {
            dt = fabs(tm_GpsTimeDiff(&(pz_initInfo->z_paraTime), &(pz_statusPool->z_gpsTime)));
            pd_Q[IUTM(w_index, w_index)] += (pz_initInfo->d_noise) * (pz_initInfo->d_noise) * dt;
            pz_ekfStateRep->z_endTime = (pz_initInfo->z_paraTime);
        }
    }
    else
    {
        u_status = 0;
    }
    return u_status;
}
/**
 * @brief get the state transition matrix of PVA model
 * @param[in]    d_deltaT represent time difference between current and the filter time
 * @param[in]    d_alphaIn represent the maneuver parameter of dynamic
 * @param[in]    u_isPPKreverse whether or not be reversed model
 * @param[out]   pd_phi[6] the state transition matrix of PVA model
 * pd_phi[0] denote A11,pd_phi[1] denote A12,pd_phi[2] denote A13
 * pd_phi[3] denote A22,pd_phi[4] denote A23,pd_phi[5] denote A33
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t calPhi(double d_deltaT, double d_alphaIn, uint8_t u_isPPKreverse, double pd_phi[6])
{
    uint8_t u_status = 0;
    uint8_t u_i = 0;
    uint8_t u_recal = 0;
    double* pd_h1 = pd_phi + 2;
    double* pd_h2 = pd_phi + 4;
    double* pd_h3 = pd_phi + 5;
    double d_alpha = d_alphaIn;
    double d_alphaT = 0.0;
    double d_eAlphaT = 0.0;
    for (u_i = 0; u_i < 6; u_i++)
    {
        pd_phi[u_i] = 0.0;
    }
    if (d_alphaIn <= 0.0)
    {
        return 0;
    }
    pd_phi[0] = pd_phi[3] = 1.0;
    pd_phi[1] = d_deltaT;
    if (1 == u_isPPKreverse)
    {
        d_alpha = -d_alpha;
    }
    /* initialize A matrix */
    if (0 == u_isPPKreverse)
    {
        if (fabs(d_deltaT - 1.000) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
        {
            *pd_h1 = DYN_H1_1s;
            *pd_h2 = DYN_H2_1s;
            *pd_h3 = DYN_H3_1s;
        }
        else if (fabs(d_deltaT - 0.200) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
        {
            *pd_h1 = DYN_H1_200ms;
            *pd_h2 = DYN_H2_200ms;
            *pd_h3 = DYN_H3_200ms;
        }
        else if (fabs(d_deltaT - 0.100) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
        {
            *pd_h1 = DYN_H1_100ms;
            *pd_h2 = DYN_H2_100ms;
            *pd_h3 = DYN_H3_100ms;
        }
        else
        {
            u_recal = 1;
        }
    }
    else //reverse
    {
        if (fabs(d_deltaT + 1.000) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
        {
            *pd_h1 = DYN_H1_1s_revs;
            *pd_h2 = DYN_H2_1s_revs;
            *pd_h3 = DYN_H3_1s_revs;
        }
        else if (fabs(d_deltaT + 0.200) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
        {
            *pd_h1 = DYN_H1_200ms_revs;
            *pd_h2 = DYN_H2_200ms_revs;
            *pd_h3 = DYN_H3_200ms_revs;
        }
        else if (fabs(d_deltaT + 0.100) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
        {
            *pd_h1 = DYN_H1_100ms_revs;
            *pd_h2 = DYN_H2_100ms_revs;
            *pd_h3 = DYN_H3_100ms_revs;
        }
        else
        {
            u_recal = 1;
        }
    }

    if (u_recal)
    {
        d_alphaT = d_alpha * d_deltaT;
        d_eAlphaT = exp(-d_alphaT);
        *pd_h1 = 1.0 / (d_alpha * d_alpha) * (-1.0 + d_alphaT + d_eAlphaT);
        *pd_h2 = 1.0 / d_alpha * (1.0 - d_eAlphaT);
        *pd_h3 = d_eAlphaT;
    }
    u_status = 1;
    return u_status;
}
/**
 * @brief get the noise of PVA model
 * @param[in]    d_deltaT represent time difference between current and the filter time
 * @param[in]    d_alphaIn represent the maneuver parameter of dynamic
 * @param[in]    d_accSigmaInit represent the enlarged factor for the PVA model
 * @param[in]    u_isPPKreverse whether or not be reversed model
 * @param[out]   pd_q[6] the noise of PVA model
 * pd_dynamicNoise[0] denote q11,pd_dynamicNoise[1] denote q12,pd_dynamicNoise[2] denote q13
 * pd_dynamicNoise[3] denote q22,pd_dynamicNoise[4] denote q23,pd_dynamicNoise[5] denote q33
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t calDynamicNoise(double d_deltaT, double d_alphaIn, double d_accSigmaInit,
    uint8_t u_isPPKreverse, double pd_dynamicNoise[6])
{
    uint8_t u_status = 0;
    uint8_t u_i = 0;
    double d_alpha = d_alphaIn;
    double d_alpha2 = 0.0;
    double d_alpha3 = 0.0;
    double d_alpha4 = 0.0;
    double d_alphaT = 0.0;
    double d_alphaT2 = 0.0;
    double d_alphaT3 = 0.0;
    double d_e_aT = 0.0;
    double d_e_2aT = 0.0;
    for (u_i = 0; u_i < 6; ++u_i)
    {
        pd_dynamicNoise[u_i] = 0.0;
    }
    if (d_alphaIn <= 0.0)
    {
        return 0;
    }
    if (1 == u_isPPKreverse)
    {
        d_alpha = -d_alphaIn;
    }
    if (fabs(d_deltaT - 1.000) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
    {
        pd_dynamicNoise[0] = DYN_Q11_1s;
        pd_dynamicNoise[1] = DYN_Q12_1s;
        pd_dynamicNoise[2] = DYN_Q13_1s;
        pd_dynamicNoise[3] = DYN_Q22_1s;
        pd_dynamicNoise[4] = DYN_Q23_1s;
        pd_dynamicNoise[5] = DYN_Q33_1s;
        if (1 == u_isPPKreverse)
        {
            pd_dynamicNoise[1] = -pd_dynamicNoise[1];
            pd_dynamicNoise[4] = -pd_dynamicNoise[4];
        }
    }
    else if (fabs(d_deltaT - 0.200) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
    {
        pd_dynamicNoise[0] = DYN_Q11_200ms;
        pd_dynamicNoise[1] = DYN_Q12_200ms;
        pd_dynamicNoise[2] = DYN_Q13_200ms;
        pd_dynamicNoise[3] = DYN_Q22_200ms;
        pd_dynamicNoise[4] = DYN_Q23_200ms;
        pd_dynamicNoise[5] = DYN_Q33_200ms;
        if (1 == u_isPPKreverse)
        {
            pd_dynamicNoise[1] = -pd_dynamicNoise[1];
            pd_dynamicNoise[4] = -pd_dynamicNoise[4];
        }
    }
    else if (fabs(d_deltaT - 0.100) < 0.001 && fabs(d_alphaIn - 0.2) < 0.001)
    {
        pd_dynamicNoise[0] = DYN_Q11_100ms;
        pd_dynamicNoise[1] = DYN_Q12_100ms;
        pd_dynamicNoise[2] = DYN_Q13_100ms;
        pd_dynamicNoise[3] = DYN_Q22_100ms;
        pd_dynamicNoise[4] = DYN_Q23_100ms;
        pd_dynamicNoise[5] = DYN_Q33_100ms;
        if (1 == u_isPPKreverse)
        {
            pd_dynamicNoise[1] = -pd_dynamicNoise[1];
            pd_dynamicNoise[4] = -pd_dynamicNoise[4];
        }
    }
    else
    {
        d_alpha2 = d_alpha * d_alpha;
        d_alpha3 = d_alpha2 * d_alpha;
        d_alpha4 = d_alpha2 * d_alpha2;
        d_alphaT = d_alpha * d_deltaT;
        d_alphaT2 = d_alphaT * d_alphaT;
        d_alphaT3 = d_alphaT2 * d_alphaT;
        d_e_aT = exp(-d_alphaT);
        d_e_2aT = exp(-2.0 * d_alphaT);
        pd_dynamicNoise[0] = 1.0 / d_alpha4 * (1.0 - d_e_2aT + 2.0 * d_alphaT + 2.0 * d_alphaT3 / 3.0 - 2.0 * d_alphaT2 - 4.0 * d_alphaT * d_e_aT);
        pd_dynamicNoise[1] = 1.0 / d_alpha3 * (d_e_2aT + 1.0 - 2.0 * d_e_aT + 2.0 * d_alphaT * d_e_aT - 2.0 * d_alphaT + d_alphaT2);
        pd_dynamicNoise[2] = 1.0 / d_alpha2 * (1.0 - d_e_2aT - 2.0 * d_alphaT * d_e_aT);
        pd_dynamicNoise[3] = 1.0 / d_alpha2 * (4.0 * d_e_aT - 3.0 - d_e_2aT + 2.0 * d_alphaT);
        pd_dynamicNoise[4] = 1.0 / d_alpha * (d_e_2aT + 1.0 - 2.0 * d_e_aT);
        pd_dynamicNoise[5] = (1.0 - d_e_2aT);
    }
    for (u_i = 0; u_i < 6; u_i++)
    {
        pd_dynamicNoise[u_i] *= d_accSigmaInit;
    }
    u_status = 1;
    return u_status;
}
/**
 * @brief summery the number of non-zero elements for the state transfer matrix A
 * @param[in]    w_stateNum represent the number of state in the filter
 * @return       the number of non-zero elements for the state transfer matrix A
 */
uint32_t sumTransAsparseEleNum(uint16_t w_stateNum)
{
    uint32_t q_eleNum = 0;
    uint8_t u_i = 0;
    //for site coordinate
    for (u_i = 0; u_i < 3; ++u_i)
    {
        q_eleNum += 3;
    }
    //for site velocity
    for (u_i = 3; u_i < 6; ++u_i)
    {
        q_eleNum += 2;
    }
    //for site acceleration
    for (u_i = 6; u_i < 9; ++u_i)
    {
        q_eleNum += 1;
    }
    q_eleNum += (w_stateNum - PVA_NUM);
    return q_eleNum;
}
/**
 * @brief get the non-zero elements for the state transfer matrix A
 * @param[in]    pd_phi[6] the state transition matrix of PVA model
 * @param[out]   pz_A the non-zero elements for the state transfer matrix A
 * @return       void
 */
void getPvaStateTransSparseMatrixA(const double pd_phi[6], gnss_sparseDoubleMatrix* pz_A)
{
  uint16_t w_i = 0;
  uint32_t q_eleIndex = 0;
  gnss_tripleDoubleEle* pz_tripleEle = (pz_A->pz_tripleEleSet);
  //for site coordinate
  for (w_i = 0; w_i < 3; w_i++)
  {
    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[0];
    ++q_eleIndex;

    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = 3 + w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[1];
    ++q_eleIndex;

    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = 6 + w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[2];
    ++q_eleIndex;
  }
  //for site velocity
  for (w_i = 3; w_i < 6; w_i++)
  {
    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[3];
    ++q_eleIndex;

    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = 3 + w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[4];
    ++q_eleIndex;
  }
  //for site acceleration
  for (w_i = 6; w_i < 9; w_i++)
  {
    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[5];
    ++q_eleIndex;
  }
  gnss_sortTripleDoubleEleByRowIndex(pz_tripleEle, q_eleIndex);
  return;
}
/**
 * @brief get the non-zero elements for the transposition of state transfer A
 * @param[in]    pd_phi[6] the state transition matrix of PVA model
 * @param[out]   pz_At the non-zero elements for the transposition of state transfer A
 * @return       void
 */
void getPvaStateTransSparseMatrixAt(const double pd_phi[6], gnss_sparseDoubleMatrix* pz_At)
{
  uint16_t w_i = 0;
  uint32_t q_eleIndex = 0;
  gnss_tripleDoubleEle* pz_tripleEle = (pz_At->pz_tripleEleSet);
  for (w_i = 0; w_i < 3; w_i++)
  {
    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[0];
    ++q_eleIndex;
  }

  for (w_i = 3; w_i < 6; w_i++)
  {
    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i - 3;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[1];
    ++q_eleIndex;

    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[3];
    ++q_eleIndex;
  }
  for (w_i = 6; w_i < 9; w_i++)
  {
    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i - 6;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[2];
    ++q_eleIndex;

    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i - 3;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[4];
    ++q_eleIndex;

    pz_tripleEle[q_eleIndex].w_rowIndex = w_i;
    pz_tripleEle[q_eleIndex].w_columIndex = w_i;
    pz_tripleEle[q_eleIndex].d_value = pd_phi[5];
    ++q_eleIndex;
  }
  gnss_sortTripleDoubleEleByColumnIndex(pz_tripleEle, q_eleIndex);
  return;
}
/**
 * @brief        get the PVA index in the filter
 * @param[in]    pz_statusPool represent the status pool in the filter
 * @param[out]   w_pvaIndex represent the PVA index in the filter
 * @return       void
 */
void getPVAindex(const gnss_EKFstateRepresentPool_t* pz_statusPool, int16_t w_pvaIndex[PVA_NUM])
{
    uint8_t u_i = 0;
    uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
    const gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
        w_pvaIndex[u_i] = -1;
    }
    for (u_i = 0; u_i < 3; ++u_i)
    {
        w_id[0] = 0;
        w_id[1] = u_i;
        w_id[2] = GNSS_FILTER_STATE_POS;
        /*position state*/
        pz_EKFstatus = getEKF_status(w_id, pz_statusPool);
        if (NULL != pz_EKFstatus)
        {
            w_pvaIndex[u_i] = (pz_EKFstatus->w_index);
        }
        /*velocity state*/
        w_id[2] = GNSS_FILTER_STATE_VEL;
        pz_EKFstatus = getEKF_status(w_id, pz_statusPool);
        if (NULL != pz_EKFstatus)
        {
            w_pvaIndex[3 + u_i] = (pz_EKFstatus->w_index);
        }
        /*acceleration state*/
        w_id[2] = GNSS_FILTER_STATE_ACC;
        pz_EKFstatus = getEKF_status(w_id, pz_statusPool);
        if (NULL != pz_EKFstatus)
        {
            w_pvaIndex[6 + u_i] = (pz_EKFstatus->w_index);
        }
    }
    return;
}
/**
 * @brief Add the PVA noise to the Q matrix
 * @param[in]    d_deltaTime represent time difference between current and the filter time
 * @param[in]    pw_pvaIndex represent the PVA index in the filter
 * @param[in]    pd_dynamicNoise[6] the noise of PVA model
 * @param[out]   pd_filterQ represent the covariance of parameter in the filter
 * @return       void
 */
void addPvaNoise(double d_deltaTime, const int16_t pw_pvaIndex[PVA_NUM],
    double pd_dynamicNoise[6], double* pd_filterQ)
{
    const int16_t* pw_coorIndex = pw_pvaIndex;
    const int16_t* pw_velIndex = pw_pvaIndex + 3;
    const int16_t* pw_accIndex = pw_pvaIndex + 6;
    uint8_t u_i = 0;
    for (u_i = 0; u_i < 3; u_i++)
    {
        pd_filterQ[IUTM(pw_coorIndex[u_i], pw_coorIndex[u_i])] += pd_dynamicNoise[0];
        pd_filterQ[IUTM(pw_coorIndex[u_i], pw_velIndex[u_i])] += pd_dynamicNoise[1];
        pd_filterQ[IUTM(pw_coorIndex[u_i], pw_accIndex[u_i])] += pd_dynamicNoise[2];

        pd_filterQ[IUTM(pw_velIndex[u_i], pw_velIndex[u_i])] += pd_dynamicNoise[3];
        pd_filterQ[IUTM(pw_velIndex[u_i], pw_accIndex[u_i])] += pd_dynamicNoise[4];

        pd_filterQ[IUTM(pw_accIndex[u_i], pw_accIndex[u_i])] += pd_dynamicNoise[5];
    }
    return;
}
/**
 * @brief using dynamic model to predict PVA parameter in the filter
 * @param[in]    pw_pvaIndex represent the PVA index in the filter
 * @param[in]    pd_phi[6] the state transition matrix of PVA model
 * @param[out]   pd_filterX represent the value of parameter in the filter
 * @return       void
 */
void dynPredictPVAparameter(const int16_t pw_pvaIndex[PVA_NUM], const double pd_phi[6], double* pd_filterX)
{
    const int16_t* pw_coorIndex = pw_pvaIndex;
    const int16_t* pw_velIndex = pw_pvaIndex + 3;
    const int16_t* pw_accIndex = pw_pvaIndex + 6;
    double d_preCoor = 0.0;
    double d_preVel = 0.0;
    double d_preAcc = 0.0;
    uint8_t u_i = 0;
    for (u_i = 0; u_i < 3; u_i++)
    {
        d_preCoor = pd_filterX[pw_coorIndex[u_i]];
        d_preVel = pd_filterX[pw_velIndex[u_i]];
        d_preAcc = pd_filterX[pw_accIndex[u_i]];
        
        pd_filterX[pw_coorIndex[u_i]] = d_preCoor + pd_phi[1] * d_preVel + pd_phi[2] * d_preAcc;
        pd_filterX[pw_velIndex[u_i]] = d_preVel + pd_phi[4] * d_preAcc;
        pd_filterX[pw_accIndex[u_i]] = pd_phi[5] * d_preAcc;
    }
    return;
}
/**
 * @brief update the time of PVA parameter in the filter
 * @param[in]    z_gpsTime represent the current time
 * @param[out]   pz_statusPool represent the status pool in the filter
 * @return       void
 */
void updatePVAstateTime(GpsTime_t z_gpsTime, gnss_EKFstateRepresentPool_t* pz_statusPool)
{
    uint8_t u_i = 0;
    uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
    gnss_EKFstateRepresent_t* pz_EKFstatus = NULL;
    for (u_i = 0; u_i < 3; ++u_i)
    {
        w_id[0] = 0;
        w_id[1] = u_i;
        w_id[2] = GNSS_FILTER_STATE_POS;
        /*position state*/
        pz_EKFstatus = getEKFstatusModify(w_id, pz_statusPool);
        if (NULL != pz_EKFstatus)
        {
            pz_EKFstatus->z_endTime = z_gpsTime;
        }
        /*velocity state*/
        w_id[2] = GNSS_FILTER_STATE_VEL;
        pz_EKFstatus = getEKFstatusModify(w_id, pz_statusPool);
        if (NULL != pz_EKFstatus)
        {
            pz_EKFstatus->z_endTime = z_gpsTime;
        }
        /*acceleration state*/
        w_id[2] = GNSS_FILTER_STATE_ACC;
        pz_EKFstatus = getEKFstatusModify(w_id, pz_statusPool);
        if (NULL != pz_EKFstatus)
        {
            pz_EKFstatus->z_endTime = z_gpsTime;
        }
    }
    return;
}
/**
 * @brief get all parameter index that excluding PVA
 * @param[in]    w_stateNum represent the number of state in the filter
 * @param[in]    pw_allParaIndex all parameter index
 * @param[in]    w_pvaIndex the index in filter for the PVA parameter
 * @param[out]   pw_paraIndexExcludingPVA the index in filter for all parameter index that excluding PVA
 * @return       the number of parameter that excluding PVA
 */
uint16_t getAllParaIndexExcludingPVA(uint16_t w_stateNum, const uint16_t* pw_allParaIndex, const int16_t w_pvaIndex[PVA_NUM], uint16_t* pw_paraIndexExcludingPVA)
{
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  uint16_t w_num = 0;
  uint8_t u_isFind = 0;
  for (w_i = 0; w_i < w_stateNum; w_i++)
  {
    u_isFind = 0;
    for (w_j = 0; w_j < PVA_NUM; w_j++)
    {
      if (pw_allParaIndex[w_i] == w_pvaIndex[w_j])
      {
        u_isFind = 1;
        break;
      }
    }
    if (1 == u_isFind)
    {
      continue;
    }
    pw_paraIndexExcludingPVA[w_num] = pw_allParaIndex[w_i];
    ++w_num;
  }
  return w_num;
}
/**
 * @brief state transfer for PVA model
 * @param[in]    pd_filterX represent the value of parameter in the filter
 * @param[in]    pd_filterQ represent the covariance of parameter in the filter
 * @param[in]    pz_statusPool represent the status pool in the filter
 * @param[in]    z_gpsTime represent current time
 * @param[in]    d_alphaIn represent the maneuver parameter of dynamic
 * @param[in]    d_accSigmaInit represent the enlarged factor for the PVA model
 * @param[in]    u_isPPKreverse whether or not be reversed model
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t dynamicStateTransfer(double* pd_filterX, double* pd_filterQ, gnss_EKFstateRepresentPool_t* pz_statusPool,
  GpsTime_t z_gpsTime, double d_alphaIn, double d_accSigmaInit, uint8_t u_isPPKreverse)
{
  //0~2 index for position,3~5 index for velocity, 6~8 index for acceleration
  int16_t pw_pvaIndex[PVA_NUM] = { -1 };
  uint16_t w_stateNum = 0;
  uint16_t w_stateNumNonePva = 0;
  uint16_t w_i = 0;
  uint16_t w_j = 0;
  uint16_t w_rowIndex = 0;
  uint32_t q_transMeleNum = 0;
  double pd_phi[6] = { 0.0 };
  double pd_dynamicNoise[6] = { 0.0 };
  double d_deltaT = 0.0;
  uint16_t pw_paraIndex[GNSS_MAX_FILTER_STATE_NUM] = { 0 };
  uint16_t pw_paraIndexExcludingPVA[GNSS_MAX_FILTER_STATE_NUM] = { 0 };
  double* pd_Q11 = NULL;
  double* pd_PhiQ11 = NULL;
  double* pd_PhiQ11PhiT = NULL;
  double* pd_Q21 = NULL;
  double* pd_Q21PhiT = NULL;
  gnss_sparseDoubleMatrix z_A = {0};
  gnss_sparseDoubleMatrix z_At = { 0 };
  w_stateNum = getAllStatusIndexEKF(pz_statusPool, pw_paraIndex);
  if (w_stateNum < PVA_NUM)
  {
    return 0;
  }
  getPVAindex(pz_statusPool, pw_pvaIndex);
  w_stateNumNonePva = getAllParaIndexExcludingPVA(w_stateNum, pw_paraIndex, pw_pvaIndex, pw_paraIndexExcludingPVA);
  d_deltaT = tm_GpsTimeDiff(&z_gpsTime, &(pz_statusPool->z_gpsTime));
  if (!calPhi(d_deltaT, d_alphaIn, u_isPPKreverse, pd_phi))
  {
    return 0;
  }
  if (!calDynamicNoise(d_deltaT, d_alphaIn, d_accSigmaInit, u_isPPKreverse, pd_dynamicNoise))
  {
    return 0;
  }
  dynPredictPVAparameter(pw_pvaIndex, pd_phi, pd_filterX);

  /* Q = FQ^F^T */
  /*
   *   |   Phi        0      |       |  Q11         Q12     |
   * F=|   9*9      9*(n-9)  |   Q^= |  9*9       9*(n-9)   | 
   *   |    0          I     |       |  Q21         Q22     |
   *   |(n-9)*9   (n-9)*(n-9)|       |(n-9)*9   (n-9)*(n-9) |
   */

  /* 
  *   | Phi*Q11*Phi^T     F*Q12    |
  * Q=|    (9*9)         9*(n-9)   |
  *   |   Q21*Phi^T        Q22     |
  *   |    (n-9)*9     (n-9)*(n-9) |
  */

  /*get Q11*/
  pd_Q11 = (double*)OS_MALLOC_FAST(NUTM(PVA_NUM) * sizeof(double));
  memset(pd_Q11, 0, sizeof(double) * NUTM(PVA_NUM));
  for (w_i = 0; w_i < PVA_NUM; ++w_i)
  {
    for (w_j = 0; w_j <= w_i; ++w_j)
    {
      pd_Q11[IUTM(w_i, w_j)] = pd_filterQ[IUTM(pw_pvaIndex[w_i], pw_pvaIndex[w_j])];
    }
  }
  /*calculate Phi*Q11(9*9)*/
  z_A.w_rowNum = z_A.w_colNum = PVA_NUM;
  q_transMeleNum = sumTransAsparseEleNum(PVA_NUM);
  z_A.q_eleNum = q_transMeleNum;
  z_A.pz_tripleEleSet = (gnss_tripleDoubleEle*)OS_MALLOC_FAST(q_transMeleNum * sizeof(gnss_tripleDoubleEle));
  gnss_initTripleDoubleEle(z_A.pz_tripleEleSet, q_transMeleNum);
  getPvaStateTransSparseMatrixA(pd_phi, &z_A);
  pd_PhiQ11 = (double*)OS_MALLOC_FAST(PVA_NUM * PVA_NUM * sizeof(double));
  memset(pd_PhiQ11, 0, sizeof(double) * PVA_NUM * PVA_NUM);
  gnss_sparseMultiplySymmetryDouble(&z_A, pd_Q11, PVA_NUM, PVA_NUM, pd_PhiQ11);
  OS_FREE(pd_Q11);
  /*calculate Phi*Q11*Phi^T*/
  z_At.w_rowNum = z_At.w_colNum = PVA_NUM;
  z_At.q_eleNum = q_transMeleNum;
  z_At.pz_tripleEleSet = (gnss_tripleDoubleEle*)OS_MALLOC_FAST(q_transMeleNum * sizeof(gnss_tripleDoubleEle));
  gnss_initTripleDoubleEle(z_At.pz_tripleEleSet, q_transMeleNum);
  getPvaStateTransSparseMatrixAt(pd_phi, &z_At);
  pd_PhiQ11PhiT= (double*)OS_MALLOC_FAST(NUTM(PVA_NUM) * sizeof(double));
  memset(pd_PhiQ11PhiT, 0, sizeof(double) * NUTM(PVA_NUM));
  gnss_fullMultiplySparseDoubleSymmetry(pd_PhiQ11, PVA_NUM, PVA_NUM, &z_At, pd_PhiQ11PhiT);
  for (w_i = 0; w_i < PVA_NUM; ++w_i)
  {
    for (w_j = 0; w_j <= w_i; ++w_j)
    {
      pd_filterQ[IUTM(pw_pvaIndex[w_i], pw_pvaIndex[w_j])] = pd_PhiQ11PhiT[IUTM(w_i, w_j)];
    }
  }
  OS_FREE(pd_PhiQ11PhiT);
  /*get Q21*/
  pd_Q21 = (double*)OS_MALLOC_FAST(w_stateNumNonePva * PVA_NUM * sizeof(double));
  memset(pd_Q21, 0, sizeof(double) * w_stateNumNonePva * PVA_NUM);
  for (w_j = 0; w_j < w_stateNumNonePva; ++w_j)
  {
    w_rowIndex = w_j * PVA_NUM;
    for (w_i = 0; w_i < PVA_NUM; ++w_i)
    {
      pd_Q21[w_rowIndex + w_i] = pd_filterQ[IUTM(pw_paraIndexExcludingPVA[w_j], pw_pvaIndex[w_i])];
    }
  }
  /*calculate Phi*Q12*/
  pd_Q21PhiT = (double*)OS_MALLOC_FAST(w_stateNumNonePva * PVA_NUM * sizeof(double));
  memset(pd_Q21PhiT, 0, sizeof(double) * w_stateNumNonePva * PVA_NUM);
  gnss_fullMultiplySparseDouble(pd_Q21, w_stateNumNonePva, PVA_NUM, &z_At, pd_Q21PhiT);
  for (w_j = 0; w_j < w_stateNumNonePva; ++w_j)
  {
    w_rowIndex = w_j * PVA_NUM;
    for (w_i = 0; w_i < PVA_NUM; ++w_i)
    {
      pd_filterQ[IUTM(pw_paraIndexExcludingPVA[w_j], pw_pvaIndex[w_i])] = pd_Q21PhiT[w_rowIndex + w_i];
    }
  }
  /* P = FPF^T + Q */
  addPvaNoise(d_deltaT, pw_pvaIndex, pd_dynamicNoise, pd_filterQ);
  updatePVAstateTime(z_gpsTime, pz_statusPool);

  OS_FREE(pd_PhiQ11);
  OS_FREE(pd_Q21);
  OS_FREE(pd_Q21PhiT);
  OS_FREE(z_A.pz_tripleEleSet);
  OS_FREE(z_At.pz_tripleEleSet);
  return 1;
}
/**
 * @brief get the index of target parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  w_id
 * @param[out] pw_paraIndex is the index of target parameter in the EKF
 * @return     void
 */
void getTargetParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const uint16_t w_id[GNSS_MAX_STATE_REP], int16_t* pw_paraIndex)
{
    const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
    if (NULL == pw_paraIndex)
    {
        return;
    }
    *pw_paraIndex = -1;
    pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
    if (NULL != pz_ekfState)
    {
        *pw_paraIndex = pz_ekfState->w_index;
    }
    return;
}
/**
 * @brief get the index of PVA parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_PVAindex is the index of PVA parameter in the EKF
 * @return     void
 */
void getPvaParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
    int16_t pw_PVAindex[PVA_NUM])
{
    uint8_t u_i = 0;
    uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
    const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
    if (NULL == pw_PVAindex)
    {
        return;
    }
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
        pw_PVAindex[u_i] = -1;
    }
    //position
    for (u_i = 0; u_i < 3; ++u_i)
    {
        w_id[0] = 0;
        w_id[1] = u_i;
        w_id[2] = GNSS_FILTER_STATE_POS;
        pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
        if (NULL != pz_ekfState)
        {
            pw_PVAindex[u_i] = pz_ekfState->w_index;
        }
    }
    //velocity
    for (u_i = 0; u_i < 3; ++u_i)
    {
        w_id[0] = 0;
        w_id[1] = u_i;
        w_id[2] = GNSS_FILTER_STATE_VEL;
        pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
        if (NULL != pz_ekfState)
        {
            pw_PVAindex[3 + u_i] = pz_ekfState->w_index;
        }
    }
    //acceleration
    for (u_i = 0; u_i < 3; ++u_i)
    {
        w_id[0] = 0;
        w_id[1] = u_i;
        w_id[2] = GNSS_FILTER_STATE_ACC;
        pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
        if (NULL != pz_ekfState)
        {
            pw_PVAindex[6 + u_i] = pz_ekfState->w_index;
        }
    }
    return;
}
/**
 * @brief get the index of ZTD parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_ZTDindex is the index of ZTD parameter in the EKF
 * @return     void
 */
void getZtdParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, int16_t* pw_ZTDindex)
{
    uint8_t u_i = 0;
    uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
    const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
    if (NULL == pw_ZTDindex)
    {
        return;
    }
    *pw_ZTDindex = -1;
    w_id[0] = 0;
    w_id[1] = 0;
    w_id[2] = GNSS_FILTER_STATE_ZTD;
    pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
    if (NULL != pz_ekfState)
    {
        *pw_ZTDindex = pz_ekfState->w_index;
    }
    return;
}
/**
 * @brief get the index of receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_rcvClkIndex is the index of receiver clock parameter in the EKF
 * @return     void
 */
void getRcvClkParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX])
{
    uint8_t u_clkNum = C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX;
    uint8_t u_i = 0;
    uint8_t u_j = 0;
    uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
    gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
    const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
    if (NULL == pw_rcvClkIndex)
    {
        return;
    }
    for (u_i = 0; u_i < u_clkNum; ++u_i)
    {
        pw_rcvClkIndex[u_i] = -1;
    }
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
        w_id[0] = 0;
        w_id[1] = u_i;
        w_id[2] = GNSS_FILTER_STATE_CLK;
        pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
        if (NULL != pz_ekfState)
        {
            pw_rcvClkIndex[u_i] = pz_ekfState->w_index;
        }
    }
    //receiver DCB
    for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
    {
        for (u_j = 1; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
        {
            z_filterType = convertFreqDCB2FilterType((gnss_FreqType)u_j);
            if (GNSS_FILTER_STATE_NUM == z_filterType)
            {
                continue;
            }
            w_id[0] = u_j;
            w_id[1] = u_i;
            w_id[2] = z_filterType;
            pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
            if (NULL != pz_ekfState)
            {
                pw_rcvClkIndex[u_j * C_GNSS_MAX + u_i] = pz_ekfState->w_index;
            }
        }
    }
    return;
}
/**
 * @brief get the index of PVA ZTD and receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_PVAindex is the index of PVA parameter in the EKF
 * @param[out] pw_ZTDindex is the index of ZTD parameter in the EKF
 * @param[out] pw_rcvClkIndex is the index of receiver clock parameter in the EKF
 * @return     1 represent success and 0 represent failure
 */
uint8_t getSiteRelativeParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
    int16_t pw_PVAindex[PVA_NUM], int16_t* pw_ZTDindex, int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX])
{
    uint8_t u_i = 0;
    uint8_t u_status = 1;
    if (NULL == pw_PVAindex || NULL == pw_ZTDindex || NULL == pw_rcvClkIndex)
    {
        return 0;
    }
    getPvaParaIndex(pz_EKFstateRepPool, pw_PVAindex);
    getZtdParaIndex(pz_EKFstateRepPool, pw_ZTDindex);
    getRcvClkParaIndex(pz_EKFstateRepPool, pw_rcvClkIndex);
    for (u_i = 0; u_i < PVA_NUM; ++u_i)
    {
        if (pw_PVAindex[u_i] < 0)
        {
            u_status = 0;
            break;
        }
    }
    if ((*pw_ZTDindex) < 0)
    {
        u_status = 0;
    }
    return u_status;
}
/**
 * @brief get the index of BDS3 receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  u_BDS3clkIndexOffset is the receiver clock for the BDS3 constellation
 * @param[out] pw_bds3RcvClkIndex is the index of BDS3 receiver clock parameter in the EKF
 * @return     void
 */
void getBDS3RcvClkParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, uint8_t u_BDS3clkIndexOffset, int16_t pw_bds3RcvClkIndex[C_GNSS_FREQ_TYPE_MAX])
{
    uint8_t u_i = 0;
    uint8_t u_j = 0;
    uint16_t w_id[GNSS_MAX_STATE_REP] = { 0 };
    gnss_FilterStateEnumType z_filterType = GNSS_FILTER_STATE_NUM;
    const gnss_EKFstateRepresent_t* pz_ekfState = NULL;
    if (NULL == pw_bds3RcvClkIndex)
    {
        return;
    }
    for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
    {
        pw_bds3RcvClkIndex[u_i] = INVALID_INDEX;
    }
    w_id[0] = 0;
    w_id[1] = C_GNSS_BDS3 + u_BDS3clkIndexOffset;
    w_id[2] = GNSS_FILTER_STATE_CLK;
    pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
    if (NULL != pz_ekfState)
    {
        pw_bds3RcvClkIndex[C_GNSS_FREQ_TYPE_L1] = pz_ekfState->w_index;
    }
    //BDS3 receiver DCB
    for (u_j = 1; u_j < C_GNSS_FREQ_TYPE_MAX; ++u_j)
    {
        z_filterType = convertFreqDCB2FilterType((gnss_FreqType)u_j);
        if (GNSS_FILTER_STATE_NUM == z_filterType)
        {
            continue;
        }
        w_id[0] = u_j;
        w_id[1] = C_GNSS_BDS3 + u_BDS3clkIndexOffset;
        w_id[2] = z_filterType;
        pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
        if (NULL != pz_ekfState)
        {
            pw_bds3RcvClkIndex[u_j] = pz_ekfState->w_index;
        }
    }
    return;
}
/**
 * @brief get velocity of horizontal and vertical
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pd_X
 * @param[out] vel_enu
 * @return     void
 */
void getEnuVel(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const double* pd_X, double* vel_enu)
{
  uint8_t u_i = 0;
  uint16_t w_id[3] = { 0 };
  double d_xyz[3] = { 0.0 };
  double d_blh[3] = { 0.0 };
  double d_vel[3] = { 0.0 };
  const gnss_EKFstateRepresent_t* pz_ekfState = NULL;

  for (u_i = 0; u_i < 3; ++u_i)
  {
    //pos
    w_id[0] = 0;
    w_id[1] = u_i;
    w_id[2] = GNSS_FILTER_STATE_POS;
    pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
    if (NULL != pz_ekfState)
    {
      d_xyz[u_i] = pd_X[pz_ekfState->w_index];
    }

    //velocity
    w_id[0] = 0;
    w_id[1] = u_i;
    w_id[2] = GNSS_FILTER_STATE_VEL;
    pz_ekfState = getEKF_status(w_id, pz_EKFstateRepPool);
    if (NULL != pz_ekfState)
    {
      d_vel[u_i] = pd_X[pz_ekfState->w_index];
    }
  }
  gnss_Ecef2Lla(d_xyz, d_blh);
  gnss_Ecef2Enu(d_blh, d_vel, vel_enu);
}

/**
 * @brief extract covariance from filter
 * @param[in] p_Q covariance
 * @param[in] pd_index index
 * @param[in] n number of index
 * @param[in] p_Qout output
 */
BOOL getPVAQ(const double* p_Q, const int16_t* pd_index, uint8_t n, double *p_Qout)
{
  BOOL status = TRUE;
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      if (pd_index[i] < 0)
      {
        status = FALSE;
        break;
      }
      p_Qout[i * n + j] = p_Q[IUTM(pd_index[i], pd_index[j])];
    }
  }
  return status;
}
