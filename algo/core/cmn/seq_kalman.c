#include "seq_kalman.h"
#include "gnss_type.h"
#include "gnss_def.h"
#include "mw_alloc.h"
#include "gnss_common.h"
#include "cmn_utils.h"

/**
 * @brief Sequential filtering init
 * @param[in] pz_seqVar struct of sequential kalman filter
 * @param[in] w_paraNum number of state parameter
 * @return 0:sucess, others:failure
 */
extern uint8_t hpp_seq_init(SeqKalmanVar_t* pz_seqVar, uint16_t w_paraNum)
{
  pz_seqVar->w_paraNum = w_paraNum;
  pz_seqVar->pd_h = (double*) OS_MALLOC_FAST(w_paraNum * sizeof(double));
  pz_seqVar->pd_hP = (double*) OS_MALLOC_FAST(w_paraNum * sizeof(double));
  pz_seqVar->pd_k = (double*) OS_MALLOC_FAST(w_paraNum * sizeof(double));
  pz_seqVar->pw_L = (int16_t*) OS_MALLOC_FAST(w_paraNum * sizeof(int16_t));

  if (any_Ptrs_Null(4, pz_seqVar->pd_h, pz_seqVar->pd_hP, pz_seqVar->pd_k, pz_seqVar->pw_L))
  {
    OS_FREE(pz_seqVar->pd_h);
    OS_FREE(pz_seqVar->pd_hP);
    OS_FREE(pz_seqVar->pd_k);
    OS_FREE(pz_seqVar->pw_L);
    return 1;
  }

  /* parameter index defaut setted to -1 */
  for (int32_t i = 0; i < w_paraNum; ++i)
  {
    pz_seqVar->pw_L[i] = INVALID_INDEX;
  }
  return 0;
}

/**
 * @brief Sequential filtering deinit
 * @param pz_seqVar struct of sequential kalman filter
 * @return
 */
extern void hpp_seq_deinit(SeqKalmanVar_t* pz_seqVar)
{
  OS_FREE(pz_seqVar->pd_h);
  OS_FREE(pz_seqVar->pd_hP);
  OS_FREE(pz_seqVar->pd_k);
  OS_FREE(pz_seqVar->pw_L);
  return;
}

/**
 * @brief add the non-zero element of design matrix to the sequential kalman filter
 * @param[in]  w_curL is the paramter index in the kalman filter, there will only push non-zero elements to the filter
 * @param[in]  d_curH is the element in design matrix
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_AddH(int16_t w_curL, double d_curH, SeqKalmanVar_t* pz_seqVar)
{
  uint16_t w_i = 0;
  if (NULL != pz_seqVar && w_curL >= 0 && ((uint16_t)w_curL) < (pz_seqVar->w_paraNum))
  {
    for (w_i = 0; w_i < (pz_seqVar->w_paraNum); ++w_i)
    {
      if ((pz_seqVar->pw_L[w_i]) == w_curL)
      {
        break;
      }
    }
    if (w_i == (pz_seqVar->w_paraNum) && (pz_seqVar->w_n) < (pz_seqVar->w_paraNum))
    {
      pz_seqVar->pd_h[pz_seqVar->w_n] = d_curH;
      pz_seqVar->pw_L[pz_seqVar->w_n] = w_curL;
      ++(pz_seqVar->w_n);
    }
  }
  return;
}
/**
 * @brief clean the variable of sequential kalman filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_ClearH(SeqKalmanVar_t* pz_seqVar)
{
  uint16_t i = 0;
  if (NULL!=pz_seqVar)
  {
    for (i = 0; i < (pz_seqVar->w_paraNum); ++i)
    {
      pz_seqVar->pd_h[i] = 0.0;
      pz_seqVar->pw_L[i] = INVALID_INDEX;
    }
    pz_seqVar->w_n = 0;
    pz_seqVar->d_Z = 0.0;
    pz_seqVar->d_R = 0.0;
    pz_seqVar->d_pZ = 0.0;
    pz_seqVar->d_zL = 0.0;
    pz_seqVar->d_rL = 0.0;
    pz_seqVar->d_res = 0.0;
  }
  return;
}
/**
 * @brief add the observation information(include OMC and variance) to the sequential kalman filter
 * @param[in]  d_curOmc is observation minus calculatiion value
 * @param[in]  d_curR is variance of observation
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_SetOMC(double d_curOmc, double d_curR, SeqKalmanVar_t* pz_seqVar)
{
  if (NULL != pz_seqVar)
  {
    pz_seqVar->d_Z = d_curOmc;
    pz_seqVar->d_R = d_curR;
  }
  return;
}
/**
 * @brief add predict step
 * @param[out] pd_deltaX is delta X relative to parameter init value
 * @param[in]  pd_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_PredictStep(double* pd_deltaX, double* pd_Pk, SeqKalmanVar_t* pz_seqVar)
{
  uint16_t n = pz_seqVar->w_paraNum;
  uint16_t m = pz_seqVar->w_n;
  //H(k)*P(k)*H(k)'
  uint16_t j = 0, k = 0;
  for (j = 0; j < n; ++j)
  {
    pz_seqVar->pd_hP[j] = 0.0;
    for (k = 0; k < m; ++k)
    {
      pz_seqVar->pd_hP[j] += pz_seqVar->pd_h[k] * pd_Pk[IUTM(pz_seqVar->pw_L[k], j)];
    }
  }
  //Pz=H(k)*P(k)_*H(k)'+R(k)
  pz_seqVar->d_pZ = pz_seqVar->d_R;
  for (k = 0; k < m; ++k)
  {
    (pz_seqVar->d_pZ) += pz_seqVar->pd_hP[pz_seqVar->pw_L[k]] * pz_seqVar->pd_h[k];
  }
  //z_=z-Hx
  pz_seqVar->d_zL = 0.0;
  for (k = 0; k < m; ++k)
  {
    (pz_seqVar->d_zL) += pz_seqVar->pd_h[k] * pd_deltaX[pz_seqVar->pw_L[k]];
  }
  pz_seqVar->d_res = pz_seqVar->d_Z - (pz_seqVar->d_zL);
  pz_seqVar->d_rL = ((pz_seqVar->d_res) * (pz_seqVar->d_res)) / (pz_seqVar->d_pZ);
  return;
}
/**
 * @brief using the observation to do measure update the kalman filter
 * @param[in]  pq_paraValid represent the mask the parameter is added to the filter
 * @param[out] pd_xk is the value of parameter X in the kalman filter
 * @param[out] pd_deltaX is delta X relative to parameter init value
 * @param[in]  pd_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @param[in]  d_outlierThres is rejected threshold used innovation
 * @return     if the return value is equal to 0,the observation had been rejected by innovation.
 */
extern uint16_t hpp_seq_measUpdate(const BOOL* pq_paraValid, double* pd_xk, double* pd_deltaX, double* pd_Pk,
    SeqKalmanVar_t* pz_seqVar, double d_outlierThres)
{
  uint16_t is_continued = 1, n = pz_seqVar->w_paraNum, i = 0, j = 0;
  double Pz_ = 1.0 / (pz_seqVar->d_pZ), innov = 0.0, temp = 0.0;
  if (d_outlierThres > 0.0 && (pz_seqVar->d_rL) > (d_outlierThres * d_outlierThres))
  {
    is_continued = 0;
  }
  if (1 == is_continued)
  {
    //K=P(k)_*H(k)'*inv(Pz)=(H(k)*P(k)_)'*inv(Pz)
    for (i = 0; i < n; ++i)
    {
        if (FALSE == pq_paraValid[i])
        {
            continue;
        }
        pz_seqVar->pd_k[i] = pz_seqVar->pd_hP[i] * Pz_;
    }
    //P(k)=P(k)-K*H(k)*P(k)_
    for (i = 0; i < n; ++i)
    {
      if (FALSE == pq_paraValid[i])
      {
         continue;
      }
      for (j = 0; j <= i; ++j)
      {
        if (FALSE == pq_paraValid[j])
        {
          continue;
        }
        pd_Pk[IUTM(i, j)] -= pz_seqVar->pd_k[i] * pz_seqVar->pd_hP[j];
      }
    }
    innov = pz_seqVar->d_Z - pz_seqVar->d_zL;
    //x=x+K*(z-z_)
    for (i = 0; i < n; ++i)
    {
      if (FALSE == pq_paraValid[i])
      {
        continue;
      }
      temp = pz_seqVar->pd_k[i] * innov;
      pd_deltaX[i] += temp;
      pd_xk[i] += temp;
    }
  }
  return is_continued;
}
