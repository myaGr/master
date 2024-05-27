/**********************************************************************************
* @note
* @par History :
*<table>
* <tr><th>Date        <th>Version  <th>Author      <th>Description
* <tr><td>2022/09/08  <td> 0.1     <td>chenjinhe   <td>Init version
* < / table>
*
**********************************************************************************
*/
#ifndef __SEQ_KALMAN_FLOAT_H__
#define __SEQ_KALMAN_FLOAT_H__
#include "cmn_def.h"

BEGIN_DECL
/* the struct used by sequential kalman filter,the type is float */
typedef struct 
{
  float*    pf_h;     /* the design matrix, and there will only store non-zero element */
  float*    pf_hP;    /* H*P */
  float*    pf_k;     /* gain matrix */
  uint16_t* pw_L;     /* the index of parameter in the kalman filter,and there will only store non-zero element */
  uint16_t  w_paraNum;/* the total number of parameter */
  uint16_t  w_n;      /* the number of parameter for single observation */
  float     f_Z;      /* prior OMC */
  float     f_R;      /* the variance of observation */
  float     f_pZ;     /* H(k)* P(k)_* H(k)'+R(k) */
  float     f_zL;     /* H*deltaX */
  float     f_rL;     /* innovation*innovation/f_pZ */
  float     f_res;    /* innovation */
}SeqKalmanFloatVar_t;

/**
 * @brief add the non-zero element of design matrix to the sequential kalman filter
 * @param[in]  w_curL is the paramter index in the kalman filter, there will only push non-zero elements to the filter
 * @param[in]  f_curH is the element in design matrix
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqAddHfloat(uint16_t w_curL, float f_curH, SeqKalmanFloatVar_t* pz_seqVar);

/**
 * @brief clean the variable of sequential kalman filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqCleanHfloat(SeqKalmanFloatVar_t* pz_seqVar);

/**
 * @brief add the observation information(include OMC and variance) to the sequential kalman filter
 * @param[in]  f_curOmc is observation minus calculatiion value
 * @param[in]  f_curR is variance of observation
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqSetOmcFloat(float f_curOmc, float f_curR, SeqKalmanFloatVar_t* pz_seqVar);

/**
 * @brief add predict step
 * @param[out] pf_deltaX is delta X relative to parameter init value
 * @param[in]  pf_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqPredictStepFloat(float* pf_deltaX, float* pf_Pk, SeqKalmanFloatVar_t* pz_seqVar);

/**
 * @brief using the observation to do measure update the kalman filter
 * @param[out] pf_xk is the value of parameter X in the kalman filter
 * @param[out] pf_deltaX is delta X relative to parameter init value
 * @param[in]  pf_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @param[in]  f_outlierThres is rejected threshold used innovation
 * @return     if the return value is equal to 0,the observation had been rejected by innovation.
 */
extern uint16_t hpp_seqMeasUpdateFloat(float* pf_xk, float* pf_deltaX, float* pf_Pk,
	SeqKalmanFloatVar_t* pz_seqVar, float f_outlierThres);

END_DECL
#endif