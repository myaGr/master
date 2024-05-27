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
#ifndef __SEQ_KALMAN_H__
#define __SEQ_KALMAN_H__
#include "stdint.h"
#include "cmn_def.h"

BEGIN_DECL
/* the struct used by sequential kalman filter,the type is double */
typedef struct 
{
  double*  pd_h;     /* the design matrix, and there will only store non-zero element */
  double*  pd_hP;    /* H*P */
  double*  pd_k;     /* gain matrix, w_paraNum */
  int16_t* pw_L;     /* the index of parameter in the kalman filter,and there will only store non-zero element */
  uint16_t w_paraNum;/* the total number of parameter */
  uint16_t w_n;      /* the number of parameter for single observation */
  double   d_Z;      /* prior OMC */
  double   d_R;      /* the variance of observation */
  double   d_pZ;     /* H(k)* P(k)_* H(k)'+R(k) */
  double   d_zL;     /* H*deltaX */
  double   d_rL;     /* innovation*innovation/f_pZ */
  double   d_res;    /* innovation */
}SeqKalmanVar_t;

/**
 * @brief Sequential filtering init
 * @param[in] pz_seqVar struct of sequential kalman filter
 * @param[in] w_paraNum number of state parameter
 * @return 0:sucess, others:failure
 */
extern uint8_t hpp_seq_init(SeqKalmanVar_t* pz_seqVar, uint16_t w_paraNum);

/**
 * @brief Sequential filtering deinit
 * @param pz_seqVar struct of sequential kalman filter
 * @return
 */
extern void hpp_seq_deinit(SeqKalmanVar_t* pz_seqVar);

/**
 * @brief add the non-zero element of design matrix to the sequential kalman filter
 * @param[in]  w_curL is the paramter index in the kalman filter, there will only push non-zero elements to the filter
 * @param[in]  d_curH is the element in design matrix
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_AddH(int16_t w_curL, double d_curH, SeqKalmanVar_t* pz_seqVar);

/**
 * @brief clean the variable of sequential kalman filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_ClearH(SeqKalmanVar_t* pz_seqVar);

/**
 * @brief add the observation information(include OMC and variance) to the sequential kalman filter
 * @param[in]  d_curOmc is observation minus calculatiion value
 * @param[in]  d_curR is variance of observation
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_SetOMC(double d_curOmc, double d_curR, SeqKalmanVar_t* pz_seqVar);

/**
 * @brief add predict step
 * @param[out] pd_deltaX is delta X relative to parameter init value
 * @param[in]  pd_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is double
 * @return     void
 */
extern void hpp_seq_PredictStep(double* pd_deltaX, double* pd_Pk, SeqKalmanVar_t* pz_seqVar);

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
	SeqKalmanVar_t* pz_seqVar, double d_outlierThres);


END_DECL

#endif