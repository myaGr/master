/**@file        fusion_kf.c
 * @brief		fusion kf algorithm file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "fusion_err_model.h"
#include "fusion_global.h"
#include "fusion_math.h"
#include "fusion_log.h"
#include <stdint.h>
#include <string.h>
#include "mw_alloc.h"
#include "mw_log.h"

 /*
 * @brief: Implement innovation statistics
 * @input: v - residual; SdDim - System Dim; MeDim - Meas Dim; pz_sysmodel - Error model
 * @output: None
 * @return: lambda
 */
float kf_calcu_lambda(float v[], uint8_t StDim, uint8_t MeDim, SysErrModel_t* pz_sysmodel)
{
  uint8_t mdim = pz_sysmodel->u_meadim;

  float* residual = (float*)OS_MALLOC(MeDim * sizeof(float));   

  float* KKK = (float*)OS_MALLOC(StDim * MeDim * sizeof(float));
  float* MatForInv = (float*)OS_MALLOC(MeDim * MeDim * sizeof(float));
  float* MeasMatrixT = (float*)OS_MALLOC(StDim * MeDim * sizeof(float));
  float* MeasMatrix = (float*)OS_MALLOC(MeDim * StDim * sizeof(float));
  float* MeaNoiseMatrix = (float*)OS_MALLOC(MeDim * MeDim * sizeof(float));
    
  float Lamida = 0.0f;

  float* resiMatForInv = (float*)OS_MALLOC(MeDim * sizeof(float));
  float resi_temp[1] = { 0.0f };

  uint8_t i;
  uint8_t ret_flag = INS_FALSE;

  for (i = 0; i < MeDim; i++)
  {
    residual[i] = v[i];
  }

  math_matextract(&(pz_sysmodel->f_sysmeasmat[0]), StDim, MeasMatrix, MeDim, StDim, 0, 0);
  math_matextract(&(pz_sysmodel->f_measnoise[0]), mdim, MeaNoiseMatrix, MeDim, MeDim, 0, 0);

  math_mattrns(MeasMatrix, MeDim, StDim, MeasMatrixT);
  math_matmul(&(pz_sysmodel->f_syscovmat[0]), MeasMatrixT, StDim, StDim, MeDim, KKK);

  math_matmul(MeasMatrix, KKK, MeDim, StDim, MeDim, MatForInv);
  math_matadd(MatForInv, MeaNoiseMatrix, MeDim, MeDim, MatForInv);

  ret_flag = math_mat_calinverse(MatForInv, MeDim);

  if (ret_flag == INS_TRUE)
  {
    math_matmul(residual, MatForInv, 1, MeDim, MeDim, resiMatForInv);
    math_matmul(resiMatForInv, residual, 1, MeDim, 1, resi_temp);
    Lamida = fabsf(resi_temp[0]);
  }
  else
  {
    Lamida = 9999999.9f;
  }

  OS_FREE(residual);
  OS_FREE(KKK);
  OS_FREE(MatForInv);
  OS_FREE(MeasMatrixT);
  OS_FREE(MeasMatrix);
  OS_FREE(MeaNoiseMatrix);
  OS_FREE(resiMatForInv);

  return Lamida;
}

 /*
* @brief: Kalman Predict Function
* @input: d_tsp - system time; pz_sysmodel - Error model
* @output: None
* @return: None
*/
void kf_predict(double d_tsp, SysErrModel_t* pz_sysmodel)
{
  uint8_t sdim = pz_sysmodel->u_sysdim;
  float* TranMt = (float*)OS_MALLOC(sdim * sdim * sizeof(float));
  float* Qk = (float*)OS_MALLOC(sdim * sdim * sizeof(float));
  double d_delta_time = 0.0;
  float* PHIP = (float*)OS_MALLOC(sdim * sdim * sizeof(float));
  float* Qk_temp = (float*)OS_MALLOC(sdim * sdim * sizeof(float));

  uint8_t i, j;

  float* TranMat = (float*)OS_MALLOC(sdim * sdim * sizeof(float));

  memcpy(TranMat, pz_sysmodel->f_systrsmat, sdim * sdim * sizeof(float));

  d_delta_time = pz_sysmodel->d_predict_time;
  d_delta_time = (d_delta_time < INFINITY_MIN) ? (pz_sysmodel->f_prdct_dt) : (d_tsp - d_delta_time);
  pz_sysmodel->d_predict_time = d_tsp;

  memcpy(TranMt, TranMat, sdim * sdim * sizeof(float));
  math_square_transpose(&(TranMt[0]), sdim);

  for (i = 0; i < sdim; i++)
  {
    for (j = 0; j < sdim; j++)
    {
      Qk_temp[j * sdim + i] = TranMat[j * sdim + i] * pz_sysmodel->f_sysnoise[i];
    }
  }
  math_matmul(&(Qk_temp[0]), &(TranMt[0]), sdim, sdim, sdim, &(Qk[0]));

  for (i = 0; i < sdim; i++)
  {
    for (j = 0; j < sdim; j++)
    {
      Qk[i * sdim + j] *= (float)d_delta_time;
    }
  }

  /*	P1 = PHI * P * PHI_t + Qd;   */
  math_matmul(&(TranMat[0]), &(pz_sysmodel->f_syscovmat[0]), sdim, sdim, sdim, &(PHIP[0]));
  math_matmul(&(PHIP[0]), &(TranMt[0]), sdim, sdim, sdim, &(pz_sysmodel->f_syscovmat[0]));
  math_matadd(&(pz_sysmodel->f_syscovmat[0]), &(Qk[0]), sdim, sdim, &(pz_sysmodel->f_syscovmat[0]));

  OS_FREE(TranMt);
  OS_FREE(Qk);
  OS_FREE(PHIP);
  OS_FREE(Qk_temp);
  OS_FREE(TranMat);

  return;
}

/*
* @brief: Kalman Update Function
* @input: z - Observed value; SdDim - System Dim; MeDim - Meas Dim; pz_sysmodel - Error model
* @output: None
* @return: success flag
*/
uint8_t kf_update(float z[], uint8_t StDim, uint8_t MeDim, SysErrModel_t* pz_sysmodel)
{
  uint8_t u_val = INS_FALSE;

  uint8_t mdim = pz_sysmodel->u_meadim;

  float* KKK = (float*)OS_MALLOC(StDim * MeDim *sizeof(float));
  float* MatForInv = (float*)OS_MALLOC(MeDim * MeDim * sizeof(float));
  float* MeasMatrixT = (float*)OS_MALLOC(StDim * MeDim * sizeof(float));
  float* MeasMatrix = (float*)OS_MALLOC(MeDim * StDim * sizeof(float));
  float* MeaNoiseMatrix = (float*)OS_MALLOC(MeDim * MeDim * sizeof(float));
  float MatDet = 0.0f;
  float* InvTran = (float*)OS_MALLOC(MeDim * MeDim * sizeof(float));
  float* MatForInv_Temp = (float*)OS_MALLOC(MeDim * MeDim * sizeof(float));
  float* KKK_Temp = (float*)OS_MALLOC(StDim * MeDim * sizeof(float));
  float* Imat = (float*)OS_MALLOC(StDim * StDim * sizeof(float));
  float* IKH = (float*)OS_MALLOC(StDim * StDim * sizeof(float));
  float* KKKMea = (float*)OS_MALLOC(StDim * StDim * sizeof(float));
  float* IKHt = (float*)OS_MALLOC(StDim * StDim * sizeof(float));
  float* IKHST = (float*)OS_MALLOC(StDim * StDim * sizeof(float));
  float* KKKt = (float*)OS_MALLOC(MeDim * StDim * sizeof(float));
  float* KKKMN = (float*)OS_MALLOC(StDim * MeDim * sizeof(float));
  float* KKKMN_KKKt = (float*)OS_MALLOC(StDim * StDim * sizeof(float));
  uint8_t i, j;  

  math_matextract(&(pz_sysmodel->f_sysmeasmat[0]), StDim, MeasMatrix, MeDim, StDim, 0, 0);
  math_matextract(&(pz_sysmodel->f_measnoise[0]), mdim, MeaNoiseMatrix, MeDim, MeDim, 0, 0);

  math_mattrns(MeasMatrix, MeDim, StDim, MeasMatrixT);
  math_matmul(&(pz_sysmodel->f_syscovmat[0]), MeasMatrixT, StDim, StDim, MeDim, KKK); /* PHt */

  math_matmul(MeasMatrix, KKK, MeDim, StDim, MeDim, MatForInv); /* H*PHt */
  math_matadd(MatForInv, MeaNoiseMatrix, MeDim, MeDim, MatForInv); /* HPHt+R */

  if ((math_choleskydc(MatForInv, MeDim) >= (int8_t)0) && (math_matinv(MatForInv, MeDim, &MatDet) >= (int8_t)0))
  {
    math_matcopy(InvTran, MatForInv, MeDim, MeDim);
    math_square_transpose(InvTran, MeDim);

    math_matmul(MatForInv, InvTran, MeDim, MeDim, MeDim, MatForInv_Temp); /* inv(HPHt+R) */
    math_matcopy(MatForInv, MatForInv_Temp, MeDim, MeDim);
        
    math_matmul(KKK, MatForInv, StDim, MeDim, MeDim, KKK_Temp); /* PHt * inv(HPHt+R) */
    math_matcopy(KKK, KKK_Temp, StDim, MeDim);

    math_matmul(KKK, z, StDim, MeDim, 1, pz_sysmodel->f_errstate); /* x=Kz */

    math_mateye(&(Imat[0]), StDim);

    math_matmul(KKK, MeasMatrix, StDim, MeDim, StDim, &(KKKMea[0]));
    math_matsub(&(Imat[0]), &(KKKMea[0]), StDim, StDim, &(IKH[0])); /* I-KH */

    memcpy(IKHt, IKH, StDim * StDim * sizeof(float));
    math_square_transpose(&(IKHt[0]), StDim); /* (I-KH)' */

    math_matmul(&(IKH[0]), &(pz_sysmodel->f_syscovmat[0]), StDim, StDim, StDim, &(IKHST[0]));
    math_matmul(&(IKHST[0]), &(IKHt[0]), StDim, StDim, StDim, &(pz_sysmodel->f_syscovmat[0])); /* (I-KH)*P*(I-KH)' */

    math_mattrns(KKK, StDim, MeDim, KKKt);
    math_matmul(KKK, MeaNoiseMatrix, StDim, MeDim, MeDim, KKKMN);

    math_matmul(KKKMN, KKKt, StDim, MeDim, StDim, KKKMN_KKKt); /* K * R * K' */
    math_matadd(&(pz_sysmodel->f_syscovmat[0]), KKKMN_KKKt, StDim, StDim, &(pz_sysmodel->f_syscovmat[0]));

    math_matcopy(&(Imat[0]), &(pz_sysmodel->f_syscovmat[0]), StDim, StDim);
    math_square_transpose(&(Imat[0]), StDim); /* P' */

    math_matadd(&(pz_sysmodel->f_syscovmat[0]), &(Imat[0]), StDim, StDim, &(pz_sysmodel->f_syscovmat[0])); /* P+P' */

    for (i = 0; i < StDim; i++)
    {
      for (j = 0; j < StDim; j++)
      {
        pz_sysmodel->f_syscovmat[i*StDim +j] *= 0.5f; /* (P+P')/2 */
      }
    }
    u_val = INS_TRUE;
  }

  OS_FREE(KKK);
  OS_FREE(MatForInv);
  OS_FREE(MeasMatrixT);
  OS_FREE(MeasMatrix);
  OS_FREE(MeaNoiseMatrix);
  OS_FREE(InvTran);
  OS_FREE(MatForInv_Temp);
  OS_FREE(KKK_Temp);
  OS_FREE(Imat);
  OS_FREE(IKH);
  OS_FREE(KKKMea);
  OS_FREE(IKHt);
  OS_FREE(IKHST);
  OS_FREE(KKKt);
  OS_FREE(KKKMN);
  OS_FREE(KKKMN_KKKt);

  return u_val;
}
