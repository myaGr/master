/**@file        ppp_integ_dt_model_ver.c
 * @brief       PPP Vertical Integrity Decision Tree Model Created by PYTHON
 * @details
 * @author      houxiaowei
 * @date        2023/5/19
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/5/19   <td>0.1      <td>houxiaowei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "mw_alloc.h"
#include "ppp_integ_model.h"

static double predict0(integ_PppFeature_t* gpz_pppFeature) {
  return 0;
}

static double predict1(integ_PppFeature_t* gpz_pppFeature) {
  return 0;
}

static double predict2(integ_PppFeature_t* gpz_pppFeature) {
  return 0;
}

static double predict3(integ_PppFeature_t* gpz_pppFeature) {
  return 0;
}

static double predict4(integ_PppFeature_t* gpz_pppFeature) {
  return 0;
}

/**
 * @brief Vertical protection level by Bagging DT model, max value from 10 trees model and EKF sigma
 * @param[in] flag gnss position status
 * @param[in] use_sat_num_float carrier phase satellite numbers used in float EKF
 * @param[in] pdop PDOP with all satellite measurements used in  include pseudo-range and carrier-phase by float EKF
 * @param[in] cno_avg average of CN0 that all signals
 * @param[in] cno_std standard of CN0 that all signals
 * @param[in] cmc_avg average ode-minus-carrier (CMC) metric use the signals with marked valid by PVT
 * @param[in] cmc_std standard ode-minus-carrier (CMC) metric use the signals with marked valid by PVT
 * @param[in] avg_unc_pr the uncertainties of pseudo-range by PVT
 * @param[in] pl_ver_ekf sigma*4.42, sigma from EKF
 * @return 0.0 error; other suc
 */
double integ_DecisionTreePredictVer(integ_PppFeature_t* gpz_pppFeature)
{
  double* pd_predict_list = OS_MALLOC_FAST(sizeof(double) * 7);
  double d_pred = 0.0;
  if (NULL == pd_predict_list)
  {
    return 0.0;
  }

  /* predict value from 10 trees */
  pd_predict_list[0] = predict0(gpz_pppFeature);
  pd_predict_list[1] = predict1(gpz_pppFeature);
  pd_predict_list[2] = predict2(gpz_pppFeature);
  pd_predict_list[3] = predict3(gpz_pppFeature);
  pd_predict_list[4] = predict4(gpz_pppFeature);
  pd_predict_list[5] = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  pd_predict_list[6] = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];

  /* Bagging, max value from 10 trees model and pl_ekf*/
  d_pred = vector_dbl_max(pd_predict_list, 7);

  /* 0.5 BIAS */
  d_pred += 0.5;

  OS_FREE(pd_predict_list);
  return d_pred;
}
