/**@file        ppp_integ_model.h
 * @brief       Interface of Calculate PPP Horizontal/Vertical Integrity
 * @details
 * @author      houxiaowei
 * @date        2023/5/16
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/5/16   <td>0.1      <td>houxiaowei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __GNSS_ENGINE_INTEGRITY_DT_MODEL_H__
#define __GNSS_ENGINE_INTEGRITY_DT_MODEL_H__

#include "cmn_utils.h"
#include "gnss_type.h"
#include "integ_type.h"

 /**
  * @brief horizontal protection level by Bagging DT model, max value from 10 trees model and EKF sigma
  * @param[in]
  * @return
  */
double integ_DecisionTreePredictHorFloat(integ_PppFeature_t* gpz_pppFeature);

/**
 * @brief horizontal protection level by Bagging DT model, max value from 10 trees model and EKF sigma
 * @param[in]
 * @return
 */
double integ_DecisionTreePredictHorFix(integ_PppFeature_t* gpz_pppFeature);

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
 * @return
 */
double integ_DecisionTreePredictVer(integ_PppFeature_t* gpz_pppFeature);

/**
 * @brief determine scene type by Bagging DT model, max value from 4 trees model
 * @param[in]
 * @return
 */
uint8_t integ_DecisionTreePredictScene(integ_PppFeature_t* gpz_pppFeature);

#endif //__GNSS_ENGINE_INTEGRITY_DT_MODEL_H__
