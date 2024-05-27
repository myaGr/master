/**@file        fusion_feedback.h
 * @brief		fusion feedback header file
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

#ifndef __FUSION_FEEDBACK_H__
#define __FUSION_FEEDBACK_H__

#include "fusion_proc.h"

void kf_bias_feedback(const float f_errstate[], NavParams_t* pz_para);
void kf_wssf_feedback(const float f_errstate[], NavParams_t* pz_para);
void kf_att_feedback(const float f_errstate[], NavParams_t* pz_nav);
void kf_pos_feedback(const float f_errstate[], NavParams_t* pz_nav);
void kf_tdcp_pos_feedback(const float f_errstate[], InertialNav_t* pz_inav);
void kf_vel_feedback(const float f_errstate[], NavParams_t* pz_nav);
void kf_misalign_feedback(const float f_errstate[], NavParams_t* pz_nav);
void kf_feedback_procedure(const float* pf_errstate, InertialNav_t* pz_inav, NavParams_t* pz_para);
void kf_feedback_handle(uint8_t isSuccess, SysErrModel_t* pz_modelTmp);

#endif // !__FUSION_FEEDBACK_H__
