/**@file        fusion_math.h
 * @brief		fusion math header file
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

#ifndef __FUSION_KF_H__
#define __FUSION_KF_H__

#include "fusion_err_model.h"
#include <stdint.h>

void kf_predict(double d_tsp, SysErrModel_t* pz_sysmodel);
uint8_t kf_update(float z[], uint8_t StDim, uint8_t MeDim, SysErrModel_t* pz_sysmodel);
float kf_calcu_lambda(float v[], uint8_t StDim, uint8_t MeDim, SysErrModel_t* sysmodel);

#endif // !__FUSION_KF_H__
