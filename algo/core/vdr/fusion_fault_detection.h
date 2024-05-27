/**@file        fusion_fault_detection.h
 * @brief		fault detection header file
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

#ifndef __FUSION_FAULT_DETECTION_H__
#define __FUSION_FAULT_DETECTION_H__

#include "fusion_gnss.h"
#include "fusion_wheel.h"
#include "fusion_if.h"

void fusion_fault_detection_handler(AsensingIMU_t* pz_imu, GNSSFormMeas_t* pz_form_meas, WheelSpd_t* pz_whsig, INSResults_t* pz_ins);

#endif // !__FUSION_FAULT_DETECTION_H__
