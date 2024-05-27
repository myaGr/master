/**@file        fusion_zupt.h
 * @brief		fusion zupt header file
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

#ifndef __FUSION_ZUPT_H__
#define __FUSION_ZUPT_H__

#include "fusion_mech.h"
#include "fusion_proc.h"
#include "fusion_global.h"
#include <stdint.h>

#define GYRO_MAX_STD ((float)(0.5*DEG2RAD))
#define ACCL_MAX_STD (0.1f)

#define ZUPT_STD_BUF_SIZE 10
#define ZUPT_IMU_BUF_SIZE 10
#define ZUPT_INIT_TIME    5.f
#define ZUPT_THRES_SIGMA  3.5f
#define ZUPT_DIFF_LIMIT_L 0.1f
#define ZUPT_GROUP_SIZE  3

typedef struct
{
  double d_imu_t;
  float f_zupt_buf[6][ZUPT_STD_BUF_SIZE];
  float f_imu_buf[6][ZUPT_IMU_BUF_SIZE];
  float f_imu_group[6][ZUPT_GROUP_SIZE];
  float f_zupt_val[6];
  float f_zupt_threshold[6];
  float f_norm_std;
  uint32_t u_val_counter;
  uint8_t u_imu_counter;
  uint8_t u_zupt_counter;
  uint8_t u_zupt_valid;
  uint8_t u_zupt_inited;
  uint8_t u_group_counter;
  int8_t s_zupt_indicator;
} AutoZuptDetect_t;

AutoZuptDetect_t* fusion_get_autozupt_para(void);
void auto_zupt_detection(double pd_meas[], NavParams_t* pz_para);
uint8_t zupt_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* uz_mdim, InertialNav_t* pz_inav);

#endif // !__FUSION_ZUPT_H__