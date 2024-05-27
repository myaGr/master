/**@file        fusion_wheel.h
 * @brief		wheel signal header file
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

#ifndef __FUSION_WHEEL_H__
#define __FUSION_WHEEL_H__

#include "fusion_api.h"
#include "fusion_proc.h"
#include <stdint.h>

typedef struct
{
  double d_timestamp;
  float f_vehspd_fl;
  float f_vehspd_fr;
  float f_vehspd_rl;
  float f_vehspd_rr;
  float f_whangle[4];
  int8_t u_dir;
  uint8_t shifter;
} WheelSpd_t;

WheelSpd_t* fusion_get_wheelspd(void);
uint8_t whspd_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav);
uint8_t odo_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav);
uint8_t whspd_steer_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav);
void whpls_convert_to_whspd(WheelSpd_t* pz_whspd, AsensingWhpulse_t* pz_whpls);

#endif // !__FUSION_WHEEL_H__
