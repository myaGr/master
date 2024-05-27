/**@file        fusion_config.h
 * @brief		fusion config file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_CONFIG_H__
#define __FUSION_CONFIG_H__

#include <stdint.h>
#include "fusion_err_model.h"

#define LVRM_NHC_BIT 1
#define LVRM_ODO_BIT 2
#define LVRM_REAR_MID_BIT 3
#define LVRM_TWO_REAR_BIT 4
#define LVRM_FRONT_REAR_BIT 5

#define TYPE_MASK_LVRM_NHC (0x1 << LVRM_NHC_BIT)
#define TYPE_MASK_LVRM_ODO  (0x1 << LVRM_ODO_BIT)
#define TYPE_MASK_LVRM_REAR_MID (0x1 << LVRM_REAR_MID_BIT)
#define TYPE_MASK_LVRM_TWO_REAR (0x1 << LVRM_TWO_REAR_BIT)
#define TYPE_MASK_LVRM_FRONT_REAR (0x1 << LVRM_FRONT_REAR_BIT)

typedef struct
{
  float f_init_posvar[3];
  float f_init_velvar[3];
  float f_init_attvar[3];
  float f_init_misvar[3];
  float f_init_gyrovar[3];
  float f_init_acclvar[3];
  float f_init_odosfvar[4];
  float f_init_clockvar[3];
  float f_init_clockdiffvar;
  float f_init_lai2gvar[3];
  float f_init_lai2vvar[3];
  float f_init_misdualantvar[2];
  float f_init_wheelbasevar;
  float f_noise[30];
  float f_aerr_model[3];
  float f_gerr_model[3];

  /* GNSS config */
  float f_pos_scl_sps;
  float f_vel_scl_sps;
  float f_pos_scl_dif;
  float f_vel_scl_dif;
  float f_pos_scl_flt;
  float f_vel_scl_flt;
  float f_pos_scl_rtk;
  float f_vel_scl_rtk;

  float f_nhc_zvar;
  float f_nhc_yvar;

  float f_zupt_velvar;
  float f_zupt_yawvar;

  float f_calc_rate;
  float f_lambda_pos1;
  float f_lambda_pos2;
  float f_lambda_vel1;
  float f_lambda_vel2;

  float f_init_align[3];
  float f_timeout;

  uint8_t u_whspd_enable;    /* 0- disable, 1- enable*/
  uint8_t u_quick_pos_enable;
  uint8_t u_i2g_opt;	     /* gnss2imu lever arm flag */
  uint8_t u_ws_opt;          /* 1 -NHC, 2 - ODO , 3 - rearmiddle, 4 - two-rear, 5 - rear-front-wibbz */
  uint8_t u_misb2g_opt;	     /* dualant misangle flag, 1:already estimated, 0: to be estimated */
  uint8_t u_misb2v_opt;	     /* imu misangle flag, 1:already estimated, 0: to be estimated */
} NavConfig_t;

NavConfig_t* fusion_get_navconfig(void);
void navconfig_initial(void);

#endif // !__FUSION_CONFIG_H__
