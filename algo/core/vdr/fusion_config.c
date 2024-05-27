/**@file        fusion_config.c
 * @brief		fusion config file
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

#include "fusion_config.h"
#include "fusion_global.h"
#include "fusion_if.h"
#include "fusion_api.h"
#include <math.h>
#include "mw_alloc.h"

static NavConfig_t gz_navconfig;

NavConfig_t* fusion_get_navconfig(void)
{
  return &gz_navconfig;
}

void navconfig_initial(void)
{
  float f_accl_vrw[3] = { 0.0f };
  float f_gyro_arw[3] = { 0.0f };
  float f_gb_instab[3] = { 0.0f };
  float f_ab_instab[3] = { 0.0f };
  float f_misangle_noise[3] = { 0.0f };
  float f_ws_noise, f_clock_err[3] = { 0.0f }; 
  float f_clockdiff_err = 0.0f;
  float f_gyro_coor_t = 1800.f, f_accl_coor_t = 1800.f;
  float f_la_i2g_noise[3] = { 0.0f };
  float f_la_i2v_noise[3] = { 0.0f };
#ifdef WHEELBASE_ERR_ESTI
  float f_wheelbase_noise = 0.0f;
#endif
  float f_misangle_imu2dualant_noise = 0.0f;
    
  memset(gz_navconfig.f_noise, 0, sizeof(gz_navconfig.f_noise));

  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

  /* 1.set Q parameters */
  if (pz_algconfig->z_init_para.u_imu_type == (uint8_t)TYPE_IMU_AG051)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      f_accl_vrw[i] = 0.005f;
      f_gyro_arw[i] = 0.2f;
      f_gb_instab[i] = 20.f;
      f_ab_instab[i] = 5e-4f;
      f_misangle_noise[i] = 0.0002f;
      f_clock_err[i] = 2.5e-3f;
      f_la_i2g_noise[i] = 2e-5f;
      f_la_i2v_noise[i] = 2e-5f;
    }
    f_ws_noise = 1e-5f;
#ifdef WHEELBASE_ERR_ESTI
    f_wheelbase_noise = 5e-6f;
#endif
    f_misangle_imu2dualant_noise = 1e-17f;
    f_clockdiff_err = 0.05f;
  }
  else if (pz_algconfig->z_init_para.u_imu_type == (uint8_t)TYPE_IMU_ASM330)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      f_accl_vrw[i] = 0.005f;
      f_gyro_arw[i] = 0.15f;
      f_gb_instab[i] = 20.0f;
      f_ab_instab[i] = 0.00375f;
      f_misangle_noise[i] = 0.0002f;
      f_clock_err[i] = 2.5e-3f;
      f_la_i2g_noise[i] = 2e-5f;
      f_la_i2v_noise[i] = 2e-5f;
    }
    f_ws_noise = 1e-5f;
#ifdef WHEELBASE_ERR_ESTI
    f_wheelbase_noise = 5e-6f;
#endif
    f_misangle_imu2dualant_noise = 1e-17f;
    f_clockdiff_err = 0.05f;
  }
  /* 2. set P0 parameters */
  if (pz_algconfig->z_init_para.u_imu_type == (uint8_t)TYPE_IMU_AG051)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      gz_navconfig.f_init_posvar[i] = SQR(5.f);
      gz_navconfig.f_init_velvar[i] = SQR(2.f);
      gz_navconfig.f_init_attvar[i] = (float)(SQR(1.0 * DEG2RAD));
      gz_navconfig.f_init_acclvar[i] = SQR(0.003f);
      gz_navconfig.f_init_gyrovar[i] = (float)(SQR(0.03 * DEG2RAD));
      gz_navconfig.f_init_misvar[i] = (float)(SQR(1.0 * DEG2RAD));
      gz_navconfig.f_init_clockvar[i] = SQR(3.0f);
      gz_navconfig.f_init_lai2gvar[i] = SQR(0.2f);
      gz_navconfig.f_init_lai2vvar[i] = SQR(0.1f);
    }
  }
  else if (pz_algconfig->z_init_para.u_imu_type == (uint8_t)TYPE_IMU_ASM330)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      gz_navconfig.f_init_posvar[i] = SQR(5.f);
      gz_navconfig.f_init_velvar[i] = SQR(2.f);
      gz_navconfig.f_init_attvar[i] = (float)(SQR(1.5 * DEG2RAD));
      gz_navconfig.f_init_acclvar[i] = SQR(0.03f);
      gz_navconfig.f_init_gyrovar[i] = (float)(SQR(0.03 * DEG2RAD));
      gz_navconfig.f_init_misvar[i] = (float)(SQR(1.0 * DEG2RAD));
      gz_navconfig.f_init_clockvar[i] = SQR(3.0f);
      gz_navconfig.f_init_lai2gvar[i] = SQR(0.2f);
      gz_navconfig.f_init_lai2vvar[i] = SQR(0.1f);
    }
  }    
  gz_navconfig.f_init_clockdiffvar = SQR(0.1f);
  gz_navconfig.f_init_odosfvar[0] = SQR(0.01f);
  gz_navconfig.f_init_odosfvar[1] = SQR(0.01f);
  gz_navconfig.f_init_wheelbasevar = SQR(0.1f);
	gz_navconfig.f_init_misdualantvar[0] = (float)(SQR(1.0 * DEG2RAD));
	gz_navconfig.f_init_misdualantvar[1] = (float)SQR((5.0 * DEG2RAD)); /* if dualant set with large yaw, should turn up initvar */

  /* 3. set R parameters */
  gz_navconfig.f_zupt_velvar = SQR(0.1f);
  gz_navconfig.f_zupt_yawvar = (float)(SQR(0.01f*DEG2RAD));

  if (pz_algconfig->z_init_para.u_imu_type == (uint8_t)TYPE_IMU_AG051)
  {
    gz_navconfig.f_nhc_yvar = SQR(0.1f);
    gz_navconfig.f_nhc_zvar = SQR(0.1f);
  }
  else if (pz_algconfig->z_init_para.u_imu_type == (uint8_t)TYPE_IMU_ASM330)
  {
    gz_navconfig.f_nhc_yvar = SQR(0.01f);
    gz_navconfig.f_nhc_zvar = SQR(0.01f);
  }
  for (uint8_t i = 0; i < 3; i++)
  {
    gz_navconfig.f_noise[i + 3] = SQR(f_accl_vrw[i]);
    gz_navconfig.f_noise[i + 6] = (float)(SQR(f_gyro_arw[i] * DEG2RAD / 60.f));
    gz_navconfig.f_noise[i + 9] = (float)(2.f * SQR(f_gb_instab[i] * DEG2RAD / 3600.f) / f_gyro_coor_t);
    gz_navconfig.f_noise[i + 12] = 2.f * SQR(f_ab_instab[i]) / f_accl_coor_t;
    gz_navconfig.f_noise[i + 15] = (float)(SQR(f_misangle_noise[i] * DEG2RAD));
    if (pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti > 0)
    {
      gz_navconfig.f_noise[i + pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti] = SQR(f_la_i2g_noise[i]);
    }
#ifdef LA_IMU2REARMID_ERR_ESTI		
    gz_navconfig.f_noise[i + LA_IMU2REARMID_ERR_ESTI] = SQR(f_la_i2v_noise[i]);
#endif
    if (pz_algconfig->z_syserrtype.u_clock_err_esti > 0)
    {
      gz_navconfig.f_noise[i + pz_algconfig->z_syserrtype.u_clock_err_esti] = SQR(f_clock_err[i]);
    }
  }
#ifdef ODO_SF_ESTI
  gz_navconfig.f_noise[ODO_SF_ESTI] = SQR(f_ws_noise);
#endif 
  if (pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti > 0)
  {
    gz_navconfig.f_noise[pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti] = SQR(f_ws_noise);
    gz_navconfig.f_noise[pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti + 1] = SQR(f_ws_noise);
  }
#ifdef WHEELBASE_ERR_ESTI
  gz_navconfig.f_noise[WHEELBASE_ERR_ESTI] = SQR(f_wheelbase_noise);
#endif
  if (pz_algconfig->z_syserrtype.u_mis_dualant_err_esti > 0)
  {
    gz_navconfig.f_noise[pz_algconfig->z_syserrtype.u_mis_dualant_err_esti] = (float)(SQR(f_misangle_imu2dualant_noise * DEG2RAD));
    gz_navconfig.f_noise[pz_algconfig->z_syserrtype.u_mis_dualant_err_esti + 1] = (float)SQR((f_misangle_imu2dualant_noise * DEG2RAD));
  }
  if (pz_algconfig->z_syserrtype.u_clock_diff_err_esti > 0)
  {
    gz_navconfig.f_noise[pz_algconfig->z_syserrtype.u_clock_diff_err_esti] = SQR(f_clockdiff_err);
  }
  gz_navconfig.f_calc_rate = fusion_get_algconfig()->u_calc_rate;

  for (uint8_t i = 0; i < 3; i++)
  {
      gz_navconfig.f_gerr_model[i] = (float)exp(-1.f / 200.f / f_gyro_coor_t);
      gz_navconfig.f_aerr_model[i] = (float)exp(-1.f / 200.f / f_accl_coor_t);
  }
  gz_navconfig.f_lambda_pos1 = 1.f;
  gz_navconfig.f_lambda_pos2 = 20.f;
  gz_navconfig.f_lambda_vel1 = 10;
  gz_navconfig.f_lambda_vel2 = 30;
  gz_navconfig.f_pos_scl_sps = 1.0;
  gz_navconfig.f_vel_scl_sps = 1.0;
  gz_navconfig.f_pos_scl_dif = 1.0;
  gz_navconfig.f_vel_scl_dif = 1.0;
  gz_navconfig.f_pos_scl_flt = 1.0;
  gz_navconfig.f_vel_scl_flt = 1.0;
  gz_navconfig.f_pos_scl_rtk = 1.0;
  gz_navconfig.f_vel_scl_rtk = 1.0;
  gz_navconfig.u_whspd_enable = fusion_get_algconfig()->z_init_para.u_whspd_mode;
}
