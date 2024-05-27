/**@file        fusion_wheel.c
 * @brief		wheel signal file
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

#include "fusion_wheel.h"
#include "fusion_math.h"
#include "fusion_global.h"
#include "fusion_err_model.h"
#include "fusion_quat.h"
#include "fusion_proc.h"
#include "fusion_log.h"
#include "fusion_if.h"
#include "fusion_kf.h"
#include <string.h>
#include "mw_log.h"
#include "mw_alloc.h"

#define KWS_INIT 0.00863

static WheelSpd_t gz_whspd;

WheelSpd_t* fusion_get_wheelspd(void)
{
  return &gz_whspd;
}

uint8_t whspd_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_TRUE;
  float f_la_imu2rearleft[3], f_la_imu2rearright[3], f_vel_v_imu[3], f_winn[3], f_winb[3], f_wnbb[3], f_wnbv[3];
  float f_w_larearleft_crosstemp[3], f_w_larearright_crosstemp[3], f_v_v_rearleft[3], f_v_v_rearright[3];
  float f_rotmat_b2n[3][3], f_rotmat_n2b[3][3], f_rotmat_n2v[3][3], f_vn_x[3][3], f_n2v_v_temp[3][3];
  float f_la_imu2rearleft_x[3][3], f_la_imu2rearright_x[3][3], f_b2v_larearleft_temp[3][3], f_b2v_larearright_temp[3][3];
  float f_v_v_rearleft_x[3][3], f_v_v_rearright_x[3][3];
  float f_wnbv_x[3][3], f_temp_rl[3][3], f_temp_rr[3][3], f_v_v_x[3][3];

  MechanNode_t* pz_node = pz_inav->pz_obs->pz_veh_node;

  SysErrModel_t* pz_model = pz_inav->pz_model;
  NavParams_t* pz_nav = fusion_get_navpara();
  NavConfig_t* pNavConfig = pz_inav->pz_config;
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

  *pu_mdim = 6;
  uint8_t u_mdim = pz_model->u_meadim;
  uint8_t u_sdim = pz_model->u_sysdim;

  float* f_sysmeasmat = (float*)OS_MALLOC(6 * u_sdim * sizeof(float)); 
  float* f_measnoise = (float*)OS_MALLOC(6 * u_mdim * sizeof(float));
    
  memset(pz_model->f_measnoise, 0, u_mdim * u_mdim * sizeof(float));
  memset(pz_model->f_sysmeasmat, 0, u_mdim * u_sdim * sizeof(float));

  float* pf_la_imu2rearmid = lvrm_get_imu2rearmid();
  float* rear2rear = lvrm_get_rear2rear();
  WheelSpd_t* pz_whspd = fusion_get_wheelspd();

  f_la_imu2rearleft[0] = pf_la_imu2rearmid[0];
  f_la_imu2rearleft[1] = pf_la_imu2rearmid[1] - *rear2rear/2.0f;
  f_la_imu2rearleft[2] = pf_la_imu2rearmid[2];
  f_la_imu2rearright[0] = pf_la_imu2rearmid[0];
  f_la_imu2rearright[1] = pf_la_imu2rearmid[1] + *rear2rear / 2.0f;
  f_la_imu2rearright[2] = pf_la_imu2rearmid[2];

  quat_quat2dcm(pz_node->f_qbn, f_rotmat_b2n);
  math_mattrns(&(f_rotmat_b2n[0][0]), 3, 3, &(f_rotmat_n2b[0][0]));
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), &(f_rotmat_n2b[0][0]), 3, 3, 3, &(f_rotmat_n2v[0][0]));
  math_matmul(&(f_rotmat_n2v[0][0]), pz_node->f_vel, 3, 3, 1, f_vel_v_imu);

  math_matadd(pz_node->f_wien, pz_node->f_wnen, 3, 1, f_winn);
  math_matmul(&(f_rotmat_n2b[0][0]), f_winn, 3, 3, 1, f_winb);
  math_matsub(pz_node->f_wibb, f_winb, 3, 1, f_wnbb);
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), f_wnbb, 3, 3, 1, f_wnbv);
  math_cross_product(f_wnbv, f_la_imu2rearleft, f_w_larearleft_crosstemp);
  math_cross_product(f_wnbv, f_la_imu2rearright, f_w_larearright_crosstemp);

  math_matadd(f_vel_v_imu, f_w_larearleft_crosstemp, 3, 1, f_v_v_rearleft);
  math_matadd(f_vel_v_imu, f_w_larearright_crosstemp, 3, 1, f_v_v_rearright);

  f_z_meas[0] = f_v_v_rearleft[0] - pz_whspd->f_vehspd_rl * (1.f - pz_nav->f_wspd_sf[2]);
  f_z_meas[1] = f_v_v_rearleft[1] - 0.0f;
  f_z_meas[2] = f_v_v_rearleft[2] - 0.0f;
  f_z_meas[3] = f_v_v_rearright[0] - pz_whspd->f_vehspd_rr * (1.f - pz_nav->f_wspd_sf[3]);
  f_z_meas[4] = f_v_v_rearright[1] - 0.0f;
  f_z_meas[5] = f_v_v_rearright[2] - 0.0f;

  math_skewsym(pz_node->f_vel, f_vn_x);
  math_matmul(&(f_rotmat_n2v[0][0]), &(f_vn_x[0][0]), 3, 3, 3, &(f_n2v_v_temp[0][0]));

#if 0
  pz_model->f_sysmeasmat[0][VEL_ERR_ESTI] = f_rotmat_n2v[0][0];
  pz_model->f_sysmeasmat[0][VEL_ERR_ESTI + 1] = f_rotmat_n2v[0][1];
  pz_model->f_sysmeasmat[0][VEL_ERR_ESTI + 2] = f_rotmat_n2v[0][2];
  pz_model->f_sysmeasmat[1][VEL_ERR_ESTI] = f_rotmat_n2v[1][0];
  pz_model->f_sysmeasmat[1][VEL_ERR_ESTI + 1] = f_rotmat_n2v[1][1];
  pz_model->f_sysmeasmat[1][VEL_ERR_ESTI + 2] = f_rotmat_n2v[1][2];
  pz_model->f_sysmeasmat[2][VEL_ERR_ESTI] = f_rotmat_n2v[2][0];
  pz_model->f_sysmeasmat[2][VEL_ERR_ESTI + 1] = f_rotmat_n2v[2][1];
  pz_model->f_sysmeasmat[2][VEL_ERR_ESTI + 2] = f_rotmat_n2v[2][2];
    
  pz_model->f_sysmeasmat[3][VEL_ERR_ESTI] = f_rotmat_n2v[0][0];
  pz_model->f_sysmeasmat[3][VEL_ERR_ESTI + 1] = f_rotmat_n2v[0][1];
  pz_model->f_sysmeasmat[3][VEL_ERR_ESTI + 2] = f_rotmat_n2v[0][2];
  pz_model->f_sysmeasmat[4][VEL_ERR_ESTI] = f_rotmat_n2v[1][0];
  pz_model->f_sysmeasmat[4][VEL_ERR_ESTI + 1] = f_rotmat_n2v[1][1];
  pz_model->f_sysmeasmat[4][VEL_ERR_ESTI + 2] = f_rotmat_n2v[1][2];
  pz_model->f_sysmeasmat[5][VEL_ERR_ESTI] = f_rotmat_n2v[2][0];
  pz_model->f_sysmeasmat[5][VEL_ERR_ESTI + 1] = f_rotmat_n2v[2][1];
  pz_model->f_sysmeasmat[5][VEL_ERR_ESTI + 2] = f_rotmat_n2v[2][2];

  pz_model->f_sysmeasmat[0][ATT_ERR_ESTI] = -f_n2v_v_temp[0][0];
  pz_model->f_sysmeasmat[0][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[0][1];
  pz_model->f_sysmeasmat[0][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[0][2];
  pz_model->f_sysmeasmat[1][ATT_ERR_ESTI] = -f_n2v_v_temp[1][0];
  pz_model->f_sysmeasmat[1][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[1][1];
  pz_model->f_sysmeasmat[1][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[1][2];
  pz_model->f_sysmeasmat[2][ATT_ERR_ESTI] = -f_n2v_v_temp[2][0];
  pz_model->f_sysmeasmat[2][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[2][1];
  pz_model->f_sysmeasmat[2][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[2][2];

  pz_model->f_sysmeasmat[3][ATT_ERR_ESTI] = -f_n2v_v_temp[0][0];
  pz_model->f_sysmeasmat[3][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[0][1];
  pz_model->f_sysmeasmat[3][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[0][2];
  pz_model->f_sysmeasmat[4][ATT_ERR_ESTI] = -f_n2v_v_temp[1][0];
  pz_model->f_sysmeasmat[4][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[1][1];
  pz_model->f_sysmeasmat[4][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[1][2];
  pz_model->f_sysmeasmat[5][ATT_ERR_ESTI] = -f_n2v_v_temp[2][0];
  pz_model->f_sysmeasmat[5][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[2][1];
  pz_model->f_sysmeasmat[5][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[2][2];
#endif
  int icount, jcount;
#if 0
  for (icount = 0; icount < 3; icount++)
  {
    for (jcount = 0; jcount < 3; jcount++)
    {
      f_sysmeasmat[icount][VEL_ERR_ESTI + jcount] = f_rotmat_n2v[icount][jcount];
      f_sysmeasmat[icount][ATT_ERR_ESTI + jcount] = -f_n2v_v_temp[icount][jcount];
      f_sysmeasmat[icount + 3][VEL_ERR_ESTI + jcount] = f_rotmat_n2v[icount][jcount];		
      f_sysmeasmat[icount + 3][ATT_ERR_ESTI + jcount] = -f_n2v_v_temp[icount][jcount];
    }
  }
#endif
    
  math_skewsym(f_la_imu2rearleft, f_la_imu2rearleft_x);
  math_matmul(&(f_la_imu2rearleft_x[0][0]), &(pz_nav->f_rotmat_misalign[0][0]), 3, 3, 3, &(f_b2v_larearleft_temp[0][0]));
  math_skewsym(f_la_imu2rearright, f_la_imu2rearright_x);
  math_matmul(&(f_la_imu2rearright_x[0][0]), &(pz_nav->f_rotmat_misalign[0][0]), 3, 3, 3, &(f_b2v_larearright_temp[0][0]));

#if 0
  for (icount = 0; icount < 3; icount++)
  {
    for (jcount = 0; jcount < 3; jcount++)
    {
      f_sysmeasmat[icount][GBIAS_ERR_ESTI + jcount] = -f_b2v_larearleft_temp[icount][jcount];
      f_sysmeasmat[icount + 3][GBIAS_ERR_ESTI + jcount] = -f_b2v_larearright_temp[icount][jcount];
    }
  }
#endif

  math_skewsym(f_wnbv, f_wnbv_x);
  math_matmul(*f_la_imu2rearleft_x, *f_wnbv_x, 3, 3, 3, *f_temp_rl);
  math_matmul(*f_la_imu2rearright_x, *f_wnbv_x, 3, 3, 3, *f_temp_rr);
  math_skewsym(f_vel_v_imu, f_v_v_x);
  math_matsub(*f_v_v_x, *f_temp_rl, 3, 3, *f_v_v_rearleft_x);
  math_matsub(*f_v_v_x, *f_temp_rr, 3, 3, *f_v_v_rearright_x);

  for (icount = 0; icount < 3; icount++)
  {
    for (jcount = 0; jcount < 3; jcount++)
    {
      f_sysmeasmat[icount * u_sdim + 3 + jcount] = f_rotmat_n2v[icount][jcount];			
      f_sysmeasmat[(icount + 3) * u_sdim + 3 + jcount] = f_rotmat_n2v[icount][jcount];

      f_sysmeasmat[icount * u_sdim + 6 + jcount] = -f_n2v_v_temp[icount][jcount];
      f_sysmeasmat[(icount + 3) * u_sdim + 6 + jcount] = -f_n2v_v_temp[icount][jcount];

      f_sysmeasmat[icount * u_sdim + 9 + jcount] = -f_b2v_larearleft_temp[icount][jcount];
      f_sysmeasmat[(icount + 3) * u_sdim + 9 + jcount] = -f_b2v_larearright_temp[icount][jcount];

      f_sysmeasmat[icount * u_sdim + 15 + jcount] = f_v_v_rearleft_x[icount][jcount];
      f_sysmeasmat[(icount + 3) * u_sdim + 15 + jcount] = f_v_v_rearright_x[icount][jcount];

#ifdef LA_IMU2REARMID_ERR_ESTI
      f_sysmeasmat[icount * u_sdim + LA_IMU2REARMID_ERR_ESTI + jcount] = f_wnbv_x[icount][jcount];
      f_sysmeasmat[(icount + 3) * u_sdim + LA_IMU2REARMID_ERR_ESTI + jcount] = f_wnbv_x[icount][jcount];
#endif
    }
#ifdef WHEELBASE_ERR_ESTI	
    f_sysmeasmat[icount * u_sdim + WHEELBASE_ERR_ESTI] = -0.5 * f_wnbv_x[icount][1];
    f_sysmeasmat[(icount + 3) * u_sdim + WHEELBASE_ERR_ESTI] = 0.5 * f_wnbv_x[icount][1];
#endif
  }

  if (pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti > 0)
  {
    f_sysmeasmat[0 * u_sdim + pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti] = -pz_whspd->f_vehspd_rl * (1.f - pz_nav->f_wspd_sf[2]);
    f_sysmeasmat[3 * u_sdim + pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti + 1] = -pz_whspd->f_vehspd_rr * (1.f - pz_nav->f_wspd_sf[3]);
  }

  f_measnoise[0] = 0.01f;
  f_measnoise[1 * u_mdim + 1] = pNavConfig->f_nhc_yvar;
  f_measnoise[2 * u_mdim + 2] = pNavConfig->f_nhc_zvar;

  f_measnoise[3 * u_mdim + 3] = 0.01f;
  f_measnoise[4 * u_mdim + 4] = pNavConfig->f_nhc_yvar;
  f_measnoise[5 * u_mdim + 5] = pNavConfig->f_nhc_zvar;

  float f_z_temp[3] = { 0.f };
  float wivv[3] = { 0.f };
  float f_ratio = 1.f;
  float f_lambda;
  math_matmul(*pz_nav->f_rotmat_misalign, pz_node->f_wibb, 3, 3, 1, wivv);

  float f_gs = (float)(sqrtf(SQR(wivv[0]) + SQR(wivv[1]) + SQR(wivv[2])) * RAD2DEG);
  for (uint8_t i = 0; i < 2; i++)
  {
    memcpy(pz_model->f_sysmeasmat, &f_sysmeasmat[i * 3*u_sdim], 3 * u_sdim * sizeof(float));
    for (uint8_t j = 0; j < 3; j++)
    {
      pz_model->f_measnoise[j* u_mdim +j] = f_measnoise[(i * 3 + j) * u_mdim + i * 3 + j];
    }
    memcpy(f_z_temp, &f_z_meas[i * 3], sizeof(f_z_temp));
    f_lambda = kf_calcu_lambda(f_z_temp, u_sdim, 3, pz_model);

    GNSSQCPara_t* pz_qc = fusion_get_qc_para();
    if (pz_nav->u_imu_shortloss_flag == 1u && pd_meas[1] - pz_nav->d_imu_shortloss_time < 3.0 + pz_qc->d_bds_zupt_time_all)
    {
      LOGI(TAG_VDR, "[wheelspeed_update]:%.3lf, not check lamda for imu lost\n", pz_node->d_time);
    }
    else
    {
      if (pd_meas[1] - pz_inav->pz_obs->d_prebds_update_time < 30.0)
      {
        if (f_lambda > 100.f)
        {
          f_ratio = 100.f;
        }
        else if (f_lambda > 30.f && f_lambda < 100.f)
        {
          f_ratio = f_lambda;
        }
        else
        {
          f_ratio = 1.0f;
        }
      }
      else
      {
        if (f_lambda > 200.f)
        {
          f_ratio = 100.f;
        }
      }
    }   

    if (f_gs > 10.f)
    {
      f_ratio = 50.0f;
    }

    for (uint8_t j = 0; j < 3; j++)
    {
      f_measnoise[(i * 3 + j) * u_mdim + i * 3 + j] *= f_ratio;
    }
      LOGI(TAG_VDR, "[wheelspeed_update]:%.3lf, %.3f, %.3f, %.3f,Lambda:%.3f,Ratio:%.3f,Gs:%f,Feedback:%d\n", pz_node->d_time,
        f_z_temp[0], f_z_temp[1], f_z_temp[2], f_lambda, f_ratio, f_gs, u_val);
  }
  memcpy(pz_model->f_sysmeasmat, &f_sysmeasmat[0], 6 * u_sdim * sizeof(float));
  memcpy(pz_model->f_measnoise, &f_measnoise[0], 6 * u_mdim * sizeof(float));

  OS_FREE(f_sysmeasmat); 
  OS_FREE(f_measnoise);

  return u_val;
}

uint8_t odo_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_TRUE;

  return u_val;
}

uint8_t whspd_steer_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_TRUE;

  return u_val;
}

void whpls_convert_to_whspd(WheelSpd_t* pz_whspd, AsensingWhpulse_t* pz_whpls)
{
  pz_whspd->d_timestamp = (double)pz_whpls->t_timestamp / 1000.0;

  pz_whspd->shifter = pz_whpls->e_gear;

  if (pz_whspd->shifter== TYPE_GEAR_R)
  {
    pz_whspd->u_dir = -1;
  }
  else if (pz_whspd->shifter == TYPE_GEAR_P)
  {
    pz_whspd->u_dir = 0;
  }
  else if ((pz_whspd->shifter == TYPE_GEAR_S)||(pz_whspd->shifter == TYPE_GEAR_W)||
      (pz_whspd->shifter == TYPE_GEAR_M)||(pz_whspd->shifter == TYPE_GEAR_D))
  {
    pz_whspd->u_dir = 1;
  }

  pz_whspd->f_vehspd_fl = (float)(pz_whpls->q_fl_whpulse * KWS_INIT * pz_whspd->u_dir);
  pz_whspd->f_vehspd_fr = (float)(pz_whpls->q_fr_whpulse * KWS_INIT * pz_whspd->u_dir);
  pz_whspd->f_vehspd_rl = (float)(pz_whpls->q_rl_whpulse * KWS_INIT * pz_whspd->u_dir);
  pz_whspd->f_vehspd_rr = (float)(pz_whpls->q_rr_whpulse * KWS_INIT * pz_whspd->u_dir);
}