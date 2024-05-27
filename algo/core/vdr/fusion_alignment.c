/**@file        fusion_alignment.c
 * @brief		fusion alignment file
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

#include "fusion_alignment.h"
#include "fusion_proc.h"
#include "fusion_transform.h"
#include "fusion_quat.h"
#include "fusion_ellip_para.h"
#include "fusion_config.h"
#include "fusion_global.h"
#include "fusion_log.h"
#include "fusion_math.h"
#include "fusion_err_model.h"
#include "fusion_if.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "mw_log.h"

static InitialAlign_t gz_align;

InitialAlign_t* fusion_get_align()
{
  return &gz_align;
}

void align_initialize_navpara(double d_poslla[3], float f_attrph[3], float f_vel_ned[3])
{
  double d_rm, d_rn;
  float f_gravity_vec[3] = { 0.0f };
  float f_rotmat_tmp[3][3] = { 0.f };
  NavParams_t* pz_nav = fusion_get_navpara();

  memcpy(pz_nav->d_poslla, d_poslla,sizeof(pz_nav->d_poslla));
  memcpy(pz_nav->f_vel_ned, f_vel_ned,sizeof(pz_nav->f_vel_ned));
  memcpy(pz_nav->f_att_rph, f_attrph, sizeof(pz_nav->f_att_rph));

  tf_euler2dcm(f_attrph, pz_nav->f_rotmat_b2n);
  math_matmul(*pz_nav->f_rotmat_b2n,*pz_nav->f_rotmat_misalign,3,3,3,*f_rotmat_tmp);
  memcpy(pz_nav->f_rotmat_b2n, f_rotmat_tmp,sizeof(f_rotmat_tmp));
  tf_dcm2eluer(pz_nav->f_rotmat_b2n, pz_nav->f_att_rph);
  quat_euler2quat(pz_nav->f_att_rph, pz_nav->f_quat_b2n);
  quat_copy_f(pz_nav->f_prev_quat_b2n, pz_nav->f_quat_b2n); 
  quat_get_qn2e(d_poslla[0], d_poslla[1], pz_nav->d_quat_n2e);

  ellip_calculateMN(d_poslla[0], &d_rm, &d_rn);
  pz_nav->d_rm = d_rm;
  pz_nav->d_rn = d_rn;

  f_gravity_vec[0] = 0.f;
  f_gravity_vec[1] = 0.f;
  f_gravity_vec[2] = ellip_get_gravity(d_poslla);
  memcpy(pz_nav->f_gravity_vct, f_gravity_vec, sizeof(pz_nav->f_gravity_vct));
  tf_get_wien(d_poslla, pz_nav->f_omega_ie);
  pz_nav->u_inited = INS_TRUE;
}

void align_calculate_att_sigant(float f_attrph[3], float f_vel_ned[3], float f_velvar[3], float f_attvar[3])
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();

  f_attrph[0] = 0.f;			                
  f_attrph[1] = 0.f;			                
  f_attrph[2] = atan2f(f_vel_ned[1], f_vel_ned[0]);	

  if (f_attrph[2] < 0.f)
  {
    f_attrph[2] += (float)(2.0 * INS_PI);
  }

  f_attvar[0] = pz_navconfig->f_init_attvar[0];
  f_attvar[1] = pz_navconfig->f_init_attvar[1];
  f_attvar[2] = (f_velvar[0] + f_velvar[1]) / (f_vel_ned[0] * f_vel_ned[0] + f_vel_ned[1] * f_vel_ned[1]);

  float f_th1 = (float)(1.f * DEG2RAD * 1.f * DEG2RAD);
  if (f_attvar[2] < f_th1)
  {
    f_attvar[2] = f_th1;
  }
}

void align_calculate_att_dualant(float f_attrph[3], float f_vel_ned[3], float f_velvar[3], float f_attvar[3])
{
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  NavConfig_t* pz_navconfig = fusion_get_navconfig();

  float* f_att_mis_b2g = mis_get_imu2dualant();

  f_attrph[0] = 0.f;			                
  f_attrph[1] = (float)(pz_gnss->f_pitch_deg * DEG2RAD);			                
  f_attrph[2] = (float)((pz_gnss->f_heading_deg) * DEG2RAD + f_att_mis_b2g[2]);	

  if (f_attrph[2] < 0.f)
  {
    f_attrph[2] += (float)(2.0 * INS_PI);
  }

  f_attvar[0] = pz_navconfig->f_init_attvar[0];
  f_attvar[1] = (float)(SQR(pz_gnss->f_pitch_std * DEG2RAD));
  f_attvar[2] = (float)(SQR(pz_gnss->f_heading_std * DEG2RAD));
}

void align_deploy_pva(double pd_meas[], InitialAlign_t* pz_align, uint8_t u_flag)
{
  double d_poslla[3];
  float f_posvar[3];
  float f_vel_ned[3];
  float f_velvar[3];
  float f_attrph[3];
  float f_attvar[3] = {0.0};
  int8_t i;
  float dt = 0.0f;
  NavParams_t* pz_nav = fusion_get_navpara();
  NavConfig_t* pz_navconfig = fusion_get_navconfig();
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  InertialNav_t* pz_inv = fusion_get_inertial_nav();

  pz_nav->d_sys_timestamp = (pz_align->d_imu_timestamp > 0.0) ? pz_align->d_imu_timestamp : pd_meas[1];
	
  /* turn up noise, avoid HMI */
  uint8_t u_bds_status = *((uint8_t*)(&pd_meas[14]));
  if (u_bds_status != TYPE_POS_FLAG_RTK_FIX)
  {
    pd_meas[8] = (pd_meas[8] > 2.0) ? pd_meas[8] : 2.0;
    pd_meas[9] = (pd_meas[9] > 2.0) ? pd_meas[9] : 2.0;
    pd_meas[10] = (pd_meas[10] > 2.0) ? pd_meas[10] : 2.0;
    LOGI(TAG_VDR, "force turn up pos noise to 3m while pass alignmnet at nonfix epoch\n");
  }

  for (i = 0; i < 3; i++)
  {
    d_poslla[i] = pd_meas[i + 2];	          /* lat lon height */
    f_vel_ned[i] = (float)(pd_meas[i + 5]);	  /* vel: N E D */
    f_posvar[i] = (float)(pd_meas[i + 8]);    /* pos variance */
    f_velvar[i] = (float)(pd_meas[i + 11]);   /* vel variance */
  }
    
  dt = (float)(pz_nav->d_sys_timestamp - pz_gnss->d_tow/1000.0);
    
  if (pz_navconfig->u_i2g_opt == 1)
  {
    float* pf_imu2gnss = lvrm_get_imu2gnss();
    compensate_pos_lever_arm(d_poslla, pf_imu2gnss, BLH_XPOS2IMU);
  }

  tf_extend_pos_byvel(d_poslla, f_vel_ned, dt);

  if (dt > 0.0f)
  {
    for (i = 0; i < 3; i++)
    {
      f_posvar[i] += f_velvar[i] * dt * dt;
      if (i == 2)
      {
        if (sqrtf(f_posvar[i]) < 0.2f)
        {
          f_posvar[i] = 0.2f * 0.2f;
        }
      }
      else
      {
        if (sqrtf(f_posvar[i]) < 0.1f)
        {
          f_posvar[i] = 0.1f * 0.1f;
        }
      }
    }
  }
  if (u_flag == 1)
  {
    align_calculate_att_sigant(f_attrph, f_vel_ned, f_velvar, f_attvar);
    LOGI(TAG_VDR, "Passed in motion alignment, gpstime:%.3f,imutime:%.3f, gnssflag:%d\n", pd_meas[1], pz_nav->d_sys_timestamp, *((uint8_t*)(&pd_meas[14])));
  }
  else if (u_flag == 2)
  {
    align_calculate_att_dualant(f_attrph, f_vel_ned, f_velvar, f_attvar);
    LOGI(TAG_VDR, "Passed in dualant alignment, gpstime:%.3f,imutime:%.3f, gnssflag:%d\n", pd_meas[1], pz_nav->d_sys_timestamp, *((uint8_t*)(&pd_meas[14])));
  }	
  align_initialize_navpara(d_poslla, f_attrph, f_vel_ned);
#ifndef FORCE_INIT
  memcpy(pz_navconfig->f_init_posvar, f_posvar, sizeof(pz_navconfig->f_init_posvar));
  memcpy(pz_navconfig->f_init_attvar, f_attvar, sizeof(pz_navconfig->f_init_attvar));
#endif

  kf_errmodel_init();
  pz_inv->u_init_nav = INS_FALSE;
  pz_inv->u_ins_status = INS_STATUS_NAVIGATION;
#if 0
  if (fabsf(pz_align->f_odospeed) < 1e-6)
  {
    pz_navconfig->u_odo_enable = 0;
  }
  else
  {
    float f_odosf = 1.f - sqrtf((float)(pd_meas[5] * pd_meas[5] + pd_meas[6] * pd_meas[6])) / pz_align->f_odospeed;
    pz_navconfig->f_odospd_sf = (fabsf(f_odosf) < 0.25f) ? f_odosf : pz_navconfig->f_odospd_sf;/*consider odo abnormal*/
  }
#endif
}


#if STATIC_NAVIGATION
/*
* @brief: INS static alignment
* @input: IMU array or GNSS array
* @output: None
* @return: None
*/
void fusion_static_align(double pd_meas[])
{
  InertialNav_t* pz_inv = fusion_get_inertial_nav();
  LOGI(TAG_VDR, "%s++, time:%.3lf, lat(deg): %.10lf, alt(m):.%3lf, latVar : %.3lf\n", __FUNCTION__, pd_meas[1], pd_meas[2] * RAD2DEG, pd_meas[4], pd_meas[8]);

  if (is_izupt() == 1)
  {
    double d_poslla[3] = { 0.0 };
    float f_vel_ned[3] = { 0.0f };
    float f_posvar[3] = { 0.0f };
    float f_velvar[3] = { 0.0f };
    uint8_t i;

    NavParams_t* pz_nav = fusion_get_navpara();
    NavConfig_t* pz_navconfig = fusion_get_navconfig();

    for (i = 0; i < 3; i++)
    {
      d_poslla[i] = pd_meas[i + 2];
      f_vel_ned[i] = 0.0f;
      f_posvar[i] = (float)pd_meas[i + 8];
      f_velvar[i] = (float)pd_meas[i + 11];
      pz_nav->f_att_rph[i] = 0.0f;
    }
    LOGI(TAG_VDR, "d_poslla(deg):%.3lf,%.3lf,f_posvar:%.3lf,%.3lf,\n",
        d_poslla[0] * RAD2DEG, d_poslla[1] * RAD2DEG, f_posvar[0], f_posvar[1]);

    align_initialize_navpara(d_poslla, pz_nav->f_att_rph, f_vel_ned);

    memcpy(pz_nav->d_poslla, d_poslla, sizeof(pz_nav->d_poslla));
    memcpy(pz_nav->f_vel_ned, f_vel_ned, sizeof(pz_nav->f_vel_ned));

    memcpy(pz_navconfig->f_init_posvar, f_posvar, sizeof(pz_navconfig->f_init_posvar));
    memcpy(pz_navconfig->f_init_velvar, f_velvar, sizeof(pz_navconfig->f_init_velvar));

    pz_nav->d_sys_timestamp = pd_meas[1];

    LOGI(TAG_VDR, "AlignStatic Pos_LLA(m): %.3lf, %.3lf, %.3lf,%.3lf\r\n",
        pz_nav->d_poslla[0], pz_nav->d_poslla[1], pz_nav->d_poslla[2], pz_navconfig->f_init_posvar[0]);

    kf_errmodel_init();

    pz_inv->u_init_nav = INS_FALSE;
    pz_inv->u_ins_status = INS_STATUS_NAVIGATION;

    LOGI(TAG_VDR, "Passed in Static alignment: %f\n", pd_meas[1]);
  }
}
#endif

/*
* @brief: alignment of ins
* @input: IMU data or GNSS data
* @output: None
* @return: 0
*/
void align_procedure(double pd_meas[])
{
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  if (pz_inav->u_align_method == 1)
  {
    static uint8_t u_first_gnss_in = 0;
    if (u_first_gnss_in == 0 && pd_meas[0] < 0.0)
    {
      u_first_gnss_in = 1;
    }

    if (u_first_gnss_in == 1)
    {
      AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
      NavConfig_t* pz_navconfig = fusion_get_navconfig();
      if ((pz_gnss->u_msg_type & NOVATEL_RECMGS_HEADING) == NOVATEL_RECMGS_HEADING &&
	      pz_navconfig->u_misb2g_opt == 1)
      {
        align_with_dualant(pd_meas, 2);
      }
      else
      {
        align_with_sigant(pd_meas, 1);
      }
    }
  }
  else if (pz_inav->u_align_method == 2)
  {
    HistoryInsResults_t* pz_history = fusion_get_historyInfo();
    align_with_history(pz_history, pd_meas);
  }
  else
  {
    /* */
  }
}

/*
* @brief: in-motion alignment with single antenna
* @input: IMU data or GNSS data
* @output: None
* @return: 0
*/
uint8_t align_with_sigant(double pd_meas[], uint8_t u_flag)
{
  float	f_avgspeed = 0.0f;
  InitialAlign_t* pz_align = fusion_get_align();

  if (pz_align->u_align_status == INS_FALSE)
  {
    pz_align->u_align_status = INS_TRUE;
    pz_align->f_avg_speed = 0.f;
    pz_align->f_odospeed = 0.f;
    pz_align->d_prev_bdstime = INITIAL_TIME;
  }

  if (pd_meas[0] > 0.0)
  {
    pz_align->f_odospeed = (float)pd_meas[8];
  }

  f_avgspeed = pz_align->f_avg_speed;

  /* align with n-frame based on GNSS pos and vel */
  if (pd_meas[0] < 0.0)
  {
    uint8_t u_bds_status;
    u_bds_status = *((uint8_t*)(&pd_meas[14]));
    LOGI(TAG_VDR, "Enter motion bds, %.3lf,%.3lf,%.3lf,%.3lf\n", pd_meas[1], pd_meas[2], pd_meas[3], pd_meas[4]);

#if STATIC_NAVIGATION 
    fusion_static_align(pd_meas);
#else
    NavConfig_t* pz_navconfig = fusion_get_navconfig();

    if ((u_bds_status != 0U) && (pz_align->d_prev_bdstime >= 0.0) &&
      (pd_meas[1] > pz_align->d_prev_bdstime) && (pd_meas[1] - pz_align->d_prev_bdstime < CONTINUE_BDS_DT))
    {
      float f_vtmp = (float)(pd_meas[5] * pd_meas[5] + pd_meas[6] * pd_meas[6]);
      float f_alpha = 0.2f;

      if (sqrt(pd_meas[8] + pd_meas[9]) < 1.0 && sqrt(pd_meas[11] + pd_meas[12]) < 0.5)
      {
        f_avgspeed = f_alpha * f_vtmp + (1.f - f_alpha) * f_avgspeed;
      }
      else
      {
        f_avgspeed = 0.f;
      }
      pz_align->f_avg_speed = f_avgspeed;
      LOGI(TAG_VDR, "In Motion Alignment, gpstime:%.3lf,imutime:%.3lf, Vel:%f.\n", pd_meas[1], pz_align->d_imu_timestamp, f_avgspeed);

      double gps_imu_timediff = (pz_align->d_imu_timestamp > 0.0) ? fabs(pd_meas[1] - pz_align->d_imu_timestamp) : 0.0;

      /* GNSS vel > 7m/s, for more reliable heading */
      if ((f_avgspeed > INS_ALIGN_VEL) && (f_vtmp > INS_ALIGN_VEL) && fabs(pd_meas[7]) < 1.0 &&
        u_bds_status >= 4 && gps_imu_timediff < fusion_get_algconfig()->f_gnss_dt * 3.0)
      {
#ifndef FORCE_INIT
        align_deploy_pva(pd_meas,pz_align,u_flag);
#endif
      }
    }
#ifdef FORCE_INIT
    align_deploy_pva(pd_meas, pz_align, u_flag);
#endif
#endif
    pz_align->d_prev_bdstime = pd_meas[1];
  }
  else
  {
    pz_align->d_imu_timestamp = pd_meas[1];
  }
  return 0;
}

uint8_t align_with_dualant(double pd_meas[], uint8_t u_flag)
{
  InitialAlign_t* pz_align = fusion_get_align();

  if (pz_align->u_align_status == INS_FALSE)
  {
    pz_align->u_align_status = INS_TRUE;
    pz_align->d_prev_bdstime = INITIAL_TIME;
  }	

  if (pd_meas[0] < 0.0)		
  {
    uint8_t u_bds_status;
    u_bds_status = *((uint8_t*)(&pd_meas[14]));

    NavConfig_t* pz_navconfig = fusion_get_navconfig();
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();

    if ((u_bds_status == TYPE_POS_FLAG_RTK_FIX) && (pz_align->d_prev_bdstime >= 0.0) &&
      (pd_meas[1] > pz_align->d_prev_bdstime) && (pd_meas[1] - pz_align->d_prev_bdstime < CONTINUE_BDS_DT))
    {
      if (pz_gnss->f_heading_std < 1.0 && pz_gnss->f_heading_std > 0.0 && pz_gnss->f_pitch_std < 1.0 &&
        pz_gnss->f_pitch_std > 0.0 && fabsf(pz_gnss->f_pitch) < 20.0 && pz_gnss->u_satused >= 5)
      {
#ifndef FORCE_INIT
        LOGI(TAG_VDR, "Enter DualAnt Alignment, %f \n", pd_meas[1]);
        align_deploy_pva(pd_meas, pz_align, u_flag);
#endif
      }
    }
#ifdef FORCE_INIT
    align_deploy_pva(pd_meas, pz_align, u_flag);
    LOGI(TAG_VDR, "Enter DualAnt Alignment, %f \n", pd_meas[1]);
#endif
    pz_align->d_prev_bdstime = pd_meas[1];
  }
  else
  {
    pz_align->d_imu_timestamp = pd_meas[1];
  }
  return 0;
}

void align_with_history(HistoryInsResults_t* pz_history, double* pd_meas)
{
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  NavParams_t* pz_nav = fusion_get_navpara();
  NavConfig_t* pz_navconfig = fusion_get_navconfig();
  double d_poslla[3];
  float f_attrph[3];
  double d_rm, d_rn;
  float f_gravity_vec[3] = { 0.0f };
  float f_rotmat_tmp[3][3] = { 0.f };

  d_poslla[0] = pz_history->d_lat;
  d_poslla[1] = pz_history->d_lon;
  d_poslla[2] = pz_history->f_alt;
  f_attrph[0] = (float)(pz_history->f_roll * DEG2RAD);
  f_attrph[1] = (float)(pz_history->f_pitch * DEG2RAD);
  f_attrph[2] = (float)(pz_history->f_yaw * DEG2RAD);
  pz_nav->d_sys_timestamp = pd_meas[1];
  memcpy(pz_nav->d_poslla, d_poslla, sizeof(pz_nav->d_poslla));
  pz_nav->f_vel_ned[0] = 0;
  pz_nav->f_vel_ned[1] = 0;
  pz_nav->f_vel_ned[2] = 0;
  memcpy(pz_nav->f_att_rph, f_attrph, sizeof(pz_nav->f_att_rph));
  tf_euler2dcm(f_attrph, pz_nav->f_rotmat_b2n);
  quat_euler2quat(pz_nav->f_att_rph, pz_nav->f_quat_b2n);
  quat_copy_f(pz_nav->f_prev_quat_b2n, pz_nav->f_quat_b2n);
  quat_get_qn2e(d_poslla[0], d_poslla[1], pz_nav->d_quat_n2e);
  ellip_calculateMN(d_poslla[0], &d_rm, &d_rn);
  pz_nav->d_rm = d_rm;
  pz_nav->d_rn = d_rn;
  f_gravity_vec[0] = 0.f;
  f_gravity_vec[1] = 0.f;
  f_gravity_vec[2] = ellip_get_gravity(d_poslla);
  memcpy(pz_nav->f_gravity_vct, f_gravity_vec, sizeof(pz_nav->f_gravity_vct));
  tf_get_wien(d_poslla, pz_nav->f_omega_ie);
  pz_nav->u_inited = INS_TRUE;
  kf_errmodel_init();
  pz_inav->u_init_nav = INS_FALSE;
  pz_inav->u_ins_status = INS_STATUS_NAVIGATION;
}
