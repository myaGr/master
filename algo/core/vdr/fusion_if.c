/**@file        fusion_if.c
 * @brief		internal interface file
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

#include "fusion_global.h"
#include "fusion_if.h"
#include "fusion_gnss.h"
#include "fusion_wheel.h"
#include "fusion_proc.h"
#include "fusion_fault_detection.h"
#include "fusion_config.h"
#include "fusion_transform.h"
#include "fusion_math.h"
#include "fusion_log.h"
#include "fusion_ellip_para.h"
#include "fusion_zupt.h"
#include "fusion_alignment.h"
#include "fusion_api.h"
#include "fusion_integrity.h"
#include <string.h>
#include <math.h>
#include "mw_log.h"

static IMUInputBuf_t gz_imu_buf;
static INSResults_t gz_ins_results;
static WHLInputBuf_t gz_whpls_buf;
double gd_last_gnss_time = INITIAL_TIME;

void fusion_if_initial(void)
{
  memset(&gz_imu_buf,0,sizeof(gz_imu_buf));
  memset(&gz_ins_results, 0, sizeof(gz_ins_results));
  gnss_form_meas_init();
  navconfig_initial();
  fusion_navsys_init();
  gd_last_gnss_time = INITIAL_TIME;
}

void ins_softreset(void)
{
  memset(&gz_imu_buf, 0, sizeof(gz_imu_buf));
  memset(&gz_ins_results, 0, sizeof(gz_ins_results));
  gnss_form_meas_init();
  navconfig_initial();
  fusion_navsys_partial_init();
  fusion_pl_para_init();

  AutoZuptDetect_t* pz_autozupt_para = fusion_get_autozupt_para();
  pz_autozupt_para->u_zupt_inited = INS_FALSE;

  InitialAlign_t* pz_align = fusion_get_align();
  pz_align->u_align_status = INS_FALSE;
  gd_last_gnss_time = INITIAL_TIME;
}

void ins_hardreset(void)
{
  fusion_if_initial();
  fusion_pl_para_init();

  AutoZuptDetect_t* pz_autozupt_para = fusion_get_autozupt_para();
  pz_autozupt_para->u_zupt_inited = INS_FALSE;

  InitialAlign_t* pz_align = fusion_get_align();
  pz_align->u_align_status = INS_FALSE;
  gd_last_gnss_time = INITIAL_TIME;
}

INSResults_t* fusion_get_ins_results(void)
{
  return &gz_ins_results;
}

int8_t imu_get_data_ringbuf(AsensingIMU_t* pz_imu)
{
  int8_t s_val = 1;
  if (gz_imu_buf.u_read_index != gz_imu_buf.u_write_index)
  {
    gz_imu_buf.u_read_index = (gz_imu_buf.u_read_index) & (IMUS_BUF_SIZE - 1);
    memcpy(pz_imu, &gz_imu_buf.z_imus[gz_imu_buf.u_read_index], sizeof(AsensingIMU_t));
    gz_imu_buf.u_read_index++;
  }
  else
  {
    s_val = -1;
  }
  return s_val;
}

void imu_put_data_ringbuf(AsensingIMU_t* pz_imu)
{
  if(gz_imu_buf.u_write_index != (gz_imu_buf.u_read_index ^ IMUS_BUF_SIZE))
  {
    gz_imu_buf.u_write_index = (gz_imu_buf.u_write_index) & (IMUS_BUF_SIZE - 1);
    memcpy(&gz_imu_buf.z_imus[gz_imu_buf.u_write_index], pz_imu, sizeof(AsensingIMU_t));
    gz_imu_buf.u_write_index++;
  }
}

int8_t whpls_get_data_ringbuf(AsensingWhpulse_t* pz_whpls)
{
  int8_t s_val = 1;

  if (gz_whpls_buf.u_read_index != gz_whpls_buf.u_write_index)
  {
    gz_whpls_buf.u_read_index = (gz_whpls_buf.u_read_index) & (WHLS_BUF_SIZE - 1);
    memcpy(pz_whpls, &gz_whpls_buf.z_whls[gz_whpls_buf.u_read_index], sizeof(AsensingWhpulse_t));
    gz_whpls_buf.u_read_index++;
  }
  else
  {
    s_val = -1;
  }
  return s_val;
}

void whpls_put_data_ringbuf(AsensingWhpulse_t* pz_wheel)
{
  if (gz_whpls_buf.u_write_index != (gz_whpls_buf.u_read_index ^ WHLS_BUF_SIZE))
  {
    gz_whpls_buf.u_write_index = (gz_whpls_buf.u_write_index) & (WHLS_BUF_SIZE - 1);
    memcpy(&gz_whpls_buf.z_whls[gz_whpls_buf.u_write_index], pz_wheel, sizeof(AsensingWhpulse_t));
    gz_whpls_buf.u_write_index++;
  }
}

void imu_form_unify_meas(AsensingIMU_t* pz_imu, WheelSpd_t* pz_whsig,double* d_imu)
{
  d_imu[0] = 1.0;
  d_imu[1] = pz_imu->t_timestamp * 0.001;
  d_imu[2] = pz_imu->f_gyro[0];
  d_imu[3] = pz_imu->f_gyro[1];
  d_imu[4] = pz_imu->f_gyro[2];
  d_imu[5] = pz_imu->f_accl[0];
  d_imu[6] = pz_imu->f_accl[1];
  d_imu[7] = pz_imu->f_accl[2];
  d_imu[8] = pz_whsig->d_timestamp;
  d_imu[9] = pz_whsig->f_vehspd_rl;
  d_imu[10] = pz_whsig->f_vehspd_rr;
  d_imu[11] = pz_whsig->f_vehspd_fl;
  d_imu[12] = pz_whsig->f_vehspd_fr;
    
  d_imu[15] = pz_imu->f_temp;
}

void compensate_vel_lever_arm(float* pf_vel, float* pf_lvrm, uint8_t u_bidirect)
{
  float f_lr_b[3] = { 0.f }, f_lr_n[3] = { 0.f };
  float f_omega_inn[3], f_winxln[3],f_lbxwibb[3];
  float f_lr_bx[3][3], f_rotmp[3][3],f_comp_vel[3];
  NavParams_t* pz_nav = fusion_get_navpara();

  math_matmul(&(pz_nav->f_rotmat_v2b[0][0]), pf_lvrm, 3, 3, 1, f_lr_b);
  math_skewsym(f_lr_b, f_lr_bx);

  if (pz_nav->u_inited > 0)
  {
    math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), f_lr_b, 3, 3, 1, f_lr_n);
    math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), *f_lr_bx, 3, 3, 3, *f_rotmp);
    math_matmul(*f_rotmp, pz_nav->f_omega_ibb, 3, 3, 1, f_lbxwibb);
  }
  else
  {
    float f_eluer[3] = { 0.f };
    float  f_rotmat[3][3] = { 0.f };
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();

    f_eluer[2] = (float)(pz_gnss->f_heading * DEG2RAD);
    tf_euler2dcm(f_eluer, f_rotmat);
    math_matmul(&(f_rotmat[0][0]), f_lr_b, 3, 3, 1, f_lr_n);
    math_matmul(&(f_rotmat[0][0]), *f_lr_bx, 3, 3, 3, *f_rotmp);
    math_matmul(*f_rotmp, pz_nav->f_omega_ibb, 3, 3, 1, f_lbxwibb); 
  }
  math_matadd(pz_nav->f_omega_ie,pz_nav->f_omega_en,3,1,f_omega_inn);
  math_cross_product(f_omega_inn, f_lr_n, f_winxln);
  math_matadd(f_winxln, f_lbxwibb,3,1, f_comp_vel);

  if (u_bidirect == 0)
  {
    pf_vel[0] += f_comp_vel[0];
    pf_vel[1] += f_comp_vel[1];
    pf_vel[2] += f_comp_vel[2];
  }
  else
  {
    pf_vel[0] -= f_comp_vel[0];
    pf_vel[1] -= f_comp_vel[1];
    pf_vel[2] -= f_comp_vel[2];
  }
}

void compensate_pos_lever_arm(double* pd_blh, float* pf_lvrm, uint8_t u_bidirect)
{
  float f_lr_n[3] = { 0.f };
  float f_rot_v2n[3][3] = { 0.f };
  double d_rm, d_rn;

  NavParams_t* pz_nav = fusion_get_navpara();

  if (pz_nav->u_inited > 0)
  {
    math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), &(pz_nav->f_rotmat_v2b[0][0]), 3, 3, 3, *f_rot_v2n);
    math_matmul(*f_rot_v2n, pf_lvrm, 3, 3, 1, f_lr_n);
  }
  else
  {
    float f_eluer[3] = { 0.f };
    float  f_rotmat[3][3] = { 0.f };
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
    f_eluer[2] = (float)(pz_gnss->f_heading * DEG2RAD);
    tf_euler2dcm(f_eluer, f_rotmat);
    math_matmul(&(f_rotmat[0][0]), pf_lvrm, 3, 3, 1, f_lr_n);
  }
  ellip_calculateMN(pd_blh[0], &d_rm, &d_rn);

  f_lr_n[0] /= (float)((d_rm + pd_blh[2]));
  f_lr_n[1] /= (float)((d_rn + pd_blh[2]) * cos(pd_blh[0]));
  f_lr_n[2] = -f_lr_n[2];

  if (u_bidirect == 0)
  {
    pd_blh[0] -= f_lr_n[0];
    pd_blh[1] -= f_lr_n[1];
    pd_blh[2] -= f_lr_n[2];
  }
  else
  {
    pd_blh[0] += f_lr_n[0];
    pd_blh[1] += f_lr_n[1];
    pd_blh[2] += f_lr_n[2];
  }
}

#if 1
int8_t fusion_dispose_output(INSResults_t* pz_ins)
{
  int8_t s_val = TYPE_STATUS_FAILURE;

  FusionAlgoConfig_t* pz_algocfg = fusion_get_algconfig();
  NavConfig_t* pz_config = fusion_get_navconfig();

  static uint64_t t_last_instime = 604900000;
  static double d_last_gnsstime = -999.9;   

  if (pz_ins->e_drposflag == TYPE_DR_POS_FLAG_FUSION || pz_ins->e_drposflag == TYPE_DR_POS_FLAG_DR_OLY)
  {
    int output_sampling_dt = 1000 / pz_algocfg->z_init_para.e_output_rate;
    int imu_sampling_dt = 1000 / fusion_get_algconfig()->z_init_para.e_sampling_rate;
    int val = pz_ins->t_timestamp % output_sampling_dt;

    if ((val < imu_sampling_dt / 2 + 1 || (output_sampling_dt - val < imu_sampling_dt / 2 + 1)) &&
      (t_last_instime > 604800000 || pz_ins->t_timestamp - t_last_instime > output_sampling_dt / 2 ||
        (((int64_t)pz_ins->t_timestamp - (int64_t)t_last_instime < -302400000) && (t_last_instime - pz_ins->t_timestamp + 604800000 > output_sampling_dt / 2)))) //avoid week flip
    {
      float f_dist[3], f_cn2e[3][3], f_dist_e[3];
      double d_blh[3], d_posecef[3];
      float  f_vel[3];
      float f_dt = (val > 0.5f * output_sampling_dt ? (output_sampling_dt - val) : -val) * 0.001f;

      s_val = TYPE_STATUS_DATA_READY;
      t_last_instime = pz_ins->t_timestamp;            

      f_vel[0] = pz_ins->f_vn;
      f_vel[1] = pz_ins->f_ve;
      f_vel[2] = pz_ins->f_vd;

      d_blh[0] = pz_ins->d_latitude;
      d_blh[1] = pz_ins->d_longitude;
      d_blh[2] = pz_ins->f_altitude;

      if (fabs(f_dt) > 1e-6)
      {
        InertialNav_t* gz_inav = fusion_get_inertial_nav();

        f_dist[0] = pz_ins->f_vn * f_dt + 0.5f * gz_inav->pz_cur_mech->f_fn[0] * SQR(f_dt);
        f_dist[1] = pz_ins->f_ve * f_dt + 0.5f * gz_inav->pz_cur_mech->f_fn[1] * SQR(f_dt);
        f_dist[2] = pz_ins->f_vd * f_dt + 0.5f * gz_inav->pz_cur_mech->f_fn[2] * SQR(f_dt);

        f_vel[0] += gz_inav->pz_cur_mech->f_fn[0] * f_dt;
        f_vel[1] += gz_inav->pz_cur_mech->f_fn[1] * f_dt;
        f_vel[2] += gz_inav->pz_cur_mech->f_fn[2] * f_dt;
        tf_lla2ecef(d_blh, d_posecef, d_blh);

        tf_cn2ebylla(d_blh, f_cn2e);

        math_matmul(*f_cn2e, f_dist, 3, 3, 1, f_dist_e);

        d_posecef[0] += f_dist_e[0];
        d_posecef[1] += f_dist_e[1];
        d_posecef[2] += f_dist_e[2];

        tf_ecef2lla(d_posecef, d_blh);

        LOGI(TAG_VDR, "[Fusion_Comp]:%.3lf,%f\n", pz_ins->q_tow / 1000.0, f_dt);
      }

      // output to gnss center, or rearmid, or imu center
      if (pz_algocfg->z_init_para.u_outputpos_flag == (uint8_t)TYPE_OUTPUTPOS_GNSS)
      {
        if (pz_config->u_i2g_opt == 1)
        {
          float* pf_imu2gnss = lvrm_get_imu2gnss();
          compensate_pos_lever_arm(d_blh, pf_imu2gnss, BLH_IMU2XPOS);
          compensate_vel_lever_arm(f_vel, pf_imu2gnss, BLH_IMU2XPOS);
        }
      }
      else if (pz_algocfg->z_init_para.u_outputpos_flag == (uint8_t)TYPE_OUTPUTPOS_REARMID)
      {
        if ((pz_config->u_ws_opt & TYPE_MASK_LVRM_REAR_MID) == TYPE_MASK_LVRM_REAR_MID)
        {
          float* pf_imu2rear = lvrm_get_imu2rearmid();
          compensate_pos_lever_arm(d_blh, pf_imu2rear, BLH_IMU2XPOS);
          compensate_vel_lever_arm(f_vel, pf_imu2rear, BLH_IMU2XPOS);
        }
      }
      else if (pz_algocfg->z_init_para.u_outputpos_flag == (uint8_t)TYPE_OUTPUTPOS_IMU)
      {
        /* at imu center */
      }
      else
      {
        /* do nothing */
      }

      pz_ins->f_vn = f_vel[0];
      pz_ins->f_ve = f_vel[1];
      pz_ins->f_vd = f_vel[2];

      pz_ins->d_latitude = d_blh[0];
      pz_ins->d_longitude = d_blh[1];
      pz_ins->f_altitude = (float)d_blh[2];
      pz_ins->q_tow = (uint32_t)((int64_t)pz_ins->t_timestamp + (int64_t)(f_dt * 1000.0));
    }        
  }
  else
  {
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
    if ((pz_gnss->d_tow - d_last_gnsstime) != 0.0 && pz_gnss->e_posflag > 0)
    {
      double d_blh[3] = { 0.0 };
      float f_vel[3] = { 0.f };

      s_val = TYPE_STATUS_DATA_READY;
      d_last_gnsstime = pz_gnss->d_tow;
            
      d_blh[0] = pz_gnss->d_lat * DEG2RAD;
      d_blh[1] = pz_gnss->d_lon * DEG2RAD;
      d_blh[2] = pz_gnss->f_alt;

      f_vel[0] = pz_gnss->f_vn;
      f_vel[1] = pz_gnss->f_ve;
      f_vel[2] = pz_gnss->f_vd;

      /* output to gnss center, or rearmid, or imu center */
      if (pz_algocfg->z_init_para.u_outputpos_flag == (uint8_t)TYPE_OUTPUTPOS_GNSS)
      {
        /* at gnss center */
      }
      else if (pz_algocfg->z_init_para.u_outputpos_flag == (uint8_t)TYPE_OUTPUTPOS_REARMID)
      {
        float* pf_imu2gnss = lvrm_get_imu2gnss();
        float* pf_imu2rear = lvrm_get_imu2rearmid();
        float f_gnss2rear[3] = {0};
        math_matsub(pf_imu2rear, pf_imu2gnss, 3, 1, f_gnss2rear);
        compensate_pos_lever_arm(d_blh, f_gnss2rear, BLH_IMU2XPOS);
        compensate_vel_lever_arm(f_vel, f_gnss2rear, BLH_IMU2XPOS);
        /*
        if (pz_config->u_i2g_opt == 1)
        {
          float* pf_imu2gnss = lvrm_get_imu2gnss();
          compensate_pos_lever_arm(d_blh, pf_imu2gnss, BLH_XPOS2IMU);
          compensate_vel_lever_arm(f_vel, pf_imu2gnss, BLH_XPOS2IMU);
        }
        if ((pz_config->u_ws_opt & TYPE_MASK_LVRM_REAR_MID) == TYPE_MASK_LVRM_REAR_MID)
        {
          float* pf_imu2rear = lvrm_get_imu2rearmid();
          compensate_pos_lever_arm(d_blh, pf_imu2rear, BLH_IMU2XPOS);
          compensate_vel_lever_arm(f_vel, pf_imu2rear, BLH_IMU2XPOS);
        }
        */
      }
      else if (pz_algocfg->z_init_para.u_outputpos_flag == (uint8_t)TYPE_OUTPUTPOS_IMU)
      {
        if (pz_config->u_i2g_opt == 1)
        {
          float* pf_imu2gnss = lvrm_get_imu2gnss();
          compensate_pos_lever_arm(d_blh, pf_imu2gnss, BLH_XPOS2IMU);
          compensate_vel_lever_arm(f_vel, pf_imu2gnss, BLH_XPOS2IMU);
        }
      }
      else
      {
        /* do nothing */
      }

      pz_ins->d_latitude = d_blh[0];
      pz_ins->d_longitude = d_blh[1];
      pz_ins->f_altitude = (float)d_blh[2];

      pz_ins->f_vn = f_vel[0];
      pz_ins->f_ve = f_vel[1];
      pz_ins->f_vd = f_vel[2];

      pz_ins->f_heading = pz_gnss->f_heading;
      pz_ins->f_speed = pz_gnss->f_speed;
      pz_ins->q_tow = (uint32_t)pz_gnss->d_tow;
      pz_ins->w_week = pz_gnss->w_week;
      pz_ins->e_drposflag = pz_gnss->e_posflag;
    }        
  }

  return s_val;
}
#else
int8_t fusion_dispose_output(INSResults_t* pz_ins)
{
  int8_t s_val = TYPE_STATUS_FAILURE;

  FusionAlgoConfig_t* pz_algocfg = fusion_get_algconfig();
  NavConfig_t* pz_config = fusion_get_navconfig();

  static uint64_t t_last_timestamp = 0;
  int output_sampling_dt = 1000 / pz_algocfg->z_init_para.e_output_rate;
  int imu_sampling_dt = 1000 / pz_algocfg->z_init_para.e_sampling_rate;
  /* float val = fmod(pz_ins->d_timestamp, output_sampling_dt); */
  int val = pz_ins->t_timestamp % output_sampling_dt;

  /*if (val < imu_sampling_dt || (output_sampling_dt - val < imu_sampling_dt)) */
  if ((val < imu_sampling_dt / 2 + 1 || (output_sampling_dt - val < imu_sampling_dt / 2 + 1))  
      && pz_ins->t_timestamp - t_last_timestamp > output_sampling_dt/2)
  {
    s_val = TYPE_STATUS_DATA_READY;

    t_last_timestamp = pz_ins->t_timestamp;
  }  

  if (s_val == TYPE_STATUS_DATA_READY)
  {
    if (pz_ins->e_drposflag == TYPE_DR_POS_FLAG_FUSION || pz_ins->e_drposflag == TYPE_DR_POS_FLAG_DR_OLY)
    {
      float f_dist[3], f_cn2e[3][3], f_dist_e[3];
      double d_blh[3], d_posecef[3];
      float  f_vel[3];
      float f_dt = (val > 0.5f * output_sampling_dt ? (output_sampling_dt - val) : -val) * 0.001f;

      f_vel[0] = pz_ins->f_vn;
      f_vel[1] = pz_ins->f_ve;
      f_vel[2] = pz_ins->f_vd;

      d_blh[0] = pz_ins->d_latitude;
      d_blh[1] = pz_ins->d_longitude;
      d_blh[2] = pz_ins->f_altitude;

      if (fabs(f_dt) > 1e-6)
      {
        InertialNav_t* gz_inav = fusion_get_inertial_nav();

        f_dist[0] = pz_ins->f_vn * f_dt + 0.5f * gz_inav->pz_cur_mech->f_fn[0] * SQR(f_dt);
        f_dist[1] = pz_ins->f_ve * f_dt + 0.5f * gz_inav->pz_cur_mech->f_fn[1] * SQR(f_dt);
        f_dist[2] = pz_ins->f_vd * f_dt + 0.5f * gz_inav->pz_cur_mech->f_fn[2] * SQR(f_dt);

        f_vel[0] += gz_inav->pz_cur_mech->f_fn[0] * f_dt;
        f_vel[1] += gz_inav->pz_cur_mech->f_fn[1] * f_dt;
        f_vel[2] += gz_inav->pz_cur_mech->f_fn[2] * f_dt;
        tf_lla2ecef(d_blh, d_posecef, d_blh);

        tf_cn2ebylla(d_blh, f_cn2e);

        math_matmul(*f_cn2e, f_dist, 3, 3, 1, f_dist_e);

        d_posecef[0] += f_dist_e[0];
        d_posecef[1] += f_dist_e[1];
        d_posecef[2] += f_dist_e[2];

        tf_ecef2lla(d_posecef, d_blh);

        LOGI(TAG_VDR, "[Fusion_Comp]:%.3lf,%f\n", pz_ins->q_tow / 1000.0, f_dt);
      }

      /* output to rearmid, or gnss center, or imu center */
      if ((pz_config->u_ws_opt & TYPE_MASK_LVRM_REAR_MID) == TYPE_MASK_LVRM_REAR_MID)
      {
        float* pf_imu2rear = lvrm_get_imu2rearmid();
        compensate_pos_lever_arm(d_blh, pf_imu2rear, BLH_IMU2XPOS);
        compensate_vel_lever_arm(f_vel, pf_imu2rear, BLH_IMU2XPOS);
      }
      else if (pz_config->u_i2g_opt)
      {
        float* pf_imu2gnss = lvrm_get_imu2gnss();
        compensate_pos_lever_arm(d_blh, pf_imu2gnss, BLH_IMU2XPOS);
        compensate_vel_lever_arm(f_vel, pf_imu2gnss, BLH_IMU2XPOS);
      }

      pz_ins->f_vn = f_vel[0];
      pz_ins->f_ve = f_vel[1];
      pz_ins->f_vd = f_vel[2];

      pz_ins->d_latitude = d_blh[0];
      pz_ins->d_longitude = d_blh[1];
      pz_ins->f_altitude = d_blh[2];

      /* pz_ins->f_tow = (float)((double)pz_ins->t_timestamp + (double)(f_dt * 1000.0)); */
      pz_ins->q_tow = (uint32_t)((int64_t)pz_ins->t_timestamp + (int64_t)(f_dt * 1000.0));
    }
    else
    {
      double d_blh[3] = { 0.0 };
      float f_vel[3] = { 0.f };
      AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
      d_blh[0] = pz_gnss->d_lat * DEG2RAD;
      d_blh[1] = pz_gnss->d_lon * DEG2RAD;
      d_blh[2] = pz_gnss->f_alt;

      f_vel[0] = pz_gnss->f_vn;
      f_vel[1] = pz_gnss->f_ve;
      f_vel[2] = pz_gnss->f_vd;

      if ((pz_config->u_ws_opt & TYPE_MASK_LVRM_REAR_MID) == TYPE_MASK_LVRM_REAR_MID)
      {
        float* pf_imu2rear = lvrm_get_imu2rearmid();
        compensate_pos_lever_arm(d_blh, pf_imu2rear, BLH_IMU2XPOS);
        compensate_vel_lever_arm(f_vel, pf_imu2rear, BLH_IMU2XPOS);
      }

      pz_ins->d_latitude = d_blh[0];
      pz_ins->d_longitude = d_blh[1];
      pz_ins->f_altitude = d_blh[2];

      pz_ins->f_vn = f_vel[0];
      pz_ins->f_ve = f_vel[1];
      pz_ins->f_vd = f_vel[2];

      pz_ins->f_heading = pz_gnss->f_heading;
      pz_ins->f_speed = pz_gnss->f_speed;
      pz_ins->q_tow = (uint32_t)pz_gnss->d_tow;
      pz_ins->w_week = pz_gnss->w_week;
    }
  }
  return s_val;
}
#endif

int8_t fusion_calculation(AsensingIMU_t* pz_imu, GNSSFormMeas_t* pz_form_meas, WheelSpd_t* pz_whsig, INSResults_t* pz_ins)
{
  double d_meas_imu[UNIFY_MEAS_SIZE] = { 0 };
  int8_t s_state = TYPE_STATUS_DATA_READY;
  float f_near_dt = (float)(fusion_get_algconfig()->f_sampling_dt / 2.f + 1e-3);
  float f_thr_up = 3.f * fusion_get_algconfig()->f_sampling_dt;

  imu_form_unify_meas(pz_imu, pz_whsig, d_meas_imu);
#if 0
  if (pz_form_meas->d_gnss_meas[1] > d_last_gnss_time + 1e-3 && pz_form_meas->d_gnss_meas[1] + f_near_dt < d_meas_imu[1]
      && pz_form_meas->d_gnss_meas[1] + f_thr_up > d_meas_imu[1])
  {
    s_state = ins_handle_meas(pz_form_meas->d_gnss_meas, pz_ins);
    d_last_gnss_time = pz_form_meas->d_gnss_meas[1];
  }
#endif
  s_state = ins_handle_meas(d_meas_imu, pz_ins);

  if (fabs(pz_form_meas->d_gnss_meas[1] - gd_last_gnss_time) > 1e-6f)
  {
    s_state = ins_handle_meas(pz_form_meas->d_gnss_meas, pz_ins);
    gd_last_gnss_time = pz_form_meas->d_gnss_meas[1];
  }
  if ((d_meas_imu[1] - gd_last_gnss_time > 5) && (is_fusion_valid() == INS_TRUE))
  {
    pz_ins->e_drposflag = TYPE_DR_POS_FLAG_DR_OLY;
  }

  pz_ins->t_timestamp = pz_imu->t_timestamp;

  return s_state;
}

int8_t fusion_ins_entry(AsensingIMU_t* pz_imu, INSResults_t* pz_ins)
{
  int8_t s_val = TYPE_STATUS_FAILURE;

  GNSSFormMeas_t* pz_gnss = gnss_get_form_meas();
  gnss_form_unify_meas(pz_gnss);

  AsensingWhpulse_t z_whpls;
  if (whpls_get_data_ringbuf(&z_whpls) > 0)
  {
    whpls_convert_to_whspd(fusion_get_wheelspd(), &z_whpls);
  }
  WheelSpd_t* pz_whsig = fusion_get_wheelspd();

  fusion_fault_detection_handler(pz_imu, pz_gnss, pz_whsig, pz_ins);

  fusion_calculation(pz_imu, pz_gnss, pz_whsig, pz_ins);

  s_val = fusion_dispose_output(pz_ins);

  uint8_t u_drMaxTime = fusion_get_algconfig()->z_init_para.u_dr_time;
  if (pz_ins->t_timestamp * 0.001 - gd_last_gnss_time > u_drMaxTime && gd_last_gnss_time > 0 && u_drMaxTime > 0)
  {
    s_val = TYPE_STATUS_FAILURE;
    LOGW(TAG_VDR, "Dr time exceed the set time %d, last GNSS %f\n", u_drMaxTime, gd_last_gnss_time);
  }
  return s_val;
}
