/**@file        fusion_mech.c
 * @brief		fusion mechanition file
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

#include "fusion_math.h"
#include "fusion_mech.h"
#include "fusion_quat.h"
#include "fusion_proc.h"
#include "fusion_global.h"
#include "fusion_ellip_para.h"
#include "fusion_transform.h"
#include "fusion_log.h"
#include "fusion_if.h"
#include <string.h>
#include "mw_log.h"
#ifdef INS_DEBUG
#include"fusion_reference_debug.h"
#endif

static MechanParas_t gz_mechan;

void mechan_initial(void)
{
  memset(&gz_mechan,0,sizeof(gz_mechan));
  gz_mechan.f_mechan_t = MECH_DEFAULT_DT;
  gz_mechan.d_prev_imu[1] = INITIAL_TIME;
}

double mechan_get_time(void)
{
  return gz_mechan.d_mech_time;
}

void mechan_set_time(double d_time)
{
  gz_mechan.d_mech_time = d_time;
}

/*
* @brief: position extrapolate
* @param[in]: d_poslla - lat lon h; f_vel_ned -  vn ve vd; d_midpos - position at midtime; f_delta_t - time interval
* @param[out]: position at midtime
* @return: None
*/
void pos_extrapolate(double d_poslla[3], float f_vel_ned[3], double d_midpos[3], float f_delta_t,
  double d_quat_n2e_in[4])
{
  double d_poslat = d_poslla[0];
  double d_rm, d_rn;
  float	d_delta_lat, d_delta_lon;
  float f_delta_theta[3];
  float f_delta_quat[4];
  double d_quat_n2e[4];
  double d_midpos_temp[2];
  double d_q_temp[4];

  d_midpos[2] = d_poslla[2] - 0.5 * f_vel_ned[2] * f_delta_t;

  ellip_calculateMN(d_poslat, &d_rm, &d_rn);
  d_delta_lat = 0.5f * f_vel_ned[0] * f_delta_t / (float)(d_rm + d_poslla[2]);
  d_delta_lon = 0.5f * f_vel_ned[1] * f_delta_t / (float)(d_rn + d_poslla[2]) / (float)cos(d_poslat);

  tf_llaerr2rotvct(d_poslat, d_delta_lat, d_delta_lon, f_delta_theta); 

  quat_rotvec2quat(f_delta_theta, f_delta_quat);

  d_q_temp[0] = f_delta_quat[0];
  d_q_temp[1] = f_delta_quat[1];
  d_q_temp[2] = f_delta_quat[2];
  d_q_temp[3] = f_delta_quat[3];

  quat_copy_d(d_quat_n2e, d_quat_n2e_in);

  quat_product_d(d_quat_n2e, d_q_temp);

  quat_get_posbyn2e(d_midpos_temp, d_quat_n2e);

  d_midpos[0] = d_midpos_temp[0];
  d_midpos[1] = d_midpos_temp[1];
}

void mechan_vel_update(const double* pd_imu_cur, NavParams_t* pz_nav, MechanNode_t* pz_node,
    MechanParas_t* pz_mechan)
{
  float	f_gravity_vct[3] = { 0.0f };
  float	f_omega_ie_n[3] = { 0.0f };
  float	f_omega_en_n[3] = { 0.0f };
  float	f_zeta[3] = { 0.0f };
  float	f_tiny_rotmat[3][3] = { 0.0f };
  float	f_curr_acc[3] = { 0.0f };
  float	f_tiny_rotmat_temp[3][3] = { 0.0f };
  float	f_delta_vel_fn[3] = { 0.0f };
  float	f_delta_vel_gn[3] = { 0.0f };
  float	f_delta_vel_gn_temp[3] = { 0.0f };
  float	f_nav1_delta_vel[3] = { 0.0f };
  float	f_nav1_vel[3] = { 0.0f };
  double*  pd_midpos = pz_mechan->d_midpos;
  float* pf_midvel = pz_mechan->f_midvel;
  float* pf_sculling = pz_mechan->f_sculling;
  uint8_t	i;

  /* position extrapolate */
  pos_extrapolate(pz_nav->d_poslla, pz_nav->f_vel_ned, pd_midpos, pz_mechan->f_delta_t, pz_nav->d_quat_n2e);

  /* calculate gravity */
  f_gravity_vct[0] = 0.0f;
  f_gravity_vct[1] = 0.0f;
  f_gravity_vct[2] = ellip_get_gravity(pd_midpos);
  memcpy(pz_nav->f_gravity_vct, f_gravity_vct, sizeof(pz_nav->f_gravity_vct));

  /* velocity extrapolate */
  memcpy(pf_midvel, pz_nav->f_delta_vel_ned, sizeof(pz_nav->f_delta_vel_ned));
  pf_midvel[0] *= 0.5f;
  pf_midvel[1] *= 0.5f;
  pf_midvel[2] *= 0.5f;
  math_matadd(pf_midvel, pz_nav->f_vel_ned, 3, 1, pf_midvel);

  memcpy(f_omega_ie_n, pz_nav->f_omega_ie, sizeof(f_omega_ie_n));
  tf_get_wien(pd_midpos, f_omega_ie_n);

  memcpy(f_omega_en_n, pz_nav->f_omega_en, sizeof(f_omega_en_n));
  tf_get_wenn(pd_midpos, pf_midvel, f_omega_en_n);

  /* calculate rotate vector in n-frame */
  math_matadd(f_omega_ie_n, f_omega_en_n, 3, 1, f_zeta);
  f_zeta[0] *= (-pz_mechan->f_delta_t * 0.5f);
  f_zeta[1] *= (-pz_mechan->f_delta_t * 0.5f);
  f_zeta[2] *= (-pz_mechan->f_delta_t * 0.5f);

  tf_rotvct2mat(f_tiny_rotmat, f_zeta);

  f_curr_acc[0] = (float)(pd_imu_cur[5]);
  f_curr_acc[1] = (float)(pd_imu_cur[6]);
  f_curr_acc[2] = (float)(pd_imu_cur[7]);

  math_matadd(f_curr_acc, pf_sculling, 3, 1, f_curr_acc);

  math_matmul(&(f_tiny_rotmat[0][0]), &(pz_nav->f_rotmat_b2n[0][0]),
    3, 3, 3, &(f_tiny_rotmat_temp[0][0]));

  math_matmul(&(f_tiny_rotmat_temp[0][0]), f_curr_acc, 3, 3, 1, f_delta_vel_fn);

  /* gravity */
  memcpy(f_delta_vel_gn, f_omega_ie_n, sizeof(f_delta_vel_gn));

  f_delta_vel_gn[0] *= 2.0f;
  f_delta_vel_gn[1] *= 2.0f;
  f_delta_vel_gn[2] *= 2.0f;
  math_matadd(f_delta_vel_gn, f_omega_en_n, 3, 1, f_delta_vel_gn);

  math_cross_product(f_delta_vel_gn, pf_midvel, f_delta_vel_gn_temp);

  math_matsub(f_gravity_vct, f_delta_vel_gn_temp,3,1, f_delta_vel_gn);

  f_delta_vel_gn[0] *= pz_mechan->f_delta_t;
  f_delta_vel_gn[1] *= pz_mechan->f_delta_t;
  f_delta_vel_gn[2] *= pz_mechan->f_delta_t;

  /* save updated velocity */
  math_matadd(f_delta_vel_gn, f_delta_vel_fn, 3, 1, f_nav1_delta_vel);
  f_delta_vel_fn[0] *= (1.0f / pz_mechan->f_delta_t);
  f_delta_vel_fn[1] *= (1.0f / pz_mechan->f_delta_t);
  f_delta_vel_fn[2] *= (1.0f / pz_mechan->f_delta_t);
  memcpy(pz_nav->f_fn, f_delta_vel_fn, sizeof(pz_nav->f_fn));
  memcpy(f_nav1_vel, f_nav1_delta_vel, sizeof(f_nav1_vel));
  math_matadd(f_nav1_vel, pz_nav->f_vel_ned, 3, 1, f_nav1_vel); /* vn(k) = vn(k-1) + f_delta_vel_fn + f_delta_vel_gn */

  /* interpolate velocity based on updated velocity */
  memcpy(pf_midvel, pz_nav->f_vel_ned, sizeof(pz_nav->f_vel_ned));
  math_matadd(pf_midvel, f_nav1_vel, 3, 1, pf_midvel);
  pf_midvel[0] *= 0.5;
  pf_midvel[1] *= 0.5;
  pf_midvel[2] *= 0.5; /* (vn(k) + vn(k-1))/ 2 */

  /* update final velocity to pz_nav */
  memcpy(pz_nav->f_vel_ned, f_nav1_vel, sizeof(pz_nav->f_vel_ned));
  memcpy(pz_nav->f_delta_vel_ned, f_nav1_delta_vel, sizeof(pz_nav->f_delta_vel_ned));

  /* save velocity parameters */
  for (i = 0; i < 3; i++)
  {
    pz_node->f_fb[i] = f_curr_acc[i] / pz_mechan->f_delta_t;
    pz_node->f_vel[i] = f_nav1_vel[i];
    pz_node->f_deta_vel[i] = f_nav1_delta_vel[i];
    pz_node->f_wien[i] = f_omega_ie_n[i];
    pz_node->f_fn[i] = pz_nav->f_fn[i] + pz_nav->f_gravity_vct[i]; /* pz_node->f_an */
    pz_node->d_rm = pz_nav->d_rm;
    pz_node->d_rn = pz_nav->d_rn;
  }
}

void mechan_pos_update(NavParams_t* pz_nav, MechanNode_t* pz_node, MechanParas_t* pz_mechan)
{
  float	f_omega_en_n[3] = { 0.0f };
  float	f_omega_in_n[3] = { 0.0f };
  float	f_zeta[3] = { 0.0f };
  double d_quat_n[4] = { 0.0 };
  float	f_xi[3] = { 0.0f };
  double d_nav1_quat_ne[4] = { 0.0 };
  double d_nav1_pos_tmp[2] = { 0.0 };
  float	f_qn2e_tmp[4] = { 0.0f };
  float	f_quatn_temp[4] = { 0.0f };
  double d_nav1_pos[3] = { 0.0 };
  double* d_midpos = pz_mechan->d_midpos;
  float* f_midvel = pz_mechan->f_midvel;
  uint8_t	i;  

  /* update relevant parameters based on new velocity */
  tf_get_wenn(d_midpos, f_midvel, f_omega_en_n);
  memcpy(pz_nav->f_omega_en, f_omega_en_n, sizeof(pz_nav->f_omega_en));
  math_matadd(pz_node->f_wien, f_omega_en_n, 3, 1, f_omega_in_n);
  /* update f_zeta */
  f_zeta[0] = f_omega_in_n[0] * pz_mechan->f_delta_t;
  f_zeta[1] = f_omega_in_n[1] * pz_mechan->f_delta_t;
  f_zeta[2] = f_omega_in_n[2] * pz_mechan->f_delta_t;

  quat_rotvec2quat(f_zeta, f_quatn_temp);

  memset(f_xi, 0, sizeof(f_xi));
  f_xi[2] = (float)(-(fusion_get_ellippara()->d_wie)) * pz_mechan->f_delta_t;

  quat_rotvec2quat(f_xi, f_qn2e_tmp);

  for (i = 0; i < 4; i++)
  {
    d_quat_n[i] = f_quatn_temp[i];
    d_nav1_quat_ne[i] = f_qn2e_tmp[i];
  }

  quat_product_d(pz_nav->d_quat_n2e, d_quat_n);
  quat_product_d(d_nav1_quat_ne, pz_nav->d_quat_n2e);

  quat_normalize_d(d_nav1_quat_ne);
  quat_get_posbyn2e(d_nav1_pos_tmp, d_nav1_quat_ne);

  d_nav1_pos[0] = d_nav1_pos_tmp[0];
  d_nav1_pos[1] = d_nav1_pos_tmp[1];
  d_nav1_pos[2] = pz_nav->d_poslla[2] - ((double)f_midvel[2] * (double)pz_mechan->f_delta_t);

  memcpy(d_midpos, d_nav1_pos, sizeof(d_nav1_pos));
  quat_copy_d(pz_nav->d_quat_n2e, d_nav1_quat_ne);

  /* save pos info */
  for (i = 0; i < 3; i++)
  {
    pz_node->d_lla[i] = d_nav1_pos[i];
    pz_node->d_qne[i] = pz_nav->d_quat_n2e[i];
  }
  pz_node->d_qne[3] = pz_nav->d_quat_n2e[3];
}

void mechan_att_update(const double* pd_imu_cur, NavParams_t* pz_nav, MechanNode_t* pz_node, MechanParas_t* pz_mechan)
{
  float	f_curr_gyro[3] = { 0.0f };
  float	f_quat_b[4] = { 0.0f };
  double	pd_midpos[3] = { 0.0 };
  float	f_omega_ie_n[3] = { 0.0f };
  float	f_omega_en_n[3] = { 0.0f };
  float	f_omega_in_n[3] = { 0.0f };
  float	f_zeta[3] = { 0.0f };
  float	f_nav1_quat_bn[4] = { 0.0f };
  float	f_qatt[4] = { 0.0f };

  f_curr_gyro[0] = (float)(pd_imu_cur[2]);
  f_curr_gyro[1] = (float)(pd_imu_cur[3]);
  f_curr_gyro[2] = (float)(pd_imu_cur[4]);

  math_matadd(f_curr_gyro, pz_mechan->f_coning, 3, 1, f_curr_gyro);
  quat_rotvec2quat(f_curr_gyro, f_quat_b);
  memcpy(pd_midpos, pz_node->d_lla, sizeof(pd_midpos));

  pd_midpos[0] = (pd_midpos[0] + pz_nav->d_poslla[0]) * 0.5;
  pd_midpos[1] = (pd_midpos[1] + pz_nav->d_poslla[1]) * 0.5;
  pd_midpos[2] = (pd_midpos[2] + pz_nav->d_poslla[2]) * 0.5;

  tf_get_wien(pd_midpos, f_omega_ie_n);
  memcpy(pz_nav->f_omega_ie, f_omega_ie_n, sizeof(pz_nav->f_omega_ie));
  tf_get_wenn(pd_midpos, pz_mechan->f_midvel, f_omega_en_n);
  memcpy(pz_nav->f_omega_en, f_omega_en_n, sizeof(pz_nav->f_omega_en));
  math_matadd(f_omega_ie_n, f_omega_en_n, 3, 1, f_omega_in_n);

  f_zeta[0] = f_omega_in_n[0] * (-pz_mechan->f_delta_t);
  f_zeta[1] = f_omega_in_n[1] * (-pz_mechan->f_delta_t);
  f_zeta[2] = f_omega_in_n[2] * (-pz_mechan->f_delta_t);

  pz_nav->f_omega_ibb[0] = (float)pd_imu_cur[2] / pz_mechan->f_delta_t;
  pz_nav->f_omega_ibb[1] = (float)pd_imu_cur[3] / pz_mechan->f_delta_t;
  pz_nav->f_omega_ibb[2] = (float)pd_imu_cur[4] / pz_mechan->f_delta_t;

  quat_rotvec2quat(f_zeta, f_nav1_quat_bn);
  quat_copy_f(f_qatt, pz_nav->f_quat_b2n);
  quat_product_f(f_qatt, f_quat_b);
  quat_product_f(f_nav1_quat_bn, f_qatt);
  quat_normalize_f(f_nav1_quat_bn);
  quat_copy_f(pz_nav->f_quat_b2n, f_nav1_quat_bn);
  quat_quat2dcm(pz_nav->f_quat_b2n, pz_nav->f_rotmat_b2n);
  tf_dcm2eluer(pz_nav->f_rotmat_b2n, pz_nav->f_att_rph);
  memcpy(pz_nav->d_poslla, pz_node->d_lla, sizeof(pz_nav->d_poslla));
  tf_get_wenn(pz_nav->d_poslla, pz_nav->f_vel_ned, pz_nav->f_omega_en);

  /* save att info */
  for (uint8_t i = 0u; i < 3u; i++)
  {
    pz_node->f_qbn[i] = f_nav1_quat_bn[i];
    pz_node->f_wien[i] = f_omega_ie_n[i];
    pz_node->f_wibb[i] = f_curr_gyro[i] / pz_mechan->f_delta_t;
    pz_node->f_wnen[i] = f_omega_en_n[i];
  }
  pz_node->f_qbn[3] = f_nav1_quat_bn[3];
}

 /*
 * @brief: Calculate the sculling correction error
 * @param[in]: pd_imu_cur - imu data; pd_imu_pre - last epoch imu data
 * @param[out]: pf_sculling - result matrix
 * @return: None
 */
void mechan_calculate_sculling(const double* pd_imu_cur, const double* pd_imu_pre, float* pf_sculling)
{
  float f_prev_gyro[3];
  float f_prev_acc[3];
  float f_curr_gyro[3];
  float f_curr_acc[3];
  float f_temp1[3];
  float f_temp2[3];
  float f_temp3[3];

  for (uint8_t i = 0; i < 3; i++)
  {
    f_prev_gyro[i] = (float)pd_imu_pre[2 + i];
    f_prev_acc[i] = (float)pd_imu_pre[5 + i];
    f_curr_gyro[i] = (float)pd_imu_cur[2 + i];
    f_curr_acc[i] = (float)pd_imu_cur[5 + i];
  }

  /* the rotational and sculling motion */
  math_cross_product(f_curr_gyro, f_curr_acc, f_temp1);
  math_cross_product(f_prev_gyro, f_curr_acc, f_temp2);
  math_cross_product(f_prev_acc, f_curr_gyro, f_temp3);

  pf_sculling[0] = 0.5f * f_temp1[0] + (f_temp2[0] + f_temp3[0]) / 12.0f;
  pf_sculling[1] = 0.5f * f_temp1[1] + (f_temp2[1] + f_temp3[1]) / 12.0f;
  pf_sculling[2] = 0.5f * f_temp1[2] + (f_temp2[2] + f_temp3[2]) / 12.0f;
}

 /*
 * @brief: Calculate the coning correction error
 * @param[in]: pd_imu_cur - imu data; pd_imu_pre - last epoch imu data
 * @param[out]: pf_coning - result matrix
 * @return: None
 */
void mechan_calculate_coning(const double* pd_imu_cur, const double* pd_imu_pre, float* pf_coning)
{
  /* calculate the second-order coning correctio term */
  float f_prev_gyro[3];
  float	f_curr_gyro[3];

  for (uint8_t i = 0; i < 3; i++)
  {
    f_prev_gyro[i] = (float)pd_imu_pre[i + 2];
    f_curr_gyro[i] = (float)pd_imu_cur[i + 2];
  }

  math_cross_product(f_prev_gyro, f_curr_gyro, pf_coning);
        
  pf_coning[0] *= (1.0f / 12.0f);
  pf_coning[1] *= (1.0f / 12.0f);
  pf_coning[2] *= (1.0f / 12.0f);
}

 /*
 * @brief: Check the input parameters of INS Mechan module
 * @param[in]: pd_imu_cur - imu data; pz_mechan - mech struct
 * @param[out]: None
 * @return: None
 */
int8_t mechan_check_datain(const double* pd_imu_cur, const MechanParas_t* pz_mechan)
{
  int8_t s_ret = 1;

  if (pz_mechan->f_delta_t > 1.5)
  {
    s_ret = -1;
    LOGE(TAG_VDR, "IMU dT Fail, %.3f, dt=%.3f\n", pd_imu_cur[1], pz_mechan->f_delta_t);
  }
  else
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      if ((float)fabs(pd_imu_cur[i + 2]) > 60.0 * DEG2RAD * pz_mechan->f_delta_t)
      {
        s_ret = -2;
        LOGE(TAG_VDR, "GYRO Fail, %.3f, imu_cur=%.3f, dt=%.3f, i=%d \n", pd_imu_cur[1], pd_imu_cur[i + 2], pz_mechan->f_delta_t, i);
        break;
      }

      if ((float)fabs(pd_imu_cur[i + 5]) > 30.0f * pz_mechan->f_delta_t)
      {
        s_ret = -2;
        LOGE(TAG_VDR, "ACC Fail, %.3f, dt=%.3f, i=%d \n", pd_imu_cur[1], pz_mechan->f_delta_t, i);
        break;
      }
    }
  }
  return s_ret;
}

 /*
 * @brief: Check the output parameters of INS Mechan module.
 * @param[in]: nav_pre - last epoch nav; f_mech_dt - delta time
 * @param[out]: nav_cur - cur epoch nav
 * @return: None
 */
int8_t mechan_check_dataout(const NavParams_t* nav_pre, const NavParams_t* nav_cur, const float f_mech_dt)
{
  int8_t s_ret = 1;
  float f_delta_vel[3] = { 0.0f };
  float f_delta_pos[3] = { 0.0f };
  float f_delta_att[3] = { 0.0f };

  /* 1. position increment */
  f_delta_pos[0] = (float)((nav_cur->d_poslla[0] - nav_pre->d_poslla[0]) * nav_pre->d_rm);
  f_delta_pos[1] = (float)((nav_cur->d_poslla[1] - nav_pre->d_poslla[1]) * nav_pre->d_rn * cos(nav_pre->d_poslla[0]));
  f_delta_pos[2] = (float)(nav_cur->d_poslla[2] - nav_pre->d_poslla[2]);

  for (uint8_t i = 0; i < 3; i++)
  {
    /* 2. velocity increment */
    f_delta_vel[i] = nav_cur->f_vel_ned[i] - nav_pre->f_vel_ned[i];

    /* 3. attitude increment */
    f_delta_att[i] = nav_cur->f_att_rph[i] - nav_pre->f_att_rph[i];

    if (f_delta_att[i] < -6.10f)
    {
      f_delta_att[i] += (float)(2.0f * INS_PI);
    }
    else if (f_delta_att[i] > 6.10f)
    {
      f_delta_att[i] -= (float)(2.0f * INS_PI);
    }
    else
    {
      /* do nothing */
    }

    /* 4. velocity increment range check */
    if (fabsf(f_delta_vel[i]) > 20.0 * f_mech_dt)
    {
      s_ret = -2;
      LOGE(TAG_VDR, "f_delta_vel Fail, %.3f, acc=%.3f, dt=%.3f, prevel=%.3f, curvel=%.3f\n", 
        nav_cur->d_sys_timestamp, f_delta_vel[i]/f_mech_dt, f_mech_dt, 
        nav_pre->f_vel_ned[i], nav_cur->f_vel_ned[i]);
      break;
    }

    /* 5. attitude increment range check */
    if (fabsf(f_delta_att[i]) > 0.872 * f_mech_dt)
    {
      s_ret = -2;
      LOGE(TAG_VDR, "f_delta_att Fail, %.3f, angvel=%.3f  \n", nav_cur->d_sys_timestamp, f_delta_att[i]/f_mech_dt);
      break;
    }
        
    /* 6. position increment range check */
    if (fabsf(f_delta_pos[i]) > 50.0 * f_mech_dt)
    {
      s_ret = -2;
      LOGE(TAG_VDR, "f_delta_pos Fail, %.3f, vel=%.3f \n", nav_cur->d_sys_timestamp, f_delta_pos[i]/f_mech_dt);
      break;
    }
  }
  return s_ret;
}

/*
* @brief: Integrate and update the navigation information using IMU data.
* @param[in]: pd_imu_cur - imu data; pz_nav - navigation variable; pz_node - mech struct
* @param[out]: None
* @return: None
*/
int8_t fusion_mechan_handler(const double* pd_imu_cur, NavParams_t* pz_nav, MechanNode_t* pz_node)
{
  int8_t s_ret = 1;
#ifdef INS_DEBUG
  static uint8_t frq = 0;
#endif

  MechanParas_t* pz_mechan = &gz_mechan;
  const double* pd_imu_pre = pz_mechan->d_prev_imu;
  float* f_sculling = pz_mechan->f_sculling;
  float* f_coning = pz_mechan->f_coning;

  NavParams_t z_nav_pre;
  memcpy(&z_nav_pre, pz_nav, sizeof(NavParams_t));

  if ((pd_imu_pre[1] > 0.0f))
  {
    pz_mechan->f_delta_t = (float)(pd_imu_cur[1] - pd_imu_pre[1]);

#if 0
    if (pz_mechan->f_delta_t < INFINITY_MIN || pz_mechan->f_delta_t > 1.5) // 0.05   ref FDE
    {
      pz_mechan->f_delta_t = pz_mechan->f_mechan_t;
    }
#endif
    if (float_equal(pz_mechan->f_delta_t, 0.0) == 1)
    {
      pz_mechan->f_delta_t = MECH_DEFAULT_DT;
    }

    s_ret = mechan_check_datain(pd_imu_cur, pz_mechan);

    if (s_ret == 1)
    {
      mechan_calculate_sculling(pd_imu_cur, pd_imu_pre, f_sculling);
      mechan_calculate_coning(pd_imu_cur, pd_imu_pre, f_coning);

      mechan_vel_update(pd_imu_cur, pz_nav, pz_node, pz_mechan);
      mechan_pos_update(pz_nav, pz_node, pz_mechan);
      mechan_att_update(pd_imu_cur, pz_nav, pz_node, pz_mechan);

      s_ret = mechan_check_dataout(&z_nav_pre, pz_nav, pz_mechan->f_delta_t );
      if (s_ret < 0)
      {          
        LOGE(TAG_VDR, "ins hardreset, mechan_check_dataout fail, %.3f \n", pd_imu_cur[1]);
        ins_softreset();
      }
      pz_nav->d_sys_timestamp = pd_imu_cur[1];
      pz_node->d_time = pd_imu_cur[1];
#ifdef INS_DEBUG
      if (frq >= 0)
      {
        frq = 0;
        double error[12] = {0};
        int ret = 0;
        ret = ref_CalInsAntError(pz_nav, error);
        if (ret > 0)
        {
          LOGE(TAG_VDR, "$INS_MECH_ERROR %10.3f pos %10.3f %10.3f %10.3f vel_n %7.3f %7.3f %7.3f vel_v %7.3f %7.3f %7.3f att %7.3f %7.3f %7.3f\n",
            pz_nav->d_sys_timestamp, error[0], error[1], error[2], error[3], error[4], error[5],
            error[6], error[7], error[8], error[9], error[10], error[11]);
        }
      }
      else
      {
        frq++;
      }
#endif
    }
    else
    {      
      LOGE(TAG_VDR, "ins hardreset, mechan_check_datain fail, %.3f \n", pd_imu_cur[1]);
      ins_softreset();
    }
  }
  else
  {
    s_ret = -1;
  }
  memcpy(pz_mechan->d_prev_imu, pd_imu_cur, sizeof(pz_mechan->d_prev_imu));
  pz_mechan->d_mech_time = pd_imu_cur[1];
  pz_nav->d_sys_timestamp = pd_imu_cur[1];
  return s_ret;
}
