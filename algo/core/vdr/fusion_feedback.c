/**@file        fusion_feedback.c
 * @brief		fusion feedback file
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
#include "fusion_config.h"
#include "fusion_global.h"
#include "fusion_proc.h"
#include "fusion_log.h"
#include "fusion_quat.h"
#include "fusion_transform.h"
#include "fusion_tc.h"
#include "fusion_if.h"
#include <stdint.h>
#include <string.h>
#include "mw_log.h"
#ifdef INS_DEBUG
#include "fusion_reference_debug.h"
#endif

/*
* @brief: feedback of IMU bias
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_bias_feedback(const float f_errstate[], NavParams_t* pz_para)
{
  pz_para->f_gyrobias[0] += f_errstate[9];
  pz_para->f_gyrobias[1] += f_errstate[10];
  pz_para->f_gyrobias[2] += f_errstate[11];

  pz_para->f_acclbias[0] += f_errstate[12];
  pz_para->f_acclbias[1] += f_errstate[13];
  pz_para->f_acclbias[2] += f_errstate[14];

  LOGI(TAG_VDR, "[bias_feedback]: bias: %f, %f, %f (deg/s), %f, %f, %f; bias_err: %lf, %lf, %lf (deg/s), %lf, %lf, %lf\n",
    pz_para->f_gyrobias[0] * RAD2DEG, pz_para->f_gyrobias[1] * RAD2DEG, pz_para->f_gyrobias[2] * RAD2DEG, pz_para->f_acclbias[0], pz_para->f_acclbias[1], pz_para->f_acclbias[2],
    f_errstate[9] * RAD2DEG, f_errstate[10] * RAD2DEG, f_errstate[11] * RAD2DEG, f_errstate[12], f_errstate[13], f_errstate[14]);
  return;
}

/*
* @brief: feedback of wheelspeed scale factor
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_wssf_feedback(const float f_errstate[], NavParams_t* pz_para)
{
  if (fusion_get_algconfig()->z_syserrtype.u_rearwheel_sf_err_esti > 0)
  {
    uint8_t u_rearwheel_sf_err_esti = fusion_get_algconfig()->z_syserrtype.u_rearwheel_sf_err_esti;
    pz_para->f_wspd_sf[2] += f_errstate[u_rearwheel_sf_err_esti];
    pz_para->f_wspd_sf[3] += f_errstate[u_rearwheel_sf_err_esti + 1];

    LOGI(TAG_VDR, "[wspd_sf_feedback]: wspd_sf: %f, %f; wspd_sf_err: %f, %f\n",
      pz_para->f_wspd_sf[2], pz_para->f_wspd_sf[3],
      f_errstate[u_rearwheel_sf_err_esti], f_errstate[u_rearwheel_sf_err_esti + 1]);
  }
  return;
}

/*
* @brief: feedback of misalignment angle
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: INS_TRUE
*/
void kf_misalign_feedback(const float f_errstate[], NavParams_t* pz_nav)
{
#if 1
  float f_misatterr[3] = { 0.0f }, f_att_misalign[3] = { 0.0f };
  float f_quat_b2v_error[4] = { 0.0f };

  f_misatterr[0] = 0.0;
  f_misatterr[1] = f_errstate[16];
  f_misatterr[2] = f_errstate[17];

  quat_rotvec2quat(f_misatterr, f_quat_b2v_error);

  quat_product_f(f_quat_b2v_error, pz_nav->f_quat_misalign);
  quat_normalize_f(f_quat_b2v_error);
  quat_copy_f(pz_nav->f_quat_misalign, f_quat_b2v_error);

  quat_quat2dcm(pz_nav->f_quat_misalign, pz_nav->f_rotmat_misalign);
  math_mattrns(&(pz_nav->f_rotmat_misalign[0][0]), 3, 3, &(pz_nav->f_rotmat_v2b[0][0]));

  tf_dcm2eluer(pz_nav->f_rotmat_misalign, pz_nav->f_mis_align);

  LOGI(TAG_VDR, "[misalign_feedback]: Mis_RPY(deg): %.3lf, %.3lf, %.3lf; Mis_RPY_err(deg): %f, %f, %f\n",
    pz_nav->f_mis_align[0] * RAD2DEG, pz_nav->f_mis_align[1] * RAD2DEG, pz_nav->f_mis_align[2] * RAD2DEG,
    f_misatterr[0] * RAD2DEG, f_misatterr[1] * RAD2DEG, f_misatterr[2] * RAD2DEG);
#endif
  return;
}

/*
* @brief: feedback of position error 
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_pos_feedback(const float f_errstate[], NavParams_t* pz_nav)
{
  float	f_delta_lat = 0.0f;
  float	f_delta_lon = 0.0f;
  float	f_theta[3];

  f_delta_lat = f_errstate[0] / (float)(pz_nav->d_rm + pz_nav->d_poslla[2]);
  f_delta_lon = f_errstate[1] / (float)(pz_nav->d_rn + pz_nav->d_poslla[2]) / (float)cos(pz_nav->d_poslla[0]);

  tf_llaerr2rotvct(pz_nav->d_poslla[0], f_delta_lat, f_delta_lon, f_theta);

  float f_quta_n[4] = { 0.0f };
  double d_latlon[2] = { 0.0 };
  double d_qtemp[4] = { 0.0 };

  f_theta[0] *= -1.0f;
  f_theta[1] *= -1.0f;
  f_theta[2] *= -1.0f;

  quat_rotvec2quat(f_theta, f_quta_n);

  d_qtemp[0] = f_quta_n[0];
  d_qtemp[1] = f_quta_n[1];
  d_qtemp[2] = f_quta_n[2];
  d_qtemp[3] = f_quta_n[3];

  quat_product_d(pz_nav->d_quat_n2e, d_qtemp);

  quat_get_posbyn2e(d_latlon, pz_nav->d_quat_n2e);

  pz_nav->d_poslla[0] = d_latlon[0];
  pz_nav->d_poslla[1] = d_latlon[1];
  pz_nav->d_poslla[2] = pz_nav->d_poslla[2] + f_errstate[2];

  LOGI(TAG_VDR, "[pos_feedback]: %.3lf,Pos_NED(m): %.9f, %.9f, %f, Pos_NED_err(m): %f, %f, %f\n", pz_nav->d_sys_timestamp,
    pz_nav->d_poslla[0] * RAD2DEG, pz_nav->d_poslla[1] * RAD2DEG, pz_nav->d_poslla[2],
    f_errstate[0], f_errstate[1], f_errstate[2]);
  return;
}

/*
* @brief: feedback of position error to ins solution at PPS time, save for tdcp measurement
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_tdcp_pos_feedback(const float f_errstate[], InertialNav_t* pz_inav)
{
  float	f_delta_lat = 0.0f;
  float	f_delta_lon = 0.0f;
  float	f_theta[3];

  f_delta_lat = f_errstate[0] / (float)(pz_inav->pz_obs->pz_bds_node->d_rm + pz_inav->pz_obs->pz_bds_node->d_lla[2]);
  f_delta_lon = f_errstate[1] / (float)(pz_inav->pz_obs->pz_bds_node->d_rn + pz_inav->pz_obs->pz_bds_node->d_lla[2]) /
    (float)cos(pz_inav->pz_obs->pz_bds_node->d_lla[0]);
  tf_llaerr2rotvct(pz_inav->pz_obs->pz_bds_node->d_lla[0], f_delta_lat, f_delta_lon, f_theta);

  float f_quta_n[4] = { 0.0f };
  double d_latlon[2] = { 0.0 };
  double d_qtemp[4] = { 0.0 };
  f_theta[0] *= -1.0f;
  f_theta[1] *= -1.0f;
  f_theta[2] *= -1.0f;
  quat_rotvec2quat(f_theta, f_quta_n);

  d_qtemp[0] = f_quta_n[0];
  d_qtemp[1] = f_quta_n[1];
  d_qtemp[2] = f_quta_n[2];
  d_qtemp[3] = f_quta_n[3];
  quat_product_d(pz_inav->pz_obs->pz_bds_node->d_qne, d_qtemp);
  quat_get_posbyn2e(d_latlon, pz_inav->pz_obs->pz_bds_node->d_qne);

  pz_inav->pz_obs->d_lla_prepps[0] = d_latlon[0];
  pz_inav->pz_obs->d_lla_prepps[1] = d_latlon[1];
  pz_inav->pz_obs->d_lla_prepps[2] = pz_inav->pz_obs->pz_bds_node->d_lla[2] + f_errstate[2];

  LOGI(TAG_VDR, "[pos_feedback_tdcp]: %.3lf,Pos_NED(m): %f, %f, %f\n", pz_inav->pz_obs->pz_bds_node->d_time,
    d_latlon[0], d_latlon[1], pz_inav->pz_obs->d_lla_prepps[2]);
  return;
}

/*
* @brief: feedback of velocity
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_vel_feedback(const float f_errstate[], NavParams_t* pz_nav)
{
  pz_nav->f_vel_ned[0] -= f_errstate[3];
  pz_nav->f_vel_ned[1] -= f_errstate[4];
  pz_nav->f_vel_ned[2] -= f_errstate[5];

  LOGI(TAG_VDR, "[vel_feedback]: Vel_NED(m/s): %.3lf, %.3lf, %.3lf; Vel_NED_Err(m/s): %f, %f, %f\n",
    pz_nav->f_vel_ned[0], pz_nav->f_vel_ned[1], pz_nav->f_vel_ned[2], f_errstate[3], f_errstate[4], f_errstate[5]);
  return;
}

/*
* @brief: feedback of attitude, ins navigation parameters
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_att_feedback(const float f_errstate[], NavParams_t* pz_nav)
{
  float f_atterr[3] = { 0.0f };
  float f_quat_e2e[4] = { 0.0f };

  f_atterr[0] = f_errstate[6];
  f_atterr[1] = f_errstate[7];
  f_atterr[2] = f_errstate[8];

  quat_rotvec2quat(f_atterr, f_quat_e2e);
  quat_product_f(f_quat_e2e, pz_nav->f_quat_b2n);
  quat_copy_f(pz_nav->f_quat_b2n, f_quat_e2e);

  if (is_izupt() == 0)
  {
    quat_copy_f(pz_nav->f_prev_quat_b2n, pz_nav->f_quat_b2n);
  }
  quat_quat2dcm(pz_nav->f_quat_b2n, pz_nav->f_rotmat_b2n);
  tf_dcm2eluer(pz_nav->f_rotmat_b2n, pz_nav->f_att_rph);
  LOGI(TAG_VDR, "[att_feedback]: Att_RPY(deg): %.3lf, %.3lf, %.3lf; Att_RPY_err(deg): %f, %f, %f\r\n",
    pz_nav->f_att_rph[0] * RAD2DEG, pz_nav->f_att_rph[1] * RAD2DEG, pz_nav->f_att_rph[2] * RAD2DEG,
    f_atterr[0] * RAD2DEG, f_atterr[1] * RAD2DEG, f_atterr[2] * RAD2DEG);
  return;
}

/*
* @brief: feedback of receiver clock error (m)
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_clk_feedback(const float f_errstate[], NavParams_t* pz_nav, InertialNav_t* pz_inav)
{ 
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  if (pz_obs->q_update_type == (uint32_t)SMT_PR_DR) /* psr used */
  { 
    if (fusion_get_algconfig()->z_syserrtype.u_clock_err_esti > 0)
    {
      uint8_t u_clock_err_esti = fusion_get_algconfig()->z_syserrtype.u_clock_err_esti;

      pz_nav->f_clk_err[0] += f_errstate[u_clock_err_esti];
      pz_nav->f_clk_err[1] += f_errstate[u_clock_err_esti + 1];
      pz_nav->f_clk_err[2] += f_errstate[u_clock_err_esti + 2];

      LOGI(TAG_VDR, "[clk_feedback]: ClkDiff(m): %.3lf, %.3lf, %.3lf; ClkDiff_Err(m): %f, %f, %f\r\n",
        pz_nav->f_clk_err[0], pz_nav->f_clk_err[1], pz_nav->f_clk_err[2], 
        f_errstate[u_clock_err_esti], f_errstate[u_clock_err_esti + 1], f_errstate[u_clock_err_esti + 2]);
    }        
  }
  return;
}

/*
* @brief: feedback of receiver clock diff error (m)
* @input: f_errstate - error correct, pz_para - ins navigation parameters
* @output: None
* @return: None
*/
void kf_clkdiff_feedback(const float f_errstate[], NavParams_t* pz_nav, InertialNav_t* pz_inav)
{
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  if (pz_obs->q_update_type == (uint32_t)SMT_CP) /* CP used */
  {
    if (fusion_get_algconfig()->z_syserrtype.u_clock_diff_err_esti > 0)
    {
      uint8_t u_clock_diff_err_esti = fusion_get_algconfig()->z_syserrtype.u_clock_diff_err_esti;

      pz_nav->f_clkdiff_err += f_errstate[u_clock_diff_err_esti];

      LOGI(TAG_VDR, "[clkdiff_feedback]: ClkDiff(m): %.3lf; ClkDiff_Err(m): %f\r\n",
        pz_nav->f_clkdiff_err, f_errstate[u_clock_diff_err_esti]);
    }
  }
  return;
}

void kf_la_imu2gnss_feedback(const float f_errstate[])
{
  if (fusion_get_algconfig()->z_syserrtype.u_la_imu2gnss_err_esti > 0)
  {
    uint8_t u_la_imu2gnss_err_esti = fusion_get_algconfig()->z_syserrtype.u_la_imu2gnss_err_esti;
    float* pf_imu2gnss;
    pf_imu2gnss = lvrm_get_imu2gnss();

    pf_imu2gnss[0] -= f_errstate[u_la_imu2gnss_err_esti];
    pf_imu2gnss[1] -= f_errstate[u_la_imu2gnss_err_esti + 1];
    pf_imu2gnss[2] -= f_errstate[u_la_imu2gnss_err_esti + 2];

    LOGI(TAG_VDR, "[la_i2g_feedback]: La_NED(m/s): %.3lf, %.3lf, %.3lf; La_NED_Err(m/s): %f, %f, %f\n",
      pf_imu2gnss[0], pf_imu2gnss[1], pf_imu2gnss[2],
      f_errstate[u_la_imu2gnss_err_esti], f_errstate[u_la_imu2gnss_err_esti + 1], f_errstate[u_la_imu2gnss_err_esti + 2]);
  }
  return;
}

#ifdef LA_IMU2REARMID_ERR_ESTI
void kf_la_imu2rearmid_feedback(const float f_errstate[])
{
  float* pf_la_imu2rearmid = lvrm_get_imu2rearmid();

  pf_la_imu2rearmid[0] -= f_errstate[LA_IMU2REARMID_ERR_ESTI];
  pf_la_imu2rearmid[1] -= f_errstate[LA_IMU2REARMID_ERR_ESTI + 1];
  pf_la_imu2rearmid[2] -= f_errstate[LA_IMU2REARMID_ERR_ESTI + 2];

  LOGI(TAG_VDR, "[la_i2rearmid_feedback]: La_NED(m/s): %.3lf, %.3lf, %.3lf; La_NED_Err(m/s): %f, %f, %f\n",
    pf_la_imu2rearmid[0], pf_la_imu2rearmid[1], pf_la_imu2rearmid[2],
    f_errstate[LA_IMU2REARMID_ERR_ESTI], f_errstate[LA_IMU2REARMID_ERR_ESTI + 1], f_errstate[LA_IMU2REARMID_ERR_ESTI + 2]);

  return;
}
#endif

#ifdef WHEELBASE_ERR_ESTI
void kf_la_wheelbase_feedback(const float f_errstate[])
{
  float* pf_rear2rear = lvrm_get_rear2rear();

  *pf_rear2rear -= f_errstate[WHEELBASE_ERR_ESTI];

  LOGI(TAG_VDR, "[la_wheelbase_feedback]: wheelbase(m/s): %.3lf; wheelbase_Err(m/s): %f\r\n",
    *pf_rear2rear, f_errstate[WHEELBASE_ERR_ESTI]);
  return;
}
#endif

void kf_misalign_imu2dualant_feedback(const float f_errstate[], InertialNav_t* pz_inav)
{
	SyncObsNav_t* pz_obs = pz_inav->pz_obs;
	if (pz_obs->q_update_type == (uint32_t)SMT_DUALANT) /* dual ant used */
	{
    if (fusion_get_algconfig()->z_syserrtype.u_mis_dualant_err_esti > 0)
    {
      uint8_t u_mis_dualant_err_esti = fusion_get_algconfig()->z_syserrtype.u_mis_dualant_err_esti;

#if 1
      float f_misatterr[3] = { 0.0f }, f_att_misalign[3] = { 0.0f };
      float f_rotmat_b2g_error[3][3] = { 0.0f };
      float* f_att_mis_b2g = mis_get_imu2dualant();
      float f_rotmat_b2g_old[3][3], f_rotmat_b2g[3][3];
      tf_euler2dcm(f_att_mis_b2g, f_rotmat_b2g_old);

      f_misatterr[0] = 0.0f;
      f_misatterr[1] = f_errstate[u_mis_dualant_err_esti];
      f_misatterr[2] = f_errstate[u_mis_dualant_err_esti + 1];

      math_skewsym(f_misatterr, f_rotmat_b2g_error);
      f_rotmat_b2g_error[0][0] = 1.0f;
      f_rotmat_b2g_error[1][1] = 1.0f;
      f_rotmat_b2g_error[2][2] = 1.0f;

      math_matmul(&(f_rotmat_b2g_error[0][0]), &(f_rotmat_b2g_old[0][0]), 3, 3, 3, &(f_rotmat_b2g[0][0]));
      tf_dcm2eluer(f_rotmat_b2g, f_att_mis_b2g);

#else
      float f_misatterr[3] = { 0.0f }, f_att_misalign[3] = { 0.0f };
      float f_quat_b2g_error[4] = { 0.0f };
      float f_rotmat_b2g_error[3][3] = { 0.0f };

      float* f_att_mis_b2g = mis_get_imu2dualant();
      float f_quat_b2g_old[4], f_quat_b2g_old2[4], f_quat_b2g[4], f_rotmat_b2g[3][3];
      quat_euler2quat(f_att_mis_b2g, f_quat_b2g_old);
      quat_rotvec2quat(f_att_mis_b2g, f_quat_b2g_old2);

      f_misatterr[0] = 0.0f;
      f_misatterr[1] = f_errstate[MISANGLE_IMU2DUALANT_ERR_ESTI];
      f_misatterr[2] = f_errstate[MISANGLE_IMU2DUALANT_ERR_ESTI + 1];

      quat_rotvec2quat(f_misatterr, f_quat_b2g_error);

      quat_product_f(f_quat_b2g_error, f_quat_b2g_old);
      quat_normalize_f(f_quat_b2g_error);
      quat_copy_f(f_quat_b2g, f_quat_b2g_error);

      quat_quat2dcm(f_quat_b2g, f_rotmat_b2g);
      tf_dcm2eluer(f_rotmat_b2g, f_att_mis_b2g);
#endif

      LOGI(TAG_VDR, "[mis_imu2dualant_feedback]: mis_b2g(deg): %.3lf,%.3lf,%.3lf; mis_b2g_Err(deg): %.3lf,%.3lf\n",
        f_att_mis_b2g[0] * RAD2DEG, f_att_mis_b2g[1] * RAD2DEG, f_att_mis_b2g[2] * RAD2DEG,
        f_errstate[u_mis_dualant_err_esti] * RAD2DEG, f_errstate[u_mis_dualant_err_esti + 1] * RAD2DEG);
    }
	}
  return;
}

void kf_feedback_procedure(const float* pf_errstate,InertialNav_t* pz_inav,NavParams_t* pz_para)
{
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

  kf_bias_feedback(pf_errstate, pz_para);

  kf_wssf_feedback(pf_errstate, pz_para);

  kf_misalign_feedback(pf_errstate, pz_para);

  kf_pos_feedback(pf_errstate, pz_para);

  kf_vel_feedback(pf_errstate, pz_para);

  kf_att_feedback(pf_errstate, pz_para);

  kf_clk_feedback(pf_errstate, pz_para, pz_inav);

  kf_clkdiff_feedback(pf_errstate, pz_para, pz_inav);

  kf_la_imu2gnss_feedback(pf_errstate);

  kf_misalign_imu2dualant_feedback(pf_errstate, pz_inav);

#ifdef LA_IMU2REARMID_ERR_ESTI
  // kf_la_imu2rearmid_feedback(pf_errstate);
#endif
#ifdef WHEELBASE_ERR_ESTI
  // kf_la_wheelbase_feedback(pf_errstate);
#endif
#ifdef INS_DEBUG
  double error[12];
  int ret = 0;

  ret = ref_CalInsAntError(pz_para, error);
  if (ret > 0)
  {
    LOGE(TAG_VDR, "$INS_ERROR %10.3f %d pos %10.3f %10.3f %10.3f vel_n %7.3f %7.3f %7.3f vel_v %7.3f %7.3f %7.3f att %7.3f %7.3f %7.3f\n",
      pz_para->d_sys_timestamp, pz_inav->pz_obs->q_update_type,
      error[0], error[1], error[2], error[3], error[4], error[5],
      error[6], error[7], error[8], error[9], error[10], error[11]);
  }
#endif
}

/* deal second thread message */
void kf_feedback_handle(uint8_t isSuccess, SysErrModel_t* pz_modelTmp)
{
  NavParams_t* pz_para = fusion_get_navpara();
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  SysErrModel_t* pz_model = fusion_get_sysmodel();

  if (isSuccess)
  {
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_PR_DR;
    kf_feedback_procedure(pz_modelTmp->f_errstate, pz_inav, pz_para);
    memcpy(pz_model->f_syscovmat, pz_modelTmp->f_syscovmat, pz_model->u_sysdim * pz_model->u_sysdim * sizeof(float));
    pz_inav->pz_obs->d_prebds_update_time = pz_inav->pz_obs->d_imu_t;
  }
  else
  {
    LOGE(TAG_VDR, "[PRDR tc update fail]: %.3f\n", pz_inav->pz_obs->d_imu_t);
    fusion_reset();
  }
}