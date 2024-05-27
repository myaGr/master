/**@file        fusion_proc.c
 * @brief		fusion proc file
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

#include "fusion_proc.h"
#include "fusion_if.h"
#include "fusion_global.h"
#include "fusion_config.h"
#include "fusion_alignment.h"
#include "fusion_mech.h"
#include "fusion_kf.h"
#include "fusion_err_model.h"
#include "fusion_axis_detect.h"
#include "fusion_ellip_para.h"
#include "fusion_log.h"
#include "fusion_zupt.h"
#include "fusion_nhc.h"
#include "fusion_wheel.h"
#include "fusion_math.h"
#include "fusion_transform.h"
#include "fusion_quat.h"
#include "fusion_meas_qc.h"
#include "fusion_gnss.h"
#include "fusion_feedback.h"
#include "fusion_tc.h"
#include "fusion_integrity.h"
#include <string.h>
#include <math.h>
#include "mw_log.h"
#include "mw_alloc.h"

#ifdef TIME_COST_COUNT
#include <windows.h>
#endif

#define DISABLE_AXIS_DETECT /* close axis detection, set default axis as FRD, set default mis_b2v as 0, for such as short data or for save time */

#define MIS_BUF_SIZE 15 * 200	

static NavParams_t gz_navpara;
static InertialNav_t gz_inertial;
static SyncQueue_t   gz_sync_queue;
static SyncObsNav_t gz_obs_nav;

#ifdef OPEN_INS_INTEGRITY
static EkfPosFeedback_t gz_ekf_dpos;

EkfPosFeedback_t* fusion_get_ekf_dpos(void)
{
  return &gz_ekf_dpos;
}
#endif

#ifdef TIME_COST_COUNT
typedef struct
{
  LARGE_INTEGER  m_nFreq;
  LARGE_INTEGER  m_nBeginTime;
  LARGE_INTEGER  m_nEndTime;
} AsensingFusionTimeCost_t;
AsensingFusionTimeCost_t gz_timecost;
#endif

void fusion_navsys_init(void)
{
  memset(&gz_navpara, 0, sizeof(gz_navpara));
  memset(&gz_inertial, 0, sizeof(gz_inertial));
  gz_inertial.u_ins_status = INS_STATUS_INITIALIZATION;
  gz_inertial.d_last_imu_time = INITIAL_TIME;
  ellip_para_initial();
  flx_axis_init();
}

void fusion_navsys_partial_init(void)
{
  float f_gyrobias_tmp[3];
  float f_acclbias_tmp[3];
  float f_wspd_sf_tmp[4];
  float f_odospd_sf_tmp;
  float f_quat_misalign_tmp[4];
  float f_rotmat_misalign_tmp[3][3];
  float f_rotmat_v2b_tmp[3][3];
  float f_mis_align_tmp[3];
  memcpy(f_gyrobias_tmp, gz_navpara.f_gyrobias, sizeof(gz_navpara.f_gyrobias));
  memcpy(f_acclbias_tmp, gz_navpara.f_acclbias, sizeof(gz_navpara.f_acclbias));
  memcpy(f_wspd_sf_tmp, gz_navpara.f_wspd_sf, sizeof(gz_navpara.f_wspd_sf));
  f_odospd_sf_tmp = gz_navpara.f_odospd_sf;
  memcpy(f_quat_misalign_tmp, gz_navpara.f_quat_misalign, sizeof(gz_navpara.f_quat_misalign));
  memcpy(f_rotmat_misalign_tmp, gz_navpara.f_rotmat_misalign, sizeof(gz_navpara.f_rotmat_misalign));
  memcpy(f_rotmat_v2b_tmp, gz_navpara.f_rotmat_v2b, sizeof(gz_navpara.f_rotmat_v2b));
  memcpy(f_mis_align_tmp, gz_navpara.f_mis_align, sizeof(gz_navpara.f_mis_align));

  memset(&gz_navpara, 0, sizeof(gz_navpara));

  memcpy(gz_navpara.f_gyrobias, f_gyrobias_tmp, sizeof(gz_navpara.f_gyrobias));
  memcpy(gz_navpara.f_acclbias, f_acclbias_tmp, sizeof(gz_navpara.f_acclbias));
  memcpy(gz_navpara.f_wspd_sf, f_wspd_sf_tmp, sizeof(gz_navpara.f_wspd_sf));
  gz_navpara.f_odospd_sf = f_odospd_sf_tmp;
  memcpy(gz_navpara.f_quat_misalign, f_quat_misalign_tmp, sizeof(gz_navpara.f_quat_misalign));
  memcpy(gz_navpara.f_rotmat_misalign, f_rotmat_misalign_tmp, sizeof(gz_navpara.f_rotmat_misalign));
  memcpy(gz_navpara.f_rotmat_v2b, f_rotmat_v2b_tmp, sizeof(gz_navpara.f_rotmat_v2b));
  memcpy(gz_navpara.f_mis_align, f_mis_align_tmp, sizeof(gz_navpara.f_mis_align));

  memset(&gz_inertial, 0, sizeof(gz_inertial));
  gz_inertial.u_ins_status = INS_STATUS_INMOTIONALIGN;
  gz_inertial.d_last_imu_time = INITIAL_TIME;
  gz_inertial.u_inspl_init_flag = 0;
  ellip_para_initial();
  // flx_axis_init();	
}

uint8_t is_izupt(void)
{
#ifndef FORCE_INIT
  return (gz_navpara.u_izupt_counter >= 2u);
#else
  return 0;
#endif
}

uint8_t* is_gzupt(void)
{
  return &gz_navpara.u_gzupt;
}

void fusion_reset(void)
{

}

/*
* @brief: get NavParameter struct
* @input: None
* @output: None
* @return: navigation struct
*/
NavParams_t* fusion_get_navpara(void)
{
  return &gz_navpara;
}

/*
* @brief: get InertialNav struct
* @input: None
* @output: None
* @return:InertialNav struct
*/
InertialNav_t* fusion_get_inertial_nav(void)
{
  return &gz_inertial;
}

/*
* @brief: Check whether the navigation status is valid
* @input: None
* @output: None
* @return: fusion valid flag
*/
uint8_t is_fusion_valid(void)
{
  return (uint8_t)((gz_inertial.u_ins_status >= INS_STATUS_NAVIGATION) &&
    (gz_inertial.u_init_nav == INS_TRUE) && (gz_inertial.u_ins_timeout == INS_FALSE));
}

/*
* @brief: Detect the independent working time of INS
* @input: pd_meas - imu data; pz_inav - InertialNav struct
* @output: None
* @return: INS_TRUE or INS_FALSE
*/
static uint8_t fusion_detect_timeout(double pd_meas[], InertialNav_t* pz_inav)
{
  uint8_t u_ret = INS_FALSE;

  float f_timeout = pz_inav->pz_config->f_timeout > 0 ? pz_inav->pz_config->f_timeout : 300;

  if (f_timeout > 0)
  {
    if (pd_meas[0] > 0.0)
    {
      if (pd_meas[1] > pz_inav->d_timeout_beginning + f_timeout + pz_inav->d_timeout_zupttime)
      {
        u_ret = INS_TRUE;
      }
    }
    else if (pd_meas[0] < 0.0)
    {
      if (pz_inav->u_last_update_type != NAVI_UT_NONE)
      {
        pz_inav->d_timeout_beginning = pd_meas[1];
        pz_inav->d_timeout_zupttime = 0.0;
      }
    }
    else
    {
      u_ret = INS_FALSE;
    }
  }
  else
  {
    pz_inav->d_timeout_beginning = pd_meas[1];
    pz_inav->d_timeout_zupttime = 0.0;
  }
  return u_ret;
}

static void pva_reinitial(double d_pos[3], float f_vel[3], float f_att[3], NavParams_t* pz_para)
{
  memcpy(pz_para->d_poslla, d_pos, sizeof(pz_para->d_poslla));
  memcpy(pz_para->f_vel_ned, f_vel, sizeof(pz_para->f_vel_ned));
  memcpy(pz_para->f_att_rph, f_att, sizeof(pz_para->f_att_rph));

  tf_euler2dcm(pz_para->f_att_rph, pz_para->f_rotmat_b2n);
  quat_euler2quat(pz_para->f_att_rph, pz_para->f_quat_b2n);
  quat_copy_f(pz_para->f_prev_quat_b2n, pz_para->f_quat_b2n);
  quat_get_qn2e(pz_para->d_poslla[0], pz_para->d_poslla[1], pz_para->d_quat_n2e);

  double dRM, dRN;
  float GravityVec[3];

  ellip_calculateMN(pz_para->d_poslla[0], &dRM, &dRN);

  pz_para->d_rm = dRM;
  pz_para->d_rn = dRN;

  GravityVec[0] = 0.f;
  GravityVec[1] = 0.f;
  GravityVec[2] = ellip_get_gravity(pz_para->d_poslla);
  memcpy(pz_para->f_gravity_vct, GravityVec, sizeof(pz_para->f_gravity_vct));
  tf_get_wien(pz_para->d_poslla, pz_para->f_omega_ie);
}

static void zupt_keep_pva(InertialNav_t* pz_inav)
{
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  NavParams_t* pz_para = fusion_get_navpara();

  if (is_izupt() == INS_TRUE)
  {
    if (pz_gnss->e_posflag != TYPE_POS_FLAG_RTK_FIX)
    {
      if (INS_FALSE == pz_inav->u_keep_pos)
      {
        for (int i = 0; i < 3; i++)
        {
          pz_inav->d_zupt_pos[i] = pz_para->d_poslla[i];
          pz_inav->f_zupt_vel[i] = 0.f;
          pz_inav->f_zupt_att[i] = pz_para->f_att_rph[i];
        }
        LOGI(TAG_VDR, "[zupt:poskeep]: %.3lf\n", pz_para->d_sys_timestamp);
        pz_inav->u_keep_pos = INS_TRUE;
      }
    }
    else
    {
      pz_inav->u_keep_pos = INS_FALSE;
    }
  }
  else
  {
    if (pz_inav->u_keep_pos == INS_TRUE)
    {
      pva_reinitial(pz_inav->d_zupt_pos, pz_inav->f_zupt_vel, pz_inav->f_zupt_att, pz_para);
      pz_inav->d_zupt_reinit_time = pz_para->d_sys_timestamp;
      LOGI(TAG_VDR, "[zupt:pva_reinitial]: %.3lf\n", pz_para->d_sys_timestamp);
    }
    pz_inav->u_keep_pos = INS_FALSE;
  }
}

int8_t mechan_get_node(MechanNode_t* pz_node)
{
  int8_t s_val = 1;
  if (gz_sync_queue.u_write_index != gz_sync_queue.u_read_index)
  {
    gz_sync_queue.u_read_index = (gz_sync_queue.u_read_index) & (SYNC_QUEUE_SIZE - 1);
    memcpy(pz_node, &gz_sync_queue.z_nodes[gz_sync_queue.u_read_index], sizeof(MechanNode_t));
    gz_sync_queue.u_read_index++;
  }
  else
  {
    s_val = -1;
  }
  return s_val;
}

#if 0
void mechan_fill_node(MechanNode_t* pz_node)
{
  if (gz_sync_queue.u_write_index != (gz_sync_queue.u_read_index ^ SYNC_QUEUE_SIZE))
  {
    gz_sync_queue.u_write_index = (gz_sync_queue.u_write_index) & (SYNC_QUEUE_SIZE - 1);
    memcpy(&gz_sync_queue.z_nodes[gz_sync_queue.u_write_index], pz_node, sizeof(MechanNode_t));
    // gz_imu_buf.u_write_index = (++gz_imu_buf.u_write_index) & (IMUS_BUF_SIZE - 1);
    gz_sync_queue.u_write_index++;
  }
}
#else
void mechan_fill_node(SyncQueue_t* pz_queue)
{
  MechanNode_t* pz_ncur = &(pz_queue->z_nmech);
  MechanNode_t* pz_nsrc = &(pz_queue->z_nodes[1]);
  MechanNode_t* pz_ndst = &(pz_queue->z_nodes[0]);

  if (pz_queue->u_count == SYNC_QUEUE_SIZE)
  {
    MechanNode_t* pz_nlst = &(pz_queue->z_nodes[SYNC_QUEUE_SIZE - 1]);
    unsigned int num = (unsigned int)SYNC_QUEUE_SIZE - 1u;
    memmove(pz_ndst, pz_nsrc, num * sizeof(MechanNode_t));
    memcpy(pz_nlst, pz_ncur, sizeof(MechanNode_t));
  }
  else if (pz_queue->u_count < SYNC_QUEUE_SIZE)
  {
    MechanNode_t* pz_nlst = &(pz_queue->z_nodes[pz_queue->u_count]);
    memcpy(pz_nlst, pz_ncur, sizeof(MechanNode_t));
    pz_queue->u_count++;
  }
  else
  {
    pz_queue->u_count = 0;
  }
}
#endif

uint8_t mechan_find_node(double d_target_time, SyncQueue_t* pz_queue)
{
  float f_time_diff;
  float f_mini_dt = 999.99f;
  uint8_t index = (pz_queue->u_read_index) & (SYNC_QUEUE_SIZE - 1);

  for (uint8_t i = 0; i < SYNC_QUEUE_SIZE; i++)
  {
    f_time_diff = (float)fabs(d_target_time - pz_queue->z_nodes[i].d_time);

    if (f_time_diff < 1e-6)
    {
      index = i;
      break;
    }
    else if (f_time_diff < f_mini_dt)
    {
      f_mini_dt = f_time_diff;
      index = i;
    }
  }
  return index;
}

void ins_get_update_type(InertialNav_t* pz_inav)
{
  pz_inav->u_ins_update_mask = TYPE_MASK_UPDATE_NONE;

  if (is_izupt() > 0)
  {
    pz_inav->u_ins_update_mask = TYPE_MASK_ZUPT_UPDATE;
  }
  else
  {
    NavConfig_t* pz_config = fusion_get_navconfig();

    if (pz_config->u_whspd_enable == TYPE_WHEEL_AIDING_NHC)
    {
      pz_inav->u_ins_update_mask = TYPE_MASK_NHC_UPDATE;
    }
    else if (pz_config->u_whspd_enable == TYPE_WHEEL_AIDING_ODO)
    {
      pz_inav->u_ins_update_mask = TYPE_MASK_ODO_UPDATE;
    }
    else if (pz_config->u_whspd_enable == TYPE_WHEEL_AIDING_TWO_REARS ||
      pz_config->u_whspd_enable == TYPE_WHEEL_AIDING_FOUR_WHLS)
    {
      pz_inav->u_ins_update_mask = TYPE_MASK_WSPD_UPDATE;
    }
    else if (pz_config->u_whspd_enable == TYPE_WHEEL_AIDING_WHLS_STEER)
    {
      pz_inav->u_ins_update_mask = TYPE_MASK_WSPD_STEER_UPDATE;
    }
    else
    {
      pz_inav->u_ins_update_mask = TYPE_MASK_NHC_UPDATE;
    }
  }
}

#ifdef MISANGLE_IMU2DUALANT_ERR_ESTI
uint8_t dualant_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_TRUE;	
  float f_kf_meas[3];
  float f_rotmat_b2n[3][3], f_rotmat_n2b[3][3], f_rotmat_n2g[3][3];
  uint8_t* dualant_zupt_update = is_gzupt();
  NavConfig_t* pNavConfig = pz_inav->pz_config;
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  MechanNode_t* pz_node = pz_inav->pz_obs->pz_bds_node;
  SysErrModel_t* pz_model = pz_inav->pz_model;
  float f_z_meas_tmp[3] = { 0 };
  float f_sysmeasmat_tmp[3][SYS_DIM] = { 0 };
  float f_measnoise_tmp[3][3] = { 0 };
  
  float* f_att_mis_b2g = mis_get_imu2dualant();
  float f_rotmat_mis_b2g[3][3];
  tf_euler2dcm(f_att_mis_b2g, f_rotmat_mis_b2g);
  
  quat_quat2dcm(pz_node->f_qbn, f_rotmat_b2n);
  math_mattrns(&(f_rotmat_b2n[0][0]), 3, 3, &(f_rotmat_n2b[0][0]));
  math_matmul(&(f_rotmat_mis_b2g[0][0]), &(f_rotmat_n2b[0][0]), 3, 3, 3, &(f_rotmat_n2g[0][0]));
  
  float f_sin_heading = sinf(pz_gnss->f_heading_deg * DEG2RAD);
  float f_cos_heading = cosf(pz_gnss->f_heading_deg * DEG2RAD);
  float f_sin_pitch = sinf(pz_gnss->f_pitch_deg * DEG2RAD);
  float f_cos_pitch = cosf(pz_gnss->f_pitch_deg * DEG2RAD);

  f_kf_meas[0] = f_rotmat_n2g[0][0] - f_cos_pitch * f_cos_heading;
  f_kf_meas[1] = f_rotmat_n2g[0][1] - f_cos_pitch * f_sin_heading;
  f_kf_meas[2] = f_rotmat_n2g[0][2] + f_sin_pitch;

  if (is_izupt() == 1 && (pd_meas[1] - pz_inav->pz_qc->d_pre_good_bds_time) > 10.0 &&
    pz_inav->pz_qc->d_pre_good_bds_time >= 0.0 && pz_inav->pz_obs->u_posflag != TYPE_POS_FLAG_RTK_FIX &&
    pd_meas[19] < 35.0f)
  {
    f_z_meas_tmp[0] = f_kf_meas[0] > 10.f ? 3.f : f_kf_meas[0];
    f_z_meas_tmp[1] = f_kf_meas[1] > 10.f ? 3.f : f_kf_meas[1];
    f_z_meas_tmp[2] = f_kf_meas[2] > 10.f ? 3.f : f_kf_meas[2];
    *dualant_zupt_update = 1;
  }
  else
  {
    f_z_meas_tmp[0] = f_kf_meas[0];
    f_z_meas_tmp[1] = f_kf_meas[1];
    f_z_meas_tmp[2] = f_kf_meas[2];
    *dualant_zupt_update = 0;
  }

#if 1
  f_sysmeasmat_tmp[0][ATT_ERR_ESTI] = 0.0;
  f_sysmeasmat_tmp[0][ATT_ERR_ESTI + 1] = -f_rotmat_n2g[0][2];
  f_sysmeasmat_tmp[0][ATT_ERR_ESTI + 2] = f_rotmat_n2g[0][1];
  f_sysmeasmat_tmp[1][ATT_ERR_ESTI] = f_rotmat_n2g[0][2];
  f_sysmeasmat_tmp[1][ATT_ERR_ESTI + 1] = 0.0;
  f_sysmeasmat_tmp[1][ATT_ERR_ESTI + 2] = -f_rotmat_n2g[0][0];
  f_sysmeasmat_tmp[2][ATT_ERR_ESTI] = -f_rotmat_n2g[0][1];
  f_sysmeasmat_tmp[2][ATT_ERR_ESTI + 1] = f_rotmat_n2g[0][0];
  f_sysmeasmat_tmp[2][ATT_ERR_ESTI + 2] = 0.0;

  f_sysmeasmat_tmp[0][MISANGLE_IMU2DUALANT_ERR_ESTI] = -f_rotmat_n2g[2][0];
  f_sysmeasmat_tmp[0][MISANGLE_IMU2DUALANT_ERR_ESTI + 1] = f_rotmat_n2g[1][0];
  f_sysmeasmat_tmp[1][MISANGLE_IMU2DUALANT_ERR_ESTI] = -f_rotmat_n2g[2][1];
  f_sysmeasmat_tmp[1][MISANGLE_IMU2DUALANT_ERR_ESTI + 1] = f_rotmat_n2g[1][1];
  f_sysmeasmat_tmp[2][MISANGLE_IMU2DUALANT_ERR_ESTI] = -f_rotmat_n2g[2][2];
  f_sysmeasmat_tmp[2][MISANGLE_IMU2DUALANT_ERR_ESTI + 1] = f_rotmat_n2g[1][2];

  float f_heading_std = pz_gnss->f_heading_std * DEG2RAD;
  float f_pitch_std = pz_gnss->f_pitch_std * DEG2RAD;
  f_measnoise_tmp[0][0] = SQR(f_sin_pitch * f_cos_heading * f_pitch_std + f_cos_pitch * f_sin_heading * f_heading_std);
  f_measnoise_tmp[1][1] = SQR(f_sin_pitch * f_sin_heading * f_pitch_std + f_cos_pitch * f_cos_heading * f_heading_std);
  f_measnoise_tmp[2][2] = SQR(f_cos_pitch * f_pitch_std);
#else
  kf_set_update_type(SMT_DUALANT, pz_inav->pz_model);
  kf_update_measurement_matrix(SYS_DIM, *dim, pz_inav->pz_model);
  kf_update_measurement_noise(pd_meas, pz_inav->pz_model);
#endif
  
#if 1
  SysErrModel_t pz_model_tmp = {0};
  memcpy(pz_model_tmp.f_sysmeasmat, f_sysmeasmat_tmp, sizeof(f_sysmeasmat_tmp));
  for (uint8_t i = 0; i < 3; i++)
  {
    pz_model_tmp.f_measnoise[i][i] = f_measnoise_tmp[i][i];
  }
  memcpy(pz_model_tmp.f_syscovmat, pz_model->f_syscovmat, sizeof(pz_model->f_syscovmat));

  float lambda = kf_calcu_lambda(f_z_meas_tmp, SYS_DIM, 3, &pz_model_tmp);	

  LOGI(TAG_VDR, "[dualant_meas]:%.3lf, lambda:%.3f, pu_mdim:%d, z:%.3f,%.3f,%.3f, noise:%.5f,%.5f,%.5f\n", pz_node->d_time,
    lambda, *pu_mdim, f_z_meas_tmp[0], f_z_meas_tmp[1], f_z_meas_tmp[2],
    sqrt(f_measnoise_tmp[0][0]), sqrt(f_measnoise_tmp[1][1]), sqrt(f_measnoise_tmp[2][2]));

  float f_misb2g_heading_std = sqrtf(pz_model->f_syscovmat[MISANGLE_IMU2DUALANT_ERR_ESTI + 1][MISANGLE_IMU2DUALANT_ERR_ESTI + 1]) * RAD2DEG;
  
  if (lambda > 20.f && f_misb2g_heading_std < 0.01)
  {
    u_val = INS_FALSE;
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
    LOGI(TAG_VDR, "[dualant_meas]: lambda too large\n");
  }
  else
  {
    memcpy(f_z_meas + *pu_mdim, f_z_meas_tmp, sizeof(f_z_meas_tmp));
    memcpy(&pz_model->f_sysmeasmat[*pu_mdim], f_sysmeasmat_tmp, sizeof(f_sysmeasmat_tmp));
    for (uint8_t i = 0; i < 3; i++)
    {
      pz_model->f_measnoise[*pu_mdim + i][*pu_mdim + i] = f_measnoise_tmp[i][i];
    }
    *pu_mdim += 3;		
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_DUALANT;
  }
#else
  memcpy(f_z_meas + *pu_mdim, f_z_meas_tmp, sizeof(f_z_meas_tmp));
  memcpy(&pz_model->f_sysmeasmat[*pu_mdim], f_sysmeasmat_tmp, sizeof(f_sysmeasmat_tmp));
  for (uint8_t i = 0; i < 3; i++)
  {
    pz_model->f_measnoise[*pu_mdim + i][*pu_mdim + i] = f_measnoise_tmp[i][i];
  }

  *pu_mdim += 3;
  pz_inav->pz_obs->q_update_type = (uint32_t)SMT_DUALANT;

  LOGI(TAG_VDR, "[dualant_meas]:%.3lf, pu_mdim:%d, z:%.3f,%.3f,%.3f, noise:%.5f,%.5f,%.5f\n", pz_node->d_time,
    *pu_mdim, f_z_meas_tmp[0], f_z_meas_tmp[1], f_z_meas_tmp[2],
    sqrt(f_measnoise_tmp[0][0]), sqrt(f_measnoise_tmp[1][1]), sqrt(f_measnoise_tmp[2][2]));
#endif

#if 0
  float f_att_mis_g2n[3], f_att_mis_b2n[3], f_att_mis_n2g[3], f_rotmat_g2n[3][3];
  tf_dcm2eluer(f_rotmat_b2n, f_att_mis_b2n);

  math_mattrns(&(f_rotmat_n2g[0][0]), 3, 3, &(f_rotmat_g2n[0][0]));
  tf_dcm2eluer(f_rotmat_g2n, f_att_mis_g2n); // f_att_mis_g2n != -f_att_mis_n2g

  LOGI(TAG_VDR, "[dualant_meas]:%.3lf, dualant:%.3f,%.3f, g2n:%.3f,%.3f,%.3f, b2n:%.3f,%.3f,%.3f\n", pz_node->d_time,
    pz_gnss->f_pitch_deg, pz_gnss->f_heading_deg, f_att_mis_g2n[0] * RAD2DEG, f_att_mis_g2n[1] * RAD2DEG, f_att_mis_g2n[2]*RAD2DEG,
    f_att_mis_b2n[0] * RAD2DEG, f_att_mis_b2n[1] * RAD2DEG, f_att_mis_b2n[2] * RAD2DEG);
#endif
  return u_val;	
}

int8_t integrated_dualant_update(double pd_meas[], float* pf_z_meas, InertialNav_t* pz_inav, uint8_t* dim)
{
  int8_t s_val = INS_TRUE;
  uint8_t u_is_ready = INS_FALSE;

  u_is_ready = dualant_meas_handler(pd_meas, pf_z_meas, dim, pz_inav);

#if 0
  if (u_is_ready == INS_TRUE && u_mdim > 0)
  {
    if (kf_update(f_z_meas, SYS_DIM, u_mdim, pz_inav->pz_model) != INS_TRUE)
    {
      s_val = INS_FALSE;
      pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
      LOGE(TAG_VDR, "%.3f, DualAnt KF Failed \n", pd_meas[1]);
    }
    else
    {
      LOGI(TAG_VDR, "[dualant_update]:%.3lf,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", pd_meas[1], pz_inav->pz_model->f_errstate[0], pz_inav->pz_model->f_errstate[1], pz_inav->pz_model->f_errstate[2],
        pz_inav->pz_model->f_errstate[3], pz_inav->pz_model->f_errstate[4], pz_inav->pz_model->f_errstate[5],
        pz_inav->pz_model->f_errstate[6] * RAD2DEG, pz_inav->pz_model->f_errstate[7] * RAD2DEG, pz_inav->pz_model->f_errstate[8] * RAD2DEG);
      LOGI(TAG_VDR, "[dualant_update]:%.3lf,CovP:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", pd_meas[1],
        sqrt(pz_inav->pz_model->f_syscovmat[0][0]), sqrt(pz_inav->pz_model->f_syscovmat[1][1]), sqrt(pz_inav->pz_model->f_syscovmat[2][2]),
        sqrt(pz_inav->pz_model->f_syscovmat[3][3]), sqrt(pz_inav->pz_model->f_syscovmat[4][4]), sqrt(pz_inav->pz_model->f_syscovmat[5][5]),
        sqrt(pz_inav->pz_model->f_syscovmat[6][6]) * RAD2DEG, sqrt(pz_inav->pz_model->f_syscovmat[7][7]) * RAD2DEG, sqrt(pz_inav->pz_model->f_syscovmat[8][8]) * RAD2DEG);
      pz_inav->pz_obs->q_update_type = (uint32_t)SMT_DUALANT;
      pz_inav->pz_obs->u_kf_update = INS_TRUE;
    }
  }
  pz_inav->pz_obs->d_prebds_update_time = pd_meas[1];
#endif
  return s_val;
}
#endif

int8_t integrated_veh_update(double pd_meas[], InertialNav_t* pz_inav)
{
  int8_t s_val = INS_TRUE;
  uint8_t u_is_ready = INS_FALSE;
  uint8_t u_mdim = 0;

  float* f_z_meas = (float*)OS_MALLOC(pz_inav->pz_model->u_meadim * sizeof(float));//

  if (pz_inav->u_ins_update_mask == TYPE_MASK_NHC_UPDATE)
  {
    u_is_ready = nhc_meas_handler(pd_meas, f_z_meas, &u_mdim, pz_inav);
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NHC;
  }
  else if (pz_inav->u_ins_update_mask == TYPE_MASK_ZUPT_UPDATE)
  {
    u_is_ready = zupt_meas_handler(pd_meas, f_z_meas, &u_mdim, pz_inav);
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_ZUPTA;
  }
  else if (pz_inav->u_ins_update_mask == TYPE_MASK_ODO_UPDATE)
  {
    u_is_ready = odo_meas_handler(pd_meas, f_z_meas, &u_mdim, pz_inav);
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_ODOMETER;
  }
  else if (pz_inav->u_ins_update_mask == TYPE_MASK_WSPD_UPDATE)
  {
    WheelSpd_t* pz_whspd = fusion_get_wheelspd();
    if (fabs(pz_whspd->d_timestamp - pz_inav->pz_obs->pz_veh_node->d_time) < 0.1)
    {
      if (pz_whspd->f_vehspd_rl > 0.5 && pz_whspd->f_vehspd_rr > 0.5)
      {
        u_is_ready = whspd_meas_handler(pd_meas, f_z_meas, &u_mdim, pz_inav);
        pz_inav->pz_obs->q_update_type = (uint32_t)SMT_WHEELPULSE;
      }
    }
    else
    {
      LOGE(TAG_VDR, "%.3f, wheelspeed timestamp error: %.3f\n",
        pz_inav->pz_obs->pz_veh_node->d_time,pz_whspd->d_timestamp);
    }
  }
  else if (pz_inav->u_ins_update_mask == TYPE_MASK_WSPD_STEER_UPDATE)
  {
    u_is_ready = whspd_steer_meas_handler(pd_meas, f_z_meas, &u_mdim, pz_inav);
  }
  else
  {
    /* do-nothing */
  }
  if (u_is_ready == INS_TRUE && u_mdim > 0)
  {
    if (kf_update(f_z_meas, pz_inav->pz_model->u_sysdim, u_mdim, pz_inav->pz_model) != INS_TRUE)
    {
      s_val = INS_FALSE;
      pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
      LOGE(TAG_VDR, "%.3f, Vehicle KF Update Failed!\n", pd_meas[1]);
    }
    else
    {
      LOGI(TAG_VDR, "[ins_update]:%.3lf,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", pd_meas[1], pz_inav->pz_model->f_errstate[0], pz_inav->pz_model->f_errstate[1], pz_inav->pz_model->f_errstate[2],
          pz_inav->pz_model->f_errstate[3], pz_inav->pz_model->f_errstate[4], pz_inav->pz_model->f_errstate[5],
          pz_inav->pz_model->f_errstate[6] * RAD2DEG, pz_inav->pz_model->f_errstate[7] * RAD2DEG, pz_inav->pz_model->f_errstate[8] * RAD2DEG);
      LOGI(TAG_VDR, "[ins_update]:%.3lf,CovP:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", pd_meas[1],
          sqrt(pz_inav->pz_model->f_syscovmat[0]), sqrt(pz_inav->pz_model->f_syscovmat[1 * pz_inav->pz_model->u_sysdim + 1]), sqrt(pz_inav->pz_model->f_syscovmat[2 * pz_inav->pz_model->u_sysdim + 2]),
          sqrt(pz_inav->pz_model->f_syscovmat[3 * pz_inav->pz_model->u_sysdim + 3]), sqrt(pz_inav->pz_model->f_syscovmat[4 * pz_inav->pz_model->u_sysdim + 4]), sqrt(pz_inav->pz_model->f_syscovmat[5 * pz_inav->pz_model->u_sysdim + 5]),
          sqrt(pz_inav->pz_model->f_syscovmat[6 * pz_inav->pz_model->u_sysdim + 6]) * RAD2DEG, sqrt(pz_inav->pz_model->f_syscovmat[7 * pz_inav->pz_model->u_sysdim + 7]) * RAD2DEG, sqrt(pz_inav->pz_model->f_syscovmat[8 * pz_inav->pz_model->u_sysdim + 8]) * RAD2DEG);
      pz_inav->pz_obs->u_kf_update = INS_TRUE;
    }
  }
  pz_inav->pz_obs->d_preins_update_time = pd_meas[1];

  OS_FREE(f_z_meas);

  return s_val;
}

void gnss_calculate_meas(const double pd_mea[], SyncObsNav_t* pz_obs)
{
  float f_pos_diff[3] = { 0.0f };
  double d_bds_pos_ecef[3] = { 0.0 };
  double d_ins_pos_ecef[3] = { 0.0 };
  double d_bds_poslla[3] = { 0.0 };
  float f_rotmat_e2n[3][3] = { 0.0f };
  float f_ecef_diff[3] = { 0.0f };
  float f_matb2n[3][3] = { 0.0f };
  float* pf_imu2gnss = NULL;
  float f_lvrm_n[3] = { 0.0f };
  double d_ins_poslla[3] = { 0.0 };
  float f_ins_velned[3] = { 0.0f };
  float f_bds_vel[3] = { 0.0f };
  float f_vel_diff[3] = { 0.0f };
  float f_rotmat_e2c[3][3] = { 0.0f };
  float f_rotmat_n2c[3][3] = { 0.0f };
  float f_bdsvel_temp[3] = { 0.f };
  float f_matv2b[3][3] = { 0.0f };
  float f_matv2n[3][3] = { 0.0f };

  NavParams_t* pz_nav = fusion_get_navpara();
  MechanNode_t* pz_node = pz_obs->pz_bds_node;
  pf_imu2gnss = lvrm_get_imu2gnss();
  quat_quat2dcm(pz_node->f_qbn, f_matb2n);
  math_mattrns(&(pz_nav->f_rotmat_misalign[0][0]), 3, 3, &(f_matv2b[0][0]));
  math_matmul(&(f_matb2n[0][0]), &(f_matv2b[0][0]), 3, 3, 3, &(f_matv2n[0][0]));
  math_matmul(&(f_matv2n[0][0]), pf_imu2gnss, 3, 3, 1, f_lvrm_n);

  d_bds_poslla[0] = pd_mea[2];
  d_bds_poslla[1] = pd_mea[3];
  d_bds_poslla[2] = pd_mea[4];

  tf_lla2ecef(d_bds_poslla, d_bds_pos_ecef, d_bds_poslla);

  NavConfig_t* pz_config = fusion_get_navconfig();

  d_ins_poslla[0] = pz_node->d_lla[0];
  d_ins_poslla[1] = pz_node->d_lla[1];
  d_ins_poslla[2] = pz_node->d_lla[2];

  f_ins_velned[0] = pz_node->f_vel[0];
  f_ins_velned[1] = pz_node->f_vel[1];
  f_ins_velned[2] = pz_node->f_vel[2];

  float f_dt = (float)(pd_mea[1] - pz_node->d_time);

  if (fabs(f_dt) > 1e-6f)
  {
    float f_dt2 = 0.0f;
    float vel = 0.0f;
    float f_dis_ned[3] = { 0.f };

    f_dt2 = f_dt * f_dt;

    /* compensate position */
    f_dis_ned[0] = f_ins_velned[0] * f_dt + pz_node->f_fn[0] * f_dt2 * 0.5f;
    f_dis_ned[1] = f_ins_velned[1] * f_dt + pz_node->f_fn[1] * f_dt2 * 0.5f;
    f_dis_ned[2] = f_ins_velned[2] * f_dt + pz_node->f_fn[2] * f_dt2 * 0.5f;

    d_ins_poslla[0] += (double)(f_dis_ned[0]) / (pz_node->d_rm + d_ins_poslla[2]);
    d_ins_poslla[1] += (double)(f_dis_ned[1]) / ((pz_node->d_rn + d_ins_poslla[2]) * cos(d_ins_poslla[0]));
    d_ins_poslla[2] -= (double)(f_dis_ned[2]);

    /* compensate velocity */
    f_ins_velned[0] += pz_node->f_fn[0] * f_dt;
    f_ins_velned[1] += pz_node->f_fn[1] * f_dt;
    f_ins_velned[2] += pz_node->f_fn[2] * f_dt;
    LOGI(TAG_VDR, "[Meas_Comp]:%.3lf,%.7f\n", pd_mea[1], f_dt);
  }
#if 1
  tf_lla2ecef(d_ins_poslla, d_ins_pos_ecef, d_bds_poslla);

  tf_ce2nbylla(d_bds_poslla, f_rotmat_e2n);

  f_ecef_diff[0] = (float)(d_ins_pos_ecef[0] - d_bds_pos_ecef[0]);
  f_ecef_diff[1] = (float)(d_ins_pos_ecef[1] - d_bds_pos_ecef[1]);
  f_ecef_diff[2] = (float)(d_ins_pos_ecef[2] - d_bds_pos_ecef[2]);

  math_matmul(&(f_rotmat_e2n[0][0]), f_ecef_diff, 3, 3, 1, f_pos_diff);

  if (pz_config->u_i2g_opt != 0)
  {
    f_pos_diff[0] += f_lvrm_n[0];
    f_pos_diff[1] += f_lvrm_n[1];
    f_pos_diff[2] += f_lvrm_n[2];
  }
#else
  f_pos_diff[0] = (d_ins_poslla[0] - d_bds_poslla[0])* (pz_node->d_rm + d_ins_poslla[2]);
  f_pos_diff[1] = (d_ins_poslla[1] - d_bds_poslla[1])* ((pz_node->d_rn + d_ins_poslla[2]) * cos(d_ins_poslla[0]));
  f_pos_diff[2] = -(d_ins_poslla[2] - d_bds_poslla[2]);
  if (pz_config->u_i2g_opt != 0)
  {
    f_pos_diff[0] += f_lvrm_n[0];
    f_pos_diff[1] += f_lvrm_n[1];
    f_pos_diff[2] += f_lvrm_n[2];
  }

#endif
  for (uint8_t i = 0; i < 3; i++)
  {
    pz_obs->f_kf_meas[i] = f_pos_diff[i];
  }

  math_matsub(f_ins_velned, f_bds_vel, 3, 1, f_vel_diff);

  if (pz_config->u_i2g_opt != 0)
  {
    float win_n[3] = { 0.0f };
    float tmp1[3] = { 0.0f };
    float tmp2[3] = { 0.0f };
    float tmp3[3] = { 0.0f };
    math_matadd(pz_node->f_wien, pz_node->f_wnen, 3, 1, win_n);
    math_cross_product(win_n, f_lvrm_n, tmp1);
    math_cross_product(pf_imu2gnss, pz_node->f_wibb, tmp2);
    math_matmul(&(f_matb2n[0][0]), tmp2, 3, 3, 1, tmp3);
    math_matadd(tmp1, tmp3, 3, 1, tmp2);
    math_matsub(f_vel_diff, tmp2, 3, 1, f_vel_diff);
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    pz_obs->f_kf_meas[i + 3] = f_vel_diff[i];
  }
}

void integrated_lc_use_pos(double pd_meas[], float* pf_z_meas, InertialNav_t* pz_inav, uint8_t* dim)
{
  *dim = 3;
#if 0
  uint8_t* gns_zupt_update = is_gzupt();
  float* f_kf_meas = pz_inav->pz_obs->f_kf_meas;
  if (is_izupt() == 1 && (pd_meas[1] - pz_inav->pz_qc->d_pre_good_bds_time) > 10.0 &&
    pz_inav->pz_qc->d_pre_good_bds_time >= 0.0 &&
    pz_inav->pz_obs->u_posflag != TYPE_POS_FLAG_RTK_FIX && pd_meas[19] < 35.0f)
  {
    pf_z_meas[0] = f_kf_meas[0] > 10.f ? 3.f : f_kf_meas[0];
    pf_z_meas[1] = f_kf_meas[1] > 10.f ? 3.f : f_kf_meas[1];
    pf_z_meas[2] = f_kf_meas[2] > 10.f ? 3.f : f_kf_meas[2];
    *gns_zupt_update = 1;
    LOGI(TAG_VDR, "%.3lf, reduce bad pos while zupt, std=50m\n", pd_meas[1]);
  }
  else
  {
    pf_z_meas[0] = f_kf_meas[0];
    pf_z_meas[1] = f_kf_meas[1];
    pf_z_meas[2] = f_kf_meas[2];
    *gns_zupt_update = 0;
    LOGI(TAG_VDR, "%.3lf, good pos while zupt, or not zupt, zuptflag=%d\n", pd_meas[1], is_izupt());
  }
#else
  float* f_kf_meas = pz_inav->pz_obs->f_kf_meas;
  pf_z_meas[0] = f_kf_meas[0];
  pf_z_meas[1] = f_kf_meas[1];
  pf_z_meas[2] = f_kf_meas[2];
#endif
  kf_set_update_type(SMT_GPSPOS, pz_inav->pz_model);
  kf_update_measurement_matrix(pz_inav->pz_model->u_sysdim, *dim, pz_inav->pz_model);
  kf_update_measurement_noise(pd_meas, pz_inav->pz_model);
}

void integrated_lc_use_vel(double pd_meas[], float* pf_z_meas, InertialNav_t* pz_inav, uint8_t* dim)
{
  *dim = 3;
  uint8_t* gns_zupt_update = is_gzupt();
  float* f_kf_meas = pz_inav->pz_obs->f_kf_meas;
  if (is_izupt() == 1 && (pd_meas[1] - pz_inav->pz_qc->d_pre_good_bds_time) > 10.0 &&
    pz_inav->pz_qc->d_pre_good_bds_time >= 0.0 &&
    pz_inav->pz_obs->u_posflag != TYPE_POS_FLAG_RTK_FIX && pd_meas[19] < 35.0f)
  {
    pf_z_meas[0] = f_kf_meas[3] > 0.2f ? 0.2f : f_kf_meas[3];
    pf_z_meas[1] = f_kf_meas[4] > 0.2f ? 0.2f : f_kf_meas[4];
    pf_z_meas[2] = f_kf_meas[5] > 0.2f ? 0.2f : f_kf_meas[5];
    *gns_zupt_update = 1;
  }
  else
  {
    pf_z_meas[0] = f_kf_meas[3];
    pf_z_meas[1] = f_kf_meas[4];
    pf_z_meas[2] = f_kf_meas[5];
    *gns_zupt_update = 0;
  }
  kf_set_update_type(SMT_GPSVEL, pz_inav->pz_model);
  kf_update_measurement_matrix(pz_inav->pz_model->u_sysdim, *dim, pz_inav->pz_model);
  kf_update_measurement_noise(pd_meas, pz_inav->pz_model);
}

void integrated_lc_use_bds(double pd_meas[], float* pf_z_meas, InertialNav_t* pz_inav, uint8_t* dim)
{
  *dim = 6;
  uint8_t* gns_zupt_update = is_gzupt();
  float* f_kf_meas = pz_inav->pz_obs->f_kf_meas;
  if (is_izupt() == 1 && (pd_meas[1] - pz_inav->pz_qc->d_pre_good_bds_time) > 10.0 &&
    pz_inav->pz_qc->d_pre_good_bds_time >= 0.0 &&
    pz_inav->pz_obs->u_posflag != TYPE_POS_FLAG_RTK_FIX && pd_meas[19] < 35.0f)
  {
    pf_z_meas[0] = f_kf_meas[0] > 10.f ? 3.f : f_kf_meas[0];
    pf_z_meas[1] = f_kf_meas[1] > 10.f ? 3.f : f_kf_meas[1];
    pf_z_meas[2] = f_kf_meas[2] > 10.f ? 3.f : f_kf_meas[2];
    pf_z_meas[3] = f_kf_meas[3] > 0.2f ? 0.2f : f_kf_meas[3];
    pf_z_meas[4] = f_kf_meas[4] > 0.2f ? 0.2f : f_kf_meas[4];
    pf_z_meas[5] = f_kf_meas[5] > 0.2f ? 0.2f : f_kf_meas[5];
    *gns_zupt_update = 1;
  }
  else
  {
    memcpy(pf_z_meas, pz_inav->pz_obs->f_kf_meas, sizeof(pz_inav->pz_obs->f_kf_meas));
    *gns_zupt_update = 0;
  }

  kf_set_update_type(SMT_BDS, pz_inav->pz_model);
  kf_update_measurement_matrix(pz_inav->pz_model->u_sysdim, *dim, pz_inav->pz_model);
  kf_update_measurement_noise(pd_meas, pz_inav->pz_model);
}

void integrated_lc_find_proper_ins(double pd_meas[], SyncQueue_t* pz_queue)
{
  uint8_t index = mechan_find_node(pd_meas[1], pz_queue);

  if (index == SYNC_QUEUE_SIZE-1)
  {
    memcpy(&gz_sync_queue.z_mech_bds, &gz_sync_queue.z_nmech, sizeof(MechanNode_t));
  }
  else
  {
    memcpy(&gz_sync_queue.z_mech_bds, &gz_sync_queue.z_nodes[index], sizeof(MechanNode_t));
  }
}

uint8_t integrated_lc_procedure(double pd_meas[], InertialNav_t* pz_inav, NavParams_t* pz_para)
{
  uint8_t u_bds_update = INS_TRUE;
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  MechanNode_t* pz_node = pz_inav->pz_obs->pz_bds_node;
  float f_z_meas[6] = { 0.f };
  uint8_t u_dim = 0;
  int8_t s_qc_val = -1;

  pz_obs->u_kf_update = INS_FALSE;

  if (pz_obs->u_posflag > 0)
  {
    gnss_calculate_meas(pd_meas, pz_inav->pz_obs);
    s_qc_val = bds_qc_procedure(pd_meas, pz_inav);
  }
  else
  {
    u_bds_update = INS_FALSE;
  }

  if (u_bds_update == INS_TRUE)
  {
    integrated_lc_use_pos(pd_meas, f_z_meas, pz_inav, &u_dim);
    pz_obs->q_update_type = (uint32_t)SMT_GPSPOS;

    /*
    s_qc_val = 1;
    if (s_qc_val == 1)
    {
      integrated_lc_use_pos(pd_meas, f_z_meas, pz_inav, &u_dim);
      pz_obs->q_update_type = (uint32_t)SMT_GPSPOS;
    }
    else if (s_qc_val == 2)
    {
      integrated_lc_use_vel(pd_meas, f_z_meas, pz_inav, &u_dim);
      pz_obs->q_update_type = (uint32_t)SMT_GPSVEL;
    }
    else
    {
      integrated_lc_use_bds(pd_meas, f_z_meas, pz_inav, &u_dim);
      pz_obs->q_update_type = (uint32_t)SMT_BDS;
    }
    */
#ifdef MISANGLE_IMU2DUALANT_ERR_ESTI
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
    
    float f_gs = sqrtf(SQR(pz_node->f_wibb[0]) + SQR(pz_node->f_wibb[1]) + SQR(pz_node->f_wibb[2])) * RAD2DEG;
    LOGI(TAG_VDR, "%f, wibb = %f \n", pd_meas[1], f_gs);

    if ((pz_gnss->u_msg_type & NOVATEL_RECMGS_HEADING) == NOVATEL_RECMGS_HEADING &&
      pz_gnss->f_heading_std <1.0 && pz_gnss->f_heading_std > 0.0 &&
      pz_gnss->f_pitch_std < 1.0 && pz_gnss->f_pitch_std > 0.0 &&
      fabsf(pz_gnss->f_pitch_deg) < 20.0 && (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX)
      && pz_gnss->u_satused >=5)
    {	
      integrated_dualant_update(pd_meas, f_z_meas, pz_inav, &u_dim);
      pz_inav->pz_obs->q_update_type = (uint32_t)SMT_DUALANT;
    }
#endif

    if (kf_update(f_z_meas, pz_inav->pz_model->u_sysdim, u_dim, pz_inav->pz_model) == INS_FALSE)
    {
      fusion_reset();
      u_bds_update = INS_FALSE;
      pz_obs->q_update_type = (uint32_t)SMT_NONE;
      LOGE(TAG_VDR, "%.3f,BDS KF Update Failed!\n", pd_meas[1]);
    }
    else
    {
      pz_obs->u_kf_update = INS_TRUE;
      pz_inav->pz_obs->d_prebds_update_time = pz_inav->pz_obs->d_imu_t;
      /*
      uint8_t u_index = s_qc_val < 0 ? 0 : s_qc_val;
      char* update_str[3] = { "gnss:bds_update","gnss:pos_update","gnss:vel_update" };
      */
      uint8_t u_index = 0;
      char* update_str[1] = { "gnss:pos_update" };
      LOGI(TAG_VDR, "[%s]:%.3lf,Posflag:%d,PosKFMeas:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", update_str[u_index], pd_meas[1], pz_obs->u_posflag, f_z_meas[0], f_z_meas[1], f_z_meas[2],
        f_z_meas[3], f_z_meas[4], f_z_meas[5]);
      LOGI(TAG_VDR, "[%s]:%.3lf,Satnum:%.3f,AvgCNO:%.3f,Std:%.3f,%.3f,%.3f,GodBds:%.3lf,GnssZupt:%d\n", update_str[u_index], pd_meas[1], pd_meas[18], pd_meas[19], sqrt(pz_inav->pz_model->f_measnoise[0]),
        sqrt(pz_inav->pz_model->f_measnoise[1 * pz_inav->pz_model->u_meadim + 1]),sqrt(pz_inav->pz_model->f_measnoise[2 * pz_inav->pz_model->u_meadim + 2]),pz_inav->pz_qc->d_pre_good_bds_time,is_izupt());
      LOGI(TAG_VDR, "[%s]:%.3lf,Posflag:%d,CovP:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", update_str[u_index], pd_meas[1], pz_obs->u_posflag,
        sqrt(pz_inav->pz_model->f_syscovmat[0]), sqrt(pz_inav->pz_model->f_syscovmat[1 * pz_inav->pz_model->u_sysdim + 1]), sqrt(pz_inav->pz_model->f_syscovmat[2 * pz_inav->pz_model->u_sysdim + 2]),
        sqrt(pz_inav->pz_model->f_syscovmat[3 * pz_inav->pz_model->u_sysdim + 3]), sqrt(pz_inav->pz_model->f_syscovmat[4 * pz_inav->pz_model->u_sysdim + 4]), sqrt(pz_inav->pz_model->f_syscovmat[5 * pz_inav->pz_model->u_sysdim + 5]),
        sqrt(pz_inav->pz_model->f_syscovmat[6 * pz_inav->pz_model->u_sysdim + 6]) * RAD2DEG, sqrt(pz_inav->pz_model->f_syscovmat[7 * pz_inav->pz_model->u_sysdim + 7]) * RAD2DEG, sqrt(pz_inav->pz_model->f_syscovmat[8 * pz_inav->pz_model->u_sysdim + 8]) * RAD2DEG);
    
#ifdef OPEN_INS_INTEGRITY
      /* comment for misra C cause s_qc_val is always -1 */
      /*
      if (s_qc_val == 2)
      {

      }
      else
      {
        gz_ekf_dpos.d_gnssposmeas_time = pd_meas[1];
        gz_ekf_dpos.f_pos_meas_z[0] = f_z_meas[0];
        gz_ekf_dpos.f_pos_meas_z[1] = f_z_meas[1];
        gz_ekf_dpos.f_pos_meas_z[2] = f_z_meas[2];
        gz_ekf_dpos.f_measnoise_pos[0] = sqrtf(pz_inav->pz_model->f_measnoise[0]);
        gz_ekf_dpos.f_measnoise_pos[1] = sqrtf(pz_inav->pz_model->f_measnoise[1 * pz_inav->pz_model->u_meadim + 1]);
        gz_ekf_dpos.f_measnoise_pos[2] = sqrtf(pz_inav->pz_model->f_measnoise[2 * pz_inav->pz_model->u_meadim + 2]);
        gz_ekf_dpos.u_posflag = pz_obs->u_posflag;
      }
      */
      gz_ekf_dpos.d_gnssposmeas_time = pd_meas[1];
      gz_ekf_dpos.f_pos_meas_z[0] = f_z_meas[0];
      gz_ekf_dpos.f_pos_meas_z[1] = f_z_meas[1];
      gz_ekf_dpos.f_pos_meas_z[2] = f_z_meas[2];
      gz_ekf_dpos.f_measnoise_pos[0] = sqrtf(pz_inav->pz_model->f_measnoise[0]);
      gz_ekf_dpos.f_measnoise_pos[1] = sqrtf(pz_inav->pz_model->f_measnoise[1 * pz_inav->pz_model->u_meadim + 1]);
      gz_ekf_dpos.f_measnoise_pos[2] = sqrtf(pz_inav->pz_model->f_measnoise[2 * pz_inav->pz_model->u_meadim + 2]);
      gz_ekf_dpos.u_posflag = pz_obs->u_posflag;
#endif
    }
  }
  LOGD(TAG_VDR, "bds_qc_procedure qc result %d", s_qc_val);
  return u_bds_update;
}

void integrated_outage_detect(double pd_meas[], SyncObsNav_t* pz_obs)
{
  if (pz_obs->u_last_posflag > 0 && pz_obs->u_posflag == 0) /* can not just use flag */
  {
    pz_obs->d_bds_outage_start_time = pd_meas[1];
  }
  else if (pz_obs->u_last_posflag == 0 && pz_obs->u_posflag > 0)
  {
    pz_obs->d_bds_outage_end_time = pd_meas[1];
  }
}

int8_t integrated_procedure(double pd_meas[], InertialNav_t* pz_inav, NavParams_t* pz_para)
{
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();
  INSResults_t* pz_ins = fusion_get_ins_results();

  if (pz_inav->u_ins_status == INS_STATUS_NAVIGATION)
  {
    pz_inav->pz_obs->u_kf_update = INS_FALSE;

    if (pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC_CP || pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC)
    {
      if (pd_meas[0] > 0)
      {
        kf_transmat_tdcp(pd_meas, pz_inav); /* cal and save PHIji */
      }
    }

    if (pd_meas[0] > 0)
    {
      if (pz_inav->pz_obs->u_ins_valid == INS_TRUE)
      {
        pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
        ins_get_update_type(pz_inav);
        if (integrated_veh_update(pd_meas, pz_inav) == INS_FALSE)
        {
          fusion_reset();
        }
        pz_inav->pz_obs->u_ins_valid = INS_FALSE;
      }
    }

    if (pd_meas[0] < 0 && pz_inav->u_keep_pos == INS_FALSE)
    {
      pz_inav->pz_obs->u_posflag = *(uint8_t*)(&pd_meas[14]);

      integrated_outage_detect(pd_meas, pz_inav->pz_obs);

      integrated_lc_find_proper_ins(pd_meas, &gz_sync_queue);

      if (fabs(gz_sync_queue.z_mech_bds.d_time - pd_meas[1]) < 0.05 * 2)
      {
        pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;

        if (pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC_PRDR)
        {
          AsensingGNSSPsrMeas_t* pz_psr_paras = fusion_get_gnss_psr_meas();
          tight_couple_psr_precheck(pz_psr_paras, pz_inav);

          float f_gnssstd = (float)sqrt(pd_meas[8] + pd_meas[9]);
#if 1
          if (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX ||
            (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FLOAT && f_gnssstd < 0.5) ||
            pz_inav->u_psr_used_satnum == 0 || is_izupt() == 1 ||
            (pz_inav->pz_qc->u_gbias_converged == INS_FALSE))
#else
          if (pz_inav->u_psr_used_satnum == 0)
#endif
          {
            integrated_lc_procedure(pd_meas, pz_inav, pz_para);
          }
          else
          {
            tight_couple_psr_procedure(pz_inav, pz_para);
          }
        }
        else if (pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC_CP)
        {
          AsensingGNSSCpMeas_t* pz_cp_paras = fusion_get_gnss_cp_meas();
          tight_couple_cp_precheck(pz_cp_paras, pz_inav);

          float f_gnssstd = (float)sqrt(pd_meas[8] + pd_meas[9]);
#if 1
          if (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX ||
            (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FLOAT && f_gnssstd < 0.5) ||
            pz_inav->u_cp_used_satnum == 0 || is_izupt() == 1 ||
            (pz_inav->pz_qc->u_gbias_converged == INS_FALSE))
#else
          if (pz_inav->u_cp_used_satnum == 0 || pd_meas[1] < 271000.0)
#endif
          {
            integrated_lc_procedure(pd_meas, pz_inav, pz_para);
          }
          else
          {
            tight_couple_cp_procedure(pz_inav, pz_para);
          }
        }
        else if (pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC)
        {
          AsensingGNSSPsrMeas_t* pz_psr_paras = fusion_get_gnss_psr_meas();
          AsensingGNSSCpMeas_t* pz_cp_paras = fusion_get_gnss_cp_meas();

          tight_couple_psr_precheck(pz_psr_paras, pz_inav);
          tight_couple_cp_precheck(pz_cp_paras, pz_inav);

          float f_gnssstd = (float)sqrt(pd_meas[8] + pd_meas[9]);
#if 1
          if (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX ||
            (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FLOAT && f_gnssstd < 0.5) ||
            (pz_inav->u_psr_used_satnum == 0 && pz_inav->u_cp_used_satnum == 0) || is_izupt() == 1 ||
            (pz_inav->pz_qc->u_gbias_converged == INS_FALSE))
#else
          if (pz_inav->u_psr_used_satnum == 0)
#endif
          {
            integrated_lc_procedure(pd_meas, pz_inav, pz_para); /* not enter tc */
          }
          else
          {
            if (pz_inav->u_psr_used_satnum > 0)
            {
              tight_couple_psr_procedure(pz_inav, pz_para);
            }
            if (pz_inav->u_cp_used_satnum > 0)
            {
              tight_couple_cp_procedure(pz_inav, pz_para);
            }
          }
        }
        else if (pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_LC)
        {
          integrated_lc_procedure(pd_meas, pz_inav, pz_para); /* pz_para not use */
        }
      }
      else
      {
        LOGE(TAG_VDR, "reject gnss for time align issue, %.3f, mech_node-bds:%f \n", pd_meas[1], gz_sync_queue.z_mech_bds.d_time - pd_meas[1]);
      }
      pz_inav->pz_obs->u_last_posflag = pz_inav->pz_obs->u_posflag;

      if (pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC_CP || pz_algconfig->z_init_para.q_vdr_mode == TYPE_VDR_TC)
      {
        if (pz_inav->pz_obs->u_kf_update > 0)
        {
          kf_tdcp_pos_feedback(pz_inav->pz_model->f_errstate, pz_inav);
        }
        else
        {
          memcpy(pz_inav->pz_obs->d_lla_prepps, pz_inav->pz_obs->pz_bds_node->d_lla, sizeof(pz_inav->pz_obs->d_lla_prepps));
        }
      }
    }

    if (pz_inav->pz_obs->u_kf_update > 0)
    {
      kf_feedback_procedure(pz_inav->pz_model->f_errstate, pz_inav, pz_para);
#ifdef OPEN_INS_INTEGRITY
      gz_ekf_dpos.u_kf_update = pz_inav->pz_obs->u_kf_update;
      gz_ekf_dpos.q_update_type = pz_inav->pz_obs->q_update_type;
      gz_ekf_dpos.f_errstate[0] += pz_inav->pz_model->f_errstate[0];
      gz_ekf_dpos.f_errstate[1] += pz_inav->pz_model->f_errstate[1];
      gz_ekf_dpos.f_errstate[2] += pz_inav->pz_model->f_errstate[2];
#endif
      memset(pz_inav->pz_model->f_errstate, 0, pz_inav->pz_model->u_sysdim*sizeof(float));
      pz_ins->q_kfmeastype = pz_ins->q_kfmeastype |pz_inav->pz_obs->q_update_type;
    }        
  }
  return 0;
}

uint8_t fusion_check_veh_nearby(double pd_meas[], uint8_t* index, InertialNav_t* pz_inav)
{
  FusionAlgoConfig_t* pz_algocfg = fusion_get_algconfig();
  float f_ins_bds_dt = (float)(pd_meas[1] - pz_inav->pz_obs->d_prebds_update_time);
  uint8_t u_ins_update_flag = INS_FALSE;

  if (f_ins_bds_dt > 2.0f * pz_algocfg->f_gnss_dt)
  {
    if (pd_meas[1] - pz_inav->pz_obs->d_preins_update_time + pz_algocfg->f_sampling_dt > pz_algocfg->f_veh_calc_dt)
    {
      u_ins_update_flag = INS_TRUE;
    }
  }
  else
  {
    if (f_ins_bds_dt > 0.5f * pz_algocfg->f_gnss_dt)
    {
      if (pd_meas[1] - pz_inav->pz_obs->d_preins_update_time + pz_algocfg->f_sampling_dt > 0.7 * pz_algocfg->f_veh_calc_dt)
      {
        u_ins_update_flag = INS_TRUE;
      }
    }
  }

  if (u_ins_update_flag == INS_TRUE)
  {
    if (pz_inav->pz_config->u_whspd_enable == TYPE_WHEEL_AIDING_NHC || is_izupt() == 1 ||
        (fabs(pd_meas[1] - pd_meas[8]) > pz_algocfg->f_sampling_dt * 20.0)) /* wheel invalid */
    {
      *index = SYNC_QUEUE_SIZE - 1; // 0;
      pz_inav->pz_config->u_whspd_enable = TYPE_WHEEL_AIDING_NHC;
      if (is_izupt() != 1)
      {
        // LOGE(TAG_VDR, "disable whlspd for time align issue, %.3f, %.3f, %d \n", pd_meas[1], pd_meas[8], is_izupt());
      }
    }
    else 
    {
      *index = mechan_find_node(pd_meas[8], &gz_sync_queue); /* make sure wheel data valid */

      if (fabs(gz_sync_queue.z_nodes[*index].d_time - pd_meas[8]) > pz_algocfg->f_sampling_dt * 2.0)
      {
        pz_inav->pz_config->u_whspd_enable = TYPE_WHEEL_AIDING_NHC;
        // LOGE(TAG_VDR, "%.3f, reject whlspd for time align issue,  mech_node-whl:%f \n", pd_meas[8], gz_sync_queue.z_nodes[*index].d_time - pd_meas[8]);
      }
    }
  }
  return u_ins_update_flag;
}

void fusion_misalign_converge_calculation()
{
  INSResults_t* pz_ins = fusion_get_ins_results();
  NavParams_t* pz_nav = fusion_get_navpara();
  SysErrModel_t* pz_model = fusion_get_sysmodel();
  float f_misb2v_pitch_std = (float)(sqrtf(pz_model->f_syscovmat[(15 + 1) * pz_model->u_sysdim + 15 + 1]) * RAD2DEG);
  float f_misb2v_heading_std = (float)(sqrtf(pz_model->f_syscovmat[(15 + 2) * pz_model->u_sysdim + 15 + 2]) * RAD2DEG);
  if (pz_ins->u_misb2v_convergeflag == 0 &&
      f_misb2v_pitch_std < 0.05 && f_misb2v_heading_std < 0.1 &&
      f_misb2v_pitch_std > 0.0 && f_misb2v_heading_std > 0.0)
  {
    static float f_misb2v_buf[3][MIS_BUF_SIZE];
    static int u_counter = 0;
    if (u_counter < MIS_BUF_SIZE)
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        f_misb2v_buf[i][u_counter] = pz_nav->f_mis_align[i];
      }
      u_counter++;
    }
    else
    {
      float f_misb2v_mean[3], f_misb2v_std[3];
      float f_misb2v_diff[3];
      for (uint8_t i = 0; i < 3; i++)
      {
        math_calculate_std(f_misb2v_buf[i], MIS_BUF_SIZE, &f_misb2v_mean[i], &f_misb2v_std[i]);
        math_calculate_maxmindiff(f_misb2v_buf[i], MIS_BUF_SIZE, &f_misb2v_diff[i]);
      }

      for (uint8_t i = 0; i < 3; i++)
      {
        memmove(f_misb2v_buf[i], f_misb2v_buf[i] + 1, (MIS_BUF_SIZE - 1) * sizeof(float));
        f_misb2v_buf[i][MIS_BUF_SIZE - 1] = pz_nav->f_mis_align[i];
      }

      if (f_misb2v_diff[1] * RAD2DEG < 0.2 && f_misb2v_diff[2] * RAD2DEG < 0.2
          && f_misb2v_std[1] * RAD2DEG < 0.2 && f_misb2v_std[2] * RAD2DEG < 0.2)
      {
        memcpy(pz_ins->f_misb2v_converge, pz_nav->f_mis_align, sizeof(pz_ins->f_misb2v_converge));
        pz_ins->u_misb2v_convergeflag = 1;
      }
    }
  }
}

int8_t sins_procedure(double* pd_meas, InertialNav_t* pz_inav, NavParams_t* pz_para)
{
  if (pz_inav->u_init_nav == INS_FALSE)
  {
    memset(&gz_obs_nav, 0, sizeof(gz_obs_nav));
    memset(&gz_sync_queue, 0, sizeof(gz_sync_queue));

    pz_inav->u_init_nav = INS_TRUE;
    pz_inav->u_ins_timeout = INS_FALSE;
    pz_inav->d_timeout_zupttime = 0.0;
    pz_inav->d_nav_start_time = pd_meas[1];
    pz_inav->pz_qc = fusion_get_qc_para();
    pz_inav->pz_obs = &gz_obs_nav;
    pz_inav->pz_obs->d_prebds_update_time = INITIAL_TIME;
    pz_inav->pz_obs->d_preins_update_time = INITIAL_TIME;
    pz_inav->pz_obs->d_bds_outage_start_time = INITIAL_TIME;
    pz_inav->pz_obs->d_bds_outage_end_time = INITIAL_TIME;
    pz_inav->pz_model = fusion_get_sysmodel();
    pz_inav->pz_config = fusion_get_navconfig();
    pz_inav->pz_obs->pz_bds_node = &gz_sync_queue.z_mech_bds;
    pz_inav->pz_obs->pz_veh_node = &gz_sync_queue.z_mech_veh;
    pz_inav->pz_cur_mech = &gz_sync_queue.z_nmech;
    pz_inav->f_gnss_sampling_dt = 1.f / fusion_get_algconfig()->z_init_para.e_gnss_rate;
    qc_initial(pz_inav->pz_config);

    mechan_initial();
    mechan_set_time(INITIAL_TIME);
  }

  if (pd_meas[0] > 0)
  {
    FusionAlgoConfig_t* pz_algocfg = fusion_get_algconfig();
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
    pz_inav->pz_obs->d_imu_t = pd_meas[1];

    if (mechan_get_time() < 0)
    {
      mechan_set_time(pd_meas[1]);
      memset(pz_inav->d_imu_buf, 0, sizeof(pz_inav->d_imu_buf));
    }
    else
    {
      float d_delta_t = (float)(pd_meas[1] - mechan_get_time());

      if (float_equal(d_delta_t, 0.0) == 1)
      {
        d_delta_t = MECH_DEFAULT_DT;
      }

      for (uint8_t i = 0; i < 3; i++)
      {
        pz_inav->d_imu_buf[i + 2] += ((pd_meas[i + 2] - pz_para->f_gyrobias[i]) * d_delta_t);
        pz_inav->d_imu_buf[i + 5] += ((pd_meas[i + 5] - pz_para->f_acclbias[i]) * d_delta_t);
      }
      pz_inav->d_imu_buf[8] = pd_meas[9] + pd_meas[10] + pd_meas[11] + pd_meas[12];            

      if (is_izupt() == INS_TRUE)
      {
        pz_inav->d_timeout_zupttime += d_delta_t;
      }

      if (pz_inav->pz_config->u_whspd_enable > 0)
      {
        zupt_keep_pva(pz_inav);
      }
      pz_inav->d_imu_buf[1] = pd_meas[1];
                
      if (fusion_mechan_handler(pz_inav->d_imu_buf, &gz_navpara, &gz_sync_queue.z_nmech) > 0)
      {
        kf_transmat_update(pd_meas[1], &gz_navpara);
        // kf_transmat_tdcp(pd_meas, pz_inav);			
        kf_predict(gz_sync_queue.z_nmech.d_time, pz_inav->pz_model);
        mechan_fill_node(&gz_sync_queue);

        if (pz_inav->pz_obs->u_ins_valid == INS_FALSE)
        {
          uint8_t index = 0;
          if (fusion_check_veh_nearby(pd_meas, &index, pz_inav) > 0)
          {
            if (index == SYNC_QUEUE_SIZE - 1)
            {
              memcpy(&gz_sync_queue.z_mech_veh, &gz_sync_queue.z_nmech, sizeof(MechanNode_t));
            }
            else
            {
              memcpy(&gz_sync_queue.z_mech_veh, &gz_sync_queue.z_nodes[index], sizeof(MechanNode_t));
            }
            pz_inav->pz_obs->u_ins_valid = INS_TRUE;
          }
        }
      }
      for (uint8_t i = 2; i < 8; i++)
      {
        pz_inav->d_imu_buf[i] = 0;
      }
    }
  }

  integrated_procedure(pd_meas, pz_inav, pz_para);

  fusion_misalign_converge_calculation();

  return 0;
}

int8_t ins_navigation_proc(double* pd_meas, InertialNav_t* pz_inav, NavParams_t* pz_para)
{
  if (pd_meas[0] > 0)
  {
    FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

    if (pz_inav->d_last_imu_time < 0)
    {
      pz_inav->d_last_imu_time = pd_meas[1];
    }
    else
    {
      float f_delta_t = (float)(pd_meas[1] - pz_inav->d_last_imu_time);
      float f_imu_dt = pz_algconfig->f_sampling_dt;
      float f_imu_maxdt = 1.5;
      float f_imu_mindt = f_imu_dt * 0.5f;

#ifdef PLAYBACK_MODE
      if (f_delta_t > f_imu_maxdt)
      {
        printf("IMU Time Gap:%.3lf,%.3f\n", pd_meas[1], f_delta_t);
      }
#endif // PLAYBACK_MODE

      if (f_delta_t < f_imu_mindt || f_delta_t >f_imu_maxdt)
      {
        f_delta_t = f_imu_dt;
      }
      pz_inav->f_imu_sampling_dt = f_delta_t;
      pz_inav->d_last_imu_time = pd_meas[1];
    }
  }

  HistoryInsResults_t* pz_history = fusion_get_historyInfo();
#ifdef DISABLE_AXIS_DETECT
  pz_inav->u_align_method = 1;

  FlxAxisDectct_t pz_flx = { 0 };
  pz_flx.u_misalign_flag = INS_TRUE;
    
  float* f_mis_b2v = mis_get_imu2vehicle();
  for (int8_t i = 0; i < 3; i++)
  {
    pz_flx.f_miseuler[i] = f_mis_b2v[i];
  }

  if (pz_inav->u_init_align == INS_FALSE)
  {
    NavConfig_t* pz_navconfig = fusion_get_navconfig();
    flx_config_misalign(pz_navconfig, &pz_flx);
  }
#else 	
  FusionAlgoConfig_t* pz_config = fusion_get_algconfig();
  if ((pz_history->u_validType & 0x01) == 0x01 && (pz_config->z_init_para.q_align_mode & 0x02) == 0x02) /* history axis valid & support history mode */
  {
    if (pz_inav->u_init_align == INS_FALSE)
    {
      if ((pz_history->u_validType & 0x02) == 0x02 && (pz_config->z_init_para.q_align_mode & 0x01) == 0x01) /* history pos valid & support history pos mode */
      {
        pz_inav->u_align_method = 2; /* use history align */
      }
      else
      {
        pz_inav->u_align_method = 1;
      }
      flx_init_misAngle_by_history(pz_history);
      pz_inav->u_init_align = INS_TRUE;
    }
    /* axis convert */
    flx_convert_imu_axis_by_history(pz_history, pd_meas);
  }
  else
  {
    /* axis detection */
    flx_detect_triaxis(pd_meas);
    pz_inav->u_align_method = 1;
  }
#endif

  switch (pz_inav->u_ins_status)
  {
  case INS_STATUS_INITIALIZATION:
    if (pz_inav->u_init_align == INS_TRUE)
    {
      pz_inav->u_ins_status = INS_STATUS_INMOTIONALIGN;
    }
#ifdef TIME_COST_COUNT
      QueryPerformanceFrequency(&gz_timecost.m_nFreq);
#endif
    break;
  case INS_STATUS_INMOTIONALIGN:
    align_procedure(pd_meas);
    break;
  case INS_STATUS_NAVIGATION:
    {
#ifdef TIME_COST_COUNT
      static uint64_t a = 0;
      static uint64_t count = 0;      
      QueryPerformanceCounter(&gz_timecost.m_nBeginTime);

      sins_procedure(pd_meas, pz_inav, pz_para);

      QueryPerformanceCounter(&gz_timecost.m_nEndTime);
      a += gz_timecost.m_nEndTime.QuadPart - gz_timecost.m_nBeginTime.QuadPart;
      printf("cost time: %lld ms count %d\n", (a) * 1000 / gz_timecost.m_nFreq.QuadPart, count++);
      if (count ==500000)
      {
        uint64_t timecost = (a) * 1000 / gz_timecost.m_nFreq.QuadPart;
      }
#else
      sins_procedure(pd_meas, pz_inav, pz_para);
#endif

      break;
    }
  default:
    break;
  }
  return (int8_t)pz_inav->u_ins_status;
}

void set_history_axial(HistoryInsResults_t* output)
{
  HistoryInsResults_t* pz_history = fusion_get_historyInfo();
  FlxAxisDectct_t* pz_flx_axis = fusion_get_flx_axis();
  if (pz_history->u_validType & 0x01)
  {
    output->u_v_axis_mode = pz_history->u_v_axis_mode;
    output->u_h_axis_mode = pz_history->u_h_axis_mode;
    output->f_mis_roll = pz_history->f_mis_roll;
    output->f_mis_pitch = pz_history->f_mis_pitch;
    output->f_mis_yaw = pz_history->f_mis_yaw;
    output->u_validType = output->u_validType | 0x01;
    return;
  }
  if (pz_flx_axis->u_is_detected == INS_TRUE && pz_flx_axis->u_axis_flag > 0 && pz_flx_axis->u_misalign_flag == INS_TRUE)
  {
    output->u_v_axis_mode = pz_flx_axis->z_vert_axis.u_axis_mode;
    output->u_h_axis_mode = pz_flx_axis->u_axis_flag;
    output->f_mis_roll = (float)(pz_flx_axis->f_miseuler[0] * RAD2DEG);
    output->f_mis_pitch = (float)(pz_flx_axis->f_miseuler[1] * RAD2DEG);
    output->f_mis_yaw = (float)(pz_flx_axis->f_miseuler[2] * RAD2DEG);
    output->u_validType = output->u_validType | 0x01;
    return;
  }
}

int8_t ins_handle_meas(double* pd_meas, INSResults_t* pz_ins)
{
  double d_meas[UNIFY_MEAS_SIZE] = { 0.0 };
  HistoryInsResults_t z_historyInfo = { 0 };

  memcpy(d_meas, pd_meas, sizeof(d_meas));

  auto_zupt_detection(pd_meas, &gz_navpara);

  ins_navigation_proc(pd_meas, &gz_inertial, &gz_navpara);

  set_history_axial(&z_historyInfo);

  if (is_fusion_valid() == INS_TRUE)
  {
    pz_ins->d_latitude = gz_navpara.d_poslla[0];
    pz_ins->d_longitude = gz_navpara.d_poslla[1];
    pz_ins->f_altitude = (float)gz_navpara.d_poslla[2];

    pz_ins->f_vn = gz_navpara.f_vel_ned[0];
    pz_ins->f_ve = gz_navpara.f_vel_ned[1];
    pz_ins->f_vd = gz_navpara.f_vel_ned[2];

    float f_rotmat_v2n[3][3] = { 0.f };
    float f_eluer_v[3];
    math_matmul(*gz_navpara.f_rotmat_b2n, *gz_navpara.f_rotmat_v2b, 3, 3, 3, *f_rotmat_v2n);

    tf_dcm2eluer(f_rotmat_v2n, f_eluer_v);

    pz_ins->f_roll = (float)(f_eluer_v[0] * RAD2DEG);
    pz_ins->f_pitch = (float)(f_eluer_v[1] * RAD2DEG);
    pz_ins->f_heading = (float)(f_eluer_v[2] * RAD2DEG);

    pz_ins->f_gyrobias_x = (float)(gz_navpara.f_gyrobias[0] * RAD2DEG);
    pz_ins->f_gyrobias_y = (float)(gz_navpara.f_gyrobias[1] * RAD2DEG);
    pz_ins->f_gyrobias_z = (float)(gz_navpara.f_gyrobias[2] * RAD2DEG);

    pz_ins->f_accbias_x = gz_navpara.f_acclbias[0];
    pz_ins->f_accbias_y = gz_navpara.f_acclbias[1];
    pz_ins->f_accbias_z = gz_navpara.f_acclbias[2];

    pz_ins->f_mis_roll = (float)(gz_navpara.f_mis_align[0] * RAD2DEG);
    pz_ins->f_mis_pitch = (float)(gz_navpara.f_mis_align[1] * RAD2DEG);
    pz_ins->f_mis_yaw = (float)(gz_navpara.f_mis_align[2] * RAD2DEG);

    pz_ins->f_whlspd_sf[0] = gz_navpara.f_wspd_sf[0];
    pz_ins->f_whlspd_sf[1] = gz_navpara.f_wspd_sf[1];
    pz_ins->f_whlspd_sf[2] = gz_navpara.f_wspd_sf[2];
    pz_ins->f_whlspd_sf[3] = gz_navpara.f_wspd_sf[3];

    pz_ins->f_lat_std = sqrtf(gz_inertial.pz_model->f_syscovmat[0]);
    pz_ins->f_lon_std = sqrtf(gz_inertial.pz_model->f_syscovmat[1 * gz_inertial.pz_model->u_sysdim + 1]);
    pz_ins->f_alt_std = sqrtf(gz_inertial.pz_model->f_syscovmat[2 * gz_inertial.pz_model->u_sysdim + 2]);

    pz_ins->f_vn_std = sqrtf(gz_inertial.pz_model->f_syscovmat[3 * gz_inertial.pz_model->u_sysdim + 3]);
    pz_ins->f_ve_std = sqrtf(gz_inertial.pz_model->f_syscovmat[4 * gz_inertial.pz_model->u_sysdim + 4]);
    pz_ins->f_vd_std = sqrtf(gz_inertial.pz_model->f_syscovmat[5 * gz_inertial.pz_model->u_sysdim + 5]);

    pz_ins->f_roll_std = sqrtf(gz_inertial.pz_model->f_syscovmat[6 * gz_inertial.pz_model->u_sysdim + 6]);
    pz_ins->f_pitch_std = sqrtf(gz_inertial.pz_model->f_syscovmat[7 * gz_inertial.pz_model->u_sysdim + 7]);
    pz_ins->f_yaw_std = sqrtf(gz_inertial.pz_model->f_syscovmat[8 * gz_inertial.pz_model->u_sysdim + 8]);                

    if (gnss_get_posflag() > 0)
    {
      pz_ins->e_drposflag = TYPE_DR_POS_FLAG_FUSION;
    }
    else
    {
      pz_ins->e_drposflag = TYPE_DR_POS_FLAG_DR_OLY;
    }

    pz_ins->u_zuptflag = is_izupt();
    z_historyInfo.u_validType = z_historyInfo.u_validType | 0x02;
    z_historyInfo.d_lat = pz_ins->d_latitude;
    z_historyInfo.d_lon = pz_ins->d_longitude;
    z_historyInfo.f_alt = pz_ins->f_altitude;
    z_historyInfo.f_vn = pz_ins->f_vn;
    z_historyInfo.f_ve = pz_ins->f_ve;
    z_historyInfo.f_vd = pz_ins->f_vd;
    z_historyInfo.f_roll = (float)(gz_navpara.f_att_rph[0] * RAD2DEG);
    z_historyInfo.f_pitch = (float)(gz_navpara.f_att_rph[1] * RAD2DEG);
    z_historyInfo.f_yaw = (float)(gz_navpara.f_att_rph[2] * RAD2DEG);
    z_historyInfo.f_mis_roll = pz_ins->f_mis_roll;
    z_historyInfo.f_mis_pitch = pz_ins->f_mis_pitch;
    z_historyInfo.f_mis_yaw = pz_ins->f_mis_yaw;
  }
  else
  {
    pz_ins->e_drposflag = TYPE_DR_POS_FLAG_MIS_EST;
  }
  /* output lever arm */
  float* pf_imu2gnss = lvrm_get_imu2gnss();
  pz_ins->f_la_imu2gnss[0] = pf_imu2gnss[0];
  pz_ins->f_la_imu2gnss[1] = pf_imu2gnss[1];
  pz_ins->f_la_imu2gnss[2] = pf_imu2gnss[2];

  float* pf_imu2rearmid = lvrm_get_imu2rearmid();
  pz_ins->f_la_imu2rearmid[0] = pf_imu2rearmid[0];
  pz_ins->f_la_imu2rearmid[1] = pf_imu2rearmid[1];
  pz_ins->f_la_imu2rearmid[2] = pf_imu2rearmid[2];

  float* pf_rear2rear = lvrm_get_rear2rear();
  pz_ins->f_la_rear2rear = *pf_rear2rear;

  /* output mis_dualant */
  if (fusion_get_algconfig()->z_syserrtype.u_mis_dualant_err_esti > 0)
  {
    float* pf_mis_dualant = mis_get_imu2dualant();
    pz_ins->f_misdualant_roll = pf_mis_dualant[0];
    pz_ins->f_misdualant_pitch = pf_mis_dualant[1];
    pz_ins->f_misdualant_yaw = pf_mis_dualant[2];
  }

  if (fusion_get_callback()->fusion_report_history)
  {
    fusion_get_callback()->fusion_report_history(&z_historyInfo);
  }

  return 0;
}
