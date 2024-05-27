/**@file        fusion_proc.h
 * @brief		fusion proc header file
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

#ifndef __FUSION_PROC_H__
#define __FUSION_PROC_H__

#include <stdint.h>
#include "fusion_api.h"
#include "fusion_mech.h"
#include "fusion_err_model.h"
#include "fusion_integrity.h"
#include "fusion_config.h"
#include "fusion_global.h"
#include "fusion_meas_qc.h"

#define	INS_STATUS_INITIALIZATION				(0x01u)
#define	INS_STATUS_INMOTIONALIGN				(0x02u)
#define	INS_STATUS_NAVIGATION					(0x04u)

#define TYPE_MASK_UPDATE_NONE   (0x0)
#define TYPE_MASK_ZUPT_UPDATE  (0x01)
#define TYPE_MASK_NHC_UPDATE  (0x02)
#define TYPE_MASK_ODO_UPDATE  (0x04)
#define TYPE_MASK_WSPD_UPDATE  (0x08)
#define TYPE_MASK_WSPD_STEER_UPDATE  (0x20)

#define ERR_STATE_BUF_SIZE 8
#define KF_CONVER_TIME (30 * GNSS_SAMPLING_RATE)

typedef enum
{
  NAVI_UT_NONE = 0,
  NAVI_UT_BDSONLY,
  NAVI_UT_ZUPTA,
  NAVI_UT_ZUPT,
  NAVI_UT_NHC,
  NAVI_UT_WLSPD,
  NAVI_UT_ODOM,
  NAVI_UT_WLSTEER
} NaviUpdate_Enum;

typedef struct
{
  double d_sys_timestamp;
  double d_rn;
  double d_rm;
  double d_poslla[3];
  float f_vel_ned[3];
  float f_att_rph[3];
  float f_fn[3];
  float f_gravity_vct[3];
  float f_omega_en[3];
  float f_omega_ie[3];
  float f_omega_ibb[3];
  float f_rotmat_b2n[3][3];
  float f_delta_vel_ned[3];
  float f_rotmat_misalign[3][3];
  float f_rotmat_v2b[3][3];
  float f_quat_misalign[4];
  float f_quat_b2n[4];
  double d_quat_n2e[4];
  float f_prev_quat_b2n[4];
  float f_gyrobias[3];
  float f_acclbias[3];
  float f_wspd_sf[4];
  float f_odospd_sf;
  float f_mis_align[3];
  float f_clk_err[3];
  float f_clkdiff_err;
  int8_t  s_axiscfg[9];
  uint8_t u_izupt_counter;
  uint8_t u_gzupt;
  uint8_t u_inited;
  uint8_t u_imu_shortloss_flag;
  double d_imu_shortloss_time;
} NavParams_t;

typedef struct
{
  double d_imu_t;
  double d_prebds_update_time;
  double d_preins_update_time;
  double d_bds_outage_start_time;
  double d_bds_outage_end_time;
  float f_kf_meas[6];
  float f_bds_dr_pos_diff;
  float f_lambda_pos;
  float f_lambda_vel;
  MechanNode_t* pz_bds_node;	
  MechanNode_t* pz_veh_node;
  double d_lla_prepps[3];
  uint8_t u_posflag;
  uint8_t u_last_posflag;
  uint8_t u_bds_valid;
  uint8_t u_ins_valid;
  uint8_t u_kf_update;
  uint8_t u_kf_q_changed;
  uint32_t q_update_type;
} SyncObsNav_t;

typedef struct
{
  double d_last_imu_time;
  double d_imu_buf[10];
  double d_nav_start_time;
  double d_timeout_beginning;
  double d_timeout_zupttime;
  double d_zupt_pos[3];
  float f_zupt_att[3];
  float f_zupt_vel[3];
  double d_zupt_reinit_time;
  float f_imu_sampling_dt;
  float f_gnss_sampling_dt;
  GNSSQCPara_t* pz_qc;
  SyncObsNav_t* pz_obs;
  SysErrModel_t* pz_model;
  NavConfig_t* pz_config;
  MechanNode_t* pz_cur_mech;
  uint8_t u_last_update_type;
  uint8_t u_init_nav;
  uint8_t u_ins_status;
  uint8_t u_gzbias_converged;
  uint8_t u_init_align;
  uint8_t u_align_method; /* 1.align with gnss, 2.align with history */
  uint8_t u_ins_update_mask;
  uint8_t u_kf_converged;
  uint8_t u_ins_timeout;
  uint8_t u_keep_pos;
  uint8_t u_adjust_syscov;
  uint8_t u_psr_used_satnum;
  AsensingSatPsrRaw_t z_psr_used_data[MAX_RAW_OBS];
  uint8_t u_cp_used_satnum;
  AsensingSatCpRaw_t z_cp_used_data[MAX_RAW_OBS];
  uint8_t u_init_nav_tdcp;
  uint8_t u_inspl_init_flag;
} InertialNav_t;

typedef struct
{
  uint8_t u_kf_update;
  uint32_t q_update_type;
  float f_errstate[9];
  double d_gnssposmeas_time;
  float f_pos_meas_z[3];
  float f_measnoise_pos[3];
  float f_pos_lambda;
  uint8_t u_posflag;
} EkfPosFeedback_t;

void fusion_reset(void);
void fusion_navsys_init(void);
void fusion_navsys_partial_init(void);
uint8_t is_izupt(void);
uint8_t* is_gzupt(void);
NavParams_t* fusion_get_navpara(void);
InertialNav_t* fusion_get_inertial_nav(void);
void kf_transmat_update(double d_timestamp, NavParams_t* pz_nav);
void kf_transmat_tdcp(double pd_meas[], InertialNav_t* pz_inav);
int8_t ins_handle_meas(double* pd_meas, INSResults_t* pz_ins);
int8_t fusion_mechan_handler(const double* pd_imu_cur, NavParams_t* pz_nav, MechanNode_t* pz_node);
int8_t bds_qc_procedure(double pd_bdsmea[], InertialNav_t* pz_inav);
uint8_t is_fusion_valid(void);
#ifdef OPEN_INS_INTEGRITY
EkfPosFeedback_t* fusion_get_ekf_dpos(void);
#endif

#endif // !__FUSION_PROC_H__
