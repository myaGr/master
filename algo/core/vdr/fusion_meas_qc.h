
/**@file        fusion_meas_qc.c
 * @brief		measurements quality control header file
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

#ifndef __FUSION_MEAS_QC_H__
#define __FUSION_MEAS_QC_H__

#include <stdint.h>

#define QC_REDUCE_NUM      30                    /* Weight reduction number */
#define QC_REJECT_NUM      20U                   /* reject GNSS number */
#define QC_CONVER_THRE     120U                  /* Convergence threshold */
#define QC_NO_OUTAGE       2                     /* GNSS NO outage num */
#define QC_BDS_OUTAGE      30                    /* GNSS outage num */
#define QC_REDUCE_THRE     10                    /* Chi-square test weight reduction threshold */
#define QC_VALID_NUM       700U                  /* GNSS VALID NUM */
#define QC_BIAS_THRE       0.008                 /* gyro bias exceed limit threshold : deg/s */
#define MAX_GYRO_BIAS      999.0f
#define MIN_GYRO_BIAS     -999.0f
#define QC_START_TIME      (60 + QC_CONVER_THRE)

#define LAMDA_RTK_FIX_REJECT_THRESHOLD   200.f

typedef struct
{
  double d_last_check_time;
  double d_last_lowspd_time;
  double d_bds_zupt_time_all;
  double d_pre_good_bds_time;
  double d_cont_rtk_fix_time;
  double d_badbds_start_time;
  double d_badbds_end_time;
  float f_gzbias_buffer[QC_CONVER_THRE];
  float f_lambda_pos_thre1;
  float f_lambda_pos_thre2;
  float f_lambda_vel_thre1;
  float f_lambda_vel_thre2;
  float f_gbias_check_thre;
  float f_pos_lambda;
  float f_vel_lambda;
  float f_beta;
  float f_adaptive_noise[6];
  uint32_t q_good_bds_num;
  uint32_t f_gbias_counter;
  uint32_t q_bds_reject_thre;
  uint32_t q_bds_converged_thre;
  uint32_t q_bds_valid_counter;	
  uint32_t q_bds_reject_counter;
  uint32_t q_last_bds_reject_num;
  uint32_t q_cont_rtk_fix_num;
  uint32_t q_cont_rtk_thres;
  uint32_t q_cont_goodbds_thres;
  uint8_t u_gbias_converged;
  uint8_t u_rtk_check_flag;
  uint8_t  u_adanoise_inited;
  uint8_t  u_adanoise_valid;
  uint8_t u_bds_rate;
} GNSSQCPara_t;

void qc_initial(NavConfig_t* pz_navconfig);
GNSSQCPara_t* fusion_get_qc_para(void);

#endif // !__FUSION_MEAS_QC_H__
