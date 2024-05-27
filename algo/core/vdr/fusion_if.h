/**@file        fusion_if.h
 * @brief		fusion internal interface header file
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

#ifndef __FUSION_IF_H__
#define __FUSION_IF_H__

#include "fusion_api.h"
#include "fusion_global.h"

#define BLH_XPOS2IMU 0
#define BLH_IMU2XPOS 1

#define IMUS_BUF_SIZE 32
#define WHLS_BUF_SIZE 32

#define fusion_version "libver_23_01_11_12" 

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t u_pos_err_esti;
  uint8_t u_vel_err_esti;
  uint8_t u_att_err_esti;
  uint8_t u_gbias_err_esti;
  uint8_t u_abias_err_esti;
  uint8_t u_misangle_err_esti;
  uint8_t u_rearwheel_sf_err_esti;
  uint8_t u_clock_err_esti;
  uint8_t u_clock_diff_err_esti;
  uint8_t u_la_imu2gnss_err_esti;
  uint8_t u_mis_dualant_err_esti;
} SysErrType_t;

typedef struct
{
  AsensingFusionInitPara_t z_init_para;
  SysErrType_t z_syserrtype;
  uint8_t u_sysdim;
  uint8_t u_meadim;
  float f_sampling_dt;
  float f_gnss_dt;
  float f_veh_calc_dt;
  uint8_t u_calc_rate;
  uint8_t u_cache_num;
  uint8_t config_init_status;
} FusionAlgoConfig_t;

typedef struct
{
  AsensingIMU_t z_imus[IMUS_BUF_SIZE];
  uint8_t u_count;
  uint8_t u_read_index;
  uint8_t	u_write_index;
} IMUInputBuf_t;

typedef struct
{
  AsensingWhpulse_t z_whls[WHLS_BUF_SIZE];
  uint8_t u_read_index;
  uint8_t	u_write_index;
} WHLInputBuf_t;

void fusion_if_initial(void);
float* lvrm_get_imu2gnss(void);
float* lvrm_get_imu2rearmid(void);
float* lvrm_get_rear2rear(void);
float* mis_get_imu2dualant(void);
float* mis_get_imu2vehicle(void);
AsensingGNSSPsrMeas_t* fusion_get_gnss_psr_meas(void);
AsensingGNSSCpMeas_t* fusion_get_gnss_cp_meas(void);
AsensingGNSSPara_t* fusion_get_gnss_para(void);
INSResults_t* fusion_get_ins_results(void);
FusionAlgoConfig_t* fusion_get_algconfig(void);
AsensingFusionApiCallback_t* fusion_get_callback();
int8_t fusion_ins_entry(AsensingIMU_t* pz_imu, INSResults_t* pz_ins);
void imu_put_data_ringbuf(AsensingIMU_t* pz_imu);
int8_t imu_get_data_ringbuf(AsensingIMU_t* pz_imu);
void whpls_put_data_ringbuf(AsensingWhpulse_t* pz_wheel);
int8_t whpls_get_data_ringbuf(AsensingWhpulse_t* pz_whpls);
AsensingPPS_t* fusion_get_pps_para(void);
uint8_t fusion_check_pps_nearby(uint64_t d_timestamp_ms, int* val);
void compensate_pos_lever_arm(double* pd_blh, float* pf_lvrm, uint8_t u_bidirect);
void compensate_vel_lever_arm(float* pf_vel, float* pf_lvrm, uint8_t u_bidirect);
HistoryInsResults_t* fusion_get_historyInfo(void);
void ins_softreset(void);
void ins_hardreset(void);
#ifdef __cplusplus
}
#endif

#endif // !__FUSION_IF_H__
