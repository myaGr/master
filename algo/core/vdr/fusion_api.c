/**@file        asensing_fusion_api.c
 * @brief		fusion algorithm api file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "fusion_api.h"
#include "fusion_config.h"
#include "fusion_if.h"
#include "fusion_gnss.h"
#include "fusion_proc.h"
#include "fusion_log.h"
#include "fusion_data_encode.h"
#include "fusion_integrity.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static AsensingFusionApiCallback_t gz_api_cb;
static FusionAlgoConfig_t gz_algo_config;
static AsensingGNSSPara_t gz_gnss_para;
static AsensingPPS_t gz_pps_para;
static AsensingGNSSPsrMeas_t gz_gnss_psr_meas;
static AsensingGNSSCpMeas_t gz_gnss_cp_meas;
static AsensingFusionResults_t gz_fusion_results;
static HistoryInsResults_t gz_historyInfo;
static float gf_imu2gnss_lr[3] = { 0 };
static float gf_imu2rearmid_lr[3] = { 0 };
static float gf_rear2front_lr = 0;
static float gf_two_rear_lr = 0;
static float gf_att_mis_b2g[3] = { 0 };
static float gf_att_mis_b2v[3] = { 0 };

FusionAlgoConfig_t* fusion_get_algconfig(void)
{
  return &gz_algo_config;
}

AsensingFusionApiCallback_t* fusion_get_callback()
{
  return &gz_api_cb;
}

int8_t fusion_check_para_config(AsensingFusionInitPara_t* pz_init_para, FusionAlgoConfig_t* pz_config)
{
  int8_t s_ret_val = 1;

  if (pz_init_para->e_sampling_rate < TYPE_RATE_50_HZ || pz_init_para->e_sampling_rate > TYPE_RATE_200_HZ)
  {
    s_ret_val = -1;
  }
  else
  {
    pz_config->f_sampling_dt = 1.f / pz_init_para->e_sampling_rate;
  }

  if (pz_init_para->e_output_rate > pz_init_para->e_sampling_rate || pz_init_para->e_output_rate < TYPE_RATE_1_HZ)
  {
    s_ret_val = -1;
  }

  if (pz_init_para->e_gnss_rate < TYPE_RATE_1_HZ || pz_init_para->e_gnss_rate > TYPE_RATE_50_HZ)
  {
    s_ret_val = -1;
  }
  else
  {
    pz_config->f_gnss_dt = 1.0f / pz_init_para->e_gnss_rate;
  }

  if (pz_init_para->u_whspd_mode > 0)
  {
    if (pz_init_para->e_whspd_rate > 0)
    {
      pz_config->f_veh_calc_dt = pz_init_para->e_gnss_rate < pz_init_para->e_whspd_rate ?
        1.0f / pz_init_para->e_gnss_rate : 1.f / pz_init_para->e_whspd_rate;
    }
    else
    {
      s_ret_val = -1;
    }
  }
  else
  {
    pz_config->f_veh_calc_dt = 1.0f / pz_init_para->e_gnss_rate;
  }

  if (pz_init_para->e_output_rate == TYPE_RATE_1_HZ)
  {
    pz_config->u_calc_rate = TYPE_RATE_5_HZ;
  }
  else
  {
    if (pz_init_para->e_gnss_rate > pz_init_para->e_output_rate)
    {
      pz_config->u_calc_rate = pz_init_para->e_gnss_rate;
    }
    else
    {
      pz_config->u_calc_rate = pz_init_para->e_output_rate;
    }
  }
  if (pz_config->u_calc_rate != 0)
  {
    pz_config->u_cache_num = pz_init_para->e_sampling_rate / pz_config->u_calc_rate;
  }

  if (pz_init_para->u_outputpos_flag != (uint8_t)TYPE_OUTPUTPOS_GNSS && 
    pz_init_para->u_outputpos_flag != (uint8_t)TYPE_OUTPUTPOS_REARMID &&
    pz_init_para->u_outputpos_flag != (uint8_t)TYPE_OUTPUTPOS_IMU)
  {
    pz_config->z_init_para.u_outputpos_flag = (uint8_t)TYPE_OUTPUTPOS_GNSS; // output to gnss antenna center
  }    

  if (pz_init_para->u_imu_type == (uint8_t)TYPE_IMU_DEFAULT)
  {
    pz_config->z_init_para.u_imu_type = (uint8_t)TYPE_IMU_AG051;
  }
  return s_ret_val;
}

/**
 * @brief: VDR INIT API
 * @param[in]: pz_init_para - init param; pz_api_cb - call back function
 * @param[out]: None
 * @return: init flag
*/
int8_t asensing_fusion_init(AsensingFusionInitPara_t* pz_init_para, AsensingFusionApiCallback_t* pz_api_cb)
{
  int8_t s_ret_val = -1;
  memset(&gz_algo_config, 0, sizeof(gz_algo_config));
  gz_algo_config.config_init_status = INS_TRUE;

  gz_api_cb.fusion_log = pz_api_cb->fusion_log;
  gz_api_cb.fusion_report = pz_api_cb->fusion_report;
  gz_api_cb.fusion_report_history = pz_api_cb->fusion_report_history;

  gz_algo_config.z_init_para.q_vdr_mode = pz_init_para->q_vdr_mode;
  gz_algo_config.z_init_para.q_align_mode = pz_init_para->q_align_mode;

  gz_algo_config.z_init_para.e_output_rate = pz_init_para->e_output_rate;
  gz_algo_config.z_init_para.e_sampling_rate = pz_init_para->e_sampling_rate;    
  gz_algo_config.z_init_para.e_gnss_rate = pz_init_para->e_gnss_rate;
  gz_algo_config.z_init_para.u_whspd_mode = pz_init_para->u_whspd_mode;
  gz_algo_config.z_init_para.e_whspd_rate = pz_init_para->e_whspd_rate;
  gz_algo_config.z_init_para.u_outputpos_flag = pz_init_para->u_outputpos_flag;
  gz_algo_config.z_init_para.u_dr_time = pz_init_para->u_dr_time;
  gz_algo_config.z_init_para.u_imu_type = pz_init_para->u_imu_type;

  s_ret_val = fusion_check_para_config(pz_init_para, &gz_algo_config);

  gz_algo_config.u_meadim = 10;

  if (gz_algo_config.z_init_para.q_vdr_mode == TYPE_VDR_LC)
  {
    gz_algo_config.u_sysdim = 20;

    gz_algo_config.z_syserrtype.u_pos_err_esti = 0;
    gz_algo_config.z_syserrtype.u_vel_err_esti = 3;
    gz_algo_config.z_syserrtype.u_att_err_esti = 6;
    gz_algo_config.z_syserrtype.u_gbias_err_esti = 9;
    gz_algo_config.z_syserrtype.u_abias_err_esti = 12;
    gz_algo_config.z_syserrtype.u_misangle_err_esti = 15;
    gz_algo_config.z_syserrtype.u_rearwheel_sf_err_esti = 18;
  }
  else if (fusion_get_algconfig()->z_init_para.q_vdr_mode == TYPE_VDR_TC_PRDR)
  {
#if CLOCK_DRIFT_ENABLE
    gz_algo_config.u_sysdim = 24;
#else
    gz_algo_config.u_sysdim = 23;

    gz_algo_config.z_syserrtype.u_pos_err_esti = 0;
    gz_algo_config.z_syserrtype.u_vel_err_esti = 3;
    gz_algo_config.z_syserrtype.u_att_err_esti = 6;
    gz_algo_config.z_syserrtype.u_gbias_err_esti = 9;
    gz_algo_config.z_syserrtype.u_abias_err_esti = 12;
    gz_algo_config.z_syserrtype.u_misangle_err_esti = 15;
    gz_algo_config.z_syserrtype.u_rearwheel_sf_err_esti = 18;
    gz_algo_config.z_syserrtype.u_clock_err_esti = 20;
#endif
  }
  else if (fusion_get_algconfig()->z_init_para.q_vdr_mode == TYPE_VDR_TC_CP)
  {
    gz_algo_config.u_sysdim = 21;

    gz_algo_config.z_syserrtype.u_pos_err_esti = 0;
    gz_algo_config.z_syserrtype.u_vel_err_esti = 3;
    gz_algo_config.z_syserrtype.u_att_err_esti = 6;
    gz_algo_config.z_syserrtype.u_gbias_err_esti = 9;
    gz_algo_config.z_syserrtype.u_abias_err_esti = 12;
    gz_algo_config.z_syserrtype.u_misangle_err_esti = 15;
    gz_algo_config.z_syserrtype.u_rearwheel_sf_err_esti = 18;
    gz_algo_config.z_syserrtype.u_clock_diff_err_esti = 20;
  }
  else if (fusion_get_algconfig()->z_init_para.q_vdr_mode == TYPE_VDR_TC)
  {
#if CLOCK_DRIFT_ENABLE
    gz_algo_config.u_sysdim = 25;
#else
    gz_algo_config.u_sysdim = 24;

    gz_algo_config.z_syserrtype.u_pos_err_esti = 0;
    gz_algo_config.z_syserrtype.u_vel_err_esti = 3;
    gz_algo_config.z_syserrtype.u_att_err_esti = 6;
    gz_algo_config.z_syserrtype.u_gbias_err_esti = 9;
    gz_algo_config.z_syserrtype.u_abias_err_esti = 12;
    gz_algo_config.z_syserrtype.u_misangle_err_esti = 15;
    gz_algo_config.z_syserrtype.u_rearwheel_sf_err_esti = 18;
    gz_algo_config.z_syserrtype.u_clock_err_esti = 20;
    gz_algo_config.z_syserrtype.u_clock_diff_err_esti = 23;
#endif
  }
  else
  {
    gz_algo_config.u_sysdim = 20; 

    gz_algo_config.z_syserrtype.u_pos_err_esti = 0;
    gz_algo_config.z_syserrtype.u_vel_err_esti = 3;
    gz_algo_config.z_syserrtype.u_att_err_esti = 6;
    gz_algo_config.z_syserrtype.u_gbias_err_esti = 9;
    gz_algo_config.z_syserrtype.u_abias_err_esti = 12;
    gz_algo_config.z_syserrtype.u_misangle_err_esti = 15;
    gz_algo_config.z_syserrtype.u_rearwheel_sf_err_esti = 18;
  }
  fusion_if_initial();
#ifdef _WIN32
  fusion_log_initial((void*)gz_api_cb.fusion_log);
#endif
  memset(&gz_fusion_results,0,sizeof(gz_fusion_results));

  if (s_ret_val < 0)
  {
    gz_algo_config.config_init_status = INS_FALSE;
  }
  return s_ret_val;
}

/**
 * @brief xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 * @param[in] para1
 * @param[out] para2
 * @return
*/
int8_t asensing_fusion_start()
{
  return 0;
}

/**
 * @brief xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 * @param[in] para1
 * @param[out] para2
 * @return
*/
int8_t asensing_fusion_stop()
{
  return 0;
}

/**
 * @brief xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 * @param[in] none
 * @param[out] none
 * @return
*/
int8_t asensing_fusion_release()
{
  return 0;
}

/**
 * @brief get the pseudo range observation struct
 * @param[in] none
 * @param[out] none
 * @return PR struct
*/
AsensingGNSSPsrMeas_t* fusion_get_gnss_psr_meas(void)
{
  return &gz_gnss_psr_meas;
}

/**
 * @brief get the carrier observation struct
 * @param[in] none
 * @param[out] none
 * @return CP struct
*/
AsensingGNSSCpMeas_t* fusion_get_gnss_cp_meas(void)
{
  return &gz_gnss_cp_meas;
}

/**
 * @brief get gnss solution struct
 * @param[in] none
 * @param[out] none
 * @return GNSS struct
*/
AsensingGNSSPara_t* fusion_get_gnss_para(void)
{
  return &gz_gnss_para;
}

/**
 * @brief get pps struct
 * @param[in] none
 * @param[out] none
 * @return pps struct
*/
AsensingPPS_t* fusion_get_pps_para(void)
{
  return &gz_pps_para;
}

/**
 * @brief get IMU to GNSS lever arm
 * @param[in] none
 * @param[out] none
 * @return imu2gnss lever arm array
*/
float* lvrm_get_imu2gnss(void)
{
  return gf_imu2gnss_lr;
}

/**
 * @brief get IMU to RearMid lever arm
 * @param[in] none
 * @param[out] none
 * @return imu2rear lever arm array
*/
float* lvrm_get_imu2rearmid(void)
{
  return gf_imu2rearmid_lr;
}

/**
 * @brief get distance between rear wheels
 * @param[in] none
 * @param[out] none
 * @return rear2rear distance
*/
float* lvrm_get_rear2rear(void)
{
  return &gf_two_rear_lr;
}

float* mis_get_imu2dualant(void)
{
	return gf_att_mis_b2g;
}

float* mis_get_imu2vehicle(void)
{
	return gf_att_mis_b2v;
}

/**
 * @brief get history info
 * @param[in] para1
 * @param[out] para2
 * @return historyInfo struct
*/
HistoryInsResults_t* fusion_get_historyInfo(void)
{
  return &gz_historyInfo;
}

char* asensing_fusion_version(void)
{
  return fusion_version;
}

void fusion_output_imu(AsensingIMU_t* pz_imu)
{
  uint8_t u_buf[128] = { 0 };
  uint32_t q_size = 0;
  q_size = data_encode_BDDBAA(pz_imu, u_buf);

  if (gz_api_cb.fusion_log != NULL)
  {
    gz_api_cb.fusion_log(u_buf, q_size);
  }
}

void fusion_output_wheelpulse(AsensingWhpulse_t* pz_wheel)
{
  uint8_t u_buf[128] = { 0 };
  uint32_t q_size = 0;
  q_size = data_encode_BDDBCC(pz_wheel, u_buf);

  if (gz_api_cb.fusion_log != NULL)
  {
    gz_api_cb.fusion_log(u_buf, q_size);
  }
}

void fusion_output_pps(AsensingPPS_t* pz_pps)
{
  uint8_t u_buf[128] = { 0 };
  uint32_t q_size = 0;
  q_size = data_encode_BDDBDD(pz_pps, u_buf);

  if (gz_api_cb.fusion_log != NULL)
  {
    gz_api_cb.fusion_log(u_buf, q_size);
  }
}

void asensing_fusion_inject_wheel(AsensingWhpulse_t* pz_wheel)
{
  if (pz_wheel)
  {
    whpls_put_data_ringbuf(pz_wheel);
#ifndef PLAYBACK_MODE
    fusion_output_wheelpulse(pz_wheel);
#endif
  }
  return;
}

/**
 * @brief Interface for injecting GNSS
 * @param[in] pz_gnss_para - gnss info struct
 * @param[out] none
 * @return
*/
void asensing_fusion_inject_gnss_para(AsensingGNSSPara_t* pz_gnss_para)
{
  if (pz_gnss_para)
  {
    gnss_put_data_ringbuf(pz_gnss_para);
  }
}

/**
 * @brief Interface for injecting Pr measurements
 * @param[in] pz_gnss_meas - pr measurements struct
 * @param[out] none
 * @return
*/
void asensing_fusion_inject_gnss_psr_meas(AsensingGNSSPsrMeas_t* pz_gnss_meas)
{
  memcpy(&gz_gnss_psr_meas, pz_gnss_meas, sizeof(gz_gnss_psr_meas));
}

/**
 * @brief Interface for injecting Cp measurements
 * @param[in] pz_gnss_cp_meas - cp measurements struct
 * @param[out] none
 * @return
*/
void asensing_fusion_inject_gnss_cp_meas(AsensingGNSSCpMeas_t* pz_gnss_cp_meas)
{
  memcpy(&gz_gnss_cp_meas, pz_gnss_cp_meas, sizeof(gz_gnss_cp_meas));
}

/**
 * @brief Interface for injecting gnss to imu lever arm
 * @param[in] f_gnss2imu - the lever arm
 * @param[out] none
 * @return
*/
void asensing_lvrm_set_gnss2imu(float f_gnss2imu[3])
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();

  memcpy(gf_imu2gnss_lr, f_gnss2imu, sizeof(gf_imu2gnss_lr));

#ifdef LA_IMU2GNSS_ERR_ESTI
  pz_navconfig->u_i2g_opt = 1;
#else
  if (fabs(f_gnss2imu[0]) > 0 || fabs(f_gnss2imu[1]) > 0 ||
      fabs(f_gnss2imu[2]) > 0)
  {
    pz_navconfig->u_i2g_opt = 1;
  }
  else
  {
    pz_navconfig->u_i2g_opt = 0;
  }
#endif
}

/**
 * @brief Interface for injecting rear mid to imu lever arm
 * @param[in] f_rearmid2imu - the lever arm
 * @param[out] none
 * @return
*/
void asensing_lvrm_set_rearmid2imu(float f_rearmid2imu[3])
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();

  memcpy(gf_imu2rearmid_lr, f_rearmid2imu, sizeof(gf_imu2rearmid_lr));

#ifdef LA_IMU2REARMID_ERR_ESTI
  pz_navconfig->u_ws_opt |= TYPE_MASK_LVRM_REAR_MID;
#else
  if (fabs(f_rearmid2imu[0]) > 0 || fabs(f_rearmid2imu[1]) > 0 ||
      fabs(f_rearmid2imu[2]) > 0)
  {
    pz_navconfig->u_ws_opt |= TYPE_MASK_LVRM_REAR_MID;
  }
  else
  {
    pz_navconfig->u_ws_opt &= (~TYPE_MASK_LVRM_REAR_MID);
  }
#endif
}

/**
 * @brief Interface for injecting rear to front lever arm
 * @param[in] f_rear2front - the lever arm
 * @param[out] none
 * @return
*/
void asensing_lvrm_set_rear2front(float f_rear2front)
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();

  gf_rear2front_lr = f_rear2front;

  if (fabs(f_rear2front) > 0)
  {
    pz_navconfig->u_ws_opt |= TYPE_MASK_LVRM_FRONT_REAR;
  }
  else
  {
    pz_navconfig->u_ws_opt &= (~TYPE_MASK_LVRM_FRONT_REAR);
  }
}

/**
 * @brief Interface for injecting rear to rear lever arm
 * @param[in] f_2rear - the lever arm
 * @param[out] none
 * @return
*/
void asensing_lvrm_set_tworears(float f_2rear)
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();

  gf_two_rear_lr = f_2rear;

#ifdef WHEELBASE_ERR_ESTI
  pz_navconfig->u_ws_opt |= TYPE_MASK_LVRM_TWO_REAR;
#else
  if (fabs(f_2rear) > 0)
  {
    pz_navconfig->u_ws_opt |= TYPE_MASK_LVRM_TWO_REAR;
  }
  else
  {
    pz_navconfig->u_ws_opt &= (~TYPE_MASK_LVRM_TWO_REAR);
  }
#endif
}

void asensing_mis_set_imu2dualant(float f_euler[3])
{
	NavConfig_t* pz_navconfig = fusion_get_navconfig();

	memcpy(gf_att_mis_b2g, f_euler, sizeof(gf_att_mis_b2g));

	if (fabs(f_euler[0]) > 0.0 || fabs(f_euler[1]) > 0.0 ||
		fabs(f_euler[2]) > 0.0)
	{
		pz_navconfig->u_misb2g_opt = 1;
	}
	else
	{
		pz_navconfig->u_misb2g_opt = 0;
	}
#ifndef PLAYBACK_MODE
	/* fusion_output_mis_dualant(gf_imu2gnss_lr); */
#endif
}

void asensing_mis_set_imu2vehicle(float f_euler[3])
{
	NavConfig_t* pz_navconfig = fusion_get_navconfig();

	memcpy(gf_att_mis_b2v, f_euler, sizeof(gf_att_mis_b2v));

	if (fabs(f_euler[0]) > 0.0 || fabs(f_euler[1]) > 0.0 ||
		fabs(f_euler[2]) > 0.0)
	{
		pz_navconfig->u_misb2v_opt = 1;
	}
	else
	{
		pz_navconfig->u_misb2v_opt = 0;
	}
#ifndef PLAYBACK_MODE
	/* fusion_output_mis_dualant(gf_imu2gnss_lr); */
#endif
}

uint8_t fusion_check_pps_nearby(uint64_t d_timestamp_ms, int* val)
{
  uint8_t u_val = 0;
  int gnss_sampling_dt = 1000 / fusion_get_algconfig()->z_init_para.e_gnss_rate;
  int imu_sampling_dt = 1000 / fusion_get_algconfig()->z_init_para.e_sampling_rate;
  *val = (d_timestamp_ms % gnss_sampling_dt);
  if (*val < imu_sampling_dt || (gnss_sampling_dt - *val < imu_sampling_dt))
  {
    u_val = 1;
  }
  return u_val;
}

void fusion_output_final_results(INSResults_t* pz_ins)
{
#if 0
  if (pz_ins->e_drposflag == TYPE_DR_POS_FLAG_FUSION || pz_ins->e_drposflag == TYPE_DR_POS_FLAG_DR_OLY)
  {
    gz_fusion_results.d_latitude = pz_ins->d_latitude;
    gz_fusion_results.d_longitude = pz_ins->d_longitude;
    gz_fusion_results.f_altitude = pz_ins->f_altitude;
    gz_fusion_results.f_heading = pz_ins->f_heading;
    gz_fusion_results.f_pitch = pz_ins->f_pitch;
    gz_fusion_results.f_roll = pz_ins->f_roll;
    gz_fusion_results.f_speed = pz_ins->f_speed;
    gz_fusion_results.e_posflag = fusion_get_gnss_para()->e_posflag > 0 ? fusion_get_gnss_para()->e_posflag : TYPE_POS_FLAG_FUSION;
  }
  else
  {
    gz_fusion_results.d_latitude = pz_ins->d_latitude;
    gz_fusion_results.d_longitude = pz_ins->d_longitude;
    gz_fusion_results.f_altitude = pz_ins->f_altitude;
    gz_fusion_results.f_heading = pz_ins->f_heading;
    gz_fusion_results.f_pitch = 0;
    gz_fusion_results.f_roll = 0;
    gz_fusion_results.f_speed = pz_ins->f_speed;
    gz_fusion_results.e_posflag = fusion_get_gnss_para()->e_posflag;
  }
  gz_fusion_results.t_timestamp = pz_ins->t_timestamp;

#endif

  if (gz_api_cb.fusion_report)
  {
    gz_api_cb.fusion_report(pz_ins);
  }
}

int8_t fusion_imu_process(AsensingIMU_t* pz_imu)
{
  int8_t s_ret_val = TYPE_STATUS_FAILURE;
  INSResults_t* pz_ins = fusion_get_ins_results();

  s_ret_val = fusion_ins_entry(pz_imu, pz_ins);

  if (s_ret_val == TYPE_STATUS_DATA_READY)
  {
#ifdef OPEN_INS_INTEGRITY
    SysErrModel_t* pz_model = fusion_get_sysmodel();

    fusion_protection_level_calculation(pz_model, pz_ins);
#endif
    fusion_output_final_results(pz_ins);

    pz_ins->q_kfmeastype = 0; /* reset kfmeastype after INS result output */
  }
  return s_ret_val;
}

void asensing_fusion_inject_pps(AsensingPPS_t* pz_pps)
{
  if (pz_pps)
  {
    memcpy(&gz_pps_para, pz_pps, sizeof(AsensingPPS_t));
#ifndef PLAYBACK_MODE
    fusion_output_pps(pz_pps);
#endif
  }
}

void asensing_fusion_inject_history(HistoryInsResults_t* pz_history)
{
  if (pz_history)
  {
    memcpy(&gz_historyInfo, pz_history, sizeof(HistoryInsResults_t));

    if (fabs(gz_historyInfo.d_lat) <= 1e-6 || fabs(gz_historyInfo.d_lon) <= 1e-6 ||
        fabs(gz_historyInfo.d_lat) > 90.0 || fabs(gz_historyInfo.d_lon) > 180.0)
    {
      gz_historyInfo.u_validType = gz_historyInfo.u_validType & (~0x02);
    }
  }
  return;
}

void asensing_fusion_inject_imu(AsensingIMU_t* pz_imu)
{
  if (pz_imu != NULL)
  {
    imu_put_data_ringbuf(pz_imu);
#ifdef HARDWARE_PPS
    int val = 0;
    if (fusion_check_pps_nearby(pz_imu->t_timestamp,&val) > 0)
    {
      AsensingPPS_t z_pps;
      z_pps.t_timestamp = pz_imu->t_timestamp - val;
      z_pps.w_valid = 1;
      asensing_fusion_inject_pps(&z_pps);
    }
#endif
    asensing_fusion_rt_handler();
#ifndef PLAYBACK_MODE
    fusion_output_imu(pz_imu);
#endif
  }
}

int8_t asensing_fusion_rt_handler(void)
{
  int8_t s_ret_val = TYPE_STATUS_FAILURE;

  if (gz_algo_config.config_init_status == INS_TRUE)
  {
    do
    {
      AsensingIMU_t z_imu = { 0 };
      if (imu_get_data_ringbuf(&z_imu) < 0)
      {
          break;
      }
      s_ret_val = fusion_imu_process(&z_imu);
    } while (1);
  }
  return s_ret_val;
}

void asensing_fusion_bg_handler(void)
{

}