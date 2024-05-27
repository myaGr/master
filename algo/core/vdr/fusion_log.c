/**@file        fusion_log.c
 * @brief		fusion log file
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

#include "fusion_log.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static fusion_log_cb gz_fusion_log_cb = NULL;
static uint32_t gq_log_output_msg_filter = 0;

void fusion_log_initial(void* pz_log_cb)
{
  fusion_log_callback_register((fusion_log_cb)pz_log_cb);
  fusion_log_filter_set(LOG_MSG_ALL);
}

void fusion_log_callback_register(fusion_log_cb z_fusion_cb)
{
  gz_fusion_log_cb = z_fusion_cb;
}

void fusion_log(uint32_t q_msg_filter, const uint8_t* pu_format, ...)
{
  int8_t s_enable_output = -1;

  if ((uint32_t)LOG_MSG_DISABLED != q_msg_filter)
  {
    if ((q_msg_filter & (uint32_t)LOG_MSG_NMEA) > 0U)
    {
      s_enable_output = 1;
    }
    else if (gq_log_output_msg_filter > 0U)
    {
      if ((gq_log_output_msg_filter & q_msg_filter) > 0U)
      {
        s_enable_output = 1;
      }
    }
    else 
    {
      s_enable_output = -1;
    }
  }

  if (s_enable_output > 0)
  {
    char msg[LOG_OUTPUT_MAX_MSG_LEN];

    va_list arg_list;

    memset(msg, 0, sizeof(msg));

    va_start(arg_list, pu_format);

    (void)vsnprintf(msg, LOG_OUTPUT_MAX_MSG_LEN, pu_format, arg_list);

    va_end(arg_list);

    if (gz_fusion_log_cb != NULL)
    {
      gz_fusion_log_cb(msg, strlen(msg));
    }
  }
}

void fusion_log_filter_set(uint32_t q_output_mask)
{
  gq_log_output_msg_filter = q_output_mask;
} 


void fusion_log_filter_get(uint32_t* pq_output_mask)
{
  if (pq_output_mask != NULL)
  {
    *pq_output_mask = gq_log_output_msg_filter;
  }
} 

void vdr_save_imu_log(AsensingIMU_t* pz_AsensingIMU)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  uint8_t data[LOG_BDDB_MAX_LENGTH] = { 0 };
  uint32_t len = data_encode_BDDB0A(pz_AsensingIMU, data);
  BDDB_LOG(DLOG_VDR_BDDB, data, len);
  DATA_LOG(DLOG_VDR, "$GPIMU,%llu,%f,%f,%f,%f,%f,%f,%f,%d\n",
    pz_AsensingIMU->t_timestamp,
    pz_AsensingIMU->f_gyro[0], pz_AsensingIMU->f_gyro[1], pz_AsensingIMU->f_gyro[2],
    pz_AsensingIMU->f_accl[0], pz_AsensingIMU->f_accl[1], pz_AsensingIMU->f_accl[2],
    pz_AsensingIMU->f_temp, pz_AsensingIMU->u_selfck);
#endif
}

void vdr_save_wheel_log(AsensingWhpulse_t* pz_AsensingWheel)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  uint8_t data[LOG_BDDB_MAX_LENGTH] = { 0 };
  uint32_t len = data_encode_BDDB20(pz_AsensingWheel, data);
  BDDB_LOG(DLOG_VDR_BDDB, data, len);
  DATA_LOG(DLOG_VDR, "$GPWHL,%llu,%d,%llu,%d,%d,%d,%d,%d\n",
    pz_AsensingWheel->t_shifter_ms, pz_AsensingWheel->e_gear, pz_AsensingWheel->t_timestamp,
    pz_AsensingWheel->q_fl_whpulse, pz_AsensingWheel->q_fr_whpulse, pz_AsensingWheel->q_rl_whpulse, pz_AsensingWheel->q_rr_whpulse,
    1);
#endif
}

void vdr_save_gnss_log(AsensingGNSSPara_t* pz_AsensingGNSS)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  uint8_t data[LOG_BDDB_MAX_LENGTH] = { 0 };
  uint32_t len = data_encode_BDDB10(pz_AsensingGNSS, data);
  BDDB_LOG(DLOG_VDR_BDDB, data, len);
  /* 0E GNSS integrity */
  len = data_encode_BDDB0E(pz_AsensingGNSS, data);
  BDDB_LOG(DLOG_VDR_BDDB, data, len);

  DATA_LOG(DLOG_VDR, "$GPOLY,%lld,%15.9f,%15.9f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%d,%.3f,%d,%f,%f,%f,%f,%f,%d,%f,%f\n",
    pz_AsensingGNSS->t_timestamp, pz_AsensingGNSS->d_lat, pz_AsensingGNSS->d_lon, pz_AsensingGNSS->f_alt,
    pz_AsensingGNSS->f_vn, pz_AsensingGNSS->f_ve, pz_AsensingGNSS->f_vd,
    pz_AsensingGNSS->e_posflag, pz_AsensingGNSS->f_latstd, pz_AsensingGNSS->f_lonstd, pz_AsensingGNSS->f_altstd,
    pz_AsensingGNSS->f_vnstd, pz_AsensingGNSS->f_vestd, pz_AsensingGNSS->f_vdstd,
    pz_AsensingGNSS->w_week, pz_AsensingGNSS->d_tow/1000.0,
    pz_AsensingGNSS->u_satused, pz_AsensingGNSS->f_avgsnr, pz_AsensingGNSS->f_pitch_deg,
    pz_AsensingGNSS->f_heading_deg, pz_AsensingGNSS->f_pitch_std, pz_AsensingGNSS->f_heading_std, 0,
    pz_AsensingGNSS->f_speed, pz_AsensingGNSS->f_heading);
#endif
}

void vdr_save_cp_meas_log(AsensingGNSSCpMeas_t* pz_gnss_cp_meas)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }

  DATA_LOG(DLOG_VDR, "$GPGCP,%lld\n",
    (uint64_t)(pz_gnss_cp_meas->d_cur_tow*1000.0));
#endif
}

void vdr_save_pr_meas_log(AsensingGNSSPsrMeas_t* pz_gnss_psr_meas)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }

  DATA_LOG(DLOG_VDR, "$GPPSR,%lld\n",
    (uint64_t)(pz_gnss_psr_meas->d_cur_tow*1000.0));
#endif
}

void vdr_save_ins_log(INSResults_t* pz_InsPositionFix)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
#define KWS_INIT 0.00863   
  uint8_t data[LOG_BDDB_MAX_LENGTH] = { 0 };

  float f_misb2v_roll_std = 0.0;
  float f_misb2v_pitch_std = 0.0;
  float f_misb2v_heading_std = 0.0;
  if (pz_InsPositionFix->e_drposflag >= 6)
  {  
    SysErrModel_t* pz_model = fusion_get_sysmodel();
    f_misb2v_roll_std = (float)(sqrtf(pz_model->f_syscovmat[15*pz_model->u_sysdim + 15]) * RAD2DEG);
    f_misb2v_pitch_std = (float)(sqrtf(pz_model->f_syscovmat[(15 + 1) * pz_model->u_sysdim + 15 + 1]) * RAD2DEG);
    f_misb2v_heading_std = (float)(sqrtf(pz_model->f_syscovmat[(15 + 2) * pz_model->u_sysdim + 15 + 2]) * RAD2DEG);
  }

  uint32_t len = data_encode_BDDB0B(pz_InsPositionFix, data);
  BDDB_LOG(DLOG_VDR_BDDB, data, len);
  DATA_LOG(DLOG_VDR, "$ASINR,%d,%d,%15.9f,%15.9f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n", 
    pz_InsPositionFix->q_tow,
    (int)pz_InsPositionFix->e_drposflag,
    pz_InsPositionFix->d_latitude * RAD2DEG, pz_InsPositionFix->d_longitude * RAD2DEG, pz_InsPositionFix->f_altitude,
    pz_InsPositionFix->f_vn, pz_InsPositionFix->f_ve, pz_InsPositionFix->f_vd,
    pz_InsPositionFix->f_heading, pz_InsPositionFix->f_pitch, pz_InsPositionFix->f_roll,
    pz_InsPositionFix->f_gyrobias_x, pz_InsPositionFix->f_gyrobias_y, pz_InsPositionFix->f_gyrobias_z,
    pz_InsPositionFix->f_accbias_x, pz_InsPositionFix->f_accbias_y, pz_InsPositionFix->f_accbias_z, pz_InsPositionFix->u_zuptflag,
    pz_InsPositionFix->f_mis_roll, pz_InsPositionFix->f_mis_pitch, pz_InsPositionFix->f_mis_yaw,
    KWS_INIT * (1.0 - pz_InsPositionFix->f_whlspd_sf[2]), KWS_INIT * (1.0 - pz_InsPositionFix->f_whlspd_sf[3]),
    pz_InsPositionFix->f_lat_std, pz_InsPositionFix->f_lon_std, pz_InsPositionFix->f_alt_std,
    f_misb2v_roll_std, f_misb2v_pitch_std, f_misb2v_heading_std,
    pz_InsPositionFix->q_kfmeastype);
#endif
}

void vdr_save_inspl_log(INSIntegrity_t* pz_integrity)
{
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
#define KWS_INIT 0.00863   
  uint8_t data[LOG_BDDB_MAX_LENGTH] = { 0 };
  uint32_t len = data_encode_BDDB0D(pz_integrity, data);
  BDDB_LOG(DLOG_VDR_BDDB, data, len);
  DATA_LOG(DLOG_VDR, "$ASITG,%llu,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
    pz_integrity->t_timestamp,
    (int)pz_integrity->e_drposflag,
    pz_integrity->f_pos_longitudinal_pl, pz_integrity->f_pos_lateral_pl,
    pz_integrity->f_pos_hpl, pz_integrity->f_pos_vpl,
    pz_integrity->f_pos_north_pl, pz_integrity->f_pos_east_pl,
    pz_integrity->f_vel_longitudinal_pl, pz_integrity->f_vel_lateral_pl,
    pz_integrity->f_vel_hpl, pz_integrity->f_vel_vpl,
    pz_integrity->f_vel_north_pl, pz_integrity->f_vel_east_pl,
    pz_integrity->f_roll_pl, pz_integrity->f_pitch_pl, pz_integrity->f_yaw_pl);
#endif
}
