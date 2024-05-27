/**@file        vdr_task.cpp
 * @brief       Differential Carrier Phase 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "gnss_type.h"
#include "gnss_common.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "vdr_task.h"
#include "vdr_api.h"
#include "sm_api.h"
#include "fusion_api.h"
#include "fusion_err_model.h"
#include "fusion_feedback.h"
#include "fusion_integrity.h"
#include "fusion_log.h"
#include "mw_alloc.h"

#define TIME_START 0
#define TIME_END   604800

FILE* fp_gpinr = NULL;
FILE* fp_algo = NULL;

static void vdr_cvt_ins_pl_2_position_fix_pl(const INSIntegrity_t* pz_inspl, gnss_Integrity_t* pz_pl)
{
  pz_pl->z_pos_longitudinal_pl = gnss_genProtectionLevelStruct(pz_inspl->f_pos_longitudinal_pl);
  pz_pl->z_pos_lateral_pl      = gnss_genProtectionLevelStruct(pz_inspl->f_pos_lateral_pl);
  pz_pl->z_pos_north_pl        = gnss_genProtectionLevelStruct(pz_inspl->f_pos_hpl);
  pz_pl->z_pos_east_pl         = gnss_genProtectionLevelStruct(pz_inspl->f_pos_vpl);
  pz_pl->z_pos_hor_pl          = gnss_genProtectionLevelStruct(pz_inspl->f_pos_north_pl);
  pz_pl->z_pos_ver_pl          = gnss_genProtectionLevelStruct(pz_inspl->f_pos_east_pl);
  pz_pl->z_vel_longitudinal_pl = gnss_genProtectionLevelStruct(pz_inspl->f_vel_longitudinal_pl);
  pz_pl->z_vel_lateral_pl      = gnss_genProtectionLevelStruct(pz_inspl->f_vel_lateral_pl);
  pz_pl->z_vel_north_pl        = gnss_genProtectionLevelStruct(pz_inspl->f_vel_hpl);
  pz_pl->z_vel_east_pl         = gnss_genProtectionLevelStruct(pz_inspl->f_vel_vpl);
  pz_pl->z_vel_hor_pl          = gnss_genProtectionLevelStruct(pz_inspl->f_vel_north_pl);
  pz_pl->z_vel_ver_pl          = gnss_genProtectionLevelStruct(pz_inspl->f_vel_east_pl);
  pz_pl->z_roll_pl             = gnss_genProtectionLevelStruct(pz_inspl->f_roll_pl);
  pz_pl->z_pitch_pl            = gnss_genProtectionLevelStruct(pz_inspl->f_pitch_pl);
  pz_pl->z_yaw_pl              = gnss_genProtectionLevelStruct(pz_inspl->f_yaw_pl);
}

void vdr_position_fix_report_callback(INSResults_t* pz_InsPositionFix)
{
  double tmp[3] = { 0 };
  double tmp2[3] = { 0 };

  if (NULL != pz_InsPositionFix)
  {
    gnss_PositionFix_t z_PositionFix = { 0 };
    z_PositionFix.u_fixSource = FIX_SOURCE_VDR;

    tm_cvt_SetGpst(&z_PositionFix.z_gpsTime, pz_InsPositionFix->w_week, pz_InsPositionFix->q_tow * 0.001);
    tm_cvt_GpstToEpoch(&z_PositionFix.z_gpsTime, &z_PositionFix.z_epoch);
    z_PositionFix.u_fixFlag = pz_InsPositionFix->e_drposflag;

    z_PositionFix.d_lla[0] = pz_InsPositionFix->d_latitude;
    z_PositionFix.d_lla[1] = pz_InsPositionFix->d_longitude;
    z_PositionFix.d_lla[2] = pz_InsPositionFix->f_altitude;
    gnss_Lla2Ecef(z_PositionFix.d_lla, z_PositionFix.d_xyz);
    z_PositionFix.f_velEnu[0] = pz_InsPositionFix->f_ve;
    z_PositionFix.f_velEnu[1] = pz_InsPositionFix->f_vn;
    z_PositionFix.f_velEnu[2] = -pz_InsPositionFix->f_vd;
    tmp[0] = z_PositionFix.f_velEnu[0];
    tmp[1] = z_PositionFix.f_velEnu[1];
    tmp[2] = z_PositionFix.f_velEnu[2];
    tmp2[0] = z_PositionFix.f_velXyz[0];
    tmp2[1] = z_PositionFix.f_velXyz[1];
    tmp2[2] = z_PositionFix.f_velXyz[2];

    gnss_Enu2Ecef(z_PositionFix.d_lla, tmp, tmp2);
    z_PositionFix.f_posLlaUnc[0] = pz_InsPositionFix->f_lat_std;
    z_PositionFix.f_posLlaUnc[1] = pz_InsPositionFix->f_lon_std;
    z_PositionFix.f_posLlaUnc[2] = pz_InsPositionFix->f_alt_std;
    tmp[0] = z_PositionFix.f_posXyzUnc[0];
    tmp[1] = z_PositionFix.f_posXyzUnc[1];
    tmp[2] = z_PositionFix.f_posXyzUnc[2];
    tmp2[0] = z_PositionFix.f_posLlaUnc[0];
    tmp2[1] = z_PositionFix.f_posLlaUnc[1];
    tmp2[2] = z_PositionFix.f_posLlaUnc[2];

    gnss_Lla2Ecef(tmp2, tmp);
    z_PositionFix.f_velEnuUnc[0] = pz_InsPositionFix->f_ve_std;
    z_PositionFix.f_velEnuUnc[1] = pz_InsPositionFix->f_vn_std;
    z_PositionFix.f_velEnuUnc[2] = pz_InsPositionFix->f_vd_std;
    z_PositionFix.z_ins.f_roll = pz_InsPositionFix->f_roll;
    z_PositionFix.z_ins.f_pitch = pz_InsPositionFix->f_pitch;
    z_PositionFix.z_ins.f_heading = pz_InsPositionFix->f_heading;
    z_PositionFix.z_ins.f_gyrobias_x = pz_InsPositionFix->f_gyrobias_x;
    z_PositionFix.z_ins.f_gyrobias_y = pz_InsPositionFix->f_gyrobias_y;
    z_PositionFix.z_ins.f_gyrobias_z = pz_InsPositionFix->f_gyrobias_z;
    z_PositionFix.z_ins.f_accbias_x = pz_InsPositionFix->f_accbias_x;
    z_PositionFix.z_ins.f_accbias_y = pz_InsPositionFix->f_accbias_y;
    z_PositionFix.z_ins.f_accbias_z = pz_InsPositionFix->f_accbias_z;
    z_PositionFix.z_ins.u_drposflag = pz_InsPositionFix->e_drposflag;
    z_PositionFix.z_ins.f_roll_std = pz_InsPositionFix->f_roll_std;
    z_PositionFix.z_ins.f_pitch_std = pz_InsPositionFix->f_pitch_std;
    z_PositionFix.z_ins.f_yaw_std = pz_InsPositionFix->f_yaw_std;
    z_PositionFix.z_ins.f_mis_roll = pz_InsPositionFix->f_mis_roll;
    z_PositionFix.z_ins.f_mis_pitch = pz_InsPositionFix->f_mis_pitch;
    z_PositionFix.z_ins.f_mis_yaw = pz_InsPositionFix->f_mis_yaw;
    z_PositionFix.z_ins.f_speed = pz_InsPositionFix->f_speed;
    z_PositionFix.z_ins.f_whlspd_sf[0] = pz_InsPositionFix->f_whlspd_sf[0];
    z_PositionFix.z_ins.f_whlspd_sf[1] = pz_InsPositionFix->f_whlspd_sf[1];
    z_PositionFix.z_ins.f_whlspd_sf[2] = pz_InsPositionFix->f_whlspd_sf[2];
    z_PositionFix.z_ins.f_whlspd_sf[3] = pz_InsPositionFix->f_whlspd_sf[3];
    z_PositionFix.z_ins.q_kfmeastype = pz_InsPositionFix->q_kfmeastype;
    /* INS protection level */
    vdr_cvt_ins_pl_2_position_fix_pl(&pz_InsPositionFix->z_inspl, &z_PositionFix.z_integrity);
    z_PositionFix.z_ins.f_la_imu2gnss[0] = pz_InsPositionFix->f_la_imu2gnss[0];
    z_PositionFix.z_ins.f_la_imu2gnss[1] = pz_InsPositionFix->f_la_imu2gnss[1];
    z_PositionFix.z_ins.f_la_imu2gnss[2] = pz_InsPositionFix->f_la_imu2gnss[2];
    z_PositionFix.z_ins.f_la_imu2rearmid[0] = pz_InsPositionFix->f_la_imu2rearmid[0];
    z_PositionFix.z_ins.f_la_imu2rearmid[1] = pz_InsPositionFix->f_la_imu2rearmid[1];
    z_PositionFix.z_ins.f_la_imu2rearmid[2] = pz_InsPositionFix->f_la_imu2rearmid[2];
    z_PositionFix.z_ins.f_la_rear2rear = pz_InsPositionFix->f_la_rear2rear;
    z_PositionFix.z_ins.f_misdualant_roll = pz_InsPositionFix->f_misdualant_roll;
    z_PositionFix.z_ins.f_misdualant_pitch = pz_InsPositionFix->f_misdualant_pitch;
    z_PositionFix.z_ins.f_misdualant_yaw = pz_InsPositionFix->f_misdualant_yaw;

    LOGD(TAG_VDR, "INSResult lat %15.9f lon %15.9f alt %f velEnu %f %f %f\n",
      z_PositionFix.d_lla[0], z_PositionFix.d_lla[1], z_PositionFix.d_lla[2], z_PositionFix.f_velEnu[0], z_PositionFix.f_velEnu[1], z_PositionFix.f_velEnu[2]);
    LOGD(TAG_VDR, "INSResult f_roll %f f_pitch %f f_heading %f f_gyrobias_x/y/z %f %f %f f_accbias_x/y/z %f %f %f roll_std %f pitch_std %f yaw_std %f mis_r %f mis_p %f mis_y %f speed %f flag %d\n",
      z_PositionFix.z_ins.f_roll, z_PositionFix.z_ins.f_pitch, z_PositionFix.z_ins.f_heading, z_PositionFix.z_ins.f_gyrobias_x, z_PositionFix.z_ins.f_gyrobias_y, z_PositionFix.z_ins.f_gyrobias_z, \
      z_PositionFix.z_ins.f_accbias_x, z_PositionFix.z_ins.f_accbias_y, z_PositionFix.z_ins.f_accbias_z, z_PositionFix.z_ins.f_roll_std, z_PositionFix.z_ins.f_pitch_std, z_PositionFix.z_ins.f_yaw_std, \
      z_PositionFix.z_ins.f_mis_roll, z_PositionFix.z_ins.f_mis_pitch, z_PositionFix.z_ins.f_mis_yaw, z_PositionFix.z_ins.f_speed, z_PositionFix.z_ins.u_drposflag);

    sm_api_PositionFix_Report(&z_PositionFix);

    // log_Package(LOG_PACKAGE_ID_INS_RESULT, (uint8_t*)pz_InsPositionFix, sizeof(INSResults_t));
  }

#if 1 
  vdr_save_ins_log(pz_InsPositionFix);

#ifdef OPEN_INS_INTEGRITY  
  if (pz_InsPositionFix != NULL)
  {
    pz_InsPositionFix->z_inspl.t_timestamp = pz_InsPositionFix->q_tow;
    vdr_save_inspl_log(&pz_InsPositionFix->z_inspl);
  }  
#endif

#ifdef MISANGLE_IMU2DUALANT_ERR_ESTI
  float* f_att_mis_b2g = mis_get_imu2dualant();
  float f_misb2g_pitch_std = sqrtf(pz_model->f_syscovmat[MISANGLE_IMU2DUALANT_ERR_ESTI][MISANGLE_IMU2DUALANT_ERR_ESTI]) * RAD2DEG;
  float f_misb2g_heading_std = sqrtf(pz_model->f_syscovmat[MISANGLE_IMU2DUALANT_ERR_ESTI + 1][MISANGLE_IMU2DUALANT_ERR_ESTI + 1]) * RAD2DEG;

  fprintf(fp_gpinr, "$ASDAT,%llu,%d,%f,%f,%f,%f,%f\n", pz_ins->t_timestamp, (int)pz_ins->e_drposflag,
    f_att_mis_b2g[0] * RAD2DEG, f_att_mis_b2g[1] * RAD2DEG, f_att_mis_b2g[2] * RAD2DEG,
    f_misb2g_pitch_std, f_misb2g_heading_std);
#endif

#ifdef LA_IMU2GNSS_ERR_ESTI   
  float* f_la_imu2gnss = lvrm_get_imu2gnss();
  float* f_la_imu2rearmid = lvrm_get_imu2rearmid();
  float* f_la_rear2rear = lvrm_get_rear2rear();

  float f_la_imu2gnss_std[3], f_la_imu2rear_std[3], f_la_rear2rear_std;
  for (uint8_t i = 0; i < 3; i++)
  {
    f_la_imu2gnss_std[i] = sqrtf(pz_model->f_syscovmat[LA_IMU2GNSS_ERR_ESTI + i][LA_IMU2GNSS_ERR_ESTI + i]);
    f_la_imu2rear_std[i] = sqrtf(pz_model->f_syscovmat[LA_IMU2REARMID_ERR_ESTI + i][LA_IMU2REARMID_ERR_ESTI + i]);
  }
  f_la_rear2rear_std = sqrtf(pz_model->f_syscovmat[WHEELBASE_ERR_ESTI][WHEELBASE_ERR_ESTI]);

  fprintf(fp_gpinr, "$ASLAC,%llu,%d,%f,%f,%f,%f,%f,%f,%f, %f,%f,%f,%f,%f,%f,%f\n", pz_ins->t_timestamp, (int)pz_ins->e_drposflag,
    f_la_imu2gnss[0], f_la_imu2gnss[1], f_la_imu2gnss[2], f_la_imu2rearmid[0], f_la_imu2rearmid[1], f_la_imu2rearmid[2], *f_la_rear2rear,
    f_la_imu2gnss_std[0], f_la_imu2gnss_std[1], f_la_imu2gnss_std[2], f_la_imu2rear_std[0], f_la_imu2rear_std[1], f_la_imu2rear_std[2], f_la_rear2rear_std);
#endif
#endif
}

static void fusion_debug_log(uint8_t* buf, uint32_t len)
{
  /* do nothing */
}

static void fusion_report_history(HistoryInsResults_t* pHistoryResults)
{

}

/**
 * @brief: VDR module initialize
 * @param[in]: p_ipc - pointer to IPC message
 * @return: None
 */
static void vdr_Init(ipc_t* p_ipc)
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParam = loc_cfg_getConfigParamGroup();
  if (NULL == pz_loc_ConfigParam)
  {
    return;
  }
  AsensingFusionInitPara_t z_init_para = { 0 };
    
  z_init_para.e_gnss_rate = pz_loc_ConfigParam->vdr_cfg.u_field[0];
  z_init_para.e_sampling_rate = pz_loc_ConfigParam->vdr_cfg.u_field[1];
  z_init_para.u_whspd_mode = pz_loc_ConfigParam->vdr_cfg.u_field[2];
  z_init_para.e_whspd_rate = pz_loc_ConfigParam->vdr_cfg.u_field[3];

  z_init_para.q_vdr_mode = pz_loc_ConfigParam->vdr_cfg.u_field[4];
  z_init_para.q_align_mode = pz_loc_ConfigParam->vdr_cfg.u_field[5];
  z_init_para.u_outputpos_flag = pz_loc_ConfigParam->vdr_cfg.u_field[6];
  z_init_para.e_output_rate = pz_loc_ConfigParam->vdr_cfg.u_field[7];
  z_init_para.u_dr_time = pz_loc_ConfigParam->vdr_cfg.u_field[8];
  z_init_para.u_imu_type = pz_loc_ConfigParam->vdr_cfg.u_field[9];

  AsensingFusionApiCallback_t z_callback = { 0 };
  z_callback.fusion_log = fusion_debug_log;
  z_callback.fusion_report = vdr_position_fix_report_callback;
  z_callback.fusion_report_history = fusion_report_history;
  asensing_fusion_init(&z_init_para, &z_callback);

  float f_gnss2imu[3] = { 0.0,0.0,0.0 };
  for (int i = 0; i < 3; i++) {
    f_gnss2imu[i] = pz_loc_ConfigParam->vdr_cfg.f_field[i];
  }
  asensing_lvrm_set_gnss2imu(f_gnss2imu); /* can read from 0B later */

  float f_rearmid2imu[3] = { 0.0,0.0,0.0 };
  for (int i = 0; i < 3; i++) {
    f_rearmid2imu[i] = pz_loc_ConfigParam->vdr_cfg.f_field[i+3];
  }
  asensing_lvrm_set_rearmid2imu(f_rearmid2imu); /* can read from 0B later */

  float f_rear2rear = pz_loc_ConfigParam->vdr_cfg.f_field[6];
#ifndef WHEELBASE_ERR_ESTI
  f_rear2rear = 1.684f;
#endif
  asensing_lvrm_set_tworears(f_rear2rear);

  float f_att_mis_b2g[3] = { 0.0 * DEG2RAD, 0.0 * DEG2RAD, 0.0 * DEG2RAD };
  for (int i = 0; i < 3; i++) {
    f_att_mis_b2g[i] = (float)(pz_loc_ConfigParam->vdr_cfg.f_field[i+7] * DEG2RAD);
  } 
  asensing_mis_set_imu2dualant(f_att_mis_b2g);
  ins_init();
}

/**
 * @brief: VDR module start
 * @param[in]: p_ipc - pointer to IPC message
 * @return: None
 */
static void vdr_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief: VDR module stop
 * @param[in]: p_ipc - pointer to IPC message
 * @return: None
 */
static void vdr_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief: VDR module release
 * @param[in]: p_ipc - pointer to IPC message
 * @return: None
 */
static void vdr_Release(ipc_t* p_ipc)
{
  ins_deInit();
}

/**
 * @brief: VDR API for IMU data put
 * @return: None
 */
static BOOL vdr_ImuData_Put(ipc_t* p_ipc)
{
  if (sizeof(ins_ImuData_t) != p_ipc->q_length)
  {
    return FALSE;
  }
  ins_ImuData_t* pz_ImuData = (ins_ImuData_t*)p_ipc->p_data;

  if (pz_ImuData->q_tow_msec / 1000.0 >= TIME_START && pz_ImuData->q_tow_msec / 1000.0 <= TIME_END)
  {
    AsensingIMU_t z_AsensingIMU = { 0 };
    z_AsensingIMU.t_timestamp = pz_ImuData->q_tow_msec;
    for (int i = 0; i < 3; i++)
    {
      z_AsensingIMU.f_gyro[i] = pz_ImuData->f_gyro[i];
      z_AsensingIMU.f_accl[i] = pz_ImuData->f_accl[i];
    }
    z_AsensingIMU.f_temp = pz_ImuData->f_temperature;

    vdr_save_imu_log(&z_AsensingIMU);

    asensing_fusion_inject_imu(&z_AsensingIMU);
  }
  return TRUE;
}

/**
 * @brief: VDR API for IMU data put
 * @return: None
 */
static BOOL vdr_WheelData_Put(ipc_t* p_ipc)
{
  if (sizeof(ins_WheelData_t) != p_ipc->q_length)
  {
    return FALSE;
  }
  ins_WheelData_t* pz_WheelData = (ins_WheelData_t*)p_ipc->p_data;

  if (pz_WheelData->t_timestamp / 1000.0 >= TIME_START && pz_WheelData->t_timestamp / 1000.0 <= TIME_END)
  {
    AsensingWhpulse_t z_AsensingWheel = { 0 };

    z_AsensingWheel.t_timestamp = pz_WheelData->t_timestamp;
    z_AsensingWheel.q_fl_whpulse = pz_WheelData->q_fl_whpulse;
    z_AsensingWheel.q_fr_whpulse = pz_WheelData->q_fr_whpulse;
    z_AsensingWheel.q_rl_whpulse = pz_WheelData->q_rl_whpulse;
    z_AsensingWheel.q_rr_whpulse = pz_WheelData->q_rr_whpulse;
    z_AsensingWheel.f_angle_front = pz_WheelData->f_angle_front;
    z_AsensingWheel.f_angle_rear = pz_WheelData->f_angle_rear;
    z_AsensingWheel.e_gear = pz_WheelData->e_gear;

    vdr_save_wheel_log(&z_AsensingWheel);

    asensing_fusion_inject_wheel(&z_AsensingWheel);
  }
  return TRUE;
}

/**
 * @brief: VDR API for GNSS fix put
 * @return: None
 */
static BOOL vdr_GnssFix_Put(ipc_t* p_ipc)
{
  if (sizeof(ins_GnssFix_t) != p_ipc->q_length)
  {
    return FALSE;
  }
  ins_GnssFix_t* pz_GnssFix = (ins_GnssFix_t*)p_ipc->p_data;

  if (pz_GnssFix->t_timestamp / 1000.0 >= TIME_START && pz_GnssFix->t_timestamp / 1000.0 <= TIME_END)
  {
    AsensingGNSSPara_t z_AsensingGNSS = { 0 };

    z_AsensingGNSS.t_timestamp = pz_GnssFix->t_timestamp;
    z_AsensingGNSS.t_timeStampOfUtc = pz_GnssFix->t_timeStampOfUtc;
    z_AsensingGNSS.d_lat = pz_GnssFix->d_lat;
    z_AsensingGNSS.d_lon = pz_GnssFix->d_lon;
    z_AsensingGNSS.f_alt = pz_GnssFix->f_alt;
    z_AsensingGNSS.f_speed = pz_GnssFix->f_speed;
    z_AsensingGNSS.f_heading = pz_GnssFix->f_heading;
    z_AsensingGNSS.f_pitch = pz_GnssFix->f_pitch;
    z_AsensingGNSS.f_hdop = pz_GnssFix->f_hdop;
    z_AsensingGNSS.f_hor_accu = pz_GnssFix->f_hor_accu;
    z_AsensingGNSS.f_vn = pz_GnssFix->f_vn;
    z_AsensingGNSS.f_ve = pz_GnssFix->f_ve;
    z_AsensingGNSS.f_vd = pz_GnssFix->f_vd;
    z_AsensingGNSS.f_latstd = pz_GnssFix->f_latstd;
    z_AsensingGNSS.f_lonstd = pz_GnssFix->f_lonstd;
    z_AsensingGNSS.f_altstd = pz_GnssFix->f_altstd;
    z_AsensingGNSS.f_vnstd = pz_GnssFix->f_vnstd;
    z_AsensingGNSS.f_vestd = pz_GnssFix->f_vestd;
    z_AsensingGNSS.f_vdstd = pz_GnssFix->f_vdstd;
    z_AsensingGNSS.f_lat_pl = pz_GnssFix->z_integrity.z_pos_north_pl.f_protection_level;
    z_AsensingGNSS.f_lon_pl = pz_GnssFix->z_integrity.z_pos_north_pl.f_protection_level;
    z_AsensingGNSS.f_alt_pl = pz_GnssFix->z_integrity.z_pos_ver_pl.f_protection_level;
    z_AsensingGNSS.u_lat_itg = (uint8_t)pz_GnssFix->z_integrity.z_pos_north_pl.u_flag;
    z_AsensingGNSS.u_lon_itg = (uint8_t)pz_GnssFix->z_integrity.z_pos_north_pl.u_flag;
    z_AsensingGNSS.u_alt_itg = (uint8_t)pz_GnssFix->z_integrity.z_pos_ver_pl.u_flag;
    z_AsensingGNSS.d_tow = pz_GnssFix->d_tow;
    z_AsensingGNSS.f_avgsnr = pz_GnssFix->f_avgsnr;
    z_AsensingGNSS.w_week = pz_GnssFix->w_week;
    z_AsensingGNSS.u_postype = 1; /* Force set as 1 */
    z_AsensingGNSS.u_satused = pz_GnssFix->u_satused;
    z_AsensingGNSS.e_posflag = pz_GnssFix->u_postype;
    z_AsensingGNSS.u_msg_type = 0;
    z_AsensingGNSS.f_heading_deg = 0;
    z_AsensingGNSS.f_heading_std = 0;
    z_AsensingGNSS.f_pitch_deg = 0;
    z_AsensingGNSS.f_pitch_std = 0;

    /* convert to */
    GpsTime_t gps_time = { 0 };
    EpochTime_t z_epochT = { 0 };
    tm_cvt_SetGpst(&gps_time, pz_GnssFix->w_week, pz_GnssFix->d_tow/1000.0);    
    tm_cvt_GpstToEpoch(&gps_time, &z_epochT);

    z_AsensingGNSS.UtcYear = z_epochT.year;
    z_AsensingGNSS.UtcMon = (uint8_t)z_epochT.month;
    z_AsensingGNSS.UtcDay = (uint8_t)z_epochT.day;
    z_AsensingGNSS.UtcHour = (uint8_t)z_epochT.hour;
    z_AsensingGNSS.UtcMin = (uint8_t)z_epochT.min;
    z_AsensingGNSS.UtcSec = (uint8_t)z_epochT.second;
    z_AsensingGNSS.UtcmSec = (uint16_t)((z_epochT.second - z_AsensingGNSS.UtcSec) * TIME_MSEC);

    vdr_save_gnss_log(&z_AsensingGNSS);

    asensing_fusion_inject_gnss_para(&z_AsensingGNSS);
  }
  return TRUE;
}

/**
 * @brief: VDR API for Gnss Meas data put
 * @return: None
 */
static BOOL vdr_GnssMeas_Put(ipc_t* p_ipc)
{
  int index = 0;
  if (sizeof(gnss_FeedbackInsMeasBlock_t) != p_ipc->q_length)
  {
    return FALSE;
  }
  gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackIns = (gnss_FeedbackInsMeasBlock_t*)p_ipc->p_data;

  if (pz_GnssFeedbackIns->z_GpsTime.q_towMsec / 1000.0 >= TIME_START && pz_GnssFeedbackIns->z_GpsTime.q_towMsec / 1000.0 <= TIME_END)
  {
    AsensingGNSSPsrMeas_t z_gnss_psr_meas;
    AsensingGNSSCpMeas_t z_gnss_cp_meas;

    memset(&z_gnss_psr_meas, 0, sizeof(AsensingGNSSPsrMeas_t));
    memset(&z_gnss_cp_meas, 0, sizeof(AsensingGNSSCpMeas_t));

    if (pz_GnssFeedbackIns->u_FeedbackMeasType == GNSS_FEEDBACK_PR_DOPPLER)
    {
      z_gnss_psr_meas.d_cur_tow = (double)(pz_GnssFeedbackIns->z_GpsTime.q_towMsec / 1000.0);
      z_gnss_psr_meas.e_posflag = pz_GnssFeedbackIns->u_CurFixFlag;
      z_gnss_psr_meas.u_count = pz_GnssFeedbackIns->u_MeasCount;
      memcpy(z_gnss_psr_meas.d_blh, pz_GnssFeedbackIns->d_CurLLA, sizeof(pz_GnssFeedbackIns->d_CurLLA));
      for (index = 0; index < z_gnss_psr_meas.u_count; index++) {
        z_gnss_psr_meas.z_raw_data[index].f_drnoise = (float)(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_dr_var);
        z_gnss_psr_meas.z_raw_data[index].f_prnoise = (float)(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_pr_var);
        z_gnss_psr_meas.z_raw_data[index].d_pr = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_pseudorange;
        z_gnss_psr_meas.z_raw_data[index].d_dr = (double)pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].f_doppler;
        z_gnss_psr_meas.z_raw_data[index].f_ele = (float)(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].f_elevation * RAD2DEG);

        memcpy(z_gnss_psr_meas.z_raw_data[index].d_sat_pos, pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_sat_pos, sizeof(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_sat_pos));
        memcpy(z_gnss_psr_meas.z_raw_data[index].d_sat_vel, pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_sat_vel, sizeof(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_sat_vel));
        z_gnss_psr_meas.z_raw_data[index].u_prn = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_svid;
        z_gnss_psr_meas.z_raw_data[index].u_sys = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_constellation;
        z_gnss_psr_meas.z_raw_data[index].u_snr = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_cn0;
        z_gnss_psr_meas.z_raw_data[index].u_valid = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_obs_valid;

      }
      asensing_fusion_inject_gnss_psr_meas(&z_gnss_psr_meas);
      vdr_save_pr_meas_log(&z_gnss_psr_meas);
    }
    else if (pz_GnssFeedbackIns->u_FeedbackMeasType == GNSS_FEEDBACK_TIME_DIFF_CP)
    {
      z_gnss_cp_meas.d_cur_tow = (double)(pz_GnssFeedbackIns->z_GpsTime.q_towMsec / 1000.0);
      z_gnss_cp_meas.e_posflag = pz_GnssFeedbackIns->u_CurFixFlag;
      z_gnss_cp_meas.u_count = pz_GnssFeedbackIns->u_MeasCount;
      z_gnss_cp_meas.e_diff_mask = (GnssTdCpMask_Enum)pz_GnssFeedbackIns->u_FeedbackMeasType;
      memcpy(z_gnss_cp_meas.d_blh, pz_GnssFeedbackIns->d_CurLLA, sizeof(pz_GnssFeedbackIns->d_CurLLA));
      for (index = 0; index < z_gnss_cp_meas.u_count; index++) {
        z_gnss_cp_meas.z_raw_cp_data[index].f_tdcp_obs = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].f_epoch_diff_obs;
        z_gnss_cp_meas.z_raw_cp_data[index].f_tdcp_res = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].f_epoch_diff_res;
        z_gnss_cp_meas.z_raw_cp_data[index].f_ele = (float)(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].f_elevation * RAD2DEG);

        memcpy(z_gnss_cp_meas.z_raw_cp_data[index].d_cur_sat_pos, pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_sat_pos, sizeof(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_sat_pos));
        memcpy(z_gnss_cp_meas.z_raw_cp_data[index].d_pre_sat_pos, pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_pre_sat_pos, sizeof(pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].d_pre_sat_pos));
        z_gnss_cp_meas.z_raw_cp_data[index].u_prn = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_svid;
        z_gnss_cp_meas.z_raw_cp_data[index].u_sys = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_constellation;
        z_gnss_cp_meas.z_raw_cp_data[index].u_snr = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_cn0;
        z_gnss_cp_meas.z_raw_cp_data[index].u_valid = pz_GnssFeedbackIns->z_GnssFeedbackInsMeasUnit[index].u_obs_valid;

      }
      asensing_fusion_inject_gnss_cp_meas(&z_gnss_cp_meas);
      vdr_save_cp_meas_log(&z_gnss_cp_meas);
    }
  }
  return TRUE;
}

/**
 * @brief: VDR API for EKF result put
 * @return: None
 */
static BOOL vdr_EkfResult_Put(ipc_t* p_ipc)
{
  ins_SysErrModData_t* p_SysErrModData = NULL;
  if (sizeof(ins_SysErrModData_t) != p_ipc->q_length)
  {
    return FALSE;
  }
  p_SysErrModData = (ins_SysErrModData_t*)p_ipc->p_data;
  kf_feedback_handle(p_SysErrModData->update_flag, &p_SysErrModData->sys_err_model);
  return TRUE;
}

/**
 * @brief: Sanity Check IPC
 * @param[in]: p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL vdr_IpcCheck(ipc_t* p_ipc)
{
  if (TASK_INDEX_VDR != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_VDR_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_VDR_IPC_END)))
  {
    return FALSE;
  }
  return TRUE;
}

/**
 * @brief: VDR IPC main proc function
 * @param[in]: p_ipc - pointer to IPC message
 * @return: None
 */
void vdr_Proc(ipc_t* p_ipc)
{
  if (FALSE == vdr_IpcCheck(p_ipc))
  {
    return;
  }
  switch (p_ipc->q_ipc_id)
  {
  case C_M_VDR_INIT:
    vdr_Init(p_ipc);
    break;
  case C_M_VDR_START:
    vdr_Start(p_ipc);
    break;
  case C_M_VDR_STOP:
    vdr_Stop(p_ipc);
    break;
  case C_M_VDR_RELEASE:
    vdr_Release(p_ipc);
    break;
  case C_M_VDR_IMU_DATA_PUT:
    vdr_ImuData_Put(p_ipc);
    break;
  case C_M_VDR_GNSS_FIX_PUT:
    vdr_GnssFix_Put(p_ipc);
    break;
  case C_M_VDR_WHEEL_DATA_PUT:
    vdr_WheelData_Put(p_ipc);
    break;
  case C_M_VDR_EKF_UPDATE_PUT:
    vdr_EkfResult_Put(p_ipc);
    break;
  case C_M_VDR_GNSS_MEAS_PUT:
    vdr_GnssMeas_Put(p_ipc);
    break;
  default:
    break;
  }
}

/**
 * @brief: VDR IPC process task
 * @param[in]: args - task configuration
 * @return: None
 */
void* vdr_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_VDR;

  ipctask_t* t = ipctask_GetInstance(taskIndex);
  t->e_status = TASK_STATUS_ENABLE;
  ipctask_SetThreadId2TaskId(get_thread_id(), taskIndex);

  if(NULL != t->float_hard_enable){
    t->float_hard_enable();
  }
  while (t->e_status == TASK_STATUS_ENABLE)
  {
    if (ipctask_ReceiveMessage(taskIndex, &z_ipc))
    {
      vdr_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }
  return NULL;
}

/**
 * @brief: VDR API for initilization
 * @return: None
 */
BOOL vdr_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief: VDR API for start
 * @return: None
 */
BOOL vdr_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_START, NULL, 0);
  return TRUE;
}

/**
 * @brief: VDR API for stop
 * @return: None
 */
BOOL vdr_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief: VDR API for release
 * @return: None
 */
BOOL vdr_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief: VDR API for IMU data put
 * @return: None
 */
BOOL vdr_api_ImuData_Put(ins_ImuData_t* pz_ImuData)
{
  pz_ImuData->u_version = VERSION_INS_IMU_DATA;
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_IMU_DATA_PUT, 
    (uint8_t*)pz_ImuData, sizeof(ins_ImuData_t));
  return TRUE;
}

/**
 * @brief: VDR API for GNSS fix put
 * @return: None
 */
BOOL vdr_api_GnssFix_Put(ins_GnssFix_t* pz_GnssFix)
{
  pz_GnssFix->u_version = VERSION_INS_GNSS_FIX;
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_GNSS_FIX_PUT,
    (uint8_t*)pz_GnssFix, sizeof(ins_GnssFix_t));
  return TRUE;
}

/**
 * @brief: VDR API for Wheel data put
 * @return: None
 */
BOOL vdr_api_WheelData_Put(ins_WheelData_t* pz_WheelData)
{
  pz_WheelData->u_version = VERSION_INS_WHEEL_DATA;
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_WHEEL_DATA_PUT,
    (uint8_t*)pz_WheelData, sizeof(ins_WheelData_t));
  return TRUE;
}

/**
 * @brief: VDR API for GNSS fix put
 * @return: None
 */
BOOL vdr_api_EkfResult_Put(ins_SysErrModData_t* pz_SysErrModData)
{
  pz_SysErrModData->u_version = VERSION_INS_EKF_RESULT;
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_EKF_UPDATE_PUT,
    (uint8_t*)pz_SysErrModData, sizeof(ins_SysErrModData_t));
  return TRUE;
}

/**
 * @brief: VDR API to GNSS feedback INS with measurement block
 * @param[in]: pz_GnssFeedbackInsMeasBlock - the measure blcok that GNSS feedback to INS
 * @return: None
 */
BOOL vdr_api_GnssInsMeasBlock_Put(gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock)
{
  pz_GnssFeedbackInsMeasBlock->u_version = VERSION_GNSS_FEEDBACK_INS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_VDR, C_M_VDR_GNSS_MEAS_PUT,
    (uint8_t*)pz_GnssFeedbackInsMeasBlock, sizeof(gnss_FeedbackInsMeasBlock_t));
  return TRUE;
}
