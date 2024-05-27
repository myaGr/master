/**@file        ipc_upgrade.c
 * @brief       ipc upgrade source file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/10/30  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "cmn_utils.h"
#include "mw_alloc.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "ipc_upgrade.h"
#include "loc_core_report.h"

void init_IntegrityStruct(gnss_Integrity_t_0* pz_itg)
{
  pz_itg->z_pos_longitudinal_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_lateral_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_north_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_east_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_hor_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_ver_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_longitudinal_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_lateral_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_north_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_east_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_hor_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_ver_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_roll_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pitch_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_yaw_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
}

BOOL ipc_upgrade_sm_Init(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_Start(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_Stop(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_Release(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_RcvMeasRTCM_Put(ipc_t* pz_src_ipc)
{
  pz_src_ipc->u_need_record = TRUE;
  return FALSE;
}

BOOL ipc_upgrade_sm_RefCorrRTCM_Put(ipc_t* pz_src_ipc)
{
  pz_src_ipc->u_need_record = TRUE;
  return FALSE;
}

static void convert_PositoinFix_From_0_To_1(const gnss_PositionFix_t_0* p_0, gnss_PositionFix_t_1* p_1)
{
  p_1->u_version = 1;
  p_1->w_size = sizeof(gnss_PositionFix_t_1);
  p_1->u_fixSource = p_0->u_fixSource;
  p_1->u_fixFlag = p_0->u_fixFlag;
  p_1->z_gpsTime = p_0->z_gpsTime;
  p_1->d_xyz[0] = p_0->d_xyz[0];
  p_1->d_xyz[1] = p_0->d_xyz[1];
  p_1->d_xyz[2] = p_0->d_xyz[2];
  p_1->d_lla[0] = p_0->d_lla[0];
  p_1->d_lla[1] = p_0->d_lla[1];
  p_1->d_lla[2] = p_0->d_lla[2];
  p_1->f_velXyz[0] = p_0->f_velXyz[0];
  p_1->f_velXyz[1] = p_0->f_velXyz[1];
  p_1->f_velXyz[2] = p_0->f_velXyz[2];
  p_1->f_velEnu[0] = p_0->f_velEnu[0];
  p_1->f_velEnu[1] = p_0->f_velEnu[1];
  p_1->f_velEnu[2] = p_0->f_velEnu[2];
  p_1->f_posXyzUnc[0] = p_0->f_posXyzUnc[0];
  p_1->f_posXyzUnc[1] = p_0->f_posXyzUnc[1];
  p_1->f_posXyzUnc[2] = p_0->f_posXyzUnc[2];
  p_1->f_posLlaUnc[0] = 0.0f;
  p_1->f_posLlaUnc[1] = 0.0f;
  p_1->f_posLlaUnc[2] = 0.0f;
  p_1->f_velEnuUnc[0] = p_0->f_velEnuUnc[0];
  p_1->f_velEnuUnc[1] = p_0->f_velEnuUnc[1];
  p_1->f_velEnuUnc[2] = p_0->f_velEnuUnc[2];
  p_1->d_clockBias = p_0->d_clockBias;
  p_1->d_clockDrift = p_0->d_clockDrift;

  p_1->z_dops.f_gdop = p_0->z_dops.f_gdop;
  p_1->z_dops.f_pdop = p_0->z_dops.f_pdop;
  p_1->z_dops.f_hdop = p_0->z_dops.f_hdop;
  p_1->z_dops.f_vdop = p_0->z_dops.f_vdop;
  p_1->z_dops.f_tdop = p_0->z_dops.f_tdop;

  p_1->z_SvStatus.u_SvValidCount = p_0->z_SvStatus.u_SvValidCount;
  p_1->z_SvStatus.u_SvInUseCount = p_0->z_SvStatus.u_SvInUseCount;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_1->z_SvStatus.t_SvValidMask[i] = p_0->z_SvStatus.t_SvValidMask[i];
    p_1->z_SvStatus.t_SvInUseMask[i] = p_0->z_SvStatus.t_SvInUseMask[i];
  }

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_0; i++)
  {
    p_1->z_SvStatus.z_SV[i] = p_0->z_SvStatus.z_SV[i];
  }

  p_1->z_rtk.f_age = p_0->z_rtk.f_age;
  p_1->z_rtk.q_StationID = p_0->z_rtk.q_StationID;
}

static void convert_PositoinFix_From_1_To_2(const gnss_PositionFix_t_1* p_1, gnss_PositionFix_t_2_0* p_2_0)
{
  p_2_0->u_version = 2;
  p_2_0->w_size = sizeof(gnss_PositionFix_t_2_0);
  p_2_0->u_fixSource = p_1->u_fixSource;
  p_2_0->u_fixFlag = p_1->u_fixFlag;
  p_2_0->z_gpsTime = p_1->z_gpsTime;
  p_2_0->d_xyz[0] = p_1->d_xyz[0];
  p_2_0->d_xyz[1] = p_1->d_xyz[1];
  p_2_0->d_xyz[2] = p_1->d_xyz[2];
  p_2_0->d_lla[0] = p_1->d_lla[0];
  p_2_0->d_lla[1] = p_1->d_lla[1];
  p_2_0->d_lla[2] = p_1->d_lla[2];
  p_2_0->f_velXyz[0] = p_1->f_velXyz[0];
  p_2_0->f_velXyz[1] = p_1->f_velXyz[1];
  p_2_0->f_velXyz[2] = p_1->f_velXyz[2];
  p_2_0->f_velEnu[0] = p_1->f_velEnu[0];
  p_2_0->f_velEnu[1] = p_1->f_velEnu[1];
  p_2_0->f_velEnu[2] = p_1->f_velEnu[2];
  p_2_0->f_posXyzUnc[0] = p_1->f_posXyzUnc[0];
  p_2_0->f_posXyzUnc[1] = p_1->f_posXyzUnc[1];
  p_2_0->f_posXyzUnc[2] = p_1->f_posXyzUnc[2];
  p_2_0->f_posLlaUnc[0] = p_1->f_posLlaUnc[0];
  p_2_0->f_posLlaUnc[1] = p_1->f_posLlaUnc[1];
  p_2_0->f_posLlaUnc[2] = p_1->f_posLlaUnc[2];
  p_2_0->f_velEnuUnc[0] = p_1->f_velEnuUnc[0];
  p_2_0->f_velEnuUnc[1] = p_1->f_velEnuUnc[1];
  p_2_0->f_velEnuUnc[2] = p_1->f_velEnuUnc[2];
  p_2_0->d_clockBias = p_1->d_clockBias;
  p_2_0->d_clockDrift = p_1->d_clockDrift;

  p_2_0->z_dops.f_gdop = p_1->z_dops.f_gdop;
  p_2_0->z_dops.f_pdop = p_1->z_dops.f_pdop;
  p_2_0->z_dops.f_hdop = p_1->z_dops.f_hdop;
  p_2_0->z_dops.f_vdop = p_1->z_dops.f_vdop;
  p_2_0->z_dops.f_tdop = p_1->z_dops.f_tdop;

  p_2_0->d_quasiGeoidHeight = p_1->d_quasiGeoidHeight;

  p_2_0->z_SvStatus.u_SvValidCount = p_1->z_SvStatus.u_SvValidCount;
  p_2_0->z_SvStatus.u_SvInUseCount = p_1->z_SvStatus.u_SvInUseCount;
  p_2_0->z_SvStatus.u_svTracked = p_1->z_SvStatus.u_svTracked;
  p_2_0->z_SvStatus.w_svFixed = p_1->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_2_0->z_SvStatus.t_SvValidMask[i] = p_1->z_SvStatus.t_SvValidMask[i];
    p_2_0->z_SvStatus.t_SvInUseMask[i] = p_1->z_SvStatus.t_SvInUseMask[i];
  }

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_0; i++)
  {
    p_2_0->z_SvStatus.z_SV[i] = p_1->z_SvStatus.z_SV[i];
  }

  p_2_0->z_rtk.f_age = p_1->z_rtk.f_age;
  p_2_0->z_rtk.q_StationID = p_1->z_rtk.q_StationID;
}

static void convert_PositoinFix_From_2_0_To_3(const gnss_PositionFix_t_2_0* p_2_0, gnss_PositionFix_t_3* p_3)
{
  p_3->u_version = 3;
  p_3->w_size = sizeof(gnss_PositionFix_t_3);
  p_3->u_fixSource = p_2_0->u_fixSource;
  p_3->u_fixFlag = p_2_0->u_fixFlag;
  p_3->z_gpsTime = p_2_0->z_gpsTime;
  p_3->d_xyz[0] = p_2_0->d_xyz[0];
  p_3->d_xyz[1] = p_2_0->d_xyz[1];
  p_3->d_xyz[2] = p_2_0->d_xyz[2];
  p_3->d_lla[0] = p_2_0->d_lla[0];
  p_3->d_lla[1] = p_2_0->d_lla[1];
  p_3->d_lla[2] = p_2_0->d_lla[2];
  p_3->f_velXyz[0] = p_2_0->f_velXyz[0];
  p_3->f_velXyz[1] = p_2_0->f_velXyz[1];
  p_3->f_velXyz[2] = p_2_0->f_velXyz[2];
  p_3->f_velEnu[0] = p_2_0->f_velEnu[0];
  p_3->f_velEnu[1] = p_2_0->f_velEnu[1];
  p_3->f_velEnu[2] = p_2_0->f_velEnu[2];
  p_3->f_posXyzUnc[0] = p_2_0->f_posXyzUnc[0];
  p_3->f_posXyzUnc[1] = p_2_0->f_posXyzUnc[1];
  p_3->f_posXyzUnc[2] = p_2_0->f_posXyzUnc[2];
  p_3->f_posLlaUnc[0] = p_2_0->f_posLlaUnc[0];
  p_3->f_posLlaUnc[1] = p_2_0->f_posLlaUnc[1];
  p_3->f_posLlaUnc[2] = p_2_0->f_posLlaUnc[2];
  p_3->f_velEnuUnc[0] = p_2_0->f_velEnuUnc[0];
  p_3->f_velEnuUnc[1] = p_2_0->f_velEnuUnc[1];
  p_3->f_velEnuUnc[2] = p_2_0->f_velEnuUnc[2];
  p_3->d_clockBias = p_2_0->d_clockBias;
  p_3->d_clockDrift = p_2_0->d_clockDrift;

  p_3->z_dops.f_gdop = p_2_0->z_dops.f_gdop;
  p_3->z_dops.f_pdop = p_2_0->z_dops.f_pdop;
  p_3->z_dops.f_hdop = p_2_0->z_dops.f_hdop;
  p_3->z_dops.f_vdop = p_2_0->z_dops.f_vdop;
  p_3->z_dops.f_tdop = p_2_0->z_dops.f_tdop;

  p_3->d_quasiGeoidHeight = p_2_0->d_quasiGeoidHeight;

  p_3->z_SvStatus.u_SvValidCount = p_2_0->z_SvStatus.u_SvValidCount;
  p_3->z_SvStatus.u_SvInUseCount = p_2_0->z_SvStatus.u_SvInUseCount;
  p_3->z_SvStatus.u_svTracked = p_2_0->z_SvStatus.u_svTracked;
  p_3->z_SvStatus.w_svFixed = p_2_0->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_3->z_SvStatus.t_SvValidMask[i] = p_2_0->z_SvStatus.t_SvValidMask[i];
    p_3->z_SvStatus.t_SvInUseMask[i] = p_2_0->z_SvStatus.t_SvInUseMask[i];
  }

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_0; i++)
  {
    p_3->z_SvStatus.z_SV[i] = p_2_0->z_SvStatus.z_SV[i];
  }

  p_3->z_rtk.f_age = p_2_0->z_rtk.f_age;
  p_3->z_rtk.q_StationID = p_2_0->z_rtk.q_StationID;
}

static void convert_PositoinFix_From_2_1_To_3(const gnss_PositionFix_t_2_1* p_2_1, gnss_PositionFix_t_3* p_3)
{
  p_3->u_version = 3;
  p_3->w_size = sizeof(gnss_PositionFix_t_3);
  p_3->u_fixSource = p_2_1->u_fixSource;
  p_3->u_fixFlag = p_2_1->u_fixFlag;
  p_3->z_gpsTime = p_2_1->z_gpsTime;
  p_3->d_xyz[0] = p_2_1->d_xyz[0];
  p_3->d_xyz[1] = p_2_1->d_xyz[1];
  p_3->d_xyz[2] = p_2_1->d_xyz[2];
  p_3->d_lla[0] = p_2_1->d_lla[0];
  p_3->d_lla[1] = p_2_1->d_lla[1];
  p_3->d_lla[2] = p_2_1->d_lla[2];
  p_3->f_velXyz[0] = p_2_1->f_velXyz[0];
  p_3->f_velXyz[1] = p_2_1->f_velXyz[1];
  p_3->f_velXyz[2] = p_2_1->f_velXyz[2];
  p_3->f_velEnu[0] = p_2_1->f_velEnu[0];
  p_3->f_velEnu[1] = p_2_1->f_velEnu[1];
  p_3->f_velEnu[2] = p_2_1->f_velEnu[2];
  p_3->f_posXyzUnc[0] = p_2_1->f_posXyzUnc[0];
  p_3->f_posXyzUnc[1] = p_2_1->f_posXyzUnc[1];
  p_3->f_posXyzUnc[2] = p_2_1->f_posXyzUnc[2];
  p_3->f_posLlaUnc[0] = p_2_1->f_posLlaUnc[0];
  p_3->f_posLlaUnc[1] = p_2_1->f_posLlaUnc[1];
  p_3->f_posLlaUnc[2] = p_2_1->f_posLlaUnc[2];
  p_3->f_velEnuUnc[0] = p_2_1->f_velEnuUnc[0];
  p_3->f_velEnuUnc[1] = p_2_1->f_velEnuUnc[1];
  p_3->f_velEnuUnc[2] = p_2_1->f_velEnuUnc[2];
  p_3->d_clockBias = p_2_1->d_clockBias;
  p_3->d_clockDrift = p_2_1->d_clockDrift;

  p_3->z_dops.f_gdop = p_2_1->z_dops.f_gdop;
  p_3->z_dops.f_pdop = p_2_1->z_dops.f_pdop;
  p_3->z_dops.f_hdop = p_2_1->z_dops.f_hdop;
  p_3->z_dops.f_vdop = p_2_1->z_dops.f_vdop;
  p_3->z_dops.f_tdop = p_2_1->z_dops.f_tdop;

  p_3->d_quasiGeoidHeight = p_2_1->d_quasiGeoidHeight;

  p_3->z_SvStatus.u_SvValidCount = p_2_1->z_SvStatus.u_SvValidCount;
  p_3->z_SvStatus.u_SvInUseCount = p_2_1->z_SvStatus.u_SvInUseCount;
  p_3->z_SvStatus.u_svTracked = p_2_1->z_SvStatus.u_svTracked;
  p_3->z_SvStatus.w_svFixed = p_2_1->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_3->z_SvStatus.t_SvValidMask[i] = p_2_1->z_SvStatus.t_SvValidMask[i];
    p_3->z_SvStatus.t_SvInUseMask[i] = p_2_1->z_SvStatus.t_SvInUseMask[i];
  }

  p_3->z_rtk.f_age = p_2_1->z_rtk.f_age;
  p_3->z_rtk.q_StationID = p_2_1->z_rtk.q_StationID;

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_1; i++)
  {
    p_3->z_SvStatus.z_SV[i] = p_2_1->z_SvStatus.z_SV[i];
  }
}

static void convert_PositoinFix_From_2_2_To_3(const gnss_PositionFix_t_2_2* p_2_2, gnss_PositionFix_t_3* p_3)
{
  p_3->u_version = 3;
  p_3->w_size = sizeof(gnss_PositionFix_t_3);
  p_3->u_fixSource = p_2_2->u_fixSource;
  p_3->u_fixFlag = p_2_2->u_fixFlag;
  p_3->z_gpsTime = p_2_2->z_gpsTime;
  p_3->d_xyz[0] = p_2_2->d_xyz[0];
  p_3->d_xyz[1] = p_2_2->d_xyz[1];
  p_3->d_xyz[2] = p_2_2->d_xyz[2];
  p_3->d_lla[0] = p_2_2->d_lla[0];
  p_3->d_lla[1] = p_2_2->d_lla[1];
  p_3->d_lla[2] = p_2_2->d_lla[2];
  p_3->f_velXyz[0] = p_2_2->f_velXyz[0];
  p_3->f_velXyz[1] = p_2_2->f_velXyz[1];
  p_3->f_velXyz[2] = p_2_2->f_velXyz[2];
  p_3->f_velEnu[0] = p_2_2->f_velEnu[0];
  p_3->f_velEnu[1] = p_2_2->f_velEnu[1];
  p_3->f_velEnu[2] = p_2_2->f_velEnu[2];
  p_3->f_posXyzUnc[0] = p_2_2->f_posXyzUnc[0];
  p_3->f_posXyzUnc[1] = p_2_2->f_posXyzUnc[1];
  p_3->f_posXyzUnc[2] = p_2_2->f_posXyzUnc[2];
  p_3->f_posLlaUnc[0] = p_2_2->f_posLlaUnc[0];
  p_3->f_posLlaUnc[1] = p_2_2->f_posLlaUnc[1];
  p_3->f_posLlaUnc[2] = p_2_2->f_posLlaUnc[2];
  p_3->f_velEnuUnc[0] = p_2_2->f_velEnuUnc[0];
  p_3->f_velEnuUnc[1] = p_2_2->f_velEnuUnc[1];
  p_3->f_velEnuUnc[2] = p_2_2->f_velEnuUnc[2];
  p_3->d_clockBias = p_2_2->d_clockBias;
  p_3->d_clockDrift = p_2_2->d_clockDrift;

  p_3->z_dops.f_gdop = p_2_2->z_dops.f_gdop;
  p_3->z_dops.f_pdop = p_2_2->z_dops.f_pdop;
  p_3->z_dops.f_hdop = p_2_2->z_dops.f_hdop;
  p_3->z_dops.f_vdop = p_2_2->z_dops.f_vdop;
  p_3->z_dops.f_tdop = p_2_2->z_dops.f_tdop;

  p_3->d_quasiGeoidHeight = p_2_2->d_quasiGeoidHeight;

  p_3->z_SvStatus.u_SvValidCount = p_2_2->z_SvStatus.u_SvValidCount;
  p_3->z_SvStatus.u_SvInUseCount = p_2_2->z_SvStatus.u_SvInUseCount;
  p_3->z_SvStatus.u_svTracked = p_2_2->z_SvStatus.u_svTracked;
  p_3->z_SvStatus.w_svFixed = p_2_2->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_3->z_SvStatus.t_SvValidMask[i] = p_2_2->z_SvStatus.t_SvValidMask[i];
    p_3->z_SvStatus.t_SvInUseMask[i] = p_2_2->z_SvStatus.t_SvInUseMask[i];
  }

  p_3->z_rtk.f_age = p_2_2->z_rtk.f_age;
  p_3->z_rtk.q_StationID = p_2_2->z_rtk.q_StationID;

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_0; i++)
  {
    p_3->z_SvStatus.z_SV[i] = p_2_2->z_SvStatus.z_SV[i];
  }
}

static void convert_PositoinFix_From_2_3_To_3(const gnss_PositionFix_t_2_3* p_2_3, gnss_PositionFix_t_3* p_3)
{
  p_3->u_version = 3;
  p_3->w_size = sizeof(gnss_PositionFix_t_3);
  p_3->u_fixSource = p_2_3->u_fixSource;
  p_3->u_fixFlag = p_2_3->u_fixFlag;
  p_3->z_gpsTime = p_2_3->z_gpsTime;
  p_3->d_xyz[0] = p_2_3->d_xyz[0];
  p_3->d_xyz[1] = p_2_3->d_xyz[1];
  p_3->d_xyz[2] = p_2_3->d_xyz[2];
  p_3->d_lla[0] = p_2_3->d_lla[0];
  p_3->d_lla[1] = p_2_3->d_lla[1];
  p_3->d_lla[2] = p_2_3->d_lla[2];
  p_3->f_velXyz[0] = p_2_3->f_velXyz[0];
  p_3->f_velXyz[1] = p_2_3->f_velXyz[1];
  p_3->f_velXyz[2] = p_2_3->f_velXyz[2];
  p_3->f_velEnu[0] = p_2_3->f_velEnu[0];
  p_3->f_velEnu[1] = p_2_3->f_velEnu[1];
  p_3->f_velEnu[2] = p_2_3->f_velEnu[2];
  p_3->f_posXyzUnc[0] = p_2_3->f_posXyzUnc[0];
  p_3->f_posXyzUnc[1] = p_2_3->f_posXyzUnc[1];
  p_3->f_posXyzUnc[2] = p_2_3->f_posXyzUnc[2];
  p_3->f_posLlaUnc[0] = p_2_3->f_posLlaUnc[0];
  p_3->f_posLlaUnc[1] = p_2_3->f_posLlaUnc[1];
  p_3->f_posLlaUnc[2] = p_2_3->f_posLlaUnc[2];
  p_3->f_velEnuUnc[0] = p_2_3->f_velEnuUnc[0];
  p_3->f_velEnuUnc[1] = p_2_3->f_velEnuUnc[1];
  p_3->f_velEnuUnc[2] = p_2_3->f_velEnuUnc[2];
  p_3->d_clockBias = p_2_3->d_clockBias;
  p_3->d_clockDrift = p_2_3->d_clockDrift;

  p_3->z_dops.f_gdop = p_2_3->z_dops.f_gdop;
  p_3->z_dops.f_pdop = p_2_3->z_dops.f_pdop;
  p_3->z_dops.f_hdop = p_2_3->z_dops.f_hdop;
  p_3->z_dops.f_vdop = p_2_3->z_dops.f_vdop;
  p_3->z_dops.f_tdop = p_2_3->z_dops.f_tdop;

  p_3->d_quasiGeoidHeight = p_2_3->d_quasiGeoidHeight;

  p_3->z_SvStatus.u_SvValidCount = p_2_3->z_SvStatus.u_SvValidCount;
  p_3->z_SvStatus.u_SvInUseCount = p_2_3->z_SvStatus.u_SvInUseCount;
  p_3->z_SvStatus.u_svTracked = p_2_3->z_SvStatus.u_svTracked;
  p_3->z_SvStatus.w_svFixed = p_2_3->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_3->z_SvStatus.t_SvValidMask[i] = p_2_3->z_SvStatus.t_SvValidMask[i];
    p_3->z_SvStatus.t_SvInUseMask[i] = p_2_3->z_SvStatus.t_SvInUseMask[i];
  }

  p_3->z_rtk.f_age = p_2_3->z_rtk.f_age;
  p_3->z_rtk.q_StationID = p_2_3->z_rtk.q_StationID;

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_1; i++)
  {
    p_3->z_SvStatus.z_SV[i] = p_2_3->z_SvStatus.z_SV[i];
  }
}

static void convert_PositoinFix_From_3_To_4(const gnss_PositionFix_t_3* p_3, gnss_PositionFix_t_4* p_4)
{
  p_4->u_version = 4;
  p_4->w_size = sizeof(gnss_PositionFix_t_4);
  p_4->u_fixSource = p_3->u_fixSource;
  p_4->u_fixFlag = p_3->u_fixFlag;
  p_4->z_gpsTime = p_3->z_gpsTime;
  p_4->d_xyz[0] = p_3->d_xyz[0];
  p_4->d_xyz[1] = p_3->d_xyz[1];
  p_4->d_xyz[2] = p_3->d_xyz[2];
  p_4->d_lla[0] = p_3->d_lla[0];
  p_4->d_lla[1] = p_3->d_lla[1];
  p_4->d_lla[2] = p_3->d_lla[2];
  p_4->f_velXyz[0] = p_3->f_velXyz[0];
  p_4->f_velXyz[1] = p_3->f_velXyz[1];
  p_4->f_velXyz[2] = p_3->f_velXyz[2];
  p_4->f_velEnu[0] = p_3->f_velEnu[0];
  p_4->f_velEnu[1] = p_3->f_velEnu[1];
  p_4->f_velEnu[2] = p_3->f_velEnu[2];
  p_4->f_posXyzUnc[0] = p_3->f_posXyzUnc[0];
  p_4->f_posXyzUnc[1] = p_3->f_posXyzUnc[1];
  p_4->f_posXyzUnc[2] = p_3->f_posXyzUnc[2];
  p_4->f_posLlaUnc[0] = p_3->f_posLlaUnc[0];
  p_4->f_posLlaUnc[1] = p_3->f_posLlaUnc[1];
  p_4->f_posLlaUnc[2] = p_3->f_posLlaUnc[2];
  p_4->f_velEnuUnc[0] = p_3->f_velEnuUnc[0];
  p_4->f_velEnuUnc[1] = p_3->f_velEnuUnc[1];
  p_4->f_velEnuUnc[2] = p_3->f_velEnuUnc[2];
  p_4->d_clockBias = p_3->d_clockBias;
  p_4->d_clockDrift = p_3->d_clockDrift;

  p_4->z_dops.f_gdop = p_3->z_dops.f_gdop;
  p_4->z_dops.f_pdop = p_3->z_dops.f_pdop;
  p_4->z_dops.f_hdop = p_3->z_dops.f_hdop;
  p_4->z_dops.f_vdop = p_3->z_dops.f_vdop;
  p_4->z_dops.f_tdop = p_3->z_dops.f_tdop;

  p_4->d_quasiGeoidHeight = p_3->d_quasiGeoidHeight;

  p_4->z_SvStatus.u_SvValidCount = p_3->z_SvStatus.u_SvValidCount;
  p_4->z_SvStatus.u_SvInUseCount = p_3->z_SvStatus.u_SvInUseCount;
  p_4->z_SvStatus.u_svTracked = p_3->z_SvStatus.u_svTracked;
  p_4->z_SvStatus.w_svFixed = p_3->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_4->z_SvStatus.t_SvValidMask[i] = p_3->z_SvStatus.t_SvValidMask[i];
    p_4->z_SvStatus.t_SvInUseMask[i] = p_3->z_SvStatus.t_SvInUseMask[i];
  }

  p_4->z_rtk.f_age = p_3->z_rtk.f_age;
  p_4->z_rtk.q_StationID = p_3->z_rtk.q_StationID;

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_1; i++)
  {
    p_4->z_SvStatus.z_SV[i] = p_3->z_SvStatus.z_SV[i];
  }
}


static void convert_PositoinFix_From_4_To_5(const gnss_PositionFix_t_4* p_4, gnss_PositionFix_t_5* p_5)
{
  p_5->u_version = 5;
  p_5->w_size = sizeof(gnss_PositionFix_t_5);
  p_5->u_fixSource = p_4->u_fixSource;
  p_5->u_fixFlag = p_4->u_fixFlag;
  p_5->z_gpsTime = p_4->z_gpsTime;
  p_5->d_xyz[0] = p_4->d_xyz[0];
  p_5->d_xyz[1] = p_4->d_xyz[1];
  p_5->d_xyz[2] = p_4->d_xyz[2];
  p_5->d_lla[0] = p_4->d_lla[0];
  p_5->d_lla[1] = p_4->d_lla[1];
  p_5->d_lla[2] = p_4->d_lla[2];
  p_5->f_velXyz[0] = p_4->f_velXyz[0];
  p_5->f_velXyz[1] = p_4->f_velXyz[1];
  p_5->f_velXyz[2] = p_4->f_velXyz[2];
  p_5->f_velEnu[0] = p_4->f_velEnu[0];
  p_5->f_velEnu[1] = p_4->f_velEnu[1];
  p_5->f_velEnu[2] = p_4->f_velEnu[2];
  p_5->f_posXyzUnc[0] = p_4->f_posXyzUnc[0];
  p_5->f_posXyzUnc[1] = p_4->f_posXyzUnc[1];
  p_5->f_posXyzUnc[2] = p_4->f_posXyzUnc[2];
  p_5->f_posLlaUnc[0] = p_4->f_posLlaUnc[0];
  p_5->f_posLlaUnc[1] = p_4->f_posLlaUnc[1];
  p_5->f_posLlaUnc[2] = p_4->f_posLlaUnc[2];
  p_5->f_velEnuUnc[0] = p_4->f_velEnuUnc[0];
  p_5->f_velEnuUnc[1] = p_4->f_velEnuUnc[1];
  p_5->f_velEnuUnc[2] = p_4->f_velEnuUnc[2];
  p_5->d_clockBias = p_4->d_clockBias;
  p_5->d_clockDrift = p_4->d_clockDrift;

  p_5->z_dops.f_gdop = p_4->z_dops.f_gdop;
  p_5->z_dops.f_pdop = p_4->z_dops.f_pdop;
  p_5->z_dops.f_hdop = p_4->z_dops.f_hdop;
  p_5->z_dops.f_vdop = p_4->z_dops.f_vdop;
  p_5->z_dops.f_tdop = p_4->z_dops.f_tdop;

  p_5->d_quasiGeoidHeight = p_4->d_quasiGeoidHeight;

  p_5->z_ins.u_drposflag    = p_4->z_ins.u_drposflag;
  p_5->z_ins.f_roll         = p_4->z_ins.f_roll;
  p_5->z_ins.f_pitch        = p_4->z_ins.f_pitch;
  p_5->z_ins.f_heading      = p_4->z_ins.f_heading;
  p_5->z_ins.f_gyrobias_x   = p_4->z_ins.f_gyrobias_x;
  p_5->z_ins.f_gyrobias_y   = p_4->z_ins.f_gyrobias_y;
  p_5->z_ins.f_gyrobias_z   = p_4->z_ins.f_gyrobias_z;
  p_5->z_ins.f_accbias_x    = p_4->z_ins.f_accbias_x;
  p_5->z_ins.f_accbias_y    = p_4->z_ins.f_accbias_y;
  p_5->z_ins.f_accbias_z    = p_4->z_ins.f_accbias_z;
  p_5->z_ins.f_roll_std     = p_4->z_ins.f_roll_std;
  p_5->z_ins.f_pitch_std    = p_4->z_ins.f_pitch_std;
  p_5->z_ins.f_yaw_std      = p_4->z_ins.f_yaw_std;
  p_5->z_ins.f_mis_roll     = p_4->z_ins.f_mis_roll;
  p_5->z_ins.f_mis_pitch    = p_4->z_ins.f_mis_pitch;
  p_5->z_ins.f_mis_yaw      = p_4->z_ins.f_mis_yaw;
  p_5->z_ins.f_speed        = p_4->z_ins.f_speed;
  p_5->z_ins.f_whlspd_sf[0] = 0.0f;
  p_5->z_ins.f_whlspd_sf[1] = 0.0f;
  p_5->z_ins.f_whlspd_sf[2] = 0.0f;
  p_5->z_ins.f_whlspd_sf[3] = 0.0f;
  p_5->z_ins.q_kfmeastype   = 0;

  p_5->z_SvStatus.u_SvValidCount = p_4->z_SvStatus.u_SvValidCount;
  p_5->z_SvStatus.u_SvInUseCount = p_4->z_SvStatus.u_SvInUseCount;
  p_5->z_SvStatus.u_svTracked = p_4->z_SvStatus.u_svTracked;
  p_5->z_SvStatus.w_svFixed = p_4->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_5->z_SvStatus.t_SvValidMask[i] = p_4->z_SvStatus.t_SvValidMask[i];
    p_5->z_SvStatus.t_SvInUseMask[i] = p_4->z_SvStatus.t_SvInUseMask[i];
  }

  p_5->z_rtk.f_age = p_4->z_rtk.f_age;
  p_5->z_rtk.q_StationID = p_4->z_rtk.q_StationID;

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_1; i++)
  {
    p_5->z_SvStatus.z_SV[i] = p_4->z_SvStatus.z_SV[i];
  }
}

static void convert_PositoinFix_From_5_To_6(const gnss_PositionFix_t_5* p_5, gnss_PositionFix_t_6* p_6)
{
  p_6->u_version = 6;
  p_6->w_size = sizeof(gnss_PositionFix_t_6);
  p_6->u_fixSource = p_5->u_fixSource;
  p_6->u_fixFlag = p_5->u_fixFlag;
  p_6->z_gpsTime = p_5->z_gpsTime;
  p_6->d_xyz[0] = p_5->d_xyz[0];
  p_6->d_xyz[1] = p_5->d_xyz[1];
  p_6->d_xyz[2] = p_5->d_xyz[2];
  p_6->d_lla[0] = p_5->d_lla[0];
  p_6->d_lla[1] = p_5->d_lla[1];
  p_6->d_lla[2] = p_5->d_lla[2];
  p_6->f_velXyz[0] = p_5->f_velXyz[0];
  p_6->f_velXyz[1] = p_5->f_velXyz[1];
  p_6->f_velXyz[2] = p_5->f_velXyz[2];
  p_6->f_velEnu[0] = p_5->f_velEnu[0];
  p_6->f_velEnu[1] = p_5->f_velEnu[1];
  p_6->f_velEnu[2] = p_5->f_velEnu[2];
  p_6->f_posXyzUnc[0] = p_5->f_posXyzUnc[0];
  p_6->f_posXyzUnc[1] = p_5->f_posXyzUnc[1];
  p_6->f_posXyzUnc[2] = p_5->f_posXyzUnc[2];
  p_6->f_posLlaUnc[0] = p_5->f_posLlaUnc[0];
  p_6->f_posLlaUnc[1] = p_5->f_posLlaUnc[1];
  p_6->f_posLlaUnc[2] = p_5->f_posLlaUnc[2];
  p_6->f_velEnuUnc[0] = p_5->f_velEnuUnc[0];
  p_6->f_velEnuUnc[1] = p_5->f_velEnuUnc[1];
  p_6->f_velEnuUnc[2] = p_5->f_velEnuUnc[2];
  p_6->d_clockBias = p_5->d_clockBias;
  p_6->d_clockDrift = p_5->d_clockDrift;

  p_6->z_dops.f_gdop = p_5->z_dops.f_gdop;
  p_6->z_dops.f_pdop = p_5->z_dops.f_pdop;
  p_6->z_dops.f_hdop = p_5->z_dops.f_hdop;
  p_6->z_dops.f_vdop = p_5->z_dops.f_vdop;
  p_6->z_dops.f_tdop = p_5->z_dops.f_tdop;

  p_6->d_quasiGeoidHeight = p_5->d_quasiGeoidHeight;

  p_6->z_ins.u_drposflag = p_5->z_ins.u_drposflag;
  p_6->z_ins.f_roll = p_5->z_ins.f_roll;
  p_6->z_ins.f_pitch = p_5->z_ins.f_pitch;
  p_6->z_ins.f_heading = p_5->z_ins.f_heading;
  p_6->z_ins.f_gyrobias_x = p_5->z_ins.f_gyrobias_x;
  p_6->z_ins.f_gyrobias_y = p_5->z_ins.f_gyrobias_y;
  p_6->z_ins.f_gyrobias_z = p_5->z_ins.f_gyrobias_z;
  p_6->z_ins.f_accbias_x = p_5->z_ins.f_accbias_x;
  p_6->z_ins.f_accbias_y = p_5->z_ins.f_accbias_y;
  p_6->z_ins.f_accbias_z = p_5->z_ins.f_accbias_z;
  p_6->z_ins.f_roll_std = p_5->z_ins.f_roll_std;
  p_6->z_ins.f_pitch_std = p_5->z_ins.f_pitch_std;
  p_6->z_ins.f_yaw_std = p_5->z_ins.f_yaw_std;
  p_6->z_ins.f_mis_roll = p_5->z_ins.f_mis_roll;
  p_6->z_ins.f_mis_pitch = p_5->z_ins.f_mis_pitch;
  p_6->z_ins.f_mis_yaw = p_5->z_ins.f_mis_yaw;
  p_6->z_ins.f_speed = p_5->z_ins.f_speed;
  p_6->z_ins.f_whlspd_sf[0] = p_5->z_ins.f_whlspd_sf[0];
  p_6->z_ins.f_whlspd_sf[1] = p_5->z_ins.f_whlspd_sf[1];
  p_6->z_ins.f_whlspd_sf[2] = p_5->z_ins.f_whlspd_sf[2];
  p_6->z_ins.f_whlspd_sf[3] = p_5->z_ins.f_whlspd_sf[3];
  p_6->z_ins.q_kfmeastype = p_5->z_ins.q_kfmeastype;
  p_6->z_ins.f_pos_longitudinal_pl = 0.0f;
  p_6->z_ins.f_pos_lateral_pl = 0.0f;
  p_6->z_ins.f_pos_hpl = 0.0f;
  p_6->z_ins.f_pos_vpl = 0.0f;
  p_6->z_ins.f_pos_north_pl = 0.0f;
  p_6->z_ins.f_pos_east_pl = 0.0f;
  p_6->z_ins.f_vel_longitudinal_pl = 0.0f;
  p_6->z_ins.f_vel_lateral_pl = 0.0f;
  p_6->z_ins.f_vel_hpl = 0.0f;
  p_6->z_ins.f_vel_vpl = 0.0f;
  p_6->z_ins.f_vel_north_pl = 0.0f;
  p_6->z_ins.f_vel_east_pl = 0.0f;
  p_6->z_ins.f_roll_pl = 0.0f;
  p_6->z_ins.f_pitch_pl = 0.0f;
  p_6->z_ins.f_yaw_pl = 0.0f;
  p_6->z_ins.f_la_imu2gnss[0] = 0.0f;
  p_6->z_ins.f_la_imu2gnss[1] = 0.0f;
  p_6->z_ins.f_la_imu2gnss[2] = 0.0f;
  p_6->z_ins.f_la_imu2rearmid[0] = 0.0f;
  p_6->z_ins.f_la_imu2rearmid[1] = 0.0f;
  p_6->z_ins.f_la_imu2rearmid[2] = 0.0f;
  p_6->z_ins.f_la_rear2rear = 0.0f;
  p_6->z_ins.f_misdualant_roll = 0.0f;
  p_6->z_ins.f_misdualant_pitch = 0.0f;
  p_6->z_ins.f_misdualant_yaw = 0.0f;

  p_6->z_SvStatus.u_SvValidCount = p_5->z_SvStatus.u_SvValidCount;
  p_6->z_SvStatus.u_SvInUseCount = p_5->z_SvStatus.u_SvInUseCount;
  p_6->z_SvStatus.u_svTracked = p_5->z_SvStatus.u_svTracked;
  p_6->z_SvStatus.w_svFixed = p_5->z_SvStatus.w_svFixed;
  for (int i = 0; i < C_GNSS1_MAX; i++)
  {
    p_6->z_SvStatus.t_SvValidMask[i] = p_5->z_SvStatus.t_SvValidMask[i];
    p_6->z_SvStatus.t_SvInUseMask[i] = p_5->z_SvStatus.t_SvInUseMask[i];
  }

  p_6->z_rtk.f_age = p_5->z_rtk.f_age;
  p_6->z_rtk.q_StationID = p_5->z_rtk.q_StationID;
  
  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_1; i++)
  {
    p_6->z_SvStatus.z_SV[i] = p_5->z_SvStatus.z_SV[i];
  }
}

static void convert_PositoinFix_From_6_To_7(const gnss_PositionFix_t_6* p_6, gnss_PositionFix_t_7* p_7)
{
  p_7->u_version = 7;
  p_7->w_size = sizeof(gnss_PositionFix_t_7);
  p_7->u_fixSource = p_6->u_fixSource;
  p_7->u_fixFlag = p_6->u_fixFlag;
  p_7->z_gpsTime = p_6->z_gpsTime;
  p_7->d_xyz[0] = p_6->d_xyz[0];
  p_7->d_xyz[1] = p_6->d_xyz[1];
  p_7->d_xyz[2] = p_6->d_xyz[2];
  p_7->d_lla[0] = p_6->d_lla[0];
  p_7->d_lla[1] = p_6->d_lla[1];
  p_7->d_lla[2] = p_6->d_lla[2];
  p_7->f_velXyz[0] = p_6->f_velXyz[0];
  p_7->f_velXyz[1] = p_6->f_velXyz[1];
  p_7->f_velXyz[2] = p_6->f_velXyz[2];
  p_7->f_velEnu[0] = p_6->f_velEnu[0];
  p_7->f_velEnu[1] = p_6->f_velEnu[1];
  p_7->f_velEnu[2] = p_6->f_velEnu[2];
  p_7->f_posXyzUnc[0] = p_6->f_posXyzUnc[0];
  p_7->f_posXyzUnc[1] = p_6->f_posXyzUnc[1];
  p_7->f_posXyzUnc[2] = p_6->f_posXyzUnc[2];
  p_7->f_posLlaUnc[0] = p_6->f_posLlaUnc[0];
  p_7->f_posLlaUnc[1] = p_6->f_posLlaUnc[1];
  p_7->f_posLlaUnc[2] = p_6->f_posLlaUnc[2];
  p_7->f_velEnuUnc[0] = p_6->f_velEnuUnc[0];
  p_7->f_velEnuUnc[1] = p_6->f_velEnuUnc[1];
  p_7->f_velEnuUnc[2] = p_6->f_velEnuUnc[2];
  p_7->d_clockBias = p_6->d_clockBias;
  p_7->d_clockDrift = p_6->d_clockDrift;

  p_7->z_dops.f_gdop = p_6->z_dops.f_gdop;
  p_7->z_dops.f_pdop = p_6->z_dops.f_pdop;
  p_7->z_dops.f_hdop = p_6->z_dops.f_hdop;
  p_7->z_dops.f_vdop = p_6->z_dops.f_vdop;
  p_7->z_dops.f_tdop = p_6->z_dops.f_tdop;

  p_7->d_quasiGeoidHeight = p_6->d_quasiGeoidHeight;

  p_7->z_ins.u_drposflag = p_6->z_ins.u_drposflag;
  p_7->z_ins.f_roll = p_6->z_ins.f_roll;
  p_7->z_ins.f_pitch = p_6->z_ins.f_pitch;
  p_7->z_ins.f_heading = p_6->z_ins.f_heading;
  p_7->z_ins.f_gyrobias_x = p_6->z_ins.f_gyrobias_x;
  p_7->z_ins.f_gyrobias_y = p_6->z_ins.f_gyrobias_y;
  p_7->z_ins.f_gyrobias_z = p_6->z_ins.f_gyrobias_z;
  p_7->z_ins.f_accbias_x = p_6->z_ins.f_accbias_x;
  p_7->z_ins.f_accbias_y = p_6->z_ins.f_accbias_y;
  p_7->z_ins.f_accbias_z = p_6->z_ins.f_accbias_z;
  p_7->z_ins.f_roll_std = p_6->z_ins.f_roll_std;
  p_7->z_ins.f_pitch_std = p_6->z_ins.f_pitch_std;
  p_7->z_ins.f_yaw_std = p_6->z_ins.f_yaw_std;
  p_7->z_ins.f_mis_roll = p_6->z_ins.f_mis_roll;
  p_7->z_ins.f_mis_pitch = p_6->z_ins.f_mis_pitch;
  p_7->z_ins.f_mis_yaw = p_6->z_ins.f_mis_yaw;
  p_7->z_ins.f_speed = p_6->z_ins.f_speed;
  p_7->z_ins.f_whlspd_sf[0] = p_6->z_ins.f_whlspd_sf[0];
  p_7->z_ins.f_whlspd_sf[1] = p_6->z_ins.f_whlspd_sf[1];
  p_7->z_ins.f_whlspd_sf[2] = p_6->z_ins.f_whlspd_sf[2];
  p_7->z_ins.f_whlspd_sf[3] = p_6->z_ins.f_whlspd_sf[3];
  p_7->z_ins.q_kfmeastype = p_6->z_ins.q_kfmeastype;
  p_7->z_ins.f_pos_longitudinal_pl = 0.0f;
  p_7->z_ins.f_pos_lateral_pl = 0.0f;
  p_7->z_ins.f_pos_hpl = 0.0f;
  p_7->z_ins.f_pos_vpl = 0.0f;
  p_7->z_ins.f_pos_north_pl = 0.0f;
  p_7->z_ins.f_pos_east_pl = 0.0f;
  p_7->z_ins.f_vel_longitudinal_pl = 0.0f;
  p_7->z_ins.f_vel_lateral_pl = 0.0f;
  p_7->z_ins.f_vel_hpl = 0.0f;
  p_7->z_ins.f_vel_vpl = 0.0f;
  p_7->z_ins.f_vel_north_pl = 0.0f;
  p_7->z_ins.f_vel_east_pl = 0.0f;
  p_7->z_ins.f_roll_pl = 0.0f;
  p_7->z_ins.f_pitch_pl = 0.0f;
  p_7->z_ins.f_yaw_pl = 0.0f;
  p_7->z_ins.f_la_imu2gnss[0] = 0.0f;
  p_7->z_ins.f_la_imu2gnss[1] = 0.0f;
  p_7->z_ins.f_la_imu2gnss[2] = 0.0f;
  p_7->z_ins.f_la_imu2rearmid[0] = 0.0f;
  p_7->z_ins.f_la_imu2rearmid[1] = 0.0f;
  p_7->z_ins.f_la_imu2rearmid[2] = 0.0f;
  p_7->z_ins.f_la_rear2rear = 0.0f;
  p_7->z_ins.f_misdualant_roll = 0.0f;
  p_7->z_ins.f_misdualant_pitch = 0.0f;
  p_7->z_ins.f_misdualant_yaw = 0.0f;

  p_7->z_SvStatus.u_SvValidCount = p_6->z_SvStatus.u_SvValidCount;
  p_7->z_SvStatus.u_SvInUseCount = p_6->z_SvStatus.u_SvInUseCount;
  p_7->z_SvStatus.u_svTracked = p_6->z_SvStatus.u_svTracked;
  p_7->z_SvStatus.w_svFixed = p_6->z_SvStatus.w_svFixed;

  p_7->z_SvStatus.t_SvValidMask[C_GNSS_GPS] = p_6->z_SvStatus.t_SvValidMask[C_GNSS1_GPS];
  p_7->z_SvStatus.t_SvInUseMask[C_GNSS_GPS] = p_6->z_SvStatus.t_SvInUseMask[C_GNSS1_GPS];
  p_7->z_SvStatus.t_SvValidMask[C_GNSS_GLO] = p_6->z_SvStatus.t_SvValidMask[C_GNSS1_GLO];
  p_7->z_SvStatus.t_SvInUseMask[C_GNSS_GLO] = p_6->z_SvStatus.t_SvInUseMask[C_GNSS1_GLO];
  p_7->z_SvStatus.t_SvValidMask[C_GNSS_BDS3] = p_6->z_SvStatus.t_SvValidMask[C_GNSS1_BDS];
  p_7->z_SvStatus.t_SvInUseMask[C_GNSS_BDS3] = p_6->z_SvStatus.t_SvInUseMask[C_GNSS1_BDS];
  p_7->z_SvStatus.t_SvValidMask[C_GNSS_GAL] = p_6->z_SvStatus.t_SvValidMask[C_GNSS1_GAL];
  p_7->z_SvStatus.t_SvInUseMask[C_GNSS_GAL] = p_6->z_SvStatus.t_SvInUseMask[C_GNSS1_GAL];
  p_7->z_SvStatus.t_SvValidMask[C_GNSS_QZS] = p_6->z_SvStatus.t_SvValidMask[C_GNSS1_QZS];
  p_7->z_SvStatus.t_SvInUseMask[C_GNSS_QZS] = p_6->z_SvStatus.t_SvInUseMask[C_GNSS1_QZS];

  p_7->z_rtk.f_age = p_6->z_rtk.f_age;
  p_7->z_rtk.q_StationID = p_6->z_rtk.q_StationID;

  for (int i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_1; i++)
  {
    p_7->z_SvStatus.z_SV[i] = p_6->z_SvStatus.z_SV[i];
  }
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_0_To_1(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_0* p_0 = (gnss_PositionFix_t_0*)pz_src_ipc->p_data;
  gnss_PositionFix_t_1* p_1 = (gnss_PositionFix_t_1*)OS_MALLOC(sizeof(gnss_PositionFix_t_1));
  memset(p_1, 0, sizeof(gnss_PositionFix_t_1));

  convert_PositoinFix_From_0_To_1(p_0, p_1);

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_1);
  pz_src_ipc->p_data = (uint8_t*)p_1;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_1_To_2(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_1* p_1 = (gnss_PositionFix_t_1*)pz_src_ipc->p_data;
  gnss_PositionFix_t_2_0* p_2 = (gnss_PositionFix_t_2_0*)OS_MALLOC(sizeof(gnss_PositionFix_t_2_0));
  memset(p_2, 0, sizeof(gnss_PositionFix_t_2_0));

  convert_PositoinFix_From_1_To_2(p_1, p_2);

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_2_0);
  pz_src_ipc->p_data = (uint8_t*)p_2;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_2_To_3(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_3* p_3 = (gnss_PositionFix_t_3*)OS_MALLOC(sizeof(gnss_PositionFix_t_3));
  memset(p_3, 0, sizeof(gnss_PositionFix_t_3));

  if (sizeof(gnss_PositionFix_t_2_0) == pz_src_ipc->q_length)
  {
    gnss_PositionFix_t_2_0* p_2_0 = (gnss_PositionFix_t_2_0*)pz_src_ipc->p_data;
    convert_PositoinFix_From_2_0_To_3(p_2_0, p_3);
  }
  else if (sizeof(gnss_PositionFix_t_2_1) == pz_src_ipc->q_length)
  {
    gnss_PositionFix_t_2_1* p_2_1 = (gnss_PositionFix_t_2_1*)pz_src_ipc->p_data;
    convert_PositoinFix_From_2_1_To_3(p_2_1, p_3);
  }
  else if (sizeof(gnss_PositionFix_t_2_2) == pz_src_ipc->q_length)
  {
    gnss_PositionFix_t_2_2* p_2_2 = (gnss_PositionFix_t_2_2*)pz_src_ipc->p_data;
    convert_PositoinFix_From_2_2_To_3(p_2_2, p_3);
  }
  else if (sizeof(gnss_PositionFix_t_2_3) == pz_src_ipc->q_length)
  {
    gnss_PositionFix_t_2_3* p_2_3 = (gnss_PositionFix_t_2_3*)pz_src_ipc->p_data;
    convert_PositoinFix_From_2_3_To_3(p_2_3, p_3);
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_3);
  pz_src_ipc->p_data = (uint8_t*)p_3;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_3_To_4(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_3* p_3 = (gnss_PositionFix_t_3*)pz_src_ipc->p_data;
  gnss_PositionFix_t_4* p_4 = (gnss_PositionFix_t_4*)OS_MALLOC(sizeof(gnss_PositionFix_t_4));
  memset(p_4, 0, sizeof(gnss_PositionFix_t_4));

  convert_PositoinFix_From_3_To_4(p_3, p_4);

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_4);
  pz_src_ipc->p_data = (uint8_t*)p_4;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_4_To_5(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_4* p_4 = (gnss_PositionFix_t_4*)pz_src_ipc->p_data;
  gnss_PositionFix_t_5* p_5 = (gnss_PositionFix_t_5*)OS_MALLOC(sizeof(gnss_PositionFix_t_5));
  memset(p_5, 0, sizeof(gnss_PositionFix_t_5));

  convert_PositoinFix_From_4_To_5(p_4, p_5);

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_5);
  pz_src_ipc->p_data = (uint8_t*)p_5;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_5_To_6(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_5* p_5 = (gnss_PositionFix_t_5*)pz_src_ipc->p_data;
  gnss_PositionFix_t_6* p_6 = (gnss_PositionFix_t_6*)OS_MALLOC(sizeof(gnss_PositionFix_t_6));
  memset(p_6, 0, sizeof(gnss_PositionFix_t_6));

  convert_PositoinFix_From_5_To_6(p_5, p_6);

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_6);
  pz_src_ipc->p_data = (uint8_t*)p_6;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_6_To_7(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_6* p_6 = (gnss_PositionFix_t_6*)pz_src_ipc->p_data;
  gnss_PositionFix_t_7* p_7 = (gnss_PositionFix_t_7*)OS_MALLOC(sizeof(gnss_PositionFix_t_7));
  memset(p_7, 0, sizeof(gnss_PositionFix_t_7));

  convert_PositoinFix_From_6_To_7(p_6, p_7);

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_7);
  pz_src_ipc->p_data = (uint8_t*)p_7;

  return TRUE;
}

static void convert_PositionFix_INS_From_2_To_3(const gnss_PositionFix_INS_t_2* pz_2, gnss_PositionFix_INS_t_3* pz_3)
{
  pz_3->u_drposflag         = pz_2->u_drposflag;
  pz_3->f_roll              = pz_2->f_roll;
  pz_3->f_pitch             = pz_2->f_pitch;
  pz_3->f_heading           = pz_2->f_heading;
  pz_3->f_gyrobias_x        = pz_2->f_gyrobias_x;
  pz_3->f_gyrobias_y        = pz_2->f_gyrobias_y;
  pz_3->f_gyrobias_z        = pz_2->f_gyrobias_z;
  pz_3->f_accbias_x         = pz_2->f_accbias_x;
  pz_3->f_accbias_y         = pz_2->f_accbias_y;
  pz_3->f_accbias_z         = pz_2->f_accbias_z;
  pz_3->f_roll_std          = pz_2->f_roll_std;
  pz_3->f_pitch_std         = pz_2->f_pitch_std;
  pz_3->f_yaw_std           = pz_2->f_yaw_std;
  pz_3->f_mis_roll          = pz_2->f_mis_roll;
  pz_3->f_mis_pitch         = pz_2->f_mis_pitch;
  pz_3->f_mis_yaw           = pz_2->f_mis_yaw;
  pz_3->f_speed             = pz_2->f_speed;
  memcpy(&pz_3->f_whlspd_sf, &pz_2->f_whlspd_sf, sizeof(float)*4);
  pz_3->q_kfmeastype        = pz_2->q_kfmeastype;
  memcpy(&pz_3->f_la_imu2gnss, &pz_2->f_la_imu2gnss, sizeof(float)*3);
  memcpy(&pz_3->f_la_imu2rearmid, &pz_2->f_la_imu2rearmid, sizeof(float)*3);
  pz_3->f_la_rear2rear      = pz_2->f_la_rear2rear;
  pz_3->f_misdualant_roll   = pz_2->f_misdualant_roll;
  pz_3->f_misdualant_pitch  = pz_2->f_misdualant_pitch;
  pz_3->f_misdualant_yaw    = pz_2->f_misdualant_yaw;
}

gnss_ProtectionLevel_t_0 ipc_genProtectionLevelStruct(float f_protection_level)
{
  gnss_ProtectionLevel_t_0 z_pl = {0};
  z_pl.f_protection_level = f_protection_level;
  z_pl.u_flag = GNSS_0_INTEGRITY_FLAG_NOT_MONITORED;
  return z_pl;
}
static void convert_ins_2_integrity(const gnss_PositionFix_INS_t_2* pz_src, gnss_Integrity_t_0* pz_dst)
{
  pz_dst->z_pos_longitudinal_pl = ipc_genProtectionLevelStruct(pz_src->f_pos_longitudinal_pl );
  pz_dst->z_pos_lateral_pl      = ipc_genProtectionLevelStruct(pz_src->f_pos_lateral_pl      );
  pz_dst->z_pos_north_pl        = ipc_genProtectionLevelStruct(pz_src->f_pos_hpl             );
  pz_dst->z_pos_east_pl         = ipc_genProtectionLevelStruct(pz_src->f_pos_vpl             );
  pz_dst->z_pos_hor_pl          = ipc_genProtectionLevelStruct(pz_src->f_pos_north_pl        );
  pz_dst->z_pos_ver_pl          = ipc_genProtectionLevelStruct(pz_src->f_pos_east_pl         );
  pz_dst->z_vel_longitudinal_pl = ipc_genProtectionLevelStruct(pz_src->f_vel_longitudinal_pl );
  pz_dst->z_vel_lateral_pl      = ipc_genProtectionLevelStruct(pz_src->f_vel_lateral_pl      );
  pz_dst->z_vel_north_pl        = ipc_genProtectionLevelStruct(pz_src->f_vel_hpl             );
  pz_dst->z_vel_east_pl         = ipc_genProtectionLevelStruct(pz_src->f_vel_vpl             );
  pz_dst->z_vel_hor_pl          = ipc_genProtectionLevelStruct(pz_src->f_vel_north_pl        );
  pz_dst->z_vel_ver_pl          = ipc_genProtectionLevelStruct(pz_src->f_vel_east_pl         );
  pz_dst->z_roll_pl             = ipc_genProtectionLevelStruct(pz_src->f_roll_pl             );
  pz_dst->z_pitch_pl            = ipc_genProtectionLevelStruct(pz_src->f_pitch_pl            );
  pz_dst->z_yaw_pl              = ipc_genProtectionLevelStruct(pz_src->f_yaw_pl              );
}

BOOL ipc_upgrade_sm_PostionFix_Report_From_7_To_8(ipc_t* pz_src_ipc)
{
  gnss_PositionFix_t_7* p_7 = (gnss_PositionFix_t_7*)pz_src_ipc->p_data;
  gnss_PositionFix_t_8* p_8 = (gnss_PositionFix_t_8*)OS_MALLOC(sizeof(gnss_PositionFix_t_8));
  memset(p_8, 0, sizeof(gnss_PositionFix_t_8));

  p_8->u_version = 8;
  p_8->w_size = sizeof(gnss_PositionFix_t_8);
  p_8->z_gpsTime          = p_7->z_gpsTime;
  p_8->z_epoch            = p_7->z_epoch;
  p_8->u_leapsec          = p_7->u_leapsec;
  p_8->u_DcpPosType       = p_7->u_DcpPosType;
  p_8->u_fixSource        = p_7->u_fixSource;
  p_8->u_fixFlag          = p_7->u_fixFlag;
  p_8->u_CN040            = p_7->u_CN040;
  p_8->d_avgCN0           = p_7->d_avgCN0;
  p_8->d_quasiGeoidHeight = p_7->d_quasiGeoidHeight;
  p_8->d_xyz[0]           = p_7->d_xyz[0];
  p_8->d_xyz[1]           = p_7->d_xyz[1];
  p_8->d_xyz[2]           = p_7->d_xyz[2];
  p_8->d_lla[0]           = p_7->d_lla[0];
  p_8->d_lla[1]           = p_7->d_lla[1];
  p_8->d_lla[2]           = p_7->d_lla[2];
  p_8->f_velXyz[0]        = p_7->f_velXyz[0];
  p_8->f_velXyz[1]        = p_7->f_velXyz[1];
  p_8->f_velXyz[2]        = p_7->f_velXyz[2];
  p_8->f_velEnu[0]        = p_7->f_velEnu[0];
  p_8->f_velEnu[1]        = p_7->f_velEnu[1];
  p_8->f_velEnu[2]        = p_7->f_velEnu[2];
  p_8->f_posXyzUnc[0]     = p_7->f_posXyzUnc[0];
  p_8->f_posXyzUnc[1]     = p_7->f_posXyzUnc[1];
  p_8->f_posXyzUnc[2]     = p_7->f_posXyzUnc[2];
  p_8->f_posLlaUnc[0]     = p_7->f_posLlaUnc[0];
  p_8->f_posLlaUnc[1]     = p_7->f_posLlaUnc[1];
  p_8->f_posLlaUnc[2]     = p_7->f_posLlaUnc[2];
  p_8->f_velEnuUnc[0]     = p_7->f_velEnuUnc[0];
  p_8->f_velEnuUnc[1]     = p_7->f_velEnuUnc[1];
  p_8->f_velEnuUnc[2]     = p_7->f_velEnuUnc[2];
  p_8->d_clockBias        = p_7->d_clockBias;
  p_8->d_clockDrift       = p_7->d_clockDrift;

  convert_ins_2_integrity(&p_7->z_ins, &p_8->z_integrity);
  p_8->z_dops             = p_7->z_dops;
  p_8->z_rtk              = p_7->z_rtk;
  convert_PositionFix_INS_From_2_To_3(&p_7->z_ins, &p_8->z_ins);
  p_8->z_SvStatus         = p_7->z_SvStatus;

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_8);
  pz_src_ipc->p_data = (uint8_t*)p_8;

  return TRUE;
}

BOOL ipc_upgrade_sm_PostionFix_Report(ipc_t* pz_src_ipc)
{
  uint8_t ret = FALSE;
  uint8_t u_version = pz_src_ipc->p_data[0];

  switch (u_version)
  {
  case 0:
    ret = ipc_upgrade_sm_PostionFix_Report_From_0_To_1(pz_src_ipc);
  case 1:
    ret = ipc_upgrade_sm_PostionFix_Report_From_1_To_2(pz_src_ipc);
  case 2:
    ret = ipc_upgrade_sm_PostionFix_Report_From_2_To_3(pz_src_ipc);
  case 3:
    ret = ipc_upgrade_sm_PostionFix_Report_From_3_To_4(pz_src_ipc);
  case 4:
    ret = ipc_upgrade_sm_PostionFix_Report_From_4_To_5(pz_src_ipc);
  case 5:
    ret = ipc_upgrade_sm_PostionFix_Report_From_5_To_6(pz_src_ipc);
  case 6:
    ret = ipc_upgrade_sm_PostionFix_Report_From_6_To_7(pz_src_ipc);
  case 7:
    ret = ipc_upgrade_sm_PostionFix_Report_From_7_To_8(pz_src_ipc);
  default:
    break;
  }

  return ret;
}

BOOL ipc_upgrade_sm_SsrLocBlock_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_RcvMeas_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_SsrQianXunStream_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_SsrGeeSpaceStream_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_sm_RcvTwinMeasRTCM_Put(ipc_t* pz_src_ipc)
{
  pz_src_ipc->u_need_record = TRUE;
  return FALSE;
}

BOOL ipc_upgrade_hpp_Init(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_Start(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_Stop(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_Release(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_RcvMeas_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_RefCorr_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_Ephemeris_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_RcvMeasRTCM_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_RefCorrRTCM_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_hpp_RcvMeasTwin_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_RcvMeasNHZ_PUT_From_0_To_1(ipc_t* pz_src_ipc)
{
  if (sizeof(GnssMeasBlock_t_1) == pz_src_ipc->q_length)
  {
    GnssMeasBlock_t_1* p1 = (GnssMeasBlock_t_1*)pz_src_ipc->p_data;
    p1->u_version = 1;
    return TRUE;
  }
  GnssMeasBlock_t_0* p0 = (GnssMeasBlock_t_0*) pz_src_ipc->p_data;
  GnssMeasBlock_t_1* p1 = (GnssMeasBlock_t_1*)OS_MALLOC(sizeof(GnssMeasBlock_t_1));

  p1->u_version = 1;
  p1->w_size = sizeof(GnssMeasBlock_t_1);
  p1->z_Clock = p0->z_Clock;
  p1->w_measCount = p0->w_measCount;
  for (uint8_t i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_0; i++)
  {
    p1->z_meas[i] = p0->z_meas[i];
  }
  for (uint8_t i = 0; i < MAX_GNSS_TRK_MEAS_NUMBER_0; i++)
  {
    p1->z_measExt[i] = p0->z_measExt[i];
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_PositionFix_t_1);
  pz_src_ipc->p_data = (uint8_t*)p1;

  return TRUE;
}


BOOL ipc_upgrade_dcp_RcvMeasNHZ_PUT(ipc_t* pz_src_ipc)
{
  uint8_t ret = FALSE;
  uint8_t u_version = pz_src_ipc->p_data[0];

  switch (u_version)
  {
  case 0:
    ret = ipc_upgrade_RcvMeasNHZ_PUT_From_0_To_1(pz_src_ipc);
  default:
    break;
  }

  return ret;
}

BOOL ipc_upgrade_ppp_Init(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_ppp_Start(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_ppp_Stop(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_ppp_Release(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_ppp_RcvMeas_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

static void convert_SatPosVelClk_From_0_To_1(const gnss_SatPosVelClk_t_0* pz_src, gnss_SatPosVelClk_t_1* pz_dst)
{
  pz_dst->z_gpsTime = pz_src->z_gpsTime;
  pz_dst->u_valid = pz_src->u_valid;
  pz_dst->u_constellation = pz_dst->u_constellation;
  pz_dst->u_svid = pz_dst->u_svid;;
  pz_dst->f_elevation = pz_dst->f_elevation;
  pz_dst->f_azimuth = pz_dst->f_azimuth;
  for (int i = 0; i < 4; i++)
  {
    pz_dst->d_satPosClk[i] = pz_src->d_satPosClk[i];
    pz_dst->d_satVelClk[i] = pz_src->d_satVelClk[i];
  }
  pz_dst->f_dt = 0.0f;
  pz_dst->q_iode = 0;
}

static void convert_TightSatelliteMeas_From_0_To_1(const gnss_TightSatelliteMeas_t_0* pz_src, gnss_TightSatelliteMeas_t_1* pz_dst)
{
  pz_dst->z_tot = pz_src->z_tot;
  pz_dst->u_constellation = pz_src->u_constellation;
  pz_dst->u_svid = pz_src->u_svid;
  convert_SatPosVelClk_From_0_To_1(&pz_src->z_satPosVelClk, &pz_dst->z_satPosVelClk);
  pz_dst->u_eclips = pz_src->u_eclips;
  for (int i = 0; i < MAX_GNSS_SIGNAL_FREQ; ++i)
  {
    pz_dst->f_gf[i] = pz_src->f_gf[i];
    pz_dst->f_mw[i] = pz_src->f_mw[i];
  }
  pz_dst->d_wetMap = pz_src->d_wetMap;
  pz_dst->d_dryMap = pz_src->d_dryMap;
  pz_dst->d_dryMap = pz_src->d_dryMap;
  pz_dst->d_Iono = pz_src->d_Iono;
  pz_dst->d_Tropo = pz_src->d_Tropo;
  for (int i = 0; i < MAX_GNSS_SIGNAL_FREQ; ++i)
  {
    pz_dst->pz_signalMeas[i] = pz_src->pz_signalMeas[i];
    pz_dst->pz_measQos[i] = pz_src->pz_measQos[i];
  }

  return;
}

BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_0_To_1(ipc_t* pz_src_ipc)
{
  gnss_ssrLosBlock_t_0* p_0 = (gnss_ssrLosBlock_t_0*)pz_src_ipc->p_data;
  gnss_ssrLosBlock_t_1* p_1 = (gnss_ssrLosBlock_t_1*)OS_MALLOC(sizeof(gnss_ssrLosBlock_t_1));
  memset(p_1, 0, sizeof(gnss_ssrLosBlock_t_1));

  p_1->u_version = 1;
  p_1->w_size = sizeof(gnss_ssrLosBlock_t_1);
  p_1->d_siteCoor[0] = p_0->d_siteCoor[0];
  p_1->d_siteCoor[1] = p_0->d_siteCoor[1];
  p_1->d_siteCoor[2] = p_0->d_siteCoor[2];

  p_1->z_epochLosInfo.z_tor             = p_0->z_epochLosInfo.z_tor;
  p_1->z_epochLosInfo.z_STECtime        = p_0->z_epochLosInfo.z_STECtime;
  p_1->z_epochLosInfo.z_STDtime         = p_0->z_epochLosInfo.z_STDtime;
  p_1->z_epochLosInfo.z_ZTDtime         = p_0->z_epochLosInfo.z_ZTDtime;
  p_1->z_epochLosInfo.u_ZTDmask         = p_0->z_epochLosInfo.u_ZTDmask;
  p_1->z_epochLosInfo.u_errorModelMask  = p_0->z_epochLosInfo.u_errorModelMask;
  p_1->z_epochLosInfo.z_ZTDcorr         = p_0->z_epochLosInfo.z_ZTDcorr;
  p_1->z_epochLosInfo.u_satCount        = p_0->z_epochLosInfo.u_satCount;
  
  for (uint8_t i = 0; i < M_ARRAY_SIZE(p_1->z_epochLosInfo.z_satLosCorr); i++)
  {
    p_1->z_epochLosInfo.z_satLosCorr[i].u_constellation    = p_0->z_epochLosInfo.z_satLosCorr[i].u_constellation;
    p_1->z_epochLosInfo.z_satLosCorr[i].u_svid             = p_0->z_epochLosInfo.z_satLosCorr[i].u_svid;
    p_1->z_epochLosInfo.z_satLosCorr[i].u_orbClkMask       = p_0->z_epochLosInfo.z_satLosCorr[i].u_orbClkMask;
    p_1->z_epochLosInfo.z_satLosCorr[i].u_atmoMask         = p_0->z_epochLosInfo.z_satLosCorr[i].u_atmoMask;
    p_1->z_epochLosInfo.z_satLosCorr[i].u_windUpMask       = p_0->z_epochLosInfo.z_satLosCorr[i].u_windUpMask;
    p_1->z_epochLosInfo.z_satLosCorr[i].u_signalNum        = p_0->z_epochLosInfo.z_satLosCorr[i].u_signalNum;
    p_1->z_epochLosInfo.z_satLosCorr[i].q_iode             = p_0->z_epochLosInfo.z_satLosCorr[i].q_iode;
    p_1->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc = p_0->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc;
    p_1->z_epochLosInfo.z_satLosCorr[i].z_orbClk           = p_0->z_epochLosInfo.z_satLosCorr[i].z_orbClk;
    p_1->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime       = p_0->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime;
    p_1->z_epochLosInfo.z_satLosCorr[i].z_stec             = p_0->z_epochLosInfo.z_satLosCorr[i].z_stec;
    p_1->z_epochLosInfo.z_satLosCorr[i].z_std              = p_0->z_epochLosInfo.z_satLosCorr[i].z_std;
    p_1->z_epochLosInfo.z_satLosCorr[i].d_windUp           = p_0->z_epochLosInfo.z_satLosCorr[i].d_windUp;
    p_1->z_epochLosInfo.z_satLosCorr[i].f_yaw              = p_0->z_epochLosInfo.z_satLosCorr[i].f_yaw;
    p_1->z_epochLosInfo.z_satLosCorr[i].f_yawRate          = p_0->z_epochLosInfo.z_satLosCorr[i].f_yawRate;

    // p_0, GNSS_SSR_SAT_CHL_NUM_LIMIT_0 = 10
    // p_1, GNSS_SSR_SAT_CHL_NUM_LIMIT_1 = 3
    uint8_t u_arr_size0 = M_ARRAY_SIZE(p_0->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr);
    uint8_t u_arr_size1 = M_ARRAY_SIZE(p_1->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr);
    for (uint8_t j = 0; j < M_MIN(u_arr_size0, u_arr_size1); j++)
    {
      p_1->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j] = p_0->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j];
    }
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
	pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_ssrLosBlock_t_1);
  pz_src_ipc->p_data = (uint8_t*)p_1;

  return TRUE;
}

BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_1_To_2(ipc_t* pz_src_ipc)
{
  gnss_ssrLosBlock_t_1* p_1 = (gnss_ssrLosBlock_t_1*)pz_src_ipc->p_data;
  gnss_ssrLosBlock_t_2* p_2 = (gnss_ssrLosBlock_t_2*)OS_MALLOC(sizeof(gnss_ssrLosBlock_t_2));
  memset(p_2, 0, sizeof(gnss_ssrLosBlock_t_2));

  p_2->u_version = 2;
  p_2->w_size = sizeof(gnss_ssrLosBlock_t_2);
  p_2->d_siteCoor[0] = p_1->d_siteCoor[0];
  p_2->d_siteCoor[1] = p_1->d_siteCoor[1];
  p_2->d_siteCoor[2] = p_1->d_siteCoor[2];

  p_2->z_epochLosInfo.z_tor = p_1->z_epochLosInfo.z_tor;
  p_2->z_epochLosInfo.z_STECtime = p_1->z_epochLosInfo.z_STECtime;
  p_2->z_epochLosInfo.z_STDtime = p_1->z_epochLosInfo.z_STDtime;
  p_2->z_epochLosInfo.z_ZTDtime = p_1->z_epochLosInfo.z_ZTDtime;
  p_2->z_epochLosInfo.u_ZTDmask = p_1->z_epochLosInfo.u_ZTDmask;
  p_2->z_epochLosInfo.u_errorModelMask = p_1->z_epochLosInfo.u_errorModelMask;
  p_2->z_epochLosInfo.z_ZTDcorr = p_1->z_epochLosInfo.z_ZTDcorr;
  p_2->z_epochLosInfo.u_satCount = p_1->z_epochLosInfo.u_satCount;

  for (uint8_t i = 0; i < M_ARRAY_SIZE(p_2->z_epochLosInfo.z_satLosCorr); i++) {
    p_2->z_epochLosInfo.z_satLosCorr[i].u_constellation = p_1->z_epochLosInfo.z_satLosCorr[i].u_constellation;
    p_2->z_epochLosInfo.z_satLosCorr[i].u_svid = p_1->z_epochLosInfo.z_satLosCorr[i].u_svid;
    p_2->z_epochLosInfo.z_satLosCorr[i].u_orbClkMask = p_1->z_epochLosInfo.z_satLosCorr[i].u_orbClkMask;
    p_2->z_epochLosInfo.z_satLosCorr[i].u_atmoMask = p_1->z_epochLosInfo.z_satLosCorr[i].u_atmoMask;
    p_2->z_epochLosInfo.z_satLosCorr[i].u_windUpMask = p_1->z_epochLosInfo.z_satLosCorr[i].u_windUpMask;
    p_2->z_epochLosInfo.z_satLosCorr[i].u_signalNum = p_1->z_epochLosInfo.z_satLosCorr[i].u_signalNum;
    p_2->z_epochLosInfo.z_satLosCorr[i].q_iode = p_1->z_epochLosInfo.z_satLosCorr[i].q_iode;
    convert_SatPosVelClk_From_0_To_1(&p_1->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc, &p_2->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc);
    p_2->z_epochLosInfo.z_satLosCorr[i].z_orbClk = p_1->z_epochLosInfo.z_satLosCorr[i].z_orbClk;
    p_2->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime = p_1->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime;
    p_2->z_epochLosInfo.z_satLosCorr[i].z_stec = p_1->z_epochLosInfo.z_satLosCorr[i].z_stec;
    p_2->z_epochLosInfo.z_satLosCorr[i].z_std = p_1->z_epochLosInfo.z_satLosCorr[i].z_std;
    p_2->z_epochLosInfo.z_satLosCorr[i].d_windUp = p_1->z_epochLosInfo.z_satLosCorr[i].d_windUp;
    p_2->z_epochLosInfo.z_satLosCorr[i].f_yaw = p_1->z_epochLosInfo.z_satLosCorr[i].f_yaw;
    p_2->z_epochLosInfo.z_satLosCorr[i].f_yawRate = p_1->z_epochLosInfo.z_satLosCorr[i].f_yawRate;

    uint8_t u_arr_size1 = M_ARRAY_SIZE(p_1->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr);
    uint8_t u_arr_size2 = M_ARRAY_SIZE(p_2->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr);
    for (uint8_t j = 0; j < M_MIN(u_arr_size1, u_arr_size2); j++)
    {
      p_2->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j] = p_1->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j];
    }
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_ssrLosBlock_t_2);
  pz_src_ipc->p_data = (uint8_t*)p_2;

  return TRUE;
}

BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_2_To_3(ipc_t* pz_src_ipc)
{
  uint8_t u_iSat = 0;
  gnss_Constellation1TypeEnumVal u_p2Constellation = C_GNSS1_MAX;
  gnss_ssrLosBlock_t_2* p_2 = (gnss_ssrLosBlock_t_2*)pz_src_ipc->p_data;
  gnss_ssrLosBlock_t_3* p_3 = (gnss_ssrLosBlock_t_3*)OS_MALLOC(sizeof(gnss_ssrLosBlock_t_3));
  memset(p_3, 0, sizeof(gnss_ssrLosBlock_t_3));
  memcpy(p_3, p_2, sizeof(gnss_ssrLosBlock_t_2));//the sise of gnss_ssrLosBlock_t_3 is equal to gnss_ssrLosBlock_t_2
  p_3->u_version = 3;
  p_3->w_size = sizeof(gnss_ssrLosBlock_t_3);
  for (u_iSat = 0; u_iSat < (p_3->z_epochLosInfo.u_satCount); ++u_iSat)
  {
    u_p2Constellation = p_2->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation;
    if (C_GNSS1_GPS == u_p2Constellation)
    {
      p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_GPS;
    }
    else if (C_GNSS1_GLO == u_p2Constellation)
    {
      p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_GLO;
    }
    else if (C_GNSS1_BDS == u_p2Constellation)
    {
      if (p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_svid >= 18)
      {
        p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_BDS3;
      }
      else
      {
        p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_BDS2;
      }
    }
    else if (C_GNSS1_GAL == u_p2Constellation)
    {
      p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_GAL;
    }
    else if (C_GNSS1_QZS == u_p2Constellation)
    {
      p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_QZS;
    }
    else
    {
      p_3->z_epochLosInfo.z_satLosCorr[u_iSat].u_constellation = C_GNSS_NONE;
    }
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_ssrLosBlock_t_3);
  pz_src_ipc->p_data = (uint8_t*)p_3;

  return TRUE;
}

BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_3_To_4(ipc_t* pz_src_ipc)
{
  gnss_ssrLosBlock_t_3* p_3 = (gnss_ssrLosBlock_t_3*)pz_src_ipc->p_data;
  gnss_ssrLosBlock_t_4* p_4 = (gnss_ssrLosBlock_t_4*)OS_MALLOC(sizeof(gnss_ssrLosBlock_t_4));
  memset(p_4, 0, sizeof(gnss_ssrLosBlock_t_3));

  p_4->u_version = 4;
  p_4->w_size = sizeof(gnss_ssrLosBlock_t_4);
  p_4->d_siteCoor[0] = p_3->d_siteCoor[0];
  p_4->d_siteCoor[1] = p_3->d_siteCoor[1];
  p_4->d_siteCoor[2] = p_3->d_siteCoor[2];

  p_4->z_epochLosInfo.z_tor = p_3->z_epochLosInfo.z_tor;
  p_4->z_epochLosInfo.z_ZTDtime = p_3->z_epochLosInfo.z_ZTDtime;
  p_4->z_epochLosInfo.u_ZTDmask = p_3->z_epochLosInfo.u_ZTDmask;
  p_4->z_epochLosInfo.u_errorModelMask = p_3->z_epochLosInfo.u_errorModelMask;
  p_4->z_epochLosInfo.z_ZTDcorr = p_3->z_epochLosInfo.z_ZTDcorr;
  p_4->z_epochLosInfo.u_satCount = p_3->z_epochLosInfo.u_satCount;

  for (uint8_t i = 0; i < M_ARRAY_SIZE(p_4->z_epochLosInfo.z_satLosCorr); i++) {
    p_4->z_epochLosInfo.z_satLosCorr[i].u_constellation = p_3->z_epochLosInfo.z_satLosCorr[i].u_constellation;
    p_4->z_epochLosInfo.z_satLosCorr[i].u_svid = p_3->z_epochLosInfo.z_satLosCorr[i].u_svid;
    p_4->z_epochLosInfo.z_satLosCorr[i].u_orbClkMask = p_3->z_epochLosInfo.z_satLosCorr[i].u_orbClkMask;
    p_4->z_epochLosInfo.z_satLosCorr[i].u_atmoMask = p_3->z_epochLosInfo.z_satLosCorr[i].u_atmoMask;
    p_4->z_epochLosInfo.z_satLosCorr[i].u_windUpMask = p_3->z_epochLosInfo.z_satLosCorr[i].u_windUpMask;
    p_4->z_epochLosInfo.z_satLosCorr[i].u_signalNum = p_3->z_epochLosInfo.z_satLosCorr[i].u_signalNum;
    p_4->z_epochLosInfo.z_satLosCorr[i].q_iode = p_3->z_epochLosInfo.z_satLosCorr[i].q_iode;
    memcpy(&p_4->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc, &p_3->z_epochLosInfo.z_satLosCorr[i].z_satPosVelClkBrdc, sizeof(gnss_SatPosVelClk_t_1));
    p_4->z_epochLosInfo.z_satLosCorr[i].u_clockContinuityIod = 0;
    memset(&p_4->z_epochLosInfo.z_satLosCorr[i].z_orbclkPrec, 0, sizeof(gnss_SsrCorrPrecision_t_0));
    p_4->z_epochLosInfo.z_satLosCorr[i].z_orbClk = p_3->z_epochLosInfo.z_satLosCorr[i].z_orbClk;
    p_4->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime = p_3->z_epochLosInfo.z_satLosCorr[i].z_orbClkTime;
    p_4->z_epochLosInfo.z_satLosCorr[i].z_STECtime = p_3->z_epochLosInfo.z_STECtime;
    p_4->z_epochLosInfo.z_satLosCorr[i].z_STDtime = p_3->z_epochLosInfo.z_STDtime;
    p_4->z_epochLosInfo.z_satLosCorr[i].z_stec = p_3->z_epochLosInfo.z_satLosCorr[i].z_stec;
    p_4->z_epochLosInfo.z_satLosCorr[i].z_std = p_3->z_epochLosInfo.z_satLosCorr[i].z_std;
    p_4->z_epochLosInfo.z_satLosCorr[i].d_windUp = p_3->z_epochLosInfo.z_satLosCorr[i].d_windUp;
    p_4->z_epochLosInfo.z_satLosCorr[i].f_yaw = p_3->z_epochLosInfo.z_satLosCorr[i].f_yaw;
    p_4->z_epochLosInfo.z_satLosCorr[i].f_yawRate = p_3->z_epochLosInfo.z_satLosCorr[i].f_yawRate;

    uint8_t u_arr_size3 = M_ARRAY_SIZE(p_3->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr);
    uint8_t u_arr_size4 = M_ARRAY_SIZE(p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr);
    for (uint8_t j = 0; j < M_MIN(u_arr_size3, u_arr_size4); j++)
    {
      p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_signalType = p_3->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_signalType;
      p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_biasMask = p_3->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_biasMask;
      p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].f_pcv = 0.0f;
      p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_discontinuityIod = p_3->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].u_phaseBiasContinueFlag;
      p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].z_codeBias = p_3->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].z_codeBias;
      p_4->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].z_phaseBias = p_3->z_epochLosInfo.z_satLosCorr[i].z_signalBiasCorr[j].z_phaseBias;
    }
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_ssrLosBlock_t_4);
  pz_src_ipc->p_data = (uint8_t*)p_4;

  return TRUE;
}

BOOL ipc_upgrade_TightSatSigMeas_Put_From_0_To_1(ipc_t* pz_src_ipc)
{
  gnss_TightSatSigMeasCollect_t_0* p_0 = (gnss_TightSatSigMeasCollect_t_0*)pz_src_ipc->p_data;
  gnss_TightSatSigMeasCollect_t_1* p_1 = (gnss_TightSatSigMeasCollect_t_1*)OS_MALLOC(sizeof(gnss_TightSatSigMeasCollect_t_1));
  memset(p_1, 0, sizeof(gnss_TightSatSigMeasCollect_t_1));

  p_1->u_version = 1;
  p_1->z_tor = p_0->z_tor;
  p_1->q_clk_jump = p_0->q_clk_jump;
  p_1->z_positionFix = p_0->z_positionFix;
  p_1->d_ZTD = p_0->d_ZTD;
  p_1->d_zhd_emp = p_0->d_zhd_emp;
  p_1->d_zwd_emp = p_0->d_zwd_emp;
  p_1->u_satMeasCount = p_0->u_satMeasCount;

  for (int i = 0; i < MAX_GNSS_ACTIVE_SAT_NUMBER; i++)
  {
    convert_TightSatelliteMeas_From_0_To_1(&p_0->pz_tightSatMeas[i], &p_1->pz_tightSatMeas[i]);
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
    pz_src_ipc->u_use_heap = FALSE;
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_TightSatSigMeasCollect_t_1);
  pz_src_ipc->p_data = (uint8_t*)p_1;

  return TRUE;
}

BOOL ipc_upgrade_TightSatSigMeas_Put_From_1_To_2(ipc_t* pz_src_ipc)
{
  gnss_TightSatSigMeasCollect_t_1* p_1 = (gnss_TightSatSigMeasCollect_t_1*)pz_src_ipc->p_data;
  gnss_TightSatSigMeasCollect_t_2* p_2 = (gnss_TightSatSigMeasCollect_t_2*)OS_MALLOC(sizeof(gnss_TightSatSigMeasCollect_t_2));
  memset(p_2, 0, sizeof(gnss_TightSatSigMeasCollect_t_2));

  p_2->u_version = 2;
  p_2->z_tor = p_1->z_tor;
  p_2->q_clk_jump = p_1->q_clk_jump;
  convert_PositoinFix_From_0_To_1(&p_1->z_positionFix, &p_2->z_positionFix);
  p_2->d_ZTD = p_1->d_ZTD;
  p_2->d_zhd_emp = p_1->d_zhd_emp;
  p_2->d_zwd_emp = p_1->d_zwd_emp;
  p_2->u_satMeasCount = p_1->u_satMeasCount;

  for (int i = 0; i < MAX_GNSS_ACTIVE_SAT_NUMBER; i++)
  {
    p_2->pz_tightSatMeas[i] = p_1->pz_tightSatMeas[i];
  }

  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(gnss_TightSatSigMeasCollect_t_2);
  pz_src_ipc->p_data = (uint8_t*)p_2;
  return TRUE;
}

BOOL ipc_upgrade_ppp_SsrLocBlock_Put(ipc_t* pz_src_ipc)
{
  uint8_t ret = FALSE;
  uint8_t u_version = pz_src_ipc->p_data[0];

  switch (u_version)
  {
  case 0:
    ret = ipc_upgrade_ppp_SsrLocBlock_Put_From_0_To_1(pz_src_ipc);
  case 1:
    ret = ipc_upgrade_ppp_SsrLocBlock_Put_From_1_To_2(pz_src_ipc);
  case 2:
    ret = ipc_upgrade_ppp_SsrLocBlock_Put_From_2_To_3(pz_src_ipc);
  case 3:
    ret = ipc_upgrade_ppp_SsrLocBlock_Put_From_3_To_4(pz_src_ipc);
  default:
    break;
  }

  return ret;
}

BOOL ipc_upgrade_ppp_TightSatSigMeasCollect_Put(ipc_t* pz_src_ipc)
{
  uint8_t ret = FALSE;
  uint8_t u_version = pz_src_ipc->p_data[0];

  switch (u_version)
  {
  case 0:
    ret = ipc_upgrade_TightSatSigMeas_Put_From_0_To_1(pz_src_ipc);
  case 1:
    ret = ipc_upgrade_TightSatSigMeas_Put_From_1_To_2(pz_src_ipc);
  default:
    break;
  }

  return ret;
}

BOOL ipc_upgrade_ppp_Algorithm_Start(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_ppp_SsrQianXunStream_Put(ipc_t* pz_src_ipc)
{
  return FALSE;
}

BOOL ipc_upgrade_vdr_GnssFixPut(ipc_t* pz_src_ipc)
{
  ins_GnssFix_t_0* p_0 = (ins_GnssFix_t_0*)pz_src_ipc->p_data;
  ins_GnssFix_t_1* p_1 = (ins_GnssFix_t_1*)OS_MALLOC(sizeof(ins_GnssFix_t_1));
  p_1->u_version        = 1;

  p_1->t_timestamp      = p_0->t_timestamp;
  p_1->t_timeStampOfUtc = p_0->t_timeStampOfUtc;
  p_1->d_lat            = p_0->d_lat;
  p_1->d_lon            = p_0->d_lon;
  p_1->f_alt            = p_0->f_alt;
  p_1->f_speed          = p_0->f_speed;
  p_1->f_heading        = p_0->f_heading;
  p_1->f_pitch          = p_0->f_pitch;
  p_1->f_hdop           = p_0->f_hdop;
  p_1->f_hor_accu       = p_0->f_hor_accu;
  p_1->f_vn             = p_0->f_vn;
  p_1->f_ve             = p_0->f_ve;
  p_1->f_vd             = p_0->f_vd;
  p_1->f_latstd         = p_0->f_latstd;
  p_1->f_lonstd         = p_0->f_lonstd;
  p_1->f_altstd         = p_0->f_altstd;
  p_1->f_vnstd          = p_0->f_vnstd;
  p_1->f_vestd          = p_0->f_vestd;
  p_1->f_vdstd          = p_0->f_vdstd;
  p_1->d_tow            = p_0->d_tow;
  p_1->f_avgsnr         = p_0->f_avgsnr;
  p_1->w_week           = p_0->w_week;
  p_1->u_postype        = p_0->u_postype;
  p_1->u_satused        = p_0->u_satused;
  init_IntegrityStruct(&p_1->z_integrity);
 
  if (pz_src_ipc->u_use_heap)
  {
    OS_FREE(pz_src_ipc->p_data);
  }

  pz_src_ipc->u_use_heap = TRUE;
  pz_src_ipc->q_length = sizeof(ins_GnssFix_t_1);
  pz_src_ipc->p_data = (uint8_t*)p_1;
  return TRUE;

  return FALSE;
}


/**
 * @brief Upgrade injected ipc to current latest ipc
 * @param[out]  pz_dst_ipc - pointer to current IPC version message
 * @param[in]   pz_src_ipc - pointer to legacy  IPC version message
 * @return      TRUE - be upgraded, the p_data is new alloc data
 *              FALSE - not be upgraded
 */
BOOL ipc_upgrade(ipc_t* pz_src_ipc)
{
  BOOL ret = FALSE;
  switch (pz_src_ipc->q_ipc_id)
  {
  case C_M_SM_INIT:
    ret = ipc_upgrade_sm_Init(pz_src_ipc);
    break;
  case C_M_SM_START:
    ret = ipc_upgrade_sm_Start(pz_src_ipc);
    break;
  case C_M_SM_STOP:
    ret = ipc_upgrade_sm_Stop(pz_src_ipc);
    break;
  case C_M_SM_RELEASE:
    ret = ipc_upgrade_sm_Release(pz_src_ipc);
    break;
  case C_M_SM_RCV_OBS_RTCM:
    ret = ipc_upgrade_sm_RcvMeasRTCM_Put(pz_src_ipc);
    break;
  case C_M_SM_REF_CORR_RTCM:
    ret = ipc_upgrade_sm_RefCorrRTCM_Put(pz_src_ipc);
    break;
  case C_M_SM_REPORT_POS_FIX:
    ret = ipc_upgrade_sm_PostionFix_Report(pz_src_ipc);
    break;
  case C_M_SM_SSR_LOS_BLK:
    ret = ipc_upgrade_sm_SsrLocBlock_Put(pz_src_ipc);
    break;
  case C_M_SM_RCV_MEAS_PUT:
    ret = ipc_upgrade_sm_RcvMeas_Put(pz_src_ipc);
    break;
  case C_M_SM_SSR_STREAM_QX:
    ret = ipc_upgrade_sm_SsrQianXunStream_Put(pz_src_ipc);
    break;
  case C_M_SM_SSR_STREAM_GEE:
    ret = ipc_upgrade_sm_SsrGeeSpaceStream_Put(pz_src_ipc);
    break;
  case C_M_SM_TWIN_RCV_OBS_RTCM:
    ret = ipc_upgrade_sm_RcvTwinMeasRTCM_Put(pz_src_ipc);
    break;
  case C_M_HPP_INIT:
    ret = ipc_upgrade_hpp_Init(pz_src_ipc);
    break;
  case C_M_HPP_START:
    ret = ipc_upgrade_hpp_Start(pz_src_ipc);
    break;
  case C_M_HPP_STOP:
    ret = ipc_upgrade_hpp_Stop(pz_src_ipc);
    break;
  case C_M_HPP_RELEASE:
    ret = ipc_upgrade_hpp_Release(pz_src_ipc);
    break;
  case C_M_HPP_RCV_MEAS_1HZ_PUT:
    ret = ipc_upgrade_hpp_RcvMeas_Put(pz_src_ipc);
    break;
  case C_M_HPP_REF_CORR_PUT:
    ret = ipc_upgrade_hpp_RefCorr_Put(pz_src_ipc);
    break;
  case C_M_HPP_EPHEMERIS_PUT:
    ret = ipc_upgrade_hpp_Ephemeris_Put(pz_src_ipc);
    break;
  case C_M_HPP_RCV_MEAS_RTCM:
    ret = ipc_upgrade_hpp_RcvMeasRTCM_Put(pz_src_ipc);
    break;
  case C_M_HPP_REF_CORR_RTCM:
    ret = ipc_upgrade_hpp_RefCorrRTCM_Put(pz_src_ipc);
    break;
  case C_M_HPP_RCV_MEAS_TWIN_PUT:
    ret = ipc_upgrade_hpp_RcvMeasTwin_Put(pz_src_ipc);
    break;
  case C_M_DCP_RCV_MEAS_NHZ_PUT:
    ret = ipc_upgrade_dcp_RcvMeasNHZ_PUT(pz_src_ipc);
    break;
  case C_M_PPP_INIT:
    ret = ipc_upgrade_ppp_Init(pz_src_ipc);
    break;
  case C_M_PPP_START:
    ret = ipc_upgrade_ppp_Start(pz_src_ipc);
    break;
  case C_M_PPP_STOP:
    ret = ipc_upgrade_ppp_Stop(pz_src_ipc);
    break;
  case C_M_PPP_RELEASE:
    ret = ipc_upgrade_ppp_Release(pz_src_ipc);
    break;
  case C_M_PPP_RCV_MEAS_1HZ_PUT:
    ret = ipc_upgrade_ppp_RcvMeas_Put(pz_src_ipc);
    break;
  case C_M_PPP_SSR_LOS_BLK:
    ret = ipc_upgrade_ppp_SsrLocBlock_Put(pz_src_ipc);
    break;
  case C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT:
    ret = ipc_upgrade_ppp_TightSatSigMeasCollect_Put(pz_src_ipc);
    break;
  case C_M_PPP_ALGO_START:
    ret = ipc_upgrade_ppp_Algorithm_Start(pz_src_ipc);
    break;
  case C_M_PPP_SSR_STREAM_QX:
    ret = ipc_upgrade_ppp_SsrQianXunStream_Put(pz_src_ipc);
    break;
  case C_M_VDR_GNSS_FIX_PUT:
    ret = ipc_upgrade_vdr_GnssFixPut(pz_src_ipc);
    break;
  default:
    break;
  }

  return ret;
}

BOOL package_upgrade_LOG_PACKAGE_ID_MONITOR(uint8_t** pp_data, uint32_t* pq_length)
{
  BOOL ret = FALSE;
  uint8_t* p_data = *pp_data;
  uint8_t  u_version = p_data[0];

  if (sizeof(loc_MonitorStructType_0) == *pq_length)
  {
    loc_MonitorStructType_0* p_loc_monitor_0 = (loc_MonitorStructType_0*)p_data;
    loc_MonitorStructType*  p_loc_monitor = (loc_MonitorStructType*)OS_MALLOC(sizeof(loc_MonitorStructType));

    memcpy(p_loc_monitor->build_version, p_loc_monitor_0->build_version, sizeof(p_loc_monitor->build_version));
    p_loc_monitor->q_ReportDuration = p_loc_monitor_0->q_ReportDuration;
    p_loc_monitor->t_StartTime      = p_loc_monitor_0->t_StartTime;
    p_loc_monitor->t_RunningTime    = p_loc_monitor_0->t_RunningTime;
    p_loc_monitor->t_ReportCount    = p_loc_monitor_0->t_ReportCount;
    p_loc_monitor->t_ReportCountPrevious = p_loc_monitor_0->t_ReportCountPrevious;
    p_loc_monitor->t_HppReportCount = p_loc_monitor_0->t_HppReportCount;
    p_loc_monitor->t_RtkReportCount = p_loc_monitor_0->t_RtkReportCount;
    p_loc_monitor->t_PppReportCount = p_loc_monitor_0->t_PppReportCount;

    *pp_data = (uint8_t*)p_loc_monitor;
    *pq_length = sizeof(loc_MonitorStructType);
    ret = TRUE;
  }
  else if ((sizeof(loc_MonitorStructType) == *pq_length) && (u_version == VERSION_LOC_MONITOR_INFO))
  {
    ret = FALSE;
  }

  return ret;
}

BOOL package_upgrade_LOG_PACKAGE_ID_LOC_CONFIG(uint8_t** pp_data, uint32_t* pq_length)
{
  BOOL ret = FALSE;
  return FALSE;
}

BOOL package_upgrade_LOG_PACKAGE_ID_INS_RESULT(uint8_t** pp_data, uint32_t* pq_length)
{
  BOOL ret = FALSE;
  return FALSE;
}

BOOL package_upgrade(uint16_t w_PackageId, uint8_t** pp_data, uint32_t* pq_length)
{
  BOOL ret = FALSE;

  switch (w_PackageId)
  {
  case LOG_PACKAGE_ID_MONITOR:
    ret = package_upgrade_LOG_PACKAGE_ID_MONITOR(pp_data, pq_length);
    break;
  case LOG_PACKAGE_ID_LOC_CONFIG:
    ret = package_upgrade_LOG_PACKAGE_ID_LOC_CONFIG(pp_data, pq_length);
    break;
  case LOG_PACKAGE_ID_INS_RESULT:
    ret = package_upgrade_LOG_PACKAGE_ID_INS_RESULT(pp_data, pq_length);
    break;
  default:
    break;
  }
  return ret;
}
