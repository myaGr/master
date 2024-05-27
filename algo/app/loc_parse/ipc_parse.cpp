#include "ag_log_parse.h"
#include "ipc_parse.h"
#include "gnss_common.h"
#include "mw_alloc.h"
#include "ipc_extend_log.h"
#include "sm_nmea.h"
#include "loc_core_report.h"
#include "ipc_upgrade.h"
#include "loc_parse.h"
#include "fusion_api.h"
#include "ag_bddb_parse.h"

typedef uint32_t(*ipc_parse_func)(ipc_t* pz_ipc, char* parse_output);
typedef uint32_t(*report_parse_func)(uint8_t* data, uint32_t length, char* parse_output);

const char* gnss_Constellation_Names[C_GNSS_MAX] = {
  "NONE",
  "GPS",
  "GLO",
  "BDS2",
  "BDS",
  "GAL",
  "QZS",
  "NavIC"
};

const char* gnss_SignalType_Names[C_GNSS_SIG_MAX] = {
  "L1C",
  "L2C",
  "L5Q",
  "G1 ",
  "G2 ",
  "B1I",
  "B2I",
  "B3I",
  "B1C",
  "B2A",
  "B2B",
  "E1 ",
  "E5A",
  "E5B",
  "L1C",
  "L2C",
  "L5Q"
};

typedef struct {
  char  filepath[256];
  FILE* fp;
} IpcParseFileHandler_t;

typedef struct {
  char  input_log_filepath[256];
  /* nmea */
  IpcParseFileHandler_t z_pvt_nmea_handler;
  IpcParseFileHandler_t z_rtk_nmea_handler;
  IpcParseFileHandler_t z_ppp_nmea_handler;
  IpcParseFileHandler_t z_dcp_nmea_handler;
  IpcParseFileHandler_t z_ins_nmea_handler;
  IpcParseFileHandler_t z_ins_gpoly_handler;

  /* rover rtcm */
  IpcParseFileHandler_t z_rover_rtcm_handler;
  IpcParseFileHandler_t z_ort_rover_rtcm_handler;
  IpcParseFileHandler_t z_base_rtcm_handler;

  /* ins BDDB*/
  IpcParseFileHandler_t z_ins_bddb_handler;

  /* orient nmea*/
  IpcParseFileHandler_t z_ort_nmea_handler;
} IpcParseCtrl_t;

static IpcParseCtrl_t gz_IpcParseCtrl = { 0 };
loc_MonitorStructType gz_monitorInfo = { 0 };
uint8_t gu_monitorInfoFlag = 0;

uint8_t get_monitor_flag()
{
  return gu_monitorInfoFlag;
}

loc_MonitorStructType* get_monitor_info()
{
  return &gz_monitorInfo;
}

void log_parse_print_pvt_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_pvt_nmea_handler.fp)
  {
    gz_IpcParseCtrl.z_pvt_nmea_handler.fp = fopen(gz_IpcParseCtrl.z_pvt_nmea_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_pvt_nmea_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_pvt_nmea_handler.fp, str);
  }
}

void log_parse_print_rtk_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_rtk_nmea_handler.fp)
  {
    gz_IpcParseCtrl.z_rtk_nmea_handler.fp = fopen(gz_IpcParseCtrl.z_rtk_nmea_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_rtk_nmea_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_rtk_nmea_handler.fp, str);
  }
}

void log_parse_print_ort_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_ort_nmea_handler.fp)
  {
    gz_IpcParseCtrl.z_ort_nmea_handler.fp = fopen(gz_IpcParseCtrl.z_ort_nmea_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_ort_nmea_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_ort_nmea_handler.fp, str);
  }
}

void log_parse_print_ppp_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_ppp_nmea_handler.fp)
  {
    gz_IpcParseCtrl.z_ppp_nmea_handler.fp = fopen(gz_IpcParseCtrl.z_ppp_nmea_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_ppp_nmea_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_ppp_nmea_handler.fp, str);
  }
}

void log_parse_print_dcp_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_dcp_nmea_handler.fp)
  {
    gz_IpcParseCtrl.z_dcp_nmea_handler.fp = fopen(gz_IpcParseCtrl.z_dcp_nmea_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_dcp_nmea_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_dcp_nmea_handler.fp, str);
  }
}

void log_parse_print_ins_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_ins_nmea_handler.fp)
  {
    gz_IpcParseCtrl.z_ins_nmea_handler.fp = fopen(gz_IpcParseCtrl.z_ins_nmea_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_ins_nmea_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_ins_nmea_handler.fp, str);
  }
}

void log_parse_print_gpimu_gpoly_nmea(char* str)
{
  if (NULL == gz_IpcParseCtrl.z_ins_gpoly_handler.fp)
  {
    gz_IpcParseCtrl.z_ins_gpoly_handler.fp = fopen(gz_IpcParseCtrl.z_ins_gpoly_handler.filepath, "w+");
  }

  if (gz_IpcParseCtrl.z_ins_gpoly_handler.fp)
  {
    fprintf(gz_IpcParseCtrl.z_ins_gpoly_handler.fp, str);
  }
}

void log_parse_print_ins_bddb(char* str, uint32_t len)
{
    if (NULL == gz_IpcParseCtrl.z_ins_bddb_handler.fp)
    {
        gz_IpcParseCtrl.z_ins_bddb_handler.fp = fopen(gz_IpcParseCtrl.z_ins_bddb_handler.filepath, "wb+");
    }

    if (gz_IpcParseCtrl.z_ins_bddb_handler.fp)
    {
        fwrite(str, len, 1, gz_IpcParseCtrl.z_ins_bddb_handler.fp);
    }
}

void log_parse_print_rover_rtcm(const uint8_t* p_data, uint32_t  q_length)
{
  if (NULL == gz_IpcParseCtrl.z_rover_rtcm_handler.fp)
  {
    gz_IpcParseCtrl.z_rover_rtcm_handler.fp = fopen(gz_IpcParseCtrl.z_rover_rtcm_handler.filepath, "wb+");
  }

  if (gz_IpcParseCtrl.z_rover_rtcm_handler.fp)
  {
    fwrite(p_data, q_length, 1, gz_IpcParseCtrl.z_rover_rtcm_handler.fp);
  }
}

void log_parse_print_ort_rover_rtcm(const uint8_t* p_data, uint32_t  q_length)
{
  if (NULL == gz_IpcParseCtrl.z_ort_rover_rtcm_handler.fp)
  {
    gz_IpcParseCtrl.z_ort_rover_rtcm_handler.fp = fopen(gz_IpcParseCtrl.z_ort_rover_rtcm_handler.filepath, "wb+");
  }

  if (gz_IpcParseCtrl.z_ort_rover_rtcm_handler.fp)
  {
    fwrite(p_data, q_length, 1, gz_IpcParseCtrl.z_ort_rover_rtcm_handler.fp);
  }
}

void log_parse_print_base_rtcm(const uint8_t* p_data, uint32_t  q_length)
{
  if (NULL == gz_IpcParseCtrl.z_base_rtcm_handler.fp)
  {
    gz_IpcParseCtrl.z_base_rtcm_handler.fp = fopen(gz_IpcParseCtrl.z_base_rtcm_handler.filepath, "wb+");
  }

  if (gz_IpcParseCtrl.z_base_rtcm_handler.fp)
  {
    fwrite(p_data, q_length, 1, gz_IpcParseCtrl.z_base_rtcm_handler.fp);
  }
}

uint32_t parse_C_M_SM_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_RCV_OBS_RTCM(ipc_t* pz_ipc, char* buf)
{
  log_parse_print_rover_rtcm(pz_ipc->p_data, pz_ipc->q_length);
  return 0;
}

uint32_t parse_C_M_SM_REF_CORR_RTCM(ipc_t* pz_ipc, char* buf)
{
  log_parse_print_base_rtcm(pz_ipc->p_data, pz_ipc->q_length);
  return 0;
}

uint32_t parse_C_M_SM_REPORT_POS_FIX(ipc_t* pz_ipc, char* buf)
{
  gnss_PositionFix_t* pz_pos_solution = (gnss_PositionFix_t*)pz_ipc->p_data;
  uint8_t gga_buf[LOC_API_NMEA_STRING_LEN] = { 0 };
  uint8_t rmc_buf[LOC_API_NMEA_STRING_LEN] = { 0 };
  uint8_t gsv_buf[24][LOC_API_NMEA_STRING_LEN] = { 0 };
  uint8_t gsa_buf[24][LOC_API_NMEA_STRING_LEN] = { 0 };
  uint8_t u_gsv_num = 0;
  uint8_t u_gsa_num = 0;

  if ((FIX_SOURCE_PVT_WLS == pz_pos_solution->u_fixSource) ||
      (FIX_SOURCE_PVT_KF == pz_pos_solution->u_fixSource) ||
      (FIX_SOURCE_HPP_V1 == pz_pos_solution->u_fixSource) ||
      (FIX_SOURCE_HPP_V2 == pz_pos_solution->u_fixSource))
  {
    
    sm_nmea_create_rmc(pz_pos_solution, rmc_buf);
    sm_nmea_create_gga(pz_pos_solution, gga_buf);
    log_parse_print_pvt_nmea((char*)rmc_buf);
    log_parse_print_pvt_nmea((char*)gga_buf);

    static uint64_t q_preHppSecondAlignMsec = 0;
    if (gnss_CheckMeasSecondAlign(pz_pos_solution->z_gpsTime.t_fullMsec, &q_preHppSecondAlignMsec, 1.0))
    {
      u_gsv_num = sm_nmea_create_gsv(pz_pos_solution, gsv_buf, sizeof(gsv_buf) / sizeof(gsv_buf[0]));
      u_gsa_num = sm_nmea_create_gsa(pz_pos_solution, gsa_buf, sizeof(gsa_buf) / sizeof(gsa_buf[0]));
      for (int i = 0; i < u_gsv_num; i++)
      {
        log_parse_print_pvt_nmea((char*)gsv_buf[i]);
      }
      for (int i = 0; i < u_gsa_num; i++)
      {
        log_parse_print_pvt_nmea((char*)gsa_buf[i]);
      }
    }
  }
  else if ((FIX_SOURCE_RTD == pz_pos_solution->u_fixSource) ||
           (FIX_SOURCE_RTK == pz_pos_solution->u_fixSource) ||
           (FIX_SOURCE_RTK_FLOAT == pz_pos_solution->u_fixSource) ||
           (FIX_SOURCE_RTK_FIX == pz_pos_solution->u_fixSource))
  {
    sm_nmea_create_rmc(pz_pos_solution, rmc_buf);
    sm_nmea_create_gga(pz_pos_solution, gga_buf);
    log_parse_print_rtk_nmea((char*)rmc_buf);
    log_parse_print_rtk_nmea((char*)gga_buf);

    static uint64_t q_preRtkSecondAlignMsec = 0;
    if (gnss_CheckMeasSecondAlign(pz_pos_solution->z_gpsTime.t_fullMsec, &q_preRtkSecondAlignMsec, 1.0))
    {
      u_gsv_num = sm_nmea_create_gsv(pz_pos_solution, gsv_buf, sizeof(gsv_buf) / sizeof(gsv_buf[0]));
      u_gsa_num = sm_nmea_create_gsa(pz_pos_solution, gsa_buf, sizeof(gsa_buf) / sizeof(gsa_buf[0]));
      for (int i = 0; i < u_gsv_num; i++)
      {
        log_parse_print_rtk_nmea((char*)gsv_buf[i]);
      }
      for (int i = 0; i < u_gsa_num; i++)
      {
        log_parse_print_rtk_nmea((char*)gsa_buf[i]);
      }
    }
  }   
  else if ((FIX_SOURCE_PPP == pz_pos_solution->u_fixSource) ||
          (FIX_SOURCE_PPP_FLOAT == pz_pos_solution->u_fixSource) ||
          (FIX_SOURCE_PPP_FIX == pz_pos_solution->u_fixSource))
  {
    sm_nmea_create_rmc(pz_pos_solution, rmc_buf);
    sm_nmea_create_gga(pz_pos_solution, gga_buf);
    log_parse_print_ppp_nmea((char*)rmc_buf);
    log_parse_print_ppp_nmea((char*)gga_buf);

    static uint64_t q_prePppSecondAlignMsec = 0;
    if (gnss_CheckMeasSecondAlign(pz_pos_solution->z_gpsTime.t_fullMsec, &q_prePppSecondAlignMsec, 1.0))
    {
      u_gsv_num = sm_nmea_create_gsv(pz_pos_solution, gsv_buf, sizeof(gsv_buf) / sizeof(gsv_buf[0]));
      u_gsa_num = sm_nmea_create_gsa(pz_pos_solution, gsa_buf, sizeof(gsa_buf) / sizeof(gsa_buf[0]));
      for (int i = 0; i < u_gsv_num; i++)
      {
        log_parse_print_ppp_nmea((char*)gsv_buf[i]);
      }
      for (int i = 0; i < u_gsa_num; i++)
      {
        log_parse_print_ppp_nmea((char*)gsa_buf[i]);
      }
    }
  }
  else if (FIX_SOURCE_DCP == pz_pos_solution->u_fixSource)
  {
    sm_nmea_create_rmc(pz_pos_solution, rmc_buf);
    sm_nmea_create_gga(pz_pos_solution, gga_buf);
    log_parse_print_dcp_nmea((char*)rmc_buf);
    log_parse_print_dcp_nmea((char*)gga_buf);

    static uint64_t q_preDcpSecondAlignMsec = 0;
    if (gnss_CheckMeasSecondAlign(pz_pos_solution->z_gpsTime.t_fullMsec, &q_preDcpSecondAlignMsec, 1.0))
    {
      u_gsv_num = sm_nmea_create_gsv(pz_pos_solution, gsv_buf, sizeof(gsv_buf) / sizeof(gsv_buf[0]));
      u_gsa_num = sm_nmea_create_gsa(pz_pos_solution, gsa_buf, sizeof(gsa_buf) / sizeof(gsa_buf[0]));

      for (int i = 0; i < u_gsv_num; i++)
      {
        log_parse_print_dcp_nmea((char*)gsv_buf[i]);
      }
      for (int i = 0; i < u_gsa_num; i++)
      {
        log_parse_print_dcp_nmea((char*)gsa_buf[i]);
      }
    }
  }
  else if (FIX_SOURCE_VDR == pz_pos_solution->u_fixSource)
  {
    sm_nmea_create_rmc(pz_pos_solution, rmc_buf);
    sm_nmea_create_gga(pz_pos_solution, gga_buf);
    log_parse_print_ins_nmea((char*)rmc_buf);
    log_parse_print_ins_nmea((char*)gga_buf);

#if 0
    gnss_PositionFix_t z_PositionFix = { 0 };
    memcpy(&z_PositionFix, pz_pos_solution, sizeof(z_PositionFix));

    char gpinr_buf[1024] = { 0 };
    INSResults_t z_InsPositionFix = { 0 };
    INSResults_t* pz_InsPositionFix = &z_InsPositionFix;
    float f_misb2v_roll_std = 0;
    float f_misb2v_pitch_std = 0;
    float f_misb2v_heading_std = 0;
    
    pz_InsPositionFix->q_tow        = pz_pos_solution->z_gpsTime.q_towMsec;
    pz_InsPositionFix->d_latitude   = z_PositionFix.d_lla[0];
    pz_InsPositionFix->d_longitude  = z_PositionFix.d_lla[1];
    pz_InsPositionFix->f_altitude   = (float)z_PositionFix.d_lla[2];
    pz_InsPositionFix->f_ve         = z_PositionFix.f_velEnu[0];
    pz_InsPositionFix->f_vn         = z_PositionFix.f_velEnu[1];
    pz_InsPositionFix->f_vd         = -z_PositionFix.f_velEnu[2];
    pz_InsPositionFix->f_lat_std    = z_PositionFix.f_posLlaUnc[0];
    pz_InsPositionFix->f_lon_std    = z_PositionFix.f_posLlaUnc[1];
    pz_InsPositionFix->f_lon_std    = z_PositionFix.f_posLlaUnc[2];
    pz_InsPositionFix->f_ve_std     = z_PositionFix.f_velEnuUnc[0];
    pz_InsPositionFix->f_vn_std     = z_PositionFix.f_velEnuUnc[1];
    pz_InsPositionFix->f_vd_std     = -z_PositionFix.f_velEnuUnc[2];

    sprintf(gpinr_buf, "$ASINR,%d,%d,%15.9f,%15.9f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            pz_InsPositionFix->q_tow, (int)pz_InsPositionFix->e_drposflag,
            pz_InsPositionFix->d_latitude * RAD2DEG, pz_InsPositionFix->d_longitude * RAD2DEG, pz_InsPositionFix->f_altitude,
            pz_InsPositionFix->f_vn, pz_InsPositionFix->f_ve, pz_InsPositionFix->f_vd,
            pz_InsPositionFix->f_heading, pz_InsPositionFix->f_pitch, pz_InsPositionFix->f_roll,
            pz_InsPositionFix->f_gyrobias_x, pz_InsPositionFix->f_gyrobias_y, pz_InsPositionFix->f_gyrobias_z,
            pz_InsPositionFix->f_accbias_x, pz_InsPositionFix->f_accbias_y, pz_InsPositionFix->f_accbias_z, pz_InsPositionFix->u_zuptflag, pz_InsPositionFix->f_mis_roll, pz_InsPositionFix->f_mis_pitch, pz_InsPositionFix->f_mis_yaw,
            pz_InsPositionFix->f_whlspd_sf[2], pz_InsPositionFix->f_whlspd_sf[3], pz_InsPositionFix->f_lat_std, pz_InsPositionFix->f_lon_std, pz_InsPositionFix->f_alt_std,
            f_misb2v_roll_std, f_misb2v_pitch_std, f_misb2v_heading_std);
    log_parse_print_gpimu_gpoly_nmea(gpinr_buf);
    memset(gpinr_buf, 0, sizeof(gpinr_buf));
    uint32_t len = ag_imu_bddb_encode_0x0B(pz_InsPositionFix, (int8_t*)gpinr_buf);
    log_parse_print_ins_bddb(gpinr_buf, len);
#endif
  }

  uint32_t index = 0;

  index += sprintf(buf + index, "C_M_SM_REPORT_POS_FIX Version %d Source %d GPST Week %4d tow %9d ms\n",
      pz_pos_solution->u_version,
      pz_pos_solution->u_fixSource,
      pz_pos_solution->z_gpsTime.w_week, 
      pz_pos_solution->z_gpsTime.q_towMsec);

  index += sprintf(buf + index, "Pos [Lat %0.6f Lon %0.6f Alt %0.6f] Vel [E %0.6f N %0.6f U %0.6f]\n",
      pz_pos_solution->d_lla[0] * RAD2DEG,
      pz_pos_solution->d_lla[1] * RAD2DEG,
      pz_pos_solution->d_lla[2],
      pz_pos_solution->f_velEnu[0],
      pz_pos_solution->f_velEnu[1],
      pz_pos_solution->f_velEnu[2]);
  index += sprintf(buf + index, "-------------------------------------------------------------------------------------------------------------\n");

  return index;
}
uint32_t parse_C_M_SM_REPORT_ORT_FIX(ipc_t* pz_ipc, char* buf)
{
  uint8_t nmea_hdt[256] = { 0 };
  uint8_t nmea_tra[256] = { 0 };
  uint8_t nmea_ksxt[256] = { 0 };
  gnss_OrientFix_t* pz_ort_solution = (gnss_OrientFix_t*)pz_ipc->p_data;
  sm_nmea_creat_hdt(pz_ort_solution, nmea_hdt);
  sm_nmea_creat_tra(pz_ort_solution, nmea_tra);
  sm_nmea_creat_ksxt(pz_ort_solution, nmea_ksxt);
  log_parse_print_ort_nmea((char*)nmea_tra);
  log_parse_print_ort_nmea((char*)nmea_hdt);
  log_parse_print_ort_nmea((char*)nmea_ksxt);
  return 0;
}
uint32_t parse_C_M_SM_SSR_LOS_BLK(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_RCV_MEAS_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_SSR_STREAM_QX(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_SSR_STREAM_GEE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_TWIN_RCV_OBS_RTCM(ipc_t* pz_ipc, char* buf)
{
  log_parse_print_ort_rover_rtcm(pz_ipc->p_data, pz_ipc->q_length);
  return 0;
}

uint32_t parse_C_M_SM_REF_CORR_MEAS_BLK(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_REPORT_GNSS_FB_INS(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_REPORT_GNSS_EPH(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SM_REPORT_GNSS_OBS(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_RCV_MEAS_1HZ_PUT(ipc_t* pz_ipc, char* buf)
{
  GnssMeasBlock_t* pz_GnssMeasBlock = (GnssMeasBlock_t*)pz_ipc->p_data;

  uint32_t index = 0;

  uint32_t q_PerSignalMeasNumber[C_GNSS_SIG_MAX] = { 0 };
  float    f_PerSignalAvgCN0[C_GNSS_SIG_MAX] = { 0 };

  index += sprintf(buf + index, "C_M_HPP_RCV_MEAS_1HZ_PUT Version %d GPST Week %4d tow %9d ms MeasCount %d\n",
    pz_GnssMeasBlock->u_version,
    pz_GnssMeasBlock->z_Clock.z_gpsTime.w_week,
    pz_GnssMeasBlock->z_Clock.z_gpsTime.q_towMsec,
    pz_GnssMeasBlock->w_measCount);

  for (int i = 0; i < pz_GnssMeasBlock->w_measCount; i++)
  {
    GnssMeas_t* pz_GnssMeas = &pz_GnssMeasBlock->z_meas[i];
    q_PerSignalMeasNumber[pz_GnssMeas->u_signal]++;
    f_PerSignalAvgCN0[pz_GnssMeas->u_signal]+= pz_GnssMeas->f_cn0;
  }

  index += sprintf(buf + index, "       Cnt AvgCn0\n");
  for (uint8_t u_Signal = C_GNSS_SIG_GPS_L1C; u_Signal < C_GNSS_SIG_MAX; u_Signal++)
  {
    gnss_ConstellationType u_Constellation = gnss_cvt_Signal2Constellation(u_Signal);

    if (q_PerSignalMeasNumber[u_Signal] > 0)
    {
      index += sprintf(buf + index, "%3s %3s %2d   %0.1f\n",
        gnss_Constellation_Names[u_Constellation],
        gnss_SignalType_Names[u_Signal],
        q_PerSignalMeasNumber[u_Signal],
        f_PerSignalAvgCN0[u_Signal] / q_PerSignalMeasNumber[u_Signal]);
    }
  }

  index += sprintf(buf + index, "No:   Svid Sig CN0     Status      PseudoRange      Doppler      CarrierPhase LLI\n");
  for (int i = 0; i < pz_GnssMeasBlock->w_measCount; i++)
  {
    GnssMeas_t* pz_GnssMeas = &pz_GnssMeasBlock->z_meas[i];
    uint64_t t_MeasStatus = 0;
    memcpy(&t_MeasStatus, &pz_GnssMeas->z_measStatusFlag, sizeof(uint64_t));
    index += sprintf(buf + index, "%2d: %3s %02d %3s %3d 0x%08x %16.6f %12.6f %17.6f %3d\n",
      i,
      gnss_Constellation_Names[pz_GnssMeas->u_constellation],
      pz_GnssMeas->u_svid,
      gnss_SignalType_Names[pz_GnssMeas->u_signal],
      (uint32_t)pz_GnssMeas->f_cn0,
      (uint32_t)t_MeasStatus,
      pz_GnssMeas->d_pseudoRange,
      pz_GnssMeas->d_doppler,
      pz_GnssMeas->d_carrierPhase,
      pz_GnssMeas->u_LLI);
  }

  index += sprintf(buf + index, "-------------------------------------------------------------------------------------------------------------\n");
  return index;
}

uint32_t parse_C_M_HPP_REF_CORR_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_EPHEMERIS_PUT(ipc_t* pz_ipc, char* buf)
{
  gnss_Ephemeris_t* pz_gnss_Ephemeris = (gnss_Ephemeris_t*)pz_ipc->p_data;

  uint32_t index = 0;

  index += sprintf(buf + index, "C_M_HPP_EPHEMERIS_PUT Version %d GPST Week %4d tow %9dms ", 
    pz_gnss_Ephemeris->u_version,
    pz_gnss_Ephemeris->z_toeOfGpst.w_week,
    pz_gnss_Ephemeris->z_toeOfGpst.q_towMsec);
  index += sprintf(buf + index, "%3s ", gnss_Constellation_Names[pz_gnss_Ephemeris->u_constellation]);

  switch (pz_gnss_Ephemeris->u_constellation)
  {
  case C_GNSS_GPS:
    index += sprintf(buf + index, "Svid %02d Toe %12.6f", 
      pz_gnss_Ephemeris->eph.z_gpsEph.svid,
      pz_gnss_Ephemeris->eph.z_gpsEph.toe);
    break;
  case C_GNSS_GLO:
    break;
  case C_GNSS_BDS3:
    index += sprintf(buf + index, "Svid %02d Toe %12.6f",
      pz_gnss_Ephemeris->eph.z_bdsEph.svid,
      pz_gnss_Ephemeris->eph.z_bdsEph.toe);
    break;
  case C_GNSS_GAL:
    index += sprintf(buf + index, "Svid %02d Toe %12.6f",
      pz_gnss_Ephemeris->eph.z_galEph.svid,
      pz_gnss_Ephemeris->eph.z_galEph.toe);
    break;
  case C_GNSS_QZS:
    index += sprintf(buf + index, "Svid %02d Toe %12.6f",
      pz_gnss_Ephemeris->eph.z_qzsEph.svid,
      pz_gnss_Ephemeris->eph.z_qzsEph.toe);
    break;
  default:
    break;
  }
  index += sprintf(buf + index, "\n");

  index += sprintf(buf + index, "-------------------------------------------------------------------------------------------------------------\n");
  return index;
}

uint32_t parse_C_M_HPP_RCV_MEAS_RTCM(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_REF_CORR_RTCM(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_HPP_RCV_MEAS_TWIN_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_DCP_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_DCP_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_DCP_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_DCP_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_DCP_RCV_MEAS_NHZ_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_DCP_TDCP_MEAS_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_RCV_MEAS_1HZ_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_SSR_LOS_BLK(ipc_t* pz_ipc, char* buf)
{
  gnss_ssrLosBlock_t* pz_SsrLocBlk = (gnss_ssrLosBlock_t*)pz_ipc->p_data;
  char sats[4] = { 0 };

  uint32_t index = 0;

  index += sprintf(buf + index, "C_M_PPP_SSR_LOS_BLK Version %d ", pz_SsrLocBlk->u_version);
  index += sprintf(buf + index, "Week %4d tow %9d ms Count %d\n", 
    pz_SsrLocBlk->z_epochLosInfo.z_tor.w_week,
    pz_SsrLocBlk->z_epochLosInfo.z_tor.q_towMsec,
    pz_SsrLocBlk->z_epochLosInfo.u_satCount);

  for (size_t i = 0; i < pz_SsrLocBlk->z_epochLosInfo.u_satCount; i++)
  {
    const gnss_satSsrLos_t* los = pz_SsrLocBlk->z_epochLosInfo.z_satLosCorr + i;
    svid_SatString(los->u_svid, los->u_constellation, sats);
    index += sprintf(buf + index, "%s,$mask,%d %d %d %4d,$pos,%13.2f,%13.2f,%13.2f\n", sats, los->u_orbClkMask, los->u_atmoMask, los->u_windUpMask, los->q_iode,
      los->z_satPosVelClkBrdc.d_satPosClk[0], los->z_satPosVelClkBrdc.d_satPosClk[1], los->z_satPosVelClkBrdc.d_satPosClk[2]);
  }

  index += sprintf(buf + index, "-------------------------------------------------------------------------------------------------------------\n");
  return index;
}

uint32_t parse_C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT(ipc_t* pz_ipc, char* buf)
{
  gnss_TightSatSigMeasCollect_t* pz_TightSatSigMeasCollect = (gnss_TightSatSigMeasCollect_t*)pz_ipc->p_data;

  uint32_t index = 0;

  index += sprintf(buf + index, "C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT Version %d ", pz_TightSatSigMeasCollect->u_version);
  index += sprintf(buf + index, "Week %4d tow %9d.%06dms Meas_Count %d\n",
              pz_TightSatSigMeasCollect->z_tor.w_week,
              pz_TightSatSigMeasCollect->z_tor.q_towMsec,
              pz_TightSatSigMeasCollect->z_tor.q_subNsec*1000000,
              pz_TightSatSigMeasCollect->u_satMeasCount);

  index += sprintf(buf + index, "  Svid Sig CN0     PseudoRange      Doppler      CarrierPhase\n");

  for (uint32_t i = 0; i < pz_TightSatSigMeasCollect->u_satMeasCount; i++)
  {
    gnss_TightSatelliteMeas_t* pz_TightSatelliteMeas = &pz_TightSatSigMeasCollect->pz_tightSatMeas[i];

    for (uint32_t j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++)
    {
      gnss_SignalMeas_t* pz_SignalMeas = &pz_TightSatelliteMeas->pz_signalMeas[j];
      if (FALSE == pz_SignalMeas->z_measStatusFlag.b_valid)
      {
        continue;
      }

      index += sprintf(buf + index, "%3s %2d %s %3d %12.6f %12.6f %17.6f\n", 
        gnss_Constellation_Names[pz_SignalMeas->u_constellation],
        pz_SignalMeas->u_svid,
        gnss_SignalType_Names[pz_SignalMeas->u_signal],
        (uint8_t)pz_SignalMeas->f_cn0,
        pz_SignalMeas->d_pseudoRange,
        pz_SignalMeas->d_doppler,
        pz_SignalMeas->d_carrierPhase);
    }
  }

  index += sprintf(buf + index, "-------------------------------------------------------------------------------------------------------------\n");
  return index;
}

uint32_t parse_C_M_PPP_ALGO_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_SSR_STREAM_QX(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_PPP_SSR_STREAM_GEE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SD_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SD_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SD_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SD_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SD_EPHEMERIS_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_SD_SAT_PVT_POLY_REQUEST(ipc_t* pz_ipc, char* buf)
{
  return 0;

}
uint32_t parse_C_M_VDR_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_VDR_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_VDR_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_VDR_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_VDR_IMU_DATA_PUT(ipc_t* pz_ipc, char* buf)
{
  ins_ImuData_t* pz_ImuData = (ins_ImuData_t*)pz_ipc->p_data;
  char gpimu_str[256] = { 0 };
  AsensingIMU_t z_AsensingIMU = { 0 };

  z_AsensingIMU.t_timestamp = pz_ImuData->q_tow_msec;
  for (int i = 0; i < 3; i++)
  {
      z_AsensingIMU.f_gyro[i] = pz_ImuData->f_gyro[i];
      z_AsensingIMU.f_accl[i] = pz_ImuData->f_accl[i];
  }
  z_AsensingIMU.f_temp = pz_ImuData->f_temperature;

  sprintf(gpimu_str, "$GPIMU,%u,%f,%f,%f,%f,%f,%f,%f,%d\n",
          pz_ImuData->q_tow_msec,
          pz_ImuData->f_gyro[0], pz_ImuData->f_gyro[1], pz_ImuData->f_gyro[2],
          pz_ImuData->f_accl[0], pz_ImuData->f_accl[1], pz_ImuData->f_accl[2],
          pz_ImuData->f_temperature, 0);
  log_parse_print_gpimu_gpoly_nmea(gpimu_str);
  memset(gpimu_str, 0, sizeof(gpimu_str));
  uint32_t len = ag_imu_bddb_encode_0x0A(&z_AsensingIMU, (int8_t *)gpimu_str);
  log_parse_print_ins_bddb(gpimu_str, len);

  uint32_t index = 0;

  index += sprintf(buf + index, "C_M_VDR_IMU_DATA_PUT Version %d ", pz_ImuData->u_version);
  index += sprintf(buf + index, "Tow %d gyro %9.6f %9.6f %9.6f accl %9.6f %9.6f %9.6f temperature %6.3f\n",
                   pz_ImuData->q_tow_msec,
                   pz_ImuData->f_gyro[0], pz_ImuData->f_gyro[1], pz_ImuData->f_gyro[2],
                   pz_ImuData->f_accl[0], pz_ImuData->f_accl[1], pz_ImuData->f_accl[2],
                   pz_ImuData->f_temperature);
  return index;
}

uint32_t parse_C_M_VDR_WHEEL_DATA_PUT(ipc_t* pz_ipc, char* buf)
{
  ins_WheelData_t* pz_WheelData = (ins_WheelData_t*)pz_ipc->p_data;
  char gpwhl_str[256] = { 0 };
  AsensingWhpulse_t z_AsensingWheel = { 0 };

  z_AsensingWheel.t_timestamp = pz_WheelData->t_timestamp;
  z_AsensingWheel.q_fl_whpulse = pz_WheelData->q_fl_whpulse;
  z_AsensingWheel.q_fr_whpulse = pz_WheelData->q_fr_whpulse;
  z_AsensingWheel.q_rl_whpulse = pz_WheelData->q_rl_whpulse;
  z_AsensingWheel.q_rr_whpulse = pz_WheelData->q_rr_whpulse;
  z_AsensingWheel.f_angle_front = pz_WheelData->f_angle_front;
  z_AsensingWheel.f_angle_rear = pz_WheelData->f_angle_rear;
  z_AsensingWheel.e_gear = (GearShift_Enum)pz_WheelData->e_gear;

  sprintf(gpwhl_str, "$GPWHL,%llu,%d,%llu,%d,%d,%d,%d,%d\n",
      pz_WheelData->t_timestamp, pz_WheelData->e_gear, pz_WheelData->t_timestamp,
      pz_WheelData->q_fl_whpulse, pz_WheelData->q_fr_whpulse, pz_WheelData->q_rl_whpulse, pz_WheelData->q_rr_whpulse,
      1);
  log_parse_print_gpimu_gpoly_nmea(gpwhl_str);
  memset(gpwhl_str, 0, sizeof(gpwhl_str));
  uint32_t len = ag_imu_bddb_encode_0x20(&z_AsensingWheel, (int8_t*)gpwhl_str);
  log_parse_print_ins_bddb(gpwhl_str, len);

  uint32_t index = 0;

  index += sprintf(buf + index, "C_M_VDR_WHEEL_DATA_PUT Version %d ", pz_WheelData->u_version);
  index += sprintf(buf + index, "timestamp %lld fl %d fr %d rl %d rr %d f_angle %f r_angle %f gear %d\n",
                   pz_WheelData->t_timestamp,
                   pz_WheelData->q_fl_whpulse, pz_WheelData->q_fr_whpulse, pz_WheelData->q_rl_whpulse,
                   pz_WheelData->q_rr_whpulse, pz_WheelData->f_angle_front, pz_WheelData->f_angle_rear,
                   pz_WheelData->e_gear);

  return index;
}

uint32_t parse_C_M_VDR_GNSS_FIX_PUT(ipc_t* pz_ipc, char* buf)
{
  ins_GnssFix_t* pz_GnssFix = (ins_GnssFix_t*)pz_ipc->p_data;
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
  z_AsensingGNSS.d_tow = pz_GnssFix->d_tow;
  z_AsensingGNSS.f_avgsnr = pz_GnssFix->f_avgsnr;
  z_AsensingGNSS.w_week = pz_GnssFix->w_week;
  z_AsensingGNSS.u_postype = 1; // Force set as 1
  z_AsensingGNSS.u_satused = pz_GnssFix->u_satused;
  z_AsensingGNSS.e_posflag = (GnssPosFlag_Enum)pz_GnssFix->u_postype;
  z_AsensingGNSS.u_msg_type = (GnssMsgType_Enum)0;
  z_AsensingGNSS.f_heading_deg = 0;
  z_AsensingGNSS.f_heading_std = 0;
  z_AsensingGNSS.f_pitch_deg = 0;
  z_AsensingGNSS.f_pitch_std = 0;

  char gpoly_str[256] = { 0 };
  sprintf(gpoly_str, "$GPOLY,%lld,%15.9f,%15.9f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%d,%.3f,%d,%f,%f,%f,%f,%f,%d,%f,%f\n",
          z_AsensingGNSS.t_timestamp, z_AsensingGNSS.d_lat, z_AsensingGNSS.d_lon, z_AsensingGNSS.f_alt,
          z_AsensingGNSS.f_vn, z_AsensingGNSS.f_ve, z_AsensingGNSS.f_vd,
          z_AsensingGNSS.e_posflag, z_AsensingGNSS.f_latstd, z_AsensingGNSS.f_lonstd, z_AsensingGNSS.f_altstd,
          z_AsensingGNSS.f_vnstd, z_AsensingGNSS.f_vestd, z_AsensingGNSS.f_vdstd,
          z_AsensingGNSS.w_week, z_AsensingGNSS.d_tow/1000.0,
          z_AsensingGNSS.u_satused, z_AsensingGNSS.f_avgsnr, z_AsensingGNSS.f_pitch_deg,
          z_AsensingGNSS.f_heading_deg, z_AsensingGNSS.f_pitch_std, z_AsensingGNSS.f_heading_std, 0,
          z_AsensingGNSS.f_speed, z_AsensingGNSS.f_heading);
  log_parse_print_gpimu_gpoly_nmea(gpoly_str);
  memset(gpoly_str, 0, sizeof(gpoly_str));
  uint32_t len = ag_imu_bddb_encode_0x10(&z_AsensingGNSS, (int8_t*)gpoly_str);
  log_parse_print_ins_bddb(gpoly_str, len);
  
  return 0;
}

uint32_t parse_C_M_VDR_EKF_UPDATE_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_VDR_GNSS_MEAS_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_EKF_INIT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_EKF_START(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_EKF_STOP(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_EKF_RELEASE(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

uint32_t parse_C_M_EKF_SEM_DATA_PUT(ipc_t* pz_ipc, char* buf)
{
  return 0;
}

static struct {
  uint32_t ipc_id;
  const char* ipc_name;
  ipc_parse_func func;
} ipc_parse_functions[] = {
  {C_M_SM_INIT,                         "C_M_SM_INIT",                      parse_C_M_SM_INIT                       },
  {C_M_SM_START,                        "C_M_SM_START",                     parse_C_M_SM_START                      },
  {C_M_SM_STOP,                         "C_M_SM_STOP",                      parse_C_M_SM_STOP                       },
  {C_M_SM_RELEASE,                      "C_M_SM_RELEASE",                   parse_C_M_SM_RELEASE                    },
  {C_M_SM_RCV_OBS_RTCM,                 "C_M_SM_RCV_OBS_RTCM",              parse_C_M_SM_RCV_OBS_RTCM               },
  {C_M_SM_REF_CORR_RTCM,                "C_M_SM_REF_CORR_RTCM",             parse_C_M_SM_REF_CORR_RTCM              },
  {C_M_SM_REPORT_POS_FIX,               "C_M_SM_REPORT_POS_FIX",            parse_C_M_SM_REPORT_POS_FIX             },
  {C_M_SM_SSR_LOS_BLK,                  "C_M_SM_SSR_LOS_BLK",               parse_C_M_SM_SSR_LOS_BLK                },
  {C_M_SM_RCV_MEAS_PUT,                 "C_M_SM_RCV_MEAS_PUT",              parse_C_M_SM_RCV_MEAS_PUT               },
  {C_M_SM_SSR_STREAM_QX,                "C_M_SM_SSR_STREAM_QX",             parse_C_M_SM_SSR_STREAM_QX              },
  {C_M_SM_SSR_STREAM_GEE,               "C_M_SM_SSR_STREAM_GEE",            parse_C_M_SM_SSR_STREAM_GEE             },
  {C_M_SM_TWIN_RCV_OBS_RTCM,            "C_M_SM_TWIN_RCV_OBS_RTCM",         parse_C_M_SM_TWIN_RCV_OBS_RTCM          },
  {C_M_SM_REF_CORR_MEAS_BLK,            "C_M_SM_REF_CORR_MEAS_BLK",         parse_C_M_SM_REF_CORR_MEAS_BLK          },
  {C_M_SM_REPORT_GNSS_FB_INS,           "C_M_SM_REPORT_GNSS_FB_INS",        parse_C_M_SM_REPORT_GNSS_FB_INS         },
  {C_M_SM_REPORT_GNSS_EPH,              "C_M_SM_REPORT_GNSS_EPH",           parse_C_M_SM_REPORT_GNSS_EPH            },
  {C_M_HPP_INIT,                        "C_M_HPP_INIT",                     parse_C_M_HPP_INIT                      },
  {C_M_HPP_START,                       "C_M_HPP_START",                    parse_C_M_HPP_START                     },
  {C_M_HPP_STOP,                        "C_M_HPP_STOP",                     parse_C_M_HPP_STOP                      },
  {C_M_HPP_RELEASE,                     "C_M_HPP_RELEASE",                  parse_C_M_HPP_RELEASE                   },
  {C_M_HPP_RCV_MEAS_1HZ_PUT,            "C_M_HPP_RCV_MEAS_1HZ_PUT",         parse_C_M_HPP_RCV_MEAS_1HZ_PUT          },
  {C_M_HPP_REF_CORR_PUT,                "C_M_HPP_REF_CORR_PUT",             parse_C_M_HPP_REF_CORR_PUT              },
  {C_M_HPP_EPHEMERIS_PUT,               "C_M_HPP_EPHEMERIS_PUT",            parse_C_M_HPP_EPHEMERIS_PUT             },
  {C_M_HPP_RCV_MEAS_RTCM,               "C_M_HPP_RCV_MEAS_RTCM",            parse_C_M_HPP_RCV_MEAS_RTCM             },
  {C_M_HPP_REF_CORR_RTCM,               "C_M_HPP_REF_CORR_RTCM",            parse_C_M_HPP_REF_CORR_RTCM             },
  {C_M_HPP_RCV_MEAS_TWIN_PUT,           "C_M_HPP_RCV_MEAS_TWIN_PUT",        parse_C_M_HPP_RCV_MEAS_TWIN_PUT         },
  {C_M_DCP_INIT,                        "C_M_DCP_INIT",                     parse_C_M_DCP_INIT                      },
  {C_M_DCP_START,                       "C_M_DCP_START",                    parse_C_M_DCP_START                     },
  {C_M_DCP_STOP,                        "C_M_DCP_STOP",                     parse_C_M_DCP_STOP                      },
  {C_M_DCP_RELEASE,                     "C_M_DCP_RELEASE",                  parse_C_M_DCP_RELEASE                   },
  {C_M_DCP_RCV_MEAS_NHZ_PUT,            "C_M_DCP_RCV_MEAS_NHZ_PUT",         parse_C_M_DCP_RCV_MEAS_NHZ_PUT          },
  {C_M_DCP_TDCP_MEAS_PUT,               "C_M_DCP_TDCP_MEAS_PUT",            parse_C_M_DCP_TDCP_MEAS_PUT             },
  {C_M_PPP_INIT,                        "C_M_PPP_INIT",                     parse_C_M_PPP_INIT                      },
  {C_M_PPP_START,                       "C_M_PPP_START",                    parse_C_M_PPP_START                     },
  {C_M_PPP_STOP,                        "C_M_PPP_STOP",                     parse_C_M_PPP_STOP                      },
  {C_M_PPP_RELEASE,                     "C_M_PPP_RELEASE",                  parse_C_M_PPP_RELEASE                   },
  {C_M_PPP_RCV_MEAS_1HZ_PUT,            "C_M_PPP_RCV_MEAS_1HZ_PUT",         parse_C_M_PPP_RCV_MEAS_1HZ_PUT          },
  {C_M_PPP_SSR_LOS_BLK,                 "C_M_PPP_SSR_LOS_BLK",              parse_C_M_PPP_SSR_LOS_BLK               },
  {C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT,   "C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT",parse_C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT },
  {C_M_PPP_ALGO_START,                  "C_M_PPP_ALGO_START",               parse_C_M_PPP_ALGO_START                },
  {C_M_PPP_SSR_STREAM_QX,               "C_M_PPP_SSR_STREAM_QX",            parse_C_M_PPP_SSR_STREAM_QX             },
  {C_M_PPP_SSR_STREAM_GEE,              "C_M_PPP_SSR_STREAM_GEE",           parse_C_M_PPP_SSR_STREAM_GEE            },
  {C_M_SD_INIT,                         "C_M_SD_INIT",                      parse_C_M_SD_INIT                       },
  {C_M_SD_START,                        "C_M_SD_START",                     parse_C_M_SD_START                      },
  {C_M_SD_STOP,                         "C_M_SD_STOP",                      parse_C_M_SD_STOP                       },
  {C_M_SD_RELEASE,                      "C_M_SD_RELEASE",                   parse_C_M_SD_RELEASE                    },
  {C_M_SD_EPHEMERIS_PUT,                "C_M_SD_EPHEMERIS_PUT",             parse_C_M_SD_EPHEMERIS_PUT              },
  {C_M_SD_SAT_PVT_POLY_REQUEST,         "C_M_SD_SAT_PVT_POLY_REQUEST",      parse_C_M_SD_SAT_PVT_POLY_REQUEST       },
  {C_M_VDR_INIT,                        "C_M_VDR_INIT",                     parse_C_M_VDR_INIT                      },
  {C_M_VDR_START,                       "C_M_VDR_START",                    parse_C_M_VDR_START                     },
  {C_M_VDR_STOP,                        "C_M_VDR_STOP",                     parse_C_M_VDR_STOP                      },
  {C_M_VDR_RELEASE,                     "C_M_VDR_RELEASE",                  parse_C_M_VDR_RELEASE                   },
  {C_M_VDR_IMU_DATA_PUT,                "C_M_VDR_IMU_DATA_PUT",             parse_C_M_VDR_IMU_DATA_PUT              },
  {C_M_VDR_GNSS_FIX_PUT,                "C_M_VDR_GNSS_FIX_PUT",             parse_C_M_VDR_GNSS_FIX_PUT              },
  {C_M_VDR_WHEEL_DATA_PUT,              "C_M_VDR_WHEEL_DATA_PUT",           parse_C_M_VDR_WHEEL_DATA_PUT            },
  {C_M_VDR_EKF_UPDATE_PUT,              "C_M_VDR_EKF_UPDATE_PUT",           parse_C_M_VDR_EKF_UPDATE_PUT            },
  {C_M_VDR_GNSS_MEAS_PUT,               "C_M_VDR_GNSS_MEAS_PUT",            parse_C_M_VDR_GNSS_MEAS_PUT            },
  {C_M_EKF_INIT,                        "C_M_EKF_INIT",                     parse_C_M_EKF_INIT                      },
  {C_M_EKF_START,                       "C_M_EKF_START",                    parse_C_M_EKF_START                     },
  {C_M_EKF_STOP,                        "C_M_EKF_STOP",                     parse_C_M_EKF_STOP                      },
  {C_M_EKF_RELEASE,                     "C_M_EKF_RELEASE",                  parse_C_M_EKF_RELEASE                   },
  {C_M_EKF_SEM_DATA_PUT,                "C_M_EKF_SEM_DATA_PUT",             parse_C_M_EKF_SEM_DATA_PUT              },
  {C_M_SM_REPORT_ORT_FIX,               "C_M_SM_REPORT_ORT_FIX",            parse_C_M_SM_REPORT_ORT_FIX             }
};

/**
 * @brief Configurate IPC parse module output file
 * @param[in]   dir   - output file path
 * @param[in]   fname - output file name
 * @return      None
 */
void ipc_parse_config_output_file(char* drive, char* dir, char* prefix, char* fname)
{
  /* nmea */
  sprintf(gz_IpcParseCtrl.z_pvt_nmea_handler.filepath, "%s%s%s%s.hpp.nmea", drive, dir, prefix, fname);
  sprintf(gz_IpcParseCtrl.z_rtk_nmea_handler.filepath, "%s%s%s%s.rtk.nmea", drive, dir, prefix, fname);
  sprintf(gz_IpcParseCtrl.z_ppp_nmea_handler.filepath, "%s%s%s%s.ppp.nmea", drive, dir, prefix, fname);
  sprintf(gz_IpcParseCtrl.z_dcp_nmea_handler.filepath, "%s%s%s%s.dcp.nmea", drive, dir, prefix, fname);
  sprintf(gz_IpcParseCtrl.z_ins_nmea_handler.filepath, "%s%s%s%s.ins.nmea", drive, dir, prefix, fname);
  sprintf(gz_IpcParseCtrl.z_ins_gpoly_handler.filepath, "%s%s%s%s.gpoly.nmea", drive, dir, prefix, fname);

  /* rover rtcm */
  sprintf(gz_IpcParseCtrl.z_rover_rtcm_handler.filepath, "%s%s%s%s.ROVER_RTCM", drive, dir, prefix, fname);
  /* rover rtcm */
  sprintf(gz_IpcParseCtrl.z_ort_rover_rtcm_handler.filepath, "%s%s%s%s.ORT_ROVER_RTCM", drive, dir, prefix, fname);

  /* base rtcm  */
  sprintf(gz_IpcParseCtrl.z_base_rtcm_handler.filepath, "%s%s%s%s.BASE_RTCM", drive, dir, prefix, fname);

  /* ins BDDB */
  sprintf(gz_IpcParseCtrl.z_ins_bddb_handler.filepath, "%s%s%s%s.bddb.bin", drive, dir, prefix, fname);

  /* orient nmea */
  sprintf(gz_IpcParseCtrl.z_ort_nmea_handler.filepath, "%s%s%s%s.ort.nmea", drive, dir, prefix, fname);
}

/**
 * @brief Release, close file
 * 
*/
void ipc_parse_release()
{
  log_fclose_safe(gz_IpcParseCtrl.z_pvt_nmea_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_ppp_nmea_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_rtk_nmea_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_dcp_nmea_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_ins_nmea_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_ins_gpoly_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_rover_rtcm_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_ort_rover_rtcm_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_base_rtcm_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_ins_bddb_handler.fp);
  log_fclose_safe(gz_IpcParseCtrl.z_ort_nmea_handler.fp);
}

/**
 * @brief Parse a IPC message
 * @param[in]   pz_ipc   - pointer to IPC message
 * @param[out]  parse_output - output parsed string
 * @return      None
 */
uint32_t ipc_parse_execute(ipc_t* pz_ipc, char* parse_output)
{
  uint32_t index = 0;
  char utc_current_str[64] = { 0 };
  uint64_t t_current_sec = pz_ipc->t_timestamp_ms / 1000;
  uint32_t q_current_msec = pz_ipc->t_timestamp_ms % 1000;
  strftime(utc_current_str, sizeof(utc_current_str), "%Y-%m-%d %H:%M:%S", gmtime((time_t*)(&t_current_sec)));

  sprintf(utc_current_str + strlen(utc_current_str), ":%03d", q_current_msec);

  index += sprintf(parse_output + index, "%s Ipc Message Src 0x%04x Dest 0x%04x Size %5d Id 0x%08x %s\n",
    utc_current_str,
    pz_ipc->u_src_id,
    pz_ipc->u_dst_id,
    pz_ipc->q_length + sizeof(ipc_t),
    pz_ipc->q_ipc_id,
    ipctask_get_string_from_id(pz_ipc->q_ipc_id));

  for (uint32_t i = 0; i < sizeof(ipc_parse_functions) / sizeof(ipc_parse_functions[0]); i++)
  {
    if (ipc_parse_functions[i].ipc_id == pz_ipc->q_ipc_id)
    {
      if (NULL != pz_ipc->p_data)
      {
        uint8_t u_version_old = 0;
        uint8_t u_version_new = 0;
        u_version_old = pz_ipc->p_data[0];
        if (ipc_upgrade(pz_ipc))
        {
          uint8_t u_version_new = 0;
          if (NULL != pz_ipc->p_data)
          {
            u_version_new = pz_ipc->p_data[0];
            index += sprintf(parse_output + index, "IPC 0x%08x upgrade From version:%02d To version:%02d\n",
              pz_ipc->q_ipc_id, u_version_old, u_version_new);
          }
        }
      }

      index += ipc_parse_functions[i].func(pz_ipc, parse_output + index);

      if (pz_ipc->u_use_heap)
      {
        OS_FREE(pz_ipc->p_data);
        pz_ipc->u_use_heap = FALSE;
      }

      break;
    }
  }
  parse_output[index] = 0;
  return index;
}


uint32_t parse_LOG_PACKAGE_ID_MONITOR(uint8_t* data, uint32_t length, char* buf)
{
  if (length != sizeof(loc_MonitorStructType))
  {
    return 0;
  }

  uint32_t index = 0;
  loc_MonitorStructType* pz_Monitor = (loc_MonitorStructType*)data;
  index += sprintf(buf + index, "Loc Monitor %s, Runtime:%lld s, Report Count[All %lld hpp:%lld rtk:%lld ppp:%lld]\n", 
    pz_Monitor->build_version,
    pz_Monitor->t_RunningTime,
    pz_Monitor->t_ReportCount,
    pz_Monitor->t_HppReportCount,
    pz_Monitor->t_RtkReportCount,
    pz_Monitor->t_PppReportCount);

  for (int i = 0; i < sizeof(pz_Monitor->z_ConfigPara.para) / sizeof(pz_Monitor->z_ConfigPara.para[0]); i++)
  {
    index += sprintf(buf + index, "Config Mask[%d]: 0x%016x\n", i, 
      (uint32_t)pz_Monitor->z_ConfigPara.para[i].t_mask);
  }

  gu_monitorInfoFlag = 1;
  memcpy(&gz_monitorInfo, pz_Monitor, sizeof(loc_MonitorStructType));
  return index;
}

uint32_t parse_LOG_PACKAGE_ID_LOC_CONFIG(uint8_t* data, uint32_t length, char* buf)
{
  uint32_t index = 0;

  return index;
}


uint32_t parse_LOG_PACKAGE_ID_INS_RESULT(uint8_t* data, uint32_t length, char* buf)
{
  if (length != sizeof(INSResults_t))
  {
    return 0;
  }

  uint32_t index = 0;

#if 1
#define KWS_INIT 0.00863   
  INSResults_t* pz_InsResult = (INSResults_t*)data;

  float f_misb2v_roll_std = 0;
  float f_misb2v_pitch_std = 0;
  float f_misb2v_heading_std = 0;
  sprintf(buf, "$ASINR,%d,%d,%15.9f,%15.9f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
    pz_InsResult->q_tow, (int)pz_InsResult->e_drposflag,
    pz_InsResult->d_latitude * RAD2DEG, pz_InsResult->d_longitude * RAD2DEG, pz_InsResult->f_altitude,
    pz_InsResult->f_vn, pz_InsResult->f_ve, pz_InsResult->f_vd,
    pz_InsResult->f_heading, pz_InsResult->f_pitch, pz_InsResult->f_roll,
    pz_InsResult->f_gyrobias_x, pz_InsResult->f_gyrobias_y, pz_InsResult->f_gyrobias_z,
    pz_InsResult->f_accbias_x, pz_InsResult->f_accbias_y, pz_InsResult->f_accbias_z, pz_InsResult->u_zuptflag, pz_InsResult->f_mis_roll, pz_InsResult->f_mis_pitch, pz_InsResult->f_mis_yaw,
    //pz_InsResult->f_whlspd_sf[2], pz_InsResult->f_whlspd_sf[3],
    KWS_INIT * (1.0 - pz_InsResult->f_whlspd_sf[2]), KWS_INIT * (1.0 - pz_InsResult->f_whlspd_sf[3]),
    pz_InsResult->f_lat_std, pz_InsResult->f_lon_std, pz_InsResult->f_alt_std,
    f_misb2v_roll_std, f_misb2v_pitch_std, f_misb2v_heading_std,
    pz_InsResult->q_kfmeastype);
  log_parse_print_gpimu_gpoly_nmea(buf);
  memset(buf, 0, sizeof(buf));
  uint32_t len = ag_imu_bddb_encode_0x0B(pz_InsResult, (int8_t*)buf);
  log_parse_print_ins_bddb(buf, len);

#if 1  //OPEN_INS_INTEGRITY
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "$ASITG,%llu,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
    pz_InsResult->z_inspl.t_timestamp, (int)pz_InsResult->z_inspl.e_drposflag,
    pz_InsResult->z_inspl.f_pos_longitudinal_pl, pz_InsResult->z_inspl.f_pos_lateral_pl,
    pz_InsResult->z_inspl.f_pos_hpl, pz_InsResult->z_inspl.f_pos_vpl,
    pz_InsResult->z_inspl.f_pos_north_pl, pz_InsResult->z_inspl.f_pos_east_pl,
    pz_InsResult->z_inspl.f_vel_longitudinal_pl, pz_InsResult->z_inspl.f_vel_lateral_pl,
    pz_InsResult->z_inspl.f_vel_hpl, pz_InsResult->z_inspl.f_vel_vpl,
    pz_InsResult->z_inspl.f_vel_north_pl, pz_InsResult->z_inspl.f_vel_east_pl,
    pz_InsResult->z_inspl.f_roll_pl, pz_InsResult->z_inspl.f_pitch_pl, pz_InsResult->z_inspl.f_yaw_pl);
  log_parse_print_gpimu_gpoly_nmea(buf);
  memset(buf, 0, sizeof(buf));
  uint32_t len2 = ag_imu_bddb_encode_0x0D(pz_InsResult, (int8_t*)buf);
  log_parse_print_ins_bddb(buf, len2);
#endif

#endif

  return index;
}

static struct {
  uint32_t report_id;
  const char* report_name;
  report_parse_func func;
} report_message_parse_functions[] = {
  {LOG_PACKAGE_ID_START,                   "LOG_PACKAGE_ID_START",           NULL                           },
  {LOG_PACKAGE_ID_MONITOR,                 "LOG_PACKAGE_ID_MONITOR",         parse_LOG_PACKAGE_ID_MONITOR   },
  {LOG_PACKAGE_ID_LOC_CONFIG,              "LOG_PACKAGE_ID_LOC_CONFIG",      parse_LOG_PACKAGE_ID_LOC_CONFIG},
  {LOG_PACKAGE_ID_INS_RESULT,              "LOG_PACKAGE_ID_INS_RESULT",      parse_LOG_PACKAGE_ID_INS_RESULT},
  {LOG_PACKAGE_ID_END,                     "LOG_PACKAGE_ID_END",             NULL                           }
};

/**
 * @brief Parse a package message
 *
 *    Package format
 * | 1Byte 1Byte | 2Byte    4Byte |  2Byte | Package data length | 2Byte   |
 * | SYNC0 SYNC1 | LOG_TYPE LENGTH| Pkg ID | Package data        | CHECKSUM|
 * |             | ---------Check Sum Compute data-------------- |         |
 * 
 * @param[in]   id      - IPC id
 * @param[in]   data    - IPC data buffer
 * @param[in]   length  - IPC data length
 * @param[out]  parse_output - output parsed string
 * @return      None
 */
uint32_t package_parse_execute(uint8_t* p_data, uint32_t length, char* parse_output)
{
  BOOL     u_upgraded = FALSE;
  uint32_t index = 0;
  uint16_t w_PackageId = LOG_PACKAGE_ID_START;

  memcpy(&w_PackageId, p_data, sizeof(uint16_t));
  p_data += sizeof(uint16_t);
  length -= sizeof(uint16_t);

  if (!(w_PackageId > LOG_PACKAGE_ID_START && w_PackageId < LOG_PACKAGE_ID_END))
  {
    return 0;
  }

  index += sprintf(parse_output, "Package report Id 0x%08x %-40s\n",
    w_PackageId,
    loc_report_get_string_from_id(w_PackageId));

  for (uint32_t i = 0; i < sizeof(report_message_parse_functions) / sizeof(report_message_parse_functions[0]); i++)
  {
    if (report_message_parse_functions[i].report_id == w_PackageId)
    {
      u_upgraded = package_upgrade(w_PackageId, &p_data, &length);
      index += report_message_parse_functions[i].func(p_data, length, parse_output + index);
      if (u_upgraded)
      {
        OS_FREE(p_data);
      }
      break;
    }
  }
  return index;
}