/**@file        playback.cpp
 * @brief       Playback source file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/24  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <thread>
#include <string>
#include <list>
#include <algorithm>

#if defined(_MSC_VER)
#include <direct.h>
#include <io.h>
#elif defined(__gnu_linux__)
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

#ifdef FEATURE_PROFILER
#include "../../3rdparty/gperftools/include/profiler.h"
#endif

#if defined(APP_HEXAGON_SSR) // APP_HEXAGON_SSR
#include "correctionsclient.h"
#include "tsx_adapter_integration.h"
#elif defined(APP_SWIFT_SSR) // APP_SWIFT_SSR
#include "./adapter/swift_core_ssr_api.h"
#endif 

#include "cmn_utils.h"
#include "ag_log_parse.h"
#include "ag_bddb_parse.h"
#include "cmn_highPrecision.h"
#include "mw_log.h"
#include "mw_ipctask.h"
#include "mw_alloc.h"
#include "sm_api.h"
#include "sm_nmea.h"
#include "hpp_api.h"
#include "dcp_api.h"
#include "ppp_api.h"
#include "sd_api.h"
#include "vdr_api.h"
#include "loc_core_api.h"
#include "loc_core_report.h"
#include "gnss_common.h"
#include "ipc_parse.h"
#include "rtcm_dec.h"
#include "ubx_dec.h"
#include "ipc_upgrade.h"
#include "vdr_task.h"
#ifdef _WIN32
#include <io.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif


#include "rinex_parse.h"

/* 3rdparty */
#include "OptionParser.h"
#include "MemoryStatistics.h"
#include "ini.h"
#ifdef INS_DEBUG
#include "fusion_reference_debug.h"
#endif

using namespace std;

#define READ_BUFFER_SIZE             (64 * 1024)
#define READ_BUFFER_RTCM_SIZE        (1 * 1024 * 1024)
#define PB_FOR_SAMPLE_REPORT_POS_NUM (20)

/* Playback Options Definition ----------------------------------------------*/

/* Playback ouput file format */
typedef uint8_t pb_OutputFileType;
#define PB_OUTPUT_FILE_CONTROL_BIN               (uint8_t)0x01 /* bin file is valid */
#define PB_OUTPUT_FILE_CONTROL_TEXT              (uint8_t)0x02 /* txt file is valid */

/* Playback input file format */
typedef enum {
  PB_INPUT_FILE_ALE = 0,
  PB_INPUT_FILE_RTCM,
  PB_INPUT_FILE_RINEX,
  PB_INPUT_FILE_UBLOX,
  PB_INPUT_FILE_BDDB,
  PB_INPUT_FILE_MAX
} pb_InputFileTypeVal;
typedef uint8_t pb_InputFileType;

/* Playback input truth type format */
typedef enum {
  PB_TRUTH_MODE_DISABLE = 0,
  PB_TRUTH_MODE_STATIC,
  PB_TRUTH_MODE_DYNAMIC_NMEA,
  PB_TRUTH_MODE_DYNAMIC_POS320,
  PB_TRUTH_MODE_MAX,
} PB_TruthModeEnumTypeVal;
typedef uint8_t PB_TruthModeEnumType;

/* Playback config setting structure */
typedef struct {
  pb_InputFileType      pb_input_file_type;
  char                  pb_input_file[256];
  float                 pb_meas_update_interval;
  float                 pb_prefix_diffage;
  const char*           pb_file_format;
  FILE*                 pb_file_fp;
} pb_config_setting_t;

/* Playback config output structure */
typedef struct {
  LogLevelEnum          pb_log_level;
  char                  pb_output_dir[256];
  char                  pb_log_prefix[256];
  uint8_t               pb_log_suffix_by_time;
  uint8_t               pb_log_progressbar_disable;
  uint8_t               pb_output_file_set_valid;
  pb_OutputFileType     pb_output_file_mask;
  char                  pb_output_ag_path[256];
  char                  pb_output_text_path[256];
  FILE*                 pb_output_ag_fp;
  FILE*                 pb_output_text_fp;
} pb_config_output_t;

/* Playback config truth structure */
typedef struct {
  PB_TruthModeEnumType  pb_truth_mode;
  double                pb_truth_static_ecef_x;
  double                pb_truth_static_ecef_y;
  double                pb_truth_static_ecef_z;
  char                  pb_truth_dynamic_file[256];
} pb_config_truth_t;

/* Playback config assistance structure */
#define PB_ASSIST_REF_RTCM_VALID    ((uint32_t)0x00000001)
#define PB_ASSIST_REF_RINEX_VALID   ((uint32_t)0x00000002)
#define PB_ASSIST_NAV_RTCM_VALID    ((uint32_t)0x00000004)
#define PB_ASSIST_NAV_RINEX_VALID   ((uint32_t)0x00000008)
#define PB_ASSIST_SSR_BNC_VALID     ((uint32_t)0x00000010)
#define PB_ASSIST_IMU_AG_BDDB_VALID ((uint32_t)0x00000020)
#define PB_ASSIST_ORT_RTCM_VALID    ((uint32_t)0x00000040)
#define PB_ASSIST_TSX_DCF           ((uint32_t)0x00000080)

typedef struct {
  uint32_t              pb_assist_file_valid_mask;
  char                  pb_assist_ref_rtcm[256];
  char                  pb_assist_ref_rinex[256];
  char                  pb_assist_nav_rtcm[256];
  char                  pb_assist_nav_rinex[256];
  char                  pb_assist_ssr_bnc[256];
  char                  pb_assist_imu_ag_bddb[256];
  char                  pb_assist_ort_rtcm[256];
  char                  pb_assist_tsx_dcf[256];
} pb_config_assist_t;

typedef struct {
  uint8_t u_Enable;
  char    p_DataLogFilepath[DLOG_MAX][256];
} pb_config_datalog_t;

/** Configurate parameter definition */
typedef struct {
  uint8_t  u_field[10];
  float    f_field[10];
} pb_config_para_t;

/* Playback loc core config structure */
typedef struct {
  /* sm module config */
  uint8_t pb_loc_core_sm_task_enable;

  /* hpp module config */
  uint8_t pb_loc_core_hpp_task_enable;
  uint8_t pb_loc_core_hpp_pvt_enable;
  uint8_t pb_loc_core_hpp_rtk_enable;

  /* dcp module config */
  uint8_t pb_loc_core_dcp_task_enable;
  
  /* ppp module config */
  uint8_t pb_loc_core_ppp_task_enable;
  
  /* sd module config */
  uint8_t pb_loc_core_sd_task_enable;
  
  /* vdr module config */
  uint8_t pb_loc_core_vdr_task_enable;

  /* log module config */
  uint8_t pb_loc_core_log_task_enable;

  /* ort module config */
  uint8_t pb_loc_core_ort_task_enable;

  pb_config_para_t para[8];
} pb_config_core_t;

/* Playback config ipc filter structure */
typedef struct {
  uint32_t              pb_filter_pass_log_ipc[16];
  uint32_t              pb_filter_pass_live_ipc[16];
} pb_config_filter_t;

typedef struct {
  pb_config_setting_t setting;
  pb_config_output_t  output;
  pb_config_truth_t   truth;
  pb_config_assist_t  assist;
  pb_config_datalog_t datalog;
  pb_config_core_t    core;
  pb_config_filter_t  filter;
} pb_config_t;

typedef struct
{
  uint32_t q_towMsecList[PB_FOR_SAMPLE_REPORT_POS_NUM];
  uint8_t u_tnum;
  uint16_t w_posfix_vailed_num;
} pb_interval_t;

pb_config_t gz_pb_config = { 0 };
pb_config_t gz_pb_config_log = { 0 };
pb_interval_t gz_pb_interval = {0};

uint64_t pb_get_file_size(const char *filename)
{
#ifdef _WIN32
  struct _stat64 file_stat;
  if (_stat64(filename, &file_stat) == 0)
  {
    return file_stat.st_size;
  }
#else
  struct stat64 file_stat;
  if (stat64(filename, &file_stat) == 0)
  {
    return file_stat.st_size;
  }
#endif
  printf("get file size error %s\n", filename);
  return 0;
}

const char* pb_convert_file_format_to_str(uint8_t pb_input_file_type)
{
  switch (gz_pb_config.setting.pb_input_file_type)
  {
  case 0: 
    return "AG";
  case 1: 
    return "RTCM";
  case 2: 
    return "RINEX";
  case 3:
    return "UBLOX";
  default: return "Error";
  }
  return "Error";
}

static int handler(void* user, const char* section, const char* name,
  const char* value)
{
  pb_config_t* pconfig = (pb_config_t*)user;

#define MATCH(s, n) (strcmp(section, s) == 0 && strcmp(name, n) == 0)

  if (MATCH("Setting", "pb_input_file_type")) {
    pconfig->setting.pb_input_file_type = atoi(value);
    pconfig->setting.pb_file_format = pb_convert_file_format_to_str(pconfig->setting.pb_input_file_type);
  }
  else if (MATCH("Setting", "pb_input_file")) {
    strcpy(pconfig->setting.pb_input_file, value);
  }
  else if (MATCH("Setting", "pb_meas_update_interval")) {
    pconfig->setting.pb_meas_update_interval = (float)(atof(value));
  }
  else if (MATCH("Setting", "pb_prefix_diffage")) {
    pconfig->setting.pb_prefix_diffage = (float)(atof(value));
  }
  else if (MATCH("Output", "pb_output_dir")) {
    strcpy(pconfig->output.pb_output_dir, value);
    if (strlen(value) == 0) {
      strcpy(pconfig->output.pb_output_dir, "./pb_output/");
    }
    else {
      memset(gz_pb_config.output.pb_output_dir, 0, sizeof(gz_pb_config.output.pb_output_dir));
      strcpy(pconfig->output.pb_output_dir, value);
    }
  }
  else if (MATCH("DataLog", "pb_datalog_enable")) {
    pconfig->datalog.u_Enable = atoi(value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_SM")) {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_SM], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_HPP")) {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_HPP], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_DCP")) {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_DCP], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_PPP")) {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_PPP], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_SD")) {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_SD], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_VDR")) {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_VDR], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_SAT_POS")){
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_SAT_POS], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_GNSS_PL"))
  {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_GNSS_PL], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_GNSS_ML_PL"))
  {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_GNSS_ML_PL], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_GNSS_ML_SCENE"))
  {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_GNSS_ML_SCENE], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_GNSS_SCENE"))
  {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_GNSS_SCENE], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_GNSS_STD"))
  {
    strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_GNSS_STD], value);
  }
  else if (MATCH("DataLog", "pb_datalog_DLOG_VDR_BDDB"))
  {
      strcpy(pconfig->datalog.p_DataLogFilepath[DLOG_VDR_BDDB], value);
  }
  else if (MATCH("Output", "pb_output_file_mask")) {
    pconfig->output.pb_output_file_set_valid = 1;
    pconfig->output.pb_output_file_mask = stoi(value, 0, 16);
  }
  else if (MATCH("Output", "pb_log_level")) {
    pconfig->output.pb_log_level = atoi(value);
  }
  else if (MATCH("Output", "pb_log_prefix")) {
    if (strlen(value) == 0) {
      strcpy(pconfig->output.pb_log_prefix, "pb-log-");
    }
    else {
      strcpy(pconfig->output.pb_log_prefix, value);
    }
  }
  else if (MATCH("Output", "pb_log_suffix_by_time")) {
    pconfig->output.pb_log_suffix_by_time = atoi(value);
  }
  else if (MATCH("Output", "pb_log_enable_progressbar")) {
    pconfig->output.pb_log_progressbar_disable = atoi(value);
  }
  else if (MATCH("Truth", "truth_mode")) {
    pconfig->truth.pb_truth_mode = atoi(value);
  }
  else if (MATCH("Truth", "truth_static_ecef_x")) {
    pconfig->truth.pb_truth_static_ecef_x = atof(value);
  }
  else if (MATCH("Truth", "truth_static_ecef_y")) {
    pconfig->truth.pb_truth_static_ecef_y = atof(value);
  }
  else if (MATCH("Truth", "truth_static_ecef_z")) {
    pconfig->truth.pb_truth_static_ecef_z = atof(value);
  }
  else if (MATCH("Truth", "truth_dynamic_file")) {
    strcpy(pconfig->truth.pb_truth_dynamic_file, value);
  }
  else if (MATCH("Assist", "assist_file_valid_mask")) {
    pconfig->assist.pb_assist_file_valid_mask = stoi(value, 0, 16);
  }
  else if (MATCH("Assist", "assist_ref_rtcm")) {
    strcpy(pconfig->assist.pb_assist_ref_rtcm, value);
  }
  else if (MATCH("Assist", "assist_ref_rinex")) {
    strcpy(pconfig->assist.pb_assist_ref_rinex, value);
  }
  else if (MATCH("Assist", "assist_nav_rtcm")) {
    strcpy(pconfig->assist.pb_assist_nav_rtcm, value);
  }
  else if (MATCH("Assist", "assist_nav_rinex")) {
    strcpy(pconfig->assist.pb_assist_nav_rinex, value);
  }
  else if (MATCH("Assist", "assist_ssr_bnc")) {
    strcpy(pconfig->assist.pb_assist_ssr_bnc, value);
  }
  else if (MATCH("Assist", "assist_imu_ag_bddb")) {
    strcpy(pconfig->assist.pb_assist_imu_ag_bddb, value);
  }
  else if (MATCH("Assist", "assist_ort_rtcm"))
  {
    strcpy(pconfig->assist.pb_assist_ort_rtcm, value);
  }
  else if (MATCH("Assist", "assist_tsx_dcf"))
  {
    strcpy(pconfig->assist.pb_assist_tsx_dcf, value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_sm_task_enable")) {
    pconfig->core.pb_loc_core_sm_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_hpp_task_enable")) {
    pconfig->core.pb_loc_core_hpp_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_hpp_pvt_enable")) {
    pconfig->core.pb_loc_core_hpp_pvt_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_hpp_rtk_enable")) {
    pconfig->core.pb_loc_core_hpp_rtk_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_dcp_task_enable")) {
    pconfig->core.pb_loc_core_dcp_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_ppp_task_enable")) {
    pconfig->core.pb_loc_core_ppp_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_sd_task_enable")) {
    pconfig->core.pb_loc_core_sd_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_vdr_task_enable")) {
    pconfig->core.pb_loc_core_vdr_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_log_task_enable")) {
    pconfig->core.pb_loc_core_log_task_enable = atoi(value);
  }
  else if (MATCH("CoreConfig", "pb_loc_core_cfg_ort_task_enable"))
  {
    pconfig->core.pb_loc_core_ort_task_enable = atoi(value);
  }
  else 
  {
    char feild_str[32] = { 0 };
    char vdr_int[10][20] = { "gnss_rate","imu_sampling_rate","whspd_mode","whspd_rate","vdr_mode","align_mode","outputpos_flag",
                             "output_rate","max_drtime","imu_type"};
    char vdr_float[10][20]={ "imu2gnss_x", "imu2gnss_y", "imu2gnss_z", "imu2rearmid_x", "imu2rearmid_y", "imu2rearmid_z",
                            "rear2rear", "mis_b2g_roll","mis_b2g_pitch","mis_b2g_heading" };

    for (int i = 0; i < M_ARRAY_SIZE(gz_pb_config.core.para); i++)
    {
      for (int j = 0; j < M_ARRAY_SIZE(gz_pb_config.core.para[i].u_field); j++)
      {
        if(5 == i){
            sprintf(feild_str, "int_val_%d_%d_%s", i,j, vdr_int[j]);
        } else {
            sprintf(feild_str, "int_val_%d_%d", i,j);
        }
        if (MATCH("CoreConfig", feild_str))
        {
            gz_pb_config.core.para[i].u_field[j] = (uint8_t)(strtol(value, NULL, 0));
            return 1;
        }
      }
    }
    for (int i = 0; i < M_ARRAY_SIZE(gz_pb_config.core.para); i++)
    {
        for (int j = 0; j < M_ARRAY_SIZE(gz_pb_config.core.para[i].f_field); j++)
        {
			if(5 == i){
				sprintf(feild_str, "float_val_%d_%d_%s",i, j, vdr_float[j]);

			} else {
				sprintf(feild_str, "float_val_%d_%d", i,j);
			}
            if (MATCH("CoreConfig", feild_str))
            {
                gz_pb_config.core.para[i].f_field[j] = strtof(value, NULL);
                return 1;
            }
        }
    }
    uint8_t u_count = M_ARRAY_SIZE(pconfig->filter.pb_filter_pass_log_ipc);
    for (uint8_t i = 0; i < u_count; i++)
    {
      char str[256] = { 0 };
      sprintf(str, "pb_filter_pass_log_ipc_%02d", i);
      if (MATCH("Filter", str))
      {
        pconfig->filter.pb_filter_pass_log_ipc[i] = ipctask_get_id_from_string(value);
        return 1;
      }
    }

    u_count = M_ARRAY_SIZE(pconfig->filter.pb_filter_pass_live_ipc);
    for (uint8_t i = 0; i < u_count; i++)
    {
      char str[256] = { 0 };
      sprintf(str, "pb_filter_pass_live_ipc_%02d", i);
      if (MATCH("Filter", str))
      {
        pconfig->filter.pb_filter_pass_live_ipc[i] = ipctask_get_id_from_string(value);
        return 1;
      }
    }
    return 0;
  }

  return 1;
}

typedef enum {
  PB_LOCATION_DATA_INVALID = 0,
  PB_LOCATION_DATA_EPH,
  PB_LOCATION_DATA_BDDB_IMU,
  PB_LOCATION_DATA_BDDB_GNSS,
  PB_LOCATION_DATA_BDDB_WHEEL,
  PB_LOCATION_DATA_REF_CORR,
  PB_LOCATION_DATA_ORT_MEAS,
  PB_LOCATION_DATA_RCV_MEAS,
  PB_LOCATION_DATA_RCV_INSRES,
} pb_LocationDataEnumTypeVal;
typedef uint8_t pb_LocationDataEnumType;

class PlaybackDataUnit
{
public:
  PlaybackDataUnit() {
    t_timestamp = 0;
    t_tow_ms = 0;
    u_fake_week = 0;
    u_type = PB_LOCATION_DATA_INVALID;
    pz_RcvMeasBlock = NULL;
    pz_OrtMeasBlock = NULL;
    pz_CorrBlock = NULL;
    pz_ImuData = NULL;
    pz_Ephemeris = NULL;
    pz_GnssTypeBDDB10 = NULL;
    pz_WheelData = NULL;
    pz_InsData = NULL;
  };

  ~PlaybackDataUnit() {};

  uint64_t t_timestamp;
  uint64_t t_tow_ms;
  uint8_t u_fake_week;
  /* 0: Invalid, 1: Meas, 2: Corr, 3: Imu*/
  pb_LocationDataEnumType  u_type;
  GnssMeasBlock_t*         pz_RcvMeasBlock;
  GnssMeasBlock_t*         pz_OrtMeasBlock;
  GnssCorrBlock_t*         pz_CorrBlock;
  loc_api_imu_data_t*      pz_ImuData;
  gnss_Ephemeris_t*        pz_Ephemeris;
  GnssFixType*             pz_GnssTypeBDDB10;
  loc_api_wheel_t*         pz_WheelData;
  INSResults_t*            pz_InsData;
};

static std::list<PlaybackDataUnit> glt_PlaybackRawInjectData_Database;

struct FindByConsSvid {
    uint8_t u_constelltion;
    uint8_t u_svid;
    
    FindByConsSvid(uint8_t u_constelltion, uint8_t u_svid) : u_constelltion(u_constelltion), u_svid(u_svid){}

    bool operator()(const PlaybackDataUnit& obj) const {
      uint8_t u_svid_iner = 0;
      if(obj.u_type == PB_LOCATION_DATA_EPH)
      {
        switch (obj.pz_Ephemeris->u_constellation)
        {
        case C_GNSS_GPS:
        case C_GNSS_QZS:
          u_svid_iner = obj.pz_Ephemeris->eph.z_gpsEph.svid;
          break;
        case C_GNSS_GAL:
          u_svid_iner = obj.pz_Ephemeris->eph.z_galEph.svid;
          break;
        case C_GNSS_BDS2:
        case C_GNSS_BDS3:
          u_svid_iner = obj.pz_Ephemeris->eph.z_bdsEph.svid;
          break;
        case C_GNSS_GLO:
          break;
        default:
          break;
        }
      }
      if(u_svid_iner !=0 &&
        u_constelltion == obj.pz_Ephemeris->u_constellation && 
        u_svid_iner == u_svid)
      {
        return true;
      }
      return false;
    }
};

static const gnss_Ephemeris_t* PlaybackDataUnit_find_latest_eph(uint8_t u_constelltion, uint8_t u_svid)
{
  auto it = std::find_if(glt_PlaybackRawInjectData_Database.rbegin(), glt_PlaybackRawInjectData_Database.rend(),
                         FindByConsSvid(u_constelltion, u_svid));

  if (it != glt_PlaybackRawInjectData_Database.rend())
  {
    return it->pz_Ephemeris;
  }
  else
  {
   return NULL;
  }
}

static void free_PlaybackDataUnit(std::list<PlaybackDataUnit>::iterator ite)
{
  safe_free_p(ite->pz_RcvMeasBlock);
  safe_free_p(ite->pz_OrtMeasBlock);
  safe_free_p(ite->pz_CorrBlock);
  safe_free_p(ite->pz_ImuData);
  safe_free_p(ite->pz_Ephemeris);
  safe_free_p(ite->pz_GnssTypeBDDB10);
  safe_free_p(ite->pz_WheelData);
}

typedef struct
{
  FILE* fp_in_file;
  FILE* fp_ref_rtcm;
  FILE* fp_ref_rinex;
  FILE* fp_nav_rinex;
  FILE* fp_ssr_bnc;
  FILE* fp_imu_bddb;
  FILE* fp_ort_rtcm;

  uint64_t unm_rtcm_back;
  uint64_t num_rtcm;
  uint64_t num_ref_rtcm;
  uint64_t num_ref_rinex;
  uint64_t num_nav_rinex;
  uint64_t num_ssrbnc;
  uint64_t num_imu_bddb;
  uint64_t num_ort_rtcm;

  uint64_t q_rtcm_towMsec;
  uint64_t q_ort_rtcm_towMsec;
  uint64_t q_ref_rtcm_towMsec;
  uint8_t u_fake_week_rtcm;
  uint8_t u_fake_week_ort_rtcm;
  uint8_t u_fake_week_ref_rtcm;
}assist_file_states_t;

/******************************************************************************
                              RTCM database load
******************************************************************************/
/**
 * @brief playback assist module, load rover block using RTCM format
 * @param[in] filename - input filename
 * @return None
 */
static uint64_t pb_assist_decode_rtcm(RtcmDecoder_t *pz_RtcmDecoder, const uint8_t *buff, uint64_t length,
                                       pb_LocationDataEnumType type, assist_file_states_t* assist_file)
{
  int8_t ret = 0;
  uint64_t n_meas = 0;

  // Combined data of previous stored and ipc
  uint32_t length_ava = 0;
  uint8_t* buffer_ava = (uint8_t*)malloc(sizeof(uint8_t) * (uint32_t)((pz_RtcmDecoder->e2emsglen + length + 1)));

  if (!rtcmE2eSkip(pz_RtcmDecoder, buff, (uint32_t)length, buffer_ava, &length_ava))
  {
	  length_ava = 0;
  }

  for (uint32_t i = 0; i < length_ava; i++)
  {
    ret = rtcm_dec_input(pz_RtcmDecoder, buffer_ava[i]);
    if (ret <= 0)
    {
      continue;
    }

    if (1 == ret)
    {
      if (pz_RtcmDecoder->is_complete)
      {

        PlaybackDataUnit pbDataUint;
        pbDataUint.u_type = type;

        if (PB_LOCATION_DATA_RCV_MEAS == type || PB_LOCATION_DATA_ORT_MEAS == type)
        {
          GnssMeasBlock_t z_GnssMeasBlock = {0};
          rtcm_ExtractMeasBlk(pz_RtcmDecoder, &z_GnssMeasBlock);
          pbDataUint.t_tow_ms = z_GnssMeasBlock.z_Clock.z_gpsTime.q_towMsec;
          if(PB_LOCATION_DATA_RCV_MEAS == type)
          {
            pbDataUint.pz_RcvMeasBlock = (GnssMeasBlock_t*)malloc(sizeof(GnssMeasBlock_t));
            if (NULL == pbDataUint.pz_RcvMeasBlock)
            {
              continue;
            }
            if ((assist_file->q_rtcm_towMsec != 0) &&
                ((double)assist_file->q_rtcm_towMsec - (double)pbDataUint.t_tow_ms) > (double) WEEK_MSEC / 2)
            {
              assist_file->u_fake_week_rtcm++;
            }
            pbDataUint.u_fake_week = assist_file->u_fake_week_rtcm;
            assist_file->q_rtcm_towMsec = pbDataUint.t_tow_ms;
            memcpy(pbDataUint.pz_RcvMeasBlock, &z_GnssMeasBlock, sizeof(GnssMeasBlock_t));
          }
          else if(PB_LOCATION_DATA_ORT_MEAS == type)
          {
            pbDataUint.pz_OrtMeasBlock = (GnssMeasBlock_t*)malloc(sizeof(GnssMeasBlock_t));
            if (NULL == pbDataUint.pz_OrtMeasBlock)
            {
              continue;
            }
            if ((assist_file->q_ort_rtcm_towMsec != 0) &&
                ((double)assist_file->q_ort_rtcm_towMsec - (double)pbDataUint.t_tow_ms) > (double) WEEK_MSEC / 2)
            {
              assist_file->u_fake_week_ort_rtcm++;
            }
            pbDataUint.u_fake_week = assist_file->u_fake_week_ort_rtcm;
            assist_file->q_ort_rtcm_towMsec = pbDataUint.t_tow_ms;
            memcpy(pbDataUint.pz_OrtMeasBlock, &z_GnssMeasBlock, sizeof(GnssMeasBlock_t));
          }
          glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
          ++n_meas;
        }
        else if (PB_LOCATION_DATA_REF_CORR == type)
        {
          GnssCorrBlock_t z_CorrectBlock = {0};
          rtcm_ExtractCorrBlk(pz_RtcmDecoder, &z_CorrectBlock);
          pbDataUint.t_tow_ms = z_CorrectBlock.z_Clock.z_gpsTime.q_towMsec;

          pbDataUint.pz_CorrBlock = (GnssCorrBlock_t *)malloc(sizeof(GnssCorrBlock_t));
          if (NULL == pbDataUint.pz_CorrBlock)
          {
            continue;
          }
          if ((assist_file->q_ref_rtcm_towMsec != 0) &&
              ((double)assist_file->q_ref_rtcm_towMsec - (double)pbDataUint.t_tow_ms) > (double) WEEK_MSEC / 2)
          {
            assist_file->u_fake_week_ref_rtcm++;
          }
          pbDataUint.u_fake_week = assist_file->u_fake_week_ref_rtcm;
          assist_file->q_ref_rtcm_towMsec = pbDataUint.t_tow_ms;
          memcpy(pbDataUint.pz_CorrBlock, &z_CorrectBlock, sizeof(GnssCorrBlock_t));
          glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
          ++n_meas;
        }
      }
    }
    else if (2 == ret && !M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_NAV_RINEX_VALID))
    {
      PlaybackDataUnit pbDataUint;
      pbDataUint.u_type = PB_LOCATION_DATA_EPH;
      pbDataUint.t_tow_ms = assist_file->q_rtcm_towMsec;
      pbDataUint.u_fake_week = assist_file->u_fake_week_rtcm;
      if (((double)assist_file->q_rtcm_towMsec - (double)pbDataUint.t_tow_ms) > WEEK_MSEC / 2.0)
      {
        pbDataUint.u_fake_week++;
      }

      pbDataUint.pz_Ephemeris = (gnss_Ephemeris_t*)malloc(sizeof(gnss_Ephemeris_t));
      if (NULL == pbDataUint.pz_Ephemeris)
      {
        continue;
      }
      memcpy(pbDataUint.pz_Ephemeris, &pz_RtcmDecoder->ephemeris, sizeof(gnss_Ephemeris_t));
      glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
    }
  }
	free(buffer_ava);
  return n_meas;
}

/******************************************************************************
           UBLOX database 
******************************************************************************/
static uint64_t pb_assist_load_ublox_stream(UbloxDecoder_t* pz_UbloxDecoder, const uint8_t *buff, uint64_t length, assist_file_states_t* assist_file)
{
  int8_t ret = 0;
  uint64_t n_meas = 0;

  for (uint32_t i = 0; i < length; i++)
  {
    ret = ublox_dec_input(pz_UbloxDecoder, buff[i]);
    if (ret <= 0)
    {
      continue;
    }
    if (ret == 1)
    {
      GnssMeasBlock_t z_GnssMeasBlock = { 0 };
      ublox_ExtractMeasBlk(pz_UbloxDecoder, &z_GnssMeasBlock);

      PlaybackDataUnit pbDataUint;
      pbDataUint.u_type = PB_LOCATION_DATA_RCV_MEAS;
      pbDataUint.t_tow_ms = pz_UbloxDecoder->q_towMsec;
      pbDataUint.pz_RcvMeasBlock = (GnssMeasBlock_t*)malloc(sizeof(GnssMeasBlock_t));
      if (NULL == pbDataUint.pz_RcvMeasBlock)
      {
        continue;
      }
      if ((assist_file->q_rtcm_towMsec != 0) &&
          ((double)assist_file->q_rtcm_towMsec - (double)pbDataUint.t_tow_ms) > (double) WEEK_MSEC / 2)
      {
        assist_file->u_fake_week_rtcm++;
      }
      pbDataUint.u_fake_week = assist_file->u_fake_week_rtcm;
      memcpy(pbDataUint.pz_RcvMeasBlock, &z_GnssMeasBlock, sizeof(GnssMeasBlock_t));
      glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
      ++n_meas;
    }
    else if (2 == ret && !M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_NAV_RINEX_VALID))
    {
      gnss_Ephemeris_t z_Ephemeris = { 0 };
      ublox_ExtractEph(pz_UbloxDecoder, &z_Ephemeris);

      PlaybackDataUnit pbDataUint;
      pbDataUint.u_type = PB_LOCATION_DATA_EPH;
      pbDataUint.t_tow_ms = pz_UbloxDecoder->q_towMsec;
      pbDataUint.u_fake_week = assist_file->u_fake_week_rtcm;
      if (((double)assist_file->q_rtcm_towMsec - (double)pbDataUint.t_tow_ms) > WEEK_MSEC / 2.0)
      {
        pbDataUint.u_fake_week++;
      }
      pbDataUint.pz_Ephemeris = (gnss_Ephemeris_t*)malloc(sizeof(gnss_Ephemeris_t));
      if (NULL == pbDataUint.pz_Ephemeris)
      {
        continue;
      }
      memcpy(pbDataUint.pz_Ephemeris, &z_Ephemeris, sizeof(gnss_Ephemeris_t));
      glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
    } 
  }
  return n_meas;
}

/**
 * @brief sequence glt_PlaybackRawInjectData_Database by tow time
 * @return None
 */
void pb_database_sequence()
{
  glt_PlaybackRawInjectData_Database.sort(
      [](PlaybackDataUnit &a, PlaybackDataUnit &b)
      {
        if (a.u_fake_week < b.u_fake_week)
        {
          return true;
        }
        else if (a.u_fake_week == b.u_fake_week)
        {
          if (a.t_tow_ms < b.t_tow_ms)
          {
            return true;
          }
          else if (a.t_tow_ms == b.t_tow_ms)
          {
            if (a.u_type < b.u_type)
            {
              return true;
            }
          }
        }
        return false;
      });
}

/******************************************************************************
           Reference Station RINEX database
******************************************************************************/
/**
 * @brief playback assist module, load assist correct block using RINEX format
 * @param[in] filename - input filename
 * @return None
 */
static void pb_assist_load_ref_rinex(const char* filename)
{
  int8_t s_ret = -1;
  char pu_buff[4096] = "";
  char pu_obsCode[C_GNSS_MAX][32][4] = { {""} };
  uint8_t pu_obsCodeNum[C_GNSS_MAX] = { 0 };
  FILE* fp_ref_rinex = fopen(filename, "r");
  if (NULL == fp_ref_rinex)
  {
    printf("Open File %s Fail, exit...\n", filename);
    return;
  }
  printf("Load... ref rinex file %s\n", filename);

  RinexDecoder_t* pz_RinexDecoder = (RinexDecoder_t*)malloc(sizeof(RinexDecoder_t));

  if (NULL == pz_RinexDecoder)
  {
    printf("Alloc RinexDecoder Size=%d Fail, exit...\n", sizeof(RtcmDecoder_t));
    return;
  }

  memset(pz_RinexDecoder, 0, sizeof(RinexDecoder_t));
  while (fgets(pu_buff, sizeof(pu_buff), fp_ref_rinex))
  {
    if (s_ret == -1)
    {
      s_ret = rinex_dec_obsf(pz_RinexDecoder, pu_buff, pu_obsCode, pu_obsCodeNum);
    }
    if (s_ret != -1 && !strstr(pu_buff, "HEADER"))
    {
      s_ret = rinex_dec_obsb(pz_RinexDecoder, pu_buff, pu_obsCode, pu_obsCodeNum);
      if (s_ret == 1 && pz_RinexDecoder->nobs != 0)
      {
        GnssCorrBlock_t z_CorrectBlock = { 0 };
        rinex_ExtractCorrBlk(pz_RinexDecoder, &z_CorrectBlock);

        PlaybackDataUnit pbDataUint;
        pbDataUint.u_type = PB_LOCATION_DATA_REF_CORR;
        pbDataUint.t_tow_ms = z_CorrectBlock.z_Clock.z_gpsTime.q_towMsec;
        pbDataUint.pz_CorrBlock = (GnssCorrBlock_t*)malloc(sizeof(GnssCorrBlock_t));
        if (NULL == pbDataUint.pz_CorrBlock)
        {
          continue;
        }
        memcpy(pbDataUint.pz_CorrBlock, &z_CorrectBlock, sizeof(GnssCorrBlock_t));
        glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
        memset(pz_RinexDecoder->obs, 0, sizeof(RinexObs_t));
        pz_RinexDecoder->nobs = 0;
      }
    }
  }
  fclose(fp_ref_rinex);

  printf("Load... ref rinex file finish\n");

  free(pz_RinexDecoder);
}

/******************************************************************************
           Rover RINEX database load
******************************************************************************/
/**
 * @brief playback assist module, load rover block using RINEX format
 * @param[in] filename - input filename
 * @return None
 */
static void pb_assist_load_rover_rinex(const char* filename)
{
  int8_t s_ret = -1;
  char pu_buff[4096] = "";
  char pu_obsCode[C_GNSS_MAX][32][4] = { {""} };
  uint8_t pu_obsCodeNum[C_GNSS_MAX] = { 0 };
  FILE* fp_rover_rinex = fopen(filename, "r");
  if (NULL == fp_rover_rinex)
  {
    printf("Open File %s Fail, exit...\n", filename);
    return;
  }
  printf("Load... rover rinex file %s\n", filename);

  RinexDecoder_t* pz_RinexDecoder = (RinexDecoder_t*)malloc(sizeof(RinexDecoder_t));

  if (NULL == pz_RinexDecoder)
  {
    printf("Alloc RinexDecoder Size=%d Fail, exit...\n", sizeof(RtcmDecoder_t));
    return;
  }

  memset(pz_RinexDecoder, 0, sizeof(RinexDecoder_t));
  while (fgets(pu_buff, sizeof(pu_buff), fp_rover_rinex))
  {
    if (s_ret == -1)
    {
      s_ret = rinex_dec_obsf(pz_RinexDecoder, pu_buff, pu_obsCode, pu_obsCodeNum);
    }
    if (s_ret != -1 && !strstr(pu_buff, "HEADER"))
    {
      s_ret = rinex_dec_obsb(pz_RinexDecoder, pu_buff, pu_obsCode, pu_obsCodeNum);
      if (s_ret == 1 && pz_RinexDecoder->nobs != 0)
      {
        GnssMeasBlock_t z_GnssMeasBlock = { 0 };
        rinex_ExtractMeasBlk(pz_RinexDecoder, &z_GnssMeasBlock);

        PlaybackDataUnit pbDataUint;
        pbDataUint.u_type = PB_LOCATION_DATA_RCV_MEAS;
        pbDataUint.t_tow_ms = z_GnssMeasBlock.z_Clock.z_gpsTime.q_towMsec;
        pbDataUint.pz_RcvMeasBlock = (GnssMeasBlock_t*)malloc(sizeof(GnssMeasBlock_t));
        if (NULL == pbDataUint.pz_RcvMeasBlock)
        {
          continue;
        }
        memcpy(pbDataUint.pz_RcvMeasBlock, &z_GnssMeasBlock, sizeof(GnssMeasBlock_t));
        glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
        memset(pz_RinexDecoder->obs, 0, sizeof(RinexObs_t));
        pz_RinexDecoder->nobs = 0;
      }
    }
  }
  fclose(fp_rover_rinex);

  printf("Load... rover rinex file finish\n");

  free(pz_RinexDecoder);
}

/******************************************************************************
           Broadcast Ephemeris RINEX database
******************************************************************************/
/**
 * @brief get broadcast ephemeris from RINEX file
 * @param[in] filename - input filename
 * @return None
 */
static void pb_assist_load_eph_rinex(const char* filename)
{
  char pu_id[5] = "";
  char pu_buff[4096] = { 0 };
  char* pu_buff0 = (char*)malloc(sizeof(char));
  int8_t s_j = 0;
  int8_t s_step = 3;
  int8_t s_index = 0;
  uint8_t u_prn = 0;
  gnss_ConstellationType u_constellation = C_GNSS_NONE;
  double pd_data[64] = { 0 };
  uint32_t pq_epoch[6] = { 0 };
  FILE* fp_eph_rinex = fopen(filename, "r");
  uint16_t w_fake_week[ALL_GNSS_SYS_SV_NUMBER] = {0};

  RinexDecoder_t* pz_RinexDecoder = (RinexDecoder_t*)malloc(sizeof(RinexDecoder_t));
  if (NULL == pz_RinexDecoder)
  {
    printf("Alloc pz_RinexDecoder Size=%d Fail, exit...\n", sizeof(RinexDecoder_t));
    return;
  }

  if (NULL == fp_eph_rinex)
  {
    printf("Open Rinex nav File %s Fail, exit...\n", filename);
    return;
  }

  printf("Load... Rinex nav file %s\n", filename);

  while (fgets(pu_buff, sizeof(pu_buff), fp_eph_rinex))
  {
    if (strstr(pu_buff, "RINEX"))
    {
      sprintf(pu_id, "%.4s", pu_buff + 5);
      if (atoi(pu_id) >= 3)
      {
        s_step = 4;
      }
      continue;
    }
    if (strstr(pu_buff, "PGM") || strstr(pu_buff, "IONOSPHERIC") || strstr(pu_buff, "TIME") || strstr(pu_buff, "LEAP") || strstr(pu_buff, "HEADER"))
    {
      continue;
    }
    if (s_index == 0)
    {
      sprintf(pu_id, "%.3s", pu_buff);
      u_prn = atoi(pu_id + 1);
      if (strstr(pu_id, "G"))
      {
        u_constellation = C_GNSS_GPS;
      }
      else if (strstr(pu_id, "C"))
      {
        u_constellation = C_GNSS_BDS3;
      }
      else if (strstr(pu_id, "E"))
      {
        u_constellation = C_GNSS_GAL;
      }
      else if (strstr(pu_id, "J"))
      {
        u_constellation = C_GNSS_QZS;
      }
      else
      {
        continue;
      }
      for (s_j = 0, pu_buff0 = pu_buff + s_step; s_j < 6; pu_buff0 += (s_j == 1 ? 5 : 3))
      {
        pq_epoch[s_j++] = atoi(pu_buff0);
      }
      for (s_j = 0, pu_buff0 = pu_buff + s_step + 19; s_j < 3; s_j++, pu_buff0 += 19)
      {
        pd_data[s_index++] = atof(pu_buff0);
      }
    }
    else
    {
      for (s_j = 0, pu_buff0 = pu_buff + s_step; s_j < 4; s_j++, pu_buff0 += 19)
      {
        pd_data[s_index++] = atof(pu_buff0);
      }
      /* decode ephemeris */
      if ((u_constellation == C_GNSS_GLO && s_index >= 13) || s_index >= 29)
      {
        if (u_constellation == C_GNSS_GAL && pd_data[20] != 513)
        {  //only using I/NAV navigation of Galileo
          s_index = 0;
          memset(pd_data, 0, sizeof(pd_data));
          continue;
        }
        memset(pz_RinexDecoder, 0, sizeof(RinexDecoder_t));
        if (!rinex_dec_eph(u_constellation, u_prn, pd_data, pq_epoch, pz_RinexDecoder))
        {
          printf("Read broadcast ephemeris error from RINEX file: sat:%s\n", pu_id);
        }
        else
        {
          uint32_t q_svidx = gnss_cvt_Svid2SvIndex(u_prn, u_constellation);
          if (ALL_GNSS_SYS_SV_NUMBER == q_svidx)
          {
            continue;
          }
          const gnss_Ephemeris_t *z_eph = PlaybackDataUnit_find_latest_eph(u_constellation, u_prn);
          if (z_eph != NULL && z_eph->z_tocOfGpst.w_week != 0 && z_eph->z_tocOfGpst.w_week != pz_RinexDecoder->ephemeris.z_tocOfGpst.w_week)
          {
            w_fake_week[q_svidx] += (pz_RinexDecoder->ephemeris.z_tocOfGpst.w_week - z_eph->z_tocOfGpst.w_week);
          }
          PlaybackDataUnit pbDataUint;
          pbDataUint.u_type = PB_LOCATION_DATA_EPH;
          pbDataUint.t_tow_ms = pz_RinexDecoder->q_towMsec;
          pbDataUint.u_fake_week = w_fake_week[q_svidx];
          pbDataUint.pz_Ephemeris = (gnss_Ephemeris_t*)malloc(sizeof(gnss_Ephemeris_t));
          if (NULL == pbDataUint.pz_Ephemeris)
          {
            continue;
          }
          memcpy(pbDataUint.pz_Ephemeris, &pz_RinexDecoder->ephemeris, sizeof(gnss_Ephemeris_t));
          glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
          s_index = 0;
          memset(pd_data, 0, sizeof(pd_data));
        }
      }
    }
  }
  fclose(fp_eph_rinex);

  if (gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_ALE)
  {
    pb_database_sequence();
  }

  printf("Load... Rinex nav file finish\n");

  free(pz_RinexDecoder);
}

/******************************************************************************
           truth dynamic file (GGA)
******************************************************************************/
/**
 * @brief get truth dynamic file (GGA) file
 * @param[in] filename - input filename
 * @return None
 */
static void pb_truth_load_dynamic_gga(const char* filename)
{
  char pu_id[5] = "";
  char pu_buff[4096] = { 0 };
  char* pu_buff0 = (char*)malloc(sizeof(char));
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_epochNum = 0;
  uint8_t u_ggaErrorState = FALSE;
  const uint8_t u_epochNumPerNode = 100;
  uint16_t w_count = 0;
  double d_hhmmss = 0;
  double d_secondOfDay = 0.0;
  double d_tempNum = 0.0;
  double d_tow = 0.0;
  FILE* fp_dynamic_gga = fopen(filename, "r");

  cmn_highPrecision_oneEpoch* pz_highPrecisionData = (cmn_highPrecision_oneEpoch*)
    malloc(u_epochNumPerNode * sizeof(cmn_highPrecision_oneEpoch));
  if (NULL == pz_highPrecisionData)
  {
    printf("Alloc cmn_highPrecision_oneEpoch Size=%d Fail, exit...\n", sizeof(cmn_highPrecision_oneEpoch));
    return;
  }
  if (NULL == fp_dynamic_gga)
  {
    printf("Open Dynamic gga File %s Fail, exit...\n", filename);
    return;
  }

  printf("Load... Rinex nav file %s\n", filename);
  cmn_highPrecision_init();

  memset(pz_highPrecisionData,0, u_epochNumPerNode * sizeof(cmn_highPrecision_oneEpoch));
  while (fgets(pu_buff, sizeof(pu_buff), fp_dynamic_gga))
  {
    w_count = 0;
    u_ggaErrorState = FALSE;
    if (strstr(pu_buff, "GGA,") && !strstr(pu_buff, ",,,,,,"))
    {
      d_secondOfDay = -1.0;
      for (u_i = 0; pu_buff[u_i] != '\0'; ++u_i) 
      {
        if (pu_buff[u_i] == ',' && pu_buff[u_i + 1] != ',') 
        {
          ++w_count;
          if (w_count == 1)
          {
            for (u_j = 1; u_j <= 9; u_j++)
            {
              if (pu_buff[u_i + u_j] == '\0' || pu_buff[u_i + u_j] == ',')
              {
                continue;
              }
            }
            std::string subString(pu_buff + u_i + 1, 9);
            d_hhmmss = std::atof(subString.c_str());
            uint8_t hour = (uint8_t)(d_hhmmss / 10000);
            uint8_t minute = (uint8_t)((d_hhmmss - hour * 10000.0) / 100);
            double second = d_hhmmss - hour * 10000.0 - minute * 100.0;
            d_secondOfDay = hour * 3600.0 + minute * 60.0 + second;
            pz_highPrecisionData[u_epochNum].d_tod = d_secondOfDay;
            u_i += 9;
            continue;
          }
          else if (d_secondOfDay > 0.0 && w_count == 2)
          {
            u_j = u_i;
          }
          else if (d_secondOfDay > 0.0 && w_count == 3 && u_i - u_j - 1 > 0)
          {
            std::string subString(pu_buff + u_j + 1, u_i - u_j - 1);
            pz_highPrecisionData[u_epochNum].pd_pos[0] = std::atof(subString.c_str());
          }
          else if (d_secondOfDay > 0.0 && w_count == 4)
          {
            u_j = u_i;
          }
          else if (d_secondOfDay > 0.0 && w_count == 5 && u_i - u_j - 1 > 0)
          {
            std::string subString(pu_buff + u_j + 1, u_i - u_j - 1);
            pz_highPrecisionData[u_epochNum].pd_pos[1] = std::atof(subString.c_str());
          }
          else if (d_secondOfDay > 0.0 && w_count == 9)
          {
            u_j = u_i;
          }
          else if (d_secondOfDay > 0.0 && w_count == 10 && u_i - u_j - 1 > 0)
          {
            std::string subString(pu_buff + u_j + 1, u_i - u_j - 1);
            pz_highPrecisionData[u_epochNum].pd_pos[2] = std::atof(subString.c_str());
          }
          else if (d_secondOfDay > 0.0 && w_count == 11)
          {
            u_j = u_i;
          }
          else if (d_secondOfDay > 0.0 && w_count == 12 && u_i - u_j - 1 > 0)
          {
            std::string subString(pu_buff + u_j + 1, u_i - u_j - 1);
            pz_highPrecisionData[u_epochNum].pd_pos[2] += std::atof(subString.c_str());
          }
        }
      }
    }
    if (w_count >= 12 && d_secondOfDay > 0.0)
    {
      for (u_i = 0; u_i < 3; u_i++)
      {
        if (fabs(pz_highPrecisionData[u_epochNum].pd_pos[0]) < 1.0)
        {
          u_ggaErrorState = TRUE;
        }
      }
      if (!u_ggaErrorState)
      {
        u_epochNum++;
      }
    }
    /*if (!fgets(pu_buff, sizeof(pu_buff), fp_dynamic_gga))
    {
      break;
    }
    if (strstr(pu_buff, "RMC,"))
    {
      uint16_t w_count = 0;
      for (u_i = 0; pu_buff[u_i] != '\0'; ++u_i) {
        if (pu_buff[u_i] == ',' && pu_buff[u_i + 1] != ',') {
          ++w_count;
          for (u_j = 1; u_j <= 6; u_j++)
          {
            if (pu_buff[u_i + u_j] == '\0')
            {
              continue;
            }
          }
          if (w_count == 9)
          {
            std::string subString(pu_buff + 1, 6);
            q_ddmmyy = std::atoi(subString.c_str());
            continue;
          }
        }
      }
      continue;
    }*/
    if (u_epochNum >= u_epochNumPerNode)
    {
      cmn_highPrecision_inject(u_epochNum, pz_highPrecisionData);
      u_epochNum = 0;
      memset(pz_highPrecisionData, 0, u_epochNumPerNode * sizeof(cmn_highPrecision_oneEpoch));
    }
  }
  fclose(fp_dynamic_gga);

  printf("Load... Dynamic gga file finish\n");

  free(pu_buff0);
  free(pz_highPrecisionData);
}
static int8_t bddb_decode_SelfCheck(uint8_t* str, uint32_t len)
{
  uint8_t u_checkA = 0;
  uint32_t i;

  for (i = 0; i < len - 1; i++)
  {
    u_checkA ^= str[i];
  }
  if (u_checkA == str[len - 1])
  {
    return 1;
  }

  return 0;
}

/******************************************************************************
           IMU database
******************************************************************************/
/**
 * @brief playback assist module, load assist imu p_data using BDDB format
 * @param[in] filename - input filename
 * @return None
 */
static void pb_assist_load_imu_ag_bddb(const char* filename)
{
  FILE* file_fp = fopen(filename, "rb");
  if (NULL == file_fp)
  {
    printf("Open File %s Fail, exit...\n", filename);
    return;
  }
  printf("Load... BDDB file %s\n", filename);
  

  /*read lever-arm from BDDB0B first*/
#if 1
#define KWS_INIT 0.00863   
  uint8_t gu_str[5000] = { 0 };
  uint32_t gq_strLength = 0;
  static uint8_t u_imu_fake_week = 0, u_gnss_fake_week = 0, u_wheel_fake_week = 0;
  static uint64_t t_imu_towMsec = 0, t_gnss_towMsec = 0, t_wheel_towMsec = 0;
  while (!feof(file_fp) || gq_strLength >= 15)
  {
    size_t len = 0;    
    if (gq_strLength < 3)
    {
      len = fread(&gu_str[gq_strLength], 1, 1, file_fp);
      gq_strLength += len;
      continue;
    }
    if (gu_str[0] != 0xBD || gu_str[1] != 0xDB)
    {
      memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
      gq_strLength--;
      continue;
    }
    if (gu_str[2] == 0x0B)
    {
      if (gq_strLength < LEN_BD_DB_0B) {
        len = fread(&gu_str[gq_strLength], 1, LEN_BD_DB_0B - gq_strLength, file_fp);
        gq_strLength += len;
      }
      if (gq_strLength < LEN_BD_DB_0B)
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
        continue;
      }
      if (bddb_decode_SelfCheck(gu_str, LEN_BD_DB_0B))
      {
        static float f_la_imu2gnss[3] = { 0.0f,0.0f,0.0f };
        static float f_la_imu2rearmid[3] = { 0.0f,0.0f,0.0f };
        static float f_mis_b2v[3] = { 0.0f,0.0f,0.0f }; /*rad*/
        static float f_mis_b2g[3] = { 0.0f,0.0f,0.0f }; /*rad*/
        static float f_whlspd_sf[4] = { 0.0f,0.0f,0.0f,0.0f };
        static float f_la_rear2rear = 0.0;
        static INSResults_t z_InsData = { 0 };
        static uint8_t preadnum = 0;
        if (preadnum < 5)
        {
          preadnum = ag_imu_bddb_parse_0x0B(gu_str, gq_strLength, &z_InsData);
          memmove(&gu_str[0], &gu_str[LEN_BD_DB_0B], LEN_BD_DB_0B);
          gq_strLength = gq_strLength - LEN_BD_DB_0B;

          if (preadnum == 5)
          {
            f_la_imu2gnss[0] = z_InsData.f_la_imu2gnss[0];
            f_la_imu2gnss[1] = z_InsData.f_la_imu2gnss[1];
            f_la_imu2gnss[2] = z_InsData.f_la_imu2gnss[2];            
            f_la_imu2rearmid[0] = z_InsData.f_la_imu2rearmid[0];
            f_la_imu2rearmid[1] = z_InsData.f_la_imu2rearmid[1];
            f_la_imu2rearmid[2] = z_InsData.f_la_imu2rearmid[2];            
            f_mis_b2v[0] = (float)(z_InsData.f_mis_roll * DEG2RAD);
            f_mis_b2v[1] = (float)(z_InsData.f_mis_pitch * DEG2RAD);
            f_mis_b2v[2] = (float)(z_InsData.f_mis_yaw * DEG2RAD);
            f_mis_b2g[0] = (float)(z_InsData.f_misdualant_roll * DEG2RAD);
            f_mis_b2g[1] = (float)(z_InsData.f_misdualant_pitch * DEG2RAD);
            f_mis_b2g[2] = (float)(z_InsData.f_misdualant_yaw * DEG2RAD);
            f_whlspd_sf[2] = z_InsData.f_whlspd_sf[2];
            f_whlspd_sf[3] = z_InsData.f_whlspd_sf[3];
            f_la_rear2rear = z_InsData.f_la_rear2rear;  
            
            asensing_lvrm_set_gnss2imu(f_la_imu2gnss);//read from 0B 
            asensing_lvrm_set_rearmid2imu(f_la_imu2rearmid);
            asensing_lvrm_set_tworears(f_la_rear2rear);
            asensing_mis_set_imu2vehicle(f_mis_b2v);
            asensing_mis_set_imu2dualant(f_mis_b2g);

            //pb_config_misalign(f_mis_b2v);            

            printf("la_imu2rearmid_m: %.4lf %.4lf %.4lf\n", f_la_imu2rearmid[0], f_la_imu2rearmid[1], f_la_imu2rearmid[2]);
            printf("la_imu2gnss_m: %.4lf %.4lf %.4lf\n", f_la_imu2gnss[0], f_la_imu2gnss[1], f_la_imu2gnss[2]);
            printf("mis_b2v_deg: %.4lf %.4lf %.4lf\n", f_mis_b2v[0] * RAD2DEG, f_mis_b2v[1] * RAD2DEG, f_mis_b2v[2] * RAD2DEG);
            printf("mis_b2g_deg: %.4lf %.4lf %.4lf\n", f_mis_b2g[0] * RAD2DEG, f_mis_b2g[1] * RAD2DEG, f_mis_b2g[2] * RAD2DEG);
            printf("kws: %.5lf %.5lf\n", KWS_INIT* (1.0 - f_whlspd_sf[2]), KWS_INIT* (1.0 - f_whlspd_sf[2]));
            printf("la_rear2rear_m: %.4lf \n", f_la_rear2rear);
            break;
          }
        }
             
      }
      else
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
      }
    }
    else
    {
      memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
      gq_strLength--;
    }
  }
  
  rewind(file_fp);
#endif

  //uint8_t gu_str[5000] = { 0 };
  //uint32_t gq_strLength = 0;
  memset(gu_str, 0, 5000*sizeof(uint8_t));
  gq_strLength = 0;
  while (!feof(file_fp) || gq_strLength >= 15)
  {
    size_t len = 0;
    if (gq_strLength < 3)
    {
      len = fread(&gu_str[gq_strLength], 1, 1, file_fp);
      gq_strLength += len;
      continue;
    }
    if (gu_str[0] != 0xBD || gu_str[1] != 0xDB)
    {
      memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
      gq_strLength--;
      continue;
    }
    if (gu_str[2] == 0x0A)
    {
      // IMU data total length 34/35 byte
      if (gq_strLength < LEN_BD_DB_0A) {
        len = fread(&gu_str[gq_strLength], 1, LEN_BD_DB_0A - gq_strLength, file_fp);
        gq_strLength += len;
      }
      if (gq_strLength < LEN_BD_DB_0A)
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
        continue;
      }
      if (bddb_decode_SelfCheck(gu_str, LEN_BD_DB_0A))
      {
        ag_imu_data_t z_ImuData = { 0 };
        ag_imu_bddb_parse_0x0A(gu_str, gq_strLength, &z_ImuData);
        memmove(&gu_str[0], &gu_str[LEN_BD_DB_0A], LEN_BD_DB_0A);
        gq_strLength = gq_strLength - LEN_BD_DB_0A;

        PlaybackDataUnit pbDataUint;
        pbDataUint.u_type = PB_LOCATION_DATA_BDDB_IMU;
        pbDataUint.t_tow_ms = z_ImuData.q_tow_msec;
        pbDataUint.pz_ImuData = (loc_api_imu_data_t*)malloc(sizeof(loc_api_imu_data_t));
        if (NULL == pbDataUint.pz_ImuData)
        {
          continue;
        }
        pbDataUint.pz_ImuData->q_tow_msec = z_ImuData.q_tow_msec;
        pbDataUint.pz_ImuData->f_temperature = z_ImuData.f_temperature;
        for (uint32_t i = 0; i < 3; i++)
        {
          pbDataUint.pz_ImuData->f_gyro[i] = z_ImuData.f_gyro[i];
          pbDataUint.pz_ImuData->f_accl[i] = z_ImuData.f_accl[i];
        }
        if ((t_imu_towMsec != 0) &&
          ((double)t_imu_towMsec - (double)pbDataUint.t_tow_ms) > (double) WEEK_MSEC / 2)
        {
          u_imu_fake_week++;
        }
        pbDataUint.u_fake_week = u_imu_fake_week;
        t_imu_towMsec = pbDataUint.t_tow_ms;
        glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
      }
      else
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
      }
    }
    else if (gu_str[2] == 0x10)
    {
      // GNSS data total length 70/74 byte
      if (gq_strLength < LEN_BD_DB_10) {
        len = fread(&gu_str[gq_strLength], 1, LEN_BD_DB_10 - gq_strLength, file_fp);
        gq_strLength += len;
      }
      if (gq_strLength < LEN_BD_DB_10)
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
        continue;
      }
      if (bddb_decode_SelfCheck(gu_str, LEN_BD_DB_10))
      {
        GnssFixType z_GnssType = { 0 };
        ag_imu_bddb_parse_0x10(gu_str, gq_strLength, &z_GnssType);
        memmove(&gu_str[0], &gu_str[LEN_BD_DB_10], LEN_BD_DB_10);
        gq_strLength = gq_strLength - LEN_BD_DB_10;

        PlaybackDataUnit pbDataUint;
        pbDataUint.u_type = PB_LOCATION_DATA_BDDB_GNSS;
        pbDataUint.t_tow_ms = (uint32_t)(z_GnssType.ItowPos * 1000);
        pbDataUint.pz_GnssTypeBDDB10 = (GnssFixType*)malloc(sizeof(GnssFixType));
        if (NULL == pbDataUint.pz_GnssTypeBDDB10)
        {
          continue;
        }
        memcpy(pbDataUint.pz_GnssTypeBDDB10, &z_GnssType, sizeof(GnssFixType));
        if ((t_gnss_towMsec != 0) &&
          ((double)t_gnss_towMsec - (double)pbDataUint.t_tow_ms) > (double)WEEK_MSEC / 2)
        {
          u_gnss_fake_week++;
        }
        pbDataUint.u_fake_week = u_gnss_fake_week;
        t_gnss_towMsec = pbDataUint.t_tow_ms;
        glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
      }
      else
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
      }
    }
    else if (gu_str[2] == 0x20)
    {
      // Wheel data total length 35 byte
      if (gq_strLength < LEN_BD_DB_20) {
        len = fread(&gu_str[gq_strLength], 1, LEN_BD_DB_20 - gq_strLength, file_fp);
        gq_strLength += len;
      }
      if (gq_strLength < LEN_BD_DB_20)
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
        continue;
      }
      if (bddb_decode_SelfCheck(gu_str, LEN_BD_DB_20))
      {
        loc_api_wheel_t z_WheelData = { 0 };
        ag_imu_bddb_parse_0x20(gu_str, gq_strLength, &z_WheelData);
        memmove(&gu_str[0], &gu_str[LEN_BD_DB_20], LEN_BD_DB_20);
        gq_strLength = gq_strLength - LEN_BD_DB_20;

        PlaybackDataUnit pbDataUint;
        pbDataUint.u_type = PB_LOCATION_DATA_BDDB_WHEEL;
        pbDataUint.t_tow_ms = (uint32_t)(z_WheelData.t_timestamp);
        pbDataUint.pz_WheelData = (loc_api_wheel_t*)malloc(sizeof(loc_api_wheel_t));
        if (NULL == pbDataUint.pz_WheelData)
        {
          continue;
        }
        memcpy(pbDataUint.pz_WheelData, &z_WheelData, sizeof(loc_api_wheel_t));
        if ((t_wheel_towMsec != 0) &&
          ((double)t_wheel_towMsec - (double)pbDataUint.t_tow_ms) > (double)WEEK_MSEC / 2)
        {
          u_wheel_fake_week++;
        }
        pbDataUint.u_fake_week = u_wheel_fake_week;
        t_wheel_towMsec = pbDataUint.t_tow_ms;
        glt_PlaybackRawInjectData_Database.push_back(pbDataUint);
      }
      else
      {
        memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
        gq_strLength--;
      }
    }
    else
    {
      memmove(&gu_str[0], &gu_str[1], gq_strLength - 1);
      gq_strLength--;
    }
  }


#ifdef FALSE_BDDB
  /* load file into memory */
  uint32_t q_FileSize = pb_get_file_size(filename);
  uint8_t* pu_FileReadBuf = (uint8_t*)malloc(q_FileSize);
  if (NULL == pu_FileReadBuf)
  {
    printf("Alloc buffer Size=%d Fail, exit...\n", q_FileSize);
    return;
  }
  memset(pu_FileReadBuf, 0, q_FileSize);
  fread(pu_FileReadBuf, q_FileSize, 1, file_fp);

  ag_imu_bddb_decoder_t* pz_imu_bddb_decoder = 
    (ag_imu_bddb_decoder_t*)malloc(sizeof(ag_imu_bddb_decoder_t));
  if (NULL == pz_imu_bddb_decoder)
  {
    printf("Alloc buffer Size=%d Fail, exit...\n", q_FileSize);
    return;
  }
  memset(pz_imu_bddb_decoder, 0, sizeof(ag_imu_bddb_decoder_t));

  /* Decode IMU BDDB buffer */
  uint8_t imu_bddb_decoded_data[AG_IMU_BDDB_DECODED_BUFFER_SIZE] = { 0 };
  uint32_t imu_bddb_decoded_length = AG_IMU_BDDB_DECODED_BUFFER_SIZE;
  uint8_t ret = 0;

  for (uint32_t i = 0; i < q_FileSize; i++)
  {
    ret = ag_imu_bddb_decode(pz_imu_bddb_decoder, pu_FileReadBuf[i], 
      imu_bddb_decoded_data, &imu_bddb_decoded_length);
    if (0 == ret)
    {
      continue;
    }

    switch (ret)
    {
    case 0x0A:
    {
      ag_imu_data_t z_ImuData = { 0 };
      ag_imu_bddb_parse_0x0A(imu_bddb_decoded_data, imu_bddb_decoded_length, &z_ImuData);

      PlaybackDataUnit pbDataUint;
      pbDataUint.u_type = PB_LOCATION_DATA_BDDB_IMU;
      pbDataUint.t_tow_ms = z_ImuData.q_tow_msec;
      pbDataUint.pz_ImuData = (loc_api_imu_data_t*)malloc(sizeof(loc_api_imu_data_t));
      if (NULL == pbDataUint.pz_ImuData)
      {
        continue;
      }

      pbDataUint.pz_ImuData->q_tow_msec = z_ImuData.q_tow_msec;
      pbDataUint.pz_ImuData->f_temperature = z_ImuData.f_temperature;
      for (uint32_t i = 0; i < 3; i++)
      {
        pbDataUint.pz_ImuData->f_gyro[i] = z_ImuData.f_gyro[i];
        pbDataUint.pz_ImuData->f_accl[i] = z_ImuData.f_accl[i];
      }
      glt_PlaybackRawInjectData_Database.push_back(pbDataUint);

      break;
    }

    case 0x0B:
    {

      ag_imu_bddb_parse_0x0B(imu_bddb_decoded_data, imu_bddb_decoded_length);
      break;
    }

    case 0x10:
    {
      GnssFixType z_GnssType = { 0 };
      ag_imu_bddb_parse_0x10(imu_bddb_decoded_data, imu_bddb_decoded_length, &z_GnssType);

      PlaybackDataUnit pbDataUint;
      pbDataUint.u_type = PB_LOCATION_DATA_BDDB_GNSS;
      pbDataUint.t_tow_ms = (uint32_t)(z_GnssType.ItowPos * 1000);
      pbDataUint.pz_GnssTypeBDDB10 = (GnssFixType*)malloc(sizeof(GnssFixType));
      if (NULL == pbDataUint.pz_GnssTypeBDDB10)
      {
        continue;
      }
      memcpy(pbDataUint.pz_GnssTypeBDDB10, &z_GnssType, sizeof(GnssFixType));
      glt_PlaybackRawInjectData_Database.push_back(pbDataUint);

      break;
    }

    default:
      break;
    }
  }
#endif

  printf("Load... BDDB file finish\n");
#ifdef FALSE_BDDB
  free(pz_imu_bddb_decoder);
  free(pu_FileReadBuf);
#endif
  fclose(file_fp);

  return;
}

void pb_truth_load_clear()
{
  cmn_highPrecision_deinit();
}
static void pb_truth_load_reference_file(const char* filename)
{
#ifdef INS_DEBUG
  FILE* file_fp = fopen(filename, "rb");
  if (NULL == file_fp)
  {
    printf("Open File %s Fail, exit...\n", filename);
    return;
  }
  printf("Load... Reference file %s\n", filename);
  ref_ModuleInit(file_fp);
  printf("Load... Reference file finish\n");
  fclose(file_fp);
#endif
}

void pb_assist_load_clear()
{
  for (std::list<PlaybackDataUnit>::iterator ite = glt_PlaybackRawInjectData_Database.begin();
    ite != glt_PlaybackRawInjectData_Database.end();
    ite++)
  {
    free_PlaybackDataUnit(ite);
  }

  glt_PlaybackRawInjectData_Database.clear();
}

/******************************************************************************
                playback callback handler
******************************************************************************/
static log_decoder_t gz_log_decoder[2];

static void pb_parsed_data_handler(int8_t decode_type, uint8_t* p_data, uint32_t q_length)
{
  static char parse_str_buffer[20 * 1024] = { 0 };
  uint32_t q_parse_str_length = 0;

  FILE* fp = gz_pb_config.output.pb_output_text_fp;  

  if ((LOG_TYPE_IPC == decode_type) || (LOG_TYPE_CRC_IPC == decode_type))
  {
    ipc_t z_ipc = { 0 };
    memcpy(&z_ipc, p_data, sizeof(ipc_t));
    if (q_length > sizeof(ipc_t))
    {
      z_ipc.p_data = p_data + sizeof(ipc_t);
    }

    q_parse_str_length += ipc_parse_execute(&z_ipc, parse_str_buffer + q_parse_str_length);
    if (NULL != fp) fwrite(parse_str_buffer, q_parse_str_length, 1, fp);
  }
  else if ((LOG_TYPE_TEXT == decode_type) || (LOG_TYPE_CRC_TEXT == decode_type))
  {
    if (NULL != fp) fwrite(p_data, q_length, 1, fp);
  }
  else if ((LOG_TYPE_PACKAGE == decode_type) || (LOG_TYPE_CRC_PACKAGE == decode_type))
  {
    q_parse_str_length += package_parse_execute(p_data, q_length,
      parse_str_buffer + q_parse_str_length);
    if (NULL != fp) fwrite(parse_str_buffer, q_parse_str_length, 1, fp);
  }

}

/**
 * @brief Playback log report callback handler
 * @param[in] p_data - log p_data pointer
 * @param[in] q_length - log p_data length
 * @return      None
 */
void pb_log_callback_handler(uint8_t* p_data, uint32_t q_length)
{
  // AG file
  if (gz_pb_config.output.pb_output_ag_fp != NULL)
  {
    fwrite(p_data, q_length, 1, gz_pb_config.output.pb_output_ag_fp);
  }
  
  // parse and execute ipc
  loc_core_log_parse(&gz_log_decoder[0], pb_parsed_data_handler, p_data, q_length, TRUE);

  return;
}

/**
 * @brief Playback location fix report callback handler
 * @param[in] location - location fix report
 * @return      None
 */
void pb_report_location_callback_handler(loc_api_location_report_t* location)
{

  return;
}

/**
 * @brief Playback orient report callback handler
 * @param[in] location - orient fix report
 * @return      None
 */
void pb_report_orient_callback_handler(loc_api_orient_report_t* location)
{
  uint8_t nmea_hdt[256] = { 0 };
  uint8_t nmea_tra[256] = { 0 };
  uint8_t nmea_ksxt[256] = { 0 };
  loc_api_nmea_create_hdt(location, nmea_hdt);
  loc_api_nmea_create_tra(location, nmea_tra);
  loc_api_nmea_create_ksxt(location, nmea_ksxt);
  return;
}

void pb_report_consolidated_location_callback_handler(loc_api_ConsoildatedPositionFix_t* pz_LocPositionFix)
{
  if ((LOC_API_FIX_SOURCE_PVT == pz_LocPositionFix->u_fixSource)
    || (LOC_API_FIX_SOURCE_RTK == pz_LocPositionFix->u_fixSource)
    || (LOC_API_FIX_SOURCE_PPP == pz_LocPositionFix->u_fixSource))
  {
    uint8_t nmea_gga[256] = { 0 };
    loc_api_nmea_create_gga(pz_LocPositionFix, nmea_gga);
    printf("GNSS: %s", nmea_gga);
  }
  else if (LOC_API_FIX_SOURCE_INS == pz_LocPositionFix->u_fixSource)
  {
    uint8_t nmea_gga[256] = { 0 };
    loc_api_nmea_create_gga(pz_LocPositionFix, nmea_gga);
    //printf("INS : %s", nmea_gga);
  }

  return;
}

/**
 * @brief location core memory free callback
 * @param[in]
 * @return      None
 */
chrono::system_clock::time_point t_start;
static uint64_t loc_core_callback_get_tick_ms()
{
  static uint8_t flag = 0;
  if (0 == flag)
  {
    flag = 1;
    t_start = std::chrono::system_clock::now();
  }

  return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - t_start).count();
}

/**
 * @brief location core memory alloc callback
 * @param[in]
 * @return      None
 */
static void* loc_core_callback_mem_alloc_handler(uint32_t sz, const char* func, uint32_t line, char* file)
{
  return MemoryStatisticsMalloc(sz, func, line, file);
}

/**
 * @brief location core memory free callback
 * @param[in]
 * @return      None
 */
static void loc_core_callback_mem_free_handler(void* ptr)
{
  return MemoryStatisticsFree(ptr);
}

/**
 * @brief Meas and satellite block process reported Callback
 * 
 * @param[in] pz_GnssMeasBlk 
 */
static void loc_core_callback_report_gnss_meas_satpos(loc_api_GnssMeasSatBlock_t* pz_GnssMeasSatBlk)
{
#if defined(APP_HEXAGON_SSR)
  tsx_inject_ApiGnssMeasSat(pz_GnssMeasSatBlk);
#endif

#ifdef APP_SWIFT_SSR
  swift_sdk_runing(pz_GnssMeasSatBlk);
#endif // APP_SWIFT_SSR

  return;
}


/**
 * @brief playback ipc process main entry
 * @param[in] ipc -> ipc message from log
 * @return None
 */
void pb_ipc_process(ipc_t* pz_ipc)
{
  if (!ipctask_ipc_validity_check(pz_ipc->u_dst_id, pz_ipc->u_src_id, 
    pz_ipc->q_ipc_id, IPC_VALIDITY_TYPE_PLAYBACK))
  {
    return;
  }

  ipc_upgrade(pz_ipc);
  
  if (pz_ipc->u_need_record)
  {
    log_ipc(pz_ipc);
  }

  switch (pz_ipc->u_dst_id)
  {
  case TASK_INDEX_SM:
    sm_Proc(pz_ipc);
    break;
  case TASK_INDEX_HPP:
    hpp_Proc(pz_ipc);
    break;
  case TASK_INDEX_DCP:
    dcp_Proc(pz_ipc);
    break;
  case TASK_INDEX_PPP:
    ppp_Proc(pz_ipc);
    break;
  case TASK_INDEX_SD:
    sd_Proc(pz_ipc);
    break;
  case TASK_INDEX_VDR:
    vdr_Proc(pz_ipc);
    break;
  default:
    break;
  }

  if (pz_ipc->u_use_heap)
  {
    OS_FREE(pz_ipc->p_data);
    pz_ipc->u_use_heap = FALSE;
  }

  return;
}


static void pb_parsed_log_mode(int8_t decode_type, uint8_t* p_data, uint32_t q_length)
{
  uint8_t u_tnum = 0;

  if ((LOG_TYPE_IPC == decode_type) || (LOG_TYPE_CRC_IPC == decode_type))
  {
    ipc_t z_ipc = {0};
    memcpy(&z_ipc, p_data, sizeof(ipc_t));
    if (q_length > sizeof(ipc_t))
    {
      z_ipc.p_data = p_data + sizeof(ipc_t);
    }
    
    if (C_M_SM_REF_CORR_RTCM == z_ipc.q_ipc_id)
    {
        gz_pb_config_log.core.pb_loc_core_hpp_rtk_enable = 1;
    }
    if(C_M_PPP_INIT == z_ipc.q_ipc_id || C_M_PPP_ALGO_START == z_ipc.q_ipc_id)
    {
        gz_pb_config_log.core.pb_loc_core_ppp_task_enable = 1;
    }
  
    /* free ipc */
    if (z_ipc.u_use_heap)
    {
      OS_FREE(z_ipc.p_data);
      z_ipc.u_use_heap = FALSE;
    }
  }
}


static void pb_parseProcessMode(char* pb_input_file)
{
  static log_decoder_t pb_log_decoder;
  printf("Start parse process mode RTK or PPP or RTK/PPP\n");
  FILE* input_log_fp = fopen(pb_input_file, "rb");
  if (NULL == input_log_fp)
  {
    return;
  }
  double d_sample = 0.0;
  double d_sample_tmp = 0;

  log_decoder_t gz_pb_log_cb_decoder = {0};

  uint8_t* input_read_buf = (uint8_t*)malloc(READ_BUFFER_SIZE);
  uint64_t t_input_total_len = 0;

  while (!feof(input_log_fp))
  {
    size_t buf_len = fread(input_read_buf, 1, READ_BUFFER_SIZE, input_log_fp);
    if (buf_len < 1)
    {
      break;
    }

    t_input_total_len += buf_len;
    loc_core_log_parse(&pb_log_decoder, pb_parsed_log_mode, input_read_buf, buf_len, TRUE);

    if (1 == gz_pb_config_log.core.pb_loc_core_hpp_rtk_enable ||
        1 == gz_pb_config_log.core.pb_loc_core_ppp_task_enable)
    {
      break;
    }
  }

  printf("Log loc engine mode: rtk: %d, ppp: %d\n", gz_pb_config_log.core.pb_loc_core_hpp_rtk_enable,
         gz_pb_config_log.core.pb_loc_core_ppp_task_enable);

  fclose(input_log_fp);
  free(input_read_buf);
}


static void pb_parsed_log_posifix_interval(int8_t decode_type, uint8_t* p_data, uint32_t q_length)
{
  uint8_t u_tnum = 0;

  if (gz_pb_interval.u_tnum >= PB_FOR_SAMPLE_REPORT_POS_NUM)
  {
    return;
  }

  if ((LOG_TYPE_IPC == decode_type) || (LOG_TYPE_CRC_IPC == decode_type))
  {
    ipc_t z_ipc = {0};
    memcpy(&z_ipc, p_data, sizeof(ipc_t));
    if (q_length > sizeof(ipc_t))
    {
      z_ipc.p_data = p_data + sizeof(ipc_t);
    }

    if (C_M_SM_REPORT_POS_FIX == z_ipc.q_ipc_id)
    {
      ipc_upgrade(&z_ipc);

      gnss_PositionFix_t *pz_pos_solution = (gnss_PositionFix_t *)z_ipc.p_data;

      gz_pb_interval.w_posfix_vailed_num++;

      uint8_t u_last_idx = 0;
      if (gz_pb_interval.u_tnum != 0)
      {
        u_last_idx = gz_pb_interval.u_tnum - 1;
      }

      if ((FIX_SOURCE_HPP_V1 == pz_pos_solution->u_fixSource) &&
          (pz_pos_solution->z_gpsTime.q_towMsec != 0) &&
          (pz_pos_solution->z_gpsTime.q_towMsec != gz_pb_interval.q_towMsecList[u_last_idx]))
      {
        gz_pb_interval.q_towMsecList[gz_pb_interval.u_tnum++] = pz_pos_solution->z_gpsTime.q_towMsec;
      }

    }
    /* free ipc */
    if (z_ipc.u_use_heap)
    {
      OS_FREE(z_ipc.p_data);
      z_ipc.u_use_heap = FALSE;
    }
  }
}

float pb_parseProcessInterval(char* pb_input_file)
{
  static log_decoder_t pb_log_decoder;
  printf("Start parse process interval\n");
  FILE* input_log_fp = fopen(pb_input_file, "rb");
  if (NULL == input_log_fp)
  {
    return 0;
  }
  double d_sample = 0.0;
  double d_sample_tmp = 0;

  log_decoder_t gz_pb_log_cb_decoder = {0};

  uint8_t* input_read_buf = (uint8_t*)malloc(READ_BUFFER_SIZE);
  uint64_t t_input_total_len = 0;

  while (!feof(input_log_fp))
  {
    size_t buf_len = fread(input_read_buf, 1, READ_BUFFER_SIZE, input_log_fp);
    if (buf_len < 1)
    {
      break;
    }

    t_input_total_len += buf_len;
    loc_core_log_parse(&pb_log_decoder, pb_parsed_log_posifix_interval, input_read_buf, buf_len, TRUE);
    if ((gz_pb_interval.u_tnum >= PB_FOR_SAMPLE_REPORT_POS_NUM) ||
      (gz_pb_interval.w_posfix_vailed_num >= PB_FOR_SAMPLE_REPORT_POS_NUM * 5))
    {
      break;
    }
  }

  if (gz_pb_interval.u_tnum >= PB_FOR_SAMPLE_REPORT_POS_NUM / 2)
  {
    d_sample = 10000.0;
    for (uint8_t i = 0; i < gz_pb_interval.u_tnum - 1; i++)
    {
      d_sample_tmp = fabs(gz_pb_interval.q_towMsecList[i + 1] - gz_pb_interval.q_towMsecList[i]);
      if (d_sample_tmp < d_sample)
      {
        d_sample = d_sample_tmp;
      }
    }
    printf("End parse process interval suc, inteval %f s \n", d_sample * TIME_MSEC_INV);
  }
  else
  {
    printf("End parse process interval failed, hpp pos report num %d\n", gz_pb_interval.u_tnum);
  }
  fclose(input_log_fp);
  free(input_read_buf);

  return (float)(d_sample * TIME_MSEC_INV);
}

/**
 * @brief IPC log playback to load assist data
 * @return None
*/
static void pb_ipc_log_parsed_load_assist(uint64_t t_timestamp_ms)
{
  if (TRUE == M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_REF_RINEX_VALID))
  {
    // TODO: later
  }

  if (TRUE == M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_NAV_RINEX_VALID))
  {
    UtcTime_t z_utctime = { 0 };
    GpsTime_t z_gpstime = { 0 };
    z_utctime.t_unixMsec = t_timestamp_ms;
    tm_cvt_UtcTimeToGpst(&z_utctime, &z_gpstime);

    std::list<PlaybackDataUnit>::iterator ite = glt_PlaybackRawInjectData_Database.begin();

    uint8_t b_need_resize = FALSE;
    BOOL b_eph_match = FALSE;

    for (ite = glt_PlaybackRawInjectData_Database.begin(); ite != glt_PlaybackRawInjectData_Database.end(); ite++)
    {
      if (ite->t_tow_ms > z_gpstime.q_towMsec)
      {
        break;
      }

      if (ite->t_tow_ms <= z_gpstime.q_towMsec)
      {
        b_need_resize = TRUE;

        b_eph_match = rinex_ephMatch_AG(z_gpstime.q_towMsec, (uint32_t)ite->t_tow_ms, ite->pz_Ephemeris->u_constellation);
        if (b_eph_match)
        {
          hpp_api_Ephemeris_Put(ite->pz_Ephemeris);
          sd_api_Ephemeris_Put(ite->pz_Ephemeris);
        }
      }
    }

    if (b_need_resize)
    {
      glt_PlaybackRawInjectData_Database.erase(glt_PlaybackRawInjectData_Database.begin(), ite);
    }
  }
}

/**
 * @brief IPC log playback
 * @return None
*/
static void pb_ipc_log_parsed_handler(int8_t decode_type, uint8_t* p_data, uint32_t q_length)
{
  if ((LOG_TYPE_IPC == decode_type) || (LOG_TYPE_CRC_IPC == decode_type))
  {
    ipc_t z_ipc = { 0 };
    memcpy(&z_ipc, p_data, sizeof(ipc_t));
    if (q_length > sizeof(ipc_t))
    {
      z_ipc.p_data = p_data + sizeof(ipc_t);
    }

    pb_ipc_log_parsed_load_assist(z_ipc.t_timestamp_ms);

    /* IPC Process main entry */
    pb_ipc_process(&z_ipc);
  }
  else if (LOG_TYPE_PACKAGE == decode_type || LOG_TYPE_CRC_PACKAGE == decode_type)
  {
    uint32_t length = q_length;

    BOOL u_upgraded = FALSE;
    uint32_t index = 0;
    uint16_t w_PackageId = LOG_PACKAGE_ID_START;

    memcpy(&w_PackageId, p_data, sizeof(uint16_t));
    p_data += sizeof(uint16_t);
    length -= sizeof(uint16_t);

    if (!(w_PackageId > LOG_PACKAGE_ID_START && w_PackageId < LOG_PACKAGE_ID_END))
    {
      return;
    }
    else if (LOG_PACKAGE_ID_TSX_CORR == w_PackageId)
    {
#if defined(APP_HEXAGON_SSR)
      uint32_t q_length_pkg = 0;
      memcpy(&q_length_pkg, p_data, sizeof(uint32_t));
      p_data += sizeof(uint32_t);
      if (q_length_pkg +  sizeof(uint32_t) != length)
      {
        return;
      }
      tsx_injectCorrectionData(q_length_pkg, p_data);
#endif // APP_HEXAGON_SSR
    }
  }
}

static void pb_lever_arm_parsed_handler(int8_t decode_type, uint8_t* p_data, uint32_t q_length)
{
  if (decode_type == LOG_TYPE_CRC_PACKAGE)
  {
    uint32_t q_parse_str_length = 0;
    char parse_str_buffer[5 * 1024] = { 0 };

    q_parse_str_length += package_parse_execute(p_data, q_length,
      parse_str_buffer + q_parse_str_length);
  }
}

/**
 * @brief read lever arm from ipc log
 * @return None
 */
void pb_ag_log_lever_arm_read()
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParam = loc_cfg_getConfigParamGroup();

  if (!(pz_loc_ConfigParam->vdr_cfg.b_enable))
  {
    return;
  }
  gz_pb_config.setting.pb_file_fp = fopen(gz_pb_config.setting.pb_input_file, "rb");
  if (NULL == gz_pb_config.setting.pb_file_fp)
  {
    printf("Open File %s Fail, exit...\n", gz_pb_config.setting.pb_input_file);
    return;
  }

  uint8_t* input_read_buf = (uint8_t*)malloc(READ_BUFFER_SIZE);

  static log_decoder_t pb_log_decoder[2] = { 0 };

  while (!feof(gz_pb_config.setting.pb_file_fp) && get_monitor_flag() == 0)
  {
    if (input_read_buf == NULL)
    {
      break;
    }
    size_t buf_len = fread(input_read_buf, 1, READ_BUFFER_SIZE, gz_pb_config.setting.pb_file_fp);
    if (buf_len < 1)
    {
      break;
    }
    loc_core_log_parse(&pb_log_decoder[0], pb_lever_arm_parsed_handler, input_read_buf, buf_len, TRUE);
  }
  if (get_monitor_flag() == 1)
  {
    loc_MonitorStructType* monitorInfo = get_monitor_info();

    memcpy(pz_loc_ConfigParam->vdr_cfg.u_field, monitorInfo->z_ConfigPara.para[5].u_field,
      sizeof(pz_loc_ConfigParam->vdr_cfg.u_field));
    memcpy(pz_loc_ConfigParam->vdr_cfg.f_field, monitorInfo->z_ConfigPara.para[5].f_field,
      sizeof(pz_loc_ConfigParam->vdr_cfg.f_field));
    vdr_api_Release();
    vdr_api_Init();
    printf("Read Lever arm ok!\n");
  }
  log_fclose_safe(gz_pb_config.setting.pb_file_fp);
  free(input_read_buf);
  return;
}

/**
 * @brief playback with ale format log
 * @return None
 */
void pb_ag_log_playback()
{
  /* parse AG log */
  char progressBar[52] = { 0 };
  const char* label = "|/-\\";
  memset(progressBar, '\0', sizeof(progressBar));
  progressBar[0] = '=';
  clock_t  clockBegin = clock();

  gz_pb_config.setting.pb_file_fp = fopen(gz_pb_config.setting.pb_input_file, "rb");
  if (NULL == gz_pb_config.setting.pb_file_fp)
  {
    printf("Open File %s Fail, exit...\n", gz_pb_config.setting.pb_input_file);
    return;
  }
  uint64_t q_FileSize = pb_get_file_size(gz_pb_config.setting.pb_input_file);
  printf("Load File Size       :  %0.1f KB\n", q_FileSize / 1024.0);

  uint8_t* input_read_buf = (uint8_t*)malloc(READ_BUFFER_SIZE);
  uint64_t t_input_total_len = 0;

  static log_decoder_t pb_log_decoder[2] = { 0 };

  while (!feof(gz_pb_config.setting.pb_file_fp))
  {
    size_t buf_len = fread(input_read_buf,  1, READ_BUFFER_SIZE, gz_pb_config.setting.pb_file_fp);
    if (buf_len < 1)
    {
      break;
    }
    t_input_total_len += buf_len;
    loc_core_log_parse(&pb_log_decoder[0], pb_ipc_log_parsed_handler, input_read_buf, buf_len, TRUE);

    if (gz_pb_config.output.pb_log_progressbar_disable)
    {
      continue;
    }

    /* progress bar show */
    uint8_t rate = (uint32_t)(100.0 * (t_input_total_len) / q_FileSize);
    if (rate % 2 == 0)
    {
      if (rate == 100)
      {
        printf("Finish[%-51s]%2d%%[%0.3fs][%c]\r\n",
          progressBar, rate, (clock() - clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
      }
      else
      {
        printf("Runing[%-51s]%2d%%[%0.3fs][%c]\r",
          progressBar, rate, (clock() - clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
      }
    }
    else
    {
      printf("Runing[%-51s]%2d%%[%0.3fs][%c]\r",
        progressBar, rate, (clock() - clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
    }
    progressBar[rate / 2] = '=';

    fflush(stdout);

  }

#ifdef _WIN32
  if (fp_gpinr)
  {
      fclose(fp_gpinr);
  }
  if (fp_algo)
  {
      fclose(fp_algo);
  }
#endif
  log_fclose_safe(gz_pb_config.setting.pb_file_fp);
  free(input_read_buf);
  return;
}

class pb_stream
{
public:
  pb_stream(const pb_config_t &gz_pb_config);
  ~pb_stream();

public:
  int readFile();
  void pb_rtcm_ubx_log_playback();
  void updateStatus(pb_LocationDataEnumType type);
  void processBar();

private:
  uint8_t* _input_read_buf;
  UbloxDecoder_t *_pz_ubx_decoder_rover;
  RtcmDecoder_t *_pz_rtcm_decoder_rover;
  RtcmDecoder_t *_pz_rtcm_decoder_ref;
  RtcmDecoder_t *_pz_rtcm_decoder_ort;
  assist_file_states_t _assist_file_states;

  /* process bar */
  uint64_t _rtcm_rover_file_size;
  uint64_t _input_total_len;
  clock_t _clockBegin;
  char _progressBar[52];
};

pb_stream::pb_stream(const pb_config_t &gz_pb_config)
{
  /* init */
  _input_read_buf = NULL;
  _pz_ubx_decoder_rover = NULL;
  _pz_rtcm_decoder_rover = NULL;
  _pz_rtcm_decoder_ref = NULL;
  _pz_rtcm_decoder_ort = NULL;
  memset(&_assist_file_states, 0 , sizeof(assist_file_states_t));
  _rtcm_rover_file_size = 0;
  _input_total_len = 0;
  _clockBegin = clock();
  memset(_progressBar, 0, 52 * sizeof(char));

  _input_read_buf = (uint8_t*)malloc(READ_BUFFER_RTCM_SIZE);
  if (NULL!= _input_read_buf)
  {
    memset(_input_read_buf, 0, sizeof(READ_BUFFER_RTCM_SIZE));
  }

  /* input file */
  _assist_file_states.fp_in_file = fopen(gz_pb_config.setting.pb_input_file, "rb");
  if (NULL == _assist_file_states.fp_in_file)
  {
    printf("Open File %s Fail, exit...\n", gz_pb_config.setting.pb_input_file);
    return;
  }
  if( gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_RTCM)
  {
    _pz_rtcm_decoder_rover = (RtcmDecoder_t *)malloc(sizeof(RtcmDecoder_t));
    if (NULL != _pz_rtcm_decoder_rover)
    {
      memset(_pz_rtcm_decoder_rover, 0, sizeof(RtcmDecoder_t));
    }
  }
  else if(gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_UBLOX)
  {
    _pz_ubx_decoder_rover = (UbloxDecoder_t*)malloc(sizeof(UbloxDecoder_t));
    if (NULL != _pz_ubx_decoder_rover)
    {
      memset(_pz_ubx_decoder_rover, 0, sizeof(UbloxDecoder_t));
    }
  }
  else if(gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_RINEX)
  {
    // TODO(houxiaowei) : add rinex play back stream
  }
  uint64_t q_FileSize = pb_get_file_size(gz_pb_config.setting.pb_input_file);
  _rtcm_rover_file_size = q_FileSize;
  printf("input file size:  %0.1f KB\n", q_FileSize / 1024.0);

  /* rtcm ref */
  if (M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_REF_RTCM_VALID))
  {
    _assist_file_states.fp_ref_rtcm = fopen(gz_pb_config.assist.pb_assist_ref_rtcm, "rb");
    if (_assist_file_states.fp_ref_rtcm == NULL)
    {
       printf("Open File %s Fail, exit...\n", gz_pb_config.assist.pb_assist_ref_rtcm);
    }
    else
    {
      _pz_rtcm_decoder_ref = (RtcmDecoder_t *)malloc(sizeof(RtcmDecoder_t));
      if (NULL != _pz_rtcm_decoder_ref)
      {
        memset(_pz_rtcm_decoder_ref, 0, sizeof(RtcmDecoder_t));
      }
      uint64_t q_FileSize = pb_get_file_size(gz_pb_config.assist.pb_assist_ref_rtcm);
      printf("ref rtcm file size  :  %0.1f KB\n", q_FileSize / 1024.0);

    }
  }
  /* rtcm ort */
  if (M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_ORT_RTCM_VALID))
  {
    _assist_file_states.fp_ort_rtcm = fopen(gz_pb_config.assist.pb_assist_ort_rtcm, "rb");
    if (_assist_file_states.fp_ort_rtcm== NULL)
    {
       printf("Open File %s Fail, exit...\n", gz_pb_config.assist.pb_assist_ort_rtcm);
    }
    else
    {
      _pz_rtcm_decoder_ort = (RtcmDecoder_t *)malloc(sizeof(RtcmDecoder_t));
      if (NULL != _pz_rtcm_decoder_ort)
      {
        memset(_pz_rtcm_decoder_ort, 0, sizeof(RtcmDecoder_t));
      }
       uint64_t q_FileSize = pb_get_file_size(gz_pb_config.assist.pb_assist_ort_rtcm);
      printf("ort rtcm file size  :  %0.1f KB\n", q_FileSize / 1024.0);
    }
  }
}

pb_stream::~pb_stream()
{
  safe_free_p(_input_read_buf);
  safe_free_p(_pz_ubx_decoder_rover);
  safe_free_p(_pz_rtcm_decoder_ort);
  safe_free_p(_pz_rtcm_decoder_ref);
  safe_free_p(_pz_rtcm_decoder_rover);

  log_fclose_safe(_assist_file_states.fp_in_file);
  log_fclose_safe(_assist_file_states.fp_ref_rtcm);
  log_fclose_safe(_assist_file_states.fp_ort_rtcm);
}

void pb_stream::processBar()
{
  const char *label = "|/-\\";

  uint8_t rate = (uint32_t)(100.0 * (_input_total_len - _assist_file_states.num_rtcm) / _rtcm_rover_file_size);
  for (uint8_t i = 0; i < rate / 2; i++)
  {
    _progressBar[i] = '=';
  }
  if (rate % 2 == 0)
  {
    if (rate == 100)
    {
      printf("Finish[%-51s]%2d%%[%0.3fs][%c]\r\n",
             _progressBar, rate, (clock() - _clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
    }
    else
    {
      printf("Runing[%-51s]%2d%%[%0.3fs][%c]\r",
             _progressBar, rate, (clock() - _clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
    }
  }
  else
  {
    printf("Runing[%-51s]%2d%%[%0.3fs][%c]\r",
           _progressBar, rate, (clock() - _clockBegin) / (double)CLOCKS_PER_SEC, label[rate % 4]);
  }

  fflush(stdout);
}

int pb_stream::readFile()
{
  int status = 0;
  size_t buf_len = 0;

  if (_assist_file_states.fp_in_file != NULL && !feof(_assist_file_states.fp_in_file) && _assist_file_states.num_rtcm <= 0)
  {
    buf_len = fread(_input_read_buf, 1, READ_BUFFER_RTCM_SIZE, _assist_file_states.fp_in_file);
    if( gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_RTCM)
    {
      _assist_file_states.num_rtcm = pb_assist_decode_rtcm(_pz_rtcm_decoder_rover, _input_read_buf, buf_len, PB_LOCATION_DATA_RCV_MEAS, &_assist_file_states);
    }
    else if(gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_UBLOX)
    {
      _assist_file_states.num_rtcm = pb_assist_load_ublox_stream(_pz_ubx_decoder_rover, _input_read_buf, buf_len, &_assist_file_states);
    }
    _assist_file_states.unm_rtcm_back = _assist_file_states.num_rtcm;
    printf("decode input file %.1f KB \n", (double)buf_len / 1024.0);
    _input_total_len += buf_len;
    status = 1;
  }

  if (_assist_file_states.fp_ref_rtcm != NULL && !feof(_assist_file_states.fp_ref_rtcm) && _assist_file_states.num_ref_rtcm <= 0)
  {
    buf_len = fread(_input_read_buf, 1, READ_BUFFER_RTCM_SIZE, _assist_file_states.fp_ref_rtcm);
    _assist_file_states.num_ref_rtcm = pb_assist_decode_rtcm(_pz_rtcm_decoder_ref, _input_read_buf, buf_len, PB_LOCATION_DATA_REF_CORR, &_assist_file_states);
    printf("decode ref rtcm %.1f KB \n", (double)buf_len / 1024.0);
    status = 1;
  }

  if (_assist_file_states.fp_ort_rtcm != NULL && !feof(_assist_file_states.fp_ort_rtcm) && _assist_file_states.num_ort_rtcm <= 0)
  {
    buf_len = fread(_input_read_buf, 1, READ_BUFFER_RTCM_SIZE, _assist_file_states.fp_ort_rtcm);
    _assist_file_states.num_ort_rtcm = pb_assist_decode_rtcm(_pz_rtcm_decoder_ort, _input_read_buf, buf_len, PB_LOCATION_DATA_ORT_MEAS, &_assist_file_states);
    printf("decode ort rtcm %.1f KB \n", (double)buf_len / 1024.0);
    status = 1;
  }

  if (1 == status)
  {
    pb_database_sequence();
  }

  /* progress bar show */
  processBar();

  return status;
}

void pb_stream::updateStatus(pb_LocationDataEnumType type)
{
  switch (type)
  {
  case PB_LOCATION_DATA_RCV_MEAS:
    _assist_file_states.num_rtcm--;
    break;
  case PB_LOCATION_DATA_REF_CORR:
    _assist_file_states.num_ref_rtcm--;
    break;
  case PB_LOCATION_DATA_ORT_MEAS:
    _assist_file_states.num_ort_rtcm--;
    break;
  default:
    break;
  }
}

/******************************************************************************
           Rover RTCM database load
******************************************************************************/
/**
 * @brief playback assist module, load rover block using RTCM format
 * @param[in] filename - input filename
 * @return None
 */
void pb_stream::pb_rtcm_ubx_log_playback()
{
  int status = 0;

  status = readFile();

  if(0 == status)
  {
    printf("Error\n");
    return;
  }
  do
  {
    if (feof(_assist_file_states.fp_in_file) && _assist_file_states.num_rtcm <= 0)
    {
      break;
    }

    for (std::list<PlaybackDataUnit>::iterator ite = glt_PlaybackRawInjectData_Database.begin();
         ite != glt_PlaybackRawInjectData_Database.end();)
    {
      if (PB_LOCATION_DATA_BDDB_IMU == ite->u_type)
      {
        loc_api_InjectImuDataToIns((uint8_t *)ite->pz_ImuData, sizeof(loc_api_imu_data_t));
      }
      else if (PB_LOCATION_DATA_BDDB_WHEEL == ite->u_type)
      {
        loc_api_InjectWheelDataToIns((uint8_t *)ite->pz_WheelData, sizeof(loc_api_wheel_t));
      }
      else if (PB_LOCATION_DATA_REF_CORR == ite->u_type)
      {
        loc_api_InjectRefCorrMeasBlk((uint8_t *)ite->pz_CorrBlock, sizeof(GnssCorrBlock_t));
      }
      else if (PB_LOCATION_DATA_ORT_MEAS == ite->u_type)
      {
        loc_api_InjectOrtMeasBlock((uint8_t *)ite->pz_OrtMeasBlock, sizeof(GnssMeasBlock_t));
      }
      else if (PB_LOCATION_DATA_RCV_MEAS == ite->u_type)
      {
        loc_api_InjectRcvMeasBlock((uint8_t *)ite->pz_RcvMeasBlock, sizeof(GnssMeasBlock_t));
      }
      else if (PB_LOCATION_DATA_EPH == ite->u_type)
      {
        loc_api_InjectEphemeris((uint8_t *)ite->pz_Ephemeris, sizeof(gnss_Ephemeris_t));
      }
      free_PlaybackDataUnit(ite);
      updateStatus(ite->u_type);
      ite = glt_PlaybackRawInjectData_Database.erase(ite);
      status = readFile();
      if (0 == status)
      {
        break;
      }
    }
  } while (true);
}

/**
 * @brief playback with rinex format file
 * @return None
 */
void pb_rinex_log_playback()
{

  pb_assist_load_rover_rinex(gz_pb_config.setting.pb_input_file);

  pb_database_sequence();

  for (std::list<PlaybackDataUnit>::iterator ite = glt_PlaybackRawInjectData_Database.begin();
    ite != glt_PlaybackRawInjectData_Database.end();
    ite++)
  {
    switch (ite->u_type)
    {
    case PB_LOCATION_DATA_BDDB_IMU:
      loc_api_InjectImuDataToIns((uint8_t*)ite->pz_ImuData, sizeof(loc_api_imu_data_t));
      break;
    case PB_LOCATION_DATA_BDDB_GNSS:
      loc_api_InjectGnssFixToIns((uint8_t*)ite->pz_GnssTypeBDDB10, sizeof(GnssFixType));
      break;
    case PB_LOCATION_DATA_REF_CORR:
      loc_api_InjectRefCorrMeasBlk((uint8_t*)ite->pz_CorrBlock, sizeof(GnssCorrBlock_t));
      break;
    case PB_LOCATION_DATA_ORT_MEAS:
      loc_api_InjectOrtMeasBlock((uint8_t*)ite->pz_OrtMeasBlock, sizeof(GnssMeasBlock_t));
      break;
    case PB_LOCATION_DATA_RCV_MEAS:
      loc_api_InjectRcvMeasBlock((uint8_t*)ite->pz_RcvMeasBlock, sizeof(GnssMeasBlock_t));
      break;
    case PB_LOCATION_DATA_EPH:
      loc_api_InjectEphemeris((uint8_t*)ite->pz_Ephemeris, sizeof(gnss_Ephemeris_t));
      break;
    default:
      break;
    }
  }

  return;
}

/**
 * @brief convert GNSS process inteval to mask
 * @param[in] mask 
 * @return mask LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_**
 */
uint64_t pb_interval2IntervalMask(float interval)
{
  uint64_t u_mask = 0;
  uint8_t u_inteval = (uint8_t)round(interval * 10);

  if (u_inteval > 10 || u_inteval < 1)
  {
    return LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_10;
  }

  if (1 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_1;
  }
  else if (2 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_2;
  }
  else if (3 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_3;
  }
  else if (4 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_4;
  }
  else if (5 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_5;
  }
  else if (6 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_6;
  }
  else if (7 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_7;
  }
  else if (8 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_8;
  }
  else if (9 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_9;
  }
  else if (10 == u_inteval)
  {
    u_mask |= LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_10;
  }

  return u_mask;
}

/**
 * @brief playback with bddb format file
 * @return None
 */
void pb_bddb_log_playback()
{

  //pb_database_sequence();

  for (std::list<PlaybackDataUnit>::iterator ite = glt_PlaybackRawInjectData_Database.begin();
    ite != glt_PlaybackRawInjectData_Database.end();
    ite++)
  {
    switch (ite->u_type)
    {
    case PB_LOCATION_DATA_BDDB_IMU:
      loc_api_InjectImuDataToIns((uint8_t*)ite->pz_ImuData, sizeof(loc_api_imu_data_t));
      break;
    case PB_LOCATION_DATA_BDDB_GNSS:
      loc_api_InjectGnssFixToIns((uint8_t*)ite->pz_GnssTypeBDDB10, sizeof(GnssFixType));
      break;
    case PB_LOCATION_DATA_BDDB_WHEEL:
      loc_api_InjectWheelDataToIns((uint8_t*)ite->pz_WheelData, sizeof(loc_api_wheel_t));
      break;
    default:
      break;
    }
  }

  return;
}

/**
 * @brief Command parse
 * @param[in]   argc
 * @param[in]   argv
 * @return:
 */
static optparse::Values cmd_options;
void command_parse(int argc, char* argv[])
{
  char version_str[256] = { 0 };
  get_version(version_str);

  const string description = "Location Engine Log Playback";
  const string usage = "usage: %prog [OPTION]...";
  optparse::OptionParser parser = optparse::OptionParser()
    .usage(usage)
    .version(description + "\n" + version_str);

  parser.add_option("-f", "--file")
    .action("store")
    .type("string")
    .help("playback data file\n");

  parser.add_option("-r", "--format")
    .action("store")
    .type("string")
    .set_default("ag")
    .help("playback data format default:%default\n"
      "ag     : ag format .\n"
      "rtcm   : rtcm data format.\n"
      "rinex  : rinex data format.\n"
      "no     : depend on config.ini setting.\n");

  parser.add_option("-o", "--output_dir")
    .action("store")
    .type("string")
    .help("playback output dir\n");

  parser.add_option("-c", "--config")
    .action("store")
    .type("string")
    .help("configuration file\n");

  parser.add_option("-m", "--mode")
    .action("store")
    .type("string")
    .set_default("hpp_ppp")
    .help("playback mode default:%default\n"
      "hpp    : hpp only playback.\n"
      "ppp    : ppp only playback.\n"
      "dcp    : dcp only playback.\n"
      "hpp_ppp: hpp ppp union playback.\n"
      "no     : depend on config.ini setting.\n");

  parser.add_option("-s", "--sample")
    .action("store")
    .type("int")
    .help("rtcm/observation sample\n"
      "suport 1/5/10 [HZ]\n"
      "default 5 [HZ]\n");

  parser.add_option("--disable_bar")
    .action("count")
    .help("disable progress bar\n");

  parser.print_version();
  parser.print_help();
  parser.print_args();

  cmd_options = parser.parse_args(argc, argv);
}

/**
 * @brief Location Engine Simulation Main Process
 * @param[in]   argc
 * @param[in]   argv
 * @return:
 */
int main(int argc, char* argv[])
{
  /****************************************************************************
       command parse
  ****************************************************************************/
  command_parse(argc, argv);

  /****************************************************************************
                   loc core memory init
  ****************************************************************************/
  loc_api_MemoryRegister_t z_MemoryRegister = { 0 };
  static uint8_t memory_pool0[1024 * 1024 * 3] = { 0 };
  z_MemoryRegister.u_type = LOC_API_MEMORY_EXT_POOL;
  z_MemoryRegister.pool_addr[0] = &(memory_pool0[0]);
  z_MemoryRegister.pool_size[0] = sizeof(memory_pool0);
  z_MemoryRegister.pool_count = 1;
  z_MemoryRegister.alloc = loc_core_callback_mem_alloc_handler;
  z_MemoryRegister.free = loc_core_callback_mem_free_handler;
  loc_api_Register_MemoryPool(&z_MemoryRegister);
  
  /****************************************************************************
                     Load Configuration
  ****************************************************************************/
  if (cmd_options.is_set(string("config")))
  {
    if (ini_parse(cmd_options["config"].c_str(), handler, &gz_pb_config) < 0) {
      printf("Can't load <config>.ini\n");
      return 0;
    }
  }
  else
  {
    if (cmd_options.is_set(string("file")))
    {
      memset(gz_pb_config.setting.pb_input_file, 0, sizeof(gz_pb_config.setting.pb_input_file));
      memcpy(gz_pb_config.setting.pb_input_file, cmd_options["file"].c_str(), cmd_options["file"].length());

      if (0 != cmd_options["format"].compare("no"))
      {
        if (0 == cmd_options["format"].compare("ag"))
        {
          gz_pb_config.setting.pb_input_file_type = PB_INPUT_FILE_ALE;
        }
        else if (0 == cmd_options["format"].compare("rtcm"))
        {
          gz_pb_config.setting.pb_input_file_type = PB_INPUT_FILE_RTCM;
        }
        else if (0 == cmd_options["format"].compare("rinex"))
        {
          gz_pb_config.setting.pb_input_file_type = PB_INPUT_FILE_RINEX;
        }
        gz_pb_config.setting.pb_file_format = pb_convert_file_format_to_str(gz_pb_config.setting.pb_input_file_type);
      }
    }

    if (cmd_options.is_set(string("output_dir")))
    {
      memset(gz_pb_config.output.pb_output_dir, 0, sizeof(gz_pb_config.output.pb_output_dir));
      memcpy(gz_pb_config.output.pb_output_dir, cmd_options["output_dir"].c_str(), cmd_options["output_dir"].length());
    }

    gz_pb_config.output.pb_log_progressbar_disable = (uint8_t)(int)cmd_options.get("disable_bar");

    // auto detect interval
    float u_auto_interval = 0.0f;
    u_auto_interval = pb_parseProcessInterval(gz_pb_config.setting.pb_input_file); 
    if (u_auto_interval != 0)
    {
      gz_pb_config.setting.pb_meas_update_interval = u_auto_interval;
    }

    // auto detect process mode RTK or PPP or RTK/PPP
    pb_parseProcessMode(gz_pb_config.setting.pb_input_file);

    if (cmd_options.is_set(string("mode")))
    {
      uint8_t count = 0;
      uint8_t count_live = 0;
      
      pb_config_filter_t* pz_filter = &(gz_pb_config.filter);
      pz_filter->pb_filter_pass_log_ipc[count++] = C_M_HPP_INIT;
      pz_filter->pb_filter_pass_log_ipc[count++] = C_M_PPP_INIT;
      pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_INIT;
      pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_INIT;
      if (0 == cmd_options["mode"].compare("hpp"))
      {
        gz_pb_config.core.pb_loc_core_sm_task_enable  = 1;
        gz_pb_config.core.pb_loc_core_hpp_task_enable = 1;
        gz_pb_config.core.pb_loc_core_hpp_pvt_enable  = 1;
        gz_pb_config.core.pb_loc_core_hpp_rtk_enable  = 1;
        gz_pb_config.core.pb_loc_core_dcp_task_enable = 1;
        gz_pb_config.core.pb_loc_core_ppp_task_enable = 0;
        gz_pb_config.core.pb_loc_core_sd_task_enable  = 1;
        gz_pb_config.core.pb_loc_core_vdr_task_enable = 0;

        memset(&pz_filter->pb_filter_pass_log_ipc[0], 0,
          sizeof(pz_filter->pb_filter_pass_log_ipc));
        memset(&pz_filter->pb_filter_pass_live_ipc[0], 0,
          sizeof(pz_filter->pb_filter_pass_live_ipc));
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_START;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_RCV_MEAS_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_REPORT_POS_FIX;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SD_SAT_PVT_POLY_REQUEST;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_RCV_MEAS_NHZ_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_TDCP_MEAS_PUT;
      }
      else if (0 == cmd_options["mode"].compare("ppp"))
      {
        gz_pb_config.core.pb_loc_core_sm_task_enable  = 1;
        gz_pb_config.core.pb_loc_core_hpp_task_enable = 0;
        gz_pb_config.core.pb_loc_core_dcp_task_enable = 0;
        gz_pb_config.core.pb_loc_core_ppp_task_enable = 1;
        gz_pb_config.core.pb_loc_core_sd_task_enable  = 0;
        gz_pb_config.core.pb_loc_core_vdr_task_enable = 0;
        memset(&pz_filter->pb_filter_pass_log_ipc[0], 0,
          sizeof(pz_filter->pb_filter_pass_log_ipc));
        memset(&pz_filter->pb_filter_pass_live_ipc[0], 0,
          sizeof(pz_filter->pb_filter_pass_live_ipc));

        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_START;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_RCV_MEAS_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_REPORT_POS_FIX;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_RCV_OBS_RTCM;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_REF_CORR_RTCM;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_RCV_MEAS_NHZ_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_TDCP_MEAS_PUT;

        count = 0;
        pz_filter->pb_filter_pass_live_ipc[count++] = C_M_PPP_SSR_LOS_BLK;

      }
      else if (0 == cmd_options["mode"].compare("dcp"))
      {
        gz_pb_config.core.pb_loc_core_sm_task_enable = 1;
        gz_pb_config.core.pb_loc_core_hpp_task_enable = 0;
        gz_pb_config.core.pb_loc_core_dcp_task_enable = 1;
        gz_pb_config.core.pb_loc_core_ppp_task_enable = 0;
        gz_pb_config.core.pb_loc_core_sd_task_enable = 0;
        gz_pb_config.core.pb_loc_core_vdr_task_enable = 0;
        memset(&pz_filter->pb_filter_pass_log_ipc[0], 0, sizeof(pz_filter->pb_filter_pass_log_ipc));
        memset(&pz_filter->pb_filter_pass_live_ipc[0], 0, sizeof(pz_filter->pb_filter_pass_live_ipc));

        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_START;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_RCV_MEAS_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_REPORT_POS_FIX;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_RCV_OBS_RTCM;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_REF_CORR_RTCM;
      }
      else if (0 == cmd_options["mode"].compare("hpp_ppp"))
      {
        gz_pb_config.core.pb_loc_core_sm_task_enable = 1;
        gz_pb_config.core.pb_loc_core_hpp_task_enable = 1;
        gz_pb_config.core.pb_loc_core_hpp_pvt_enable = 1;
        gz_pb_config.core.pb_loc_core_hpp_rtk_enable = gz_pb_config_log.core.pb_loc_core_hpp_rtk_enable;
        gz_pb_config.core.pb_loc_core_dcp_task_enable = 1;
        gz_pb_config.core.pb_loc_core_ppp_task_enable = gz_pb_config_log.core.pb_loc_core_ppp_task_enable;
        gz_pb_config.core.pb_loc_core_sd_task_enable = 1;
        gz_pb_config.core.pb_loc_core_vdr_task_enable = 0;
        memset(&pz_filter->pb_filter_pass_log_ipc[0], 0,
          sizeof(pz_filter->pb_filter_pass_log_ipc));
        memset(&pz_filter->pb_filter_pass_live_ipc[0], 0,
          sizeof(pz_filter->pb_filter_pass_live_ipc));


        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_START;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_RCV_MEAS_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SM_REPORT_POS_FIX;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_SD_SAT_PVT_POLY_REQUEST;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_RCV_MEAS_NHZ_PUT;
        pz_filter->pb_filter_pass_log_ipc[count++] = C_M_DCP_TDCP_MEAS_PUT;

        pz_filter->pb_filter_pass_live_ipc[count_live++] = C_M_PPP_SSR_LOS_BLK;

        if ((cmd_options.is_set(string("sample"))) && 
        ((u_auto_interval == 0) || ((int)round(1.0 / u_auto_interval) != atoi(cmd_options["sample"].c_str())))
        )
        {
          pz_filter->pb_filter_pass_log_ipc[count++] = C_M_PPP_ALGO_START;
          gz_pb_config.setting.pb_meas_update_interval = (float)(1.0f / atof(cmd_options["sample"].c_str()));
        }
        else
        {
          pz_filter->pb_filter_pass_live_ipc[count_live++] = C_M_PPP_ALGO_START;
        }
      }
      gz_pb_config.core.pb_loc_core_log_task_enable = 1;
    }
  }
 
  if (0 == strlen(gz_pb_config.setting.pb_input_file))
  {
    printf("The input file is empty, exit...\n");
    return 0;
  }

  /****************************************************************************
                       Playback file information
  ****************************************************************************/
  char filename[256] = { 0 };
  char input_filename[256] = { 0 };
  struct timespec ts;
  int n = 0;
  int timespec_get_succ = timespec_get(&ts, TIME_UTC);
  int pb_output_dir_length = strlen(gz_pb_config.output.pb_output_dir);
  if (pb_output_dir_length > 0)
  {
    if (gz_pb_config.output.pb_output_dir[pb_output_dir_length - 1] != '/')
    {
      gz_pb_config.output.pb_output_dir[pb_output_dir_length] = '/';
    }
#ifdef _MSC_VER
    if (_access(gz_pb_config.output.pb_output_dir, 0) == -1)
    {
      if (-1 == _mkdir(gz_pb_config.output.pb_output_dir))
      {
        printf("playback exit: mkdir output dir %s fail\n", gz_pb_config.output.pb_output_dir);
        return 0;
      }
    }
#else
    if (access(gz_pb_config.output.pb_output_dir, 0) == -1)
    {
      if (-1 == mkdir(gz_pb_config.output.pb_output_dir, 0777))
      {
        printf("playback exit: mkdir output dir %s fail\n", gz_pb_config.output.pb_output_dir);
        return 0;
      }
    }
#endif
  }
  
  char* p1 = strrchr(gz_pb_config.setting.pb_input_file, '\\');
  char* p2 = strrchr(gz_pb_config.setting.pb_input_file, '/');

  if (NULL == p1 && NULL == p2)
  {
    memcpy(input_filename, gz_pb_config.setting.pb_input_file, sizeof(gz_pb_config.setting.pb_input_file));
  }
  else if (NULL != p1)
  {
    memcpy(input_filename, p1 + 1, strlen(p1 + 1));
  }
  else if (NULL != p2)
  {
    memcpy(input_filename, p2 + 1, strlen(p2 + 1));
  }

  p1 = strrchr(input_filename, '.');
  if (NULL != p1)
  {
    memset(p1, 0, strlen(p1));
  }
  // pb_output_file_mask default
  if(gz_pb_config.output.pb_output_file_set_valid == 0)
  {
    gz_pb_config.output.pb_output_file_mask = PB_OUTPUT_FILE_CONTROL_BIN | PB_OUTPUT_FILE_CONTROL_TEXT;
  }
  // AG file
  if (M_IS_SET_MASK(gz_pb_config.output.pb_output_file_mask, PB_OUTPUT_FILE_CONTROL_BIN))
  {
    memset(filename, 0, sizeof(filename));
    n = 0;
    n += sprintf(filename + n, "%s", gz_pb_config.output.pb_output_dir);
    n += sprintf(filename + n, "%s-pb-ag", input_filename);
    if (gz_pb_config.output.pb_log_suffix_by_time && timespec_get_succ)
    {
      n += (int)strftime(filename + n, sizeof(filename), "%Y%m%d-%H%M%S", gmtime(&ts.tv_sec));
    }
    n += sprintf(filename + n, ".bin");
    memcpy(gz_pb_config.output.pb_output_ag_path, filename, sizeof(filename));
    gz_pb_config.output.pb_output_ag_fp = fopen(filename, "wb+");
  }
  // txt file
  if (M_IS_SET_MASK(gz_pb_config.output.pb_output_file_mask, PB_OUTPUT_FILE_CONTROL_TEXT))
  {
    memset(filename, 0, sizeof(filename));
    n = 0;
    n += sprintf(filename + n, "%s", gz_pb_config.output.pb_output_dir);
    n += sprintf(filename + n, "%s-pb-text", input_filename);
    if (gz_pb_config.output.pb_log_suffix_by_time && timespec_get_succ)
    {
      n += (int)strftime(filename + n, sizeof(filename), "%Y%m%d-%H%M%S", gmtime(&ts.tv_sec));
    }
    n += sprintf(filename + n, ".log");
    memcpy(gz_pb_config.output.pb_output_text_path, filename, sizeof(filename));
    gz_pb_config.output.pb_output_text_fp = fopen(filename, "w+");
  }

  // nmea rtcm
  ipc_parse_config_output_file((char*)"", gz_pb_config.output.pb_output_dir, (char*)"", input_filename);

  /****************************************************************************
                          Data Log Init
  ****************************************************************************/
  DataLogManagerConfig_t z_DataLogManagerConfig = { 0 };
  memcpy(z_DataLogManagerConfig.pb_output_dir, gz_pb_config.output.pb_output_dir, sizeof(z_DataLogManagerConfig.pb_output_dir));
  z_DataLogManagerConfig.u_Enable = gz_pb_config.datalog.u_Enable;
  for (uint8_t u_i = 0; u_i < DLOG_MAX; u_i++)
  {
    memcpy(z_DataLogManagerConfig.p_DataLogFilepath[u_i],
      gz_pb_config.datalog.p_DataLogFilepath[u_i], 
      sizeof(gz_pb_config.datalog.p_DataLogFilepath[u_i]));
  }
  log_DataLogManager_Initialize(&z_DataLogManagerConfig);

#if defined(APP_HEXAGON_SSR)
  tsx_adapter_init(gz_pb_config.assist.pb_assist_tsx_dcf);
  tsx_print_version();
#endif

  /****************************************************************************
                   loc core callback init
  ****************************************************************************/
  loc_api_callback_t loc_api_callback = {0};
  loc_api_callback.report_log = pb_log_callback_handler;
  loc_api_callback.report_location = pb_report_location_callback_handler;
  loc_api_callback.report_orient = pb_report_orient_callback_handler;
  loc_api_callback.report_consolidated_location = pb_report_consolidated_location_callback_handler;
  loc_api_callback.get_tick_ms = loc_core_callback_get_tick_ms;
  loc_api_callback.report_gnss_measurement_satellite = loc_core_callback_report_gnss_meas_satpos;
#if defined(APP_HEXAGON_SSR)
  loc_api_callback.report_gnss_navigation_data = tsx_inject_navigation_data;
#endif
  loc_api_Register_Callback(&loc_api_callback);

  /****************************************************************************
                          Module Init
  ****************************************************************************/
  loc_api_config_para_group_t z_loc_api_config = { 0 };
  char time_str[256] = { 0 };
  strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", gmtime(&ts.tv_sec));

  char pb_task_enable_mask_str[256] = {0};
  n = 0;
  if (gz_pb_config.core.pb_loc_core_sm_task_enable) 
  {
    n += sprintf(pb_task_enable_mask_str + n, "| SM ");
    M_SET_MASK(z_loc_api_config.para[0].t_mask, LOC_API_CFG_MASK_0_SM_ENABLE);
    M_SET_MASK(z_loc_api_config.para[0].t_mask, LOC_API_CFG_MASK_0_SM_RCV_RTCM_DEC_ENABLE);
    M_SET_MASK(z_loc_api_config.para[0].t_mask, LOC_API_CFG_MASK_0_SM_REF_RTCM_DEC_ENABLE);

    z_loc_api_config.para[0].t_mask |= pb_interval2IntervalMask(gz_pb_config.setting.pb_meas_update_interval);
  }
  
  if (M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_NAV_RINEX_VALID))
  {
    M_SET_MASK(z_loc_api_config.para[0].t_mask, LOC_API_CFG_MASK_0_SM_DECODE_RTCM_NAV_DISABLE);
  }

  if (gz_pb_config.core.pb_loc_core_hpp_task_enable) {
    n += sprintf(pb_task_enable_mask_str + n, "| HPP ");
    M_SET_MASK(z_loc_api_config.para[1].t_mask, LOC_API_CFG_MASK_1_HPP_ENABLE);

    if (gz_pb_config.core.pb_loc_core_hpp_pvt_enable) {
      M_SET_MASK(z_loc_api_config.para[1].t_mask, LOC_API_CFG_MASK_1_HPP_PVT_ENABLE);
    }

    if (gz_pb_config.core.pb_loc_core_hpp_rtk_enable) {
      M_SET_MASK(z_loc_api_config.para[1].t_mask, LOC_API_CFG_MASK_1_HPP_RTK_ENABLE);
    }
    memcpy(z_loc_api_config.para[1].u_field, gz_pb_config.core.para[1].u_field,
      sizeof(z_loc_api_config.para[1].u_field));
    z_loc_api_config.para[1].f_field[1] = (float)gz_pb_config.setting.pb_prefix_diffage;
  }

  if (gz_pb_config.core.pb_loc_core_dcp_task_enable) {
    n += sprintf(pb_task_enable_mask_str + n, "| DCP ");
    M_SET_MASK(z_loc_api_config.para[2].t_mask, LOC_API_CFG_MASK_2_DCP_ENABLE);
  }
  
  if (gz_pb_config.core.pb_loc_core_ppp_task_enable) {
    n += sprintf(pb_task_enable_mask_str + n, "| PPP ");
    M_SET_MASK(z_loc_api_config.para[3].t_mask, LOC_API_CFG_MASK_3_PPP_ENABLE);
  }
  
  if (gz_pb_config.core.pb_loc_core_sd_task_enable) {
    n += sprintf(pb_task_enable_mask_str + n, "| SD ");
    M_SET_MASK(z_loc_api_config.para[4].t_mask, LOC_API_CFG_MASK_4_SD_ENABLE);
  }
  
  if (gz_pb_config.core.pb_loc_core_vdr_task_enable) {
    n += sprintf(pb_task_enable_mask_str + n, "| VDR ");
    M_SET_MASK(z_loc_api_config.para[5].t_mask, LOC_API_CFG_MASK_5_VDR_ENABLE);
  }

  if (gz_pb_config.core.pb_loc_core_ort_task_enable)
  {
    n += sprintf(pb_task_enable_mask_str + n, "| ORT ");
    M_SET_MASK(z_loc_api_config.para[2].t_mask, LOC_API_CFG_MASK_2_ORT_ENABLE);
  }

  z_loc_api_config.para[6].t_mask = 
    LOC_API_CFG_MASK_6_LOG_ENABLE |
    LOC_API_CFG_MASK_6_LOG_TEXT_OUTPUT_IPC |
    LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE1 |
    LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE2;

  if(M_IS_SET_MASK(gz_pb_config.output.pb_output_file_mask, PB_OUTPUT_FILE_CONTROL_TEXT))
  {
    // log
    switch (gz_pb_config.output.pb_log_level)
    {
    case LOG_LEVEL_E:
      z_loc_api_config.para[6].t_mask |= LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_ERROR;
      break;
    case LOG_LEVEL_W:
      z_loc_api_config.para[6].t_mask |= LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_WARNING;
      break;
    case LOG_LEVEL_I:
      z_loc_api_config.para[6].t_mask |= LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_INFO;
      break;
    case LOG_LEVEL_D:
      z_loc_api_config.para[6].t_mask |= LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_DEBUG;
      break;
    default:
      z_loc_api_config.para[6].t_mask |= LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_INFO;
      break;
    }
  }

  for(uint8_t u_i = 0; u_i <  M_MIN(M_ARRAY_SIZE(z_loc_api_config.para), M_ARRAY_SIZE(gz_pb_config.core.para)); u_i++)
  {
    uint8_t u_size_u = M_MIN(M_ARRAY_SIZE(z_loc_api_config.para[u_i].u_field), M_ARRAY_SIZE(gz_pb_config.core.para[u_i].u_field));
    memcpy(z_loc_api_config.para[u_i].u_field, gz_pb_config.core.para[u_i].u_field, sizeof(uint8_t) * u_size_u);
    uint8_t u_size_f = M_MIN(M_ARRAY_SIZE(z_loc_api_config.para[u_i].f_field), M_ARRAY_SIZE(gz_pb_config.core.para[u_i].f_field));
    memcpy(z_loc_api_config.para[u_i].f_field, gz_pb_config.core.para[u_i].f_field, sizeof(float) * u_size_f);
  }

  z_loc_api_config.para[7].t_mask = LOC_API_CFG_MASK_7_IPC_SINGLE;

  loc_api_Register_Config(&z_loc_api_config);

  /****************************************************************************
                    ipc filter setting
  ****************************************************************************/
  ipc_filter_t ipc_filter = { 0 };
  pb_config_filter_t* pz_pb_filter = &(gz_pb_config.filter);
  uint32_t len1 = 0;
  uint32_t len2 = 0;
  uint32_t len = 0;

  ipc_filter.task_enable_list[TASK_INDEX_SM]  = gz_pb_config.core.pb_loc_core_sm_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_HPP] = gz_pb_config.core.pb_loc_core_hpp_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_DCP] = gz_pb_config.core.pb_loc_core_dcp_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_PPP] = gz_pb_config.core.pb_loc_core_ppp_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_SD]  = gz_pb_config.core.pb_loc_core_sd_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_VDR] = gz_pb_config.core.pb_loc_core_vdr_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_LOG] = gz_pb_config.core.pb_loc_core_log_task_enable;
  ipc_filter.task_enable_list[TASK_INDEX_ORT] = gz_pb_config.core.pb_loc_core_ort_task_enable;

  len1 = sizeof(pz_pb_filter->pb_filter_pass_log_ipc) / sizeof(pz_pb_filter->pb_filter_pass_log_ipc[0]);
  len2 = sizeof(ipc_filter.filter_pass_log_ipc) / sizeof(ipc_filter.filter_pass_log_ipc[0]);
  len = len1 < len2 ? len1 : len2;
  for (uint8_t u_i = 0; u_i < len; u_i++)
  {
    ipc_filter.filter_pass_log_ipc[u_i] = pz_pb_filter->pb_filter_pass_log_ipc[u_i];
  }

  len1 = sizeof(pz_pb_filter->pb_filter_pass_live_ipc) / sizeof(pz_pb_filter->pb_filter_pass_live_ipc[0]);
  len2 = sizeof(ipc_filter.filter_pass_live_ipc) / sizeof(ipc_filter.filter_pass_live_ipc[0]);
  len = len1 < len2 ? len1 : len2;
  for (uint8_t u_i = 0; u_i < len; u_i++)
  {
    ipc_filter.filter_pass_live_ipc[u_i] = pz_pb_filter->pb_filter_pass_live_ipc[u_i];
  }

  ipctask_ipc_filter_set(&ipc_filter);

  loc_api_Initialize();

  /****************************************************************************
                      Start running
  ****************************************************************************/

  printf("Playback Start %s\n", time_str);
  printf("Input File Path     :  %s \n", gz_pb_config.setting.pb_input_file);
  printf("Input File Format   :  %s \n", gz_pb_config.setting.pb_file_format);
  printf("PB Feature Mask     :  %s \n", pb_task_enable_mask_str);
  printf("Output Dir          :  %s \n", gz_pb_config.output.pb_output_dir);
  printf("Output AG           :  %s \n", gz_pb_config.output.pb_output_ag_path);
  printf("Output Text         :  %s \n", gz_pb_config.output.pb_output_text_path);
  printf("Assist Feature Mask :  0x%x \n", gz_pb_config.assist.pb_assist_file_valid_mask);
  printf("Assist Nav Rinex    :  %s \n", gz_pb_config.assist.pb_assist_nav_rinex);
  printf("Assist Nav RTCM     :  %s \n", gz_pb_config.assist.pb_assist_nav_rtcm);
  printf("Assist Ref Rinex    :  %s \n", gz_pb_config.assist.pb_assist_ref_rinex);
  printf("Assist Ref RTCM     :  %s \n", gz_pb_config.assist.pb_assist_ref_rtcm);
  printf("Assist IMU AG BDDB  :  %s \n", gz_pb_config.assist.pb_assist_imu_ag_bddb);
  printf("Assist Ort RTCM     :  %s \n", gz_pb_config.assist.pb_assist_ort_rtcm);

#ifdef FEATURE_PROFILER
  ProfilerStart("profiler_capture.prof");
#endif

  if (M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_REF_RINEX_VALID))
  {
    pb_assist_load_ref_rinex(gz_pb_config.assist.pb_assist_ref_rinex);
  }

  if (M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_NAV_RINEX_VALID))
  {
    pb_assist_load_eph_rinex(gz_pb_config.assist.pb_assist_nav_rinex);
  }

  if (M_IS_SET_MASK(gz_pb_config.assist.pb_assist_file_valid_mask, PB_ASSIST_IMU_AG_BDDB_VALID))
  {
    pb_assist_load_imu_ag_bddb(gz_pb_config.assist.pb_assist_imu_ag_bddb);
  }
  if (gz_pb_config.truth.pb_truth_mode == PB_TRUTH_MODE_DYNAMIC_POS320)
  {
    pb_truth_load_reference_file(gz_pb_config.truth.pb_truth_dynamic_file);
  }

  if (PB_TRUTH_MODE_DYNAMIC_NMEA == gz_pb_config.truth.pb_truth_mode)
  {
    pb_truth_load_dynamic_gga(gz_pb_config.truth.pb_truth_dynamic_file);
  }

  if (strlen(gz_pb_config.setting.pb_input_file) > 0)
  {
    if (gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_ALE)
    {
      pb_ag_log_lever_arm_read();
      pb_ag_log_playback();
    }
    else if (gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_RTCM || 
             gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_UBLOX
             )
    {
      pb_stream ps(gz_pb_config);
      ps.pb_rtcm_ubx_log_playback();
    }
    else if (gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_RINEX)
    {
      pb_rinex_log_playback();
    }
    else if (gz_pb_config.setting.pb_input_file_type == PB_INPUT_FILE_BDDB)
    {
      pb_bddb_log_playback();
    }
  } 

  loc_api_Release();

  ipc_parse_release();

  log_DataLogManager_Release();

  log_fclose_safe(gz_pb_config.output.pb_output_ag_fp);
  log_fclose_safe(gz_pb_config.output.pb_output_text_fp);

  pb_assist_load_clear();
  pb_truth_load_clear();

  MemoryStatisticsPrint(NULL);

#ifdef FEATURE_PROFILER
  ProfilerStop();
#endif

  return 0;
}
