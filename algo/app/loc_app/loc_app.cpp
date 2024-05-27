/**@file        vlt-app.cpp
 * @brief       Visual location terminal main source file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/02/09  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#ifdef __linux
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#else
#include <direct.h>
#include <io.h>
#endif

#include <thread>
#include <iostream>
#include <chrono>
#include <ctime>

#include "ini.h"
#include "loguru.h"
#include "loc_core_api.h"
#include "uart_driver.h"
#include "ntrip_client.h"
#include "gnss_type.h"

#include "ag_log_parse.h"
#include "MemoryStatistics.h"

#include "OptionParser.h"

#include "cmn_utils.h"
#include "sm_nmea.h"
#include "qxsi/qxsi_ids_ssr2los_interface.h"
#include "loc_core_ssr_api.h"

#include "qxids_sdk.h"
#include "qx_ssr_client.h"

#if defined(APP_HEXAGON_SSR) // APP_HEXAGON_SSR
#include "correctionsclient.h"
#include "tsx_adapter_integration.h"
#elif defined(APP_SWIFT_SSR) // APP_SWIFT_SSR
#include "./adapter/swift_core_ssr_api.h"
#endif 


#define APPKEY      "A48o38t4dk3i"
#define APPSECRET   "8a4f7529a9f749b3"
#define APPDEVICE_ID   "hongqi1"
#define APPDEVICE_TYPE "hongqi"
#define LOG_DIR "./data/"
#include "nwstream_client.h"




using namespace std;
using namespace std::chrono;

typedef struct {
  NtripStatus_e ntrip_status;
} vlt_AppCtrl_t;

typedef struct {
  char output_dir[256];
  FILE* output_ag_fp;
  FILE* output_text_fp;
  uint8_t log_suffix_by_time;
} app_config_output_t;

typedef enum {
  APP_SOURCE_UART = 0,
  APP_SOURCE_NETWORK
} app_DataSourceEnumVal;
typedef uint8_t app_DataSourceEnum;

typedef struct {
  uint8_t gnss_source;
  char gnss_uart[256];
  uint32_t gnss_baudrate;
  char gnss_network_addr[256];
  uint32_t gnss_network_port;

  uint8_t imu_source;
  char imu_uart[256];
  uint32_t imu_baudrate;
  char imu_network_addr[256];
  uint32_t imu_network_port;
} app_config_driver_t;

typedef struct {
  uint8_t feature;
  char url[256];
  char mountpoint[256];
  uint32_t port;
  char user[256];
  char passwd[256];
  uint32_t ntrip_nmea_cycle;
} app_config_ntrip_t;


typedef struct {
  char dcf_filename[256];  // Deployment Configuration File
} app_config_hexgon_sdk_t;


typedef struct {
  app_config_output_t output;
  app_config_driver_t driver;
  app_config_ntrip_t ntrip;
  qxids_config_t qx_sdkcfg;
  app_config_hexgon_sdk_t cfg_hexgon_sdk;
} app_config_t;

app_config_t g_appConfigParam;
std::unique_ptr<NtripClient> ntripClient = nullptr;
vlt_AppCtrl_t gz_vltAppCtrl;

loc_api_config_para_group_t gz_loc_api_config_para_group = { 0 };
static const qxsi_ids_ssr2los_interface_t* p_qx_ssr2los_interace = NULL;

std::unique_ptr<QXSSRClient> qxssrClient;

#if defined(APP_HEXAGON_SSR)
static int16_t scaled_latitude = 0;
static int16_t scaled_longitude = 0;
static CorrectionsHandle s_handle = NULL;
#endif
static int handler(void* user, const char* section, const char* name,
  const char* value)
{
  unsigned int tmp = 0;
  app_config_t* pconfig = (app_config_t*)user;

#define MATCH(s, n) (strcmp(section, s) == 0 && strcmp(name, n) == 0)
  
  if (MATCH("OUTPUT", "output_dir")) {
    strcpy(pconfig->output.output_dir, value);
  }
  else if (MATCH("OUTPUT", "log_suffix_by_time")) {
      pconfig->output.log_suffix_by_time = atoi(value);
  }
  else if (MATCH("DRIVER", "imu_source")) {
    pconfig->driver.imu_source = atoi(value);
  }
  else if (MATCH("DRIVER", "imu_uart")) {
    strcpy(pconfig->driver.imu_uart, value);
  }
  else if (MATCH("DRIVER", "imu_baudrate")) {
    pconfig->driver.imu_baudrate = atoi(value);
  }
  else if (MATCH("DRIVER", "imu_network_addr")) {
    strcpy(pconfig->driver.imu_network_addr, value);
  }
  else if (MATCH("DRIVER", "imu_network_port")) {
    pconfig->driver.imu_network_port = atoi(value);
  }
  else if (MATCH("DRIVER", "gnss_source")) {
    pconfig->driver.gnss_source = atoi(value);
  }
  else if (MATCH("DRIVER", "gnss_uart")) {
    strcpy(pconfig->driver.gnss_uart, value);
  }
  else if (MATCH("DRIVER", "gnss_baudrate")) {
    pconfig->driver.gnss_baudrate = atoi(value);
  }
  else if (MATCH("DRIVER", "gnss_network_addr")) {
    strcpy(pconfig->driver.gnss_network_addr, value);
  }
  else if (MATCH("DRIVER", "gnss_network_port")) {
    pconfig->driver.gnss_network_port = atoi(value);
  }
  else if (MATCH("ntrip", "feature")) {
    pconfig->ntrip.feature = atoi(value);
  }
  else if (MATCH("ntrip", "url")) {
    strcpy(pconfig->ntrip.url, value);
  }
  else if (MATCH("ntrip", "mountpoint")) {
    strcpy(pconfig->ntrip.mountpoint, value);
  }
  else if (MATCH("ntrip", "port")) {
    pconfig->ntrip.port = atoi(value);
  }
  else if (MATCH("ntrip", "user")) {
    strcpy(pconfig->ntrip.user, value);
  }
  else if (MATCH("ntrip", "passwd")) {
    strcpy(pconfig->ntrip.passwd, value);
  }
  else if (MATCH("ntrip", "nmea_cycle")) {
    pconfig->ntrip.ntrip_nmea_cycle = atoi(value);
  }
  else if (MATCH("qx_sdk", "auth_mode")) {
    tmp = atoi(value);
    if ((QXIDS_KEY_TYPE_AK == tmp) || (QXIDS_KEY_TYPE_DSK == tmp)) {
      pconfig->qx_sdkcfg.key_type = (qxids_key_type_e)tmp;
    }
  }
  else if (MATCH("qx_sdk", "log_mode")) {
      tmp = atoi(value);
      pconfig->qx_sdkcfg.log_mode = tmp;
  }
  else if (MATCH("qx_sdk", "log_level")) {
	tmp = atoi(value);
	if(tmp <= QXIDS_LOG_V){
        pconfig->qx_sdkcfg.log_level = (qxids_log_level_e)tmp;
	}
  }
  else if (MATCH("qx_sdk", "log_dir")) {
	strcpy(pconfig->qx_sdkcfg.log_dir, value);
  }
  else if (MATCH("qx_sdk", "appkey")) {
	strcpy(pconfig->qx_sdkcfg.app_key, value);
  }
  else if (MATCH("qx_sdk", "appsecret")) {
	strcpy(pconfig->qx_sdkcfg.app_secret, value);
  }
  else if (MATCH("qx_sdk", "device_id")) {
	strcpy(pconfig->qx_sdkcfg.device_id, value);
  }
  else if (MATCH("qx_sdk", "device_type")) {
	strcpy(pconfig->qx_sdkcfg.device_type, value);
  }
  else if (MATCH("hexagon_sdk", "DCF")) {
	strcpy(pconfig->cfg_hexgon_sdk.dcf_filename, value);
  }

  char item_str[32] = { 0 };
  char feild_str[32] = { 0 };

  for (int i = 0; i < M_ARRAY_SIZE(gz_loc_api_config_para_group.para); i++)
  {
      sprintf(item_str, "Param%d", i);
      sprintf(feild_str, "mask");
      if (MATCH(item_str, feild_str))
      {
          gz_loc_api_config_para_group.para[i].t_mask = strtol(value, NULL, 16);
          return 1;
      }

      if (i == 5) {
          char vdr_int[10][20] = {
                "gnss_rate",
                "imu_sampling_rate",
                "whspd_mode",
                "whspd_rate",
                "vdr_mode",
                "align_mode",
                "outputpos_flag",
                "output_rate",
                "max_drtime",
                "imu_type"};

          char vdr_float[10][20] = {
                "imu2gnss_x",
                "imu2gnss_y",
                "imu2gnss_z",
                "imu2rearmid_x",
                "imu2rearmid_y",
                "imu2rearmid_z",
                "rear2rear",
                "mis_b2g_roll",
                "mis_b2g_pitch",
                "mis_b2g_heading" };

          for (int j = 0; j < M_ARRAY_SIZE(gz_loc_api_config_para_group.para[i].u_field); j++)
          {
              sprintf(feild_str, "int_val%d_%s", j,vdr_int[j]);
              if (MATCH(item_str, feild_str))
              {
                  gz_loc_api_config_para_group.para[i].u_field[j] = strtol(value, NULL, 0);
                  return 1;
              }
          }

          for (int j = 0; j < M_ARRAY_SIZE(gz_loc_api_config_para_group.para[i].f_field); j++)
          {
              sprintf(feild_str, "float_val%d_%s", j,vdr_float[j]);
              if (MATCH(item_str, feild_str))
              {
                  gz_loc_api_config_para_group.para[i].f_field[j] = strtof(value, NULL);
                  return 1;
              }
          }
      }
      else {
          for (int j = 0; j < M_ARRAY_SIZE(gz_loc_api_config_para_group.para[i].u_field); j++)
          {
              sprintf(feild_str, "int_val%d", j);
              if (MATCH(item_str, feild_str))
              {
                  gz_loc_api_config_para_group.para[i].u_field[j] = strtol(value, NULL, 10);
                  return 1;
              }
          }

          for (int j = 0; j < M_ARRAY_SIZE(gz_loc_api_config_para_group.para[i].f_field); j++)
          {
              sprintf(feild_str, "float_val%d", j);
              if (MATCH(item_str, feild_str))
              {
                  gz_loc_api_config_para_group.para[i].f_field[j] = strtof(value, NULL);
                  return 1;
              }
          }
      }
  }
  return 1;
}

/**
 * @brief Report loc core output log
 * @param[in] data - log buffer
 * @param[in] length - log length
 * @return      None
 */
static void loc_core_callback_report_log_handler(uint8_t* data, uint32_t length)
{
  static log_decoder_t log_decoder;
  static uint8_t u_ag_log_decoded_data[LOG_DECODER_SIZE] = { 0 };
  static uint32_t q_ag_log_decoded_length = LOG_DECODER_SIZE;
  int8_t ret = 0;

  memset(u_ag_log_decoded_data, 0, sizeof(u_ag_log_decoded_data));
  q_ag_log_decoded_length = M_ARRAY_SIZE(u_ag_log_decoded_data) - LOG_PRECHECK_LEN;

  for (uint32_t i = 0; i < length; i++)
  {
    ret = ag_log_decode(&log_decoder, data[i], u_ag_log_decoded_data, &q_ag_log_decoded_length);
    if (0 == ret)
    {
      continue;
    }
    else if (-1 == ret)
    {
      printf("Log Decode error, buffer length %d < %d\n",
        q_ag_log_decoded_length, log_decoder.length);
    }

    if ((LOG_TYPE_TEXT == ret) || (LOG_TYPE_CRC_TEXT == ret))
    {
      fwrite(u_ag_log_decoded_data, q_ag_log_decoded_length, 1, g_appConfigParam.output.output_text_fp);
    }
    else if ((LOG_TYPE_IPC == ret) || (LOG_TYPE_CRC_IPC == ret))
    {
      // Nothing
    }

    memset(u_ag_log_decoded_data, 0, sizeof(u_ag_log_decoded_data));
    q_ag_log_decoded_length = M_ARRAY_SIZE(u_ag_log_decoded_data) - LOG_PRECHECK_LEN;
  }
  fwrite(data, length, 1, g_appConfigParam.output.output_ag_fp);

  return;
}

/**
 * @brief Report loc core navigation solution
 * @param[in] location - navigation solution result
 * @return      None
 */
static void loc_core_callback_report_location_handler(loc_api_location_report_t* location)
{
  static bool send_init = false;
  static uint32_t send_count = 0;

  char nmea_gga[256] = { 0 };
  double lat = location->lat;
  double lon = location->lon;
  
  if (lat > 180.0) {
	lat -= 180.0;
  }
  
  if (lon > 180) {
	lon -= 180.0;
  }

  sm_nmea_create_gga_deprecate(location, nmea_gga);

  if ((g_appConfigParam.ntrip.feature) &&
    (true == ntripClient->get_enable()))
  {
    if (send_init)
    {
      send_count++;
    }

    if ((!send_init) || (send_count > g_appConfigParam.ntrip.ntrip_nmea_cycle))
    {
      if (((M_IS_SET_MASK(gz_vltAppCtrl.ntrip_status, NTRIP_AUTHORIZED)) &&
        (true == ntripClient->authorized()) &&
        (location->fix_quality != GNSS_FIX_FLAG_INVALID)))
      {
        ntripClient->send_nmea(nmea_gga, strlen(nmea_gga));
        send_count = 0;
        send_init = true;
        LOG_F(INFO, "Succ to send ntrip nmea, ntrip %d loc flag %d",
          gz_vltAppCtrl.ntrip_status, location->fix_quality);

        if (qxssrClient != NULL) {
          qxssrClient->inject_nmea_to_qx_ssr((char*)nmea_gga, strlen(nmea_gga));
        }
      }
      else
      {
        LOG_F(INFO, "Fail to send ntrip nmea, ntrip %d loc flag %d",
          gz_vltAppCtrl.ntrip_status, location->fix_quality);
      }
    }
  }
#if defined(APP_HEXAGON_SSR)
  uint16_t	selected_tile = 0;
  int16_t scaled_latitude = (int16_t)lat* CORRECTIONSCLIENT_POSITION_SCALE_FACTOR;
  int16_t scaled_longitude = (int16_t)lon* CORRECTIONSCLIENT_POSITION_SCALE_FACTOR;
  CorrectionsClient_UpdateTileSelection(s_handle,scaled_latitude,scaled_longitude,&selected_tile);
#endif
  if ((qxssrClient != NULL) && (location->fix_quality != GNSS_FIX_FLAG_INVALID)) {
    qxssrClient->inject_nmea_to_qx_ssr((char*)nmea_gga, strlen(nmea_gga));
  }

  {
    char str[256] = { 0 };
    uint32_t len = 0;

    len = strlen(nmea_gga);
    memset(str, 0, sizeof(str));
    memcpy(str, nmea_gga, len - 2);
    LOG_F(INFO, "%s", str);
  }

  return;
}

/**
 * @brief Report orient result
 * @param[in] location - orient result
 * @return      None
 */
static void loc_core_callback_report_orient_handler(loc_api_orient_report_t* location)
{
  return;
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
 * @brief Meas block process reported Callback
 * @param[in] pz_GnssMeasBlk
 * @return None
 */
static void loc_core_callback_report_gnss_meas(loc_api_GnssMeasBlock_t* pz_GnssMeasBlk)
{
// #if defined(FEATURE_USE_QXWZ_SSR)
#if 0
  qxsi_station_info_t* pz_qxsi_station_info = NULL;
  loc_api_ssrLosBlock_t* pz_SsrLosBlk = NULL;
  qxsi_station_los_t* p_qxsi_los = NULL;
  uint8_t u_los_pull_succ = 0;
  uint8_t u_ret = 0;

  if ((NULL == pz_GnssMeasBlk)||(NULL == p_qx_ssr2los_interace))
  {
    return;
  }
  while (1)
  {
    pz_qxsi_station_info = (qxsi_station_info_t*)malloc(sizeof(qxsi_station_info_t));
    if (NULL == pz_qxsi_station_info) {
      break;
    }
    u_ret = ssr_Convert_LocApiGnssMeasBlkToQxsiStationInfo(pz_GnssMeasBlk, pz_qxsi_station_info);
    if (TRUE != u_ret) {
      break;
    }
    p_qxsi_los = (qxsi_station_los_t*)malloc(sizeof(qxsi_station_los_t));
    if (NULL == p_qxsi_los) {
      break;
    }

    u_los_pull_succ = p_qx_ssr2los_interace->pull_los_data_by_obs(pz_qxsi_station_info, p_qxsi_los);

    pz_SsrLosBlk = (loc_api_ssrLosBlock_t*)malloc(sizeof(loc_api_ssrLosBlock_t));
    if (NULL == pz_SsrLosBlk) {
      break;
    }

    ssr_Convert_QxsiStationLosToLocApiSsrLosBlk(p_qxsi_los, pz_SsrLosBlk);
    loc_api_InjectSsrLosBlock(pz_SsrLosBlk);

    break;
  }
  
  if (NULL != pz_qxsi_station_info) free(pz_qxsi_station_info);
  if (NULL != p_qxsi_los) free(p_qxsi_los);
  if (NULL != pz_SsrLosBlk) free(pz_SsrLosBlk);
#endif
  return;
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
 * @brief Meas and satellite data frame
 * @param[in] loc_api_NavigationData_t 
 */
static void loc_core_callback_report_navgation_data(loc_api_NavigationData_t* pz_ApiNav)
{
#if defined(APP_HEXAGON_SSR)
  tsx_inject_navigation_data(pz_ApiNav);
#endif
  return;
}

// #ifdef FEATURE_USE_QXWZ_SSR
#if 0
/**
 * @brief QianXun Ssr2Los adapter log callback
 * @param[in]   buf
 * @param[in]   len
 * @return      None
 */
static void loc_qx_cb_log(const char* const buf, const int len)
{
  return;
}

/**
 * @brief QianXun Ssr2Los status log callback
 * @param[in]   code
 * @return      None
 */
static void loc_qx_cb_status(qxsi_ids_ssr2los_status_e code)
{
  return;
}

/**
 * @brief release QianXun Ssr2Los adapter
 * @param[in]   None
 * @return      return 0 on success, -1 on failure
 */
static BOOL loc_SsrQXAdapter_Release()
{
  int32_t ret = -1;
  if (NULL == p_qx_ssr2los_interace)
  {
    return FALSE;
  }

  ret = p_qx_ssr2los_interace->cleanup();
  if (-1 == ret)
  {
    return FALSE;
  }

  p_qx_ssr2los_interace = NULL;
  return TRUE;
}
#endif

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
 * @brief Ntrip client report info callback
 * @param[in]
 * @return      None
 */
static void ntrip_client_callback_report_status_handler(uint32_t status)
{
  gz_vltAppCtrl.ntrip_status = status;
  return;
}

/**
 * @brief Ntrip client recv callback
 * @param[in]
 * @return      None
 */
static void ntrip_client_callback_receive_data_handler(uint8_t* buf, uint32_t len)
{
  LOG_F(INFO, "Receive ntrip client data length %d", len);
  loc_api_InjectRefCorrRTCM(buf, len);
  return;
}

static void loadConfigByDefault(app_config_t* cfg)
{
  if (NULL == cfg)
  {
    return;
  }

#if 0
  strcpy(cfg->output.output_dir, "./logs/");
  cfg->driver.gnss_baudrate = 460800;
  strcpy(cfg->driver.gnss_uart, "COM3");
  cfg->driver.imu_baudrate = 460800;
  strcpy(cfg->driver.imu_uart,  "COM1");

  app_config_ntrip_t* account = &(cfg->ntrip);
  account->feature = 1;
  strcpy(account->url,          "ntrip.qxwz.com");
  strcpy(account->mountpoint,   "AUTO");
  account->port = 8002;
  strcpy(account->user,         "gzdydz0041");
  strcpy(account->passwd,       "dy2022n");
  account->ntrip_nmea_cycle = 60;

  qxids_config_t* default_cfg = &(cfg->qx_sdkcfg);
	default_cfg->size		   = sizeof(qxids_config_t);
	default_cfg->log_mode	   = QXIDS_LOG_FILE; /* QXIDS_LOG_CONSOLE | QXIDS_LOG_FILE */
	default_cfg->log_level	   = QXIDS_LOG_V;
	default_cfg->key_type =QXIDS_KEY_TYPE_AK;
	sprintf(default_cfg->app_key, "%s", APPKEY);
	sprintf(default_cfg->app_secret, "%s", APPSECRET);
	sprintf(default_cfg->device_id, "%s", APPDEVICE_ID);
	sprintf(default_cfg->device_type, "%s", APPDEVICE_TYPE);
	sprintf(default_cfg->log_dir, "%s", LOG_DIR);
#endif

}

static void cb_ssr_data(char const* buf, qxids_size_t len)
{
  int ret = 0;
  if ((buf != NULL) && (len > 0)) {
 #ifdef FEATURE_USE_QXWZ_SSR
    loc_api_InjectSsrStream((uint8_t*)buf, (uint32_t)len, GNSS_SSR_SERVICE_SOURCE_QX);
 #else
    if (p_qx_ssr2los_interace != NULL) {
        ret = p_qx_ssr2los_interace->push_ssr((const unsigned char* const)buf, len);
        if (0 == ret)
        {
          printf("SSR push string length %d succ\n", len);
        }
        else
        {
          printf("SSR push string length %d fail\n", len);
        }
    }
#endif
  }
}

int main(int argc, char* argv[])
{
  const string description = "Location Engine Application Executor";
  char version[256] = { 0 };
  sprintf(version, "Version Build %s %s", __TIME__, __DATE__);
  const string usage = "usage: %prog [-c] [cfg_file]...";
  optparse::OptionParser parser = optparse::OptionParser()
    .description(description)
    .usage(usage)
    .version(version);

  parser.add_option("-c", "--config")
    .action("store")
    .type("string")
    .set_default("app.ini")
    .help("Application config filename");

  optparse::Values options = parser.parse_args(argc, argv);

  parser.print_help();
  parser.print_version();
  parser.print_args();

  /***************************************************************
  * set general default config and config file loading
  ****************************************************************/
  loadConfigByDefault(&g_appConfigParam);

  if (strlen(options["config"].c_str()) > 0)
  {
    if (ini_parse(options["config"].c_str(), handler, &g_appConfigParam) < 0) {
      printf("Can't load config %s, quit\n", options["config"].c_str());
      return 1;
    }
  }

  /***************************************************************
  * Application and core log related configuration
  ****************************************************************/

  /* Set log call back */
  char filename[256] = { 0 };
  struct timespec ts;
  int n = 0;
  int timespec_get_succ = timespec_get(&ts, TIME_UTC);
#ifdef _MSC_VER
  if (_access(g_appConfigParam.output.output_dir, 0) == -1)
  {
      if (0 == _mkdir(g_appConfigParam.output.output_dir))
      {
         printf("make new dir %s", g_appConfigParam.output.output_dir);
      }
  }
#else
  if (access(g_appConfigParam.output.output_dir, 0) == -1)
  {
      if (0 == mkdir(g_appConfigParam.output.output_dir,777))
      {
         printf("make new dir %s", g_appConfigParam.output.output_dir);
      }
  }
#endif

  memset(filename, 0, sizeof(filename));
  n = 0;
  n += sprintf(filename + n, "%s", g_appConfigParam.output.output_dir);
  n += sprintf(filename + n, "log-full");
  if (g_appConfigParam.output.log_suffix_by_time && timespec_get_succ)
  {
    n += (int)strftime(filename + n, sizeof(filename), "-%Y%m%d-%H%M%S", gmtime(&ts.tv_sec));
  }
  n += sprintf(filename + n, ".bin");
  g_appConfigParam.output.output_ag_fp = fopen(filename, "wb+");

  memset(filename, 0, sizeof(filename));
  n = 0;
  n += sprintf(filename + n, "%s", g_appConfigParam.output.output_dir);
  n += sprintf(filename + n, "log-text");
  if (g_appConfigParam.output.log_suffix_by_time && timespec_get_succ)
  {
    n += (int)strftime(filename + n, sizeof(filename), "-%Y%m%d-%H%M%S", gmtime(&ts.tv_sec));
  }
  n += sprintf(filename + n, ".txt");
  g_appConfigParam.output.output_text_fp = fopen(filename, "w+");

  /* App log create */
  loguru::init(argc, argv);
  memset(filename, 0, sizeof(filename));
  n = 0;
  n += sprintf(filename + n, "%s/", g_appConfigParam.output.output_dir);
  n += sprintf(filename + n, "log-app");
  if (g_appConfigParam.output.log_suffix_by_time && timespec_get_succ)
  {
    n += (int)strftime(filename + n, sizeof(filename), "-%Y%m%d-%H%M%S", gmtime(&ts.tv_sec));
  }
  n += sprintf(filename + n, ".txt");
  loguru::add_file(filename, loguru::Truncate, loguru::Verbosity_DEBUG);

  // Only show most relevant things on stderr:
  loguru::g_stderr_verbosity = loguru::Verbosity_INFO;
  LOG_F(INFO, "Log module enable.");

  /***************************************************************
* swift adapter
****************************************************************/
#ifdef APP_SWIFT_SSR
    swift_adapter_init();

    // swift_adapter_push_ssr(const uint8_t * ssr_buff, uint32_t length)  // ssr stream is needed to put,TBD
#endif // APP_SWIFT_SSR


  /***************************************************************
  * TSX adapter
  ****************************************************************/
#if defined(APP_HEXAGON_SSR)
  tsx_adapter_init(g_appConfigParam.cfg_hexgon_sdk.dcf_filename);
  tsx_print_version();
  // tsx_tmp_injectTSX();
  // tsx_test_demo2();
  // tsx_test_demo();

  int ret_v = 0;
  BOOL	   ret		 = true; 
  const char* host = "a6gg1eiuqll30-ats.iot.us-west-2.amazonaws.com";
  char* mountpoint = "EUR"; 
  //char* mountpoint = "TSX-QM__/001/EUR/0/5823";
  char* name = "asensing";
  char* pswd = "Elite-Truck-Orbit";
  char* path = NULL;
  const char* port = "443";
  uint16_t m_ver;
  uint16_t sub_ver;
  static uint8_t hxgon_buf[4000];

  CorrectionsClient_GetAPIVersion(&m_ver,&sub_ver);
  LOG_F(INFO, "hexagon m_ver %x,sub_ver %x\n",m_ver,sub_ver);
  ret = CorrectionsClient_Init(host,port,CORRECTION_SERVER_TSX,&s_handle);
  if(false == ret){
	  LOG_F(INFO, "hexagon init failed ret %d\n",ret);
	  return -1;
  }
  ret_v = CorrectionsClient_SetMountpoint(s_handle,mountpoint);
  if(CONNECTION_SUCCESS != ret_v){
	  LOG_F(INFO, "hexagon set mountpoint failed \n");
	  return -1;
  }
  CorrectionsClient_SetCredentials(s_handle,name,pswd);
  //CorrectionsClient_SetCertificatePath(s_handle,path);
  ret_v = CorrectionsClient_Connect(s_handle);
  if(CONNECTION_SUCCESS != ret_v){
	  LOG_F(INFO, "hexagon connect failed %d\n",ret_v);
	  return -1;
  }
  CorrectionsConnectionStatus state = CORRECTIONS_DISCONNECTED;
  CorrectionStatistics	connection_stats;
  memset(&connection_stats,0,sizeof(CorrectionStatistics));

#endif

  /***************************************************************
  * Ntrip client initialize
  ****************************************************************/
// #ifdef FEATURE_USE_QXWZ_SSR
#if 0

    qxsi_ids_ssr2los_config_t config = { 0 };
    qxsi_ids_ssr2los_callbacks_t cbs = { 0 };
    int ret = 0;
    config.enable = 1;
    config.level = QXSI_LOG_LEVEL_DEBUG;

    cbs.log_callback = loc_qx_cb_log;
    cbs.status_callback =loc_qx_cb_status;

    p_qx_ssr2los_interace = qxsi_ids_get_ssr2los_interface();
    if (NULL == p_qx_ssr2los_interace)
    {
      LOG_F(INFO, "get ssr2los interface fail\n");
      return -1;
    }

    ret = p_qx_ssr2los_interace->init(&config, &cbs);
    if (-1 == ret)
    {
      LOG_F(INFO, "init ssr2los interface fail\n");
      return -1;
    }
#endif

  if (g_appConfigParam.ntrip.feature)
  {
    NtripAccount_t ntrip_account = {0};
    ntrip_account.enable = 1;
    strcpy(ntrip_account.url, g_appConfigParam.ntrip.url);
    ntrip_account.port = g_appConfigParam.ntrip.port;
    strcpy(ntrip_account.mountpoint, g_appConfigParam.ntrip.mountpoint);
    strcpy(ntrip_account.user, g_appConfigParam.ntrip.user);
    strcpy(ntrip_account.passwd, g_appConfigParam.ntrip.passwd);
    ntripClient = std::unique_ptr<NtripClient>(new NtripClient(&ntrip_account));
    ntripClient->set_callback_receive_data(ntrip_client_callback_receive_data_handler);
    ntripClient->set_callback_report_status(ntrip_client_callback_report_status_handler);
    ntripClient->connect();
    LOG_F(INFO, "Ntrip client module enable.");
  }
  else
  {
    LOG_F(INFO, "Ntrip client module disable.");
  }

  /***************************************************************
  * QianXun SSR sdk initialize
  ****************************************************************/
#ifdef FEATURE_USE_QXWZ_SSR
	qxids_callbacks_t qx_cbs;

	qx_cbs.size = sizeof(qxids_callbacks_t);
	qx_cbs.fill_nssr_data = cb_ssr_data;

	LOG_F(INFO, "current account info: %s %s %s %s \n", g_appConfigParam.qx_sdkcfg.app_key, g_appConfigParam.qx_sdkcfg.app_secret, g_appConfigParam.qx_sdkcfg.device_id,
		   g_appConfigParam.qx_sdkcfg.device_type);
	qxssrClient = std::unique_ptr<QXSSRClient>(new QXSSRClient(&g_appConfigParam.qx_sdkcfg,&qx_cbs));
	qxssrClient->start_qx_ssr_service();
#endif

  /***************************************************************
  * Location core initialize
  ****************************************************************/ 
  loc_api_callback_t loc_api_callback = {0};

  loc_api_callback.report_log      = loc_core_callback_report_log_handler;
  loc_api_callback.report_location = loc_core_callback_report_location_handler;
  loc_api_callback.report_orient = loc_core_callback_report_orient_handler;
  loc_api_callback.get_tick_ms     = loc_core_callback_get_tick_ms;
  loc_api_callback.report_gnss_measurement = loc_core_callback_report_gnss_meas;
  loc_api_callback.report_gnss_measurement_satellite = loc_core_callback_report_gnss_meas_satpos;
  loc_api_callback.report_gnss_navigation_data = loc_core_callback_report_navgation_data;
  loc_api_Register_Callback(&loc_api_callback);

  loc_api_MemoryRegister_t z_MemoryRegister = { 0 };
  static uint8_t memory_pool0[1024 * 1024 * 5] = { 0 };
  z_MemoryRegister.u_type = LOC_API_MEMORY_EXT_API;
  z_MemoryRegister.pool_addr[0] = &(memory_pool0[0]);
  z_MemoryRegister.pool_size[0] = sizeof(memory_pool0);
  z_MemoryRegister.pool_count = 1;
  z_MemoryRegister.alloc = loc_core_callback_mem_alloc_handler;
  z_MemoryRegister.free = loc_core_callback_mem_free_handler;
  loc_api_Register_MemoryPool(&z_MemoryRegister);

  gz_loc_api_config_para_group.para[0].t_mask = 0x0000000000000027;
  gz_loc_api_config_para_group.para[1].t_mask = 0x0000000000000007;
  gz_loc_api_config_para_group.para[2].t_mask = 0x0000000000000000;
  gz_loc_api_config_para_group.para[3].t_mask = 0x0000000000000000;
  gz_loc_api_config_para_group.para[4].t_mask = 0x0000000000000001;
  gz_loc_api_config_para_group.para[5].t_mask = 0x0000000000000000;
  gz_loc_api_config_para_group.para[6].t_mask = 0x00000000000000A5;
  gz_loc_api_config_para_group.para[7].t_mask = 0x0000000000000001;

  loc_api_Register_Config(&gz_loc_api_config_para_group);
  loc_api_Initialize();

  std::unique_ptr<NwStreamClient> gnss_NwStreamClient = nullptr;
  std::unique_ptr<UartDriver>     gnss_UartClient = nullptr;

  if (APP_SOURCE_UART == g_appConfigParam.driver.gnss_source)
  {
    char* gnss_uart = g_appConfigParam.driver.gnss_uart;
    uint32_t gnss_baudrate = g_appConfigParam.driver.gnss_baudrate;

    gnss_UartClient = std::unique_ptr<UartDriver>(new UartDriver(gnss_uart, gnss_baudrate));

    gnss_UartClient->start_Run(
      [](uint8_t* data, uint32_t len) {
        loc_api_InjectRcvMeasRTCM(data, len);
      }
    );
  }
  else if (APP_SOURCE_NETWORK == g_appConfigParam.driver.gnss_source)
  {
    char* server_url = g_appConfigParam.driver.gnss_network_addr;
    uint32_t length = strlen(server_url);
    uint32_t port = g_appConfigParam.driver.gnss_network_port;

    gnss_NwStreamClient = std::unique_ptr<NwStreamClient>(new NwStreamClient((char*)server_url, length, port));

    gnss_NwStreamClient->set_callback_receive_data (
      [](uint8_t* data, uint32_t len) {
        loc_api_InjectRcvMeasRTCM(data, len);
      }
    );

    gnss_NwStreamClient->connect();
  }

  LOG_F(INFO, "Loc app start run...");
  while (1)
  {
#if defined(APP_HEXAGON_SSR)
    CorrectionsClient_GetStatus(s_handle, &state, &connection_stats);
    LOG_F(INFO, "hexagon state %d, mountpoint %s data_rate_kbps %f current_outage_time_sec %d uptime_sec %d, received %d droped %d\n", state, connection_stats.current_mountpoint, connection_stats.data_rate_kbps, connection_stats.current_outage_time_sec, connection_stats.current_uptime_sec, connection_stats.total_bytes_received, connection_stats.total_bytes_dropped);
    int32_t q_length = CorrectionsClient_Receive(s_handle, hxgon_buf, 4000);
    if (q_length != 0)
    {
      tsx_injectCorrectionData(q_length, (const uint8_t *)hxgon_buf);
      /* package */
      uint32_t q_length_in = (uint32_t)(q_length);
      uint8_t *u_data = (uint8_t *)malloc(q_length_in + sizeof(uint32_t));
      memcpy(u_data, &q_length_in, sizeof(uint32_t));
      memcpy(u_data + sizeof(uint32_t), hxgon_buf, q_length_in);
      LOG_F(INFO, "log_Package LOG_PACKAGE_ID_TSX_CORR %d", q_length_in);
      log_Package(LOG_PACKAGE_ID_TSX_CORR, u_data, sizeof(uint32_t) + q_length_in);
      free(u_data);
    }
#endif

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  /***************************************************************
  * QianXun SSR sdk release
  ****************************************************************/
  if (nullptr != qxssrClient)
  {
    qxssrClient->stop_qx_ssr_service();
  }

  if (nullptr != ntripClient)
  {
    ntripClient->release();
  }

  if (nullptr != gnss_NwStreamClient)
  {
    gnss_NwStreamClient->release();
  }

  loc_api_Release();
// #ifdef FEATURE_USE_QXWZ_SSR
#if 0
  loc_SsrQXAdapter_Release();
#endif

  MemoryStatisticsPrint((char*)"memory.txt");

  return 0;
}

