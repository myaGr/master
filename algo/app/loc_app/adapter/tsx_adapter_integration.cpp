/**@file        tsx_adapter_integration.cpp
 * @brief       TSX adapter
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/07  <td>0.1      <td>houxiaowei   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "tsx_adapter_integration.h"

#if defined(APP_HEXAGON_SSR)
#include "loguru.h"
#include "loc_core_api.h"
#include "loc_core_ssr_api.h"
#include "tsx_test_types.h"
#if defined(_MSC_VER)
#include <direct.h>
#include <io.h>
#elif defined(__gnu_linux__)
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include <cstring>
#endif // _MSC_VER


#define HARD_CODE_ORBIT_SIGMA     (0.05)                    /* [m] orbit the baseline 1 sigma correction value accuracies */
#define HARD_CODE_CLOCK_SIGMA     (0.01)                    /* [m] orbit the baseline 1 sigma correction value accuracies */
#define TSX_CLIGHT                ((double)299792458.0)     /* speed of light (m/s) */
#define TSX_OMGE_GPS              ((double)7.2921151467E-5) /* GPS WGS84 Earth rotation Rate */
// #define TSX_DEBUG

static double g_nav_tow;
static uint16_t g_w_week;

typedef uint64_t UINT64;
typedef int32_t INT32;

extern "C" {
  extern int n_sub_corr;
  extern CorrectionsStructure* astCorrections;
  extern int n_sub_frame;
  extern SubFrameStructure* astSubFrames;
}

static TSXHandle tsx_handle = NULL;

typedef struct
{
  uint8_t u_satCount;
  TSXSatelliteCorrectionsEntry z_sat_corr[LOC_API_SSR_SAT_NUM_LIMIT];
} TSXSatelliteCorrectionsCollect_t;


typedef struct 
{
  FILE* fp_debug_log;
  FILE* fp_correction;
  FILE* fp_subframe;
}tsx_config_t;

tsx_config_t g_tsx_config = {0};

/**
 * @brief Get file size
 *
 * @param[in] filename
 * @return 0: failed, other: success
 */
static uint64_t tsx_adapter_get_file_size(const char *filename)
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

/**
 * @brief  Log tsx version
 * @return 0: success, others: failed
 */
extern uint8_t tsx_print_version()
{
  if(tsx_handle == NULL)
  {
    return 1;
  }
  uint16_t api_major_version = 0;
  uint16_t api_minor_version = 0;
  uint32_t product_id = 0;
  uint16_t feature_version = 0;
  uint16_t maintenance_version = 0;
  char build_type[1024] = {0};
  uint32_t sequence = 0;

  bool status_ver_pai = TSX_GetAPIVersion(&api_major_version, &api_minor_version);
  bool status_ver_lib = TSX_GetLibraryVersion(&product_id, &feature_version, &maintenance_version, build_type, &sequence);
  if (!status_ver_pai || !status_ver_lib)
  {
    return 1;
  }
  LOG_F(INFO, "TSX_GetAPIVersion:");
  LOG_F(INFO, "api_major_version  :   %u", api_major_version);
  LOG_F(INFO, "api_minor_version  :   %u", api_minor_version);

  LOG_F(INFO, "TSX_GetLibraryVersion:");
  LOG_F(INFO, "product_id         :   %u", product_id);
  LOG_F(INFO, "feature_version    :   %u", feature_version);
  LOG_F(INFO, "maintenance_version:   %u", maintenance_version);
  LOG_F(INFO, "build_type         :   %s", build_type);
  LOG_F(INFO, "sequence           :   %u", sequence);

  return 0;
}

/**
 * @brief Init
 * @param[in] file_name DCF
 * @return uint8_t 0:success, others: failed
 */
extern uint8_t tsx_adapter_init(const char *file_name)
{
  FILE *fp_hxg_dcf = fopen(file_name, "rb");
  if (NULL == fp_hxg_dcf)
  {
    LOG_F(INFO, "open hexagon DCF file error %s", file_name);
    return -1;
  }

  /* init config*/
#if defined(TSX_DEBUG)
  g_tsx_config.fp_debug_log = fopen("tsx.log", "w+");  /* dnt release */
  g_tsx_config.fp_correction = fopen("corrections.log", "w+");
  g_tsx_config.fp_subframe =  fopen("nav_subframe.log", "w+");
#endif

  /* init TSX adapter*/
  uint64_t file_size = tsx_adapter_get_file_size(file_name);
  uint8_t *input_read_buf = (uint8_t *)malloc(file_size);
  size_t buf_len = fread(input_read_buf, 1, file_size, fp_hxg_dcf);
  TSXAdapterResult status = TSX_Initialize(input_read_buf, buf_len, &tsx_handle);
  if (TSXADAPTER_SUCCESS != status)
  {
    LOG_F(INFO, "TSX_Initialize failed %d", status);
    return -1;
  }
  if (tsx_print_version() != 0)
  {
    LOG_F(INFO, "tsx_version failed");
    return -1;
  }
  return 0;
}

static loc_api_SignalType tsx_cvt_Signal2ApiSignal(const TSXSignalType src)
{
  loc_api_SignalType u_sig = LOC_API_GNSS_SIG_MAX;
  switch (src)
  {
  case TSX_SIGNAL_INVALID    : u_sig = LOC_API_GNSS_SIG_MAX; break;
  case TSX_SIGNAL_GPSL1CA    : u_sig = LOC_API_GNSS_SIG_GPS_L1C; break;
  // case TSX_SIGNAL_GPSL1P     : u_sig = LOC_API_GNSS_SIG_MAX; break;
  // case TSX_SIGNAL_GPSL2Y     : u_sig = LOC_API_GNSS_SIG_MAX; break;
  case TSX_SIGNAL_GPSL2CM    : u_sig = LOC_API_GNSS_SIG_GPS_L2C; break;
  case TSX_SIGNAL_GPSL5Q     : u_sig = LOC_API_GNSS_SIG_GPS_L5Q; break;
  case TSX_SIGNAL_GALE1C     : u_sig = LOC_API_GNSS_SIG_GAL_E1; break;
  case TSX_SIGNAL_GALE1B     : u_sig = LOC_API_GNSS_SIG_GAL_E1; break;
  case TSX_SIGNAL_GALE5AQ    : u_sig = LOC_API_GNSS_SIG_GAL_E5A; break;
  case TSX_SIGNAL_GALE5BQ    : u_sig = LOC_API_GNSS_SIG_GAL_E5B; break;
  case TSX_SIGNAL_GALALTBOCQ : u_sig = LOC_API_GNSS_SIG_MAX; break;
  case TSX_SIGNAL_GALE6B     : u_sig = LOC_API_GNSS_SIG_MAX; break;
  case TSX_SIGNAL_BDSB1I     : u_sig = LOC_API_GNSS_SIG_BDS_B1I; break;
  case TSX_SIGNAL_BDSB2I     : u_sig = LOC_API_GNSS_SIG_BDS_B2I; break;
  case TSX_SIGNAL_BDSB3I     : u_sig = LOC_API_GNSS_SIG_BDS_B3I; break;
  case TSX_SIGNAL_BDSB2AP    : u_sig = LOC_API_GNSS_SIG_BDS_B2A; break;
  case TSX_SIGNAL_BDSB1CP    : u_sig = LOC_API_GNSS_SIG_BDS_B1C; break;
  case TSX_SIGNAL_BDSB2BI    : u_sig = LOC_API_GNSS_SIG_BDS_B2B; break;
  case TSX_SIGNAL_QZSSL1CA   : u_sig = LOC_API_GNSS_SIG_QZS_L1C; break;
  // case TSX_SIGNAL_QZSSL1S    : u_sig = LOC_API_GNSS_SIG_MAX; break;
  case TSX_SIGNAL_QZSSL2CM   : u_sig = LOC_API_GNSS_SIG_QZS_L2C; break;
  // case TSX_SIGNAL_QZSSL2CL   : u_sig = LOC_API_GNSS_SIG_QZS_L2C; break;
  case TSX_SIGNAL_QZSSL5Q    : u_sig = LOC_API_GNSS_SIG_QZS_L5Q; break;
  // case TSX_SIGNAL_QZSSL6P    : u_sig = LOC_API_GNSS_SIG_MAX; break;
  default: u_sig = LOC_API_GNSS_SIG_MAX; break;
  }
  return u_sig;
}


static loc_api_gnssConstellationType tsx_cvt_TsxSys2ApiSys(TSXConstellationType ts_sys)
{
  loc_api_gnssConstellationType api_sys = LOC_API_GNSS_MAX;
  switch (ts_sys)
  {
  case TSX_CONSTELLATION_GPS:
    api_sys = LOC_API_GNSS_GPS;
    break;
  case TSX_CONSTELLATION_GALILEO:
    api_sys = LOC_API_GNSS_GAL;
    break;
  case TSX_CONSTELLATION_BEIDOU:
    api_sys = LOC_API_GNSS_BDS;
    break;
  case TSX_CONSTELLATION_QZSS:
    api_sys = LOC_API_GNSS_QZS;
    break;
  default:
    break;
  }
  return api_sys;
}

static TSXConstellationType tsx_cvt_ApiSys2TsxSys(loc_api_gnssConstellationType api_sys)
{
  TSXConstellationType tsx_sys = TSX_CONSTELLATION_UNKNOWN;
  switch (api_sys)
  {
  case LOC_API_GNSS_GPS:
    tsx_sys = TSX_CONSTELLATION_GPS;
    break;
  case LOC_API_GNSS_GAL:
    tsx_sys = TSX_CONSTELLATION_GALILEO;
    break;
  case LOC_API_GNSS_BDS:
    tsx_sys = TSX_CONSTELLATION_BEIDOU;
    break;
  case LOC_API_GNSS_QZS:
    tsx_sys = TSX_CONSTELLATION_QZSS;
    break;
  default:
    break;
  }
  return tsx_sys;
}

/**
 * @brief Get the minist api time
 * @param[in] src 
 * @param[out] dst 
 */
static void tsx_cvt_time2ApiTime(const TSXGNSSTime* src, loc_api_GpsTime_t* dst)
{
  dst->w_week = (uint16_t)src->week;
  dst->tow = (double)src->seconds;
}

static loc_api_GpsTime_t tsx_ApiTimeMin(const loc_api_GpsTime_t *t1, const loc_api_GpsTime_t *t2)
{
  if (t1->w_week == 0 && t1->tow == 0 && !(t2->w_week == 0 && t2->tow == 0))
  {
    return *t2;
  }

  if (t2->w_week == 0 && t2->tow == 0 && !(t1->w_week == 0 && t1->tow == 0))
  {
    return *t1;
  }

  if (t2->w_week < t1->w_week && t1->tow < t2->tow)
  {
    return *t1;
  }
  else
  {
    return *t2;
  }
}

/**
 * @brief convert TSX integrith flag to Api integrity flag
 * @param[in] src 
 * @return loc_api_integrityFlag 
 */
static loc_api_integrityFlag tsx_cvt_IntegFlag2ApiIntegFlag(const TSXIntegrityDNUFlag src)
{
  loc_api_integrityFlag dst = LOC_API_INTEGRITY_FLAG_NOT_MONITORED;
  switch (src)
  {
  case INTEGRITY_NO_INFO:
    dst = LOC_API_INTEGRITY_FLAG_NOT_MONITORED;
    break;
  case INTEGRITY_SAFE_FOR_USE:
    dst = LOC_API_INTEGRITY_FLAG_MONITORED_OK;
    break;
  case INTEGRITY_DO_NOT_USE:
    dst = LOC_API_INTEGRITY_FLAG_MONITORED_FAIL;
    break;
  default:
    break;
  }
  return dst;
}


/**
 * @brief Convert a signal type to frequency type
 * @param[in] signal type
 * @return frequency type
 */
static loc_api_gnssFreqType tsx_cvt_ApiSig2ApiFreqType(loc_api_SignalType signal)
{
  loc_api_gnssFreqType f =  LOC_API_GNSS_FREQ_TYPE_INVALID;
  switch (signal)
  {

  case LOC_API_GNSS_SIG_GPS_L1C:
    f = LOC_API_GNSS_FREQ_TYPE_L1;
    break;
  case LOC_API_GNSS_SIG_GPS_L2C:
    f = LOC_API_GNSS_FREQ_TYPE_L2;
    break;
  case LOC_API_GNSS_SIG_GPS_L5Q:
    f = LOC_API_GNSS_FREQ_TYPE_L5;
    break;
  case LOC_API_GNSS_SIG_GLO_G1:
    f = LOC_API_GNSS_FREQ_TYPE_L1;
    break;
  case LOC_API_GNSS_SIG_GLO_G2:
    f = LOC_API_GNSS_FREQ_TYPE_L2;
    break;
  case LOC_API_GNSS_SIG_BDS_B1I:
    f = LOC_API_GNSS_FREQ_TYPE_L1;
    break;
  // case  LOC_API_GNSS_SIG_BDS_B2I:
  //   f =  LOC_API_GNSS_FREQ_TYPE_L5;
  //   break;
  case LOC_API_GNSS_SIG_BDS_B3I:
    f = LOC_API_GNSS_FREQ_TYPE_L2;
    break;
  case LOC_API_GNSS_SIG_BDS_B1C:
    f = LOC_API_GNSS_FREQ_TYPE_INVALID;
    break;
  case LOC_API_GNSS_SIG_BDS_B2A:
    f = LOC_API_GNSS_FREQ_TYPE_L5;
    break;
  case LOC_API_GNSS_SIG_BDS_B2B:
    f = LOC_API_GNSS_FREQ_TYPE_INVALID;
    break;
  case LOC_API_GNSS_SIG_GAL_E1:
    f = LOC_API_GNSS_FREQ_TYPE_L1;
    break;
  case LOC_API_GNSS_SIG_GAL_E5A:
    f = LOC_API_GNSS_FREQ_TYPE_L5;
    break;
  case LOC_API_GNSS_SIG_GAL_E5B:
    f = LOC_API_GNSS_FREQ_TYPE_L2;
    break;
  case LOC_API_GNSS_SIG_QZS_L1C:
    f = LOC_API_GNSS_FREQ_TYPE_L1;
    break;
  case LOC_API_GNSS_SIG_QZS_L2C:
    f = LOC_API_GNSS_FREQ_TYPE_L2;
    break;
  case LOC_API_GNSS_SIG_QZS_L5Q:
    f = LOC_API_GNSS_FREQ_TYPE_L5;
    break;
  default:
    f = LOC_API_GNSS_FREQ_TYPE_INVALID;
    break;
  }
  return f;
}

/**
 * @brief Convert TSX_frequency to API_frequency
 * @param[in] tsx_frq 
 * @return loc_api_gnssFreqType, 
 */
static loc_api_gnssFreqType tsx_cvt_frq2ApiFrq(const TSXFrequency tsx_frq)
{
  loc_api_gnssFreqType u_apiFrq = LOC_API_GNSS_FREQ_TYPE_INVALID;
  switch (tsx_frq)
  {
  case TSX_FREQ_L1: 
  case TSX_FREQ_E1: 
  case TSX_FREQ_B1:
  u_apiFrq = LOC_API_GNSS_FREQ_TYPE_L1; break;
  case TSX_FREQ_L2:
  case TSX_FREQ_E5B:
  case TSX_FREQ_B3:
  u_apiFrq = LOC_API_GNSS_FREQ_TYPE_L2; break;
  case TSX_FREQ_L5:
  case TSX_FREQ_E5A:
  case TSX_FREQ_B2A:
  u_apiFrq = LOC_API_GNSS_FREQ_TYPE_L5; break;
  default:
  u_apiFrq = LOC_API_GNSS_FREQ_TYPE_INVALID;
    break;
  }
  return u_apiFrq;
}

#if 0
static TSXSignalType tsx_cvt_ApiSignal2Signal(const  loc_api_SignalType src)
{
  TSXSignalType u_sig = TSX_SIGNAL_INVALID;
  switch (src)
  {
  case LOC_API_GNSS_SIG_MAX    : u_sig = TSX_SIGNAL_INVALID    ; break;
  case LOC_API_GNSS_SIG_GPS_L1C: u_sig = TSX_SIGNAL_GPSL1CA    ; break;
  case LOC_API_GNSS_SIG_GPS_L2C: u_sig = TSX_SIGNAL_GPSL2CM    ; break;
  case LOC_API_GNSS_SIG_GPS_L5Q: u_sig = TSX_SIGNAL_GPSL5Q     ; break;
  case LOC_API_GNSS_SIG_GAL_E1 : u_sig = TSX_SIGNAL_GALE1C     ; break;
  case LOC_API_GNSS_SIG_GAL_E5A: u_sig = TSX_SIGNAL_GALE5AQ    ; break;
  case LOC_API_GNSS_SIG_GAL_E5B: u_sig = TSX_SIGNAL_GALE5BQ    ; break;
  case LOC_API_GNSS_SIG_BDS_B1I: u_sig = TSX_SIGNAL_BDSB1I     ; break;
  case LOC_API_GNSS_SIG_BDS_B2I: u_sig = TSX_SIGNAL_BDSB2I     ; break;
  case LOC_API_GNSS_SIG_BDS_B3I: u_sig = TSX_SIGNAL_BDSB3I     ; break;
  case LOC_API_GNSS_SIG_BDS_B2A: u_sig = TSX_SIGNAL_BDSB2AP    ; break;
  case LOC_API_GNSS_SIG_BDS_B1C: u_sig = TSX_SIGNAL_BDSB1CP    ; break;
  case LOC_API_GNSS_SIG_BDS_B2B: u_sig = TSX_SIGNAL_BDSB2BI    ; break;
  case LOC_API_GNSS_SIG_QZS_L1C: u_sig = TSX_SIGNAL_QZSSL1CA   ; break;
  case LOC_API_GNSS_SIG_QZS_L2C: u_sig = TSX_SIGNAL_QZSSL2CM   ; break;
  case LOC_API_GNSS_SIG_QZS_L5Q: u_sig = TSX_SIGNAL_QZSSL5Q    ; break;
   default: u_sig = TSX_SIGNAL_INVALID; break;
  }
  return u_sig;
}

static TSXSignalType tsx_cvt_ApiSignal2Signal_Nav(const  loc_api_SignalType src)
{
  TSXSignalType u_sig = TSX_SIGNAL_INVALID;
  switch (src)
  {
  case LOC_API_GNSS_SIG_MAX    : u_sig = TSX_SIGNAL_INVALID    ; break;
  case LOC_API_GNSS_SIG_GPS_L1C: u_sig = TSX_SIGNAL_GPSL1CA    ; break;
  case LOC_API_GNSS_SIG_GPS_L2C: u_sig = TSX_SIGNAL_GPSL2CM    ; break;
  case LOC_API_GNSS_SIG_GPS_L5Q: u_sig = TSX_SIGNAL_GPSL5Q     ; break;
  case LOC_API_GNSS_SIG_GAL_E1 : u_sig = TSX_SIGNAL_GALE1B     ; break; // Only used for navigation data
  case LOC_API_GNSS_SIG_GAL_E5A: u_sig = TSX_SIGNAL_GALE5AQ    ; break;
  case LOC_API_GNSS_SIG_GAL_E5B: u_sig = TSX_SIGNAL_GALE5BQ    ; break;
  case LOC_API_GNSS_SIG_BDS_B1I: u_sig = TSX_SIGNAL_BDSB1I     ; break;
  case LOC_API_GNSS_SIG_BDS_B2I: u_sig = TSX_SIGNAL_BDSB2I     ; break;
  case LOC_API_GNSS_SIG_BDS_B3I: u_sig = TSX_SIGNAL_BDSB3I     ; break;
  case LOC_API_GNSS_SIG_BDS_B2A: u_sig = TSX_SIGNAL_BDSB2AP    ; break;
  case LOC_API_GNSS_SIG_BDS_B1C: u_sig = TSX_SIGNAL_BDSB1CP    ; break;
  case LOC_API_GNSS_SIG_BDS_B2B: u_sig = TSX_SIGNAL_BDSB2BI    ; break;
  case LOC_API_GNSS_SIG_QZS_L1C: u_sig = TSX_SIGNAL_QZSSL1CA   ; break;
  case LOC_API_GNSS_SIG_QZS_L2C: u_sig = TSX_SIGNAL_QZSSL2CM   ; break;
  case LOC_API_GNSS_SIG_QZS_L5Q: u_sig = TSX_SIGNAL_QZSSL5Q    ; break;
   default: u_sig = TSX_SIGNAL_INVALID; break;
  }
  return u_sig;
}

/**
 * @brief convert the satellite coordinate from the transmit time to receiver time
 * @param[in] pd_siteEcef the site coordinate in ECEF 
 * @param[in] pd_satEcef the satellite coordinate of transmit time in ECEF
 * @param[out] pd_satEcefRot the satellite coordinate of receiver time in ECEF
 * @return void
 */
void tsx_gnss_satRot(const double pd_siteEcef[3], const double pd_satEcef[3], double pd_satEcefRot[3])
{
    uint8_t u_i = 0;
    double pd_deltaCoor[3] = { 0.0 };
    double d_dist = 0.0;
    double d_rotateAngle = 0.0;
    double d_sinRotAngle = 0.0;
    double d_cosRotAngle = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
        pd_satEcefRot[u_i] = 0.0;
    }
    // correct for earth rotation apply ClockWise(negative rotation)
    pd_deltaCoor[0] = pd_satEcef[0] - pd_siteEcef[0];
    pd_deltaCoor[1] = pd_satEcef[1] - pd_siteEcef[1];
    pd_deltaCoor[2] = pd_satEcef[2] - pd_siteEcef[2];
    d_dist = sqrt(pd_deltaCoor[0] * pd_deltaCoor[0] + pd_deltaCoor[1] * pd_deltaCoor[1] + pd_deltaCoor[2] * pd_deltaCoor[2]);
    d_rotateAngle = (d_dist / TSX_CLIGHT) * TSX_OMGE_GPS;
    d_sinRotAngle = sin(d_rotateAngle);
    d_cosRotAngle = cos(d_rotateAngle);
    pd_satEcefRot[0] = pd_satEcef[0] * d_cosRotAngle + pd_satEcef[1] * d_sinRotAngle;
    pd_satEcefRot[1] = -pd_satEcef[0] * d_sinRotAngle + pd_satEcef[1] * d_cosRotAngle;
    pd_satEcefRot[2] = pd_satEcef[2];
}


/**
 * @brief inner product
 * @param a[in]
 * @param b[in] vector a,b (n x 1)
 * @param n[in] size of vector a,b
 * @return
 */
extern double tsx_gnss_Dot(const double* a, const double* b, int n)
{
  double c = 0.0;

  while (--n >= 0) c += a[n] * b[n];
  return c;
}

/**
 * @brief euclid norm
 * @param a[in] vector a (n x 1)
 * @param n[in] size of vector a
 * @return || a ||
 */
extern double tsx_gnss_Norm(const double* a, int n)
{
  return sqrt(tsx_gnss_Dot(a, a, n));
}

static double tsx_calOribitLosFromDeltaXYZ(const double* d_roverPos, const double* d_satPos, const TSXVectorECEF* delta_position)
{
  double d_delta[3] = {0.0};
  double d_ephSatPosRot[3] = {0.0};
  double d_preSatPosRot[3] = {0.0};
  double d_preSatPos[3] = {0.0};
  double d_ephRange = 0.0;
  double d_preRange = 0.0;
  double d_eEph[3] = {0.0};
  double d_ePre[3] = {0.0};
  double los = 0.0;

  d_delta[0] = delta_position->x;
  d_delta[1] = delta_position->y;
  d_delta[2] = delta_position->z;

  d_preSatPos[0] = d_satPos[0] + d_delta[0];
  d_preSatPos[1] = d_satPos[1] + d_delta[1];
  d_preSatPos[2] = d_satPos[2] + d_delta[2];

  tsx_gnss_satRot(d_roverPos, d_satPos, d_ephSatPosRot);
  tsx_gnss_satRot(d_roverPos, d_preSatPos, d_preSatPosRot);

  d_eEph[0] = d_roverPos[0] - d_ephSatPosRot[0];
  d_eEph[1] = d_roverPos[1] - d_ephSatPosRot[1];
  d_eEph[2] = d_roverPos[2] - d_ephSatPosRot[2];
  d_ePre[0] = d_roverPos[0] - d_preSatPosRot[0];
  d_ePre[1] = d_roverPos[1] - d_preSatPosRot[1];
  d_ePre[2] = d_roverPos[2] - d_preSatPosRot[2];

  d_ephRange = tsx_gnss_Norm(d_eEph, 3);
  d_preRange = tsx_gnss_Norm(d_ePre, 3);

  los = d_preRange - d_ephRange;

  return los;
}

static const loc_api_SatPosInfo_t *tsx_FindSatInfo(const loc_api_GnssMeasSatBlock_t *pz_ApiGnssMeasSatBlk,
  loc_api_gnssConstellationType u_constellation, uint8_t u_svid)
{

  for (uint8_t i = 0; i < pz_ApiGnssMeasSatBlk->u_satCount; i++)
  {
    if (pz_ApiGnssMeasSatBlk->z_SatPosInfo[i].u_constellation == u_constellation &&
        pz_ApiGnssMeasSatBlk->z_SatPosInfo[i].u_svid == u_svid)
    {
      return &pz_ApiGnssMeasSatBlk->z_SatPosInfo[i];
    }
  }
  return NULL;
}
#endif // 0

/**
 * @brief Get the ionospheric coefficient base the L1 frequency, 1TECU
 * @param[in] constellation
 * @return 1 tecu = () [m], base L1 of evry constellation
 */
static double tsx_base_L1_1tecu(uint8_t constellation)
{
  static double g_stec_gpsl1_1tecu = 40.31 * (1.0e+16) /  (1.57542E9 * 1.57542E9);   /* L1/E1/B1C  frequency (Hz) */
  static double g_stec_glol1_1tecu = 40.31 * (1.0e+16) /  (1.60200E9 * 1.60200E9);   /* GLONASS G1 base frequency (Hz) */
  static double g_stec_bdsl1_1tecu = 40.31 * (1.0e+16) /  (1.561098E9 * 1.561098E9); /* BDS B1I     frequency (Hz) */
         
  if (LOC_API_GNSS_GPS == constellation)
  {
    return g_stec_gpsl1_1tecu;
  }
  else if (LOC_API_GNSS_GLO == constellation)
  {
    return g_stec_glol1_1tecu;
  }
  else if (LOC_API_GNSS_BDS == constellation)
  {
    return g_stec_bdsl1_1tecu;
  }
  else if (LOC_API_GNSS_GAL == constellation)
  {
    return g_stec_gpsl1_1tecu;
  }
  else if (LOC_API_GNSS_QZS == constellation)
  {
    return g_stec_gpsl1_1tecu;
  }
  return 1.0;
}

static double tsx_TimeDiff(const loc_api_GpsTime_t *z_time1, const loc_api_GpsTime_t *z_time2)
{
  double d_t1 = z_time1->w_week * 86400.0 * 7.0 + z_time1->tow;
  double d_t2 = z_time2->w_week * 86400.0 * 7.0 + z_time2->tow;

  return (d_t1 - d_t2);
}

static void tsx_cvt_SatCorr2ApiLosBlk(const loc_api_GnssMeasSatBlock_t *pz_ApiGnssMeasSatBlk,
                                      const double *d_pos,
                                      const TSXSatelliteCorrectionsCollect_t *pz_tsxSatCol,
                                      loc_api_ssrLosBlock_t *pz_ApiLosBlk)
{
  loc_api_GpsTime_t z_time = {0};
  loc_api_GpsTime_t z_tor = {pz_ApiGnssMeasSatBlk->w_week, (double)(pz_ApiGnssMeasSatBlk->q_towMsec * 0.001)};

  pz_ApiLosBlk->u_version = VERSION_LOC_API_SSR_LOS_BLOCK;
  pz_ApiLosBlk->w_size = sizeof(loc_api_ssrLosBlock_t);
  
  /* time */
  pz_ApiLosBlk->z_epochLosInfo.z_tor.w_week = pz_ApiGnssMeasSatBlk->w_week;
  pz_ApiLosBlk->z_epochLosInfo.z_tor.tow = (double)pz_ApiGnssMeasSatBlk->q_towMsec * 0.001;

  /* not tide correction, copy from AS HPP algo  */
  pz_ApiLosBlk->d_siteCoor[0] = d_pos[0]; 
  pz_ApiLosBlk->d_siteCoor[1] = d_pos[1];
  pz_ApiLosBlk->d_siteCoor[2] = d_pos[2];
  pz_ApiLosBlk->z_epochLosInfo.u_satCount =
      pz_tsxSatCol->u_satCount > LOC_API_SSR_SAT_NUM_LIMIT ? LOC_API_SSR_SAT_NUM_LIMIT : pz_tsxSatCol->u_satCount;

  /* Init Constellation, svid */
  for (uint8_t i = 0; i < pz_tsxSatCol->u_satCount && i < LOC_API_SSR_SAT_NUM_LIMIT; i++)
  {
    loc_api_satSsrLos_t* pz_satLos = &pz_ApiLosBlk->z_epochLosInfo.z_satLosCorr[i];
    const TSXSatelliteCorrectionsEntry* pz_satCorr = &pz_tsxSatCol->z_sat_corr[i];

    pz_satLos->u_constellation = tsx_cvt_TsxSys2ApiSys(pz_satCorr->corrections.constellation);
    pz_satLos->u_svid = pz_satCorr->corrections.svid;
    pz_satLos->q_iode = -1;
    pz_satLos->z_satPosVelClkBrdc.q_iode = -1;
  }

  /* Orbit, clock */
  for (uint8_t i = 0; i < pz_tsxSatCol->u_satCount && i < LOC_API_SSR_SAT_NUM_LIMIT; i++)
  {
    loc_api_satSsrLos_t* pz_satLos = &pz_ApiLosBlk->z_epochLosInfo.z_satLosCorr[i];
    const TSXSatelliteCorrectionsEntry* pz_satCorr = &pz_tsxSatCol->z_sat_corr[i];
    const TSXSatelliteCorrectionsData* pz_corc = &pz_satCorr->corrections;

    /* Orbit*/
    if (pz_corc->have_orbit)
    {
      pz_satLos->u_orbClkMask |= LOC_API_SSR_SAT_ORB_CLK_PRE_ORBIT;
      tsx_cvt_time2ApiTime(&pz_corc->orbit_reference_time, &z_time); // tot - refer
      double dt = tsx_TimeDiff(&z_tor, &z_time);
      dt -= 0.0767; // 0.0767 is transfer time(P = 2.3e7 [m]])
      double delta_vt[3] = {pz_corc->delta_position_dot.x * dt,
                            pz_corc->delta_position_dot.y * dt,
                            pz_corc->delta_position_dot.z * dt};
      pz_satLos->z_orbclkPrec.q_delta_pos[0] = -(int32_t)round((pz_corc->delta_position.x + delta_vt[0]) * 1000.0);
      pz_satLos->z_orbclkPrec.q_delta_pos[1] = -(int32_t)round((pz_corc->delta_position.y + delta_vt[1]) * 1000.0);
      pz_satLos->z_orbclkPrec.q_delta_pos[2] = -(int32_t)round((pz_corc->delta_position.z + delta_vt[2]) * 1000.0);
      pz_satLos->z_orbclkPrec.f_qi += HARD_CODE_ORBIT_SIGMA * 1000.0;
      pz_satLos->q_iode = (int32_t)pz_corc->iode;
      pz_satLos->z_orbClkTime = tsx_ApiTimeMin(&pz_satLos->z_orbClkTime, &z_time);

    }
    /* Clock */
    if (pz_tsxSatCol->z_sat_corr[i].corrections.have_clock_bias)
    {
      pz_satLos->u_orbClkMask |= LOC_API_SSR_SAT_ORB_CLK_PRE_CLOCK;
      pz_satLos->z_orbclkPrec.q_clock_bias = (int32_t)round(pz_corc->clock_bias * 1000.0);
      pz_satLos->z_orbclkPrec.f_qi += HARD_CODE_CLOCK_SIGMA * 1000.0;
      pz_satLos->u_clockContinuityIod = pz_corc->clock_bias_discontinuity_iod;
      tsx_cvt_time2ApiTime(&pz_corc->orbit_reference_time, &z_time);
      // pz_satLos->z_orbClkTime = tsx_ApiTimeMin(&pz_satLos->z_orbClkTime, &z_time);
      pz_satLos->z_orbClkTime = z_time;
    }
    /* Orbit clock integrity */
    pz_satLos->z_orbclkPrec.u_preItg = tsx_cvt_IntegFlag2ApiIntegFlag(pz_satCorr->integrity.ocb_health);
    pz_satLos->z_orbclkPrec.u_postItg = tsx_cvt_IntegFlag2ApiIntegFlag(pz_satCorr->integrity.ocb_health);
  }

  /* STEC */
  for (uint8_t i = 0; i < pz_tsxSatCol->u_satCount && i < LOC_API_SSR_SAT_NUM_LIMIT; i++)
  {
    loc_api_satSsrLos_t* pz_satLos = &pz_ApiLosBlk->z_epochLosInfo.z_satLosCorr[i];
    const TSXSatelliteCorrectionsEntry* pz_satCorr = &pz_tsxSatCol->z_sat_corr[i];
	  const TSXSatelliteCorrectionsData* pz_corc = &pz_satCorr->corrections;
    if (!pz_tsxSatCol->z_sat_corr[i].corrections.have_iono)
    {
      continue;
    }
    double gps_l1_1tecu = tsx_base_L1_1tecu(LOC_API_GNSS_GPS);
    if(1.0 == gps_l1_1tecu)
    {
      continue;
    }
    pz_satLos->u_atmoMask |= LOC_API_SSR_ATMO_STEC_CORR;
    tsx_cvt_time2ApiTime(&pz_corc->iono_reference_time, &z_time);
    pz_satLos->z_STECtime = z_time;
    pz_satLos->z_stec.q_corr = -(int32_t)round(pz_corc->slant_iono_delay / gps_l1_1tecu * 1000.0);
    pz_satLos->z_stec.f_qi = (float)(pz_corc->slant_iono_delay_std_dev / gps_l1_1tecu);
    pz_satLos->z_stec.u_preItg = tsx_cvt_IntegFlag2ApiIntegFlag(pz_satCorr->integrity.iono_health);
    pz_satLos->z_stec.u_postItg = tsx_cvt_IntegFlag2ApiIntegFlag(pz_satCorr->integrity.iono_health);
  }

  /* Bias */
  for (uint8_t i = 0; i < pz_tsxSatCol->u_satCount && i < LOC_API_SSR_SAT_NUM_LIMIT; i++)
  {
    uint8_t idx = 0;
    loc_api_SignalType signal = LOC_API_GNSS_SIG_MAX;
    loc_api_satSsrLos_t* pz_satLos = &pz_ApiLosBlk->z_epochLosInfo.z_satLosCorr[i];
    const TSXSatelliteCorrectionsEntry* pz_satCorr = &pz_tsxSatCol->z_sat_corr[i];
	  const TSXSatelliteCorrectionsData* pz_corc = &pz_satCorr->corrections;
    /* Bias code */
    for (uint8_t j = 0; j < pz_corc->number_of_code_biases &&
                        pz_satLos->u_signalNum < LOC_API_SSR_SAT_CHL_NUM_LIMIT; j++)
    {
      idx = pz_satLos->u_signalNum;
      signal = tsx_cvt_Signal2ApiSignal(pz_corc->code_bias[j].signal_type);
      if(signal == LOC_API_GNSS_SIG_MAX) continue;
      pz_satLos->z_signalBiasCorr[idx].u_signalType = signal;
      pz_satLos->z_signalBiasCorr[idx].z_codeBias.q_corr = (int32_t)round(pz_corc->code_bias[j].bias * 1000.0);
      pz_satLos->z_signalBiasCorr[idx].u_biasMask |= LOC_API_SSR_SAT_BIAS_CODE_CORR;
      ++pz_satLos->u_signalNum;
    }
    /* Bias phase */
    for (uint8_t j = 0; j < pz_corc->number_of_phase_biases; j++)
    {
      const TSXPhaseBias* pz_CorrPhaseBias = &pz_corc->phase_bias[j];

      signal = tsx_cvt_Signal2ApiSignal(pz_CorrPhaseBias->signal_type);
      if(signal == LOC_API_GNSS_SIG_MAX) continue;
      idx = pz_satLos->u_signalNum;
      for (uint8_t k = 0; k < pz_satLos->u_signalNum; k++)
      {
        if(signal == pz_satLos->z_signalBiasCorr[k].u_signalType)
        {
          idx = k;
          break;
        }
      }
      if (idx == LOC_API_SSR_SAT_CHL_NUM_LIMIT)
      {
        break;
      }
      pz_satLos->z_signalBiasCorr[idx].z_phaseBias.q_corr = (int32_t)round(pz_CorrPhaseBias->bias * 1000.0);
      if(pz_CorrPhaseBias->is_integer_ar_capable)
      {
        pz_satLos->z_signalBiasCorr[idx].u_biasMask |= LOC_API_SSR_SAT_BIAS_PHASE_CORR;
      }
      pz_satLos->z_signalBiasCorr[idx].u_discontinuityIod = pz_CorrPhaseBias->discontinuity_iod;
      if(idx == pz_satLos->u_signalNum)
      {
        ++pz_satLos->u_signalNum;
      }
    }
  }

  /* PCV */
  for (uint8_t i = 0; i < pz_tsxSatCol->u_satCount && i < LOC_API_SSR_SAT_NUM_LIMIT; i++)
  {
    loc_api_satSsrLos_t* pz_satLos = &pz_ApiLosBlk->z_epochLosInfo.z_satLosCorr[i];
    const TSXSatelliteCorrectionsEntry* pz_satCorr = &pz_tsxSatCol->z_sat_corr[i];
	  const TSXSatelliteCorrectionsData* pz_corc = &pz_satCorr->corrections;
	
    if(pz_corc->number_of_pcv_corrections > 0)
    {
      tsx_cvt_time2ApiTime(&pz_corc->iono_reference_time, &z_time);
      pz_satLos->z_pcvTime = z_time;
      for (uint8_t j = 0; j < pz_corc->number_of_pcv_corrections; j++)
      {
        loc_api_gnssFreqType u_apiFrq = tsx_cvt_frq2ApiFrq(pz_corc->pcv[j].frequency);

        for (uint8_t k = 0; k < pz_satLos->u_signalNum; k++)
        {
          loc_api_gnssFreqType u_apiFrq_2 = tsx_cvt_ApiSig2ApiFreqType(pz_satLos->z_signalBiasCorr[k].u_signalType);
          if (u_apiFrq == u_apiFrq_2)
          {
            pz_satLos->z_signalBiasCorr[k].f_pcv = (float)pz_corc->pcv[j].variation;
            break;
          }
        }
      }
    }
  }
}

/**
 * @brief SSR get LOS by call TSX adapter and inject to Algo
 * @param[in] pz_LocGnssMeasBlk
 * @param[in,out] pz_SsrCorBlk
 */
extern void tsx_inject_ApiGnssMeasSat(const loc_api_GnssMeasSatBlock_t *pz_ApiGnssMeasSatBlk)
{
  g_w_week = pz_ApiGnssMeasSatBlk->w_week;
  if(tsx_handle == NULL)
  {
    return;
  }
  static uint32_t w_preSec = 0;
  /* Update gnss time,  interval 1s */
  uint32_t w_sec = (uint32_t)(pz_ApiGnssMeasSatBlk->q_towMsec * 0.001);
  if(w_sec == w_preSec)
  {
    LOG_F(INFO, "time replay %u ms", w_sec);
    return;
  }
  TSXGNSSTime z_tsx_time = {pz_ApiGnssMeasSatBlk->w_week, w_sec};
  
  TSXAdapterResult status_upd_time = TSX_UpdateGNSSTime(tsx_handle, z_tsx_time);

  /* Update user position */
  double d_pos[3] = {pz_ApiGnssMeasSatBlk->d_xyz[0] , pz_ApiGnssMeasSatBlk->d_xyz[1], pz_ApiGnssMeasSatBlk->d_xyz[2]};
  TSXUserPosition z_tsx_pos = {{d_pos[0], d_pos[1], d_pos[2]}, z_tsx_time};
  TSXAdapterResult status_upd_pos = TSX_UpdateUserPosition(tsx_handle, z_tsx_pos);

  if (status_upd_time != TSXADAPTER_SUCCESS || status_upd_pos != TSXADAPTER_SUCCESS)
  {
    LOG_F(INFO, "Update time or pos error, %u ms", w_sec);
    return;
  }

  /* Get correction data from adapter */
  TSXConstellationType tsx_sys = TSX_CONSTELLATION_UNKNOWN;
  TSXSatelliteCorrectionsCollect_t *pz_tsxSatCol = (TSXSatelliteCorrectionsCollect_t *)malloc(sizeof(TSXSatelliteCorrectionsCollect_t));
  memset(pz_tsxSatCol, 0, sizeof(TSXSatelliteCorrectionsCollect_t));

  for (uint8_t i = 0; i < pz_ApiGnssMeasSatBlk->u_satCount; i++)
  {
    tsx_sys = tsx_cvt_ApiSys2TsxSys(pz_ApiGnssMeasSatBlk->z_SatPosInfo[i].u_constellation);
    if (TSX_CONSTELLATION_UNKNOWN == tsx_sys)
    {
      continue;
    }
    TSXSatelliteCorrectionsEntry* pz_satCorr = &pz_tsxSatCol->z_sat_corr[pz_tsxSatCol->u_satCount];
    /* Get satellite correction entry*/
    TSXAdapterResult rc = TSX_GetSatelliteCorrectionsEntry(tsx_handle, tsx_sys, pz_ApiGnssMeasSatBlk->z_SatPosInfo[i].u_svid, pz_satCorr);

    if (rc == TSXADAPTER_SUCCESS)
    {
      if (
          pz_satCorr->corrections.have_clock_bias ||
          pz_satCorr->corrections.have_iono ||
          pz_satCorr->corrections.have_orbit)
      {
        ++pz_tsxSatCol->u_satCount;    
#if defined(TSX_DEBUG)
        if (g_tsx_config.fp_debug_log != NULL)
        {
          TSXSatelliteCorrectionsData *pz_corc = &pz_satCorr->corrections;
          fprintf(g_tsx_config.fp_debug_log, "\n", pz_corc->constellation);
          fprintf(g_tsx_config.fp_debug_log, "Constellation Type: %d\n", pz_corc->constellation);
          fprintf(g_tsx_config.fp_debug_log, "SVID: %d\n", pz_corc->svid);
          fprintf(g_tsx_config.fp_debug_log, "Meas Time: %u,%u\n", pz_ApiGnssMeasSatBlk->w_week, (uint32_t)(pz_ApiGnssMeasSatBlk->q_towMsec * 0.001));
          fprintf(g_tsx_config.fp_debug_log, "Clock bias: %f\n", pz_corc->clock_bias);
          fprintf(g_tsx_config.fp_debug_log, "clock_bias_reference_time: %u,%u\n", pz_corc->clock_bias_reference_time.week, pz_corc->clock_bias_reference_time.seconds);
          fprintf(g_tsx_config.fp_debug_log, "clock_bias_discontinuity_iod: %d\n", pz_corc->clock_bias_discontinuity_iod);
          fprintf(g_tsx_config.fp_debug_log, "iode: %d\n", pz_corc->iode);
          fprintf(g_tsx_config.fp_debug_log, "delta_position: x=%f, y=%f, z=%f\n", pz_corc->delta_position.x, pz_corc->delta_position.y, pz_corc->delta_position.z);
          fprintf(g_tsx_config.fp_debug_log, "delta_position_dot: x=%f, y=%f, z=%f\n", pz_corc->delta_position_dot.x, pz_corc->delta_position_dot.y, pz_corc->delta_position_dot.z);
          fprintf(g_tsx_config.fp_debug_log, "orbit_reference_time: %u,%u\n", pz_corc->orbit_reference_time.week, pz_corc->orbit_reference_time.seconds);
          fprintf(g_tsx_config.fp_debug_log, "slant_iono_delay: %f\n", pz_corc->slant_iono_delay);
          fprintf(g_tsx_config.fp_debug_log, "slant_iono_delay_std_dev: %f\n", pz_corc->slant_iono_delay_std_dev);
          fprintf(g_tsx_config.fp_debug_log, "iono_reference_time: %u,%u\n", pz_corc->iono_reference_time.week, pz_corc->iono_reference_time.seconds);
          fprintf(g_tsx_config.fp_debug_log, "No.Code Biasess: %d\n", pz_corc->number_of_code_biases);
          for (size_t j = 0; j < pz_corc->number_of_code_biases; j++)
          {
            fprintf(g_tsx_config.fp_debug_log, "\tCode bias: %d, %f\n", pz_corc->code_bias[j].signal_type, pz_corc->code_bias[j].bias);
          }
          fprintf(g_tsx_config.fp_debug_log, "code_bias_reference_time: %u,%u\n", pz_corc->code_bias_reference_time.week, pz_corc->code_bias_reference_time.seconds);
          fprintf(g_tsx_config.fp_debug_log, "No.phase Biases: %d\n", pz_corc->number_of_phase_biases);
          for (size_t j = 0; j < pz_corc->number_of_phase_biases; j++)
          {
            fprintf(g_tsx_config.fp_debug_log, "\tphase bias: %d, %f, %d, %d\n", pz_corc->phase_bias[j].signal_type, pz_corc->phase_bias[j].bias,
                    pz_corc->phase_bias[j].is_integer_ar_capable, pz_corc->phase_bias[j].discontinuity_iod);
          }
          fprintf(g_tsx_config.fp_debug_log, "number_of_pcv_corrections: %d\n", pz_corc->number_of_pcv_corrections);
           for (size_t j = 0; j < pz_corc->number_of_pcv_corrections; j++)
          {
            fprintf(g_tsx_config.fp_debug_log, "\tpcv: %d, %f\n", pz_corc->pcv[j].frequency, pz_corc->pcv[j].variation);
          }
          fprintf(g_tsx_config.fp_debug_log, "pcv_reference_time: %u,%u\n", pz_corc->pcv_reference_time.week, pz_corc->pcv_reference_time.seconds);
          fflush(g_tsx_config.fp_debug_log);
      }
#endif
      }
    }
    if (pz_tsxSatCol->u_satCount >= LOC_API_SSR_SAT_NUM_LIMIT)
    {
      break;
    }
  }

  if(pz_tsxSatCol->u_satCount != 0)
  {
    /* Convert to loc_api_ssrLosBlock_t */
    loc_api_ssrLosBlock_t *pz_ApiLosBlk = (loc_api_ssrLosBlock_t *)malloc(sizeof(loc_api_ssrLosBlock_t));
    memset(pz_ApiLosBlk, 0 , sizeof(loc_api_ssrLosBlock_t));
    tsx_cvt_SatCorr2ApiLosBlk(pz_ApiGnssMeasSatBlk, d_pos, pz_tsxSatCol, pz_ApiLosBlk);

    /* Inject losBlock  */
    loc_api_InjectSsrLosBlock(pz_ApiLosBlk);

    free(pz_ApiLosBlk);
  }

  free(pz_tsxSatCol);
}

static uint32_t
BitOp_EndianSwap32(
   uint32_t ulInput_)
{
   uint32_t ulResult;
   ulResult =  (ulInput_ & 0x000000ffU) << 24;
   ulResult |= (ulInput_ & 0x0000ff00U) << 8;
   ulResult |= (ulInput_ & 0x00ff0000U) >> 8;
   ulResult |= (ulInput_ & 0xff000000U) >> 24;
   return ulResult;
}


static uint32_t Memfunc_GetBit(
   const uint8_t* const pucSource_,
   const uint32_t ulSourceLength_,
   uint32_t ulStartBit_,
   const uint32_t ulLength_)
{
   uint32_t ulReturnValue = 0U;

   static const uint32_t ulBITS_PER_BYTE = 8U;

   // Check if the input data is within allowable size and buffer is valid
   if ((ulLength_ == 0UL) || (ulLength_ > 32UL) || (pucSource_ == NULL))
   {
      ulReturnValue = 0UL;
   }
   else
   {
      uint32_t ulSourceIndex = ulStartBit_ >> 3UL; // index into pucSource_[] where data starts
      ulStartBit_ &= 7UL; // bits into the first pucSource_[] element its starts at
      uint32_t ulMaxBytes = (ulLength_ + ulStartBit_ + 7U) >> 3U; // The maximum number of bytes needed to fulfill the length

      for(INT32 lDestIndex = 3; lDestIndex >= 0; lDestIndex--)
      {
         if ( (ulSourceIndex < ulSourceLength_) && (ulMaxBytes != 0U) )// prevent reading out of bound memory
         {
            ulReturnValue |= (static_cast<uint32_t>(pucSource_[ulSourceIndex])
               << (static_cast<uint32_t>(lDestIndex) * ulBITS_PER_BYTE));

            ulMaxBytes--;
         }

         ulSourceIndex++;
      }

      (void)ulMaxBytes;   // It is used where needed

      if (ulStartBit_ != 0UL) // if we are NOT byte aligned
      {
         ulReturnValue <<= ulStartBit_; // left justify

         if ((ulLength_ + ulStartBit_) > 32UL)
         {
            if (ulSourceIndex < ulSourceLength_) // prevent reading out of bound memory
            {
               ulReturnValue |= static_cast<uint32_t>(pucSource_[ulSourceIndex])
                  >> (ulBITS_PER_BYTE - ulStartBit_); // grab part of 5th byte
            }
         }
      }

      if (ulLength_ < 32U) // if we didn't want 32 bits
      {
         ulReturnValue >>= (32UL - ulLength_); // right justify result
      }
   }
   return ulReturnValue;
}

static void Memfunc_SetBit(
   uint8_t* const pucDest_,
   const uint32_t ulDestLength_,
   uint32_t ulStartBit_,
   const uint32_t ulLength_,
   const uint32_t ulValue_)
{
   static const uint8_t ucMASK_FF = 0xFFU;
   static const uint32_t ulMASK_00000007 = 0x7U;
   static const uint32_t ulBITS_IN_uint32_t = 32U;
   static const uint32_t ulBITS_PER_BYTE = 8U;

   if ((ulLength_ > 0UL)
      && (ulLength_ <= ulBITS_IN_uint32_t)
      && (pucDest_ != NULL))
   {
      INT32 lLength = static_cast<INT32>(ulLength_);
      // div 8 - which byte in pucDest_ to start at
      uint32_t ulByteIndex = ulStartBit_ >> 3U; // loop index
      // mod 8 - bits into the first byte to start at
      ulStartBit_ &= ulMASK_00000007;

      uint8_t ucMask; // bit mask for current byte in pucDest_[]
      // modify bytes while lLength_ remains
      uint32_t ulByteCount = 0U; // safety check to ensure max 5 cycles
      // loop up to 5 bytes
      while((lLength > 0) && (ulByteCount < 5U) && (ulByteIndex < ulDestLength_))
      {
         // First, form a mask byte for the current byte in pucDest_:
         // if we are NOT left aligned in the current byte
         if (ulStartBit_ != 0U)
         {
            if ((ulStartBit_ + static_cast<uint32_t>(lLength)) < ulBITS_PER_BYTE)
            {
               // if we don't need all the bits on the right of this byte
               const uint32_t ulShift = ulBITS_PER_BYTE
                  - static_cast<uint32_t>(lLength);
               ucMask = ucMASK_FF << ulShift;
               ucMask >>= ulStartBit_;
            }
            else
            {
               // else we are filling all the bits at the right
               ucMask = ucMASK_FF >> ulStartBit_;
            }
            // update lLength_ for bits we do this loop
            lLength -= (static_cast<INT32>(ulBITS_PER_BYTE)
               - static_cast<INT32>(ulStartBit_));
            // no left bit offset next time
            ulStartBit_ = 0U;
         }
         else // else ulStartBit_ is 0 (byte-aligned case)
         {
            ucMask = ucMASK_FF;
            // if we need less than a byte
            if (static_cast<uint32_t>(lLength) < ulBITS_PER_BYTE)
            {
               // shift to skip bit(s) on right
               ucMask <<= ulBITS_PER_BYTE - static_cast<uint32_t>(lLength);
            }
            // update ulLength_ for bits we do this loop
            lLength -= static_cast<INT32>(ulBITS_PER_BYTE);
         }

         // Now, clear out the existing bits in pucDest_ at the
         // current index and OR in the new bits from ulValue_:
         // clear out previous bits
         pucDest_[ulByteIndex] = pucDest_[ulByteIndex]
            & static_cast<uint8_t>(~ucMask);
         uint8_t ucByte = 0U;
         if (lLength >= 0) // if updated lLength is not negative
         {
            ucByte = static_cast<uint8_t>(ulValue_
               >> static_cast<uint32_t>(lLength));
         }
         else // lLength < 0 - must shift value LEFT
         {
            const INT32 lAbsLength = -lLength;
            ucByte = static_cast<uint8_t>(ulValue_
               << static_cast<uint32_t>(lAbsLength));
         }

         // OR in new bits
         pucDest_[ulByteIndex] = pucDest_[ulByteIndex] | (ucMask & ucByte);
         // increment byte index
         ulByteIndex++;
         // increment byte count
         ulByteCount++;
      }
      (void) ulByteIndex; // Discard value from last loop iteration
      (void) ulByteCount; // Discard value from last loop iteration
   }
   else
   {
      // nothing
   }
}

//-----------------------------------------------------------------------------
static TSXSignalType ConvertSignalType(uint32_t ulRTCMSignalMask_, TSXConstellationType eConstellation_, double* pdFrequencyMHz_)
{
   TSXSignalType eResult = TSX_SIGNAL_INVALID;
   const double dL1_CENTRE_FREQUENCY_MHZ = 1575.42;
   const double dL2_CENTRE_FREQUENCY_MHZ = 1227.60;
   const double dE5B_CENTRE_FREQUENCY_MHZ = 1207.140;
   const double dL5_CENTRE_FREQUENCY_MHZ = 1176.45;

   if(pdFrequencyMHz_ != NULL)
   {

      if(ulRTCMSignalMask_ == 0x40000000)
      {
         //Signal ID = 2
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GPS:
            eResult = TSX_SIGNAL_GPSL1CA;
            *pdFrequencyMHz_ = dL1_CENTRE_FREQUENCY_MHZ;
            break;
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE1B;
            *pdFrequencyMHz_ = dL1_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x10000000)
      {
         //Signal ID = 4
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE1B;
            *pdFrequencyMHz_ = dL1_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x08000000)
      {
         //Signal ID = 5
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE1B;
            *pdFrequencyMHz_ = dL1_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x00040000)
      {
         //Signal ID = 14
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE5BQ;
            *pdFrequencyMHz_ = dE5B_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x00020000)
      {
         //Signal ID = 15
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GPS:
            eResult = TSX_SIGNAL_GPSL2CM;
            *pdFrequencyMHz_ = dL2_CENTRE_FREQUENCY_MHZ;
            break;
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE5BQ;
            *pdFrequencyMHz_ = dE5B_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x00010000)
      {
         //Signal ID = 16
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GPS:
            eResult = TSX_SIGNAL_GPSL2Y;
            *pdFrequencyMHz_ = dL2_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x00000400)
      {
         //Signal ID = 22
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GPS:
            eResult = TSX_SIGNAL_GPSL5Q;
            *pdFrequencyMHz_ = dL5_CENTRE_FREQUENCY_MHZ;
            break;
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE5AQ;
            *pdFrequencyMHz_ = dL5_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else if(ulRTCMSignalMask_ == 0x00000200)
      {
         //Signal ID = 23
         switch(eConstellation_)
         {
         case TSX_CONSTELLATION_GPS:
            eResult = TSX_SIGNAL_GPSL5Q;
            *pdFrequencyMHz_ = dL5_CENTRE_FREQUENCY_MHZ;
            break;
         case TSX_CONSTELLATION_GALILEO:
            eResult = TSX_SIGNAL_GALE5AQ;
            *pdFrequencyMHz_ = dL5_CENTRE_FREQUENCY_MHZ;
            break;
         default:
            eResult = TSX_SIGNAL_INVALID;
            break;
         }
      }
      else
      {
         eResult = TSX_SIGNAL_INVALID;
      }
   }
   return eResult;
}


static bool
FillSubframeAndSize(
   uint8_t* subframe_data,
   uint32_t& subframe_size_bytes,
   const uint16_t FrameSize,
   const uint8_t* message,
   TSXConstellationType constellation,
   const unsigned long ulMessageSize)
{
   bool bResult = FALSE;

   uint32_t ulSTInvert = 0x3FFFFFC0U; // ST invert
   uint32_t ulMASK_00000003 = 0x00000003U;
   uint8_t ucMASK_BIT1 = 0x01U;
   uint32_t ulNUM_WORD_BITS = 30U;
   uint32_t ulWORDS_PER_SUBFRAME = 10U;
   uint32_t ulGPS_NAV_RAW_SUBFRAME_SIZE = static_cast<uint32_t>(38U);

   subframe_size_bytes = FrameSize / 8;
   if((FrameSize % 8) > 0)
   {
      subframe_size_bytes++;
   }

   uint32_t aulWords[10];
   uint32_t ulNumWords = static_cast<uint32_t>(FrameSize) / 32;
   if((FrameSize % 8) > 0)
   {
      ulNumWords++;
   }

   uint32_t ulTargetBits = ulNumWords * 30;
   uint32_t ulTargetBytes = ulTargetBits / 8;
   if((ulTargetBits % 8) > 0)
   {
      ulTargetBytes++;
   }

   if(ulMessageSize >= (15 + subframe_size_bytes))
   {
      //This assumes a single subframe per message. This will not
      //handle cases where the NDF message contains multiple subframes.
      uint32_t ulBitShift = 0;
      uint32_t ulStartOffset = 0;

      if(constellation == TSX_CONSTELLATION_GPS)
      {
         ulStartOffset = 118;
      }
      else
      {
         //Constellations other than GPS seem to have 2 bits of padding at
         // the start of the NDF payload.
         ulStartOffset = 120;
      }


      if((ulTargetBits % 8) > 0)
      {
         ulBitShift = 8 - (ulTargetBits % 8);
      }

      // The subframe is output as an array of little-endian 32-bit integers.
      // Since data words are only 30 bits for GPS, each of these contains
      // two bits of padding. We need to reorder this into an array of bytes
      // with no interstitial padding in order for the engine to deal with it.
      for(uint32_t ulIndex = 0; ulIndex < ulNumWords; ulIndex++)
      {
         aulWords[ulIndex] = Memfunc_GetBit(message, ulMessageSize, ulStartOffset + (32 * ulIndex), 32);
         aulWords[ulIndex] = BitOp_EndianSwap32(aulWords[ulIndex]);
      }


      if(constellation == TSX_CONSTELLATION_GPS)
      {
         uint8_t aucSubframeData[64];
         uint32_t ulFreeBit = 0;
         UINT64 ullWordBuffer = 0;
         INT32 lWordIndex = ulNumWords - 1;

         for(uint32_t ulIndex = ulTargetBytes; ulIndex > 0; ulIndex--)
         {
            if(ulFreeBit == 0 && lWordIndex >= 0)
            {
               ullWordBuffer = static_cast<UINT64>(aulWords[lWordIndex]);
               ulFreeBit = 30;
               ullWordBuffer <<= ulBitShift;
               ulFreeBit += ulBitShift;
               lWordIndex--;
            }

            if(ulFreeBit < 34)
            {
               if(lWordIndex >= 0)
               {
                  UINT64 ullTemp = 0;
                  ullTemp = static_cast<UINT64>(aulWords[lWordIndex]);
                  lWordIndex--;
                  ullTemp <<= ulFreeBit;
                  ullWordBuffer |= ullTemp;
                  ulFreeBit += 30;
               }
            }

            aucSubframeData[ulIndex - 1] = static_cast<uint8_t>(ullWordBuffer & 0xFF);
            ullWordBuffer >>= 8;
            ulFreeBit -= 8;
         }

         // Inverts words flipped by STMicro to align with expected output
         // ICD output for subframe words
         uint8_t ucLastWordD29D30; // First word used D29 = D30 = 0
         uint32_t ulBitOffset;
         uint32_t ulDataWordIn;

         ucLastWordD29D30 = 0U;
         for(uint32_t ulWordIndex = 0U; ulWordIndex < ulWORDS_PER_SUBFRAME; ulWordIndex++)
         {
            ulBitOffset = ulWordIndex * ulNUM_WORD_BITS;

            // Read input word
            // NOTE: Word bit order: MSB [0 0 D1 D2 .. D29 D30] LSB
            ulDataWordIn = Memfunc_GetBit(
               aucSubframeData,
               ulGPS_NAV_RAW_SUBFRAME_SIZE,
               ulBitOffset,
               ulNUM_WORD_BITS);

            // Inverts words flipped by STMicro
            if ((ucLastWordD29D30 & ucMASK_BIT1) == ucMASK_BIT1)
            {
               ulDataWordIn = ulDataWordIn ^ ulSTInvert;
            }

            ucLastWordD29D30 = static_cast<uint8_t>(ulDataWordIn & ulMASK_00000003);

            // Write output word to subframe buffer
            Memfunc_SetBit(
               subframe_data,
               ulGPS_NAV_RAW_SUBFRAME_SIZE,
               ulBitOffset,
               ulNUM_WORD_BITS,
               ulDataWordIn);
         }

         subframe_size_bytes = 38;
      }
      else if(constellation == TSX_CONSTELLATION_GALILEO)
      {
         if(FrameSize >= 228)
         {
            const uint32_t ulPayloadStart = 118;
            uint8_t ucPartial = 0;

            uint8_t aucSTStruct[29];

            for(uint32_t ulMessageByte = 0; ulMessageByte < 29; ulMessageByte++)
            {
               uint32_t ulSourceByte = ((ulMessageByte / 4) * 4) + (3 - (ulMessageByte % 4));

               if(ulMessageByte == 28)
               {
                  //The final byte can't be endian-swapped.
                  ulSourceByte = 28;
               }

               uint32_t ulStartBit = (ulSourceByte * 8) + ulPayloadStart;

               aucSTStruct[ulMessageByte] = Memfunc_GetBit(message, ulMessageSize, ulStartBit, 8);
            }

            //Load the even page
            for(uint32_t ulByteIndex = 0; ulByteIndex < 14; ulByteIndex++)
            {
               subframe_data[ulByteIndex] = aucSTStruct[ulByteIndex];
            }

            ucPartial = aucSTStruct[14];
            ucPartial &= 0xC0;
            subframe_data[14] = ucPartial;

            //Load the odd page
            for(uint32_t ulByteIndex = 15; ulByteIndex < 29; ulByteIndex++)
            {
               uint8_t ucPageByte = aucSTStruct[ulByteIndex] >> 6;
               ucPageByte |= (aucSTStruct[ulByteIndex - 1] << 2);
               subframe_data[ulByteIndex] = ucPageByte;
            }

            //NOTE: The above truncates the GAL I/NAV Reserved2 field, but that field
            // is excluded from the subframe CRC and is also suspected to be corrupted
            // in the ST NDF format. Force the length to 30 to "re-attach" the missing tail bits.
            subframe_size_bytes = 30;
         }
         else
         {
            //Earlier ST firmware had smaller payloads for Galileo NDF
            //which did not actually contain complete I/NAV data. These
            //are not supported by this decoder.
            bResult = FALSE;
         }
      }
      else
      {
         //Unsupported constellation.
         bResult = FALSE;
      }


      if(constellation == TSX_CONSTELLATION_GPS ||
         constellation == TSX_CONSTELLATION_GALILEO)
      {
         bResult = TRUE;
      }
   }

   return bResult;
}

/**
 * @brief TSX inject navigation
 * @param[in] pz_ApiNavData 
 */
extern void tsx_inject_navigation_data(loc_api_NavigationData_t *pz_ApiNavData)
{
  uint8_t SatSystem;
  uint8_t SatNumber;
  uint8_t SigType;
  uint16_t FrameSize;
  uint8_t subframe_data[64] = {0};
  uint32_t subframe_size_bytes;

  if(tsx_handle == NULL)
  {
    return;
  }

  if(g_nav_tow != pz_ApiNavData->tow)
  {
    g_nav_tow = pz_ApiNavData->tow;
  }

  uint8_t *buffer = pz_ApiNavData->buffer_rtcm;
  int recv = pz_ApiNavData->length_rtcm;
  SatSystem = Memfunc_GetBit(buffer, recv, 56, 4);
  SatNumber = Memfunc_GetBit(buffer, recv, 60, 6) + 1;
  SigType = Memfunc_GetBit(buffer, recv, 70, 5);
  FrameSize = Memfunc_GetBit(buffer, recv, 106, 12);

  uint32_t ulSignalMask = (uint32_t)(0x1) << (31 - SigType);

  TSXConstellationType stConstellation;

  switch (SatSystem)
  {
  case 0:
    stConstellation = TSX_CONSTELLATION_GPS;
    break;
  case 2:
    stConstellation = TSX_CONSTELLATION_GALILEO;
    break;
  default:
    stConstellation = TSX_CONSTELLATION_UNKNOWN;
    break;
  }

  if (stConstellation == TSX_CONSTELLATION_UNKNOWN)
  {
    printf("\n");
    return;
  }

  double fFrequency = 0.0;
  TSXSignalType stSignal = ConvertSignalType(
      ulSignalMask,
      stConstellation,
      &fFrequency);

  bool ret = FillSubframeAndSize(subframe_data, subframe_size_bytes, FrameSize, buffer, stConstellation, recv);
  if(!ret)
  {
    LOG_F(INFO, "Fill Subframe error %f", pz_ApiNavData->tow);
  }

  TSXAdapterResult rc = TSX_ProcessNavigationData(
      tsx_handle,
      stConstellation,
      stSignal,
      SatNumber,
      subframe_data,
      subframe_size_bytes);

  /* write to file */
#if defined(TSX_DEBUG)
  if(g_tsx_config.fp_subframe!=NULL && g_w_week!=0)
  {
    FILE* fp = g_tsx_config.fp_subframe;
    int32_t sec = (int)(pz_ApiNavData->tow / 1000);
    int32_t sec_ms = (pz_ApiNavData->tow - sec*1000);
    
    fprintf(fp, "{\n\t{%d, %d, %d},", g_w_week, sec, sec_ms);
    // sys, sig, svid, frq, length
    fprintf(fp, "%d, %d, %d, %d, %d", stConstellation, stSignal, SatNumber, 0, subframe_size_bytes);
    fprintf(fp, "{");
    for (size_t i = 0; i < subframe_size_bytes; i++)
    {
      if(i == subframe_size_bytes -1)
      {
        fprintf(fp, "%#x",subframe_data[i]);
        break;
      }
      fprintf(fp, "%#x,",subframe_data[i]);
    }
    fprintf(fp, "}");
    fprintf(fp,"\n},\n");

    fflush(fp);
  }
#endif

  if (rc == TSXADAPTER_SUCCESS)
  {
    LOG_F(INFO, "inject navigation data suc length=%d, %d, %d", pz_ApiNavData->length, pz_ApiNavData->sys, pz_ApiNavData->svid);
  }
  else if(rc == TSXADAPTER_PARTIAL)
  {
    LOG_F(DEBUG, "inject navigation data paritial length=%d, %d, %d", pz_ApiNavData->length, pz_ApiNavData->sys, pz_ApiNavData->svid);
  }
  else
  {
    LOG_F(DEBUG, "inject navigation data failed length=%d, rc=%d, %d, %d", pz_ApiNavData->length, rc, pz_ApiNavData->sys, pz_ApiNavData->svid);
  }
  return;
}


/**
 * @brief TSX inject correction data
 * @param[in] q_length 
 * @param[in] buffer 
 * @return int32_t status: see TSXAdapterResult
 */
extern int32_t tsx_injectCorrectionData(uint32_t q_length, const uint8_t* buffer)
{
#if defined(TSX_DEBUG)
  if (g_nav_tow != 0 && g_w_week!=0 && g_tsx_config.fp_correction != NULL)
  {
    FILE *fp = g_tsx_config.fp_correction;
  
    fprintf(fp, "{\n");
    fprintf(fp, "\t{%d, %d, 0.0}, %d,", g_w_week, (int)(g_nav_tow * 0.001), q_length);

    fprintf(fp, "{");
    for (uint32_t i = 0; i < q_length; i++)
    {
      if (i == q_length - 1)
      {
        fprintf(fp, "%#x", buffer[i]);
        break;
      }
      fprintf(fp, "%#x,", buffer[i]);
    }
    fprintf(fp, "}\n");

    fprintf(fp, "},\n");
    fflush(fp);
  }
#endif

  if(tsx_handle == NULL)
  {
    return (int32_t)(TSXADAPTER_ERR_GEOGRAPHICAL_MISMATCH + 1);
  }
  bool user_key_updated;
  TSXCorrectionsDataset* decoded_corrections = (TSXCorrectionsDataset*)malloc(sizeof(TSXCorrectionsDataset));
  memset(decoded_corrections, 0, sizeof(TSXCorrectionsDataset));
  TSXAdapterResult rc = TSX_ProcessCorrectionsData(tsx_handle, buffer, q_length, decoded_corrections, &user_key_updated);
  free(decoded_corrections);
  if (rc == TSXADAPTER_SUCCESS)
  {
    LOG_F(INFO, "inject correction data suc length=%d", q_length);
  }
  else if(rc == TSXADAPTER_PARTIAL)
  {
    LOG_F(DEBUG, "inject correction data paritial length=%d", q_length);
  }
  else
  {
    LOG_F(DEBUG, "inject correction data failed length=%d, %d", q_length, rc);
  }
  return (int32_t)rc;
}

#endif // APP_HEXAGON_SSR
