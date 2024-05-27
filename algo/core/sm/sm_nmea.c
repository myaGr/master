/**@file        sm_nmea.c
 * @brief
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/09/24  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "cmn_utils.h"
#include "gnss_common.h"
#include "sm_nmea.h"

#ifndef NMEA_VEL_KNOT
#define NMEA_VEL_KNOT	(0.5144)
#endif

#define NMEA_KNOT2KMH	(1.852)

/**
 *  @brief add checksum after a nmea string
 *  @param[in] nmea_str
 *  @param[out] len
 */
static BOOL sm_nmea_add_checksum(char* nmea_str, uint32_t len)
{
  uint32_t    i = 0;
  uint32_t    nmea_str_length = 0;
  uint8_t     sys_check;

  /* inputs check */
  if (NULL == nmea_str) {
    return false;
  }

  /* cal the sys check */
  nmea_str_length = (int)strlen(nmea_str);

  /* check the len */
  if (nmea_str_length > len - 5) {
    return false;
  }

  sys_check = nmea_str[1];
  for (i = 2; i < nmea_str_length; i++)
  {
    sys_check ^= nmea_str[i];
  }

  sprintf(&nmea_str[nmea_str_length], "%1c%02X%c%c", '*', sys_check, 0x0D, 0x0A);

  return TRUE;
}

/**
 * @brief create gga string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gga string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gga(const gnss_PositionFix_t* pz_PositionFix, uint8_t* gga)
{
  char gga_buf[256] = { 0 };
  uint32_t q_index = 0;
  EpochTime_t z_Epoch = { 0 };
  float f_hdop = M_MIN(99.9f, pz_PositionFix->z_dops.f_hdop);
  char ageOfDiffStr[256] = { 0 };
  char staid[16] = { 0 };

  tm_cvt_GpstToEpoch(&pz_PositionFix->z_gpsTime, &z_Epoch);

  q_index += sprintf(gga_buf + q_index, "$GNGGA,");

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&
    (0 == pz_PositionFix->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(gga_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(gga_buf + q_index, "%02u%02u%02u.%03u,",
      (uint32_t)z_Epoch.hour, (uint32_t)z_Epoch.min, (uint32_t)z_Epoch.second,
      (uint32_t)gnss_Round(1000 * ((double)z_Epoch.second - (uint32_t)z_Epoch.second),0));
  }

  if ((GNSS_FIX_FLAG_SPS == pz_PositionFix->u_fixFlag) || (GNSS_FIX_FLAG_DR == pz_PositionFix->u_fixFlag))
  {
    sprintf(ageOfDiffStr, "");
    sprintf(staid, "");
  }
  else if ((GNSS_FIX_FLAG_DGNSS == pz_PositionFix->u_fixFlag) ||
    (GNSS_FIX_FLAG_FIXED == pz_PositionFix->u_fixFlag) ||
    (GNSS_FIX_FLAG_FLOATING == pz_PositionFix->u_fixFlag))
  {
    sprintf(ageOfDiffStr, "%.1f", (pz_PositionFix->z_rtk.f_age < 3600.0 && pz_PositionFix->z_rtk.f_age >= 0.0) ? pz_PositionFix->z_rtk.f_age : 0.0);
    sprintf(staid, "%04u", (pz_PositionFix->z_rtk.q_StationID > 0 && pz_PositionFix->z_rtk.q_StationID < 9999) ? pz_PositionFix->z_rtk.q_StationID : 0);
  }

  /* lat, lon, alt item */
  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    /*q_index += */sprintf(gga_buf + q_index, ",,,,0,,,,,,,,");
  }
  else
  {
    double lat = pz_PositionFix->d_lla[0] * RAD2DEG;
    double lon = pz_PositionFix->d_lla[1] * RAD2DEG;

    if (lat > 180.0) {
      lat -= 180.0;
    }

    if (lon > 180) {
      lon -= 180.0;
    }
    char lat_flag = (lat >= 0) ? 'N' : 'S';
    char lon_flag = (lon >= 0) ? 'E' : 'W';

    if (lat < 0) {
      lat = -lat;
    }

    if (lon < 0) {
      lon = -lon;
    }

    q_index += sprintf(gga_buf + q_index, "%02u%09.6lf,%c,%03u%09.6lf,%c,",
      (uint32_t)lat, (lat - (uint32_t)lat) * 60.0, lat_flag,
      (uint32_t)lon, (lon - (uint32_t)lon) * 60.0, lon_flag);
    q_index += sprintf(gga_buf + q_index, "%u,", (uint32_t)(pz_PositionFix->u_fixFlag));
    q_index += sprintf(gga_buf + q_index, "%u,", (uint32_t)(pz_PositionFix->z_SvStatus.u_SvInUseCount));
    q_index += sprintf(gga_buf + q_index, "%.1f,", f_hdop);
    q_index += sprintf(gga_buf + q_index, "%.3lf,M,%.3lf,M,",
                       pz_PositionFix->d_lla[2] - pz_PositionFix->d_quasiGeoidHeight,
                       pz_PositionFix->d_quasiGeoidHeight);
    /*q_index += */sprintf(gga_buf + q_index, "%s,%s", ageOfDiffStr, staid);
  }

  sm_nmea_add_checksum(gga_buf, sizeof(gga_buf));

  memcpy(gga, gga_buf, strlen(gga_buf));
  return TRUE;
}

/**
 * @brief create gga string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gga string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gga_ConsoildatedPositionFix(const loc_api_ConsoildatedPositionFix_t* pz_PositionFix, uint8_t* gga)
{
  char gga_buf[256] = { 0 };
  uint32_t q_index = 0;
  EpochTime_t z_Epoch = { 0 };
  GpsTime_t z_GpsTime = { 0 };
  char ageOfDiffStr[256] = { 0 };
  char staid[16] = { 0 };

  tm_cvt_SetGpst(&z_GpsTime, pz_PositionFix->w_week, 0.001 * pz_PositionFix->q_towMsec);
  tm_cvt_GpstToEpoch(&z_GpsTime, &z_Epoch);

  q_index += sprintf(gga_buf + q_index, "$GNGGA,");

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&
      (0 == z_GpsTime.t_fullMsec))
  {
    q_index += sprintf(gga_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(gga_buf + q_index, "%02u%02u%02u.%03u,",
      (uint32_t)z_Epoch.hour, (uint32_t)z_Epoch.min, (uint32_t)z_Epoch.second,
      (uint32_t)gnss_Round(1000 * ((double)z_Epoch.second - (uint32_t)z_Epoch.second), 0));
  }

  if ((GNSS_FIX_FLAG_SPS == pz_PositionFix->u_fixFlag) || (GNSS_FIX_FLAG_DR == pz_PositionFix->u_fixFlag))
  {
    sprintf(ageOfDiffStr, "");
    sprintf(staid, "");
  }
  else if ((GNSS_FIX_FLAG_DGNSS == pz_PositionFix->u_fixFlag) ||
    (GNSS_FIX_FLAG_FIXED == pz_PositionFix->u_fixFlag) ||
    (GNSS_FIX_FLAG_FLOATING == pz_PositionFix->u_fixFlag))
  {
    sprintf(ageOfDiffStr, "%.1f", (pz_PositionFix->f_age < 3600.0 && pz_PositionFix->f_age >= 0.0) ? pz_PositionFix->f_age : 0.0);
    sprintf(staid, "%04u", (pz_PositionFix->q_StationID > 0 && pz_PositionFix->q_StationID < 9999) ? pz_PositionFix->q_StationID : 0);
  }

  /* lat, lon, alt item */
  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    /*q_index += */sprintf(gga_buf + q_index, ",,,,0,,,,,,,,");
  }
  else
  {
    double lat = pz_PositionFix->d_lla[0] * RAD2DEG;
    double lon = pz_PositionFix->d_lla[1] * RAD2DEG;

    if (lat > 180.0)
    {
      lat -= 180.0;
    }

    if (lon > 180)
    {
      lon -= 180.0;
    }
    char lat_flag = (lat >= 0) ? 'N' : 'S';
    char lon_flag = (lon >= 0) ? 'E' : 'W';

    if (lat < 0)
    {
      lat = -lat;
    }

    if (lon < 0)
    {
      lon = -lon;
    }

    q_index += sprintf(gga_buf + q_index, "%02u%09.6lf,%c,%03u%09.6lf,%c,",
                       (uint32_t)lat, (lat - (uint32_t)lat) * 60.0, lat_flag,
                       (uint32_t)lon, (lon - (uint32_t)lon) * 60.0, lon_flag);
    q_index += sprintf(gga_buf + q_index, "%u,", (uint32_t)pz_PositionFix->u_fixFlag);
    q_index += sprintf(gga_buf + q_index, "%u,", (uint32_t)pz_PositionFix->u_svUsed);
    q_index += sprintf(gga_buf + q_index, "%.1f,", pz_PositionFix->f_hdop);
    q_index += sprintf(gga_buf + q_index, "%.3lf,M,%.3lf,M,",
                       pz_PositionFix->d_lla[2] - pz_PositionFix->d_quasiGeoidHeight,
                       pz_PositionFix->d_quasiGeoidHeight);
    /*q_index += */sprintf(gga_buf + q_index, "%s,%s", ageOfDiffStr, staid);
  }

  sm_nmea_add_checksum(gga_buf, sizeof(gga_buf));

  memcpy(gga, gga_buf, strlen(gga_buf));
  return TRUE;
}

/**
 * @brief create rmc string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] rmc string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_rmc(const gnss_PositionFix_t* pz_PositionFix, uint8_t* rmc)
{
  char rmc_buf[256] = { 0 };
  uint32_t q_index = 0;
  EpochTime_t z_Epoch = { 0 };

  tm_cvt_GpstToEpoch(&pz_PositionFix->z_gpsTime, &z_Epoch);

  q_index += sprintf(rmc_buf + q_index, "$GNRMC,");

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&
    (0 == pz_PositionFix->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(rmc_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(rmc_buf + q_index, "%02u%02u%02u.%03u,",
      (uint32_t)z_Epoch.hour, (uint32_t)z_Epoch.min, (uint32_t)z_Epoch.second,
      (uint32_t)gnss_Round(1000 * ((double)z_Epoch.second - (uint32_t)z_Epoch.second),0));
  }

  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    q_index += sprintf(rmc_buf + q_index, "V,,,,,,,");
  }
  else
  {
    q_index += sprintf(rmc_buf + q_index, "A,");
    double lat = pz_PositionFix->d_lla[0] * RAD2DEG;
    double lon = pz_PositionFix->d_lla[1] * RAD2DEG;

    if (lat > 180.0) {
      lat -= 180.0;
    }

    if (lon > 180) {
      lon -= 180.0;
    }
    char lat_flag = (lat >= 0) ? 'N' : 'S';
    char lon_flag = (lon >= 0) ? 'E' : 'W';

    if (lat < 0) {
      lat = -lat;
    }

    if (lon < 0) {
      lon = -lon;
    }

    double d_Speed = sqrt(((double)pz_PositionFix->f_velEnu[0] * pz_PositionFix->f_velEnu[0])
      + ((double)pz_PositionFix->f_velEnu[1] * pz_PositionFix->f_velEnu[1])) / NMEA_VEL_KNOT;
    d_Speed = d_Speed < 999.9 ? d_Speed : 999.9;
    double d_Heading = atan2(pz_PositionFix->f_velEnu[0], pz_PositionFix->f_velEnu[1]);

    q_index += sprintf(rmc_buf + q_index, "%02u%09.6lf,%c,%03u%09.6lf,%c,",
      (uint32_t)lat, (lat - (uint32_t)lat) * 60.0, lat_flag,
      (uint32_t)lon, (lon - (uint32_t)lon) * 60.0, lon_flag);

    q_index += sprintf(rmc_buf + q_index, "%.1lf,%.1lf,", d_Speed, d_Heading);
  }

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&
    (0 == pz_PositionFix->z_gpsTime.t_fullMsec || 0 == pz_PositionFix->z_gpsTime.w_week))
  {
    q_index += sprintf(rmc_buf + q_index, ",,,");
  }
  else
  {
    q_index += sprintf(rmc_buf + q_index, "%02u%02u%02u,,,",
      (uint32_t)z_Epoch.day, (uint32_t)z_Epoch.month, (uint32_t)(z_Epoch.year % 1000));
  }

  char rmc_mode_indicator = 'D';
  switch (pz_PositionFix->u_fixFlag)
  {
    case GNSS_FIX_FLAG_INVALID:	 rmc_mode_indicator = 'V'; break;
    case GNSS_FIX_FLAG_SPS:      rmc_mode_indicator = 'A'; break;
    case GNSS_FIX_FLAG_DGNSS:    rmc_mode_indicator = 'D'; break;
    case GNSS_FIX_FLAG_PPS:      rmc_mode_indicator = 'P'; break;
    case GNSS_FIX_FLAG_FIXED:    rmc_mode_indicator = 'R'; break;
    case GNSS_FIX_FLAG_FLOATING: rmc_mode_indicator = 'F'; break;
    case GNSS_FIX_FLAG_DR:		   rmc_mode_indicator = 'E'; break;
    case GNSS_FIX_FLAG_MANUAL:	 rmc_mode_indicator = 'M'; break;
    case GNSS_FIX_FLAG_SIMULATOR:rmc_mode_indicator = 'S'; break;
    default:break;
  }
  /*q_index += */sprintf(rmc_buf + q_index, "%c,V", rmc_mode_indicator);

  sm_nmea_add_checksum(rmc_buf, sizeof(rmc_buf));

  memcpy(rmc, rmc_buf, strlen(rmc_buf));
  return TRUE;
}

/**
 * @brief create gga string from input position fix
 * @param[in] pz_location_report
 * @param[out] gga string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gga_deprecate(const loc_api_location_report_t* pz_location_report, uint8_t* gga)
{
  char gga_buf[256] = { 0 };
  uint32_t q_index = 0;
  EpochTime_t z_Epoch = { 0 };
  char ageOfDiffStr[256] = { 0 };

  /*z_Epoch.year = pz_location_report->utc_year;
  z_Epoch.month = pz_location_report->utc_month;
  z_Epoch.day = pz_location_report->utc_day;*/
  z_Epoch.hour = pz_location_report->utc_hour;
  z_Epoch.min = pz_location_report->utc_minute;
  z_Epoch.second = (float)pz_location_report->utc_second;

  q_index += sprintf(gga_buf + q_index, "$GNGGA,");

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_location_report->fix_quality) &&
    (0 == pz_location_report->utc_timestamp))
  {
    q_index += sprintf(gga_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(gga_buf + q_index, "%02u%02u%02u.%03u,",
      (uint32_t)z_Epoch.hour, (uint32_t)z_Epoch.min, (uint32_t)z_Epoch.second,
      (uint32_t)gnss_Round(1000 * ((double)z_Epoch.second - (uint32_t)z_Epoch.second), 0));
  }

  if ((GNSS_FIX_FLAG_SPS == pz_location_report->fix_quality) || (GNSS_FIX_FLAG_DR == pz_location_report->fix_quality))
  {
    sprintf(ageOfDiffStr, "");
  }
  else if ((GNSS_FIX_FLAG_DGNSS == pz_location_report->fix_quality) ||
    (GNSS_FIX_FLAG_FIXED == pz_location_report->fix_quality) ||
    (GNSS_FIX_FLAG_FLOATING == pz_location_report->fix_quality))
  {
    sprintf(ageOfDiffStr, "%.1f", (pz_location_report->age < 3600.0 && pz_location_report->age >= 0.0) ? pz_location_report->age : 0.0);
  }

  /* lat, lon, alt item */
  if (GNSS_FIX_FLAG_INVALID == pz_location_report->fix_quality)
  {
    /*q_index += */sprintf(gga_buf + q_index, ",,,,0,,,,,,,,");
  }
  else
  {
    double lat = pz_location_report->lat;
    double lon = pz_location_report->lon;

    if (lat > 180.0) {
      lat -= 180.0;
    }

    if (lon > 180) {
      lon -= 180.0;
    }
    char lat_flag = (lat >= 0) ? 'N' : 'S';
    char lon_flag = (lon >= 0) ? 'E' : 'W';

    if (lat < 0) {
      lat = -lat;
    }

    if (lon < 0) {
      lon = -lon;
    }

    q_index += sprintf(gga_buf + q_index, "%02u%09.6lf,%c,%03u%09.6lf,%c,",
      (uint32_t)lat, (lat - (uint32_t)lat) * 60.0, lat_flag,
      (uint32_t)lon, (lon - (uint32_t)lon) * 60.0, lon_flag);
    q_index += sprintf(gga_buf + q_index, "%u,", (uint32_t)pz_location_report->fix_quality);
    q_index += sprintf(gga_buf + q_index, "%u,", (uint32_t)pz_location_report->sv_used);
    q_index += sprintf(gga_buf + q_index, "%.1f,", pz_location_report->hdop);
    q_index += sprintf(gga_buf + q_index, "%.3lf,M,%.3lf,M,",
      pz_location_report->alt - pz_location_report->quasi_geoid_h,
      pz_location_report->quasi_geoid_h);
    /*q_index += */sprintf(gga_buf + q_index, "%s,", ageOfDiffStr);
  }

  sm_nmea_add_checksum(gga_buf, sizeof(gga_buf));

  memcpy(gga, gga_buf, strlen(gga_buf));
  return TRUE;
}

/**
 * @brief convert freq to signal ID
 * @param[in] u_sysType is system type
 * @param[in] u_freqType is frequence type
 * @return signal ID
 */
static uint8_t sm_nmea_freq2SignalID(gnss_ConstellationType u_sysType, gnss_FreqType u_freqType)
{
  uint8_t u_signalID = 0;

  switch (u_sysType)
  {
  case C_GNSS_GPS:
    switch (u_freqType)
    {
    case C_GNSS_FREQ_TYPE_L1:
      u_signalID = 1;
      break;
    case C_GNSS_FREQ_TYPE_L2:
      u_signalID = 5;
      break;
    case C_GNSS_FREQ_TYPE_L5:
      u_signalID = 8;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_GLO:
    switch (u_freqType)
    {
    case C_GNSS_FREQ_TYPE_L1:
      u_signalID = 1;
      break;
    case C_GNSS_FREQ_TYPE_L2:
      u_signalID = 3;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_BDS3:
    switch (u_freqType)
    {
    case C_GNSS_FREQ_TYPE_L1:
      u_signalID = 1;
      break;
    case C_GNSS_FREQ_TYPE_L2:
      u_signalID = 5;
      break;
    case C_GNSS_FREQ_TYPE_L5:
      u_signalID = 8;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_GAL:
    switch (u_freqType)
    {
    case C_GNSS_FREQ_TYPE_L1:
      u_signalID = 6;
      break;
    case C_GNSS_FREQ_TYPE_L2:
      u_signalID = 1;
      break;
    case C_GNSS_FREQ_TYPE_L5:
      u_signalID = 2;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_QZS:
    switch (u_freqType)
    {
    case C_GNSS_FREQ_TYPE_L1:
      u_signalID = 1;
      break;
    case C_GNSS_FREQ_TYPE_L2:
      u_signalID = 5;
      break;
    case C_GNSS_FREQ_TYPE_L5:
      u_signalID = 8;
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }

  return u_signalID;
}

/**
 * @brief create one gsv string from input position fix
 * @param[in] u_sysType is target system type
 * @param[in] u_freqType is target frequence type
 * @param[in] pz_PositionFix_SatStatus is satellite status
 * @param[in/out] gsv_num is total gsv num
 * @param[in] gsv_max is max gsv num
 * @param[out] gsv string
 * @return TRUE - success
 */
static int32_t sm_nmea_create_one_gsv(gnss_ConstellationType u_sysType, gnss_FreqType u_freqType,
  const gnss_PositionFix_SvStatus_t* pz_PositionFix_SatStatus, uint8_t gsv[][NMEA_GSV_STRING_LEN], uint8_t* pu_gsv_num, char gsv_max)
{
  uint8_t u_i = 0;
  uint8_t u_sysGsvNum = 0;
  uint8_t u_sysSvTotal = 0;
  uint8_t u_sysSvIndex = 0;
  uint8_t u_signalID = 0;
  uint8_t u_gsvLength = 0;
  char gsv_type_str[20] = { '\0' };

  switch (u_sysType)
  {
  case C_GNSS_NONE:
  {
    break;
  }
  case C_GNSS_GPS:
  {
    sprintf(gsv_type_str, "$GPGSV");
    break;
  }
  case C_GNSS_GLO:
  {
    sprintf(gsv_type_str, "$GLGSV");
    break;
  }
  case C_GNSS_QZS:
  {
    sprintf(gsv_type_str, "$GQGSV");
    break;
  }
  case C_GNSS_BDS3:
  {
    sprintf(gsv_type_str, "$GBGSV");
    break;
  }
  case C_GNSS_GAL:
  {
    sprintf(gsv_type_str, "$GAGSV");
    break;
  }
  default:
  {
    break;
  }
  }

  u_signalID = sm_nmea_freq2SignalID(u_sysType, u_freqType);

  if (NULL == pz_PositionFix_SatStatus || NULL == gsv || NULL == pu_gsv_num)
  {
    return 0;
  }
  for (u_i = 0; u_i < pz_PositionFix_SatStatus->u_MeasTrackCount; ++u_i)
  {
    if (C_GNSS_NONE == pz_PositionFix_SatStatus->z_SV[u_i].u_constellation ||
      gnss_cvt_Sig2FreqType(pz_PositionFix_SatStatus->z_SV[u_i].u_signal) != u_freqType)
    {
      continue;
    }
    if (u_sysType == pz_PositionFix_SatStatus->z_SV[u_i].u_constellation)
    {
      ++u_sysSvTotal;
    }
  }
  u_sysGsvNum = (u_sysSvTotal == 0) ? 0 : (u_sysSvTotal - 1) / 4 + 1;
  if (*pu_gsv_num + u_sysGsvNum > gsv_max)
  {
    u_sysGsvNum = gsv_max - *pu_gsv_num;
  }
  for (u_i = 0; u_i < u_sysGsvNum; u_i++)
  {
    memset(gsv[*pu_gsv_num + u_i], 0, NMEA_GSV_STRING_LEN * sizeof(char));
    sprintf((char*)gsv[*pu_gsv_num + u_i], "%s,%u,%u,%u", gsv_type_str, (uint32_t)u_sysGsvNum, (uint32_t)(u_i + 1), (uint32_t)u_sysSvTotal);
  }
  /* fill svid cn0 ele azi info */
  for (u_i = 0; u_i < pz_PositionFix_SatStatus->u_MeasTrackCount; u_i++)
  {
    if (C_GNSS_NONE == pz_PositionFix_SatStatus->z_SV[u_i].u_constellation ||
      u_sysType != pz_PositionFix_SatStatus->z_SV[u_i].u_constellation ||
      gnss_cvt_Sig2FreqType(pz_PositionFix_SatStatus->z_SV[u_i].u_signal) != u_freqType)
    {
      continue;
    }
    if (*pu_gsv_num + (u_sysSvIndex / 4) >= gsv_max)
    {
      break;
    }
    char* str_ptr = (char*)gsv[*pu_gsv_num + (u_sysSvIndex / 4)];
    u_sysSvIndex++;

    char temp[16] = { '\0' };
    if (pz_PositionFix_SatStatus->z_SV[u_i].f_elevation != 0.0f)
    {
      sprintf(temp, ",%u,%d,%d,%d", (uint32_t)pz_PositionFix_SatStatus->z_SV[u_i].u_svid, (int)pz_PositionFix_SatStatus->z_SV[u_i].f_elevation,
        (int)pz_PositionFix_SatStatus->z_SV[u_i].f_azimuth, (int)pz_PositionFix_SatStatus->z_SV[u_i].f_cn0);
    }
    else
    {
      sprintf(temp, ",%u,,,%d", (uint32_t)pz_PositionFix_SatStatus->z_SV[u_i].u_svid, (int)pz_PositionFix_SatStatus->z_SV[u_i].f_cn0);
    }
    strcat(str_ptr, temp);
  }
  for (u_i = 0; u_i < u_sysGsvNum; u_i++)
  {
    u_gsvLength = (uint8_t)strlen((const char*)gsv[*pu_gsv_num + u_i]);
    sprintf((char *)gsv[*pu_gsv_num + u_i] + u_gsvLength, ",%u", (uint32_t)u_signalID);
    sm_nmea_add_checksum((char*)gsv[*pu_gsv_num + u_i], sizeof(gsv[*pu_gsv_num + u_i]));
  }
  (*pu_gsv_num) += u_sysGsvNum;

  return 1;
}

/**
 * @brief create gsv string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gsv string
 * @return TRUE - success
 *         FALSE - fail
 */
uint8_t sm_nmea_create_gsv(const gnss_PositionFix_t* pz_PositionFix, uint8_t gsv[][NMEA_GSV_STRING_LEN], uint8_t u_gsv_max)
{
  uint8_t   u_gsvNum = 0;
  uint8_t   u_isys;
  uint8_t   u_signalIndex;

  for (u_isys = C_GNSS_GPS; u_isys < C_GNSS_MAX; u_isys++)
  {
    if (C_GNSS_GLO == u_isys)
    {
      sm_nmea_create_one_gsv(u_isys, C_GNSS_FREQ_TYPE_L1, &pz_PositionFix->z_SvStatus, gsv, &u_gsvNum, u_gsv_max);
    }
    else
    {
      // loop freq
      for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; u_signalIndex++)
      {
        sm_nmea_create_one_gsv(u_isys, u_signalIndex, &pz_PositionFix->z_SvStatus, gsv, &u_gsvNum, u_gsv_max);
      }
    }
  }

  return u_gsvNum;
}

/**
 * @brief create gsv string from input position fix
 * @param[in] pz_location_report
 * @param[out] gsv string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_gsv_deprecate(const loc_api_location_report_t* pz_location_report, uint8_t gsv[][NMEA_GSV_STRING_LEN], uint8_t* pu_gsv_num, char gsv_max)
{
  //sm_nmea_create_one_gsv(C_GNSS_GPS, &pz_location_report->z_MeasStatus, gsv, pu_gsv_num, gsv_max);
  //sm_nmea_create_one_gsv(C_GNSS_GLO, &pz_location_report->z_MeasStatus, gsv, pu_gsv_num, gsv_max);
  //sm_nmea_create_one_gsv(C_GNSS_BDS3, &pz_location_report->z_MeasStatus, gsv, pu_gsv_num, gsv_max);
  //sm_nmea_create_one_gsv(C_GNSS_GAL, &pz_location_report->z_MeasStatus, gsv, pu_gsv_num, gsv_max);
  //sm_nmea_create_one_gsv(C_GNSS_QZS, &pz_location_report->z_MeasStatus, gsv, pu_gsv_num, gsv_max);

  return TRUE;
}

/**
 * @brief create gsa string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] gsa string
 * @return TRUE - success
 *         FALSE - fail
 */
int32_t sm_nmea_create_gsa(const gnss_PositionFix_t* pz_PositionFix, uint8_t gsa[][NMEA_GSA_STRING_LEN], uint8_t u_gsa_max)
{
  uint8_t u_gps_sv_ch_num = 0;
  uint8_t u_gln_sv_ch_num = 0;
  uint8_t u_bds_sv_ch_num = 0;
  uint8_t u_bds_sv_ch_num_row_2 = 0;
  uint8_t u_gal_sv_ch_num = 0;
  uint8_t u_qzs_sv_ch_num = 0;
  uint8_t u_gsa_num = 0;
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  //uint8_t u_prn = 0;
  //uint8_t u_constellation = 0;
  gnss_PositionFix_SvStatus_t z_SvStatus;
  char dop_str[64] = { 0 };
  char gsa_fix_mode = pz_PositionFix->u_fixSource == FIX_SOURCE_INVALID ? 1 : 3;

  char gps_gsa[NMEA_GSA_STRING_LEN] = { 0 };
  char gln_gsa[NMEA_GSA_STRING_LEN] = { 0 };
  char bds_gsa[NMEA_GSA_STRING_LEN] = { 0 };
  char bds_gsa_row_2[NMEA_GSA_STRING_LEN] = { 0 };
  char gal_gsa[NMEA_GSA_STRING_LEN] = { 0 };
  char qzs_gsa[NMEA_GSA_STRING_LEN] = { 0 };

  sprintf(gps_gsa, "$GNGSA,A,%d,", gsa_fix_mode);
  sprintf(gln_gsa, "$GNGSA,A,%d,", gsa_fix_mode);
  sprintf(bds_gsa, "$GNGSA,A,%d,", gsa_fix_mode);
  sprintf(bds_gsa_row_2, "$GNGSA,A,%d,", gsa_fix_mode);
  sprintf(gal_gsa, "$GNGSA,A,%d,", gsa_fix_mode);
  sprintf(qzs_gsa, "$GNGSA,A,%d,", gsa_fix_mode);

  z_SvStatus = pz_PositionFix->z_SvStatus;
  for (u_i = C_GNSS_GPS; u_i < C_GNSS_MAX; ++u_i)
  {
    for (u_j = 0; u_j < 64; u_j++)
    {
      if ((z_SvStatus.t_SvInUseMask[u_i] & ((uint64_t)1 << u_j)) == 0)
      {
        continue;
      }

      if (u_i == C_GNSS_GPS)
      {
        char temp[16];
        sprintf(temp, "%02u,", (uint32_t)(u_j + 1));
        if (u_gps_sv_ch_num < 12)
        {
          strcat(gps_gsa, temp);
          u_gps_sv_ch_num++;
        }
      }
      else if (u_i == C_GNSS_GLO)
      {
        char temp[16];
        sprintf(temp, "%02u,", (uint32_t)(u_j + 1));
        if (u_gln_sv_ch_num < 12)
        {
          strcat(gln_gsa, temp);
          u_gln_sv_ch_num++;
        }
      }
      else if (u_i == C_GNSS_BDS3)
      {
        char temp[16];
        sprintf(temp, "%02u,", (uint32_t)(u_j + 1));
        if (u_bds_sv_ch_num < 12)
        {
          strcat(bds_gsa, temp);
          u_bds_sv_ch_num++;
        }
        else if (u_bds_sv_ch_num_row_2 < 12)
        {
          strcat(bds_gsa_row_2, temp);
          u_bds_sv_ch_num_row_2++;
        }
      }
      else if (u_i == C_GNSS_GAL)
      {
        char temp[16];
        sprintf(temp, "%02u,", (uint32_t)(u_j + 1));
        if (u_gal_sv_ch_num < 12)
        {
          strcat(gal_gsa, temp);
          u_gal_sv_ch_num++;
        }
      }
      else if (u_i == C_GNSS_QZS)
      {
        char temp[16];
        sprintf(temp, "%03u,", (uint32_t)(u_j + 193));
        if (u_qzs_sv_ch_num < 12)
        {
          strcat(qzs_gsa, temp);
          u_qzs_sv_ch_num++;
        }
      }
    }
  }

  for (u_i = u_gps_sv_ch_num; u_i < 12; u_i++)
  {
    strcat(gps_gsa, ",");
  }

  for (u_i = u_gln_sv_ch_num; u_i < 12; u_i++)
  {
    strcat(gln_gsa, ",");
  }

  for (u_i = u_bds_sv_ch_num; u_i < 12; u_i++)
  {
    strcat(bds_gsa, ",");
  }

  for (u_i = u_bds_sv_ch_num_row_2; u_i < 12; u_i++)
  {
    strcat(bds_gsa_row_2, ",");
  }

  for (u_i = u_gal_sv_ch_num; u_i < 12; u_i++)
  {
    strcat(gal_gsa, ",");
  }

  for (u_i = u_qzs_sv_ch_num; u_i < 12; u_i++)
  {
    strcat(qzs_gsa, ",");
  }

  sprintf(dop_str, "%2.1f,%2.1f,%2.1f", pz_PositionFix->z_dops.f_pdop, pz_PositionFix->z_dops.f_hdop, pz_PositionFix->z_dops.f_vdop);
  strcat(gps_gsa, dop_str);
  strcat(gln_gsa, dop_str);
  strcat(bds_gsa, dop_str);
  strcat(bds_gsa_row_2, dop_str);
  strcat(gal_gsa, dop_str);
  strcat(qzs_gsa, dop_str);

  // NMEA 0183 V4.11, system ID
  strcat(gps_gsa, ",1");
  strcat(gln_gsa, ",2");
  strcat(bds_gsa, ",4");
  strcat(bds_gsa_row_2, ",4");
  strcat(gal_gsa, ",3");
  strcat(qzs_gsa, ",5");

  sm_nmea_add_checksum(gps_gsa, NMEA_GSA_STRING_LEN);
  sm_nmea_add_checksum(gln_gsa, NMEA_GSA_STRING_LEN);
  sm_nmea_add_checksum(bds_gsa, NMEA_GSA_STRING_LEN);
  sm_nmea_add_checksum(bds_gsa_row_2, NMEA_GSA_STRING_LEN);
  sm_nmea_add_checksum(gal_gsa, NMEA_GSA_STRING_LEN);
  sm_nmea_add_checksum(qzs_gsa, NMEA_GSA_STRING_LEN);

  if (u_gps_sv_ch_num != 0 && u_gsa_num < u_gsa_max)
  {
    memcpy(gsa[u_gsa_num], gps_gsa, NMEA_GSA_STRING_LEN * sizeof(char));
    u_gsa_num++;
  }

  if (u_gln_sv_ch_num != 0 && u_gsa_num < u_gsa_max)
  {
    memcpy(gsa[u_gsa_num], gln_gsa, NMEA_GSA_STRING_LEN * sizeof(char));
    u_gsa_num++;
  }

  if (u_bds_sv_ch_num != 0 && u_gsa_num < u_gsa_max)
  {
    memcpy(gsa[u_gsa_num], bds_gsa, NMEA_GSA_STRING_LEN * sizeof(char));
    u_gsa_num++;
  }

  if (u_bds_sv_ch_num_row_2 != 0 && u_gsa_num < u_gsa_max)
  {
    memcpy(gsa[u_gsa_num], bds_gsa_row_2, NMEA_GSA_STRING_LEN * sizeof(char));
    u_gsa_num++;
  }

  if (u_gal_sv_ch_num != 0 && u_gsa_num < u_gsa_max)
  {
    memcpy(gsa[u_gsa_num], gal_gsa, NMEA_GSA_STRING_LEN * sizeof(char));
    u_gsa_num++;
  }

  if (u_qzs_sv_ch_num != 0 && u_gsa_num < u_gsa_max)
  {
    memcpy(gsa[u_gsa_num], qzs_gsa, NMEA_GSA_STRING_LEN * sizeof(char));
    u_gsa_num++;
  }

  return u_gsa_num;
}

static float sm_nmea_velGetTrackAngle(const float f_velEnu[3])
{
  double d_bearing = 0.0;
  double d_ve2 = 0.0;
  double d_vn2 = 0.0;
  /*double d_vh4 = 0.0;*/
  d_ve2 = (double)(f_velEnu[0]) * (f_velEnu[0]);
  d_vn2 = (double)(f_velEnu[1]) * (f_velEnu[1]);
  /*d_vh4 = (d_ve2 + d_vn2) * (d_ve2 + d_vn2);*/
  if (d_ve2 > 1e-10 || d_vn2 > 1e-10)
  {
    d_bearing = atan2(f_velEnu[0], f_velEnu[1]) * RAD2DEG;
  }
  return (float)d_bearing;
}
/**
 * @brief calculate the track angle in strcut pz_PositionFix
 * @param[out] z_api_location_report location info for report
 */
static float sm_nmea_getTrackAngle(const gnss_PositionFix_t* pz_PositionFix)
{
  float d_bearing = 0.0;
  if (NULL == pz_PositionFix)
  {
    return d_bearing;
  }
  d_bearing = sm_nmea_velGetTrackAngle(pz_PositionFix->f_velEnu);
  if (d_bearing < 0.0)
  {
    d_bearing += 360.0;
  }
  return d_bearing;
}

/**
 * @brief create vtg string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] vtg string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_vtg(const gnss_PositionFix_t* pz_PositionFix, uint8_t* vtg)
{
  char vtg_buf[256] = { 0 };
  uint32_t q_index = 0;
  float trackangle = 0.0;

  q_index += sprintf(vtg_buf + q_index, "$GNVTG,");

  trackangle = sm_nmea_getTrackAngle(pz_PositionFix);

  q_index += sprintf(vtg_buf + q_index, "%.1f,T,0.0,M,",trackangle);

  double d_Speed = sqrt(((double)pz_PositionFix->f_velEnu[0] * pz_PositionFix->f_velEnu[0])
	+ ((double)pz_PositionFix->f_velEnu[1] * pz_PositionFix->f_velEnu[1])) / NMEA_VEL_KNOT;
  d_Speed = d_Speed < 999.9 ? d_Speed : 999.9;

  q_index += sprintf(vtg_buf + q_index, "%.1lf,N,",d_Speed);

  double d_Speed_km = d_Speed*NMEA_KNOT2KMH;

  q_index += sprintf(vtg_buf + q_index, "%.1lf,K,",d_Speed_km);


  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    /*q_index += */sprintf(vtg_buf + q_index, "N,");
  }
  else
  {
    /*q_index += */sprintf(vtg_buf + q_index, "A,");

  }

  sm_nmea_add_checksum(vtg_buf, sizeof(vtg_buf));

  memcpy(vtg, vtg_buf, strlen(vtg_buf));
  return TRUE;
}

/**
 * @brief create zda string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] zda string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_zda(const gnss_PositionFix_t* pz_PositionFix, uint8_t* zda)
{
  char zda_buf[256] = { 0 };
  uint32_t q_index = 0;
  EpochTime_t z_Epoch = { 0 };

  tm_cvt_GpstToEpoch(&pz_PositionFix->z_gpsTime, &z_Epoch);

  q_index += sprintf(zda_buf + q_index, "$GNZDA,");

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&
    (0 == pz_PositionFix->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(zda_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(zda_buf + q_index, "%02u%02u%02u.%03u,",
      (uint32_t)z_Epoch.hour, (uint32_t)z_Epoch.min, (uint32_t)z_Epoch.second,
      (uint32_t)gnss_Round(1000 * ((double)z_Epoch.second - (uint32_t)z_Epoch.second),0));
  }

  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    /*q_index += */sprintf(zda_buf + q_index, ",,,,00,00,");
  }
  else
  {
    /*q_index += */sprintf(zda_buf + q_index, "%02u,%02u,%04u,08,00,",
      (uint32_t)z_Epoch.day, (uint32_t)z_Epoch.month, (uint32_t)z_Epoch.year);
  }

  sm_nmea_add_checksum(zda_buf, sizeof(zda_buf));

  memcpy(zda, zda_buf, strlen(zda_buf));
  return TRUE;
}


static uint8_t sm_nmea_getDistance(const double* pd_refPos, const double* pd_roverPos, double* d_length, double pd_enu[3])
{
  uint8_t u_i = 0;
  double pd_vector_M2A[3] = { .0 };
  double pd_refLla[3] = { .0 };
  double E[9] = { .0 };


  if ((gnss_Dot(pd_refPos, pd_refPos, 3) < FABS_ZEROS) ||
    (gnss_Dot(pd_roverPos, pd_roverPos, 3) < FABS_ZEROS))
  {
    return 1;
  }


  gnss_Ecef2Lla(pd_refPos, pd_refLla);
  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_vector_M2A[u_i] = pd_roverPos[u_i] - pd_refPos[u_i];
  }
  gnss_Pos2EnuMat(pd_refLla, E);
  gnss_MatrixMultiply("NN", 3, 1, 3, 1.0, E, pd_vector_M2A, 0.0, pd_enu);

  *d_length = sqrt(gnss_Dot(pd_enu, pd_enu, 3));

  return 0;
}

/**
 * @brief create ntr string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] ntr string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_create_ntr(const gnss_PositionFix_t* pz_PositionFix, uint8_t* ntr)

{
  char ntr_buf[256] = { 0 };
  uint32_t q_index = 0;
  double z_pd_enu[3] = { 0.0 };
  double z_d_length = 0.0;
  EpochTime_t z_Epoch = { 0 };

  tm_cvt_GpstToEpoch(&pz_PositionFix->z_gpsTime, &z_Epoch);

  q_index += sprintf(ntr_buf + q_index, "$GNNTR,");

  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&
    (0 == pz_PositionFix->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(ntr_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(ntr_buf + q_index, "%02u%02u%02u.%03u,",
      (uint32_t)z_Epoch.hour, (uint32_t)z_Epoch.min, (uint32_t)z_Epoch.second,
      (uint32_t)gnss_Round(1000 * ((double)z_Epoch.second - (uint32_t)z_Epoch.second),0));
  }

  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    /*q_index += */sprintf(ntr_buf + q_index, ",,,,,,,");
  }
  else
  {
    sm_nmea_getDistance(pz_PositionFix->z_rtk.pd_StationCoordinate,pz_PositionFix->d_xyz,&z_d_length,z_pd_enu);
    /*q_index += */sprintf(ntr_buf + q_index, "%04.3lf,%04.3lf,%04.3lf,%04.3lf,%04u",
      z_d_length, z_pd_enu[1],z_pd_enu[0],z_pd_enu[2],pz_PositionFix->z_rtk.q_StationID);
  }

  sm_nmea_add_checksum(ntr_buf, sizeof(ntr_buf));

  memcpy(ntr, ntr_buf, strlen(ntr_buf));
  return TRUE;
}

/**
 * @brief create zone string from input position fix
 * @param[in] pz_PositionFix
 * @param[out] zone string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_zone(const gnss_PositionFix_t* pz_PositionFix, uint8_t* zone)
{
  char zone_buf[256] = { 0 };
  uint32_t q_index = 0;
  int32_t q_fixFlag = 0;
  float f_trackValue = 0.0;
  double d_lat = 0.0;
  double d_lon = 0.0;
  q_index += sprintf(zone_buf + q_index, "$GNZONE,");
  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag) &&(0 == pz_PositionFix->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(zone_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(zone_buf + q_index, "%.3lf,", pz_PositionFix->z_gpsTime.q_towMsec / 1000.0);
  }
  if (GNSS_FIX_FLAG_INVALID == (pz_PositionFix->u_fixFlag))
  {
    q_fixFlag = 1;
  }
  else if (GNSS_FIX_FLAG_SPS == (pz_PositionFix->u_fixFlag))
  {
    q_fixFlag = 2;
  }
  else if (GNSS_FIX_FLAG_DGNSS == (pz_PositionFix->u_fixFlag))
  {
    q_fixFlag = 3;
  }
  else if (GNSS_FIX_FLAG_FLOATING == (pz_PositionFix->u_fixFlag))
  {
    q_fixFlag = 4;
  }
  else if (GNSS_FIX_FLAG_FIXED == (pz_PositionFix->u_fixFlag))
  {
    q_fixFlag = 5;
  }
  /* lat, lon, alt item */
  if (GNSS_FIX_FLAG_INVALID == pz_PositionFix->u_fixFlag)
  {
    /*q_index += */sprintf(zone_buf + q_index, "%d,,,,,,,,,,,", q_fixFlag);
  }
  else
  {
    q_index += sprintf(zone_buf + q_index, "%d,", q_fixFlag);
    d_lat = pz_PositionFix->d_lla[0] * RAD2DEG;
    d_lon = pz_PositionFix->d_lla[1] * RAD2DEG;
    q_index += sprintf(zone_buf + q_index, "%.8lf,%.8lf,%.3lf,%.3lf,", d_lon, d_lat, pz_PositionFix->d_lla[2], pz_PositionFix->d_lla[2] - pz_PositionFix->d_quasiGeoidHeight);
    q_index += sprintf(zone_buf + q_index, "%.3f,%.3f,%.3f,", pz_PositionFix->f_velEnu[1], pz_PositionFix->f_velEnu[0], -(pz_PositionFix->f_velEnu[2]));
    f_trackValue = sm_nmea_velGetTrackAngle(pz_PositionFix->f_velEnu);
    q_index += sprintf(zone_buf + q_index, "%.3f,", f_trackValue);
    q_index += sprintf(zone_buf + q_index, "%02u,%02u,", (uint32_t)pz_PositionFix->z_SvStatus.u_SvTrackCount, (uint32_t)pz_PositionFix->z_SvStatus.u_SvInUseCount);
    /*q_index += */sprintf(zone_buf + q_index, "%.2f", pz_PositionFix->d_avgCN0);
  }
  sm_nmea_add_checksum(zone_buf, sizeof(zone_buf));
  memcpy(zone, zone_buf, strlen(zone_buf));
  return TRUE;
}
/**
 * @brief create hdt string from input orient fix
 * @param[in] pz_ortResult
 * @param[out] hdt string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_hdt(const gnss_OrientFix_t* pz_ortResult, uint8_t* ps_hdt)
{
  uint32_t q_index = 0;
  double d_heading = (double)(pz_ortResult->z_ortResult.f_heading);
  char ps_hdtBuf[256] = { 0 };
  q_index += sprintf(ps_hdtBuf + q_index, "$GNHDT,");
  /*q_index += */sprintf(ps_hdtBuf + q_index, "%.4lf,T", d_heading);
  sm_nmea_add_checksum(ps_hdtBuf, sizeof(ps_hdtBuf));
  memcpy(ps_hdt, ps_hdtBuf, strlen(ps_hdtBuf));
  return TRUE;
}

/**
 * @brief create tra string from input orient fix
 * @param[in] pz_ortResult
 * @param[out] tra string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_tra(const gnss_OrientFix_t* pz_ortResult, uint8_t* ps_tra)
{
  char tra_buf[256] = { 0 };
  char ageOfDiffStr[256] = { 0 };
  char staid[16] = { 0 };
  uint32_t q_index = 0;
  q_index += sprintf(tra_buf + q_index, "$GNTRA,");
  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_ortResult->u_ortFixFlag) &&
    (0 == pz_ortResult->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(tra_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(tra_buf + q_index, "%02u%02u%02u.%02u,",
      (uint32_t)pz_ortResult->z_epoch.hour, (uint32_t)pz_ortResult->z_epoch.min, (uint32_t)pz_ortResult->z_epoch.second,
      (uint32_t)gnss_Round(100 * ((double)pz_ortResult->z_epoch.second - (uint32_t)pz_ortResult->z_epoch.second), 0));
  }

  sprintf(staid, "");
  if ((GNSS_FIX_FLAG_SPS == pz_ortResult->u_ortFixFlag) || (GNSS_FIX_FLAG_DR == pz_ortResult->u_ortFixFlag))
  {
    sprintf(ageOfDiffStr, "");
  }
  else if ((GNSS_FIX_FLAG_DGNSS == pz_ortResult->u_ortFixFlag) ||
    (GNSS_FIX_FLAG_FIXED == pz_ortResult->u_ortFixFlag) ||
    (GNSS_FIX_FLAG_FLOATING == pz_ortResult->u_ortFixFlag))
  {
    sprintf(ageOfDiffStr, "%.1f", ((pz_ortResult->f_age) < 3600.0 && (pz_ortResult->f_age) >= 0.0) ? (pz_ortResult->f_age) : 0.0);
  }

  if (GNSS_FIX_FLAG_INVALID == pz_ortResult->u_ortFixFlag)
  {
    /*q_index += */sprintf(tra_buf + q_index, ",,,0,,,");
  }
  else
  {
    q_index += sprintf(tra_buf + q_index, "%.2f,%.2f,%.2f,", pz_ortResult->z_ortResult.f_heading, pz_ortResult->z_ortResult.f_pitch, pz_ortResult->z_ortResult.f_roll);
    q_index += sprintf(tra_buf + q_index, "%u,", (uint32_t)pz_ortResult->u_ortFixFlag);
    q_index += sprintf(tra_buf + q_index, "%u,", (uint32_t)pz_ortResult->u_SvInUseCount);
    /*q_index += */sprintf(tra_buf + q_index, "%s,%s", ageOfDiffStr, staid);
  }
  sm_nmea_add_checksum(tra_buf, sizeof(tra_buf));
  memcpy(ps_tra, tra_buf, strlen(tra_buf));
  return TRUE;
}

/**
 * @brief create ksxt string from input orient fix
 * @param[in] pz_ortResult
 * @param[out] ksxt string
 * @return TRUE - success
 *         FALSE - fail
 */
BOOL sm_nmea_creat_ksxt(const gnss_OrientFix_t* pz_ortResult, uint8_t* ps_ksxt)
{
  char ksxt_buf[256] = { 0 };
  float f_trackValue = 0.0;
  float f_horiVel = 0.0;//unit in km/h
  uint32_t q_index = 0;
  uint8_t u_posFlag = 0;
  uint8_t u_ortFlag = 0;
  uint8_t u_deltaValid = 0;
  uint8_t u_i = 0;
  double pd_deltaX[3] = { 0.0 };
  double pd_deltaENU[3] = { 0.0 };
  double pd_refLLA[3] = { 0.0 };
  q_index += sprintf(ksxt_buf + q_index, "$KSXT,");
  /* Time item */
  if ((GNSS_FIX_FLAG_INVALID == pz_ortResult->u_ortFixFlag) && (GNSS_FIX_FLAG_INVALID == pz_ortResult->z_mainAntInfo.u_posFixFlag) && (0 == pz_ortResult->z_gpsTime.t_fullMsec))
  {
    q_index += sprintf(ksxt_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%04u%02u%02u%02u%02u%02u.%02u,", (uint32_t)pz_ortResult->z_epoch.year, (uint32_t)pz_ortResult->z_epoch.month, (uint32_t)pz_ortResult->z_epoch.day,
      (uint32_t)pz_ortResult->z_epoch.hour, (uint32_t)pz_ortResult->z_epoch.min, (uint32_t)pz_ortResult->z_epoch.second,
      (uint32_t)gnss_Round(100 * ((double)pz_ortResult->z_epoch.second - (uint32_t)pz_ortResult->z_epoch.second), 0));
  }

  /* lat, lon, alt item */
  if (GNSS_FIX_FLAG_INVALID == pz_ortResult->z_mainAntInfo.u_posFixFlag)
  {
    q_index += sprintf(ksxt_buf + q_index, ",,,");
  }
  else
  {
    double lat = pz_ortResult->z_mainAntInfo.d_lla[0] * RAD2DEG;
    double lon = pz_ortResult->z_mainAntInfo.d_lla[1] * RAD2DEG;
    f_trackValue = sm_nmea_velGetTrackAngle(pz_ortResult->z_mainAntInfo.f_velEnu);
    if (f_trackValue < 0.0)
    {
      f_trackValue += 360.0;
    }
    f_horiVel = sqrtf(pz_ortResult->z_mainAntInfo.f_velEnu[0] * pz_ortResult->z_mainAntInfo.f_velEnu[0] + pz_ortResult->z_mainAntInfo.f_velEnu[1] * pz_ortResult->z_mainAntInfo.f_velEnu[1]) * 3.6f;
    q_index += sprintf(ksxt_buf + q_index, "%.8lf,%.8lf,%.4lf,", lon, lat, pz_ortResult->z_mainAntInfo.d_lla[2]);
    if (fabs(pz_ortResult->z_mainAntInfo.pd_refStationCoordinate[0]) > 1.0e-3 || fabs(pz_ortResult->z_mainAntInfo.pd_refStationCoordinate[1]) > 1.0e-3
      || fabs(pz_ortResult->z_mainAntInfo.pd_refStationCoordinate[2]) > 1.0e-3)
    {
      for (u_i = 0; u_i < 3; ++u_i)
      {
        pd_deltaX[u_i] = pz_ortResult->z_mainAntInfo.d_xyz[u_i] - pz_ortResult->z_mainAntInfo.pd_refStationCoordinate[u_i];
      }
      gnss_Ecef2Lla(pz_ortResult->z_mainAntInfo.pd_refStationCoordinate, pd_refLLA);
      gnss_Ecef2Enu(pd_refLLA, pd_deltaX, pd_deltaENU);
      u_deltaValid = 1;
    }
  }
  if ((GNSS_FIX_FLAG_INVALID == pz_ortResult->u_ortFixFlag))
  {
    q_index += sprintf(ksxt_buf + q_index, ",,");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%.2f,%.2f,", pz_ortResult->z_ortResult.f_heading, pz_ortResult->z_ortResult.f_pitch);
  }
  if (GNSS_FIX_FLAG_INVALID == pz_ortResult->z_mainAntInfo.u_posFixFlag)
  {
    q_index += sprintf(ksxt_buf + q_index, ",,,");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%.2f,%.2f,,", f_trackValue, f_horiVel);
  }

  if (GNSS_FIX_FLAG_FIXED == pz_ortResult->z_mainAntInfo.u_posFixFlag)
  {
    u_posFlag = 3;
  }
  else if (GNSS_FIX_FLAG_DGNSS == pz_ortResult->z_mainAntInfo.u_posFixFlag || GNSS_FIX_FLAG_FLOATING == pz_ortResult->z_mainAntInfo.u_posFixFlag)
  {
    u_posFlag = 2;
  }
  else if (GNSS_FIX_FLAG_SPS == pz_ortResult->z_mainAntInfo.u_posFixFlag)
  {
    u_posFlag = 1;
  }

  if (0 == u_posFlag)
  {
    q_index += sprintf(ksxt_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%u,", (uint32_t)u_posFlag);
  }

  if (GNSS_FIX_FLAG_FIXED == pz_ortResult->u_ortFixFlag)
  {
    u_ortFlag = 3;
  }
  else if (GNSS_FIX_FLAG_DGNSS == pz_ortResult->u_ortFixFlag || GNSS_FIX_FLAG_FLOATING == pz_ortResult->u_ortFixFlag)
  {
    u_ortFlag = 2;
  }
  else if (GNSS_FIX_FLAG_SPS == pz_ortResult->u_ortFixFlag)
  {
    u_ortFlag = 1;
  }

  if (0 == u_ortFlag)
  {
    q_index += sprintf(ksxt_buf + q_index, ",");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%u,", (uint32_t)u_ortFlag);
  }

  q_index += sprintf(ksxt_buf + q_index, "%u,", (uint32_t)pz_ortResult->u_SvInUseCount);
  q_index += sprintf(ksxt_buf + q_index, "%u,", (uint32_t)pz_ortResult->z_mainAntInfo.u_SvInUseCount);
  if (0 == u_deltaValid)
  {
    q_index += sprintf(ksxt_buf + q_index, ",,,");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%.3lf,%.3lf,%.3lf,", pd_deltaENU[0], pd_deltaENU[1], pd_deltaENU[2]);
  }
  if (GNSS_FIX_FLAG_INVALID == pz_ortResult->z_mainAntInfo.u_posFixFlag)
  {
    q_index += sprintf(ksxt_buf + q_index, ",,,");
  }
  else
  {
    q_index += sprintf(ksxt_buf + q_index, "%.3f,%.3f,%.3f,", (float)(pz_ortResult->z_mainAntInfo.f_velEnu[0] * 3.6),
      (float)(pz_ortResult->z_mainAntInfo.f_velEnu[1] * 3.6), (float)(pz_ortResult->z_mainAntInfo.f_velEnu[2] * 3.6));
  }
  /*q_index += */sprintf(ksxt_buf + q_index, ",");
  sm_nmea_add_checksum(ksxt_buf, sizeof(ksxt_buf));
  memcpy(ps_ksxt, ksxt_buf, strlen(ksxt_buf));
  return TRUE;
}
