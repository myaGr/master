/**@file        gnss_engine_api.c
 * @brief       gnss position engine api source file
 * @author      caizhijie
 * @date        2022/04/18
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/18  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include <mw_log.h>
#include "rtk_type.h"
#include "gnss_engine_api.h"
#include "gnss_pe.h"
#include "gnss_sd_nm.h"
#include "gnss_rtcm.h"
#include "gnss_nmea.h"
#include "gnss_sys_api.h"
#include "gnss_rtk.h"
#include "rtklib.h"
#include "asg_rtcm_decode.h"
#include "gnss_hsm_lite_api.h"
#include "gnss_api_def.h"
#include "gnss_tdcp.h"
#include "gnss_sd.h"
#include "gnss_type.h"
#include "gnss_common.h"
#include "mw_alloc.h"
#include "cmn_utils.h"
#include "loc_core_report.h"
#include "integ_type.h"
#include "gnss_cfd.h"
#include "gnss_mode.h"

extern PE_MODES  peMode;

#undef MODULE_TAG
#define MODULE_TAG OBJ_PE

extern meas_blk_t* gpz_Meas;
extern USER_PVT*  gpz_user_pvt;

static GnssEngineCallback_t gz_GnssEngineCb;
static gnss_PositionFix_t  gz_GnssEnginePositionFix;

static BOOL gq_isMultiFreqModel = FALSE;

static void gnss_hsm_lite_nmea_create(USER_PVT* user_pvt, gnss_NmeaInfo_t* nmeainfo)
{
  GNSS_TIME* pTime = gnss_tm_get_time();
  gnss_nmea_result_t user_nmea = { 0 };

  user_nmea.size = sizeof(gnss_nmea_result_t);
  user_nmea.flags = 0;
  user_nmea.latitude = user_pvt->lla.pos[0];
  user_nmea.longitude = user_pvt->lla.pos[1];
  user_nmea.altitude = user_pvt->lla.pos[2];
  user_nmea.speed = sqrt(((double)user_pvt->lla.vel[0] * user_pvt->lla.vel[0])
    + ((double)user_pvt->lla.vel[1] * user_pvt->lla.vel[1])) / NMEA_VEL_KNOT;
  user_nmea.speed = user_nmea.speed < 999.9 ? user_nmea.speed : 999.9;
  user_nmea.bearing = user_pvt->heading;
  user_nmea.accuracy = user_pvt->accuracy;
  user_nmea.time_flag = 1;
  user_nmea.timestamp = user_pvt->timeStampOfUtc;
  user_nmea.year = pTime->utcTime.Year;
  user_nmea.month = pTime->utcTime.Month;
  user_nmea.day = pTime->utcTime.Day;
  user_nmea.hour = pTime->utcTime.Hour;
  user_nmea.minute = pTime->utcTime.Minute;
  user_nmea.second = pTime->utcTime.Second;
  user_nmea.PDOP = user_pvt->DOP.pDOP;
  user_nmea.HDOP = user_pvt->DOP.hDOP;
  user_nmea.VDOP = user_pvt->DOP.vDOP;
  user_nmea.TDOP = user_pvt->DOP.tDOP;

  /* check HDOP value if in abnormal condition */
  if (user_nmea.HDOP > 99.99)
  {
    user_nmea.HDOP = 99.99f;
  }
  else if (user_nmea.HDOP < 0.0)
  {
    user_nmea.HDOP = 0.0f;
  }

  switch (user_pvt->diff_status)
  {
  case DIFF_SOL_STATUS_STANDALONE:
  {
    user_nmea.indicator = GNSS_NMEA_QUALITY_GNSS_SPS;
    break;
  }
  case DIFF_SOL_STATUS_RTD:
  {
    user_nmea.indicator = GNSS_NMEA_QUALITY_DGNSS_SPS;
    break;
  }
  case DIFF_SOL_STATUS_RTK_FLOAT:
  {
    user_nmea.indicator = GNSS_NMEA_QUALITY_RTK_FLOATING;
    break;
  }
  case DIFF_SOL_STATUS_RTK_FIX:
  {
    user_nmea.indicator = GNSS_NMEA_QUALITY_RTK_FIXED;
    break;
  }
  case DIFF_SOL_STATUS_RTK_PROPAGATE:
  {
    user_nmea.indicator = GNSS_NMEA_QUALITY_DR;
    break;
  }
  default:
  {
    user_nmea.indicator = GNSS_NMEA_QUALITY_INVALID;
    break;
  }
  }

  user_nmea.sv_inuesd = user_pvt->usedSvNum;
  user_nmea.ageOfDiff = user_pvt->diff_age;
  user_nmea.ve = user_pvt->lla.vel[0];
  user_nmea.vn = user_pvt->lla.vel[1];
  user_nmea.vu = user_pvt->lla.vel[2];

#if defined(GNSS_NMEA_ENABLE_GSV) || defined(GNSS_NMEA_ENABLE_GSA)

  meas_blk_t* meas_blk = gpz_Meas;

  for (uint32_t i = 0; i < meas_blk->measCnt; i++)
  {
    gnss_meas_t* sv_measurement = &(meas_blk->meas[i]);
    if (i >= GNSS_NMEA_MAX_MEASUREMENT)
    {
      break;
    }

    user_nmea.sv_list[i].size = sizeof(gnss_nmea_sv_info);
    switch (sv_measurement->gnssMode)
    {
    case GPS_MODE:
      if (sv_measurement->prn < 40)
      {
        user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_GPS;
        if (sv_measurement->freq_index == 0)
        {
          user_nmea.sv_list[i].svid = sv_measurement->prn;
        }
        else if (sv_measurement->freq_index == 1)
        {
          user_nmea.sv_list[i].svid = sv_measurement->prn + 400;
        }
        if (sv_measurement->freq_index == 2)
        {
          user_nmea.sv_list[i].svid = sv_measurement->prn + 500;
        }
      }
      else
      {
        user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_QZSS;
        if (sv_measurement->freq_index == 0)
        {
          user_nmea.sv_list[i].svid = sv_measurement->prn;
        }
        else if (sv_measurement->freq_index == 1)
        {
          user_nmea.sv_list[i].svid = sv_measurement->prn + 202 - 192;
        }
        if (sv_measurement->freq_index == 2)
        {
          user_nmea.sv_list[i].svid = sv_measurement->prn + 212 - 192;
        }
      }
      break;
    case GLN_MODE:
      user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_GLONASS;
      user_nmea.sv_list[i].svid = sv_measurement->prn + 64;
      break;
    case BDS_MODE:
      user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_BEIDOU;
      if (sv_measurement->freq_index == 0)
      {
        user_nmea.sv_list[i].svid = sv_measurement->prn + 140;
      }
      else if (sv_measurement->freq_index == 1)
      {
        user_nmea.sv_list[i].svid = sv_measurement->prn + 540;
      }
      else if (sv_measurement->freq_index == 2)
      {
        user_nmea.sv_list[i].svid = sv_measurement->prn + 640;
      }
      break;
    case GAL_MODE:
      user_nmea.sv_list[i].constellation = GNSS_NMEA_CONSTELLATION_GALILEO;
      if (sv_measurement->freq_index == 0)
      {
        user_nmea.sv_list[i].svid = sv_measurement->prn + 300;
      }
      else if (sv_measurement->freq_index == 1)
      {
        user_nmea.sv_list[i].svid = sv_measurement->prn + 650;
      }
      else if (sv_measurement->freq_index == 2)
      {
        user_nmea.sv_list[i].svid = sv_measurement->prn + 600;
      }
      break;
    default:break;
    }
    user_nmea.sv_list[i].c_n0_dbhz = (float)sv_measurement->cno;
    user_nmea.sv_list[i].elevation = sv_measurement->sv_info.fltElev;
    user_nmea.sv_list[i].azimuth = sv_measurement->sv_info.fltAz;

    if ((sv_measurement->status & 0x01) == 0x01)
    {
      user_nmea.sv_list[i].flags = GNSS_NMEA_SV_FLAGS_USED_IN_FIX;
    }
  }

  user_nmea.num_svs = (meas_blk->measCnt < GNSS_NMEA_MAX_MEASUREMENT) ? meas_blk->measCnt : GNSS_NMEA_MAX_MEASUREMENT;

  {
    gnss_nmea_sv_info temp = { 0 };
    int isChange = 0;
    int len = user_nmea.num_svs;

    for (int i = 0; i < len - 1; i++)
    {
      isChange = 0;

      for (int j = 0; j < len - i - 1; j++)
      {
        int charge = 0;

        if (user_nmea.sv_list[j].constellation > user_nmea.sv_list[j + 1].constellation)
        {
          charge = 1;
        }
        else if (user_nmea.sv_list[j].constellation == user_nmea.sv_list[j + 1].constellation)
        {
          if (user_nmea.sv_list[j].svid > user_nmea.sv_list[j + 1].svid)
          {
            charge = 1;
          }
        }

        if (charge)
        {
          memcpy(&temp, &user_nmea.sv_list[j], sizeof(gnss_nmea_sv_info));
          memcpy(&user_nmea.sv_list[j], &user_nmea.sv_list[j + 1], sizeof(gnss_nmea_sv_info));
          memcpy(&user_nmea.sv_list[j + 1], &temp, sizeof(gnss_nmea_sv_info));
          isChange = 1;
        }
      }

      if (!isChange)
      {
        break;
      }
    }
  }


#endif

  if (nmeainfo == NULL) {
    return;
  }

  memset(nmeainfo, 0, sizeof(gnss_NmeaInfo_t));
  gnss_nmea_extract_gga(&user_nmea, nmeainfo->gga, &(user_pvt->quasi_geoid_h));
  gnss_nmea_extract_rmc(&user_nmea, nmeainfo->rmc);
  gnss_nmea_extract_gsa(&user_nmea, nmeainfo->gsa, &nmeainfo->gsa_num, sizeof(nmeainfo->gsa) / sizeof(nmeainfo->gsa[0]));
  gnss_nmea_extract_gsv(&user_nmea, nmeainfo->gsv, &nmeainfo->gsv_num, sizeof(nmeainfo->gsv) / sizeof(nmeainfo->gsv[0]));
  gnss_nmea_extract_zda(&user_nmea, nmeainfo->zda);
  gnss_nmea_extract_gst(&user_nmea, nmeainfo->gst, user_pvt);

  return;
}

static void gnss_engine_report_location(gnss_NavSolution_t* location_info)
{
  if (NULL != gz_GnssEngineCb.report_location)
  {
    (*gz_GnssEngineCb.report_location)(location_info);
  }
}

static void gnss_engine_report_log(int level, char* log, int length)
{
  if (NULL != gz_GnssEngineCb.report_log)
  {
    (*gz_GnssEngineCb.report_log)(level, log, length);
  }
}

void gnss_engine_set_callback(GnssEngineCallback_t* callback)
{
  if (NULL != callback)
  {
    gz_GnssEngineCb.report_location = callback->report_location;
    gz_GnssEngineCb.report_log = callback->report_log;
  }
  return;
}

void gnss_engine_init(float f_eleMask)
{
  Gnss_Cfg_t gnss_cfg = { 0 };
  Gnss_PE_DefaultCfg(&gnss_cfg);
  gnss_cfg.meas_rate = MEAS_RATE_1HZ;
  gnss_cfg.ele_mask = (double)f_eleMask;
  gnss_Pe_Init(&gnss_cfg);
  return;
}

void gnss_engine_release()
{
  gnss_Pe_Close();
  return;
}

/**
 * get the position result from PVT1.0
 * @param[in,out] pz_PositionFix
 */
void gnss_engine_get_position_fix(gnss_PositionFix_t *pz_PositionFix)
{
  if (pz_PositionFix != NULL)
  {
    memcpy(pz_PositionFix, &gz_GnssEnginePositionFix, sizeof(gz_GnssEnginePositionFix));
  }
}

/**
 * @brief      get flag whether vehicle is parking judge bu peMode
 * @return     1 represent parking and 0 represent not
 */
uint8_t gnss_engine_get_peMode_stastic_flag()
{
  uint8_t b_staticFlag = FALSE;

  if ((peMode.staticData.historyStatic & 0x3) == 0x3)
  {
    b_staticFlag = TRUE;
  }

  if (peMode.staticData.lsVel >= 0.1 || peMode.staticData.deltaDoplVar >= 0.1)
  {
    b_staticFlag = FALSE;
  }

  return b_staticFlag;
}

/**
 * @brief      get flag whether vehicle is parking judge by peMode
 * @return     1 represent parking and 0 represent not
 */
uint8_t gnss_engine_get_peMode_historyStastic_flag()
{
  uint8_t b_staticFlag = FALSE;

  if ((peMode.staticData.historyStatic & 0x3) == 0x3)
  {
    b_staticFlag = TRUE;
  }

  return b_staticFlag;
}

/**
 * @brief fillup the sv information
 * @param[in/out]  z_SvStatus is the sv information
 * @param[in]  meas_blk is the measurement block
 * @return     void
 */
void gnssEnginePositionSatFill(gnss_PositionFix_SvStatus_t* z_SvStatus, meas_blk_t* meas_blk, GnssMeasBlock_t* pz_gnssMeasBlock)
{
  gnss_PositionFixSV_t z_SVtemp = { 0 };
  uint8_t u_isChange = 0;
  uint8_t u_MeasCount = 0;
  uint8_t u_MeasInUseCount = 0;
  uint8_t u_i = 0;
  uint8_t u_k = 0;

  for (u_i = 0; u_i < meas_blk->measCnt; u_i++)
  {
    gnss_meas_t* sv_measurement = &(meas_blk->meas[u_i]);
    if (u_i >= MAX_MEAS_NUM)
    {
      break;
    }
    if (0 == sv_measurement->prn)
    {
      continue;
    }

    switch (sv_measurement->gnssMode)
    {
    case GPS_MODE:
      if (sv_measurement->prn <= 32)
      {
        z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GPS;
        z_SvStatus->z_SV[u_i].u_svid = sv_measurement->prn;
        z_SvStatus->t_SvTrackMask[C_GNSS_GPS] |= ((uint64_t)1 << (sv_measurement->prn - 1));
        if ((sv_measurement->status & 0x01) == 0x01)
        {
          u_MeasInUseCount++;
          z_SvStatus->t_SvInUseMask[C_GNSS_GPS] |= ((uint64_t)1 << (sv_measurement->prn - 1));
        }
      }
      else
      {
        z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_QZS;
        z_SvStatus->z_SV[u_i].u_svid = sv_measurement->prn - 192;
        z_SvStatus->t_SvTrackMask[C_GNSS_QZS] |= ((uint64_t)1 << (sv_measurement->prn - 193));
        if ((sv_measurement->status & 0x01) == 0x01)
        {
          u_MeasInUseCount++;
          z_SvStatus->t_SvInUseMask[C_GNSS_QZS] |= ((uint64_t)1 << (sv_measurement->prn - 193));
        }
      }

      if (sv_measurement->freq_index == 0)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_GPS_L1C;
      }
      else if (sv_measurement->freq_index == 1)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_GPS_L2C;
      }
      else if (sv_measurement->freq_index == 2)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_GPS_L5Q;
      }
      break;
    case GLN_MODE:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GLO;
      z_SvStatus->z_SV[u_i].u_svid = sv_measurement->prn;
      z_SvStatus->t_SvTrackMask[C_GNSS_GLO] |= ((uint64_t)1 << (sv_measurement->prn - 1));
      if ((sv_measurement->status & 0x01) == 0x01)
      {
        u_MeasInUseCount++;
        z_SvStatus->t_SvInUseMask[C_GNSS_GLO] |= ((uint64_t)1 << (sv_measurement->prn - 1));
      }
      break;
    case BDS_MODE:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_BDS3;
      z_SvStatus->z_SV[u_i].u_svid = sv_measurement->prn;
      z_SvStatus->t_SvTrackMask[C_GNSS_BDS3] |= ((uint64_t)1 << (sv_measurement->prn - 1));
      if ((sv_measurement->status & 0x01) == 0x01)
      {
        u_MeasInUseCount++;
        z_SvStatus->t_SvInUseMask[C_GNSS_BDS3] |= ((uint64_t)1 << (sv_measurement->prn - 1));
      }

      if (sv_measurement->freq_index == 0)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_BDS_B1I;
      }
      else if (sv_measurement->freq_index == 1)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_BDS_B3I;
      }
      else if (sv_measurement->freq_index == 2)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_BDS_B2A;
      }
      break;
    case GAL_MODE:
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GAL;
      z_SvStatus->z_SV[u_i].u_svid = sv_measurement->prn;
      z_SvStatus->t_SvTrackMask[C_GNSS_GAL] |= ((uint64_t)1 << (sv_measurement->prn - 1));
      if ((sv_measurement->status & 0x01) == 0x01)
      {
        u_MeasInUseCount++;
        z_SvStatus->t_SvInUseMask[C_GNSS_GAL] |= ((uint64_t)1 << (sv_measurement->prn - 1));
      }

      if (sv_measurement->freq_index == 0)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_GAL_E1;
      }
      else if (sv_measurement->freq_index == 1)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_GAL_E5B;
      }
      else if (sv_measurement->freq_index == 2)
      {
        z_SvStatus->z_SV[u_i].u_signal = C_GNSS_SIG_GAL_E5A;
      }
      break;
    default:break;
    }
    z_SvStatus->z_SV[u_i].f_cn0 = (float)sv_measurement->cno;
    z_SvStatus->z_SV[u_i].f_elevation = sv_measurement->sv_info.fltElev;
    z_SvStatus->z_SV[u_i].f_azimuth = sv_measurement->sv_info.fltAz;
  }

  // for Glonass
  for (u_k = 0; u_k < pz_gnssMeasBlock->w_measCount; u_k++)
  {
    if (u_i >= MAX_MEAS_NUM)
    {
      break;
    }

    GnssMeas_t* p_meas = &pz_gnssMeasBlock->z_meas[u_k];
    if (C_GNSS_GLO == p_meas->u_constellation)
    {
      z_SvStatus->z_SV[u_i].u_constellation = C_GNSS_GLO;
      z_SvStatus->z_SV[u_i].u_svid = p_meas->u_svid;
      z_SvStatus->t_SvTrackMask[C_GNSS_GLO] |= ((uint64_t)1 << (p_meas->u_svid - 1));
      z_SvStatus->z_SV[u_i++].f_cn0 = p_meas->f_cn0;
    }
  }

  z_SvStatus->u_MeasTrackCount = u_i;
  z_SvStatus->u_MeasInUseCount = u_MeasInUseCount;
  z_SvStatus->u_SvInUseCount = gpz_Meas->validSatnum;
  z_SvStatus->u_SvTrackCount = gpz_Meas->trckedSatNum;

  u_MeasCount = z_SvStatus->u_MeasTrackCount;

  if (u_MeasCount < 1)
  {
    return;
  }

  for (uint8_t i = 0; i < u_MeasCount - 1; i++)
  {
    u_isChange = 0;

    for (uint8_t j = 0; j < u_MeasCount - i - 1; j++)
    {
      int charge = 0;

      if (z_SvStatus->z_SV[j].u_constellation > z_SvStatus->z_SV[j + 1].u_constellation)
      {
        charge = 1;
      }
      else if (z_SvStatus->z_SV[j].u_constellation == z_SvStatus->z_SV[j + 1].u_constellation)
      {
        if (z_SvStatus->z_SV[j].u_svid > z_SvStatus->z_SV[j + 1].u_svid)
        {
          charge = 1;
        }
        else if (z_SvStatus->z_SV[j].u_svid == z_SvStatus->z_SV[j + 1].u_svid)
        {
          if (z_SvStatus->z_SV[j].u_signal > z_SvStatus->z_SV[j + 1].u_signal)
          {
            charge = 1;
          }
        }
      }

      if (charge)
      {
        memcpy(&z_SVtemp, &z_SvStatus->z_SV[j], sizeof(gnss_PositionFixSV_t));
        memcpy(&z_SvStatus->z_SV[j], &z_SvStatus->z_SV[j + 1], sizeof(gnss_PositionFixSV_t));
        memcpy(&z_SvStatus->z_SV[j + 1], &z_SVtemp, sizeof(gnss_PositionFixSV_t));
        u_isChange = 1;
      }
    }

    if (!u_isChange)
    {
      break;
    }
  }
}

void gnss_engine_create_empty_pos(const GnssMeasBlock_t* pz_MeasBlock, gnss_PositionFix_t* pz_PositionFix)
{
  if (NULL != pz_PositionFix)
  {
    memset(pz_PositionFix, 0, sizeof(gnss_PositionFix_t));
    pz_PositionFix->z_gpsTime = pz_MeasBlock->z_Clock.z_gpsTime;
    pz_PositionFix->u_fixSource = FIX_SOURCE_HPP_V1;
    pz_PositionFix->u_DcpPosType = GNSS_DCP_POS_INVALID;
    pz_PositionFix->u_fixFlag = GNSS_FIX_FLAG_INVALID;
    if (NULL != gpz_user_pvt && gpz_user_pvt->posfix_wn != 0 && pz_PositionFix->z_gpsTime.q_towMsec != 0)
    {
      pz_PositionFix->z_gpsTime.w_week = gpz_user_pvt->posfix_wn;
    }
  }

  return;
}
void gnss_engine_inject_meas(GnssMeasBlock_t* pz_gnssMeasBlock, gnss_FeedbackInsMeasBlock_t* pz_CurrentFeedbackInsMeas)
{
  meas_blk_t* p_meas_blk = NULL;
  asg_obs_t* p_asg_obs = NULL;
  BOOL q_isMultiFreqMeas = FALSE;

  /* init gz_GnssEnginePositionFix */
  memset(&gz_GnssEnginePositionFix, 0, sizeof(gz_GnssEnginePositionFix));
  gz_GnssEnginePositionFix.u_fixSource = FIX_SOURCE_HPP_V1;
  gz_GnssEnginePositionFix.u_version = VERSION_POSITION_FIX;
  gz_GnssEnginePositionFix.w_size = sizeof(gz_GnssEnginePositionFix);
  gz_GnssEnginePositionFix.u_DcpPosType = GNSS_DCP_POS_INVALID;
  gz_GnssEnginePositionFix.z_gpsTime = pz_gnssMeasBlock->z_Clock.z_gpsTime;
  tm_cvt_GpstToEpoch(&gz_GnssEnginePositionFix.z_gpsTime, &gz_GnssEnginePositionFix.z_epoch);
  gz_GnssEnginePositionFix.u_leapsec = tm_LeapSecond_FromEpoch(&gz_GnssEnginePositionFix.z_epoch);
  gnss_InitIntegrityStruct(&gz_GnssEnginePositionFix.z_integrity);

  p_meas_blk = (meas_blk_t*)OS_MALLOC_FAST(sizeof(meas_blk_t));
  if (NULL == p_meas_blk)
  {
    return;
  }
  p_asg_obs = (asg_obs_t*)OS_MALLOC(sizeof(asg_obs_t));
  if (NULL == p_asg_obs)
  {
    OS_FREE(p_meas_blk);
    return;
  }
  memset(p_meas_blk, 0, sizeof(meas_blk_t));
  memset(p_asg_obs, 0, sizeof(asg_obs_t));

  p_meas_blk->tor = pz_gnssMeasBlock->z_Clock.z_gpsTime.q_towMsec * 0.001;
  q_isMultiFreqMeas = gnss_IdentifyMultiFreq(pz_gnssMeasBlock->z_meas, pz_gnssMeasBlock->w_measCount);
  if (TRUE == q_isMultiFreqMeas)
  {
    gq_isMultiFreqModel = TRUE;
  }

  for (int k = 0; k < pz_gnssMeasBlock->w_measCount; k++)
  {
    if ((p_meas_blk->measCnt) >= M_ARRAY_SIZE(p_meas_blk->meas))
    {
      /* Pass measurement more than 64 */
      break;
    }

    GnssMeas_t* p_meas = &pz_gnssMeasBlock->z_meas[k];
    gnss_meas_t* meas = &(p_meas_blk->meas[p_meas_blk->measCnt]);
    uint8_t v_meas_valid = 1;

    if (C_GNSS_GLO == p_meas->u_constellation)
    {
      continue;
    }

    meas->prn = p_meas->u_svid;
    meas->cno = (uint32_t)p_meas->f_cn0;
    meas->pseudoRange = p_meas->d_pseudoRange;
    meas->pseudoRangeRate = p_meas->d_doppler;
    meas->carrierPhase = p_meas->d_carrierPhase;
    meas->cycleSlipCount = p_meas->u_LLI;
    meas->pseudoRange_raw = p_meas->d_pseudoRange;
    meas->sv_info.p[0] = 0.0;
    meas->sv_info.p[1] = 0.0;
    meas->sv_info.p[2] = 0.0;
    meas->sv_info.v[0] = 0.0;
    meas->sv_info.v[1] = 0.0;
    meas->sv_info.v[2] = 0.0;

    if (meas->carrierPhase == 0.0)
    {
      meas->cycleSlipCount = 4;
    }

    switch (p_meas->u_constellation)
    {
    case C_GNSS_GPS:
      meas->gnssMode = GPS_MODE;
      meas->tot = p_meas_blk->tor - meas->pseudoRange / CLIGHT;
      switch (p_meas->u_signal)
      {
      case C_GNSS_SIG_GPS_L1C:
        meas->freq_index = 0;
        break;
      case C_GNSS_SIG_GPS_L2C:
        meas->freq_index = 1;
        break;
      case C_GNSS_SIG_GPS_L5Q:
        meas->freq_index = 2;
        break;
      default:
        v_meas_valid = 0;
        break;
      }
      break;
#if 0
    case C_GNSS_GLO:
      break;
#endif
    case C_GNSS_BDS3:
    case C_GNSS_BDS2:
      meas->gnssMode = BDS_MODE;
      meas->tot = p_meas_blk->tor - 14 - meas->pseudoRange / CLIGHT;
      switch (p_meas->u_signal)
      {
      case C_GNSS_SIG_BDS_B1I:
        meas->freq_index = 0;
        break;
      case C_GNSS_SIG_BDS_B3I:
        meas->freq_index = 1;
        break;
        case C_GNSS_SIG_BDS_B2I:
          meas->freq_index = 2;
          break;
      case C_GNSS_SIG_BDS_B2A:
        meas->freq_index = 2;
        break;
      default:
        v_meas_valid = 0;
        break;
      }
      break;
    case C_GNSS_GAL:
      meas->gnssMode = GAL_MODE;
      meas->tot = p_meas_blk->tor - meas->pseudoRange / CLIGHT;
      switch (p_meas->u_signal)
      {
      case C_GNSS_SIG_GAL_E1:
        meas->freq_index = 0;
        break;
      case C_GNSS_SIG_GAL_E5B:
        meas->freq_index = 1;
        break;
      case C_GNSS_SIG_GAL_E5A:
        meas->freq_index = 2;
        break;
      default:
        v_meas_valid = 0;
        break;
      }
      break;
    case C_GNSS_QZS:
      meas->gnssMode = GPS_MODE;
      meas->prn = 192 + meas->prn;
      meas->tot = p_meas_blk->tor - meas->pseudoRange / CLIGHT;
      switch (p_meas->u_signal)
      {
      case C_GNSS_SIG_QZS_L1C:
        meas->freq_index = 0;
        break;
      case C_GNSS_SIG_QZS_L2C:
        meas->freq_index = 1;
        break;
      case C_GNSS_SIG_QZS_L5Q:
        meas->freq_index = 2;
        break;
      default:
        v_meas_valid = 0;
        break;
      }
      break;
    default: break;
    }

    if ((TRUE == gq_isMultiFreqModel) && (2 == (meas->freq_index)))//filter the L5 frequecny observation for the multi-frequecny measure
    {
      continue;
    }

    if (meas->tot < 0)
    {
      meas->tot += 604800.0;
    }

    if ((meas->cno >= 35) && (meas->freq_index == 0))
    {
      p_meas_blk->Cno35Cnt[meas->gnssMode]++;
    }

    if (meas->cno >= 20)
    {
      p_meas_blk->Cno20Cnt++;
    }

    if (v_meas_valid)
    {
      meas->measState |= AGGNSS_MEASUREMENT_STATE_TOW_DECODED | AGGNSS_MEASUREMENT_STATE_MSEC_AMBIGUOUS;
      meas->status |= PE_MEAS_VALID_PR | PE_MEAS_VALID_DR | PE_MEAS_VALID_L1;
      p_meas_blk->measCnt++;
    }
  }

  int idx = 0;

  int freq_id = 0;
  memset(p_asg_obs, 0, sizeof(asg_obs_t));

  GNSS_TIME* pTime = gnss_tm_get_time();
  if (pTime == NULL)
  {
    OS_FREE(p_meas_blk);
    OS_FREE(p_asg_obs);
    return;
  }
  char* buf = NULL;
  int buf_len = 512;

  buf = (char*)OS_MALLOC_FAST(buf_len);
  for (uint32_t meas_idx = 0; meas_idx < p_meas_blk->measCnt; meas_idx++)
  {
    gnss_meas_t* m = &(p_meas_blk->meas[meas_idx]);
#if 0
    char temp[128] = { 0 };
    memset(buf, 0, 256);
    sprintf(temp, "%2d  ", m->gnssMode); strcat(buf, temp);
    sprintf(temp, "%3d  ", m->prn); strcat(buf, temp);
    sprintf(temp, "%2d  ", m->cno); strcat(buf, temp);
    sprintf(temp, "%1d  ", m->freq_index); strcat(buf, temp);
    sprintf(temp, "%2d  ", m->status); strcat(buf, temp);
    sprintf(temp, "%2d  ", m->measState); strcat(buf, temp);
    sprintf(temp, "%2d  ", m->measState); strcat(buf, temp);
    sprintf(temp, "%16.9f  ", m->tot); strcat(buf, temp);
    sprintf(temp, "%17.9f  ", m->pseudoRange); strcat(buf, temp);
    sprintf(temp, "%16.9f  ", m->pseudoRangeRate); strcat(buf, temp);
    sprintf(temp, "%20.9f  ", m->carrierPhase); strcat(buf, temp);
    sprintf(temp, "%d  ", m->cycleSlipCount); strcat(buf, temp);
    LOGI(TAG_HPP, "%s\n", buf);
#endif

    asg_obsd_t* asg_obsd = NULL;

    if (m->freq_index == 0)
    {
      asg_obsd = &(p_asg_obs->data[p_asg_obs->n++]);
    }
    else
    {
      for (idx = 0; idx < p_asg_obs->n; idx++)
      {
        if (p_asg_obs->data[idx].prn == m->prn)
        {
          if (m->gnssMode == GPS_MODE && p_asg_obs->data[idx].sys == SYS_GPS)
          {
            asg_obsd = &p_asg_obs->data[idx];
            break;
          }
          else if (m->gnssMode == GPS_MODE && p_asg_obs->data[idx].sys == SYS_QZS)
          {
            asg_obsd = &p_asg_obs->data[idx];
            break;
          }
          else if (m->gnssMode == BDS_MODE && p_asg_obs->data[idx].sys == SYS_CMP)
          {
            asg_obsd = &p_asg_obs->data[idx];
            break;
          }
          else if (m->gnssMode == GAL_MODE && p_asg_obs->data[idx].sys == SYS_GAL)
          {
            asg_obsd = &p_asg_obs->data[idx];
            break;
          }
        }
      }

      if (asg_obsd == NULL)
      {
        asg_obsd = &(p_asg_obs->data[p_asg_obs->n++]);
      }
    }

    gtime_t time = { 0 };
    switch (m->gnssMode)
    {
    case GPS_MODE: time = gnss_gpst2time((int)pTime->week[GPS_MODE], gpz_Meas->tor); break;
    case GLN_MODE: time = gnss_gpst2time((int)pTime->week[GLN_MODE], gpz_Meas->tor); break;
    case BDS_MODE: time = bdt2time((int)pTime->week[BDS_MODE], gpz_Meas->tor); break;
    case GAL_MODE: time = gst2time((int)pTime->week[GAL_MODE], gpz_Meas->tor); break;
    default: break;
    }

    asg_obsd->time.time = time.time;
    asg_obsd->time.sec = time.sec;
    asg_obsd->prn = m->prn;
    freq_id = m->freq_index;

    if (NFREQ == 1)
    {
      if ((freq_id == 1) || (freq_id == 2))
      {
        OS_FREE(p_meas_blk);
        OS_FREE(p_asg_obs);
        return;
      }
    }
    else if (NFREQ == 2)
    {
      if (freq_id == 2)
      {
        freq_id = 1;
      }
    }
    else if (NFREQ == 3)
    {
      /* do nothing */
    }
    else
    {
      OS_FREE(p_meas_blk);
      OS_FREE(p_asg_obs);
      return;
    }

    switch (m->gnssMode)
    {
    case GPS_MODE:
      asg_obsd->sys = SYS_GPS;
      if (asg_obsd->prn > 192 && asg_obsd->prn <= 199)
      {
        asg_obsd->sys = SYS_QZS;
      }
      switch (m->freq_index)
      {
      case 0: asg_obsd->code[freq_id] = CODE_L1C; break;
      case 1: asg_obsd->code[freq_id] = CODE_L2W; break;
      case 2: asg_obsd->code[freq_id] = CODE_L5Q; break;
      }
      break;
    case GLN_MODE:
      asg_obsd->sys = SYS_GLO;
      switch (m->freq_index)
      {
      case 0:break;
      case 1:break;
      case 2:break;
      }
      break;
    case BDS_MODE:
      asg_obsd->sys = SYS_CMP;
      switch (m->freq_index)
      {
      case 0: asg_obsd->code[freq_id] = CODE_L1I; break;
      case 1: asg_obsd->code[freq_id] = CODE_L7I; break;
      case 2: asg_obsd->code[freq_id] = CODE_L5D; break;
      }
      break;
    case GAL_MODE:
      asg_obsd->sys = SYS_GAL;
      switch (m->freq_index)
      {
      case 0: asg_obsd->code[freq_id] = CODE_L1C; break;
      case 1: asg_obsd->code[freq_id] = CODE_L7I; break;
      case 2: asg_obsd->code[freq_id] = CODE_L5I; break;
      }
      break;
    }

    asg_obsd->sat = satno(asg_obsd->sys, asg_obsd->prn);
    asg_obsd->SNR[freq_id] = m->cno;
    asg_obsd->P[freq_id] = m->pseudoRange;
    asg_obsd->D[freq_id] = (float)m->pseudoRangeRate;
    asg_obsd->L[freq_id] = m->carrierPhase;
    asg_obsd->LLI[freq_id] = m->cycleSlipCount;
  }
  OS_FREE(buf);

  memcpy(gpz_Meas, p_meas_blk, sizeof(meas_blk_t));
  OS_FREE(p_meas_blk);

  /* V1.0 PVT process */
  gnss_Pe_Exec(gpz_Meas, p_asg_obs, pz_CurrentFeedbackInsMeas);

  OS_FREE(p_asg_obs);

  USER_PVT* pvt = gpz_user_pvt;
  //GNSS_TIME* ptime = gnss_tm_get_time();
  switch (gpz_user_pvt->diff_status)
  {
  case DIFF_SOL_STATUS_STANDALONE:
    gz_GnssEnginePositionFix.u_fixFlag = GNSS_FIX_FLAG_SPS;
    break;
  case DIFF_SOL_STATUS_RTD:
    gz_GnssEnginePositionFix.u_fixFlag = GNSS_FIX_FLAG_DGNSS;
    break;
  case DIFF_SOL_STATUS_RTK_FLOAT:
    gz_GnssEnginePositionFix.u_fixFlag = GNSS_FIX_FLAG_FLOATING;
    break;
  case DIFF_SOL_STATUS_RTK_FIX:
    gz_GnssEnginePositionFix.u_fixFlag = GNSS_FIX_FLAG_FIXED;
    break;
  case DIFF_SOL_STATUS_RTK_PROPAGATE:
    gz_GnssEnginePositionFix.u_fixFlag = GNSS_FIX_FLAG_DR;
    break;
  default:
    gz_GnssEnginePositionFix.u_fixFlag = GNSS_FIX_FLAG_INVALID;
    break;
  }

  if (GNSS_FIX_FLAG_INVALID != gz_GnssEnginePositionFix.u_fixFlag)
  {
    gz_GnssEnginePositionFix.d_avgCN0 = pvt->avgCNO;
    gz_GnssEnginePositionFix.u_CN040 = pvt->CN040;
    gz_GnssEnginePositionFix.d_quasiGeoidHeight = pvt->geoidal_sep;
    gz_GnssEnginePositionFix.d_lla[0] = pvt->lla.pos[0]; /* Latitude (rad) */
    gz_GnssEnginePositionFix.d_lla[1] = pvt->lla.pos[1]; /* Longitude (rad) */
    gz_GnssEnginePositionFix.d_lla[2] = pvt->lla.pos[2];
    gz_GnssEnginePositionFix.d_xyz[0] = pvt->ecef.pos[0];
    gz_GnssEnginePositionFix.d_xyz[1] = pvt->ecef.pos[1];
    gz_GnssEnginePositionFix.d_xyz[2] = pvt->ecef.pos[2];
    gz_GnssEnginePositionFix.f_velXyz[0] = pvt->ecef.vel[0];
    gz_GnssEnginePositionFix.f_velXyz[1] = pvt->ecef.vel[1];
    gz_GnssEnginePositionFix.f_velXyz[2] = pvt->ecef.vel[2];
    gz_GnssEnginePositionFix.f_velEnu[0] = pvt->lla.vel[0];
    gz_GnssEnginePositionFix.f_velEnu[1] = pvt->lla.vel[1];
    gz_GnssEnginePositionFix.f_velEnu[2] = pvt->lla.vel[2];
    gz_GnssEnginePositionFix.f_posLlaUnc[0] = pvt->accu.accu1_north;
    gz_GnssEnginePositionFix.f_posLlaUnc[1] = pvt->accu.accu1_east;
    gz_GnssEnginePositionFix.f_posLlaUnc[2] = pvt->accu.accu1_up;
    gz_GnssEnginePositionFix.f_posXyzUnc[0] = 0.0f;
    gz_GnssEnginePositionFix.f_posXyzUnc[1] = 0.0f;
    gz_GnssEnginePositionFix.f_posXyzUnc[2] = 0.0f;
    gz_GnssEnginePositionFix.f_velEnuUnc[0] = pvt->posErr.ve_err;
    gz_GnssEnginePositionFix.f_velEnuUnc[1] = pvt->posErr.vn_err;
    gz_GnssEnginePositionFix.f_velEnuUnc[2] = pvt->posErr.vd_err;
    float f_hor_pl = sqrtf(pvt->accu.accu2_north * pvt->accu.accu2_north + pvt->accu.accu2_east * pvt->accu.accu2_east);
    gz_GnssEnginePositionFix.z_integrity.z_pos_hor_pl = gnss_genProtectionLevelStruct(f_hor_pl);
    gz_GnssEnginePositionFix.z_integrity.z_pos_ver_pl = gnss_genProtectionLevelStruct(pvt->accu.accu2_up);
    gz_GnssEnginePositionFix.z_integrity.z_pos_north_pl = gnss_genProtectionLevelStruct(pvt->accu.accu2_north);
    gz_GnssEnginePositionFix.z_integrity.z_pos_east_pl = gnss_genProtectionLevelStruct(pvt->accu.accu2_east);
    gz_GnssEnginePositionFix.z_dops.f_gdop = sqrtf(pvt->DOP.tDOP * pvt->DOP.tDOP + pvt->DOP.pDOP * pvt->DOP.pDOP);
    gz_GnssEnginePositionFix.z_dops.f_pdop = M_MIN(99.9f, pvt->DOP.pDOP);
    gz_GnssEnginePositionFix.z_dops.f_hdop = M_MIN(99.9f, pvt->DOP.hDOP);
    gz_GnssEnginePositionFix.z_dops.f_vdop = M_MIN(99.9f, pvt->DOP.vDOP);
    gz_GnssEnginePositionFix.z_dops.f_tdop = M_MIN(99.9f, pvt->DOP.tDOP);
  }
  gnssEnginePositionSatFill(&gz_GnssEnginePositionFix.z_SvStatus, gpz_Meas, pz_gnssMeasBlock);

#if 0
  /* Report log */
  char* status_str = NULL;
  switch (gz_GnssEnginePositionFix.u_fixFlag)
  {
  case GNSS_FIX_FLAG_SPS:
    status_str = "SPS";
    break;
  case GNSS_FIX_FLAG_DGNSS:
    status_str = "RTD";
    break;
  case GNSS_FIX_FLAG_FLOATING:
    status_str = "Float";
    break;
  case GNSS_FIX_FLAG_FIXED:
    status_str = "Fix";
    break;
  case GNSS_FIX_FLAG_DR:
    status_str = "DR";
    break;
  default:
    status_str = "Invaild";
    break;
  }
#endif
  if (GNSS_FIX_FLAG_INVALID == gz_GnssEnginePositionFix.u_fixFlag)
  {
    GLOGW("----GNSS PVT Engine Failed %.0f, meas count: %d",
          pz_gnssMeasBlock->z_Clock.z_gpsTime.q_towMsec * TIME_MSEC_INV, pz_gnssMeasBlock->w_measCount);
  }
  else
  {
    GLOGI("----GNSS PVT Engine Success Week %d Tow %f, meas count %d",
          gz_GnssEnginePositionFix.z_gpsTime.w_week,
          gz_GnssEnginePositionFix.z_gpsTime.q_towMsec * 0.001, pz_gnssMeasBlock->w_measCount);
#if 0
    GLOGW("Position Lat %12.6f Lon %12.6f Alt %12.6f Status %s ",
          gz_GnssEnginePositionFix.d_lla[0] * R2D,
          gz_GnssEnginePositionFix.d_lla[1] * R2D,
          gz_GnssEnginePositionFix.d_lla[2],
          status_str);
    GLOGW("Velocity E   %12.6f N   %12.6f U   %12.6f",
          gz_GnssEnginePositionFix.f_velEnu[0],
          gz_GnssEnginePositionFix.f_velEnu[1],
          gz_GnssEnginePositionFix.f_velEnu[2]);
#endif
    GLOGW("BLH: %12.6f %12.6f %9.3f,ENU: %8.3f %8.3f %8.3f",
      gz_GnssEnginePositionFix.d_lla[0] * R2D, gz_GnssEnginePositionFix.d_lla[1] * R2D, gz_GnssEnginePositionFix.d_lla[2],
      gz_GnssEnginePositionFix.f_velEnu[0], gz_GnssEnginePositionFix.f_velEnu[1], gz_GnssEnginePositionFix.f_velEnu[2]);
  }

  if (gz_GnssEnginePositionFix.z_gpsTime.w_week != gpz_user_pvt->posfix_wn)
  {
    LOGE(TAG_HPP, "GPS week match error, rtk2.0:%d, rtk1.0:%d\n", gz_GnssEnginePositionFix.z_gpsTime.w_week, gpz_user_pvt->posfix_wn);
  }
  return;
}

void gnss_engine_inject_correct(GnssCorrBlock_t* pz_CorrBlk)
{
  asg_rtcm_lite_t* p_rtcm = NULL;
  asg_obs_t* p_qxwz_obs = NULL;
  int freq_id = 0;
  int prn = 0;

  GNSS_TIME* pTime = gnss_tm_get_time();
  if (pTime == NULL) {
    return;
  }

  p_rtcm = (asg_rtcm_lite_t* )OS_MALLOC_FAST(sizeof(asg_rtcm_lite_t));
  p_qxwz_obs = (asg_obs_t* )OS_MALLOC_FAST(sizeof(asg_obs_t));

  memset(p_rtcm, 0, sizeof(asg_rtcm_lite_t));
  memset(p_qxwz_obs,0,sizeof(asg_obs_t));

  for (int meas_idx = 0; meas_idx < pz_CorrBlk->w_measCount; meas_idx++)
  {
    GnssMeas_t* m = &pz_CorrBlk->z_meas[meas_idx];
    asg_obsd_t* asg_obsd = NULL;
    
    prn = m->u_svid;
    if (m->u_constellation == C_GNSS_QZS)
    {
      prn += 192;
    }

    if (C_GNSS_FREQ_TYPE_L1 == gnss_cvt_Sig2FreqType(m->u_signal))
    {
      asg_obsd = &(p_qxwz_obs->data[p_qxwz_obs->n++]);
    }
    else
    {
      for (int idx = 0; idx < p_qxwz_obs->n; idx++)
      {
        if (p_qxwz_obs->data[idx].prn == prn)
        {
          if (m->u_constellation == C_GNSS_GPS && p_qxwz_obs->data[idx].sys == SYS_GPS)
          {
            asg_obsd = &p_qxwz_obs->data[idx];
            break;
          }
          else if (m->u_constellation == C_GNSS_QZS && p_qxwz_obs->data[idx].sys == SYS_QZS)
          {
            asg_obsd = &p_qxwz_obs->data[idx];
            break;
          }
          else if (m->u_constellation == C_GNSS_BDS3 && p_qxwz_obs->data[idx].sys == SYS_CMP)
          {
            asg_obsd = &p_qxwz_obs->data[idx];
            break;
          }
          else if (m->u_constellation == C_GNSS_GAL && p_qxwz_obs->data[idx].sys == SYS_GAL)
          {
            asg_obsd = &p_qxwz_obs->data[idx];
            break;
          }
        }
      }

      if (asg_obsd == NULL)
      {
        asg_obsd = &(p_qxwz_obs->data[p_qxwz_obs->n++]);
      }
    }

    gtime_t time = { 0 };
    time = gnss_gpst2time((int)pTime->week[GPS_MODE], 0.001 * pz_CorrBlk->z_Clock.z_gpsTime.q_towMsec);

    asg_obsd->time.time = time.time;
    asg_obsd->time.sec = time.sec;
    asg_obsd->prn = prn;
    asg_obsd->rcv = 1;

    freq_id = gnss_cvt_Sig2FreqType(m->u_signal);

    if (NFREQ == 1) {
      if ((freq_id == 1) || (freq_id == 2)) {
	  	OS_FREE(p_rtcm);
		OS_FREE(p_qxwz_obs);
        return;
      }
    }
    else if (NFREQ == 2) {
      if (freq_id == 2) {
        freq_id = 1;
      }
    }
    else if (NFREQ == 3) {
      /* do nothing */
    }
    else {
	  OS_FREE(p_rtcm);
	  OS_FREE(p_qxwz_obs);
      return;
    }
    
    if (freq_id == 3)
    {
      continue;
    }

    gnss_FreqType freqType = gnss_cvt_Sig2FreqType(m->u_signal);
    switch (m->u_constellation)
    {
    case C_GNSS_GPS:
      asg_obsd->sys = SYS_GPS;
      switch (freqType)
      {
      case C_GNSS_FREQ_TYPE_L1: asg_obsd->code[freq_id] = CODE_L1C; break;
      case C_GNSS_FREQ_TYPE_L2: asg_obsd->code[freq_id] = CODE_L2W; break;
      case C_GNSS_FREQ_TYPE_L5: asg_obsd->code[freq_id] = CODE_L5Q; break;
      }
      break;
    case C_GNSS_GLO:
      asg_obsd->sys = SYS_GLO;
      switch (freqType)
      {
      case C_GNSS_FREQ_TYPE_L1:break;
      case C_GNSS_FREQ_TYPE_L2:break;
      case C_GNSS_FREQ_TYPE_L5:break;
      }
      break;
    case C_GNSS_BDS3:
      asg_obsd->sys = SYS_CMP;
      switch (freqType)
      {
      case C_GNSS_FREQ_TYPE_L1: asg_obsd->code[freq_id] = CODE_L1I; break;
      case C_GNSS_FREQ_TYPE_L2: asg_obsd->code[freq_id] = CODE_L7I; break;
      case C_GNSS_FREQ_TYPE_L5: asg_obsd->code[freq_id] = CODE_L5D; break;
      }
      break;
    case C_GNSS_GAL:
      asg_obsd->sys = SYS_GAL;
      switch (freqType)
      {
      case C_GNSS_FREQ_TYPE_L1: asg_obsd->code[freq_id] = CODE_L1C; break;
      case C_GNSS_FREQ_TYPE_L2: asg_obsd->code[freq_id] = CODE_L7I; break;
      case C_GNSS_FREQ_TYPE_L5: asg_obsd->code[freq_id] = CODE_L5I; break;
      }
      break;
    case C_GNSS_QZS:
      asg_obsd->sys = SYS_QZS;
      asg_obsd->prn = prn;
      switch (freqType)
      {
      case C_GNSS_FREQ_TYPE_L1: asg_obsd->code[freq_id] = CODE_L1C; break;
      case C_GNSS_FREQ_TYPE_L2: asg_obsd->code[freq_id] = CODE_L2W; break;
      case C_GNSS_FREQ_TYPE_L5: asg_obsd->code[freq_id] = CODE_L5Q; break;
      }
      break;
    default:
      break;
    }

    asg_obsd->sat = satno(asg_obsd->sys, asg_obsd->prn);
    asg_obsd->SNR[freq_id] = (unsigned char)m->f_cn0;
    asg_obsd->P[freq_id] = m->d_pseudoRange;
    asg_obsd->D[freq_id] = (float)m->d_doppler;
    asg_obsd->L[freq_id] = m->d_carrierPhase;
    asg_obsd->LLI[freq_id] = m->u_LLI;
  }

  memcpy(&p_rtcm->obs, p_qxwz_obs, sizeof(asg_obs_t));
  OS_FREE(p_qxwz_obs);
  p_rtcm->sta.pos[0] = pz_CorrBlk->d_refPosEcef[0];
  p_rtcm->sta.pos[1] = pz_CorrBlk->d_refPosEcef[1];
  p_rtcm->sta.pos[2] = pz_CorrBlk->d_refPosEcef[2];
  
  gtime_t t = gnss_gpst2time(pTime->week[0],
    pz_CorrBlk->z_Clock.z_gpsTime.q_towMsec * 0.001);
  p_rtcm->time.sec = t.sec;
  p_rtcm->time.time = t.time;
  gnss_rtk_rtcm3_lite_proc(p_rtcm);
  OS_FREE(p_rtcm);
  return;
}

void gnss_engine_inject_ephemeris(gnss_Ephemeris_t* pz_eph)
{
  if (pz_eph->u_constellation == C_GNSS_GPS)
  {
    GpsEphemeris_t* eph = &(pz_eph->eph.z_gpsEph);
    GPS_EPH_INFO dst_eph = { 0 };

    dst_eph.eph_status = EPH_STATUS_VALID;
    dst_eph.eph_source = 0;
    dst_eph.IODE = eph->iode;
    dst_eph.fit_interval = eph->fit;
    dst_eph.C_rs = (float)eph->crs;
    dst_eph.C_rc = (float)eph->crc;
    dst_eph.C_us = (float)eph->cus;
    dst_eph.C_uc = (float)eph->cuc;
    dst_eph.C_is = (float)eph->cis;
    dst_eph.C_ic = (float)eph->cic;
    dst_eph.t_oe = (float)eph->toe;
    dst_eph.delta_n = eph->DeltaN;
    dst_eph.M_0 = eph->M0;
    dst_eph.e = eph->e;
    dst_eph.sqrt_A = eph->sqrt_A;
    dst_eph.OMEGA_0 = eph->Omega0;
    dst_eph.i_0 = eph->i0;
    dst_eph.omega = eph->Omega;
    dst_eph.OMEGADOT = eph->OmegaDot;
    dst_eph.IDOT = eph->idot;
    dst_eph.subframe1.weeknum = eph->week;
    dst_eph.subframe1.codeL2 = eph->code;
    dst_eph.subframe1.L2Pdata = 0;
    dst_eph.subframe1.udre = eph->accuracy;
    dst_eph.subframe1.SV_health = eph->health;
    dst_eph.subframe1.IODC = eph->iodc;
    dst_eph.subframe1.T_GD = (float)eph->tgd;
    dst_eph.subframe1.t_oc = (float)eph->toc;
    dst_eph.subframe1.a_f0 = (float)eph->af0;
    dst_eph.subframe1.a_f1 = (float)eph->af1;
    dst_eph.subframe1.a_f2 = (float)eph->af2;
    if (eph->accuracy < 6)
    {
      /* SV accuracy; meters*/
      dst_eph.subframe1.SVacc = (float)pow(2, (1.0 + eph->accuracy / 2));
    }
    else
    {
      /* SV accuracy; meters*/
      dst_eph.subframe1.SVacc = (float)pow(2, (eph->accuracy - 2));
    }

    gnss_Sd_Nm_AddEph(GPS_MODE, eph->svid, &dst_eph);
  }
  else if (pz_eph->u_constellation == C_GNSS_GLO)
  {
    /* Not support GLN */
  }
  else if (C_GNSS_BDS3 ==(pz_eph->u_constellation)|| C_GNSS_BDS2 == (pz_eph->u_constellation))
  {
    BdsEphemeris_t* eph = &(pz_eph->eph.z_bdsEph);
    BDS_EPH_INFO dst_eph = { 0 };

    dst_eph.prn = eph->svid;
    dst_eph.eph_status = EPH_STATUS_VALID;
    dst_eph.eph_source = 0;
    dst_eph.EphDecFlag = 0;//////////////////
    dst_eph.SatH1 = eph->health;
    dst_eph.IODC = (uint8_t)eph->iodc;
    dst_eph.IODE = eph->iode;
    dst_eph.URAI = eph->accuracy;
    dst_eph.WN = eph->week;
    dst_eph.toc = (int32_t)eph->toc;
    dst_eph.toe = (int32_t)eph->toe;
    dst_eph.af0 = eph->af0;
    dst_eph.af1 = (float)eph->af1;
    dst_eph.af2 = (float)eph->af2;
    dst_eph.TGD1 = (float)eph->tgdB1I;
    dst_eph.TGD2 = (float)eph->tgdB2I;
    dst_eph.cuc = (float)eph->cuc;
    dst_eph.cus = (float)eph->cus;
    dst_eph.cic = (float)eph->cic;
    dst_eph.cis = (float)eph->cis;
    dst_eph.crc = (float)eph->crc;
    dst_eph.crs = (float)eph->crs;
    dst_eph.delta_n = eph->DeltaN;
    dst_eph.M0 = eph->M0;
    dst_eph.ecc = eph->e;
    dst_eph.sqrta = eph->sqrt_A;
    dst_eph.OMEGA_0 = eph->Omega0;
    dst_eph.OMEGA_Dot = eph->OmegaDot;
    dst_eph.idot = eph->idot;
    dst_eph.i0 = eph->i0;
    dst_eph.w = eph->Omega;

    gnss_Sd_Nm_AddEph(BDS_MODE, eph->svid, &dst_eph);
  }
  else if (pz_eph->u_constellation == C_GNSS_GAL)
  {
    GalEphemeris_t* eph = &(pz_eph->eph.z_galEph);
    GAL_EPH_INFO dst_eph = { 0 };
    
    dst_eph.prn = eph->svid;
    dst_eph.eph_status = EPH_STATUS_VALID;
    dst_eph.eph_source = 0;
    dst_eph.WN = eph->week;
    dst_eph.IOD = eph->iode;
    dst_eph.svHealth = eph->E1health;
    dst_eph.sisa = eph->accuracy;
    dst_eph.E1BDVS = eph->E1DVS;
    dst_eph.E1BSHS = eph->E1health;
    dst_eph.toc = (int32_t)eph->toc;
    dst_eph.toe = (int32_t)eph->toe;
    dst_eph.af0 = eph->af0;
    dst_eph.af1 = (float)eph->af1;
    dst_eph.af2 = (float)eph->af2;
    dst_eph.bgd_e5a = eph->tgdE1E5a;
    dst_eph.bgd_e5b = eph->tgdE1E5b;
    dst_eph.cuc = (float)eph->cuc;
    dst_eph.cus = (float)eph->cus;
    dst_eph.cic = (float)eph->cic;
    dst_eph.cis = (float)eph->cis;
    dst_eph.crc = (float)eph->crc;
    dst_eph.crs = (float)eph->crs;
    dst_eph.delta_n = eph->DeltaN;
    dst_eph.M0 = eph->M0;
    dst_eph.ecc = eph->e;
    dst_eph.sqrta = eph->sqrt_A;
    dst_eph.OMEGA_0 = eph->Omega0;
    dst_eph.OMEGA_Dot = eph->OmegaDot;
    dst_eph.idot = eph->idot;
    dst_eph.i0 = eph->i0;
    dst_eph.w = eph->Omega;

    gnss_Sd_Nm_AddEph(GAL_MODE, eph->svid, &dst_eph);
  }
  else if (pz_eph->u_constellation == C_GNSS_QZS)
  {
    GpsEphemeris_t* eph = &(pz_eph->eph.z_qzsEph);
    GPS_EPH_INFO dst_eph = { 0 };

    dst_eph.eph_status = EPH_STATUS_VALID;
    dst_eph.eph_source = 0;
    dst_eph.IODE = eph->iode;
    dst_eph.fit_interval = eph->fit;
    dst_eph.C_rs = (float)eph->crs;
    dst_eph.C_rc = (float)eph->crc;
    dst_eph.C_us = (float)eph->cus;
    dst_eph.C_uc = (float)eph->cuc;
    dst_eph.C_is = (float)eph->cis;
    dst_eph.C_ic = (float)eph->cic;
    dst_eph.t_oe = (float)eph->toe;
    dst_eph.delta_n = eph->DeltaN;
    dst_eph.M_0 = eph->M0;
    dst_eph.e = eph->e;
    dst_eph.sqrt_A = eph->sqrt_A;
    dst_eph.OMEGA_0 = eph->Omega0;
    dst_eph.i_0 = eph->i0;
    dst_eph.omega = eph->Omega;
    dst_eph.OMEGADOT = eph->OmegaDot;
    dst_eph.IDOT = eph->idot;
    dst_eph.subframe1.weeknum = eph->week;
    dst_eph.subframe1.codeL2 = eph->code;
    dst_eph.subframe1.L2Pdata = 0;
    dst_eph.subframe1.udre = eph->accuracy;
    dst_eph.subframe1.SV_health = eph->health;
    dst_eph.subframe1.IODC = eph->iodc;
    dst_eph.subframe1.T_GD = (float)eph->tgd;
    dst_eph.subframe1.t_oc = (float)eph->toc;
    dst_eph.subframe1.a_f0 = (float)eph->af0;
    dst_eph.subframe1.a_f1 = (float)eph->af1;
    dst_eph.subframe1.a_f2 = (float)eph->af2;
    if (eph->accuracy < 6)
    {
      /* SV accuracy; meters*/
      dst_eph.subframe1.SVacc = (float)pow(2, (1.0 + eph->accuracy / 2));
    }
    else
    {
      /* SV accuracy; meters*/
      dst_eph.subframe1.SVacc = (float)pow(2, (eph->accuracy - 2));
    }

    gnss_Sd_Nm_AddEph(GPS_MODE, eph->svid, &dst_eph);
  }
  
  return;
}


const char* asg_log_type_name[] = {
    "SYS",
    "API",
    "RE",
    "RTCM",
    "KF",
    "LS",
    "SD",
    "TM",
    "PE",
    "QOS",
    "TEST",
    "DE",
    "DATA",
    "AL",
    "RTK",
    "COMMON",
    "ALGO",
    "FE",
    "ADR",
    "AHRS",
    "CA",
    "FLP",
    "IMU",
    "INS"
};

void gnss_log_print(int level, int line, Gnss_Module_t tag, const char* format, ...)
{
  static char buffer[512] = {0};
  int n = 0;
  va_list vArgList;
  
  if (level < LOG_INFO)
  {
    return;
  }
  
  if (tag != OBJ_SYS)
  {
    //return;
  }
  memset(buffer, 0, sizeof(buffer));
  line = 1;
  n += sprintf(buffer, "%4d[%-5s] ", line, asg_log_type_name[tag]);
  va_start(vArgList, format);
  n += vsnprintf(buffer+n, sizeof(buffer)-n-1, format, vArgList);
  va_end(vArgList);
  
  gnss_engine_report_log(level, buffer, n);
}

uint8_t gnss_engine_cvt_constellation(uint8_t u_GnssMode, uint8_t prn)
{
  uint8_t u_constellation = C_GNSS_NONE;

  switch (u_GnssMode)
  {
  case GPS_MODE:
    if (prn <= 32)
    {
      u_constellation = C_GNSS_GPS;
    }
    else
    {
      u_constellation = C_GNSS_QZS;
    }
    break;
  case GLN_MODE:
    u_constellation = C_GNSS_GLO;
    break;
  case BDS_MODE:
    if (prn > 18)
    {
      u_constellation = C_GNSS_BDS3;
    }
    else
    {
      u_constellation = C_GNSS_BDS2;
    }
    break;
  case GAL_MODE:
    u_constellation = C_GNSS_GAL;
    break;
  default:
    break;
  }

  return u_constellation;
}

void gnss_engine_inject_pvt_solution(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_PVTResult_t* pz_pvt_result)
{
  /* gnss_PositionFix_t */
  pz_pvt_result->z_positionFix = gz_GnssEnginePositionFix;
  pz_SatSigMeasCollect->z_positionFix = gz_GnssEnginePositionFix;

  /* gnss_MeasQos_t */
  pz_pvt_result->u_meas_count = gpz_Meas->measCnt;
  for (uint32_t i = 0; i < gpz_Meas->measCnt; ++i)
  {
    const gnss_meas_t* pz_meas = &gpz_Meas->meas[i];
    pz_pvt_result->pz_meas_qos[i].z_tor = pz_SatSigMeasCollect->z_tor;
    pz_pvt_result->pz_meas_qos[i].u_constellation = gnss_engine_cvt_constellation(pz_meas->gnssMode, pz_meas->prn);
    pz_pvt_result->pz_meas_qos[i].u_svid = pz_meas->prn;
    // GNSS mode  order = RTK v1.0, GRCE
    pz_pvt_result->pz_meas_qos[i].u_signal = gnss_FreqIdx2Signal(pz_meas->freq_index,
                                                                pz_pvt_result->pz_meas_qos[i].u_constellation);
    pz_pvt_result->pz_meas_qos[i].u_freq_idx = pz_meas->freq_index;
    pz_pvt_result->pz_meas_qos[i].u_status = pz_meas->status;
    pz_pvt_result->pz_meas_qos[i].q_cycleSlipCount = pz_meas->cycleSlipCount;
    pz_pvt_result->pz_meas_qos[i].q_measStatusMask = 0;
    pz_pvt_result->pz_meas_qos[i].d_tot = pz_meas->tot;
    pz_pvt_result->pz_meas_qos[i].d_pseudoRange_filter = 0;
    pz_pvt_result->pz_meas_qos[i].d_doppler_filter = pz_meas->range;
    pz_pvt_result->pz_meas_qos[i].f_pr_residual_pre = pz_meas->pr_diff;
    pz_pvt_result->pz_meas_qos[i].f_pr_residual_post = pz_meas->post_prdiff;
    pz_pvt_result->pz_meas_qos[i].f_dr_residual_pre = pz_meas->dr_diff;
    pz_pvt_result->pz_meas_qos[i].f_dr_residual_post = pz_meas->post_drdiff;
    pz_pvt_result->pz_meas_qos[i].d_pr_unc = pz_meas->prNoise;
    pz_pvt_result->pz_meas_qos[i].d_dr_unc = pz_meas->drNoise;
    pz_pvt_result->pz_meas_qos[i].w_pr_mask = pz_meas->isPrChiSqTest;
    pz_pvt_result->pz_meas_qos[i].w_dr_mask = pz_meas->isDrChiSqTest;
    pz_pvt_result->pz_meas_qos[i].q_pr_deweight_mask = 0;
    pz_pvt_result->pz_meas_qos[i].q_dr_deweight_mask = 0;
    pz_pvt_result->pz_meas_qos[i].u_pr_deweight_cnt = 0;
    pz_pvt_result->pz_meas_qos[i].u_dr_deweight_cnt = 0;
  }
}

void gnss_engine_nmea_creat(gnss_PositionFix_t* pz_position, gnss_NmeaInfo_t* pz_nema_info)
{
  USER_PVT z_user_pvt = {0};
  if((NULL == pz_position)||(NULL == pz_nema_info)){
	  return;
  }

  /* pos vel status*/
  for (int8_t i = 0; i < 3; ++i)
  {
    z_user_pvt.lla.pos[i] = pz_position->d_lla[i]; // rad
    z_user_pvt.lla.vel[i] = pz_position->f_velEnu[i]; // rad
  }
  z_user_pvt.usedSvNum = pz_position->z_SvStatus.u_SvInUseCount;
  z_user_pvt.diff_age = pz_position->z_rtk.f_age;
  switch (pz_position->u_fixFlag)
  {
  case GNSS_FIX_FLAG_FLOATING:
    z_user_pvt.diff_status = DIFF_SOL_STATUS_RTK_FLOAT;
    break;
  case GNSS_FIX_FLAG_FIXED:
    z_user_pvt.diff_status = DIFF_SOL_STATUS_RTK_FIX;
    break;
  case GNSS_FIX_FLAG_SPS:
    z_user_pvt.diff_status = DIFF_SOL_STATUS_STANDALONE;
    break;
  }
  

  gnss_hsm_lite_nmea_create(&z_user_pvt, pz_nema_info);

}