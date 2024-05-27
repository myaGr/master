/**@file        sm_task.cpp
 * @brief       Session Manager(SM) task source file
 * @version     V0.2
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>caizhijie   <td>Init version
 * <tr><td>2022/05/22  <td>0.2      <td>caizhijie   <td>Add configuration and twin meas
 * </table>
 *
 **********************************************************************************
 */

#include "cmn_utils.h"
#include "gnss_common.h"
#include "mw_alloc.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "sm_task.h"
#include "sm_api.h"
#include "sm_nmea.h"
#include "rtcm_dec.h"
#include "sd_api.h"
#include "sd_task.h"
#include "hpp_api.h"
#include "dcp_api.h"
#include "ppp_api.h"
#include "vdr_api.h"
#include "ort_api.h"
#include "loc_core_api.h"
#include "loc_core_cfg.h"
#include "loc_core_report.h"
#include "qxsi_ids_ssr2los_interface.h"
#include "service_ssr_cvt.h"

/** SM module controller structure */
typedef struct {
  /** SM status flag */
  uint8_t b_init     : 1;
  uint8_t b_reserved : 7;

  /** RTCM decoder for receiver raw rtcm */
  RtcmDecoder_t* pz_smRcvRtcmDecoder;

  /** RTCM decoder for reference correct rtcm */
  RtcmDecoder_t* pz_smRefRtcmDecoder;

  /** RTCM decoder for receiver twin raw rtcm */
  RtcmDecoder_t* pz_smRcvTwinRtcmDecoder;

  /** Store Current GNSS Meas Block*/
  GnssMeasBlock_t* pz_CurrentGnssMeasBlock;

  /** Store LLI flag between two epoch */
  LliAddUp_t* pz_lliAddup;

  /** Store GNSS Position Fix reports */
  gnss_PositionFix_t* pz_PositionFixs[TASK_INDEX_MAX];

} sm_TaskController_t;

static sm_TaskController_t gz_sm_task_ctrl = { 0 };

static loc_ConfigParamGroup_t* gpz_SmConfigParamGroup = NULL;

/**
* @brief Convert internal constellation type to loc engine constellation type
* @param[in]   gnss_FreqType - Input constellation type
* @return      loc_api_gnssFreqType - Output loc constellation type
*/
static loc_api_gnssConstellationType sm_ConvertConstellationTypeToLoc(gnss_ConstellationType u_ConstellationType)
{
  loc_api_gnssConstellationType u_type = LOC_API_GNSS_MAX;

  switch (u_ConstellationType)
  {
  case C_GNSS_GPS:
    u_type = LOC_API_GNSS_GPS;
    break;
  case C_GNSS_GLO:
    u_type = LOC_API_GNSS_GLO;
    break;
  case C_GNSS_BDS2:
  case C_GNSS_BDS3:
    u_type = LOC_API_GNSS_BDS;
    break;
  case C_GNSS_GAL:
    u_type = LOC_API_GNSS_GAL;
    break;
  case C_GNSS_QZS:
    u_type = LOC_API_GNSS_QZS;
    break;
  default:
    u_type = LOC_API_GNSS_MAX;
    break;
  }
  return  u_type;
}

/**
 * @brief SM module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_Init(ipc_t* p_ipc)
{ 
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  if (TRUE == gz_sm_task_ctrl.b_init)
  {
    return;
  }
  gz_sm_task_ctrl.b_init = TRUE;

  gpz_SmConfigParamGroup = loc_cfg_getConfigParamGroup();
  /* receiver raw rtcm stream decoder */
  if (gpz_SmConfigParamGroup->sm_cfg.b_enable_rcv_rtcm_dec)
  {
    gz_sm_task_ctrl.pz_smRcvRtcmDecoder = (RtcmDecoder_t*)OS_MALLOC(sizeof(RtcmDecoder_t));
    if (NULL != gz_sm_task_ctrl.pz_smRcvRtcmDecoder)
    {
      memset(gz_sm_task_ctrl.pz_smRcvRtcmDecoder, 0, sizeof(RtcmDecoder_t));
    }
  }
  else
  {
    gz_sm_task_ctrl.pz_smRcvRtcmDecoder = NULL;
  }
  
  /* reference raw correction rtcm stream decoder */
  if (gpz_SmConfigParamGroup->sm_cfg.b_enable_ref_rtcm_dec)
  {
    gz_sm_task_ctrl.pz_smRefRtcmDecoder = (RtcmDecoder_t*)OS_MALLOC(sizeof(RtcmDecoder_t));
    memset(gz_sm_task_ctrl.pz_smRefRtcmDecoder, 0, sizeof(RtcmDecoder_t));
  }
  else
  {
    gz_sm_task_ctrl.pz_smRefRtcmDecoder = NULL;
  }
  
  /* receiver twin raw rtcm stream decoder */
  if (gpz_SmConfigParamGroup->sm_cfg.b_enable_rcv_twin_rtcm_dec)
  {
    gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder = (RtcmDecoder_t*)OS_MALLOC(sizeof(RtcmDecoder_t));
    memset(gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder, 0, sizeof(RtcmDecoder_t));
  }
  else
  {
    gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder = NULL;
  }

  if (gpz_SmConfigParamGroup->sm_cfg.b_enable_pos_and_meas_report)
  {
    gz_sm_task_ctrl.pz_CurrentGnssMeasBlock = (GnssMeasBlock_t*)OS_MALLOC(sizeof(GnssMeasBlock_t));
    memset(gz_sm_task_ctrl.pz_CurrentGnssMeasBlock, 0, sizeof(GnssMeasBlock_t));
  }
  else
  {
    gz_sm_task_ctrl.pz_CurrentGnssMeasBlock = NULL;
  }

  gz_sm_task_ctrl.pz_lliAddup = (LliAddUp_t*)OS_MALLOC(sizeof(LliAddUp_t));
  memset(gz_sm_task_ctrl.pz_lliAddup, 0, sizeof(LliAddUp_t));

  gz_sm_task_ctrl.pz_PositionFixs[TASK_INDEX_HPP] = (gnss_PositionFix_t*)OS_MALLOC(sizeof(gnss_PositionFix_t));
  gz_sm_task_ctrl.pz_PositionFixs[TASK_INDEX_PPP] = (gnss_PositionFix_t*)OS_MALLOC(sizeof(gnss_PositionFix_t));

  return;
}

/**
 * @brief SM module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_Start(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  return;
}

/**
 * @brief SM module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_Stop(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  return;
}

/**
 * @brief SM module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_Release(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  if (FALSE == gz_sm_task_ctrl.b_init)
  {
    return;
  }
  gz_sm_task_ctrl.b_init = FALSE;

  if (gz_sm_task_ctrl.pz_smRcvRtcmDecoder)
  {
    OS_FREE(gz_sm_task_ctrl.pz_smRcvRtcmDecoder);
    gz_sm_task_ctrl.pz_smRcvRtcmDecoder = NULL;
  }

  if (gz_sm_task_ctrl.pz_smRefRtcmDecoder)
  {
    OS_FREE(gz_sm_task_ctrl.pz_smRefRtcmDecoder);
    gz_sm_task_ctrl.pz_smRefRtcmDecoder = NULL;
  }

  if (gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder)
  {
    OS_FREE(gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder);
    gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder = NULL;
  }

  if (gz_sm_task_ctrl.pz_CurrentGnssMeasBlock)
  {
    OS_FREE(gz_sm_task_ctrl.pz_CurrentGnssMeasBlock);
    gz_sm_task_ctrl.pz_CurrentGnssMeasBlock = NULL;
  }

  if (gz_sm_task_ctrl.pz_lliAddup)
  {
    OS_FREE(gz_sm_task_ctrl.pz_lliAddup);
    gz_sm_task_ctrl.pz_lliAddup = NULL;
  }
  for (uint8_t i = 0; i < TASK_INDEX_MAX; ++i)
  {
    OS_FREE(gz_sm_task_ctrl.pz_PositionFixs[i]);
  }
  return;
}

/**
 * @brief SM module inject receiver measurement data
 * @param[in]   p_ipc - IPC with receiver measurement data in RTCM
 * @return      None
 */
void sm_RcvMeasRTCM_Put(ipc_t* p_ipc)
{
  int8_t ret = 0;
  uint32_t i = 0;

  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  //BOOL is_e2e_data = FALSE;

  LOGD(TAG_SM, "SM receive rover rtcm data length %d\n", p_ipc->q_length);

  if ((FALSE == pz_loc_ConfigParamGroup->sm_cfg.b_enable_rcv_rtcm_dec) ||
    (NULL == gz_sm_task_ctrl.pz_smRcvRtcmDecoder)|| p_ipc->q_length<=0)
  {
    return;
  }

  // Combined data of previous stored and ipc
  uint32_t length = 0;
  uint8_t* buffer = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (gz_sm_task_ctrl.pz_smRcvRtcmDecoder->e2emsglen + p_ipc->q_length + 1));
  if (!rtcmE2eSkip(gz_sm_task_ctrl.pz_smRcvRtcmDecoder, p_ipc->p_data, p_ipc->q_length, buffer, &length))
  {
    OS_FREE(buffer);
    return;
  }
  
  // decode running
  for (i = 0; i < length; i++)
  {
    LOGD(TAG_SM, "input_rtcm3: i=%d data=%02x\n", i, buffer[i]);
    ret = rtcm_dec_input(gz_sm_task_ctrl.pz_smRcvRtcmDecoder, buffer[i]);
    if (ret <= 0)
    {
      continue;
    }

    LOGD(TAG_SM, "Rtcm Decode Succ t=%d id=%d len=%d sync=%d\n",
      gz_sm_task_ctrl.pz_smRcvRtcmDecoder->q_towMsec,
      gz_sm_task_ctrl.pz_smRcvRtcmDecoder->type_id,
      gz_sm_task_ctrl.pz_smRcvRtcmDecoder->length,
      gz_sm_task_ctrl.pz_smRcvRtcmDecoder->is_complete);

    if ((1 == ret) && (FALSE == gpz_SmConfigParamGroup->sm_cfg.b_disable_decode_rtcm_obs))
    {
      if (gz_sm_task_ctrl.pz_smRcvRtcmDecoder->is_complete)
      {
        GnssMeasBlock_t z_MeasBlock = { 0 };
        rtcm_ExtractMeasBlk(gz_sm_task_ctrl.pz_smRcvRtcmDecoder, &z_MeasBlock);
        LOGW(TAG_SM, "Inject Rcv Meas t=%d count=%d\n",
          z_MeasBlock.z_Clock.z_gpsTime.q_towMsec,
          z_MeasBlock.w_measCount);
        sm_api_RcvMeasBlk_Put(&z_MeasBlock);
      }
    }

    if ((2 == ret) && ((FALSE == gpz_SmConfigParamGroup->sm_cfg.b_disable_decode_rtcm_nav)))
    {
      switch (gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.u_constellation)
      {
      case C_GNSS_GPS:
        LOGD(TAG_SM, "Inject Rcv GPS Eph Svid=%2d t=%d toe=%f\n",
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_gpsEph.svid,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.z_toeOfGpst.q_towMsec,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_gpsEph.toe);
        break;
      case C_GNSS_GLO:
        break;
      case C_GNSS_BDS3:
        LOGD(TAG_SM, "Inject Rcv BDS Eph Svid=%2d t=%d toe=%f\n",
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_bdsEph.svid,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.z_toeOfGpst.q_towMsec,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_bdsEph.toe);
        break;
      case C_GNSS_GAL:
        LOGD(TAG_SM, "Inject Rcv GAL Eph Svid=%2d t=%d toe=%f\n",
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_galEph.svid,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.z_toeOfGpst.q_towMsec,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_galEph.toe);
        break;
      case C_GNSS_QZS:
        LOGD(TAG_SM, "Inject Rcv QZS Eph Svid=%2d t=%d toe=%f\n",
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_qzsEph.svid,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.z_toeOfGpst.q_towMsec,
          gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris.eph.z_qzsEph.toe);
        break;
      default:
        break;
      }

      hpp_api_Ephemeris_Put(&gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris);
      sd_api_Ephemeris_Put(&gz_sm_task_ctrl.pz_smRcvRtcmDecoder->ephemeris);
    }
  }

  OS_FREE(buffer);
  return;
}

/**
 * @brief SM Put reference correction
 * @param[in]   p_ipc - IPC with reference correction data in RTCM
 * @return      None
 */
void sm_RefCorrRTCM_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();

  LOGI(TAG_SM, "Receive base RTCM data length %d\n", p_ipc->q_length);

  if ((FALSE == pz_loc_ConfigParamGroup->sm_cfg.b_enable_ref_rtcm_dec) ||
    (NULL == gz_sm_task_ctrl.pz_smRefRtcmDecoder))
  {
    return;
  }

  int8_t ret = 0;
  // Combined data of previous stored and ipc
  uint32_t length = 0;
  uint8_t* buffer = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (gz_sm_task_ctrl.pz_smRefRtcmDecoder->e2emsglen + p_ipc->q_length + 1));
  if (!rtcmE2eSkip(gz_sm_task_ctrl.pz_smRefRtcmDecoder, p_ipc->p_data, p_ipc->q_length, buffer, &length))
  {
    OS_FREE(buffer);
    return;
  }
  for (uint32_t i = 0; i < length; i++)
  {
    ret = rtcm_dec_input(gz_sm_task_ctrl.pz_smRefRtcmDecoder, buffer[i]);
    if (ret <= 0)
    {
      continue;
    }

    if (1 == ret)
    {
      if (gz_sm_task_ctrl.pz_smRefRtcmDecoder->is_complete)
      {
        GnssCorrBlock_t z_CorrectBlock = { 0 };
        rtcm_ExtractCorrBlk(gz_sm_task_ctrl.pz_smRefRtcmDecoder, &z_CorrectBlock);
        LOGI(TAG_SM, "Inject Ref Meas %d towMsec %d ms\n", z_CorrectBlock.w_measCount,
          z_CorrectBlock.z_Clock.z_gpsTime.q_towMsec);
        hpp_api_RefCorrBlk_Put(&z_CorrectBlock);
      }
    }
  }
  OS_FREE(buffer);
  return;
}

/**
 * @brief SM Put reference correction measurement block
 * @param[in]   p_ipc - IPC with reference correction data
 * @return      None
 */
void sm_RefCorrMeasBlock_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  uint8_t* buffer = p_ipc->p_data;
  //uint32_t length = p_ipc->q_length;

  GnssCorrBlock_t* pz_CorrectBlock = (GnssCorrBlock_t*)buffer;
  hpp_api_RefCorrBlk_Put(pz_CorrectBlock);

  return;
}

/**
 * @brief SM report position and measurement data
 * @param[in]   pz_GnssMeasBlock - Gnss Measurement Block
 * @param[in]   pz_PositionFix   - Gnss Position Fix
 * @return      None
 */
static void sm_ReportPositionAndMeas(GnssMeasBlock_t* pz_GnssMeasBlock, gnss_PositionFix_t* pz_PositionFix)
{
  if ((NULL == pz_GnssMeasBlock) || (NULL == pz_PositionFix))
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  uint16_t w_MeasCount = pz_GnssMeasBlock->w_measCount;
  uint32_t q_SizeOfStationMeasureReport = sizeof(gnss_CombinedPositionMeasureReport_t) + w_MeasCount * sizeof(GnssMeasReport_t);
  gnss_CombinedPositionMeasureReport_t* pz_AggregativePositionMeasureReport = 
    (gnss_CombinedPositionMeasureReport_t*)OS_MALLOC(q_SizeOfStationMeasureReport);

  if (NULL == pz_AggregativePositionMeasureReport)
  {
    return;
  }
  memset(pz_AggregativePositionMeasureReport, 0, q_SizeOfStationMeasureReport);

  pz_AggregativePositionMeasureReport->u_Version = VERSION_GNSS_SAT_MEAS_REPORT;
  pz_AggregativePositionMeasureReport->w_Size = q_SizeOfStationMeasureReport;
  pz_AggregativePositionMeasureReport->w_GpsWeek = pz_PositionFix->z_gpsTime.w_week;
  pz_AggregativePositionMeasureReport->q_GpsTowMsec = pz_PositionFix->z_gpsTime.q_towMsec;
  pz_AggregativePositionMeasureReport->u_FixStatus = pz_PositionFix->u_fixFlag;
  for (int i = 0; i < 3; i++)
  {
    pz_AggregativePositionMeasureReport->d_PosLla[i] = pz_PositionFix->d_lla[i];
    pz_AggregativePositionMeasureReport->f_VelEnu[i] = pz_PositionFix->f_velEnu[i];
    pz_AggregativePositionMeasureReport->f_PosXyzUnc[i] = pz_PositionFix->f_posXyzUnc[i];
    pz_AggregativePositionMeasureReport->f_VelEnuUnc[i] = pz_PositionFix->f_velEnuUnc[i];
  }
  
  pz_AggregativePositionMeasureReport->w_MeasCount = w_MeasCount;
  for (int i = 0; i < w_MeasCount; i++)
  {
    GnssMeas_t* pz_Src = &(pz_GnssMeasBlock->z_meas[i]);
    GnssMeasReport_t* pz_Dst = &(pz_AggregativePositionMeasureReport->z_Meas[i]);
    pz_Dst->u_constellation   = sm_ConvertConstellationTypeToLoc(pz_Src->u_constellation);
    pz_Dst->u_svid            = pz_Src->u_svid;
    pz_Dst->u_signal          = pz_Src->u_signal;
    pz_Dst->u_LLI             = pz_Src->u_LLI;
    pz_Dst->f_cn0             = pz_Src->f_cn0;
    pz_Dst->d_pseudoRange     = pz_Src->d_pseudoRange;
    pz_Dst->d_doppler         = pz_Src->d_doppler;
    pz_Dst->d_carrierPhase    = pz_Src->d_carrierPhase;
  }

  loc_report_CombinedPositionMeasure(pz_AggregativePositionMeasureReport);
  OS_FREE(pz_AggregativePositionMeasureReport);

  return;
}
/**
 * @brief calculate the track angle in strcut loc_api_location_report_t
 * @param[out] z_api_location_report location info for report
 */
static void sm_calTrackAngle(loc_api_location_report_t* pz_api_location_report)
{
  if (NULL == pz_api_location_report)
  {    
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  double d_ve2 = 0.0;
  double d_vn2 = 0.0;
  double d_vh4 = 0.0;
  double d_bearing = 0.0;
  double d_trackAngleStd = 0.0;
  d_ve2 = (double)(pz_api_location_report->vel_e) * (pz_api_location_report->vel_e);
  d_vn2 = (double)(pz_api_location_report->vel_n) * (pz_api_location_report->vel_n);
  d_vh4= (d_ve2 + d_vn2) * (d_ve2 + d_vn2);
  if (d_ve2 > 1e-10 || d_vn2 > 1e-10)
  {
    d_bearing = atan2(pz_api_location_report->vel_e, pz_api_location_report->vel_n) * RAD2DEG;
    if (d_bearing < 0.0)
    {
      d_bearing += 360.0;
    }
    pz_api_location_report->track_angle = (float)d_bearing;
    d_trackAngleStd = sqrt(d_vn2 / d_vh4 * pz_api_location_report->vel_e_std * pz_api_location_report->vel_e_std +
      d_ve2 / d_vh4 * pz_api_location_report->vel_n_std * pz_api_location_report->vel_n_std) * RAD2DEG;
    if (d_trackAngleStd > 179.9)
    {
      d_trackAngleStd = 179.9;
    }
    else if (d_trackAngleStd < 1.0)
    {
      d_trackAngleStd = 1.0;
    }
    pz_api_location_report->track_angle_std = (float)d_trackAngleStd;
  }
  else
  {
    pz_api_location_report->track_angle = 0.0f;
    pz_api_location_report->track_angle_std= 179.9f;
  }
  return;
}
/**
 * @brief make loc_api_location_report_t
 * @param[in] pz_PositionFix_report position information from algorithm
 * @param[out] z_api_location_report location info for report
 */
static void sm_CreateLocationReportInfo(const gnss_PositionFix_t* pz_PositionFix_report, 
  loc_api_location_report_t* pz_api_location_report)
{
  if ((NULL == pz_PositionFix_report) || (NULL == pz_api_location_report))
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  if (any_Ptrs_Null(2, pz_PositionFix_report, pz_api_location_report))
  {
    return;
  }
  pz_api_location_report->utc_timestamp = tm_cvt_FullMsecToUnixMsec(pz_PositionFix_report->z_gpsTime.t_fullMsec);
  pz_api_location_report->utc_year      = pz_PositionFix_report->z_epoch.year;
  pz_api_location_report->utc_month     = pz_PositionFix_report->z_epoch.month;
  pz_api_location_report->utc_day       = pz_PositionFix_report->z_epoch.day;
  pz_api_location_report->utc_hour      = pz_PositionFix_report->z_epoch.hour;
  pz_api_location_report->utc_minute    = pz_PositionFix_report->z_epoch.min;
  pz_api_location_report->utc_second    = pz_PositionFix_report->z_epoch.second;
  pz_api_location_report->week          = pz_PositionFix_report->z_gpsTime.w_week;
  pz_api_location_report->tow           = (double)pz_PositionFix_report->z_gpsTime.q_towMsec * TIME_MSEC_INV;
  pz_api_location_report->leapsec       = pz_PositionFix_report->u_leapsec;
  pz_api_location_report->fix_quality   = pz_PositionFix_report->u_fixFlag;

  double lat = pz_PositionFix_report->d_lla[0] * RAD2DEG;
  double lon = pz_PositionFix_report->d_lla[1] * RAD2DEG;

  if (lat > 180.0) {
    lat -= 180.0;
  }
  else if (lat < -180.0) {
    lat += 180.0;
  }

  if (lon > 180) {
    lon -= 180.0;
  }
  else if (lon < -180.0) {
    lon += 180.0;
  }
  pz_api_location_report->lat       = lat;
  pz_api_location_report->lon       = lon;
  pz_api_location_report->alt       = (float) pz_PositionFix_report->d_lla[2];
  pz_api_location_report->latstd    = pz_PositionFix_report->f_posLlaUnc[0];
  pz_api_location_report->lonstd    = pz_PositionFix_report->f_posLlaUnc[1];
  pz_api_location_report->altstd    = pz_PositionFix_report->f_posLlaUnc[2];
  pz_api_location_report->lat_protection_level    = pz_PositionFix_report->z_integrity.z_pos_north_pl.f_protection_level;
  pz_api_location_report->lon_protection_level    = pz_PositionFix_report->z_integrity.z_pos_east_pl.f_protection_level;
  pz_api_location_report->alt_protection_level    = pz_PositionFix_report->z_integrity.z_pos_ver_pl.f_protection_level;
  pz_api_location_report->lat_integrity_flag    = pz_PositionFix_report->z_integrity.z_pos_north_pl.u_flag;
  pz_api_location_report->lon_integrity_flag    = pz_PositionFix_report->z_integrity.z_pos_east_pl.u_flag;
  pz_api_location_report->alt_integrity_flag    = pz_PositionFix_report->z_integrity.z_pos_ver_pl.u_flag;
  pz_api_location_report->vel_e     = pz_PositionFix_report->f_velEnu[0];
  pz_api_location_report->vel_n     = pz_PositionFix_report->f_velEnu[1];
  pz_api_location_report->vel_d     = -pz_PositionFix_report->f_velEnu[2];
  pz_api_location_report->vel_e_std = pz_PositionFix_report->f_velEnuUnc[0];
  pz_api_location_report->vel_n_std = pz_PositionFix_report->f_velEnuUnc[1];
  pz_api_location_report->vel_d_std = pz_PositionFix_report->f_velEnuUnc[2];
  pz_api_location_report->age       = pz_PositionFix_report->z_rtk.f_age;
  pz_api_location_report->sv_used   = pz_PositionFix_report->z_SvStatus.u_SvInUseCount;
  pz_api_location_report->sv_trked  = pz_PositionFix_report->z_SvStatus.u_SvTrackCount;
  pz_api_location_report->sv_fixed  = 0;// pz_PositionFix_report->z_SvStatus.u_SvFixedCount
  pz_api_location_report->gdop      = pz_PositionFix_report->z_dops.f_gdop;
  pz_api_location_report->pdop      = pz_PositionFix_report->z_dops.f_pdop;
  pz_api_location_report->hdop      = pz_PositionFix_report->z_dops.f_hdop;
  pz_api_location_report->tdop      = pz_PositionFix_report->z_dops.f_tdop;
  pz_api_location_report->quasi_geoid_h = pz_PositionFix_report->d_quasiGeoidHeight;
  pz_api_location_report->avg_CN0       = pz_PositionFix_report->d_avgCN0;
  pz_api_location_report->CN040         = pz_PositionFix_report->u_CN040;
  pz_api_location_report->dcp_pos_type  = pz_PositionFix_report->u_DcpPosType;
  sm_calTrackAngle(pz_api_location_report);
//  memcpy(&pz_api_location_report->z_MeasStatus, &pz_PositionFix_report->z_SvStatus, sizeof(gnss_PositionFix_SvStatus_t));

  return;
}

/**
 * @brief make loc_api_orient_report_t
 * @param[in] pz_OrientFix_report is orient information from algorithm
 * @param[out] pz_api_orient_report location info for report
 */
static void sm_CreateOrientReportInfo(const gnss_OrientFix_t* pz_OrientFix_report, loc_api_orient_report_t* pz_api_orient_report)
{
  uint8_t u_i = 0;
  double pd_auxiAntEeefPos[3] = { 0.0 };
  if ((NULL == pz_OrientFix_report) || (NULL == pz_api_orient_report))
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }
  pz_api_orient_report->t_utcTimeStamp = tm_cvt_FullMsecToUnixMsec(pz_OrientFix_report->z_gpsTime.t_fullMsec);
  pz_api_orient_report->w_utcYear = pz_OrientFix_report->z_epoch.year;
  pz_api_orient_report->w_utcMonth = pz_OrientFix_report->z_epoch.month;
  pz_api_orient_report->w_utcDay = pz_OrientFix_report->z_epoch.day;
  pz_api_orient_report->w_utcHour = pz_OrientFix_report->z_epoch.hour;
  pz_api_orient_report->w_utcMinute = pz_OrientFix_report->z_epoch.min;
  pz_api_orient_report->d_utcSecond = pz_OrientFix_report->z_epoch.second;
  pz_api_orient_report->w_week = pz_OrientFix_report->z_gpsTime.w_week;
  pz_api_orient_report->d_tow = (double)pz_OrientFix_report->z_gpsTime.q_towMsec * TIME_MSEC_INV;
  pz_api_orient_report->u_leapsec = pz_OrientFix_report->u_leapsec;
  pz_api_orient_report->u_ortFixQuality = pz_OrientFix_report->u_ortFixFlag;
  pz_api_orient_report->u_SvTrackCount = pz_OrientFix_report->u_SvTrackCount;
  pz_api_orient_report->u_SvTrackCountEleThres= pz_OrientFix_report->u_SvTrackCountEleThres;
  pz_api_orient_report->u_SvInUseCount = pz_OrientFix_report->u_SvInUseCount;
  pz_api_orient_report->u_SvInUseCountMuliFreq = pz_OrientFix_report->u_SvInUseCountMuliFreq;
  pz_api_orient_report->u_ortSolStatus = pz_OrientFix_report->u_ortSolStatus;
  pz_api_orient_report->u_ortPosVelType = pz_OrientFix_report->u_ortPosVelType;
  pz_api_orient_report->u_GPSsignalUsedMask = pz_OrientFix_report->u_GPSsignalUsedMask;
  pz_api_orient_report->u_GLOsignalUsedMask = pz_OrientFix_report->u_GLOsignalUsedMask;
  pz_api_orient_report->u_GalSignalUsedMask = pz_OrientFix_report->u_GalSignalUsedMask;
  pz_api_orient_report->u_BDSsignalUsedMask = pz_OrientFix_report->u_BDSsignalUsedMask;
  pz_api_orient_report->f_age = pz_OrientFix_report->f_age;
  pz_api_orient_report->f_heading = pz_OrientFix_report->z_ortResult.f_heading;
  pz_api_orient_report->f_pitch = pz_OrientFix_report->z_ortResult.f_pitch;
  pz_api_orient_report->f_roll = pz_OrientFix_report->z_ortResult.f_roll;
  pz_api_orient_report->f_headingStd = pz_OrientFix_report->z_ortResult.f_headingStd;
  pz_api_orient_report->f_pitchStd = pz_OrientFix_report->z_ortResult.f_pitchStd;
  pz_api_orient_report->f_rollStd = pz_OrientFix_report->z_ortResult.f_rollStd;
  pz_api_orient_report->u_posFixQuality = pz_OrientFix_report->z_mainAntInfo.u_posFixFlag;
  pz_api_orient_report->u_mainAntSvInUseCount = pz_OrientFix_report->z_mainAntInfo.u_SvInUseCount;
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_api_orient_report->f_ENU[u_i] = pz_OrientFix_report->z_ortResult.f_ENU[u_i];
    pz_api_orient_report->f_auxiAntVelXyz[u_i] = pz_OrientFix_report->z_ortResult.f_velXyz[u_i];
    pz_api_orient_report->f_auxiAntVelEnu[u_i] = pz_OrientFix_report->z_ortResult.f_velEnu[u_i];
    pz_api_orient_report->d_mainAntXyz[u_i] = pz_OrientFix_report->z_mainAntInfo.d_xyz[u_i];
    if (0 != (pz_api_orient_report->u_posFixQuality) && 0 != (pz_api_orient_report->u_ortFixQuality))
    {
      pd_auxiAntEeefPos[u_i] = pz_OrientFix_report->z_mainAntInfo.d_xyz[u_i] + (double)(pz_OrientFix_report->z_ortResult.f_deltaXYZ[u_i]);
    }
    pz_api_orient_report->d_mainAntLLA[u_i] = pz_OrientFix_report->z_mainAntInfo.d_lla[u_i];
    pz_api_orient_report->f_mainAntVelXyz[u_i] = pz_OrientFix_report->z_mainAntInfo.f_velXyz[u_i];
    pz_api_orient_report->f_mainAntVelEnu[u_i] = pz_OrientFix_report->z_mainAntInfo.f_velEnu[u_i];
    pz_api_orient_report->pd_refStationCoordinate[u_i] = pz_OrientFix_report->z_mainAntInfo.pd_refStationCoordinate[u_i];
  }
  if (0 != (pz_api_orient_report->u_posFixQuality) && 0 != (pz_api_orient_report->u_ortFixQuality))
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_api_orient_report->d_auxiAntXyz[u_i] = pd_auxiAntEeefPos[u_i];
    }
    gnss_Ecef2Lla(pz_api_orient_report->d_auxiAntXyz, pz_api_orient_report->d_auxiAntLLA);
  }
  else
  {
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_api_orient_report->d_auxiAntXyz[u_i] = 0.0;
      pz_api_orient_report->d_auxiAntLLA[u_i] = 0.0;
    }
  }
  return;
}

static void sm_ConvertPostionFixToInsGnssFix(gnss_PositionFix_t* pz_PositionFix, 
  ins_GnssFix_t* pz_InsGnssFix)
{
  if ((NULL == pz_PositionFix) || (NULL == pz_InsGnssFix))
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  pz_InsGnssFix->t_timestamp = pz_PositionFix->z_gpsTime.q_towMsec;
  pz_InsGnssFix->t_timeStampOfUtc = pz_PositionFix->z_gpsTime.q_towMsec;
  pz_InsGnssFix->d_lat = pz_PositionFix->d_lla[0] * RAD2DEG;
  pz_InsGnssFix->d_lon = pz_PositionFix->d_lla[1] * RAD2DEG;
  pz_InsGnssFix->f_alt = (float)pz_PositionFix->d_lla[2];
  pz_InsGnssFix->f_ve = pz_PositionFix->f_velEnu[0];
  pz_InsGnssFix->f_vn = pz_PositionFix->f_velEnu[1];
  pz_InsGnssFix->f_vd = -pz_PositionFix->f_velEnu[2];
  pz_InsGnssFix->f_latstd = pz_PositionFix->f_posLlaUnc[0];
  pz_InsGnssFix->f_lonstd = pz_PositionFix->f_posLlaUnc[1];
  pz_InsGnssFix->f_altstd = pz_PositionFix->f_posLlaUnc[2];
  pz_InsGnssFix->f_vestd = pz_PositionFix->f_velEnuUnc[0];
  pz_InsGnssFix->f_vnstd = pz_PositionFix->f_velEnuUnc[1];
  pz_InsGnssFix->f_vdstd = pz_PositionFix->f_velEnuUnc[2];
  pz_InsGnssFix->f_speed = sqrtf(pz_InsGnssFix->f_vn * pz_InsGnssFix->f_vn + pz_InsGnssFix->f_ve * pz_InsGnssFix->f_ve);
  pz_InsGnssFix->f_heading = atan2f(pz_InsGnssFix->f_ve, pz_InsGnssFix->f_vn) * (float)RAD2DEG;
  pz_InsGnssFix->f_heading = pz_InsGnssFix->f_heading < 0 ? 360 + pz_InsGnssFix->f_heading : pz_InsGnssFix->f_heading;
  pz_InsGnssFix->f_hdop = pz_PositionFix->z_dops.f_hdop;
  pz_InsGnssFix->f_hor_accu = sqrtf(pz_PositionFix->f_posXyzUnc[0] * pz_PositionFix->f_posXyzUnc[0]
    + pz_PositionFix->f_posXyzUnc[1] * pz_PositionFix->f_posXyzUnc[1]
    + pz_PositionFix->f_posXyzUnc[2] * pz_PositionFix->f_posXyzUnc[2]);
  pz_InsGnssFix->d_tow = pz_PositionFix->z_gpsTime.q_towMsec;
  pz_InsGnssFix->f_avgsnr = pz_PositionFix->d_avgCN0;
  pz_InsGnssFix->w_week = pz_PositionFix->z_gpsTime.w_week;
  pz_InsGnssFix->u_postype = pz_PositionFix->u_fixFlag;
  pz_InsGnssFix->u_satused = pz_PositionFix->z_SvStatus.u_SvInUseCount;
  pz_InsGnssFix->z_integrity = pz_PositionFix->z_integrity;
  return;
}

/**
 * @brief Session Manager report nmea
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_PostionFix_Report(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  if (sizeof(gnss_PositionFix_t) != p_ipc->q_length)
  {
    LOGE(TAG_SM, "Ipc 0x08x length %d mismatch payload %d\n",
      p_ipc->q_ipc_id, p_ipc->q_length, sizeof(gnss_PositionFix_t));
    return;
  }

  gnss_PositionFix_t* pz_PositionFix = (gnss_PositionFix_t*)p_ipc->p_data;
  if (pz_PositionFix->u_fixFlag != GNSS_FIX_FLAG_INVALID)
  {
    timepro_set_gpst(&pz_PositionFix->z_gpsTime);
  }

  loc_api_location_report_t z_api_location_report = { 0 };
  sm_CreateLocationReportInfo(pz_PositionFix, &z_api_location_report);

  static loc_api_ConsoildatedPositionFix_t z_ConsoildatePositionReport = {0};

  loc_api_ConsoildatedPositionFix_t* pz_ConsoildatePositionReport = &z_ConsoildatePositionReport;
  
  gnss_cvt_GnssPositionFix_To_LocApiPositionFix(pz_PositionFix, pz_ConsoildatePositionReport);

  //loc_MonitorStructType* pz_ReportMonitor = loc_report_GetMonitor();
  loc_ConfigParamGroup_t* pz_ConfigParamGrp = loc_cfg_getConfigParamGroup();

  /* Handle the position report if VDR module is enabled */
  if (pz_ConfigParamGrp->vdr_cfg.b_enable)
  {
    if (FIX_SOURCE_VDR == pz_PositionFix->u_fixSource)
    {
      loc_core_report_consoildated_location(pz_ConsoildatePositionReport);
    }

    /* If VDR module is enabled, transmit the highest priority GNSS Fix to VDR  */
    if (pz_ConfigParamGrp->dcp_cfg.b_enable)
    {
      if (FIX_SOURCE_DCP == pz_PositionFix->u_fixSource)
      {
        ins_GnssFix_t z_GnssFix = { 0 };
        sm_ConvertPostionFixToInsGnssFix(pz_PositionFix, &z_GnssFix);
        if (GNSS_FIX_FLAG_INVALID != pz_PositionFix->u_fixFlag) 
        {
          vdr_api_GnssFix_Put(&z_GnssFix);
        }
      }
    }
    else if (pz_ConfigParamGrp->ppp_cfg.b_enable)
    {
      if ((FIX_SOURCE_PPP == pz_PositionFix->u_fixSource) ||
        (FIX_SOURCE_PPP_FLOAT == pz_PositionFix->u_fixSource) ||
        (FIX_SOURCE_PPP_FIX == pz_PositionFix->u_fixSource))
      {
        ins_GnssFix_t z_GnssFix = { 0 };
        sm_ConvertPostionFixToInsGnssFix(pz_PositionFix, &z_GnssFix);
        if (GNSS_FIX_FLAG_INVALID != pz_PositionFix->u_fixFlag) 
        {
          vdr_api_GnssFix_Put(&z_GnssFix);
        }
      }
    }
    else if (pz_ConfigParamGrp->hpp_cfg.b_enable)
    {
      if (pz_ConfigParamGrp->hpp_cfg.b_enable_rtk)
      {
        if ((FIX_SOURCE_RTK == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_RTD == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_RTK_FLOAT == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_RTK_FIX == pz_PositionFix->u_fixSource))
        {
          ins_GnssFix_t z_GnssFix = { 0 };
          sm_ConvertPostionFixToInsGnssFix(pz_PositionFix, &z_GnssFix);
          if (GNSS_FIX_FLAG_INVALID != pz_PositionFix->u_fixFlag) 
          {
            vdr_api_GnssFix_Put(&z_GnssFix);
          }
        }
      }
      else if (pz_ConfigParamGrp->hpp_cfg.b_enable_pvt)
      {
        if ((FIX_SOURCE_PVT_WLS == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_PVT_KF == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_HPP_V1 == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_HPP_V2 == pz_PositionFix->u_fixSource))
        {
          ins_GnssFix_t z_GnssFix = { 0 };
          sm_ConvertPostionFixToInsGnssFix(pz_PositionFix, &z_GnssFix);
          if (GNSS_FIX_FLAG_INVALID != pz_PositionFix->u_fixFlag)
          {
            vdr_api_GnssFix_Put(&z_GnssFix);
          }
        }
      }
    }
  }

  /* Handle the GNSS position report */
  if (pz_ConfigParamGrp->dcp_cfg.b_enable)
  {
    if (FIX_SOURCE_DCP == pz_PositionFix->u_fixSource)
    {
      loc_core_report_location(&z_api_location_report);
      loc_core_report_consoildated_location(pz_ConsoildatePositionReport);
    }
  }
  else if (pz_ConfigParamGrp->ppp_cfg.b_enable)
  {
    if ((FIX_SOURCE_PPP == pz_PositionFix->u_fixSource) ||
       (FIX_SOURCE_PPP_FLOAT == pz_PositionFix->u_fixSource) ||
       (FIX_SOURCE_PPP_FIX == pz_PositionFix->u_fixSource))
    {
      loc_core_report_location(&z_api_location_report);
      loc_core_report_consoildated_location(pz_ConsoildatePositionReport);
    }
  }
  else if (pz_ConfigParamGrp->hpp_cfg.b_enable)
  {
    if (pz_ConfigParamGrp->hpp_cfg.b_enable_rtk)
    {
      if ((FIX_SOURCE_RTK == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_RTD == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_RTK_FLOAT == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_RTK_FIX == pz_PositionFix->u_fixSource))
      {
        loc_core_report_location(&z_api_location_report);
        loc_core_report_consoildated_location(pz_ConsoildatePositionReport);
      }
    }
    else if (pz_ConfigParamGrp->hpp_cfg.b_enable_pvt)
    {
      if ((FIX_SOURCE_PVT_WLS == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_PVT_KF == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_HPP_V1 == pz_PositionFix->u_fixSource) ||
          (FIX_SOURCE_HPP_V2 == pz_PositionFix->u_fixSource))
      {
        loc_core_report_location(&z_api_location_report);
        loc_core_report_consoildated_location(pz_ConsoildatePositionReport);
      }
    }
  }

  /* Store the HPP Position Report for SSR service requirment */
  if ((FIX_SOURCE_PVT_WLS == pz_PositionFix->u_fixSource) ||
      (FIX_SOURCE_PVT_KF == pz_PositionFix->u_fixSource) ||
      (FIX_SOURCE_HPP_V1 == pz_PositionFix->u_fixSource) ||
      (FIX_SOURCE_HPP_V2 == pz_PositionFix->u_fixSource))
  {
    if (NULL != gz_sm_task_ctrl.pz_PositionFixs[TASK_INDEX_HPP])
    {
      memcpy(gz_sm_task_ctrl.pz_PositionFixs[TASK_INDEX_HPP], pz_PositionFix,
             sizeof(gnss_PositionFix_t));
    }
  }

  loc_report_MonitorReport();

  return;
}

/**
 * @brief Session Manager report orient result
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_OrientFix_Report(ipc_t* p_ipc)
{
  gnss_OrientFix_t* pz_OrientFix = NULL;
  loc_api_orient_report_t z_api_orient_report = { 0 };
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Orient data is Null pointer");
    return;
  }

  if (sizeof(gnss_OrientFix_t) != p_ipc->q_length)
  {
    LOGE(TAG_SM, "Ipc 0x08x length %d mismatch payload %d\n",
      p_ipc->q_ipc_id, p_ipc->q_length, sizeof(gnss_OrientFix_t));
    return;
  }
  pz_OrientFix = (gnss_OrientFix_t*)p_ipc->p_data;
  if (pz_loc_ConfigParamGroup->dcp_cfg.b_enable && FIX_SOURCE_DCP ==pz_OrientFix->u_fixSource)
  {
    sm_CreateOrientReportInfo(pz_OrientFix, &z_api_orient_report);
    loc_core_report_orient(&z_api_orient_report);
  }
  else if (!(pz_loc_ConfigParamGroup->dcp_cfg.b_enable) && FIX_SOURCE_RTK == pz_OrientFix->u_fixSource)
  {
    sm_CreateOrientReportInfo(pz_OrientFix, &z_api_orient_report);
    loc_core_report_orient(&z_api_orient_report);
  }
  return;
}

/**
 * @brief Session Manager Put SSR Loc Block Data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrLocBlock_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  gnss_ssrLosBlock_t* pz_SsrLocBlk = (gnss_ssrLosBlock_t*)p_ipc->p_data;
  ppp_api_SsrLocBlock_Put(pz_SsrLocBlk);
  return;
}


/**
 * @brief       Clean up pz_lliAddup after BG
 * @return      None
 */
void sm_CleanUpLLI()
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  if (gz_sm_task_ctrl.pz_lliAddup)
  {
    for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
    {
      for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
      {
        gz_sm_task_ctrl.pz_lliAddup->u_LLI[u_i][u_j] = 0;
      }
    }
  }
  return;
}

/**
 * @brief       Add up lli flag between two bg epoch
 * @param[in]   pz_MeasBlock Receiver Measurement Block structure
 * @return      None
 */
void sm_AddUpLLI(GnssMeasBlock_t* pz_MeasBlock)
{
  if (NULL == pz_MeasBlock)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  uint8_t u_i = 0;
  uint8_t u_iFreq = 0;
  uint16_t w_iSat = 0;
  if (gz_sm_task_ctrl.pz_lliAddup)
  {
    for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; u_i++)
    {
      if (pz_MeasBlock->z_meas[u_i].u_LLI)
      {
        w_iSat = gnss_cvt_Svid2SvIndex(pz_MeasBlock->z_meas[u_i].u_svid, pz_MeasBlock->z_meas[u_i].u_constellation);
        u_iFreq = gnss_cvt_Sig2FreqType(pz_MeasBlock->z_meas[u_i].u_signal);
        if (w_iSat < ALL_GNSS_SYS_SV_NUMBER && u_iFreq < MAX_GNSS_SIGNAL_FREQ)
        {
          gz_sm_task_ctrl.pz_lliAddup->u_LLI[w_iSat][u_iFreq] |= pz_MeasBlock->z_meas[u_i].u_LLI;
        }
      }
    }
  }
  return;
}

/**
 * @brief       add lli bewteen two BG epoch to meas
 * @param[in]   pz_MeasBlock Receiver Measurement Block structure
 * @return      None
 */
void sm_UpdateLLI(GnssMeasBlock_t* pz_MeasBlock)
{
  if (NULL == pz_MeasBlock)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  uint8_t u_i = 0;
  uint8_t u_iFreq = 0;
  uint16_t w_iSat = 0;
  if (gz_sm_task_ctrl.pz_lliAddup)
  {
    for (u_i = 0; u_i < MAX_GNSS_TRK_MEAS_NUMBER; u_i++)
    {
      w_iSat = gnss_cvt_Svid2SvIndex(pz_MeasBlock->z_meas[u_i].u_svid, pz_MeasBlock->z_meas[u_i].u_constellation);
      u_iFreq = gnss_cvt_Sig2FreqType(pz_MeasBlock->z_meas[u_i].u_signal);
      if (w_iSat < ALL_GNSS_SYS_SV_NUMBER && u_iFreq < MAX_GNSS_SIGNAL_FREQ)
      {
        if (0 != gz_sm_task_ctrl.pz_lliAddup->u_LLI[w_iSat][u_iFreq])
        {
          pz_MeasBlock->z_meas[u_i].u_LLI |= gz_sm_task_ctrl.pz_lliAddup->u_LLI[w_iSat][u_iFreq];
        }
      }
    }
  }
  return;
}

/**
 * @brief Convert GnssMeasBlockCollect_t to loc_api_GnssMeasBlock_t for report out
 * @param[in] pz_measblockCollect
 * @param[in,out] pz_ApiGnssMeasBlock
 * @return
 */
static BOOL sm_CvtGnssMeasBlockToApiReport(const GnssMeasBlockCollect_t* pz_measblockCollect,
																			 loc_api_GnssMeasBlock_t* pz_ApiGnssMeasBlock)
{
  if ((NULL == pz_measblockCollect) || (NULL == pz_ApiGnssMeasBlock))
  {
    return FALSE;
  }

  pz_ApiGnssMeasBlock->q_towMsec = pz_measblockCollect->z_measBlock.z_Clock.z_gpsTime.q_towMsec;
  pz_ApiGnssMeasBlock->w_week = pz_measblockCollect->z_measBlock.z_Clock.z_gpsTime.w_week;
  pz_ApiGnssMeasBlock->d_lla[0] = pz_measblockCollect->d_lla[0];
  pz_ApiGnssMeasBlock->d_lla[1] = pz_measblockCollect->d_lla[1];
  pz_ApiGnssMeasBlock->d_lla[2] = pz_measblockCollect->d_lla[2];
  gnss_Lla2Ecef(pz_ApiGnssMeasBlock->d_lla, pz_ApiGnssMeasBlock->d_xyz);
  pz_ApiGnssMeasBlock->w_measCount = pz_measblockCollect->z_measBlock.w_measCount;
  if (pz_ApiGnssMeasBlock->w_measCount != pz_measblockCollect->z_measBlock.w_measCount)
  {
    return FALSE;
  }

  for (uint16_t i = 0; i < pz_ApiGnssMeasBlock->w_measCount; i++)
  {
    loc_api_GnssMeas_t* p_dst = &(pz_ApiGnssMeasBlock->z_GnssMeas[i]);
    const GnssMeas_t* p_src = &(pz_measblockCollect->z_measBlock.z_meas[i]);

    p_dst->z_measStatus.b_valid   = (uint16_t)p_src->z_measStatusFlag.b_valid;
    p_dst->z_measStatus.b_prValid = (uint16_t)p_src->z_measStatusFlag.b_prValid;
    p_dst->z_measStatus.b_drValid = (uint16_t)p_src->z_measStatusFlag.b_drValid;
    p_dst->z_measStatus.b_cpValid = (uint16_t)p_src->z_measStatusFlag.b_cpValid;
    p_dst->u_constellation = sm_ConvertConstellationTypeToLoc((gnss_ConstellationType)p_src->u_constellation);
    p_dst->u_svid                 = p_src->u_svid;
    p_dst->u_signal               = p_src->u_signal;
    p_dst->u_LLI                  = p_src->u_LLI;
    p_dst->f_cn0                  = p_src->f_cn0;
    p_dst->d_pseudoRange          = p_src->d_pseudoRange;
    p_dst->f_doppler              = (float)p_src->d_doppler;
    p_dst->d_carrierPhase         = p_src->d_carrierPhase;
  }

  return TRUE;
}

/**
 * @brief Convert GnssMeasSatBlockCollect_t to loc_api_GnssMeasSatBlock_t for report out
 * @param[in] src
 * @param[in,out] dst
 * @return
 */
static BOOL sm_CvtGnssMeasSatBlockToApiReport(const GnssMeasSatBlockCollect_t *pz_measSatBlock,
                                              loc_api_GnssMeasSatBlock_t *pz_apiMeasSatBlock)
{
  if ((NULL == pz_measSatBlock) || (NULL == pz_apiMeasSatBlock))
  {
    return FALSE;
  }

  pz_apiMeasSatBlock->q_towMsec = pz_measSatBlock->z_measBlock.z_Clock.z_gpsTime.q_towMsec;
  pz_apiMeasSatBlock->w_week = pz_measSatBlock->z_measBlock.z_Clock.z_gpsTime.w_week;
  pz_apiMeasSatBlock->d_lla[0] = pz_measSatBlock->d_lla[0];
  pz_apiMeasSatBlock->d_lla[1] = pz_measSatBlock->d_lla[1];
  pz_apiMeasSatBlock->d_lla[2] = pz_measSatBlock->d_lla[2];
  gnss_Lla2Ecef(pz_apiMeasSatBlock->d_lla, pz_apiMeasSatBlock->d_xyz);
  pz_apiMeasSatBlock->w_measCount = pz_measSatBlock->z_measBlock.w_measCount;
  pz_apiMeasSatBlock->u_satCount = pz_measSatBlock->z_measBlock.u_satCount;

  for (uint16_t i = 0; i < pz_apiMeasSatBlock->w_measCount; i++)
  {
    loc_api_GnssMeas_t* p_dst = &(pz_apiMeasSatBlock->z_GnssMeas[i]);
    const GnssMeas_t* p_src = &(pz_measSatBlock->z_measBlock.z_meas[i]);

    p_dst->z_measStatus.b_valid   = (uint16_t)p_src->z_measStatusFlag.b_valid;
    p_dst->z_measStatus.b_prValid = (uint16_t)p_src->z_measStatusFlag.b_prValid;
    p_dst->z_measStatus.b_drValid = (uint16_t)p_src->z_measStatusFlag.b_drValid;
    p_dst->z_measStatus.b_cpValid = (uint16_t)p_src->z_measStatusFlag.b_cpValid;
    p_dst->u_constellation        = sm_ConvertConstellationTypeToLoc(p_src->u_constellation);
    p_dst->u_svid                 = p_src->u_svid;
    p_dst->u_signal               = p_src->u_signal;
    p_dst->u_LLI                  = p_src->u_LLI;
    p_dst->f_cn0                  = p_src->f_cn0;
    p_dst->d_pseudoRange          = p_src->d_pseudoRange;
    p_dst->f_doppler              = (float)p_src->d_doppler;
    p_dst->d_carrierPhase         = p_src->d_carrierPhase;
  }

  for (uint8_t i = 0; i < pz_apiMeasSatBlock->u_satCount; i++)
  {
    const gnss_SatPosVelClk_t* pz_src = &pz_measSatBlock->z_measBlock.z_satPosVelClk[i];
    loc_api_SatPosInfo_t* pz_dst = &pz_apiMeasSatBlock->z_SatPosInfo[i];

    pz_dst->q_towMsec       = pz_src->z_gpsTime.q_towMsec;
    pz_dst->w_week          = pz_src->z_gpsTime.w_week;
    pz_dst->u_valid         = pz_src->u_valid;
    pz_dst->u_constellation = sm_ConvertConstellationTypeToLoc(pz_src->u_constellation);
    pz_dst->u_svid          = pz_src->u_svid;
    pz_dst->f_elevation     = pz_src->f_elevation;
    pz_dst->f_azimuth       = pz_src->f_azimuth;
    pz_dst->q_iode          = pz_src->q_iode;

    memcpy(&pz_dst->d_satPosClk[0], &pz_src->d_satPosClk[0], sizeof(double) * 4);
    memcpy(&pz_dst->d_satVelClk[0], &pz_src->d_satVelClk[0], sizeof(double) * 4);
  
  }
  return TRUE;
}

/**
 * @brief SM Receiver measurement put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_RcvMeas_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  static uint64_t q_preSecondAlignMsec = 0;
  //int32_t ret = 0;
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();

  GnssMeasBlock_t* pz_MeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;

  /* Inject dcp meas */
  dcp_api_RcvMeasBlk_Put(pz_MeasBlock);

  if (gnss_CheckMeasSecondAlign(pz_MeasBlock->z_Clock.z_gpsTime.t_fullMsec,
      &q_preSecondAlignMsec, pz_loc_ConfigParamGroup->sm_cfg.f_meas_update_interval))
  {
    sm_UpdateLLI(pz_MeasBlock);
    ort_api_RcvMeasBlk_Put(pz_MeasBlock);
    hpp_api_RcvMeasBlk_Put(pz_MeasBlock);
    if (gz_sm_task_ctrl.pz_CurrentGnssMeasBlock)
    {
      memcpy(gz_sm_task_ctrl.pz_CurrentGnssMeasBlock, pz_MeasBlock, sizeof(GnssMeasBlock_t));
    }
    sm_CleanUpLLI();
  }
  else
  {
    sm_AddUpLLI(pz_MeasBlock);
  }

  return;
}

/**
 * @brief Session Manager Put SSR Stream Data from QianXun
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrQianXunStream_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  uint8_t* const buffer = p_ipc->p_data;
  uint32_t length = p_ipc->q_length;

  ppp_api_SsrQianXunStream_Put(buffer, length);

  return;
}

/**
 * @brief Session Manager Put SSR Stream Data from GeeSpace
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrGeeSpaceStream_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  //uint8_t* buffer = p_ipc->p_data;
  //uint32_t length = p_ipc->q_length;
  /** TODO: Decode Stream Adapter */

  return;
}

/**
 * @brief Session Manager Put AG SSR Los data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrAgLos_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  uint8_t* const buffer = p_ipc->p_data;
  uint32_t length = p_ipc->q_length;

  ppp_api_SsrAgLos_Put(buffer, length);

  return;
}

/**
 * @brief SM module inject twin receiver measurement data
 * @param[in]   p_ipc - IPC with receiver measurement data in RTCM
 * @return      None
 */
void sm_RcvTwinMeasRTCM_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();

  LOGI(TAG_SM, "Receive twin RTCM data length %d\n", p_ipc->q_length);

  if ((FALSE == pz_loc_ConfigParamGroup->sm_cfg.b_enable_rcv_twin_rtcm_dec) ||
    (NULL == gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder))
  {
    return;
  }

  int8_t ret = 0;
  // Combined data of previous stored and ipc
  uint32_t length = 0;
  uint8_t* buffer = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder->e2emsglen + p_ipc->q_length + 1));
  if (!rtcmE2eSkip(gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder, p_ipc->p_data, p_ipc->q_length, buffer, &length))
  {
    OS_FREE(buffer);
    return;
  }
  for (uint32_t i = 0; i < length; i++)
  {
    ret = rtcm_dec_input(gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder, buffer[i]);
    if (ret <= 0)
    {
      continue;
    }

    if (1 == ret)
    {
      if (gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder->is_complete)
      {
        GnssMeasBlock_t z_TwinMeasBlock = { 0 };
        rtcm_ExtractMeasBlk(gz_sm_task_ctrl.pz_smRcvTwinRtcmDecoder, &z_TwinMeasBlock);
        LOGW(TAG_SM, "Inject Twin Rcv Meas t=%d count=%d\n", z_TwinMeasBlock.z_Clock.z_gpsTime.q_towMsec, z_TwinMeasBlock.w_measCount);
        sm_api_OrtMeasBlk_Put(&z_TwinMeasBlock);
      }
    }
  }
  OS_FREE(buffer);
  return;
}

/**
* @brief Convert internal fix type to loc engine fix type
* @param[in]   u_FixFlagType - Input fix type
* @return      loc_api_FixFlagType - Output loc fix type
*/
static loc_api_FixFlagType sm_ConvertFixFlagTypeToLoc(gnss_FixFlagType u_FixFlagType)
{
  loc_api_FixFlagType u_type = GNSS_FIX_FLAG_INVALID;
  switch (u_FixFlagType)
  {
  case GNSS_FIX_FLAG_INVALID:
    u_type = GNSS_FIX_FLAG_INVALID;
    break;
  case GNSS_FIX_FLAG_SPS:
    u_type = LOC_API_FIX_FLAG_SPS;
    break;
  case GNSS_FIX_FLAG_DGNSS:
    u_type = LOC_API_FIX_FLAG_DGNSS;
    break;
  case GNSS_FIX_FLAG_PPS:
    u_type = LOC_API_FIX_FLAG_PPS;
    break;
  case GNSS_FIX_FLAG_FIXED:
    u_type = LOC_API_FIX_FLAG_FIXED;
    break;
  case GNSS_FIX_FLAG_FLOATING:
    u_type = LOC_API_FIX_FLAG_FLOATING;
    break;
  case GNSS_FIX_FLAG_DR:
    u_type = LOC_API_FIX_FLAG_DR;
    break;
  case GNSS_FIX_FLAG_MANUAL:
    u_type = LOC_API_FIX_FLAG_MANUAL;
    break;
  case GNSS_FIX_FLAG_SIMULATOR:
    u_type = LOC_API_FIX_FLAG_SIMULATOR;
    break;
  default:
    break;
  }
  return u_type;
}

/**
* @brief Convert internal freq type to loc engine freq type
* @param[in]   gnss_FreqType - Input freq type
* @return      loc_api_gnssFreqType - Output loc freq type
*/
static loc_api_gnssFreqType sm_ConvertFreqTypeToLoc(gnss_FreqType u_FreqType)
{
  loc_api_gnssFreqType u_type = LOC_API_GNSS_FREQ_TYPE_INVALID;
  switch (u_FreqType)
  {
  case C_GNSS_FREQ_TYPE_L1:
    u_type = LOC_API_GNSS_FREQ_TYPE_L1;
    break;
  case C_GNSS_FREQ_TYPE_L2:
    u_type = LOC_API_GNSS_FREQ_TYPE_L2;
    break;
  case C_GNSS_FREQ_TYPE_L5:
    u_type = LOC_API_GNSS_FREQ_TYPE_L5;
    break;
  default:
    u_type = LOC_API_GNSS_FREQ_TYPE_INVALID;
    break;
  }

  return u_type;
}

/**
 * @brief SM module report meas block collect
 * @param[in] p_ipc
 * @note
 * Report measurement to extern user for some functions.
	 1. Support SSR adapter to transform ssr to los
 */
static void sm_GnssMeasBlockCollect_Report(ipc_t* p_ipc)
{
	const GnssMeasBlockCollect_t* pz_MeasBlockCollect = (GnssMeasBlockCollect_t*)p_ipc->p_data;

	uint32_t q_size = sizeof(loc_api_GnssMeasBlock_t);
	if (pz_MeasBlockCollect->z_measBlock.w_measCount > 0)
	{
		q_size += (pz_MeasBlockCollect->z_measBlock.w_measCount - 1) * sizeof(loc_api_GnssMeas_t);
	}
	loc_api_GnssMeasBlock_t* pz_ApiGnssMeasBlock = (loc_api_GnssMeasBlock_t*)OS_MALLOC(q_size);
	if (NULL == pz_ApiGnssMeasBlock)
	{
		return;
	}
	if (sm_CvtGnssMeasBlockToApiReport(pz_MeasBlockCollect, pz_ApiGnssMeasBlock))
	{
		loc_core_report_gnss_meas_blk(pz_ApiGnssMeasBlock);
	}
	OS_FREE(pz_ApiGnssMeasBlock);
}

/**
 * @brief SM module report meas and satellite position block collect
 * @param[in] p_ipc
 * @note
 * Report measurement and satellite position to extern user for some functions.
	 1. Support SSR adapter to transform ssr to los
 */
static void sm_GnssMeasSatBlockCollect_Report(ipc_t* p_ipc)
{
	const GnssMeasSatBlockCollect_t* pz_MeasSat = (GnssMeasSatBlockCollect_t*)p_ipc->p_data;

	uint32_t q_size = sizeof(loc_api_GnssMeasSatBlock_t);
	if (pz_MeasSat->z_measBlock.w_measCount > 0)
	{
		q_size += (pz_MeasSat->z_measBlock.w_measCount - 1) * sizeof(loc_api_GnssMeas_t);
	}
	if (pz_MeasSat->z_measBlock.u_satCount > 0)
	{
		q_size += (pz_MeasSat->z_measBlock.u_satCount - 1) * sizeof(loc_api_SatPosInfo_t);
	}
	loc_api_GnssMeasSatBlock_t* pz_ApiGnssMeasSatBlock = (loc_api_GnssMeasSatBlock_t*)OS_MALLOC(q_size);
	if (NULL == pz_ApiGnssMeasSatBlock)
	{
		return;
	}
  pz_ApiGnssMeasSatBlock->u_version = VERSION_API_GNSS_MEAS_SAT_DATA;
	if (sm_CvtGnssMeasSatBlockToApiReport(pz_MeasSat, pz_ApiGnssMeasSatBlock))
	{
      loc_core_report_gnss_meas_sat_blk(pz_ApiGnssMeasSatBlock);
	}
	OS_FREE(pz_ApiGnssMeasSatBlock);
}

/**
 * SM module report navigation data frame
 * @param[in] ipc
 */
static void sm_NavDataFrame_Report(ipc_t* ipc)
{
  const NavigationData_t* pz_navData = (NavigationData_t*)ipc->p_data;

  uint32_t q_size = sizeof(loc_api_NavigationData_t) + pz_navData->length;
  loc_api_NavigationData_t* pz_ApiNavigationData = (loc_api_NavigationData_t*)OS_MALLOC(q_size);

  if (NULL != pz_ApiNavigationData)
  {
    pz_ApiNavigationData->version = VERSION_API_NAVIGATION_DATA;
    pz_ApiNavigationData->tow = pz_navData->d_tow;
    pz_ApiNavigationData->sys = pz_navData->sys;
    pz_ApiNavigationData->svid = pz_navData->svid;
    pz_ApiNavigationData->signal_type = pz_navData->signal_type;
    pz_ApiNavigationData->length_rtcm = pz_navData->length_rtcm;
    memcpy(pz_ApiNavigationData->buffer_rtcm, pz_navData->buffer_rtcm, sizeof(uint8_t) * pz_navData->length_rtcm);
    pz_ApiNavigationData->length = pz_navData->length;
    memcpy(pz_ApiNavigationData->buffer, pz_navData->buffer, sizeof(uint8_t) * pz_navData->length);
    loc_core_report_navigation_data(pz_ApiNavigationData);
    OS_FREE(pz_ApiNavigationData);
  }
}

/**
 * @brief SM module report the information from GNSS to INS
 * @param[in]   p_ipc - IPC with receiver measurement data in RTCM
 * @return      None
 */
static void sm_GnssFeedbackInsMeasBlock_Report(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  if (sizeof(gnss_FeedbackInsMeasBlock_t) != p_ipc->q_length)
  {
    LOGE(TAG_SM, "Ipc 0x08x length %d mismatch payload %d\n",
      p_ipc->q_ipc_id, p_ipc->q_length, sizeof(gnss_FeedbackInsMeasBlock_t));
    return;
  }

  loc_api_GnssFeedbackInsMeasBlock_t z_loc_GnssFeedbackIns = { 0 };
  gnss_FeedbackInsMeasBlock_t* pz_FeedbackInsMeasBlock = (gnss_FeedbackInsMeasBlock_t*)p_ipc->p_data;

  z_loc_GnssFeedbackIns.q_GpsTowMsec = pz_FeedbackInsMeasBlock->z_GpsTime.q_towMsec;
  z_loc_GnssFeedbackIns.w_GpsWeek = pz_FeedbackInsMeasBlock->z_GpsTime.w_week;
  z_loc_GnssFeedbackIns.u_CurFixFlag = sm_ConvertFixFlagTypeToLoc(pz_FeedbackInsMeasBlock->u_CurFixFlag);
  z_loc_GnssFeedbackIns.u_PreFixFlag = sm_ConvertFixFlagTypeToLoc(pz_FeedbackInsMeasBlock->u_PreFixFlag);
  z_loc_GnssFeedbackIns.u_FreqType   = sm_ConvertFreqTypeToLoc(pz_FeedbackInsMeasBlock->u_FreqType);

  for (uint8_t i = 0; i < 3; i++)
  {
    z_loc_GnssFeedbackIns.d_CurLLA[i] = pz_FeedbackInsMeasBlock->d_CurLLA[i];
    z_loc_GnssFeedbackIns.d_PreLLA[i] = pz_FeedbackInsMeasBlock->d_PreLLA[i];
  }

  z_loc_GnssFeedbackIns.f_QuasiGeoidHeight = pz_FeedbackInsMeasBlock->f_QuasiGeoidHeight;
  z_loc_GnssFeedbackIns.f_interval = pz_FeedbackInsMeasBlock->f_interval;
  z_loc_GnssFeedbackIns.u_FeedbackMeasType = pz_FeedbackInsMeasBlock->u_FeedbackMeasType;
  z_loc_GnssFeedbackIns.u_MeasCount = pz_FeedbackInsMeasBlock->u_MeasCount;
  for (uint8_t i = 0; i < z_loc_GnssFeedbackIns.u_MeasCount; i++)
  {
    gnss_FeedbackInsMeasUnit_t* p_i = &pz_FeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[i];
    loc_api_GnssFeedbackInsMeasUnit_t* p_o = &z_loc_GnssFeedbackIns.z_GnssFeedbackInsMeasUnit[i];

    p_o->u_constellation = sm_ConvertConstellationTypeToLoc(p_i->u_constellation);
    p_o->u_svid = p_i->u_svid;
    p_o->u_cn0 = p_i->u_cn0;
    p_o->u_obs_valid = p_i->u_obs_valid;
    p_o->f_elevation = p_i->f_elevation;
    p_o->f_azimuth = p_i->f_azimuth;
    for (uint8_t j = 0; j < 3; j++)
    {
      p_o->f_unit_dir_vect[j] = p_i->f_unit_dir_vect[j];
      p_o->d_sat_pos[j] = p_i->d_sat_pos[j];
      p_o->d_sat_vel[j] = p_i->d_sat_vel[j];
      p_o->d_pre_sat_pos[j] = p_i->d_pre_sat_pos[j];
    }
    p_o->d_pr_var = p_i->d_pr_var;
    p_o->d_dr_var = p_i->d_dr_var;
    p_o->f_epoch_diff_obs = p_i->f_epoch_diff_obs;
    p_o->f_epoch_diff_res = p_i->f_epoch_diff_res;
    p_o->d_pseudorange = p_i->d_pseudorange;
    p_o->f_doppler = p_i->f_doppler;
  }
  vdr_api_GnssInsMeasBlock_Put(pz_FeedbackInsMeasBlock);
  loc_core_report_gnss_feedback_ins(&z_loc_GnssFeedbackIns);
  return;
}

static void gnssGPSEphToApi(const GpsEphemeris_t* eph, loc_api_GpsEphemeris_t* api_eph)
{
  api_eph->svid = eph->svid;
  api_eph->week = eph->week;
  api_eph->accuracy = eph->accuracy;
  api_eph->health = eph->health;
  api_eph->iodc = eph->iodc;
  api_eph->code = eph->code;
  api_eph->fit = eph->fit;
  api_eph->iode = eph->iode;

  api_eph->tgd = eph->tgd;
  api_eph->toc = eph->toc;
  api_eph->toe = eph->toe;
  api_eph->sqrt_A = eph->sqrt_A;
  api_eph->e = eph->e;
  api_eph->M0 = eph->M0;
  api_eph->Omega0 = eph->Omega0;
  api_eph->Omega = eph->Omega;
  api_eph->OmegaDot = eph->OmegaDot;

  api_eph->i0 = eph->i0;
  api_eph->idot = eph->idot;
  api_eph->DeltaN = eph->DeltaN;
  api_eph->crc = eph->crc;
  api_eph->crs = eph->crs;
  api_eph->cuc = eph->cuc;
  api_eph->cus = eph->cus;
  api_eph->cic = eph->cic;
  api_eph->cis = eph->cis;
  api_eph->af0 = eph->af0;
  api_eph->af1 = eph->af1;
  api_eph->af2 = eph->af2;
}
static void gnssBDSEphToApi(const BdsEphemeris_t* eph, loc_api_BdsEphemeris_t* api_eph)
{
  api_eph->svid = eph->svid;
  api_eph->week = eph->week;
  api_eph->accuracy = eph->accuracy;
  api_eph->health = eph->health;
  api_eph->iodc = eph->iodc;
  api_eph->code = eph->code;
  api_eph->fit = eph->fit;
  api_eph->iode = eph->iode;

  api_eph->tgdB1I = eph->tgdB1I;
  api_eph->tgdB2I = eph->tgdB2I;
  api_eph->toc = eph->toc;
  api_eph->toe = eph->toe;
  api_eph->sqrt_A = eph->sqrt_A;
  api_eph->e = eph->e;
  api_eph->M0 = eph->M0;
  api_eph->Omega0 = eph->Omega0;
  api_eph->Omega = eph->Omega;
  api_eph->OmegaDot = eph->OmegaDot;

  api_eph->i0 = eph->i0;
  api_eph->idot = eph->idot;
  api_eph->DeltaN = eph->DeltaN;
  api_eph->crc = eph->crc;
  api_eph->crs = eph->crs;
  api_eph->cuc = eph->cuc;
  api_eph->cus = eph->cus;
  api_eph->cic = eph->cic;
  api_eph->cis = eph->cis;
  api_eph->af0 = eph->af0;
  api_eph->af1 = eph->af1;
  api_eph->af2 = eph->af2;
}
static void gnssGALEphToApi(const GalEphemeris_t* eph, loc_api_GalEphemeris_t* api_eph)
{
  api_eph->svid = eph->svid;
  api_eph->week = eph->week;
  api_eph->accuracy = eph->accuracy;
  api_eph->E1health = eph->E1health;
  api_eph->E1DVS = eph->E1DVS;
  api_eph->E5bhealth = eph->E5bhealth;
  api_eph->E5bDVS = eph->E5bDVS;
  api_eph->iodc = eph->iodc;
  api_eph->code = eph->code;
  api_eph->fit = eph->fit;
  api_eph->iode = eph->iode;

  api_eph->tgdE1E5a = eph->tgdE1E5a;
  api_eph->tgdE1E5b = eph->tgdE1E5b;
  api_eph->toc = eph->toc;
  api_eph->toe = eph->toe;
  api_eph->sqrt_A = eph->sqrt_A;
  api_eph->e = eph->e;
  api_eph->M0 = eph->M0;
  api_eph->Omega0 = eph->Omega0;
  api_eph->Omega = eph->Omega;
  api_eph->OmegaDot = eph->OmegaDot;

  api_eph->i0 = eph->i0;
  api_eph->idot = eph->idot;
  api_eph->DeltaN = eph->DeltaN;
  api_eph->crc = eph->crc;
  api_eph->crs = eph->crs;
  api_eph->cuc = eph->cuc;
  api_eph->cus = eph->cus;
  api_eph->cic = eph->cic;
  api_eph->cis = eph->cis;
  api_eph->af0 = eph->af0;
  api_eph->af1 = eph->af1;
  api_eph->af2 = eph->af2;
}

/**
 * @brief SM module report the ephemeris information from GNSS
 * @param[in]   p_ipc - IPC with receiver ephemeris data in RTCM
 * @return      None
 */
static void sm_GnssEphemeris_Report(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "ipc pointer is NULL sm_GnssEphemeris_Report\n");
    return;
  }
  if ((sizeof(gnss_Ephemeris_t) != p_ipc->q_length))
  {
    LOGE(TAG_SM, "Ipc 0x08x length %d mismatch payload %d\n",
      p_ipc->q_ipc_id, p_ipc->q_length, sizeof(gnss_Ephemeris_t));
    return;
  }
  gnss_Ephemeris_t* pz_GnssEph = (gnss_Ephemeris_t*)p_ipc->p_data;
  loc_api_GnssEphemeris_t z_api_GnssEph = { 0 };

  z_api_GnssEph.u_constellation = sm_ConvertConstellationTypeToLoc(pz_GnssEph->u_constellation);
  z_api_GnssEph.q_towMsecOfToe = pz_GnssEph->z_toeOfGpst.q_towMsec;
  z_api_GnssEph.w_weekOfToe = pz_GnssEph->z_toeOfGpst.w_week;
  z_api_GnssEph.q_towMsecOfToc = pz_GnssEph->z_tocOfGpst.q_towMsec;
  z_api_GnssEph.w_weekOfToc = pz_GnssEph->z_tocOfGpst.w_week;

  /* Caution: This code is high risk, need to add struction convert field by field  */
  switch (pz_GnssEph->u_constellation)
  {
  case C_GNSS_GPS:
    gnssGPSEphToApi(&pz_GnssEph->eph.z_gpsEph, &z_api_GnssEph.eph.z_gpsEph);
    break;
  case C_GNSS_GLO:
    memcpy(&z_api_GnssEph.eph.z_gloEph, &pz_GnssEph->eph.z_gloEph, sizeof(GloEphemeris_t)); // TBD
    break;
  case C_GNSS_BDS2:
  case C_GNSS_BDS3:
    gnssBDSEphToApi(&pz_GnssEph->eph.z_bdsEph, &z_api_GnssEph.eph.z_bdsEph);
    break;
  case C_GNSS_GAL:
    gnssGALEphToApi(&pz_GnssEph->eph.z_galEph, &z_api_GnssEph.eph.z_galEph);
    break;
  case C_GNSS_QZS:
    gnssGPSEphToApi(&pz_GnssEph->eph.z_qzsEph, &z_api_GnssEph.eph.z_qzsEph);
    break;
  default:
    break;
  }
  
  loc_core_report_gnss_eph(&z_api_GnssEph);

  return;
}

/**
* @brief SM module report the information from GNSS to INS
* @param[in]   p_ipc - IPC with receiver measurement data in RTCM
* @return      None
*/
static void sm_VdrPositionFix_Report(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  return;
}

/**
 * @brief SM Receiver measurement put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sm_OrtMeas_Put(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  static uint64_t q_preSecondAlignMsec = 0;
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();

  GnssMeasBlock_t* pz_MeasBlock = (GnssMeasBlock_t*)p_ipc->p_data;

  /* Inject dcp meas */
  dcp_api_RcvOrtMeasBlk_Put(pz_MeasBlock);

  if (gnss_CheckMeasSecondAlign(pz_MeasBlock->z_Clock.z_gpsTime.t_fullMsec,
    &q_preSecondAlignMsec, pz_loc_ConfigParamGroup->sm_cfg.f_meas_update_interval))
  {
    ort_api_OrtMeasBlk_Put(pz_MeasBlock);
  }

  return;
}

/**
 * @brief Sanity Check IPC
 * @param[in]   p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL sm_IpcCheck(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return FALSE;
  }

  if (TASK_INDEX_SM != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_SM_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_SM_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief SM IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_Proc(ipc_t* p_ipc)
{
  if (NULL == p_ipc)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  if (FALSE == sm_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_SM_INIT:
    sm_Init(p_ipc);
    break;
  case C_M_SM_START:
    sm_Start(p_ipc);
    break;
  case C_M_SM_STOP:
    sm_Stop(p_ipc);
    break;
  case C_M_SM_RELEASE:
    sm_Release(p_ipc);
    break;
  case C_M_SM_RCV_OBS_RTCM:
    sm_RcvMeasRTCM_Put(p_ipc);
    break;
  case C_M_SM_REF_CORR_RTCM:
    sm_RefCorrRTCM_Put(p_ipc);
    break;
  case C_M_SM_REPORT_POS_FIX:
    sm_PostionFix_Report(p_ipc);
    break;
  case C_M_SM_SSR_LOS_BLK:
    sm_SsrLocBlock_Put(p_ipc);
    break;
  case C_M_SM_RCV_MEAS_PUT:
    sm_RcvMeas_Put(p_ipc);
    break;
  case C_M_SM_SSR_STREAM_QX:
    sm_SsrQianXunStream_Put(p_ipc);
    break;
  case C_M_SM_SSR_STREAM_GEE:
    sm_SsrGeeSpaceStream_Put(p_ipc);
    break;
  case C_M_SM_TWIN_RCV_OBS_RTCM:
    sm_RcvTwinMeasRTCM_Put(p_ipc);
    break;
  case C_M_SM_REPORT_GNSS_FB_INS:
    sm_GnssFeedbackInsMeasBlock_Report(p_ipc);
    break;
  case C_M_SM_REPORT_GNSS_EPH:
    sm_GnssEphemeris_Report(p_ipc);
    break;
  case C_M_SM_ORT_MEAS_PUT:
    sm_OrtMeas_Put(p_ipc);
    break;
  case C_M_SM_REPORT_ORT_FIX:
    sm_OrientFix_Report(p_ipc);
		break;
	case C_M_SM_REPORT_RCV_OBS_MEAS_BLK:
		sm_GnssMeasBlockCollect_Report(p_ipc);
    break;
	case C_M_SM_REPORT_RCV_OBS_MEAS_SAT_BLK:
		sm_GnssMeasSatBlockCollect_Report(p_ipc);
    break;
  case C_M_SM_REPORT_NAV_DATA_FRAME:
    sm_NavDataFrame_Report(p_ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief SM IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* sm_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_SM;

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
      sm_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief SM API for initilization
 * @return      None
 */
BOOL sm_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief SM API for start
 * @return      None
 */
BOOL sm_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_START, NULL, 0);
  return TRUE;
}

/**
 * @brief SM API for stop
 * @return      None
 */
BOOL sm_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief SM API for release
 * @return      None
 */
BOOL sm_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief SM API to report position fix
 * @param[in]   src - source task index
 * @return      None
 */
void sm_api_PositionFix_Report(gnss_PositionFix_t* pz_PositionFix)
{
  if (NULL == pz_PositionFix)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }
  pz_PositionFix->u_version = VERSION_POSITION_FIX;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_POS_FIX,
    (uint8_t*)pz_PositionFix, sizeof(gnss_PositionFix_t));
  return;
}
/**
 * @brief SM API to report orientation fix
 * @param[in]   pz_OrientFix - orientation result
 * @return      None
 */
void sm_api_OrientFix_Report(gnss_OrientFix_t* pz_OrientFix)
{
  if (NULL == pz_OrientFix)
  {
    LOGE(TAG_SM, "pz_OrientFixis null pointer");
    return;
  }
  pz_OrientFix->u_version = VERSION_GNSS_ORT_FIX;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_ORT_FIX,
    (uint8_t*)pz_OrientFix, sizeof(gnss_OrientFix_t));
  return;
}

/**
 * @brief GNSS API to inject receiver observe measurement structure
 * @param[in]   pz_GnssMeasBlk - measurement structure
 * @return      None
 * @note receive measure not only getting from extern injection, but also
 *       from inject RTCM stream decoding
 */
void sm_api_RcvMeasBlk_Put(GnssMeasBlock_t* pz_GnssMeasBlk)
{
  if (NULL == pz_GnssMeasBlk)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }
  pz_GnssMeasBlk->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_RCV_MEAS_PUT,
    (uint8_t*)pz_GnssMeasBlk, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief GNSS API to inject orient recevier observe measurement structure
 * @param[in]   meas - measurement structure
 * @return      None
 * @note receive measure not only getting from extern injection, but also
 *       from inject RTCM stream decoding
 */
void sm_api_OrtMeasBlk_Put(GnssMeasBlock_t* pz_GnssMeasBlk)
{
  if (NULL == pz_GnssMeasBlk)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }
  pz_GnssMeasBlk->u_version = VERSION_GNSS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_ORT_MEAS_PUT,
    (uint8_t*)pz_GnssMeasBlk, sizeof(GnssMeasBlock_t));
  return;
}

/**
 * @brief SM API to GNSS feedback INS with measurement block
 * @param[in]   pz_GnssFeedbackInsMeasBlock - the measure blcok that GNSS feedback to INS
 * @return      None
 */
void sm_api_GnssFeedbackInsMeasBlock_Report(gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock)
{
  if (NULL == pz_GnssFeedbackInsMeasBlock)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }
  pz_GnssFeedbackInsMeasBlock->u_version = VERSION_GNSS_FEEDBACK_INS_MEAS_BLOCK;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_GNSS_FB_INS,
    (uint8_t*)pz_GnssFeedbackInsMeasBlock, sizeof(gnss_FeedbackInsMeasBlock_t));
  return;
}

/**
 * @brief Report meas block collect
 * @param[in] pz_GnssMeasBlockCollect
 */
void sm_api_GnssMeasBlockCollect_Report(GnssMeasBlockCollect_t * pz_GnssMeasBlockCollect)
{
  if (NULL == pz_GnssMeasBlockCollect)
  {
    LOGE(TAG_SM, "Null pointer input sm_api_GnssMeasBlockCollect_Report");
    return;
  }
	pz_GnssMeasBlockCollect->version = VERSION_GNSS_MEAS_BLOCK_COLLECT;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_RCV_OBS_MEAS_BLK,
    (uint8_t*)pz_GnssMeasBlockCollect, sizeof(GnssMeasBlockCollect_t));
  return;
}

/**
 * @brief Report meas and satellite block collect
 * @param[in] pz_GnssMeasSatBlockCollect
 */
void sm_api_GnssMeasSatBlockCollect_Report(GnssMeasSatBlockCollect_t * pz_GnssMeasSatBlockCollect)
{
  if (NULL == pz_GnssMeasSatBlockCollect)
  {
    LOGE(TAG_SM, "Null pointer input sm_api_GnssMeasSatBlockCollect_Report");
    return;
  }
	pz_GnssMeasSatBlockCollect->version = VERSION_GNSS_MEAS_SAT_BLOCK_COLLECT;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_RCV_OBS_MEAS_SAT_BLK,
    (uint8_t*)pz_GnssMeasSatBlockCollect, sizeof(GnssMeasSatBlockCollect_t));
  return;
}

/**
 * @brief Report navigation data frame
 * @param[in] pz_NavData
 */
void sm_api_NavigationData_Report(NavigationData_t * pz_NavData)
{
  if (NULL == pz_NavData)
  {
    LOGE(TAG_SM, "Null pointer input sm_api_NavigationData_Report");
    return;
  }
  pz_NavData->version = VERSION_NAVIGATION_DATA;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_NAV_DATA_FRAME,
    (uint8_t*)pz_NavData, sizeof(NavigationData_t));
  return;
}



/**
 * @brief SM API to GNSS feedback with ephemeris
 * @param[in]   pz_GnssEph - the ephemeris blcok
 * @return      None
 */
void sm_api_GnssEphemeris_Report(gnss_Ephemeris_t* pz_GnssEph)
{
  pz_GnssEph->u_version = VERSION_GNSS_EPHEMERIS;
  ipctask_SendMessage(TASK_INDEX_SM, C_M_SM_REPORT_GNSS_EPH,
    (uint8_t*)pz_GnssEph, sizeof(gnss_Ephemeris_t));
  return;
}
