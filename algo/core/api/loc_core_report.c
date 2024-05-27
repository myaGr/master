/**@file        loc_core_report.c
 * @brief       Location engine core report data source file, 
                it's unavaliable for user
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/01  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */


#include "cmn_utils.h"

#include "mw_log.h"
#include "mw_alloc.h"
#include "gnss_common.h"
#include "loc_core_report.h"
#include "loc_core_api.h"
#include "gnss_type.h"

static loc_api_callback_t gz_loc_core_report_callback = { 0 };
static loc_MonitorStructType* pz_loc_Monitor = NULL;

static struct {
  LogPackageIdType w_report_id;
  const char* id_name;
} log_package_id_names[] = {
  {LOG_PACKAGE_ID_START,           "LOG_PACKAGE_ID_START"     },
  {LOG_PACKAGE_ID_MONITOR,         "LOG_PACKAGE_ID_MONITOR"   },
  {LOG_PACKAGE_ID_LOC_CONFIG,      "LOG_PACKAGE_ID_LOC_CONFIG"},
  {LOG_PACKAGE_ID_INS_RESULT,      "LOG_PACKAGE_ID_INS_RESULT"} };

/**
 * @brief Convert ipc id to string
 * @return: string
 */
const char* loc_report_get_string_from_id(uint16_t w_report_id)
{
  uint32_t q_arr_count = sizeof(log_package_id_names) / sizeof(log_package_id_names[0]);
  for (uint32_t i = 0; i < q_arr_count; i++)
  {
    if (log_package_id_names[i].w_report_id == w_report_id)
    {
      return log_package_id_names[i].id_name;
    }
  }

  return "N/A";
}


void loc_report_SetCallback(loc_api_callback_t* pz_Callback)
{
  if (NULL == pz_Callback)
  {
    return;
  }

  gz_loc_core_report_callback.report_log = pz_Callback->report_log;
  gz_loc_core_report_callback.report_location = pz_Callback->report_location;
  gz_loc_core_report_callback.report_ephemeris = pz_Callback->report_ephemeris;
  gz_loc_core_report_callback.get_tick_ms = pz_Callback->get_tick_ms;
  gz_loc_core_report_callback.report_gnss_feedback_ins = 
    pz_Callback->report_gnss_feedback_ins;
  gz_loc_core_report_callback.report_consolidated_location =
    pz_Callback->report_consolidated_location;
  gz_loc_core_report_callback.report_gnss_measurement =
    pz_Callback->report_gnss_measurement;
  gz_loc_core_report_callback.report_gnss_measurement_satellite=
		  pz_Callback->report_gnss_measurement_satellite;
  gz_loc_core_report_callback.report_orient = pz_Callback->report_orient;
  gz_loc_core_report_callback.report_gnss_navigation_data = pz_Callback->report_gnss_navigation_data;
  return;
}

void loc_core_report_log(uint8_t* buf, uint32_t len)
{
  if (NULL == gz_loc_core_report_callback.report_log)
  {
    return;
  }
  gz_loc_core_report_callback.report_log(buf, len);
  return;
}

void loc_core_report_location(loc_api_location_report_t* info)
{
  if (NULL == gz_loc_core_report_callback.report_location)
  {
    return;
  }
  gz_loc_core_report_callback.report_location(info);
  return;
}

void loc_core_report_orient(loc_api_orient_report_t* info)
{
  if (NULL == gz_loc_core_report_callback.report_orient)
  {
    return;
  }
  gz_loc_core_report_callback.report_orient(info);
  return;
}

void loc_core_report_consoildated_location(loc_api_ConsoildatedPositionFix_t* info)
{
  if (NULL == gz_loc_core_report_callback.report_consolidated_location)
  {
    return;
  }
  gz_loc_core_report_callback.report_consolidated_location(info);
  return;
}

void loc_core_report_gnss_feedback_ins(loc_api_GnssFeedbackInsMeasBlock_t* pz_GnssFeedbackIns)
{
  if (NULL == gz_loc_core_report_callback.report_gnss_feedback_ins)
  {
    return;
  }
  gz_loc_core_report_callback.report_gnss_feedback_ins(pz_GnssFeedbackIns);
  return;
}

void loc_core_report_gnss_eph(loc_api_GnssEphemeris_t* pz_GnssEph)
{
  if (NULL == gz_loc_core_report_callback.report_ephemeris)
  {
    return;
  }
  gz_loc_core_report_callback.report_ephemeris(pz_GnssEph);
  return;
}

void loc_core_report_gnss_meas_blk(loc_api_GnssMeasBlock_t* pz_GnssMeasBlk)
{
  if (NULL == gz_loc_core_report_callback.report_gnss_measurement)
  {
    return;
  }
  gz_loc_core_report_callback.report_gnss_measurement(pz_GnssMeasBlk);
  return;
}

void loc_core_report_gnss_meas_sat_blk(loc_api_GnssMeasSatBlock_t* pz_GnssMeasSatBlk)
{
  if (NULL == gz_loc_core_report_callback.report_gnss_measurement_satellite)
  {
    return;
  }
  gz_loc_core_report_callback.report_gnss_measurement_satellite(pz_GnssMeasSatBlk);
  return;
}

void loc_core_report_navigation_data(loc_api_NavigationData_t* pz_NavData)
{
  if (NULL == gz_loc_core_report_callback.report_gnss_navigation_data)
  {
    return;
  }
  gz_loc_core_report_callback.report_gnss_navigation_data(pz_NavData);
}

uint64_t loc_core_get_tick_ms()
{
  if (NULL == gz_loc_core_report_callback.get_tick_ms)
  {
    return 0;
  }
  return gz_loc_core_report_callback.get_tick_ms();
}

#if 0

uint8_t* loc_report_cvt_AllocSsrLosBlockBuffer (
  gnss_ssrLosBlock_t* pz_SsrLosBlock,
  uint32_t* pq_buffer_size)
{
  if ((NULL == pz_SsrLosBlock) ||
    (NULL == pq_buffer_size))
  {
    return NULL;
  }

  /* 1. Serialize gnss_AggregativePositionMeasureReport_t */
  uint32_t q_SizeOfGnssZtdCorr     = 22;
  uint32_t q_SizeOfGpsTime         = 6;
  uint32_t q_SizeOfSrrLos          = 4 * q_SizeOfGpsTime + 3 + q_SizeOfGnssZtdCorr +
    pz_SsrLosBlock->z_epochLosInfo.u_satCount * sizeof(gnss_satSsrLos_t);
  uint32_t q_SizeOfSrrLosBlock     = q_SizeOfSrrLos + 3 + 8 * 3;

  uint32_t q_Length                = q_SizeOfSrrLosBlock;
  uint32_t q_BufferSize = q_Length + 8 + 2;

  uint16_t w_Index = 0;
  uint8_t* p_Data = (uint8_t*)OS_MALLOC(q_BufferSize);
  if (NULL == p_Data)
  {
    return NULL;
  }
  memset(p_Data, 0, q_BufferSize);

  /* Fill Header */
  w_Index += setU1(p_Data + w_Index, LOG_SYNC0);
  w_Index += setU1(p_Data + w_Index, LOG_SYNC1);
  w_Index += setU2(p_Data + w_Index, LOC_REPORT_ID_SSR_LOS_BLK);
  w_Index += setU4(p_Data + w_Index, q_Length);

  /* Fill Body */
  w_Index += setU1(p_Data + w_Index, pz_SsrLosBlock->u_version);
  w_Index += setU2(p_Data + w_Index, pz_SsrLosBlock->w_size);
  w_Index += setR8(p_Data + w_Index, pz_SsrLosBlock->d_siteCoor[0]);
  w_Index += setR8(p_Data + w_Index, pz_SsrLosBlock->d_siteCoor[1]);
  w_Index += setR8(p_Data + w_Index, pz_SsrLosBlock->d_siteCoor[2]);

  gnss_epochSsrLos_t* p_SsrLos = &(pz_SsrLosBlock->z_epochLosInfo);
  w_Index += setU4(p_Data + w_Index, p_SsrLos->z_tor.q_towMsec);
  w_Index += setU2(p_Data + w_Index, p_SsrLos->z_tor.w_week);
  w_Index += setU4(p_Data + w_Index, p_SsrLos->z_STECtime.q_towMsec);
  w_Index += setU2(p_Data + w_Index, p_SsrLos->z_STECtime.w_week);
  w_Index += setU4(p_Data + w_Index, p_SsrLos->z_STDtime.q_towMsec);
  w_Index += setU2(p_Data + w_Index, p_SsrLos->z_STDtime.w_week);
  w_Index += setU4(p_Data + w_Index, p_SsrLos->z_ZTDtime.q_towMsec);
  w_Index += setU2(p_Data + w_Index, p_SsrLos->z_ZTDtime.w_week);
  w_Index += setU1(p_Data + w_Index, p_SsrLos->u_ZTDmask);
  w_Index += setU1(p_Data + w_Index, p_SsrLos->u_errorModelMask);

  w_Index += setU1(p_Data + w_Index, p_SsrLos->z_ZTDcorr.u_preItg);
  w_Index += setU1(p_Data + w_Index, p_SsrLos->z_ZTDcorr.u_postItg);
  w_Index += setR4(p_Data + w_Index, p_SsrLos->z_ZTDcorr.f_qi);
  w_Index += setR8(p_Data + w_Index, p_SsrLos->z_ZTDcorr.d_wetCorr);
  w_Index += setR8(p_Data + w_Index, p_SsrLos->z_ZTDcorr.d_dryCorr);

  w_Index += setU1(p_Data + w_Index, p_SsrLos->u_satCount);

  for (uint8_t i = 0; i < p_SsrLos->u_satCount; i++)
  {
    memcpy(p_Data + w_Index, &(p_SsrLos->z_satLosCorr[i]), sizeof(gnss_satSsrLos_t));
    w_Index += sizeof(gnss_satSsrLos_t);
  }

  /* checksum */
  uint8_t ck[2] = { 0 };
  loc_checksum((uint8_t*)p_Data + 2, q_Length + 6, &ck[0], &ck[1]);
  w_Index += setU1(p_Data + w_Index, ck[0]);
  w_Index += setU1(p_Data + w_Index, ck[1]);

  *pq_buffer_size = q_BufferSize;
  return p_Data;
}

BOOL loc_report_cvt_ParseBufferToSsrLosBlock(
  gnss_ssrLosBlock_t* pz_SsrLosBlock,
  uint8_t* p_data, uint32_t q_length)
{
  uint16_t w_Index = 0;
  if ((NULL == pz_SsrLosBlock)||(NULL == p_data)){
    return FALSE;
  }
  w_Index = 8;
  pz_SsrLosBlock->u_version = getU1(p_data + w_Index); w_Index += 1;
  pz_SsrLosBlock->w_size = getU2(p_data + w_Index); w_Index += 2;
  pz_SsrLosBlock->d_siteCoor[0] = getR8(p_data + w_Index); w_Index += 8;
  pz_SsrLosBlock->d_siteCoor[1] = getR8(p_data + w_Index); w_Index += 8;
  pz_SsrLosBlock->d_siteCoor[2] = getR8(p_data + w_Index); w_Index += 8;

  gnss_epochSsrLos_t* p_SsrLos = &(pz_SsrLosBlock->z_epochLosInfo);
  p_SsrLos->z_tor.q_towMsec = getU4(p_data + w_Index); w_Index += 4;
  p_SsrLos->z_tor.w_week = getU2(p_data + w_Index); w_Index += 2;
  p_SsrLos->z_STECtime.q_towMsec = getU4(p_data + w_Index); w_Index += 4;
  p_SsrLos->z_STECtime.w_week = getU2(p_data + w_Index); w_Index += 2;
  p_SsrLos->z_STDtime.q_towMsec = getU4(p_data + w_Index); w_Index += 4;
  p_SsrLos->z_STDtime.w_week = getU2(p_data + w_Index); w_Index += 2;
  p_SsrLos->z_ZTDtime.q_towMsec = getU4(p_data + w_Index); w_Index += 4;
  p_SsrLos->z_ZTDtime.w_week = getU2(p_data + w_Index); w_Index += 2;
  p_SsrLos->u_ZTDmask = getU1(p_data + w_Index); w_Index += 1;
  p_SsrLos->u_errorModelMask = getU1(p_data + w_Index); w_Index += 1;

  p_SsrLos->z_ZTDcorr.u_preItg = getU1(p_data + w_Index); w_Index += 1;
  p_SsrLos->z_ZTDcorr.u_postItg = getU1(p_data + w_Index); w_Index += 1;
  p_SsrLos->z_ZTDcorr.f_qi = getR4(p_data + w_Index); w_Index += 4;
  p_SsrLos->z_ZTDcorr.d_wetCorr = getR8(p_data + w_Index); w_Index += 8;
  p_SsrLos->z_ZTDcorr.d_dryCorr = getR8(p_data + w_Index); w_Index += 8;
  p_SsrLos->u_satCount = getU1(p_data + w_Index); w_Index += 1;

  for (uint8_t i = 0; i < p_SsrLos->u_satCount; i++)
  {
    memcpy(&(p_SsrLos->z_satLosCorr[i]), p_data + w_Index, sizeof(gnss_satSsrLos_t));
    w_Index += sizeof(gnss_satSsrLos_t);
  }
  w_Index+=2;
  if (q_length != w_Index)
  {
    return FALSE;
  }

  return TRUE;
}
#endif

/**
 * @brief Location core report GNSS combined position and measurement data as
 *        a serialized buffer to user
 * @param[in]   pz_CombinedPositionMeasureReport - Combined position and measurement data
 * @return      None
 */
void loc_report_CombinedPositionMeasure(
  gnss_CombinedPositionMeasureReport_t* pz_CombinedPositionMeasureReport)
{
#if 0
  uint32_t q_data_size = 0;

  /* 1. Serialize gnss_CombinedPositionMeasureReport_t */
  uint16_t w_MeasCount = pz_CombinedPositionMeasureReport->w_MeasCount;
  uint32_t q_Length = 72 + 32 * w_MeasCount;
  uint32_t q_BufferSize = q_Length + 8 + 2;

  uint16_t w_Index = 0;
  uint8_t* p_Data = (uint8_t*)OS_MALLOC(q_BufferSize);
  if (NULL == p_Data)
  {
    return;
  }
  memset(p_Data, 0, q_BufferSize);

  /* Fill Header */
  w_Index += setU1(p_Data + w_Index, LOG_SYNC0);
  w_Index += setU1(p_Data + w_Index, LOG_SYNC1);
  w_Index += setU2(p_Data + w_Index, LOC_REPORT_ID_POS_AND_MEAS);
  w_Index += setU4(p_Data + w_Index, q_Length);

  /* Fill gnss_AggregativePositionMeasureReport_t payload */
  w_Index += setU1(p_Data + w_Index, pz_CombinedPositionMeasureReport->u_Version);
  w_Index += setU2(p_Data + w_Index, pz_CombinedPositionMeasureReport->w_Size);
  w_Index += setU2(p_Data + w_Index, pz_CombinedPositionMeasureReport->w_GpsWeek);
  w_Index += setU4(p_Data + w_Index, pz_CombinedPositionMeasureReport->q_GpsTowMsec);
  w_Index += setU1(p_Data + w_Index, pz_CombinedPositionMeasureReport->u_FixStatus);
  w_Index += setR8(p_Data + w_Index, pz_CombinedPositionMeasureReport->d_PosLla[0]);
  w_Index += setR8(p_Data + w_Index, pz_CombinedPositionMeasureReport->d_PosLla[1]);
  w_Index += setR8(p_Data + w_Index, pz_CombinedPositionMeasureReport->d_PosLla[2]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_VelEnu[0]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_VelEnu[1]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_VelEnu[2]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_PosXyzUnc[0]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_PosXyzUnc[1]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_PosXyzUnc[2]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_VelEnuUnc[0]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_VelEnuUnc[1]);
  w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->f_VelEnuUnc[2]);
  w_Index += setU2(p_Data + w_Index, pz_CombinedPositionMeasureReport->w_MeasCount);

  for (int k = 0; k < pz_CombinedPositionMeasureReport->w_MeasCount; k++)
  {
    w_Index += setU1(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].u_constellation);
    w_Index += setU1(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].u_svid);
    w_Index += setU1(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].u_signal);
    w_Index += setU1(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].u_LLI);
    w_Index += setR4(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].f_cn0);
    w_Index += setR8(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].d_doppler);
    w_Index += setR8(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].d_pseudoRange);
    w_Index += setR8(p_Data + w_Index, pz_CombinedPositionMeasureReport->z_Meas[k].d_carrierPhase);
  }

  /* checksum */
  uint8_t ck[2] = { 0 };
  loc_checksum((uint8_t*)p_Data + 2, q_Length + 6, &ck[0], &ck[1]);
  w_Index += setU1(p_Data + w_Index, ck[0]);
  w_Index += setU1(p_Data + w_Index, ck[1]);

  loc_core_report_log(p_Data, q_BufferSize);
  OS_FREE(p_Data);
#endif
  return;
}

/**
 * @brief Get loc_MonitorStructType to fill information
 * @return pointer to loc_MonitorStructType
 */
loc_MonitorStructType* loc_report_GetMonitor()
{
  if (NULL == pz_loc_Monitor)
  {
    pz_loc_Monitor = (loc_MonitorStructType*)OS_MALLOC(sizeof(loc_MonitorStructType));
    if (NULL == pz_loc_Monitor)
    {
      return NULL;
    }
    memset(pz_loc_Monitor, 0, sizeof(loc_MonitorStructType));
    pz_loc_Monitor->q_ReportDuration = 1000;
    return pz_loc_Monitor;
  }

  return pz_loc_Monitor;
}

/**
 * @brief Release loc_MonitorStructType
 * @return None
 */
void loc_report_ReleaseMonitor()
{
  if (NULL != pz_loc_Monitor)
  {
    OS_FREE(pz_loc_Monitor);
  }
  return;
}

/**
 * @brief Report loc engine monistor information
 * @return      None
 */
void loc_report_MonitorReport()
{
  if (NULL == pz_loc_Monitor)
  {
    pz_loc_Monitor = (loc_MonitorStructType*)OS_MALLOC(sizeof(loc_MonitorStructType));
    if (NULL == pz_loc_Monitor)
    {
      return;
    }
    memset(pz_loc_Monitor, 0, sizeof(loc_MonitorStructType));
    pz_loc_Monitor->u_version = VERSION_LOC_MONITOR_INFO;
    pz_loc_Monitor->q_ReportDuration = 2000;
  }

  if (pz_loc_Monitor->t_ReportCount == 0)
  {
    pz_loc_Monitor->t_StartTime = loc_core_get_tick_ms() / 1000;
  }

  pz_loc_Monitor->t_ReportCount++;
  if ((pz_loc_Monitor->t_ReportCount - pz_loc_Monitor->t_ReportCountPrevious) < pz_loc_Monitor->q_ReportDuration)
  {
    return;
  }
  pz_loc_Monitor->t_ReportCountPrevious = pz_loc_Monitor->t_ReportCount;
  pz_loc_Monitor->t_RunningTime = loc_core_get_tick_ms() / 1000 - pz_loc_Monitor->t_StartTime;

  if (strlen(pz_loc_Monitor->build_version) == 0)
  {
    get_version(pz_loc_Monitor->build_version);
  }

  loc_api_config_para_group_t* pz_loc_api_config = loc_cfg_getLocApiConfigPara();
  memcpy(&pz_loc_Monitor->z_ConfigPara, pz_loc_api_config, sizeof(loc_api_config_para_group_t));

  log_Package(LOG_PACKAGE_ID_MONITOR, (uint8_t*)pz_loc_Monitor, sizeof(loc_MonitorStructType));

  return;
}

/***************************************************************************
  Time provider
***************************************************************************/
typedef struct {
  uint8_t  u_isStartSet;
  uint8_t  u_isGpstSet;
  uint64_t t_StartTick;
  uint64_t t_StartUnixMsec;
  uint64_t t_NowTick;
  uint64_t t_NowUnixMsec;
} TimeProviderStructType;

TimeProviderStructType gz_TimeProvider;

/**
 * @brief Time provider set start gps time
 * @param[in]   pz_StartGpst  - start gps time
 * @return None
 */
void timepro_set_gpst(GpsTime_t* pz_StartGpst)
{
  if (!gz_TimeProvider.u_isGpstSet)
  {
    gz_TimeProvider.t_StartTick = loc_core_get_tick_ms();
    gz_TimeProvider.t_StartUnixMsec = tm_cvt_FullMsecToUnixMsec(pz_StartGpst->t_fullMsec);
    gz_TimeProvider.u_isStartSet = TRUE;
    gz_TimeProvider.u_isGpstSet = TRUE;
  }

  return;
}

/**
 * @brief Time provider get unix millisecond
 * @param[in]
 * @return unix millisecond
 */
uint64_t timepro_get_now()
{
  if (!gz_TimeProvider.u_isStartSet)
  {
    gz_TimeProvider.t_StartTick = loc_core_get_tick_ms();
    gz_TimeProvider.u_isStartSet = TRUE;
  }

  gz_TimeProvider.t_NowTick = loc_core_get_tick_ms();
  gz_TimeProvider.t_NowUnixMsec = gz_TimeProvider.t_StartUnixMsec +
    (gz_TimeProvider.t_NowTick - gz_TimeProvider.t_StartTick);

  return gz_TimeProvider.t_NowUnixMsec;
}

/**
 * @brief Time cost
 * @param[in] t_tstart timestamp
 * @return time cost before t_tstart to now. If LOG_OPT_DONT_TIME_COST be set, return 0
 */
uint64_t time_cost(uint64_t t_tstart)
{
  uint64_t t_cost = 0;
  loc_ConfigParamGroup_t* pz_conf = loc_cfg_getConfigParamGroup();
  if(!M_IS_SET_MASK(pz_conf->log_cfg.u_logTimeOption, LOG_OPT_DONT_TIME_COST))
  {
    uint64_t t_tend = timepro_get_now();
    t_cost = t_tend - t_tstart;

  }
  return t_cost;
}
