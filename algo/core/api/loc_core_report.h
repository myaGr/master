/**@file        loc_core_report.h
 * @brief       Location engine core report data header file,
                it's avaliable for user
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

#ifndef __LOC_CORE_REPORT_H_
#define __LOC_CORE_REPORT_H_

#include <stdint.h>
#include "gnss_type.h"
#include "loc_core_api.h"

BEGIN_DECL

const char* loc_report_get_string_from_id(uint16_t w_report_id);

/* GNSS Measurement structure */
typedef struct {
  loc_api_gnssConstellationType  u_constellation;         /** Measurement constellation, see loc_api_gnssConstellationType */
  uint8_t       u_svid;                  /** Measurement satellite index of the iteself system */
  uint8_t       u_signal;                /** Measurement signal type, see gnss_SignalType */
  uint8_t       u_LLI;                   /** Loss of lock indicator */
  float         f_cn0;                   /** Carrier-to-noise density */
  double        d_doppler;               /** Doppler */
  double        d_pseudoRange;           /** Pseudo-range */
  double        d_carrierPhase;          /** Carrier phase */
} GnssMeasReport_t;

 /* Station and Measurement Report Struture */
#define VERSION_GNSS_SAT_MEAS_REPORT  (0)
typedef struct {
  uint8_t          u_Version;
  /** w_Size = sizeof(gnss_AggregativePositionMeasureReport_t) + w_MeasCount * sizeof(GnssMeasReport_t) */
  uint16_t         w_Size;
  uint16_t         w_GpsWeek;
  uint32_t         q_GpsTowMsec;
  uint8_t          u_FixStatus;
  double           d_PosLla[3];
  float            f_VelEnu[3];
  float            f_PosXyzUnc[3];
  float            f_VelEnuUnc[3];
  uint16_t         w_MeasCount;
  GnssMeasReport_t z_Meas[1];
} gnss_CombinedPositionMeasureReport_t;

void loc_report_SetCallback(loc_api_callback_t* pz_Callback);

void loc_core_report_log(uint8_t* buf, uint32_t len);

void loc_core_report_location(loc_api_location_report_t* info);

void loc_core_report_orient(loc_api_orient_report_t* info);

void loc_core_report_consoildated_location(loc_api_ConsoildatedPositionFix_t* info);

void loc_core_report_gnss_feedback_ins(loc_api_GnssFeedbackInsMeasBlock_t* pz_GnssFeedbackIns);

void loc_core_report_gnss_eph(loc_api_GnssEphemeris_t* pz_GnssEph);

void loc_core_report_gnss_meas_blk(loc_api_GnssMeasBlock_t* pz_GnssObs);

void loc_core_report_gnss_meas_sat_blk(loc_api_GnssMeasSatBlock_t* pz_GnssMeasSatBlk);

void loc_core_report_navigation_data(loc_api_NavigationData_t* pz_NavData);

uint64_t loc_core_get_tick_ms();

/**
 * @brief Location core report Gnss combined position and measurement data as
 *        a serialized buffer to user
 * @param[in]   pz_StationMeasureReport - Combined position and measurement data
 * @return      None
 */
void loc_report_CombinedPositionMeasure(gnss_CombinedPositionMeasureReport_t* pz_AggregativePositionMeasureReport);


BOOL loc_report_cvt_ParseBufferToSsrLosBlock(
  gnss_ssrLosBlock_t* pz_SsrLosBlock,
  uint8_t* p_data, uint32_t q_length);

/**
 * @brief Time provider set start gps time
 * @param[in]   pz_StartGpst  - start gps time
 * @return None
 */
void timepro_set_gpst(GpsTime_t* pz_StartGpst);

/**
 * @brief Time provider get unix millisecond
 * @param[in]
 * @return unix millisecond
 */
uint64_t timepro_get_now();

/**
 * @brief Time cost
 * @param[in] t_tstart timestamp
 * @return time cost before t_tstart to now. If LOG_OPT_DONT_TIME_COST be set, return 0
 */
uint64_t time_cost(uint64_t t_tstart);

/* Location engine monitor structure */
#define VERSION_LOC_MONITOR_INFO  (1)
typedef struct {
  uint8_t  u_version;
  char     build_version[256];
  uint32_t q_ReportDuration;
  uint64_t t_StartTime;
  uint64_t t_RunningTime;
  uint64_t t_ReportCount;
  uint64_t t_ReportCountPrevious;
  uint64_t t_HppReportCount;
  uint64_t t_RtkReportCount;
  uint64_t t_PppReportCount;
  loc_api_config_para_group_t z_ConfigPara;
} loc_MonitorStructType;

/**
 * @brief Get loc_MonitorStructType to fill information
 * @return pointer to loc_MonitorStructType
 */

loc_MonitorStructType* loc_report_GetMonitor();

/**
 * @brief Release loc_MonitorStructType
 * @return None
 */
void loc_report_ReleaseMonitor();

/**
 * @brief Report loc engine monistor information
 * @return      None
 */
void loc_report_MonitorReport();

END_DECL

#endif

