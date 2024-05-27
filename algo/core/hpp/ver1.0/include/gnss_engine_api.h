/**@file        gnss_engine_api.h
 * @brief       gnss position engine api header file
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

#ifndef __GNSS_ENGINE_API_H__
#define __GNSS_ENGINE_API_H__

#include "gnss_def.h"
#include "gnss_type.h"
#include "integ_type.h"

BEGIN_DECL


typedef struct {
  void (*report_location)(gnss_NavSolution_t* location_info);
  void (*report_log)(int level, char* log, int length);
} GnssEngineCallback_t;

void gnss_engine_set_callback(GnssEngineCallback_t* callback);

void gnss_engine_init(float f_eleMask);

void gnss_engine_release();

void gnss_engine_create_empty_pos(const GnssMeasBlock_t* pz_MeasBlock, gnss_PositionFix_t* pz_PositionFix);

void gnss_engine_inject_meas(GnssMeasBlock_t* pz_MeasBlk, gnss_FeedbackInsMeasBlock_t* pz_CurrentFeedbackInsMeas);

void gnss_engine_inject_correct(GnssCorrBlock_t* pz_CorrBlk);

void gnss_engine_inject_ephemeris(gnss_Ephemeris_t* pz_eph);

void gnss_engine_inject_pvt_solution(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_PVTResult_t* pz_pvt_result);

void gnss_engine_nmea_creat(gnss_PositionFix_t* pz_position, gnss_NmeaInfo_t* pz_nema_info);

void gnss_engine_get_position_fix(gnss_PositionFix_t* pz_PositionFix);

uint8_t gnss_engine_get_peMode_stastic_flag();

uint8_t gnss_engine_get_peMode_historyStastic_flag();

END_DECL

#endif