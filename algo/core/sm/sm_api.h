/**@file        sm_api.h
 * @brief       Session Manager(SM) api header file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SM_API_H__
#define __SM_API_H__

#include "gnss_type.h"
#include "mw_ipctask.h"
#include "loc_core_api.h"
#include "loc_core_cfg.h"
#include "fusion_api.h"

BEGIN_DECL

/**
 * @brief SM IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
  void sm_Proc(ipc_t* p_ipc);

/**
 * @brief SM IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* sm_task(void* args);

/**
 * @brief SM API for initilization
 * @return      None
 */
BOOL sm_api_Init();

/**
 * @brief SM API for start
 * @return      None
 */
BOOL sm_api_Start();

/**
 * @brief SM API for stop
 * @return      None
 */
BOOL sm_api_Stop();

/**
 * @brief SM API for release
 * @return      None
 */
BOOL sm_api_Release();

/**
 * @brief SM API to report position fix
 * @param[in]   src - source task index
 * @return      None
 */
void sm_api_PositionFix_Report(gnss_PositionFix_t* pz_PositionFix);

/**
 * @brief SM API to report orientation fix
 * @param[in]   pz_OrientFix - orientation result
 * @return      None
 */
void sm_api_OrientFix_Report(gnss_OrientFix_t* pz_OrientFix);

/**
 * @brief GNSS API to inject receiver observe measurement structure
 * @param[in]   meas - measurement structure
 * @return      None
 * @note receive measure not only getting from extern injection, but also
 *       from inject RTCM stream decoding
 */
void sm_api_RcvMeasBlk_Put(GnssMeasBlock_t* pz_GnssMeasBlk);

/**
 * @brief GNSS API to inject orient recevier observe measurement structure
 * @param[in]   meas - measurement structure
 * @return      None
 * @note receive measure not only getting from extern injection, but also
 *       from inject RTCM stream decoding
 */
void sm_api_OrtMeasBlk_Put(GnssMeasBlock_t* pz_GnssMeasBlk);

/**
 * @brief SM API to report GNSS to INS cooperate information
 * @param[in]   pz_measFeedbackInsBlock - the measure blcok that GNSS feedback to INS
 * @return      None
 */
void sm_api_GnssFeedbackInsMeasBlock_Report(gnss_FeedbackInsMeasBlock_t* pz_measFeedbackInsBlock);

/**
 * @brief Report meas block collect
 * @param[in] pz_GnssMeasBlockCollect
 */
void sm_api_GnssMeasBlockCollect_Report(GnssMeasBlockCollect_t * pz_GnssMeasBlockCollect);

/**
 * @brief Report meas and satellite block collect
 * @param[in] pz_GnssMeasSatBlockCollect
 */
void sm_api_GnssMeasSatBlockCollect_Report(GnssMeasSatBlockCollect_t * pz_GnssMeasSatBlockCollect);

/**
 * @brief Report navigation data frame
 * @param[in] pz_NavData
 */
void sm_api_NavigationData_Report(NavigationData_t * pz_NavData);

/**
 * @brief SM API to GNSS feedback with ephemeris
 * @param[in]   pz_GnssEph - the ephemeris blcok
 * @return      None
 */
void sm_api_GnssEphemeris_Report(gnss_Ephemeris_t* pz_GnssEph);

/**
 * @brief SM API to GNSS feedback with observation
 * @param[in]   pz_GnssObs - the measure blcok
 * @return      None
 */
void sm_api_GnssFeedbackObs_Report(gnss_TightSatSigMeasCollect_t* pz_GnssObs);

void sm_api_VdrPositionFix_Report(INSResults_t* pz_VdrPosFix);

/**
 * @brief SM module inject receiver measurement data
 * @param[in]   p_ipc - IPC with receiver measurement data in RTCM
 * @return      None
 */
void sm_RcvMeasRTCM_Put(ipc_t* p_ipc);

/**
 * @brief SM Put reference correction
 * @param[in]   p_ipc - IPC with reference correction data in RTCM
 * @return      None
 */
void sm_RefCorrRTCM_Put(ipc_t* p_ipc);

/**
 * @brief SM Put reference correction measurement block
 * @param[in]   p_ipc - IPC with reference correction data
 * @return      None
 */
void sm_RefCorrMeasBlock_Put(ipc_t* p_ipc);

/**
 * @brief Session Manager Put SSR Loc Block Data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrLocBlock_Put(ipc_t* p_ipc);

/**
 * @brief Session Manager Put SSR Stream Data from QianXun
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrQianXunStream_Put(ipc_t* p_ipc);

/**
 * @brief Session Manager Put SSR Stream Data from GeeSpace
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrGeeSpaceStream_Put(ipc_t* p_ipc);

/**
 * @brief Session Manager Put AG SSR Los data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sm_SsrAgLos_Put(ipc_t* p_ipc);

/**
 * @brief SM module inject twin receiver measurement data
 * @param[in]   p_ipc - IPC with receiver measurement data in RTCM
 * @return      None
 */
void sm_RcvTwinMeasRTCM_Put(ipc_t* p_ipc);

END_DECL

#endif