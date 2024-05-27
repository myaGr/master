/**@file        vdr_api.h
 * @brief       Vehicle Dead Reckoning
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

#ifndef __VDR_API_H__
#define __VDR_API_H__

#include "mw_ipctask.h"
#include "ekf_task.h"

BEGIN_DECL

/**
 * @brief: VDR IPC main proc function
 * @param[in]: p_ipc - pointer to IPC message
 * @return: None
 */
void vdr_Proc(ipc_t* p_ipc);


/**
 * @brief: VDR IPC process task
 * @param[in]: args - task configuration
 * @return: None
 */
void* vdr_task(void* args);

/**
 * @brief: VDR API for initilization
 * @return: None
 */
BOOL vdr_api_Init();

/**
 * @brief: VDR API for start
 * @return: None
 */
BOOL vdr_api_Start();

/**
 * @brief: VDR API for stop
 * @return: None
 */
BOOL vdr_api_Stop();

/**
 * @brief: VDR API for release
 * @return: None
 */
BOOL vdr_api_Release();

/**
 * @brief: VDR API for IMU data put
 * @return: None
 */
BOOL vdr_api_ImuData_Put(ins_ImuData_t* pz_ImuData);

/**
 * @brief: VDR API for GNSS fix put
 * @return: None
 */
BOOL vdr_api_GnssFix_Put(ins_GnssFix_t* pz_GnssFix);

/**
 * @brief: VDR API for Wheel data put
 * @return: None
 */
BOOL vdr_api_WheelData_Put(ins_WheelData_t* pz_WheelData);

/**
 * @brief: VDR API for GNSS fix put
 * @return: None
 */
BOOL vdr_api_EkfResult_Put(ins_SysErrModData_t* pz_GnssFix);

/**
 * @brief: VDR API to GNSS feedback INS with measurement block
 * @param[in]: pz_GnssFeedbackInsMeasBlock - the measure blcok that GNSS feedback to INS
 * @return: None
 */
BOOL vdr_api_GnssInsMeasBlock_Put(gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock);

END_DECL

#endif