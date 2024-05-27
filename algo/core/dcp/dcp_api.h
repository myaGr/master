/**@file        dcp_api.h
 * @brief       Differential Carrier Phase 
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

#ifndef __DCP_API_H__
#define __DCP_API_H__

#include "gnss_type.h"

#include "mw_ipctask.h"

BEGIN_DECL

/**
 * @brief DCP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void dcp_Proc(ipc_t* p_ipc);


/**
 * @brief DCP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* dcp_task(void* args);

/**
 * @brief DCP API for initilization
 * @return      None
 */
BOOL dcp_api_Init();

/**
 * @brief DCP API for start
 * @return      None
 */
BOOL dcp_api_Start();

/**
 * @brief DCP API for stop
 * @return      None
 */
BOOL dcp_api_Stop();

/**
 * @brief DCP API for release
 * @return      None
 */
BOOL dcp_api_Release();

/**
 * @brief HPP API to Put Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void dcp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas);

/**
 * @brief DCP API to Put Orient Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void dcp_api_RcvOrtMeasBlk_Put(GnssMeasBlock_t* meas);

/**
 * @brief HPP API to Put Tdcp Measurement @1Hz from HPP Task
 * @param[in]   tdcpMeas - Tdcp Measurement @1Hz from HPP Task
 * @return None
 */
void dcp_api_TdcpMeasBlk_Put(gnss_TdcpMeasBlock_t* tdcpMeas);

/**
 * @brief DCP API to Put Tdcp Measurement of main antenna from ORT Task
 * @param[in]   pz_tdcpMeas - Tdcp Measurement @1Hz from ORT Task
 * @return None
 */
void dcp_api_MainAntTdcpMeasBlk_Put(gnss_TdcpMeasBlock_t* pz_tdcpMeas);

/**
 * @brief DCP API to Put Tdcp Measurement of auxi antenna from ORT Task
 * @param[in]   pz_tdcpMeas - Tdcp Measurement @1Hz from ORT Task
 * @return None
 */
void dcp_api_AuxiAntTdcpMeasBlk_Put(gnss_TdcpMeasBlock_t* pz_tdcpMeas);

/**
 * @brief DCP API to Put orient result from ORT Task
 * @param[in]   pz_attitudeResult - orient result from ORT Task
 * @return None
 */
void dcp_api_orientBackGround_Put(gnss_OrientFix_t* pz_attitudeResult);


END_DECL

#endif