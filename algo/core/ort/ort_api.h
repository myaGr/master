/**@file        ort_api.h
 * @brief       Double Antenna Orient Calculate Module Api header 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/12  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __ORT_API_H__
#define __ORT_API_H__

#include "gnss_type.h"
#include "mw_ipctask.h"

BEGIN_DECL

/**
 * @brief ORT IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ort_Proc(ipc_t* p_ipc);

/**
 * @brief ORT IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ort_task(void* args);

/**
 * @brief ORT API for initilization
 * @return      None
 */
BOOL ort_api_Init();

/**
 * @brief ORT API for start
 * @return      None
 */
BOOL ort_api_Start();

/**
 * @brief ORT API for stop
 * @return      None
 */
BOOL ort_api_Stop();

/**
 * @brief ORT API for release
 * @return      None
 */
BOOL ort_api_Release();

/**
 * @brief ORT API to Put Rcv Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ort_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas);

/**
 * @brief ORT API to Put Orient Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ort_api_OrtMeasBlk_Put(GnssMeasBlock_t* meas);

/**
 * @brief ORT API to Put PVT information of main antenna from SM or other task
 * @param[in]   pz_pvtInfo - the PVT information of main antenna
 * @return None
 */
void ort_api_mainAntennaPVTinfo_put(gnss_PVTResult_t* pz_pvtInfo);

END_DECL

#endif
