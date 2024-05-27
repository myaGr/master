/**@file        ppp_api.h
 * @brief       Precision Point Positioning
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

#ifndef __PPP_API_H__
#define __PPP_API_H__

#include "mw_ipctask.h"

#include "gnss_type.h"

BEGIN_DECL

/**
 * @brief PPP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_Proc(ipc_t* p_ipc);


/**
 * @brief PPP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ppp_task(void* args);

/**
 * @brief PPP API for initilization
 * @return      None
 */
BOOL ppp_api_Init();

/**
 * @brief PPP API for start
 * @return      None
 */
BOOL ppp_api_Start();

/**
 * @brief PPP API for stop
 * @return      None
 */
BOOL ppp_api_Stop();

/**
 * @brief PPP API for release
 * @return      None
 */
BOOL ppp_api_Release();

/**
 * @brief HPP API to Put Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ppp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas);

/**
 * @brief PPP API to Put SSR Loc Block Data
 * @param[in]   pz_SsrLocBlk - SSR Loc Block Data
 * @return      None
 */
void ppp_api_SsrLocBlock_Put(gnss_ssrLosBlock_t* pz_SsrLocBlk);

/**
 * @brief PPP API to Put tight satellite signal measurement collect
          and convert it as a sparse satellite signal measurement
          collect stored in PPP
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_TightSatSigMeasCollect_Put(gnss_TightSatSigMeasCollect_t* pz_TightSatSigMeasCollect);

/**
 * @brief PPP API to start run algorithm
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_Algorithm_Start();

/**
 * @brief PPP API to inject Qianxun SSR stream
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_SsrQianXunStream_Put(uint8_t* data, uint32_t len);

/**
 * @brief PPP API to inject AG SSR Los data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_SsrAgLos_Put(uint8_t* data, uint32_t len);

END_DECL

#endif