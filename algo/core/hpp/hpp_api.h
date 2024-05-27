/**@file        hpp_api.h
 * @brief       High Precision Position(HPP) api header file
 * @details     HPP include Multi-Constellation PVT and RTK
 * @author      caizhijie
 * @date        2022/04/26
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/26  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __HPP_API_H__
#define __HPP_API_H__

#include "mw_ipctask.h"
#include "gnss_type.h"

#include "loc_core_cfg.h"

BEGIN_DECL

/**
 * @brief HPP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void hpp_Proc(ipc_t* p_ipc);

/**
 * @brief HPP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* hpp_task(void* args);

/**
 * @brief HPP API for initilization
 * @return      None
 */
BOOL hpp_api_Init();

/**
 * @brief HPP API for start
 * @return      None
 */
BOOL hpp_api_Start();

/**
 * @brief HPP API for stop
 * @return      None
 */
BOOL hpp_api_Stop();

/**
 * @brief HPP API for release
 * @return      None
 */
BOOL hpp_api_Release();

/**
 * @brief HPP API to Put Receiver Measurement Block
 * @param[in] meas - Receiver Measurement Block
 * @return None
 */
void hpp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas);

/**
 * @brief HPP API to Put Reference Correction Block
 * @param[in] corr - Reference Correction Block
 * @return    None
 */
void hpp_api_RefCorrBlk_Put(GnssCorrBlock_t* corr);

/**
 * @brief HPP API to Put Ephemeris Structure
 * @param[in] eph - Ephemeris Structure
 * @return    None
 */
void hpp_api_Ephemeris_Put(gnss_Ephemeris_t* eph);

/**
 * @brief HPP API to Put receiver twin measurement
 * @return    None
 */
void hpp_api_RcvMeasTwin_Put(GnssMeasBlock_t* twin_meas);

END_DECL

#endif