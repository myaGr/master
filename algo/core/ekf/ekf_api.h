/**@file        ekf_api.h
 * @brief       Vehicle Dead Reckoning
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>chenbin     <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __EKF_API_H__
#define __EKF_API_H__

#include "mw_ipctask.h"
#include "ekf_task.h"

BEGIN_DECL

/**
 * @brief EKF IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ekf_Proc(ipc_t* p_ipc);


/**
 * @brief EKF IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ekf_task(void* args);

/**
 * @brief EKF API for initilization
 * @return  None
 */
BOOL ekf_api_Init();

/**
 * @brief EKF API for start
 * @return  None
 */
BOOL ekf_api_Start();

/**
 * @brief EKF API for stop
 * @return  None
 */
BOOL ekf_api_Stop();

/**
 * @brief EKF API for release
 * @return  None
 */
BOOL ekf_api_Release();

/**
 * @brief EKF API for SysErrModel data put
 * @return  None
 */
BOOL ekf_api_SysErrModData_Put(ins_SysErrModData_t* pz_SysErrModData);


END_DECL

#endif