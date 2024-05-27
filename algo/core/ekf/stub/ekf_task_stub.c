/**@file        ekf_task_stub.cpp
 * @brief       Differential Carrier Phase 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>chenbin   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "gnss_type.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "ekf_api.h"

/**
 * @brief EKF module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ekf_Init(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief EKF module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ekf_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief EKF module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ekf_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief EKF module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void ekf_Release(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief EKF API for SysErrModel data put
 * @return      None
 */
static BOOL ekf_SysErrModData_Put(ipc_t* p_ipc)
{
  return TRUE;
}


/**
 * @brief Sanity Check IPC 
 * @param[in]   p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL ekf_IpcCheck(ipc_t* p_ipc)
{
  return TRUE;
}

/**
 * @brief EKF IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ekf_Proc(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief EKF IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ekf_task(void* args)
{
  return NULL;
}

/**
 * @brief EKF API for initilization
 * @return      None
 */
BOOL ekf_api_Init()
{
  return TRUE;
}

/**
 * @brief EKF API for start
 * @return      None
 */
BOOL ekf_api_Start()
{
  return TRUE;
}

/**
 * @brief EKF API for stop
 * @return      None
 */
BOOL ekf_api_Stop()
{
  return TRUE;
}

/**
 * @brief EKF API for release
 * @return      None
 */
BOOL ekf_api_Release()
{
  return TRUE;
}

/**
 * @brief EKF API for SysErrModel data put
 * @return      None
 */
BOOL ekf_api_SysErrModData_Put(ins_SysErrModData_t* pz_SysErrModData)
{
  return TRUE;
}
