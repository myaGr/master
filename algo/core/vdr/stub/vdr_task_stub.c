/**@file        vdr_task_stub.c
 * @brief       Stub functions for vdr module
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/06/06  <td>0.1      <td>zhanglei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "gnss_type.h"
#include "vdr_api.h"

/**
 * @brief VDR module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void vdr_Init(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief VDR module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void vdr_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief VDR module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void vdr_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief VDR module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void vdr_Release(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief VDR API for IMU data put
 * @return      None
 */
static BOOL vdr_ImuData_Put(ipc_t* p_ipc)
{
  return TRUE;
}

/**
 * @brief VDR API for GNSS fix put
 * @return      None
 */
static BOOL vdr_GnssFix_Put(ipc_t* p_ipc)
{
  return TRUE;
}

/**
 * @brief Sanity Check IPC 
 * @param[in]   p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL vdr_IpcCheck(ipc_t* p_ipc)
{
  return TRUE;
}

/**
 * @brief VDR IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void vdr_Proc(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief VDR IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* vdr_task(void* args)
{
  return NULL;
}

/**
 * @brief VDR API for initilization
 * @return      None
 */
BOOL vdr_api_Init()
{
  return TRUE;
}

/**
 * @brief VDR API for start
 * @return      None
 */
BOOL vdr_api_Start()
{
  return TRUE;
}

/**
 * @brief VDR API for stop
 * @return      None
 */
BOOL vdr_api_Stop()
{
  return TRUE;
}

/**
 * @brief VDR API for release
 * @return      None
 */
BOOL vdr_api_Release()
{
  return TRUE;
}

/**
 * @brief VDR API for IMU data put
 * @return      None
 */
BOOL vdr_api_ImuData_Put(ins_ImuData_t* pz_ImuData)
{
  return TRUE;
}

/**
 * @brief VDR API for GNSS fix put
 * @return      None
 */
BOOL vdr_api_GnssFix_Put(ins_GnssFix_t* pz_GnssFix)
{
  return TRUE;
}

/**
 * @brief VDR API for Wheel data put
 * @return      None
 */
BOOL vdr_api_WheelData_Put(ins_WheelData_t* pz_WheelData)
{
  return TRUE;
}

/**
 * @brief VDR API to GNSS feedback INS with measurement block
 * @param[in]   pz_GnssFeedbackInsMeasBlock - the measure blcok that GNSS feedback to INS
 * @return      None
 */
BOOL vdr_api_GnssInsMeasBlock_Put(gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock)
{
  return TRUE;
}

