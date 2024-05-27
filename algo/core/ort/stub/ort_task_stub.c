/**@file        ort_task.c
 * @brief       Double Antenna Orient Calculate Task
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

#include "ort_api.h"

/**
 * @brief ORT IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ort_Proc(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief ORT IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ort_task(void* args)
{
  return NULL;
}

/**
 * @brief ORT API for initilization
 * @return      None
 */
BOOL ort_api_Init()
{
  return TRUE;
}

/**
 * @brief ORT API for start
 * @return      None
 */
BOOL ort_api_Start()
{
  return TRUE;
}

/**
 * @brief ORT API for stop
 * @return      None
 */
BOOL ort_api_Stop()
{
  return TRUE;
}

/**
 * @brief ORT API for release
 * @return      None
 */
BOOL ort_api_Release()
{
  return TRUE;
}

/**
 * @brief ORT API to Put Rcv Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ort_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas)
{
  return;
}

/**
 * @brief ORT API to Put Orient Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ort_api_OrtMeasBlk_Put(GnssMeasBlock_t* meas)
{
  return;
}
void ort_api_mainAntennaPVTinfo_put(gnss_PVTResult_t* pz_pvtInfo)
{
	return;
}
