/**@file        ppp_task_stub.c
 * @brief       Stub functions for Precision Point Positioning module
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/06/05  <td>0.1      <td>zhanglei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "ppp_api.h"


/**
 * @brief PPP IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_Proc(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief PPP IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ppp_task(void* args)
{
  return NULL;
}

/**
 * @brief PPP API for initilization
 * @return      None
 */
BOOL ppp_api_Init()
{
  return TRUE;
}

/**
 * @brief PPP API for start
 * @return      None
 */
BOOL ppp_api_Start()
{
  return TRUE;
}

/**
 * @brief PPP API for stop
 * @return      None
 */
BOOL ppp_api_Stop()
{
  return TRUE;
}

/**
 * @brief PPP API for release
 * @return      None
 */
BOOL ppp_api_Release()
{
  return TRUE;
}

/**
 * @brief PPP API to Put Receiver Measurement Block from SM or other task
 * @param[in]   meas - Raw GNSS Measurement
 * @return None
 */
void ppp_api_RcvMeasBlk_Put(GnssMeasBlock_t* meas)
{
  return;
}

/**
 * @brief PPP API to Put SSR Loc Block Data
 * @param[in]   pz_ssrLocBlk - SSR Loc Block Data
 * @return      None
 */
void ppp_api_SsrLocBlock_Put(gnss_ssrLosBlock_t* pz_SsrLocBlk)
{
  return;
}

/**
 * @brief PPP API to Put tight satellite signal measurement collect
          and convert it as a sparse satellite signal measurement
          collect stored in PPP
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_TightSatSigMeasCollect_Put(gnss_TightSatSigMeasCollect_t* pz_TightSatSigMeasCollect)
{
  return;
}

/**
 * @brief PPP API to start run algorithm
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_Algorithm_Start()
{
  return;
}

/**
 * @brief PPP API to inject Qianxun SSR stream
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_SsrQianXunStream_Put(uint8_t* data, uint32_t len)
{
  return;
}

/**
 * @brief PPP API to inject AG SSR Los data
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ppp_api_SsrAgLos_Put(uint8_t* data, uint32_t len)
{
  return;
}
