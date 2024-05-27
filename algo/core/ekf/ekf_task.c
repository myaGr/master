/**@file        ekf_task.cpp
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
#include "vdr_api.h"
#include "fusion_kf.h"

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
  uint8_t ret = 0;
  ins_SysErrModData_t* p_SysErrModData = NULL;
  if (sizeof(ins_SysErrModData_t) != p_ipc->q_length)
  {
    return FALSE;
  }
  p_SysErrModData = (ins_SysErrModData_t*)p_ipc->p_data;
  ret = kf_update(p_SysErrModData->f_z_meas, p_SysErrModData->u_sysDim, p_SysErrModData->u_measDim, &(p_SysErrModData->sys_err_model));
  p_SysErrModData->update_flag = ret;
  vdr_api_EkfResult_Put(p_SysErrModData);
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
  if (TASK_INDEX_EKF != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_EKF_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_EKF_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief EKF IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void ekf_Proc(ipc_t* p_ipc)
{
  if (FALSE == ekf_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_EKF_INIT:
    ekf_Init(p_ipc);
    break;
  case C_M_EKF_START:
    ekf_Start(p_ipc);
    break;
  case C_M_EKF_STOP:
    ekf_Stop(p_ipc);
    break;
  case C_M_EKF_RELEASE:
    ekf_Release(p_ipc);
    break;
  case C_M_EKF_SEM_DATA_PUT:
    ekf_SysErrModData_Put(p_ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief EKF IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* ekf_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_EKF;

  ipctask_t* t = ipctask_GetInstance(taskIndex);
  t->e_status = TASK_STATUS_ENABLE;
  ipctask_SetThreadId2TaskId(get_thread_id(), taskIndex);

  if(NULL != t->float_hard_enable){
    t->float_hard_enable();
  }

  while (t->e_status == TASK_STATUS_ENABLE)
  {
    if (ipctask_ReceiveMessage(taskIndex, &z_ipc))
    {
      ekf_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief EKF API for initilization
 * @return      None
 */
BOOL ekf_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_EKF, C_M_EKF_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief EKF API for start
 * @return      None
 */
BOOL ekf_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_EKF, C_M_EKF_START, NULL, 0);
  return TRUE;
}

/**
 * @brief EKF API for stop
 * @return      None
 */
BOOL ekf_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_EKF, C_M_EKF_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief EKF API for release
 * @return      None
 */
BOOL ekf_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_EKF, C_M_EKF_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief EKF API for SysErrModel data put
 * @return      None
 */
BOOL ekf_api_SysErrModData_Put(ins_SysErrModData_t* pz_SysErrModData)
{
  pz_SysErrModData->u_version = VERSION_INS_SEM_DATA;
  ipctask_SendMessage(TASK_INDEX_EKF, C_M_EKF_SEM_DATA_PUT,
    (uint8_t*)pz_SysErrModData, sizeof(ins_SysErrModData_t));
  return TRUE;
}

