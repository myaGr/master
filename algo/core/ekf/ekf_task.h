/**@file        ekf_task.h
 * @brief       Differential Carrier Phase 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/28  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __EKF_TASK_H__
#define __EKF_TASK_H__

#include "cmn_def.h"
#include "fusion_err_model.h"


BEGIN_DECL

#define VERSION_INS_SEM_DATA  (0)

typedef struct
{
  uint8_t   u_version;
  uint8_t   update_flag;
  SysErrModel_t sys_err_model;
  float *f_z_meas;
  uint8_t u_sysDim;
  uint8_t u_measDim;
} ins_SysErrModData_t;

END_DECL

#endif
