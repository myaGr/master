/**@file        fusion_alignment.h
 * @brief		fusion alignment file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_ALIGNMENT_H__
#define __FUSION_ALIGNMENT_H__

#define INS_ALIGN_VEL (49.f)
#define CONTINUE_BDS_DT (1.5)
#include <stdint.h>
#include "fusion_api.h"

typedef struct
{
  double d_prev_bdstime;
  double d_imu_timestamp;
  float  f_avg_speed;
  float  f_odospeed;
  uint8_t u_align_status;
}InitialAlign_t;

InitialAlign_t* fusion_get_align();
uint8_t align_with_sigant(double pd_meas[], uint8_t u_flag);
uint8_t align_with_dualant(double pd_meas[], uint8_t u_flag);
void align_with_history(HistoryInsResults_t* pz_history, double* pd_meas);
void align_procedure(double pd_meas[]);
#endif // !__FUSION_ALIGNMENT_H__
