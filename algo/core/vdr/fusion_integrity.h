/**@file        fusion_integrity.h
 * @brief		fusion integrity header file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_INTEGRITY_H__
#define __FUSION_INTEGRITY_H__

#include <stdint.h>
#include "fusion_err_model.h"
#include "fusion_api.h"

#define OPEN_INS_INTEGRITY

typedef struct
{
  float   f_delta_pl[9];
  float   f_pl_pre[9];
  float   f_measnoise[9];
  double  d_last_badbds_diverg_time; 
  float   f_pl_diff[9]; 
  double  d_last_pldiff_cal_time; 
  uint8_t u_outage_closely_follow_badbds; 
  float   f_dz_use[9];
  uint8_t u_last_posflag;
  double  d_last_gnssposmeas_time;
  uint8_t u_badfix;
} PLCalParas_t;

void fusion_protection_level_calculation(SysErrModel_t* pz_model, INSResults_t* pz_ins);
void fusion_pl_para_init(void);

#endif // !__FUSION_INTEGRITY_H__