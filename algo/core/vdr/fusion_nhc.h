/**@file        fusion_nhc.h
 * @brief		fusion nhc header file
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

#ifndef __FUSION_NHC_H__
#define __FUSION_NHC_H__

#include "fusion_proc.h"
#include <stdint.h>

uint8_t nhc_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav);

#endif // !__FUSION_NHC_H__
