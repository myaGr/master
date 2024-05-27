/**@file        fusion_tc.h
 * @brief		fusion tight coupling header file
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

#ifndef __FUSION_TC_H__
#define __FUSION_TC_H__

#include <stdint.h>
#include "fusion_proc.h"

#define MEAS_TYPE_PR 0x01
#define MEAS_TYPE_DR 0x02

#define INS_SYS_GPS     0x00                /* navigation system: GPS */
#define INS_SYS_GLO     0x01                /* navigation system: GLONASS */
#define INS_SYS_BDS     0x02                /* navigation system: BeiDou */
#define INS_SYS_GAL     0x03                /* navigation system: Galileo */
#define INS_SYS_QZS     0x04                /* navigation system: QZSS */

uint8_t tight_couple_psr_precheck(AsensingGNSSPsrMeas_t* pz_psr_paras, InertialNav_t* pz_inav);
uint8_t tight_couple_psr_procedure(InertialNav_t* pz_inav, NavParams_t* pz_para);
uint8_t tight_couple_cp_precheck(AsensingGNSSCpMeas_t* pz_cp_paras, InertialNav_t* pz_inav);
uint8_t tight_couple_cp_procedure(InertialNav_t* pz_inav, NavParams_t* pz_para);

#endif
