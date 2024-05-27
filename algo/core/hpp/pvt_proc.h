/**@file        pvt_proc.h
 * @brief       GNSS Position/Velocity/Time Main Process module
 * @details
 * @author      caizhijie
 * @date        2022/06/13
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/13  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __PVT_PROC_H__
#define __PVT_PROC_H__

#include "cmn_def.h"
#include "gnss_type.h"

BEGIN_DECL

/**
 * @brief Initialze PVT module
 * @return      None
 */
void pvt_init();

/**
 * @brief Deinitialze PVT module
 * @return      None
 */
void pvt_deinit();

/**
 * @brief Position Velocity Time Main Process
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void hpp_Pvt_Process(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect);

END_DECL

#endif
