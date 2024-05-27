/**@file        ppp_task.h
 * @brief       Precision Point Positioning 
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

#ifndef __PPP_TASK_H__
#define __PPP_TASK_H__

#include "cmn_def.h"

BEGIN_DECL

/**
 * @brief Get current in use Satellite Signal Measurement Collect
 * @return The pointer to current structure
 */
gnss_SatSigMeasCollect_t* ppp_GetCurrentSatSigMeasCollect();

/**
 * @brief Get previous in use Satellite Signal Measurement Collect
 * @return The pointer to previous structure
 */
gnss_SatSigMeasCollect_t* ppp_GetPreviousSatSigMeasCollect();

END_DECL

#endif
