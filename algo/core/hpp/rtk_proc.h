/**@file        rtk_proc.h
 * @brief       Real-time kinematic(RTK) Main Process module
 * @details
 * @author      caizhijie
 * @date        2022/06/21
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __RTK_PROC_H__
#define __RTK_PROC_H__

#include "cmn_def.h"
#include "rtk_type.h"

BEGIN_DECL

/**
 * @brief Initialze RTK module
 * @param[in]  pz_opt represent the algorithm option for the RTK
 * @param[in]  f_sample background process sample
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t rtk_init(const rtk_alg_opt_t* pz_opt, float f_sample);

/**
 * @brief deinitilize the RTK algorithm
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void rtk_deInit();

 /**
 *@brief load the default option for the RTK algorithm
 * @param[out]  pz_opt represent the algorithm optimization for the RTK
 * @return     void
 */
 void RTK_loadDefaultOption(rtk_alg_opt_t* pz_opt);
/**
 * @brief update the Ref correct data fit Ref position or not
 * @param[in]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[out] void
 * @return     TRUE for update success; FALSE for other case
 */
BOOL rtk_corrUpdate(const GnssCorrBlock_t * pz_GnssCorrBlockUpdate);

/**
 * @brief RTK Main Process
 * @param[in]      pz_pvt_result is information of pvt
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in/out]  pz_satSigMeasCollectPre is the observation information of previous epoch
 * @param[out]     pz_PositionFix is RTK position Fix report
 * @return         0 represent success and other failure
 */
BOOL hpp_Rtk_Process(const gnss_PVTResult_t* pz_pvt_result, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_SatSigMeasCollect_t* pz_satSigMeasCollectPre, gnss_PositionFix_t* pz_PositionFix);

END_DECL

#endif