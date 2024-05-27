/**@file        rtk_pre_fix.h
 * @brief       pre fix solution for the RTK algorithm
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/03/13  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __RTK_PRE_FIX_H__
#define __RTK_PRE_FIX_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "rtk_type.h"
#include "gnss_common.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "seq_kalman.h"
#include "gnss_filter_type.h"
#include "cmn_utils.h"

BEGIN_DECL


/**
 * @brief          RTK prefix solution availability check
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         void
 */
  void rtk_preFixSol_availabilityCheck(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo,
    gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, uint16_t w_nv);

/**
 * @brief the inteface of RTK prefix algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @return         void
 */
  uint8_t RTK_preFixSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
    rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
    gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs);

  /**
 * @brief the interface of clean prefix reject label
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @return         void
 */
  void RTK_preFixCleanRejectLabel(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief the interface of RTK prefix algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @return         void
 */
void RTK_preFixHalfCycleDetect(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool);

/**
 * @brief the interface of RTK prefix amb calculate
 * @param[in]     u_nv is number of prefix satellite
 * @param[in]     pz_satSigMeasCollect is the observation information
 * @param[in]     pz_rtkCorrBlock is the OSR correction
 * @param[in]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]     pz_preFixedAmbPool is the pool of prefix info
 * @return         void
 */
void rtk_PreFixAmbCalculate(uint8_t u_nv, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool);

END_DECL
#endif