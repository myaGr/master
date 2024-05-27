/**@file        rtk_corr_check.h
 * @brief       check the consistency of observations and coordinate
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/02/20  <td>0.1      <td>            <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __RTK_CORR_CHECK_H__
#define __RTK_CORR_CHECK_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "rtk_type.h"
#include "cmn_turboEdit.h"

BEGIN_DECL
/**
 * @brief detect the slip of RTK VRS observations
 * @param[in]       pz_RTKcorrBlockCur is the current RTK VRS observations and coordinate
 * @param[in/out]   pz_gfFilterSet is the filter information used by GF method
 * @param[out]      pz_RTKcorrSlipFlag saves Detect result
 * @return 0 for success ,others for error case
 */
void rtk_corrDetectCycleSlip(cmn_turboEditFilterSet* pz_gfFilterSet, GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag,
  const GnssCorrBlock_t* pz_RTKcorrBlockCur, const GnssCorrBlock_t* pz_RTKcorrBlockPre);

/**
 * @brief check the Ref correct data availability
 * @param[in]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[out] void
 * @return     true for data is valid; false for other case
 */
BOOL rtk_checkCorrValid(const GnssCorrBlock_t * pz_GnssCorrBlockUpdate);

/**
 * @brief check the Ref correct data fit Ref position or not
 * @param[in]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[in]  pd_refpos2Check represent Ref position which need to be checked
 * @param[out] void
 * @return     true for data fit position; false for other case
 */
BOOL rtk_checkCorrPos(GnssCorrBlock_t * pz_GnssCorrBlockUpdate, const double pd_refpos2Check[3]);

/**
 * @brief      while ref station changes, check amb offset and update it
 * @param[in/out]  pz_rtkCorrBlock[2] is the OSR correction
 * @param[in]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[out]     pz_RTKcorrSlipFlag is Correction satellite SlipFlag 
 * @param[out]     pz_preFixedAmbPool is the pool of prefix info
 * @return     REF_AMB_CHANGE_POSTPONE = 2  for it's not a good time to update amb, Postpone the update
               REF_AMB_CHANGE_SUCC     = 1  for update success or no need to update;
               REF_AMB_CHANGE_ERROR    = 0  for error case, need to reset AMB;
 */
uint8_t rtk_refChangeCheckAndUpdateAmb(GnssCorrBlock_t* pz_RTKcorrBlock[2], rtk_filterInfo_t* pz_RTKfilterInfo,
  gnss_EKFstateRepresentPool_t* pz_rtkEKFstateRepPool, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  GnssCorrSlipFlag_t* pz_RTKcorrSlipFlag, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool);

END_DECL
#endif
