/**@file        rtk_pos_check.h
 * @brief       check the consistency of 1. rtk history & current pos 2. filter result & spp result
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
#ifndef __RTK_POS_CHECK_H__
#define __RTK_POS_CHECK_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "rtk_type.h"

BEGIN_DECL

/**
 * @brief              init history pos structure
 * @param[in/out]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @return             void
 */
void rtk_initHistoryPos(rtk_history_pos_t* pz_historyPos_t);

/**
 * @brief              push the filter pos in history pos structure
 * @param[in/out]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]          u_solTag = RTK_FILTER_POS_... the pushed filter sol type
 * @param[in]          z_tor time of pushed filter pos
 * @param[in]          u_diffFlagWithSPP flag of whether rtk sol has significant difference with spp
 * @param[in]          pz_posSPP result of spp
 * @return             TRUE for success ,FALSE for error case
 */
BOOL rtk_pushHistoryPos(rtk_filterInfo_t* pz_RTKfilterInfo, const uint16_t u_solTag,
    const GpsTime_t z_tor, const uint32_t u_diffFlagWithSPP, const gnss_PositionFix_t* pz_posSPP);

/**
  * @brief              get flag of whether current sol has significant difference with history predict one
  * @param[in]          pz_filterSol save position result of every steps in rtk process
  * @param[in]          u_solType see RTK_FILTER_POS_...
  * @return             RTK_DIFF_BETWEEN_POS_NONE for no diff or unknowen, RTK_DIFF_BETWEEN_POS_SMALL/LARGE for small/large diff
  */
uint8_t rtk_getDiffFlagBewteen_history_current(const rtk_filter_sol_t* pz_filterSol, uint8_t u_solType);

/**
 * @brief              get flag of whether rtk sol has significant difference with spp
 * @param[in]          rtk_filter_sol_t save position result of every steps in rtk process 
 * @param[in]          u_diffFlagWithSPP flag of whether rtk sol has significant difference with spp
 * @return             RTK_DIFF_BETWEEN_POS_NONE for no diff or unknowen, RTK_DIFF_BETWEEN_POS_SMALL/LARGE for small/large diff
 */
uint8_t rtk_getDiffFlagBewteen_spp_rtk(const rtk_filter_sol_t* pz_filterSol,
  const gnss_PositionFix_t* pz_positionFix, uint8_t u_solType);

/**
 * @brief              get Predict Pos By History Pos
 * @param[in/out]      pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in]          z_tor time of current epoch
 * @return             TRUE for success ,FALSE for error case
 */
BOOL rtk_getPredictPosByHistoryPos(rtk_filterInfo_t* pz_filterInfo, const GpsTime_t z_tor);

/**
 * @brief fill RTK position information
 * @param[in] pz_satSigMeasCollect is the observation information
 * @param[in/out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in] pz_fixedAmbPool input information for RTK ambiguity resolution
 * @param[out] pz_posSolution position struct contains pos/vel and unc of pos/vel
 * @param[out] pz_preFixedAmbPool prefixed ambiguity pool
 * return
 */
void rtk_CreatePosSolution(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, rtk_filterInfo_t* pz_RTKfilterInfo,
  const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_PositionFix_t* pz_posSolution, gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool);

END_DECL
#endif
