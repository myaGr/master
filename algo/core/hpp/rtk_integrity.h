/**@file        rtk_integrity.h
 * @brief       Calculate PL for RTK FLOAT/FIX
 * @details     
 * @author      houxiaowei
 * @date        2023/5/24
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/5/24   <td>0.1      <td>houxiaowei    <td>Init version
 * <tr><td>2024/2/21   <td>0.2      <td>houxiaowei    <td> delete rtk factor to call 1.0, add DT model
 * </table>
 *
 **********************************************************************************
 */
#ifndef __GNSS_ENGINE_RTK_INTEGRITY_H__
#define __GNSS_ENGINE_RTK_INTEGRITY_H__

#include "integ_type.h"
#include "rtk_type.h"

extern integ_RtkIntegrity_t* rtk_integ_init();

extern void rtk_integ_deinit(integ_RtkIntegrity_t *pz_integ);

/**
 * @brief Intergrity status
 * @param[in,out] pz_integ contains config and rtk factor
 * @return TRUE: ok, False: not ok
 */
extern int32_t rtk_integ_valid(const integ_RtkIntegrity_t *pz_integ);

/**
 * @brief rtk integrity prepare work
 * @param[in,out] pz_integ RTK integrity interface struct
 * @param[in] pz_RTKfilterInfo EKF filter information
 * @param[in] pz_pvt_result  pack from PVT
 * @param[in] pz_satSigMeasCollect
 * @param[in] u_common_sat common satellites between rover station and base station
 */
extern void rtk_integ_start(integ_RtkIntegrity_t* pz_integ,
                            const rtk_filterInfo_t* pz_RTKfilterInfo,
                            const gnss_PVTResult_t* pz_pvt_result,
                            uint8_t u_common_sat);

/**
 * @brief clear integrity information when RTK solution is invalid
 * @param[in,out] pz_integ RTK integrity interface struct
 */
extern void rtk_integ_clearInfo(integ_RtkIntegrity_t* pz_integ);

/**
 * @brief Save omc residual
 * @param[in] z_residualType is the residual type, rtk_ResidualType
 * @param[in] d_lockTime carrier phase lock time
 * @param[in] d_all_cn0 sum of all signal cn0, contain cp and pr
 * @param[in] pu_resNumCodeAry array of different frequency code residual numbers
 * @param[in] pu_resNumPhaseAry array of different frequency phase residual numbers
 * @param[in] u_valid_sat array valid satellite
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in] pz_filterObs is the filter obs information
 * @param[in] pz_fixedAmbPool is the FIXED AMB, could be NULL in float case
 * @param[in,out] pz_integ RTK integrity interface struct
 */
extern void rtk_integ_save_residualPost(rtk_ResidualType z_residualType, double d_lockTime, double d_all_cn0,
                                        const uint8_t *pu_resNumCodeAry, const uint8_t *pu_resNumPhaseAry,
                                        const uint8_t *u_valid_sat, const rtk_filterInfo_t *pz_RTKfilterInfo,
                                        const rtk_EpochFilterObs_t *pz_filterObs,
                                        const gnss_fixedSignalAmbPool_t *pz_fixedAmbPool,
                                        integ_RtkIntegrity_t *pz_integ);


/**
 * @brief Save doppler omc residual
 * @param[in,out] pz_integ RTK integrity interface struct
 * @param[in] d_sigma standard error of unit weight
 * @param[in] u_dr_num  numbers cont for doppler
 */
extern void rtk_integ_save_residualDr(double d_sigma, uint8_t u_dr_num, integ_RtkIntegrity_t* pz_integ);

/**
 * @brief save position engine status, Eg. scene, solution tag, KF count...
 * @param[in] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[in] pz_posSolution poisition struct
 * @param[in,out] pz_integ
 * @return FALSE: filter solution invalid, TRUE: success
 */
extern void rtk_integ_save_statusInfo(const rtk_filterInfo_t* pz_RTKfilterInfo,
                                      const gnss_PositionFix_t* pz_posSolution,
                                      integ_RtkIntegrity_t* pz_integ);

/**
 * @brief save WL feature information
 * @param[in] pz_fixedAmbPool
 * @param[in] u_rtkFilterPosStatus
 * @param[in,out] pz_rtkAmbFixInputInfo
 */
extern void rtk_integ_save_wideLaneInfo(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
                                        uint8_t u_rtkFilterPosStatus,
                                        gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo);

/**
 * @brief Extract STD from EKF filter P matrix
 * @param[in] pz_EKFstateRepPool
 * @param[in] pz_posSolution
 * @param[in,out] pz_RTKfilterInfo
 */
extern void rtk_integ_save_KFSTD(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
                                 const gnss_PositionFix_t* pz_posSolution,
                                 rtk_filterInfo_t* pz_RTKfilterInfo);

/**
 * @brief Log feature
 * @param[in] pz_RTKfilterInfo
 */
extern void rkt_integ_logFeature(const rtk_filterInfo_t* pz_RTKfilterInfo);

/**
 * @brief Calculate STD by Decision Tree
 * @param[in] pz_RTKfilterInfo RTK filter information
 * @param[in,out] pz_posSolution position solution of RTK
 */
extern void rtk_integ_calSTDByDecisionTree(const rtk_filterInfo_t* pz_RTKfilterInfo, gnss_PositionFix_t* pz_posSolution);

#endif //__GNSS_ENGINE_RTK_INTEGRITY_H__
