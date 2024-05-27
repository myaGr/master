/**@file        ppp_integrity.h
 * @brief       Calculate PL for PPP FLOAT/FIX
 * @details     
 * @author      houxiaowei
 * @date        2023/2/23
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/2/23   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __GNSS_ENGINE_PPP_INTEGRITY_H__
#define __GNSS_ENGINE_PPP_INTEGRITY_H__

#include "integ_type.h"
#include "gnss_def.h"
#include "gnss_type.h"
#include "seq_kalman.h"
#include "ppp_type.h"

void ppp_IntegInit();

void ppp_IntegDeInit();

void ppp_IntegClear(const GpsTime_t* gpst);

/**
 * @brief Get integrity, get protection level by "cov" or "filter" or "fusing"
 * @param[in/out] pz_pl protection level result
 */
void ppp_IntegFillUpIntegrity(gnss_PositionFix_t* pz_posSolution, integ_PLResult_t* pz_pl);

/**
 * @brief Get integrity type config
 * @return
 */
integ_FunType ppp_IntegGetIntegrityType();

/**
 * @brief Intergrity status
 * @return TRUE: ok, False: not ok
 */
BOOL ppp_IntegValid();

/**
 * @brief Set position engine status
 * @param[in] u_fix_flag
 */
void ppp_IntegSetPosStatus(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FixFlagType u_fix_flag);

/**
 * @brief get the num of observation whos CN0 > 40
 * @param[in] gnss_SatSigMeasCollect_t* pz_satSigMeasCollect
 */
void ppp_IntegSetCN040(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief get the num of bit 1 from an uint64_t integer
 * @param[in] uint64_t integer
 */
uint8_t ppp_IntegCountSetBits(uint64_t u_n);

/**
 * @brief Set position rotation matrix
 * @param[in] d_pos LLA, {lat,lon,height}
 */
void ppp_IntegSetPosAndPosRotation(const double d_pos[3]);

/**
 * @brief Save filter covariance matrix
 * @param[in] pz_PPPfilterInfo float filter info
 * @param[in] pz_fixedAmbPool fix filter info
 * @param[in] pz_EKFstateRepPool ekf state pool
 */
void ppp_IntegSaveFilterPositionMatrix(const ppp_filterInfo_t* pz_PPPfilterInfo,
                                       const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
                                       const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool);


/*----------------------------------------------------------------------------*/
/* Integrity -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief Calculate protection level by filter covariance, factor is ppf(1e-5)
 * @param[in/out] pz_pl protection level result
 */
void ppp_IntegCalPlByFilter(gnss_PositionFix_t* pz_posSolution, integ_PLResult_t* pz_pl);


/*----------------------------------------------------------------------------*/
/* Raim ----------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief Calculate protection level by RAIM, need to optimize.
 * @param[in,out] pz_int_pl protection level result
 */
void ppp_IntegCalPLByRAIM(integ_PLResult_t* pz_int_pl);


/**
 * @brief Extract information from sequential EKF
 * @param[in] z_freqType
 * @param[in] pz_seqVar
 * @param[in] pw_PVAindex
 * @param[in,out] pz_si signal information
 */
void ppp_IntegExtractInfoFromSeqKalmanFilter(gnss_FreqType z_freqType, const SeqKalmanVar_t* pz_seqVar,
                                             const int16_t pw_PVAindex[9], integ_SignalInfo_t* pz_si);

/**
 * @brief Save signal information that needed EKF variable
 * @param[in] u_constellation
 * @param[in] u_svid
 * @param[in] z_freqType
 * @param[in] pz_si signal info
 */
void ppp_IntegAddSignalInfo(gnss_ConstellationType u_constellation, uint8_t u_svid, gnss_FreqType z_freqType,
                            const integ_SignalInfo_t* pz_si);

/**
 * @brief Add observation residual of after EKF
 * @param[in] b_fixed
 * @param[in] u_constellation
 * @param[in] u_svid
 * @param[in] z_freqType
 * @param[in] d_res
 * note: must call this function after ppp_IntegAddSignalInfo
 */
void ppp_IntegAddObsRes(BOOL b_fixed, gnss_ConstellationType u_constellation, uint8_t u_svid, gnss_FreqType z_freqType,
                        double d_res);

/*----------------------------------------------------------------------------*/
/*  DataSet output for machine learning --------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief Machine Learning DLOG_GNSS_PL
 */
void log_MachineLearingData();

/**
 *@brief add the sky scene to the pppFeature
 * @param[in] pz_filterInfo
 */
void ppp_IntegSetScene(const ppp_filterInfo_t* pz_filterInfo);

/**
 *@brief add feature from pz_satSigMeasCollect for the scene determined by machine learning
 * @param[in] pz_satSigMeasCollect
 */
void ppp_IntegSetMLScene(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 *@brief add the feature from gz_pppAmbFixInputInfo/pz_fixedAmbPool/pz_PositionFix to the pppFeature
 * @param[in] pz_pppAmbFixInputInfo
 * @param[in] pz_fixedAmbPool
 * @param[in] pz_PositionFix
 */
void ppp_IntegSetOtherFeature(const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, 
                              const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
                              const gnss_PositionFix_t* pz_PositionFix);

/**
 * @brief Calculate DOPS by filter the invalid satellites
 * @param[in] pz_SatSigMeasCollect
 * @param[in] pz_filterObs
 * TODO(houxiaowei): DOPS have defined in gnss_common.h, reuse and call the function
 */
void ppp_IntegCalSetDOPS(const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, const ppp_EpochFilterObs_t* pz_filterObs);

/**
 * @brief Fill up Machine Learning date feature information
 * @param[in] pz_satSigMeasCollect satellite observation
 * @param[in] pz_PositionFix position info
 * @param[in] pz_filterInfo float filter info
 * @param[in] pz_EKFstateRepPool EKF state pool
 * @param[in] pz_filterObs  information of observation after filter
 * @param[in] pz_pppAmbFixInputInfo
 */
void ppp_IntegFillMlFeature(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_PositionFix_t* pz_PositionFix,
                            const ppp_filterInfo_t* pz_filterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
                            const ppp_EpochFilterObs_t* pz_filterObs, const gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
                            const gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);


/**
 * @brief Calculate protection level by Decision Tree
 * @param[in,out] pz_pl protection level result
 */
void ppp_IntegCalPLByDecisionTree(gnss_PositionFix_t* pz_PositionFix, integ_PLResult_t* pz_pl);

/*----------------------------------------------------------------------------*/
/*  Tool functions --------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief get the num of bit 1 from an uint64_t integer
 * @param[in] uint64_t integer
 */
uint8_t ppp_IntegCountSetBits(uint64_t t_n);


#endif //__GNSS_ENGINE_PPP_INTEGRITY_H__
