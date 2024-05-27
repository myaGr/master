/**@file        gnss_prep.h
 * @brief       previous process for data quality check
 * @details     cycle slip, clock jump repaire
 * @author      houxiaowei
 * @date        2022/5/27
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/5/27   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __PREPROCESS__
#define __PREPROCESS__

#include "gnss_type.h"


/**
 * @brief preprocess: cycle-slip, clock jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_CheckSatSignalQuality(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur);

/**
 * @brief empirical value of troposphere, save to satsigmeas
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_UpdateTroposphericModel(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief empirical value of ionosphere, save to satsigmeas
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_UpdateIonosphericModel(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief update satellite elevation, use pz_satSigMeasCollect->z_posSolution.xyz
 * @param[in/out] pz_satSigMeasCollect
 */
void pp_UpdateSatEleAzi(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/** 
 * @brief clock jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_ClkJumpRepaire(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur);

/**
 * @brief GF, detect cycle slip by geometry free phase jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_DetectSlipGF(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur);

/**
 * @brief MW, detect slip by Melbourne-Wubbena linear combination jump
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_DetectSlipMW(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur);

/**
 * @brief DOPPLER, detect slip by doppler
 * @param[in] sat_signal_pre
 * @param[in,out] sat_signal_cur
 */
void pp_DetectSlipDoppler(const gnss_SatSigMeasCollect_t* sat_signal_pre, gnss_SatSigMeasCollect_t* sat_signal_cur);

#endif // !__PREPROCESS__
