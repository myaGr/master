/**********************************************************************************
* @note
* @par History :
*<table>
* <tr><th>Date        <th>Version  <th>Author      <th>Description
* <tr> < td>2022 / 07 / 28 < td > 0.1 < td > chenjinhe   <td>Init version
* < / table>
*
**********************************************************************************
*/
#ifndef __PPP_FILTER_SOL_H__
#define __PPP_FILTER_SOL_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "ppp_type.h"
BEGIN_DECL
/**
 * @brief the inteface of PPP-RTK float algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[out]     pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out]     pz_ssrStatus is status of ssr correction
 * @return         0 represent success and other failed
 */
uint8_t PPP_filterSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
                         const gnss_ssrLosBlock_t* pz_SsrLocBlk, ppp_filterInfo_t* pz_PPPfilterInfo,
                         gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, ppp_EpochFilterObs_t* pz_FilterObs);
/**
 * @brief computing residual of  the zero-combine code observations
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in] pz_fixedAmbPool is the information of amb fix
 * @param[out] pz_FilterObs is the status of ssr correction
 * @param[in] z_residualType is the type of residual
 * @return    the number of updated observations
 */
uint32_t PPP_zeroCombineCodeMeasResidual(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_filterInfo_t* pz_PPPfilterInfo, const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, ppp_EpochFilterObs_t* pz_FilterObs, PPP_ResidualType z_residualType);
/**
 * @brief computing residual of zero-combine phase observations
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[in] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in] pz_fixedAmbPool is the information of amb fix
 * @param[out] pz_FilterObs is the information of observation after filter
 * @param[in] z_residualType is the type of residual
 * @return    the number of updated observations
 */
uint32_t PPP_zeroCombinePhaseMeasResidual(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const gnss_ssrLosBlock_t* pz_SsrLocBlk, const ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool
  , gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, ppp_EpochFilterObs_t* pz_FilterObs, PPP_ResidualType z_residualType);
END_DECL
#endif