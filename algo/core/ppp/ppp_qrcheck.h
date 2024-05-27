/**@file        ppp_qrcheck.h
 * @brief       Preprocessing pseudo-range using QR parity check
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/09/11  <td>0.1      <td>yuezhu      <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef _PPP_QR_CHECH_H_
#define _PPP_QR_CHECH_H_
#include "cmn_def.h"
#include "cmn_utils.h"
#include "gnss_type.h"
#include "ppp_type.h"
BEGIN_DECL

#define SOL_DIFF_INVALID_VALUE  (999.9)
#define SOL_CONSISTENCE_THRES   (20.0)


/**
 * @brief the interface of QR parity check algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_SsrLocBlk is the SSR product
 * @param[out]     pz_PPPfilterInfo the filter for the PPP algorithm
 * @param[in]      pz_EKFstateRepPool is the pool of EKF state represent
 * @return         0 represent successs and other failed
 */
int32_t ppp_QRcheck(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_ssrLosBlock_t* pz_SsrLocBlk,
  ppp_filterInfo_t* pz_PPPfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool);

END_DECL
#endif