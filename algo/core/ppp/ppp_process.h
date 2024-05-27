/**@file        ppp_process.h
 * @brief       manage the routeline of Precision Point Positioning
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/12  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __PPP_PROCESS_H__
#define __PPP_PROCESS_H__

#include "cmn_def.h"
#include "gnss_type.h"
#include "ppp_type.h"
BEGIN_DECL
/**
 * @brief initilize the PPP algorithm
 * @param[in]  pz_opt represent the algorithm option for the PPP-RTK
 * @param[out] void
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t ppp_Algorithm_Init(const ppp_alg_opt_t* pz_opt);
/**
 * @brief deinitilize the PPP algorithm
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void ppp_Algorithm_Deinit();

/**
 * @brief the inteface of prococess the PPP-RTK algorithm
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_SsrLocBlk is the SSR product
 * @param[out] pz_PositionFix is PPP position Fix report
 * @return     status mask of ppp algorithm
 */
uint32_t ppp_Algorithm_Prcocess(gnss_ssrLosBlock_t* pz_SsrLocBlk, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
                               gnss_PositionFix_t* pz_PositionFix);

END_DECL

#endif