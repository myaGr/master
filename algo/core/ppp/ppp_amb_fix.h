/**@file        ppp_amb_fix.h
 * @brief       integer ambiguity resolution
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/09/21  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __PPP_AMB_FIX_H__
#define __PPP_AMB_FIX_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "ppp_type.h"
BEGIN_DECL
/**
 * @brief wide-lane ambiguity resolution for ppp
 * @param[in]    pz_pppAmbFixInputInfo - input information for ppp-ar
 * @param[out]   pz_fixedAmbPool - result of ppp-ar
 * @param[out]   z_freqType - common frequency for narrow-lane amb fix
 * @return status -   1: pppar success, 0: pppar fail
 */
  extern uint8_t ppp_wideLaneAmbFix(gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
	gnss_fixedSignalAmbPool_t* pz_fixedAmbPool, gnss_FreqType* z_freqType);

/**
 * @brief narrow-lane ambiguity resolution for ppp
 * @param[in]    z_freqType - frequency type of ppp-ar
 * @param[in]    pz_pppAmbFixInputInfo - input information for ppp-ar
 * @param[out]   pz_fixedAmbPool - result of ppp-ar
 * @return  status   --   0: pppar success, 1: pppar fail
 */
extern uint8_t ppp_zeroCombineAmbFix(gnss_FreqType z_freqType, gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo,
	gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief PPP ambiguity resolve
 * @param[in/out]  pz_pppAmbFixInputInfo is input information for pppar
 * @param[out]  pz_fixedAmbPool is result of amb resolution
 * @return    status of PPP-AR
 */
extern uint8_t PPP_AmbResolution(gnss_pppAmbFixInputInfo_t* pz_pppAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);
END_DECL
#endif