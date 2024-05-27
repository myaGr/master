/**@file        cmn_CSmask_combine.h
 * @brief       combine the mask of cycle slip
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/02/17  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __CMN_CS_MASK_COMBINE_H__
#define __CMN_CS_MASK_COMBINE_H__
#include "cmn_def.h"

BEGIN_DECL

#define SLIP_FLAG_LLI    (uint8_t)0x1
#define SLIP_FLAG_GF     (uint8_t)0x2
#define SLIP_FLAG_TDCP   (uint8_t)0x4
#define SLIP_FLAG_DOP    (uint8_t)0x8
#define SLIP_FLAG_MW     (uint8_t)0x10
#define SLIP_FLAG_LACK   (uint8_t)0x20 /* lack of detect method */

/**
  * @brief combine the mask of cycle slip
  * @param[in] u_tdcpMethodValid is the field mask the TDCP method whether valid
  * @param[in] u_FreqNum is frequency number for one satellite
  * @param[in] u_slipMask is result of cycle slip detect
  * @param[in] u_LLI LLI
  * @param[in] d_deltaTime the delta time between current epoch and previous epoch
  * @param[in] u_isCurrentCloseSky whether the station located in close sky for current epoch
  * @param[in] u_slipMethodValid is the valid of detecting cycle slip method
  * @param[out] pu_slipFlag the cycle slip type
  * @return 1 represent cycle slip and 0 represent non cycle slip happpen
  */
uint8_t cmn_combineCycleSlipMask(uint8_t u_tdcpMethodValid, uint8_t u_FreqNum, uint8_t u_slipMask,
  uint8_t u_LLI, double d_deltaTime, uint8_t u_isCurrentCloseSky, uint8_t u_slipMethodValid, uint8_t* pu_slipFlag);
END_DECL
#endif