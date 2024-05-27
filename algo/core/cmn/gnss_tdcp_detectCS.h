/**@file        gnss_tdcp_detect_cs.h
 * @brief       Detect cycle slip module
 * @details
 * @author      chenjinhe
 * @date        2022/09/14
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/26  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __GNSS_TDCP_DETECT_CS_H__
#define __GNSS_TDCP_DETECT_CS_H__

#include "cmn_def.h"
#include "gnss_type.h"

BEGIN_DECL

typedef struct
{
  gnss_TdcpMeasBlock_t*       pz_preTdcpMeas;//the pointer of previous epoch TDCP observations
  gnss_TdcpMeasBlock_t*       pz_curTdcpMeas;//the pointer of current epoch TDCP observations
}gnss_TdcpMeasPair_t;//the pointer pair of previous epoch and current epoch to TDCP observations

/**
 * @brief convert the observation information to TDCP observations
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_curDcpMeas is the current epoch TDCP observations
 * @return the number of TDCP measure
 */
uint16_t cmn_convertSigMeasToDcpStructNonSSR(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_TdcpMeasBlock_t* pz_curDcpMeas);

/**
 * @brief convert the observation information to TDCP observations
 * @param[in] pz_SsrLocBlk is the SSR product
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_curDcpMeas is the current epoch TDCP observations
 * @return the number of TDCP measure
 */
uint16_t cmn_convertSigMeasToDcpStruct(const gnss_ssrLosBlock_t* pz_SsrLocBlk, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_TdcpMeasBlock_t* pz_curDcpMeas);

/**
 * @brief using TDCP method to detect cycle slip
 * @param[in]     z_tdcpMeasPointerPair is the pointer pair of previous epoch and current epoch to TDCP observations
 * @param[in/out] pz_satSigMeasCollect is the observation information
 * @return 1 represent successful and 0 represent failure
 */
uint8_t cmn_detectCycleSlipByTDCP(gnss_TdcpMeasPair_t z_tdcpMeasPointerPair, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

END_DECL

#endif