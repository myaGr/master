/**@file        dcp_ort_sol.h
 * @brief       solute the real-time orient using the carrier-phase, doppler or pseudo-range observations
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/08  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __DCP_ORT_SOL_H__
#define __DCP_ORT_SOL_H__
#include "cmn_def.h"
#include "gnss_type.h"
BEGIN_DECL

typedef enum 
{
  ORT_DCP_SUCC=0,
  ORT_DCP_MEM_FAIL,
  ORT_DCP_TIME_REPEAT,
  ORT_DCP_GET_MAIN_ANT_OBS_FAIL,
  ORT_DCP_GET_AUXI_ANT_OBS_FAIL,
}ort_dcpSolStatus_t;
typedef uint8_t ort_dcpSolStatus;
/**
 * @brief initialize DCP orient model
 * @param[in]  pz_dcpConfig the configuration for the DCP model
 * @return void
 */
void DCP_ortInit(const gnss_DcpConfigOpt_t* pz_dcpConfig);

/**
*@brief deinitilize the DCP orient algorithm
* @param[in/out]  void
* @return         void
*/
void DCP_ortDeinit();

/**
*@brief update the back ground result for main antenna
* @param[in]  pz_tdcpMeasBlock is TDCP measuremt for main antenna
* @return         void
*/
void DCP_updateMainAntBackGroundResult(const gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock);

/**
*@brief update the back ground result for auxi antenna
* @param[in]  pz_tdcpMeasBlock is TDCP measuremt for main antenna
* @return         void
*/
void DCP_updateAuxiAntBackGroundResult(const gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock);

/**
*@brief update the back ground result of orient
* @param[in]  pz_tdcpMeasBlock is TDCP measuremt for main antenna
* @return         void
*/
void DCP_updatOrtBackGroundResult(const gnss_OrientFix_t* pz_ortResult);

/**
*@brief push auxi antenna measure
* @param[in]  pz_auxiCurentTdcpMeas is TDCP measuremt for auxi antenna of current epoch
* @return         void
*/
void DCP_PushRcvAuxiMeasBlk_NHz(const gnss_TdcpMeasBlock_t* pz_auxiCurentTdcpMeas);

/**
*@brief push main antenna measure
* @param[in]  pz_mainCurentTdcpMeas is TDCP measuremt for main antenna of current epoch
* @return         void
*/
void DCP_PushRcvMainMeasBlk_NHz(const gnss_TdcpMeasBlock_t* pz_mainCurentTdcpMeas);

/**
 * @brief calculate the current orient using the previous epoch and current observations by TDCP or TDPR
 * @param[out] z_obsTime time of current epoch
 * @param[out] pz_ortResult is the result of heading .etc
 * @return 0 represent success and other failure
 */
ort_dcpSolStatus DCP_SolveRealtimeOrt(GpsTime_t z_obsTime, gnss_OrientFix_t* pz_ortResult);
END_DECL
#endif