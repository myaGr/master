/**@file        dcp_solution.h
 * @brief       solute the relative position using the carrier-phase, doppler or pseudo-range observations
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/08/30  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __DCP_SOLUTION_H__
#define __DCP_SOLUTION_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "dcp_common.h"
BEGIN_DECL

typedef struct
{
  gnss_TdcpMeasBlock_t*        pz_baseDcpMeas;        /*the TDCP measure block updated by HPP or PPP-RTK or orient*/
  gnss_TdcpMeasBlock_t*        pz_preDcpMeas;         /*the TDCP measure block of previous epoch used by feedback measure to INS*/
  gnss_DcpPosSeq_t*            pz_dcpPosSeq;          /*postion smooth*/
  gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeas;/*the GNSS measure block that feedback to INS*/
  gnss_DcpConfigOpt_t          z_dcpOpt;              /*the configuration of DCP*/
  GpsTime_t                    z_gpsTimePreEmpty;     /* previous empty time */
}dcp_solInfo_t;

/**
 * @brief initialize DCP model
 * @param[in]  pz_dcpConfig the configuration for the DCP model
 * @param[out] pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void DCP_init(const gnss_DcpConfigOpt_t* pz_dcpConfig, dcp_solInfo_t* pz_dcpSolInfo);

/**
*@brief deinitilize the DCP algorithm
* @param[in/out]  pz_dcpSolInfo the solution information for the DCP model
* @param[out]     void
* @return         void
*/
void DCP_deinit(dcp_solInfo_t* pz_dcpSolInfo);

/**
*@brief load the default option for the DCP algorithm
* @param[in]  pz_opt represent the algorithm optimization for the DCP
* @return
*/
void DCP_loadDefaultOption(gnss_DcpConfigOpt_t* pz_opt);

/**
 * @brief update the TDCP measure block of previous epoch used by HPP or RTK or PPP-RTK or orient
 * @param[in] pz_baseDcpMeas
 * @param[in] pz_dcpSolInfo the solution information for the DCP model
 * @return void
 */
void dcp_updateBaseMeasBlock(const gnss_TdcpMeasBlock_t* pz_baseDcpMeas, dcp_solInfo_t* pz_dcpSolInfo);

/**
 * @brief create empty pos to report
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @return None
 */
void dcp_CreateEmptyPos(const GnssMeasBlock_t* pz_MeasBlock, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  dcp_solInfo_t* pz_dcpSolInfo);

/**
 * @brief calculate the current position using the previous epoch and current observations by TDCP or TDPR
 * @param[out] pz_curDcpMeas TDCP measure block of current epoch
 * @param[out] pz_dcpSolInfo the solution information for the DCP model
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_SolveRealtimePos(gnss_TdcpMeasBlock_t* pz_curDcpMeas, dcp_solInfo_t* pz_dcpSolInfo);

END_DECL
#endif