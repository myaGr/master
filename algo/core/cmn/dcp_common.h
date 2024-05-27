/**@file        dcp_common.h
 * @brief       define the interface of DCP model
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/10/25  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __DCP_COMMON_H__
#define __DCP_COMMON_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "seq_kalman_float.h"
BEGIN_DECL

/* cycle slip detect model */
typedef uint8_t dcp_CycleSlipMask;
#define DCP_NON_DETECT                   ((uint8_t)0x00)                     /* cycle slip non detect */
#define DCP_NO_CYCLE_SLIP_BY_GF          ((uint8_t)0x01)                     /* non cycle slip detect by GF combination*/
#define DCP_NO_CYCLE_SLIP_BY_DOPPLER     ((uint8_t)0x02)                     /* non cycle slip detect by doppler*/
#define DCP_PR_DR_CONSISTENCY            ((uint8_t)0x04)                     /* the pseudo-range is consistent with doppler*/
#define TD_MAX_PARA                      (3 + C_GNSS_MAX)                    /* the number of parameter in filter for DCP model */
#define TD_MAX_Q_ELE                     ((TD_MAX_PARA) * (TD_MAX_PARA+1)/2) /* the number of parameter covariance in filter for DCP model */

/* the calculated information used by TDCP or TDPR algorithm */
typedef struct
{
  gnss_DcpType      u_posObainType;                      /*position type of obtained method */
  uint8_t           u_goodQualityObsNum[C_GNSS_MAX];     /*the number of residual less than specified threshold*/
  uint8_t           u_paraNum;                           /*the number of parameter in the DCP filter*/
  uint8_t           u_enterFilterNum[C_GNSS_MAX];        /*the number of observations that entering filter*/
  dcp_CycleSlipMask pu_slipTag[MAX_GNSS_TRK_MEAS_NUMBER];/*the flag of cycle slip*/
  float             pf_wP[MAX_GNSS_TRK_MEAS_NUMBER];     /*the factor of weight calculated by robust estimation*/
  float             pf_rcvClkDrift[C_GNSS_MAX];          /*receiver clock drift for time difference obervations*/
  float             f_deltaTime;                         /*the interval between current epoch and previous epoch*/
  float             f_sigma0;                            /*sqrt(vtpv/(n-t))*/
  float             pf_X[TD_MAX_PARA];                   /*the X in the float kalman filter*/
  float             pf_deltaX[TD_MAX_PARA];              /*the delta X in the float kalman filter*/
  float             pf_Q[TD_MAX_Q_ELE];                  /*the covariance of float kalman filter*/
  double            pd_curXyz[3];                        /*the current position in ECEF obained by TDCP or TDPR or TDDR*/
} dcp_interRecordInfo_t;

#define DCP_CARRIER_PRIOR_SIGMA0 (0.05f)
#define DCP_DOPPLE_PRIOR_SIGMA0  (3.0f)
#define DCP_PSEUDO_PRIOR_SIGMA0  (3.0f)

/**
 * @brief initialize the dcp_interRecordInfo_t
 * @param[out] pz_dcpInfo, initilize the calculated information used by TDCP or TDPR algorithm
 * @return void
 */
void dcp_initDcpInterInfo(dcp_interRecordInfo_t* pz_dcpInfo);

/**
 * @brief fillup the current position solution struct by the previous epoch
 * @param[in] pz_prePosSol,the previous position information
 * @param[out] pz_curPosSol,the current position information
 * @return void
 */
void dcp_fillupPosSolUsingPreEpoch(const gnss_PositionFix_t* pz_prePosSol, gnss_PositionFix_t* pz_curPosSol);

/**
 * @brief judge whether going the TDCP or TDPR routeline
 * @param[in] pu_preSvUsed,  the number of satellites used by previous epoch
 * @param[in] pu_flags,      the type of position solution for the previous epoch
 * @param[in] pd_xyz[3],     the ECEF position for the previous epoch
 * @param[in] pf_deltaTime,  the interval between the previous epoch and current epoch
 * @return 1 represent going the TDCP or TDPR routeline, 0 represent don't go the TDCP or TDPR routeline
 */
uint8_t dcp_checkGoingTDroute(const uint8_t* pu_preSvUsed, const uint8_t* pu_flags, const double pd_xyz[3], const float* pf_deltaTime);

/**
 * @brief find the index observation in the previous epoch
 * @param[in]  pz_preDcpMeas TDCP measure block of previous epoch
 * @param[in] u_curSys, GNSS constellation
 * @param[in] u_curPrn, satellite index of the system
 * @param[in] e_curFreqType, frequency type of satellite
 * @param[out] pu_preIndex, the index of observation in previous epoch
 * @return 1 represent find target observations success, otherwise, 0 represent find target observations failure
 */
uint8_t dcp_findMeasPreEpoch(const gnss_TdcpMeasBlock_t* pz_preDcpMeas, uint8_t u_curSys, uint8_t u_curPrn, gnss_FreqType e_curFreqType, uint8_t* pu_preIndex);

/**
 * @brief find the index of satellite data to get the satellite position and clock
 * @param[in] u_sys,GNSS constellation
 * @param[in] u_prn,satellite index of the system
 * @param[in] pz_dcpMeas, the data used by TDCP,include current epoch and previous epoch
 * @return int16_t,index of satellite data
 */
int16_t dcp_findSatDataIndex(uint8_t u_sys, uint8_t u_prn, const gnss_TdcpMeasBlock_t* pz_dcpMeas);

/**
 * @brief calculate the current position using the previous epoch and current observations by TDCP or TDPR
 * @param[in/out] pz_dcpConfig the configuration for the DCP model
 * @param[in]     pz_preDcpMeas TDCP measure block of previous epoch
 * @param[out]    pz_curDcpMeas TDCP measure block of current epoch
 * @param[out]    pz_dcpInfo, the calculated information used by TDCP or TDPR algorithm
 * @return 1 represent success and 0 represent failure
 */
uint8_t dcp_gainTDpos(gnss_DcpConfigOpt_t* pz_dcpConfig, gnss_TdcpMeasBlock_t* pz_preDcpMeas, gnss_TdcpMeasBlock_t* pz_curDcpMeas,
  dcp_interRecordInfo_t* pz_dcpInfo);

/**
 * @brief initilize the struct used by sequential kalman filter,the type is float
 * @param[in]   w_paraNum               the total number of parameter
 * @param[out]  pz_seqKalmanFloatVar,   the struct used by sequential kalman filter,the type is float
 * @return      1 represent obtained memory success and 0 represent obtained memory failure
 */
uint8_t dcp_initSeqKalmanFloatVar(uint16_t w_paraNum, SeqKalmanFloatVar_t* pz_seqKalmanFloatVar);

/**
 * @brief deinitilize the struct used by sequential kalman filter,the type is float
 * @param[out]  pz_seqKalmanFloatVar,   the struct used by sequential kalman filter,the type is float
 * @return      void
 */
void dcp_deinitSeqKalmanFloatVar(SeqKalmanFloatVar_t* pz_seqKalmanFloatVar);

END_DECL
#endif