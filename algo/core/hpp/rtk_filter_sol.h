/**@file        rtk_filter_sol.h
 * @brief       the float solution for the RTK algorithm
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/02/20  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __RTK_FILTER_SOL_H__
#define __RTK_FILTER_SOL_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "rtk_type.h"

BEGIN_DECL

#define AMB_PRINT_SIZE 256
/* zero AMB update print */
typedef struct
{
  char* pu_LLI;
  char* pu_GF;
  char* pu_Doppler;
  char* pu_TDCP;
  char* pu_LackOfDetect;
  char* pu_LackOfCP;
  char* pu_LackOfCorr;
  char* pu_CorrSlip;
  char* pu_InitFail;
  char* pu_DCBout;
  char* pu_newLock;
  char pu_LLI_sat[AMB_PRINT_SIZE];
  char pu_GF_sat[AMB_PRINT_SIZE];
  char pu_Doppler_sat[AMB_PRINT_SIZE];
  char pu_TDCP_sat[AMB_PRINT_SIZE];
  char pu_LackOfDetect_sat[AMB_PRINT_SIZE];
  char pu_CorrSlip_sat[AMB_PRINT_SIZE];
  char pu_LackOfCP_sat[AMB_PRINT_SIZE];
  char pu_LackOfCorr_sat[AMB_PRINT_SIZE];
  char pu_InitFail_sat[AMB_PRINT_SIZE];
  char pu_DCBout_sat[AMB_PRINT_SIZE];
  char pu_newLock_sat[AMB_PRINT_SIZE];
} rtk_AMBupdatePrint_t;

/* measment snr print */
typedef struct
{
  char* pu_prn;
  char* pu_ele;
  char* pu_azi;
  char* pu_snr[MAX_GNSS_SIGNAL_FREQ];
  char pu_prn_sat[AMB_PRINT_SIZE];
  char pu_ele_sat[AMB_PRINT_SIZE];
  char pu_azi_sat[AMB_PRINT_SIZE];
  char pu_snr_sat[MAX_GNSS_SIGNAL_FREQ][AMB_PRINT_SIZE];
} rtk_measSNRprint_t;

/**
 * @brief sat meas`s singal num, filter NULL
 * @param pz_satMeas
 * @return sat meas`s singal num 0~3
 */
uint8_t RTK_signalNum(const gnss_SatelliteMeas_t* pz_satMeas);
/**
 * @brief the inteface of RTK float algorithm
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return         0 represent success and other failed
 */
int32_t RTK_filterSolute(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  rtk_EpochFilterObs_t* pz_filterObs, GnssCorrSlipFlag_t* pz_corrSlipFlag);

/**
 * @brief malloc rtk_measSNRprint_t and init
 * @return     rtk_measSNRprint_t
 */
rtk_measSNRprint_t* RTK_malloc_SNRprint();

/**
 * @brief calculate omc residual
 * @param[in]  pd_X is value for the state
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_rtkCorrBlock is the OSR correction
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pz_filterObs is the filter obs information
 * @return    the number of Residuals
 */
uint8_t RTK_calculateResidual(const double* pd_X, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect,
  const GnssCorrBlock_t* pz_rtkCorrBlock, rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_EpochFilterObs_t* pz_filterObs, rtk_ResidualType z_residualType);

/**
 * @brief      check filter need to reset or not
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0x0: No need to reset;
                         0x1: close sky;
                         0x2: float filter thres outage;
 */
uint8_t rtk_filterResetCheck(rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool);

/**
 * @brief      reset RTK filter module
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t rtk_resetFilter(rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool);

/**
 * @brief      check positive definite of gz_RTKfilterInfo's P mat module
 * @param[out] pz_RTKfilterInfo the filter for the RTK algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0: Positive Definite; other: error case
 */
uint8_t rtk_positiveDefiniteCheck(rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool);


END_DECL
#endif