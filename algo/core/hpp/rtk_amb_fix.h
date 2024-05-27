/**@file        rtk_amb_fix.h
 * @brief       integer ambiguity resolution for RTK
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
#ifndef __RTK_AMB_FIX_H__
#define __RTK_AMB_FIX_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "rtk_type.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "seq_kalman.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include "mlambda.h"

BEGIN_DECL

#define GNSS_PART_LAMBDA_FULL     0
#define GNSS_PART_LAMBDA_QOS      1
#define GNSS_PART_LAMBDA_AMB      2
#define GNSS_PART_LAMBDA_MP       3
#define GNSS_PART_LAMBDA_GROUP    4

#define GNSS_MIN_LOCK             2

#define GNSS_VAR_HOLDAMB 0.001    /* constraint to hold ambiguity (cycle^2) */

typedef struct
{
  uint8_t     u_satIndex;
  uint8_t     u_signal;
  uint16_t    u_amb_index;
  uint16_t    u_ref_amb_index;
} gnss_AmbFix_t; //zerocombine sat list info

typedef struct
{
  uint8_t     u_satIndex;
  uint16_t    u_amb_index_L1;
  uint16_t    u_amb_index_L2;
  uint16_t    u_ref_amb_index_L1;
  uint16_t    u_ref_amb_index_L2;
} gnss_WideLandAmbFix_t; //widelane sat list info

/**
 * @brief initialize the information for the RTK ambiguity resolution
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_filterObs is the information of observation after filter
 * @param[in]  pz_OSRblock is the OSR correction
 * @param[in]  pz_RTKfilterInfo is the EKF filter information of RTK
 * @param[in]  pz_EKFstateRepPool is the EKF state represent pool for RTK
 * @param[out] pz_RTKambFixInputInfo is the input information for RTK ambiguity fix 
 * @return     void
 */
void rtk_setAmbFixInputInfo(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, rtk_EpochFilterObs_t* pz_filterObs, const GnssCorrBlock_t* pz_OSRblock,
  rtk_filterInfo_t* pz_RTKfilterInfo, gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, gnss_rtkAmbFixInputInfo_t* pz_RTKambFixInputInfo);
#if 0
/**
 * @brief calculate satellite cn0 and ele threshold
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @return		  u_status=0 fail, else success
 */
static uint8_t rtk_ambCalSatQos(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief get reference satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     u_targetConstellation
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     q_isArcLenLimit that TRUE limit the arc len and FALSE do not consider 
 * @return svid
 */
static uint8_t rtk_ambFixRefSatSelectSf(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  uint8_t u_targetConstellation, gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, BOOL q_isArcLenLimit);

/**
 * @brief get rover satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     pu_toBeFixSatNum the number of satellite to be fixed
 * @return 0: success, other: fail
 */
static uint8_t rtk_ambFixGetSatList(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_AmbFix_t* pz_ambFix, uint8_t pu_toBeFixSatNum[C_GNSS_MAX]);

/**
 * @brief get reference satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @return 0: success, other: fail
 */
static RTKAmbFixFailInfoType rtk_ambFixRefSatSelect(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief get X and Q of zero combine ambiguity
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     u_nb
 * @param[out]    pd_X
 * @param[out]    pd_Q
 * @return 0: OK, other: fail
 */
static uint8_t rtk_ambFixGetXQ(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_AmbFix_t* pz_ambFix, uint8_t u_nb, double* pd_X, double* pd_Q);

/**
 * @brief RTK zero combine ambiguity result save
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static void rtk_ambFixSaveResult(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  uint8_t u_nb, gnss_AmbFix_t* pz_ambFix, const double* pd_amb_int);

/**
 * @brief RTK zero combine ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static RTKAmbFixFailInfoType rtk_ambFixAR(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief RTK zero combine ambiguity update
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static void rtk_ambZeroCombineUpdate(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief clear ambiguity info
 * @param[in/out]  pz_fixedAmbPool is result of amb resolution
 * @return         void
 */
static void RTK_ResetFixedAmbPool(gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief set pz_prefixedAmbPool from pz_curfixedAmbPool
 * @param[in]    pz_curfixedAmbPool
 * @param[out]    pz_prefixedAmbPool
 * @return none
 */
static void rtk_ambUpdateFixedAmbPool(const gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
  gnss_fixedSignalAmbPool_t** pz_prefixedAmbPool);

/**
 * @brief RTK zero combine ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static RTKAmbFixFailInfoType RTK_ZeroCombineAmbFix(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief get widelane reference satellite
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     pz_ambFix  ambiguity information
 * @return 0: success, other: fail
 */
static RTKAmbFixFailInfoType rtk_widelaneFixRefSatSelect(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2);

/**
 * @brief get widelane satellite list
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]	    z_freqType1 is first frequence
 * @param[in]	    z_freqType2 is second frequence
 * @param[in]     pz_ambFix  ambiguity information
 * @param[in]     q_isArcLenLimit that TRUE limit the arc len and FALSE do not consider
 * @return 0: success, other: fail
 */
static uint8_t rtk_widelaneGetSatList(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, gnss_WideLandAmbFix_t* pz_ambFix, BOOL q_isArcLenLimit);

/**
 * @brief RTK widelane ambiguity result save
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @param[in]	     z_freqType1 is first frequence
 * @param[in]	     z_freqType2 is second frequence
 * @param[in]      u_nb is number of ambiguity
 * @param[in]      pz_ambFix is satellite information
 * @param[in]      pd_amb_int is widelane integer ambiguity
 * @return         void
 */
static void rtk_widelaneSaveResult(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2, uint8_t u_nb, gnss_WideLandAmbFix_t* pz_ambFix, const double* pd_amb_int);

/**
 * @brief get X and Q of widelane ambiguity
 * @param[in]     pz_rtkAmbFixInputInfo
 * @param[in]     pz_fixedAmbPool
 * @param[in]     pz_ambFix is satellite information
 * @param[in]     u_nb is number of ambiguity
 * @param[out]    pd_X
 * @param[out]    pd_Q
 * @return 0: OK, 1: fail
 */
static uint8_t rtk_widelaneGetXQ(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_WideLandAmbFix_t* pz_ambFix, uint8_t u_nb, double* pd_X, double* pd_Q);

/**
 * @brief RTK widelane ambiguity confirm
 * @param[in/out]  u_nb is number of bias
 * @return         TRUE success; else fail
 */
static uint8_t rtk_widelaneConfirm(uint8_t u_nb, float f_ratio, double d_adop);

/**
 * @brief RTK zerocombine ambiguity confirm
 * @param[in]  u_nb is number of bias
 * @param[in]  f_ratio is lambda ratio
 * @param[in]  d_adop is adop
 * @return         TRUE success; else fail
 */
static uint8_t rtk_ambConfirm(uint8_t u_nb, float f_ratio, double d_adop);

/**
 * @brief RTK widelane ambiguity resolution
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of ambigutiy resolution
 * @param[in]	     z_freqType1 is first frequence
 * @param[in]	     z_freqType2 is second frequence
 * @return         u_status 0 success; else fail
 */
static uint8_t rtk_widelaneFix(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2);

/**
 * @brief RTK widelane ambiguity update
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[in]	     z_freqType1 is first frequence
 * @param[in]	     z_freqType2 is second frequence
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         void
 */
static void rtk_widelaneFixUpdate(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool,
  gnss_FreqType z_freqType1, gnss_FreqType z_freqType2);

/**
 * @brief RTK zerocombine fix and hold
 * @param[in/out]  pz_curfixedAmbPool is input information for RTK ambiguity resolution
 * @param[out]     pz_preFixedAmbPool is result of amb resolution
 * @param[out]     pz_rtkFilterInfo is filter information
 * @return         void
 */
static void rtk_zerocombineAmbHold(gnss_fixedSignalAmbPool_t* pz_curfixedAmbPool,
  gnss_fixedSignalAmbPool_t* pz_preFixedAmbPool, rtk_filterInfo_t* pz_rtkFilterInfo);

/**
 * @brief RTK zero combine ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
static void rtk_zeroCombineAmbResolution(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

/**
 * @brief RTK widelane ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         void
 */
static void rtk_widelaneAmbResolution(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);
#endif
/**
 * @brief RTK ambiguity resolve
 * @param[in/out]  pz_rtkAmbFixInputInfo is input information for RTK ambiguity resolution
 * @param[out]     pz_fixedAmbPool is result of amb resolution
 * @return         status of RTK ambiguity resolution
 */
uint8_t RTK_AmbResolution(gnss_rtkAmbFixInputInfo_t* pz_rtkAmbFixInputInfo, gnss_fixedSignalAmbPool_t* pz_fixedAmbPool);

END_DECL
#endif