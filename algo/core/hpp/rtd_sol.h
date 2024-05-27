/**@file        rtd_sol.h
 * @brief       using the pseudo-rang to do RTD
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/04/17  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __RTD_SOL_H__
#define __RTD_SOL_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "rtk_type.h"
BEGIN_DECL

typedef struct
{
  BOOL    q_QRsuccess;         /*the status of QR check,TRUE represent successful and FALSE represent failure*/
  uint8_t u_PRtotalNum;        /*the total number of norm pseudo-range observations*/
  uint8_t pu_PRnum[C_GNSS_MAX];/*the number of norm pseudo-range observations for per constellation*/
  double  pd_QRcheckSiteCoor[3];/*the site coordinate calculated by QR check*/
}rtd_QRcheckSiteCoorInfo_t;
typedef struct  /* EKF filter information */
{
  rtk_EKFState    z_kfStatus;               /* KF_INIT,KF_RUN,... */
  uint16_t        w_nmax;
  uint16_t        w_n;
  BOOL            q_filterPosValid;         /* filter pos is available*/
  float           f_age;                    /* age of VRS */
  double*         pd_deltaX;
  double*         pd_X;                     /* x */
  double*         pd_Q;                     /* only store the lower triangle */
  BOOL*           pq_paraValid;             /*mask the parameter is added to the filter*/
  rtk_alg_opt_t   z_opt;
  uint8_t         pu_measUpdatePseudoNum[C_GNSS_FREQ_TYPE_MAX];
  rtd_QRcheckSiteCoorInfo_t* pz_QRcheckSiteCoorInfo;
} rtd_filterInfo_t;

typedef struct
{
  GpsTime_t                 z_gpsTime;
  double                    d_continueFilterTime;  /* continue time of filter */
  gnss_EKFstateRepresent_t* pz_satPool[3 + (C_GNSS_MAX + 1)];
} rtd_EKFstateRepresentPool_t;


/**
 * @brief Initialize RTD module
 * @param[in]  pz_opt represent the algorithm option for the RTD
 * @param[out] pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t rtd_init(const rtk_alg_opt_t* pz_opt, rtd_filterInfo_t* pz_RTDfilterInfo, rtd_EKFstateRepresentPool_t* pz_EKFstateRepPool);

/**
 * @brief de-initialize RTD module
 * @param[out] pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out] pz_EKFstateRepPool is the pool of EKF state represent
 * @return     void
 */
void rtd_deinit(rtd_filterInfo_t* pz_RTDfilterInfo, rtd_EKFstateRepresentPool_t* pz_EKFstateRepPool);

/**
 * @brief the inteface of RTD solute algorithm
 * @param[in]      pz_pvtResult is information of pvt
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @param[in]      pz_rtkCorrBlock is the OSR correction
 * @param[out]     pz_RTDfilterInfo the filter for the RTD algorithm
 * @param[out]     pz_EKFstateRepPool is the pool of EKF state represent
 * @return         0 represent success and other failed
 */
int32_t RTD_solute(const gnss_PVTResult_t* pz_pvtResult, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_rtkCorrBlock,
  rtd_filterInfo_t* pz_RTDfilterInfo, rtd_EKFstateRepresentPool_t* pz_EKFstateRepPool);
END_DECL
#endif