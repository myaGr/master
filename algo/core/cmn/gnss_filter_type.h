/**@file        gnss_filter_type.h
 * @brief       Location Engine GNSS Filter Structure Types
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
#ifndef __GNSS_FILTER_TYPE_H__
#define __GNSS_FILTER_TYPE_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include <math.h>

BEGIN_DECL

#define GNSS_MAX_STATE_REP        (3)


#define DYN_H1_1s           0.46826882694955f
#define DYN_H2_1s           0.90634623461009f
#define DYN_H3_1s           0.81873075307798f

#define DYN_H1_200ms        0.01973597880808f
#define DYN_H2_200ms        0.19605280423838f
#define DYN_H3_200ms        0.96078943915232f

#define DYN_H1_100ms        0.0049668326688817590f
#define DYN_H2_100ms        0.099006633466223737f
#define DYN_H3_100ms        0.98019867330675525f

#define DYN_H1_1s_revs       DYN_H1_1s
#define DYN_H2_1s_revs      (-DYN_H2_1s)
#define DYN_H3_1s_revs       DYN_H3_1s

#define DYN_H1_200ms_revs    DYN_H1_200ms
#define DYN_H2_200ms_revs   (-DYN_H2_200ms)
#define DYN_H3_200ms_revs    DYN_H3_200ms

#define DYN_H1_100ms_revs    DYN_H1_100ms
#define DYN_H2_100ms_revs   (-DYN_H2_100ms)
#define DYN_H3_100ms_revs    DYN_H3_100ms

#define DYN_Q11_1s          0.01792802206774f
#define DYN_Q12_1s          0.04385513885856f
#define DYN_Q13_1s          0.05469131832920f
#define DYN_Q22_1s          0.11507415690720f
#define DYN_Q23_1s          0.16429269939838f
#define DYN_Q33_1s          0.32967995396436f

#define DYN_Q11_200ms       0.00000625978698f
#define DYN_Q12_200ms       0.00007790177189f
#define DYN_Q13_200ms       0.00051246202946f
#define DYN_Q22_200ms       0.00103525556642f
#define DYN_Q23_200ms       0.00768734040995f
#define DYN_Q33_200ms       0.07688365361336f

#define DYN_Q11_100ms       1.9779357839089237e-07f
#define DYN_Q12_100ms       4.9338853466132293e-06f
#define DYN_Q13_100ms       6.5347885165080430e-05f
#define DYN_Q22_100ms       0.00013135186744603342f
#define DYN_Q23_100ms       0.0019604626940628034f
#define DYN_Q33_100ms       0.039210560847676823f

typedef enum {
  GNSS_FILTER_STATE_POS = 1,
  GNSS_FILTER_STATE_VEL,
  GNSS_FILTER_STATE_ACC,
  GNSS_FILTER_STATE_CLK,
  GNSS_FILTER_STATE_DRIFT,
  GNSS_FILTER_STATE_ZTD,
  GNSS_FILTER_STATE_IONO,
  GNSS_FILTER_STATE_AMB_L1,
  GNSS_FILTER_STATE_AMB_L2,
  GNSS_FILTER_STATE_AMB_L5,
  GNSS_FILTER_STATE_DCB12,
  GNSS_FILTER_STATE_DCB15,
  GNSS_FILTER_STATE_NUM
} gnss_FilterStateEnumTypeVal;
typedef uint16_t gnss_FilterStateEnumType;

/* KF status for RTK or PPP-RTK */
typedef uint8_t ppp_EKFState;
typedef uint8_t rtk_EKFState;
typedef uint8_t ort_EKFState;
#define KF_IDLE_HPP     0x00  /* KF has done nothing */
#define KF_INIT         0x01  /* KF is to init by spp results */
#define KF_RESET        0x02  /* KF is to reset pva and amb */
#define KF_RESET_AMB    0x04  /* KF is to reset amb only */
#define KF_RESTART_HPP  0x08  /* KF is to restart */
#define KF_RUN          0x10  /* KF is running */
#define KF_ENLARGEQ     0x20  /* KF is to enlarge Q while KF is running */

#define POS_SIGMA      (30.0) /* initial sigma of receiver pos (m) */
#define VEL_SIGMA      (10.0) /* initial sigma of receiver vel ((m/s)) */
#define ACC_SIGMA      (10.0) /* initial sigma of receiver acc ((m/ss)) */

#define RCV_CLK_SIGMA  (100.0) /* initial sigma of receiver clock (m) */
#define RCV_CLK_DRIFT_SIGMA  (100.0) /* initial sigma of receiver clock drift (m) */
#define RCV_DCB_SIGMA  (10.0) /* initial sigma of receiver DCB (m) */

#define AMB_SIGMA  (100.0) /* initial sigma of ambiguity (cycle) */
#define AMB_NOISE  (1.0e-4) /* noise of ambiguity (cycle) */
#define RTK_AMB_NOISE  (1.0e-3) /* noise of ambiguity for RTK (cycle) */

#define IONO_SIGMA  (30.0) /* initial sigma of ionospheric (m) */
#define IONO_NOISE  (0.05) /* noise of ionospheric (m) */

#define ZTD_SIGMA   (0.3) /* initial sigma of ZTD (m) */
#define ZTD_NOISE  (1.1111e-5) /* noise of ZTD (m), 0.04/3600.0 meter*/

typedef struct
{
  uint16_t w_id[GNSS_MAX_STATE_REP];
  int16_t w_index;
  GpsTime_t  z_beginTime;
  GpsTime_t  z_endTime;
}gnss_EKFstateRepresent_t;

typedef struct
{
  GpsTime_t                 z_gpsTime;
  double                    d_continueFilterTime;  /* continue time of filter */
  gnss_EKFstateRepresent_t* pz_satPool[GNSS_MAX_FILTER_STATE_NUM];
}gnss_EKFstateRepresentPool_t;
/**
 * @brief map the dcb type to filter type
 * @param[in]   z_f frequency type
 * @return gnss_FilterStateEnumType is the filter type
 */
gnss_FilterStateEnumType convertFreqDCB2FilterType(gnss_FreqType z_f);
/**
 * @brief map the ambiguity type relative frequency to filter type
 * @param[in]   z_f frequency type
 * @return gnss_FilterStateEnumType is the filter type
 */
gnss_FilterStateEnumType convertFreqAmb2FilterType(gnss_FreqType z_f);
/**
 * @brief map the frequency type to amb type
 * @param[in]   z_f1 frequency type
 * @param[in]   z_f2 frequency type
 * @return gnss_fixedAmbType is the amb type
 */
gnss_fixedAmbType convertFreqType2AmbType(gnss_FreqType z_f1, gnss_FreqType z_f2);
/**
 * @brief map the filter type to frequency type
 * @param[in]   z_filterType filter type
 * @return gnss_FreqType is the frequency type
 */
gnss_FreqType convertFilterType2FreqAmb(gnss_FilterStateEnumType z_filterType);
/**
 * @brief the initilization information of parameters in the filter
 * @param[in]    gnss_filterInitInfo_t* pz_filterInitInfo
 * @return void
 */
void initFilterInitInfo(gnss_filterInitInfo_t* pz_filterInitInfo);
/**
 * @brief initilize the ekf state represent
 * @param[out]   gnss_EKFstateRepresent_t* pz_EKFstateRep
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @return void
 */
void initEKFstateRepresent(gnss_EKFstateRepresent_t* pz_EKFstateRep, uint16_t w_nmax,
	double* pd_X, double* pd_Q);
/**
 * @brief get ekf state pointer
 * @param[in]    w_id
 * @param[in]    pz_statusPool
 * @return pointer
 */
gnss_EKFstateRepresent_t* getEKFstatusModify(const uint16_t w_id[GNSS_MAX_STATE_REP], gnss_EKFstateRepresentPool_t* pz_statusPool);
/**
 * @brief get ekf state pointer
 * @param[in]    w_id
 * @param[in]    pz_statusPool
 * @return pointer
 */
extern const gnss_EKFstateRepresent_t* getEKF_status(const uint16_t w_id[GNSS_MAX_STATE_REP], const gnss_EKFstateRepresentPool_t* pz_statusPool);
/**
 * @brief remove the ekf state that long time not be updated
 * @param[in]    pz_statusPool
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @return       void
 */
void removeEKFstatus(gnss_EKFstateRepresentPool_t* pz_statusPool, uint16_t w_nmax,
	double* pd_X, double* pd_Q, BOOL* pq_paraValid);

/**
 * @brief remove the target ekf state that long time not be updated
 * @param[in]    w_id represent the status of EKF
 * @param[in]    pz_statusPool
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @return       void
 */
void removeTargetEKFstatus(const uint16_t w_id[GNSS_MAX_STATE_REP], gnss_EKFstateRepresentPool_t* pz_statusPool,
	uint16_t w_nmax, double* pd_X, double* pd_Q, BOOL* pq_paraValid);
/**
 * @brief remove the AMB ekf state in error case
 * @param[in]    pz_statusPool
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @return       void
 */
void removeAMBEKFstatus(gnss_EKFstateRepresentPool_t* pz_statusPool, uint16_t w_nmax,
  double* pd_X, double* pd_Q, BOOL* pq_paraValid);
/**
 * @brief count number of the ekf state
 * @param[in]    pz_statusPool
 * @return       the number of the ekf state
 */
uint16_t countEKFstatus(const gnss_EKFstateRepresentPool_t* pz_statusPool);
/**
 * @brief get the indexs of all EKF state
 * @param[in]    pz_statusPool
 * @param[in]    pw_paraIndex the indexs of all EKF state
 * @return       the number of the ekf state
 */
uint16_t getAllStatusIndexEKF(const gnss_EKFstateRepresentPool_t* pz_statusPool, uint16_t pw_paraIndex[GNSS_MAX_FILTER_STATE_NUM]);
/**
 * @brief add the target ekf state
 * @param[in]    w_id represent the status of EKF
 * @param[in]    pz_obsTime represent the observation time of parameter
 * @param[in]    pz_statusPool
 * @return       gnss_EKFstateRepresent_t* the pointer of ekf state represent
 */
gnss_EKFstateRepresent_t* addTargetEKFstatus(const uint16_t w_id[GNSS_MAX_STATE_REP],
	const GpsTime_t* pz_obsTime, gnss_EKFstateRepresentPool_t* pz_statusPool);
/**
 * @brief add the parameter to the EKF filter
 * @param[in]    w_id represent the status of EKF
 * @param[in]    w_nmax represent maximum number status of EKF
 * @param[in]    pz_initInfo represent the initilization infrmation of parameter
 * @param[out]   pd_X represent the value of parameter in the filter
 * @param[out]   pd_Q represent the covariance of parameter in the filter
 * @param[out]   pq_paraValid represent the mask the parameter is added to the filter
 * @param[out]   pz_statusPool represet the state poll in the filter
 * @return       0 represent failure and other represent success
 */
uint8_t addState2EKFfilter(const uint16_t w_id[GNSS_MAX_STATE_REP], uint16_t w_nmax,
	const gnss_filterInitInfo_t* pz_initInfo, double* pd_X, double* pd_Q, BOOL* pq_paraValid,
	gnss_EKFstateRepresentPool_t* pz_statusPool);
/**
 * @brief get the state transition matrix of PVA model
 * @param[in]    d_deltaT represent time difference between current and the filter time
 * @param[in]    d_alphaIn represent the maneuver parameter of dynamic
 * @param[in]    u_isPPKreverse whether or not be reversed model
 * @param[out]   pd_phi[6] the state transition matrix of PVA model
 * pd_phi[0] denote A11,pd_phi[1] denote A12,pd_phi[2] denote A13
 * pd_phi[3] denote A22,pd_phi[4] denote A23,pd_phi[5] denote A33
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t calPhi(double d_deltaT, double d_alphaIn, uint8_t u_isPPKreverse, double pd_phi[6]);
/**
 * @brief get the noise of PVA model
 * @param[in]    d_deltaT represent time difference between current and the filter time
 * @param[in]    d_alphaIn represent the maneuver parameter of dynamic
 * @param[in]    d_accSigmaInit represent the enlarged factor for the PVA model
 * @param[in]    u_isPPKreverse whether or not be reversed model
 * @param[out]   pd_q[6] the noise of PVA model
 * pd_dynamicNoise[0] denote q11,pd_dynamicNoise[1] denote q12,pd_dynamicNoise[2] denote q13
 * pd_dynamicNoise[3] denote q22,pd_dynamicNoise[4] denote q23,pd_dynamicNoise[5] denote q33
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t calDynamicNoise(double d_deltaT, double d_alphaIn, double d_accSigmaInit,
	uint8_t u_isPPKreverse, double pd_dynamicNoise[6]);
/**
 * @brief state transfer for PVA model
 * @param[in]    pd_filterX represent the value of parameter in the filter
 * @param[in]    pd_filterQ represent the covariance of parameter in the filter
 * @param[in]    pz_statusPool represent the status pool in the filter
 * @param[in]    z_gpsTime represent current time
 * @param[in]    d_alphaIn represent the maneuver parameter of dynamic
 * @param[in]    d_accSigmaInit represent the enlarged factor for the PVA model
 * @param[in]    u_isPPKreverse whether or not be reversed model
 * @return  uint8_t 0 represent failure and 1 represent success
 */
uint8_t dynamicStateTransfer(double* pd_filterX, double* pd_filterQ, gnss_EKFstateRepresentPool_t* pz_statusPool,
                             GpsTime_t z_gpsTime, double d_alphaIn, double d_accSigmaInit, uint8_t u_isPPKreverse);
/**
 * @brief get the index of target parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  w_id
 * @param[out] pw_paraIndex is the index of target parameter in the EKF
 * @return     void
 */
void getTargetParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const uint16_t w_id[GNSS_MAX_STATE_REP], int16_t* pw_paraIndex);
/**
 * @brief get the index of PVA parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_PVAindex is the index of PVA parameter in the EKF
 * @return     void
 */
void getPvaParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
	int16_t pw_PVAindex[PVA_NUM]);
/**
 * @brief get the index of ZTD parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_ZTDindex is the index of ZTD parameter in the EKF
 * @return     void
 */
void getZtdParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, int16_t* pw_ZTDindex);
/**
 * @brief get the index of receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_rcvClkIndex is the index of receiver clock parameter in the EKF
 * @return     void
 */
void getRcvClkParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX]);
/**
 * @brief get the index of PVA ZTD and receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[out] pw_PVAindex is the index of PVA parameter in the EKF
 * @param[out] pw_ZTDindex is the index of ZTD parameter in the EKF
 * @param[out] pw_rcvClkIndex is the index of receiver clock parameter in the EKF
 * @return     1 represent success and 0 represent failure
 */
uint8_t getSiteRelativeParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool,
	int16_t pw_PVAindex[PVA_NUM], int16_t* pw_ZTDindex, int16_t pw_rcvClkIndex[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX]);
/**
 * @brief get the index of BDS3 receiver clock parameter in the EKF
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  u_BDS3clkIndexOffset is the receiver clock for the BDS3 constellation
 * @param[out] pw_bds3RcvClkIndex is the index of BDS3 receiver clock parameter in the EKF
 * @return     void
 */
void getBDS3RcvClkParaIndex(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, uint8_t u_BDS3clkIndexOffset, int16_t pw_bds3RcvClkIndex[C_GNSS_FREQ_TYPE_MAX]);

/**
 * @brief get velocity of horizontal and vertical
 * @param[in]  pz_EKFstateRepPool is the pool of EKF state represent
 * @param[in]  pd_X
 * @param[out] vel_enu
 * @return     void
 */
void getEnuVel(const gnss_EKFstateRepresentPool_t* pz_EKFstateRepPool, const double* pd_X, double* vel_enu);

/**
 * @brief extract covariance from filter
 * @param[in] p_Q covariance
 * @param[in] pd_index index
 * @param[in] n number of index
 * @param[in] p_Qout output
 */
BOOL getPVAQ(const double* p_Q, const int16_t* pd_index, uint8_t n, double *p_Qout);

END_DECL
#endif