/**@file        ppp_type.h
 * @brief       Location Engine PPP Structure Types
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
#ifndef __PPP_TYPE_H__
#define __PPP_TYPE_H__
#include "stdio.h"
#include "cmn_def.h"
#include "gnss_filter_type.h"
#include "gnss_def.h"
#include "gnss_identify_scene.h"

BEGIN_DECL

#define PPP_NO_SSR_CORRECTED            (uint8_t)0x00 /*the satellite not be corrected by SSR*/
#define PPP_SSR_CORRECTED               (uint8_t)0x01 /*the satellite has been corrected by SSR*/

#define PPP_MAX_SSR_AGE   (180.0)         /* max age of ssr */  
#define MAX_A_B(x,y)    ((x)>(y)?(x):(y))

typedef uint8_t ppp_ssrCodeBiasCorrStatus; //the corrected status of satellite code bias 

typedef uint8_t ppp_ssrPhaseBiasCorrStatus;//the corrected status of satellite phase bias 

typedef uint8_t ppp_ssrOrbitCorrStatus;    //the corrected status of satellite orbit

typedef uint8_t ppp_ssrClkCorrStatus;      //the corrected status of satellite clock

typedef uint8_t ppp_ssrIonoCorrStatus;     //the corrected status of ionospheric

typedef uint8_t ppp_ssrZTDcorrStatus;      //the corrected status of ZTD

typedef enum 
{
  PPP_PAR_AR_METHOD_COV = 0,           /* by covariance */
  PPP_PAR_AR_METHOD_RED,               /* by Reduction */
  PPP_PAR_AR_METHOD_ADOP,              /* by ADOP */
  PPP_PAR_AR_METHOD_SNR,               /* by snr */
  PPP_PAR_AR_METHOD_ELE,               /* by elevation */
  PPP_PAR_AR_METHOD_LOCK,              /* by lock */
  PPP_PAR_AR_METHOD_MAX
} PPPAmbFixMethodVal;
typedef uint8_t PPPAmbFixMethodValType;

/** Enum for Residual types */
typedef enum {
  RES_PRIOR = 0,
  RES_POST,
  RES_FIXED,
  RES_PREFIX,
  RES_MAX
} PPP_ResidualTypeEnumVal;
typedef uint8_t PPP_ResidualType;

typedef enum 
{
  AMB_AR_SUCCESS = 0,                   /* AR success */
  AMB_AR_FAIL_Freq,                     /* frequency of amb error */
  AMB_AR_FAIL_RefsatFail,               /* refsat select fail */
  AMB_AR_FAIL_MemError,                 /* memory calloc fail */
  AMB_AR_FAIL_DataError,                /* lack of some data  */
  AMB_AR_FAIL_LessNS,                   /* number of satellite is less than MINAR */
  AMB_AR_FAIL_ParFail,                  /* partial amb ar fail */
  AMB_AR_FAIL_LambdaError,              /* lambda error */
  AMB_AR_FAIL_FloatError,               /* accuracy of float is not enough */
  AMB_AR_FAIL_FIXquality,               /* quality of fix is bad */
  AMB_AR_FAIL_UnknowError,              /* un-know error */
} PPPAmbFixFailInfo;
typedef uint8_t PPPAmbFixFailInfoType;

typedef enum {
	STOCHASTIC_ELE = 0,           /* model of elevation */
	STOCHASTIC_SNR,               /* model of snr */
	STOCHASTIC_ELE_SNR,           /* combined model of ele and snr */
} PPPStochasticModel;
typedef uint8_t PPPStochasticModelType;

/* status mask of algorithm */
typedef enum
{
  PPP_STATUS_UNKNOW              = 0,        /* unkonw fail */
  PPP_STATUS_SUCCESS             = (1 << 0 ), /* run success */

  PPP_STATUS_AR_WLFail           = (1 << 1 ), /* refsat select fail */
  PPP_STATUS_AR_NLFail           = (1 << 2 ), /* number of satellite is less than MINAR */
  PPP_STATUS_STEC_WARNING        = (1 << 3 ), /* ionosphere is active */
  PPP_STATUS_few_SSR3            = (1 << 4 ), /* few product of ionosphere,less than 68% */
  PPP_STATUS_a_few_SSR3          = (1 << 5 ), /* a few product of ionosphere, less than 90% */
  PPP_STATUS_MUCH_GROSS_SPP      = (1 << 6 ), /* too much gross satellites for spp */
  PPP_STATUS_SNR_AVE_35          = (1 << 7 ), /* average SNR is low */

  PPP_STATUS_NO_SSR1             = (1 << 8 ), /* no product of orbit and clk */
  PPP_STATUS_NO_SSR2             = (1 << 9 ), /* no product of ionosphere */
  PPP_STATUS_NO_SSR3             = (1 << 10), /* no product of bias */
  PPP_STATUS_NO_OBS              = (1 << 11), /* no observation */
  PPP_STATUS_NO_NAV              = (1 << 12), /* no navigation ephemeris */

  PPP_STATUS_LITTLE_SSR2         = (1 << 13), /* too little product of bias */
  PPP_STATUS_LITTLE_SAT_SPP      = (1 << 14), /* too little healthy satellites for spp */
  PPP_STATUS_LITTLE_SAT_PPP      = (1 << 15), /* too little healthy satellites for ppp */
  PPP_STATUS_MUCH_GROSS_PPP      = (1 << 16), /* too much gross satellites for ppp */
  PPP_STATUS_FLOAT_AR_Bad        = (1 << 17), /* accuracy of float is not enough for AR */
  PPP_STATUS_EKF_RESET           = (1 << 18), /* reset EKF */
  
  PPP_STATUS_SLIP_WARNING        = (1 << 19), /* too much cycle slip  */

  PPP_STATUS_AR_CheckVel         = (1 << 20), /* velocity of fix is bad */
  PPP_STATUS_AR_CheckDop         = (1 << 21), /* dop of fix is bad */
  PPP_STATUS_AR_CheckWLnoNL      = (1 << 22), /* WL is not good and NL fix fail */
  PPP_STATUS_AR_CheckNLnoWL      = (1 << 23), /* NL is not good and WL fix fail */
  PPP_STATUS_AR_CheckResidual    = (1 << 24), /* residual of fix is bad */
  
  PPP_STATUS_MEMORY_ERROR        = (1 << 25), /* memory is error */
  
}alg_status_e;

/*the option used by PPP algorithm*/
typedef struct
{
  algo_useConstellation z_usedSys; /* define constellation used in the PPP algorithm */
  algo_useFreq          z_usedFreq;/* define frequency used in the PPP algorithm */
  BOOL                  q_isSperateBDS2And3;
  double                d_elmin;   /* elevation mask angle (rad) */
} ppp_alg_opt_t;

/*the previous sat continue flag of clock and phase bias */
typedef struct
{
  gnss_ConstellationType     u_constellation;                               /* see gnss_ConstellationType */
  uint8_t                    u_svid;                                        /* satellite index of the system */
  uint8_t                    u_clockContinuityIod;                          /* IOD to indicate whether there has been a discontinuity in clock estimation */
  uint8_t                    u_signalNum;                                   /* number of u_signalType */
  gnss_SignalType            u_signalType[GNSS_SSR_SAT_CHL_NUM_LIMIT];      /* Follow gnss_SignalType */
  uint8_t                    u_discontinuityIod[GNSS_SSR_SAT_CHL_NUM_LIMIT];/* This IOD indicates whether there has been a discontinuity in the phase bias estimation process */
} ppp_iod_sat_t;

/*the previous continue flag of clock and phase bias */
typedef struct
{
  GpsTime_t z_time;
  uint8_t u_satnum;
  ppp_iod_sat_t iod_array[GNSS_SSR_SAT_NUM_LIMIT];
} ppp_iod_count_t;

typedef struct
{
  ppp_ssrCodeBiasCorrStatus  u_codeBiasCorrStatus;       /* status of code bias correction */
  ppp_ssrPhaseBiasCorrStatus u_phaseBiasCorrStatus;      /* status of phase bias correction */
} ppp_ssrSignalBiasStatus_t;

typedef struct {
  gnss_ConstellationType     u_constellation;            /* see gnss_ConstellationType */
  uint8_t                    u_svid;                     /* satellite index of the system */
  ppp_ssrOrbitCorrStatus     u_orbitCorrStatus;          /* status of orbit correction */
  ppp_ssrClkCorrStatus       u_clkCorrStatus;            /* status of clock correction */
  ppp_ssrIonoCorrStatus      u_IonoCorrStatus;           /* status of ionosphere correction */
  ppp_ssrSignalBiasStatus_t* pz_signalBiasCorrStatus[MAX_GNSS_SIGNAL_FREQ];
  gnss_MeasStatusFlag_t*     z_measUpdateFlag[MAX_GNSS_SIGNAL_FREQ];  /* health flag in filter */
  float                      f_ionResidual;                           /* residual of ionosphere */
  float*                     pf_codeResidual[MAX_GNSS_SIGNAL_FREQ];   /* residual of pseudorange */
  float*                     pf_phaseResidual[MAX_GNSS_SIGNAL_FREQ];  /* residual of phase */
  float*                     pf_codeVar[MAX_GNSS_SIGNAL_FREQ];   /* variance of pseudorange, m */
  float*                     pf_phaseVar[MAX_GNSS_SIGNAL_FREQ];  /* variance of phase, m */
  float*                     pf_codeSigma[MAX_GNSS_SIGNAL_FREQ];  /* sigma of code obs, m */
  float*                     pf_phaseSigma[MAX_GNSS_SIGNAL_FREQ];  /* sigma of phase obs, m */
  uint8_t                    u_lsqFlag[MAX_GNSS_SIGNAL_FREQ];     /* flag of LSQ checking,0: healh, 1:unhealth */
}ppp_SatFilterObs_t;

typedef struct {
  GpsTime_t               z_tor;        /* Time of measurement receive (tor) */
  float                   f_priorCodeSTD;  /* std of code's prior residual */
  float                   f_priorPhaseSTD; /* std of phase's prior residual */
  float                   f_postCodeSTD;   /* std of code's posterior residual */
  float                   f_postPhaseSTD;  /* std of phase's posterior residual */
  float                   f_fixedCodeSTD;   /* std of code's posterior residual */
  float                   f_fixedPhaseSTD;  /* std of phase's posterior residual */
  float                   f_prefixCodeSTD;   /* std of code's posterior residual */
  float                   f_prefixPhaseSTD;  /* std of phase's posterior residual */
  uint8_t                 u_ppp_use_dr_num_float; /* num of dopper used in ppp float */
  uint8_t                 u_ppp_use_pr_num_float; /* num of psedorange used in ppp float */
  uint8_t                 u_ppp_use_cp_num_float; /* num of carrier phase used in ppp float */
  ppp_SatFilterObs_t*     pz_SatFilterObs[ALL_GNSS_SYS_SV_NUMBER];
}ppp_EpochFilterObs_t;     /* information of observation after filter */

typedef struct  /* EKF filter information */
{
  uint8_t               u_start;                  /* status of ppp starting,0:success, other:fail */
  uint8_t               u_ns;                     /* number of satellites used in PPP FLOAT*/
  uint8_t               u_goodSceneStatusFlag;    /* 1 represent receiver locating in good scene, 0 represent receiver locating in complex scene*/
  gnss_FilterPosType    u_solTag;                 /* algorithm filter position type */
  ppp_EKFState          z_kfStatus;               /* KF_INIT,KF_RUN,... */
  uint16_t              w_nmax;
  uint16_t              w_n;
  uint32_t              w_algStatus;              /* status mask of ppp algorithm */
  BOOL                  q_filterPosValid;         /* fliter pos is available*/
  float                 f_age;                    /* age of ssr */
  double*               pd_deltaX;
  double*               pd_X;                     /* x */
  double*               pd_Q;                     /* only store the lower triangle */
  BOOL*                 pq_paraValid;             /*mask the parameter is added to the filter*/
  FILE*                 fp_log;
  FILE*                 fp_nmea;
  ppp_alg_opt_t         z_opt;
  uint8_t               u_tdcpMethodValid;
  double                d_deltaTimeDopplerDetect;
  uint64_t              t_openSkyCount;
  uint64_t              t_closeSkyCount;
  gnss_SceneType        z_sceneType;
  double                pd_phaseCodeClkDiff[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX];
  BOOL                  q_QRcheckStatus;          /* TRUE represent successful and FALSE represent failure  */
  ppp_iod_count_t       z_iod_count;              /* the previous continue flag of clock and phase bias */
} ppp_filterInfo_t;

typedef struct /* input information for amb fix */
{
  const gnss_SatSigMeasCollect_t* pz_satSignalPool;                             /* observation measurement of gnss satellites */
  gnss_EKFstateRepresentPool_t*   pz_EKFstateRepPool;                           /* information of EKF */
  ppp_filterInfo_t*               pz_pppFilterInfo;                             /* result of EKF filter */
  ppp_EpochFilterObs_t*           pz_FilterObs;                                 /* information of observation after filter */
  const gnss_ssrLosBlock_t*       pz_ssrLocBlk;                                 /* block ssr data */
  gnss_fixedAmbType               u_fixTypeIdx;                                 /* index of pz_preFixedAmbPool,  0:NL  1:WL  2:UW */
  gnss_fixedSignalAmbPool_t*      pz_preFixedAmbPool[GNSS_MAX_AMB_FIXED_TYPE];  /* information of previous epoch amb fix, 0:L1/N1, 1:W1, 2:UW */
}gnss_pppAmbFixInputInfo_t; 

/**
 * @brief get ssr data pointer
 * @param[in]    constellation
 * @param[in]    svid
 * @param[in]    signal
 * @param[in]    pz_ssrLocBlk
 * @return pointer
 */
extern const gnss_SignalCorr_t* getSSR_Bias(uint8_t constellation, uint8_t svid, uint8_t signal, const gnss_ssrLosBlock_t* pz_ssrLocBlk);

/**
 * @brief get ssr data pointer
 * @param[in]    constellation
 * @param[in]    svid
 * @param[in]    pz_ssrLocBlk
 * @return pointer
 */
extern const gnss_satSsrLos_t* getSSR_constLos(uint8_t constellation, uint8_t svid, const gnss_ssrLosBlock_t* pz_ssrLocBlk);

/**
 * @brief get ssr data pointer
 * @param[in]    constellation
 * @param[in]    svid
 * @param[in]    pz_ssrLocBlk
 * @return pointer
 */
extern gnss_satSsrLos_t* getSSR_Los(uint8_t constellation, uint8_t svid, gnss_ssrLosBlock_t* pz_ssrLocBlk);

/**
 * @brief sat meas`s singal num, filter NULL
 * @param[in] pz_satMeas
 * @param[in] u_obs_type
 * @return sat meas`s singal num 0~3
 */
extern uint8_t PPP_signalNum(const gnss_SatelliteMeas_t* pz_satMeas, gnss_ObsType u_obs_type);

/**
 * @brief integrity of ssr checking
 * @param[in] post_itg
 * @param[in] pre_itg
 * @return health flag
 */
extern double integrity_check(gnss_integrityFlag post_itg, gnss_integrityFlag pre_itg);

END_DECL
#endif