/**@file        rtk_type.h
 * @brief       Location Engine RTK Structure Types
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/02/19  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __RTK_TYPE_H__
#define __RTK_TYPE_H__
#include "cmn_def.h"
#include "gnss_def.h"
#include "gnss_type.h"
#include "gnss_filter_type.h"
#include "integ_type.h"

BEGIN_DECL

/* flag for ref change and update Amb */
#define REF_AMB_CHANGE_ERROR     (0)        /*for error case, need to reset AMB*/
#define REF_AMB_CHANGE_SUCC      (1)        /*for update success or no need to update*/
#define REF_AMB_CHANGE_POSTPONE  (2)        /*it's a bad time to update amb, postpone the update*/

/* flag of whether rtk sol has significant difference with spp */
#define RTK_DIFF_BETWEEN_POS_NONE     (0)        /* none difference between spp & rtk sol */
#define RTK_DIFF_BETWEEN_POS_SMALL    (1)        /* small difference between spp & rtk sol */
#define RTK_DIFF_BETWEEN_POS_LARGE    (2)        /* large difference between spp & rtk sol */

/* avaliablity of saved position in rtk process */
#define RTK_FILTER_SOL_STAT_NONE       (0)        /* 0:not init yet*/
#define RTK_FILTER_SOL_STAT_SAVE       (1)        /* 1:sol saved but not checked */
#define RTK_FILTER_SOL_STAT_RESET      (2)        /* 2:sol need to reset */
#define RTK_FILTER_SOL_STAT_ERR        (3)        /* 3:sol has obvious mistakes, like *p is null */
#define RTK_FILTER_SOL_STAT_WARN       (4)        /* 4:sol is suspicious*/
#define RTK_FILTER_SOL_STAT_VALID      (5)        /* 5:sol is valid */ 
#define MAX_RTK_FILTER_SOL_STAT        (6)        /* num of RTK_FILTER_POS_STAT */

/* index of saved position in rtk process */
typedef enum
{
  RTK_FILTER_POS_SPP     = 0,     /* 0:SPP  */
  RTK_FILTER_POS_PREDICT = 1,     /* 1:Predict */
  RTK_FILTER_POS_INS     = 2,     /* 2:Ins machinery arrangement */
  RTK_FILTER_POS_RTD     = 3,     /* 3:RTD */
  RTK_FILTER_POS_FLOAT   = 4,     /* 4:Float */
  RTK_FILTER_POS_PREFIX  = 5,     /* 5:Pre-Fix */
  RTK_FILTER_POS_FIX     = 6,     /* 6:Fix */
  RTK_FILTER_POS_FIX_WL  = 7,     /* 7:WideLane FIX */
  RTK_FILTER_POS_FIX_EWL = 8,     /* 8:Extra WideLane FIX */
  MAX_FILTER_POS_STATUS  = 9      /* num of saved pos status in rtk filter */
} rtk_FilterPosEnumTypeVal;

#define MAX_NUM_HISTORY_POS      (6)        /* max num of history pos in rtk filter */

#define RTK_PHASE_FACTOR_A             (0.003)   /* m */
#define RTK_PHASE_FACTOR_B             (0.003)   /* m */
#define RTK_SAT_CLOCK_VEL_SECOND_ERR   (5e-12) /* second/second */
#define RTK_FACTOR_PR_RELATIVE_CR      (100.0) 

#define RTK_FIXTYPE_UNCOMBINED      (0)   /* 0: uncombined mode */
#define RTK_FIXTYPE_WIDELANE        (1)   /* 1: widelane mode */
#define RTK_FIXTYPE_EXTRA_WIDELANE  (2)   /* 2: extra-widelane mode */
#define RTK_FIXTYPE_MAX             (3)   /* 3: MAX */

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
  AMB_AR_FAIL_FixCheck,                 /* check of fix solution not pass  */
  AMB_AR_FAIL_UnknowError,              /* un-know error */
} RTKAmbFixFailInfo;
typedef uint8_t RTKAmbFixFailInfoType;

/** Enum for Residual types */
typedef enum {
  RES_PRIOR = 0,
  RES_POST,
  RES_FIXED,
  RES_PREFIX,
  RES_RTD,
  RES_MAX
} rtk_ResidualTypeEnumVal;
typedef uint8_t rtk_ResidualType;

/* The option used by RTK algorithm */
typedef struct
{
  algo_useConstellation z_usedSys;          /* define constellation used in the RTK algorithm */
  algo_useFreq          z_usedFreq;         /* define frequency used in the RTK algorithm */
  BOOL                  q_isSperateBDS2And3;/* whether BDS2 and BDS3 to sperate */
  double                d_elmin;            /* elevation mask angle (rad) */
  double                d_maxinno;          /* reject threshold of innovation (m) */
  double                d_maxDiffAge;       /* max diff age, 150s */
  double                d_maxPrefixAge;     /* max prefix recursion age, 60s */
  float                 f_sample;           /* sample */
  BOOL                  b_enableStaticConstraints;/* TRUE enable the static constraints and FALSE disable the static constraints*/
  BOOL                  b_enableL1onlyFix;  /*TRUE enable L1-only fix and FALSE disable L1-only fix*/
} rtk_alg_opt_t;

/* save position result of every steps in rtk process */
typedef struct
{
  uint8_t               pu_posValid[MAX_FILTER_POS_STATUS];      /* valid flag of pd_filterPos,
                                                                   See RTK_FILTER_POS_STAT... */
  uint32_t              pq_diffFlagWithSPP[MAX_FILTER_POS_STATUS];/* flag of whether rtk sol has
                                                                  significant difference with spp,
                                                                   See RTK_SPP_DIFF_... */
  uint32_t              pq_diffFlagWithPredict[MAX_FILTER_POS_STATUS];/* flag of whether rtk sol has
                                                                  significant difference with history predict one,
                                                                   See RTK_SPP_DIFF_... */
  double                pd_filterPos[MAX_FILTER_POS_STATUS][3];  /* See RTK_FILTER_POS...
                                                                   0:SPP 1:Predict 2:Ins machinery arrangement
                                                                   3:RTD 4:Float 5:Pre-Fix 6:Fix 7:Fix_WL */
  double                pd_filterVel[MAX_FILTER_POS_STATUS][3];  /* vel of pd_filterPos */
  double                pd_filterPosQ[MAX_FILTER_POS_STATUS][6]; /* var & co-var of pd_filterPos*/
} rtk_filter_sol_t;

/* save history position result in rtk process */
typedef struct
{
  uint8_t               u_num;                                  /* num of saved pos */
  GpsTime_t             z_tor;                                  /* time of last saved pos */
  uint16_t              pu_posType[MAX_NUM_HISTORY_POS];        /* See RTK_FILTER_POS_...*/
  uint32_t              q_DiffFlagWithSPP;                      /* flag of whether history pos has significant diff
                                                                   with spp, every epoch occupy 2 bit */
  double                pd_historyPos[MAX_NUM_HISTORY_POS][3];  /* Save history pos of rtk filter,priority level:
                                                                   1st fixed pos, 2nd float, 3rd rtd, drop the others */
  double                pd_SPPvelENU[MAX_NUM_HISTORY_POS][3];   /* Save history vel of SPP in ENU */
  double                pd_deltaTime[MAX_NUM_HISTORY_POS];      /* delta time between two epoch(this & former) */
} rtk_history_pos_t;

/* the initial information of receiver clock*/
typedef uint8_t rtk_RcvInitClkStatus;
#define RTK_RCV_INIT_CLK_INVALID  (0)
#define RTK_RCV_INIT_CLK_VALID    (1)
typedef struct
{
  rtk_RcvInitClkStatus pu_clkStatus[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX];
  uint32_t             pq_usedObsNum[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX];
  double               pd_initRcvClockValue[C_GNSS_FREQ_TYPE_MAX * C_GNSS_MAX];
}rtk_rcv_init_clk_info_t;

typedef struct  /* EKF filter information */
{
  uint8_t         u_start;                  /* status of RTK starting,0:success, other:fail */
  uint8_t         u_ns;                     /* number of satellites used in RTK FLOAT*/
  uint8_t         u_threeFreqTag;           /* flag of whether date have three freq */
  uint8_t         u_goodSceneStatusFlag;        /* 1 represent receiver locating in good scene, 0 represent receiver locating in complex scene*/
  BOOL            q_QRcheckStatus;          /* TRUE represent successful, and FALSE represent failure */
  rtk_EKFState    z_kfStatus;               /* KF_INIT,KF_RUN,... */
  uint16_t        w_nmax;
  uint16_t        w_n;
  BOOL            q_filterPosValid;         /* filter pos is available*/
  uint8_t         u_solTag;                 /* solution pos status, see MAX_FILTER_POS_STATUS */
  float           f_age;                    /* age of VRS */
  uint32_t        q_StationID;              /* StationID of VRS */
  double          pd_StationCoordinate[3];  /* Station Coordinate of VRS */
  float           f_sample;                 /* background process sample */
  double*         pd_deltaX;
  double*         pd_X;                     /* x */
  double*         pd_Q;                     /* only store the lower triangle */
  BOOL*           pq_paraValid;             /*mask the parameter is added to the filter*/
  rtk_filter_sol_t* pz_filterSol;           /* save position result of every steps */
  rtk_history_pos_t* pz_historyPos_t;         /* save history position position */
  uint8_t         u_prefixStatus;
  rtk_alg_opt_t   z_opt;
  uint8_t         u_tdcpMethodValid;
  double          d_deltaTimeDopplerDetect;
  uint8_t         pu_measUpdatePseudoNum[C_GNSS_FREQ_TYPE_MAX];
  uint8_t         pu_measUpdateCarrierNum[C_GNSS_FREQ_TYPE_MAX];
  uint16_t        w_floatCount;             /* number of continuous epoch enter float filter yet not fix */
  uint16_t        w_rtdCount;               /* number of continuous epoch update float filter yet not float */
  uint16_t        w_filterWithPhaseCount;   /* number of continuous epoch update float filter with phase */
  double          d_resetPercent;           /* Proportion of reset AMB  */
  uint64_t        t_openSkyCount;
  uint64_t        t_closeSkyCount;
  const gnss_PVTResult_t* pz_pvtResult;
  rtk_rcv_init_clk_info_t* pz_rcvInitClkInfo;
  double pd_phaseCodeClkDiff[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX];
  integ_RtkIntegrity_t* pz_integ; 
  GpsTime_t      z_timeLastHold; /* time of last hold */
  BOOL           b_staticFlag;
} rtk_filterInfo_t;

typedef struct {
  gnss_MeasStatusFlag_t*  z_measUpdateFlag[MAX_GNSS_SIGNAL_FREQ];  /* health flag in filter */
  float                   pf_codeResidual[MAX_GNSS_SIGNAL_FREQ];   /* residual of pseudorange */
  float                   pf_phaseResidual[MAX_GNSS_SIGNAL_FREQ];  /* residual of phase */
  float                   pf_codeVar[MAX_GNSS_SIGNAL_FREQ];        /* variance of pseudorange, m */
  float                   pf_phaseVar[MAX_GNSS_SIGNAL_FREQ];       /* variance of phase, m */
  uint8_t                 pu_notNewLock[MAX_GNSS_SIGNAL_FREQ];     /* flag of whether AMB is new spawned in filter */
  uint8_t                 pu_preFixUpdateFlag[MAX_GNSS_SIGNAL_FREQ];  /* pre fix health flag in filter */
}rtk_SatFilterObs_t;

typedef struct {
  GpsTime_t                   z_tor;           /* Time of measurement receive (tor) */
  uint16_t                    w_numPhaseRes;   /* number of phase residual */
  uint16_t                    w_numOverQuarter;/* number of phase residual over quarter wave length */
  float                       f_pdop;          /* pdop of float sol. */
  double                      d_maxCodeBias;  /* std of code's prior residual */
  double                      d_maxPhaseBias; /* std of phase's prior residual */
  double                      d_priorCodeSTD;  /* std of code's prior residual */
  double                      d_priorPhaseSTD; /* std of phase's prior residual */
  double                      d_postCodeSTD;   /* std of code's posterior residual */
  double                      d_postPhaseSTD;  /* std of phase's posterior residual */
  double                      d_fixedCodeSTD;   /* std of code's posterior residual */
  double                      d_fixedPhaseSTD;  /* std of phase's posterior residual */
  double                      d_prefixCodeSTD;   /* std of code's posterior residual */
  double                      d_prefixPhaseSTD;  /* std of phase's posterior residual */
  double                      d_rtdCodeSTD;   /* std of code's post residual */
  uint32_t                    w_groupNUM[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ];  /* num of phase post res devided by sys&freq */
  double                      d_groupSTD[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ];  /* std of phase post res devided by sys&freq */
  rtk_ResidualType            z_resType;         /* flag of pz_SatFilterObs's res type */
  rtk_SatFilterObs_t*		      pz_SatFilterObs[ALL_GNSS_SYS_SV_NUMBER];
  GnssMultiPathFlag_t*		    pz_MPflag;         /* flag of Obs's multipath level */
}rtk_EpochFilterObs_t;     /* information of observation after filter */

#define FIRST_SEARCH_LOOP   0   //first search: L2L5 widelane lambda search; 
#define SECOND_SEARCH_LOOP  1   //second search: L1L5 widelane lambda search; 
#define THIRD_SEARCH_LOOP   2   //third search: L1L2 widelane lambda search; 
#define FOURTH_SEARCH_LOOP  3   //fourth search: full AMB lambda search;
#define FIFTH_SEARCH_LOOP   4   //fifth search: partial AMB lambda search with ele & cn0;
#define SIXTH_SEARCH_LOOP   5   //sixth search: partial AMB lambda search with group devide by freq & sys;
#define SEVENTH_SEARCH_LOOP 6   //sixth search: partial AMB lambda search with mp level;
#define MAX_SEARCH_LOOP     7
typedef struct /* input information for amb fix */
{
  algo_useFreq                    z_fixUsedFreq;                           /* define frequency used in the RTK ambiguity resolution algorithm */
  const gnss_SatSigMeasCollect_t* pz_satSignalPool;                        /* observation measurement of gnss satellites */
  gnss_EKFstateRepresentPool_t*   pz_EKFstateRepPool;                      /* information of EKF */
  rtk_filterInfo_t *              pz_rtkFilterInfo;                        /* result of EKF filter */
  rtk_EpochFilterObs_t*           pz_FilterObs;                            /* information of observation after filter */
  const GnssCorrBlock_t*          pz_OSRblock;                             /* block OSR data */
  uint8_t                         u_fixTypeIdx;                            /* index of pz_preFixedAmbPool */
  uint8_t                         pu_searchLoopNb[MAX_SEARCH_LOOP];        /* the number of ambiguity for difference combination*/
  gnss_fixedSignalAmbPool_t*      pz_preFixedAmbPool[GNSS_MAX_AMB_FIXED_TYPE];  /* information of previous epoch amb fix, 0:L1/N1, 1:W1, 2:UW */
}gnss_rtkAmbFixInputInfo_t;


/**
 * @brief get ssr data pointer
 * @param[in]    u_sys GNSS constellation
 * @param[in]    u_prn satellite index of the system
 * @param[in]    u_signalTargetType the signal type of observations
 * @param[in]    pz_rtkCorrBlock is the OSR correction
 * @return pointer of GnssMeas_t
 */
const GnssMeas_t* getRtkMeasCorr(uint8_t u_sys, uint8_t u_prn, gnss_SignalType u_signalTargetType, const GnssCorrBlock_t* pz_rtkCorrBlock);

/**
 * @brief init the satellite position and clock of the Ref correct data
 * @param[in/out]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @return         void
 */
void rtk_initCorrSatPosClk(GnssCorrBlock_t* pz_GnssCorrBlockUpdate);

/**
 * @brief get the index satellite position and clock of the Ref correct data
 * @param[in]    u_constellation GNSS constellation
 * @param[in]    u_svid satellite index of the system
 * @param[in]    pz_GnssCorrBlockUpdate represent Ref correct data
 * @param[in]    w_satNum represent the satellite number of gnss_SatPosClkInfo_t struct
 * @return       the index satellite position and clock of the Ref correct data
 */
uint16_t rtk_getIndexInCorrSatPosClk(uint8_t u_constellation, uint8_t u_svid, const gnss_SatPosClkInfo_t pz_corrSatPosClk[MAX_GNSS_ACTIVE_SAT_NUMBER] , uint16_t w_satNum);
/**
 * @brief calculate the satellite position and clock of the Ref correct data
 * @param[in/out]  pz_GnssCorrBlockUpdate represent Ref correct data
 * @return         void
 */
void rtk_calCorrDataSatPosClk(GnssCorrBlock_t* pz_GnssCorrBlockUpdate);

/**
 * @brief re-calculate the satellite position and satellite clock of correction data by matching the IODE of GNSS observation data
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[out] pz_GnssCorrBlockUpdate represent Ref correct data
 * @return         void
 */
void rtk_reCalculateCorrDataSatPosClkByRoverIode(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, GnssCorrBlock_t* pz_GnssCorrBlockUpdate);

/**
 * @brief calculate the satellite number that the intersection of observations and VRS
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @param[in]  pz_OSRblock is the VRS observation information
 * @return     the satellite number that the intersection of observations and VRS
 */
uint8_t RTK_intersectionSatNumObsAndVRS(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const GnssCorrBlock_t* pz_OSRblock);

/**
 * @brief malloc the memory of struct rtk_EpochFilterObs_t
 * @param[out]  pz_satSigMeasCollect is the observation information
 * @return     the pointer of rtk_EpochFilterObs_t
 */
rtk_EpochFilterObs_t* malloc_pz_RTKfilterObs(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief free pz_FilterObs
 * @param[in]  pz_FilterObs
 * @return     void
 */
void free_pz_RTKFilterObs(rtk_EpochFilterObs_t** pz_FilterObs);

END_DECL
#endif