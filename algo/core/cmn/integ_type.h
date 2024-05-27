/**@file        integ_type.h
 * @brief       Location Engine Integrity Structure Types
 * @details     
 * @author      houxiaowei
 * @date        2023/5/17
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/5/17   <td>0.1      <td>houxiaowei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __GNSS_ENGINE_INTEGRITY_TYPE_H__
#define __GNSS_ENGINE_INTEGRITY_TYPE_H__

#include "gnss_def.h"
#include "gnss_type.h"
#include "gnss_identify_scene.h"

#define SMT_FIXUNCHIGHB    1.2
#define SMT_FIXUNCLOWB    0.3
#define SMT_FLTUNCHIGHB    7.0
#define SMT_FLTUNCLOWB    3.0
#define SMT_SPPUNCHIGHB 10.0
#define SMT_SPPUNCLOWB    5.0

#define MAXUNCBUFF	50					/* max uncertainty buff*/

typedef struct
{
  uint8_t smtlen;			  // exact smt window length
  uint8_t smtlen0;			// when smt start
  uint8_t smting;
  uint8_t uncnum;
  uint8_t state_buff[MAXUNCBUFF];
  double unc0;
  double unc_smt;
  double unc_buff[MAXUNCBUFF];   /* uncertainty window */
}integ_Uncsmt_t;

/** Enum for the function to calculate protection level */
typedef enum
{
  C_INTEGRITY_FUN_FILTER = 0,
  C_INTEGRITY_FUN_RAIM,
  C_INTEGRITY_FUN_DECISION_TREE,
  C_INTEGRITY_FUN_NEURAL_NETWORK,
  C_INTEGRITY_FUN_MAX,
} integ_FunTypeEnumVal;
typedef uint8_t integ_FunType;

/** Config for integrity  */
typedef struct
{
  uint8_t u_fun_type; /* see integ_FunType */
  uint8_t u_valid;    /* see integ_LaunchType */
} integ_Config_t;

/** status of integrity  */
typedef enum
{
  C_INTEGRITY_STATUS_RESERVE = 0,
} integ_StatusTypeEnumVal;
typedef uint8_t integ_StatusType;

/** Launch valid */
typedef enum
{
  C_INTEGRITY_LAUNCH_SET = 0,  /* bit 0 */
  C_INTEGRITY_LAUNCH_INIT,     /* bit 1 */
} integ_LaunchTypeEnumVal;
typedef uint8_t integ_LaunchType;

/** protection level result */
typedef struct
{
  gnss_FixFlagType u_fixFlagType;
  float f_hv_protection_level[2];   // horizontal and vertical protection level
  float f_enu_protection_level[3];  // enu protection level
  float f_xyz_protection_level[3];  // xyz protection level
  uint8_t u_sceneType; //1: open sky; 2: not open sky but position error is ok; 3: semi sky; 4: close sky
} integ_PLResult_t;

/** Signal integrity information block */
typedef struct
{
  uint8_t u_valid: 1;         /* 1: Invalid, 1:Valid  */
  uint8_t u_frq;              /* gnss_FreqType */
  double pd_H[P_NUM];         /* coefficient of position */
  double pd_K[P_NUM];         /* gain of position */
  double d_R;                 /* observation noise */
  double d_pz;                /* H(k)* P(k)_* H(k)'+R(k) */
  double d_obs_res;           /* residual float filter */
  double d_obs_res_fix;       /* residual fix filter  */
  double d_effective;         /* H@K */
  double d_weight;
} integ_SignalInfo_t;

/** Sat integrity information block */
typedef struct
{
  gnss_ConstellationType u_constellation;
  uint8_t u_svid;
  integ_SignalInfo_t z_signalInfo[MAX_GNSS_SIGNAL_FREQ];    /* C_GNSS_FREQ_TYPE_MAX */
} integ_SatInfo_t;

typedef struct
{
  double d_nm1;
  double d_rm2;            /* magnification factor of the weight */
  double d_Rm[9];
  double d_Am[3];
  double d_U[9];           /*  */
} integ_Stored_t;

/** Block integrity information  */
typedef struct
{
  GpsTime_t z_tor;                 /* Time of measurement receive (tor) */
  uint8_t u_fixFlag;
  double d_lla[3];
  double d_neuRotation[9];
  double d_QEcefPos[9];
  gnss_PositionFix_Dops_t z_dops;
  uint8_t u_satMeasCount;
  integ_SatInfo_t z_satIntInfo[MAX_GNSS_ACTIVE_SAT_NUMBER];
  integ_Stored_t z_stored;
} integ_InfoBlock_t;

/** data for machine learning */
typedef struct
{
  GpsTime_t z_tor;
  uint8_t u_fixFlag;             /* see: gnss_FixFlagType */
  gnss_PositionFix_Dops_t z_dops;
  gnss_SceneType z_sceneType;    /* scene type */
  gnss_fixedAmbType u_fixStatus; /* see gnss_fixedAmbType, GNSS_W1_AMB_FIXED, GNSS_L1_AMB_FIXED... */
  uint8_t u_ppp_use_dr_num_float;
  uint8_t u_ppp_use_cp_num_float;
  uint8_t u_ppp_use_pr_num_float;
  uint8_t u_ppp_use_sat_num_wideLane;
  uint8_t u_ppp_use_sat_num_fix;
  uint8_t u_CN040;               /* Sat number fre1 cn0>40  */
  uint8_t u_fixedns[C_GNSS_MAX];
  uint8_t u_fixedTotalSatCount;  /* number of fixed satellite */
  uint8_t u_nb;                  /* number of fixed bias */
  uint8_t u_satMeasCount;
  uint8_t u_ns;                  /* number of satellites used in PPP FLOAT*/
  uint32_t w_continuefixedEpoch;
  uint64_t t_openSkyCount;
  uint64_t t_closeSkyCount;
  float f_ratio;
  float f_cn0_avg;               /* cn0  average */
  float f_cn0_std;               /* cn0 standard */
  float f_cmc_avg;               /* code-minus-carrier average */
  float f_cmc_std;               /* code-minus-carrier standard */
  float f_avg_unc_pr;            /* empirical uncertain variance */
  float f_velEnu[3];             /* Velocity: lat, lon, alt */
  float f_posLlaUnc[3];          /* Uncertainy position LLA  (m) */
  float f_ambPool_adop;
  float f_ambPool_pdop;
  float f_age;                   /* age of ssr */
  float f_priorCodeSTD;          /* std of code's prior residual */
  float f_priorPhaseSTD;         /* std of phase's prior residual */
  float f_postCodeSTD;           /* std of code's posterior residual */
  float f_postPhaseSTD;          /* std of phase's posterior residual */
  float f_fixedCodeSTD;          /* std of code's posterior residual */
  float f_fixedPhaseSTD;         /* std of phase's posterior residual */
  double d_lambda_sigma[2];      /* LAMBDA sum of squared residual, 0:best, 1: second best */
  double pd_phaseCodeClkDiff[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX];
  BOOL q_QRcheckStatus;          /* TRUE represent successful and FALSE represent failure  */
  integ_PLResult_t z_pl_ekf_result;
  gnss_PositionFix_SvStatus_t  z_SvStatus;
  integ_Uncsmt_t integ_uncsmt;

  /* feature for machine learning scene */
  uint16_t w_cn0Num;
  uint16_t w_LLInum;
  uint16_t w_noLLInum;
  float f_max_CN0;
  uint8_t pu_LLIsatNum[C_GNSS_MAX];
  uint8_t pu_noLLIsatNum[C_GNSS_MAX];
  uint8_t pu_satNumPerSys[C_GNSS_MAX];
  uint8_t pu_CN0greater35NumPerSys[C_GNSS_MAX];
  uint8_t u_sysNum;
}integ_PppFeature_t;

typedef struct
{
  uint8_t        u_status;            /* 1:success */
  uint8_t        u_fix_sat_number;    /* fixed satllite number */
  uint8_t        u_phase_number;
  uint8_t        u_code_number;
  uint8_t        u_avg_lock;       /* average lock */
  uint8_t        u_ref_lock;       /* reference satellite lock  */
  float          f_phase_uwrmse;   /* std unit weight of carrier-phase RTK [0]:FIX, [1]: FLOAT */
  float          f_code_uwrmse;
  float          f_avg_cn0;        /* average cn0 */
  float          f_ratio;          /* ratio for ratio */
  float          f_rss[2];         /* LAMBDA sum of squared residual */
  float          f_adop;
  float          f_pdop;
  float          f_uwrmse;         /* LSQ Unit Weighted Root Mean Square Error, weight=1 */
}integ_RtkWLFeature_t;

/* rtk ML feature for calculate protection level */
typedef struct
{
  gnss_FixFlagType          u_fixFlag;        /* fix flag */
  gnss_FilterPosType        u_solTag;         /* solution pos status, see GNSS_FILTER_POS_STATUS */
  uint16_t       w_sol_bit;                   /* if pos is valid gnss_FilterPosType */
  uint8_t        u_common_sat;                /* common satellite between rover and base */
  uint8_t        u_scene;                     /* scenario, 1:opensky, 0:other */
  float          f_cmc;                       /* TODO(houxiaowei): multipath */
  float          f_pdop;
  float          f_pdop_ekf_pr;               /* PDOP code */
  float          f_pdop_ekf_cp;               /* PDOP phase */
  uint8_t        u_opensky_count;             /* count for open sky in 16 epochs */
  uint8_t        u_closesky_count;            /* count for close sky in 16 epochs */
  uint8_t        u_dr_number;                 /* number of DR update EKF */
  float          u_doppler_uwrmse;            /* std unit weight of doppler */
  uint8_t        u_sat_number[2];             /* number of satellite [0]:FIX, [1]: FLOAT */
  uint8_t        u_phase_number[2];           /* number of phase in [0]:FIX, [1]: FLOAT */
  uint8_t        u_code_number[2];            /* number of code [0]:FIX, [1]: FLOAT */
  float          f_phase_uwrmse[2];           /* std unit weight of carrier-phase RTK [0]:FIX, [1]: FLOAT, (houxiaowei):STD */
  float          f_code_uwrmse[2];            /* std unit weight of pseudo-range RTK fix or float,  [0]:FIX, [1]: FLOAT, houxiaowei:STD  */
  float          f_avg_lock[2];               /*  average lock phase num, [0]:FIX, [1]:FLOAT  */
  float          f_avg_cn0[2];                /*  average CN0, [0]:FIX, [1]:FLOAT  */
  float          f_ratio;
  float          f_pdop_fix;
  float          f_adop_fix;
  float          f_rss[2];                    /* LAMBDA sum of squared residual */
  float          f_kf_std[2];                 /* std of horizontal/vertical in float single difference EKF */
  uint8_t        u_gap_max;                   /* max amb diff in FLOAT */

  /* wl */
  integ_RtkWLFeature_t z_wl[GNSS_FILTER_POS_STATUS];

  /* prefix */
  float          f_previous_ratio;
  uint8_t        u_previous_fix_sat;
  float          f_prefix_age;
  float          f_prefix_uwrmse;
  float          f_prefix_ls_std[2];  /* prefix lsq STD in  horizontal/vertical */
  uint8_t        u_prefix_sat_number;
  uint8_t        u_prefix_sig_number;
  float          f_prefix_pdop;
}integ_RtkFeature_t;

/** RTK integrity interface struct */
typedef struct
{
  integ_Config_t    config;            /* rtk integrity configure */
  /* feature */
  GpsTime_t          z_tor;
  uint8_t            u_fix_count;      /* count for GNSS_FILTER_POS_FIX latest 20 epochs */
  uint8_t            u_sol_bit_count;  /* count for u_sol_bit, MAX=20 HISFATTHRES */
  uint8_t            u_ekf_count;      /* count EKF run */
  uint16_t           w_pre_sol_bit;    /* previous sol bit be sett if pos is valid, gnss_FilterPosType*/
  uint64_t           t_seq_open_sky;   /* sequence of  */
  uint64_t           t_seq_close_sky;  /* sequence of  */
  integ_RtkFeature_t z_rtk_feature;    /* rtk integrity feature */
  integ_Uncsmt_t     z_smt;            /* sliding window */
}integ_RtkIntegrity_t;


#endif //__GNSS_ENGINE_INTEGRITY_TYPE_H__
