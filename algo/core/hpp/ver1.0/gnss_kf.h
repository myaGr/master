/************************************************************
* Copyrights(C)
* All rights Reserved
* 文件名称：rtd_kf.h
* 文件描述：
* 版本号：1.0.0
* 作者：Yongsong.wang
* 日期：2016/08/08
************************************************************/
#ifndef __GNSS__KF__H__
#define __GNSS__KF__H__

#include "gnss.h"
#include "gnss_types.h"
#include "gnss_kf_math.h"
#include "gnss_ls.h"
#include "gnss_config.h"

#ifdef __cplusplus
extern "C" {
#endif

	// constrain define
#define ZUPT


#define h1_1s           0.46826882694955
#define h2_1s           0.90634623461009
#define h3_1s           0.81873075307798

#define h1_200ms        0.01973597880808
#define h2_200ms        0.19605280423838
#define h3_200ms        0.96078943915232

#define h1_100ms        0.0049668326688817590
#define h2_100ms        0.099006633466223737
#define h3_100ms        0.98019867330675525

#define h1_1s_revs       h1_1s
#define h2_1s_revs      (-h2_1s)
#define h3_1s_revs       h3_1s

#define h1_200ms_revs    h1_200ms
#define h2_200ms_revs   (-h2_200ms)
#define h3_200ms_revs    h3_200ms

#define h1_100ms_revs    h1_100ms
#define h2_100ms_revs   (-h2_100ms)
#define h3_100ms_revs    h3_100ms

#define q11_1s          0.01792802206774
#define q12_1s          0.04385513885856
#define q13_1s          0.05469131832920
#define q22_1s          0.11507415690720
#define q23_1s          0.16429269939838
#define q33_1s          0.32967995396436

#define q11_200ms       0.00000625978698
#define q12_200ms       0.00007790177189
#define q13_200ms       0.00051246202946
#define q22_200ms       0.00103525556642
#define q23_200ms       0.00768734040995
#define q33_200ms       0.07688365361336

#define q11_100ms       1.9779357839089237e-07
#define q12_100ms       4.9338853466132293e-06
#define q13_100ms       6.5347885165080430e-05
#define q22_100ms       0.00013135186744603342
#define q23_100ms       0.0019604626940628034
#define q33_100ms       0.039210560847676823

#define MAX_VELOCITY    100

#define MEAS_PR_UPDATE  1
#define MEAS_DR_UPDATE  2
#define COAST_UPDATE    3

#define SBOUND_PR       3
#define SBOUND_PR_1     6
#define SBOUND_DR       2
#define SBOUND_DR_1     4
#define SBOUND_PR_REJ   5
#define SBOUND_PR_OUT   10
#define SBOUND_PR_OUT_1 15
#define SBOUND_DR_OUT   5
#define SBOUND_DR_OUT_1   10

#define KF_MEAS_REJECT   0x00
#define KF_MEAS_UPDATE   0x01
#define KF_MEAS_DEWEIGHT 0x02

#define  MAX_RES_NUM     5
#define  BACK_LLA_NUM    12

#define KF_INIT_POS_LS      0
#define KF_INIT_POS_ACC     1
#define KF_INIT_POS_APPX    2

	typedef struct
	{
		/* biasLinkFlag:  clock bias link flag
		0 -- 4bias;
		1 -- GPS/GLN link;
		2 -- GPS/GLN/BD link;
		3 -- GPS/GLN/BD/Galielo link
		*/
		uint8_t     biasLinkFlag;
		/*
		svModeInUseFlag: SV mode in use flag:4bits
		bit0 -- gps
		bit1 -- gln
		bit2 -- bd
		bit3 -- galileo
		the bit is 0 means not used, 1 means used
		*/
		uint8_t     isPrChiSqTest;
		uint8_t     isDrChiSqTest;
		uint16_t    svModeInUse;
		uint16_t    lastSvModeInUse;
		uint8_t     freeNum;                       /* Real freedom number of PR */
		uint8_t     prNum;
		uint8_t     prNumFLP;
		uint8_t     drNum;
		uint8_t     prRejNum;
		uint8_t     prDewNum;
		uint8_t     drRejNum;
		uint8_t     drDewNum;
	} KF_CTRL;

	typedef struct
	{
		uint8_t                     kfCnt;
		uint8_t                     staticFlag;
		uint8_t                     historyStatic;
		uint8_t                     speedStaticCnt;
		uint8_t                     speedDynamicCnt;
		uint8_t                     lowSpeedMode;
		uint8_t                     avgCnt;
		uint32_t                    historyStaticLong;
		float                    avgVel;
		float                    lsVel,deltaDoplVar,Vel3D;
		float                    staticHeading;
		double                    llaPos_static[3];
		double                    ecefPos_static[3];
	} STATIC_DATA;


	typedef enum
	{
		NO_SOLN,              // No valid solution
		EXTRAPOLATE,          // KF extrapolate
		COAST1,               // Partial position-time solution with 1 channel data set
		COAST2,               // Partial position-time solution with 2 channel data sets
		COAST3,               // Partial position-time solution with 3 channel data sets
		FULL_FIX,             // Full position fix
	} KF_FIX_MODE;

	typedef enum
	{
		KF_STATE_NORMAL,            // KF works at normal state
		KF_STATE_POSBIAS,           // KF pos is bias
	} KF_STATE;

	typedef struct
	{
		uint8_t                    kfFixStatus;
		uint8_t                    kfFixSource;
		uint8_t                    kfFixStatus_last;
		uint8_t                    kfFixSource_last;
		uint8_t                    kf_had_firstFix;
		uint8_t                    staticFlag;
		uint8_t                    biasValid[BIAS_NUM];

		uint8_t                    isAltHold;
		uint8_t                    isHeadingHold;
		uint8_t                    lowVelocity_cnt;

		uint16_t                   kfWeek;
		double                   tor;
		double                   lastTor;
		double                   kfTow;

		KF_FIX_MODE           kfFixMode;
		ERR_EST_TYPE          kfErrEst;
		DOP_TYPE              kfDOP;

		float                   kfHeadingVel;
		float                   kfHeadingDir;
		float                   kfHeadingDir_last;

		double                   kfLLApos[3];
		float                   kfENUvel[3];

		double                   ecefPos[3];
		float                   ecefVel[3];
		float                   ecefPosPmat[6];/* pos var-cov,{c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} */
		float                   enPosCovar;
		double                   clkBias[BIAS_NUM];
		double                   dcb[BIAS_NUM * 2];
		float                   clkDrift;

		double                   ecefPos_last[3];
		float                   ecefVel_last[3];
		double                   llaPos_last[3];
		float                   enuVel_last[3];
		double                   clkBias_last[BIAS_NUM];
		float                   clkDrift_last;

		float                   clkDriftRef;             /* smoothed drift for coast */
		float                   altitudeRef;             /* smoothed altitude for coast */
		float                   headingRef;              /* smoothed heading for coast */
		float                   upVelRef;                /* smoothed up velocity for coast */

		double                   altVar;
		double                   velVarZupt;
		double                   posVarZupt;
		double                   coastTime;
	} KF_PVT_INFO;

	typedef struct
	{
		uint8_t                    flag;                   /* Init flag */
		float                   alpha;          
		float                   beta;
		float                   altitude;
		float                   velUp;
		double                   tor;
	} FiltRefAlt;


	typedef struct
	{
		double     U_plus[N_MAT];              // upper triangular matrix of UD decomposition of P(k) matrix
		double     U_minus[N_MAT];             // upper triangular matrix of UD decomposition of P(k|k-1) matrix
		double     D_plus[N_STATE + 1];        // diagonal matrix of UD decomposition of P(k) matrix
		double     D_minus[N_STATE + 1];       // diagonal matrix of UD decomposition of P(k|k-1) matrix
		double     W1[N_MAT];                  // W1 = A*U, upper triangular matrix
		double     W2[N_MAT];                  // W2 = I(N) when initialization; W = [W1 W2], W is needed in UD KF calculation
	} KF_INFO_BACK;

	typedef struct  
	{
		double     t;
		double     LLApos[3];
		float     ENUvel[3];
	} KF_LLA_BACK;

	typedef struct Kf_tag
	{
		uint8_t                    state;
		uint8_t                    lastKfStatus;        /* last KF status  */
		uint8_t                    lastRtdStatus : 1;                 /* 0: not use RTD; 1: use RTD */
		uint8_t                    isKfRsltGood : 1;                  /* check if KF result is good */
		uint8_t                    enlargeQ :1;
		uint8_t                    enlargeCnt;
		uint8_t                    lsCheckCnt;
		uint8_t                    lsPassCheckCnt;
		uint8_t                    NoZuptStart;
		uint8_t                    NoZuptCnt;
		uint32_t                   kfCnt;
		uint32_t                   kfZuptCnt;
		Ls_t*                 lsBlk;
		meas_blk_t*           meas_blk;
		Gnss_Cfg_t*           pCfg;
		ECEF                  kfEcef;
		LLA                   kfLla;

		KF_CTRL               kf_ctrl;                    /* Used for KF control */
		STATIC_DATA           kf_Static_Data;
		FiltRefAlt            filt_Alt_Data;
		LS_PVT_INFO*          ls_Pvt_Info;
		KF_PVT_INFO*          kf_Pvt_Info;
		KF_PVT_INFO*          kf_Pvt_Back;
		double                   QFactor;
		uint8_t                    kfInitPosType;
		uint8_t                    biasErrFlag[GNSS_MAX_MODE];
		uint8_t                    biasDiffUpCnt[GNSS_MAX_MODE];
		uint8_t                    biasDiffLocalUpCnt[GNSS_MAX_MODE];
		double                   biasDiff[GNSS_MAX_MODE];
		double                   biasDiffLocal[GNSS_MAX_MODE];
		double                   biasDiffInit[GNSS_MAX_MODE];
		double                   biasDiffLocalInit[GNSS_MAX_MODE];
		double                   avgInno;
		double                   stdInno;
		double                   Qp;                 // PSD of position
		double                   Qv;                 // PSD of velocity
		double                   Qa;                 // PSD of acceleration
		double                   posRes;
		double                   posResStd;
		double                   velRes;
		double                   velResStd;
		double                   torOfAdjP;
		float                   enPosCovarBack;
		ERR_EST_TYPE          errBack;
		KF_INFO_BACK          kfInfoBack;
		KF_LLA_BACK           kfLLABack[BACK_LLA_NUM];
		uint8_t                    fre1_satcnt[GNSS_MAX_MODE];
		uint8_t                    fre2_satcnt[GNSS_MAX_MODE * 2];
	} Kf_t;

	void gnss_Kf_Init(Kf_t* p_Kf);
	void gnss_Kf_Main(meas_blk_t* pMeas,Kf_t *p_Kf);
	void gnss_Kf_SetKfPosVel(double *r, double *v);

#ifdef __cplusplus
}
#endif


#endif