/************************************************************
* Copyrights(C) ǧѰλ���������޹�˾
* All rights Reserved
* �ļ����ƣ�gnss_ls.h
* �ļ�������
* �汾�ţ�1.0.0
* ���ߣ�Yongsong.wang
* ���ڣ�2016/08/08
************************************************************/
#ifndef __GNSS__LS__H__
#define __GNSS__LS__H__

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include "gnss.h"
#include "gnss_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PR_WEIGHT_FACTOR   5.0
#define DR_WEIGHT_FACTOR   5.0
	// Define the LS converge threshold and iteration time limits
#define LS_POS_CONVERGE_THRES     1e-3
#define LS_VEL_CONVERGE_THRES     5e-4
#define LS_POS_MAX_ITERA          10      /*max LS iteration loops when calculating position */
#define LS_VEL_MAX_ITERA          10       /*max LS iteration loops when calculating velocity */

	// Define the RAIM threshold in LS 
#define LS_POS_RAIM_THRES1        500


	/* Define the state of LS */
#define LS_STATUS_HAS_NONE        0
#define LS_STATUS_HAS_2D_POS      (1<<0)
#define LS_STATUS_HAS_3D_POS      (1<<1)
#define LS_STATUS_HAS_2D_VEL      (1<<2)
#define LS_STATUS_HAS_3D_VEL      (1<<3)

	// Define the clock status
#define LS_CLOCK_BIAS_UNKNOWN           0
#define LS_CLOCK_GPS_BIAS_KNOWN        (1<<0)
#define LS_CLOCK_GLN_BIAS_KNOWN        (1<<1)
#define LS_CLOCK_BDS_BIAS_KNOWN        (1<<2)
#define LS_CLOCK_GAL_BIAS_KNOWN        (1<<3)
#define LS_CLOCK_DRIFT_KNOWN           (1<<4)

	// Define the LS process type
#define LS_POS_ID                  1
#define LS_VEL_ID                  2

#define LS_RAIM_NO_ERROR           (0)               // Didn't find error for PR or DR
#define LS_RAIM_FOUND_ERROR        (1<<0)               // Found some error for PR or DR
#define LS_RAIM_NEED_MORE          (1<<1)               // need another RAIM no matter finding error or not 

#define DR_RAIM_LOOP_MAX           2
#define PR_RAIM_LOOP_MAX           7
	// Define the PR and DR RAIM status flag

	typedef struct
	{
		uint8_t       fix_mode;
		float      hDOP, vDOP, tDOP, pDOP;
	} LS_DOP_TYPE;

	typedef struct
	{
		float     pErr, hErr, vErr, tErr, latErr, lonErr;
	} LS_ERR_EST_TYPE;

	typedef struct
	{
		uint8_t                 valid;
		uint8_t                 last_valid;
		uint8_t                 fix_status;
		uint8_t                 fix_dim;
		uint8_t                 isDriftValid : 1;
		uint8_t                 isBiasLink;
		uint8_t                 biasValid[BIAS_NUM];
		uint8_t                 dcbValid[BIAS_NUM*2];
		double                lla[3];
		double                ecefPos[3];
		float                ecefVel[3];
		float                enuVel[3];
		float                clkBias[BIAS_NUM];
		float                clkDcb[BIAS_NUM*2];
		float                clkDrift;
		float                heading;

		double                LLApos[3];

		uint8_t                 biasValid_last[BIAS_NUM];
		double                ecefPos_last[3];
		float                ecefVel_last[3];
		float                clkBias_last[BIAS_NUM];

		double                tor;
		double                lastTor;
		LS_ERR_EST_TYPE    err;
		double                pos_res;
		double                vel_res;
	} LS_PVT_INFO;


	typedef struct  
	{
		uint8_t                prRaimFlag;                      /* PR RAIM flag */
		uint8_t                drRaimFlag;                      /* DR RAIM flag */
		uint8_t                prRaimLoop;                      /* PR RAIM loop count */
		uint8_t                drRaimLoop;                      /* DR RAIM loop count */
		uint8_t                res_cnt;
		uint8_t                loop_cnt;
		float               *pos_res;
		float               *vel_res;
		uint8_t                *pos_indx;
		uint8_t                *vel_indx;
	}Ls_Raim_Ctrl_t;

	typedef struct Ls_Ctrl_tag
	{
		uint16_t                satModeCnt;                   /* the sat system used in Pos/Vel */
		uint8_t                 lsBiasNum;                    /* Bias Number used in LS POS */
		uint8_t                 lsDcbNum[2];                     /* DCB Number used in LS POS */
		uint8_t                 lsDcbCalFlag[BIAS_NUM*2];		 /*whether calculate DCB flag*/
		uint8_t                 is2DPos : 1;                  /* If 2D position is used */
		uint8_t                 isForce2D : 1;          
		uint8_t                 is2DVel : 1;                  /* If 2D velocity is used */
		uint8_t                 velUpdateFlag: 1;            /* could do vel fix or not */
		uint8_t                 isPrRaim : 1;                 /* PR RAIM flag */  
		uint8_t                 isDrRaim : 1;                 /* DR RAIM flag */
		uint8_t                 isLocalBias : 1;                /* If use RTD PR */
		uint8_t                 isBiasLink : 1;                /*LS bias link or not*/
		uint8_t                 biasIndx[4]; 
		uint8_t                 dcbIndx[8];
		uint8_t                 prNum;                        /* Used PR number */
		uint8_t                 drNum;                        /* Used DR number */
		uint8_t                 validSvNum[GNSS_MAX_MODE];    
		uint8_t                 prUsedSV[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];       /* the index of PR */
		uint8_t                 drUsedSV[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];       /* the index of DR */
		uint8_t                 clockStatus;                  /* 0:Bias status */
		uint8_t                 init_qos_crl_cnt;             /* count of initial LS qos control*/
		uint16_t                status;                       /* LS status */
		uint16_t                lastStatus;                   /* Last epoch LS status */
		uint32_t                lsCnt;                        /* count of continuous valid LS result */
		uint32_t                lsInvalidCnt;                 /* count of continuous invalid LS result */
		float                aidHeight;                    /* height used for 2D position */
		float                aidUpVel;                     /* up velocity used for 2D velocity */
		double                tor;                          /* TOR of this epoch */
		double                lastTor;                      /* TOR of last epoch */
	} Ls_Ctrl_t;

	typedef struct 
	{
		uint8_t                 init;
		float                coef[2];
		ECEF               fitEcef;
	} Ls_Fit_t;
	/* Define the structure for LS */
	typedef struct Ls_tag
	{
		uint8_t                 posCvg;                      /* Position converge state */
		uint8_t                 velCvg;                      /* Velocity converge state */
		float                drift;
		float                drift_unc;
		float                prePrRes[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];      /* the prefix PR residual */
		float                preDrRes[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];      /* the prefix DR residual */
		float                postPrRes[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];     /* the postfix PR residual */
		float                postDrRes[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];     /* the postfix DR residual */
		float                prWeight[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];      /* Define the PR weight for LS */
		float                drWeight[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];      /* Define the DR weight for LS */
		float                pos_res;
		float                vel_res;
		float                vel_enu[3];
		double                bias[BIAS_NUM];
		double                dcb[BIAS_NUM*2];
		double                biasLocal[BIAS_NUM];
		double                bias_unc[BIAS_NUM];
		double                biasDiff[GNSS_MAX_MODE];
		double                biasDiffLocal[GNSS_MAX_MODE];
		uint8_t                 biasDiffUpCnt[GNSS_MAX_MODE];
		uint8_t                 biasDiffLocalUpCnt[GNSS_MAX_MODE];
		float                vel_std;
		meas_blk_t *       meas;                       /* meas for LS */
		Gnss_Cfg_t *       pCfg;
		LS_PVT_INFO*       ls_Pvt_Info;
		LS_PVT_INFO        ls_Pvt_Back;
		ECEF               lsEcef;                      /* LS ECEF results */
		LLA                lsLla;                       /* LS LLA results */
		Ls_Ctrl_t          lsCtrl;                      /* LS control block */
		Ls_Raim_Ctrl_t     raimCtrl;
		DOP_TYPE           DoP;
		LS_ERR_EST_TYPE    err_est;
		Ls_Fit_t           fitRst;
	} Ls_t;




	/* Functions for LS */
	void gnss_Ls_Init(Ls_t* pLs);
	void gnss_Ls_Prepare(meas_blk_t* pMeas,Ls_t* pLs);
	void gnss_Ls_Main(meas_blk_t* pMeas,Ls_t* pLs);
	void gnss_Ls_Pos_Raim(Ls_t* pLs);
	void gnss_Ls_Vel_Raim(Ls_t* pLs);
	uint8_t gnss_Ls_Pos_Raim_SatSys(Ls_t* pLs);
	void gnss_Ls_Bias_Raim(Ls_t* pLs);
	uint8_t gnss_Ls_Pos(Ls_t* pLs);
	uint8_t gnss_Ls_Vel(Ls_t* pLs,uint8_t flag);
	uint8_t gnss_Ls_RaimThres_Chck(Ls_t* pLs, uint8_t lsType,double* thresHold);
	void gnss_LS_BiasLink_Check(Ls_t* pLs);
	void gnss_Ls_Check_BiasNum(Ls_t* pLs);
	void gnss_Ls_BiasCorrection(uint8_t satMode, int32_t biasCorrection,Ls_t* pLs);
	void gnss_Ls_Filter(Ls_t* pLs);
	void gnss_ls_dynamic_ele_pre(Ls_t* pLs);
#ifdef __cplusplus
}
#endif


#endif
