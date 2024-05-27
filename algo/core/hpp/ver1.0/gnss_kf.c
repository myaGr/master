#include <string.h>
#include "gnss_kf.h"
#include "gnss_kf_math.h"
#include "gnss_sys_api.h"
#include "gnss_math.h"
#include "gnss_mode.h"
#include "gnss_kf_algo.h"
#include "gnss_pe.h"
#include "gnss_sd.h"
#include "gnss_def.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_KF

extern PE_MODES    peMode;
extern KF_INFO     kfInfo;
extern USER_PVT    user_pvt_backup;
extern history_Info    HisInfo;
//extern AGFusionResults     DRFusionRslt;
extern uint8_t firstFix;
extern uncertainty_t g_pvtunc;
extern VDR_Gnss_Feedback_t   vdrFeedbackInfo;
//biasLinkFlag: 0 -- 4bias; 1 -- GPS/GLN link; 2 -- GPS/GLN/BD link; 3 -- GPS/GLN/BD/Galielo link//                             0  1  2  3
const uint8_t clockBiasIdx[BIAS_NUM][BIAS_NUM] =
{{6, 6, 6, 6},        //gps
{7, 6, 6, 6},        //gln
{8, 8, 6, 6},        //bds
{9, 9, 9, 6}         //galileo
};  

//clockBiasBitMask: 1111b -- 4bias; 1101b -- GPS/GLN link; 1001b -- GPS/GLN/BD link; 0001b -- GPS/GLN/BD/Galielo link
const uint16_t clockBiasBitMask[BIAS_NUM] = {4095, 13, 9, 1};

#define NOISE_SCALE          64
#define ALT_CHECK_INNO 
#define KF_USE_HISTORY_TIME  30
#define KF_USE_HISTORY_TIME_AUTO  300

extern Gnss_Cfg_t g_pe_cfg;
extern PeStateMonitor_t peState;
//#define NOISE_ONE

void gnss_Kf_SetKfPosVel(double *r, double *v)
{
	int i;
	if (r)
	{
		for (i=0;i<3;i++) kfInfo.X[i]=r[i];
	}
	if (v)
	{
		for (i=0;i<3;i++) kfInfo.X[i+3]=v[i];
	}
}

/***********************************************************************
* 函数介绍: gnss_kf_Init
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
void gnss_Kf_Init(Kf_t* p_Kf)
{
	memset(&kfInfo,0,sizeof(KF_INFO));
	kfInfo.status = KF_INIT;
	memset(p_Kf,0,sizeof(Kf_t));
	p_Kf->kf_ctrl.biasLinkFlag = 0;
}
/**********************************************************************
* Function Name:    gnss_Kf_AmatInit
*
* Description:
*    Initialize the A matrix of KF
*
* Input:
*    Tms:       KF update time (ms)
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_AmatInit(double Tin)
{
	uint8_t          i;
	double         T;
#ifndef PVMODE
	double         h1, h2, h3,a;
#endif
	T = Tin;
#ifndef PVMODE
	a = 0.2;
	/* initialize A matrix */

	if (T == 1.000)
	{
		h1 = h1_1s;
		h2 = h2_1s;
		h3 = h3_1s;
	}
	else if (T == 0.200)
	{
		h1 = h1_200ms;
		h2 = h2_200ms;
		h3 = h3_200ms;
	}else
	{
		h1 = 1 / (a*a) * (-1 + a * T + exp(-a * T));
		h2 = 1 / a * (1 - exp(-a * T));
		h3 = exp(-a * T);
	}
#endif
	memset(kfInfo.A, 0, sizeof(double)*N_MAT);
	uMatInit(kfInfo.A, N_STATE);

	for (i = 0; i < 3; i++)
	{
		kfInfo.A[uMatIdx(i + 1, i + 4, N_STATE)] = T;
#ifndef PVMODE
		kfInfo.A[uMatIdx(i + 1, i + 12, N_STATE)] = h1;
		kfInfo.A[uMatIdx(i + 4, i + 12, N_STATE)] = h2;
		kfInfo.A[uMatIdx(i + 12, i + 12, N_STATE)] = h3;
#endif
	}

	for (i = 0; i < 4; i++)
	{
		kfInfo.A[uMatIdx(i + 7, 11, N_STATE)] = T;
	}
}

/***********************************************************************
* 函数介绍: gnss_Kf_QmatSet
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
void gnss_Kf_QmatSet(double Tin, double* q)
{
	double        a = 0.2;
	double        T = Tin;

	if (T == 1.000)
	{
		q[0] = q11_1s;
		q[1] = q12_1s;
		q[2] = q13_1s;
		q[3] = q22_1s;
		q[4] = q23_1s;
		q[5] = q33_1s;

	}else if (T == 0.200)
	{
		q[0] = q11_200ms;
		q[1] = q12_200ms;
		q[2] = q13_200ms;
		q[3] = q22_200ms;
		q[4] = q23_200ms;
		q[5] = q33_200ms;
	}else
	{
		q[0] = 1 / pow(a,4) * (1 - exp(-2*a*T) + 2 * a * T + 2 * pow(a,3)*pow(T,3) / 3 - 2 * pow(a,2)*pow(T,2) - 4 * a * T * exp(- a * T));
		q[1] = 1 / pow(a,3) * (exp(-2*a*T) + 1 - 2 * exp(-a * T) + 2 * a * T * exp(-a * T) - 2 * a * T + pow(a,2) * pow(T,2));
		q[2] = 1 / pow(a,2) * (1 - exp(-2 * a * T) - 2 * a * T * exp(-a * T));
		q[3] = 1 / pow(a,2) * (4 * exp(-a*T) - 3 - exp(-2*a*T) + 2*a*T);
		q[4] = 1 / pow(a,1) * (exp(-2 * a * T) + 1 - 2 * exp(-a * T));
		q[5] = (1 - exp(-2*a*T));
	}
}
/**********************************************************************
* Function Name:    gnss_Kf_QmatInit
*
* Description:
*    Initialize the Q matrix of KF
*
* Input:
*    Tms:       KF update time (ms)
*
* Return:
*
* Dependency
*      None
*
* Author: 
* 
**********************************************************************/
void gnss_Kf_QmatInit(Kf_t* p_Kf,double T,float* accSigmaInit)
{
	uint8_t             i;
	double            q11, q12, q13, q22, q23, q33;
	double            qdf1, qdf2, qdf3, qdf4, qdf5;
	float            accSigma;
#ifndef PVMODE
	static double     q[6];
	float            a_enu[3],a_ecef[3];
#endif
	meas_blk_t*    pMeas;
	STATIC_DATA*   kfStaticData;

	kfStaticData = &(peMode.staticData);
	pMeas = p_Kf->meas_blk;

	if (accSigmaInit)
	{
		accSigma = *accSigmaInit;
	}else
	{
		accSigma = 1;
	}

	/* constrain of T*/
	if (!IS_PPK_REVERSE && T > 1.0)
	{
		T = 1.0;
	}
	else if (IS_PPK_REVERSE && T < -1.0)
	{
		T = -1.0;
	}

#if 0
	if (peMode.userContext.context == USER_CONTEXT_HAND)
	{
		qdf1 = (7 / 300.0) * 300.0;
		qdf2 = (1 / 50.0) * 5.0;
		qdf3 = (1 / 50.0) * 5.0;
		qdf4 = (1 / 25.0) * 50.0;
	}else
	{
		qdf1 = (7 / 300.0) * 300.0;
		qdf2 = (1 / 50.0) * 5.0;
		qdf3 = (1 / 50.0) * 5.0;
		qdf4 = (1 / 25.0) * 50.0;
	}

#endif
	if (g_pe_cfg.chipType == QCOM)
	{
		if (pMeas->avgCno >= 20)
		{
			qdf1 = (7 / 300.0) * 300.0;
			qdf2 = (1 / 50.0) * 50.0;
			qdf3 = (1 / 50.0) * 50.0;
			qdf4 = (1 / 25.0) * 50.0;
		}
		else
		{
			qdf1 = (7 / 300.0) * 150.0;
			qdf2 = (1 / 50.0) * 25.0;
			qdf3 = (1 / 50.0) * 25.0;
			qdf4 = (1 / 25.0) * 25.0;
		}
	}
	else
	{
		if (!IS_PPK_REVERSE)
		{
			qdf1 = 0.5*(7 / 300.0) * 150.0 * T;
			qdf2 = 0.5*(1 / 50.0) * 25.0 * T * T;
			qdf3 = 0.5*(1 / 50.0) * 25.0 * T * T;
			qdf4 = 0.5*(1 / 25.0) * 25.0 * T;
		}
		else
		{
			qdf1 = 0.5*(7 / 300.0) * 150.0 * fabs(T);
			qdf2 = -0.5*(1 / 50.0) * 25.0 * T * T;
			qdf3 = -0.5*(1 / 50.0) * 25.0 * T * T;
			qdf4 = 0.5*(1 / 25.0) * 25.0 *fabs(T);
		}

		if (!(g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR))
		{
			qdf1 *= 10;
			qdf2 *= 10;
			qdf3 *= 10;
			qdf4 *= 10;
		}
	}

	if(pMeas->rtdUsed != p_Kf->lastRtdStatus && g_pe_cfg.chipType == SPRD)
	{
		qdf1 = (7 / 300.0) * 300.0;
		qdf2 = (1 / 50.0) * 50.0;
		qdf3 = (1 / 50.0) * 50.0;
		qdf4 = (1 / 25.0) * 50.0;
	}

	qdf5 = 0.0001;

	SYS_LOGGING(OBJ_KF,LOG_INFO,"%s %4d accSigma: %f",__FUNCTION__,p_Kf->kfCnt,accSigma);

#ifndef PVMODE

	gnss_Kf_QmatSet(fabs(T), &q[0]);

	q11 = q[0] * p_Kf->Qa;
	q12 = q[1] * p_Kf->Qa;
	q13 = q[2] * p_Kf->Qa;
	q22 = q[3] * p_Kf->Qa;
	q23 = q[4] * p_Kf->Qa;
	q33 = q[5] * p_Kf->Qa;
#else
	
	if (!IS_PPK_REVERSE)
	{
		q11 = p_Kf->Qp * T + 0.3 * p_Kf->Qv *T*T*T;
		q12 = 0.5 * p_Kf->Qv*T*T;
		q22 = 1.0 * p_Kf->Qv *T;
		q13 = 0.0;
		q23 = 0.0;
		q33 = 0.0;
	}
	else
	{
		q11 = p_Kf->Qp * fabs(T) + 0.3 * p_Kf->Qv *fabs(T*T*T);
		q12 = -0.5 * p_Kf->Qv*T*T;
		q22 = 1.0 * p_Kf->Qv *fabs(T);
		q13 = 0.0;
		q23 = 0.0;
		q33 = 0.0;
	}

#endif

#ifndef PVMODE
	a_ecef[0] = (float)(kfInfo.X[11]);
	a_ecef[1] = (float)(kfInfo.X[12]);
	a_ecef[2] = (float)(kfInfo.X[13]);

	gnssConvEcef2EnuVel(&a_ecef[0],&a_enu[0],&(p_Kf->kf_Pvt_Info->kfLLApos[0]));
	SYS_LOGGING(OBJ_KF,LOG_INFO,"PVT Q:%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f",
		q11,q12,q13,q22,q23,q33,a_enu[0],a_enu[1],a_enu[2]);
#else
	SYS_LOGGING(OBJ_KF,LOG_INFO,"PVT Q:%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f",
		q11,q12,q13,q22,q23,q33);
#endif
	if (p_Kf->enlargeQ)
	{
		p_Kf->enlargeCnt++;
		//gnss_Kf_Adjust_PMinus(&kfInfo,p_Kf);
		if (p_Kf->enlargeCnt <= ENLARGEQ_SECDS)
		{
			q11 *= p_Kf->QFactor;
			q22 *= p_Kf->QFactor;
			qdf1 *= p_Kf->QFactor;
			qdf4 *= p_Kf->QFactor;
			SYS_LOGGING(OBJ_KF,LOG_INFO,"Enlarge Q:%d,%02d,%10.6f,%10.6f",kfStaticData->staticFlag,p_Kf->enlargeCnt,
				q11,q22);
		}else
		{
			p_Kf->enlargeCnt = 0;
			p_Kf->enlargeQ = FALSE;
		}
	}
	memset(kfInfo.Q, 0, sizeof(double)*N_STATE * N_STATE);

	for (i = 0; i < 3; i++)
	{
		kfInfo.Q[i][i] = q11;
		kfInfo.Q[i + 3][i + 3] = q22;
		kfInfo.Q[i][i + 3] = q12;
		kfInfo.Q[i + 3][i] = q12;
#ifndef PVMODE
		kfInfo.Q[i + 11][i + 11] = q33;
		kfInfo.Q[i][i + 11] = q13;
		kfInfo.Q[i + 11][i] = q13;
		kfInfo.Q[i + 3][i + 11] = q23;
		kfInfo.Q[i + 11][i + 3] = q23;
#endif
	}

	for (i = 0; i < 4; i++)
	{
		kfInfo.Q[i + 6][i + 6] = qdf1;
		kfInfo.Q[10][i + 6] = qdf3;
		kfInfo.Q[i + 6][10] = qdf2;
	}

	//kfInfo.Q[0][0] = kfInfo.Q[1][1] = kfInfo.Q[2][2] = q11;
	//kfInfo.Q[3][3] = kfInfo.Q[4][4] = kfInfo.Q[5][5] = q22;
	//kfInfo.Q[6][6] = kfInfo.Q[7][7] = kfInfo.Q[8][8] = kfInfo.Q[9][9] = qdf1;
	kfInfo.Q[10][10] = qdf4;

	for (i = 0; i < GNSS_MAX_MODE * 2; i++)
	{
		kfInfo.Q[i + 11][i + 11] = qdf5;
	}

	//kfInfo.Q[11][11] = kfInfo.Q[12][12] = kfInfo.Q[13][13] = q33;

	//kfInfo.Q[0][3] = kfInfo.Q[1][4] = kfInfo.Q[2][5] = q12;
	//kfInfo.Q[3][0] = kfInfo.Q[4][1] = kfInfo.Q[5][2] = q12;

	//kfInfo.Q[0][11] = kfInfo.Q[1][12] = kfInfo.Q[2][13] = q13;
	//kfInfo.Q[11][0] = kfInfo.Q[12][1] = kfInfo.Q[13][2] = q13;

	//kfInfo.Q[3][11] = kfInfo.Q[4][12] = kfInfo.Q[5][13] = q23;
	//kfInfo.Q[11][3] = kfInfo.Q[12][4] = kfInfo.Q[13][5] = q23;

	//kfInfo.Q[10][6] = kfInfo.Q[10][7] = kfInfo.Q[10][8] = kfInfo.Q[10][9] = qdf3;
	//kfInfo.Q[6][10] = kfInfo.Q[7][10] = kfInfo.Q[8][10] = kfInfo.Q[9][10] = qdf2;
}


/**********************************************************************
* Function Name:    gnss_Kf_PrNoiseVarCal
*
* Description:
*    function for PR noise variance calculation
*
* Input:
*     measData:     codePhase_std, CN0_1
*
* Return:
*     PR noise variance
*
* Dependency
*      None
*
* Author: 
* Date: 09.06
**********************************************************************/
float gnss_Kf_PrNoiseVarCal(gnss_meas_t* meas, uint32_t avgCno,uint8_t flag)
{
	float          pr_noise = 0;
	uint32_t          cno;
	sat_data_t*  sp;
	Kf_t*        p_Kf;

	p_Kf = gnss_Pe_Get_Kf();

	sp = gnss_sd_get_sv_data(meas->gnssMode,meas->prn,meas->freq_index);

#ifdef USED_IN_MC262M
	if (meas->codeDetStd > 0.0 && g_pe_cfg.chipType == SPRD)
	{
		if (meas->gnssMode == GPS_MODE)
		{
			pr_noise = (float)(0.1 * meas->codeDetStd * 300.0  + 0.15);
		}
		else if (meas->gnssMode == GLN_MODE)
		{
			pr_noise = (float)(0.1 * meas->codeDetStd * 600.0  + 0.15);
		}
		else if (meas->gnssMode == BDS_MODE)
		{
			pr_noise = (float)(0.1 * meas->codeDetStd * 150.0 + 0.15);
		}
		else  if (meas->gnssMode == GAL_MODE)
		{
			//Galileo
		}
		else
		{
			//unexpected
		}

		// de-weight
		if (meas->prChckCnt > 0 && flag == TRUE)
		{
			pr_noise *= (float)pow(PR_WEIGHT_FACTOR,meas->prChckCnt);
			SYS_LOGGING(OBJ_KF,LOG_INFO,"PR De-weight in KF:%d,%03d,%02d",meas->gnssMode,meas->prn,meas->prChckCnt);
		}
		pr_noise = pr_noise * pr_noise * NOISE_SCALE;
	}
	else
#endif
	{
		cno = meas->cno;
		if(g_pe_cfg.automobile)
		{
			if(cno > 50) cno = 50;
			if(avgCno > 30)
			{
				cno -=5;
			}
		}
		else
		{
			if (g_pe_cfg.chipType == SPRD)
			{
				if(cno > 40) cno = 40;
			}else
			{
				if(cno > 45) cno = 45;
			}
		}
#ifdef NOISE_ONE
		pr_noise = (float)(70.56 * exp(-0.016653 * (cno - 10) * 10) - 0.15); //ori
		// de-weight
		if (meas->prChckCnt > 0)
		{
			pr_noise *= (float)pow(PR_WEIGHT_FACTOR,meas->prChckCnt);
			SYS_LOGGING(OBJ_KF,LOG_INFO,"PR De-weight in KF:%d,%03d,%02d",meas->gnssMode,meas->prn,meas->prChckCnt);
		}
		pr_noise = pr_noise * pr_noise * NOISE_SCALE;
#else
		pr_noise = (float)(1.0e5 * pow(10, -0.1 * cno));

		if (meas->cno < 20 && (peMode.userSceData.isUnderEleRoad == FALSE))
		{
			pr_noise *= 2;
		}
		if (meas->prChckCnt > 0 && flag == TRUE)
		{
			pr_noise *= (float)pow(PR_WEIGHT_FACTOR,meas->prChckCnt);
			SYS_LOGGING(OBJ_KF,LOG_INFO,"PR De-weight in KF:%d,%03d,%02d",meas->gnssMode,meas->prn,meas->prChckCnt);
		}
#endif
	}
#if 0
	if (g_pe_cfg.chipType == QCOM && sp->prR >= 50)
	{
		if (pr_noise < sp->prR)
		{
			pr_noise = sp->prR;
		}
	}
	if (g_pe_cfg.chipType == QCOM && pr_noise > 2000.0)
	{
		pr_noise = 2000.0;
	}
#endif
	if (g_pe_cfg.chipType == UBLOX && peMode.userSceData.isUnderEleRoad)
	{
		if ((meas->quality & PR_GOOD) == FALSE && pr_noise < sp->prR && cno >= 45)
		{
			pr_noise = (float)sp->prR;
		}

	}
	
	if ((g_pe_cfg.rtk_cfg.pos_mode == 0) && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100) && (!peMode.userSceData.isUnderEleRoad))
	{
		if ((p_Kf->meas_blk->avgCno > 37) && (p_Kf->meas_blk->measCnt > 40) && p_Kf->meas_blk->isSmallDiff && (p_Kf->posRes < 2) && (p_Kf->ls_Pvt_Info->pos_res < 2))
		{
			pr_noise = (float)(pr_noise * 0.1);
		}
		else
		{
			pr_noise = (float)(pr_noise * 0.5);
		}
	}

	if (g_pe_cfg.chipType == UBLOX && pr_noise > 2000.0 && peMode.userSceData.isUnderEleRoad)
	{
		pr_noise = 2000.0;
	}
	if (g_pe_cfg.chipType == SPRD && pr_noise > 2000.0)
	{
		pr_noise = 2000.0;
	}
	if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
	{

		if ((g_pe_cfg.automobile == 0))
		{
			if (!peMode.staticData.staticFlag)
			{
				pr_noise = (float)(pr_noise * 0.3);
			}
			else
			{
				if (p_Kf->meas_blk->avgCno >= 34)
				{
					pr_noise = (float)(pr_noise * 0.3);
				}
			}
		}


		if (meas->freq_index > 1)
		{
			if (meas->cno > 30)
			{
				pr_noise = (float)(pr_noise * 0.2);
			}
			else if (meas->stdPRDRDiff > 1e-3 && meas->stdPRDRDiff < 5.0 && fabs(meas->sumPRDRDiff) < 10.0)
			{
				pr_noise = (float)(pr_noise * 0.4);
			}
			else
			{
				pr_noise = (float)(pr_noise * 0.8);
			}
		}
		else
		{
			if (meas->stdPRDRDiff > 1e-3 && meas->stdPRDRDiff < 5.0 && fabs(meas->sumPRDRDiff) < 10.0)
			{
				pr_noise = (float)(pr_noise * 0.5);
			}
		}
	}
	
	return pr_noise;
}

/**********************************************************************
* Function Name:    gnss_Kf_DrNoiseVarCal
*
* Description:
*    function for DR noise variance calculation
*
* Input:
*     measData:     carrPhase_std, CN0_1
*
* Return:
*     DR noise variance
*
* Dependency
*      None
*
* Author: 
* Date: 09.06
**********************************************************************/
float gnss_Kf_DrNoiseVarCal(gnss_meas_t* meas, uint32_t avgCno)
{
	float    pr_Noise;
	//float    dr_noise;
	float    drVar;
	uint32_t    cno;
#ifdef USED_IN_MC262M
	if (meas->dopplerStd > 0 && g_pe_cfg.chipType == SPRD)
	{
		dr_noise = (float)(meas->dopplerStd);
		drVar = dr_noise * dr_noise;
	}
	else
#endif
	{
		cno = meas->cno;
		if(g_pe_cfg.automobile)
		{
			if(cno > 50) cno = 50;
			if(avgCno > 30)
			{
				cno -=5;
			}
		}
		else
		{
			if(cno > 45) cno = 45;
		}

#ifndef NOISE_ONE
		pr_Noise = (float)(70.56 * exp(-0.016653 * (cno - 10) * 10) - 0.15);
		if (meas->drChckCnt > 0)
		{
			pr_Noise *= (float)pow(DR_WEIGHT_FACTOR,meas->drChckCnt);
		}
		if (peState.posHasBias)
		{
			drVar = (float)0.001 * pr_Noise * pr_Noise * NOISE_SCALE;
		}else if (peMode.userSceData.isPDRMode)
		{
			drVar = (float)0.002 * pr_Noise * pr_Noise * NOISE_SCALE;
		}
		else
		{
			drVar = (float)0.001 * pr_Noise * pr_Noise * NOISE_SCALE;
		}
#else
		pr_Noise = (float)1.0e5 * pow(10,-0.1* cno);
		if (meas->drChckCnt > 0)
		{
			pr_Noise *= (float)pow(DR_WEIGHT_FACTOR,meas->drChckCnt);
		}
		drVar = (float)0.001 * pr_Noise;
#endif
	}

	if (g_pe_cfg.chipType == SPRD)
	{
		drVar += (float)0.20;
	}else
	{
		if (peState.posHasBias)
		{
			if (meas->quality & PR_GOOD)
			{
				drVar += (float)0.20;
			}else
			{
				drVar += (float)0.50;
			}
		}else if (peMode.userSceData.isPDRMode)
		{
			if (meas->quality & PR_GOOD)
			{
				drVar += (float)0.10;
			}else
			{
				drVar += (float)0.20;
			}
		}
		else
		{
			drVar += (float)0.10;
		}
	}

	return drVar;
}

/***********************************************************************
* 函数介绍: gnss_Kf_LastLsCopy
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/12/
***********************************************************************/
void gnss_Kf_LastLsCopy(LS_PVT_INFO* lsPvtInfo)
{
	uint8_t      i;
	double     dt = 0;

	lsPvtInfo->last_valid = lsPvtInfo->valid;

	if (fabs(lsPvtInfo->lastTor) > 1e-6)
	{
		dt = lsPvtInfo->tor - lsPvtInfo->lastTor;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
	}

	if (lsPvtInfo->valid)
	{
		if ((dt >= 0.400 && !IS_PPK_REVERSE) || (dt <= -0.400 && IS_PPK_REVERSE) || (fabs(lsPvtInfo->lastTor) < 1e-6))
		{
			lsPvtInfo->lastTor = lsPvtInfo->tor;

			for (i = 0; i < 3; i++)
			{
				lsPvtInfo->ecefPos_last[i] = lsPvtInfo->ecefPos[i];
				lsPvtInfo->ecefVel_last[i] = lsPvtInfo->ecefVel[i];
			}
			for (i = 0; i < BIAS_NUM; i++)
			{
				lsPvtInfo->clkBias_last[i] = lsPvtInfo->clkBias[i];
				lsPvtInfo->biasValid_last[i] = lsPvtInfo->biasValid[i];
			}
		}
	}
	else
	{
		lsPvtInfo->lastTor = lsPvtInfo->tor;
		return;
	}
}

void  gnss_Kf_Check_BackupPos(Kf_t *p_Kf)
{
	uint8_t                   i;
	int32_t                  delta_week;
	float                  vel_ls = 0,vel_backup = 0;
	double                  dt,distance,distance_h/*,delta_alt*/;
	double                  deltaTimeThres;
	LS_PVT_INFO*         lsPvtInfo;
	GNSS_TIME*           pTime;
	lsPvtInfo = p_Kf->ls_Pvt_Info;

	pTime = gnss_tm_get_time();
	if(pTime->init==FALSE)
	{
		return;
	}
	if(user_pvt_backup.have_position == HAVE_POS_INIT)
	{
		pTime = gnss_tm_get_time();
		dt = pTime->rcvr_time[GPS_MODE]- user_pvt_backup.posfix_t;
		delta_week = (int32_t)(pTime->week[0] - user_pvt_backup.posfix_wn);
		if(delta_week>0)
		{
			dt += (double)SECS_IN_WEEK*delta_week;
		}
		if(dt > 300)
		{
			return;
		}

		if(HisInfo.kfFixStatus !=FIX_SOURCE_3D||HisInfo.kfRunEpochCnt<10)
		{
			return;
		}

		vel_backup  = (float)sqrt( (double)user_pvt_backup.lla.vel[0]*user_pvt_backup.lla.vel[0] + (double)user_pvt_backup.lla.vel[1]*user_pvt_backup.lla.vel[1] );
		vel_ls   = (float)sqrt( (double)lsPvtInfo->enuVel[0]*lsPvtInfo->enuVel[0]+ (double)lsPvtInfo->enuVel[1]*lsPvtInfo->enuVel[1]);
		distance = ((double)vel_ls+vel_backup)/2.0*dt;
		distance_h = gnssCalPosDis(user_pvt_backup.lla.pos,lsPvtInfo->LLApos,TRUE);
		//delta_alt = lsPvtInfo->LLApos[2]-user_pvt_backup.lla.pos[2];

		if(g_pe_cfg.automobile)
		{
			deltaTimeThres = KF_USE_HISTORY_TIME_AUTO;
		}
		else
		{
			deltaTimeThres = KF_USE_HISTORY_TIME;
		}

		if(distance < 50 && distance_h < 150 && dt < deltaTimeThres)
		{
			for(i = 0;i<3;i++)
			{
				user_pvt_backup.ecef.pos[i] += ((double)user_pvt_backup.ecef.vel[i]+lsPvtInfo->ecefVel[i])/2.0*dt;
				lsPvtInfo->ecefPos[i] = user_pvt_backup.ecef.pos[i];
			}
			if((distance < 8.0 && distance_h < 35)|| (distance < 2.5 && distance_h < 60 && dt < 15))
			{
				p_Kf->kfInitPosType = KF_INIT_POS_ACC;
			}
			else
			{
				p_Kf->kfInitPosType = KF_INIT_POS_APPX;
			}
			SYS_LOGGING(OBJ_KF,LOG_INFO,"KF use last good position to start");
		}
	}
}
/***********************************************************************
* 函数介绍: gnss_Kf_CheckLs
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/12/
***********************************************************************/
uint8_t gnss_Kf_CheckLs(Kf_t *p_Kf)
{
	uint8_t                   i;
	float                  dt;
	float                  threshold = 0,/*thres_alt,*/ deltaR;
	float                  velocity = 0, velocity_last = 0, vel_heading = 0, ls_vel[3] = { 0.0 }, ls_enuVel[3] = {0.0},ls_drift = 0.0;
	float                  distance = 0,disDiff = 0.0,distance_h = 0,disDiff_h = 0;
	//float                  delta_alt = 0.0;
	float                  deltaBias[3]={0.0};
	float                  biasChkThres = 50.0;
	double                  lsPoslla_last[3] = {0.0};
	double                  biasDiffUse[GNSS_MAX_MODE] = {0.0};
	LS_PVT_INFO*         lsPvtInfo;
	Ls_t*                p_Ls;
	meas_blk_t*          pMeas;
	double                  ls_pos_res_thres = 100.0;

	uint8_t                   lsPassCheckCnt_thres = 2,prnum_thres = 8, Cno35Cnt = 0;
	double                  opensky_factor = 0.0,ls_init_kf_thres = 10.0;

	if (p_Kf == NULL)
	{
		return FALSE;
	}

	lsPvtInfo = p_Kf->ls_Pvt_Info;
	p_Ls = p_Kf->lsBlk;

	if (lsPvtInfo == NULL || p_Ls == NULL)
	{
		return FALSE;
	}

	pMeas = p_Ls->meas;

	if (pMeas == NULL)
	{
		return FALSE;
	}

	/* If no drift is unknown, not start KF */
	if (lsPvtInfo->isDriftValid == FALSE && 
		(g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == MEAS_TYPE_MASK_DR)
	{
		return FALSE;
	}
	for (i = 0; i < GNSS_MAX_MODE; i++)
	{
		Cno35Cnt += pMeas->Cno35Cnt[i];
	}

	if ((g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755) && (!firstFix))
	{
		ls_pos_res_thres = 10.0;
		lsPassCheckCnt_thres = 5;
		prnum_thres = 12;
	}

	if(lsPvtInfo->valid)
	{
		p_Kf->lsCheckCnt++;
		//load  bias diff for use
		for (i = 0; i < GNSS_MAX_MODE; i++)
		{
			if(pMeas->rtdUsed && p_Kf->biasDiffUpCnt[i]>=BIASDIFF_UPCNT_THRES)
			{
				biasDiffUse[i] = p_Kf->biasDiff[i];
			}
			else if(!pMeas->rtdUsed && p_Kf->biasDiffLocalUpCnt[i]>=BIASDIFF_UPCNT_THRES)
			{
				biasDiffUse[i] = p_Kf->biasDiffLocal[i];
			}
			else
			{
				biasDiffUse[i] = 0.0;
			}
		}

		//LS bias check with stored bias diff
		if(lsPvtInfo->biasValid[GPS_MODE]&&lsPvtInfo->biasValid[GLN_MODE]&&fabs(biasDiffUse[GLN_MODE])>0)
		{
			deltaBias[0] = (float)fabs((double)lsPvtInfo->clkBias[GPS_MODE]-lsPvtInfo->clkBias[GLN_MODE]-biasDiffUse[GLN_MODE]);
		}
		if(lsPvtInfo->biasValid[GPS_MODE]&&lsPvtInfo->biasValid[BDS_MODE]&&fabs(biasDiffUse[BDS_MODE])>0)
		{
			deltaBias[1] = (float)fabs((double)lsPvtInfo->clkBias[GPS_MODE]-lsPvtInfo->clkBias[BDS_MODE]-biasDiffUse[BDS_MODE]);
		}
		if (lsPvtInfo->biasValid[GPS_MODE] && lsPvtInfo->biasValid[GAL_MODE] && fabs(biasDiffUse[GAL_MODE]) > 0)
		{
			deltaBias[2] = (float)fabs((double)lsPvtInfo->clkBias[GPS_MODE] - lsPvtInfo->clkBias[GAL_MODE] - biasDiffUse[GAL_MODE]);
		}

		if (g_pe_cfg.automobile == 0 && g_pe_cfg.chipType == QCOM)
		{
			if(deltaBias[0]>biasChkThres||deltaBias[1]>biasChkThres || deltaBias[2] > biasChkThres)
			{
				SYS_LOGGING(OBJ_KF,LOG_INFO,"LS couldn't pass bias check to start KF");
				p_Kf->lsPassCheckCnt = 0;
				return FALSE;
			}
		}

		//LS position residual check
		if (lsPvtInfo->pos_res > ls_pos_res_thres)
		{
			SYS_LOGGING(OBJ_KF,LOG_INFO,"LS couldn't pass position residual check to start KF");
			p_Kf->lsPassCheckCnt = 0;
			return FALSE;
		}

		/*
		LS velocity validation check
		1. If LS velocity > 15m/s, velocity residual should be smaller than 2.0
		*/
		velocity = (float)sqrt((double)lsPvtInfo->enuVel[0] * lsPvtInfo->enuVel[0] + (double)lsPvtInfo->enuVel[1] * lsPvtInfo->enuVel[1]);
		if (lsPvtInfo->vel_res >= 2.0 && velocity >= 15)
		{
			SYS_LOGGING(OBJ_KF,LOG_INFO,"LS couldn't pass velocity residual check to start KF");
			p_Kf->lsPassCheckCnt = 0;
			return FALSE;
		}
	}
	/*
	1. At least two valid LS results
	2. The time difference between two good results should be within threshold
	*/
	if (lsPvtInfo->last_valid  && lsPvtInfo->valid)
	{
		dt = (float)(lsPvtInfo->tor - lsPvtInfo->lastTor);
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= SECS_IN_WEEK;
		}
		if (((dt > 1.5 || dt < 0.05) && !IS_PPK_REVERSE)||((dt < -1.5 || dt > -0.05) && IS_PPK_REVERSE))
		{
			p_Kf->lsCheckCnt = 0;
			p_Kf->lsPassCheckCnt = 0;
			SYS_LOGGING(OBJ_KF,LOG_INFO,"Abnormal KF update time gap (%8.6f)",dt);
			return FALSE;
		}
		velocity = 0.0;
		velocity_last = 0.0;

		if ((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == FALSE)
		{
			for (i = 0; i < 3; i++)
			{
				ls_vel[i] = (float)((lsPvtInfo->ecefPos[i] - lsPvtInfo->ecefPos_last[i])/dt);
			}
			for (i = 0; i < 3; i++)
			{
				velocity += (ls_vel[i] * ls_vel[i]);
			}
			velocity = (float)sqrt(velocity);

			gnssConvEcef2EnuVel(ls_vel,ls_enuVel, lsPvtInfo->LLApos);

			for (i = 0; i < 2; i++)
			{
				vel_heading += (ls_enuVel[i] * ls_enuVel[i]);
			}
			vel_heading = (float)(sqrt(vel_heading));

			if (!g_pe_cfg.ppk_mode)
			{
				if (velocity > MAX_VELOCITY || (vel_heading < fabs(ls_enuVel[2]) && fabs(ls_enuVel[2]) > 10.0))  //100m/s is not suitable for aircraft and high speed train, to be refine
				{
					p_Kf->lsPassCheckCnt = 0;
					return FALSE;
				}
			}
			else
			{
				if (velocity > 500 || (fabs(ls_enuVel[2]) > 500.0))  //100m/s is not suitable for aircraft and high speed train, to be refine
				{
					p_Kf->lsPassCheckCnt = 0;
					return FALSE;
				}
			}

			for (i = 0; i < BIAS_NUM; i++)
			{   
				if (lsPvtInfo->biasValid[i] && lsPvtInfo->biasValid_last[i])
				{
					ls_drift = (lsPvtInfo->clkBias[i] - lsPvtInfo->clkBias_last[i])/dt;
					break;
				}
			}

			if (fabs(ls_drift) < 1e-6)
			{
				p_Kf->lsPassCheckCnt = 0;
				return FALSE;
			}

			// PR RAIM and DR RAIM didn't find any outliers
			if ((p_Ls->lsCtrl.prNum - p_Ls->lsCtrl.lsBiasNum) >= 6 &&
				p_Ls->pos_res < 30.0)
			{
				p_Kf->lsPassCheckCnt++;
				p_Kf->kfInitPosType = KF_INIT_POS_LS;
				memcpy(lsPvtInfo->ecefVel, ls_vel,sizeof(lsPvtInfo->ecefVel));
				lsPvtInfo->clkDrift = ls_drift;
				lsPvtInfo->isDriftValid = TRUE;

			}
			else
			{
				p_Kf->lsPassCheckCnt = 0;
			}
		}
		else
		{

			for (i = 0; i < 3; i++)
			{
				velocity += (lsPvtInfo->ecefVel[i] * lsPvtInfo->ecefVel[i]);
				velocity_last += (lsPvtInfo->ecefVel_last[i] * lsPvtInfo->ecefVel_last[i]);
			}

			velocity = (float)(sqrt(velocity));
			velocity_last = (float)(sqrt(velocity_last));

			for (i = 0; i < 2; i++)
			{
				vel_heading += (lsPvtInfo->enuVel[i] * lsPvtInfo->enuVel[i]);
			}
			vel_heading = (float)(sqrt(vel_heading));


			for (i = 0; i < 3; i++)
			{
				deltaR = (float)(lsPvtInfo->ecefPos[i] - lsPvtInfo->ecefPos_last[i]);
				distance += deltaR * deltaR;
			}
			distance = (float)(sqrt(distance));

			gnssConvEcef2Lla(lsPvtInfo->ecefPos_last, lsPoslla_last);
			distance_h = gnssCalPosDis(lsPvtInfo->LLApos, lsPoslla_last, TRUE);

			if (!g_pe_cfg.ppk_mode)
			{
				if (velocity > MAX_VELOCITY || (vel_heading < fabs(lsPvtInfo->enuVel[2]) && fabs(lsPvtInfo->enuVel[2]) > 10.0))  //100m/s is not suitable for aircraft and high speed train, to be refine
				{
					p_Kf->lsPassCheckCnt = 0;
					return FALSE;
				}
			}
			else
			{
				if (velocity > 500 || (fabs(lsPvtInfo->enuVel[2]) > 500.0))  //100m/s is not suitable for aircraft and high speed train, to be refine
				{
					p_Kf->lsPassCheckCnt = 0;
					return FALSE;
				}
			}

			//calculate the distance diff between two LS position 
			disDiff = (float)fabs(distance - ((double)velocity + velocity_last) / 2.0*fabs(dt));
			disDiff_h = (float)fabs(distance_h - vel_heading * fabs(dt));
#if 0
			delta_alt = (float)fabs(lsPvtInfo->lla[2] - lsPoslla_last[2] - (double)lsPvtInfo->enuVel[2] * dt);
			thres_alt = 30;
#endif

			if (velocity * fabs(dt) > 50)
			{
				threshold = (float)(velocity * fabs(dt));
			}
			else
			{
				threshold = 50;
			}

			//low cn0 mode, such as indoor environment
			if (pMeas->avgCno < 18)
			{
				threshold *= 1.5;
			}

			if (disDiff > threshold)
			{
				p_Kf->lsPassCheckCnt = 0;
				SYS_LOGGING(OBJ_KF, LOG_INFO, "LS couldn't pass check to start KF as jump : %f", disDiff);
				return FALSE;
			}


			//check LS altitude change
#if 0
			if (delta_alt > thres_alt)
			{
				p_Kf->lsPassCheckCnt = 0;
				return FALSE;
			}

#endif
			p_Kf->lsPassCheckCnt++;
			p_Kf->kfInitPosType = KF_INIT_POS_LS;

			//Check history position before last stop
			gnss_Kf_Check_BackupPos(p_Kf);
		}

		/* LS position pass check logic:
		(1) LS pos pass continuous two times' check (three epoch'position);
		(2) LS pass one check, and the distance between two epoch's LS position is small and LS residual is small;
		(3) LS pass one check, and LS check cnt >=15.
		*/
		if(p_Kf->lsPassCheckCnt>=lsPassCheckCnt_thres)
		{
			return TRUE;
		}
		else if(disDiff < threshold/2.0 && p_Ls->pos_res<35.0 && p_Ls->lsCtrl.prNum >= (5 + p_Ls->lsCtrl.lsBiasNum))
		{
			return TRUE;
		}
		else if(disDiff_h < threshold/3.0 && p_Ls->pos_res<15.0 && p_Ls->lsCtrl.prNum >= (5 + p_Ls->lsCtrl.lsBiasNum))
		{
			return TRUE;
		}
		else if(p_Kf->kfInitPosType > KF_INIT_POS_LS )
		{
			return TRUE;
		}
		else if(p_Kf->lsCheckCnt>=15)
		{
			return TRUE;
		}
		else 
		{
			return FALSE;
		}
	}
	else
	{
#if 1   
		/* If no doppler meas, not start KF at the first epoch of LS fix*/
		if ((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == FALSE)
		{
			return FALSE;
		}
		/* 1. If tunnel mode, then strict KF condition */
		if (peMode.tunnelMode.exitTunnelFlag)
		{
			return FALSE;
		}

		/* 2. average CNO check */
		if (pMeas->avgCno < 35)
		{
			return FALSE;
		}

		/* 3. Only one LS result */
		if (lsPvtInfo->valid)
		{
			if (p_Ls->lsCtrl.isPrRaim == FALSE && p_Ls->lsCtrl.isDrRaim == FALSE)
			{
				if (Cno35Cnt > 16 && pMeas->Cno45Cnt > 0)
				{
					opensky_factor = pMeas->Cno45Cnt * 1.0 / Cno35Cnt;
					if (opensky_factor < 0.2)
					{
						ls_init_kf_thres = 4.0;
					}
					else if (opensky_factor < 0.4)
					{
						ls_init_kf_thres = 6.0;
					}
					else if (opensky_factor < 0.6)
					{
						ls_init_kf_thres = 8.0;
					}
					SYS_LOGGING(OBJ_KF, LOG_INFO, "Opensky_Factor %d %d %10.4f %10.4f", Cno35Cnt, pMeas->Cno45Cnt, opensky_factor, ls_init_kf_thres);
				}
				// PR RAIM and DR RAIM didn't find any outliers
				if ((p_Ls->lsCtrl.prNum - p_Ls->lsCtrl.lsBiasNum) >= prnum_thres&&
					p_Ls->pos_res < ls_init_kf_thres)
				{
					p_Kf->kfInitPosType = KF_INIT_POS_LS;
					return TRUE;
				}else
				{
					return FALSE;
				}
			}
			else 
			{
				return FALSE;
			}
		}
#endif 
		return FALSE;
	}
}

/***********************************************************************
* 函数介绍: gnss_Kf_Start
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
void gnss_Kf_Start(double T, Kf_t* p_Kf, uint8_t flag)
{
	uint32_t            i;
	double            dt;
	double            biasTmp;
	KF_PVT_INFO*   kfPvtInfo;
	LS_PVT_INFO*   lsPvtInfo;
	meas_blk_t*    pMeas;
	GNSS_TIME*        pTime;

	pTime = gnss_tm_get_time();

	kfPvtInfo = p_Kf->kf_Pvt_Info;
	lsPvtInfo = p_Kf->ls_Pvt_Info;
	pMeas = p_Kf->meas_blk;

	if (flag == KF_INIT || flag == KF_RESET)
	{
		kfPvtInfo->tor = lsPvtInfo->tor;
		kfPvtInfo->lastTor = lsPvtInfo->tor;
		kfInfo.tor = lsPvtInfo->tor;
	}

	kfInfo.periodTms = (int16_t)(T * 1000);

	/* initialize Q matrix */
	gnss_Kf_QmatInit(p_Kf,T,NULL);

	/* initialize A matrix */
	gnss_Kf_AmatInit(T);

	/* initialize KF state X */
	if ((flag == KF_INIT) || (flag == KF_RESET))
	{
		/* initialize U and D */
		memset(kfInfo.U_plus, 0, sizeof(double)*N_MAT);
		uMatInit(kfInfo.U_plus, N_STATE);
		memset(kfInfo.U_minus, 0, sizeof(double)*N_MAT);
		uMatInit(kfInfo.U_minus, N_STATE);

		if(p_Kf->kfInitPosType == KF_INIT_POS_ACC)
		{
			kfInfo.D_plus[1] = kfInfo.D_plus[2] = kfInfo.D_plus[3] = 4;
			kfInfo.D_plus[4] = kfInfo.D_plus[5] = kfInfo.D_plus[6] = 4;
			kfInfo.D_plus[7] = kfInfo.D_plus[8] = kfInfo.D_plus[9] = kfInfo.D_plus[10] =100;
			kfInfo.D_plus[11] = 1;
			kfInfo.D_plus[12] = kfInfo.D_plus[13] = kfInfo.D_plus[14] = kfInfo.D_plus[15] = 10;
			kfInfo.D_plus[16] = kfInfo.D_plus[17] = kfInfo.D_plus[18] = kfInfo.D_plus[19] = 10;
		}
		else if(p_Kf->kfInitPosType == KF_INIT_POS_APPX)
		{
			kfInfo.D_plus[1] = kfInfo.D_plus[2] = kfInfo.D_plus[3] = 25;
			kfInfo.D_plus[4] = kfInfo.D_plus[5] = kfInfo.D_plus[6] = 4;
			kfInfo.D_plus[7] = kfInfo.D_plus[8] = kfInfo.D_plus[9] = kfInfo.D_plus[10] =100;
			kfInfo.D_plus[11] = 1;
			kfInfo.D_plus[12] = kfInfo.D_plus[13] = kfInfo.D_plus[14] = kfInfo.D_plus[15] = 10;
			kfInfo.D_plus[16] = kfInfo.D_plus[17] = kfInfo.D_plus[18] = kfInfo.D_plus[19] = 10;
		}
		else
		{
			kfInfo.D_plus[1] = kfInfo.D_plus[2] = kfInfo.D_plus[3] = 625;
			kfInfo.D_plus[4] = kfInfo.D_plus[5] = kfInfo.D_plus[6] = 4;
			kfInfo.D_plus[7] = kfInfo.D_plus[8] = kfInfo.D_plus[9] = kfInfo.D_plus[10] = 100;
			kfInfo.D_plus[11] = 1;
			kfInfo.D_plus[12] = kfInfo.D_plus[13] = kfInfo.D_plus[14] = kfInfo.D_plus[15] = 10;
			kfInfo.D_plus[16] = kfInfo.D_plus[17] = kfInfo.D_plus[18] = kfInfo.D_plus[19] = 10;
		}
#ifndef PVMODE
		kfInfo.D_plus[12] = kfInfo.D_plus[13] = kfInfo.D_plus[14] = 1; 
#endif
		memcpy(kfInfo.D_minus, kfInfo.D_plus, sizeof(kfInfo.D_minus));

		for (i = 0; i < 3; i++)
		{
			kfInfo.X[i] = lsPvtInfo->ecefPos[i];       //x, y, z
			kfInfo.X[i + 3] = lsPvtInfo->ecefVel[i];   //vx, vy, vz
#ifndef PVMODE
			kfInfo.X[i + 11] = 0;                      //ax, ay, az
#endif
		}

		/* 
		1. biasLinkFlag = 0, use LS bias to init KF
		2. biasLinkFlag != 0, use biasDiff and LS bias to init KF 
		*/
		if (p_Kf->kf_ctrl.biasLinkFlag == 0)
		{
			if (lsPvtInfo->isBiasLink == FALSE)
			{
				for (i = 0; i < BIAS_NUM; i++)
				{
					kfInfo.X[i + 6] = lsPvtInfo->clkBias[i];   //bias[4]
				}
			}else
			{
				kfInfo.X[6] = lsPvtInfo->clkBias[0];
				if (pMeas->rtdUsed)
				{
					kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
					kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
					kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
				}else
				{
					kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
					kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
					kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
				}
			}
		}else
		{
			if (lsPvtInfo->biasValid[GPS_MODE] == FALSE)
			{
				if (lsPvtInfo->biasValid[GLN_MODE] == TRUE)
				{
					if (pMeas->rtdUsed)
					{            
						kfInfo.X[6] = lsPvtInfo->clkBias[GLN_MODE] + p_Kf->biasDiff[GLN_MODE];
						kfInfo.X[7] = lsPvtInfo->clkBias[GLN_MODE];
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
					}else
					{
						kfInfo.X[6] = lsPvtInfo->clkBias[GLN_MODE] + p_Kf->biasDiffLocal[GLN_MODE];
						kfInfo.X[7] = lsPvtInfo->clkBias[GLN_MODE];
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
					}
				}
				else if (lsPvtInfo->biasValid[BDS_MODE] == TRUE)
				{ 
					if (pMeas->rtdUsed)
					{            
						kfInfo.X[6] = lsPvtInfo->clkBias[BDS_MODE] + p_Kf->biasDiff[BDS_MODE];
						kfInfo.X[8] = lsPvtInfo->clkBias[BDS_MODE];
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
					}else
					{
						kfInfo.X[6] = lsPvtInfo->clkBias[BDS_MODE] + p_Kf->biasDiffLocal[BDS_MODE];
						kfInfo.X[8] = lsPvtInfo->clkBias[BDS_MODE];
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
					}
				}
				else if (lsPvtInfo->biasValid[GAL_MODE] == TRUE)
				{
					if (pMeas->rtdUsed)
					{
						kfInfo.X[6] = lsPvtInfo->clkBias[GAL_MODE] + p_Kf->biasDiff[GAL_MODE];
						kfInfo.X[9] = lsPvtInfo->clkBias[GAL_MODE];
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
					}
					else
					{
						kfInfo.X[6] = lsPvtInfo->clkBias[GAL_MODE] + p_Kf->biasDiffLocal[GAL_MODE];
						kfInfo.X[9] = lsPvtInfo->clkBias[GAL_MODE];
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
					}
				}
				else
				{
					SYS_LOGGING(OBJ_KF, LOG_INFO,"%s, NO BIAS START KF",__FUNCTION__);
				}
			}else
			{
				kfInfo.X[6] = lsPvtInfo->clkBias[GPS_MODE];
				if (pMeas->rtdUsed)
				{
					kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
					kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
					kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
				}else
				{
					kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
					kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
					kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
				}
			}
		}

		//kfInfo.X[9] = lsPvtInfo->clkBias[GAL_MODE];
		kfInfo.X[10] = lsPvtInfo->clkDrift;            //drift

		for (i = 0; i < GNSS_MAX_MODE * 2; i++)
		{
			if (lsPvtInfo->dcbValid[i] > 0)
			{
				kfInfo.X[11 + i] = lsPvtInfo->clkDcb[i];
			}
			else
			{
				kfInfo.X[11 + i] = pTime->dcb[i];
			}
		}
		p_Kf->kf_Static_Data.kfCnt = 0;
		p_Kf->kfCnt = 0;
		p_Kf->kfZuptCnt = 0;
		kfPvtInfo->clkDriftRef = 0.0;
	}
	else if (flag == KF_RESTART)
	{
		//kfInfo.status &= (~KF_RESTART);
		/*kfInfo.status |= KF_RUN;*/
		dt = kfInfo.tor - kfPvtInfo->lastTor;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}

		for (i = 0; i < 3; i++)
		{
			kfInfo.X[i] = kfPvtInfo->ecefPos_last[i] + dt * kfPvtInfo->ecefVel_last[i];       //x, y, z
			kfInfo.X[i + 3] = kfPvtInfo->ecefVel_last[i];   //vx, vy, vz
#ifndef PVMODE
			kfInfo.X[i + 11] = 0;                           //ax, ay, az
#endif
		}
		if (p_Kf->kf_ctrl.biasLinkFlag == 0)
		{
			for (i = 0; i < BIAS_NUM; i++)
			{
				if(kfPvtInfo->biasValid[i])
				{
					kfInfo.X[i + 6] = kfPvtInfo->clkBias_last[i] + dt * kfPvtInfo->clkDrift_last;   //bias[4]
				}
				else
				{
					if(lsPvtInfo->biasValid[i])
					{
						kfInfo.X[i + 6] = lsPvtInfo->clkBias[i];
					}
				}
			}
		}else
		{
			if (kfPvtInfo->biasValid[GPS_MODE] == FALSE)
			{
				if (kfPvtInfo->biasValid[GLN_MODE] == TRUE)
				{
					biasTmp = kfPvtInfo->clkBias_last[GLN_MODE] + dt * kfPvtInfo->clkDrift_last;
					if (pMeas->rtdUsed)
					{
						kfInfo.X[6] = biasTmp + p_Kf->biasDiff[GLN_MODE];
						kfInfo.X[7] = biasTmp;
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
					}else
					{
						kfInfo.X[6] = biasTmp + p_Kf->biasDiffLocal[GLN_MODE];
						kfInfo.X[7] = biasTmp;
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
					}
				}
				else if (kfPvtInfo->biasValid[BDS_MODE] == TRUE)
				{ 
					biasTmp = kfPvtInfo->clkBias_last[BDS_MODE] + dt * kfPvtInfo->clkDrift_last;
					if (pMeas->rtdUsed)
					{
						kfInfo.X[6] = biasTmp + p_Kf->biasDiff[BDS_MODE];
						kfInfo.X[8] = biasTmp;
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
					}else
					{
						kfInfo.X[6] = biasTmp + p_Kf->biasDiffLocal[BDS_MODE];
						kfInfo.X[8] = biasTmp;
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
						kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
					}
				}
				else if (kfPvtInfo->biasValid[GAL_MODE] == TRUE)
				{
					biasTmp = kfPvtInfo->clkBias_last[GAL_MODE] + dt * kfPvtInfo->clkDrift_last;
					if (pMeas->rtdUsed)
					{
						kfInfo.X[6] = biasTmp + p_Kf->biasDiff[GAL_MODE];
						kfInfo.X[9] = biasTmp;
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
					}
					else
					{
						kfInfo.X[6] = biasTmp + p_Kf->biasDiffLocal[GAL_MODE];
						kfInfo.X[9] = biasTmp;
						kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
						kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
					}
				}
				else
				{
					SYS_LOGGING(OBJ_KF, LOG_INFO,"%s, NO BIAS START KF",__FUNCTION__);
				}
			}else
			{
				kfInfo.X[6] = kfPvtInfo->clkBias_last[GPS_MODE] + dt * kfPvtInfo->clkDrift_last;
				if (pMeas->rtdUsed)
				{
					kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
					kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
					kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
				}else
				{
					kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
					kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
					kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
				}
			}

			//if(kfPvtInfo->biasValid[GAL_MODE])
			//{
			//	kfInfo.X[9] = kfPvtInfo->clkBias_last[GAL_MODE] + dt * kfPvtInfo->clkDrift_last;   //bias[4]
			//}
			//else
			//{
			//	if(lsPvtInfo->biasValid[GAL_MODE])
			//	{
			//		kfInfo.X[9] = lsPvtInfo->clkBias[GAL_MODE];
			//	}
			//}
		}

		kfInfo.X[10] = kfPvtInfo->clkDrift_last;            //drift

		for (i = 0; i < GNSS_MAX_MODE * 2; i++)
		{
			if (lsPvtInfo->dcbValid[i] > 0)
			{
				kfInfo.X[11 + i] = lsPvtInfo->clkDcb[i];
			}
			else
			{
				kfInfo.X[11 + i] = pTime->dcb[i];
			}
		}

		kfPvtInfo->clkDriftRef = kfPvtInfo->clkDrift_last;
		p_Kf->kf_Static_Data.kfCnt = 0;
		p_Kf->kfCnt = 0;
		p_Kf->kfZuptCnt = 0;
	}
	else
	{
		//error
	}

	/*
	KF is in INIT or RESET mode, then we need set KF to RUN mode and 
	*/
	if ((kfInfo.status & KF_INIT) || (kfInfo.status & KF_RESET) || (kfInfo.status & KF_RESTART))
	{
		if (kfInfo.status & KF_INIT)
		{
			kfInfo.status &= (~KF_INIT);
		}
		else if (kfInfo.status & KF_RESET)
		{
			kfInfo.status &= (~KF_RESET);
		}else 
		{
			kfInfo.status &= (~KF_RESTART);
		}

		kfInfo.status |= KF_RUN;
	}

	/* initialize KF fix mode */
	if (flag == KF_INIT || flag == KF_RESET || flag == KF_RESTART)
	{
		kfPvtInfo->kfFixMode = FULL_FIX;
	}
}

/**********************************************************************
* Function Name:    gnss_Kf_SetClkBias
*
* Description:
*    set the clock bias of KF according to satMode and biasLinkFlag
*
* Input:
*     bias:        clock bias
*     satMode:     satellite mode
*     biasLinkFlag: clock bias flag
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_SetClkBias(double bias, uint8_t satMode, double dt)
{
	kfInfo.X[satMode+6] = bias - kfInfo.X[10] * dt;
}
/**********************************************************************
* Function Name:    gnssKfBiasCorrection
*
* Description:
*    KF bias correction when local time correction
*
* Input:
*
*
* Return:
*
*
* Dependency
*      None
*
* Author: jpdeng
**********************************************************************/
void gnss_Kf_BiasCorrection(int8_t msCorr, uint32_t biasIdx)
{
	kfInfo.X[6 + biasIdx] -= (float)(msCorr * LIGHT_MSEC);
	SYS_LOGGING(OBJ_KF,LOG_INFO,"bias 1ms correction:%d, satMode:%d", msCorr, biasIdx);
}

/***********************************************************************
* 函数介绍: gnss_Kf_ReInit_Bias
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/04/
***********************************************************************/
void gnss_Kf_ReInit_Bias(uint8_t gnssMode,Kf_t* p_Kf,double dt,uint16_t* flag)
{
	uint8_t             i;
	double            biasDiffUse[GNSS_MAX_MODE] = {0.0};
	meas_blk_t*    pMeas;
	//LS_PVT_INFO*   lsPvtInfo;
	Ls_t*          p_Ls;

	//lsPvtInfo = p_Kf->ls_Pvt_Info;
	p_Ls = p_Kf->lsBlk;
	pMeas = p_Kf->meas_blk;

	//load  bias diff for use
	for (i = 0; i < GNSS_MAX_MODE; i++)
	{
		if(pMeas->rtdUsed && p_Kf->biasDiffUpCnt[i]>=BIASDIFF_UPCNT_THRES)
		{
			biasDiffUse[i] = p_Kf->biasDiff[i];
		}
		else if(!pMeas->rtdUsed && p_Kf->biasDiffLocalUpCnt[i]>=BIASDIFF_UPCNT_THRES)
		{
			biasDiffUse[i] = p_Kf->biasDiffLocal[i];
		}
		else
		{
			biasDiffUse[i] = 0.0;
		}
	}

	if (gnssMode == GPS_MODE)
	{
		/* GPS is new used, then need initial bias 
		(1) If GPS satellite number is large than 4, then use LS bias
		(2) Use GLONASS bias or BDS bias of KF to initial GPS bias
		*/ 
		if (pMeas->prNumEachSystem[0] >= 4&&p_Ls->ls_Pvt_Info->biasValid[gnssMode])
		{
			gnss_Kf_SetClkBias(p_Ls->bias[gnssMode], gnssMode, dt);
		}else
		{
			/*
			If GLNOASS number is larger than BDS number, use GLONASS, or BDS
			*/
			if (pMeas->prNumEachSystem[1] >= pMeas->prNumEachSystem[2] && pMeas->prNumEachSystem[1] >= pMeas->prNumEachSystem[3] && p_Kf->kf_Pvt_Info->biasValid[GLN_MODE] && biasDiffUse[GLN_MODE] != 0.0)
			{
				gnss_Kf_SetClkBias(kfInfo.X[7] + biasDiffUse[GLN_MODE], gnssMode, 0);
			}
			else if(pMeas->prNumEachSystem[2] >= pMeas->prNumEachSystem[3] && pMeas->prNumEachSystem[2] >= pMeas->prNumEachSystem[1] && p_Kf->kf_Pvt_Info->biasValid[BDS_MODE] && biasDiffUse[BDS_MODE] != 0.0)
			{
				gnss_Kf_SetClkBias(kfInfo.X[8] + biasDiffUse[BDS_MODE], gnssMode, 0);
			}
			else if (pMeas->prNumEachSystem[3] >= pMeas->prNumEachSystem[1] && pMeas->prNumEachSystem[3] >= pMeas->prNumEachSystem[2] && p_Kf->kf_Pvt_Info->biasValid[GAL_MODE] && biasDiffUse[GAL_MODE] != 0.0)
			{
				gnss_Kf_SetClkBias(kfInfo.X[9] + biasDiffUse[GAL_MODE], gnssMode, 0);
			}
			else 
			{
				(*flag) = FALSE;//bias initialization fail
			}
		}
	}else
	{
		if (biasDiffUse[gnssMode] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[GPS_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[6] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if ((gnssMode == GAL_MODE) && biasDiffUse[gnssMode] != 0.0 && biasDiffUse[GLN_MODE] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[GLN_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[7] + biasDiffUse[GLN_MODE] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if ((gnssMode == GAL_MODE) && biasDiffUse[gnssMode] != 0.0 && biasDiffUse[BDS_MODE] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[BDS_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[8] + biasDiffUse[BDS_MODE] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if((gnssMode==BDS_MODE)&&biasDiffUse[gnssMode] != 0.0 && biasDiffUse[GLN_MODE] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[GLN_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[7] + biasDiffUse[GLN_MODE] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if ((gnssMode == BDS_MODE) && biasDiffUse[gnssMode] != 0.0 && biasDiffUse[GAL_MODE] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[GAL_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[9] + biasDiffUse[GAL_MODE] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if((gnssMode==GLN_MODE)&&biasDiffUse[gnssMode] != 0.0 && biasDiffUse[BDS_MODE] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[BDS_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[8] + biasDiffUse[BDS_MODE] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if ((gnssMode == GLN_MODE) && biasDiffUse[gnssMode] != 0.0 && biasDiffUse[GAL_MODE] != 0.0&&p_Kf->kf_Pvt_Info->biasValid[GAL_MODE])
		{
			gnss_Kf_SetClkBias(kfInfo.X[9] + biasDiffUse[GAL_MODE] - biasDiffUse[gnssMode], gnssMode, 0);
		}
		else if(p_Ls->ls_Pvt_Info->biasValid[gnssMode])
		{
			gnss_Kf_SetClkBias(p_Ls->bias[gnssMode], gnssMode, dt);
		}
		else
		{
			(*flag) = FALSE;// bias initialization fail
		}
	}            
	if(*flag)
	{
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Clock Bias changed:%02d,%12.4f,%12.4f",gnssMode,kfInfo.X[6] - p_Kf->biasDiff[gnssMode],p_Ls->bias[gnssMode]-kfInfo.X[10] * dt);
	}
}

/***********************************************************************
* 函数介绍: gnss_Kf_BiasChangeProc
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
void gnss_Kf_BiasChangeProc(Kf_t* p_Kf,double dt)
{
	uint8_t			   i,j,idx;
	uint16_t 		   modeDiff, modeInc, modeSame, bitMask, unLinkMode,biasInitFlag[BIAS_NUM], biasChange;
	uint16_t 		   svModeInUseFlag, lastSvModeInUseFlag, biasLinkFlag;
	double 		   biasDiff;
	LS_PVT_INFO*   lsPvtInfo;
	KF_CTRL*	   pKfCtrl;   
	Ls_t*		   p_Ls;
	meas_blk_t*    pMeas;
	gnss_meas_t*   pSvMeas;

	pKfCtrl = &(p_Kf->kf_ctrl);
	lsPvtInfo = p_Kf->ls_Pvt_Info;
	p_Ls = p_Kf->lsBlk;
	pMeas = p_Kf->meas_blk;

	svModeInUseFlag = pKfCtrl->svModeInUse;
	lastSvModeInUseFlag = pKfCtrl->lastSvModeInUse;
	biasLinkFlag = pKfCtrl->biasLinkFlag;

	biasChange = (pKfCtrl->svModeInUse != pKfCtrl->lastSvModeInUse) && pKfCtrl->lastSvModeInUse;
	/* new constellation SV used */
	if (biasChange)
	{
		bitMask = clockBiasBitMask[biasLinkFlag];
		if (biasLinkFlag == 1)
		{
			lastSvModeInUseFlag = ((lastSvModeInUseFlag >> 1)&0x1) | lastSvModeInUseFlag;
			svModeInUseFlag = ((svModeInUseFlag >> 1)&0x1) | svModeInUseFlag;
			lastSvModeInUseFlag &= 0xD;
			svModeInUseFlag &= 0xD;
		}
		else if (biasLinkFlag == 2)
		{
			lastSvModeInUseFlag = ((lastSvModeInUseFlag >> 1)&0x1) | ((lastSvModeInUseFlag >> 2)&0x1) | lastSvModeInUseFlag;
			svModeInUseFlag = ((svModeInUseFlag >> 1)&0x1) | ((svModeInUseFlag >> 2)&0x1) | svModeInUseFlag;
			lastSvModeInUseFlag &= 0x9;
			svModeInUseFlag &= 0x9;
		}
		else if (biasLinkFlag == 3)
		{
			lastSvModeInUseFlag = ((lastSvModeInUseFlag >> 1) & 0x1) | ((lastSvModeInUseFlag >> 2) & 0x1) | ((lastSvModeInUseFlag >> 3) & 0x1) | lastSvModeInUseFlag;
			svModeInUseFlag = ((svModeInUseFlag >> 1) & 0x1) | ((svModeInUseFlag >> 2) & 0x1) | ((svModeInUseFlag >> 3) & 0x1) | svModeInUseFlag;
			lastSvModeInUseFlag &= 0x1;
			svModeInUseFlag &= 0x1;
		}
		modeDiff = svModeInUseFlag ^ lastSvModeInUseFlag;
		modeInc = modeDiff & svModeInUseFlag;
		modeSame = svModeInUseFlag & lastSvModeInUseFlag;

		if (bitMask & modeSame)
		{
			unLinkMode = modeInc & bitMask;
		}
		else
		{
			unLinkMode = modeInc;
		}

		/* If bias number changed and LS validation is true */
		if (unLinkMode)
		{
			for (i = 0; i < BIAS_NUM; i++)
			{
				biasInitFlag[i] = TRUE;
				if (unLinkMode & (1 << i))
				{
					gnss_Kf_ReInit_Bias(i,p_Kf,dt,&biasInitFlag[i]);
				}
				else if (unLinkMode & (1 << (i + 4)))
				{
					if (lsPvtInfo->dcbValid[i])
					{
						kfInfo.X[11 + i] = lsPvtInfo->clkDcb[i];
					}
				}
				else if (unLinkMode & (1 << (i + 8)))
				{
					if (lsPvtInfo->dcbValid[i + 4])
					{
						kfInfo.X[11 + i + 4] = lsPvtInfo->clkDcb[i + 4];
					}
				}
			}

			//if bias init fail, discard this mode's satellites
			for(j =0;j<pMeas->measCnt;j++)
			{
				pSvMeas = &pMeas->meas[j];
				if(!biasInitFlag[pSvMeas->gnssMode])
				{
					pSvMeas->status &=0xFE;
					pKfCtrl->svModeInUse &= ~(1 << pSvMeas->gnssMode);
				}
			}
		}
	}
#if 1
	if (lsPvtInfo->fix_status == FIX_STATUS_NEW)
	{
		for (i = 0; i < BIAS_NUM;i++)
		{
			idx = clockBiasIdx[i][biasLinkFlag];
			if (svModeInUseFlag & (1 << i))
			{
				biasDiff = (lsPvtInfo->clkBias[i] + lsPvtInfo->clkDrift*(kfInfo.tor-lsPvtInfo->tor))- (kfInfo.X[idx] + kfInfo.X[10] * dt);
				if (fabs(biasDiff) > 50 && pMeas->avgCno >= 25 && p_Ls->pos_res < 50.0 && p_Ls->pos_res > 0.0)
				{
					//gnss_Kf_SetClkBias(lsPvtInfo->clkBias[i], i, biasLinkFlag,dt);
					//SYS_LOGGING(OBJ_KF,LOG_INFO,"Clock Bias changed for gnss(%d),biasDiff(%12.3f)",i,biasDiff);
				}
			}
		}
	}
#endif
}


/***********************************************************************
* 函数介绍: gnss_Kf_Adjust_Qp
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：08/12/
***********************************************************************/
static double gnss_Kf_Adjust_Qp(Kf_t *p_Kf, uint8_t ppk_mode)
{
	static uint8_t      cnt1 = 0,cnt2 = 0;
	double            Qp = 0.0;
	float            vel;
	meas_blk_t*    pMeas;
	KF_PVT_INFO*   kfPvtInfo;
	//KF_PVT_INFO*   kfPvtBack;

	pMeas = p_Kf->meas_blk;
	kfPvtInfo = p_Kf->kf_Pvt_Info;
	//kfPvtBack = p_Kf->kf_Pvt_Back;

	if (!ppk_mode)
	{
		vel = kfPvtInfo->kfHeadingVel;
	}
	else
	{
		vel = (float)sqrt((double)kfPvtInfo->kfENUvel[0]* kfPvtInfo->kfENUvel[0] + (double)kfPvtInfo->kfENUvel[1] * kfPvtInfo->kfENUvel[1] +(double) kfPvtInfo->kfENUvel[2] * kfPvtInfo->kfENUvel[2]);
	}
	/*
	* Adjust Qp according to environments and user dynamic
	*/
	if (vel > 10.0)
	{
		cnt2 = 0;
		if (cnt1 < 4)
		{
			cnt1++;
		}else
		{
			if (vel >= 18.0)
			{
				Qp = 5.0;
			}else if (vel >= 13.0)
			{
				Qp = 4.0;
			}else
			{
				Qp = 2.0;
			}

		}
	}else if (vel < 8.0)
	{
		cnt1 = 0;
		if (cnt2 < 4)
		{
			cnt2++;
		}else
		{
			Qp = 0.0;
		}
	}

	SYS_LOGGING(OBJ_KF,LOG_INFO,"%s,%14.6f,%02d,%02d,%3.1f",__FUNCTION__,pMeas->tor,cnt1,cnt2,Qp);
	return Qp;
}

/***********************************************************************
* 函数介绍: gnss_Kf_Adjust_QRatio
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：08/12/
***********************************************************************/
static float gnss_Kf_Adjust_QRatio(Kf_t *p_Kf,uint8_t ppk_mode)
{
	static uint8_t      cnt1 = 0,cnt2 = 0,highSpeed = 0,lowSpeed = 1;
	static float     ratio = 1.0;
	meas_blk_t*    pMeas;
	KF_PVT_INFO*   kfPvtInfo;
	KF_PVT_INFO*   kfPvtBack;
	float            vel;

	/* protection */
	if (p_Kf == NULL)
	{
		return ratio;
	}

	pMeas = p_Kf->meas_blk;
	kfPvtInfo = p_Kf->kf_Pvt_Info;
	kfPvtBack = p_Kf->kf_Pvt_Back;

	if (pMeas == NULL || kfPvtInfo == NULL || kfPvtBack == NULL)
	{
		return ratio;
	}

	if (!ppk_mode)
	{
		vel = kfPvtInfo->kfHeadingVel;
	}
	else
	{
		vel = (float)sqrt((double)kfPvtInfo->kfENUvel[0] * kfPvtInfo->kfENUvel[0] + (double)kfPvtInfo->kfENUvel[1] * kfPvtInfo->kfENUvel[1] + (double)kfPvtInfo->kfENUvel[2] * kfPvtInfo->kfENUvel[2]);
	}

	/*
	*	This function is aimed to enlarge Q or reduce Q in some special environment or 
	signal condition.
	1) For high speed and good signal environment, we can use large Q to trust measurements more;
	2)
	*/
	if (vel > 10.0 && pMeas->measCnt >= 10)
	{
		cnt2 = 0;
		if (cnt1 < 10)
		{
			cnt1++;
		}
		if (cnt1 >= 10)
		{
			highSpeed = 1;
			lowSpeed = 0;
		}
	}else if (vel < 8.0 || pMeas->measCnt < 8)
	{
		cnt1 = 0;
		if (cnt2 < 10)
		{
			cnt2++;
		}
		if (cnt2 >= 10)
		{
			highSpeed = 0;
			lowSpeed = 1;
		}
	}

	if (lowSpeed)
	{
		ratio = 1.0;
	}else
	{
		ratio = (float)(1.0 + (vel / 10.0) * (vel / 10.0));
	}

	SYS_LOGGING(OBJ_KF,LOG_INFO,"%s,%14.6f,%10.4f,%02d,%02d,%02d,%02d,%3.1f",__FUNCTION__,pMeas->tor, vel,pMeas->avgCno,pMeas->measCnt,highSpeed,lowSpeed,ratio);
	return ratio;
}

/***********************************************************************
* 函数介绍: gnss_Kf_Adjust_Q
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/12/
***********************************************************************/
void gnss_Kf_Adjust_Q(Kf_t *p_Kf, double T)
{
	uint8_t             i;
	float            scale,QRatio = 1.0;
	float            /*acc = 0,*/acc_kf = 0/*,acc_ls = 0*/;
#ifndef PVMODE
	float            acc_ecef[3];
	float            acc_enu[3];
	float            weight2;
#endif
	float            weight1;
	float            sigmaInit = 1.0,lowLmt = 0.5;
	float            vel = 0, vel_last = 0;
	double            dt;
	KF_PVT_INFO*   kfPvtInfo;
	KF_PVT_INFO*   kfPvtBack;
	LS_PVT_INFO*   lsPvtInfo;

	Ls_t*          p_Ls;
	meas_blk_t*    pMeas;
	STATIC_DATA*   pStaticData;

	kfPvtInfo = p_Kf->kf_Pvt_Info;
	kfPvtBack = p_Kf->kf_Pvt_Back;
	p_Ls = p_Kf->lsBlk;
	pMeas = p_Kf->meas_blk;
	pStaticData = &peMode.staticData;
	lsPvtInfo = p_Kf->ls_Pvt_Info;
	/*****************************************************\
	* Following routine is mainly used for dynamic adjust Q matrix
	\******************************************************/
	/* Acc noise sigma adjust */
	if (((kfPvtInfo->tor > kfPvtBack->tor && !IS_PPK_REVERSE)|| (kfPvtInfo->tor < kfPvtBack->tor && IS_PPK_REVERSE) )
		&& kfPvtBack->tor>0)
	{
#ifndef PVMODE
		weight1 = (float)0.3;
		weight2 = (float)0.3;
		// calculate KF state acc
		for (i = 0; i < 3; i++)
		{
			acc_ecef[i] = (float)kfInfo.X[i + 11];
		}
		gnssConvEcef2EnuVel(acc_ecef,acc_enu,kfPvtInfo->kfLLApos);
		for (i = 0; i < 2;i++)
		{
			acc += acc_enu[i] * acc_enu[i];
		}
		acc = (float)sqrt(acc);
		// calculate KF velocity generated acc
		for (i = 0; i < 2;i++)
		{
			acc_kf += (kfPvtInfo->kfENUvel[i] - kfPvtBack->kfENUvel[i]) * (kfPvtInfo->kfENUvel[i] - kfPvtBack->kfENUvel[i]);
		}
		acc_kf = (float)(sqrt(acc_kf) / (kfPvtInfo->tor - kfPvtBack->tor));

		// calculate LS  velocity generated acc
		scale = weight1 * acc + (1 - weight2) * acc_kf;
#else
		weight1 = (float)0.2;
		if (lsPvtInfo->valid == FALSE)
		{
			weight1 = (float)1.0;
		}
		// calculate KF velocity generated acc
		for (i = 0; i < 2;i++)
		{
			acc_kf += (kfPvtInfo->kfENUvel[i] - kfPvtBack->kfENUvel[i]) * (kfPvtInfo->kfENUvel[i] - kfPvtBack->kfENUvel[i]);
		}
		dt = kfPvtInfo->tor - kfPvtBack->tor;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
		acc_kf = (float)(sqrt(acc_kf) / fabs(dt));
		if (p_Ls->vel_std > 0.0)
		{
			scale = weight1 * acc_kf + (1 - weight1) * p_Ls->vel_std;
		}
		else
		{
			scale = acc_kf;
		}
#endif

	}
	else if (p_Ls->vel_std > 0.0)
	{
		scale = p_Ls->vel_std;
	}
	else
	{
		scale = 0.5;
	}

	/* when KF start, use default sigma */
	if (p_Kf->kfCnt < 20)
	{
		sigmaInit = 1.0;
	}else
	{
		sigmaInit = (float)(scale);
	}


	/*Adjust Q according to environments */
	if (g_pe_cfg.chipType == SPRD && pMeas->rtdUsed)
	{
		QRatio = gnss_Kf_Adjust_QRatio(p_Kf, g_pe_cfg.ppk_mode);
		p_Kf->Qp = gnss_Kf_Adjust_Qp(p_Kf,g_pe_cfg.ppk_mode);
	}
	else if((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == 0)
	{
		QRatio = gnss_Kf_Adjust_QRatio(p_Kf, g_pe_cfg.ppk_mode);
		p_Kf->Qp = gnss_Kf_Adjust_Qp(p_Kf, g_pe_cfg.ppk_mode);
	}
	else
	{
		QRatio = 1.0;
		p_Kf->Qp = 0.0;
	}

	sigmaInit *= QRatio;

	/*
	*  Refine Q according to velocity
	*  1. If velocity is larger than 15m/s, then use 1.0
	*  2. If velocity is larger than 10m/s and less than 15m/s, then use 0.6
	*  3. If velocity is larger than 8m/s and less than 10m/s, then use 0.3
	*/
	if (!g_pe_cfg.ppk_mode)
	{
		vel = kfPvtInfo->kfHeadingVel;
		vel_last = kfPvtBack->kfHeadingVel;
	}
	else
	{
		vel = (float)sqrt((double)kfPvtInfo->kfENUvel[0] * kfPvtInfo->kfENUvel[0] + (double)kfPvtInfo->kfENUvel[1] * kfPvtInfo->kfENUvel[1] + (double)kfPvtInfo->kfENUvel[2] * kfPvtInfo->kfENUvel[2]);
		vel_last = (float)sqrt((double)kfPvtBack->kfENUvel[0] * kfPvtBack->kfENUvel[0] + (double)kfPvtBack->kfENUvel[1] * kfPvtBack->kfENUvel[1] + (double)kfPvtBack->kfENUvel[2] * kfPvtBack->kfENUvel[2]);
		lowLmt = 2.0;
	}
	if (pMeas->measCnt >= 10)
	{
		if (vel > 15.0 && vel_last > 15.0)               // lower speed
		{
			if (sigmaInit < 3*lowLmt)
			{
				sigmaInit  = 3*lowLmt;
			}      
		}else if (vel > 10.0 && vel_last > 10.0)
		{
			if (sigmaInit < 2*lowLmt)
			{
				sigmaInit  = 2*lowLmt;
			}    
		}else if (vel > 8.0 && vel_last > 8.0)
		{
			if (sigmaInit < lowLmt)
			{
				sigmaInit  = lowLmt;
			}
		}
	}

	/* Ab-normal very large accSigma detected */
	if (sigmaInit > 5.0 && (g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR))
	{
		sigmaInit = 5.0;
	} 

	if (!(g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR))
	{
		if (sigmaInit < 3.0)
		{
			sigmaInit = 3.0;
		}
		if (p_Kf->Qp < 1.0)
		{
			p_Kf->Qp = 1.0;
		}
	}

	
#ifndef PVMODE
	p_Kf->Qv = 0.0;
	p_Kf->Qa = sigmaInit;
#else
	p_Kf->Qv = sigmaInit;
	p_Kf->Qa = 0.0;
#endif

	/* From static to dynamic, adjust P matrix */
	if ((pStaticData->historyStatic & 0x3) == 0x2)
	{
#ifdef ZUPT
		if (g_pe_cfg.chipType == QCOM && !peMode.userContext.isVehicleMode)
		{
			gnss_Kf_Adjust_PMinus(&kfInfo,p_Kf);
			p_Kf->enlargeQ = TRUE;
			p_Kf->QFactor = 2.0;
		}
#endif  
		//if(g_pe_cfg.chipType != SPRD)
		//{
		//	gnss_Kf_Restore_Filter(p_Kf,&kfInfo);
		//	p_Kf->enlargeQ = TRUE;
		//	p_Kf->QFactor = 2.0;
		//}
	}

	gnss_Kf_QmatInit(p_Kf,T,&sigmaInit);
}

/***********************************************************************
* 函数介绍: gnss_Kf_PR_Adj
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/01/
***********************************************************************/
void gnss_Kf_PR_Adj(Kf_t* p_Kf,uint8_t gnssMode,double* pseudoRange)
{
	meas_blk_t*      pMeas;
	pMeas = p_Kf->meas_blk;


	if (p_Kf->kf_ctrl.biasLinkFlag == 1)
	{
		if (gnssMode == GLN_MODE)
		{
			if (pMeas->rtdUsed)
			{
				(*pseudoRange) += p_Kf->biasDiff[GLN_MODE];
			}else
			{
				(*pseudoRange) += p_Kf->biasDiffLocal[GLN_MODE];
			}
		}
	}else if (p_Kf->kf_ctrl.biasLinkFlag == 2)
	{
		if (pMeas->rtdUsed)
		{   
			if (gnssMode == GLN_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiff[GLN_MODE];
			}else if (gnssMode == BDS_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiff[BDS_MODE];
			}
		}else
		{
			if (gnssMode == GLN_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiffLocal[GLN_MODE];
			}else if (gnssMode == BDS_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiffLocal[BDS_MODE];
			}
		}
	}else if (p_Kf->kf_ctrl.biasLinkFlag == 3)
	{
		if (pMeas->rtdUsed)
		{   
			if (gnssMode == GLN_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiff[GLN_MODE];
			}else if (gnssMode == BDS_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiff[BDS_MODE];
			}else if (gnssMode == GAL_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiff[GAL_MODE];
			}
		}else
		{
			if (gnssMode == GLN_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiffLocal[GLN_MODE];
			}else if (gnssMode == BDS_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiffLocal[BDS_MODE];
			}else if (gnssMode == GAL_MODE)
			{
				(*pseudoRange) += p_Kf->biasDiffLocal[GAL_MODE];
			}

		}
	}
}

/***********************************************************************
* 函数介绍: gnss_Kf_Predict
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/12/
***********************************************************************/
void gnss_Kf_Predict(Kf_t *p_Kf,double dt)
{
	uint8_t             i;
	double            range, /*range1,*/ toa;
	double            satPosCorr[3];
	double			   dKfInfoXPre[6] = { 0 };
	gnss_meas_t*   pSvMeas;
	SV_INFO*       pSvInfo;
	meas_blk_t*    pMeas;

#if defined(PLAYBACK_MODE)
	float      fVelbefor[8] = { 0 }, fVelAfter[8] = { 0 }, fVelAcc[8] = { 0 };
	double      dKfInfoXAcc[6] = { 0 };
	float      fPosError_Vel[8] = { 0 }, fPosError_Deltx[8] = { 0 };
#endif
	pMeas = p_Kf->meas_blk;

	SYS_LOGGING(OBJ_KF, LOG_INFO, "KF update dt:%16.8f", dt);
	/* If KF rate changed, then use new system transit matrix 'A' and system noise 'Q' */
	if (fabs(dt - kfInfo.periodTms * 0.001) > 0.001)
	{
		SYS_LOGGING(OBJ_KF, LOG_INFO, "KF data rate change: old(%d) new(%d)", kfInfo.periodTms, (uint32_t)(dt * 1000));
		kfInfo.periodTms = (int16_t)(dt * 1000);
		gnss_Kf_AmatInit(dt);
		gnss_Kf_QmatInit(p_Kf, dt, NULL);
	}

	/* Adjust Q */
	gnss_Kf_Adjust_Q(p_Kf, dt);

	/* bias change detection for multi-sys switch */
	if (!(p_Kf->lastKfStatus & KF_RESTART) && !(p_Kf->lastKfStatus & KF_RESET))
	{
		gnss_Kf_BiasChangeProc(p_Kf, dt);
	}

	g_pvtunc.prediff.preposf = 0;

	//Raw KfInfoX
	for ( i = 0; i < 6; i++)
	{
		dKfInfoXPre[i] = kfInfo.X[i];
	}
	//PV Mode to PVA Mode   Vel =Vel + Acc
	if (peMode.feedbackFlag.uAccVdrFeedback && peMode.feedbackFlag.uEleVdrFeedback)
	{
		for (i = 0; i < 3; i++)
		{
			kfInfo.X[i + 3] = kfInfo.X[i + 3] + vdrFeedbackInfo.fAccIne[i] * dt;
		}
	}
	/* KF prediction */
	if (p_Kf->kfCnt > 0)
	{
		udKfPredict();
#if defined(PLAYBACK_MODE)
		for (i = 0; i < 6; i++)
		{
			dKfInfoXAcc[i] = kfInfo.X[i];
		}
#endif // !
		if (peMode.feedbackFlag.uDeltxVdrFeedback && peMode.feedbackFlag.uEleVdrFeedback)
		{
			for (i = 0; i < 3; i++)
			{
				kfInfo.X[i] = dKfInfoXPre[i] + vdrFeedbackInfo.fDistance_XYZ[i];
			}
		}
#if defined(PLAYBACK_MODE)
		if (pMeas->hpRefQflag[1])
		{
			for ( i = 0; i < 3; i++)
			{
				fVelbefor[i] = (float)(dKfInfoXPre[i+3] - pMeas->hpRefEcef[i+3]);
			}
			gnssConvEcef2EnuVel(fVelbefor, fVelbefor + 4, pMeas->hpRefLla);
			fVelbefor[7] =(float) sqrt((double)fVelbefor[4] * fVelbefor[4] + (double)fVelbefor[5] * fVelbefor[5]);

			for (i = 0; i < 3; i++)
			{
				fVelAcc[i] = (float)(dKfInfoXAcc[i+3] - pMeas->hpRefEcef[i+3]);
			}
			gnssConvEcef2EnuVel(fVelAcc, fVelAcc + 4, pMeas->hpRefLla);
			fVelAcc[7] = (float)sqrt((double)fVelAcc[4] * fVelAcc[4] + (double)fVelAcc[5] * fVelAcc[5]);

			for ( i = 0; i < 3; i++)
			{
				fPosError_Vel[i] = (float)(dKfInfoXAcc[i] - pMeas->hpRefEcef[i]);
				fPosError_Deltx[i] = (float)(kfInfo.X[i] - pMeas->hpRefEcef[i]);
			}
			gnssConvEcef2EnuVel(fPosError_Vel, fPosError_Vel + 4, pMeas->hpRefLla);
			fPosError_Vel[7] = (float)sqrt((double)fPosError_Vel[4] * fPosError_Vel[4] +(double)fPosError_Vel[5] * fPosError_Vel[5]);

			gnssConvEcef2EnuVel(fPosError_Deltx, fPosError_Deltx + 4, pMeas->hpRefLla);
			fPosError_Deltx[7] = (float)sqrt((double)fPosError_Deltx[4] * fPosError_Deltx[4] + (double)fPosError_Deltx[5] * fPosError_Deltx[5]);

		}
		GLOGI("KFVelAccFeedback:%8.3f %d %d %d  %d %8.4f %8.4f %8.4f %8.4f ", pMeas->tor, peMode.feedbackFlag.uAccVdrFeedback, peMode.feedbackFlag.uEleVdrFeedback, peMode.feedbackFlag.uVelVdrFeedback, peMode.feedbackFlag.uDeltxVdrFeedback,
			fVelAcc[7], fVelbefor[7], fPosError_Deltx[7], fPosError_Vel[7]);
#endif // !
		g_pvtunc.prediff.preposf = 1;
		for (i = 0; i < 3; i++)
		{
			g_pvtunc.prediff.prepos[i] = kfInfo.X[i];
		}
	}

	/* If restart and LS converge, then use LS bias */
	if ((p_Kf->lastKfStatus  & KF_RESTART) && (g_pe_cfg.chipType == SPRD))
	{
		if (p_Kf->lsBlk->posCvg == TRUE)
		{
			for (i = 0; i < BIAS_NUM; i++)
			{
				kfInfo.X[6 + i] = p_Kf->lsBlk->bias[i];
			}
		}
		if(p_Kf->lsBlk->ls_Pvt_Info->isDriftValid)
		{
			kfInfo.X[10] = p_Kf->lsBlk->drift;
		}
	}
	/* prepare dcos */
	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];
		if (pSvMeas->prn == 0) continue;
		pSvInfo = &pSvMeas->sv_info;
		range = gnssClcSqrtAminusB_DBL(kfInfo.X, pSvInfo->p, 3);
		if (range == 0.0) continue;
#if 0
		range1 = 1 / range;
		pSvInfo->dcos[0] = ((kfInfo.X[0] - pSvInfo->p[0]) * range1);
		pSvInfo->dcos[1] = ((kfInfo.X[1] - pSvInfo->p[1]) * range1);
		pSvInfo->dcos[2] = ((kfInfo.X[2] - pSvInfo->p[2]) * range1);
#endif
		//earth rotation correction
		toa = range / LIGHT_SEC;
		gnssEarthRotateCorr(pSvInfo->p, satPosCorr, toa);
		pSvMeas->range = gnssClcSqrtAminusB_DBL(kfInfo.X, satPosCorr, 3);

		//calculate the direction cosine:
		pSvInfo->dcos[0] = ((kfInfo.X[0] - pSvInfo->p[0]) / pSvMeas->range);
		pSvInfo->dcos[1] = ((kfInfo.X[1] - pSvInfo->p[1]) / pSvMeas->range);
		pSvInfo->dcos[2] = ((kfInfo.X[2] - pSvInfo->p[2]) / pSvMeas->range);
#if 0
		if (p_Kf->kf_ctrl.biasLinkFlag == 1)
		{
			if (pSvMeas->gnssMode == GLN_MODE)
			{
				pSvMeas->pseudoRange += p_Kf->biasDiff[GLN_MODE];
			}
		}else if (p_Kf->kf_ctrl.biasLinkFlag == 2)
		{
			if (pSvMeas->gnssMode == GLN_MODE)
			{
				pSvMeas->pseudoRange += p_Kf->biasDiff[GLN_MODE];
			}else if (pSvMeas->gnssMode == BDS_MODE)
			{
				pSvMeas->pseudoRange += p_Kf->biasDiff[BDS_MODE];
			}
		}else if (p_Kf->kf_ctrl.biasLinkFlag == 3)
		{
			if (pSvMeas->gnssMode == GLN_MODE)
			{
				pSvMeas->pseudoRange += p_Kf->biasDiff[GLN_MODE];
			}else if (pSvMeas->gnssMode == BDS_MODE)
			{
				pSvMeas->pseudoRange += p_Kf->biasDiff[BDS_MODE];
			}else if (pSvMeas->gnssMode == GAL_MODE)
			{
				pSvMeas->pseudoRange += p_Kf->biasDiff[GAL_MODE];
			}
		}
#endif
	}

	if (g_pe_cfg.chipType == SPRD||g_pe_cfg.chipType == QCOM||g_pe_cfg.chipType == UBLOX)
	{
		/* If re-start and SPRD, do bias/drift check */
		gnss_Kf_Bias_Adjust(p_Kf);
		gnss_Kf_Drift_Adjust(p_Kf);
	}

	p_Kf->lastRtdStatus = pMeas->rtdUsed;
}

/***********************************************************************
* 函数介绍: gnss_Kf_Altitude_Hold
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/30/
***********************************************************************/
void gnss_Kf_Altitude_Hold(double lla[],float altitudeRef,double var,double* deltaX)
{
	double      H[N_STATE] = {0.0};
	double      cosLat, cosLon, sinLat, sinLon;
	double      altitude;
	//double      test;
	double      innovation, variance;

	cosLat = cos(lla[0]);
	sinLat = sin(lla[0]);
	cosLon = cos(lla[1]);
	sinLon = sin(lla[1]);

	H[0] = cosLat * cosLon;
	H[1] = cosLat * sinLon;
	H[2] = sinLat;

	altitude = lla[2];
	variance = var;

	innovation = altitudeRef - altitude;
	udKFUpdate(H, deltaX, variance, innovation,SBOUND_PR ,COAST_UPDATE, 1,0);

}

/***********************************************************************
* 函数介绍: gnss_Kf_Clock_Hold
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/30/
***********************************************************************/
void gnss_Kf_Clock_Hold(float clkDriftRef,double* deltaX)
{
	double      H[N_STATE] = {0.0};
	double      innovation, variance;
	//double      test;

	H[10] = -1.0;
	variance = 4.0;

	innovation = kfInfo.X[10] - (double)clkDriftRef;
	udKFUpdate(H, deltaX, variance, innovation,SBOUND_DR, COAST_UPDATE, 1,0);
}
/***********************************************************************
* 函数介绍: gnss_Kf_Heading_Hold
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/30/
***********************************************************************/
void gnss_Kf_Heading_Hold(double lla[],float enuVelRef[],double* deltaX)
{
	uint8_t       i;
	double      cosLat, cosLon, sinLat, sinLon, sinHeading, cosHeading;
	double      horVel;
	double      H[N_STATE] = {0.0};
	double      innovation, variance;
	//double      test;

	cosLat = cos(lla[0]);
	sinLat = sin(lla[0]);
	cosLon = cos(lla[1]);
	sinLon = sin(lla[1]);

	//calculate horizontal speed
	horVel = sqrt((double)enuVelRef[0] * enuVelRef[0] + (double)enuVelRef[1] * enuVelRef[1]);
	if (horVel < 1e-3) return;

	sinHeading = enuVelRef[0] / horVel;  // Ve/hor
	cosHeading = enuVelRef[1] / horVel;  // Vn/hor

	H[3] = -sinLat * cosLon * sinHeading + sinLon * cosHeading;
	H[4] = -sinLat * sinLon * sinHeading - cosHeading * cosLon;
	H[5] = sinHeading * cosLat;

	innovation = 0;

	for (i = 3; i < 6; i++)
	{
		innovation += H[i] * kfInfo.X[i];
	}

	innovation = -innovation;

	if (horVel > 2) // > 2m/s
	{
		variance = 0.05;
	}
	else
	{
		variance = 0.1;
	}

	udKFUpdate(H, deltaX, variance, innovation,SBOUND_DR, COAST_UPDATE, 1,0);
}

/***********************************************************************
* 函数介绍: gnss_Kf_Heading_Hold_Proc
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：03/22/
***********************************************************************/
void gnss_Kf_Heading_Hold_Proc(double lla[], float headingRef, double* deltaX)
{
	float      horVel;
	float      enuVel[3], ecefVel[3];

	ecefVel[0] = (float)kfInfo.X[3];
	ecefVel[1] = (float)kfInfo.X[4];
	ecefVel[2] = (float)kfInfo.X[5];
	gnssConvEcef2EnuVel(ecefVel, enuVel, lla);

	horVel = (float)sqrt((double)enuVel[0] * enuVel[0] + (double)enuVel[1] * enuVel[1]);
	enuVel[1] = (float)(horVel * cos(headingRef * DEG2RAD));
	enuVel[0] = (float)(horVel * sin(headingRef * DEG2RAD));
	gnss_Kf_Heading_Hold(lla,enuVel,deltaX);
	SYS_LOGGING(OBJ_KF,LOG_INFO,"Heading hold:%f",headingRef * DEG2RAD);
}

/**********************************************************************
* Function Name:    gnss_Kf_CoastMode
*
* Description:
*    The main function of KF
*
* Input:
*     deltaX:      KF update correction
*     mode:        fix mode
*     clkDriftRef: reference clock drift
*     altitudeRef
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_CoastMode(KF_PVT_INFO* kfPvtInfo,double* deltaX)
{
	double      lla[3];
	history_Info p;

	gnss_Pe_Get_HisInfo(&p);

	gnssConvEcef2Lla(kfInfo.X, lla);

	if (kfPvtInfo->kfFixMode == COAST3 || kfPvtInfo->kfFixMode == COAST2 || kfPvtInfo->isAltHold == TRUE) // altitude hold
	{
		kfPvtInfo->altVar = 2*fabs(kfPvtInfo->kfENUvel[2]);
		if(kfPvtInfo->altVar < 5.0 )
		{
			kfPvtInfo->altVar = 5.0;
		}
		else if(kfPvtInfo->altVar > 20.0 )
		{
			kfPvtInfo->altVar = 20.0;
		}

		kfPvtInfo->altVar *= kfPvtInfo->altVar;
		if (peState.posHasBias)
		{
			kfPvtInfo->altitudeRef = (float)p.preciseAlt;
		}
		gnss_Kf_Altitude_Hold(lla,kfPvtInfo->altitudeRef,kfPvtInfo->altVar,deltaX);
#if defined(PLAYBACK_MODE)
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Altitude hold:%f %f",kfPvtInfo->altitudeRef,kfPvtInfo->altVar);
#endif
	}

	if ((kfPvtInfo->kfFixMode == COAST2 || kfPvtInfo->kfFixMode == COAST1) && (fabs(kfPvtInfo->clkDriftRef) > FLT_EPSILON)) //clock drift hold
	{
		gnss_Kf_Clock_Hold(kfPvtInfo->clkDriftRef,deltaX);
#if defined(PLAYBACK_MODE)
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Clock hold:%f",kfPvtInfo->clkDriftRef);
#endif
	}

	if (kfPvtInfo->kfFixMode == COAST1 && fabs(kfPvtInfo->headingRef) > 0) //heading hold
	{
		gnss_Kf_Heading_Hold_Proc(lla, kfPvtInfo->headingRef,deltaX);
#if defined(PLAYBACK_MODE)
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Heading hold:%f",kfPvtInfo->headingRef * DEG2RAD);
#endif
	}
}

/**********************************************************************
* Function Name:    gnss_Kf_DrUpdate
*
* Description:
*    update KF by dr measurements
*
* Input:
*     dcos:         direction cosine
*     svVel:        sv velocity
*     dr:           dr measurement
*     drNoiseVar:   dr noise variance
*     flag:         KF update flag
*                   0 -- not save update results in KF
*                   1 -- save update results in KF
* Output:
*     deltaX:   KF state update values
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnss_Kf_DrUpdate(gnss_meas_t* pSvMeas,double* deltaX, float dr, float* drNoiseVar, double* testOut,uint8_t* updateFlag, uint8_t flag,uint8_t testFlag)
{
	uint8_t        j, updateCnt = 0, i = 0;
	float       drNoiseVarTmp;
	double       test, tmp,testThres,testThresOut;
	double       H[N_STATE] = {0};
	double       innovation;
	double       drKf;
	double*      dcos;
	float*      svVel;
	//float       fuser_vel[3] = { 0 };
	double       user_vel[3] = { 0 };


	drNoiseVarTmp = *drNoiseVar;

	dcos = &(pSvMeas->sv_info.dcos[0]);
	svVel = &(pSvMeas->sv_info.v[0]);
	// prepare the H matrix for KF DR update
	H[3] = dcos[0];
	H[4] = dcos[1];
	H[5] = dcos[2];
	H[10] = 1;

	// estimate DR
	drKf = 0;
	
	if (0 == flag)
	{
		if (peMode.feedbackFlag.uVelVdrFeedback)
		{
			for ( i = 0; i < 3; i++) user_vel[i] = (double)vdrFeedbackInfo.fvelEcef[i];
		}
		else
		{
			user_vel[0] = kfInfo.X[3];
			user_vel[1] = kfInfo.X[4];
			user_vel[2] = kfInfo.X[5];
		}
	}
	else
	{
		user_vel[0] = kfInfo.X[3];
		user_vel[1] = kfInfo.X[4];
		user_vel[2] = kfInfo.X[5];
	}

	for (j = 0; j < 3; j++)
	{
		drKf += dcos[j] * (user_vel[j] + deltaX[j + 3] - (double)svVel[j]);
	}

	drKf += kfInfo.X[10] + deltaX[10];

	innovation = dr - drKf;
	if(g_pe_cfg.automobile)
	{
		testThres = SBOUND_DR;
		testThresOut = SBOUND_DR_OUT;
	}
	else
	{
		testThres = SBOUND_DR ;
		testThresOut = SBOUND_DR_OUT;
		if (peMode.userSceData.isPDRMode)
		{
			if ((pSvMeas->quality & PR_GOOD) == 0)
			{
				testThres = 1.0;
			}
		}
	}

	test = udKFUpdate(H, deltaX, drNoiseVarTmp, innovation,testThres, MEAS_DR_UPDATE, flag,testFlag);
	*updateFlag = KF_MEAS_UPDATE;

	/*DR de-weight use*/
	if (testFlag)
	{
		while (test > testThres && test < testThresOut && updateCnt < 3)
		{
			SYS_LOGGING(OBJ_KF,LOG_INFO,"De-weight use DR");
			tmp = test / testThres;
			drNoiseVarTmp = drNoiseVarTmp * (float)tmp * (float)tmp * 4;
			test = udKFUpdate(H, deltaX, (double)drNoiseVarTmp, innovation,testThres, MEAS_DR_UPDATE, flag,testFlag);
			updateCnt++;
			*updateFlag = KF_MEAS_DEWEIGHT;
		}
		if(test>testThres)
		{
			*updateFlag = KF_MEAS_REJECT;
		}
	}
	*drNoiseVar = drNoiseVarTmp;
	if (testOut)
	{
		*testOut = test;
	}
	return innovation;
}


/**********************************************************************
* Function Name:    gnss_Kf_PrUpdate
*
* Description:
*    update KF by pr measurements
*
* Input:
*     dcos:         direction cosine
*     r:            KF predicted range between receiver and SV
*     pr:           pseudo range measurement
*     prNoisVar:    pr noise variance
*     deweightCnt   pr deweight cnt
*     idx:          bias index
*     flag:         KF update flag
*                   0 -- not save update results in KF
*                   1 -- save update results in KF
* Output:
*     deltaX:   KF state update values
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
double gnss_Kf_PrUpdate(gnss_meas_t* pSvMeas, double* deltaX, double pr, float* prNoisVar, uint8_t* updateFlag, double* testOut,uint8_t flag,uint8_t testFlag, uint8_t idx)
{
	uint8_t       j, updateCnt = 0,deWeightCnt;
	double      H[N_STATE] = {0.0};
	double      innovation;
	double      test, tmp, testThres,testThresRej,testThresOut;
	double      prKf;
	float      prNoiseTmp;
	double*     dcos,r;
	meas_blk_t* pMeas;
	Kf_t*    p_Kf;

	pMeas = gnss_Pe_Get_Meas();
	p_Kf = gnss_Pe_Get_Kf();

	dcos = &(pSvMeas->sv_info.dcos[0]);
	r = pSvMeas->range;
	deWeightCnt = pSvMeas->prChckCnt;

	prNoiseTmp = *prNoisVar;
	// prepare the H matrix for KF PR update
	H[0] = dcos[0];
	H[1] = dcos[1];
	H[2] = dcos[2];
	H[idx] = 1;

	if (pSvMeas->freq_index == 1)
	{
		H[pSvMeas->gnssMode + 11] = -1;
	}
	else if (pSvMeas->freq_index == 2)
	{
		H[pSvMeas->gnssMode + 11 + GNSS_MAX_MODE] = -1;
	}

	// estimate PR
	prKf = r + kfInfo.X[idx];
	if (pSvMeas->freq_index == 1)
	{
		pr += kfInfo.X[pSvMeas->gnssMode + 11];
	}
	else if (pSvMeas->freq_index == 2)
	{
		pr += kfInfo.X[pSvMeas->gnssMode + 11 + GNSS_MAX_MODE];
	}

	for (j = 0; j < 3; j++)
	{
		prKf += dcos[j] * deltaX[j];
	}

	prKf += deltaX[idx];

	if (pSvMeas->freq_index == 1)
	{
		prKf -= deltaX[pSvMeas->gnssMode + 11];
	}
	else if (pSvMeas->freq_index == 2)
	{
		prKf -= deltaX[pSvMeas->gnssMode + 11 + GNSS_MAX_MODE];
	}

	innovation = (pr - prKf);

	if(g_pe_cfg.automobile)
	{
		testThres = SBOUND_PR ;
		testThresOut = SBOUND_PR_OUT;
		testThresRej = SBOUND_PR_REJ;
	}
	else
	{
		testThres = SBOUND_PR ;
		testThresOut = SBOUND_PR_OUT;
		testThresRej = SBOUND_PR_REJ;
	}
	/* If good SV, then enlarge test threshold for SPRD */
	if (g_pe_cfg.chipType == SPRD)
	{
		if(!pSvMeas->isPrChiSqTest || peState.posHasBias)
		{
			testThres *= 3.0;
			testThresOut *= 3.0;
			testThresRej *= 3.0;
		}
	}else if (g_pe_cfg.chipType == QCOM)
	{
		if (peState.posHasBias && !p_Kf->isKfRsltGood)
		{
			testThres *= 2.0;
			testThresOut *= 2.0;
			testThresRej *= 2.0;
		}
		else if (peMode.userSceData.isPDRMode)
		{
#if 1
			if (pMeas->prDiff10Cnt >= 8 && (pSvMeas->quality & PR_GOOD) == 0)
			{
				testThres = 1.0;
			}else
			{
				testThres = 2.0;
			}
#else
			testThres = 1.0;
#endif
		}
	}

	/* If innovation is small and R is large, then don't do test check */
	if (fabs(innovation) <= INO_THRES && prNoiseTmp >= 100 && peMode.userSceData.isPDRMode == FALSE)
	{
		testFlag = FALSE;
	}


	test = udKFUpdate(H, deltaX, (double)prNoiseTmp, innovation,testThres, MEAS_PR_UPDATE, flag,testFlag);
	*updateFlag = KF_MEAS_UPDATE;
	if (testOut) 
	{
		if (innovation > 0) *testOut = test;
		else *testOut = -test;
	}


	/* PR de-weight use */
	if (testFlag)
	{   
		//if this sv is deweight before and test is still large, then not use it for KF update
		if(test > testThresRej && deWeightCnt > 0)
		{   
			SYS_LOGGING(OBJ_KF,LOG_INFO,"not use this PR as deweight and test larger than the bound");
			*updateFlag = KF_MEAS_REJECT; 
			return innovation;
		}
		while (test > testThres && test < testThresOut && updateCnt < 3)
		{
			SYS_LOGGING(OBJ_KF,LOG_INFO,"De-weight use PR");
			tmp = test / testThres;
			prNoiseTmp  = prNoiseTmp  * (float)tmp * (float)tmp * 4;
			test = udKFUpdate(H, deltaX, (double)prNoiseTmp , innovation,testThres, MEAS_PR_UPDATE, flag,testFlag);
			updateCnt++;
			*updateFlag = KF_MEAS_DEWEIGHT; 
		}

		if(test > testThres)
		{
			*updateFlag = KF_MEAS_REJECT; 
		}
	}

	*prNoisVar = prNoiseTmp;
	if (testOut) 
	{
		if (innovation > 0) *testOut = test;
		else *testOut = -test;
	}

	return innovation;
}

/**********************************************************************
* Function Name:    gnss_Kf_UpdateDisCalc
*
* Description:
*    calculate the distance between update values
*
* Input:
*     updateVal:  update values
*     num:        number update values
*
* Output:
*     distance:   distance between update values
*     minDis:     minimum distance of all distances
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_UpdateDisCalc(float updateVal[][3], float* distance, float* minDis, uint8_t num)
{
	uint8_t     i, j;
	float    diff[3], tmp;

	for (i = 0; i < num; i++)
	{
		for (j = 0; j < num; j++)
		{
			diff[0] = updateVal[i][0] - updateVal[j][0];
			diff[1] = updateVal[i][1] - updateVal[j][1];
			diff[2] = updateVal[i][2] - updateVal[j][2];
			distance[i] += (float)sqrt((double)diff[0] * diff[0] + (double)diff[1] * diff[1] + (double)diff[2] * diff[2]);
		}

		if (num > 1)
		{
			distance[i] /= (num - 1);
		}
	}

	tmp = distance[0];

	for (i = 1; i < num; i++)
	{
		if (distance[i] < tmp)
		{
			tmp = distance[i];
		}
	}

	*minDis = tmp;
}

/**********************************************************************
* Function Name:    gnssKfHeadingCheck
*
* Description:
*    To detect bad DR due to DR update heading
*
* Input:
*     measData:     measurement and SV related information
*     biasLinkFlag: clock bias flag
*                   0 -- 4bias;
*                   1 -- GPS/GLN link;
*                   2 -- GPS/GLN/BD link;
*                   3 -- GPS/GLN/BD/Galielo link
*     measIdx:     idx of the measurements for KF update
*     validNum:    number of valid measurements
*
*
*Output:
*     inovCheck:   0 -- bad PR measurement
*                  1 -- bad DR measurement
*
*
*
* Return:
*
*
* Dependency
*      None
*
* Author: Qiufang
**********************************************************************/
void gnss_Kf_HeadingCheck(Kf_t* p_Kf)
{
	uint8_t                 i, j, drNum = 0/*, drNumFlp = 0*/;
	uint8_t                 k = 0;
	uint8_t                 rejNum = 0,rejLmt = 2,updateFlag,flag;
	int8_t                 index[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM], indexRaw[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0}/*, indexFlp[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 }*/;
	int8_t                 index_temp;
	float                drNoiseVar;
	float                ecefVel[3];
	float                enuVel[3];
	float                sumSin = 0, sumCos = 0;
	float                avgHeading, deltaHeading_temp;
	float                heading[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0},distanceAbs[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0}, headingRaw[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 }/*, headingFlp[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 }*/;
	float                deltaHeading[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0};
	double                deltaX[N_STATE] = {0};
	float                median,rejectK = 10.0;
	//SV_INFO*           pSvInfo;
	KF_PVT_INFO*       kfPvtInfo;
	meas_blk_t*        pMeas;
	gnss_meas_t*       pSvMeas;
	float                THRES = (float)(PI / 4);
	//float                fuser_vel[3] = { 0 };
	USER_MOTION*       pUsrContext;
	STATIC_DATA*       kfStaticData;
	double				   user_vel[3] = { 0 };
	USER_PVT           user;


	pUsrContext = &(peMode.userContext);
	pMeas = p_Kf->meas_blk;
	kfPvtInfo = p_Kf->kf_Pvt_Info;
	kfStaticData = &(peMode.staticData);

	/*
	(1) measCnt < 7; (2) static = TRUE ; (3) velocity < 2.0
	*/
	if (pMeas->measCnt <= 6 || kfStaticData->staticFlag || kfPvtInfo->kfHeadingVel < 2.0)
	{
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Heading Check Condition Fail: %02d,%02d,%5.2f",pMeas->measCnt,p_Kf->kf_Static_Data.staticFlag,kfPvtInfo->kfHeadingVel);
		return;
	}

	if (peState.posHasBias)
	{
		return;
	}

	/* If DR number is less than 6, then we don't use heading check */
	gnss_Pe_PRDR_Num(pMeas);
	if (pMeas->validDrNum <= 6)
	{
		return;
	}
	memset(index,-1,sizeof(int8_t) * MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	gnss_Pe_Get_PosFix(&user);
	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];
		//pSvInfo = &pSvMeas->sv_info;

		//dr update
		memset(&deltaX, 0, N_STATE * sizeof(double));

		if (pSvMeas->status & 0x2) // DR valid
		{
			drNoiseVar = gnss_Kf_DrNoiseVarCal(pSvMeas,pMeas->avgCno);
			gnss_Kf_DrUpdate(pSvMeas, deltaX, (float)pSvMeas->pseudoRangeRate, &drNoiseVar,NULL, &updateFlag, 0,0);

			if (peMode.feedbackFlag.uVelVdrFeedback)
			{
				for (k = 0; k < 3; k++) user_vel[k] = (double)vdrFeedbackInfo.fvelEcef[k];
			}
			else
			{
				user_vel[0] = kfInfo.X[3];
				user_vel[1] = kfInfo.X[4];
				user_vel[2] = kfInfo.X[5];
			}
			for (j = 0; j < 3; j++)
			{
				ecefVel[j] = (float)(user_vel[j] + deltaX[j + 3]);
			}
			gnssConvEcef2EnuVel(ecefVel, enuVel, kfPvtInfo->kfLLApos);
			heading[drNum] = (float)atan2(enuVel[0], enuVel[1]);
			index[drNum] = i;
			drNum++;
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
			SYS_LOGGING(OBJ_KF, LOG_INFO, "Vel Heading Compare: %10.4f %02d %03d %02d %f %f", pMeas->tor, pSvMeas->gnssMode,
				pSvMeas->prn, pSvMeas->cno, (heading[drNum-1]*RAD2DEG), sqrt(deltaX[0] * deltaX[0] + deltaX[1] * deltaX[1] + deltaX[2] * deltaX[2]));
#endif
		}
	}
	memcpy(indexRaw, index, sizeof(index));
	memcpy(headingRaw, heading, sizeof(headingRaw));
	/* determine the DR reject Number */
	if (drNum >= 12)
	{
		rejLmt = 4;
	}else if (drNum >= 10)
	{
		rejLmt = 3;
	}
#if 1
	/* determine the threshold according to user context */
	if (kfPvtInfo->kfHeadingVel >= DRIVE_THRES)
	{
		THRES = (float)(PI / 20);          // 6 degree
	}else if (kfPvtInfo->kfHeadingVel >= RUN_THRES)
	{
		if (pUsrContext->context == USER_CONTEXT_HAND)
		{
			THRES = (float)(PI / 12);         // 15 degree
		}
		else
		{
			THRES = (float)(PI / 18);        // 7.5 degree
		}         
	}else if (kfPvtInfo->kfHeadingVel >= STATIC_THRES)
	{
		if (pUsrContext->context == USER_CONTEXT_HAND)
		{
			THRES = (float)(PI / 6);             // 30 degree
		}else
		{
			THRES = (float)(PI / 12);            // 15 degree
		}
	}else
	{
		if (pUsrContext->context == USER_CONTEXT_HAND)
		{
			THRES = (float)(PI / 6);             // 30 degree
		}else
		{
			THRES = (float)(PI / 12);            // 15 degree
		}
	}   
#else
	switch(pUsrContext->motion)
	{
	case USER_MOTION_STATIC:
		{
			if (pUsrContext->context == USER_CONTEXT_HAND)
			{
				THRES = (float)(PI / 6);             // 30 degree
			}else
			{
				THRES = (float)(PI / 12);            // 15 degree
			}
			break;
		}
	case USER_MOTION_WALK:
		{
			if (pUsrContext->context == USER_CONTEXT_HAND)
			{
				THRES = (float)(PI / 6);             // 30 degree
			}else
			{
				THRES = (float)(PI / 12);            // 15 degree
			}
			break;
		}
	case USER_MOTION_RUN:
		{
			if (pUsrContext->context == USER_CONTEXT_HAND)
			{
				THRES = (float)(PI / 12);         // 15 degree
			}
			else
			{
				THRES = (float)(PI / 24);        // 7.5 degree
			}         
			break;
		}
	case USER_MOTION_DRIVE:
		{
			THRES = (float)(PI / 30);          // 6 degree
			break;
		}
	default: break;
	}
#endif
	if (kfPvtInfo->kfHeadingVel >= 10.0)
	{
		rejectK = 10.0;
	}else
	{
		rejectK = 20.0;
	}
	
	//add process for 180 deg boundary
	flag = gnss_median(heading,drNum,&median);
	if (flag == FALSE) return ;
	for (i = 0; i < drNum;i++)
	{
		if(((double)heading[i] - median) > PI)
		{
			heading[i] -= (float)(2*PI);
		}
		else if(((double)heading[i] - median) < -PI)
		{
			heading[i] += (float)(2*PI);
		}
	}
	/* MAD remove very large deviation */
	gnss_MAD(heading,distanceAbs,drNum,&median);
	for (i = 0; i < drNum;i++)
	{
		if (index[i] < 0) continue;
		if(pMeas->meas[index[i]].quality & DR_GOOD) continue;

		if (distanceAbs[i] > (double)rejectK * median / 0.6745 && rejNum < rejLmt)
		{
			pMeas->meas[index[i]].status &= 0xFD;
			rejNum++;
#if defined(PLAYBACK_MODE)
			SYS_LOGGING(OBJ_KF,LOG_INFO,"reject DR by heading check MAD: tor = %16.8f, sv_id=%2d,mode=%3d,heading=%10.4f", pMeas->tor,
				pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode, heading[i]*RAD2DEG);
#endif
			index[i] = -1;
		}
	}

	/* average heading */
	for (i = 0; i < drNum; i++)
	{
		if (index[i] == -1) continue;
		sumSin += (float)sin(heading[i]);
		sumCos += (float)cos(heading[i]);
	}
	if (drNum > 0)
	{
		avgHeading = (float)atan2(sumSin, sumCos);
	}else
	{
		return;
	}


	/* calculate heading error */
	for (i = 0; i < drNum; i++)
	{
		deltaHeading[i] = heading[i] - avgHeading;

		if (deltaHeading[i] > PI)
		{
			deltaHeading[i] -= (float)PI2;
		}

		if (deltaHeading[i] < -PI)
		{
			deltaHeading[i] += (float)PI2;
		}

		deltaHeading[i] = (float)fabs(deltaHeading[i]);
	}

	/* sort deltaHeading */
	for (i = 0; i < drNum; i++)
	{
		for (j = i + 1; j < drNum; j++)
		{
			if (deltaHeading[i] < deltaHeading[j])
			{
				deltaHeading_temp = deltaHeading[i];
				index_temp = index[i];
				deltaHeading[i] = deltaHeading[j];
				index[i] = index[j];
				deltaHeading[j] = deltaHeading_temp;
				index[j] = index_temp;
			}
		}
	}

	//index_temp = -1;
	/* 1. large heading error detected */
	for (i = 0; i < drNum; i++)
	{
		if (index[i] < 0) continue;
		if(pMeas->meas[index[i]].quality & DR_GOOD) continue;

		if (deltaHeading[i] > THRES && rejNum < rejLmt)
		{
			//index_temp = i;
			pMeas->meas[index[i]].status &= 0xFD;
			rejNum++;
#if defined(PLAYBACK_MODE)
			SYS_LOGGING(OBJ_KF,LOG_INFO,"%s,reject DR by heading check: tor = %16.8f, sv_id=%2d,mode=%3d,avgheading=%10.4f,heading=%10.4f,DRDiff=%10.4f", __FUNCTION__,pMeas->tor,
				pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode, avgHeading * RAD2DEG, deltaHeading[i]*RAD2DEG,pMeas->meas[index[i]].dr_diff);
#endif
		}
	}

#if 0
	/* 2. If no large heading detected, do more check */
	if (rejNum < rejLmt && index_temp < (drNum - 2))
	{
		i = index_temp + 1;
		if (deltaHeading[i] / deltaHeading[i+1] > 2.0 && deltaHeading[i] * RAD2DEG >= 5
			&& index[i] != -1 && index[i+1] != -1)
		{
			pMeas->meas[index[i]].status &= 0xFD;
			rejNum++;
			SYS_LOGGING(OBJ_KF,LOG_INFO,"reject largest DR: tor = %16.8f, sv_id=%2d,mode=%3d,avgheading=%10.4f,heading=%10.4f", pMeas->tor,
				pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode, avgHeading * RAD2DEG, deltaHeading[i]*RAD2DEG);
		}      
	}   
#endif

#if 0
	// remove those rejected PR 
	for (i = 0; i < drNum; i++)
	{
		if ((pMeas->meas[index[i]].status & 0x2) == 0)
		{
			continue;
		}
		indexFlp[drNumFlp] = indexRaw[i];
		headingFlp[drNumFlp] = heading[i];
		drNumFlp++;
	}
	gnss_Kf_Vel_HeadingCheckFlp(drNumFlp, indexFlp, headingFlp, pMeas, THRES);
#endif
}

/**********************************************************************
* Function Name:    gnss_Kf_InovationCheck
*
* Description:
*    To detect bad PR and DR according to PR/DR upate
*
* Input:
*     measData:     measurement and SV related information
*     biasLinkFlag: clock bias flag
*                   0 -- 4bias;
*                   1 -- GPS/GLN link;
*                   2 -- GPS/GLN/BD link;
*                   3 -- GPS/GLN/BD/Galielo link
*     measIdx:     idx of the measurements for KF update
*     validNum:    number of valid measurements
*
*
*Output:
*     inovCheck:   0 -- bad PR measurement
*                  1 -- bad DR measurement
*
*
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_InovationCheck(Kf_t* p_Kf)
{
	uint32_t          i,j;
	uint8_t           idx, prNum = 0, prNum1 = 0, prNum2 = 0,drNum = 0 , updateFlag = TRUE;
	uint8_t           rejNum = 0,testCnt = 0,svNumThres;
	uint8_t           prRejLmt = 2,drRejLmt = 2;
	//uint8_t         index[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 };
	//uint8_t  		 index1[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 };
	//uint8_t         index2[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 };
	//uint8_t         indexTmp[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 };
	//uint8_t         indexFlp[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 };

	uint8_t           *index = (uint8_t*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	uint8_t           *index1 = (uint8_t*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	uint8_t           *index2 = (uint8_t*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	uint8_t           *indexTmp = (uint8_t*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	uint8_t           *indexFlp = (uint8_t*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	memset(index, 		0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	memset(index1, 		0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	memset(index2, 		0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	memset(indexTmp, 	0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	memset(indexFlp, 	0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
	
	//uint8_t           status;
	double          heading;
	//float        llaHeading[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0};
	//float        llaHeadingFlp[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = { 0 };
	float          *llaHeading = (float*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(float));
	float          *llaHeadingFlp = (float*)Sys_Malloc(MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(float));
	memset(llaHeading, 		0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(float));
	memset(llaHeadingFlp, 	0, 	MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(float));
	
	float          rejK = 30.0,factor = 2.0,testFactor1,testFactor2,ratioThres;
	float          median;
	float          prDisMean,prDisStd,drDisMean,drDisStd;
	float          prThres1,prThres2,/*drThres1,*/drThres2;
	float          velocity = 0;
	float          min_dis;
	float          prNoiseVar = 0, drNoiseVar;
	double          deltaX[N_STATE] = {0};
	float          distance[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0};
	float          distanceAbs[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0};
	float          updateVal[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM][3] = {{0}};
	double          updateLla[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM][3] = {{0}};
	double          updateLla1[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM][3] = {{0}};
	double          updateLla2[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM][3] = {{0}};
	double          xyz[3] = {0},lla[3] = {0};
	double          test;
	float          deltaAlt[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0};
	//SV_INFO*     pSvInfo;
	gnss_meas_t* pSvMeas;
	meas_blk_t*  pMeas;
	Ls_t*        p_Ls;
	KF_PVT_INFO* kfPvtInfo;
	LS_PVT_INFO* lsPvtInfo;
	double          PR;
#if defined(PLAYBACK_MODE)
	GNSS_TIME*   pTime;
#endif
	//float			 fuser_vel[3] = { 0 };
	//double          user_vel[3] = { 0 };

	pMeas = p_Kf->meas_blk;
	p_Ls = p_Kf->lsBlk;
	kfPvtInfo = p_Kf->kf_Pvt_Info;
	lsPvtInfo = p_Kf->ls_Pvt_Info;

#if defined(PLAYBACK_MODE)
	pTime = gnss_tm_get_time();
#endif

	if(g_pe_cfg.chipType == UBLOX)
	{   
		if (peMode.userSceData.isUnderEleRoad && pMeas->measCnt < 8)
		{
			svNumThres = 4;
			prRejLmt = 1;
			drRejLmt = 1;
		}
		else
		{
			svNumThres = 5;
		}
	}
	else
	{
		svNumThres = 6;
	}

	/* If total measurement count is less than 6, then don't do innovation check */
	if (pMeas->measCnt <= svNumThres)
	{
		goto GNSS_KF_INNO_CHECK_RETURN;
	}

	/* check valid PR number and DR number */
	gnss_Pe_PRDR_Num(pMeas);
	if (pMeas->validPrNum <= svNumThres|| p_Kf->state == KF_STATE_POSBIAS || peState.posHasBias)
	{
		goto DR_INNO_CHECK;
	}
	if (lsPvtInfo->pos_res < 4 && g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		goto DR_INNO_CHECK;
	}
	else if (lsPvtInfo->pos_res < 4.0 && lsPvtInfo->pos_res > 1e-3 && pMeas->measCnt > 24 && pMeas->avgCno > 32)
	{
		goto DR_INNO_CHECK;
	}

	velocity = (float)gnssClcSqrtSum_DBL(&kfInfo.X[3], 3);

	/* PR update check */
	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];
		//pSvInfo = &pSvMeas->sv_info;

		/* get the idx of clock bias by satMode and clockBiasFlag */
		idx = clockBiasIdx[pSvMeas->gnssMode][p_Kf->kf_ctrl.biasLinkFlag];

		//pr update
		memset(&deltaX, 0, N_STATE * sizeof(double));

		if (pSvMeas->status & 0x1) // PR valid
		{
			prNoiseVar = gnss_Kf_PrNoiseVarCal(pSvMeas,pMeas->avgCno,1);
			PR = pSvMeas->pseudoRange;
			gnss_Kf_PR_Adj(p_Kf,pSvMeas->gnssMode,&PR);
			//gnss_Kf_PrUpdate(pSvInfo->dcos, deltaX, pSvMeas->range, PR, &prNoiseVar, pSvMeas->prChckCnt, &updateFlag, 0, idx);
			gnss_Kf_PrUpdate(pSvMeas, deltaX, PR, &prNoiseVar, &updateFlag,&test, 0,0, idx);

			for (j = 0; j < 3; j++)
			{
				updateVal[prNum][j] = (float)deltaX[j];
				xyz[j] = kfInfo.X[j] + deltaX[j];
			}
			gnssConvEcef2Lla(xyz,lla);            
			for (j = 0; j < 3; j++)
			{
				updateLla[prNum][j] = lla[j];
			}

			index[prNum++] = i;

			if (test > SBOUND_PR)
			{
				testCnt++;
			}
		}
	}

	SYS_LOGGING(OBJ_KF,LOG_INFO,"PR Test Fail in KF inno:%14.6f,%02d,%02d",pMeas->tor,testCnt,prNum);


	/* bad PR detection */
	memcpy(indexTmp,index,MAX_MEAS_NUM *AGGNSS_MAX_FREQ_NUM* sizeof(uint8_t));
	memset(distance,0,MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM*sizeof(float));
	gnss_Kf_UpdateDisCalc(updateVal, distance, &min_dis, prNum);
	gnss_math_fstd(distance,prNum,&prDisMean,&prDisStd);
	gnss_Sort_WithIndx(distance,index,prNum);

	/* MAD */
	gnss_MAD(distance,distanceAbs,prNum,&median);
	if (peMode.userContext.motion == USER_MOTION_STATIC || peMode.userContext.motion == USER_MOTION_WALK)
	{
		prThres1 = 2.5;

		if(peMode.userSceData.isPDRMode)
		{
			prThres2 = 1.0;
			ratioThres = 5.0;
		}
		else if(g_pe_cfg.chipType == SPRD || g_pe_cfg.chipType == UBLOX)
		{
			prThres2 = 1.0;
			ratioThres = 8.0;
		}
		else
		{
			prThres2 = prThres1;
			ratioThres = 8.0;
		}
	}else
	{
		prThres1 = 1.0;
		prThres2 = prThres1;
		ratioThres = 8.0;
	}
	/* Enlarge threshold when position is biased */
	if (peState.posHasBias)
	{
		prThres1 *= 10.0;
		prThres2 *= 10.0;
	}
	/*1. PR distance reject strategy */
	for (i = prNum - 1; i > 0; i--)
	{
		if (pMeas->meas[index[i]].quality & PR_GOOD)
		{
			continue;
		}
		if ((min_dis < 15 || (distance[i] / min_dis) > 3) && rejNum < prRejLmt)
		{
			if (((double)distance[i] - min_dis) > 30.0 || (distance[i] / min_dis > 2 && (distance[i] + min_dis - velocity) > 9))
			{
				pMeas->meas[index[i]].status &= 0xFE;
				rejNum++;
#if defined(PLAYBACK_MODE)
				SYS_LOGGING(OBJ_KF,LOG_INFO,"reject PR by PR update1: tor = %16.8f,tow = %16.8f,sv_id=%02d,mode=%3d,cno=%02d,mis=%10.4f,dis=%10.4f,stdDiff=%10.4f,diff=%12.4f", pMeas->tor,pTime->rcvr_time[GPS_MODE],
					pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode,pMeas->meas[index[i]].cno, min_dis, distance[i],pMeas->meas[index[i]].stdPRDRDiff,pMeas->meas[index[i]].pr_diff);
#endif
			}
			else if (distance[i] / min_dis > 2 && (distance[i] + min_dis) > 4 * velocity && velocity > 0 && distance[i] > 10)
			{
				pMeas->meas[index[i]].status &= 0xFE;
				rejNum++;
#if defined(PLAYBACK_MODE)
				SYS_LOGGING(OBJ_KF,LOG_INFO,"reject PR by PR update2: tor = %16.8f,tow = %16.8f,sv_id=%02d,mode=%3d,cno=%02d,mis=%10.4f,dis=%10.4f,stdDiff=%10.4f,diff=%12.4f", pMeas->tor,pTime->rcvr_time[GPS_MODE],
					pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode,pMeas->meas[index[i]].cno, min_dis, distance[i],pMeas->meas[index[i]].stdPRDRDiff,pMeas->meas[index[i]].pr_diff);
#endif
			}else
			{
				if (distanceAbs[i] > rejK * (median / 0.6745) && distance[i] > prThres1 && (pMeas->meas[index[i]].quality & PR_GOOD)==0)
				{
					pMeas->meas[index[i]].status &= 0xFE;
					rejNum++;
#if defined(PLAYBACK_MODE)
					SYS_LOGGING(OBJ_KF,LOG_INFO,"reject PR by PR update3: tor = %16.8f,tow = %16.8f,sv_id=%02d,mode=%3d,cno=%02d,mis=%10.4f,dis=%10.4f,stdDiff=%10.4f,diff=%12.4f", pMeas->tor,pTime->rcvr_time[GPS_MODE],
						pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode,pMeas->meas[index[i]].cno, min_dis, distance[i],pMeas->meas[index[i]].stdPRDRDiff,pMeas->meas[index[i]].pr_diff);
#endif
				}
			}
		}
	}
	/* 2. If didn't reject any PR by large distance check, then reject the largest outliers */
	i = prNum - 1;
	if (rejNum < 1 && prNum >= 8)
	{
		prDisMean = (prDisMean * prNum - distance[i] - distance[i-1]) / (prNum - 2);
		testFactor1 = (distance[i] - prDisMean) / (distance[i-2] - prDisMean);
		if (((testFactor1 > 2.0 && distance[i] >= prThres1)||(testFactor1 >ratioThres && distance[i] >= prThres2) )&& 
			(pMeas->meas[index[i]].quality & PR_GOOD)==0 && fabs(pMeas->meas[index[i]].pr_diff)>5.0)
		{
			if ((gnss_Pe_Dop_Check(pMeas,index[i],1) == TRUE) || testFactor1 > 5.0)
			{
				pMeas->meas[index[i]].status &= 0xFE;
				rejNum++;
#if defined(PLAYBACK_MODE)
				SYS_LOGGING(OBJ_KF,LOG_INFO,"reject max PR: tor = %16.8f,tow = %16.8f,sv_id=%02d,mode=%3d,cno=%02d,max=%10.4f,secMax=%10.4f,stdDiff=%10.4f,diff=%12.4f", pMeas->tor,pTime->rcvr_time[GPS_MODE],
					pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode,pMeas->meas[index[i]].cno, distance[i],distance[i-1],pMeas->meas[index[i]].stdPRDRDiff,pMeas->meas[index[i]].pr_diff);
#endif
			}
		}
		i = prNum - 2;
		testFactor1 = (distance[i] - prDisMean) / (distance[i-1] - prDisMean);
		if (((testFactor1 > 2.0 && distance[i] >= prThres1)||(testFactor1 > ratioThres && distance[i] >= prThres2))&& 
			(pMeas->meas[index[i]].quality & PR_GOOD) ==0 && fabs(pMeas->meas[index[i]].pr_diff)>5.0)
		{
			if ((gnss_Pe_Dop_Check(pMeas,index[i],1) == TRUE) || testFactor1 > 5.0)
			{
				pMeas->meas[index[i]].status &= 0xFE;
				rejNum++;
#if defined(PLAYBACK_MODE)
				SYS_LOGGING(OBJ_KF,LOG_INFO,"reject max PR: tor = %16.8f,tow = %16.8f,sv_id=%02d,mode=%3d,cno=%02d,max=%10.4f,secMax=%10.4f,stdDiff=%10.4f,diff=%12.4f", pMeas->tor,pTime->rcvr_time[GPS_MODE],
					pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode,pMeas->meas[index[i]].cno, distance[i],distance[i-1],pMeas->meas[index[i]].stdPRDRDiff,pMeas->meas[index[i]].pr_diff);
#endif
			}
		}
	}
	else if(rejNum == 1 && (prNum - rejNum)>= 8)
	{
		i = prNum - 2; 
		prDisMean = (prDisMean * prNum - distance[i+1] - distance[i]) / (prNum - 2);
		testFactor1 = (distance[i] - prDisMean) / (distance[i-1] - prDisMean);
		if (((testFactor1 > 2.0 && distance[i] >= prThres1)||(testFactor1 > 8.0 && distance[i] >= prThres2) )&& 
			(pMeas->meas[index[i]].quality & PR_GOOD)==0 && fabs(pMeas->meas[index[i]].pr_diff)>5.0)
		{
			if ((gnss_Pe_Dop_Check(pMeas,index[i],1) == TRUE) || testFactor1 > 5.0)
			{
				pMeas->meas[index[i]].status &= 0xFE;
				rejNum++;
#if defined(PLAYBACK_MODE)
				SYS_LOGGING(OBJ_KF,LOG_INFO,"reject max PR: tor = %16.8f,tow = %16.8f,sv_id=%02d,mode=%3d,cno=%02d,max=%10.4f,secMax=%10.4f,stdDiff=%10.4f,diff=%12.4f", pMeas->tor,pTime->rcvr_time[GPS_MODE],
					pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode,pMeas->meas[index[i]].cno, distance[i],distance[i-1],pMeas->meas[index[i]].stdPRDRDiff,pMeas->meas[index[i]].pr_diff);
#endif
			}
		}
	}
	GLOGD("rej num:%d", rejNum);

	// remove those rejected PR 
	for (i = 0;i < prNum; i++)
	{
		if ((pMeas->meas[indexTmp[i]].status & 0x1) == 0)
		{
			continue;
		}
		index1[prNum1] = indexTmp[i];
		updateLla1[prNum1][0] = updateLla[i][0];
		updateLla1[prNum1][1] = updateLla[i][1];
		updateLla1[prNum1++][2] = updateLla[i][2];

	}
	if (kfPvtInfo->kfHeadingVel >= 0.5)
	{
		for (i = 0; i < prNum1;i++)
		{
			heading = gnss_lla2_heading(&(kfPvtInfo->kfLLApos[0]),&updateLla1[i][0]);
			if (heading > PI) heading -= 2 * PI;
			llaHeading[i] = (float)heading;
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
			SYS_LOGGING(OBJ_KF,LOG_INFO,"Heading Compare: %10.4f %2d %3d %f",pMeas->tor,pMeas->meas[index1[i]].gnssMode,
				pMeas->meas[index1[i]].prn,(heading)*RAD2DEG);
#endif
		}
		memcpy(llaHeadingFlp, llaHeading, MAX_MEAS_NUM* AGGNSS_MAX_FREQ_NUM * sizeof(float));
		memcpy(indexFlp, index1, MAX_MEAS_NUM* AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
		gnss_Kf_Lla_HeadingCheck(prNum1, index1, llaHeading, pMeas);
#if 0
		gnss_Kf_Lla_HeadingCheckFlp(prNum1, indexFlp, llaHeadingFlp, pMeas);
#endif // 0

	}
#ifdef ALT_CHECK_INNO

	// remove those rejected PR 
	for (i = 0;i < prNum; i++)
	{
		if ((pMeas->meas[indexTmp[i]].status & 0x1) == 0)
		{
			continue;
		}
		index2[prNum2] = indexTmp[i];
		updateLla2[prNum2][0] = updateLla[i][0];
		updateLla2[prNum2][1] = updateLla[i][1];
		updateLla2[prNum2++][2] = updateLla[i][2];

	}

	for (i = 0; i < prNum2; i++)
	{
		for (j = 0;j < prNum2;j++)
		{
			if (i == j) continue;
			deltaAlt[i] += (float)fabs(updateLla2[i][2] - updateLla2[j][2]);
		}
		if (prNum2 - 1 != 0) {
			deltaAlt[i] /= (prNum2 - 1);
		}
		
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
		SYS_LOGGING(OBJ_KF,LOG_INFO,"DeltaAlt:%02d,%03d,%10.6f",
			pMeas->meas[index2[i]].gnssMode,pMeas->meas[index2[i]].prn,deltaAlt[i]);
#endif
	}
	gnss_Kf_Alt_Check(&deltaAlt[0],&index2[0],prNum2,p_Kf);
#endif

DR_INNO_CHECK:
	if (pMeas->validDrNum <= svNumThres)
	{
		goto GNSS_KF_INNO_CHECK_RETURN;
	}
	if (lsPvtInfo->vel_res < 0.2 && g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		goto GNSS_KF_INNO_CHECK_RETURN;
	}
	else if (lsPvtInfo->vel_res < 0.2 && lsPvtInfo->vel_res > 1e-3 && pMeas->measCnt > 24 && pMeas->avgCno > 32)
	{
		goto GNSS_KF_INNO_CHECK_RETURN;
	}

	/* DR update check */
	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];
		//pSvInfo = &pSvMeas->sv_info;

		//DR update
		memset(&deltaX, 0, N_STATE * sizeof(double));

		if (pSvMeas->status & 0x2) // DR valid
		{
			drNoiseVar = gnss_Kf_DrNoiseVarCal(pSvMeas,pMeas->avgCno);
			gnss_Kf_DrUpdate(pSvMeas, deltaX, (float)pSvMeas->pseudoRangeRate, &drNoiseVar,NULL, &updateFlag, 0,0);

			for (j = 0; j < 3; j++)
			{
				updateVal[drNum][j] = (float)deltaX[j + 3];
			}

			index[drNum++] = i;
		}
	}

	/* bad DR detect */
	memset(distance,0,MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM*sizeof(float));
	gnss_Kf_UpdateDisCalc(updateVal, distance, &min_dis, drNum);
	gnss_math_fstd(distance,drNum,&drDisMean,&drDisStd);
	gnss_Sort_WithIndx(distance,index,drNum);
	rejNum = 0;

	/* 3. DR distance reject strategy */
	gnss_Kf_DR_ErrNum(drNum, &drRejLmt);
	if (drRejLmt > 0)
	{
		for (i = drNum - 1; i > 0; i--)
		{
			if(pMeas->meas[index[i]].quality & DR_GOOD)
			{
				continue;
			}
			if ((p_Ls->raimCtrl.drRaimFlag & LS_RAIM_FOUND_ERROR) == 0)  //need more work here
			{
				if ((distance[i] - min_dis) > 2 && rejNum < drRejLmt)
				{
					pMeas->meas[index[i]].status &= 0xFD;
					rejNum++;
#if defined(PLAYBACK_MODE)
					SYS_LOGGING(OBJ_KF,LOG_INFO,"reject DR by DR update: tor = %16.8f, sv_id=%2d,mode=%3d,mis=%10.4f,dis=%10.4f", pMeas->tor,
						pMeas->meas[index[i]].prn,pMeas->meas[index[i]].gnssMode,  min_dis, distance[i]);
#endif
				}
			}
			else
			{
				if ((distance[i] - min_dis) > 3 && (min_dis < 3 || (distance[i] / min_dis) > 2) && rejNum < drRejLmt)
				{
					pMeas->meas[index[i]].status &= 0xFD;
					rejNum++;
#if defined(PLAYBACK_MODE)
					SYS_LOGGING(OBJ_KF,LOG_INFO,"reject DR by DR update: tor = %16.8f, sv_id=%2d,mode=%3d,mis=%10.4f,dis=%10.4f", pMeas->tor,
						pMeas->meas[index[i]].prn,pMeas->meas[index[i]].gnssMode, min_dis, distance[i]);
#endif
				}
			}
		}
	}


	/* 4. If DR RAIM can't pass but didn't reject any satellites */
	//drThres1 = (float)0.20;
	drThres2 = (float)0.10;

	if (p_Ls->lsCtrl.isDrRaim == TRUE || drNum >= 6)
	{
		if (rejNum == 0 && drRejLmt > 0)
		{
			drDisMean = 0;
			for (i = 0; i < (uint32_t)drNum - 2; i++)
			{
				drDisMean += distance[i];
			}
			drDisMean /= (drNum - 2);

			if(drDisMean < 0.1)
			{
				factor = 3.0f;
			}
			else if(drDisMean <= 0.20)
			{
				factor = 2.0f;
			}
			else 
			{
				factor = 1.6f;
			}

			i = drNum - 1;
			testFactor1 = (distance[i] - drDisMean) / (distance[i-2] - drDisMean);
			testFactor2 = distance[i] / drDisMean;
			if (testFactor1 > 1.8 && (distance[i] > drThres2)&&((testFactor2 > factor)||(((double)distance[i]-drDisMean)> 0.5)) &&
				(pMeas->meas[index[i]].quality & DR_GOOD) == 0)
			{
				if (pMeas->meas[index[i]].cno > (pMeas->avgCno + 15) && testFactor1 < 4.0 && (testFactor2 < 2.5 || ((double)distance[i] - drDisMean) < 0.5))
				{
					//Do not reject DR sat
				}
				else if (pMeas->meas[index[i]].cno > (pMeas->avgCno + 10) && testFactor1 < 3.0 && testFactor2 < 3.0 && ((double)distance[i] - drDisMean) < 0.5)
				{
					//Do not reject DR sat
				}
				else
				{
					pMeas->meas[index[i]].status &= 0xFD;
					rejNum++;
				}
			}
			i = drNum - 2;
			testFactor1 = (distance[i] - drDisMean) / (distance[i-1] - drDisMean);
			testFactor2 = distance[i] / drDisMean;
			if (testFactor1 > 1.8 && (distance[i] > drThres2)&&((testFactor2 > factor)||(((double)distance[i]-drDisMean)> 0.5)) &&
				(pMeas->meas[index[i]].quality & DR_GOOD) == 0)
			{
				if (pMeas->meas[index[i]].cno > (pMeas->avgCno + 15) && testFactor1 < 4.0 && (testFactor2 < 2.5 || ((double)distance[i] - drDisMean) < 0.5))
				{
					//Do not reject DR sat
				}
				else if (pMeas->meas[index[i]].cno > (pMeas->avgCno + 10) && testFactor1 < 3.0 && testFactor2 < 3.0 && ((double)distance[i] - drDisMean) <= 0.5)
				{
					//Do not reject DR sat
				}
				else
				{
					pMeas->meas[index[i]].status &= 0xFD;
					rejNum++;
				}
			}
		}
	}
	GLOGD("rej num:%d", rejNum);

#if 0
	// If DR update distance can't find any outliers, then do further check
	i = drNum - 1;
	if (rejNum < 1 && drNum >= 5)
	{
		drDisMean = (drDisMean * drNum - distance[i]) / (drNum - 1);
		if ((distance[i] - drDisMean) / (distance[i-1] - drDisMean) > 2.0
			&& distance[i] >= 1.0)
		{
			pMeas->meas[index[i]].status &= 0xFD;
			rejNum++;
			SYS_LOGGING(OBJ_KF,LOG_INFO,"reject max DR: tor = %16.8f, sv_id=%2d,mode=%3d,max=%f,secMax=%f", pMeas->tor,
				pMeas->meas[index[i]].prn, pMeas->meas[index[i]].gnssMode, distance[i],distance[i-1]);
		}
	}
#endif

GNSS_KF_INNO_CHECK_RETURN:

	if (index != NULL) {
		Sys_Free(index);
	}

	
	if (index1 != NULL) {
		Sys_Free(index1);
	}

	
	if (index2 != NULL) {
		Sys_Free(index2);
	}

	
	if (indexTmp != NULL) {
		Sys_Free(indexTmp);
	}

	
	if (indexFlp != NULL) {
		Sys_Free(indexFlp);
	}

	if (llaHeading != NULL) {
		Sys_Free(llaHeading);
	}

	if (llaHeadingFlp != NULL) {
		Sys_Free(llaHeadingFlp);
	}	
	
	return;
}
void gnss_Kf_PosBias_check(Kf_t* p_Kf,uint8_t measNum ,uint8_t rejNum, uint8_t dewNum)
{ 
	uint8_t    measBadNum = 0;
	//uint8_t    posBiasFlag = FALSE;

	measBadNum = rejNum + dewNum;

	if(measBadNum >= measNum/2)
	{
		//posBiasFlag = TRUE;
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Detect pos bias as too many meas reject/deweight in KF update");
	}

	/*if(posBiasFlag)
	{
	p_Kf->enlargeQ = TRUE;
	p_Kf->QFactor = 100;
	}*/
}

/***********************************************************************
* 函数介绍: gnss_Kf_Update
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/12/
***********************************************************************/
void gnss_Kf_Update(Kf_t *p_Kf,double* deltaX)
{
	uint8_t                     idx,svNumUsed,svModeUsed,biasNum,updateFlag;
	uint8_t                     prMeasNum = 0,prRejNum = 0,prDewNum = 0,prMeasNum_flp = 0;
	uint8_t                     drMeasNum = 0,drRejNum = 0,drDewNum = 0;
	uint32_t                    i;
	float                    prNoiseVar = 500;//codePhStd
	float                    drNoiseVar; //dopplerStd;
	double                    innovation,test;
	meas_blk_t*            pMeas;
	gnss_meas_t*           pSvMeas;
	SV_INFO*               pSvInfo;
	double                    PR;
	if (p_Kf == NULL)
	{
		SYS_LOGGING(OBJ_KF,LOG_ERROR,"Empty KF Handler(%s)",__FUNCTION__);
		return;
	}

	pMeas = p_Kf->meas_blk;

	if (pMeas == NULL)
	{
		return;
	}

	if ((kfInfo.status & KF_RUN) == 0)
	{
		return;
	}

	// update check after 10 time KF normal update
	if (p_Kf->kfCnt > 10)
	{
		/* KF innovation check */
		gnss_Kf_InovationCheck(p_Kf);
		/* KF heading check for DR */
		gnss_Kf_HeadingCheck(p_Kf);
	}


	svModeUsed = 0;
	svNumUsed = 0;

	for (i = 0; i < pMeas->measCnt;i++)
	{
		pSvMeas = &(pMeas->meas[i]);
		pSvInfo = &(pSvMeas->sv_info);
		/* DR update */
		if (pSvMeas->status & 0x2)
		{
			drNoiseVar = gnss_Kf_DrNoiseVarCal(pSvMeas,pMeas->avgCno);
			updateFlag  = KF_MEAS_UPDATE;
			innovation = gnss_Kf_DrUpdate(pSvMeas, deltaX, (float)pSvMeas->pseudoRangeRate, &drNoiseVar, &test,&updateFlag, 1,pSvMeas->isDrChiSqTest);

			SYS_LOGGING(OBJ_KF,LOG_DEBUG,"DR :%14.6f,%02d, %03d, %10.5f, %14.4f, %10.4f, %2d", pMeas->tor,
				pSvMeas->gnssMode,pSvMeas->prn,innovation,drNoiseVar,test,updateFlag);

			pSvMeas->drNoise = drNoiseVar;
			drMeasNum++;

			if(!updateFlag)
			{
				pSvMeas->status &=0xFD;
			}

			if(updateFlag == KF_MEAS_REJECT)
			{
				drRejNum++;
			}
			else if(updateFlag == KF_MEAS_DEWEIGHT)
			{
				drDewNum++;
			}
		}
	}

	for (i = 0; i < pMeas->measCnt;i++)
	{
		pSvMeas = &(pMeas->meas[i]);
		pSvInfo = &(pSvMeas->sv_info);

		/* PR update */
		if (pSvMeas->status & 0x1)
		{
			idx = clockBiasIdx[pSvMeas->gnssMode][p_Kf->kf_ctrl.biasLinkFlag];
			prNoiseVar = gnss_Kf_PrNoiseVarCal(pSvMeas,pMeas->avgCno,1);
			updateFlag  = KF_MEAS_UPDATE;
			PR = pSvMeas->pseudoRange;
			gnss_Kf_PR_Adj(p_Kf,pSvMeas->gnssMode,&PR);
			innovation = gnss_Kf_PrUpdate(pSvMeas,deltaX, PR, &prNoiseVar, &updateFlag, &test,1,pSvMeas->isPrChiSqTest,idx);
			pSvMeas->prNoise = prNoiseVar;

			prMeasNum_flp++;
			prMeasNum ++;

			if(updateFlag)
			{
				svNumUsed++;
			}
			else
			{
				pSvMeas->status &=0xFE;
			}
			if(updateFlag == KF_MEAS_REJECT)
			{
				prRejNum++;
			}
			else if(updateFlag == KF_MEAS_DEWEIGHT || pSvMeas->prChckCnt > 0)
			{
				prDewNum++;
			}

			SYS_LOGGING(OBJ_KF,LOG_DEBUG,"PR :%14.6f,%02d, %03d, %10.5f, %14.4f, %10.4f, %2d,%10.4f,%10.4f", pMeas->tor,
				pSvMeas->gnssMode,pSvMeas->prn,innovation,prNoiseVar,fabs(test),updateFlag,pSvInfo->fltAz,pSvInfo->fltElev);
		}
#if 0
		/* DR update */
		if (pSvMeas->status & 0x2)
		{
			drNoiseVar = gnss_Kf_DrNoiseVarCal(pSvMeas,pMeas->avgCno);
			innovation = gnss_Kf_DrUpdate(pSvMeas, deltaX, (float)pSvMeas->pseudoRangeRate, &drNoiseVar, &updateFlag, 1,1);
			SYS_LOGGING(OBJ_KF,LOG_INFO,"%s DR :%14.6f,%02d, %03d, %10.5f, %14.4f",__FUNCTION__, pMeas->tor,
				pSvMeas->gnssMode,pSvMeas->prn,innovation,drNoiseVar);
			svNumUsed++;
		}
#endif
		/* If PR and DR are both used, then set 'svModeUsed' */
		if ((pSvMeas->status & 0x3) == 0x3)
		{
			svModeUsed |= (1<<pSvMeas->gnssMode);
		}
	}

	// KF count 
	if (pMeas->measCnt > 0)
	{
		p_Kf->kfCnt++;
	}

	/* Calculate biasNum */
	biasNum = gnss_Kf_Get_BiasNum(svModeUsed,p_Kf->kf_ctrl.biasLinkFlag);

	/* Calculate the freeNum */
	if (svNumUsed > biasNum)
	{
		p_Kf->kf_ctrl.freeNum = svNumUsed - (biasNum - 1);
	}else
	{
		p_Kf->kf_ctrl.freeNum = 0;
	}

	/* save PR and DR update status */
	p_Kf->kf_ctrl.prRejNum = prRejNum;
	p_Kf->kf_ctrl.prDewNum = prDewNum;
	p_Kf->kf_ctrl.drRejNum = drRejNum;
	p_Kf->kf_ctrl.drDewNum = drDewNum;
	p_Kf->kf_ctrl.prNum = prMeasNum;
	p_Kf->kf_ctrl.prNumFLP = prMeasNum_flp;
	p_Kf->kf_ctrl.drNum = drMeasNum;

	SYS_LOGGING(OBJ_KF,LOG_INFO,"prNum:%02d,prRejNum:%02d,prDewNum:%02d,drNum:%02d,drRejNum:%02d,drDewNum:%02d",
		prMeasNum,prRejNum,prDewNum,drMeasNum,drRejNum,drDewNum);
}
/***********************************************************************
* 函数介绍: gnss_Kf_Restart_Check
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：06/26/
***********************************************************************/
uint8_t gnss_Kf_Restart_Check(Kf_t* p_Kf, double  dt1, double dt2)
{
	uint8_t             i;
	uint8_t             flag = TRUE;
	float            delta_heading, delta_vel, ls_vel, last_vel;
	KF_PVT_INFO*   kfPvtInfo;
	LS_PVT_INFO*   lsPvtInfo;
	Ls_t*          p_Ls;

	kfPvtInfo = p_Kf->kf_Pvt_Info;
	lsPvtInfo = p_Kf->ls_Pvt_Info;
	p_Ls = p_Kf->lsBlk;

	if(!(kfPvtInfo->kf_had_firstFix))
	{
		return FALSE;
	}

	for(i = 0; i< 3; i++)
	{
		if(fabs(kfPvtInfo->ecefPos_last[i]) < 1e-4)
		{
			return FALSE;
		}
	}

	if(lsPvtInfo->valid && p_Ls->lsCtrl.drNum > 6 && p_Ls->vel_res < 1.0 && (fabs(dt1) >= 5||fabs(dt2) >= 5))
	{
		delta_heading = (float)((lsPvtInfo->heading - kfPvtInfo->kfHeadingDir) * RAD2DEG);
		if(delta_heading > 180)
		{
			delta_heading -= 360;
		}
		else if(delta_heading < -180)
		{
			delta_heading += 360;
		}

		ls_vel = (float)sqrt((double)lsPvtInfo->enuVel[0]*lsPvtInfo->enuVel[0] + (double)lsPvtInfo->enuVel[1]*lsPvtInfo->enuVel[1]);
		last_vel = (float)sqrt((double)kfPvtInfo->enuVel_last[0]*kfPvtInfo->enuVel_last[0] + (double)kfPvtInfo->enuVel_last[1]*kfPvtInfo->enuVel_last[1]);
		delta_vel = ls_vel - last_vel;

		if((fabs(delta_heading) > 15 && ls_vel > 2.0 && last_vel > 2.0) || fabs(delta_vel) > 3.0)
		{
			flag = FALSE;
		}
	}

	return flag;
}
/**********************************************************************
* Function Name:    gnss_Kf_RunCheck
*
* Description:
*    Check fix mode and min CN0 for KF running
*
* Input:
*     measData:     measurement and SV related information
*     measIdx:      idx of the measurements after sort
*     validNum:     number of valid measurements
*     biasLinkFlag: clock bias flag
*                   0 -- 4bias;
*                   1 -- GPS/GLN link;
*                   2 -- GPS/GLN/BD link;
*                   3 -- GPS/GLN/BD/Galielo link
*
* Return:
*    kfPvtInfo->kfFixMode:      NO_SOLN -- KF will stop run
*                               original value: FULL_FIX or COAST mode or NO_SOLN
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_RunCheck(Kf_t* p_Kf,double dt)
{
	uint8_t                  i, noUpdate = 0;
	uint8_t                  svModeValid, svPrNumValid, svDrNumValid,biasNum;
	uint16_t                 maxCno = 0;
	meas_blk_t*         pMeas;
	gnss_meas_t*        pSvMeas;
	KF_PVT_INFO*        kfPvtInfo;
	STATIC_DATA*        pStaticData;

	pMeas = p_Kf->meas_blk;
	kfPvtInfo = p_Kf->kf_Pvt_Info;

	pStaticData =&(peMode.staticData);

	/* If KF is in INIT or RESET status, just return */
	if ((kfInfo.status & KF_INIT) || (kfInfo.status & KF_RESET))
	{
		return;
	}

	/* If no measurements or tunnel mode, If in coast mode,then propagate KF */
	if (pMeas->measCnt == 0 || peMode.tunnelMode.mode)
	{
		if (kfPvtInfo->kfFixMode >= EXTRAPOLATE)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
			return;
		}
	}

	svModeValid = 0;
	svPrNumValid = 0;
	svDrNumValid = 0;
	maxCno = pMeas->maxCno;

	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];
		if(pSvMeas->freq_index > 0) continue;

		if (pSvMeas->status & 0x1)
		{
			svPrNumValid++;
			svModeValid |= (1 << pSvMeas->gnssMode);
		}
		if (pSvMeas->status & 0x2)
		{
			svDrNumValid++;
		}	
	}

	/* calculate the valid SV number */
	biasNum = gnss_Kf_Get_BiasNum(svModeValid,p_Kf->kf_ctrl.biasLinkFlag);

	if (svPrNumValid > biasNum)
	{
		svPrNumValid -= biasNum;
	}
	else
	{
		svPrNumValid = 0;
		if (kfPvtInfo->kfFixMode > EXTRAPOLATE)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
			return;
		}
	}
	if(svDrNumValid > 0 )
	{
		svDrNumValid -=1;
	}
#if 0
	/*
	KF No update Logic
	1. If full rank and maximum cno < 18, then no update
	2. If only two valid meas and maximum cno < 22, then no update
	3. If only one valid meas and maximum cno < 25, then no update
	*/
	if (svNumValid == 3 && maxCno < 18)
	{
		noUpdate = 1;
	}
	else if (svNumValid == 2 && maxCno < 22)
	{
		noUpdate = 1;
	}
	else if (svNumValid == 1 && maxCno < 25)
	{
		noUpdate = 1;
	}
#else
	if (svPrNumValid <= 0)
	{
		noUpdate = 1;
	}
#endif

	/*
	If KF need no update, then don't do coast mode update
	*/
	if (noUpdate)
	{
		SYS_LOGGING(OBJ_KF,LOG_INFO,"NO KF UPDATE,validSvNum:%d,maxCno:%d",svPrNumValid,maxCno);
		if (kfPvtInfo->kfFixMode > EXTRAPOLATE)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
		}
	}

	/* If time gap is larger than 10s, then reset KF */
	if (fabs(dt)> 10 )
	{
		kfPvtInfo->kfFixMode = NO_SOLN;
	}
	/* Restart KF */
	else if ((kfPvtInfo->kfFixMode <= EXTRAPOLATE && svPrNumValid >= 2 && svDrNumValid >= 2)||fabs(dt) >=3)// && maxCno > 25 && p_Kf->lsBlk->posCvg == TRUE)
	{
		if(gnss_Kf_Restart_Check(p_Kf , dt, kfPvtInfo->coastTime))
		{
			if (p_Kf->meas_blk->avgCno < 32 && p_Kf->meas_blk->measCnt < 15 && fabs(dt) > 5 && !p_Kf->ls_Pvt_Info->valid && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
			{
				kfInfo.status = KF_RESET;
				kfInfo.status &= (~KF_RUN);
				p_Kf->lsCheckCnt = 0;
				p_Kf->lsPassCheckCnt = 0;
				kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
				kfPvtInfo->kf_had_firstFix = FALSE;
				p_Kf->ls_Pvt_Info->lastTor = 0.0;
				memset(pStaticData, 0, sizeof(STATIC_DATA));
				SYS_LOGGING(OBJ_KF, LOG_INFO, "KF Reset at %16.6f", p_Kf->meas_blk->tor);
			}
			else
			{
				kfInfo.status = KF_RESTART;
				p_Kf->lsCheckCnt = 0;
				p_Kf->lsPassCheckCnt = 0;
				memset(pStaticData, 0, sizeof(STATIC_DATA));
				SYS_LOGGING(OBJ_KF, LOG_INFO, "KF Restart at %16.6f", pMeas->tor);
			}
		}
		else
		{
			kfInfo.status = KF_RESET;
			kfInfo.status &= (~KF_RUN);
			p_Kf->lsCheckCnt = 0;
			p_Kf->lsPassCheckCnt = 0;
			kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
			kfPvtInfo->kf_had_firstFix = FALSE;
			p_Kf->ls_Pvt_Info->lastTor = 0.0;
			memset(pStaticData,0,sizeof(STATIC_DATA));
			SYS_LOGGING(OBJ_KF,LOG_INFO,"KF Reset at %16.6f",p_Kf->meas_blk->tor);
		}
	}
	/* Reset KF */
	if (kfPvtInfo->kfFixMode == NO_SOLN)
	{
		kfInfo.status = KF_RESET;
		kfInfo.status &= (~KF_RUN);
		p_Kf->lsCheckCnt = 0;
		p_Kf->lsPassCheckCnt = 0;
		kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
		kfPvtInfo->kf_had_firstFix = FALSE;
		p_Kf->ls_Pvt_Info->lastTor = 0.0;
		memset(pStaticData,0,sizeof(STATIC_DATA));
		SYS_LOGGING(OBJ_KF,LOG_INFO,"KF Reset at %16.6f",p_Kf->meas_blk->tor);
	}
}

/**********************************************************************
* Function Name:    gnss_Kf_FixModeManager
*
* Description:
*    The main function of KF
*
* Input:
*     kfPvtInfo:    PVT information
*     validSvNum:   KF used SV number
*     dt:           time interval from last KF update to this KF update
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_FixModeManager(Kf_t* p_Kf, double dt)
{
	uint8_t              validSvNum;
	KF_PVT_INFO*    kfPvtInfo;
	history_Info    hisInfo;
	float             distance;

	gnss_Pe_Get_HisInfo(&hisInfo);

	kfPvtInfo = p_Kf->kf_Pvt_Info;
	kfPvtInfo->isAltHold = FALSE;

	validSvNum = p_Kf->kf_ctrl.freeNum;
	if (kfPvtInfo->kfFixMode == FULL_FIX)
	{
		kfPvtInfo->coastTime = 0;

		if (validSvNum == 3)
		{
			kfPvtInfo->kfFixMode = COAST3;
		}

		if (validSvNum == 2)
		{
			kfPvtInfo->kfFixMode = COAST2;
		}

		if (validSvNum == 1)
		{
			kfPvtInfo->kfFixMode = COAST1;
		}

		if (validSvNum == 0)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
		}

		/* when kfFixMode is full, check if need altitude hold */
		if (kfPvtInfo->kfFixMode == FULL_FIX)
		{
			distance = gnssCalPosDis(hisInfo.llaPosForAlt,kfPvtInfo->kfLLApos,1);
			/* 
			1. If horizon velocity is smaller than 2, then use altitude hold 
			2. check if in open sky scenario, if yes, then don't use altitude hold */
			if (peMode.userSceData.isPDRMode || peState.posHasBias)
			{
				if (fabs(kfPvtInfo->kfLLApos[2] - hisInfo.preciseAlt) > 40.0 && distance < 5e3 && fabs(hisInfo.preciseAlt) > 1e-3)
				{
					kfPvtInfo->isAltHold = TRUE;
				}
			}
			else
			{
				if (kfPvtInfo->kfHeadingVel < 2.0 && p_Kf->kfCnt > 20 &&
					kfPvtInfo->kfFixStatus == FIX_STATUS_NEW)
				{
					kfPvtInfo->isAltHold = TRUE;
				}
			}
			if (peMode.userSceData.isOpenSky == TRUE)
			{
				kfPvtInfo->isAltHold = FALSE;
			}
		}

	}
	else if (kfPvtInfo->kfFixMode == COAST3)
	{
		if (validSvNum >= 4)
		{
			kfPvtInfo->kfFixMode = FULL_FIX;
		}

		if (validSvNum == 2)
		{
			kfPvtInfo->kfFixMode = COAST2;
		}

		if (validSvNum == 1)
		{
			kfPvtInfo->kfFixMode = COAST1;
		}

		if (validSvNum == 0)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
		}
	}
	else if (kfPvtInfo->kfFixMode == COAST2)
	{
		if (validSvNum >= 4)
		{
			kfPvtInfo->kfFixMode = FULL_FIX;
		}

		if (validSvNum == 3)
		{
			kfPvtInfo->kfFixMode = COAST3;
		}

		if (validSvNum == 1)
		{
			kfPvtInfo->kfFixMode = COAST1;
		}

		if (validSvNum == 0)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
		}
	}
	else if (kfPvtInfo->kfFixMode == COAST1)
	{
		if (validSvNum >= 4)
		{
			kfPvtInfo->kfFixMode = FULL_FIX;
		}

		if (validSvNum == 2)
		{
			kfPvtInfo->kfFixMode = COAST2;
		}

		if (validSvNum == 3)
		{
			kfPvtInfo->kfFixMode = COAST3;
		}

		if (validSvNum == 0)
		{
			kfPvtInfo->kfFixMode = EXTRAPOLATE;
		}
	}

	if (kfPvtInfo->kfFixMode >= COAST1 && kfPvtInfo->kfFixMode <= COAST3)
	{
		if (kfPvtInfo->coastTime < 20.0)
		{
			kfPvtInfo->coastTime += fabs(dt);
		}
	}

	if (kfPvtInfo->kfFixMode == EXTRAPOLATE)
	{
		if (kfPvtInfo->coastTime < 20.0)
		{
			kfPvtInfo->coastTime += 1 * fabs(dt);
		}
	}

	if (kfPvtInfo->coastTime > 10.0)
	{
		kfPvtInfo->kfFixMode = NO_SOLN;
	}
}
/***********************************************************************
* 函数介绍: gnss_Kf_DistanceCheck
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/30/
***********************************************************************/
uint8_t gnss_Kf_DistanceCheck(KF_PVT_INFO* kfPvtInfo, meas_blk_t* pMeas)
{
	uint8_t        i,measCnt = 0;
	uint8_t        bGoodFix = TRUE;
	uint16_t       min_cno = 100;
	uint16_t       cn0;
	//S16       prnIdx = -1;
	float       average_cno = 0.0;
	float       delta_t;
	float       distance;
	float       vel, lastVel;
	float       threshold;

	if (!kfPvtInfo->kf_had_firstFix)
	{
		kfPvtInfo->kf_had_firstFix = TRUE;
	}
	else
	{
		if (pMeas->measCnt == 0)
		{
			return FALSE;
		}

		for (i = 0; i < pMeas->measCnt; i++)
		{
			if (pMeas->meas[i].prn == 0) continue;
			cn0 = pMeas->meas[i].cno;
			average_cno += cn0;
			measCnt++;
			if (cn0 < min_cno)
			{
				min_cno = cn0;
			}
		}
		if (measCnt == 0) return FALSE;

		average_cno = average_cno / measCnt;

		delta_t = (float)(kfPvtInfo->tor - kfPvtInfo->lastTor);
		if (delta_t < -SECS_IN_WEEK / 2.0)
		{
			delta_t += SECS_IN_WEEK;
		}
		else if (delta_t > SECS_IN_WEEK / 2.0)
		{
			delta_t -= SECS_IN_WEEK;
		}

		if (average_cno < 20 || measCnt < 5 || min_cno < 15 /*|| pdop > 4*/)
		{
			distance = (float)gnssClcSqrtAminusB_DBL(kfPvtInfo->ecefPos, kfPvtInfo->ecefPos_last, 3);

			vel = gnssClcSqrtSum_FLT(kfPvtInfo->ecefVel, 3);

			lastVel = gnssClcSqrtSum_FLT(kfPvtInfo->ecefVel_last, 3);

			threshold = (float)(0.5 * ((double)vel + lastVel) * fabs(delta_t));

			if ((((double)distance - threshold) > 5 * fabs(delta_t)) && distance > 50)
			{
				if (fabs(delta_t) < 30)
				{
					//extrapolate current position based on the last position
					for (i = 0; i < 3; i++)
					{
						/*temp = 0.5 * (kfPvtInfo->ecefVel_last[i] + kfPvtInfo->ecefVel[i]);
						kfPvtInfo->ecefPos[i] = (float)(kfPvtInfo->ecefPos_last[i] + delta_t * temp);*/
						//kfPvtInfo->ecefPos[i] = (float)(kfPvtInfo->ecefPos_last[i] + kfPvtInfo->ecefVel_last[i] * delta_t +
						//	0.5 * (kfPvtInfo->ecefVel_last[i] - kfPvtInfo->ecefVel[i]) * delta_t);
					}

					//gnssConvEcef2Lla(kfPvtInfo->ecefPos, kfPvtInfo->kfLLApos);

				}
				else
				{
					bGoodFix = FALSE;
				}
			}
		}
	}

	return bGoodFix;
}

/***********************************************************************
* 函数介绍: gnss_Kf_Fill_Heading
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：05/14/
***********************************************************************/
void gnss_Kf_Fill_Heading(Kf_t *p_Kf)
{
	uint8_t                flag = FALSE,cnt = 0;
	int32_t               i,j,temp_j;
	double               velocity = 0.0,distance,temp_heading,avgheading = 0.0,stdheading = 0.0, avgheading1 = 0.0, stdheading1 = 0.0, avgheading2 = 0.0, stdheading2 = 0.0;
	double               dheading[5] = {0.0}, pro_dheading[5] = {0.0}, pro_dheading1[5] = { 0.0 };
	KF_PVT_INFO*      kfPvtInfo = p_Kf->kf_Pvt_Info;
	STATIC_DATA*      pStatic = &(peMode.staticData);
	/* low speed mode heading process */
	if (pStatic->staticFlag)
	{
		p_Kf->kf_Pvt_Info->lowVelocity_cnt = 0;
		/* init static heading when just enter static mode */
		if ((pStatic->historyStatic & 0x3) == 0x1)
		{
			/* find a solid heading from back LLA&ENU data */
			for (i = 0; i < BACK_LLA_NUM;i++)
			{
				velocity = sqrt((double)p_Kf->kfLLABack[i].ENUvel[0] * p_Kf->kfLLABack[i].ENUvel[0] +
					(double)p_Kf->kfLLABack[i].ENUvel[1] * p_Kf->kfLLABack[i].ENUvel[1]);
				if (p_Kf->kfLLABack[i].t > 0 && (kfPvtInfo->tor - p_Kf->kfLLABack[i].t) < 20 && velocity >= 1.0)
				{
					dheading[cnt] = (float)atan2(p_Kf->kfLLABack[i].ENUvel[0], p_Kf->kfLLABack[i].ENUvel[1]);
					if (dheading[cnt] < 0) pro_dheading[cnt] = dheading[cnt] + (float)(2 * PI);
					else pro_dheading[cnt] = dheading[cnt];
					flag = TRUE;
					cnt++;
					if (cnt > 4)
					{
						for (j = 1; j < cnt; j++)
						{
							pro_dheading1[j - 1] = pro_dheading[j];
						}
						gnss_math_dstd(pro_dheading, cnt, &avgheading, &stdheading);
						gnss_math_dstd(pro_dheading1, cnt-1, &avgheading1, &stdheading1);
						for (j = 2; j < cnt; j++)
						{
							pro_dheading1[j - 2] = pro_dheading[j];
						}
						gnss_math_dstd(pro_dheading1, cnt - 2, &avgheading2, &stdheading2);
						if (stdheading * R2D > 4.5  && stdheading * R2D < 25.0 && stdheading1 * R2D > 4.5 && stdheading  < 1.7 * stdheading2 && stdheading < 2.5 * stdheading1 && stdheading1 < 1.6 * stdheading2)
						{
							pStatic->staticHeading = (float)dheading[0];
						}
						else if (stdheading * R2D > 10.0  && stdheading * R2D < 25.0 && stdheading1 * R2D > 8.0 && stdheading >= 1.7 * stdheading2 && stdheading < 4.5 * stdheading2 && stdheading1 < 2.5 * stdheading2 && stdheading1 > 1.35 * stdheading2)
						{
							pStatic->staticHeading = (float)dheading[0];
						}
						else if (stdheading * R2D > 8.0  && stdheading * R2D < 25.0 && stdheading1 * R2D > 8.0 && stdheading > 10.0 * stdheading2)
						{
							pStatic->staticHeading = (float)dheading[1];
						}
						else
						{
							temp_heading = fabs(pro_dheading[0] - avgheading);
							temp_j = 0;
							for (j = 1; j < cnt; j++)
							{
								if (fabs(pro_dheading[j] - avgheading) < temp_heading)
								{
									temp_heading = fabs(pro_dheading[j] - avgheading);
									temp_j = j;
								}
							}
							pStatic->staticHeading = (float)dheading[temp_j];
						}
						SYS_LOGGING(OBJ_KF, LOG_INFO, "Fill static heading with back velocity:tor = %8.3f, dt = %8.3f, vel=%8.3f", kfPvtInfo->tor, kfPvtInfo->tor - p_Kf->kfLLABack[i].t, velocity);
						break;
					}
				}
				if (cnt > 2 && cnt <= 4 && i > BACK_LLA_NUM - 2)
				{
					gnss_math_dstd(pro_dheading, cnt, &avgheading, &stdheading);
					temp_heading = fabs(pro_dheading[0] - avgheading);
					temp_j = 0;
					for (j = 1; j < cnt; j++)
					{
						if (fabs(pro_dheading[j] - avgheading) < temp_heading)
						{
							temp_heading = fabs(pro_dheading[j] - avgheading);
							temp_j = j;
						}
					}
					pStatic->staticHeading = (float)dheading[temp_j];
					break;
				}
				else if (cnt > 0 && i > BACK_LLA_NUM - 2)
				{
					if (cnt == 2)
					{
						pStatic->staticHeading = (float)dheading[1];
						break;
					}
					else if (cnt == 1)
					{
						pStatic->staticHeading = (float)dheading[0];
						break;
					}
				}
			}
			/* If can't find a solid history heading, then use heading from position */
			if (flag == FALSE)
			{
				for (i = BACK_LLA_NUM - 1; i >= 0; i--)
				{
					if ((kfPvtInfo->tor - p_Kf->kfLLABack[i].t) <= 20 && p_Kf->kfLLABack[i].t > 0)
					{
						distance = gnssCalPosDis(p_Kf->kfLLABack[i].LLApos,kfPvtInfo->kfLLApos,TRUE);
						pStatic->staticHeading = gnss_lla2_heading(p_Kf->kfLLABack[i].LLApos,kfPvtInfo->kfLLApos);
						if (pStatic->staticHeading > (float)PI) pStatic->staticHeading -= (float)(2*PI);
						else if (pStatic->staticHeading <= (float)-PI) pStatic->staticHeading += (float)(2*PI);
						flag = TRUE;
						SYS_LOGGING(OBJ_KF,LOG_INFO,"Fill static heading with back pos:tor = %8.3f, dt = %8.3f, distance=%8.3f, static_heading=%8.3f",kfPvtInfo->tor,kfPvtInfo->tor - p_Kf->kfLLABack[i].t,distance,pStatic->staticHeading*RAD2DEG);
						break;
					}
				}
				if (flag == TRUE)
				{
					if (fabs(((double)p_Kf->kf_Pvt_Info->kfHeadingDir_last - pStatic->staticHeading) * R2D) > 10 && fabs(pStatic->staticHeading) > 0)
					{
						pStatic->staticHeading = p_Kf->kf_Pvt_Info->kfHeadingDir_last;
						flag = FALSE;
					}
				}
				else
				{
					pStatic->staticHeading = p_Kf->kf_Pvt_Info->kfHeadingDir_last;
				}
			}
		}
		else
		{
		    pStatic->staticHeading = p_Kf->kf_Pvt_Info->kfHeadingDir_last;
		}

		if(fabs(pStatic->staticHeading)> 0 )
		{
			kfPvtInfo->kfHeadingDir = pStatic->staticHeading;
			kfPvtInfo->kfENUvel[0] = (float)(kfPvtInfo->kfHeadingVel * sin(kfPvtInfo->kfHeadingDir));
			kfPvtInfo->kfENUvel[1] = (float)(kfPvtInfo->kfHeadingVel * cos(kfPvtInfo->kfHeadingDir));
			kfPvtInfo->enuVel_last[0] = kfPvtInfo->kfENUvel[0];
			kfPvtInfo->enuVel_last[1] = kfPvtInfo->kfENUvel[1];
		}	
	}
	else
	{
		// TODO: not static, heading calculation
		velocity = gnssClcSqrtSum_FLT(kfPvtInfo->kfENUvel, 2);
		if (velocity < 1.45)
		{
			for (i = 0; i < BACK_LLA_NUM; i++)
			{
				velocity = sqrt((double)p_Kf->kfLLABack[i].ENUvel[0] * p_Kf->kfLLABack[i].ENUvel[0] +
					(double)p_Kf->kfLLABack[i].ENUvel[1] * p_Kf->kfLLABack[i].ENUvel[1]);
				if (p_Kf->kfLLABack[i].t > 0 && (kfPvtInfo->tor - p_Kf->kfLLABack[i].t) <= 15 && velocity >= 1.0)
				{
					dheading[cnt] = (float)atan2(p_Kf->kfLLABack[i].ENUvel[0], p_Kf->kfLLABack[i].ENUvel[1]);
					if (dheading[cnt] < 0) pro_dheading[cnt] = dheading[cnt] + (float)(2 * PI);
					else pro_dheading[cnt] = dheading[cnt];
					flag = TRUE;
					p_Kf->kf_Pvt_Info->lowVelocity_cnt = 0;
					cnt++;
					if (cnt > 3)
					{
						for (j = 1; j < cnt; j++)
						{
							pro_dheading1[j - 1] = pro_dheading[j];
						}
						gnss_math_dstd(pro_dheading, cnt, &avgheading, &stdheading);
						gnss_math_dstd(pro_dheading1, cnt - 1, &avgheading1, &stdheading1);
						if (stdheading * R2D > 4.5 && stdheading * R2D < 10.0 && stdheading1 * R2D > 2.5 && stdheading < 2.5 * stdheading1)
						{
							kfPvtInfo->kfHeadingDir = (float)dheading[0];
						}
						else if (stdheading * R2D > 8.0 && stdheading * R2D < 25.0 && stdheading1 * R2D > 8.0 && stdheading >= 1.5 * stdheading1 && stdheading < 2.5 * stdheading1)
						{
							kfPvtInfo->kfHeadingDir = (float)dheading[0];
						}
						else if (stdheading * R2D > 8.0 && stdheading * R2D < 25.0 && stdheading1 * R2D > 8.0 && stdheading < 1.5 * stdheading1)
						{
							kfPvtInfo->kfHeadingDir = (float)dheading[1];
						}
						else
						{
							temp_heading = fabs(pro_dheading[0] - avgheading);
							temp_j = 0;
							for (j = 1; j < cnt; j++)
							{
								if (fabs(pro_dheading[j] - avgheading) < temp_heading)
								{
									temp_heading = fabs(pro_dheading[j] - avgheading);
									temp_j = j;
								}
							}
							kfPvtInfo->kfHeadingDir = (float)dheading[temp_j];
						}
						SYS_LOGGING(OBJ_KF, LOG_INFO, "Fill static heading with back velocity:tor = %8.3f, dt = %8.3f, vel=%8.3f", kfPvtInfo->tor, kfPvtInfo->tor - p_Kf->kfLLABack[i].t, velocity);
						break;
					}
				}
				if (cnt > 0 && cnt <= 3 && i > BACK_LLA_NUM - 2)
				{
					if (cnt > 1)
					{
						kfPvtInfo->kfHeadingDir = (float)dheading[1];
						break;
					}
					else/* if (cnt == 1)*/
					{
						kfPvtInfo->kfHeadingDir = p_Kf->kf_Pvt_Info->kfHeadingDir_last;
						break;
					}
				}
			}
			if (flag == FALSE)
			{
				velocity = gnssClcSqrtSum_FLT(kfPvtInfo->kfENUvel, 2);
				if (velocity > 0.5 && velocity <= 1.15)
				{
					p_Kf->kf_Pvt_Info->lowVelocity_cnt += 1;
					if (p_Kf->kf_Pvt_Info->lowVelocity_cnt > 5) p_Kf->kf_Pvt_Info->lowVelocity_cnt = 5;
				}
				else
				{
					p_Kf->kf_Pvt_Info->lowVelocity_cnt = 0;
				}
				if(p_Kf->kf_Pvt_Info->lowVelocity_cnt < 2) kfPvtInfo->kfHeadingDir = p_Kf->kf_Pvt_Info->kfHeadingDir_last;
				else p_Kf->kf_Pvt_Info->kfHeadingDir_last = kfPvtInfo->kfHeadingDir;
			}
		}
	}
	p_Kf->kf_Pvt_Info->kfHeadingDir_last = kfPvtInfo->kfHeadingDir;
	GLOGD("flag: %d", flag);
}

/***********************************************************************
* 函数介绍: gnss_Kf_QualityControl
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/12/
***********************************************************************/
void gnss_Kf_QualityControl(Kf_t *p_Kf)
{
	uint8_t                i;
	uint8_t                bGoodFix = TRUE;
	KF_PVT_INFO*      kfPvtInfo = p_Kf->kf_Pvt_Info;
	LS_PVT_INFO*      lsPvtInfo = p_Kf->ls_Pvt_Info;
	//KF_PVT_INFO*      kfPvtBack = p_Kf->kf_Pvt_Back;
	GNSS_TIME*        pTime;
	Gnss_Cfg_t*       pCfg;

	pCfg = p_Kf->pCfg;


	pTime = gnss_tm_get_time();
	/* 
	If kfFixMode is in EXTRAPOLATE or NO_SOLN, then set 
	KF fix status as OLD which means don't output results
	*/
	p_Kf->state = KF_STATE_NORMAL;
	if (kfPvtInfo->kfFixMode < COAST1 || kfInfo.forceReset)
	{
		kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
		gnss_Kf_LastLsCopy(lsPvtInfo);
		//bGoodFix = FALSE;
		return;
	}

	//KF Alt check and Vel valid range check
	if (kfPvtInfo->kfLLApos[2] > pCfg->altHighLimit || kfPvtInfo->kfLLApos[2] < pCfg->altLowLimit)
	{
		bGoodFix = FALSE;
		kfInfo.status = KF_RESET;
		SYS_LOGGING(OBJ_KF, LOG_INFO, "KF Altitude check fail");
		//KF Vel check
	}
	else if ((kfPvtInfo->kfHeadingVel > 100.0 && !g_pe_cfg.ppk_mode)|| (kfPvtInfo->kfHeadingVel > 500.0 && g_pe_cfg.ppk_mode))
	{
		bGoodFix = FALSE;
		kfInfo.status = KF_RESET;
		SYS_LOGGING(OBJ_KF, LOG_INFO, "Abnormal KF horizon velocity");
	}
	else if (fabs(kfPvtInfo->kfENUvel[2]) > 10.0)
	{
	  bGoodFix = FALSE;
	  kfInfo.status = KF_RESET;
	  SYS_LOGGING(OBJ_KF, LOG_INFO, "Abnormal KF vertical velocity");
	}

	if (bGoodFix)
	{
		if (kfPvtInfo->kfLLApos[2] < pCfg->altHighLimit && peMode.tunnelMode.mode == FALSE &&
			kfPvtInfo->kfLLApos[2] > pCfg->altLowLimit && kfPvtInfo->kfErrEst.h < 10000)
		{
			bGoodFix = gnss_Kf_DistanceCheck(kfPvtInfo,p_Kf->meas_blk);
		}
		else
		{
			/* If tunnel Mode */
			if (peMode.tunnelMode.mode == TRUE)
			{
				bGoodFix = FALSE;
			}
		}
	}

	if (bGoodFix)
	{
		// save current results
		kfPvtInfo->lastTor = kfPvtInfo->tor;
		for (i = 0; i < 3; i++)
		{
			kfPvtInfo->ecefPos_last[i] = kfPvtInfo->ecefPos[i];
			kfPvtInfo->ecefVel_last[i] = kfPvtInfo->ecefVel[i];

			kfPvtInfo->llaPos_last[i] = kfPvtInfo->kfLLApos[i];
			kfPvtInfo->enuVel_last[i] = kfPvtInfo->kfENUvel[i];
		}

		// save LLA data
		if (p_Kf->kfCnt >= 10 && peMode.staticData.staticFlag == FALSE)
		{
			for (i = BACK_LLA_NUM-1; i > 0;i--)
			{
				memcpy(&(p_Kf->kfLLABack[i]),&(p_Kf->kfLLABack[i-1]), sizeof(KF_LLA_BACK));
			}
			for (i = 0; i < 3; i++)
			{
				p_Kf->kfLLABack[0].LLApos[i] = kfPvtInfo->kfLLApos[i];
				p_Kf->kfLLABack[0].ENUvel[i] = kfPvtInfo->kfENUvel[i];
			}
			p_Kf->kfLLABack[0].t = kfPvtInfo->tor;
		}
		// calculate bias Number
		gnss_Pe_BiasNum(p_Kf->meas_blk);
		for (i = 0; i < BIAS_NUM; i++)
		{
			kfPvtInfo->clkBias_last[i] = kfPvtInfo->clkBias[i];
			kfPvtInfo->biasValid[i] = ((p_Kf->meas_blk->satModeCnt)>>i)&0x1;
		}
		kfPvtInfo->clkDrift_last = kfPvtInfo->clkDrift;


		kfPvtInfo->kfFixStatus = FIX_STATUS_NEW;
		if (kfPvtInfo->kfFixMode == FULL_FIX)
		{
			kfPvtInfo->kfFixSource = FIX_SOURCE_3D;
			/* calculate reference altitude for altitude hold */
			gnss_Kf_Alt_AlphaBeta(p_Kf);
			kfPvtInfo->altitudeRef = (float)kfPvtInfo->kfLLApos[2];
			/* calculate reference heading for heading hold */
			if (kfPvtInfo->kfHeadingVel >= 1.5)
			{
				if(kfPvtInfo->headingRef == 0.0)
				{
					kfPvtInfo->headingRef = (float)(kfPvtInfo->kfHeadingDir * RAD2DEG);
				}else
				{
					kfPvtInfo->headingRef = (float)0.5 * kfPvtInfo->headingRef + (float)(0.5 * kfPvtInfo->kfHeadingDir * RAD2DEG);
				}
			}

			/* calculate smoothed clock drift for clock hold */
			if (kfPvtInfo->clkDriftRef == 0.0)
			{
				kfPvtInfo->clkDriftRef = kfPvtInfo->clkDrift_last;
			}else
			{
				kfPvtInfo->clkDriftRef = (float)(kfPvtInfo->clkDriftRef * 3.0 / 4.0 + kfPvtInfo->clkDrift_last / 4.0);
			}

		}
		else if (kfPvtInfo->kfFixMode == COAST3)
		{
			kfPvtInfo->kfFixSource = FIX_SOURCE_2D;
		}
		else
		{
			kfPvtInfo->kfFixSource = FIX_SOURCE_APPX;
		}
		kfPvtInfo->kfFixStatus_last = kfPvtInfo->kfFixStatus;
		kfPvtInfo->kfFixSource_last = kfPvtInfo->kfFixSource;

		kfPvtInfo->kfTow = pTime->rcvr_time[GPS_MODE];
		kfPvtInfo->kfWeek = pTime->week[GPS_MODE];
		gnss_Kf_Fill_Heading(p_Kf);
	}
	else //KF has no new fix
	{
		kfPvtInfo->kfFixSource = FIX_SOURCE_NONE;
		if (kfPvtInfo->kfFixStatus != FIX_STATUS_NONE)
		{
			kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
		}
		gnss_Kf_LastLsCopy(lsPvtInfo);
		SYS_LOGGING(OBJ_KF,LOG_INFO,"%s,%d",__FUNCTION__,__LINE__);
	}
}
/***********************************************************************
* 函数介绍: gnss_Kf_ResultLoad
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/13/
***********************************************************************/
void gnss_Kf_ResultLoad(Kf_t *p_Kf)
{
	uint8_t           i;
	float          diagPenu[6];
	double          diagPecef[N_STATE];
	KF_PVT_INFO* kfPvtInfo;
	KF_PVT_INFO* kfPvtBack;
	STATIC_DATA*  kfStaticData;
	USER_MOTION*  pUsrContext;

	kfPvtInfo = p_Kf->kf_Pvt_Info;
	kfPvtBack = p_Kf->kf_Pvt_Back;
	kfStaticData = &(peMode.staticData);
	pUsrContext = &(peMode.userContext);

	// save last valid results
	memcpy(kfPvtBack,kfPvtInfo,sizeof(KF_PVT_INFO));

	// get current results
	kfPvtInfo->tor = kfInfo.tor;
	// ECEF position and velocity
	for (i = 0 ; i < 3; i++)
	{
		kfPvtInfo->ecefPos[i] = kfInfo.X[i];
		if(!(kfStaticData->staticFlag && (pUsrContext->isVehicleMode || (g_pe_cfg.chipType == UBLOX && (kfStaticData->historyStatic & 0x3) == 0x1))))
		{
			kfPvtInfo->ecefVel[i] = (float)kfInfo.X[i + 3];
		}
	}

	// Bias and Drift
	for (i = 0 ; i < BIAS_NUM; i++)
	{
		kfPvtInfo->clkBias[i] = kfInfo.X[i + 6];
		kfPvtInfo->dcb[i] = kfInfo.X[i + 11];
		kfPvtInfo->dcb[i + GNSS_MAX_MODE] = kfInfo.X[i + 11 + GNSS_MAX_MODE];
	}
	SYS_LOGGING(OBJ_PE, LOG_INFO, "dcb:%14.6f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", kfInfo.tor, 
		kfPvtInfo->dcb[0], kfPvtInfo->dcb[1], kfPvtInfo->dcb[2], kfPvtInfo->dcb[3], kfPvtInfo->dcb[4], kfPvtInfo->dcb[5], kfPvtInfo->dcb[6], kfPvtInfo->dcb[7]);

	kfPvtInfo->clkDrift = (float)kfInfo.X[10];

	/* get LLA position */
	gnssConvEcef2Lla(kfPvtInfo->ecefPos, kfPvtInfo->kfLLApos);

	/* get ENU velocity */
	gnssConvEcef2EnuVel(kfPvtInfo->ecefVel, kfPvtInfo->kfENUvel, kfPvtInfo->kfLLApos);

	/* heading velocity and direction */
	kfPvtInfo->kfHeadingVel = gnssClcSqrtSum_FLT(kfPvtInfo->kfENUvel, 2);
	kfPvtInfo->kfHeadingDir = (float)atan2(kfPvtInfo->kfENUvel[0], kfPvtInfo->kfENUvel[1]);

	/* err_est, set forceReset */
	gnssGetEnuPmatVar(kfInfo.U_plus, kfInfo.D_plus, kfPvtInfo->kfLLApos, diagPecef, diagPenu, &(kfPvtInfo->enPosCovar), kfPvtInfo->ecefPosPmat);

	SYS_LOGGING(OBJ_PE,LOG_INFO,"PMATRIX:%14.6f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f",kfInfo.tor,diagPecef[0],diagPecef[1],diagPecef[2],diagPecef[3],diagPecef[4],
		diagPecef[5],diagPecef[6],diagPecef[7],diagPecef[8],diagPecef[9],diagPecef[10],diagPenu[0],diagPenu[1],diagPenu[2]);

	for (i = 0; i < N_STATE; i++)
	{
		if (diagPecef[i] < 0.0)
		{
			kfInfo.forceReset = TRUE;     /**/
			diagPecef[i] = 50000.0;
		}
	}

	/* horizon,vertical position error, bias and drift error */
	kfPvtInfo->kfErrEst.h = (float)sqrt((double)diagPenu[0] + diagPenu[1]);
	kfPvtInfo->kfErrEst.v = (float)sqrt(diagPenu[2]);
	kfPvtInfo->kfErrEst.p = (float)sqrt((double)diagPenu[0] + diagPenu[1] + diagPenu[2]);

	if (p_Kf->kf_ctrl.biasLinkFlag == 0)
	{
		kfPvtInfo->kfErrEst.bias[0] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[1] = (float)sqrt(diagPecef[7]);
		kfPvtInfo->kfErrEst.bias[2] = (float)sqrt(diagPecef[8]);
		kfPvtInfo->kfErrEst.bias[3] = (float)sqrt(diagPecef[9]);
	}
	else if (p_Kf->kf_ctrl.biasLinkFlag == 1)
	{
		kfPvtInfo->kfErrEst.bias[0] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[1] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[2] = (float)sqrt(diagPecef[8]);
		kfPvtInfo->kfErrEst.bias[3] = (float)sqrt(diagPecef[9]);
	}
	else if (p_Kf->kf_ctrl.biasLinkFlag == 2)
	{
		kfPvtInfo->kfErrEst.bias[0] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[1] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[2] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[3] = (float)sqrt(diagPecef[9]);
	}
	else 
	{
		kfPvtInfo->kfErrEst.bias[0] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[1] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[2] = (float)sqrt(diagPecef[6]);
		kfPvtInfo->kfErrEst.bias[3] = (float)sqrt(diagPecef[6]);
	}


	kfPvtInfo->kfErrEst.drift = (float)(sqrt(diagPecef[10]) / LIGHT_SEC);

	/* Lat, lon and altitude error */
	kfPvtInfo->kfErrEst.lon_err = (float)sqrt(diagPenu[0]);
	kfPvtInfo->kfErrEst.lat_err = (float)sqrt(diagPenu[1]);
	kfPvtInfo->kfErrEst.alt_err = (float)sqrt(diagPenu[2]);

	/* ENU velocity variance */
	kfPvtInfo->kfErrEst.ve_err = (float)sqrt(diagPenu[3]);
	kfPvtInfo->kfErrEst.vn_err = (float)sqrt(diagPenu[4]);
	kfPvtInfo->kfErrEst.vd_err = (float)sqrt(diagPenu[5]);

	if (kfStaticData->staticFlag&&fabs(p_Kf->errBack.p)> 0)
	{
		kfPvtInfo->kfErrEst = p_Kf->errBack;
	}
	if (kfStaticData->staticFlag&&fabs(p_Kf->enPosCovarBack)> 0)
	{
		kfPvtInfo->enPosCovar = p_Kf->enPosCovarBack;
	}
}


/***********************************************************************
* 函数介绍: gnss_Kf_Reset_Proc
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/14/
***********************************************************************/
uint8_t gnss_Kf_Reset_Proc(Kf_t *p_Kf)
{
	uint8_t                   lsPassCheck;
	double                  T;
	LS_PVT_INFO*         lsPvtInfo;

	if (!IS_PPK_REVERSE)
	{
		T = 1.0;
	}
	else
	{
		T = -1.0;
	}

	lsPvtInfo = p_Kf->ls_Pvt_Info;
	lsPassCheck = gnss_Kf_CheckLs(p_Kf);
	if (lsPassCheck == TRUE)
	{
		SYS_LOGGING(OBJ_KF,LOG_INFO,"LS check pass to start KF");
		gnss_Kf_Start(T,p_Kf, KF_INIT);
		return TRUE;
	}else
	{
		gnss_Kf_LastLsCopy(lsPvtInfo);
		return FALSE;
	}
}

/***********************************************************************
* 函数介绍: gnss_Kf_BiasLink_Check
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：05/24/
***********************************************************************/
void gnss_Kf_BiasLink_Check(meas_blk_t* pMeas,Kf_t *p_Kf)
{
	uint8_t              i;
	double             biasDiffUse[GNSS_MAX_MODE] = {0.0};

	//load  bias diff for use
	for (i = 0; i < GNSS_MAX_MODE; i++)
	{
		if(pMeas->rtdUsed && p_Kf->biasDiffUpCnt[i]>=BIASDIFF_UPCNT_THRES)
		{
			biasDiffUse[i] = p_Kf->biasDiff[i];
		}
		else if(!pMeas->rtdUsed && p_Kf->biasDiffLocalUpCnt[i]>=BIASDIFF_UPCNT_THRES)
		{
			biasDiffUse[i] = p_Kf->biasDiffLocal[i];
		}
		else
		{
			biasDiffUse[i] = 0.0;
		}
	}

	p_Kf->kf_ctrl.biasLinkFlag = 0;
	if (IS_MEAS_GLN_USED((&g_pe_cfg)) && IS_MEAS_BDS_USED((&g_pe_cfg)) && IS_MEAS_GAL_USED((&g_pe_cfg)))
	{
		if (fabs(biasDiffUse[GLN_MODE]) > 0.01 && fabs(biasDiffUse[BDS_MODE]) > 0.01 && fabs(biasDiffUse[GAL_MODE]) > 0.01)
		{
			/***
			(1). If there're GPS,GLN,BDS and GAL, and only six measurements, then use bias link;
			(2). If GPS and GLN bias has large error, or GPS and BDS bias has large error, or GPS and GAL bias has large error, then use bias link;
			(3).
			****/
			if (pMeas->prNumEachSystem[GPS_MODE] > 0 && pMeas->prNumEachSystem[GLN_MODE] > 0 && pMeas->prNumEachSystem[BDS_MODE] > 0 && pMeas->prNumEachSystem[GAL_MODE] > 0
				&& pMeas->validPrNum <= 8)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 3;
			}
			else if (p_Kf->biasErrFlag[GLN_MODE] == TRUE || p_Kf->biasErrFlag[BDS_MODE] == TRUE || p_Kf->biasErrFlag[GAL_MODE] == TRUE)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 3;
			}
		}
	}
	else if (IS_MEAS_GLN_USED((&g_pe_cfg)) && IS_MEAS_BDS_USED((&g_pe_cfg)) && IS_MEAS_GAL_MASKED((&g_pe_cfg)))
	{
		if (fabs(biasDiffUse[GLN_MODE]) > 0.01 && fabs(biasDiffUse[BDS_MODE]) > 0.01)
		{
			/***
			(1). If there're GPS,GLN and BDS, and only six measurements, then use bias link;
			(2). If GPS and GLN bias has large error, or GPS and BDS bias has large error, then use bias link;
			(3). 
			****/
			if (pMeas->prNumEachSystem[GPS_MODE] > 0 && pMeas->prNumEachSystem[GLN_MODE] > 0 && pMeas->prNumEachSystem[BDS_MODE] > 0
				&& pMeas->validPrNum <= 6)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 2;
			}else if (p_Kf->biasErrFlag[GLN_MODE] == TRUE || p_Kf->biasErrFlag[BDS_MODE] == TRUE)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 2;
			}
		}
	}
	else if (IS_MEAS_GLN_USED((&g_pe_cfg)) && IS_MEAS_BDS_MASKED((&g_pe_cfg)) && IS_MEAS_GAL_MASKED((&g_pe_cfg)))
	{
		if (fabs(biasDiffUse[GLN_MODE]) > 0.01)
		{
			if (pMeas->prNumEachSystem[GPS_MODE] > 0 && pMeas->prNumEachSystem[GLN_MODE] > 0
				&& pMeas->validPrNum <= 4)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 1;
			}else if (p_Kf->biasErrFlag[GLN_MODE] == TRUE)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 1;
			}else if (pMeas->avgCno < 20)
			{
				p_Kf->kf_ctrl.biasLinkFlag = 1;
			}
		}
	}
}
/***********************************************************************
* 函数介绍: gnss_Kf_Reset_UseLs
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/24/
***********************************************************************/
static void gnss_Kf_Reset_UseLs(meas_blk_t* pMeas,Kf_t *p_Kf)
{
	uint8_t                     flag1 = FALSE,flag2 = TRUE, PosBias = TRUE;
	uint8_t                     i,j;
	uint8_t                     a;
	float                    distance,dis[3];
	double                    dt,ecefPos[3],llaPos[3];
	double                    heightThres;
	static LS_PVT_INFO     lsPvtRlst[3];
	static KF_PVT_INFO     kfPvtRlst[3];
	static double             lsPosRes[3];
	static double             kfPosRes[3];

	p_Kf->NoZuptStart = FALSE;

	/* If KF didn't run, then reset */
	if ((kfInfo.status & KF_RUN) == 0)
	{
		memset(lsPvtRlst,0,3*sizeof(LS_PVT_INFO));
		memset(kfPvtRlst,0,3*sizeof(KF_PVT_INFO));
		memset(lsPosRes,0,3*sizeof(double));
		memset(kfPosRes,0,3*sizeof(double));
		return;
	}

	lsPvtRlst[2] = lsPvtRlst[1];
	lsPvtRlst[1] = lsPvtRlst[0];
	lsPvtRlst[0] = *(p_Kf->ls_Pvt_Info);

	kfPvtRlst[2] = kfPvtRlst[1];
	kfPvtRlst[1] = kfPvtRlst[0];
	kfPvtRlst[0] = *(p_Kf->kf_Pvt_Info);

	lsPosRes[2] = lsPosRes[1];
	lsPosRes[1] = lsPosRes[0];
	lsPosRes[0] = lsPvtRlst[0].pos_res;

	kfPosRes[2] = kfPosRes[1];
	kfPosRes[1] = kfPosRes[0];
	kfPosRes[0] = p_Kf->posResStd;

	/* LS and KF time epoch check */
	if (fabs(lsPvtRlst[0].tor - kfPvtRlst[0].tor) > 0.1) 
	{
		return;
	}

	/* check time gap */
	for (i = 1;i < 3;i++)
	{
		dt = fabs(lsPvtRlst[i].tor - lsPvtRlst[i-1].tor);
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
		if (!IS_PPK_REVERSE && (dt > 1.5 || dt < 0.1)) return;
		if (IS_PPK_REVERSE && (dt < -1.5 || dt > -0.1)) return;

		dt = fabs(kfPvtRlst[i].tor - kfPvtRlst[i-1].tor);
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
		if (!IS_PPK_REVERSE && (dt > 1.5 || dt < 0.1)) return;
		if (IS_PPK_REVERSE && (dt < -1.5 || dt > -0.1)) return;
	}

	for (i = 0; i < 3; i++)
	{
		if (lsPosRes[i] > kfPosRes[i] && kfPosRes[i] > 0) return;
	}

	/* check the distance between LS and KF */
	for (i = 0; i < 3; i++)
	{
		dis[i] = gnssCalPosDis(lsPvtRlst[i].LLApos,kfPvtRlst[i].kfLLApos,1);
		if (dis[i] < 40.0 && flag1 == FALSE)
		{
			flag1 = TRUE;
		}
		if (dis[i] > 10.0 && flag2 == TRUE)
		{
			flag2 = FALSE;
		}
	}


	for (i = 0; i < 3; i++)
	{
		if ((kfPosRes[i] < 4.0) || (lsPosRes[i] > 2.0))
		{
			PosBias = FALSE;
		}
		
		if (dis[i] < 4.0)
		{
			PosBias = FALSE;
		}
	}
	if (PosBias && (pMeas->goodPrCnt >= 10) && ((double)pMeas->goodPrCnt / pMeas->CheckPrCnt) > 0.8 && (pMeas->avgCno > 36))
	{
		p_Kf->NoZuptStart = TRUE;
		SYS_LOGGING(OBJ_KF, LOG_ERROR, "Not do zupt because position bias(%s,%14.4f)", __FUNCTION__, pMeas->tor);
	}


	p_Kf->isKfRsltGood = flag2;
	if (flag1 || pMeas->prDiff10Cnt >= 6 || pMeas->prDiff20Cnt >= 8 || pMeas->prDiffDistanceAvg10Cnt >= 6)
	{
		return;
	}

	a = p_Kf->kf_ctrl.prRejNum;
	if (p_Kf->posResStd > 0.0 && p_Kf->posResStd < 25.0 && (p_Kf->kf_ctrl.prNum - a) >= 6)
	{
		return;
	}

	/* Assume no good LS to reset KF */
	if (peMode.userSceData.isUnderEleRoad)
	{
		return;
	}

	/* If KF didn't de-weight or reject too many satellite, then bypass */
	if (p_Kf->kf_ctrl.prNum > 0 && \
	   ((double)p_Kf->kf_ctrl.prDewNum + p_Kf->kf_ctrl.prRejNum) / p_Kf->kf_ctrl.prNum < 0.40)
	{
		return;
	}

	/* check LS position validation */
	for (i = 0; i < 3; i++)
	{
		if (lsPvtRlst[i].valid == FALSE || lsPvtRlst[i].pos_res > 50.0 || fabs(lsPvtRlst[i].enuVel[2]) > 10.0
			|| lsPvtRlst[i].vel_res > 2.0 || lsPvtRlst[i].fix_dim != DIM_3D)
		{
			return;
		}
	}

	/* check LS result quality */
	if (peMode.staticData.staticFlag)
	{
		heightThres = 25.0;
	}else
	{
		heightThres = 50.0;
	}
	for (i = 1;i < 3;i++)
	{
		dt = lsPvtRlst[i-1].tor - lsPvtRlst[i].tor;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}

		for (j = 0; j < 3;j++)
		{
			ecefPos[j] = lsPvtRlst[i].ecefPos[j] + lsPvtRlst[i].ecefVel[j] * dt;
		}
		gnssConvEcef2Lla(ecefPos,llaPos);

		distance = gnssCalPosDis(llaPos,lsPvtRlst[i-1].LLApos,1);

		if (distance > 20.0)
		{
			return;
		}
		/* height check */
		if (fabs(lsPvtRlst[i].LLApos[2] - lsPvtRlst[i-1].LLApos[2]) > heightThres)
		{
			return;
		}
	}

	/* smooth LS results */
	ecefPos[0] = lsPvtRlst[0].ecefPos[0];
	ecefPos[1] = lsPvtRlst[0].ecefPos[1];
	ecefPos[2] = lsPvtRlst[0].ecefPos[2];
	for (i = 1; i < 3; i++)
	{
		dt = lsPvtRlst[0].tor - lsPvtRlst[i].tor;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
		for (j = 0;j < 3;j++)
		{
			ecefPos[j] += (lsPvtRlst[i].ecefPos[j] + lsPvtRlst[i].ecefVel[j] * dt);
		}
	}
	for (j = 0; j < 3;j++)
	{
		ecefPos[j] /= 3;
	}

	/* Reset KF position use LS result */
	for (i = 0; i < 3; i++)
	{
		kfInfo.X[i] = ecefPos[i];
	}
	for (j = 0; j < 4;j++)
	{
		if (lsPvtRlst[0].biasValid[j])
		{
			kfInfo.X[6+j] = lsPvtRlst[0].clkBias[j];
		}
	}
	SYS_LOGGING(OBJ_KF,LOG_ERROR,"Reset KF position use LS(%s,%14.4f)",__FUNCTION__,pMeas->tor);
}
/***********************************************************************
* 函数介绍: gnss_Kf_OdoUp
* 输入参数：
* 输出参数：
* 返回值：
* 作者：	
* 日期
***********************************************************************/
static void gnss_Kf_OdoUp(KF_INFO* pkfInfo, double* deltaX, Kf_t* p_Kf)
{
	float		  fOdo2GnssVel = 0, fGnssVel = 0;
	float       drNoiseVarTmp;
	float		  fVelecef[3] = { 0 }, fGnssVelEnuBefor[3] = { 0 }, fGnssVelInb[3] = { 0 };
	float       fBeforVel[8] = { 0 };
	//double       test;
	double       H[N_STATE] = { 0 };
	double       innovation = 0;
	double       drKf = 0;
/* get ENU velocity */
	double      dPoslla[3] = { 0 };
	double      dCn2b[9] = { 0 };


#if defined(PLAYBACK_MODE)
	float      fVelEnu1[3] = { 0 };
	float      fGnssRefBefor[4] = { 0 }, fGnssBefor = 0;
	float	     fRef = 0, fGnss = 0;
	float      fOdoRef = 0, fGnssRef[4] = { 0 };
#endif

	fGnssVel = (float)sqrt(pkfInfo->X[3] * pkfInfo->X[3] + pkfInfo->X[4] * pkfInfo->X[4] + pkfInfo->X[5] * pkfInfo->X[5]);
	if (!peMode.feedbackFlag.uOdoVdrfeedback ||fGnssVel < 0.005 || fabs(vdrFeedbackInfo.fOdospeed) < 0.005 || peMode.staticData.staticFlag || !peMode.feedbackFlag.uEleVdrFeedback)
	{
#if defined(PLAYBACK_MODE)
		GLOGI("Can not do odo up: %10.3f %d %8.3f %8.3f %d", p_Kf->meas_blk->tor, peMode.feedbackFlag.uOdoVdrfeedback, fGnssVel, vdrFeedbackInfo.fOdospeed, peMode.staticData.staticFlag);
#endif	
		return;
	}
	gnssConvEcef2Lla(pkfInfo->X, dPoslla);
	fVelecef[0] = (float)pkfInfo->X[3];
	fVelecef[1] = (float)pkfInfo->X[4];
	fVelecef[2] = (float)pkfInfo->X[5];
	gnssConvEcef2EnuVel(fVelecef, fGnssVelEnuBefor, dPoslla);
	///////
	gnss_Math_MatrixTrans(vdrFeedbackInfo.dCb2n, 3, 3, dCn2b);
	gnss_Math_MatrixDot_DF(dCn2b, 3, 3, fGnssVelEnuBefor, 3, 1, fGnssVelInb);

	if (vdrFeedbackInfo.isReverse != 0 || vdrFeedbackInfo.fOdospeed < 0 || fGnssVelInb[1] < 0)
	{
#if defined(PLAYBACK_MODE)
		GLOGI("GnssVelReverse can not do odo up! %8.3f %8.3f %d %8.3f", p_Kf->meas_blk->tor, fGnssVelInb[1], vdrFeedbackInfo.isReverse, vdrFeedbackInfo.fOdospeed);
#endif // 0
		return;
	}

	/* get ENU velocity */
	fBeforVel[0] = (float)(pkfInfo->X[3] + deltaX[3]);
	fBeforVel[1] = (float)(pkfInfo->X[4] + deltaX[4]);
	fBeforVel[2] = (float)(pkfInfo->X[5] + deltaX[5]);
	gnssConvEcef2EnuVel(fBeforVel, fBeforVel + 4, dPoslla);

	drNoiseVarTmp = (float)(0.1 * 0.1);
	// prepare the H matrix for KF DR update
	H[3] = pkfInfo->X[3] / fGnssVel;
	H[4] = pkfInfo->X[4] / fGnssVel;
	H[5] = pkfInfo->X[5] / fGnssVel;
	// estimate DR
	//drKf = 0;
	drKf = sqrt((double)fBeforVel[0] * fBeforVel[0] + (double)fBeforVel[1] * fBeforVel[1] + (double)fBeforVel[2] * fBeforVel[2]);
	fOdo2GnssVel = (float)sqrt((double)vdrFeedbackInfo.fOdo2GnssVelENU[0] * vdrFeedbackInfo.fOdo2GnssVelENU[0] + (double)vdrFeedbackInfo.fOdo2GnssVelENU[1] * vdrFeedbackInfo.fOdo2GnssVelENU[1] + (double)vdrFeedbackInfo.fOdo2GnssVelENU[2] * vdrFeedbackInfo.fOdo2GnssVelENU[2]);
	innovation = fOdo2GnssVel - drKf;
	//test = udKFUpdate(H, deltaX, drNoiseVarTmp, innovation, testThres, MEAS_DR_UPDATE, flag, testFlag);
	udKFUpdate(H, deltaX, drNoiseVarTmp, innovation, SBOUND_DR, COAST_UPDATE, 1, 0);

#if defined(PLAYBACK_MODE)
	if (p_Kf->meas_blk->hpRefQflag[1])
	{
		fVelecef[0] = (float)(pkfInfo->X[3] + deltaX[3]);
		fVelecef[1] = (float)(pkfInfo->X[4] + deltaX[4]);
		fVelecef[2] = (float)(pkfInfo->X[5] + deltaX[5]);
		gnssConvEcef2EnuVel(fVelecef, fVelEnu1, dPoslla);

		fGnss = (float)sqrt((double)fVelecef[0] * fVelecef[0] + (double)fVelecef[1] * fVelecef[1] + (double)fVelecef[2] * fVelecef[2]);
		fRef = (float)sqrt(p_Kf->meas_blk->hpRefLla[3] * p_Kf->meas_blk->hpRefLla[3] + p_Kf->meas_blk->hpRefLla[4] * p_Kf->meas_blk->hpRefLla[4] + p_Kf->meas_blk->hpRefLla[5] * p_Kf->meas_blk->hpRefLla[5]);
		fOdoRef = fOdo2GnssVel - fRef;
		fGnssRef[0] = (float)(fVelEnu1[0] - p_Kf->meas_blk->hpRefLla[3]);
		fGnssRef[1] = (float)(fVelEnu1[1] - p_Kf->meas_blk->hpRefLla[4]);
		fGnssRef[2] = (float)(fVelEnu1[2] - p_Kf->meas_blk->hpRefLla[5]);
		fGnssRef[3] = fGnss - fRef;

		fGnssRefBefor[0] = (float)(fBeforVel[4] - p_Kf->meas_blk->hpRefLla[3]);
		fGnssRefBefor[1] = (float)(fBeforVel[5] - p_Kf->meas_blk->hpRefLla[4]);
		fGnssRefBefor[2] = (float)(fBeforVel[6] - p_Kf->meas_blk->hpRefLla[5]);
		fGnssBefor = (float)sqrt((double)fBeforVel[0] * fBeforVel[0] + (double)fBeforVel[1] * fBeforVel[1] + (double)fBeforVel[2] * fBeforVel[2]);
		fGnssRefBefor[3] = fGnssBefor - fRef;
		GLOGI("GNSSODOUPError: %8.3f %8.3f %8.3f %8.3f %8.3f  %8.3f %8.3f %8.3f ", p_Kf->meas_blk->tor, fOdoRef, fGnssRef[3], fGnssRef[0], fGnssRef[1],
			fGnssRefBefor[3], fGnssRefBefor[0], fGnssRefBefor[1]);
	}
#endif // 0

	return;
}

/***********************************************************************
* 函数介绍: gnss_kf_main
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/04/
***********************************************************************/
void gnss_Kf_Main(meas_blk_t* pMeas,Kf_t *p_Kf)
{
	uint8_t                   kfCntThres;
	uint32_t                  i;
	LS_PVT_INFO*         lsPvtInfo;
	KF_PVT_INFO*         kfPvtInfo;
	//KF_PVT_INFO*         kfPvtBack;
	KF_CTRL*             kfCtrl;
	double                  deltaX[N_STATE] = {0.0};
	double                  dt;
	GNSS_TIME*           pTime;
	uint16_t                  temp_value;
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
	double						dPreVelX[3] = { 0 };
	float                  fLastVel = 0, fCurVel = 0, fPreCurVel = 0;
	float                  fCurVelError = 0, fGnssDiff = 0, fRefDiff = 0, fPreCurVelError = 0;
	static float           fRefLastVel = 0, fRefCurVel = 0;
#endif
	/* KF Handler check */
	if (p_Kf == NULL)
	{
		SYS_LOGGING(OBJ_KF,LOG_ERROR,"Empty KF Handler %s(%d)",__FUNCTION__,__LINE__);
		return;
	}

	lsPvtInfo = p_Kf->ls_Pvt_Info;
	kfPvtInfo = p_Kf->kf_Pvt_Info;
	//kfPvtBack = p_Kf->kf_Pvt_Back;
	kfCtrl = &p_Kf->kf_ctrl;

	pTime = gnss_tm_get_time();
	dt = pTime->dt;
	if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ && pTime->init == TRUE)
	{
		dt = pMeas->tor - kfInfo.tor;
		if (dt > (SECS_IN_WEEK / 2)) dt -= (double)SECS_IN_WEEK;
		else if (dt < -(SECS_IN_WEEK / 2)) dt += (double)SECS_IN_WEEK;
	}

	if ((dt < 0 && !IS_PPK_REVERSE) || (dt > 0 && IS_PPK_REVERSE))
	{
		kfInfo.status = KF_RESET;
		kfInfo.status &= (~KF_RUN);
		p_Kf->lsCheckCnt = 0;
		p_Kf->lsPassCheckCnt = 0;
		kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
		kfPvtInfo->kf_had_firstFix = FALSE;
		lsPvtInfo->lastTor = 0.0;
		memset(&(p_Kf->kf_Static_Data), 0, sizeof(STATIC_DATA));
		SYS_LOGGING(OBJ_KF, LOG_INFO, "Abnormal Time Gap=%.6f , KF Not Work!", dt);
		return;
	}

	// If detect TOR and TOT have different jump, then reset KF
	if (pTime->isLargeTimeJmp)
	{
		kfInfo.status = KF_RESET;
		kfInfo.status &= (~KF_RUN);
		p_Kf->lsCheckCnt = 0;
		p_Kf->lsPassCheckCnt = 0;
		kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
		kfPvtInfo->kf_had_firstFix = FALSE;
		lsPvtInfo->lastTor = 0.0;
		memset(&(p_Kf->kf_Static_Data),0,sizeof(STATIC_DATA));
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Reset KF because of large time Jump");
	}

	if (kfInfo.status & KF_RUN)
	{
		SYS_LOGGING(OBJ_KF,LOG_INFO,"KF BIAS: %14.6f,%14.4f,%14.4f,%14.4f,%14.4f",kfInfo.tor,
			kfInfo.X[6],kfInfo.X[7],kfInfo.X[8],kfInfo.X[9]);
	}

	/* Check if need Bias Link for different satellite system */
	gnss_Kf_Check_BiasErr(p_Kf);

	/*
	Following routines 
	*/
	gnss_Pe_PRDR_Num(pMeas);
	gnss_Pe_BiasNum(pMeas);

	/* Check if need biasLink */
	gnss_Kf_BiasLink_Check(pMeas,p_Kf);

	SYS_LOGGING(OBJ_KF,LOG_INFO,"%s,link flag:%14.4f,%d,%02d,%02d,%02d",__FUNCTION__,pMeas->tor,p_Kf->kf_ctrl.biasLinkFlag,
		pMeas->prNumEachSystem[0],pMeas->prNumEachSystem[1],pMeas->prNumEachSystem[2]);
	/* 1. If KF was in initial status,check LS */
	if (kfInfo.status & KF_INIT)
	{
		if (lsPvtInfo->valid == FALSE)
		{
			SYS_LOGGING(OBJ_KF,LOG_INFO,"There is no LS result to start KF");
			return;
		}

		if (lsPvtInfo->fix_dim != DIM_3D)
		{
			SYS_LOGGING(OBJ_KF,LOG_INFO,"There is no 3D LS result to start KF");
		}

		if(!gnss_Kf_Reset_Proc(p_Kf))
		{
			return;
		}
	}
	else
	{
		gnss_Kf_RunCheck(p_Kf,dt);
	}
	p_Kf->lastKfStatus = kfInfo.status;
	kfCtrl->svModeInUse = pMeas->satModeCnt;
	kfInfo.tor = pMeas->tor;

	if (kfInfo.status & KF_RESET)
	{
		if(!gnss_Kf_Reset_Proc(p_Kf))
		{
			return;
		}
	}
	else if (kfInfo.status & KF_RESTART)
	{
		SYS_LOGGING(OBJ_KF,LOG_INFO,"KF Restart");
		gnss_Kf_Start(1,p_Kf, KF_RESTART);
	}

	/* 2. KF Filter Update logic */
	if ((kfPvtInfo->kfFixMode > NO_SOLN)  && (kfInfo.status & KF_RUN))
	{
		/* KF predicting process */
		gnss_Kf_Predict(p_Kf,dt);

		/* save filter status */
		gnss_Kf_Save_Filter(p_Kf,&kfInfo);

		/* KF update */
		if (kfPvtInfo->kfFixMode > EXTRAPOLATE)
		{
			/* set the flag to determine if need Chi-square test in KF */
			gnss_Kf_Set_ChiSqStatus(p_Kf);

			//gnss_Kf_PosBias_Detect(p_Kf,&kfInfo);

			/* KF Normal update */
			gnss_Kf_Update(p_Kf,deltaX);
		}
		/* KF fix mode detection */
		gnss_Kf_FixModeManager(p_Kf, dt);

		/* KF COAST mode */
		if ((kfPvtInfo->kfFixMode <= COAST3 && kfPvtInfo->kfFixMode >= COAST1) || kfPvtInfo->kfFixMode == FULL_FIX)
		{
			if (kfPvtInfo->kfFixMode <= COAST3 && p_Kf->kfCnt < 2 && p_Kf->meas_blk->measCnt <=3)
			{
				kfPvtInfo->kfFixStatus = FIX_STATUS_OLD;
				return;
			}
			else
			{
				gnss_Kf_CoastMode(kfPvtInfo, deltaX);
			}
		}
		gnss_Pe_PRDR_Num(pMeas);
#ifdef ZUPT
		if(p_Kf->kfInitPosType == KF_INIT_POS_ACC)
		{
			kfCntThres = 2;
		}
		else if(p_Kf->kfInitPosType == KF_INIT_POS_APPX)
		{
			kfCntThres = 5;
		}
		else if (g_pe_cfg.chipType == GNSS_MEAS_TYPE_UBLOX)
		{
			kfCntThres = 50;
		}
		else
		{
			kfCntThres = 10;
		}

		if (p_Kf->NoZuptStart)
		{
			p_Kf->NoZuptCnt = 0;
			p_Kf->NoZuptCnt++;
		}
		else if ((p_Kf->NoZuptCnt > 0) && (p_Kf->NoZuptCnt < 20))
		{
			p_Kf->NoZuptCnt++;
		}
		else
		{
			p_Kf->NoZuptCnt = 0;
		}
		if (peMode.staticData.staticFlag && !(peMode.userContext.isVehicleMode) && p_Kf->kfCnt>kfCntThres && (p_Kf->NoZuptCnt == 0))
		{
			if (g_pe_cfg.automobile == FALSE)
			{
				kfPvtInfo->posVarZupt = 0.04;
				kfPvtInfo->velVarZupt = 0.01;
				if (peState.posHasBias)
				{
					kfPvtInfo->posVarZupt = 0.10;
					kfPvtInfo->velVarZupt = 0.05;
				}else if (peMode.userSceData.isPDRMode)
				{
					kfPvtInfo->posVarZupt = 1.00;
					kfPvtInfo->velVarZupt = 0.05;
				}
			}else
			{
				kfPvtInfo->posVarZupt = 0.04;
				kfPvtInfo->velVarZupt = 0.01;
			}
			temp_value = (1000.0 / kfInfo.periodTms) <= 1.0 ? 1 : (uint16_t)(round(1000.0 / kfInfo.periodTms));
			/*if exceed 5mins then start ZUPT Hold Static*/
			if (p_Kf->kfZuptCnt > (uint32_t)(temp_value * 5 * 60) || g_pe_cfg.applyScenario == APPLY_SCENE_CAMERA)
			{
				gnss_Kf_ZUPT_Hold_Static(&kfInfo, deltaX, p_Kf);
			}
			else
			{
				gnss_Kf_ZUPT(&kfInfo, deltaX, p_Kf);
			}
			p_Kf->kfZuptCnt++;
		}
		else
		{
			p_Kf->kfZuptCnt = 0;
		}
#endif
		if (peMode.staticData.historyStatic && peMode.staticData.staticFlag == FALSE)
		{
			gnss_Kf_UpVel_Hold(&kfInfo,deltaX,kfPvtInfo,0.01);
		}else if (peMode.userSceData.isPDRMode)
		{
			gnss_Kf_PDR_UpVel_Hold(&kfInfo,deltaX,kfPvtInfo,0.01);
		}
		/* Heading hold after static */
		if (peMode.userSceData.isPDRMode)
		{
			if (kfPvtInfo->kfHeadingDir != 0.0)
			{
				kfPvtInfo->headingRef = (float)(kfPvtInfo->kfHeadingDir * RAD2DEG);
				//gnss_Kf_PosHeading_Hold(p_Kf,deltaX);
			}
		}
		gnss_Kf_OdoUp(&kfInfo, deltaX, p_Kf);
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
		dPreVelX[0] = kfInfo.X[3];
		dPreVelX[1] = kfInfo.X[4];
		dPreVelX[2] = kfInfo.X[5];
#endif
		/* KF update correction */
		for (i = 0; i < N_STATE; i++)
		{
			kfInfo.X[i] += deltaX[i];
		}
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
		fLastVel = (float)sqrt((double)kfPvtInfo->ecefVel[0] * kfPvtInfo->ecefVel[0] + (double)kfPvtInfo->ecefVel[1] * kfPvtInfo->ecefVel[1] + (double)kfPvtInfo->ecefVel[2] * kfPvtInfo->ecefVel[2]);
		fCurVel = (float)sqrt(kfInfo.X[3] * kfInfo.X[3] + kfInfo.X[4] * kfInfo.X[4] + kfInfo.X[5] * kfInfo.X[5]);
		fPreCurVel = (float)sqrt(dPreVelX[0] * dPreVelX[0] + dPreVelX[1] * dPreVelX[1] + dPreVelX[2] * dPreVelX[2]);

		fRefLastVel = fRefCurVel;
		if (pMeas->hpRefQflag[1])
		{
			fRefCurVel = (float)sqrt(pMeas->hpRefEcef[3] * pMeas->hpRefEcef[3] + pMeas->hpRefEcef[4] * pMeas->hpRefEcef[4] + pMeas->hpRefEcef[5] * pMeas->hpRefEcef[5]);
		}
		else {
			fRefCurVel = 0;
		}
		fGnssDiff = fCurVel - fLastVel;
		if (pMeas->hpRefQflag[1])
		{
			fRefDiff = fRefCurVel - fRefLastVel;
			fCurVelError = fCurVel - fRefCurVel;
			fPreCurVelError = fPreCurVel - fRefCurVel;
		}
		GLOGI("VEL deltxX DR: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", pMeas->tor, deltaX[0], deltaX[1], deltaX[2], deltaX[3], deltaX[4], deltaX[5],
			sqrt(deltaX[3] * deltaX[3] + deltaX[4] * deltaX[4] + deltaX[5] * deltaX[5]), fGnssDiff, fRefDiff, fCurVelError, fPreCurVelError);
#endif

		/* static constrain */
		gnss_Kf_Static_Proc(p_Kf);

		/* bias link process */
		if (pMeas->rtdUsed)
		{
			if (p_Kf->kf_ctrl.biasLinkFlag == 1)
			{
				kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
			}else if (p_Kf->kf_ctrl.biasLinkFlag == 2)
			{
				kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
				kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
			}else if (p_Kf->kf_ctrl.biasLinkFlag == 3)
			{
				kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiff[GLN_MODE];
				kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiff[BDS_MODE];
				kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiff[GAL_MODE];
			}

		}else
		{
			if (p_Kf->kf_ctrl.biasLinkFlag == 1)
			{
				kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
			}else if (p_Kf->kf_ctrl.biasLinkFlag == 2)
			{
				kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
				kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
			}else if (p_Kf->kf_ctrl.biasLinkFlag == 3)
			{
				kfInfo.X[7] = kfInfo.X[6] - p_Kf->biasDiffLocal[GLN_MODE];
				kfInfo.X[8] = kfInfo.X[6] - p_Kf->biasDiffLocal[BDS_MODE];
				kfInfo.X[9] = kfInfo.X[6] - p_Kf->biasDiffLocal[GAL_MODE];
			}
		}

		gnss_Kf_Post_Res(p_Kf,&kfInfo);
	}

	/* 3. KF results load */
	gnss_Kf_ResultLoad(p_Kf);

	/* 4. KF results QOS */
	gnss_Kf_QualityControl(p_Kf);
	if (peMode.userSceData.isPDRMode || peState.posHasBias)
	{
		gnss_Kf_Rslt_Smooth(p_Kf);
	}

	/* 5. Save some parameters */
	kfCtrl->lastSvModeInUse = kfCtrl->svModeInUse;

	/* 6. calculate heading according to position */
	gnss_Kf_RefHeading_Cal(p_Kf);

	/* 7. Check If need use LS to reset KF when KF position is biased */
	gnss_Kf_Reset_UseLs(pMeas,p_Kf);
}
