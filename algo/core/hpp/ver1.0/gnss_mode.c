/************************************************************
* Copyrights(C) 
* All rights Reserved
* �ļ����ƣ�gnss_mode.c
* �ļ��������û������Լ�ģʽ�ļ���Լ������߼�
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�09/10
************************************************************/
#include "gnss_mode.h"
#include "gnss_kf.h"
#include "gnss_sys_api.h"
#include "gnss_math.h"
#include "gnss_pe.h"
#include "gnss_sd.h"
#include "gnss_hsm_lite_api.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_QOS

extern uint8_t firstFix;
extern Gnss_Cfg_t g_pe_cfg;
extern float realVel;   // reference horizon velocity

extern VDR_Gnss_Feedback_t   vdrFeedbackInfo;
#define TUNNEL_CODESTD_THRES     10
/***********************************************************************
* ��������: gnss_Mode_Init
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/14/
***********************************************************************/
void gnss_Mode_Init(PE_MODES* pMode)
{
	memset(pMode,0,sizeof(PE_MODES));
	pMode->userContext.motion = USER_MOTION_UNKNOWN;
	pMode->userContext.context = USER_CONTEXT_UNKNOWN;
}

/***********************************************************************
* ��������: gnss_TunnelModeDet_MeasStd
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/27/
***********************************************************************/
#ifdef USED_IN_MC262M
void gnss_TunnelModeDet_MeasStd(meas_blk_t* pMeas,PE_MODES* pMode)
{
	uint8_t             tunnelModeDet = TRUE;
	uint8_t             diffValid[MEAS_SAVE_NUM] = {0};
	uint32_t            i,j,cnt = 0,cnt1 = 0,smallCnt = 0;
	double            dt,ratio;
	double            total_diff_avg = 0.0,diff_avg_std = 0.0,total_std_avg = 0.0,total_std_std = 0.0;
	double            diff_avg[MEAS_SAVE_NUM] = {0.0},diff_std[MEAS_SAVE_NUM]  = {0.0};
	double            diff[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0.0};
	double            avg_codeStd = 0.0;
	gnss_meas_t*   pSvMeas;
	sat_data_t*    sp;

	if(!firstFix)
	{
		return;
	}

	for (i = 0; i < pMeas->measCnt;i++)
	{
		pSvMeas = &pMeas->meas[i];
		if (pSvMeas->prn == 0) continue;

		avg_codeStd += pSvMeas->codeDetStd;
		cnt++;
		if(pSvMeas->codeDetStd < TUNNEL_CODESTD_THRES)
		{
			smallCnt++;
		}
	}

	if(cnt >0 )
	{
		avg_codeStd /= cnt;
		if(avg_codeStd < 20) 
		{
			return;
		}

		if((double)smallCnt/(double)cnt > 0.5)
		{
			return;
		}
	}

	for(j = 0; j< (MEAS_SAVE_NUM-1); j++)
	{ 
		cnt = 0;
		cnt1 = 0;
		memset(diff,0,MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(double));
		for (i = 0; i < pMeas->measCnt;i++)
		{
			pSvMeas = &pMeas->meas[i];
			if (pSvMeas->prn == 0) continue;
			sp = gnss_sd_get_sv_data(pSvMeas->gnssMode,pSvMeas->prn,pSvMeas->freq_index);
			if(sp == NULL) continue;

			dt = pMeas->tor - sp->measBack[0].tor;
			if(fabs(dt)>1.5) continue;

			dt = sp->measBack[j].tot - sp->measBack[j+1].tot;
			if (fabs(dt) < 2.0 && sp->measBack[j].pr > 0.0 && sp->measBack[j+1].pr > 0.0)
			{
				diff[cnt] = sp->measBack[j].codeStd - sp->measBack[j+1].codeStd;
				if(diff[cnt] <= 0 /*|| sp->measBack[j].codeStd < TUNNEL_CODESTD_THRES*/)
				{
					cnt1++;
				}
				else if(diff[cnt] <100)
				{
					cnt++;
				}
			}
		}

		if(cnt >= 5 )
		{   
			ratio = ((double)cnt1/(double)(cnt+cnt1));
			if(ratio <= 0.10||(ratio < 0.2 && cnt1 <= 2 && cnt >= 8))
			{
				diffValid[j] = TRUE;
				gnss_math_dstd(diff,cnt,&diff_avg[j],&diff_std[j]);
				//SYS_LOGGING(OBJ_QOS,LOG_INFO,"cal code std diff:tor =%8.3f, group = %d,cnt = %d,diff_avg = %8.3f,diff_std=%8.3f",pMeas->tor,j,cnt,diff_avg[j],diff_std[j]);
			}
		}
	}

	for(j = 0; j< (MEAS_SAVE_NUM-2); j++)
	{
		if(!diffValid[j])
		{
			tunnelModeDet = FALSE;
			break;
		}
	}

	if(pMeas->avgCno > 35)
	{
		tunnelModeDet = FALSE;
	}
	else if(pMeas->avgCno >25)
	{   
		if(!diffValid[MEAS_SAVE_NUM-2])
		{
			tunnelModeDet = FALSE;
		}
		else if(pMeas->avgCno >30)
		{
			gnss_math_dstd(diff_avg,MEAS_SAVE_NUM-1,&total_diff_avg,&diff_avg_std);
			gnss_math_dstd(diff_std,MEAS_SAVE_NUM-1,&total_std_avg,&total_std_std);
			if(diff_avg_std > 6.0 || total_std_std > 6.0)
			{
				tunnelModeDet = FALSE;
			}
		}
	}

	if(tunnelModeDet)
	{   
		pMode->tunnelMode.mode = TRUE;
		pMode->tunnelMode.detectCnt = 2;
		SYS_LOGGING(OBJ_KF,LOG_INFO,"Detect tunnel mode at tor =%8.3f",pMeas->tor);
	}
}
#endif
/***********************************************************************
* ��������: gnss_Tunnel_Mode
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/04/
***********************************************************************/
void gnss_Tunnel_Mode(meas_blk_t* pMeas,PE_MODES* pMode,KF_PVT_INFO* kf_pvt)
{
	uint8_t            i;
	uint8_t            cnt_180 = 0;
	uint8_t            cnt_150 = 0;
	uint8_t            cnt_200 = 0;
	uint8_t            cnt_60 = 0;
	uint8_t            avgCnt = 0;
	uint16_t           Cn0_1, maxCn0_1 = 0;
	float           avgCn0_1 = 0;
	float           avgCn0_1_Threshold;
	double           dt;
	gnss_meas_t*  pSvMeas;

	pMode->tunnelMode.last_mode = pMode->tunnelMode.mode;

	if(g_pe_cfg.chipType != UBLOX)
	{
		avgCn0_1_Threshold = (float)10.7;
	}
	else
	{
		avgCn0_1_Threshold = (float)18.1;
	}

	for (i = 0 ; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];

		if (pSvMeas->status || pSvMeas->cno > 14)
		{
			Cn0_1 = pSvMeas->cno;
		}
		else
		{
			continue;
		}

		if (Cn0_1 > maxCn0_1)
		{
			maxCn0_1 = Cn0_1;
		}

		if (Cn0_1 >= 18)
		{
			cnt_180++;
		}

		if (Cn0_1 >= 15)
		{
			cnt_150++;
		}

		if (Cn0_1 >= 25)
		{
			cnt_200++;
		}

		if (Cn0_1 <= 6)
		{
			cnt_60++;
		}

		avgCn0_1 += Cn0_1;
		avgCnt++;
	}

	if (avgCnt >= 1)
	{
		avgCn0_1 /= (float)avgCnt;
	}
	else
	{
		avgCn0_1 = 0;
	}

	// tunnel mode detection 
	if(g_pe_cfg.chipType != UBLOX)
	{
		if (avgCn0_1 < avgCn0_1_Threshold && cnt_150 < 3 && kf_pvt->kfFixStatus > FIX_STATUS_NONE /*&& HighCN0_cnt > 2*/)
		{
			pMode->tunnelMode.detectCnt++;
		}
		else
		{
			pMode->tunnelMode.detectCnt = 0;
		}
	}
	else
	{
		if (avgCn0_1 < avgCn0_1_Threshold && cnt_180 < 3 && kf_pvt->kfFixStatus > FIX_STATUS_NONE
			&& ((pMeas->maxCno < 28 && pMeas->maxCno_L5 < 28) && pMeas->measCnt < 12))
		{
			pMode->tunnelMode.detectCnt++;
		}
		else
		{
			pMode->tunnelMode.detectCnt = 0;
		}
	}

	if (pMode->tunnelMode.detectCnt >= 2)
	{
		pMode->tunnelMode.mode = TRUE;
		pMode->tunnelMode.detectCnt = 2;
		SYS_LOGGING(OBJ_KF,LOG_INFO,"TUNNEL MODE is detected! %16.8f", pMeas->tor);
	}

	// Non-tunnel detection
	if(g_pe_cfg.chipType != UBLOX)
	{
		if (((maxCn0_1 >= 20 && avgCn0_1 >= 15) && (avgCnt - cnt_60) > 3 && cnt_150 >= 3) || cnt_150 > 5)
		{
			pMode->tunnelMode.mode = FALSE;
			pMode->tunnelMode.detectCnt = 0;
		}
	}
	else
	{
		if ((((maxCn0_1 >= 20 && avgCn0_1 >= 18.5) && (avgCnt - cnt_60) > 3 && cnt_150 >= 3) || cnt_150 > 5))
		{
			pMode->tunnelMode.mode = FALSE;
			pMode->tunnelMode.detectCnt = 0;
		}
	}

#ifdef USED_IN_MC262M
	if(!pMode->tunnelMode.mode && (g_pe_cfg.chipType == QCOM))
	{
		gnss_TunnelModeDet_MeasStd(pMeas,pMode);
	}
#endif
	// exitTunnelFlag
	if (!pMode->tunnelMode.mode && pMode->tunnelMode.last_mode)
	{
		pMode->tunnelMode.exitTunnelFlag = TRUE;
		pMode->tunnelMode.tunnel_althold = TUNNEL_HOLD_NUM;
		pMode->tunnelMode.exitTunnelTime = pMeas->tor;
		SYS_LOGGING(OBJ_PE, LOG_INFO, "exitTunnelTime = %6.2f", pMode->tunnelMode.exitTunnelTime);
	}
	if (pMode->tunnelMode.exitTunnelFlag)
	{
		dt = pMeas->tor - pMode->tunnelMode.exitTunnelTime;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
	}
	else
	{
		dt = 0;
	}
	if (fabs(dt) > 12 || pMode->tunnelMode.mode)
	{
		pMode->tunnelMode.exitTunnelFlag = FALSE;
		pMode->tunnelMode.exitTunnelTime = 0;
	}

	SYS_LOGGING(OBJ_PE,LOG_INFO,"%s,%14.6f,tunnelMode=%d,exitTunnel=%d",__FUNCTION__,
		pMeas->tor,pMode->tunnelMode.mode,pMode->tunnelMode.exitTunnelFlag);
}

/***********************************************************************
* ��������: gnss_Indoor_Mode
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/23/
***********************************************************************/
void gnss_Indoor_Mode(meas_blk_t* pMeas,PE_MODES* pMode)
{
	/*
	Indoor mode is a special user context in which signal power is very
	low and HDOP is very bad. In such special context, we need use some 
	special QoS criteria, using height aiding and float control.

	In order to detect Indoor mode, we need know the indoor context character:
	(1) signal power is low; (2) Only half sky's satellites are visible;
	(3) Aiding ephemeris was must, which means we can't decode ephemeris in such
	low signal power environment;
	(4) Lower speed condition (sensor aiding)
	*/
	return;
}

/***********************************************************************
* ��������: gnss_Motion_Detect
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/23/
***********************************************************************/
void gnss_Motion_Detect(meas_blk_t* pMeas,PE_MODES* pMode,KF_PVT_INFO* kf_pvt)
{
	/*
	'Context detect' was used for motion state detection and user context determination.
	Following mode are the most useful: 
	(1) Speed mode (low speed or high speed)
	(2) Carrier type (mobile phone, vehicle, ship or aircraft)
	(3) ......
	*/
	uint32_t               i;
	USER_MOTION*      pUsrContext;
	STATIC_DATA*      kfStaticData;

	pUsrContext = &pMode->userContext;
	kfStaticData = &(pMode->staticData);

	/* If can't get stable KF results, return */
	if (kf_pvt->kfFixStatus != FIX_STATUS_NEW)
	{
		pUsrContext->motion = USER_MOTION_UNKNOWN;
		pUsrContext->epochCnt = 0;
		pUsrContext->inital = FALSE;
		memset(&pUsrContext->motionCnt[0],0,USER_MOTION_MAX * sizeof(uint8_t));
		return;
	}

	pUsrContext->epochCnt++;
	/* Speed compare */ 
	if (kf_pvt->kfHeadingVel < WALK_THRES)
	{
		pUsrContext->motionCnt[USER_MOTION_STATIC] += 1;
	}else if (kf_pvt->kfHeadingVel >= WALK_THRES && kf_pvt->kfHeadingVel < RUN_THRES)
	{
		pUsrContext->motionCnt[USER_MOTION_WALK] += 1;
	}else if (kf_pvt->kfHeadingVel >= RUN_THRES && kf_pvt->kfHeadingVel < DRIVE_THRES)
	{
		pUsrContext->motionCnt[USER_MOTION_RUN] += 1;
	}else if (kf_pvt->kfHeadingVel >= DRIVE_THRES)
	{
		pUsrContext->motionCnt[USER_MOTION_DRIVE] += 1;
	}

	if (!pUsrContext->inital)
	{
		if (pUsrContext->epochCnt >= 5)
		{
			/* Initial user context determination */
			for (i = USER_MOTION_STATIC; i <= USER_MOTION_DRIVE;i++)
			{
				if ((double)pUsrContext->motionCnt[i] / (double)pUsrContext->epochCnt >= 0.8)
				{
					pUsrContext->motion = i;
					pUsrContext->epochCnt = 0;
					pUsrContext->inital = TRUE;
					pUsrContext->contextEpochCnt = 0;
					memset(&pUsrContext->motionCnt[0],0,USER_MOTION_MAX * sizeof(uint8_t));
					break;
				}
			}
			if(!pUsrContext->inital)
			{
				if (((double)pUsrContext->motionCnt[USER_MOTION_STATIC]+pUsrContext->motionCnt[USER_MOTION_WALK])/ (double)pUsrContext->epochCnt >= 0.8)
				{
					if(pUsrContext->motionCnt[USER_MOTION_STATIC]>pUsrContext->motionCnt[USER_MOTION_WALK])
					{
						pUsrContext->motion = USER_MOTION_STATIC;
					}
					else
					{
						pUsrContext->motion = USER_MOTION_WALK;
					}
					pUsrContext->epochCnt = 0;
					pUsrContext->inital = TRUE;
					pUsrContext->contextEpochCnt = 0;
					memset(&pUsrContext->motionCnt[0],0,USER_MOTION_MAX * sizeof(uint8_t));
				}
			}

		}else
		{
			return;
		}
	}else
	{
		if (pUsrContext->epochCnt >= 3)
		{
			switch(pUsrContext->motion)
			{
			case USER_MOTION_STATIC:
				{
					if (pUsrContext->motionCnt[2] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_WALK;    
					}else if (pUsrContext->motionCnt[3] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_RUN;
					}else if (pUsrContext->motionCnt[4] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_DRIVE;
					}               
					break;
				}
			case USER_MOTION_WALK:
				{
					if (pUsrContext->motionCnt[1] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_STATIC;  
					}else if (pUsrContext->motionCnt[3] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_RUN;  
					}else if (pUsrContext->motionCnt[4] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_DRIVE;
					}               
					break;
				}
			case USER_MOTION_RUN:
				{
					if (pUsrContext->motionCnt[1] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_STATIC;  
					}else if (pUsrContext->motionCnt[2] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_WALK;  
					}else if (pUsrContext->motionCnt[4] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_DRIVE; 
					}
					break;
				}
			case USER_MOTION_DRIVE:
				{
					if (pUsrContext->motionCnt[3] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_RUN;
					}else if (pUsrContext->motionCnt[2] == pUsrContext->epochCnt)
					{
						pUsrContext->motion = USER_MOTION_WALK;
					}else if (pUsrContext->motionCnt[1] == pUsrContext->epochCnt)
					{        
						pUsrContext->motion = USER_MOTION_STATIC;
					}
					break;
				}
			default:break;
			}
			pUsrContext->epochCnt = 0;
			memset(&pUsrContext->motionCnt[0],0,USER_MOTION_MAX * sizeof(uint8_t));
		}
	}

	/* User context determine 
	1. First, determine whether it's hand or vehicle usage;
	2. If in hand and 10s motion is DRIVE, then it's in vehicle;
	3. If in vehicle and 180s motion is static or walk, then it's in hand;
	*/
	switch(pUsrContext->context)
	{
	case USER_CONTEXT_UNKNOWN:
		{
			pUsrContext->contextEpochCnt++;
			if (pUsrContext->motion == USER_MOTION_STATIC || pUsrContext->motion == USER_MOTION_WALK)
			{
				pUsrContext->contextCnt[USER_CONTEXT_HAND]++;
			}else if (pUsrContext->motion == USER_MOTION_DRIVE)
			{
				pUsrContext->contextCnt[USER_CONTEXT_VEHICLE]++;
			}
			if (pUsrContext->contextCnt[USER_CONTEXT_HAND] >= CONTEXT_INIT_THRES && 
				pUsrContext->contextCnt[USER_CONTEXT_HAND] == pUsrContext->contextEpochCnt)
			{
				pUsrContext->context = USER_CONTEXT_HAND;
				memset(&pUsrContext->contextCnt[0],0,sizeof(uint32_t)*USER_CONTEXT_MAX);
				pUsrContext->contextEpochCnt = 0;
			}else if (pUsrContext->contextCnt[USER_CONTEXT_VEHICLE] >= CONTEXT_INIT_THRES && 
				pUsrContext->contextCnt[USER_CONTEXT_VEHICLE] == pUsrContext->contextEpochCnt)
			{
				pUsrContext->context = USER_CONTEXT_VEHICLE;
				memset(&pUsrContext->contextCnt[0],0,sizeof(uint32_t)*USER_CONTEXT_MAX);
				pUsrContext->contextEpochCnt = 0;
			}

			if (pUsrContext->contextEpochCnt >= CONTEXT_INIT_THRES && pUsrContext->context == USER_CONTEXT_UNKNOWN)
			{
				pUsrContext->contextEpochCnt = 0;
				pUsrContext->contextCnt[USER_CONTEXT_HAND] = 0;
				pUsrContext->contextCnt[USER_CONTEXT_VEHICLE] = 0;
			}
			break;
		}
	case USER_CONTEXT_HAND:
		{
			pUsrContext->contextEpochCnt++;
			if (pUsrContext->motion == USER_MOTION_DRIVE)
			{
				pUsrContext->contextCnt[USER_CONTEXT_VEHICLE]++;
			}
			if (pUsrContext->contextCnt[USER_CONTEXT_VEHICLE] >= 10 &&
				pUsrContext->contextCnt[USER_CONTEXT_VEHICLE] == pUsrContext->contextEpochCnt)
			{
				pUsrContext->context = USER_CONTEXT_VEHICLE;
				memset(&pUsrContext->contextCnt[0],0,sizeof(uint32_t)*USER_CONTEXT_MAX);
				pUsrContext->contextEpochCnt = 0;
			}
			if (pUsrContext->contextEpochCnt >= 10)
			{
				pUsrContext->contextEpochCnt = 0;
				pUsrContext->contextCnt[USER_CONTEXT_VEHICLE] = 0;
			}
			break;
		}
	case USER_CONTEXT_VEHICLE:
		{
			pUsrContext->contextEpochCnt++;
			if (pUsrContext->motion == USER_MOTION_STATIC || pUsrContext->motion == USER_MOTION_WALK)
			{
				pUsrContext->contextCnt[USER_CONTEXT_HAND]++;
			}
			if (pUsrContext->contextCnt[USER_CONTEXT_HAND] >= 180 && 
				pUsrContext->contextCnt[USER_CONTEXT_HAND] == pUsrContext->contextEpochCnt)
			{
				pUsrContext->context = USER_CONTEXT_HAND;
				memset(&pUsrContext->contextCnt[0],0,sizeof(uint32_t)*USER_CONTEXT_MAX);
				pUsrContext->contextEpochCnt = 0;
			}
			if (pUsrContext->contextEpochCnt >= 180)
			{
				pUsrContext->contextEpochCnt = 0;
				pUsrContext->contextCnt[USER_CONTEXT_HAND] = 0;
			}
			break;
		}
	default: break;
	}

	if(g_pe_cfg.chipType == SPRD && pUsrContext->context == USER_CONTEXT_VEHICLE)
	{
		pUsrContext->isVehicleMode = TRUE;
	}
	else if (g_pe_cfg.automobile && g_pe_cfg.chipType == QCOM)
	{
		pUsrContext->isVehicleMode = TRUE;
	}

	SYS_LOGGING(OBJ_QOS,LOG_INFO,"User Motion: %d %d %d %d %d %8.3f %d %d",pUsrContext->motion,
		pUsrContext->motionCnt[0],pUsrContext->motionCnt[1],pUsrContext->motionCnt[2],
		pUsrContext->motionCnt[3],kf_pvt->kfHeadingVel,kfStaticData->staticFlag,pUsrContext->context);
}

static void gnss_DownTown_Det_QCOM(meas_blk_t* pMeas,PE_MODES* pMode)
{
	USER_SCENARIO*     p;

	p = &(pMode->userSceData);
	/* Downtown mode detection
	*/
	if(pMeas->prDiffStd_1 < 0)
	{
		p->diffBackCnt = 0;
		p->smoothDiffStd = -1.0;
	}
	else
	{
		gnss_math_data_smooth(pMeas->prDiffStd_1,p->prDiffStdBack,&(p->diffBackCnt),5,&(p->smoothDiffStd));
	}

	if(pMeas->prdrDiffStd_avg < 0 )
	{
		p->prDrdiffBackCnt = 0;
		p->smoothPrDrDiffStd = -1.0;
	}
	else
	{
		gnss_math_data_smooth((float)pMeas->prdrDiffStd_avg,p->prdrDiffStdBack,&(p->prDrdiffBackCnt),2,&(p->smoothPrDrDiffStd));
	}

	if(!p->isDownTown)
	{

#if 0
		if(p->smoothDiffStd > 75 && p->smoothPrDrDiffStd >20 && !pMode->staticData.staticFlag && kf_pvt->kfHeadingVel > 1.0)
		{
			p->downtownCnt  = p->downtownCnt + 2;
		}
		else if(p->smoothDiffStd > 65 && p->smoothPrDrDiffStd >18 && !pMode->staticData.staticFlag && kf_pvt->kfHeadingVel > 1.0)
		{
			p->downtownCnt ++;
		}
		else
		{
			p->downtownCnt = 0;
		}
#endif

		if(g_pe_cfg.chipType == QCOM)
		{
			if(p->smoothDiffStd > 95 && p->smoothPrDrDiffStd >30 && !pMode->staticData.staticFlag)
			{
				p->downtownCnt = p->downtownCnt +2 ;
			}
			else if(p->smoothDiffStd > 80 && p->smoothPrDrDiffStd >20)		
			{
				p->downtownCnt ++;
			}
			else
			{
				p->downtownCnt = 0;
			}

			if(p->downtownCnt >= 8)
			{	
				p->isDownTown = TRUE;
				p->noDowntownCnt = 0;
			}
		}
		else
		{
			if(p->smoothDiffStd > 100 && !pMode->staticData.staticFlag)
			{
				p->downtownCnt = p->downtownCnt +2 ;
			}
			else if(p->smoothDiffStd > 80)		
			{
				p->downtownCnt ++;
			}
			else
			{
				p->downtownCnt = 0;
			}

			if(p->downtownCnt >= 10)
			{	
				p->isDownTown = TRUE;
				p->noDowntownCnt = 0;
			}
		}
	}
	else/* if(p->isDownTown)*/
	{   
#if 0
		if(p->smoothDiffStd > 0 && p->smoothDiffStd < 50 && p->smoothPrDrDiffStd && p->smoothPrDrDiffStd < 15 )
		{
			p->noDowntownCnt =  p->noDowntownCnt + 2;
		}
		else if(p->smoothDiffStd > 0 && p->smoothDiffStd < 60 && p->smoothPrDrDiffStd && p->smoothPrDrDiffStd < 17.5 )
		{
			p->noDowntownCnt ++ ;
		}
		else
		{
			p->noDowntownCnt = 0;
		}
#endif 
		/*if(p->smoothDiffStd > 0 && p->smoothDiffStd < 60 && p->smoothPrDrDiffStd && p->smoothPrDrDiffStd < 15.0 )
		{
		p->noDowntownCnt = p->noDowntownCnt + 2;
		}
		else*/ 
		if(g_pe_cfg.chipType == QCOM)
		{
			if(p->smoothDiffStd > 0 && p->smoothDiffStd < 70 && p->smoothPrDrDiffStd && p->smoothPrDrDiffStd < 17.5 )
			{
				p->noDowntownCnt ++ ;
			}
			else
			{
				p->noDowntownCnt = 0;
			}

			if(pMode->tunnelMode.mode|| (p->smoothDiffStd < 0 && p->smoothPrDrDiffStd < 0))
			{
				p->noDowntownCnt = 8;
			}

			if(p->noDowntownCnt >= 8)
			{   
				p->isDownTown = FALSE;
				p->downtownCnt = 0;
			}
		}
		else
		{
			if(p->smoothDiffStd > 0 && p->smoothDiffStd < 70)
			{
				p->noDowntownCnt ++ ;
			}
			else
			{
				p->noDowntownCnt = 0;
			}

			if(pMode->tunnelMode.mode|| (p->smoothDiffStd < 0 && p->smoothPrDrDiffStd < 0))
			{
				p->noDowntownCnt = 10;
			}

			if(p->noDowntownCnt >= 10)
			{   
				p->isDownTown = FALSE;
				p->downtownCnt = 0;
			}
		}
	}
}

static void gnss_UnderEle_Det_QCOM(meas_blk_t* pMeas,PE_MODES* pMode)
{
	static uint8_t          cnt1 = 0,cnt2 = 0;
	USER_SCENARIO*     p;
	GNSS_TIME*         pTime;

	p = &(pMode->userSceData);
	pTime = gnss_tm_get_time();

	if (pTime->init == FALSE || pMode->tunnelMode.mode)
	{
		cnt1 = 0;
		cnt2 = 0;
		p->isUnderEleRoad = FALSE;
		return;
	}

	if(g_pe_cfg.chipType == QCOM)
	{
		if (!p->isUnderEleRoad)
		{
			cnt2++;
			if (pMeas->avgCno < 20)
			{
				cnt1 += 2;
			}else if (pMeas->avgCno <= 23)
			{
				cnt1++;
			}
			if ((double)cnt1/((double)cnt2) >= 0.75 && cnt2 >= 8)
			{
				p->isUnderEleRoad = TRUE;
				cnt1 = 0;
				cnt2 = 0;
			}else if (cnt2 >= 16)
			{
				cnt1 = 0;
				cnt2 = 0;
			}
		}else
		{
			cnt2++;
			if (pMeas->avgCno >= 30)
			{
				cnt1 += 2;
			}else if (pMeas->avgCno >= 25)
			{
				cnt1++;
			}
			if ((double)cnt1/((double)cnt2) >= 0.75 && cnt2 >= 8)
			{
				p->isUnderEleRoad = FALSE;
				cnt1 = 0;
				cnt2 = 0;
			}else if (cnt2 >= 16)
			{
				cnt1 = 0;
				cnt2 = 0;
			}
		}
	}
	else
	{
		if (!p->isUnderEleRoad)
		{
			cnt2++;
			if (pMeas->avgCno <= 32)
			{
				cnt1++;
			}
			if ((double)cnt1/((double)cnt2) >= 0.75 && cnt2 >= 8)
			{
				p->isUnderEleRoad = TRUE;
				cnt1 = 0;
				cnt2 = 0;
			}else if (cnt2 >= 16)
			{
				cnt1 = 0;
				cnt2 = 0;
			}
		}else
		{
			cnt2++;
			if (pMeas->avgCno >= 40)
			{
				cnt1 += 2;
			}else if (pMeas->avgCno >= 36)
			{
				cnt1++;
			}
			if ((double)cnt1/((double)cnt2) >= 0.75 && cnt2 >= 8)
			{
				p->isUnderEleRoad = FALSE;
				cnt1 = 0;
				cnt2 = 0;
			}else if (cnt2 >= 16)
			{
				cnt1 = 0;
				cnt2 = 0;
			}
		}
	}
	//SYS_LOGGING(OBJ_QOS,LOG_INFO,"%s,%10.4f,%02d,%02d,%02d,%d",__FUNCTION__,pMeas->tor,pMeas->avgCno,cnt1,cnt2,p->isUnderEleRoad);
}
/***********************************************************************
* ��������: gnss_PDRMode_Det
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/30/
***********************************************************************/
void gnss_PDRMode_Det(PE_MODES* pMode)
{
	pMode->userSceData.isWALKMODE = FALSE;

#if 1 //todo: DR def changed
	if (pMode->userContext.context != USER_CONTEXT_VEHICLE)
	{
		pMode->userSceData.isWALKMODE = TRUE;
	}
#endif
}
/***********************************************************************
* ��������: gnss_Scenario_Detect_QCOM
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/30/
***********************************************************************/
void gnss_Scenario_Detect_QCOM(meas_blk_t* pMeas,PE_MODES* pMode,KF_PVT_INFO* kf_pvt)
{
	//GNSS_TIME*    pTime;
	USER_SCENARIO*     p;
	/*UtcTimeType   utcTime;

	pTime = gnss_tm_get_time();
	utcTime = pTime->utcTime;*/

	p = &(pMode->userSceData);
	p->isOpenSky = FALSE;
	/* If average cno is larger than 40dB-Hz and measurement count is larger than 10,
	set open sky flag
	*/
	if(g_pe_cfg.chipType == QCOM)
	{
		if (pMeas->avgCno >= 30 && pMeas->measCnt >= 10 && pMeas->isSmallDiff == TRUE)
		{
			p->isOpenSky = TRUE;
		}
	}
	else
	{
		if (pMeas->avgCno >= 38 && pMeas->measCnt >= 8 && pMeas->isSmallDiff == TRUE)
		{
			p->isOpenSky = TRUE;
		}
	}

	/* PDR mode detection */
	if(g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
	{
		gnss_PDRMode_Det(pMode);
	}

	/* under ele detection */
	gnss_UnderEle_Det_QCOM(pMeas,pMode);

	/* Downtown scenario detection */
	gnss_DownTown_Det_QCOM(pMeas,pMode);

	//SYS_LOGGING(OBJ_QOS,LOG_INFO,"Downtown mode detection: %14.4f,%14.4f,%14.4f,%d,%d,%02d,%02d,%02d",pMeas->tor,p->smoothDiffStd,p->smoothPrDrDiffStd,pMeas->avgCno,p->isDownTown,utcTime.Hour,utcTime.Minute,(uint32_t)(utcTime.Second));

	SYS_LOGGING(OBJ_QOS,LOG_INFO,"%s,%14.4f,%d,%d %d %d",__FUNCTION__,pMeas->tor,pMeas->isSmallDiff,p->isWALKMODE,p->isUnderEleRoad,p->isDownTown);
}

/***********************************************************************
* ��������: gnss_Scenario_Detect_SPRD
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/30/
***********************************************************************/
void gnss_Scenario_Detect_SPRD(meas_blk_t* pMeas,PE_MODES* pMode,KF_PVT_INFO* kf_pvt)
{
	//GNSS_TIME*    pTime;
	USER_SCENARIO*     p;
	/*UtcTimeType   utcTime;

	pTime = gnss_tm_get_time();
	utcTime = pTime->utcTime;*/

	p = &(pMode->userSceData);

	if (kf_pvt->kfHeadingVel < 2.0)
	{
		p->isLowSpeed = TRUE;
	}else
	{
		p->isLowSpeed = FALSE;
	}

	p->isOpenSky = FALSE;
	/* If average cno is larger than 30dB-Hz and measurement count is larger than 10,
	set open sky flag
	*/
	if (pMeas->avgCno >= 30 && pMeas->measCnt >= 10 && pMeas->isSmallDiff == TRUE)
	{
		p->isOpenSky = TRUE;
	}

	p->isUnderEleRoad = FALSE;
	if (pMeas->avgCno < 25 && !pMode->tunnelMode.mode)
	{
		p->isUnderEleRoad = TRUE;
	}

	/* Downtown mode detection
	*/
	if(pMeas->prDiffStd_1 < 0)
	{
		p->diffBackCnt = 0;
		p->smoothDiffStd = -1.0;
	}
	else
	{
		gnss_math_data_smooth(pMeas->prDiffStd_1,p->prDiffStdBack,&(p->diffBackCnt),5,&(p->smoothDiffStd));
	}

	if(!p->isDownTown)
	{
		if(p->smoothDiffStd > 100 && !pMode->staticData.staticFlag)		
		{
			p->downtownCnt = p->downtownCnt +2;
		}
		if(p->smoothDiffStd > 80 && !pMode->staticData.staticFlag)		
		{
			p->downtownCnt ++;
		}
		else
		{
			p->downtownCnt = 0;
		}

		if(p->downtownCnt >= 20)
		{	
			p->isDownTown = TRUE;
			p->noDowntownCnt = 0;
		}
	}
	else/* if(p->isDownTown)*/
	{
		if(p->smoothDiffStd > 0 && p->smoothDiffStd < 55 && pMeas->prDiffStd_1 <55 )
		{
			p->noDowntownCnt ++ ;
		}
		else
		{
			p->noDowntownCnt = 0;
		}

		if(pMode->tunnelMode.mode)
		{
			p->noDowntownCnt = 20;
		}

		if(p->noDowntownCnt >= 20)
		{
			p->isDownTown = FALSE;
			p->downtownCnt = 0;
		}
	}

	//SYS_LOGGING(OBJ_QOS,LOG_INFO,"Downtown mode detection: %14.4f,%14.4f,%14.4f,%d,%d,%02d,%02d,%02d",pMeas->tor,p->smoothDiffStd,pMeas->prDiffStd_1,pMeas->avgCno,p->isDownTown,utcTime.Hour,utcTime.Minute,(uint32_t)(utcTime.Second));

	SYS_LOGGING(OBJ_QOS,LOG_INFO,"%s,%14.4f,%d,%d %d %d",__FUNCTION__,pMeas->tor,pMeas->isSmallDiff,p->isOpenSky,p->isUnderEleRoad,p->isDownTown);
}
/**********************************************************************
* Function Name:    gnss_StaticDoplVar_Calc
*
* Description:
*    Doppler variance calculation in static case
*
* Input:
*     measData:     measurement and SV related information
*     lsPvtInfo:    current Ls result
*
*Output:
*
*Return:
*     DoplVar:      Doppler variance
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
static float gnss_StaticDoplVar_Calc(LS_PVT_INFO* lsPvtInfo, meas_blk_t* pMeas)
{
	uint8_t            i, j, drNum = 0;
	uint8_t            maxDisId;
	float           calDopl;
	double           varTemp = 0;
	float           deltaDoplVar = -1;
	float           deltaDoplAvg = 0;
	float           maxDis;
	float           deltaDopl[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0.0};
	float           distance[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM] = {0.0};
	gnss_meas_t*  pSvMeas;
	DOP_TYPE      dop;

	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &pMeas->meas[i];

		if (pSvMeas->status & 0x2)
		{
			calDopl = (float)(-(pSvMeas->sv_info.dcos[0] * pSvMeas->sv_info.v[0] +
				pSvMeas->sv_info.dcos[1] * pSvMeas->sv_info.v[1] +
				pSvMeas->sv_info.dcos[2] * pSvMeas->sv_info.v[2]));

			deltaDopl[drNum] = (float)(calDopl - pSvMeas->pseudoRangeRate + lsPvtInfo->clkDrift);
			deltaDoplAvg += deltaDopl[drNum];
			drNum++;
		}
	}

	if (drNum < 4)
	{
		return deltaDoplVar;
	}

	gnss_Pe_Dop(pMeas, &dop, 2);
	if ((dop.pDOP > 4.0) || FLT_IS_EQUAL(dop.pDOP, 0.0f))
	{
		return deltaDoplVar;
	}

	//find the worst doppler
	for (i = 0; i < drNum; i++)
	{
		for (j = 0; j < drNum; j++)
		{
			distance[i] += (float)fabsf(deltaDopl[i] - deltaDopl[j]);
		}

		//if (drNum > 1)
		{
			distance[i] /= (drNum - 1);
		}
	}

	maxDis = distance[0];
	maxDisId = 0;

	for (i = 1; i < drNum; i++)
	{
		if (distance[i] > maxDis)
		{
			maxDis = distance[i];
			maxDisId = i;
		}
	}

	//average deltaDopl and deltaDopl variance
	//if (drNum > 1)
	{
		deltaDoplAvg -= deltaDopl[maxDisId];
		deltaDoplAvg /= (drNum - 1);

		for (i = 0; i < drNum; i++)
		{
			if (i == maxDisId)
			{
				continue;
			}

			varTemp += ((double)deltaDopl[i] - deltaDoplAvg) * ((double)deltaDopl[i] - deltaDoplAvg);
		}

		deltaDoplVar = (float)sqrt(varTemp / (drNum - 1));
	}

	return deltaDoplVar;
}

static uint8_t gnss_static_feedback(const uint8_t staticFlag, const meas_blk_t* pMeas)
{
	uint8_t            uVdrfeedback = 0;
	uint8_t            uGnssstaticFlag = staticFlag;
	uint8_t					   uEventReturn = FALSE;
	uint8_t            uFlag = 0;
	USER_PVT           user;

#if defined(PLAYBACK_MODE)
	double             ref_vel = -1.0;
	double             RefDis[3] = { 0 };
	static double      Lastrefpos[3] = { 0 };
	gtime_t            gtutcTime;
	int64_t            timeStampOfUtc = 0;
	GNSS_TIME*         pTime;
#endif

	if (pMeas == NULL)
	{
		return uEventReturn;
	}
	gnss_Pe_Get_PosFix(&user);
	uVdrfeedback = gnss_Pe_Ins_Vel_Check(&vdrFeedbackInfo, VDR_FEEDBACK_STATIC, &user);


	if (uVdrfeedback)
	{
		if (vdrFeedbackInfo.uOdoValid)  // 
		{
			uEventReturn = vdrFeedbackInfo.vdrZuptFlag;
			uFlag = 2;
		}
		else
		{
			if (vdrFeedbackInfo.vdrZuptFlag == TRUE)
			{
				uEventReturn = vdrFeedbackInfo.vdrZuptFlag;
				uFlag = 3;
			}
			else
			{
				if (staticFlag &&vdrFeedbackInfo.fAcc1sDelta < 2e-1 &&vdrFeedbackInfo.fGyro1sDelta < 2e-4)
				{
					uEventReturn = TRUE;
					if (vdrFeedbackInfo.ustationary_WhyT2F >= 3)
					{
						uEventReturn = FALSE;
						uFlag = 4;
					}
				}
				else
				{
					uEventReturn = FALSE;
					uFlag = 5;
				}
			}
		}
	}
	else
	{
		uEventReturn = staticFlag;
		uFlag = 6;
	}
	GLOGD("VDR flag:%d", uFlag);
#if defined(PLAYBACK_MODE)
	pTime = gnss_tm_get_time();
	gtutcTime = gpst2utc(pTime->GPSTime);
	timeStampOfUtc = (long long)((long long)gtutcTime.time * 1000 + gtutcTime.sec * 1000);
	if (pMeas->hpRefQflag[1])
	{
		ref_vel = sqrt(pMeas->hpRefLla[3] * pMeas->hpRefLla[3] + pMeas->hpRefLla[4] * pMeas->hpRefLla[4]);
		RefDis[2] = gnssCalPosDis(pMeas->hpRefLla, Lastrefpos, 1);
		memcpy(Lastrefpos, pMeas->hpRefLla, sizeof(Lastrefpos));
	}

	GLOGI("ADR GNSS static:%8.3f, %d, %d, %d, %d, %d, %d, %6.5f, %6.5f, %d, %6.5f, %9.7f %9.7f %d %d",
		pTime->rcvr_time[GPS_MODE], timeStampOfUtc - vdrFeedbackInfo.timeStampOfUtc, uVdrfeedback,
		uGnssstaticFlag, uEventReturn, vdrFeedbackInfo.vdrZuptFlag, vdrFeedbackInfo.vdrZuptFlagImu, ref_vel, RefDis[2], vdrFeedbackInfo.uOdoValid, vdrFeedbackInfo.fOdospeed,
		vdrFeedbackInfo.fGyro1sDelta, vdrFeedbackInfo.fAcc1sDelta,  vdrFeedbackInfo.ustationary_WhyT2F,	uFlag);
#endif
	return uEventReturn;
}
/***********************************************************************
* ��������: gnss_Static_Mode_Detection
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/04/
***********************************************************************/
void  gnss_Static_Mode_Detection(Kf_t* p_Kf, PE_MODES* pMode)
{
	uint8_t                exitCnt,i,Cno35Cnt = 0;
	uint8_t                uVdrfeedback = FALSE;
	uint8_t                uGnssstaticFlag = FALSE;
	float                  lsSpeed, deltaDoplVar, lsVel3D = 0.0, kfVel3D = 0.0;
	float                  speedTh_1, speedTh_2, speedTh_3,speedTh_4,varTh_1, varTh_2,varTh_3;
	float                  lsENUvel[3];
	KF_PVT_INFO*           kfPvtInfo;
	LS_PVT_INFO*           lsPvtInfo;
	meas_blk_t*            pMeas;
	STATIC_DATA*           staticData;
#if defined(PLAYBACK_MODE)
	GNSS_TIME*             pTime;
#endif
	USER_PVT               pvt;

	lsPvtInfo = p_Kf->ls_Pvt_Info;
	kfPvtInfo = p_Kf->kf_Pvt_Info;
	pMeas = p_Kf->meas_blk;
	staticData = &pMode->staticData;

#if defined(PLAYBACK_MODE)
	pTime = gnss_tm_get_time();
#endif

	gnss_Pe_Get_PosFix(&pvt);

	if ((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == 0)
	{
		return;
	}

	/* If have no LS position fix, clear static status */
	if (pvt.ecef.have_position == POS_FRM_NULL)
	{
		memset(&(pMode->staticData),0,sizeof(STATIC_DATA));
		return;
	}

	//ls speed
	if(lsPvtInfo->valid)
	{
		gnssConvEcef2EnuVel(lsPvtInfo->ecefVel, lsENUvel, kfPvtInfo->kfLLApos);
		lsSpeed = gnssClcSqrtSum_FLT(lsENUvel, 2);
		lsVel3D = gnssClcSqrtSum_FLT(lsPvtInfo->ecefVel, 3);
		kfVel3D = gnssClcSqrtSum_FLT(kfPvtInfo->ecefVel, 3);
	}
	else
	{
		lsSpeed = 0.0;
	}

	for (i = 0; i < GNSS_MAX_MODE; i++)
	{
		Cno35Cnt += pMeas->Cno35Cnt[i];
	}
	//Doppler variance in static case
	deltaDoplVar = gnss_StaticDoplVar_Calc(lsPvtInfo,pMeas);

#if 0
	//lowSpeedMode detection
	if (lsSpeed > 0 && p_Kf->kfCnt > 10 && staticData->staticFlag == FALSE)
	{
		if (kfPvtInfo->kfHeadingVel > 1.0)
		{
			if (staticData->avgCnt < 15)
			{
				staticData->avgCnt++;
				staticData->avgVel = staticData->avgVel * (((float)staticData->avgCnt - (float)1.0) / (float)staticData->avgCnt) + kfPvtInfo->kfHeadingVel * ((float)1.0 / (float)staticData->avgCnt);
			}
			else
			{
				staticData->avgVel = staticData->avgVel * (float)(14.0 / 15.0) + kfPvtInfo->kfHeadingVel * (float)(1.0 / 15.0);
			}
		}

		if (staticData->avgVel < 2.0)
		{
			staticData->lowSpeedMode = TRUE;
		}
		else
		{
			staticData->lowSpeedMode = FALSE;
		}

		SYS_LOGGING(OBJ_KF,LOG_INFO,"Low speed mode: %d", kfStaticData->lowSpeedMode);
	}
#endif
	//static constrain detection
	speedTh_1 = (float)0.3;
	speedTh_2 = (float)0.6;
	speedTh_3 = (float)0.2;
	speedTh_4 = (float)0.3;

	if (g_pe_cfg.automobile)
	{
		/*not weak signal scene*/
		if (pMeas->avgCno >= 30 && Cno35Cnt >= 10)
		{
			varTh_1 = (float)0.2;
			varTh_2 = (float)0.4;
			varTh_3 = (float)0.3;
		}
		else
		{
			varTh_1 = (float)0.35;
			varTh_2 = (float)0.55;
			varTh_3 = (float)0.45;
		}
		// TODO: Need refine threshold for weak signal
	}else
	{
		varTh_1 = (float)0.2;
		varTh_2 = (float)0.4;
		varTh_3 = (float)0.3;
		// TODO: Need refine threshold for weak signal 
	}

	if(pMode->userSceData.isUnderEleRoad && 
		pMode->userSceData.isPDRMode == FALSE)
	{
		speedTh_1 = (float)0.38;
		speedTh_2 = (float)0.98;
		speedTh_3 = (float)0.6;
		speedTh_4 = (float)1;
		if (pMode->userSceData.LowAvgCnocnt >= 4)
		{
			varTh_1 = (float)1.25;
			varTh_2 = (float)1.0;
			varTh_3 = (float)1.35;
		}
		else
		{
			varTh_1 = (float)0.50;
			varTh_3 = (float)0.55;
		}
	}

	/* Enter static detection */ 
	if (staticData->staticFlag == FALSE)
	{
		if (lsSpeed > 0)
		{
			if(pMode->userSceData.isUnderEleRoad && g_pe_cfg.chipType == UBLOX && 
				(deltaDoplVar > 0 && deltaDoplVar < varTh_1/2.0 && lsSpeed < speedTh_2/2.0))
			{
				staticData->speedStaticCnt = staticData->speedStaticCnt + 2;
			}
			else if ((deltaDoplVar > 0 && deltaDoplVar < varTh_1 && lsSpeed < speedTh_2) || (lsSpeed < speedTh_1))
			{
				staticData->speedStaticCnt++;
			}
			else if(pMode->userSceData.LowAvgCnocnt < 4)
			{
				staticData->speedStaticCnt = 0;
			}
			if (pMode->userSceData.LowAvgCnocnt >= 4)
			{
				if (deltaDoplVar > 0 && deltaDoplVar < varTh_1 / 2)
				{
					staticData->speedStaticCnt = staticData->speedStaticCnt + 4;
				}
				else if (deltaDoplVar > 0 && deltaDoplVar < varTh_1)
				{
					staticData->speedStaticCnt = staticData->speedStaticCnt + 2;
				}
				else
				{
					staticData->speedStaticCnt = 0;
				}
			}	
		}
		else if (deltaDoplVar < varTh_1 && deltaDoplVar > 0.0)
		{
			staticData->speedStaticCnt++;
		}
		else
		{
			staticData->speedStaticCnt = 0;
		}

		if (staticData->speedStaticCnt > 5)
		{
			staticData->speedStaticCnt = 5;
		}

		if(pMode->userSceData.isUnderEleRoad)
		{
			if (staticData->speedStaticCnt >= 4)
			{
				staticData->staticFlag = TRUE;
				staticData->speedDynamicCnt = 0;
			}
		}
		else
		{
			if (staticData->speedStaticCnt >= 2)
			{
				staticData->staticFlag = TRUE;
				staticData->speedDynamicCnt = 0;
			}
		}
	} else
	{
		/* Exit static detection */
		if (lsSpeed > 0)
		{
			if (deltaDoplVar > varTh_3 && lsSpeed > 1.0)
			{
				staticData->speedDynamicCnt = staticData->speedDynamicCnt + 3;
			}
			else if (deltaDoplVar > varTh_3 && lsSpeed > speedTh_4)
			{
				staticData->speedDynamicCnt = staticData->speedDynamicCnt+2;
			}
			if (deltaDoplVar > varTh_1 && lsSpeed > speedTh_3)
			{
				staticData->speedDynamicCnt++;
			}
			else if((staticData->historyStatic & 0x3) == 0x3 && lsSpeed > speedTh_3 && deltaDoplVar > varTh_1/2.0)
			{
				staticData->speedDynamicCnt = staticData->speedDynamicCnt + 2;
			}
			else
			{
				staticData->speedDynamicCnt  = 0;
			}
		}
		else if (deltaDoplVar > varTh_2)
		{
			staticData->speedDynamicCnt++;
		}
		else
		{
			staticData->speedDynamicCnt = 5;
		}

		if (staticData->speedDynamicCnt > 5)
		{
			staticData->speedDynamicCnt = 5;
		}

		if (pMode->indoorMode.mode && staticData->avgVel < 2 && staticData->avgVel != 0.0)
		{
			exitCnt = 3;
		}
		else
		{
			exitCnt = 2;
		}

		if (staticData->speedDynamicCnt >= exitCnt)
		{
			staticData->staticFlag = FALSE;
			staticData->speedStaticCnt = 0;
			staticData->staticHeading = 0;
		}
	}

	staticData->lsVel = lsSpeed;
	staticData->deltaDoplVar = deltaDoplVar;
	staticData->Vel3D = lsVel3D < kfVel3D ? lsVel3D : kfVel3D;
#if defined(PLAYBACK_MODE)
	SYS_LOGGING(OBJ_KF,LOG_INFO,"static constrain detect: tor = %16.8f, kfCnt = %05d, lsSpeed = %10.6f, doplVar = %8.4f, cno = %02d,realVel = %8.4f,staticCnt = %d, dynamicCnt = %d, staticFlag = %d,heading = %6.2f,minVel3D = %.3f",
		pTime->rcvr_time[GPS_MODE], p_Kf->kfCnt, lsSpeed, deltaDoplVar, pMeas->avgCno,realVel,staticData->speedStaticCnt, staticData->speedDynamicCnt, staticData->staticFlag,kfPvtInfo->kfHeadingDir * RAD2DEG, staticData->Vel3D);
#endif

	kfPvtInfo->staticFlag = staticData->staticFlag = gnss_static_feedback(staticData->staticFlag, pMeas);

	//kfPvtInfo->staticFlag = staticData->staticFlag;
	// save history static flag
	staticData->historyStatic = (staticData->historyStatic<<1) | (staticData->staticFlag &0x1);
	staticData->historyStaticLong = (staticData->historyStaticLong << 1) | (staticData->staticFlag & 0x1);
	if ((staticData->historyStatic & 0x3) == 0x1&&
		kfPvtInfo->kfFixStatus == FIX_STATUS_NEW)
	{
		p_Kf->errBack = kfPvtInfo->kfErrEst;
		p_Kf->enPosCovarBack = kfPvtInfo->enPosCovar;
	}
}
/***********************************************************************
* ��������: gnss_Mode_Detection
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/04/
***********************************************************************/
void gnss_Mode_Detection(meas_blk_t* pMeas,PE_MODES* pMode,KF_PVT_INFO* kf_pvt)
{
	/* 1. Tunnel Mode */
	gnss_Tunnel_Mode(pMeas,pMode,kf_pvt);

	/* 2. Indoor Mode */
	gnss_Indoor_Mode(pMeas,pMode);

	/* 3. User context detection */
	gnss_Motion_Detect(pMeas,pMode,kf_pvt);

	/* 4. user scenario detection */
	if(g_pe_cfg.chipType == QCOM || g_pe_cfg.chipType == UBLOX)
	{
		gnss_Scenario_Detect_QCOM(pMeas,pMode,kf_pvt);
	}
	else if (g_pe_cfg.chipType == SPRD)
	{
		gnss_Scenario_Detect_SPRD(pMeas,pMode,kf_pvt);
	}
}
