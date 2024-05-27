/************************************************************
* Copyrights(C) 
* All rights Reserved
* 文件名称：gnss_ls_raim.c
* 文件描述：最小二乘RAIM算法
* 版本号：1.0.0
* 作者：
* 日期：08/29
************************************************************/
#include "gnss_ls.h"
#include "gnss_math.h"
#include "gnss_sys_api.h"
#include "gnss_mode.h"
#include "gnss_pe.h"
#include "gnss_sd.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_LS

extern Gnss_Cfg_t g_pe_cfg;
extern uint8_t firstFix;
extern PE_MODES peMode;
/***********************************************************************
* 函数介绍: gnss_Ls_RaimThres_Chck
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/03/
***********************************************************************/
uint8_t gnss_Ls_RaimThres_Chck(Ls_t* pLs, uint8_t lsType,double* thresHold)
{
	int32_t       rdn;
	double       Table_Pos[6] = {50.0, 45.0, 40.0, 35.0,30.0,25.0};
 	double       Table_Vel[5] = {1.6,  1.4,  1.2,  0.8, 0.5};
	double       FD_THRESHOLD;
	uint8_t        Cno35Cnt = 0,i;

	for (i = 0; i < GNSS_MAX_MODE; i++)
	{
		Cno35Cnt += pLs->meas->Cno35Cnt[i];
	}
	/* Decrease Table_Pos in relatively good scene for ST_8100*/
	if (!firstFix  && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
	{
		if (pLs->meas->Cno45Cnt*1.0 > 0.3*Cno35Cnt)
		{
			Table_Pos[3] = 25.0;
			Table_Pos[4] = 20.0;
			Table_Pos[5] = 15.0;
			if (pLs->meas->avgCno > 35 && Cno35Cnt >= 15 && pLs->meas->Cno45Cnt*1.0 > 0.6*Cno35Cnt && pLs->meas->Cno45Cnt > 5)
			{
				Table_Pos[5] = 10.0;
			}
		}
		else if (pLs->meas->Cno45Cnt*1.0 > 0.1*Cno35Cnt && pLs->meas->Cno45Cnt > 2)
		{
			Table_Pos[3] = 30.0;
			Table_Pos[4] = 25.0;
			Table_Pos[5] = 20.0;
		}
	}
	if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 ||
		g_pe_cfg.sub_chipType == SUB_CHIPTYPE_QCOM_855)
	{
		Table_Pos[3] = 20.0;
		Table_Pos[4] = 15.0;
		Table_Pos[5] = 8.0;
		if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 && pLs->meas->avgCno > 35 && Cno35Cnt >= 15 && 
			pLs->meas->Cno45Cnt*1.0  > 0.6*Cno35Cnt && pLs->meas->Cno45Cnt > 10)
		{
			Table_Pos[5] = 7.5;
		}
	}
	if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 && pLs->meas->avgCno > 35 && Cno35Cnt >= 15 && 
		pLs->meas->Cno45Cnt*1.0  > 0.7*Cno35Cnt && pLs->meas->Cno45Cnt > 10)
	{
		Table_Pos[3] = 15.0;
		Table_Pos[4] = 10.0;
		Table_Pos[5] = 6.0;
		if (pLs->meas->Cno45Cnt*1.0 / Cno35Cnt > 0.85)
		{
			Table_Pos[5] = 5.0;
		}
	}

	if (lsType == LS_POS_ID)
	{
		if (pLs->lsCtrl.is2DPos)
		{
			rdn = (int32_t) pLs->lsCtrl.prNum - pLs->lsCtrl.lsBiasNum - 2+ pLs->lsCtrl.lsDcbNum[0] +pLs->lsCtrl.lsDcbNum[1];
		}else
		{
			rdn = (int32_t) pLs->lsCtrl.prNum - pLs->lsCtrl.lsBiasNum - 3 + pLs->lsCtrl.lsDcbNum[0] +pLs->lsCtrl.lsDcbNum[1];
		}
		if (rdn > 12)
		{
			FD_THRESHOLD = Table_Pos[5];
		}
		else if (rdn > 8)
		{
			FD_THRESHOLD = Table_Pos[4];
		}
		else if (rdn > 4)
		{
			FD_THRESHOLD = Table_Pos[3];
		}else if(rdn >= 1)
		{
			FD_THRESHOLD = Table_Pos[rdn - 1];
		}
		else
		{
			return FALSE;
		}
		if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 && pLs->meas->avgCno > 35 && Cno35Cnt > 12 &&
			pLs->meas->Cno45Cnt*1.0 < 0.2*Cno35Cnt)
		{
			if ((pLs->pos_res < FD_THRESHOLD / 2 && rdn >= 10) || (pLs->pos_res < FD_THRESHOLD && rdn > 12) ||
				(pLs->pos_res < FD_THRESHOLD / 2 && pLs->pos_res > FD_THRESHOLD / 3 && rdn >= 8))
			{
				FD_THRESHOLD = 7.5;	
			}
		}
		else if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 && pLs->meas->avgCno > 35 && Cno35Cnt >= 15 &&
			pLs->meas->Cno45Cnt*1.0 > 0.3*Cno35Cnt && pLs->meas->Cno45Cnt*1.0 < 0.6*Cno35Cnt)
		{
			if (pLs->pos_res < FD_THRESHOLD && rdn >= 21 && pLs->DoP.pDOP < 2.0)
			{
				FD_THRESHOLD = 6.5;
			}
			else if (pLs->pos_res < FD_THRESHOLD && rdn >= 18 && pLs->DoP.pDOP < 2.0)
			{
				FD_THRESHOLD = 7.0;
			}
		}
		/* firstFix FD_THRESHOLD adjust in good scene(Cno45Cnt > 5 or 10) for ST_8100*/
		if (!firstFix && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100) && pLs->meas->avgCno > 35 && Cno35Cnt >= 15 &&
			pLs->meas->Cno45Cnt*1.0 > 0.3*Cno35Cnt && pLs->meas->Cno45Cnt*1.0 < 0.6*Cno35Cnt)
		{
			if (pLs->pos_res < FD_THRESHOLD && rdn >= 21 && pLs->DoP.pDOP < 2.0 && pLs->meas->Cno45Cnt > 10)
			{
				FD_THRESHOLD = 7.5;
			}
			else if (pLs->pos_res < FD_THRESHOLD && rdn >= 21 && pLs->DoP.pDOP < 2.0 && pLs->meas->Cno45Cnt > 5)
			{
				FD_THRESHOLD = 10.0;
			}
		}
		if (pLs->pos_res > FD_THRESHOLD)
		{
			if (thresHold)
			{
				(*thresHold) = FD_THRESHOLD;
			}       
			return TRUE;
		}else
		{
			return FALSE;
		}      
	}else
	{
		/* Check the freedom number */
		if (pLs->lsCtrl.is2DVel)
		{
			rdn = (int32_t)pLs->lsCtrl.drNum - 3;
		}
		else
		{
			rdn = (int32_t)pLs->lsCtrl.drNum - 4;
		}
		/* chose the threshold */
		if (rdn > 5)
		{
			FD_THRESHOLD  = Table_Vel[4] ;
		}
		else if (rdn >= 1)
		{
			FD_THRESHOLD  = Table_Vel[rdn - 1] ;
		}
		else
		{
			return FALSE;
		}

		if (pLs->vel_res > FD_THRESHOLD)
		{
			if (thresHold)
			{
				(*thresHold) = FD_THRESHOLD;
			}
			return TRUE;
		}else
		{
			return FALSE;
		}
	}
}

void gnss_Raim_Sort_EqualRes(Ls_t* pLs, uint8_t gnssMode, int8_t freq)
{
	uint8_t				temp_pos_indx;
	uint32_t				i, /*j,*/ cnt = 0, indx[2] = { 0 }, indx1, indx2;
	float				temp_pos_res;
	gnss_meas_t*	pSv0, *pSv1;
	meas_blk_t*     pMeas;

	pMeas = pLs->meas;

	for (i = 0; i < pLs->raimCtrl.res_cnt; i++)
	{
		indx1 = pLs->raimCtrl.pos_indx[i];
		indx2 = pLs->lsCtrl.prUsedSV[indx1];

		if (freq == -1)
		{
			if ((cnt < 2) && (pMeas->meas[indx2].gnssMode == gnssMode))
			{
				indx[cnt] = i;
				cnt++;
			}
		}
		else
		{
			if ((cnt < 2) && (pMeas->meas[indx2].gnssMode == gnssMode) && (pMeas->meas[indx2].freq_index == freq))
			{
				indx[cnt] = i;
				cnt++;
			}
		}
	}

	if ((cnt == 2) && (fabsf(pLs->raimCtrl.pos_res[indx[0]] - pLs->raimCtrl.pos_res[indx[1]]) < 0.001))
	{
		indx1 = pLs->raimCtrl.pos_indx[indx[0]];
		indx2 = pLs->lsCtrl.prUsedSV[indx1];
		pSv0 = &(pMeas->meas[indx2]);

		indx1 = pLs->raimCtrl.pos_indx[indx[1]];
		indx2 = pLs->lsCtrl.prUsedSV[indx1];
		pSv1 = &(pMeas->meas[indx2]);

		if (pSv0->cno > pSv1->cno)
		{
			temp_pos_indx = pLs->raimCtrl.pos_indx[indx[0]];
			pLs->raimCtrl.pos_indx[indx[0]] = pLs->raimCtrl.pos_indx[indx[1]];
			pLs->raimCtrl.pos_indx[indx[1]] = temp_pos_indx;

			temp_pos_res = pLs->raimCtrl.pos_res[indx[0]];
			pLs->raimCtrl.pos_res[indx[0]] = pLs->raimCtrl.pos_res[indx[1]];
			pLs->raimCtrl.pos_res[indx[1]] = temp_pos_res;

			SYS_LOGGING(OBJ_LS, LOG_INFO, "sort the pos_res according to cn0 for equal res:%d %d, %d %d", pSv0->gnssMode, pSv0->prn, pSv1->gnssMode, pSv1->prn);
		}
		else if (pSv0->cno == pSv1->cno)
		{
			if (pSv0->sv_info.fltElev > pSv1->sv_info.fltElev)
			{
				temp_pos_indx = pLs->raimCtrl.pos_indx[indx[0]];
				pLs->raimCtrl.pos_indx[indx[0]] = pLs->raimCtrl.pos_indx[indx[1]];
				pLs->raimCtrl.pos_indx[indx[1]] = temp_pos_indx;

				temp_pos_res = pLs->raimCtrl.pos_res[indx[0]];
				pLs->raimCtrl.pos_res[indx[0]] = pLs->raimCtrl.pos_res[indx[1]];
				pLs->raimCtrl.pos_res[indx[1]] = temp_pos_res;

				SYS_LOGGING(OBJ_LS, LOG_INFO, "sort the pos_res according to elev for equal res:%d %d, %d %d", pSv0->gnssMode, pSv0->prn, pSv1->gnssMode, pSv1->prn);
			}
		}
	}
}

/*Distance Raim Judge*/
void gnss_Ls_Distance_Raim_Judge(Ls_t* pLs, uint8_t sat_index)
{
	uint8_t               /*rej_indx,*/ indx, indx1;
	uint32_t              i;
	uint8_t               flag = FALSE;
	gnss_meas_t*     pSvMeas;

	indx = sat_index;
	indx1 = pLs->lsCtrl.prUsedSV[indx];
	pSvMeas = &(pLs->meas->meas[indx1]);
	/* Adjust the PR used SV list */
	pLs->lsCtrl.prNum--;
	for (i = indx; i < pLs->lsCtrl.prNum; i++)
	{
		pLs->lsCtrl.prUsedSV[i] = pLs->lsCtrl.prUsedSV[i + 1];
	}

	/* Clear PR status ,Remove the detected error PR */
	pSvMeas->status &= 0xFE;

	gnss_Ls_Pos(pLs);
	flag = gnss_Ls_RaimThres_Chck(pLs, LS_POS_ID, NULL);
	pLs->lsCtrl.isPrRaim = flag;

	SYS_LOGGING(OBJ_LS, LOG_INFO, "PR RAIM Distance: tor = %14.6f,gnssMode = %d,PRN = %03d(cno=%02d) was rejected,PRDiff = %10.4f,PrRaimFlag = %d", pLs->meas->tor, pLs->meas->meas[indx1].gnssMode,
		pLs->meas->meas[indx1].prn, pLs->meas->meas[indx1].cno, pLs->meas->meas[indx1].pr_diff, pLs->lsCtrl.isPrRaim);
}

/***********************************************************************
* 函数介绍: gnss_Ls_Pos_Raim_Judge
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/03/
***********************************************************************/
void gnss_Ls_Pos_Raim_Judge(Ls_t* pLs)
{
	uint8_t               rej_indx;
	uint8_t               indx,indx1;
	uint8_t               flag = FALSE,flag1 = FALSE;
	uint8_t               gnssMode;
	uint32_t              i;
	uint32_t              cnoThres;
	float              avg_res = 0,min_res = 0,secMin_res = 0,std_res = 0.0;
	float              ratio1 = 0.0,ratio2 = 0.0;
	gnss_meas_t*     pSvMeas;
	//Ls_t             lsTmp;
	/* STEP1: check the residual count after reject satellite one by one */
	if (pLs->raimCtrl.res_cnt == 0)
	{
		pLs->raimCtrl.prRaimFlag &= (~LS_RAIM_NEED_MORE); 
		SYS_LOGGING(OBJ_LS,LOG_INFO,"Can't find any post-res in RAIM at %s(%d)",__FUNCTION__,__LINE__);
		return;
	}else if (pLs->raimCtrl.res_cnt == 1)
	{
		flag = TRUE;
		rej_indx = 0;
		SYS_LOGGING(OBJ_LS,LOG_INFO,"Find only one post-res in RAIM at %s(%d)",__FUNCTION__,__LINE__);
		goto PR_RAIM_REJECT_SV;
	} 

	/* STEP2: sort the pos_res */
	gnss_Sort_WithIndx(pLs->raimCtrl.pos_res,pLs->raimCtrl.pos_indx,pLs->raimCtrl.res_cnt);

	//sort equal pos_res when one system only have two meas to avoid difference between Windows and Linux
	for (gnssMode = 0; gnssMode < GNSS_MAX_MODE; gnssMode++)
	{
		if (pLs->lsCtrl.validSvNum[gnssMode] == 2)
		{
			gnss_Raim_Sort_EqualRes(pLs, gnssMode, -1);
		}
		else if (pLs->lsCtrl.lsDcbCalFlag[gnssMode] || pLs->lsCtrl.lsDcbCalFlag[gnssMode + GNSS_MAX_MODE])
		{
			if ((pLs->lsCtrl.validSvNum[gnssMode] - pLs->meas->fre2_satcnt[gnssMode] == 2) || 
				(pLs->lsCtrl.validSvNum[gnssMode] - pLs->meas->fre2_satcnt[gnssMode + GNSS_MAX_MODE] == 2))
			{
				gnss_Raim_Sort_EqualRes(pLs, gnssMode, 0);
			}
			else if (pLs->meas->fre2_satcnt[gnssMode] == 2)
			{
				gnss_Raim_Sort_EqualRes(pLs, gnssMode, 1);
			}
			else if (pLs->meas->fre2_satcnt[gnssMode + GNSS_MAX_MODE] == 2)
			{
				gnss_Raim_Sort_EqualRes(pLs, gnssMode, 2);
			}
		}
	}

	/* STEP3: Find out the most potential error PR according to (1)post residual,
	(2) cno, (3) MP flag, (4) HDOP */
	for (i = 0; i < pLs->raimCtrl.res_cnt;i++)
	{
		avg_res += pLs->raimCtrl.pos_res[i];
	}
	avg_res /= pLs->raimCtrl.res_cnt;
	min_res = pLs->raimCtrl.pos_res[0];
	secMin_res = pLs->raimCtrl.pos_res[1];
	indx = pLs->raimCtrl.pos_indx[0];
	indx1 = pLs->lsCtrl.prUsedSV[indx];
	pSvMeas = &(pLs->meas->meas[indx1]);

	/* SNR based condition */
	for (i = 0; i < pLs->raimCtrl.res_cnt; i++)
	{
		std_res += (pLs->raimCtrl.pos_res[i] - avg_res) * (pLs->raimCtrl.pos_res[i] - avg_res);
	}
	std_res = (float)sqrt(std_res / pLs->raimCtrl.res_cnt);
	if (secMin_res != avg_res)
	{
		ratio1 = (float)fabsf(min_res - avg_res) / (float)fabsf(secMin_res - avg_res);
	}
	else
	{
		ratio1 = avg_res / min_res;
	}
	ratio2 = (float)fabsf(min_res - avg_res) / std_res;
	/*
	1. If minimal residual is half of original residual and cno is less than 34, then reject this satellite
	2. If min_res and secMin_res are both smaller than 35, then chose min_res or secMin_res reject
	3. If (min_res - avg_res) / (secMin_res - avg_res) > 2.0, then reject satellite with smallest res
	4. Sequential RAIM can detect bad PR with those very obvious residual offset
	*/
	if(g_pe_cfg.automobile)
	{
		cnoThres = 45;
	}
	else
	{
		/*for phone data*/
		if (!firstFix && (peMode.userContext.context != USER_CONTEXT_VEHICLE))
		{
			if (min_res > 12 && pLs->meas->avgCno > 35 && pLs->pos_res> 15.0)
			{
				cnoThres = 45;
			}
			else if (min_res > 8.0 && pLs->meas->avgCno > 35 && pLs->pos_res> 10.0)
			{
				cnoThres = 38;
			}
			else
			{
				cnoThres = 35;
			}
		}
		else
		{
			cnoThres = 35;
		}
	}
	if (pLs->raimCtrl.res_cnt > 0 && pLs->posCvg == FALSE)
	{
		flag = TRUE;
		rej_indx = 0;
		goto PR_RAIM_REJECT_SV;
	}   
	else if (min_res < (pLs->pos_res / 2.0) && pSvMeas->cno < cnoThres)
	{
		flag = TRUE;
		rej_indx = 0;
		goto PR_RAIM_REJECT_SV;
	}
	else if (ratio1 > 2.0 && ratio2 > 1.8 && pSvMeas->cno < cnoThres)
	{
		flag = TRUE;
		rej_indx = 0;
		SYS_LOGGING(OBJ_LS, LOG_INFO, "PRSNRCHECK:%10.4f,%10.4f,%10.4f", ratio1, ratio2, pSvMeas->pr_diff);
		goto PR_RAIM_REJECT_SV;
	}
	else if (ratio1 > 8.0 && ratio2 > 4.0 && ((double)secMin_res - min_res) > 30.0)
	{
		flag = TRUE;
		rej_indx = 0;
		SYS_LOGGING(OBJ_LS, LOG_INFO, "PRSNRCHECK:%10.4f,%10.4f,%10.4f", ratio1, ratio2, pSvMeas->pr_diff);
		goto PR_RAIM_REJECT_SV;
	}
	else if (ratio1 > 50.0 && ratio2 > 4.0 && pSvMeas->cno >= cnoThres && g_pe_cfg.automobile)
	{
		if((pSvMeas->sv_info.fltElev > 0.0) && (pSvMeas->sv_info.fltElev < 20.0))
		{ 
			flag = TRUE;
			rej_indx = 0;
			SYS_LOGGING(OBJ_LS, LOG_INFO, "PRSNRCHECK:%10.4f,%10.4f,%10.4f", ratio1, ratio2, pSvMeas->pr_diff);
			goto PR_RAIM_REJECT_SV;
		}
	}
	else if (min_res < 35 && secMin_res < 35)
	{
		if (((fabsf(pLs->pos_res - min_res) > 60.0f)&&pSvMeas->cno < (cnoThres+5) )||
			(pSvMeas->cno < cnoThres))
		{
			flag = TRUE;
			rej_indx = 0;
			goto PR_RAIM_REJECT_SV;
		}else
		{
			indx = pLs->raimCtrl.pos_indx[1];
			indx1 = pLs->lsCtrl.prUsedSV[indx];
			pSvMeas = &(pLs->meas->meas[indx1]);
			if ((fabsf(pLs->pos_res - secMin_res) > 60.0f&&pSvMeas->cno < (cnoThres+5)) || 
				(pSvMeas->cno < cnoThres))
			{
				rej_indx = 1;
				flag = TRUE;
				goto PR_RAIM_REJECT_SV;
			}
		}
	}





	/* STEP4: Remove the found error PR */
PR_RAIM_REJECT_SV:
	if (flag)
	{
		indx = pLs->raimCtrl.pos_indx[rej_indx];
		indx1 = pLs->lsCtrl.prUsedSV[indx];
		pSvMeas = &(pLs->meas->meas[indx1]);
		/* Adjust the PR used SV list */
		pLs->lsCtrl.prNum--;
		for (i = indx; i < pLs->lsCtrl.prNum;i++)
		{
			pLs->lsCtrl.prUsedSV[i] = pLs->lsCtrl.prUsedSV[i+1];
		}
		/*
		if (pLs->lsCtrl.prNum > indx)
		{
		memcpy(&pLs->lsCtrl.prUsedSV[indx],&pLs->lsCtrl.prUsedSV[indx+1],sizeof(uint8_t) * (pLs->lsCtrl.prNum - indx));
		}*/
		/* Clear DR status */
		pSvMeas->status &= 0xFE;
		/* 
		(1). Remove the detected error PR
		(2). Calculate position again
		*/
		//memcpy(&lsTmp,pLs,sizeof(Ls_t));
		gnss_Ls_Pos(pLs);
		flag1 = gnss_Ls_RaimThres_Chck(pLs, LS_POS_ID,NULL);
		pLs->lsCtrl.isPrRaim = flag1;

		pLs->raimCtrl.prRaimFlag |= LS_RAIM_FOUND_ERROR;

		SYS_LOGGING(OBJ_LS,LOG_INFO,"PR RAIM: tor = %14.6f,gnssMode = %d,PRN = %03d(cno=%02d) was rejected, min_res = %10.4f,sec_res = %10.4f,PRDiff = %10.4f",pLs->meas->tor,pLs->meas->meas[indx1].gnssMode,
			pLs->meas->meas[indx1].prn,pLs->meas->meas[indx1].cno,min_res,secMin_res,pLs->meas->meas[indx1].pr_diff);
	}
	/* STEP5: Check if need a new round PR RAIM */
	if ((pLs->lsCtrl.prNum < 6) || (flag == FALSE))
	{
		pLs->raimCtrl.prRaimFlag &= (~LS_RAIM_NEED_MORE); 
	}else
	{
		if (flag1 == TRUE)
		{   
			pLs->raimCtrl.prRaimFlag |= LS_RAIM_NEED_MORE;
		}else
		{
			pLs->raimCtrl.prRaimFlag &= (~LS_RAIM_NEED_MORE);
		}
	}
}

/***********************************************************************
* 函数介绍: gnss_Ls_Pos_Raim
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：11/18/
***********************************************************************/
void gnss_Ls_Distance_Raim(Ls_t* pLs)
{
	uint8_t              cvg;
	uint32_t             i,j = 0,k;
	double             dLat,dLon;	

	LLA* llaData = (LLA*)Sys_Malloc(sizeof(LLA)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	uint8_t*  index = (uint8_t*)Sys_Malloc(sizeof(uint8_t)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	double* distance = (double*)Sys_Malloc(sizeof(double)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	Ls_t* pLsRaim = (Ls_t*)Sys_Malloc(sizeof(Ls_t));
	if ((llaData == NULL) || (index == NULL) || (distance == NULL)||(pLsRaim == NULL))
	{
		Sys_Free(llaData); Sys_Free(index); Sys_Free(distance); Sys_Free(pLsRaim); 
		return;
	}
	memset(llaData, 0, sizeof(LLA)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	memset(index, 0, sizeof(uint8_t)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	memset(distance, 0, sizeof(double)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	memset(pLsRaim, 0, sizeof(Ls_t));

	for (i = 0; i < pLs->lsCtrl.prNum;i++)
	{
		/* 1). copy the last epoch LS results */
		memcpy(pLsRaim,pLs,sizeof(Ls_t));

		/* 2). remove the ith satellites */
		pLsRaim->lsCtrl.prNum--;
		for (k = i; k < pLsRaim->lsCtrl.prNum;k++)
		{
			pLsRaim->lsCtrl.prUsedSV[k] = pLsRaim->lsCtrl.prUsedSV[k+1];
		}
		//memcpy(&(pLsRaim->lsCtrl.prUsedSV[i]),&(pLsRaim->lsCtrl.prUsedSV[i+1]),sizeof(uint8_t) * (pLsRaim->lsCtrl.prNum - i));

		/* 3). LS with removing one satellites */
		cvg = gnss_Ls_Pos(pLsRaim);

		if (cvg == FALSE)
		{
			continue;
		}

		index[j] = i;
		llaData[j].lla[0] = pLsRaim->lsLla.lla[0];
		llaData[j].lla[1] = pLsRaim->lsLla.lla[1];
		llaData[j++].lla[2] = pLsRaim->lsLla.lla[2];
	}

	for (i = 0; i < j;i++)
	{
		for (k = 0;k < j;k++)
		{
			if (i == k) continue;
			dLat = (llaData[i].lla[0] - llaData[k].lla[0]) * (180 / PI) * 111319;
			dLon = (llaData[i].lla[1] - llaData[k].lla[1]) * (180 / PI) * 111133;
			distance[i] += sqrt(dLat*dLat + dLon*dLon);
		}
        if (j > 1)
        {
            distance[i] /= (j - 1);
        }
	}
	SYS_LOGGING(OBJ_LS,LOG_INFO,"Exit %s",__FUNCTION__);
	Sys_Free(llaData); Sys_Free(index); Sys_Free(distance); Sys_Free(pLsRaim); 

}
/***********************************************************************
* 函数介绍: gnss_Ls_Pos_Raim_SatSys
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/13/
***********************************************************************/
uint8_t gnss_Ls_Pos_Raim_SatSys(Ls_t* pLs)
{
	uint8_t             i,j,k, flag = FALSE, indx, prNumTmp;
	uint8_t             rejSatSys[GNSS_MAX_MODE]= {0};
	uint8_t             prUsedSVTmp[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM]; 
	meas_blk_t*    pMeas;
	gnss_meas_t*   pSvMeas;
	Ls_t           ls_raim;

	ls_raim = *pLs;
	pMeas = pLs->meas;

	if (ls_raim.lsCtrl.prNum < 3)
	{
		return flag;
	}

	for(j = 0;j < GNSS_MAX_MODE; j++)
	{   
		prNumTmp = 0;
		memset(prUsedSVTmp,0,MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
		//if LS position couldn't converge, try to reject one satellite system
		for (i = 0; i < ls_raim.lsCtrl.prNum; i++)
		{	
			indx = ls_raim.lsCtrl.prUsedSV[i];
			if(ls_raim.meas->meas[indx].gnssMode!= j)
			{
				prUsedSVTmp[prNumTmp++] = indx;
			}
		}

		memcpy(ls_raim.lsCtrl.prUsedSV,prUsedSVTmp,MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
		ls_raim.lsCtrl.prNum = prNumTmp;
		flag = gnss_Ls_Pos(&ls_raim);

		if(flag)
		{
			SYS_LOGGING(OBJ_LS,LOG_INFO,"Ls could converge after reject gnss mode:%02d",j);
			memcpy(pLs->lsCtrl.prUsedSV,ls_raim.lsCtrl.prUsedSV,MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM*sizeof(uint8_t));
			pLs->lsCtrl.prNum = ls_raim.lsCtrl.prNum ;
			pLs->lsCtrl.validSvNum[j] = 0;


			for(k = 0;k < pMeas->measCnt;k++)
			{
				pSvMeas = &(pMeas->meas[k]);
				if(pSvMeas->gnssMode == j)
				{
					pSvMeas->status &= 0xFE;
				}
			}
			//re-calculate LS position
			gnss_Ls_Pos(pLs);
			return flag;
		}
		else
		{
			ls_raim = *pLs;
		}
	}
	return flag;
}

/***********************************************************************
* 函数介绍: gnss_Ls_Pos_Raim
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/03/
***********************************************************************/
void gnss_Ls_Pos_Raim(Ls_t* pLs)
{
	uint8_t          cvg,/*indx,*/ biasLinkFlag,firstRaimThres = 0,Cno35Cnt = 0,foreRaimFlag = 0,forceRaimDistanceCnt = 0, forceRaimloopThres = 0;
	uint8_t          *p2 = NULL;
	uint8_t         i,j;
	
	double         dLat, dLon;
	float         *p1 = NULL, foreRaimPosResThres = 8.0;
	float         avgDistance, stdDistance, sec_avgDistance, sec_stdDistance, third_avgDistance, third_stdDistance;
	meas_blk_t* pMeas;
	//gnss_meas_t* pSvMeas;

	LLA* llaData; 
	uint8_t* index_raw; 
	float* distance; 
	Ls_t* pLsRaim;

	if (pLs->lsCtrl.prNum < 6)
	{
		pLs->lsCtrl.isPrRaim = FALSE;
		SYS_LOGGING(OBJ_LS, LOG_INFO, "PR number %d can't meet PR RAIM", pLs->lsCtrl.prNum);
		return;
	}

	llaData = (LLA*)Sys_Malloc(sizeof(LLA) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
	index_raw = (uint8_t*)Sys_Malloc(sizeof(uint8_t) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
	distance = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
	pLsRaim = (Ls_t*)Sys_Malloc(sizeof(Ls_t));
	if ((llaData == NULL) || (index_raw == NULL) || (distance == NULL)||(pLsRaim == NULL))
	{
		Sys_Free(llaData); Sys_Free(index_raw); Sys_Free(distance); Sys_Free(pLsRaim); 
		return;
	}
	memset(llaData, 0, sizeof(LLA)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	memset(index_raw, 0, sizeof(uint8_t)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	memset(distance, 0, sizeof(float)*MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM);
	memset(pLsRaim, 0, sizeof(Ls_t));
	
	pMeas = pLs->meas;
	biasLinkFlag = pLs->lsCtrl.isBiasLink;

	gnss_LS_BiasLink_Check(pLs);

	/* check the bias number */
	gnss_Ls_Check_BiasNum(pLs);

	/* LS Position */
	if (pLs->posCvg == FALSE || pLs->lsCtrl.is2DPos == TRUE)
	{
		cvg = gnss_Ls_Pos(pLs);
	}
	else
	{
		cvg = TRUE;
	}
	if (cvg && (!firstFix))
	{
		if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
		{
			gnss_ls_dynamic_ele_pre(pLs);
		}
	}
	if (!firstFix)
	{
		for (i = 0; i < GNSS_MAX_MODE; i++)
		{
			Cno35Cnt += pLs->meas->Cno35Cnt[i];
		}
		if (Cno35Cnt > 15 && pLs->meas->Cno45Cnt * 1.0 > 0.65 * Cno35Cnt && pLs->meas->Cno45Cnt > 10 && pLs->meas->avgCno > 35)
		{
			firstRaimThres = 10;
		}
		else if (Cno35Cnt > 15 && pLs->meas->Cno45Cnt * 1.0 > 0.75 * Cno35Cnt && pLs->meas->Cno45Cnt > 12 && pLs->meas->avgCno > 35)
		{
			firstRaimThres = 12;
		}
	}

	if (pLs->lsCtrl.prNum < (5 + pLs->lsCtrl.lsBiasNum))
	{
		pLs->lsCtrl.isPrRaim = FALSE;
		SYS_LOGGING(OBJ_LS,LOG_INFO,"PR number(prNum:%d,biasNum:%d) can't meet PR RAIM",pLs->lsCtrl.prNum,
			pLs->lsCtrl.lsBiasNum);
		Sys_Free(llaData); Sys_Free(index_raw); Sys_Free(distance); Sys_Free(pLsRaim); 
		return;
	}

	if (cvg == FALSE)
	{
		pLs->lsCtrl.isPrRaim = TRUE;       // LS position fail, then do RAIM 
	}
	else
	{
		pLs->lsCtrl.isPrRaim = gnss_Ls_RaimThres_Chck(pLs,LS_POS_ID,NULL);
		if (!pLs->lsCtrl.isPrRaim && !firstFix)
		{
			if (pLs->pos_res < foreRaimPosResThres && Cno35Cnt > 12)
			{
				foreRaimFlag = 1;
				SYS_LOGGING(OBJ_LS, LOG_INFO, "Force RAIM %d,pdop:%10.4f,pos_res:%10.4f", pLs->lsCtrl.isPrRaim, pLs->DoP.pDOP, pLs->pos_res);
			}
		}
	}

	if (pLs->lsCtrl.isPrRaim || foreRaimFlag)
	{
		pLs->raimCtrl.pos_res = (float *)Sys_Calloc(pLs->lsCtrl.prNum,sizeof(float));
		if (pLs->raimCtrl.pos_res == NULL)
		{
			Sys_Free(llaData); Sys_Free(index_raw); Sys_Free(distance); Sys_Free(pLsRaim); 
			return;
		}
		pLs->raimCtrl.pos_indx = (uint8_t *)Sys_Calloc(pLs->lsCtrl.prNum,sizeof(int8_t));
		if (pLs->raimCtrl.pos_indx == NULL)
		{
			Sys_Free(pLs->raimCtrl.pos_res);
			pLs->raimCtrl.pos_res = NULL;
			Sys_Free(llaData); Sys_Free(index_raw); Sys_Free(distance); Sys_Free(pLsRaim); 
			return;
		}
		pLs->raimCtrl.loop_cnt = 0;
POS_RAIM_FLOW:
		p1 = pLs->raimCtrl.pos_res;
		p2 = pLs->raimCtrl.pos_indx;
		forceRaimDistanceCnt = 0;
		pLs->raimCtrl.res_cnt = 0;
		pLs->raimCtrl.loop_cnt++;
		/* 1. RAIM detection value calculation */
		for (i = 0; i < pLs->lsCtrl.prNum;i++)
		{
			/* 1). copy the last epoch LS results */
			memcpy(pLsRaim,pLs,sizeof(Ls_t));
			gnss_LS_BiasLink_Check(pLsRaim);

			/* 2). remove the i-th satellites */
			pLsRaim->lsCtrl.prNum--;
			for (j = i; j < pLsRaim->lsCtrl.prNum;j++)
			{
				pLsRaim->lsCtrl.prUsedSV[j] = pLsRaim->lsCtrl.prUsedSV[j+1];
			}
			/*
			if (pLsRaim->lsCtrl.prNum > i)
			{
			memcpy(&(pLsRaim->lsCtrl.prUsedSV[i]),&(pLsRaim->lsCtrl.prUsedSV[i+1]),sizeof(uint8_t) * (pLsRaim->lsCtrl.prNum - i));
			}
			*/
			//indx = pLs->lsCtrl.prUsedSV[i];

			/* 3). LS with removing one satellites */
			cvg = gnss_Ls_Pos(pLsRaim);
			if ((cvg == FALSE) || ((pLsRaim->DoP.pDOP / pLs->DoP.pDOP) > 2.0 && pLsRaim->DoP.pDOP > 4.0) ||
				(pLsRaim->DoP.pDOP > 6.0 && pLs->DoP.pDOP < 4.0))
			{
				if (cvg == TRUE)
				{
					/*1.use raim_pdop and ls_pdop to judge,ls_pdop > 3.0 &&raim_pdop/ls_pdop > 2.0
					  2.use raim_pdop/pos_res and ls_pdop to judge,raim_pos_res > 1.5 &&raim_pdop/ls_pdop > 2.0
					*/
					if (pLs->DoP.pDOP > 3.0 && (pLsRaim->DoP.pDOP / pLs->DoP.pDOP) > 2.0)
					{
						continue;
					}
					else if ((pLsRaim->DoP.pDOP / pLs->DoP.pDOP) > 2.0 && pLsRaim->pos_res > 1.5)
					{
						continue;
					}

				}
				else
				{
					continue;
				}
			}
			llaData[pLs->raimCtrl.res_cnt].lla[0] = pLsRaim->lsLla.lla[0];
			llaData[pLs->raimCtrl.res_cnt].lla[1] = pLsRaim->lsLla.lla[1];
			llaData[pLs->raimCtrl.res_cnt].lla[2] = pLsRaim->lsLla.lla[2];
			(*p1++) = pLsRaim->pos_res;
			(*p2++) = i;
			index_raw[pLs->raimCtrl.res_cnt] = i;/*store index used for Distance_Raim*/
			pLs->raimCtrl.res_cnt++;
		}

		/*Force Raim check*/
		if (foreRaimFlag && pLs->raimCtrl.res_cnt <= 7)
		{
			if (pLs->raimCtrl.pos_res)
			{
				Sys_Free(pLs->raimCtrl.pos_res);
				pLs->raimCtrl.pos_res = NULL;
			}
			if (pLs->raimCtrl.pos_indx)
			{
				Sys_Free(pLs->raimCtrl.pos_indx);
				pLs->raimCtrl.pos_indx = NULL;
			}
			Sys_Free(llaData); Sys_Free(index_raw); Sys_Free(distance); Sys_Free(pLsRaim); 
			return;
		}
		else if (foreRaimFlag && pLs->raimCtrl.res_cnt > 7)
		{
			for (i = 0; i < pLs->raimCtrl.res_cnt; i++)
			{
				for (j = 0; j < pLs->raimCtrl.res_cnt; j++)
				{
					if (i == j) continue;
					dLat = (llaData[i].lla[0] - llaData[j].lla[0]) * (180 / PI) * 111319;
					dLon = (llaData[i].lla[1] - llaData[j].lla[1]) * (180 / PI) * 111133;
					distance[i] += (float)sqrt(dLat*dLat + dLon * dLon);
				}
				distance[i] /= (j - 1);
			}
			gnss_Sort_WithIndx(distance, index_raw, pLs->raimCtrl.res_cnt);
			gnss_math_fstd(distance, pLs->raimCtrl.res_cnt, &avgDistance, &stdDistance);
			gnss_math_fstd(distance, pLs->raimCtrl.res_cnt - 1, &sec_avgDistance, &sec_stdDistance);
			gnss_math_fstd(distance, pLs->raimCtrl.res_cnt - 2, &third_avgDistance, &third_stdDistance);
			SYS_LOGGING(OBJ_LS, LOG_INFO, "FORCE RAIM DISTANCE INFO first:%10.16f,%10.16f,sec:%10.16f,%10.16f,third:%10.16f,%10.16f",
				avgDistance, stdDistance, sec_avgDistance, sec_stdDistance, third_avgDistance, third_stdDistance);
			for (j = pLs->raimCtrl.res_cnt; j > pLs->raimCtrl.res_cnt - 4; j--)
			{
				if (stdDistance / third_stdDistance > 2.5 && distance[j - 1] > 2.5 * third_avgDistance && distance[j - 1] > 2.0 * avgDistance)
				{
					forceRaimDistanceCnt++;
				}
			}
			if (forceRaimDistanceCnt > 0)
			{
				if (forceRaimDistanceCnt > 3)
				{
					forceRaimloopThres = 3;
				}
				else
				{
					forceRaimloopThres = 2;
				}

				/*2_0. Distance check, Reject largest distance sat*/
				if (stdDistance / third_stdDistance > 5.0 && distance[pLs->raimCtrl.res_cnt - 1] > 10)
				{
					gnss_Ls_Distance_Raim_Judge(pLs, index_raw[pLs->raimCtrl.res_cnt - 1]);
					SYS_LOGGING(OBJ_LS, LOG_INFO, "Start a new round POS Force RAIM");
					goto POS_RAIM_FLOW;
				}

				/* 2_1. PR RAIM detection logic */
				gnss_Ls_Pos_Raim_Judge(pLs);
				if ((pLs->raimCtrl.prRaimFlag & LS_RAIM_NEED_MORE) && pLs->raimCtrl.loop_cnt < forceRaimloopThres)
				{
					SYS_LOGGING(OBJ_LS, LOG_INFO, "Start a new round POS Force RAIM");
					goto POS_RAIM_FLOW;
				}
			}
		}
		else
		{
			/* 2. PR RAIM detection logic */
			gnss_Ls_Pos_Raim_Judge(pLs);

			if ((pLs->raimCtrl.prRaimFlag & LS_RAIM_NEED_MORE) &&
				(pLs->raimCtrl.loop_cnt < PR_RAIM_LOOP_MAX || (!firstFix && pLs->raimCtrl.loop_cnt < firstRaimThres)))
			{
				SYS_LOGGING(OBJ_LS, LOG_INFO, "Start a new round POS RAIM");
				goto POS_RAIM_FLOW;
			}
		}

		/* 3. Free the allocated memory */
		if (pLs->raimCtrl.pos_res)
		{
			Sys_Free(pLs->raimCtrl.pos_res);
			pLs->raimCtrl.pos_res = NULL;
		}
		if (pLs->raimCtrl.pos_indx)
		{
			Sys_Free(pLs->raimCtrl.pos_indx);
			pLs->raimCtrl.pos_indx = NULL;
		}
	}

	pLs->lsCtrl.isBiasLink = biasLinkFlag;

	/* Didn't found any error (unused code)*/
	/*
	if (((pLs->raimCtrl.prRaimFlag & LS_RAIM_FOUND_ERROR) == 0) && (pLs->lsCtrl.isPrRaim == FALSE))
	{
		gnss_Ls_Distance_Raim(pLs);
	}
	*/
	Sys_Free(llaData); Sys_Free(index_raw); Sys_Free(distance); Sys_Free(pLsRaim); 
}

/***********************************************************************
* 函数介绍: gnss_Ls_Vel_Raim_Judge
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/03/
***********************************************************************/
void gnss_Ls_Vel_Raim_Judge(Ls_t* pLs)
{
	uint8_t              indx,indx1;
	uint32_t             i;
	uint32_t             cnoThres;
	float             avg_res = 0,min_res = 0.0,secMin_res = 0.0;
	uint8_t              flag = FALSE,flag1 = FALSE;
	//Ls_t            lsTmp;
	gnss_meas_t*    pSvMeas;
	/* STEP1 : check the residual count after remove satellite one by one
	(1). If no post-res can be calculated when removing one satellite, then don't reject any satellite
	and step out DR RAIM;
	(2). 
	*/
	if (pLs->raimCtrl.res_cnt == 0)
	{
		pLs->raimCtrl.drRaimFlag &= (~LS_RAIM_NEED_MORE); 
		SYS_LOGGING(OBJ_LS,LOG_INFO,"Can't find any post-res in RAIM at %s(%d)",__FUNCTION__,__LINE__);
		return;
	}else if (pLs->raimCtrl.res_cnt == 1)
	{
		flag = TRUE;
		SYS_LOGGING(OBJ_LS,LOG_INFO,"Find only one post-res in RAIM at %s(%d)",__FUNCTION__,__LINE__);
		goto DR_RAIM_REJECT_SV;
	}else
	{
		SYS_LOGGING(OBJ_LS,LOG_INFO,"Find more than one post-res in RAIM at %s(%d %d)",__FUNCTION__,pLs->raimCtrl.res_cnt,__LINE__);
	}
	/* STEP2: sorting the residual */
	gnss_Sort_WithIndx(pLs->raimCtrl.vel_res,pLs->raimCtrl.vel_indx,pLs->raimCtrl.res_cnt);
	/* STEP3: Find the potential error DR */
	for (i = 0; i < pLs->raimCtrl.res_cnt; i++)
	{
		avg_res += pLs->raimCtrl.vel_res[i];
	}
	avg_res /= pLs->raimCtrl.res_cnt;

	min_res = pLs->raimCtrl.vel_res[0];
	secMin_res = pLs->raimCtrl.vel_res[1];
	/* STEP4: condition check  */
	if ( min_res < 0.5)
	{
		flag = TRUE;
	}else if((secMin_res != avg_res) && (fabs((min_res - avg_res) / (secMin_res - avg_res)) > 2))
	{
		if (avg_res > 0.3)
		{
			flag = TRUE;
		}else
		{
			flag = FALSE;
		}

	}else if (min_res < pLs->vel_res && pLs->raimCtrl.res_cnt >= 8)
	{
		flag = TRUE;
	}else if (min_res < 1.2 && pLs->vel_res > 1.5 && (pLs->vel_res / min_res) >= 1.8)
	{
		flag = TRUE;
	}else
	{
		flag = FALSE;
		SYS_LOGGING(OBJ_LS,LOG_INFO,"LS DR RAIM FAIL DETECT ERRORS:%10.5f,%10.5f,%10.5f",min_res,secMin_res,avg_res);
	}
	/* some DR need remove */
DR_RAIM_REJECT_SV:
	if (flag == TRUE)
	{
		indx = pLs->raimCtrl.vel_indx[0];
		indx1 = pLs->lsCtrl.drUsedSV[indx];

		if(g_pe_cfg.automobile)
		{
			cnoThres = 40;
		}
		else
		{
			cnoThres = 30;
		}

		if (pLs->meas->meas[indx1].prDiffValid == TRUE)
		{
			if (fabs(pLs->meas->meas[indx1].dr_diff) < 0.30 && pLs->meas->meas[indx1].drChckCnt == 0)
			{
				flag = FALSE;
				goto DR_RAIM_CHECK_MORE;
			}
			if (fabs(pLs->meas->meas[indx1].dr_diff) < 1.00 && pLs->meas->meas[indx1].cno >= cnoThres && 
				fabsf(min_res - avg_res) < 4.0f * fabsf(secMin_res - avg_res))
			{
				flag = FALSE;
				goto DR_RAIM_CHECK_MORE;
			}
			else if (fabsf(pLs->meas->meas[indx1].dr_diff) >= 1.00 && pLs->meas->meas[indx1].cno > cnoThres &&
				fabsf(min_res - avg_res) < 1.5 * fabsf(secMin_res - avg_res) && fabsf(pLs->meas->meas[indx1].dr_diff) < 1.5)
			{
				flag = FALSE;
				goto DR_RAIM_CHECK_MORE;
			}
		}
		/* Adjust the DR used SV list */
		pLs->lsCtrl.drNum--;
		for (i = indx; i < pLs->lsCtrl.drNum;i++)
		{
			pLs->lsCtrl.drUsedSV[i] = pLs->lsCtrl.drUsedSV[i+1];
		}
		/*
		if (pLs->lsCtrl.drNum > indx)
		{
		memcpy(&pLs->lsCtrl.drUsedSV[indx],&pLs->lsCtrl.drUsedSV[indx+1],sizeof(uint8_t) * (pLs->lsCtrl.drNum - indx));
		}*/
		/* Clear DR status */
		pSvMeas = &(pLs->meas->meas[indx1]);
		pSvMeas->status &= 0xFD;
		/* 
		(1). Remove the detected error DR
		(2). Calculate velocity again
		*/
		gnss_Ls_Vel(pLs,FALSE);
		flag1 = gnss_Ls_RaimThres_Chck(pLs, LS_VEL_ID,NULL);
		pLs->lsCtrl.isDrRaim = flag1;

		pLs->raimCtrl.drRaimFlag |= LS_RAIM_FOUND_ERROR;

		SYS_LOGGING(OBJ_LS,LOG_INFO,"DR RAIM: gnssMode = %02d,PRN = %02d(cno=%02d,DRDiff=%10.4f) was rejected",pLs->meas->meas[indx1].gnssMode,
			pLs->meas->meas[indx1].prn,pLs->meas->meas[indx1].cno,pLs->meas->meas[indx1].dr_diff);
	}
DR_RAIM_CHECK_MORE:
	/* STEP5: Check if need a new round DR RAIM */
	if ((pLs->lsCtrl.drNum < 6) || (flag == FALSE))
	{
		pLs->raimCtrl.drRaimFlag &= (~LS_RAIM_NEED_MORE); 
	}else
	{
		if (flag1 == TRUE)
		{   
			pLs->raimCtrl.drRaimFlag |= LS_RAIM_NEED_MORE;
		}else
		{
			pLs->raimCtrl.drRaimFlag &= (~LS_RAIM_NEED_MORE);
		}
	}
}


/***********************************************************************
* 函数介绍: gnss_Ls_Vel_Raim
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/03/
***********************************************************************/
void gnss_Ls_Vel_Raim(Ls_t* pLs)
{
	uint8_t        cvg,is2DVelBack;
	uint8_t        *p2 = NULL;
	uint8_t        loopLimit = DR_RAIM_LOOP_MAX;
	uint32_t       i,j;
	Ls_t      lsRaim,*pLsRaim;
	float       *p1 = NULL;

	pLsRaim = &lsRaim;

	/* If can't get 2D/3D position fix, then return */
	if (((pLs->lsCtrl.status & LS_STATUS_HAS_2D_POS) == 0)
		&& ((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS) == 0))
	{
		pLs->lsCtrl.isDrRaim = FALSE;
		SYS_LOGGING(OBJ_LS,LOG_INFO,"There is no LS POS for LS DR RAIM");
		return;
	}
	/* Only when drNum >= 6, then perform DR RAIM */
	if (pLs->lsCtrl.drNum < 6)
	{  
		pLs->lsCtrl.isDrRaim = FALSE;
		SYS_LOGGING(OBJ_LS,LOG_INFO,"DR number(drNum:%d) can't meet DR RAIM",pLs->lsCtrl.drNum);
		return;
	}

	is2DVelBack = pLs->lsCtrl.is2DVel;
	//do up vel hold under elevated road
	if(g_pe_cfg.chipType == UBLOX && peMode.userSceData.isUnderEleRoad)
	{
		pLs->lsCtrl.is2DVel = TRUE;
	}
	if (pLs->velCvg == FALSE)
	{
		cvg = gnss_Ls_Vel(pLs, FALSE);
	}
	else
	{
		cvg = TRUE;
	}
	if (cvg == FALSE)// can't converge, then do DR RAIM
	{
		pLs->lsCtrl.isDrRaim = TRUE;    
	}else // or compare the residual with threshold
	{
		pLs->lsCtrl.isDrRaim = gnss_Ls_RaimThres_Chck(pLs,LS_VEL_ID,NULL);
	}

	/* DR RAIM main loops */
	if (pLs->lsCtrl.isDrRaim)
	{
		pLs->raimCtrl.vel_res =(float *)Sys_Calloc(pLs->lsCtrl.drNum,sizeof(float));
		if (pLs->raimCtrl.vel_res == NULL)
		{
			return;
		}
		pLs->raimCtrl.vel_indx =(uint8_t *)Sys_Calloc(pLs->lsCtrl.drNum,sizeof(int8_t));
		if (pLs->raimCtrl.vel_indx == NULL)
		{
			Sys_Free(pLs->raimCtrl.vel_res);
			pLs->raimCtrl.vel_res = NULL;
			return;
		}
		pLs->raimCtrl.loop_cnt = 0;
#if 0
		/* adjust loop-limit number according to valid DR number 
		DR Number:6,7,8,9,10,11,12,13,14,15,...
		RejectLmt:1,2,3,4,4, 4, 4, 4, 4, 5, 5
		*/
		if (pLs->lsCtrl.drNum >= 15)
		{
			loopLimit = 5;
		}      
		else if (pLs->lsCtrl.drNum >= 10)
		{
			loopLimit = 4;
		}else if (pLs->lsCtrl.drNum >= 6)
		{
			loopLimit = pLs->lsCtrl.drNum - 5;
		}
#endif
VEL_RAIM_FLOW:
		p1 = pLs->raimCtrl.vel_res;
		p2 = pLs->raimCtrl.vel_indx;
		pLs->raimCtrl.res_cnt = 0;
		pLs->raimCtrl.loop_cnt++;

		/* 1. DR RAIM detection value calculation */
		for (i = 0; i < pLs->lsCtrl.drNum;i++)
		{
			/* 1). copy the last epoch LS results */
			memcpy(pLsRaim,pLs,sizeof(Ls_t));

			/* 2). LS position */
			//gnss_Ls_Pos(pLsRaim);

			/* 3). remove the i-th satellites */
			pLsRaim->lsCtrl.drNum--;
			for (j = i; j < pLsRaim->lsCtrl.drNum;j++)
			{
				pLsRaim->lsCtrl.drUsedSV[j] = pLsRaim->lsCtrl.drUsedSV[j+1];
			}
			/*
			if (pLsRaim->lsCtrl.drNum > i)
			{
			memcpy(&(pLsRaim->lsCtrl.drUsedSV[i]),&(pLsRaim->lsCtrl.drUsedSV[i+1]),sizeof(uint8_t) * (pLsRaim->lsCtrl.drNum - i));
			}
			*/
			/* 4). LS with removing one satellites */
			cvg = gnss_Ls_Vel(pLsRaim,TRUE);
			if (cvg == FALSE || pLsRaim->DoP.pDOP > 4.0)
			{
				continue;
			}
			(*p1++) = pLsRaim->vel_res;
			(*p2++) = i;
			pLs->raimCtrl.res_cnt++;
		}

		/* 2. DR RAIM detection logic */
		gnss_Ls_Vel_Raim_Judge(pLs);
		// adjust loop-limit number according to valid DR number
		if (pLs->lsCtrl.drNum >= 25)
		{
			loopLimit = 7;
		}
		else if (pLs->lsCtrl.drNum >= 20)
		{
			loopLimit = 6;
		}
		else if (pLs->lsCtrl.drNum >= 15)
		{
			loopLimit = 5;
		}else if (pLs->lsCtrl.drNum >= 11)
		{
			loopLimit = 4;
		}else if (pLs->lsCtrl.drNum >= 7)
		{
			loopLimit = 2;
		}

		if ((pLs->raimCtrl.drRaimFlag & LS_RAIM_NEED_MORE) && 
			pLs->raimCtrl.loop_cnt < loopLimit)
		{
			goto VEL_RAIM_FLOW;
		}

		pLs->lsCtrl.is2DVel = is2DVelBack;

		/* 3. Free the allocated memory */
		if (pLs->raimCtrl.vel_res)
		{
			Sys_Free(pLs->raimCtrl.vel_res);
			pLs->raimCtrl.vel_res = NULL;
		}
		if (pLs->raimCtrl.vel_indx)
		{
			Sys_Free(pLs->raimCtrl.vel_indx);
			pLs->raimCtrl.vel_indx = NULL;
		}
		SYS_LOGGING(OBJ_LS,LOG_INFO,"Finish DR RAIM(%d loops)", pLs->raimCtrl.loop_cnt);
	}
}


/***********************************************************************
* 函数介绍: gnss_Ls_Bias_Raim
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：2/28/
***********************************************************************/
void gnss_Ls_Bias_Raim(Ls_t* pLs)
{

}
/***********************************************************************
* 函数介绍: gnss_ls_dynamic_ele_pre
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：2020/02/017/
***********************************************************************/
void gnss_ls_dynamic_ele_pre(Ls_t* pLs)
{
	uint32_t            i,j;
	//sat_data_t*    sp;
	TRANS          tr;
	D_COS         dcos;
	double            range,toa;
	double            satPosCorr[3];
	double            dynamicElevationThreshold = 13;
	double            ave_elevation = 0.0;
	uint8_t             elevationCnt = 0, indx;
	double            elevationSum = 0.0;
	uint32_t            dynamic_cno_threshold = 0;
	meas_blk_t* pMeas;
	gnss_meas_t* pSvMeas;

	pMeas = pLs->meas;

	//cal elevation before fix
	for (i = 0; i < pMeas->measCnt; i++)
	{
		pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->prn == 0 ) continue;
		if ((pSvMeas->status & 0x1) == 0) continue;
		range = gnssClcSqrtAminusB_DBL(pLs->lsEcef.p, pSvMeas->sv_info.p, 3);
		//calculate the direction cosine:
		if (range > 0.0)
		{
			toa = range / LIGHT_SEC;
			gnssEarthRotateCorr(pSvMeas->sv_info.p, satPosCorr, toa);
			pSvMeas->range = gnssClcSqrtAminusB_DBL(pLs->lsEcef.p, satPosCorr, 3);
			if (DBL_IS_EQUAL(pSvMeas->range, 0.0)) continue;

			pSvMeas->sv_info.dcos[0] = ((satPosCorr[0]- pLs->lsEcef.p[0]) / pSvMeas->range);
			pSvMeas->sv_info.dcos[1] = ((satPosCorr[1]- pLs->lsEcef.p[1]) / pSvMeas->range);
			pSvMeas->sv_info.dcos[2] = ((satPosCorr[2]- pLs->lsEcef.p[2]) / pSvMeas->range);

			tr.et1 = -(float)sin(pLs->lsLla.lla[1]);   //etx
			tr.et2 = (float)cos(pLs->lsLla.lla[1]);    //ety
			tr.utz = (float)sin(pLs->lsLla.lla[0]);
			tr.ce = (float)cos(pLs->lsLla.lla[0]);    //ntz
#if 0
			tr.nt1 = -tr.utz * tr.et2;       //ntx
			tr.nt2 = tr.utz * tr.et1;       //nty
#endif
			tr.utx = tr.ce * tr.et2;
			tr.uty = -tr.ce * tr.et1;

			/*dcos.east_dcos = (float)(tr.et1 * pSvMeas->sv_info.dcos[0] + tr.et2 * pSvMeas->sv_info.dcos[1]);
			dcos.north_dcos = (float)(tr.nt1 * pSvMeas->sv_info.dcos[0] + tr.nt2 * pSvMeas->sv_info.dcos[1] +
				tr.ce * pSvMeas->sv_info.dcos[2]);*/
			/*  compute elevation angle */
			dcos.sin_ea = (float)(pSvMeas->sv_info.dcos[0] * tr.utx +
				pSvMeas->sv_info.dcos[1] * tr.uty +
				pSvMeas->sv_info.dcos[2] * tr.utz);
			dcos.elev = (float)gnss_asinf(dcos.sin_ea);
			pSvMeas->sv_info.fltElev = (float)(dcos.elev * RAD2DEG);
			// elevation sum
			elevationSum += pSvMeas->sv_info.fltElev;
			elevationCnt++;
			/*SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s %02d %03d ele %f %d %d %f",__FUNCTION__, pSvMeas->gnssMode,
				pSvMeas->prn, pSvMeas->sv_info.fltElev, pSvMeas->cno, pLs->lsCtrl.prNum, pSvMeas->sumPRDRDiff);*/
		}
	}
	// there is no satellites with positive elevation angle
	if (elevationCnt < 6)
	{
		return;
	}


	ave_elevation = elevationSum / (double)elevationCnt;

	if (ave_elevation < 30.0)
	{
		dynamicElevationThreshold = dynamicElevationThreshold + 1;
	}
	else if (ave_elevation >= 30.0 && ave_elevation < 40.0)
	{
		dynamicElevationThreshold = dynamicElevationThreshold + 3;
	}
	else if (ave_elevation >= 40.0 && ave_elevation < 50.0)
	{
		dynamicElevationThreshold = dynamicElevationThreshold + 5;
	}
	else/* if (ave_elevation >= 50.0)*/
	{
		dynamicElevationThreshold = dynamicElevationThreshold + 6;
	}
	//GLOGW("debug_ele thres %f ave_elevation %f %d", dynamicElevationThreshold, ave_elevation, pLs->meas->avgCno);

	if (pLs->meas->avgCno > 35 && pLs->lsCtrl.prNum>20)
	{
		dynamic_cno_threshold = pLs->meas->avgCno;
		if (dynamic_cno_threshold > 40)
		{
			dynamic_cno_threshold = 40;
		}
	}
	else
	{
		dynamic_cno_threshold = 35;
	}
	for (i = 0; i < pLs->lsCtrl.prNum; i++)
	{
		indx = pLs->lsCtrl.prUsedSV[i];
		// If lower than elevation threshold and cno is lower than 35, mark this satellite
		if (pLs->meas->meas[indx].sv_info.fltElev <= dynamicElevationThreshold)
		{
			if (pLs->meas->meas[indx].cno <= dynamic_cno_threshold)
			{
				if ((ave_elevation - pLs->meas->meas[indx].sv_info.fltElev) > 20.0)
				{
					//reject PR
					pLs->lsCtrl.prNum--;
					pLs->meas->meas[indx].status &= 0xFE;
					for (j = i; j < pLs->lsCtrl.prNum; j++)
					{
						pLs->lsCtrl.prUsedSV[j] = pLs->lsCtrl.prUsedSV[j + 1];
						
					}
					i--;
					SYS_LOGGING(OBJ_QOS, LOG_INFO, "%02d %03d %f %d %d reject because big ele_gap", pLs->meas->meas[indx].gnssMode,
						pLs->meas->meas[indx].prn, pLs->meas->meas[indx].sv_info.fltElev, pLs->meas->meas[indx].cno, pLs->lsCtrl.prNum);
				}
			}
		}
	}
}
