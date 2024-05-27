/************************************************************
* Copyrights(C) 
* All rights Reserved
* �ļ����ƣ�gnss_sd.c
* �ļ����������ǵ�״̬�����ݹ���
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�09/23
************************************************************/
#include "gnss.h"
#include "gnss_sd.h"
#include "gnss_sd_nm.h"
#include "gnss_pe.h"
#include "gnss_sys_api.h"
#include "gnss_tm.h"
#include "gnss_math.h"
#include "gnss_config.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_SD

Sd_t                    Sd_data;
extern Gnss_Cfg_t		g_pe_cfg;
extern meas_blk_t*      gpz_Meas;

#if 0
extern Rtcm_data_t       g_rtcm_data;
#endif

extern void gnss_Pe_Get_PosFix(USER_PVT* pvt);
/**********************************************************************
* Function Name:    gnss_sd_svch_get
*
* Description:
*    AP to get satellite's channel number. This API will get the channel number of satellites.
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int16_t gnss_sd_svch_get(uint8_t gnssMode, uint8_t sv, uint32_t freq_index)
{
	uint8_t            i, sat = 0;
	int16_t           ch = -1;
	sat_data_t*    sp;

	/* SV ID check */
	if (gnssMode == GPS_MODE)
	{
		if ((!sv)
			|| ((sv > MAX_GPS_PRN) && (sv < MIN_WAAS_PRN))
			|| ((sv > MAX_WAAS_PRN) && (sv < MIN_QZSS_PRN))
			|| (sv > MAX_QZSS_PRN))
		{
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"gnssMode(%02d) prn(%03d)check fail",gnssMode,sv);
			return -2;
		}
	}
	else if (gnssMode == GLN_MODE)
	{
		if ((!sv)  || (sv > MAX_GLN_PRN))
		{
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"gnssMode(%02d) prn(%03d)check fail",gnssMode,sv);
			return -2;
		}
	}
	else if (gnssMode == BDS_MODE)
	{
		if (!sv || sv > MAX_BDS_PRN)
		{
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"gnssMode(%02d) prn(%03d)check fail",gnssMode,sv);
			return -2;
		}
	}
	else if (gnssMode == GAL_MODE)
	{
		if (!sv || sv > MAX_GAL_PRN)
		{
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"gnssMode(%02d) prn(%03d)check fail",gnssMode,sv);
			return -2;
		}
	}

	sat = satno(GNSS_MODE2SYS(gnssMode, sv), sv);
	if (sat <= 0) return -2;
	i = Sd_data.sat_data_idx[sat - 1][freq_index];
	sp = Sd_data.sat_data[i];

	if (sp != NULL && sp->gnssMode == gnssMode && sp->prn == sv && sp->freq_index == freq_index)
	{
		ch = i;
	}

#if 0
	/* search for sv in channels */
	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM; i++)
	{
		sp = Sd_data.sat_data[i];

		if (sp == NULL)
		{
			continue;
		}

		if (sp->gnssMode == gnssMode && sp->prn == sv && sp->freq_index == freq_index)
		{
			ch = i;
			break;
		}
	}
#endif

	return ch;
}

uint8_t gnss_sd_prn_check(uint8_t gnssMode,int16_t prn)
{
	if (gnssMode == GPS_MODE)
	{
		if ((prn <= MAX_GPS_PRN && prn >= MIN_GPS_PRN) || 
			(prn <= MAX_QZSS_PRN && prn >= MIN_QZSS_PRN) ||
			(prn <= MAX_WAAS_PRN && prn >= MIN_WAAS_PRN))
		{
			return TRUE;
		}else
		{
			return FALSE;
		}
	}else if (gnssMode == GLN_MODE)
	{
		if (prn <= MAX_GLN_PRN && prn >= MIN_GLN_PRN)
		{
			return TRUE;
		}else
		{
			return FALSE;
		}
	}else if (gnssMode == BDS_MODE)
	{
		if (prn <= MAX_BDS_PRN && prn >= MIN_BDS_PRN)
		{
			return TRUE;
		}else
		{
			return FALSE;
		}
	}else if (gnssMode == GAL_MODE)
	{
		if (prn <= MAX_GAL_PRN && prn >= MIN_GAL_PRN)
		{
			return TRUE;
		}else
		{
			return FALSE;
		}
	}else
	{
		return FALSE;
	}
}

/**********************************************************************
* Function Name:    gnss_sd_svch_rm
*
* Description:
*    AP to remove satellite's channel. This will be used when satellite is invisible
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_sd_svch_rm(uint8_t gnssMode, uint8_t sv)
{
	uint32_t           i;
	sat_data_t*   sp;
	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM; i++)
	{
		sp = Sd_data.sat_data[i];
		if (sp == NULL)
		{
			continue;
		}
		if (sp->gnssMode == gnssMode && sp->prn == sv)
		{
			Sys_Free(sp);
			Sd_data.sat_data[i] = NULL;
			break;
		}
	}
}
/**********************************************************************
* Function Name:    gnss_sd_svch_insert
*
* Description:
*    AP to insert satellite's channel when there is a new measurement.
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_sd_svch_insert(uint8_t gnssMode, uint8_t sv, uint32_t freq_index)
{
	uint32_t           i, sat = 0;
	int16_t           ch;
	sat_data_t*    sp;

	ch = gnss_sd_svch_get(gnssMode,sv,freq_index);
	sat = satno(GNSS_MODE2SYS(gnssMode, sv), sv);

	if (ch >= 0 || ch == -2 || sat <= 0)
	{
		return;
	}

	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM;i++)
	{
		sp = Sd_data.sat_data[i];
		if (sp == NULL)
		{
			sp = (sat_data_t*)Sys_Calloc(1,sizeof(sat_data_t));
			if (sp == NULL)
			{
				SYS_LOGGING(OBJ_SD,LOG_ERROR,"Memory alloc Fail %s %d",__FUNCTION__,__LINE__);
				return;
			}
			Sd_data.sat_data_idx[sat - 1][freq_index] = i;
			Sd_data.sat_data[i] = sp;
			sp->gnssMode = gnssMode;
			sp->prn = sv;
#if 0
			sp->reqEphThresT = 300;
#endif
			sp->freq_index = freq_index;
			break;
		}
	}
}

/**********************************************************************
* Function Name:    gnss_sd_sv_add
*
* Description:
*    AP to add new satellite measurement
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_sd_add_sv(uint8_t gnssMode, uint8_t sv, uint32_t freq_index)
{
	int16_t        ch;

	ch = gnss_sd_svch_get(gnssMode,sv,freq_index);

	if (ch >= 0 || ch == -2)
	{
		return;
	}
	gnss_sd_svch_insert(gnssMode,sv,freq_index);
}

/**********************************************************************
* Function Name:    gnss_sd_get_sv_data
*
* Description:
*    AP to get satellite data
*
* Input:
*       None
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
sat_data_t* gnss_sd_get_sv_data(uint8_t gnssMode, uint8_t sv, uint32_t freq_index)
{
	int16_t    ch = -1;

	ch = gnss_sd_svch_get(gnssMode,sv,freq_index);
	if (ch < 0)
	{
		return NULL;
	}else
	{
		return Sd_data.sat_data[ch];
	}
}

/***********************************************************************
* ��������: gnss_sd_save_meas
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/17/
***********************************************************************/
void gnss_sd_save_meas(sat_data_t* sp,gnss_meas_t* pSvMeas,double tor,double biad_adj, double drift_adj)
{
	uint32_t      i;
	
	if (sp == NULL || pSvMeas == NULL) 
	{
		return;
	}

	for (i = MEAS_SAVE_NUM - 1; i > 0;i--)
	{
		memcpy(&sp->measBack[i],&sp->measBack[i-1],sizeof(MEAS_SAVE));
	}
	sp->measBack[0].pr = pSvMeas->pseudoRange;
	sp->measBack[0].dr = pSvMeas->pseudoRangeRate;
	sp->measBack[0].biasAdjMs = biad_adj;
	sp->measBack[0].driftAdj = drift_adj;
	sp->measBack[0].tor = tor;
	sp->measBack[0].tot = pSvMeas->tot;
#ifdef USED_IN_MC262M
	sp->measBack[0].codeStd = pSvMeas->codeDetStd;
	sp->measBack[0].cr = pSvMeas->carrierPhase;
#endif
	sp->measBack[0].cycleSlipCnt = pSvMeas->cycleSlipCount;
	if (sp->measBackCnt < MEAS_SAVE_NUM) 
	{
		sp->measBackCnt++;
	}
}

/***********************************************************************
* ��������: gnss_sd_save_meas
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�1/17/
***********************************************************************/
void gnss_sd_clear_meas(Sd_t* p_Sd)
{
	uint8_t          i;
	sat_data_t* sp;

	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM; i++)
	{
		sp = p_Sd->sat_data[i];
		if (sp == NULL || sp->prn == 0) continue;

		memset(&sp->measBack[0],0,sizeof(MEAS_SAVE) * MEAS_SAVE_NUM);
		sp->prR = 0.0;
		sp->prRCnt = 0;
		sp->goodPrCnt = 0;
		sp->last_dr = 0.0;
		sp->last_pr = 0.0;
		sp->last_tor = 0.0;
	}
}

/***********************************************************************
* ��������: gnss_Sd_Init
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/11/
***********************************************************************/
void gnss_Sd_Init(Sd_t* p_Sd)
{
	memset(p_Sd,0,sizeof(Sd_t));
}
/***********************************************************************
* ��������: gnss_Sd_Del, this should be called when program stopped
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/26/
***********************************************************************/
void gnss_Sd_Del(void)
{
	uint32_t          i;

	// Delete satellite data
	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM;i++)
	{
		if (Sd_data.sat_data[i])
		{
			Sys_Free(Sd_data.sat_data[i]);
			Sd_data.sat_data[i] = NULL;
		}
	}

	// Delete NM
	gnss_Sd_Nm_Del();

#if 0
	// RTD data delete
	gnss_sd_rtcm_del(&g_rtcm_data);
#endif

	// clear SD data
	memset(&Sd_data,0,sizeof(Sd_t));
}
/**********************************************************************
* Function Name:    gnss_sd_user_dcos
*
* Description:
*    This function is used to calculate user direction cosine
*    cosx = x / r;  cosy = y / r;   cosz = z / r
*    r = sqrt(x^2+y^2+z^2);
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
static void gnss_Sd_User_dcos(void* rcv_ecef, float* udcos)
{
	double             curr_r;
	USER_ECEF*      user_ecef = (USER_ECEF*)rcv_ecef;

	curr_r = gnssClcSqrtSum_DBL(user_ecef->pos,3);

	/*  catch X,Y,Z = 0,0,0  */
	if (!SIGN_D(&curr_r))
	{
		curr_r = 1.0;
		user_ecef->pos[0] = 1.0;
	}

	/*  find direction cosines to center of earth */
	udcos[0] = (float)(user_ecef->pos[0] / curr_r);
	udcos[1] = (float)(user_ecef->pos[1] / curr_r);
	udcos[2] = (float)(user_ecef->pos[2] / curr_r);
}

/**********************************************************************
* Function Name:    gnss_Sd_Sat_dcos
*
* Description:
*    This function is used to direction cosine of satellite
*
* Input:
*       satMode: 0 --> GPS,  1 --> GLN,  2 --> BDS, 3 --> GAL
*       svId: PRN number. For GLN, it's PRN, not (freqNo + 8)
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint8_t gnss_Sd_Sat_dcos(GNSS_TIME* pTime,void* user_ecef,sat_data_t* sp)
{
	uint16_t        tot_week;
	double        posdc[3];
	double        inv_r;
	double        range;
	double        tot, dt = 0.0;
	D_COS*     dcos;
	TRANS*     tr = NULL;
	ECEF*      sv_ecef;

	USER_ECEF* userEcefPos = (USER_ECEF*)user_ecef;


	if (pTime == NULL || user_ecef == NULL || sp == NULL)
	{
		return FRM_NULL;
	}
	sv_ecef = &sp->ecef;
	/* approx TOT of this SV */
	if (pTime->timeStatus[sp->gnssMode] >= TM_STATUS_APPX)
	{
		if (sp->gnssMode == BDS_MODE)
		{
			tot = pTime->rcvr_time[sp->gnssMode] - 0.120;
		}
		else
		{
			tot = pTime->rcvr_time[sp->gnssMode] - 0.077;
		}
		tot_week = pTime->week[sp->gnssMode];
	}
	else
	{
		sv_ecef->have_spos = FRM_NULL;
		return FRM_NULL;
	}

	/* when tot < 0, then rover */
	if (tot < 0.0)
	{
		if (sp->gnssMode != GLN_MODE)
		{
			tot += (double)SECS_IN_WEEK;
			tot_week--;
		}
		else
		{
			tot += SECS_IN_DAY;
		}
	}

	/* Get satellite position */
	if (sv_ecef->have_spos == FRM_EPH)
	{
		dt = tot - sv_ecef->t;
		if (sp->gnssMode != GLN_MODE)
		{
			if (dt > SECS_IN_WEEK / 2.0)
			{
				dt -= (double)SECS_IN_WEEK;
			}
			else if (dt < -SECS_IN_WEEK / 2.0)
			{
				dt += (double)SECS_IN_WEEK;
			}
		}
		else
		{
			if (dt > SECS_IN_DAY / 2.0)
			{
				dt -= SECS_IN_DAY;
			}
			else if (dt < -SECS_IN_DAY / 2.0)
			{
				dt += SECS_IN_DAY;
			}
		}
	}

	if ((fabs(dt) > 0.15) || (sv_ecef->have_spos == FRM_NULL))
	{
		/* Get satellite position */
		gnss_Sd_Pos(sp->gnssMode, sp->prn, tot_week, tot, sv_ecef, sp->freq_index);
	}

	if (sv_ecef->have_spos == FRM_NULL)
	{
		return FRM_NULL;
	}

	/* get enu_trans matrix */
	tr = &Sd_data.sd_enu_trans;
	dcos = &sp->d_cos;
	/*  difference user and sv positions & find radius */
	posdc[0] = sv_ecef->p[0] - userEcefPos->pos[0];
	posdc[1] = sv_ecef->p[1] - userEcefPos->pos[1];
	posdc[2] = sv_ecef->p[2] - userEcefPos->pos[2];
	range = gnssClcSqrtSum_DBL(posdc,3);
	inv_r = (double)1.0 / range;
	/*  compute user-sv direction cosines */
	dcos->dcosx = (float)(posdc[0] * inv_r);
	dcos->dcosy = (float)(posdc[1] * inv_r);
	dcos->dcosz = (float)(posdc[2] * inv_r);
	dcos->east_dcos  = tr->et1 * dcos->dcosx + tr->et2 * dcos->dcosy;
	dcos->north_dcos = tr->nt1 * dcos->dcosx + tr->nt2 * dcos->dcosy +
		tr->ce * dcos->dcosz;
	/*  compute elevation angle */
	dcos->sin_ea = dcos->dcosx * tr->utx +
		dcos->dcosy * tr->uty +
		dcos->dcosz * tr->utz;
	dcos->elev   = (float)gnss_asinf(dcos->sin_ea);

	/*  compute azimuth angle */
	if (dcos->east_dcos != 0.0 || dcos->north_dcos != 0.0)
	{
		dcos->az = (float)atan2(dcos->east_dcos, dcos->north_dcos);
	}
	else
	{
		dcos->az = 0.0;
	}

	//trun to 0~2*pi;
	if( dcos->az < 0 )
	{
		dcos->az = dcos->az + (float)(2*PI);
	}

	dcos->range  = range;
	sp->range = range;

	/*  return satellite position status (FRM_ALM or FRM_EPH) */
	return sv_ecef->have_spos;
}

/***********************************************************************
* ��������: gnss_Sd_Sv_Update
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/26/
***********************************************************************/
void gnss_Sd_Sv_Update(USER_PVT* user)
{
	uint8_t             have_spos;
	uint32_t            i;
	GNSS_TIME*     pTime;
	sat_data_t*    sp;

	pTime = gnss_tm_get_time();

	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM; i++)
	{
		sp = Sd_data.sat_data[i];
		if (sp == NULL)
		{
			continue;
		}
		if (!sp->prn)
		{
			continue;
		}
		// calculate satellite direction cosine and satellite position
		have_spos = gnss_Sd_Sat_dcos(pTime,&user->ecef,sp);
		if (have_spos == FRM_NULL)
		{
			continue;
		}
		// calculate IONO and TROP correction
		gnss_sd_sat_iontrocorr(sp->gnssMode, sp->prn, user->lla.pos, &sp->d_cos, &sp->mcorr, sp->freq_index);
	}
}

/***********************************************************************
* ��������: gnss_Sd_Rm_Sv
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/26/
***********************************************************************/
void gnss_Sd_Rm_Sv(Sd_t* sd)
{
	uint8_t               gnssMode,prn;
	uint32_t              i;
	double              dt;
	sat_data_t*      sp;

	for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM;i++)
	{
		sp = sd->sat_data[i];
		if (sp == NULL) {
			continue;
		}
		if (sp->prn == 0) {
			continue;
		}

		dt = gpz_Meas->tor - sp->last_tor;
		if (dt > 302400) dt = dt - 604800;
		else if (dt < -302400) dt = dt + 604800;
	
		gnssMode = sp->gnssMode;
		prn = sp->prn;
		if ((sp->d_cos.elev*RAD2DEG <= SV_INVISIBLE_THRES && gnss_Pe_Find_SvMeas(gnssMode,prn) == FALSE)
			|| (sp->ecef.have_spos == FRM_NULL && fabs(dt) > 60))
		{
			SYS_LOGGING(OBJ_SD, LOG_INFO,"gnssMode(%d) prn(%02d) became invisible",sp->gnssMode,sp->prn);
			// remove saved EPH and ALM
			gnss_Sd_Nm_RmSv(gnssMode,prn);

			// remove this satellite data
			//gnss_sd_svch_rm(gnssMode,prn);
			Sys_Free(sp); Sd_data.sat_data[i] = NULL;
			
			// remove this satellite's RTD data
			gnss_sd_rm_sv_rtd(gnssMode,prn);
		}
	}

}

double gnss_sd_get_geoId(void)
{
	return Sd_data.sd_geoid_corr;
}

/***********************************************************************
* ��������: gnss_Sd_Main
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/26/
***********************************************************************/
void gnss_Sd_Main(Sd_t* sd,void* upvt)
{
	uint32_t           flag;
	uint32_t           i;
	//float           distance;
	double           dt;
	USER_PVT*     user,pvt;
	GNSS_TIME*    pTime;

	pTime = gnss_tm_get_time();
	// time status check, if status of any mode is unknown, then return
	for (i = 0; i < GNSS_MAX_MODE; i++)
	{
		if (pTime->timeStatus[i] < TM_STATUS_APPX)
		{
			return;
		}
	}

	memset(&pvt,0,sizeof(USER_PVT));
	// Check if there is user PVT information
	if (upvt)
	{
		user = (USER_PVT*)upvt;
	}else
	{
		gnss_Pe_Get_PosFix(&pvt);
		user = &pvt;
	}

	// If there is no user ECEF position, then return
	if (user->have_position >= HAVE_POS_OLD || user->have_position < HAVE_POS_APPX)
	{
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"There is no user ECEF position");
		return;
	}

	/* calculate GeoId correction */
	if (sd->geoId_init == FALSE)
	{
		memcpy(sd->geoId_LLA,user->lla.pos,3*sizeof(double));
		sd->sd_geoid_corr = gnss_sd_geoidh(sd->geoId_LLA);
		sd->geoId_init = TRUE;
	}else
	{
		/*distance = gnssCalPosDis(sd->geoId_LLA,user->lla.pos,1);
		if (distance > 50000.0)*/
		{
			memcpy(sd->geoId_LLA,user->lla.pos,3*sizeof(double));
			sd->sd_geoid_corr = gnss_sd_geoidh(sd->geoId_LLA);
		}
	}

	// check if need dcos and IONO update
	if (!sd->Mcorr_init)
	{
		flag = TRUE;
		sd->Mcorr_init = TRUE;
		sd->last_mcorr_tor = pTime->rcvr_time[0];
	}else
	{
		dt = pTime->rcvr_time[0] - sd->last_mcorr_tor;
		if (dt < -SECS_IN_WEEK / 2.0)
		{
			dt += (double)SECS_IN_WEEK;
		}
		else if (dt > SECS_IN_WEEK / 2.0)
		{
			dt -= (double)SECS_IN_WEEK;
		}
		
		if ((((dt - 0.9) > DOUBLE_EPS) && !IS_PPK_REVERSE) ||
		   (((dt + 0.9) < DOUBLE_EPS) && IS_PPK_REVERSE))
		{
			flag = TRUE;
			sd->last_mcorr_tor = pTime->rcvr_time[0];
		}
		else
		{
			flag = FALSE;
		}
	}

	if (flag == FALSE)
	{
		return;
	}

	// 1. calculate the user direct cosine
	gnss_Sd_User_dcos((void*)&user->ecef,&sd->udcos[0]);

	// 2. Prepare for measurement corrections
	gnss_sd_mcorr_prep((void*)&user->ecef,
		(void*)&user->lla,
		&sd->sd_enu_trans,
		&sd->sd_flla,
		&sd->sd_geoid_corr);

	// 3. calculate each satellite's direction cosine and error correction
	gnss_Sd_Sv_Update(user);

	// 4. Check satellites' elevation to remove invisible satellites
	gnss_Sd_Rm_Sv(sd);
}