/************************************************************
* Copyrights(C) 
* All rights Reserved
* �ļ����ƣ�gnss_sd_rtd_cor.c
* �ļ��������������㡢�����㡢��ֵ����У��
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�10/20
************************************************************/
#include "macro.h"
#include "gnss.h"
#include "gnss_sd.h"
#include "gnss_sd_nm.h"
#include "gnss_math.h"
#include "gnss_tm.h"
#include "gnss_sys_api.h"
#include "gnss_comm.h"
#include "gnss_config.h"
#include "gnss_pe.h"
#include "gnss_sd.h"
#include "gnss_rtk.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_SD

#if 0
Rtcm_data_t         g_rtcm_data;
#endif

//Duplicated: Rtd_data_t          Rtd_data;
extern gnss_nm_t*   p_Nm;
extern PEReqCommand_t peReqCommand;
extern Gnss_Cfg_t g_pe_cfg;
double                 rtdTimeGap = 0.0;
/***********************************************************************
* ��������: gnss_sd_set_diffage
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/16/
***********************************************************************/
/*void gnss_sd_set_diffage(double age)
{
	rtdTimeGap = age;
}*/
/***********************************************************************
* ��������: gnss_sd_rtd_init
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/20/
***********************************************************************/
void gnss_sd_rtcm_init(Rtcm_data_t* p_rtcm)
{
	if (p_rtcm == NULL)
	{
		return;
	}
#ifdef AG_GNSS_RTD_FUNCTION_IMPL
	memset(&p_rtcm->rtd, 0, sizeof(pe_rtd_data_t)); 
#else
	memset(&p_rtcm->rtk, 0, sizeof(pe_rtk_data_t)); 
#endif   
}

/***********************************************************************
* ��������: gnss_sd_rtd_del
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/20/
***********************************************************************/
void gnss_sd_rtcm_del(Rtcm_data_t* p_rtcm)
{
	uint32_t i;

#ifdef AG_GNSS_RTD_FUNCTION_IMPL
	for (i = 0; i < MAX_PRN_ALL_MODE; i++)
	{
		if (p_rtcm->rtd.dgnss_data[i])
		{
			Sys_Free(p_rtcm->rtd.dgnss_data[i]);
			p_rtcm->rtd.dgnss_data[i] = NULL;
		}
	} 
#else
	for (i = 0; i < MAXOBSBUF; i++)
	{
		if (p_rtcm->rtk.obs[i].data)
		{
			Sys_Free(p_rtcm->rtk.obs[i].data);
			p_rtcm->rtk.obs[i].data = NULL;
		}
	} 
#endif   
}


/***********************************************************************
* ��������: gnss_sd_get_sv_rtd
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/20/
***********************************************************************/
dgnss_t* gnss_sd_get_sv_rtd(uint8_t gnssMode,uint8_t prn)
{
#ifdef AG_GNSS_RTD_FUNCTION_IMPL
	uint32_t      idx;

	idx = gnss_sv_Idx(gnssMode,prn);

	return g_rtcm_data.rtd.dgnss_data[idx];
#else
	return NULL;
#endif
}

/***********************************************************************
* ��������: gnss_sd_rm_sv_rtd
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/20/
***********************************************************************/
void gnss_sd_rm_sv_rtd(uint8_t gnssMode, uint8_t prn)
{
#ifdef AG_GNSS_RTD_FUNCTION_IMPL
	uint32_t       idx;

	idx = gnss_sv_Idx(gnssMode,prn);

	if (g_rtcm_data.rtd.dgnss_data[idx])
	{
		Sys_Free(g_rtcm_data.rtd.dgnss_data[idx]);
		g_rtcm_data.rtd.dgnss_data[idx] = NULL;
	}
#else
	PARAMETER_NOT_USED(gnssMode);
	PARAMETER_NOT_USED(prn);
#endif
}

/***********************************************************************
* ��������: gnss_sd_rtcm_rtd_proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/20/
***********************************************************************/
void gnss_sd_rtcm_rtd_proc(rtcm_data_pack_t* rawData)
{
#ifdef AG_GNSS_RTD_FUNCTION_IMPL
	uint8_t               flag[4] = {0};
	uint32_t              i;
	uint32_t              idx;
	GNSS_TIME*       pTime;
	rtcm2_dgnss_t*   dgnss_data = &rawData->rtcm2_data;

	pTime = gnss_tm_get_time();

	for (i = 0; i < rawData->rtcm2_data.cnt; i++)
	{
		if (gnss_sd_prn_check(dgnss_data->data[i].gnssMode, dgnss_data->data[i].prn) == FALSE)
		{
			continue;
		}

		idx = gnss_sv_Idx(dgnss_data->data[i].gnssMode, dgnss_data->data[i].prn);

		if (NULL == g_rtcm_data.rtd.dgnss_data[idx])
		{
			if (NULL == (g_rtcm_data.rtd.dgnss_data[idx] = Sys_Calloc(1,sizeof(dgnss_t))))
			{
				GLOGE("Can't Alloc Mem %s",__FUNCTION__);
				continue;
			}
		}

		memcpy(g_rtcm_data.rtd.dgnss_data[idx], &dgnss_data->data[i], sizeof(dgnss_t));

		if((dgnss_data->data[i].gnssMode == GPS_MODE) && (flag[GPS_MODE] == FALSE))
		{
			g_rtcm_data.rtd.time[GPS_MODE] =  dgnss_data->data[i].time;
			flag[GPS_MODE] = TRUE;
			GLOGI("GPS RTCM Time: %lld GPS System Time: %lld", (int64_t)g_rtcm_data.rtd.time[GPS_MODE].time, (int64_t)pTime->GPSTime.time);
		}
		else if (dgnss_data->data[i].gnssMode == GLN_MODE && flag[GLN_MODE] == FALSE)
		{
			g_rtcm_data.rtd.time[GLN_MODE] =  dgnss_data->data[i].time;
			flag[GLN_MODE] = TRUE;
			GLOGI("GLN RTCM Time: %lld GLN System Time: %lld", (int64_t)g_rtcm_data.rtd.time[GLN_MODE].time, (int64_t)pTime->GLNTime.time);
		}
		else if (dgnss_data->data[i].gnssMode == BDS_MODE && flag[BDS_MODE] == FALSE)
		{
			g_rtcm_data.rtd.time[BDS_MODE] = dgnss_data->data[i].time;
			flag[BDS_MODE] = TRUE;
			GLOGI("BDS RTCM Time: %lld BDS System Time: %lld", (int64_t)g_rtcm_data.rtd.time[BDS_MODE].time, (int64_t)pTime->BDSTime.time);
		}
	}
#else
	PARAMETER_NOT_USED(rawData);
#endif
}

/**
*  @brief  gnss rtcm32 dgnss data process
*  @author caizhijie
*  @date   04/24 11:47
*/
void gnss_sd_rtcm_rtk_proc(rtcm_data_pack_t* rawData)
{
#ifdef AG_GNSS_RTK_FUNCTION_IMPL
	gnss_rtk_rtcm3_proc(rawData);
#endif
}

/***********************************************************************
* ��������: gnss_sd_rtd_glb_chck
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/20/
***********************************************************************/
void gnss_sd_rtd_glb_chck(meas_blk_t* pMeas,Rtcm_data_t* p_rtcm, PE_MODES* pMode)
{
	return;
}

