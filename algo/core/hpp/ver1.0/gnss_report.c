/************************************************************
* Copyrights(C) 
* All rights Reserved
* 文件名称：gnss_report.c
* 文件描述：
* 版本号：1.0.0
* 作者：
* 日期：10/14
************************************************************/
#include "gnss_pe.h"
#include "gnss_sys_api.h"
#include "gnss_comm.h"
#include "gnss_tm.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_SYS

#define REPORT(fmt, ...)    SYS_LOGGING(OBJ_ALGO, LOG_INFO, fmt, ##__VA_ARGS__)

extern gtime_t g_localgpstor;

static void gnss_report_svid(const meas_blk_t* pMeas)
{
	uint32_t prn,j;
	char str[1024] = {'\0'}, *p = &str[0];

	p += sprintf(p, "ME PRN List: ");

	// print GPS
	for(prn = MIN_GPS_PRN;prn <= MAX_GPS_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GPS_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "G%02d ", pSvMeas->prn);
			}
		}
	}

	// print GLN
	for(prn = MIN_GLN_PRN;prn <= MAX_GLN_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GLN_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "N%02d ", pSvMeas->prn);
			}
		}
	}
	// print BDS
	for(prn = MIN_BDS_PRN;prn <= MAX_BDS_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((BDS_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "B%02d ", pSvMeas->prn);
			}
		}
	}

    // print GAL
	for(prn = MIN_GAL_PRN;prn <= MAX_GAL_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GAL_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "E%02d ", pSvMeas->prn);
			}
		}
	}
    
	// print QZSS
	for (prn = MIN_QZSS_PRN;prn <= MAX_QZSS_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GPS_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "G%03d ", pSvMeas->prn);
			}
		}
	}
	//p += sprintf(p, NEWLINE);
	REPORT(str);
}

static void gnss_report_cno(const meas_blk_t* pMeas)
{
	uint32_t prn,j;
	char str[1024] = {'\0'}, *p = &str[0];

	p += sprintf(p, "ME CNO List: ");
	// print GPS
	for(prn = MIN_GPS_PRN;prn <= MAX_GPS_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GPS_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "%-3u ", pSvMeas->cno);
			}
		}
	}

	// print GLN
	for(prn = MIN_GLN_PRN;prn <= MAX_GLN_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GLN_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "%-3u ", pSvMeas->cno);
			}
		}
	}
	// print BDS
	for(prn = MIN_BDS_PRN;prn <= MAX_BDS_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((BDS_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "%-3u ", pSvMeas->cno);
			}
		}
	}

    // print GAL
	for(prn = MIN_GAL_PRN;prn <= MAX_GAL_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GAL_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "%-3u ", pSvMeas->cno);
			}
		}
	}
    
	// print QZSS
	for(prn = MIN_QZSS_PRN;prn <= MAX_QZSS_PRN;prn++)
	{
		for (j = 0; j < pMeas->measCnt;j++)
		{
			const gnss_meas_t* pSvMeas = &pMeas->meas[j];
			if ((GPS_MODE == pSvMeas->gnssMode) && pSvMeas->prn == prn)
			{
				p += sprintf(p, "%-3u ", pSvMeas->cno);
			}
		}
	}
	//p += sprintf(p, NEWLINE);
	REPORT(str);
}

static void gnss_report_ls(const meas_blk_t* pMeas, const Ls_t* pLs)
{
	uint32_t     i,idx;
	char str[1024] = {'\0'}, *p = &str[0];

	p += sprintf(p, "LS PR  List: ");
	// GPS
	for (i = 0; i < pLs->lsCtrl.prNum;i++)
	{
		idx = pLs->lsCtrl.prUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GPS_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0 && pSvMeas->prn <= MAX_GPS_PRN)
		{
			p += sprintf(p, "G%02d ", pSvMeas->prn);
		}
	}
	// GLN
	for (i = 0; i < pLs->lsCtrl.prNum;i++)
	{
		idx = pLs->lsCtrl.prUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GLN_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0)
		{
			p += sprintf(p, "N%02d ", pSvMeas->prn);
		}
	}

    // BDS
	for (i = 0; i < pLs->lsCtrl.prNum;i++)
	{
		idx = pLs->lsCtrl.prUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (BDS_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0)
		{
			p += sprintf(p, "B%02d ", pSvMeas->prn);
		}
	}

    // GAL
	for (i = 0; i < pLs->lsCtrl.prNum;i++)
	{
		idx = pLs->lsCtrl.prUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GAL_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0)
		{
			p += sprintf(p, "E%02d ", pSvMeas->prn);
		}
	}
    
	// QZSS
	for (i = 0; i < pLs->lsCtrl.prNum;i++)
	{
		idx = pLs->lsCtrl.prUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GPS_MODE == pSvMeas->gnssMode && pSvMeas->prn >= MIN_QZSS_PRN && pSvMeas->prn <= MAX_QZSS_PRN)
		{
			p += sprintf(p, "G%03d ", pSvMeas->prn);
		}
	}

	REPORT(str);
	p = &str[0];
	p += sprintf(p, "LS DR  List: ");
	// GPS
	for (i = 0; i < pLs->lsCtrl.drNum;i++)
	{
		idx = pLs->lsCtrl.drUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GPS_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0 && pSvMeas->prn <= MAX_GPS_PRN)
		{
			p += sprintf(p, "G%02d ", pSvMeas->prn);
		}
	}
	// GLN
	for (i = 0; i < pLs->lsCtrl.drNum;i++)
	{
		idx = pLs->lsCtrl.drUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GLN_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0)
		{
			p += sprintf(p, "N%02d ", pSvMeas->prn);
		}
	}

    // BDS
	for (i = 0; i < pLs->lsCtrl.drNum;i++)
	{
		idx = pLs->lsCtrl.drUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (BDS_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0)
		{
			p += sprintf(p, "B%02d ", pSvMeas->prn);
		}
	}

    // GAL
	for (i = 0; i < pLs->lsCtrl.drNum;i++)
	{
		idx = pLs->lsCtrl.drUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GAL_MODE == pSvMeas->gnssMode && pSvMeas->prn > 0)
		{
			p += sprintf(p, "E%02d ", pSvMeas->prn);
		}
	}
    
	//QZSS
	for (i = 0; i < pLs->lsCtrl.drNum;i++)
	{
		idx = pLs->lsCtrl.drUsedSV[i];
		const gnss_meas_t* pSvMeas = &pMeas->meas[idx];
		if (GPS_MODE == pSvMeas->gnssMode && pSvMeas->prn >= MIN_QZSS_PRN && pSvMeas->prn <= MAX_QZSS_PRN)
		{
			p += sprintf(p, "G%03d ", pSvMeas->prn);
		}
	}
	*p = '\0';
	REPORT(str);
}

static void gnss_report_kf(const meas_blk_t* pMeas, const Kf_t* pKf)
{
	uint32_t     i;
	char str[1024] = {'\0'}, *p = &str[0];

	p += sprintf(p, "KF PR  List: ");
	// GPS
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GPS_MODE && pSvMeas->prn > 0 && pSvMeas->prn <= MAX_GPS_PRN && (pSvMeas->status & 0x1) == 1)
		{
			p += sprintf(p, "G%02d ", pSvMeas->prn);
		}
	}
	// GLN
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GLN_MODE && pSvMeas->prn > 0 && (pSvMeas->status & 0x1) == 1)
		{
			p += sprintf(p, "N%02d ", pSvMeas->prn);
		}
	}

    // BDS
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == BDS_MODE && pSvMeas->prn > 0 && (pSvMeas->status & 0x1) == 1)
		{
			p += sprintf(p, "B%02d ", pSvMeas->prn);
		}
	}

    // GAL
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GAL_MODE && pSvMeas->prn > 0 && (pSvMeas->status & 0x1) == 1)
		{
			p += sprintf(p, "E%02d ", pSvMeas->prn);
		}
	}
    
	//QZSS
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GPS_MODE && pSvMeas->prn >= MIN_QZSS_PRN && pSvMeas->prn <= MAX_QZSS_PRN && (pSvMeas->status & 0x1) == 1)
		{
			p += sprintf(p, "G%03d ", pSvMeas->prn);
		}
	}
	REPORT(str);
	p = &str[0];
	p += sprintf(p, "KF DR  List: ");
	// GPS
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GPS_MODE && pSvMeas->prn > 0 && pSvMeas->prn <= MAX_GPS_PRN && (pSvMeas->status & 0x2) == 0x2)
		{
			p += sprintf(p, "G%02d ", pSvMeas->prn);
		}
	}
	// GLN
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GLN_MODE && pSvMeas->prn > 0 && (pSvMeas->status & 0x2) == 0x2)
		{
			p += sprintf(p, "N%02d ", pSvMeas->prn);
		}
	}

    // BDS
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == BDS_MODE && pSvMeas->prn > 0 && (pSvMeas->status & 0x2) == 0x2)
		{
			p += sprintf(p, "B%02d ", pSvMeas->prn);
		}
	}

    // GAL
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GAL_MODE && pSvMeas->prn > 0 && (pSvMeas->status & 0x2) == 0x2)
		{
			p += sprintf(p, "E%02d ", pSvMeas->prn);
		}
	}
    
	// QZSS
	for (i = 0; i < pMeas->measCnt;i++)
	{
		const gnss_meas_t* pSvMeas = &(pMeas->meas[i]);
		if (pSvMeas->gnssMode == GPS_MODE && pSvMeas->prn >= MIN_QZSS_PRN && pSvMeas->prn <= MAX_QZSS_PRN && (pSvMeas->status & 0x2) == 0x2)
		{
			p += sprintf(p, "G%03d ", pSvMeas->prn);
		}
	}
	*p = '\0';
	REPORT(str);
}


static void gnss_report_pvt(const Ls_t* pLs, const Kf_t* pKf)
{
	const LS_PVT_INFO*   lsPvtInfo = pLs->ls_Pvt_Info;
	const KF_PVT_INFO*   kfPvtInfo = pKf->kf_Pvt_Info;
	char str[1024] = {'\0'}, *p = &str[0];

	// LS results
	p = &str[0];
	if (lsPvtInfo->fix_status == FIX_STATUS_NEW)
	{
		p += sprintf(p, "LS status:NEW ");
		REPORT(str);
		// xyz
		p = &str[0];
		p += sprintf(p, "LS: X  [%15.4f], Y  [%15.4f], Z  [%15.4f], dT[%12.4f], drift[%10.6f]",
			lsPvtInfo->ecefPos[0], lsPvtInfo->ecefPos[1], lsPvtInfo->ecefPos[2], lsPvtInfo->clkBias[0],
			lsPvtInfo->clkDrift);
		*p = '\0';
		REPORT(str);
		//vx,vy,vz
		p = &str[0];
		p += sprintf(p, "LS: VX [%15.4f], VY [%15.4f], VZ [%15.4f]",
			lsPvtInfo->ecefVel[0], lsPvtInfo->ecefVel[1], lsPvtInfo->ecefVel[2]);
		*p = '\0';
		REPORT(str);

		//LLA
		p = &str[0];
		p += sprintf(p, "LS: Lat[%15.6f], Lon[%15.6f], Alt[%15.6f], PRRes[%10.4f], DRRes[%10.4f]",
			lsPvtInfo->lla[0] * RAD2DEG, lsPvtInfo->lla[1] * RAD2DEG, lsPvtInfo->lla[2],pLs->pos_res,pLs->vel_res);
		*p = '\0';
		REPORT(str);

		// ENU
		p = &str[0];
		p += sprintf(p, "LS: VE [%15.4f], VN [%15.4f], VU [%15.4f]",
			lsPvtInfo->enuVel[0], lsPvtInfo->enuVel[1], lsPvtInfo->enuVel[2]);
		*p = '\0';
		REPORT(str);
	}else
	{
		p += sprintf(p, "LS status:OLD ");
		REPORT(str);
		// xyz
		p = &str[0];
		p += sprintf(p, "LS: X  [%15.4f], Y  [%15.4f], Z  [%15.4f], dT[%12.4f], drift[%10.6f]",
			0.0,0.0,0.0,0.0,0.0);
		*p = '\0';
		REPORT(str);
		//vx,vy,vz
		p = &str[0];
		p += sprintf(p, "LS: VX [%15.4f], VY [%15.4f], VZ [%15.4f]",0.0,0.0,0.0);
		*p = '\0';
		REPORT(str);

		//LLA
		p = &str[0];
		p += sprintf(p, "LS: Lat[%15.6f], Lon[%15.6f], Alt[%15.6f]",
			0.0,0.0,0.0);
		*p = '\0';
		REPORT(str);

		// ENU
		p = &str[0];
		p += sprintf(p, "LS: VE [%15.4f], VN [%15.4f], VU [%15.4f]",
			0.0,0.0,0.0);
		*p = '\0';
		REPORT(str);
	}



	// KF results
	p = &str[0];
	if (kfPvtInfo->kfFixStatus == FIX_STATUS_NEW)
	{
		p += sprintf(p, "KF status:NEW ");
		REPORT(str);
		// xyz
		p = &str[0];
		p += sprintf(p, "KF: X  [%15.4f], Y  [%15.4f], Z  [%15.4f], dT[%12.4f], drift[%10.6f]",
			kfPvtInfo->ecefPos[0], kfPvtInfo->ecefPos[1], kfPvtInfo->ecefPos[2], kfPvtInfo->clkBias[0],
			kfPvtInfo->clkDrift);
		*p = '\0';
		REPORT(str);

		//vx,vy,vz
		p = &str[0];
		p += sprintf(p, "KF: VX [%15.4f], VY [%15.4f], VZ [%15.4f]",
			kfPvtInfo->ecefVel[0], kfPvtInfo->ecefVel[1], kfPvtInfo->ecefVel[2]);
		*p = '\0';
		REPORT(str);

		//LLA
		p = &str[0];
		p += sprintf(p, "KF: Lat[%15.6f], Lon[%15.6f], Alt[%15.6f]",
			kfPvtInfo->kfLLApos[0] * RAD2DEG, kfPvtInfo->kfLLApos[1] * RAD2DEG, kfPvtInfo->kfLLApos[2]);
		*p = '\0';
		REPORT(str);

		// ENU
		p = &str[0];
		p += sprintf(p, "KF: VE [%15.4f], VN [%15.4f], VU [%15.4f]",
			kfPvtInfo->kfENUvel[0], kfPvtInfo->kfENUvel[1], kfPvtInfo->kfENUvel[2]);
		*p = '\0';
		REPORT(str);

	}else
	{
		p += sprintf(p, "KF status:OLD ");
		REPORT(str);
		// xyz
		p = &str[0];
		p += sprintf(p, "KF: X  [%15.4f], Y  [%15.4f], Z  [%15.4f], dT[%12.4f], drift[%10.6f]",
			0.0, 0.0, 0.0, 0.0,0.0);
		*p = '\0';
		REPORT(str);

		//vx,vy,vz
		p = &str[0];
		p += sprintf(p, "KF: VX [%15.4f], VY [%15.4f], VZ [%15.4f]",
			0.0,0.0,0.0);
		*p = '\0';
		REPORT(str);

		//LLA
		p = &str[0];
		p += sprintf(p, "KF: Lat[%15.6f], Lon[%15.6f], Alt[%15.6f]",
			0.0, 0.0, 0.0);
		*p = '\0';
		REPORT(str);

		// ENU
		p = &str[0];
		p += sprintf(p, "KF: VE [%15.4f], VN [%15.4f], VU [%15.4f]",
			0.0, 0.0, 0.0);
		*p = '\0';
		REPORT(str);
	}

}
/***********************************************************************
* 函数介绍: gnss_Pe_Report
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/14/
***********************************************************************/
void gnss_Pe_Report(const meas_blk_t* pMeas, const Ls_t* pLs, const Kf_t* pKf)
{
#ifdef PLAYBACK_MODE
  GNSS_TIME*    pTime;
  UtcTimeType   utcTime;
  pTime = gnss_tm_get_time();
  utcTime = pTime->utcTime;


  if (pKf->pCfg->meas_rate == MEAS_RATE_2HZ || pKf->pCfg->meas_rate == MEAS_RATE_5HZ || pKf->pCfg->meas_rate == MEAS_RATE_10HZ) {
    return;
  }

  if (pTime->init == TRUE)
  {
     gnss_tm_gps2utc(&utcTime,pTime->week[GPS_MODE],pTime->rcvr_time[GPS_MODE]);
  }

  SYS_LOGGING(OBJ_ALGO,LOG_INFO,"===== Calculate PVT (TimeNs = %16.8f,GPST = %16.8f,UTC = %d:%02d:%02d,%02d:%02d:%06.3f)=====", pMeas->tor,pTime->rcvr_time[GPS_MODE],
    utcTime.Year,utcTime.Month,utcTime.Day,
    utcTime.Hour,utcTime.Minute,utcTime.Second);
  gnss_report_svid(pMeas);
  gnss_report_cno(pMeas);
  gnss_report_ls(pMeas,pLs);
  gnss_report_kf(pMeas,pKf);
  gnss_report_pvt(pLs,pKf);
#endif
}

void gnss_Pe_Log_Meas(meas_blk_t* pMeas)
{
#if defined(PLAYBACK_MODE)
	FILE *fidd;
	uint16_t  week = 1922;
	UtcTimeType utc;
	uint32_t i;
	double ll = 0;
	char sys = 'U';	//unknown by default
	//int cnt = 0;
	uint8_t prn;

	if( pMeas->measCnt < 1 )
		return;

	fidd = fopen(".\\data\\post_file\\mem_rinex.16O","a");
	gnss_tm_gps2utc( &utc,  week, pMeas->tor + 17 );
	fprintf(fidd,"%1c%5d%3d%3d%3d%3d%12.7f%3d%3d\n",'>',
		utc.Year, utc.Month, utc.Day, utc.Hour,utc.Minute,utc.Second, 0, pMeas->measCnt );
	for( i = 0; i < pMeas->measCnt; i++ ){
		if(pMeas->meas[i].gnssMode == GPS_MODE ){
			sys = 'G';
		}
		else if(pMeas->meas[i].gnssMode == GLN_MODE ){
			sys = 'R';
		}
		prn = pMeas->meas[i].prn;
		if( prn > 190 ){
			prn = prn - 192;
		}

		fprintf( fidd,"%1c%2d%16.3f%16.3f%16.3f%16.3f\n",
			sys,
			prn,
			pMeas->meas[i].pseudoRange,
			ll,
#ifdef USED_IN_MC262M
			pMeas->meas[i].dopplerStd,
#else
			0.0,
#endif
			(double)pMeas->meas[i].cno );
	}

	fclose(fidd);
#endif
}

