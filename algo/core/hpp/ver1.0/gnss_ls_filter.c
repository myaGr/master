/************************************************************
* Copyrights(C) 
* All rights Reserved
* �ļ����ƣ�gnss_ls_filter.c
* �ļ���������С���˶�λ����
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�11/16
************************************************************/
#include "gnss.h"
#include "gnss_ls.h"
#include "gnss_math.h"
#include "gnss_sys_api.h"
#include "gnss_kf_math.h"

/***********************************************************************
* ��������: gnss_Ls_Filter
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/16/
***********************************************************************/
void gnss_Ls_Filter(Ls_t* pLs)
{
	uint8_t                 i;
    float                velTmp;
    double                dt;
    double                posTmp;
	LS_PVT_INFO*       lsPvtInfo;
	Ls_Fit_t*          lsFitRst;

	lsFitRst = &(pLs->fitRst);
	lsPvtInfo = pLs->ls_Pvt_Info;

	if (lsFitRst->init == FALSE)
	{
		if (lsPvtInfo->fix_status == FIX_STATUS_NEW)
		{
			for (i = 0; i < 3; i++)
			{
				lsFitRst->fitEcef.p[i] = lsPvtInfo->ecefPos[i];
				lsFitRst->fitEcef.v[i] = lsPvtInfo->ecefVel[i];
			}
			lsFitRst->fitEcef.t = lsPvtInfo->tor;
			lsFitRst->init = TRUE;
			lsFitRst->coef[0] = 0.5;
			lsFitRst->coef[1] = 0.5;
		}
	}else
	{
		/* smooth filter assuming LS velocity is accurate */
        dt = lsPvtInfo->tor - lsFitRst->fitEcef.t;
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
           posTmp = lsFitRst->fitEcef.p[i];
           velTmp = lsFitRst->fitEcef.v[i];
           lsFitRst->fitEcef.p[i] = 0.8 * (posTmp + velTmp * dt) + 0.2 * lsPvtInfo->ecefPos[i];
           lsFitRst->fitEcef.v[i] = lsPvtInfo->ecefVel[i];
        }

        lsFitRst->fitEcef.t = lsPvtInfo->tor;
	}

}