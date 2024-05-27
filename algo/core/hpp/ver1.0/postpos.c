/*------------------------------------------------------------------------------
* postpos.c : post-processing positioning
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/05/08  1.0  new
*           2008/06/16  1.1  support binary inputs
*           2009/01/02  1.2  support new rtk positioing api
*           2009/09/03  1.3  fix bug on combined mode of moving-baseline
*           2009/12/04  1.4  fix bug on obs data buffer overflow
*           2010/07/26  1.5  support ppp-kinematic and ppp-static
*                            support multiple sessions
*                            support sbas positioning
*                            changed api:
*                                postpos()
*                            deleted api:
*                                postposopt()
*           2010/08/16  1.6  fix bug sbas message synchronization (2.4.0_p4)
*           2010/12/09  1.7  support qzss lex and ssr corrections
*           2011/02/07  1.8  fix bug on sbas navigation data conflict
*           2011/03/22  1.9  add function reading g_tec file
*           2011/08/20  1.10 fix bug on freez if solstatic=single and combined
*           2011/09/15  1.11 add function reading stec file
*           2012/02/01  1.12 support keyword expansion of rtcm ssr corrections
*           2013/03/11  1.13 add function reading otl and erp data
*           2014/06/29  1.14 fix problem on overflow of # of satellites
*           2015/03/23  1.15 fix bug on ant type replacement by rinex header
*                            fix bug on combined filter for moving-base mode
*           2015/04/29  1.16 fix bug on reading rtcm ssr corrections
*                            add function to read satellite fcb
*                            add function to read stec and troposphere file
*                            add keyword replacement in dcb, erp and ionos file
*           2015/11/13  1.17 add support of L5 antenna phase center paramters
*                            add *.stec and *.trp file for ppp correction
*           2015/11/26  1.18 support opt->freqopt(disable L2)
*           2016/01/12  1.19 add carrier-phase bias correction by ssr
*           2016/07/31  1.20 fix error message problem in rnx2rtkp
*           2016/08/29  1.21 suppress warnings
*           2016/10/10  1.22 fix bug on identification of file fopt->blq
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "macro.h"
#include "gnss.h"
#include "gnss_math.h"
#include "gnss_rtk.h"
#include "gnss_pe.h"

#if 0


#undef MODULE_TAG
#define MODULE_TAG OBJ_PCSIM

#ifdef AG_GNSS_RTK_FUNCTION_IMPL

#define MIN(x,y)    ((x)<(y)?(x):(y))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

#define MAXPRCDAYS  100          /* max days of continuous processing */
#define MAXINFILE   1000         /* max number of input files */
#define MAXEXEVENT 10000      /* max external event time tag */
#define N_FIT_POINT 6

extern Gnss_Cfg_t  g_pe_cfg;

static int antpos(prcopt_t *opt, int rcvno, const obs_t *obs, const nav_t *nav,
                  const sta_t *sta, const char *posfile);

/* constants/global variables ------------------------------------------------*/

static obs_t obss={0};          /* observation data */
static nav_t navs={0};          /* navigation data */
static sbs_t sbss={0};          /* sbas messages */
//static lex_t lexs={0};          /* lex messages */
static sta_t stas[MAXRCV];      /* station infomation */
static int nepoch=0;            /* number of observation epochs */
static int iobsu =0;            /* current rover observation data index */
static int iobsr =0;            /* current reference observation data index */
static int isbs  =0;            /* current sbas message index */
static int ilex  =0;            /* current lex message index */
static int revs  =0;            /* analysis direction (0:forward,1:backward) */
static int aborts=0;            /* abort status */
static USER_PVT *solf;             /* forward solutions */
static USER_PVT *solb;             /* backward solutions */
static double *rbf;             /* forward base positions */
static double *rbb;             /* backward base positions */
static int isolf=0;             /* current forward solutions index */
static int isolb=0;             /* current backward solutions index */
static char proc_rov [64]="";   /* rover for current processing */
static char proc_base[64]="";   /* base station for current processing */
static gtime_t ex_event_tag[MAXEXEVENT];
static int ex_event_cnt=0;

static const int solq_gga[MAXSOLQ+1] = {  /* nmea quality flags to rtklib sol quality */
								  /* nmea 0183 v.2.3 quality flags: */
								  /*  0=invalid, 1=gps fix (sps), 2=dgps fix, 3=pps fix, 4=rtk, 5=float rtk */
								  /*  6=estimated (dead reckoning), 7=manual input, 8=simulation */

	0, 4, 5, 0, 2, 1, 0, 0
};
static void post_clean_exevent(void)
{
	ex_event_cnt=0;
}
extern int post_get_exevent_cnt(void)
{
	return ex_event_cnt;
}
extern void post_add_exevent_tag(gtime_t tag)
{
	if (ex_event_cnt<MAXEXEVENT)
	{
		ex_event_tag[ex_event_cnt++]=tag;
	}
}
/* show message and check break ----------------------------------------------*/
static int checkbrk(const char *format, ...)
{
    va_list arg;
    char buff[1024],*p=buff;
    if (!*format) return showmsg("");
    va_start(arg,format);
    p+=vsprintf(p,format,arg);
    va_end(arg);
    if (*proc_rov&&*proc_base) sprintf(p," (%s-%s)",proc_rov,proc_base);
    else if (*proc_rov ) sprintf(p," (%s)",proc_rov );
    else if (*proc_base) sprintf(p," (%s)",proc_base);
    return showmsg(buff);
}

/* search next observation data index ----------------------------------------*/
static int nextobsf(const obs_t *obs, int *i, int rcv)
{
    double tt;
    int n;
    
    for (;*i<obs->n;(*i)++) if (obs->data[*i].rcv==rcv) break;
    for (n=0;*i+n<obs->n;n++) {
        tt=timediff(obs->data[*i+n].time,obs->data[*i].time);
        if (obs->data[*i+n].rcv!=rcv||tt>DTTOL) break;
    }
    return n;
}
static int nextobsb(const obs_t *obs, int *i, int rcv)
{
    double tt;
    int n;
    
    for (;*i>=0;(*i)--) if (obs->data[*i].rcv==rcv) break;
    for (n=0;*i-n>=0;n++) {
        tt=timediff(obs->data[*i-n].time,obs->data[*i].time);
        if (obs->data[*i-n].rcv!=rcv||tt<-DTTOL) break;
    }
    return n;
}

/* input obs data, navigation messages and sbas correction -------------------*/
extern int inputobs(obsd_t *obs, int solq, const prcopt_t *popt)
{
    gtime_t time={0};
    int i,nu,nr,n=0;
    
    gnss_util_trace(3,"infunc  : revs=%d iobsu=%d iobsr=%d isbs=%d\n",revs,iobsu,iobsr,isbs);
    
    if (0<=iobsu&&iobsu<obss.n) {
        settime((time=obss.data[iobsu].time));
        if (checkbrk("processing : %s Q=%d",time_str(time,0),solq)) {
            aborts=1; showmsg("aborted"); return -1;
        }
    }
    if (!revs) { /* input forward data */
        if ((nu=nextobsf(&obss,&iobsu,1))<=0) return -1;
        if (popt->intpref) {
            for (;(nr=nextobsf(&obss,&iobsr,2))>0;iobsr+=nr)
                if (timediff(obss.data[iobsr].time,obss.data[iobsu].time)>-DTTOL) break;
        }
        else {
            for (i=iobsr;(nr=nextobsf(&obss,&i,2))>0;iobsr=i,i+=nr)
                if (timediff(obss.data[i].time,obss.data[iobsu].time)>DTTOL) break;
        }
        nr=nextobsf(&obss,&iobsr,2);
        for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsu+i];
        for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsr+i];
        iobsu+=nu;
	#if 0
        /* update sbas corrections */
        while (isbs<sbss.n) {
            time=gpst2time(sbss.msgs[isbs].week,sbss.msgs[isbs].tow);
            
            if (asg_getbitu(sbss.msgs[isbs].msg,8,6)!=9) { /* except for geo nav */
                sbsupdatecorr(sbss.msgs+isbs,&navs);
            }
            if (timediff(time,obs[0].time)>-1.0-DTTOL) break;
            isbs++;
        }
	#endif        
    }
    else { /* input backward data */
        if ((nu=nextobsb(&obss,&iobsu,1))<=0) return -1;
        if (popt->intpref) {
            for (;(nr=nextobsb(&obss,&iobsr,2))>0;iobsr-=nr)
                if (timediff(obss.data[iobsr].time,obss.data[iobsu].time)<DTTOL) break;
        }
        else {
            for (i=iobsr;(nr=nextobsb(&obss,&i,2))>0;iobsr=i,i-=nr)
                if (timediff(obss.data[i].time,obss.data[iobsu].time)<-DTTOL) break;
        }
        nr=nextobsb(&obss,&iobsr,2);
        for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsu-nu+1+i];
        for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsr-nr+1+i];
        iobsu-=nu;
        
	#if 0
        /* update sbas corrections */
        while (isbs>=0) {
            time=gpst2time(sbss.msgs[isbs].week,sbss.msgs[isbs].tow);
            
            if (asg_getbitu(sbss.msgs[isbs].msg,8,6)!=9) { /* except for geo nav */
                sbsupdatecorr(sbss.msgs+isbs,&navs);
            }
            if (timediff(time,obs[0].time)<1.0+DTTOL) break;
            isbs--;
        }
	#endif 
    }
    return n;
}

/* read obs and nav data -----------------------------------------------------*/
static int readobsnav(gtime_t ts, gtime_t te, double ti, char **infile,
                      const int *index, int n, const prcopt_t *prcopt,
                      obs_t *obs, nav_t *nav, sta_t *sta)
{
    int i,j,ind=0,nobs=0,rcv=1;
    
    gnss_util_trace(3,"readobsnav: ts=%s n=%d\n",time_str(ts,0),n);
    
    obs->data=NULL; obs->n =obs->nmax =0;
    nav->eph =NULL; nav->n =nav->nmax =0;
    nav->geph=NULL; nav->ng=nav->ngmax=0;
    nav->seph=NULL; nav->ns=nav->nsmax=0;
    nepoch=0;
    
    for (i=0;i<n;i++) {
        if (checkbrk("")) return 0;
        
        if (index[i]!=ind) {
            if (obs->n>nobs) rcv++;
            ind=index[i]; nobs=obs->n; 
        }
        /* read rinex obs and nav file */
        if (readrnxt(infile[i],rcv,ts,te,ti,prcopt->rnxopt[rcv<=1?0:1],obs,nav,
                     rcv<=2?sta+rcv-1:NULL)<0) {
            checkbrk("error : insufficient memory");
            gnss_util_trace(1,"insufficient memory\n");
            return 0;
        }
    }
    if (obs->n<=0) {
        checkbrk("error : no obs data");
        gnss_util_trace(1,"\n");
        return 0;
    }
    if (nav->n<=0&&nav->ng<=0&&nav->ns<=0) {
        checkbrk("error : no nav data");
        gnss_util_trace(1,"\n");
        return 0;
    }
    /* sort observation data */
    nepoch=sortobs(obs);
    
    /* delete duplicated ephemeris */
    uniqnav(nav);
    
    /* set time span for progress display */
    if (ts.time==0||te.time==0) {
        for (i=0;   i<obs->n;i++) if (obs->data[i].rcv==1) break;
        for (j=obs->n-1;j>=0;j--) if (obs->data[j].rcv==1) break;
        if (i<j) {
            if (ts.time==0) ts=obs->data[i].time;
            if (te.time==0) te=obs->data[j].time;
            settspan(ts,te);
        }
    }
    return 1;
}

static int readobsnav_multibase(double ti, char **infile,
    const int *index, int n, const prcopt_t *prcopt,
    obs_t *obs, nav_t *nav, sta_t *sta, ppk_base_swtich_t *p_switch)
{
    int i, j, ind = 0, nobs = 0, rcv = 1;
    gtime_t ts = { 0 }, te = { 0 };

    obs->data = NULL; obs->n = obs->nmax = 0;
    nav->eph = NULL; nav->n = nav->nmax = 0;
    nav->geph = NULL; nav->ng = nav->ngmax = 0;
    nav->seph = NULL; nav->ns = nav->nsmax = 0;
    nepoch = 0;

    for (i = 0; i < n; i++) {
        if (checkbrk("")) return 0;

        if (index[i] != ind) {
            if (obs->n > nobs) rcv++;
            ind = index[i]; nobs = obs->n;
        }
        /* read rinex obs and nav file */
        if (readrnxt_multibase(infile[i], rcv, ti, 
            prcopt->rnxopt[rcv <= 1 ? 0 : 1], obs, nav,
            rcv <= 2 ? sta + rcv - 1 : NULL, p_switch) < 0) {
            checkbrk("error : insufficient memory");
            gnss_util_trace(1, "insufficient memory\n");
            return 0;
        }
    }
    if (obs->n <= 0) {
        checkbrk("error : no obs data");
        gnss_util_trace(1, "\n");
        return 0;
    }
    if (nav->n <= 0 && nav->ng <= 0 && nav->ns <= 0) {
        checkbrk("error : no nav data");
        gnss_util_trace(1, "\n");
        return 0;
    }
    /* sort observation data */
    nepoch = sortobs(obs);

    /* delete duplicated ephemeris */
    uniqnav(nav);

    /* set time span for progress display */
    if (ts.time == 0 || te.time == 0) {
        for (i = 0; i < obs->n; i++) if (obs->data[i].rcv == 1) break;
        for (j = obs->n - 1; j >= 0; j--) if (obs->data[j].rcv == 1) break;
        if (i < j) {
            if (ts.time == 0) ts = obs->data[i].time;
            if (te.time == 0) te = obs->data[j].time;
            settspan(ts, te);
        }
    }
    return 1;
}


/* free obs and nav data -----------------------------------------------------*/
static void freeobsnav(obs_t *obs, nav_t *nav)
{
    gnss_util_trace(3,"freeobsnav:\n");

	if (obs->data)
	{
    	Sys_Free(obs->data);
		obs->n =obs->nmax =0;
    }

	if (nav->eph)
	{
		Sys_Free(nav->eph);
		nav->n =nav->nmax =0;
    }

	if (nav->geph)
	{
		Sys_Free(nav->geph);
		nav->ng=nav->ngmax=0;
    }

	if (nav->seph)
	{
		Sys_Free(nav->seph);
		nav->ns=nav->nsmax=0;
	}
}

/* read rinex obs and nav data -----------------------------------------------------*/
extern int postprepare(char **infile,const int *index, int n, prcopt_t *prcopt)
{
	gtime_t t={0};

	/* read obs and nav data */
	if (!readobsnav(t,t,0.0,infile,index,n,prcopt,&obss,&navs,stas))
	{
		printf("load rinex obs and nav failed!");
		freeobsnav(&obss,&navs);
		return 0;
	}

	/* rover/reference fixed position */
	if (prcopt->mode==PMODE_FIXED) {
		if (!antpos(prcopt,1,&obss,&navs,stas,"")) {
			freeobsnav(&obss,&navs);
			return 0;
		}
	}
	else if (PMODE_DGPS<=prcopt->mode&&prcopt->mode<=PMODE_STATIC) {
		if (!antpos(prcopt,2,&obss,&navs,stas,"")) {
			freeobsnav(&obss,&navs);
			return 0;
		}
	}

	iobsu=iobsr=isbs=ilex=revs=aborts=0;

	return 1;
}
extern void freernxobsnav()
{
	freeobsnav(&obss,&navs);
	post_clean_exevent();
}
extern nav_t* getpostnav()
{
	return (&navs);
}

void post_free_sol(void)
{
    if (solb != NULL)
    {
        Sys_Free(solb);
    }

    if (solf != NULL)
    {
        Sys_Free(solf);
    }
}
int post_preprocess(int reverse)
{
	if (reverse)
	{
		revs = 1; iobsu = iobsr = obss.n - 1; isbs = sbss.n - 1;
		ilex = 0 /* lex not used */;
		if (solb)
		{
			Sys_Free(solb);
		}
		solb = (USER_PVT *)Sys_Malloc(sizeof(USER_PVT) * nepoch);
		if (solb == NULL) return 0;
		//rbb = (double *)malloc(sizeof(double) * nepoch * 3);
	}
	else
	{
		iobsu = iobsr = isbs = ilex = revs = aborts = 0;
		if (solf)
		{
			Sys_Free(solf);
		}
		solf = (USER_PVT *)Sys_Malloc(sizeof(USER_PVT) * nepoch);
		if (solf == NULL) return 0;
		isolb = isolf = 0;
		//rbf = (double *)malloc(sizeof(double) * nepoch * 3);
	}

	return 1;
}
int post_readrnx(char ** infile, const int *index, int n, 
    prcopt_t *prcopt, ppk_base_swtich_t *p_switch)
{
    /* read obs and nav data */
    if (!readobsnav_multibase(0.0, infile, index, n, prcopt, 
        &obss, &navs, stas, p_switch))
    {
        printf("load rinex obs and nav failed!\r\n");
        freeobsnav(&obss, &navs);
        return 0;
    }

    return 1;
}

double* post_get_base_pos(prcopt_t *prcopt, int base_idx)
{
    double *rr, del[3], pos[3], dr[3] = { 0 };
    int i;

    if (base_idx + 1 < MAXRCV)
    {
        rr = prcopt->rb;

        if (asg_norm(stas[base_idx + 1].pos, 3) <= 0.0) {
            return 0;
        }
        /* antenna delta */
        if (stas[base_idx + 1].deltype == 0) { /* enu */
            for (i = 0; i < 3; i++) del[i] = stas[base_idx + 1].del[i];
            del[2] += stas[base_idx + 1].hgt;
            ecef2pos(stas[base_idx + 1].pos, pos);
            enu2ecef(pos, del, dr);
        }
        else { /* xyz */
            for (i = 0; i < 3; i++) dr[i] = stas[base_idx + 1].del[i];
        }
        for (i = 0; i < 3; i++) rr[i] = stas[base_idx + 1].pos[i] + dr[i];

        return rr;
    }
    else
        return NULL;
}

void post_sol_forward_add(const USER_PVT *f)
{
    if (!f) return;
    if (solf && isolf < nepoch)
    {
        solf[isolf++] = *f;
    }
}

void post_sol_backward_add(const USER_PVT *b)
{
    if (!b) return;
    if (solb && isolb < nepoch)
    {
        solb[isolb++] = *b;
    }
}

int post_sol_percentage(int reverse)
{
	if(reverse) return ((isolb) * 100 / nepoch);
	else        return ((isolf) * 100 / nepoch);
}

#ifdef ENABLE_PPK_MODE
static void _sol_fill_pvt(sol_t *psol, USER_PVT* pvt, double *posVarXY)
{
	uint8_t              k, m;
	float             ecefPosVar[3][3], enuPosVar[3][3], ceg[3][3], cge[3][3], matTmp[3][3];
	double             Nh = 0.0;
	int             week = 0;

	if (psol->stat == SOLQ_FLOAT)
	{
		pvt->diff_status = DIFF_SOL_STATUS_RTK_FLOAT;  //float results
	}
	else if (psol->stat == SOLQ_FIX)
	{
		pvt->diff_status = DIFF_SOL_STATUS_RTK_FIX;  //fix results
	}
	else if (psol->stat == SOLQ_DR)
	{
		pvt->diff_status = DIFF_SOL_STATUS_RTK_PROPAGATE;  //propagated by rtk results
	}
	else if (psol->stat == SOLQ_DGPS)
	{
		pvt->diff_status = DIFF_SOL_STATUS_RTD;
	}
	else
	{
		return;
	}


	for (k = 0; k < 3; k++)
	{
		pvt->ecef.pos[k] = psol->rr[k];
		pvt->ecef.vel[k] = (float)psol->rr[k + 3];
	}

	gnssConvEcef2Lla(pvt->ecef.pos, pvt->lla.pos);
	//calculate altitude to Mean sea level
	Nh = gnss_sd_get_geoId();
	pvt->altitudeMsl = pvt->lla.pos[2] - Nh;

	/* get ENU velocity */
	gnssConvEcef2EnuVel(pvt->ecef.vel, pvt->lla.vel, pvt->lla.pos);

	//set pos/vel std of rtk position fix
	gnssGetEcef2EnuMatrix(ceg, pvt->lla.pos);
	for (k = 0; k < 3; k++)
	{
		for (m = 0; m < 3; m++)
		{
			cge[k][m] = ceg[m][k];
		}
	}

	/* covert x,y,z variance to ENU */
	for (k = 0; k < 3; k++)
	{
		ecefPosVar[k][k] = psol->qr[k];
	}

	ecefPosVar[0][1] = ecefPosVar[1][0] = psol->qr[3];
	ecefPosVar[1][2] = ecefPosVar[2][1] = psol->qr[4];
	ecefPosVar[2][0] = ecefPosVar[0][2] = psol->qr[5];

	gnss_Matrix_Mult(ceg, ecefPosVar, matTmp);
	gnss_Matrix_Mult(matTmp, cge, enuPosVar);


	pvt->posErr.lat_err = (float)sqrt(enuPosVar[1][1]);
	pvt->posErr.lon_err = (float)sqrt(enuPosVar[0][0]);
	pvt->posErr.alt_err = (float)sqrt(enuPosVar[2][2]);

	pvt->accuracy = (float)sqrt(pvt->posErr.lat_err*pvt->posErr.lat_err + pvt->posErr.lon_err * pvt->posErr.lon_err);

	//calculate horizontal elliptical uncertainty
	gnss_Pe_Cal_HorizEllipseUnc(enuPosVar[0][0], enuPosVar[1][1], enuPosVar[0][1], pvt);

	//set pos fix accuracy confidence
	gnss_Pe_Pos_Confidence_Set(pvt);

	// fix time
	pvt->posfix_t = time2gpst(psol->time, &week);
	pvt->posfix_wn = (U16)week;

	// Fill ECEF position
	pvt->ecef.have_position = POS_FRM_KF;
	pvt->lla.have_position = POS_FRM_KF;
	pvt->ecef.posfix_t = pvt->posfix_t;
	pvt->ecef.posfix_wn = pvt->posfix_wn;

	pvt->have_position = HAVE_POS_FIX3D; // HAVE_POS_APPX

	gnss_pe_fill_fusion_mode(pvt);

	pvt->sol = *psol;
	*posVarXY = enuPosVar[0][1];
}

static int _nmea_pvt_to_oly(char *p_str, const USER_PVT *p_pvt)
{
	double utc_row[6],tow, dms1[3], dms2[3];
	int week;
	uint64_t TimeStampOfUtc;
	char *p = p_str, *q, sum;

	time2epoch(gpst2utc(p_pvt->sol.time), utc_row);
	tow = time2gpst(p_pvt->sol.time, &week);
	

	TimeStampOfUtc = (uint64_t)(p_pvt->utcTime.time * 1000 + p_pvt->utcTime.sec * 1000);
	//lat and lon cov
	deg2dms(fabs(p_pvt->lla.pos[0])*R2D, dms1, 7);
	deg2dms(fabs(p_pvt->lla.pos[1])*R2D, dms2, 7);
	//write an empty string with hhmmss
	p += sprintf(p_str, "%s,%d:%.3f,%d,%llu,%d,%s%02.0f%010.7f,%s%03.0f%010.7f,%.4f,%.2f,%.2f,%d,%.2f,%.1f,%d,%02d%02d%06.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.1f",
		"$GPOLY",week,tow,25,
		TimeStampOfUtc,
		solq_gga[p_pvt->sol.stat],
		p_pvt->lla.pos[0]<0.0? "-" : "",
		dms1[0], dms1[1] + dms1[2] / 60.0,
		p_pvt->lla.pos[1]<0.0? "-" : "",
		dms2[0], dms2[1] + dms2[2] / 60.0,
		p_pvt->lla.pos[2] - gnss_sd_geoidh(p_pvt->lla.pos),
		p_pvt->heading,
		p_pvt->velocity,
		p_pvt->usedSvNum,
		p_pvt->accuracy,
		p_pvt->DOP.hDOP,
		0,
		(int)(utc_row[3]), (int)(utc_row[4]), utc_row[5],
		1,
		p_pvt->lla.vel[1],
		p_pvt->lla.vel[0],
		-p_pvt->lla.vel[2],
		p_pvt->posErr.lat_err, 
		p_pvt->posErr.lon_err,
		p_pvt->posErr.alt_err,
		p_pvt->posErr.vn_err,
		p_pvt->posErr.ve_err,
		p_pvt->posErr.vd_err,
		p_pvt->DOP.pDOP,
		sqrtf(p_pvt->DOP.hDOP * p_pvt->DOP.hDOP + p_pvt->DOP.tDOP * p_pvt->DOP.tDOP),
		(double)(p_pvt->avgCNO));

	//check sum
	for (q = p_str + 1, sum = 0;*q;q++) sum ^= *q;

	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);

	return p - p_str;
}

/* forward/backward cfd fusion--------------------------------------------------
*args	: none
*return	: none
*author	: none
*note	:common type cfd fusion
*------------------------------------------------------------------------------*/
static void post_cfd_fusion(const USER_PVT* pvtf, const USER_PVT* pvtb, USER_PVT* pvt, double posVarXY)
{
	char fbv = 0;
	double fhcfd = 0, bhcfd = 0;
	double posVarX, posVarY;

	if (pvtf->accu.valid) fbv |= 1;
	if (pvtb->accu.valid) fbv |= 2;
	if (fbv == 3)
	{
		/*ave
		pvt->accu.accu1_east = (pvtf->accu.accu1_east + pvtb->accu.accu1_east) / 2;
		pvt->accu.accu1_north = (pvtf->accu.accu1_north + pvtb->accu.accu1_north) / 2;
		pvt->accu.accu1_up = (pvtf->accu.accu1_up + pvtb->accu.accu1_up) / 2;
		pvt->accu.accu2_east = (pvtf->accu.accu2_east + pvtb->accu.accu2_east) / 2;
		pvt->accu.accu2_north = (pvtf->accu.accu2_north + pvtb->accu.accu2_north) / 2;
		pvt->accu.accu2_up = (pvtf->accu.accu2_up + pvtb->accu.accu2_up) / 2;
		pvt->accu.accu3_east = (pvtf->accu.accu3_east + pvtb->accu.accu3_east) / 2;
		pvt->accu.accu3_north = (pvtf->accu.accu3_north + pvtb->accu.accu3_north) / 2;
		pvt->accu.accu3_up = (pvtf->accu.accu3_up + pvtb->accu.accu3_up) / 2;
		pvt->accu.valid = 1;
		pvt->accu.accu_type = pvtf->accu.accu_type;
		*/
		/*min
		fhcfd = sqrt(accuf->accu2_east*accuf->accu2_east + accuf->accu2_north*accuf->accu2_north);
		bhcfd = sqrt(accub->accu2_east*accub->accu2_east + accub->accu2_north*accub->accu2_north);
		if (fhcfd < bhcfd)
		{
		memcpy(accu,accuf,sizeof(navigation_accuracy_t));
		}
		else
		{
		memcpy(accu,accub,sizeof(navigation_accuracy_t));
		}
		*/
		/*max*/
		fhcfd = sqrt(pvtf->accu.accu2_east*pvtf->accu.accu2_east + pvtf->accu.accu2_north*pvtf->accu.accu2_north);
		bhcfd = sqrt(pvtb->accu.accu2_east*pvtb->accu.accu2_east + pvtb->accu.accu2_north*pvtb->accu.accu2_north);
		if (fhcfd > bhcfd && (g_pe_cfg.rinexDataType == RINEX_DATA_TYPE_UM4B0) && (g_pe_cfg.applyScenario == APPLY_SCENE_DRONE))
		{
			memcpy(&(pvt->accu), &(pvtb->accu), sizeof(navigation_accuracy_t));
		}
		else
		{
			memcpy(&(pvt->accu), &(pvtf->accu), sizeof(navigation_accuracy_t));
		}
		
	}
	else if (fbv == 1)
	{
		memcpy(&(pvt->accu),&(pvtf->accu),sizeof(navigation_accuracy_t));
	}
	else if (fbv == 2)
	{
		memcpy(&(pvt->accu),&(pvtb->accu),sizeof(navigation_accuracy_t));
	}
	else
	{
		gnss_Pe_Pos_Confidence_Set(pvt);
	}

	gnss_Pe_PosErr_Convert(pvt,&posVarXY);

	if (pvt->diff_status != DIFF_SOL_STATUS_NONE)
	{
		posVarX = pvt->posErr.lon_err * pvt->posErr.lon_err;
		posVarY = pvt->posErr.lat_err * pvt->posErr.lat_err;
		gnss_Pe_Cal_HorizEllipseUnc(posVarX, posVarY, posVarXY, pvt);
	}
}
static void post_cfd_log(const sol_t *sol, const USER_PVT* pvt)
{
	gtime_t time;
	double ep[6];
	time = gpst2utc(sol->time);
	if (time.sec >= 0.995) { time.time++; time.sec = 0.0; }
	time2epoch(time, ep);

	GLOGI("fusion cfd %d herr %02.0f%02.0f%06.3f %f uerr %f", sol->stat, ep[3], ep[4], ep[5],
		sqrt(pvt->posErr.lat_err*pvt->posErr.lat_err + pvt->posErr.lon_err*pvt->posErr.lon_err),
		pvt->posErr.alt_err);
}

#endif

/* validation of combined solutions ------------------------------------------*/
static int post_valcomb(const sol_t *solf, const sol_t *solb)
{
	double dr[3], var[3];
	int i;
	char tstr[32];

	/* compare forward and backward solution */
	for (i = 0;i<3;i++) {
		dr[i] = solf->rr[i] - solb->rr[i];
		var[i] = solf->qr[i] + solb->qr[i];
	}
	for (i = 0;i<3;i++) {
		if (dr[i] * dr[i] <= 16.0*var[i]) continue; /* ok if in 4-sigma */

		time2str(solf->time, tstr, 2);
		gnss_util_trace(2, "degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n",
			tstr + 11, dr[0], dr[1], dr[2], SQRT(var[0]), SQRT(var[1]), SQRT(var[2]));
		return 0;
	}
	return 1;
}

/* polynomial interpolation by Neville's algorithm ---------------------------*/
static double interppol(const double *x, double *y, int n)
{
	int i,j;

	for (j=1;j<n;j++) {
		for (i=0;i<n-j;i++) {
			y[i]=(x[i+j]*y[i]-x[i]*y[i+1])/(x[i+j]-x[i]);
		}
	}
	return y[0];
}
/* solution to covariance ----------------------------------------------------*/
static void post_sol2cov(const sol_t *sol, double *P)
{
	P[0]     =sol->qr[0]; /* xx or ee */
	P[4]     =sol->qr[1]; /* yy or nn */
	P[8]     =sol->qr[2]; /* zz or uu */
	P[1]=P[3]=sol->qr[3]; /* xy or en */
	P[5]=P[7]=sol->qr[4]; /* yz or nu */
	P[2]=P[6]=sol->qr[5]; /* zx or ue */
}
/* interpolate external event pos -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
static void post_interp_event_pos(const USER_PVT *p_pvt,const sol_t *sols,FILE *fp_nmea)
{
#ifdef ENABLE_PPK_MODE
	static gtime_t timetag[N_FIT_POINT];
	static double pos[12][N_FIT_POINT];
	static int ex_event_index=0;
	int i,j,len;
	double t[N_FIT_POINT],lla[3],event_pos[6],event_std[6],P[9],Q[9];;
	char gpstm[64] = { 0 }, nmea[1024] = {0};
	sol_t sol=*sols;
	USER_PVT pvt=*p_pvt;
	
	for (i = 0; i < N_FIT_POINT-1; i++)
	{
		timetag[i] = timetag[i+1];
		for (j = 0; j < 12; j++)
		{
			pos[j][i]=pos[j][i+1];
		}
	}
	timetag[N_FIT_POINT-1]=sols->time;
	ecef2pos(sols->rr,lla);
	pos[0][N_FIT_POINT-1]=sols->rr[0];
	pos[1][N_FIT_POINT-1]=sols->rr[1];
	pos[2][N_FIT_POINT-1]=sols->rr[2];

	pos[3][N_FIT_POINT-1]=lla[0]; //rad
	pos[4][N_FIT_POINT-1]=lla[1]; //rad
	pos[5][N_FIT_POINT-1]=lla[2]; //m

	post_sol2cov(sols,P);
	covenu(lla,P,Q);
	pos[6][N_FIT_POINT-1]=SQRT(Q[4]); //stdn
	pos[7][N_FIT_POINT-1]=SQRT(Q[0]); //stde
	pos[8][N_FIT_POINT-1]=SQRT(Q[8]); //stdu

	pos[9][N_FIT_POINT-1]=p_pvt->posErr.lat_err; //cfdn
	pos[10][N_FIT_POINT-1]=p_pvt->posErr.lon_err; //cfde
	pos[11][N_FIT_POINT-1]=p_pvt->posErr.alt_err; //cfdu

	if (timetag[0].time==0)//not enough point
	{
		return;
	}
	if (ex_event_index >= ex_event_cnt)
	{
		return;
	}
	if (timediff(timetag[0],ex_event_tag[ex_event_index]) > 0.0)
	{
		//make index allign
		for (ex_event_index++; ex_event_index < ex_event_cnt; ex_event_index++)
		{
			if (timediff(timetag[0], ex_event_tag[ex_event_index]) <= 0.0)
			{
				break;
			}
		}
	}
	if (timediff(timetag[N_FIT_POINT-1],ex_event_tag[ex_event_index]) < 0.0)
	{
		return;
	}
	//do interpolation
	if (timediff(ex_event_tag[ex_event_index],timetag[N_FIT_POINT/2 - 1]) > 0.0 &&
		timediff(ex_event_tag[ex_event_index],timetag[N_FIT_POINT/2]) < 0.0)
	{
		for (i = 0; i < N_FIT_POINT; i++)
		{
			t[i] = timediff(timetag[i],ex_event_tag[ex_event_index]);
		}
		for (i = 0;i < 6;i++) 
		{
			event_pos[i] = interppol(t,pos[i],N_FIT_POINT);
		}
		for (i = 6;i < 12;i++) 
		{
			event_std[i-6] = interppol(t,pos[i],N_FIT_POINT);
		}
		
		time2str(ex_event_tag[ex_event_index],gpstm,7);
		ecef2pos(event_pos,lla);
		GLOGI("ex_event %4d %s xyz=[ %15.3f %15.3f %15.3f ] lla=[ %15.8f %16.8f %10.3f ] x-l=[ %15.8f %16.8f %10.3f ] std=[ %6.3f %6.3f %6.3f ] cfd=[ %6.3f %6.3f %6.3f ]",
			ex_event_index+1,gpstm,
			event_pos[0],event_pos[1],event_pos[2],
			event_pos[3]*R2D,event_pos[4]*R2D,event_pos[5],
			lla[0]-event_pos[3],lla[1]-event_pos[4],lla[2]-event_pos[5],
			event_std[0],event_std[1],event_std[2],
			event_std[3],event_std[4],event_std[5]);

		sol.time = ex_event_tag[ex_event_index];
		sol.rr[0] = event_pos[0];
		sol.rr[1] = event_pos[1];
		sol.rr[2] = event_pos[2];
		sol.stat = SOLQ_FIX;
		pvt.sol = sol;
		pvt.utcTime = gpst2utc(sol.time);
		pvt.lla.pos[0] = lla[0];
		pvt.lla.pos[1] = lla[1];
		pvt.lla.pos[2] = lla[2];
		pvt.posErr.lat_err = event_std[3];
		pvt.posErr.lon_err = event_std[4];
		pvt.posErr.alt_err = event_std[5];
		if (fp_nmea && (len = outnmea_rmc(nmea, &sol)) > 0)
		{
			fwrite(nmea, sizeof(char), len, fp_nmea);
		}

		if (fp_nmea && (len = outnmea_gga(nmea, &sol)) > 0)
		{
			fwrite(nmea, sizeof(char), len, fp_nmea);
		}

		if (fp_nmea && (len = _nmea_pvt_to_oly(nmea, &pvt)) > 0)
		{
			fwrite(nmea, sizeof(char), len, fp_nmea);
		}

		ex_event_index++;
	}
#endif	
}
/* forward/backward solutions fusion -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void post_sol_fusion(FILE *fp_nmea,FILE *fp_event, const prcopt_t *popt, const solopt_t *sopt)
{
#ifdef ENABLE_PPK_MODE
	sol_t sols = { { 0 } };
	USER_PVT pvt = { 0 };
	char nmea[1024];
	double tt, Qf[9], Qb[9], Qs[9];
	int i, j, k, pri[] = { 10,1,3,7,2,4,7,7 }, len, fusion = 0;
	double posVarXY=0.0;

	printf("sol fusion : isolf=%d isolb=%d nepoch=%d\n", isolf, isolb, nepoch);
	GLOGI("sol fusion : isolf=%d isolb=%d nepoch=%d", isolf, isolb, nepoch);

	for (i = 0, j = isolb>0?(isolb - 1):0;i<isolf;i++, j--) {

		fusion = 0;
		if (isolb <= 0) //no valid reverse sol
		{
			pvt = solf[i];
			sols = pvt.sol;
			j++;
		}
		else if (isolb>0 && j<0)
		{
			pvt = solf[i];
			sols = pvt.sol;
		}
		else if ((tt = timediff(solf[i].sol.time, solb[j].sol.time))<-DTTOL) {
			pvt = solf[i];
			sols = pvt.sol;
			j++;
		}
		else if (tt>DTTOL) {
			pvt = solb[j];
			sols = pvt.sol;
			i--;
		}
		else if (pri[solf[i].sol.stat]<pri[solb[j].sol.stat]) {
			pvt = solf[i];
			sols = pvt.sol;
		}
		else if (pri[solf[i].sol.stat]>pri[solb[j].sol.stat]) {
			pvt = solb[j];
			sols = pvt.sol;
		}
		else {
			pvt = solf[i];
			sols = pvt.sol;
			sols.time = timeadd(sols.time, -tt / 2.0);

			if ((popt->mode == PMODE_KINEMA || popt->mode == PMODE_MOVEB) &&
				sols.stat == SOLQ_FIX) {

				/* degrade fix to float if validation failed */
				//if (!post_valcomb(&solf[i].sol, &solb[j].sol)) sols.stat = SOLQ_FLOAT;
			}
			for (k = 0;k<3;k++) {
				Qf[k + k * 3] = solf[i].sol.qr[k];
				Qb[k + k * 3] = solb[j].sol.qr[k];
			}
			Qf[1] = Qf[3] = solf[i].sol.qr[3];
			Qf[5] = Qf[7] = solf[i].sol.qr[4];
			Qf[2] = Qf[6] = solf[i].sol.qr[5];
			Qb[1] = Qb[3] = solb[j].sol.qr[3];
			Qb[5] = Qb[7] = solb[j].sol.qr[4];
			Qb[2] = Qb[6] = solb[j].sol.qr[5];

			if (smoother(solf[i].sol.rr, Qf, solb[j].sol.rr, Qb, 3, sols.rr, Qs)) continue;
			sols.qr[0] = (float)Qs[0];
			sols.qr[1] = (float)Qs[4];
			sols.qr[2] = (float)Qs[8];
			sols.qr[3] = (float)Qs[1];
			sols.qr[4] = (float)Qs[5];
			sols.qr[5] = (float)Qs[2];

			fusion = 1;
		}

		//outsol(fp_nmea, &sols, NULL, sopt);

		if (fusion)
		{
			_sol_fill_pvt(&sols, &pvt, &posVarXY);
			post_cfd_fusion(&(solf[i]), &(solb[j]), &(pvt), posVarXY);
		}

		if ((len = outnmea_rmc(nmea, &sols)) > 0)
		{
			fwrite(nmea, sizeof(char), len, fp_nmea);
		}

		if ((len = outnmea_gga(nmea, &sols)) > 0)
		{
			fwrite(nmea, sizeof(char), len, fp_nmea);
		}

		if ((len = _nmea_pvt_to_oly(nmea, &pvt)) > 0)
		{
			fwrite(nmea, sizeof(char), len, fp_nmea);
		}

		post_cfd_log(&sols, &pvt);
		//external event process
		if (fp_event) post_interp_event_pos(&pvt,&sols,fp_event);
	}

#endif
}

/* average of single position ------------------------------------------------*/
static int avepos(double *ra, int rcv, const obs_t *obs, const nav_t *nav,
                  const prcopt_t *opt)
{
    obsd_t data[MAXOBS];
    gtime_t ts={0};
    sol_t sol={{0}};
    int i,j,n=0,m,iobs;
    char msg[128];
    
    gnss_util_trace(3,"avepos: rcv=%d obs.n=%d\n",rcv,obs->n);
    
    for (i=0;i<3;i++) ra[i]=0.0;
    
    for (iobs=0;(m=nextobsf(obs,&iobs,rcv))>0;iobs+=m) {
        
        for (i=j=0;i<m&&i<MAXOBS;i++) {
            data[j]=obs->data[iobs+i];
            if ((satsys(data[j].sat,NULL)&opt->navsys)&&
                opt->exsats[data[j].sat-1]!=1) j++;
        }
        if (j<=0||!screent(data[0].time,ts,ts,1.0)) continue; /* only 1 hz */
        
        if (!ag_gnss_pntpos(data,j,nav,opt,&sol,NULL,NULL,msg)) continue;
        
        for (i=0;i<3;i++) ra[i]+=sol.rr[i];
        n++;
    }
    if (n<=0) {
        gnss_util_trace(1,"no average of base station position\n");
        return 0;
    }
    for (i=0;i<3;i++) ra[i]/=n;
    return 1;
}
/* station position from file ------------------------------------------------*/
static int getstapos(const char *file, char *name, double *r)
{
    FILE *fp;
    char buff[256],sname[256],*p,*q;
    double pos[3];
    
    gnss_util_trace(3,"getstapos: file=%s name=%s\n",file,name);
    
    if (!(fp=fopen(file,"r"))) {
        gnss_util_trace(1,"station position file open error: %s\n",file);
        return 0;
    }
    while (fgets(buff,sizeof(buff),fp)) {
        if ((p=strchr(buff,'%'))) *p='\0';
        
        if (sscanf(buff,"%lf %lf %lf %s",pos,pos+1,pos+2,sname)<4) continue;
        
        for (p=sname,q=name;*p&&*q;p++,q++) {
            if (toupper((int)*p)!=toupper((int)*q)) break;
        }
        if (!*p) {
            pos[0]*=D2R;
            pos[1]*=D2R;
            pos2ecef(pos,r);
            fclose(fp);
            return 1;
        }
    }
    fclose(fp);
    gnss_util_trace(1,"no station position: %s %s\n",name,file);
    return 0;
}
/* antenna phase center position ---------------------------------------------*/
static int antpos(prcopt_t *opt, int rcvno, const obs_t *obs, const nav_t *nav,
                  const sta_t *sta, const char *posfile)
{
    double *rr=rcvno==1?opt->ru:opt->rb,del[3],pos[3],dr[3]={0};
    int i,postype=rcvno==1?opt->rovpos:opt->refpos;
    char *name;
    
    gnss_util_trace(3,"antpos  : rcvno=%d\n",rcvno);
    
    if (postype==POSOPT_SINGLE) { /* average of single position */
        if (!avepos(rr,rcvno,obs,nav,opt)) {
            showmsg("error : station pos computation");
            return 0;
        }
    }
    else if (postype==POSOPT_FILE) { /* read from position file */
        name=stas[rcvno==1?0:1].name;
        if (!getstapos(posfile,name,rr)) {
            showmsg("error : no position of %s in %s",name,posfile);
            return 0;
        }
    }
    else if (postype==POSOPT_RINEX) { /* get from rinex header */
        if (asg_norm(stas[rcvno==1?0:1].pos,3)<=0.0) {
            showmsg("error : no position in rinex header");
            gnss_util_trace(1,"no position position in rinex header\n");
            return 0;
        }
        /* antenna delta */
        if (stas[rcvno==1?0:1].deltype==0) { /* enu */
            for (i=0;i<3;i++) del[i]=stas[rcvno==1?0:1].del[i];
            del[2]+=stas[rcvno==1?0:1].hgt;
            ecef2pos(stas[rcvno==1?0:1].pos,pos);
            enu2ecef(pos,del,dr);
        }
        else { /* xyz */
            for (i=0;i<3;i++) dr[i]=stas[rcvno==1?0:1].del[i];
        }
        for (i=0;i<3;i++) rr[i]=stas[rcvno==1?0:1].pos[i]+dr[i];
    }
    return 1;
}

#endif


#endif

