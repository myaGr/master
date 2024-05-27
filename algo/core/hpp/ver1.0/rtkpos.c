/*------------------------------------------------------------------------------
* rtkpos.c : precise positioning
*
*          Copyright (C) 2007-2015 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/12 1.0  new
*           2007/03/13 1.1  add slip detection by LLI flag
*           2007/04/18 1.2  add antenna pcv correction
*                           change rtkpos argin
*           2008/07/18 1.3  refactored
*           2009/01/02 1.4  modify rtk positioning api
*           2009/03/09 1.5  support glonass, gallileo and qzs
*           2009/08/27 1.6  fix bug on numerical exception
*           2009/09/03 1.7  add check of valid satellite number
*                           add check time sync for moving-base
*           2010/04/04 1.9  support ppp-kinematic and ppp-static modes
*                           support earth tide correction
*                           changed api:
*                               rtkpos()
*           2010/09/07 1.10 add elevation mask to hold ambiguity
*           2012/02/01 1.11 add extended receiver error model
*                           add glonass interchannel bias correction
*                           add slip detectior by L1-L5 gf jump
*                           output snr of rover receiver in residuals
*           2013/03/10 1.12 add otl and pole tides corrections
*           2014/05/26 1.13 support beidou and galileo
*                           add output of gal-gps and bds-gps time offset
*           2014/05/28 1.14 fix bug on memory exception with many sys and freq
*           2014/08/26 1.15 add functino to swap sol-stat file with keywords
*           2014/10/21 1.16 fix bug on beidou amb-res with pos2-bdsarmode=0
*           2014/11/08 1.17 fix bug on ar-degradation by unhealthy satellites
*           2015/03/23 1.18 residuals referenced to reference satellite
*           2015/05/20 1.19 no output solution status file with Q=0
*           2015/07/22 1.20 fix bug on base station position setting
*           2016/07/30 1.21 suppress single solution if !prcopt.outsingle
*                           fix bug on slip detection of backward filter
*           2016/08/20 1.22 fix bug on ddres() function
*-----------------------------------------------------------------------------*/
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#endif
#include <stdarg.h>
#include "macro.h"
#include "rtklib.h"
#include "gnss_sys_api.h"
#include "gnss_pe.h"
#include "gnss_sd.h"
#include "gnss_mode.h"
#include "gnss_rtk.h"
#include "gnss_math.h"
#include "gnss_qos.h"
#include "gnss_cfd.h"
#include "gnss_kf_algo.h"
#include "rtk_seq_ekf.h"

#ifdef AG_GNSS_RTK_FUNCTION_IMPL

#undef MODULE_TAG
#define MODULE_TAG OBJ_RTK

/* extern global data */
extern PE_MODES  peMode;
extern Kf_t*      gpz_kf_data;  //import from PE module
extern pe_rtk_data_t rtcm3data;  //rtcm3 data from NRTK svr
extern Gnss_Cfg_t  g_pe_cfg;
extern uncertainty_t g_unc;
static uint8_t glo_ar_freq_mask = 0; /* bit0:L1 bit2:L2 */
int static_flag = 1;
extern VDR_Gnss_Feedback_t          vdrFeedbackInfo;
extern rtk_t rtkctrl;            //rtk control and results

proc_ctrl_t g_proc;

static const char rcsid[]="$Id:$";

/* constants/macros ----------------------------------------------------------*/

#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))
#define MIN(x,y)    ((x)<=(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)

#define VAR_POS     SQR(30.0) /* initial variance of receiver pos (m^2) */
#define VAR_VEL     SQR(10.0) /* initial variance of receiver vel ((m/s)^2) */
#define VAR_ACC     SQR(10.0) /* initial variance of receiver acc ((m/ss)^2) */
#define VAR_HWBIAS  SQR(1.0)  /* initial variance of h/w bias ((m/MHz)^2) */
#define VAR_GRA     SQR(0.001) /* initial variance of gradient (m^2) */
#define INIT_ZWD    0.15     /* initial zwd (m) */
#define VAR_RCV_CLK SQR(100.0) /* initial variance of receiver clock (m^2) */

#define PRN_HWBIAS  1E-6     /* process noise of h/w bias (m/MHz/sqrt(s)) */
#define GAP_RESION  120      /* gap to reset ionosphere parameters (epochs) */
#define MAXACC      30.0     /* max accel for doppler slip detection (m/s^2) */

#define VAR_HOLDAMB 0.001    /* constraint to hold ambiguity (cycle^2) */

#define TTOL_MOVEB  (1.0+2*DTTOL)
                             /* time sync tolerance for moving-baseline (s) */

/* locol glonass cpb info, unit: m */
#define LOCAL_GLO_CPB_UBLOX_M8       (0.0)
#define LOCAL_GLO_CPB_UBLOX_F9       (0.0)
#define LOCAL_GLO_CPB_ST_8090        (0.0)
#define LOCAL_GLO_CPB_ST_8100        (0.0)
#define LOCAL_GLO_CPB_QCOM           (0.0)
#define LOCAL_GLO_CPB_BRCM_47755     (0.0)

typedef struct
{
	int sat_id;
	int freq;
	int amb_index;
	int ref_amb_index;
}fix_sat_info;

#ifdef EXTGSI

extern int ag_rtk_resamb_WLNL(rtk_t *rtk, const obsd_t *obs, const int *sat,
                       const int *iu, const int *ir, int ns, const nav_t *nav,
                       const double *azel);
extern int ag_rtk_resamb_TCAR(rtk_t *rtk, const obsd_t *obs, const int *sat,
                       const int *iu, const int *ir, int ns, const nav_t *nav,
                       const double *azel);
#else

extern int ag_rtk_resamb_WLNL(rtk_t *rtk, const obsd_t *obs, const int *sat,
                       const int *iu, const int *ir, int ns, const nav_t *nav,
                       const double *azel) {return 0;}
extern int ag_rtk_resamb_TCAR(rtk_t *rtk, const obsd_t *obs, const int *sat,
                       const int *iu, const int *ir, int ns, const nav_t *nav,
                       const double *azel) {return 0;}
#endif

/* test navi system (0:gps/qzs/sbs,1:glo,2:gal,3:bds, -1:error) ----------------------*/
static int get_sys_index(int sys)
{
	switch (sys) {
	case SYS_GPS:
	case SYS_SBS: return 0;
	case SYS_GLO: return 1;
	case SYS_GAL: return 2;
	case SYS_CMP: return 3;
	case SYS_QZS: return (NSYSDD == 4 ? 0 : 4);
	default: return -1;
	}
	return -1;
}
/* single-differenced observable ---------------------------------------------*/
static double sdobs(const obsd_t *obs, int i, int j, int f)
{
    double pi=f<NFREQ?obs[i].L[f]:obs[i].P[f-NFREQ];
    double pj=f<NFREQ?obs[j].L[f]:obs[j].P[f-NFREQ];
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
#if 0
/* single-differenced geometry-free linear combination of phase --------------*/
static double gfobs_L1L2(const obsd_t *obs, int i, int j, const double *lam)
{
    double pi=sdobs(obs,i,j,0)*lam[0],pj=sdobs(obs,i,j,1)*lam[1];
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
static double gfobs_L1L5(const obsd_t *obs, int i, int j, const double *lam)
{
    double pi=sdobs(obs,i,j,0)*lam[0],pj=sdobs(obs,i,j,2)*lam[2];
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
#endif
/* single-differenced measurement error variance -----------------------------*/
static double varerr(int sat, int sys, double el, double bl, double dt, int f,
                     const prcopt_t *opt)
{
    double a,b,c=opt->err[3]*bl/1E4,d=CLIGHT*opt->sclkstab*dt,fact=1.0;
    double sinel=sin(el);
    int i=sys==SYS_GLO?1:(sys==SYS_GAL?2:0),nf=NF(opt);
    
    /* extended error model */
    if (f>=nf&&opt->exterr.ena[0]) { /* code */
        a=opt->exterr.cerr[i][  (f-nf)*2];
        b=opt->exterr.cerr[i][1+(f-nf)*2];
        if (sys==SYS_SBS) {a*=EFACT_SBS; b*=EFACT_SBS;}
    }
    else if (f<nf&&opt->exterr.ena[1]) { /* phase */
        a=opt->exterr.perr[i][  f*2];
        b=opt->exterr.perr[i][1+f*2];
        if (sys==SYS_SBS) {a*=EFACT_SBS; b*=EFACT_SBS;}
    }
    else { /* normal error model */
        if (f>=nf) fact=opt->eratio[f-nf];
        if (fact<=0.0)  fact=opt->eratio[0];
        fact*=sys==SYS_GLO?EFACT_GLO:(sys==SYS_SBS?EFACT_SBS:EFACT_GPS);
        a=fact*opt->err[1];
        b=fact*opt->err[2];
    }
    return 2.0*(opt->ionoopt==IONOOPT_IFLC?3.0:1.0)*(a*a+b*b/sinel/sinel+c*c)+d*d;
}
/* baseline length -----------------------------------------------------------*/
static double baseline(const double *ru, const double *rb, double *dr)
{
    int i;
    for (i=0;i<3;i++) dr[i]=ru[i]-rb[i];
    return asg_norm(dr,3);
}

/* initialize covariance -------------------------------------------*/
static void initp(rtk_t *rtk, double var, int i)
{
	//int j;
	//for (j = 0;j<rtk->nx;j++) {
	//	rtk->P[i + j * rtk->nx] = rtk->P[j + i * rtk->nx] = i == j ? var : 0.0;
	//}

	int j, ix = rtk->ix[i];

	if (rtk->ix[i] > -1) {
		for (j = 0; j < rtk->nx; j++) {
			rtk->P[ISM(ix, j)] = rtk->P[ISM(j, ix)] = ix == j ? var : 0.0;
		}
		if (var == 0.0) rtk->ix[i] = -1;
	}
	else {
		for (j = 0; j < rtk->nx; j++) {
			if (rtk->P[ISM(j, j)] > 0.0) {
				continue;
			}
			rtk->P[ISM(j, j)] = var;
			rtk->ix[i] = var == 0.0 ? -1 : j;
			break;
		}
	}
}
extern void gnss_rtk_init_ambp(rtk_t *rtk, int sat, int f)
{
	if (rtk->ix[IB(sat, f, &rtk->opt)] > -1)
	{
		initp(rtk, SQR(rtk->opt.std[0]), IB(sat, f, &rtk->opt));
	}
}
/* offset to kf amb state -------------------------------------------------------------
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slip_offset(rtk_t *rtk, int sat, int f,double slip)
{
	int i = IB(sat, f, &rtk->opt);

	if (rtk->ix[i] < 0) return;
	rtk->x[rtk->ix[i]] -= slip;
}
/* offset to prefix amb -------------------------------------------------------------
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_prefix_offset(rtk_t *rtk, int sat, int sys, int f,double slip)
{ 
	double offset;
	int m = get_sys_index(sys);

	if (m<0 || !(rtk->fixstate[sat-1]&FREQ_FIX(f))) {
		return;
	}

	offset = floor(slip+0.5);

	//GLOGI("prefix offset: %9.3f %9.3f %9.3f", slip, offset, slip-offset);

	if (fabs(slip-offset) > 0.2)
	{
		rtk->fixstate[sat-1] &= ~FREQ_FIX(f);
		rtk->prefixn--;
		rtk->biasnum[m]--;
		return;
	}

	if (rtk->ssat[sat-1] != NULL)
	{
		rtk->ssat[sat-1]->sdamb[f] += offset;
	}

}
/* initialize state and covariance -------------------------------------------*/
static void initx(rtk_t *rtk, double xi, double var, int i)
{
    //int j;
    //rtk->x[i]=xi;
    //for (j=0;j<rtk->nx;j++) {
    //   rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;

	int j, ix = rtk->ix[i];

	if (rtk->ix[i] > -1) {
		rtk->x[ix] = xi;
		for (j = 0; j < rtk->nx; j++) {
			rtk->P[ISM(ix, j)] = rtk->P[ISM(j, ix)] = ix == j ? var : 0.0;
		}
		if (var == 0.0) rtk->ix[i] = -1;
	}
	else {
		for (j = 0; j < rtk->nx; j++) {
			if (rtk->P[ISM(j, j)] > 0.0) {
				continue;
			}
			rtk->x[j] = xi;
			rtk->P[ISM(j, j)] = var;
			rtk->ix[i] = var == 0.0 ? -1 : j;
			break;
		}
	}
}

/* select common satellites between rover and reference station --------------*/
static void ag_gnss_check_freq_valid(rtk_t *rtk, const obsd_t *obs, int sat, int i, int j)
{
	int f, freq_i = 0, freq_j = 0, nf = NF(&(rtk->opt));
	//char *stri, *strj;
	int freq_mask = 0x7; //default all freq valid

	for (f=0;f<nf;f++)
	{
		//stri=(char*)code2obs(obs[i].code[f], &freq_i);
		//strj=(char*)code2obs(obs[j].code[f], &freq_j);
		if (freq_i<=0 || freq_j<=0 || freq_i!=freq_j)
		{
			freq_mask &= ~FREQ_MASK(f);
			continue;
		}

		/*if (rtk->ssat[sat-1].sys==SYS_CMP)
		{
			if ((stri[0]=='6' && strj[0]=='5') ||
				(stri[0]=='5' && strj[0]=='6'))
			{
				freq_mask &= ~FREQ_MASK(f);
			}
		}*/
	}
	if (rtk->ssat[sat - 1])
	{
		rtk->ssat[sat - 1]->freq_valid = (uint8_t)freq_mask;
	}
}
/* select common satellites between rover and reference station --------------*/
static int ag_gnss_selsat(rtk_t *rtk, const obsd_t *obs, double *azel, int nu, int nr,
                  const prcopt_t *opt, int *sat, int *iu, int *ir)
{
    int i,j,k=0;
    
    for (i=0,j=nu;i<nu&&j<nu+nr;i++,j++) {
        if      (obs[i].sat<obs[j].sat) j--;
        else if (obs[i].sat>obs[j].sat) i--;
        else if (azel[1+j*2]>=opt->elmin) { /* elevation at base station */
            sat[k]=obs[i].sat; iu[k]=i; ir[k++]=j;
            gnss_util_trace(3,"(%2d) sat=%3d iu=%2d ir=%2d\n",k-1,obs[i].sat,i,j);
            ag_gnss_check_freq_valid(rtk, obs, obs[i].sat, i, j);
        }
    }
    return k;
}
/* kalman filter predict -------------------------------------------------------------
* 
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
static void filter_predict(rtk_t *rtk, double tt)
{
	double /**x=rtk->x,*/*F=rtk->F;
	double *F_,*P_,*Pp_,*FP;
	int i,j,k,*ix,n=rtk->nx;
	double Fr[NX_PVA*NX_PVA]={0.0},xp[NX_PVA]={0.0};
	double	xAcc[NX_PVA] = { 0.0 };
#if defined(PLAYBACK_MODE)
	double	xpAcc[NX_PVA] = { 0.0 };
	float  fVelErrorRaw[8] = { 0 };
	float  fVelErrorPred[8] = { 0 };
#endif // !
	matcpy(xAcc, rtk->x, NX_PVA, 1);
	//if (uAccFlpFeedback && uEleFlpFeedback)
	//{
	//	rtk->x[6] = flpFeedbackInfo.fAccIne[0];
	//	rtk->x[7] = flpFeedbackInfo.fAccIne[1];
	//	rtk->x[8] = flpFeedbackInfo.fAccIne[2];
	//}
	//x
	for (i=0;i<NX_PVA;i++) for (j=0;j<NX_PVA;j++)  Fr[i+j*NX_PVA]=F[i+j*NX_PVA];
#ifndef ASM_MATMUL
	matmul("NN",NX_PVA,1,NX_PVA,1.0,Fr,rtk->x,0.0,xp);
#else
	Fast_Rtk_matmul("NN", NX_PVA, 1, NX_PVA, 1.0, Fr, rtk->x, 0.0, xp);
#endif
	matcpy(rtk->x,xp,NX_PVA,1);

#if defined(PLAYBACK_MODE)
	if (gpz_kf_data->meas_blk&&gpz_kf_data->meas_blk->hpRefQflag[1] > 0)
	{
#ifndef ASM_MATMUL
		matmul("NN", NX_PVA, 1, NX_PVA, 1.0, Fr, xAcc, 0.0, xpAcc);
#else
		Fast_Rtk_matmul("NN", NX_PVA, 1, NX_PVA, 1.0, Fr, xAcc, 0.0, xpAcc);
#endif
		fVelErrorRaw[0] = (float)(xpAcc[3] - gpz_kf_data->meas_blk->hpRefEcef[3]);
		fVelErrorRaw[1] = (float)(xpAcc[4] - gpz_kf_data->meas_blk->hpRefEcef[4]);
		fVelErrorRaw[2] = (float)(xpAcc[5] - gpz_kf_data->meas_blk->hpRefEcef[5]);
		gnssConvEcef2EnuVel(fVelErrorRaw, fVelErrorRaw + 4, gpz_kf_data->meas_blk->hpRefLla);
		fVelErrorRaw[7] = sqrtf(fVelErrorRaw[4] * fVelErrorRaw[4] + fVelErrorRaw[5] * fVelErrorRaw[5]);

		fVelErrorPred[0] = (float)(rtk->x[3] - gpz_kf_data->meas_blk->hpRefEcef[3]);
		fVelErrorPred[1] = (float)(rtk->x[4] - gpz_kf_data->meas_blk->hpRefEcef[4]);
		fVelErrorPred[2] = (float)(rtk->x[5] - gpz_kf_data->meas_blk->hpRefEcef[5]);
		gnssConvEcef2EnuVel(fVelErrorPred, fVelErrorPred + 4, gpz_kf_data->meas_blk->hpRefLla);
		fVelErrorPred[7] = sqrtf(fVelErrorPred[4] * fVelErrorPred[4] + fVelErrorPred[5] * fVelErrorPred[5]);
		GLOGI("PE Use INS vel Pos to calculate KFPOS diff:%8.3f %d %d %d %8.4f %8.4f", gpz_kf_data->meas_blk->tor, peMode.feedbackFlag.uVelVdrFeedback, peMode.feedbackFlag.uEleVdrFeedback, peMode.feedbackFlag.uAccVdrFeedback, fVelErrorRaw[7], fVelErrorPred[7]);
	}
#endif // !
	//P
	ix = imat(n, 1); 
	if (ix == NULL) {
		return;
	}
	for (i = k = 0; i < n; i++) {
		if (rtk->P[ISM(i, i)] > 0.0) ix[k++] = i;
	}
	F_=mat(k,k); P_=mat(k,k); 
	for (i=0;i<k;i++) 
	for (j=0;j<k;j++) 
	{
		if (ix[i]<NX_PVA&&ix[j]<NX_PVA)
		{
			F_[i+j*k]=F[ix[i]+ix[j]*NX_PVA];
		}
		else if (ix[i]==ix[j])
		{
			F_[i+j*k]=1.0;
		}
		else
		{
			F_[i+j*k]=0.0;
		}
		P_[i+j*k]=rtk->P[ISM(ix[i], ix[j])];
	}
	
	FP = mat(k, k); 
#ifndef ASM_MATMUL
	matmul("NN",k,k,k,1.0,F_,P_,0.0,FP);
#else
	Fast_Rtk_matmul("NN", k, k, k, 1.0, F_, P_, 0.0, FP);
#endif
	Sys_Free(P_);
	Pp_ = mat(k, k); 
#ifndef ASM_MATMUL
	matmul("NT",k,k,k,1.0,FP,F_,0.0,Pp_);
#else
	Fast_Rtk_matmul("NT", k, k, k, 1.0, FP, F_, 0.0, Pp_);
#endif
	Sys_Free(F_); Sys_Free(FP);

	for (i=0;i<k;i++) {
		for (j=0;j<k;j++) {
			rtk->P[ISM(ix[i], ix[j])] = Pp_[i + j * k];
		}
	}
	Sys_Free(ix);  Sys_Free(Pp_);
}
static void asensing_rtk_kf_add_rcv_clk(rtk_t* rtk, double tt, const double pseudo_rcv_clock[NFREQ * NSYSDD])
{
	double dcb_init_value = 0.0, dcb_sigma = VAR_RCV_CLK, enfactor = 20.0 * 20.0;
	int i = 0, nf = NF(&rtk->opt), freq1_index = 0;
	int n_clock = nf * NSYSDD;
	gnss_util_trace(3, "asensing_rtk_kf_add_rcv_clk   : tt=%.3f\n", tt);
	for (i = 0; i < n_clock; i++)
	{
		freq1_index = i % NSYSDD;
		if (i < NSYSDD)
		{
			initx(rtk, pseudo_rcv_clock[i], VAR_RCV_CLK, IRC(i, &rtk->opt));
		}
		else if (rtk->ix[IRC(i, &rtk->opt)] < 0)
		{
			dcb_init_value = 0.0;
			if (0.0 != pseudo_rcv_clock[i] && 0.0 != pseudo_rcv_clock[freq1_index])
			{
				dcb_init_value = pseudo_rcv_clock[i] - pseudo_rcv_clock[freq1_index];
				dcb_sigma = VAR_RCV_CLK;
			}
			else
			{
				dcb_sigma = VAR_RCV_CLK * enfactor;
			}
			initx(rtk, dcb_init_value, dcb_sigma, IRC(i, &rtk->opt));
		}
	}
	return;
}
static void asensing_rtk_kf_update_rcv_clk(rtk_t* rtk, double tt, const double pseudo_rcv_clock[NFREQ * NSYSDD])
{
	double dcb_init_value = 0.0, dcb_sigma = VAR_RCV_CLK, enfactor = 20.0 * 20.0;
	int i = 0, nf = NF(&rtk->opt), freq1_index = 0, index = 0;
	int n_clock = nf * NSYSDD;
	gnss_util_trace(3, "asensing_rtk_kf_update_rcv_clk   : tt=%.3f\n", tt);
	for (i = 0; i < n_clock; i++)
	{
		index = rtk->ix[IRC(i, &rtk->opt)];
		if (index < 0)
		{
			continue;
		}
		freq1_index = i % NSYSDD;
		if (i < NSYSDD)
		{
			rtk->x[index] = pseudo_rcv_clock[i];
			rtk->P[ISM(index, index)] = VAR_RCV_CLK;
		}
		else
		{
			dcb_init_value = 0.0;
			if (0.0 != pseudo_rcv_clock[i] && 0.0 != pseudo_rcv_clock[freq1_index])
			{
				dcb_init_value = pseudo_rcv_clock[i] - pseudo_rcv_clock[freq1_index];
				dcb_sigma = VAR_RCV_CLK;
			}
			else
			{
				dcb_sigma = VAR_RCV_CLK * enfactor;
			}
			if (1 == rtk->dcb_para_init || fabs(rtk->x[index] - dcb_init_value) > 10.0)
			{
				rtk->x[index] = dcb_init_value;
				rtk->P[ISM(index, index)] = dcb_sigma;
			}
		}
	}
	return;
}
/* temporal update of position/velocity/acceleration -------------------------*/
static void ag_rtk_kf_udpos(rtk_t *rtk, double tt)
{
    double var=0.0, pseudo_rcv_clk[NSYSDD * NFREQ] = { 0.0 };
    int i,j,f,nf=NF(&rtk->opt);
    
    gnss_util_trace(3,"ag_rtk_kf_udpos   : tt=%.3f\n",tt);
    
	rtk->kf_tor = rtk->ptor;
    /* fixed mode */
    if (rtk->opt.mode==PMODE_FIXED) {
        for (i=0;i<3;i++) initx(rtk,rtk->opt.ru[i],1E-8,i);
        return;
    }
    /* initialize position for first epoch */
    if (rtk->kfstatus==RTK_KF_INIT||rtk->kfstatus==RTK_KF_RESET||asg_norm(rtk->x,3)<=0.0) 
    {
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        if (rtk->opt.dynamics) {
            for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
#ifndef RTK_PVMODE
            for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
#endif
        }
		if (rtk->opt.adaptSeqEKF)
		{
			asensing_rtk_kf_add_rcv_clk(rtk, tt, pseudo_rcv_clk);
			rtk->dcb_para_init = 1;
		}
        rtk->kfstatus=RTK_KF_RUN|RTK_KF_RESET_AMB;
        rtk->kfcnt=0;
		rtk->kfstartwait=0;
		rtk->shift.adjPcnt=0;
		rtk->shift.nofixc=0;
		rtk->shift.nofixcnt = 0;
		rtk->nfix = 0;
		rtk->shift.feedback_flag = FALSE;
		rtk->shift.last_feedback_flag = FALSE;
		return;
    }
    /* static mode */
    if (rtk->opt.mode==PMODE_STATIC) return;
    
    /* kinmatic mode without dynamics */
    if (!rtk->opt.dynamics) {
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        return;
    }
    /* check variance of estimated postion */
		for (i = 0; i < 3; i++)
		{
			var += rtk->P[ISM(i, i)];
		}
		var/=3.0;
    
    if (var>VAR_POS) {
        /* reset position with large variance */
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
#ifndef RTK_PVMODE
        for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
#endif
        gnss_util_trace(2,"reset rtk position due to large variance: var=%.3f\n",var);
        return;
    }

	rtk->F = eye(NX_PVA);
	rtk->Q = zeros(NX_PVA, NX_PVA);
	gnss_rtk_InitF(rtk->updFQtt);
	gnss_rtk_InitQ(rtk->updFQtt, NULL);

	if (static_flag == 1)
	{
		if (/*((peMode.staticData.historyStatic & 0xF) == 0x0)
			|| */peMode.staticData.lsVel >= 0.1 || peMode.staticData.deltaDoplVar >= 0.1)
			static_flag = 0;
	}

	if (static_flag == 1)
	{
		for (i = 0; i < 3; i++)
		{
			if (rtk->P[ISM(i, i)] < 1e-3)
				rtk->P[ISM(i, i)] += 1e-3;
		}
		gnss_util_trace(3, "static mode: tor=%.3f lsVel=%.2f DoplVar=%.2f\n", rtk->ptor, peMode.staticData.lsVel, peMode.staticData.deltaDoplVar);
	}
	else
	{
		filter_predict(rtk,tt);
		for (i = 0; i < 3; i++)
		{
			rtk->P[ISM(i, i)] += rtk->Q[i + i * NX_PVA];  //q11;
			rtk->P[ISM((i + 3), (i + 3))] += rtk->Q[(i + 3) + (i + 3)*NX_PVA]; //q22;
			rtk->P[ISM(i, (i + 3))] += rtk->Q[(i)+(i + 3)*NX_PVA]; //q12;
		#ifndef RTK_PVMODE
			rtk->P[ISM((i + 6), (i + 6))] += rtk->Q[(i + 6) + (i + 6)*NX_PVA]; //q33;
			rtk->P[ISM(i, (i + 6))] += rtk->Q[(i)+(i + 6)*NX_PVA]; //q13;
			rtk->P[ISM((i + 3), (i + 6))] += rtk->Q[(i + 3) + (i + 6)*NX_PVA]; //q23;
		#endif
		}
	}

	Sys_Free(rtk->F); Sys_Free(rtk->Q);

	for (f=0;f<nf;f++)
	{
		for (i = 1; i <= MAXSAT; i++)
		{
			j = IB(i, f, &rtk->opt);
			if (rtk->ix[j] < 0) {
				continue;
			}
			rtk->P[ISM(rtk->ix[j], rtk->ix[j])] += rtk->opt.prn[0] * rtk->opt.prn[0] * fabs(tt);
		}
	}

	
}
/* temporal update of ionospheric parameters ---------------------------------*/
static void udion(rtk_t *rtk, double tt, double bl, const int *sat, int ns)
{
    double el,fact;
    int i,j;
    
    gnss_util_trace(3,"udion   : tt=%.1f bl=%.0f ns=%d\n",tt,bl,ns);
    
    for (i=1;i<=MAXSAT;i++) {
        j=II(i,&rtk->opt);
        if (rtk->ssat[i - 1] == NULL)
        {
            continue;
        }
        if (rtk->x[j]!=0.0&&
            rtk->ssat[i-1]->outc[0]>GAP_RESION&&rtk->ssat[i-1]->outc[1]>GAP_RESION)
            rtk->x[j]=0.0;
    }
    for (i=1;i<ns;i++) {
        j=II(sat[i],&rtk->opt);
        if (rtk->ssat[i - 1] == NULL)
        {
            continue;
        }
        if (rtk->x[j]==0.0) {
            initx(rtk,1E-6,SQR(rtk->opt.std[1]*bl/1E4),j);
        }
        else {
            /* elevation dependent factor of process noise */
					if (NULL != rtk->ssat[sat[i] - 1])
					{
						el = rtk->ssat[sat[i] - 1]->azel[1];
						fact = cos(el);
						rtk->P[j + j * rtk->nx] += SQR(rtk->opt.prn[1] * bl / 1E4 * fact) * tt;
					}
        }
    }
}
/* temporal update of tropospheric parameters --------------------------------*/
static void udtrop(rtk_t *rtk, double tt, double bl)
{
    int i,j,k;
    
    gnss_util_trace(3,"udtrop  : tt=%.1f\n",tt);
    
    for (i=0;i<2;i++) {
        j=IT(i,&rtk->opt);
        
        if (rtk->x[j]==0.0) {
            initx(rtk,INIT_ZWD,SQR(rtk->opt.std[2]),j); /* initial zwd */
            
            if (rtk->opt.tropopt>=TROPOPT_ESTG) {
                for (k=0;k<2;k++) initx(rtk,1E-6,VAR_GRA,++j);
            }
        }
        else {
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[2])*tt;
            
            if (rtk->opt.tropopt>=TROPOPT_ESTG) {
                for (k=0;k<2;k++) {
                    rtk->P[++j*(1+rtk->nx)]+=SQR(rtk->opt.prn[2]*0.3)*fabs(rtk->tt);
                }
            }
        }
    }
}
/* temporal update of receiver dcb parameters --------------------------------*/
static void time_update_rcv_dcb(rtk_t* rtk, double tt)
{
	double dcb_noise = 10.0;
	double dcb_sigma = VAR_RCV_CLK * (20.0 * 20.0);
	int i = 0, j = 0, nf = NF(&rtk->opt)/*, freq1_index = 0*/;
	int n_clock = nf * NSYSDD;
	for (i = NSYSDD; i < n_clock; i++)
	{
		j = rtk->ix[IRC(i, &rtk->opt)];
		if (j >= 0)
		{
			if (rtk->P[ISM(j,j)] < dcb_sigma)
			{
				rtk->P[ISM(j, j)] += SQR(dcb_noise) * tt;
			}
		}
	}
	return;
}
/* temporal update of receiver h/w biases ------------------------------------*/
static void udrcvbias(rtk_t *rtk, double tt)
{
    int i,j;
    
    gnss_util_trace(3,"udrcvbias: tt=%.1f\n",tt);
    
    for (i=0;i<NFREQGLO;i++) {
        j=IL(i,&rtk->opt);
        
        if (rtk->x[j]==0.0) {
            initx(rtk,1E-6,VAR_HWBIAS,j);
        }
        /* hold to fixed solution */
        else if (rtk->nfix>=rtk->opt.minfix&&rtk->sol.ratio>rtk->opt.thresar[0]) {
            initx(rtk,rtk->xa[j],rtk->Pa[j+j*rtk->na],j);
        }
        else {
            rtk->P[j+j*rtk->nx]+=SQR(PRN_HWBIAS)*tt;
        }
    }
}
/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv)
{
    unsigned int slip,LLI;
    int f,sat=obs[i].sat;
    
    gnss_util_trace(5,"detslp_ll: i=%d rcv=%d\n",i,rcv);
	if (NULL == rtk->ssat[sat - 1])
	{
		return;
	}
    for (f=0;f<rtk->opt.nf;f++) {
        
        if (obs[i].L[f]==0.0) continue;
        
        /* restore previous LLI */
        if (rcv==1) LLI=asg_getbitu(&rtk->ssat[sat-1]->slip[f],0,2); /* rover */
        else        LLI=asg_getbitu(&rtk->ssat[sat-1]->slip[f],2,2); /* base  */
        
        /* detect slip by cycle slip flag in LLI */
        if (rtk->tt>=0.0) { /* forward */
            if (obs[i].LLI[f]&1) {
				gnss_util_trace(2, "slip detected forward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                       sat,rcv,f+1,obs[i].LLI[f]);
            }
            slip=obs[i].LLI[f];
        }
        else { /* backward */
            if (LLI&1) {
				gnss_util_trace(2, "slip detected backward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                       sat,rcv,f+1,LLI);
            }
            slip=LLI;
        }
        /* detect slip by parity unknown flag transition in LLI */
		if (!(LLI & 2) && (obs[i].LLI[f] & 2)) {
			gnss_util_trace(2, "slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n",
				sat, rcv, f + 1, LLI, obs[i].LLI[f]);
			slip |= 1;
		}
		if ((LLI & 2) && !(obs[i].LLI[f] & 2)) {
			gnss_util_trace(2, "slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n",
				sat, rcv, f + 1, LLI, obs[i].LLI[f]);
			if (!(rcv == 1 && obs[i].SNR[f] >= 160 && (rtk->ssat[sat - 1]->slipflag[rcv - 1][f] & CYCSLIPDET_TIMEGAP) == 0))
			{
				slip |= 1;
			}
		}
        /* save current LLI */
        if (rcv==1) setbitu(&rtk->ssat[sat-1]->slip[f],0,2,obs[i].LLI[f]);
        else        setbitu(&rtk->ssat[sat-1]->slip[f],2,2,obs[i].LLI[f]);
        
        /* save slip and half-cycle valid flag */
        rtk->ssat[sat-1]->slip[f]|=(unsigned char)slip;
        rtk->ssat[sat-1]->half[f]=(obs[i].LLI[f]&2)?0:1;
    }
}
#if 0
/* detect cycle slip by L1-L2 geometry free phase jump -----------------------*/
static void detslp_gf_L1L2(rtk_t *rtk, const obsd_t *obs, int i, int j,
                           const nav_t *nav)
{
    int sat=obs[i].sat,slip=0;
    double g0,g1;
	if (NULL == rtk->ssat[sat - 1])
	{
		return;
	}
    if (rtk->opt.nf<=1||(g1=gfobs_L1L2(obs,i,j,nav->lam[sat-1]))==0.0) {
    	return;
    }
    
    g0=rtk->ssat[sat-1]->gf; 
    rtk->ssat[sat-1]->gf=g1;

    if (rtk->rtkevent&RTK_BASECHANGE)  return;
        
    if (g0!=0.0&&fabs(g1-g0)>rtk->opt.thresslip) {
        slip=1;
        rtk->ssat[sat-1]->slip[0]|=1;
        rtk->ssat[sat-1]->slip[1]|=1;

    }
	GLOGI("slp_gf12: sat=%3d i=%2d j=%2d gf=%8.3f %d",sat,i,j,g1-g0,slip);
}
/* detect cycle slip by L1-L5 geometry free phase jump -----------------------*/
static void detslp_gf_L1L5(rtk_t *rtk, const obsd_t *obs, int i, int j,
                           const nav_t *nav)
{
#if 0
    int sat=obs[i].sat,slip=0;
    double g0,g1;

    if (rtk->opt.nf<=2||(g1=gfobs_L1L5(obs,i,j,nav->lam[sat-1]))==0.0) return;
    
    g0=rtk->ssat[sat-1]->gf2; rtk->ssat[sat-1]->gf2=g1;

    if (rtk->rtkevent&RTK_BASECHANGE)  return;
        
    if (g0!=0.0&&fabs(g1-g0)>rtk->opt.thresslip) {
        slip=1;
        rtk->ssat[sat-1]->slip[0]|=1;
        rtk->ssat[sat-1]->slip[2]|=1;

    }
	GLOGI("slp_gf15: sat=%3d i=%2d j=%2d gf=%8.3f %d",sat,i,j,g1-g0,slip);
#endif
}
#endif
/* detect cycle slip by doppler and phase difference -------------------------*/
#ifdef USED_IN_MC262M
static void detslp_dop(rtk_t *rtk, const obsd_t *obs, int i, int rcv,
                       const nav_t *nav)
{
    /* detection with doppler disabled because of clock-jump issue (v.2.3.0) */
#if 1
    int f,sat=obs[i].sat;
    double tt,dph,dpt,lam,thres;
    
    gnss_util_trace(3,"detslp_dop: i=%d rcv=%d\n",i,rcv);
    
    for (f=0;f<rtk->opt.nf;f++) {
        if (obs[i].L[f]==0.0||obs[i].D[f]==0.0||rtk->ssat[sat-1]->ph[rcv-1][f]==0.0) {
            continue;
        }
        if (fabs(tt=timediff(obs[i].time,rtk->ssat[sat-1]->pt[rcv-1][f]))<DTTOL) continue;
        if ((lam=nav->lam[sat-1][f])<=0.0) continue;
        
        /* cycle slip threshold (cycle) */
        thres=MAXACC*tt*tt/2.0/lam+rtk->opt.err[4]*fabs(tt)*4.0;
        
        /* phase difference and doppler x time (cycle) */
        dph=obs[i].L[f]-rtk->ssat[sat-1]->ph[rcv-1][f];
        dpt=-obs[i].D[f]*tt;
        
        if (fabs(dph-dpt)<=thres) continue;
        
        rtk->ssat[sat-1]->slip[f]|=1;
        
		gnss_util_trace(2,"slip detected (sat=%2d rcv=%d L%d=%.3f %.3f thres=%.3f)\n",
               sat,rcv,f+1,dph,dpt,thres);
    }
#endif
}
#endif
/* temporal update of phase biases -------------------------------------------*/
static void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
                   const int *iu, const int *ir, int ns, const nav_t *nav)
{
	double cp = 0.0, pr = 0.0, cp1 = 0.0, cp2 = 0.0, pr1 = 0.0, pr2 = 0.0, * bias = NULL, offset = 0.0, lami = 0.0, lam1 = 0.0, lam2 = 0.0, C1 = 0.0, C2 = 0.0;
	int i = 0, j = 0, f = 0, slip = 0, reset = 0, nf = NF(&rtk->opt);
    
    gnss_util_trace(3,"udbias  : tt=%.1f ns=%d\n",tt,ns);
    
    for (i=0;i<ns;i++) {
        
        /* detect cycle slip by LLI */
		if (NULL == rtk->ssat[sat[i] - 1])
		{
			continue;
		}
        for (f=0;f<rtk->opt.nf;f++) rtk->ssat[sat[i]-1]->slip[f]&=0xFC;

		for (f=0;f<rtk->opt.nf;f++)
		{
			/* cycle slip determine */
			slip=gnss_rtk_slip_determine(f,rtk->ssat[sat[i]-1],1);
			if ((rtk->rtkevent&RTK_BASECHANGE)==0 && (rtk->ssat[sat[i] - 1]->slipflag[1][f] & CYCSLIPDET_DDSAT))
			{
				slip |= 1;
			}
			rtk->ssat[sat[i]-1]->slip[f]|=slip;
		}

        detslp_ll(rtk,obs,iu[i],1);
        detslp_ll(rtk,obs,ir[i],2);
        
        /* detect cycle slip by geometry-free phase jump */
        //detslp_gf_L1L2(rtk,obs,iu[i],ir[i],nav);  //actually L1L5
        //detslp_gf_L1L5(rtk,obs,iu[i],ir[i],nav);

		gnss_rtk_slip_lc_fusion(sat[i],rtk->ssat[sat[i]-1],1);
		if ((rtk->rtkevent&RTK_BASECHANGE) == 0)
		{
			gnss_rtk_slip_lc_fusion(sat[i], rtk->ssat[sat[i] - 1], 2);
		}

        /* update half-cycle valid flag */
        for (f=0;f<nf;f++) {
            rtk->ssat[sat[i]-1]->half[f]=
                !((obs[iu[i]].LLI[f]&2)||(obs[ir[i]].LLI[f]&2));
        }
    }
    for (f=0;f<nf;f++) {
        /* reset phase-bias if instantaneous AR or expire obs outage counter */
        for (i=1;i<=MAXSAT;i++) {
			if (rtk->ssat[i - 1] == NULL)
			{
				continue;
			}
            reset=++rtk->ssat[i-1]->outc[f]>(unsigned int)rtk->opt.maxout;

			if (rtk->ssat[i - 1]->present[0])
			{
				slip = gnss_rtk_slip_determine(f, rtk->ssat[i - 1], 1);
				if (slip == 1)
				{
					reset = 1;
				}
			}
			//if (fabs(rtk->ptor-5981.954)<1e-3) reset=1;
			if (rtk->kfstatus&RTK_KF_RESET_AMB)
			{
				reset=1;
			}

            if (rtk->opt.modear==ARMODE_INST && rtk->ix[IB(i, f, &rtk->opt)] > -1) {
                initx(rtk,0.0,0.0,IB(i,f,&rtk->opt));
            }
			else if (reset&&rtk->ix[IB(i, f, &rtk->opt)] > -1) {
                initx(rtk,0.0,0.0,IB(i,f,&rtk->opt));
                gnss_util_trace(3,"udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n",
                      i,f+1,rtk->ssat[i-1]->outc[f]);
            }
            if (rtk->opt.modear!=ARMODE_INST&&reset) {
                rtk->ssat[i-1]->lock[f]=-rtk->opt.minlock;
            }
        }
        /* reset phase-bias if detecting cycle slip */
        for (i=0;i<ns;i++) {
			if (NULL == rtk->ssat[sat[i] - 1])
			{
				continue;
			}
            j=IB(sat[i],f,&rtk->opt);
            //rtk->P[j+j*rtk->nx]+=rtk->opt.prn[0]*rtk->opt.prn[0]*tt; //has predicted
            slip=rtk->ssat[sat[i]-1]->slip[f];
            if (rtk->opt.ionoopt==IONOOPT_IFLC) slip|=rtk->ssat[sat[i]-1]->slip[1];
            if (rtk->opt.modear==ARMODE_INST||!(slip&1)) continue;
			initx(rtk, 0.0, 0.0, j);
            rtk->ssat[sat[i]-1]->lock[f]=-rtk->opt.minlock;
        }
        bias=zeros(ns,1);
        
        /* estimate approximate phase-bias by phase - code */
        for (i=j=0,offset=0.0;i<ns;i++) {
			if (NULL == rtk->ssat[obs[iu[i]].sat - 1])
			{
				continue;
			}
            if (rtk->opt.ionoopt!=IONOOPT_IFLC) {
				if (rtk->opt.adaptSeqEKF && (rtk->ssat[obs[iu[i]].sat - 1]->usectrl.rqflag[f] & PE_MEAS_VALID_PR) == 0)
				{
					continue;
				}
                cp=sdobs(obs,iu[i],ir[i],f); /* cycle */
                pr=sdobs(obs,iu[i],ir[i],f+NFREQ);
                lami=nav->lam[sat[i]-1][f];
                if (cp==0.0||pr==0.0||lami<=0.0) continue;
                
                bias[i]=cp-pr/lami;
            }
            else {
				if (rtk->opt.adaptSeqEKF && ((rtk->ssat[obs[iu[i]].sat - 1]->usectrl.rqflag[0] & PE_MEAS_VALID_PR) == 0
					|| (rtk->ssat[obs[iu[i]].sat - 1]->usectrl.rqflag[1] & PE_MEAS_VALID_PR) == 0))
				{
					continue;
				}
                cp1=sdobs(obs,iu[i],ir[i],0);
                cp2=sdobs(obs,iu[i],ir[i],1);
                pr1=sdobs(obs,iu[i],ir[i],NFREQ);
                pr2=sdobs(obs,iu[i],ir[i],NFREQ+1);
                lam1=nav->lam[sat[i]-1][0];
                lam2=nav->lam[sat[i]-1][1];
                if (cp1==0.0||cp2==0.0||pr1==0.0||pr2==0.0||lam1<=0.0||lam2<=0.0) continue;
                
                C1= SQR(lam2)/(SQR(lam2)-SQR(lam1));
                C2=-SQR(lam1)/(SQR(lam2)-SQR(lam1));
                bias[i]=(C1*lam1*cp1+C2*lam2*cp2)-(C1*pr1+C2*pr2);
            }
			if (rtk->ix[IB(sat[i], f, &rtk->opt)] > -1) {
				if (rtk->opt.ionoopt != IONOOPT_IFLC)
				{
					offset += (bias[i] - rtk->x[rtk->ix[IB(sat[i], f, &rtk->opt)]]) * lami;
				}
				else
				{
					offset += bias[i] - rtk->x[rtk->ix[IB(sat[i], f, &rtk->opt)]];
				}
				j++;
			}
        }
        /* correct phase-bias offset to enssure phase-code coherency */
        if (j>0) {
            for (i=1;i<=MAXSAT;i++) {
				if (rtk->ix[IB(i, f, &rtk->opt)] > -1) {
					if (rtk->opt.ionoopt != IONOOPT_IFLC)
					{
						lami = nav->lam[i - 1][f];
						if (lami <= 0.0) continue;
						rtk->x[rtk->ix[IB(i, f, &rtk->opt)]] += offset / lami / j;
					}
					else
					{
						rtk->x[rtk->ix[IB(i, f, &rtk->opt)]] += offset / j;
					}
				}
            }
            GLOGI("L%d phase-bias offset=%.3f",f+1,offset/j);
        }
        /* set initial states of phase-bias */
        for (i=0;i<ns;i++) {
			if (NULL == rtk->ssat[sat[i] - 1])
			{
				continue;
			}
			if (bias[i] == 0.0 || rtk->ix[IB(sat[i], f, &rtk->opt)] > -1) continue;
			if (rtk->kfcnt < 10 && rtk->ssat[sat[i] - 1]->usectrl.pr_qoschck[f])
			{
				initx(rtk, bias[i], SQR(rtk->opt.std[0] * 5), IB(sat[i], f, &rtk->opt));
			}
			else
			{
				initx(rtk, bias[i], SQR(rtk->opt.std[0]), IB(sat[i], f, &rtk->opt));
			}
        }
        Sys_Free(bias);
    }
	if (rtk->kfstatus&RTK_KF_RESET_AMB)
	{
		rtk->kfstatus&=~RTK_KF_RESET_AMB;
	}
}
/* temporal update of states --------------------------------------------------*/
static void ag_rtk_udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
                    const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double tt=fabs(rtk->tt),bl,dr[3];
    
    gnss_util_trace(3,"ag_rtk_udstate : ns=%d\n",ns);
    
    /* temporal update of position/velocity/acceleration */
    //udpos(rtk,tt);
    
    /* temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt>=IONOOPT_EST) {
        bl=baseline(rtk->x,rtk->rb,dr);
        udion(rtk,tt,bl,sat,ns);
    }
    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt>=TROPOPT_EST) {
		bl = baseline(rtk->x,rtk->rb,dr);
        udtrop(rtk,tt,bl);
    }
    /* temporal update of eceiver h/w bias */
    if (rtk->opt.glomodear==2&&(rtk->opt.navsys&SYS_GLO)) {
        udrcvbias(rtk,tt);
    }
    /* temporal update of phase-bias */
    if (rtk->opt.mode>PMODE_DGPS) {
        udbias(rtk,tt,obs,sat,iu,ir,ns,nav);
    }
	if (rtk->opt.adaptSeqEKF)
	{
		time_update_rcv_dcb(rtk, tt);
	}
}
/* undifferenced phase/code residual for satellite ---------------------------*/
static void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
                      const double *azel, const double *dant,
                      const prcopt_t *opt, double *y)
{
    const double *lam=nav->lam[obs->sat-1];
    double f1,f2,C1,C2,dant_if;
    int i,nf=NF(opt);
    
    if (opt->ionoopt==IONOOPT_IFLC) { /* iono-free linear combination */
        if (lam[0]==0.0||lam[1]==0.0) return;
        
        if (testsnr(base,0,azel[1],obs->SNR[0]*0.25,&opt->snrmask)||
            testsnr(base,1,azel[1],obs->SNR[1]*0.25,&opt->snrmask)) return;
        
        f1=CLIGHT/lam[0];
        f2=CLIGHT/lam[1];
        C1= SQR(f1)/(SQR(f1)-SQR(f2));
        C2=-SQR(f2)/(SQR(f1)-SQR(f2));
        dant_if=C1*dant[0]+C2*dant[1];
        
        if (obs->L[0]!=0.0&&obs->L[1]!=0.0) {
            y[0]=C1*obs->L[0]*lam[0]+C2*obs->L[1]*lam[1]-r-dant_if;
        }
        if (obs->P[0]!=0.0&&obs->P[1]!=0.0) {
            y[1]=C1*obs->P[0]+C2*obs->P[1]-r-dant_if;
        }
    }
    else {
        for (i=0;i<nf;i++) {
            if (lam[i]==0.0) continue;
            
            /* check snr mask */
            if (testsnr(base,i,azel[1],obs->SNR[i]*0.25,&opt->snrmask)) {
                continue;
            }
            /* residuals = observable - pseudorange */
            if (obs->L[i]!=0.0) y[i   ]=obs->L[i]*lam[i]-r-dant[i];
            if (obs->P[i]!=0.0) y[i+nf]=obs->P[i]       -r-dant[i];
        }
    }
}
/* undifferenced phase/code residuals ----------------------------------------*/
static int ag_gnss_zdres(int base, const obsd_t *obs, int n, const double *rs,
                 const double *dts, const int *svh, const nav_t *nav,
                 const double *rr, const prcopt_t *opt, int index, double *y,
                 double *e, double *azel)
{
    double r,rr_[3] = { 0 },pos[3] = { 0 },dant[NFREQ]={0},/*disp[3],*/ dt1 = 0.0;
    double zhd=0.0,zazel[]={0.0,90.0*D2R};
    int i,nf=NF(opt), k, f, flag = FALSE;
    
    gnss_util_trace(3,"ag_gnss_zdres   : n=%d\n",n);
    
    for (i=0;i<n*nf*2;i++) y[i]=0.0;
    
    if (asg_norm(rr,3)<=0.0) return 0; /* no receiver position */
    
#if 0
    /* earth tide correction */
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rr_,opt->tidecorr,&nav->erp,
                 opt->odisp[base],disp);
        for (i=0;i<3;i++) rr_[i]+=disp[i];
    }
#endif

	if (opt->tropopt != TROPOPT_OFF)
	{
		for (i = 0; i < 3; i++) rr_[i] = rr[i];
		ecef2pos(rr_, pos);
	}

	if (base == 1 && g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		dt1 = timediff(rtcm3data.obs[0].data[0].time, rtkctrl.pt[1]);
		if (fabs(dt1) < 0.001 && rtcm3data.obs[0].n == rtcm3data.obs[1].n)
		{
			flag = TRUE;
		}
	}
    
    for (i=0;i<n;i++) {
		if (NULL == rtkctrl.ssat[obs[i].sat - 1])
		{
			continue;
		}
		if (flag == TRUE /*&& rtkctrl.ssat[obs[i].sat - 1]->slp_ref.toe[0] == rtkctrl.ssat[obs[i].sat - 1]->slp_ref.toe[1]*/)
		{
			for (f = 0; f < nf && f < NFREQ; f++)
			{
				y[f + i * nf * 2] = rtkctrl.ssat[obs[i].sat - 1]->td[1].y[f];
				y[f + nf + i * nf * 2] = rtkctrl.ssat[obs[i].sat - 1]->td[1].y[f + nf];
			}
			for (k = 0; k < 3; k++)
			{
				e[k + i * 3] = rtkctrl.ssat[obs[i].sat - 1]->e[k];
			}
			continue;
		}

        /* compute geometric-range and azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr,e+i*3))<=0.0) continue;
        //if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) continue;
		if (azel[1 + i * 2] < opt->elmin) continue;
        
        /* excluded satellite? */
        if (satexclude(obs[i].sat,svh[i],opt)) continue;
        
        /* satellite clock-bias */
        r+=-CLIGHT*dts[i*2];
        
        /* troposphere delay model (hydrostatic) */
		if (opt->tropopt!=TROPOPT_OFF)
		{
			zhd=tropmodel(obs[0].time,pos,zazel,0.0);
			r+=tropmapf(obs[i].time,pos,azel+i*2,NULL)*zhd;
		}
        
#if 0
        /* receiver antenna phase center correction */
        antmodel(opt->pcvr+index,opt->antdel[index],azel+i*2,opt->posopt[1],dant);
#endif
        
        /* undifferenced phase/code residual for satellite */
        zdres_sat(base,r,obs+i,nav,azel+i*2,dant,opt,y+i*nf*2);
    }
    gnss_util_trace(3,"rr_%c=%.3f %.3f %.3f\n",base?'r':'u',rr_[0],rr_[1],rr_[2]);
    gnss_util_trace(4,"pos=%.9f %.9f %.3f\n",pos[0]*R2D,pos[1]*R2D,pos[2]);
    for (i=0;i<n;i++) {
        gnss_util_trace(4,"sat=%2d %13.3f %13.3f %13.3f %13.10f %6.1f %5.1f\n",
              obs[i].sat,rs[i*6],rs[1+i*6],rs[2+i*6],dts[i*2],azel[i*2]*R2D,
              azel[1+i*2]*R2D);
    }
    gnss_util_trace(4,"y=\n"); gnss_util_tracemat(4,y,nf*2,n,13,3);
    
    return 1;
}
/* test valid observation data -----------------------------------------------*/
static int validobs(int i, int j, int f, int nf, const double *y, int rtdflag)
{
    /* if no phase observable, pseudorange is also unusable */
    return y[f+i*nf*2]!=0.0&&y[f+j*nf*2]!=0.0&&
           (f<nf||(rtdflag)||(y[f-nf+i*nf*2]!=0.0&&y[f-nf+j*nf*2]!=0.0));
}
/* double-differenced measurement error covariance ---------------------------*/
static void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
                  int nv, double *R)
{
    int i,j,k=0,b;
    
    gnss_util_trace(3,"ddcov   : n=%d\n",n);
    
    for (i=0;i<nv*nv;i++) R[i]=0.0;
    for (b=0;b<n;k+=nb[b++]) {
        
        for (i=0;i<nb[b];i++) for (j=0;j<nb[b];j++) {
            R[k+i+(k+j)*nv]=Ri[k+i]+(i==j?Rj[k+i]:0.0);
        }
    }
    gnss_util_trace(4,"R=\n"); gnss_util_tracemat(4,R,nv,nv,8,6);
}
#if 0
/* baseline length constraint ------------------------------------------------*/
static int constbl(rtk_t *rtk, const double *x, const double *P, double *v,
                   double *H, double *Ri, double *Rj, int index)
{
    const double thres=0.1; /* threshold for nonliearity (v.2.3.0) */
    double xb[3],b[3],bb,var=0.0;
    int i;
     
    gnss_util_trace(3,"constbl : \n");
    
    /* no constraint */
    if (rtk->opt.baseline[0]<=0.0) return 0;
    
    /* time-adjusted baseline vector and length */
    for (i=0;i<3;i++) {
        xb[i]=rtk->rb[i]+rtk->rb[i+3]*rtk->sol.age;
        b[i]=x[i]-xb[i];
    }
    bb=asg_norm(b,3);
    
    /* approximate variance of solution */
    if (P) {
        for (i=0;i<3;i++) var+=P[i+i*rtk->nx];
        var/=3.0;
    }
    /* check nonlinearity */
    if (var>thres*thres*bb*bb) {
        gnss_util_trace(3,"constbl : equation nonlinear (bb=%.3f var=%.3f)\n",bb,var);
        return 0;
    }
    /* constraint to baseline length */
    v[index]=rtk->opt.baseline[0]-bb;
    if (H) {
        for (i=0;i<3;i++) H[i+index*rtk->nx]=b[i]/bb;
    }
	if (Ri&&Rj)
	{
    Ri[index]=0.0;
    Rj[index]=SQR(rtk->opt.baseline[1]);
    gnss_util_trace(4,"baseline len   v=%13.3f R=%8.6f %8.6f\n",v[index],Ri[index],Rj[index]);
	}
    
    return 1;
}
#endif
/* precise tropspheric model -------------------------------------------------*/
static double prectrop(gtime_t time, const double *pos, int r,
                       const double *azel, const prcopt_t *opt, const double *x,
                       double *dtdx)
{
    double m_w=0.0,cotz,grad_n,grad_e;
    int i=IT(r,opt);
    
    /* wet mapping function */
    tropmapf(time,pos,azel,&m_w);
    
    if (opt->tropopt>=TROPOPT_ESTG&&azel[1]>0.0) {
        
        /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
        cotz=1.0/tan(azel[1]);
        grad_n=m_w*cotz*cos(azel[0]);
        grad_e=m_w*cotz*sin(azel[0]);
        m_w+=grad_n*x[i+1]+grad_e*x[i+2];
        dtdx[1]=grad_n*x[i];
        dtdx[2]=grad_e*x[i];
    }
    else dtdx[1]=dtdx[2]=0.0;
    dtdx[0]=m_w;
    return m_w*x[i];
}
/* get local glonass cpb info, unit: m -------------------------------------------------------------
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
static double gnss_rtk_get_glocpb_local(int chiptype, int sub_chiptype, int f)
{
	if (chiptype == UBLOX)
	{
		switch (sub_chiptype)
		{
		case SUB_CHIPTYPE_UBLOX_M8:   return LOCAL_GLO_CPB_UBLOX_M8;
		case SUB_CHIPTYPE_UBLOX_F9:   return LOCAL_GLO_CPB_UBLOX_F9;
		case SUB_CHIPTYPE_ST_8090:    return LOCAL_GLO_CPB_ST_8090;
		case SUB_CHIPTYPE_ST_8100:    return LOCAL_GLO_CPB_ST_8100;
		default: return 0.0;
		}
	}
	else if (chiptype == QCOM)
	{
		switch (sub_chiptype)
		{
		case SUB_CHIPTYPE_QCOM_SF:
		case SUB_CHIPTYPE_QCOM_DF:    return LOCAL_GLO_CPB_QCOM;
		case SUB_CHIPTYPE_BRCM_47755: return LOCAL_GLO_CPB_BRCM_47755;
		default: return 0.0;
		}
	}

	return 0.0;
}
/* get ref glonass cpb info, unit: m -------------------------------------------------------------
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
static int gnss_rtk_get_glocpb_ref(int f, double *cpb)
{
	if (!rtcm3data.glocpb_available || rtcm3data.obs[0].n < 1)
	{
		*cpb = 0.0;
		glo_ar_freq_mask &= ~(1<<f);
		return 0;
	}

	switch (rtcm3data.obs[0].data[0].code[f])
	{
	case CODE_L1C:
		if (rtcm3data.pe_glo_cpb.sigmask & 0x1)
		{
			*cpb = rtcm3data.pe_glo_cpb.cpb[0];
			glo_ar_freq_mask |= 0x1;
			return 1;
		}
		break;
	case CODE_L1P:
		if (rtcm3data.pe_glo_cpb.sigmask & 0x2)
		{
			*cpb = rtcm3data.pe_glo_cpb.cpb[1];
			glo_ar_freq_mask |= 0x1;
			return 1;
		}
		break;
	case CODE_L2C:
		if (rtcm3data.pe_glo_cpb.sigmask & 0x4)
		{
			*cpb = rtcm3data.pe_glo_cpb.cpb[2];
			glo_ar_freq_mask |= 0x2;
			return 1;
		}
		break;
	case CODE_L2P:
		if (rtcm3data.pe_glo_cpb.sigmask & 0x8)
		{
			*cpb = rtcm3data.pe_glo_cpb.cpb[3];
			glo_ar_freq_mask |= 0x2;
			return 1;
		}
		break;
	default: break;
	}
	
	*cpb = 0.0;
	glo_ar_freq_mask &= ~(1<<f);
	return 0;
}
/* glonass inter-channel bias correction -------------------------------------*/
static double gloicbcorr(int sat1, int sat2, const prcopt_t *opt, double lam1,
                         double lam2, int f)
{
    double dfreq, cpb_r, cpb_u;

	if (f>=NFREQGLO||f>=opt->nf) return 0.0; //for phase only

	if ((rtcm3data.info_mask & PE_RTCM_INFO_GLNCPB) && rtcm3data.glocpb_available == 1)
	{
		if (!gnss_rtk_get_glocpb_ref(f, &cpb_r))
		{
			return 0.0;
		}
		cpb_u = gnss_rtk_get_glocpb_local(g_pe_cfg.chipType, g_pe_cfg.sub_chipType, f);
		dfreq=(CLIGHT/lam1-CLIGHT/lam2)/(f==0?DFRQ1_GLO:DFRQ2_GLO);

		return -dfreq * (cpb_u - cpb_r) / 2848;
	}

	glo_ar_freq_mask |= 1 << f; //do gloar anyway if gloarmode!=0
    
    if (!opt->exterr.ena[2]) return 0.0;
    
    dfreq=(CLIGHT/lam1-CLIGHT/lam2)/(f==0?DFRQ1_GLO:DFRQ2_GLO);
    
    return opt->exterr.gloicb[f]*0.01*dfreq; /* (m) */
}
/* test navi system (m=0:gps/qzs/sbs,1:glo,2:gal,3:bds) ----------------------*/
static int test_sys(int sys, int m)
{
    switch (sys) {
        case SYS_GPS: return m==0;
        case SYS_QZS: return (NSYSDD == 4 ? m == 0 : m == 4);
        case SYS_SBS: return m==0;
        case SYS_GLO: return m==1;
        case SYS_GAL: return m==2;
        case SYS_CMP: return m==3;
    }
    return 0;
}

/* dym cp inno test for f9 ------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
static int gnss_rtk_inno_test(rtk_t* rtk, int m, int sat, int f, int nf, double v)
{
	if (rtk->ssat[sat - 1] == NULL)
	{
		return 0;
	}
	
	int lockc = rtk->ssat[sat - 1]->lock[f%nf] + rtk->opt.minlock;
	double maxinno = rtk->opt.maxinno;
#ifdef USED_IN_MC262M
	int lockms = rtk->ssat[sat - 1]->lockms[0][f%nf];
#endif
	int kfc = rtk->kfcnt;
	//int vs = rtk->ssat[sat - 1]->usectrl.rqflag[f%nf] & PE_MEAS_VALID_PR;
	int qcsc = sumofbit1(rtk->qcpasscnt, 5);
	int clsc = sumofbit1(rtk->closkycnt, 20);
	int rejf = 0;
	/*original inno test*/
	if (maxinno <= 0.0) return 1;
	//if (g_pe_cfg.sub_chipType != SUB_CHIPTYPE_UBLOX_F9 || rtk->ssat[sat - 1].pr_std[f%nf] < 1e-4) return v < maxinno;
#ifdef USED_IN_MC262M
	gnss_util_trace(4, "innot sat %d f %d vs %d lockvs %d lockvr %f lockms %d v %f  lockc %d rtd %d\n", sat, f, vs, rtk->lockvs[m], rtk->lockvr[m], lockms, v, lockc, rtk->rtduseflag);
	//if (f < nf && (rtk->lockvr[m] > 0.5 && rtk->lockvs[m] > 2) && (lockms <= UBX_LOCKMS_THRES) && !rtk->rtduseflag && (clsc > 2)&&(v > 5)) return 0;
	if (f < nf && (rtk->lockvr[m] > 0.5 && rtk->lockvs[m] > 2) && (lockms <= UBX_LOCKMS_THRES && lockms > 1e-3) && (clsc > 2) && (v > 5)) return 0;
#endif
	//if ((rtk->lockvr[m] > 0.5) && (lockms <= UBX_LOCKMS_THRES) && !rtk->rtduseflag && (qcsc < 18) && (v > 3)) return 0;
	//if ((f < nf && (clsc > 2 || vs == 0)) && !rtk->rtduseflag)
	//if ((f < nf && (clsc > 2 || vs == 0)))
	if ((f < nf && (qcsc < 3 || clsc > 2)) && !rtk->rtduseflag)
	{
		if (lockc <= 0)
		{
			 rejf = (rejf || v > 10);
		}
		if (lockc == 1)
		{
			rejf = (rejf || v > 6);
		}
		if (lockc == 2 || lockc == 3)
		{
			rejf = (rejf || v > 6);
		}
		if (lockc > 3)
		{
			rejf = (rejf || v > 6);
		}
		return !rejf;
	}
	else if ((f < nf && (qcsc < 3 || clsc > 2)) && rtk->rtduseflag && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_ST_8100)
	{
		if (lockc <= 0)
		{
			if (kfc == 0) rejf = (rejf || v > 2);
			else rejf = (rejf || v > 6);
		}
		if (lockc == 1)
		{
			rejf = (rejf || v > 4);
		}
		if (lockc == 2 || lockc == 3)
		{
			rejf = (rejf || v > 4);
		}
		if (lockc > 3)
		{
			rejf = (rejf || v > 4);
		}
		return !rejf;
	}
	else if ((f < nf && (qcsc < 3 || clsc > 2)) && rtk->rtduseflag && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
	{
		if (lockc <= 0)
		{
			if (kfc == 0) rejf = (rejf || v > 1.5);
			else rejf = (rejf || v > 4);
		}
		if (lockc == 1)
		{
			rejf = (rejf || v > 3);
		}
		if (lockc == 2 || lockc == 3)
		{
			rejf = (rejf || v > 2);
		}
		if (lockc > 3)
		{
			rejf = (rejf || v > 2);
		}
		//GLOGW("inno test: sat %3d, f %d, tor %.3f, lockc %d, v %.3f, rejf %d", sat, f + 1, rtk->ptor, lockc, v, rejf);
		return !rejf;
	}
	return v < maxinno;
}

/*select reference satellite for ddres*/
static int gnss_rtk_select_ref(int ddtype, const rtk_t* rtk, const double *x, const double *y, int iu, int ir, int f, int m, int sat)
{
	int sysi, nf, result;
	nf = NF(&rtk->opt);

	result = TRUE;
	if (NULL == rtk->ssat[sat - 1])
	{
		return FALSE;
	}
	sysi = rtk->ssat[sat - 1]->sys;

	do {
		if (rtk->ssat[sat - 1] == NULL) {
			result = FALSE;
			break;
		}
		if (!test_sys(sysi, m)) {
			result = FALSE;
			break;
		}
		if (f < nf && rtk->ix[IB(sat, f, &rtk->opt)] < 0) {
			result = FALSE;
			break;
		}
		if ((rtk->ssat[sat - 1]->freq_valid&FREQ_MASK(f%nf)) == 0) {
			result = FALSE;
			break;
		}
		if (!validobs(iu, ir, f, nf, y, rtk->rtduseflag)) {
			result = FALSE;
			break;
		}
		if ((peMode.staticData.staticFlag && gpz_kf_data->meas_blk->avgCno > 40 && (rtk->ssat[sat - 1]->usectrl.canberef & 0xF))
			|| (rtk->ssat[sat - 1]->usectrl.canberef & 0x01)) {
			result = FALSE;
			break;
		}
		if ((rtk->ssat[sat - 1]->usectrl.rqflag[f%nf] & PE_MEAS_VALID_PR) == 0) {
			result = FALSE;
			break;
		}
		//if (rtk->ssat[sat[j] - 1].usectrl.pr_qoschck[f%nf] & MAD_CHECK) continue;
		if (ddtype == DDRES_FIXED && rtk->ssat[sat - 1]->fix[f%nf] <= 1) {
			result = FALSE;
			break;
		}
#if defined(PLAYBACK_MODE)
		if (ddtype == DDRES_HPREF && f < nf && (rtk->ssat[sat - 1]->slip[f] & 0x3)) {
			result = FALSE;
			break;
		}
#endif
	} while (0);

	return result;
}

/* select reference satellete to ddres and limit by cn0---------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
static int gnss_rtk_get_ref(int ddtype, rtk_t *rtk, double *azel, const int *iu, const int *ir, int ns,
	const double *x, const int *sat, double *y, int m, int f)
{
	int i, j, mfsat, nf = NF(&rtk->opt);
	unsigned char mfcn0[MAXSAT] = { 0 };
	char cn036 = 0;
	double avecn0/*, rcn036*/;

	mfsat = 0;
	avecn0 = 0.0/*, rcn036 = 0.0*/;

	for (j = 0; j < ns; j++)
	{
		if (NULL == rtk->ssat[sat[j] - 1])
		{
			continue;
		}
		if (FALSE == gnss_rtk_select_ref(ddtype, rtk, x, y, iu[j], ir[j], f, m, sat[j])) continue;

		if (mfsat >= MAXSAT) continue;
		mfcn0[mfsat++] = rtk->ssat[sat[j] - 1]->snr[f%nf];
		if (rtk->ssat[sat[j] - 1]->snr[f%nf] / 4.0 >= 36.0) cn036++;
	}

	if (mfsat > 0)
	{
		for (j = 0; j < mfsat; j++)
		{
			avecn0 += (double)mfcn0[j];
		}
		avecn0 /= mfsat;
		//rcn036 = (double)cn036 / (double)mfsat;
	}
	
	for (i = -1, j = 0; j < ns; j++)
	{
		if (NULL == rtk->ssat[sat[j] - 1])
		{
			continue;
		}
		if (FALSE == gnss_rtk_select_ref(ddtype, rtk, x, y, iu[j], ir[j], f, m, sat[j])) continue;

		if (avecn0 > 150 && cn036 > 4 && g_pe_cfg.automobile != 0 && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_QCOM_855)
		{
			if (i < 0) i = j;
			else if (azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2] && rtk->ssat[sat[j] - 1]->snr[f%nf] >= avecn0)
				i = j;
			else if (azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2] && rtk->ssat[sat[j] - 1]->snr[f%nf] / 4.0 >= 40.0)
				i = j;
		}
		else {
			if (i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) i = j;
		}
	}

	return i;
}

/* double-differenced phase/code residuals -----------------------------------*/
static int ag_rtk_ddres(int ddtype,rtk_t *rtk, const nav_t *nav, double dt, const double *x,
                 const int *sat, double *y, double *e,
                 double *azel, const int *iu, const int *ir, int ns, double *v,
                 double *H, int *vflg)
{
    prcopt_t *opt=&rtk->opt;
    double bl=0.0,dr[3],posu[3],posr[3],didxi=0.0,didxj=0.0;
	double* im = NULL;
	double* vrej=NULL;
    double* vval=NULL;
    double* tropr = NULL; 
	double* tropu = NULL; 
    double* dtdxr = NULL; 
	double* dtdxu = NULL; 
    double* Ri=NULL;
    double* Rj=NULL;
	double lami,lamj,fi,fj,df;
	double* Hi=NULL;
    int i,j,k,m,f,ff,nv=0,nb[NFREQ*NSYSDD*2+2]={0},b=0,sysi,sysj,nf=NF(opt);
	int iscalr=0, refsat[NSYSDD]={ -1,-1,-1,-1 }, nrej[NSYSDD]={ 0,0,0,0 }, nval[NSYSDD]={ 0,0,0,0 }, irej=0, ival=0, refsatcnt=0, nrefcyc=0;
	double r_factor = 1.0;
    
    glo_ar_freq_mask = 0;
	rtk->cp_pr_cnt = 0;
    gnss_util_trace(3,"ag_rtk_ddres   : type=%d dt=%.1f nx=%d ns=%d\n",ddtype,dt,rtk->nx,ns);

	if (ddtype == DDRES_SINGLE) {vrej = mat(ns, NSYSDD);vval = mat(ns, NSYSDD);}

	iscalr=ddtype==DDRES_PREDICT||ddtype==DDRES_UPDATE||ddtype==DDRES_FIXED;
	if (iscalr)
	{
		bl=baseline(x,rtk->rb,dr);
		if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
		{
			Ri = mat(CMN_OBS_NUM_HF, 1); Rj = mat(CMN_OBS_NUM_HF, 1);
		}
		else
		{
			Ri = mat(CMN_OBS_NUM, 1); Rj = mat(CMN_OBS_NUM, 1);
		}
	}

	for (i = 0; i < NSYSDD; i++) refsat[i] = -1;

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
    	if (rtk->ssat[i] == NULL)
    	{
			continue;
    	}
        rtk->ssat[i]->resp[j]=rtk->ssat[i]->resc[j]=0.0;
    }
	if (opt->ionoopt>=IONOOPT_EST||opt->tropopt>=TROPOPT_EST)
	{
		ecef2pos(x,posu); ecef2pos(rtk->rb,posr);
		im=mat(ns,1);
		tropu=mat(ns,1); tropr=mat(ns,1); dtdxu=mat(ns,3); dtdxr=mat(ns,3);
		/* compute factors of ionospheric and tropospheric delay */
		for (i=0;i<ns;i++) {
			if (opt->ionoopt>=IONOOPT_EST) {
				im[i]=(ionmapf(posu,azel+iu[i]*2)+ionmapf(posr,azel+ir[i]*2))/2.0;
			}
			if (opt->tropopt>=TROPOPT_EST) {
				tropu[i]=prectrop(rtk->sol.time,posu,0,azel+iu[i]*2,opt,x,dtdxu+i*3);
				tropr[i]=prectrop(rtk->sol.time,posr,1,azel+ir[i]*2,opt,x,dtdxr+i*3);
			}
		}
	}
	//if (rtk->ptor > 202551.0&&rtk->ptor < 202580.0) rtk->ssat[13].usectrl.canberef = 1;
LAB_CAL_DDRES:
    for (m=0;m<NSYSDD;m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
    
    for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++) {
        //if (ddtype==DDRES_PREDICT && rtk->rtduseflag && f < nf) continue;
        /* search reference satellite with highest elevation */

		i = gnss_rtk_get_ref(ddtype, rtk, azel, iu, ir, ns, x, sat, y, m, f);

		if (i < 0)
		{
			if(ddtype == DDRES_SINGLE) gnss_util_trace(3, "ddres no refsat 0:gps,1:glo,2:gal,3:bds m=%d %c%d\n", m, f < nf ? 'L' : 'P', f%nf+1);
			continue;
		}
		if (NULL == rtk->ssat[sat[i] - 1])
		{
			continue;
		}

		//GLOGW("refsat: %3d, %c%d, %d, %.3f", sat[i], f < nf ? 'L' : 'P', f%nf + 1, ddtype, rtk->ptor);
		if (ddtype == DDRES_PREDICT)
		{
			GLOGD("ddres refsat=%d lock=%4d slip=%02x half=%d ele=%4.1f cn0=%d slipc=%u", sat[i], rtk->ssat[sat[i] - 1]->lock[f%nf], rtk->ssat[sat[i] - 1]->slip[f%nf] & 0x3,
				rtk->ssat[sat[i] - 1]->half[f%nf], rtk->ssat[sat[i] - 1]->azel[1] * R2D, rtk->ssat[sat[i] - 1]->snr[f%nf]/4,
				rtk->ssat[sat[i] - 1]->slipc[f%nf]);
		}
        
		if (0 == (f % nf) && f >= nf) 
		{
			nval[m] = 0;
			refsat[m] = sat[i];
			irej = 0;
			ival = 0;
		}
        /* make double difference */
        for (j=0;j<ns;j++) {
            if (i==j) continue;
            if (rtk->ssat[sat[i]-1] == NULL || rtk->ssat[sat[j]-1] == NULL) continue;
            sysi=rtk->ssat[sat[i]-1]->sys;
            sysj=rtk->ssat[sat[j]-1]->sys;
            if (!test_sys(sysj,m)) continue;
			if ((g_pe_cfg.meas_rate > MEAS_RATE_1HZ && nv >= CMN_OBS_NUM_HF) || (g_pe_cfg.meas_rate == MEAS_RATE_1HZ && nv >= CMN_OBS_NUM)) continue;
			if ((rtk->ssat[sat[j] - 1]->freq_valid&FREQ_MASK(f%nf)) == 0) continue;
			if (f < nf && rtk->ix[IB(sat[j], f, opt)] < 0) continue;		
			if (!validobs(iu[j], ir[j], f, nf, y, rtk->rtduseflag))
			{
				if (ddtype == DDRES_SINGLE) gnss_util_trace(3, "validobs fail sat=%d %c%d\n", sat[j], f < nf ? 'L' : 'P', f%nf + 1);
				continue;
			}
			
            ff=f%nf;
            lami=nav->lam[sat[i]-1][ff];
            lamj=nav->lam[sat[j]-1][ff];
            if (lami<=0.0||lamj<=0.0) continue;
            if (H) {
                Hi=H+nv*rtk->nx;
                for (k=0;k<rtk->nx;k++) Hi[k]=0.0;
            }
            /* double-differenced residual */
            v[nv]=(y[f+iu[i]*nf*2]-y[f+ir[i]*nf*2])-
                  (y[f+iu[j]*nf*2]-y[f+ir[j]*nf*2]);
            
            /* partial derivatives by rover position */
            if (H) {
                for (k=0;k<3;k++) {
                    Hi[k]=-e[k+iu[i]*3]+e[k+iu[j]*3];
                }
            }
            /* double-differenced ionospheric delay term */
            if (opt->ionoopt==IONOOPT_EST) {
                fi=lami/lam_carr[0]; fj=lamj/lam_carr[0];
                didxi=(f<nf?-1.0:1.0)*fi*fi*im[i];
                didxj=(f<nf?-1.0:1.0)*fj*fj*im[j];
                v[nv]-=didxi*x[II(sat[i],opt)]-didxj*x[II(sat[j],opt)];
                if (H) {
                    Hi[II(sat[i],opt)]= didxi;
                    Hi[II(sat[j],opt)]=-didxj;
                }
            }
            /* double-differenced tropospheric delay term */
            if (opt->tropopt==TROPOPT_EST||opt->tropopt==TROPOPT_ESTG) {
                v[nv]-=(tropu[i]-tropu[j])-(tropr[i]-tropr[j]);
                for (k=0;k<(opt->tropopt<TROPOPT_ESTG?1:3);k++) {
                    if (!H) continue;
                    Hi[IT(0,opt)+k]= (dtdxu[k+i*3]-dtdxu[k+j*3]);
                    Hi[IT(1,opt)+k]=-(dtdxr[k+i*3]-dtdxr[k+j*3]);
                }
            }
            /* double-differenced phase-bias term */
            if (f<nf) {

				gnss_rtk_amb_out(rtk, v[nv], f, sat[j], sat[i], lami, ddtype);
                if (opt->ionoopt!=IONOOPT_IFLC) {
                    if (ddtype == DDRES_FIXED && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100) {
                        double ddamb = m != 1 ? (v[nv] / lami) : ((v[nv] - (lami - lamj) * x[rtk->ix[IB(sat[j], f, opt)]]) / lami);
                        gnss_rtk_ddamb_resolve(rtk, m, f, sat[i], sat[j], ddamb);
                    }
					v[nv] -= lami * x[rtk->ix[IB(sat[i], f, opt)]] - lamj * x[rtk->ix[IB(sat[j], f, opt)]];
					if (H) {
						Hi[rtk->ix[IB(sat[i], f, opt)]] = lami;
						Hi[rtk->ix[IB(sat[j], f, opt)]] = -lamj;
					}
                }
                else {
                    v[nv]-=x[IB(sat[i],f,opt)]-x[IB(sat[j],f,opt)];
                    if (H) {
                        Hi[IB(sat[i],f,opt)]= 1.0;
                        Hi[IB(sat[j],f,opt)]=-1.0;
                    }
                }
            }
            /* glonass receiver h/w bias term */
            if (rtk->opt.glomodear==2&&sysi==SYS_GLO&&sysj==SYS_GLO&&ff<NFREQGLO) {
                df=(CLIGHT/lami-CLIGHT/lamj)/1E6; /* freq-difference (MHz) */
                v[nv]-=df*x[IL(ff,opt)];
                if (H) Hi[IL(ff,opt)]=df;
            }
            /* glonass interchannel bias correction */
            else if (sysi==SYS_GLO&&sysj==SYS_GLO) {
                
                v[nv]-=gloicbcorr(sat[i],sat[j],&rtk->opt,lami,lamj,f);
            }
			
            if (f<nf) {
            	rtk->ssat[sat[j]-1]->resc[f   ]=v[nv];
            }
            else {
            	rtk->ssat[sat[j]-1]->resp[f-nf]=v[nv];
            }
            
            /* test innovation */
            //if (opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno) {
			if (!gnss_rtk_inno_test(rtk, m, sat[j], f, nf, fabs(v[nv]))) {
                if (f<nf) {
                    rtk->ssat[sat[i]-1]->rejc[f]++;
                    rtk->ssat[sat[j]-1]->rejc[f]++;
                }
				if (g_pe_cfg.sub_chipType != SUB_CHIPTYPE_UBLOX_F9)
				{
					gnss_util_trace(2, "outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
						sat[i], sat[j], f < nf ? "L" : "P", f%nf + 1, v[nv]);
				}
				else
				{
					gnss_util_trace(2, "outlier rejected (sat=%3d-%3d %s%d v=%.3f lockc %d cls %d)\n",
						sat[i], sat[j], f < nf ? "L" : "P", 
						f%nf + 1, 
						v[nv], 
						rtk->ssat[sat[j] - 1]->lock[f%nf], 
						sumofbit1(rtk->closkycnt, 20));
				}
				if(f<nf) {
					rtk->dd_res[ddtype].rejc++;
				}
				else{
					rtk->dd_res[ddtype].rejp++;
					if(vrej) {
						if ((ff == 0)&&(rtk->ssat[sat[j] - 1]->usectrl.rqflag[ff] & PE_MEAS_VALID_PR))
						{
							vrej[irej * NSYSDD + m] = v[nv];
							nrej[m]++;
							irej++;
						}
					}
				}
                continue;
            }
			if ((rtk->ssat[sat[j]-1]->usectrl.rqflag[f%nf] & PE_MEAS_VALID_PR) == 0 && (f>=nf || fabs(v[nv]) > 1.0/*||
				g_pe_cfg.sub_chipType==SUB_CHIPTYPE_ST_8100*/))  //reject sat
			{
				gnss_util_trace(3,"reject by PE flag: sat=%3d-%3d %s%d v=%13.3f\n",sat[i],sat[j],f<nf?"L":"P",f%nf+1,v[nv]);
				if (ddtype==DDRES_PREDICT)
				{
#if 0
					GLOGW("reject: sat=%3d %s%d v=%6.1f %6.1f %5.2f q=%03x %03x rej=%d lce=%d %2u %4.1f",
						sat[j],f<nf?"L":"P",f%nf+1,v[nv],
						rtk->ssat[sat[j]-1]->usectrl.pr_diff,
						rtk->ssat[sat[j]-1]->usectrl.dr_diff,
						rtk->ssat[sat[j]-1]->usectrl.pr_qoschck[ff],
						rtk->ssat[sat[j]-1]->usectrl.dr_qoschck[ff],
						rtk->ssat[sat[j]-1]->usectrl.rqflag[f%nf]&0x3,
						rtk->ssat[sat[j]-1]->hod[ff].obs[0][0].llicnt,
						rtk->ssat[sat[j]-1]->hod[ff].obs[0][0].cn0,
						rtk->ssat[sat[j]-1]->azel[1]*R2D);
#else
                    GLOGI("reject: sat=%3d %s%d v=%6.1f %6.1f %5.2f q=%03x %03x rej=%d %4.1f",
                        sat[j], f < nf ? "L" : "P", f%nf + 1, v[nv],
                        rtk->ssat[sat[j] - 1]->usectrl.pr_diff,
                        rtk->ssat[sat[j] - 1]->usectrl.dr_diff,
                        rtk->ssat[sat[j] - 1]->usectrl.pr_qoschck[ff],
                        rtk->ssat[sat[j] - 1]->usectrl.dr_qoschck[ff],
                        rtk->ssat[sat[j] - 1]->usectrl.rqflag[f%nf] & 0x3,
                        //rtk->ssat[sat[j] - 1]->hod[ff].obs[0][0].llicnt,
                        //rtk->ssat[sat[j] - 1]->hod[ff].obs[0][0].cn0,
                        rtk->ssat[sat[j] - 1]->azel[1] * R2D);
#endif
				}
				continue;
			}
			if (iscalr)
			{
				if (ddtype == DDRES_PREDICT && (rtk->rtduseflag || (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310
                                                            && rtk->kfcnt < 10 && gpz_kf_data->posRes < 4.0 && gpz_kf_data->meas_blk->avgCno >= 38)) && f >= nf)
				{
					Ri[nv]= get_pe_meas_prNoise(sat[i],ff==0?ff:(ff+1)) / rtk->Qratio;
					Rj[nv]= get_pe_meas_prNoise(sat[j],ff==0?ff:(ff+1)) / rtk->Qratio;
					if(Ri[nv] < 0 || Rj[nv]< 0) continue;
				}
				else{
				/* single-differenced measurement error variances */
				Ri[nv]=varerr(sat[i],sysi,azel[1+iu[i]*2],bl,dt,f,opt);
				Rj[nv]=varerr(sat[j],sysj,azel[1+iu[j]*2],bl,dt,f,opt);
				}
				/* single-differenced measurement error variances */
				if(ddtype==DDRES_PREDICT) 
				{   
					r_factor = gnss_rtk_adjerror(rtk, sat[j], ns, fabs(v[nv]), f, Rj[nv]);
					Rj[nv] *= r_factor;
					if (f >= nf && r_factor >= 1.5)
					{
						rtk->dewNum_pr++;
					}
					else if (f < nf && r_factor >= 1.5)
					{
						rtk->dewNum_cr++;
					}
#ifdef USED_IN_MC262M
					if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9)
					{
						Rj[nv] *= gnss_rtk_ubx_weight(rtk, sat[j], f, nf, fabs(v[nv]));
					}
#endif
					/* set valid data flags */
					if (opt->mode>PMODE_DGPS) {
						if (f<nf) rtk->ssat[sat[i]-1]->vsat[f]=rtk->ssat[sat[j]-1]->vsat[f]=1;
					}
					else {
						rtk->ssat[sat[i]-1]->vsat[f-nf]=rtk->ssat[sat[j]-1]->vsat[f-nf]=1;
					}
				}
				gnss_util_trace(3,"sat=%3d-%3d %s%d v=%13.3f R=%8.6f %8.6f\n",sat[i],
					  sat[j],f<nf?"L":"P",f%nf+1,v[nv],Ri[nv],Rj[nv]);
			}
			else
			{
				gnss_util_trace(3,"sat=%3d-%3d %s%d v=%13.3f\n",sat[i],sat[j],f<nf?"L":"P",f%nf+1,v[nv]);
			}
            
			if (f >= nf && ddtype == DDRES_SINGLE) 
			{
				if (ff == 0)
				{
					vval[ival * NSYSDD + m] = v[nv];
					ival++;
					nval[m]++;
				}
			}
            vflg[nv++]=(sat[i]<<16)|(sat[j]<<8)|((f<nf?0:1)<<4)|(f%nf);
            nb[b]++;
        }

        b++;
    }

	rtk->cp_pr_cnt = nv;
    if (ddtype==DDRES_PREDICT && rtk->rtduseflag)
    {
		gnss_rtk_dr_sd_res(rtk,v,&nv,H,Hi,Ri,Rj,&nb[b],&b,ns,nf,NULL);
    }

    /* end of system loop */
	if (ddtype == DDRES_SINGLE && refsatcnt < 2)
	{
		nrefcyc = gnss_rtk_refsat_check(rtk, nrej, vrej, vval, ns, refsat, nval);
		if(nrefcyc)
		{
			for(i=0;i<NSYSDD;i++)
			{
				refsat[i]=-1;
				nrej[i]=0;
				nval[i]=0;
			}
			for (i=0;i<NFREQ*NSYSDD*2+2;i++) nb[i]=0;
			refsatcnt++;
			rtk->nrefs = refsatcnt;
			rtk->dd_res[ddtype].flag=0;
			rtk->dd_res[ddtype].rejc=rtk->dd_res[ddtype].rejp=0;
			for(i=0;i<nv;i++) vflg[i]=0;
			nv=0;b=0;irej=0;ival=0;
			for (i = 0;i < ns * NSYSDD;i++) {
				vrej[i] = 0;vval[i] = 0;
			}
			goto LAB_CAL_DDRES;
		}
	}
	if(vrej) Sys_Free(vrej);
	if(vval) Sys_Free(vval);
#if 0
    /* baseline length constraint for moving baseline */
    if (opt->mode==PMODE_MOVEB&&constbl(rtk,x,P,v,H,Ri,Rj,nv)) {
        vflg[nv++]=3<<4;
        nb[b++]++;
    }
	/* do quality control */
	if (ddtype==DDRES_PREDICT&&H&&Ri&&Rj) 
	{

	}

    if (H) {gnss_util_trace(4,"H=\n"); gnss_util_tracemat(4,H,rtk->nx,nv,7,4);}
#endif
	if (iscalr)
	{
		rtk->R = mat(nv, nv);
		/* double-differenced measurement error covariance */
		if(rtk->R) ddcov(nb,b,Ri,Rj,nv,rtk->R);
		Sys_Free(Ri); Sys_Free(Rj);
	}
    
	if (opt->ionoopt>=IONOOPT_EST||opt->tropopt>=TROPOPT_EST)
	{
		Sys_Free(im);
		Sys_Free(tropu); Sys_Free(tropr); Sys_Free(dtdxu); Sys_Free(dtdxr);
	}
    
    return nv;
}
static int asensing_rtk_sd_obs_meas_update(rtk_t* rtk, const nav_t* nav, double dt, const double* x,
	const int* sat, double* y, double* e,
	double* azel, const int* iu, const int* ir, int ns,int* max_obs_limit)
{
	prcopt_t* opt = &rtk->opt;
	double bl = 0.0, dr[3]/*, posu[3], posr[3], didxi = 0.0, didxj = 0.0*/;
	double lamj /* ,fi, fj, df*/;
	int i, j, k, m, f, ff, nv = 0, /*sysi,*/ sysj, nf = NF(opt), row_index = 0;
	//int nrej[NSYSDD] = { 0,0,0,0 }, refsatcnt = 0, nrefcyc = 0;
	double /*r_factor = 1.0, */rec_clk_dcb[NFREQ * NSYSDD] = { 0.0 };
	int sat_amb_index[MAXSAT * NFREQ] = { 0 }, max_amb_para = MAXSAT * NFREQ, index = -1, amb_para_num = 0;
	int pva_index[9] = { 0 }, rcv_clock_index[NSYSDD * NFREQ] = { 0 }, para_num = 0, * ix = NULL, amb_index = 0, sat_id = 0;
	double* P_ = NULL, * x_value = NULL, * delta_x = NULL, Rj = 0.0, v = 0.0, pseudo_res_update_thres = 50.0;
	int limit_obs_num = CMN_OBS_NUM;
	if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		limit_obs_num = CMN_OBS_NUM_HF;
	}
	limit_obs_num += 3 * nf * 2;
	if (max_obs_limit)
	{
		*max_obs_limit = limit_obs_num;
	}

	for (i = 0; i < 9; ++i)
	{
		pva_index[i] = -1;
	}
	for (i = 0; i < NSYSDD * NFREQ; ++i)
	{
		rcv_clock_index[i] = -1;
		index = rtk->ix[IRC(i, &rtk->opt)];
		if (index < 0 || rtk->P[ISM(index, index)]<=0.0)
		{
			continue;
		}
		rcv_clock_index[i] = index;
		rec_clk_dcb[i] = x[index];
		++para_num;
	}
	for (i = 0; i < 3; i++)
	{
		index = rtk->ix[i];
		if (index < 0)
		{
			continue;
		}
		pva_index[i] = index;
		++para_num;
	}
	if (rtk->opt.dynamics)
	{
		for (i = 3; i < 6; i++)
		{
			index = rtk->ix[i];
			if (index < 0)
			{
				continue;
			}
			pva_index[i] = index;
			++para_num;
		}
#ifndef RTK_PVMODE
		for (i = 6; i < 9; i++)
		{
			index = rtk->ix[i];
			if (index < 0)
			{
				continue;
			}
			pva_index[i] = index;
			++para_num;
		}
#endif
	}
	for (i = 0; i < max_amb_para; ++i)
	{
		sat_amb_index[i] = -1;
	}
	for (i = 0; i < NFREQ; ++i)
	{
		for (j = 0; j < MAXSAT; ++j)
		{
			index = rtk->ix[IB(j+1, i, opt)];
			if (index < 0)
			{
				continue;
			}
			sat_amb_index[i * MAXSAT + j] = index;
			++amb_para_num;
			++para_num;
		}
	}
	ix = imat(para_num, 1);
	index = 0;
	for (i = 0; i < 9; ++i)
	{
		if (pva_index[i] >= 0)
		{
			ix[index++] = pva_index[i];
		}
	}
	for (i = 0; i < NSYSDD * NFREQ; ++i)
	{
		if (rcv_clock_index[i] >= 0)
		{
			ix[index++] = rcv_clock_index[i];
		}
	}
	for (i = 0; i < NFREQ; ++i)
	{
		for (j = 0; j < MAXSAT; ++j)
		{
			if (sat_amb_index[i * MAXSAT + j] >= 0)
			{
				ix[index++] = sat_amb_index[i * MAXSAT + j];
			}
		}
	}
	P_ = mat(para_num, para_num);
	x_value= mat(para_num, 1);
	delta_x= mat(para_num, 1);
	for (i = 0; i < para_num; ++i)
	{
		delta_x[i] = 0.0;
		x_value[i] = x[ix[i]];
		for (j = 0; j < para_num; ++j)
		{
			P_[i*para_num + j] = rtk->P[ISM(ix[i], ix[j])];
		}
	}
	bl = baseline(x, rtk->rb, dr);
	initH(para_num);
	for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
	{
		for (f = opt->mode > PMODE_DGPS ? 0 : nf; f < nf * 2; f++)
		{
			for (j = 0; j < ns; j++) 
			{
				clearN();
				sat_id = sat[j] - 1;
				if (NULL == rtk->ssat[sat_id])
				{
					continue;
				}
				sysj = rtk->ssat[sat_id]->sys;
				if (!test_sys(sysj, m) || SYS_GLO == sysj)
				{
					continue;
				}
				if (nv >= limit_obs_num)
				{
					continue;
				}
				if ((rtk->ssat[sat_id]->freq_valid & FREQ_MASK(f % nf)) == 0)
				{
					continue;
				}
				ff = f % nf;
				if ((rtk->ssat[sat_id]->usectrl.rqflag[ff] & PE_MEAS_VALID_PR) == 0 && f >= nf)  //reject sat pseudo-range
				{
					continue;
				}
				if (f < nf)
				{
					amb_index = -1;
					index = f * MAXSAT + sat_id;
					for (k = 0; k < para_num; ++k)
					{
						if (ix[k] == sat_amb_index[index])
						{
							amb_index = k;
							break;
						}
					}
				}
				if (f < nf && amb_index < 0)
				{
					continue;
				}
				if (!validobs(iu[j], ir[j], f, nf, y, rtk->rtduseflag))
				{
					continue;
				}

				lamj = nav->lam[sat[j] - 1][ff];
				if (lamj <= 0.0)
				{
					continue;
				}
				/* single-differenced residual */
				v = (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);
				/* partial derivatives by rover position */
				for (k = 0; k < 3; k++)
				{
					addH(pva_index[k], -e[k + iu[j] * 3]);
				}
				/*add the receiver clock parameter*/
				addH(rcv_clock_index[m], 1.0);
				v -= rec_clk_dcb[m];
				/*add the receiver dcb parameter*/
				if (ff >= 1)
				{
					addH(rcv_clock_index[ff * NSYSDD + m], 1.0);
					v -= rec_clk_dcb[ff * NSYSDD + m];
				}
				if (f < nf) 
				{
					index = f * MAXSAT + sat_id;
					v -= lamj * x[sat_amb_index[index]];
					addH(amb_index, lamj);
				}
				if ((rtk->rtduseflag || (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310
                                 && rtk->kfcnt < 10 && gpz_kf_data->posRes < 4.0 && gpz_kf_data->meas_blk->avgCno >= 38)) && f >= nf)
				{
					Rj = get_pe_meas_prNoise(sat[j], ff == 0 ? ff : (ff + 1)) /*/ rtk->Qratio*/;
					if (Rj < 0)
					{
						continue;
					}
				}
				else 
				{
					/* single-differenced measurement error variances */
					Rj = varerr(sat[j], sysj, azel[1 + iu[j] * 2], bl, dt, f, opt);
				}
				/* single-differenced measurement error variances */
				//r_factor = gnss_rtk_adjerror(rtk, sat[j], ns, fabs(v), f, Rj);
				//Rj *= r_factor;
				setOMC(v, Rj);
				predictStep(delta_x, P_);
				if (f >= nf)
				{
					if (rtk->rtduseflag)
					{
						pseudo_res_update_thres = 10.0;
					}
					else
					{
						pseudo_res_update_thres = 50.0;
					}
					if (fabs(getInnoRes()) > pseudo_res_update_thres)
					{
						continue;
					}
				}
				measUpdate(x_value, delta_x, P_, 0.0);
				++nv;
				gnss_util_trace(3, "sat=%3d-%3d %s%d v=%13.3f R=%8.6f\n", sat[i],
					sat[j], f < nf ? "L" : "P", f% nf + 1, v, Rj);
			}
		}
	}
	for (i = 0; i < para_num; ++i) 
	{
		rtk->x[ix[i]] = x_value[i];
		row_index = i * para_num;
		for (j = 0; j <= i; ++j)
		{
			rtk->P[ISM(ix[i], ix[j])] = P_[row_index + j];
		}
	}
	reset_seq_ekf();
	freeSeqMatrix();
	Sys_Free(ix);
	Sys_Free(P_);
	Sys_Free(x_value);
	Sys_Free(delta_x);
	return nv;
}
/* double-differenced fixed phase residuals -----------------------------------*/
static int ag_rtk_fixed_phase_ddres(rtk_t* rtk, const nav_t* nav, const double* x,const int* sat, double* y, 
	double* azel, const int* iu, const int* ir, int ns, double* v,int* vflg)
{
	prcopt_t* opt = &rtk->opt;
	double posu[3], posr[3], didxi = 0.0, didxj = 0.0;
    double* im = NULL;
	double* tropr = NULL;
	double* tropu = NULL;
    double* dtdxr = NULL;
    double* dtdxu = NULL;
	double lami, lamj, fi, fj, df;
	int i, j, m, f, ff, nv = 0, sysi, sysj, nf = NF(opt);
	if (opt->ionoopt >= IONOOPT_EST || opt->tropopt >= TROPOPT_EST)
	{
		ecef2pos(x, posu); ecef2pos(rtk->rb, posr);
		im = mat(ns, 1);
		tropu = mat(ns, 1); tropr = mat(ns, 1); dtdxu = mat(ns, 3); dtdxr = mat(ns, 3);
		/* compute factors of ionospheric and tropospheric delay */
		for (i = 0; i < ns; i++) 
		{
			if (opt->ionoopt >= IONOOPT_EST) 
			{
				im[i] = (ionmapf(posu, azel + iu[i] * 2) + ionmapf(posr, azel + ir[i] * 2)) / 2.0;
			}
			if (opt->tropopt >= TROPOPT_EST) 
			{
				tropu[i] = prectrop(rtk->sol.time, posu, 0, azel + iu[i] * 2, opt, x, dtdxu + i * 3);
				tropr[i] = prectrop(rtk->sol.time, posr, 1, azel + ir[i] * 2, opt, x, dtdxr + i * 3);
			}
		}
	}
	for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
	{
		for (f = opt->mode > PMODE_DGPS ? 0 : nf; f < nf * 2; f++)
		{
			i = gnss_rtk_get_ref(DDRES_FIXED, rtk, azel, iu, ir, ns, x, sat, y, m, f);
			if (i < 0)
			{
				continue;
			}
			/* make double difference */
			for (j = 0; j < ns; j++)
			{
				if (i == j)
				{
					continue;
				}
				if (rtk->ssat[sat[i] - 1] == NULL || rtk->ssat[sat[j] - 1] == NULL)
				{
					continue;
				}
				sysi = rtk->ssat[sat[i] - 1]->sys;
				sysj = rtk->ssat[sat[j] - 1]->sys;
				if (!test_sys(sysj, m))
				{
					continue;
				}
				if ((g_pe_cfg.meas_rate > MEAS_RATE_1HZ && nv >= CMN_OBS_NUM_HF) || (g_pe_cfg.meas_rate == MEAS_RATE_1HZ && nv >= CMN_OBS_NUM))
				{
					continue;
				}
				if ((rtk->ssat[sat[j] - 1]->freq_valid & FREQ_MASK(f % nf)) == 0)
				{
					continue;
				}
				if (f < nf && rtk->ix[IB(sat[j], f, opt)] < 0)
				{
					continue;
				}
				if (!validobs(iu[j], ir[j], f, nf, y, rtk->rtduseflag))
				{
					continue;
				}

				ff = f % nf;
				if ((rtk->ssat[sat[j] - 1]->fix[ff] != 2))
				{
					continue;
				}
				lami = nav->lam[sat[i] - 1][ff];
				lamj = nav->lam[sat[j] - 1][ff];
				if (lami <= 0.0 || lamj <= 0.0)
				{
					continue;
				}
				/* double-differenced residual */
				v[nv] = (y[f + iu[i] * nf * 2] - y[f + ir[i] * nf * 2]) -
					(y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);
				/* double-differenced ionospheric delay term */
				if (opt->ionoopt == IONOOPT_EST)
				{
					fi = lami / lam_carr[0]; fj = lamj / lam_carr[0];
					if (i < (ns * 1) && NULL != im)
					{
						didxi = (f < nf ? -1.0 : 1.0) * fi * fi * im[i];
						didxj = (f < nf ? -1.0 : 1.0) * fj * fj * im[j];
						v[nv] -= didxi * x[II(sat[i], opt)] - didxj * x[II(sat[j], opt)];
					}
				}
				/* double-differenced tropospheric delay term */
				if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG)
				{
					v[nv] -= (tropu[i] - tropu[j]) - (tropr[i] - tropr[j]);
				}
				/* double-differenced phase-bias term */
				if (f < nf)
				{
					if (opt->ionoopt != IONOOPT_IFLC)
					{
						v[nv] -= lami * x[rtk->ix[IB(sat[i], f, opt)]] - lamj * x[rtk->ix[IB(sat[j], f, opt)]];
					}
					else
					{
						v[nv] -= x[IB(sat[i], f, opt)] - x[IB(sat[j], f, opt)];
					}
				}
				/* glonass receiver h/w bias term */
				if (rtk->opt.glomodear == 2 && sysi == SYS_GLO && sysj == SYS_GLO && ff < NFREQGLO)
				{
					df = (CLIGHT / lami - CLIGHT / lamj) / 1E6; /* freq-difference (MHz) */
					v[nv] -= df * x[IL(ff, opt)];
				}
				/* glonass interchannel bias correction */
				else if (sysi == SYS_GLO && sysj == SYS_GLO)
				{

					v[nv] -= gloicbcorr(sat[i], sat[j], &rtk->opt, lami, lamj, f);
				}
				vflg[nv++] = (sat[i] << 16) | (sat[j] << 8) | ((f < nf ? 0 : 1) << 4) | (f % nf);
			}
		}
	}
	if (opt->ionoopt >= IONOOPT_EST || opt->tropopt >= TROPOPT_EST)
	{
		Sys_Free(im);
		Sys_Free(tropu); Sys_Free(tropr); Sys_Free(dtdxu); Sys_Free(dtdxr);
	}
	return nv;
}
/* get receiver clock of single-differenced between base station and rover station -----------------------------------*/
static void asensing_rtk_rcv_clock(rtk_t* rtk, const nav_t* nav, const double* x, const int* sat,
	double* y, const int* iu, const int* ir, int ns, double pseudo_rcv_clk[NSYSDD * NFREQ])
{
	prcopt_t* opt = &(rtk->opt);
	int m = 0, f = 0, nf = NF(opt), j = 0, sysj = 0, nv = 0, num = NSYSDD * NFREQ, ff = 0;
	double v[CMN_OBS_NUM] = { 0.0 }, lamj = 0.0, df = 0.0, median_value = 0.0;
	uint8_t median_status = FALSE;
	for (j = 0; j < num; ++j)
	{
		pseudo_rcv_clk[j] = 0.0;
	}
	for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
	{
		for (f = nf; f < 2 * nf; f++)
		{
			ff = f % nf;
			memset(v, 0, sizeof(double) * CMN_OBS_NUM);
			nv = 0;
			for (j = 0; j < ns; j++)
			{
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				sysj = rtk->ssat[sat[j] - 1]->sys;
				if (!test_sys(sysj, m))
				{
					continue;
				}
				if (nv >= CMN_OBS_NUM)
				{
					continue;
				}
				if ((rtk->ssat[sat[j] - 1]->freq_valid & FREQ_MASK(ff)) == 0)
				{
					continue;
}
				if (!validobs(iu[j], ir[j], f, nf, y, rtk->rtduseflag))
				{
					continue;
				}
				/* single-differenced residual */
				v[nv] = (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);
				/* glonass receiver h/w bias term */
				if (rtk->opt.glomodear == 2 && sysj == SYS_GLO && f < NFREQGLO)
				{
					lamj = nav->lam[sat[j] - 1][f];
					df = (CLIGHT / lamj) / 1E6; /* freq-difference (MHz) */
					v[nv] -= df * x[IL(ff, opt)];
				}
				++nv;
			}
			median_value = 0.0;
			median_status = gnss_median_dbl(v, nv, &median_value);
			if (TRUE == median_status)
			{
				pseudo_rcv_clk[ff * NSYSDD + m] = median_value;
			}
		}
	}
	return;
}
/* time-interpolation of residuals (for post-mission) ------------------------*/
static double ag_gnss_post_intpres(gtime_t time, const obsd_t *obs, int n, const nav_t *nav,
                      rtk_t *rtk, double *y)
{
#if 0

    static obsd_t obsb[MAXOBS];
    static double yb[MAXOBS*NFREQ*2],rs[MAXOBS*6],dts[MAXOBS*2],var[MAXOBS];
    static double e[MAXOBS*3],azel[MAXOBS*2];
    static int nb=0,svh[MAXOBS*2];
    prcopt_t *opt=&rtk->opt;
    double tt=timediff(time,obs[0].time),ttb,*p,*q;
    int i,j,k,nf=NF(opt);
    
    gnss_util_trace(3,"ag_gnss_post_intpres : n=%d tt=%.1f\n",n,tt);
    
    if (nb==0||fabs(tt)<DTTOL) {
        nb=n; for (i=0;i<n;i++) obsb[i]=obs[i];
        return tt;
    }
    ttb=timediff(time,obsb[0].time);
    if (fabs(ttb)>opt->maxtdiff*2.0||ttb==tt) return tt;
    
    ag_gnss_satposs(time,obsb,nb,nav,opt->sateph,rs,dts,var,svh,azel);
    
    if (!ag_gnss_zdres(1,obsb,nb,rs,dts,svh,nav,rtk->rb,opt,1,yb,e,azel)) {
        return tt;
    }
    for (i=0;i<n;i++) {
        for (j=0;j<nb;j++) if (obsb[j].sat==obs[i].sat) break;
        if (j>=nb) continue;
        for (k=0,p=y+i*nf*2,q=yb+j*nf*2;k<nf*2;k++,p++,q++) {
            if (*p==0.0||*q==0.0) *p=0.0; else *p=(ttb*(*p)-tt*(*q))/(ttb-tt);
        }
    }
    return fabs(ttb)>fabs(tt)?ttb:tt;
#endif
	return 0.0;
}
static int gnss_rtk_dessort_int(int *a, int n)
{
	int temp = 0;
	int i, j;
	if (a == NULL) return 0;

	for (i = n - 1; i > 0; i--)
	{
		for (j = 0; j < i; j++)
		{
			if (a[j] < a[j + 1])
			{
				temp = a[j];
				a[j] = a[j + 1];
				a[j + 1] = temp;
			}
		}
	}
	return 1;
}
static int gnss_rtk_dessort_dbl(double *a, int n)
{
	double temp = 0.0;
	int i, j;
	if (a == NULL) return 0;

	for (i = n - 1; i > 0; i--)
	{
		for (j = 0; j < i; j++)
		{
			if (a[j] < a[j + 1])
			{
				temp = a[j];
				a[j] = a[j + 1];
				a[j + 1] = temp;
			}
		}
	}
	return 1;
}
/* select  sat to fix by cn0 -------------------------------------------------------
*args   : rtk_t *rtk
* return : fixed cn0
* author : none
*-----------------------------------------------------------------------------*/
static int gnss_rtk_ambfix_choice(rtk_t *rtk)
{
	int m, f, nofix, noglofix, nf = NF(&rtk->opt), i, j, k = 0, cnt = 0, thres = 0, na = rtk->na;
	int cn0[CMN_OBS_NUM] = { 0 }, cn0_back[CMN_OBS_NUM] = { 0 }, VPT_cn0 = 0;
	double ele[CMN_OBS_NUM] = { 0.0 }, ele_back[CMN_OBS_NUM] = { 0.0 }, VPT_ele = 0.0;
	double scale = 0.8;

	for (m = 0; m < NSYSDD; m++)
	{
		nofix = (m == 1 && rtk->opt.glomodear == 0) || (m == 3 && rtk->opt.bdsmodear == 0);
		for (f = 0; f < nf; f++)
		{
			noglofix = (m == 1 && rtk->opt.glomodear == 1 && !(glo_ar_freq_mask&(1 << f)));
			for (i = 0; i < MAXSAT; i++)
			{
				if (rtk->ssat[i] == NULL) continue;

				if (rtk->ix[i + na] < 0.0 || !test_sys(rtk->ssat[i]->sys, m) ||
					!rtk->ssat[i]->vsat[f]) continue;

				if (((rtk->ssat[i]->usectrl.rqflag[f] & PE_MEAS_VALID_PR) == 0) && fabs(rtk->ssat[i]->resp[f]) > 5.0)
					continue;

				if (rtk->ssat[i]->lock[f] > 0 && !(rtk->ssat[i]->slip[f] & 2) &&
					rtk->ssat[i]->vsat[f] && rtk->ssat[i]->azel[1] >= rtk->opt.elmaskar && !nofix
					&& !noglofix && !(rtk->ssat[i]->usectrl.fix_rejflag&FREQ_MASK(f)) &&
					!(rtk->ssat[i]->usectrl.halfflag&FREQ_MASK(f)))
				{
					cn0[cnt] = rtk->ssat[i]->snr[f]/4;
					ele[cnt] = rtk->ssat[i]->azel[1];
					cnt++;
					if (cnt >= CMN_OBS_NUM) break;
				}
			}
		}
	}

	if (cnt < 10) return 0;

	for (i = 0; i < cnt; i++)
	{
		cn0_back[i] = cn0[i];
		ele_back[i] = ele[i];
	}

	gnss_rtk_dessort_int(cn0, cnt);
	gnss_rtk_dessort_dbl(ele, cnt);

	for (k = 0; k < 5; k++)
	{
		thres = (int)(cnt*(scale + 0.1*k));

		if (thres >= cnt) break;

		VPT_cn0 = cn0[thres];
		VPT_ele = ele[thres];

		for (j = 0, i = 0; i < cnt; i++)
		{
			if (cn0_back[i] > VPT_cn0&&ele_back[i] > VPT_ele)
			{
				j++;
			}
		}

		if (j > cnt*scale || j > 10) break;
	}

	rtk->VPT_cn0 = (VPT_cn0 > 42) ? 42 : VPT_cn0;
	rtk->VPT_ele = (VPT_ele > 40 * D2R) ? 40 * D2R : VPT_ele;

	GLOGI("AMB fix VPT_cn0=%d VPT_ele=%.2f", rtk->VPT_cn0, rtk->VPT_ele*R2D);

	return 1;
}
/* single to double-difference satellite list and reference satellite--------------------*/
static int ddmat(rtk_t *rtk, fix_sat_info* sat_list)
{
    int i,j,k,m,f,nb=0/*,nx=rtk->nx*/,na=rtk->na,nf=NF(&rtk->opt),nofix,nb_f,noglofix, flag = 0;

	if (NULL == sat_list)
	{
		return 0;
	}
	if (rtk->ambfix_flag == Part_LAMBDA_ELECN0)
	{
		flag = gnss_rtk_ambfix_choice(rtk);

		if (flag == 0)
		{
			return 0;
		}

		if (rtk->VPT_cn0 < 30) rtk->VPT_cn0 = 30;
		if (rtk->VPT_ele < rtk->opt.elmaskar) rtk->VPT_ele = rtk->opt.elmaskar;
	}
	else
	{
		rtk->VPT_cn0 = 0;
		rtk->VPT_ele = 0.0;
	}

	for (i = 0; i < MAXSAT; i++) for (j = 0; j < NFREQ; j++)
	{
		if (rtk->ssat[i] == NULL)
		{
			continue;
		}
		rtk->ssat[i]->fix[j] = 0;
	}
	for (m = 0; m < NSYSDD; m++)
	{ /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */

		nofix = (m == 1 && rtk->opt.glomodear == 0) || (m == 3 && rtk->opt.bdsmodear == 0);

		for (f = 0, k = na; f < nf; f++, k += MAXSAT)
		{
			nb_f = 0;
			noglofix = (m == 1 && rtk->opt.glomodear == 1 && !(glo_ar_freq_mask & (1 << f)));
			for (i = k; i < k + MAXSAT; i++)
			{
				if (rtk->ssat[i - k] == NULL)
				{
					continue;
				}
				if (rtk->ix[i] < 0.0 || !test_sys(rtk->ssat[i - k]->sys, m) ||
					!rtk->ssat[i - k]->vsat[f] || !rtk->ssat[i - k]->half[f])
				{
					continue;
				}
				if (rtk->ssat[i - k]->lock[f] > 0 && !(rtk->ssat[i - k]->slip[f] & 2) && (!(rtk->ssat[i - k]->ambfix[f])) &&
					(rtk->ssat[i - k]->snr[f] / 4 >= rtk->VPT_cn0) && (rtk->ssat[i - k]->azel[1] >= rtk->VPT_ele) &&
					rtk->ssat[i - k]->azel[1] >= rtk->opt.elmaskar && !nofix && !noglofix && !(rtk->ssat[i - k]->usectrl.fix_rejflag & FREQ_MASK(f)) &&
					!(rtk->ssat[i - k]->usectrl.halfflag & FREQ_MASK(f)))
				{
					rtk->ssat[i - k]->fix[f] = 2; /* fix */
					GLOGD("ddmat refsat=%d lock=%4d slip=%02x half=%d ele=%4.1f slipc=%u", i - k + 1, rtk->ssat[i - k]->lock[f], rtk->ssat[i - k]->slip[f],
						rtk->ssat[i - k]->half[f], rtk->ssat[i - k]->azel[1] * R2D, rtk->ssat[i - k]->slipc[f]);
					break;
				}
				else
				{
					rtk->ssat[i - k]->fix[f] = 1;
				}
			}
			if (i >= k + MAXSAT)
			{
				continue;
			}
			for (j = k; j < k + MAXSAT; j++)
			{
				if (rtk->ssat[j - k] == NULL)
				{
					continue;
				}
				if (i == j || rtk->ix[j] < 0 || !test_sys(rtk->ssat[j - k]->sys, m))
				{
					continue;
				}
				if (!rtk->ssat[j - k]->vsat[f])
				{
					rtk->ssat[j - k]->fix[f] = 0;
				}
				else if (rtk->ssat[j - k]->lock[f] > 0 && !(rtk->ssat[j - k]->slip[f] & 2) && (!(rtk->ssat[j - k]->ambfix[f])) && rtk->ssat[i - k]->vsat[f] &&
					(rtk->ssat[j - k]->snr[f] / 4 >= rtk->VPT_cn0) && (rtk->ssat[j - k]->azel[1] >= rtk->VPT_ele) &&
					rtk->ssat[j - k]->azel[1] >= rtk->opt.elmaskar && !nofix && !noglofix && !(rtk->ssat[j - k]->usectrl.fix_rejflag & FREQ_MASK(f)) &&
					!(rtk->ssat[j - k]->usectrl.halfflag & FREQ_MASK(f)))
				{
					sat_list[nb].sat_id = j - k + 1;
					sat_list[nb].freq = f;
					sat_list[nb].amb_index = rtk->ix[j];
					sat_list[nb].ref_amb_index = rtk->ix[i];
					nb++;
					nb_f++;
					rtk->ssat[j - k]->fix[f] = 2; /* fix */
				}
				else
				{
					rtk->ssat[j - k]->fix[f] = 1;
				}
				GLOGD("%3d vsat=%d lock=%4d slip=%02x half=%d ele=%4.1f slipc=%5u fix=%u", j - k + 1, rtk->ssat[j - k]->vsat[f], rtk->ssat[j - k]->lock[f],
					rtk->ssat[j - k]->slip[f], rtk->ssat[j - k]->half[f], rtk->ssat[j - k]->azel[1] * R2D, rtk->ssat[j - k]->slipc[f], rtk->ssat[j - k]->fix[f]);
			}
			if (nb_f == 0)
			{
				rtk->ssat[i - k]->fix[f] = 1;
			}
			if (f == 0 && m == 0)
			{
				rtk->fix_amb_L1[0] = nb_f;
			}
			else if (f == 0 && m == 3)
			{
				rtk->fix_amb_L1[1] = nb_f;
			}
		}
	}
	return nb;
}
static int gnss_rtk_amb_delbyddamb(rtk_t *rtk, const double *bias)
{
	int i, n = 0, t = 0, m, f, z, index[MAXSAT] = { 0 }, nv = 0, nf = NF(&(rtk->opt));
	float dot = 0.4f;

	for (m = 0; m < NSYSDD; m++) for (f = 0; f < nf; f++) {

		for (n = i = 0; i < MAXSAT; i++) {
			if (rtk->ssat[i] == NULL)
			{
				continue;
			}
			if (!test_sys(rtk->ssat[i]->sys, m))
			{
				continue;
			}
			if (rtk->ssat[i]->fix[f] != 2)
			{
				continue;
			}
			index[n++] = i;
		}

		if (n < 2) continue;

		z = 0;
		for (i = 1; i < n; i++)
		{
			gnss_util_trace(3, "amb bias=%.3f dot=%.3f\n", fabs(bias[nv] - round(bias[nv])), dot);
			if (NULL == rtk->ssat[index[i]])
			{
				continue;
			}
			if (fabs(bias[nv] - round(bias[nv])) > dot)
			{
				t++;
				z++;
				rtk->ssat[index[i]]->ambfix[f] = 1;
				GLOGI("Delete amb by dd amb sat=%d,ddamb=%.3f f=%d", index[i] + 1, bias[nv], f);
				gnss_util_trace(3, "Delete amb by dd amb sat=%d,ddamb=%.3f\n", index[i] + 1, bias[nv]);
			}
			nv++;
		}
		/*ref sat not to fix*/
		if (z > (n * 2 / 3))
		{
			if (rtk->ssat[index[0]])
			{
				rtk->ssat[index[0]]->ambfix[f] = 1;
			}
			GLOGI("Delete amb refsat sat=%d f=%d delcnt=%d fixcnt=%d", index[0] + 1, f, z, n);
		}
	}
	GLOGI("AMB PROCESS t=%d dot=%.1f", t, dot);
	return t;
}
/* restore single-differenced ambiguity --------------------------------------*/
static void restamb(rtk_t *rtk, const double *bias, int nb, double *xa)
{
    int i,n,m,f,index[MAXSAT],nv=0,nf=NF(&rtk->opt);
    
    gnss_util_trace(3,"restamb :\n");
    
    for (i=0;i<rtk->nx;i++) xa[i]=rtk->x [i];
    for (i=0;i<rtk->na;i++) xa[i]=rtk->xa[i];
    
    for (m=0;m<NSYSDD;m++) for (f=0;f<nf;f++) {
        
        for (n=i=0;i<MAXSAT;i++) {
			if (rtk->ssat[i] == NULL)
			{
				continue;
			}

            if (!test_sys(rtk->ssat[i]->sys,m)||rtk->ssat[i]->fix[f]!=2) {
                continue;
            }
			index[n++] = rtk->ix[IB(i + 1, f, &rtk->opt)];
        }
        if (n<2) continue;
        
        xa[index[0]]=rtk->x[index[0]];
        
        for (i=1;i<n;i++) {
            xa[index[i]]=xa[index[0]]-bias[nv++];
        }
    }
}
/* hold integer ambiguity ----------------------------------------------------*/
static void ag_rtk_holdamb(rtk_t *rtk, const double *xa, int prefixhold)
{
    double *v,*H,*R;
    int i,n,m,f,info,index[MAXSAT], index_prefix[MAXSAT], nb=rtk->nx-rtk->na,nv=0,nf=NF(&rtk->opt);
    
    gnss_util_trace(3,"ag_rtk_holdamb :\n");
    
    v=mat(nb,1); H=zeros(nb,rtk->nx);
    
    for (m=0;m<NSYSDD;m++) for (f=0;f<nf;f++) {
        
        for (n=i=0;i<MAXSAT;i++) {
			if (rtk->ssat[i] == NULL)
			{
				continue;
			}
			if (prefixhold == FALSE)
			{
				if (!test_sys(rtk->ssat[i]->sys, m) || rtk->ssat[i]->fix[f] != 2 ||
					rtk->ssat[i]->azel[1] < rtk->opt.elmaskhold) {
					continue;
				}
			}
			else
			{
				if (!test_sys(rtk->ssat[i]->sys, m) || rtk->ssat[i]->fix[f] == 0 ||
					!(rtk->fixstate[i] & FREQ_FIX(f)) || rtk->ix[IB(i + 1, f, &rtk->opt)] < 0 ||
					rtk->ssat[i]->azel[1] < rtk->opt.elmaskhold) {
					continue;
				}
				index_prefix[n] = f * MAXSAT + i;
			}
			index[n++] = rtk->ix[IB(i + 1, f, &rtk->opt)];
			rtk->ssat[i]->fix[f] = 3; /* hold */
        }
        /* constraint to fixed ambiguity */
        for (i=1;i<n;i++) {
			if (prefixhold == FALSE)
			{
				v[nv] = (xa[index[0]] - xa[index[i]]) - (rtk->x[index[0]] - rtk->x[index[i]]);
			}
			else
			{
				if (NULL != rtk->ssat[index_prefix[0] % MAXSAT] && NULL != rtk->ssat[index_prefix[i] % MAXSAT])
				{
					v[nv] = (rtk->ssat[index_prefix[0] % MAXSAT]->sdamb[index_prefix[0] / MAXSAT]
						- rtk->ssat[index_prefix[i] % MAXSAT]->sdamb[index_prefix[i] / MAXSAT])
						- (rtk->x[index[0]] - rtk->x[index[i]]);

					//GLOGW("use prefix to hold: tor %.3f, prefix amb %08.3f, float amb %08.3f", rtk->ptor, (rtk->ssat[index_prefix[0] % MAXSAT]->sdamb[index_prefix[0] / MAXSAT]
					//	- rtk->ssat[index_prefix[i] % MAXSAT]->sdamb[index_prefix[i] / MAXSAT]), (rtk->x[index[0]] - rtk->x[index[i]]));
				}
			}
			H[index[0] + nv * rtk->nx] = 1.0;
			H[index[i] + nv * rtk->nx] = -1.0;
			nv++;
        }
    }
    if (nv>0) {
        R=zeros(nv,nv);
        for (i=0;i<nv;i++) R[i+i*nv]=VAR_HOLDAMB;
        
        /* update states with constraints */
        if ((info= ag_rtk_kf_filter(rtk->x,rtk->P,H,v,R,rtk->nx,nv))) {
			gnss_util_trace(2,"filter error (info=%d)\n",info);
        }
        Sys_Free(R);
    }
    Sys_Free(v); Sys_Free(H);
}
/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
static int ag_rtk_LAMBDA(rtk_t *rtk, fix_sat_info* sat_list, int nb, double *bias, double *xa)
{
    prcopt_t *opt=&rtk->opt;
	int i, j, j_ref = 0, k = 0, ny, info, nx = rtk->nx, na = rtk->na, * ix, n = rtk->nx, m = 0/*, nf = NF(&rtk->opt), f = 0*/;
	double* y, * Qy, * b, * db, * Qb, * Qab, * QQ, s[2], * sub_y, * sub_x, * one_row_value = NULL;
	double* x, * P, ref_amb = 0.0;
	int ref_index = -1, ref_index_com = -1, index = 0, * fix_ix = NULL, ref_sat_index_set[NSYSDD * NFREQ];
	int max_ref_num = NSYSDD * NFREQ, row_temp = 0;
	sub_x = mat(NR(opt) + NC(opt), 1);
    rtk->sol.ratio=0.0;

	ix = imat(n, 1);
	ny = na + nb;
	y = mat(ny, 1);
	Qy = mat(ny, ny);
	fix_ix = imat(n, 1);
	for (i = nx = 0; i < NX(opt); i++)
	{
		if (rtk->ix[i] > -1 && rtk->P[ISM(rtk->ix[i], rtk->ix[i])] > 0.0)
		{
			ix[nx++] = i;
		}
	}
	one_row_value = mat(nx, 1);
	x = mat(nx, 1); P = mat(nx, nx);

	for (i = 0; i < nx; i++) 
	{
		x[i] = rtk->x[rtk->ix[ix[i]]];
		row_temp = i * nx;
		for (j = 0; j < nx; j++)
		{
			P[row_temp + j] = rtk->P[ISM(rtk->ix[ix[i]], rtk->ix[ix[j]])];
		}
	}
	for (i = 0; i < na; ++i)
	{
		for (j = 0; j < nx; ++j)
		{
			if (ix[j] == i)
			{
				break;
			}
		}
		if (j < nx)
		{
			y[i] = x[j];
		}
	}

	for (i = 0; i < max_ref_num; ++i)
	{
		ref_sat_index_set[i] = -1;
	}
	index = 0;
	for (i = 0; i < nb; ++i)
	{
		ref_index = sat_list[i].ref_amb_index;
		for (j_ref = 0; j_ref < nx; ++j_ref)
		{
			if (rtk->ix[ix[j_ref]] == ref_index)
			{
				break;
			}
		}
		if (j_ref < nx)
		{
			for (j = 0; j < index; ++j)
			{
				if (ref_sat_index_set[j] == ref_index)
				{
					break;
				}
			}
			if (j >= index)
			{
				ref_sat_index_set[index++] = ref_index;
			}
		}
	}
	
	for (i = 0; i < index; ++i)
	{
		ref_index = ref_sat_index_set[i];
		for (j_ref = 0; j_ref < nx; ++j_ref)
		{
			if (rtk->ix[ix[j_ref]] == ref_index)
			{
				break;
			}
		}
		if (j_ref >= nx)
		{
			continue;
		}
		memset(one_row_value, 0, nx * sizeof(double));
		ref_amb = x[j_ref];
		row_temp = j_ref * nx;
		for (k = 0; k < nx; ++k)
		{
			one_row_value[k] = P[row_temp + k];
		}
		for (k = 0; k < nb; ++k)
		{
			ref_index_com = sat_list[k].ref_amb_index;
			if (ref_index != ref_index_com)
			{
				continue;
			}
			for (j = 0; j < nx; ++j)
			{
				if (rtk->ix[ix[j]] == sat_list[k].amb_index)
				{
					break;
				}
			}
			if (j >= nx)
			{
				continue;
			}
			y[na + k] = ref_amb - x[j];
			row_temp = j * nx;
			for (m = 0; m < nx; ++m)
			{
				P[row_temp + m] = one_row_value[m] - P[row_temp + m];
			}
		}
	}

	for (i = 0; i < index; ++i)
	{
		ref_index = ref_sat_index_set[i];
		for (j_ref = 0; j_ref < nx; ++j_ref)
		{
			if (rtk->ix[ix[j_ref]] == ref_index)
			{
				break;
			}
		}
		if (j_ref >= nx)
		{
			continue;
		}
		memset(one_row_value, 0, nx * sizeof(double));
		for (k = 0; k < nx; ++k)
		{
			one_row_value[k] = P[k * nx + j_ref];
		}
		for (k = 0; k < nb; ++k)
		{
			ref_index_com = sat_list[k].ref_amb_index;
			if (ref_index != ref_index_com)
			{
				continue;
			}
			for (j = 0; j < nx; ++j)
			{
				if (rtk->ix[ix[j]] == sat_list[k].amb_index)
				{
					break;
				}
			}
			if (j >= nx)
			{
				continue;
			}
			for (m = 0; m < nx; ++m)
			{
				row_temp = m * nx;
				P[row_temp + j] = one_row_value[m] - P[row_temp + j];
			}
		}
	}
	
	index = 0;
	for (k = 0; k < na; ++k)
	{
		for (j = 0; j < nx; ++j)
		{
			if (ix[j] == k)
			{
				break;
			}
		}
		if (j < nx)
		{
			fix_ix[index++] = j;
		}
	}

	for (k = 0; k < nb; ++k)
	{
		for (j = 0; j < nx; ++j)
		{
			if (rtk->ix[ix[j]] == sat_list[k].amb_index)
			{
				break;
			}
		}
		if (j < nx)
		{
			fix_ix[index++] = j;
		}
	}

	for (i = 0; i < index; ++i)
	{
		row_temp = fix_ix[i] * nx;
		for (j = 0; j < index; ++j)
		{
			Qy[j * ny + i] = P[row_temp + fix_ix[j]];
		}
	}

	Sys_Free(ix); Sys_Free(fix_ix); Sys_Free(P);
    /* transform single to double-differenced phase-bias (y=D'*x, Qy=D'*P*D) */
	sub_y = mat(nb, 1);
	Sys_Free(x);
	Sys_Free(one_row_value);

	Qb = mat(nb, nb); Qab = mat(na, nb);
    /* phase-bias covariance (Qb) and real-parameters to bias covariance (Qab) */
    for (i=0;i<nb;i++) for (j=0;j<nb;j++) Qb [i+j*nb]=Qy[na+i+(na+j)*ny];
    for (i=0;i<na;i++) for (j=0;j<nb;j++) Qab[i+j*na]=Qy[   i+(na+j)*ny];
	Sys_Free(Qy);
    
	b = mat(nb, 2);
	info = lambda(nb, 2, y + na, Qb, b, s);
    /* lambda/mlambda integer least-square estimation */
    if (!info) {    
        rtk->sol.ratio=s[0]>0?(float)(s[1]/s[0]):0.0f;
        if (rtk->sol.ratio>999.9) rtk->sol.ratio=999.9f;
		if (s[0] > 0) { rtk->fat.rs0 = s[0] > 999.9f? 999.9f:s[0], rtk->fat.rs1 = s[1] > 999.9f ? 999.9f : s[1]; }
		else { rtk->fat.rs0 = 999.9f, rtk->fat.rs1 = 999.9f; }
        
        /* validation by popular ratio-test */
        if (s[0]<=0.0||s[1]/s[0]>=opt->thresar[0]) {
            
            /* transform float to fixed solution (xa=xa-Qab*Qb\(b0-b)) */
            for (i=0;i<na;i++) {
                rtk->xa[i]=rtk->x[i];
				sub_x[i] = rtk->x[i];
                for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[ISM(i, j)];
            }
            for (i=0;i<nb;i++) {
                bias[i]=b[i];
				sub_y[i] = y[na + i] - b[i + nb];
                y[na+i]-=b[i];
            }
            if (!matinvsm(Qb,nb)) {
				db = mat(nb, 1);
#ifndef ASM_MATMUL
                matmul("NN",nb,1,nb, 1.0,Qb ,y+na,0.0,db);
                matmul("NN",na,1,nb,-1.0,Qab,db  ,1.0,rtk->xa);

				//sub fix
				matmul("NN", nb, 1, nb, 1.0, Qb, sub_y, 0.0, db);
				matmul("NN", na, 1, nb, -1.0, Qab, db, 1.0, sub_x);
#else
				Fast_Rtk_matmul("NN", nb, 1, nb, 1.0, Qb, y + na, 0.0, db);
				Fast_Rtk_matmul("NN", na, 1, nb, -1.0, Qab, db, 1.0, rtk->xa);
#endif
				Sys_Free(db);
				Sys_Free(sub_y);

                /* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
				QQ = mat(na, nb);
#ifndef ASM_MATMUL
                matmul("NN",na,nb,nb, 1.0,Qab,Qb ,0.0,QQ);
                matmul("NT",na,na,nb,-1.0,QQ ,Qab,1.0,rtk->Pa);
#else
				Fast_Rtk_matmul("NN", na, nb, nb, 1.0, Qab, Qb, 0.0, QQ);
				Fast_Rtk_matmul("NT", na, na, nb, -1.0, QQ, Qab, 1.0, rtk->Pa);
#endif
				Sys_Free(QQ);

                GLOGI("amb validation ok (nb=%d ratio=%.2f s=%.2f/%.2f)",nb,s[0]==0.0?0.0:s[1]/s[0],s[0],s[1]);
                
                /* restore single-differenced ambiguity */
                restamb(rtk,bias,nb,xa);

				// rtk fix status check
				rtk->comfirmamb = gnss_rtk_comfirmamb(rtk->x, xa, sub_x);

				rtk->amb_gap = gnss_rtk_amb_gap(y+na,b,nb);
            }
            else nb=0;
        }
        else { /* validation failed */
            GLOGI("amb validation fail (nb=%d ratio=%.2f s=%.2f/%.2f)",nb,s[1]/s[0],s[0],s[1]);
            nb=0;
        }
    }
    else {
		nb = 0;
    }
    
	Sys_Free(y); Sys_Free(b); Sys_Free(Qb); Sys_Free(Qab); Sys_Free(sub_y); Sys_Free(sub_x); Sys_Free(one_row_value);
    return nb; /* number of ambiguities */
}
static int ag_rtk_Part_LAMBDA(rtk_t *rtk, double *bias, double *xa)
{
	double *y = NULL;
	int nb = 0, ny = 0/*, nx = rtk->nx*/, na = rtk->na/*, nf = NF(&rtk->opt)*/, i = 0;
	int /*ref_num = nf * NSYSDD,*/ max_amb_num = MAXSAT * NFREQ;
	fix_sat_info sat_tobe_fix[MAXSAT * NFREQ];
	for (i = 0; i < max_amb_num; ++i)
	{
		sat_tobe_fix[i].freq = 0;
		sat_tobe_fix[i].sat_id = 0;
		sat_tobe_fix[i].amb_index = 0;
		sat_tobe_fix[i].ref_amb_index = 0;
	}

	if (rtk->opt.mode <= PMODE_DGPS || rtk->opt.modear == ARMODE_OFF ||
		rtk->opt.thresar[0] < 1.0) {
		return 0;
	}
	if (rtk->ambfix_flag == Part_LAMBDA_DDAMB)
	{
		nb = ddmat(rtk, sat_tobe_fix);
		if (nb < 16)
		{
			GLOGI("no enough double-difference for Part_LAMBDA_DDAMB,nb=%d", nb);
			return 0;
		}
		else
		{
			ny = na + nb;
			y = mat(ny, 1);
			if (NULL == y)
			{
				return 0;
			}
			for (i = 0; i < na; ++i)
			{
				y[i] = rtk->x[i];
			}
			for (i = 0; i < nb; ++i)
			{
				y[i + na] = rtk->x[sat_tobe_fix[i].ref_amb_index] - rtk->x[sat_tobe_fix[i].amb_index];
			}
			gnss_rtk_amb_delbyddamb(rtk, y + na);
			Sys_Free(y);
		}
	}

	if ((nb = ddmat(rtk, sat_tobe_fix)) <= 3) {
		GLOGI("no enough double-difference,nb=%d", nb);
		return 0;
	}
	else
	{
		rtk->lambda_nb += nb;

		if ((rtk->ambfix_flag == Part_LAMBDA_ALL) || (rtk->lambda_nb < 100))
		{
			ag_rtk_LAMBDA(rtk, sat_tobe_fix, nb, bias, xa);
		}
	}

	if (rtk->sol.ratio > rtk->opt.thresar[0])
		return nb;
	else
		return 0;
}

/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
static int ag_rtk_resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa)
{
	int nb = 0/*, i, j*/;
	double dtf = timediff(rtk->sol.time, rtk->prefixt);

	rtk->lambda_nb = 0;
	rtk->sol.ratio = 0.0;

	if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
	{
		if ((g_pe_cfg.meas_rate == MEAS_RATE_2HZ || g_pe_cfg.meas_rate == MEAS_RATE_5HZ || g_pe_cfg.meas_rate == MEAS_RATE_10HZ)
			&& rtk->prefixf && (dtf < 1) && ((g_proc.flag & g_proc.lambda_epoch) == 0))
		{
			GLOGI("not do LAMBDA at epoch:%.6f", rtk->ptor);
			return 0;
		}
	}

	rtk->ambfix_flag = Part_LAMBDA_ALL;
	nb = ag_rtk_Part_LAMBDA(rtk, bias, xa);

	if (g_pe_cfg.meas_rate == MEAS_RATE_1HZ)
	{
		if ((rtk->sol.ratio < rtk->opt.thresar[0]) && (!rtk->prefixf))
		{
			rtk->ambfix_flag = Part_LAMBDA_ELECN0;
			nb = ag_rtk_Part_LAMBDA(rtk, bias, xa);
		}

#if 0
		if ((rtk->sol.ratio < rtk->opt.thresar[0]) && (!rtk->prefixf) &&
			(rtk->shift.nofixcnt > 8) && (rtk->closkycnt & 0xF) == 0x0
			&& (rtk->nollicnt > 8) && (gpz_kf_data->meas_blk->avgCno >= 38))
		{
			rtk->ambfix_flag = Part_LAMBDA_DDAMB;
			nb = ag_rtk_Part_LAMBDA(rtk, bias, xa);

			for (i = 0; i < MAXSAT; i++)
			{
				for (j = 0; j < NFREQ; j++)
				{
					if (rtk->ssat[i] == NULL)
					{
						continue;
					}
					rtk->ssat[i]->ambfix[j] = 0;
				}
			}
		}
#endif
	}

	return nb;
}
/* should amb hold check -------------------------------------------------------------
*args   : rtk_t *rtk
* return : 1 hold, 0 not hold
* author : none
*-----------------------------------------------------------------------------*/
static int gnss_rtk_amb_HoldCheck(rtk_t* rtk)
{
	if (rtk->opt.modear == ARMODE_FIXHOLD && (g_proc.flag & 0xFC00) > 0)
	{
		if (g_pe_cfg.meas_rate == MEAS_RATE_1HZ) rtk->nfix++;
		return 1;
		//if (++rtk->nfix >= rtk->opt.minfix)
		//{
		//	return 1;
		//}
		//else
		//{
		//	if (rtk->nbias > 9 && rtk->sol.ratio > 3.0) {
		//		return 1;
		//	}
		//}
	}

	return 0;
}
/* calculate post sigma -------------------------------------------------------------
*
* args   : 
* return : sigma
* author : none
*-----------------------------------------------------------------------------*/
static double gnss_rtk_get_sigma(const double *v, const double *R, int nv, int ns, const int* vflg,int* carrier_num, int* fixed_sat_num)
{
	double sigma = 0.0;
	int i = 0, type = 0, sat2 = 0, vsat[MAXSAT] = { 0 };
	if (carrier_num)
	{
		*carrier_num = 0;
	}
	if (fixed_sat_num)
	{
		*fixed_sat_num = 0;
	}
	if (nv < 1)
	{
		return 0.0;
	}
	for (i = 0; i < nv; ++i)
	{
		type = (vflg[i] >> 4) & 0xF;
		sat2 = (vflg[i] >> 8) & 0xFF;
		if (0 != type || sat2<1 || sat2>MAXSAT)
		{
			continue;
		}
		sigma += (v[i] * v[i]);
		vsat[sat2 - 1] = 1;
		if (carrier_num)
		{
			++(*carrier_num);
		}
	}
	if (fixed_sat_num)
	{
		for (i = 0; i < MAXSAT; i++)
		{
			if (vsat[i])
			{
				++(*fixed_sat_num);
			}
		}
	}
	return sigma;
}
/* validation of solution ----------------------------------------------------*/
static int ag_rtk_valpos(rtk_t* rtk, const double* v, const double* R, const int* vflg,
	int nv, double thres)
{
#ifdef PLAYBACK_MODE
	double fact = thres * thres;
	int i, stat = 1, sat1, sat2, type, freq;
	const char* stype = NULL;

	gnss_util_trace(3, "ag_rtk_valpos  : nv=%d thres=%.1f\n", nv, thres);

	/* post-fit residual test */
	for (i = 0; i < nv; i++) {
		if (v[i] * v[i] <= fact * R[i + i * nv]) continue;
		sat1 = (vflg[i] >> 16) & 0xFF;
		sat2 = (vflg[i] >> 8) & 0xFF;
		type = (vflg[i] >> 4) & 0xF;
		freq = vflg[i] & 0xF;
		stype = type == 0 ? "L" : (type == 1 ? "P" : "C");
		gnss_util_trace(2, "large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",
			sat1, sat2, stype, freq + 1, v[i], SQRT(R[i + i * nv]));
	}

	return stat;
#else
	return 1;
#endif
}
static void ag_rtk_getprepos(rtk_t *rtk)
{
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		rtk->unc.prediff.prepos[i] = rtk->x[i];
	}
	rtk->unc.prediff.preposf = 1;
}
/* relative positioning ------------------------------------------------------*/
static int ag_rtk_relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                  const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;
    gtime_t time=obs[0].time;
    double *rs,*dts,*var,*y,*e,*azel,*v,*H,*xp,*xa,*bias,dt;
	int i, j, f, n = nu + nr, ns, ny, nv = 0, nv_fix = 0, sat[MAXSAT], iu[MAXSAT], ir[MAXSAT], niter;
    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int stat=rtk->opt.mode<=PMODE_DGPS?SOLQ_DGPS:SOLQ_FLOAT;
	int nf = opt->ionoopt == IONOOPT_IFLC ? 1 : opt->nf, carrier_num = 0, fixed_carrier_num = 0;
    sol_t solpvt=rtk->sol,solpre=rtk->sol;
    double xyz[3]={0.0};
    
    gnss_util_trace(3,"ag_rtk_relpos  : nx=%d nu=%d nr=%d\n",rtk->nx,nu,nr);
	//gnss_rtk_unc_batch_test();
    /* time update for pva, amb */
	ag_rtk_kf_udpos(rtk,rtk->tt);
	ag_rtk_getprepos(rtk);
	/* save predict sol */
	gnss_rtk_save_floatsol(rtk,&solpre);

    if (gnss_rtk_extpol_check(rtk,&solpre)) {
        rtk->kfextpcnt++;
        GLOGI("ag_rtk_relpos: time update only! extcnt=%d",rtk->kfextpcnt);
		if (rtk->kfextpcnt>=5)
		{
			rtk->kfstatus=RTK_KF_RESET;
		}
        return 1;
    }
	gnss_rtk_cfd_init(rtk);
    
    dt=timediff(time,obs[nu].time);
    
    rs=mat(6,n); 
    dts=mat(2,n); 
    var=mat(1,n); 
    y=mat(nf*2,n); 
    e=mat(3,n); 
    azel=zeros(2,n);
    
    for (i=0;i<MAXSAT;i++) {
		if (rtk->ssat[i] == NULL) continue;
        rtk->ssat[i]->sys=satsys(i+1,NULL);
		for (j = 0; j < NFREQ; j++)
		{
			rtk->ssat[i]->vsat[j] = 0;
			rtk->ssat[i]->ambfix[j] = 0;
			rtk->ssat[i]->tdy[j] = 0.0;
			rtk->ssat[i]->tdres[j] = 0.0;
		}
    }
	for(i=0;i<MAX_DDRES_NUM;i++)
	{
		rtk->dd_res[i].flag=0;
		rtk->dd_res[i].rejc=rtk->dd_res[i].rejp=0;
	}
	rtk->dewNum_cr = 0;
	rtk->dewNum_pr = 0;

    /* satellite positions/clocks */
	ag_gnss_satposs(time, obs, n, nav, EPHOPT_BRDC_TOE, rs, dts, var, svh, azel);
	Sys_Free(var);
    
    /* undifferenced residuals for base station */
    if (!ag_gnss_zdres(1,obs+nu,nr,rs+nu*6,dts+nu*2,svh+nu,nav,rtk->rb,opt,1,
               y+nu*nf*2,e+nu*3,azel+nu*2)) {
        GLOGI("base station position error: zero value");
        
        Sys_Free(rs); Sys_Free(dts); Sys_Free(y); Sys_Free(e); Sys_Free(azel);
        return 0;
    }
	/* undifferenced residuals for rover station using pvt pos */
    if (solpvt.stat==SOLQ_SINGLE) for(i=0;i<3;i++) xyz[i]=solpvt.rr[i];
    else for(i=0;i<3;i++) xyz[i]=rtk->x[i];
	ag_gnss_zdres(0,obs,nu,rs,dts,svh,nav,xyz,opt,0,y,e,azel);
    /* time-interpolation of residuals (for post-processing) */
    if (opt->intpref) {
        dt=ag_gnss_post_intpres(time,obs+nu,nr,nav,rtk,y+nu*nf*2);
    }
    /* select common satellites between rover and base-station */
    if ((ns=ag_gnss_selsat(rtk,obs,azel,nu,nr,opt,sat,iu,ir))<=0) {
        GLOGI("no common satellite");
        
        Sys_Free(rs); Sys_Free(dts); Sys_Free(y); Sys_Free(e); Sys_Free(azel);
        return 0;
    }
#ifdef USED_IN_MC262M
	if(g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9) gnss_rtk_ubx_lockv(rtk,sat,nf,ns);
#endif
	/* ref detect */
	gnss_rtk_slpdet_ref(rtk, nav, nu, nr, y, e, azel);
    /* td slip detect */
    gnss_rtk_slip_detect_td(rtk,nav,obs,nu,sat,iu,ir,ns,y,e,azel,n,xyz);

	gnss_rtk_ambp_reset_check(rtk, ns, sat);
    /* temporal update of states */

    ag_rtk_udstate(rtk,obs,sat,iu,ir,ns,nav);

    /* update previous sd amb fix status */
    gnss_rtk_sdstate_upd(rtk);

    gnss_util_trace(3,"x(0)="); gnss_util_tracemat(3,rtk->x,1,NR(opt),13,4);
    
	xp = mat(rtk->nx, 1); matcpy(xp, rtk->x, rtk->nx, 1);

	if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		ny = CMN_OBS_NUM_HF;
	}
	else
	{
		ny = CMN_OBS_NUM;
	}
	v = mat(ny, 1); 
	H = zeros(rtk->nx, ny);
    
	/* ddres check for single sol */
	if (opt->dynamics&&solpvt.stat==SOLQ_SINGLE)
	{
		for(i=0;i<3;i++) xp[i]=solpvt.rr[i];
		nv=ag_rtk_ddres(DDRES_SINGLE,rtk,nav,dt,xp,sat,y,e,azel,iu,ir,ns,v,NULL,vflg);
		gnss_rtk_kf_ddres_check(DDRES_SINGLE,rtk,v,vflg,nv,ns);
		for(i=0;i<3;i++) xp[i]=rtk->x[i];
	}

#if defined(PLAYBACK_MODE)
	if (gpz_kf_data->meas_blk&&gpz_kf_data->meas_blk->hpRefQflag[1]>0)
	{
		//gpz_kf_data->meas_blk->hpRefEcef[0] = -2848708.66352;
		//gpz_kf_data->meas_blk->hpRefEcef[1] = 4649106.24341;
		//gpz_kf_data->meas_blk->hpRefEcef[2] = 3298328.46391;
		ag_gnss_zdres(0,obs,nu,rs,dts,svh,nav,gpz_kf_data->meas_blk->hpRefEcef,opt,0,y,e,azel);
		for(i=0;i<3;i++) xp[i]=gpz_kf_data->meas_blk->hpRefEcef[i];
		nv= ag_rtk_ddres(DDRES_HPREF,rtk,nav,dt,xp,sat,y,e,azel,iu,ir,ns,v,NULL,vflg);
		gnss_rtk_kf_ddres_check(DDRES_HPREF,rtk,v,vflg,nv,ns);
		for(i=0;i<3;i++) xp[i]=rtk->x[i];
		float XPreError[9] = { 0 }, XPreErrorPlan[9] = { 0 };
		for (i = 0; i < 6; i++)
		{
			XPreError[i] = (float)(xp[i] - gpz_kf_data->meas_blk->hpRefEcef[i]);
		}
		gnssConvEcef2EnuVel(XPreError, XPreErrorPlan, gpz_kf_data->meas_blk->hpRefLla);
		gnssConvEcef2EnuVel(XPreError + 3, XPreErrorPlan+4, gpz_kf_data->meas_blk->hpRefLla);
		XPreErrorPlan[3] = sqrtf(XPreErrorPlan[0] * XPreErrorPlan[0] + XPreErrorPlan[1] * XPreErrorPlan[1]);
		XPreErrorPlan[7] = sqrtf(XPreErrorPlan[4] * XPreErrorPlan[4] + XPreErrorPlan[5] * XPreErrorPlan[5]);
		GLOGI("RTKXPreError: %8.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f", gpz_kf_data->meas_blk->tor, XPreErrorPlan[3], XPreErrorPlan[7], XPreError[0], XPreError[1], XPreError[2], XPreError[3], XPreError[4], XPreError[5]);
	}
#endif
    /* add 2 iterations for baseline-constraint moving-base */
    niter=opt->niter+(opt->mode==PMODE_MOVEB&&opt->baseline[0]>0.0?2:0);
    
    for (i=0;i<niter;i++) {
        /* undifferenced residuals for rover */
        if (!ag_gnss_zdres(0,obs,nu,rs,dts,svh,nav,xp,opt,0,y,e,azel)) {
			gnss_util_trace(2,"rover initial position error\n");
            stat=SOLQ_NONE;
            break;
        }
        /* get ls fix sol by prefix info */
        if(i==0) {
            gnss_rtk_lsfixsol(rtk, nav, xp, sat, y, e, azel, iu, ir, ns);
            if(g_pe_cfg.sub_chipType==SUB_CHIPTYPE_ST_8100){
                gnss_rtk_prefix_resolve_half(rtk, nav, sat, azel, y, iu, ir, ns, obs, nu, rs, dts, svh);
            }
        }

        /* double-differenced residuals and partial derivatives */
        if ((nv= ag_rtk_ddres(DDRES_PREDICT,rtk,nav,dt,xp,sat,y,e,azel,iu,ir,ns,v,H,vflg))<1) {
            gnss_util_trace(2, "no double-differenced residual\n");
            stat=SOLQ_NONE;
			Sys_Free(rtk->R);
            break;
        }
		/* ddres check for predict x */
		gnss_rtk_kf_ddres_check(DDRES_PREDICT,rtk,v,vflg,nv,ns);
		//if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || SUB_CHIPTYPE_UBLOX_F9== g_pe_cfg.sub_chipType)
		{
			if ((g_proc.flag & 0xFC00) > 0 || ((g_proc.flag & 0x0001) == 0 && (g_proc.flag & g_proc.hold_epoch) == 0) || ((rtkctrl.kfcnt <= 10) && (rtkctrl.kfcnt > 0)))
			{
				/* kalman filter measurement update */
				if ((info = ag_rtk_kf_filter(xp, rtk->P, H, v, rtk->R, rtk->nx, nv))) {
					gnss_util_trace(2, "filter error (info=%d)\n", info);
					stat = SOLQ_NONE;
					Sys_Free(rtk->R);
					break;
				}
			}
		}
        gnss_util_trace(3,"x(%d)=",i+1); gnss_util_tracemat(3,xp,1,NR(opt),13,4);
		Sys_Free(rtk->R);
    }
    if (stat!=SOLQ_NONE&& ag_gnss_zdres(0,obs,nu,rs,dts,svh,nav,xp,opt,0,y,e,azel)) {
        
        /* post-fit residuals for float solution */
        nv= ag_rtk_ddres(DDRES_UPDATE,rtk,nav,dt,xp,sat,y,e,azel,iu,ir,ns,v,NULL,vflg);

		/* ddres check for updated x */
		gnss_rtk_kf_ddres_check(DDRES_UPDATE,rtk,v,vflg,nv,ns);

		/*get float unit power sigma*/
		gnss_rtk_sigma(DDRES_UPDATE,rtk,v,rtk->R,vflg,rtk->cp_pr_cnt,ns);

        /* validation of float solution */
        if (ag_rtk_valpos(rtk,v,rtk->R,vflg,rtk->cp_pr_cnt,4.0)) 
				{
            /* update state and covariance matrix */
			matcpy(rtk->x, xp, rtk->nx, 1);
			rtk->kfcnt++;
            rtk->kfextpcnt=0; //clear extrapolate count
            /* update ambiguity control struct */
            rtk->sol.ns=0;
			for (i = 0; i < ns; i++)
			{
				if (NULL == rtk->ssat[sat[i] - 1])
				{
					continue;
				}
				for (f = 0; f < nf; f++) 
				{
					if (!rtk->ssat[sat[i] - 1]->vsat[f]) continue;
					rtk->ssat[sat[i] - 1]->lock[f]++;
					rtk->ssat[sat[i] - 1]->outc[f] = 0;
					if (f == 0) rtk->sol.ns++; /* valid satellite count by L1 */
				}
			}
            /* lack of valid satellites */
            if (rtk->sol.ns<1) stat=SOLQ_NONE;
            if (/*rtk->sol.ns<5 &&*/ rtk->rtduseflag) stat=SOLQ_DGPS;
        }
        else stat=SOLQ_NONE;
		Sys_Free(rtk->R);
    }
	Sys_Free(xp); Sys_Free(H);
	xa = mat(rtk->nx, 1); bias = mat(rtk->nx, 1);
	rtk->xa = zeros(rtk->na, 1); rtk->Pa = zeros(rtk->na, rtk->na);

	if (g_pe_cfg.meas_rate == MEAS_RATE_1HZ)
	{
		ag_rtk_resamb_WL(rtk, 0, 1, sat, iu, ir, ns, nav, y, e);
	}

    /* resolve integer ambiguity by WL-NL */
    if (stat!=SOLQ_NONE&&rtk->opt.modear==ARMODE_WLNL) {
        
        if (ag_rtk_resamb_WLNL(rtk,obs,sat,iu,ir,ns,nav,azel)) {
            stat=SOLQ_FIX;
        }
    }
    /* resolve integer ambiguity by TCAR */
    else if (stat!=SOLQ_NONE&&rtk->opt.modear==ARMODE_TCAR) {
        
        if (ag_rtk_resamb_TCAR(rtk,obs,sat,iu,ir,ns,nav,azel)) {
            stat=SOLQ_FIX;
        }
    }
    /* resolve integer ambiguity by LAMBDA */
    else if (stat!=SOLQ_NONE&&/*!rtk->rtduseflag&&*/(rtk->nbias=ag_rtk_resamb_LAMBDA(rtk,bias,xa))>1) {
        
        if (ag_gnss_zdres(0,obs,nu,rs,dts,svh,nav,xa,opt,0,y,e,azel)) {
            
            /* post-fit reisiduals for fixed solution */
            nv= ag_rtk_ddres(DDRES_FIXED,rtk,nav,dt,xa,sat,y,e,azel,iu,ir,ns,v,NULL,vflg);

			/* ddres check for fixed x */
			gnss_rtk_kf_ddres_check(DDRES_FIXED,rtk,v,vflg,nv,ns);

			/*get fixed unit power sigma*/
			gnss_rtk_sigma(DDRES_FIXED,rtk,v,rtk->R,vflg,rtk->cp_pr_cnt,ns);
            
            /* validation of fixed solution */
            if (ag_rtk_valpos(rtk,v,rtk->R,vflg,rtk->cp_pr_cnt,4.0)) 
			{
				carrier_num = 0;
				fixed_carrier_num = 0;
				rtk->sol.fixed_sat_num = 0;
				rtk->sol.post_sigma = gnss_rtk_get_sigma(v, rtk->R, rtk->cp_pr_cnt, rtk->sol.ns, vflg, &carrier_num, &(rtk->sol.fixed_sat_num));
				nv_fix = ag_rtk_fixed_phase_ddres(rtk, nav, xa, sat, y, azel, iu, ir, ns, v, vflg);
				rtk->sol.fixed_sat_num = 0;
				rtk->sol.fix_post_sigma = gnss_rtk_get_sigma(v, rtk->R, nv_fix, rtk->sol.ns, vflg, &fixed_carrier_num, &(rtk->sol.fixed_sat_num));
				if (gnss_rtk_fix2float_check(rtk, ns,carrier_num, fixed_carrier_num))
				{
					stat = SOLQ_FLOAT;
					rtk->nfix = 0;
				}
				else
				{
					stat = SOLQ_FIX;
				}
            }
			Sys_Free(rtk->R);
        }
    }

	Sys_Free(e); Sys_Free(v);

#if defined(PLAYBACK_MODE)
	if (gpz_kf_data->meas_blk&&gpz_kf_data->meas_blk->hpRefQflag[1] > 0)
	{
		gnss_rtk_get_refdisinfo(rtk, gpz_kf_data->meas_blk->hpRefEcef, stat);
	}
#endif

	gnss_rtk_getshiftinfo(rtk, &solpvt, stat);

	/* kf reset check */
	gnss_rtk_kf_reset_check(rtk,ns);
	/* solution determine */
	stat=gnss_rtk_sol_determine(rtk,&solpvt,&solpre,stat,ns);

	gnss_rtk_fixstatus_check(rtk, stat);

	if (stat == SOLQ_FIX)
	{
		/* hold integer ambiguity */
		if (gnss_rtk_amb_HoldCheck(rtk))
		{
			ag_rtk_holdamb(rtk, xa, FALSE);
		}
		//if (gnss_rtk_prefix_upd_check(rtk, rtk->nbias))
		{
			gnss_rtk_sdamb_state(rtk, bias);
		}
	}

	gnss_rtk_sol_fill(rtk, &solpvt, stat);

	gnss_rtk_prefix_resolve(rtk, nav, sat, azel, y, iu, ir, ns, obs, nu, rs, dts, svh);

	Sys_Free(xa); Sys_Free(rs); Sys_Free(dts); Sys_Free(y); Sys_Free(azel); Sys_Free(bias); Sys_Free(rtk->xa); Sys_Free(rtk->Pa);

	rtk->shift.last_feedback_flag = rtk->shift.feedback_flag;
	rtk->shift.feedback_flag = FALSE;

	if ((g_proc.flag & 0xFC01) > 0)
	{
		gnss_rtk_feedback2pe(rtk, stat, ns);
	}

	//gnss_rtk_kf_static_check(rtk);

	for (i = 0; i < n; i++) {
		for (j = 0; j < nf; j++) {
			if (obs[i].L[j] == 0.0) {
				continue;
			}

			if (rtk->ssat[obs[i].sat - 1] == NULL)
			{
				continue;
			}
			rtk->ssat[obs[i].sat - 1]->pt[obs[i].rcv - 1][j] = obs[i].time;
#ifdef USED_IN_MC262M
			rtk->ssat[obs[i].sat - 1]->ph[obs[i].rcv - 1][j] = obs[i].L[j];
#endif
		}
	}

    for (i=0;i<MAXSAT;i++) for (j=0;j<nf;j++) {
		if (rtk->ssat[i] == NULL)
		{
			continue;
		}
        if (rtk->ssat[i]->fix[j]==2&&stat!=SOLQ_FIX) rtk->ssat[i]->fix[j]=1;
        if (rtk->ssat[i]->slip[j]&1) rtk->ssat[i]->slipc[j]++;
        if(stat!=SOLQ_FIX) rtk->ssat[i]->usectrl.halfflag=0;
    }

	if ((rtk->kfstatus&RTK_KF_RUN) == 0) gnss_rtk_clean_prefix(rtk);

	gnss_rtk_solpre_save(rtk, stat);

	gnss_rtk_shiftsol_save(rtk, stat);

	gnss_rtk_state_sta(rtk, stat);
	if ((stat == SOLQ_FIX || stat == SOLQ_FLOAT || stat == SOLQ_DGPS)) gnss_rtk_unc_predict(rtk, stat);
	/*rtk confidence*/
	gnss_rtk_factor_out(rtk, stat);

    return stat!=SOLQ_NONE;
}
short checkQsingular(rtk_t* rtk)
{
	short i = 0, index = -1, para_index = -1, is_reset = 0;
	short nx = NX(&(rtk->opt));
	for (i = 0; i < nx; i++)
	{
		index = rtk->ix[i];
		if (index < 0)
		{
			continue;
		}
		para_index = ISM(index, index);
		if (para_index < 0)
		{
			continue;
		}
		if ((rtk->P[para_index]) <= 0.0)
		{
			is_reset = 1;
			break;
		}
	}
	return is_reset;
}
void clearPreFixedAmb(rtk_t* rtk)
{
	short i = 0, j = 0;
	short nf = (short)(rtk->opt.ionoopt == IONOOPT_IFLC ? 1 : rtk->opt.nf);
	for (i = 0; i < MAXSAT; i++)
	{
		if (NULL == rtk->ssat[i])
		{
			continue;
		}
		for (j = 0; j < nf; ++j)
		{
			rtk->ssat[i]->sdamb[j] = 0.0;
		}
	}
}
void cleanRTKinfo(rtk_t* rtk)
{
	prcopt_t* opt = &rtk->opt;
	int i = 0;
	int P_ele_num = NSM(rtk->nx);
	clearPreFixedAmb(rtk);
	gnss_rtk_clean_prefix(rtk);
	for (i = 0; i < NX(opt); ++i) 
	{
		rtk->ix[i] = -1;
	}
	for (i = 0; i < rtk->nx; ++i)
	{
		rtk->x[i] = 0.0;
	}
	for (i = 0; i < P_ele_num; ++i)
	{
		rtk->P[i] = 0.0;
	}
	rtk->kfstatus = RTK_KF_RESET;
	return;
}
static int asensing_rtk_seq_ekf_relpos(rtk_t* rtk, const obsd_t* obs, int nu, int nr,
	const nav_t* nav)
{
	prcopt_t* opt = &rtk->opt;
	gtime_t time = obs[0].time;
	double* rs, * dts, * var, * y, * e, * azel, * v, * H, * xp, * xa, * bias, dt;
	int i, j, f, n = nu + nr, ns, ny, nv = 0, nv_d = 0, nv_fix = 0, sat[MAXSAT], iu[MAXSAT], ir[MAXSAT], niter;
	int info, vflg[MAXOBS * NFREQ * 2 + 1], svh[MAXOBS * 2];
	int stat = rtk->opt.mode <= PMODE_DGPS ? SOLQ_DGPS : SOLQ_FLOAT;
	int nf = opt->ionoopt == IONOOPT_IFLC ? 1 : opt->nf, carrier_num = 0, fixed_carrier_num = 0;
	sol_t solpvt = rtk->sol, solpre = rtk->sol;
	double xyz[3] = { 0.0 }, pseudo_rcv_clk[NSYSDD * NFREQ] = { 0.0 };
	double* Ri = NULL, * Rj = NULL, * Hi = NULL, * R_d = NULL;
	int nb[NFREQ * NSYSDD * 2 + 2] = { 0 }, b = 0, max_obs_limit = CMN_OBS_NUM, max_doppler_num = 0;
	short is_reset = 0;
	gnss_util_trace(3, "ag_rtk_relpos  : nx=%d nu=%d nr=%d\n", rtk->nx, nu, nr);
	//gnss_rtk_unc_batch_test();
	/* time update for pva, amb */
	rtk->dcb_para_init = 0;
	is_reset = checkQsingular(rtk);
	if (1 == is_reset)
	{
		GLOGW("reset the parameter cause by checkQsingular!");
		cleanRTKinfo(rtk);
	}
	ag_rtk_kf_udpos(rtk, rtk->tt);
	ag_rtk_getprepos(rtk);
	/* save predict sol */
	gnss_rtk_save_floatsol(rtk, &solpre);

	if (gnss_rtk_extpol_check(rtk, &solpre)) {
		rtk->kfextpcnt++;
		GLOGW("ag_rtk_relpos: time update only! extcnt=%d", rtk->kfextpcnt);
		if (rtk->kfextpcnt >= 5)
		{
			rtk->kfstatus = RTK_KF_RESET;
		}
		return 1;
	}
	gnss_rtk_cfd_init(rtk);

	dt = timediff(time, obs[nu].time);

	rs = mat(6, n);
	dts = mat(2, n);
	var = mat(1, n);
	y = mat(nf * 2, n);
	e = mat(3, n);
	azel = zeros(2, n);

	for (i = 0; i < MAXSAT; i++) {
		if (rtk->ssat[i] == NULL) continue;
		rtk->ssat[i]->sys = satsys(i + 1, NULL);
		for (j = 0; j < NFREQ; j++)
		{
			rtk->ssat[i]->vsat[j] = 0;
			rtk->ssat[i]->ambfix[j] = 0;
			rtk->ssat[i]->tdy[j] = 0.0;
			rtk->ssat[i]->tdres[j] = 0.0;
		}
	}
	for (i = 0; i < MAX_DDRES_NUM; i++)
	{
		rtk->dd_res[i].flag = 0;
		rtk->dd_res[i].rejc = rtk->dd_res[i].rejp = 0;
	}
	rtk->dewNum_cr = 0;
	rtk->dewNum_pr = 0;

	/* satellite positions/clocks */
	ag_gnss_satposs(time, obs, n, nav, EPHOPT_BRDC_TOE, rs, dts, var, svh, azel);
	Sys_Free(var);

	/* undifferenced residuals for base station */
	if (!ag_gnss_zdres(1, obs + nu, nr, rs + nu * 6, dts + nu * 2, svh + nu, nav, rtk->rb, opt, 1,
		y + nu * nf * 2, e + nu * 3, azel + nu * 2)) {
		GLOGI("base station position error: zero value");

		Sys_Free(rs); Sys_Free(dts); Sys_Free(y); Sys_Free(e); Sys_Free(azel);
		return 0;
	}
	/* undifferenced residuals for rover station using pvt pos */
	if (solpvt.stat == SOLQ_SINGLE) for (i = 0; i < 3; i++) xyz[i] = solpvt.rr[i];
	else for (i = 0; i < 3; i++) xyz[i] = rtk->x[i];
	ag_gnss_zdres(0, obs, nu, rs, dts, svh, nav, xyz, opt, 0, y, e, azel);
	/* time-interpolation of residuals (for post-processing) */
	if (opt->intpref) {
		dt = ag_gnss_post_intpres(time, obs + nu, nr, nav, rtk, y + nu * nf * 2);
	}
	/* select common satellites between rover and base-station */
	if ((ns = ag_gnss_selsat(rtk, obs, azel, nu, nr, opt, sat, iu, ir)) <= 0) {
		GLOGW("select common satellite fail");

		Sys_Free(rs); Sys_Free(dts); Sys_Free(y); Sys_Free(e); Sys_Free(azel);
		return 0;
	}
#ifdef USED_IN_MC262M
	if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9) gnss_rtk_ubx_lockv(rtk, sat, nf, ns);
#endif
	/* ref detect */
	gnss_rtk_slpdet_ref(rtk, nav, nu, nr, y, e, azel);
	/* td slip detect */
	gnss_rtk_slip_detect_td(rtk, nav, obs, nu, sat, iu, ir, ns, y, e, azel, n, xyz);

	gnss_rtk_ambp_reset_check(rtk, ns, sat);
	/* temporal update of states */

	ag_rtk_udstate(rtk, obs, sat, iu, ir, ns, nav);

	/* update previous sd amb fix status */
	gnss_rtk_sdstate_upd(rtk);

	gnss_util_trace(3, "x(0)="); gnss_util_tracemat(3, rtk->x, 1, NR(opt), 13, 4);

	xp = mat(rtk->nx, 1); matcpy(xp, rtk->x, rtk->nx, 1);
	asensing_rtk_rcv_clock(rtk, nav, xp, sat, y, iu, ir, ns, pseudo_rcv_clk);
	asensing_rtk_kf_update_rcv_clk(rtk, rtk->tt, pseudo_rcv_clk);
	matcpy(xp, rtk->x, rtk->nx, 1);

	if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		ny = CMN_OBS_NUM_HF;
	}
	else
	{
		ny = CMN_OBS_NUM;
	}
	v = mat(ny, 1);

	/* ddres check for single sol */
	if (opt->dynamics && solpvt.stat == SOLQ_SINGLE)
	{
		for (i = 0; i < 3; i++) xp[i] = solpvt.rr[i];
		nv = ag_rtk_ddres(DDRES_SINGLE, rtk, nav, dt, xp, sat, y, e, azel, iu, ir, ns, v, NULL, vflg);
		gnss_rtk_kf_ddres_check(DDRES_SINGLE, rtk, v, vflg, nv, ns);
		for (i = 0; i < 3; i++) xp[i] = rtk->x[i];
	}

#if defined(PLAYBACK_MODE)
	if (gpz_kf_data->meas_blk && gpz_kf_data->meas_blk->hpRefQflag[1] > 0)
	{
		//gpz_kf_data->meas_blk->hpRefEcef[0] = -2848708.66352;
		//gpz_kf_data->meas_blk->hpRefEcef[1] = 4649106.24341;
		//gpz_kf_data->meas_blk->hpRefEcef[2] = 3298328.46391;
		ag_gnss_zdres(0, obs, nu, rs, dts, svh, nav, gpz_kf_data->meas_blk->hpRefEcef, opt, 0, y, e, azel);
		for (i = 0; i < 3; i++) xp[i] = gpz_kf_data->meas_blk->hpRefEcef[i];
		nv = ag_rtk_ddres(DDRES_HPREF, rtk, nav, dt, xp, sat, y, e, azel, iu, ir, ns, v, NULL, vflg);
		gnss_rtk_kf_ddres_check(DDRES_HPREF, rtk, v, vflg, nv, ns);
		for (i = 0; i < 3; i++) xp[i] = rtk->x[i];
		float XPreError[9] = { 0 }, XPreErrorPlan[9] = { 0 };
		for (i = 0; i < 6; i++)
		{
			XPreError[i] = (float)(xp[i] - gpz_kf_data->meas_blk->hpRefEcef[i]);
		}
		gnssConvEcef2EnuVel(XPreError, XPreErrorPlan, gpz_kf_data->meas_blk->hpRefLla);
		gnssConvEcef2EnuVel(XPreError + 3, XPreErrorPlan + 4, gpz_kf_data->meas_blk->hpRefLla);
		XPreErrorPlan[3] = sqrtf(XPreErrorPlan[0] * XPreErrorPlan[0] + XPreErrorPlan[1] * XPreErrorPlan[1]);
		XPreErrorPlan[7] = sqrtf(XPreErrorPlan[4] * XPreErrorPlan[4] + XPreErrorPlan[5] * XPreErrorPlan[5]);
		GLOGI("RTKXPreError: %8.3f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f", gpz_kf_data->meas_blk->tor, XPreErrorPlan[3], XPreErrorPlan[7], XPreError[0], XPreError[1], XPreError[2], XPreError[3], XPreError[4], XPreError[5]);
	}
#endif
	/* add 2 iterations for baseline-constraint moving-base */
	niter = opt->niter + (opt->mode == PMODE_MOVEB && opt->baseline[0] > 0.0 ? 2 : 0);

	for (i = 0; i < niter; i++) {
		/* undifferenced residuals for rover */
		if (!ag_gnss_zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel)) {
			gnss_util_trace(2, "rover initial position error\n");
			stat = SOLQ_NONE;
			break;
		}
		/* get ls fix sol by prefix info */
		if (i == 0) {
			gnss_rtk_lsfixsol(rtk, nav, xp, sat, y, e, azel, iu, ir, ns);
			if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100) {
				gnss_rtk_prefix_resolve_half(rtk, nav, sat, azel, y, iu, ir, ns, obs, nu, rs, dts, svh);
			}
		}

		/* double-differenced residuals and partial derivatives */
		if ((nv = ag_rtk_ddres(DDRES_PREDICT, rtk, nav, dt, xp, sat, y, e, azel, iu, ir, ns, v, NULL, vflg)) < 1) {
			gnss_util_trace(2, "no double-differenced residual\n");
			stat = SOLQ_NONE;
			Sys_Free(rtk->R);
			break;
		}
		/* ddres check for predict x */
		gnss_rtk_kf_ddres_check(DDRES_PREDICT, rtk, v, vflg, nv, ns);
		matcpy(xp, rtk->x, rtk->nx, 1);
		asensing_rtk_rcv_clock(rtk, nav, xp, sat, y, iu, ir, ns, pseudo_rcv_clk);
		asensing_rtk_kf_update_rcv_clk(rtk, rtk->tt, pseudo_rcv_clk);
		nv = asensing_rtk_sd_obs_meas_update(rtk, nav, dt, xp, sat, y, e, azel, iu, ir, ns, &max_obs_limit);
		matcpy(xp, rtk->x, rtk->nx, 1);
		max_doppler_num = max_obs_limit - nv;
		if (max_doppler_num > 20)
		{
			max_doppler_num = 20;
		}
		if (rtk->rtduseflag && max_doppler_num>1)//using doppler observations to update filter
		{
			H = zeros(rtk->nx, max_doppler_num);
			Hi = H;
			if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
			{
				Ri = mat(CMN_OBS_NUM_HF, 1); Rj = mat(CMN_OBS_NUM_HF, 1);
			}
			else
			{
				Ri = mat(CMN_OBS_NUM, 1); Rj = mat(CMN_OBS_NUM, 1);
			}
			gnss_rtk_dr_sd_res(rtk, v, &nv_d, H, Hi, Ri, Rj, &nb[0], &b, ns, nf, &max_doppler_num);
			if (nv_d > 0)
			{
				R_d = mat(nv_d, nv_d);
				/* double-differenced measurement error covariance */
				if (R_d) ddcov(nb, b, Ri, Rj, nv_d, R_d);
				if ((info = ag_rtk_kf_filter(xp, rtk->P, H, v, R_d, rtk->nx, nv_d)))
				{
				}
				Sys_Free(R_d);
			}
			Sys_Free(Ri); Sys_Free(Rj); Sys_Free(H);
		}
		gnss_util_trace(3, "x(%d)=", i + 1); gnss_util_tracemat(3, xp, 1, NR(opt), 13, 4);
		Sys_Free(rtk->R);
	}
	if (stat != SOLQ_NONE && ag_gnss_zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel)) {

		/* post-fit residuals for float solution */
		nv = ag_rtk_ddres(DDRES_UPDATE, rtk, nav, dt, xp, sat, y, e, azel, iu, ir, ns, v, NULL, vflg);

		/* ddres check for updated x */
		gnss_rtk_kf_ddres_check(DDRES_UPDATE, rtk, v, vflg, nv, ns);

		/*get float unit power sigma*/
		gnss_rtk_sigma(DDRES_UPDATE, rtk, v, rtk->R, vflg, rtk->cp_pr_cnt, ns);

		/* validation of float solution */
		if (ag_rtk_valpos(rtk, v, rtk->R, vflg, rtk->cp_pr_cnt, 4.0))
		{

			/* update state and covariance matrix */
			matcpy(rtk->x, xp, rtk->nx, 1);
			rtk->kfcnt++;
			rtk->kfextpcnt = 0; //clear extrapolate count
			/* update ambiguity control struct */
			rtk->sol.ns = 0;
			for (i = 0; i < ns; i++)
			{
				if (NULL == rtk->ssat[sat[i] - 1])
				{
					continue;
				}
				for (f = 0; f < nf; f++)
				{
					if (!rtk->ssat[sat[i] - 1]->vsat[f]) continue;
					rtk->ssat[sat[i] - 1]->lock[f]++;
					rtk->ssat[sat[i] - 1]->outc[f] = 0;
					if (f == 0) rtk->sol.ns++; /* valid satellite count by L1 */
				}
			}
			/* lack of valid satellites */
			if (rtk->sol.ns < 1) stat = SOLQ_NONE;
			if (/*rtk->sol.ns<5 &&*/ rtk->rtduseflag) stat = SOLQ_DGPS;
		}
		else stat = SOLQ_NONE;
		Sys_Free(rtk->R);
	}
	Sys_Free(xp);
	xa = mat(rtk->nx, 1); bias = mat(rtk->nx, 1);
	rtk->xa = zeros(rtk->na, 1); rtk->Pa = zeros(rtk->na, rtk->na);

	if (g_pe_cfg.meas_rate == MEAS_RATE_1HZ)
	{
		ag_rtk_resamb_WL(rtk, 0, 1, sat, iu, ir, ns, nav, y, e);
	}

	/* resolve integer ambiguity by WL-NL */
	if (stat != SOLQ_NONE && rtk->opt.modear == ARMODE_WLNL) {

		if (ag_rtk_resamb_WLNL(rtk, obs, sat, iu, ir, ns, nav, azel)) {
			stat = SOLQ_FIX;
		}
	}
	/* resolve integer ambiguity by TCAR */
	else if (stat != SOLQ_NONE && rtk->opt.modear == ARMODE_TCAR) {

		if (ag_rtk_resamb_TCAR(rtk, obs, sat, iu, ir, ns, nav, azel)) {
			stat = SOLQ_FIX;
		}
	}
	/* resolve integer ambiguity by LAMBDA */
	else if (stat != SOLQ_NONE &&/*!rtk->rtduseflag&&*/(rtk->nbias = ag_rtk_resamb_LAMBDA(rtk, bias, xa)) > 1) {

		if (ag_gnss_zdres(0, obs, nu, rs, dts, svh, nav, xa, opt, 0, y, e, azel)) {

			/* post-fit reisiduals for fixed solution */
			nv = ag_rtk_ddres(DDRES_FIXED, rtk, nav, dt, xa, sat, y, e, azel, iu, ir, ns, v, NULL, vflg);

			/* ddres check for fixed x */
			gnss_rtk_kf_ddres_check(DDRES_FIXED, rtk, v, vflg, nv, ns);

			/*get fixed unit power sigma*/
			gnss_rtk_sigma(DDRES_FIXED, rtk, v, rtk->R, vflg, rtk->cp_pr_cnt, ns);

			/* validation of fixed solution */
			if (ag_rtk_valpos(rtk, v, rtk->R, vflg, rtk->cp_pr_cnt, 4.0))
			{
				carrier_num = 0;
				fixed_carrier_num = 0;
				rtk->sol.fixed_sat_num = 0;
				rtk->sol.post_sigma = gnss_rtk_get_sigma(v, rtk->R, rtk->cp_pr_cnt, rtk->sol.ns, vflg, &carrier_num, &(rtk->sol.fixed_sat_num));
				nv_fix = ag_rtk_fixed_phase_ddres(rtk, nav, xa, sat, y, azel, iu, ir, ns, v, vflg);
				rtk->sol.fixed_sat_num = 0;
				rtk->sol.fix_post_sigma = gnss_rtk_get_sigma(v, rtk->R, nv_fix, rtk->sol.ns, vflg, &fixed_carrier_num, &(rtk->sol.fixed_sat_num));
				if (gnss_rtk_fix2float_check(rtk, ns, carrier_num, fixed_carrier_num))
				{
					stat = SOLQ_FLOAT;
					rtk->nfix = 0;
				}
				else
				{
					stat = SOLQ_FIX;
				}
			}
			Sys_Free(rtk->R);
		}
	}

	Sys_Free(e); Sys_Free(v);

#if defined(PLAYBACK_MODE)
	if (gpz_kf_data->meas_blk && gpz_kf_data->meas_blk->hpRefQflag[1] > 0)
	{
		gnss_rtk_get_refdisinfo(rtk, gpz_kf_data->meas_blk->hpRefEcef, stat);
	}
#endif

	gnss_rtk_getshiftinfo(rtk, &solpvt, stat);

	/* kf reset check */
	gnss_rtk_kf_reset_check(rtk, ns);
	/* solution determine */
	stat = gnss_rtk_sol_determine(rtk, &solpvt, &solpre, stat, ns);

	gnss_rtk_fixstatus_check(rtk, stat);

	if (stat == SOLQ_FIX)
	{
		/* hold integer ambiguity */
		if (gnss_rtk_amb_HoldCheck(rtk))
		{
			ag_rtk_holdamb(rtk, xa, FALSE);
		}
		//if (gnss_rtk_prefix_upd_check(rtk, rtk->nbias))
		{
			gnss_rtk_sdamb_state(rtk, bias);
		}
	}

	gnss_rtk_sol_fill(rtk, &solpvt, stat);

	gnss_rtk_prefix_resolve(rtk, nav, sat, azel, y, iu, ir, ns, obs, nu, rs, dts, svh);

	Sys_Free(xa); Sys_Free(rs); Sys_Free(dts); Sys_Free(y); Sys_Free(azel); Sys_Free(bias); Sys_Free(rtk->xa); Sys_Free(rtk->Pa);

	rtk->shift.last_feedback_flag = rtk->shift.feedback_flag;
	rtk->shift.feedback_flag = FALSE;

	if ((g_proc.flag & 0xFC01) > 0)
	{
		gnss_rtk_feedback2pe(rtk, stat, ns);
	}

	//gnss_rtk_kf_static_check(rtk);

	for (i = 0; i < n; i++) {
		for (j = 0; j < nf; j++) {
			if (obs[i].L[j] == 0.0) {
				continue;
			}

			if (rtk->ssat[obs[i].sat - 1] == NULL)
			{
				continue;
			}
			rtk->ssat[obs[i].sat - 1]->pt[obs[i].rcv - 1][j] = obs[i].time;
#ifdef USED_IN_MC262M
			rtk->ssat[obs[i].sat - 1]->ph[obs[i].rcv - 1][j] = obs[i].L[j];
#endif
		}
	}

	for (i = 0; i < MAXSAT; i++)
	{
		if (rtk->ssat[i] == NULL)
		{
			continue;
		}
		for (j = 0; j < nf; j++) 
		{
			if (rtk->ssat[i]->fix[j] == 2 && stat != SOLQ_FIX) rtk->ssat[i]->fix[j] = 1;
			if (rtk->ssat[i]->slip[j] & 1) rtk->ssat[i]->slipc[j]++;
			if (stat != SOLQ_FIX) rtk->ssat[i]->usectrl.halfflag = 0;
		}
	}

	if ((rtk->kfstatus & RTK_KF_RUN) == 0) gnss_rtk_clean_prefix(rtk);

	gnss_rtk_solpre_save(rtk, stat);

	gnss_rtk_shiftsol_save(rtk, stat);

	gnss_rtk_state_sta(rtk, stat);
	if ((stat == SOLQ_FIX || stat == SOLQ_FLOAT || stat == SOLQ_DGPS)) gnss_rtk_unc_predict(rtk, stat);
	/*rtk confidence*/
	gnss_rtk_factor_out(rtk, stat);

	return stat != SOLQ_NONE;
}
/* initialize rtk control ------------------------------------------------------
* initialize rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
*          prcopt_t *opt    I   positioning options (see rtklib.h)
* return : none
*-----------------------------------------------------------------------------*/
extern void ag_rtk_init(rtk_t *rtk, const prcopt_t *opt)
{
    //sol_t sol0={{0}};
    //ssat_t ssat0={0};
    int i;
    gtime_t time0={0};
	ddres_t res={0};
	wl_t wl;
    
    gnss_util_trace(3,"ag_rtk_init :\n");

    memset(&wl, 0, sizeof(wl_t));
    //rtk->sol=sol0;
	rtk->wl = wl;
    memset(&(rtk->sol), 0, sizeof(sol_t));
    for (i=0;i<6;i++) {
    	rtk->rb[i]=0.0;
    }
	if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
	{
		rtk->nx = opt->mode <= PMODE_FIXED ? (NX_PVA + NC(opt) + CMN_CP_NUM_HF) : 0;//pppnx(opt);
	}
	else
	{
		rtk->nx = opt->mode <= PMODE_FIXED ? (NX_PVA + NC(opt) + CMN_CP_NUM) : 0;//pppnx(opt);
	}
    rtk->na=opt->mode<=PMODE_FIXED?(NR(opt)+NC(opt)):0;//pppnx(opt);
    rtk->tt=0.0;
	rtk->ix = (int*)Sys_Calloc(NX(opt), sizeof(int));
	for (i = 0; i < NX(opt); i++) {
		rtk->ix[i] = -1;
	}
    rtk->x=zeros(rtk->nx,1);
    rtk->P=zeros(1, NSM(rtk->nx));
    rtk->nfix=0;
    /*for (i=0;i<MAXSAT;i++) {
        rtk->ssat[i]=ssat0;
    }*/
    
    rtk->opt=*opt;

	rtk->rbflag=0;
	for (i=0;i<3;i++)
	{
		rtk->rbspp[i]=0.0;
		rtk->prr[i]=0.0;
	}
	for (i=0;i<4;i++)
	{
		rtk->llisatcnt[i]=0;
		rtk->satcnt[i]=0;
	}
	for (i = 0;i < 2;i++)
	{
		rtk->fix_amb_L1[i] = 0;
	}
	rtk->noSkipCnt = 0;
	rtk->trstCloCnt = 0;
	rtk->prefloat = FALSE;
	for (i = 0; i < NSYSDD; i++) rtk->refs[i] = 0;
	rtk->pt[0]=rtk->pt[1]=time0;
	rtk->nsat=0;
	rtk->tdsigma=0.0;
	rtk->ptor=0.0;
	rtk->rtkevent=0;
	rtk->kfstatus=RTK_KF_IDLE;
	rtk->kfextpol=0;
	rtk->kfextpcnt=0;
	rtk->kfcnt=0;
	rtk->qcpasscnt=0;
	rtk->nollicnt=0;
	rtk->dd_res[0]=rtk->dd_res[1]=rtk->dd_res[2]=rtk->dd_res[3]=res;
	rtk->shift.Qfact=1.0;
	rtk->shift.maxadjQcnt=0;
	rtk->shift.adjQcnt=0;
	rtk->shift.adjPcnt=0;
	rtk->updFQtt = 0.0;
	rtk->Qratio = 1.0;
	static_flag = 1;  //global variable, only use for static mode or first static period of dynamic mode 
}
/* free rtk control ------------------------------------------------------------
* free memory for rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
* return : none
*-----------------------------------------------------------------------------*/
extern void ag_rtk_free(rtk_t *rtk)
{
    gnss_util_trace(3,"ag_rtk_free :\n");
    
    rtk->nx=rtk->na=0;
		rtk->updFQtt = 0.0;
	if (rtk->ix) Sys_Free(rtk->ix);
	if (rtk->x)  Sys_Free(rtk->x);
    if (rtk->P)  Sys_Free(rtk->P);
}
/* precise positioning ---------------------------------------------------------
* input observation data and navigation message, compute rover position by 
* precise positioning
* args   : rtk_t *rtk       IO  rtk control/result struct
*            rtk->sol       IO  solution
*                .time      O   solution time
*                .rr[]      IO  rover position/velocity
*                               (I:fixed mode,O:single mode)
*                .dtr[0]    O   receiver clock bias (s)
*                .dtr[1]    O   receiver glonass-gps time offset (s)
*                .Qr[]      O   rover position covarinace
*                .stat      O   solution status (SOLQ_???)
*                .ns        O   number of valid satellites
*                .age       O   age of differential (s)
*                .ratio     O   ratio factor for ambiguity validation
*            rtk->rb[]      IO  base station position/velocity
*                               (I:relative mode,O:moving-base mode)
*            rtk->nx        I   number of all states
*            rtk->na        I   number of integer states
*            rtk->ns        O   number of valid satellite
*            rtk->tt        O   time difference between current and previous (s)
*            rtk->x[]       IO  float states pre-filter and post-filter
*            rtk->P[]       IO  float covariance pre-filter and post-filter
*            rtk->xa[]      O   fixed states after AR
*            rtk->Pa[]      O   fixed covariance after AR
*            rtk->ssat[s]   IO  sat(s+1) status
*                .sys       O   system (SYS_???)
*                .az   [r]  O   azimuth angle   (rad) (r=0:rover,1:base)
*                .el   [r]  O   elevation angle (rad) (r=0:rover,1:base)
*                .vs   [r]  O   data valid single     (r=0:rover,1:base)
*                .resp [f]  O   freq(f+1) pseudorange residual (m)
*                .resc [f]  O   freq(f+1) carrier-phase residual (m)
*                .vsat [f]  O   freq(f+1) data vaild (0:invalid,1:valid)
*                .fix  [f]  O   freq(f+1) ambiguity flag
*                               (0:nodata,1:float,2:fix,3:hold)
*                .slip [f]  O   freq(f+1) slip flag
*                               (bit8-7:rcv1 LLI, bit6-5:rcv2 LLI,
*                                bit2:parity unknown, bit1:slip)
*                .lock [f]  IO  freq(f+1) carrier lock count
*                .outc [f]  IO  freq(f+1) carrier outage count
*                .slipc[f]  IO  freq(f+1) cycle slip count
*                .rejc [f]  IO  freq(f+1) data reject count
*                .gf        IO  geometry-free phase (L1-L2) (m)
*                .gf2       IO  geometry-free phase (L1-L5) (m)
*            rtk->nfix      IO  number of continuous fixes of ambiguity
*            rtk->tstr      O   time string for debug
*            rtk->opt       I   processing options
*          obsd_t *obs      I   observation data for an epoch
*                               obs[i].rcv=1:rover,2:reference
*                               sorted by receiver and satellte
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation messages
* return : status (0:no solution,1:valid solution)
* notes  : before calling function, base station position rtk->sol.rb[] should
*          be properly set for relative mode except for moving-baseline
*-----------------------------------------------------------------------------*/
#define RTK_CHECK_EXPOL do{if (rtk->kfstatus&RTK_KF_RUN) {rtk->kfextpol=1; goto lab_rtkpos;} else return 0;}while(0)
extern int ag_rtk_position(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;
    sol_t solb={{0}};
    int i,nu,nr;
    char msg[128]="";
	char utct[64]={0};
    
    /* set base staion position */
    if (opt->refpos<=POSOPT_RINEX&&opt->mode!=PMODE_SINGLE&&
        opt->mode!=PMODE_MOVEB) {
        for (i=0;i<6;i++) rtk->rb[i]=i<3?opt->rb[i]:0.0;
    }
    /* count rover/base station observations */
		for (nu = 0; nu < n && obs[nu].rcv == 1;)
		{
			nu++;
		}
		for (nr = 0; nu + nr < n && obs[nu + nr].rcv == 2;)
		{
			nr++;
		}

	if (nu==0)
	{
		rtk->rtkevent|=RTK_NOROVROBS|RTK_NOROVRSPP;
		rtk->sol.stat=SOLQ_NONE;
		GLOGI("no rov obs, kfstatus=%u", rtk->kfstatus);
		return 0;
	}
	else
	{
		time2str(gpst2utc(obs[0].time), utct, 3);
		gnss_util_trace(3, "ag_rtk_position: tor=%.6f GPST=%.4f UTC=%s n=%d nu=%d nr=%d tt=%.6f\n", rtk->ptor, time2gpst(obs[0].time, NULL), utct, n, nu, nr, rtk->tt);
		SYS_LOGGING(OBJ_ALGO, LOG_INFO, "tor=%.6f GPST=%.4f UTC=%s n=%d nu=%d nr=%d tt=%.6f", rtk->ptor, time2gpst(obs[0].time, NULL), utct, n, nu, nr, rtk->tt);
	}
	gnss_util_trace(4,"obs=\n"); gnss_util_traceobs(4,obs,n);

	if (!gnss_rtk_get_init_rr(&rtk->sol,obs[0].time))
	{
		GLOGI("get init rr from pvt failed, kfstatus=%d",rtk->kfstatus);
		rtk->rtkevent|=RTK_NOROVRSPP;
		rtk->rtduseflag = 0;
		if (peMode.tunnelMode.mode == TRUE || nu < 8 || !(gpz_kf_data->lastKfStatus & KF_RUN)) return 0;
		RTK_CHECK_EXPOL;
	}
    
    /* check number of data of base station and age of differential */
    if (nr==0) {
        GLOGI("no base station observation data for rtk, kfstatus=%d",rtk->kfstatus);
        rtk->rtkevent|=RTK_NOBASEOBS;
        RTK_CHECK_EXPOL;
    }
    if (opt->mode==PMODE_MOVEB) { /*  moving baseline */
        
        /* estimate position/velocity of base station */
        if (!ag_gnss_pntpos(obs+nu,nr,nav,&rtk->opt,&solb,NULL,NULL,msg)) {
            GLOGI("base station SPP error (%s)",msg);
            return 0;
        }
        rtk->sol.age=(float)timediff(rtk->sol.time,solb.time);
        
        if (fabs(rtk->sol.age)>TTOL_MOVEB) {
            GLOGI("time sync error for moving-base (age=%.1f)",rtk->sol.age);
            return 0;
        }
        for (i=0;i<6;i++) rtk->rb[i]=solb.rr[i];
        
        /* time-synchronized position of base station */
        for (i=0;i<3;i++) rtk->rb[i]+=rtk->rb[i+3]*rtk->sol.age;
    }
    else {
        rtk->sol.age=(float)timediff(obs[0].time,obs[nu].time);
        
        if (fabs(rtk->sol.age)>opt->maxtdiff) {
			gnss_rtk_basedata_clean();
            GLOGW("age of differential error (age=%.1f), kfstatus=%d",rtk->sol.age,rtk->kfstatus);
            rtk->rtkevent|=RTK_DIFAGEEXPIRE;
            RTK_CHECK_EXPOL;
        }
    }

	/* check rb */
	if (!rtk->rbflag)
	{
		GLOGW("no valid base pos, kfstatus=%d",rtk->kfstatus);
		rtk->rtkevent|=RTK_NOBASEPOS;
		RTK_CHECK_EXPOL;
	}
	//TODO: what do when base station changed normally
	if (rtk->rtkevent&RTK_BASECHANGE)
	{
		GLOGW("rb changed");
		gnss_rtk_rb_changed(rtk);
	}

lab_rtkpos:
	if (!gnss_rtk_opensky_check(rtk))  {
		return 0;
	}
	//if (!gnss_rtk_kf_start_check(rtk))
	//{
	//	GLOGI("bad time to start kf!");
	//	return 0;
	//}
    
    /* relative potitioning */
	if (0 == rtk->opt.adaptSeqEKF)
	{
		ag_rtk_relpos(rtk, obs, nu, nr, nav);
	}
	else
	{
		asensing_rtk_seq_ekf_relpos(rtk, obs, nu, nr, nav);
	}
    
    return 1;
}

/* ddres eliminating fixed dd bias */
extern int gnss_rtk_fddres(rtk_t *rtk,const nav_t *nav,const double *x,const int *sat,double *y,double *e,
	double *azel,const int *iu,const int *ir,int ns,double *v,double *H,int *satuse)
{
	prcopt_t *opt=&rtk->opt;
	//double /*bl,dr[3],*/dt=rtk->sol.age;
	double lami,lamj,*Hi=NULL;
	int i,j,k,m,f,ff,nv=0,nb[NFREQ*NSYSDD*2+2]={ 0 },b=0,sysi,sysj,nf=NF(opt);
	uint8_t satmask[MAXSAT] = { 0 }, satcnt = 0;
	char prefixsat[1024] = { 0 }, *pstr = &prefixsat[0];

	//gnss_util_trace(3,"fddres   : dt=%.1f nx=%d ns=%d\n",dt,rtk->nx,ns);

	for (m=0;m<NSYSDD;m++)
	{
		if (rtk->biasnum[m]<2)
			continue;
		for (f=opt->mode>PMODE_DGPS ? 0 : nf;f<nf*2;f++) {

			if ((nf<=f) || (nv>=MAXSAT)) continue;

			ff=f%nf;
			/* search reference satellite with highest elevation */
			for (i=-1,j=0;j<ns;j++) {
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				sysi=rtk->ssat[sat[j]-1]->sys;
				if (!(rtk->fixstate[sat[j]-1]&FREQ_FIX(ff))) continue;
				if (!test_sys(sysi,m)) continue;
				if (!validobs(iu[j],ir[j],f,nf,y,0)) continue;
				//if (rtk->ssat[sat[j]-1].fix[ff]<2) continue;
				if (i < 0 || (azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2] &&
					(rtk->ssat[sat[j] - 1]->snr[f] > 35 * 4 || rtk->ssat[sat[j] - 1]->snr[f] > rtk->ssat[sat[i] - 1]->snr[f]))) i = j;
			}
			if (i<0) continue;
			if (f < nf) GLOGI("prefix sol f=%d refsat=%d", f, sat[i]);

			/* make double difference */
			for (j=0;j<ns;j++) {
				if (i==j) continue;
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				//sysi=rtk->ssat[sat[i]-1]->sys;
				sysj=rtk->ssat[sat[j]-1]->sys;
				if (!test_sys(sysj,m)) continue;
				if (!validobs(iu[j],ir[j],f,nf,y,0)) continue;
				if (!(rtk->fixstate[sat[j]-1]&FREQ_FIX(ff))) continue;

				if (f < nf) pstr += sprintf(pstr, " %d-%d-%d", f, sat[i], sat[j]);
				//ff=f%nf;
				lami=nav->lam[sat[i]-1][ff];
				lamj=nav->lam[sat[j]-1][ff];
				if (lami<=0.0||lamj<=0.0) continue;
				if (H) Hi=H+nv*3;

				/* double-differenced residual */
				v[nv]=(y[f+iu[i]*nf*2]-y[f+ir[i]*nf*2])-
					(y[f+iu[j]*nf*2]-y[f+ir[j]*nf*2]);

				/* eliminate fixed ddamb*/
				v[nv]-=rtk->ssat[sat[i]-1]->sdamb[ff]*lami-rtk->ssat[sat[j]-1]->sdamb[ff]*lamj;

				/* partial derivatives by rover position */
				if (H) {
					for (k=0;k<3;k++) {
						Hi[k]=-e[k+iu[i]*3]+e[k+iu[j]*3];
					}
				}

				/* test innovation */
				if (opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno) {
					gnss_util_trace(2,"outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
						sat[i],sat[j],f<nf ? "L" : "P",f%nf+1,v[nv]);
					continue;
				}
				GLOGI("fsat=%3d-%3d %s%d v=%13.3f ddamb=%8.3f",sat[i],sat[j],f<nf ? "L" : "P",f%nf+1,v[nv],
					rtk->ssat[sat[i]-1]->sdamb[ff]-rtk->ssat[sat[j]-1]->sdamb[ff]);
				nv++;
				satmask[sat[j] - 1] = 1;
				nb[b]++;
			}
			
			b++;
		}
		/* end of system loop */
	}

	*pstr = '\0';
	GLOGI("prefix sol f-refsat-rovsat:%s", prefixsat);

	for (i = 0;i < MAXSAT;i++) { if (satmask[i] == 1) satcnt++; }
	*satuse = satcnt;

	return nv;
}

extern void gnss_rtk_solpre_save(rtk_t *rtk, int stat)
{
	int i; // /*m, f, info,*/ nf = NF(&rtk->opt);

	if (stat != SOLQ_FIX && rtk->prefixf)
	{
		for (i = 0; i < 3; i++)
		{
			rtk->sol.rr[i] = rtk->prefixsol[i];
		}
		for (i = 0; i < 6; i++) rtk->sol.qr[i] = rtk->prefixQmat[i];
#if defined(PLAYBACK_MODE) && defined(_MSC_VER)
		rtk->sol.stat = (g_pe_cfg.ppk_mode != 0) ? SOLQ_DGPS : ((rtk->prefloat == FALSE) ? SOLQ_FIX : SOLQ_FLOAT);
#else
		rtk->sol.stat = ((rtk->prefloat == FALSE) ? SOLQ_FIX : SOLQ_FLOAT);
#endif
		GLOGI("use prefix or prefloat sol, %d", rtk->prefloat);

		if (rtk->prefloat == FALSE && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 && (g_proc.flag & g_proc.hold_epoch) != 0)
		{
			/*should amb hold check*/
			ag_rtk_holdamb(rtk, NULL, TRUE);
		}
	}
}

extern void gnss_rtk_shiftsol_save(rtk_t *rtk, int stat)
{
	uint32_t i = 0;

	for (i = 0; i < 3; i++) rtk->shift.prexyz[i] = rtk->sol.rr[i];
}

extern void gnss_rtk_clean_prefix(rtk_t *rtk)
{
	int m, i;
	rtk->prefixf = 0;
	rtk->prefixn = 0;
	rtk->prefix_sat = 0;
	rtk->prefixr = 0.0;
	rtk->prefixt.time = 0;
	rtk->prefloat = FALSE;

	for (m = 0; m < NSYSDD; m++)
	{
		rtk->biasnum[m] = 0;
	}

	for (i = 0; i < MAXSAT; i++) 
	{
		rtk->fixstate[i] = 0;
	}
}

extern int gnss_rtk_get_prefixsol(rtk_t *rtk,double *H,double *v,int nv,int satuse)
{
	double x[3] = { 0.0 }, Q[9] = { 0.0 }, blh[3] = { 0.0 }, fage = 0.0;
	int info,i;
	double pdop = 0.0;

	if (nv < 4 || satuse < 4)
	{
		GLOGI("prefix: fix nv or sv num check fail. nv=%d, sv=%d",nv,satuse);
		return 0;
	}	

	fage = timediff(rtk->sol.time, rtk->prefixt);
	if ((info=lsq(H,v,3,nv,x,Q)) != 0)
	{
		GLOGI("prefix: lsq err. info=%d",info);
		return 0;
	}

	pdop=gnss_rtk_getpdop_prefix(H, 3, nv);
	GLOGI("prefix pdop=%.3f nv=%d", pdop, nv);
#ifndef ASM_MATMUL
	matmul("TN",nv,1,3,1.0,H,x,-1.0,v);
#else
	Fast_Rtk_matmul("TN", nv, 1, 3, 1.0, H, x, -1.0, v);
#endif
	rtk->prefixsigma0=sqrt(dot(v,v,nv)/(nv-3.0));
	if (rtk->prefixsigma0 > 0.1 || pdop > 50.0)
	{
		gnss_rtk_clean_prefix(rtk);
		GLOGI("prefix: sigma0=%.4f pdop=%.3f ,too large, clean prefix", rtk->prefixsigma0, pdop);
		return 0;
	}

	for (i=0;i<3;i++)
	{
		rtk->prefixsol[i]=x[i]+rtk->x[i];
	}
	ecef2pos(rtk->prefixsol,blh);
	blh[0]=blh[0]*R2D;
	blh[1]=blh[1]*R2D;
	//save var-cov info
	rtk->prefixQmat[0] = (float) (Q[0] * rtk->prefixsigma0 * rtk->prefixsigma0);
	rtk->prefixQmat[1] = (float) (Q[4] * rtk->prefixsigma0 * rtk->prefixsigma0);
	rtk->prefixQmat[2] = (float) (Q[8] * rtk->prefixsigma0 * rtk->prefixsigma0);
	rtk->prefixQmat[3] = (float) (Q[1] * rtk->prefixsigma0 * rtk->prefixsigma0);
	rtk->prefixQmat[4] = (float) (Q[5] * rtk->prefixsigma0 * rtk->prefixsigma0);
	rtk->prefixQmat[5] = (float) (Q[2] * rtk->prefixsigma0 * rtk->prefixsigma0);

	rtk->prefix_sat = satuse;
	
	GLOGI("prefix: %.3f fage=%.3f dx=%.3f %.3f %.3f lla=%.8f %.8f %.8f sigma0=%.3f nv=%d sv=%d",rtk->ptor,fage,x[0],x[1],x[2],
		blh[0],blh[1],blh[2],rtk->prefixsigma0,nv,satuse);
	return 1;
}
/* check if need to update prefix sol-------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_prefix_upd_check(rtk_t *rtk, int nb)
{
	return 1;
	//if (nb > 5 && rtk->sol.ratio > 3.0)
	//{
	//	return 1;
	//}
	//return 0;
}
/*restore sd amb fix state && sd ambs*/
extern void gnss_rtk_sdamb_state(rtk_t *rtk,const double *bias)
{
	int i, n, m, f, index[MAXSAT] = {0}, nv = 0, nf = NF(&(rtk->opt));
	char fixedsat[1024]={0};
	char *pf = fixedsat;

	rtk->prefixt = rtk->sol.time;
	rtk->prefixr = rtk->sol.ratio;
	rtk->prefixn = 0;
	rtk->prefloat = FALSE;

	for (m=0;m<NSYSDD;m++) for (f=0;f<nf;f++) {
		pf = &fixedsat[0];
		if (f==0) rtk->biasnum[m]=0;
		for (n=i=0;i<MAXSAT;i++) {
			if (rtk->ssat[i] == NULL)
			{
				continue;
			}
			if (!test_sys(rtk->ssat[i]->sys,m))
			{
				continue;
			}
			if (rtk->ssat[i]->fix[f]<2)
			{
				rtk->fixstate[i] &= ~FREQ_FIX(f);
				continue;
			}
			index[n++]=i;
			rtk->fixstate[i] |= FREQ_FIX(f);
			pf+=sprintf(pf," %3d",i+1);
		}
		*pf = '\0';
		if (n>0) GLOGI("fixed sat: %d-L%d[%s]%d", m, f+1, fixedsat, n);
		if (n<2) continue;
		rtk->biasnum[m]+=n;
		rtk->prefixn+=n;
		rtk->ssat[index[0]]->sdamb[f] = rtk->x[rtk->ix[IB(index[0] + 1, f, &rtk->opt)]];
		for (i=1; i<n; i++)
		{
			rtk->ssat[index[i]]->sdamb[f]=rtk->ssat[index[0]]->sdamb[f]-bias[nv++];
#ifdef RTK_DEBUG_LOG
			GLOGI("%3d-%3d restore bias %13.3f %13.3f %13.3f", index[0]+1, index[i]+1, bias[nv-1],
			rtk->ssat[index[0]]->sdamb[f], rtk->ssat[index[i]]->sdamb[f]);
#endif
		}
	}
	
}
extern void gnss_rtk_sol_fill(rtk_t *rtk, sol_t *solspp, int stat)
{
	int i = 0;

	/* save solution status */
	if (stat == SOLQ_FIX) {
		for (i = 0; i < 3; i++) {
			rtk->sol.rr[i] = rtk->xa[i];
			rtk->sol.qr[i] = (float)rtk->Pa[i + i * rtk->na];
		}
		rtk->sol.qr[3] = (float)rtk->Pa[1];
		rtk->sol.qr[4] = (float)rtk->Pa[1 + 2 * rtk->na];
		rtk->sol.qr[5] = (float)rtk->Pa[2];

#if 0
		double pos[3];
		ecef2pos(rtk->x, pos);
		GLOGI("rtk->x:%f,%f,%f", rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2]);
		GLOGI("lla:%.7f,%.7f,%.3f", pos[0] * R2D, pos[1] * R2D, pos[2]);
#endif

		if (((g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) == 0 && rtk->opt.dynamics) || g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
		{
			for (i = 3; i < 6; i++) {
				rtk->sol.rr[i] = rtk->xa[i];
			}
		}
	}
	else if (stat == SOLQ_FLOAT || stat == SOLQ_DGPS) {
		for (i = 0; i < 3; i++) {
			rtk->sol.rr[i] = rtk->x[i];
			if (rtk->wl.flag & 1)
			{
				rtk->sol.rr[i] = rtk->wl.sol[i];
			}
			rtk->sol.qr[i] = (float)rtk->P[ISM(i, i)];
		}
		rtk->sol.qr[3] = (float)rtk->P[ISM(0, 1)];
		rtk->sol.qr[4] = (float)rtk->P[ISM(1, 2)];
		rtk->sol.qr[5] = (float)rtk->P[ISM(0, 2)];


		if (((g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) == 0 && rtk->opt.dynamics) || g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
		{
			for (i = 3; i < 6; i++) {
				rtk->sol.rr[i] = rtk->x[i];
			}
		}
	}
	else if (stat == SOLQ_SINGLE || (stat == SOLQ_NONE && solspp->stat == SOLQ_SINGLE)) {
		rtk->sol = *solspp;//output spp results
	}
	else if ((stat == SOLQ_DR) && (solspp->stat == SOLQ_SINGLE)) {
		stat = SOLQ_SINGLE;
		rtk->sol = *solspp;//output spp results
		//rtk->sol=solpre;
	}
	else {
		rtk->sol.stat = SOLQ_NONE;
		stat = SOLQ_NONE;
	}

	if (stat != SOLQ_NONE) rtk->sol.stat = stat;

	if ((stat == SOLQ_FLOAT || stat == SOLQ_DGPS) && (rtk->wl.flag & 1)) rtk->sol.stat = SOLQ_FIX;
}
/* ddamb resolve -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_ddamb_resolve(rtk_t *rtk, int m, int f, int sati, int satj, double resamb)
{
	double ddamb,bias,fcb=0.0;

	bias = floor(resamb + 0.5);
	fcb = fabs(bias - resamb);

#ifdef RTK_DEBUG_LOG
	GLOGI("ddamb: %10.3f %d %d sat=%3d-%3d L%d v=%13.3f %6.3f j: %d %d i: %d ref: %d %d %4.1f %2u %10.3f %10.3f", rtk->ptor,
		rtk->ssat[sati - 1]->fix[f], rtk->ssat[satj - 1]->fix[f], sati, satj, f + 1,
		resamb, fcb,
		rtk->ssat[satj - 1]->hod[f].obs[0][0].llicnt, rtk->ssat[satj - 1]->slip[f] & 0x3, rtk->ssat[sati - 1]->hod[f].obs[0][0].llicnt,
		rtk->ssat[satj - 1]->hod[f].obs[1][0].llicnt, rtk->ssat[sati - 1]->hod[f].obs[1][0].llicnt,
		rtk->ssat[satj - 1]->azel[1] * R2D, rtk->ssat[satj - 1]->snr[f] / 4,
		rtk->ssat[sati - 1]->sdamb[f], rtk->ssat[satj - 1]->sdamb[f]);
#endif

	if (rtk->sol.ratio <= 3.0 || rtk->nbias <= 5) //TODO: take the satnum into account
	{
		return;
	}

	if (!rtk->prefixn)
	{
		return;
	}
	if (!(rtk->fixstate[sati - 1] & FREQ_FIX(f))) //refsat not in prefix status   //if (rtk->ssat[i].fix[f]<2)
	{
		return;
	}

	if (fcb > 0.1)
	{
		if (fcb > 0.4) //half cycle issue possible
		{
			//TODO: mark it, do not use it for several epoch until it recover, or just mark not select as prefix refsat
			if (NULL != rtk->ssat[satj - 1])
			{
				rtk->ssat[satj - 1]->usectrl.halfflag |= FREQ_MASK(f);
			}
		}
		return;
	}

	if (!(rtk->fixstate[satj - 1] & FREQ_FIX(f)))
	{
		if (NULL != rtk->ssat[satj - 1] && NULL != rtk->ssat[sati - 1])
		{
			rtk->ssat[satj - 1]->sdamb[f] = rtk->ssat[sati - 1]->sdamb[f] - bias;

			rtk->fixstate[satj - 1] |= FREQ_FIX(f);
			rtk->biasnum[m] += 1;
			rtk->prefixn += 1;
		}

		GLOGI("ddamb resolve: %d L%d sat=%3d-%3d %13.3f %u", m, f + 1, sati, satj, bias, rtk->biasnum[m]);
	}
	else //check prefix ddamb and bias
	{
		if (NULL != rtk->ssat[satj - 1] && NULL != rtk->ssat[sati - 1])
		{
			ddamb = rtk->ssat[sati - 1]->sdamb[f] - rtk->ssat[satj - 1]->sdamb[f];
			if (fabs(ddamb - bias) > 0.01)
			{
				rtk->fixstate[sati - 1] &= ~FREQ_FIX(f);
				rtk->fixstate[satj - 1] &= ~FREQ_FIX(f);
				rtk->biasnum[m] -= 2;
				rtk->prefixn -= 2;

				GLOGW("ddamb check fail: %d L%d sat=%3d-%3d %13.3f %13.3f %u", m, f + 1, sati, satj, ddamb, bias, rtk->biasnum[m]);
			}
		}
	}

}

/*update sd amb fix status via slip flag */
extern void gnss_rtk_sdstate_upd(rtk_t *rtk)
{
	int m, i, f, nf = NF(&rtk->opt);
	unsigned char prefixn, fixstate[MAXSAT] = { 0 }, biasnum[NSYSDD] = {0};

	if (rtk->prefixn == 0) return;

LAB_PRE_FLOAT:
	prefixn = rtk->prefixn;
	for (m = 0; m < NSYSDD; m++)
	{
		for (f = 0; f < nf; f++)
		{
			for (i = 0; i < MAXSAT; i++)
			{
				if (rtk->ssat[i] == NULL)
				{
					continue;
				}
				if (!test_sys(rtk->ssat[i]->sys, m))
				{
					continue;
				}
				fixstate[i] = rtk->fixstate[i];
				biasnum[m] = rtk->biasnum[m];
			}
		}
	}
	
	for (m = 0; m < NSYSDD; m++) 
	for (f = 0; f < nf; f++) 
	{
		for (i = 0; i < MAXSAT; i++) 
		{
			if (rtk->ssat[i] == NULL)
			{
				continue;
			}
			if (!test_sys(rtk->ssat[i]->sys, m))
			{
				continue;
			}
			if (rtk->prefloat == FALSE && (rtk->fixstate[i] & FREQ_FIX(f)) && ((rtk->ssat[i]->slip[f] & 0x3) != 0 || (rtk->ssat[i]->usectrl.halfflag&FREQ_MASK(f))))
			{
				fixstate[i] &= ~FREQ_FIX(f);
				prefixn--;
				biasnum[m]--;
				GLOGI("prefix delete sat %d f=%d", i + 1, f);
			}
			//((rtk->ssat[i]->slipflag[0][f] & CYCSLIPDET_LLI) && !(rtk->ssat[i]->slipflag[0][f] & CYCSLIPDET_LLI_HC))
			if (rtk->prefloat == TRUE && (rtk->fixstate[i] & FREQ_FIX(f)) && (rtk->ssat[i]->slip[f] & 0x1) != 0)
			{
				fixstate[i] &= ~FREQ_FIX(f);
				prefixn--;
				biasnum[m]--;
				GLOGI("prefloat delete sat %d f=%d", i + 1, f);
			}
		}
	}

	if (rtk->prefloat == FALSE && prefixn <= 5)
	{
		rtk->prefloat = TRUE;
		GLOGI("prefix change to prefloat");
		goto LAB_PRE_FLOAT;
	}

	rtk->prefixn = prefixn;
	for (m = 0; m < NSYSDD; m++)
	{
		for (f = 0; f < nf; f++)
		{
			for (i = 0; i < MAXSAT; i++)
			{
				if (rtk->ssat[i] == NULL)
				{
					continue;
				}
				if (!test_sys(rtk->ssat[i]->sys, m))
				{
					continue;
				}
				rtk->fixstate[i] = fixstate[i];
				rtk->biasnum[m] = biasnum[m];
			}
		}
	}

	GLOGI("upd_fix_stat: fage=%.3f nb=%u GREC:%u %u %u %u, prefloat %d", timediff(rtk->sol.time, rtk->prefixt), rtk->prefixn,
		rtk->biasnum[0], rtk->biasnum[1], rtk->biasnum[2], rtk->biasnum[3], rtk->prefloat);
}
/*  -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_prefix_resolve_half(rtk_t *rtk, const nav_t *nav, const int *sat, const double *azel, const double *_y, const int *iu, const int *ir, int ns,
	const obsd_t *obs, int n, const double *rs, const double *dts, const int *svh)
{
	double r, rr_[3], e[3], *y = NULL;
	int i,j, nf = NF(&rtk->opt);
	double lami, lamj, v, ddamb, bias;
	int m, f, sysi, sysj;

	if (!rtk->prefixf)
	{
		return;
	}
	//malloc y, only for cp
	y = mat(nf, n);
	if (!y)  return;

	//zdres
	for (i = 0;i<n*nf;i++) y[i] = 0.0;
	for (i = 0;i<3;i++) rr_[i] = rtk->prefixsol[i]; //get prefix X

	for (i = 0;i<n;i++) 
	{
		/* compute geometric-range and azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rr_, e)) <= 0.0) continue;
		if (azel[i*2+1]<rtk->opt.elmin) continue;

		/* excluded satellite? */
		if (satexclude(obs[i].sat, svh[i], &rtk->opt)) continue;

		/* satellite clock-bias */
		r += -CLIGHT * dts[i * 2];

		for (f = 0;f<nf;f++) 
		{
			if (nav->lam[obs[i].sat - 1][f] == 0.0) continue;
			/* check snr mask */
			if (obs[i].SNR[f] * 0.25 < 20)  continue;
			/* residuals = observable - pseudorange */
			if (obs[i].L[f] != 0.0) y[f + i * nf] = obs[i].L[f] * nav->lam[obs[i].sat - 1][f] - r;
		}
		
	}
	
	//ddres
	for (m = 0;m < NSYSDD;m++)
	{
		for (f = 0;f < nf;f++) 
		{
			/* search reference satellite with highest elevation */
			for (i = -1, j = 0;j < ns;j++) {
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				sysi = rtk->ssat[sat[j] - 1]->sys;
				if (!test_sys(sysi, m)) continue;
				if (!(y[f + iu[j] * nf] != 0.0&&_y[f + ir[j] * nf * 2] != 0.0)) continue;
				if (!(rtk->fixstate[sat[j] - 1] & FREQ_FIX(f))) continue;
				if (i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) i = j;
			}

			if (i < 0)
			{
				for (i = -1, j = 0;j < ns;j++) {
					sysi = rtk->ssat[sat[j] - 1]->sys;
					if (!test_sys(sysi, m)) continue;
					if (!(y[f + iu[j] * nf] != 0.0&&_y[f + ir[j] * nf * 2] != 0.0)) continue;
					if (rtk->ssat[sat[j] - 1]->usectrl.halfflag&FREQ_MASK(f)) continue;
					if (rtk->ssat[sat[j] - 1]->slip[f]&0x3) continue;
					if (i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) i = j;
				}
			}

			if (i < 0) continue;

			/* make double difference */
			for (j = 0;j < ns;j++) {
				if (i == j) continue;
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				//sysi = rtk->ssat[sat[i] - 1]->sys;
				sysj = rtk->ssat[sat[j] - 1]->sys;
				if (!test_sys(sysj, m)) continue;
				if (!(y[f + iu[j] * nf] != 0.0&&_y[f + ir[j] * nf * 2] != 0.0)) continue;

				lami = nav->lam[sat[i] - 1][f];
				lamj = nav->lam[sat[j] - 1][f];
				if (lami <= 0.0 || lamj <= 0.0) continue;

				/* double-differenced residual */
				v = (y[f + iu[i] * nf] - _y[f + ir[i] * nf * 2]) -
					(y[f + iu[j] * nf] - _y[f + ir[j] * nf * 2]);

				ddamb = m!=1?(v/lami):((v-(lami-lamj)*rtk->x[rtk->ix[IB(sat[j], f, &rtk->opt)]])/lami);
				
				bias = round(ddamb);
				if (fabs(bias - ddamb) > 0.4) // half cycle ambiguity possible
				{
					rtk->ssat[sat[j] - 1]->usectrl.fix_rejflag |= FREQ_MASK(f);
				}
				else if (fabs(bias - ddamb) < 0.1)
				{
					rtk->ssat[sat[j] - 1]->usectrl.halfflag &= ~FREQ_MASK(f);
				}
#ifdef RTK_DEBUG_LOG
				GLOGI("prefix resolve: %d %d sat=%3d-%3d  L%d %13.3f %6.3f %d %d j: %d %d i: %d ref: %d %d", (rtk->fixstate[sat[i] - 1] & FREQ_FIX(f))>0,
					(rtk->fixstate[sat[j] - 1] & FREQ_FIX(f))>0, sat[i], sat[j], f+1, ddamb, bias - ddamb,rtk->ssat[sat[j] - 1]->usectrl.fix_rejflag,
					rtk->ssat[sat[j] - 1]->usectrl.halfflag,
					rtk->ssat[sat[j] - 1]->hod[f].obs[0][0].llicnt, rtk->ssat[sat[j] - 1]->slip[f]&0x3, rtk->ssat[sat[i] - 1]->hod[f].obs[0][0].llicnt,
					rtk->ssat[sat[j] - 1]->hod[f].obs[1][0].llicnt, rtk->ssat[sat[i] - 1]->hod[f].obs[1][0].llicnt);
#endif
			}
		}
	}
	
	Sys_Free(y);
}
extern void gnss_rtk_prefix_resolve(rtk_t *rtk, const nav_t *nav, const int *sat, const double *azel, const double *_y, const int *iu, const int *ir, int ns,
	const obsd_t *obs, int n, const double *rs, const double *dts, const int *svh)
{
	double r, rr_[3], e[3], *y = NULL;
	int i, j, nf = NF(&rtk->opt);
	double lami, lamj, v, ddamb, bias;
	int m, f, sysi, sysj;

	if (rtk->sol.stat == SOLQ_FIX && !(rtk->wl.flag & 1))
	{
		for (i = 0; i < 3; i++) rr_[i] = rtk->xa[i]; //get prefix X
	}
	else if (rtk->prefixf == TRUE)
	{
		for (i = 0; i < 3; i++) rr_[i] = rtk->prefixsol[i]; //get prefix X
	}
	else
	{
		return;
	}

	//malloc y, only for cp
	y = mat(nf, n);
	if (!y)  return;

	//zdres
	for (i = 0; i < n*nf; i++) y[i] = 0.0;

	for (i = 0; i < n; i++)
	{
		/* compute geometric-range and azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rr_, e)) <= 0.0) continue;
		if (azel[i * 2 + 1] < rtk->opt.elmin) continue;

		/* excluded satellite? */
		if (satexclude(obs[i].sat, svh[i], &rtk->opt)) continue;

		/* satellite clock-bias */
		r += -CLIGHT * dts[i * 2];

		for (f = 0; f < nf; f++)
		{
			if (nav->lam[obs[i].sat - 1][f] == 0.0) continue;
			/* check snr mask */
			if (obs[i].SNR[f] * 0.25 < 20)  continue;
			/* residuals = observable - pseudorange */
			if (obs[i].L[f] != 0.0) y[f + i * nf] = obs[i].L[f] * nav->lam[obs[i].sat - 1][f] - r;
		}

	}

	//ddres
	for (m = 0; m < NSYSDD; m++)
	{
		for (f = 0; f < nf; f++)
		{
			/* search reference satellite with highest elevation */
			for (i = -1, j = 0; j < ns; j++) {
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				sysi = rtk->ssat[sat[j] - 1]->sys;
				if (!test_sys(sysi, m)) continue;
				if (!(y[f + iu[j] * nf] != 0.0&&_y[f + ir[j] * nf * 2] != 0.0)) continue;
				if (!(rtk->fixstate[sat[j] - 1] & FREQ_FIX(f))) continue;
				if (i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) i = j;
			}

			if (i < 0)
			{
				for (i = -1, j = 0; j < ns; j++) {
					if (NULL == rtk->ssat[sat[j] - 1])
					{
						continue;
					}
					sysi = rtk->ssat[sat[j] - 1]->sys;
					if (!test_sys(sysi, m)) continue;
					if (!(y[f + iu[j] * nf] != 0.0&&_y[f + ir[j] * nf * 2] != 0.0)) continue;
					if (rtk->ssat[sat[j] - 1]->usectrl.halfflag&FREQ_MASK(f)) continue;
					if (rtk->ssat[sat[j] - 1]->slip[f] & 0x3) continue;
					if (i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) i = j;
				}
			}

			if (i < 0) continue;

			/* make double difference */
			for (j = 0; j < ns; j++) {
				if (i == j) continue;
				if (NULL == rtk->ssat[sat[j] - 1])
				{
					continue;
				}
				//sysi = rtk->ssat[sat[i] - 1]->sys;
				sysj = rtk->ssat[sat[j] - 1]->sys;
				if (!test_sys(sysj, m)) continue;
				if (!(y[f + iu[j] * nf] != 0.0&&_y[f + ir[j] * nf * 2] != 0.0)) continue;

				lami = nav->lam[sat[i] - 1][f];
				lamj = nav->lam[sat[j] - 1][f];
				if (lami <= 0.0 || lamj <= 0.0) continue;

				/* double-differenced residual */
				v = (y[f + iu[i] * nf] - _y[f + ir[i] * nf * 2]) -
					(y[f + iu[j] * nf] - _y[f + ir[j] * nf * 2]);

				ddamb = m != 1 ? (v / lami) : ((v - (lami - lamj)*rtk->x[rtk->ix[IB(sat[j], f, &rtk->opt)]]) / lami);

				bias = round(ddamb);

				if (rtk->prefloat == FALSE)
				{
					if ((rtk->ssat[sat[j] - 1]->slip[f] & 0x3) == 0)
					{
						if (!(rtk->fixstate[sat[j] - 1] & FREQ_FIX(f)))
						{
							if (fabs(bias - ddamb) < 0.1)
							{
								rtk->ssat[sat[j] - 1]->sdamb[f] = rtk->ssat[sat[i] - 1]->sdamb[f] - bias;
								rtk->fixstate[sat[j] - 1] |= FREQ_FIX(f);
								if (rtk->biasnum[m] == 0)
								{
									rtk->fixstate[sat[i] - 1] |= FREQ_FIX(f);
									rtk->biasnum[m] += 2;
									rtk->prefixn += 2;
								}
								else
								{
									rtk->biasnum[m] += 1;
									rtk->prefixn += 1;
								}
								GLOGI("prefix resolve: %d L%d sat=%3d-%3d %13.3f %u", m, f + 1, sat[i], sat[j], bias, rtk->biasnum[m]);
							}
						}
					}
				}
				else
				{
					if (!(rtk->fixstate[sat[j] - 1] & FREQ_FIX(f)) && (rtk->ssat[sat[j] - 1]->slip[f] & 0x1) == 0)
					{
						rtk->ssat[sat[j] - 1]->sdamb[f] = rtk->ssat[sat[i] - 1]->sdamb[f] - ddamb;
						rtk->fixstate[sat[j] - 1] |= FREQ_FIX(f);
						rtk->biasnum[m] += 1;
						rtk->prefixn += 1;

						GLOGW("prefloat resolve: %d L%d sat=%3d-%3d, ddamb %13.3f, bias %13.3f, %u", m, f + 1, sat[i], sat[j], ddamb, bias, rtk->biasnum[m]);
					}
				}

#ifdef RTK_DEBUG_LOG
				GLOGI("prefix resolve: %d %d sat=%3d-%3d  L%d %13.3f %6.3f %d %d j: %d %d i: %d ref: %d %d", (rtk->fixstate[sat[i] - 1] & FREQ_FIX(f)) > 0,
					(rtk->fixstate[sat[j] - 1] & FREQ_FIX(f)) > 0, sat[i], sat[j], f + 1, ddamb, bias - ddamb, rtk->ssat[sat[j] - 1]->usectrl.fix_rejflag,
					rtk->ssat[sat[j] - 1]->usectrl.halfflag,
					rtk->ssat[sat[j] - 1]->hod[f].obs[0][0].llicnt, rtk->ssat[sat[j] - 1]->slip[f] & 0x3, rtk->ssat[sat[i] - 1]->hod[f].obs[0][0].llicnt,
					rtk->ssat[sat[j] - 1]->hod[f].obs[1][0].llicnt, rtk->ssat[sat[i] - 1]->hod[f].obs[1][0].llicnt);
#endif
			}
		}
	}

	Sys_Free(y);
}
/* get LS fix sol */
extern int gnss_rtk_lsfixsol(rtk_t *rtk, const nav_t *nav, const double *x, const int *sat, double *y, double *e,
	double *azel, const int *iu, const int *ir, int ns)
{
	double H_f[MAXSAT * 3] = { 0.0 }, v_f[MAXSAT] = {0.0}, dtf = timediff(rtk->sol.time, rtk->prefixt);
	int nv_f = 0,satcnt=0,fixage=60;
	
	if (g_pe_cfg.rinexDataType == RINEX_DATA_TYPE_UM4B0 || g_pe_cfg.applyScenario == APPLY_SCENE_DRONE)
	{
		fixage = 60*30;
	}

	GLOGI("LSFIXSOL dtf=%1f prefixn=%d prefixr=%.1f reverse=%u", dtf, rtk->prefixn, rtk->prefixr,IS_PPK_REVERSE);
	rtk->prefixf = 0;
	if ((rtk->prefloat == FALSE || (rtk->prefloat == TRUE && fabs(dtf) < fixage)) && (rtk->prefixn > 5))
	{
		nv_f = gnss_rtk_fddres(rtk, nav, x, sat, y, e, azel, iu, ir, ns, v_f, H_f,&satcnt);
#ifdef _WIN32
		gnss_util_trace(5, "nv_f=%2d\n", nv_f);
		gnss_util_trace(5, "H_f=\n");
		gnss_util_tracemat(5, H_f, nv_f, 3, 7, 4);
		gnss_util_trace(5, "v_f=\n");
		gnss_util_tracemat(5, v_f, 1, nv_f, 6, 3);
#endif
		if (!(rtk->prefixf = gnss_rtk_get_prefixsol(rtk, H_f, v_f, nv_f,satcnt)))
		{
			gnss_util_trace(3, "get pre fix sol fail\n");
			return 0;
		}
		
		return 1;
	}

	return 0;
}
extern double get_pe_meas_prNoise(int sat,int freq_index)
{
	meas_blk_t*    pMeas = NULL;
	gnss_meas_t*   pSvMeas;
	uint32_t            i;
	int            gnssmode;
	double         R=-1.0;
	int            prn =0,sys=0;
	float            prNoiseVar = 0.0;

	pMeas = gpz_kf_data->meas_blk;
	
	sys=satsys(sat,&prn);
	gnssmode = GNSS_SYS2MODE(sys);
	
	for(i=0;i<pMeas->measCnt;i++)
	{
		pSvMeas = &(pMeas->meas[i]);
		if ((pSvMeas->status & 0x1) != 0x1) continue;
		if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
		{
			prNoiseVar = gnss_Kf_PrNoiseVarCal(pSvMeas, pMeas->avgCno, 1);
		}
		if (pSvMeas->prNoise < 0.00001 && prNoiseVar < 0.00001) continue;
		if(pSvMeas->gnssMode == gnssmode && pSvMeas->prn == prn && pSvMeas->freq_index== freq_index)
		{
			 R = pSvMeas->prNoise < 0.00001 ? prNoiseVar : pSvMeas->prNoise;
		     break;
		}
	}

	return R;
}
/* get dopper sd res -------------------------------------------------------------
* get dopper sd res
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_dr_sd_res(rtk_t *rtk,double *v,int *nv,double *H,double *Hi,double *Ri,double *Rj,int *nb,int *b,int ns,int nf, int* max_doppler_num)
{
	meas_blk_t*    pMeas = NULL;
	gnss_meas_t*   pSvMeas = NULL;
	gnss_meas_t*   ref_pSvMeas = NULL;
	//SV_INFO*       pSvInfo = NULL;
	double*		   dcos = NULL;
	double*		   refdcos = NULL;
	float*           svVel = NULL;
	float*           refsvVel = NULL;
	double            drKf = 0,ref_drKf = 0;
	double			   drms = 0;
	int            i,j,k,/*idx,*/cno_threshold;
	int            tempnb = *nb,temp_nv=*nv,temp_b = *b,temp_nv_back=*nv;
	float            ref_drNoiseVar = 0.0, drNoiseVar = 0.0;

	pMeas = gpz_kf_data->meas_blk;

	//get ref sat with biggest cno, pMeas->meas has been sorting by cno
	for (i = 0; i < (int)pMeas->measCnt; i++)
	{
		if ((pMeas->meas[i].status & 0x2) != 0x2) continue;
		if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
		{
			ref_drNoiseVar = gnss_Kf_DrNoiseVarCal(&(pMeas->meas[i]), pMeas->avgCno);
		}
		if (pMeas->meas[i].drNoise < 0.00001 && ref_drNoiseVar < 0.00001) continue;

		refdcos = &(pMeas->meas[i].sv_info.dcos[0]);
		refsvVel = &(pMeas->meas[i].sv_info.v[0]);
		ref_pSvMeas = &(pMeas->meas[i]);
		break;
	}

	if (i >= (int)pMeas->measCnt) {
	  return;
	}

	if (NULL == refdcos || 
		NULL == refsvVel ||
		NULL== ref_pSvMeas)
	{
	  return;
	}

	//set cno_threshold
	/* change can_threshold to float ???*/
	cno_threshold = (int)(pMeas->avgCno*1.1f);
	if (pMeas->avgCno*1.1 > 35.0)
	{
		cno_threshold = 35;
	}

	for (k = 0; k < 3; k++)
	{
		ref_drKf += refdcos[k] * (rtk->x[k + 3] - (double)refsvVel[k]);
	}
	for (j = 0; j < (int)pMeas->measCnt; j++)
	{
		if (max_doppler_num)
		{
			if (temp_nv >= *max_doppler_num)
			{
				GLOGI("DR sd res count overflow, disgard others");
				continue;
			}
		}
		if ((g_pe_cfg.meas_rate > MEAS_RATE_1HZ && temp_nv >= CMN_OBS_NUM_HF)|| (g_pe_cfg.meas_rate == MEAS_RATE_1HZ && temp_nv >= CMN_OBS_NUM))
		{
			GLOGI("DR sd res count overflow, disgard others");
			break;
		}
		if (j == i) continue;
		if ((pMeas->meas[j].status & 0x2) != 0x2) continue;
		if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
		{
			drNoiseVar = gnss_Kf_DrNoiseVarCal(&(pMeas->meas[j]), pMeas->avgCno);
		}
		if (pMeas->meas[j].drNoise < 0.00001 && drNoiseVar < 0.00001) continue;
		if(g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
		{
			if(pMeas->meas[j].cno < (uint32_t)cno_threshold &&(!peMode.userSceData.isUnderEleRoad|| peMode.staticData.staticFlag)) continue;
			if ((temp_nv - temp_nv_back) > 10 &&(!peMode.userSceData.isUnderEleRoad || peMode.staticData.staticFlag)&&(rtk->qcpasscnt & 0xF)== 0&& pMeas->avgCno<36.0)
			{
				GLOGI("only update ten doppler measurements, disgard others");
				break;
			}
		}
		/*idx = gnss_sv_Idx(pMeas->meas[i].gnssMode,pMeas->meas[i].prn);
		if(pMeas->satmask[idx] & 0x3 !=0x3)
		{
			break;
		}*/
		//if (pMeas->meas[j].freq_index > 0) continue;//L1 only

		//for(j = 0; j < pMeas->measCnt&&j< MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM;j++)
		//{
		//	if (i == j) continue;
		//	if (pMeas->meas[i].freq_index < 1) continue;//L2 only
		//	if((pMeas->meas[i].prn==pMeas->meas[j].prn)&&(pMeas->meas[i].gnssMode==pMeas->meas[j].gnssMode))
		//	{
		//		if(pMeas->meas[i].dr_diff>pMeas->meas[j].dr_diff) i=j;
		//		break;
		//	}			
		//}

		pSvMeas = &(pMeas->meas[j]);
		//pSvInfo = &(pSvMeas->sv_info);
		dcos = &(pSvMeas->sv_info.dcos[0]);
		svVel = &(pSvMeas->sv_info.v[0]);

		Ri[temp_nv] = ref_pSvMeas->drNoise < 0.00001 ? ref_drNoiseVar : ref_pSvMeas->drNoise;
		if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
		{
			if(peMode.userSceData.isWALKMODE|| peMode.staticData.staticFlag)
			{
				Ri[temp_nv] *= 30.0;
			}
			else
			{
				Ri[temp_nv] *= 10.0;
			}
		}
		//Ri[nv] = pSvMeas->dopplerStd*2;
		if (pSvMeas->status & 0x2)
		{
			Rj[temp_nv] = pSvMeas->drNoise < 0.00001 ? drNoiseVar : pSvMeas->drNoise;

			if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
			{
				if(peMode.userSceData.isWALKMODE|| peMode.staticData.staticFlag)
				{
					Rj[temp_nv] *= 30.0;
				}
				else
				{
					Rj[temp_nv] *= 10.0;
				}
			}
			if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 && (pSvMeas->cycleSlipCount & 0x3) && 
				(peMode.staticData.historyStatic & 0xF) == 0) 
				Rj[temp_nv] *= 10.0;
			//Rj[nv] = pSvMeas->dopplerStd*2;
			drKf = 0;

			for (k = 0; k < 3; k++)
			{
				drKf += dcos[k] * (rtk->x[k + 3] - (double)svVel[k]);
			}
			v[temp_nv] = ref_pSvMeas->pseudoRangeRate_raw-ref_drKf - ( pSvMeas->pseudoRangeRate_raw- drKf);
			if (fabs(v[temp_nv]) > 10) continue;
			if (H) {
				Hi = H + temp_nv * rtk->nx;
				for (k = 0; k < rtk->nx; k++) Hi[k] = 0.0;
				for (k = 3; k < 6; k++) {
					Hi[k] = refdcos[k - 3] - dcos[k - 3];
				}
			}
			drms += v[temp_nv] * v[temp_nv] / (Rj[temp_nv] + Ri[temp_nv]);
			GLOGI("DR sd res: %d %3d %d  v[%2d] %7.3f %7.3f R %7.3f %7.3f ", pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index,
				temp_nv,v[temp_nv],pSvMeas->dr_diff,Ri[temp_nv],Rj[temp_nv]);
		}
		temp_nv++;
		tempnb++;
	}
	if ((temp_nv - temp_nv_back) > 0)
	{
		rtk->fat.sigmad = sqrt(drms / ((int64_t)temp_nv - temp_nv_back));
		rtk->fat.drn = (temp_nv - temp_nv_back);
	}
	else
	{
		rtk->fat.sigmad = 0;
		rtk->fat.drn = 0;
	}

	temp_b++;

	*nv = temp_nv;
	*nb = tempnb;
	*b = temp_b;

}

/* lock sat sta for f9  ---------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
#ifdef USED_IN_MC262M
extern void gnss_rtk_ubx_lockv(rtk_t *rtk, const int *sat, int nf, int ns)
{
	int i = 0, m = 0;
	int lockv = 0, f, trkv = 0;
	for (m = 0; m < NSYSDD; m++)
	{
		rtk->lockvs[m] = 0;
		rtk->lockvr[m] = 0;
		lockv = trkv = 0;
		for (i = 0; i < ns; i++)
		{
			if (!test_sys(rtk->ssat[sat[i] - 1]->sys, m)) continue;
			for (f = 0; f < nf; f++)
			{
				if (rtk->ssat[sat[i] - 1]->lockms[0][f] > 0) ++trkv;
				if (rtk->ssat[sat[i] - 1]->lockms[0][f] > UBX_LOCKMS_THRES) ++lockv;
			}
		}
		rtk->lockvs[m] = lockv;
		if (trkv != 0) rtk->lockvr[m] = (float)lockv / trkv;
		else rtk->lockvr[m] = 0;
		gnss_util_trace(3, "lockrt %d %d %d %f ", m, trkv, lockv, rtk->lockvr[m]);
	}
	gnss_util_trace(3, "\n");
	
}
#endif

#endif

