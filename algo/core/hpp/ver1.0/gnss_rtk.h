/************************************************************
* Copyrights(C) 2017 ASG Corporation.
* All rights Reserved
* File: gnss_rtk.h
* Description: 
* Version: 1.0.0
* Author: kun.xu
* Date: 2017/05/08
************************************************************/
#ifndef __GNSS__RTK__HEADER__
#define __GNSS__RTK__HEADER__

#include "gnss.h"
#include "gnss_config.h"
#include "asg_rtcm_decode.h"
#include "gnss_rtcm.h"

#ifdef __cplusplus
extern "C" {
#endif	

#define GNSS_MODE2SYS(gnss,prn)  gnss_rtk_sys_conv(gnss,prn,1)
#define GNSS_SYS2MODE(gnss)      gnss_rtk_sys_conv(gnss,0,0)


#define NSM(n)      ((n + 1)*n / 2)   //symmetric matrix 
#define ISM(i, j)    (j >= i ? ((j + 1)*j / 2 + i) : ((i + 1)*i / 2 + j))  //symmetric matrix 

extern int  gnss_rtk_init(const rtk_algo_cfg_t *rtk_cfg);
extern void gnss_rtk_free();
extern int  gnss_rtk_sys_conv(int gnss, int prn, int mode2sys);
extern void gnss_rtk_nav_conv(GNSS_TIME* pTime, int sys,int prn, void *praweph, void *prtkeph);
extern void gnss_rtk_nav_update(int gnssmode,int prn, void *pnew, void *pbak);
extern void gnss_rtk_rtcm3_lite_proc(asg_rtcm_lite_t* rtcm_lite);
extern void gnss_rtk_rtcm3_proc(rtcm_data_pack_t* rawData);
extern void gnss_rtk_basedata_clean();
extern void gnss_rtk_process(meas_blk_t* pMeas,asg_obs_t* asg_obs);
extern void gnss_rtk_slip_detect_td(rtk_t *rtk,const nav_t *nav,const obsd_t *obs,int nu,int *sat,int *iu,int *ir,int ns,double *y,double *e,double *azel,int n_all,double *crr);
extern int  gnss_rtk_get_init_rr(sol_t *sol,gtime_t time);
extern int  gnss_rtk_slip_determine(int f,ssat_t *psat,int rcv);
extern void gnss_rtk_slip_lc_fusion(int sat,ssat_t *psat,int rcv);
extern int  gnss_rtk_kf_start_check(rtk_t *rtk);
extern int  gnss_rtk_quick_start_check(rtk_t *rtk);
extern int  gnss_rtk_extpol_check(rtk_t *rtk,sol_t *solp);
extern void gnss_rtk_kf_reset_check(rtk_t *rtk,int ns);
extern void gnss_rtk_fill_pvt(rtk_t *rtk,USER_PVT* pvt);
extern void gnss_rtk_rb_changed(rtk_t *rtk);
extern int  gnss_rtk_opensky_check(rtk_t *rtk);
extern int ag_rtk_resamb_WL(rtk_t* rtk, int f1, int f2, const int* sat, const int* iu, const int* ir, int ns, const nav_t* nav, const double *y, const double *e);
extern int ag_rtk_wl_constraint(rtk_t* rtk, double* H, double *R, const int* sat, double* v, int* _nv, int f1, int f2, nav_t* nav);
extern int gnss_rtk_wl_confirm(rtk_t *rtk);
extern void gnss_rtk_kf_ddres_check(int soltype,rtk_t *rtk,const double *v,const int *vflg,int nv,int ns);
extern int  gnss_rtk_sol_determine(rtk_t *rtk,sol_t *solspp,sol_t *solpre,int stat,int ns);
extern void gnss_rtk_feedback2pe(rtk_t *rtk, int stat, int ns);
extern int gnss_rtk_comfirmamb(double *floatx, double *fixx, double *subx);
extern int gnss_rtk_amb_gap(double* floatx, double *b, int nb);
extern void gnss_rtk_fixstatus_check(rtk_t *rtk, int stat);
extern void gnss_rtk_save_floatsol(rtk_t *rtk,sol_t *sol);
extern prcopt_t* gnss_rtk_get_prcopt();
extern solopt_t* gnss_rtk_get_solopt();
extern void gnss_rtk_set_refposopt(int refposopt);
extern void gnss_rtk_ud_optpos(double rb[3],double ru[3]);
extern void gnss_rtk_nav_revconv(int sys,int prn, void *praweph, void *prtkeph);
extern void gnss_rtk_nav_inject2pe(const obsd_t *obs, int n, const nav_t *nav, int opt);
extern int  gnss_rtk_obsd2andr(const obsd_t *obs, int n, const nav_t *nav, AGGnss_Data_t *pRawData,GNSS_TIME* pTime);
extern int gnss_rtk_andr2obsd(const AGGnss_Data_t *pRawData, const meas_blk_t* pMeas, const nav_t *nav, GNSS_TIME* pTime, obs_t *obsdata);
extern int gnss_rtk_andr2obsd_qcom(const AGGnss_Data_t *pRawData, const meas_blk_t* pMeas, const nav_t *nav, GNSS_TIME* pTime, obs_t *obsdata);
extern int  gnss_rtk_obsd2rtcm(const obsd_t *obs, int n, rtcm_data_pack_t *rtcmdata);
extern int  gnss_rtk_loadrtcm3(gtime_t time,ag_rtcm_t *rtcm,FILE *fp,rtcm_data_pack_t *rtcmdata);
extern int  gnss_rtk_zerobaseline(FILE* fp, const obsd_t *obs, int n);
extern void gnss_rtk_kf_static_check(rtk_t *rtk);
extern double gnss_rtk_adjerror(rtk_t *rtk, int isat, int nsat, double v, int f, double R);
extern int  gnss_rtk_refsat_check(rtk_t *rtk,int *nrej,double *vrej,double *vval,int ns,int *refsat, int *nsval);
extern void gnss_rtk_getshiftinfo(rtk_t *rtk, sol_t *solspp,int stat);
extern int gnss_rtk_slctspp_check(rtk_t *rtk, sol_t *solspp);
extern int gnss_rtk_slctrtk_check(rtk_t *rtk, sol_t *solspp);
extern int gnss_rtk_shift_check(rtk_t *rtk);
extern int gnss_rtk_amb_out(rtk_t *rtk, double v, int f, int rovsat, int refsat, double lam, int ddtype);
extern void gnss_rtk_ambp_reset_check(rtk_t *rtk, int ns, int *sat);
extern double gnss_rtk_getpdop_prefix(double *H, int n, int m);
#if defined(PLAYBACK_MODE)
extern void gnss_rtk_get_refdisinfo(rtk_t *rtk, double *refx, int stat);
#endif
extern int gnss_rtk_fddres(rtk_t *rtk,const nav_t *nav,const double *x,const int *sat,double *y,double *e,
	double *azel,const int *iu,const int *ir,int ns,double *v,double *H,int *satuse);
extern void gnss_rtk_solpre_save(rtk_t *rtk, int stat);
extern void gnss_rtk_shiftsol_save(rtk_t *rtk, int stat);
extern int gnss_rtk_lsfixsol(rtk_t *rtk, const nav_t *nav, const double *x, const int *sat, double *y, double *e,
	double *azel, const int *iu, const int *ir, int ns);
extern void gnss_rtk_prefix_resolve_half(rtk_t *rtk, const nav_t *nav, const int *sat, const double *azel, const double *_y, const int *iu, const int *ir, int ns,
	const obsd_t *obs, int n, const double *rs, const double *dts, const int *svh);
extern void gnss_rtk_prefix_resolve(rtk_t *rtk, const nav_t *nav, const int *sat, const double *azel, const double *_y, const int *iu, const int *ir, int ns,
	const obsd_t *obs, int n, const double *rs, const double *dts, const int *svh);
extern void gnss_rtk_clean_prefix(rtk_t *rtk);
extern int gnss_rtk_get_prefixsol(rtk_t *rtk,double *H,double *v,int nv,int satuse);
extern int gnss_rtk_prefix_upd_check(rtk_t *rtk, int nb);
extern void gnss_rtk_sdamb_state(rtk_t *rtk,const double *bias);
extern void gnss_rtk_sol_fill(rtk_t *rtk, sol_t *solspp, int stat);
extern void gnss_rtk_sdstate_upd(rtk_t *rtk);
extern void gnss_rtk_ddamb_resolve(rtk_t *rtk, int m, int f, int sati, int satj, double resamb);
extern void gnss_rtk_sigma(int ddtype,rtk_t *rtk,const double *v,const double *R,
	const int *vflg,int nv,int ns);
extern void gnss_rtk_accu_upd(rtk_t *rtk,USER_PVT* pvt);
extern void gnss_rtk_confidence(rtk_t *rtk,int state);
extern void gnss_rtk_cfd_init(rtk_t *rtk);
extern void gnss_rtk_init_ambp(rtk_t *rtk, int sat, int f);
extern void gnss_rtk_slip_offset(rtk_t *rtk, int sat, int f,double slip);
extern void gnss_rtk_prefix_offset(rtk_t *rtk, int sat, int sys, int f,double slip);
extern sol_t gnss_rtk_get_sol();
extern void gnss_rtk_slpdet_ref(rtk_t *rtk, const nav_t *nav, int nu, int nr, const double *y, const double *e, const double *azel);
extern double get_pe_meas_prNoise(int sat,int freq_index);
extern void gnss_rtk_dr_sd_res(rtk_t *rtk,double *v,int *nv,double *H,double *Hi,double *Ri,double *Rj,int *nb,int *b,int ns,int nf,int* max_doppler_num);
extern double gnss_rtk_ubx_weight(rtk_t *rtk, int sat, int f, int nf, double v);
extern void gnss_rtk_ubx_lockv(rtk_t *rtk, const int *sat, int nf, int ns);
extern double gnss_rtk_adop_get(void);
extern int gnss_rtk_fix2float_check(rtk_t* rtk, int ns, int carrier_num, int fix_carrier_num);
extern void gnss_rtk_InitF(double T);
extern void gnss_rtk_InitQ(double Tin, float* accSigmaInit);
extern void gnss_rtk_factor(rtk_t *rtk, int state, double rherr, double ruerr);
extern void gnss_rtk_state_sta(rtk_t *rtk, int state);
extern void gnss_rtk_state_sta_init(rtk_t *rtk);
/* common use functions */
extern uint8_t   sumofbit1(uint64_t marklist, uint8_t lsbwide);
extern void mattrans(double *H,int row,int col,double *T);
extern int  matdelcol(double *H,int row,int col,int colinx);
extern int  matdelrow(double *H,int row,int col,int rowinx);

nav_t* gnss_rtk_get_nav(void);
#ifdef __cplusplus
}
#endif

#endif