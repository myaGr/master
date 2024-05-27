/************************************************************
* Copyrights(C) 2019 ASG Corporation.
* All rights Reserved
* File: gnss_cfd.c
* Description:
* Version: 1.0.0
* Author: xintan.li
* Date: 2019/05/31
************************************************************/
#ifndef GNSS_CFD_H
#define GNSS_CFD_H
#include "rtklib.h"
#include "ag_base.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
	uint8_t inputdim;
	uint8_t hidlayer;
	uint8_t hidnode;
	uint8_t act_h;	/*0:relu 1:linear*/
	uint8_t act_o;
	uint8_t name[20];
	const float *coes;
	const float *bias;
	const float *maxfat;
	const float *minfat;
	const float *maxerr;
	const float *minerr;
}model_t;
extern void gnss_rtk_factor(rtk_t *rtk, int state, double rherr, double ruerr);
extern void gnss_rtk_state_sta(rtk_t *rtk, int state);
extern void gnss_rtk_state_sta_init(rtk_t *rtk);
extern void gnss_rtk_unc_getfat(rtk_t *rtk, int state, float *fat);
extern void gnss_rtk_unc_predict(rtk_t *rtk, int state);
extern void gnss_rtk_unc_realtime_test(rtk_t *rtk, int state, double herr, double uerr);
extern void gnss_rtk_unc_batch_test();
extern void gnss_rtk_unc_get_model(int state,int prefixf,const model_t **modelh,const model_t **modelv);
extern int gnss_pe_spp_get_fat(float *sppfat, double *herr, double *verr, int *hpv);
extern uncertainty_t* gnss_get_unc(int difstate);
extern void gnss_pe_unc_predict(uncertainty_t* pvtunc);
extern void gnss_unc_out();
extern void gnss_rtk_factor_out(rtk_t *rtk, int state);
extern int gnss_unc_opensky_check();

#ifdef __cplusplus
}
#endif
#endif