#ifndef __GNSS__KF__ALGO__H__
#define __GNSS__KF__ALGO__H__

#include "gnss_kf.h"


#define ENLARGEQ_SECDS     10
#define BIAS_ERROR_THRES   100
#ifdef __cplusplus
extern "C" {
#endif

	uint8_t gnss_Kf_Get_BiasNum(uint8_t svModeValid,uint8_t biasLinkFlag);
	void gnss_Kf_Static_Detect(Kf_t* p_Kf);
	void gnss_Kf_Static_Proc(Kf_t* p_Kf);
	void gnss_Kf_DR_ErrNum(uint8_t drNum, uint8_t* DrRejLmt);
	void gnss_Kf_Adjust_PMinus(KF_INFO* kfInfo_temp,Kf_t* p_Kf);
	void gnss_Kf_Bias_Adjust(Kf_t* p_Kf);
	void gnss_Kf_Drift_Adjust(Kf_t* p_Kf);
	void gnss_Kf_Alt_Check(float* deltaAlt,uint8_t* index,uint8_t prNum,Kf_t* p_Kf);
	void gnss_Kf_Check_BiasErr(Kf_t* p_Kf);
	void gnss_Kf_ZUPT(KF_INFO* kfInfo_temp,double* deltaX,Kf_t* p_Kf);
	void gnss_Kf_ZUPT_Hold_Static(KF_INFO* kfInfo_temp, double* deltaX, Kf_t* p_Kf);
	void gnss_Kf_UpVel_Hold(KF_INFO* kfInfo_temp,double* deltaX,KF_PVT_INFO* kfPvtInfo,double var);
	void gnss_Kf_Bias_Hold(KF_INFO* kfInfo_temp,double* deltaX);
	void gnss_Kf_Post_Res(Kf_t* p_Kf,KF_INFO* pkfInfo);
	void gnss_Kf_Alt_AlphaBeta(Kf_t* p_Kf);
	void gnss_Kf_Save_Filter(Kf_t* p_Kf,KF_INFO* pkfInfo);
	void gnss_Kf_Restore_Filter(Kf_t* p_Kf,KF_INFO* kfInfo_temp);
	void gnss_Kf_RefHeading_Cal(Kf_t* p_Kf);
	void gnss_Kf_PosHeading_Hold(Kf_t* p_Kf,double* deltaX);
	void gnss_Kf_Set_ChiSqStatus(Kf_t* p_Kf);
	void gnss_Kf_Lla_HeadingCheck(uint8_t n, uint8_t* indx, float* deltaHead, meas_blk_t* pMeas);
#if 0
	void gnss_Kf_Lla_HeadingCheckFlp(uint8_t n, uint8_t* indx, float* deltaHead, meas_blk_t* pMeas);
	void gnss_Kf_Vel_HeadingCheckFlp(uint8_t n, uint8_t* indx, float* deltaHead, meas_blk_t* pMeas, float THRES);
#endif
	float gnss_Kf_PrNoiseVarCal(gnss_meas_t* meas, uint32_t avgCno,uint8_t flag);
	float gnss_Kf_DrNoiseVarCal(gnss_meas_t* meas, uint32_t avgCno);
	void gnss_Kf_PR_Adj(Kf_t* p_Kf,uint8_t gnssMode,double* pseudoRange);
	double gnss_Kf_PrUpdate(gnss_meas_t* pSvMeas, double* deltaX, double pr, float* prNoisVar, uint8_t* updateFlag, double* testOut,uint8_t flag,uint8_t testFlag, uint8_t idx);
	void gnss_Kf_PosBias_Detect(Kf_t* p_Kf,KF_INFO* kfInfo_temp);
	void gnss_Kf_Rslt_Smooth(Kf_t* p_Kf);
	void gnss_Kf_PDR_UpVel_Hold(KF_INFO* kfInfo_temp,double* deltaX,KF_PVT_INFO*kfPvtInfo,double varIn);
#ifdef __cplusplus
}
#endif

#endif