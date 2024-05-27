/************************************************************
* Copyrights(C) ǧѰλ���������޹�˾
* All rights Reserved
* �ļ����ƣ�gnss_pe.h
* �ļ�������
* �汾�ţ�1.0.0
* ���ߣ�Yongsong.wang
* ���ڣ�2016/09/02
************************************************************/
#ifndef __GNSS__PE__H__
#define __GNSS__PE__H__

#include "gnss_types.h"
#include "gnss_config.h"
#include "gnss_ls.h"
#include "gnss_kf.h"
#include "asg_rtcm_decode.h"
#include "gnss_tdcp.h"
#include "gnss_type.h"

enum
{
    BIAS_NONE = 0,
	BIAS_LS,
	BIAS_KF
};

#define POSITION_HOLD_TIME         600

#define BIAS_ADJUST_TIME           (1e-3)

#define REQ_TIME         1
#define REQ_LOC          2
#define REQ_EPH          3
#define REQ_IONO         4
#define REQ_UTC          5
#define REQ_ALM          6
#define REQ_GLO_AUX      7


// QCOM platform
#define DEFAULT_GPSGLN_HW_BIAS              (-1464.186)
#define DEFAULT_GPSBDS_HW_BIAS              (786.481)
#define DEFAULT_GPSGLN_HW_BIAS_AUTO        (-1475.0)

// BRCOM 47755 platform
#define DEFAULT_GPSGLN_HW_BIAS_BRCOM        (0)
#define DEFAULT_GPSBDS_HW_BIAS_BRCOM        (-6.8)
#define DEFAULT_GPSGAL_HW_BIAS_BRCOM        (-2.0)

// MTK
#define DEFAULT_GPSBDS_HW_BIAS_AUTO_MTK   (-25)

// SPRD platform
#define DEFAULT_GPSGLN_HW_BIAS_SPRD         (0.0)

//UBLOX F9
#define DEFAULT_GPSGLN_HW_BIAS_AUTO_F9   (5.7)
#define DEFAULT_GPSBDS_HW_BIAS_AUTO_F9   (16.3)
#define DEFAULT_GPSGAL_HW_BIAS_AUTO_F9   (-1.0)

//ST8100 ISB DEFAULT
#define DEFAULT_GPSBDS_HW_BIAS_AUTO_ST8100   (-2.0)
#define DEFAULT_GPSGAL_HW_BIAS_AUTO_ST8100   (-5.0)

extern double  DEFAULT_DCB_TABLE[10][8];

#define VDR_FEEDBACK_NONE               0x00
#define VDR_FEEDBACK_OK                 0x01
#define VDR_FEEDBACK_ELE                0x02
#define VDR_FEEDBACK_VEL                0x04
#define VDR_FEEDBACK_POS                0x08
#define VDR_FEEDBACK_ODO2GNSSVEL        0x10
#define VDR_FEEDBACK_ACCINE             0x20
#define VDR_FEEDBACK_RTK                0x40
#define VDR_FEEDBACK_HEADING            0x80
#define VDR_FEEDBACK_STATIC             0x100
#define VDR_FEEDBACK_DELTAX             0x200

#define  MAX_SMOOTH_NUM     5

#define BIASDIFF_UPCNT_THRES     30 //bias diff update cnt threshold

#ifdef __cplusplus
extern "C" {
#endif	

typedef struct
{
    uint32_t        epochCnt;              // total count of epoch 
    uint32_t        meas_epochCnt;         // total count of epoch when there is some measurements 
}PE_Context_t;

typedef struct
{
	uint8_t                    posResCnt;
	uint8_t                    largePosResStdCnt;
	uint8_t                    posIsBias;
	double                   posResetTime;
	double                   posResBack[MAX_SMOOTH_NUM];
	double                   smoothPosResStd;
} PosBiasDet_KfRes;

typedef struct
{
	uint8_t                    prDiffBackCnt;
	uint8_t                    prDiffBackCnt_a;
	uint8_t                    prDisBackCnt;
	uint8_t                    largeDiffCnt;
	uint8_t                    posIsBias;
	float                   prDiffBack[MAX_SMOOTH_NUM];
	float                   prDiffBack_a[MAX_SMOOTH_NUM];
	float                   prDisBack[MAX_SMOOTH_NUM];
	float                   smoothPrDiffStd;
	float                   smoothPrDiffStd_a;
	float                   smoothPrDisAvg;
	double                   decEpoch;
} PosBiasDet_PrDiff;

typedef struct
{
	uint8_t                    posIsBias; 
	uint8_t                    abKfStateCnt;
	double                   decEpoch;
} PosBiasDet_KfState;

#if 0
typedef struct
{
	uint8_t                    posIsBias; 
	uint8_t                    detectCnt;
	double                   decEpoch;
} PosBiasDet_GoodMeas;

#endif
typedef struct
{
	uint8_t                    posHasBias; 
	uint8_t                    posRecoverCnt;
	double                   posBiasDecTime;
	PosBiasDet_KfRes      kfResPosBiasDect;
	PosBiasDet_PrDiff     prDiffPosBiasDect;
	PosBiasDet_KfState    kfStatePosBiasDect;
} PeStateMonitor_t;

typedef struct
{
	uint32_t               vdrSysFlag;
	uint8_t                vdrZuptFlag;           /*0 - moving, 1- static*/
	uint8_t                vdrZuptFlagImu;        /*0 - moving, 1- static*/
	//uint32_t             vdrstaticHistory;
	//uint32_t             vdrSysState;
	int64_t                timeStampOfUtc;       /* ms since 1970 */
	float                  fvelEcef[3];
	float                  heading;
	float                  pitch;
	float                  roll;
	double                 dCb2n[9];
	double                 llaPos[3];
	uint8_t                isReverse;
	float                  fOdospeed;
	uint8_t                uOdoValid;
	uint64_t               uInsVelOK;
	float                  fOdo2GnssVelENU[3];
	float                  fAccIne[3];
	float                  fGyro1sDelta;
	float                  fAcc1sDelta;
	uint8_t                ustationary_WhyT2F;
	float                  fDistance_XYZ[3];
	//float                fLastVel[3];
}VDR_Gnss_Feedback_t;


void gnss_Pe_Exec(meas_blk_t* pMeas, asg_obs_t* asg_obs, gnss_FeedbackInsMeasBlock_t* pz_CurrentFeedbackInsMeas);
void gnss_Pe_Init_UsrPos(const USER_PVT* p);
void gnss_Pe_Init_HisInfo(const history_Info* p);
void gnss_Pe_Get_PosFix(USER_PVT* pvt);
USER_PVT* gnss_Pe_Get_User_Pvt();
void gnss_Pe_Dop(meas_blk_t* pMeas,DOP_TYPE* dop,uint8_t flag);
uint8_t gnss_Pe_Dop_Check(meas_blk_t* pMeas, uint8_t idx,uint8_t flag);
void gnss_Pe_Report(const meas_blk_t* pMeas,const Ls_t* pLs,const Kf_t* pKf);
void gnss_Pe_20ms_Jump(meas_blk_t* pMeas);
void gnss_Pe_Bias_Jump(meas_blk_t* pMeas);
void gnss_Pe_Drift_Jump(meas_blk_t* pMeas);
void gnss_Pe_Time_Jump(meas_blk_t* pMeas);
void gnss_Pe_PRDR_Num(meas_blk_t* pMeas);
void gnss_Pe_Get_Kf_Data( Kf_t *pkfData );
uint8_t gnss_Pe_Get_RTD_Status(void);
uint8_t gnss_Pe_Get_SvRtd_Status(uint8_t gnssMode,uint8_t prn);
void gnss_Pe_Get_Ls_Data( Ls_t *plsData );
void gnss_Pe_BiasNum(meas_blk_t* pMeas);
PEReqCommand_t* gnss_Pe_Get_ReqCommand(void);
void gnss_Pe_Set_ReqCommand(uint8_t gnssMode, uint8_t prn, int32_t dataType,PEReqCommand_t* peReqCom);
void gnss_Pe_Full_Agnss_Request();
void gnss_Pe_Set_HW_Bias(GNSS_TIMESYSPARAM* pTimeParam);
uint8_t gnss_Pe_Find_SvMeas(uint8_t gnssMode,uint8_t prn);
Kf_t* gnss_Pe_Get_Kf(void);
void gnss_Pe_Get_RtdIllegalArea_Status(uint8_t* flag);
uint8_t gnss_Pe_illegalArea_Dec(double* llaPos);
void gnss_Pe_Get_PeState(PeStateMonitor_t* p);
void gnss_Pe_Get_HisInfo(history_Info* p);
meas_blk_t* gnss_Pe_Get_Meas(void);
void gnss_pe_fill_fusion_mode(USER_PVT* pvt);
void gnss_Pe_Cal_HorizEllipseUnc(float varX, float varY, float varXY, USER_PVT* pvt);
void gnss_Pe_Pos_Confidence_Set(USER_PVT* pvt);

void gnss_Pe_set_ppk_filter_direction(uint8_t reverse);
uint8_t   gnss_Pe_get_ppk_filter_direction();
void gnss_Pe_PosErr_Convert(USER_PVT* pvt, float* posVar_xy);
void gnss_Pe_Sbas_Proc(sbas_msg_t *psbas);

uint8_t gnss_Pe_Ins_Vel_Check(const VDR_Gnss_Feedback_t*  pFlpFeedInfo, const uint32_t uMode, const USER_PVT *puser);
void gnss_Pe_PosErr_Calc(double* pos_ecef, double* ref_ecef, double* posErr);
void gnss_Pe_DCB_Smooth(Ls_t* pLs, Kf_t* pKf);
void gnss_pe_cal_prdiff_std(meas_blk_t* pMeas);

#define  IS_PPK_REVERSE  (gnss_Pe_get_ppk_filter_direction())

// RTD core message process interface
void gnss_Pe_Meas_Proc(const void* msg, void* user, uint32_t* size);
void gnss_Pe_Iono_Proc(const void* msg);
void gnss_Pe_NavBit_Proc(const void* msg);
void gnss_Pe_NavModel_Proc(const void* msg);
void gnss_Pe_Rtcm_Proc(void* msg);
void gnss_Pe_Init(const Gnss_Cfg_t* cfg);
void gnss_Pe_Close(void);

uint16_t gnss_calc_meas_avg_cn0();
double gnss_calc_algo_epoch_dt(double tor, double last_tor);
USER_PVT* gnss_get_last_pvt();
meas_blk_t* ag_pe_get_meas_unc();
unc_scene_t* ag_pe_get_scene_tag_unc();


#ifdef __cplusplus
}
#endif

#endif
