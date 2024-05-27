#ifndef __GNSS__MODE__H__
#define __GNSS__MODE__H__

#include "gnss.h"
#include "gnss_types.h"
#include "gnss_kf.h"

#define TUNNEL_HOLD_NUM         5


#define CONTEXT_INIT_THRES           20

/* Define user context */
#define USER_CONTEXT_UNKNOWN    0
#define USER_CONTEXT_HAND       1
#define USER_CONTEXT_VEHICLE    2
#define USER_CONTEXT_MAX        3

/* Define current user motion status */
#define USER_MOTION_UNKNOWN     0
#define USER_MOTION_STATIC      1
#define USER_MOTION_WALK        2
#define USER_MOTION_RUN         3
#define USER_MOTION_DRIVE       4
#define USER_MOTION_MAX         5

#define STATIC_THRES            0.2
#define WALK_THRES              0.55
#define RUN_THRES               3.0
#define DRIVE_THRES             5.5

#ifdef __cplusplus
extern "C" {
#endif

	/* tunnel mode struct */
	typedef struct
	{
		uint8_t     mode;
		uint8_t     last_mode;
		uint8_t     detectCnt;
		uint8_t     exitTunnelFlag;
		uint8_t     tunnel_althold;
		double    exitTunnelTime;
	} TUNNEL_MODE;

	/* Indoor mode struct */
	typedef struct
	{
		uint8_t     mode;
		uint8_t     indoorEnterCnt;
		uint8_t     indoorExitState;
		uint8_t     indoorExitCnt_1;
		uint8_t     indoorExitCnt_2;
	} INDOOR_MODE;

	/* User dynamic context */
	typedef struct
	{
		uint8_t     inital;
		uint8_t     context;
		uint8_t     isVehicleMode;
		uint8_t     motion;
		uint8_t     motionCnt[USER_MOTION_MAX];
		uint32_t    epochCnt;
		uint32_t    contextCnt[USER_CONTEXT_MAX];
		uint32_t    contextEpochCnt;
	} USER_MOTION;

	/* User scenario */
	typedef struct
	{
		uint8_t     isOpenSky : 1;
		uint8_t     isDownTown : 1;
		uint8_t     isUnderEleRoad : 1;
		uint8_t     isLowSpeed : 1;
		uint8_t     isPDRMode : 1;
		uint8_t     isWALKMODE : 1;
		uint8_t     downtownCnt;
		uint8_t     noDowntownCnt;
		uint8_t     LowAvgCnocnt;
		uint8_t     diffBackCnt;
		uint8_t     prDrdiffBackCnt;
		float       prDiffStdBack[5];
		float       smoothDiffStd;
		float       prdrDiffStdBack[2];
		float       smoothPrDrDiffStd;
	} USER_SCENARIO;
	/*FeedBack Flag*/
	typedef struct
	{
		uint8_t uVelVdrFeedback : 1;
		uint8_t uPosVdrFeedback : 1;
		uint8_t uAccVdrFeedback : 1;
		uint8_t uEleVdrFeedback : 1;
		uint8_t uDeltxVdrFeedback : 1;
		uint8_t uHeadingValid : 1;
		uint8_t uStaticVdrfeedback : 1;
		uint8_t uOdoVdrfeedback : 1;
	}VDR_FEEDBACKFLAG;

	typedef struct
	{
		uint8_t              threeGppMode;
		TUNNEL_MODE          tunnelMode;
		INDOOR_MODE          indoorMode;
		USER_MOTION          userContext;
		STATIC_DATA          staticData;
		USER_SCENARIO        userSceData;
		VDR_FEEDBACKFLAG     feedbackFlag;
	} PE_MODES;


	void gnss_Mode_Init(PE_MODES* pMode);
	void gnss_Mode_Detection(meas_blk_t* pMeas,PE_MODES* pMode,KF_PVT_INFO* kf_pvt);
	void  gnss_Static_Mode_Detection(Kf_t* p_Kf, PE_MODES* pMode);

#ifdef __cplusplus
}
#endif


#endif