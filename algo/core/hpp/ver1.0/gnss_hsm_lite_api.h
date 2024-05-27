/**
 ***************************************************************************
 * Copyright (C), 2019 ASG Corporation.
 *
 * \file	gnss_hsm_lite_api.h
 * \version
 * \author  caizhijie
 * \date    2019/07/17 10:54:26
 * \warning
 ***************************************************************************
 * \brief
 * \attention
 *
 */

#include <stdint.h>

#ifndef __GNSS_HSM_LITE_API_H__
#define __GNSS_HSM_LITE_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gnss_def.h"
#include "gnss.h"
#include "asg_rtcm_decode.h"
#include "gnss_tdcp.h"

#define tag_asg            "[ASG_RAW]"

#define tag_gnss_meas		"gnss_obs"
#define tag_gnss_meas_start "gnss_obs_start"
#define tag_gnss_meas_end	"gnss_obs_stop"

#define tag_gnss_ref		"ref_obs"
#define tag_gnss_ref_sta	"ref_obs_sta"
#define tag_gnss_ref_start 	"ref_obs_start"
#define tag_gnss_ref_end	"ref_obs_stop"

#define tag_gnss_raw		"raw_obs"
#define tag_gnss_raw_start 	"raw_obs_start"
#define tag_gnss_raw_end	"raw_obs_stop"

#define tag_CTL             "$EACTL"
#define tag_RTM             "$EARTM"
#define tag_NAV             "$NAVGP"
#define tag_ION             "$NAVIO"
#define tag_OBS             "$EAOBS"
#define tag_LOC             "$EALOC"
#define tag_DCB             "$EADCB"
#define tag_IMU             "$GPIMU"
#define tag_PPS             "$GPPPS"
#define tag_OLY             "$GPOLY"
#define tag_PEI             "$EAPEI"
#define tag_FBC             "$EAFBC"

#ifdef PLAYBACK_MODE
#define primitive_LOG_ENABLE
#else
//#define nmea_LOG_ENABLE
#endif

typedef struct
{
	uint8_t             available;
	uint8_t             vdrZuptFlag;             // ******** bit sort form low to high  1:flpZuptFlag   2:flpZuptFlagImu   3~6:ustationary_WhyT2F
	uint8_t				uOdoValid;               // ******** bit sort form low to high  1:uOdoValid   3~4:isReverse
	uint32_t			uInsVelOK;
	int64_t             timeStampOfUtc;          
	float               velEcef[3];                //5    Number of decimal places
	float               heading;                  //7
	float               pitch;                    //7
	float               roll;                     //7
	float				fOdo2GnssVelENU[3];       //5
	float               fAccIne[3];               //5
	float				fGyro1sDelta;             //7
	float				fAcc1sDelta;              //7
	double	        	llaPos[3];                //10
	float 				fDistance_XYZ[3];         //7
}  asg_flp_feed_back_t;
	
typedef struct _asg_extern_gnss_info_t {
	uint8_t pos_flag; // 1:No Fix,2:Fix 2D,3:Fix 3D,4:RTK Fixed,5:RTK Float,6:PPS
	int leapsecond;
	int week;
	double tow;
	double lla_pos[3];
	double enu_vel[3];
	double clock_drift;
	double clock_offset;
	int32_t meas_year;
	int32_t meas_month;
	int32_t meas_day;
	int32_t meas_hours;
	int32_t meas_mins;
	int32_t meas_secs;
	int32_t meas_msecs;
} asg_extern_gnss_info_t;

typedef struct {
	void  (*log_callback)(uint8_t* log_buffer, size_t length);
	void  (*log_nmea_callback)(uint8_t* log_buffer, size_t length);
	void* (*get_eph_callback)(uint8_t gnssMode, uint8_t prn, gtime_t time);
	void* (*get_iono_callback)(uint8_t gnssMode);
	uint32_t (*time_callback)();
	void (*mutex_lock)(void);
	void (*mutex_unlock)(void);
} asg_extern_callback_api_t;

typedef struct {
	uint8_t	meas_rate;
} asg_init_parameter_t;

void gnss_hsm_lite_server_config(uint8_t active, uint8_t scenario);

void gnss_hsm_lite_api_init(asg_extern_callback_api_t* cb_api, asg_init_parameter_t* para);

void gnss_hsm_lite_api_deinit();

void gnss_hsm_lite_api_eph_status_reset();

void gnss_hsm_lite_api_eph_status_set(uint8_t gnssMode, uint8_t sv_id, uint8_t status);

uint8_t gnss_hsm_lite_api_eph_status_get(uint8_t gnssMode, uint8_t sv_id);

void gnss_hsm_lite_api_inject_extern_gnss_info(asg_extern_gnss_info_t* gnss_info);

void gnss_hsm_lite_api_inject_flp_feed_back_info(asg_flp_feed_back_t* feed_back_info);

void gnss_hsm_lite_api_inject_ephemeris(uint8_t gnssMode, uint8_t sv_id, void* eph);

void gnss_hsm_lite_api_inject_iono(uint8_t gnssMode, void* eph);

void gnss_hsm_lite_api_inject_measurement(meas_blk_t* measurement);

void gnss_hsm_lite_api_inject_dgnss(asg_rtcm_lite_t* data);

void gnss_hsm_lite_api_start_algo(asg_nmea_info_t* nmea_info, asg_epoch_data_tdcp* tdcp_data);

void* gnss_hsm_lite_api_get_eph(uint8_t gnssMode, uint8_t prn, gtime_t time);

void* gnss_hsm_lite_api_get_iono(uint8_t gnssMode);

void gnss_hsm_lite_set_dcb_param(double dcb_param[], uint8_t row, uint8_t cols);

void* gnss_hsm_lite_malloc(int size);

void gnss_hsm_lite_free(void* ptr);

USER_PVT* gnss_hsm_lite_get_pvt();

char* gnss_hsm_lite_getver();

double gnss_hsm_lite_get_gps_time(int* week);

uint8_t gnss_hsm_satid2satchannel(uint32_t satid, uint8_t* gnssMode, uint8_t* freq_index);

uint32_t gnss_hsm_satchannel2satid(uint8_t prn, uint8_t gnssMode, uint8_t freq_index);

uint32_t gnss_hsm_satchannel2code(uint8_t prn, uint8_t gnssMode, uint8_t freq_index);

uint8_t calc_checksum(char* str, int len);

bool    add_checksum(char* str);

void  gnss_ephemeris_insert(int gnssMode, int prn, void* src);

void* gnss_ephemeris_get(int gnssMode, int prn, gtime_t time);



#ifdef __cplusplus
}
#endif

#endif
