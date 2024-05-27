/******************************************************************
* Copyrights(C) 
* All rights Reserved
* �ļ����ƣ�gnss_type.h
* �ļ�����������ļ��ض����������̵ı�������
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�08/08
******************************************************************/
#ifndef __GNSS__TYPE__H__
#define __GNSS__TYPE__H__

#include <time.h>
#include "ag_base.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(_WIN32)
#  define PARAMETER_NOT_USED(p) ((void)(p))
#else
#  define PARAMETER_NOT_USED(p)
#endif

	/* Define the ECEF data */
	typedef struct
	{
		uint8_t    have_spos;
		double   p[3];
		float   v[3];
		float   a[3];
		double   t;
	}ECEF;

	/* Define LLA data */
	typedef struct
	{
		double   lla[3];
	}LLA;

	/* Define ENU velocity */
	typedef struct
	{
		double   v[3];
	}ENU;

	/*  */
	typedef struct
	{
		float pDOP;  /* PDOP */
		float hDOP;  /* HDOP */
		float vDOP;  /* VDOP */
		float tDOP;  /* TDOP */
	} DOPs_t;

	typedef int32_t VehicleContextMode_t;
#define VEHICNTXT_MODE_UNKNOWN				0
#define VEHICNTXT_MODE_RAMP_UP				1
#define VEHICNTXT_MODE_RAMP_DOWN			2
#define VEHICNTXT_MODE_RAMP_FLAT			3
#define VEHICNTXT_MODE_UNDER_GARAGE			4
	
	typedef struct
	{
		double	hor_vel_std;  // meters
		float	VDR_attitude_x; // degrees
		float	VDR_attitude_y; // degrees
		float	VDR_attitude_z; // degrees
		VehicleContextMode_t vehicle_context_mode;
	} FusionResult;

	typedef struct
	{
		float
			p,						/* Total pos error; meters */
			h,						/* Horiz. pos error; meters */
			v,						/* Vert. pos error; meters */
			bias[4],				/* Time bias error; meters */
			drift;
		float 
			ve_err, 				/* East velocity std error, m/s*/
			vn_err, 				/* north velocity std error, m/s */
			vd_err; 				/* down velocity std error, m/s */
		float 
			lat_err,				/* latitude std err, meter */
			lon_err,				/* lontitude std err, meter */
			alt_err;				/* altitude std err, meter */
	} ErrEstimation_t;

	typedef struct
	{
		float               altitude_msl;
		float               velocity_unc;
		float               heading_unc;
		float               ellipse_unc_semi_major;
		float               ellipse_unc_semi_minor;
		float               ellipse_unc_orientation;
		float               north_velocity;
		float               east_velocity;
		float               up_velocity;
		ErrEstimation_t   err_estimate;
	} GnssResultExtra_t;

    /* Accuracy in three Confidence Interval */
    typedef struct _navigation_accuracy_t {
        uint8_t          valid;
        /**
        * Self defined accuracy type related to fusion mode.
        * 0: invalid     1: PPS        2: RTD
        * 4: RTK fix     5: RTK float  60: DR
        * 61: PPS+DR     62: RTD+DR    64: RTK fix+DR
        * 65: RTK float+DR
        */
		uint8_t          accu_type;
        /* 68.26% confidence N/E/U */
        float         accu1_north;
        float         accu1_east;
        float         accu1_up;
        /* 95% confidence N/E/U */
		float         accu2_north;
        float         accu2_east;
        float         accu2_up;
        /* 99.90% confidence N/E/U */
        float         accu3_north;
        float         accu3_east;
        float         accu3_up;
    } navigation_accuracy_t;


	/*fusion type*/
	typedef uint8_t GnssFusionType;
#define FUSION_TYPE_INVALID             0
#define FUSION_TYPE_GNSSONLY            1
#define FUSION_TYPE_DGNSS               2
#define FUSION_TYPE_FUSION              3   /* duplicated. */
#define FUSION_TYPE_DR                  4
#define FUSION_TYPE_CHIP                5
#define FUSION_TYPE_CHIP_FUSION         6
#define FUSION_TYPE_GO_FUSION           8	/* fusion with standalone pvt result. */
#define FUSION_TYPE_DGNSS_FUSION        9
#define FUSION_TYPE_RTK_FIX             10
#define FUSION_TYPE_RTK_FLOAT           11
#define FUSION_TYPE_RTK_PROPAGATE       12
#define FUSION_TYPE_RTK_FIX_FUSION      13
#define FUSION_TYPE_RTK_FLOAT_FUSION    14

	/* Milliseconds since January 1, 1970 */
	typedef uint64_t GnssUtcTime_t;

	/* Represents a location. */
	typedef struct
	{
		/* set to sizeof(GpsLocation) */
		size_t            size;
		/* Source of pvt result */
		int32_t           src;
		/* Contains GpsLocationFlags bits. */
		int16_t           flags;
		/* type for position fusion method. */
		GnssFusionType    fusion_type;
		/* Represents latitude in degrees. */
		double               latitude;
		/* Represents longitude in degrees. */
		double               longitude;
		/* Represents altitude in meters above the WGS 84 reference ellipsoid. */
		double               altitude;
		/* Represents speed in meters per second. */
		float               speed;
		/* Represents heading in degrees. */
		float               bearing;
		/* Represents expected accuracy in meters. */
		float               accuracy;
		/* Timestamp for the location fix. */
		GnssUtcTime_t     timestamp;
		/* DOP */
		DOPs_t            dop;
		/* Average CN0 */
		float               avg_cno;
		/* Number of satellites used for positioning. */
		uint8_t           sat_used;
		/* Confidence */
		navigation_accuracy_t accu;

		GnssResultExtra_t extra;

	} solution_rpt_t;

#define AGGNSS_MAX_FREQ_NUM 2

#ifdef __cplusplus
}
#endif

#endif
