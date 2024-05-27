/**
 ***************************************************************************
 * Copyright (C), 2016 ASG Corporation.
 *
 * \file	gnss_nmea.h
 * \version 
 * \author  caizhijie
 * \date    2017/12/08 14:36
 * \warning 
 * Email    caizhijie_hit@163.com
 ***************************************************************************
 * \brief   gnss nmea generate string module
 * \attention
 *
 */

#ifndef __GNSS_NMEA_H__
#define __GNSS_NMEA_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define GNSS_NMEA_ENABLE_GGA
#define GNSS_NMEA_ENABLE_GSA
#define GNSS_NMEA_ENABLE_ZDA
#define GNSS_NMEA_ENABLE_GST
//#define GNSS_NMEA_ENABLE_GLL
#define GNSS_NMEA_ENABLE_GSV
#define GNSS_NMEA_ENABLE_RMC
//#define GNSS_NMEA_ENABLE_VTG
#define GNSS_NMEA_ENABLE_OLY
//#define GNSS_NMEA_ENABLE_DISPALY

#define NMEA_STRING_LEN (256)
#define NMEA_GSA_NUM    (3)
#define NMEA_GSV_NUM    (27)

#ifndef PI
#define PI      (double)3.14159265358979323846
#endif

#ifndef DEG2RAD
#define DEG2RAD	((double)PI/(double)180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG	((double)180.0/(double)PI)
#endif

#ifndef NMEA_VEL_KNOT
#define NMEA_VEL_KNOT	(0.5144)
#endif

#ifndef NMEA_VEL_MS2KMH
#define NMEA_VEL_MS2KMH  (3.6)
#endif

#define GNSS_NMEA_MAX_MEASUREMENT (64)

typedef enum {
	Angular_Degree = 0x01,
	Angular_Radian = 0x02
} AngularFormat;

typedef enum {
	GNSS_NMEA_QUALITY_INVALID = 0,
	GNSS_NMEA_QUALITY_GNSS_SPS,
	GNSS_NMEA_QUALITY_DGNSS_SPS,
	GNSS_NMEA_QUALITY_GNSS_PPS,
	GNSS_NMEA_QUALITY_RTK_FIXED,
	GNSS_NMEA_QUALITY_RTK_FLOATING,
	GNSS_NMEA_QUALITY_DR,
	GNSS_NMEA_QUALITY_MANUAL,
	GNSS_NMEA_QUALITY_SIMULATOR
} GNSS_NMEA_QUALITY_INDICATOR_ENUM;

typedef enum {
	GNSS_NMEA_CONSTELLATION_UNKNOWN = 0,
	GNSS_NMEA_CONSTELLATION_GPS = 1,
	GNSS_NMEA_CONSTELLATION_SBAS = 2,
	GNSS_NMEA_CONSTELLATION_GLONASS = 3,
	GNSS_NMEA_CONSTELLATION_QZSS = 4,
	GNSS_NMEA_CONSTELLATION_BEIDOU = 5,
	GNSS_NMEA_CONSTELLATION_GALILEO = 6
} GNSS_NMEA_CONSTELLATION_ENUM;

typedef enum {
	GNSS_NMEA_SV_FLAGS_NONE = 0,
	GNSS_NMEA_SV_FLAGS_HAS_EPHEMERIS_DATA = 1,
	GNSS_NMEA_SV_FLAGS_HAS_ALMANAC_DATA = 2,
	GNSS_NMEA_SV_FLAGS_USED_IN_FIX = 4,
	GNSS_NMEA_SV_FLAGS_HAS_CARRIER_FREQUENCY = 8
} GNSS_NMEA_SATELLITE_ENUM;

typedef struct {
	AngularFormat type;
	double deg;
	double min;
	double rad;
} Angular_t;

typedef struct 
{
	float tm_msec;
	int tm_sec;   // seconds after the minute - [0, 60] including leap second
	int tm_min;   // minutes after the hour - [0, 59]
	int tm_hour;  // hours since midnight - [0, 23]
	int tm_mday;  // day of the month - [1, 31]
	int tm_mon;   // months since January - [0, 11]
	int tm_year;  // years since 1900
	int tm_wday;  // days since Sunday - [0, 6]
	int tm_yday;  // days since January 1 - [0, 365]
	int tm_isdst; // daylight savings time flag
} gnss_nmea_time_t;


typedef struct _gnss_nmea_info_t {
	int8_t gga_num;
	int8_t gsa_num;
	int8_t gll_num;
	int8_t gsv_num;
	int8_t rmc_num;
	int8_t vtg_num;
	int8_t gga[NMEA_STRING_LEN];
	int8_t gsa[NMEA_GSA_NUM][NMEA_STRING_LEN];
	int8_t gll[NMEA_STRING_LEN];
	int8_t gsv[NMEA_GSV_NUM][NMEA_STRING_LEN];
	int8_t rmc[NMEA_STRING_LEN];
	int8_t vtg[NMEA_STRING_LEN];
} gnss_nmea_info_t;

typedef struct {
	/** set to sizeof(GnssSvInfo) */
	unsigned int size;
	/**
	* Pseudo-random number for the SV, or FCN/OSN number for Glonass. The
	* distinction is made by looking at constellation field. Values should be
	* in the range of:
	*
	* - GPS:     1-32
	* - SBAS:    120-151, 183-192
	* - GLONASS: 1-24, the orbital slot number (OSN), if known.  Or, if not:
	*            93-106, the frequency channel number (FCN) (-7 to +6) offset by + 100
	*            i.e. report an FCN of -7 as 93, FCN of 0 as 100, and FCN of +6 as 106.
	* - QZSS:    193-200
	* - Galileo: 1-36
	* - Beidou:  1-37
	*/
	int16_t svid;
	/**
	* Defines the constellation of the given SV. Value should be one of those
	* GNSS_CONSTELLATION_* constants
	*/
	GNSS_NMEA_CONSTELLATION_ENUM constellation;
	/**
	* Carrier-to-noise density in dB-Hz, typically in the range [0, 63].
	* It contains the measured C/N0 value for the signal at the antenna port.
	*
	* This is a mandatory value.
	*/
	float c_n0_dbhz;
	/** Elevation of SV in degrees. */
	float elevation;
	/** Azimuth of SV in degrees. */
	float azimuth;
	/**
	* Contains additional data about the given SV. Value should be one of those
	* GNSS_SV_FLAGS_* constants
	*/
	GNSS_NMEA_SATELLITE_ENUM flags;
} gnss_nmea_sv_info;

typedef struct 
{
	/** set to sizeof(GpsLocation) */
	unsigned int    size;
	/** Contains GpsLocationFlags bits. */
	uint16_t        flags;
	/** Represents latitude in degrees. */
	double          latitude;
	/** Represents longitude in degrees. */
	double          longitude;
	/** Represents altitude in meters above the WGS 84 reference ellipsoid. */
	double          altitude;
	/** Represents speed in meters per second. */
	double          speed;
	/** Represents heading in degrees. */
	double          bearing;
	/** Represents expected accuracy in meters. */
	double          accuracy;
	char			time_flag;/*0:timestamp mode; 1:weektime mode */
	/** Timestamp for the location fix. */
	long long       timestamp;
	/** utc time mode. */
	int				year;
	int				month;
	int				day;
	int				hour;
	int				minute;
	double			second;
	/** Represents position dilution of precision. */
	float			PDOP; 
	/** Represents  horizontal dilution of precision. */
	float			HDOP; 
	/** Represents vertical dilution of precision. */
	float			VDOP;
	/** Represents time dilution of precision. */
	float			TDOP;
	/** Represents gnss result quality indicator. */
	GNSS_NMEA_QUALITY_INDICATOR_ENUM indicator;
	/** Represents in-used satellite in visual . */
	int				sv_inuesd;
    /** average cno for in-used satellite */
    int             avgCNO;
    /** Age of Differential GPS data */
    double          ageOfDiff;
    /** Represents east velocity m/s */
    float           ve;
    /** Represents north velocity m/s */
    float           vn;
    /** Represents up velocity m/s */
    float           vu;
    /** Represents east velocity std err m/s */
    float           ve_err;
    /** Represents north velocity std err m/s */
    float           vn_err;
    /** Represents up velocity std err m/s */
    float           vu_err;
    /** Represents latitude std err */
    float           latitude_err;
    /** Represents lontitude std err */
    float           lontitude_err;
    /** Represents altitude std err */
    float           altitude_err;
            
#ifdef GNSS_NMEA_ENABLE_GSV
	/** Number of GPS SVs currently visible, refers to the SVs stored in sv_list */
	int				num_svs;
	/**
	* Pointer to an array of SVs information for all GNSS constellations,
	* except GPS, which is reported using sv_list
	*/
	gnss_nmea_sv_info sv_list[GNSS_NMEA_MAX_MEASUREMENT];
#endif

} gnss_nmea_result_t;

/**
 *  @brief  
 *	@author caizhijie
 *	@date	2017/12/08 15:34
 */
bool gnss_angular_cov_rad2degree(Angular_t* angular);

/**
 *  @brief  
 *	@author caizhijie
 *	@date	2017/12/08 15:34
 */
bool gnss_angular_cov_degree2rad(Angular_t* angular);

/**
 *  @brief  convert timestamp to struct tm base rtklib time2epoch func
 *	@author caizhijie
 *	@date	2018/03/13 13:27
 */
bool gnss_nmea_convert_timestamp2utc(int64_t timestamp, gnss_nmea_time_t* nmea_time);

/**
 *  @brief  extract gga format nmea string from gnss_nmea_result_t struct
 *	@author caizhijie
 *	@date	2017/12/08 14:48
 */
int32_t gnss_nmea_extract_gga(gnss_nmea_result_t* usr, char* gga, double* quasi_geoid_h);

/**
 *  @brief  extract gsa format nmea string from gnss_nmea_result_t struct
 *	@author caizhijie
 *	@date	2017/12/08 14:48
 */
int32_t gnss_nmea_extract_gsa(gnss_nmea_result_t* usr, char gsa[][NMEA_STRING_LEN], int8_t* gsa_num, int8_t gsa_max);

int32_t gnss_nmea_extract_gst(gnss_nmea_result_t* usr, char* gst, USER_PVT* user_pvt);

int32_t gnss_nmea_extract_gst_bg(gnss_nmea_result_t* usr, char* gst);

/**
 *  @brief  extract gll format nmea string from gnss_nmea_result_t struct
 *	@author caizhijie
 *	@date	2017/12/08 14:48
 */
int32_t gnss_nmea_extract_gll(gnss_nmea_result_t* usr, char* gll);

/**
 *  @brief  extract gsv format nmea string from gnss_nmea_result_t struct
 *	@author caizhijie
 *	@date	2017/12/08 14:48
 */
int32_t gnss_nmea_extract_gsv(gnss_nmea_result_t* usr, char gsv[][NMEA_STRING_LEN], char* gsv_num, char gsv_max);

/**
 *  @brief  extract rmc format nmea string from gnss_nmea_result_t struct
 *	@author caizhijie
 *	@date	2017/12/08 14:48
 */
int32_t gnss_nmea_extract_rmc(gnss_nmea_result_t* usr, char* rmc);

/**
 *  @brief  extract vtg format nmea string from gnss_nmea_result_t struct
 *	@author caizhijie
 *	@date	2017/12/08 14:48
 */
int32_t gnss_nmea_extract_vtg(gnss_nmea_result_t* usr, char* vtg);

/**
 *  @brief  extract zda nmea string from gnss_nmea_result_t
 *	@author caizhijie
 *	@date	2020/09/27 17:16
 */
int32_t gnss_nmea_extract_zda(gnss_nmea_result_t* usr, char* zda);

/**
 *  @brief  extract all nmea string from gnss_nmea_result_t
 *	@author caizhijie
 *	@date	2017/12/12 13:24
 */
bool gnss_nmea_extract_all_info(gnss_nmea_result_t* gnss_user_info, gnss_nmea_info_t* gnss_nmea_info);

/**
 *  @brief  log output display gnss nmea info
 *	@author caizhijie
 *	@date	2017/12/13 14:30
 */
bool gnss_nmea_display_all_info(gnss_nmea_info_t* gnss_nmea_info);

#endif // !__GNSS_NMEA_H__
