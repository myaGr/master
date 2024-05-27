/**
 ***************************************************************************
 * Copyright (C), 2016 ASG Corporation.
 *
 * \file	gnss_nmea.c
 * \version 
 * \author  caizhijie
 * \date    12/08 14:34
 * \warning 
 * Email    caizhijie_hit@163.com
 ***************************************************************************
 * \brief   gnss nmea generate string module 
 * \attention
 *
 */

#include "gnss.h"
#include "gnss_nmea.h"
#include <stdio.h>
#include <float.h>
#include <math.h>

#define AG_NMEA_MAX_LEN   (256)

char gst_body[256]={'\0'};

/**
 *  @brief  convert radian system to degree system 
 *	@author caizhijie
 *	@date	12/08 15:34
 */
bool gnss_angular_cov_rad2degree(Angular_t* angular)
{
	double deg = 0.0f;

	if (angular == NULL) 
	{
		return false;
	}

	if ((angular->type&Angular_Radian)!= Angular_Radian)
	{
		return false;
	}
	
	if (angular->rad < 0)
	{
		angular->rad = (-1)*angular->rad;
	}

	deg = angular->rad * RAD2DEG;
	angular->deg = (float)((int)(deg));
	angular->min = 60*(deg - angular->deg);
	angular->type = (AngularFormat)(angular->type |Angular_Radian);


	return true;
}

/**
 *  @brief  convert degree system to radian system 
 *	@author caizhijie
 *	@date	12/08 15:34
 */
bool gnss_angular_cov_degree2rad(Angular_t* angular)
{
	double deg = 0.0f;

	if (angular == NULL)
	{
		return false;
	}

	if ((angular->type&Angular_Degree) != Angular_Degree)
	{
		return false;
	}

	deg = angular->deg + angular->min / 60.0f;
	angular->rad = deg * DEG2RAD;
	angular->type = (AngularFormat)(angular->type |Angular_Degree);

	return true;
}

/**
 *  @brief  
 *	@author caizhijie
 *	@date	12/11 11:33
 */
bool gnss_nmea_add_checksum(char* nmea_str)
{
	uint32_t    i = 0;
	uint32_t    nmea_str_length = 0;
	uint8_t     sys_check;

	/* inputs check */
	if (NULL == nmea_str) {
		return false;
	}

	/* cal the sys check */
	nmea_str_length = (int)strlen(nmea_str);

	/* check the len */
	if (nmea_str_length > AG_NMEA_MAX_LEN - 5) {
		return false;
	}

	sys_check = nmea_str[1];
	for (i = 2; i < nmea_str_length; i++)
	{
		sys_check ^= nmea_str[i];
	}

	sprintf(&nmea_str[nmea_str_length], "%1c%02X%c%c", '*', sys_check, 0x0D, 0x0A);

	return true;
}


/**
 *  @brief  convert timestamp to struct tm base rtklib time2epoch func
 *	@author caizhijie
 *	@date	03/13 13:27
 */
bool gnss_nmea_convert_timestamp2utc(int64_t timestamp, gnss_nmea_time_t* nmea_time)
{
	float ep[6] = { 0 };
	int64_t t_time = timestamp / 1000;
	float t_sec = (float)(0.001 * (float)(timestamp % 1000));

	const int mday[] = { /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};

	int days = 0;
	int sec = 0;
	int mon = 0;
	int day = 0;

	if ((nmea_time == NULL))
	{
		return false;
	}

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t_time / 86400);
	sec = (int)(t_time - (int64_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++) {
		if (day >= mday[mon]) day -= mday[mon]; else break;
	}
	ep[0] = 1970 + (float)days / 1461 * 4 + (float)mon / 12;
	ep[1] = (float)(mon % 12 + 1);
	ep[2] = (float)(day + 1);
	ep[3] = (float)sec / 3600;
	ep[4] = (float)(sec % 3600) / 60;
	ep[5] = (float)(sec % 60 + t_sec);

	nmea_time->tm_year = (int)ep[0];
	nmea_time->tm_mon = (int)ep[1];
	nmea_time->tm_mday = (int)ep[2];
	nmea_time->tm_hour = (int)ep[3];
	nmea_time->tm_min = (int)ep[4];
	nmea_time->tm_sec = (int)ep[5];
	nmea_time->tm_msec = 0;

	return true;
}

void gnss_nmea_convert_utc(gnss_nmea_result_t* usr, gnss_nmea_time_t* nmea_time)
{
	if (usr->time_flag == 0)
	{
		gnss_nmea_convert_timestamp2utc(usr->timestamp, nmea_time);
	}
	else if (usr->time_flag == 1)
	{
		nmea_time->tm_hour = usr->hour;
		nmea_time->tm_min = usr->minute;
		nmea_time->tm_sec = (int)usr->second;
		nmea_time->tm_mday = usr->day;
		nmea_time->tm_mon = usr->month;
		nmea_time->tm_year = usr->year;
		nmea_time->tm_msec =(float)(usr->second - usr->second);
	}

    /* append time carry a number in adding when during confine of as 59s 59m 23h case */
    if (nmea_time->tm_msec >= 0.995) {
		nmea_time->tm_msec = 0.0;
		nmea_time->tm_sec++;
	}
    if (nmea_time->tm_sec >= 60) {
        nmea_time->tm_sec = 0;
        nmea_time->tm_min++;
    }
    if (nmea_time->tm_min >= 60) {
        nmea_time->tm_min = 0;
        nmea_time->tm_hour++;
    }
    if (nmea_time->tm_hour >= 24) {
        nmea_time->tm_hour = 0;
        nmea_time->tm_mday++;
    }
    switch (nmea_time->tm_mon)
    {
       case 1:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 2:
       {
            if((nmea_time->tm_year%4==0&&nmea_time->tm_year%100!=0)||(nmea_time->tm_year%400==0))
            {
                if (nmea_time->tm_mday >= 30) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}
            }
            else
            {
                if (nmea_time->tm_mday >= 29) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}
            }
            break;
       }
       case 3:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 4:if (nmea_time->tm_mday >= 31) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 5:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 6:if (nmea_time->tm_mday >= 31) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 7:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 8:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 9:if (nmea_time->tm_mday >= 31) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 10:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 11:if (nmea_time->tm_mday >= 31) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       case 12:if (nmea_time->tm_mday >= 32) {nmea_time->tm_mday = 1;nmea_time->tm_mon++;}break;
       default:break;
    }
    if (nmea_time->tm_mon >= 13)
    {
        nmea_time->tm_mon = 1;
        nmea_time->tm_year++;
    }

}

/**
 *  @brief  extract gga format nmea string from GnssUserInfo_t struct
 *	@author caizhijie
 *	@date	12/08 14:48
 */
int32_t gnss_nmea_extract_gga(gnss_nmea_result_t* usr, char* gga, double* quasi_geoid_h)
{
	if (quasi_geoid_h)
	{
		*quasi_geoid_h = 0.0;
	}
#ifdef GNSS_NMEA_ENABLE_GGA

	Angular_t latitude;
	Angular_t longitude;
	gnss_nmea_time_t nmea_time = { 0 };
	char lat_flag = '\0';
	char lon_flag = '\0';
    char ageOfDiffStr[256] = {0};

	double pos[3] = { usr->latitude, usr->longitude, usr->altitude };
	double geoid_corr = geoidh(pos);
	if (quasi_geoid_h)
	{
		*quasi_geoid_h = geoid_corr;
	}

	gnss_nmea_convert_utc(usr, &nmea_time);

    memset(&latitude,0,sizeof(latitude));
    memset(&longitude,0,sizeof(longitude));

	latitude.rad = usr->latitude;  latitude.type = (AngularFormat)(latitude.type | Angular_Radian);
	longitude.rad = usr->longitude; longitude.type = (AngularFormat)(longitude.type |Angular_Radian);

	lat_flag = (latitude.rad >= 0) ? 'N' : 'S';
	lon_flag = (longitude.rad >= 0) ? 'E' : 'W';
	gnss_angular_cov_rad2degree(&latitude);
	gnss_angular_cov_rad2degree(&longitude);
        
	if ((usr->indicator == GNSS_NMEA_QUALITY_GNSS_SPS) || (usr->indicator == GNSS_NMEA_QUALITY_DR))
	{
		sprintf(ageOfDiffStr, "");
	}
	else if ((usr->indicator == GNSS_NMEA_QUALITY_DGNSS_SPS) || 
			 (usr->indicator == GNSS_NMEA_QUALITY_RTK_FIXED) || 
			 (usr->indicator == GNSS_NMEA_QUALITY_RTK_FLOATING))
	{
		sprintf(ageOfDiffStr, "%.1f", (usr->ageOfDiff < 3600.0 && usr->ageOfDiff >= 0.0) ? usr->ageOfDiff : 0.0);
	}

	if (usr->indicator == GNSS_NMEA_QUALITY_INVALID)
	{
		if ((nmea_time.tm_year == 0) && (nmea_time.tm_mon == 0) && (nmea_time.tm_mday == 0))
		{
			sprintf((char *)gga, "$GNGGA,,,,,,0,,,,,,,,");
		}
		else
		{
			sprintf((char *)gga, "$GNGGA,%02d%02d%02d.%02d,,,,,0,,,,,,,,",
				nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100);
		}
	}
	else if ((latitude.deg == 0) && (latitude.min == 0) && (longitude.deg == 0) && (longitude.min == 0))
	{
		sprintf((char *)gga, "$GNGGA,%02d%02d%02d.%02d,,,,,0,,,,,,,,",
			nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100);
	}
	else
	{
		sprintf((char *)gga, "$GNGGA,%02d%02d%02d.%02d,%02d%09.6f,%c,%03d%09.6f,%c,%d,%02d,%2.1f,%2.3f,M,%2.3f,M,%s,",
			nmea_time.tm_hour, nmea_time.tm_min, 
		    nmea_time.tm_sec, (int)(nmea_time.tm_msec*100+0.5)%100,
			(int32_t)latitude.deg, latitude.min, lat_flag,
			(int32_t)longitude.deg, longitude.min, lon_flag,
			usr->indicator,
			usr->sv_inuesd,
			usr->HDOP,
			usr->altitude - geoid_corr,
			geoid_corr,
			ageOfDiffStr);
	}
	gnss_nmea_add_checksum((char *)gga);

#endif

	return true;
}

int32_t gnss_nmea_extract_gst(gnss_nmea_result_t* usr, char* gst, USER_PVT* user_pvt)
{
#ifdef GNSS_NMEA_ENABLE_GST

	gnss_nmea_time_t nmea_time = { 0 };

	float accuracy = (float)(user_pvt->accuracy > 0.1 ? user_pvt->accuracy : 0.1);
	float ellipseUncSemiMajor =(float) (user_pvt->ellipseUncSemiMajor > 0.1 ? user_pvt->ellipseUncSemiMajor : 0.1);
	float ellipseUncSemiMinor = (float)(user_pvt->ellipseUncSemiMinor > 0.1 ? user_pvt->ellipseUncSemiMinor : 0.1);
	float ellipseUncOrientation = (float)user_pvt->ellipseUncOrientation;
	float lat_err = (float)(user_pvt->posErr.lat_err > 0.1 ? user_pvt->posErr.lat_err : 0.1);
	float lon_err = (float)(user_pvt->posErr.lon_err > 0.1 ? user_pvt->posErr.lon_err : 0.1);
	float alt_err = (float)(user_pvt->posErr.alt_err > 0.1 ? user_pvt->posErr.alt_err : 0.1);

	gnss_nmea_convert_utc(usr, &nmea_time);

	memset(gst_body, 0, sizeof(gst_body));

	if (usr->indicator == GNSS_NMEA_QUALITY_INVALID)
	{
		if ((nmea_time.tm_year == 0) && (nmea_time.tm_mon == 0) && (nmea_time.tm_mday == 0))
		{
			sprintf((char*)gst, "$GPGST,,,,,,,,");
		}
		else
		{
			sprintf((char*)gst, "$GPGST,%02d%02d%02d.%02d,,,,,,,",
				nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100);
		}
		sprintf(gst_body, ",,,,,,");
	}
	else
	{
		sprintf((char*)gst, "$GPGST,%02d%02d%02d.%02d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
			nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100,
			accuracy, ellipseUncSemiMajor, ellipseUncSemiMinor, ellipseUncOrientation,
			lat_err, lon_err, alt_err);

		sprintf(gst_body, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", accuracy, ellipseUncSemiMajor,
			ellipseUncSemiMinor, ellipseUncOrientation, lat_err, lon_err, alt_err);
	}
	gnss_nmea_add_checksum(gst);

#endif
	return true;
}

int32_t gnss_nmea_extract_gst_bg(gnss_nmea_result_t* usr, char* gst)
{
#ifdef GNSS_NMEA_ENABLE_GST

	gnss_nmea_time_t nmea_time = { 0 };

	gnss_nmea_convert_utc(usr, &nmea_time);

	sprintf(gst, "$GPGST,%02d%02d%02d.%02d,%s",
		nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100, gst_body);

	gnss_nmea_add_checksum(gst);

#endif
	return 1;
}

/**
 *  @brief  extract gsa format nmea string from GnssUserInfo_t struct
 *	@author caizhijie
 *	@date	12/08 14:48
 */
int32_t gnss_nmea_extract_gsa(gnss_nmea_result_t* usr, char gsa[][NMEA_STRING_LEN], int8_t* gsa_num, int8_t gsa_max)
{
#ifdef GNSS_NMEA_ENABLE_GSA

	int8_t gps_sv_ch_num = 0;
	int8_t gln_sv_ch_num = 0;
	int8_t bds_sv_ch_num = 0;
	int8_t gal_sv_ch_num = 0;
	int8_t qzs_sv_ch_num = 0;

	char gsa_fix_mode = usr->indicator == GNSS_NMEA_QUALITY_INVALID ? 1 : 3;

	char gps_gsa[NMEA_STRING_LEN] = { 0 };
	char gln_gsa[NMEA_STRING_LEN] = { 0 };
	char bds_gsa[NMEA_STRING_LEN] = { 0 };
	char gal_gsa[NMEA_STRING_LEN] = { 0 };
	char qzs_gsa[NMEA_STRING_LEN] = { 0 };

	sprintf(gps_gsa, "$GPGSA,A,%d,", gsa_fix_mode);
	sprintf(gln_gsa, "$GLGSA,A,%d,", gsa_fix_mode);
	sprintf(bds_gsa, "$BDGSA,A,%d,", gsa_fix_mode);
	sprintf(gal_gsa, "$GAGSA,A,%d,", gsa_fix_mode);
	sprintf(qzs_gsa, "$QZGSA,A,%d,", gsa_fix_mode);

	for (int i = 0; i < usr->num_svs; i++)
	{	
		if ((usr->sv_list[i].flags & GNSS_NMEA_SV_FLAGS_USED_IN_FIX) == 0)
		{
			continue;
		}

		if (usr->sv_list[i].constellation == GNSS_NMEA_CONSTELLATION_GPS)
		{
			char temp[16];
			sprintf(temp, "%02d,", usr->sv_list[i].svid);
			strcat(gps_gsa, temp);
			gps_sv_ch_num++;
		}

		if (usr->sv_list[i].constellation == GNSS_NMEA_CONSTELLATION_GLONASS)
		{
			char temp[16];
			sprintf(temp, "%02d,", usr->sv_list[i].svid);
			strcat(gln_gsa, temp);
			gln_sv_ch_num++;
		}

		if (usr->sv_list[i].constellation == GNSS_NMEA_CONSTELLATION_BEIDOU)
		{
			char temp[16];
			sprintf(temp, "%02d,", usr->sv_list[i].svid);
			strcat(bds_gsa, temp);
			bds_sv_ch_num++;
		}

		if (usr->sv_list[i].constellation == GNSS_NMEA_CONSTELLATION_GALILEO)
		{
			char temp[16];
			sprintf(temp, "%02d,", usr->sv_list[i].svid);
			strcat(gal_gsa, temp);
			gal_sv_ch_num++;
		}

		if (usr->sv_list[i].constellation == GNSS_NMEA_CONSTELLATION_QZSS)
		{
			char temp[16];
			sprintf(temp, "%02d,", usr->sv_list[i].svid);
			strcat(qzs_gsa, temp);
			qzs_sv_ch_num++;
		}
	}

	for (int i = gps_sv_ch_num; i < 12; i++)
	{
		strcat(gps_gsa, ",");
	}

	for (int i = gln_sv_ch_num; i < 12; i++)
	{
		strcat(gln_gsa, ",");
	}

	for (int i = bds_sv_ch_num; i < 12; i++)
	{
		strcat(bds_gsa, ",");
	}

	for (int i = gal_sv_ch_num; i < 12; i++)
	{
		strcat(gal_gsa, ",");
	}

	for (int i = qzs_sv_ch_num; i < 12; i++)
	{
		strcat(qzs_gsa, ",");
	}

	char dop_str[64] = { 0 };
	sprintf(dop_str, "%2.1f,%2.1f,%2.1f", usr->PDOP, usr->HDOP, usr->VDOP);
	strcat(gps_gsa, dop_str);
	strcat(gln_gsa, dop_str);
	strcat(bds_gsa, dop_str);
	strcat(gal_gsa, dop_str);
	strcat(qzs_gsa, dop_str);

	gnss_nmea_add_checksum(gps_gsa);
	gnss_nmea_add_checksum(gln_gsa);
	gnss_nmea_add_checksum(bds_gsa);
	gnss_nmea_add_checksum(gal_gsa);
	gnss_nmea_add_checksum(qzs_gsa);

	if (gps_sv_ch_num != 0 && *gsa_num < gsa_max)
	{
		memcpy(gsa[*gsa_num], gps_gsa, NMEA_STRING_LEN * sizeof(char));
		(*gsa_num)++;
	}

	if (gln_sv_ch_num != 0 && *gsa_num < gsa_max)
	{
		memcpy(gsa[*gsa_num], gln_gsa, NMEA_STRING_LEN * sizeof(char));
		(*gsa_num)++;
	}

	if (bds_sv_ch_num != 0 && *gsa_num < gsa_max)
	{
		memcpy(gsa[*gsa_num], bds_gsa, NMEA_STRING_LEN * sizeof(char));
		(*gsa_num)++;
	}

	if (gal_sv_ch_num != 0 && *gsa_num < gsa_max)
	{
		memcpy(gsa[*gsa_num], gal_gsa, NMEA_STRING_LEN * sizeof(char));
		(*gsa_num)++;
	}

	if (qzs_sv_ch_num != 0 && *gsa_num < gsa_max)
	{
		memcpy(gsa[*gsa_num], qzs_gsa, NMEA_STRING_LEN * sizeof(char));
		(*gsa_num)++;
	}
#endif

	return 1;
}

/**
 *  @brief  extract gll format nmea string from GnssUserInfo_t struct
 *	@author caizhijie
 *	@date	12/08 14:48
 */
int32_t gnss_nmea_extract_gll(gnss_nmea_result_t* usr, char* gll)
{
#ifdef GNSS_NMEA_ENABLE_GLL

	Angular_t latitude = { 0 };
	Angular_t longitude = { 0 };
	gnss_nmea_time_t nmea_time = { 0 };

	gnss_nmea_convert_timestamp2utc(usr->timestamp, &nmea_time);

	char lat_flag = (latitude.rad >= 0) ? 'N' : 'S';
	char lon_flag = (longitude.rad >= 0) ? 'E' : 'W';
	latitude.rad = usr->latitude;  latitude.type |= Angular_Radian;
	longitude.rad = usr->longitude; longitude.type |= Angular_Radian;
	gnss_angular_cov_rad2degree(&latitude);
	gnss_angular_cov_rad2degree(&longitude);

	char gll_status = 'V';
	char gll_mode_indicator = 'V';

	switch (usr->indicator)
	{
	case GNSS_NMEA_QUALITY_INVALID:		gll_mode_indicator = 'N'; gll_status = 'V'; break;
	case GNSS_NMEA_QUALITY_GNSS_SPS:		gll_mode_indicator = 'A'; gll_status = 'V'; break;
	case GNSS_NMEA_QUALITY_DGNSS_SPS:	gll_mode_indicator = 'D'; gll_status = 'D'; break;
	case GNSS_NMEA_QUALITY_GNSS_PPS:		gll_mode_indicator = 'A'; gll_status = 'A'; break;
	case GNSS_NMEA_QUALITY_RTK_FIXED:	gll_mode_indicator = 'D'; gll_status = 'D'; break;
	case GNSS_NMEA_QUALITY_RTK_FLOATING:	gll_mode_indicator = 'D'; gll_status = 'D'; break;
	case GNSS_NMEA_QUALITY_DR:			gll_mode_indicator = 'E'; gll_status = 'A'; break;
	case GNSS_NMEA_QUALITY_MANUAL:		gll_mode_indicator = 'M'; gll_status = 'A'; break;
	case GNSS_NMEA_QUALITY_SIMULATOR:	gll_mode_indicator = 'S'; gll_status = 'A'; break;
	default:gll_mode_indicator = 'N'; gll_status = 'V'; break;
	}
	
	if (usr->indicator == GNSS_NMEA_QUALITY_INVALID)
	{
		sprintf(gll, "$GPGLL,,,,,,V,N");
	}
	else if ((nmea_time.tm_year == 0)&&(nmea_time.tm_mon == 0)&&(nmea_time.tm_mday == 0))
	{
		sprintf(gll, "$GPGLL,,,,,,V,N");
	}
	else
	{
		sprintf(gll, "$GPGLL,%02d%09.6f,%c,%02d%09.6f,%c,%02d%02d%02d.%02d,%c,%c",
			(int32_t)latitude.deg, latitude.min, lat_flag,
			(int32_t)longitude.deg, longitude.min, lon_flag,
			nmea_time.tm_hour, nmea_time.tm_min, 
			nmea_time.tm_sec, (int)(nmea_time.tm_msec*100+0.5)%100,
			gll_status,
			gll_mode_indicator);
	}
	gnss_nmea_add_checksum(gll);
#endif

	return 1;
}
int32_t extract_one_gnss_nmea_gsv(GNSS_NMEA_CONSTELLATION_ENUM sys_type, gnss_nmea_result_t* usr, char gsv[][NMEA_STRING_LEN], char* gsv_num, char gsv_max)
{
	int sys_gsv_num = 0;
	int sys_sv_total = 0;
	int sys_sv_index = 0;
	char gsv_type_str[20] = { '\0' };
	switch (sys_type)
	{
	case GNSS_NMEA_CONSTELLATION_UNKNOWN: { 				break; }
	case GNSS_NMEA_CONSTELLATION_GPS: {sprintf(gsv_type_str, "$GPGSV"); break; }
	case GNSS_NMEA_CONSTELLATION_SBAS: {/*add your code here*/	break; }
	case GNSS_NMEA_CONSTELLATION_GLONASS: {sprintf(gsv_type_str, "$GLGSV"); break; }
	case GNSS_NMEA_CONSTELLATION_QZSS: {sprintf(gsv_type_str, "$QZGSV"); break; }
	case GNSS_NMEA_CONSTELLATION_BEIDOU: {sprintf(gsv_type_str, "$BDGSV"); break; }
	case GNSS_NMEA_CONSTELLATION_GALILEO: {	sprintf(gsv_type_str, "$GAGSV");break; }
	default: {				break; }
	}
	int i = 0;
	if (NULL == usr || NULL == gsv || NULL == gsv_num || *gsv_num<0)
	{
		return 0;
	}
	for (i = 0; i < usr->num_svs; ++i)
	{
		if (GNSS_NMEA_CONSTELLATION_UNKNOWN == usr->sv_list[i].constellation)
		{
			continue;
		}
		if (sys_type == usr->sv_list[i].constellation)
		{
			++sys_sv_total;
		}
	}
	sys_gsv_num = (sys_sv_total == 0) ? 0 : (sys_sv_total - 1) / 4 + 1;
	if (*gsv_num + sys_gsv_num > gsv_max)
	{
		sys_gsv_num = gsv_max - *gsv_num;
	}
	for (i = 0; i < sys_gsv_num; i++) 
	{
		memset(gsv[*gsv_num + i], 0, NMEA_STRING_LEN * sizeof(char));
		sprintf(gsv[*gsv_num + i], "%s,%d,%d,%d", gsv_type_str, sys_gsv_num, i + 1, sys_sv_total);
	}
	/* fill svid cn0 ele azi info */
	for (i = 0; i < usr->num_svs && i < GNSS_NMEA_MAX_MEASUREMENT; i++)
	{
		if (GNSS_NMEA_CONSTELLATION_UNKNOWN == usr->sv_list[i].constellation || sys_type != usr->sv_list[i].constellation)
		{
			continue;
		}
		if (*gsv_num + (sys_sv_index / 4) >= gsv_max)
		{
			break;
		}
		char* str_ptr = gsv[*gsv_num + (sys_sv_index / 4)];
		sys_sv_index++;

		char temp[16] = { '\0' };
		if (usr->sv_list[i].elevation != 0.0f) 
		{
			sprintf(temp, ",%d,%d,%d,%d", usr->sv_list[i].svid, (int)usr->sv_list[i].elevation,
				(int)usr->sv_list[i].azimuth, (int)usr->sv_list[i].c_n0_dbhz);
		}
		else 
		{
			sprintf(temp, ",%d,,,%d", usr->sv_list[i].svid, (int)usr->sv_list[i].c_n0_dbhz);
		}
		strcat(str_ptr, temp);
	}
	for (i = 0; i < sys_gsv_num; i++)
	{
		gnss_nmea_add_checksum(gsv[*gsv_num + i]);
	}
	(*gsv_num) += sys_gsv_num;
	return 1;
}
/**
 *  @brief  extract gsv format nmea string from GnssUserInfo_t struct
 *	@author caizhijie
 *	@date	12/08 14:49
 */
int32_t gnss_nmea_extract_gsv(gnss_nmea_result_t* usr, char gsv[][NMEA_STRING_LEN], char* gsv_num, char gsv_max)
{
#ifdef GNSS_NMEA_ENABLE_GSV
	extract_one_gnss_nmea_gsv(GNSS_NMEA_CONSTELLATION_GPS, usr, gsv, gsv_num, gsv_max);
	extract_one_gnss_nmea_gsv(GNSS_NMEA_CONSTELLATION_GLONASS, usr, gsv, gsv_num, gsv_max);
	extract_one_gnss_nmea_gsv(GNSS_NMEA_CONSTELLATION_BEIDOU, usr, gsv, gsv_num, gsv_max);
	extract_one_gnss_nmea_gsv(GNSS_NMEA_CONSTELLATION_GALILEO, usr, gsv, gsv_num, gsv_max);
	extract_one_gnss_nmea_gsv(GNSS_NMEA_CONSTELLATION_QZSS, usr, gsv, gsv_num, gsv_max);
#endif
	return 1;
}

/**
 *  @brief  extract rmc format nmea string from GnssUserInfo_t struct
 *	@author caizhijie
 *	@date	12/08 14:49
 */
int32_t gnss_nmea_extract_rmc(gnss_nmea_result_t* usr, char* rmc)
{
#ifdef GNSS_NMEA_ENABLE_RMC

	Angular_t latitude;
	Angular_t longitude;
	gnss_nmea_time_t nmea_time = { 0 };
	char lat_flag = '\0';
	char lon_flag = '\0';
	char rmc_mode_indicator = '\0';

    memset(&latitude,0,sizeof(latitude));
    memset(&longitude,0,sizeof(longitude));

	gnss_nmea_convert_utc(usr, &nmea_time);

	latitude.rad = usr->latitude;  latitude.type = (AngularFormat)(latitude.type | Angular_Radian);
	longitude.rad = usr->longitude; longitude.type = (AngularFormat)(longitude.type |Angular_Radian);
	lat_flag = (latitude.rad >= 0) ? 'N' : 'S';
	lon_flag = (longitude.rad >= 0) ? 'E' : 'W';
	gnss_angular_cov_rad2degree(&latitude);
	gnss_angular_cov_rad2degree(&longitude);

	rmc_mode_indicator = 'D';
	switch (usr->indicator)
	{
	case GNSS_NMEA_QUALITY_INVALID:	 rmc_mode_indicator = 'V'; break;
	case GNSS_NMEA_QUALITY_GNSS_SPS:  rmc_mode_indicator = 'A'; break;
	case GNSS_NMEA_QUALITY_DGNSS_SPS: rmc_mode_indicator = 'D'; break;
	case GNSS_NMEA_QUALITY_GNSS_PPS:  rmc_mode_indicator = 'P'; break;
	case GNSS_NMEA_QUALITY_RTK_FIXED: rmc_mode_indicator = 'R'; break;
	case GNSS_NMEA_QUALITY_RTK_FLOATING:rmc_mode_indicator = 'F'; break;
	case GNSS_NMEA_QUALITY_DR:		 rmc_mode_indicator = 'E'; break;
	case GNSS_NMEA_QUALITY_MANUAL:	 rmc_mode_indicator = 'M'; break;
	case GNSS_NMEA_QUALITY_SIMULATOR: rmc_mode_indicator = 'S'; break;
	default:break;
	}

	if (usr->indicator == GNSS_NMEA_QUALITY_INVALID)
	{
		if ((nmea_time.tm_year == 0) && (nmea_time.tm_mon == 0) && (nmea_time.tm_mday == 0))
		{
			sprintf((char *)rmc, "$GNRMC,,V,,,,,,,,,,N");
		}
		else
		{
			sprintf((char *)rmc, "$GNRMC,%02d%02d%02d.%02d,V,,,,,,,%02d%02d%02d,,,%c",
				nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100,
				nmea_time.tm_mday, nmea_time.tm_mon, nmea_time.tm_year % 100, rmc_mode_indicator);
		}
	}
	else if ((latitude.deg == 0) && (latitude.min == 0) && (longitude.deg == 0) && (longitude.min == 0))
	{
		sprintf((char *)rmc, "$GNRMC,%02d%02d%02d.%02d,V,,,,,,,%02d%02d%02d,,,%c",
			nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100,
			nmea_time.tm_mday, nmea_time.tm_mon, nmea_time.tm_year % 100, rmc_mode_indicator);
	}
	else
	{
		sprintf((char *)rmc, "$GNRMC,%02d%02d%02d.%02d,A,%02d%09.6f,%c,%03d%09.6f,%c,%2.1f,%2.1f,%02d%02d%02d,,,%c",
			nmea_time.tm_hour, nmea_time.tm_min, 
			nmea_time.tm_sec, (int)(nmea_time.tm_msec*100+0.5)%100,
			(int32_t)latitude.deg, latitude.min, lat_flag,
			(int32_t)longitude.deg, longitude.min, lon_flag,
			usr->speed,
			usr->bearing,
			nmea_time.tm_mday, nmea_time.tm_mon, nmea_time.tm_year % 100, rmc_mode_indicator);
	}
	gnss_nmea_add_checksum((char *)rmc);

#endif

	return 1;
}

/**
 *  @brief  extract gga format nmea string from GnssUserInfo_t struct
 *	@author caizhijie
 *	@date	12/08 14:49
 */
int32_t gnss_nmea_extract_vtg(gnss_nmea_result_t* usr, char* vtg)
{
#ifdef GNSS_NMEA_ENABLE_VTG

	char vtg_mode_indicator = 'N';
	switch (usr->indicator)
	{
	case GNSS_NMEA_QUALITY_INVALID:	vtg_mode_indicator = 'N'; break;
	case GNSS_NMEA_QUALITY_GNSS_SPS:  vtg_mode_indicator = 'A'; break;
	case GNSS_NMEA_QUALITY_DGNSS_SPS: vtg_mode_indicator = 'D'; break;
	case GNSS_NMEA_QUALITY_GNSS_PPS:  vtg_mode_indicator = 'P'; break;
	case GNSS_NMEA_QUALITY_RTK_FIXED: vtg_mode_indicator = 'D'; break;
	case GNSS_NMEA_QUALITY_RTK_FLOATING:vtg_mode_indicator = 'D'; break;
	case GNSS_NMEA_QUALITY_DR:		vtg_mode_indicator = 'E'; break;
	case GNSS_NMEA_QUALITY_MANUAL:	vtg_mode_indicator = 'M'; break;
	case GNSS_NMEA_QUALITY_SIMULATOR:	vtg_mode_indicator = 'S'; break;
	default:break;
	}

	if (usr->indicator == GNSS_NMEA_QUALITY_INVALID)
	{
		sprintf(vtg, "$GPVTG,,T,,M,,N,,K,N");
	}
	else
	{
		sprintf(vtg, "$GPVTG,%07.3f,T,,M,%05.1f,N,%06.1f,K,%c",
			usr->bearing,
			usr->speed,
			usr->speed * NMEA_VEL_KNOT * NMEA_VEL_MS2KMH,
			vtg_mode_indicator);
	}
	gnss_nmea_add_checksum(vtg);
#else
    (void)usr;
    (void)vtg;
#endif

	return 1;
}

int32_t gnss_nmea_extract_zda(gnss_nmea_result_t* usr, char* zda)
{
#ifdef GNSS_NMEA_ENABLE_ZDA

	gnss_nmea_time_t nmea_time = { 0 };
	gnss_nmea_convert_utc(usr, &nmea_time);
	int local_zone = -8;

	if (usr->indicator == GNSS_NMEA_QUALITY_INVALID) {
		sprintf((char*)zda, "$GNZDA,,,,,%02d,00", local_zone);
	}
	else {
		sprintf((char*)zda, "$GNZDA,%02d%02d%02d.%02d,%02d,%02d,%04d,%02d,00",
			nmea_time.tm_hour, nmea_time.tm_min, nmea_time.tm_sec, (int)(nmea_time.tm_msec * 100 + 0.5) % 100,
			nmea_time.tm_mday, nmea_time.tm_mon, nmea_time.tm_year, local_zone);
	}

	gnss_nmea_add_checksum((char*)zda);
#endif

	return true;
}



/**
 *  @brief  extract all nmea string from GnssUserInfo_t
 *	@author caizhijie
 *	@date	12/12 13:24
 */
bool gnss_nmea_extract_all_info(gnss_nmea_result_t* gnss_user_info, gnss_nmea_info_t* gnss_nmea_info)
{
	char rmc[NMEA_STRING_LEN] = { 0 };
	char gga[NMEA_STRING_LEN] = { 0 };
	char gll[NMEA_STRING_LEN] = { 0 };
	char gsa[NMEA_GSA_NUM][NMEA_STRING_LEN] = { {0} };
	int8_t gsa_num = 0;
	char gsv[NMEA_GSV_NUM][NMEA_STRING_LEN] = { {0} };
	char gsv_num = 0;
	char vtg[NMEA_STRING_LEN] = { 0 };
	double quasi_geoid_h = 0.0;

	if ((gnss_user_info == NULL) || (gnss_nmea_info == NULL))
	{
		return false;
	}
	
	if (gnss_nmea_extract_rmc(gnss_user_info, rmc)) 
	{
		memcpy(&gnss_nmea_info->rmc, rmc, NMEA_STRING_LEN*sizeof(char));
		gnss_nmea_info->rmc_num++;
	}
	if (gnss_nmea_extract_gga(gnss_user_info, gga,&quasi_geoid_h))
	{
		memcpy(&gnss_nmea_info->gga, gga, NMEA_STRING_LEN * sizeof(char));
		gnss_nmea_info->gga_num++;
	}
	
	if (gnss_nmea_extract_gll(gnss_user_info, gll))
	{
		memcpy(&gnss_nmea_info->gll, gll, NMEA_STRING_LEN * sizeof(char));
		gnss_nmea_info->gll_num++;
	}
	
	if (gnss_nmea_extract_gsa(gnss_user_info, gsa, &gsa_num, NMEA_GSA_NUM))
	{
		memcpy(gnss_nmea_info->gsa, gsa, NMEA_GSA_NUM * NMEA_STRING_LEN * sizeof(char));
		gnss_nmea_info->gsa_num = gsa_num;
	}
	
	if (gnss_nmea_extract_gsv(gnss_user_info, gsv, &gsv_num, NMEA_GSV_NUM))
	{
		memcpy(gnss_nmea_info->gsv, gsv, NMEA_GSV_NUM * NMEA_STRING_LEN * sizeof(char));
		gnss_nmea_info->gsv_num = gsv_num;
	}
	
	if (gnss_nmea_extract_vtg(gnss_user_info, vtg))
	{
		memcpy(&gnss_nmea_info->vtg, vtg, NMEA_STRING_LEN * sizeof(char));
		gnss_nmea_info->vtg_num++;
	}

	return true;
}


