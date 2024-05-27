/**@file        gnss_common.h
 * @brief       Common GNSS funtions
 * @details     1. Coordinate, Time, Frequency convert functions 
 * @author      caizhijie
 * @date        2022/04/25
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __GNSS_COMMON_H__
#define __GNSS_COMMON_H__

#include "cmn_def.h"
#include "gnss_def.h"
#include "gnss_type.h"
#include "integ_type.h"
#include "loc_core_api.h"

BEGIN_DECL

extern const char system_char[C_GNSS_MAX][8];

/**
 * @brief Get the wavelength of a signal
 * @param[in] signal type
 * @return Signal's wavelength
 */
double wavelength(gnss_SignalType signal);

/**
 * @brief Convert a signal type to frequency type
 * @param[in] signal type
 * @return frequency type
 */
gnss_FreqType gnss_cvt_Sig2FreqType(gnss_SignalType signal);

/**
 * @brief Convert frequency type to amb type
 * @param[in] frequency type
 * @return gnss_fixedAmbType
 */
gnss_fixedAmbType gnss_cvt_FreqType2AmbType(gnss_FreqType freq);

/**
 * @brief Convert a signal type to constellation type
 * @param[in] signal type
 * @return Constellation type
 */
gnss_ConstellationType gnss_cvt_Signal2Constellation(gnss_SignalType signal);

/**
* @brief Convert internal constellation type to Loc engine constellation type
* @param[in]   u_ConstellationType - Input constellation type
* @return      u_type - Output loc constellation type
*/
loc_api_gnssConstellationType gnss_cvt_ConstellationTypeToLoc(gnss_ConstellationType u_ConstellationType);

/**
 * @brief Convert a signal type to code string
 * @param[in] signal type
 * @param[out] signal str
 * @return none
 */
void gnss_cvt_Sig2CodeStr(gnss_SignalType signal, char* str);

/**
 * @brief Get the ionospheric coefficient base the L1 frequency
 * @param[in] constellation
 * @return the frequency of  base L1
 */
double gnss_BaseL1Freq(uint8_t constellation);

/**
 * @brief Get the ionospheric coefficient base the L1 frequency
 * @param[in] constellation
 * @param[in] signal
 * @return the ionospheric coefficient base the L1 frequency
 */
double gnss_ionoCoefBaseL1Freq(uint8_t constellation, gnss_SignalType signal);

/**
 * @brief Convert a frequency type to frequency used in the algorithm
 * @param[in] z_f frequency type
 * @return frequency used in the algorithm
 */
algo_useFreq gnss_freqType2Algo(gnss_FreqType z_f);

/**
 * @brief Get satellite type from svid
 * @param[in] constellation
 * @param[in] svid
 * @return satellite type
 */
gnss_SatelliteType svid2SatType(uint8_t constellation, uint8_t svid);

/**
 * @brief Convert a constellation type to constellation used in the algorithm
 * @param[in] z_constellation constellation type
 * @return constellation used in the algorithm
 */
algo_useConstellation gnss_satType2Algo(gnss_ConstellationType z_c);

/**
 * @brief         get more than one min member's index in arr[]
 * @param[in]     pd_arr[] the double array
 * @param[in]     u_size size of arr[]
 * @param[out]    pu_min_indices[] num_min min index in arr[]
 * @param[in]     u_num_min number of min
 * @return 0: success, other: fail
 */
uint8_t cmn_find_min_indices(double pd_arr[], uint8_t u_size, int16_t pu_min_indices[], uint8_t u_num_min);

/**
 * @brief descending sort the float array
 * @param[in] pf_data,float array,it will be sort by ascending
 * @param[in] q_num,the number of float array
 * @return void
 */
int gnss_descSortFloat(float* pf_data, uint32_t q_num);

/**
 * @brief ascending sort the float array
 * @param[in] pf_data,float array,it will be sorted by ascending
 * @param[in] q_num,the number of float array
 * @return void
 */
void gnss_ascSortFloat(float* pf_data, uint32_t q_num);
/**
 * @brief ascending sort the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @return void
 */
void gnss_ascSortDouble(double* pd_data, uint32_t q_num);
/**
 * @brief get the median value from the float array
 * @param[in] pf_data,float array,it will be sorted by ascending
 * @param[in] q_num,the number of float array
 * @param[out] pf_median,the median value of float array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_ascSortMedianFloat(const float* pf_data, uint32_t q_num, float* pf_median);
/**
 * @brief get the median value from the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @param[out] pd_median,the median value of double array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_ascSortMedianDouble(const double* pd_data, uint32_t q_num, double* pd_median);
/**
 * @brief get the median value from the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @param[out] pd_median,the median value of double array
 * @param[out] pd_std,the std value of double array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_ascSortMedianStdDouble(const double* pd_data, uint32_t q_num, double* pd_median, double* pd_std);
/**
 * @brief MAD method for the double array
 * @param[in] pd_data,double array
 * @param[in] q_num,the number of double array
 * @param[out] pd_dataAbs,the abs of minus median value of double array
 * @param[out] pd_median,the median value of pd_data array
 * @param[out] pd_medianAbs,the median value of pd_dataAbs array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_MadDouble(const double* pd_data, uint32_t q_num, double* pd_dataAbs, double* pd_median, double* pd_medianAbs);
/**
 * @brief MAD method for the float array
 * @param[in] pf_data,float array
 * @param[in] q_num,the number of float array
 * @param[out] pf_dataAbs,the abs of minus median value of float array
 * @param[out] pf_median,the median value of pf_dataAbs array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_MadFloat(const float* pf_data, uint32_t q_num, float* pf_dataAbs, float* pf_median);

/*----------------------------------------------------------------------------*/
/* coordinates transformation ------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief transform ecef position to geodetic position
 * @param[in] r ecef position {x,y,z} (m)
 * @param[out] pos geodetic position {lat,lon,h} (rad,m)
 */
void gnss_Ecef2Lla(const double* ecef, double* lla);

/**
 * @brief transform geodetic position to ecef position
 * @param[in] pos geodetic position {lat,lon,h} (rad,m)
 * @param[out] r ecef position {x,y,z} (m)
 */
void gnss_Lla2Ecef(const double* pos, double* ecef);

/**
 * @brief compute ecef to local coordinate transfromation matrix
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[out] E ecef to local coord transformation matrix (3x3)
 * notes: matirix stored by column-major order (fortran convention)
 */
void gnss_Pos2EnuMat(const double* pos, double* E);

/**
 * @brief compute ecef to local coordinate transfromation matrix
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[out] E ecef to local coord transformation matrix (3x3)
 * notes: matirix stored by column-major order (fortran convention)
 */
void gnss_Pos2EnuMat_row(const double* pos, double* E);

/**
 * @brief transform ecef vector to local tangental coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] r vector in ecef coordinate {x,y,z}
 * @param[out] e vector in local tangental coordinate {e,n,u}
 */
void gnss_Ecef2Enu(const double* pos, const double* r, double* e);

/**
 * @brief transform ecef vector to local tangental coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] r vector in ecef coordinate {x,y,z}
 * @param[out] e vector in local tangental coordinate {e,n,u}
 */
void gnss_Ecef2Enu_edit(const double* pos, double* r, double* e);

/**
 * @brief transform local tangental coordinate vector to ecef
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] e vector in local tangental coordinate {e,n,u}
 * @param[out] r vector in ecef coordinate {x,y,z}
 */
void gnss_Enu2Ecef(const double* pos, const double* e, double* r);

/**
 * @brief transform ecef covariance to local tangental coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] P covariance in ecef coordinate
 * @param[out] Q covariance in local tangental coordinate
 */
void gnss_CovEcef2Enu(const double* pos, const double* P, double* Q);

/**
 * @brief transform local enu covariance to xyz-ecef coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[out] Q covariance in local enu coordinate
 * @param[out] P covariance in xyz-ecef coordinate
 */
void gnss_CovEnu2Ecef(const double* pos, const double* Q, double* P);

/**
 * @brief Compute horizontal speed(m/s) from input east and north velocity
 * @param[in] f_VelE - east velocity(m/s)
 * @param[in] f_VelN - north velocity(m/s)
 * @return   computed horizontal speed
 */
float gnss_CovEnVel2Speed(float f_VelE, float f_VelN);

/**
 * @brief eci to ecef transformation matrix
 * @param[in] gpst
 * @param[in] erpv erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * @param[out] U eci to ecef transformation matrix (3 x 3)
 * @param[in/out] gmst greenwich mean sidereal time (rad)
 *                     (NULL: no output)
 * @return none
 * note   : see ref [3] chap 5
 *          TODO: not thread-safe
 */
void gnss_Eci2Ecef(const GpsTime_t* gpst, const double* erpv, double* U, double* gmst);

/**
 * @brief get sun and moon position in ecef
 * @param[in] gpst
 * @param[in] erpv erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * @param[in/out] rsun sun position in ecef  (m) (NULL: not output)
 * @param[in/out] rmoon moon position in ecef (m) (NULL: not output)
 * @param[out] gmst
 */
void gnss_SunMoonPos(const GpsTime_t* gpst, const double* erpv, double* rsun, double* rmoon, double* gmst);

/**
 * @brief convert coordinate from ITRF to WGS84
 * @param[in] gpst tor
 * @param[in] pd_pos_itrf ITRF
 * @param[out] pd_pos_wgs84 WGS84
 * @return None
 */
void gnss_ITRF2WGS84(const GpsTime_t* gpst, const double* pd_pos_itrf, double* pd_pos_wgs84);

/*----------------------------------------------------------------------------*/
/* matrix routines and math --------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief inner product
 * @param a[in]
 * @param b[in] vector a,b (n x 1)
 * @param n[in] size of vector a,b
 * @return
 */
double gnss_Dot(const double* a, const double* b, int n);

/**
 * @brief euclid norm
 * @param a[in] vector a (n x 1)
 * @param n[in] size of vector a
 * @return || a ||
 */
double gnss_Norm(const double* a, int n);

/**
 * @brief Normalize vector
 * @param[in] n
 * @param[in] src vector a (n x 1)
 * @param[out] dst normlized vector (n x 1) || b || = 1
 */
BOOL gnss_UnitVector(const double *src, double *dst, int n);

/**
 * @brief Outer product of 3d vectors
 * @param[in] a
 * @param[in] b vector a,b (3 x 1)
 * @param[out] c outer product (a x b) (3 x 1)
 */
void gnss_CrossMultiply(const double* a, const double* b, double* c);

/**
 * @brief multiply matrix, C = alpha*A*B + beta*C
 * @param[in] tr 'TT'/'TN'/'NT'; each one for matrix A and B; T: transposition;
 * @param[in] n
 * @param[in] k A: n*k
 * @param[in] m B: k*m
 * @param[in] alpha
 * @param[in] A
 * @param[in] B
 * @param[in] beta
 * @param[in/out] C
 */
void gnss_MatrixMultiply(const char* tr, int n, int k, int m, double alpha,
  const double* A, const double* B, double beta, double* C);

/**
 * @brief keep decimal place
 * @param[in] val
 * @param[in] decimal
 * @return vaule that kepp decimal place
 */
double gnss_Round(double a, int decimal);

/**
 * @brief mean double
 * @param[in] val
 * @param[in] n
 * @return
*/
double gnss_MeanDouble(const double* val, uint16_t n);

/**
 * @brief mean float
 * @param[in] val
 * @param[in] n
 * @return
*/
extern float gnss_MeanFloat(const float* val, uint16_t n);

/**
 * @brief Check the input time is aligned with second
 * @param[in] t_fullMsec - Full Millisecond start at UTC 1970-01-01 00:00:00, unit: 1ms
 * @param[in] pq_preSecondAlignMsec - Previous aligned time
 * @param[in] f_AlignInterval - Interval setting
 * @return TRUE - align with second
 *         FALSE - not align with second
 */
uint8_t gnss_CheckMeasSecondAlign(uint64_t t_fullMsec,
  uint64_t* pq_preSecondAlignMsec, float f_AlignInterval);

/**
 * @brief  Check the input time is overrun second, (q_TowMsec - *pq_preSecondAlignMsec) >=f_AlignInterval
 * @param[in] t_fullMsec - Full Millisecond start at UTC 1970-01-01 00:00:00, unit: 1ms
 * @param[in] pq_preSecondAlignMsec - Previous aligned time
 * @param[in] f_AlignInterval - Interval setting
 * @return TRUE - align with second
 *         FALSE - not align with second
 */
uint8_t gnss_CheckMeasSecondOverrun(uint64_t t_fullMsec,
  uint64_t* pq_preSecondAlignMsec, float f_AlignInterval);

/**
 * @brief convert the satellite coordinate from the transmit time to receiver time
 * @param[in] pd_siteEcef the site coordinate in ECEF 
 * @param[in] pd_satEcef the satellite coordinate of transmit time in ECEF
 * @param[out] pd_satEcefRot the satellite coordinate of receiver time in ECEF
 * @return void
 */
void gnss_satRot(const double pd_siteEcef[3], const double pd_satEcef[3], double pd_satEcefRot[3]);

/**
 * @brief satellite azimuth/elevation angle
 * @param pos geodetic position {lat,lon,h} (rad,m)
 * @param e receiver-to-satellilte unit vevtor (ecef)
 * @param azel azimuth/elevation {az,el} (rad) (NULL: no output)
 *                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
 */
extern double gnss_Satazel(const double *pos, const double *e, float* azel);

/**
 * @brief calculate the unit vector from satellite to the site
 * @param[in] pd_siteEcef the site coordinate in ECEF
 * @param[in] pd_satEcef the satellite coordinate of receiver time in ECEF
 * @param[out] pd_unitVector the unit vector of receiver time in ECEF
 * @return the distance from satellite to site
 */
double gnss_unitVector(const double pd_siteEcef[3], const double pd_satEcef[3], double pd_unitVector[3]);
/*----------------------------------------------------------------------------*/
/* time and string functions -------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief initilize the GPS Time
 * @param[in] GpsTime_t*
 * @return void
 */
void tm_initGpsTime(GpsTime_t* pz_gpsTime);
/**
 * @brief Convert Epoch time to time stamp millisecond
 * @param[in] epoch
 * @return t_UnixMsec - unix time stamp millisecond
 */
uint64_t tm_cvt_EpochToUnixMsec(const EpochTime_t* pz_epoch);

/**
 * @brief Convert Epoch time to utc time
 * @param[in] pz_epoch
 * @param[out] pz_utcTime
 */
void tm_cvt_EpochToUtcTime(const EpochTime_t* pz_epoch, UtcTime_t* pz_utcTime);

/**
 * @brief Convert unix time stamp millisecond to Epoch time
 * @param[in] t_UnixMsec
 * @return Epoch time
 */
void tm_cvt_UnixMsecToEpoch(uint64_t t_UnixMsec, EpochTime_t* pz_epoch);

/**
 * @brief Convert Epoch time to utc time
 * @param[in] t_UnixMsec
 * @param[out] pz_utcTime
 */
void tm_cvt_UnixMsecToUtcTime(uint64_t t_UnixMsec, UtcTime_t* pz_utcTime);

/**
 * @brief Convert utc time to Epoch time
 * @param[in] pz_utcTime
 * @param[out] pz_epoch
 */
void tm_cvt_UtcTimeToEpoch(const UtcTime_t* pz_utcTime, EpochTime_t* pz_epoch);

/**
 * @brief Convert Utc time to Unix Millsecond
 * @param[in] pz_utcTime
 * @return t_UnixMsec - unix time stamp millisecond
 */
uint64_t tm_cvt_UtcTimeToUnixMsec(UtcTime_t* pz_utcTime);

/**
 * @brief Convert Utc time to GPST
 * @param[in] pz_utcTime
 * @return
 */
void tm_cvt_UtcTimeToGpst(UtcTime_t* pz_utcTime, GpsTime_t* gpst);

/**
 * @brief Insert a new latest leap second record into list table
 * @param[in] year
 * @param[in] month
 * @param[in] leap_second
 * @return TRUE - Insert Success
 *         FALSE - Insert Fail
 */
uint8_t tm_LeapSecond_Insert(uint16_t year, uint16_t month, uint16_t leap_second);

/**
 * @brief Get Current Leap Second from a input epoch time
 * @param[in] epoch
 * @return leap_second - the current leap second
 */
uint8_t tm_LeapSecond_FromEpoch(const EpochTime_t* z_epoch);

/**
 * @brief Get Current Leap Second from a input time stamp millisecond
 * @param[in] t_unixMsec
 * @return leap_second - the current leap second
 */
uint8_t tm_LeapSecond_FromUnixMsec(uint64_t t_unixMsec);

/**
 * @brief Convert Full Millisecond to unix time stamp millisecond
 * @param[in] t_FullMsec
 * @return time stamp millisecond
 */
uint64_t tm_cvt_FullMsecToUnixMsec(uint64_t t_FullMsec);

/**
 * @brief Convert unix time stamp millisecond to Full Millisecond
 * @param[in] t_unixMsec
 * @return Full Millisecond
 */
uint64_t tm_cvt_UnixMsecToFullMsec(uint64_t t_unixMsec);

/**
 * @brief Convert GPS week and tow to full GNSS millisecond
 * @param[in] w_GpsWeek - GPS Week
 * @param[in] d_GpsTow - GPS tow
 * @param[out] t_FullMsec - Full GNSS Millisecond
 * @return
 */
uint64_t tm_cvt_GpsWeekTowToFullMsec(uint16_t w_GpsWeek, uint32_t q_GpsTowMsec);

/**
 * @brief Convert full GNSS second to GPS week and tow
 * @param[in] w_time - integer of full GNSS second
 * @param[in] d_sec - decimal of full GNSS second
 * @param[out] w_GpsWeek - GPS Week
 * @param[out] d_GpsTow - GPS tow
 * @return
 */
void tm_cvt_FullMsecToGpsWeekTow(uint32_t w_time, double d_sec, uint16_t* w_GpsWeek, double* q_GpsTow);

/**
 * @brief Convert GPS time to GAL Time
 * @param[in] gpst - GPS time structure
 * @param[out] gst - GAL time structure
 * @return
 */
void tm_cvt_GpstToGst(GpsTime_t* gpst, GalTime_t* gst);

/**
 * @brief Convert GAL time to GPS Time
 * @param[in] gst - GAL time structure
 * @param[out] gpst - GPS time structure
 * @return
 */
void tm_cvt_GstToGpst(GalTime_t* gst, GpsTime_t* gpst);

/**
 * @brief Convert GPS time to BDS Time
 * @param[in] gpst - GPS time structure
 * @param[out] bdt - BDS time structure
 * @return
 */
void tm_cvt_GpstToBdt(GpsTime_t* gpst, BdsTime_t* bdt);

/**
 * @brief Convert BDS time to GPS Time
 * @param[in] bdt - BDS time structure
 * @param[out] gpst - GPS time structure
 * @return
 */
void tm_cvt_BdtToGpst(BdsTime_t* bdt, GpsTime_t* gpst);

/**
 * @brief Set a GPS Time by input GPS week and tow
 * @param[in] w_GpsWeek - GPS Week
 * @param[in] d_GpsTow - GPS tow
 * @param[out] gpst - GPS Time Struct
 * @return
 */
void tm_cvt_SetGpst(GpsTime_t* gpst, uint16_t w_GpsWeek, double d_GpsTow);

/**
 * @brief Set a GAL Time by input GAL week and tow
 * @param[in] w_GalWeek - GAL Week
 * @param[in] d_GalTow - GAL tow
 * @param[out] gst - GAL Time Struct
 * @return
 */
void tm_cvt_SetGst(GalTime_t* gst, uint16_t w_GalWeek, double d_GalTow);

/**
 * @brief Set a BDS Time by input BDS week and tow
 * @param[in] w_BDSWeek - BDS Week
 * @param[in] d_BDSTow - BDS tow
 * @param[out] bdt - BDS Time Struct
 * @return
 */
void tm_cvt_SetBdt(BdsTime_t* bdt, uint16_t w_BdsWeek, double d_BdsTow);

/**
 * @brief Convert GPS Time to Utc time
 * @param[in] gpst - GPS Time Week
 * @return UtcTime_t
 */
void tm_cvt_GpstToUtc(const GpsTime_t* gpst, UtcTime_t* pz_utcTime);

/**
 * @brief Convert GAL Time to Utc time
 * @param[in] gst - GAL Time Week
 * @return UtcTime_t
 */
void tm_cvt_GstToUtc(GalTime_t* gst, UtcTime_t* pz_utcTime);

/**
 * @brief Convert BDS Time to Utc time
 * @param[in] bdt - BDS Time Week
 * @return UtcTime_t
 */
void tm_cvt_BdtToUtc(BdsTime_t* bdt, UtcTime_t* pz_utcTime);

/**
 * @brief Convert GPS Time to Epoch time
 * @param[in] gpst - GPS Time Week
 * @return EpochTime_t
 */
void tm_cvt_GpstToEpoch(const GpsTime_t* gpst, EpochTime_t* pz_epoch);

/**
 * @brief Convert GAL Time to Epoch time
 * @param[in] gst - GAL Time Week
 * @return EpochTime_t
 */
void tm_cvt_GstToEpoch(GalTime_t* gst, EpochTime_t* pz_epoch);

/**
 * @brief Convert BDS Time to Epoch time
 * @param[in] bdt - BDS Time Week
 * @return EpochTime_t
 */
void tm_cvt_BdtToEpoch(BdsTime_t* bdt, EpochTime_t* pz_epoch);

/**
 * @brief Add Gpst with t seccond
 * @param[in] gpst - GPS time
 * @param[in] t - Delta second
 * @return
 */
void tm_GpstTimeAdd(GpsTime_t* gpst, double t);

/**
 * @brief Add Gst with t seccond
 * @param[in] gst - GAL time
 * @param[in] t - Delta second
 * @return
 */
void tm_GstTimeAdd(GalTime_t* gst, double t);

/**
 * @brief Add Bdt with t seccond
 * @param[in] Bdt - BDS time
 * @param[in] t - Delta second
 * @return
 */
void tm_BdtTimeAdd(BdsTime_t* bdt, double t);

/**
 * @brief Calculate the difference of gpst1 and gpst2
 * @param[in] gpst1
 * @param[in] gpst2
 * @return
 */
double tm_GpsTimeDiff(const GpsTime_t* gpst1, const GpsTime_t* gpst2);

/**
 * @brief Add Utc time with t seccond
 * @param[in] gpst - GPS time
 * @param[in] t - Delta second
 * @return
 */
void tm_UtcTimeAdd(UtcTime_t* utc, double t);

/**
 * @brief Calculate the difference of utc1 and utc2
 * @param[in] utc1
 * @param[in] utc2
 * @return difference
 */
double tm_UtcTimeDiff(const UtcTime_t* utc1, const UtcTime_t* utc2);

/**
 * @brief Get mjd (Modified Julian Day)
 * @param t GPST
 * @return seconds
 */
long double tm_GetMJD(const GpsTime_t* gpst);

/**
 * @brief Get jd (Julian Day)
 * @param t GPST
 * @return seconds
 */
long double tm_GetJD(const GpsTime_t* t);

/**
 * @brief convert gpst to gmst (Greenwich mean sidereal time)
 * @param[in] gpst
 * @param[in] ut1_utc
 * @return gmst (rad)
 */
double tm_cvt_GpstToGmst(GpsTime_t* gpst);

/**
 * @brief Map svid and constellation to index of measurement buffer
 * @param[in] svid
 * @param[in] constellation
 * @return idx, 0 ~ ALL_GNSS_SYS_SV_NUMBER-1: success
                ALL_GNSS_SYS_SV_NUMBER:fail
 */
uint32_t gnss_cvt_Svid2SvIndex(uint8_t svid, uint8_t constellation);

/**
 * @brief sat prn and system from index
 * @param[in] idx
 * @param[out] sys NULL: no output
 * @return sat prn, -1:error
 */
uint8_t gnss_cvt_SvIndex2Svid(uint8_t idx, uint8_t *sys);

/**
 * @brief judge the satellite whether belong to BDS3
 * @param[in] svid
 * @param[in] constellation
 * @return FALSE represent not belong to BDS3 and TRUE represent belong to BDS3
 */
BOOL gnss_isBDS3Sat(uint8_t svid, uint8_t constellation);

/**
 * @brief signal ID -> index
 * @param[in] signal
 * @return index, MAX_GNSS_SIGNAL_FREQ:error
 */

uint8_t gnss_Signal2FreqIdx(uint8_t signal);
/**
 * @brief index -> signal ID
 * @param[in] index
 * @param[in] sys
 * @return signal, 0 ~ C_GNSS_SIG_MAX-1. C_GNSS_SIG_MAX:error
 */
uint8_t gnss_FreqIdx2Signal(uint8_t index, uint8_t sys);

/**
 * @brief signal ID -> 1C,2W...
 * @param[in] e_ag_signal_ch
 * @param[out] c_sig
 * @return none
 */
void gnss_Signal2Char(gnss_SignalType e_ag_signal_ch, char* c_sig);

/**
 * @brief satellite string, eg.G02
 * @param[in] svid
 * @param[in] sys
 * @param[out] s
 */
void svid_SatString(uint8_t svid, uint8_t sys, char* s);

/**
 * @brief satellite string, eg.G02
 * @param[in] satidx
 * @param[out] s
 */
void satidx_SatString(uint8_t satidx, char* s);

/**
 * @brief Refresh measurement collect to remove expired measurement
          Note: the valid time limit of satellite measurement is 0.15s, tot~=80ms
                the valid time limit of satellite measurement is 0.5s,tor interval=1000ms
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_SatSigMeasCollect_Refresh(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect);

/**
 * @brief Clear satellite signal measurement collect.
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_SatSigMeasCollect_Clear(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect);

/**
 * @brief Copy satellite signal measurement collect.
 * @param[in]   pz_SatSigMeasCollect_src
 * @param[in]   pz_SatSigMeasCollect_dst
 * @return      None
 */
void gnss_SatSigMeasCollect_Copy(
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect_src,
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect_dst);

/**
 * @brief Pack a spare satellite signal measurement collect structure as a
          tight satellite signal measurement collect structure.
          The tight structure is used to transmit by IPC
 * @param[in]   pz_satSigMeasCollect
 * @param[out]  pz_tightSatSigMeasCollect
 * @return      None
 */
void
gnss_SatSigMeasCollect_Pack(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const gnss_PVTResult_t* pz_pvt_result,
                            gnss_TightSatSigMeasCollect_t* pz_tightSatSigMeasCollect);

/**
 * @brief Uppack a tight satellite signal measurement collect structure as
          a spare satellite signal measurement collect structure
          The tight structure is used to transmit by IPC
 * @param[in]   pz_tightSatSigMeasCollect
 * @param[out]  pz_SatSigMeasCollect
 * @return      None
 */
void gnss_SatSigMeasCollect_Unpack(
        const gnss_TightSatSigMeasCollect_t* pz_tightSatSigMeasCollect,
        gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief convert satellite signal measurement collect to TDCP measurement
 * @param[in] pz_SatSigMeasCollect satellite signal measurement
 * @param[out] pz_tdcpMeas TDCP measurement
 */
void
gnss_cvt_SatSigMeas2TdcpMeas(const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect,
                             const gnss_PositionFix_t* pz_PositionFix, gnss_TdcpMeasBlock_t* pz_tdcpMeas);
/**
 * @brief Append time-adjusted measurement into satellite signal measurement collect.
          If the satellite measure pointer was used, just update without allocating new memory.
          If the satellite measure pointer is NULL, allocate new memory.
          After appending new measurement, the collect data maybe contain old measurement.
          If don't use them, it need to delete the old measurement and free their memory
 * @param[in]   pz_MeasBlock
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_AppendMeasBlkToSatSigMeasCollect(GnssMeasBlock_t* pz_MeasBlock, gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect);

/**
 * @brief Append time-adjusted correction data struct into satellite signal measurement collect.
          If the satellite measure pointer was used, just update without allocating new memory.
          If the satellite measure pointer is NULL, allocate new memory.
          After appending new measurement, the collect data maybe contain old measurement.
          If don't use them, it need to delete the old measurement and free their memory
 * @param[in]   pz_corrBlock
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_AppendCorrBlkToSatSigMeasCollect(const GnssCorrBlock_t* pz_corrBlock, gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect);

/**
 * @brief convert receiver measurement block structure to TDCP measurement
 * @param[int] pz_MeasBlock
 * @param[out] pz_tdcpMeas
 */
void gnss_cvt_Meas2TdcpMeas(const GnssMeasBlock_t* pz_MeasBlock, gnss_TdcpMeasBlock_t* pz_tdcpMeas);

/**
 * @brief Compute Satellite Postion Velocity and Clock from SD polynomial.
          Store the satellite information into the z_satPosVelClk of measurement.
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_ComputeSatPosVelClk(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief Raw receiver measurement don't have Week number, it needs to
          get week from ephemeris and create a valid receiver time from
          measurement tow and ephemeris week.
 * @param [in]w_measCount meas count of pz_meas
 * @param [in]pz_meas Measurement
 * @param [in/out]pz_gpsTime GPS time
 * @return TRUE:success  FALSE:failed
 */
BOOL gnss_AdjustMeasBlockWeek(uint16_t w_measCount, const GnssMeas_t* pz_meas, GpsTime_t* pz_gpsTime);

/**
 * @brief Compensate for 20ms jump for ST chip
 * @param[in,out] pz_satSigMeasCollect
 */
void gnss_SatSigMeasTimeJumpST(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief Using the result of PVT to fill up the mask of observations
 * @param[in]      pz_pvt_result is information of pvt
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @return         void
 */
void gnss_fillUpObsMask(const gnss_PVTResult_t* pz_pvt_result, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief malloc the memory used in the struct gnss_fixedSignalAmbPool_t
 * @param[out]      pz_fixedAmbPool is the ambiguity pool of fixed signal
 * @param[in]       pz_satSigMeasCollect is the observation information
 * @return          0 represent successful, other represent failure
 */
uint8_t malloc_fixedSignalAmbPool(gnss_fixedSignalAmbPool_t** pz_fixedAmbPool, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief free the memory used in the struct gnss_fixedSignalAmbPool_t
 * @param[in]      pz_fixedAmbPool is the ambiguity pool of fixed signal
 * @return         void
 */
void free_fixedSignalAmbPool(gnss_fixedSignalAmbPool_t** pz_fixedAmbPool);

/**
 * @brief LD factorization (Q=L'*diag(D)*L)
 */
extern int8_t LD(uint32_t n, const double* Q, double* L, double* D);
/**
 * @brief integer gauss transformation
 */
extern void gauss(uint32_t n, double* L, double* Z, uint32_t i, uint32_t j);
/**
 * @brief permutations
 */
extern void perm(int32_t n, double* L, double* D, int32_t j, double del, double* Z);

/**
 * @brief lambda/mlambda integer least-square estimation
 * @param[in]   n  -  number of float parameters
 * @param[in]   m  -  number of fixed solutions
 * @param[in]   a  -  float parameters (n x 1)
 * @param[in]   Q  -  covariance matrix of float parameters (n x n)
 * @param[out]  F  -  fixed solutions (n x m)
 * @param[out]  s  -  sum of squared residulas of fixed solutions (1 x m)
 * @param[out]  boot_rate  -  rate of bootstrap
 * @param[out]  pf_adop
 * @return status (0:ok,other:error)
 */
extern int8_t amb_lambda(uint32_t n, uint32_t m, const double* a, const double* Q, double* F,
  double* s, double* boot_rate, float* pf_adop);

/**
 * @brief lambda/mlambda best integer equivariant
 * @param[in]   n  -  number of float parameters
 * @param[in]   m  -  number of fixed solutions
 * @param[in]   a  -  float parameters (n x 1)
 * @param[in]   Q  -  covariance matrix of float parameters (n x n)
 * @param[out]  F  -  fixed solutions (n x m)
 * @param[out]  s  -  sum of squared residulas of fixed solutions (1 x m)
 * @param[out]  boot_rate  -  rate of bootstrap
 * @param[out]  pf_adop
 * @return status (0:ok,other:error)
 */
extern int8_t lambdaBie(uint32_t n, uint32_t m, const double* a, const double* Q, double* F,
  double* s, float* boot_rate, float* pf_adop);

/**
 * @brief get satellite transmit time
 * @param[in] pz_meas measurement
 * @param[in,out] gpst time of receiver
 * @return FALSE:success, TRUE: failed
 */
BOOL gnss_CalMeasTot(const gnss_SatelliteMeas_t* pz_meas, GpsTime_t* tor);

/**
 * @brief calculate time of transmit of satellite by the matched gnss measurement
 * @param[in] pz_meas GNSS Measurement structure
 * @param[in] n number of the pz_meas
 * @param[in] u_constellation system
 * @param[in] u_svid PRN
 * @param[in,out] tor time of recevier[in],  time of transmit[out]
 * @return FALSE: failed; TRUE: success
 */
BOOL gnss_CalGnssMeasTot(const GnssMeas_t* pz_meas, uint8_t n, gnss_ConstellationType u_constellation, uint8_t u_svid, GpsTime_t* tor);

/**
 * @brief compute DOP (dilution of precision)
 * @param[in] u_ns -- number of satellite
 * @param[in] pf_azel -- satellite azimuth/elevation angle (rad)
 * @param[out] pf_dop -- DOPs {GDOP,PDOP,HDOP,VDOP}
 * @return none
 */
extern void gnss_dops(uint8_t u_ns, const float* pf_azel, float* pf_dop);

/**
 * @brief initilize the struct of GnssMeasFeedbackInsBlock_t
 * @param[out] pz_feedbackInsMeasBlock the measure blcok that GNSS feedback to INS
 * @return void
 */
void cmn_InitGnssMeasFeedbackInsBlock(gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock);

/**
 * @brief Data log of protection level
 * @param[in] u_solFlag algorithm solution status, MAX_FILTER_POS_STATUS, ppp default is 0
 * @param[in] pz_pl protection level result
 * @param[in] gpst GSPT
 */
extern void
gnss_log_ProtectionLevel(uint8_t u_solFlag, const integ_PLResult_t* pz_pl, const gnss_PositionFix_t* pz_posSolution);

/**
 * @brief Data log of STD
 * @param[in] u_solFlag algorithm solution status, gnss_FixFlagType
 * @param[in] pz_posSolution
 */
extern void gnss_log_STD(uint8_t u_solFlag, const gnss_PositionFix_t* pz_posSolution);

/**
 * @brief Position uncertainty default value that by position solution status
 * @param[in,out] pz_posSolution
 */
extern void gnss_PositionUncertaintyMajic(gnss_PositionFix_t* pz_posSolution);

/**
 * @brief make loc_api_location_report_t
 * @param[in] pz_PositionFix_report position information from algorithm
 * @param[out] z_api_location_report location info for report
 */
void gnss_cvt_GnssPositionFix_To_LocApiPositionFix(const gnss_PositionFix_t* pz_PositionFix_report,
  loc_api_ConsoildatedPositionFix_t* pz_ConsoildatedPositionFix);

/**
  * @brief make loc_api_location_report_t
  * @param[in] z_api_location_report location info for report
  * @param[out] pz_PositionFix_report position information from algorithm
  */
void gnss_cvt_LocApiPositionFix_To_GnssPositionFix(const loc_api_ConsoildatedPositionFix_t* pz_ConsoildatedPositionFix,
  gnss_PositionFix_t* pz_PositionFix_report);

/**
 * @brief identify the GNSS measure whether it is multi-frequency data that frequency number greater than 2
 * @param[in] pz_meas GNSS Measurement structure
 * @param[in] w_measCount number of the pz_meas
 * @return    TRUE: is multi-frequency data
 */
BOOL gnss_IdentifyMultiFreq(const GnssMeas_t* pz_meas, uint16_t w_measCount);

/**
 * @brief      get heading & pitch, calculate their uncertainty
 * @param[in]  pd_mainAntXyz is coordinate of main antenna (ecef)
 * @param[in]  pd_auxiAntXyz is coordinate of auxi antenna (ecef)
 * @param[in]  pd_xyzUnc uncertainty of main-auxi-baseline (ecef)
 * @param[out] pz_ortResult is the result of orient that include heading,pitch and roll
 * @return     0 represent successs and other failture
 */
uint8_t gnss_getOrtHeadingPitch(const double* pd_mainAntXyz, const double* pd_auxiAntXyz, const double* pd_xyzUnc, gnss_headingPitchRoll_t* pz_ortResult);

/**
  * @brief make gnss_OrientFix_t
  * @param[in] pz_locApiOrientReport location info for report
  * @param[out] pz_orientFix orient information from algorithm
  */
void gnss_cvt_LocApiOrientFix_To_GnssOrientFix(const loc_api_orient_report_t* pz_locApiOrientReport, gnss_OrientFix_t* pz_orientFix);

/**
 * @brief fill up fix flag and the type of position and velocity for orient
 * @param[in]  u_fixFlag is the type of fix falg
 * @return     orient type of position and velocity
 */
gnss_orientPosVelType gnss_cvt_ortFixFlag2PosVelType(gnss_FixFlagType u_fixFlag);

uint8_t gnss_CvtConstellation(uint8_t u_Constl, uint8_t u_Svid);

/**
 * @brief whether skip C_GNSS_BDS2 variable in a loop
 * @param[in]  q_isSperateBDS2And3 whether sperate for BDS2 and BDS3
 * @param[in]  u_constellation is the type of constellation
 * @return     TRUE represent skip C_GNSS_BDS2 variable in a loop, and FALSE represent do not skip C_GNSS_BDS2 variable in a loop
 */
BOOL gnss_WhetherSkipBDS2VarInLoop(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_constellation);

/**
 * @brief get the contellation enum value in a loop
 * @param[in]  q_isSperateBDS2And3 whether sperate for BDS2 and BDS3
 * @param[in]  u_constellation is the type of constellation
 * @return     the contellation enum value in a loop
 */
gnss_ConstellationType gnss_getConstellationEnumValueInLoop(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_constellation);

/**
 * @brief according the signal type to determine whether BDS2 and BDS3 are separate for gnss_SatSigMeasCollect_t struct
 * @param[in]  z_targetFreq is the target frequency
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     TRUE represent BDS2 and BDS3 will be separated and FALSE represent BDS2 and BDS3 will not be separated
 */
BOOL gnss_isSeparateBDS2And3ForSatSignalCollect(gnss_FreqType z_targetFreq, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect);

/**
 * @brief  generate integrity flag, AL default 3.0 m
 * @param[in] f_protection_level >=0
 * @param[in] f_al >=0
 * @return
 */
gnss_integrityFlag gnss_genIntegrityFlag(float f_protection_level, float f_al);

/**
 * @brief generate gnss_ProtectionLevel_t by PL and AL
 * @param[in] f_protection_level
 * @return gnss_ProtectionLevel_t that include pl and flag
 */
gnss_ProtectionLevel_t gnss_genProtectionLevelStruct(float f_protection_level);

/**
 * @brief Initialize gnss_Integrity_t
 * @param[in,out] pz_itg
 */
void gnss_InitIntegrityStruct(gnss_Integrity_t* pz_itg);

END_DECL

#endif

