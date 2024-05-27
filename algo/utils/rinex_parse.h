/**@file        rinex_parse.h
 * @brief       RINEX stream parse module
 * @author      wuzhiyuan
 * @date        2023/09/06
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/09/06  <td>0.1      <td>wuzhiyuan   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __RIMEX_PARSE_H__
#define __RINEX_PARSE_H__

#include "cmn_def.h"
#include "gnss_type.h"
#include "gnss_def.h"

BEGIN_DECL

typedef struct {
  uint16_t w_staid;
  double   d_staPos[3]; /* station position (ecef) (m) */
  double   d_antHeight; /* antenna height (m) */
} RinexStationInfo_t;

typedef struct {
  uint32_t tow_ms; /** receiver sampling time (GPST) */
  uint8_t sys;     /** gnss_ConstellationType */
  uint8_t svid;    /** svid of each system */
  uint8_t signal;  /** gnss_SignalType */
  uint8_t snr;     /* signal strength (0.25 dBHz) */
  uint8_t lli;     /* loss of lock indicator */
  double pseudorange;   /* observation data pseudorange (m) */
  double doppler;       /* observation data doppler frequency (Hz) */
  double carrier_phase; /* observation data carrier-phase (cycle) */
} RinexObs_t;

typedef struct {
  uint32_t tow_ms;
  uint32_t indicator;
} LossofLock_t;

typedef struct {
  uint32_t q_towMsec;
  uint32_t w_gpsWeek;
  uint32_t type_id;
  uint8_t is_complete;
  uint32_t nbyte;
  uint32_t length;
  RinexStationInfo_t satInfo;
  uint8_t nobs;
  RinexObs_t obs[128];
  gnss_Ephemeris_t ephemeris;
  LossofLock_t GpsLossOfLock[32][3];
  LossofLock_t GlnLossOfLock[24][3];
  LossofLock_t BdsLossOfLock[64][3];
  LossofLock_t GalLossOfLock[36][3];
  LossofLock_t QzsLossOfLock[7][3];
} RinexDecoder_t;

/**
 * @brief parse broadcast ephemeris for each system
 * @param[in]  u_constellation represent the constellation
 * @param[in]  u_svid represent the satellite PRN
 * @param[in]  pd_data represent the broadcast ephemeris
 * @param[in]  pq_epoch represent the Epoch time
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @return     status -- 1: parse success, other: fail
 */
int8_t rinex_dec_eph(uint8_t u_constellation, uint8_t u_svid, const double* pd_data, uint32_t* pq_epoch, RinexDecoder_t* pz_RinexDecoder);

/**
 * @brief match broadcast ephemeris for AG format
 * @param[in]  q_towMsec represent the tow time of observaton
 * @param[in]  q_towEph represent the tow time of ephemeris
 * @param[in]  u_constellation
 * @return     status -- TRUE: match success, FALSE: fail
 */
BOOL rinex_ephMatch_AG(uint32_t q_towMsec, uint32_t q_towEph, uint8_t u_constellation);

/**
 * @brief convert character to system in RINEX file
 * @param[in]  pu_buff represent the character such as "G" or "E"
 * @return     the index of system in RTK2.0
 */
uint8_t rinex_CharToSys(char* pu_buff);

/**
 * @brief get information from the head of RINEX file
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pu_buff represent the message of RINEX file
 * @param[in]  pu_obsCode represent the code information of observation
 * @param[in]  pu_obsCodeNum represent the number of observation type
 * @return     status -- -1: still in the head of file, 0: leave the head of file
 */
int8_t rinex_dec_obsf(RinexDecoder_t* pz_RinexDecoder, char* pu_buff, char pu_obsCode[][32][4], uint8_t* pu_obsCodeNum);

/**
 * @brief parse observation in RINEX format
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pu_buff represent the message of RINEX file
 * @param[in]  pu_obsCode represent the code information of observation
 * @param[in]  pu_obsCodeNum represent the number of observation type
 * @return     status -- TRUE: parse success, FALSE: parse fail
 */
int8_t rinex_dec_obsb(RinexDecoder_t* pz_RinexDecoder, char* pu_buff, const char pu_obsCode[][32][4], const uint8_t* pu_obsCodeNum);

/**
 * @brief  Extract GnssMeasBlock from RINEX Decoder to GnssMeasBlock
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pz_MeasBlock represent the measurement block structure
 * @return none
 */
void rinex_ExtractMeasBlk(RinexDecoder_t* pz_RinexDecoder, GnssMeasBlock_t* pz_MeasBlock);

/**
 * @brief  Extract GnssCorrBlock from RINEX Decoder to GnssMeasBlock
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pz_CorrectBlock represent the correction block structure
 * @return none
 */
void rinex_ExtractCorrBlk(RinexDecoder_t* pz_RinexDecoder, GnssCorrBlock_t* pz_CorrectBlock);


END_DECL

#endif