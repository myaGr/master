/**@file        rtcm_dec.h
 * @brief       RTCM stream decode module
 * @author      caizhijie
 * @date        2022/04/05
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/05  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __RTCM_DECODE_H__
#define __RTCM_DECODE_H__

#include "cmn_def.h"
#include "gnss_type.h"
#include "gnss_def.h"

BEGIN_DECL

/** RTCM decoder buffer size */
#define RTCM_BUF_SIZE (1024 + 6)
#define E2EPRPREAMB (0x0FF05A03)    /*267409923 */
#define E2EMSGHEADLEN  4               /* nbyte of e2e head type */
#define E2EMSGMAXLEN 16                /* max nbyte of e2e */

typedef struct {
  uint16_t w_staid;
  double   d_staPos[3]; /* station position (ecef) (m) */
  double   d_antHeight; /* antenna height (m) */
} RtcmStationInfo_t;

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
} RtcmObs_t;

typedef struct {
  uint32_t tow_ms;
  uint32_t indicator;
} LossOfLock_t;

typedef struct {
  uint32_t q_towMsec;
  uint32_t w_gpsWeek;
  uint32_t type_id;
  uint8_t is_complete;
  uint8_t buf[RTCM_BUF_SIZE];
  uint8_t is_e2e_data;
  uint8_t e2emsglen;
  uint8_t e2emsgbuf[E2EMSGMAXLEN + 1];
  uint32_t nbyte;
  uint32_t length;
  RtcmStationInfo_t satInfo;
  uint8_t nobs;
  RtcmObs_t obs[MAX_GNSS_TRK_MEAS_NUMBER];
  gnss_Ephemeris_t ephemeris;
  LossOfLock_t GpsLossOfLock[32][3];
  LossOfLock_t GlnLossOfLock[24][3];
  LossOfLock_t BdsLossOfLock[64][3];
  LossOfLock_t GalLossOfLock[36][3];
  LossOfLock_t QzsLossOfLock[7][3];
} RtcmDecoder_t;

int8_t rtcm_dec_input(RtcmDecoder_t* decoder, uint8_t data);

/**
 * @brief  skip e2e msg from GNSS stream
 * @param[in] decoder,  decode information
 * @param[in] inputbuf, GNSS stream
 * @param[in] inlen, length of inputbuf
 * @param[out] outputbuf, skip result for next decoding
 * @param[out] outlen, length of outputbuf
 * @return BOOL, false: do not decode next; ture: do decoding
 */
BOOL rtcmE2eSkip(RtcmDecoder_t* decoder, const uint8_t* inputbuf, uint32_t inlen, uint8_t* outputbuf, uint32_t* outlen);

/**
 * @brief  Extract GnssMeasBlock from RTCM Decoder to GnssMeasBlock
 * @return none
 */
void rtcm_ExtractMeasBlk(RtcmDecoder_t* rtcm, GnssMeasBlock_t* meas_blk);

/**
 * @brief  Extract GnssCorrBlock from RTCM Decoder to GnssMeasBlock
 * @return none
 */
void rtcm_ExtractCorrBlk(RtcmDecoder_t* rtcm, GnssCorrBlock_t* corr_blk);

END_DECL

#endif

