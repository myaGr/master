/**@file        rtcm_dec.c
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

#include "gnss_def.h"
#include "rtcm_dec.h"
#include "cmn_utils.h"
#include "gnss_common.h"
#include "mw_log.h"
#include "mw_alloc.h"
#include "sm_api.h"

#define RTCM3PREAMB 0xD3            /* rtcm ver.3 frame preamble */
#define RTCM_PRECHECK_LEN 6

typedef struct {            /* multi-signal-message header type */
  uint8_t iod;              /* issue of data station */
  uint8_t time_s;           /* cumulative session transmitting time */
  uint8_t clk_str;          /* clock steering indicator */
  uint8_t clk_ext;          /* external clock indicator */
  uint8_t smooth;           /* divergence free smoothing indicator */
  uint8_t tint_s;           /* soothing interval */
  uint8_t nsat, nsig;       /* number of satellites/signals */
  uint8_t sats[64];         /* satellites */
  uint8_t sigs[32];         /* signals */
  uint8_t cellmask[64];     /* cell mask */
} rt_msm_h_t;


static gnss_SignalType convert_signal_id_to_type(gnss_ConstellationType sys, uint8_t sig_id)
{
  gnss_SignalType signal = C_GNSS_SIG_MAX;

  switch (sys)
  {
  case C_GNSS_GPS:
    switch (sig_id)
    {
    case 2:   /* GPS L1C */
      signal = C_GNSS_SIG_GPS_L1C;
      break;
    case 10:   /* GPS L2W */
      signal = C_GNSS_SIG_GPS_L2C;
      break;
    case 15:   /* GPS L2C(M) */
      signal = C_GNSS_SIG_GPS_L2C;
      break;
    case 16:   /* GPS L2C(L) */
      signal = C_GNSS_SIG_GPS_L2C;
      break;
    case 17:   /* GPS L2C(M+L) */
      signal = C_GNSS_SIG_GPS_L2C;
      break;
    case 22:  /* GPS L5I */
      signal = C_GNSS_SIG_GPS_L5Q;
      break;
    case 23:  /* GPS L5Q */
      signal = C_GNSS_SIG_GPS_L5Q;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_GLO:
    switch (sig_id)
    {
    case 2:  /* GLONASS GI */
      signal = C_GNSS_SIG_GLO_G1;
      break;
    case 8:  /* GLONASS G2 */
      signal = C_GNSS_SIG_MAX;
      break;
    case 9:  /* GLONASS G2 */
      /*signal = C_GNSS_SIG_GLO_G2;*/
      signal = C_GNSS_SIG_MAX;/*Close GLONASS L2 frequecny to save memory*/
      break;
    default:
      signal = C_GNSS_SIG_MAX;
      break;
    }
    break;
  case C_GNSS_BDS3:
    switch (sig_id)
    {
    case 2: 
      signal = C_GNSS_SIG_BDS_B1I;
      break;
    case 8:
      signal = C_GNSS_SIG_BDS_B3I;
      break;
    case 14:  /* BDS-2 B2I  7I*/
      signal = C_GNSS_SIG_BDS_B2I;
      break;
    case 15:  /* BDS-2 B2I 7Q*/
      signal = C_GNSS_SIG_BDS_B2I;
      break;
    case 23:
      signal = C_GNSS_SIG_BDS_B2A;
      break;
    case 31:
      signal = C_GNSS_SIG_MAX;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_GAL:
    switch (sig_id)
    {
    case 2:
      signal = C_GNSS_SIG_GAL_E1;
      break;
    case 4: /* C1B */
      signal = C_GNSS_SIG_GAL_E1;
      break;
    case 5:
      signal = C_GNSS_SIG_GAL_E1;
      break;
    case 14:
      signal = C_GNSS_SIG_GAL_E5B;
      break;
    case 15:
      signal = C_GNSS_SIG_GAL_E5B;
      break;
    case 22:
      signal = C_GNSS_SIG_GAL_E5A;
      break;
    case 23:
      signal = C_GNSS_SIG_GAL_E5A;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_QZS:
    switch (sig_id)
    {
    case 2:   /* QZS L1C */
      signal = C_GNSS_SIG_QZS_L1C;
      break;
    case 17:   /* QZS L2C */
      signal = C_GNSS_SIG_QZS_L2C;
      break;
    case 22:  /* QZS L5Q */
      signal = C_GNSS_SIG_QZS_L5Q;
      break;
    case 23:  /* QZS L5Q */
      signal = C_GNSS_SIG_QZS_L5Q;
      break;
    default:
      break;
    }
    break;
  }

  return signal;
}

static gnss_SignalType convert_signal_id_to_type_default(gnss_ConstellationType sys, uint8_t sig_id)
{
  gnss_SignalType sig_type = convert_signal_id_to_type(sys, sig_id);
  if(C_GNSS_SIG_MAX == sig_type)
  {
    if(C_GNSS_GPS == sys) sig_type = C_GNSS_SIG_GPS_L1C;
//    if(C_GNSS_BDS == sys) sig_type = C_GNSS_SIG_BDS_B1I;
    if(C_GNSS_GAL == sys) sig_type = C_GNSS_SIG_GAL_E1;
    if(C_GNSS_QZS == sys) sig_type = C_GNSS_SIG_QZS_L1C;
  }
  return sig_type;
}

/* decode type 1005: Stationary Antenna Reference Point, No Height Information-*/
extern int decode_type1005(RtcmDecoder_t* decoder)
{
  uint32_t i = 24 + 12;
  if (i + 140 == decoder->length * 8) {
    decoder->satInfo.w_staid = loc_getbitu(decoder->buf, i, 12); i += 12;
    i += 6 + 4;
    decoder->satInfo.d_staPos[0] = 0.0001 * loc_getbits_38(decoder->buf, i); i += 38 + 2;
    decoder->satInfo.d_staPos[1] = 0.0001 * loc_getbits_38(decoder->buf, i); i += 38 + 2;
    decoder->satInfo.d_staPos[2] = 0.0001 * loc_getbits_38(decoder->buf, i);
    decoder->satInfo.d_antHeight = 0.0;
  }
  else {
    return -1;
  }
  return 0;
}

/* decode type 1006: Stationary Antenna Reference Point, With Height Information-*/
extern int decode_type1006(RtcmDecoder_t* decoder)
{
  uint32_t i = 24 + 12;
  if (i + 156 == decoder->length * 8) {
    decoder->satInfo.w_staid = loc_getbitu(decoder->buf, i, 12); i += 12;
    i += 6 + 4;
    decoder->satInfo.d_staPos[0] = 0.0001 * loc_getbits_38(decoder->buf, i); i += 38 + 2;
    decoder->satInfo.d_staPos[1] = 0.0001 * loc_getbits_38(decoder->buf, i); i += 38 + 2;
    decoder->satInfo.d_staPos[2] = 0.0001 * loc_getbits_38(decoder->buf, i); i += 38;
    decoder->satInfo.d_antHeight = 0.0001 * loc_getbitu(decoder->buf, i, 16);
  }
  else {
    return -1;
  }
  return 0;
}

/* decode type 1019: gps ephemerides -----------------------------------------*/
extern int decode_type1019(RtcmDecoder_t* decoder)
{
  GpsEphemeris_t eph = { 0 };
  uint32_t i = 24 + 12;
  if (i + 476 <= decoder->length * 8) {
    eph.svid      = loc_getbitu(decoder->buf, i, 6);                   i += 6;
    eph.week      = loc_getbitu(decoder->buf, i, 10);                  i += 10;
    eph.accuracy  = loc_getbitu(decoder->buf, i, 4);                   i += 4;
    eph.code      = loc_getbitu(decoder->buf, i, 2);                   i += 2;
    eph.idot      = loc_getbits(decoder->buf, i, 14) * P2_43 * PI;     i += 14;
    eph.iode      = loc_getbitu(decoder->buf, i, 8);                   i += 8;
    eph.toc       = loc_getbitu(decoder->buf, i, 16) * 16.0;           i += 16;
    eph.af2       = loc_getbits(decoder->buf, i, 8) * P2_55;           i += 8;
    eph.af1       = loc_getbits(decoder->buf, i, 16) * P2_43;          i += 16;
    eph.af0       = loc_getbits(decoder->buf, i, 22) * P2_31;          i += 22;
    eph.iodc      = loc_getbitu(decoder->buf, i, 10);                  i += 10;
    eph.crs       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.DeltaN    = loc_getbits(decoder->buf, i, 16) * P2_43 * PI;     i += 16;
    eph.M0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cuc       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.e         = loc_getbitu(decoder->buf, i, 32) * P2_33;          i += 32;
    eph.cus       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.sqrt_A    = loc_getbitu(decoder->buf, i, 32) * P2_19;          i += 32;
    eph.toe       = loc_getbitu(decoder->buf, i, 16) * 16.0;           i += 16;
    eph.cic       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.Omega0    = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cis       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.i0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.crc       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.Omega     = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.OmegaDot  = loc_getbits(decoder->buf, i, 24) * P2_43 * PI;     i += 24;
    eph.tgd       = loc_getbits(decoder->buf, i, 8) * P2_31;           i += 8;
    eph.health    = loc_getbitu(decoder->buf, i, 6);                   i += 6;
    /* flag       = loc_getbitu(decoder->buf, i, 1);*/                 i += 1;
    eph.fit       = loc_getbitu(decoder->buf, i, 1) ? 0 : 4; /* 0:4hr,1:>4hr */
  }
  else {
    return -1;
  }

  if (eph.week > 0 && eph.week < 2048) {
    eph.week = 2048 + eph.week%1024;
  }
  
  tm_cvt_SetGpst(&decoder->ephemeris.z_toeOfGpst, eph.week, eph.toe);
  memcpy(&decoder->ephemeris.eph.z_gpsEph, &eph, sizeof(GpsEphemeris_t));
  decoder->ephemeris.u_constellation = C_GNSS_GPS;
  decoder->q_towMsec = decoder->ephemeris.z_toeOfGpst.q_towMsec;

  return 2;
}


/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
extern int decode_type1044(RtcmDecoder_t* decoder)
{
  GpsEphemeris_t eph = { 0 };
  uint32_t i = 24 + 12;
  if (i + 473 <= decoder->length * 8) {
    eph.svid      = loc_getbitu(decoder->buf, i, 4) + 192;             i += 4;
    eph.toc       = loc_getbitu(decoder->buf, i, 16) * 16.0;           i += 16;
    eph.af2       = loc_getbits(decoder->buf, i, 8) * P2_55;           i += 8;
    eph.af1       = loc_getbits(decoder->buf, i, 16) * P2_43;          i += 16;
    eph.af0       = loc_getbits(decoder->buf, i, 22) * P2_31;          i += 22;
    eph.iode      = loc_getbitu(decoder->buf, i, 8);                   i += 8;
    eph.crs       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.DeltaN    = loc_getbits(decoder->buf, i, 16) * P2_43 * PI;     i += 16;
    eph.M0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cuc       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.e         = loc_getbitu(decoder->buf, i, 32) * P2_33;          i += 32;
    eph.cus       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.sqrt_A    = loc_getbitu(decoder->buf, i, 32) * P2_19;          i += 32;
    eph.toe       = loc_getbitu(decoder->buf, i, 16) * 16.0;           i += 16;
    eph.cic       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.Omega0    = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cis       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.i0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.crc       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.Omega     = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.OmegaDot  = loc_getbits(decoder->buf, i, 24) * P2_43 * PI;     i += 24;
    eph.idot      = loc_getbits(decoder->buf, i, 14) * P2_43 * PI;     i += 14;
    eph.code      = loc_getbitu(decoder->buf, i, 2);                   i += 2;
    eph.week      = loc_getbitu(decoder->buf, i, 10);                  i += 10;
    eph.accuracy  = loc_getbitu(decoder->buf, i, 4);                   i += 4;
    eph.health    = loc_getbitu(decoder->buf, i, 6);                   i += 6;
    eph.tgd       = loc_getbits(decoder->buf, i, 8) * P2_31;           i += 8;
    eph.iodc      = loc_getbitu(decoder->buf, i, 10);                  i += 10;
    eph.fit       = loc_getbitu(decoder->buf, i, 1) ? 0 : 2 ; /* 0:2hr,1:>2hr */
  }
  else {
    return -1;
  }

  if (eph.week > 0 && eph.week < 2048) {
    eph.week = 2048 + eph.week % 1024;
  }
  
  tm_cvt_SetGpst(&decoder->ephemeris.z_toeOfGpst, eph.week, eph.toe);
  memcpy(&decoder->ephemeris.eph.z_qzsEph, &eph, sizeof(GpsEphemeris_t));
  decoder->ephemeris.u_constellation = C_GNSS_QZS;
  decoder->q_towMsec = decoder->ephemeris.z_toeOfGpst.q_towMsec;

  return 2;
}

/* decode type 63 or 1042: beidou ephemerides (rtcm draft) ---------------------------*/
static int decode_type1042(RtcmDecoder_t* decoder)
{
  BdsEphemeris_t eph = { 0 };
  uint32_t i = 24 + 12;
  BOOL q_isBDS3Sat = FALSE;
  if (i + 499 <= decoder->length * 8) {
    eph.svid      = loc_getbitu(decoder->buf, i, 6);                   i += 6;
    eph.week      = loc_getbitu(decoder->buf, i, 13);                  i += 13;
    eph.accuracy  = loc_getbitu(decoder->buf, i, 4);                   i += 4;
    eph.idot      = loc_getbits(decoder->buf, i, 14) * P2_43 * PI;     i += 14;
    eph.iode      = loc_getbitu(decoder->buf, i, 5);                   i += 5; /* AODE */
    eph.toc       = loc_getbitu(decoder->buf, i, 17) * 8.0;            i += 17;
    eph.af2       = loc_getbits(decoder->buf, i, 11) * P2_66;          i += 11;
    eph.af1       = loc_getbits(decoder->buf, i, 22) * P2_50;          i += 22;
    eph.af0       = loc_getbits(decoder->buf, i, 24) * P2_33;          i += 24;
    eph.iodc      = loc_getbitu(decoder->buf, i, 5);                   i += 5; /* AODC */
    eph.crs       = loc_getbits(decoder->buf, i, 18) * P2_06;          i += 18;
    eph.DeltaN    = loc_getbits(decoder->buf, i, 16) * P2_43 * PI;     i += 16;
    eph.M0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cuc       = loc_getbits(decoder->buf, i, 18) * P2_31;          i += 18;
    eph.e         = loc_getbitu(decoder->buf, i, 32) * P2_33;          i += 32;
    eph.cus       = loc_getbits(decoder->buf, i, 18) * P2_31;          i += 18;
    eph.sqrt_A    = loc_getbitu(decoder->buf, i, 32) * P2_19;          i += 32;
    eph.toe       = loc_getbitu(decoder->buf, i, 17) * 8.0;            i += 17;
    eph.cic       = loc_getbits(decoder->buf, i, 18) * P2_31;          i += 18;
    eph.Omega0    = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cis       = loc_getbits(decoder->buf, i, 18) * P2_31;          i += 18;
    eph.i0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.crc       = loc_getbits(decoder->buf, i, 18) * P2_06;          i += 18;
    eph.Omega     = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.OmegaDot  = loc_getbits(decoder->buf, i, 24) * P2_43 * PI;     i += 24;
    eph.tgdB1I    = loc_getbits(decoder->buf, i, 10) * 1E-10;          i += 10;
    eph.tgdB2I    = loc_getbits(decoder->buf, i, 10) * 1E-10;          i += 10;
    eph.health    = loc_getbitu(decoder->buf, i, 1);                  /* i += 1;*/
  }
  else {
    return -1;
  }
  
  q_isBDS3Sat = gnss_isBDS3Sat(eph.svid, C_GNSS_BDS3);
  BdsTime_t bdt = { 0 };
  tm_cvt_SetBdt(&bdt, eph.week, eph.toe);
  tm_cvt_BdtToGpst(&bdt, &decoder->ephemeris.z_toeOfGpst);
  memcpy(&decoder->ephemeris.eph.z_bdsEph, &eph, sizeof(BdsEphemeris_t));
  if (TRUE == q_isBDS3Sat)
  {
    decoder->ephemeris.u_constellation = C_GNSS_BDS3;
  }
  else
  {
    decoder->ephemeris.u_constellation = C_GNSS_BDS2;
  }
  decoder->q_towMsec = decoder->ephemeris.z_toeOfGpst.q_towMsec;

  return 2;
}

/* decode type 1045: galileo F/NAV satellite ephemerides (ref [15]) ----------*/
static int decode_type1045(RtcmDecoder_t* decoder)
{
  GalEphemeris_t eph = { 0 };
  uint32_t i = 24 + 12;
  if (i + 484 <= decoder->length * 8) {
    eph.svid      = loc_getbitu(decoder->buf, i, 6);                   i += 6;
    eph.week      = loc_getbitu(decoder->buf, i, 12);                  i += 12;
    eph.iode      = loc_getbitu(decoder->buf, i, 10);                  i += 10;
    eph.accuracy  = loc_getbitu(decoder->buf, i, 8);                   i += 8;
    eph.idot      = loc_getbits(decoder->buf, i, 14) * P2_43 * PI;     i += 14;
    eph.toc       = loc_getbitu(decoder->buf, i, 14) * 60.0;           i += 14;
    eph.af2       = loc_getbits(decoder->buf, i, 6) * P2_59;           i += 6;
    eph.af1       = loc_getbits(decoder->buf, i, 21) * P2_46;          i += 21;
    eph.af0       = loc_getbits(decoder->buf, i, 31) * P2_34;          i += 31;
    eph.crs       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.DeltaN    = loc_getbits(decoder->buf, i, 16) * P2_43 * PI;     i += 16;
    eph.M0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cuc       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.e         = loc_getbitu(decoder->buf, i, 32) * P2_33;          i += 32;
    eph.cus       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.sqrt_A    = loc_getbitu(decoder->buf, i, 32) * P2_19;          i += 32;
    eph.toe       = loc_getbitu(decoder->buf, i, 14) * 60.0;           i += 14;
    eph.cic       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.Omega0    = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cis       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.i0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.crc       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.Omega     = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.OmegaDot  = loc_getbits(decoder->buf, i, 24) * P2_43 * PI;     i += 24;
    eph.tgdE1E5a  = loc_getbits(decoder->buf, i, 10) * P2_32;          i += 10; /* E5a/E1 */
    eph.E1health  = loc_getbitu(decoder->buf, i, 2);                   i += 2; /* E1 OSHS */
    eph.E1DVS     = loc_getbitu(decoder->buf, i, 1);                  /* i += 1;*/ /* E1 OSDVS */
  }
  else {
    return -1;
  }
  
  GalTime_t gst = { 0 };
  tm_cvt_SetGst(&gst, eph.week, eph.toe);
  tm_cvt_GstToGpst(&gst, &decoder->ephemeris.z_toeOfGpst);
  memcpy(&decoder->ephemeris.eph.z_galEph, &eph, sizeof(GalEphemeris_t));
  decoder->ephemeris.u_constellation = C_GNSS_GAL;
  decoder->q_towMsec = decoder->ephemeris.z_toeOfGpst.q_towMsec;

  return 2;
}

/* decode type 1046: galileo I/NAV satellite ephemerides (ref [17]) ----------*/
static int decode_type1046(RtcmDecoder_t* decoder)
{
  GalEphemeris_t eph = { 0 };
  uint32_t i = 24 + 12;
  if (i + 492 <= decoder->length * 8) {
    eph.svid      = loc_getbitu(decoder->buf, i, 6);                   i += 6;
    eph.week      = loc_getbitu(decoder->buf, i, 12);                  i += 12;
    eph.iode      = loc_getbitu(decoder->buf, i, 10);                  i += 10;
    eph.accuracy  = loc_getbitu(decoder->buf, i, 8);                   i += 8;
    eph.idot      = loc_getbits(decoder->buf, i, 14) * P2_43 * PI;     i += 14;
    eph.toc       = loc_getbitu(decoder->buf, i, 14) * 60.0;           i += 14;
    eph.af2       = loc_getbits(decoder->buf, i, 6) * P2_59;           i += 6;
    eph.af1       = loc_getbits(decoder->buf, i, 21) * P2_46;          i += 21;
    eph.af0       = loc_getbits(decoder->buf, i, 31) * P2_34;          i += 31;
    eph.crs       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.DeltaN    = loc_getbits(decoder->buf, i, 16) * P2_43 * PI;     i += 16;
    eph.M0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cuc       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.e         = loc_getbitu(decoder->buf, i, 32) * P2_33;          i += 32;
    eph.cus       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.sqrt_A    = loc_getbitu(decoder->buf, i, 32) * P2_19;          i += 32;
    eph.toe       = loc_getbitu(decoder->buf, i, 14) * 60.0;           i += 14;
    eph.cic       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.Omega0    = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.cis       = loc_getbits(decoder->buf, i, 16) * P2_29;          i += 16;
    eph.i0        = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.crc       = loc_getbits(decoder->buf, i, 16) * P2_05;          i += 16;
    eph.Omega     = loc_getbits(decoder->buf, i, 32) * P2_31 * PI;     i += 32;
    eph.OmegaDot  = loc_getbits(decoder->buf, i, 24) * P2_43 * PI;     i += 24;
    eph.tgdE1E5a  = loc_getbits(decoder->buf, i, 10) * P2_32;          i += 10; /* E5a/E1 */
    eph.tgdE1E5b  = loc_getbits(decoder->buf, i, 10) * P2_32;          i += 10; /* E5b/E1 */
    eph.E5bhealth = loc_getbitu(decoder->buf, i, 2);                   i += 2; /* E5b OSHS */
    eph.E5bDVS    = loc_getbitu(decoder->buf, i, 1);                   i += 1; /* E5b OSDVS */
    eph.E1health  = loc_getbitu(decoder->buf, i, 2);                   i += 2; /* E1 OSHS */
    eph.E1DVS     = loc_getbitu(decoder->buf, i, 1);                   /*i += 1;*/ /* E1 OSDVS */
  }
  else {
    return -1;
  }
  
  GalTime_t gst = { 0 };
  tm_cvt_SetGst(&gst, eph.week, eph.toe);
  tm_cvt_GstToGpst(&gst, &decoder->ephemeris.z_toeOfGpst);
  memcpy(&decoder->ephemeris.eph.z_galEph, &eph, sizeof(GalEphemeris_t));
  decoder->ephemeris.u_constellation = C_GNSS_GAL;
  decoder->q_towMsec = decoder->ephemeris.z_toeOfGpst.q_towMsec;
  return 2;
}

static gnss_ConstellationType convert_sys_type_to_constellationType(uint8_t sys_type)
{
  gnss_ConstellationType sys = C_GNSS_MAX;
  switch (sys_type)
  {
    case 0:
      sys = C_GNSS_GPS;
      break;
    case 1:
      sys = C_GNSS_GLO;
      break;
    case 2:
      sys = C_GNSS_GAL;
      break;
    case 3:  // SBAS
      break;
    case 4:
      sys = C_GNSS_QZS;
      break;
    case 5:
      sys = C_GNSS_BDS2;
      break;
    case 6: // IRNSS
      break;
    default:
      break;
  }
  return sys;
}

/* decode type 4075: Navigation data frames (NDF) message for the transport of raw GNSS messages. ----------*/
static int decode_type4075(RtcmDecoder_t* decoder)
{
  int32_t i = 24 + 12;

 /* uint16_t reference_id = loc_getbitu(decoder->buf, i, 12);*/ i+=12; // reference statin id
  i+=2; // reserved
  uint8_t frame_count = loc_getbitu(decoder->buf, i, 6); i+=6;

  for (uint8_t j = 0; j < frame_count; ++j)
  {
    NavigationData_t navi_data = {0};
    uint8_t sys          = loc_getbitu(decoder->buf, i, 4);  i+=4;
    uint8_t svid         = loc_getbitu(decoder->buf, i, 6);  i+=6;
    /*uint8_t sat_info     = loc_getbitu(decoder->buf, i, 4);*/  i+=4;
    uint8_t sig_id       = loc_getbitu(decoder->buf, i, 5);  i+=5;
    uint32_t epoch_time  = loc_getbitu(decoder->buf, i, 30); i+=30;  // ms
    /*uint8_t track        = loc_getbitu(decoder->buf, i, 1);*/  i+=1;
    uint16_t length_bits = loc_getbitu(decoder->buf, i, 12); i+=12;
    svid = svid + 1;
    sig_id = sig_id + 1;
    
    if(length_bits > MAX_NAV_DATA_FRAME_BITS) // length_bits outage
    {
      continue;
    }
    navi_data.d_tow = epoch_time;
    navi_data.sys =  convert_sys_type_to_constellationType(sys);
    navi_data.svid = svid;
    if(C_GNSS_MAX == navi_data.sys || C_GNSS_GLO == navi_data.sys) // GLO not support
    {
      continue;
    }
    if (!(C_GNSS_GPS == navi_data.sys || C_GNSS_GAL == navi_data.sys)) // TSX noly support GPS/GAL 
    {
      continue;
    }
    navi_data.signal_type = convert_signal_id_to_type(navi_data.sys, sig_id);
    if(C_GNSS_SIG_MAX == navi_data.signal_type)
    {
      continue;
    }

    /* NDF */
    //int ttt = i;
    for (uint32_t k = 0; k < length_bits; ++k, ++i)
    {
      uint8_t tmp =  loc_getbitu(decoder->buf, i, 1);
      loc_setbitu(navi_data.buffer, k, 1, tmp);
    }
    navi_data.length = length_bits / 8;
    if (length_bits % 8 != 0)
    {
      ++navi_data.length;
    }

    memcpy(navi_data.buffer_rtcm, decoder->buf, sizeof(uint8_t) * decoder->length);
    navi_data.length_rtcm = decoder->length;
    /* Report navigation data frame */
    sm_api_NavigationData_Report(&navi_data);
  }
  return 3; /*  ndf */
}


static uint64_t minimum_lock_time(uint32_t lock, uint64_t* suppleCoeff)
{
  uint64_t k = 0;
  uint64_t th = 0;
  uint64_t idx = lock / 32 - 1;

  if (/*lock >= 0 && */lock < 32)
  {
    k = 1;
    th = 0;
  }
  else
  {
    k = ((uint64_t)0x1) << idx;
    th = k * (lock - lock % 32) - k * 32;
  }

  if (suppleCoeff)
  {
    *suppleCoeff = k;
  }
  return (uint64_t)lock * k - th;
}

static uint8_t lossOfLock_v0(RtcmDecoder_t* decoder,
  uint8_t sys, uint8_t svid, uint8_t signal, uint32_t lock, uint32_t half_cycle)
{
  LossOfLock_t* ptr = NULL;
  uint8_t freq_idx = 0;
  switch (sys)
  {
  case C_GNSS_GPS:
    if (C_GNSS_SIG_GPS_L1C == signal) { freq_idx = 0; }
    else if (C_GNSS_SIG_GPS_L2C == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_GPS_L5Q == signal) { freq_idx = 2; }
    if (GPS_ID_OK(svid))
    {
      ptr = &(decoder->GpsLossOfLock[svid - 1][freq_idx]);
    }
    break;
  case C_GNSS_GLO:
    break;
  case C_GNSS_BDS2:
  case C_GNSS_BDS3:
    if (C_GNSS_SIG_BDS_B1I == signal) { freq_idx = 0; }
    else if (C_GNSS_SIG_BDS_B3I == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_BDS_B2I == signal) { freq_idx = 2; }
    else if (C_GNSS_SIG_BDS_B2A == signal) { freq_idx = 2; }
    if (BDS_ID_OK(svid))
    {
      ptr = &(decoder->BdsLossOfLock[svid - 1][freq_idx]);
    }
    break;
  case C_GNSS_GAL:
    if (C_GNSS_SIG_GAL_E1 == signal) { freq_idx = 0; }
    else if (C_GNSS_SIG_GAL_E5B == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_GAL_E5A == signal) { freq_idx = 2; }
    if (GAL_ID_OK(svid))
    {
      ptr = &(decoder->GalLossOfLock[svid - 1][freq_idx]);
    }
    break;
  case C_GNSS_QZS:
    if (C_GNSS_SIG_QZS_L1C == signal) { freq_idx = 0; }
    else if (C_GNSS_SIG_QZS_L2C == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_QZS_L5Q == signal) { freq_idx = 2; }
    if (QZS_ID_OK(svid))
    {
      ptr = &(decoder->QzsLossOfLock[svid - 1][freq_idx]);
    }
    break;
  default:
    break;
  }

  if (NULL == ptr)
  {
    return 0;
  }

  LossOfLock_t p_lockOfLock = *ptr;
  LossOfLock_t n_lockOfLock = { 0 };
  n_lockOfLock.tow_ms = decoder->q_towMsec;
  n_lockOfLock.indicator = lock;

  int lli = (!n_lockOfLock.indicator && !p_lockOfLock.indicator) || 
    (n_lockOfLock.indicator < p_lockOfLock.indicator);
  memcpy(ptr, &n_lockOfLock, sizeof(LossOfLock_t));

  if (half_cycle)
  {
    lli += 2;
  }
  return lli;
}

static uint8_t lossOfLock(RtcmDecoder_t* decoder, 
  uint8_t sys, uint8_t svid, uint8_t signal, uint32_t lock, uint32_t half_cycle)
{
  uint8_t LLI = 0;
  uint8_t b_noLossOfCon = TRUE; /* Loss Of Continuity */
  
  LossOfLock_t* ptr = NULL;
  uint8_t freq_idx = 0;
  switch (sys)
  {
  case C_GNSS_GPS: 
    if (C_GNSS_SIG_GPS_L1C == signal)      { freq_idx = 0; }
    else if (C_GNSS_SIG_GPS_L2C == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_GPS_L5Q == signal) { freq_idx = 2; }
    ptr = &(decoder->GpsLossOfLock[svid][freq_idx]);
  case C_GNSS_BDS3:
    if (C_GNSS_SIG_BDS_B1I == signal)      { freq_idx = 0; }
    else if (C_GNSS_SIG_BDS_B3I == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_BDS_B2I == signal) { freq_idx = 2; }
    else if (C_GNSS_SIG_BDS_B2A == signal) { freq_idx = 2; }
    ptr = &(decoder->BdsLossOfLock[svid][freq_idx]);
  case C_GNSS_GAL:
    if (C_GNSS_SIG_GAL_E1 == signal)       { freq_idx = 0; }
    else if (C_GNSS_SIG_GAL_E5B == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_GAL_E5A == signal) { freq_idx = 2; }
    ptr = &(decoder->GalLossOfLock[svid][freq_idx]);
  case C_GNSS_QZS:
    if (C_GNSS_SIG_QZS_L1C == signal)      { freq_idx = 0; }
    else if (C_GNSS_SIG_QZS_L2C == signal) { freq_idx = 1; }
    else if (C_GNSS_SIG_QZS_L5Q == signal) { freq_idx = 2; }
    ptr = &(decoder->QzsLossOfLock[svid][freq_idx]);
    break;
  default:
    break;
  }
  
  if (NULL == ptr)
  {
    return LLI;
  }

  LossOfLock_t p_lockOfLock = *ptr;
  LossOfLock_t n_lockOfLock = {0};

  n_lockOfLock.tow_ms = decoder->q_towMsec;
  n_lockOfLock.indicator = lock;
  
  uint64_t a_suppleCoeff = 0;
  uint64_t b_suppleCoeff = 0;
  uint64_t p_lock_ms = minimum_lock_time(p_lockOfLock.indicator, &a_suppleCoeff);
  uint64_t n_lock_ms = minimum_lock_time(n_lockOfLock.indicator, &b_suppleCoeff);
  uint64_t dt_ms = n_lockOfLock.tow_ms - p_lockOfLock.tow_ms;
  memcpy(ptr, &n_lockOfLock, sizeof(LossOfLock_t));

  if (p_lock_ms > n_lock_ms)
  {
    b_noLossOfCon = FALSE;
  }
  else if ((p_lock_ms == n_lock_ms) && (dt_ms >= a_suppleCoeff))
  {
    if (dt_ms >= a_suppleCoeff)
    {
      b_noLossOfCon = FALSE;
    }
    else
    {
      b_noLossOfCon = TRUE;
    }
  }
  else if (p_lock_ms < n_lock_ms)
  {
    if (b_suppleCoeff > p_lock_ms)
    {
      if (dt_ms >= n_lock_ms + b_suppleCoeff - p_lock_ms)
      {
        b_noLossOfCon = FALSE;
      }
      else if ((dt_ms < n_lock_ms + b_suppleCoeff - p_lock_ms) && (dt_ms >= n_lock_ms))
      {
        b_noLossOfCon = TRUE;
      }
      else if (dt_ms < n_lock_ms)
      {
        b_noLossOfCon = TRUE;
      }
    }
    else
    {
      if (dt_ms > n_lock_ms)
      {
        b_noLossOfCon = FALSE;
      }
      else
      {
        b_noLossOfCon = TRUE;
      }
    }
  }

  if ((p_lock_ms == 0) || (n_lock_ms == 0))
  {
    b_noLossOfCon = TRUE;
  }
  
  if (FALSE == b_noLossOfCon)
  {
    LLI = 0x1;
  }
  
  /** Force set LLI as 0x3 if half cycle slip happen */
  if (half_cycle)
  {
    LLI = 0x3;
  }

  return LLI;
}


/* decode type msm message header --------------------------------------------*/
static int32_t rtcm_decode_msm_head(RtcmDecoder_t* decoder,
  uint32_t sys, uint32_t* sync, uint32_t* iod, rt_msm_h_t* h, uint32_t* hsize)
{
  uint32_t q_towMsec = 0;
#if 0
  uint32_t q_towMsecGLN = 0;
  uint32_t q_dowMsec = 0;
  uint32_t q_todMsec = 0;
  uint32_t q_leapMsec = 18000;
#endif
  uint32_t i = 24;
  uint8_t j;
  uint32_t mask;
  int32_t ncell = 0;

  memset(h, 0, sizeof(rt_msm_h_t));

  /* type = loc_getbitu(decoder->buf, i, 12);*/ i += 12;
  if (i + 157 > decoder->length * 8)
  {
    return -1;
  }

  /* staid = loc_getbitu(decoder->buf, i, 12);*/ i += 12;
  switch (sys)
  {
  case C_GNSS_GPS:
  case C_GNSS_GAL:
  case C_GNSS_QZS:
    q_towMsec = loc_getbitu(decoder->buf, i, 30); i += 30;
    break;
  case C_GNSS_BDS3:
    q_towMsec = loc_getbitu(decoder->buf, i, 30); i += 30;
    q_towMsec += 14 * 1000;
    break;
  case C_GNSS_GLO:
    /* TBD */
    i += 30;
#if 0
    q_dowMsec = loc_getbitu(decoder->buf, i, 3); i += 3;
    q_todMsec = loc_getbitu(decoder->buf, i, 27); i += 27;
    q_towMsecGLN = q_dowMsec * 86400000 + q_todMsec + q_leapMsec - 3600000 * 3;
    if (q_towMsecGLN >= 604800000) q_towMsecGLN -= 604800000;
    else if (q_towMsecGLN < 0) q_towMsecGLN += 604800000;
#endif
    q_towMsec = decoder->q_towMsec;
    break;
  }

  if (q_towMsec >= 604800000) {
    q_towMsec = q_towMsec - 604800000;
  }
  decoder->q_towMsec = q_towMsec;

  *sync = loc_getbitu(decoder->buf, i, 1);        i += 1;
  *iod = loc_getbitu(decoder->buf, i, 3);         i += 3;
  h->time_s = loc_getbitu(decoder->buf, i, 7);    i += 7;
  h->clk_str = loc_getbitu(decoder->buf, i, 2);   i += 2;
  h->clk_ext = loc_getbitu(decoder->buf, i, 2);   i += 2;
  h->smooth = loc_getbitu(decoder->buf, i, 1);    i += 1;
  h->tint_s = loc_getbitu(decoder->buf, i, 3);    i += 3;
  for (j = 1; j <= 64; j++) {
    mask = loc_getbitu(decoder->buf, i, 1); i += 1;
    if (mask) {
      h->sats[h->nsat++] = j;
    }
  }
  for (j = 1; j <= 32; j++) {
    mask = loc_getbitu(decoder->buf, i, 1); i += 1;
    if (mask) {
      h->sigs[h->nsig++] = j;
    }
  }

  for (j = 0; j < h->nsat * h->nsig; j++) {
    h->cellmask[j] = loc_getbitu(decoder->buf, i, 1); i += 1;
    if (h->cellmask[j]) {
      ncell++;
    }
  }
  *hsize = i;

  return ncell;
}
/* get the index in RtcmDecoder_t acoording the constellation,prn and signal type */
static int32_t obainRtcmDataIndex(gnss_ConstellationType u_sys, uint8_t u_svId, uint8_t u_signalType, const RtcmDecoder_t* pz_rtcmData, uint8_t u_maxNumRtcmObs,BOOL* pz_isExist)
{
  int32_t q_index = -1;
  uint8_t u_i = 0;
  *pz_isExist = FALSE;
  for (u_i = 0; u_i < (pz_rtcmData->nobs); ++u_i)
  {
    if ((pz_rtcmData->obs[u_i].sys) == u_sys && (pz_rtcmData->obs[u_i].svid == u_svId) && (pz_rtcmData->obs[u_i].signal) == u_signalType)
    {
      q_index = (int32_t)u_i;
      *pz_isExist = TRUE;
      break;
    }
  }
  if (FALSE == *pz_isExist)
  {
    q_index = (int32_t)(pz_rtcmData->nobs);
  }
  if (q_index >= (int32_t)u_maxNumRtcmObs)
  {
    q_index = -1;
  }
  return q_index;
}
/* save obs data in msm message ----------------------------------------------*/
static void save_rt_msm_obs(RtcmDecoder_t* decoder,
  gnss_ConstellationType sys,
  rt_msm_h_t* h,
  const double* r,
  const double* pr,
  const double* cp,
  const double* rr,
  const double* rrf,
  const double* cnr,
  const uint32_t* lock,
  const uint32_t* ex,
  const uint32_t* half)
{
  double wl = 0.0;
  int i = 0;
  int j = 0;
  int k = 0;
  gnss_SignalType signal = C_GNSS_SIG_MAX;
  uint32_t q_towDiff = 0;
  int32_t q_index = -1;
  uint8_t u_maxNumRtcmObs = sizeof(decoder->obs) / sizeof(decoder->obs[0]);
  BOOL z_isExist = FALSE;
  BOOL q_isBDS3Sat = FALSE;
  gnss_ConstellationType u_SavedSys = sys;
  /* type = loc_getbitu(decoder->buf, 24, 12); */

  for (i = j = 0; i < h->nsat; i++)
  {
    if ((h->sats[i]) > 0)
    {
      q_towDiff = (decoder->q_towMsec) - decoder->obs[0].tow_ms;
      if (decoder->is_complete || fabs(q_towDiff) > 1e-9)
      {
        decoder->is_complete = 0;
        decoder->nobs = 0;
      }
    }
    for (k = 0; k < h->nsig; k++)
    {
      RtcmObs_t obs = { 0 };

      if (!h->cellmask[k + i * h->nsig]) 
      {
        continue;
      }

      signal = convert_signal_id_to_type(sys, h->sigs[k]);
      if (C_GNSS_BDS3 == sys || C_GNSS_BDS2 == sys)
      {
        q_isBDS3Sat = gnss_isBDS3Sat(h->sats[i], sys);
        if (TRUE == q_isBDS3Sat)
        {
          u_SavedSys = C_GNSS_BDS3;
        }
        else
        {
          u_SavedSys = C_GNSS_BDS2;
        }
      }
      q_index = obainRtcmDataIndex(u_SavedSys, h->sats[i], signal, decoder, u_maxNumRtcmObs, &z_isExist);
      if (C_GNSS_SIG_MAX != signal && q_index >= 0)
      {
        /* satellite carrier wave length */
        wl = wavelength(signal);

        obs.svid = h->sats[i];
        obs.sys = u_SavedSys;
        obs.tow_ms = decoder->q_towMsec;
        obs.snr = (uint8_t)cnr[j];
        obs.signal = signal;
        /* pseudorange (m) */
        if (r[i] != 0.0 && pr[j] > -1E12)
        {
          obs.pseudorange = r[i] + pr[j];
        }
        /* carrier-phase (cycle) */
        if (r[i] != 0.0 && cp[j] > -1E12 && wl > 0.0) 
        {
          obs.carrier_phase = (r[i] + cp[j]) / wl;
        }
        /* doppler (hz) */
        if (rr && rrf && rrf[j] > -1E12 && wl > 0.0) 
        {
#ifndef UNICORE
            obs.doppler = (float)(-(rr[i] + rrf[j]) / wl);
#else
            obs.doppler = (float)((rr[i] + rrf[j]) / wl);
#endif // !UNICORE
        }
        obs.lli = lossOfLock_v0(decoder, obs.sys, obs.svid, obs.signal, lock[j], half[j]);
        decoder->obs[q_index] = obs;
        if (FALSE == z_isExist)
        {
          ++(decoder->nobs);
        }
      }
      j++;
    }
  }
}


static int8_t decode_msm4(RtcmDecoder_t* decoder, uint8_t sys)
{
  rt_msm_h_t h = { 0 };
  double r[64] = { 0 };
  double pr[64] = { 0 };
  double cp[64] = { 0 };
  double cnr[64] = { 0 };
  uint32_t i = 24;
  int32_t j = 0;
  uint32_t sync;
  uint32_t iod;
  int32_t ncell;
  int32_t rng;
  int32_t rng_m;
  int32_t prv;
  int32_t cpv;
  uint32_t lock[64];
  uint32_t half[64];

  /* decode msm header */
  ncell = rtcm_decode_msm_head(decoder, sys, &sync, &iod, &h, &i);
  if (ncell < 0) {
    return -1;
  }

  if (i + h.nsat * 18 + ncell * 48 > decoder->length * 8) {
    return -1;
  }

  for (j = 0; j < h.nsat; j++) {
    r[j] = 0.0;
  }

  for (j = 0; j < ncell; j++) {
    pr[j] = -1E16;
    cp[j] = -1E16;
  }
  
  /* decode satellite data */
  for (j = 0; j < h.nsat; j++) {
    /* rough ranges integer milliseconds (1 ms) */
    rng = loc_getbitu(decoder->buf, i, 8); i += 8;
    if (rng != 255) {
      r[j] = rng * CLIGHT_RANGE_MSEC;
    }
  }
  for (j = 0; j < h.nsat; j++) {
    /* rough range modulo 1 millisecond (2e-10 ms)*/
    rng_m = loc_getbitu(decoder->buf, i, 10); i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * CLIGHT_RANGE_MSEC;
    }
  }

  /* decode signal data */
  for (j = 0; j < ncell; j++) { /* pseudorange */
    prv = loc_getbits(decoder->buf, i, 15); i += 15;
    if (prv != -16384) pr[j] = prv * P2_24 * CLIGHT_RANGE_MSEC;
  }
  for (j = 0; j < ncell; j++) { /* phaserange */
    cpv = loc_getbits(decoder->buf, i, 22); i += 22;
    if (cpv != -2097152) cp[j] = cpv * P2_29 * CLIGHT_RANGE_MSEC;
  }
  for (j = 0; j < ncell; j++) { /* lock time */
    lock[j] = loc_getbitu(decoder->buf, i, 4); i += 4;
  }
  for (j = 0; j < ncell; j++) { /* half-cycle ambiguity */
    half[j] = loc_getbitu(decoder->buf, i, 1); i += 1;
  }
  for (j = 0; j < ncell; j++) { /* cnr */
    cnr[j] = loc_getbitu(decoder->buf, i, 6) * 1.0; i += 6;
  }

  /* save obs data in msm message */
  save_rt_msm_obs(decoder, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);
  if (!sync)
  {
    decoder->is_complete = 1;
  }

  return 1;
}

static int8_t decode_msm5(RtcmDecoder_t* decoder, uint8_t sys)
{
  rt_msm_h_t h = { 0 };
  static double r[64];
  static double rr[64];
  static double pr[64];
  static double cp[64];
  static double rrf[64];
  static double cnr[64];
  uint32_t i = 24;
  int32_t j = 0;
  uint32_t sync;
  uint32_t iod;
  int32_t ncell;
  int32_t rng;
  int32_t rng_m;
  int32_t rate;
  int32_t prv;
  int32_t cpv;
  int32_t rrv;
  uint32_t lock[64];
  uint32_t ex[64];
  uint32_t half[64];

  memset(r, 0, sizeof(r));
  memset(rr, 0, sizeof(rr));
  memset(pr, 0, sizeof(pr));
  memset(cp, 0, sizeof(cp));
  memset(rrf, 0, sizeof(rrf));
  memset(cnr, 0, sizeof(cnr));
  /* decode msm header */
  ncell = rtcm_decode_msm_head(decoder, sys, &sync, &iod, &h, &i);
  if (ncell < 0) {
    return -1;
  }

  if (i + h.nsat * 36 + ncell * 63 > decoder->length * 8) {
    return -1;
  }

  for (j = 0; j < h.nsat; j++) {
    r[j] = 0.0;
    rr[j] = 0.0;
    ex[j] = 15;
  }

  for (j = 0; j < ncell; j++) {
    pr[j] = -1E16;
    cp[j] = -1E16;
    rrf[j] = -1E16;
  }

  /* decode satellite data part */
  for (j = 0; j < h.nsat; j++)
  {
    /* rough ranges integer milliseconds (1 ms) */
    rng = loc_getbitu(decoder->buf, i, 8); i += 8;
    if (rng != 255) {
      r[j] = rng * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < h.nsat; j++)
  {
    /* extended information specific for each GNSS */
    ex[j] = loc_getbitu(decoder->buf, i, 4); i += 4;
  }

  for (j = 0; j < h.nsat; j++)
  {
    /* rough range modulo 1 millisecond (2e-10 ms)*/
    rng_m = loc_getbitu(decoder->buf, i, 10); i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < h.nsat; j++)
  {
    /* rough Phase Range Rate (1 m/s)*/
    rate = loc_getbits(decoder->buf, i, 14); i += 14;
    if (rate != -8192) {
      rr[j] = rate * 1.0;
    }
  }

  /* decode signal data part */
  for (j = 0; j < ncell; j++) {
    /* psudorange
       Range:[2e-10, 2e-24]=292m Unit:(2e-24 ms)=0.018m */
    prv = loc_getbits(decoder->buf, i, 15); i += 15;
    if (prv != -16384) {
      pr[j] = prv * P2_24 * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < ncell; j++) {
    /* phaserange
       Range:[2e-8, 2e-29]=1171m Unit:(2e-29 ms)=0.0006m */
    cpv = loc_getbits(decoder->buf, i, 22); i += 22;
    if (cpv != -2097152) {
      cp[j] = cpv * P2_29 * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < ncell; j++) {
    /* GNSS PhaseRange Lock Time Indicator */
    lock[j] = loc_getbitu(decoder->buf, i, 4); i += 4;
  }

  for (j = 0; j < ncell; j++) {
    /* half-cycle amiguity
      0 - No half-cycle ambiguity. 1 - Half-cycle ambiguity */
    half[j] = loc_getbitu(decoder->buf, i, 1); i += 1;
  }

  for (j = 0; j < ncell; j++)
  {
    /* GNSS signal CNRs */
    cnr[j] = loc_getbitu(decoder->buf, i, 6); i += 6;
  }

  for (j = 0; j < ncell; j++) {
    /* GNSS signal fine PhaseRangeRates */
    rrv = loc_getbits(decoder->buf, i, 15); i += 15;
    if (rrv != -16384)
    {
      rrf[j] = rrv * 0.0001;
    }
  }

  /* save obs data in msm message */
  save_rt_msm_obs(decoder, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);
  if (!sync)
  {
    decoder->is_complete = 1;
  }

  return 1;
}

static int8_t decode_msm7(RtcmDecoder_t* decoder, uint8_t sys)
{
  rt_msm_h_t h = { 0 };
  static double r[64];
  static double rr[64];
  static double pr[64];
  static double cp[64];
  static double rrf[64];
  static double cnr[64];
  uint32_t i = 24;
  int32_t j = 0;
  uint32_t sync;
  uint32_t iod;
  int32_t ncell;
  int32_t rng;
  int32_t rng_m;
  int32_t rate;
  int32_t prv;
  int32_t cpv;
  int32_t rrv;
  uint32_t lock[64];
  uint32_t ex[64];
  uint32_t half[64];

  memset(r, 0,sizeof(r));
  memset(rr, 0,sizeof(rr));
  memset(pr, 0,sizeof(pr));
  memset(cp, 0,sizeof(cp));
  memset(rrf, 0,sizeof(rrf));
  memset(cnr, 0,sizeof(cnr));
  /* decode msm header */
  ncell = rtcm_decode_msm_head(decoder, sys, &sync, &iod, &h, &i);
  if (ncell < 0) {
    return -1;
  }

  if (i + h.nsat * 36 + ncell * 80 > decoder->length * 8) {
    return -1;
  }

  for (j = 0; j < h.nsat; j++) {
    r[j] = 0.0;
    rr[j] = 0.0;
    ex[j] = 15;
  }

  for (j = 0; j < ncell; j++) {
    pr[j] = -1E16;
    cp[j] = -1E16;
    rrf[j] = -1E16;
  }

  /* decode satellite data part */
  for (j = 0; j < h.nsat; j++)
  {
    /* rough ranges integer milliseconds (1 ms) */
    rng = loc_getbitu(decoder->buf, i, 8); i += 8;
    if (rng != 255) {
      r[j] = rng * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < h.nsat; j++)
  {
    /* extended information specific for each GNSS */
    ex[j] = loc_getbitu(decoder->buf, i, 4); i += 4;
  }

  for (j = 0; j < h.nsat; j++)
  {
    /* rough range modulo 1 millisecond (2e-10 ms)*/
    rng_m = loc_getbitu(decoder->buf, i, 10); i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < h.nsat; j++)
  {
    /* rough Phase Range Rate (1 m/s)*/
    rate = loc_getbits(decoder->buf, i, 14); i += 14;
    if (rate != -8192) {
      rr[j] = rate * 1.0;
    }
  }

  /* decode signal data part */
  for (j = 0; j < ncell; j++) {
    /* psudorange with extend resolution
       Range:[2e-10, 2e-29]=292m Unit:(2e-29 ms)=0.0006m */
    prv = loc_getbits(decoder->buf, i, 20); i += 20;
    if (prv != -524288) {
      pr[j] = prv * P2_29 * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < ncell; j++) {
    /* phaserange with extend resolution Range:[2e-8, 2e-31]=1171m Unit:(2e-31 ms)=0.00014m */
    cpv = loc_getbits(decoder->buf, i, 24); i += 24;
    if (cpv != -8388608) {
      cp[j] = cpv * P2_31 * CLIGHT_RANGE_MSEC;
    }
  }

  for (j = 0; j < ncell; j++) {
    /* GNSS PhaseRange Lock Time Indicator with extended range and resolution */
    lock[j] = loc_getbitu(decoder->buf, i, 10); i += 10;
  }

  for (j = 0; j < ncell; j++) {
    /* half-cycle amiguity
      0 - No half-cycle ambiguity. 1 - Half-cycle ambiguity */
    half[j] = loc_getbitu(decoder->buf, i, 1); i += 1;
  }

  for (j = 0; j < ncell; j++)
  {
    /* GNSS signal CNRs */
    cnr[j] = loc_getbitu(decoder->buf, i, 10) * 0.0625; i += 10;
  }

  for (j = 0; j < ncell; j++) {
    /* GNSS signal fine PhaseRangeRates */
    rrv = loc_getbits(decoder->buf, i, 15); i += 15;
    if (rrv != -16384) 
    {
      rrf[j] = rrv * 0.0001;
    }
  }

  /* save obs data in msm message */
  save_rt_msm_obs(decoder, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);
  if (!sync)
  {
    decoder->is_complete = 1;
  }

  return 1;
}

static int8_t rtcm_decoder(RtcmDecoder_t* decoder)
{
  int8_t ret = -1;
  uint32_t type_id = loc_getbitu(decoder->buf, 24, 12);

  if (decoder->is_complete == 1) {
    decoder->is_complete = 0;
    decoder->nobs = 0;
    memset(decoder->obs, 0, sizeof(decoder->obs));
  }
  
  decoder->type_id = type_id;
  switch (type_id)
  {
    case 1005: ret = decode_type1005(decoder); break;
    case 1006: ret = decode_type1006(decoder); break;
    case 1019: ret = decode_type1019(decoder); break;
    case 1042: ret = decode_type1042(decoder); break;
    case 1044: ret = decode_type1044(decoder); break;
    case 1045: ret = decode_type1045(decoder); break;
    case 1046: ret = decode_type1046(decoder); break;
    case 4075: ret = decode_type4075(decoder); break;
    case 1074: ret = decode_msm4(decoder, C_GNSS_GPS); break;
    case 1075: ret = decode_msm5(decoder, C_GNSS_GPS); break;
    case 1077: ret = decode_msm7(decoder, C_GNSS_GPS); break;
    //case 1084: ret = decode_msm4(decoder, C_GNSS_GLO); break;
    case 1085: ret = decode_msm5(decoder, C_GNSS_GLO); break;
    case 1087: ret = decode_msm7(decoder, C_GNSS_GLO); break;
    case 1094: ret = decode_msm4(decoder, C_GNSS_GAL); break;
    case 1095: ret = decode_msm5(decoder, C_GNSS_GAL); break;
    case 1097: ret = decode_msm7(decoder, C_GNSS_GAL); break;
    case 1114: ret = decode_msm4(decoder, C_GNSS_QZS); break;
    case 1115: ret = decode_msm5(decoder, C_GNSS_QZS); break;
    case 1117: ret = decode_msm7(decoder, C_GNSS_QZS); break;
    case 1124: ret = decode_msm4(decoder, C_GNSS_BDS3); break;
    case 1125: ret = decode_msm5(decoder, C_GNSS_BDS3); break;
    case 1127: ret = decode_msm7(decoder, C_GNSS_BDS3); break;
    default:break;
  }

  return ret;
}

static BOOL e2e_dec_check(uint8_t* data)
{
  uint32_t E2ePremable = 0;

  E2ePremable = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3] << 0);
  if (E2EPRPREAMB == E2ePremable)
  {
    return TRUE;
  }
  return FALSE;
}

static int8_t rtcm_dec_input_frame(RtcmDecoder_t* decoder, uint8_t data)
{
  /* Synchronize frame */
  if (decoder->nbyte == 0)
  {
    if (RTCM3PREAMB == data)
    {
      decoder->buf[decoder->nbyte++] = data;
    }
    return 0;
  }

  if (decoder->nbyte == 1)
  {
    if (0x00 == (data & 0xFC))
    {
      decoder->buf[decoder->nbyte++] = data;
      return 0;
    }
    else
    {
      LOGI(TAG_SM, "Wrong D3 Premable Buffer 0x%02x 0x%02x\n",
           decoder->buf[0], data);
      decoder->nbyte = 0;
      return -2;
    }
  }

  if (decoder->nbyte > RTCM_BUF_SIZE)
  {
    decoder->nbyte = 0;
    return -3;
  }

  decoder->buf[decoder->nbyte++] = data;

  if (decoder->nbyte == RTCM_PRECHECK_LEN)
  {
    decoder->length = loc_getbitu(decoder->buf, 14, 10);
    decoder->type_id = loc_getbitu(decoder->buf, 24, 12);
  }

  if (decoder->nbyte < RTCM_PRECHECK_LEN || decoder->nbyte < decoder->length + RTCM_PRECHECK_LEN) {
    return 0;
  }
  decoder->length += 3;
  decoder->nbyte = 0;

  /* check parity */
  if (loc_crc24q(decoder->buf, decoder->length) !=
    loc_getbitu(decoder->buf, (decoder->length) * 8, 24))
  {
    LOGI(TAG_SM, "Wrong crc24q\n");
    return -4;
  }

  /* decode rtcm3 message */
  return rtcm_decoder(decoder);
}

/**
 * @brief discard firtst 1 byte, re decode again
 * @param[in,out] decoder
 * @param[in,out] data
 * @return
*/
static int8_t rtcm_dec_input_fixed(RtcmDecoder_t* decoder, uint8_t data)
{
  int8_t ret = -1;
  uint8_t stash_buf[RTCM_BUF_SIZE] = {0};
  uint32_t stash_length = decoder->length + 2;
  memmove(stash_buf, decoder->buf + 1, stash_length);
  stash_buf[stash_length] = '\0';
  memset(decoder, 0, sizeof(RtcmDecoder_t));
  for (uint32_t j = 0; j < stash_length; j++)
  {
    ret = rtcm_dec_input_frame(decoder, stash_buf[j]);
  }
  return ret;
}

int8_t rtcm_dec_input(RtcmDecoder_t* decoder, uint8_t data)
{
  int8_t ret = rtcm_dec_input_frame(decoder, data);
  if(-4 == ret)
  {
    ret = rtcm_dec_input_fixed(decoder, data);
  }
  return ret;
}
static BOOL tailBuffCheck(uint32_t i, uint32_t length, const uint8_t* buff)
{
  uint32_t j = 0;
  uint32_t k = 0;
  uint8_t e2epre[4] = { 0x0F, 0xF0, 0x5A, 0x03 };    /*267409923 */
  if (i >= length)  // it is no enough 16 bytes for e2e 
  {
    return TRUE;
  }
  
  if ((i + E2EMSGHEADLEN) > length) // it is no enough 4 bytes for e2e header
  {
    for (j = 0; j < (length - i); j++)
    {
      if (0x0F != buff[j])
      {
        continue;
      }
      for (k = j; k < (length - i); k++)
      {
        if (e2epre[k-j] != buff[k]) 
        {
          break;
        }
      }
      if (k == (length - i))
      {
        return TRUE;
      }
      
    }
  }
  return FALSE;

}

/**
 * @brief Component for skip e2e msg from GNSS stream
 * @param[in] decoder,  decode information
 * @param[in] inputbuf, GNSS stream
 * @param[in] inlen, length of inputbuf
 * @param[out] outputbuf, skip result for next decoding
 * @param[out] outlen, length of outputbuf
 * @return BOOL, false: do not decode next; ture: do decoding
 * @note: inlen nedded < 1030
 */
static BOOL rtcmE2eSkipComponent(RtcmDecoder_t* decoder, const uint8_t* inputbuf, uint32_t inlen, uint8_t* outputbuf, uint32_t* outlen)
{
  if (NULL == inputbuf || NULL == outputbuf || inlen <= 0)
  {
    return FALSE;
  }
  uint32_t i = 0;
  uint32_t j = 0;
  uint32_t pos = 0;
  uint32_t length = 0;
  BOOL is_e2e_data = FALSE;
  uint8_t* e2elen = &decoder->e2emsglen;
  uint8_t* e2ebuf = decoder->e2emsgbuf;

  *outlen = 0;
  // Too less nbytes for e2e type, stored in e2e_buffer, and return false
  if ((*e2elen + inlen) < E2EMSGHEADLEN)
  {
    for (i = 0; i < inlen; i++)
    {
      e2ebuf[(*e2elen)++] = inputbuf[i];
    }
  }
  else
  {
    // Combined data of previous stored and ipc data
    uint8_t* buffer = (uint8_t*)OS_MALLOC_FAST(sizeof(uint8_t) * (*e2elen + inlen + 1));
    for (i = 0; i < (*e2elen); i++)
    {
      buffer[length++] = e2ebuf[i];
    }
    for (i = 0; i < inlen; i++)
    {
      buffer[length++] = inputbuf[i];
    }

    // skip running
    for (i = 0; i < length; i++)
    {
      pos = i;
      if (i + E2EMSGHEADLEN <= length)
      {
        is_e2e_data = e2e_dec_check(&buffer[i]); // skip e2e
        if (is_e2e_data == TRUE)
        {
          if (decoder->is_e2e_data < 100) decoder->is_e2e_data++;
          for (j = i; j < length && j < (i + 16); j++)
          {
            //LOGD(TAG_SM, "e2e checked: i=%d data=%02x \n", j, buffer[j]);
          }
          i += E2EMSGMAXLEN;
        }
        if (i >= length) // i += E2EMAXLEN is more than length
        {
          if ((i - E2EMSGMAXLEN) > 0) pos = i - E2EMSGMAXLEN;
          else pos = 0;
        }
      }
      if (tailBuffCheck(i, length, &buffer[i])) // check last 4 bytes 
      {
        (*e2elen) = 0;
        for (j = pos; j < length; j++)
        {
          e2ebuf[(*e2elen)++] = buffer[j];
        }
        break;
      }
      (*e2elen) = 0;
      outputbuf[(*outlen)++] = buffer[i];
      if (*outlen >= inlen)
      {
        break;
      }
      //LOGD(TAG_SM, "E2E input_rtcm3: i=%d data=%02x\n", i, buffer[i]);
    }
    OS_FREE(buffer);
  }

  if (decoder->is_e2e_data<3)  // do skip or not
  {
    *outlen = inlen;
    memcpy(outputbuf, inputbuf, sizeof(uint8_t) * inlen);
    return TRUE;
  }

  if ((*outlen) > 0)
  {
    return TRUE;
  }

  return FALSE;
}

/**
 * @brief  skip e2e msg from GNSS stream
 * @param[in] decoder,  decode information
 * @param[in] inputbuf, GNSS stream
 * @param[in] inlen, length of inputbuf
 * @param[out] outputbuf, skip result for next decoding
 * @param[out] outlen, length of outputbuf
 * @return BOOL, false: do not decode next; ture: do decoding
 */
BOOL rtcmE2eSkip(RtcmDecoder_t* decoder, const uint8_t* inputbuf, uint32_t inlen, uint8_t* outputbuf, uint32_t* outlen)
{ 
  uint32_t q_len = 0;
  uint32_t q_segment_length = 0;

  /* 1030 loop */
  for (uint32_t i = 0; i < inlen; i += RTCM_BUF_SIZE)
  {
    q_len = 0;
    q_segment_length = (i + RTCM_BUF_SIZE <= inlen) ? RTCM_BUF_SIZE : (inlen - i);
    rtcmE2eSkipComponent(decoder, inputbuf + i, q_segment_length, outputbuf + *outlen, &q_len);
    *outlen += q_len;
  }
  if ((*outlen) > 0)
  {
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief  Extract GnssMeasBlock from RTCM Decoder to GnssMeasBlock
 * @return none
 */
void rtcm_ExtractMeasBlk(RtcmDecoder_t* rtcm, GnssMeasBlock_t* meas_blk)
{
  uint8_t u_MaxMeasCount = sizeof(meas_blk->z_meas)/ sizeof(meas_blk->z_meas[0]);

  if (any_Ptrs_Null(2, rtcm, meas_blk))
  {
    return;
  }

  meas_blk->u_version = VERSION_GNSS_MEAS_BLOCK;
  meas_blk->w_size = sizeof(GnssMeasBlock_t);
  meas_blk->z_Clock.z_gpsTime.w_week = rtcm->w_gpsWeek;
  meas_blk->z_Clock.z_gpsTime.q_towMsec = rtcm->q_towMsec;

  for (uint32_t i = 0; i < rtcm->nobs; i++)
  {
    if (i >= u_MaxMeasCount || (meas_blk->w_measCount)>= u_MaxMeasCount)
    {
      LOGI(TAG_SM, "Meas Index %d more than Max %d\n", i, u_MaxMeasCount);
      break;
    }

    RtcmObs_t* p_obs = &rtcm->obs[i];
    GnssMeas_t* p_meas = &meas_blk->z_meas[meas_blk->w_measCount];
    meas_blk->w_measCount++;

    p_meas->z_measStatusFlag.b_valid = TRUE;
    p_meas->z_measStatusFlag.b_prValid = TRUE;
    p_meas->z_measStatusFlag.b_drValid = TRUE;
    p_meas->z_measStatusFlag.b_cpValid = TRUE;
    p_meas->u_constellation = p_obs->sys;
    p_meas->u_svid = p_obs->svid;
    p_meas->u_signal = p_obs->signal;
    p_meas->u_LLI = p_obs->lli;
    p_meas->f_cn0 = p_obs->snr;
    p_meas->d_pseudoRange = p_obs->pseudorange;
    p_meas->d_doppler = p_obs->doppler * wavelength(p_obs->signal) * (-1.0);
    p_meas->d_carrierPhase = p_obs->carrier_phase; 
  }
  if (!gnss_AdjustMeasBlockWeek(meas_blk->w_measCount, meas_blk->z_meas, &meas_blk->z_Clock.z_gpsTime)) // GPS WEEK
  {
    LOGW(TAG_SM, "Adjust meas GPS week failed week=%d tow=%.0f\n", meas_blk->z_Clock.z_gpsTime.w_week,
      meas_blk->z_Clock.z_gpsTime.q_towMsec * TIME_MSEC_INV);
  }
  return;
}

/**
 * @brief  Extract GnssCorrBlock from RTCM Decoder to GnssMeasBlock
 * @return none
 */
void rtcm_ExtractCorrBlk(RtcmDecoder_t* rtcm, GnssCorrBlock_t* corr_blk)
{
  uint8_t u_MaxMeasCount = 0;
  if (any_Ptrs_Null(2, rtcm, corr_blk))
  {
    return;
  }
  u_MaxMeasCount = sizeof(corr_blk->z_meas) / sizeof(corr_blk->z_meas[0]);

  corr_blk->u_version = VERSION_GNSS_MEAS_BLOCK;
  corr_blk->w_size = sizeof(GnssMeasBlock_t);
  corr_blk->w_refStationId = rtcm->satInfo.w_staid;
  corr_blk->d_refPosEcef[0] = rtcm->satInfo.d_staPos[0];
  corr_blk->d_refPosEcef[1] = rtcm->satInfo.d_staPos[1];
  corr_blk->d_refPosEcef[2] = rtcm->satInfo.d_staPos[2];
  corr_blk->z_Clock.z_gpsTime.w_week = rtcm->w_gpsWeek;
  corr_blk->z_Clock.z_gpsTime.q_towMsec = rtcm->q_towMsec;
  corr_blk->w_measCount = rtcm->nobs;

  for (uint32_t i = 0, j = 0; i < rtcm->nobs; i++)
  {
    RtcmObs_t* p_obs = &rtcm->obs[i];
    if (j >= MAX_GNSS_TRK_MEAS_NUMBER)
    {
      LOGI(TAG_SM, "Meas Index %d more than Max %d\n", j, u_MaxMeasCount);
      break;
    }
    GnssMeas_t* p_meas = &corr_blk->z_meas[j];

    p_meas->z_measStatusFlag.b_valid = TRUE;
    p_meas->z_measStatusFlag.b_prValid = TRUE;
    p_meas->z_measStatusFlag.b_drValid = TRUE;
    p_meas->z_measStatusFlag.b_cpValid = TRUE;
    p_meas->u_constellation = p_obs->sys;
    p_meas->u_svid = p_obs->svid;
    p_meas->u_signal = p_obs->signal;
    p_meas->u_LLI = p_obs->lli;
    p_meas->f_cn0 = p_obs->snr;
    p_meas->d_pseudoRange = p_obs->pseudorange;
    p_meas->d_doppler = p_obs->doppler * wavelength(p_obs->signal) * (-1.0);
    p_meas->d_carrierPhase = p_obs->carrier_phase;
    j++;
  }
  if (!gnss_AdjustMeasBlockWeek(corr_blk->w_measCount, corr_blk->z_meas, &corr_blk->z_Clock.z_gpsTime)) // GPS WEEK
  {
    LOGW(TAG_SM, "Adjust meas GPS week failed week=%d tow=%.0f\n", corr_blk->z_Clock.z_gpsTime.w_week,
      corr_blk->z_Clock.z_gpsTime.q_towMsec * TIME_MSEC_INV);
  }
  return;
}
