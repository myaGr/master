/**@file        ublox_dec.c
 * @brief       Ublox stream decode module
 * @author      chenyang
 * @date        2023/08/23
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/08/23  <td>0.1      <td>chenyang   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
 #include "cmn_utils.h"
 #include "gnss_def.h"
 #include "ubx_dec.h" 
 #include "gnss_common.h"
 #include "mw_log.h"
 
 
 #define ROUND(x)    (int)floor((x)+0.5)

 /* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char* p) { unsigned short u; memcpy(&u, p, 2); return u; }
//static short          I2(unsigned char* p) { short          i; memcpy(&i, p, 2); return i; }
static unsigned int   U4(unsigned char* p) { unsigned int   u; memcpy(&u, p, 4); return u; }
//static int            I4(unsigned char* p) { int            u; memcpy(&u, p, 4); return u; }
static float          R4(unsigned char* p) { float          r; memcpy(&r, p, 4); return r; }
static double         R8(unsigned char* p) { double         r; memcpy(&r, p, 8); return r; }
//static double         I8(unsigned char* p) { return I4(p + 4) * 4294967296.0 + U4(p); }

static const double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */
static const double gst0[] = { 1999,8,22,0,0,0 }; /* galileo system time reference */
static const double bdt0[] = { 2006,1, 1,0,0,0 }; /* beidou time reference */

const unsigned int _tbl_CRC24Q[] = {
  0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
  0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
  0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
  0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
  0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
  0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
  0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
  0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
  0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
  0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
  0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
  0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
  0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
  0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
  0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
  0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
  0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
  0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
  0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
  0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
  0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
  0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
  0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
  0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
  0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
  0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
  0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
  0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
  0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
  0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
  0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
  0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
unsigned int _rtk_crc24q(const unsigned char* buff, int len)
{
  unsigned int crc = 0;
  int i;


  for (i = 0; i < len; i++) crc = ((crc << 8) & 0xFFFFFF) ^ _tbl_CRC24Q[(crc >> 16) ^ buff[i]];
  return crc;
}
extern gtime_t ag_gnss_epoch2time(const double* ep)
{
  const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
  gtime_t time = { 0 };
  int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

  if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

  /* leap year if year%4==0 in 1901-2099 */
  days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
  sec = (int)floor(ep[5]);
  time.time = (time_t)days * 86400 + ((time_t)ep[3]) * 3600 + ((time_t)ep[4]) * 60 + (time_t)sec;
  time.sec = ep[5] - sec;
  return time;
}
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void ag_gnss_time2epoch(gtime_t t, double* ep)
{
  const int mday[] = { /* # of days in a month */
      31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
      31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
  };
  int days, sec, mon, day;

  /* leap year if year%4==0 in 1901-2099 */
  days = (int)(t.time / 86400);
  sec = (int)(t.time - (time_t)days * 86400);
  for (day = days % 1461, mon = 0; mon < 48; mon++) {
    if (day >= mday[mon]) day -= mday[mon]; else break;
  }
  ep[0] = 1970.0 + days / 1461 * 4.0 + mon / 12; ep[1] = mon % 12 + 1.0; ep[2] = day + 1.0;
  ep[3] = sec / 3600; ep[4] = sec % 3600 / 60; ep[5] = sec % 60 + t.sec;
}
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t ag_gnss_gpst2time(int week, double sec)
{
  gtime_t t = ag_gnss_epoch2time(gpst0);

  if (sec < -1E9 || 1E9 < sec) sec = 0.0;
  t.time += (time_t)86400 * 7 * week + (int)sec;
  t.sec = sec - (int)sec;
  return t;
}
double ag_gnss_time2gpst(gtime_t t, int* week)
{
  gtime_t t0 = ag_gnss_epoch2time(gpst0);
  time_t sec = t.time - t0.time;
  int32_t w = (int)(sec / (86400 * 7));

  if (week) *week = w;
  return (double)(sec - w * 86400.0 * 7.0) + t.sec;
}
/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t ag_gnss_bdt2time(int week, double sec)
{
  gtime_t t = ag_gnss_epoch2time(bdt0);

  if (sec < -1E9 || 1E9 < sec) sec = 0.0;
  t.time += (time_t)86400 * 7 * week + (time_t)sec;
  t.sec = sec - (int)sec;
  return t;
}
/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
double ag_gnss_time2bdt(gtime_t t, int* week)
{
  gtime_t t0 = ag_gnss_epoch2time(bdt0);
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));

  if (week) *week = w;
  return (double)(sec - w * 86400.0 * 7.0) + t.sec;
}
/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t ag_gnss_timeadd(gtime_t t, double sec)
{
  double tt;

  t.sec += sec;
  tt = floor(t.sec);
  t.time += (int)tt;
  t.sec -= tt;
  return t;
}
/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t ag_gnss_gpst2bdt(gtime_t t)
{
  return ag_gnss_timeadd(t, -14.0);
}
/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
extern gtime_t ag_gnss_bdt2gpst(gtime_t t)
{
  return ag_gnss_timeadd(t, 14.0);
}
 /* checksum ------------------------------------------------------------------*/
static int checksum(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}
#if 0
static void setcs(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    buff[len-2]=cka;
    buff[len-1]=ckb;
}
#endif
/* obs search */
static int obs_search(UbloxObs_t*pdata, int num, int sat, int *p_match)
{
    int i;

    if (p_match)
    {
        *p_match = FALSE;
    }

    for (i = 0; i < num; i++)
    {
        if (pdata[i].sat == sat)
        {
            if (p_match)
            {
                *p_match = TRUE;
            }
            break;
        }
    }

    return i;
}
extern double ag_gnss_getwavelen(int sys, int frq, int fcn)
{
  if (sys == C_GNSS_GLO) {
  }
  else if (sys == C_GNSS_BDS3) {
    if (frq == 0) return CLIGHT / FREQ1_CMP; /* B1 */
    else if (frq == 1) return CLIGHT / FREQ2_CMP; /* B2 */
    else if (frq == 2) return CLIGHT / FREQ5; /* B3 */
  }
  else if (sys == C_GNSS_GAL) {
    if (frq == 0) return CLIGHT / FREQ1; /* L1/E1 */
    else if (frq == 1) return CLIGHT / FREQ7; /* E5b */
    else if (frq == 2) return CLIGHT / FREQ5; /* L5/E5a */
  }
  else {
    if (frq == 0) return CLIGHT / FREQ1; /* L1/E1 */
    else if (frq == 1) return CLIGHT / FREQ2; /* L2 */
    else if (frq == 2) return CLIGHT / FREQ5; /* L5/E5a */
    else if (frq == 3) return CLIGHT / FREQ6; /* L6/LEX */
    else if (frq == 4) return CLIGHT / FREQ7; /* E5b */
    else if (frq == 5) return CLIGHT / FREQ8; /* E5a+b */
    else if (frq == 6) return CLIGHT / FREQ9; /* S */
  }
  return 0.0;
}
/* Sat frequency */
static double sys_freq(int sys, int frq, int glo_chn)
{
	double lam= ag_gnss_getwavelen(sys, frq, glo_chn);

	if (lam > 0.0) return CLIGHT/lam;

    return 0.0;
}
#if 0
/* Ublox Signal Identifiers to Freq Idx */
static int ubx_sig2freq(int sigid, int sys)
{
    int ret_val = -1;

    switch (sys) 
    {
    case SYS_GPS:
        switch (sigid)
        {
        case 0: 
            ret_val = 0;
            break;

        case 3:
        case 4:
            ret_val = 1;
        }
        break;

    case SYS_SBS:
        ret_val = 0;
        break;

    case SYS_GAL:
        switch (sigid)
        {
        case 0:
        case 1:
            ret_val = 0;
            break;

        case 5:
        case 6:
            ret_val = 1; /* E5b Freq Index = 1 */
        }
        break;

    case SYS_CMP:
        switch (sigid)
        {
        case 0:
        case 1:
            ret_val = 0;
            break;

        case 2:
        case 3:
            ret_val = 1; 
        }
        break;

    case SYS_QZS:
        switch (sigid)
        {
        case 0:
            ret_val = 0;
            break;

        case 5:
            ret_val = 1;
        }
        break;

    case SYS_GLO:
        switch (sigid)
        {
        case 0:
            ret_val = 0;
            break;

        case 2:
            ret_val = 1;
        }
        break;
    }

    return ret_val;
}
/* UBX GNSSId to system_2.0 (ref [2] 25) -----------------------------------------*/
static int ublox_sys(int gnssid)
{
  switch (gnssid) {
  case 0: return C_GNSS_GPS;
  //case 1: return SYS_SBS;
  case 1: return C_GNSS_MAX;/////SBS
  case 2: return C_GNSS_GAL;
  case 3: return C_GNSS_BDS3;
  case 5: return C_GNSS_QZS;
  case 6: return C_GNSS_GLO;
  }
  return 0;
}
#endif
/* satellite carrier wave length -----------------------------------------------
* author : none
* args   : int    sys       I   satellite number
*          int    frq       I   frequency index (0:L1,1:L2,2:L5/3,...)
*          int    fcn       I   glonass frequency channel number [-7, 6]
* return : carrier wave length (m) (0.0: error)
*-----------------------------------------------------------------------------*/
extern double ubx_getwavelen(int sys, int frq)
{
  if (sys == C_GNSS_GPS) {
    if (frq == 0) return CLIGHT / GPS_L1C_FREQ; /* L1 */
    else if (frq == 1) return CLIGHT / GPS_L2C_FREQ; /* L2 */
    else if (frq == 2) return CLIGHT / GPS_L5Q_FREQ; /* L5 */
  }
  else if (sys == C_GNSS_BDS3) {
    if (frq == 0) return CLIGHT / BDS_B1I_FREQ; /* B1 */
    else if (frq == 1) return CLIGHT / BDS_B2I_FREQ; /* B2 */
    else if (frq == 2) return CLIGHT / BDS_B3I_FREQ; /* B3 */
  }
  else if (sys == C_GNSS_GAL) {
    if (frq == 0) return CLIGHT / GAL_E1_FREQ; /* L1/E1 */
    else if (frq == 1) return CLIGHT / GAL_E5B_FREQ; /* E5b */
    else if (frq == 2) return CLIGHT / GAL_E5A_FREQ; /* L5/E5a */
  }
  else {
    if (frq == 0) return CLIGHT / GPS_L1C_FREQ; /* L1/E1 */
    else if (frq == 1) return CLIGHT / GPS_L2C_FREQ; /* L2 */
    else if (frq == 2) return CLIGHT / GPS_L5Q_FREQ; /* L5/E5a */
    else if (frq == 3) return CLIGHT / FREQ6; /* L6/LEX */
    else if (frq == 4) return CLIGHT / GAL_E5B_FREQ; /* E5b */
    else if (frq == 5) return CLIGHT / GAL_E5A_FREQ; /* E5a+b */
    else if (frq == 6) return CLIGHT / FREQ9; /* S */
  }
  return 0.0;
}

extern double ag_gnss_timediff(gtime_t t1, gtime_t t2)
{
  return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

 /* UBX GNSSId to system (ref [2] 25) -----------------------------------------*/
static int ubx_sys(int gnssid)
{
    switch (gnssid) {
        case 0: return C_GNSS_GPS;
        case 1: return C_GNSS_MAX;
        case 2: return C_GNSS_GAL;
        case 3: return C_GNSS_BDS3;
        case 5: return C_GNSS_QZS;
        case 6: return C_GNSS_MAX;
    }
    return 0;
}
/* UBX SigId to signal (ref [5] 1.5.4) ---------------------------------------*/
static int ubx_sig(int sys, int sigid)
{
    if (sys == C_GNSS_GPS) {
        if (sigid == 0) return CODE_L1C; /* L1C/A */
        if (sigid==3) return CODE_L2L; /* L2CL */
        if (sigid==4) return CODE_L2M; /* L2CM */
        //if (sigid==6) return CODE_L5I; /* L5I */
        if (sigid==7) return CODE_L5Q; /* L5Q */
    }
    else if (sys == C_GNSS_GLO) {
        if (sigid == 0) return CODE_L1C; /* G1C/A (GLO L1 OF) */
        if (sigid == 2) return CODE_L2C; /* G2C/A (GLO L2 OF) */
    }
    else if (sys == C_GNSS_GAL) {
        if (sigid==0) return CODE_L1C; /* E1C */
        if (sigid==1) return CODE_L1B; /* E1B */
        if (sigid==3) return CODE_L5I; /* E5aI */
        if (sigid==4) return CODE_L5Q; /* E5aQ */
        if (sigid==5) return CODE_L7I; /* E5bI */
        if (sigid==6) return CODE_L7Q; /* E5bQ */
    }
    else if (sys == C_GNSS_QZS) {
        if (sigid==0) return CODE_L1C; /* L1C/A */
        //if (sigid==1) return CODE_L1Z; /* L1S */
        //if (sigid==4) return CODE_L2S; /* L2CM */
        if (sigid==5) return CODE_L2L; /* L2CL */
        if (sigid==8) return CODE_L5I; /* L5I */
        if (sigid==9) return CODE_L5Q; /* L5Q */
    }
    else if (sys == C_GNSS_BDS3) {
        if (sigid==0) return CODE_L1I; /* B1I D1 */
        if (sigid==1) return CODE_L1I; /* B1I D2 */
        if (sigid == 2) return CODE_L7I; /* B2I D1 */
        if (sigid == 3) return CODE_L7I; /* B2I D2 */
        if (sigid == 7) return CODE_L5X; /* B2a */
    }
    else if (sys == C_GNSS_MAX) {
        if (sigid==0) return CODE_L1C; /* L1C/A */
    }
    return CODE_NONE;
}

/* signal index in obs data --------------------------------------------------*/
static int sig_idx(int sys, int code)
{
	if (sys == C_GNSS_GPS) {
		if (code==CODE_L1C) return 1;
#ifdef FREQ_L5
        if (code == CODE_L5Q) return 2;
#else
        if (code == CODE_L2L) return 2;
        if (code == CODE_L2M) return NFREQ + 1;
#endif
	}
	else if (sys == C_GNSS_GLO) {
		if (code==CODE_L1C) return 1;
		if (code==CODE_L2C) return 2;
	}
	else if (sys == C_GNSS_GAL) {
		if (code==CODE_L1C) return 1;
		if (code==CODE_L1B) return NFREQ+1;
#ifdef FREQ_L5
        if (code == CODE_L5Q) return 2;
#else
        if (code == CODE_L7Q) return 2;
        if (code == CODE_L7I) return NFREQ + 2;
#endif
	}
	else if (sys == C_GNSS_QZS) {
		if (code==CODE_L1C) return 1;
#ifdef FREQ_L5
        if (code == CODE_L5Q) return 2;
#else
        if (code == CODE_L2L) return 2;
#endif
	}
	else if (sys == C_GNSS_BDS3) {
		if (code==CODE_L1I) return 1;
#ifdef FREQ_L5
        if (code == CODE_L5X) return 2;
#else
        if (code == CODE_L7I) return 2;
#endif
	}
	else if (sys == C_GNSS_MAX) {
		if (code==CODE_L1C) return 0;
	}
	return 0;
}
#if 0
/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;
    
    if (utc[3]>=256.0) return;
    ag_gnss_time2gpst(time,&week);
    utc[3]+= week/256*256.0;
    if      (utc[3]<(week-128.0)) utc[3]+=256.0;
    else if (utc[3]>(week+128.0)) utc[3]-=256.0;
}
#endif
static gnss_SignalType convert_signal_id_to_type(gnss_ConstellationType sys, uint8_t sig_id)
{
  gnss_SignalType signal = C_GNSS_SIG_MAX;

  switch (sys)
  {
  case C_GNSS_GPS:
    switch (sig_id)
    {
    case CODE_L1C:   /* GPS L1C */
      signal = C_GNSS_SIG_GPS_L1C;
      break;
    case CODE_L2L:   /* GPS L2W */
      signal = C_GNSS_SIG_GPS_L2C;
      break;
    case CODE_L2M:   /* GPS L2C(M) */
      signal = C_GNSS_SIG_GPS_L2C;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_GLO:
    signal = C_GNSS_SIG_MAX;
    break;
  case C_GNSS_BDS3:
    switch (sig_id)
    {
    case CODE_L1I:
      signal = C_GNSS_SIG_BDS_B1I;
      break;
    case CODE_L7I:  /* BDS-2 B2I*/
      signal = C_GNSS_SIG_BDS_B2I;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_GAL:
    switch (sig_id)
    {
    case CODE_L1C:
      signal = C_GNSS_SIG_GAL_E1;
      break;
    case CODE_L7Q:
      signal = C_GNSS_SIG_GAL_E5B;
      break;
    default:
      break;
    }
    break;
  case C_GNSS_QZS:
    switch (sig_id)
    {
    case CODE_L1C:   /* QZS L1C */
      signal = C_GNSS_SIG_QZS_L1C;
      break;
    case CODE_L2L:   /* QZS L2C */
      signal = C_GNSS_SIG_QZS_L2C;
      break;
    default:
      break;
    }
    break;
  }

  return signal;
}
/* decode ubx-rxm-rawx: multi-gnss raw measurement data (ref [3]) ------------*/
static int decode_rxmrawx(UbloxDecoder_t* decoder)
{
    //gtime_t time;
    double tow,cp1,pr1/*,tadj=0.0,toff=0.0,freq,tn*/;
    int i,j,sys,prn,sat,n=0,nsat,week,tstat,lockt,slip,halfv,halfc,fcn,cpstd;
    int freq_idx, num_idx, match, ver=0, sigid, code, f;/* F9 */
    //int std_slip=0;
    //int32_t lock[MAXSAT][NFREQ] = {0};
    //int32_t half[MAXSAT][NFREQ] = {0};
    //char *q;
    unsigned char *p= decoder->buff+6;
    
    nsat=U1(p+11);
    if (decoder->length< (uint32_t)(24+32*nsat)) {
        return -1;
    }
    tow=R8(p);
    week=U2(p+8);
    //time= ag_gnss_gpst2time(week,tow);
#if 0
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX RXM-RAWX  (%4d): time=%s nsat=%d",raw->len,
                time_str(time,2),getU1(p+11));
    }
#endif
    if (week==0) {
        //GLOGW("rxmrawx week=0 (len=%d nsat=%d)", decoder->length, nsat);
        decoder->nobs = 0;
        return 0;
    }
#if 0
    /* time tag adjustment option (-TADJ) */
    if ((q=strstr(raw->opt,"-TADJ="))) {
        sscanf(q,"-TADJ=%lf",&tadj);
    }
    /* slip theshold of std-dev of carreir-phase (-STD_SLIP) */
    if ((q=strstr(raw->opt,"-STD_SLIP="))) {
        sscanf(q,"-STD_SLIP=%d",&std_slip);
    }
#endif
#if 0
    /* time tag adjustment */
    if (tadj>0.0) {
        tn=time2gpst(time,&week)/tadj;
        toff=(tn-floor(tn+0.5))*tadj;
        time=timeadd(time,-toff);
    }
#endif

    if (tow >= 604800) {
        tow = tow - 604800;
    }

    decoder->leapSec = I1(p + 10);
    decoder->recStat = U1(p + 12);
    ver  =U1(p+13); /* version ([5] 5.15.3.1) */
    for (i=0,p+=16;i<nsat&&n<MAXOBS;i++,p+=32) {
        
        /*if (!(sys=ubx_sys(U1(p+20)))) {
            continue;
        }*/
        sys = ubx_sys(U1(p + 20));
        if (sys == C_GNSS_MAX)
        {
          continue;
        }
        prn=U1(p+21);
        if (!(sat= gnss_cvt_Svid2SvIndex(prn,sys))) {
            continue;
        }
		    sigid=U1(p+22);    /* sigId ([5] 5.15.3.1) */
		    if (ver>=1) {
			    code=ubx_sig(sys,sigid);
		    }
		    else {
			    code=(sys== C_GNSS_BDS3)?CODE_L1I:((sys== C_GNSS_GAL)?CODE_L1X:CODE_L1C);
		    }
        uint8_t type = convert_signal_id_to_type(sys, code);
		    /* signal index in obs data */
		    f=sig_idx(sys,code);

		    if (f==0||f>NFREQ) {
			    continue;
		    }
		    freq_idx = f - 1;
        //freq_idx = ubx_sig2freq(sigid, sys); /* F9 Signal Id */
        num_idx = obs_search(decoder->obs, n, sat, &match);
        if (!match)
        {
            /* Increase Only If New Sat Found */
            n++;
           for (j = 0; j < NFREQ + NEXOBS; j++)
            {
              decoder->obs[num_idx].carrier_phase[j] = decoder->obs[num_idx].pseudorange[j] = 0.0;
              decoder->obs[num_idx].doppler[j] = 0.0;
              decoder->obs[num_idx].snr[j] = decoder->obs[num_idx].lli[j] = 0;
              decoder->obs[num_idx].code[j] = CODE_NONE;
              decoder->obs[num_idx].prStd[j] = decoder->obs[num_idx].cpStd[j] = decoder->obs[num_idx].drStd[j] = 0;
              decoder->obs[num_idx].lockTime[j] = decoder->obs[num_idx].trkStat[j] = 0;
              decoder->obs[num_idx].glofcn[j] = 0;
            }
        }

        cpstd=U1(p+28)&15; /* carrier-phase std-dev */
        tstat=U1(p+30); /* tracking status */
        pr1=tstat&1?R8(p  ):0.0;
        cp1=tstat&2?R8(p+8):0.0;
        if (cp1==-0.5/*||cpstd>CPSTD_VALID*/) cp1=0.0; /* invalid phase */
        decoder->obs[num_idx].svid = prn;
        decoder->obs[num_idx].sat = sat;
        //decoder->obs[num_idx].sys = ublox_sys(U1(p + 20));
        decoder->obs[num_idx].sys = sys;
        decoder->obs[num_idx].tow_ms  = (uint32_t)(tow*1000);
        decoder->obs[num_idx].code[freq_idx] = type;
        decoder->obs[num_idx].pseudorange[freq_idx]=pr1;
        decoder->obs[num_idx].carrier_phase[freq_idx] =cp1;
        decoder->obs[num_idx].prStd[freq_idx] =U1(p+27)&0xF; /* pr std-dev */
        decoder->obs[num_idx].cpStd[freq_idx] =cpstd; /* cp std-dev */
        decoder->obs[num_idx].drStd[freq_idx] =U1(p+29)&0xF; /* dr std-dev */
        decoder->obs[num_idx].trkStat[freq_idx] =tstat; /* tracking status */
        
        fcn = (int)U1(p + 23) - 7;    /* fcn = freqId-7 */
        decoder->obs[num_idx].glofcn[freq_idx] = fcn;
        /* offset by time tag adjustment */
#if 0
        if (toff != 0.0) {
#ifdef FREQ_L5
            freq = sys_freq(sys, freq_idx + 1, fcn);
#else
            freq = sys_freq(sys, freq_idx, fcn);
#endif
            decoder->obs[num_idx].pseudorange[freq_idx] -= toff * CLIGHT;
            decoder->obs[num_idx].carrier_phase[freq_idx] -= toff * freq;
        }
#endif
        decoder->obs[num_idx].doppler[freq_idx] =R4(p+16);
        decoder->obs[num_idx].snr[freq_idx] =U1(p+26)/* *4 */;
        decoder->obs[num_idx].lli[freq_idx] =0;
       // decoder->obs[num_idx].code[freq_idx] = (U8)code;
        
        lockt=U2(p+24);    /* lock time count (ms) */
        decoder->obs[num_idx].lockTime[freq_idx] = lockt;
        slip=lockt==0||lockt< decoder->lockt[sat-1][freq_idx]?1:0;
#if 0
        if (std_slip>0) {
            slip|=(cpstd>=std_slip)?1:0; /* slip by std-dev of cp */
        }
#endif
        halfv=tstat&4?1:0; /* half cycle valid */
        halfc=tstat&8?1:0; /* half cycle subtracted from phase */
        
        if (cp1!=0.0) { /* carrier-phase valid */
            
            /* LLI: bit1=loss-of-lock,bit2=half-cycle-invalid */
            decoder->obs[num_idx].lli[freq_idx] |=slip;
            decoder->obs[num_idx].lli[freq_idx] |=halfc!= decoder->halfc[sat - 1][freq_idx] ? 1 : 0;
            decoder->obs[num_idx].lli[freq_idx] |=halfv?0:2;
        }
        decoder->lockt[sat - 1][freq_idx] = lockt;
        decoder->halfc[sat - 1][freq_idx] = halfc;
    }
    decoder->q_towMsec = (uint32_t)(tow * 1000);
    decoder->w_gpsWeek = week;
    decoder->nobs=n;
    return 1;
}
#if 0
/* save subframe -------------------------------------------------------------*/
static int save_subfrm(int sat, raw_t *raw)
{
    #if 0
    unsigned char *p=raw->buff+6,*q;
    int i,j,n,id=(U4(p+6)>>2)&0x7;

    
    if (id<1||5<id) return 0;
    
    q=raw->subfrm[sat-1]+(id-1)*30;
    
    for (i=n=0,p+=2;i<10;i++,p+=4) {
        for (j=23;j>=0;j--) {
            *q=(*q<<1)+((getU4(p)>>j)&1); if (++n%8==0) q++;
        }
    }
    return id;
    #endif
    return 0;
}
#endif
static unsigned int asg_getbitu(const unsigned char* buff, int pos, int len)
{
  unsigned int bits = 0;
  int i;
  for (i = pos; i < pos + len; i++) {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  }
  return bits;
}
static int asg_getbits(const unsigned char* buff, int pos, int len)
{
  unsigned int bits = asg_getbitu(buff, pos, len);
  if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) return (int)bits;
  return (int)(bits | (~0u << len)); /* extend sign */
}
/* get two component bits ----------------------------------------------------*/
static unsigned int ag_getbitu2(const unsigned char* buff, int p1, int l1, int p2,
  int l2)
{
  return (asg_getbitu(buff, p1, l1) << l2) + asg_getbitu(buff, p2, l2);
}
static int ag_getbits2(const unsigned char* buff, int p1, int l1, int p2, int l2)
{
  if (asg_getbitu(buff, p1, 1))
    return (int)((asg_getbits(buff, p1, l1) << l2) + asg_getbitu(buff, p2, l2));
  else
    return (int)ag_getbitu2(buff, p1, l1, p2, l2);
}
/* get three component bits --------------------------------------------------*/
static unsigned int ag_getbitu3(const unsigned char* buff, int p1, int l1, int p2,
  int l2, int p3, int l3)
{
  return (asg_getbitu(buff, p1, l1) << (l2 + l3)) + (asg_getbitu(buff, p2, l2) << l3) +
    asg_getbitu(buff, p3, l3);
}
static int ag_getbits3(const unsigned char* buff, int p1, int l1, int p2, int l2,
  int p3, int l3)
{
  if (asg_getbitu(buff, p1, 1))
    return (int)((asg_getbits(buff, p1, l1) << (l2 + l3)) +
      (asg_getbitu(buff, p2, l2) << l3) + asg_getbitu(buff, p3, l3));
  else
    return (int)ag_getbitu3(buff, p1, l1, p2, l2, p3, l3);
}
/* merge two components ------------------------------------------------------*/
static unsigned int ag_merge_two_u(unsigned int a, unsigned int b, int n)
{
  return (a << n) + b;
}
static int ag_merge_two_s(int a, unsigned int b, int n)
{
  return (int)((a << n) + b);
}
/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
static int adjgpsweek(int week)
{
  if (week > 0 && week < 1024)
  {
    week += 2048;
  }
  return week;
}
/* decode gps/qzss navigation data subframe 1 --------------------------------*/
static int ag_decode_subfrm1(const unsigned char* buff, eph_t* eph)
{
  double tow, toc;
  int i = 48, week, iodc0, iodc1, tgd;

  tow = asg_getbitu(buff, 24, 17) * 6.0;           /* transmission time */
  week = asg_getbitu(buff, i, 10);       i += 10;
  eph->code = asg_getbitu(buff, i, 2);       i += 2;
  eph->sva = asg_getbitu(buff, i, 4);       i += 4;   /* ura index */
  eph->svh = asg_getbitu(buff, i, 6);       i += 6;
  iodc0 = asg_getbitu(buff, i, 2);       i += 2;
  eph->flag = asg_getbitu(buff, i, 1);       i += 1 + 87;
  tgd = asg_getbits(buff, i, 8);       i += 8;
  iodc1 = asg_getbitu(buff, i, 8);       i += 8;
  toc = asg_getbitu(buff, i, 16) * 16.0;  i += 16;
  eph->f2 = asg_getbits(buff, i, 8) * P2_55; i += 8;
  eph->f1 = asg_getbits(buff, i, 16) * P2_43; i += 16;
  eph->f0 = asg_getbits(buff, i, 22) * P2_31;

  eph->tgd[0] = tgd == -128 ? 0.0 : tgd * P2_31; /* ref [4] */
  eph->iodc = (iodc0 << 8) + iodc1;
  eph->week = adjgpsweek(week); /* week of tow */
  eph->ttr = ag_gnss_gpst2time(eph->week, tow);
  eph->toc = ag_gnss_gpst2time(eph->week, toc);

  return 1;
}
/* decode gps/qzss navigation data subframe 2 --------------------------------*/
static int ag_decode_subfrm2(const unsigned char* buff, eph_t* eph)
{
  double sqrtA;
  int i = 48;
  unsigned int en_sqrt_a = 0;


  eph->iode = asg_getbitu(buff, i, 8);              i += 8;
  eph->crs = asg_getbits(buff, i, 16) * P2_05;         i += 16;
  eph->deln = asg_getbits(buff, i, 16) * P2_43 * SC2RAD; i += 16;
  eph->M0 = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->cuc = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  eph->e = asg_getbitu(buff, i, 32) * P2_33;        i += 32;
  eph->cus = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  en_sqrt_a = asg_getbitu(buff, i, 32);          i += 32;
  eph->toes = asg_getbitu(buff, i, 16) * 16.0;         i += 16;
  eph->fit = asg_getbitu(buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */

  sqrtA = en_sqrt_a * P2_19;
  eph->A = sqrtA * sqrtA;

  return 2;
}
/* decode gps/qzss navigation data subframe 3 --------------------------------*/
static int ag_decode_subfrm3(const unsigned char* buff, eph_t* eph)
{
  double tow, toc;
  int i = 48, iode;

  eph->cic = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  eph->OMG0 = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->cis = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  eph->i0 = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->crc = asg_getbits(buff, i, 16) * P2_05;         i += 16;
  eph->omg = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->OMGd = asg_getbits(buff, i, 24) * P2_43 * SC2RAD; i += 24;
  iode = asg_getbitu(buff, i, 8);              i += 8;
  eph->idot = asg_getbits(buff, i, 14) * P2_43 * SC2RAD;

  /* check iode and iodc consistency */
  if (iode != eph->iode || iode != (eph->iodc & 0xFF)) return 0;

  /* adjustment for week handover */
  tow = ag_gnss_time2gpst(eph->ttr, &eph->week);
  toc = ag_gnss_time2gpst(eph->toc, NULL);
  if (eph->toes < tow - 302400.0) { eph->week++; tow -= 604800.0; }
  else if (eph->toes > tow + 302400.0) { eph->week--; tow += 604800.0; }
  eph->toe = ag_gnss_gpst2time(eph->week, eph->toes);
  eph->toc = ag_gnss_gpst2time(eph->week, toc);
  eph->ttr = ag_gnss_gpst2time(eph->week, tow);

  return 3;
}
/* decode gps/qzss navigation data frame ---------------------------------------
* decode navigation data frame and extract ephemeris and ion/utc parameters
* args   : unsigned char *buff I gps navigation data frame (without parity)
*                                  buff[0-29]: 24 bits x 10 words
*          eph_t *eph    IO     ephemeris message      (NULL: no input)
*          alm_t *alm    IO     almanac                (NULL: no input)
*          double *ion   IO     ionospheric parameters (NULL: no input)
*          double *utc   IO     delta-utc parameters   (NULL: no input)
*          int   *leaps  IO     leap seconds (s)       (NULL: no input)
* return : status (0:no valid, 1-5:subframe id)
* notes  : use cpu time to resolve modulo 1024 ambiguity of the week number
*          see ref [1]
*          utc[3] reference week for utc parameter is truncated in 8 bits
*          ion and utc parameters by qzss indicate local iono and qzst-utc
*          parameters.
*-----------------------------------------------------------------------------*/
extern int ag_decode_frame(const unsigned char* buff, eph_t* eph,
  double* ion, double* utc, int* leaps)
{
  int id = asg_getbitu(buff, 43, 3); /* subframe id */


  switch (id) {
  case 1: return ag_decode_subfrm1(buff, eph);
  case 2: return ag_decode_subfrm2(buff, eph);
  case 3: return ag_decode_subfrm3(buff, eph);
  }
  return 0;
}
/* decode ephemeris ----------------------------------------------------------*/
static int decode_ephem(int sat, UbloxDecoder_t* decoder)
{
    
    #if 1
    eph_t eph={0};
    int prn;
    uint8_t u_sys = 0;
    if (ag_decode_frame(decoder->subfrm[sat]   ,&eph,NULL,NULL,NULL)!=1||
        ag_decode_frame(decoder->subfrm[sat]+30,&eph,NULL,NULL,NULL)!=2||
        ag_decode_frame(decoder->subfrm[sat]+60,&eph,NULL,NULL,NULL)!=3) return 0;
    
    //if (!strstr(raw->opt,"-EPHALL")) 
    //{
    //    if (eph.iode== decoder->ephemeris.iode&&
    //        eph.iodc== decoder->ephemeris.iodc) return 0; /* unchanged */
    //}

    prn = gnss_cvt_SvIndex2Svid(sat, &u_sys);
    eph.sat = prn;
    decoder->ephemeris=eph;
    //raw->ephsat=sat;
    return 2;
    #endif
}
#if 0
/* decode almanac and ion/utc ------------------------------------------------*/
static int decode_alm1(int sat, raw_t *raw)
{
#if 0
    int sys=satsys(sat,NULL);
    
    if (sys==SYS_GPS) {
        decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_gps,
                     raw->nav.utc_gps,&raw->nav.leaps);
        adj_utcweek(raw->time,raw->nav.utc_gps);
    }
    else if (sys==SYS_QZS) {
        decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_qzs,
                     raw->nav.utc_qzs,&raw->nav.leaps);
        adj_utcweek(raw->time,raw->nav.utc_qzs);
    }
#endif
    return 9;
}

/* decode almanac ------------------------------------------------------------*/
static int decode_alm2(int sat, raw_t *raw)
{
#if 0
    int sys=satsys(sat,NULL);
    
    if (sys==SYS_GPS) {
        decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL);
    }
    else if (sys==SYS_QZS) {
        decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,raw->nav.ion_qzs,
                     raw->nav.utc_qzs,&raw->nav.leaps);
        adj_utcweek(raw->time,raw->nav.utc_qzs);
    }
#endif
    return  0;
}
#endif
/* decode gps and qzss navigation data ---------------------------------------*/
static int decode_nav(UbloxDecoder_t* decoder, int sat, int off)
{
    unsigned int words[10];
    int i,id;
    unsigned char *p= decoder->buff+6+off;
    
    if (decoder->length< (uint32_t)(48+off)) {
        return -1;
    }
    for (i=0;i<10;i++,p+=4) words[i]=U4(p)>>6; /* 24 bits without parity */
    
    id=(words[1]>>2)&7;
    if (id<1||5<id) {
        return -1;
    }
    for (i=0;i<10;i++) {
        loc_setbitu(decoder->subfrm[sat]+(id-1)*30,i*24,24,words[i]);
    }
    if (id==3) return decode_ephem(sat, decoder);
    //if (id==4) return decode_alm1 (sat, decoder);
    //if (id==5) return decode_alm2 (sat, decoder);
    return 0;
}
/* decode Galileo I/NAV ephemeris ----------------------------------------------
* decode Galileo I/NAV (ref [5] 4.3)
* args   : unsigned char *buff I Galileo I/NAV subframe bits
*                                  buff[ 0-15]: I/NAV word type 0 (128 bit)
*                                  buff[16-31]: I/NAV word type 1
*                                  buff[32-47]: I/NAV word type 2
*                                  buff[48-63]: I/NAV word type 3
*                                  buff[64-79]: I/NAV word type 4
*                                  buff[80-95]: I/NAV word type 5
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ag_decode_gal_inav(const unsigned char* buff, eph_t* eph)
{
  double tow, toc, tt, sqrtA;
  int i, time_f, week, svid, e5b_hs, e1b_hs, e5b_dvs, e1b_dvs, type[5], iod_nav[4];
  unsigned int en_sqrt_a = 0;
  i = 0; /* word type 0 */
  type[0] = asg_getbitu(buff, i, 6);              i += 6;
  time_f = asg_getbitu(buff, i, 2);              i += 2 + 88;
  week = asg_getbitu(buff, i, 12);              i += 12; /* gst-week */
  tow = asg_getbitu(buff, i, 20);

  i = 128; /* word type 1 */
  type[1] = asg_getbitu(buff, i, 6);              i += 6;
  iod_nav[0] = asg_getbitu(buff, i, 10);              i += 10;
  eph->toes = asg_getbitu(buff, i, 14) * 60.0;         i += 14;
  eph->M0 = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->e = asg_getbitu(buff, i, 32) * P2_33;        i += 32;
  en_sqrt_a = asg_getbitu(buff, i, 32);
  sqrtA = en_sqrt_a * P2_19;

  i = 128 * 2; /* word type 2 */
  type[2] = asg_getbitu(buff, i, 6);              i += 6;
  iod_nav[1] = asg_getbitu(buff, i, 10);              i += 10;
  eph->OMG0 = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->i0 = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->omg = asg_getbits(buff, i, 32) * P2_31 * SC2RAD; i += 32;
  eph->idot = asg_getbits(buff, i, 14) * P2_43 * SC2RAD;

  i = 128 * 3; /* word type 3 */
  type[3] = asg_getbitu(buff, i, 6);              i += 6;
  iod_nav[2] = asg_getbitu(buff, i, 10);              i += 10;
  eph->OMGd = asg_getbits(buff, i, 24) * P2_43 * SC2RAD; i += 24;
  eph->deln = asg_getbits(buff, i, 16) * P2_43 * SC2RAD; i += 16;
  eph->cuc = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  eph->cus = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  eph->crc = asg_getbits(buff, i, 16) * P2_05;         i += 16;
  eph->crs = asg_getbits(buff, i, 16) * P2_05;         i += 16;
  eph->sva = asg_getbitu(buff, i, 8);

  i = 128 * 4; /* word type 4 */
  type[4] = asg_getbitu(buff, i, 6);              i += 6;
  iod_nav[3] = asg_getbitu(buff, i, 10);              i += 10;
  svid = asg_getbitu(buff, i, 6);              i += 6;
  eph->cic = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  eph->cis = asg_getbits(buff, i, 16) * P2_29;        i += 16;
  toc = asg_getbitu(buff, i, 14) * 60.0;         i += 14;
  eph->f0 = asg_getbits(buff, i, 31) * P2_34;        i += 31;
  eph->f1 = asg_getbits(buff, i, 21) * P2_46;        i += 21;
  eph->f2 = asg_getbits(buff, i, 6) * P2_59;

  i = 128 * 5; /* word type 5 */
  /*type[5] = asg_getbitu(buff, i, 6);*/              i += 6 + 41;
  eph->tgd[0] = asg_getbits(buff, i, 10) * P2_32;        i += 10; /* BGD E5a/E1 */
  eph->tgd[1] = asg_getbits(buff, i, 10) * P2_32;        i += 10; /* BGD E5b/E1 */
  e5b_hs = asg_getbitu(buff, i, 2);              i += 2;
  e1b_hs = asg_getbitu(buff, i, 2);              i += 2;
  e5b_dvs = asg_getbitu(buff, i, 1);              i += 1;
  e1b_dvs = asg_getbitu(buff, i, 1);

  /* test word types */
  if (type[0] != 0 || type[1] != 1 || type[2] != 2 || type[3] != 3 || type[4] != 4) {
    return 0;
  }
  /* test word type 0 time field */
  if (time_f != 2) {
    return 0;
  }
  /* test consistency of iod_nav */
  if (iod_nav[0] != iod_nav[1] || iod_nav[0] != iod_nav[2] || iod_nav[0] != iod_nav[3]) {
    return 0;
  }
  if (!(eph->sat = gnss_cvt_Svid2SvIndex(svid, C_GNSS_GAL))) {
    return 0;
  }
  eph->A = sqrtA * sqrtA;
  eph->iode = eph->iodc = iod_nav[0];
  eph->svh = (e5b_hs << 7) | (e5b_dvs << 6) | (e1b_hs << 1) | e1b_dvs;
  eph->ttr = ag_gnss_gpst2time(week, tow);
  tt = ag_gnss_timediff(ag_gnss_gpst2time(week, eph->toes), eph->ttr); /* week complient to toe */
  if (tt > 302400.0) week--;
  else if (tt < -302400.0) week++;
  eph->toe = ag_gnss_gpst2time(week, eph->toes);
  eph->toc = ag_gnss_gpst2time(week, toc);
  eph->week = week; /* gps-week = gst-week + 1024 */
  eph->code = 1;         /* data source = I/NAV E1B */

  return 1;
}
/* decode BeiDou D1 ephemeris --------------------------------------------------
* decode BeiDou D1 ephemeris (IGSO/MEO satellites) (ref [3] 5.2)
* args   : unsigned char *buff I beidou D1 subframe bits
*                                  buff[ 0- 37]: subframe 1 (300 bits)
*                                  buff[38- 75]: subframe 2
*                                  buff[76-113]: subframe 3
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ag_decode_bds_d1(const unsigned char* buff, eph_t* eph)
{
  double toc_bds, sqrtA;
  unsigned int toe1, toe2, sow1, sow2, sow3, en_sqrt_a = 0;
  int i, frn1, frn2, frn3;

  i = 8 * 38 * 0; /* subframe 1 */
  frn1 = asg_getbitu(buff, i + 15, 3);
  sow1 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->svh = asg_getbitu(buff, i + 42, 1); /* SatH1 */
  eph->iodc = asg_getbitu(buff, i + 43, 5); /* AODC */
  eph->sva = asg_getbitu(buff, i + 48, 4);
  eph->week = asg_getbitu(buff, i + 60, 13); /* week in BDT */
  toc_bds = ag_getbitu2(buff, i + 73, 9, i + 90, 8) * 8.0;
  eph->tgd[0] = asg_getbits(buff, i + 98, 10) * 0.1 * 1E-9;
  eph->tgd[1] = ag_getbits2(buff, i + 108, 4, i + 120, 6) * 0.1 * 1E-9;
  eph->f2 = asg_getbits(buff, i + 214, 11) * P2_66;
  eph->f0 = ag_getbits2(buff, i + 225, 7, i + 240, 17) * P2_33;
  eph->f1 = ag_getbits2(buff, i + 257, 5, i + 270, 17) * P2_50;
  eph->iode = asg_getbitu(buff, i + 287, 5); /* AODE */

  i = 8 * 38 * 1; /* subframe 2 */
  frn2 = asg_getbitu(buff, i + 15, 3);
  sow2 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->deln = ag_getbits2(buff, i + 42, 10, i + 60, 6) * P2_43 * SC2RAD;
  eph->cuc = ag_getbits2(buff, i + 66, 16, i + 90, 2) * P2_31;
  eph->M0 = ag_getbits2(buff, i + 92, 20, i + 120, 12) * P2_31 * SC2RAD;
  eph->e = ag_getbitu2(buff, i + 132, 10, i + 150, 22) * P2_33;
  eph->cus = asg_getbits(buff, i + 180, 18) * P2_31;
  eph->crc = ag_getbits2(buff, i + 198, 4, i + 210, 14) * P2_06;
  eph->crs = ag_getbits2(buff, i + 224, 8, i + 240, 10) * P2_06;
  en_sqrt_a = ag_getbitu2(buff, i + 250, 12, i + 270, 20);
  sqrtA = en_sqrt_a * P2_19;
  toe1 = asg_getbitu(buff, i + 290, 2); /* TOE 2-MSB */
  eph->A = sqrtA * sqrtA;

  i = 8 * 38 * 2; /* subframe 3 */
  frn3 = asg_getbitu(buff, i + 15, 3);
  sow3 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  toe2 = ag_getbitu2(buff, i + 42, 10, i + 60, 5); /* TOE 5-LSB */
  eph->i0 = ag_getbits2(buff, i + 65, 17, i + 90, 15) * P2_31 * SC2RAD;
  eph->cic = ag_getbits2(buff, i + 105, 7, i + 120, 11) * P2_31;
  eph->OMGd = ag_getbits2(buff, i + 131, 11, i + 150, 13) * P2_43 * SC2RAD;
  eph->cis = ag_getbits2(buff, i + 163, 9, i + 180, 9) * P2_31;
  eph->idot = ag_getbits2(buff, i + 189, 13, i + 210, 1) * P2_43 * SC2RAD;
  eph->OMG0 = ag_getbits2(buff, i + 211, 21, i + 240, 11) * P2_31 * SC2RAD;
  eph->omg = ag_getbits2(buff, i + 251, 11, i + 270, 21) * P2_31 * SC2RAD;
  eph->toes = ag_merge_two_u(toe1, toe2, 15) * 8.0;

  /* check consistency of subframe numbers, sows and toe/toc */
  if (frn1 != 1 || frn2 != 2 || frn3 != 3) {
    return 0;
  }
  if (sow2 != sow1 + 6 || sow3 != sow2 + 6) {
    return 0;
  }
  if (toc_bds != eph->toes) {
    return 0;
  }
  eph->ttr = ag_gnss_bdt2gpst(ag_gnss_bdt2time(eph->week, sow1));      /* bdt -> gpst */
  if (eph->toes > sow1 + 302400.0) eph->week++;
  else if (eph->toes < sow1 - 302400.0) eph->week--;
  eph->toe = ag_gnss_bdt2gpst(ag_gnss_bdt2time(eph->week, eph->toes)); /* bdt -> gpst */
  eph->toc = ag_gnss_bdt2gpst(ag_gnss_bdt2time(eph->week, toc_bds));   /* bdt -> gpst */
  return 1;
}
/* decode BeiDou D2 ephemeris --------------------------------------------------
* decode BeiDou D2 ephemeris (GEO satellites) (ref [3] 5.3)
* args   : unsigned char *buff I beidou D2 subframe 1 page bits
*                                  buff[  0- 37]: page 1 (300 bits)
*                                  buff[ 38- 75]: page 2
*                                  ...
*                                  buff[342-379]: page 10
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ag_decode_bds_d2(const unsigned char* buff, eph_t* eph)
{
  double toc_bds, sqrtA;
  unsigned int f1p4, cucp5, ep6, cicp7, i0p8, OMGdp9, omgp10;
  unsigned int sow1, sow3, sow4, sow5, sow6, sow7, sow8, sow9, sow10, en_sqrt_a = 0;
  int i, f1p3, cucp4, ep5, cicp6, i0p7, OMGdp8, omgp9;
  int pgn1, pgn3, pgn4, pgn5, pgn6, pgn7, pgn8, pgn9, pgn10;


  i = 8 * 38 * 0; /* page 1 */
  pgn1 = asg_getbitu(buff, i + 42, 4);
  sow1 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->svh = asg_getbitu(buff, i + 46, 1); /* SatH1 */
  eph->iodc = asg_getbitu(buff, i + 47, 5); /* AODC */
  eph->sva = asg_getbitu(buff, i + 60, 4);
  eph->week = asg_getbitu(buff, i + 64, 13); /* week in BDT */
  toc_bds = ag_getbitu2(buff, i + 77, 5, i + 90, 12) * 8.0;
  eph->tgd[0] = asg_getbits(buff, i + 102, 10) * 0.1 * 1E-9;
  eph->tgd[1] = asg_getbits(buff, i + 120, 10) * 0.1 * 1E-9;

  i = 8 * 38 * 2; /* page 3 */
  pgn3 = asg_getbitu(buff, i + 42, 4);
  sow3 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->f0 = ag_getbits2(buff, i + 100, 12, i + 120, 12) * P2_33;
  f1p3 = asg_getbits(buff, i + 132, 4);

  i = 8 * 38 * 3; /* page 4 */
  pgn4 = asg_getbitu(buff, i + 42, 4);
  sow4 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  f1p4 = ag_getbitu2(buff, i + 46, 6, i + 60, 12);
  eph->f2 = ag_getbits2(buff, i + 72, 10, i + 90, 1) * P2_66;
  eph->iode = asg_getbitu(buff, i + 91, 5); /* AODE */
  eph->deln = asg_getbits(buff, i + 96, 16) * P2_43 * SC2RAD;
  cucp4 = asg_getbits(buff, i + 120, 14);

  i = 8 * 38 * 4; /* page 5 */
  pgn5 = asg_getbitu(buff, i + 42, 4);
  sow5 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  cucp5 = asg_getbitu(buff, i + 46, 4);
  eph->M0 = ag_getbits3(buff, i + 50, 2, i + 60, 22, i + 90, 8) * P2_31 * SC2RAD;
  eph->cus = ag_getbits2(buff, i + 98, 14, i + 120, 4) * P2_31;
  ep5 = asg_getbits(buff, i + 124, 10);

  i = 8 * 38 * 5; /* page 6 */
  pgn6 = asg_getbitu(buff, i + 42, 4);
  sow6 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  ep6 = ag_getbitu2(buff, i + 46, 6, i + 60, 16);
  en_sqrt_a = ag_getbitu3(buff, i + 76, 6, i + 90, 22, i + 120, 4);
  sqrtA = en_sqrt_a * P2_19;
  cicp6 = asg_getbits(buff, i + 124, 10);
  eph->A = sqrtA * sqrtA;

  i = 8 * 38 * 6; /* page 7 */
  pgn7 = asg_getbitu(buff, i + 42, 4);
  sow7 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  cicp7 = ag_getbitu2(buff, i + 46, 6, i + 60, 2);
  eph->cis = asg_getbits(buff, i + 62, 18) * P2_31;
  eph->toes = ag_getbitu2(buff, i + 80, 2, i + 90, 15) * 8.0;
  i0p7 = ag_getbits2(buff, i + 105, 7, i + 120, 14);

  i = 8 * 38 * 7; /* page 8 */
  pgn8 = asg_getbitu(buff, i + 42, 4);
  sow8 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  i0p8 = ag_getbitu2(buff, i + 46, 6, i + 60, 5);
  eph->crc = ag_getbits2(buff, i + 65, 17, i + 90, 1) * P2_06;
  eph->crs = asg_getbits(buff, i + 91, 18) * P2_06;
  OMGdp8 = ag_getbits2(buff, i + 109, 3, i + 120, 16);

  i = 8 * 38 * 8; /* page 9 */
  pgn9 = asg_getbitu(buff, i + 42, 4);
  sow9 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  OMGdp9 = asg_getbitu(buff, i + 46, 5);
  eph->OMG0 = ag_getbits3(buff, i + 51, 1, i + 60, 22, i + 90, 9) * P2_31 * SC2RAD;
  omgp9 = ag_getbits2(buff, i + 99, 13, i + 120, 14);

  i = 8 * 38 * 9; /* page 10 */
  pgn10 = asg_getbitu(buff, i + 42, 4);
  sow10 = ag_getbitu2(buff, i + 18, 8, i + 30, 12);
  omgp10 = asg_getbitu(buff, i + 46, 5);
  eph->idot = ag_getbits2(buff, i + 51, 1, i + 60, 13) * P2_43 * SC2RAD;

  /* check consistency of page numbers, sows and toe/toc */
  if (pgn1 != 1 || pgn3 != 3 || pgn4 != 4 || pgn5 != 5 || pgn6 != 6 || pgn7 != 7 || pgn8 != 8 || pgn9 != 9 ||
    pgn10 != 10) {
    return 0;
  }
  if (sow3 != sow1 + 6 || sow4 != sow3 + 3 || sow5 != sow4 + 3 || sow6 != sow5 + 3 ||
    sow7 != sow6 + 3 || sow8 != sow7 + 3 || sow9 != sow8 + 3 || sow10 != sow9 + 3) {
    return 0;
  }
  if (toc_bds != eph->toes) {
    return 0;
  }
  eph->f1 = ag_merge_two_s(f1p3, f1p4, 18) * P2_50;
  eph->cuc = ag_merge_two_s(cucp4, cucp5, 4) * P2_31;
  eph->e = ag_merge_two_s(ep5, ep6, 22) * P2_33;
  eph->cic = ag_merge_two_s(cicp6, cicp7, 8) * P2_31;
  eph->i0 = ag_merge_two_s(i0p7, i0p8, 11) * P2_31 * SC2RAD;
  eph->OMGd = ag_merge_two_s(OMGdp8, OMGdp9, 5) * P2_43 * SC2RAD;
  eph->omg = ag_merge_two_s(omgp9, omgp10, 5) * P2_31 * SC2RAD;

  eph->ttr = ag_gnss_bdt2gpst(ag_gnss_bdt2time(eph->week, sow1));      /* bdt -> gpst */
  if (eph->toes > sow1 + 302400.0) eph->week++;
  else if (eph->toes < sow1 - 302400.0) eph->week--;
  eph->toe = ag_gnss_bdt2gpst(ag_gnss_bdt2time(eph->week, eph->toes)); /* bdt -> gpst */
  eph->toc = ag_gnss_bdt2gpst(ag_gnss_bdt2time(eph->week, toc_bds));   /* bdt -> gpst */
  return 1;
}
/* decode galileo navigation data --------------------------------------------*/
static int decode_enav(UbloxDecoder_t* decoder, int sat, int off)
{
    eph_t eph={0};
    unsigned char *p= decoder ->buff+6+off,buff[32],crc_buff[26]={0};
    int i,j,k,part1,page1,part2,page2,type;
    int prn;
    uint8_t u_sys = 0;
    
    if (decoder->length< (uint32_t)(44+off)) {
        return -1;
    }
    for (i=k=0;i<8;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    part1=loc_getbitu(buff   ,0,1);
    page1=loc_getbitu(buff   ,1,1);
    part2=loc_getbitu(buff+16,0,1);
    page2=loc_getbitu(buff+16,1,1);
    
    /* skip alert page */
    if (page1==1||page2==1) return 0;
    
    /* test even-odd parts */
    if (part1!=0||part2!=1) {
        return -1;
    }
    /* test crc (4(pad) + 114 + 82 bits) */
    for (i=0,j=  4;i<15;i++,j+=8) loc_setbitu(crc_buff,j,8,loc_getbitu(buff   ,i*8,8));
    for (i=0,j=118;i<11;i++,j+=8) loc_setbitu(crc_buff,j,8,loc_getbitu(buff+16,i*8,8));
    if (_rtk_crc24q(crc_buff,25)!=loc_getbitu(buff+16,82,24)) {
        return -1;
    }
    type=loc_getbitu(buff,2,6); /* word type */
    
    /* skip word except for ephemeris, iono, utc parameters */
    if (type>6) return 0;
    
    /* clear word 0-6 flags */
    if (type==2) decoder->subfrm[sat][112]=0;
    
    /* save page data (112 + 16 bits) to frame buffer */
    k=type*16;
    for (i=0,j=2;i<14;i++,j+=8) decoder->subfrm[sat][k++]=loc_getbitu(buff   ,j,8);
    for (i=0,j=2;i< 2;i++,j+=8) decoder->subfrm[sat][k++]=loc_getbitu(buff+16,j,8);
    
    /* test word 0-6 flags */
    decoder->subfrm[sat][112]|=(1<<type);
    if (decoder->subfrm[sat][112]!=0x7F) return 0;
    
    /* decode galileo inav ephemeris */
    if (!ag_decode_gal_inav(decoder->subfrm[sat],&eph)) {
        return 0;
    }
    /* test svid consistency */
    if (eph.sat!=sat) {
        return -1;
    }
#if 0
    if (!strstr(raw->opt,"-EPHALL")) 
#endif
    {
        if (eph.iode== decoder->ephemeris.iode&& /* unchanged */
          ag_gnss_timediff(eph.toe, decoder->ephemeris.toe)==0.0&&
          ag_gnss_timediff(eph.toc, decoder->ephemeris.toc)==0.0) return 0;
    }
    prn = gnss_cvt_SvIndex2Svid(sat, &u_sys);

    eph.sat= prn;
    decoder->ephemeris=eph;
    //raw->ephsat=sat;
    return 2;
}
/* decode beidou navigation data ---------------------------------------------*/
static int decode_cnav(UbloxDecoder_t* decoder, int sat, int off)
{
    //raw_t* raw;
    eph_t eph={0};
    unsigned int words[10];
    int i, id, pgn, prn;
    uint8_t u_sys = 0;
    unsigned char *p= decoder->buff+6+off;
    
    if (decoder->length<(uint32_t)(48+off)) {
        return -1;
    }
    for (i=0;i<10;i++,p+=4) words[i]=getU4(p)&0x3FFFFFFF; /* 30 bits */
    
    prn = gnss_cvt_SvIndex2Svid(sat,&u_sys);
    id=(words[0]>>12)&0x07; /* subframe id (3bit) */
    if (id<1||5<id) {
        return -1;
    }
    if (prn>5) { /* IGSO/MEO */
        
        for (i=0;i<10;i++) {
            loc_setbitu(decoder->subfrm[sat]+(id-1)*38,i*30,30,words[i]);
        }
        if (id!=3) return 0;
        
        /* decode beidou D1 ephemeris */
        if (!ag_decode_bds_d1(decoder->subfrm[sat],&eph)) return 0;
    }
    else { /* GEO */
        if (id!=1) return 0;
        
        /* subframe 1 */
        pgn=(words[1]>>14)&0x0F; /* page number (4bit) */
        if (pgn<1||10<pgn) {
            return -1;
        }
        for (i=0;i<10;i++) {
            loc_setbitu(decoder->subfrm[sat]+(pgn-1)*38,i*30,30,words[i]);
        }
        if (pgn!=10) return 0;
        
        /* decode beidou D2 ephemeris */
        if (!ag_decode_bds_d2(decoder->subfrm[sat],&eph)) return 0;
    }
#if 0
    if (!strstr(raw->opt,"-EPHALL")) 
#endif
    {
    //if (ag_gnss_timediff(eph.toe, decoder->ephemeris.toe) == 0.0 &&
    //    eph.iode == decoder->ephemeris.iode &&
    //    eph.iodc == decoder->ephemeris.iodc)
    //    return 0; /* unchanged */
    }
    eph.sat = prn;
    decoder->ephemeris=eph;
    //raw->ephsat=sat;
    return 2;
}
#if 0
/* decode glonass navigation data --------------------------------------------*/
static int decode_gnav(raw_t *raw, int sat, int off, int frq)
{
#if 0
    geph_t geph={0};
    int i,j,k,m,prn;
    unsigned char *p=raw->buff+6+off,buff[64],*fid;
    
    satsys(sat,&prn);
    
    if (raw->len<24+off) {
        return -1;
    }
    for (i=k=0;i<4;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    /* test hamming of glonass string */
    if (!test_glostr(buff)) {
        return -1;
    }
    m=loc_getbitu(buff,1,4);
    if (m<1||15<m) {
        return -1;
    }
    /* flush frame buffer if frame-id changed */
    fid=raw->subfrm[sat-1]+150;
    if (fid[0]!=buff[12]||fid[1]!=buff[13]) {
        for (i=0;i<4;i++) memset(raw->subfrm[sat-1]+i*10,0,10);
        memcpy(fid,buff+12,2); /* save frame-id */
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    
    if (m!=4) return 0;
    
    /* decode glonass ephemeris strings */
    geph.tof=raw->time;
    if (!decode_glostr(raw->subfrm[sat-1],&geph)||geph.sat!=sat) return 0;
    geph.frq=frq-7;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (geph.iode==raw->nav.geph[prn-1].iode) return 0; /* unchanged */
    }
    raw->nav.geph[prn-1]=geph;
    raw->ephsat=sat;
#endif
    return 2;
}
/* decode sbas navigation data -----------------------------------------------*/
static int decode_snav(raw_t *raw, int sat, int off)
{
#if 0
    int i,j,k,prn,tow,week;
    unsigned char *p=raw->buff+6+off,buff[64];
    
    if (raw->len<40+off) {
        return -1;
    }
    tow=(int)time2gpst(timeadd(raw->time,-1.0),&week);
    satsys(sat,&prn);
    raw->sbsmsg.prn=prn;
    raw->sbsmsg.tow=tow;
    raw->sbsmsg.week=week;
    for (i=k=0;i<8;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    memcpy(raw->sbsmsg.msg,buff,29);
    raw->sbsmsg.msg[28]&=0xC0;
#endif
    return 3;
}
#endif
/* decode ubx-rxm-sfrbx: raw subframe data (ref [3]) -------------------------*/
static int decode_rxmsfrbx(UbloxDecoder_t* decoder)
{
    
    int prn,sat,sys;
    unsigned char *p= decoder->buff+6;
    
#if 0    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX RXM-SFRBX (%4d): sys=%d prn=%3d",raw->len,
                U1(p),U1(p+1));
    }
#endif
   /* if (!(sys=ubx_sys(U1(p)))) {
        return -1;
    }*/
    sys = ubx_sys(U1(p));
    //decoder->navsys = ublox_sys(getU1(p));

    prn=U1(p+1);
    if (!(sat= gnss_cvt_Svid2SvIndex(prn, sys))) {
        return -1;
    }
    if (sys== C_GNSS_QZS) {
      decoder->ephsat = prn + 192;
    }
    else {
      decoder->ephsat = prn;
    }
    decoder->ephsys = sys;
    switch (sys) {
        case C_GNSS_GPS: return decode_nav (decoder,sat,8);
        case C_GNSS_QZS:  return decode_nav (decoder,sat,8);
        case C_GNSS_GAL:  return decode_enav(decoder,sat,8);
        case C_GNSS_BDS3: return decode_cnav(decoder,sat,8);
        //case SYS_GLO: return decode_gnav(raw,sat,8,getU1(p+3));
        //case SYS_SBS: return decode_snav(raw,sat,8);
    }
    return 2;
}
/* decode ublox raw message --------------------------------------------------*/
static int8_t ublox_decoder(UbloxDecoder_t* decoder)
{
     
    int8_t ret = -1;
    int type=(U1(decoder->buff+2)<<8)+U1(decoder->buff+3);
    
    decoder->type_id = type;
    
    /* checksum */
    if (!checksum(decoder->buff, decoder->length)) {
        return UBX_ERR_CHECKSUM;
    }
    switch (type) {
      case ID_RXMRAWX:  ret=decode_rxmrawx(decoder); break;
      case ID_RXMSFRBX: ret=decode_rxmsfrbx(decoder); break;
    }
#if 0
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX 0x%02X 0x%02X (%4d)",type>>8,type&0xF,
                raw->len);
    }
#endif
    return ret;
}
/* sync code -----------------------------------------------------------------*/
static int sync_ubx(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==UBXSYNC1&&buff[1]==UBXSYNC2;
}
/* input ublox raw message from stream -----------------------------------------
* fetch next ublox raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL    : input all ephemerides
*          -INVCP     : invert polarity of carrier-phase
*          -TADJ=tint : adjust time tags to multiples of tint (sec)
*          -STD_SLIP=std: slip by std-dev of carrier phase under std
*
*          The supported messages are as follows.
*
*          UBX-RXM-RAW  : raw measurement data
*          UBX-RXM-RAWX : multi-gnss measurement data
*          UBX-RXM-SFRB : subframe buffer
*          UBX-RXM-SFRBX: subframe buffer extension
*
*          UBX-TRK-MEAS and UBX-TRK-SFRBX are based on NEO-M8N (F/W 2.01).
*          UBX-TRK-D5 is based on NEO-7N (F/W 1.00). They are not formally
*          documented and not supported by u-blox.
*          Users can use these messages by their own risk.
*-----------------------------------------------------------------------------*/
int8_t ublox_dec_input(UbloxDecoder_t* decoder, uint8_t data)
{
    /* synchronize frame */
    if (decoder->nbyte==0) {
        if (!sync_ubx(decoder->buff,data)) return 0;
        decoder->nbyte=2;
        return 0;
    }
    decoder->buff[decoder->nbyte++]=data;

    if (decoder->nbyte==6) {
        if ((decoder->length=U2(decoder->buff+4)+8)>MAXRAWLEN) {
          decoder->nbyte=0;
            return UBX_ERR_LENTH;
        }
    }
    if (decoder->nbyte<6|| decoder->nbyte< decoder->length) return 0;
    decoder->nbyte=0;
    
    /* decode ublox raw message */
    return ublox_decoder(decoder);
}
/* input ublox raw message from file -------------------------------------------
* fetch next ublox raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_ubxf(raw_t *raw, FILE *fp)
{
  return 0;
#if 0
    int i,data;
 
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_ubx(raw->buff,(unsigned char)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+2,1,4,fp)<4) return -2;
    raw->nbyte=6;
    
    if ((raw->len=getU2(raw->buff+4)+8)>MAXRAWLEN) {
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+6,1,raw->len-6,fp)<(size_t)(raw->len-6)) return -2;
    raw->nbyte=0;
    
    /* decode ubx raw message */
    //return decode_ubx(raw);
#endif
}
/* generate ublox binary message -----------------------------------------------
* generate ublox binary message from message string
* args   : char  *msg   IO     message string 
*            "CFG-PRT   portid res0 res1 mode baudrate inmask outmask flags"
*            "CFG-USB   vendid prodid res1 res2 power flags vstr pstr serino"
*            "CFG-MSG   msgid rate0 rate1 rate2 rate3 rate4 rate5 rate6"
*            "CFG-NMEA  filter version numsv flags"
*            "CFG-RATE  meas nav time"
*            "CFG-CFG   clear_mask save_mask load_mask [dev_mask]"
*            "CFG-TP    interval length status time_ref res adelay rdelay udelay"
*            "CFG-NAV2  ..."
*            "CFG-DAT   maja flat dx dy dz rotx roty rotz scale"
*            "CFG-INF   protocolid res0 res1 res2 mask0 mask1 mask2 ... mask5"
*            "CFG-RST   navbbr reset res"
*            "CFG-RXM   gpsmode lpmode"
*            "CFG-ANT   flags pins"
*            "CFG-FXN   flags treacq tacq treacqoff tacqoff ton toff res basetow"
*            "CFG-SBAS  mode usage maxsbas res scanmode"
*            "CFG-LIC   key0 key1 key2 key3 key4 key5"
*            "CFG-TM    intid rate flags"
*            "CFG-TM2   ch res0 res1 rate flags"
*            "CFG-TMODE tmode posx posy posz posvar svinmindur svinvarlimit"
*            "CFG-EKF   ..."
*            "CFG-GNSS  ..."
*            "CFG-ITFM  conf conf2"
*            "CFG-LOGFILTER ver flag min_int time_thr speed_thr pos_thr"
*            "CFG-NAV5  ..."
*            "CFG-NAVX5 ..."
*            "CFG-ODO   ..."
*            "CFG-PM2   ..."
*            "CFG-PWR   ver rsv1 rsv2 rsv3 state"
*            "CFG-RINV  flag data ..."
*            "CFG-SMGR  ..."
*            "CFG-TMODE2 ..."
*            "CFG-TMODE3 ..."
*            "CFG-TPS   ..."
*            "CFG-TXSLOT ..."
*          unsigned char *buff O binary message
* return : length of binary message (0: error)
* note   : see reference [1][3] for details.
*          the following messages are not supported:
*             CFG-DOSC,CFG-ESRC
*-----------------------------------------------------------------------------*/
extern int gen_ubx(const char *msg, unsigned char *buff)
{
#if 0
    const char *cmd[]={
        "PRT","USB","MSG","NMEA","RATE","CFG","TP","NAV2","DAT","INF",
        "RST","RXM","ANT","FXN","SBAS","LIC","TM","TM2","TMODE","EKF",
        "GNSS","ITFM","LOGFILTER","NAV5","NAVX5","ODO","PM2","PWR","RINV","SMGR",
        "TMODE2","TMODE3","TPS","TXSLOT",""
    };
    const unsigned char id[]={
        0x00,0x1B,0x01,0x17,0x08,0x09,0x07,0x1A,0x06,0x02,
        0x04,0x11,0x13,0x0E,0x16,0x80,0x10,0x19,0x1D,0x12,
        0x3E,0x39,0x47,0x24,0x23,0x1E,0x3B,0x57,0x34,0x62,
        0x36,0x71,0x31,0x53
    };
    const int prm[][32]={
        {FU1,FU1,FU2,FU4,FU4,FU2,FU2,FU2,FU2},    /* PRT */
        {FU2,FU2,FU2,FU2,FU2,FU2,FS32,FS32,FS32}, /* USB */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1},        /* MSG */
        {FU1,FU1,FU1,FU1},                        /* NMEA */
        {FU2,FU2,FU2},                            /* RATE */
        {FU4,FU4,FU4,FU1},                        /* CFG */
        {FU4,FU4,FI1,FU1,FU2,FI2,FI2,FI4},        /* TP */
        {FU1,FU1,FU2,FU1,FU1,FU1,FU1,FI4,FU1,FU1,FU1,FU1,FU1,FU1,FU2,FU2,FU2,FU2,
         FU2,FU1,FU1,FU2,FU4,FU4},                /* NAV2 */
        {FR8,FR8,FR4,FR4,FR4,FR4,FR4,FR4,FR4},    /* DAT */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1}, /* INF */
        {FU2,FU1,FU1},                            /* RST */
        {FU1,FU1},                                /* RXM */
        {FU2,FU2},                                /* ANT */
        {FU4,FU4,FU4,FU4,FU4,FU4,FU4,FU4},        /* FXN */
        {FU1,FU1,FU1,FU1,FU4},                    /* SBAS */
        {FU2,FU2,FU2,FU2,FU2,FU2},                /* LIC */
        {FU4,FU4,FU4},                            /* TM */
        {FU1,FU1,FU2,FU4,FU4},                    /* TM2 */
        {FU4,FI4,FI4,FI4,FU4,FU4,FU4},            /* TMODE */
        {FU1,FU1,FU1,FU1,FU4,FU2,FU2,FU1,FU1,FU2}, /* EKF */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU4},    /* GNSS */
        {FU4,FU4},                                /* ITFM */
        {FU1,FU1,FU2,FU2,FU2,FU4},                /* LOGFILTER */
        {FU2,FU1,FU1,FI4,FU4,FI1,FU1,FU2,FU2,FU2,FU2,FU1,FU1,FU1,FU1,FU1,FU1,FU2,
         FU1,FU1,FU1,FU1,FU1,FU1},                /* NAV5 */
        {FU2,FU2,FU4,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU2,FU1,FU1,FU1,FU1,
         FU1,FU1,FU1,FU1,FU1,FU1,FU2},            /* NAVX5 */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1},    /* ODO */
        {FU1,FU1,FU1,FU1,FU4,FU4,FU4,FU4,FU2,FU2}, /* PM2 */
        {FU1,FU1,FU1,FU1,FU4},                    /* PWR */
        {FU1,FU1},                                /* RINV */
        {FU1,FU1,FU2,FU2,FU1,FU1,FU2,FU2,FU2,FU2,FU4}, /* SMGR */
        {FU1,FU1,FU2,FI4,FI4,FI4,FU4,FU4,FU4},    /* TMODE2 */
        {FU1,FU1,FU2,FI4,FI4,FI4,FU4,FU4,FU4},    /* TMODE3 */
        {FU1,FU1,FU1,FU1,FI2,FI2,FU4,FU4,FU4,FU4,FI4,FU4}, /* TPS */
        {FU1,FU1,FU1,FU1,FU4,FU4,FU4,FU4,FU4}     /* TXSLOT */
    };
    unsigned char *q=buff;
    char mbuff[1024],*args[32],*p;
    int i,j,n,narg=0;
    
    
    strcpy(mbuff,msg);
    for (p=strtok(mbuff," ");p&&narg<32;p=strtok(NULL," ")) {
        args[narg++]=p;
    }
    if (narg<1||strncmp(args[0],"CFG-",4)) return 0;
    
    for (i=0;*cmd[i];i++) {
        if (!strcmp(args[0]+4,cmd[i])) break;
    }
    if (!*cmd[i]) return 0;
    
    *q++=UBXSYNC1;
    *q++=UBXSYNC2;
    *q++=UBXCFG;
    *q++=id[i];
    q+=2;
    for (j=1;prm[i][j-1]||j<narg;j++) {
        switch (prm[i][j-1]) {
            case FU1 : setU1(q,j<narg?(unsigned char )atoi(args[j]):0); q+=1; break;
            case FU2 : setU2(q,j<narg?(unsigned short)atoi(args[j]):0); q+=2; break;
            case FU4 : setU4(q,j<narg?(unsigned int  )atoi(args[j]):0); q+=4; break;
            case FI1 : setI1(q,j<narg?(signed char   )atoi(args[j]):0); q+=1; break;
            case FI2 : setI2(q,j<narg?(short         )atoi(args[j]):0); q+=2; break;
            case FI4 : setI4(q,j<narg?(int           )atoi(args[j]):0); q+=4; break;
            case FR4 : setR4(q,j<narg?(float         )atof(args[j]):0); q+=4; break;
            case FR8 : setR8(q,j<narg?(double)atof(args[j]):0); q+=8; break;
            case FS32: sprintf((char *)q,"%-32.32s",j<narg?args[j]:""); q+=32; break;
            default  : setU1(q,j<narg?(unsigned char )atoi(args[j]):0); q+=1; break;
        }
    }
    n=(int)(q-buff)+2;
    setU2(buff+4,(unsigned short)(n-8));
    setcs(buff,n);
    
    return n;
#else
	return 0;
#endif
}

/**
 * @brief  Extract GnssMeasBlock from ULBOX Decoder to GnssMeasBlock
 * @return none
 */
 void ublox_ExtractMeasBlk(UbloxDecoder_t* ublox, GnssMeasBlock_t* meas_blk)
{
  uint8_t u_MaxMeasCount = sizeof(meas_blk->z_meas) / sizeof(meas_blk->z_meas[0]);

  if (any_Ptrs_Null(2, ublox, meas_blk))
  {
    return;
  }

  meas_blk->u_version = VERSION_GNSS_MEAS_BLOCK;
  meas_blk->w_size = sizeof(GnssMeasBlock_t);
  meas_blk->z_Clock.z_gpsTime.w_week = ublox->w_gpsWeek;///???
  meas_blk->z_Clock.z_gpsTime.q_towMsec = ublox->q_towMsec;///???

  for (uint32_t i = 0; i < ublox->nobs; i++)
  {
    if (i >= u_MaxMeasCount)
    {
      LOGI(TAG_SM, "Meas Index %d more than Max %d\n", i, u_MaxMeasCount);
      break;
    }


    for (uint32_t j = 0; j < NFREQ ; j++)
    {
      GnssMeas_t* p_meas = &meas_blk->z_meas[meas_blk->w_measCount];
      
      p_meas->z_measStatusFlag.b_valid = TRUE;
      p_meas->z_measStatusFlag.b_prValid = TRUE;
      p_meas->z_measStatusFlag.b_drValid = TRUE;
      p_meas->z_measStatusFlag.b_cpValid = TRUE;
      p_meas->u_svid = ublox->obs[i].svid;
      p_meas->u_constellation = ublox->obs[i].sys;
      p_meas->f_cn0 = ublox->obs[i].snr[j];
      p_meas->u_LLI = ublox->obs[i].lli[j];
      p_meas->u_signal = ublox->obs[i].code[j];
      p_meas->d_carrierPhase = ublox->obs[i].carrier_phase[j];
      p_meas->d_pseudoRange = ublox->obs[i].pseudorange[j];
      //p_meas->d_doppler = ublox->obs[i].doppler[j] * ubx_getwavelen(ublox->obs[i].sys,j) * (-1.0);
      p_meas->d_doppler = ublox->obs[i].doppler[j] * wavelength(ublox->obs[i].code[j]) * (-1.0);
      if (ublox->obs[i].carrier_phase[j] != 0 || ublox->obs[i].pseudorange[j] != 0 || ublox->obs[i].doppler[j] != 0) {
        meas_blk->w_measCount++;
      }

    }

  }
}
 void convert_ublox_2_GnssEph(UbloxDecoder_t* ublox, gnss_Ephemeris_t* eph, uint8_t sys)
{
  eph->u_constellation = sys;
  if (sys == C_GNSS_GPS || sys == C_GNSS_QZS)
  {
    if (ublox->ephemeris.week > 0 && ublox->ephemeris.week < 2048) {
      ublox->ephemeris.week = 2048 + ublox->ephemeris.week % 1024;
    }
  }

  if (sys== C_GNSS_BDS3) {
    eph->eph.z_bdsEph.svid = ublox->ephsat;
    eph->eph.z_bdsEph.week = ublox->ephemeris.week;
    eph->eph.z_bdsEph.accuracy = ublox->ephemeris.sva;
    eph->eph.z_bdsEph.health = ublox->ephemeris.svh;
    eph->eph.z_bdsEph.iodc = ublox->ephemeris.iodc;
    eph->eph.z_bdsEph.code = ublox->ephemeris.code;
    eph->eph.z_bdsEph.fit = (uint8_t)ublox->ephemeris.fit;
    eph->eph.z_bdsEph.iode = ublox->ephemeris.iode;
    eph->eph.z_bdsEph.sqrt_A = sqrt(ublox->ephemeris.A);
    eph->eph.z_bdsEph.e = ublox->ephemeris.e;
    eph->eph.z_bdsEph.M0 = ublox->ephemeris.M0;
    eph->eph.z_bdsEph.Omega0 = ublox->ephemeris.OMG0;
    eph->eph.z_bdsEph.Omega = ublox->ephemeris.omg;
    eph->eph.z_bdsEph.OmegaDot = ublox->ephemeris.OMGd;
    eph->eph.z_bdsEph.i0 = ublox->ephemeris.i0;
    eph->eph.z_bdsEph.idot = ublox->ephemeris.idot;
    eph->eph.z_bdsEph.DeltaN = ublox->ephemeris.deln;
    eph->eph.z_bdsEph.crc = ublox->ephemeris.crc;
    eph->eph.z_bdsEph.crs = ublox->ephemeris.crs;
    eph->eph.z_bdsEph.cuc = ublox->ephemeris.cuc;
    eph->eph.z_bdsEph.cus = ublox->ephemeris.cus;
    eph->eph.z_bdsEph.cic = ublox->ephemeris.cic;
    eph->eph.z_bdsEph.cis = ublox->ephemeris.cis;
    eph->eph.z_bdsEph.af0 = ublox->ephemeris.f0;
    eph->eph.z_bdsEph.af1 = ublox->ephemeris.f1;
    eph->eph.z_bdsEph.af2 = ublox->ephemeris.f2;
    eph->eph.z_bdsEph.toc = ag_gnss_time2gpst(ublox->ephemeris.toc, NULL)-14;///????
    eph->eph.z_bdsEph.toe = ag_gnss_time2gpst(ublox->ephemeris.toe, NULL)-14;//???
    eph->eph.z_bdsEph.tgdB1I = ublox->ephemeris.tgd[0];
    eph->eph.z_bdsEph.tgdB2I = ublox->ephemeris.tgd[1];
    BdsTime_t bdt = { 0 };
    tm_cvt_SetBdt(&bdt, eph->eph.z_bdsEph.week, eph->eph.z_bdsEph.toe);
    tm_cvt_BdtToGpst(&bdt, &eph->z_toeOfGpst);
  
  }
  else if (sys == C_GNSS_GAL) {
    eph->eph.z_galEph.svid = ublox->ephsat;
    eph->eph.z_galEph.week = ublox->ephemeris.week;
    eph->eph.z_galEph.accuracy = ublox->ephemeris.sva;
    eph->eph.z_galEph.E1health = ublox->ephemeris.svh;
    eph->eph.z_galEph.E1DVS = ublox->ephemeris.svh;
    eph->eph.z_galEph.E5bhealth= ublox->ephemeris.svh;
    eph->eph.z_galEph.E5bDVS= ublox->ephemeris.svh;
    eph->eph.z_galEph.iodc = ublox->ephemeris.iodc;
    eph->eph.z_galEph.code = ublox->ephemeris.code;
    eph->eph.z_galEph.fit = (uint8_t)ublox->ephemeris.fit;
    eph->eph.z_galEph.iode = ublox->ephemeris.iode;
    eph->eph.z_galEph.sqrt_A = sqrt(ublox->ephemeris.A);
    eph->eph.z_galEph.e = ublox->ephemeris.e;
    eph->eph.z_galEph.M0 = ublox->ephemeris.M0;
    eph->eph.z_galEph.Omega0 = ublox->ephemeris.OMG0;
    eph->eph.z_galEph.Omega = ublox->ephemeris.omg;
    eph->eph.z_galEph.OmegaDot = ublox->ephemeris.OMGd;
    eph->eph.z_galEph.i0 = ublox->ephemeris.i0;
    eph->eph.z_galEph.idot = ublox->ephemeris.idot;
    eph->eph.z_galEph.DeltaN = ublox->ephemeris.deln;
    eph->eph.z_galEph.crc = ublox->ephemeris.crc;
    eph->eph.z_galEph.crs = ublox->ephemeris.crs;
    eph->eph.z_galEph.cuc = ublox->ephemeris.cuc;
    eph->eph.z_galEph.cus = ublox->ephemeris.cus;
    eph->eph.z_galEph.cic = ublox->ephemeris.cic;
    eph->eph.z_galEph.cis = ublox->ephemeris.cis;
    eph->eph.z_galEph.af0 = ublox->ephemeris.f0;
    eph->eph.z_galEph.af1 = ublox->ephemeris.f1;
    eph->eph.z_galEph.af2 = ublox->ephemeris.f2;
    eph->eph.z_galEph.toc = ag_gnss_time2gpst(ublox->ephemeris.toc, NULL);///????
    eph->eph.z_galEph.toe = ag_gnss_time2gpst(ublox->ephemeris.toe, NULL);//???
    eph->eph.z_galEph.tgdE1E5a = ublox->ephemeris.tgd[0];
    eph->eph.z_galEph.tgdE1E5b = ublox->ephemeris.tgd[1];
    GalTime_t gst = { 0 };
    tm_cvt_SetGst(&gst, eph->eph.z_galEph.week, eph->eph.z_galEph.toe);
    tm_cvt_GstToGpst(&gst, &eph->z_toeOfGpst);
  }
  else {
    eph->eph.z_gpsEph.svid = ublox->ephsat;
    eph->eph.z_gpsEph.week = ublox->ephemeris.week;
    eph->eph.z_gpsEph.accuracy = ublox->ephemeris.sva;
    eph->eph.z_gpsEph.health = ublox->ephemeris.svh;
    eph->eph.z_gpsEph.iodc = ublox->ephemeris.iodc;
    eph->eph.z_gpsEph.code = ublox->ephemeris.code;
    eph->eph.z_gpsEph.fit = (uint8_t)ublox->ephemeris.fit;
    eph->eph.z_gpsEph.iode = ublox->ephemeris.iode;
    eph->eph.z_gpsEph.sqrt_A = sqrt(ublox->ephemeris.A);
    eph->eph.z_gpsEph.e = ublox->ephemeris.e;
    eph->eph.z_gpsEph.M0 = ublox->ephemeris.M0;
    eph->eph.z_gpsEph.Omega0 = ublox->ephemeris.OMG0;
    eph->eph.z_gpsEph.Omega = ublox->ephemeris.omg;
    eph->eph.z_gpsEph.OmegaDot = ublox->ephemeris.OMGd;
    eph->eph.z_gpsEph.i0 = ublox->ephemeris.i0;
    eph->eph.z_gpsEph.idot = ublox->ephemeris.idot;
    eph->eph.z_gpsEph.DeltaN = ublox->ephemeris.deln;
    eph->eph.z_gpsEph.crc = ublox->ephemeris.crc;
    eph->eph.z_gpsEph.crs = ublox->ephemeris.crs;
    eph->eph.z_gpsEph.cuc = ublox->ephemeris.cuc;
    eph->eph.z_gpsEph.cus = ublox->ephemeris.cus;
    eph->eph.z_gpsEph.cic = ublox->ephemeris.cic;
    eph->eph.z_gpsEph.cis = ublox->ephemeris.cis;
    eph->eph.z_gpsEph.af0 = ublox->ephemeris.f0;
    eph->eph.z_gpsEph.af1 = ublox->ephemeris.f1;
    eph->eph.z_gpsEph.af2 = ublox->ephemeris.f2;
    eph->eph.z_gpsEph.toc = ag_gnss_time2gpst(ublox->ephemeris.toc,NULL);///????
    eph->eph.z_gpsEph.toe = ag_gnss_time2gpst(ublox->ephemeris.toe,NULL);//???
    eph->eph.z_gpsEph.tgd = ublox->ephemeris.tgd[0];
    tm_cvt_SetGpst(&eph->z_toeOfGpst, eph->eph.z_gpsEph.week, eph->eph.z_gpsEph.toe);
    if (eph->u_constellation == C_GNSS_QZS) {
      memcpy(&eph->eph.z_qzsEph, &eph->eph.z_gpsEph, sizeof(GpsEphemeris_t));
    }
  }

}



/**
 * @brief  Extract GnssEphemeris from UBLOX Decoder to GnssEphemeris
 * @return none
 */
 void ublox_ExtractEph(UbloxDecoder_t* ublox, gnss_Ephemeris_t* eph)
{
  uint8_t sys;
  if (any_Ptrs_Null(2, ublox,eph))
  {
    return;
  }
  eph->u_version = VERSION_GNSS_EPHEMERIS;
  eph->u_constellation = ublox->ephsys;
  sys = ublox->ephsys;
  switch (sys) 
  {
    case C_GNSS_GPS: convert_ublox_2_GnssEph(ublox, eph, sys); break;
    case C_GNSS_BDS3: convert_ublox_2_GnssEph(ublox, eph, sys); break;
    case C_GNSS_GAL: convert_ublox_2_GnssEph(ublox, eph, sys); break;
    case C_GNSS_QZS: convert_ublox_2_GnssEph(ublox, eph, sys); break;
  }
   
}