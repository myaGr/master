#include <math.h>
#include "gnss.h"
#include "gnss_comm.h"

#ifdef _WIN32
#include <Windows.h>
#elif defined(__GNUC__) && defined(__linux__)
#include <sys/time.h>
#endif

#if 0
const static double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
const static double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
const static double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */
#endif

double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};

double leapsInUse[MAXLEAPS+1][7];

#if 0

const uint32_t tbl_CRC24Q[]={
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

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
int32_t satno(int32_t sys, int32_t prn)
{
	if (prn <= 0) return 0;
	switch (sys)
	{
	case SYS_GPS:
		if (prn < MIN_GPS_PRN || prn > MAX_GPS_PRN) return 0;
		return prn-MIN_GPS_PRN+1;
	case SYS_GLO:
		if (prn < MIN_GLN_PRN || prn > MAX_GLN_PRN) return 0;
		return N_GPS_SVS+prn-MIN_GLN_PRN+1;
	case SYS_GAL:
		if (prn < MIN_GAL_PRN || prn > MAX_GAL_PRN) return 0;
		return N_GPS_SVS+N_GLN_SVS+prn-MIN_GAL_PRN+1;
	case SYS_QZS:
		if (prn < MIN_QZSS_PRN || prn > MAX_QZSS_PRN) return 0;
		return N_GPS_SVS+N_GLN_SVS+N_GAL_SVS+prn-MIN_QZSS_PRN+1;
	case SYS_CMP:
		if (prn < MIN_BDS_PRN || prn > MAX_BDS_PRN) return 0;
		return N_GPS_SVS+N_GLN_SVS+N_GAL_SVS+N_QZSS_SVS+prn-MIN_BDS_PRN+1;
		/*case SYS_LEO:
		if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
		return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNLEO+1;
		case SYS_SBS:
		if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
		return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATLEO+prn-MINPRNSBS+1;*/
	default:
		return 0;
	}
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
uint32_t asg_getbitu(const unsigned char *buff, int32_t pos, int32_t len)
{
	uint32_t bits=0;
	int32_t i;
	for (i = pos; i < pos+len; i++)
		bits = (bits<<1)+((buff[i/8]>>(7-i%8))&1u);
	return bits;
}

int32_t asg_getbits(const unsigned char *buff, int32_t pos, int32_t len)
{
	uint32_t bits = asg_getbitu(buff, pos, len);
	if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int32_t)bits;
	return (int32_t)(bits|(~0u<<len)); /* extend sign */
}

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
uint32_t crc24q(const unsigned char *buff, int32_t len)
{
	uint32_t crc=0;
	int32_t i;

	for (i=0; i<len; i++)
		crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];

	return crc;
}

int32_t decode_word(uint32_t word, unsigned char* data)
{
	const unsigned int hamming[]={
		0xBB1F3480,0x5D8F9A40,0xAEC7CD00,0x5763E680,0x6BB1F340,0x8B7A89C0
	};
	uint32_t parity = 0,w;
	int32_t i;

	if (NULL == data) return FALSE;

	if (word & 0x40000000) word ^= 0x3FFFFFC0;

	for (i = 0; i < 6; i++)
	{
		parity <<= 1;
		for (w = (word & hamming[i]) >> 6; w ;w >>= 1)
			parity ^= w & 1;
	}

	if (parity != (word & 0x3F)) return FALSE;
	for (i = 0; i < 3; i++) data[i] = (unsigned char)(word>>(22-i*8));
	return TRUE;
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec+=sec;
	tt=floor(t.sec);
	t.time+=(int)tt;
	t.sec-=tt;
	return t;
}

/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double timediff(gtime_t t1, gtime_t t2)
{
	return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double *ep)
{
	const int32_t doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
	gtime_t time={0};
	int32_t days,sec,year=(int32_t)ep[0],mon=(int32_t)ep[1],day=(int32_t)ep[2];

	if (year<1970||2099<year||mon<1||12<mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
	sec=(int32_t)floor(ep[5]);
	time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
	time.sec=ep[5]-sec;
	return time;
}

/* get current time in utc -----------------------------------------------------
* get current time in utc
* args   : none
* return : current time in utc
*-----------------------------------------------------------------------------*/
static double timeoffset_=0.0;		  /* time offset (s) */

gtime_t timeget(void)
{
	gtime_t time;
	double ep[6]={0};
#ifdef _WIN32
	SYSTEMTIME ts;

	GetSystemTime(&ts); /* utc */
	ep[0]=ts.wYear; ep[1]=ts.wMonth;  ep[2]=ts.wDay;
	ep[3]=ts.wHour; ep[4]=ts.wMinute; ep[5]=ts.wSecond+ts.wMilliseconds*1E-3;
#else
	struct timeval tv;
	struct tm *tt = NULL;
	struct tm result;

	if (!gettimeofday(&tv,NULL) && (tt = gmtime_r(&tv.tv_sec, &result))) {
		ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
		ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
	}
#endif
	time=epoch2time(ep);

#ifdef CPUTIME_IN_GPST /* cputime operated in gpst */
	time=gpst2utc(time);
#endif
	return timeadd(time,timeoffset_);
}
#endif

/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t utc2gpst(gtime_t t)
{
	int32_t i;

	for (i=0;leapsInUse[i][0]>0;i++) {
		if (timediff(t, gnss_epoch2time(leapsInUse[i]))>=0.0) return timeadd(t,-leapsInUse[i][6]);
	}
	return t;
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu;
	int i;

	for (i=0;leapsInUse[i][0]>0;i++) {
		tu=timeadd(t,leapsInUse[i][6]);
		if (timediff(tu, gnss_epoch2time(leapsInUse[i]))>=0.0) return tu;
	}
	return t;
}

#if 0
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t gpst2time(int32_t week, double sec)
{
    gtime_t t=epoch2time(gpst0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double time2gpst(gtime_t t, int *week)
{
	gtime_t t0=epoch2time(gpst0);
	time_t sec=t.time-t0.time;
	int32_t w=(int)(sec/(86400*7));

	if (week) *week=w;
	return (double)(sec-w*86400*7)+t.sec;
}

/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t bdt2time(int week, double sec)
{
	gtime_t t=epoch2time(bdt0);

	if (sec<-1E9||1E9<sec) sec=0.0;
	t.time+=86400*7*week+(int)sec;
	t.sec=sec-(int)sec;
	return t;
}
/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
double time2bdt(gtime_t t, int *week)
{
	gtime_t t0=epoch2time(bdt0);
	time_t sec=t.time-t0.time;
	int w=(int)(sec/(86400*7));

	if (week) *week=w;
	return (double)(sec-w*86400*7)+t.sec;
}


/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
int32_t adjgpsweek(int32_t week)
{
	int32_t w;
	(void)time2gpst(utc2gpst(timeget()),&w);
	if (w<1560) w=1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */
	return week+(w-week+512)/1024*1024;
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
void time2epoch(gtime_t t, double *ep)
{
	const int mday[]={ /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days,sec,mon,day;

	/* leap year if year%4==0 in 1901-2099 */
	days=(int)(t.time/86400);
	sec=(int)(t.time-(time_t)days*86400);
	for (day=days%1461,mon=0;mon<48;mon++) {
		if (day>=mday[mon]) day-=mday[mon]; else break;
	}
	ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
	ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
double time2doy(gtime_t t)
{
	double ep[6];

	gnss_time2epoch(t,ep);
	ep[1]=ep[2]=1.0; ep[3]=ep[4]=ep[5]=0.0;
	return timediff(t,epoch2time(ep))/86400.0+1.0;
}
#endif