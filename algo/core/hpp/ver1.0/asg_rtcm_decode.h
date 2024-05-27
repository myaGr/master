/*------------------------------------------------------------------------------
* rtklib.h : rtklib constants, types and function prototypes
*
*          Copyright (C) 2007-2019 by T.TAKASU, All rights reserved.
*
* options : -DENAGLO   enable GLONASS
*           -DENAGAL   enable Galileo
*           -DENAQZS   enable QZSS
*           -DENACMP   enable BeiDou
*           -DENAIRN   enable IRNSS
*           -DNFREQ=n  set number of obs codes/frequencies
*           -DNEXOBS=n set number of extended obs codes
*           -DMAXOBS=n set max number of obs data in an epoch
*           -DEXTLEX   enable QZSS LEX extension
*           -DWIN32    use WIN32 API
*           -DWIN_DLL  generate library as Windows DLL
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/13 1.0  rtklib ver.1.0.0
*           2007/03/20 1.1  rtklib ver.1.1.0
*           2008/07/15 1.2  rtklib ver.2.1.0
*           2008/10/19 1.3  rtklib ver.2.1.1
*           2009/01/31 1.4  rtklib ver.2.2.0
*           2009/04/30 1.5  rtklib ver.2.2.1
*           2009/07/30 1.6  rtklib ver.2.2.2
*           2009/12/25 1.7  rtklib ver.2.3.0
*           2010/07/29 1.8  rtklib ver.2.4.0
*           2011/05/27 1.9  rtklib ver.2.4.1
*           2013/03/28 1.10 rtklib ver.2.4.2
*           2016/01/26 1.11 rtklib ver.2.4.3
*-----------------------------------------------------------------------------*/
#ifndef __ASG_RTCM_DECODE_H__
#define __ASG_RTCM_DECODE_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif


#define RTCM_BUFFER_LEN (1024)

typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} asg_gtime_t;


typedef struct {        /* observation data record */
    asg_gtime_t time;       /* receiver sampling time (GPST) */
    unsigned char sat,rcv; /* satellite/receiver number */
    unsigned char SNR [NFREQ+NEXOBS]; /* signal strength (0.25 dBHz) */
    unsigned char LLI [NFREQ+NEXOBS]; /* loss of lock indicator */
    unsigned char code[NFREQ+NEXOBS]; /* code indicator (CODE_???) */
    double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
    double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
    float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */
	int    sys; /* SYS_GPS, SYS_GLO, SYS_GAL... */
	int    prn; /* svid of each system */
} asg_obsd_t;

typedef struct {        /* observation data */
    int n,nmax;         /* number of obervation data/allocated */
    asg_obsd_t data[MAXOBS];    /* observation data records */
} asg_obs_t;

typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
	int sat;            /* satellite number */
	int iode, iodc;      /* IODE,IODC */
	int sva;            /* SV accuracy (URA index) */
	int svh;            /* SV health (0:ok) */
	int week;           /* GPS/QZS: gps week, GAL: galileo week */
	int code;           /* GPS/QZS: code on L2, GAL/CMP: data sources */
	int flag;           /* GPS/QZS: L2 P data flag, CMP: nav type */
	asg_gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
						/* SV orbit parameters */
	double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;        /* Toe (s) in week */
	double fit;         /* fit interval (h) */
	double f0, f1, f2;    /* SV clock parameters (af0,af1,af2) */
	double tgd[4];      /* group delay parameters, unit: seconds */
						/* GPS/QZS:tgd[0]=TGD */
						/* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
						/* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
	double Adot, ndot;   /* Adot,ndot for CNAV */
} asg_eph_t;


typedef struct {        /* navigation data type */
	int n, nmax;         /* number of broadcast ephemeris */
#if 0
	int ng, ngmax;       /* number of glonass ephemeris */
	int ns, nsmax;       /* number of sbas ephemeris */
	int ne, nemax;       /* number of precise ephemeris */
	int nc, ncmax;       /* number of precise clock */
	int na, namax;       /* number of almanac data */
	int nt, ntmax;       /* number of tec grid data */
	int nf, nfmax;       /* number of satellite fcb data */
#endif
	asg_eph_t* eph;     /* GPS/QZS/GAL ephemeris */
#if 0
	geph_t * geph;       /* GLONASS ephemeris */
	seph_t* seph;       /* SBAS ephemeris */
	peph_t* peph;       /* precise ephemeris */
	pclk_t* pclk;       /* precise clock */
	alm_t* alm;         /* almanac data */
	tec_t* tec;         /* tec grid data */
	fcbd_t* fcb;        /* satellite fcb data */
	erp_t  erp;         /* earth rotation parameters */
	double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
	double utc_glo[4];  /* GLONASS UTC GPS time parameters */
	double utc_gal[4];  /* Galileo UTC GPS time parameters */
	double utc_qzs[4];  /* QZS UTC GPS time parameters */
	double utc_cmp[4];  /* BeiDou UTC parameters */
	double utc_irn[4];  /* IRNSS UTC parameters */
	double utc_sbs[4];  /* SBAS UTC parameters */
	double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
	double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_irn[8];  /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	int leaps;          /* leap seconds (s) */
#endif
	double lam[MAXSAT][NFREQ]; /* carrier wave lengths (m) */
#if 0
	double cbias[MAXSAT][3]; /* satellite dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
	double rbias[MAXRCV][2][3]; /* receiver dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
	double wlbias[MAXSAT];   /* wide-lane bias (cycle) */
	double glo_cpbias[4];    /* glonass code-phase bias {1C,1P,2C,2P} (m) */
	char glo_fcn[MAXPRNGLO + 1]; /* glonass frequency channel number + 8 */
	pcv_t pcvs[MAXSAT]; /* satellite antenna pcv */
	sbssat_t sbssat;    /* SBAS satellite corrections */
	sbsion_t sbsion[MAXBAND + 1]; /* SBAS ionosphere corrections */
	dgps_t dgps[MAXSAT]; /* DGPS corrections */
	ssr_t ssr[MAXSAT];  /* SSR corrections */
	lexeph_t lexeph[MAXSAT]; /* LEX ephemeris */
	lexion_t lexion;    /* LEX ionosphere correction */
	pppcorr_t pppcorr;  /* ppp corrections */
#endif
} asg_nav_t;

typedef struct {        /* station parameter type */
    //char name   [MAXANT]; /* marker name */
    //char marker [MAXANT]; /* marker number */
    //char antdes [MAXANT]; /* antenna descriptor */
    //char antsno [MAXANT]; /* antenna serial number */
    //char rectype[MAXANT]; /* receiver type descriptor */
    //char recver [MAXANT]; /* receiver firmware version */
    //char recsno [MAXANT]; /* receiver serial number */
    int antsetup;       /* antenna setup id */
    int itrf;           /* ITRF realization year */
    int deltype;        /* antenna delta type (0:enu,1:xyz) */
    double pos[3];      /* station position (ecef) (m) */
    double del[3];      /* antenna position delta (e/n/u or x/y/z) (m) */
    double hgt;         /* antenna height (m) */
} asg_sta_t;


typedef struct {
	uint8_t pos_flag; // 1:No Fix,2:Fix 2D,3:Fix 3D,4:RTK Fixed,5:RTK Float,6:PPS
	int leapsecond;
	int week;
	double second;
	double lla_pos[3];
	double enu_vel[3];
	double clock_drift;
	double clock_offset;
} asg_location_t;

typedef struct {        /* RTCM control struct type */
    int staid;          /* station id */
    int stah;           /* station health */
    int seqno;          /* sequence number for rtcm 2 or iods msm */
    int outtype;        /* output message type */
    asg_gtime_t time;       /* message time */
    //asg_gtime_t time_s;     /* message start time */
    asg_obs_t obs;          /* observation data (uncorrected) */
	asg_nav_t nav;          /* satellite ephemerides */
    asg_sta_t sta;          /* station parameters */
    //dgps_t *dgps;       /* output of dgps corrections */
    //ssr_t ssr[MAXSAT];  /* output of ssr corrections */
    char msg[128];      /* special message */
    char msgtype[256];  /* last message type */
    char msmtype[6][128]; /* msm signal types */
    int obsflag;        /* obs data complete flag (1:ok,0:not complete) */
    int ephsat;         /* update satellite of ephemeris */
    //double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
    unsigned short lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */
    //unsigned short loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
    //asg_gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
    int nbyte;          /* number of bytes in message buffer */ 
    int nbit;           /* number of bits in word buffer */ 
    int len;            /* message length (bytes) */
    unsigned char buff[RTCM_BUFFER_LEN]; /* message buffer */
    unsigned int word;  /* word buffer for rtcm 2 */
    //unsigned int nmsg2[100]; /* message count of RTCM 2 (1-99:1-99,0:other) */
    //unsigned int nmsg3[400]; /* message count of RTCM 3 (1-299:1001-1299,300-399:2000-2099,0:ohter) */
    //char opt[256];      /* RTCM dependent options */
	int id_type;       /* message type */
	asg_location_t  location;
} asg_rtcm_lite_t;

double  		asg_str2num(const char* s, int i, int n);
int     		asg_str2time(const char* s, int i, int n, asg_gtime_t* t);
void    		asg_time2str(asg_gtime_t t, char* str, int n);
asg_gtime_t 	asg_epoch2time(const double* ep);
void    		asg_time2epoch(asg_gtime_t t, double* ep);
asg_gtime_t 	asg_gpst2time(int week, double sec);
double  		rtklib_time2gpst(asg_gtime_t t, int* week);
asg_gtime_t 	asg_gst2time(int week, double sec);
double  		asg_time2gst(asg_gtime_t t, int* week);
asg_gtime_t 	asg_bdt2time(int week, double sec);
double  		asg_time2bdt(asg_gtime_t t, int* week);
char* 			asg_time_str(asg_gtime_t t, int n);
asg_gtime_t 	asg_timeadd(asg_gtime_t t, double sec);
double  		asg_timediff(asg_gtime_t t1, asg_gtime_t t2);
asg_gtime_t 	asg_gpst2utc(asg_gtime_t t);
asg_gtime_t 	asg_utc2gpst(asg_gtime_t t);
asg_gtime_t 	asg_gpst2bdt(asg_gtime_t t);
asg_gtime_t 	asg_bdt2gpst(asg_gtime_t t);
asg_gtime_t 	asg_timeget(void);
void    		asg_timeset(asg_gtime_t t);
double  		asg_utc2gmst(asg_gtime_t t, double ut1_utc);
int 			asg_satno(int sys, int prn);
int 			asg_satsys(int sat, int* prn);

int				asg_init_rtcm(asg_rtcm_lite_t* rtcm);
int				asg_init_rtcm_with_nav(asg_rtcm_lite_t* rtcm);
int				asg_input_rtcm3(asg_rtcm_lite_t* rtcm, unsigned char data);
double			asg_satwavelen(int sat, int frq, const void* nav);

extern int convert_obs_raw_2_rtcmlite(raw_t *raw, asg_rtcm_lite_t *plite);
extern int convert_nav_raw_2_rtcmlite(raw_t *raw, asg_nav_t *pnav);

#ifdef __cplusplus
}
#endif


#endif /* RTKLIB_H */
