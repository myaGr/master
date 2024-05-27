/**@file        ublox_dec.h
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
 #ifndef __UBLOX_DECODE_H__
 #define __UBLOX_DECODE_H__

 #include "cmn_def.h"
 #include "gnss_type.h"
 #include "gnss_def.h"

BEGIN_DECL

#ifdef __cplusplus
extern "C" {
#endif
 
 #undef MODULE_TAG
 #define MODULE_TAG OBJ_DATA

 #define UBXSYNC1    0xB5        /* ubx message sync code 1 */
 #define UBXSYNC2    0x62        /* ubx message sync code 2 */
 #define UBXCFG      0x06        /* ubx message cfg-??? */
 
 #define PREAMB_CNAV 0x8B        /* cnav preamble */

#define UBX_ERR_CHECKSUM       (-1)
#define UBX_ERR_LENTH          (-2)

#define MAXRAWLEN   4096                /* max length of receiver raw message */
#define MAXERRMSG   4096                /* max length of error/warning message */

#ifndef NFREQ
#define NFREQ           (2)
#endif

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2     frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ9       2.492028E9          /* S      frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1 frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2 frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 frequency (Hz) */
#define FREQ_B2_CMP   1.191795E9        /* BeiDou B2(B2a+B2b) frequency (Hz) */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A        (GAL) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A    30                  /* obs code: E6A        (GAL) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
#define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
#define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I    40                  /* obs code: B1I        (BDS) */
#define CODE_L2Q    41                  /* obs code: B1Q        (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define CODE_L5D    56      
#define CODE_L5P    57
#define MAXCODE     57                  /* max number of obs code */
 //#define ID_NAVSOL   0x0106      /* ubx message id: nav solution info */
 //#define ID_NAVTIME  0x0120      /* ubx message id: nav time gps */
 //#define ID_RXMRAW   0x0210      /* ubx message id: raw measurement data */
 //#define ID_RXMSFRB  0x0211      /* ubx message id: subframe buffer */
 //#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
 //#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */
 //#define ID_TRKD5    0x030A      /* ubx message id: trace mesurement data */
 //#define ID_TRKMEAS  0x0310      /* ubx message id: trace mesurement data */
 //#define ID_TRKSFRBX 0x030F      /* ubx message id: trace subframe buffer */

 #define FU1         1           /* ubx message field types */
 #define FU2         2
 #define FU4         3
 #define FI1         4
 #define FI2         5
 #define FI4         6
 #define FR4         7
 #define FR8         8
 #define FS32        9
 
 #define CPSTD_VALID 5           /* std-dev threshold of carrier-phase valid */
 typedef uint8_t U8;		/* 8-bit unsigned-integer */


#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */

/** UBLOX decoder buffer size ???*/
#define UBLOX_BUF_SIZE (1024 + 6)

#define MAXSAT		172
#define MAXOBS    120                  /* max number of obs in an epoch, USED IN PEDATAIN, DO NOT CHANGE THIS VALUE! */

 typedef struct {        /* time struct */
   time_t time;        /* time (s) expressed by standard time_t */
   double sec;         /* fraction of second under 1 s */
 } gtime_t;

 typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
   int sat;            /* satellite number */
   int iode, iodc;      /* IODE,IODC */
   int sva;            /* SV accuracy (URA index) */
   int svh;            /* SV health (0:ok) */
   int week;           /* GPS/QZS: gps week, GAL: galileo week */
   int code;           /* GPS/QZS: code on L2, GAL/CMP: data sources */
   int flag;           /* GPS/QZS: L2 P data flag, CMP: nav type */
   gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
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
 } eph_t;


typedef struct {
  uint32_t tow_ms; /** receiver sampling time (GPST) */
  uint8_t sys;     /** gnss_ConstellationType */
  uint8_t svid;    /** svid of each system */
  uint8_t sat;     /* satellite number of all systems*/
  uint8_t signal;  /** gnss_SignalType */
  uint8_t snr[2];     /* signal strength (0.25 dBHz) */
  uint8_t lli[2];     /* loss of lock indicator */
  uint8_t code[2];    /* code indicator (CODE_???) */
  double pseudorange[2];   /* observation data pseudorange (m) */
  double doppler[2];       /* observation data doppler frequency (Hz) */
  double carrier_phase[2]; /* observation data carrier-phase (cycle) */
  /* extent field */
  uint8_t prStd[2];
	uint8_t cpStd[2];
  uint8_t drStd[2];
  uint8_t trkStat[2];
  int8_t  glofcn[2]; /* glonass fcn, [-7,6], 127 means invalid */
  uint16_t lockTime[2];
} UbloxObs_t;


typedef struct {
  gtime_t time;       /* message time */
  uint32_t q_towMsec;
  uint32_t w_gpsWeek;
  uint32_t type_id;
  uint8_t buff[UBLOX_BUF_SIZE];
  unsigned char subfrm[MAXSAT][380];  /* subframe buffer ?????*/
  uint32_t nbyte;
  uint32_t length;
  uint8_t nobs;
  UbloxObs_t obs[128];
  eph_t ephemeris;
  uint8_t ephsys;
  uint8_t ephsat;
  //uint8_t E1health; /** SV health (0:ok) GAL*/
  //uint8_t E1DVS;    /** SV Data Validity Status (0:ok) */
  //uint8_t E5bhealth;/** SV health (0:ok) */
  //uint8_t E5bDVS;   /** SV Data Validity Status (0:ok) */
  /* extent field */
  double lockt[MAXSAT][2]; /* lock time (s) */
  unsigned char halfc[MAXSAT][2]; /* half-cycle add flag */
  int8_t  leapSec;
  uint8_t  recStat;
} UbloxDecoder_t;


typedef struct {        /* receiver raw data control type */
  gtime_t time;       /* message time */
  gtime_t tobs;       /* observation data time */
  UbloxObs_t obs;      /* raw observation data */
  int nbyte;          /* number of bytes in message buffer */
  int len;            /* message length (bytes) */
  unsigned char buff[MAXRAWLEN]; /* message buffer */
} raw_t;

int8_t ublox_dec_input(UbloxDecoder_t* decoder, uint8_t data);

/**
 * @brief  Extract GnssMeasBlock from UBLOX Decoder to GnssMeasBlock
 * @return none
 */
void ublox_ExtractMeasBlk(UbloxDecoder_t* ublox, GnssMeasBlock_t* meas);

/**
 * @brief  Extract GnssCorrBlock from RTCM Decoder to GnssMeasBlock
 * @return none
 */
void ublox_ExtractEph(UbloxDecoder_t* ublox, gnss_Ephemeris_t* eph);


#ifdef __cplusplus
}
#endif
END_DECL
#endif
