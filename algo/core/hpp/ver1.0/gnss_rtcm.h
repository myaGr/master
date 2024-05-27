#ifndef _GNSS_RTCM_H_
#define _GNSS_RTCM_H_

#include "gnss_types.h"
#include "gnss.h"
#include "rtklib.h"
#ifdef USE_ASG_SDK
#include "asg_sdk_api.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ENABLE_RTCM_V3 1
//typedef struct {        /* RTCM control struct type */
//	int32_t           staid;          /* station id */
//	int32_t           stah;           /* station health */
//	int32_t           seqno;          /* sequence number for rtcm 2 or iods msm */
//	int32_t           outtype;        /* output message type */
//	gtime_t       gpsTime;       /* message time */
//    gtime_t       gpsTimeModify;
//	gtime_t       glnTime;     /* message start time */
//    gtime_t       glnTimeModify;
//    gtime_t       bdsTime;
//    gtime_t       bdsTimeModify;
//	obs_t         obs;          /* observation data (uncorrected) */
//	//nav_t         nav;          /* satellite ephemerides */
//	sta_t         sta;          /* station parameters */
//	dgnss_t       *dgps;       /* output of dgps corrections */
//#if ENABLE_RTCM_V3
//	ssr_t         ssr[MAXSAT];  /* output of ssr corrections */
//#endif
//	char          msg[128];      /* special message */
//	char          msgtype[256];  /* last message type */
//	char          msmtype[6][128]; /* msm signal types */
//	int32_t           obsflag;        /* obs data complete flag (1:ok,0:not complete) */
//	int32_t           ephsat;         /* update satellite of ephemeris */
//	//double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
//	unsigned char lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */
//	unsigned char loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
//	//gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
//	int32_t           nbyte;          /* number of bytes in message buffer */
//	int32_t           nbit;           /* number of bits in word buffer */
//	int32_t           len;            /* message length (bytes) */
//	unsigned char buff[1200]; /* message buffer */
//	uint32_t           word;           /* word buffer for rtcm 2 */
//	uint32_t           nmsg2[100];     /* message count of RTCM 2 (1-99:1-99,0:other) */
//#if ENABLE_RTCM_V3
//	uint32_t nmsg3[300];     /* message count of RTCM 3 (1-299:1001-1299,0:ohter) */
//#endif
//	char          opt[256];      /* RTCM dependent options */
//} rtcm_t;

typedef struct {
    uint8_t      msg_sync;   /* Sync Flag */
    uint16_t     msg_id;     /* msg ID */
    uint16_t     tran_interval; /* Transmission Interval */
} sta_msg_para_t;

typedef struct {
    int32_t     MJD_n;          /* Modified Julian Day (MJD) Number */
    int32_t     utc_sod;        /* Seconds of Day (UTC) */
    int32_t     Nm;             /* No. of Message ID Announcements to Follow (Nm) */
    int32_t     leap_sec;       /* Leap Seconds, GPS-UTC */
    sta_msg_para_t msg_para[MAXMSGNUM];
} sys_para_t;

typedef struct {
	uint16_t phyid;
	uint16_t itrf;
	double rb[3];
} phy_stainfo_t;

typedef struct {
	uint8_t indicator; /* 0: unaligned  1:aligned */
	uint8_t sigmask;   /* bit0:L1CA, bit1:L1P, bit2:L2CA, bit3:L2P */
	float cpb[4]; /* 0:L1CA, 1:L1P, 2:L2CA, 3:L2P */
} gln_cpb_t;

typedef struct {
	rtcm_t* rtcm;
	int           id_type;
	gtime_t       gpsTime;          /* message time */
    gtime_t       gpsTimeModify;
    gtime_t       glnTime;          /* message start time */
    gtime_t       glnTimeModify;
    gtime_t       bdsTime;
    gtime_t       bdsTimeModify;
    dgnss_t       *dgnss;           /* output of dgnss corrections */
    sys_para_t    sys_para;         /* add in FindCM, not mandatory */
	phy_stainfo_t phy_sta;          /* add in FindCM, not mandatory */
	gln_cpb_t     glo_cpb;          /* glonass cpb info from MT1230 */
} ag_rtcm_t;

typedef enum {
   Type_Sat_ARP_1005 = 1005,             /* Stationary Antenna Reference Point, with Height Information */
   Type_Sat_ARP_1006 = 1006,             /* Stationary Antenna Reference Point, with Height Information */
   Type_Ant_Des_1008 = 1008,             /* Antenna Descriptor & Serial Number */
   Type_Sys_Para_1013 = 1013,            /* system parameter */
   Type_Phy_Sta_1032 = 1032,             /* Physical Reference Station Position Information */
   Type_Rcv_Ant_Des_1033 = 1033,         /* Receiver and Antenna Descriptors */
   Type_Gps_Msm4_1074 = 1074,            /* include gps msm4 */
   Type_Glo_Msm4_1084 = 1084,            /* include glo msm4 */
   Type_Gal_Msm4_1094 = 1094,            /* include gal msm4 */
   Type_Bds_Msm4_1124 = 1124,            /* include bds msm4 */
   Type_Gln_CPB_1230 = 1230,             /* include gln cpb */
   Type_Gnss_Msm4_9999 = 9999
} rtcm32_msg_type;

#define RTCM3_MSG_NONE   0x0000
#define RTCM3_MSG_1006   0x0001    /* Stationary Antenna Reference Point, with Height Information */
#define RTCM3_MSG_1008   0x0002    /* Antenna Descriptor & Serial Number */
#define RTCM3_MSG_1013   0x0004    /* system parameter, currently not used */
#define RTCM3_MSG_1032   0x0008    /* Physical Reference Station Position Information, for post-process */
#define RTCM3_MSG_1033   0x0010    /* Receiver and Antenna Descriptors, extension of 1008 */
#define RTCM3_MSG_1074   0x0020    /* GPS     MSM4 */
#define RTCM3_MSG_1084   0x0040    /* GLONASS MSM4 */
#define RTCM3_MSG_1124   0x0080    /* BDS     MSM4 */
#define RTCM3_MSG_1005   0x0100    /* Stationary Antenna Reference Point, with Height Information */
#define RTCM3_MSG_1230   0x0200    /* GLONASS CPB info */
#define RTCM3_MSG_1094   0x0400    /* GALILEO MSM4 */

/* dgnss data decoded from rtcm2 bit buffer */
typedef struct {
    uint32_t cnt;
    dgnss_t data[MAX_MEAS_NUM];
} rtcm2_dgnss_t;      /* rename from OceanA branch DgnssDataPack_t */

/* rtk data decoded from rtcm3 bit buffer */
typedef struct {
    uint64_t        msg_mask;     /* RTCM3_MSG_XXXX | RTCM3_MSG_XXXX | ... */
	/* ref pos + ant info */
    sta_t      sta;          /* station parameters */
    /* ref obs */
    int32_t        obsflag;      /* obs data complete flag (1:ok,0:not complete) */
    int32_t        obsn;         /* number of obervation data/allocated */
    obsd_t     obsd[MAXOBS]; /* observation data records */
    /* sys parameters */
    sys_para_t sys_para;     /* add in FindCM, not mandatory */
	/* physical sta pos */
    phy_stainfo_t phy_sta;   /* add in FindCM, not mandatory */
} rtcm3_rtk_t;

/* ext rtcm data */
typedef struct {
	/* MT 1230 */
	uint8_t         indicator; /* 0: unaligned  1:aligned */
	uint8_t         sigmask;   /* bit0:L1CA, bit1:L1P, bit2:L2CA, bit3:L2P */
	float      cpb[4];    /* 0:L1CA, 1:L1P, 2:L2CA, 3:L2P */
	/* Other MT reserved for future */
	int8_t         reserved1[4];
	uint8_t         reserved2[4];
	int32_t        reserved3[4];
	float      reserved4[4];
	double     reserved5[4];
	uint64_t        msg_mask;     /* RTCM3_MSG_XXXX | RTCM3_MSG_XXXX | ... */
} rtcm3_extinfo_pack_t;

/* data packet trans between RTCM,HSM,RTD thread */
typedef union {
    rtcm2_dgnss_t rtcm2_data;
    rtcm3_rtk_t   rtcm3_data;
} rtcm_data_pack_t;

/* local rtd buffer in PE */
typedef struct
{
    /* rtd data */
	uint8_t            RtdEnterCnt;
	uint8_t            RtdExitCnt;
	uint8_t            RtdUseFlag : 1;
	uint8_t            RtdDataFull : 1;
    gtime_t       time[GNSS_MAX_MODE];             // most recent RTD message type1/type31
    dgnss_t*      dgnss_data[MAX_PRN_ALL_MODE];
} pe_rtd_data_t;     /* rename from OceanA branch Rtd_data_t */

#define PE_RTCM_INFO_REFPOS  0x0001
#define PE_RTCM_INFO_REFOBS  0x0002
#define PE_RTCM_INFO_ANTINFO 0x0004
#define PE_RTCM_INFO_SYSPARA 0x0008
#define PE_RTCM_INFO_PHYSTA  0x0010
#define PE_RTCM_INFO_GLNCPB  0x0020
/* local rtk buffer in PE */
typedef struct
{
  uint32_t           info_mask;          /* mask for rtcm info: PE_RTCM_INFO_REFPOS|PE_RTCM_INFO_REFOBS|... */
  double           rb[2][3];           /* base position(ecef) (m) */
  uint8_t            base_switch;
  int16_t           staid_rb[2];        /* last two base station id */
  int16_t           staid_obs;          /* newest ref obs data */
  int32_t           obsflag[MAXOBSBUF]; /* obs data complete flag (1:ok,0:not complete) */
  obs_t         obs[MAXOBSBUF];     /* ref obs data buffer */
  sys_para_t    sys_para;           /* system parameter, currently not used */
  phy_stainfo_t phy_sta;         /* add in FindCM, not mandatory */
  uint8_t            glocpb_available;/* determine wether use the stored glonass cpb, 0:unavailable  1:available */
  gln_cpb_t     pe_glo_cpb;      /* glonass cpb info from MT1230 */
  uint8_t            ref_freqmask;    /* ref obs data freq mask, bit0:L1 bit1:L2 bit2:L5 */
  uint8_t is_cur_obs_coor_consist;
} pe_rtk_data_t;

typedef union {
    pe_rtd_data_t rtd;
    pe_rtk_data_t rtk;
} Rtcm_data_t;

/* DGNSS data, old version, duplicated. */
typedef struct {
	uint32_t cnt;
	dgnss_t data[MAX_MEAS_NUM];
} DgnssDataPack_t;

/* RTCM APIs */
extern ag_rtcm_t* Gnss_Sys_RTCMCreate();
extern void Gnss_Sys_RTCMFree(ag_rtcm_t* _rtcm);
extern ag_rtcm_t* Gnss_Sys_GetRTCMCb(void);
extern int32_t Gnss_Sys_InputRTCMv2(ag_rtcm_t* rtcm, unsigned char data);
extern int32_t Gnss_Sys_InputRTCMv3(ag_rtcm_t* rtcm, unsigned char data);
extern int32_t Gnss_Sys_InputRTCMv2File(ag_rtcm_t* rtcm, FILE *fp);
extern int32_t Gnss_Sys_InputRTCMv3File(ag_rtcm_t* rtcm, FILE *fp);
extern int32_t asg_is_rtcm_being_reset();
extern void Gnss_Sys_output_rtcm2(ag_rtcm_t* rtcm, int32_t ret_type, rtcm_data_pack_t* rtcm_pack);
extern void Gnss_Sys_output_rtcm3(ag_rtcm_t* rtcm, int32_t ret_type, rtcm_data_pack_t* rtcm_pack);
extern int32_t GnssRtcm2DecodeRTD(char* string, int32_t size, rtcm_data_pack_t* rtcm_pack);
extern int32_t GnssRtcm3DecodeRTK(char* string, int32_t size, rtcm_data_pack_t* rtcm_pack);
extern time_t GnssRtcmGetRecvTime();
extern int32_t gnss_rtcm_start(void *rmt_msg_hdlr);
extern int32_t gnss_rtcm_stop(void *rmt_msg_hdlr);

#ifdef __cplusplus
}
#endif

#endif

