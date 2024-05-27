/*
 * Copyright (c) 2021 Qianxun SI Inc. All rights reserved.
 *
 * Revision: 1.1
 */

#ifndef QXSI_SSR2LOS_TYPE_DEF_H
#define QXSI_SSR2LOS_TYPE_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* observation data records number*/
#define QXSI_OBS_DATA_NUM_LIMIT                        (80)
/* max number of channel for one satellite*/
#define QXSI_OBS_SAT_CHL_NUM_LIMIT                     (15)

/* Satellites broadcast ephemeris, orbit and clock mask */
typedef unsigned char qxsi_ssr_sat_orb_clk_mask;
#define QXSI_SSR_SAT_ORB_CLK_EPH                       (0x01U)    /* broadcast ephemeris pos/vel/clk is valid */
#define QXSI_SSR_SAT_ORB_CLK_ORBIT_CORR                (0x02U)    /* orbit correction is calculated */
#define QXSI_SSR_SAT_ORB_CLK_CLOCK_CORR                (0x04U)    /* clock correction is calculated */

/* Satellites bias mask */
typedef unsigned char qxsi_ssr_sat_bias_mask;
#define QXSI_SSR_SAT_BIAS_CODE_CORR                    (0x01U)    /* code bias correction is calculated */
#define QXSI_SSR_SAT_BIAS_PHASE_CORR                   (0x02U)    /* phase bias correction is calculated */

/* Atmosphere mask */
typedef unsigned char qxsi_ssr_atmo_mask;
#define QXSI_SSR_ATMO_STEC_CORR                        (0x01U)    /* STEC correction is calculated */
#define QXSI_SSR_ATMO_STD_CORR                         (0x02U)    /* STD correction is calculated */
#define QXSI_SSR_ATMO_ZTD_CORR                         (0x04U)    /* ZTD correction is calculated */

/* Error model mask */
typedef unsigned char qxsi_ssr_site_error_model_mask;
#define QXSI_SSR_ERROR_MODEL_SITE_SOLID_TIDE_CORR      (0x01U)    /*solid tide is corrected in orb_clk LOS*/
#define QXSI_SSR_ERROR_MODEL_SITE_OCEAN_TIDE_CORR      (0x02U)    /*ocean tide is corrected in orb_clk LOS*/
#define QXSI_SSR_ERROR_MODEL_SITE_POLE_TIDE_CORR       (0x04U)    /*pole tide is corrected in orb_clk LOS*/

typedef unsigned char qxsi_ssr_sat_error_model_mask;
#define QXSI_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR      (0x01U)    /*gravitaional delay is corrected in orb_clk LOS*/
#define QXSI_SSR_ERROR_MODEL_SAT_BDS_MULTIPATH_CORR    (0x02U)    /*BDS2 satellite pseudorange multipath is corrected in code bias(2I/7I/6I) LOS*/
#define QXSI_SSR_ERROR_MODEL_SAT_WIND_UP_VALID         (0x04U)    /*satellite wind up correction is calculated and valid*/
#define QXSI_SSR_ERROR_MODEL_SAT_YAW_INFO_VALID        (0x08U)    /*satellite attitude Yaw angle and Yaw rate is calculated and valid*/

/* Integrity Flag Define */
typedef unsigned char qxsi_integrity_flag;
#define QXSI_INTEGRITY_FLAG_MONITORED_OK    (0U)
#define QXSI_INTEGRITY_FLAG_MONITORED_FAIL  (1U)
#define QXSI_INTEGRITY_FLAG_NOT_MONITORED   (2U)

/* Logging level */
typedef enum {
    QXSI_LOG_LEVEL_ERROR = 1,    /* error information */
    QXSI_LOG_LEVEL_WARN  = 2,    /* warning information */
    QXSI_LOG_LEVEL_INFO  = 3,    /* contain sensitive information */
    QXSI_LOG_LEVEL_DEBUG = 4     /* all debugging information */
} qxsi_log_level_e;

/* Types of navigation system */
typedef enum {
    QXSI_SAT_SYS_NONE = 0x00,   /* navigation system: none */
    QXSI_SAT_SYS_GPS = 0x01,    /* navigation system: GPS */
    QXSI_SAT_SYS_GAL = 0x08,    /* navigation system: Galileo */
    QXSI_SAT_SYS_BDS = 0x20,    /* navigation system: BeiDou */
} qxsi_sat_sys_e;

/* Types of channel */
typedef enum {
    CHANNEL_IDX_HEAD = 0,
    CHANNEL_IDX_C = 1,          /* channel: C */
    CHANNEL_IDX_S = 2,          /* channel: S */
    CHANNEL_IDX_L = 3,          /* channel: L */
    CHANNEL_IDX_X = 4,          /* channel: X */
    CHANNEL_IDX_P = 5,          /* channel: P */
    CHANNEL_IDX_W = 6,          /* channel: W */
    CHANNEL_IDX_Y = 7,          /* channel: Y */
    CHANNEL_IDX_M = 8,          /* channel: M */
    CHANNEL_IDX_N = 9,          /* channel: N */
    CHANNEL_IDX_D = 10,         /* channel: D */
    CHANNEL_IDX_I = 11,         /* channel: I */
    CHANNEL_IDX_Q = 12,         /* channel: Q */
    CHANNEL_IDX_A = 13,         /* channel: A */
    CHANNEL_IDX_B = 14,         /* channel: B */
    CHANNEL_IDX_Z = 15,         /* channel: Z */
    CHANNEL_IDX_E = 16,         /* channel: E */
    CHANNEL_IDX_TAIL = 17,
} qxsi_channel_idx_e;

/* Time based on GPS week and week seconds */
typedef struct {
    int week_num;               /* week num since GPS time start point */
    double sec_of_week;         /* seconds of the week */
} qxsi_gps_time_t;

typedef struct {
    double x; /* meter */
    double y; /* meter */
    double z; /* meter */
} qxsi_coor_xyz_t;

/* Struct of velocity */
typedef struct {
    double x; /* meter/s */
    double y; /* meter/s */
    double z; /* meter/s */
} qxsi_vel_xyz_t;

/* Struct of channel */
typedef struct {
    qxsi_channel_idx_e tag;             /* channel tag */
    int frq;                            /* frequency number */
} qxsi_channel_t;

/* Observation data */
typedef struct {
    double L;                           /* observation data carrier-phase (cycle) */
    double P;                           /* observation data pseudorange (m) */
    double D;                           /* observation data doppler frequency (Hz) */
    int snr;                            /* Signal to Noise Ratio */
    int lli;                            /* lost lock of indicator */
    int signal_strength;                /* signal strength */
} qxsi_obs_data_t;

/* Satellite id */
typedef struct {
    qxsi_sat_sys_e sys;                 /* satellite system */
    int prn;                            /* satellite index of the system */
} qxsi_sat_id_t;

/* Observation data of one channel */
typedef struct {
    qxsi_channel_t channel;             /* channel */
    qxsi_obs_data_t data;               /* observation data of the channel */
} qxsi_channel_obs_data_t;

/* Observation data of one satellite */
typedef struct {
    qxsi_sat_id_t id;                   /* satellite id */
    int num;                            /* channel count of one satellite */
    qxsi_channel_obs_data_t chl_data[QXSI_OBS_SAT_CHL_NUM_LIMIT];  /* all channel data */
} qxsi_sat_meas_data_t;

/* Observation data of one epoch */
typedef struct {
    qxsi_gps_time_t time;               /* the GPS time of the epoch */
    int num;                            /* satellite count */
    qxsi_sat_meas_data_t sat_data[QXSI_OBS_DATA_NUM_LIMIT];      /* observation data of the epoch */
} qxsi_epoch_meas_data_t;

/* Station position */
typedef struct {
    qxsi_gps_time_t time;
    qxsi_coor_xyz_t pos;                /* rover or reference station position */
    qxsi_vel_xyz_t vel;                 /* velocity of the station */
} qxsi_sta_pos_t;

/* Navigation data */
typedef struct {
    int prn;
    double iode;                        /* issue of data(Ephemeris) */
    double iodc;                        /* issue of data clock */
    int sva;                            /* satellite accuracy(Index) */
    int svh;                            /* satellite health */
    int code;                           /* GPS: code on L2 channel */
                                        /* GAL: data source defined as rinex 3.03 */
    int flag;                           /* L2P data flag */
    qxsi_gps_time_t toe;                /* orbit reference time */
    qxsi_gps_time_t toc;                /* satellite clock reference time */
    qxsi_gps_time_t ttr;                /* Transmission time of message */
    double sqrt_A;                      /* Square Root of the Semi-Major Axis */
    double e;                           /* Eccentricity */
    double i0;                          /* Inclination Angle at Reference Time(radians) */
    double OMG0;                        /* Longitude of Ascending Node of Orbit Plane at Weekly Epoch(radians) */
    double omg;                         /* Argument of Perigee(radians) */
    double M0;                          /* Mean Anomaly at Reference Time(radians) */
    double deln;                        /* Mean Motion Difference from Computed Value(radians/s) */
    double OMGd;                        /* Rate of Right Ascension */
    double idot;                        /* Rate of change of inclination(radians/s) */
    double crc;                         /* Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius(meters) */
    double crs;                         /* Amplitude of the Sine Correction Term to the Orbit Radius(meters) */
    double cuc;                         /* Amplitude of Cosine Harmonic Correction Term to the Argument of Latitude(radians) */
    double cus;                         /* Amplitude of Sine Harmonic Correction Term to the Argument of Latitude(radians) */
    double cic;                         /* Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination(radians) */
    double cis;                         /* Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination(radians) */
    double fit;                         /* Fit interval(hours) */
    double f0;                          /* clock bias(second) */
    double f1;                          /* clock drift(second/s) */
    double f2;                          /* clock drift rate(second/s2) */
    double tgd[4];                      /* TGD(seconds) group delay parametes */
                                        /* GPS: tgd[0]=TGD */
                                        /* GAL: tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                                        /* BDS: tgd[0]=BGD1,tgd[1]=BGD2 */
} qxsi_nav_t;

/* Struct of broadcast ephemeris for GPS system */
typedef struct {
    qxsi_nav_t data;
} qxsi_gps_nav_t;

/* Struct of broadcast ephemeris for BDS system */
typedef struct {
    qxsi_nav_t data;
} qxsi_bds_nav_t;

/* Struct of broadcast ephemeris for GAL system */
typedef struct {
    qxsi_nav_t data;
} qxsi_gal_nav_t;

/* Correction data with quality indicator and integrity */
typedef struct {
    qxsi_integrity_flag pre_itg;        /* pre-check integrity */
    qxsi_integrity_flag post_itg;       /* post-check integrity */
    /* QI unit for different SSR corrections refer to following:
     * orb_clk: mm
     * code bias/phase bias/ZTD/STD: m
     * STEC: TECU
     */
    float qi;                           /* quality indicator */
    double corr;                        /* correction data value */
} qxsi_corr_data_t;

/* ZTD Correction data with quality indicator and integrity */
typedef struct {
    qxsi_integrity_flag pre_itg;        /* pre-check integrity */
    qxsi_integrity_flag post_itg;       /* post-check integrity */
    float qi;                           /* quality indicator, unit: m */
    double wet_corr;                    /* wet part correction data value */
    double dry_corr;                    /* dry part correction data value */
} qxsi_ztd_corr_data_t;

/* Correction data of one signal */
typedef struct {
    qxsi_channel_t signal_type;         /* satallite signal type */
    qxsi_corr_data_t P;                 /* satallite code correction data */
    qxsi_corr_data_t L;                 /* satallite phase correction data */
    qxsi_ssr_sat_bias_mask bias_mask;   /* flag of code or phase corrected indicator */
    unsigned char phase_bias_continue_flag;/* flag of phase bias continued indicator */
} qxsi_signal_corr_t;

/* Satellite information */
typedef struct {
    int iode;                           /* ephemeris iode */
    qxsi_coor_xyz_t pos;                /* ephemeris postion of satellite */
    qxsi_vel_xyz_t vel;                 /* ephemeris velocity of satellite */
    double clock;                       /* ephemeris clock of satellite */
    double clock_drift;                 /* ephemeris clock drift of satellite */
} qxsi_sat_brdc_info_t;

/* Line of sight data of one satellite */
typedef struct {
    qxsi_sat_id_t id;                   /* satellite ID */
    qxsi_ssr_sat_orb_clk_mask orb_clk_mask;/* flag of satellite orbit or clock corrected indicator */
    qxsi_ssr_atmo_mask atmo_mask;       /* flag of atmosphere corrected indicator */
    qxsi_ssr_sat_error_model_mask sat_error_model_mask;/* flag of error model indicator for satellite*/
    unsigned char signal_num;           /* signal number of one satellite */

    qxsi_sat_brdc_info_t brdc_info;     /* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
    qxsi_corr_data_t orb_clk;           /* correction for the orbit/clock SSR data */
    qxsi_gps_time_t orb_clk_time;       /* correction data time for the orbit/clock SSR data */

    qxsi_corr_data_t stec;              /* correction for the STEC SSR data */
    qxsi_corr_data_t std;               /* correction for the STD SSR data */
    double windup;                      /* correction of satellite wind up */
    float yaw;                          /* yaw angle of satellite, unit in degree */
    float yaw_rate;                     /* yaw rate of satellite, unit in degree per second */
    qxsi_signal_corr_t signal_bias[QXSI_OBS_SAT_CHL_NUM_LIMIT];    /* all signal data for one satellite */
} qxsi_sat_los_t;

/* Line of sight data of station */
typedef struct {
    qxsi_gps_time_t obs_time;           /* observation time */
    qxsi_gps_time_t stec_time;          /* STEC correction data time */
    qxsi_gps_time_t std_time;           /* STD correction data time */
    qxsi_gps_time_t ztd_corr_time;      /* ZTD correction data time */
    qxsi_ssr_atmo_mask ztd_mask;        /* flag of ZTD corrected indicator */
    qxsi_ssr_site_error_model_mask site_error_model_mask;/* flag of error model indicator for site*/
    unsigned char sat_num;              /* satellite number of current epoch */
    qxsi_ztd_corr_data_t ztd;           /* correction for the ZTD SSR data */
    qxsi_sat_los_t sat_los_corr[QXSI_OBS_DATA_NUM_LIMIT];       /* all satellite correction data for current epoch */
} qxsi_station_los_t;

/* Station observation and position */
typedef struct {
    qxsi_epoch_meas_data_t obs;         /* current observation data */
    qxsi_coor_xyz_t pos;                /* station init position of current epoch */
} qxsi_station_info_t;

#ifdef __cplusplus
}
#endif

#endif //QXSI_SSR2LOS_TYPE_DEF_H
