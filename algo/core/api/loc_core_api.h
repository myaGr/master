/**@file        loc_core_api.h
 * @brief       Location engine core api header file
 * @version     V0.3
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * <tr><td>2022/12/28  <td>0.2      <td>caizhijie   <td>Add INS Interface
 * <tr><td>2023/05/19  <td>0.3      <td>caizhijie   <td>Add Tight INS Interface
 * </table>
 *
 **********************************************************************************
 */

#ifndef __LOC_CORE_API_H__
#define __LOC_CORE_API_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Integrity flags */
typedef enum
{
  LOC_API_INTEGRITY_FLAG_MONITORED_OK = 0,
  LOC_API_INTEGRITY_FLAG_MONITORED_FAIL,
  LOC_API_INTEGRITY_FLAG_NOT_MONITORED,
} loc_api_integrityFlagVal;
typedef uint8_t loc_api_integrityFlag;

/* protection level */
typedef struct
{
  loc_api_integrityFlagVal  u_flag;                      /* integrity flag */
  float                     f_protection_level;          /* protection level, m */
}loc_api_protectionLevel_t;

/* integrity  */
typedef struct
{
  loc_api_protectionLevel_t       z_pos_longitudinal_pl;   /* Position longitudinal */
  loc_api_protectionLevel_t       z_pos_lateral_pl;        /* Position lateral  */
  loc_api_protectionLevel_t       z_pos_north_pl;          /* Position North */
  loc_api_protectionLevel_t       z_pos_east_pl;           /* Position East */
  loc_api_protectionLevel_t       z_pos_hor_pl;            /* Position Horizontal */
  loc_api_protectionLevel_t       z_pos_ver_pl;            /* Position Vertical */
  loc_api_protectionLevel_t       z_vel_longitudinal_pl;   /* Velocity longitudinal */
  loc_api_protectionLevel_t       z_vel_lateral_pl;        /* Velocity lateral  */
  loc_api_protectionLevel_t       z_vel_north_pl;          /* Velocity North */
  loc_api_protectionLevel_t       z_vel_east_pl;           /* Velocity East */
  loc_api_protectionLevel_t       z_vel_hor_pl;            /* Velocity Horizontal */
  loc_api_protectionLevel_t       z_vel_ver_pl;            /* Velocity Vertical */
  loc_api_protectionLevel_t       z_roll_pl;               /* Roll */
  loc_api_protectionLevel_t       z_pitch_pl;              /* Pitch */
  loc_api_protectionLevel_t       z_yaw_pl;                /* Yaw */
}loc_api_integrity_t;

/* SSR Service Source */
typedef enum
{
  GNSS_SSR_SERVICE_SOURCE_NONE        = 0x00,
  GNSS_SSR_SERVICE_SOURCE_QX          = 0x01,
  GNSS_SSR_SERVICE_SOURCE_Gee         = 0x02,
  GNSS_SSR_SERVICE_SOURCE_AG_SRR_LOS  = 0x03
} gnss_ssrServiceSourceVal;
typedef uint8_t gnss_ssrServiceSource;

/** the frequency type of GNSS measure feedback INS */
typedef enum {
  LOC_API_GNSS_FREQ_TYPE_L1 = 0x0,
  LOC_API_GNSS_FREQ_TYPE_L2,
  LOC_API_GNSS_FREQ_TYPE_L5,
  LOC_API_GNSS_FREQ_TYPE_INVALID
} loc_api_gnssFreqTypeVal;
typedef uint8_t loc_api_gnssFreqType;

typedef enum
{
  LOC_API_GNSS_NONE = 0,
  LOC_API_GNSS_GPS = 1,
  LOC_API_GNSS_GLO = 2,
  LOC_API_GNSS_BDS = 3,
  LOC_API_GNSS_GAL = 4,
  LOC_API_GNSS_QZS = 5,
  LOC_API_GNSS_NAVIC = 6,
  LOC_API_GNSS_MAX = 7
} loc_api_gnssConstellationTypeVal;
typedef uint8_t loc_api_gnssConstellationType;

/** The fix type of loc engine */
typedef enum {
  LOC_API_FIX_FLAG_INVALID = 0,
  LOC_API_FIX_FLAG_SPS,
  LOC_API_FIX_FLAG_DGNSS,
  LOC_API_FIX_FLAG_PPS,
  LOC_API_FIX_FLAG_FIXED,
  LOC_API_FIX_FLAG_FLOATING,
  LOC_API_FIX_FLAG_DR,
  LOC_API_FIX_FLAG_MANUAL,
  LOC_API_FIX_FLAG_SIMULATOR
} loc_api_FixFlagTypeEnumVal;
typedef uint8_t loc_api_FixFlagType;

typedef enum
{ 
  LOC_API_FIX_SOURCE_NONE = 0,
  LOC_API_FIX_SOURCE_PVT  = 1,
  LOC_API_FIX_SOURCE_RTK  = 2,
  LOC_API_FIX_SOURCE_PPP  = 3,
  LOC_API_FIX_SOURCE_INS  = 4,
  LOC_API_FIX_SOURCE_MAX
} loc_api_FixSourceTypeEnumVal;
typedef uint8_t loc_api_FixSourceType;

/** Location Engine Memory type */
typedef uint8_t loc_api_MemoryRegisterType;
#define LOC_API_MEMORY_NONE                   (uint8_t)0
#define LOC_API_MEMORY_OS                     (uint8_t)1
#define LOC_API_MEMORY_EXT_POOL               (uint8_t)2
#define LOC_API_MEMORY_EXT_API                (uint8_t)3

#define LOC_API_MEMORY_POOL_COUNT_MAX         (uint8_t)8

#define LOC_API_MAX_GNSS_TRK_MEAS_NUMBER      (uint8_t)120

/** Location Engine Memory Register Structure */
typedef struct {
  loc_api_MemoryRegisterType u_type;
  /** External memory alloc callback, if unavaliable, set NULL */
  void* (*alloc)(uint32_t sz, const char* func, uint32_t line, char* file);
  /** External memory free callback, if unavaliable, set NULL */
  void (*free)(void* ptr);
  uint32_t pool_count;
  uint8_t* pool_addr[LOC_API_MEMORY_POOL_COUNT_MAX];
  uint32_t pool_size[LOC_API_MEMORY_POOL_COUNT_MAX];
} loc_api_MemoryRegister_t;

typedef struct
{
  uint64_t utc_timestamp; /* ms */
  uint16_t utc_year;
  uint16_t utc_month;
  uint16_t utc_day;
  uint16_t utc_hour;
  uint16_t utc_minute;
  double utc_second;

  uint16_t week; /* GPS week number */
  double tow; /* time of week */
  uint8_t leapsec; /* leap second */

  uint8_t fix_quality; /* 0/1/2/4/5 */

  double lon; /* Longitude (deg) */
  double lat; /* Latitude (deg) */
  float alt;

  float lonstd; /* unit: m */
  float latstd; /* unit: m */
  float altstd; /* unit: m */
  float   lon_protection_level;    /* longitude protection level, unit: m */
  float   lat_protection_level;    /* latitude  protection level, unit: m */
  float   alt_protection_level;    /* altitude  protection level, unit: m */
  uint8_t lon_integrity_flag;      /* longitude integrity flag, see loc_api_integrityFlag*/
  uint8_t lat_integrity_flag;      /* latitude  integrity flag, see loc_api_integrityFlag*/
  uint8_t alt_integrity_flag;      /* altitude  integrity flag, see loc_api_integrityFlag*/

  float vel_n; /* NED north velocity (m/s) */
  float vel_e; /* NED east velocity (m/s) */
  float vel_d; /* NED down velocity (m/s) */

  float vel_n_std; /* unit: m/s */
  float vel_e_std; /* unit: m/s */
  float vel_d_std; /* unit: m/s */

  float age; /* diff age (s) */

  uint16_t sv_used;
  uint16_t sv_trked;
  uint16_t sv_fixed;

  float gdop;
  float pdop;
  float hdop;
  float vdop;
  float tdop;

  double quasi_geoid_h;//add new field to represent quasi-geoid height
  float avg_CN0;//add new field to represent average CN0
  uint8_t CN040;
  uint8_t dcp_pos_type;
  float track_angle;/* GNSS course, trackTrue, bearing...,unit in degree */
  float track_angle_std;/*unit in degree*/
} loc_api_location_report_t;

typedef struct
{
  uint64_t t_utcTimeStamp; /* ms */
  uint16_t w_utcYear;
  uint16_t w_utcMonth;
  uint16_t w_utcDay;
  uint16_t w_utcHour;
  uint16_t w_utcMinute;
  double   d_utcSecond;

  uint16_t w_week;                 /* GPS week number */
  double   d_tow;                  /* time of week */
  uint8_t  u_leapsec;              /* leap second */

  uint8_t  u_ortFixQuality;           /* 0/1/2/4/5 */
  uint8_t  u_SvTrackCount;         /*sv track count*/
  uint8_t  u_SvTrackCountEleThres; /*sv in track count that elevation greater than threshold*/
  uint8_t  u_SvInUseCount;         /*sv in used count*/
  uint8_t  u_SvInUseCountMuliFreq; /*sv in used count that had dual or multi frequency*/
  uint8_t  u_ortSolStatus;         /*the solution status of GNSS orientation*/
  uint8_t  u_ortPosVelType;        /*the position or velocity type of GNSS orientation*/
  uint8_t  u_GPSsignalUsedMask;    /*the mask of GPS used in solution*/
  uint8_t  u_GLOsignalUsedMask;    /*the mask of GLONASS used in solution*/
  uint8_t  u_GalSignalUsedMask;    /*the mask of Galileo used in solution*/
  uint8_t  u_BDSsignalUsedMask;    /*the mask of BDS used in solution*/
  float    f_age;                  /*different age (s)*/
  float    f_heading;              /*Heading value,unit in degree,from 0 to 360*/
  float    f_pitch;                /*Pitch value,unit in degree,from -90 to 90*/
  float    f_roll;                 /*Roll value,unit in degree,from -90 to 90*/
  float    f_headingStd;           /*the STD of heading,unit in degree*/
  float    f_pitchStd;             /*the STD of pitch, unit in degree*/
  float    f_rollStd;              /*the STD of roll, unit in degree*/
  float    f_ENU[3];               /*the coordinate of ENU direction based at base station,the length can be calculated by the format:length=sqrt(f_ENU[0]*f_ENU[0]+f_ENU[1]*f_ENU[1]+f_ENU[2]*f_ENU[2])*/
  double   d_auxiAntXyz[3];        /* Position: X,Y,Z in ECEF (m) of auxiliary antenna*/
  double   d_auxiAntLLA[3];        /* Position: lat, lon, alt of auxiliary antenna*/
  float    f_auxiAntVelXyz[3];     /* Velocity: X,Y,Z in ECEF (m/s) of auxiliary antenna*/
  float    f_auxiAntVelEnu[3];     /* Velocity: lat, lon, alt of auxiliary antenna*/

  uint8_t  u_posFixQuality;        /* 0/1/2/4/5 */
  uint8_t  u_mainAntSvInUseCount;  /* sv in used count */
  double   d_mainAntXyz[3];        /* Position: X,Y,Z in ECEF (m) */
  double   d_mainAntLLA[3];        /* Position: lat, lon, alt */
  float    f_mainAntVelXyz[3];     /* Velocity: X,Y,Z in ECEF (m/s) */
  float    f_mainAntVelEnu[3];     /* Velocity: lat, lon, alt */
  double   pd_refStationCoordinate[3];/* Station Coordinate of VRS */
}loc_api_orient_report_t;

typedef struct
{
  loc_api_gnssConstellationType u_constellation;/** see loc_api_gnssConstellationType */
  uint8_t       u_svid;         /** satellite index of the system */
  uint8_t       u_signal;       /** see gnss_SignalType */
  float         f_cn0;
  float         f_elevation;    /** degree */
  float         f_azimuth;      /** degree */
} loc_api_SvInfo_t;

typedef struct
{
  uint8_t      u_MeasTrackCount;  // measurement track count
  uint8_t      u_MeasInUseCount;  // measurement in use count 
  uint8_t      u_SvTrackCount;    // sv track count
  uint8_t      u_SvInUseCount;    // sv in used count
  uint64_t     t_SvTrackMask[LOC_API_GNSS_MAX];
  uint64_t     t_SvInUseMask[LOC_API_GNSS_MAX];
  loc_api_SvInfo_t z_SV[LOC_API_MAX_GNSS_TRK_MEAS_NUMBER];
} loc_api_SvStatus_t;

#define VERSION_CONSOILDATE_POSITION_FIX  (6)
typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  uint32_t      q_towMsec;           /* MilliSeconds of the week, unit: 1ms */
  uint16_t      w_week;              /* Week Number since GPS time start point */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: loc_api_FixSourceType */
  uint8_t       u_fixFlag;           /* see: loc_api_FixSourceType */
  uint8_t       u_svUsed;
  float         d_avgCN0;            /* Average CN0 */
  double        d_quasiGeoidHeight;  /* Quasi-geoid height */
  double        d_xyz[3];            /* Position: X,Y,Z in ECEF (m) */
  double        d_lla[3];            /* Position: lat, lon, alt */
  float         f_velXyz[3];         /* Velocity: X,Y,Z in ECEF (m/s) */
  float         f_velEnu[3];         /* Velocity: lat, lon, alt */
  float         f_posXyzUnc[3];      /* Uncertainy position XYZ uncertainy (m) */
  float         f_posLlaUnc[3];      /* Uncertainy position LLA  (m) */
  float         f_velEnuUnc[3];      /* Uncertainy velocity : ENU  (m/s) */
  double        d_clockBias;         /* Primary constellation clock bias */
  double        d_clockDrift;        /* Primary constellation clock drift */
  float         f_gdop;
  float         f_pdop;
  float         f_hdop;
  float         f_vdop;
  float         f_tdop;
  uint32_t      q_StationID;
  float         f_age;               /* different age (s) */
  double        pd_StationCoordinate[3];  /* Station Coordinate of VRS */
  uint8_t       u_drposflag;
  float         f_roll;		           /* degrees */
  float         f_pitch;		         /* degrees */
  float         f_heading;	         /* degrees */
  float         f_gyrobias_x; 	     /* degrees/s */
  float         f_gyrobias_y;        /* degrees/s */
  float         f_gyrobias_z;        /* degrees/s */
  float         f_accbias_x;         /* meters/sec2 */
  float         f_accbias_y;         /* meters/sec2 */
  float         f_accbias_z;         /* meters/sec2 */
  float         f_roll_std;          /* degrees */
  float         f_pitch_std;         /* degrees */
  float         f_yaw_std;           /* degrees */
  float         f_mis_roll;          /* degrees */
  float         f_mis_pitch;         /* degrees */
  float         f_mis_yaw;           /* degrees */
  float         f_speed;             /* meters/sec */
  loc_api_SvStatus_t z_SvStatus;
  float         f_whlspd_sf[4];
  uint32_t      q_kfmeastype;
  float         f_la_imu2gnss[3];
  float         f_la_imu2rearmid[3];
  float         f_la_rear2rear;
  float         f_misdualant_roll;     /* deg */
  float         f_misdualant_pitch;    /* deg */
  float         f_misdualant_yaw;      /* deg */
  loc_api_integrity_t z_integity;      /* integrity */
} loc_api_ConsoildatedPositionFix_t;

/** The type of GNSS measurement that feedback to INS */
typedef enum {
  LOC_API_FEEDBACK_NONE         = 0x00,
  LOC_API_FEEDBACK_TIME_DIFF_CP = 0x01,
  LOC_API_FEEDBACK_TIME_DIFF_PR = 0x02,
  LOC_API_FEEDBACK_TIME_DIFF_DR = 0x04,
  LOC_API_FEEDBACK_PR_DOPPLER   = 0x08,
} loc_api_FeedbackInsMeasTypeVal;
typedef uint8_t loc_api_FeedbackInsMeasType;

/* the maximum satellite number of GNSS feedback to INS*/
#define MAX_LOC_API_FEEDBACK_INS_MEAS_NUM        (40)

/* The measurement unit type of GNSS feedback to INS */
typedef struct
{
  /** Measurement constellation, see loc_api_gnssConstellationType */
  loc_api_gnssConstellationType u_constellation;
  /** Measurement satellite index of the iteself system, range [0:N] */
  uint8_t                u_svid;
  /** Measurement CN0 */
  uint8_t                u_cn0;
  /** Observation valid flag */
  uint8_t                u_obs_valid;
  /** The elevation of satellite, uint in rad */
  float                  f_elevation;
  /** The azimuth of satellite, uint in rad */
  float                  f_azimuth;
  /** The unit vector from satellite to site direction */
  float                  f_unit_dir_vect[3];
  /** The satellite position of current epoch */
  double                 d_sat_pos[3];
  /** The satellite velocity of current epoch */
  double                 d_sat_vel[3];
  /** The satellite position of previous epoch */
  double                 d_pre_sat_pos[3];
  /** The variance of pseudo-range observation of current epoch */
  double                 d_pr_var;
  /** The variance of pseudo-range observation of current epoch */
  double                 d_dr_var;
  /** The observation and its residual of time difference epoch.
    If u_FeedbackMeasType = LOC_API_FEEDBACK_PR_DOPPLER, difference obs and res are invalid.
    If u_FeedbackMeasType = LOC_API_FEEDBACK_TIME_DIFF_PR, difference obs and res are from pseudo range.
    If u_FeedbackMeasType = LOC_API_FEEDBACK_TIME_DIFF_DR, difference obs and res are from doppler.
    If u_FeedbackMeasType = LOC_API_FEEDBACK_TIME_DIFF_CP, difference obs and res are from carrier phase.
  */
  float                  f_epoch_diff_obs;
  float                  f_epoch_diff_res;
  /** The observation of current epoch.
    If u_FeedbackMeasType = LOC_API_FEEDBACK_PR_DOPPLER, pseudorange and doppler are valid.
    If u_FeedbackMeasType = LOC_API_FEEDBACK_TIME_DIFF_XX, pseudorange and doppler are invalid.
  */
  double                 d_pseudorange;
  float                  f_doppler;
} loc_api_GnssFeedbackInsMeasUnit_t;

typedef struct
{
  uint32_t  q_GpsTowMsec;
  uint16_t  w_GpsWeek;
  uint8_t   u_CurFixFlag; /* Current Epoch Fix Flag, see: loc_api_FixFlagType */
  uint8_t   u_PreFixFlag; /* Previous Epoch Fix Flag, see: loc_api_FixFlagType */
  uint8_t   u_FreqType;   /* Frequecy Type see: loc_api_gnssFreqType */
  double    d_CurLLA[3];  /* Current Epoch Fix Position in LLA */
  double    d_PreLLA[3];  /* Previous Epoch Fix Position in LLA */
  float     f_QuasiGeoidHeight; /* Quasi-geoid height */
  float     f_interval;   /* The interval between current epoch and previous epoch*/
  uint8_t   u_FeedbackMeasType; /** the type of GNSS measurment, see: gnss_FeedbackInsMeasType */
  uint8_t   u_MeasCount;  /* The number of epoch-difference observations data */

  /* The satellite observation blcok that GNSS feedback to INS */
  loc_api_GnssFeedbackInsMeasUnit_t z_GnssFeedbackInsMeasUnit[MAX_LOC_API_FEEDBACK_INS_MEAS_NUM];
} loc_api_GnssFeedbackInsMeasBlock_t;

/** GPS Legacy Navigation ephemeris (L-NAV) L1/L2 50 bps PRN 1~32 */
typedef struct
{
  uint8_t   svid;     /** Satellite Number */
  uint16_t  week;     /** Gps week number */
  uint8_t   accuracy; /** SV accuracy (URA index) (meter) */
  uint8_t   health;   /** SV health (0:ok) */
  uint16_t  iodc;     /** Issue of Data, Clock 10bit */
  uint8_t   code;     /** GPS: code on L2,00:Res, 01: P code, 10: C/A code 11: L2C */
  uint8_t   fit;      /** fit interval 0:(4h) 1:(>4h)*/
  uint8_t   iode;     /** Issue of Data, Ephemeris 8bit */
  double    tgd;      /** Time Group Delay Differential  8bit scale:2e-31 */
  double    toc;      /** Time of clock 16bit scale:2e4 (second) */
  double    toe;      /** Time of Ephemeris 16bit scale:2e4 (second) */
  double    sqrt_A;   /** Square Root of the Semi-Major Axis */
  double    e;        /** Eccentricity */
  double    M0;       /** Mean Anomaly at Reference Time */
  double    Omega0;   /** Longitude of Ascending Node of Orbit Plane at Weekly Epoch */
  double    Omega;    /** Argument of Perigee */
  double    OmegaDot; /** Rate of Right Ascension */
  double    i0;       /** Inclination Angle at Reference Time */
  double    idot;     /** Rate of Inclination Angle 14bit scale:2e-43 */
  double    DeltaN;   /** Mean Motion difference from computed value at reference time */
  double    crc;      /** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius */
  double    crs;      /** Amplitude of the Sine Correction Term to the Orbit Radius */
  double    cuc;      /** Amplitude of Cosine Harmonic Correction Term to the Argument of Latitude */
  double    cus;      /** Amplitude of Sine Harmonic Correction Term to the Argument of Latitude */
  double    cic;      /** Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination  */
  double    cis;      /** Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination  */
  double    af0;      /** SV Clock Bias Correction Coefficient */
  double    af1;      /** SV Clock Drift Correction Coefficient */
  double    af2;      /** Drift Rate Correction Coefficient */
} loc_api_GpsEphemeris_t;

typedef struct
{
  uint8_t   svid;     /** Satellite Number */
  /* TBD */
} loc_api_GloEphemeris_t;

/** BDS MEO/IGSO B1I D1 Navigation 50 bps, GEO B1I D2 Navigation 500 bps */
typedef struct
{
  uint8_t   svid;     /** Satellite Number */
  uint16_t  week;     /** Gps week number */
  uint8_t   accuracy; /** SV accuracy (URA index) (meter) */
  uint8_t   health;   /** SV health (0:ok) */
  uint16_t  iodc;     /** Issue of Data, Clock 10bit */
  uint8_t   code;     /** BDS: data source */
  uint8_t   fit;      /** fit interval 0:(4h) 1:(>4h)*/
  uint8_t   iode;     /** Issue of Data, Ephemeris 8bit */
  double    tgdB1I;   /** B1I Time Group Delay Differential  8bit scale:2e-31 */
  double    tgdB2I;   /** B2I Time Group Delay Differential  8bit scale:2e-31 */
  double    toc;      /** Time of clock 16bit scale:2e4 (second) */
  double    toe;      /** Time of Ephemeris 16bit scale:2e4 (second) */
  double    sqrt_A;   /** Square Root of the Semi-Major Axis */
  double    e;        /** Eccentricity */
  double    M0;       /** Mean Anomaly at Reference Time */
  double    Omega0;   /** Longitude of Ascending Node of Orbit Plane at Weekly Epoch */
  double    Omega;    /** Argument of Perigee */
  double    OmegaDot; /** Rate of Right Ascension */
  double    i0;       /** Inclination Angle at Reference Time */
  double    idot;     /** Rate of Inclination Angle 14bit scale:2e-43 */
  double    DeltaN;   /** Mean Motion difference from computed value at reference time */
  double    crc;      /** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius */
  double    crs;      /** Amplitude of the Sine Correction Term to the Orbit Radius */
  double    cuc;      /** Amplitude of Cosine Harmonic Correction Term to the Argument of Latitude */
  double    cus;      /** Amplitude of Sine Harmonic Correction Term to the Argument of Latitude */
  double    cic;      /** Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination  */
  double    cis;      /** Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination  */
  double    af0;      /** SV Clock Bias Correction Coefficient */
  double    af1;      /** SV Clock Drift Correction Coefficient */
  double    af2;      /** Drift Rate Correction Coefficient */
} loc_api_BdsEphemeris_t;

/** GAL I/NAV ephemeris E1,E5b */
typedef struct
{
  uint8_t   svid;     /** Satellite Number */
  uint16_t  week;     /** Gal week number */
  uint8_t   accuracy; /** SV accuracy (URA index) (meter) */
  uint8_t   E1health; /** SV health (0:ok) */
  uint8_t   E1DVS;    /** SV Data Validity Status (0:ok) */
  uint8_t   E5bhealth;/** SV health (0:ok) */
  uint8_t   E5bDVS;   /** SV Data Validity Status (0:ok) */
  uint16_t  iodc;     /** Issue of Data, Clock 10bit */
  uint8_t   code;     /** Gal: data source */
  uint8_t   fit;      /** fit interval 0:(4h) 1:(>4h)*/
  uint8_t   iode;     /** Issue of Data, Ephemeris 8bit */
  double    tgdE1E5a; /** Time Group Delay Differential  8bit scale:2e-31 */
  double    tgdE1E5b; /** Time Group Delay Differential  8bit scale:2e-31 */
  double    toc;      /** Time of clock 16bit scale:2e4 (second) */
  double    toe;      /** Time of Ephemeris 16bit scale:2e4 (second) */
  double    sqrt_A;   /** Square Root of the Semi-Major Axis */
  double    e;        /** Eccentricity */
  double    M0;       /** Mean Anomaly at Reference Time */
  double    Omega0;   /** Longitude of Ascending Node of Orbit Plane at Weekly Epoch */
  double    Omega;    /** Argument of Perigee */
  double    OmegaDot; /** Rate of Right Ascension */
  double    i0;       /** Inclination Angle at Reference Time */
  double    idot;     /** Rate of Inclination Angle 14bit scale:2e-43 */
  double    DeltaN;   /** Mean Motion difference from computed value at reference time */
  double    crc;      /** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius */
  double    crs;      /** Amplitude of the Sine Correction Term to the Orbit Radius */
  double    cuc;      /** Amplitude of Cosine Harmonic Correction Term to the Argument of Latitude */
  double    cus;      /** Amplitude of Sine Harmonic Correction Term to the Argument of Latitude */
  double    cic;      /** Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination  */
  double    cis;      /** Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination  */
  double    af0;      /** SV Clock Bias Correction Coefficient */
  double    af1;      /** SV Clock Drift Correction Coefficient */
  double    af2;      /** Drift Rate Correction Coefficient */
} loc_api_GalEphemeris_t;

#define VERSION_GNSS_EPHEMERIS  (0)
typedef struct
{
  uint8_t          u_version;
  union
  {
    loc_api_GpsEphemeris_t z_gpsEph;
    loc_api_GloEphemeris_t z_gloEph;
    loc_api_BdsEphemeris_t z_bdsEph;
    loc_api_GalEphemeris_t z_galEph;
    loc_api_GpsEphemeris_t z_qzsEph;
  } eph;
  loc_api_gnssConstellationType u_constellation;  /* see:loc_api_gnssConstellationType */
  uint32_t  q_towMsecOfToe;   /* ephemeris tow and week */
  uint16_t  w_weekOfToe;
  uint32_t  q_towMsecOfToc;   /* clock tow and week */
  uint16_t  w_weekOfToc;
} loc_api_GnssEphemeris_t;

typedef enum
{
  LOC_API_GNSS_SIG_GPS_L1C = 0,
  LOC_API_GNSS_SIG_GPS_L2C,
  LOC_API_GNSS_SIG_GPS_L5Q,
  LOC_API_GNSS_SIG_GLO_G1,
  LOC_API_GNSS_SIG_GLO_G2,
  LOC_API_GNSS_SIG_BDS_B1I,
  LOC_API_GNSS_SIG_BDS_B2I,
  LOC_API_GNSS_SIG_BDS_B3I,
  LOC_API_GNSS_SIG_BDS_B1C,
  LOC_API_GNSS_SIG_BDS_B2A,
  LOC_API_GNSS_SIG_BDS_B2B,
  LOC_API_GNSS_SIG_GAL_E1,
  LOC_API_GNSS_SIG_GAL_E5A,
  LOC_API_GNSS_SIG_GAL_E5B,
  LOC_API_GNSS_SIG_QZS_L1C,
  LOC_API_GNSS_SIG_QZS_L2C,
  LOC_API_GNSS_SIG_QZS_L5Q,
  LOC_API_GNSS_SIG_MAX,
} loc_api_SignalTypeEnumVal;
typedef uint8_t loc_api_SignalType;

/** Flag to indicate the gnss measurement status */
typedef struct
{
  uint16_t     b_valid   : 1;
  uint16_t     b_prValid : 1;
  uint16_t     b_drValid : 1;
  uint16_t     b_cpValid : 1;
} loc_api_GnssMeasStatus_t;

/* GNSS Measurement structure */
typedef struct
{
  loc_api_GnssMeasStatus_t        z_measStatus;     /** Measurement status flags */
  loc_api_gnssConstellationType   u_constellation;  /** Measurement constellation, see loc_api_gnssConstellationType */
  uint8_t             u_svid;                       /** Measurement satellite index of the iteself system */
  loc_api_SignalType  u_signal;                     /** Measurement signal type, see loc_api_SignalType */
  uint8_t             u_LLI;                        /** Loss of lock indicator */
  float               f_cn0;                        /** Carrier-to-noise density */
  float               f_doppler;                    /** Doppler */
  double              d_pseudoRange;                /** Pseudo-range */
  double              d_carrierPhase;               /** Carrier phase (cycle) */
} loc_api_GnssMeas_t;

typedef struct
{
  uint32_t  q_towMsec;
  uint16_t  w_week;
  double    d_lla[3];
  double    d_xyz[3];
  uint16_t  w_measCount;
  /** z_GnssMeas is a flexible length array with w_measCount measurements */
  loc_api_GnssMeas_t z_GnssMeas[1];
} loc_api_GnssMeasBlock_t;

/** Satellite position and velocity structure */
typedef struct {
 
  uint32_t             q_towMsec;      /** tot, no satellite clock correction */
  uint16_t             w_week;        
  uint8_t              u_valid;
  loc_api_gnssConstellationType  u_constellation;/** see loc_api_gnssConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  float                f_elevation;    /** rad */
  float                f_azimuth;      /** rad */
  int32_t              q_iode;         /** ephemeris iode */
  double               d_satPosClk[4]; /** Satellite Position X,Y,Z and Clock Bias, unit [m] */
  double               d_satVelClk[4]; /** Satellite Velocity X,Y,Z and Clock Drift unit [m] */
} loc_api_SatPosInfo_t;

/** GNSS measurement and satellite position */
#define VERSION_API_GNSS_MEAS_SAT_DATA  (0)
typedef struct
{
  uint8_t                      u_version;
  uint32_t                     q_towMsec;
  uint16_t                     w_week;
  double                       d_lla[3];
  double                       d_xyz[3];
  uint16_t                     w_measCount;
  uint8_t                      u_satCount;
  /** z_GnssMeas is a flexible length array with w_measCount measurements */
  loc_api_GnssMeas_t           z_GnssMeas[1];
  loc_api_SatPosInfo_t         z_SatPosInfo[1];
} loc_api_GnssMeasSatBlock_t;

#define VERSION_API_NAVIGATION_DATA  (0)
typedef struct
{
  uint8_t version;
  double tow;                        /* ms */
  loc_api_gnssConstellationType sys; /* see gnss_ConstellationType */
  uint8_t svid;
  loc_api_SignalType signal_type;
  uint16_t length_rtcm;
  uint8_t buffer_rtcm[1024];
  uint16_t length;                   /* bytes */
  uint8_t buffer[1];
}loc_api_NavigationData_t;

/* IMU Inject Data */
typedef struct {
  uint32_t q_tow_msec;
  float    f_gyro[3];
  float    f_accl[3];
  float    f_temperature;
} loc_api_imu_data_t;

typedef struct
{
  uint8_t   u_version;
  uint64_t  t_timestamp;
  uint32_t  q_fl_whpulse;
  uint32_t  q_fr_whpulse;
  uint32_t  q_rl_whpulse;
  uint32_t  q_rr_whpulse;
  float     f_angle_front;
  float     f_angle_rear;
  float     f_odometer;
  uint8_t   e_gear;
  uint8_t   u_selfck; /*self check*/
} loc_api_wheel_t;

/* PPS Inject Data */
typedef struct {
  uint64_t t_timestamp;
  uint16_t w_valid;
} loc_api_pps_t;

/** Register Callback function definition */
typedef struct {
  /** Get millisecond tick */
  uint64_t (*get_tick_ms)(void);
  
  /** Report loc engine log */
  void (*report_log)(uint8_t* buf, uint32_t len);

  /** Report location information */
  void (*report_location)(loc_api_location_report_t* info);
  
  /** Report GNSS to INS cooperate information */
  void (*report_gnss_feedback_ins)(loc_api_GnssFeedbackInsMeasBlock_t* pz_GnssFeedbackIns);

  /** Report Consoildate Position information */
  void (*report_consolidated_location)(loc_api_ConsoildatedPositionFix_t* info);
  
  /** Report gnss ephemeris */
  void (*report_ephemeris)(loc_api_GnssEphemeris_t* pz_GnssEph);
  
  /** Report gnss measurement */
  void (*report_gnss_measurement)(loc_api_GnssMeasBlock_t* pz_GnssMeasBlk);

  /** Report gnss measurement */
  void (*report_gnss_measurement_satellite)(loc_api_GnssMeasSatBlock_t* pz_GnssMeasSatBlk);

  /** Report orient information */
  void (*report_orient)(loc_api_orient_report_t* pz_orientResult);

  /** Report gnss measurement */
  void (*report_gnss_navigation_data)(loc_api_NavigationData_t * pz_NavData);

} loc_api_callback_t;

/** Configurate parameter definition */
typedef struct {
  uint64_t t_mask;
  uint8_t  u_field[10];
  float    f_field[10];
} loc_api_config_para_t;

typedef struct {
  loc_api_config_para_t para[8];
} loc_api_config_para_group_t;

/**
 * @brief     Location Engine API for Register callback functions
 * @param[in] cb - the callback function package
 * @return    None
 */
int32_t loc_api_Register_Callback(loc_api_callback_t* cb);

/**
 * @brief     Location Engine API for Register Memory pool
 * @param[in] pool_mem   - Memory pools pointer array
 * @param[in] pool_size  - Memory pools size array
 * @param[in] pool_count - Memory pools count
 * @return    None
 */
int32_t loc_api_Register_MemoryPool(loc_api_MemoryRegister_t* pz_MemoryRegister);

/**
 * @brief Location Engine API for Register configurations
 * @return      None
 */
int32_t loc_api_Register_Config(loc_api_config_para_group_t* pz_api_config_para_grp);

/**
 * @brief Location Engine API for start
 * @return      None
 */
int32_t loc_api_Initialize();

/**
 * @brief Location Engine API for release
 * @return      None
 */
int32_t loc_api_Release();

/**
 * @brief     Inject GNSS Receiver measurement by RTCM format
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 */
void loc_api_InjectRcvMeasRTCM(uint8_t* data, uint32_t len);

/**
 * @brief     Inject GNSS Reference station observed Correction measurement
              using RTCM format
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 * @note      The correction measurement is from NRTK service
 */
void loc_api_InjectRefCorrRTCM(uint8_t* data, uint32_t len);

/**
 * @brief     Inject GNSS Receiver measurement using measurement block
 * @param[in] p_data - pointer to GNSS Receiver measurement
 * @param[in] q_length - length of GNSS Receiver measurement
 * @return    None
 */
void loc_api_InjectRcvMeasBlock(uint8_t* p_data, uint32_t q_length);

/**
 * @brief     Inject GNSS API Ephemeris data
 * @param[in] p_data - pointer to GNSS Ephemeris data, loc_api_GnssEphemeris_t
 * @param[in] q_length - length of GNSS Ephemeris data, sizeof(loc_api_GnssEphemeris_t)
 * @return    None
 */
void loc_api_InjectApiEphemeris(uint8_t* p_data, uint32_t q_length);

/**
 * @brief Inject GNSS Ephemeris data
 * @param[in] p_data - pointer to GNSS Ephemeris data, gnss_Ephemeris_t
 * @param[in] q_length - length of GNSS Ephemeris data, sizeof(gnss_Ephemeris_t)
 * @return None
 */
void loc_api_InjectEphemeris(uint8_t* p_data, uint32_t q_length);

/**
 * @brief     Inject GNSS Reference station observed Correction measurement
              using measurement block
 * @param[in] p_data - pointer to GNSS Reference station observe
 * @param[in] q_length - length of GNSS Reference station observe
 * @return    None
 * @note      The correction measurement is from NRTK service as measurement block
 */
void loc_api_InjectRefCorrMeasBlk(uint8_t* p_data, uint32_t q_length);

/**
 * @brief     Inject Orient Receiver measurement by RTCM format
 * @param[in] data - pointer to Orient Receiver measurement
 * @param[in] len - length of Orient Receiver measurement
 * @return    None
 */
void loc_api_InjectOrtMeasRTCM(uint8_t* data, uint32_t len);

/**
 * @brief     Inject Orient Receiver measurement using measurement block
 * @param[in] p_data - pointer to GNSS Receiver measurement
 * @param[in] q_length - length of GNSS Receiver measurement
 * @return    None
 */
void loc_api_InjectOrtMeasBlock(uint8_t* p_data, uint32_t q_length);

/**
 * @brief     Inject SSR Correction stream
 * @param[in] data - stream data buffer
 * @param[in] len - stream data length
 * @param[in] source - stream data service source
 * @return    None
 */
void loc_api_InjectSsrStream(uint8_t* data, uint32_t len, gnss_ssrServiceSource source);

/**
 * @brief     Inject IMU data for INS module
 * @param[in] p_data - pointer to IMU data
 * @param[in] q_length - length of IMU data
 * @return    None
 */
void loc_api_InjectImuDataToIns(uint8_t* p_data, uint32_t q_length);

/**
 * @brief  Inject GNSS fix information for INS module
 * @param[in] p_data - pointer to GNSS fix information
 * @param[in] q_length - length of GNSS fix information
 * @return
 */
void loc_api_InjectGnssFixToIns(uint8_t* p_data, uint32_t q_length);

/**
 * @brief     Inject PPS time data
 * @param[in] pz_pps - pointer to loc_api_pps_t
 * @return    None
 */
void loc_api_InjectPPS(loc_api_pps_t* pz_pps);

/**
  * @brief	  Inject wheel data for INS module
  * @param[in] p_data - pointer to wheel data
  * @param[in] q_length - length of wheel data
  * @return	  None
*/
void loc_api_InjectWheelDataToIns(uint8_t* p_data, uint32_t q_length);


/**
* @brief Location Engine API for get version
* @return      Gnss version string
*/
const char* loc_api_GetVersion();


#ifdef __cplusplus
}
#endif

#endif
