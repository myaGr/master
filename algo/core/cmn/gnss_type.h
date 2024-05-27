/**@file        gnss_type.h
 * @brief       Location Engine GNSS Structure Types
 * @details     
 * @author      caizhijie
 * @date        2022/04/25
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __GNSS_TYPE_H__
#define __GNSS_TYPE_H__

#include "cmn_def.h"
#include "gnss_def.h"

BEGIN_DECL

/** Enum for all the supported GNSS Constellation types */
typedef enum
{
  C_GNSS_NONE  = 0,
  C_GNSS_GPS   = 1,
  C_GNSS_GLO   = 2,
  C_GNSS_BDS2  = 3,
  C_GNSS_BDS3  = 4,
  C_GNSS_GAL   = 5,
  C_GNSS_QZS   = 6,
  C_GNSS_NAVIC = 7,
  C_GNSS_MAX   = 8
} gnss_ConstellationTypeEnumVal;
typedef uint8_t gnss_ConstellationType;

/** Enum for all the supported GNSS satellite types */
typedef enum {
  C_SAT_TYPE_MEO = 0,
  C_SAT_TYPE_IGSO,
  C_SAT_TYPE_GEO,
  C_SAT_TYPE_MAX
} gnss_SatelliteTypeEnumVal;
typedef uint8_t gnss_SatelliteType;

/** Enum for all the supported GNSS single frequency measurement types */
typedef enum
{
  C_GNSS_SIG_GPS_L1C = 0,
  C_GNSS_SIG_GPS_L2C,
  C_GNSS_SIG_GPS_L5Q,
  C_GNSS_SIG_GLO_G1,
  C_GNSS_SIG_GLO_G2,
  C_GNSS_SIG_BDS_B1I,
  C_GNSS_SIG_BDS_B2I,
  C_GNSS_SIG_BDS_B3I,
  C_GNSS_SIG_BDS_B1C,
  C_GNSS_SIG_BDS_B2A,
  C_GNSS_SIG_BDS_B2B,
  C_GNSS_SIG_GAL_E1,
  C_GNSS_SIG_GAL_E5A,
  C_GNSS_SIG_GAL_E5B,
  C_GNSS_SIG_QZS_L1C,
  C_GNSS_SIG_QZS_L2C,
  C_GNSS_SIG_QZS_L5Q,
  C_GNSS_SIG_MAX,
} gnss_SignalTypeEnumVal;
typedef uint8_t gnss_SignalType;

/** Enum for all the supported frequency types */
typedef enum {
  C_GNSS_FREQ_TYPE_L1 = 0,
  C_GNSS_FREQ_TYPE_L2,
  C_GNSS_FREQ_TYPE_L5,
  C_GNSS_FREQ_TYPE_MAX
} gnss_FreqTypeEnumVal;
typedef uint8_t gnss_FreqType;

/** Enum for all the GNSS observation types */
typedef enum {
  C_GNSS_OBS_TYPE_PR = 0,
  C_GNSS_OBS_TYPE_DR,
  C_GNSS_OBS_TYPE_CR,
  C_GNSS_OBS_TYPE_MAX
} gnss_ObsTypeEnumVal;
typedef uint8_t gnss_ObsType;

/** Integrity flags */
typedef enum
{
  GNSS_INTEGRITY_FLAG_MONITORED_OK = 0,
  GNSS_INTEGRITY_FLAG_MONITORED_FAIL,
  GNSS_INTEGRITY_FLAG_NOT_MONITORED,
} gnss_integrityFlagVal;
typedef uint8_t gnss_integrityFlag;

/** Flag to indicate the gnss measurement status */
typedef struct {
  uint64_t      b_valid   : 1;
  uint64_t      b_prValid : 1;
  uint64_t      b_drValid : 1;
  uint64_t      b_cpValid : 1;
} gnss_MeasStatusFlag_t;

/* GPS Time Types, start at 1980-01-06 00:00:00 */
typedef struct {
  uint32_t      q_towMsec; /* MilliSeconds of the week, unit: 1ms */
  uint32_t      q_subNsec; /* Sub NanoSeconds of a MilliSeconds, unit: 1ns */
  uint16_t      w_week;    /* Week Number since GPS time start point */
  uint64_t      t_fullMsec;  /* Full Millisecond start at UTC 1970-01-01 00:00:00, unit: 1ms */
} GpsTime_t;

/* Glonass Time Types */
typedef struct {
  uint8_t       u_fourYear; /* GLO 4 year number from 1996 at the reference tick */
  uint16_t      w_days;     /* GLO day number in 4 years at the reference tick */
  uint32_t      q_towMsec;  /* GLO msec in the day at reference tick [msecs] */
  uint16_t      w_subMsec; /* unit:0.0001ms Sub MilliSeconds of tow */
} GloTime_t;

/* BDS Time Types is same as GpsTime_t
   BDS Time Types, start at 2006-01-01 00:00:00 UTC */
typedef GpsTime_t BdsTime_t;

/* GAL Time Types is same as GpsTime_t
   GAL Time Types, start at 1999-08-22 00:00:00 GPST */
typedef GpsTime_t GalTime_t;

/* UTC Time Types */
typedef struct {
  uint64_t t_unixMsec;
  uint32_t q_subNsec; /* Sub NanoSeconds of a MilliSeconds, unit: 1ns */
} UtcTime_t;

/* Epoch UTC Time Types */
typedef struct {
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hour;
  uint16_t min;
  float    second;
} EpochTime_t;

/* Receiver GNSS Clock structure */
typedef struct {
  GpsTime_t     z_gpsTime;    /** Receiver time using GPST */
  double        d_clockBias;  /** Receiver clock bias GPS L1 based */
  double        d_clockDrift; /** Receiver clock drift GPS L1 based */
} GnssClock_t;

/* GNSS Measurement structure */
typedef struct {
  gnss_MeasStatusFlag_t z_measStatusFlag;/** Measurement status flags */
  gnss_ConstellationType       u_constellation;         /** Measurement constellation, see gnss_ConstellationType */
  uint8_t       u_svid;                  /** Measurement satellite index of the iteself system */
  uint8_t       u_signal;                /** Measurement signal type, see gnss_SignalType */
  uint8_t       u_LLI;                   /** Loss of lock indicator */
  float         f_cn0;                   /** Carrier-to-noise density */
  double        d_pseudoRange;           /** Pseudo-range */
  double        d_doppler;               /** Doppler */
  double        d_carrierPhase;          /** Carrier phase (cycle) */
} GnssMeas_t;

/* GNSS Measurement LLI, add up lli between 2 BG epoch */
typedef struct {
  uint8_t       u_LLI[ALL_GNSS_SYS_SV_NUMBER][MAX_GNSS_SIGNAL_FREQ];/** Loss of lock indicator */
} LliAddUp_t;

/* GNSS Measurement structure extension */
typedef struct {
  uint8_t       u_multiPath;             /** Flag to Multi-path */
} GnssMeasExt_t;

/* Receiver Measurement Block structure */
#define VERSION_GNSS_MEAS_BLOCK  (1)
typedef struct {
  uint8_t       u_version;                            /** Structure version */
  uint16_t      w_size;                               /** Structure size */
  GnssClock_t   z_Clock;                              /** Measurement clock */
  uint16_t      w_measCount;                          /** Measurement count */
  GnssMeas_t    z_meas[MAX_GNSS_TRK_MEAS_NUMBER];     /** Measurement array */
  GnssMeasExt_t z_measExt[MAX_GNSS_TRK_MEAS_NUMBER];  /** Extension measurement array */
} GnssMeasBlock_t;

/* for report GNSS meas and get SSR LOS, convert to loc_api_GnssMeasBlock_t */
#define VERSION_GNSS_MEAS_BLOCK_COLLECT  (0)
typedef struct {
	uint8_t version;
	double d_lla[3];
	GnssMeasBlock_t z_measBlock;
}GnssMeasBlockCollect_t;

/* Correction Measurement Block structure */
#define VERSION_GNSS_CORR_BLOCK  (0)

typedef struct
{
  gnss_ConstellationType u_constellation;/** see gnss_ConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  int32_t              q_iode;         /** ephemeris iode */
  double               d_satPosClk[4]; /** Satellite Position X,Y,Z and Clock Bias */
}gnss_SatPosClkInfo_t;

typedef struct {
  uint8_t       u_version;                        /** Structure version */
  uint16_t      w_size;                           /** Structure size */
  uint16_t      w_refStationId;                   /** Reference station ID */
  uint16_t      w_switchStationId;                /** Station ID flag for station switch, when
                                                  station switches and AMB change succ, it is
                                                  equal to station ID to be switched to.
                                                  In other case, it equals w_refStationId.*/
  double        d_refPosEcef[3];                  /** Reference position in ECEF coordinate */
  GnssClock_t   z_Clock;                          /** Correction Measurement time */
  uint16_t      w_measCount;                      /** Correction Measurement count */
  uint16_t      w_satCount;                       /** Correction satellite position and clock count */
  GnssMeas_t    z_meas[MAX_GNSS_TRK_MEAS_NUMBER]; /** Correction Measurement array */
  double        d_trop[ALL_GNSS_SYS_SV_NUMBER];   /** (update in future)Tropospheric delay for each sat*/
  gnss_SatPosClkInfo_t pz_satPosClk[MAX_GNSS_ACTIVE_SAT_NUMBER];/** Correction satellite position and clock array */
} GnssCorrBlock_t;

typedef struct {
  uint8_t  u_slip[ALL_GNSS_SYS_SV_NUMBER][MAX_GNSS_SIGNAL_FREQ];/** Correction satellite SlipFlag */
  uint8_t  u_changeStationFlag;                                 /** Corr station switch Flag */
} GnssCorrSlipFlag_t;

typedef struct {
  uint8_t  pu_MPflag[ALL_GNSS_SYS_SV_NUMBER][MAX_GNSS_SIGNAL_FREQ];/** Multipath flag, see MULTIPATH_FLAG_... */
} GnssMultiPathFlag_t;

#define RTK_CORR_MAX_BLOCK      (2)               /*the maximum number of RTK correction block, default > 1*/

/** GPS Legacy Navigation ephemeris (L-NAV) L1/L2 50 bps PRN 1~32 */
typedef struct {
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
} GpsEphemeris_t;

typedef struct { 
  uint8_t   svid;     /** Satellite Number */
  /* TBD */
} GloEphemeris_t;

/** BDS MEO/IGSO B1I D1 Navigation 50 bps, GEO B1I D2 Navigation 500 bps */
typedef struct {
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
} BdsEphemeris_t;

/** GAL I/NAV ephemeris E1,E5b */
typedef struct {
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
} GalEphemeris_t;

#define VERSION_NAVIGATION_DATA  (1)
typedef struct
{
  uint8_t                version;
  double                 d_tow;                                /* ms */ 
  gnss_ConstellationType sys;                                  /* see gnss_ConstellationType */
  uint8_t                svid;
  gnss_SignalType        signal_type;
  uint16_t               length;                               /* bytes */
  uint8_t                buffer[MAX_NAV_DATA_FRAME_SIZE + 1];
  uint16_t               length_rtcm;
  uint8_t                buffer_rtcm[1024];
}NavigationData_t;

#define VERSION_GNSS_EPHEMERIS  (0)
typedef struct {
  uint8_t          u_version;
  union {
    GpsEphemeris_t z_gpsEph;
    GloEphemeris_t z_gloEph;
    BdsEphemeris_t z_bdsEph;
    GalEphemeris_t z_galEph;
    GpsEphemeris_t z_qzsEph;
  } eph;
  gnss_ConstellationType  u_constellation;  /* see:gnss_ConstellationType */
  GpsTime_t z_toeOfGpst;
  GpsTime_t z_tocOfGpst;
} gnss_Ephemeris_t;

typedef enum {
  COORD_SRC_NONE = 0,
  COORD_SRC_EXT,
  COORD_SRC_WLS,
  COORD_SRC_KF,
  COORD_SRC_RTK,
  COORD_SRC_PPP,
  COORD_SRC_VDR,
  COORD_SRC_AVG_SAT,
  COORD_SRC_MAX,
} gnss_CoordSrcEnumTypeVal;
typedef uint8_t gnss_CoordSrcEnumType;

typedef struct {
  GpsTime_t z_time;
  uint8_t u_src;   /** see gnss_CoordSrcEnumType */
  double  d_xyz[3];
  double  d_xyz_unc[3];
} gnss_coord_t;

/* algorithm solution filter position status */
typedef enum
{
   GNSS_FILTER_POS_SPP     = 0,     /* 0:SPP  */
   GNSS_FILTER_POS_PREDICT = 1,     /* 1:Predict */
   GNSS_FILTER_POS_INS     = 2,     /* 2:Ins machinery arrangement */
   GNSS_FILTER_POS_RTD     = 3,     /* 3:RTD */
   GNSS_FILTER_POS_FLOAT   = 4,     /* 4:Float */
   GNSS_FILTER_POS_PREFIX  = 5,     /* 5:Pre-Fix */
   GNSS_FILTER_POS_FIX     = 6,     /* 6:Fix */
   GNSS_FILTER_POS_FIX_WL  = 7,     /* 7:WideLane FIX */
   GNSS_FILTER_POS_FIX_EWL = 8,     /* 8:Extra WideLane FIX */
   GNSS_FILTER_POS_STATUS  = 9      /* num of saved pos status in rtk filter */
} gnss_FilterPosEnumTypeVal;
typedef uint8_t gnss_FilterPosType;

typedef enum {
  GNSS_FIX_FLAG_INVALID = 0,
  GNSS_FIX_FLAG_SPS,
  GNSS_FIX_FLAG_DGNSS,
  GNSS_FIX_FLAG_PPS,
  GNSS_FIX_FLAG_FIXED,
  GNSS_FIX_FLAG_FLOATING,
  GNSS_FIX_FLAG_DR,
  GNSS_FIX_FLAG_MANUAL,
  GNSS_FIX_FLAG_SIMULATOR
} gnss_FixFlagTypeEnumVal;
typedef uint8_t gnss_FixFlagType;

typedef enum { /* don`t insert form mid, only support append*/
  FIX_SOURCE_INVALID = 0,
  FIX_SOURCE_PVT_WLS,
  FIX_SOURCE_PVT_KF,
  FIX_SOURCE_RTD,
  FIX_SOURCE_RTK,
  FIX_SOURCE_RTK_FLOAT,
  FIX_SOURCE_RTK_FIX,
  FIX_SOURCE_PPP,
  FIX_SOURCE_PPP_FLOAT,
  FIX_SOURCE_PPP_FIX,
  FIX_SOURCE_VDR,
  FIX_SOURCE_HPP_V1,
  FIX_SOURCE_DCP,
  FIX_SOURCE_HPP_V2,
  FIX_SOURCE_MAX
} gnss_FixSourceTypeEnumVal;
typedef uint8_t gnss_FixSourceType;

/* position type of obtained method */
typedef enum {
  GNSS_DCP_POS_INVALID = 0,
  GNSS_DCP_POS_CARRIER,
  GNSS_DCP_POS_DOPPLER,
  GNSS_DCP_POS_PSEUDO,
} gnss_DcpTypeEnumVal;
typedef uint8_t gnss_DcpType;

typedef struct {
  float         f_gdop;
  float         f_pdop;
  float         f_hdop;
  float         f_vdop;
  float         f_tdop;
} gnss_PositionFix_Dops_t;

typedef struct {
  gnss_ConstellationType u_constellation;/** see gnss_ConstellationType */
  uint8_t       u_svid;         /** satellite index of the system */
  uint8_t       u_signal;       /** see gnss_SignalType */
  float         f_cn0;
  float         f_elevation;    /** degree */
  float         f_azimuth;      /** degree */
} gnss_PositionFixSV_t;

typedef struct {
  uint8_t      u_MeasTrackCount;  // measurement track count
  uint8_t      u_MeasInUseCount;  // measurement in use count 
  uint8_t      u_SvTrackCount;    // sv track count
  uint8_t      u_SvInUseCount;    // sv in used count
  uint64_t     t_SvTrackMask[C_GNSS_MAX];
  uint64_t     t_SvInUseMask[C_GNSS_MAX];
  gnss_PositionFixSV_t z_SV[MAX_GNSS_TRK_MEAS_NUMBER];
} gnss_PositionFix_SvStatus_t;

typedef struct {
  uint32_t      q_StationID;
  float         f_age;          /* different age (s) */
  double        pd_StationCoordinate[3];  /* Station Coordinate of VRS */
} gnss_PositionFix_RTK_t;

typedef struct {
  uint8_t  u_drposflag;
  float    f_roll;       /* degree */
  float    f_pitch;      /* degree */
  float    f_heading;    /* degree */
  float    f_gyrobias_x; /* degree/second */
  float    f_gyrobias_y; /* degree/second */
  float    f_gyrobias_z; /* degree/second */
  float    f_accbias_x;  /* meter/second^2 */
  float    f_accbias_y;  /* meter/second^2 */
  float    f_accbias_z;  /* meter/second^2 */
  float    f_roll_std;   /* degree */
  float    f_pitch_std;  /* degree */
  float    f_yaw_std;    /* degree */
  float    f_mis_roll;   /* degree */
  float    f_mis_pitch;  /* degree */
  float    f_mis_yaw;    /* degree */
  float    f_speed;      /* meter/sec */
  float    f_whlspd_sf[4];
  uint32_t q_kfmeastype;
  float    f_la_imu2gnss[3];
  float    f_la_imu2rearmid[3];
  float    f_la_rear2rear;
  float    f_misdualant_roll;     /* deg */
  float    f_misdualant_pitch;    /* deg */
  float    f_misdualant_yaw;      /* deg */
} gnss_PositionFix_INS_t;

/** Protection level and flag */
typedef struct
{
  gnss_integrityFlag  u_flag;               /* integrity flag */
  float               f_protection_level;   /* protection level, m */
}gnss_ProtectionLevel_t;

/** Integrity in multiple coordinate system */
typedef struct
{
  gnss_ProtectionLevel_t        z_pos_longitudinal_pl;   /* Position longitudinal */
  gnss_ProtectionLevel_t        z_pos_lateral_pl;        /* Position lateral  */
  gnss_ProtectionLevel_t        z_pos_north_pl;          /* Position North */
  gnss_ProtectionLevel_t        z_pos_east_pl;           /* Position East */
  gnss_ProtectionLevel_t        z_pos_hor_pl;            /* Position Horizontal */
  gnss_ProtectionLevel_t        z_pos_ver_pl;            /* Position Vertical */
  gnss_ProtectionLevel_t        z_vel_longitudinal_pl;   /* Velocity longitudinal */
  gnss_ProtectionLevel_t        z_vel_lateral_pl;        /* Velocity lateral  */
  gnss_ProtectionLevel_t        z_vel_north_pl;          /* Velocity North */
  gnss_ProtectionLevel_t        z_vel_east_pl;           /* Velocity East */
  gnss_ProtectionLevel_t        z_vel_hor_pl;            /* Velocity Horizontal */
  gnss_ProtectionLevel_t        z_vel_ver_pl;            /* Velocity Vertical */
  gnss_ProtectionLevel_t        z_roll_pl;               /* Roll */
  gnss_ProtectionLevel_t        z_pitch_pl;              /* Pitch */
  gnss_ProtectionLevel_t        z_yaw_pl;                /* Yaw */
}gnss_Integrity_t;

#define VERSION_POSITION_FIX  (8)
typedef struct {
  uint8_t                      u_version;
  uint16_t                     w_size;
  GpsTime_t                    z_gpsTime;
  EpochTime_t                  z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t                      u_leapsec;           /* Leap second */
  uint8_t                      u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t                      u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t                      u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t                      u_CN040;             /* Sat number fre1 cn0>40  */
  float                        d_avgCN0;            /* Average CN0 */
  double                       d_quasiGeoidHeight;  /* Quasi-geoid height */
  double                       d_xyz[3];            /* Position: X,Y,Z in ECEF (m) */
  double                       d_lla[3];            /* Position: lat, lon, alt */
  float                        f_velXyz[3];         /* Velocity: X,Y,Z in ECEF (m/s) */
  float                        f_velEnu[3];         /* Velocity: lat, lon, alt */
  float                        f_posXyzUnc[3];      /* Uncertainy position XYZ uncertainy (m) */
  float                        f_posLlaUnc[3];      /* Uncertainy position LLA  (m) */
  float                        f_velEnuUnc[3];      /* Uncertainy velocity : ENU  (m/s) */
  double                       d_clockBias;         /* Primary constellation clock bias */
  double                       d_clockDrift;        /* Primary constellation clock drift */
  gnss_Integrity_t             z_integrity;         /* Integrity of GNSS or INS */
  gnss_PositionFix_Dops_t      z_dops;
  gnss_PositionFix_RTK_t       z_rtk;
  gnss_PositionFix_INS_t       z_ins;
  gnss_PositionFix_SvStatus_t  z_SvStatus;
} gnss_PositionFix_t;

/*the solution status of GNSS orientation*/
typedef enum
{
  C_GNSS_ORT_SOL_STATUS_SOL_COMPUTED = 0,           /*Solution computed*/
  C_GNSS_ORT_SOL_STATUS_INSUFFICIENT_OBS = 1,       /*Insufficient observations*/
  C_GNSS_ORT_SOL_STATUS_NO_CONVERGENCE = 2,         /*No convergence*/
  C_GNSS_ORT_SOL_STATUS_SINGULARITY = 3,            /*Singularity at parameters matrix*/
  C_GNSS_ORT_SOL_STATUS_COV_TRACE = 4,              /*Covariance trace exceeds maximum (trace > 1000 m)*/
  C_GNSS_ORT_SOL_STATUS_TEST_DIST = 5,              /*Test distance exceeded (maximum of 3 rejections if distance >10 km)*/
  C_GNSS_ORT_SOL_STATUS_COLD_START = 6,             /*Not yet converged from cold start*/
  C_GNSS_ORT_SOL_STATUS_V_H_LIMIT = 7,              /*Height or velocity limits exceeded (in accordance with export licensing restrictions)*/
  C_GNSS_ORT_SOL_STATUS_VARIANCE = 8,               /*Variance exceeds limits*/
  C_GNSS_ORT_SOL_STATUS_RESIDUALS = 9,              /*Residuals are too large*/
  C_GNSS_ORT_SOL_STATUS_INTEGRITY_WARNING = 13,     /*Large residuals make position unreliable*/
  C_GNSS_ORT_SOL_STATUS_PENDING = 18,               /*When a FIX position command is entered, the receiver computes its own position and determines if the fixed position is valid*/
  C_GNSS_ORT_SOL_STATUS_INVALID_FIX = 19,           /*The fixed position, entered using the FIX position command, is not valid*/
  C_GNSS_ORT_SOL_STATUS_UNAUTHORIZED = 20,          /*Position type is unauthorized*/
  C_GNSS_ORT_SOL_STATUS_INVALID_RATE = 22           /*The selected logging rate is not supported for this solution type*/
}gnss_orientSolStatusTypeEnumVal;
typedef uint8_t gnss_orientSolStatusType;
/*the position or velocity type of GNSS orientation*/
typedef enum
{
  C_GNSS_ORT_POS_VEL_TYPE_NONE = 0,                         /*No solution*/
  C_GNSS_ORT_POS_VEL_TYPE_FIXEDPOS = 1,                     /*Position has been fixed by the FIX position command or by position averaging*/
  C_GNSS_ORT_POS_VEL_TYPE_FIXEDHEIGHT = 2,                  /*Position has been fixed by the FIX height or FIX auto command or by position averaging*/
  C_GNSS_ORT_POS_VEL_TYPE_DOPPLER_VELOCITY = 8,             /*Velocity computed using instantaneous Doppler*/
  C_GNSS_ORT_POS_VEL_TYPE_SINGLE = 16,                      /*Solution calculated using only data supplied by the GNSS satellites*/
  C_GNSS_ORT_POS_VEL_TYPE_PSRDIFF = 17,                     /*Solution calculated using pseudorange differential (DGPS, DGNSS) corrections*/
  C_GNSS_ORT_POS_VEL_TYPE_WAAS = 18,                        /*Solution calculated using corrections from an SBAS satellite*/
  C_GNSS_ORT_POS_VEL_TYPE_PROPAGATED = 19,                  /*Propagated by a Kalman filter without new observations*/
  C_GNSS_ORT_POS_VEL_TYPE_L1_FLOAT = 32,                    /*Single-frequency RTK solution with unresolved, float carrier phase ambiguities*/
  C_GNSS_ORT_POS_VEL_TYPE_NARROW_FLOAT = 34,                /*Multi-frequency RTK solution with unresolved, float carrier phase ambiguities*/
  C_GNSS_ORT_POS_VEL_TYPE_L1_INT = 48,                      /*Single-frequency RTK solution with carrier phase ambiguities resolved to integers*/
  C_GNSS_ORT_POS_VEL_TYPE_WIDE_INT = 49,                    /*Multi-frequency RTK solution with carrier phase ambiguities resolved to wide-lane integers*/
  C_GNSS_ORT_POS_VEL_TYPE_NARROW_INT = 50,                  /*Multi-frequency RTK solution with carrier phase ambiguities resolved to narrow-lane integers*/
  C_GNSS_ORT_POS_VEL_TYPE_RTK_DIRECT_INS = 51,              /*RTK status where the RTK filter is directly initialized from the INS filter*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_SBAS = 52,                    /*INS position, where the last applied position update used a GNSS solution computed using corrections from an SBAS (WAAS) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_PSRSP = 53,                   /*INS position, where the last applied position update used a single point GNSS (SINGLE) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_PSRDIFF = 54,                 /*INS position, where the last applied position update used a pseudorange differential GNSS (PSRDIFF) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_RTKFLOAT = 55,                /*INS position, where the last applied position update used a floating ambiguity RTK (L1_FLOAT or NARROW_FLOAT) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_RTKFIXED = 56,                /*INS position, where the last applied position update used a fixed integer ambiguity RTK (L1_INT, WIDE_INT or NARROW_INT) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_PPP_CONVERGING = 68,              /*Converging TerraStar-C, TerraStar-C PRO or TerraStar-X solution*/
  C_GNSS_ORT_POS_VEL_TYPE_PPP = 69,                         /*Converged TerraStar-C, TerraStar-C PRO or TerraStar-X solution*/
  C_GNSS_ORT_POS_VEL_TYPE_OPERATIONAL = 70,                 /*Solution accuracy is within UAL operational limit*/
  C_GNSS_ORT_POS_VEL_TYPE_WARNING = 71,                     /*Solution accuracy is outside UAL operational limit but within warning limit*/
  C_GNSS_ORT_POS_VEL_TYPE_OUT_OF_BOUNDS = 72,               /*Solution accuracy is outside UAL limits*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_PPP_CONVERGING = 73,          /*INS position, where the last applied position update used a converging TerraStar-C,TerraStar-C PRO or TerraStar-X PPP (PPP_CONVERGING) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_PPP = 74,                     /*INS position, where the last applied position update used a converged TerraStar-C,TerraStar-C PRO or TerraStar-X PPP (PPP) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_PPP_BASIC_CONVERGING = 77,        /*Converging TerraStar-L solution*/
  C_GNSS_ORT_POS_VEL_TYPE_PPP_BASIC = 78,                   /*Converged TerraStar-L solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_PPP_BASIC_CONVERGING = 79,    /*INS position, where the last applied position update used a converging TerraStar-L PPP (PPP_BASIC) solution*/
  C_GNSS_ORT_POS_VEL_TYPE_INS_PPP_BASIC = 80                /*INS position, where the last applied position update used a converged TerraStar-L PPP (PPP_BASIC) solution*/
}gnss_orientPosVelTypeEnumValue;
typedef uint8_t gnss_orientPosVelType;

/*the mask of GPS used in solution*/
typedef enum
{
  C_GNSS_ORT_GPS_L1_USED = 0x01,/*GPS L1 used in Solution*/
  C_GNSS_ORT_GPS_L2_USED = 0x02,/*GPS L2 used in Solution*/
  C_GNSS_ORT_GPS_L5_USED = 0x04,/*GPS L5 used in Solution*/
}gnss_orientGPSmaskEnumValue;
typedef uint8_t gnss_orientGPSmask;

/*the mask of GLONASS used in solution*/
typedef enum
{
  C_GNSS_ORT_GLO_L1_USED = 0x10,/*GLONASS L1 used in Solution*/
  C_GNSS_ORT_GLO_L2_USED = 0x20,/*GLONASS L2 used in Solution*/
  C_GNSS_ORT_GLO_L3_USED = 0x40,/*GLONASS L3 used in Solution*/
}gnss_orientGLOmaskEnumValue;
typedef uint8_t gnss_orientGLOmask;

/*the mask of Galileo used in solution*/
typedef enum
{
  C_GNSS_ORT_GAL_L1_USED = 0x01,/*Galileo E1 used in Solution*/
  C_GNSS_ORT_GAL_L2_USED = 0x02,/*Galileo E5A used in Solution*/
  C_GNSS_ORT_GAL_L5_USED = 0x04,/*Galileo E5B used in Solution*/
}gnss_orientGALmaskEnumValue;
typedef uint8_t gnss_orientGALmask;

/*the mask of BeiDou used in solution*/
typedef enum
{
  C_GNSS_ORT_BDS_L1_USED = 0x10,/*BeiDou B1 used in Solution*/
  C_GNSS_ORT_BDS_L2_USED = 0x20,/*BeiDou B2 used in Solution*/
  C_GNSS_ORT_BDS_L3_USED = 0x40,/*BeiDou B3 used in Solution*/
  C_GNSS_ORT_BDS_L5_USED = 0x20,/*BeiDou B2a used in Solution,according to manual to modify,(B2I,B2a,B2b value is 0x20)*/
}gnss_orientBDSmaskEnumValue;
typedef uint8_t gnss_orientBDSmask;

typedef struct
{
  float                    f_heading;      /*Heading value,unit in degree,from 0 to 360*/
  float                    f_pitch;        /*Pitch value,unit in degree,from -90 to 90*/
  float                    f_roll;         /*Roll value,unit in degree,from -90 to 90*/
  float                    f_headingStd;   /*the STD of heading,unit in degree*/
  float                    f_pitchStd;     /*the STD of pitch, unit in degree*/
  float                    f_rollStd;      /*the STD of roll, unit in degree*/
  float                    f_deltaXYZ[3];  /*the coordinate of XYZ direction based at main antenna pointer to auxiliary antenna,the length can be calculated by the format:length=sqrt(f_deltaXYZ[0]*f_deltaXYZ[0]+f_deltaXYZ[1]*f_deltaXYZ[1]+f_deltaXYZ[2]*f_deltaXYZ[2])*/
  float                    f_ENU[3];       /*the coordinate of ENU direction based at main antenna pointer to auxiliary antenna,the length can be calculated by the format:length=sqrt(f_ENU[0]*f_ENU[0]+f_ENU[1]*f_ENU[1]+f_ENU[2]*f_ENU[2])*/
  float                    f_velXyz[3];    /* Velocity: X,Y,Z in ECEF (m/s) */
  float                    f_velEnu[3];    /* Velocity: lat, lon, alt */
}gnss_headingPitchRoll_t;

typedef struct
{
  uint8_t       u_posFixFlag;        /* see: gnss_FixFlagType */
  uint8_t       u_SvInUseCount;      /* sv in used count */ 
  double        d_xyz[3];            /* Position: X,Y,Z in ECEF (m) */
  double        d_lla[3];            /* Position: lat, lon, alt */
  float         f_velXyz[3];         /* Velocity: X,Y,Z in ECEF (m/s) */
  float         f_velEnu[3];         /* Velocity: lat, lon, alt */
  double        pd_refStationCoordinate[3];/* Station Coordinate of VRS */
}gnss_ortMainAntInfo_t;

#define ORT_MAIN_AUXI_TIME_DIFF_THRES  (0.03)                                           /*the maximum difference between main antenna measure time and postion result*/
#define VERSION_GNSS_ORT_FIX  (0)
typedef struct
{
  uint8_t                  u_version;
  uint16_t                 w_size;
  uint8_t                  u_fixSource;             /* see: gnss_FixSourceType */
  GpsTime_t                z_gpsTime;
  EpochTime_t              z_epoch;                 /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t                  u_leapsec;               /* Leap second */
  uint8_t                  u_DcpPosType;            /* DCP position type, see: gnss_DcpType */
  uint8_t                  u_ortFixFlag;               /* see: gnss_FixFlagType */
  uint8_t                  u_SvTrackCount;          /*sv track count*/ 
  uint8_t                  u_SvTrackCountEleThres;  /*sv in track count that elevation greater than threshold*/
  uint8_t                  u_SvInUseCount;          /*sv in used count*/ 
  uint8_t                  u_SvInUseCountMuliFreq;  /*sv in used count that had dual or multi frequency*/
  gnss_orientSolStatusType u_ortSolStatus;          /*the solution status of GNSS orientation*/
  gnss_orientPosVelType    u_ortPosVelType;         /*the position or velocity type of GNSS orientation*/
  gnss_orientGPSmask       u_GPSsignalUsedMask;     /*the mask of GPS used in solution*/
  gnss_orientGLOmask       u_GLOsignalUsedMask;     /*the mask of GLONASS used in solution*/
  gnss_orientGALmask       u_GalSignalUsedMask;     /*the mask of Galileo used in solution*/
  gnss_orientBDSmask       u_BDSsignalUsedMask;     /*the mask of BDS used in solution*/
  float                    f_age;                   /*different age (s)*/
  gnss_headingPitchRoll_t  z_ortResult;             /*the orient result of heading, pitch and roll */
  gnss_ortMainAntInfo_t    z_mainAntInfo;
} gnss_OrientFix_t;

/* Temporary location report location information */
#define VERSION_GNSS_NAV_SOLUTION  (0)
typedef struct {
  uint8_t              u_version;
  gnss_PositionFix_t   z_positionFix;
  char                 gga[256];
  char                 rmc[256];
} gnss_NavSolution_t;

typedef struct {
  GpsTime_t            z_gpsTime;      /** tot, no satellite clock correction */
  uint8_t              u_valid;
  gnss_ConstellationType u_constellation;/** see gnss_ConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  float                f_elevation;    /** rad */
  float                f_azimuth;      /** rad */
  float                f_dt;           /** difference time t with first polynomial t*/
  int32_t              q_iode;         /** ephemeris iode */
  double               d_satPosClk[4]; /** Satellite Position X,Y,Z and Clock Bias, unit [m] */
  double               d_satVelClk[4]; /** Satellite Velocity X,Y,Z and Clock Drift unit [m] */
} gnss_SatPosVelClk_t;

typedef struct {
  gnss_ConstellationType  u_constellation;/** see gnss_ConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  GpsTime_t            z_fitStart;     /** Interpolation start time */
  GpsTime_t            z_fitEnd;       /** Interpolation end time */
  float                f_toe_age;      /** from toe to interpolation start time */
  int32_t              q_iode;         /** ephemeris iode */
  double               d_clkPolynomial[4];
  double               d_posClkPolynomial[4][4];
  double               d_velClkPolynomial[4][4];
} gnss_SatPosVelClkPolynomial_t;

/* Receiver Measurements and satellite position/velocity Block structure */
#define VERSION_GNSS_MEAS_SAT_BLOCK  (0)
typedef struct {
  uint8_t             u_version;                                  /** Structure version */
  uint16_t            w_size;                                     /** Structure size */
  GnssClock_t         z_Clock;                                    /** Measurement clock */
  uint16_t            w_measCount;                                /** Measurement count */
  GnssMeas_t          z_meas[MAX_GNSS_TRK_MEAS_NUMBER];           /** Measurement array */
  GnssMeasExt_t       z_measExt[MAX_GNSS_TRK_MEAS_NUMBER];        /** Extension measurement array */
  uint8_t             u_satCount;
  gnss_SatPosVelClk_t z_satPosVelClk[MAX_GNSS_ACTIVE_SAT_NUMBER]; /** Satellite position velocity clock */
} GnssMeasSatBlock_t;

/* for report GNSS meas satellite position and get SSR LOS, convert to loc_api_GnssMeasSatBlock_t */
#define VERSION_GNSS_MEAS_SAT_BLOCK_COLLECT  (0)
typedef struct {
	uint8_t version;
	double d_lla[3];
	GnssMeasSatBlock_t z_measBlock;
}GnssMeasSatBlockCollect_t;

/* cycle slip */
typedef uint8_t algo_cycleSlipFlag;
#define CYCLE_SLIP_DEFAULT            ((uint8_t)0x00)
#define NON_CYCLE_SLIP_BY_GF          ((uint8_t)0x01)
#define NON_CYCLE_SLIP_BY_MW          ((uint8_t)0x02)
#define NON_1CYCLE_SLIP_BY_DOPPLER    ((uint8_t)0x04)
#define NON_CYCLE_SLIP_BY_TDCP        ((uint8_t)0x08)
#define NON_5CYCLE_SLIP_BY_DOPPLER    ((uint8_t)0x10)
#define NON_CYCLE_SLIP_BY_MULTIFREQUENCY ((uint8_t)0x20)
/* valid of cycle slip method */
#define GF_DETECT_VALID               ((uint8_t)0x01)
#define MW_DETECT_VALID               ((uint8_t)0x02)
#define DOPPLER_DETECT_VALID          ((uint8_t)0x04)
#define TDCP_DETECT_VALID             ((uint8_t)0x08)
#define MULTIFREQUENCY_DETECT_VALID   ((uint8_t)0x10)

/* cycle slip flag for ref corr */
#define CORR_SLIP_DEFAULT            ((uint8_t)0x00)
#define HAS_CORR_SLIP_BY_GF          ((uint8_t)0x01)
#define NON_CORR_SLIP_BY_GF          ((uint8_t)0x02)
#define HAS_CORR_SLIP_BY_TD          ((uint8_t)0x04)
#define NON_CORR_SLIP_BY_TD          ((uint8_t)0x08)

/* Multipath flag detected by CMC/GIF/SNR */
#define MULTIPATH_FLAG_CP            ((uint8_t)0x07)
#define MULTIPATH_FLAG_CP_UNKNOW     ((uint8_t)0x00)
#define MULTIPATH_FLAG_CP_NORMAL     ((uint8_t)0x01)
#define MULTIPATH_FLAG_CP_LOW        ((uint8_t)0x02)
#define MULTIPATH_FLAG_CP_HIGH       ((uint8_t)0x04)
#define MULTIPATH_FLAG_PR            ((uint8_t)0x70)
#define MULTIPATH_FLAG_PR_UNKNOW     ((uint8_t)0x00)
#define MULTIPATH_FLAG_PR_NORMAL     ((uint8_t)0x10)
#define MULTIPATH_FLAG_PR_LOW        ((uint8_t)0x20)
#define MULTIPATH_FLAG_PR_HIGH       ((uint8_t)0x40)

/* the mask of pseoudo-range */
typedef uint8_t algo_PRqualityMask;
#define PR_QUALITY_NORM                ((uint8_t)0x00)
#define PR_QUALITY_GROSS_BY_QR_PARITY  ((uint8_t)0x01)

/* the mask of carrier-phase */
typedef uint8_t algo_CPqualityMask;
#define CP_QUALITY_NORM                ((uint8_t)0x00)
#define CP_QUALITY_GROSS_BY_QR_PARITY  ((uint8_t)0x01)

typedef struct {
  algo_PRqualityMask u_PRquality;
  algo_CPqualityMask u_CPquality;
}gnss_SignalQualityMask_t;

typedef struct {
  GpsTime_t             z_latest_tor;     /** latest measure time of receive(tor) */
  gnss_MeasStatusFlag_t z_measStatusFlag;
  gnss_ConstellationType  u_constellation;  /** see gnss_ConstellationType */
  uint8_t               u_svid;           /** satellite index of the system */
  gnss_SignalType       u_signal;         /** see gnss_SignalType */
  uint8_t               u_LLI;
  float                 f_cn0;
  double                d_pseudoRange;
  double                d_doppler;
  double                d_carrierPhase;
  uint8_t               u_slipMask;       /* cycle-slip flag */
  uint8_t               u_slipMethodValid;/* the valid of detecting cycle slip method*/
  uint8_t               b_half_slip;      /* half-cycle valid flag */
  uint32_t              q_slipc;          /* cycle-slip counter */
  double                d_pr_unc;         /* pseudorange compute unc, unit: meter*/
  double                d_dr_unc;         /* doppler compute unc, unit: m/s */
} gnss_SignalMeas_t;

typedef struct {
  GpsTime_t             z_tot;                            /** time of satellite signal transmit (tot) */
  gnss_ConstellationType u_constellation;                  /** see gnss_ConstellationType*/
  uint8_t               u_svid;
  gnss_SignalQualityMask_t pz_signalQuality[MAX_GNSS_SIGNAL_FREQ];
  gnss_SatPosVelClk_t   z_satPosVelClk;
  uint8_t               u_eclips;                         /** eclips satellite */
  double                d_wetMap;                         /** tropospheric map of wet part*/
  double                d_dryMap;                         /** tropospheric map of dry part*/
  double                d_Iono;                           /** Ionospheric correction, m  */
  double                d_Tropo;                          /** Tropospheric correction, m */
  double                d_Gravitation;                    /** Gravitation delay,m */
  uint8_t               u_rejectLabel[MAX_GNSS_SIGNAL_FREQ];
  float                 f_gf[MAX_GNSS_SIGNAL_FREQ];
  float                 f_mw[MAX_GNSS_SIGNAL_FREQ];
  int32_t               q_mw_cnt[MAX_GNSS_SIGNAL_FREQ];   /* mw count */
  float                 f_mw_avg[MAX_GNSS_SIGNAL_FREQ];   /* mw average */
  float                 f_mw_std[MAX_GNSS_SIGNAL_FREQ];   /* mw standard deviation */
  double                d_gfArcLen[MAX_GNSS_SIGNAL_FREQ];
  double                d_mwArcLen[MAX_GNSS_SIGNAL_FREQ];
  gnss_SignalMeas_t*    pz_signalMeas[MAX_GNSS_SIGNAL_FREQ];
} gnss_SatelliteMeas_t;

#define VERSION_GNSS_OBSERVATIONS  (0)
typedef struct {
  uint8_t               u_version;
  GpsTime_t             z_tor;        /** Time of measurement receive (tor) */
  int32_t               q_clk_jump;   /** Receiver clock jump (ms) */
  double                d_ZTD;        /** Tropospheric empirical delaly */
  double                d_zhd_emp;    /** Tropospheric empirical delaly of dry part */
  double                d_zwd_emp;    /** Tropospheric empirical delaly of wet part */
  uint8_t               u_satMeasCount;
  uint8_t               u_satMeasIdxTable[MAX_GNSS_ACTIVE_SAT_NUMBER];
  float                 f_earthTide[3]; /* earth tide correction for recevier */

  /** ALL_GNSS_SYS_SV_NUMBER=N_GPS_SV+N_GLO_SV+N_BDS_SV+N_GAL_SV+N_QZS_SV */
  gnss_SatelliteMeas_t* pz_satMeas[ALL_GNSS_SYS_SV_NUMBER];
  gnss_PositionFix_t    z_positionFix;  /** pvt pos solution */
} gnss_SatSigMeasCollect_t;

typedef struct
{
  double d_val[5];
  double d_std;
  double d_avg;
  float  f_pr_err;
  float  f_dr_err;
} gnss_PrDrGroup_t;

typedef struct
{
  GpsTime_t               z_tor;
  gnss_ConstellationType  u_constellation;            /** see gnss_ConstellationType */
  uint8_t                 u_svid;                     /** satellite index of the system */
  uint8_t                 u_signal;                   /** see gnss_SignalType */
  uint8_t                 u_freq_idx;                 /**  */
  /** bit0: PR valid status; 
      bit1: DR valid status;
      bit2: PR RTD status;   
      bit3: CP valid status */
  uint8_t                 u_status;                   
  int32_t                 q_cycleSlipCount;           /** cycle slip count */
  uint32_t                q_measStatusMask;           /** see gnss_MeasureStatusType */
  double                  d_tot;                      /** time of transmit */
  double                  d_range;                    /** geometric range between satellite and receiver antennas, used in KF for real range */
  double                  d_pseudoRange_filter;       /** pseudorange corrected by satellite clock, ion, trop and so on  */
  double                  d_doppler_filter;           /** doppler corrected by satellite clock drift and so on */
  float                   f_pr_residual_pre;          /** pseudorange pre residual, f_pr_diff */
  float                   f_pr_residual_post;         /** pseudorange post residual, f_post_prdiff */
  float                   f_dr_residual_pre;          /** doppler pre residual */
  float                   f_dr_residual_post;         /** doppler post residual */
  double                  d_pr_unc;                   /** pseudorange compute unc, unit: meter*/
  double                  d_dr_unc;                   /** doppler compute unc, unit: m/s */
  gnss_PrMaskType         w_pr_mask;                  /** ChiSq test, pre valid, post valid, ajust clock, unc valid, cluster A, dcbValid*/
  gnss_DrMaskType         w_dr_mask;
  gnss_PrDeWeightMaskType q_pr_deweight_mask;
  gnss_PrDeWeightMaskType q_dr_deweight_mask;
  uint8_t                 u_pr_deweight_cnt;          /** count of pseudorange check */
  uint8_t                 u_dr_deweight_cnt;          /** count of pseudorange check */
  gnss_PrDrGroup_t        z_prdr;
} gnss_MeasQos_t;

/** pack from PVT to PPP */
typedef struct
{
  gnss_PositionFix_t z_positionFix;                          /** pvt pos solution */
  uint8_t            u_meas_count;
  gnss_MeasQos_t     pz_meas_qos[MAX_GNSS_TRK_MEAS_NUMBER]; /** meas residual,flag,slip... */
} gnss_PVTResult_t;

typedef struct {
  GpsTime_t             z_tot;         /** time of satellite signal transmit (tot) */
  gnss_ConstellationType u_constellation;
  uint8_t               u_svid;
  gnss_SatPosVelClk_t   z_satPosVelClk;
  uint8_t               u_eclips;      /** eclips satellite */
  float                 f_gf[MAX_GNSS_SIGNAL_FREQ];
  float                 f_mw[MAX_GNSS_SIGNAL_FREQ];
  double                d_wetMap;      /** tropospheric map of wet part*/
  double                d_dryMap;      /** tropospheric map of dry part*/
  double                d_Iono;        /** Ionospheric correction, m  */
  double                d_Tropo;       /** Tropospheric correction, m */
  gnss_SignalMeas_t     pz_signalMeas[MAX_GNSS_SIGNAL_FREQ];
  gnss_MeasQos_t        pz_measQos[MAX_GNSS_SIGNAL_FREQ];    /** pvt quality */
} gnss_TightSatelliteMeas_t;

#define VERSION_GNSS_TIGHT_SAT_SIG_MEAS_COLLECT  (2)
typedef struct {
  uint8_t                   u_version;
  GpsTime_t                 z_tor;         /** Time of measurement receive (tor) */
  int32_t                   q_clk_jump;    /** receiver clock jump (ms) */
  gnss_PositionFix_t        z_positionFix; /** pvt pos solution */
  double                    d_ZTD;         /** tropospheric empirical delaly */
  double                    d_zhd_emp;     /** tropospheric empirical delaly of dry part */
  double                    d_zwd_emp;     /** tropospheric empirical delaly of wet part */
  uint8_t                   u_satMeasCount;
  gnss_TightSatelliteMeas_t pz_tightSatMeas[MAX_GNSS_ACTIVE_SAT_NUMBER];
} gnss_TightSatSigMeasCollect_t;

#define PVA_NUM  (9)

/* KF status for DCP */
#define DCP_KF_INIT         0x01  /* KF is to init by DCP results */
#define DCP_KF_RESET        0x02  /* KF is to reset PVA*/
#define DCP_KF_RUN          0x04  /* KF is running */

/*the configuration for DCP model*/
typedef struct
{
  algo_useConstellation z_usedSys;                /* define constellation used in the DCP algorithm */
  uint8_t               u_enableTDDRorTDPR;       /* 0 close the TDDR or TDPR and 1 open the TDDR or TDPR */
  uint8_t               u_enableDcpPosSmooth;     /* 0 close the dcp position smooth and 1 open the dcp position smooth */
  uint8_t               u_enableCalCurrentSatPos; /* 0 close the routeline of satellite position calculating for current epoch and 1 open the routeline of satellite position calculating for current epoch */
  uint8_t               u_enableFeedbackMeasToINS;/*0 close the function of feedback measure to INS and 1 open the function of feedback measure to INS*/
  gnss_FreqType         z_optimalFreqType;        /*the optimal frequency type*/
  float                 f_eleCutOff;              /* elevation mask angle (deg) */
  BOOL                  pq_isSeparateBDS2And3[C_GNSS_FREQ_TYPE_MAX];/*whether BDS2 and BDS3 are separate*/
} gnss_DcpConfigOpt_t;

/*the information used by DCP position smooth*/
typedef struct
{
  uint8_t   u_kfStatus;                       /* the state of dcp position filter*/
  uint8_t   u_fixFlag;                        /* see: gnss_FixFlagType */
  float     f_deltaT;                         /* delta time between observation time and filter time*/
  GpsTime_t z_filterTime;                     /* filter time*/
  float     pf_X[PVA_NUM];                    /* the PVA parameter */
  float     pf_P[PVA_NUM * (PVA_NUM + 1) / 2];/* the convariance of PVA parameter */
  double    pd_initPos[3];                    /* the site coordinate offset used by DCP position smooth */
} gnss_DcpPosSeq_t;

#define VERSION_GNSS_TDCP_MEAS_BLOCK  (0)
/*the satellite information used by DCP*/
typedef struct
{
  uint8_t   u_valid;           /** mask the valid of satellite */
  uint8_t   u_isCalculateDist; /** mask wether calculated the distance of site to satellite */
  gnss_ConstellationType   u_constellation;   /** see gnss_ConstellationType */
  uint8_t   u_svid;            /** satellite index of the system */
  float     f_elevation;       /** the satellite elevation,unit in deg */
  float     f_azimuth;         /** the satellite azimuth,unit in deg */
  int32_t   q_iode;            /** ephemeris iode */
  double    pd_satPosClk[4];   /** Satellite Position X,Y,Z and Clock Bias */
  double    pd_satVelClk[4];   /** Satellite Velocity X,Y,Z and Clock Drift */
  double    d_sat2SiteDist;    /** the distance from satellite to site, ignore earth rotion correction */
  double    pd_site2SatUnit[3];/** the unit vector from site to satellite */
} gnss_DcpSatInfo_t;

/*the epoch information used by DCP*/
#define VERSION_TDCP_MEAS_BLOCK  (0)
typedef struct
{
  uint8_t            u_version;                             /*version number*/
  uint16_t           w_size;                                /*size of this struct*/
  GpsTime_t          z_obsTime;                             /*observation time*/
  gnss_PositionFix_t z_posSol;                              /*the position solution*/
  uint16_t           w_satNum;                              /*satellite number*/
  uint16_t           w_measNum;                             /*the number of observations*/
  gnss_DcpSatInfo_t  pz_satInfo[MAX_GNSS_ACTIVE_SAT_NUMBER];/*the satellite information used by DCP*/
  GnssMeas_t         pz_meas[MAX_GNSS_TRK_MEAS_NUMBER];     /*observations*/
  GnssMeasExt_t      pz_measExt[MAX_GNSS_TRK_MEAS_NUMBER];  /*extern observations*/
} gnss_TdcpMeasBlock_t;

/******************************************************************************
    GNSS SSR Loc Block Structure Definition Begin
******************************************************************************/

/* observation data records number*/
#define GNSS_SSR_SAT_NUM_LIMIT                        45
/* max number of channel for one satellite*/
#define GNSS_SSR_SAT_CHL_NUM_LIMIT                    3

/* Satellites broadcast ephemeris, orbit and clock mask */
typedef enum
{
  /* ephemeris is valid */
  GNSS_SSR_SAT_ORB_CLK_EPH = 0x01,

  /* orbit correction is calculated */
  GNSS_SSR_SAT_ORB_CLK_ORBIT_CORR = 0x02,

  /* clock correction is calculated */
  GNSS_SSR_SAT_ORB_CLK_CLOCK_CORR = 0x04,

  /* precision oribit, relative to EPH dx/dy/dz */
  GNSS_SSR_SAT_ORB_CLK_PRE_ORBIT = 0x08,

  /* precision clock, absolute */
  GNSS_SSR_SAT_ORB_CLK_PRE_CLOCK = 0x10,
  
} gnss_ssrSatOrbClkMaskVal;
typedef uint8_t gnss_ssrSatOrbClkMask;

/* Satellites bias mask */
typedef enum
{
  /* code bias correction is calculated */
  GNSS_SSR_SAT_BIAS_CODE_CORR = 0x01,

  /* phase bias correction is calculated */
  GNSS_SSR_SAT_BIAS_PHASE_CORR = 0x02
} gnss_ssrSatBiasMaskVal;
typedef uint8_t gnss_ssrSatBiasMask;

/* Atmosphere mask */
typedef enum
{
  /* STEC correction is calculated */
  GNSS_SSR_ATMO_STEC_CORR = 0x01,

  /* STD correction is calculated */
  GNSS_SSR_ATMO_STD_CORR = 0x02,

  /* ZTD correction is calculated */
  GNSS_SSR_ATMO_ZTD_CORR = 0x04
} gnss_ssrAtmoMaskVal;
typedef uint8_t gnss_ssrAtmoMask;

/* Error model mask */
typedef enum
{
  /* solid tide correction is calculated */
  GNSS_SSR_ERROR_MODEL_SOLID_TIDE_CORR = 0x01,

  /* ocean tide correction is calculated */
  GNSS_SSR_ERROR_MODEL_OCEAN_TID_CORR = 0x02,

  /* pole tide is calculated */
  GNSS_SSR_ERROR_MODEL_POLE_TIDE_CORR = 0x04
} gnss_ssrErrorModelMaskVal;
typedef uint8_t gnss_ssrErrorModelMask;

typedef enum
{
  /*gravitaional delay is corrected in orb_clk LOS*/
  GNSS_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR = 0x01,

  /*BDS2 satellite pseudorange multipath is corrected in code bias(2I/7I/6I) LOS*/
  GNSS_SSR_ERROR_MODEL_SAT_BDS_MULTIPATH_CORR = 0x02,

  /*satellite wind up correction is calculated and valid*/
  GNSS_SSR_ERROR_MODEL_SAT_WIND_UP_VALID = 0x04,

  /*satellite attitude Yaw angle and Yaw rate is calculated and valid*/
  GNSS_SSR_ERROR_MODEL_SAT_YAW_INFO_VALID = 0x08
} gnss_ssrSatErrorModelMaskVal;
typedef uint8_t gnss_ssrSatErrorModelMask;

/* ZTD correction data with quality indicator and integrity */
typedef struct
{
  gnss_integrityFlag     u_preItg;   /* pre-check integrity */
  gnss_integrityFlag     u_postItg;  /* post-check integrity */
  float                  f_qi;       /* quality indicator, unit: m */
  double                 d_wetCorr;  /* wet part correction data value */
  double                 d_dryCorr;  /* dry part correction data value */
} gnss_ZtdCorrData_t;

/* Correction data with quality indicator and integrity */
typedef struct
{
  gnss_integrityFlag     u_preItg;   /* pre-check integrity */
  gnss_integrityFlag     u_postItg;  /* post-check integrity */
  /* QI unit for different SSR corrections refer to following:
   * orb_clk: mm
   * code bias/phase bias/ZTD/STD: m
   * STEC: TECU */
  float                  f_qi;       /* quality indicator */
  int32_t                q_corr;     /* correction data value,enlarge 1000 for the origin*/
} gnss_SsrCorrBias_t;

/* Correction data of one signal */
typedef struct
{
  gnss_SignalType        u_signalType;           /* Follow gnss_SignalType */
  gnss_ssrSatBiasMask    u_biasMask;             /* flag of code or phase corrected indicator */
  float                  f_pcv;                  /* Phase centre variation data */
  uint8_t                u_discontinuityIod;     /* This IOD indicates whether there has been a discontinuity in the phase bias estimation process */
  gnss_SsrCorrBias_t     z_codeBias;             /* satallite code correction data */
  gnss_SsrCorrBias_t     z_phaseBias;            /* satallite phase correction data */
} gnss_SignalCorr_t;

/* Correction data with quality indicator and integrity for satellite precision orbit and clock */
typedef struct
{
  gnss_integrityFlag     u_preItg;       /* pre-check integrity */
  gnss_integrityFlag     u_postItg;      /* post-check integrity */
  float                  f_qi;           /* quality indicator, mm */
  int32_t                q_delta_pos[3]; /* Orbit position correction dxyz at the orbit reference time, mm*/
  int32_t                q_clock_bias;   /* Full precise satellite clock bias at the clock bias reference time, mm*/
} gnss_SsrCorrPrecision_t;

/* Line of sight data of one satellite */
typedef struct
{
  gnss_ConstellationType   u_constellation;        /* see gnss_ConstellationType */
  uint8_t                   u_svid;                 /* satellite index of the system */
  GpsTime_t                 z_STECtime;             /* STEC correction data time */
  GpsTime_t                 z_STDtime;              /* STD correction data time */
  GpsTime_t                 z_orbClkTime;           /* correction data time for the orbit/clock SSR data */
  gnss_ssrSatOrbClkMask     u_orbClkMask;           /* flag of satellite orbit or clock corrected indicator */
  gnss_ssrAtmoMask          u_atmoMask;             /* flag of atmosphere corrected indicator */
  gnss_ssrSatErrorModelMask u_windUpMask;           /* flag of satellite wind up corrected indicator */
  uint8_t                   u_signalNum;            /* signal number of one satellite */
  int32_t                   q_iode;                 /* ephemeris iode */
  gnss_SatPosVelClk_t       z_satPosVelClkBrdc;     /* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
  uint8_t                   u_clockContinuityIod;   /* IOD to indicate whether there has been a discontinuity in clock estimation */
  gnss_SsrCorrPrecision_t   z_orbclkPrec;           /* correction for the orbit/clock correction delta with ephemeris */
  gnss_SsrCorrBias_t        z_orbClk;               /* correction for the orbit/clock SSR data */
  gnss_SsrCorrBias_t        z_stec;                 /* correction for the STEC SSR data */
  gnss_SsrCorrBias_t        z_std;                  /* correction for the STD SSR data */
  double                    d_windUp;               /* correction of satellite wind up */
  float                     f_yaw;                  /* yaw angle of satellite, unit in degree */
  float                     f_yawRate;              /* yaw rate of satellite, unit in degree per second */
  gnss_SignalCorr_t         z_signalBiasCorr[GNSS_SSR_SAT_CHL_NUM_LIMIT];/* all signal data for one satellite */
} gnss_satSsrLos_t;

/* Line of sight data of station */
typedef struct
{
  GpsTime_t              z_tor;           /* observation time */
  GpsTime_t              z_ZTDtime;       /* ZTD correction data time */
  gnss_ssrAtmoMask       u_ZTDmask;       /* flag of ZTD corrected indicator */
  gnss_ssrErrorModelMask u_errorModelMask;/* flag of error model corrected indicator */
  gnss_ZtdCorrData_t     z_ZTDcorr;       /* correction for the ZTD SSR data */
  uint8_t                u_satCount;      /* satellite count */
  gnss_satSsrLos_t       z_satLosCorr[GNSS_SSR_SAT_NUM_LIMIT];/* all satellite correction data for current epoch */
} gnss_epochSsrLos_t;

#define VERSION_GNSS_SSR_LOS_BLOCK  (4)
typedef struct
{
  uint8_t                u_version;
  uint16_t               w_size;
  double                 d_siteCoor[3];  /* the site initial position of current epoch*/
  gnss_epochSsrLos_t     z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} gnss_ssrLosBlock_t;

/******************************************************************************
    GNSS SSR Loc Block Structure Definition End
******************************************************************************/

typedef struct {        /* earth rotation parameter data type */
  double mjd;           /* mjd (days) */
  double xp, yp;        /* pole offset (rad) */
  double xpr, ypr;      /* pole offset rate (rad/day) */
  double ut1_utc;       /* ut1-utc (s) */
  double lod;           /* length of day (s/day) */
} gnss_Erpd_t;

typedef struct {        /* earth rotation parameter type */
  int n;                /* number of data */
  int nmax;             /* number number of data */
  gnss_Erpd_t* data;    /* earth rotation parameter data */
} gnss_Erp_t;

/*The type of fixed ambiguity*/
typedef uint8_t gnss_fixedAmbType;
#define GNSS_NONE_AMB_FIXED   ((uint8_t)0x00)
#define GNSS_L1_AMB_FIXED     ((uint8_t)0x01)
#define GNSS_L2_AMB_FIXED     ((uint8_t)0x02)
#define GNSS_L3_AMB_FIXED     ((uint8_t)0x04)
#define GNSS_N1_AMB_FIXED     ((uint8_t)0x08)
#define GNSS_W1_AMB_FIXED     ((uint8_t)0x10)
#define GNSS_W2_AMB_FIXED     ((uint8_t)0x20)
#define GNSS_UW_AMB_FIXED     ((uint8_t)0x40)
#define GNSS_MAX_AMB_FIXED_TYPE     ((uint8_t)0x41)

#define GNSS_NL_AMB_FIXED     (GNSS_L1_AMB_FIXED|GNSS_L2_AMB_FIXED|GNSS_L3_AMB_FIXED)
#define GNSS_WL_AMB_FIXED     (GNSS_W1_AMB_FIXED|GNSS_W2_AMB_FIXED|GNSS_UW_AMB_FIXED)

/* The type of prefixed ambiguity flag */
#define GNSS_AMB_FLAG_NONE        0x0
#define GNSS_AMB_FLAG_FLOAT       0x1
#define GNSS_AMB_FLAG_FIXED       0x2
#define GNSS_AMB_FLAG_PREFIXED    0x3
#define GNSS_AMB_FLAG_TOSWITCH    0x4  //flag of switching amb when ref station change

typedef struct
{
  gnss_ConstellationType     u_constellation;
  uint8_t                    u_svid;
  uint32_t                   u_continue_fix[MAX_GNSS_SIGNAL_FREQ];
  gnss_SignalType            u_freqType[MAX_GNSS_SIGNAL_FREQ];
  gnss_SignalType            u_twinFreqType;
  gnss_fixedAmbType          u_ambFixType;
  uint8_t                    u_fix[MAX_GNSS_SIGNAL_FREQ]; /* ambiguity fix flag (1:float, 2:fixed, 3:prefix) */
  double                     d_fixedValue;
  double                     d_fixedValueRTK[MAX_GNSS_SIGNAL_FREQ];
} gnss_fixedSignalAmb_t;

typedef struct
{
  GpsTime_t         z_time;
  gnss_fixedAmbType u_fixStatus;    /* GNSS_W1_AMB_FIXED, GNSS_L1_AMB_FIXED... */
  uint8_t           u_ambfix_mode;  /* LAMBDA_FULL, LAMBDA_QOS ... */
  uint8_t           pw_refSat[MAX_GNSS_SIGNAL_FREQ * C_GNSS_MAX];
  uint32_t          w_continuefixedEpoch;
  uint8_t           u_fixedns[C_GNSS_MAX];
  uint8_t           u_fixedTotalSatCount;  /* number of fixed satellite */
  uint8_t           u_nb;           /* number of fixed bias */
  uint8_t           u_gap;          /* whether 2nd best sol & best sol has gap,2 for large, 1 for small, 0 for not */
  uint8_t           u_ppp_use_sat_num_wideLane; /* sat num used in ppp wideLane */
  uint8_t           u_ppp_use_sat_num_fix; /* sat num used in ppp fix */
  float             f_ratio;
  double            f_lambda_sigma[2]; /* LAMBDA sum of squared residual, 0:best, 1: second best */
  float             f_adop;
  float             f_pdop;
  float             f_cn0Thres;
  float             f_eleThres;
  double*           pd_x_fix;
  double*           pd_q_fix;
  gnss_fixedSignalAmb_t* pz_fixedAmbSet[ALL_GNSS_SYS_SV_NUMBER];
} gnss_fixedSignalAmbPool_t;

/* the initilization information of parameters in the filter */
typedef struct
{
  uint8_t   u_resetPara;  /* wether or not reset the parameter in the filter */
  GpsTime_t z_paraTime;   /* the time of added parameter to the filter */
  double    d_initValue;  /* the initilization value of parameter */
  double    d_sigma0;     /* the sigma of parameter */
  double    d_noise;      /* the noise of parameter */
} gnss_filterInitInfo_t;

typedef struct
{
  char gga[256];
  char rmc[256];
  char zda[256];
  char gst[256];
  char oly[256];
  char eapei[256];
  char accur[256];
  int8_t gsa_num;
  char gsa[10][256];
  char gsv_num;
  char gsv[24][256];
} gnss_NmeaInfo_t;


#ifndef ENABLE_MULTI_FREQ
#define GNSS_MAX_FILTER_STATE_NUM (100)
#else
#define GNSS_MAX_FILTER_STATE_NUM (150)
#endif

#define PVA_NUM  (9)
#define PV_NUM   (6)
#define P_NUM    (3)
#define NUTM(n)     ((n + 1)*n / 2)   //The number of lower triangle elements in symmetric matrix
#define IUTM(i, j)  (j >= i ? ((j + 1)*j / 2 + i) : ((i + 1)*i / 2 + j)) //symmetric matrix

/** The type of GNSS measurement that feedback to INS */
typedef enum {
  GNSS_FEEDBACK_NONE         = 0x00,
  GNSS_FEEDBACK_TIME_DIFF_CP = 0x01,
  GNSS_FEEDBACK_TIME_DIFF_PR = 0x02,
  GNSS_FEEDBACK_TIME_DIFF_DR = 0x04,
  GNSS_FEEDBACK_PR_DOPPLER   = 0x08,
} gnss_FeedbackInsMeasTypeVal;
typedef uint8_t gnss_FeedbackInsMeasType;

/* the maximum satellite number of GNSS feedback to INS*/
#define MAX_FEEDBACK_INS_MEAS_NUM        (40)

/* The measurement unit type of GNSS feedback to INS */
typedef struct
{
  /** Measurement constellation, see gnss_ConstellationType */
  gnss_ConstellationType  u_constellation;
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
    If u_FeedbackMeasType = GNSS_FEEDBACK_PR_DOPPLER, difference obs and res are invalid.
    If u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_PR, difference obs and res are from pseudo range.
    If u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_DR, difference obs and res are from doppler.
    If u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_CP, difference obs and res are from carrier phase.
  */
  float                  f_epoch_diff_obs;
  float                  f_epoch_diff_res;
  /** The observation of current epoch.
    If u_FeedbackMeasType = GNSS_FEEDBACK_PR_DOPPLER, pseudorange and doppler are valid.
    If u_FeedbackMeasType = GNSS_FEEDBACK_TIME_DIFF_XX, pseudorange and doppler are invalid.
  */
  double                 d_pseudorange;
  float                  f_doppler;
} gnss_FeedbackInsMeasUnit_t;

/* The measurement block that GNSS feedback to INS */
#define VERSION_GNSS_FEEDBACK_INS_MEAS_BLOCK  (0)
typedef struct
{
  uint8_t   u_version;
  GpsTime_t z_GpsTime;    /* The time of epoch as GpsTime format */
  uint8_t   u_CurFixFlag; /* Current Epoch Fix Flag, see: gnss_FixFlagType */
  uint8_t   u_PreFixFlag; /* Previous Epoch Fix Flag, see: gnss_FixFlagType */
  uint8_t   u_FreqType;   /* Frequecy Type see: gnss_FreqType */
  double    d_CurLLA[3];  /* Current Epoch Fix Position in LLA */
  double    d_PreLLA[3];  /* Previous Epoch Fix Position in LLA */
  float     f_QuasiGeoidHeight; /* Quasi-geoid height */
  float     f_interval;   /* The interval between current epoch and previous epoch*/
  uint8_t   u_FeedbackMeasType; /** the type of GNSS measurment, see: gnss_FeedbackInsMeasType */
  uint8_t   u_MeasCount;  /* The number of epoch-difference observations data */

  /* The satellite observation blcok that GNSS feedback to INS */
  gnss_FeedbackInsMeasUnit_t z_GnssFeedbackInsMeasUnit[MAX_FEEDBACK_INS_MEAS_NUM];
} gnss_FeedbackInsMeasBlock_t;
#define GNSS_PHASE_CODE_CLK_MAX_BIAS  (100.0)

#define VERSION_INS_IMU_DATA   (0)
typedef struct
{
  uint8_t   u_version;
  uint32_t  q_tow_msec;/*time in ms*/
  float     f_gyro[3];  /* angular rate in rad/s */
  float     f_accl[3];  /* specific force in m/s2 */
  float     f_temperature;/* temperature in celsius */
} ins_ImuData_t;

#define VERSION_INS_WHEEL_DATA  (0)
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
} ins_WheelData_t;

#define VERSION_INS_GNSS_FIX  (1)
typedef struct {
  uint8_t              u_version;
  uint64_t             t_timestamp;      /* time (local) in milliseconds */
  uint64_t             t_timeStampOfUtc; /* time translated from UTC time */
  double               d_lat;            /* Latitude in degree */
  double               d_lon;            /* Longitude in degree*/
  float                f_alt;            /* Ellipsodial height */
  float                f_speed;          /* meters/sec */
  float                f_heading;        /* deg */
  float                f_pitch;          /* deg */
  float                f_hdop;
  float                f_hor_accu;
  float                f_vn;             /* meters/sec */
  float                f_ve;             /* meters/sec */
  float                f_vd;             /* meters/sec */
  float                f_latstd;         /* m */
  float                f_lonstd;         /* m */
  float                f_altstd;         /* m */
  float                f_vnstd;          /* m/s */
  float                f_vestd;          /* m/s */
  float                f_vdstd;          /* m/s */
  double               d_tow;            /* seconds of week in ms*/
  float                f_avgsnr;         /* average CN0 */
  uint16_t             w_week;           /* gps weeknumber */
  uint8_t              u_postype;        /* position src type*/
  uint8_t              u_satused;        /* Number of used sattelites */
  gnss_Integrity_t     z_integrity;      /* Integrity of GNSS */
  //GnssPosFlag_Enum    e_posflag;    /* Position flag */
  //GnssMsgType_Enum    u_msg_type;   /*gnss data type*/
  //float               f_heading_deg; /*heading of dualant*/
  //float               f_heading_std; /*heading std of dualant*/
  //float               f_pitch_deg; /*pitch of dualant*/
  //float               f_pitch_std; /*pitch std of dualant*/
} ins_GnssFix_t;

#define VERSION_INS_EKF_RESULT  (0)

typedef struct {
  uint32_t q_tow_msec;
  float f_temperature;
  float f_gyro[3];
  float f_accl[3];
} ag_imu_data_t;


/* For BDDB decoding type definition */
#define GNSS_STATE_NONE 0             
#define GNSS_STATE_FIXEDPOS 1   
#define GNSS_STATE_FIXEDHEIGHT 2   
#define GNSS_STATE_DOPPLER_VELOCITY 8   
#define GNSS_STATE_SINGLE 16 
#define GNSS_STATE_PSRDIFF 17 
#define GNSS_STATE_WAAS 18 
#define GNSS_STATE_PROPAGATED 19 
#define GNSS_STATE_OMNISTAR 20 
#define GNSS_STATE_L1_FLOAT 32 
#define GNSS_STATE_IONOFREE_FLOAT 33 
#define GNSS_STATE_NARROW_FLOAT 34 
#define GNSS_STATE_L1_INT 48 
#define GNSS_STATE_WIDE_INT 49 
#define GNSS_STATE_NARROW_INT 50 
#define GNSS_STATE_OMNISTAR_HP 64 
#define GNSS_STATE_OMNISTAR_XP 65 
#define GNSS_STATE_PPP_CONVERGING 68 
#define GNSS_STATE_PPP 69 
#define GNSS_STATE_OPERATIONAL 70 
#define GNSS_STATE_WARNING 71 
#define GNSS_STATE_OUT_OF_BOUNDS 72 
#define GNSS_STATE_PPP_BASIC_CONVERGING 77 
#define GNSS_STATE_PPP_BASIC 78 

typedef struct
{
  double Lon_deg;/*Longitude (deg)*/
  double Lat_deg;/*Latitude (deg)*/
  float Alt_m;/*Altitude (m)*/

  float LonStd;
  float LatStd;
  float AltStd;

  float VelN_mps;/* NED north velocity (m/s) */
  float VelE_mps;/* NED east velocity (m/s) */
  float VelD_mps;/* NED down velocity (m/s) */

  float VelNstd;
  float VelEstd;
  float VelDstd;

  float Heading_deg;
  float HeadingStd;

  float Pitch_deg;
  float PitchStd;

  float BaseLineLength_m; 
  float RtkAge_s;

  uint8_t SvNumMst;
  uint8_t SvNumSlv;

  uint8_t FixStatusPos;/*0:fix not available or invalid; 1:GPS fix ;2:C/A differential GPS; 4:RTK fixed; 5:RTK floating*/
  uint8_t FixStatusAtt;/*0:fix not available or invalid; 1:GPS fix ;2:C/A differential GPS; 4:RTK fixed; 5:RTK floating*/

  uint16_t UtcYear;
  uint8_t UtcMon;
  uint8_t UtcDay;
  uint8_t UtcHour;
  uint8_t UtcMin;
  uint8_t  UtcSec;
  uint32_t UtcmSec;

  uint8_t LeapSec;

  float Trk_deg;
  float Hspd_mps;
  float Vspd_mps;
  float SAcc;
  float HAcc;

  float LatencyVel_s;

  double ItowPos;/*sec*/
  double ItowVel;/*sec*/
  double ItowHeading;/*sec*/
  double ItowTime;/*sec*/

  uint32_t  GPSWeek;

  uint8_t DiffDataLinkBroken;
  uint8_t GnssAntInfo;

  uint32_t MsgValidType;

  double SysTime_ms;
  double SyncTime_ms;
  uint8_t   SelfCheck;
 

  double LastHspd;
  double LastVspd;
  uint32_t RecMsgOutput;  
  uint32_t RecMsgEkf;  
  uint8_t MsgType;
  uint8_t PosFlag;
  float MsgDelay;
  float avg_CN0;
  /****/
} GnssFixType;

END_DECL

#endif
