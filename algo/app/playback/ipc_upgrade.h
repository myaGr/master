/**@file        ipc_upgrade.h
 * @brief       ipc upgrade header file
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/10/30  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __IPC_UPGRADE_H__
#define __IPC_UPGRADE_H__


#include "cmn_def.h"
#include "gnss_type.h"

BEGIN_DECL

BOOL ipc_upgrade_sm_Init(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_Start(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_Stop(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_Release(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_RcvMeasRTCM_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_RefCorrRTCM_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_PostionFix_Report(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_SsrLocBlock_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_RcvMeas_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_SsrQianXunStream_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_SsrGeeSpaceStream_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_sm_RcvTwinMeasRTCM_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_Init(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_Start(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_Stop(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_Release(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_RcvMeas_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_RefCorr_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_Ephemeris_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_RcvMeasRTCM_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_RefCorrRTCM_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_hpp_RcvMeasTwin_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_Init(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_Start(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_Stop(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_Release(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_RcvMeas_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_0_To_1(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_1_To_2(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_2_To_3(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_SsrLocBlock_Put_From_3_To_4(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_TightSatSigMeas_Put_From_0_To_1(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_TightSatSigMeas_Put_From_1_To_2(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_SsrLocBlock_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_TightSatSigMeasCollect_Put(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_Algorithm_Start(ipc_t* pz_src_ipc);
BOOL ipc_upgrade_ppp_SsrQianXunStream_Put(ipc_t* pz_src_ipc);

/**
 * @brief Upgrade injected ipc to current latest ipc
 * @param[in]   pz_src_ipc - pointer to legacy  IPC version message
 * @return      TRUE - be upgraded, the p_data is new alloc data
 *              FALSE - not be upgraded
 */
BOOL ipc_upgrade(ipc_t* pz_src_ipc);

BOOL package_upgrade(uint16_t w_PackageId, uint8_t** pp_data, uint32_t* pq_length);

/******************************************************************************/
/* Version 0 of Maximum receiver track measurement number */
#define MAX_GNSS_TRK_MEAS_NUMBER_0      (88)
#define MAX_GNSS_TRK_MEAS_NUMBER_1      (120)

typedef enum
{
  GNSS_0_INTEGRITY_FLAG_MONITORED_OK = 0,
  GNSS_0_INTEGRITY_FLAG_MONITORED_FAIL,
  GNSS_0_INTEGRITY_FLAG_NOT_MONITORED,
} gnss_integrityFlagVal_0;
typedef uint8_t gnss_integrityFlag_0;

typedef struct
{
  gnss_integrityFlag  u_flag;               /* integrity flag */
  float               f_protection_level;   /* protection level, m */
}gnss_ProtectionLevel_t_0;

/** Integrity in multiple coordinate system */
typedef struct
{
  gnss_ProtectionLevel_t_0        z_pos_longitudinal_pl;   /* Position longitudinal */
  gnss_ProtectionLevel_t_0        z_pos_lateral_pl;        /* Position lateral  */
  gnss_ProtectionLevel_t_0        z_pos_north_pl;          /* Position North */
  gnss_ProtectionLevel_t_0        z_pos_east_pl;           /* Position East */
  gnss_ProtectionLevel_t_0        z_pos_hor_pl;            /* Position Horizontal */
  gnss_ProtectionLevel_t_0        z_pos_ver_pl;            /* Position Vertical */
  gnss_ProtectionLevel_t_0        z_vel_longitudinal_pl;   /* Velocity longitudinal */
  gnss_ProtectionLevel_t_0        z_vel_lateral_pl;        /* Velocity lateral  */
  gnss_ProtectionLevel_t_0        z_vel_north_pl;          /* Velocity North */
  gnss_ProtectionLevel_t_0        z_vel_east_pl;           /* Velocity East */
  gnss_ProtectionLevel_t_0        z_vel_hor_pl;            /* Velocity Horizontal */
  gnss_ProtectionLevel_t_0        z_vel_ver_pl;            /* Velocity Vertical */
  gnss_ProtectionLevel_t_0        z_roll_pl;               /* Roll */
  gnss_ProtectionLevel_t_0        z_pitch_pl;              /* Pitch */
  gnss_ProtectionLevel_t_0        z_yaw_pl;                /* Yaw */
}gnss_Integrity_t_0;

/* Version 0 of GPS Time Types, start at 1980-01-06 00:00:00 */
typedef struct {
  uint32_t      q_towMsec; /* MilliSeconds of the week, unit: 1ms */
  uint32_t      q_subNsec; /* Sub NanoSeconds of a MilliSeconds, unit: 1ns */
  uint16_t      w_week;    /* Week Number since GPS time start point */
  uint64_t      t_fullMsec;  /* Full Millisecond start at UTC 1970-01-01 00:00:00, unit: 1ms */
} GpsTime_t_0;

/* Version 0 of Receiver GNSS Clock structure */
typedef struct {
  GpsTime_t_0   z_gpsTime;    /** Receiver time using GPST */
  double        d_clockBias;  /** Receiver clock bias GPS L1 based */
  double        d_clockDrift; /** Receiver clock drift GPS L1 based */
} GnssClock_t_0;

/* Version 0 of GNSS Measurement structure */
typedef struct {
  gnss_MeasStatusFlag_t z_measStatusFlag;/** Measurement status flags */
  gnss_ConstellationType u_constellation;/** Measurement constellation, see gnss_ConstellationType */
  uint8_t       u_svid;                  /** Measurement satellite index of the iteself system */
  uint8_t       u_signal;                /** Measurement signal type, see gnss_SignalType */
  uint8_t       u_LLI;                   /** Loss of lock indicator */
  float         f_cn0;                   /** Carrier-to-noise density */
  double        d_pseudoRange;           /** Pseudo-range */
  double        d_doppler;               /** Doppler */
  double        d_carrierPhase;          /** Carrier phase */
} GnssMeas_t_0;

/* Version 0 of GNSS Measurement structure extension */
typedef struct {
  uint8_t       u_multiPath;             /** Flag to Multi-path */
} GnssMeasExt_t_0;

/* Version 0 of Receiver Measurement Block structure */
typedef struct {
  uint8_t         u_version;                            /** Structure version */
  uint16_t        w_size;                               /** Structure size */
  GnssClock_t_0   z_Clock;                              /** Measurement clock */
  uint16_t        w_measCount;                          /** Measurement count */
  GnssMeas_t_0    z_meas[MAX_GNSS_TRK_MEAS_NUMBER_0];     /** Measurement array */
  GnssMeasExt_t_0 z_measExt[MAX_GNSS_TRK_MEAS_NUMBER_0];  /** Extension measurement array */
} GnssMeasBlock_t_0;

/* Version 1 of Receiver Measurement Block structure */
typedef struct
{
  uint8_t         u_version;                            /** Structure version */
  uint16_t        w_size;                               /** Structure size */
  GnssClock_t_0   z_Clock;                              /** Measurement clock */
  uint16_t        w_measCount;                          /** Measurement count */
  GnssMeas_t_0    z_meas[MAX_GNSS_TRK_MEAS_NUMBER_1];     /** Measurement array */
  GnssMeasExt_t_0 z_measExt[MAX_GNSS_TRK_MEAS_NUMBER_1];  /** Extension measurement array */
} GnssMeasBlock_t_1;

/******************************************************************************/

/** Enum for all the supported GNSS Constellation types */
typedef enum
{
  C_GNSS1_GPS = 0,
  C_GNSS1_GLO,
  C_GNSS1_BDS,
  C_GNSS1_GAL,
  C_GNSS1_QZS,
  C_GNSS1_MAX
} gnss_Constellation1TypeEnumVal;

typedef struct {
  float         f_gdop;
  float         f_pdop;
  float         f_hdop;
  float         f_vdop;
  float         f_tdop;
} gnss_PositionFix_Dops_t_0;

typedef struct {
  gnss_ConstellationType u_constellation;/** see gnss_ConstellationType */
  uint8_t       u_svid;         /** satellite index of the system */
  uint8_t       u_signal;       /** see gnss_SignalType */
  float         f_cn0;
  float         f_elevation;    /** rad */
  float         f_azimuth;      /** rad */
} gnss_PositionFixSV_t_0;

typedef struct {
  uint8_t      u_SvValidCount;
  uint8_t      u_SvInUseCount;
  uint64_t     t_SvValidMask[C_GNSS1_MAX];
  uint64_t     t_SvInUseMask[C_GNSS1_MAX];
  gnss_PositionFixSV_t_0 z_SV[MAX_GNSS_TRK_MEAS_NUMBER_0];
} gnss_PositionFix_SatStatus_t_0;

typedef struct {
  uint8_t      u_SvValidCount;
  uint8_t      u_SvInUseCount;
  uint8_t      u_svTracked;
  uint8_t      w_svFixed;
  uint64_t     t_SvValidMask[C_GNSS1_MAX];
  uint64_t     t_SvInUseMask[C_GNSS1_MAX];
  gnss_PositionFixSV_t_0 z_SV[MAX_GNSS_TRK_MEAS_NUMBER_0];
} gnss_PositionFix_SatStatus_t_1;

typedef struct
{
  uint8_t      u_SvValidCount;
  uint8_t      u_SvInUseCount;
  uint8_t      u_svTracked;
  uint8_t      w_svFixed;
  uint64_t     t_SvValidMask[C_GNSS1_MAX];
  uint64_t     t_SvInUseMask[C_GNSS1_MAX];
  gnss_PositionFixSV_t_0 z_SV[MAX_GNSS_TRK_MEAS_NUMBER_1];
} gnss_PositionFix_SatStatus_t_2;

typedef struct
{
  uint8_t      u_SvValidCount;
  uint8_t      u_SvInUseCount;
  uint8_t      u_svTracked;
  uint8_t      w_svFixed;
  uint64_t     t_SvValidMask[C_GNSS_MAX];
  uint64_t     t_SvInUseMask[C_GNSS_MAX];
  gnss_PositionFixSV_t_0 z_SV[MAX_GNSS_TRK_MEAS_NUMBER_1];
} gnss_PositionFix_SatStatus_t_3;

typedef struct {
  uint32_t      q_StationID;
  float         f_age;          /* different age (s) */
} gnss_PositionFix_RTK_t_0;

typedef struct {
  uint32_t      q_StationID;
  float         f_age;          /* different age (s) */
  double        pd_StationCoordinate[3];  /* Station Coordinate of VRS */
} gnss_PositionFix_RTK_t_1;

typedef struct
{
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
} gnss_PositionFix_INS_t_0;

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
} gnss_PositionFix_INS_t_1;

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
  float    f_pos_longitudinal_pl;
  float    f_pos_lateral_pl;
  float    f_pos_hpl;
  float    f_pos_vpl;
  float    f_pos_north_pl;
  float    f_pos_east_pl;
  float    f_vel_longitudinal_pl;
  float    f_vel_lateral_pl;
  float    f_vel_hpl;
  float    f_vel_vpl;
  float    f_vel_north_pl;
  float    f_vel_east_pl;
  float    f_roll_pl;
  float    f_pitch_pl;
  float    f_yaw_pl;
  float    f_la_imu2gnss[3];
  float    f_la_imu2rearmid[3];
  float    f_la_rear2rear;
  float    f_misdualant_roll;     /* deg */
  float    f_misdualant_pitch;    /* deg */
  float    f_misdualant_yaw;      /* deg */
} gnss_PositionFix_INS_t_2;

typedef struct {
  uint8_t  u_drposflag;
  float    f_roll;                /* degree */
  float    f_pitch;               /* degree */
  float    f_heading;             /* degree */
  float    f_gyrobias_x;          /* degree/second */
  float    f_gyrobias_y;          /* degree/second */
  float    f_gyrobias_z;          /* degree/second */
  float    f_accbias_x;           /* meter/second^2 */
  float    f_accbias_y;           /* meter/second^2 */
  float    f_accbias_z;           /* meter/second^2 */
  float    f_roll_std;            /* degree */
  float    f_pitch_std;           /* degree */
  float    f_yaw_std;             /* degree */
  float    f_mis_roll;            /* degree */
  float    f_mis_pitch;           /* degree */
  float    f_mis_yaw;             /* degree */
  float    f_speed;               /* meter/sec */
  float    f_whlspd_sf[4];
  uint32_t q_kfmeastype;
  float    f_la_imu2gnss[3];
  float    f_la_imu2rearmid[3];
  float    f_la_rear2rear;
  float    f_misdualant_roll;     /* deg */
  float    f_misdualant_pitch;    /* deg */
  float    f_misdualant_yaw;      /* deg */
} gnss_PositionFix_INS_t_3;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  uint8_t       u_fixSource;    /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;      /* see: gnss_FixFlagType */
  GpsTime_t     z_gpsTime;
  double        d_xyz[3];       /* Position: X,Y,Z in ECEF (m) */
  double        d_lla[3];       /* Position: lat, lon, alt */
  float         f_velXyz[3];    /* Velocity: X,Y,Z in ECEF (m/s) */
  float         f_velEnu[3];    /* Velocity: lat, lon, alt */
  float         f_posXyzUnc[3]; /* Position uncertainy (m) */
  float         f_velEnuUnc[3]; /* Velocity uncertainy (m/s) */
  double        d_clockBias;    /* Primary constellation clock bias */
  double        d_clockDrift;   /* Primary constellation clock drift */
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_SatStatus_t_0 z_SvStatus;
  gnss_PositionFix_RTK_t_0       z_rtk;
} gnss_PositionFix_t_0;

typedef struct {
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0        z_dops;
  gnss_PositionFix_SatStatus_t_1   z_SvStatus;
  gnss_PositionFix_RTK_t_0         z_rtk;
} gnss_PositionFix_t_1;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_SatStatus_t_1 z_SvStatus;
  gnss_PositionFix_RTK_t_0       z_rtk;
  gnss_PositionFix_INS_t_0       z_ins;
} gnss_PositionFix_t_2_0;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_SatStatus_t_2 z_SvStatus;
  gnss_PositionFix_RTK_t_0       z_rtk;
  gnss_PositionFix_INS_t_0       z_ins;
} gnss_PositionFix_t_2_1;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_0       z_rtk;
  gnss_PositionFix_INS_t_0       z_ins;
  gnss_PositionFix_SatStatus_t_1 z_SvStatus;
} gnss_PositionFix_t_2_2;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_0       z_rtk;
  gnss_PositionFix_INS_t_0       z_ins;
  gnss_PositionFix_SatStatus_t_2 z_SvStatus;
} gnss_PositionFix_t_2_3;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_0       z_rtk;
  gnss_PositionFix_INS_t_0       z_ins;
  gnss_PositionFix_SatStatus_t_2 z_SvStatus;
} gnss_PositionFix_t_3;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_1       z_rtk;
  gnss_PositionFix_INS_t_0       z_ins;
  gnss_PositionFix_SatStatus_t_2 z_SvStatus;
} gnss_PositionFix_t_4;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_1       z_rtk;
  gnss_PositionFix_INS_t_1       z_ins;
  gnss_PositionFix_SatStatus_t_2 z_SvStatus;
} gnss_PositionFix_t_5;

typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_1       z_rtk;
  gnss_PositionFix_INS_t_2       z_ins;
  gnss_PositionFix_SatStatus_t_2 z_SvStatus;
} gnss_PositionFix_t_6;


typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_1       z_rtk;
  gnss_PositionFix_INS_t_2       z_ins;
  gnss_PositionFix_SatStatus_t_3 z_SvStatus;
} gnss_PositionFix_t_7;

/** ADD f_posLlaPL */
typedef struct
{
  uint8_t       u_version;
  uint16_t      w_size;
  GpsTime_t     z_gpsTime;
  EpochTime_t   z_epoch;             /* Epoch UTC year,month,day,hour,minute,second */
  uint8_t       u_leapsec;           /* Leap second */
  uint8_t       u_DcpPosType;        /* DCP position type, see: gnss_DcpType */
  uint8_t       u_fixSource;         /* see: gnss_FixSourceType */
  uint8_t       u_fixFlag;           /* see: gnss_FixFlagType */
  uint8_t       u_CN040;             /* Sat number fre1 cn0>40  */
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
  gnss_Integrity_t_0             z_integrity;         /* Integrity of GNSS or INS */
  gnss_PositionFix_Dops_t_0      z_dops;
  gnss_PositionFix_RTK_t_1       z_rtk;
  gnss_PositionFix_INS_t_3       z_ins;
  gnss_PositionFix_SatStatus_t_3 z_SvStatus;
} gnss_PositionFix_t_8;

/******************************************************************************/
/* observation data records number*/
#define GNSS_SSR_SAT_NUM_LIMIT_0        (45)
/* max number of channel for one satellite*/
#define GNSS_SSR_SAT_CHL_NUM_LIMIT_0 (10)

typedef struct {
  GpsTime_t_0          z_gpsTime;
  uint8_t              u_valid;
  gnss_ConstellationType u_constellation;/** see gnss_ConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  float                f_elevation;    /** rad */
  float                f_azimuth;      /** rad */
  double               d_satPosClk[4]; /** Satellite Position X,Y,Z and Clock Bias */
  double               d_satVelClk[4]; /** Satellite Velocity X,Y,Z and Clock Drift */
  float                f_Iono;         /** Ionospheric correction, m  TODO(houxiaowei): no use, need to delete */
  float                f_Tropo;        /** Tropospheric correction, m TODO(houxiaowei): no use, need to delete */
} gnss_SatPosVelClk_t_0;

/* ZTD correction data with quality indicator and integrity */
typedef struct {
  gnss_integrityFlag     u_preItg;   /* pre-check integrity */
  gnss_integrityFlag     u_postItg;  /* post-check integrity */
  float                  f_qi;       /* quality indicator, unit: m */
  double                 d_wetCorr;  /* wet part correction data value */
  double                 d_dryCorr;  /* dry part correction data value */
} gnss_ZtdCorrData_t_0;

/* Correction data with quality indicator and integrity */
typedef struct {
  gnss_integrityFlag     u_preItg;   /* pre-check integrity */
  gnss_integrityFlag     u_postItg;  /* post-check integrity */
  /* QI unit for different SSR corrections refer to following:
   * orb_clk: mm
   * code bias/phase bias/ZTD/STD: m
   * STEC: TECU */
  float                  f_qi;       /* quality indicator */
  int32_t                q_corr;     /* correction data value,enlarge 1000 for the origin*/
} gnss_SsrCorrBias_t_0;

/* Correction data of one signal */
typedef struct {
  gnss_SignalType        u_signalType;           /* Follow gnss_SignalType */
  gnss_ssrSatBiasMask    u_biasMask;             /* flag of code or phase corrected indicator */
  uint8_t                u_phaseBiasContinueFlag;/* flag of phase bias continued indicator */
  gnss_SsrCorrBias_t_0   z_codeBias;             /* satallite code correction data */
  gnss_SsrCorrBias_t_0   z_phaseBias;            /* satallite phase correction data */
} gnss_SignalCorr_t_0;

/* Line of sight data of one satellite */
typedef struct {
  gnss_ConstellationType  u_constellation;   /* see gnss_ConstellationType */
  uint8_t                 u_svid;            /* satellite index of the system */
  gnss_ssrSatOrbClkMask   u_orbClkMask;      /* flag of satellite orbit or clock corrected indicator */
  gnss_ssrAtmoMask        u_atmoMask;        /* flag of atmosphere corrected indicator */
  gnss_ssrSatErrorModelMask u_windUpMask;      /* flag of satellite wind up corrected indicator */
  uint8_t                 u_signalNum;       /* signal number of one satellite */
  int32_t                 q_iode;            /* ephemeris iode */
  gnss_SatPosVelClk_t_0   z_satPosVelClkBrdc;/* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
  gnss_SsrCorrBias_t_0    z_orbClk;          /* correction for the orbit/clock SSR data */
  GpsTime_t               z_orbClkTime;      /* correction data time for the orbit/clock SSR data */
  gnss_SsrCorrBias_t_0    z_stec;            /* correction for the STEC SSR data */
  gnss_SsrCorrBias_t_0    z_std;             /* correction for the STD SSR data */
  double                  d_windUp;          /* correction of satellite wind up */
  float                   f_yaw;             /* yaw angle of satellite, unit in degree */
  float                   f_yawRate;         /* yaw rate of satellite, unit in degree per second */
  gnss_SignalCorr_t_0     z_signalBiasCorr[GNSS_SSR_SAT_CHL_NUM_LIMIT_0];/* all signal data for one satellite */
} gnss_satSsrLos_t_0;

/* Line of sight data of station */
typedef struct {
  GpsTime_t              z_tor;           /* observation time */
  GpsTime_t              z_STECtime;      /* STEC correction data time */
  GpsTime_t              z_STDtime;       /* STD correction data time */
  GpsTime_t              z_ZTDtime;       /* ZTD correction data time */
  gnss_ssrAtmoMask       u_ZTDmask;       /* flag of ZTD corrected indicator */
  gnss_ssrErrorModelMask u_errorModelMask;/* flag of error model corrected indicator */
  gnss_ZtdCorrData_t_0   z_ZTDcorr;       /* correction for the ZTD SSR data */
  uint8_t                u_satCount;      /* satellite count */
  gnss_satSsrLos_t_0     z_satLosCorr[GNSS_SSR_SAT_NUM_LIMIT_0];/* all satellite correction data for current epoch */
} gnss_epochSsrLos_t_0;

typedef struct {
  uint8_t                u_version;
  uint16_t               w_size;
  double                 d_siteCoor[3];  /* the site initial position of current epoch*/
  gnss_epochSsrLos_t_0   z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} gnss_ssrLosBlock_t_0;

/******************************************************************************/

/* max number of channel for one satellite*/
#define GNSS_SSR_SAT_CHL_NUM_LIMIT_1 (3)

/* Line of sight data of one satellite */
typedef struct {
  gnss_ConstellationType  u_constellation;   /* see gnss_ConstellationType */
  uint8_t                 u_svid;            /* satellite index of the system */
  gnss_ssrSatOrbClkMask   u_orbClkMask;      /* flag of satellite orbit or clock corrected indicator */
  gnss_ssrAtmoMask        u_atmoMask;        /* flag of atmosphere corrected indicator */
  gnss_ssrSatErrorModelMask u_windUpMask;      /* flag of satellite wind up corrected indicator */
  uint8_t                 u_signalNum;       /* signal number of one satellite */
  int32_t                 q_iode;            /* ephemeris iode */
  gnss_SatPosVelClk_t_0   z_satPosVelClkBrdc;/* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
  gnss_SsrCorrBias_t_0    z_orbClk;          /* correction for the orbit/clock SSR data */
  GpsTime_t               z_orbClkTime;      /* correction data time for the orbit/clock SSR data */
  gnss_SsrCorrBias_t_0    z_stec;            /* correction for the STEC SSR data */
  gnss_SsrCorrBias_t_0    z_std;             /* correction for the STD SSR data */
  double                  d_windUp;          /* correction of satellite wind up */
  float                   f_yaw;             /* yaw angle of satellite, unit in degree */
  float                   f_yawRate;         /* yaw rate of satellite, unit in degree per second */
  gnss_SignalCorr_t_0     z_signalBiasCorr[GNSS_SSR_SAT_CHL_NUM_LIMIT_1];/* all signal data for one satellite */
} gnss_satSsrLos_t_1;

/* Line of sight data of station */
typedef struct {
  GpsTime_t              z_tor;           /* observation time */
  GpsTime_t              z_STECtime;      /* STEC correction data time */
  GpsTime_t              z_STDtime;       /* STD correction data time */
  GpsTime_t              z_ZTDtime;       /* ZTD correction data time */
  gnss_ssrAtmoMask       u_ZTDmask;       /* flag of ZTD corrected indicator */
  gnss_ssrErrorModelMask u_errorModelMask;/* flag of error model corrected indicator */
  gnss_ZtdCorrData_t_0   z_ZTDcorr;       /* correction for the ZTD SSR data */
  uint8_t                u_satCount;      /* satellite count */
  gnss_satSsrLos_t_1     z_satLosCorr[GNSS_SSR_SAT_NUM_LIMIT_0];/* all satellite correction data for current epoch */
} gnss_epochSsrLos_t_1;

typedef struct {
  uint8_t                u_version;
  uint16_t               w_size;
  double                 d_siteCoor[3];  /* the site initial position of current epoch*/
  gnss_epochSsrLos_t_1   z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} gnss_ssrLosBlock_t_1;


typedef struct
{
  GpsTime_t_0          z_gpsTime;
  uint8_t              u_valid;
  gnss_ConstellationType  u_constellation;/** see gnss_ConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  float                f_elevation;    /** rad */
  float                f_azimuth;      /** rad */
  float                f_dt;           /** difference time t with first polynomial t*/
  int32_t              q_iode;         /** ephemeris iode */
  double               d_satPosClk[4]; /** Satellite Position X,Y,Z and Clock Bias */
  double               d_satVelClk[4]; /** Satellite Velocity X,Y,Z and Clock Drift */
} gnss_SatPosVelClk_t_1;

/******************************************************************************/

/* Line of sight data of one satellite */
typedef struct {
  gnss_ConstellationType  u_constellation;   /* see gnss_ConstellationType */
  uint8_t                 u_svid;            /* satellite index of the system */
  gnss_ssrSatOrbClkMask   u_orbClkMask;      /* flag of satellite orbit or clock corrected indicator */
  gnss_ssrAtmoMask        u_atmoMask;        /* flag of atmosphere corrected indicator */
  gnss_ssrSatErrorModelMask u_windUpMask;      /* flag of satellite wind up corrected indicator */
  uint8_t                 u_signalNum;       /* signal number of one satellite */
  int32_t                 q_iode;            /* ephemeris iode */
  gnss_SatPosVelClk_t_1   z_satPosVelClkBrdc;/* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
  gnss_SsrCorrBias_t_0    z_orbClk;          /* correction for the orbit/clock SSR data */
  GpsTime_t               z_orbClkTime;      /* correction data time for the orbit/clock SSR data */
  gnss_SsrCorrBias_t_0    z_stec;            /* correction for the STEC SSR data */
  gnss_SsrCorrBias_t_0    z_std;             /* correction for the STD SSR data */
  double                  d_windUp;          /* correction of satellite wind up */
  float                   f_yaw;             /* yaw angle of satellite, unit in degree */
  float                   f_yawRate;         /* yaw rate of satellite, unit in degree per second */
  gnss_SignalCorr_t_0     z_signalBiasCorr[GNSS_SSR_SAT_CHL_NUM_LIMIT_1];/* all signal data for one satellite */
} gnss_satSsrLos_t_2;

/* Line of sight data of station */
typedef struct {
  GpsTime_t              z_tor;           /* observation time */
  GpsTime_t              z_STECtime;      /* STEC correction data time */
  GpsTime_t              z_STDtime;       /* STD correction data time */
  GpsTime_t              z_ZTDtime;       /* ZTD correction data time */
  gnss_ssrAtmoMask       u_ZTDmask;       /* flag of ZTD corrected indicator */
  gnss_ssrErrorModelMask u_errorModelMask;/* flag of error model corrected indicator */
  gnss_ZtdCorrData_t_0   z_ZTDcorr;       /* correction for the ZTD SSR data */
  uint8_t                u_satCount;      /* satellite count */
  gnss_satSsrLos_t_2     z_satLosCorr[GNSS_SSR_SAT_NUM_LIMIT_0];/* all satellite correction data for current epoch */
} gnss_epochSsrLos_t_2;

typedef struct {
  uint8_t                u_version;
  uint16_t               w_size;
  double                 d_siteCoor[3];  /* the site initial position of current epoch*/
  gnss_epochSsrLos_t_2   z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} gnss_ssrLosBlock_t_2;

typedef struct {
  uint8_t                u_version;
  uint16_t               w_size;
  double                 d_siteCoor[3];  /* the site initial position of current epoch*/
  gnss_epochSsrLos_t_2   z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} gnss_ssrLosBlock_t_3; /*the struct is same as gnss_ssrLosBlock_t_2 that because of constellation enum had changed,there will only replace the field u_constellation*/


/******************************************************************************/

/* Correction data of one signal */
typedef struct
{
  gnss_SignalType        u_signalType;           /* Follow gnss_SignalType */
  gnss_ssrSatBiasMask    u_biasMask;             /* flag of code or phase corrected indicator */
  float                  f_pcv;                  /* Phase centre variation data */
  uint8_t                u_discontinuityIod;     /* This IOD indicates whether there has been a discontinuity in the phase bias estimation process */
  gnss_SsrCorrBias_t_0   z_codeBias;             /* satallite code correction data */
  gnss_SsrCorrBias_t_0   z_phaseBias;            /* satallite phase correction data */
} gnss_SignalCorr_t_1;

/* Correction data with quality indicator and integrity for satellite precision orbit and clock */
typedef struct
{
  gnss_integrityFlag     u_preItg;       /* pre-check integrity */
  gnss_integrityFlag     u_postItg;      /* post-check integrity */
  float                  f_qi;           /* quality indicator, mm */
  int32_t                q_delta_pos[3]; /* Orbit position correction dxyz at the orbit reference time, mm*/
  int32_t                q_clock_bias;   /* Full precise satellite clock bias at the clock bias reference time, mm*/
} gnss_SsrCorrPrecision_t_0;

/* Line of sight data of one satellite */
typedef struct {
  gnss_ConstellationType  u_constellation;   /* see gnss_ConstellationType */
  uint8_t                 u_svid;            /* satellite index of the system */
  GpsTime_t               z_STECtime;      /* STEC correction data time */
  GpsTime_t               z_STDtime;       /* STD correction data time */
  GpsTime_t               z_orbClkTime;      /* correction data time for the orbit/clock SSR data */
  gnss_ssrSatOrbClkMask   u_orbClkMask;      /* flag of satellite orbit or clock corrected indicator */
  gnss_ssrAtmoMask        u_atmoMask;        /* flag of atmosphere corrected indicator */
  gnss_ssrSatErrorModelMask u_windUpMask;      /* flag of satellite wind up corrected indicator */
  uint8_t                 u_signalNum;       /* signal number of one satellite */
  int32_t                 q_iode;            /* ephemeris iode */
  gnss_SatPosVelClk_t_1   z_satPosVelClkBrdc;/* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
  uint8_t                 u_clockContinuityIod;  /* IOD to indicate whether there has been a discontinuity in clock estimation */
  gnss_SsrCorrPrecision_t_0 z_orbclkPrec;           /* correction for the orbit/clock correction delta with ephemeris */
  gnss_SsrCorrBias_t_0    z_orbClk;          /* correction for the orbit/clock SSR data */
  gnss_SsrCorrBias_t_0    z_stec;            /* correction for the STEC SSR data */
  gnss_SsrCorrBias_t_0    z_std;             /* correction for the STD SSR data */
  double                  d_windUp;          /* correction of satellite wind up */
  float                   f_yaw;             /* yaw angle of satellite, unit in degree */
  float                   f_yawRate;         /* yaw rate of satellite, unit in degree per second */
  gnss_SignalCorr_t_1     z_signalBiasCorr[GNSS_SSR_SAT_CHL_NUM_LIMIT_1];/* all signal data for one satellite */
} gnss_satSsrLos_t_4;

/* Line of sight data of station */
typedef struct {
  GpsTime_t              z_tor;           /* observation time */
  GpsTime_t              z_ZTDtime;       /* ZTD correction data time */
  gnss_ssrAtmoMask       u_ZTDmask;       /* flag of ZTD corrected indicator */
  gnss_ssrErrorModelMask u_errorModelMask;/* flag of error model corrected indicator */
  gnss_ZtdCorrData_t_0   z_ZTDcorr;       /* correction for the ZTD SSR data */
  uint8_t                u_satCount;      /* satellite count */
  gnss_satSsrLos_t_4     z_satLosCorr[GNSS_SSR_SAT_NUM_LIMIT_0];/* all satellite correction data for current epoch */
} gnss_epochSsrLos_t_4;

typedef struct {
  uint8_t                u_version;
  uint16_t               w_size;
  double                 d_siteCoor[3];  /* the site initial position of current epoch*/
  gnss_epochSsrLos_t_4   z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} gnss_ssrLosBlock_t_4;



/******************************************************************************/


/*gnss_TightSatelliteMeas_t****************************************************/
#define VERSION_GNSS_TIGHT_SAT_SIG_MEAS_COLLECT_IPC_0  0
typedef struct {
  GpsTime_t               z_tot;         /** time of satellite signal transmit (tot) */
  gnss_ConstellationType  u_constellation;
  uint8_t                 u_svid;
  gnss_SatPosVelClk_t_0   z_satPosVelClk;
  uint8_t                 u_eclips;      /** eclips satellite */
  float                   f_gf[MAX_GNSS_SIGNAL_FREQ];
  float                   f_mw[MAX_GNSS_SIGNAL_FREQ];
  double                  d_wetMap;      /** tropospheric map of wet part*/
  double                  d_dryMap;      /** tropospheric map of dry part*/
  double                  d_Iono;        /** Ionospheric correction, m  */
  double                  d_Tropo;       /** Tropospheric correction, m */
  gnss_SignalMeas_t       pz_signalMeas[MAX_GNSS_SIGNAL_FREQ];
  gnss_MeasQos_t          pz_measQos[MAX_GNSS_SIGNAL_FREQ];    /** pvt quality */
} gnss_TightSatelliteMeas_t_0;

typedef struct {
  uint8_t                         u_version;
  GpsTime_t                       z_tor;         /** Time of measurement receive (tor) */
  int32_t                         q_clk_jump;    /** receiver clock jump (ms) */
  gnss_PositionFix_t_0            z_positionFix; /** pvt pos solution */
  double                          d_ZTD;         /** tropospheric empirical delaly */
  double                          d_zhd_emp;     /** tropospheric empirical delaly of dry part */
  double                          d_zwd_emp;     /** tropospheric empirical delaly of wet part */
  uint8_t                         u_satMeasCount;
  gnss_TightSatelliteMeas_t_0     pz_tightSatMeas[MAX_GNSS_ACTIVE_SAT_NUMBER];
} gnss_TightSatSigMeasCollect_t_0;


#define VERSION_GNSS_TIGHT_SAT_SIG_MEAS_COLLECT_IPC_1  1
typedef struct {
  GpsTime_t               z_tot;         /** time of satellite signal transmit (tot) */
  gnss_ConstellationType  u_constellation;
  uint8_t                 u_svid;
  gnss_SatPosVelClk_t_1   z_satPosVelClk;
  uint8_t                 u_eclips;      /** eclips satellite */
  float                   f_gf[MAX_GNSS_SIGNAL_FREQ];
  float                   f_mw[MAX_GNSS_SIGNAL_FREQ];
  double                  d_wetMap;      /** tropospheric map of wet part*/
  double                  d_dryMap;      /** tropospheric map of dry part*/
  double                  d_Iono;        /** Ionospheric correction, m  */
  double                  d_Tropo;       /** Tropospheric correction, m */
  gnss_SignalMeas_t       pz_signalMeas[MAX_GNSS_SIGNAL_FREQ];
  gnss_MeasQos_t          pz_measQos[MAX_GNSS_SIGNAL_FREQ];    /** pvt quality */
} gnss_TightSatelliteMeas_t_1;

typedef struct
{
  uint8_t                         u_version;
  GpsTime_t                       z_tor;         /** Time of measurement receive (tor) */
  int32_t                         q_clk_jump;    /** receiver clock jump (ms) */
  gnss_PositionFix_t_0            z_positionFix; /** pvt pos solution */
  double                          d_ZTD;         /** tropospheric empirical delaly */
  double                          d_zhd_emp;     /** tropospheric empirical delaly of dry part */
  double                          d_zwd_emp;     /** tropospheric empirical delaly of wet part */
  uint8_t                         u_satMeasCount;
  gnss_TightSatelliteMeas_t_1     pz_tightSatMeas[MAX_GNSS_ACTIVE_SAT_NUMBER];
} gnss_TightSatSigMeasCollect_t_1;

#define VERSION_GNSS_TIGHT_SAT_SIG_MEAS_COLLECT_IPC_2  2
typedef struct
{
  uint8_t                         u_version;
  GpsTime_t                       z_tor;         /** Time of measurement receive (tor) */
  int32_t                         q_clk_jump;    /** receiver clock jump (ms) */
  gnss_PositionFix_t_1            z_positionFix; /** pvt pos solution */
  double                          d_ZTD;         /** tropospheric empirical delaly */
  double                          d_zhd_emp;     /** tropospheric empirical delaly of dry part */
  double                          d_zwd_emp;     /** tropospheric empirical delaly of wet part */
  uint8_t                         u_satMeasCount;
  gnss_TightSatelliteMeas_t_1     pz_tightSatMeas[MAX_GNSS_ACTIVE_SAT_NUMBER];
} gnss_TightSatSigMeasCollect_t_2;

/******************************************************************************/

#define VERSION_TDCP_MEAS_BLOCK_IPC_0  0

typedef struct
{
  uint8_t              u_version;                             /*version number*/
  uint16_t             w_size;                                /*size of this struct*/
  GpsTime_t            z_obsTime;                             /*observation time*/
  gnss_PositionFix_t_0 z_posSol;                              /*the position solution*/
  uint16_t             w_satNum;                              /*satellite number*/
  uint16_t             w_measNum;                             /*the number of observations*/
  gnss_DcpSatInfo_t    pz_satInfo[MAX_GNSS_ACTIVE_SAT_NUMBER];/*the satellite information used by DCP*/
  GnssMeas_t           pz_meas[MAX_GNSS_TRK_MEAS_NUMBER];     /*observations*/
  GnssMeasExt_t        pz_measExt[MAX_GNSS_TRK_MEAS_NUMBER];  /*extern observations*/
} gnss_TdcpMeasBlock_t_0;

#define VERSION_TDCP_MEAS_BLOCK_IPC_1  1
typedef struct
{
  uint8_t              u_version;                             /*version number*/
  uint16_t             w_size;                                /*size of this struct*/
  GpsTime_t            z_obsTime;                             /*observation time*/
  gnss_PositionFix_t_1 z_posSol;                              /*the position solution*/
  uint16_t             w_satNum;                              /*satellite number*/
  uint16_t             w_measNum;                             /*the number of observations*/
  gnss_DcpSatInfo_t    pz_satInfo[MAX_GNSS_ACTIVE_SAT_NUMBER];/*the satellite information used by DCP*/
  GnssMeas_t           pz_meas[MAX_GNSS_TRK_MEAS_NUMBER_0];     /*observations*/
  GnssMeasExt_t        pz_measExt[MAX_GNSS_TRK_MEAS_NUMBER_0];  /*extern observations*/
} gnss_TdcpMeasBlock_t_1;

/* Location engine monitor structure version 0 */
typedef struct
{
  char     build_version[256];
  uint32_t q_ReportDuration;
  uint64_t t_StartTime;
  uint64_t t_RunningTime;
  uint64_t t_ReportCount;
  uint64_t t_ReportCountPrevious;
  uint64_t t_HppReportCount;
  uint64_t t_RtkReportCount;
  uint64_t t_PppReportCount;
} loc_MonitorStructType_0;

/******************************************************************************/


/***************************** ins_GnssFix_t Start ******************************/
typedef struct {
  uint8_t       u_version;
  uint64_t      t_timestamp;        /* time (local) in milliseconds */
  uint64_t      t_timeStampOfUtc;   /* time translated from UTC time */
  double        d_lat;              /* Latitude in degree */
  double        d_lon;              /* Longitude in degree*/
  float         f_alt;              /* Ellipsodial height */
  float         f_speed;            /* meters/sec */
  float         f_heading;          /* deg */
  float         f_pitch;            /* deg */
  float         f_hdop;
  float         f_hor_accu;         
  float         f_vn;               /* meters/sec */
  float         f_ve;               /* meters/sec */
  float         f_vd;               /* meters/sec */
  float         f_latstd;           /* m */
  float         f_lonstd;           /* m */
  float         f_altstd;           /* m */
  float         f_vnstd;            /* m/s */
  float         f_vestd;            /* m/s */
  float         f_vdstd;            /* m/s */
  double        d_tow;              /* seconds of week in ms*/
  float         f_avgsnr;           /* average CN0 */
  uint16_t      w_week;             /* gps weeknumber */
  uint8_t       u_postype;          /* position src type*/
  uint8_t       u_satused;          /* Number of used sattelites */
} ins_GnssFix_t_0;

typedef struct {
  uint8_t               u_version;
  uint64_t              t_timestamp;        /* time (local) in milliseconds */
  uint64_t              t_timeStampOfUtc;   /* time translated from UTC time */
  double                d_lat;              /* Latitude in degree */
  double                d_lon;              /* Longitude in degree*/
  float                 f_alt;              /* Ellipsodial height */
  float                 f_speed;            /* meters/sec */
  float                 f_heading;          /* deg */
  float                 f_pitch;            /* deg */
  float                 f_hdop;
  float                 f_hor_accu;         
  float                 f_vn;               /* meters/sec */
  float                 f_ve;               /* meters/sec */
  float                 f_vd;               /* meters/sec */
  float                 f_latstd;           /* m */
  float                 f_lonstd;           /* m */
  float                 f_altstd;           /* m */
  float                 f_vnstd;            /* m/s */
  float                 f_vestd;            /* m/s */
  float                 f_vdstd;            /* m/s */
  double                d_tow;              /* seconds of week in ms*/
  float                 f_avgsnr;           /* average CN0 */
  uint16_t              w_week;             /* gps weeknumber */
  uint8_t               u_postype;          /* position src type*/
  uint8_t               u_satused;          /* Number of used sattelites */
  gnss_Integrity_t_0    z_integrity;        /* Integrity of GNSS */
} ins_GnssFix_t_1;

/***************************** ins_GnssFix_t End   ******************************/

END_DECL

#endif
