/**@file        fusion_api.h
 * @brief       Vehicle Dead Reckoning
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/06  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_API_H__
#define __FUSION_API_H__


#include <stdint.h>
#include "fusion_global.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MAX_RAW_OBS 40

#define TYPE_WHEEL_AIDING_NHC 0x0
#define TYPE_WHEEL_AIDING_ODO 0x01
#define TYPE_WHEEL_AIDING_TWO_REARS 0x02
#define TYPE_WHEEL_AIDING_FOUR_WHLS 0x04
#define TYPE_WHEEL_AIDING_WHLS_STEER 0x08

#define TYPE_VDR_LC 0x01
#define TYPE_VDR_TC_PRDR 0x02
#define TYPE_VDR_TC_CP 0x04
#define TYPE_VDR_TC 0x08

#define CLOCK_DRIFT_ENABLE 0 /* for TC_PRDR */

typedef enum
{
  TYPE_IMU_DEFAULT = 0,
  TYPE_IMU_AG051 = 1,
  TYPE_IMU_ASM330 = 2
} ImuType_Enum;

typedef enum
{
  TYPE_OUTPUTPOS_DEFAULT = 0,
  TYPE_OUTPUTPOS_GNSS = 1,
  TYPE_OUTPUTPOS_REARMID = 2,
  TYPE_OUTPUTPOS_IMU = 3
} InsOutputposFlag_Enum;

typedef enum
{
  TYPE_RATE_1_HZ = 1,
  TYPE_RATE_5_HZ = 5,
  TYPE_RATE_10_HZ = 10,
  TYPE_RATE_20_HZ = 20,
  TYPE_RATE_25_HZ = 25,
  TYPE_RATE_50_HZ = 50,
  TYPE_RATE_100_HZ = 100,
  TYPE_RATE_200_HZ = 200,
  TYPE_RATE_400_HZ = 400,
  TYPE_RATE_MAX_HZ = 400
} AsensingRate_Enum;

typedef enum
{
  TYPE_POS_FLAG_UNAVAILABLE = 0,
  TYPE_POS_FLAG_GNSSONLY = 1,
  TYPE_POS_FLAG_DGNSS = 2,
  TYPE_POS_FLAG_RTK_FIX = 4,
  TYPE_POS_FLAG_RTK_FLOAT = 5,
  TYPE_POS_FLAG_FUSION = 6,
  TYPE_POS_FLAG_HW = 8,
  TYPE_POS_FLAG_MAX
} GnssPosFlag_Enum;

typedef enum
{
  NOVATEL_RECMGS_BESTPOS = 0x00000001u,
  NOVATEL_RECMGS_BESTVEL = 0x00000002u,
  NOVATEL_RECMGS_HEADING = 0x00000004u,
  NOVATEL_RECMGS_BESTXYZ = 0x00000008u,
  NOVATEL_RECMGS_TIME = 0x00000010u,
  NOVATEL_RECMGS_TIMESYNC = 0x00000020u,
  NOVATEL_RECMGS_OTHER = 0x00000040u,

  GNSS_BESTPOS_BESTVEL = 0x00000003u,
  GNSS_BESTPOS_BESTVEL_HEADING = 0x00000007u
} GnssMsgType_Enum;

typedef enum
{
  TYPE_DR_POS_FLAG_UNAVAILABLE = 0,
  TYPE_DR_POS_FLAG_MIS_EST = 1,
  TYPE_DR_POS_FLAG_IN_MOTION = 2,
  TYPE_DR_POS_FLAG_FUSION = 6,
  TYPE_DR_POS_FLAG_DR_OLY = 7,
  TYPE_DR_POS_FLAG_MAX
} DrPosFlag_Enum;

typedef enum
{
  ASG_TIME_DIFF_NONE = 0x00,
  ASG_TIME_DIFF_CARRIER = 0x01,
  ASG_TIME_DIFF_PSEUDO = 0x02,
  ASG_TIME_DIFF_DOPPLER = 0x04    
} GnssTdCpMask_Enum;

typedef enum
{
  TYPE_STATUS_FAILURE = -1,
  TYPE_STATUS_SUCCESS,
  TYPE_STATUS_CONFIG_ISSUE,
  TYPE_STATUS_INVALID_PARAMS,
  TYPE_STATUS_INVALID_CONFIG,
  TYPE_STATUS_DATA_READY
} FusionStatus_Enum;

typedef enum
{
  TYPE_GEAR_S = 0x01,
  TYPE_GEAR_W = 0x02,
  TYPE_GEAR_M = 0x03,
  TYPE_GEAR_D = 0x05,
  TYPE_GEAR_N = 0x06,
  TYPE_GEAR_R = 0x07,
  TYPE_GEAR_P = 0x08
} GearShift_Enum;

typedef struct
{
  uint64_t t_timestamp; /* time in ms */
  float f_gyro[3];      /* angular rate in rad/s */
  float f_accl[3];      /* specific force in m/s2 */
  float f_temp;	        /* temperature in celsius */
  uint8_t u_selfck;     /* self check */
} AsensingIMU_t;

typedef struct
{
  uint64_t            t_timestamp;      /* time (local) in milliseconds */
  uint64_t            t_timeStampOfUtc; /* time translated from UTC time */
  double              d_lat;            /* Latitude in degrees */
  double              d_lon;            /* Longitude in degrees */
  float               f_alt;            /* Ellipsodial height */
  float               f_speed;          /* meters/sec */
  float               f_heading;        /* deg */
  float               f_pitch;          /* deg */
  float               f_hdop;
  float               f_hor_accu;
  float               f_vn;             /* meters/sec */
  float               f_ve;             /* meters/sec */
  float               f_vd;             /* meters/sec */
  float               f_latstd;         /* m */
  float               f_lonstd;         /* m */
  float               f_altstd;         /* m */
  float               f_vnstd;	        /* m/s */
  float               f_vestd;	        /* m/s */
  float               f_vdstd;	        /* m/s */
  float               f_lat_pl;         /* Latitude  protection level, m */
  float               f_lon_pl;         /* Longitude protection level, m */
  float               f_alt_pl;         /* Altitude  protection level, m */
  uint8_t             u_lat_itg;        /* Latitude  integrity flag, 0:OK, 1:not OK, 2:not monitored */
  uint8_t             u_lon_itg;        /* Longitude integrity flag, 0:OK, 1:not OK, 2:not monitored*/
  uint8_t             u_alt_itg;        /* Altitude  integrity flag, 0:OK, 1:not OK, 2:not monitored*/
  double              d_tow;            /* seconds of week in ms */
  float               f_avgsnr;	        /* average CN0 */
  uint16_t            w_week;           /* gps weeknumber */
  uint8_t             u_postype;        /* position src type -- 1: gnss pos&vel std valid and gnss 3-dimensional vel(i.e. f_vn/f_ve/f_vd) valid, 0:other case */
  uint8_t             u_satused;        /* Number of used sattelites */
  GnssPosFlag_Enum    e_posflag;        /* Position flag */
  GnssMsgType_Enum    u_msg_type;       /* gnss data type */
  float               f_heading_deg;    /* heading of dualant */
  float               f_heading_std;    /* heading std of dualant */
  float               f_pitch_deg;      /* pitch of dualant */
  float               f_pitch_std;      /* pitch std of dualant */
  uint16_t            UtcYear; 
  uint8_t             UtcMon; 
  uint8_t             UtcDay; 
  uint8_t             UtcHour; 
  uint8_t             UtcMin; 
  uint8_t             UtcSec; 
  uint16_t            UtcmSec; 
  uint8_t             LeapSec; 
} AsensingGNSSPara_t;

typedef struct
{
  double d_pr;	       /* satellite pseudorange measuremnt */
  double d_dr;	       /* satellite doppler measuremnt */
  double d_sat_pos[3]; /* satellite position */
  double d_sat_vel[3]; /* satellite velocity */
  float  f_prnoise;    /* pseudorange measuremnt variance */
  float  f_drnoise;    /* doppler measuremnt variance */
  float  f_ele;	       /* elevation of the satellite in deg */
  float  f_azi;        /* azimuth of the satellite in deg */
  uint8_t u_prn;	   /* satellite number */
  uint8_t u_sys;	   /* satellite system */
  uint8_t u_snr;	   /* signal strength */
  uint8_t u_valid;     /* bit 0: psr  bit 1: dop */
} AsensingSatPsrRaw_t;

typedef struct
{
  double d_cur_tow;
  double d_blh[3];
  GnssPosFlag_Enum e_posflag;
  uint8_t u_count;
  AsensingSatPsrRaw_t z_raw_data[MAX_RAW_OBS];
} AsensingGNSSPsrMeas_t;

typedef struct
{
  float  f_tdcp_obs;       /* tdcp measuremnt */
  float  f_tdcp_res;       /* tdcp measuremnt variance */
  double d_cur_sat_pos[3]; /* cur satellite position */
  double d_pre_sat_pos[3]; /* pre satellite position */
  float  f_ele;	           /* elevation of the satellite in deg */
  float  f_azi;            /* azimuth of the satellite in deg */
  uint8_t u_prn;	       /*  satellite number */
  uint8_t u_sys;	       /* satellite system */
  uint8_t u_snr;	       /* signal strength */
  uint8_t u_valid;         /* 0x00: invalid, 0x01: valid */
} AsensingSatCpRaw_t;

typedef struct
{
  double d_cur_tow;
  double d_blh[3];
  GnssPosFlag_Enum e_posflag;
  uint8_t u_count;
  GnssTdCpMask_Enum e_diff_mask;
  AsensingSatCpRaw_t z_raw_cp_data[MAX_RAW_OBS];
} AsensingGNSSCpMeas_t;

typedef struct
{
  uint64_t t_timestamp;
  uint32_t q_fl_whpulse;
  uint32_t q_fr_whpulse;
  uint32_t q_rl_whpulse;
  uint32_t q_rr_whpulse;
  float f_angle_front;
  float f_angle_rear;
  float f_odometer;
  GearShift_Enum e_gear;
  uint8_t u_selfck;
  uint64_t t_shifter_ms;
  uint64_t t_angle_ms;
} AsensingWhpulse_t;

typedef struct
{
  uint64_t t_timestamp; /* time (local) in milliseconds */
  uint16_t w_valid;
} AsensingPPS_t;

typedef struct
{
  uint64_t        t_timestamp; /* ms */
  DrPosFlag_Enum  e_drposflag;
  float           f_pos_longitudinal_pl;
  float           f_pos_lateral_pl;
  float           f_pos_hpl;
  float           f_pos_vpl;
  float           f_pos_north_pl;
  float           f_pos_east_pl;
  float           f_vel_longitudinal_pl;
  float           f_vel_lateral_pl;
  float           f_vel_hpl;
  float           f_vel_vpl;
  float           f_vel_north_pl;
  float           f_vel_east_pl;
  float           f_roll_pl;
  float           f_pitch_pl;
  float           f_yaw_pl;
} INSIntegrity_t;

/* The measurement block that GNSS feedback to INS */
#define VERSION_VDR_POSITION_FIX_REPORT  (0)
typedef struct
{
  uint8_t                  u_version;
  uint64_t                 t_timestamp;        /* ms */
  double                   d_latitude;         /* rad */
  double                   d_longitude;        /* rad */
  float                    f_altitude;         /* meters */
  float                    f_heading;          /* degrees */
  float                    f_speed;            /* meters/sec */
  float                    f_vn;               /* meters/sec */
  float                    f_ve;               /* meters/sec */
  float                    f_vd;               /* meters/sec */
  float                    f_roll;             /* degrees */
  float                    f_pitch;            /* degrees */
  float                    f_yaw;              /* degrees */
  float                    f_gyrobias_x;       /* degrees/s */
  float                    f_gyrobias_y;       /* degrees/s */
  float                    f_gyrobias_z;       /* degrees/s */
  float                    f_accbias_x;        /* meters/sec2 */
  float                    f_accbias_y;        /* meters/sec2 */
  float                    f_accbias_z;        /* meters/sec2 */
  float                    f_horpos_std;       /* meters */
  float                    f_lat_std;          /* meters */
  float                    f_lon_std;          /* meters */
  float                    f_alt_std;          /* meters */
  float                    f_vn_std;           /* meters/sec */
  float                    f_ve_std;           /* meters/sec */
  float                    f_vd_std;           /* meters/sec */
  float                    f_roll_std;         /* deg */
  float                    f_pitch_std;        /* deg */
  float                    f_yaw_std;          /* deg */
  float                    f_mis_roll;         /* deg */
  float                    f_mis_pitch;        /* deg */
  float                    f_mis_yaw;          /* deg */
  uint32_t                 q_tow;              /* in ms */
  float                    f_whlspd_sf[4];
  uint16_t                 w_week;
  DrPosFlag_Enum           e_drposflag;
  uint8_t                  u_zuptflag;         /* 0:moving 1:static */
  uint8_t                  f_imuaxis_flag;
  uint8_t                  u_axis_orient;
  uint8_t                  u_misb2v_convergeflag;
  float                    f_misb2v_converge[3];
  uint32_t                 q_kfmeastype;
  INSIntegrity_t           z_inspl;            /* ins protection level */
  float                    f_la_imu2gnss[3];
  float                    f_la_imu2rearmid[3];
  float                    f_la_rear2rear;
  float                    f_misdualant_roll;  /* deg */
  float                    f_misdualant_pitch; /* deg */
  float                    f_misdualant_yaw;   /* deg */
} INSResults_t;

typedef struct
{
  uint64_t t_timestamp;
  uint64_t t_timestamp_utc; /* time translated from UTC time */
  double   d_latitude;      /* degree */
  double   d_longitude;     /* degree */
  float    f_altitude;      /* meters */
  float    f_roll;          /* degree */
  float    f_pitch;         /* degree */
  float    f_heading;       /* degree */
  float    f_speed;         /* meters/sec */
  float    f_vn;
  float    f_ve;
  float    f_vd;
  GnssPosFlag_Enum e_posflag;
} AsensingFusionResults_t;

typedef struct
{
  uint8_t u_validType;
  double d_lat;        /* imu latitude (deg) */
  double d_lon;        /* imu longitude (deg) */
  float f_alt;         /* imu altitude (m) */
  float f_vn;          /* imu north speed (m/s) */
  float f_ve;          /* imu east speed (m/s) */
  float f_vd;          /* imu down speed (m/s) */
  float f_roll;        /* imu roll (deg) */
  float f_pitch;       /* imu pitch (deg) */
  float f_yaw;         /* imu yaw (deg) */
  float f_mis_roll;    /* deg */
  float f_mis_pitch;   /* deg */
  float f_mis_yaw;     /* deg */
  uint8_t u_v_axis_mode;
  uint8_t u_h_axis_mode;
} HistoryInsResults_t;

typedef struct
{
  void (*fusion_log)(uint8_t* buf, uint32_t len);

  void (*fusion_report)(INSResults_t* pz_InsPositionFix);

  void (*fusion_report_history)(HistoryInsResults_t* pHistoryResults);
} AsensingFusionApiCallback_t;

typedef struct
{
  uint8_t q_vdr_mode;        /* VDR algo mode, 0x01: LC, 0x02: TC_PRDR, 0x04: TC_CP ... */
  uint8_t q_align_mode;
  uint8_t  u_whspd_mode;
  uint8_t  u_outputpos_flag; /* 1:output to GNSS center, 2:output to rearmid, 3:output at imu center */
  uint8_t  u_dr_time;
  uint8_t  u_imu_type;       /* 1:AG051, 2:ASM330 */
  AsensingRate_Enum e_gnss_rate;
  AsensingRate_Enum e_sampling_rate;
  AsensingRate_Enum e_output_rate;
  AsensingRate_Enum e_whspd_rate;
} AsensingFusionInitPara_t;

char* asensing_fusion_version(void);
void asensing_fusion_inject_imu(AsensingIMU_t* pz_imu);
void asensing_fusion_inject_wheel(AsensingWhpulse_t* pz_wheel);
void asensing_fusion_inject_pps(AsensingPPS_t* pz_pps);
void asensing_fusion_inject_history(HistoryInsResults_t* pz_history);
void asensing_lvrm_set_gnss2imu(float f_gnss2imu[3]);
void asensing_lvrm_set_rearmid2imu(float f_rearmid2imu[3]);
void asensing_lvrm_set_rear2front(float f_rear2front);
void asensing_lvrm_set_tworears(float f_2rear);
void asensing_mis_set_imu2dualant(float f_euler[3]);
void asensing_mis_set_imu2vehicle(float f_euler[3]);
void asensing_fusion_inject_gnss_para(AsensingGNSSPara_t* pz_gnss_para);
void asensing_fusion_inject_gnss_psr_meas(AsensingGNSSPsrMeas_t* pz_gnss_meas);
void asensing_fusion_inject_gnss_cp_meas(AsensingGNSSCpMeas_t* pz_gnss_cp_meas);
int8_t asensing_fusion_rt_handler(void);
void asensing_fusion_bg_handler(void);
int8_t asensing_fusion_init(AsensingFusionInitPara_t* pz_init_para, AsensingFusionApiCallback_t* pz_api_cb);

#ifdef __cplusplus
}
#endif

#endif