#ifndef __GNSS__H__
#define __GNSS__H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "rtklib.h"
#include "gnss_api_def.h"
#include "gnss_types.h"
#include "gnss_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define POS_FRM_NULL            0
#define POS_FRM_LS              1
#define POS_FRM_KF              2

#define MIN_GPS_PRN             1
#define MAX_GPS_PRN             32

#define MIN_GLN_PRN             1
#define MAX_GLN_PRN             32

#define MIN_GAL_PRN             1
#define MAX_GAL_PRN             37

#define MIN_WAAS_PRN            120
#define MAX_WAAS_PRN            138

#define MIN_QZSS_PRN            193
#define MAX_QZSS_PRN            199

#define MIN_BDS_PRN             1
#ifndef UBX_RAW
#define MAX_BDS_PRN             65
#else
#define MAX_BDS_PRN             37
#endif

#define N_GPS_SVS               (MAX_GPS_PRN-MIN_GPS_PRN+1)
#define N_GLN_SVS               (MAX_GLN_PRN-MIN_GLN_PRN+1)
#define N_GAL_SVS               (MAX_GAL_PRN-MIN_GAL_PRN+1)
#define N_BDS_SVS               (MAX_BDS_PRN-MIN_BDS_PRN+1)
#define N_WAAS_SVS              (MAX_WAAS_PRN-MIN_WAAS_PRN+1)  /* Maximum number of valid WAAS C/A codes */
#define N_QZSS_SVS              (MAX_QZSS_PRN-MIN_QZSS_PRN+1)  /* Maximum number of valid QZSS C/A codes */

#define MAX_GPS_MODE_PRN        (N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS)
#define MAX_PRN_ALL_MODE        (MAX_GPS_MODE_PRN + N_GLN_SVS + N_BDS_SVS + N_GAL_SVS)

#define N_MAXSAT                (N_GPS_SVS+N_GLN_SVS+N_BDS_SVS+N_QZSS_SVS+N_GAL_SVS)
#define SQR(x)                  ((x)*(x))

#define RTD_AGE                 (35)
#define MAXLEAPS    64                  /* max number of leap seconds table */

  ///////////////////////////////////
#define ZX_GPS_MAX_PRN          (32)
#define ZX_QZSS_MIN_PRN         (33)
#define ZX_QZSS_MAX_PRN         (37)
#define ZX_GLN_MIN_PRN          (65)
#define ZX_GLN_MAX_PRN          (88)
#define ZX_BDS_MIN_PRN          (151)
#define ZX_BDS_MAX_PRN          (164)

/*GNSS type for data request*/
#define PE_GNSS_GPS             (0x1<<0)
#define PE_GNSS_GLO             (0x1<<1)
#define PE_GNSS_BDS             (0x1<<2)
#define PE_GNSS_GAL             (0x1<<3)
#define PE_GNSS_QZS             (0x1<<4)

/*Assistance data type definition*/
#define PE_REQ_REF_TIME         (0x1<<0)
#define PE_REQ_REF_LOC          (0x1<<1)
#define PE_REQ_GPS_EPH          (0x1<<2)
#define PE_REQ_GPS_IONO         (0x1<<3)
#define PE_REQ_GPS_UTC          (0x1<<4)
#define PE_REQ_GPS_ALM          (0x1<<5)
#define PE_REQ_GLO_EPH          (0x1<<6)
#define PE_REQ_GLO_UTC          (0x1<<7)
#define PE_REQ_GLO_ALM          (0x1<<8)
#define PE_REQ_GLO_AUX          (0x1<<9)
#define PE_REQ_BDS_EPH          (0x1<<10)
#define PE_REQ_BDS_IONO         (0x1<<11)
#define PE_REQ_BDS_UTC          (0x1<<12)
#define PE_REQ_BDS_ALM          (0x1<<13)
#define PE_REQ_GAL_EPH          (0x1<<14)
#define PE_REQ_GAL_ALM          (0x1<<15)
#define PE_REQ_GAL_IONO         (0x1<<16)
#define PE_REQ_GAL_UTC          (0x1<<17)
#define PE_REQ_QZS_EPH          (0x1<<18)
#define PE_REQ_QZS_IONO         (0x1<<19)
#define PE_REQ_QZS_UTC          (0x1<<20)
#define PE_REQ_QZS_ALM          (0x1<<21)

  enum
  {
    GPS_MODE = 0,
    GLN_MODE,
    BDS_MODE,
    GAL_MODE,
    GNSS_MAX_MODE
  };

  typedef struct
  {
    uint8_t        reqFlag[GNSS_MAX_MODE][7];
    uint8_t        reqFlag_Qzss[7];
    int32_t       flag;
    int32_t       gnss;
    int32_t       reqType;
    int64_t       lastRequestTime[GNSS_MAX_MODE][7];
    int64_t       lastRequestTime_Qzss[7];
  } PEReqCommand_t;

  ///////////////////////////////////

  enum
  {
    MEO_SAT = 0,
    GEO_SAT,
    SBS_SAT
  };
  enum
  {
    DYN_UNDEFINED = 0,    /* Undefined dynamics                    */
    DYN_LAND = 1,    /* Land dynamics                         */
    DYN_SEA = 2,    /* Sea dynamics                          */
    DYN_AIR = 3,    /* Air dynamics                          */
    DYN_STATIONARY = 4,    /* Stationary dynamics                   */
    DYN_AUTOMOBILE = 5,    /* Automobile dynamics                   */
    N_DYN = 6     /* Total number of dynamic modes         */
  };

  enum
  {
    WAIT_FIRST = 0,
    WAIT_SECOND = 1,
    WAIT_PROC = 2
  };

#define   BIAS_NUM                     4
  /* Define the maximum GNSS measurements */
#define   MAX_MEAS_NUM                 MAXOBS
#define   LS_PARA_NUM                  (3+BIAS_NUM*3)
#define   DCB_SAT_LIMIT                2

/* Define the system parameters */
#define   LIGHT_SEC                    2.99792458e8
#define   LIGHT_MSEC                   2.99792458e5
#define   LIGHT_NSEC                   2.99792458e-1

#define   GPS_L1_CARRIER               1575.42e6
#define   GPS_L2_CARRIER               1227.60e6
#define   GPS_L5_CARRIER               1176.45e6
#define   GLN_L1_CARRIER(k)            (1602e6+562.5e3 * (k))
#define   GLN_L2_CARRIER(k)            (1246e6+437.5e3 * (k))      
#define   GLN_L3_CARRIER               1202.025e6          
#define   BDS_B1_CARRIER               1561.098e6
#define   BDS_B2A_CARRIER              1176.45e6    
#define   BDS_B2I_CARRIER              1207.14e6  
#define   BDS_B3_CARRIER               1268.52e6           

#define   GAL_E1_CARRIER               1575.42e6
#define   GAL_E5b_CARRIER              1207.14e6
#define   GAL_E5a_CARRIER              1176.45e6

#define   GPS_L1_WAVELENGTH            (LIGHT_SEC/GPS_L1_CARRIER)
#define   GLN_L1_WAVELENGTH(k)         (LIGHT_SEC/GLN_L1_CARRIER(k))
#define   BDS_L1_WAVELENGTH            (LIGHT_SEC/BDS_B1_CARRIER)
#define   GAL_L1_WAVELENGTH            (LIGHT_SEC/GAL_E1_CARRIER)

#define   WGS84_OMEGDOTE                 (double)7.2921151467e-5
#define   A_WGS84                        (double)6378137.0
#define   B_WGS84                        (double)6356752.314245179
#define   F_WGS84                        (double)0.00335281066474
#define   E2_WGS84                       (double)6.69437999013e-3
#define   ONE_MIN_E2                     (double)0.99330562000987
#define   WGS84_SQRT_U                   (double)1.99649818432174e7
#define   GTRF_SQRT_U                    (double)1.99649803856653e7
#define   MAXNIGP                        201                 /* max number of IGP in SBAS band */


#define SECS_IN_HOUR        ((int32_t)3600)
#define SECS_IN_DAY			((int64_t)3600 * 24)
#define SECS_IN_WEEK		((int32_t)SECS_IN_DAY * 7)
#define MSECS_IN_WEEK		((int32_t)SECS_IN_WEEK * 1000)
#define MSECS_IN_DAY		((int32_t)SECS_IN_DAY * 1000)
#define DAY_IN_FOUR_YEAR	(int32_t)1461
#define EARTH_TEMP_1		(double)(((A_WGS84*A_WGS84)-(B_WGS84*B_WGS84))/(A_WGS84*A_WGS84))
#define EARTH_TEMP_2		(double)(((A_WGS84*A_WGS84)-(B_WGS84*B_WGS84))/(B_WGS84*B_WGS84))


#ifndef PI
#define PI					(double)3.14159265358979323846
#endif

#define PI2             ((double)2.0*PI)
#define SC2RAD          3.1415926535898     /* semi-circle to radian (IS-GPS) */

#define GPS_MATH_E                     ((double)2.71828182846)

#define HAVE_POS_NONE     0    /* No position in memory.                */
#define HAVE_POS_INIT     1    /* Position came from outside receiver   */
#define HAVE_POS_APPX     2    /* Position came from NavFilter in coast mode */
#define HAVE_POS_FIX2D    3    /* 2D fix in  2D/3D mode             */
#define HAVE_POS_FIX3D    4    /* 3D fix in  2D/3D mode             */
#define HAVE_POS_OLD      5    /* Fix is considered old but is trusted  */

////////////////////////////////////////////////////////////////////////////////////////////

#define     OSC_BIAS_SET_THRESH     (double)3.59750950e6   /* = (12e-3*LIGHT) Meters */
#define     OSC_BIAS_KNOWN_THRESH   (double)2.99792458e4   /* = (100e-6*LIGHT) Meters */
#define     OSC_BIAS_TEN_MSEC       (double)2.99792458e6   /* = (10e-3*LIGHT) Meters */
#define     OSC_BIAS_ONE_MSEC       (double)2.99792458e5   /* = (1e-3*LIGHT) Meters */

#define G_DIV 0.3

    ////////////////////////////////////////////////////////////
  typedef struct
  {
    uint8_t            status;
    uint8_t            gnssMode;       /* 0: GPS,1:GLN,2:BDS,3:GAL*/
    uint8_t            c_gln;          /* GLONASS EPH change flag */
    uint8_t            tk_gln;         /* tk with GLONASS */
    int16_t           prn;            /* satellite PRN */
    int32_t           iod;            /* issue of data (IOD) */
    double           prc;            /* pseudo-range correction (PRC) (m) */
    double           rrc;            /* range rate correction (RRC) (m/s) */
    double           udre;           /* UDRE */
    gtime_t       time;           /* Most recent correction update time */
  }dgnss_t;

#if 0
  typedef struct
  {
    uint8_t            RtdEnterCnt;
    uint8_t            RtdExitCnt;
    uint8_t            RtdUseFlag : 1;
    uint8_t            RtdDataFull : 1;
    gtime_t       time[GNSS_MAX_MODE];             // most recent RTD message type1/type31
    dgnss_t* dgnss_data[MAX_PRN_ALL_MODE];
  }Rtd_data_t;
#endif
  ////////////////////////////////////////////////////////////////////////////////////////////
  /* ---------------------------------------------------------*\
  | UTC time structure
  |
  | This structure provide basic information of UTC time
  |
  \* ----------------------------------------------------------*/
  typedef struct
  {
    uint16_t     Year;
    uint16_t     Month;
    uint16_t     Day;
    uint16_t     Hour;
    uint16_t     Minute;
    double     Second;

  } UtcTimeType;    // used for PE engine, also we need other utc time format
  /* ---------------------------------------------------------*\
  | Global structure
  |
  | Including: (1) bias; (2) time; (3) time status; (4) drift; (5) drift status
  |                (6) bias_unc; (7) drift_unc and etc
  \* ----------------------------------------------------------*/
  typedef struct
  {
    uint8_t           init : 1;
    uint8_t           isLeapSecondCorrect : 1;
    uint8_t           isLargeTimeJmp : 1;
    uint8_t           isTimeUpdate : 1;
    uint8_t           biasSrc;
    uint8_t           time_convert;
    uint8_t           driftStatus;                                            /* drift status "have_cor" in old structure */
    uint8_t           timeStatus[GNSS_MAX_MODE];                              /* time status */
    uint8_t           timeInitSrc;                                            /* time init source: GROM, AGNSS, BRDC_DECODE, EE */

    uint8_t           leapSecond;                                             /* current leap second number */
    uint8_t           aidLeapSecond;
    uint8_t           torStatus[GNSS_MAX_MODE];
    uint8_t			     dcb_valid_flag[GNSS_MAX_MODE][AGGNSS_MAX_FREQ_NUM + 1];
    uint16_t          N4;                                                     /* only used for GLN (1~ 31)*/
    uint16_t          NT;                                                     /* only used for GLN (1~1461)*/
    uint16_t          N4_save;
    uint16_t          NT_save;
    uint16_t          week[GNSS_MAX_MODE];                                    /* week number for GPS, BDS and GLL */
    uint16_t          week_save[GNSS_MAX_MODE];
    uint16_t          systemWeek;                                             /* week number from network time */
    uint32_t          weekCheckNum[GNSS_MAX_MODE];
    double          msec_correction[GNSS_MAX_MODE];                         /* Ms correction make bias within 1ms */
    double          drift_correction;
    double          bias[GNSS_MAX_MODE];                                    /* bias for four systems, meters */
    double          last_bias[GNSS_MAX_MODE];                                    /* bias for four systems, meters */
    double          dcb[GNSS_MAX_MODE * 2];									 /* dcb for four systems, meters */
    float          bias_unc[GNSS_MAX_MODE];                                /* current bias uncertainty which will be related to time status */
    float          drift;                                                  /* local clock drift, m/s */
    float          drift_unc;                                              /* drift uncertainty, m/s */
    float          drift_acc;
    uint64_t          systemTime;

    double          rcvr_time[GNSS_MAX_MODE];                               /* GPS, GLN,BDS,GAL's tor, in second */
    int64_t          rcvr_time_ns[GNSS_MAX_MODE];
    double          rcvr_time_subNs[GNSS_MAX_MODE];
    int64_t          time_ns;
    int64_t          last_time_ns;

    double          delta_bias;                                             /* Used in toaster mode */
    double          systemTow;                                              /* TOW from network time */
    double          tor;
    double          startTor;
    double          gpsTor;                       // used for MTK
    double          lastGpsTor;                   // used for MTK
    double          time_ns_bias;
    UtcTimeType  utcTime;
    UtcTimeType  aidUtcTime;
    gtime_t      GPSTime;
    gtime_t      GLNTime;
    gtime_t      BDSTime;
    double          dt; //time gap between current epoch and last epoch
  } GNSS_TIME;
  ///////////////////////////////////////////////////////////////////////////////////////////
  typedef struct
  {

    double   alpha_0;          /* Seconds                                  */
    double   alpha_1;          /* Sec. per semicircle                      */
    double   alpha_2;          /* Sec. per semicircle^2                    */
    double   alpha_3;          /* Sec. per semicircle^3                    */
    double   beta_0;           /* Seconds                                  */
    double   beta_1;           /* Sec. per semicircle                      */
    double   beta_2;           /* Sec. per semicircle^2                    */
    double   beta_3;           /* Sec. per semicircle^3                    */
    uint8_t    have_ion;         /* Boolean flag                             */

  } ION_INFO;

  /*----------------------------------------------------------------------*\
  | Time Status Code
  |
  | Status flag has the following valid states. Only the highest state
  | should be considered valid for setting the application's real-time
  | clock.
  \*----------------------------------------------------------------------*/
  enum
  {
    TM_STATUS_NONE = 0,                  /* Time is unknown                         */
    TM_STATUS_APPX,                      /* Time is good to a few seconds      */
    TM_STATUS_TOASTER,                   /* Time is good to +/- 40 msecs      */
    TM_STATUS_SET,                       /* Time is good to +/- 10 msecs      */
    TM_STATUS_ACCU                       /* Time is good to +/- 1 usec         */
  };

  /*----------------------------------------------------------------------*\
  | Time Init Source
  |
  | Source flag for Time Init
  \*----------------------------------------------------------------------*/
  enum
  {
    TM_INIT_NONE = 0,                  /* Time is not init yet */
    TM_INIT_GROM,                      /* Time is init by GROM */
    TM_INIT_AGNSS,                     /* Time is init by AGNSS */
    TM_INIT_BRDC,                      /* Time is init by decoded BRDC */
    TM_INIT_EE,                         /* Time is init by EE */
    TM_INIT_AGNSS_L                     /* Time is init by AGNSS_LITE_QX */
  };

  /*----------------------------------------------------------------------*\
  | Eph Source
  | Source flag for Eph
  \*----------------------------------------------------------------------*/
  enum
  {
    EPH_SOURCE_BRDC = 0,                  /* Eph decoded from BRDC */
    EPH_SOURCE_AGNSS,                     /* Eph aided by AGNSS */
    EPH_SOURCE_EE,                        /* Eph aid by EE */
    EPH_SOURCE_GROM,                      /* Eph load from GROM */
    EPH_SOURCE_AGNSS_L                    /* Eph load from AGNSS_LITE */
  };

  /*----------------------------------------------------------------------*\
  | Drift Status Code
  |
  | Status flag for the oscillator drift/bias
  \*----------------------------------------------------------------------*/
  enum
  {
    DRIFT_UNKNOWN = 0,                /* No information                        */
    DRIFT_OLD,                                /* Old                                   */
    DRIFT_TOASTER_FIX,                 /* Based on toaster fix                  */
    DRIFT_FIX,                                 /* Based on pos/timing fix               */
    DRIFT_APPX,                              /* Approximate 1 SV solution             */
    DRIFT_MEAS,                              /* Have knowledge from raw meas          */
    DRIFT_MEMO,                             /* Have drift from memory                 */
    DRIFT_DERIVE,                           /* derive drift from meas and ppos*/
    DRIFT_ECLK                                /* from external reference clock */
  };

  /*definition for receiver test mode */
  typedef enum
  {
    RCVR_MODE_NORMAL,
    RCVR_MODE_HI_SENSIVITY,
    RCVR_MODE_HI_DYNAMIC,
    RCVR_MODE_TUNNEL,
    RCVR_MODE_3GPP,
    RCVR_MODE_TRACK_SENSIVITY,
    RCVR_MODE_INDOOR,
    RCVR_MODE_ATT
  } RCVR_MODE;
  /* Position Fix Source */
  enum
  {
    FIX_SOURCE_NONE = 0,       /* No position generated                 */
    FIX_SOURCE_INIT,           /* Position came from outside receiver   */
    /* and is approximate.                   */
    FIX_SOURCE_APPX,           /* Position came from outside receiver/Nav Filter Coast mode   */
    /* and is accurate < 100 meters.         */
    FIX_SOURCE_2D,             /* 2D position fix GPS ONLY                      */
    FIX_SOURCE_3D,             /* 3D position fix GPS ONLY                      */
    FIX_SOURCE_OLD,
  };

  /* Position Fix Status */
  enum
  {
    FIX_STATUS_NONE = 0,       /* No fix since power-up or reset        */
    FIX_STATUS_OLD,            /* Position fix is valid but old         */
    /* That is, there's no current position  */
    /* fix, but the last generated position  */
    /* is valid.                             */
    FIX_STATUS_NEW             /* Position fix is valid and new.        */
  };
  enum
  {
    DIM_2D = 0,                /* 2D mode of operation                  */
    DIM_3D,                    /* 3D mode of operation                  */
    DIM_1SV,                   /* 1SV mode of operation                 */
    DIM_AUTO,                  /* Auto mode of operation                */
    DIM_DGPS_REF,              /* DGPS mode of operation                */
    DIM_2D_CLK_HOLD,           /* 2D Clock Hold mode of operation       */
    DIM_OVERDET_CLK            /* Over-determined clock mode           */
  };
#define PR_GOOD         0x1
#define DR_GOOD         0x2
  /* DOP parameters */
  typedef struct
  {
    float
      pDOP,                      /* PDOP                                 */
      hDOP,                      /* HDOP                                 */
      vDOP,                      /* VDOP                                 */
      tDOP;                      /* TDOP                                 */
  } DOP_TYPE;

  typedef struct
  {
    float     dcosx;              /* dir. cosines user to SV            */
    float     dcosy;              /* dir. cosines user to SV            */
    float     dcosz;              /* dir. cosines user to SV            */
    float     east_dcos;          /* EAST transform of dir. cosines     */
    float     north_dcos;         /* NORTH transform of dir. cosines    */
    float     sin_ea;             /* sine of elevation angle            */
    float     elev;               /* elevation angle; Radians           */
    float     az;                 /* azimuth angle; Radians             */
    double     range;              /* SV-to-user range                   */
  } D_COS;

  typedef struct
  {
    float
      p,                      /* Total pos error; meters              */
      h,                      /* Horiz. pos error; meters             */
      v,                      /* Vert. pos error; meters              */
      bias[4],                   /* Time bias error; meters              */
      drift;
    float
      ve_err,                 /* East velocity std error, m/s*/
      vn_err,                 /* north velocity std error, m/s */
      vd_err;                 /* down velocity std error, m/s */
    float
      lat_err,                /* latitude std err, meter */
      lon_err,                /* lontitude std err, meter */
      alt_err;                /* altitude std err, meter */

  } ERR_EST_TYPE;
  ////////////////////////////////////////////////////////////////////////
  /* User ECEF structure */
  typedef  struct
  {
    uint8_t      pos_approx;
    uint8_t      have_position;
    uint16_t     posfix_wn;     // the week of last fix

    float     vel[3];
    double     pos[3];
    double     curr_pos_r;    // distance to earth center
    double     posfix_t;      // the time (in sec) of last fix

  } USER_ECEF;

  /* User LLA structure */
  typedef  struct
  {
    uint8_t     have_position;
    uint16_t    posfix_wn;     // the week of last fix

    float    vel[3];        // ve, vn, vu
    double    pos[3];        // lat, lon, alt
    double    posfix_t;      // the time (in sec) of last fix
  } USER_LLA;

  /* Differential solution status. */
  typedef enum {
    DIFF_SOL_STATUS_NONE,
    DIFF_SOL_STATUS_STANDALONE,
    DIFF_SOL_STATUS_RTD,
    DIFF_SOL_STATUS_RTK_FLOAT,
    DIFF_SOL_STATUS_RTK_FIX,
    DIFF_SOL_STATUS_RTK_PROPAGATE,
    DIFF_SOL_STATUS_EXPIRE,
  } DiffSolutionStatus;

  /* User PVT structure */
  typedef  struct
  {
    uint8_t          pr_res_num;
    uint8_t          dr_res_num;
    uint8_t          cr_res_num;
    uint8_t          prNum_rtk;
    uint8_t          rtdCrDewNum;
    uint8_t          rtdPrDewNum;
    uint8_t 			prNum;
    uint8_t 			prRejNum;
    uint8_t			prDewNum;
    uint8_t			drNum;
    uint8_t 			drRejNum;
    uint8_t			drDewNum;
    uint8_t          svNum_for_fix;
    //uint64_t         pr_used_mask;
    //uint64_t         dr_used_mask;
    float 		smallPrDiffRate;
    float 		prDiffStd;
    float 		usedPrResStd;
    float 		usedDrResStd;
    float 		usedCrResStd;
    float 		cr_rate;
    float 		good_cr_rate;
    float 		fixed_ratio;
    float 		pre_cr_res_std;
    double         posRes;
    double         velRes;
  }MEAS_USE_INFO;

  typedef struct {
    uint8_t          zupt : 1;             /* 1: static, 0: non-static */
    uint8_t          ramp : 2;             /* 0: flat; 1: ramp up; 2:ramp down. */
    uint8_t          mm_used : 2;          /* map matching, 0:no use 1:adopt local 2:adopt server 3:adopt both */
    float         pitch;                /* in degree */
    float         roll;                 /* in degree */
    float         mis_angle_roll;       /* installation angle */
    float         mis_angle_pitch;      /* installation angle */
    float         mis_angle_yaw;        /* installation angle */
    float         gyro_bias[3];         /* gyroscope bias in XYZ coordinate. */
    float         acc_bias[3];          /* accelerometer bias in XYZ coordinate. */
  } flp_result_t;

  /* User PVT structure */
  typedef  struct
  {
    DiffSolutionStatus diff_status;    // If position is SPP/RTD/RTK...
    uint8_t            gpsRtdValid : 1;     // If GPS RTD data is valid
    uint8_t            glnRtdValid : 1;     // If GLN RTD data is valid
    uint8_t            bdsRtdValid : 1;     // If BDS RTD data is valid
    uint8_t            first_pos_flag : 1;
    uint8_t            have_position;       // Position status
    uint8_t            pos_approx;          // approxiate position. 1:true; 0:false
    uint8_t            fom;                 // indicator of position accuracy
    uint8_t            usedSvNum;           // total used satellite number 
    uint8_t            fixedSvNum;           // total fixed satellite number in rtk 
    uint8_t            enterTunnelFlag : 1;
    uint8_t            exitTunnelFlag : 1;
    uint16_t           avgCNO;              // average cno for used satellite
    uint8_t            CN040;
    uint16_t           posfix_wn;           // the week of last fix
    double           posfix_t;            // the time (in sec) of last fix
    float           heading;
    float           headingUnc;          //heading uncertainty/deg
    float           velocity;
    float           velocityUnc;         //horizon speed unc/m/s
    float           accuracy;

    /*   Three parameters of horizontal elliptical uncertainty*/
    float           ellipseUncSemiMajor;
    float           ellipseUncSemiMinor;
    float           ellipseUncOrientation;
    double           diff_age;            /* differential age */
    double           geoidal_sep;         /* Geoidal Separation */

    USER_ECEF     ecef;
    USER_LLA      lla;
    double           altitudeMsl;//Altitude with respect to mean sea level
    DOP_TYPE      DOP;
    ERR_EST_TYPE  posErr;
    gtime_t       time;
    gtime_t       utcTime;
    long long     timeStamp;
    long long     timeStampOfUtc;
    long long     sys_time_stamp;
    /**
    * This flag is only used to form gga(gnssonly/dgnss/dr...).
    * It has limited values in FUSION_TYPE_xxx.
    */
    GnssFusionType  pos_fusion_mode;
    navigation_accuracy_t accu;
    MEAS_USE_INFO   meas_use_info;
    flp_result_t    flp_rslt;
    double quasi_geoid_h;
  } USER_PVT;

  typedef struct
  {
    uint8_t      eph_status;
    uint16_t     sv_health;
    double     dcos[3];
    float     v[3];              /* sv velocity in x,y,z         */
    float     drSv;
    float     fltElev;
    float     fltAz;
    double     p[3];              /* sv position in x,y,z         */
  } SV_INFO;

#define PE_MEAS_VALID_PR  0x1
#define PE_MEAS_VALID_DR  0x2
#define PE_MEAS_VALID_RTD 0x4
#define PE_MEAS_VALID_L1  0x8

  /* Define the RAW measurements structure */
  typedef struct gnss_meas_tag
  {
    uint8_t                 gnssMode;                          /* the satellite system mode */
    uint8_t                 prn;                               /* PRN */
    uint8_t                 status;                            /* bit0: PR valid status;bit1: DR valid status;
                                                          bit2: PR RTD status;bit3:carrierPhase valid status*/
    uint8_t				   freq_index;						  /* 1:L1 2:L2 3:L5*/
    uint8_t                 prDiffValid : 1;
    uint8_t                 isPrChiSqTest : 1;
    uint8_t                 isDrChiSqTest : 1;
    uint8_t                 quality;
    uint8_t                 prChckCnt;
    uint8_t                 drChckCnt;
    uint8_t                 drClusterA;
    uint8_t                 dcbValid : 1;
    uint8_t                 prAdjustFlag;
    uint8_t                 mulit_path; 							// add for st chip spp
    uint16_t                prWeightChck;
    uint16_t                drWeightChck;
    int16_t                code_noise; 							// add for st chip spp
    int16_t                phase_noise; 						// add for st chip spp
    uint32_t                cno;                               /* C/N0 */
    uint32_t                measState;
    float                pr_diff;
    float                dr_diff;
    float                post_prdiff;
    float                post_drdiff;
    float                first_diff;
    int32_t                cycleSlipCount;                      //cycle slip output by baseband
    double                range;                             /* Used in KF for real range */
    double                pseudoRange;                       /* PR */
    double                pseudoRangeRate;                   /* DR */
    double                tot;                               /* time of transit */
    double                prNoise;
    double                drNoise;
    double                stdPRDRDiff;
    double                sumPRDRDiff;
    double                drDiffStd;
    SV_INFO            sv_info;                           /* the satellite position and velocity
                                                          the position and velocity is calculated at tot
                                                          also considering the rotation of earth */
#ifdef USED_IN_MC262M
    int64_t                received_sv_time_in_ns;

    //carrier measurement for rtk
    float                carrierFreqHz;                       //frequency of carrier in HZ, eg. GPS L1=1575.42*(10e6)
#endif
    double                carrierPhase;                        //complete carrier phase measurement in cycles: integer + fractional
    double                pseudoRange_raw;                     //raw pr, should not be changed
    double                pseudoRangeRate_raw;                 //raw dr, should not be changed
  }gnss_meas_t;

  typedef struct
  {
    uint8_t                 rtdUsed : 1;                       /*0: not use RTD; 1: use RTD */
    uint8_t                 hasDiff : 1;                       /*0: invalid; 1: valid */
    uint8_t                 isSmallDiff : 1;
    uint8_t                 invalidMeas : 1;                   /*0: valid meas; 1: invalid meas- wrong tot of bds */
    uint8_t                 biasNum;
    uint8_t                 biasIndx[BIAS_NUM];
    uint16_t                satModeCnt;
    uint8_t				   fre2_satcnt[GNSS_MAX_MODE * 2];
    uint8_t                 Cno35Cnt[GNSS_MAX_MODE];
    uint8_t                 Cno20Cnt;
    uint8_t				   Cno40Cnt;
    uint8_t                 Cno45Cnt;
    uint8_t                 validPrNum;                    /* valid PR number */
    uint8_t                 validDrNum;                    /* valid DR number */
    uint8_t                 validSatnum;                   /* valid sat number */
    uint8_t                 trckedSatNum;                  /* trcked sat num */
    uint8_t                 prNumEachSystem[GNSS_MAX_MODE];
    uint8_t                 prDiff10Cnt;                   /* count of PR DIFF smaller than 10 */
    uint8_t                 prDiff20Cnt;                   /* count of PR DIFF smaller than 20 */
    uint8_t                 prDiff5Cnt;                   /* count of PR DIFF bigger than 5 */
    uint8_t                 prDiffNum;
    uint8_t                 prDiffDistanceAvg10Cnt;
    uint8_t                 satmask[MAX_PRN_ALL_MODE];
    uint8_t         goodPrCnt;
    uint8_t         CheckPrCnt;
    uint32_t                maxCno;
    uint32_t                avgCno;
    uint32_t                maxCno_L5;  /* maxCno of L2 or L5*/
    uint32_t                avgCno_L5;  /* avgCno of L2 or L5*/
    uint32_t                cpNoLLI;    /* carrier phase with no LLI */
    uint32_t                measCnt;
    float                prDiffStd_1; // for downtown mode detection
    float                prDiffStd_2;// for position bias detection
    float                prDiffStd_3[GNSS_MAX_MODE];//for ISB detection
    float                avgPrDiffDis;
    float                prDiffStd_a; // pr diff std of cluster a in Qos
    double                tor;
    double                last_tor;
    double                prdrDiffStd_avg;
    double                msec_adj[GNSS_MAX_MODE];
    double                drift_adj;
    gnss_meas_t        meas[MAX_MEAS_NUM]; /*AGGNSS_MAX_FREQ_NUM*/
    AGGnss_Clock_t      clock;
#if defined(PLAYBACK_MODE)
    double                hpRefEcef[6];  //x,y,z,vx,vy,vz
    double                hpRefLla[6];
    double                hpRefAtt[3];
    uint8_t                 hpRefQflag[2]; //gnss flag, fusion flag
#endif
  } meas_blk_t;


  typedef struct {
    char gga[256];
    char rmc[256];
    char zda[256];
    char gst[256];
    char oly[256];
    char eapei[256];
    char accur[256];
    char gsa_num;
    char gsa[10][256];
    char gsv_num;
    char gsv[24][256];
  } asg_nmea_info_t;


#ifdef __cplusplus
}
#endif

#endif
