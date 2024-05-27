/**@file        gnss_def.c
 * @brief       Location Engine GNSS Related Definition
 * @details     1. Mathmatic definition
 *              2. Geography definition
 *              3. GNSS signal definition
 * @author      caizhijie
 * @date        2022/04/25
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * <tr><td>2022/09/24  <td>0.2      <td>caizhijie   <td>Add Measurement number definition
 * </table>
 *
 **********************************************************************************
 */

#ifndef __GNSS_DEF_H__
#define __GNSS_DEF_H__

#include "cmn_def.h"

#define MAX_NAV_DATA_FRAME_SIZE (40)                /* max size of navigation frame data, bytes */
#define MAX_NAV_DATA_FRAME_BITS (MAX_NAV_DATA_FRAME_SIZE*8)
#define BUFF_SIZE     (1024)                        /* length of buff 1b */

/* Mathmatic definition */
#define SQR(x)        ((x)*(x))                     /* pow(*,2) */
#define PI            ((double)3.1415926535897932)  /* pi */
#define ASU           ((double)149597870691.0)      /* astronomical unit, 1 AU (m) */
#define DEG2RAD       (PI/180.0)                    /* deg to rad */
#define RAD2DEG       (180.0/PI)                    /* rad to deg */
#define ARCSEC2RAD    (DEG2RAD/3600.0)              /* arc sec to radian */
#define INVALID_INDEX ((int8_t)-1)


#define P2_01 ((double)5.000000000000000E-01) /* 2^-01 */
#define P2_02 ((double)2.500000000000000E-01) /* 2^-02 */
#define P2_03 ((double)1.250000000000000E-01) /* 2^-03 */
#define P2_04 ((double)6.250000000000000E-02) /* 2^-04 */
#define P2_05 ((double)3.125000000000000E-02) /* 2^-05 */
#define P2_06 ((double)1.562500000000000E-02) /* 2^-06 */
#define P2_07 ((double)7.812500000000000E-03) /* 2^-07 */
#define P2_08 ((double)3.906250000000000E-03) /* 2^-08 */
#define P2_09 ((double)1.953125000000000E-03) /* 2^-09 */
#define P2_10 ((double)9.765625000000000E-04) /* 2^-10 */
#define P2_11 ((double)4.882812500000000E-04) /* 2^-11 */
#define P2_12 ((double)2.441406250000000E-04) /* 2^-12 */
#define P2_13 ((double)1.220703125000000E-04) /* 2^-13 */
#define P2_14 ((double)6.103515625000000E-05) /* 2^-14 */
#define P2_15 ((double)3.051757812500000E-05) /* 2^-15 */
#define P2_16 ((double)1.525878906250000E-05) /* 2^-16 */
#define P2_17 ((double)7.629394531250000E-06) /* 2^-17 */
#define P2_18 ((double)3.814697265625000E-06) /* 2^-18 */
#define P2_19 ((double)1.907348632812500E-06) /* 2^-19 */
#define P2_20 ((double)9.536743164062500E-07) /* 2^-20 */
#define P2_21 ((double)4.768371582031250E-07) /* 2^-21 */
#define P2_22 ((double)2.384185791015625E-07) /* 2^-22 */
#define P2_23 ((double)1.192092895507812E-07) /* 2^-23 */
#define P2_24 ((double)5.960464477539062E-08) /* 2^-24 */
#define P2_25 ((double)2.980232238769531E-08) /* 2^-25 */
#define P2_26 ((double)1.490116119384766E-08) /* 2^-26 */
#define P2_27 ((double)7.450580596923828E-09) /* 2^-27 */
#define P2_28 ((double)3.725290298461914E-09) /* 2^-28 */
#define P2_29 ((double)1.862645149230957E-09) /* 2^-29 */
#define P2_30 ((double)9.313225746154785E-10) /* 2^-30 */
#define P2_31 ((double)4.656612873077393E-10) /* 2^-31 */
#define P2_32 ((double)2.328306436538696E-10) /* 2^-32 */
#define P2_33 ((double)1.164153218269348E-10) /* 2^-33 */
#define P2_34 ((double)5.820766091346741E-11) /* 2^-34 */
#define P2_35 ((double)2.910383045673370E-11) /* 2^-35 */
#define P2_36 ((double)1.455191522836685E-11) /* 2^-36 */
#define P2_37 ((double)7.275957614183426E-12) /* 2^-37 */
#define P2_38 ((double)3.637978807091713E-12) /* 2^-38 */
#define P2_39 ((double)1.818989403545856E-12) /* 2^-39 */
#define P2_40 ((double)9.094947017729282E-13) /* 2^-40 */
#define P2_41 ((double)4.547473508864641E-13) /* 2^-41 */
#define P2_42 ((double)2.273736754432321E-13) /* 2^-42 */
#define P2_43 ((double)1.136868377216160E-13) /* 2^-43 */
#define P2_44 ((double)5.684341886080801E-14) /* 2^-44 */
#define P2_45 ((double)2.842170943040401E-14) /* 2^-45 */
#define P2_46 ((double)1.421085471520200E-14) /* 2^-46 */
#define P2_47 ((double)7.105427357601002E-15) /* 2^-47 */
#define P2_48 ((double)3.552713678800501E-15) /* 2^-48 */
#define P2_49 ((double)1.776356839400250E-15) /* 2^-49 */
#define P2_50 ((double)8.881784197001252E-16) /* 2^-50 */
#define P2_51 ((double)4.440892098500626E-16) /* 2^-51 */
#define P2_52 ((double)2.220446049250313E-16) /* 2^-52 */
#define P2_53 ((double)1.110223024625157E-16) /* 2^-53 */
#define P2_54 ((double)5.551115123125783E-17) /* 2^-54 */
#define P2_55 ((double)2.775557561562891E-17) /* 2^-55 */
#define P2_56 ((double)1.387778780781446E-17) /* 2^-56 */
#define P2_57 ((double)6.938893903907228E-18) /* 2^-57 */
#define P2_58 ((double)3.469446951953614E-18) /* 2^-58 */
#define P2_59 ((double)1.734723475976807E-18) /* 2^-59 */
#define P2_60 ((double)8.673617379884035E-19) /* 2^-60 */
#define P2_61 ((double)4.336808689942018E-19) /* 2^-61 */
#define P2_62 ((double)2.168404344971009E-19) /* 2^-62 */
#define P2_63 ((double)1.084202172485504E-19) /* 2^-63 */
#define P2_64 ((double)5.421010862427522E-20) /* 2^-64 */
#define P2_65 ((double)2.710505431213761E-20) /* 2^-65 */
#define P2_66 ((double)1.355252715606881E-20) /* 2^-66 */
#define P2_67 ((double)6.776263578034403E-21) /* 2^-67 */
#define P2_68 ((double)3.388131789017201E-21) /* 2^-68 */
#define P2_69 ((double)1.694065894508601E-21) /* 2^-69 */

/**********************************************************************************
                           Geography definition
**********************************************************************************/
/* Light speed and distance */
#define CLIGHT                ((double)299792458.0) /* speed of light (m/s) */
#define CLIGHT_RANGE_SEC      (CLIGHT)               /* range in 1  s */
#define CLIGHT_RANGE_MSEC     (CLIGHT*0.001)         /* range in 1 ms */
#define CLIGHT_RANGE_NSEC     (CLIGHT*0.000000001)   /* range in 1 ns */

#define RE_WGS84    ((double)6378137.0)        /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563)        /* earth flattening (WGS84) */
#define ETHAV       ((double)7.2921151467E-5)  /* earth angular velocity (IS-GPS) (rad/s) */

/* GPS Geocentric gravitational constant */
#define MU_GPS      ((double)3.986005000E14)

/* GLO Geocentric gravitational constant */
#define MU_GLO      ((double)3.986004400E14)

/* GAL Geocentric gravitational constant */
#define MU_GAL      ((double)3.986004418E14)

/* BDS Geocentric gravitational constant */
#define MU_BDS      ((double)3.986004418E14)

/* GPS WGS84 Earth rotation Rate */
#define OMGE_GPS    ((double)7.2921151467E-5)

/* GLO Earth rotation Rate */
#define OMGE_GLO    ((double)7.2921150000E-5)

/* GAL Earth rotation Rate */
#define OMGE_GAL    ((double)7.2921151467E-5)

/* BDS Earth rotation Rate */
#define OMGE_BDS    ((double)7.2921150000E-5)

/* Relativistic clock correction constant.
             sqrt(MU)
    F= -2 * ----------- = -2*u^(1/2)*\(c^2)
            (CLIGHT^2)
 */
#define GPS_F_RELATIVITY_CORR ((double)-4.442807633e-10)
#define GLO_F_RELATIVITY_CORR ((double)-4.442807299012562E-10)
#define GAL_F_RELATIVITY_CORR ((double)-4.4428073090439775E-10)
#define BDS_F_RELATIVITY_CORR ((double)-4.4428073090439775E-10)

/* relative tolerance for Kepler equation */
#define RELATIVE_TOL_KEPLER   1E-13 

/* Acceleration of gravity of GuangZhou */
#define Gravity_ShangHai  (9.7964f)

/* Acceleration of gravity of GuangZhou */
#define Gravity_GuangZhou (9.7883105f)

/* Acceleration of gravity of equator*/
#define Gravity_Equator   (9.780325f)

/**********************************************************************************
                  GNSS Satellite Number Definition
**********************************************************************************/
#define MIN_GPS_SVID          1
#define MAX_GPS_SVID          32
#define N_GPS_SV              (MAX_GPS_SVID - MIN_GPS_SVID + 1)
#define GPS_ID_OK(x)          ((x)>=MIN_GPS_SVID&&(x)<=MAX_GPS_SVID)

#define MIN_GLO_SVID          1
#define MAX_GLO_SVID          32
#define N_GLO_SV              (MAX_GLO_SVID - MIN_GLO_SVID + 1)
#define GLO_ID_OK(x)          ((x)>=MIN_GLO_SVID&&(x)<=MAX_GLO_SVID)

#define MIN_BDS_SVID          1
#define MAX_BDS_SVID          63
#define N_BDS_SV              (MAX_BDS_SVID - MIN_BDS_SVID + 1)
#define BDS_ID_OK(x)          ((x)>=MIN_BDS_SVID&&(x)<=MAX_BDS_SVID)
#define BDS_ID_MEO(x)         ((x)>=6&&x<=58)
#define BDS_ID_GEO(x)         (((x)>0&&x<=5)||((x)>=59&&(x)<=63))

#define MIN_GAL_SVID          1
#define MAX_GAL_SVID          37
#define N_GAL_SV              (MAX_GAL_SVID - MIN_GAL_SVID + 1)
#define GAL_ID_OK(x)          ((x)>=MIN_GAL_SVID&&(x)<=MAX_GAL_SVID)

#define MIN_WAAS_SVID         120
#define MAX_WAAS_SVID         138
#define N_WAAS_SV             (MAX_WAAS_SVID - MIN_WAAS_SVID + 1)

#define MIN_QZS_SVID          1
#define MAX_QZS_SVID          7
#define N_QZS_SV              (MAX_QZS_SVID - MIN_QZS_SVID + 1)
#define QZS_ID_OK(x)          ((x)>=MIN_QZS_SVID&&(x)<=MAX_QZS_SVID)
#define QZS_ID_OFFSET         192

#define ALL_GNSS_SYS_SV_NUMBER   (  N_GPS_SV + \
                                    N_GLO_SV + \
                                    N_BDS_SV + \
                                    N_GAL_SV + \
                                    N_QZS_SV)

#define MAX_SV_BETWEEN_GPS_GLO     ( N_GPS_SV > N_GLO_SV ? N_GPS_SV : N_GLO_SV )
#define MAX_SV_BETWEEN_BDS_GAL_QZS ( N_BDS_SV > N_GAL_SV ? (N_BDS_SV>N_QZS_SV?N_BDS_SV:N_QZS_SV) : (N_GAL_SV>N_QZS_SV?N_GAL_SV:N_QZS_SV) )
#define MAX_SV_NUM_IN_ONE_SYS      ( MAX_SV_BETWEEN_GPS_GLO > MAX_SV_BETWEEN_BDS_GAL_QZS ? MAX_SV_BETWEEN_GPS_GLO : MAX_SV_BETWEEN_BDS_GAL_QZS )

/* Maximum receiver track measurement number */
#define MAX_GNSS_TRK_MEAS_NUMBER      (150)

/* Maximum visual satellite number */
#define MAX_GNSS_ACTIVE_SAT_NUMBER    (42)

/* Maximum signal frequency number */
#define MAX_GNSS_SIGNAL_FREQ          (3)

/* GNSS Signal Definition */
/* GPS C/A Signal Conversions */
#define CA_CHIPS_MSEC         (1023)   /* Number of C/A chips per msec */
#define CA_PERIOD_MSEC        (1)      /* PRN period in milliseconds */
#define CA_FREQ               (1000.0 * CA_CHIPS_MSEC)
#define GPS_L1C_FREQ          (1540.0 * CA_FREQ)
#define GPS_L2C_FREQ          (1200.0 * CA_FREQ)
#define GPS_L5Q_FREQ          (1150.0 * CA_FREQ)
#define GPS_L1C_WAVE          (CLIGHT / GPS_L1C_FREQ)
#define GPS_L2C_WAVE          (CLIGHT / GPS_L2C_FREQ)
#define GPS_L5Q_WAVE          (CLIGHT / GPS_L5Q_FREQ)

/* BDS Signal Conversions */
#define BDS_CA_CHIPS_MSEC     (2046)  /* Number of C/A chips per msec */
#define BDS_B1C_CA_CHIPS_MSEC (10230)  /* Number of C/A chips per msec */

#define BDS_CA_PERIOD_MSEC    (1)     /* chip period in milliseconds */
#define BDS_CA_FREQ           (1000.0 * BDS_CA_CHIPS_MSEC)
#define BDS_B1I_FREQ          (763.0 * BDS_CA_FREQ)   /* 1561.098 MHz */
#define BDS_B1C_FREQ          (770.0 * BDS_CA_FREQ)   /* 1575.42 MHz */
#define BDS_B2I_FREQ          (590.0 * BDS_CA_FREQ)   /* 1207.14 MHz */
#define BDS_B2A_FREQ          (575.0 * BDS_CA_FREQ)   /* 1176.45 MHz */
#define BDS_B2B_FREQ          (590.0 * BDS_CA_FREQ)   /* 1207.14 MHz */
#define BDS_B3I_FREQ          (620.0 * BDS_CA_FREQ)   /* 1268.52 MHz */
#define BDS_B1I_WAVE          (CLIGHT / BDS_B1I_FREQ)
#define BDS_B1C_WAVE          (CLIGHT / BDS_B1C_FREQ)
#define BDS_B2I_WAVE          (CLIGHT / BDS_B2I_FREQ)
#define BDS_B2A_WAVE          (CLIGHT / BDS_B2A_FREQ)
#define BDS_B2B_WAVE          (CLIGHT / BDS_B2B_FREQ)
#define BDS_B3I_WAVE          (CLIGHT / BDS_B3I_FREQ)

/* GAL Signal Conversions */
#define GAL_E1_FREQ           GPS_L1C_FREQ
#define GAL_E5A_FREQ          GPS_L5Q_FREQ
#define GAL_E5B_FREQ          (1180.0 * CA_FREQ)
#define GAL_E1_WAVE           (CLIGHT / GAL_E1_FREQ)
#define GAL_E5A_WAVE          (CLIGHT / GAL_E5A_FREQ)
#define GAL_E5B_WAVE          (CLIGHT / GAL_E5B_FREQ)

/* QZSS Signal Conversions */
#define QZS_L1C_FREQ          GPS_L1C_FREQ
#define QZS_L2C_FREQ          GPS_L2C_FREQ
#define QZS_L5Q_FREQ          GPS_L5Q_FREQ
#define QZS_L1C_WAVE          (CLIGHT / QZS_L1C_FREQ)
#define QZS_L2C_WAVE          (CLIGHT / QZS_L2C_FREQ)
#define QZS_L5Q_WAVE          (CLIGHT / QZS_L5Q_FREQ)

/* Time Convert */
#define TIME_MSEC             (1000)
#define TIME_MSEC_INV         (0.001)
#define TIME_NSEC             (1000000000)
#define TIME_NSEC_INV         (0.000000001)
#define TIME_MSEC_SUB         (10000000)
#define LEAP_SECS              18

#define MINUTE_SEC            ((uint32_t)60)
#define MINUTE_MSEC           (TIME_MSEC * MINUTE_SEC)
#define HOUR_SEC              (60 * MINUTE_SEC)
#define HOUR_MSEC             (TIME_MSEC * HOUR_SEC)
#define DAY_SEC               (24 * HOUR_SEC)
#define DAY_MSEC              (TIME_MSEC * DAY_SEC)
#define WEEK_SEC              (7 * DAY_SEC)
#define WEEK_MSEC             (TIME_MSEC * WEEK_SEC)

#define GPS_BDS_OFFSET_TOW_MSEC   (14000)
#define GPS_BDS_OFFSET_WEEK       (1356)

#define GPS_GAL_OFFSET_TOW_MSEC   (0)
#define GPS_GAL_OFFSET_WEEK       (1024)


#define MJD_JDAY  2400000.5
#define TAI_TDT   32.184     /* 32.184 = TAI - TDT */
#define TAI_GPST  -19        /* 19 = TAI - GPST  */
#define TTOL      0.005      /* tolerance of time difference (s) */


typedef uint8_t algo_useConstellation;/* define constellation used in the algorithm */
#define ALGO_NON_SYS   ((uint8_t)0x00) /* invaild value for the constellation used in the algorithm */
#define ALGO_GPS_SYS   ((uint8_t)0x01)/* enable GPS constellation used in the algorithm */
#define ALGO_BDS_SYS   ((uint8_t)0x02)/* enable BDS constellation used in the algorithm */
#define ALGO_GLO_SYS   ((uint8_t)0x04)/* enable GLONASS constellation used in the algorithm */
#define ALGO_GAL_SYS   ((uint8_t)0x08)/* enable Galileo constellation used in the algorithm */
#define ALGO_QZS_SYS   ((uint8_t)0x10)/* enable QZSS constellation used in the algorithm */

typedef uint8_t algo_useFreq;         /* define frequency used in the algorithm */
#define ALGO_NON_FREQ   ((uint8_t)0x0)/* invalid value for the frequency used in the algorithm */
#define ALGO_L1_FREQ   ((uint8_t)0x01)/* enable L1 frequency used in the algorithm */
#define ALGO_L2_FREQ   ((uint8_t)0x02)/* enable L2 frequency used in the algorithm */
#define ALGO_L5_FREQ   ((uint8_t)0x04)/* enable L5 frequency used in the algorithm */

/* meas mask flag */
typedef uint32_t gnss_MeasureStatusType;
#define GNSS_MEASURE_STATUS_PR_VALID ((uint32_t)0x00000001 << 0)
#define GNSS_MEASURE_STATUS_DR_VALID ((uint32_t)0x00000001 << 1)
#define GNSS_MEASURE_STATUS_CP_VALID ((uint32_t)0x00000001 << 2)
#define GNSS_MEASURE_STATUS_MP_FLAG  ((uint32_t)0x00000001 << 3)

typedef uint32_t gnss_PrMaskType;
#define GNSS_PR_MASK_PRE_VALID       ((uint32_t)0x00000001 << 0)
#define GNSS_PR_MASK_POST_VALID      ((uint32_t)0x00000001 << 1)

typedef uint32_t gnss_DrMaskType;
#define GNSS_DR_MASK_PRE_VALID       ((uint32_t)0x00000001 << 0)
#define GNSS_DR_MASK_POST_VALID      ((uint32_t)0x00000001 << 1)

typedef uint32_t gnss_PrDeWeightMaskType;
#define GNSS_PR_DEW_MASK_HIGH_ELE    ((uint32_t)0x00000001 << 0)
#define GNSS_PR_DEW_MASK_CNO          ((uint32_t)0x00000001 << 1)

typedef uint32_t gnss_DrDeWeightMaskType;
#define GNSS_DR_DEW_MASK_HIGH_ELE    ((uint32_t)0x00000001 << 0)
#define GNSS_DR_DEW_MASK_CNO         ((uint32_t)0x00000001 << 1)

#define HEIGHT_ION        350000.0           /* ionosphere height (m) */
#define IONO_PRO_OPT_OFF  0                  /* ionosphere process option: correction off */
#define IONO_PRO_OPT_BRDC 1                  /* ionosphere process option: broadcast model */
#define IONO_PRO_OPT_SBAS 2                  /* ionosphere process option: SBAS model */
#define IONO_PRO_OPT_IFLC 3                  /* ionosphere process option: L1/L2 iono-free LC */
#define IONO_PRO_OPT_EST  4                  /* ionosphere process option: estimation */
#define IONO_PRO_OPT_TEC  5                  /* ionosphere process option: IONEX TEC model */
#define IONO_PRO_OPT_QZS  6                  /* ionosphere process option: QZSS broadcast model */
#define IONO_PRO_OPT_STEC 8                  /* ionosphere process option: SLANT TEC model */

/* Satellite data filed */
#define GNSS_EPHEMERIS_CACHE_NUM              2                   /* ephemeris cache numbers every satellite */
#define GNSS_EPHEMERIS_CACHE_EXPIRE           (14400.0)           /* expire time difference to ephemeris toe   */
#define GNSS_MAXDTOE                          (7200.0)            /* max time difference to GPS Toe (s) */
#define GNSS_MAXDTOE_GLO                      (1800.0)            /* max time difference to GLONASS Toe (s) */
#define GNSS_MAXDTOE_BDS                      (21600.0)           /* max time difference to BeiDou Toe (s) */
#define GNSS_MAXDTOE_GAL                      (14400.0)           /* max time difference to Galileo Toe (s) */
#define GNSS_MAXDTOE_QZS                      (7200.0)            /* max time difference to QZSS Toe (s) */
#define SAT_POLYNOMIAL_DURATION               (90)                /* satelite polynomial time duration */
#define SAT_POLYNOMIAL_INTERVAL               (30)                /* satelite polynomial time interval */
#define SAT_POLYNOMIAL_REQ_TIME               (70)                /* satelite polynomial request time */
#define SAT_POLYNOMIAL_REQ_ADVANCED_TIME      (-10)               /* satelite polynomial request time */


#endif