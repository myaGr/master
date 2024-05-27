#ifndef __GNSS_CONFIG_DEFAULT_H__
#define __GNSS_CONFIG_DEFAULT_H__

#include "macro.h"
#include "ag_base.h"
#include "gnss_config.h"
#include "gnss_sys_api.h"

enum
{
  RE_START_MODE_COLD,
  RE_START_MODE_HOT
};

#define DFLTCFG_PE_CN0_MASK               12
#define DFLTCFG_PE_ALT_LOW_LIMIT          -1000.0
#define DFLTCFG_PE_ALT_HIGH_LIMIT         18000.0
#define DFLTCFG_PE_ENABLE_INDOOR          0
#define DFLTCFG_PE_ENABLE_WAAS            0
#define DFLTCFG_PE_PDOP_MASK              6.0
#define DFLTCFG_PE_DYNAMICS               5
#define DFLTCFG_PE_ENABLE_RTD             0
#define DFLTCFG_PE_POS_RES_THRES          25.0
#define DFLTCFG_PE_ELE_MASK               4.0
#define DFLTCFG_PE_START_MODE             RE_START_MODE_HOT
#if (PLATFORM == PLT_QUALCOM)
#define DFLTCFG_PE_MEAS_TYPE              GNSS_MEAS_TYPE_QCOM
#define DFLTCFG_PE_SUB_CHIPTYPE           SUB_CHIPTYPE_QCOM_SF
#else
#define DFLTCFG_PE_MEAS_TYPE              GNSS_MEAS_TYPE_UBLOX
#ifdef UBX_RAW
#define DFLTCFG_PE_SUB_CHIPTYPE           SUB_CHIPTYPE_UBLOX_F9
#else
#define DFLTCFG_PE_SUB_CHIPTYPE           SUB_CHIPTYPE_ST_8100
#endif
#endif
#if (PLATFORM == PLT_QUALCOM)
#define DFLTCFG_PE_AUTO_MOBILE            1
#else
#define DFLTCFG_PE_AUTO_MOBILE            1
#endif
#define DFLTCFG_PE_APPLY_SCENE            APPLY_SCENE_AUTOROOF
#define DFLTCFG_PE_RINEX_DATA_TYPE        RINEX_DATA_TYPE_NORMAL
#define DFLTCFG_PE_MEAS_TYPE_MASK         (MEAS_TYPE_MASK_PR|MEAS_TYPE_MASK_DR|MEAS_TYPE_MASK_CP|MEAS_TYPE_MASK_SNR)
#define DFLTCFG_PE_CODE_STD_PHONE         40.0
#define DFLTCFG_PE_CODE_STD_AUTO          160.0
#define DFLTCFG_PE_CARR_STD_PHONE         3.0
#define DFLTCFG_PE_CARR_STD_AUTO          4.0
#define DFLTCFG_PE_RTD_USAGE_FLAG         0
#ifdef ENABLE_PPK_MODE
#define DFLTCFG_PE_PPK_MODE               1
#define DFLTCFG_PE_GNSS_USAGE_FLAG        0x17
#define DFLTCFG_RTK_NAV_SYS_MASK          (RTK_NAV_MASK_BIT_GPS|RTK_NAV_MASK_BIT_GLN|RTK_NAV_MASK_BIT_BDS)
#else
#define DFLTCFG_PE_PPK_MODE               0
#define DFLTCFG_PE_GNSS_USAGE_FLAG        0x5
#define DFLTCFG_RTK_NAV_SYS_MASK          (RTK_NAV_MASK_BIT_GPS|RTK_NAV_MASK_BIT_BDS|RTK_NAV_MASK_BIT_GAL|RTK_NAV_MASK_BIT_QZSS)
#endif

#define DFLTCFG_RTK_ENABLE_FIRST_STATIC   1
#define DFLTCFG_RTK_ENABLE_RTD            0
#if AGGNSS_MAX_FREQ_NUM > 1
#define DFLTCFG_RTK_FREQ_COMBINE          2
#define DFLTCFG_RTK_ELE_MASK              15.0    /* degree */
#define DFLTCFG_RTK_AR_LOCK_CNT           2
#define DFLTCFG_RTK_AR_ELE_MASK           20.0    /* degree */
#else
#define DFLTCFG_RTK_FREQ_COMBINE          1
#define DFLTCFG_RTK_ELE_MASK              10.0    /* degree */
#if (PLATFORM == PLT_QUALCOM)
#define DFLTCFG_RTK_AR_LOCK_CNT           6
#define DFLTCFG_RTK_AR_ELE_MASK           15.0    /* degree */
#else
#define DFLTCFG_RTK_AR_LOCK_CNT           0
#define DFLTCFG_RTK_AR_ELE_MASK           0    /* degree */
#endif
#endif
#define DFLTCFG_RTK_ENABLE_BDS_ARMODE     1
#define DFLTCFG_RTK_ENABLE_GLN_ARMODE     0
#define DFLTCFG_RTK_OUTPUT_RINEX_FILE     0
#define DFLTCFG_RTK_DEBUG_LEVEL           0
#define DFLTCFG_RTK_PROCESS_TYPE          0
#define DFLTCFG_RTK_POS_MODE              RTK_POS_MODE_KINEMATIC
#define DFLTCFG_RTK_ARMODE                RTK_ARMODE_FIX_HOLD
#define DFLTCFG_RTK_AR_MIN_FIX            2
#define DFLTCFG_RTK_AR_OUT_CNT            5
#define DFLTCFG_RTK_AR_THRES              2.5
#define DFLTCFG_RTK_ELE_MASK_HOLD         0       /* degree */

#define DFLTCFG_RTK_MAX_AGE               120.0    /* second */
#define DFLTCFG_RTK_REJ_THRES_INNO        30      /* m */
#define DFLTCFG_RTK_ERR_RATIO_1           100.0
#define DFLTCFG_RTK_ERR_RATIO_2           100.0
#define DFLTCFG_RTK_ERR_PHASE_FACTOR_A    0.003   /* m */
#define DFLTCFG_RTK_ERR_PHASE_FACTOR_B    0.003   /* m */
#define DFLTCFG_RTK_ERR_PHASE_FACTOR_C    0.0     /* m/10km */
#define DFLTCFG_RTK_ERR_PHASE_DOPP_FREQ   1.0     /* Hz */
#define DFLTCFG_RTK_INIT_STATE_STD_BIAS   30.0    /* m */
#define DFLTCFG_RTK_PROC_NOISE_STD_BIAS   0.0001  /* m */

#ifdef USE_AG_DR
#define DFLTCFG_DE_ENABLE                 1
#else
#define DFLTCFG_DE_ENABLE                 0
#endif

#define DFLCFG_SYS_MEAS_RATE              MEAS_RATE_1HZ
#define DFLTCFG_SYS_HSM_COMMPORT          5000
#define DFLTCFG_SYS_RTCM_COMMPORT         5001
#define DFLTCFG_SYS_AL_COMMPORT           5004
#define DFLTCFG_SYS_RE_COMMPORT           5002
#define DFLTCFG_SYS_DE_COMMPORT           5003
#define DFLTCFG_SYS_DATA_COMMPORT         5005
#define DFLTCFG_SYS_LOG_PATH              LOCAL_LOG_FOLDER
#define DFLTCFG_SYS_UNIX_SOCKET_PATH      UNIX_SOCKET_PATH
#ifdef HTC_UTTOOL
#define DFLTCFG_SYS_UTTOOL_ENABLE         FALSE
#endif
#ifdef ENABLE_PPK_MODE
#define DFLTCFG_SYS_LOG_LEVEL             LOG_NONE
#else
#define DFLTCFG_SYS_LOG_LEVEL             LOG_INFO
#endif
#define DFLTCFG_SYS_LOG_MASK              (LOG_MASK_DEBUG_LOG|LOG_MASK_PLAYBACK|LOG_MASK_NMEA|LOG_MASK_RAWIN)
#define DFLTCFG_SYS_CB_LOG_MASK           (LOG_MASK_NO_LOG)
#ifdef HTC_UTTOOL
#define DFLTCFG_SYS_LOG_ROLLING_SIZE      100	//MByte
#else
#define DFLTCFG_SYS_LOG_ROLLING_SIZE      -1
#endif

#ifdef ENABLE_PPK_MODE
#define DFLTCFG_PCSIM_ENABLE_PLAYBACK     TRUE
#else
#define DFLTCFG_PCSIM_ENABLE_PLAYBACK     FALSE
#endif

#endif

