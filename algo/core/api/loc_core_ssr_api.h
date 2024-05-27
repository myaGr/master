/**@file        loc_core_ssr_api.h
 * @brief       Location engine core ssr api header file
 * @version     V0.3
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/01  <td>0.1      <td>liuguo      <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __LOC_CORE_SSR_API_H__
#define __LOC_CORE_SSR_API_H__

#include <stdint.h>
#include "loc_core_api.h"
#include "qxsi/qxsi_ssr2los_type_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* observation data records number*/
#define LOC_API_SSR_SAT_NUM_LIMIT                    (45)
/* max number of channel for one satellite*/
#define LOC_API_SSR_SAT_CHL_NUM_LIMIT                (3)

/* Satellites broadcast ephemeris, orbit and clock mask */
typedef enum
{
  /* ephemeris is valid */
  LOC_API_SSR_SAT_ORB_CLK_EPH = 0x01,

  /* orbit correction is calculated */
  LOC_API_SSR_SAT_ORB_CLK_ORBIT_CORR = 0x02,

  /* clock correction is calculated */
  LOC_API_SSR_SAT_ORB_CLK_CLOCK_CORR = 0x04,

  /* precision oribit, relative to EPH dx/dy/dz */
  LOC_API_SSR_SAT_ORB_CLK_PRE_ORBIT = 0x08,

  /* precision clock, absolute */
  LOC_API_SSR_SAT_ORB_CLK_PRE_CLOCK = 0x10,

} loc_api_ssrSatOrbClkMaskVal;
typedef uint8_t loc_api_ssrSatOrbClkMask;

/* Satellites bias mask */
typedef enum
{
  /* code bias correction is calculated */
  LOC_API_SSR_SAT_BIAS_CODE_CORR = 0x01,

  /* phase bias correction is calculated */
  LOC_API_SSR_SAT_BIAS_PHASE_CORR = 0x02
} loc_api_ssrSatBiasMaskVal;
typedef uint8_t loc_api_ssrSatBiasMask;

/* Atmosphere mask */
typedef enum
{
  /* STEC correction is calculated */
  LOC_API_SSR_ATMO_STEC_CORR = 0x01,

  /* STD correction is calculated */
  LOC_API_SSR_ATMO_STD_CORR = 0x02,

  /* ZTD correction is calculated */
  LOC_API_SSR_ATMO_ZTD_CORR = 0x04
} loc_api_ssrAtmoMaskVal;
typedef uint8_t loc_api_ssrAtmoMask;

/* Error model mask */
typedef enum
{
  /* solid tide correction is calculated */
  LOC_API_SSR_ERROR_MODEL_SOLID_TIDE_CORR = 0x01,

  /* ocean tide correction is calculated */
  LOC_API_SSR_ERROR_MODEL_OCEAN_TID_CORR = 0x02,

  /* pole tide is calculated */
  LOC_API_SSR_ERROR_MODEL_POLE_TIDE_CORR = 0x04
} loc_api_ssrErrorModelMaskVal;
typedef uint8_t loc_api_ssrErrorModelMask;

typedef enum
{
  /*gravitaional delay is corrected in orb_clk LOS*/
  LOC_API_SSR_ERROR_MODEL_SAT_GRAVITATION_CORR = 0x01,

  /*BDS2 satellite pseudorange multipath is corrected in code bias(2I/7I/6I) LOS*/
  LOC_API_SSR_ERROR_MODEL_SAT_BDS_MULTIPATH_CORR = 0x02,

  /*satellite wind up correction is calculated and valid*/
  LOC_API_SSR_ERROR_MODEL_SAT_WIND_UP_VALID = 0x04,

  /*satellite attitude Yaw angle and Yaw rate is calculated and valid*/
  LOC_API_SSR_ERROR_MODEL_SAT_YAW_INFO_VALID = 0x08
} loc_api_ssrSatErrorModelMaskVal;
typedef uint8_t loc_api_ssrSatErrorModelMask;

typedef struct
{
  uint16_t w_week;  /* week number of time */
  double   tow;    /* time of week */
} loc_api_GpsTime_t;

/* SSR Service Source */
typedef enum
{
  LOC_API_SSR_SERVICE_SOURCE_NONE = 0,
  LOC_API_SSR_SERVICE_SOURCE_QX,
  LOC_API_SSR_SERVICE_SOURCE_Gee,
  LOC_API_SSR_SERVICE_SOURCE_AG_SRR_LOS
} loc_api_ssrServiceSourceVal;
typedef uint8_t loc_api_ssrServiceSource;

typedef struct
{
  loc_api_GpsTime_t    z_gpsTime;      /** tot, no satellite clock correction */
  uint8_t              u_valid;
  loc_api_gnssConstellationType u_constellation;/** see loc_api_ConstellationType */
  uint8_t              u_svid;         /** satellite index of the system */
  float                f_elevation;    /** rad */
  float                f_azimuth;      /** rad */
  float                f_dt;           /** difference time t with first polynomial t*/
  int32_t              q_iode;         /** ephemeris iode */
  double               d_satPosClk[4]; /** Satellite Position X,Y,Z and Clock Bias, unit [m] */
  double               d_satVelClk[4]; /** Satellite Velocity X,Y,Z and Clock Drift unit [m] */
} loc_api_SatPosVelClk_t;

/* ZTD correction data with quality indicator and integrity */
typedef struct
{
  loc_api_integrityFlag  u_preItg;   /* pre-check integrity */
  loc_api_integrityFlag  u_postItg;  /* post-check integrity */
  float                  f_qi;       /* quality indicator, unit: m */
  double                 d_wetCorr;  /* wet part correction data value */
  double                 d_dryCorr;  /* dry part correction data value */
} loc_api_ZtdCorrData_t;

/* Correction data with quality indicator and integrity */
typedef struct
{
  loc_api_integrityFlag     u_preItg;   /* pre-check integrity */
  loc_api_integrityFlag     u_postItg;  /* post-check integrity */
  /* QI unit for different SSR corrections refer to following:
   * orb_clk: mm
   * code bias/phase bias/ZTD/STD: m
   * STEC: TECU */
  float                  f_qi;       /* quality indicator */
  int32_t                q_corr;     /* correction data value,enlarge 1000 for the origin*/
} loc_api_SsrCorrBias_t;

/* Correction data of one signal */
typedef struct
{
  loc_api_SignalType        u_signalType;           /* Follow loc_api_SignalType */
  loc_api_ssrSatBiasMask    u_biasMask;             /* flag of code or phase corrected indicator */
  uint8_t                   u_discontinuityIod;     /* This IOD indicates whether there has been a discontinuity in the phase bias estimation process */
  float                     f_pcv;                  /* Phase centre variation data */
  loc_api_SsrCorrBias_t     z_codeBias;             /* satallite code correction data */
  loc_api_SsrCorrBias_t     z_phaseBias;            /* satallite phase correction data */
} loc_api_SignalCorr_t;

/* Correction data with quality indicator and integrity for satellite precision orbit and clock */
typedef struct
{
  loc_api_integrityFlag     u_preItg;       /* pre-check integrity */
  loc_api_integrityFlag     u_postItg;      /* post-check integrity */
  float                     f_qi;           /* quality indicator, mm */
  int32_t                   q_delta_pos[3]; /* Orbit position correction dxyz at the orbit reference time, mm*/
  int32_t                   q_clock_bias;   /* Full precise satellite clock bias at the clock bias reference time, mm*/
} loc_api_SsrCorrPrecision_t;

/* Line of sight data of one satellite */
typedef struct
{
  loc_api_gnssConstellationType u_constellation;         /* see loc_api_ConstellationType */
  uint8_t                       u_svid;                  /* satellite index of the system */
  loc_api_GpsTime_t             z_orbClkTime;            /* correction data time for the orbit/clock SSR data */
  loc_api_GpsTime_t             z_STECtime;              /* STEC correction data time */
  loc_api_GpsTime_t             z_STDtime;               /* STD correction data time */
  loc_api_ssrSatOrbClkMask      u_orbClkMask;            /* flag of satellite orbit or clock corrected indicator */
  loc_api_ssrAtmoMask           u_atmoMask;              /* flag of atmosphere corrected indicator */
  loc_api_ssrSatErrorModelMask  u_windUpMask;            /* flag of satellite wind up corrected indicator */
  uint8_t                       u_signalNum;             /* signal number of one satellite */
  int32_t                       q_iode;                  /* ephemeris iode */
  loc_api_SatPosVelClk_t        z_satPosVelClkBrdc;      /* satellite information of ephemeris calculated, for example, satellite position/velocity/clock/clock drift,etc. */
  uint8_t                       u_clockContinuityIod;    /* IOD to indicate whether there has been a discontinuity in clock estimation */
  loc_api_SsrCorrPrecision_t    z_orbclkPrec;            /* correction for the orbit/clock correction delta with ephemeris */
  loc_api_SsrCorrBias_t         z_orbClk;                /* correction for the orbit/clock SSR data */
  loc_api_SsrCorrBias_t         z_stec;                  /* correction for the STEC SSR data */
  loc_api_SsrCorrBias_t         z_std;                   /* correction for the STD SSR data */
  double                        d_windUp;                /* correction of satellite wind up */
  float                         f_yaw;                   /* yaw angle of satellite, unit in degree */
  float                         f_yawRate;               /* yaw rate of satellite, unit in degree per second */
  loc_api_GpsTime_t             z_pcvTime;               /* correction data time for the orbit/clock SSR data */
  loc_api_SignalCorr_t          z_signalBiasCorr[LOC_API_SSR_SAT_CHL_NUM_LIMIT];/* all signal data for one satellite */
} loc_api_satSsrLos_t;

/* Line of sight data of station */
typedef struct
{
  loc_api_GpsTime_t             z_tor;           /* observation time */
  loc_api_GpsTime_t             z_ZTDtime;       /* ZTD correction data time */
  loc_api_ssrAtmoMask           u_ZTDmask;       /* flag of ZTD corrected indicator */
  loc_api_ssrErrorModelMask     u_errorModelMask;/* flag of error model corrected indicator */
  loc_api_ZtdCorrData_t         z_ZTDcorr;       /* correction for the ZTD SSR data */
  uint8_t                       u_satCount;      /* satellite count */
  loc_api_satSsrLos_t           z_satLosCorr[LOC_API_SSR_SAT_NUM_LIMIT];/* all satellite correction data for current epoch */
} loc_api_epochSsrLos_t;

#define VERSION_LOC_API_SSR_LOS_BLOCK (0)
typedef struct
{
  uint8_t                       u_version;
  uint16_t                      w_size;
  double                        d_siteCoor[3];  /* the site initial position of current epoch*/
  loc_api_epochSsrLos_t         z_epochLosInfo; /* observation LOS of SSR data for current epoch */
} loc_api_ssrLosBlock_t;

/**
 * @brief Convert Loc Api reported measurement to Qianxun station input info
 * @param[in] pz_LocGnssMeasBlk - location core API's reported measurement
 * @param[in] pz_qxsi_station_info - Qianxun station input info
 * @return    TRUE - Success
 *            FALSE - Fail
 */
uint8_t ssr_Convert_LocApiGnssMeasBlkToQxsiStationInfo(const loc_api_GnssMeasBlock_t* pz_LocGnssMeasBlk,
                                                       qxsi_station_info_t* pz_qxsi_station_info);

/**
 * @brief Convert SSR los block from Qianxun to location core API
 * @param[in] pz_qxsi_los - Qianxun's station los
 * @param[in] pz_SsrLosBlk - Location core's SSR los
 * @return    TRUE - Success
 *            FALSE - Fail
 */
uint8_t ssr_Convert_QxsiStationLosToLocApiSsrLosBlk(const qxsi_station_los_t* pz_qxsi_los,
                                                   loc_api_ssrLosBlock_t* pz_SsrLosBlk);

/**
  * @brief     Inject Observation LOS of SSR data
  * @param[in] pz_LocSsrLosBlk - pointer to gnss_ssrLosBlock_t
  * @return    None
  * @note      gnss_ssrLosBlock_t is defined by Location engine, it needs to
  *     convert SSR service's data to this structure before call the function
  */
void loc_api_InjectSsrLosBlock(const loc_api_ssrLosBlock_t* pz_LocSsrLosBlk);

#ifdef __cplusplus
}
#endif

#endif

