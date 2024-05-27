/**@file        sd_task.c
 * @brief       Satellite Datebase Process
 * @details     1. Ephemeris/Almanac calculate
 *              2. SSR data calculate
 * @author      caizhijie
 * @date        2022/04/26
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/26  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include <math.h>

#include "gnss_common.h"
#include "cmn_utils.h"
#include "mw_alloc.h"
#include "mw_ipctask.h"
#include "mw_log.h"
#include "sd_task.h"
#include "sd_api.h"
#include "hpp_api.h"
#include "loc_core_cfg.h"
#include "sm_api.h"

typedef struct {
  uint8_t b_Init;
} sd_task_ctrl_t;
static sd_task_ctrl_t sd_hpp_task_ctrl;
static sd_GnssEphemerisPool_t              gz_GnssEphemeris_Pool = { 0 };
static sd_GnssSatPosVelClkPolynomialPool_t gz_GnssSatPvtPoly_Pool = { 0 };

/**
 * @brief SD module initialize
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sd_Init(ipc_t* p_ipc)
{
  if (FALSE == gz_GnssEphemeris_Pool.b_init)
  {
    mutex_init(&gz_GnssEphemeris_Pool.z_mtx, NULL);
    gz_GnssEphemeris_Pool.b_init = TRUE;
  }

  if (FALSE == gz_GnssSatPvtPoly_Pool.b_init)
  {
    mutex_init(&gz_GnssSatPvtPoly_Pool.z_mtx, NULL);
    gz_GnssSatPvtPoly_Pool.b_init = TRUE;
  }

  sd_hpp_task_ctrl.b_Init = TRUE;

  LOGI(TAG_SD, "SD Init success %d Eph Pool %d Poly Pool %d\n",
    sd_hpp_task_ctrl.b_Init, 
    gz_GnssEphemeris_Pool.b_init,
    gz_GnssSatPvtPoly_Pool.b_init);

  return;
}

/**
 * @brief SD module start
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sd_Start(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief SD module stop
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sd_Stop(ipc_t* p_ipc)
{
  return;
}

/**
 * @brief SD module release
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sd_Release(ipc_t* p_ipc)
{
  for (uint8_t u_constellation = C_GNSS_GPS; u_constellation < C_GNSS_MAX; u_constellation++)
  {
    gz_GnssEphemeris_Pool.t_gnssEphMask[u_constellation] = 0x0;
    gz_GnssSatPvtPoly_Pool.t_gnssSatPvtPolyMask[u_constellation] = 0x0;
  }

  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; i++)
  {
    for (int j = 0; j < GNSS_EPHEMERIS_CACHE_NUM; ++j)
    {
      if (NULL != gz_GnssEphemeris_Pool.pz_gnssEph[i][j])
      {
        OS_FREE(gz_GnssEphemeris_Pool.pz_gnssEph[i][j]);
      }
    }
    if (NULL != gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[i])
    {
      OS_FREE(gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[i]);
      gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[i] = NULL;
    }
  }

  if (TRUE == gz_GnssEphemeris_Pool.b_init)
  {
    mutex_destroy(&gz_GnssEphemeris_Pool.z_mtx);
    gz_GnssEphemeris_Pool.b_init = FALSE;
  }

  if (TRUE == gz_GnssSatPvtPoly_Pool.b_init)
  {
    mutex_destroy(&gz_GnssSatPvtPoly_Pool.z_mtx);
    gz_GnssSatPvtPoly_Pool.b_init = FALSE;
  }

  sd_hpp_task_ctrl.b_Init = FALSE;
  return;
}

/**
 * @brief This function converts a position in self-defined coordinate
          to a Earth-Centred, Earth-Fixed coordinate

    | sv_pos_ecef[0] |                   | sv_pos[0] |
    | sv_pos_ecef[1] | = Rz(A) * Rx(B) * | sv_pos[1] |
    | sv_pos_ecef[2] |                   | sv_pos[2] |

                        | +cos(A) +sin(A)   0     |
                Rz(A) = | -sin(A) +cos(A)   0     |
                        |   0       0       1     |

                        |   1       0       0     |
                Rx(B) = |   0     +cos(B) +sin(B) |
                        |   0     -sin(B) +cos(B) |

 * @param[in]   d_Alpha - earth rotation [Rad]
 * @param[in]   d_Beta  - 
 * @param[in]   self_pos
 * @param[out]  ecef_pos
 * @return: none
 */
static void sd_cvt_UserDefinedXyz2Ecef(double alpha, double beta,
  const double* self_pos, double* ecef_pos)
{
  double SinAlpha = 0.0;
  double CosAlpha = 0.0;
  double SinBeta = 0.0; 
  double CosBeta = 0.0;

  if ((NULL == self_pos) ||
    (NULL == ecef_pos))
  {
    return;
  }

  SinAlpha = sin(alpha);
  CosAlpha = cos(alpha);
  SinBeta = sin(beta);
  CosBeta = cos(beta);

  ecef_pos[0] =  CosAlpha * self_pos[0] + SinAlpha * CosBeta * self_pos[1] + SinAlpha * SinBeta * self_pos[2];
  ecef_pos[1] = -SinAlpha * self_pos[0] + CosAlpha * CosBeta * self_pos[1] + CosAlpha * SinBeta * self_pos[2];
  ecef_pos[2] = -SinBeta  * self_pos[1] + CosBeta  * self_pos[2];

  return;
}

/**
 * @brief The satellite velocity computation for GEO contains two steps.
    1. Rotate from the standard inertial frame to a user defined inertial frame
    2. Rotate from the user defined inertial frame to ECEF
  For step 2. the transformation is defined as:
    Vel_ECEF = Rz(A(t))* Rx(B) * Vel_user + RzDot(A(t)) * Rx(B) * Pos_user
    where
    B is a const value. A(t) is a time function
            |  cos(A)  sin(A)  0 |
    Rz(A) = | -sin(A)  cos(A)  0 |
            |   0        0     1 |

            |   1        0        0   |
    Rx(B) = |   0      cos(B)  sin(B) |
            |   0      -sin(B) cos(B) |

    Rz(A(t))* Rx(B) * Vel_user has been computed outside by calling
      ConvertUserInertialPos2ECEF
    RzDot(A(t)) * Rx(B) * Pos_user is the 2nd part correction and computed here.

 * @param[in]   d_Alpha - earth rotation [Rad]
 * @param[in]   d_Beta  -
 * @param[in]   self_pos
 * @param[in]  pd_EarthRateRps
 * @param[out]  pd_SvVCorr
 * @return: none
 */
static void ComputeGeoSatVel2ndCorr(double d_Alpha,
  double d_Beta,
  const double* pd_SvPos,
  const double pd_EarthRateRps,
  double* pd_SvVCorr)
{
  double d_SinA = 0.0;
  double d_CosA = 0.0;
  double d_SinB = 0.0;
  double d_CosB = 0.0;

  if ((NULL == pd_SvPos) ||
    (NULL == pd_SvVCorr))
  {
    return;
  }

  d_SinA = sin(d_Alpha);
  d_CosA = cos(d_Alpha);
  d_SinB = sin(d_Beta);
  d_CosB = cos(d_Beta);

  pd_SvVCorr[0] = (-d_SinA * pd_SvPos[0] + d_CosA * d_CosB * pd_SvPos[1] +
    d_CosA * d_SinB * pd_SvPos[2]) * pd_EarthRateRps;

  pd_SvVCorr[1] = (-d_CosA * pd_SvPos[0] - d_SinA * d_CosB * pd_SvPos[1] -
    d_SinA * d_SinB * pd_SvPos[2]) * pd_EarthRateRps;

  pd_SvVCorr[2] = 0;

  return;
}

/**
 * @brief Compute GPS Satellite Position, Velocity, Clock Bias and Clock drift
 * @param[in]   gps_eph - GPS L-NAV ephemeris parameter
 * @param[in]   t - time of transmit
 * @param[out]  pz_SatPvt - Satellite Position, Velocity and Time
 * @return      None
 */
void sd_ComputeGpsSvPosVelClk(GpsEphemeris_t* gps_eph, double t,
  gnss_SatPosVelClk_t* pz_SatPvt)
{
  double d_SvPos[4] = { 0 };
  double d_SvVel[4] = { 0 };
  double d_Gravitational = MU_GPS;
  double OmegaEarthRate = OMGE_GPS;
  pz_SatPvt->q_iode = gps_eph->iode;

  /* Ephemeris Time from ephemeris reference epoch */
  double tk = t - gps_eph->toe;
  double tc = t - gps_eph->toc;
  double d_WeekSec = 1.0 * (double)WEEK_SEC;
  double d_HalfWeekSec = 0.5 * d_WeekSec;

  if (tk > d_HalfWeekSec)
  {
    tk -= d_WeekSec;
  }
  else if (tk < -d_HalfWeekSec)
  {
    tk += d_WeekSec;
  }

  if (tc > d_HalfWeekSec)
  {
    tc -= d_WeekSec;
  }
  else if (tc < -d_HalfWeekSec)
  {
    tc += d_WeekSec;
  }

  /*  Eccentricity  */
  double e = gps_eph->e;
  double d_Boe = sqrt(1.0 - e * e);

  /* Semi-major axis */
  double axis = gps_eph->sqrt_A * gps_eph->sqrt_A;
  
  /* Corrected mean motion */
  double n = sqrt(d_Gravitational / (axis * axis * axis)) + gps_eph->DeltaN;
    
  /* Mean anomaly */
  double Mk = gps_eph->M0 + n * tk;
  Mk = fmod(Mk, 2.0 * PI);

  /* Kepler equation( Mk = Ek -e sinEk) may be solved for Eccentric
     anomaly (Ek) by iteration: */
  double Ek = Mk;
  double E = Mk;
  int iterCnt = 0;
  do {
    iterCnt++;
    Ek = E;
    E = E + (Mk - E + e * sin(E)) / (1 - e * cos(E));
    if (fabs(Ek - E) < RELATIVE_TOL_KEPLER)
    {
      Ek = E;
      break;
    }
  } while (iterCnt < 30);
  
  double SinEk = sin(Ek);
  double CosEk = cos(Ek);
  double Tftr = (1 - e * CosEk);
  double TftrInv = 1 / Tftr;

  /* Compute the True anomaly with Earth as the focus. */
  double vk = atan2((d_Boe * SinEk), (CosEk - e));

  /* Argument of latitude */
  double Phik = vk + gps_eph->Omega;
  double Cos2Phik = cos(2.0 * Phik);
  double Sin2Phik = sin(2.0 * Phik);
  
  /* Second Harmonic Perturbations */
  double delta_uk = gps_eph->cuc * Cos2Phik + gps_eph->cus * Sin2Phik;
  double delta_rk = gps_eph->crc * Cos2Phik + gps_eph->crs * Sin2Phik;
  double delta_ik = gps_eph->cic * Cos2Phik + gps_eph->cis * Sin2Phik;
  
  /* Corrected Argument of Latitude */
  double uk = Phik + delta_uk;
  
  /* Corrected Radius */
  double rk = axis * Tftr + delta_rk;
  
  /* Corrected Inclination */
  double ik = gps_eph->i0 + gps_eph->idot * tk + delta_ik;
  double CosIk = cos(ik);
  double SinIk = sin(ik);

  /* Positions in orbital plane */
  double CosUk = cos(uk);
  double SinUk = sin(uk);
  double d_Xprime = rk * CosUk;
  double d_Yprime = rk * SinUk;
  
  /* Longitude of Ascending Node Rate */
  double OmegaDot = gps_eph->OmegaDot - OmegaEarthRate;
  /* Corrected longitude of ascending node */
  double Omegak = gps_eph->Omega0 + OmegaDot*tk - OmegaEarthRate * gps_eph->toe;
  Omegak = fmod(Omegak, 2.0 * PI);

  /* Earth-fixed coordinates */
  double CosOk = cos(Omegak);
  double SinOk = sin(Omegak);
  d_SvPos[0] = d_Xprime * CosOk - d_Yprime * CosIk * SinOk;
  d_SvPos[1] = d_Xprime * SinOk + d_Yprime * CosIk * CosOk;
  d_SvPos[2] = d_Yprime * SinIk;
  d_SvPos[3] = gps_eph->af0 + tc * (gps_eph->af1 + tc * gps_eph->af2);
  d_SvPos[3] *= CLIGHT;

  /* Computer Satellite Velocity */

  /* True Anomaly Rate */
  double Ek_dot = n * TftrInv;
  double vk_dot = d_Boe * n * TftrInv * TftrInv;

  double vs2phi =  2 * vk_dot * Cos2Phik;
  double vc2phi = -2 * vk_dot * Sin2Phik;
  /* Corrected Inclination Angle Rate */
  double ik_dot = vs2phi * gps_eph->cis + vc2phi * gps_eph->cic + gps_eph->idot;
  /* Corrected Argument of Latitude Rate */
  double uk_dot = vs2phi * gps_eph->cus + vc2phi * gps_eph->cuc + vk_dot;
  /* Corrected Radius Rate */
  double rk_dot = vs2phi * gps_eph->crs + vc2phi * gps_eph->crc + axis * e * SinEk * Ek_dot;

  /* Velocities in orbital plane */
  double d_VXprime = rk_dot * CosUk - d_Yprime * uk_dot;
  double d_VYprime = rk_dot * SinUk + d_Xprime * uk_dot;
  
  /* Earth-Fixed x velocity (m/s) */
  d_SvVel[0] = d_VXprime * CosOk - d_VYprime * CosIk * SinOk
    - d_SvPos[1] * OmegaDot + d_SvPos[2] * SinOk * ik_dot;
  d_SvVel[1] = d_VXprime * SinOk + d_VYprime * CosIk * CosOk
    + d_SvPos[0] * OmegaDot - d_SvPos[2] * CosOk * ik_dot;
  d_SvVel[2] = d_VYprime * SinIk + d_Yprime * CosIk * ik_dot;
  d_SvVel[3] = (gps_eph->af1 + gps_eph->af2 * tc * 2) * CLIGHT;

  /* relativity correction */
  d_SvPos[3] += -2.0*(d_SvPos[0] * d_SvVel[0] + d_SvPos[1] * d_SvVel[1] + d_SvPos[2] * d_SvVel[2]) / CLIGHT;
  
  for (int i = 0; i < 4; i++)
  {
    pz_SatPvt->d_satPosClk[i] = d_SvPos[i];
    pz_SatPvt->d_satVelClk[i] = d_SvVel[i];
  }

  return;
}

static void sd_ComputeGloSvPosVelClk(GloEphemeris_t* glo_eph, double t,
  gnss_SatPosVelClk_t* pz_SatPvt)
{
  return;
}

/**
 * @brief
 * @param[in] u_constellation
 * @param[in] u_svid
 * @param[in] pz_t tot, time of satellite transmit, tor-P/c
 * @return satellite clock
 */
static double sd_SatClk(uint8_t u_constellation, uint8_t u_svid, const GpsTime_t* pz_t)
{
  double af[3] = {0.0};
  double d_clk = 0.0;
  gnss_Ephemeris_t z_ephemeris = {0};

  BOOL status = sd_api_GnssEphemeris_Get(pz_t, u_constellation, u_svid, -1, &z_ephemeris);
  if (status)
  {
    double tc = tm_GpsTimeDiff(pz_t, &z_ephemeris.z_tocOfGpst);
    double ts = tc;

    if (C_GNSS_GPS == z_ephemeris.u_constellation)
    {
      af[0] = z_ephemeris.eph.z_gpsEph.af0;
      af[1] = z_ephemeris.eph.z_gpsEph.af1;
      af[2] = z_ephemeris.eph.z_gpsEph.af2;
    }
    else if (C_GNSS_GAL == z_ephemeris.u_constellation)
    {
      af[0] = z_ephemeris.eph.z_galEph.af0;
      af[1] = z_ephemeris.eph.z_galEph.af1;
      af[2] = z_ephemeris.eph.z_galEph.af2;
    }
    else if (C_GNSS_BDS3 == z_ephemeris.u_constellation || C_GNSS_BDS2 == z_ephemeris.u_constellation)
    {
      af[0] = z_ephemeris.eph.z_bdsEph.af0;
      af[1] = z_ephemeris.eph.z_bdsEph.af1;
      af[2] = z_ephemeris.eph.z_bdsEph.af2;
    }

    for (int i = 0; i < 2; ++i)
    {
      tc = ts - (af[0] + tc * (af[1] + tc * af[2]));
    }
    d_clk = (af[0] + tc * (af[1] + tc * af[2]));
  }
  return d_clk;
}

/**
 * @brief Compute BDS Satellite Position, Velocity, Clock Bias and Clock drift
 * @param[in]   bds_eph - BDS D1 ephemeris parameter
 * @param[in]   t - time of transmit
 * @param[out]  pz_SatPvt - Satellite Position, Velocity and Time
 * @return      None
 */
void sd_ComputeBdsSvPosVelClk(BdsEphemeris_t* bds_eph, double t,
  gnss_SatPosVelClk_t* pz_SatPvt)
{
  double d_SvPos[4] = { 0 };
  double d_SvVel[4] = { 0 };
  double d_Gravitational = MU_BDS;
  double d_OmegaEarthRate = OMGE_BDS;
  pz_SatPvt->q_iode = bds_eph->iode;

  /* Ephemeris Time from ephemeris reference epoch */
  double tk = t - bds_eph->toe - 14;
  double tc = t - bds_eph->toc - 14;
  double d_WeekSec = 1.0 * (double)WEEK_SEC;
  double d_HalfWeekSec = 0.5 * d_WeekSec;

  if (tk > d_HalfWeekSec)
  {
    tk -= d_WeekSec;
  }
  else if (tk < -d_HalfWeekSec)
  {
    tk += d_WeekSec;
  }

  if (tc > d_HalfWeekSec)
  {
    tc -= d_WeekSec;
  }
  else if (tc < -d_HalfWeekSec)
  {
    tc += d_WeekSec;
  }

  /*  Eccentricity  */
  double e = bds_eph->e;
  double d_Boe = sqrt(1.0 - e * e);

  /* Semi-major axis */
  double axis = bds_eph->sqrt_A * bds_eph->sqrt_A;

  /* Corrected mean motion */
  double n = sqrt(d_Gravitational / (axis * axis * axis)) + bds_eph->DeltaN;

  /* Mean anomaly */
  double Mk = bds_eph->M0 + n * tk;
  Mk = fmod(Mk, 2.0 * PI);

  /* Kepler equation( Mk = Ek -e sinEk) may be solved for Eccentric
     anomaly (Ek) by iteration: */
  double Ek = Mk;
  double E = Mk;
  int iterCnt = 0;
  do {
    iterCnt++;
    Ek = E;
    E = E + (Mk - E + e * sin(E)) / (1 - e * cos(E));
    if (fabs(Ek - E) < RELATIVE_TOL_KEPLER)
    {
      Ek = E;
      break;
    }
  } while (iterCnt < 30);

  double SinEk = sin(Ek);
  double CosEk = cos(Ek);
  double Tftr = (1 - e * CosEk);
  double TftrInv = 1 / Tftr;

  /* Compute the True anomaly with Earth as the focus. */
  double vk = atan2((d_Boe * SinEk), (CosEk - e));

  /* Argument of latitude */
  double Phik = vk + bds_eph->Omega;
  double Cos2Phik = cos(2.0 * Phik);
  double Sin2Phik = sin(2.0 * Phik);

  /* Second Harmonic Perturbations */
  double delta_uk = bds_eph->cuc * Cos2Phik + bds_eph->cus * Sin2Phik;
  double delta_rk = bds_eph->crc * Cos2Phik + bds_eph->crs * Sin2Phik;
  double delta_ik = bds_eph->cic * Cos2Phik + bds_eph->cis * Sin2Phik;

  /* Corrected Argument of Latitude */
  double uk = Phik + delta_uk;

  /* Corrected Radius */
  double rk = axis * Tftr + delta_rk;

  /* Corrected Inclination */
  double ik = bds_eph->i0 + bds_eph->idot * tk + delta_ik;
  double CosIk = cos(ik);
  double SinIk = sin(ik);

  /* Positions in orbital plane */
  double CosUk = cos(uk);
  double SinUk = sin(uk);
  double d_Xprime = rk * CosUk;
  double d_Yprime = rk * SinUk;
  
  /* Longitude of Ascending Node Rate */
  double OmegaDot = 0.0;
  if (BDS_ID_MEO(bds_eph->svid))
  {
    OmegaDot = bds_eph->OmegaDot - d_OmegaEarthRate;
  }
  else if (BDS_ID_GEO(bds_eph->svid))
  {
    OmegaDot = bds_eph->OmegaDot;
  }
  /* Corrected longitude of ascending node */
  double Omegak = bds_eph->Omega0 + OmegaDot * tk - d_OmegaEarthRate * bds_eph->toe;
  Omegak = fmod(Omegak, 2.0 * PI);
  /* Earth-fixed coordinates */
  double CosOk = cos(Omegak);
  double SinOk = sin(Omegak);

  double orbital_xyz[3] = { d_Xprime, d_Yprime, 0.0 };
  double d_GkPos[3] = { 0 };
  sd_cvt_UserDefinedXyz2Ecef(-Omegak, -ik, orbital_xyz, d_GkPos);
  memcpy(d_SvPos, d_GkPos, sizeof(d_GkPos));
  if (BDS_ID_GEO(bds_eph->svid))
  {
    sd_cvt_UserDefinedXyz2Ecef(d_OmegaEarthRate * tk, (-5.0 * DEG2RAD), d_GkPos, d_SvPos);
  }

  /* Computer Satellite Velocity */

  /* True Anomaly Rate */
  double Ek_dot = n * TftrInv;
  double vk_dot = d_Boe * n * TftrInv * TftrInv;

  double vs2phi =  2 * vk_dot * Cos2Phik;
  double vc2phi = -2 * vk_dot * Sin2Phik;
  /* Corrected Inclination Angle Rate */
  double ik_dot = vs2phi * bds_eph->cis + vc2phi * bds_eph->cic + bds_eph->idot;
  /* Corrected Argument of Latitude Rate */
  double uk_dot = vs2phi * bds_eph->cus + vc2phi * bds_eph->cuc + vk_dot;
  /* Corrected Radius Rate */
  double rk_dot = vs2phi * bds_eph->crs + vc2phi * bds_eph->crc + axis * e * SinEk * Ek_dot;

  /* Velocities in orbital plane */
  double d_VXprime = rk_dot * CosUk - d_Yprime * uk_dot;
  double d_VYprime = rk_dot * SinUk + d_Xprime * uk_dot;

  /* Earth-Fixed x velocity (m/s) */
  double d_GkVel[3] = { 0 };

  d_GkVel[0] = d_VXprime * CosOk - d_VYprime * CosIk * SinOk
    - d_GkPos[1] * OmegaDot + d_GkPos[2] * SinOk * ik_dot;
  d_GkVel[1] = d_VXprime * SinOk + d_VYprime * CosIk * CosOk
    + d_GkPos[0] * OmegaDot - d_GkPos[2] * CosOk * ik_dot;
  d_GkVel[2] = d_VYprime * SinIk + d_Yprime * CosIk * ik_dot;

  if (BDS_ID_GEO(bds_eph->svid))
  {
    sd_cvt_UserDefinedXyz2Ecef(d_OmegaEarthRate * tk, (-5.0 * DEG2RAD),
      d_GkVel, d_SvVel);
    ComputeGeoSatVel2ndCorr(d_OmegaEarthRate * tk, (-5.0 * DEG2RAD), d_GkPos, d_OmegaEarthRate, d_GkVel);
    d_SvVel[0] += d_GkVel[0];
    d_SvVel[1] += d_GkVel[1];
  }
  else
  {
    d_SvVel[0] = d_GkVel[0];
    d_SvVel[1] = d_GkVel[1];
    d_SvVel[2] = d_GkVel[2];
  }

  d_SvPos[3] = bds_eph->af0 + tc * (bds_eph->af1 + tc * bds_eph->af2)
    + BDS_F_RELATIVITY_CORR * e * bds_eph->sqrt_A * SinEk;
  d_SvPos[3] *= CLIGHT;
  d_SvVel[3] = (bds_eph->af1 + bds_eph->af2 * tc * 2) * CLIGHT;
  
  for (int i = 0; i < 4; i++)
  {
    pz_SatPvt->d_satPosClk[i] = d_SvPos[i];
    pz_SatPvt->d_satVelClk[i] = d_SvVel[i];
  }

  return;
}

/**
 * @brief Compute GAL Satellite Position, Velocity, Clock Bias and Clock drift
 * @param[in]   gal_eph - GAL I-NAV ephemeris parameter
 * @param[in]   t - time of transmit
 * @param[out]  pz_SatPvt - Satellite Position, Velocity and Time
 * @return      None
 */
void sd_ComputeGalSvPosVelClk(GalEphemeris_t* gal_eph, double t,
  gnss_SatPosVelClk_t* pz_SatPvt)
{
  double d_SvPos[4] = { 0 };
  double d_SvVel[4] = { 0 };
  double d_Gravitational = MU_GAL;
  double d_OmegaEarthRate = OMGE_GAL;
  pz_SatPvt->q_iode = gal_eph->iode;

  /* Ephemeris Time from ephemeris reference epoch */
  double tk = t - gal_eph->toe;
  double tc = t - gal_eph->toc;
  double d_WeekSec = 1.0 * (double)WEEK_SEC;
  double d_HalfWeekSec = 0.5 * d_WeekSec;

  if (tk > d_HalfWeekSec)
  {
    tk -= d_WeekSec;
  }
  else if (tk < -d_HalfWeekSec)
  {
    tk += d_WeekSec;
  }

  if (tc > d_HalfWeekSec)
  {
    tc -= d_WeekSec;
  }
  else if (tc < -d_HalfWeekSec)
  {
    tc += d_WeekSec;
  }

  /*  Eccentricity  */
  double e = gal_eph->e;
  double d_Boe = sqrt(1.0 - e * e);

  /* Semi-major axis */
  double axis = gal_eph->sqrt_A * gal_eph->sqrt_A;

  /* Corrected mean motion */
  double n = sqrt(d_Gravitational / (axis * axis * axis)) + gal_eph->DeltaN;

  /* Mean anomaly */
  double Mk = gal_eph->M0 + n * tk;
  Mk = fmod(Mk, 2.0 * PI);

  /* Kepler equation( Mk = Ek -e sinEk) may be solved for Eccentric
     anomaly (Ek) by iteration: */
  double Ek = Mk;
  double E = Mk;
  int iterCnt = 0;
  do {
    iterCnt++;
    Ek = E;
    E = E + (Mk - E + e * sin(E)) / (1 - e * cos(E));
    if (fabs(Ek - E) < RELATIVE_TOL_KEPLER)
    {
      Ek = E;
      break;
    }
  } while (iterCnt < 30);

  double SinEk = sin(Ek);
  double CosEk = cos(Ek);
  double Tftr = (1 - e * CosEk);
  double TftrInv = 1 / Tftr;

  /* Compute the True anomaly with Earth as the focus. */
  double vk = atan2((d_Boe * SinEk), (CosEk - e));

  /* Argument of latitude */
  double Phik = vk + gal_eph->Omega;
  double Cos2Phik = cos(2.0 * Phik);
  double Sin2Phik = sin(2.0 * Phik);

  /* Second Harmonic Perturbations */
  double delta_uk = gal_eph->cuc * Cos2Phik + gal_eph->cus * Sin2Phik;
  double delta_rk = gal_eph->crc * Cos2Phik + gal_eph->crs * Sin2Phik;
  double delta_ik = gal_eph->cic * Cos2Phik + gal_eph->cis * Sin2Phik;

  /* Corrected Argument of Latitude */
  double uk = Phik + delta_uk;

  /* Corrected Radius */
  double rk = axis * Tftr + delta_rk;

  /* Corrected Inclination */
  double ik = gal_eph->i0 + gal_eph->idot * tk + delta_ik;
  double CosIk = cos(ik);
  double SinIk = sin(ik);

  /* Positions in orbital plane */
  double CosUk = cos(uk);
  double SinUk = sin(uk);
  double d_Xprime = rk * CosUk;
  double d_Yprime = rk * SinUk;

  /* Longitude of Ascending Node Rate */
  double OmegaDot = gal_eph->OmegaDot - d_OmegaEarthRate;
  /* Corrected longitude of ascending node */
  double Omegak = gal_eph->Omega0 + OmegaDot * tk - d_OmegaEarthRate * gal_eph->toe;
  Omegak = fmod(Omegak, 2.0 * PI);
  /* Earth-fixed coordinates */
  double CosOk = cos(Omegak);
  double SinOk = sin(Omegak);

  double orbital_xyz[3] = { d_Xprime, d_Yprime, 0.0 };
  sd_cvt_UserDefinedXyz2Ecef(-Omegak, -ik, orbital_xyz, d_SvPos);
  d_SvPos[3] = gal_eph->af0 + tc * (gal_eph->af1 + tc * gal_eph->af2)
               + GAL_F_RELATIVITY_CORR * e * gal_eph->sqrt_A * SinEk;
  d_SvPos[3] *= CLIGHT;

  /* Computer Satellite Velocity */

  /* True Anomaly Rate */
  double Ek_dot = n * TftrInv;
  double vk_dot = d_Boe * n * TftrInv * TftrInv;

  double vs2phi = 2 * vk_dot * Cos2Phik;
  double vc2phi = -2 * vk_dot * Sin2Phik;
  /* Corrected Inclination Angle Rate */
  double ik_dot = vs2phi * gal_eph->cis + vc2phi * gal_eph->cic + gal_eph->idot;
  /* Corrected Argument of Latitude Rate */
  double uk_dot = vs2phi * gal_eph->cus + vc2phi * gal_eph->cuc + vk_dot;
  /* Corrected Radius Rate */
  double rk_dot = vs2phi * gal_eph->crs + vc2phi * gal_eph->crc + axis * e * SinEk * Ek_dot;

  /* Velocities in orbital plane */
  double d_VXprime = rk_dot * CosUk - d_Yprime * uk_dot;
  double d_VYprime = rk_dot * SinUk + d_Xprime * uk_dot;

  /* Earth-Fixed x velocity (m/s) */
  d_SvVel[0] = d_VXprime * CosOk - d_VYprime * CosIk * SinOk
    - d_SvPos[1] * OmegaDot + d_SvPos[2] * SinOk * ik_dot;
  d_SvVel[1] = d_VXprime * SinOk + d_VYprime * CosIk * CosOk
    + d_SvPos[0] * OmegaDot - d_SvPos[2] * CosOk * ik_dot;
  d_SvVel[2] = d_VYprime * SinIk + d_Yprime * CosIk * ik_dot;
  d_SvVel[3] = (gal_eph->af1 + gal_eph->af2 * tc * 2) * CLIGHT;

  for (int i = 0; i < 4; i++)
  {
    pz_SatPvt->d_satPosClk[i] = d_SvPos[i];
    pz_SatPvt->d_satVelClk[i] = d_SvVel[i];
  }

  return;
}

/**
 * @brief calculate BDS iode
 * @param gpst GPS time
 * @return
 */
static int32_t sd_BdsIode(const GpsTime_t* gpst)
{
  return (int32_t) (gpst->q_towMsec * TIME_MSEC_INV) / 720 % 240;
}

/**
 * @brief Save broadcast ephemeris
 * @param u_constellation
 * @param u_svid
 * @param ephemeris
 */
static void sd_Ephemeris_Save(uint8_t u_constellation, uint8_t u_svid, const gnss_Ephemeris_t* ephemeris)
{
  int32_t q_iode_in = -1;
  int32_t q_iode_old = -1;
  int8_t  s_idx = -1;
  double dtm = 0.0;
  uint32_t u_sat_idx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);

  if (ALL_GNSS_SYS_SV_NUMBER == u_sat_idx)
  {
    return;
  }

  for (int8_t i = 0; i < GNSS_EPHEMERIS_CACHE_NUM; ++i)
  {
    if (NULL == gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i])
    {
      gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i] = (gnss_Ephemeris_t*) OS_MALLOC(sizeof(gnss_Ephemeris_t));
      if (NULL != gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i])
      {
        s_idx = i;
        break;
      }
    }
  }

  // find the old one and replace
  double dt = -999999.0;
  if (-1 == s_idx)
  {
     for (int8_t i = 0; i < GNSS_EPHEMERIS_CACHE_NUM; ++i)
    {
      if (NULL == gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i])
      {
        continue;
      }
      dtm = tm_GpsTimeDiff(&ephemeris->z_toeOfGpst, &gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i]->z_toeOfGpst);
      switch (ephemeris->u_constellation)
      {
        case C_GNSS_GPS:
          q_iode_in = ephemeris->eph.z_gpsEph.iode;
          q_iode_old = gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i]->eph.z_gpsEph.iode;
          break;
        case C_GNSS_BDS2:
        case C_GNSS_BDS3:
          q_iode_in = ephemeris->eph.z_bdsEph.iode;
          q_iode_old = gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i]->eph.z_bdsEph.iode;
          break;
        case C_GNSS_GAL:
          q_iode_in = ephemeris->eph.z_galEph.iode;
          q_iode_old = gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][i]->eph.z_galEph.iode;
          break;
        default:
          break;
      }
      if ((dtm > dt) && (q_iode_in != q_iode_old))
      {
        dt = dtm;
        s_idx = i;
      }
      if ((fabs(dtm) <= FABS_ZEROS) && (q_iode_in == q_iode_old))
      {
        s_idx = -1;
        break;
      }
    }
  }
  if (-1 != s_idx)
  {
    M_SET_BIT(gz_GnssEphemeris_Pool.t_gnssEphMask[u_constellation], u_svid);
    memcpy(gz_GnssEphemeris_Pool.pz_gnssEph[u_sat_idx][s_idx], ephemeris, sizeof(gnss_Ephemeris_t)); // copy

    sm_api_GnssEphemeris_Report((gnss_Ephemeris_t*)ephemeris);

   // LOGI(TAG_SD, "Inject Ephemeris %d, %d, %d, %.0f\n", u_constellation, u_svid, s_idx,
     //    ephemeris->z_toeOfGpst.q_towMsec * TIME_MSEC_INV);
  }
}

/**
 * @brief delete expire ephemeris
 * @param gpst GPST
 */
void sd_Ephemeris_refresh(const GpsTime_t* gpst)
{
  if ((0 == gpst->q_towMsec) && (0 == gpst->w_week))
  {
    return;
  }

  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    for (int j = 0; j < GNSS_EPHEMERIS_CACHE_NUM; ++j)
    {
      if (NULL == gz_GnssEphemeris_Pool.pz_gnssEph[i][j])
      {
        continue;
      }
      double dt = tm_GpsTimeDiff(gpst, &gz_GnssEphemeris_Pool.pz_gnssEph[i][j]->z_toeOfGpst);
      if (dt > GNSS_EPHEMERIS_CACHE_EXPIRE)
      {
        OS_FREE(gz_GnssEphemeris_Pool.pz_gnssEph[i][j]);
      }
    }
  }

  // mask
  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    uint8_t u_num = 0;
    for (int j = 0; j < GNSS_EPHEMERIS_CACHE_NUM; ++j)
    {
      if (NULL == gz_GnssEphemeris_Pool.pz_gnssEph[i][j])
      {
        u_num++;
      }
    }
    if (GNSS_EPHEMERIS_CACHE_NUM == u_num )
    {
      uint8_t u_constellation = 0;
      uint8_t u_svid = 0;

      u_svid = gnss_cvt_SvIndex2Svid(i, &u_constellation);
      if (ALL_GNSS_SYS_SV_NUMBER == u_svid)
      {
        continue;
      }
      M_CLEAR_BIT(gz_GnssEphemeris_Pool.t_gnssEphMask[u_constellation], u_svid);
    }
  }
}

/**
 * @brief SD module Ephermeris Put
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
static void sd_Ephemeris_Put(ipc_t* p_ipc)
{
  gnss_Ephemeris_t* ephemeris = (gnss_Ephemeris_t*)p_ipc->p_data;

  uint8_t u_svid = 0;
  uint8_t u_constellation = ephemeris->u_constellation;
  GpsTime_t z_toeOfGpst = { 0 };
  GpsTime_t z_tocOfGpst = { 0 };

  switch (u_constellation)
  {
  case C_GNSS_GPS:
  {
    tm_cvt_SetGpst(&z_toeOfGpst, ephemeris->eph.z_gpsEph.week, ephemeris->eph.z_gpsEph.toe);
    tm_cvt_SetGpst(&z_tocOfGpst, ephemeris->eph.z_gpsEph.week, ephemeris->eph.z_gpsEph.toc);
    u_svid = ephemeris->eph.z_gpsEph.svid;
    break;
  }
  case C_GNSS_GLO:
    u_svid = 0;
    break;
  case C_GNSS_BDS2:
  case C_GNSS_BDS3:
  {
    BdsTime_t bdt = {0};
    tm_cvt_SetBdt(&bdt, ephemeris->eph.z_bdsEph.week, ephemeris->eph.z_bdsEph.toe);
    tm_cvt_BdtToGpst(&bdt, &z_toeOfGpst);
    tm_cvt_SetBdt(&bdt, ephemeris->eph.z_bdsEph.week, ephemeris->eph.z_bdsEph.toc);
    tm_cvt_BdtToGpst(&bdt, &z_tocOfGpst);
    ephemeris->eph.z_bdsEph.iode = sd_BdsIode(&z_toeOfGpst);
    u_svid = ephemeris->eph.z_bdsEph.svid;
    break;
  }
  case C_GNSS_GAL:
  {
    GalTime_t gst = { 0 };
    tm_cvt_SetGst(&gst, ephemeris->eph.z_galEph.week, ephemeris->eph.z_galEph.toe);
    tm_cvt_GstToGpst(&gst, &z_toeOfGpst);
    tm_cvt_SetGst(&gst, ephemeris->eph.z_galEph.week, ephemeris->eph.z_galEph.toc);
    tm_cvt_GstToGpst(&gst, &z_tocOfGpst);
    u_svid = ephemeris->eph.z_galEph.svid;
    break;
  }
  case C_GNSS_QZS:
  {
    tm_cvt_SetGpst(&z_toeOfGpst, ephemeris->eph.z_gpsEph.week, ephemeris->eph.z_gpsEph.toe);
    tm_cvt_SetGpst(&z_tocOfGpst, ephemeris->eph.z_gpsEph.week, ephemeris->eph.z_gpsEph.toc);
    u_svid = ephemeris->eph.z_gpsEph.svid - QZS_ID_OFFSET;
    break;
  }
  default:
    u_svid = 0;
    break;
  }
  uint32_t u_satIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);

  if (ALL_GNSS_SYS_SV_NUMBER == u_satIdx)
  {
    return;
  }

  /* Save ephemeris */
  ephemeris->z_toeOfGpst = z_toeOfGpst;
  ephemeris->z_tocOfGpst = z_tocOfGpst;
  sd_Ephemeris_Save(u_constellation, u_svid, ephemeris);

  return;
}

/**
 * @brief calculate ploynomial parameter that satellite position, clok, satellite velocity, clock drift
 * @param[in] data y
 * @param[in] num number of y
 * @param[in] order ploynomial order
 * @param[out] ploynomial ploynomial parameter
 * @return
 */
static BOOL sd_Ploynomial(const double* data, int num, int order, double* ploynomial)
{
  BOOL status = FALSE;
  matrix_t* mat_Van = NULL;
  matrix_t* mat_InvVan = NULL;
  double y_new[5] = {0};
  double y[5] = {0};

  if (order >= 4)
  { return FALSE; }
  for (uint8_t i = 0; i < num; ++i)
  {
    y[i] = data[i];
  }

  double Vandermonde4[4][4] = {{1, 0,  0,    0},
                               {1, 30, 900,  27000},
                               {1, 60, 3600, 216000},
                               {1, 90, 8100, 729000}};
  double Vandermonde3[4][3] = {{1, 0,  0},
                               {1, 30, 900},
                               {1, 60, 3600},
                               {1, 90, 8100}};
  double Vandermonde2[4][2] = {{1, 0},
                               {1, 30},
                               {1, 60},
                               {1, 90}};
  double InvVandermonde4[4][4] = {{1.0,              0.0,              0.0,              0.0},
                                  {-4.95e+00 / 81.0, 0.1,              -0.05,            9.00e-01 / 81.0},
                                  {9.00e-02 / 81.0,  -2.25e-01 / 81.0, 1.80e-01 / 81.0,  -4.50e-02 / 81.0},
                                  {-5.00e-04 / 81.0, 1.50e-03 / 81.0,  -1.50e-03 / 81.0, 5.00e-04 / 81.0}};
  double InvVandermonde3[3][4] = {{9.50000000e-01,  1.50000000e-01,  -1.50000000e-01, 5.00000000e-02},
                                  {-3.50000000e-02, 2.16666667e-02,  2.83333333e-02,  -1.50000000e-02},
                                  {2.77777778e-04,  -2.77777778e-04, -2.77777778e-04, 2.77777778e-04}};
  double InvVandermonde2[2][4] = {{0.7,   0.4,         0.1,        -0.2},
                                  {-0.01, -0.00333333, 0.00333333, 0.01}};

  matrix_t* mat_y = matrix_new_from_buffer(num, 1, y);
  matrix_t* mat_x = matrix_new_from_buffer(order + 1, 1, ploynomial);
  matrix_t* mat_yd = matrix_new_from_buffer(num, 1, y_new);

  switch (order)
  {
    case 1:
      mat_InvVan = matrix_new_from_buffer(2, 4, (double*) InvVandermonde2);
      mat_Van = matrix_new_from_buffer(4, 2, (double*) Vandermonde2);
      break;
    case 2:
      mat_InvVan = matrix_new_from_buffer(3, 4, (double*) InvVandermonde3);
      mat_Van = matrix_new_from_buffer(4, 3, (double*) Vandermonde3);
      break;
    case 3:
      mat_InvVan = matrix_new_from_buffer(4, 4, (double*) InvVandermonde4);
      mat_Van = matrix_new_from_buffer(4, 4, (double*) Vandermonde4);
      break;
    default:
      break;
  }
  if (mat_InvVan != NULL && mat_Van != NULL)
  {
    matrix_mul(mat_InvVan, mat_y, mat_x); /* x = A^-1 * y */
    matrix_mul(mat_Van, mat_x, mat_yd); // A*x = yd
    matrix_minus(mat_yd, mat_y);  /* sigma */
    double sigma = matrix_col_vector_norm(mat_yd, 1, 1, num);
    if (sigma <= 0.01)
    {
      status = TRUE;
    }
    else
    {
      status = FALSE;
    }
  }

  matrix_free(&mat_y);
  matrix_free(&mat_x);
  matrix_free(&mat_yd);
  matrix_free(&mat_Van);
  matrix_free(&mat_InvVan);
  return status;
}

/**
 * @brief Compute Satellite Position/Velocity/Clock Polynomial
 * @param[in]   constellation
 * @param[in]   svid
 * @param[in]   t time of ploynominal
 * @param[in]   pz_SatPosVelClkPolynomial
 * @return      None
 */
static BOOL sd_ComputeSatPvtPolynomial(uint8_t constellation, uint8_t svid, GpsTime_t* t,
                                       gnss_SatPosVelClkPolynomial_t* pz_SatPosVelClkPolynomial)
{
  BOOL ploy_pos_status[4] = {0};
  BOOL ploy_vel_status[4] = {0};
  BOOL status = TRUE;
  double fit_interval = SAT_POLYNOMIAL_INTERVAL;
  int num_measure = 4;
  int n_suc = 0;
  GpsTime_t tot = {0};
  gnss_SatPosVelClk_t z_satPosVelClk[4] = {0};
  double data_posck[4] = {0};
  double data_velcd[4] = {0};
  int32_t q_iode = -1;
  char sat_id[4] = {0};

  svid_SatString(svid, constellation, sat_id);

  /* get satellite position */
  for (uint8_t i = 0; i < num_measure; ++i)
  {
    tot = *t;
    tm_GpstTimeAdd(&tot, ((double)(SAT_POLYNOMIAL_INTERVAL)) * i);
    // do not correct the satellite clock
    if (sd_api_SatPositionFromBrdc(&tot, constellation, svid, q_iode, FALSE, &z_satPosVelClk[i]))
    {
      n_suc++;
      q_iode = z_satPosVelClk[i].q_iode;
    }
    else
    { break; }
  }

  /* must get 4 time`s satpos (SAT_POLYNOMIAL_INTERVAL/+30/+60/+90) */
  if (n_suc != num_measure)
  { status = FALSE; }
  else
  {
    status = TRUE;
    /* position & velocity */
    for (uint8_t i = 0; i < 3; ++i) // x/y/z
    {
      for (uint8_t j = 0; j < 4; ++j) // 0/30/60/90
      {
        data_posck[j] = z_satPosVelClk[j].d_satPosClk[i];
        data_velcd[j] = z_satPosVelClk[j].d_satVelClk[i];
      }
      ploy_pos_status[i] = sd_Ploynomial(data_posck, 4, 3, pz_SatPosVelClkPolynomial->d_posClkPolynomial[i]);
      ploy_vel_status[i] = sd_Ploynomial(data_velcd, 4, 2, pz_SatPosVelClkPolynomial->d_velClkPolynomial[i]);
    }

    /* clock and drift */
    for (uint8_t i = 0; i < 4; ++i) // (SAT_POLYNOMIAL_INTERVAL/+30/+60/+90)
    {
      data_posck[i] = z_satPosVelClk[i].d_satPosClk[3];
      data_velcd[i] = z_satPosVelClk[i].d_satVelClk[3];
    }
    ploy_pos_status[3] = sd_Ploynomial(data_posck, 4, 2, pz_SatPosVelClkPolynomial->d_posClkPolynomial[3]);
    ploy_vel_status[3] = sd_Ploynomial(data_velcd, 4, 1, pz_SatPosVelClkPolynomial->d_velClkPolynomial[3]);
    for (uint8_t i = 0; i < 4; ++i)
    {
      if (ploy_pos_status[i] == FALSE || ploy_vel_status[i] == FALSE)
      {
        status = FALSE;
        break;
      }
    }
    if (status)
    {
      pz_SatPosVelClkPolynomial->u_constellation = constellation;
      pz_SatPosVelClkPolynomial->u_svid = svid;
      pz_SatPosVelClkPolynomial->z_fitStart = *t;
      pz_SatPosVelClkPolynomial->z_fitEnd = pz_SatPosVelClkPolynomial->z_fitStart;
      tm_GpstTimeAdd(&pz_SatPosVelClkPolynomial->z_fitEnd, SAT_POLYNOMIAL_DURATION);
      pz_SatPosVelClkPolynomial->q_iode = z_satPosVelClk[0].q_iode;
    }
  }
  LOGI(TAG_SD, "Polynominal sat %s, %d, status:%d\n",sat_id, t->q_towMsec, status);
  return status;
}

/**
 * @brief Get satellite polynomial of Position, Velocity and Clock
 * @param[in]   constellation
 * @param[in]   svid
 * @param[out]  pz_SatPosVelClkPolynomial
 * @Note  This function has a mutex to support multi-task call.
 */
static BOOL sd_SatPosVelClkPolynomial_Get(uint8_t constellation, uint8_t svid,
  gnss_SatPosVelClkPolynomial_t* pz_SatPosVelClkPolynomial)
{
  if (any_Ptrs_Null(1, pz_SatPosVelClkPolynomial))
  {
    return FALSE;
  }

  if (FALSE == sd_hpp_task_ctrl.b_Init)
  {
    return FALSE;
  }

  BOOL b_success = FALSE;

  uint32_t u_satIdx = gnss_cvt_Svid2SvIndex(svid, constellation);
  if (ALL_GNSS_SYS_SV_NUMBER == u_satIdx)
  {
    return FALSE;
  }

  mutex_lock(&gz_GnssSatPvtPoly_Pool.z_mtx);

  if (M_IS_SET_BIT(gz_GnssSatPvtPoly_Pool.t_gnssSatPvtPolyMask[constellation], svid) &&
    (gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[u_satIdx] != NULL))
  {
    memcpy(pz_SatPosVelClkPolynomial, gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[u_satIdx],
      sizeof(gnss_SatPosVelClkPolynomial_t));
    b_success = TRUE;
  }

  mutex_unlock(&gz_GnssSatPvtPoly_Pool.z_mtx);
  return b_success;
}

static void sd_GnssEphemerisPool_SortOut(sd_SatPosVelClkRequest_t* pz_SatPvtRequest)
{
  // TBD
  return;
}

static void sd_GnssSatPvtPolyPool_SortOut(sd_SatPosVelClkRequest_t* pz_SatPvtRequest)
{
  // TBD
  return;
}

/**
 * @brief Request to compute satellite polynomial of Position, Velocity and Clock
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sd_SatPosVelClkPolynomial_Request(ipc_t* p_ipc)
{
  uint8_t u_constellation = 0;
  uint8_t u_svid = 0;
  if (NULL == p_ipc->p_data || sizeof(sd_SatPosVelClkRequest_t) != p_ipc->q_length)
  {
    return;
  }

  sd_SatPosVelClkRequest_t* pz_SatPvtRequest = (sd_SatPosVelClkRequest_t*)p_ipc->p_data;
  GpsTime_t* pz_ReqTime = &(pz_SatPvtRequest->z_ReqTime);

  if (pz_ReqTime->t_fullMsec == 0)
  {
    return;
  }

  /* delete expire ephemeris */
  sd_Ephemeris_refresh(pz_ReqTime);

  gnss_Ephemeris_t* pz_eph = OS_MALLOC(sizeof(gnss_Ephemeris_t));
  if (NULL == pz_eph)
  {
    return;
  }

  for (uint8_t u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; ++u_i)
  {
    u_constellation = 0;
    u_svid = gnss_cvt_SvIndex2Svid(u_i, &u_constellation);
    if(ALL_GNSS_SYS_SV_NUMBER == u_svid)
    {
      continue;
    }
    if (!(sd_api_GnssEphemeris_Get(&pz_SatPvtRequest->z_ReqTime, u_constellation, u_svid, -1, pz_eph)) ||
      (!M_IS_SET_BIT(gz_GnssEphemeris_Pool.t_gnssEphMask[u_constellation], u_svid)) ||
      (!M_IS_SET_BIT(pz_SatPvtRequest->t_gnssReqMask[u_constellation], u_svid)))
    {
      continue;
    }

    gnss_SatPosVelClkPolynomial_t z_SatPosVelClkPolynomial = { 0 };
    if (sd_ComputeSatPvtPolynomial(u_constellation, u_svid, pz_ReqTime, &z_SatPosVelClkPolynomial))
    {
      if (NULL == gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[u_i])
      {
        gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[u_i] = (gnss_SatPosVelClkPolynomial_t*)OS_MALLOC(sizeof(gnss_SatPosVelClkPolynomial_t));
        if (NULL == gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[u_i])
        {
          continue;
        }
      }
      M_SET_BIT(gz_GnssSatPvtPoly_Pool.t_gnssSatPvtPolyMask[u_constellation], u_svid);
      memcpy(gz_GnssSatPvtPoly_Pool.pz_gnssSatPvtPoly[u_i], &z_SatPosVelClkPolynomial, sizeof(gnss_SatPosVelClkPolynomial_t));
    }
    else
    {
      M_CLEAR_BIT(gz_GnssSatPvtPoly_Pool.t_gnssSatPvtPolyMask[u_constellation], u_svid);
    }
  }

  /* To free some out of date ephemeris and polynomial */
  sd_GnssEphemerisPool_SortOut(pz_SatPvtRequest);
  sd_GnssSatPvtPolyPool_SortOut(pz_SatPvtRequest);

  OS_FREE(pz_eph);
  
  return;
}


/**
 * @brief Sanity Check IPC 
 * @param[in]   p_ipc - pointer to IPC message
 * @return: TRUE - success
 *          FALSE - fail for any reason
 */
static BOOL sd_IpcCheck(ipc_t* p_ipc)
{
  if (TASK_INDEX_SD != p_ipc->u_dst_id)
  {
    return FALSE;
  }

  if (!((p_ipc->q_ipc_id > C_M_SD_IPC_BEGINNING) &&
    (p_ipc->q_ipc_id < C_M_SD_IPC_END)))
  {
    return FALSE;
  }

  return TRUE;
}

/**
 * @brief SD IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sd_Proc(ipc_t* p_ipc)
{
  if (FALSE == sd_IpcCheck(p_ipc))
  {
    return;
  }

  switch (p_ipc->q_ipc_id)
  {
  case C_M_SD_INIT:
    sd_Init(p_ipc);
    break;
  case C_M_SD_START:
    sd_Start(p_ipc);
    break;
  case C_M_SD_STOP:
    sd_Stop(p_ipc);
    break;
  case C_M_SD_RELEASE:
    sd_Release(p_ipc);
    break;
  case C_M_SD_EPHEMERIS_PUT:
    sd_Ephemeris_Put(p_ipc);
    break;
  case C_M_SD_SAT_PVT_POLY_REQUEST:
    sd_SatPosVelClkPolynomial_Request(p_ipc);
    break;
  default:
    break;
  }

  return;
}

/**
 * @brief SD IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* sd_task(void* args)
{
  ipc_t z_ipc;
  TaskIndex_e taskIndex = TASK_INDEX_SD;

  ipctask_t* t = ipctask_GetInstance(taskIndex);
  t->e_status = TASK_STATUS_ENABLE;
  ipctask_SetThreadId2TaskId(get_thread_id(), taskIndex);

  if(NULL != t->float_hard_enable){
    t->float_hard_enable();
  }

  while (t->e_status == TASK_STATUS_ENABLE)
  {
    if (ipctask_ReceiveMessage(taskIndex, &z_ipc))
    {
      sd_Proc(&z_ipc);
      ipctask_ReleaseMessage(&z_ipc);
    }
  }

  return NULL;
}

/**
 * @brief SD API for initilization
 * @return      None
 */
BOOL sd_api_Init()
{
  ipctask_SendMessage(TASK_INDEX_SD, C_M_SD_INIT, NULL, 0);
  return TRUE;
}

/**
 * @brief SD API for start
 * @return      None
 */
BOOL sd_api_Start()
{
  ipctask_SendMessage(TASK_INDEX_SD, C_M_SD_START, NULL, 0);
  return TRUE;
}

/**
 * @brief SD API for stop
 * @return      None
 */
BOOL sd_api_Stop()
{
  ipctask_SendMessage(TASK_INDEX_SD, C_M_SD_STOP, NULL, 0);
  return TRUE;
}

/**
 * @brief SD API for release
 * @return      None
 */
BOOL sd_api_Release()
{
  ipctask_SendMessage(TASK_INDEX_SD, C_M_SD_RELEASE, NULL, 0);
  return TRUE;
}

/**
 * @brief SD API Put Ephemeris
 * @return      None
 */
BOOL sd_api_Ephemeris_Put(gnss_Ephemeris_t* eph)
{
  eph->u_version = VERSION_GNSS_EPHEMERIS;
  ipctask_SendMessage(TASK_INDEX_SD, C_M_SD_EPHEMERIS_PUT, 
    (uint8_t*)eph, sizeof(gnss_Ephemeris_t));
  return TRUE;
}

/**
 * @brief SD API to request compute satellite PVT information
 * @param[in]   pz_SatPvtReq
 * @return      None
 */
void sd_api_SatPosVelClkPolynomial_Request(sd_SatPosVelClkRequest_t* pz_SatPvtReq)
{
  pz_SatPvtReq->u_version = VERSION_SAT_POS_VEL_CLK_REQ;
  ipctask_SendMessage(TASK_INDEX_SD, C_M_SD_SAT_PVT_POLY_REQUEST,
    (uint8_t*)pz_SatPvtReq, sizeof(sd_SatPosVelClkRequest_t));
  return;
}


/**
 * @brief Get satellite ephemeris
 * @param[in] pz_tot signal time of transmit
 * @param[in] u_constellation system
 * @param[in] u_svid satellite id
 * @param[in] q_iode if is -1, don`t match the iode in ephemeris
 * @param[out] pz_eph ephemeris
 * @return
 */
BOOL sd_api_GnssEphemeris_Get(const GpsTime_t* pz_tot, uint8_t u_constellation, uint8_t u_svid, int32_t q_iode,
                              gnss_Ephemeris_t* pz_eph)
{
  if (FALSE == sd_hpp_task_ctrl.b_Init)
  {
    return FALSE;
  }

  BOOL b_success = FALSE;

  uint32_t u_satIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
  if (ALL_GNSS_SYS_SV_NUMBER == u_satIdx)
  {
    return FALSE;
  }
  double tmax = 7200.0;
  double t = 0;


  if(NULL != pz_tot)
  {
    switch (u_constellation)
    {
      case C_GNSS_GPS:
        tmax = GNSS_MAXDTOE;
        break;
      case C_GNSS_GLO:
        tmax = GNSS_MAXDTOE_GLO;
        break;
      case C_GNSS_GAL:
        tmax = GNSS_MAXDTOE_GAL;
        break;
      case C_GNSS_BDS2:
      case C_GNSS_BDS3:
        tmax = GNSS_MAXDTOE_BDS;
        break;
      case C_GNSS_QZS:
        tmax = GNSS_MAXDTOE;
        break;
      default:
        tmax = GNSS_MAXDTOE;
        break;
    }
    t = tmax;
  }
  int32_t u_idx = -1;
  for (int i = 0; i < GNSS_EPHEMERIS_CACHE_NUM; ++i)
  {
    if (M_IS_SET_BIT(gz_GnssEphemeris_Pool.t_gnssEphMask[u_constellation], u_svid) &&
        (gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i] != NULL))
    {
      int32_t iode_eph = -1;
      switch (u_constellation)
      {
        case C_GNSS_GPS: iode_eph = gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i]->eph.z_gpsEph.iode;
          break;
        case C_GNSS_GAL: iode_eph = gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i]->eph.z_galEph.iode;
          break;
        case C_GNSS_BDS2:
        case C_GNSS_BDS3: iode_eph = gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i]->eph.z_bdsEph.iode;
          break;
        case C_GNSS_QZS: iode_eph = gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i]->eph.z_gpsEph.iode;
          break;
        default: iode_eph = -1;
          break;
      }
      if (-1 == iode_eph)
      {
        continue;
      }
      if ((q_iode >= 0) && (iode_eph != q_iode))
      {
        continue;
      }
      double dtm = 0.0;
      if (NULL == pz_tot)
      {
        GpsTime_t tm_t = {0};
        dtm = fabs(tm_GpsTimeDiff(&tm_t, &gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i]->z_toeOfGpst));
      }
      else
      {
        dtm = fabs(tm_GpsTimeDiff(pz_tot, &gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][i]->z_toeOfGpst));
        if (fabs(dtm) > tmax)
        {
          continue;
        }
      }
      if ((fabs(dtm) < t) || ((-1 == u_idx) && (NULL == pz_tot)))
      {
        t = dtm;
        u_idx = i;
      }
    }
  }

  if (u_idx != -1)
  {
    memcpy(pz_eph, gz_GnssEphemeris_Pool.pz_gnssEph[u_satIdx][u_idx], sizeof(gnss_Ephemeris_t));
    b_success = TRUE;
  }
  return b_success;
}

/**
 * @brief Get satellite Position, Velocity and Clock
 * @param[in]   constellation
 * @param[in]   svid
 * @param[in]   tot by GPST
 * @param[out]  pz_SatPosVelClk
 * @Note  This function has a mutex to support multi-task call.
 */
BOOL sd_api_SatPosVelClk_Get(uint8_t u_constellation, uint8_t u_svid, GpsTime_t* pz_tot,
                             gnss_SatPosVelClk_t* pz_SatPosVelClk)
{
  if (NULL == pz_SatPosVelClk || NULL == pz_tot)
  {
    return FALSE;
  }

  uint32_t u_satIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
  if (ALL_GNSS_SYS_SV_NUMBER == u_satIdx)
  {
    return FALSE;
  }

  BOOL b_success = FALSE;
  gnss_SatPosVelClkPolynomial_t z_SatPosVelClkPolynomial = { 0 };

  b_success = sd_SatPosVelClkPolynomial_Get(u_constellation, u_svid,
                                            &z_SatPosVelClkPolynomial);
  if (FALSE == b_success)
  {
    return FALSE;
  }

  if (fabsf(z_SatPosVelClkPolynomial.f_toe_age) > 7200.0)
  {
    return FALSE;
  }

  double dt = tm_GpsTimeDiff(pz_tot, &z_SatPosVelClkPolynomial.z_fitStart);
  double dclk = sd_SatClk(u_constellation, u_svid, pz_tot);
  dt -= dclk;

  if (!(dt >= 0.0 && dt <= SAT_POLYNOMIAL_DURATION))
  {
    return FALSE;
  }

  pz_SatPosVelClk->u_valid = TRUE;
  pz_SatPosVelClk->z_gpsTime = *pz_tot;
  pz_SatPosVelClk->u_constellation = u_constellation;
  pz_SatPosVelClk->u_svid = u_svid;
  pz_SatPosVelClk->f_elevation = 0.0f;
  pz_SatPosVelClk->f_azimuth = 0.0f;
  pz_SatPosVelClk->q_iode = z_SatPosVelClkPolynomial.q_iode;
  pz_SatPosVelClk->f_dt = (float)dt;

  matrix_t* mat_satPosClk = matrix_new_from_buffer(4, 1, pz_SatPosVelClk->d_satPosClk);
  matrix_t* mat_satVelClk = matrix_new_from_buffer(4, 1, pz_SatPosVelClk->d_satVelClk);
  matrix_t* mat_posClkPolynomial = matrix_new_from_buffer(4, 4, (double*)z_SatPosVelClkPolynomial.d_posClkPolynomial);
  matrix_t* mat_velClkPolynomial = matrix_new_from_buffer(4, 4, (double*)z_SatPosVelClkPolynomial.d_velClkPolynomial);

  matrix_t* mat_dts = matrix_new(4, 1);
  MAT(mat_dts, 0, 0) = 1.0;
  MAT(mat_dts, 1, 0) = MAT(mat_dts, 0, 0) * dt;
  MAT(mat_dts, 2, 0) = MAT(mat_dts, 1, 0) * dt;
  MAT(mat_dts, 3, 0) = MAT(mat_dts, 2, 0) * dt;
  matrix_mul(mat_posClkPolynomial, mat_dts, mat_satPosClk);
  matrix_mul(mat_velClkPolynomial, mat_dts, mat_satVelClk);

  matrix_free(&mat_satPosClk);
  matrix_free(&mat_satVelClk);
  matrix_free(&mat_posClkPolynomial);
  matrix_free(&mat_velClkPolynomial);
  matrix_free(&mat_dts);

  return TRUE;
}

/**
 * @brief
 * @param[in] pz_tot signal time of transmit, =TOR-P/c, no clk correction
 * @param[in] u_constellation system
 * @param[in] u_svid satellite number
 * @param[in] q_iode if is -1, don`t match the iode in ephemeris
 * @param[in] u_clkCor TRUE/FALSE, TRUE: correct satellite clock, FALSE: don`t
 * @param[out] pz_satPosVelClk
 * @return false: failed. true: sucess
 */
BOOL sd_api_SatPositionFromBrdc(const GpsTime_t* pz_tot, uint8_t u_constellation, uint8_t u_svid, int32_t q_iode,
                                BOOL u_clkCor, gnss_SatPosVelClk_t* pz_satPosVelClk)
{
  BOOL status = FALSE;
  char char_sat[4] = "";
  double tow = pz_tot->q_towMsec * TIME_MSEC_INV + pz_tot->q_subNsec * TIME_NSEC_INV;
  GpsTime_t pz_tot_new = {0};
  pz_tot_new = *pz_tot;
  if (u_clkCor)
  {
    double clk = sd_SatClk(u_constellation, u_svid, pz_tot);
    tow -= clk;
    tm_GpstTimeAdd(&pz_tot_new, -clk);
  }

  gnss_Ephemeris_t ephemeris = {0};
  if (!sd_api_GnssEphemeris_Get(&pz_tot_new, u_constellation, u_svid, q_iode, &ephemeris))
  {
    svid_SatString(u_svid, u_constellation, char_sat);
    LOGW(TAG_SD, "Get eph fail, sat=%3s iode=%d\n", char_sat, q_iode);
    status = FALSE;
  }
  else
  {
    switch (u_constellation)
    {
      case C_GNSS_GPS:
        sd_ComputeGpsSvPosVelClk(&ephemeris.eph.z_gpsEph, tow, pz_satPosVelClk);
        status = TRUE;
        break;
      case C_GNSS_GAL:
        sd_ComputeGalSvPosVelClk(&ephemeris.eph.z_galEph, tow, pz_satPosVelClk);
        status = TRUE;
        break;
      case C_GNSS_BDS2:
      case C_GNSS_BDS3:
        sd_ComputeBdsSvPosVelClk(&ephemeris.eph.z_bdsEph, tow, pz_satPosVelClk);
        status = TRUE;
        break;
      case C_GNSS_QZS:
        sd_ComputeGpsSvPosVelClk(&ephemeris.eph.z_gpsEph, tow, pz_satPosVelClk);
        status = TRUE;
        break;
      default:
        status = FALSE;
        break;
    }
  }
  if (status)
  {
    pz_satPosVelClk->z_gpsTime = *pz_tot;
    pz_satPosVelClk->u_valid = status;
    pz_satPosVelClk->u_constellation = u_constellation;
    pz_satPosVelClk->u_svid = u_svid;
  }
  return status;
}
