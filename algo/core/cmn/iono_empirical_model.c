/**@file        iono_empirical_model.c
 * @brief       Ionosphere Empirical Models
 * @details     Broadcast ionosphere model (or called Klobuchar model
 * @author      houxiaowei
 * @date        2022/8/29
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/8/29   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "iono_empirical_model.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include "gnss_def.h"

#define ERROR_BRDCI   0.5             /* broadcast iono model error factor */

/**
 * @brief ionosphere mapping function; compute ionospheric delay mapping function by single layer model
 * @param[in] pos receiver position {lat,lon,h} (rad,m)
 * @param[in] azel azimuth/elevation angle {az,el} (rad)
 * @return ionospheric mapping function
 */
extern double ion_Mapfution(const double *pos, const double *azel)
{
  double value = 1.0;
  if (pos[2] < HEIGHT_ION)
  {
    value = 1.0/cos(asin((RE_WGS84+pos[2])/(RE_WGS84+HEIGHT_ION)*sin(PI/2.0-azel[1])));
  }
  return value;
}

/**
 * @brief compute ionospheric delay by broadcast ionosphere model (klobuchar model)
 * @param[in] t time (gpst)
 * @param[in] ion iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
 * @param[in] pos receiver position {lat,lon,h} (rad,m)
 * @param[in] azel azimuth/elevation angle {az,el} (rad)
 * @return ionospheric delay (L1) (m)
*/
extern double ion_KlbModel(GpsTime_t t, const double ion[8], const double *pos, const double *azel){

  const double ion_default[] = { /* 2022/8/17 brdc GPS IONOSPHERIC CORR*/
          1.0245E-08, 2.2352E-08, -5.9605E-08, -1.1921E-07,
          1.0035E+05, 1.3107E+05, -6.5536E+04, -3.9322E+05
  };
  double tt, f, psi, phi, lam, amp, per, x;

  if (pos[2] < -1E3 || azel[1] <= 0) return 0.0;
  if (vector_norm(ion, 8) <= 0.0) ion = ion_default;

  /* earth centered angle (semi-circle) */
  psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;

  /* subionospheric latitude/longitude (semi-circle) */
  phi = pos[0] / PI + psi * cos(azel[0]);
  if (phi > 0.416) phi = 0.416;
  else if (phi < -0.416) phi = -0.416;
  lam = pos[1] / PI + psi * sin(azel[0]) / cos(phi * PI);

  /* geomagnetic latitude (semi-circle) */
  phi += 0.064 * cos((lam - 1.617) * PI);

  /* local time (s) */
  double sec = t.q_subNsec * TIME_NSEC_INV + t.q_towMsec * TIME_MSEC_INV;
  tt = 43200.0 * lam + sec;
  tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

  /* slant factor */
  f = 1.0 + 16.0 * pow(0.53 - azel[1] / PI, 3.0);

  /* ionospheric delay */
  amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
  per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
  amp = amp < 0.0 ? 0.0 : amp;
  per = per < 72000.0 ? 72000.0 : per;
  x = 2.0 * PI * (tt - 50400.0) / per;
  double delay = CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);

  return delay;
}

/**
 * @brief get ionosphere empirical correction on GPS (L1) (m)
 * @param[in] time time (gpst)
 * @param[in] klb iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}, if input [0], use the default value
 * @param[in] pos receiver position {lat,lon,h} (rad|m)
 * @param[in] azel azimuth/elevation angle {az,el} (rad)
 * @param[in] ionoopt ionospheric correction option (IONOOPT_???)
 * @param[out] ion ionospheric delay (L1) (m)
 * @param[out] var ionospheric delay (L1) variance (m^2)
 * @return status(0:ok,1:error)
*/
extern int iono_GetIonoEmpiricalCorrection(GpsTime_t time, double klb[8], const double* pos, const double* azel, int ionoopt,
                                double* ion, double* var)
{

  int8_t status = 1;
  
  /* GPS broadcast ionosphere model */
  if (IONO_PRO_OPT_BRDC == ionoopt) {
    *ion = ion_KlbModel(time, klb, pos, azel);
    *var = SQR(*ion * ERROR_BRDCI);
    status = 0;
  }
  return status;
}

