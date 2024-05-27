/**@file        fusion_ellip_para.c
 * @brief		fusion earth para file 
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "fusion_ellip_para.h"
#include <math.h>

static EllipPara_t gz_ellippara;

/*
* @brief: get ellipsoid para
* @param[in]:  void
* @param[out]: None
* @return: ellipsoid para struct
*/
EllipPara_t* fusion_get_ellippara(void)
{
  return &gz_ellippara;
}

/*
* @brief: calculate normal gravity
* @param[in]: lat,lon,alt
* @param[out]: None
* @return: normal gravity
*/
float ellip_get_gravity(double pd_poslla[3])
{
  double d_s2, d_s4;
  double d_ng;

  d_s2 = sin(pd_poslla[0]);
  d_s2 *= d_s2;
  d_s4 = d_s2 * d_s2;

  d_ng = (EARTH_GRAVITY_PARA1 * (1.0 + EARTH_GRAVITY_PARA2 * d_s2 + EARTH_GRAVITY_PARA3 * d_s4));
  d_ng += ((EARTH_GRAVITY_PARA4 + EARTH_GRAVITY_PARA5 * d_s2) * pd_poslla[2]);
  d_ng += (EARTH_GRAVITY_PARA6 * pd_poslla[2] * pd_poslla[2]);

  return (float)d_ng;
}

/*
* @brief: calculate radius of curvature in prime vertical, radius of curvature in meridian
* @param[in]: lat, radius of curvature in prime vertical, radius of curvature in meridian
* @param[out]: radius of curvature in prime vertical, radius of curvature in meridian
* @return: None
*/
void ellip_calculateMN(double d_lat, double* pd_M, double* pd_N)
{
  double d_sinlat = sin(d_lat);
  double d_w2 = 1.0 - gz_ellippara.d_ecc2 * d_sinlat * d_sinlat;
  double d_w = sqrt(d_w2);

  *pd_N = gz_ellippara.d_semi_major / d_w;
  *pd_M = (*pd_N) * (1.0 - gz_ellippara.d_ecc2) / d_w2;
}

/*
* @brief: ellipsoid para initialize
* @param[in]: None
* @param[out]: None
* @return
*/
void ellip_para_initial(void)
{
  gz_ellippara.d_semi_major = EARTH_R0;
  gz_ellippara.d_semi_minor = EARTH_RP;
  gz_ellippara.d_wie = EARTH_WIE;

  gz_ellippara.d_f = (EARTH_R0 - EARTH_RP) / EARTH_R0;
  gz_ellippara.d_ecc2 = 1.0 - ((EARTH_RP * EARTH_RP) / (EARTH_R0 * EARTH_R0));
}

/*
* @brief: configure radius of curvature in prime vertical, radius of curvature in meridian
* @input: radius of curvature in prime vertical, radius of curvature in meridian
* @output: None
* @return
*/
void ellip_setRMN(double d_rm, double d_rn, EllipPara_t* pz_ellip)
{
  pz_ellip->d_rm = d_rm;
  pz_ellip->d_rn = d_rn;
}