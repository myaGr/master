/**@file        fusion_transform.c
 * @brief		fusion transform file
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

#include "fusion_transform.h"
#include "fusion_ellip_para.h"
#include "fusion_math.h"
#include "fusion_global.h"

 /*
 * @brief: Calculate the heading difference
 * @param[in]: heading1 - value1; heading2 - value2
 * @param[out]: None
 * @return: the diff
 */
double tf_GetHeadDiff(double heading1, double heading2)
{
  double diffHead = 0;

  diffHead = heading1 - heading2;
  if (diffHead > 180)
  {
    diffHead = diffHead - 360;
  }
  else if (diffHead < -180)
  {
    diffHead = diffHead + 360;
  }
  return diffHead;
}

 /*
 * @brief: two lla cal relative distance
 * @param[in]: lla - Observation point lla; ref - reference point lla
 * @param[out]: ned - ned relative distance
 * @return: None
 */
void tf_GetNedDiffPos(double* lla, double* ref, double* ned)
{
  double rm, rn;
  ellip_calculateMN(ref[0], &rm, &rn);
  ned[0] = (lla[0] - ref[0]) * (rm + ref[2]);
  ned[1] = (lla[1] - ref[1]) * (rn + ref[2]) * cos(ref[0]);
  ned[2] = -(lla[2] - ref[2]);
}

/*
* @brief: The rotation vector is calculated from the position error
* @param[in]: d_lat - latitude; f_delta_lat - diff lat; f_delta_lon - diff lon
* @param[out]: fz_rotvct - rotation vector
* @return: None
*/
void tf_llaerr2rotvct(double d_lat, float f_delta_lat, float f_delta_lon, float fz_rotvct[])
{
  fz_rotvct[0] = (float)(f_delta_lon * cos(d_lat));
  fz_rotvct[1] = (float)(-f_delta_lat);
  fz_rotvct[2] = (float)(-f_delta_lon * sin(d_lat));
}

/*
* @brief: Calculate the angular velocity of the earth's rotation(N frame)
* @param[in]: d_lla - Longitude latitude height
* @param[out]: f_wien - wie in N frame
* @return: None
*/
void tf_get_wien(double d_lla[3], float f_wien[3])
{
  double d_lat = d_lla[0];

  f_wien[0] = (float)(cos(d_lat) * (fusion_get_ellippara()->d_wie));
  f_wien[1] = 0.0f;
  f_wien[2] = (float)(-sin(d_lat) * (fusion_get_ellippara()->d_wie));
}

/*
* @brief: Calculate wenn
* @param[in]: d_lla - Longitude latitude height; f_vel_ned - vel in NED
* @param[out]: f_wenn - wen in N frame
* @return: None
*/
void tf_get_wenn(double d_lla[3], float f_vel_ned[3], float f_wenn[3])
{
  double d_ve = f_vel_ned[1];
  double d_vn = f_vel_ned[0];
  double d_lat = d_lla[0];
  double d_alt = d_lla[2];
  double d_rm, d_rn;
  ellip_calculateMN(d_lat, &d_rm, &d_rn);
  f_wenn[0] = (float)(d_ve / (d_rn + d_alt));
  f_wenn[1] = (float)(-d_vn / (d_rm + d_alt));
  f_wenn[2] = (float)(-tan(d_lat) * f_wenn[0]);
}

/*
* @brief: use rotation matrix cal rotarion vector
* @param[in]: f_tiny_rotmat - rotation matrix
* @param[out]: f_rotvct - rotation vector
* @return: None
*/
void tf_rotvct2mat(float f_tiny_rotmat[3][3], float f_rotvct[3])
{
  float f_rotmat_tmp[3][3];

  math_mateye(&(f_tiny_rotmat[0][0]),3);
  math_skewsym(f_rotvct, f_rotmat_tmp);
  math_matadd(&(f_tiny_rotmat[0][0]), &(f_rotmat_tmp[0][0]), 3, 3, &(f_tiny_rotmat[0][0]));
}

/*
* @brief: lla to ecef
* @param[in]: d_poslla - lla; d_reflla - reference lla
* @param[out]: d_posecef - ecef
* @return: None
*/
void tf_lla2ecef(double d_poslla[3], double d_posecef[3], double d_reflla[3])
{
  double d_rm, d_rn;
  double d_coslat, d_sinlat, d_coslon, d_sinlon, d_rnh;
  ellip_calculateMN(d_reflla[0], &d_rm, &d_rn);

  d_coslat = cos(d_poslla[0]);
  d_sinlat = sin(d_poslla[0]);
  d_coslon = cos(d_poslla[1]);
  d_sinlon = sin(d_poslla[1]);
  d_rnh = d_rn + d_poslla[2];

  d_posecef[0] = d_rnh * d_coslat * d_coslon;
  d_posecef[1] = d_rnh * d_coslat * d_sinlon;
  d_posecef[2] = (d_rn * (1.0 - fusion_get_ellippara()->d_ecc2) + d_poslla[2]) * d_sinlat;
}

/* transform ecef to lat lon height (rad, rad, m) */
void tf_ecef2lla(double* pd_pos_ecef, double* pd_pos_lla)
{
  double d_e2 = fusion_get_ellippara()->d_ecc2;
  double d_r2 = pd_pos_ecef[0] * pd_pos_ecef[0] + pd_pos_ecef[1] * pd_pos_ecef[1];
  double d_z = pd_pos_ecef[2];
  double d_zk = 0.0;
  double d_v = fusion_get_ellippara()->d_semi_major;
  double d_sinp;

  while (fabs(d_z - d_zk) >= 1E-4)
  {
    d_zk = d_z;
    d_sinp = d_z / sqrt(d_r2 + d_z * d_z);
    d_v = fusion_get_ellippara()->d_semi_major / sqrt(1.0 - d_e2 * d_sinp * d_sinp);
    d_z = pd_pos_ecef[2] + d_v * d_e2 * d_sinp;
  }

  pd_pos_lla[0] = d_r2 > 1E-12 ? atan(d_z / sqrt(d_r2)) : (pd_pos_ecef[2] > 0.0 ? INS_PI / 2.0 : -INS_PI / 2.0);
  pd_pos_lla[1] = d_r2 > 1E-12 ? atan2(pd_pos_ecef[1], pd_pos_ecef[0]) : 0.0;
  pd_pos_lla[2] = sqrt(d_r2 + d_z * d_z) - d_v;
}

/*
* @brief: cal CEN matrix
* @param[in]: d_poslla - lla
* @param[out]:f_ce2n - CEN matrix
* @return: None
*/
void tf_ce2nbylla(double d_poslla[3], float f_ce2n[3][3])
{
  float	f_coslat = (float)cos(d_poslla[0]);
  float	f_sinlat = (float)sin(d_poslla[0]);
  float	f_coslon = (float)cos(d_poslla[1]);
  float	f_sinlon = (float)sin(d_poslla[1]);

  f_ce2n[0][0] = -f_sinlat * f_coslon;
  f_ce2n[1][0] = -f_sinlon;
  f_ce2n[2][0] = -f_coslat * f_coslon;
  f_ce2n[0][1] = -f_sinlat * f_sinlon;
  f_ce2n[1][1] = f_coslon;
  f_ce2n[2][1] = -f_coslat * f_sinlon;
  f_ce2n[0][2] = f_coslat;
  f_ce2n[1][2] = 0.0f;
  f_ce2n[2][2] = -f_sinlat;
}

/*
* @brief: lla to Cn2e
* @param[in]: d_poslla - lla
* @param[out]: f_cn2e - CNE matrix
* @return: None
*/
void tf_cn2ebylla(double d_poslla[3], float f_cn2e[3][3])
{
  float	f_coslat = (float)cos(d_poslla[0]);
  float	f_sinlat = (float)sin(d_poslla[0]);
  float	f_coslon = (float)cos(d_poslla[1]);
  float	f_sinlon = (float)sin(d_poslla[1]);

  f_cn2e[0][0] = -f_sinlat * f_coslon;
  f_cn2e[0][1] = -f_sinlon;
  f_cn2e[0][2] = -f_coslat * f_coslon;
  f_cn2e[1][0] = -f_sinlat * f_sinlon;
  f_cn2e[1][1] = f_coslon;
  f_cn2e[1][2] = -f_coslat * f_sinlon;
  f_cn2e[2][0] = f_coslat;
  f_cn2e[2][1] = 0.0f;
  f_cn2e[2][2] = -f_sinlat;
}

/*
* @brief: euler to DCM
* @param[in]: f_euler - euler angle
* @param[out]: f_dcm - DCM matrix
* @return: None
*/
void tf_euler2dcm(float f_euler[3], float f_dcm[3][3])
{
  float	f_cosroll = cosf(f_euler[0]);
  float	f_sinroll = sinf(f_euler[0]);
  float	f_cospitch = cosf(f_euler[1]);
  float	f_sinpitch = sinf(f_euler[1]);
  float	f_coshead = cosf(f_euler[2]);
  float	f_sinHead = sinf(f_euler[2]);

  f_dcm[0][0] = f_cospitch * f_coshead;
  f_dcm[0][1] = -f_cosroll * f_sinHead + f_sinroll * f_sinpitch * f_coshead;
  f_dcm[0][2] = f_sinroll * f_sinHead + f_cosroll * f_sinpitch * f_coshead;

  f_dcm[1][0] = f_cospitch * f_sinHead;
  f_dcm[1][1] = f_cosroll * f_coshead + f_sinroll * f_sinpitch * f_sinHead;
  f_dcm[1][2] = -f_sinroll * f_coshead + f_cosroll * f_sinpitch * f_sinHead;

  f_dcm[2][0] = -f_sinpitch;
  f_dcm[2][1] = f_sinroll * f_cospitch;
  f_dcm[2][2] = f_cosroll * f_cospitch;
}

/*
* @brief: DCM to euler
* @param[in]: f_dcm - DCM matrix
* @param[out]: f_euler - euler angle
* @return: None
*/
void tf_dcm2eluer(float f_dcm[3][3], float f_euler[3])
{
  f_euler[1] = (float)atan(-f_dcm[2][0] / sqrt(f_dcm[2][1] * f_dcm[2][1] + f_dcm[2][2] * f_dcm[2][2]));

  if (f_dcm[2][0] < -0.999)
  {
    f_euler[0] = -99999.99f;
    f_euler[2] = (float)atan2((f_dcm[1][2] - f_dcm[0][1]), (f_dcm[0][2] + f_dcm[1][1]));
  }
  else if (f_dcm[2][0] > 0.999)
  {
    f_euler[0] = -99999.99f;
    f_euler[2] = (float)(atan2((f_dcm[1][2] + f_dcm[0][1]), (f_dcm[0][2] - f_dcm[1][1])) + (float)INS_PI);
  }
  else
  {
    f_euler[0] = (float)atan2(f_dcm[2][1], f_dcm[2][2]);
    f_euler[2] = (float)atan2(f_dcm[1][0], f_dcm[0][0]);
  }
}

/*
* @brief: Use velocity to extrapolate position
* @param[in]: pf_vel_ned - NED vel; f_dt - delta time
* @param[out]: pd_blh - result blh
* @return: None
*/
void tf_extend_pos_byvel(double* pd_blh, float* pf_vel_ned, float f_dt)
{
  float f_diff_ned[3] = { 0.0f };
  double d_pos_xyz[3] = { 0.0 };
  float f_diff_xyz[3] = { 0.0f };
  float f_mat_n2e[3][3] = { 0.0f };

  if (f_dt > 0.f)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      f_diff_ned[i] = pf_vel_ned[i] * f_dt;
    }

    tf_lla2ecef(pd_blh, d_pos_xyz, pd_blh);

    tf_cn2ebylla(pd_blh, f_mat_n2e);

    math_matmul(&(f_mat_n2e[0][0]), f_diff_ned, 3, 3, 1, f_diff_xyz);

    for (uint8_t i = 0; i < 3; i++)
    {
      d_pos_xyz[i] += f_diff_xyz[i];
    }

    /* convert the xyz to lla (rad) */
    tf_ecef2lla(d_pos_xyz, pd_blh);
  }
}
