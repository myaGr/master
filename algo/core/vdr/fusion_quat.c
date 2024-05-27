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

#include "fusion_quat.h"
#include "fusion_global.h"
#include "fusion_math.h"

 /*
 * @brief: Multiplication of quaternions(float)
 * @param[in]: f_q1 - data1; f_q2 - data2
 * @param[out]: f_q1 - result quaternions
 * @return: None
 */
void quat_product_f(float f_q1[4], float f_q2[4])
{
  float	f_qvpv = f_q1[1] * f_q2[1] + f_q1[2] * f_q2[2] + f_q1[3] * f_q2[3];
  float	f_sr = f_q1[0] * f_q2[0] - f_qvpv;

  float	f_crs1 = f_q1[2] * f_q2[3] - f_q1[3] * f_q2[2];
  float	f_crs2 = f_q1[3] * f_q2[1] - f_q1[1] * f_q2[3];
  float	f_crs3 = f_q1[1] * f_q2[2] - f_q1[2] * f_q2[1];

  float	f_qq1 = f_q1[0] * f_q2[1] + f_q2[0] * f_q1[1] + f_crs1;
  float	f_qq2 = f_q1[0] * f_q2[2] + f_q2[0] * f_q1[2] + f_crs2;
  float	f_qq3 = f_q1[0] * f_q2[3] + f_q2[0] * f_q1[3] + f_crs3;

  f_q1[0] = f_sr;
  f_q1[1] = f_qq1;
  f_q1[2] = f_qq2;
  f_q1[3] = f_qq3;

  if (f_sr < 0.0)
  {
    f_q1[0] = -f_q1[0];
    f_q1[1] = -f_q1[1];
    f_q1[2] = -f_q1[2];
    f_q1[3] = -f_q1[3];
  }
}

/*
* @brief: Multiplication of quaternions(double)
* @param[in]: f_q1 - data1; f_q2 - data2
* @param[out]: f_q1 - result quaternions
* @return: None
*/
void quat_product_d(double d_q1[4], double d_q2[4])
{
  double d_qvpv = d_q1[1] * d_q2[1] + d_q1[2] * d_q2[2] + d_q1[3] * d_q2[3];
  double d_sr = d_q1[0] * d_q2[0] - d_qvpv;
  double d_crs1 = d_q1[2] * d_q2[3] - d_q1[3] * d_q2[2];
  double d_crs2 = d_q1[3] * d_q2[1] - d_q1[1] * d_q2[3];
  double d_crs3 = d_q1[1] * d_q2[2] - d_q1[2] * d_q2[1];
  double d_qq1 = d_q1[0] * d_q2[1] + d_q2[0] * d_q1[1] + d_crs1;
  double d_qq2 = d_q1[0] * d_q2[2] + d_q2[0] * d_q1[2] + d_crs2;
  double d_qq3 = d_q1[0] * d_q2[3] + d_q2[0] * d_q1[3] + d_crs3;

  d_q1[0] = d_sr;
  d_q1[1] = d_qq1;
  d_q1[2] = d_qq2;
  d_q1[3] = d_qq3;

  if (d_sr < 0.0)
  {
    d_q1[0] = -d_q1[0];
    d_q1[1] = -d_q1[1];
    d_q1[2] = -d_q1[2];
    d_q1[3] = -d_q1[3];
  }
}

/*
* @brief: Rotation matrix to quaternions
* @param[in]: f_rotvec - Rotation vector
* @param[out]: f_qpr - quaternions
* @return: None
*/
void quat_rotvec2quat(float f_rotvec[3], float f_qpr[4])
{
  float	mag2 = f_rotvec[0] * f_rotvec[0] + f_rotvec[1] * f_rotvec[1] + f_rotvec[2] * f_rotvec[2];

  if ((mag2 < (INS_PI * INS_PI / 1000.0f)))
  {
    float c, s;
    mag2 *= 0.25f;
    c = 1.0f - mag2 / 2.0f * (1.0f - mag2 / 12.0f * (1.0f - mag2 / 30.0f));
    s = 1.0f - mag2 / 6.0f * (1.0f - mag2 / 20.0f * (1.0f - mag2 / 42.0f));
    s *= 0.5f;

    f_qpr[0] = c;
    f_qpr[1] = s * f_rotvec[0];
    f_qpr[2] = s * f_rotvec[1];
    f_qpr[3] = s * f_rotvec[2];
  }
  else
  {
    float mag = (float)(sqrt((double)mag2));
    float s_mag = (float)(sin((double)mag / 2.0) / mag);

    f_qpr[0] = (float)(cos((double)mag / 2.0));
    f_qpr[1] = f_rotvec[0] * s_mag;
    f_qpr[2] = f_rotvec[1] * s_mag;
    f_qpr[3] = f_rotvec[2] * s_mag;

    if (f_qpr[0] < 0.0f)
    {
      f_qpr[0] = -f_qpr[0];
      f_qpr[1] = -f_qpr[1];
      f_qpr[2] = -f_qpr[2];
      f_qpr[3] = -f_qpr[3];
    }
  }
}

/*
* @brief: lla to pos quaternions
* @param[in]: d_lat - latitude; d_lon - longitude
* @param[out]: d_quatn2e - pos quaternions
* @return: None
*/
void quat_get_qn2e(double d_lat, double d_lon, double d_quatn2e[4])
{
  double d_mlat = -INS_PI / 4.0 - d_lat / 2.0;
  double d_cosmlat = cos(d_mlat);
  double d_sinmlat = sin(d_mlat);
  double d_cosmlon = cos(d_lon / 2.0);
  double d_sinmlon = sin(d_lon / 2.0);

  d_quatn2e[0] = d_cosmlat * d_cosmlon;
  d_quatn2e[1] = -d_sinmlat * d_sinmlon;
  d_quatn2e[2] = d_sinmlat * d_cosmlon;
  d_quatn2e[3] = d_cosmlat * d_sinmlon;
}

/*
* @brief: pos quaternions to lla
* @param[in]: d_quatn2e - pos quaternions
* @param[out]: d_lat_lon - lat and lon
* @return:
*/
void quat_get_posbyn2e(double d_lat_lon[2], double d_quatn2e[4])
{
  d_lat_lon[0] = -2.0 * atan(d_quatn2e[2] / d_quatn2e[0]) - INS_PI / 2.0;
  d_lat_lon[1] = 2.0 * atan2(d_quatn2e[3], d_quatn2e[0]);
}

/*
* @brief: quaternions copy(float)
* @param[in]: f_q1 - src quaternions
* @param[out]: f_q2 - dst quaternions
* @return: None
*/
void quat_copy_f(float f_q2[4], float f_q1[4])
{
  f_q2[0] = f_q1[0];
  f_q2[1] = f_q1[1];
  f_q2[2] = f_q1[2];
  f_q2[3] = f_q1[3];
}

/*
* @brief: quaternions copy(double)
* @param[in]: f_q1 - src quaternions
* @param[out]: f_q2 - dst quaternions
* @return: None
*/
void quat_copy_d(double d_q2[4], double d_q1[4])
{
  d_q2[0] = d_q1[0];
  d_q2[1] = d_q1[1];
  d_q2[2] = d_q1[2];
  d_q2[3] = d_q1[3];
}

/*
* @brief: quaternion normalization
* @param[in]: f_q - src quaternion
* @param[out]: f_q - dst quaternion
* @return: None
*/
void quat_normalize_f(float f_q[4])
{
  float mag = sqrtf(f_q[0] * f_q[0] + f_q[1] * f_q[1] + f_q[2] * f_q[2] + f_q[3] * f_q[3]);
  mag = 1.0f / mag;
  f_q[0] *= mag;
  f_q[1] *= mag;
  f_q[2] *= mag;
  f_q[3] *= mag;
}

void quat_normalize_d(double d_q[4])
{
  double mag = sqrt(d_q[0] * d_q[0] + d_q[1] * d_q[1] + d_q[2] * d_q[2] + d_q[3] * d_q[3]);
  mag = 1.0 / mag;
  d_q[0] *= mag;
  d_q[1] *= mag;
  d_q[2] *= mag;
  d_q[3] *= mag;
}

/*
* @brief: quaternion to DCM
* @param[in]: f_q - quaternion
* @param[out]: f_dcm - DCM
* @return: None
*/
void quat_quat2dcm(float f_q[4], float f_dcm[3][3])
{
  f_dcm[0][0] = f_q[0] * f_q[0] + f_q[1] * f_q[1] - f_q[2] * f_q[2] - f_q[3] * f_q[3];
  f_dcm[0][1] = 2.0f * (f_q[1] * f_q[2] - f_q[0] * f_q[3]);
  f_dcm[0][2] = 2.0f * (f_q[1] * f_q[3] + f_q[0] * f_q[2]);

  f_dcm[1][0] = 2.0f * (f_q[1] * f_q[2] + f_q[0] * f_q[3]);
  f_dcm[1][1] = f_q[0] * f_q[0] - f_q[1] * f_q[1] + f_q[2] * f_q[2] - f_q[3] * f_q[3];
  f_dcm[1][2] = 2.0f * (f_q[2] * f_q[3] - f_q[0] * f_q[1]);

  f_dcm[2][0] = 2.0f * (f_q[1] * f_q[3] - f_q[0] * f_q[2]);
  f_dcm[2][1] = 2.0f * (f_q[2] * f_q[3] + f_q[0] * f_q[1]);
  f_dcm[2][2] = f_q[0] * f_q[0] - f_q[1] * f_q[1] - f_q[2] * f_q[2] + f_q[3] * f_q[3];
}

/*
* @brief: att to att quaternion
* @param[in]: f_att - Attitude Angle
* @param[out]: f_e2q - att quaternion
* @return: None
*/
void quat_euler2quat(float f_att[3], float f_e2q[4])
{
  float	f_cosroll = cosf(f_att[0] / 2.0f);
  float	f_sinroll = sinf(f_att[0] / 2.0f);
  float	f_cospitch = cosf(f_att[1] / 2.0f);
  float	f_sinpitch = sinf(f_att[1] / 2.0f);
  float	f_coshead = cosf(f_att[2] / 2.0f);
  float	f_sinhead = sinf(f_att[2] / 2.0f);

  f_e2q[0] = f_cosroll * f_cospitch * f_coshead + f_sinroll * f_sinpitch * f_sinhead;
  f_e2q[1] = f_sinroll * f_cospitch * f_coshead - f_cosroll * f_sinpitch * f_sinhead;
  f_e2q[2] = f_cosroll * f_sinpitch * f_coshead + f_sinroll * f_cospitch * f_sinhead;
  f_e2q[3] = f_cosroll * f_cospitch * f_sinhead - f_sinroll * f_sinpitch * f_coshead;
}

/*
* @brief: cal Conjugate quaternions
* @param[in]: f_q_conj - src quaternions
* @param[out]: f_q_conj - dst quaternions
* @return:
*/
void quat_conjugate(float f_q_conj[4])
{
  f_q_conj[1] = -f_q_conj[1];
  f_q_conj[2] = -f_q_conj[2];
  f_q_conj[3] = -f_q_conj[3];
}

/*
* @brief: quaternions to rotation vector
* @param[in]: f_q2r - quaternions
* @param[out]: f_rotvct - rotation vector
* @return: None
*/
void quat_quat2rotvct(float f_q2r[4], float f_rotvct[3])
{
  float f_a, f_b, f_c, f_d;

  if (f_q2r[0] < 0.0f)
  {
    f_a = -f_q2r[0];
    f_b = -f_q2r[1];
    f_c = -f_q2r[2];
    f_d = -f_q2r[3];
  }
  else
  {
    f_a = f_q2r[0];
    f_b = f_q2r[1];
    f_c = f_q2r[2];
    f_d = f_q2r[3];
  }
  if (f_a == 0.0f)
  {
    f_rotvct[0] = (float)(INS_PI) * f_b;
    f_rotvct[1] = (float)(INS_PI) * f_c;
    f_rotvct[2] = (float)(INS_PI) * f_d;
  }
  else
  {
    float	f_halfphi2, f_halfphi4, f_halfphi6, f_halfphi8;
    float	f_temp;
    f_halfphi2 = (f_b * f_b + f_c * f_c + f_d * f_d) / (f_a * f_a);
    f_halfphi4 = f_halfphi2 * f_halfphi2;
    f_halfphi6 = f_halfphi4 * f_halfphi2;
    f_halfphi8 = f_halfphi6 * f_halfphi2;

    f_temp = 0.5f * (1.0f - f_halfphi2 / 6.0f + f_halfphi4 / 120.0f - f_halfphi6 / 5040.0f + f_halfphi8 / 362880.0f);
    f_rotvct[0] = f_b / f_temp;
    f_rotvct[1] = f_c / f_temp;
    f_rotvct[2] = f_d / f_temp;
  }
}
