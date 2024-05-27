/**@file        fusion_transform.h
 * @brief		fusion transfrom file
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

#ifndef __FUSION_TRANSFORM_H__
#define __FUSION_TRANSFORM_H__

#include <stdint.h>

void tf_get_wien(double d_lla[3], float f_wien[3]);
void tf_get_wenn(double d_lla[3], float f_vel_ned[3], float f_wenn[3]);
void tf_rotvct2mat(float f_tiny_rotmat[3][3], float f_rotvct[3]);
void tf_lla2ecef(double d_poslla[3], double d_posecef[3], double d_reflla[3]);
void tf_ecef2lla(double* pd_pos_ecef, double* pd_pos_lla);
void tf_ce2nbylla(double d_poslla[3], float f_ce2n[3][3]);
void tf_cn2ebylla(double d_poslla[3], float f_cn2e[3][3]);
void tf_euler2dcm(float f_euler[3], float f_dcm[3][3]);
void tf_dcm2eluer(float f_dcm[3][3], float f_euler[3]);
void tf_llaerr2rotvct(double d_lat, float f_delta_lat, float f_delta_lon, float fz_rotvct[]);
void tf_GetNedDiffPos(double* lla, double* ref, double* ned);
double tf_GetHeadDiff(double heading1, double heading2);
void tf_extend_pos_byvel(double* pd_blh, float* pf_vel_ned, float f_dt);

#endif // !__FUSION_TRANSFORM_H__
