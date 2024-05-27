/**@file        fusion_quat.h
 * @brief		fusion quat file
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

#ifndef __FUSION_QUAT_H__
#define __FUSION_QUAT_H__

#include <stdint.h>

void quat_quat2rotvct(float f_q2r[4], float f_rotvct[3]);
void quat_conjugate(float f_q_conj[4]);
void quat_euler2quat(float f_att[3], float f_e2q[4]);
void quat_quat2dcm(float f_q[4], float f_dcm[3][3]);
void quat_normalize_d(double d_q[4]);
void quat_normalize_f(float f_q[4]);
void quat_copy_d(double d_q2[4], double d_q1[4]);
void quat_copy_f(float f_q2[4], float f_q1[4]);
void quat_get_posbyn2e(double d_lat_lon[2], double d_quatn2e[4]);
void quat_get_qn2e(double d_lat, double d_lon, double d_quatn2e[4]);
void quat_rotvec2quat(float f_rotvec[3], float f_qpr[4]);
void quat_product_d(double d_q1[4], double d_q2[4]);
void quat_product_f(float f_q1[4], float f_q2[4]);

#endif // !__FUSION_QUAT_H__
