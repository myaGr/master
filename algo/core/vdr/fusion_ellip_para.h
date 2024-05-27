/**@file        fusion_ellip_para.h
 * @brief		fusion earth params file
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

#ifndef __FUSION_ELLIP_PARA_H__
#define __FUSION_ELLIP_PARA_H__

#include <stdint.h>

#define EARTH_WIE 7.292115147e-5
#define EARTH_R0 6378137.0
#define EARTH_RP 6356752.3142
#define EARTH_GRAVITY_PARA1 9.7803267715
#define EARTH_GRAVITY_PARA2 0.0052790414
#define EARTH_GRAVITY_PARA3 0.0000232718
#define EARTH_GRAVITY_PARA4 -0.000003087691089
#define EARTH_GRAVITY_PARA5 0.000000004397731
#define EARTH_GRAVITY_PARA6 0.000000000000721

typedef struct
{
  double d_rm;
  double d_rn;
  double d_wie;
  double d_f;
  double d_ecc2;
  double d_semi_major;
  double d_semi_minor;
  float  f_g0;
} EllipPara_t;

EllipPara_t* fusion_get_ellippara(void);
void ellip_para_initial(void);
float ellip_get_gravity(double pd_poslla[3]);
void ellip_calculateMN(double d_lat, double* pd_M, double* pd_N);
void ellip_setRMN(double d_rm, double d_rn, EllipPara_t* pz_ellip);

#endif // !__FUSION_ELLIP_PARA_H__
