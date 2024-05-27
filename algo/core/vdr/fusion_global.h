/**@file        fusion_global.h
 * @brief		fusion global file
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

#ifndef __FUSION_GLOBAL_H__
#define __FUSION_GLOBAL_H__

#define STATIC_NAVIGATION 0
#define UNIFY_MEAS_SIZE 20

#ifndef INS_PI
#define INS_PI (3.14159265358979323846)
#endif // !PI

#ifndef INFINITY_MIN
#define INFINITY_MIN (1e-7)
#endif // !INFINITY_MIN

#ifndef RAD2DEG
#define RAD2DEG (180.0/INS_PI)
#endif // !RAD2DEG

#ifndef DEG2RAD
#define DEG2RAD (INS_PI/180.0f)
#endif // !DEG2RAD

#ifndef INS_FALSE
#define INS_FALSE (0)
#endif // !INS_FALSE

#ifndef INS_TRUE
#define INS_TRUE (1)
#endif // !INS_TRUE

#ifndef Gravity_GuangZhou
#define Gravity_GuangZhou (9.7883105f)
#endif // !Gravity_GuangZhou

#define SQR(x) ((x)*(x))
#define GNSS_SAMPLING_RATE  5
#define INITIAL_TIME		-9999.999
#define MAX_TIME_SPAN   31104000.0f /* All Seconds in One Year */

#endif // !__FUSION_GLOBAL_H__
