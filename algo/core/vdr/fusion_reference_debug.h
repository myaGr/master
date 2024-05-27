/**@file        reference_debug.h
 * @brief		reference_debug interface
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

#ifndef __FUSION_REFERENCE_DEBUG_H__
#define __FUSION_REFERENCE_DEBUG_H__

#ifdef INS_DEBUG

#include <stdio.h>
#include "fusion_proc.h"
#include "stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.141592653
#endif
typedef struct
{
  double tow; /* s */
  double lat; /* deg */
  double lon; /* deg */
  double alt; /* m, Ellipsoidal height, Used to calculate the distance between the satellite and the Earth */
  double heading; /* deg */
  double pitch; /* deg */
  double roll; /* deg */
  double vn; /* m/s */
  double ve; /* m/s */
  double vd; /* m/s */
  double height; /* m Altitude */
} RefInfo;

typedef struct
{
  int num;
  RefInfo *data;
} RefList;

typedef struct
{
  /* time domain */
  float h_acc_max;
  float h_acc_min;
  float h_acc_mean;
  float h_acc_var;
  float v_acc_max;
  float v_acc_min;
  float v_acc_mean;
  float v_acc_var;
  float h_gyro_max;
  float h_gyro_min;
  float h_gyro_mean;
  float h_gyro_var;
  float v_gyro_max;
  float v_gyro_min;
  float v_gyro_mean;
  float v_gyro_var;
  /* freq domain */
  float h_acc_dc_component;
  float h_acc_mag_avg;
  float h_acc_mag_var;
  float v_acc_dc_component;
  float v_acc_mag_avg;
  float v_acc_mag_var;
  float h_gyro_dc_component;
  float h_gyro_mag_avg;
  float h_gyro_mag_var;
  float v_gyro_dc_component;
  float v_gyro_mag_avg;
  float v_gyro_mag_var;
} MlFeature;

typedef struct
{
  float Re;
  float Im;
} Complex;

void ref_ModuleInit(FILE* fp);
RefInfo ref_GetReferenceInfo(double t);
RefInfo ref_GetReferenceInfoFast(double t);
int ref_CalInsError(NavParams_t* ins, double* error);
int ref_CalInsAntError(NavParams_t* ins, double* error);
void ref_CalPrRes(AsensingGNSSPsrMeas_t* pz_psr_meas);
int ref_CalGnssError(AsensingGNSSPara_t* pz_gnss, double* error);
void CalTimeDomainFeature(float imuData[4][64], MlFeature* mlFea);
void CalFrqDomainFeature(float imuData[4][64], MlFeature* mlFea);
void Asm_Mag(Complex* x, float* Mag);
void fft(Complex* v, int n, Complex* tmp);

#ifdef __cplusplus
}
#endif
#endif
#endif