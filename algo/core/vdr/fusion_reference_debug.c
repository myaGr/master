/**@file        fusion_reference_debug.c
 * @brief		calculation error by reference
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

#ifdef INS_DEBUG

#include "ctype.h"
#include "string.h"
#include "stdlib.h"
#include "fusion_reference_debug.h"
#include "fusion_if.h"
#include "fusion_math.h"
#include "fusion_transform.h"
#include "fusion_tc.h"
#include "mw_log.h"

#define REF_FREQ 100
#define EPSILON (1e-13)
#define EARTH_E2 (0.00669438002290069)
#define R0_EARTH 6378137.0

RefList gz_refList = { 0 };

/* Sort from smallest to largest */
int cmpDouble(const void* a, const void* b)
{
  double* m = (double*)a;
  double* n = (double*)b;
  return (*m - *n);
}

double calMidRes(double* prRes, uint8_t satNum)
{
  double sortArray[MAX_RAW_OBS];
  double midRes = 0.0;
  memcpy(sortArray, prRes, sizeof(sortArray));

  if (satNum == 0)
  {
    return 0.0;
  }
  qsort(sortArray, satNum, sizeof(double), cmpDouble);
  if (satNum % 2 == 0 && satNum > 1)
  {
    midRes = (sortArray[satNum / 2] + sortArray[satNum / 2 - 1]) * 0.5;
  }
  else
  {
    midRes = sortArray[satNum / 2];
  }
  return midRes;
}

static void ref_Blh2Ecef(double* blh, double* ecef)
{
  double dsp, dcp, dsl, dcl, dgsq, den, dtemp;
  dsp = sin(blh[0]);
  dcp = cos(blh[0]);
  dsl = sin(blh[1]);
  dcl = cos(blh[1]);

  dgsq = 1.0 - (double)(EARTH_E2)*dsp * dsp;

  dtemp = sqrt(dgsq);
  if (fabs(dtemp) < EPSILON)
  {
    dtemp = EPSILON;
  }
  den = (double)(R0_EARTH / dtemp);
  dtemp = (den + blh[2]) * dcp;

  ecef[0] = dtemp * dcl;
  ecef[1] = dtemp * dsl;

  den = den - (double)(EARTH_E2 * den);
  ecef[2] = (den + blh[2]) * dsp;
}

int ref_CalGnssError(AsensingGNSSPara_t* pz_gnss, double* error)
{
  double blh[3], refBlh[3], errPosNED[3];
  if (pz_gnss->e_posflag == TYPE_POS_FLAG_UNAVAILABLE)
  {
    return -1;
  }

  double curTime = pz_gnss->d_tow / 1000.0;
  RefInfo refInfo = ref_GetReferenceInfo(curTime);

  if (refInfo.tow < 0)
  {
    return -1;
  }

  blh[0] = pz_gnss->d_lat * DEG2RAD;
  blh[1] = pz_gnss->d_lon * DEG2RAD;
  blh[2] = pz_gnss->f_alt;
  refBlh[0] = refInfo.lat * DEG2RAD;
  refBlh[1] = refInfo.lon * DEG2RAD;
  refBlh[2] = refInfo.alt;
  /* NED pos error */
  tf_GetNedDiffPos(blh, refBlh, errPosNED);
  error[0] = fabs(errPosNED[0]);
  error[1] = fabs(errPosNED[1]);
  error[2] = fabs(errPosNED[2]);
  /* NED vel error */
  error[3] = fabs(refInfo.vn - pz_gnss->f_vn);
  error[4] = fabs(refInfo.ve - pz_gnss->f_ve);
  error[5] = fabs(refInfo.vd - pz_gnss->f_vd);
  return 1;
}

int ref_CalInsAntError(NavParams_t* ins, double* error)
{
  double blh[3], refBlh[3], errPosNED[3];
  float errVelNED[3], errVelFRD[3];
  float vel[3];
  float f_rotmat_v2n[3][3] = { 0.f };
  float f_rotmat_n2v[3][3] = { 0.f };
  float f_eluer_v[3];
  RefInfo refInfo = ref_GetReferenceInfo(ins->d_sys_timestamp);
  float* pf_imu2gnss = lvrm_get_imu2gnss();

  if (refInfo.tow < 0)
  {
    return -1;
  }

  blh[0] = ins->d_poslla[0];
  blh[1] = ins->d_poslla[1];
  blh[2] = ins->d_poslla[2];
  vel[0] = ins->f_vel_ned[0];
  vel[1] = ins->f_vel_ned[1];
  vel[2] = ins->f_vel_ned[2];
  refBlh[0] = refInfo.lat * DEG2RAD;
  refBlh[1] = refInfo.lon * DEG2RAD;
  refBlh[2] = refInfo.alt;
  compensate_pos_lever_arm(blh, pf_imu2gnss, BLH_IMU2XPOS);
  compensate_vel_lever_arm(vel, pf_imu2gnss, BLH_IMU2XPOS);
  math_matmul(*ins->f_rotmat_b2n, *ins->f_rotmat_v2b, 3, 3, 3, *f_rotmat_v2n);
  math_mattrns(&(f_rotmat_v2n[0][0]), 3, 3, &(f_rotmat_n2v[0][0]));
  tf_dcm2eluer(f_rotmat_v2n, f_eluer_v);
  /* NED pos error */
  tf_GetNedDiffPos(blh, refBlh, errPosNED);
  error[0] = fabs(errPosNED[0]);
  error[1] = fabs(errPosNED[1]);
  error[2] = fabs(errPosNED[2]);
  /* NED vel error */
  errVelNED[0] = (float)(refInfo.vn - vel[0]);
  errVelNED[1] = (float)(refInfo.ve - vel[1]);
  errVelNED[2] = (float)(refInfo.vd - vel[2]);
  math_matmul(&(f_rotmat_n2v[0][0]), errVelNED, 3, 3, 1, errVelFRD);
  error[3] = fabs(errVelNED[0]);
  error[4] = fabs(errVelNED[1]);
  error[5] = fabs(errVelNED[2]);
  /* FRD vel error */
  error[6] = fabs(errVelFRD[0]);
  error[7] = fabs(errVelFRD[1]);
  error[8] = fabs(errVelFRD[2]);
  /* Cvn att error */
  double heading = tf_GetHeadDiff(refInfo.heading, f_eluer_v[2] * RAD2DEG + 360);
  error[9] = fabs(refInfo.roll - f_eluer_v[0] * RAD2DEG);
  error[10] = fabs(refInfo.pitch - f_eluer_v[1] * RAD2DEG);
  error[11] = fabs(heading);
  return 1;
}

int ref_CalInsError(NavParams_t* ins, double* error)
{
  double blh[3], refBlh[3], errPosNED[3];
  float vel_n_err[3], vel_v_err[3];
  float vel[3];
  float f_rotmat_v2n[3][3] = { 0.f };
  float f_rotmat_n2v[3][3] = { 0.0 };
  float f_eluer_v[3];
  RefInfo refInfo = ref_GetReferenceInfo(ins->d_sys_timestamp);
  float* pf_imu2rear = lvrm_get_imu2rearmid();

  if (refInfo.tow < 0)
  {
      return -1;
  }
  blh[0] = ins->d_poslla[0];
  blh[1] = ins->d_poslla[1];
  blh[2] = ins->d_poslla[2];
  vel[0] = ins->f_vel_ned[0];
  vel[1] = ins->f_vel_ned[1];
  vel[2] = ins->f_vel_ned[2];
  refBlh[0] = refInfo.lat * DEG2RAD;
  refBlh[1] = refInfo.lon * DEG2RAD;
  refBlh[2] = refInfo.alt;
  compensate_pos_lever_arm(blh, pf_imu2rear, BLH_IMU2XPOS);
  compensate_vel_lever_arm(vel, pf_imu2rear, BLH_IMU2XPOS);
  math_matmul(*ins->f_rotmat_b2n, *ins->f_rotmat_v2b, 3, 3, 3, *f_rotmat_v2n);
  tf_dcm2eluer(f_rotmat_v2n, f_eluer_v);
  /* NED pos error */
  tf_GetNedDiffPos(blh, refBlh, errPosNED);
  error[0] = fabs(errPosNED[0]);
  error[1] = fabs(errPosNED[1]);
  error[2] = fabs(errPosNED[2]);
  /* NED vel error */
  vel_n_err[0] = (refInfo.vn - vel[0]);
  vel_n_err[1] = (refInfo.ve - vel[1]);
  vel_n_err[2] = (refInfo.vd - vel[2]);
  math_mattrns(&(f_rotmat_v2n[0][0]), 3, 3, &(f_rotmat_n2v[0][0]));
  math_matmul(&(f_rotmat_n2v[0][0]), vel_n_err, 3, 3, 1, vel_v_err);
  error[3] = fabs(vel_v_err[0]);
  error[4] = fabs(vel_v_err[1]);
  error[5] = fabs(vel_v_err[2]);
  /* Cvn att error */
  double heading = tf_GetHeadDiff(refInfo.heading, f_eluer_v[2] * RAD2DEG + 360);
  error[6] = fabs(refInfo.roll - f_eluer_v[0] * RAD2DEG);
  error[7] = fabs(refInfo.pitch - f_eluer_v[1] * RAD2DEG);
  error[8] = fabs(heading);
  return 1;
}

/* The reference is at the antenna */
#ifdef TC_PRDR
void ref_CalPrRes(AsensingGNSSPsrMeas_t* pz_psr_meas)
{
  double refBlh[3] = { 0.0 };
  double refEcef[3] = { 0.0 };
  double satEcef[3] = { 0.0 };
  double prRes[MAX_RAW_OBS] = { 0.0 };
  double distance = 0.0;
  double midRes = 0.0;
  uint8_t i;
  uint8_t j = 0;
  uint8_t satNum = 0;
  RefInfo refInfo = ref_GetReferenceInfo(pz_psr_meas->d_cur_tow);

  refBlh[0] = refInfo.lat * DEG2RAD;
  refBlh[1] = refInfo.lon * DEG2RAD;
  refBlh[2] = refInfo.alt;
  tf_lla2ecef(refBlh, refEcef, refBlh);
  for (i = 0; i < pz_psr_meas->u_count; i++)
  {
    if ((pz_psr_meas->z_raw_data[i].u_valid & MEAS_TYPE_PR) != MEAS_TYPE_PR)
    {
      continue;
    }
    if (pz_psr_meas->z_raw_data[i].f_ele < 10)
    {
      continue;
    }
    if (pz_psr_meas->z_raw_data[i].u_snr < 20)
    {
      continue;
    }
    satEcef[0] = pz_psr_meas->z_raw_data[i].d_sat_pos[0];
    satEcef[1] = pz_psr_meas->z_raw_data[i].d_sat_pos[1];
    satEcef[2] = pz_psr_meas->z_raw_data[i].d_sat_pos[2];
    distance = sqrt((satEcef[0] - refEcef[0]) * (satEcef[0] - refEcef[0]) + (satEcef[1] - refEcef[1]) * (satEcef[1] - refEcef[1]) +
      (satEcef[2] - refEcef[2]) * (satEcef[2] - refEcef[2]));
    prRes[satNum] = pz_psr_meas->z_raw_data[i].d_pr - distance;
    satNum++;
  }
  midRes = calMidRes(prRes, satNum);
  LOGE(TAG_VDR, "current epoch sat num is %d\n", pz_psr_meas->u_count);
  LOGE(TAG_VDR, "         valid sys prn cn0   ele    azi   prRes   pr_R\n");
  for (i = 0; i < pz_psr_meas->u_count; i++)
  {
    if ((pz_psr_meas->z_raw_data[i].u_valid & MEAS_TYPE_PR) != MEAS_TYPE_PR)
    //if (FloatEqual(pz_psr_meas->z_raw_data[i].d_sat_pos[0], 0.0) || FloatEqual(pz_psr_meas->z_raw_data[i].d_sat_pos[1], 0.0) ||
    //  FloatEqual(pz_psr_meas->z_raw_data[i].d_sat_pos[2], 0.0))
    {
      pz_psr_meas->z_raw_data[i].d_refPrRes = -999;
    }
    else
    {
      satEcef[0] = pz_psr_meas->z_raw_data[i].d_sat_pos[0];
      satEcef[1] = pz_psr_meas->z_raw_data[i].d_sat_pos[1];
      satEcef[2] = pz_psr_meas->z_raw_data[i].d_sat_pos[2];
      distance = sqrt((satEcef[0] - refEcef[0]) * (satEcef[0] - refEcef[0]) + (satEcef[1] - refEcef[1]) * (satEcef[1] - refEcef[1]) +
        (satEcef[2] - refEcef[2]) * (satEcef[2] - refEcef[2]));
      pz_psr_meas->z_raw_data[i].d_refPrRes = pz_psr_meas->z_raw_data[i].d_pr - distance - midRes;
    }
    LOGE(TAG_VDR, "$SATINFO %5d %3d %3d %3d %5.2f %6.2f %7.2f %6.2f\n",
      pz_psr_meas->z_raw_data[i].u_valid, pz_psr_meas->z_raw_data[i].u_sys, pz_psr_meas->z_raw_data[i].u_prn,
      pz_psr_meas->z_raw_data[i].u_snr, pz_psr_meas->z_raw_data[i].f_ele, pz_psr_meas->z_raw_data[i].f_azi,
      pz_psr_meas->z_raw_data[i].d_refPrRes, sqrt(pz_psr_meas->z_raw_data[i].f_prnoise));
  }
}
#endif

void ref_DecodeBlhData(char* data, RefInfo* res)
{
  RefInfo refData = { 0 };
  double degree, minute, second;
  char* temp = NULL;

  temp = strtok(data, " ");
  res->tow = atof(temp);

  temp = strtok(NULL, " ");
  degree = atof(temp);
  temp = strtok(NULL, " ");
  minute = atof(temp);
  temp = strtok(NULL, " ");
  second = atof(temp);
  res->lat = degree + minute / 60 + second / 60 / 60;

  temp = strtok(NULL, " ");
  degree = atof(temp);
  temp = strtok(NULL, " ");
  minute = atof(temp);
  temp = strtok(NULL, " ");
  second = atof(temp);
  res->lon = degree + minute / 60 + second / 60 / 60;

  temp = strtok(NULL, " ");
  res->height = atof(temp);

  temp = strtok(NULL, " ");
  res->heading = atof(temp);

  temp = strtok(NULL, " ");
  res->pitch = atof(temp);

  temp = strtok(NULL, " ");
  res->roll = atof(temp);

  temp = strtok(NULL, " ");
  temp = strtok(NULL, " ");
  res->vn = atof(temp);

  temp = strtok(NULL, " ");
  res->ve = atof(temp);

  temp = strtok(NULL, " ");
  res->vd = atof(temp) * (-1);

  temp = strtok(NULL, " ");
  res->alt = atof(temp);
}

void ref_ModuleInit(FILE* fp)
{
  memset(&gz_refList, 0, sizeof(RefList));
  while (!feof(fp))
  {
    char line[1024] = { 0 };
    fgets(line, sizeof(line), fp);
    if (!isdigit(line[0]) || strlen(line) < 11)
    {
      continue;
    }
    gz_refList.num++;
  }
  gz_refList.data = (RefInfo*)malloc(sizeof(RefInfo) * gz_refList.num);
  fseek(fp, 0, SEEK_SET);

  int i = 0;
  while (!feof(fp))
  {
    char line[1024] = { 0 };
    fgets(line, sizeof(line), fp);
    if (!isdigit(line[0]) || strlen(line) < 11)
    {
      continue;
    }
    RefInfo res = { 0 };
    ref_DecodeBlhData(line, &res);
    memcpy(&gz_refList.data[i], &res, sizeof(RefInfo));
    i++;
  }
}

RefInfo ref_GetReferenceInfo(double t)
{
  int left = 0;
  int right = gz_refList.num;
  int mid = 0;
  RefInfo midInfo;

  if (fabs(t - gz_refList.data[0].tow) < 0.005)
  {
    return gz_refList.data[0];
  }
  if (fabs(t - gz_refList.data[gz_refList.num - 1].tow) < 0.005)
  {
    return gz_refList.data[gz_refList.num - 1];
  }

  while (left <= right)
  {
    mid = (left + right) / 2;
    if (mid < 1)
    {
      break;
    }
    memcpy(&midInfo, &gz_refList.data[mid - 1], sizeof(RefInfo));
    if (t - midInfo.tow > 0.005)
    {
      left = mid + 1;
    }
    else if (t - midInfo.tow < -0.005)
    {
      right = mid - 1;
    }
    else
    {
      return midInfo;
    }
  }
  midInfo.tow = -1;

  return midInfo;
}

RefInfo ref_GetReferenceInfoFast(double t)
{
  int id;

  id = t * REF_FREQ - gz_refList.data[0].tow * REF_FREQ + 1;
  return gz_refList.data[id - 1];
}

int sortInt(const void* a, const void* b)
{
  int* m = (int*)a;
  int* n = (int*)b;

  if ((*m) < (*n))
  {
    return -1;
  }
  return 1;
}

int sortFloat(const void* a, const void* b)
{
  float* m = (float*)a;
  float* n = (float*)b;

  if ((*m) < (*n))
  {
    return -1;
  }
  return 1;
}

float calFloatMean(float* data, int8_t len)
{
  float res = 0.0;

  if (len <= 0)
  {
    return 0.0;
  }

  for (int i = 0; i < len; i++)
  {
    res = res + data[i];
  }
  res = res / len;
  return res;
}

float calFloatVar(float* data, float mean, int8_t len)
{
  float res = 0.0;

  if (len <= 1)
  {
    return 0.0;
  }

  for (int i = 0; i < len; i++)
  {
    res = res + (data[i] - mean) * (data[i] - mean);
  }
  res = res / (len - 1);
  res = sqrtf(res);
  return res;
}

void Asm_Mag(Complex *x, float* Mag)
{
  for (int i = 0; i < 32; i++)
  {
    Mag[i] = sqrtf(x[i].Re * x[i].Re + x[i].Im * x[i].Im) * 2 / 64;
    if (i == 0)
    {
      Mag[i] = Mag[i] / 2;
    }
  }
}

void fft(Complex* v, int n, Complex* tmp)
{
  if (n > 1)
  {
    int k, m;
    Complex z, w, *vo, *ve;

    ve = tmp;
    vo = tmp + n / 2;
    for (k = 0; k < n / 2; k++)
    {
      ve[k] = v[2 * k];
      vo[k] = v[2 * k + 1];
    }
    fft(ve, n / 2, v);
    fft(vo, n / 2, v);
    for (m = 0; m < n / 2; m++)
    {
      w.Re = cos(2 * PI * m / (double)n);
      w.Im = -sin(2 * PI * m / (double)n);
      z.Re = w.Re * vo[m].Re - w.Im * vo[m].Im;
      z.Im = w.Re * vo[m].Im + w.Im * vo[m].Re;
      v[m].Re = ve[m].Re + z.Re;
      v[m].Im = ve[m].Im + z.Im;
      v[m + n / 2].Re = ve[m].Re - z.Re;
      v[m + n / 2].Im = ve[m].Im - z.Im;
    }
  }
  return;
}

void CalTimeDomainFeature(float imuData[4][64], MlFeature* mlFea)
{
  float sortData[64];

  memcpy(sortData, &imuData[0][0], sizeof(float) * 64);
  qsort(sortData, 64, sizeof(float), sortFloat);
  mlFea->h_gyro_max = sortData[63];
  mlFea->h_gyro_min = sortData[0];
  mlFea->h_gyro_mean = calFloatMean(sortData, 64);
  mlFea->h_gyro_var = calFloatVar(sortData, mlFea->h_gyro_mean, 64);

  memcpy(sortData, &imuData[1][0], sizeof(float) * 64);
  qsort(sortData, 64, sizeof(float), sortFloat);
  mlFea->v_gyro_max = sortData[63];
  mlFea->v_gyro_min = sortData[0];
  mlFea->v_gyro_mean = calFloatMean(sortData, 64);
  mlFea->v_gyro_var = calFloatVar(sortData, mlFea->v_gyro_mean, 64);

  memcpy(sortData, &imuData[2][0], sizeof(float) * 64);
  qsort(sortData, 64, sizeof(float), sortFloat);
  mlFea->h_acc_max = sortData[63];
  mlFea->h_acc_min = sortData[0];
  mlFea->h_acc_mean = calFloatMean(sortData, 64);
  mlFea->h_acc_var = calFloatVar(sortData, mlFea->h_acc_mean, 64);

  memcpy(sortData, &imuData[3][0], sizeof(float) * 64);
  qsort(sortData, 64, sizeof(float), sortFloat);
  mlFea->v_acc_max = sortData[63];
  mlFea->v_acc_min = sortData[0];
  mlFea->v_acc_mean = calFloatMean(sortData, 64);
  mlFea->v_acc_var = calFloatVar(sortData, mlFea->v_acc_mean, 64);
}

void CalFrqDomainFeature(float imuData[4][64], MlFeature* mlFea)
{
  Complex v[64], scratch[64];
  float Mag[32];

  for (int k = 0; k < 64; k++)
  {
    v[k].Re = imuData[0][k];
    v[k].Im = 0;
  }
  fft(v, 64, scratch);
  Asm_Mag(v, Mag);
  mlFea->h_gyro_dc_component = Mag[0];
  mlFea->h_gyro_mag_avg = calFloatMean(Mag, 32);
  mlFea->h_gyro_mag_var = calFloatVar(Mag, mlFea->h_gyro_mag_avg, 32);

  for (int k = 0; k < 64; k++)
  {
    v[k].Re = imuData[1][k];
    v[k].Im = 0;
  }
  fft(v, 64, scratch);
  Asm_Mag(v, Mag);
  mlFea->v_gyro_dc_component = Mag[0];
  mlFea->v_gyro_mag_avg = calFloatMean(Mag, 32);
  mlFea->v_gyro_mag_var = calFloatVar(Mag, mlFea->v_gyro_mag_avg, 32);

  for (int k = 0; k < 64; k++)
  {
    v[k].Re = imuData[2][k];
    v[k].Im = 0;
  }
  fft(v, 64, scratch);
  Asm_Mag(v, Mag);
  mlFea->h_acc_dc_component = Mag[0];
  mlFea->h_acc_mag_avg = calFloatMean(Mag, 32);
  mlFea->h_acc_mag_var = calFloatVar(Mag, mlFea->h_acc_mag_avg, 32);

  for (int k = 0; k < 64; k++)
  {
    v[k].Re = imuData[3][k];
    v[k].Im = 0;
  }
  fft(v, 64, scratch);
  Asm_Mag(v, Mag);
  mlFea->v_acc_dc_component = Mag[0];
  mlFea->v_acc_mag_avg = calFloatMean(Mag, 32);
  mlFea->v_acc_mag_var = calFloatVar(Mag, mlFea->v_acc_mag_avg, 32);
}

#endif