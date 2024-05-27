/**@file        fusion_axis_detect.c
 * @brief		fusion axis detect file
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

#include "fusion_global.h"
#include "fusion_log.h"
#include "fusion_proc.h"
#include "fusion_math.h"
#include "fusion_config.h"
#include "fusion_quat.h"
#include "fusion_transform.h"
#include "fusion_axis_detect.h"
#include "fusion_if.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "mw_log.h"

#define FlEX_AXIAL_DETECTION

static FlxAxisResult_t gz_axis_result;
static FlxAxisDectct_t gz_axis_detect;

FlxAxisResult_t* fusion_get_flx_result(void)
{
  return &gz_axis_result;
}

static void flx_convert_vertical_axis(double pd_mea[], FlxAxisDectct_t* pz_flx)
{
  double d_temp[6];

  if (6 != pz_flx->z_vert_axis.u_axis_mode)
  {
    for (int k = 0; k < 6; k++)
    {
      d_temp[k] = pd_mea[2 + k];
    }
    if (1 == pz_flx->z_vert_axis.u_axis_mode) /* x-up 0,0,1,0,1,0,-1,0,0 */
    {
      pd_mea[2] = d_temp[2];
      pd_mea[4] = -d_temp[0];
      pd_mea[5] = d_temp[5];
      pd_mea[7] = -d_temp[3];
    }
    if (2 == pz_flx->z_vert_axis.u_axis_mode) /* x-down 0,0,-1,0,1,0,1,0,0 */
    {
      pd_mea[2] = -d_temp[2];
      pd_mea[4] = d_temp[0];
      pd_mea[5] = -d_temp[5];
      pd_mea[7] = d_temp[3];
    }
    if (3 == pz_flx->z_vert_axis.u_axis_mode) /* y-up 1,0,0,0,0,1,0,-1,0 */
    {
      pd_mea[2] = -d_temp[2];
      pd_mea[3] = d_temp[0];
      pd_mea[4] = -d_temp[1];
      pd_mea[5] = -d_temp[5];
      pd_mea[6] = d_temp[3];
      pd_mea[7] = -d_temp[4];
    }
    if (4 == pz_flx->z_vert_axis.u_axis_mode) /* y-down 1,0,0,0,0,-1,0,1,0 */
    {
      pd_mea[3] = -d_temp[2];
      pd_mea[4] = d_temp[1];
      pd_mea[6] = -d_temp[5];
      pd_mea[7] = d_temp[4];
    }
    if (5 == pz_flx->z_vert_axis.u_axis_mode) /* z-up 1,0,0,0,-1,0,0,0,-1 */
    {
      pd_mea[2] = d_temp[0];
      pd_mea[3] = -d_temp[1];
      pd_mea[4] = -d_temp[2];
      pd_mea[5] = d_temp[3];
      pd_mea[6] = -d_temp[4];
      pd_mea[7] = -d_temp[5];
    }
  }
}

void flx_detect_vertical_axis(double d_mea[], FlxAxisDectct_t* pz_flx_axis)
{
  if (d_mea[0] > 0)
  {
    if (pz_flx_axis->u_is_detected == INS_FALSE)
    {
      VerticalAxis_t* pz_vert = &pz_flx_axis->z_vert_axis;
      uint8_t u_imurate = fusion_get_algconfig()->z_init_para.e_sampling_rate;
      for (int i = 0; i < 3; i++)
      {
        pz_vert->f_acc_xyz[i] = (float)(0.5 * pz_vert->f_acc_xyz[i] + 0.5 * d_mea[5 + i]);
      }

      if (pz_vert->q_count < 1000000000)
      {
        pz_vert->q_count++;
        /* detect forward-right-down axis */
        float f_tnum = (float)(pz_vert->q_count - 1);

        pz_vert->f_acc_tmp[0] = (f_tnum * pz_vert->f_acc_tmp[0] + pz_vert->f_acc_xyz[0]) / (float)pz_vert->q_count;
        pz_vert->f_acc_tmp[1] = (f_tnum * pz_vert->f_acc_tmp[1] + pz_vert->f_acc_xyz[1]) / (float)pz_vert->q_count;
        pz_vert->f_acc_tmp[2] = (f_tnum * pz_vert->f_acc_tmp[2] + pz_vert->f_acc_xyz[2]) / (float)pz_vert->q_count;
      }

      if (pz_vert->q_count >= 20U * u_imurate && pz_vert->u_last_axis_mode > 100)
      {
        float f_maxval = 0.0f;
        uint8_t  u_axis_index = 2;
        for (uint8_t j = 0; j < 3; j++)
        {
          if (fabsf(pz_vert->f_acc_tmp[j]) > f_maxval)
          {
            f_maxval = fabsf((pz_vert->f_acc_tmp[j]));
            u_axis_index = j;
          }
        }

        if (0 == u_axis_index)
        {
          if (pz_vert->f_acc_tmp[0] > 0.0f)
          {
            pz_vert->u_axis_mode = 1; /* x-up */
          }
          else
          {
            pz_vert->u_axis_mode = 2; /* x-down */
          }
        }
        if (1 == u_axis_index)
        {
          if (pz_vert->f_acc_tmp[1] > 0.0f)
          {
            pz_vert->u_axis_mode = 3; /* y-up */
          }
          else
          {
            pz_vert->u_axis_mode = 4; /* y-down */
          }
        }
        if (2 == u_axis_index)
        {
          if (pz_vert->f_acc_tmp[2] > 0.0f)
          {
            pz_vert->u_axis_mode = 5; /* z-up */
          }
          else
          {
            pz_vert->u_axis_mode = 6; /* z-down */
          }
        }
        pz_flx_axis->u_is_detected = INS_TRUE;
        pz_vert->u_last_axis_mode = pz_vert->u_axis_mode;
        LOGI(TAG_VDR, "Down axis detect : pz_vert->u_axis_mode=%d \n", pz_vert->u_axis_mode);
      }
    }
    flx_convert_vertical_axis(d_mea, pz_flx_axis);
  }
}

#ifdef FlEX_AXIAL_DETECTION

FlxAxisDectct_t* fusion_get_flx_axis(void)
{
  return &gz_axis_detect;
}

void flx_axis_init(void)
{
  memset(&gz_axis_detect, 0, sizeof(gz_axis_detect));
  gz_axis_detect.z_vert_axis.u_axis_mode = 6;
  gz_axis_detect.z_vert_axis.u_last_axis_mode = 255;
}

static void flx_calculate_static(float* accHP)
{
  AcclHPStatic_t* ptr = &gz_axis_detect.z_acc_static;

  if (is_izupt() == 0)
  {
    if ((ptr->preZuptFlag == 1U) && (ptr->cnt >= FLEX_AXIS_STATIC_BUF_LEN))
    {
      for (int32_t i = 0; i < 3; i++)
      {
        ptr->staticValue[i] = math_calculate_mean(ptr->buf[i], (int32_t)FLEX_AXIS_STATIC_BUF_LEN);
      }
    }
  }
  else
  {
    if (ptr->cnt < FLEX_AXIS_STATIC_BUF_LEN)
    {
      ptr->buf[0][ptr->cnt] = accHP[0];
      ptr->buf[1][ptr->cnt] = accHP[1];
      ptr->buf[2][ptr->cnt] = accHP[2];
      ptr->cnt++;
    }
    else
    {
      for (int32_t i = 0; i < 3; i++)
      {
        memmove(&ptr->buf[i][0], &ptr->buf[i][1], sizeof(float) * (FLEX_AXIS_STATIC_BUF_LEN - 1U));
        ptr->buf[i][FLEX_AXIS_STATIC_BUF_LEN - 1U] = accHP[i];
      }
    }
  }
  ptr->preZuptFlag = (uint8_t)is_izupt();
  accHP[0] -= ptr->staticValue[0];
  accHP[1] -= ptr->staticValue[1];
  accHP[2] -= ptr->staticValue[2];
}

void flx_filter_imu_meas(void)
{
  gz_axis_detect.q_fliter_cnt++;

  for (int32_t i = 0; i < 3; i++)
  {
    gz_axis_detect.f_gyrolp[i] = (gz_axis_detect.f_gyrolp[i] * ((float)gz_axis_detect.q_fliter_cnt - 1.0f) / ((float)gz_axis_detect.q_fliter_cnt))
        + (gz_axis_detect.z_imu_cpy.gyro[i] / ((float)gz_axis_detect.q_fliter_cnt));

    gz_axis_detect.f_acclp[i] = (gz_axis_detect.f_acclp[i] * ((float)gz_axis_detect.q_fliter_cnt - 1.0f) / ((float)gz_axis_detect.q_fliter_cnt))
        + (gz_axis_detect.z_imu_cpy.acc[i] / ((float)gz_axis_detect.q_fliter_cnt));
  }

  /* Similar to high-pass filtering,Eliminate zero bias projection on each axis of the gyro */
  math_matsub(gz_axis_detect.z_imu_cpy.gyro, gz_axis_detect.f_gyrolp, 3, 1, gz_axis_detect.z_imu_cpy.gyro);
  /* Similar to high - pass filtering, Eliminate the projection of gravity and zero bias on all ACC axes */
  math_matsub(gz_axis_detect.z_imu_cpy.acc, gz_axis_detect.f_acclp, 3, 1, gz_axis_detect.z_imu_cpy.acc);
  flx_calculate_static(gz_axis_detect.z_imu_cpy.acc);
}

static void flx_prepare_imu_meas(const double* pd_meas,double d_ppsgap)
{
  double d_pretime = gz_axis_detect.z_imu_cpy.timeTag;

  gz_axis_detect.u_iready_flag = 0;
  gz_axis_detect.f_preodo_val = gz_axis_detect.z_imu_cpy.odometer;
  memcpy(&gz_axis_detect.z_imu_cpy, &gz_axis_detect.z_imumsg, sizeof(gz_axis_detect.z_imumsg));
  memset(&gz_axis_detect.z_imumsg, 0, sizeof(gz_axis_detect.z_imumsg));
    
  if (gz_axis_detect.z_imu_cpy.cnt > 0)
  {
    if (d_pretime < 1.0)
    {
      gz_axis_detect.u_iready_flag = 0;
    }
    else
    {
      gz_axis_detect.u_iready_flag = 1;
    }

    for (int32_t i = 0; i < 3; i++)
    {
      gz_axis_detect.z_imu_cpy.gyro[i] /= (float)gz_axis_detect.z_imu_cpy.cnt;
      gz_axis_detect.z_imu_cpy.acc[i] /= (float)gz_axis_detect.z_imu_cpy.cnt;
    }

    gz_axis_detect.z_imu_cpy.odometer /= (float)gz_axis_detect.z_imu_cpy.cnt;

    if (gz_axis_detect.u_dataloss_flag != 0U) /* probably missing one PPS */
    {
      gz_axis_detect.u_iready_flag = 0;
    }

    if (gz_axis_detect.u_iready_flag != 0U)
    {
      flx_filter_imu_meas();
    }

    gz_axis_detect.u_dataloss_flag = 0;
  }
}

static void flx_prepare_gnss_meas(double* pd_meas)
{
  GNSSMsg_t* pz_gmsg = &gz_axis_detect.z_gnssmsg;
  uint8_t  gps_flag = *((uint8_t*)(&pd_meas[14]));
  const float* gyro = gz_axis_detect.z_imu_cpy.gyro;
  float angle = sqrtf((gyro[0] * gyro[0]) + (gyro[1] * gyro[1]) + (gyro[2] * gyro[2]));
  float angleDiff = 0.0f;

  float preVel = pz_gmsg->vel;
  uint8_t preQualityFlag = pz_gmsg->qualityFlag;
  float timegap = (pz_gmsg->timeTag > 1.0) ? ((float)(pd_meas[1] - pz_gmsg->timeTag)) : 1.0f;

  pz_gmsg->vel = (float)sqrt((pd_meas[5] * pd_meas[5]) + (pd_meas[6] * pd_meas[6]));
  pz_gmsg->acc = (pz_gmsg->vel - preVel) / timegap;

  if (pz_gmsg->flexAxisAngleBufCnt < (uint8_t)FLEX_AXIS_ANGLE_BUF_LEN)
  {
    pz_gmsg->flexAxisAngleBuf[pz_gmsg->flexAxisAngleBufCnt] = angle;
    pz_gmsg->flexAxisAngleBufCnt++;
  }
  else
  {
    memmove(pz_gmsg->flexAxisAngleBuf, &pz_gmsg->flexAxisAngleBuf[1], sizeof(float) * ((uint8_t)FLEX_AXIS_ANGLE_BUF_LEN - 1U));
    pz_gmsg->flexAxisAngleBuf[FLEX_AXIS_ANGLE_BUF_LEN - 1] = angle;
  }

  for (int i = 0; i < FLEX_AXIS_ANGLE_BUF_LEN; i++)
  {
    angleDiff += pz_gmsg->flexAxisAngleBuf[i];
  }

  if ((gps_flag > 0U) && (pd_meas[16] < 2.0) && (pd_meas[19] > 20.0))
  {
    pz_gmsg->qualityFlag = 1U;
  }
  else
  {
    pz_gmsg->qualityFlag = 0U;
  }

  if (angle < 5.f * (float)DEG2RAD && (pz_gmsg->flexAxisAngleBufCnt >= (uint8_t)FLEX_AXIS_ANGLE_BUF_LEN &&
    fabsf(angleDiff) < 15.f * (float)DEG2RAD) && preQualityFlag > 0U && pz_gmsg->qualityFlag > 0U &&
    pz_gmsg->vel > 1.0f && fabsf(pz_gmsg->acc) > 1.0f)
  {
    gz_axis_detect.u_gready_flag = 1U;
  }
  else
  {
    gz_axis_detect.u_gready_flag = 0U;
  }

  if (pz_gmsg->timeTag < 1.0)
  {
    gz_axis_detect.u_gready_flag = 0U;
  }
  pz_gmsg->timeTag = pd_meas[1];
}

static void flx_prapare_meas_buffer(void)
{
  IMUMsgBuf_t* pz_imsgbuf = &gz_axis_detect.z_imubuf;
  GNSSMsgBuf_t* pz_gmsgbuf = &gz_axis_detect.z_gnssbuf;
  const IMUMsg_t* pz_imsg = &gz_axis_detect.z_imu_cpy;
  const GNSSMsg_t* pz_gmsg = &gz_axis_detect.z_gnssmsg;

  if (pz_imsgbuf->fullFlag != 0U)
  {
    memmove(&pz_imsgbuf->accData[0][0], &pz_imsgbuf->accData[0][1], sizeof(float) * (FLEX_AXIS_FIT_WINDOW_LENGTH - 1U));
    memmove(&pz_imsgbuf->accData[1][0], &pz_imsgbuf->accData[1][1], sizeof(float) * (FLEX_AXIS_FIT_WINDOW_LENGTH - 1U));
    memmove(&pz_imsgbuf->accData[2][0], &pz_imsgbuf->accData[2][1], sizeof(float) * (FLEX_AXIS_FIT_WINDOW_LENGTH - 1U));
    memmove(&pz_imsgbuf->odo[0], &pz_imsgbuf->odo[1], sizeof(float) * (FLEX_AXIS_FIT_WINDOW_LENGTH - 1U));
    pz_imsgbuf->accData[0][FLEX_AXIS_FIT_WINDOW_LENGTH - 1U] = pz_imsg->acc[0];
    pz_imsgbuf->accData[1][FLEX_AXIS_FIT_WINDOW_LENGTH - 1U] = pz_imsg->acc[1];
    pz_imsgbuf->accData[2][FLEX_AXIS_FIT_WINDOW_LENGTH - 1U] = pz_imsg->acc[2];
    pz_imsgbuf->odo[FLEX_AXIS_FIT_WINDOW_LENGTH - 1U] = pz_imsg->odometer;
  }
  else
  {
    pz_imsgbuf->accData[0][pz_imsgbuf->cnt] = pz_imsg->acc[0];
    pz_imsgbuf->accData[1][pz_imsgbuf->cnt] = pz_imsg->acc[1];
    pz_imsgbuf->accData[2][pz_imsgbuf->cnt] = pz_imsg->acc[2];
    pz_imsgbuf->odo[pz_imsgbuf->cnt] = pz_imsg->odometer;
    pz_imsgbuf->cnt++;
    if (pz_imsgbuf->cnt >= FLEX_AXIS_FIT_WINDOW_LENGTH)
    {
      pz_imsgbuf->fullFlag = 1;
    }
  }

  if (pz_gmsgbuf->fullFlag != 0U)
  {
    memmove(&pz_gmsgbuf->data[0], &pz_gmsgbuf->data[1], sizeof(float) * (FLEX_AXIS_FIT_WINDOW_LENGTH - 1U));
    pz_gmsgbuf->data[FLEX_AXIS_FIT_WINDOW_LENGTH - 1U] = pz_gmsg->acc;
    memmove(&pz_gmsgbuf->timeTag[0], &pz_gmsgbuf->timeTag[1], sizeof(double) * (FLEX_AXIS_FIT_WINDOW_LENGTH - 1U));
    pz_gmsgbuf->timeTag[FLEX_AXIS_FIT_WINDOW_LENGTH - 1U] = pz_gmsg->timeTag;
  }
  else
  {
    pz_gmsgbuf->data[pz_gmsgbuf->cnt] = pz_gmsg->acc;
    pz_gmsgbuf->timeTag[pz_gmsgbuf->cnt] = pz_gmsg->timeTag;
    pz_gmsgbuf->cnt++;

    if (pz_gmsgbuf->cnt >= FLEX_AXIS_FIT_WINDOW_LENGTH)
    {
      pz_gmsgbuf->fullFlag = 1;
    }
  }
}

static void flx_ls_fit(const double* pd_meas)
{
  float f_imu_mean[3] = { 0.0f };
  float f_imu_std[3] = { 0.0f };
  float f_gnss_mean = 0.0f;
  float f_gnss_std = 0.0f;
  float f_igmean[3] = { 0.0f };
  float f_igstd[3] = { 0.0f };
  float f_igbuf[3][FLEX_AXIS_FIT_WINDOW_LENGTH] = { 0.0f };
  float bA[3] = { 0.0f };
  float kA[3] = { 0.0f };
  float rA[3] = { 0.0f };
  float k_m[3] = { 0.0f };
  IMUMsgBuf_t* pz_imsgbuf = &gz_axis_detect.z_imubuf;
  GNSSMsgBuf_t* pz_gmsgbuf = &gz_axis_detect.z_gnssbuf;
  float f_gnss2buf[FLEX_AXIS_FIT_WINDOW_LENGTH] = { 0.0f };
  float f_gnss2mean = 0.0f;
  float f_gnss2std = 0.0f;

  for (int32_t i = 0; i < 3; i++)
  {
    for (uint32_t j = 0; j < FLEX_AXIS_FIT_WINDOW_LENGTH; j++)
    {
      f_igbuf[i][j] = pz_imsgbuf->accData[i][j] * pz_gmsgbuf->data[j];
    }
  }

  for (uint32_t j = 0; j < FLEX_AXIS_FIT_WINDOW_LENGTH; j++)
  {
    f_gnss2buf[j] = pz_gmsgbuf->data[j] * pz_gmsgbuf->data[j];
  }

  for (int32_t i = 0; i < 3; i++)
  {
    math_calculate_std(pz_imsgbuf->accData[i], FLEX_AXIS_FIT_WINDOW_LENGTH, &f_imu_mean[i], &f_imu_std[i]);
    math_calculate_std(f_igbuf[i], FLEX_AXIS_FIT_WINDOW_LENGTH, &f_igmean[i], &f_igstd[i]);
  }

  math_calculate_std(pz_gmsgbuf->data, FLEX_AXIS_FIT_WINDOW_LENGTH, &f_gnss_mean, &f_gnss_std);
  math_calculate_std(f_gnss2buf, FLEX_AXIS_FIT_WINDOW_LENGTH, &f_gnss2mean, &f_gnss2std);

  for (int32_t i = 0; i < 3; i++)
  {
    bA[i] = ((((f_gnss_mean * f_gnss_mean) + (f_gnss_std * f_gnss_std)) * f_imu_mean[i]) - (f_gnss_mean * f_igmean[i]))
        / (f_gnss_std * f_gnss_std);
    kA[i] = (f_igmean[i] - (f_imu_mean[i] * f_gnss_mean)) / (f_gnss_std * f_gnss_std);
    rA[i] = (f_igmean[i] - (f_gnss_mean * f_imu_mean[i])) / (f_imu_std[i] * f_gnss_std);
    k_m[i] = f_imu_mean[i] / f_gnss_mean;
  }
#ifdef FUSION_POST_PROCESS
  (void)printf("FlxLSFit:%.3lf,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
      pd_meas[1], kA[0], bA[0], kA[1], bA[1], kA[2], bA[2], rA[0], k_m[0]);
#endif
  memcpy(gz_axis_detect.z_ls_para.bA, bA, sizeof(bA));
  memcpy(gz_axis_detect.z_ls_para.kA, kA, sizeof(kA));
  memcpy(gz_axis_detect.z_ls_para.k_m, k_m, sizeof(k_m));
  memcpy(gz_axis_detect.z_ls_para.rA, rA, sizeof(rA));
}

static void flx_calculate_axis_flag(void)
{
  float* bA = gz_axis_detect.z_ls_para.bA;
  float* kA = gz_axis_detect.z_ls_para.kA;
  float* rA = gz_axis_detect.z_ls_para.rA;

  /* Fitting linear function  with least square */
  float kAmax = 0.0f;
  float rAmax = 0.0f;
  uint8_t kAmaxAxis = 0;
  uint8_t rAmaxAxis = 0;

  for (uint8_t i = 0; i < 2U; i++)
  {
    if (fabsf(kA[i]) > kAmax)
    {
      kAmax = fabsf(kA[i]);
      kAmaxAxis = i;
    }

    if (fabsf(rA[i]) > rAmax)
    {
      rAmax = fabsf(rA[i]);
      rAmaxAxis = i;
    }
  }

  if (kAmaxAxis == rAmaxAxis)
  {
    if ((fabsf(kA[kAmaxAxis]) > 0.5f) && (kA[kAmaxAxis] < 1.5f) &&
      (fabsf(bA[kAmaxAxis]) < 0.25f))
    {
      if ((rA[kAmaxAxis] > 0.7f) && (kAmaxAxis == 0U))
      {
        gz_axis_detect.u_axis_flag = 1;
      }
      if ((rA[kAmaxAxis] < -0.7f) && (kAmaxAxis == 0U))
      {
        gz_axis_detect.u_axis_flag = 3;
      }
      if ((rA[kAmaxAxis] > 0.7f) && (kAmaxAxis == 1U))
      {
        gz_axis_detect.u_axis_flag = 4;
      }
      if ((rA[kAmaxAxis] < -0.7f) && (kAmaxAxis == 1U))
      {
        gz_axis_detect.u_axis_flag = 2;
      }
      if (gz_axis_detect.u_axis_flag >= 1U && gz_axis_detect.u_axis_flag <= 4U)
      {
        LOGI(TAG_VDR, "Forward and Right axis detect : u_axis_flag=%d \n", gz_axis_detect.u_axis_flag);
      }
    }
  }
}

void flx_config_misalign(NavConfig_t* pz_navconfig,FlxAxisDectct_t* pz_flx)
{
  for (int8_t i = 0; i < 3; i++)
  {
    pz_navconfig->f_init_align[i] = pz_flx->f_miseuler[i];
  }
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  NavParams_t* pz_nav = fusion_get_navpara();
  memcpy(pz_nav->f_mis_align, pz_navconfig->f_init_align, sizeof(pz_nav->f_mis_align));
  quat_euler2quat(pz_navconfig->f_init_align, pz_nav->f_quat_misalign);
  tf_euler2dcm(pz_navconfig->f_init_align, pz_nav->f_rotmat_misalign);
  math_matrix_transpose(*(pz_nav->f_rotmat_misalign),3,3, *(pz_nav->f_rotmat_v2b));
  pz_inav->u_init_align = pz_flx->u_misalign_flag;
}

static void flx_calculate_miseuler(FlxAxisDectct_t* pz_flx)
{
  float temp1[3] = { 0.0f };
  float temp2[3] = { 0.0f };
  float temp3[3] = { 0.0f };
  float f_gyrolp[3] = { 0.0f };
  float f_acclp[3] = { 0.0f };
  const float* kA = pz_flx->z_ls_para.kA;
  memcpy(temp3, pz_flx->f_acclp, sizeof(pz_flx->f_acclp));
  memset(pz_flx->f_axiscfg, 0, sizeof(pz_flx->f_axiscfg));

  for (int32_t i = 0; i < 3; i++)
  {
    temp3[i] += pz_flx->z_acc_static.staticValue[i];
  }

  for (int32_t i = 0; i < 3; i++)
  {
    temp1[i] = temp3[i];
    temp2[i] = kA[i] / sqrtf((kA[0] * kA[0]) + (kA[1] * kA[1]) + (kA[2] * kA[2]));
  }

  if (1U == pz_flx->u_axis_flag)
  {
    pz_flx->f_axiscfg[0] = 1.0f;
    pz_flx->f_axiscfg[4] = 1.0f;
    pz_flx->f_axiscfg[8] = 1.0f;
  }
  if (2U == pz_flx->u_axis_flag)
  {
    pz_flx->f_axiscfg[1] = -1.0f;
    pz_flx->f_axiscfg[3] = 1.0f;
    pz_flx->f_axiscfg[8] = 1.0f;
  }
  if (3U == pz_flx->u_axis_flag)
  {
    pz_flx->f_axiscfg[0] = -1.0f;
    pz_flx->f_axiscfg[4] = -1.0f;
    pz_flx->f_axiscfg[8] = 1.0f;
  }
  if (4U == pz_flx->u_axis_flag)
  {
    pz_flx->f_axiscfg[1] = 1.0f;
    pz_flx->f_axiscfg[3] = -1.0f;
    pz_flx->f_axiscfg[8] = 1.0f;
  }

  float c2[3] = { 0.0f };
  math_matmul(pz_flx->f_axiscfg, temp1, 3, 3, 1, f_acclp);
  math_matmul(pz_flx->f_axiscfg, pz_flx->f_gyrolp, 3, 3, 1, f_gyrolp);
  math_matmul(pz_flx->f_axiscfg, temp2, 3, 3, 1, c2);
  float roll = atan2f(-f_acclp[1], -f_acclp[2]);
  float pitch = atanf(f_acclp[0] / sqrtf((f_acclp[1] * f_acclp[1]) + (f_acclp[2] * f_acclp[2])));
  float a1 = cosf(pitch);
  float a2 = -cosf(roll);
  float a3 = sinf(roll) * sinf(pitch);
  float a11 = c2[0] / a1;

  if (a11 > 1.0f)
  {
    a11 = 1.0f;
  }
  if (a11 < -1.0f)
  {
    a11 = -1.0f;
  }
  float a22 = (c2[1] - (a3 * a11)) / a2;
  float heading = atan2f(a22, a11);

  pz_flx->f_miseuler[0] = roll;
  pz_flx->f_miseuler[1] = pitch;
  pz_flx->f_miseuler[2] = heading;
  pz_flx->u_misalign_flag = INS_TRUE;
#if FUSION_POST_PROCESS
  (void)printf("misAngle:%f,%f,%f\r\n", pz_flx->f_miseuler[0] * RAD2DEG, pz_flx->f_miseuler[1] * RAD2DEG,
      pz_flx->f_miseuler[2] * RAD2DEG);
#endif
  LOGI(TAG_VDR, "calculate MisAngle after axis detected :%f,%f,%f\r\n", pz_flx->f_miseuler[0] * RAD2DEG, pz_flx->f_miseuler[1] * RAD2DEG,
    pz_flx->f_miseuler[2] * RAD2DEG);

  NavConfig_t* pz_navconfig = fusion_get_navconfig();
  flx_config_misalign(pz_navconfig, pz_flx);
}

static void flx_accumulate_imu_meas(const double pd_meas[])
{
  gz_axis_detect.z_imumsg.timeTag = pd_meas[1];

  for (int32_t i = 0; i < 3; i++)
  {
    gz_axis_detect.z_imumsg.gyro[i] += (float)(pd_meas[2 + i]);
    gz_axis_detect.z_imumsg.acc[i] += (float)(pd_meas[5 + i]);
  }
  gz_axis_detect.z_imumsg.odometer += (float)(pd_meas[8]);
  gz_axis_detect.z_imumsg.cnt++;
}

void flx_convert_imu_axis(double* pd_meas)
{
  double temp[6] = { 0.0f };
  memcpy(temp, pd_meas + 2, sizeof(temp));

  if (1U == gz_axis_detect.u_axis_flag)
  {
    pd_meas[2] = temp[0];
    pd_meas[3] = temp[1];
    pd_meas[4] = temp[2];
    pd_meas[5] = temp[3];
    pd_meas[6] = temp[4];
    pd_meas[7] = temp[5];
  }
  if (2U == gz_axis_detect.u_axis_flag)
  {
    pd_meas[2] = -temp[1];
    pd_meas[3] = temp[0];
    pd_meas[4] = temp[2];
    pd_meas[5] = -temp[4];
    pd_meas[6] = temp[3];
    pd_meas[7] = temp[5];
  }
  if (3U == gz_axis_detect.u_axis_flag)
  {
    pd_meas[2] = -temp[0];
    pd_meas[3] = -temp[1];
    pd_meas[4] = temp[2];
    pd_meas[5] = -temp[3];
    pd_meas[6] = -temp[4];
    pd_meas[7] = temp[5];
  }
  if (4U == gz_axis_detect.u_axis_flag)
  {
    pd_meas[2] = temp[1];
    pd_meas[3] = -temp[0];
    pd_meas[4] = temp[2];
    pd_meas[5] = temp[4];
    pd_meas[6] = -temp[3];
    pd_meas[7] = temp[5];
  }
}

void flx_convert_imu_axis_by_history(HistoryInsResults_t* pz_history, double* pd_mea)
{
  double d_temp[6] = { 0.0f };
  memcpy(d_temp, pd_mea + 2, sizeof(d_temp));

  if (pd_mea[0] < 0)
  {
    return;
  }

  if (pz_history->u_v_axis_mode == 1) /* x-up 0,0,1,0,1,0,-1,0,0 */
  {
    pd_mea[2] = d_temp[2];
    pd_mea[3] = d_temp[1];
    pd_mea[4] = -d_temp[0];
    pd_mea[5] = d_temp[5];
    pd_mea[6] = d_temp[4];
    pd_mea[7] = -d_temp[3];
  }
  if (pz_history->u_v_axis_mode == 2) /* x-down 0,0,-1,0,1,0,1,0,0 */
  {
    pd_mea[2] = -d_temp[2];
    pd_mea[3] = d_temp[1];
    pd_mea[4] = d_temp[0];
    pd_mea[5] = -d_temp[5];
    pd_mea[6] = d_temp[4];
    pd_mea[7] = d_temp[3];
  }
  if (pz_history->u_v_axis_mode == 3) /* y-up 1,0,0,0,0,1,0,-1,0 */
  {
    pd_mea[2] = -d_temp[2];
    pd_mea[3] = d_temp[0];
    pd_mea[4] = -d_temp[1];
    pd_mea[5] = -d_temp[5];
    pd_mea[6] = d_temp[3];
    pd_mea[7] = -d_temp[4];
  }
  if (pz_history->u_v_axis_mode == 4) /* y-down 1,0,0,0,0,-1,0,1,0 */
  {
    pd_mea[2] = d_temp[0];
    pd_mea[3] = -d_temp[2];
    pd_mea[4] = d_temp[1];
    pd_mea[5] = d_temp[3];
    pd_mea[6] = -d_temp[5];
    pd_mea[7] = d_temp[4];
  }
  if (pz_history->u_v_axis_mode == 5) /* z-up 1,0,0,0,-1,0,0,0,-1 */
  {
    pd_mea[2] = d_temp[0];
    pd_mea[3] = -d_temp[1];
    pd_mea[4] = -d_temp[2];
    pd_mea[5] = d_temp[3];
    pd_mea[6] = -d_temp[4];
    pd_mea[7] = -d_temp[5];
  }
  memcpy(d_temp, pd_mea + 2, sizeof(d_temp));

  if (pz_history->u_h_axis_mode == 1)
  {
    pd_mea[2] = d_temp[0];
    pd_mea[3] = d_temp[1];
    pd_mea[4] = d_temp[2];
    pd_mea[5] = d_temp[3];
    pd_mea[6] = d_temp[4];
    pd_mea[7] = d_temp[5];
  }
  if (pz_history->u_h_axis_mode == 2)
  {
    pd_mea[2] = -d_temp[1];
    pd_mea[3] = d_temp[0];
    pd_mea[4] = d_temp[2];
    pd_mea[5] = -d_temp[4];
    pd_mea[6] = d_temp[3];
    pd_mea[7] = d_temp[5];
  }
  if (pz_history->u_h_axis_mode == 3)
  {
    pd_mea[2] = -d_temp[0];
    pd_mea[3] = -d_temp[1];
    pd_mea[4] = d_temp[2];
    pd_mea[5] = -d_temp[3];
    pd_mea[6] = -d_temp[4];
    pd_mea[7] = d_temp[5];
  }
  if (pz_history->u_h_axis_mode == 4)
  {
    pd_mea[2] = d_temp[1];
    pd_mea[3] = -d_temp[0];
    pd_mea[4] = d_temp[2];
    pd_mea[5] = d_temp[4];
    pd_mea[6] = -d_temp[3];
    pd_mea[7] = d_temp[5];
  }
}

void flx_detect_triaxis(double* pd_meas)
{
  FlxAxisDectct_t* pz_flx = &gz_axis_detect;

  flx_detect_vertical_axis(pd_meas, pz_flx);

  if (pz_flx->u_is_detected == INS_TRUE)
  {
    if (pz_flx->u_axis_flag == 0U)
    {
      if (pd_meas[0] > 0.0)
      {
        int imu_sampling_dt = 1000 / fusion_get_algconfig()->z_init_para.e_sampling_rate;
        int gnss_sampling_dt = 1000 / fusion_get_algconfig()->z_init_para.e_gnss_rate;
        double d_pps_gap = gnss_sampling_dt * 0.001;
        int s_val = (unsigned long long)(pd_meas[1] * 1000) % gnss_sampling_dt;
        /* if imut-lastimut > 0.1s, then data lost */
        if ((fabs(pz_flx->z_imumsg.timeTag - pd_meas[1]) > 0.1) && (pz_flx->z_imumsg.timeTag > 1.0))
        {
          pz_flx->u_dataloss_flag = 1;
        }
                
        if ((s_val < imu_sampling_dt || gnss_sampling_dt - s_val < imu_sampling_dt))
        {
          flx_prepare_imu_meas(pd_meas, d_pps_gap); /* Times Up, make a copy, filtering and wait for GNSS infor */
        }

        flx_accumulate_imu_meas(pd_meas); /* accumulate IMUMeas */
      }
      else if (pd_meas[0] < 0.0)
      {
        flx_prepare_gnss_meas(pd_meas); /* GNSS info ready */
      }

      double deltaT = pz_flx->z_gnssmsg.timeTag - pz_flx->z_imu_cpy.timeTag;
      if ((pz_flx->u_gready_flag != 0U) && (pz_flx->u_iready_flag != 0U) && (deltaT >= 0.0) && (deltaT < 0.8))
      {
        const IMUMsgBuf_t* pz_imubuf = &pz_flx->z_imubuf;
        const GNSSMsgBuf_t* pz_gnssbuf = &pz_flx->z_gnssbuf;

        flx_prapare_meas_buffer(); /* both ready, move to buffer */
        if ((pz_gnssbuf->fullFlag != 0U) && (pz_imubuf->fullFlag != 0U))
        {
          flx_ls_fit(pd_meas); /* LS FIT HERE */
          flx_calculate_axis_flag();
        }

        if ((pz_flx->u_axis_flag != 0U) && (pz_flx->u_misalign_flag == 0U))   /* Computer Misaligment */
        {
          flx_calculate_miseuler(pz_flx); /* cal misalignment angle */
        }
        pz_flx->u_gready_flag = 0;
        pz_flx->u_iready_flag = 0;
      }
    }
    else
    {
      if (pd_meas[0] > 0.0)
      {
        flx_convert_imu_axis(pd_meas); /* convert to right axis */
      }
    }
    gz_axis_result.AxisHeading = (int32_t)pz_flx->u_axis_flag;
  }
}
#endif

int32_t flx_set_axis_mode(int32_t mode)
{
  int32_t RetVal = 0;

  FlxAxisDectct_t* pz_flx_axis = fusion_get_flx_axis();

  if (mode > 0)
  {
    pz_flx_axis->z_vert_axis.u_last_axis_mode = mode / 10;
    pz_flx_axis->z_vert_axis.u_axis_mode = mode / 10;
#ifdef FlEX_AXIAL_DETECTION
    int32_t tmode = mode % 10;
    pz_flx_axis->u_axis_flag = (uint8_t)(tmode);

    if (pz_flx_axis->z_vert_axis.u_axis_mode > 0 && pz_flx_axis->z_vert_axis.u_axis_mode < 7 &&
        pz_flx_axis->u_axis_flag  > 0U && pz_flx_axis->u_axis_flag < 5U)
    {
      pz_flx_axis->u_is_detected = INS_TRUE;
      RetVal = 1;
    }
#else
    if (pz_flx_axis->z_vert_axis.u_axis_mode > 0 && pz_flx_axis->z_vert_axis.u_axis_mode < 7)
    {
      pz_flx_axis->u_is_detected = INS_TRUE;
      RetVal = 1;
    }
#endif
  }

  if (pz_flx_axis->u_is_detected == INS_FALSE)
  {
    pz_flx_axis->z_vert_axis.u_axis_mode = 6;
    pz_flx_axis->z_vert_axis.u_last_axis_mode = 255;
#ifdef FlEX_AXIAL_DETECTION
    pz_flx_axis->u_axis_flag = 0;
#endif
  }
  return RetVal;
}

void IMUAxisConv(float mea[])
{
  float temp[6] = { 0 };
  FlxAxisDectct_t* pz_flx_axis = fusion_get_flx_axis();

  for (int k = 0; k < 6; k++)
  {
    temp[k] = mea[2 + k];
  }
  if (1 == pz_flx_axis->z_vert_axis.u_axis_mode) /* x-up 0,0,1,0,1,0,-1,0,0 */
  {
    mea[2] = temp[2];
    mea[4] = -temp[0];
    mea[5] = temp[5];
    mea[7] = -temp[3];
  }
  else if (2 == pz_flx_axis->z_vert_axis.u_axis_mode) /* x-down 0,0,-1,0,1,0,1,0,0 */
  {
    mea[2] = -temp[2];
    mea[4] = temp[0];
    mea[5] = -temp[5];
    mea[7] = temp[3];
  }
  else if (3 == pz_flx_axis->z_vert_axis.u_axis_mode) /* y-up 1,0,0,0,0,1,0,-1,0 */
  {
    mea[2] = -temp[2];
    mea[3] = temp[0];
    mea[4] = -temp[1];
    mea[5] = -temp[5];
    mea[6] = temp[3];
    mea[7] = -temp[4];
  }
  else if (4 == pz_flx_axis->z_vert_axis.u_axis_mode) /* y-down 1,0,0,0,0,-1,0,1,0 */
  {
    mea[3] = -temp[2];
    mea[4] = temp[1];
    mea[6] = -temp[5];
    mea[7] = temp[4];
  }
  else
  {
    if (5 == pz_flx_axis->z_vert_axis.u_axis_mode) /* z-up 1,0,0,0,-1,0,0,0,-1 */
    {
      mea[2] = temp[0];
      mea[3] = -temp[1];
      mea[4] = -temp[2];
      mea[5] = temp[3];
      mea[6] = -temp[4];
      mea[7] = -temp[5];
    }
  }

  for (int k = 0; k < 6; k++)
  {
    temp[k] = mea[2 + k];
  }

  if (1 == gz_axis_result.AxisHeading)
  {
    mea[2] = temp[0];
    mea[3] = temp[1];
    mea[5] = temp[3];
    mea[6] = temp[4];
  }
  else if (2 == gz_axis_result.AxisHeading)
  {
    mea[2] = -temp[1];
    mea[3] = temp[0];
    mea[5] = -temp[4];
    mea[6] = temp[3];
  }
  else if (3 == gz_axis_result.AxisHeading)
  {
    mea[2] = -temp[0];
    mea[3] = -temp[1];
    mea[5] = -temp[3];
    mea[6] = -temp[4];
  }
  else
  {
    if (4 == gz_axis_result.AxisHeading)
    {
      mea[2] = temp[1];
      mea[3] = -temp[0];
      mea[5] = temp[4];
      mea[6] = -temp[3];
    }
  }
  return;
}

void flx_init_misAngle_by_history(HistoryInsResults_t* pz_history)
{
  NavParams_t* pz_nav = fusion_get_navpara();
  NavConfig_t* pz_navconfig = fusion_get_navconfig();
  float f_misAngle[3];
  f_misAngle[0] = (float)(pz_history->f_mis_roll * DEG2RAD);
  f_misAngle[1] = (float)(pz_history->f_mis_pitch * DEG2RAD);
  f_misAngle[2] = (float)(pz_history->f_mis_yaw * DEG2RAD);
  memcpy(pz_navconfig->f_init_align, f_misAngle, sizeof(pz_navconfig->f_init_align));
  memcpy(pz_nav->f_mis_align, f_misAngle, sizeof(pz_nav->f_mis_align));
  quat_euler2quat(pz_navconfig->f_init_align, pz_nav->f_quat_misalign);
  tf_euler2dcm(pz_navconfig->f_init_align, pz_nav->f_rotmat_misalign);
  math_matrix_transpose(*(pz_nav->f_rotmat_misalign), 3, 3, *(pz_nav->f_rotmat_v2b));
}
