/**@file        fusion_fault_detection.c
 * @brief		fault detection file
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

#include "fusion_fault_detection.h"
#include "fusion_log.h"
#include "fusion_math.h"
#include <string.h>
#include "mw_log.h"

#define IMU_DETECT_BUF_SIZE 200 /* 1s */
#define WHEEL_DETECT_BUF_SIZE 20*3 /* 3s */
#define WHEEL_SLIP_DETECT_DV 3.0f
#define WHEEL_DIFF_DETECT_DV 2.0f

void fusion_fault_detection_handler(AsensingIMU_t* pz_imu, GNSSFormMeas_t* pz_form_meas, WheelSpd_t* pz_whsig, INSResults_t* pz_ins)
{
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();
  NavParams_t* pz_nav = fusion_get_navpara();

  /* 1.time stamp detection */
  static double d_lasttime = -999.9;
  double d_imu_dt = (d_lasttime > 0.0) ? pz_imu->t_timestamp / 1000.0 - d_lasttime / 1000.0 : pz_algconfig->f_sampling_dt;
  if (d_imu_dt > 10.0 || d_imu_dt < -1.0)
  {
    ins_hardreset();
    LOGE(TAG_VDR, "ins hardreset, time gap abnormal, %.3f, %.3f, dt=%.3f\n", pz_imu->t_timestamp / 1000.0, d_lasttime / 1000.0, 
      pz_imu->t_timestamp / 1000.0-d_lasttime / 1000.0);
  }
  else if (d_imu_dt > 1.5)
  {
    ins_softreset();
    LOGE(TAG_VDR, "ins softreset, time gap abnormal, %.3f, %.3f, dt=%.3f\n", pz_imu->t_timestamp / 1000.0, d_lasttime / 1000.0,
      pz_imu->t_timestamp / 1000.0 - d_lasttime / 1000.0);
  }
  else if (d_imu_dt > 0.15)
  {
    pz_nav->u_imu_shortloss_flag = 1u;
    pz_nav->d_imu_shortloss_time = pz_imu->t_timestamp / 1000.0;
    LOGE(TAG_VDR, "imu time gap abnormal, dt=%.3f, %.3f, %.3f, dt=%.3f\n", d_imu_dt, pz_imu->t_timestamp / 1000.0, d_lasttime / 1000.0,
      pz_imu->t_timestamp / 1000.0 - d_lasttime / 1000.0);
  }

  /* 2.outlier detection */
  float f_accvalue = sqrtf((pz_imu->f_accl[0] * pz_imu->f_accl[0]) + (pz_imu->f_accl[1] * pz_imu->f_accl[1]) + (pz_imu->f_accl[2] * pz_imu->f_accl[2]));
  if (fabsf(pz_imu->f_gyro[0]) > 100.0 * DEG2RAD || fabsf(pz_imu->f_gyro[1]) > 100.0 * DEG2RAD || fabsf(pz_imu->f_gyro[2]) > 100.0 * DEG2RAD ||
    f_accvalue - 9.8 > 20.0)
  {
    ins_hardreset();
    LOGE(TAG_VDR, "ins hardreset, imu outlier detected, %.3f \n", pz_imu->t_timestamp / 1000.0);
  }

  /* 3.constant value detection */
  static float f_gyro_buf[3][IMU_DETECT_BUF_SIZE];
  float f_gyro_mean[3], f_gyro_std[3];
  static float f_accl_buf[3][IMU_DETECT_BUF_SIZE];
  float f_accl_mean[3], f_accl_std[3];
  float f_gyro_incre[3] = { 0.0 };
  static uint8_t u_detect_counter = 0;
  if (u_detect_counter < IMU_DETECT_BUF_SIZE)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      f_gyro_buf[i][u_detect_counter] = pz_imu->f_gyro[i];
      f_accl_buf[i][u_detect_counter] = pz_imu->f_accl[i];
    }
    u_detect_counter++;
  }
  else
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      memmove(f_gyro_buf[i], f_gyro_buf[i] + 1, (IMU_DETECT_BUF_SIZE - 1) * sizeof(float));
      f_gyro_buf[i][IMU_DETECT_BUF_SIZE - 1] = pz_imu->f_gyro[i];

      memmove(f_accl_buf[i], f_accl_buf[i] + 1, (IMU_DETECT_BUF_SIZE - 1) * sizeof(float));
      f_accl_buf[i][IMU_DETECT_BUF_SIZE - 1] = pz_imu->f_accl[i];
    }
    for (uint8_t i = 0; i < 3; i++)
    {
      math_calculate_std(f_gyro_buf[i], IMU_DETECT_BUF_SIZE, &f_gyro_mean[i], &f_gyro_std[i]);
      math_calculate_std(f_accl_buf[i], IMU_DETECT_BUF_SIZE, &f_accl_mean[i], &f_accl_std[i]);

      for (uint32_t j = 0; j < IMU_DETECT_BUF_SIZE; j++)
      {
        f_gyro_incre[i] += (float)(f_gyro_buf[i][j] * d_imu_dt);
      }
    }
    if ((f_gyro_std[0] < INFINITY_MIN) || (f_gyro_std[1] < INFINITY_MIN) || (f_gyro_std[2] < INFINITY_MIN) ||
      (f_accl_std[0] < INFINITY_MIN) || (f_accl_std[1] < INFINITY_MIN) || (f_accl_std[2] < INFINITY_MIN))
    {
      ins_softreset();
      LOGE(TAG_VDR, "ins softreset, imu same for 1 sec, %.3f \n", pz_imu->t_timestamp / 1000.0);
    }

    /* 4.imu move detection */
    /*
    if (((f_gyro_std[0] > 25.0) || (f_gyro_std[1] > 25.0) || (f_gyro_std[2] > 25.0)) &&
      (fabs(f_gyro_incre[0]) > 15.0 || fabs(f_gyro_incre[1]) > 15.0 || fabs(f_gyro_incre[2]) > 15.0))
    {
      ins_hardreset();
      LOGE(TAG_VDR, "ins hardreset, imu move detected, %.3f, gyro std:%.3f,%.3f,%.3f, gyro incre:%.3f,%.3f,%.3f\n", pz_imu->t_timestamp / 1000.0,
        f_gyro_std[0], f_gyro_std[1], f_gyro_std[2], f_gyro_incre[0], f_gyro_incre[1], f_gyro_incre[2]);
    }
    if ((fabs(f_gyro_incre[0]) > 75.0 || fabs(f_gyro_incre[1]) > 75.0 || fabs(f_gyro_incre[2]) > 75.0))
    {
      ins_hardreset();
      LOGE(TAG_VDR, "ins hardreset, imu move detected2, %.3f, gyro incre:%.3f,%.3f,%.3f\n", pz_imu->t_timestamp / 1000.0,
        f_gyro_incre[0], f_gyro_incre[1], f_gyro_incre[2]);
    }
    */
  }

  /* 5.bias detection */
  if (fabs(pz_ins->f_gyrobias_x) > 10.0 || fabs(pz_ins->f_gyrobias_y) > 10.0 || fabs(pz_ins->f_gyrobias_z) > 10.0 ||
    fabs(pz_ins->f_accbias_x) > 10.0 || fabs(pz_ins->f_accbias_y) > 10.0 || fabs(pz_ins->f_accbias_z) > 10.0)
  {
    ins_hardreset();
    LOGE(TAG_VDR, "ins hardreset, bias is abnormal, %.3f \n", pz_imu->t_timestamp / 1000.0);
  }

  /* 6.misalignment angle detection */
  if (pz_ins->u_misb2v_convergeflag == 1)
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      if (fabsf(pz_nav->f_mis_align[0] - pz_ins->f_misb2v_converge[0]) > 10.0f * DEG2RAD ||
        fabsf(pz_nav->f_mis_align[1] - pz_ins->f_misb2v_converge[1]) > 10.0f * DEG2RAD ||
        fabsf(pz_nav->f_mis_align[2] - pz_ins->f_misb2v_converge[2]) > 15.0f * DEG2RAD)
      {
        ins_hardreset();
        LOGE(TAG_VDR, "ins hardreset, imu misalignment change detected, %.3f \n", pz_imu->t_timestamp / 1000.0);
      }
    }
  }	
  /* 7.wheel speed detection */
  /* 7.1 wheel slipping detection */
  NavConfig_t* pz_config = fusion_get_navconfig();
  if (pz_whsig->d_timestamp > 0.0)
  {
    if (fabs(pz_whsig->f_vehspd_fl - pz_whsig->f_vehspd_fr < WHEEL_SLIP_DETECT_DV) &&
      fabs(pz_whsig->f_vehspd_fl - pz_whsig->f_vehspd_rl < WHEEL_SLIP_DETECT_DV) &&
      fabs(pz_whsig->f_vehspd_fl - pz_whsig->f_vehspd_rr < WHEEL_SLIP_DETECT_DV) && pz_whsig->d_timestamp > 0.0)
    {
      pz_config->u_whspd_enable = TYPE_WHEEL_AIDING_TWO_REARS;/* default use two wheels */
    }
    else /* wheel slipping, disable wheel meas */
    {
      pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_TWO_REARS;
      pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_FOUR_WHLS;
      LOGE(TAG_VDR, "wheel slipping, disable wheel meas, %.3f \n", pz_imu->t_timestamp / 1000.0);
    }
    /* 7.2 wheel speed abnormal difference detection */
    if ((pz_whsig->f_vehspd_fl - pz_whsig->f_vehspd_fr > WHEEL_DIFF_DETECT_DV) ||
      (pz_whsig->f_vehspd_rl - pz_whsig->f_vehspd_rr > WHEEL_DIFF_DETECT_DV))
    {
      pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_TWO_REARS;
      pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_FOUR_WHLS;
      LOGE(TAG_VDR, "wheel speed difference abnormal, disable wheel meas, %.3f \n", pz_imu->t_timestamp / 1000.0);
    }
    /* 7.3 constant wheel speed detection */
    static float f_wheelspeed_buf[4][WHEEL_DETECT_BUF_SIZE];
    float f_wheelspeed_mean[4], f_wheelspeed_std[4];
    static uint8_t u_detect_counter2 = 0;
    if (u_detect_counter2 < WHEEL_DETECT_BUF_SIZE)
    {
      f_wheelspeed_buf[0][u_detect_counter2] = pz_whsig->f_vehspd_fl;
      f_wheelspeed_buf[1][u_detect_counter2] = pz_whsig->f_vehspd_fr;
      f_wheelspeed_buf[2][u_detect_counter2] = pz_whsig->f_vehspd_rl;
      f_wheelspeed_buf[3][u_detect_counter2] = pz_whsig->f_vehspd_rr;
      u_detect_counter2++;
    }
    else
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        memmove(f_wheelspeed_buf[i], f_wheelspeed_buf[i] + 1, (WHEEL_DETECT_BUF_SIZE - 1) * sizeof(float));
      }
      f_wheelspeed_buf[0][WHEEL_DETECT_BUF_SIZE - 1] = pz_whsig->f_vehspd_fl;
      f_wheelspeed_buf[1][WHEEL_DETECT_BUF_SIZE - 1] = pz_whsig->f_vehspd_fr;
      f_wheelspeed_buf[2][WHEEL_DETECT_BUF_SIZE - 1] = pz_whsig->f_vehspd_rl;
      f_wheelspeed_buf[3][WHEEL_DETECT_BUF_SIZE - 1] = pz_whsig->f_vehspd_rr;

      for (uint8_t i = 0; i < 4; i++)
      {
        math_calculate_std(f_wheelspeed_buf[i], WHEEL_DETECT_BUF_SIZE, &f_wheelspeed_mean[i], &f_wheelspeed_std[i]);
      }
      double d_gnssvel_hor = sqrt(pz_form_meas->d_gnss_meas[5] * pz_form_meas->d_gnss_meas[5] + pz_form_meas->d_gnss_meas[6] * pz_form_meas->d_gnss_meas[6]);
      if (((f_wheelspeed_std[2] < INFINITY_MIN) || (f_wheelspeed_std[3] < INFINITY_MIN)) &&
        (f_wheelspeed_mean[0] - d_gnssvel_hor) > 3.0) /* to be improved... */
      {
        pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_TWO_REARS;
        pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_FOUR_WHLS;
        LOGI(TAG_VDR, "wheel speed same for 3 sec, disable wheel meas, %.3f \n", pz_imu->t_timestamp / 1000.0);
      }
    }
    /* 7.4 wheel speed time error detection */
    if (fabs(pz_whsig->d_timestamp - pz_inav->d_last_imu_time) >= 0.1 && pz_inav->d_last_imu_time > 0.0)
    {
      pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_TWO_REARS;
      pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_FOUR_WHLS;
      // LOGE(TAG_VDR, "wheelspeed timestamp error, disable wheel meas, %.3f, %.3f, %.3f \n", pz_whsig->d_timestamp, pz_imu->t_timestamp / 1000.0, pz_inav->d_last_imu_time);
    }
  }
  else
  {
    pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_TWO_REARS;
    pz_config->u_whspd_enable &= ~TYPE_WHEEL_AIDING_FOUR_WHLS;
  }
  d_lasttime = (double)pz_imu->t_timestamp;
}