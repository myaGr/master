/**@file        fusion_zupt.c
 * @brief		fusion zupt file
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

#include "fusion_zupt.h"
#include "fusion_math.h"
#include "fusion_global.h"
#include "fusion_err_model.h"
#include "fusion_quat.h"
#include "fusion_proc.h"
#include "fusion_wheel.h"
#include "fusion_transform.h"
#include "fusion_log.h"
#include "fusion_gnss.h"
#include "fusion_kf.h"
#include <string.h>
#include "mw_log.h"

static AutoZuptDetect_t gz_auto_zupt;

AutoZuptDetect_t* fusion_get_autozupt_para(void)
{
  return &gz_auto_zupt;
}

uint8_t zupt_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* uz_mdim, InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_TRUE;
  MechanNode_t* pz_node = pz_inav->pz_obs->pz_veh_node;
  SysErrModel_t* pz_model = pz_inav->pz_model;

#if 0
  * uz_mdim = 3;

  f_z_meas[0] = pz_node->f_vel[0] - 0.0;
  f_z_meas[1] = pz_node->f_vel[1] - 0.0;
  f_z_meas[2] = pz_node->f_vel[2] - 0.0;

  kf_set_update_type(SMT_ZUPT, pz_model);
  kf_update_measurement_matrix(SYS_DIM, *uz_mdim, pz_model);
  kf_update_measurement_noise(pd_meas, pz_model);
#else
  *uz_mdim = 4;
  NavParams_t* pz_nav = fusion_get_navpara();
  float f_prequat[4] = { 0.f };
  float f_rotvec[3] = { 0.f };
  quat_copy_f(f_prequat, pz_nav->f_prev_quat_b2n);
  quat_conjugate(f_prequat);
  quat_product_f(f_prequat, pz_nav->f_quat_b2n);
  quat_normalize_f(f_prequat);
  quat_quat2rotvct(f_prequat, f_rotvec);

  f_z_meas[0] = pz_node->f_vel[0] - 0.0f;
  f_z_meas[1] = pz_node->f_vel[1] - 0.0f;
  f_z_meas[2] = pz_node->f_vel[2] - 0.0f;
  f_z_meas[3] = -f_rotvec[2] - 0.0f;

  kf_set_update_type(SMT_ZUPTA, pz_model);
  kf_update_measurement_matrix(pz_model->u_sysdim, *uz_mdim, pz_model);
  kf_update_measurement_noise(pd_meas, pz_model);

  float f_lambda = kf_calcu_lambda(f_z_meas, pz_model->u_sysdim, *uz_mdim, pz_model);
  float f_ratio = 1.f;

  if (f_lambda > 30.f)
  {
    f_ratio = f_lambda;
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    pz_model->f_measnoise[i * pz_model->u_meadim+i] *= f_ratio;
  }

  LOGI(TAG_VDR, "[zupt:Update]:%.3lf,KFmeas:%.3f,%.3f,%.3f,%.3f,Lambda:%.3f,Std:%.3f,%.3f,%.3f,%.3lf\n", 
    pd_meas[1],f_z_meas[0], f_z_meas[1], f_z_meas[2], f_z_meas[3]*RAD2DEG, f_lambda,
    sqrtf(pz_model->f_measnoise[0]), sqrtf(pz_model->f_measnoise[pz_model->u_meadim+1]), 
    sqrtf(pz_model->f_measnoise[2 * pz_model->u_meadim + 2]), sqrt(pz_model->f_measnoise[3 * pz_model->u_meadim + 3] * RAD2DEG));
#endif
  return u_val;
}

void auto_zupt_detection(double pd_meas[], NavParams_t* pz_para)
{
  AutoZuptDetect_t* pz_zupt = &gz_auto_zupt;

  if (pz_zupt->u_zupt_inited == INS_FALSE)
  {
    pz_zupt->u_zupt_counter = 0;
    pz_zupt->u_imu_counter = 0;
    pz_zupt->u_val_counter = 1;
    pz_zupt->u_group_counter = 0;
    pz_zupt->d_imu_t = pd_meas[1];
    memset(pz_zupt->f_imu_buf,0,sizeof(pz_zupt->f_imu_buf));
    memset(pz_zupt->f_zupt_buf, 0, sizeof(pz_zupt->f_zupt_buf));
    memset(pz_zupt->f_zupt_val, 0, sizeof(pz_zupt->f_zupt_val));
    memset(pz_zupt->f_imu_group,0,sizeof(pz_zupt->f_imu_group));
    pz_zupt->u_zupt_inited = INS_TRUE;
    pz_zupt->u_zupt_valid = INS_FALSE;
  }

  if (pd_meas[0] > 0)
  {
    if (pz_zupt->u_imu_counter < ZUPT_IMU_BUF_SIZE)
    {
      for (uint8_t i = 0 ; i < 6; i++)
      {
        pz_zupt->f_imu_buf[i][pz_zupt->u_imu_counter] = (float)pd_meas[i + 2];
      }
      pz_zupt->u_imu_counter++;
    }
    else
    {
      float f_imu_mean[6],f_imu_std[6];

      for (uint8_t i = 0; i < 6; i++)
      {
        math_calculate_std(pz_zupt->f_imu_buf[i], ZUPT_IMU_BUF_SIZE, &f_imu_mean[i], &f_imu_std[i]);
      }

      for (uint8_t i = 0; i < 6; i++)
      {
        memmove(pz_zupt->f_imu_buf[i], pz_zupt->f_imu_buf[i]+1, (ZUPT_IMU_BUF_SIZE -1)*sizeof(float));
        pz_zupt->f_imu_buf[i][ZUPT_IMU_BUF_SIZE - 1] = (float)pd_meas[i + 2];
      }

      if (f_imu_std[0] < GYRO_MAX_STD && f_imu_std[1] < GYRO_MAX_STD && f_imu_std[2] < GYRO_MAX_STD &&
          f_imu_std[3] < ACCL_MAX_STD && f_imu_std[4] < ACCL_MAX_STD && f_imu_std[5] < 1.5*ACCL_MAX_STD)
      {
        float f_dt = (float)(pd_meas[1] - pz_zupt->d_imu_t);

        if (f_dt < ZUPT_INIT_TIME)
        {
          for (uint8_t i = 0; i < 6; i++)
          {
            pz_zupt->f_zupt_val[i] = ((float)(pz_zupt->u_val_counter - 1) * pz_zupt->f_zupt_val[i] + f_imu_std[i]) / (float)pz_zupt->u_val_counter;
          }
          pz_zupt->u_val_counter++;
        }
        else
        {
          float f_zupt_mean[6], f_zupt_std[6];
          float f_threshold,f_thdiff,f_scale_df;
          if (pz_zupt->u_zupt_counter < ZUPT_STD_BUF_SIZE)
          {
            for (uint8_t i = 0; i < 6; i++)
            {
              pz_zupt->f_zupt_buf[i][pz_zupt->u_zupt_counter] = f_imu_std[i];
            }
            pz_zupt->u_zupt_counter++;
          }
          else
          {
            for (uint8_t i = 0; i < 6; i++)
            {
              memmove(pz_zupt->f_zupt_buf[i], pz_zupt->f_zupt_buf[i] + 1, (ZUPT_STD_BUF_SIZE - 1) * sizeof(float));
              pz_zupt->f_zupt_buf[i][ZUPT_STD_BUF_SIZE - 1] = f_imu_std[i];
            }
          }
          for (uint8_t i = 0; i < 6; i++)
          {
            math_calculate_std(pz_zupt->f_zupt_buf[i], pz_zupt->u_zupt_counter, &f_zupt_mean[i], &f_zupt_std[i]);
            f_threshold = ZUPT_THRES_SIGMA * f_zupt_std[i];

            f_scale_df = f_zupt_mean[i] * ZUPT_DIFF_LIMIT_L;
            f_thdiff = (f_threshold > f_scale_df) ? f_threshold : f_scale_df;

            if (pz_zupt->u_zupt_counter < (uint8_t)ZUPT_INIT_TIME)
            {
              pz_zupt->f_zupt_threshold[i] = pz_zupt->f_zupt_val[i] + f_thdiff;
            }
            else
            {
              pz_zupt->f_zupt_threshold[i] = f_zupt_mean[i] + f_thdiff;
            }
          }
          pz_zupt->u_zupt_valid = INS_TRUE;
          pz_zupt->d_imu_t = pd_meas[1];
          pz_zupt->u_val_counter = 1;
        }
      }
      else
      {
        pz_zupt->d_imu_t = pd_meas[1];
      }
      /* Pass Zupt Init */
      if (pz_zupt->u_zupt_valid == INS_TRUE)
      {
        if (fabs(pd_meas[9] + pd_meas[10] + pd_meas[11] + pd_meas[12]) > 0)
        {
          pz_zupt->s_zupt_indicator = -1;
        }
        else
        {
          uint8_t u_zupt_flag = 0;
          float f_group_mean[6] = { 0.f }, f_group_std[6] = { 0.f };

          for (uint8_t i = 0; i < 6; i++)
          {
            if (f_imu_std[i] > pz_zupt->f_zupt_threshold[i])
            {
              pz_zupt->s_zupt_indicator = -1;

              if (f_imu_std[i] > 1.5f* pz_zupt->f_zupt_threshold[i])
              {
                u_zupt_flag = 1;
                break;
              }
            }
          }

          if (pz_zupt->u_group_counter < ZUPT_GROUP_SIZE)
          {
            for (uint8_t i = 0; i < 6; i++)
            {
              pz_zupt->f_imu_group[i][pz_zupt->u_group_counter] = f_imu_mean[i];
            }
            pz_zupt->u_group_counter++;
          }
          else
          {
            for (uint8_t i = 0; i < 6; i++)
            {
              math_calculate_std(pz_zupt->f_imu_group[i], pz_zupt->u_group_counter, &f_group_mean[i], &f_group_std[i]);

              memmove(pz_zupt->f_imu_group[i], pz_zupt->f_imu_group[i]+1, (ZUPT_GROUP_SIZE - 1) * sizeof(float));
              pz_zupt->f_imu_group[i][ZUPT_GROUP_SIZE - 1] = f_imu_mean[i];
            }
            if ((f_group_std[0] < 0.0007f) && (f_group_std[1] < 0.0007f) && (f_group_std[2] < 0.0007f)
                && (u_zupt_flag == 1U) && (pz_zupt->s_zupt_indicator == -1))
            {
              pz_zupt->s_zupt_indicator = 2;
            }
            else if (f_group_std[3] > 0.12f)
            {
              pz_zupt->s_zupt_indicator = -1;
            }
            pz_zupt->f_norm_std = sqrtf(SQR(f_group_std[3]) + SQR(f_group_std[4]));
          }
          if (u_zupt_flag == 0 && fusion_get_navconfig()->u_whspd_enable > 0)
          {
            pz_zupt->s_zupt_indicator = 2;
          }
        }
        /* add 127 to avoid overflow */
        if (pz_zupt->s_zupt_indicator >= 0 && pz_zupt->s_zupt_indicator < 127)
        {
          pz_zupt->s_zupt_indicator = pz_zupt->s_zupt_indicator + (int8_t)1;
        }
        else
        {
          pz_zupt->s_zupt_indicator = 0;
        }    
        pz_zupt->s_zupt_indicator = pz_zupt->s_zupt_indicator > 2 ? 2 : pz_zupt->s_zupt_indicator;
      }
      pz_zupt->u_imu_counter = 0;
    }
  }
  if (pz_zupt->s_zupt_indicator >= 2)
  {
    GNSSFormMeas_t* pz_gnss = gnss_get_form_meas();

    if (fabs(pd_meas[1] - pz_gnss->d_gnss_meas[1]) < 1.0 && *((uint8_t*)(&pz_gnss->d_gnss_meas[14])) > 0 && pz_gnss->d_gnss_meas[17] > 1.0)
    {
      pz_zupt->s_zupt_indicator = -1;
    }
    if (pz_zupt->f_norm_std > 0.05)
    {
      pz_zupt->s_zupt_indicator = -1;
    }
  }
  pz_para->u_izupt_counter = pz_zupt->s_zupt_indicator < 0 ? 0 : pz_zupt->s_zupt_indicator;    
}