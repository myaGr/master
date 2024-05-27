/**@file        fusion_meas_qc.c
 * @brief		measurements quality control file
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

#include "fusion_api.h"
#include "fusion_log.h"
#include "fusion_global.h"
#include "fusion_proc.h"
#include "fusion_kf.h"
#include "fusion_if.h"
#include "fusion_meas_qc.h"
#include "fusion_config.h"
#include "fusion_if.h"
#include "fusion_integrity.h"
#include "fusion_math.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "mw_log.h"

#define TIMELEN_INS_PRIOR 400.0
#define GNSS_NOISE_SCALE_S 20.0
#define GNSS_NOISE_SCALE_E 2.0

static GNSSQCPara_t gz_qc_para;

GNSSQCPara_t* fusion_get_qc_para(void)
{
  return &gz_qc_para;
}

int8_t qc_check_meas_pos(double pd_meas[],float f_kfmeas[3], GNSSQCPara_t* pz_qc, SysErrModel_t* sysmodel)
{
  int8_t s_pos_pass = 1;
  float f_lambda_pos_thre1 = pz_qc->f_lambda_pos_thre1;
  float f_lambda_pos_thre2 = pz_qc->f_lambda_pos_thre2;

  float posStd = (float)sqrt(pd_meas[8] + pd_meas[9]);
  uint8_t u_bds_status = *((uint8_t*)(&pd_meas[14]));

  /* turn up bad gnss noise within 400s after good gnss */
#if 1
  NavParams_t* pz_nav = fusion_get_navpara();
  static double d_zupt_timelen = 0.0;
  if (pz_qc->d_last_check_time >= 0.0)
  {
    d_zupt_timelen += (pd_meas[1] - pz_qc->d_last_check_time - 1.0 / pz_qc->u_bds_rate);
    d_zupt_timelen = (d_zupt_timelen > 0.0) ? d_zupt_timelen : 0.0;
  }    

  double d_timelen_badgnss = pd_meas[1] - pz_qc->d_pre_good_bds_time - d_zupt_timelen; /* remove zupt time. outage? */
  static double d_last_pre_good_bds_time = -1.0;

  if ((double_equal(pz_qc->d_pre_good_bds_time, d_last_pre_good_bds_time) == -1) && 
    (pz_qc->d_pre_good_bds_time - pz_nav->d_imu_shortloss_time > 3.0))
  {
    pz_nav->u_imu_shortloss_flag = 0u;
    LOGI(TAG_VDR, "[gnss:QC_PosMeaCheck]:new pre_good_bds_time comes, pre_good_bds_time:%.3f, last pre_good_bds_time:%.3f\n",
      pz_qc->d_pre_good_bds_time, d_last_pre_good_bds_time);
  }

  if (double_equal(pz_qc->d_pre_good_bds_time, d_last_pre_good_bds_time) == -1)
  {
    d_zupt_timelen = 0.0; /* reset as 0 */
  }

  if (d_timelen_badgnss > 2.0/pz_qc->u_bds_rate && d_timelen_badgnss < TIMELEN_INS_PRIOR && u_bds_status != TYPE_POS_FLAG_RTK_FIX
    && pz_qc->d_pre_good_bds_time >= 0.0 && (pz_nav->u_imu_shortloss_flag != 1u)) /* for imu lost case */
  {      
    double d_scale = (GNSS_NOISE_SCALE_E - GNSS_NOISE_SCALE_S) / TIMELEN_INS_PRIOR * d_timelen_badgnss + GNSS_NOISE_SCALE_S;
    pd_meas[8] *= SQR(d_scale);
    pd_meas[9] *= SQR(d_scale);
    pd_meas[10] *= SQR(d_scale);

    LOGI(TAG_VDR, "[gnss:QC_PosMeaCheck]:%.3lf, pre_good_bds_time:%.3f, badgnss_timelen:%.3f, std enlarge scale:%.3f, zupt/outage timelen:%.3f\n",
      pd_meas[1], pz_qc->d_pre_good_bds_time, d_timelen_badgnss, d_scale, d_zupt_timelen);
    d_last_pre_good_bds_time = pz_qc->d_pre_good_bds_time;
  }
#endif

  kf_set_update_type(SMT_GPSPOS, sysmodel);
  kf_update_measurement_matrix(sysmodel->u_sysdim,3, sysmodel);
  kf_update_measurement_noise(pd_meas, sysmodel);
  pz_qc->f_pos_lambda = kf_calcu_lambda(f_kfmeas, sysmodel->u_sysdim, 3, sysmodel);

#ifdef OPEN_INS_INTEGRITY             
  EkfPosFeedback_t* pz_ekf_dpos = fusion_get_ekf_dpos();
  pz_ekf_dpos->f_pos_lambda = pz_qc->f_pos_lambda;
#endif

  LOGI(TAG_VDR, "[gnss:QC_PosMeaCheck]:%.3lf, posflag: %d,lambdapos = %.3f,speed:%.3f,Satnum:%.3f,AvgCN0:%.3f,rtk_check:%d,PosThresh:%.3f,%.3f\n",
      pd_meas[1], u_bds_status, pz_qc->f_pos_lambda,pd_meas[17],pd_meas[18], pd_meas[19], pz_qc->u_rtk_check_flag,
      f_lambda_pos_thre1,f_lambda_pos_thre2);    

  if (pz_qc->f_pos_lambda > f_lambda_pos_thre1 && pz_qc->u_rtk_check_flag == 1)
  {
    if (pz_qc->f_pos_lambda > f_lambda_pos_thre2)
    {
#if 0
      /* Fail, give up update*/
      s_pos_pass = 0;
#else
      /* adjust measurement noise */
      float f_scale = pz_qc->f_pos_lambda / f_lambda_pos_thre1;
          
      if (u_bds_status != TYPE_POS_FLAG_RTK_FIX)
      {
        f_scale *= 25.0;
      }
      pd_meas[8] *= f_scale;
      pd_meas[9] *= f_scale;
      pd_meas[10] *= f_scale;
      s_pos_pass = 3;
      LOGI(TAG_VDR, "[gnss:MeaNoise]:%.3lf,Std enlarge factor:%.3f\n", pd_meas[1], sqrt(f_scale));
#endif
    }
    else
    {
      /* adjust measurement noise */
      float f_scale = pz_qc->f_pos_lambda / f_lambda_pos_thre1;

      if (u_bds_status != TYPE_POS_FLAG_RTK_FIX)
      {
        f_scale *= 9.0;
      }
      pd_meas[8] *= f_scale;
      pd_meas[9] *= f_scale;
      pd_meas[10] *= f_scale;
      s_pos_pass = 2;
      LOGI(TAG_VDR, "[gnss:MeaNoise]:%.3lf,Std enlarge factor:%.3f\n", pd_meas[1], sqrt(f_scale));
    }
  }
  return s_pos_pass;
}

int8_t qc_check_meas_vel(double pd_meas[], const float f_kfmeas[6], GNSSQCPara_t* pz_qc,SysErrModel_t* sysmodel)
{
  int8_t s_vel_pass = 1;
  uint8_t u_bds_status = *((uint8_t*)(&pd_meas[14]));
  float f_meas_temp[3] = { 0.0f };
  f_meas_temp[0] = f_kfmeas[3];
  f_meas_temp[1] = f_kfmeas[4];
  f_meas_temp[2] = f_kfmeas[5];

  kf_set_update_type(SMT_GPSVEL, sysmodel);
  kf_update_measurement_matrix(sysmodel->u_sysdim, 3, sysmodel);
  kf_update_measurement_noise(pd_meas, sysmodel);
  pz_qc->f_vel_lambda = kf_calcu_lambda(f_meas_temp, sysmodel->u_sysdim, 3, sysmodel);

  LOGI(TAG_VDR, "[QC_VelMeaCheck]:%.3lf, lambdavel = %f\n", pd_meas[1], pz_qc->f_vel_lambda);

  if (pz_qc->f_vel_lambda > pz_qc->f_lambda_vel_thre1)
  {
    if (pz_qc->f_vel_lambda > pz_qc->f_lambda_vel_thre2)
    {
      /* reject vel update */
      s_vel_pass = 0;
    }
    else
    {
      s_vel_pass = 2;
      /* adjust vel measurement noise */
      float f_scale = pz_qc->f_vel_lambda / pz_qc->f_lambda_vel_thre1;
      pd_meas[11] *= f_scale;
      pd_meas[12] *= f_scale;
      pd_meas[13] *= f_scale;
    }
  }
  return s_vel_pass;
}

void qc_initial(NavConfig_t* pz_navconfig)
{
  gz_qc_para.f_lambda_pos_thre1 = pz_navconfig->f_lambda_pos1;
  gz_qc_para.f_lambda_pos_thre2 = pz_navconfig->f_lambda_pos2;
  gz_qc_para.f_lambda_vel_thre1 = pz_navconfig->f_lambda_vel1;
  gz_qc_para.f_lambda_vel_thre2 = pz_navconfig->f_lambda_vel2;
  gz_qc_para.u_bds_rate = fusion_get_algconfig()->z_init_para.e_gnss_rate;
  gz_qc_para.u_rtk_check_flag = 0;
  gz_qc_para.f_gbias_check_thre = (float)(QC_BIAS_THRE * DEG2RAD);
  gz_qc_para.q_bds_reject_thre = 0U;
  gz_qc_para.q_bds_converged_thre = QC_CONVER_THRE;
  gz_qc_para.d_last_lowspd_time = INITIAL_TIME;
  gz_qc_para.q_bds_valid_counter = 0U;
    
  gz_qc_para.q_bds_reject_counter = 0; /* reject GNSS count */
  gz_qc_para.d_last_check_time = INITIAL_TIME;  /* the last time come in BDS_QC */
  gz_qc_para.f_gbias_counter = 0;
  gz_qc_para.u_adanoise_inited = INS_FALSE;
  gz_qc_para.q_cont_rtk_thres = 2 * gz_qc_para.u_bds_rate;
  gz_qc_para.q_cont_goodbds_thres = 10 * gz_qc_para.u_bds_rate;
  memset(gz_qc_para.f_gzbias_buffer, 0, sizeof(gz_qc_para.f_gzbias_buffer));
}

void qc_check_bds_number(double pd_mea[], GNSSQCPara_t* pz_qc)
{
  if ((pd_mea[1] - pz_qc->d_last_check_time) < (double)QC_NO_OUTAGE)
  {
    pz_qc->q_bds_valid_counter++;

    if (pz_qc->q_bds_valid_counter > QC_VALID_NUM)
    {
      pz_qc->q_bds_valid_counter = QC_VALID_NUM;
    }
  }
  else
  {
    if ((pd_mea[1] - pz_qc->d_last_check_time) > (double)QC_BDS_OUTAGE)
    {
      pz_qc->q_bds_valid_counter = 0;
      pz_qc->q_bds_reject_counter = 0;
      pz_qc->u_adanoise_inited = INS_FALSE;
    }
  }
}

void qc_adjust_reject_thre(double pd_mea[],GNSSQCPara_t* pz_qc, NavConfig_t* pz_navconfig)
{
  pz_qc->q_bds_reject_thre = ((pz_navconfig->u_whspd_enable != 0)) ? (QC_REJECT_NUM * 20U) : QC_REJECT_NUM;

  if (pd_mea[17] < 2.78)
  {
    pz_qc->d_last_lowspd_time = pd_mea[1];
    pz_qc->q_last_bds_reject_num = pz_qc->q_bds_reject_counter;
  }

  if ((pd_mea[1] - pz_qc->d_last_lowspd_time) < 20.0)
  {
    pz_qc->q_bds_reject_thre = pz_qc->q_last_bds_reject_num + 20U * pz_qc->u_bds_rate;
  }

  if (pd_mea[1] - pz_qc->d_pre_good_bds_time < 120.0 + pz_qc->d_bds_zupt_time_all && pz_qc->d_pre_good_bds_time >= 0.0)
  {
    pz_qc->q_bds_reject_thre = pz_qc->q_bds_reject_counter + 1;
  }
}

void qc_check_converged_gbias(GNSSQCPara_t* pz_qc)
{
  float f_gbias_max = MIN_GYRO_BIAS;
  float f_gbias_min = MAX_GYRO_BIAS;
  uint32_t i = 0;
  uint32_t j = 0;
  static double d_last_check_time = -999.99;

  NavParams_t* pz_para = fusion_get_navpara();

  /* check gyro if convergence */
  if (pz_qc->f_gbias_counter < pz_qc->q_bds_converged_thre)
  {
    if (pz_para->d_sys_timestamp - d_last_check_time > 0.9)
    {
      pz_qc->f_gzbias_buffer[pz_qc->f_gbias_counter] = pz_para->f_gyrobias[2];
      pz_qc->f_gbias_counter++;

      pz_qc->u_gbias_converged = INS_FALSE;
      d_last_check_time = pz_para->d_sys_timestamp;
    }
  }
  else
  {
    for (i = 0; i < (pz_qc->q_bds_converged_thre - 1U); i++)
    {
      pz_qc->f_gzbias_buffer[i] = pz_qc->f_gzbias_buffer[i + 1U];

      if (pz_qc->f_gzbias_buffer[i] > f_gbias_max)
      {
        f_gbias_max = pz_qc->f_gzbias_buffer[i];
      }

      if (pz_qc->f_gzbias_buffer[i] < f_gbias_min)
      {
        f_gbias_min = pz_qc->f_gzbias_buffer[i];
      }
    }

    pz_qc->f_gzbias_buffer[pz_qc->q_bds_converged_thre - 1U] = pz_para->f_gyrobias[2];

    if (pz_para->f_gyrobias[2] > f_gbias_max)
    {
      f_gbias_max = pz_para->f_gyrobias[2];
    }

    if (pz_para->f_gyrobias[2] < f_gbias_min)
    {
      f_gbias_min = pz_para->f_gyrobias[2];
    }

    if ((f_gbias_max - f_gbias_min) < pz_qc->f_gbias_check_thre)
    {
      pz_qc->u_gbias_converged = INS_TRUE;
    }
    else
    {
      pz_qc->u_gbias_converged = INS_FALSE;
      LOGI(TAG_VDR, "gbias not converged,%.3lf, gbias_diff:%.4lf\n", pz_para->d_sys_timestamp, ((double)f_gbias_max - f_gbias_min)*RAD2DEG);
    }
  }
}

void qc_adjust_rtk_check(double pd_mea[],InertialNav_t* pz_inav, NavConfig_t* pz_navconfig, GNSSQCPara_t* pz_qc)
{
  float f_lambda_pos_thre1 = pz_navconfig->f_lambda_pos1;

  float f_lambda_pos_thre2 = pz_navconfig->f_lambda_pos2;

  uint8_t u_bds_status = *((uint8_t*)(&pd_mea[14]));

  float f_gnssstd = (float)sqrt(pd_mea[8] + pd_mea[9]);

  pz_qc->u_rtk_check_flag = 0;

  if ((pz_navconfig->u_whspd_enable == 0 && fusion_get_algconfig()->z_init_para.u_whspd_mode == TYPE_WHEEL_AIDING_NHC) ||
    (pz_navconfig->u_whspd_enable == 0 && fusion_get_algconfig()->z_init_para.u_whspd_mode > TYPE_WHEEL_AIDING_NHC && is_izupt() != 1))
  {
    if ((u_bds_status != TYPE_POS_FLAG_RTK_FIX) ||
        ((u_bds_status == TYPE_POS_FLAG_RTK_FIX) && (f_gnssstd > 0.8)))
    {
      pz_qc->u_rtk_check_flag = 1; /* to be improved */

      LOGI(TAG_VDR, "[gnss:rtk_check:]:whspd invalid, check lamda,%.3lf,%d,%.3f\n",
        pd_mea[1], u_bds_status, f_gnssstd);
    }
    else
    {
      LOGI(TAG_VDR, "[gnss:rtk_check:]:whspd invalid,%.3lf,%d,%.3f\n",
        pd_mea[1], u_bds_status, f_gnssstd);
    }
  }
  else
  {
    NavParams_t* pz_nav = fusion_get_navpara();
    if (pz_nav->u_imu_shortloss_flag == 1u && pd_mea[1] - pz_nav->d_imu_shortloss_time < 5.0 + pz_qc->d_bds_zupt_time_all
      && u_bds_status == TYPE_POS_FLAG_RTK_FIX)
    {
      pz_qc->u_rtk_check_flag = 0; 

      LOGI(TAG_VDR, "[gnss:rtk_check:]:not check fix after imu lost ,%.3lf,%.3f\n",
        pd_mea[1], pz_nav->d_imu_shortloss_time);
    }
    else
    { 
      if ((u_bds_status == TYPE_POS_FLAG_RTK_FIX && 
        (f_gnssstd < 0.2 && (pd_mea[18] > 35.0/*for 571 1.0data*/ || (pd_mea[19] > 38.0 /* && pd_mea[18] > 15.0*/)))
        && pz_qc->q_cont_rtk_fix_num >= (uint32_t)(MAX(pz_qc->u_bds_rate,5))) ||
        (pz_qc->q_good_bds_num == gz_qc_para.q_cont_goodbds_thres
          && pz_qc->d_badbds_end_time > 0.0 && pd_mea[1] - pz_qc->d_badbds_end_time < 30.0/*120.0*/ + pz_qc->d_bds_zupt_time_all)
          )
      {
        pz_qc->u_rtk_check_flag = 0; /* not check good bds within 30s after tunnel */
            
        LOGI(TAG_VDR, "[gnss:rtk_check:]:not check lamda,%.3lf,%.3f,%.3f,%d,%d, %d, %d\n",
          pd_mea[1], pz_qc->d_badbds_end_time, pz_qc->d_bds_zupt_time_all,
          pz_qc->q_good_bds_num, gz_qc_para.q_cont_goodbds_thres, pz_qc->q_cont_rtk_fix_num, (pd_mea[19] > 38.0));
      }
      else
      {
        pz_qc->u_rtk_check_flag = 1;

        LOGI(TAG_VDR, "[gnss:rtk_check:]:check lamda,%.3lf,%.3f,%.3f,%d,%d, %d\n",
          pd_mea[1], pz_qc->d_badbds_end_time, pz_qc->d_bds_zupt_time_all,
          pz_qc->q_good_bds_num, gz_qc_para.q_cont_goodbds_thres, pz_qc->q_cont_rtk_fix_num);
      }
    }
  }

#if 1
  if (pz_qc->d_pre_good_bds_time >= 0.0 && pd_mea[1] - pz_qc->d_pre_good_bds_time < 120.0 + pz_qc->d_bds_zupt_time_all)
  {
#if 0
    if (u_bds_status == TYPE_POS_FLAG_GNSSONLY)
    {
      f_lambda_pos_thre1 = 1.0f;
      f_lambda_pos_thre2 = 10.0f;
    }
    else if (u_bds_status == TYPE_POS_FLAG_DGNSS)
    {
      f_lambda_pos_thre1 = 5.0f;
      f_lambda_pos_thre2 = 20.0f;
    }
    else if (u_bds_status == TYPE_POS_FLAG_RTK_FLOAT)
    {
      if (pd_mea[19] < 35.0 && pd_mea[17] < 5.0)
      {
        f_lambda_pos_thre1 = 1.0f;
        f_lambda_pos_thre2 = 20.0f;
      }
      else
      {
        f_lambda_pos_thre1 = 36;
        f_lambda_pos_thre2 = 200;
      }
    }
#else
    f_lambda_pos_thre1 = 1.0f;
    f_lambda_pos_thre2 = 20.0f;
#endif
  }
#endif
  pz_qc->f_lambda_pos_thre1 = f_lambda_pos_thre1;

  pz_qc->f_lambda_pos_thre2 = f_lambda_pos_thre2;
}

void qc_verdict_good_meas(double pd_mea[], InertialNav_t* pz_inav,GNSSQCPara_t* pz_qc)
{
  if (pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX && sqrt(pd_mea[8] + pd_mea[9]) < 0.2 
    && (pd_mea[1] - pz_qc->d_last_check_time < 1.5* pz_inav->f_gnss_sampling_dt)) //good fix
  {
    pz_qc->q_cont_rtk_fix_num++;
    if (pz_qc->q_cont_rtk_fix_num >= pz_qc->q_cont_rtk_thres)
    {
      pz_qc->d_cont_rtk_fix_time = pd_mea[1];
      pz_qc->q_cont_rtk_fix_num = pz_qc->q_cont_rtk_thres;
    }
  }
  else
  {
    pz_qc->q_cont_rtk_fix_num = 0;
  }

  pz_inav->pz_obs->f_bds_dr_pos_diff = sqrtf(SQR(pz_inav->pz_obs->f_kf_meas[0]) + SQR(pz_inav->pz_obs->f_kf_meas[1]));
    
#if 0
  if (pz_inav->pz_obs->f_bds_dr_pos_diff < 1.f && d_horpos_std < 0.5
      && (pd_mea[17] > 5.0 && pd_mea[19] > 28.0 || pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX))
  {
    pz_qc->q_good_bds_num++;
    if (pz_qc->q_good_bds_num >= q_thrshold || pz_qc->q_cont_rtk_fix_num == pz_qc->q_cont_rtk_thres)
    {
      pz_qc->q_good_bds_num = q_thrshold;
      pz_qc->d_pre_good_bds_time = pd_mea[1];
      pz_qc->d_bds_zupt_time_all = 0.f;
      pz_inav->u_adjust_syscov = INS_FALSE;
    }
  }
  else
  {
    if (is_izupt() == 1)
    {
      pz_qc->d_bds_zupt_time_all += pz_inav->f_gnss_sampling_dt;
      if (pz_qc->d_bds_zupt_time_all > MAX_TIME_SPAN)
      {
        pz_qc->d_bds_zupt_time_all = MAX_TIME_SPAN;
      }
    }
    pz_qc->q_good_bds_num = 0;
  }
#endif

  if (is_izupt() == 1)
  {
    pz_qc->d_bds_zupt_time_all += pz_inav->f_gnss_sampling_dt;
    if (pz_qc->d_bds_zupt_time_all > MAX_TIME_SPAN)
    {
      pz_qc->d_bds_zupt_time_all = MAX_TIME_SPAN;
    }
  }
  else
  {
    if (((pz_inav->pz_obs->u_posflag == TYPE_POS_FLAG_RTK_FIX && sqrt(pd_mea[8] + pd_mea[9]) < 0.2) ||
      (pd_mea[19] > 28.0 && pz_inav->pz_obs->u_posflag > TYPE_POS_FLAG_GNSSONLY && sqrt(pd_mea[8] + pd_mea[9]) < 0.3)) &&
      (pd_mea[1] - pz_qc->d_last_check_time < 1.5 * pz_inav->f_gnss_sampling_dt))
    {
      pz_qc->q_good_bds_num++;

      if (pz_qc->q_good_bds_num >= gz_qc_para.q_cont_goodbds_thres || pz_qc->q_cont_rtk_fix_num == pz_qc->q_cont_rtk_thres)
      {
        if (pd_mea[1] - pz_qc->d_pre_good_bds_time > 30.0 + pz_qc->d_bds_zupt_time_all && pz_qc->d_pre_good_bds_time>=0.0)
        {
          pz_qc->d_badbds_start_time = pz_qc->d_pre_good_bds_time;
          pz_qc->d_badbds_end_time = pd_mea[1];
        }
        pz_qc->q_good_bds_num = gz_qc_para.q_cont_goodbds_thres;
        pz_qc->d_pre_good_bds_time = pd_mea[1];
        pz_qc->d_bds_zupt_time_all = 0.f;
        pz_inav->u_adjust_syscov = INS_FALSE;
      }
    }
    else
    {
      pz_qc->q_good_bds_num = 0;
    }
  }

  LOGI(TAG_VDR, "[gnss:good_bds]:%.3lf,pre GodBdsT:%.3lf,GodBdsN:%d,posdiff:%.3f,pre cont_rtk_tcur:%.3lf,cont_rtk_num:%d,TimeZuptAll:%.3lf\n",
    pd_mea[1], pz_qc->d_pre_good_bds_time, pz_qc->q_good_bds_num,
    pz_inav->pz_obs->f_bds_dr_pos_diff, pz_qc->d_cont_rtk_fix_time, pz_qc->q_cont_rtk_fix_num, pz_qc->d_bds_zupt_time_all);
}

uint8_t qc_check_precondition(double pd_mea[],float pf_kfmeas[],GNSSQCPara_t* pz_qc,InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_FALSE;

  qc_verdict_good_meas(pd_mea, pz_inav, pz_qc); /* judge good gnss (time ,zupttimelen) */
  qc_adjust_rtk_check(pd_mea, pz_inav, pz_inav->pz_config, pz_qc); /* judge use lamda or not, adjust lamda thresh */
  /* qc_adjust_reject_thre(pd_mea, pz_qc, pz_inav->pz_config); // set and adjust bds reject thresh */
  qc_check_converged_gbias(pz_qc); /* if bias maxdiff < 0.08deg/s, converge */
  qc_check_bds_number(pd_mea, pz_qc); /* bds continuous valid timelen (based on time gap) */

#if 1
  if (pz_qc->u_gbias_converged == INS_TRUE)
#else
  if (pz_qc->q_bds_valid_counter > pz_qc->q_bds_converged_thre && pz_qc->u_gbias_converged == INS_TRUE)
#endif
  {
    u_val = INS_TRUE;
  }
  return u_val;
}

int8_t qc_chi_square_check(double pd_mea[],float pf_kfmeas[], GNSSQCPara_t* pz_qc)
{
  int8_t s_lc_pass = -1;
  int8_t s_pos_pass = 0;
  int8_t s_vel_pass = 0;
  SysErrModel_t* pz_model = fusion_get_sysmodel();

  s_pos_pass = qc_check_meas_pos(pd_mea, pf_kfmeas, pz_qc, pz_model);

  s_vel_pass = qc_check_meas_vel(pd_mea, pf_kfmeas, pz_qc, pz_model);

  s_vel_pass = pz_qc->u_adanoise_valid == INS_FALSE ? 0 : s_vel_pass;

  LOGD(TAG_VDR, "qc chi square check pos %d vel %d\n", s_pos_pass, s_vel_pass);

  /* comment for Misra C */

  //if (s_pos_pass > 0 && s_vel_pass > 0)
  //{
  //  s_lc_pass = -1;
  //}
  //else if (s_vel_pass > 0)
  //{
  //  s_lc_pass = 2; /* reject pos update */
  //}
  //else if (s_pos_pass > 0)
  //{
  //  s_lc_pass = 1; /* reject vel update */
  //}
  //else
  //{
  //  s_lc_pass = 0;
  //}
  return s_lc_pass;
}

/*
* @brief: GNSS Quality control
* @input: pd_bdsmea - gnss meas; pz_inav - navigation variable
* @output: None
* @return: 0 or 1 or -1 or 2: 0->all reject£¬ 1->reject vel£¬ 2->reject pos, -1 ->all accept
*/
int8_t bds_qc_procedure(double pd_bdsmea[], InertialNav_t* pz_inav)
{
  int8_t s_pass_test = -1;
  GNSSQCPara_t* pz_qc = &gz_qc_para;
  float* pf_kfmea = pz_inav->pz_obs->f_kf_meas;
  uint8_t u_bds_posflag = pz_inav->pz_obs->u_posflag;

  uint8_t u_qc_use_flag = INS_FALSE;
  uint8_t u_qc_precheck_ret = INS_FALSE; 
  u_qc_precheck_ret = qc_check_precondition(pd_bdsmea, pf_kfmea, pz_qc, pz_inav);
  if (u_qc_precheck_ret == INS_FALSE)
  {
    if (u_bds_posflag == TYPE_POS_FLAG_RTK_FIX || (u_bds_posflag == TYPE_POS_FLAG_RTK_FLOAT && pd_bdsmea[18] > 18.0 && pd_bdsmea[19] > 38.0))
    {
      /* not check lamda for good gnss while gbias unconverge */
    }
    else
    {
      if (pz_qc->d_pre_good_bds_time > 0.0)
      {
        u_qc_use_flag = INS_TRUE;
        LOGE(TAG_VDR, "gbias return to unconverge, but gnss bad, check lamda\n");
      }
    }    
  }
  else
  {
    u_qc_use_flag = INS_TRUE;
  }

  if (u_qc_use_flag == INS_TRUE)
  {
    s_pass_test = qc_chi_square_check(pd_bdsmea, pf_kfmea, pz_qc);
    /*
    if (s_pass_test == 0 || s_pass_test == 2)
    {

      pz_qc->q_bds_reject_counter++;
      if (pz_qc->q_bds_reject_counter > pz_qc->q_bds_reject_thre)
      {
        pz_qc->q_bds_valid_counter = 0;
        pz_qc->q_bds_reject_counter = 0;
      }

      s_pass_test = 0;
#if 1
      LOGE(TAG_VDR, "[bds_qc]: %.3f,BDS Is Rejected,reject counter: %d,reject threshold: %d!\n", pd_bdsmea[1],
        gz_qc_para.q_bds_reject_counter, gz_qc_para.q_bds_reject_thre);
#endif
    }
    else
    {
      if (pd_bdsmea[1] - pz_qc->d_last_lowspd_time > 10.0)
      {
        if (pz_qc->q_bds_reject_counter > 0)
        {
          pz_qc->q_bds_reject_counter = 0;
        }
      }
    }
    */
    /* keep current logic */
    if (pd_bdsmea[1] - pz_qc->d_last_lowspd_time > 10.0)
    {
      if (pz_qc->q_bds_reject_counter > 0)
      {
        pz_qc->q_bds_reject_counter = 0;
      }
    }
    pz_inav->pz_obs->f_lambda_pos = pz_qc->f_pos_lambda;
    pz_inav->pz_obs->f_lambda_vel = pz_qc->f_vel_lambda;
  }
  else
  {
    s_pass_test =  -1;
    if (gz_qc_para.q_bds_reject_counter > 0U)
    {
      gz_qc_para.q_bds_reject_counter = 0U;
    }
    LOGI(TAG_VDR, "[bds_qc]:%.3lf, gbias not converged and gnss not bad, no lambda check, q_bds_valid_counter = %d\n", pd_bdsmea[1], gz_qc_para.q_bds_valid_counter);
  }
  gz_qc_para.d_last_check_time = pd_bdsmea[1];

  return s_pass_test;
}