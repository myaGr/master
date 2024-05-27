/**@file        fusion_integrity.c
 * @brief		fusion integrity algorithm file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "fusion_integrity.h"
#include "fusion_global.h"
#include "fusion_proc.h"
#include "fusion_if.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mw_log.h"
#include "mw_alloc.h"

#ifdef OPEN_INS_INTEGRITY

#define AL 3.0f
#define PL_DIM 3 /*  3:only pos 6:pos+vel 9:pos+vel+att */

#define MIN_POS_PL 0.25f
#define MIN_VEL_PL 0.03f
#define MIN_ATT_PL 0.03f
#define MAX_POS_PL 99.99f

static PLCalParas_t gz_pl_paras;

void fusion_pl_para_init(void)
{
  memset(&gz_pl_paras, 0, sizeof(gz_pl_paras));
  gz_pl_paras.d_last_gnssposmeas_time = INITIAL_TIME;
  gz_pl_paras.d_last_badbds_diverg_time = INITIAL_TIME;
  gz_pl_paras.d_last_pldiff_cal_time = INITIAL_TIME;
}

static void pl_calculate_outage(INSResults_t* pz_ins, PLCalParas_t* pz_pl_paras, uint8_t i, float f_outage_timelen)
{
  EkfPosFeedback_t* pz_ekf_dpos = fusion_get_ekf_dpos();
  float f_ins_output_dt = 1.0f / fusion_get_algconfig()->z_init_para.e_output_rate;
  float f_outage_diverg = 4.0e-5f / 6.0f * (float)(pow(f_outage_timelen, 3.0) - pow((double)f_outage_timelen - f_ins_output_dt, 3.0)); /* too large ? */

  if (is_izupt() == 1)
  {
    pz_pl_paras->f_delta_pl[i] = 0.0; // -fabsf(pz_ekf_dpos->f_errstate[i]);

    LOGI(TAG_VDR, "[pl+0, in outage zupt]:%.3lf,dx=%f,pl_pre=%.3f,outage_dt=%.3f,pl_dt=%.3f\n",
      pz_ins->t_timestamp / 1000.0,
      pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i],
      f_outage_timelen, f_outage_diverg);
  }
  else
  {
    if ((pz_ins->q_kfmeastype & SMT_NHC) != 0)
    {
      f_outage_diverg *= 10.0;
      LOGI(TAG_VDR, "[in outage]: no wheelspeed, enlarge d_pl=%.7f\n",
        f_outage_diverg);
    }

    /* first 53s f_outage_diverg too small, turn up */
    /* 52s/1m, 67s/2m, 76s/3m */
    if (pz_ins->t_timestamp / 1000.0 - pz_pl_paras->d_last_badbds_diverg_time < 5.0
      && pz_ins->t_timestamp / 1000.0 - pz_pl_paras->d_last_pldiff_cal_time < 5.0)
    {
      pz_pl_paras->u_outage_closely_follow_badbds = 1u;
      LOGI(TAG_VDR, "[in outage closely follow badbds]: pl_diff=%.7f\n", pz_pl_paras->f_pl_diff[i]);
    }
    if (pz_pl_paras->u_outage_closely_follow_badbds == 1u && f_outage_timelen < 67.0f)
    {
      f_outage_diverg = pz_pl_paras->f_pl_diff[i] * 1.5f;
    }

    pz_pl_paras->f_delta_pl[i] = f_outage_diverg; // -fabsf(pz_ekf_dpos->f_errstate[i]);

    LOGI(TAG_VDR, "[pl+diverg, in outage]:%.3lf,dx=%f,pl_pre=%.3f,outage_dt=%.3f,d_pl=%.7f\n",
      pz_ins->t_timestamp / 1000.0,
      pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i],
      f_outage_timelen, f_outage_diverg);
  }
}

static void pl_calculate_up(INSResults_t* pz_ins, PLCalParas_t* pz_pl_paras, uint8_t i)
{
  EkfPosFeedback_t* pz_ekf_dpos = fusion_get_ekf_dpos();
  float f_ins_output_dt = 1.0f / fusion_get_algconfig()->z_init_para.e_output_rate;

  /* lamda too small && R too large(similar to outage), shall not judge using R */
  if ((pz_ekf_dpos->f_pos_lambda < 0.2f && pz_ekf_dpos->f_pos_lambda > 0.0f && sqrtf(SQR(pz_pl_paras->f_measnoise[0]) + SQR(pz_pl_paras->f_measnoise[1])) > 6.0f)
    || (sqrtf(SQR(pz_pl_paras->f_measnoise[0]) + SQR(pz_pl_paras->f_measnoise[1])) >= 45.0f))
  {
    //float f_badbds_diverg = 0.01 / fusion_get_algconfig()->z_init_para.e_output_rate; //diverg 0.03m in 1s  ...notice pl<0                            
    //float f_badbds_diverg = 4.0e-5f / 6.0 * (pow(d_outage_timelen, 3) - pow(d_outage_timelen - ins_output_dt, 3));
    //float f_badbds_diverg = fabsf(pz_ekf_dpos->f_errstate[i]) * 2.0 + 0.07*ins_output_dt; /*easy to cause PL too large*/
    float f_badbds_diverg = fabsf(pz_ekf_dpos->f_errstate[i]) * 2.0f + 0.01f * f_ins_output_dt; /*easy to cause PL too large*/

    static uint8_t u_num = 0;
    static float f_last_pl[9] = { 0.0f };
    if (u_num < 1u)
    {
      memcpy(f_last_pl, pz_pl_paras->f_pl_pre, sizeof(f_last_pl));
      pz_pl_paras->d_last_pldiff_cal_time = pz_ins->t_timestamp / 1000.0;
      u_num = 1u;
    }
    else
    {
      if (pz_ins->t_timestamp / 1000.0 - pz_pl_paras->d_last_pldiff_cal_time > 2.0 && pz_ins->t_timestamp / 1000.0 - pz_pl_paras->d_last_pldiff_cal_time < 5.0)
      {
        for (uint8_t k = 0; k < PL_DIM; k++)
        {
          pz_pl_paras->f_pl_diff[k] = (pz_pl_paras->f_pl_pre[k] - f_last_pl[k]) / ((float)(pz_ins->t_timestamp / 1000.0 - pz_pl_paras->d_last_pldiff_cal_time)) * f_ins_output_dt;

          LOGI(TAG_VDR, "[calculate pl_diff]: pl_diff=%.7f, t_cur=%.3f,pl:%.3f,  t_pre=%.3f,pl:%.3f \n",
            pz_pl_paras->f_pl_diff[k], pz_ins->t_timestamp / 1000.0, pz_pl_paras->f_pl_pre[k], pz_pl_paras->d_last_pldiff_cal_time, f_last_pl[k]);

          f_last_pl[k] = pz_pl_paras->f_pl_pre[k];
        }
        pz_pl_paras->d_last_pldiff_cal_time = pz_ins->t_timestamp / 1000.0;
      }
    }
    pz_pl_paras->d_last_badbds_diverg_time = pz_ins->t_timestamp / 1000.0;

    pz_pl_paras->f_delta_pl[i] = f_badbds_diverg + fabsf(pz_ekf_dpos->f_errstate[i]);

    LOGI(TAG_VDR, "[pl+diverg+|dx|, in bad bds]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
      pz_ins->t_timestamp / 1000.0,
      pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
  }
  else if (pz_pl_paras->f_pl_pre[i] /* *2.0 */ <= pz_pl_paras->f_measnoise[i] && sqrtf(SQR(pz_pl_paras->f_measnoise[0]) + SQR(pz_pl_paras->f_measnoise[1])) < 15.0f
    /* && fabsf(pz_ekf_dpos->f_pos_meas_z[i]) < 1.0 */)  /* avoid MI */
  {
    static double d_last_t = -2.0;
    if (pz_ins->t_timestamp / 1000.0 - d_last_t > 1.0)
    {
      for (uint8_t j = 0; j < PL_DIM; j++)
      {
        pz_pl_paras->f_dz_use[j] = (fabsf(pz_ekf_dpos->f_pos_meas_z[j]) > 0.1f) ? 0.1f : fabsf(pz_ekf_dpos->f_pos_meas_z[j]);
      }
      d_last_t = pz_ins->t_timestamp / 1000.0;
    }

    pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]) + pz_pl_paras->f_dz_use[i];

    if (pz_pl_paras->f_pl_pre[i] + pz_pl_paras->f_delta_pl[i] >= AL && pz_pl_paras->f_pl_pre[i] + fabsf(pz_ekf_dpos->f_errstate[i]) < AL)
    {
      pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]); /* improve availability */
      LOGI(TAG_VDR, "[pl+|dx|, not+|dz|]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f,measz=%.3f\n",
        pz_ins->t_timestamp / 1000.0,
        pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i], pz_ekf_dpos->f_pos_meas_z[i]);
    }
    else
    {
      LOGI(TAG_VDR, "[pl+|dx|+|dz|]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f,measz=%.3f,dz_use=%.3f\n",
        pz_ins->t_timestamp / 1000.0,
        pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i], pz_ekf_dpos->f_pos_meas_z[i], pz_pl_paras->f_dz_use[i]);
    }
  }
  else
  {
#if 0
    if (pz_ekf_dpos->q_update_type == (uint32_t)SMT_BDS ||
      pz_ekf_dpos->q_update_type == (uint32_t)SMT_GPSPOS ||
      pz_ekf_dpos->q_update_type == (uint32_t)SMT_GPSVEL ||
      pz_ekf_dpos->q_update_type == (uint32_t)SMT_DUALANT)
    {
      f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);

      LOGI(TAG_VDR, "[pl+|dx|]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
        pz_ins->t_timestamp / 1000.0,
        pz_ekf_dpos->f_errstate[i], f_pl_pre[i], f_measnoise[i]);
    }
    else
    {
#if 0
      f_delta_pl[i] = -fabsf(pz_ekf_dpos->f_errstate[i]);

      LOGI(TAG_VDR, "[pl-|dx| for ws_mea]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
        pz_ins->t_timestamp / 1000.0,
        pz_ekf_dpos->f_errstate[i], f_pl_pre[i], f_measnoise[i]);
#else     
      //f_delta_pl[i] = -fabsf(pz_ekf_dpos->f_errstate[i]);

      LOGI(TAG_VDR, "[pl-0 for ws_mea]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
        pz_ins->t_timestamp / 1000.0,
        pz_ekf_dpos->f_errstate[i], f_pl_pre[i], f_measnoise[i]);
#endif
    }
#else
    pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);

    LOGI(TAG_VDR, "[pl+|dx|]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
      pz_ins->t_timestamp / 1000.0,
      pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
#endif
  }
}

static void pl_calculate_down(INSResults_t* pz_ins, PLCalParas_t* pz_pl_paras, uint8_t i)
{
  GNSSQCPara_t* pz_qc = fusion_get_qc_para();
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  EkfPosFeedback_t* pz_ekf_dpos = fusion_get_ekf_dpos();

  if (pz_pl_paras->f_pl_pre[i] - fabsf(pz_ekf_dpos->f_errstate[i]) <= AL && pz_pl_paras->f_pl_pre[i] < fabsf(pz_pl_paras->f_measnoise[i]) &&
    (sqrtf(SQR(pz_pl_paras->f_measnoise[0]) + SQR(pz_pl_paras->f_measnoise[1])) >= 1.0f &&
      (sqrtf(SQR(pz_ekf_dpos->f_pos_meas_z[0]) + SQR(pz_ekf_dpos->f_pos_meas_z[1])) > 0.5f)))   //1.5 //poa vel att...
  {
    pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);

    LOGI(TAG_VDR, "[pl+|dx|, avoid TIR]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
      pz_ins->t_timestamp / 1000.0,
      pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
  }
  else
  {
    FusionAlgoConfig_t* pz_algocfg = fusion_get_algconfig();

    if (pz_ekf_dpos->u_posflag == TYPE_POS_FLAG_RTK_FIX && /*f_measnoise[i] < 0.08 &&*/
      ((pz_ekf_dpos->f_pos_lambda > 10.0f && pz_pl_paras->f_pl_pre[i] < 0.1f) ||
        (sqrtf(SQR(pz_pl_paras->f_pl_pre[0]) + SQR(pz_pl_paras->f_pl_pre[1])) > 1.0f && (sqrtf(SQR(pz_ekf_dpos->f_pos_meas_z[0]) + SQR(pz_ekf_dpos->f_pos_meas_z[1])) < 0.2f)
          && pz_pl_paras->u_last_posflag > 0u && pz_pl_paras->u_last_posflag != TYPE_POS_FLAG_RTK_FIX &&
          (pz_ekf_dpos->d_gnssposmeas_time - pz_pl_paras->d_last_gnssposmeas_time < 1.8 / pz_algocfg->z_init_para.e_gnss_rate))))
    {
      pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);
      pz_pl_paras->u_badfix = 1u;

      LOGI(TAG_VDR, "[pl+|dx|, for bad fix]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
        pz_ins->t_timestamp / 1000.0,
        pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
    }
    else
    {
      if (pz_pl_paras->f_pl_pre[i] - fabsf(pz_ekf_dpos->f_errstate[i]) <= 1e-7f)
      {
        LOGI(TAG_VDR, "[pl too small, not -|dx|]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
      else if (pz_ekf_dpos->u_posflag != TYPE_POS_FLAG_RTK_FIX &&
        ((pz_pl_paras->f_pl_pre[i] < 0.5f) ||
          (pz_ekf_dpos->f_pos_lambda == 0.0f && pz_pl_paras->f_pl_pre[i] < 2.0f * fabsf(pz_ekf_dpos->f_pos_meas_z[i])) ||
          (pz_ekf_dpos->f_pos_lambda == 0.0f && pz_pl_paras->f_pl_pre[i] < 2.0f) ||
          (pz_ekf_dpos->f_pos_lambda == 0.0f && pz_pl_paras->f_pl_pre[i] < 4.0f && pz_qc->q_bds_valid_counter < (uint32_t)(12 * pz_algocfg->z_init_para.e_gnss_rate))))
      {
        pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);

        LOGI(TAG_VDR, "[pl+|dx|,nonfix]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
      else if (pz_ekf_dpos->u_posflag != TYPE_POS_FLAG_RTK_FIX && pz_ekf_dpos->f_pos_lambda == 0.0f
        && pz_pl_paras->f_pl_pre[i] < 5.0f) /* for case: nonfix gnss std too small and no lamda check */
      {
        pz_pl_paras->f_delta_pl[i] = -0.1f * fabsf(pz_ekf_dpos->f_errstate[i]);

        LOGI(TAG_VDR, "[pl-0.1*|dx|, avoid MI]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
      else if (pz_ekf_dpos->u_posflag == TYPE_POS_FLAG_RTK_FIX &&
        ((pz_ekf_dpos->f_pos_lambda == 0.0f && pz_ins->t_timestamp / 1000.0 - pz_inav->d_zupt_reinit_time < 3.0 && fabsf(pz_ekf_dpos->f_pos_meas_z[i]) > 0.3f) ||
          ((pz_ekf_dpos->f_pos_lambda == 0.0f || pz_ekf_dpos->f_pos_lambda > 30.0f/*10.0f*/) 
            && (pz_pl_paras->f_pl_pre[i] < 2.0f * fabsf(pz_ekf_dpos->f_pos_meas_z[i])
            || fabsf(pz_ekf_dpos->f_pos_meas_z[0]) > 0.15f/*0.1f*/ || fabsf(pz_ekf_dpos->f_pos_meas_z[1]) > 0.15f/*0.1f*/) && pz_pl_paras->f_pl_pre[i] < 5.0f)))
      {
        pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);
        
        LOGI(TAG_VDR, "[pl+|dx|,fix]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
      else if (pz_ekf_dpos->f_pos_lambda > 1.0f && pz_pl_paras->f_pl_pre[i] < 0.5f /*2.5 pl_min too large*/)
      {
        pz_pl_paras->f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);
        
        pz_pl_paras->u_badfix = 1u;

        LOGI(TAG_VDR, "[pl+|dx|,bad fix]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
#if 0 /*can not use*/
      else if (pz_ekf_dpos->u_posflag == TYPE_POS_FLAG_RTK_FIX && fabsf(pz_ekf_dpos->f_errstate[i]) < 0.08
        && f_pl_pre[i] > 1.0 && f_pl_pre[i] < 2.0) /* for case: bad fix */
      {
        f_delta_pl[i] = fabsf(pz_ekf_dpos->f_errstate[i]);

        LOGI(TAG_VDR, "[pl+|dx|, avoid MI]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], f_pl_pre[i], f_measnoise[i]);
      }
#endif
      else if (pz_ekf_dpos->u_posflag == TYPE_POS_FLAG_RTK_FIX && pz_pl_paras->u_last_posflag != TYPE_POS_FLAG_RTK_FIX
        && fabsf(pz_ekf_dpos->f_errstate[i]) > pz_pl_paras->f_pl_pre[i] / 3.0f
        && (pz_ekf_dpos->d_gnssposmeas_time - pz_pl_paras->d_last_gnssposmeas_time < 1.8 / pz_algocfg->z_init_para.e_gnss_rate)
        /* && fabsf(pz_ekf_dpos->f_errstate[i]) > 0.5 * fabsf(pz_ekf_dpos->f_pos_meas_z[i]) */) /* for case: bad fix but no lamda check*/
      {
        pz_pl_paras->f_delta_pl[i] = -0.3f * fabsf(pz_ekf_dpos->f_errstate[i]);
        pz_pl_paras->u_badfix = 1u;

        LOGI(TAG_VDR, "[pl-0.3*|dx|, avoid MI]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
      else if (is_izupt() == 1 && pz_ekf_dpos->u_posflag == TYPE_POS_FLAG_RTK_FIX)
      {
        //for bad fix while zupt
        pz_pl_paras->f_delta_pl[i] = 0.0;
        pz_pl_paras->u_badfix = 1u;

        LOGI(TAG_VDR, "[pl-0,for bad fix while zupt, avoid HMI]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
      else
      {
        pz_pl_paras->f_delta_pl[i] = -fabsf(pz_ekf_dpos->f_errstate[i]);

        LOGI(TAG_VDR, "[pl-|dx|]:%.3lf,dx=%f,pl_pre=%.3f,measnoise=%.3f\n",
          pz_ins->t_timestamp / 1000.0,
          pz_ekf_dpos->f_errstate[i], pz_pl_paras->f_pl_pre[i], pz_pl_paras->f_measnoise[i]);
      }
    }
  }
}

void fusion_protection_level_calculation(SysErrModel_t* pz_model, INSResults_t* pz_ins)
{
  PLCalParas_t* pz_pl_paras = &gz_pl_paras;
  NavParams_t* pz_nav = fusion_get_navpara();
  InertialNav_t* pz_inav = fusion_get_inertial_nav();
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  uint8_t u_before_first_gnssmeas = 0u;
  pz_ins->z_inspl.t_timestamp = pz_ins->t_timestamp;
  pz_ins->z_inspl.e_drposflag = pz_ins->e_drposflag;

  if (pz_ins->e_drposflag == TYPE_DR_POS_FLAG_FUSION || pz_ins->e_drposflag == TYPE_DR_POS_FLAG_DR_OLY)
  {
    EkfPosFeedback_t* pz_ekf_dpos = fusion_get_ekf_dpos();
    float f_heading = pz_nav->f_att_rph[2];
    float f_pl_cur[9] = { 0.0f };

    memset(pz_pl_paras->f_delta_pl, 0, sizeof(pz_pl_paras->f_delta_pl));
    pz_pl_paras->u_badfix = 0u;

    if (pz_inav->u_inspl_init_flag == 0u)
    {
      for (uint8_t i = 0; i < PL_DIM; i++)
      {
        if (pz_pl_paras->f_pl_pre[i] < sqrtf(pz_model->f_syscovmat[i * pz_model->u_sysdim + i]))
        {
          pz_pl_paras->f_pl_pre[i] = sqrtf(pz_model->f_syscovmat[i * pz_model->u_sysdim + i]);
          LOGI(TAG_VDR, "[turn up pl_pre to P]:%.3lf, pl_pre:%.3lf\n", pz_ins->t_timestamp / 1000.0, pz_pl_paras->f_pl_pre[i]);
        }
        /*if (pz_ekf_dpos->u_posflag != TYPE_POS_FLAG_RTK_FIX)
        {
          f_pl_pre[i] = (f_pl_pre[i] > 5.0) ? f_pl_pre[i] : 5.0;
        }*/
      }
      for (uint8_t i = 0; i < 3; i++)
      {
        pz_pl_paras->f_pl_pre[i] = (pz_pl_paras->f_pl_pre[i] > MIN_POS_PL) ? pz_pl_paras->f_pl_pre[i] : MIN_POS_PL;
        pz_pl_paras->f_pl_pre[i] = (pz_pl_paras->f_pl_pre[i] < MAX_POS_PL) ? pz_pl_paras->f_pl_pre[i] : MAX_POS_PL;
        pz_pl_paras->f_pl_pre[i + 3] = (pz_pl_paras->f_pl_pre[i + 3] > MIN_VEL_PL) ? pz_pl_paras->f_pl_pre[i + 3] : MIN_VEL_PL;
        pz_pl_paras->f_pl_pre[i + 6] = (pz_pl_paras->f_pl_pre[i + 6] > MIN_ATT_PL) ? pz_pl_paras->f_pl_pre[i + 6] : MIN_ATT_PL;
      }
      pz_inav->u_inspl_init_flag = 1u;
    }

    float f_gnss_sample_dt = 1.0f / fusion_get_algconfig()->z_init_para.e_gnss_rate;
    float f_outage_timelen = 0.0f;
    if (pz_ekf_dpos->d_gnssposmeas_time > 0.0) 
    {
      f_outage_timelen = (float)(pz_ins->t_timestamp / 1000.0 - pz_ekf_dpos->d_gnssposmeas_time);

      pz_pl_paras->f_measnoise[0] = pz_ekf_dpos->f_measnoise_pos[0];
      pz_pl_paras->f_measnoise[1] = pz_ekf_dpos->f_measnoise_pos[1];
      pz_pl_paras->f_measnoise[2] = pz_ekf_dpos->f_measnoise_pos[2];
#if (PL_DIM == 6) 
      for (uint8_t j = 0; j < 3; j++)
      {
        pz_pl_paras->f_measnoise[j + 3] = 0.5f;
      }
#elif (PL_DIM == 9) 
      for (uint8_t j = 0; j < 6; j++)
      {
        pz_pl_paras->f_measnoise[j + 3] = 0.5f;
      }
#endif
    }
    else
    {
      u_before_first_gnssmeas = 1u; /* pz_ekf_dpos not available yet*/
    }

    /*LOGI(TAG_VDR, "[pl outage check]:%.3lf,%.3lf\n",
        pz_ins->t_timestamp / 1000.0, pz_ekf_dpos->d_gnssposmeas_time);*/

    for (uint8_t i = 0; i < PL_DIM; i++)
    {
      if ((f_outage_timelen > 1.5 * f_gnss_sample_dt) || (u_before_first_gnssmeas == 1u)) /* outage case */
      {
        pl_calculate_outage(pz_ins, pz_pl_paras, i, f_outage_timelen);
      }
      else
      {
        pz_pl_paras->u_outage_closely_follow_badbds = 0u;
        if (pz_pl_paras->f_pl_pre[i] <= pz_pl_paras->f_measnoise[i])
        {
          pl_calculate_up(pz_ins, pz_pl_paras, i);
        }
        else
        {
          pl_calculate_down(pz_ins, pz_pl_paras, i);
        }
      }
      f_pl_cur[i] = pz_pl_paras->f_pl_pre[i] + pz_pl_paras->f_delta_pl[i];

      /* reset dx */
      pz_ekf_dpos->f_errstate[i] = 0.0;
    }

    memset(pz_pl_paras->f_dz_use, 0, sizeof(pz_pl_paras->f_dz_use));

    /*after outage& good gnss detected, force turn down PL*/
    GNSSQCPara_t* pz_qc = fusion_get_qc_para();
    if (((pz_inav->pz_qc->u_rtk_check_flag == 0 && pz_inav->pz_qc->d_badbds_end_time > 0.0 &&
      pz_ins->t_timestamp / 1000.0 - pz_inav->pz_qc->d_badbds_end_time < 30.0/*120.0*/ + pz_inav->pz_qc->d_bds_zupt_time_all &&
      pz_ins->t_timestamp / 1000.0 - pz_inav->pz_qc->d_badbds_end_time > 2.0 + pz_inav->pz_qc->d_bds_zupt_time_all) /*for case: bad fix*/
      || ((pz_qc->q_cont_rtk_fix_num == pz_qc->q_cont_rtk_thres) && sqrtf(SQR(pz_ekf_dpos->f_pos_meas_z[0]) + SQR(pz_ekf_dpos->f_pos_meas_z[1])) < 0.3))
      && (pz_pl_paras->u_badfix == 0u))
    {
      LOGI(TAG_VDR, "[pl force down after outage or at good fix]:%.3lf,%.3lf,%.3lf,posflag:%d,zupt:%d,cont_fix_num:%d,z:%.3f,%.3f,badfix:%d\n",
        pz_ins->t_timestamp / 1000.0, pz_inav->pz_qc->d_badbds_end_time, pz_inav->pz_qc->d_bds_zupt_time_all,
        /*pz_ekf_dpos->u_posflag,*/ pz_gnss->e_posflag, is_izupt(), pz_qc->q_cont_rtk_fix_num,
        pz_ekf_dpos->f_pos_meas_z[0], pz_ekf_dpos->f_pos_meas_z[1], pz_pl_paras->u_badfix);

      /*force turn down PL for good bds within 30s after tunnel*/
      if (pz_gnss->e_posflag == TYPE_POS_FLAG_RTK_FIX && is_izupt() == 0) /*avoid bad fix in zupt*/
      {
        /*f_pl_cur[0] = (f_pl_cur[0] > MIN_POS_PL * 3.0) ? (MIN_POS_PL * 3.0) : f_pl_cur[0];
        f_pl_cur[1] = (f_pl_cur[1] > MIN_POS_PL * 3.0) ? (MIN_POS_PL * 3.0) : f_pl_cur[1];
        f_pl_cur[2] = (f_pl_cur[2] > MIN_POS_PL * 3.0) ? (MIN_POS_PL * 3.0) : f_pl_cur[2];*/

        /*for bad fix*/
        f_pl_cur[0] = (f_pl_cur[0] > 2.0f) ? 2.0f : f_pl_cur[0];
        f_pl_cur[1] = (f_pl_cur[1] > 2.0f) ? 2.0f : f_pl_cur[1];
        f_pl_cur[2] = (f_pl_cur[2] > 2.0f) ? 2.0f : f_pl_cur[2];
      }
      else /*avoid MI*/
      {
        f_pl_cur[0] = (f_pl_cur[0] > 3.5f) ? 3.5f : f_pl_cur[0];
        f_pl_cur[1] = (f_pl_cur[1] > 3.5f) ? 3.5f : f_pl_cur[1];
        f_pl_cur[2] = (f_pl_cur[2] > 3.5f) ? 3.5f : f_pl_cur[2];
      }
      if (PL_DIM > 3)
      {
        f_pl_cur[3] = (f_pl_cur[3] > 0.1f) ? 0.1f : f_pl_cur[3];
        f_pl_cur[4] = (f_pl_cur[4] > 0.1f) ? 0.1f : f_pl_cur[4];
        f_pl_cur[5] = (f_pl_cur[5] > 0.1f) ? 0.1f : f_pl_cur[5];
      }
      if (PL_DIM > 6)
      {
        f_pl_cur[6] = (f_pl_cur[6] > (float)(0.2 * DEG2RAD)) ? (float)(0.2f * DEG2RAD) : f_pl_cur[6];
        f_pl_cur[7] = (f_pl_cur[7] > (float)(0.2 * DEG2RAD)) ? (float)(0.2f * DEG2RAD) : f_pl_cur[7];
        f_pl_cur[8] = (f_pl_cur[8] > (float)(0.5 * DEG2RAD)) ? (float)(0.5f * DEG2RAD) : f_pl_cur[8];
      }
    }

    /*reset pos lamda*/
    pz_ekf_dpos->f_pos_lambda = 0.0f;

    /* output max(inspl,gnsspl) ... */


    /*limit min PL*/
    float f_minpospl_scale = 1.0;
    if (pz_gnss->e_posflag == TYPE_POS_FLAG_RTK_FIX)
    {
      f_minpospl_scale = 1.0; /*0.2m*/
    }
    else if (pz_gnss->e_posflag == TYPE_POS_FLAG_RTK_FLOAT)
    {
      f_minpospl_scale = 2.5; /*0.5m*/
    }
    else if (pz_gnss->e_posflag == TYPE_POS_FLAG_DGNSS)
    {
      f_minpospl_scale = 5.0; /*1.0m*/
    }
    else if (pz_gnss->e_posflag == TYPE_POS_FLAG_GNSSONLY)
    {
      f_minpospl_scale = 5.0; /*1.0m*/
    }
    f_pl_cur[0] = (f_pl_cur[0] > MIN_POS_PL * f_minpospl_scale) ? f_pl_cur[0] : MIN_POS_PL * f_minpospl_scale;
    f_pl_cur[1] = (f_pl_cur[1] > MIN_POS_PL * f_minpospl_scale) ? f_pl_cur[1] : MIN_POS_PL * f_minpospl_scale;
    f_pl_cur[2] = (f_pl_cur[2] > MIN_POS_PL * f_minpospl_scale) ? f_pl_cur[2] : MIN_POS_PL * f_minpospl_scale;
    if (PL_DIM > 3)
    {
      f_pl_cur[3] = (f_pl_cur[3] > MIN_VEL_PL) ? f_pl_cur[3] : MIN_VEL_PL;
      f_pl_cur[4] = (f_pl_cur[4] > MIN_VEL_PL) ? f_pl_cur[4] : MIN_VEL_PL;
      f_pl_cur[5] = (f_pl_cur[5] > MIN_VEL_PL) ? f_pl_cur[5] : MIN_VEL_PL;
    }
    if (PL_DIM > 6)
    {
      f_pl_cur[6] = (f_pl_cur[6] > MIN_ATT_PL) ? f_pl_cur[6] : MIN_ATT_PL;
      f_pl_cur[7] = (f_pl_cur[7] > MIN_ATT_PL) ? f_pl_cur[7] : MIN_ATT_PL;
      f_pl_cur[8] = (f_pl_cur[8] > MIN_ATT_PL) ? f_pl_cur[8] : MIN_ATT_PL;
    }

    memcpy(pz_pl_paras->f_pl_pre, f_pl_cur, sizeof(f_pl_cur));

    pz_ins->z_inspl.f_pos_longitudinal_pl = fabsf(f_pl_cur[0] * cosf(f_heading) + f_pl_cur[1] * sinf(f_heading));
    pz_ins->z_inspl.f_pos_lateral_pl = fabsf(f_pl_cur[0] * sinf(f_heading) - f_pl_cur[1] * cosf(f_heading));
    pz_ins->z_inspl.f_pos_hpl = sqrtf(SQR(f_pl_cur[0]) + SQR(f_pl_cur[1]));
    pz_ins->z_inspl.f_pos_vpl = f_pl_cur[2];
    pz_ins->z_inspl.f_pos_north_pl = f_pl_cur[0];
    pz_ins->z_inspl.f_pos_east_pl = f_pl_cur[1];

    pz_ins->z_inspl.f_vel_longitudinal_pl = fabsf(f_pl_cur[3] * cosf(f_heading) + f_pl_cur[4] * sinf(f_heading));
    pz_ins->z_inspl.f_vel_lateral_pl = fabsf(f_pl_cur[3] * sinf(f_heading) - f_pl_cur[4] * cosf(f_heading));
    pz_ins->z_inspl.f_vel_hpl = sqrtf(SQR(f_pl_cur[3]) + SQR(f_pl_cur[4]));
    pz_ins->z_inspl.f_vel_vpl = f_pl_cur[5];
    pz_ins->z_inspl.f_vel_north_pl = f_pl_cur[3];
    pz_ins->z_inspl.f_vel_east_pl = f_pl_cur[4];

    pz_ins->z_inspl.f_roll_pl = f_pl_cur[6];
    pz_ins->z_inspl.f_pitch_pl = f_pl_cur[7];
    pz_ins->z_inspl.f_yaw_pl = f_pl_cur[8];

    pz_pl_paras->u_last_posflag = pz_ekf_dpos->u_posflag;
    pz_pl_paras->d_last_gnssposmeas_time = pz_ekf_dpos->d_gnssposmeas_time;
  }
  else
  {
    /*output gnss pl(only pos)*/
    float f_heading = (float)(pz_gnss->f_heading * DEG2RAD);

    /*limit min gnss std*/
    float f_minpospl_scale = 1.0;
    if (pz_gnss->e_posflag == TYPE_POS_FLAG_RTK_FIX)
    {
      f_minpospl_scale = 1.0; /*0.2m*/
    }
    else if (pz_gnss->e_posflag == TYPE_POS_FLAG_RTK_FLOAT)
    {
      f_minpospl_scale = 2.5; /*0.5m*/
    }
    else if (pz_gnss->e_posflag == TYPE_POS_FLAG_DGNSS)
    {
      f_minpospl_scale = 5.0; /*1.0m*/
    }
    else if (pz_gnss->e_posflag == TYPE_POS_FLAG_GNSSONLY)
    {
      f_minpospl_scale = 5.0; /*1.0m*/
    }
    pz_gnss->f_latstd = (pz_gnss->f_latstd > MIN_POS_PL * f_minpospl_scale) ? pz_gnss->f_latstd : MIN_POS_PL * f_minpospl_scale;
    pz_gnss->f_lonstd = (pz_gnss->f_lonstd > MIN_POS_PL * f_minpospl_scale) ? pz_gnss->f_lonstd : MIN_POS_PL * f_minpospl_scale;
    pz_gnss->f_altstd = (pz_gnss->f_altstd > MIN_POS_PL * f_minpospl_scale) ? pz_gnss->f_altstd : MIN_POS_PL * f_minpospl_scale;

    /* limit min PL */
    pz_ins->z_inspl.f_pos_longitudinal_pl = fabsf(pz_gnss->f_latstd * cosf(f_heading) + pz_gnss->f_lonstd * sinf(f_heading));
    pz_ins->z_inspl.f_pos_lateral_pl = fabsf(pz_gnss->f_latstd * sinf(f_heading) - pz_gnss->f_lonstd * cosf(f_heading));
    pz_ins->z_inspl.f_pos_hpl = sqrtf(SQR(pz_gnss->f_latstd) + SQR(pz_gnss->f_lonstd));
    pz_ins->z_inspl.f_pos_vpl = pz_gnss->f_altstd;
    pz_ins->z_inspl.f_pos_north_pl = pz_gnss->f_latstd;
    pz_ins->z_inspl.f_pos_east_pl = pz_gnss->f_lonstd;    

    pz_ins->z_inspl.f_vel_longitudinal_pl = (pz_ins->z_inspl.f_vel_longitudinal_pl > MIN_VEL_PL) ? pz_ins->z_inspl.f_vel_longitudinal_pl : MIN_VEL_PL;
    pz_ins->z_inspl.f_vel_lateral_pl = (pz_ins->z_inspl.f_vel_lateral_pl > MIN_VEL_PL) ? pz_ins->z_inspl.f_vel_lateral_pl : MIN_VEL_PL;
    pz_ins->z_inspl.f_vel_hpl = (pz_ins->z_inspl.f_vel_hpl > MIN_VEL_PL) ? pz_ins->z_inspl.f_vel_hpl : MIN_VEL_PL;
    pz_ins->z_inspl.f_vel_vpl = (pz_ins->z_inspl.f_vel_vpl > MIN_VEL_PL) ? pz_ins->z_inspl.f_vel_vpl : MIN_VEL_PL;
    pz_ins->z_inspl.f_vel_north_pl = (pz_ins->z_inspl.f_vel_north_pl > MIN_VEL_PL) ? pz_ins->z_inspl.f_vel_north_pl : MIN_VEL_PL;
    pz_ins->z_inspl.f_vel_east_pl = (pz_ins->z_inspl.f_vel_east_pl > MIN_VEL_PL) ? pz_ins->z_inspl.f_vel_east_pl : MIN_VEL_PL;

    pz_ins->z_inspl.f_roll_pl = (pz_ins->z_inspl.f_roll_pl > MIN_ATT_PL) ? pz_ins->z_inspl.f_roll_pl : MIN_ATT_PL;
    pz_ins->z_inspl.f_pitch_pl = (pz_ins->z_inspl.f_pitch_pl > MIN_ATT_PL) ? pz_ins->z_inspl.f_pitch_pl : MIN_ATT_PL;
    pz_ins->z_inspl.f_yaw_pl = (pz_ins->z_inspl.f_yaw_pl > MIN_ATT_PL) ? pz_ins->z_inspl.f_yaw_pl : MIN_ATT_PL;

    /*save pl_pre*/
    pz_pl_paras->f_pl_pre[0] = pz_ins->z_inspl.f_pos_north_pl;
    pz_pl_paras->f_pl_pre[1] = pz_ins->z_inspl.f_pos_east_pl;
    pz_pl_paras->f_pl_pre[2] = pz_ins->z_inspl.f_pos_vpl;
    pz_pl_paras->f_pl_pre[3] = pz_ins->z_inspl.f_vel_north_pl;
    pz_pl_paras->f_pl_pre[4] = pz_ins->z_inspl.f_vel_east_pl;
    pz_pl_paras->f_pl_pre[5] = pz_ins->z_inspl.f_vel_vpl;
    pz_pl_paras->f_pl_pre[6] = pz_ins->z_inspl.f_roll_pl;
    pz_pl_paras->f_pl_pre[7] = pz_ins->z_inspl.f_pitch_pl;
    pz_pl_paras->f_pl_pre[8] = pz_ins->z_inspl.f_yaw_pl;
  }

  /* limit max pos PL */
  pz_ins->z_inspl.f_pos_longitudinal_pl = (pz_ins->z_inspl.f_pos_longitudinal_pl < MAX_POS_PL) ? pz_ins->z_inspl.f_pos_longitudinal_pl : MAX_POS_PL;
  pz_ins->z_inspl.f_pos_lateral_pl = (pz_ins->z_inspl.f_pos_lateral_pl < MAX_POS_PL) ? pz_ins->z_inspl.f_pos_lateral_pl : MAX_POS_PL;
  pz_ins->z_inspl.f_pos_hpl = (pz_ins->z_inspl.f_pos_hpl < MAX_POS_PL) ? pz_ins->z_inspl.f_pos_hpl : MAX_POS_PL;
  pz_ins->z_inspl.f_pos_vpl = (pz_ins->z_inspl.f_pos_vpl < MAX_POS_PL) ? pz_ins->z_inspl.f_pos_vpl : MAX_POS_PL;
  pz_ins->z_inspl.f_pos_north_pl = (pz_ins->z_inspl.f_pos_north_pl < MAX_POS_PL) ? pz_ins->z_inspl.f_pos_north_pl : MAX_POS_PL;
  pz_ins->z_inspl.f_pos_east_pl = (pz_ins->z_inspl.f_pos_east_pl < MAX_POS_PL) ? pz_ins->z_inspl.f_pos_east_pl : MAX_POS_PL;
}
#endif
