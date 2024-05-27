/**@file        fusion_tc.c
 * @brief		fusion tight coupling file
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

#include "fusion_tc.h"
#include "fusion_ellip_para.h"
#include "fusion_math.h"
#include "fusion_transform.h"
#include "fusion_if.h"
#include "fusion_quat.h"
#include "fusion_kf.h"
#include "fusion_log.h"
#include <string.h>
#include "mw_alloc.h"
#include "mw_log.h"
#include "ekf_task.h"
#include "ekf_api.h"

uint8_t tight_couple_psr_precheck(AsensingGNSSPsrMeas_t* pz_psr_paras, InertialNav_t* pz_inav)
{
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  MechanNode_t* pz_node = pz_obs->pz_bds_node;
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();

  float f_v_hor_cons_check = 0.0, f_v_alt_cons_check = 0.0;
  float f_ds_alt = 0.0, f_ds_hor = 0.0;
#if 1
  static float f_last_lat_deg = 0.0;
  static float f_last_lon_deg = 0.0;
  static float f_last_alt = 0.0;
  static float f_last_tow = 0.0;
  float f_v_mean_hor = 0.0, f_v_mean_alt = 0.0;
  float f_ds_n = 0.0;
  float f_ds_e = 0.0;
  float f_dt = 0.0;
  static float f_last_hor_speed = 0.0;
  static float f_last_alt_speed = 0.0;
  float f_hor_speed = 0.0;

  f_dt = (float)(pz_gnss->d_tow/1000.0 - f_last_tow);
  f_dt = (f_dt < -604879.0) ? f_dt + 604800 : f_dt;
  if (f_last_lat_deg > 0.0 && f_last_lon_deg > 0.0 && f_dt < 0.5 && f_dt >0.01)
  {
    f_hor_speed = sqrtf(pz_gnss->f_vn * pz_gnss->f_vn + pz_gnss->f_ve * pz_gnss->f_ve);
    f_v_mean_hor = (f_hor_speed + f_last_hor_speed) / 2.0f;
    f_v_mean_alt = (-pz_gnss->f_vd + f_last_alt_speed) / 2.0f;
    f_ds_n = (float)((pz_gnss->d_lat - f_last_lat_deg) * DEG2RAD * (pz_node->d_rm + pz_node->d_lla[2]));
    f_ds_e = (float)((pz_gnss->d_lon - f_last_lon_deg) * DEG2RAD * (pz_node->d_rn + pz_node->d_lla[2]) * cos(f_last_lat_deg * DEG2RAD));
    f_ds_alt = (pz_gnss->f_alt - f_last_alt);
    f_ds_hor = sqrtf(f_ds_n * f_ds_n + f_ds_e * f_ds_e);

    f_v_hor_cons_check = f_ds_hor / f_dt - f_v_mean_hor;
    f_v_alt_cons_check = f_ds_alt / f_dt - f_v_mean_alt;
  }
  f_last_lat_deg = (float)pz_gnss->d_lat;
  f_last_lon_deg = (float)pz_gnss->d_lon;
  f_last_alt = pz_gnss->f_alt;
  f_last_tow =(float)(pz_gnss->d_tow / 1000.0f);
  f_last_hor_speed = f_hor_speed;
  f_last_alt_speed = -pz_gnss->f_vd;
#endif
  double d_dt_psr_delay = pz_inav->pz_obs->pz_bds_node->d_time - pz_psr_paras->d_cur_tow;
  pz_inav->u_psr_used_satnum = 0;
  memset(pz_inav->z_psr_used_data, 0, sizeof(pz_inav->z_psr_used_data));
  if ((pz_obs->u_posflag != TYPE_POS_FLAG_UNAVAILABLE) &&
      d_dt_psr_delay < 0.35 && pz_gnss->f_avgsnr > 20.0)
  {
    for (int i = 0; i < pz_psr_paras->u_count; ++i)
    {
      if ((pz_psr_paras->z_raw_data[i].u_valid > 0)
          && pz_psr_paras->z_raw_data[i].u_snr >= 30 && pz_psr_paras->z_raw_data[i].f_ele >= 20.0)//
      {
        memcpy(&(pz_inav->z_psr_used_data[pz_inav->u_psr_used_satnum]), &(pz_psr_paras->z_raw_data[i]), sizeof(AsensingSatPsrRaw_t));
        pz_inav->u_psr_used_satnum++;
      }
    }
    LOGI(TAG_VDR, "$GPGNP, % .3f, % .3f, % d, % .3f, % d, % .3f, % .3f, % .3f, % d, % .3f, % .3f, % .3f, % .3f\n",
      pz_inav->pz_obs->pz_bds_node->d_time, pz_psr_paras->d_cur_tow, pz_obs->u_posflag,
      pz_gnss->f_avgsnr, pz_gnss->u_satused,
      pz_gnss->f_latstd, pz_gnss->f_lonstd, pz_gnss->f_altstd, pz_inav->u_psr_used_satnum,
      f_ds_hor, f_ds_alt, f_v_hor_cons_check, f_v_alt_cons_check);
  }
  else
  {
    LOGI(TAG_VDR, "$GPGNPX, % .3f, % .3f, % d, % .3f, % d, % .3f, % .3f, % .3f, % d, % .3f, % .3f, % .3f, % .3f\n",
      pz_inav->pz_obs->pz_bds_node->d_time, pz_psr_paras->d_cur_tow, pz_obs->u_posflag,
      pz_gnss->f_avgsnr, pz_gnss->u_satused,
      pz_gnss->f_latstd, pz_gnss->f_lonstd, pz_gnss->f_altstd, pz_inav->u_psr_used_satnum,
      f_ds_hor, f_ds_alt, f_v_hor_cons_check, f_v_alt_cons_check);
  }
  return 0;
}

uint8_t tight_couple_psr_procedure(InertialNav_t* pz_inav, NavParams_t* pz_para)
{
  uint8_t u_bds_update = INS_TRUE;
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  MechanNode_t* pz_node = pz_obs->pz_bds_node;
  AsensingSatPsrRaw_t* pz_psr_used_data = pz_inav->z_psr_used_data;
  SysErrModel_t* pz_model = fusion_get_sysmodel();
  uint8_t u_meadim = pz_model->u_meadim;
  uint8_t u_sysdim = pz_model->u_sysdim;

  float* f_z_meas = (float*)OS_MALLOC(u_meadim * sizeof(float)); 
  float* f_measnoise = (float*)OS_MALLOC(u_meadim * u_meadim * sizeof(float));
  float* f_sysmeasmat = (float*)OS_MALLOC(u_meadim * u_sysdim * sizeof(float));
  uint8_t u_mdim = 0;
  NavParams_t* pz_nav = fusion_get_navpara();
  pz_obs->u_kf_update = INS_FALSE;

  float f_n2e_poserr[3][3], * pf_n2e_poserr = (float*)(&f_n2e_poserr[0][0]);
  float f_e_sr[3], * pf_e_sr = (float*)(&f_e_sr[0]);
  float f_esr_n2e[3], * pf_esr_n2e = (float*)(&f_esr_n2e[0]);

  float f_rotmat_b2n[3][3];
  float f_matv2b[3][3],f_rotmat_v2n[3][3];
  float f_att[3];

  float* pf_imu2gnss = NULL;
  float f_lvrm_n[3] = { 0.f };
  float f_lvrm_b[3] = { 0.f };
  float f_lvrm_n_x[3][3] = { 0.f };

  double d_ins2bds_poslla[3] = { 0.0 };
  double d_ins2bds_pos_ecef[3] = { 0.0 };

  float f_wnbb[3], f_winb[3], f_wnbb_x[3][3];
  uint8_t u_pr_num = 0;
  uint8_t u_dr_num = 0;

  float f_diffrange, f_tempvector1[3];

  double d_rn, d_h, d_rn_h, d_rm_h;
  double d_sinlat, d_coslat, d_sinlon, d_coslon;

  d_rn = pz_node->d_rn;
  d_h = pz_node->d_lla[2];
  d_rn_h = pz_node->d_rn + d_h;
  d_sinlat = sin(pz_node->d_lla[0]);
  d_coslat = cos(pz_node->d_lla[0]);
  d_sinlon = sin(pz_node->d_lla[1]);
  d_coslon = cos(pz_node->d_lla[1]);
  d_rm_h = pz_node->d_rm + d_h;

  /* lat lon h */
  f_n2e_poserr[0][0] = (float)(-d_rn_h * d_sinlat * d_coslon / d_rm_h);
  f_n2e_poserr[0][1] = (float)(-d_sinlon);
  f_n2e_poserr[0][2] = (float)(-d_coslat * d_coslon);
  f_n2e_poserr[1][0] = (float)(-d_rn_h * d_sinlat * d_sinlon / d_rm_h);
  f_n2e_poserr[1][1] = (float)d_coslon;
  f_n2e_poserr[1][2] = (float)(-d_coslat * d_sinlon);
  f_n2e_poserr[2][0] = (float)((d_rn * (1 - EARTH_WIE) + d_h) * d_coslat / d_rm_h);
  f_n2e_poserr[2][1] = 0.f;
  f_n2e_poserr[2][2] = (float)(-d_sinlat);

  f_att[0] = pz_para->f_att_rph[0];
  f_att[1] = pz_para->f_att_rph[1];
  f_att[2] = pz_para->f_att_rph[2];
  tf_euler2dcm(f_att, f_rotmat_b2n);

  pf_imu2gnss = lvrm_get_imu2gnss();
  math_mattrns(&(pz_nav->f_rotmat_misalign[0][0]), 3, 3, &(f_matv2b[0][0]));
  math_matmul(&(f_rotmat_b2n[0][0]), &(f_matv2b[0][0]), 3, 3, 3, &(f_rotmat_v2n[0][0]));
  math_matmul(&(f_rotmat_v2n[0][0]), pf_imu2gnss, 3, 3, 1, f_lvrm_n);

  math_matmul(&(f_matv2b[0][0]), pf_imu2gnss, 3, 3, 1, f_lvrm_b);

  math_skewsym(f_lvrm_n, f_lvrm_n_x);
  d_ins2bds_poslla[0] = pz_node->d_lla[0] + (double)(f_lvrm_n[0]) / (pz_node->d_rm + d_h);
  d_ins2bds_poslla[1] = pz_node->d_lla[1] + (double)(f_lvrm_n[1]) / ((pz_node->d_rn + d_h) * cos(d_ins2bds_poslla[0]));
  d_ins2bds_poslla[2] = pz_node->d_lla[2] + (double)(-f_lvrm_n[2]);
#ifdef REF_READY
  T_LLH Ref_pos;
  float32 Cn2e_Ref[3][3], vel_ref[3];
  float64 PosLLARef[3], RefPosECEF[3];
  Ref_pos.dlat = gRefData.lat * DEG2RAD + Lntg[0] / pLEKF->pEP->Rnh;
  Ref_pos.dlon = gRefData.lon * DEG2RAD + Lntg[1] / pLEKF->pEP->RehcosL;
  Ref_pos.dhigh = gRefData.alt - Lntg[2];

  LLH2ECEF(&Ref_pos, &Ecef_refpos);

  PosLLARef[0] = Ref_pos.dlat;
  PosLLARef[1] = Ref_pos.dlon;
  PosLLARef[2] = Ref_pos.dhigh;

  vel_ref[0] = gRefData.vn;
  vel_ref[1] = gRefData.ve;
  vel_ref[2] = gRefData.vd;

  Convert_GetN2EDCMbyLLAPos(PosLLARef, Cn2e_Ref);
#endif
  tf_lla2ecef(d_ins2bds_poslla, d_ins2bds_pos_ecef, d_ins2bds_poslla);

  float f_winn[3], f_mat_tmp1[3][3], f_mat_tmp2[3][3], f_mat_tmp3[3][3];
  float f_vec_tmp1[3];
  float f_cmp_gyro[3];

  f_cmp_gyro[0] = (float)(pz_node->f_wibb[0] - pz_para->f_gyrobias[0]);
  f_cmp_gyro[1] = (float)(pz_node->f_wibb[1] - pz_para->f_gyrobias[1]);
  f_cmp_gyro[2] = (float)(pz_node->f_wibb[2] - pz_para->f_gyrobias[2]);

  f_winn[0] = (float)(EARTH_WIE * cos(d_ins2bds_poslla[0]) + pz_node->f_vel[1] / (pz_node->d_rn + d_ins2bds_poslla[2]));
  f_winn[1] = (float)(-pz_node->f_vel[0] / (pz_node->d_rm + d_ins2bds_poslla[2]));
  f_winn[2] = (float)(-EARTH_WIE * sin(d_ins2bds_poslla[0]) - pz_node->f_vel[1] * tan(d_ins2bds_poslla[0]) / (pz_node->d_rn + d_ins2bds_poslla[2]));

#if 1
  math_matmul(*f_rotmat_b2n, f_winn, 3, 3, 1, f_winb);
  math_matsub(f_cmp_gyro, f_winb, 3, 1, f_wnbb);
  math_skewsym(f_wnbb, f_wnbb_x);
  math_matmul(*f_rotmat_b2n, *f_wnbb_x, 3, 3, 3, *f_mat_tmp1);
  math_matmul(*f_mat_tmp1, *f_matv2b, 3, 3, 3, *f_mat_tmp2);
  math_matmul(*f_mat_tmp2, pf_imu2gnss, 3, 3, 1, f_vec_tmp1);
  math_skewsym(f_vec_tmp1, f_mat_tmp1);

  math_skewsym(f_lvrm_b, f_mat_tmp2);
  math_matmul(*f_rotmat_b2n, *f_mat_tmp2, 3, 3, 3, *f_mat_tmp3);
#else
  math_skewsym(f_winn, f_mat_tmp1);
  math_matmul(*f_mat_tmp1, *f_lvrm_n_x, 3, 3, 3, *f_mat_tmp2);

  math_skewsym(f_lvrm_b, f_mat_tmp1);
  math_matmul(*f_rotmat_b2n, *f_mat_tmp1, 3, 3, 3, *f_mat_tmp3);
  math_matmul(*f_mat_tmp3, f_cmp_gyro, 3, 3, 1, f_vec_tmp1);	
  math_skewsym(f_vec_tmp1, f_mat_tmp4);

  math_matadd(*f_mat_tmp4, *f_mat_tmp2, 3, 3, *f_mat_tmp1);
#endif

  float  f_mat_n2e[3][3] = { 0.f };
  tf_cn2ebylla(d_ins2bds_poslla, f_mat_n2e);
#if 0
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i) /* walk through all used sat, calculate the residual */
  {
    /* calculate distance between sat and reciver */
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

    nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    diffrange[i] = nowrange;

    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_refpos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_refpos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_refpos.z;

    nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    diffrangegnss[i] = nowrange;

    Range_Res[i] = pLEKF->ledr.G_psr_data[i].P - diffrangegnss[i];
  }
#endif

#define	TC_SAT_SORT
#define SAT_NUM 40

#ifdef TC_SAT_SORT
  float* f_s_pr_lamida = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_pr_noise = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_pr_z = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_pr_H = (float*)OS_MALLOC(SAT_NUM * u_sysdim * sizeof(float));
  float* f_s_dr_lamida = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_dr_noise = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_dr_z = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_dr_H = (float*)OS_MALLOC(SAT_NUM * u_sysdim * sizeof(float));  

  for (uint32_t i = 0; i < pz_inav->u_psr_used_satnum; i++) /* walk through all use sat,cal lamda to select sat to filter */
  {
    f_s_pr_lamida[i] = 9999.9f;
    f_s_dr_lamida[i] = 9999.9f;
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
    if (pz_gnss->f_avgsnr < 30.0f && pz_psr_used_data[i].u_snr > 45)
    {
      continue;
    }
    f_tempvector1[0] = (float)(pz_psr_used_data[i].d_sat_pos[0] - d_ins2bds_pos_ecef[0]);
    f_tempvector1[1] = (float)(pz_psr_used_data[i].d_sat_pos[1] - d_ins2bds_pos_ecef[1]);
    f_tempvector1[2] = (float)(pz_psr_used_data[i].d_sat_pos[2] - d_ins2bds_pos_ecef[2]);

    f_diffrange = sqrtf(f_tempvector1[0] * f_tempvector1[0] + f_tempvector1[1] * f_tempvector1[1] + f_tempvector1[2] * f_tempvector1[2]);

    f_e_sr[0] = -f_tempvector1[0] / f_diffrange;
    f_e_sr[1] = -f_tempvector1[1] / f_diffrange;
    f_e_sr[2] = -f_tempvector1[2] / f_diffrange;

    if ((pz_psr_used_data[i].u_valid & MEAS_TYPE_PR) == MEAS_TYPE_PR)
    {
      float* f_mat_tmpx = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float f_mat_trans, f_lamida, f_mat_tmpb[3];
      float* f_measmat_tmp = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float f_z_tmp, f_noise;
      FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

      if (pz_psr_used_data[i].u_sys == INS_SYS_GPS || pz_psr_used_data[i].u_sys == INS_SYS_QZS)
      {
        f_z_tmp = (float)(pz_psr_used_data[i].d_pr - f_diffrange - pz_para->f_clk_err[0]);
        f_measmat_tmp[pz_algconfig->z_syserrtype.u_clock_err_esti] = -1.0;
        // noise = pLEKF->ledr.G_psr_data[i].prnoise + 10 * 10;
      }
      else if (pz_psr_used_data[i].u_sys == INS_SYS_BDS)
      {
        f_z_tmp = (float)(pz_psr_used_data[i].d_pr - f_diffrange - pz_para->f_clk_err[1]);
        f_measmat_tmp[pz_algconfig->z_syserrtype.u_clock_err_esti + 1] = -1.0;
        // noise = pLEKF->ledr.G_psr_data[i].prnoise + 150 * 150;
      }
      else if (pz_psr_used_data[i].u_sys == INS_SYS_GAL)
      {
        f_z_tmp = (float)(pz_psr_used_data[i].d_pr - f_diffrange - pz_para->f_clk_err[2]);
        f_measmat_tmp[pz_algconfig->z_syserrtype.u_clock_err_esti + 2] = -1.0;
        // noise = pLEKF->ledr.G_psr_data[i].prnoise + 150.f * 150;
      }
      else
      {
#ifdef _WIN32
        printf("Error\n");
#endif	
        continue;
      }
      if (pz_psr_used_data[i].f_prnoise < 100.0 && fabs(f_z_tmp) < 2.f/*30.f*/)
      {
        math_matmul(pf_e_sr, pf_n2e_poserr, 1, 3, 3, pf_esr_n2e);

        math_matmul(pf_esr_n2e, *f_lvrm_n_x, 1, 3, 3, f_mat_tmpb);

        f_noise = pz_psr_used_data[i].f_prnoise;
        f_noise = f_noise < 1.f ? 1.f : f_noise;

        f_measmat_tmp[0 + 0] = -1.0f * f_esr_n2e[0];
        f_measmat_tmp[0 + 1] = -1.0f * f_esr_n2e[1];
        f_measmat_tmp[0 + 2] = -1.0f * f_esr_n2e[2];

        f_measmat_tmp[6 + 0] = -1.0f * f_mat_tmpb[0];
        f_measmat_tmp[6 + 1] = -1.0f * f_mat_tmpb[1];
        f_measmat_tmp[6 + 2] = -1.0f * f_mat_tmpb[2];

        math_matmul(f_measmat_tmp, &(pz_model->f_syscovmat[0]), 1, u_sysdim, u_sysdim, f_mat_tmpx);
        math_matmul(f_mat_tmpx, f_measmat_tmp, 1, u_sysdim, 1, &f_mat_trans);
        f_mat_trans = 1.0f / (f_mat_trans + f_noise);
        f_lamida = fabsf(f_z_tmp * f_mat_trans * f_z_tmp);

        f_s_pr_lamida[i] = f_lamida;
        u_pr_num++;

        /* save for use */
        f_s_pr_z[i] = f_z_tmp;
        f_s_pr_noise[i] = f_noise;
        for (int k = 0; k < u_sysdim; ++k)
        {
          f_s_pr_H[i * u_sysdim + k] = f_measmat_tmp[k];
        }
      }
      OS_FREE(f_mat_tmpx);
      OS_FREE(f_measmat_tmp);
    }
    if ((pz_psr_used_data[i].u_valid & MEAS_TYPE_DR) == MEAS_TYPE_DR)
    {
      float f_att_mat[3], f_bias_mat[3], f_noise;
      float f_ins_vel_ecef[3];
      float f_det_vel[3], f_caldoppler;
      float f_mat_trans, f_lamida, f_z_tmp;
      float* f_mat_tmpx = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float* f_measmat_tmp = (float*)OS_MALLOC(u_sysdim * sizeof(float));
                
      math_matmul(*f_mat_n2e, pz_node->f_vel, 3, 3, 1, f_ins_vel_ecef);

      f_det_vel[0] = (float)(f_ins_vel_ecef[0] - pz_psr_used_data[i].d_sat_vel[0]);
      f_det_vel[1] = (float)(f_ins_vel_ecef[1] - pz_psr_used_data[i].d_sat_vel[1]);
      f_det_vel[2] = (float)(f_ins_vel_ecef[2] - pz_psr_used_data[i].d_sat_vel[2]);

      f_caldoppler = f_e_sr[0] * f_det_vel[0] + f_e_sr[1] * f_det_vel[1] + f_e_sr[2] * f_det_vel[2];
      f_z_tmp = (float)(pz_psr_used_data[i].d_dr - f_caldoppler); /* to be completed */
      if (pz_psr_used_data[i].f_drnoise < 1.f && fabs(f_z_tmp) < 0.1f)
      {
        math_matmul(f_e_sr, *f_mat_n2e, 1, 3, 3, f_esr_n2e);

        math_matmul(f_esr_n2e, *f_mat_tmp1, 1, 3, 3, f_att_mat);

        math_matmul(f_esr_n2e, *f_mat_tmp3, 1, 3, 3, f_bias_mat);

        f_measmat_tmp[3 + 0] = -1.0f * f_esr_n2e[0];
        f_measmat_tmp[3 + 1] = -1.0f * f_esr_n2e[1];
        f_measmat_tmp[3 + 2] = -1.0f * f_esr_n2e[2];

        f_measmat_tmp[6 + 0] = -1.0f * f_att_mat[0];
        f_measmat_tmp[6 + 1] = -1.0f * f_att_mat[1];
        f_measmat_tmp[6 + 2] = -1.0f * f_att_mat[2];

        f_measmat_tmp[9 + 0] = f_bias_mat[0];
        f_measmat_tmp[9 + 1] = f_bias_mat[1];
        f_measmat_tmp[9 + 2] = f_bias_mat[2];

        math_matmul(f_measmat_tmp, &(pz_model->f_syscovmat[0]), 1, u_sysdim, u_sysdim, f_mat_tmpx);
        math_matmul(f_mat_tmpx, f_measmat_tmp, 1, u_sysdim, 1, &f_mat_trans);
        f_noise = pz_psr_used_data[i].f_drnoise / 10.0f;
        f_noise = f_noise < 0.001f ? 0.001f : f_noise;
        f_mat_trans = 1.0f / (f_mat_trans + f_noise);
        f_lamida = fabsf(f_z_tmp * f_mat_trans * f_z_tmp);

        f_s_dr_lamida[i] = f_lamida;
        u_dr_num++;
        /* save for use */
        f_s_dr_z[i] = f_z_tmp;
        f_s_dr_noise[i] = f_noise;
        for (int k = 0; k < u_sysdim; ++k)
        {
          f_s_dr_H[i * u_sysdim + k] = f_measmat_tmp[k];
        }
      }
      OS_FREE(f_mat_tmpx);
      OS_FREE(f_measmat_tmp);
    }
  }
#if 0
  /* sort lamida */
  float temp;
  int temp2;
  int s_pr_lamida_satinnum[MAX_R_LEKF * 3];
  int s_dr_lamida_satinnum[MAX_R_LEKF * 3];
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    s_pr_lamida_satinnum[i] = i;
    s_dr_lamida_satinnum[i] = i;
  }
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    for (int j = 0; j < pLEKF->ledr.Gpsr_used_satnum - i - 1; ++j)
    {
      if (s_pr_lamida[j] > s_pr_lamida[j + 1])
      {
        temp = s_pr_lamida[j];
        s_pr_lamida[j] = s_pr_lamida[j + 1];
        s_pr_lamida[j + 1] = temp;

        temp2 = s_pr_lamida_satinnum[j];
        s_pr_lamida_satinnum[j] = s_pr_lamida_satinnum[j + 1];
        s_pr_lamida_satinnum[j + 1] = temp2;
      }
    }
  }
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    for (int j = 0; j < pLEKF->ledr.Gpsr_used_satnum - i - 1; ++j)
    {
      if (s_dr_lamida[j] > s_dr_lamida[j + 1])
      {
        temp = s_dr_lamida[j];
        s_dr_lamida[j] = s_dr_lamida[j + 1];
        s_dr_lamida[j + 1] = temp;

        temp2 = s_dr_lamida_satinnum[j];
        s_dr_lamida_satinnum[j] = s_dr_lamida_satinnum[j + 1];
        s_dr_lamida_satinnum[j + 1] = temp2;
      }
    }
  }
  /* init default parameter here */
  uint8_t mnc_dr = (dr_num < 8) ? dr_num : 8;
  uint8_t mnc_pr = (pr_num < 8) ? pr_num : 8;
#endif

#if 1	/* sort pr&dr lamida together */
  float f_s_prdr_lamida[SAT_NUM * 2];
  float f_temp;
  uint32_t u_temp2;
  uint32_t u_s_prdr_lamida_satinnum[SAT_NUM * 2];
  for (uint32_t i = 0; i < pz_inav->u_psr_used_satnum; ++i)
  {
    u_s_prdr_lamida_satinnum[i] = i;
    f_s_prdr_lamida[i] = f_s_pr_lamida[i];
  }
  for (uint32_t i = 0; i < pz_inav->u_psr_used_satnum; ++i)
  {
    u_s_prdr_lamida_satinnum[i + pz_inav->u_psr_used_satnum] = i + pz_inav->u_psr_used_satnum;
    f_s_prdr_lamida[i + pz_inav->u_psr_used_satnum] = f_s_dr_lamida[i];
  }
  for (uint32_t i = 0; i < (uint32_t)(pz_inav->u_psr_used_satnum * 2); ++i)
  {
    for (uint32_t j = 0; j < pz_inav->u_psr_used_satnum * 2 - i - 1; ++j)
    {
      if (f_s_prdr_lamida[j] > f_s_prdr_lamida[j + 1])
      {
        f_temp = f_s_prdr_lamida[j];
        f_s_prdr_lamida[j] = f_s_prdr_lamida[j + 1];
        f_s_prdr_lamida[j + 1] = f_temp;

        u_temp2 = u_s_prdr_lamida_satinnum[j];
        u_s_prdr_lamida_satinnum[j] = u_s_prdr_lamida_satinnum[j + 1];
        u_s_prdr_lamida_satinnum[j + 1] = u_temp2;
      }
    }
  }
  uint32_t u_mnc_total = (u_pr_num + u_dr_num <= u_meadim) ? (u_pr_num + u_dr_num) : u_meadim;
#if 0
  mnc_dr = 0;
  mnc_pr = 0;
  for (int i = 0; i < mnc_total; ++i)
  {
    (f_s_prdr_lamida_satinnum[i] < pLEKF->ledr.Gpsr_used_satnum) ? mnc_pr++ : mnc_dr++; /* sort pr&dr lamida together */
  }
  mnc_dr = (dr_num < mnc_dr) ? dr_num : mnc_dr;
  mnc_pr = (pr_num < mnc_pr) ? pr_num : mnc_pr;
#endif
#endif

  for (int i = 0; i < pz_inav->u_psr_used_satnum; ++i) /* walk through all use sat,find sat to filter */
  {
    int dr_lamida_k = -1;
    int pr_lamida_k = -1;

#if 0
    if (GnssMsg_GetGnssDataPointer()->avg_CN0 < 30 && pLEKF->ledr.G_psr_data[i].SNR > 45)
    {
      continue;
    }
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

    diffrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    e_sr[0] = -tempvector1[0] / diffrange;
    e_sr[1] = -tempvector1[1] / diffrange;
    e_sr[2] = -tempvector1[2] / diffrange;
#endif

#ifdef REF_READY
    float32 ref_los[3], ref_vel_ecef[3], Doppler_Res = 0;
    float32 ref_ins;
    asensing_rtk_meas_to_ins_pr* pRaw = TcinsCal_GetTcPsrGnssDataPointer();
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_refpos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_refpos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_refpos.z;
    diffrangegnss[i] = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
    ref_los[0] = -tempvector1[0] / diffrangegnss[i];
    ref_los[1] = -tempvector1[1] / diffrangegnss[i];//
    ref_los[2] = -tempvector1[2] / diffrangegnss[i];// 
    Range_Res[i] = pLEKF->ledr.G_psr_data[i].P - diffrangegnss[i];

    ref_ins = sqrtd((Ecef_refpos.x - Ecef_pos.x) * (Ecef_refpos.x - Ecef_pos.x) + (Ecef_refpos.y - Ecef_pos.y) * (Ecef_refpos.y - Ecef_pos.y) +
      (Ecef_refpos.z - Ecef_pos.z) * (Ecef_refpos.z - Ecef_pos.z));

    Lekf_matmmlt(ref_vel_ecef, *Cn2e_Ref, vel_ref, 3, 3, 1);

    float64 refDoppler, det_vel[3];
    det_vel[0] = ref_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
    det_vel[1] = ref_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
    det_vel[2] = ref_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

    refDoppler = ref_los[0] * det_vel[0] + ref_los[1] * det_vel[1] + ref_los[2] * det_vel[2];

    Doppler_Res = pLEKF->ledr.G_psr_data[i].D - refDoppler;

#endif
#if 0
    for (int si = 0; si < mnc_dr; ++si) /* control DR meas num here */
    {
      if (i == s_dr_lamida_satinnum[si])
      {
        dr_lamida_k = si;
      }
    }
    for (int si = 0; si < mnc_pr; ++si) /* control PR meas num here */
    {
      if (i == s_pr_lamida_satinnum[si])
      {
        pr_lamida_k = si;
      }
    }
#else
    for (uint32_t si = 0; si < u_mnc_total; ++si) /* control DR meas num here */
    {
      if (i == u_s_prdr_lamida_satinnum[si])
      {
        pr_lamida_k = si;
      }
      if (i == u_s_prdr_lamida_satinnum[si] - pz_inav->u_psr_used_satnum)
      {
        dr_lamida_k = si;
      }
    }
#endif
    if (pr_lamida_k >= 0)
    {
#if 1
      float* f_meamat_tmp = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float f_z_tmp, f_noise, f_lamida;
#else
      float32 mat_tmpx[NUM_X_LEKF], mat_trans, lamida;
      float32 Z_tmp, H_tmp[NUM_X_LEKF] = { 0 }, noise;
      float32 mat_tmpB[3];
      Lekf_matmmlt(pe_srCne, pe_sr, pCne, 1, 3, 3);

      Lekf_matmmlt(mat_tmpA, pe_srCne, *Dr_inv, 1, 3, 3);
      Lekf_matmmlt(mat_tmpB, mat_tmpA, *mat_tmp0, 1, 3, 3);

      if (pLEKF->ledr.G_psr_data[i].sys == SYS_GPS || pLEKF->ledr.G_psr_data[i].sys == SYS_QZS)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[0]);
        H_tmp[CLKDIFF_X_G_LEKF] = 1.0;
      }
      else if (pLEKF->ledr.G_psr_data[i].sys == SYS_CMP)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[1]);
        H_tmp[CLKDIFF_X_B_LEKF] = 1.0;
      }
      else if (pLEKF->ledr.G_psr_data[i].sys == SYS_GAL)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[2]);
        H_tmp[CLKDIFF_X_J_LEKF] = 1.0;
      }
      else
      {
#ifdef _WIN32
        printf("Error\n");
#endif
        continue;
      }
      noise = pLEKF->ledr.G_psr_data[i].prnoise * 4.0;
      noise = noise < 9.f ? 9.f : noise;
      H_tmp[POS_X_LEKF + 0] = -1.0 * e_srCne[0];
      H_tmp[POS_X_LEKF + 1] = -1.0 * e_srCne[1];
      H_tmp[POS_X_LEKF + 2] = -1.0 * e_srCne[2];

      H_tmp[ATT_X_LEKF + 0] = mat_tmpB[0];
      H_tmp[ATT_X_LEKF + 1] = mat_tmpB[1];
      H_tmp[ATT_X_LEKF + 2] = mat_tmpB[2];

      Lekf_matmmlt(mat_tmpx, H_tmp, *pLEKF->P, 1, NUM_X_LEKF, NUM_X_LEKF);

      Lekf_matmmlt(&mat_trans, mat_tmpx, H_tmp, 1, NUM_X_LEKF, 1);

      mat_trans = 1.0f / (mat_trans + noise);

      lamida = fabs(Z_tmp * mat_trans * Z_tmp);
#endif
      f_lamida = f_s_pr_lamida[i];
      f_z_tmp = f_s_pr_z[i];
      f_noise = f_s_pr_noise[i];
      if (f_lamida < 3.0f && pz_psr_used_data[i].f_prnoise < 100.0 && fabs(f_z_tmp) < 2.f)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        for (int k = 0; k < u_sysdim; ++k)
        {
          f_meamat_tmp[k] = f_s_pr_H[i * u_sysdim + k];
        }
        memcpy(&f_sysmeasmat[(u_mdim)*u_sysdim], f_meamat_tmp, u_sysdim * sizeof(float));
        f_z_meas[u_mdim] = f_z_tmp;
        f_measnoise[u_mdim * u_meadim + u_mdim] = f_noise;
        u_mdim++;
#endif

#if defined(_WIN32) 
#if defined(REF_READY)
        fprintf(fp_tight_log, "$GPTIP,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n",
          pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, (Range_Res[i]), (Z_tmp),
          ref_ins, pLEKF->ledr.G_psr_data[i].prnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#else
        LOGI(TAG_VDR, "$GPTIP,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f\n",
          pz_node->d_time, pz_psr_used_data[i].u_sys, pz_psr_used_data[i].u_prn,
          pz_psr_used_data[i].u_snr, 0.0, (f_z_tmp),
          0.0, pz_psr_used_data[i].f_prnoise, f_lamida, pz_psr_used_data[i].f_ele);
#endif
#endif
      }
      else
      {
#if defined(_WIN32) 
#if defined(REF_READY)
        fprintf(fp_tight_log, "$GPTIPX,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n",
          pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, (Range_Res[i]), (Z_tmp),
          ref_ins, pLEKF->ledr.G_psr_data[i].prnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#else
        LOGE(TAG_VDR, "$GPTIPX,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f\n",
          pz_node->d_time, pz_psr_used_data[i].u_sys, pz_psr_used_data[i].u_prn,
          pz_psr_used_data[i].u_snr, 0.0, (f_z_tmp),
          0.0, pz_psr_used_data[i].f_prnoise, f_lamida, pz_psr_used_data[i].f_ele);
#endif
#endif
      }
      OS_FREE(f_meamat_tmp);
    }

#if 1
    if (dr_lamida_k >= 0)
    {
#if 1
      float* f_meamat_tmp = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float f_z_tmp, f_noise, f_lamida;
#else
      float32 mat_A0[3], att_mat[3], bias_mat[3], noise;
      float32 ins_vel_ecef[3];
      float64 det_vel[3], calDoppler;

      float32 mat_tmpx[NUM_X_LEKF], mat_trans, lamida;
      float32 Z_tmp, H_tmp[NUM_X_LEKF] = { 0 };

      Lekf_matmmlt(ins_vel_ecef, *Cn2e, ins_vel, 3, 3, 1);

      det_vel[0] = ins_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
      det_vel[1] = ins_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
      det_vel[2] = ins_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

      calDoppler = e_sr[0] * det_vel[0] + e_sr[1] * det_vel[1] + e_sr[2] * det_vel[2];

      Lekf_matmmlt(mat_A0, e_sr, *Cn2e, 1, 3, 3);

      Lekf_matmmlt(att_mat, mat_A0, *mat_tmp1, 1, 3, 3);

      Lekf_matmmlt(bias_mat, mat_A0, *mat_tmp3, 1, 3, 3);


      H_tmp[V_X_LEKF + 0] = -1.0 * mat_A0[0];
      H_tmp[V_X_LEKF + 1] = -1.0 * mat_A0[1];
      H_tmp[V_X_LEKF + 2] = -1.0 * mat_A0[2];

      H_tmp[ATT_X_LEKF + 0] = -1.0 * att_mat[0];
      H_tmp[ATT_X_LEKF + 1] = -1.0 * att_mat[1];
      H_tmp[ATT_X_LEKF + 2] = -1.0 * att_mat[2];

      H_tmp[BG_X_LEKF + 0] = -1.0 * bias_mat[0];
      H_tmp[BG_X_LEKF + 1] = -1.0 * bias_mat[1];
      H_tmp[BG_X_LEKF + 2] = -1.0 * bias_mat[2];

      Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].D - calDoppler); /* to be completed */

      Lekf_matmmlt(mat_tmpx, H_tmp, *pLEKF->P, 1, NUM_X_LEKF, NUM_X_LEKF);

      Lekf_matmmlt(&mat_trans, mat_tmpx, H_tmp, 1, NUM_X_LEKF, 1);

      noise = pLEKF->ledr.G_psr_data[i].drnoise / 10;

      noise = noise < 0.001f ? 0.001f : noise;

      mat_trans = 1.0f / (mat_trans + noise);

      lamida = fabs(Z_tmp * mat_trans * Z_tmp);
#endif
      f_lamida = f_s_dr_lamida[i];
      f_z_tmp = f_s_dr_z[i];
      f_noise = f_s_dr_noise[i];
      if (f_lamida < 0.5 && pz_psr_used_data[i].f_drnoise < 1.f && fabs(f_z_tmp) < 0.1f)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        for (int k = 0; k < u_sysdim; ++k)
        {
          f_meamat_tmp[k] = f_s_dr_H[i * u_sysdim + k];
        }
        memcpy(&f_sysmeasmat[(u_mdim)*u_sysdim], f_meamat_tmp, u_sysdim * sizeof(float));
        f_z_meas[u_mdim] = f_z_tmp;
        f_measnoise[u_mdim * u_meadim + u_mdim] = f_noise;
        u_mdim++;
#endif

#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
#if defined(REF_READY)
        fprintf(fp_tight_log, "$GPTID,%.3f,sys:%d,prn:%d,cn0:%d,ref:%.3f,ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n",
          pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, Doppler_Res, (Z_tmp),
          pLEKF->ledr.G_psr_data[i].drnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#else
        LOGI(TAG_VDR, "$GPTID,%.3f,sys:%d,prn:%d,cn0:%d,ref-D:%.3f,ins-D:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f\n",
          pz_node->d_time, pz_psr_used_data[i].u_sys, pz_psr_used_data[i].u_prn,
          pz_psr_used_data[i].u_snr, 0.0, (f_z_tmp),
          0.0, pz_psr_used_data[i].f_drnoise, f_lamida, pz_psr_used_data[i].f_ele);
#endif
#endif
      }
      else
      {
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
#if defined(REF_READY)
        fprintf(fp_tight_log, "$GPTIDX,%.3f,sys:%d,prn:%d,cn0:%d,ref:%.3f,ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n",
          pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, Doppler_Res, (Z_tmp),
          pLEKF->ledr.G_psr_data[i].drnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#else
        LOGE(TAG_VDR, "$GPTIDX,%.3f,sys:%d,prn:%d,cn0:%d,ref-D:%.3f,ins-D:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f\n",
          pz_node->d_time, pz_psr_used_data[i].u_sys, pz_psr_used_data[i].u_prn,
          pz_psr_used_data[i].u_snr, 0.0, (f_z_tmp),
          0.0, pz_psr_used_data[i].f_drnoise, f_lamida, pz_psr_used_data[i].f_ele);
#endif
#endif
      }
      OS_FREE(f_meamat_tmp);
    }
#endif
#ifndef SEQ_UPDATE
    if (u_mdim >= u_meadim)
    {
      break;
    }
#endif
  }
  OS_FREE(f_s_pr_lamida); 
  OS_FREE(f_s_pr_noise); 
  OS_FREE(f_s_pr_z);
  OS_FREE(f_s_pr_H); 
  OS_FREE(f_s_dr_lamida); 
  OS_FREE(f_s_dr_noise);
  OS_FREE(f_s_dr_z); 
  OS_FREE(f_s_dr_H);
#else
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i) /* walk through all use sat,find sat to filter */
  {
    chooseflag[i] = 0;

    if (GnssMsg_GetGnssDataPointer()->avg_CN0 < 30 && pLEKF->ledr.G_psr_data[i].SNR > 45)
    {
        continue;
    }
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

    diffrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    e_sr[0] = -tempvector1[0] / diffrange;
    e_sr[1] = -tempvector1[1] / diffrange;
    e_sr[2] = -tempvector1[2] / diffrange;
#ifdef REF_READY
    float32 ref_los[3], ref_vel_ecef[3], Doppler_Res = 0;
    float32 ref_ins;
    asensing_rtk_meas_to_ins_pr* pRaw = TcinsCal_GetTcPsrGnssDataPointer();
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_refpos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_refpos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_refpos.z;
    diffrangegnss[i] = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
    ref_los[0] = -tempvector1[0] / diffrangegnss[i];
    ref_los[1] = -tempvector1[1] / diffrangegnss[i];
    ref_los[2] = -tempvector1[2] / diffrangegnss[i];
    Range_Res[i] = pLEKF->ledr.G_psr_data[i].P - diffrangegnss[i];

    ref_ins = sqrtd((Ecef_refpos.x - Ecef_pos.x) * (Ecef_refpos.x - Ecef_pos.x) + (Ecef_refpos.y - Ecef_pos.y) * (Ecef_refpos.y - Ecef_pos.y) +
      (Ecef_refpos.z - Ecef_pos.z) * (Ecef_refpos.z - Ecef_pos.z));

    Lekf_matmmlt(ref_vel_ecef, *Cn2e_Ref, vel_ref, 3, 3, 1);

    float64 refDoppler, det_vel[3];
    det_vel[0] = ref_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
    det_vel[1] = ref_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
    det_vel[2] = ref_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

    refDoppler = ref_los[0] * det_vel[0] + ref_los[1] * det_vel[1] + ref_los[2] * det_vel[2];

    Doppler_Res = pLEKF->ledr.G_psr_data[i].D - refDoppler;

#endif

#if 1
    if ((pLEKF->ledr.G_psr_data[i].prvalid & MEAS_TYPE_DR) == MEAS_TYPE_DR)
    {
      float32 mat_A0[3], att_mat[3], bias_mat[3], noise;
      float32 ins_vel_ecef[3];
      float64 det_vel[3], calDoppler;

      float32 mat_tmpx[NUM_X_LEKF], mat_trans, lamida;
      float32 Z_tmp, H_tmp[NUM_X_LEKF] = { 0 };

      Lekf_matmmlt(ins_vel_ecef, *Cn2e, ins_vel, 3, 3, 1);

      det_vel[0] = ins_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
      det_vel[1] = ins_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
      det_vel[2] = ins_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

      calDoppler = e_sr[0] * det_vel[0] + e_sr[1] * det_vel[1] + e_sr[2] * det_vel[2];

      Lekf_matmmlt(mat_A0, e_sr, *Cn2e, 1, 3, 3);

      Lekf_matmmlt(att_mat, mat_A0, *mat_tmp1, 1, 3, 3);

      Lekf_matmmlt(bias_mat, mat_A0, *mat_tmp3, 1, 3, 3);

      H_tmp[V_X_LEKF + 0] = -1.0 * mat_A0[0];
      H_tmp[V_X_LEKF + 1] = -1.0 * mat_A0[1];
      H_tmp[V_X_LEKF + 2] = -1.0 * mat_A0[2];

      H_tmp[ATT_X_LEKF + 0] = -1.0 * att_mat[0];
      H_tmp[ATT_X_LEKF + 1] = -1.0 * att_mat[1];
      H_tmp[ATT_X_LEKF + 2] = -1.0 * att_mat[2];

      H_tmp[BG_X_LEKF + 0] = -1.0 * bias_mat[0];
      H_tmp[BG_X_LEKF + 1] = -1.0 * bias_mat[1];
      H_tmp[BG_X_LEKF + 2] = -1.0 * bias_mat[2];

      Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].D - calDoppler); /* to be completed */

      Lekf_matmmlt(mat_tmpx, H_tmp, *pLEKF->P, 1, NUM_X_LEKF, NUM_X_LEKF);

      Lekf_matmmlt(&mat_trans, mat_tmpx, H_tmp, 1, NUM_X_LEKF, 1);

      noise = pLEKF->ledr.G_psr_data[i].drnoise / 10;

      noise = noise < 0.001f ? 0.001f : noise;

      mat_trans = 1.0f / (mat_trans + noise);

      lamida = fabs(Z_tmp * mat_trans * Z_tmp);

      if (lamida < 0.5 && pLEKF->ledr.G_psr_data[i].drnoise < 1.f)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        memcpy(&pH[(NumR + available_num) * NUM_X_LEKF], H_tmp, sizeof(H_tmp));
        pZ[available_num + NumR] = Z_tmp;
        RDiag[NumR + available_num] = noise;
        available_num++;
#endif

#if defined(_WIN32) && defined(REF_READY)
        fprintf(fp_tight_log, "$GPTID,%.3f,sys:%d,prn:%d,cn0:%d,ref:%.3f,ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n", pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, Doppler_Res, (Z_tmp),
          pLEKF->ledr.G_psr_data[i].drnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#endif
      }
    }
#endif	
#if 1		
    if ((pLEKF->ledr.G_psr_data[i].prvalid & MEAS_TYPE_PR) == MEAS_TYPE_PR)
    {
      float32 mat_tmpx[NUM_X_LEKF], mat_trans, lamida;
      float32 Z_tmp, H_tmp[NUM_X_LEKF] = { 0 }, noise;
      float32 mat_tmpB[3];
      Lekf_matmmlt(pe_srCne, pe_sr, pCne, 1, 3, 3);

      Lekf_matmmlt(mat_tmpA, pe_srCne, *Dr_inv, 1, 3, 3);
      Lekf_matmmlt(mat_tmpB, mat_tmpA, *mat_tmp0, 1, 3, 3);

      if (pLEKF->ledr.G_psr_data[i].sys == SYS_GPS || pLEKF->ledr.G_psr_data[i].sys == SYS_QZS)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[0]);
        H_tmp[CLKDIFF_X_G_LEKF] = 1.0;
      }
      else if (pLEKF->ledr.G_psr_data[i].sys == SYS_CMP)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[1]);
        H_tmp[CLKDIFF_X_B_LEKF] = 1.0;
      }
      else if (pLEKF->ledr.G_psr_data[i].sys == SYS_GAL)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[2]);
        H_tmp[CLKDIFF_X_J_LEKF] = 1.0;
      }
      else
      {
#ifdef _WIN32
        printf("Error\n");
#endif	
        continue;
      }
      noise = pLEKF->ledr.G_psr_data[i].prnoise;
      noise = noise < 9.f ? 9.f : noise;
      H_tmp[POS_X_LEKF + 0] = -1.0 * e_srCne[0];
      H_tmp[POS_X_LEKF + 1] = -1.0 * e_srCne[1];
      H_tmp[POS_X_LEKF + 2] = -1.0 * e_srCne[2];

      H_tmp[ATT_X_LEKF + 0] = mat_tmpB[0];
      H_tmp[ATT_X_LEKF + 1] = mat_tmpB[1];
      H_tmp[ATT_X_LEKF + 2] = mat_tmpB[2];

      Lekf_matmmlt(mat_tmpx, H_tmp, *pLEKF->P, 1, NUM_X_LEKF, NUM_X_LEKF);

      Lekf_matmmlt(&mat_trans, mat_tmpx, H_tmp, 1, NUM_X_LEKF, 1);

      mat_trans = 1.0f / (mat_trans + noise);

      lamida = fabs(Z_tmp * mat_trans * Z_tmp);

      if (lamida < 3.0f && pLEKF->ledr.G_psr_data[i].prnoise < 100 && fabs(Z_tmp) < 5.f)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        memcpy(&pH[(NumR + available_num) * NUM_X_LEKF], H_tmp, sizeof(H_tmp));
        pZ[available_num + NumR] = Z_tmp;
        RDiag[NumR + available_num] = noise;
        available_num++;
#endif

#if defined(_WIN32) && defined(REF_READY)
        fprintf(fp_tight_log, "$GPTIP,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n", pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, (Range_Res[i]), (Z_tmp),
          ref_ins, pLEKF->ledr.G_psr_data[i].prnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#endif
      }
    }
#endif
#ifndef SEQ_UPDATE
      if (available_num + NumR >= NUM_X_LEKF - 7)
      {
        break;
      }
#endif
  }
#endif

#if 0
#if 1
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; i++)
  {
    switch (pLEKF->ledr.G_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout_txt, "%c%d %lf %lf %d %lf ", sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].P, diffrange[i], chooseflag[i], pLEKF->ledr.G_psr_data[i].prnoise);
  }
#endif
  for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; ++i) /* walk through all used sat, calculate the residual */
  {
    tempvector1[0] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
    tempvector1[1] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
    tempvector1[2] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

    nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    diffrange[i] = nowrange;

    tempvector1[0] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[0] - Ecef_gnsspos.x;
    tempvector1[1] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[1] - Ecef_gnsspos.y;
    tempvector1[2] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[2] - Ecef_gnsspos.z;

    nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    diffrangegnss[i] = nowrange;

    Range_Res[i] = pLEKF->ledr.C_psr_data[i].P - diffrange[i];
  }
  midnum[1] = getMidd(&Sortdiffnum[0], &Range_Res[0], pLEKF->ledr.Cpsr_used_satnum);

  if (available_num < 5)
  {
    for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; ++i) /* walk through all use sat,find sat to filter */
    {
        chooseflag[i] = 0;
      if (((Range_Res[i] < 8.0) || (pLEKF->ledr.Cpsr_used_satnum <= 5)) && (Range_Res[i] < Sortdiffnum[MIN(pLEKF->ledr.Cpsr_used_satnum - 1, 5)]) && (fabs(pLEKF->ledr.C_psr_data[i].P - diffrange[i]) < 20.0))
      {
        chooseflag[i] = 1;
        tempvector1[0] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
        tempvector1[1] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
        tempvector1[2] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

        e_sr[0] = -tempvector1[0] / sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
        e_sr[1] = -tempvector1[1] / sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
        e_sr[2] = -tempvector1[2] / sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

        Lekf_matmmlt(pe_srCne, pe_sr, pCne, 1, 3, 3);

        pZ[available_num + NumR] = (float)(pLEKF->ledr.C_psr_data[i].P - diffrange[i] - pIMUS->Clk_diff); /* to be completed */

        pH[(NumR + available_num) * NUM_X_LEKF + POS_X_LEKF + 0] = -1.0 * e_srCne[0];
        pH[(NumR + available_num) * NUM_X_LEKF + POS_X_LEKF + 1] = -1.0 * e_srCne[1];
        pH[(NumR + available_num) * NUM_X_LEKF + POS_X_LEKF + 2] = -1.0 * e_srCne[2];
        pH[(NumR + available_num) * NUM_X_LEKF + CLKDIFF_X_LEKF] = -1.0;

        tempR = 15.5;

        RDiag[NumR + available_num] = tempR * tempR; /* how to confirm */
        available_num++;
      }
    }
  }
#if 1
  for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; i++) {
    switch (pLEKF->ledr.C_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout_txt, "%c%d %lf %lf %d %lf ", sys, pLEKF->ledr.C_psr_data[i].prn, pLEKF->ledr.C_psr_data[i].P, diffrange[i], chooseflag[i], pLEKF->ledr.C_psr_data[i].prnoise);
  }
  fprintf(Debugmesout_txt, "%lf %lf %lf %d %lf", midnum[0], midnum[1], pIMUS->Clk_diff, available_num, rtkmeasPsr->cur_tow);
  fprintf(Debugmesout_txt, "\n");

  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; i++) {
    switch (pLEKF->ledr.G_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout3_txt, "%c%d ", sys, pLEKF->ledr.G_psr_data[i].prn);
  }
  for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; i++) {
    switch (pLEKF->ledr.C_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout3_txt, "%c%d ", sys, pLEKF->ledr.C_psr_data[i].prn);
  }
  fprintf(Debugmesout3_txt, "\n");

#endif
#endif
#if 0
  fprintf(Debugmesout2_txt, "%10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %lf %d %lf %lf %lf\n", rtkmeas->cur_BLH[0], rtkmeas->cur_BLH[1], rtkmeas->cur_BLH[2],
    rtkmeasPsr->pvt_blh[0], rtkmeasPsr->pvt_blh[1], rtkmeasPsr->pvt_blh[2], (double)(Llh_pos.dlat * RAD2DEG), (double)(Llh_pos.dlon * RAD2DEG), Llh_pos.dhigh, AttOld[2], available_num, pIMUS->Clk_diff, midnum[0], rtkmeas->cur_tow);
#endif	

  if (u_mdim > 0)
  {
    ins_SysErrModData_t z_ekfData = {0};

    z_ekfData.sys_err_model.f_sysnoise = (float*)OS_MALLOC(u_sysdim * sizeof(float));
    z_ekfData.sys_err_model.f_syscovmat = (float*)OS_MALLOC(u_sysdim * u_sysdim * sizeof(float));
    z_ekfData.sys_err_model.f_measnoise = (float*)OS_MALLOC(u_meadim * u_meadim * sizeof(float));
    z_ekfData.sys_err_model.f_sysmeasmat = (float*)OS_MALLOC(u_meadim * u_sysdim * sizeof(float));
    z_ekfData.sys_err_model.f_errstate = (float*)OS_MALLOC(u_sysdim * sizeof(float));
    z_ekfData.sys_err_model.f_systrsmat = (float*)OS_MALLOC(u_sysdim * u_sysdim * sizeof(float));
    z_ekfData.sys_err_model.f_systrsmat_tdcp = (float*)OS_MALLOC(u_sysdim * u_sysdim * sizeof(float));
    z_ekfData.sys_err_model.f_systrsmat_tdcp_save = (float*)OS_MALLOC(u_sysdim * u_sysdim * sizeof(float));
    z_ekfData.f_z_meas = (float*)OS_MALLOC(u_meadim * sizeof(float));
        
    z_ekfData.u_sysDim = u_sysdim;
    z_ekfData.u_measDim = u_mdim;
    z_ekfData.sys_err_model.u_meadim = pz_model->u_meadim;
    memcpy(z_ekfData.f_z_meas, f_z_meas, u_meadim * sizeof(float));
    math_matcopy(&(z_ekfData.sys_err_model.f_sysmeasmat[0]), &(f_sysmeasmat[0]), u_mdim, u_sysdim);
    memcpy(z_ekfData.sys_err_model.f_measnoise, f_measnoise, sizeof(float) * u_meadim* u_meadim);
    memcpy(z_ekfData.sys_err_model.f_syscovmat, pz_model->f_syscovmat, sizeof(float) * u_sysdim* u_sysdim);
    ekf_api_SysErrModData_Put(&z_ekfData);

    OS_FREE(z_ekfData.sys_err_model.f_sysnoise);
    OS_FREE(z_ekfData.sys_err_model.f_syscovmat);
    OS_FREE(z_ekfData.sys_err_model.f_measnoise);
    OS_FREE(z_ekfData.sys_err_model.f_sysmeasmat);
    OS_FREE(z_ekfData.sys_err_model.f_errstate);
    OS_FREE(z_ekfData.sys_err_model.f_systrsmat);
    OS_FREE(z_ekfData.sys_err_model.f_systrsmat_tdcp);
    OS_FREE(z_ekfData.sys_err_model.f_systrsmat_tdcp_save);
    OS_FREE(z_ekfData.f_z_meas);
  }
  else
  {
    u_bds_update = INS_FALSE;
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
  }

  OS_FREE(f_z_meas); 
  OS_FREE(f_measnoise); 
  OS_FREE(f_sysmeasmat);
    
  return u_bds_update;
}

#if 0
void get_transmat_block(static float f_mat_input[SYS_DIM][SYS_DIM], uint8_t m, uint8_t n, float f_mat_ret[3][3])
{
  for (uint8_t i = 0; i < 3; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
      f_mat_ret[i][j] = f_mat_input[m+i][n+j];
    }		
  }
}
#endif

uint8_t tight_couple_cp_precheck(AsensingGNSSCpMeas_t* pz_cp_paras, InertialNav_t* pz_inav)
{
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  MechanNode_t* pz_node = pz_obs->pz_bds_node;
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();

  float f_v_hor_cons_check = 0.0, f_v_alt_cons_check = 0.0;
  float f_ds_alt = 0.0, f_ds_hor = 0.0;
#if 1
  static float f_last_lat_deg = 0.0;
  static float f_last_lon_deg = 0.0;
  static float f_last_alt = 0.0;
  static float f_last_tow = 0.0;
  float f_v_mean_hor = 0.0, f_v_mean_alt = 0.0;
  float f_ds_n = 0.0;
  float f_ds_e = 0.0;
  float f_dt = 0.0;
  static float f_last_hor_speed = 0.0;
  static float f_last_alt_speed = 0.0;
  float f_hor_speed = 0.0;

  f_dt =(float)(pz_gnss->d_tow/1000.0 - f_last_tow);
  f_dt = (f_dt < -604879.0) ? f_dt + 604800 : f_dt;
  if (f_last_lat_deg > 0.0 && f_last_lon_deg > 0.0 && f_dt < 0.5 && f_dt >0.01)
  {
    f_hor_speed = sqrtf(pz_gnss->f_vn * pz_gnss->f_vn + pz_gnss->f_ve * pz_gnss->f_ve);
    f_v_mean_hor = (f_hor_speed + f_last_hor_speed) / 2.0f;
    f_v_mean_alt = (-pz_gnss->f_vd + f_last_alt_speed) / 2.0f;
    f_ds_n = (float)((pz_gnss->d_lat - f_last_lat_deg) * DEG2RAD * (pz_node->d_rm + pz_node->d_lla[2]));
    f_ds_e = (float)((pz_gnss->d_lon - f_last_lon_deg) * DEG2RAD * (pz_node->d_rn + pz_node->d_lla[2]) * cos(f_last_lat_deg * DEG2RAD));
    f_ds_alt = (pz_gnss->f_alt - f_last_alt);
    f_ds_hor = sqrtf(f_ds_n * f_ds_n + f_ds_e * f_ds_e);

    f_v_hor_cons_check = f_ds_hor / f_dt - f_v_mean_hor;
    f_v_alt_cons_check = f_ds_alt / f_dt - f_v_mean_alt;
  }
  f_last_lat_deg = (float)pz_gnss->d_lat;
  f_last_lon_deg = (float)pz_gnss->d_lon;
  f_last_alt = pz_gnss->f_alt;
  f_last_tow = (float)(pz_gnss->d_tow/1000.0);
  f_last_hor_speed = f_hor_speed;
  f_last_alt_speed = -pz_gnss->f_vd;
#endif
  double d_dt_psr_delay = pz_inav->pz_obs->pz_bds_node->d_time - pz_cp_paras->d_cur_tow;

  pz_inav->u_cp_used_satnum = 0;
  memset(pz_inav->z_cp_used_data, 0, sizeof(pz_inav->z_cp_used_data));
    
  static uint8_t u_first_flag = INS_TRUE;
  if (u_first_flag==INS_TRUE)
  {
      u_first_flag = INS_FALSE;
  }
  else
  {
    if ((pz_obs->u_posflag != TYPE_POS_FLAG_UNAVAILABLE) &&
      fabs(d_dt_psr_delay) < 0.35 && pz_gnss->f_avgsnr > 20.0 &&
      pz_cp_paras->e_diff_mask == ASG_TIME_DIFF_CARRIER)
    {
      for (int i = 0; i < pz_cp_paras->u_count; ++i)
      {
        if ((pz_cp_paras->z_raw_cp_data[i].u_valid > 0) && pz_cp_paras->z_raw_cp_data[i].u_snr >= 30 &&
          pz_cp_paras->z_raw_cp_data[i].f_ele >= 20.0)
        {
          memcpy(&(pz_inav->z_cp_used_data[pz_inav->u_cp_used_satnum]), &(pz_cp_paras->z_raw_cp_data[i]), sizeof(AsensingSatCpRaw_t));
          pz_inav->u_cp_used_satnum++;
        }
      }
      if (pz_inav->pz_obs->d_lla_prepps[0] < 0.001 || pz_inav->pz_obs->d_lla_prepps[1] < 0.001)
      {
        memcpy(pz_inav->pz_obs->d_lla_prepps, pz_inav->pz_obs->pz_bds_node->d_lla, sizeof(pz_inav->pz_obs->d_lla_prepps));
      }
      LOGI(TAG_VDR, "$GPGNC, % .3f, % .3f, % d, % .3f, % d, % .3f, % .3f, % .3f, % d, % .3f, % .3f, % .3f, % .3f\n",
        pz_inav->pz_obs->pz_bds_node->d_time, pz_cp_paras->d_cur_tow, pz_obs->u_posflag,
        pz_gnss->f_avgsnr, pz_gnss->u_satused,
        pz_gnss->f_latstd, pz_gnss->f_lonstd, pz_gnss->f_altstd, pz_inav->u_cp_used_satnum,
        f_ds_hor, f_ds_alt, f_v_hor_cons_check, f_v_alt_cons_check);
    }
    else
    {
      LOGE(TAG_VDR, "$GPGNCX, %.3f, %.3f, %d,%d, %.3f, %d, %.3f, %.3f, %.3f, %d, %.3f, %.3f, %.3f, %.3f\n",
        pz_inav->pz_obs->pz_bds_node->d_time, pz_cp_paras->d_cur_tow, pz_obs->u_posflag,
        pz_cp_paras->e_diff_mask,
        pz_gnss->f_avgsnr, pz_gnss->u_satused,
        pz_gnss->f_latstd, pz_gnss->f_lonstd, pz_gnss->f_altstd, pz_inav->u_psr_used_satnum,
        f_ds_hor, f_ds_alt, f_v_hor_cons_check, f_v_alt_cons_check);
    }
  }	
  return 0;
}

uint8_t tight_couple_cp_procedure(InertialNav_t* pz_inav, NavParams_t* pz_para)
{
  uint8_t u_bds_update = INS_TRUE;
  SyncObsNav_t* pz_obs = pz_inav->pz_obs;
  MechanNode_t* pz_node = pz_obs->pz_bds_node;
  AsensingSatCpRaw_t* pz_cp_used_data = pz_inav->z_cp_used_data;
  SysErrModel_t* pz_model = fusion_get_sysmodel();
  uint8_t u_meadim = pz_model->u_meadim;
  uint8_t u_sysdim = pz_model->u_sysdim;

  float* f_z_meas = (float*)OS_MALLOC(u_meadim * sizeof(float));
  float* f_measnoise = (float*)OS_MALLOC(u_meadim * u_meadim * sizeof(float));
  float* f_sysmeasmat = (float*)OS_MALLOC(u_meadim * u_sysdim * sizeof(float));
    
  uint8_t u_mdim = 0;
  pz_obs->u_kf_update = INS_FALSE;

  float f_n2e_poserr[3][3], * pf_n2e_poserr = (float*)(&f_n2e_poserr[0][0]);
  float f_n2e_poserr_pre[3][3], * pf_n2e_poserr_pre = (float*)(&f_n2e_poserr_pre[0][0]);	
  float f_e_sr_cur[3], * pf_e_sr_cur = (float*)(&f_e_sr_cur[0]);
  float f_esr_n2e_cur[3], * pf_esr_n2e_cur = (float*)(&f_esr_n2e_cur[0]);
  float f_e_sr_pre[3], * pf_e_sr_pre = (float*)(&f_e_sr_pre[0]);
  float f_esr_n2e_pre[3], * pf_esr_n2e_pre = (float*)(&f_esr_n2e_pre[0]);

  float f_rotmat_b2n[3][3];
  float f_att[3];

  float* pf_imu2gnss = NULL;
  float f_lvrm_n[3] = { 0.f };
  float f_lvrm_n_x[3][3] = { 0.f };

  double d_ins2bds_poslla[3] = { 0.0 };
  double d_ins2bds_poslla_pre[3] = { 0.0 };
  double d_ins2bds_pos_ecef[3] = { 0.0 };
  double d_ins2bds_pos_ecef_pre[3] = { 0.0 };
  uint8_t u_cp_num = 0;
  float f_diffrange, f_diffrange_pre, f_tmpvec_cur[3], f_tmpvec_pre[3];

  double d_rn, d_h, d_rn_h, d_rm_h;
  double d_sinlat, d_coslat, d_sinlon, d_coslon;

  d_rn = pz_node->d_rn;
  d_h = pz_obs->d_lla_prepps[2];
  d_rn_h = pz_node->d_rn + d_h;
  d_sinlat = sin(pz_obs->d_lla_prepps[0]);
  d_coslat = cos(pz_obs->d_lla_prepps[0]);
  d_sinlon = sin(pz_obs->d_lla_prepps[1]);
  d_coslon = cos(pz_obs->d_lla_prepps[1]);
  d_rm_h = pz_node->d_rm + d_h;

  f_n2e_poserr_pre[0][0] = (float)(-d_rn_h * d_sinlat * d_coslon / d_rm_h);
  f_n2e_poserr_pre[0][1] = (float)(-d_sinlon);
  f_n2e_poserr_pre[0][2] = (float)(-d_coslat * d_coslon);
  f_n2e_poserr_pre[1][0] = (float)(-d_rn_h * d_sinlat * d_sinlon / d_rm_h);
  f_n2e_poserr_pre[1][1] = (float)d_coslon;
  f_n2e_poserr_pre[1][2] = (float)(-d_coslat * d_sinlon);
  f_n2e_poserr_pre[2][0] = (float)((d_rn * (1 - EARTH_WIE) + d_h) * d_coslat / d_rm_h);
  f_n2e_poserr_pre[2][1] = 0.f;
  f_n2e_poserr_pre[2][2] = (float)(-d_sinlat);

  d_rn = pz_node->d_rn;
  d_h = pz_node->d_lla[2];
  d_rn_h = pz_node->d_rn + d_h;
  d_sinlat = sin(pz_node->d_lla[0]);
  d_coslat = cos(pz_node->d_lla[0]);
  d_sinlon = sin(pz_node->d_lla[1]);
  d_coslon = cos(pz_node->d_lla[1]);
  d_rm_h = pz_node->d_rm + d_h;
    
  f_n2e_poserr[0][0] = (float)(-d_rn_h * d_sinlat * d_coslon / d_rm_h);
  f_n2e_poserr[0][1] = (float)(-d_sinlon);
  f_n2e_poserr[0][2] = (float)(-d_coslat * d_coslon);
  f_n2e_poserr[1][0] = (float)(-d_rn_h * d_sinlat * d_sinlon / d_rm_h);
  f_n2e_poserr[1][1] = (float)d_coslon;
  f_n2e_poserr[1][2] = (float)(-d_coslat * d_sinlon);
  f_n2e_poserr[2][0] = (float)((d_rn * (1 - EARTH_WIE) + d_h) * d_coslat / d_rm_h);
  f_n2e_poserr[2][1] = 0.f;
  f_n2e_poserr[2][2] = (float)(-d_sinlat);

  f_att[0] = pz_para->f_att_rph[0];
  f_att[1] = pz_para->f_att_rph[1];
  f_att[2] = pz_para->f_att_rph[2];
  tf_euler2dcm(f_att, f_rotmat_b2n);

  pf_imu2gnss = lvrm_get_imu2gnss();
  math_matmul(*f_rotmat_b2n, pf_imu2gnss, 3, 3, 1, f_lvrm_n);
  math_skewsym(f_lvrm_n, f_lvrm_n_x);

  d_ins2bds_poslla[0] = pz_node->d_lla[0] + (double)(f_lvrm_n[0]) / (pz_node->d_rm + d_h);
  d_ins2bds_poslla[1] = pz_node->d_lla[1] + (double)(f_lvrm_n[1]) / ((pz_node->d_rn + d_h) * cos(d_ins2bds_poslla[0]));
  d_ins2bds_poslla[2] = pz_node->d_lla[2] + (double)(-f_lvrm_n[2]);
  tf_lla2ecef(d_ins2bds_poslla, d_ins2bds_pos_ecef, d_ins2bds_poslla);

  d_ins2bds_poslla_pre[0] = pz_obs->d_lla_prepps[0] + (double)(f_lvrm_n[0]) / (pz_node->d_rm + d_h);
  d_ins2bds_poslla_pre[1] = pz_obs->d_lla_prepps[1] + (double)(f_lvrm_n[1]) / ((pz_node->d_rn + d_h) * cos(d_ins2bds_poslla_pre[0]));
  d_ins2bds_poslla_pre[2] = pz_obs->d_lla_prepps[2] + (double)(-f_lvrm_n[2]);
  tf_lla2ecef(d_ins2bds_poslla_pre, d_ins2bds_pos_ecef_pre, d_ins2bds_poslla_pre);

#ifdef REF_READY
  T_LLH Ref_pos;
  float32 Cn2e_Ref[3][3], vel_ref[3];
  float64 PosLLARef[3], RefPosECEF[3];
  Ref_pos.dlat = gRefData.lat * DEG2RAD + Lntg[0] / pLEKF->pEP->Rnh;
  Ref_pos.dlon = gRefData.lon * DEG2RAD + Lntg[1] / pLEKF->pEP->RehcosL;
  Ref_pos.dhigh = gRefData.alt - Lntg[2];

  LLH2ECEF(&Ref_pos, &Ecef_refpos);

  PosLLARef[0] = Ref_pos.dlat;
  PosLLARef[1] = Ref_pos.dlon;
  PosLLARef[2] = Ref_pos.dhigh;

  vel_ref[0] = gRefData.vn;
  vel_ref[1] = gRefData.ve;
  vel_ref[2] = gRefData.vd;

  Convert_GetN2EDCMbyLLAPos(PosLLARef, Cn2e_Ref);
#endif	
        
#if 0
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
      tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
      tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
      tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

      nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

      diffrange[i] = nowrange;

      tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_refpos.x;
      tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_refpos.y;
      tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_refpos.z;

      nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

      diffrangegnss[i] = nowrange;

      Range_Res[i] = pLEKF->ledr.G_psr_data[i].P - diffrangegnss[i];
  }
#endif

#define	TC_SAT_SORT
#define SAT_NUM 40
#ifdef TC_SAT_SORT
  float* f_s_cp_lamida = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_cp_noise = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_cp_z = (float*)OS_MALLOC(SAT_NUM * sizeof(float));
  float* f_s_cp_H = (float*)OS_MALLOC(SAT_NUM * u_sysdim * sizeof(float));

  for (uint32_t i = 0; i < pz_inav->u_cp_used_satnum; i++)
  {
    f_s_cp_lamida[i] = 1.0e9;
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
    if (pz_gnss->f_avgsnr < 30.0 && pz_cp_used_data[i].u_snr > 45)
    {
      continue;
    }
    f_tmpvec_cur[0] = (float)(pz_cp_used_data[i].d_cur_sat_pos[0] - d_ins2bds_pos_ecef[0]);
    f_tmpvec_cur[1] = (float)(pz_cp_used_data[i].d_cur_sat_pos[1] - d_ins2bds_pos_ecef[1]);
    f_tmpvec_cur[2] = (float)(pz_cp_used_data[i].d_cur_sat_pos[2] - d_ins2bds_pos_ecef[2]);

    f_tmpvec_pre[0] = (float)(pz_cp_used_data[i].d_pre_sat_pos[0] - d_ins2bds_pos_ecef_pre[0]);
    f_tmpvec_pre[1] = (float)(pz_cp_used_data[i].d_pre_sat_pos[1] - d_ins2bds_pos_ecef_pre[1]);
    f_tmpvec_pre[2] = (float)(pz_cp_used_data[i].d_pre_sat_pos[2] - d_ins2bds_pos_ecef_pre[2]);

    f_diffrange = sqrtf(f_tmpvec_cur[0] * f_tmpvec_cur[0] + f_tmpvec_cur[1] * f_tmpvec_cur[1] + f_tmpvec_cur[2] * f_tmpvec_cur[2]);
    f_e_sr_cur[0] = -f_tmpvec_cur[0] / f_diffrange;
    f_e_sr_cur[1] = -f_tmpvec_cur[1] / f_diffrange;
    f_e_sr_cur[2] = -f_tmpvec_cur[2] / f_diffrange;

    f_diffrange_pre = sqrtf(f_tmpvec_pre[0] * f_tmpvec_pre[0] + f_tmpvec_pre[1] * f_tmpvec_pre[1] + f_tmpvec_pre[2] * f_tmpvec_pre[2]);
    f_e_sr_pre[0] = -f_tmpvec_pre[0] / f_diffrange_pre;
    f_e_sr_pre[1] = -f_tmpvec_pre[1] / f_diffrange_pre;
    f_e_sr_pre[2] = -f_tmpvec_pre[2] / f_diffrange_pre;

    float f_tdcp_ins = f_diffrange - f_diffrange_pre;

    if ((pz_cp_used_data[i].u_valid & 0x01) == 0x01)
    {
      float f_mat_trans, f_lamida, f_esr_n2e_lnx_cur[3], f_esr_n2e_lnx_pre[3];
      float f_z_tmp, f_noise;
      float* f_mat_tmpx = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float* f_measmat_tmp = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float* f_measmat_tmp1 = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float* f_measmat_tmp2 = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float* f_measmat_tmp3 = (float*)OS_MALLOC(u_sysdim * sizeof(float));
      float* f_measmat_tmp4 = (float*)OS_MALLOC(u_sysdim * sizeof(float));

      f_z_tmp = (float)(pz_cp_used_data[i].f_tdcp_obs - f_tdcp_ins - pz_para->f_clkdiff_err);

      if (fabsf(pz_cp_used_data[i].f_tdcp_res) < 10.0)
      {
        FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

#if 0
        get_transmat_block(pz_inav->pz_model->f_systrsmat_tdcp_save, 0, 0, f_tranmat_reverse_rr);
        get_transmat_block(pz_inav->pz_model->f_systrsmat_tdcp_save, 0, 3, f_tranmat_reverse_rv);
        get_transmat_block(pz_inav->pz_model->f_systrsmat_tdcp_save, 6, 3, f_tranmat_reverse_wv);
        get_transmat_block(pz_inav->pz_model->f_systrsmat_tdcp_save, 6, 6, f_tranmat_reverse_ww);

        math_matmul(pf_e_sr_cur, pf_n2e_poserr, 1, 3, 3, pf_esr_n2e_cur);
        math_matmul(pf_e_sr_pre, pf_n2e_poserr_pre, 1, 3, 3, pf_esr_n2e_pre);							
        math_matmul(pf_esr_n2e_pre, f_tranmat_reverse_rr, 1, 3, 3, f_esr_n2e_phi_pre);				
        math_matsub(pf_esr_n2e_cur, f_esr_n2e_phi_pre, 1, 3, f_pos_mat);

        math_matmul(pf_esr_n2e_cur, *f_lvrm_n_x, 1, 3, 3, f_esr_n2e_lnx_cur);
        math_matmul(pf_esr_n2e_pre, *f_lvrm_n_x, 1, 3, 3, f_esr_n2e_lnx_pre);
        math_matmul(f_esr_n2e_lnx_pre, f_tranmat_reverse_ww, 1, 3, 3, f_esr_n2e_lnx_phi_pre);					
        math_matsub(f_esr_n2e_lnx_cur, f_esr_n2e_lnx_phi_pre, 1, 3, f_att_mat);
                                        
        f_measmat_tmp[POS_ERR_ESTI + 0] = -1.0 * f_pos_mat[0];
        f_measmat_tmp[POS_ERR_ESTI + 1] = -1.0 * f_pos_mat[1];
        f_measmat_tmp[POS_ERR_ESTI + 2] = -1.0 * f_pos_mat[2];

        f_measmat_tmp[ATT_ERR_ESTI + 0] = -1.0 * f_att_mat[0];
        f_measmat_tmp[ATT_ERR_ESTI + 1] = -1.0 * f_att_mat[1];
        f_measmat_tmp[ATT_ERR_ESTI + 2] = -1.0 * f_att_mat[2];

#if 0 /* wrong, to be modified */
        math_matmul(pf_esr_n2e_pre, f_tranmat_reverse_rv, 1, 3, 3, f_temp_v1);			
        math_matmul(f_esr_n2e_lnx_pre, f_tranmat_reverse_wv, 1, 3, 3, f_temp_v2);
        math_matadd(f_temp_v1, f_temp_v2, 1, 3, f_vel_mat);
        f_measmat_tmp[VEL_ERR_ESTI + 0] = f_vel_mat[0];
        f_measmat_tmp[VEL_ERR_ESTI + 1] = f_vel_mat[1];
        f_measmat_tmp[VEL_ERR_ESTI + 2] = f_vel_mat[2];
#endif
#else
        math_matmul(pf_e_sr_cur, pf_n2e_poserr, 1, 3, 3, pf_esr_n2e_cur);
        math_matmul(pf_esr_n2e_cur, *f_lvrm_n_x, 1, 3, 3, f_esr_n2e_lnx_cur);
        f_measmat_tmp1[0 + 0] = f_esr_n2e_cur[0];
        f_measmat_tmp1[0 + 1] = f_esr_n2e_cur[1];
        f_measmat_tmp1[0 + 2] = f_esr_n2e_cur[2];
        f_measmat_tmp1[6 + 0] = f_esr_n2e_lnx_cur[0];
        f_measmat_tmp1[6 + 1] = f_esr_n2e_lnx_cur[1];
        f_measmat_tmp1[6 + 2] = f_esr_n2e_lnx_cur[2];

        math_matmul(pf_e_sr_pre, pf_n2e_poserr_pre, 1, 3, 3, pf_esr_n2e_pre);				
        math_matmul(pf_esr_n2e_pre, *f_lvrm_n_x, 1, 3, 3, f_esr_n2e_lnx_pre);
        f_measmat_tmp2[0 + 0] = f_esr_n2e_pre[0];
        f_measmat_tmp2[0 + 1] = f_esr_n2e_pre[1];
        f_measmat_tmp2[0 + 2] = f_esr_n2e_pre[2];
        f_measmat_tmp2[6 + 0] = f_esr_n2e_lnx_pre[0];
        f_measmat_tmp2[6 + 1] = f_esr_n2e_lnx_pre[1];
        f_measmat_tmp2[6 + 2] = f_esr_n2e_lnx_pre[2];

        math_matmul(f_measmat_tmp2, pz_inav->pz_model->f_systrsmat_tdcp_save, 1, u_sysdim, u_sysdim, f_measmat_tmp3);
        math_matsub(f_measmat_tmp3, f_measmat_tmp1, 1, u_sysdim, f_measmat_tmp);
#endif
        f_measmat_tmp[pz_algconfig->z_syserrtype.u_clock_diff_err_esti] = -1.0;

        f_noise = pz_cp_used_data[i].f_tdcp_res * pz_cp_used_data[i].f_tdcp_res *(50.0f * 50.0f);
        f_noise = f_noise < 0.03f * 0.03f ? 0.03f * 0.03f : f_noise;

        math_matmul(f_measmat_tmp, &(pz_model->f_syscovmat[0]), 1, u_sysdim, u_sysdim, f_mat_tmpx);
        math_matmul(f_mat_tmpx, f_measmat_tmp, 1, u_sysdim, 1, &f_mat_trans);
        f_mat_trans = 1.0f / (f_mat_trans + f_noise);
        f_lamida = fabsf(f_z_tmp * f_mat_trans * f_z_tmp);

        f_s_cp_lamida[i] = f_lamida;
        u_cp_num++;

        f_s_cp_z[i] = f_z_tmp;
        f_s_cp_noise[i] = f_noise;
        for (int k = 0; k < u_sysdim; ++k)
        {
          f_s_cp_H[i * u_sysdim + k] = f_measmat_tmp[k];
        }
      }
      else
      {
        LOGE(TAG_VDR, "error, %.3f, tdcp_res=%.3f, f_z_tmp=%.3f, u_cp_num=%d \n",
          pz_node->d_time, pz_cp_used_data[i].f_tdcp_res, f_z_tmp, u_cp_num);
      }
      OS_FREE(f_mat_tmpx);
      OS_FREE(f_measmat_tmp);
      OS_FREE(f_measmat_tmp1);
      OS_FREE(f_measmat_tmp2);
      OS_FREE(f_measmat_tmp3);
      OS_FREE(f_measmat_tmp4);
    }	
    else
    {
      LOGE(TAG_VDR, "error, %.3f, u_valid=%d, u_cp_num=%d \n",
        pz_node->d_time, pz_cp_used_data[i].u_valid, u_cp_num);
    }
  }

#if 0
  float temp;
  int temp2;
  int s_pr_lamida_satinnum[MAX_R_LEKF * 3];
  int s_dr_lamida_satinnum[MAX_R_LEKF * 3];
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    s_pr_lamida_satinnum[i] = i;
    s_dr_lamida_satinnum[i] = i;
  }
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    for (int j = 0; j < pLEKF->ledr.Gpsr_used_satnum - i - 1; ++j)
    {
      if (s_pr_lamida[j] > s_pr_lamida[j + 1])
      {
        temp = s_pr_lamida[j];
        s_pr_lamida[j] = s_pr_lamida[j + 1];
        s_pr_lamida[j + 1] = temp;

        temp2 = s_pr_lamida_satinnum[j];
        s_pr_lamida_satinnum[j] = s_pr_lamida_satinnum[j + 1];
        s_pr_lamida_satinnum[j + 1] = temp2;
      }
    }
  }
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    for (int j = 0; j < pLEKF->ledr.Gpsr_used_satnum - i - 1; ++j)
    {
      if (s_dr_lamida[j] > s_dr_lamida[j + 1])
      {
        temp = s_dr_lamida[j];
        s_dr_lamida[j] = s_dr_lamida[j + 1];
        s_dr_lamida[j + 1] = temp;

        temp2 = s_dr_lamida_satinnum[j];
        s_dr_lamida_satinnum[j] = s_dr_lamida_satinnum[j + 1];
        s_dr_lamida_satinnum[j + 1] = temp2;
      }
    }
  }

  /* init default parameter here */
  uint8_t mnc_dr = (dr_num < 8) ? dr_num : 8;
  uint8_t mnc_pr = (pr_num < 8) ? pr_num : 8;
#endif

#if 1
  float f_temp;
  uint32_t u_temp2;
  uint32_t* u_s_cp_lamida_satinnum = (uint32_t*)OS_MALLOC(SAT_NUM * sizeof(uint32_t));
  for (uint32_t i = 0; i < pz_inav->u_cp_used_satnum; ++i)
  {
    u_s_cp_lamida_satinnum[i] = i;
  }

  for (uint32_t i = 0; i < pz_inav->u_cp_used_satnum; ++i)
  {
    for (uint32_t j = 0; j < pz_inav->u_cp_used_satnum - i - 1; ++j)
    {
      if (f_s_cp_lamida[j] > f_s_cp_lamida[j + 1])
      {
        f_temp = f_s_cp_lamida[j];
        f_s_cp_lamida[j] = f_s_cp_lamida[j + 1];
        f_s_cp_lamida[j + 1] = f_temp;

        u_temp2 = u_s_cp_lamida_satinnum[j];
        u_s_cp_lamida_satinnum[j] = u_s_cp_lamida_satinnum[j + 1];
        u_s_cp_lamida_satinnum[j + 1] = u_temp2;
      }
    }
  }
  uint32_t u_mnc_total = (u_cp_num <= u_meadim) ? u_cp_num : u_meadim;

#if 0
  mnc_dr = 0;
  mnc_pr = 0;
  for (int i = 0; i < mnc_total; ++i)
  {
    (f_s_prdr_lamida_satinnum[i] < pLEKF->ledr.Gpsr_used_satnum) ? mnc_pr++ : mnc_dr++; /* sort pr&dr lamida together */
  }
  mnc_dr = (dr_num < mnc_dr) ? dr_num : mnc_dr;
  mnc_pr = (pr_num < mnc_pr) ? pr_num : mnc_pr;
#endif
#endif

  for (int i = 0; i < pz_inav->u_cp_used_satnum; ++i)
  {
    int cp_lamida_k = -1;

#ifdef REF_READY
    float32 ref_los[3], ref_vel_ecef[3], Doppler_Res = 0;
    float32 ref_ins;
    asensing_rtk_meas_to_ins_pr* pRaw = TcinsCal_GetTcPsrGnssDataPointer();
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_refpos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_refpos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_refpos.z;
    diffrangegnss[i] = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
    ref_los[0] = -tempvector1[0] / diffrangegnss[i];
    ref_los[1] = -tempvector1[1] / diffrangegnss[i];
    ref_los[2] = -tempvector1[2] / diffrangegnss[i]; 
    Range_Res[i] = pLEKF->ledr.G_psr_data[i].P - diffrangegnss[i];

    ref_ins = sqrtd((Ecef_refpos.x - Ecef_pos.x) * (Ecef_refpos.x - Ecef_pos.x) + (Ecef_refpos.y - Ecef_pos.y) * (Ecef_refpos.y - Ecef_pos.y) +
      (Ecef_refpos.z - Ecef_pos.z) * (Ecef_refpos.z - Ecef_pos.z));

    Lekf_matmmlt(ref_vel_ecef, *Cn2e_Ref, vel_ref, 3, 3, 1);

    float64 refDoppler, det_vel[3];
    det_vel[0] = ref_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
    det_vel[1] = ref_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
    det_vel[2] = ref_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

    refDoppler = ref_los[0] * det_vel[0] + ref_los[1] * det_vel[1] + ref_los[2] * det_vel[2];

    Doppler_Res = pLEKF->ledr.G_psr_data[i].D - refDoppler;

#endif
    for (uint32_t si = 0; si < u_mnc_total; ++si) /* control TDCP meas num here */
    {
      if (i == u_s_cp_lamida_satinnum[si])
      {
        cp_lamida_k = si;
      }			
    }

    if (cp_lamida_k >= 0)
    {
      float f_z_tmp, f_noise, f_lamida;
      float* f_meamat_tmp = (float*)OS_MALLOC(u_sysdim * sizeof(float));

      f_lamida = f_s_cp_lamida[cp_lamida_k];
      f_z_tmp = f_s_cp_z[i];
      f_noise = f_s_cp_noise[i];
      if (f_lamida < 10.0f && fabsf(pz_cp_used_data[i].f_tdcp_res) < 10.0)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        for (int k = 0; k < u_sysdim; ++k)
        {
          f_meamat_tmp[k] = f_s_cp_H[i * u_sysdim + k];
        }

        memcpy(&f_sysmeasmat[(u_mdim)*u_sysdim], f_meamat_tmp, u_sysdim * sizeof(float));
        f_z_meas[u_mdim] = f_z_tmp;
        f_measnoise[u_mdim*u_meadim+u_mdim] = f_noise;
        u_mdim++;
#endif

#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
#if defined(REF_READY)
        fprintf(fp_tight_log, "$GPTIC,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n",
          pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, (Range_Res[i]), (Z_tmp),
          ref_ins, pLEKF->ledr.G_psr_data[i].prnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#else
        LOGI(TAG_VDR, "$GPTIC,%.3f,sys:%d,prn:%d,cn0:%d,ref-cp:%.3f,ins-cp:%.3f,ref-ins:%.3f,res:%.3f,lamd:%.3f,ele:%.3f\n",
          pz_node->d_time, pz_cp_used_data[i].u_sys, pz_cp_used_data[i].u_prn,
          pz_cp_used_data[i].u_snr, 0.0, (f_z_tmp),
          0.0, pz_cp_used_data[i].f_tdcp_res, f_lamida, pz_cp_used_data[i].f_ele);
#endif
#endif
      }
      else
      {
#if defined(_WIN32) || (defined(__linux__) && (defined(__x86_64__) || defined(__i386__) ))
#if defined(REF_READY)
        fprintf(fp_tight_log, "$GPTICX,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n",
          pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, (Range_Res[i]), (Z_tmp),
          ref_ins, pLEKF->ledr.G_psr_data[i].prnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#else
        LOGE(TAG_VDR, "$GPTICX,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,res:%.3f,lamd:%.3f,ele:%.3f\n",
          pz_node->d_time, pz_cp_used_data[i].u_sys, pz_cp_used_data[i].u_prn,
          pz_cp_used_data[i].u_snr, 0.0, (f_z_tmp),
          0.0, pz_cp_used_data[i].f_tdcp_res, f_lamida, pz_cp_used_data[i].f_ele);
#endif
#endif
      }
      OS_FREE(f_meamat_tmp);
    }
    else
    {
      /* do nothing */
    }
        
#ifndef SEQ_UPDATE
    if (u_mdim >= u_meadim)
    {
      break;
    }
#endif
  }
  OS_FREE(f_s_cp_lamida);
  OS_FREE(f_s_cp_noise);
  OS_FREE(f_s_cp_z);
  OS_FREE(f_s_cp_H);
  OS_FREE(u_s_cp_lamida_satinnum);
    
#else
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; ++i)
  {
    chooseflag[i] = 0;

    if (GnssMsg_GetGnssDataPointer()->avg_CN0 < 30 && pLEKF->ledr.G_psr_data[i].SNR > 45)
    {
        continue;
    }
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

    diffrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

    e_sr[0] = -tempvector1[0] / diffrange;
    e_sr[1] = -tempvector1[1] / diffrange;
    e_sr[2] = -tempvector1[2] / diffrange;
#ifdef REF_READY
    float32 ref_los[3], ref_vel_ecef[3], Doppler_Res = 0;
    float32 ref_ins;
    asensing_rtk_meas_to_ins_pr* pRaw = TcinsCal_GetTcPsrGnssDataPointer();
    tempvector1[0] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[0] - Ecef_refpos.x;
    tempvector1[1] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[1] - Ecef_refpos.y;
    tempvector1[2] = pLEKF->ledr.G_psr_data[i].cur_sat_pos[2] - Ecef_refpos.z;
    diffrangegnss[i] = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
    ref_los[0] = -tempvector1[0] / diffrangegnss[i];
    ref_los[1] = -tempvector1[1] / diffrangegnss[i];
    ref_los[2] = -tempvector1[2] / diffrangegnss[i]; 
    Range_Res[i] = pLEKF->ledr.G_psr_data[i].P - diffrangegnss[i];

    ref_ins = sqrtd((Ecef_refpos.x - Ecef_pos.x) * (Ecef_refpos.x - Ecef_pos.x) + (Ecef_refpos.y - Ecef_pos.y) * (Ecef_refpos.y - Ecef_pos.y) +
      (Ecef_refpos.z - Ecef_pos.z) * (Ecef_refpos.z - Ecef_pos.z));

    Lekf_matmmlt(ref_vel_ecef, *Cn2e_Ref, vel_ref, 3, 3, 1);

    float64 refDoppler, det_vel[3];
    det_vel[0] = ref_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
    det_vel[1] = ref_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
    det_vel[2] = ref_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

    refDoppler = ref_los[0] * det_vel[0] + ref_los[1] * det_vel[1] + ref_los[2] * det_vel[2];

    Doppler_Res = pLEKF->ledr.G_psr_data[i].D - refDoppler;

#endif

#if 1
    if ((pLEKF->ledr.G_psr_data[i].prvalid & MEAS_TYPE_DR) == MEAS_TYPE_DR)
    {
      float32 mat_A0[3], att_mat[3], bias_mat[3], noise;
      float32 ins_vel_ecef[3];
      float64 det_vel[3], calDoppler;

      float32 mat_tmpx[NUM_X_LEKF], mat_trans, lamida;
      float32 Z_tmp, H_tmp[NUM_X_LEKF] = { 0 };

      Lekf_matmmlt(ins_vel_ecef, *Cn2e, ins_vel, 3, 3, 1);

      det_vel[0] = ins_vel_ecef[0] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[0];
      det_vel[1] = ins_vel_ecef[1] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[1];
      det_vel[2] = ins_vel_ecef[2] - pLEKF->ledr.G_psr_data[i].cur_sat_vel[2];

      calDoppler = e_sr[0] * det_vel[0] + e_sr[1] * det_vel[1] + e_sr[2] * det_vel[2];

      Lekf_matmmlt(mat_A0, e_sr, *Cn2e, 1, 3, 3);

      Lekf_matmmlt(att_mat, mat_A0, *mat_tmp1, 1, 3, 3);

      Lekf_matmmlt(bias_mat, mat_A0, *mat_tmp3, 1, 3, 3);


      H_tmp[V_X_LEKF + 0] = -1.0 * mat_A0[0];
      H_tmp[V_X_LEKF + 1] = -1.0 * mat_A0[1];
      H_tmp[V_X_LEKF + 2] = -1.0 * mat_A0[2];

      H_tmp[ATT_X_LEKF + 0] = -1.0 * att_mat[0];
      H_tmp[ATT_X_LEKF + 1] = -1.0 * att_mat[1];
      H_tmp[ATT_X_LEKF + 2] = -1.0 * att_mat[2];

      H_tmp[BG_X_LEKF + 0] = -1.0 * bias_mat[0];
      H_tmp[BG_X_LEKF + 1] = -1.0 * bias_mat[1];
      H_tmp[BG_X_LEKF + 2] = -1.0 * bias_mat[2];

      Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].D - calDoppler); /* to be completed */

      Lekf_matmmlt(mat_tmpx, H_tmp, *pLEKF->P, 1, NUM_X_LEKF, NUM_X_LEKF);

      Lekf_matmmlt(&mat_trans, mat_tmpx, H_tmp, 1, NUM_X_LEKF, 1);

      noise = pLEKF->ledr.G_psr_data[i].drnoise / 10;

      noise = noise < 0.001f ? 0.001f : noise;

      mat_trans = 1.0f / (mat_trans + noise);

      lamida = fabs(Z_tmp * mat_trans * Z_tmp);

      if (lamida < 0.5 && pLEKF->ledr.G_psr_data[i].drnoise < 1.f)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        memcpy(&pH[(NumR + available_num) * NUM_X_LEKF], H_tmp, sizeof(H_tmp));
        pZ[available_num + NumR] = Z_tmp;
        RDiag[NumR + available_num] = noise;
        available_num++;
#endif

#if defined(_WIN32) && defined(REF_READY)
        fprintf(fp_tight_log, "$GPTID,%.3f,sys:%d,prn:%d,cn0:%d,ref:%.3f,ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n", pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, Doppler_Res, (Z_tmp),
          pLEKF->ledr.G_psr_data[i].drnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#endif
      }
    }
#endif	
#if 1		
    if ((pLEKF->ledr.G_psr_data[i].prvalid & MEAS_TYPE_PR) == MEAS_TYPE_PR)
    {
      float32 mat_tmpx[NUM_X_LEKF], mat_trans, lamida;
      float32 Z_tmp, H_tmp[NUM_X_LEKF] = { 0 }, noise;
      float32 mat_tmpB[3];
      Lekf_matmmlt(pe_srCne, pe_sr, pCne, 1, 3, 3);

      Lekf_matmmlt(mat_tmpA, pe_srCne, *Dr_inv, 1, 3, 3);
      Lekf_matmmlt(mat_tmpB, mat_tmpA, *mat_tmp0, 1, 3, 3);

      if (pLEKF->ledr.G_psr_data[i].sys == SYS_GPS || pLEKF->ledr.G_psr_data[i].sys == SYS_QZS)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[0]);
        H_tmp[CLKDIFF_X_G_LEKF] = 1.0;
      }
      else if (pLEKF->ledr.G_psr_data[i].sys == SYS_CMP)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[1]);
        H_tmp[CLKDIFF_X_B_LEKF] = 1.0;
      }
      else if (pLEKF->ledr.G_psr_data[i].sys == SYS_GAL)
      {
        Z_tmp = (float)(pLEKF->ledr.G_psr_data[i].P - diffrange - pIMUS->Clk_Err[2]);
        H_tmp[CLKDIFF_X_J_LEKF] = 1.0;
      }
      else
      {
#ifdef _WIN32
        printf("Error\n");
#endif
        continue;
      }
      noise = pLEKF->ledr.G_psr_data[i].prnoise;

      noise = noise < 9.f ? 9.f : noise;
      H_tmp[POS_X_LEKF + 0] = -1.0 * e_srCne[0];
      H_tmp[POS_X_LEKF + 1] = -1.0 * e_srCne[1];
      H_tmp[POS_X_LEKF + 2] = -1.0 * e_srCne[2];

      H_tmp[ATT_X_LEKF + 0] = mat_tmpB[0];
      H_tmp[ATT_X_LEKF + 1] = mat_tmpB[1];
      H_tmp[ATT_X_LEKF + 2] = mat_tmpB[2];

      Lekf_matmmlt(mat_tmpx, H_tmp, *pLEKF->P, 1, NUM_X_LEKF, NUM_X_LEKF);

      Lekf_matmmlt(&mat_trans, mat_tmpx, H_tmp, 1, NUM_X_LEKF, 1);

      mat_trans = 1.0f / (mat_trans + noise);

      lamida = fabs(Z_tmp * mat_trans * Z_tmp);

      if (lamida < 3.0f && pLEKF->ledr.G_psr_data[i].prnoise < 100 && fabs(Z_tmp) < 5.f)
      {
#ifdef SEQ_UPDATE
        EKF_Sequential_Update(pLEKF, Z_tmp, H_tmp, mat_trans, noise);
#else
        memcpy(&pH[(NumR + available_num) * NUM_X_LEKF], H_tmp, sizeof(H_tmp));
        pZ[available_num + NumR] = Z_tmp;
        RDiag[NumR + available_num] = noise;
        available_num++;
#endif

#if defined(_WIN32) && defined(REF_READY)
        fprintf(fp_tight_log, "$GPTIP,%.3f,sys:%d,prn:%d,cn0:%d,ref-P:%.3f,ins-P:%.3f,ref-ins:%.3f,var:%.3f,lamd:%.3f,ele:%.3f,azi:%.3f\n", pRaw->cur_tow, pLEKF->ledr.G_psr_data[i].sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].SNR, (Range_Res[i]), (Z_tmp),
          ref_ins, pLEKF->ledr.G_psr_data[i].prnoise, lamida, pLEKF->ledr.G_psr_data[i].ele_azi[0], pLEKF->ledr.G_psr_data[i].ele_azi[1]);
#endif
      }
    }
#endif
#ifndef SEQ_UPDATE
    if (available_num + NumR >= NUM_X_LEKF - 7)
    {
      break;
    }
#endif
  }
#endif

#if 0
#if 1
  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; i++)
  {
    switch (pLEKF->ledr.G_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout_txt, "%c%d %lf %lf %d %lf ", sys, pLEKF->ledr.G_psr_data[i].prn, pLEKF->ledr.G_psr_data[i].P, diffrange[i], chooseflag[i], pLEKF->ledr.G_psr_data[i].prnoise);
  }
#endif

  for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; ++i)
  {
      tempvector1[0] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
      tempvector1[1] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
      tempvector1[2] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

      nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

      diffrange[i] = nowrange;

      tempvector1[0] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[0] - Ecef_gnsspos.x;
      tempvector1[1] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[1] - Ecef_gnsspos.y;
      tempvector1[2] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[2] - Ecef_gnsspos.z;

      nowrange = sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

      diffrangegnss[i] = nowrange;

      Range_Res[i] = pLEKF->ledr.C_psr_data[i].P - diffrange[i];
  }
  midnum[1] = getMidd(&Sortdiffnum[0], &Range_Res[0], pLEKF->ledr.Cpsr_used_satnum);
  if (available_num < 5)
  {
    for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; ++i)
    {
      chooseflag[i] = 0;
      if (((Range_Res[i] < 8.0) || (pLEKF->ledr.Cpsr_used_satnum <= 5)) && (Range_Res[i] < Sortdiffnum[MIN(pLEKF->ledr.Cpsr_used_satnum - 1, 5)]) && (fabs(pLEKF->ledr.C_psr_data[i].P - diffrange[i]) < 20.0))
      {
        chooseflag[i] = 1;
        tempvector1[0] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[0] - Ecef_pos.x;
        tempvector1[1] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[1] - Ecef_pos.y;
        tempvector1[2] = pLEKF->ledr.C_psr_data[i].cur_sat_pos[2] - Ecef_pos.z;

        e_sr[0] = -tempvector1[0] / sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
        e_sr[1] = -tempvector1[1] / sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);
        e_sr[2] = -tempvector1[2] / sqrtd(tempvector1[0] * tempvector1[0] + tempvector1[1] * tempvector1[1] + tempvector1[2] * tempvector1[2]);

        Lekf_matmmlt(pe_srCne, pe_sr, pCne, 1, 3, 3);

        pZ[available_num + NumR] = (float)(pLEKF->ledr.C_psr_data[i].P - diffrange[i] - pIMUS->Clk_diff);

        pH[(NumR + available_num) * NUM_X_LEKF + POS_X_LEKF + 0] = -1.0 * e_srCne[0];
        pH[(NumR + available_num) * NUM_X_LEKF + POS_X_LEKF + 1] = -1.0 * e_srCne[1];
        pH[(NumR + available_num) * NUM_X_LEKF + POS_X_LEKF + 2] = -1.0 * e_srCne[2];
        pH[(NumR + available_num) * NUM_X_LEKF + CLKDIFF_X_LEKF] = -1.0;

        tempR = 15.5;

        RDiag[NumR + available_num] = tempR * tempR;
        available_num++;
      }
    }
  }
#if 1
  for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; i++) {
    switch (pLEKF->ledr.C_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout_txt, "%c%d %lf %lf %d %lf ", sys, pLEKF->ledr.C_psr_data[i].prn, pLEKF->ledr.C_psr_data[i].P, diffrange[i], chooseflag[i], pLEKF->ledr.C_psr_data[i].prnoise);
  }
  fprintf(Debugmesout_txt, "%lf %lf %lf %d %lf", midnum[0], midnum[1], pIMUS->Clk_diff, available_num, rtkmeasPsr->cur_tow);
  fprintf(Debugmesout_txt, "\n");

  for (int i = 0; i < pLEKF->ledr.Gpsr_used_satnum; i++) {
    switch (pLEKF->ledr.G_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout3_txt, "%c%d ", sys, pLEKF->ledr.G_psr_data[i].prn);
  }
  for (int i = 0; i < pLEKF->ledr.Cpsr_used_satnum; i++) {
    switch (pLEKF->ledr.C_psr_data[i].sys)
    {
    case SYS_GPS:
      sys = 'G';
      break;
    case SYS_GAL:
      sys = 'J';
      break;
    case SYS_QZS:
      sys = 'E';
      break;
    case SYS_CMP:
      sys = 'C';
      break;
    default:
      break;
    }
    fprintf(Debugmesout3_txt, "%c%d ", sys, pLEKF->ledr.C_psr_data[i].prn);
  }
  fprintf(Debugmesout3_txt, "\n");

#endif
#endif
#if 0
  fprintf(Debugmesout2_txt, "%10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %10.7lf %lf %d %lf %lf %lf\n", rtkmeas->cur_BLH[0], rtkmeas->cur_BLH[1], rtkmeas->cur_BLH[2],
    rtkmeasPsr->pvt_blh[0], rtkmeasPsr->pvt_blh[1], rtkmeasPsr->pvt_blh[2], (double)(Llh_pos.dlat * RAD2DEG), (double)(Llh_pos.dlon * RAD2DEG), Llh_pos.dhigh, AttOld[2], available_num, pIMUS->Clk_diff, midnum[0], rtkmeas->cur_tow);
#endif	

  if (u_mdim > 0)
  {
    math_matcopy(&(pz_model->f_sysmeasmat[0]), &(f_sysmeasmat[0]), u_mdim, u_sysdim);
    memcpy(pz_model->f_measnoise, f_measnoise, u_meadim * u_meadim * sizeof(float));
    if (kf_update(f_z_meas, u_sysdim, u_mdim, pz_inav->pz_model) == INS_FALSE)
    {
      fusion_reset();
      u_bds_update = INS_FALSE;
      pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
    }
    else
    {
      pz_obs->u_kf_update = INS_TRUE;
      pz_obs->q_update_type = (uint32_t)SMT_CP;
      pz_inav->pz_obs->d_prebds_update_time = pz_inav->pz_obs->d_imu_t;
    }
    // kf_reset_trnsmat_tdcp();
  }
  else
  {
    u_bds_update = INS_FALSE;
    pz_inav->pz_obs->q_update_type = (uint32_t)SMT_NONE;
  }
  OS_FREE(f_z_meas);
  OS_FREE(f_measnoise);
  OS_FREE(f_sysmeasmat);

  return u_bds_update;
}
