/**@file        fusion_err_model.c
 * @brief		INS Error Model 
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

#ifdef _WIN32
#include <Windows.h>
#endif
#include "fusion_global.h"
#include "fusion_err_model.h"
#include "fusion_proc.h"
#include "fusion_math.h"
#include "fusion_config.h"
#include "fusion_quat.h"
#include "fusion_log.h"
#include "fusion_if.h"
#include "fusion_ellip_para.h"
#include <stdlib.h>
#include <string.h>
#include "mw_log.h"
#include "mw_alloc.h"
#include "cmn_utils.h"

static SysErrModel_t gz_sysmodel;

SysErrModel_t* fusion_get_sysmodel(void)
{
  return &gz_sysmodel;
}

void ins_init()
{
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

  gz_sysmodel.u_sysdim = pz_algconfig->u_sysdim;
  gz_sysmodel.u_meadim = pz_algconfig->u_meadim;
  gz_sysmodel.d_timestamp = INITIAL_TIME;
  gz_sysmodel.d_predict_time = INITIAL_TIME;
  gz_sysmodel.f_prdct_dt = 1.0f / pz_algconfig->z_init_para.e_sampling_rate;
  gz_sysmodel.e_sysmeatype = (SysMeaType_Enum)0;

  gz_sysmodel.f_sysnoise = (float*)OS_MALLOC(gz_sysmodel.u_sysdim * sizeof(float));
  gz_sysmodel.f_syscovmat = (float*)OS_MALLOC(gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
  gz_sysmodel.f_measnoise = (float*)OS_MALLOC(gz_sysmodel.u_meadim * gz_sysmodel.u_meadim * sizeof(float));
  gz_sysmodel.f_sysmeasmat = (float*)OS_MALLOC(gz_sysmodel.u_meadim * gz_sysmodel.u_sysdim * sizeof(float));
  gz_sysmodel.f_errstate = (float*)OS_MALLOC(gz_sysmodel.u_sysdim * sizeof(float));
  gz_sysmodel.f_systrsmat = (float*)OS_MALLOC(gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
  gz_sysmodel.f_systrsmat_tdcp = (float*)OS_MALLOC(gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
  gz_sysmodel.f_systrsmat_tdcp_save = (float*)OS_MALLOC(gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));

  if (any_Ptrs_Null(8, gz_sysmodel.f_sysnoise, gz_sysmodel.f_syscovmat, gz_sysmodel.f_measnoise, gz_sysmodel.f_sysmeasmat,
    gz_sysmodel.f_errstate, gz_sysmodel.f_systrsmat, gz_sysmodel.f_systrsmat_tdcp, gz_sysmodel.f_systrsmat_tdcp_save))
  {
    LOGW(TAG_VDR, "%s sysmodel matrix malloc failed", __FUNCTION__);
    OS_FREE(gz_sysmodel.f_sysnoise);
    OS_FREE(gz_sysmodel.f_syscovmat);
    OS_FREE(gz_sysmodel.f_measnoise);
    OS_FREE(gz_sysmodel.f_sysmeasmat);
    OS_FREE(gz_sysmodel.f_errstate);
    OS_FREE(gz_sysmodel.f_systrsmat);
    OS_FREE(gz_sysmodel.f_systrsmat_tdcp);
    OS_FREE(gz_sysmodel.f_systrsmat_tdcp_save);
  }
}

void ins_deInit()
{
  OS_FREE(gz_sysmodel.f_sysnoise);
  OS_FREE(gz_sysmodel.f_syscovmat);
  OS_FREE(gz_sysmodel.f_measnoise);
  OS_FREE(gz_sysmodel.f_sysmeasmat);
  OS_FREE(gz_sysmodel.f_errstate);
  OS_FREE(gz_sysmodel.f_systrsmat);
  OS_FREE(gz_sysmodel.f_systrsmat_tdcp);
  OS_FREE(gz_sysmodel.f_systrsmat_tdcp_save);
  LOGI(TAG_VDR, "%s sysmodel matrix free", __FUNCTION__);
}

void kf_errmodel_init(void)
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();    
  GNSSQCPara_t* pz_qc = fusion_get_qc_para();

  pz_qc->d_pre_good_bds_time = -1.0;
  gz_sysmodel.d_predict_time = INITIAL_TIME;
  gz_sysmodel.d_timestamp = INITIAL_TIME;
  gz_sysmodel.f_prdct_dt = 1.0f / pz_algconfig->z_init_para.e_sampling_rate;
  gz_sysmodel.e_sysmeatype = (SysMeaType_Enum)0;
  gz_sysmodel.u_sysdim = pz_algconfig->u_sysdim;
  gz_sysmodel.u_meadim = pz_algconfig->u_meadim;
  memset(gz_sysmodel.f_sysnoise, 0, gz_sysmodel.u_sysdim * sizeof(float));
  memset(gz_sysmodel.f_syscovmat, 0, gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
  memset(gz_sysmodel.f_measnoise, 0, gz_sysmodel.u_meadim * gz_sysmodel.u_meadim * sizeof(float));
  memset(gz_sysmodel.f_sysmeasmat, 0, gz_sysmodel.u_meadim * gz_sysmodel.u_sysdim * sizeof(float));
  memset(gz_sysmodel.f_errstate, 0, gz_sysmodel.u_sysdim * sizeof(float));
  memset(gz_sysmodel.f_systrsmat, 0, gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
  memset(gz_sysmodel.f_systrsmat_tdcp, 0, gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
  memset(gz_sysmodel.f_systrsmat_tdcp_save, 0, gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));
        
  for (uint8_t i = 0; i < gz_sysmodel.u_sysdim; i++)
  {
    gz_sysmodel.f_sysnoise[i] = pz_navconfig->f_noise[i];
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    gz_sysmodel.f_syscovmat[i * gz_sysmodel.u_sysdim + i] = pz_navconfig->f_init_posvar[i];
    gz_sysmodel.f_syscovmat[(i + 3) * gz_sysmodel.u_sysdim + i + 3] = pz_navconfig->f_init_velvar[i];
    gz_sysmodel.f_syscovmat[(i + 6) * gz_sysmodel.u_sysdim + i + 6] = pz_navconfig->f_init_attvar[i];
    gz_sysmodel.f_syscovmat[(i + 9) * gz_sysmodel.u_sysdim + i + 9] = pz_navconfig->f_init_gyrovar[i];
    gz_sysmodel.f_syscovmat[(i + 12) * gz_sysmodel.u_sysdim + i + 12] = pz_navconfig->f_init_acclvar[i];
    gz_sysmodel.f_syscovmat[(i + 15) * gz_sysmodel.u_sysdim + i + 15] = pz_navconfig->f_init_misvar[i];
    if (pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti > 0)
    {
      gz_sysmodel.f_syscovmat[(i + pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti) * gz_sysmodel.u_sysdim + i + pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti] = pz_navconfig->f_init_lai2gvar[i];
    }
#ifdef LA_IMU2REARMID_ERR_ESTI
    gz_sysmodel.f_syscovmat[i + LA_IMU2REARMID_ERR_ESTI][i + LA_IMU2REARMID_ERR_ESTI] = pz_navconfig->f_init_lai2vvar[i];
#endif
  }
  LOGI(TAG_VDR, "[init]:%.3lf,pvar:%.3f,%.3f\n", gz_sysmodel.d_timestamp,
    pz_navconfig->f_init_posvar[0], pz_navconfig->f_init_posvar[1], pz_navconfig->f_init_posvar[2]);
  LOGI(TAG_VDR, "[init]:%.3lf,pvar1:%.3f,%.3f\n", gz_sysmodel.d_timestamp,
    gz_sysmodel.f_syscovmat[0], gz_sysmodel.f_syscovmat[1 * gz_sysmodel.u_sysdim + 1], gz_sysmodel.f_syscovmat[2 * gz_sysmodel.u_sysdim + 2]);

#ifdef ODO_SF_ESTI
  gz_sysmodel.f_syscovmat[ODO_SF_ESTI][ODO_SF_ESTI] = pz_navconfig->f_init_odosfvar[0];
#endif 
  if (pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti > 0)
  {
    gz_sysmodel.f_syscovmat[(pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti) * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti] = pz_navconfig->f_init_odosfvar[0];
    gz_sysmodel.f_syscovmat[(pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti + 1) * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti + 1] = pz_navconfig->f_init_odosfvar[1];
  }
#ifdef WHEELBASE_ERR_ESTI
  gz_sysmodel.f_syscovmat[WHEELBASE_ERR_ESTI][WHEELBASE_ERR_ESTI] = pz_navconfig->f_init_wheelbasevar;
#endif
  if (pz_algconfig->z_syserrtype.u_mis_dualant_err_esti > 0)
  {
    gz_sysmodel.f_syscovmat[pz_algconfig->z_syserrtype.u_mis_dualant_err_esti * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_mis_dualant_err_esti] = pz_navconfig->f_init_misdualantvar[0];
    gz_sysmodel.f_syscovmat[(pz_algconfig->z_syserrtype.u_mis_dualant_err_esti + 1) * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_mis_dualant_err_esti + 1] = pz_navconfig->f_init_misdualantvar[1];
  }
  if (pz_algconfig->z_syserrtype.u_clock_err_esti > 0)
  {
    gz_sysmodel.f_syscovmat[pz_algconfig->z_syserrtype.u_clock_err_esti * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_clock_err_esti] = pz_navconfig->f_init_clockvar[0];
    gz_sysmodel.f_syscovmat[(pz_algconfig->z_syserrtype.u_clock_err_esti + 1) * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_clock_err_esti + 1] = pz_navconfig->f_init_clockvar[1];
    gz_sysmodel.f_syscovmat[(pz_algconfig->z_syserrtype.u_clock_err_esti + 2) * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_clock_err_esti + 2] = pz_navconfig->f_init_clockvar[2];
  }
  if (pz_algconfig->z_syserrtype.u_clock_diff_err_esti > 0)
  {
    gz_sysmodel.f_syscovmat[pz_algconfig->z_syserrtype.u_clock_diff_err_esti * gz_sysmodel.u_sysdim + pz_algconfig->z_syserrtype.u_clock_diff_err_esti] = pz_navconfig->f_init_clockdiffvar;
    kf_reset_trnsmat_tdcp();
  }
  kf_reset_trnsmat();
}

/*
* @brief: reset trans matrxi
* @param[in]: None
* @param[out]: None
* @return:
*/
void kf_reset_trnsmat(void)
{
  memset(gz_sysmodel.f_systrsmat, 0, gz_sysmodel.u_sysdim * gz_sysmodel.u_sysdim * sizeof(float));

  for (uint8_t i = 0; i < gz_sysmodel.u_sysdim; ++i)
  {
    gz_sysmodel.f_systrsmat[i*gz_sysmodel.u_sysdim+i] = 1.f;
  }
}

void kf_reset_trnsmat_tdcp(void)
{
  memset(gz_sysmodel.f_systrsmat_tdcp, 0, gz_sysmodel.u_sysdim* gz_sysmodel.u_sysdim * sizeof(float));

  for (uint8_t i = 0; i < gz_sysmodel.u_sysdim; ++i)
  {
    gz_sysmodel.f_systrsmat_tdcp[i*gz_sysmodel.u_sysdim+i] = 1.f;
  }
}

/*
* @brief: update trans matrix
* @param[in]: d_timestamp - system time; pz_nav - navigation variable
* @param[out]: None
* @return: 
*/
void kf_transmat_update(double d_timestamp, NavParams_t* pz_nav)
{
  NavConfig_t* pz_navconfig = fusion_get_navconfig();  
  FusionAlgoConfig_t* pz_algconfig = fusion_get_algconfig();

  float f_delta_t = (float)(d_timestamp - gz_sysmodel.d_timestamp);

  if (f_delta_t < 0.f || f_delta_t > 0.05)
  {
      f_delta_t = gz_sysmodel.f_prdct_dt;
  }

  float	f_omega_en[3] = { 0.0f };
  double d_rm = 0.0;
  double d_rn = 0.0;
  double d_r = 0.0;
  float	f_gravtmp = 0.0f;
  float	f_omega_ie[3] = { 0.0f };
  float	f_n[3] = { 0.0f };
  float	f_vc0 = 0.0f;
  float	f_vc1 = 0.0f;
  float	f_vc2 = 0.0f;
  float	f_mattmp[3][3] = { 0.0f };
  float	f_omega_in[3] = { 0.0f };
  uint8_t i, j;
    
  uint8_t sdim = gz_sysmodel.u_sysdim;
  float* f_trnsmat = (float*)OS_MALLOC(sdim * sdim * sizeof(float));
  float f_theta[3][3] = { 0.f }, f_vnx[3][3] = { 0.f };
  float f_ptemp[3][3] = { 0.f }, f_wenx[3][3] = { 0.f };
  d_rm = pz_nav->d_rm;
  d_rn = pz_nav->d_rn;
  d_r = sqrt(d_rm * d_rn);

  memcpy(f_omega_en, pz_nav->f_omega_en, sizeof(f_omega_en));

  f_theta[0][1] = (float)(1.f / (d_rn + pz_nav->d_poslla[2]));
  f_theta[1][0] = (float)(-1.f / (d_rm + pz_nav->d_poslla[2]));
  f_theta[2][1] = (float)(-tan(pz_nav->d_poslla[0]) / (d_rn + pz_nav->d_poslla[2]));

  math_skewsym(pz_nav->f_vel_ned, f_vnx);

  math_matmul(*f_vnx,*f_theta,3,3,3,*f_ptemp);

  math_skewsym(f_omega_en, f_wenx);

  math_matadd(*f_wenx, *f_ptemp,3,3,*f_ptemp);

  f_trnsmat[0 * sdim + 0] = 1.0f + (-f_ptemp[0][0] * f_delta_t);
  f_trnsmat[0 * sdim + 1] = -f_ptemp[0][1] * f_delta_t;
  f_trnsmat[0 * sdim + 2] = -f_ptemp[0][2] * f_delta_t;

  f_trnsmat[1 * sdim + 0] = -f_ptemp[1][0] * f_delta_t;
  f_trnsmat[1 * sdim + 1] = 1.0f + (-f_ptemp[1][1] * f_delta_t);
  f_trnsmat[1 * sdim + 2] = -f_ptemp[1][2] * f_delta_t;

  f_trnsmat[2 * sdim + 0] = -f_ptemp[2][0] * f_delta_t;
  f_trnsmat[2 * sdim + 1] = -f_ptemp[2][1] * f_delta_t;
  f_trnsmat[2 * sdim + 2] = 1.0f + (-f_ptemp[2][2] * f_delta_t);


  f_trnsmat[0 * sdim + 3] = f_delta_t;
  f_trnsmat[1 * sdim + 4] = f_delta_t;
  f_trnsmat[2 * sdim + 5] = f_delta_t;

  float f_fvr[3][3] = { 0.f }, f_fvv[3][3] = { 0.f };
  float f_rtmp[3][3] = { 0.f }, f_vtmp[3][3] = { 0.f };
  float f_vvx[3][3] = { 0.f }, f_vv[3] = { 0.f };

  f_fvr[0][0] = (float)(-2.f * EARTH_WIE * sin(pz_nav->d_poslla[0]) / (d_rm + pz_nav->d_poslla[2]));
  f_fvr[0][2] = (float)(pz_nav->f_vel_ned[1] / (SQR(d_rn + pz_nav->d_poslla[2])));

  f_fvr[1][2] = (float)(-pz_nav->f_vel_ned[0] / (SQR(d_rm + pz_nav->d_poslla[2])));

  f_fvr[2][0] = (float)(-2.f * EARTH_WIE * cos(pz_nav->d_poslla[0]) / (d_rm + pz_nav->d_poslla[2]) - ((pz_nav->f_vel_ned[1]) / ((d_rm + pz_nav->d_poslla[2]) * (d_rn + pz_nav->d_poslla[2]) * SQR(cos(pz_nav->d_poslla[0])))));
  f_fvr[2][2] = (float)(-pz_nav->f_vel_ned[1] * tan(pz_nav->d_poslla[0]) / (SQR(d_rn + pz_nav->d_poslla[2])));

  math_matmul(*f_vnx,*f_fvr,3,3,3,*f_rtmp);

  f_fvv[0][1] = (float)(1.f / (d_rn + pz_nav->d_poslla[2]));
  f_fvv[1][0] = (float)(-1.f / (d_rm + pz_nav->d_poslla[2]));
  f_fvv[2][1] = (float)(-tan(pz_nav->d_poslla[0]) / (d_rn + pz_nav->d_poslla[2]));

  math_matmul(*f_vnx, *f_fvv, 3, 3, 3, *f_vtmp);

  f_vv[0] = (-2.f * f_omega_ie[0] - f_omega_en[0]);
  f_vv[1] = (-2.f * f_omega_ie[1] - f_omega_en[1]);
  f_vv[2] = (-2.f * f_omega_ie[2] - f_omega_en[2]);

  math_skewsym(f_vv, f_vvx);

  math_matadd(*f_vvx, *f_vtmp, 3, 3, *f_vtmp);

  f_gravtmp = -f_delta_t * pz_nav->f_gravity_vct[2] / (float)(d_r + pz_nav->d_poslla[2]);

  f_trnsmat[3 * sdim + 0] = f_rtmp[0][0] * f_delta_t;
  f_trnsmat[3 * sdim + 1] = f_rtmp[0][1] * f_delta_t;
  f_trnsmat[3 * sdim + 2] = f_rtmp[0][2] * f_delta_t;
    
  f_trnsmat[4 * sdim + 0] = f_rtmp[1][0] * f_delta_t;
  f_trnsmat[4 * sdim + 1] = f_rtmp[1][1] * f_delta_t;
  f_trnsmat[4 * sdim + 2] = f_rtmp[1][2] * f_delta_t;

  f_trnsmat[5 * sdim + 0] = f_rtmp[2][0] * f_delta_t;
  f_trnsmat[5 * sdim + 1] = f_rtmp[2][1] * f_delta_t;
  f_trnsmat[5 * sdim + 2] = -2.f * f_gravtmp + f_rtmp[2][2] * f_delta_t;

  memcpy(f_omega_ie, pz_nav->f_omega_ie, sizeof(f_omega_ie));

  f_trnsmat[3 * sdim + 3] = 1.0f + f_vtmp[0][0] * f_delta_t;
  f_trnsmat[3 * sdim + 4] = f_vtmp[0][1] * f_delta_t;
  f_trnsmat[3 * sdim + 5] = f_vtmp[0][2] * f_delta_t;

  f_trnsmat[4 * sdim + 3] = f_vtmp[1][0] * f_delta_t;
  f_trnsmat[4 * sdim + 4] = 1.0f + f_vtmp[1][1] * f_delta_t;
  f_trnsmat[4 * sdim + 5] = f_vtmp[1][2] * f_delta_t;

  f_trnsmat[5 * sdim + 3] = f_vtmp[2][0] * f_delta_t;
  f_trnsmat[5 * sdim + 4] = f_vtmp[2][1] * f_delta_t;
  f_trnsmat[5 * sdim + 5] = 1.0f + f_vtmp[2][2] * f_delta_t;

  memcpy(f_n, pz_nav->f_fn, sizeof(f_n));

  f_vc0 = f_n[0] * f_delta_t;
  f_vc1 = f_n[1] * f_delta_t;
  f_vc2 = f_n[2] * f_delta_t;

  f_trnsmat[3 * sdim + 6] = 0.f;
  f_trnsmat[3 * sdim + 7] = -f_vc2;
  f_trnsmat[3 * sdim + 8] = f_vc1;

  f_trnsmat[4 * sdim + 6] = f_vc2;
  f_trnsmat[4 * sdim + 7] = 0.f;
  f_trnsmat[4 * sdim + 8] = -f_vc0;

  f_trnsmat[5 * sdim + 6] = -f_vc1;
  f_trnsmat[5 * sdim + 7] = f_vc0;
  f_trnsmat[5 * sdim + 8] = 0.f;

  quat_quat2dcm(pz_nav->f_quat_b2n, f_mattmp);

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      f_mattmp[i][j] *= f_delta_t;
    }
  }
  math_matadd(f_omega_ie, f_omega_en, 3, 1, f_omega_in);

  f_omega_in[0] *= (-f_delta_t);
  f_omega_in[1] *= (-f_delta_t);
  f_omega_in[2] *= (-f_delta_t);

  f_fvr[0][0] = (float)(-EARTH_WIE * sin(pz_nav->d_poslla[0]) / (d_rm + pz_nav->d_poslla[2]));
  f_fvr[2][0] = (float)(-EARTH_WIE * cos(pz_nav->d_poslla[0]) / (d_rm + pz_nav->d_poslla[2]) - ((pz_nav->f_vel_ned[1]) / ((d_rm + pz_nav->d_poslla[2]) * (d_rn + pz_nav->d_poslla[2]) * SQR(cos(pz_nav->d_poslla[0])))));

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      f_trnsmat[(6 + i) * sdim + j] = f_fvr[i][j] * f_delta_t;
      f_trnsmat[(6 + i) * sdim + (3 + j)] = f_fvv[i][j] * f_delta_t;
    }
  }

  f_trnsmat[6 * sdim + 6] = 1.0f;
  f_trnsmat[6 * sdim + 7] = -f_omega_in[2];
  f_trnsmat[6 * sdim + 8] = f_omega_in[1];

  f_trnsmat[7 * sdim + 6] = f_omega_in[2];
  f_trnsmat[7 * sdim + 7] = 1.0f;
  f_trnsmat[7 * sdim + 8] = -f_omega_in[0];

  f_trnsmat[8 * sdim + 6] = -f_omega_in[1];
  f_trnsmat[8 * sdim + 7] = f_omega_in[0];
  f_trnsmat[8 * sdim + 8] = 1.0f;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      f_trnsmat[(3 + i) * sdim + (12 + j)] = f_mattmp[i][j];
      f_trnsmat[(6 + i) * sdim + (9 + j)] = -f_mattmp[i][j];
    }
    f_trnsmat[(9 + i) * sdim + (9 + i)] = pz_navconfig->f_gerr_model[i];
    f_trnsmat[(12 + i) * sdim + (12 + i)] = pz_navconfig->f_aerr_model[i];
    f_trnsmat[(15 + i) * sdim + (15 + i)] = 1.0f;

    if (pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti > 0)
    {
      f_trnsmat[(pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti + i) * sdim + (pz_algconfig->z_syserrtype.u_la_imu2gnss_err_esti + i)] = 1.0f;
    }
#ifdef LA_IMU2REARMID_ERR_ESTI
    f_trnsmat[LA_IMU2REARMID_ERR_ESTI + i][LA_IMU2REARMID_ERR_ESTI + i] = 1.0f;
#endif
  }
#ifdef ODO_SF_ESTI
  f_trnsmat[ODO_SF_ESTI][ODO_SF_ESTI] = 1.0f;
#endif 

  if (pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti > 0)
  {
    f_trnsmat[pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti * sdim + pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti] = 1.0f;
    f_trnsmat[(pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti + 1) * sdim + pz_algconfig->z_syserrtype.u_rearwheel_sf_err_esti + 1] = 1.0f;
  }

#ifdef WHEELBASE_ERR_ESTI
  f_trnsmat[WHEELBASE_ERR_ESTI][WHEELBASE_ERR_ESTI] = 1.0f;
#endif
  if (pz_algconfig->z_syserrtype.u_mis_dualant_err_esti > 0)
  {
    f_trnsmat[pz_algconfig->z_syserrtype.u_mis_dualant_err_esti * sdim + pz_algconfig->z_syserrtype.u_mis_dualant_err_esti] = 1.0f;
    f_trnsmat[(pz_algconfig->z_syserrtype.u_mis_dualant_err_esti + 1) * sdim + pz_algconfig->z_syserrtype.u_mis_dualant_err_esti + 1] = 1.0f;
  }
  if (pz_algconfig->z_syserrtype.u_clock_err_esti > 0)
  {
    f_trnsmat[pz_algconfig->z_syserrtype.u_clock_err_esti * sdim + pz_algconfig->z_syserrtype.u_clock_err_esti] = 1.0f;
    f_trnsmat[(pz_algconfig->z_syserrtype.u_clock_err_esti + 1) * sdim + pz_algconfig->z_syserrtype.u_clock_err_esti + 1] = 1.0f;
    f_trnsmat[(pz_algconfig->z_syserrtype.u_clock_err_esti + 2) * sdim + pz_algconfig->z_syserrtype.u_clock_err_esti + 2] = 1.0f;
  }
  if (pz_algconfig->z_syserrtype.u_clock_diff_err_esti > 0)
  {
    f_trnsmat[pz_algconfig->z_syserrtype.u_clock_diff_err_esti * sdim + pz_algconfig->z_syserrtype.u_clock_diff_err_esti] = 1.0f;
  }
  memcpy(gz_sysmodel.f_systrsmat, &(f_trnsmat[0]), sdim * sdim * sizeof(float));
  gz_sysmodel.d_timestamp = d_timestamp;

  OS_FREE(f_trnsmat);
}

/*
* @brief: update trans matrix
* @param[in]: pd_meas - IMU/GNSS array; pz_inav - ins struct
* @param[out]: None
* @return
*/
void kf_transmat_tdcp(double pd_meas[], InertialNav_t* pz_inav)
{
  uint8_t u_sdim = gz_sysmodel.u_sysdim;
  float* TranMat = (float*)OS_MALLOC(u_sdim * u_sdim * sizeof(float));
  float* TranMt = (float*)OS_MALLOC(u_sdim * u_sdim * sizeof(float));
  float* f_systrsmat_bak = (float*)OS_MALLOC(u_sdim * u_sdim * sizeof(float));
  static uint8_t u_first_pps_flag = 0;

  if (u_first_pps_flag == 0)
  {
    int val0 = 0;
    if (fusion_check_pps_nearby((uint64_t)(pd_meas[1] * 1000), &val0) > 0)
    {
      u_first_pps_flag = 1;
    }
  }	

  if (u_first_pps_flag == 1)
  {
    uint8_t i = 0;
    int val = 0;
    float MatDet = 0.0f;
    if (fusion_check_pps_nearby((uint64_t)(pd_meas[1] * 1000), &val) > 0)
    {
      memcpy(pz_inav->pz_model->f_systrsmat_tdcp_save, pz_inav->pz_model->f_systrsmat_tdcp, u_sdim * u_sdim * sizeof(float));
      memset(pz_inav->pz_model->f_systrsmat_tdcp, 0, u_sdim * u_sdim * sizeof(float));
      for (i = 0; i < u_sdim; ++i)
      {
        pz_inav->pz_model->f_systrsmat_tdcp[i* u_sdim+i] = 1.f;
      }
    }
    else
    {
      memcpy(TranMat, pz_inav->pz_model->f_systrsmat, u_sdim * u_sdim * sizeof(float));
      memcpy(TranMt, TranMat, u_sdim * u_sdim * sizeof(float));
#if 1
      math_square_transpose(&(TranMt[0]), u_sdim);
#else
      math_matinv(TranMt, SYS_DIM, &MatDet);
#endif
      math_matmul(&(pz_inav->pz_model->f_systrsmat_tdcp[0]), &(TranMt[0]), u_sdim, u_sdim, u_sdim, &(f_systrsmat_bak[0]));
      memcpy(pz_inav->pz_model->f_systrsmat_tdcp, f_systrsmat_bak, u_sdim * u_sdim * sizeof(float));
    }
  }	
  OS_FREE(TranMat);
  OS_FREE(TranMt);
  OS_FREE(f_systrsmat_bak);
}


/*
* @brief: set update type
* @input: MeaType - update type; pz_model 
* @output: None
* @return
*/
void kf_set_update_type(SysMeaType_Enum MeaType, SysErrModel_t* pz_model)
{
  pz_model->e_sysmeatype = MeaType;
}

/*
* @brief: update obs matrix
* @input: None
* @output: MeaMat - obs matrix
* @return
*/
void lvrm_meas_matrix_pos(float* MeaMat)
{
  int i, j;
  float* imu2gnss;
  float lr_nframe[3] = { 0.0f };
  float skew_lrn[3][3] = { 0.0f }, lr_vframe_x[3][3] = { 0.f };
  float f_rotmat_v2n[3][3] = { 0.f }, f_rotmat_temp[3][3] = { 0.f };
  NavParams_t* pz_nav = fusion_get_navpara();

  imu2gnss = lvrm_get_imu2gnss();
  math_matmul(&(pz_nav->f_rotmat_b2n[0][0]),*pz_nav->f_rotmat_v2b,3,3,3,*f_rotmat_v2n);
  math_matmul(*f_rotmat_v2n, imu2gnss, 3, 3, 1, lr_nframe);
  math_skewsym(imu2gnss,lr_vframe_x);
  math_skewsym(lr_nframe, skew_lrn);
  math_matmul(*f_rotmat_v2n, *lr_vframe_x,3,3,3,*f_rotmat_temp);

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      MeaMat[i * gz_sysmodel.u_sysdim + j + 6] = skew_lrn[i][j];
      MeaMat[i * gz_sysmodel.u_sysdim + j + 15] = -f_rotmat_temp[i][j];
#ifdef LA_IMU2GNSS_ERR_ESTI
      MeaMat[i * gz_sysmodel.u_sysdim + j + LA_IMU2GNSS_ERR_ESTI] = f_rotmat_v2n[i][j];
#endif
    }
  }
}

void lvrm_meas_matrix_vel(float* MeaMat, SysMeaType_Enum e_sysmeatype)
{
  int i, j;
  float* imu2gnss;
  float lr_nframe[3] = { 0.0f };
  float skew_lrn[3][3] = { 0.0f };
  float win_n[3] = { 0.0f };
  float skew_win_n[3][3] = { 0.0f };
  float tmp1[3][3] = { 0.0f };
  float tmp2[3][3] = { 0.0f };
  float mat_ma1[3][3] = { 0.0f };
  float tmp3[3] = { 0.0f };
  float tmp4[3] = { 0.0f };
  float psiMat[9] = { 0.0f }, biasMat[9] = { 0.0f }, maMat[9] = { 0.0f };
  float mat_ma2[9] = { 0.0f };
  NavParams_t* pz_nav = fusion_get_navpara();

  imu2gnss = lvrm_get_imu2gnss();
  math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), imu2gnss, 3, 3, 1, lr_nframe);
  math_skewsym(lr_nframe, skew_lrn);
  math_matadd(pz_nav->f_omega_ie, pz_nav->f_omega_en, 3, 1, win_n);
  math_skewsym(win_n, skew_win_n);
  math_matmul(&(skew_win_n[0][0]), &(skew_lrn[0][0]), 3, 3, 3, &(tmp1[0][0]));

  math_cross_product(imu2gnss, pz_nav->f_omega_ibb, tmp3);
  math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), tmp3, 3, 3, 1, tmp4);
  math_skewsym(tmp4, tmp2);

  math_matadd(&(tmp1[0][0]), &(tmp2[0][0]), 3, 3, psiMat);

  math_skewsym(imu2gnss, tmp1);
  math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), &(tmp1[0][0]), 3, 3, 3, biasMat);

  math_mattrns(&(pz_nav->f_rotmat_b2n[0][0]), 3, 3, &(tmp1[0][0]));
  math_skewsym(tmp3, mat_ma1);
  math_matmul(&(pz_nav->f_rotmat_b2n[0][0]), &(mat_ma1[0][0]), 3, 3, 3, mat_ma2);
  math_matmul(mat_ma2, &(tmp1[0][0]), 3, 3, 3, &(mat_ma1[0][0]));
  math_matmul(&(skew_win_n[0][0]), &(skew_lrn[0][0]), 3, 3, 3, &(tmp1[0][0]));
  math_matadd(&(mat_ma1[0][0]), &(tmp1[0][0]), 3, 3, maMat);

  for (i = 0; i < 9; i++)
  {
    psiMat[i] = -psiMat[i];
    biasMat[i] = -biasMat[i];
  }

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (e_sysmeatype == SMT_BDS)
      {
        MeaMat[(i + 3) * gz_sysmodel.u_sysdim + j + 6] = psiMat[i * 3 + j];
        MeaMat[(i + 3) * gz_sysmodel.u_sysdim + j + 9] = biasMat[i * 3 + j];
        MeaMat[(i + 3) * gz_sysmodel.u_sysdim + j + 15] = maMat[i * 3 + j];
#ifdef LA_IMU2GNSS_ERR_ESTI
        /* MeaMat[(i + 3) * gz_sysmodel.u_sysdim +j + LA_IMU2GNSS_ERR_ESTI] = laMat[i][j]; */
#endif
      }
      if (e_sysmeatype == SMT_GPSVEL)
      {
        MeaMat[i * gz_sysmodel.u_sysdim + j + 6] = psiMat[i * 3 + j];
        MeaMat[i * gz_sysmodel.u_sysdim + j + 9] = biasMat[i * 3 + j];
        MeaMat[i * gz_sysmodel.u_sysdim + j + 15] = maMat[i * 3 + j];
#ifdef LA_IMU2GNSS_ERR_ESTI
        /* MeaMat[i* gz_sysmodel.u_sysdim + j + LA_IMU2GNSS_ERR_ESTI] = laMat[i][j]; */
#endif
      }
    }
  }
}

void kf_meamatrix_update_bds(float* MeaMat)
{
  int icount;
  NavConfig_t* pNavConfig = fusion_get_navconfig();

  for (icount = 0; icount < 6; icount++)
  {
    MeaMat[icount * gz_sysmodel.u_sysdim + icount] = 1.0f;
  }

  if (pNavConfig->u_i2g_opt > 0)
  {
    lvrm_meas_matrix_pos(MeaMat);
    lvrm_meas_matrix_vel(MeaMat, SMT_BDS);
  }
}


void kf_meamatrix_update_pos(float* MeaMat)
{
  int icount;
  NavConfig_t* pNavConfig = fusion_get_navconfig();

  for (icount = 0; icount < 3; icount++)
  {
    MeaMat[icount * gz_sysmodel.u_sysdim + icount] = 1.0f;
  }

  if (pNavConfig->u_i2g_opt > 0)
  {
    lvrm_meas_matrix_pos(MeaMat);
  }
}

void kf_meamatrix_update_vel(float* MeaMat)
{
  int icount;
  NavConfig_t* pNavConfig = fusion_get_navconfig();

  for (icount = 0; icount < 3; icount++)
  {
    MeaMat[icount * gz_sysmodel.u_sysdim + icount + 3] = 1.0f;
  }

  if (pNavConfig->u_i2g_opt > 0)
  {
    lvrm_meas_matrix_vel(MeaMat, SMT_GPSVEL);
  }
}

void kf_meamatrix_update_nhc(float* MeaMat) /* not ok, to be align with fusion_nhc.c */
{
  NavConfig_t* pNavConfig = fusion_get_navconfig();
    
  float f_rotmat_n2b[3][3], f_rotmat_n2v[3][3], f_vn_x[3][3], f_n2v_v_temp[3][3];
  float f_la_imu2rearmid_x[3][3], f_b2v_la_temp[3][3], f_v_v_x[3][3];
  float f_vel_v_imu[3], f_winn[3], f_winb[3], f_wnbb[3], f_wnbv[3], f_w_la_crosstemp[3], f_v_v[3];
  float* pf_la_imu2rearmid = lvrm_get_imu2rearmid();
  NavParams_t* pz_nav = fusion_get_navpara();

  math_mattrns(&(pz_nav->f_rotmat_b2n[0][0]), 3, 3, &(f_rotmat_n2b[0][0]));	
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), &(f_rotmat_n2b[0][0]), 3, 3, 3, &(f_rotmat_n2v[0][0]));
  math_matmul(&(f_rotmat_n2v[0][0]), pz_nav->f_vel_ned, 3, 3, 1, f_vel_v_imu);

  math_matadd(pz_nav->f_omega_ie, pz_nav->f_omega_en, 3, 1, f_winn);
  math_matmul(&(f_rotmat_n2b[0][0]), f_winn, 3, 3, 1, f_winb);
  math_matsub(pz_nav->f_omega_ibb, f_winb, 3, 1, f_wnbb);
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), f_wnbb, 3, 3, 1, f_wnbv);
  math_cross_product(f_wnbv, pf_la_imu2rearmid, f_w_la_crosstemp);

  math_matadd(f_vel_v_imu, f_w_la_crosstemp, 3, 1, f_v_v);
  math_skewsym(pz_nav->f_vel_ned, f_vn_x);
  math_matmul(&(f_rotmat_n2v[0][0]), &(f_vn_x[0][0]), 3, 3, 3, &(f_n2v_v_temp[0][0]));
    
  math_skewsym(pf_la_imu2rearmid, f_la_imu2rearmid_x);
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), &(f_la_imu2rearmid_x[0][0]), 3, 3, 3, &(f_b2v_la_temp[0][0]));
    
  math_skewsym(f_v_v, f_v_v_x);
  for (int icount = 0; icount < 2; icount++)
  {
    for (int jcount = 0; jcount < 3; jcount++)
    {
      MeaMat[icount * gz_sysmodel.u_sysdim + 3 + jcount] = f_rotmat_n2v[icount + 1][jcount];
      MeaMat[icount * gz_sysmodel.u_sysdim + 6 + jcount] = -f_n2v_v_temp[icount + 1][jcount];
      MeaMat[icount * gz_sysmodel.u_sysdim + 9 + jcount] = -f_b2v_la_temp[icount + 1][jcount];
      MeaMat[icount * gz_sysmodel.u_sysdim + 15 + jcount] = f_v_v_x[icount + 1][jcount];
    }
  }
}

void kf_update_measurement_matrix(uint8_t u_sdim, uint8_t u_mdim, SysErrModel_t* pz_model)
{
  float* MeaMat = (float*)OS_MALLOC(u_mdim * u_sdim * sizeof(float));
  int icount;

  SysMeaType_Enum KFmeatype = pz_model->e_sysmeatype;
  NavParams_t* pz_nav = fusion_get_navpara();

  switch (KFmeatype)
  {
  case SMT_BDS:
    kf_meamatrix_update_bds(MeaMat);
    break;
  case SMT_GPSPOS:
    kf_meamatrix_update_pos(MeaMat);
    break;
  case SMT_GPSVEL:
    kf_meamatrix_update_vel(MeaMat);
    break;
  case SMT_NHC:
    kf_meamatrix_update_nhc(MeaMat);
    break;
  case SMT_ZUPTA:
    for (icount = 3; icount < 6; icount++)
    {
      MeaMat[(icount - 3) * u_sdim + icount] = 1.0f;
    }
    MeaMat[3 * u_sdim + 8] = 1.0f;
    break;
  case SMT_ZUPT:
    for (icount = 3; icount < 6; icount++)
    {
      MeaMat[(icount - 3) * u_sdim + icount] = 1.0f;
    }
    break;
  default:
    MeaMat[0] = 0.0f;
    break;
  }
  math_matcopy(&(pz_model->f_sysmeasmat[0]), &(MeaMat[0]), u_mdim, u_sdim);
  OS_FREE(MeaMat);
}

void kf_meanoise_update_bds(double pd_var[], float* KF_R)
{
  int i;
  NavConfig_t* pNavConfig = fusion_get_navconfig();
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  uint8_t* gns_zupt_update = is_gzupt();
  uint8_t u_mdim = gz_sysmodel.u_meadim;

  GnssPosFlag_Enum    flag = pz_gnss->e_posflag;
  float  PosScale = 0.f;
  float  VelScale = 0.f;

  switch (flag)
  {
  case TYPE_POS_FLAG_GNSSONLY:
    PosScale = pNavConfig->f_pos_scl_sps;
    VelScale = pNavConfig->f_vel_scl_sps;
    break;
  case TYPE_POS_FLAG_DGNSS:
    PosScale = pNavConfig->f_pos_scl_dif;
    VelScale = pNavConfig->f_vel_scl_dif;
    break;
  case TYPE_POS_FLAG_RTK_FIX:
    PosScale = pNavConfig->f_pos_scl_rtk;
    VelScale = pNavConfig->f_vel_scl_rtk;
    break;
  case TYPE_POS_FLAG_RTK_FLOAT:
    PosScale = pNavConfig->f_pos_scl_flt;
    VelScale = pNavConfig->f_vel_scl_flt;
    break;
  default:
    PosScale = pNavConfig->f_pos_scl_sps;
    VelScale = pNavConfig->f_vel_scl_sps;
    break;
  }

  for (i = 0; i < 3; i++)
  {
    if (flag == TYPE_POS_FLAG_RTK_FIX)
    {
      KF_R[i * u_mdim + i] = PosScale * (float)(pd_var[i + 8]);
      KF_R[(i + 3) * u_mdim + i + 3] = VelScale * (float)(pd_var[i + 11]);

      KF_R[i * u_mdim + i] = (KF_R[i * u_mdim + i] > 0.01f * 0.01f) ? KF_R[i * u_mdim + i] : 0.01f * 0.01f;
      KF_R[(i + 3) * u_mdim + i + 3] = (KF_R[(i + 3) * u_mdim + i + 3] > 0.02f * 0.02f) ? KF_R[(i + 3) * u_mdim + i + 3] : 0.02f * 0.02f;
    }
    else
    {
      if (*gns_zupt_update == 0)
      {
        KF_R[i * u_mdim + i] = PosScale * (float)(pd_var[i + 8]);
        KF_R[(i + 3) * u_mdim + i + 3] = VelScale * (float)(pd_var[i + 11]);
      }
      else
      {
        KF_R[i * u_mdim + i] = (float)(50.f * 50.f);
        KF_R[(i + 3) * u_mdim + i + 3] = (float)(5.f * 5.f);
      }

      float minPosVar = (0.05f * 0.05f);
      float minVelVar = (0.05f * 0.05f);

      KF_R[i * u_mdim + i] = (KF_R[i * u_mdim + i] > minPosVar) ? KF_R[i * u_mdim + i] : minPosVar;
      KF_R[(i + 3) * u_mdim + i + 3] = (KF_R[(i + 3) * u_mdim + i + 3] > minVelVar) ? KF_R[(i + 3) * u_mdim + i + 3] : minVelVar;
    }
    pd_var[i + 8] = KF_R[i * u_mdim + i];
    pd_var[i + 11] = KF_R[(i + 3) * u_mdim + i + 3];
  }
  LOGI(TAG_VDR, "BDS Update MeaNoise: %8.3lf, %8.3lf, %8.3lf, %8.3lf, %8.3lf, %8.3lf\r\n",
    sqrt(KF_R[0]), sqrt(KF_R[1 * u_mdim + 1]), sqrt(KF_R[2 * u_mdim + 2]), sqrt(KF_R[3 * u_mdim + 3]), sqrt(KF_R[4 * u_mdim + 4]), sqrt(KF_R[5 * u_mdim + 5]));
}


void kf_meanoise_update_pos(double pd_var[], float* KF_R)
{
  int i;
  NavConfig_t* pNavConfig = fusion_get_navconfig();
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  uint8_t* gns_zupt_update = is_gzupt();
  uint8_t u_mdim = gz_sysmodel.u_meadim;

  GnssPosFlag_Enum flag = pz_gnss->e_posflag;
    
  float PosScale = 1.f;
#if 0    
  switch (flag)
  {
  case TYPE_POS_FLAG_GNSSONLY:
      PosScale = pNavConfig->f_pos_scl_sps;
      break;
  case TYPE_POS_FLAG_DGNSS:
      PosScale = pNavConfig->f_pos_scl_dif;
      break;
  case TYPE_POS_FLAG_RTK_FIX:
      PosScale = pNavConfig->f_pos_scl_rtk;
      break;
  case TYPE_POS_FLAG_RTK_FLOAT:
      PosScale = pNavConfig->f_pos_scl_flt;
      break;
  default:
      PosScale = pNavConfig->f_pos_scl_sps;
      break;
  }
#endif
  for (i = 0; i < 3; i++)
  {
    if (flag == TYPE_POS_FLAG_RTK_FIX)
    {
      KF_R[i * u_mdim + i] = PosScale * (float)(pd_var[i + 8]);
      KF_R[i * u_mdim + i] = (KF_R[i * u_mdim + i] > 0.01f * 0.01f) ? KF_R[i * u_mdim + i] : 0.01f * 0.01f;
    }
    else
    {
#if 0
      if (*gns_zupt_update == 0)
      {
        KF_R[i * u_mdim + i] = PosScale * (float)(pd_var[i + 8]);
      }
      else
      {
        KF_R[i * u_mdim + i] = (float)(50.f * 50.f);
      }
#else
      KF_R[i * u_mdim + i] = PosScale * (float)(pd_var[i + 8]);
#endif

      float minPosVar = (0.05f * 0.05f) * 4;
      KF_R[i * u_mdim + i] = (KF_R[i * u_mdim + i] > minPosVar) ? KF_R[i * u_mdim + i] : minPosVar;
    }
    pd_var[i + 8] = KF_R[i * u_mdim + i]; 
  }
  LOGI(TAG_VDR, "[MeaNoise]:%.3lf,Std:%.3f,%.3f,%.3f,GnssFlag:%d,GnssZupt:%d\n", pz_gnss->d_tow/1000.0, sqrtf(KF_R[0]),
      sqrtf(KF_R[1 * u_mdim + 1]), sqrtf(KF_R[2 * u_mdim + 2]), (uint8_t)flag, *gns_zupt_update);
}

void kf_meanoise_update_vel(double pd_var[], float* KF_R)
{
  int i;
  NavConfig_t* pNavConfig = fusion_get_navconfig();
  AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();
  GnssPosFlag_Enum flag = pz_gnss->e_posflag;
  float VelScale = 0.f;
  uint8_t* gns_zupt_update = is_gzupt();
  uint8_t u_mdim = gz_sysmodel.u_meadim;

  switch (flag)
  {
  case TYPE_POS_FLAG_GNSSONLY:
      VelScale = pNavConfig->f_vel_scl_sps;
      break;
  case TYPE_POS_FLAG_DGNSS:
      VelScale = pNavConfig->f_vel_scl_dif;
      break;
  case TYPE_POS_FLAG_RTK_FIX:
      VelScale = pNavConfig->f_vel_scl_rtk;
      break;
  case TYPE_POS_FLAG_RTK_FLOAT:
      VelScale = pNavConfig->f_vel_scl_flt;
      break;
  default:
      VelScale = pNavConfig->f_vel_scl_sps;
      break;
  }
  for (i = 0; i < 3; i++)
  {
    if (*gns_zupt_update == 0)
    {
      KF_R[i * u_mdim + i] = VelScale * (float)(pd_var[i + 11]);
    }
    else
    {
      KF_R[i * u_mdim + i] = (float)(50.f * 50.f);
    }
    pd_var[i + 11] = KF_R[i * u_mdim + i];
  }
}

/*
* @brief: update meas noise
* @input: pd_var - meas noise; pz_model - system model
* @output: None
* @return
*/
void kf_update_measurement_noise(double pd_var[], SysErrModel_t* pz_model)
{
  NavConfig_t* pNavConfig = fusion_get_navconfig();

  float dTmp = (float)(1.0 * DEG2RAD);
  int32_t i = 0;

  uint8_t u_mdim = gz_sysmodel.u_meadim;
  float* KF_R = (float*)OS_MALLOC(u_mdim * u_mdim * sizeof(float));

  SysMeaType_Enum KFmeatype = pz_model->e_sysmeatype;

  switch (KFmeatype)
  {
  case SMT_BDS:
      kf_meanoise_update_bds(pd_var, KF_R);
      break;
  case SMT_GPSPOS:
      kf_meanoise_update_pos(pd_var, KF_R);
      break;
  case SMT_GPSVEL:
      kf_meanoise_update_vel(pd_var, KF_R);
      break;
  case SMT_ODOMETER:
      KF_R[0] = 0.01f;
      KF_R[1 * u_mdim + 1] = 0.01f;
      KF_R[2 * u_mdim + 2] = 0.01f;
      break;
  case SMT_NHC:
      KF_R[0] = pNavConfig->f_nhc_yvar;	/* 0.01; */
      KF_R[1 * u_mdim + 1] = pNavConfig->f_nhc_zvar; /* (float)(pd_var[13]); */
      break;
  case SMT_ZUPTA:
      for (i = 0; i < 3; i++)
      {
        KF_R[i * u_mdim + i] = pNavConfig->f_zupt_velvar;
      }
      KF_R[3 * u_mdim + 3] = pNavConfig->f_zupt_yawvar;
      break;
  case SMT_ZUPT:
      for (i = 0; i < 3; i++)
      {
        KF_R[i * u_mdim + i] = pNavConfig->f_zupt_velvar;
      }
      break;
  default:
      KF_R[0] = 0.0f;
      break;
  }
  memcpy(pz_model->f_measnoise, KF_R, u_mdim * u_mdim *sizeof(float));
  OS_FREE(KF_R);
}
