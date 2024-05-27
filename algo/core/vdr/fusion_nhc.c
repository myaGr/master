/**@file        fusion_nhc.c
 * @brief		fusion nhc file
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

#include "fusion_nhc.h"
#include "fusion_math.h"
#include "fusion_global.h"
#include "fusion_err_model.h"
#include "fusion_quat.h"
#include "fusion_proc.h"
#include "fusion_if.h"
#include "fusion_log.h"
#include "fusion_kf.h"
#include <string.h>
#include "mw_log.h"

uint8_t nhc_meas_handler(double pd_meas[], float f_z_meas[], uint8_t* pu_mdim, InertialNav_t* pz_inav)
{
  uint8_t u_val = INS_TRUE;
  float f_rotmat_b2n[3][3], f_rotmat_n2b[3][3], f_rotmat_n2v[3][3], f_vn_x[3][3], f_n2v_v_temp[3][3];
  float f_la_imu2rear_x[3][3], f_b2v_la_temp[3][3], f_v_v_x[3][3];
  float f_vel_v_imu[3], f_winn[3], f_winb[3], f_wnbb[3], f_wnbv[3], f_w_la_crosstemp[3], f_v_v[3];
  float f_wnbv_x[3][3],f_temp[3][3];
  MechanNode_t* pz_node = pz_inav->pz_obs->pz_veh_node;
  SysErrModel_t* pz_model = pz_inav->pz_model;
  float* pf_la_imu2rearmid = lvrm_get_imu2rearmid();
  float* rear2rear = lvrm_get_rear2rear();
  NavParams_t* pz_nav = fusion_get_navpara();
  NavConfig_t* pNavConfig = pz_inav->pz_config;
  float f_la_imu2rearright[3];

  *pu_mdim = 2;
  memset(pz_model->f_measnoise, 0, pz_model->u_meadim* pz_model->u_meadim * sizeof(float));
  memset(pz_model->f_sysmeasmat, 0, pz_model->u_meadim * pz_model->u_sysdim * sizeof(float));
        
  f_la_imu2rearright[0] = pf_la_imu2rearmid[0];
  f_la_imu2rearright[1] = (float)(pf_la_imu2rearmid[1] + *rear2rear / 2.0);
  f_la_imu2rearright[2] = pf_la_imu2rearmid[2];

  quat_quat2dcm(pz_node->f_qbn, f_rotmat_b2n);
  math_mattrns(&(f_rotmat_b2n[0][0]),3,3, &(f_rotmat_n2b[0][0]));
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), &(f_rotmat_n2b[0][0]), 3, 3, 3, &(f_rotmat_n2v[0][0]));
  math_matmul(&(f_rotmat_n2v[0][0]), pz_node->f_vel, 3, 3, 1, f_vel_v_imu);
        
  math_matadd(pz_node->f_wien, pz_node->f_wnen, 3, 1, f_winn);
  math_matmul(&(f_rotmat_n2b[0][0]), f_winn, 3, 3, 1, f_winb);
  math_matsub(pz_node->f_wibb, f_winb, 3, 1, f_wnbb);
  math_matmul(&(pz_nav->f_rotmat_misalign[0][0]), f_wnbb, 3, 3, 1, f_wnbv);
  math_skewsym(f_la_imu2rearright, f_la_imu2rear_x);
  math_skewsym(f_wnbv, f_wnbv_x);
  math_matmul(*f_la_imu2rear_x, *f_wnbv_x,3,3,3, *f_temp);
  math_cross_product(f_wnbv, f_la_imu2rearright, f_w_la_crosstemp);

  math_matadd(f_vel_v_imu, f_w_la_crosstemp, 3, 1, f_v_v);
    
  f_z_meas[0] = f_v_v[1] - 0.0f;
  f_z_meas[1] = f_v_v[2] - 0.0f;

#if 1
  math_skewsym(pz_node->f_vel,f_vn_x);
  math_matmul(&(f_rotmat_n2v[0][0]), &(f_vn_x[0][0]), 3, 3, 3, &(f_n2v_v_temp[0][0]));
#if 0
  pz_model->f_sysmeasmat[0][VEL_ERR_ESTI] = f_rotmat_n2v[1][0];
  pz_model->f_sysmeasmat[0][VEL_ERR_ESTI+1] = f_rotmat_n2v[1][1];
  pz_model->f_sysmeasmat[0][VEL_ERR_ESTI+2] = f_rotmat_n2v[1][2];
  pz_model->f_sysmeasmat[1][VEL_ERR_ESTI] = f_rotmat_n2v[2][0];
  pz_model->f_sysmeasmat[1][VEL_ERR_ESTI + 1] = f_rotmat_n2v[2][1];
  pz_model->f_sysmeasmat[1][VEL_ERR_ESTI + 2] = f_rotmat_n2v[2][2];

  pz_model->f_sysmeasmat[0][ATT_ERR_ESTI] = -f_n2v_v_temp[1][0];
  pz_model->f_sysmeasmat[0][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[1][1];
  pz_model->f_sysmeasmat[0][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[1][2];
  pz_model->f_sysmeasmat[1][ATT_ERR_ESTI] = -f_n2v_v_temp[2][0];
  pz_model->f_sysmeasmat[1][ATT_ERR_ESTI + 1] = -f_n2v_v_temp[2][1];
  pz_model->f_sysmeasmat[1][ATT_ERR_ESTI + 2] = -f_n2v_v_temp[2][2];
#endif
        
  math_matmul(&(f_la_imu2rear_x[0][0]), &(pz_nav->f_rotmat_misalign[0][0]), 3, 3, 3, &(f_b2v_la_temp[0][0]));

#if 0
  pz_model->f_sysmeasmat[0][GBIAS_ERR_ESTI] = -f_b2v_la_temp[1][0];
  pz_model->f_sysmeasmat[0][GBIAS_ERR_ESTI + 1] = -f_b2v_la_temp[1][1];
  pz_model->f_sysmeasmat[0][GBIAS_ERR_ESTI + 2] = -f_b2v_la_temp[1][2];
  pz_model->f_sysmeasmat[1][GBIAS_ERR_ESTI] = -f_b2v_la_temp[2][0];
  pz_model->f_sysmeasmat[1][GBIAS_ERR_ESTI + 1] = -f_b2v_la_temp[2][1];
  pz_model->f_sysmeasmat[1][GBIAS_ERR_ESTI + 2] = -f_b2v_la_temp[2][2];
#endif

  math_skewsym(f_vel_v_imu, f_v_v_x);
  math_matsub(*f_v_v_x, *f_temp, 3, 3, *f_v_v_x);
  for (int icount = 0; icount < 2; icount++)
  {
    for (int jcount = 0; jcount < 3; jcount++)
    {
      pz_model->f_sysmeasmat[icount * pz_model->u_sysdim + 3 + jcount] = f_rotmat_n2v[icount + 1][jcount];
      pz_model->f_sysmeasmat[icount * pz_model->u_sysdim + 6 + jcount] = -f_n2v_v_temp[icount + 1][jcount];
      pz_model->f_sysmeasmat[icount * pz_model->u_sysdim + 9 + jcount] = -f_b2v_la_temp[icount + 1][jcount];
      pz_model->f_sysmeasmat[icount * pz_model->u_sysdim + 15 + jcount] = f_v_v_x[icount + 1][jcount];
#ifdef LA_IMU2REARMID_ERR_ESTI
      pz_model->f_sysmeasmat[icount][LA_IMU2REARMID_ERR_ESTI + jcount] = f_wnbv_x[icount + 1][jcount];
#endif
    }
#ifdef WHEELBASE_ERR_ESTI	
    pz_model->f_sysmeasmat[icount][WHEELBASE_ERR_ESTI] = 0.5* f_wnbv_x[icount + 1][1];
#endif
  }
#else
  kf_set_update_type(SMT_NOHOLO, pz_model);
  kf_update_measurement_matrix(pz_model->u_sysdim, *uz_mdim, pz_model);
  kf_update_measurement_noise(pd_meas, pz_model);
#endif
    
  float f_gs = (float)(sqrtf(SQR(pz_node->f_wibb[0])+SQR(pz_node->f_wibb[1])+SQR(pz_node->f_wibb[2]))*RAD2DEG);

  float f_ratio = f_gs > 15.f ? 100.f : 1.0f;

  pz_model->f_measnoise[0] = pNavConfig->f_nhc_yvar * f_ratio;
  pz_model->f_measnoise[1* pz_model->u_meadim +1] = pNavConfig->f_nhc_zvar * f_ratio;

  float lambda = kf_calcu_lambda(f_z_meas, pz_model->u_sysdim, *pu_mdim, pz_model);

  if (lambda > 50.f)
  {
    u_val = INS_FALSE;
  }
  LOGI(TAG_VDR, "[nhc_meas]:%.3lf,  z:%.3f, %.3f, std:%.3f,%.3f, lamda:%.3f, gs:%.3f\n", pz_node->d_time,
      f_z_meas[0], f_z_meas[1], sqrtf(pz_model->f_measnoise[0]), sqrtf(pz_model->f_measnoise[1 * pz_model->u_meadim + 1]),lambda, f_gs);
  return u_val;
}