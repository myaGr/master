/**@file        fusion_err_model.h
 * @brief		INS System Error Model header file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing    <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_ERR_MODEL_H__
#define __FUSION_ERR_MODEL_H__

#include <stdint.h>
#include "fusion_mech.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SMT_BDS = 0x00000001u,
  SMT_NHC = 0x00000002u,        /* 2 */
  SMT_ODOMETER = 0x00000004u,
  SMT_ZUPT = 0x00000008u,       /* 8 */
  SMT_ZUPTA = 0x00000010u,      /* 16 */
  SMT_GPSPOS = 0x00000020u,     /* 32 */
  SMT_GPSVEL = 0x00000040u,     /* 64 */
  SMT_BDSQC = 0x00000080u,
  SMT_WHEELPULSE = 0x00000100u, /* 256 */
  SMT_PR = 0x00000200u,         /* 512 */
  SMT_DR = 0x00000400u,         /* 1024 */
  SMT_PR_DR = 0x00000800u,      /* 2048 */
  SMT_CP = 0x00001000u,         /* 4096 */
  SMT_DUALANT = 0x00002000u,    /* GNSS+DUALANT 8192 */
  SMT_ZEROW = 0x00004000u,      /* 16384 */
  SMT_ZEROA = 0x00008000u,      /* 32768 */
  SMT_NONE = 0x00100000u        /* 1048576 */
} SysMeaType_Enum;

typedef struct
{
  uint8_t u_sysdim;
  uint8_t u_meadim;

  double d_timestamp;
  double d_predict_time;

  float* f_errstate;
  float* f_sysnoise;
  float* f_systrsmat;
  float* f_systrsmat_tdcp;
  float* f_systrsmat_tdcp_save;
  float* f_syscovmat;
  float* f_measnoise;
  float* f_sysmeasmat;
  float f_prdct_dt;
  SysMeaType_Enum e_sysmeatype;
} SysErrModel_t;

void kf_reset_trnsmat(void);
void kf_reset_trnsmat_tdcp(void);
void ins_init();
void ins_deInit();
void kf_errmodel_init(void);
SysErrModel_t* fusion_get_sysmodel(void);
void kf_set_update_type(SysMeaType_Enum MeaType, SysErrModel_t* pz_model);
void kf_update_measurement_noise(double pd_var[], SysErrModel_t* pz_model);
void kf_update_measurement_matrix(uint8_t u_sdim, uint8_t u_mdim, SysErrModel_t* pz_model);

#ifdef __cplusplus
}
#endif

#endif // !__FUSION_ERR_MODEL_H__
