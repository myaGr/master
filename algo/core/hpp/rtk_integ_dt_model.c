/**@file        rtk_integ_dt_model.c
 * @brief       
 * @details     
 * @author      houxiaowei
 * @date        2024/2/19
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/2/19  <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "rtk_integ_dt_model.h"
#include "rtk_type.h"
#include "integ_type.h"
#include "gnss_common.h"
#include "cmn_utils.h"

static double predict_wl_ewl_prefix_0(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  float f_pdop = pz_feat->f_pdop;
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];
  float f_prefix_ls_std_1 = pz_feat->f_prefix_ls_std[1];
  uint8_t u_prefix_sat_number = pz_feat->u_prefix_sat_number;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  float f_prefix_age = pz_feat->f_prefix_age;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  float f_prefix_pdop = pz_feat->f_prefix_pdop;
  uint8_t u_common_sat = pz_feat->u_common_sat;
  uint8_t u_dr_number = pz_feat->u_dr_number;
  uint8_t u_opensky_count = pz_feat->u_opensky_count;

  if(f_prefix_ls_std_1 <= 0.03) {
    if(f_code_uwrmse_1 <= 3.24) {
      if(f_kf_std_0 <= 0.02) {
        if(f_code_uwrmse_1 <= 1.9) {
          if(u_opensky_count <= 17.5) {
            return 0.35 + f_factor * 0.581;
          }
          else{ // u_opensky_count > 17.5
            return 0.12 + f_factor * 0.161;
          }
        }
        else{ // f_code_uwrmse_1 > 1.9
          if(u_ekf_count <= 234.5) {
            return 0.52 + f_factor * 1.013;
          }
          else{ // u_ekf_count > 234.5
            return 0.24 + f_factor * 0.317;
          }
        }
      }
      else{ // f_kf_std_0 > 0.02
        if(f_pdop_ekf_cp <= 3.54) {
          if(u_common_sat <= 27.5) {
            return 0.49 + f_factor * 0.681;
          }
          else{ // u_common_sat > 27.5
            return 0.24 + f_factor * 0.164;
          }
        }
        else{ // f_pdop_ekf_cp > 3.54
          if(u_prefix_sat_number <= 6.5) {
            return 0.39 + f_factor * 0.343;
          }
          else{ // u_prefix_sat_number > 6.5
            return 1.46 + f_factor * 0.744;
          }
        }
      }
    }
    else{ // f_code_uwrmse_1 > 3.24
      if(f_kf_std_0 <= 0.06) {
        if(f_avg_cn0_1 <= 30.31) {
          if(f_avg_cn0_1 <= 29.92) {
            return 1.38 + f_factor * 3.241;
          }
          else{ // f_avg_cn0_1 > 29.92
            return 3.53 + f_factor * 6.053;
          }
        }
        else{ // f_avg_cn0_1 > 30.31
          if(f_prefix_pdop <= 2.88) {
            return 0.80 + f_factor * 1.373;
          }
          else{ // f_prefix_pdop > 2.88
            return 0.43 + f_factor * 0.553;
          }
        }
      }
      else{ // f_kf_std_0 > 0.06
        if(u_ekf_count <= 174.5) {
          if(u_common_sat <= 23.5) {
            return 1.17 + f_factor * 1.005;
          }
          else{ // u_common_sat > 23.5
            return 4.15 + f_factor * 3.732;
          }
        }
        else{ // u_ekf_count > 174.5
          if(u_dr_number <= 2.5) {
            return 1.46 + f_factor * 4.637;
          }
          else{ // u_dr_number > 2.5
            return 0.62 + f_factor * 0.862;
          }
        }
      }
    }
  }
  else{ // f_prefix_ls_std_1 > 0.03
    if(f_avg_lock_1 <= 15.66) {
      return 5.77;
    }
    else{ // f_avg_lock_1 > 15.66
      if(f_prefix_age <= 17.0) {
        if(f_pdop <= 2.49) {
          return 9.92;
        }
        else{ // f_pdop > 2.49
          return 11.05;
        }
      }
      else{ // f_prefix_age > 17.0
        return 9.26;
      }
    }
  }
}

static double predict_wl_ewl_prefix_1(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  float f_pdop = pz_feat->f_pdop;
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  float f_prefix_ls_std_1 = pz_feat->f_prefix_ls_std[1];
  uint8_t u_solTag = pz_feat->u_solTag;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  float f_prefix_age = pz_feat->f_prefix_age;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  float f_prefix_pdop = pz_feat->f_prefix_pdop;
  float f_previous_ratio = pz_feat->f_previous_ratio;
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  uint8_t u_common_sat = pz_feat->u_common_sat;
  uint8_t u_dr_number = pz_feat->u_dr_number;
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  uint8_t u_opensky_count = pz_feat->u_opensky_count;

  if(f_prefix_ls_std_1 <= 0.03) {
    if(f_code_uwrmse_1 <= 2.95) {
      if(f_kf_std_0 <= 0.02) {
        if(f_code_uwrmse_1 <= 1.77) {
          if(u_opensky_count <= 17.5) {
            return 0.37 + f_factor * 0.681;
          }
          else{ // u_opensky_count > 17.5
            return 0.11 + f_factor * 0.135;
          }
        }
        else{ // f_code_uwrmse_1 > 1.77
          if(u_ekf_count <= 212.5) {
            return 0.50 + f_factor * 0.912;
          }
          else{ // u_ekf_count > 212.5
            return 0.21 + f_factor * 0.253;
          }
        }
      }
      else{ // f_kf_std_0 > 0.02
        if(u_common_sat <= 26.5) {
          if(f_pdop_ekf_pr <= 1.5) {
            return 0.38 + f_factor * 0.525;
          }
          else{ // f_pdop_ekf_pr > 1.5
            return 0.61 + f_factor * 0.876;
          }
        }
        else{ // u_common_sat > 26.5
          if(u_solTag <= 7.5) {
            return 0.21 + f_factor * 0.149;
          }
          else{ // u_solTag > 7.5
            return 0.40 + f_factor * 0.300;
          }
        }
      }
    }
    else{ // f_code_uwrmse_1 > 2.95
      if(f_kf_std_0 <= 0.06) {
        if(f_prefix_pdop <= 2.88) {
          if(f_previous_ratio <= 9.66) {
            return 0.65 + f_factor * 1.154;
          }
          else{ // f_previous_ratio > 9.66
            return 1.75 + f_factor * 2.173;
          }
        }
        else{ // f_prefix_pdop > 2.88
          if(f_avg_cn0_1 <= 30.2) {
            return 3.16 + f_factor * 5.632;
          }
          else{ // f_avg_cn0_1 > 30.2
            return 0.41 + f_factor * 0.548;
          }
        }
      }
      else{ // f_kf_std_0 > 0.06
        if(u_ekf_count <= 170.5) {
          if(u_common_sat <= 23.5) {
            return 1.11 + f_factor * 0.905;
          }
          else{ // u_common_sat > 23.5
            return 3.99 + f_factor * 3.657;
          }
        }
        else{ // u_ekf_count > 170.5
          if(u_dr_number <= 2.5) {
            return 1.39 + f_factor * 4.145;
          }
          else{ // u_dr_number > 2.5
            return 0.61 + f_factor * 0.943;
          }
        }
      }
    }
  }
  else{ // f_prefix_ls_std_1 > 0.03
    if(f_pdop <= 1.9) {
      return 6.43;
    }
    else{ // f_pdop > 1.9
      if(f_prefix_age <= 20.2) {
        if(f_kf_std_0 <= 1.08) {
          return 11.34;
        }
        else{ // f_kf_std_0 > 1.08
          return 9.99;
        }
      }
      else{ // f_prefix_age > 20.2
        return 9.26;
      }
    }
  }
}

static double predict_wl_ewl_prefix_2(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  float f_pdop = pz_feat->f_pdop;
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  float f_prefix_ls_std_1 = pz_feat->f_prefix_ls_std[1];
  uint8_t u_solTag = pz_feat->u_solTag;
  uint8_t u_code_number_1 = pz_feat->u_code_number[1];
  float f_prefix_age = pz_feat->f_prefix_age;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  float f_prefix_pdop = pz_feat->f_prefix_pdop;
  uint8_t u_common_sat = pz_feat->u_common_sat;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  uint8_t u_scene = pz_feat->u_scene;

  if(f_prefix_ls_std_1 <= 0.03) {
    if(f_code_uwrmse_1 <= 3.04) {
      if(f_kf_std_0 <= 0.02) {
        if(f_code_uwrmse_1 <= 1.77) {
          if(u_solTag <= 7.5) {
            return 0.11 + f_factor * 0.135;
          }
          else{ // u_solTag > 7.5
            return 0.60 + f_factor * 0.864;
          }
        }
        else{ // f_code_uwrmse_1 > 1.77
          if(u_ekf_count <= 210.5) {
            return 0.48 + f_factor * 0.851;
          }
          else{ // u_ekf_count > 210.5
            return 0.22 + f_factor * 0.281;
          }
        }
      }
      else{ // f_kf_std_0 > 0.02
        if(u_common_sat <= 26.5) {
          if(f_pdop_ekf_cp <= 3.54) {
            return 0.50 + f_factor * 0.740;
          }
          else{ // f_pdop_ekf_cp > 3.54
            return 1.28 + f_factor * 0.708;
          }
        }
        else{ // u_common_sat > 26.5
          if(u_solTag <= 7.5) {
            return 0.21 + f_factor * 0.146;
          }
          else{ // u_solTag > 7.5
            return 0.40 + f_factor * 0.300;
          }
        }
      }
    }
    else{ // f_code_uwrmse_1 > 3.04
      if(f_kf_std_0 <= 0.06) {
        if(f_avg_cn0_1 <= 30.31) {
          if(u_code_number_1 <= 29.5) {
            return 1.39 + f_factor * 4.624;
          }
          else{ // u_code_number_1 > 29.5
            return 4.11 + f_factor * 6.607;
          }
        }
        else{ // f_avg_cn0_1 > 30.31
          if(f_prefix_pdop <= 2.88) {
            return 0.81 + f_factor * 1.443;
          }
          else{ // f_prefix_pdop > 2.88
            return 0.42 + f_factor * 0.528;
          }
        }
      }
      else{ // f_kf_std_0 > 0.06
        if(u_ekf_count <= 174.5) {
          if(u_common_sat <= 23.5) {
            return 1.10 + f_factor * 0.829;
          }
          else{ // u_common_sat > 23.5
            return 4.04 + f_factor * 3.687;
          }
        }
        else{ // u_ekf_count > 174.5
          if(u_scene <= 0.5) {
            return 0.62 + f_factor * 0.978;
          }
          else{ // u_scene > 0.5
            return 1.43 + f_factor * 4.523;
          }
        }
      }
    }
  }
  else{ // f_prefix_ls_std_1 > 0.03
    if(f_pdop <= 1.97) {
      return 5.48;
    }
    else{ // f_pdop > 1.97
      if(f_prefix_age <= 17.0) {
        if(f_kf_std_0 <= 1.12) {
          return 11.57;
        }
        else{ // f_kf_std_0 > 1.12
          return 10.38;
        }
      }
      else{ // f_prefix_age > 17.0
        return 9.84;
      }
    }
  }
}

static double predict_float_rtd_0(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:5, n_leaves:31
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  uint8_t u_sol_bit_count = pz_integ->u_sol_bit_count;
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];
  uint8_t u_dr_number = pz_feat->u_dr_number;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  uint8_t u_common_sat = pz_feat->u_common_sat;
  float f_pdop = pz_feat->f_pdop;
  uint8_t u_phase_number_1 = pz_feat->u_phase_number[1];
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];

  if(f_code_uwrmse_1 <= 4.38) {
    if(f_kf_std_0 <= 8.43) {
      if(f_code_uwrmse_1 <= 3.23) {
        if(f_kf_std_0 <= 1.36) {
          if(u_sol_bit_count <= 15.5) {
            return 0.63 + f_factor * 0.817;
          }
          else{ // u_sol_bit_count > 15.5
            return 1.28 + f_factor * 3.128;
          }
        }
        else{ // f_kf_std_0 > 1.36
          if(f_avg_lock_1 <= 8.32) {
            return 1.90 + f_factor * 3.879;
          }
          else{ // f_avg_lock_1 > 8.32
            return 3.66 + f_factor * 4.780;
          }
        }
      }
      else{ // f_code_uwrmse_1 > 3.23
        if(f_kf_std_1 <= 1.25) {
          if(u_sol_bit_count <= 16.5) {
            return 1.22 + f_factor * 2.027;
          }
          else{ // u_sol_bit_count > 16.5
            return 3.40 + f_factor * 4.699;
          }
        }
        else{ // f_kf_std_1 > 1.25
          if(f_avg_lock_1 <= 6.6) {
            return 3.58 + f_factor * 6.212;
          }
          else{ // f_avg_lock_1 > 6.6
            return 6.31;
          }
        }
      }
    }
    else{ // f_kf_std_0 > 8.43
      if(f_code_uwrmse_1 <= 3.06) {
        if(u_phase_number_1 <= 5.5) {
          if(f_kf_std_0 <= 16.3) {
            return 3.62 + f_factor * 3.328;
          }
          else{ // f_kf_std_0 > 16.3
            return 14.02;
          }
        }
        else{ // u_phase_number_1 > 5.5
          if(f_code_uwrmse_1 <= 2.03) {
            return 2.96 + f_factor * 4.557;
          }
          else{ // f_code_uwrmse_1 > 2.03
            return 5.69;
          }
        }
      }
      else{ // f_code_uwrmse_1 > 3.06
        if(f_pdop_ekf_cp <= 36.52) {
          if(f_kf_std_0 <= 15.93) {
            return 8.69;
          }
          else{ // f_kf_std_0 > 15.93
            return 13.44;
          }
        }
        else{ // f_pdop_ekf_cp > 36.52
          return 111.75;
        }
      }
    }
  }
  else{ // f_code_uwrmse_1 > 4.38
    if(f_kf_std_0 <= 13.36) {
      if(u_ekf_count <= 253.5) {
        if(u_phase_number_1 <= 18.5) {
          if(f_kf_std_0 <= 8.05) {
            return 7.35;
          }
          else{ // f_kf_std_0 > 8.05
            return 10.99;
          }
        }
        else{ // u_phase_number_1 > 18.5
          if(f_avg_cn0_1 <= 33.41) {
            return 22.64;
          }
          else{ // f_avg_cn0_1 > 33.41
            return 11.18;
          }
        }
      }
      else{ // u_ekf_count > 253.5
        if(f_avg_cn0_1 <= 35.59) {
          if(f_pdop <= 1.41) {
            return 9.81;
          }
          else{ // f_pdop > 1.41
            return 5.37;
          }
        }
        else{ // f_avg_cn0_1 > 35.59
          if(u_dr_number <= 11.5) {
            return 2.79 + f_factor * 4.072;
          }
          else{ // u_dr_number > 11.5
            return 7.98;
          }
        }
      }
    }
    else{ // f_kf_std_0 > 13.36
      if(f_pdop_ekf_pr <= 2.31) {
        if(u_common_sat <= 22.5) {
          if(f_pdop_ekf_cp <= 2.68) {
            return 10.52;
          }
          else{ // f_pdop_ekf_cp > 2.68
            return 15.97;
          }
        }
        else{ // u_common_sat > 22.5
          if(f_avg_cn0_1 <= 33.34) {
            return 23.10;
          }
          else{ // f_avg_cn0_1 > 33.34
            return 15.81;
          }
        }
      }
      else{ // f_pdop_ekf_pr > 2.31
        if(f_code_uwrmse_1 <= 7.23) {
          if(u_common_sat <= 20.5) {
            return 17.08;
          }
          else{ // u_common_sat > 20.5
            return 25.31;
          }
        }
        else{ // f_code_uwrmse_1 > 7.23
          if(u_common_sat <= 13.5) {
            return 16.07;
          }
          else{ // u_common_sat > 13.5
            return 30.91;
          }
        }
      }
    }
  }
}

static double predict_float_rtd_1(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:5, n_leaves:31
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  uint8_t u_sol_bit_count = pz_integ->u_sol_bit_count;
  uint8_t u_sat_number_1 = pz_feat->u_sat_number[1];
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];
  uint8_t u_dr_number = pz_feat->u_dr_number;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  uint8_t u_common_sat = pz_feat->u_common_sat;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  float f_pdop = pz_feat->f_pdop;
  uint8_t u_phase_number_1 = pz_feat->u_phase_number[1];
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];

  if(f_code_uwrmse_1 <= 4.36) {
    if(f_kf_std_0 <= 8.43) {
      if(f_code_uwrmse_1 <= 3.23) {
        if(f_kf_std_0 <= 1.36) {
          if(u_sol_bit_count <= 16.5) {
            return 0.63 + f_factor * 0.802;
          }
          else{ // u_sol_bit_count > 16.5
            return 1.29 + f_factor * 3.108;
          }
        }
        else{ // f_kf_std_0 > 1.36
          if(f_avg_lock_1 <= 8.06) {
            return 1.91 + f_factor * 4.039;
          }
          else{ // f_avg_lock_1 > 8.06
            return 3.58 + f_factor * 4.657;
          }
        }
      }
      else{ // f_code_uwrmse_1 > 3.23
        if(f_kf_std_1 <= 0.82) {
          if(u_sol_bit_count <= 19.5) {
            return 1.23 + f_factor * 2.266;
          }
          else{ // u_sol_bit_count > 19.5
            return 3.19 + f_factor * 4.063;
          }
        }
        else{ // f_kf_std_1 > 0.82
          if(f_avg_lock_1 <= 6.76) {
            return 3.47 + f_factor * 6.128;
          }
          else{ // f_avg_lock_1 > 6.76
            return 5.88;
          }
        }
      }
    }
    else{ // f_kf_std_0 > 8.43
      if(f_code_uwrmse_1 <= 3.06) {
        if(u_phase_number_1 <= 6.5) {
          if(f_kf_std_0 <= 16.01) {
            return 4.84 + f_factor * 9.091;
          }
          else{ // f_kf_std_0 > 16.01
            return 11.72;
          }
        }
        else{ // u_phase_number_1 > 6.5
          if(f_pdop_ekf_pr <= 9.35) {
            return 4.00 + f_factor * 7.144;
          }
          else{ // f_pdop_ekf_pr > 9.35
            return 12.98;
          }
        }
      }
      else{ // f_code_uwrmse_1 > 3.06
        if(f_pdop_ekf_pr <= 49.09) {
          if(f_kf_std_0 <= 15.93) {
            return 8.64;
          }
          else{ // f_kf_std_0 > 15.93
            return 13.34;
          }
        }
        else{ // f_pdop_ekf_pr > 49.09
          return 92.30;
        }
      }
    }
  }
  else{ // f_code_uwrmse_1 > 4.36
    if(f_kf_std_0 <= 12.99) {
      if(u_ekf_count <= 253.5) {
        if(u_sat_number_1 <= 15.5) {
          if(u_sat_number_1 <= 10.5) {
            return 7.37;
          }
          else{ // u_sat_number_1 > 10.5
            return 10.89;
          }
        }
        else{ // u_sat_number_1 > 15.5
          if(f_avg_cn0_1 <= 33.48) {
            return 21.81;
          }
          else{ // f_avg_cn0_1 > 33.48
            return 11.48;
          }
        }
      }
      else{ // u_ekf_count > 253.5
        if(f_avg_cn0_1 <= 35.7) {
          if(f_pdop <= 1.41) {
            return 9.70;
          }
          else{ // f_pdop > 1.41
            return 5.30;
          }
        }
        else{ // f_avg_cn0_1 > 35.7
          if(u_dr_number <= 11.5) {
            return 2.75 + f_factor * 3.838;
          }
          else{ // u_dr_number > 11.5
            return 7.94;
          }
        }
      }
    }
    else{ // f_kf_std_0 > 12.99
      if(f_pdop_ekf_pr <= 2.31) {
        if(u_common_sat <= 22.5) {
          if(f_pdop_ekf_cp <= 2.68) {
            return 10.29;
          }
          else{ // f_pdop_ekf_cp > 2.68
            return 15.57;
          }
        }
        else{ // u_common_sat > 22.5
          if(f_avg_cn0_1 <= 33.33) {
            return 23.07;
          }
          else{ // f_avg_cn0_1 > 33.33
            return 15.61;
          }
        }
      }
      else{ // f_pdop_ekf_pr > 2.31
        if(f_code_uwrmse_1 <= 7.39) {
          if(u_common_sat <= 20.5) {
            return 17.27;
          }
          else{ // u_common_sat > 20.5
            return 25.16;
          }
        }
        else{ // f_code_uwrmse_1 > 7.39
          if(u_common_sat <= 13.5) {
            return 15.11;
          }
          else{ // u_common_sat > 13.5
            return 31.22;
          }
        }
      }
    }
  }
}

static double predict_float_rtd_2(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:5, n_leaves:31
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  uint8_t u_sol_bit_count = pz_integ->u_sol_bit_count;
  uint8_t u_sat_number_1 = pz_feat->u_sat_number[1];
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];
  uint8_t u_dr_number = pz_feat->u_dr_number;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  uint8_t u_common_sat = pz_feat->u_common_sat;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  uint8_t u_phase_number_1 = pz_feat->u_phase_number[1];
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];

  if(f_code_uwrmse_1 <= 4.44) {
    if(f_kf_std_0 <= 8.43) {
      if(f_code_uwrmse_1 <= 3.23) {
        if(f_kf_std_0 <= 1.36) {
          if(u_sol_bit_count <= 16.5) {
            return 0.63 + f_factor * 0.843;
          }
          else{ // u_sol_bit_count > 16.5
            return 1.30 + f_factor * 3.271;
          }
        }
        else{ // f_kf_std_0 > 1.36
          if(f_avg_lock_1 <= 8.18) {
            return 1.90 + f_factor * 3.875;
          }
          else{ // f_avg_lock_1 > 8.18
            return 3.66 + f_factor * 5.020;
          }
        }
      }
      else{ // f_code_uwrmse_1 > 3.23
        if(u_sol_bit_count <= 19.5) {
          if(f_kf_std_1 <= 1.34) {
            return 1.40 + f_factor * 2.652;
          }
          else{ // f_kf_std_1 > 1.34
            return 3.61 + f_factor * 7.247;
          }
        }
        else{ // u_sol_bit_count > 19.5
          if(u_dr_number <= 7.5) {
            return 3.65 + f_factor * 5.514;
          }
          else{ // u_dr_number > 7.5
            return 6.74;
          }
        }
      }
    }
    else{ // f_kf_std_0 > 8.43
      if(f_code_uwrmse_1 <= 3.06) {
        if(u_phase_number_1 <= 6.5) {
          if(f_kf_std_0 <= 16.01) {
            return 4.79 + f_factor * 9.122;
          }
          else{ // f_kf_std_0 > 16.01
            return 11.79;
          }
        }
        else{ // u_phase_number_1 > 6.5
          if(f_pdop_ekf_pr <= 9.22) {
            return 4.04 + f_factor * 7.178;
          }
          else{ // f_pdop_ekf_pr > 9.22
            return 12.92;
          }
        }
      }
      else{ // f_code_uwrmse_1 > 3.06
        if(f_pdop_ekf_pr <= 49.09) {
          if(f_kf_std_0 <= 15.93) {
            return 8.67;
          }
          else{ // f_kf_std_0 > 15.93
            return 13.55;
          }
        }
        else{ // f_pdop_ekf_pr > 49.09
          return 85.21;
        }
      }
    }
  }
  else{ // f_code_uwrmse_1 > 4.44
    if(f_kf_std_0 <= 13.07) {
      if(u_ekf_count <= 253.5) {
        if(u_sat_number_1 <= 15.5) {
          if(u_sat_number_1 <= 10.5) {
            return 7.49;
          }
          else{ // u_sat_number_1 > 10.5
            return 11.08;
          }
        }
        else{ // u_sat_number_1 > 15.5
          if(f_avg_cn0_1 <= 33.41) {
            return 22.19;
          }
          else{ // f_avg_cn0_1 > 33.41
            return 11.72;
          }
        }
      }
      else{ // u_ekf_count > 253.5
        if(f_avg_cn0_1 <= 35.7) {
          if(f_avg_lock_1 <= 10.15) {
            return 5.28;
          }
          else{ // f_avg_lock_1 > 10.15
            return 9.57;
          }
        }
        else{ // f_avg_cn0_1 > 35.7
          if(u_dr_number <= 11.5) {
            return 2.76 + f_factor * 3.991;
          }
          else{ // u_dr_number > 11.5
            return 7.99;
          }
        }
      }
    }
    else{ // f_kf_std_0 > 13.07
      if(f_pdop_ekf_pr <= 2.31) {
        if(u_common_sat <= 22.5) {
          if(f_pdop_ekf_cp <= 2.75) {
            return 10.41;
          }
          else{ // f_pdop_ekf_cp > 2.75
            return 15.95;
          }
        }
        else{ // u_common_sat > 22.5
          if(f_avg_cn0_1 <= 33.33) {
            return 22.89;
          }
          else{ // f_avg_cn0_1 > 33.33
            return 15.84;
          }
        }
      }
      else{ // f_pdop_ekf_pr > 2.31
        if(f_code_uwrmse_1 <= 7.23) {
          if(u_common_sat <= 20.5) {
            return 17.20;
          }
          else{ // u_common_sat > 20.5
            return 25.21;
          }
        }
        else{ // f_code_uwrmse_1 > 7.23
          if(f_kf_std_0 <= 21.78) {
            return 24.78;
          }
          else{ // f_kf_std_0 > 21.78
            return 32.66;
          }
        }
      }
    }
  }
}

static double predict_fix(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:7, n_leaves:119
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  float f_pdop_fix = pz_feat->f_pdop_fix;
  float ewl_f_adop = pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_adop;
  uint8_t u_code_number_0 = pz_feat->u_code_number[0];
  float f_prefix_age = pz_feat->f_prefix_age;
  uint8_t u_previous_fix_sat = pz_feat->u_previous_fix_sat;
  uint8_t u_prefix_sig_number = pz_feat->u_prefix_sig_number;
  float wl_f_ratio = pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_ratio;
  uint8_t u_phase_number_1 = pz_feat->u_phase_number[1];
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  float wl_f_rss_0 = pz_feat->z_wl[GNSS_FILTER_POS_FIX_WL].f_rss[0];
  float f_avg_cn0_0 = pz_feat->f_avg_cn0[0];
  uint8_t u_closesky_count = pz_feat->u_closesky_count;
  uint8_t u_fix_count = pz_integ->u_fix_count;
  float f_pdop = pz_feat->f_pdop;
  float ewl_f_pdop = pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_pdop;
  float ewl_f_rss_0 = pz_feat->z_wl[GNSS_FILTER_POS_FIX_EWL].f_rss[0];
  uint8_t u_sat_number_0 = pz_feat->u_sat_number[0];
  float f_avg_lock_0 = pz_feat->f_avg_lock[0];
  float f_rss_0 = pz_feat->f_rss[0];
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  uint8_t u_dr_number = pz_feat->u_dr_number;
  float f_code_uwrmse_0 = pz_feat->f_code_uwrmse[0];
  uint8_t u_sat_number_1 = pz_feat->u_sat_number[1];
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  uint8_t u_opensky_count = pz_feat->u_opensky_count;
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  float f_prefix_pdop = pz_feat->f_prefix_pdop;
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  float f_adop_fix = pz_feat->f_adop_fix;
  float f_kf_std_0 = pz_feat->f_kf_std[0];
  uint8_t u_common_sat = pz_feat->u_common_sat;
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];

  if(u_opensky_count <= 8.5) {
    if(u_phase_number_1 <= 26.5) {
      if(u_opensky_count <= 0.5) {
        if(f_avg_lock_0 <= 13.89) {
          if(f_avg_cn0_0 <= 30.56) {
            if(f_kf_std_0 <= 0.16) {
              if(f_avg_lock_0 <= 5.08) {
                return 1.25 + f_factor * 3.644;
              }
              else{ // f_avg_lock_0 > 5.08
                return 3.59 + f_factor * 4.625;
              }
            }
            else{ // f_kf_std_0 > 0.16
              if(f_pdop_fix <= 3.11) {
                return 2.11 + f_factor * 3.119;
              }
              else{ // f_pdop_fix > 3.11
                return 0.81 + f_factor * 2.126;
              }
            }
          }
          else{ // f_avg_cn0_0 > 30.56
            if(u_fix_count <= 19.5) {
              if(u_ekf_count <= 64.5) {
                return 0.97 + f_factor * 1.317;
              }
              else{ // u_ekf_count > 64.5
                return 0.63 + f_factor * 0.530;
              }
            }
            else{ // u_fix_count > 19.5
              if(f_avg_cn0_0 <= 32.47) {
                return 1.01 + f_factor * 2.506;
              }
              else{ // f_avg_cn0_0 > 32.47
                return 0.40 + f_factor * 0.564;
              }
            }
          }
        }
        else{ // f_avg_lock_0 > 13.89
          if(f_rss_0 <= 135.82) {
            if(u_ekf_count <= 51.5) {
              if(f_pdop_fix <= 24.97) {
                return 1.68 + f_factor * 0.541;
              }
              else{ // f_pdop_fix > 24.97
                return 8.15;
              }
            }
            else{ // u_ekf_count > 51.5
              if(f_avg_cn0_1 <= 32.25) {
                return 1.45 + f_factor * 0.708;
              }
              else{ // f_avg_cn0_1 > 32.25
                return 0.94 + f_factor * 0.700;
              }
            }
          }
          else{ // f_rss_0 > 135.82
            if(f_avg_cn0_1 <= 33.93) {
              if(f_rss_0 <= 380.74) {
                return 0.54 + f_factor * 0.369;
              }
              else{ // f_rss_0 > 380.74
                return 0.20 + f_factor * 0.065;
              }
            }
            else{ // f_avg_cn0_1 > 33.93
              if(f_kf_std_1 <= 0.1) {
                return 1.50 + f_factor * 0.568;
              }
              else{ // f_kf_std_1 > 0.1
                return 0.84 + f_factor * 0.691;
              }
            }
          }
        }
      }
      else{ // u_opensky_count > 0.5
        if(u_sat_number_1 <= 12.5) {
          if(f_avg_cn0_1 <= 34.84) {
            if(u_prefix_sig_number <= 8.5) {
              if(f_rss_0 <= 709.52) {
                return 0.64 + f_factor * 0.884;
              }
              else{ // f_rss_0 > 709.52
                return 3.05 + f_factor * 2.872;
              }
            }
            else{ // u_prefix_sig_number > 8.5
              if(f_pdop <= 3.42) {
                return 0.33 + f_factor * 0.376;
              }
              else{ // f_pdop > 3.42
                return 0.88 + f_factor * 0.623;
              }
            }
          }
          else{ // f_avg_cn0_1 > 34.84
            if(f_avg_cn0_1 <= 38.74) {
              if(f_avg_lock_1 <= 12.62) {
                return 0.77 + f_factor * 0.704;
              }
              else{ // f_avg_lock_1 > 12.62
                return 1.69 + f_factor * 2.273;
              }
            }
            else{ // f_avg_cn0_1 > 38.74
              if(u_common_sat <= 23.5) {
                return 0.67 + f_factor * 0.561;
              }
              else{ // u_common_sat > 23.5
                return 0.32 + f_factor * 0.188;
              }
            }
          }
        }
        else{ // u_sat_number_1 > 12.5
          if(u_ekf_count <= 10.5) {
            if(f_avg_lock_0 <= 2.73) {
              return 0.32 + f_factor * 0.147;
            }
            else{ // f_avg_lock_0 > 2.73
              return 4.52 + f_factor * 20.247;
            }
          }
          else{ // u_ekf_count > 10.5
            if(f_avg_cn0_1 <= 29.84) {
              if(u_ekf_count <= 244.5) {
                return 0.23 + f_factor * 0.101;
              }
              else{ // u_ekf_count > 244.5
                return 1.79 + f_factor * 2.168;
              }
            }
            else{ // f_avg_cn0_1 > 29.84
              if(f_code_uwrmse_1 <= 3.17) {
                return 0.39 + f_factor * 0.808;
              }
              else{ // f_code_uwrmse_1 > 3.17
                return 0.59 + f_factor * 0.640;
              }
            }
          }
        }
      }
    }
    else{ // u_phase_number_1 > 26.5
      if(u_common_sat <= 27.5) {
        if(f_avg_cn0_0 <= 30.21) {
          if(u_previous_fix_sat <= 9.5) {
            if(f_pdop_ekf_pr <= 1.49) {
              return 2.35 + f_factor * 3.713;
            }
            else{ // f_pdop_ekf_pr > 1.49
              if(f_pdop <= 1.02) {
                return 3.39 + f_factor * 4.835;
              }
              else{ // f_pdop > 1.02
                return 4.66 + f_factor * 5.999;
              }
            }
          }
          else{ // u_previous_fix_sat > 9.5
            return 0.99 + f_factor * 3.845;
          }
        }
        else{ // f_avg_cn0_0 > 30.21
          if(u_ekf_count <= 211.5) {
            if(u_dr_number <= 10.5) {
              if(u_fix_count <= 8.5) {
                return 0.81 + f_factor * 2.787;
              }
              else{ // u_fix_count > 8.5
                return 0.38 + f_factor * 0.516;
              }
            }
            else{ // u_dr_number > 10.5
              if(f_avg_cn0_1 <= 40.26) {
                return 1.24 + f_factor * 4.655;
              }
              else{ // f_avg_cn0_1 > 40.26
                return 5.85;
              }
            }
          }
          else{ // u_ekf_count > 211.5
            if(u_opensky_count <= 1.5) {
              if(u_sat_number_0 <= 8.5) {
                return 0.66 + f_factor * 0.738;
              }
              else{ // u_sat_number_0 > 8.5
                return 0.43 + f_factor * 0.544;
              }
            }
            else{ // u_opensky_count > 1.5
              if(f_avg_cn0_0 <= 37.02) {
                return 0.25 + f_factor * 0.470;
              }
              else{ // f_avg_cn0_0 > 37.02
                return 0.37 + f_factor * 0.418;
              }
            }
          }
        }
      }
      else{ // u_common_sat > 27.5
        if(u_ekf_count <= 209.5) {
          if(u_ekf_count <= 195.5) {
            if(u_previous_fix_sat <= 6.5) {
              if(f_avg_lock_1 <= 9.05) {
                return 0.43 + f_factor * 0.708;
              }
              else{ // f_avg_lock_1 > 9.05
                return 1.41 + f_factor * 0.538;
              }
            }
            else{ // u_previous_fix_sat > 6.5
              if(wl_f_ratio <= 64.64) {
                return 0.30 + f_factor * 0.182;
              }
              else{ // wl_f_ratio > 64.64
                return 1.20 + f_factor * 1.931;
              }
            }
          }
          else{ // u_ekf_count > 195.5
            if(u_sat_number_0 <= 15.5) {
              if(f_pdop <= 1.24) {
                return 1.40 + f_factor * 1.248;
              }
              else{ // f_pdop > 1.24
                return 0.30 + f_factor * 0.189;
              }
            }
            else{ // u_sat_number_0 > 15.5
              return 11.32;
            }
          }
        }
        else{ // u_ekf_count > 209.5
          if(f_avg_cn0_1 <= 33.77) {
            if(f_code_uwrmse_0 <= 3.3) {
              if(f_pdop_ekf_pr <= 1.06) {
                return 0.22 + f_factor * 0.095;
              }
              else{ // f_pdop_ekf_pr > 1.06
                return 0.06 + f_factor * 0.014;
              }
            }
            else{ // f_code_uwrmse_0 > 3.3
              return 0.47 + f_factor * 0.361;
            }
          }
          else{ // f_avg_cn0_1 > 33.77
            if(f_prefix_pdop <= 2.39) {
              if(ewl_f_adop <= 1.04) {
                return 0.33 + f_factor * 0.292;
              }
              else{ // ewl_f_adop > 1.04
                return 0.17 + f_factor * 0.074;
              }
            }
            else{ // f_prefix_pdop > 2.39
              if(f_kf_std_0 <= 0.07) {
                return 0.17 + f_factor * 0.085;
              }
              else{ // f_kf_std_0 > 0.07
                return 0.25 + f_factor * 0.133;
              }
            }
          }
        }
      }
    }
  }
  else{ // u_opensky_count > 8.5
    if(u_phase_number_1 <= 39.5) {
      if(u_closesky_count <= 0.5) {
        if(f_code_uwrmse_0 <= 1.74) {
          if(f_avg_cn0_1 <= 37.59) {
            if(f_avg_cn0_1 <= 31.69) {
              if(f_pdop <= 1.75) {
                return 0.14 + f_factor * 0.231;
              }
              else{ // f_pdop > 1.75
                return 1.00 + f_factor * 1.591;
              }
            }
            else{ // f_avg_cn0_1 > 31.69
              if(u_fix_count <= 18.5) {
                return 0.15 + f_factor * 0.591;
              }
              else{ // u_fix_count > 18.5
                return 0.09 + f_factor * 0.089;
              }
            }
          }
          else{ // f_avg_cn0_1 > 37.59
            if(f_pdop <= 2.22) {
              if(wl_f_rss_0 <= 890.28) {
                return 0.19 + f_factor * 0.288;
              }
              else{ // wl_f_rss_0 > 890.28
                return 0.79 + f_factor * 1.625;
              }
            }
            else{ // f_pdop > 2.22
              if(ewl_f_pdop <= 3.25) {
                return 0.40 + f_factor * 0.332;
              }
              else{ // ewl_f_pdop > 3.25
                return 1.08 + f_factor * 0.845;
              }
            }
          }
        }
        else{ // f_code_uwrmse_0 > 1.74
          if(f_avg_cn0_1 <= 30.4) {
            if(f_pdop_ekf_cp <= 1.67) {
              if(f_pdop_ekf_pr <= 1.08) {
                return 0.17;
              }
              else{ // f_pdop_ekf_pr > 1.08
                return 0.59 + f_factor * 0.709;
              }
            }
            else{ // f_pdop_ekf_cp > 1.67
              if(f_code_uwrmse_0 <= 3.48) {
                return 0.47 + f_factor * 2.032;
              }
              else{ // f_code_uwrmse_0 > 3.48
                return 3.97 + f_factor * 6.163;
              }
            }
          }
          else{ // f_avg_cn0_1 > 30.4
            if(u_fix_count <= 10.5) {
              if(u_phase_number_1 <= 23.5) {
                return 1.17 + f_factor * 2.608;
              }
              else{ // u_phase_number_1 > 23.5
                return 0.34 + f_factor * 0.709;
              }
            }
            else{ // u_fix_count > 10.5
              if(f_avg_cn0_0 <= 37.45) {
                return 0.16 + f_factor * 0.496;
              }
              else{ // f_avg_cn0_0 > 37.45
                return 0.35 + f_factor * 0.788;
              }
            }
          }
        }
      }
      else{ // u_closesky_count > 0.5
        if(u_sat_number_1 <= 15.5) {
          if(f_avg_cn0_0 <= 30.38) {
            if(f_code_uwrmse_0 <= 3.32) {
              if(f_adop_fix <= 0.66) {
                return 1.00 + f_factor * 1.575;
              }
              else{ // f_adop_fix > 0.66
                return 0.33 + f_factor * 0.556;
              }
            }
            else{ // f_code_uwrmse_0 > 3.32
              if(f_kf_std_0 <= 0.12) {
                return 3.47 + f_factor * 5.330;
              }
              else{ // f_kf_std_0 > 0.12
                return 0.69 + f_factor * 0.807;
              }
            }
          }
          else{ // f_avg_cn0_0 > 30.38
            if(wl_f_rss_0 <= 362.76) {
              if(f_code_uwrmse_1 <= 2.77) {
                return 0.37 + f_factor * 0.563;
              }
              else{ // f_code_uwrmse_1 > 2.77
                return 0.56 + f_factor * 0.707;
              }
            }
            else{ // wl_f_rss_0 > 362.76
              if(f_prefix_pdop <= 6.51) {
                return 0.71 + f_factor * 0.588;
              }
              else{ // f_prefix_pdop > 6.51
                return 2.90 + f_factor * 2.224;
              }
            }
          }
        }
        else{ // u_sat_number_1 > 15.5
          if(u_code_number_0 <= 49.5) {
            if(f_avg_cn0_1 <= 30.39) {
              if(f_kf_std_0 <= 0.04) {
                return 2.87 + f_factor * 4.409;
              }
              else{ // f_kf_std_0 > 0.04
                return 0.42 + f_factor * 0.815;
              }
            }
            else{ // f_avg_cn0_1 > 30.39
              if(f_avg_cn0_0 <= 35.76) {
                return 0.17 + f_factor * 0.231;
              }
              else{ // f_avg_cn0_0 > 35.76
                return 0.29 + f_factor * 0.429;
              }
            }
          }
          else{ // u_code_number_0 > 49.5
            if(u_common_sat <= 28.5) {
              if(f_code_uwrmse_1 <= 0.87) {
                return 4.24 + f_factor * 5.267;
              }
              else{ // f_code_uwrmse_1 > 0.87
                return 1.36 + f_factor * 2.182;
              }
            }
            else{ // u_common_sat > 28.5
              if(f_avg_cn0_0 <= 38.02) {
                return 0.35 + f_factor * 0.047;
              }
              else{ // f_avg_cn0_0 > 38.02
                return 0.18 + f_factor * 0.066;
              }
            }
          }
        }
      }
    }
    else{ // u_phase_number_1 > 39.5
      if(f_avg_cn0_0 <= 37.32) {
        if(f_avg_cn0_1 <= 32.03) {
          if(f_pdop_ekf_cp <= 0.81) {
            if(f_avg_lock_1 <= 17.78) {
              if(f_code_uwrmse_0 <= 1.25) {
                return 0.13 + f_factor * 0.087;
              }
              else{ // f_code_uwrmse_0 > 1.25
                return 0.28 + f_factor * 0.210;
              }
            }
            else{ // f_avg_lock_1 > 17.78
              if(u_sat_number_1 <= 26.5) {
                return 0.40 + f_factor * 0.266;
              }
              else{ // u_sat_number_1 > 26.5
                return 0.55 + f_factor * 0.468;
              }
            }
          }
          else{ // f_pdop_ekf_cp > 0.81
            if(u_ekf_count <= 40.5) {
              return 1.02 + f_factor * 0.404;
            }
            else{ // u_ekf_count > 40.5
              if(u_sat_number_0 <= 17.5) {
                return 0.06 + f_factor * 0.019;
              }
              else{ // u_sat_number_0 > 17.5
                return 0.19 + f_factor * 0.149;
              }
            }
          }
        }
        else{ // f_avg_cn0_1 > 32.03
          if(u_closesky_count <= 2.5) {
            if(f_code_uwrmse_0 <= 3.04) {
              if(f_adop_fix <= 0.44) {
                return 0.46 + f_factor * 0.078;
              }
              else{ // f_adop_fix > 0.44
                return 0.05 + f_factor * 0.008;
              }
            }
            else{ // f_code_uwrmse_0 > 3.04
              if(u_prefix_sig_number <= 11.5) {
                return 0.53 + f_factor * 0.542;
              }
              else{ // u_prefix_sig_number > 11.5
                return 0.18 + f_factor * 0.052;
              }
            }
          }
          else{ // u_closesky_count > 2.5
            if(u_ekf_count <= 253.5) {
              if(u_code_number_0 <= 17.5) {
                return 0.51 + f_factor * 0.169;
              }
              else{ // u_code_number_0 > 17.5
                return 0.15 + f_factor * 0.082;
              }
            }
            else{ // u_ekf_count > 253.5
              if(ewl_f_rss_0 <= 155.92) {
                return 0.09 + f_factor * 0.048;
              }
              else{ // ewl_f_rss_0 > 155.92
                return 0.36 + f_factor * 0.057;
              }
            }
          }
        }
      }
      else{ // f_avg_cn0_0 > 37.32
        if(f_avg_cn0_0 <= 42.5) {
          if(u_sat_number_0 <= 7.5) {
            if(u_ekf_count <= 100.5) {
              if(u_phase_number_1 <= 55.5) {
                return 0.31 + f_factor * 0.669;
              }
              else{ // u_phase_number_1 > 55.5
                return 2.52 + f_factor * 2.354;
              }
            }
            else{ // u_ekf_count > 100.5
              if(f_code_uwrmse_1 <= 2.17) {
                return 0.17 + f_factor * 0.211;
              }
              else{ // f_code_uwrmse_1 > 2.17
                return 0.59 + f_factor * 0.885;
              }
            }
          }
          else{ // u_sat_number_0 > 7.5
            if(f_prefix_age <= 0.6) {
              if(f_pdop <= 0.69) {
                return 0.23 + f_factor * 0.045;
              }
              else{ // f_pdop > 0.69
                return 0.15 + f_factor * 0.085;
              }
            }
            else{ // f_prefix_age > 0.6
              if(f_pdop <= 1.38) {
                return 0.08 + f_factor * 0.067;
              }
              else{ // f_pdop > 1.38
                return 0.19 + f_factor * 0.320;
              }
            }
          }
        }
        else{ // f_avg_cn0_0 > 42.5
          if(f_avg_cn0_1 <= 45.3) {
            if(u_common_sat <= 37.5) {
              if(f_kf_std_0 <= 0.01) {
                return 0.07 + f_factor * 0.022;
              }
              else{ // f_kf_std_0 > 0.01
                return 0.15 + f_factor * 0.162;
              }
            }
            else{ // u_common_sat > 37.5
              if(u_sat_number_1 <= 33.5) {
                return 0.41 + f_factor * 0.276;
              }
              else{ // u_sat_number_1 > 33.5
                return 0.09 + f_factor * 0.040;
              }
            }
          }
          else{ // f_avg_cn0_1 > 45.3
            if(u_ekf_count <= 209.5) {
              if(wl_f_ratio <= 4.79) {
                return 1.89 + f_factor * 1.491;
              }
              else{ // wl_f_ratio > 4.79
                return 1.32 + f_factor * 0.745;
              }
            }
            else{ // u_ekf_count > 209.5
              if(u_code_number_0 <= 39.5) {
                return 0.11;
              }
              else{ // u_code_number_0 > 39.5
                return 0.05;
              }
            }
          }
        }
      }
    }
  }
}

static double predict_fix_0(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:5, n_leaves:32
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  float f_code_uwrmse_0 = pz_feat->f_code_uwrmse[0];
  uint8_t u_prefix_sat_number = pz_feat->u_prefix_sat_number;
  float f_previous_ratio = pz_feat->f_previous_ratio;
  uint8_t u_code_number_1 = pz_feat->u_code_number[1];
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  uint8_t u_common_sat = pz_feat->u_common_sat;
  uint8_t u_sat_number_1 = pz_feat->u_sat_number[1];
  uint8_t u_phase_number_1 = pz_feat->u_phase_number[1];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  uint8_t u_sat_number_0 = pz_feat->u_sat_number[0];
  float f_ratio = pz_feat->f_ratio;
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  uint8_t u_previous_fix_sat = pz_feat->u_previous_fix_sat;
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  uint8_t u_opensky_count = pz_feat->u_opensky_count;
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  float f_avg_cn0_0 = pz_feat->f_avg_cn0[0];
  float f_avg_lock_0 = pz_feat->f_avg_lock[0];

  if(u_opensky_count <= 12.5) {
    if(u_sat_number_1 <= 15.5) {
      if(u_ekf_count <= 188.5) {
        if(u_opensky_count <= 1.5) {
          if(u_common_sat <= 26.5) {
            return 1.14 + f_factor * 0.906;
          }
          else{ // u_common_sat > 26.5
            return 3.65 + f_factor * 25.291;
          }
        }
        else{ // u_opensky_count > 1.5
          if(f_avg_lock_0 <= 17.3) {
            return 0.49 + f_factor * 0.501;
          }
          else{ // f_avg_lock_0 > 17.3
            return 1.11 + f_factor * 1.679;
          }
        }
      }
      else{ // u_ekf_count > 188.5
        if(f_avg_cn0_1 <= 34.55) {
          if(f_avg_cn0_0 <= 33.28) {
            return 0.34 + f_factor * 0.606;
          }
          else{ // f_avg_cn0_0 > 33.28
            return 0.21 + f_factor * 0.118;
          }
        }
        else{ // f_avg_cn0_1 > 34.55
          if(u_code_number_1 <= 19.5) {
            return 0.54 + f_factor * 0.636;
          }
          else{ // u_code_number_1 > 19.5
            return 0.32 + f_factor * 0.335;
          }
        }
      }
    }
    else{ // u_sat_number_1 > 15.5
      if(u_sat_number_0 <= 14.5) {
        if(u_ekf_count <= 97.5) {
          if(f_avg_cn0_0 <= 38.16) {
            return 0.28 + f_factor * 0.330;
          }
          else{ // f_avg_cn0_0 > 38.16
            return 0.60 + f_factor * 0.774;
          }
        }
        else{ // u_ekf_count > 97.5
          if(f_avg_cn0_0 <= 37.28) {
            return 0.21 + f_factor * 0.247;
          }
          else{ // f_avg_cn0_0 > 37.28
            return 0.34 + f_factor * 0.384;
          }
        }
      }
      else{ // u_sat_number_0 > 14.5
        if(f_avg_cn0_1 <= 31.49) {
          if(f_pdop_ekf_pr <= 1.34) {
            return 1.35 + f_factor * 2.207;
          }
          else{ // f_pdop_ekf_pr > 1.34
            return 0.46 + f_factor * 0.587;
          }
        }
        else{ // f_avg_cn0_1 > 31.49
          if(f_kf_std_1 <= 2.92) {
            return 0.22 + f_factor * 0.261;
          }
          else{ // f_kf_std_1 > 2.92
            return 1.11 + f_factor * 3.857;
          }
        }
      }
    }
  }
  else{ // u_opensky_count > 12.5
    if(f_code_uwrmse_1 <= 3.34) {
      if(u_prefix_sat_number <= 9.5) {
        if(f_code_uwrmse_0 <= 1.55) {
          if(u_opensky_count <= 16.5) {
            return 0.23 + f_factor * 0.408;
          }
          else{ // u_opensky_count > 16.5
            return 0.11 + f_factor * 0.088;
          }
        }
        else{ // f_code_uwrmse_0 > 1.55
          if(u_sat_number_0 <= 9.5) {
            return 0.36 + f_factor * 0.932;
          }
          else{ // u_sat_number_0 > 9.5
            return 0.18 + f_factor * 0.285;
          }
        }
      }
      else{ // u_prefix_sat_number > 9.5
        if(f_avg_cn0_0 <= 39.0) {
          if(u_sat_number_1 <= 18.5) {
            return 0.13 + f_factor * 0.338;
          }
          else{ // u_sat_number_1 > 18.5
            return 0.06 + f_factor * 0.021;
          }
        }
        else{ // f_avg_cn0_0 > 39.0
          if(f_code_uwrmse_0 <= 1.25) {
            return 0.09 + f_factor * 0.037;
          }
          else{ // f_code_uwrmse_0 > 1.25
            return 0.13 + f_factor * 0.122;
          }
        }
      }
    }
    else{ // f_code_uwrmse_1 > 3.34
      if(f_pdop_ekf_pr <= 1.0) {
        if(u_phase_number_1 <= 55.5) {
          if(f_ratio <= 19.62) {
            return 0.20 + f_factor * 0.544;
          }
          else{ // f_ratio > 19.62
            return 1.63 + f_factor * 3.671;
          }
        }
        else{ // u_phase_number_1 > 55.5
          if(f_previous_ratio <= 13.63) {
            return 3.30 + f_factor * 4.113;
          }
          else{ // f_previous_ratio > 13.63
            return 4.09 + f_factor * 3.835;
          }
        }
      }
      else{ // f_pdop_ekf_pr > 1.0
        if(u_previous_fix_sat <= 9.5) {
          if(f_kf_std_1 <= 0.02) {
            return 0.90 + f_factor * 0.928;
          }
          else{ // f_kf_std_1 > 0.02
            return 0.41 + f_factor * 0.891;
          }
        }
        else{ // u_previous_fix_sat > 9.5
          if(f_code_uwrmse_0 <= 3.47) {
            return 0.14 + f_factor * 0.219;
          }
          else{ // f_code_uwrmse_0 > 3.47
            return 0.32 + f_factor * 0.845;
          }
        }
      }
    }
  }
}

static double predict_fix_1(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:5, n_leaves:31
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  uint8_t u_code_number_1 = pz_feat->u_code_number[1];
  uint8_t u_common_sat = pz_feat->u_common_sat;
  uint8_t u_sat_number_1 = pz_feat->u_sat_number[1];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  uint8_t u_opensky_count = pz_feat->u_opensky_count;
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  uint8_t u_sat_number_0 = pz_feat->u_sat_number[0];
  float f_avg_cn0_0 = pz_feat->f_avg_cn0[0];
  float f_ratio = pz_feat->f_ratio;
  uint8_t u_closesky_count = pz_feat->u_closesky_count;
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  float f_code_uwrmse_0 = pz_feat->f_code_uwrmse[0];
  float f_avg_lock_0 = pz_feat->f_avg_lock[0];
  uint8_t u_prefix_sat_number = pz_feat->u_prefix_sat_number;
  float f_code_uwrmse_1 = pz_feat->f_code_uwrmse[1];
  uint8_t u_phase_number_1 = pz_feat->u_phase_number[1];
  uint8_t u_dr_number = pz_feat->u_dr_number;
  uint8_t u_previous_fix_sat = pz_feat->u_previous_fix_sat;

  if(u_opensky_count <= 12.5) {
    if(u_sat_number_1 <= 15.5) {
      if(u_ekf_count <= 188.5) {
        if(u_opensky_count <= 1.5) {
          if(u_common_sat <= 26.5) {
            return 1.15 + f_factor * 0.905;
          }
          else{ // u_common_sat > 26.5
            return 3.42 + f_factor * 25.663;
          }
        }
        else{ // u_opensky_count > 1.5
          if(f_avg_lock_0 <= 19.73) {
            return 0.54 + f_factor * 0.760;
          }
          else{ // f_avg_lock_0 > 19.73
            return 1.70 + f_factor * 3.452;
          }
        }
      }
      else{ // u_ekf_count > 188.5
        if(f_avg_cn0_1 <= 34.55) {
          if(f_avg_cn0_0 <= 33.28) {
            return 0.34 + f_factor * 0.534;
          }
          else{ // f_avg_cn0_0 > 33.28
            return 0.21 + f_factor * 0.115;
          }
        }
        else{ // f_avg_cn0_1 > 34.55
          if(u_code_number_1 <= 19.5) {
            return 0.54 + f_factor * 0.627;
          }
          else{ // u_code_number_1 > 19.5
            return 0.31 + f_factor * 0.321;
          }
        }
      }
    }
    else{ // u_sat_number_1 > 15.5
      if(u_sat_number_0 <= 14.5) {
        if(u_ekf_count <= 24.5) {
          if(u_dr_number <= 9.5) {
            return 0.55 + f_factor * 1.045;
          }
          else{ // u_dr_number > 9.5
            return 2.05 + f_factor * 8.366;
          }
        }
        else{ // u_ekf_count > 24.5
          if(f_avg_cn0_0 <= 37.74) {
            return 0.23 + f_factor * 0.242;
          }
          else{ // f_avg_cn0_0 > 37.74
            return 0.35 + f_factor * 0.446;
          }
        }
      }
      else{ // u_sat_number_0 > 14.5
        if(f_kf_std_1 <= 3.02) {
          if(f_avg_cn0_0 <= 33.16) {
            return 0.40 + f_factor * 0.642;
          }
          else{ // f_avg_cn0_0 > 33.16
            return 0.21 + f_factor * 0.210;
          }
        }
        else{ // f_kf_std_1 > 3.02
          return 1.18 + f_factor * 3.924;
        }
      }
    }
  }
  else{ // u_opensky_count > 12.5
    if(f_code_uwrmse_1 <= 3.34) {
      if(u_prefix_sat_number <= 9.5) {
        if(f_code_uwrmse_1 <= 2.33) {
          if(u_closesky_count <= 3.5) {
            return 0.13 + f_factor * 0.146;
          }
          else{ // u_closesky_count > 3.5
            return 0.24 + f_factor * 0.425;
          }
        }
        else{ // f_code_uwrmse_1 > 2.33
          if(f_kf_std_1 <= 0.02) {
            return 0.61 + f_factor * 1.056;
          }
          else{ // f_kf_std_1 > 0.02
            return 0.27 + f_factor * 0.595;
          }
        }
      }
      else{ // u_prefix_sat_number > 9.5
        if(f_avg_cn0_0 <= 39.0) {
          if(f_avg_cn0_1 <= 31.69) {
            return 0.31 + f_factor * 0.653;
          }
          else{ // f_avg_cn0_1 > 31.69
            return 0.07 + f_factor * 0.053;
          }
        }
        else{ // f_avg_cn0_0 > 39.0
          if(f_code_uwrmse_0 <= 1.25) {
            return 0.09 + f_factor * 0.037;
          }
          else{ // f_code_uwrmse_0 > 1.25
            return 0.13 + f_factor * 0.118;
          }
        }
      }
    }
    else{ // f_code_uwrmse_1 > 3.34
      if(f_pdop_ekf_pr <= 1.0) {
        if(u_phase_number_1 <= 55.5) {
          if(f_ratio <= 19.62) {
            return 0.20 + f_factor * 0.560;
          }
          else{ // f_ratio > 19.62
            return 1.78 + f_factor * 3.990;
          }
        }
        else{ // u_phase_number_1 > 55.5
          if(f_avg_lock_1 <= 18.26) {
            return 3.43 + f_factor * 3.922;
          }
          else{ // f_avg_lock_1 > 18.26
            return 4.32 + f_factor * 3.823;
          }
        }
      }
      else{ // f_pdop_ekf_pr > 1.0
        if(u_previous_fix_sat <= 9.5) {
          if(f_kf_std_1 <= 0.02) {
            return 0.91 + f_factor * 0.934;
          }
          else{ // f_kf_std_1 > 0.02
            return 0.42 + f_factor * 0.987;
          }
        }
        else{ // u_previous_fix_sat > 9.5
          if(f_code_uwrmse_0 <= 3.89) {
            return 0.16 + f_factor * 0.275;
          }
          else{ // f_code_uwrmse_0 > 3.89
            return 0.36 + f_factor * 0.982;
          }
        }
      }
    }
  }
}

static double predict_fix_2(const rtk_filterInfo_t* pz_RTKfilterInfo, const float f_factor)
{
  // max_depth:5, n_leaves:31
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;
  const integ_RtkFeature_t* pz_feat = &pz_RTKfilterInfo->pz_integ->z_rtk_feature;

  uint8_t u_sol_bit_count = pz_integ->u_sol_bit_count;
  uint8_t u_code_number_1 = pz_feat->u_code_number[1];
  uint8_t u_common_sat = pz_feat->u_common_sat;
  uint8_t u_sat_number_1 = pz_feat->u_sat_number[1];
  float f_avg_lock_1 = pz_feat->f_avg_lock[1];
  float f_pdop = pz_feat->f_pdop;
  float f_pdop_ekf_pr = pz_feat->f_pdop_ekf_pr;
  float f_pdop_ekf_cp = pz_feat->f_pdop_ekf_cp;
  uint8_t u_sat_number_0 = pz_feat->u_sat_number[0];
  float f_kf_std_1 = pz_feat->f_kf_std[1];
  uint8_t u_ekf_count = pz_integ->u_ekf_count;
  uint8_t u_previous_fix_sat = pz_feat->u_previous_fix_sat;
  float f_avg_cn0_1 = pz_feat->f_avg_cn0[1];
  uint8_t u_opensky_count = pz_feat->u_opensky_count;
  float f_code_uwrmse_0 = pz_feat->f_code_uwrmse[0];
  float f_avg_cn0_0 = pz_feat->f_avg_cn0[0];
  float f_avg_lock_0 = pz_feat->f_avg_lock[0];

  if(u_opensky_count <= 12.5) {
    if(u_sat_number_1 <= 15.5) {
      if(u_ekf_count <= 188.5) {
        if(u_opensky_count <= 1.5) {
          if(f_avg_cn0_1 <= 34.44) {
            return 0.64 + f_factor * 0.475;
          }
          else{ // f_avg_cn0_1 > 34.44
            return 1.37 + f_factor * 3.096;
          }
        }
        else{ // u_opensky_count > 1.5
          if(f_avg_lock_0 <= 19.73) {
            return 0.54 + f_factor * 0.693;
          }
          else{ // f_avg_lock_0 > 19.73
            return 1.62 + f_factor * 3.327;
          }
        }
      }
      else{ // u_ekf_count > 188.5
        if(f_avg_cn0_1 <= 34.55) {
          if(f_avg_cn0_0 <= 33.28) {
            return 0.33 + f_factor * 0.486;
          }
          else{ // f_avg_cn0_0 > 33.28
            return 0.21 + f_factor * 0.119;
          }
        }
        else{ // f_avg_cn0_1 > 34.55
          if(u_code_number_1 <= 19.5) {
            return 0.54 + f_factor * 0.639;
          }
          else{ // u_code_number_1 > 19.5
            return 0.32 + f_factor * 0.334;
          }
        }
      }
    }
    else{ // u_sat_number_1 > 15.5
      if(u_opensky_count <= 5.5) {
        if(u_ekf_count <= 34.5) {
          if(u_common_sat <= 27.5) {
            return 0.87 + f_factor * 4.025;
          }
          else{ // u_common_sat > 27.5
            return 0.33 + f_factor * 0.923;
          }
        }
        else{ // u_ekf_count > 34.5
          if(f_avg_lock_1 <= 9.82) {
            return 0.30 + f_factor * 0.284;
          }
          else{ // f_avg_lock_1 > 9.82
            return 0.45 + f_factor * 0.454;
          }
        }
      }
      else{ // u_opensky_count > 5.5
        if(u_sat_number_1 <= 23.5) {
          if(u_ekf_count <= 98.5) {
            return 0.42 + f_factor * 0.999;
          }
          else{ // u_ekf_count > 98.5
            return 0.25 + f_factor * 0.315;
          }
        }
        else{ // u_sat_number_1 > 23.5
          if(f_avg_cn0_1 <= 32.34) {
            return 0.75 + f_factor * 1.702;
          }
          else{ // f_avg_cn0_1 > 32.34
            return 0.16 + f_factor * 0.104;
          }
        }
      }
    }
  }
  else{ // u_opensky_count > 12.5
    if(f_code_uwrmse_0 <= 2.34) {
      if(u_previous_fix_sat <= 9.5) {
        if(f_avg_cn0_0 <= 38.16) {
          if(u_ekf_count <= 88.5) {
            return 0.28 + f_factor * 0.602;
          }
          else{ // u_ekf_count > 88.5
            return 0.09 + f_factor * 0.068;
          }
        }
        else{ // f_avg_cn0_0 > 38.16
          if(f_pdop <= 1.11) {
            return 0.12 + f_factor * 0.138;
          }
          else{ // f_pdop > 1.11
            return 0.26 + f_factor * 0.486;
          }
        }
      }
      else{ // u_previous_fix_sat > 9.5
        if(f_avg_cn0_0 <= 38.92) {
          if(f_avg_cn0_1 <= 31.94) {
            return 0.24 + f_factor * 0.364;
          }
          else{ // f_avg_cn0_1 > 31.94
            return 0.07 + f_factor * 0.051;
          }
        }
        else{ // f_avg_cn0_0 > 38.92
          if(f_code_uwrmse_0 <= 1.25) {
            return 0.09 + f_factor * 0.038;
          }
          else{ // f_code_uwrmse_0 > 1.25
            return 0.13 + f_factor * 0.118;
          }
        }
      }
    }
    else{ // f_code_uwrmse_0 > 2.34
      if(f_pdop_ekf_pr <= 0.93) {
        if(f_pdop_ekf_cp <= 0.75) {
          return 3.54 + f_factor * 4.113;
        }
        else{ // f_pdop_ekf_cp > 0.75
          if(f_pdop_ekf_cp <= 0.84) {
            return 1.12 + f_factor * 2.886;
          }
          else{ // f_pdop_ekf_cp > 0.84
            return 0.20 + f_factor * 0.350;
          }
        }
      }
      else{ // f_pdop_ekf_pr > 0.93
        if(u_sat_number_0 <= 9.5) {
          if(f_kf_std_1 <= 0.02) {
            return 0.83 + f_factor * 1.166;
          }
          else{ // f_kf_std_1 > 0.02
            return 0.36 + f_factor * 0.818;
          }
        }
        else{ // u_sat_number_0 > 9.5
          if(u_sol_bit_count <= 17.5) {
            return 0.24 + f_factor * 0.471;
          }
          else{ // u_sol_bit_count > 17.5
            return 0.05;
          }
        }
      }
    }
  }
}



/**
 * @brief RTK Decision Tree predict STD
 * @param[in] pz_RTKfilterInfo RTK filer information
 */
extern float rtk_dt_STDPredict(const rtk_filterInfo_t* pz_RTKfilterInfo)
{
  float f_factor = 1.0f;
  double d_list[10] = {0.0};
  double d_value = 0.0;
  double d_pdop_max = 0.0;
  double d_pdop_min = 0.0;
  const integ_RtkIntegrity_t* pz_integ = pz_RTKfilterInfo->pz_integ;

  d_list[0] = pz_integ->z_rtk_feature.f_pdop;
  d_list[1] = pz_integ->z_rtk_feature.f_pdop_ekf_pr;
  d_list[2] = pz_integ->z_rtk_feature.f_pdop_ekf_cp;
  d_pdop_max = vector_dbl_max(d_list, 3);
  d_pdop_min = vector_dbl_min(d_list, 3);

  if (pz_RTKfilterInfo->u_solTag == RTK_FILTER_POS_FLOAT ||
    pz_RTKfilterInfo->u_solTag == RTK_FILTER_POS_RTD)
  {
    // FLOAT, RTD
    d_list[0] = predict_float_rtd_0(pz_RTKfilterInfo, f_factor);
    d_list[1] = predict_float_rtd_1(pz_RTKfilterInfo, f_factor);
    d_list[2] = predict_float_rtd_2(pz_RTKfilterInfo, f_factor);
    d_list[3] = pz_integ->z_rtk_feature.f_kf_std[0] * 2.0;
    d_value = gnss_MeanDouble(d_list, 4);
    d_value *= d_pdop_max;
  }

  else if (pz_RTKfilterInfo->u_solTag == RTK_FILTER_POS_FIX_WL ||
    pz_RTKfilterInfo->u_solTag == RTK_FILTER_POS_FIX_EWL ||
    pz_RTKfilterInfo->u_solTag == RTK_FILTER_POS_PREFIX)
  {
    // WL, EWL, PREFIX
    d_list[0] = predict_wl_ewl_prefix_0(pz_RTKfilterInfo, f_factor);
    d_list[1] = predict_wl_ewl_prefix_1(pz_RTKfilterInfo, f_factor);
    d_list[2] = predict_wl_ewl_prefix_2(pz_RTKfilterInfo, f_factor);
    d_value = gnss_MeanDouble(d_list, 3);
  }
  else if (pz_RTKfilterInfo->u_solTag == RTK_FILTER_POS_FIX)
  {
    // FIX
    d_list[0] = predict_fix(pz_RTKfilterInfo, f_factor);
    d_list[1] = predict_fix_0(pz_RTKfilterInfo, f_factor);
    d_list[2] = predict_fix_1(pz_RTKfilterInfo, f_factor);
    d_list[3] = predict_fix_2(pz_RTKfilterInfo, f_factor);
    d_value = gnss_MeanDouble(d_list, 4);
    d_value *= d_pdop_min;
  }
  else
  {
    d_value = 99.9;
  }
  return (float)d_value;
}
