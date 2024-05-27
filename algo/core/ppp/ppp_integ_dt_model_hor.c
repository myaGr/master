/**@file        ppp_integ_dt_model_hor.c
 * @brief       PPP Horizontal Integrity Decision Tree Model Created by PYTHON
 * @details
 * @author      houxiaowei
 * @date        2023/5/16
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/5/16   <td>0.1      <td>houxiaowei    <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "mw_alloc.h"
#include "gnss_common.h"
#include "ppp_integ_model.h"
#include "ppp_integrity.h"

#define DT_IMPURITY_FACTOR 2.0
static double predict_float_0(integ_PppFeature_t* gpz_pppFeature) {
  uint8_t openSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_openSkyCount);
  uint8_t u_SvTrackCount = gpz_pppFeature->z_SvStatus.u_SvTrackCount;
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  float f_age = gpz_pppFeature->f_age;
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  float avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t closeSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount);
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;

  if (pl_ekf_hor <= 2.27) {
    if (f_cn0_avg <= 37.25) {
      if (u_ppp_use_pr_num_float <= 15.5) {
        if (f_postCodeSTD <= 1.61) {
          if (openSkyCount <= 58.5) {
            if (f_cn0_avg <= 29.25) {
              if (pl_ekf_hor <= 1.51) {
                return 1.99 + DT_IMPURITY_FACTOR * 1.392;
              }
              else { // pl_ekf_hor > 1.51
                return 5.37;
              }
            }
            else { // f_cn0_avg > 29.25
              if (pl_ekf_hor <= 1.22) {
                return 0.87 + DT_IMPURITY_FACTOR * 0.685;
              }
              else { // pl_ekf_hor > 1.22
                return 1.76 + DT_IMPURITY_FACTOR * 1.277;
              }
            }
          }
          else { // openSkyCount > 58.5
            if (pl_ekf_hor <= 2.21) {
              if (pl_ekf_hor <= 0.8) {
                return 2.37 + DT_IMPURITY_FACTOR * 0.897;
              }
              else { // pl_ekf_hor > 0.8
                return 3.72 + DT_IMPURITY_FACTOR * 6.449;
              }
            }
            else { // pl_ekf_hor > 2.21
              if (f_postPhaseSTD <= 0.03) {
                return 8.29;
              }
              else { // f_postPhaseSTD > 0.03
                return 82.63;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.61
          if (pl_ekf_ver <= 0.92) {
            if (openSkyCount <= 60.5) {
              if (u_satMeasCount <= 24.5) {
                return 1.28 + DT_IMPURITY_FACTOR * 0.780;
              }
              else { // u_satMeasCount > 24.5
                return 2.59 + DT_IMPURITY_FACTOR * 2.071;
              }
            }
            else { // openSkyCount > 60.5
              if (pl_ekf_hor <= 1.04) {
                return 3.19 + DT_IMPURITY_FACTOR * 1.109;
              }
              else { // pl_ekf_hor > 1.04
                return 5.76;
              }
            }
          }
          else { // pl_ekf_ver > 0.92
            if (closeSkyCount <= 6.5) {
              if (f_cmc_avg <= 1.88) {
                return 44.12;
              }
              else { // f_cmc_avg > 1.88
                return 6.16;
              }
            }
            else { // closeSkyCount > 6.5
              if (f_hdop <= 1.03) {
                return 4.85 + DT_IMPURITY_FACTOR * 3.206;
              }
              else { // f_hdop > 1.03
                return 2.67 + DT_IMPURITY_FACTOR * 2.187;
              }
            }
          }
        }
      }
      else { // u_ppp_use_pr_num_float > 15.5
        if (pl_ekf_hor <= 1.25) {
          if (f_cmc_avg <= 1.8) {
            if (pl_ekf_hor <= 0.99) {
              if (u_ppp_use_pr_num_float <= 27.5) {
                return 0.87 + DT_IMPURITY_FACTOR * 0.762;
              }
              else { // u_ppp_use_pr_num_float > 27.5
                return 0.46 + DT_IMPURITY_FACTOR * 0.168;
              }
            }
            else { // pl_ekf_hor > 0.99
              if (f_cmc_std <= 1.47) {
                return 1.10 + DT_IMPURITY_FACTOR * 0.897;
              }
              else { // f_cmc_std > 1.47
                return 1.66 + DT_IMPURITY_FACTOR * 2.365;
              }
            }
          }
          else { // f_cmc_avg > 1.8
            if (u_CN040 <= 12.5) {
              if (u_MeasInUseCount <= 33.5) {
                return 2.71 + DT_IMPURITY_FACTOR * 1.265;
              }
              else { // u_MeasInUseCount > 33.5
                return 1.62 + DT_IMPURITY_FACTOR * 0.939;
              }
            }
            else { // u_CN040 > 12.5
              if (pl_ekf_hor <= 1.06) {
                return 1.08 + DT_IMPURITY_FACTOR * 0.786;
              }
              else { // pl_ekf_hor > 1.06
                return 2.42 + DT_IMPURITY_FACTOR * 3.134;
              }
            }
          }
        }
        else { // pl_ekf_hor > 1.25
          if (closeSkyCount <= 5.5) {
            if (openSkyCount <= 59.5) {
              if (pl_ekf_ver <= 1.12) {
                return 3.00 + DT_IMPURITY_FACTOR * 2.856;
              }
              else { // pl_ekf_ver > 1.12
                return 108.12;
              }
            }
            else { // openSkyCount > 59.5
              if (f_age <= 24.6) {
                return 3.13 + DT_IMPURITY_FACTOR * 10.765;
              }
              else { // f_age > 24.6
                return 8.35;
              }
            }
          }
          else { // closeSkyCount > 5.5
            if (f_postCodeSTD <= 1.07) {
              if (f_postCodeSTD <= 0.77) {
                return 1.37 + DT_IMPURITY_FACTOR * 0.565;
              }
              else { // f_postCodeSTD > 0.77
                return 1.87 + DT_IMPURITY_FACTOR * 0.716;
              }
            }
            else { // f_postCodeSTD > 1.07
              if (pl_ekf_hor <= 1.75) {
                return 2.37 + DT_IMPURITY_FACTOR * 1.761;
              }
              else { // pl_ekf_hor > 1.75
                return 3.25 + DT_IMPURITY_FACTOR * 1.638;
              }
            }
          }
        }
      }
    }
    else { // f_cn0_avg > 37.25
      if (pl_ekf_hor <= 1.15) {
        if (pl_ekf_hor <= 0.53) {
          if (pl_ekf_hor <= 0.34) {
            if (f_postCodeSTD <= 0.96) {
              if (f_postCodeSTD <= 0.44) {
                return 0.18 + DT_IMPURITY_FACTOR * 0.013;
              }
              else { // f_postCodeSTD > 0.44
                return 0.24 + DT_IMPURITY_FACTOR * 0.035;
              }
            }
            else { // f_postCodeSTD > 0.96
              if (avg_unc_pr <= 4.93) {
                return 0.30 + DT_IMPURITY_FACTOR * 0.055;
              }
              else { // avg_unc_pr > 4.93
                return 0.45 + DT_IMPURITY_FACTOR * 0.103;
              }
            }
          }
          else { // pl_ekf_hor > 0.34
            if (f_postCodeSTD <= 0.5) {
              if (f_postCodeSTD <= 0.39) {
                return 0.26 + DT_IMPURITY_FACTOR * 0.023;
              }
              else { // f_postCodeSTD > 0.39
                return 0.33 + DT_IMPURITY_FACTOR * 0.056;
              }
            }
            else { // f_postCodeSTD > 0.5
              if (f_postCodeSTD <= 1.16) {
                return 0.41 + DT_IMPURITY_FACTOR * 0.122;
              }
              else { // f_postCodeSTD > 1.16
                return 0.67 + DT_IMPURITY_FACTOR * 0.336;
              }
            }
          }
        }
        else { // pl_ekf_hor > 0.53
          if (f_postCodeSTD <= 0.56) {
            if (f_postCodeSTD <= 0.39) {
              if (pl_ekf_hor <= 0.77) {
                return 0.37 + DT_IMPURITY_FACTOR * 0.058;
              }
              else { // pl_ekf_hor > 0.77
                return 0.52 + DT_IMPURITY_FACTOR * 0.089;
              }
            }
            else { // f_postCodeSTD > 0.39
              if (u_SvTrackCount <= 22.5) {
                return 1.97 + DT_IMPURITY_FACTOR * 0.093;
              }
              else { // u_SvTrackCount > 22.5
                return 0.56 + DT_IMPURITY_FACTOR * 0.126;
              }
            }
          }
          else { // f_postCodeSTD > 0.56
            if (f_postCodeSTD <= 1.09) {
              if (pl_ekf_hor <= 0.78) {
                return 0.64 + DT_IMPURITY_FACTOR * 0.258;
              }
              else { // pl_ekf_hor > 0.78
                return 0.90 + DT_IMPURITY_FACTOR * 0.424;
              }
            }
            else { // f_postCodeSTD > 1.09
              if (u_SvTrackCount <= 25.5) {
                return 3.03 + DT_IMPURITY_FACTOR * 1.991;
              }
              else { // u_SvTrackCount > 25.5
                return 1.05 + DT_IMPURITY_FACTOR * 0.813;
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 1.15
        if (f_postCodeSTD <= 0.97) {
          if (f_postCodeSTD <= 0.69) {
            if (f_cmc_avg <= 6.26) {
              if (f_postCodeSTD <= 0.56) {
                return 0.81 + DT_IMPURITY_FACTOR * 0.353;
              }
              else { // f_postCodeSTD > 0.56
                return 1.10 + DT_IMPURITY_FACTOR * 0.525;
              }
            }
            else { // f_cmc_avg > 6.26
              if (f_age <= 5.75) {
                return 14.99;
              }
              else { // f_age > 5.75
                return 2.01 + DT_IMPURITY_FACTOR * 3.210;
              }
            }
          }
          else { // f_postCodeSTD > 0.69
            if (f_cmc_avg <= 5.3) {
              if (pl_ekf_hor <= 1.49) {
                return 1.20 + DT_IMPURITY_FACTOR * 0.529;
              }
              else { // pl_ekf_hor > 1.49
                return 1.68 + DT_IMPURITY_FACTOR * 0.680;
              }
            }
            else { // f_cmc_avg > 5.3
              if (u_CN040 <= 19.5) {
                return 6.01;
              }
              else { // u_CN040 > 19.5
                return 1.66 + DT_IMPURITY_FACTOR * 0.762;
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.97
          if (f_postCodeSTD <= 1.53) {
            if (pl_ekf_ver <= 1.08) {
              if (closeSkyCount <= 0.5) {
                return 2.71 + DT_IMPURITY_FACTOR * 4.360;
              }
              else { // closeSkyCount > 0.5
                return 1.59 + DT_IMPURITY_FACTOR * 0.537;
              }
            }
            else { // pl_ekf_ver > 1.08
              if (pl_ekf_hor <= 1.38) {
                return 4.59 + DT_IMPURITY_FACTOR * 11.056;
              }
              else { // pl_ekf_hor > 1.38
                return 2.65 + DT_IMPURITY_FACTOR * 1.112;
              }
            }
          }
          else { // f_postCodeSTD > 1.53
            if (openSkyCount <= 63.5) {
              if (pl_ekf_hor <= 1.77) {
                return 2.25 + DT_IMPURITY_FACTOR * 3.755;
              }
              else { // pl_ekf_hor > 1.77
                return 3.95 + DT_IMPURITY_FACTOR * 1.652;
              }
            }
            else { // openSkyCount > 63.5
              if (u_MeasInUseCount <= 32.5) {
                return 4.73 + DT_IMPURITY_FACTOR * 12.386;
              }
              else { // u_MeasInUseCount > 32.5
                return 22.93;
              }
            }
          }
        }
      }
    }
  }
  else { // pl_ekf_hor > 2.27
    if (closeSkyCount <= 0.5) {
      if (f_cn0_avg <= 36.75) {
        if (f_cmc_avg <= 1.13) {
          if (u_MeasInUseCount <= 22.5) {
            if (u_CN040 <= 14.0) {
              if (pl_ekf_hor <= 8.58) {
                return 97.06;
              }
              else { // pl_ekf_hor > 8.58
                return 78.99;
              }
            }
            else { // u_CN040 > 14.0
              if (pl_ekf_ver <= 2.48) {
                return 1.22 + DT_IMPURITY_FACTOR * 0.066;
              }
              else { // pl_ekf_ver > 2.48
                return 0.89 + DT_IMPURITY_FACTOR * 0.006;
              }
            }
          }
          else { // u_MeasInUseCount > 22.5
            if (pl_ekf_ver <= 1.98) {
              if (f_age <= 10.95) {
                return 140.15;
              }
              else { // f_age > 10.95
                return 88.19;
              }
            }
            else { // pl_ekf_ver > 1.98
              if (u_satMeasCount <= 22.5) {
                return 90.70;
              }
              else { // u_satMeasCount > 22.5
                return 114.31;
              }
            }
          }
        }
        else { // f_cmc_avg > 1.13
          if (pl_ekf_ver <= 1.44) {
            if (f_postCodeSTD <= 1.51) {
              return 133.21;
            }
            else { // f_postCodeSTD > 1.51
              if (f_postCodeSTD <= 2.19) {
                return 144.37;
              }
              else { // f_postCodeSTD > 2.19
                return 140.98;
              }
            }
          }
          else { // pl_ekf_ver > 1.44
            if (pl_ekf_ver <= 1.44) {
              return 60.44;
            }
            else { // pl_ekf_ver > 1.44
              if (f_cmc_std <= 1.0) {
                return 37.66;
              }
              else { // f_cmc_std > 1.0
                return 7.14;
              }
            }
          }
        }
      }
      else { // f_cn0_avg > 36.75
        if (pl_ekf_hor <= 22.14) {
          if (pl_ekf_hor <= 3.69) {
            if (f_postCodeSTD <= 1.02) {
              if (u_ppp_use_pr_num_float <= 8.5) {
                return 4.30 + DT_IMPURITY_FACTOR * 3.479;
              }
              else { // u_ppp_use_pr_num_float > 8.5
                return 1.28 + DT_IMPURITY_FACTOR * 0.705;
              }
            }
            else { // f_postCodeSTD > 1.02
              if (f_postPhaseSTD <= 0.02) {
                return 3.81 + DT_IMPURITY_FACTOR * 1.245;
              }
              else { // f_postPhaseSTD > 0.02
                return 13.50;
              }
            }
          }
          else { // pl_ekf_hor > 3.69
            if (f_cn0_avg <= 37.25) {
              if (f_gdop <= 5.86) {
                return 86.38;
              }
              else { // f_gdop > 5.86
                return 5.26;
              }
            }
            else { // f_cn0_avg > 37.25
              if (f_cn0_std <= 8.53) {
                return 8.12;
              }
              else { // f_cn0_std > 8.53
                return 15.48;
              }
            }
          }
        }
        else { // pl_ekf_hor > 22.14
          if (pl_ekf_ver <= 19.03) {
            if (f_gdop <= 6.8) {
              if (f_cn0_avg <= 40.75) {
                return 58.72;
              }
              else { // f_cn0_avg > 40.75
                return 56.38;
              }
            }
            else { // f_gdop > 6.8
              if (pl_ekf_hor <= 24.3) {
                return 68.20;
              }
              else { // pl_ekf_hor > 24.3
                return 64.73;
              }
            }
          }
          else { // pl_ekf_ver > 19.03
            if (f_postCodeSTD <= 1.42) {
              if (pl_ekf_ver <= 22.59) {
                return 53.29;
              }
              else { // pl_ekf_ver > 22.59
                return 47.15;
              }
            }
            else { // f_postCodeSTD > 1.42
              return 36.10;
            }
          }
        }
      }
    }
    else { // closeSkyCount > 0.5
      if (pl_ekf_ver <= 3.98) {
        if (avg_unc_pr <= 8.26) {
          if (f_postCodeSTD <= 1.03) {
            if (f_postCodeSTD <= 0.67) {
              if (pl_ekf_hor <= 3.07) {
                return 1.22 + DT_IMPURITY_FACTOR * 0.586;
              }
              else { // pl_ekf_hor > 3.07
                return 2.08 + DT_IMPURITY_FACTOR * 3.132;
              }
            }
            else { // f_postCodeSTD > 0.67
              if (u_ppp_use_pr_num_float <= 16.5) {
                return 2.80 + DT_IMPURITY_FACTOR * 2.710;
              }
              else { // u_ppp_use_pr_num_float > 16.5
                return 1.63 + DT_IMPURITY_FACTOR * 0.687;
              }
            }
          }
          else { // f_postCodeSTD > 1.03
            if (pl_ekf_hor <= 3.09) {
              if (u_MeasInUseCount <= 39.5) {
                return 3.15 + DT_IMPURITY_FACTOR * 4.110;
              }
              else { // u_MeasInUseCount > 39.5
                return 5.15;
              }
            }
            else { // pl_ekf_hor > 3.09
              if (f_cmc_avg <= 2.01) {
                return 4.17 + DT_IMPURITY_FACTOR * 2.031;
              }
              else { // f_cmc_avg > 2.01
                return 6.28;
              }
            }
          }
        }
        else { // avg_unc_pr > 8.26
          if (pl_ekf_hor <= 2.99) {
            if (openSkyCount <= 61.5) {
              if (u_MeasTrackCount <= 54.5) {
                return 3.98 + DT_IMPURITY_FACTOR * 4.009;
              }
              else { // u_MeasTrackCount > 54.5
                return 6.16;
              }
            }
            else { // openSkyCount > 61.5
              if (f_cn0_avg <= 32.25) {
                return 6.78;
              }
              else { // f_cn0_avg > 32.25
                return 59.07;
              }
            }
          }
          else { // pl_ekf_hor > 2.99
            if (u_MeasTrackCount <= 55.5) {
              if (u_CN040 <= 6.5) {
                return 7.63;
              }
              else { // u_CN040 > 6.5
                return 5.01;
              }
            }
            else { // u_MeasTrackCount > 55.5
              if (f_postPhaseSTD <= 0.0) {
                return 6.95;
              }
              else { // f_postPhaseSTD > 0.0
                return 14.22;
              }
            }
          }
        }
      }
      else { // pl_ekf_ver > 3.98
        if (f_postCodeSTD <= 1.71) {
          if (u_ppp_use_pr_num_float <= 9.5) {
            if (f_cn0_std <= 9.27) {
              if (f_cn0_std <= 2.59) {
                return 4.00 + DT_IMPURITY_FACTOR * 2.642;
              }
              else { // f_cn0_std > 2.59
                return 10.59;
              }
            }
            else { // f_cn0_std > 9.27
              if (avg_unc_pr <= 23.02) {
                return 6.36;
              }
              else { // avg_unc_pr > 23.02
                return 14.31;
              }
            }
          }
          else { // u_ppp_use_pr_num_float > 9.5
            if (f_hdop <= 1.67) {
              if (u_MeasTrackCount <= 39.5) {
                return 2.32 + DT_IMPURITY_FACTOR * 1.996;
              }
              else { // u_MeasTrackCount > 39.5
                return 5.54;
              }
            }
            else { // f_hdop > 1.67
              if (u_CN040 <= 8.5) {
                return 8.57;
              }
              else { // u_CN040 > 8.5
                return 5.60;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.71
          if (f_cn0_std <= 8.53) {
            if (avg_unc_pr <= 14.74) {
              if (u_MeasInUseCount <= 12.5) {
                return 30.59;
              }
              else { // u_MeasInUseCount > 12.5
                return 11.12;
              }
            }
            else { // avg_unc_pr > 14.74
              if (f_cn0_avg <= 35.25) {
                return 15.00;
              }
              else { // f_cn0_avg > 35.25
                return 30.44;
              }
            }
          }
          else { // f_cn0_std > 8.53
            if (avg_unc_pr <= 21.67) {
              if (f_postPhaseSTD <= 0.01) {
                return 9.15;
              }
              else { // f_postPhaseSTD > 0.01
                return 5.86;
              }
            }
            else { // avg_unc_pr > 21.67
              if (f_cmc_std <= 0.31) {
                return 31.25;
              }
              else { // f_cmc_std > 0.31
                return 13.73;
              }
            }
          }
        }
      }
    }
  }
}

static double predict_float_1(integ_PppFeature_t* gpz_pppFeature) {
  uint8_t openSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_openSkyCount);
  uint8_t u_SvTrackCount = gpz_pppFeature->z_SvStatus.u_SvTrackCount;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  float f_age = gpz_pppFeature->f_age;
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  uint8_t z_sceneType = gpz_pppFeature->z_sceneType;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  float avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t closeSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount);
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  uint8_t u_ns = gpz_pppFeature->u_ns;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;

  if (pl_ekf_hor <= 2.27) {
    if (f_cn0_avg <= 37.25) {
      if (u_ppp_use_pr_num_float <= 18.5) {
        if (f_postCodeSTD <= 1.62) {
          if (closeSkyCount <= 2.5) {
            if (pl_ekf_hor <= 2.21) {
              if (pl_ekf_hor <= 0.96) {
                return 2.32 + DT_IMPURITY_FACTOR * 1.136;
              }
              else { // pl_ekf_hor > 0.96
                return 3.90 + DT_IMPURITY_FACTOR * 6.268;
              }
            }
            else { // pl_ekf_hor > 2.21
              if (u_ppp_use_pr_num_float <= 14.0) {
                return 8.20;
              }
              else { // u_ppp_use_pr_num_float > 14.0
                return 106.65;
              }
            }
          }
          else { // closeSkyCount > 2.5
            if (pl_ekf_hor <= 1.27) {
              if (pl_ekf_ver <= 0.21) {
                return 4.58 + DT_IMPURITY_FACTOR * 0.923;
              }
              else { // pl_ekf_ver > 0.21
                return 1.00 + DT_IMPURITY_FACTOR * 0.980;
              }
            }
            else { // pl_ekf_hor > 1.27
              if (f_cn0_avg <= 29.75) {
                return 4.46 + DT_IMPURITY_FACTOR * 2.596;
              }
              else { // f_cn0_avg > 29.75
                return 1.85 + DT_IMPURITY_FACTOR * 1.939;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.62
          if (pl_ekf_hor <= 1.13) {
            if (openSkyCount <= 63.5) {
              if (f_cn0_avg <= 29.25) {
                return 2.44 + DT_IMPURITY_FACTOR * 1.477;
              }
              else { // f_cn0_avg > 29.25
                return 1.11 + DT_IMPURITY_FACTOR * 0.532;
              }
            }
            else { // openSkyCount > 63.5
              if (u_CN040 <= 7.5) {
                return 3.71 + DT_IMPURITY_FACTOR * 0.996;
              }
              else { // u_CN040 > 7.5
                return 2.52 + DT_IMPURITY_FACTOR * 1.363;
              }
            }
          }
          else { // pl_ekf_hor > 1.13
            if (closeSkyCount <= 5.5) {
              if (closeSkyCount <= 4.5) {
                return 7.49;
              }
              else { // closeSkyCount > 4.5
                return 74.25;
              }
            }
            else { // closeSkyCount > 5.5
              if (f_cn0_avg <= 29.25) {
                return 4.57 + DT_IMPURITY_FACTOR * 4.807;
              }
              else { // f_cn0_avg > 29.25
                return 2.83 + DT_IMPURITY_FACTOR * 2.355;
              }
            }
          }
        }
      }
      else { // u_ppp_use_pr_num_float > 18.5
        if (pl_ekf_hor <= 1.25) {
          if (f_cmc_std <= 1.62) {
            if (pl_ekf_hor <= 0.89) {
              if (u_ppp_use_pr_num_float <= 27.5) {
                return 0.73 + DT_IMPURITY_FACTOR * 0.502;
              }
              else { // u_ppp_use_pr_num_float > 27.5
                return 0.39 + DT_IMPURITY_FACTOR * 0.089;
              }
            }
            else { // pl_ekf_hor > 0.89
              if (f_cmc_avg <= 1.62) {
                return 1.00 + DT_IMPURITY_FACTOR * 0.311;
              }
              else { // f_cmc_avg > 1.62
                return 1.67 + DT_IMPURITY_FACTOR * 0.778;
              }
            }
          }
          else { // f_cmc_std > 1.62
            if (f_postCodeSTD <= 1.42) {
              if (f_cn0_avg <= 33.25) {
                return 1.57 + DT_IMPURITY_FACTOR * 0.746;
              }
              else { // f_cn0_avg > 33.25
                return 0.86 + DT_IMPURITY_FACTOR * 0.728;
              }
            }
            else { // f_postCodeSTD > 1.42
              if (pl_ekf_hor <= 0.79) {
                return 1.35 + DT_IMPURITY_FACTOR * 0.814;
              }
              else { // pl_ekf_hor > 0.79
                return 2.12 + DT_IMPURITY_FACTOR * 1.509;
              }
            }
          }
        }
        else { // pl_ekf_hor > 1.25
          if (f_postCodeSTD <= 1.2) {
            if (f_cmc_avg <= 3.92) {
              if (f_postCodeSTD <= 0.96) {
                return 1.62 + DT_IMPURITY_FACTOR * 0.613;
              }
              else { // f_postCodeSTD > 0.96
                return 2.17 + DT_IMPURITY_FACTOR * 0.703;
              }
            }
            else { // f_cmc_avg > 3.92
              if (f_cn0_avg <= 32.5) {
                return 10.16;
              }
              else { // f_cn0_avg > 32.5
                return 2.22 + DT_IMPURITY_FACTOR * 0.857;
              }
            }
          }
          else { // f_postCodeSTD > 1.2
            if (pl_ekf_hor <= 1.78) {
              if (z_sceneType <= 2.5) {
                return 4.67 + DT_IMPURITY_FACTOR * 3.081;
              }
              else { // z_sceneType > 2.5
                return 2.27 + DT_IMPURITY_FACTOR * 0.853;
              }
            }
            else { // pl_ekf_hor > 1.78
              if (avg_unc_pr <= 9.83) {
                return 3.34 + DT_IMPURITY_FACTOR * 1.030;
              }
              else { // avg_unc_pr > 9.83
                return 4.28 + DT_IMPURITY_FACTOR * 1.822;
              }
            }
          }
        }
      }
    }
    else { // f_cn0_avg > 37.25
      if (pl_ekf_hor <= 1.15) {
        if (pl_ekf_hor <= 0.53) {
          if (pl_ekf_hor <= 0.35) {
            if (f_postCodeSTD <= 1.01) {
              if (f_postCodeSTD <= 0.45) {
                return 0.18 + DT_IMPURITY_FACTOR * 0.013;
              }
              else { // f_postCodeSTD > 0.45
                return 0.24 + DT_IMPURITY_FACTOR * 0.037;
              }
            }
            else { // f_postCodeSTD > 1.01
              if (avg_unc_pr <= 4.98) {
                return 0.32 + DT_IMPURITY_FACTOR * 0.064;
              }
              else { // avg_unc_pr > 4.98
                return 0.46 + DT_IMPURITY_FACTOR * 0.104;
              }
            }
          }
          else { // pl_ekf_hor > 0.35
            if (f_postCodeSTD <= 0.5) {
              if (f_postCodeSTD <= 0.38) {
                return 0.26 + DT_IMPURITY_FACTOR * 0.022;
              }
              else { // f_postCodeSTD > 0.38
                return 0.33 + DT_IMPURITY_FACTOR * 0.055;
              }
            }
            else { // f_postCodeSTD > 0.5
              if (f_postCodeSTD <= 1.15) {
                return 0.41 + DT_IMPURITY_FACTOR * 0.124;
              }
              else { // f_postCodeSTD > 1.15
                return 0.67 + DT_IMPURITY_FACTOR * 0.341;
              }
            }
          }
        }
        else { // pl_ekf_hor > 0.53
          if (f_postCodeSTD <= 0.56) {
            if (f_postCodeSTD <= 0.39) {
              if (pl_ekf_hor <= 0.77) {
                return 0.37 + DT_IMPURITY_FACTOR * 0.059;
              }
              else { // pl_ekf_hor > 0.77
                return 0.52 + DT_IMPURITY_FACTOR * 0.088;
              }
            }
            else { // f_postCodeSTD > 0.39
              if (u_SvTrackCount <= 22.5) {
                return 1.99 + DT_IMPURITY_FACTOR * 0.061;
              }
              else { // u_SvTrackCount > 22.5
                return 0.56 + DT_IMPURITY_FACTOR * 0.126;
              }
            }
          }
          else { // f_postCodeSTD > 0.56
            if (f_postCodeSTD <= 1.09) {
              if (pl_ekf_hor <= 0.78) {
                return 0.64 + DT_IMPURITY_FACTOR * 0.259;
              }
              else { // pl_ekf_hor > 0.78
                return 0.90 + DT_IMPURITY_FACTOR * 0.415;
              }
            }
            else { // f_postCodeSTD > 1.09
              if (u_SvTrackCount <= 25.5) {
                return 3.07 + DT_IMPURITY_FACTOR * 1.989;
              }
              else { // u_SvTrackCount > 25.5
                return 1.06 + DT_IMPURITY_FACTOR * 0.827;
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 1.15
        if (f_postCodeSTD <= 0.97) {
          if (f_postCodeSTD <= 0.69) {
            if (f_cmc_avg <= 6.26) {
              if (f_postCodeSTD <= 0.56) {
                return 0.82 + DT_IMPURITY_FACTOR * 0.360;
              }
              else { // f_postCodeSTD > 0.56
                return 1.10 + DT_IMPURITY_FACTOR * 0.523;
              }
            }
            else { // f_cmc_avg > 6.26
              if (f_age <= 5.8) {
                return 15.01;
              }
              else { // f_age > 5.8
                return 2.12 + DT_IMPURITY_FACTOR * 3.169;
              }
            }
          }
          else { // f_postCodeSTD > 0.69
            if (pl_ekf_ver <= 1.12) {
              if (u_SvTrackCount <= 23.5) {
                return 4.28 + DT_IMPURITY_FACTOR * 0.847;
              }
              else { // u_SvTrackCount > 23.5
                return 1.27 + DT_IMPURITY_FACTOR * 0.541;
              }
            }
            else { // pl_ekf_ver > 1.12
              if (f_cmc_avg <= 5.07) {
                return 1.72 + DT_IMPURITY_FACTOR * 0.743;
              }
              else { // f_cmc_avg > 5.07
                return 5.04;
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.97
          if (f_postCodeSTD <= 1.53) {
            if (pl_ekf_ver <= 1.1) {
              if (closeSkyCount <= 0.5) {
                return 2.74 + DT_IMPURITY_FACTOR * 4.719;
              }
              else { // closeSkyCount > 0.5
                return 1.64 + DT_IMPURITY_FACTOR * 0.594;
              }
            }
            else { // pl_ekf_ver > 1.1
              if (pl_ekf_hor <= 1.38) {
                return 6.66;
              }
              else { // pl_ekf_hor > 1.38
                return 2.67 + DT_IMPURITY_FACTOR * 1.114;
              }
            }
          }
          else { // f_postCodeSTD > 1.53
            if (closeSkyCount <= 0.5) {
              if (u_MeasInUseCount <= 32.5) {
                return 4.69 + DT_IMPURITY_FACTOR * 12.566;
              }
              else { // u_MeasInUseCount > 32.5
                return 21.52;
              }
            }
            else { // closeSkyCount > 0.5
              if (pl_ekf_hor <= 1.77) {
                return 2.25 + DT_IMPURITY_FACTOR * 3.168;
              }
              else { // pl_ekf_hor > 1.77
                return 3.98 + DT_IMPURITY_FACTOR * 1.626;
              }
            }
          }
        }
      }
    }
  }
  else { // pl_ekf_hor > 2.27
    if (closeSkyCount <= 0.5) {
      if (f_cn0_avg <= 36.75) {
        if (f_cmc_avg <= 1.56) {
          if (u_CN040 <= 14.0) {
            if (pl_ekf_ver <= 2.03) {
              if (f_age <= 11.05) {
                return 139.20;
              }
              else { // f_age > 11.05
                return 92.23;
              }
            }
            else { // pl_ekf_ver > 2.03
              if (pl_ekf_ver <= 6.48) {
                return 100.13;
              }
              else { // pl_ekf_ver > 6.48
                return 80.48;
              }
            }
          }
          else { // u_CN040 > 14.0
            if (f_cmc_avg <= 0.81) {
              return 1.40 + DT_IMPURITY_FACTOR * 0.097;
            }
            else { // f_cmc_avg > 0.81
              if (u_MeasTrackCount <= 42.5) {
                return 0.70 + DT_IMPURITY_FACTOR * 0.052;
              }
              else { // u_MeasTrackCount > 42.5
                return 0.90 + DT_IMPURITY_FACTOR * 0.008;
              }
            }
          }
        }
        else { // f_cmc_avg > 1.56
          if (pl_ekf_ver <= 1.44) {
            if (pl_ekf_hor <= 2.49) {
              return 140.98;
            }
            else { // pl_ekf_hor > 2.49
              if (f_cn0_std <= 10.75) {
                return 144.30;
              }
              else { // f_cn0_std > 10.75
                return 144.44;
              }
            }
          }
          else { // pl_ekf_ver > 1.44
            if (f_cmc_avg <= 1.61) {
              return 39.88;
            }
            else { // f_cmc_avg > 1.61
              if (pl_ekf_ver <= 1.76) {
                return 9.62;
              }
              else { // pl_ekf_ver > 1.76
                return 5.21;
              }
            }
          }
        }
      }
      else { // f_cn0_avg > 36.75
        if (pl_ekf_hor <= 22.14) {
          if (pl_ekf_hor <= 3.69) {
            if (f_postCodeSTD <= 1.02) {
              if (u_ppp_use_pr_num_float <= 8.5) {
                return 4.24 + DT_IMPURITY_FACTOR * 3.436;
              }
              else { // u_ppp_use_pr_num_float > 8.5
                return 1.27 + DT_IMPURITY_FACTOR * 0.667;
              }
            }
            else { // f_postCodeSTD > 1.02
              if (f_postPhaseSTD <= 0.02) {
                return 3.82 + DT_IMPURITY_FACTOR * 1.252;
              }
              else { // f_postPhaseSTD > 0.02
                return 11.96;
              }
            }
          }
          else { // pl_ekf_hor > 3.69
            if (f_cn0_avg <= 37.25) {
              if (u_ns <= 8.0) {
                return 4.77 + DT_IMPURITY_FACTOR * 4.241;
              }
              else { // u_ns > 8.0
                return 86.27;
              }
            }
            else { // f_cn0_avg > 37.25
              if (f_cn0_std <= 8.53) {
                return 8.19;
              }
              else { // f_cn0_std > 8.53
                return 15.39;
              }
            }
          }
        }
        else { // pl_ekf_hor > 22.14
          if (f_cn0_std <= 7.78) {
            if (f_cmc_avg <= 0.42) {
              return 44.80;
            }
            else { // f_cmc_avg > 0.42
              return 28.45;
            }
          }
          else { // f_cn0_std > 7.78
            if (f_cn0_avg <= 38.25) {
              if (f_age <= 7.35) {
                return 64.69;
              }
              else { // f_age > 7.35
                return 68.38;
              }
            }
            else { // f_cn0_avg > 38.25
              if (pl_ekf_hor <= 34.82) {
                return 57.50;
              }
              else { // pl_ekf_hor > 34.82
                return 52.68;
              }
            }
          }
        }
      }
    }
    else { // closeSkyCount > 0.5
      if (pl_ekf_ver <= 3.97) {
        if (avg_unc_pr <= 8.41) {
          if (f_postCodeSTD <= 1.03) {
            if (f_postCodeSTD <= 0.67) {
              if (pl_ekf_hor <= 3.07) {
                return 1.21 + DT_IMPURITY_FACTOR * 0.579;
              }
              else { // pl_ekf_hor > 3.07
                return 2.12 + DT_IMPURITY_FACTOR * 3.177;
              }
            }
            else { // f_postCodeSTD > 0.67
              if (u_ppp_use_pr_num_float <= 16.5) {
                return 2.86 + DT_IMPURITY_FACTOR * 2.990;
              }
              else { // u_ppp_use_pr_num_float > 16.5
                return 1.62 + DT_IMPURITY_FACTOR * 0.674;
              }
            }
          }
          else { // f_postCodeSTD > 1.03
            if (pl_ekf_hor <= 3.09) {
              if (u_MeasInUseCount <= 39.5) {
                return 3.17 + DT_IMPURITY_FACTOR * 4.091;
              }
              else { // u_MeasInUseCount > 39.5
                return 5.20;
              }
            }
            else { // pl_ekf_hor > 3.09
              if (f_cmc_avg <= 1.94) {
                return 4.12 + DT_IMPURITY_FACTOR * 2.027;
              }
              else { // f_cmc_avg > 1.94
                return 6.27;
              }
            }
          }
        }
        else { // avg_unc_pr > 8.41
          if (pl_ekf_hor <= 2.99) {
            if (u_MeasTrackCount <= 54.5) {
              if (f_cn0_avg <= 30.25) {
                return 5.90;
              }
              else { // f_cn0_avg > 30.25
                return 3.59 + DT_IMPURITY_FACTOR * 2.168;
              }
            }
            else { // u_MeasTrackCount > 54.5
              if (f_cmc_avg <= 5.73) {
                return 5.87;
              }
              else { // f_cmc_avg > 5.73
                return 12.61;
              }
            }
          }
          else { // pl_ekf_hor > 2.99
            if (u_MeasTrackCount <= 55.5) {
              if (u_CN040 <= 6.5) {
                return 7.59;
              }
              else { // u_CN040 > 6.5
                return 5.04;
              }
            }
            else { // u_MeasTrackCount > 55.5
              if (f_postPhaseSTD <= 0.0) {
                return 7.19;
              }
              else { // f_postPhaseSTD > 0.0
                return 14.30;
              }
            }
          }
        }
      }
      else { // pl_ekf_ver > 3.97
        if (u_ppp_use_pr_num_float <= 10.5) {
          if (f_cn0_std <= 10.01) {
            if (f_postCodeSTD <= 1.2) {
              if (u_MeasTrackCount <= 35.5) {
                return 6.53;
              }
              else { // u_MeasTrackCount > 35.5
                return 10.54;
              }
            }
            else { // f_postCodeSTD > 1.2
              if (pl_ekf_ver <= 32.96) {
                return 11.93;
              }
              else { // pl_ekf_ver > 32.96
                return 23.51;
              }
            }
          }
          else { // f_cn0_std > 10.01
            if (u_MeasTrackCount <= 31.5) {
              if (u_SvTrackCount <= 24.5) {
                return 11.19;
              }
              else { // u_SvTrackCount > 24.5
                return 4.56 + DT_IMPURITY_FACTOR * 2.829;
              }
            }
            else { // u_MeasTrackCount > 31.5
              if (u_SvTrackCount <= 28.5) {
                return 2.77 + DT_IMPURITY_FACTOR * 4.714;
              }
              else { // u_SvTrackCount > 28.5
                return 7.67;
              }
            }
          }
        }
        else { // u_ppp_use_pr_num_float > 10.5
          if (f_postCodeSTD <= 1.75) {
            if (f_hdop <= 1.67) {
              if (u_MeasTrackCount <= 39.5) {
                return 2.17 + DT_IMPURITY_FACTOR * 1.573;
              }
              else { // u_MeasTrackCount > 39.5
                return 5.35;
              }
            }
            else { // f_hdop > 1.67
              if (f_postCodeSTD <= 0.7) {
                return 3.14 + DT_IMPURITY_FACTOR * 4.746;
              }
              else { // f_postCodeSTD > 0.7
                return 7.35;
              }
            }
          }
          else { // f_postCodeSTD > 1.75
            if (u_CN040 <= 6.5) {
              if (closeSkyCount <= 58.5) {
                return 2.63 + DT_IMPURITY_FACTOR * 0.859;
              }
              else { // closeSkyCount > 58.5
                return 13.96;
              }
            }
            else { // u_CN040 > 6.5
              if (pl_ekf_hor <= 6.51) {
                return 4.17 + DT_IMPURITY_FACTOR * 2.357;
              }
              else { // pl_ekf_hor > 6.51
                return 9.90;
              }
            }
          }
        }
      }
    }
  }
}

static double predict_float_2(integ_PppFeature_t* gpz_pppFeature) {
  uint8_t openSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_openSkyCount);
  uint8_t u_SvTrackCount = gpz_pppFeature->z_SvStatus.u_SvTrackCount;
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  float f_age = gpz_pppFeature->f_age;
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  float avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t closeSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount);
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  uint8_t u_ns = gpz_pppFeature->u_ns;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;

  if (pl_ekf_hor <= 2.27) {
    if (f_cn0_avg <= 37.25) {
      if (u_ppp_use_pr_num_float <= 15.5) {
        if (f_postCodeSTD <= 1.61) {
          if (closeSkyCount <= 5.5) {
            if (pl_ekf_hor <= 2.21) {
              if (pl_ekf_hor <= 0.8) {
                return 2.37 + DT_IMPURITY_FACTOR * 0.892;
              }
              else { // pl_ekf_hor > 0.8
                return 3.71 + DT_IMPURITY_FACTOR * 6.325;
              }
            }
            else { // pl_ekf_hor > 2.21
              if (u_SvTrackCount <= 33.0) {
                return 106.66;
              }
              else { // u_SvTrackCount > 33.0
                return 8.34;
              }
            }
          }
          else { // closeSkyCount > 5.5
            if (f_cn0_avg <= 29.25) {
              if (pl_ekf_hor <= 1.51) {
                return 1.99 + DT_IMPURITY_FACTOR * 1.399;
              }
              else { // pl_ekf_hor > 1.51
                return 5.31;
              }
            }
            else { // f_cn0_avg > 29.25
              if (pl_ekf_hor <= 1.22) {
                return 0.88 + DT_IMPURITY_FACTOR * 0.726;
              }
              else { // pl_ekf_hor > 1.22
                return 1.75 + DT_IMPURITY_FACTOR * 1.277;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.61
          if (pl_ekf_ver <= 0.92) {
            if (openSkyCount <= 61.5) {
              if (u_satMeasCount <= 24.5) {
                return 1.28 + DT_IMPURITY_FACTOR * 0.782;
              }
              else { // u_satMeasCount > 24.5
                return 2.56 + DT_IMPURITY_FACTOR * 2.025;
              }
            }
            else { // openSkyCount > 61.5
              if (pl_ekf_hor <= 1.04) {
                return 3.20 + DT_IMPURITY_FACTOR * 1.128;
              }
              else { // pl_ekf_hor > 1.04
                return 5.79;
              }
            }
          }
          else { // pl_ekf_ver > 0.92
            if (closeSkyCount <= 6.5) {
              if (f_cmc_avg <= 1.88) {
                return 44.23;
              }
              else { // f_cmc_avg > 1.88
                return 6.26;
              }
            }
            else { // closeSkyCount > 6.5
              if (f_hdop <= 1.03) {
                return 4.82 + DT_IMPURITY_FACTOR * 3.233;
              }
              else { // f_hdop > 1.03
                return 2.68 + DT_IMPURITY_FACTOR * 2.318;
              }
            }
          }
        }
      }
      else { // u_ppp_use_pr_num_float > 15.5
        if (pl_ekf_hor <= 1.25) {
          if (f_cmc_avg <= 1.8) {
            if (u_ppp_use_pr_num_float <= 27.5) {
              if (u_ns <= 20.5) {
                return 0.87 + DT_IMPURITY_FACTOR * 0.669;
              }
              else { // u_ns > 20.5
                return 1.52 + DT_IMPURITY_FACTOR * 0.728;
              }
            }
            else { // u_ppp_use_pr_num_float > 27.5
              if (pl_ekf_ver <= 0.53) {
                return 0.41 + DT_IMPURITY_FACTOR * 0.149;
              }
              else { // pl_ekf_ver > 0.53
                return 0.95 + DT_IMPURITY_FACTOR * 0.173;
              }
            }
          }
          else { // f_cmc_avg > 1.8
            if (u_CN040 <= 12.5) {
              if (u_MeasInUseCount <= 33.5) {
                return 2.72 + DT_IMPURITY_FACTOR * 1.267;
              }
              else { // u_MeasInUseCount > 33.5
                return 1.64 + DT_IMPURITY_FACTOR * 0.928;
              }
            }
            else { // u_CN040 > 12.5
              if (pl_ekf_hor <= 1.06) {
                return 1.08 + DT_IMPURITY_FACTOR * 0.780;
              }
              else { // pl_ekf_hor > 1.06
                return 2.39 + DT_IMPURITY_FACTOR * 3.092;
              }
            }
          }
        }
        else { // pl_ekf_hor > 1.25
          if (closeSkyCount <= 5.5) {
            if (openSkyCount <= 59.5) {
              if (pl_ekf_ver <= 1.12) {
                return 3.02 + DT_IMPURITY_FACTOR * 2.958;
              }
              else { // pl_ekf_ver > 1.12
                return 104.51;
              }
            }
            else { // openSkyCount > 59.5
              if (f_age <= 24.6) {
                return 3.15 + DT_IMPURITY_FACTOR * 10.700;
              }
              else { // f_age > 24.6
                return 8.39;
              }
            }
          }
          else { // closeSkyCount > 5.5
            if (f_postCodeSTD <= 1.07) {
              if (f_postCodeSTD <= 0.77) {
                return 1.37 + DT_IMPURITY_FACTOR * 0.577;
              }
              else { // f_postCodeSTD > 0.77
                return 1.87 + DT_IMPURITY_FACTOR * 0.721;
              }
            }
            else { // f_postCodeSTD > 1.07
              if (pl_ekf_hor <= 1.75) {
                return 2.35 + DT_IMPURITY_FACTOR * 1.753;
              }
              else { // pl_ekf_hor > 1.75
                return 3.25 + DT_IMPURITY_FACTOR * 1.619;
              }
            }
          }
        }
      }
    }
    else { // f_cn0_avg > 37.25
      if (pl_ekf_hor <= 1.15) {
        if (pl_ekf_hor <= 0.53) {
          if (pl_ekf_hor <= 0.35) {
            if (f_postCodeSTD <= 1.01) {
              if (f_postCodeSTD <= 0.45) {
                return 0.18 + DT_IMPURITY_FACTOR * 0.014;
              }
              else { // f_postCodeSTD > 0.45
                return 0.24 + DT_IMPURITY_FACTOR * 0.037;
              }
            }
            else { // f_postCodeSTD > 1.01
              if (avg_unc_pr <= 4.93) {
                return 0.32 + DT_IMPURITY_FACTOR * 0.063;
              }
              else { // avg_unc_pr > 4.93
                return 0.46 + DT_IMPURITY_FACTOR * 0.104;
              }
            }
          }
          else { // pl_ekf_hor > 0.35
            if (f_postCodeSTD <= 0.51) {
              if (f_postCodeSTD <= 0.38) {
                return 0.26 + DT_IMPURITY_FACTOR * 0.023;
              }
              else { // f_postCodeSTD > 0.38
                return 0.33 + DT_IMPURITY_FACTOR * 0.055;
              }
            }
            else { // f_postCodeSTD > 0.51
              if (f_postCodeSTD <= 1.16) {
                return 0.41 + DT_IMPURITY_FACTOR * 0.125;
              }
              else { // f_postCodeSTD > 1.16
                return 0.67 + DT_IMPURITY_FACTOR * 0.347;
              }
            }
          }
        }
        else { // pl_ekf_hor > 0.53
          if (f_postCodeSTD <= 0.56) {
            if (f_postCodeSTD <= 0.39) {
              if (pl_ekf_hor <= 0.77) {
                return 0.37 + DT_IMPURITY_FACTOR * 0.058;
              }
              else { // pl_ekf_hor > 0.77
                return 0.52 + DT_IMPURITY_FACTOR * 0.090;
              }
            }
            else { // f_postCodeSTD > 0.39
              if (u_SvTrackCount <= 22.5) {
                return 1.98 + DT_IMPURITY_FACTOR * 0.069;
              }
              else { // u_SvTrackCount > 22.5
                return 0.56 + DT_IMPURITY_FACTOR * 0.126;
              }
            }
          }
          else { // f_postCodeSTD > 0.56
            if (f_postCodeSTD <= 1.09) {
              if (pl_ekf_hor <= 0.78) {
                return 0.64 + DT_IMPURITY_FACTOR * 0.259;
              }
              else { // pl_ekf_hor > 0.78
                return 0.90 + DT_IMPURITY_FACTOR * 0.416;
              }
            }
            else { // f_postCodeSTD > 1.09
              if (u_SvTrackCount <= 25.5) {
                return 3.07 + DT_IMPURITY_FACTOR * 1.996;
              }
              else { // u_SvTrackCount > 25.5
                return 1.05 + DT_IMPURITY_FACTOR * 0.810;
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 1.15
        if (f_postCodeSTD <= 0.97) {
          if (f_postCodeSTD <= 0.69) {
            if (f_cmc_avg <= 6.25) {
              if (f_postCodeSTD <= 0.56) {
                return 0.82 + DT_IMPURITY_FACTOR * 0.350;
              }
              else { // f_postCodeSTD > 0.56
                return 1.10 + DT_IMPURITY_FACTOR * 0.514;
              }
            }
            else { // f_cmc_avg > 6.25
              if (f_age <= 5.75) {
                return 15.01;
              }
              else { // f_age > 5.75
                return 1.93 + DT_IMPURITY_FACTOR * 2.002;
              }
            }
          }
          else { // f_postCodeSTD > 0.69
            if (f_cmc_avg <= 5.3) {
              if (pl_ekf_hor <= 1.49) {
                return 1.21 + DT_IMPURITY_FACTOR * 0.554;
              }
              else { // pl_ekf_hor > 1.49
                return 1.69 + DT_IMPURITY_FACTOR * 0.707;
              }
            }
            else { // f_cmc_avg > 5.3
              if (u_CN040 <= 19.5) {
                return 6.11;
              }
              else { // u_CN040 > 19.5
                return 1.63 + DT_IMPURITY_FACTOR * 0.660;
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.97
          if (f_postCodeSTD <= 1.52) {
            if (pl_ekf_ver <= 1.1) {
              if (openSkyCount <= 63.5) {
                return 1.63 + DT_IMPURITY_FACTOR * 0.593;
              }
              else { // openSkyCount > 63.5
                return 2.63 + DT_IMPURITY_FACTOR * 3.692;
              }
            }
            else { // pl_ekf_ver > 1.1
              if (pl_ekf_hor <= 1.38) {
                return 5.95;
              }
              else { // pl_ekf_hor > 1.38
                return 2.67 + DT_IMPURITY_FACTOR * 1.113;
              }
            }
          }
          else { // f_postCodeSTD > 1.52
            if (openSkyCount <= 63.5) {
              if (pl_ekf_hor <= 1.76) {
                return 2.17 + DT_IMPURITY_FACTOR * 1.806;
              }
              else { // pl_ekf_hor > 1.76
                return 3.95 + DT_IMPURITY_FACTOR * 1.622;
              }
            }
            else { // openSkyCount > 63.5
              if (u_MeasInUseCount <= 32.5) {
                return 4.63 + DT_IMPURITY_FACTOR * 11.993;
              }
              else { // u_MeasInUseCount > 32.5
                return 21.52;
              }
            }
          }
        }
      }
    }
  }
  else { // pl_ekf_hor > 2.27
    if (closeSkyCount <= 0.5) {
      if (f_cn0_avg <= 36.75) {
        if (f_cmc_avg <= 1.56) {
          if (u_CN040 <= 14.0) {
            if (pl_ekf_ver <= 2.03) {
              if (f_age <= 11.05) {
                return 139.19;
              }
              else { // f_age > 11.05
                return 81.99;
              }
            }
            else { // pl_ekf_ver > 2.03
              if (u_CN040 <= 8.5) {
                return 7.60;
              }
              else { // u_CN040 > 8.5
                return 91.52;
              }
            }
          }
          else { // u_CN040 > 14.0
            if (f_age <= 13.45) {
              if (f_postCodeSTD <= 0.34) {
                return 0.90 + DT_IMPURITY_FACTOR * 0.006;
              }
              else { // f_postCodeSTD > 0.34
                return 0.68 + DT_IMPURITY_FACTOR * 0.054;
              }
            }
            else { // f_age > 13.45
              return 1.19 + DT_IMPURITY_FACTOR * 0.063;
            }
          }
        }
        else { // f_cmc_avg > 1.56
          if (pl_ekf_ver <= 1.44) {
            if (f_gdop <= 3.57) {
              if (pl_ekf_ver <= 1.42) {
                return 144.42;
              }
              else { // pl_ekf_ver > 1.42
                return 144.29;
              }
            }
            else { // f_gdop > 3.57
              return 140.97;
            }
          }
          else { // pl_ekf_ver > 1.44
            if (f_cmc_std <= 1.0) {
              if (pl_ekf_hor <= 2.52) {
                return 5.67;
              }
              else { // pl_ekf_hor > 2.52
                return 85.64;
              }
            }
            else { // f_cmc_std > 1.0
              if (pl_ekf_ver <= 1.76) {
                return 9.64;
              }
              else { // pl_ekf_ver > 1.76
                return 5.31;
              }
            }
          }
        }
      }
      else { // f_cn0_avg > 36.75
        if (pl_ekf_hor <= 22.14) {
          if (pl_ekf_hor <= 3.69) {
            if (f_postCodeSTD <= 1.02) {
              if (u_ppp_use_pr_num_float <= 8.5) {
                return 4.36 + DT_IMPURITY_FACTOR * 3.456;
              }
              else { // u_ppp_use_pr_num_float > 8.5
                return 1.27 + DT_IMPURITY_FACTOR * 0.714;
              }
            }
            else { // f_postCodeSTD > 1.02
              if (f_postPhaseSTD <= 0.02) {
                return 3.84 + DT_IMPURITY_FACTOR * 1.316;
              }
              else { // f_postPhaseSTD > 0.02
                return 11.99;
              }
            }
          }
          else { // pl_ekf_hor > 3.69
            if (f_cn0_avg <= 37.25) {
              if (u_ns <= 8.0) {
                return 4.23 + DT_IMPURITY_FACTOR * 3.257;
              }
              else { // u_ns > 8.0
                return 86.29;
              }
            }
            else { // f_cn0_avg > 37.25
              if (f_cn0_std <= 8.53) {
                return 8.20;
              }
              else { // f_cn0_std > 8.53
                return 15.35;
              }
            }
          }
        }
        else { // pl_ekf_hor > 22.14
          if (f_cn0_std <= 7.78) {
            if (f_cmc_avg <= 0.42) {
              return 44.50;
            }
            else { // f_cmc_avg > 0.42
              return 28.45;
            }
          }
          else { // f_cn0_std > 7.78
            if (f_cn0_avg <= 38.25) {
              if (f_age <= 7.3) {
                return 64.51;
              }
              else { // f_age > 7.3
                return 68.31;
              }
            }
            else { // f_cn0_avg > 38.25
              if (pl_ekf_hor <= 34.82) {
                return 57.32;
              }
              else { // pl_ekf_hor > 34.82
                return 52.63;
              }
            }
          }
        }
      }
    }
    else { // closeSkyCount > 0.5
      if (pl_ekf_ver <= 3.97) {
        if (avg_unc_pr <= 8.26) {
          if (f_postCodeSTD <= 1.03) {
            if (f_postCodeSTD <= 0.68) {
              if (pl_ekf_hor <= 3.06) {
                return 1.22 + DT_IMPURITY_FACTOR * 0.561;
              }
              else { // pl_ekf_hor > 3.06
                return 2.06 + DT_IMPURITY_FACTOR * 3.016;
              }
            }
            else { // f_postCodeSTD > 0.68
              if (u_ppp_use_pr_num_float <= 16.5) {
                return 2.82 + DT_IMPURITY_FACTOR * 2.738;
              }
              else { // u_ppp_use_pr_num_float > 16.5
                return 1.64 + DT_IMPURITY_FACTOR * 0.689;
              }
            }
          }
          else { // f_postCodeSTD > 1.03
            if (pl_ekf_hor <= 3.09) {
              if (u_MeasInUseCount <= 38.5) {
                return 3.13 + DT_IMPURITY_FACTOR * 4.099;
              }
              else { // u_MeasInUseCount > 38.5
                return 5.06;
              }
            }
            else { // pl_ekf_hor > 3.09
              if (f_cmc_avg <= 1.94) {
                return 4.10 + DT_IMPURITY_FACTOR * 4.916;
              }
              else { // f_cmc_avg > 1.94
                return 6.17;
              }
            }
          }
        }
        else { // avg_unc_pr > 8.26
          if (pl_ekf_hor <= 2.99) {
            if (openSkyCount <= 61.5) {
              if (u_MeasTrackCount <= 54.5) {
                return 3.98 + DT_IMPURITY_FACTOR * 4.069;
              }
              else { // u_MeasTrackCount > 54.5
                return 6.16;
              }
            }
            else { // openSkyCount > 61.5
              if (pl_ekf_hor <= 2.45) {
                return 59.07;
              }
              else { // pl_ekf_hor > 2.45
                return 5.81;
              }
            }
          }
          else { // pl_ekf_hor > 2.99
            if (u_MeasTrackCount <= 55.5) {
              if (u_CN040 <= 6.5) {
                return 7.57;
              }
              else { // u_CN040 > 6.5
                return 5.03;
              }
            }
            else { // u_MeasTrackCount > 55.5
              if (f_postPhaseSTD <= 0.0) {
                return 7.68;
              }
              else { // f_postPhaseSTD > 0.0
                return 14.35;
              }
            }
          }
        }
      }
      else { // pl_ekf_ver > 3.97
        if (f_postCodeSTD <= 1.75) {
          if (u_ppp_use_pr_num_float <= 9.5) {
            if (f_cn0_std <= 9.27) {
              if (f_cn0_std <= 2.59) {
                return 4.14 + DT_IMPURITY_FACTOR * 2.660;
              }
              else { // f_cn0_std > 2.59
                return 10.61;
              }
            }
            else { // f_cn0_std > 9.27
              if (u_MeasTrackCount <= 31.5) {
                return 9.80;
              }
              else { // u_MeasTrackCount > 31.5
                return 5.60;
              }
            }
          }
          else { // u_ppp_use_pr_num_float > 9.5
            if (u_MeasTrackCount <= 39.5) {
              if (f_gdop <= 4.01) {
                return 2.33 + DT_IMPURITY_FACTOR * 2.112;
              }
              else { // f_gdop > 4.01
                return 6.56;
              }
            }
            else { // u_MeasTrackCount > 39.5
              if (f_postCodeSTD <= 1.25) {
                return 4.67 + DT_IMPURITY_FACTOR * 2.508;
              }
              else { // f_postCodeSTD > 1.25
                return 7.89;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.75
          if (f_cn0_std <= 8.53) {
            if (avg_unc_pr <= 14.74) {
              if (u_MeasInUseCount <= 12.5) {
                return 32.95;
              }
              else { // u_MeasInUseCount > 12.5
                return 11.24;
              }
            }
            else { // avg_unc_pr > 14.74
              if (f_cn0_avg <= 35.25) {
                return 14.93;
              }
              else { // f_cn0_avg > 35.25
                return 30.44;
              }
            }
          }
          else { // f_cn0_std > 8.53
            if (u_SvTrackCount <= 24.5) {
              if (f_cmc_avg <= 0.32) {
                return 22.06;
              }
              else { // f_cmc_avg > 0.32
                return 9.96;
              }
            }
            else { // u_SvTrackCount > 24.5
              if (u_satMeasCount <= 16.5) {
                return 3.71 + DT_IMPURITY_FACTOR * 2.396;
              }
              else { // u_satMeasCount > 16.5
                return 8.12;
              }
            }
          }
        }
      }
    }
  }
}

static double predict_float_3(integ_PppFeature_t* gpz_pppFeature) {
  uint8_t openSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_openSkyCount);
  uint8_t u_SvTrackCount = gpz_pppFeature->z_SvStatus.u_SvTrackCount;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  float f_age = gpz_pppFeature->f_age;
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  float avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t closeSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount);
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;

  if (pl_ekf_hor <= 2.28) {
    if (f_cn0_avg <= 37.25) {
      if (u_ppp_use_pr_num_float <= 15.5) {
        if (f_postCodeSTD <= 1.61) {
          if (closeSkyCount <= 5.5) {
            if (pl_ekf_hor <= 2.21) {
              if (pl_ekf_hor <= 0.8) {
                return 2.37 + DT_IMPURITY_FACTOR * 0.884;
              }
              else { // pl_ekf_hor > 0.8
                return 3.74 + DT_IMPURITY_FACTOR * 6.373;
              }
            }
            else { // pl_ekf_hor > 2.21
              if (f_cmc_avg <= 1.5) {
                return 105.89;
              }
              else { // f_cmc_avg > 1.5
                return 8.76;
              }
            }
          }
          else { // closeSkyCount > 5.5
            if (f_cn0_avg <= 29.25) {
              if (pl_ekf_hor <= 1.51) {
                return 1.98 + DT_IMPURITY_FACTOR * 1.393;
              }
              else { // pl_ekf_hor > 1.51
                return 5.28;
              }
            }
            else { // f_cn0_avg > 29.25
              if (pl_ekf_hor <= 1.22) {
                return 0.86 + DT_IMPURITY_FACTOR * 0.665;
              }
              else { // pl_ekf_hor > 1.22
                return 1.76 + DT_IMPURITY_FACTOR * 1.272;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.61
          if (pl_ekf_ver <= 0.92) {
            if (openSkyCount <= 60.5) {
              if (u_satMeasCount <= 24.5) {
                return 1.27 + DT_IMPURITY_FACTOR * 0.777;
              }
              else { // u_satMeasCount > 24.5
                return 2.58 + DT_IMPURITY_FACTOR * 2.017;
              }
            }
            else { // openSkyCount > 60.5
              if (pl_ekf_hor <= 1.04) {
                return 3.19 + DT_IMPURITY_FACTOR * 1.110;
              }
              else { // pl_ekf_hor > 1.04
                return 5.85;
              }
            }
          }
          else { // pl_ekf_ver > 0.92
            if (closeSkyCount <= 6.5) {
              if (f_cmc_avg <= 1.88) {
                return 43.14;
              }
              else { // f_cmc_avg > 1.88
                return 6.21;
              }
            }
            else { // closeSkyCount > 6.5
              if (f_hdop <= 1.04) {
                return 4.77 + DT_IMPURITY_FACTOR * 3.372;
              }
              else { // f_hdop > 1.04
                return 2.63 + DT_IMPURITY_FACTOR * 2.158;
              }
            }
          }
        }
      }
      else { // u_ppp_use_pr_num_float > 15.5
        if (pl_ekf_hor <= 1.25) {
          if (f_cmc_avg <= 1.8) {
            if (pl_ekf_hor <= 0.99) {
              if (u_ppp_use_pr_num_float <= 27.5) {
                return 0.87 + DT_IMPURITY_FACTOR * 0.764;
              }
              else { // u_ppp_use_pr_num_float > 27.5
                return 0.46 + DT_IMPURITY_FACTOR * 0.168;
              }
            }
            else { // pl_ekf_hor > 0.99
              if (f_cmc_std <= 1.47) {
                return 1.09 + DT_IMPURITY_FACTOR * 0.901;
              }
              else { // f_cmc_std > 1.47
                return 1.77 + DT_IMPURITY_FACTOR * 3.467;
              }
            }
          }
          else { // f_cmc_avg > 1.8
            if (u_CN040 <= 12.5) {
              if (u_MeasInUseCount <= 34.5) {
                return 2.68 + DT_IMPURITY_FACTOR * 1.265;
              }
              else { // u_MeasInUseCount > 34.5
                return 1.57 + DT_IMPURITY_FACTOR * 0.855;
              }
            }
            else { // u_CN040 > 12.5
              if (pl_ekf_hor <= 1.08) {
                return 1.09 + DT_IMPURITY_FACTOR * 0.810;
              }
              else { // pl_ekf_hor > 1.08
                return 2.44 + DT_IMPURITY_FACTOR * 3.226;
              }
            }
          }
        }
        else { // pl_ekf_hor > 1.25
          if (closeSkyCount <= 5.5) {
            if (openSkyCount <= 59.5) {
              if (pl_ekf_ver <= 1.12) {
                return 2.97 + DT_IMPURITY_FACTOR * 2.768;
              }
              else { // pl_ekf_ver > 1.12
                return 105.94;
              }
            }
            else { // openSkyCount > 59.5
              if (f_age <= 5.65) {
                return 28.00;
              }
              else { // f_age > 5.65
                return 4.55 + DT_IMPURITY_FACTOR * 5.017;
              }
            }
          }
          else { // closeSkyCount > 5.5
            if (f_postCodeSTD <= 1.07) {
              if (f_postCodeSTD <= 0.77) {
                return 1.36 + DT_IMPURITY_FACTOR * 0.582;
              }
              else { // f_postCodeSTD > 0.77
                return 1.88 + DT_IMPURITY_FACTOR * 0.712;
              }
            }
            else { // f_postCodeSTD > 1.07
              if (pl_ekf_hor <= 1.75) {
                return 2.37 + DT_IMPURITY_FACTOR * 1.782;
              }
              else { // pl_ekf_hor > 1.75
                return 3.30 + DT_IMPURITY_FACTOR * 1.703;
              }
            }
          }
        }
      }
    }
    else { // f_cn0_avg > 37.25
      if (pl_ekf_hor <= 1.15) {
        if (pl_ekf_hor <= 0.53) {
          if (pl_ekf_hor <= 0.35) {
            if (f_postCodeSTD <= 1.01) {
              if (f_postCodeSTD <= 0.45) {
                return 0.18 + DT_IMPURITY_FACTOR * 0.013;
              }
              else { // f_postCodeSTD > 0.45
                return 0.24 + DT_IMPURITY_FACTOR * 0.037;
              }
            }
            else { // f_postCodeSTD > 1.01
              if (avg_unc_pr <= 4.96) {
                return 0.32 + DT_IMPURITY_FACTOR * 0.064;
              }
              else { // avg_unc_pr > 4.96
                return 0.46 + DT_IMPURITY_FACTOR * 0.104;
              }
            }
          }
          else { // pl_ekf_hor > 0.35
            if (f_postCodeSTD <= 0.5) {
              if (f_postCodeSTD <= 0.38) {
                return 0.26 + DT_IMPURITY_FACTOR * 0.023;
              }
              else { // f_postCodeSTD > 0.38
                return 0.33 + DT_IMPURITY_FACTOR * 0.054;
              }
            }
            else { // f_postCodeSTD > 0.5
              if (f_postCodeSTD <= 1.15) {
                return 0.41 + DT_IMPURITY_FACTOR * 0.126;
              }
              else { // f_postCodeSTD > 1.15
                return 0.67 + DT_IMPURITY_FACTOR * 0.345;
              }
            }
          }
        }
        else { // pl_ekf_hor > 0.53
          if (f_postCodeSTD <= 0.56) {
            if (f_postCodeSTD <= 0.39) {
              if (pl_ekf_hor <= 0.77) {
                return 0.37 + DT_IMPURITY_FACTOR * 0.059;
              }
              else { // pl_ekf_hor > 0.77
                return 0.52 + DT_IMPURITY_FACTOR * 0.089;
              }
            }
            else { // f_postCodeSTD > 0.39
              if (u_SvTrackCount <= 22.5) {
                return 1.97 + DT_IMPURITY_FACTOR * 0.089;
              }
              else { // u_SvTrackCount > 22.5
                return 0.56 + DT_IMPURITY_FACTOR * 0.125;
              }
            }
          }
          else { // f_postCodeSTD > 0.56
            if (f_postCodeSTD <= 1.12) {
              if (pl_ekf_hor <= 0.78) {
                return 0.65 + DT_IMPURITY_FACTOR * 0.265;
              }
              else { // pl_ekf_hor > 0.78
                return 0.91 + DT_IMPURITY_FACTOR * 0.431;
              }
            }
            else { // f_postCodeSTD > 1.12
              if (u_SvTrackCount <= 25.5) {
                return 3.06 + DT_IMPURITY_FACTOR * 2.039;
              }
              else { // u_SvTrackCount > 25.5
                return 1.07 + DT_IMPURITY_FACTOR * 0.832;
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 1.15
        if (f_postCodeSTD <= 0.97) {
          if (f_postCodeSTD <= 0.69) {
            if (f_cmc_avg <= 6.26) {
              if (f_postCodeSTD <= 0.56) {
                return 0.82 + DT_IMPURITY_FACTOR * 0.362;
              }
              else { // f_postCodeSTD > 0.56
                return 1.10 + DT_IMPURITY_FACTOR * 0.517;
              }
            }
            else { // f_cmc_avg > 6.26
              if (f_age <= 5.75) {
                return 15.03;
              }
              else { // f_age > 5.75
                return 2.06 + DT_IMPURITY_FACTOR * 3.045;
              }
            }
          }
          else { // f_postCodeSTD > 0.69
            if (pl_ekf_ver <= 1.12) {
              if (u_SvTrackCount <= 23.5) {
                return 4.26 + DT_IMPURITY_FACTOR * 0.895;
              }
              else { // u_SvTrackCount > 23.5
                return 1.26 + DT_IMPURITY_FACTOR * 0.596;
              }
            }
            else { // pl_ekf_ver > 1.12
              if (f_cmc_avg <= 5.25) {
                return 1.72 + DT_IMPURITY_FACTOR * 0.749;
              }
              else { // f_cmc_avg > 5.25
                return 5.08;
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.97
          if (f_postCodeSTD <= 1.52) {
            if (pl_ekf_ver <= 1.1) {
              if (closeSkyCount <= 0.5) {
                return 2.63 + DT_IMPURITY_FACTOR * 3.687;
              }
              else { // closeSkyCount > 0.5
                return 1.63 + DT_IMPURITY_FACTOR * 0.572;
              }
            }
            else { // pl_ekf_ver > 1.1
              if (pl_ekf_hor <= 1.38) {
                return 6.50;
              }
              else { // pl_ekf_hor > 1.38
                return 2.65 + DT_IMPURITY_FACTOR * 1.105;
              }
            }
          }
          else { // f_postCodeSTD > 1.52
            if (openSkyCount <= 63.5) {
              if (pl_ekf_hor <= 1.76) {
                return 2.25 + DT_IMPURITY_FACTOR * 3.688;
              }
              else { // pl_ekf_hor > 1.76
                return 3.97 + DT_IMPURITY_FACTOR * 1.632;
              }
            }
            else { // openSkyCount > 63.5
              if (u_MeasInUseCount <= 32.5) {
                return 4.36 + DT_IMPURITY_FACTOR * 8.250;
              }
              else { // u_MeasInUseCount > 32.5
                return 22.04;
              }
            }
          }
        }
      }
    }
  }
  else { // pl_ekf_hor > 2.28
    if (closeSkyCount <= 0.5) {
      if (f_cn0_avg <= 36.75) {
        if (f_cmc_avg <= 1.79) {
          if (f_postPhaseSTD <= 0.01) {
            if (pl_ekf_ver <= 3.9) {
              if (pl_ekf_ver <= 1.55) {
                return 118.87;
              }
              else { // pl_ekf_ver > 1.55
                return 6.94;
              }
            }
            else { // pl_ekf_ver > 3.9
              if (pl_ekf_ver <= 7.86) {
                return 91.98;
              }
              else { // pl_ekf_ver > 7.86
                return 78.03;
              }
            }
          }
          else { // f_postPhaseSTD > 0.01
            if (pl_ekf_ver <= 1.99) {
              if (pl_ekf_hor <= 3.37) {
                return 141.81;
              }
              else { // pl_ekf_hor > 3.37
                return 131.29;
              }
            }
            else { // pl_ekf_ver > 1.99
              if (f_cmc_avg <= 0.78) {
                return 107.33;
              }
              else { // f_cmc_avg > 0.78
                return 8.86;
              }
            }
          }
        }
        else { // f_cmc_avg > 1.79
          if (pl_ekf_ver <= 1.43) {
            return 141.35;
          }
          else { // pl_ekf_ver > 1.43
            if (pl_ekf_ver <= 1.76) {
              if (u_MeasTrackCount <= 53.5) {
                return 8.86;
              }
              else { // u_MeasTrackCount > 53.5
                return 10.43;
              }
            }
            else { // pl_ekf_ver > 1.76
              if (pl_ekf_hor <= 5.51) {
                return 4.50 + DT_IMPURITY_FACTOR * 1.663;
              }
              else { // pl_ekf_hor > 5.51
                return 10.03;
              }
            }
          }
        }
      }
      else { // f_cn0_avg > 36.75
        if (pl_ekf_hor <= 22.18) {
          if (pl_ekf_hor <= 3.69) {
            if (f_postCodeSTD <= 1.02) {
              if (u_ppp_use_pr_num_float <= 8.5) {
                return 4.32 + DT_IMPURITY_FACTOR * 3.410;
              }
              else { // u_ppp_use_pr_num_float > 8.5
                return 1.26 + DT_IMPURITY_FACTOR * 0.698;
              }
            }
            else { // f_postCodeSTD > 1.02
              if (f_postPhaseSTD <= 0.02) {
                return 3.81 + DT_IMPURITY_FACTOR * 1.196;
              }
              else { // f_postPhaseSTD > 0.02
                return 12.10;
              }
            }
          }
          else { // pl_ekf_hor > 3.69
            if (f_cn0_avg <= 37.25) {
              if (u_SvTrackCount <= 29.5) {
                return 86.38;
              }
              else { // u_SvTrackCount > 29.5
                return 6.13;
              }
            }
            else { // f_cn0_avg > 37.25
              if (f_cn0_std <= 8.53) {
                return 8.16;
              }
              else { // f_cn0_std > 8.53
                return 15.40;
              }
            }
          }
        }
        else { // pl_ekf_hor > 22.18
          if (u_satMeasCount <= 15.5) {
            if (f_cmc_avg <= 0.37) {
              if (pl_ekf_hor <= 45.53) {
                return 49.68;
              }
              else { // pl_ekf_hor > 45.53
                return 44.04;
              }
            }
            else { // f_cmc_avg > 0.37
              return 28.53;
            }
          }
          else { // u_satMeasCount > 15.5
            if (f_cn0_avg <= 38.25) {
              if (pl_ekf_ver <= 13.83) {
                return 68.30;
              }
              else { // pl_ekf_ver > 13.83
                return 64.62;
              }
            }
            else { // f_cn0_avg > 38.25
              if (pl_ekf_hor <= 33.33) {
                return 58.17;
              }
              else { // pl_ekf_hor > 33.33
                return 54.42;
              }
            }
          }
        }
      }
    }
    else { // closeSkyCount > 0.5
      if (pl_ekf_ver <= 3.98) {
        if (avg_unc_pr <= 8.49) {
          if (f_postCodeSTD <= 1.1) {
            if (f_postCodeSTD <= 0.67) {
              if (pl_ekf_hor <= 3.07) {
                return 1.21 + DT_IMPURITY_FACTOR * 0.684;
              }
              else { // pl_ekf_hor > 3.07
                return 2.13 + DT_IMPURITY_FACTOR * 3.365;
              }
            }
            else { // f_postCodeSTD > 0.67
              if (u_ppp_use_pr_num_float <= 16.5) {
                return 2.93 + DT_IMPURITY_FACTOR * 3.050;
              }
              else { // u_ppp_use_pr_num_float > 16.5
                return 1.69 + DT_IMPURITY_FACTOR * 0.714;
              }
            }
          }
          else { // f_postCodeSTD > 1.1
            if (pl_ekf_hor <= 3.09) {
              if (pl_ekf_hor <= 2.28) {
                return 29.98;
              }
              else { // pl_ekf_hor > 2.28
                return 3.49 + DT_IMPURITY_FACTOR * 2.401;
              }
            }
            else { // pl_ekf_hor > 3.09
              if (f_cmc_avg <= 2.08) {
                return 4.38 + DT_IMPURITY_FACTOR * 2.148;
              }
              else { // f_cmc_avg > 2.08
                return 6.35;
              }
            }
          }
        }
        else { // avg_unc_pr > 8.49
          if (pl_ekf_hor <= 3.02) {
            if (u_MeasTrackCount <= 53.5) {
              if (f_cn0_avg <= 29.75) {
                return 6.10;
              }
              else { // f_cn0_avg > 29.75
                return 3.46 + DT_IMPURITY_FACTOR * 2.615;
              }
            }
            else { // u_MeasTrackCount > 53.5
              if (f_cmc_avg <= 7.07) {
                return 5.79;
              }
              else { // f_cmc_avg > 7.07
                return 12.07;
              }
            }
          }
          else { // pl_ekf_hor > 3.02
            if (u_MeasTrackCount <= 55.5) {
              if (f_cn0_std <= 6.3) {
                return 7.93;
              }
              else { // f_cn0_std > 6.3
                return 5.37;
              }
            }
            else { // u_MeasTrackCount > 55.5
              if (f_postPhaseSTD <= 0.0) {
                return 7.11;
              }
              else { // f_postPhaseSTD > 0.0
                return 14.38;
              }
            }
          }
        }
      }
      else { // pl_ekf_ver > 3.98
        if (f_postCodeSTD <= 1.71) {
          if (u_ppp_use_pr_num_float <= 9.5) {
            if (f_cn0_std <= 9.27) {
              if (f_cn0_std <= 2.59) {
                return 3.86 + DT_IMPURITY_FACTOR * 2.407;
              }
              else { // f_cn0_std > 2.59
                return 10.56;
              }
            }
            else { // f_cn0_std > 9.27
              if (avg_unc_pr <= 22.35) {
                return 6.35;
              }
              else { // avg_unc_pr > 22.35
                return 14.22;
              }
            }
          }
          else { // u_ppp_use_pr_num_float > 9.5
            if (f_hdop <= 1.67) {
              if (u_MeasTrackCount <= 39.5) {
                return 2.28 + DT_IMPURITY_FACTOR * 1.761;
              }
              else { // u_MeasTrackCount > 39.5
                return 5.64;
              }
            }
            else { // f_hdop > 1.67
              if (u_CN040 <= 8.5) {
                return 8.79;
              }
              else { // u_CN040 > 8.5
                return 5.66;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.71
          if (f_cn0_std <= 10.01) {
            if (pl_ekf_ver <= 15.92) {
              if (avg_unc_pr <= 15.24) {
                return 10.59;
              }
              else { // avg_unc_pr > 15.24
                return 14.97;
              }
            }
            else { // pl_ekf_ver > 15.92
              if (f_hdop <= 1.91) {
                return 13.71;
              }
              else { // f_hdop > 1.91
                return 28.25;
              }
            }
          }
          else { // f_cn0_std > 10.01
            if (u_ppp_use_pr_num_float <= 12.5) {
              if (avg_unc_pr <= 21.74) {
                return 7.49;
              }
              else { // avg_unc_pr > 21.74
                return 11.97;
              }
            }
            else { // u_ppp_use_pr_num_float > 12.5
              if (f_cmc_std <= 0.87) {
                return 1.78 + DT_IMPURITY_FACTOR * 0.793;
              }
              else { // f_cmc_std > 0.87
                return 0.23;
              }
            }
          }
        }
      }
    }
  }
}

static double predict_float_4(integ_PppFeature_t* gpz_pppFeature) {
  uint8_t openSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_openSkyCount);
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  uint8_t u_SvTrackCount = gpz_pppFeature->z_SvStatus.u_SvTrackCount;
  float f_vdop = gpz_pppFeature->z_dops.f_vdop;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  float pl_ekf_hor = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  float f_postPhaseSTD = gpz_pppFeature->f_postPhaseSTD;
  float f_hdop = gpz_pppFeature->z_dops.f_hdop;
  float f_cn0_avg = gpz_pppFeature->f_cn0_avg;
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  uint8_t u_ppp_use_pr_num_float = gpz_pppFeature->u_ppp_use_pr_num_float;
  float avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  uint8_t closeSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount);
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  uint8_t u_ns = gpz_pppFeature->u_ns;
  float f_postCodeSTD = gpz_pppFeature->f_postCodeSTD;

  if (pl_ekf_hor <= 2.27) {
    if (u_CN040 <= 17.5) {
      if (f_postCodeSTD <= 1.29) {
        if (u_CN040 <= 7.5) {
          if (u_satMeasCount <= 18.5) {
            if (f_cmc_avg <= 2.97) {
              if (pl_ekf_hor <= 1.31) {
                return 0.57 + DT_IMPURITY_FACTOR * 0.238;
              }
              else { // pl_ekf_hor > 1.31
                return 1.61 + DT_IMPURITY_FACTOR * 0.565;
              }
            }
            else { // f_cmc_avg > 2.97
              if (openSkyCount <= 59.5) {
                return 2.15 + DT_IMPURITY_FACTOR * 0.763;
              }
              else { // openSkyCount > 59.5
                return 4.12 + DT_IMPURITY_FACTOR * 0.932;
              }
            }
          }
          else { // u_satMeasCount > 18.5
            if (f_cn0_avg <= 32.75) {
              if (pl_ekf_hor <= 2.04) {
                return 3.59 + DT_IMPURITY_FACTOR * 0.893;
              }
              else { // pl_ekf_hor > 2.04
                return 7.58;
              }
            }
            else { // f_cn0_avg > 32.75
              if (pl_ekf_hor <= 1.7) {
                return 2.00 + DT_IMPURITY_FACTOR * 1.210;
              }
              else { // pl_ekf_hor > 1.7
                return 3.09 + DT_IMPURITY_FACTOR * 0.927;
              }
            }
          }
        }
        else { // u_CN040 > 7.5
          if (pl_ekf_hor <= 1.25) {
            if (f_cmc_avg <= 2.13) {
              if (pl_ekf_hor <= 0.85) {
                return 0.59 + DT_IMPURITY_FACTOR * 0.407;
              }
              else { // pl_ekf_hor > 0.85
                return 0.97 + DT_IMPURITY_FACTOR * 0.849;
              }
            }
            else { // f_cmc_avg > 2.13
              if (u_CN040 <= 12.5) {
                return 2.27 + DT_IMPURITY_FACTOR * 1.465;
              }
              else { // u_CN040 > 12.5
                return 1.04 + DT_IMPURITY_FACTOR * 0.743;
              }
            }
          }
          else { // pl_ekf_hor > 1.25
            if (f_postCodeSTD <= 0.7) {
              if (f_cmc_avg <= 6.26) {
                return 1.13 + DT_IMPURITY_FACTOR * 0.819;
              }
              else { // f_cmc_avg > 6.26
                return 3.51 + DT_IMPURITY_FACTOR * 2.979;
              }
            }
            else { // f_postCodeSTD > 0.7
              if (openSkyCount <= 59.5) {
                return 1.93 + DT_IMPURITY_FACTOR * 1.066;
              }
              else { // openSkyCount > 59.5
                return 3.27 + DT_IMPURITY_FACTOR * 8.846;
              }
            }
          }
        }
      }
      else { // f_postCodeSTD > 1.29
        if (pl_ekf_ver <= 0.9) {
          if (closeSkyCount <= 0.5) {
            if (pl_ekf_hor <= 0.99) {
              if (u_CN040 <= 7.5) {
                return 3.56 + DT_IMPURITY_FACTOR * 0.886;
              }
              else { // u_CN040 > 7.5
                return 1.99 + DT_IMPURITY_FACTOR * 0.880;
              }
            }
            else { // pl_ekf_hor > 0.99
              if (f_cmc_avg <= 3.27) {
                return 5.99;
              }
              else { // f_cmc_avg > 3.27
                return 3.51 + DT_IMPURITY_FACTOR * 3.013;
              }
            }
          }
          else { // closeSkyCount > 0.5
            if (pl_ekf_hor <= 1.25) {
              if (f_cmc_std <= 2.6) {
                return 0.95 + DT_IMPURITY_FACTOR * 0.513;
              }
              else { // f_cmc_std > 2.6
                return 1.89 + DT_IMPURITY_FACTOR * 0.874;
              }
            }
            else { // pl_ekf_hor > 1.25
              if (u_satMeasCount <= 24.5) {
                return 1.64 + DT_IMPURITY_FACTOR * 0.890;
              }
              else { // u_satMeasCount > 24.5
                return 3.95 + DT_IMPURITY_FACTOR * 3.762;
              }
            }
          }
        }
        else { // pl_ekf_ver > 0.9
          if (closeSkyCount <= 5.5) {
            if (closeSkyCount <= 4.5) {
              if (f_vdop <= 2.23) {
                return 6.42;
              }
              else { // f_vdop > 2.23
                return 26.83;
              }
            }
            else { // closeSkyCount > 4.5
              if (u_MeasTrackCount <= 50.5) {
                return 109.87;
              }
              else { // u_MeasTrackCount > 50.5
                return 3.79 + DT_IMPURITY_FACTOR * 2.444;
              }
            }
          }
          else { // closeSkyCount > 5.5
            if (f_cn0_avg <= 29.25) {
              if (u_MeasTrackCount <= 47.5) {
                return 2.07 + DT_IMPURITY_FACTOR * 0.743;
              }
              else { // u_MeasTrackCount > 47.5
                return 5.78;
              }
            }
            else { // f_cn0_avg > 29.25
              if (u_ns <= 13.5) {
                return 2.41 + DT_IMPURITY_FACTOR * 2.415;
              }
              else { // u_ns > 13.5
                return 3.30 + DT_IMPURITY_FACTOR * 2.014;
              }
            }
          }
        }
      }
    }
    else { // u_CN040 > 17.5
      if (pl_ekf_hor <= 0.77) {
        if (pl_ekf_hor <= 0.43) {
          if (f_postCodeSTD <= 0.95) {
            if (pl_ekf_hor <= 0.29) {
              if (f_postCodeSTD <= 0.44) {
                return 0.17 + DT_IMPURITY_FACTOR * 0.012;
              }
              else { // f_postCodeSTD > 0.44
                return 0.22 + DT_IMPURITY_FACTOR * 0.034;
              }
            }
            else { // pl_ekf_hor > 0.29
              if (f_postCodeSTD <= 0.42) {
                return 0.23 + DT_IMPURITY_FACTOR * 0.018;
              }
              else { // f_postCodeSTD > 0.42
                return 0.32 + DT_IMPURITY_FACTOR * 0.064;
              }
            }
          }
          else { // f_postCodeSTD > 0.95
            if (avg_unc_pr <= 6.65) {
              if (f_postCodeSTD <= 1.19) {
                return 0.34 + DT_IMPURITY_FACTOR * 0.081;
              }
              else { // f_postCodeSTD > 1.19
                return 0.48 + DT_IMPURITY_FACTOR * 0.118;
              }
            }
            else { // avg_unc_pr > 6.65
              if (pl_ekf_hor <= 0.23) {
                return 0.27 + DT_IMPURITY_FACTOR * 0.053;
              }
              else { // pl_ekf_hor > 0.23
                return 0.74 + DT_IMPURITY_FACTOR * 0.232;
              }
            }
          }
        }
        else { // pl_ekf_hor > 0.43
          if (f_postCodeSTD <= 0.96) {
            if (f_postCodeSTD <= 0.45) {
              if (pl_ekf_hor <= 0.56) {
                return 0.32 + DT_IMPURITY_FACTOR * 0.041;
              }
              else { // pl_ekf_hor > 0.56
                return 0.42 + DT_IMPURITY_FACTOR * 0.074;
              }
            }
            else { // f_postCodeSTD > 0.45
              if (pl_ekf_hor <= 0.57) {
                return 0.46 + DT_IMPURITY_FACTOR * 0.142;
              }
              else { // pl_ekf_hor > 0.57
                return 0.63 + DT_IMPURITY_FACTOR * 0.212;
              }
            }
          }
          else { // f_postCodeSTD > 0.96
            if (u_SvTrackCount <= 25.5) {
              if (f_postPhaseSTD <= 0.0) {
                return 1.77 + DT_IMPURITY_FACTOR * 0.306;
              }
              else { // f_postPhaseSTD > 0.0
                return 4.57 + DT_IMPURITY_FACTOR * 0.618;
              }
            }
            else { // u_SvTrackCount > 25.5
              if (f_cn0_std <= 7.04) {
                return 0.64 + DT_IMPURITY_FACTOR * 0.218;
              }
              else { // f_cn0_std > 7.04
                return 1.01 + DT_IMPURITY_FACTOR * 0.869;
              }
            }
          }
        }
      }
      else { // pl_ekf_hor > 0.77
        if (f_postCodeSTD <= 0.96) {
          if (pl_ekf_hor <= 1.26) {
            if (f_postCodeSTD <= 0.55) {
              if (u_MeasTrackCount <= 35.5) {
                return 2.02;
              }
              else { // u_MeasTrackCount > 35.5
                return 0.61 + DT_IMPURITY_FACTOR * 0.119;
              }
            }
            else { // f_postCodeSTD > 0.55
              if (u_SvTrackCount <= 23.5) {
                return 4.03 + DT_IMPURITY_FACTOR * 0.761;
              }
              else { // u_SvTrackCount > 23.5
                return 0.88 + DT_IMPURITY_FACTOR * 0.302;
              }
            }
          }
          else { // pl_ekf_hor > 1.26
            if (f_postCodeSTD <= 0.69) {
              if (f_cmc_avg <= 7.81) {
                return 0.95 + DT_IMPURITY_FACTOR * 0.468;
              }
              else { // f_cmc_avg > 7.81
                return 3.93 + DT_IMPURITY_FACTOR * 3.592;
              }
            }
            else { // f_postCodeSTD > 0.69
              if (f_cn0_avg <= 33.75) {
                return 6.07;
              }
              else { // f_cn0_avg > 33.75
                return 1.53 + DT_IMPURITY_FACTOR * 0.774;
              }
            }
          }
        }
        else { // f_postCodeSTD > 0.96
          if (pl_ekf_hor <= 1.17) {
            if (u_ppp_use_pr_num_float <= 16.5) {
              if (f_cn0_avg <= 43.25) {
                return 1.85 + DT_IMPURITY_FACTOR * 0.802;
              }
              else { // f_cn0_avg > 43.25
                return 5.03;
              }
            }
            else { // u_ppp_use_pr_num_float > 16.5
              if (f_cn0_std <= 13.71) {
                return 1.18 + DT_IMPURITY_FACTOR * 0.800;
              }
              else { // f_cn0_std > 13.71
                return 4.28 + DT_IMPURITY_FACTOR * 0.846;
              }
            }
          }
          else { // pl_ekf_hor > 1.17
            if (closeSkyCount <= 0.5) {
              if (avg_unc_pr <= 5.74) {
                return 2.92 + DT_IMPURITY_FACTOR * 1.369;
              }
              else { // avg_unc_pr > 5.74
                return 6.99;
              }
            }
            else { // closeSkyCount > 0.5
              if (pl_ekf_ver <= 1.14) {
                return 1.92 + DT_IMPURITY_FACTOR * 0.808;
              }
              else { // pl_ekf_ver > 1.14
                return 2.84 + DT_IMPURITY_FACTOR * 2.874;
              }
            }
          }
        }
      }
    }
  }
  else { // pl_ekf_hor > 2.27
    if (openSkyCount <= 63.5) {
      if (pl_ekf_ver <= 3.97) {
        if (avg_unc_pr <= 8.41) {
          if (f_postCodeSTD <= 1.03) {
            if (f_postCodeSTD <= 0.67) {
              if (pl_ekf_hor <= 3.07) {
                return 1.21 + DT_IMPURITY_FACTOR * 0.575;
              }
              else { // pl_ekf_hor > 3.07
                return 2.08 + DT_IMPURITY_FACTOR * 3.051;
              }
            }
            else { // f_postCodeSTD > 0.67
              if (u_ppp_use_pr_num_float <= 16.5) {
                return 2.83 + DT_IMPURITY_FACTOR * 2.915;
              }
              else { // u_ppp_use_pr_num_float > 16.5
                return 1.62 + DT_IMPURITY_FACTOR * 0.681;
              }
            }
          }
          else { // f_postCodeSTD > 1.03
            if (pl_ekf_hor <= 3.09) {
              if (u_MeasInUseCount <= 39.5) {
                return 3.17 + DT_IMPURITY_FACTOR * 4.049;
              }
              else { // u_MeasInUseCount > 39.5
                return 5.21;
              }
            }
            else { // pl_ekf_hor > 3.09
              if (f_cmc_avg <= 1.94) {
                return 4.14 + DT_IMPURITY_FACTOR * 4.986;
              }
              else { // f_cmc_avg > 1.94
                return 6.31;
              }
            }
          }
        }
        else { // avg_unc_pr > 8.41
          if (pl_ekf_hor <= 2.99) {
            if (u_MeasTrackCount <= 54.5) {
              if (f_cn0_avg <= 30.25) {
                return 5.81;
              }
              else { // f_cn0_avg > 30.25
                return 3.61 + DT_IMPURITY_FACTOR * 2.201;
              }
            }
            else { // u_MeasTrackCount > 54.5
              if (f_cmc_avg <= 6.88) {
                return 5.90;
              }
              else { // f_cmc_avg > 6.88
                return 13.43;
              }
            }
          }
          else { // pl_ekf_hor > 2.99
            if (u_MeasTrackCount <= 55.5) {
              if (f_cn0_std <= 6.3) {
                return 7.90;
              }
              else { // f_cn0_std > 6.3
                return 5.33;
              }
            }
            else { // u_MeasTrackCount > 55.5
              if (f_postPhaseSTD <= 0.0) {
                return 7.23;
              }
              else { // f_postPhaseSTD > 0.0
                return 14.29;
              }
            }
          }
        }
      }
      else { // pl_ekf_ver > 3.97
        if (f_postCodeSTD <= 1.71) {
          if (u_ppp_use_pr_num_float <= 9.5) {
            if (f_cn0_std <= 9.27) {
              if (f_cn0_std <= 2.59) {
                return 4.06 + DT_IMPURITY_FACTOR * 2.759;
              }
              else { // f_cn0_std > 2.59
                return 10.55;
              }
            }
            else { // f_cn0_std > 9.27
              if (u_MeasTrackCount <= 31.5) {
                return 9.87;
              }
              else { // u_MeasTrackCount > 31.5
                return 5.55;
              }
            }
          }
          else { // u_ppp_use_pr_num_float > 9.5
            if (f_hdop <= 1.67) {
              if (u_MeasTrackCount <= 39.5) {
                return 2.34 + DT_IMPURITY_FACTOR * 2.087;
              }
              else { // u_MeasTrackCount > 39.5
                return 5.61;
              }
            }
            else { // f_hdop > 1.67
              if (u_CN040 <= 8.5) {
                return 9.03;
              }
              else { // u_CN040 > 8.5
                return 5.75;
              }
            }
          }
        }
        else { // f_postCodeSTD > 1.71
          if (f_cn0_std <= 8.53) {
            if (pl_ekf_ver <= 15.85) {
              if (avg_unc_pr <= 14.19) {
                return 10.85;
              }
              else { // avg_unc_pr > 14.19
                return 15.28;
              }
            }
            else { // pl_ekf_ver > 15.85
              if (pl_ekf_hor <= 24.48) {
                return 33.52;
              }
              else { // pl_ekf_hor > 24.48
                return 16.07;
              }
            }
          }
          else { // f_cn0_std > 8.53
            if (avg_unc_pr <= 21.67) {
              if (f_postPhaseSTD <= 0.01) {
                return 9.19;
              }
              else { // f_postPhaseSTD > 0.01
                return 5.64;
              }
            }
            else { // avg_unc_pr > 21.67
              if (f_postCodeSTD <= 1.92) {
                return 25.93;
              }
              else { // f_postCodeSTD > 1.92
                return 12.00;
              }
            }
          }
        }
      }
    }
    else { // openSkyCount > 63.5
      if (f_cn0_avg <= 36.75) {
        if (f_cmc_avg <= 1.79) {
          if (u_CN040 <= 14.0) {
            if (f_postPhaseSTD <= 0.01) {
              if (f_cn0_avg <= 32.0) {
                return 8.40;
              }
              else { // f_cn0_avg > 32.0
                return 86.07;
              }
            }
            else { // f_postPhaseSTD > 0.01
              if (pl_ekf_ver <= 1.99) {
                return 140.06;
              }
              else { // pl_ekf_ver > 1.99
                return 101.97;
              }
            }
          }
          else { // u_CN040 > 14.0
            if (f_cmc_avg <= 0.81) {
              return 1.40 + DT_IMPURITY_FACTOR * 0.097;
            }
            else { // f_cmc_avg > 0.81
              if (u_MeasTrackCount <= 42.5) {
                return 0.70 + DT_IMPURITY_FACTOR * 0.052;
              }
              else { // u_MeasTrackCount > 42.5
                return 0.90 + DT_IMPURITY_FACTOR * 0.008;
              }
            }
          }
        }
        else { // f_cmc_avg > 1.79
          if (pl_ekf_ver <= 1.43) {
            return 141.41;
          }
          else { // pl_ekf_ver > 1.43
            if (pl_ekf_ver <= 1.76) {
              if (u_MeasTrackCount <= 53.5) {
                return 8.90;
              }
              else { // u_MeasTrackCount > 53.5
                return 10.42;
              }
            }
            else { // pl_ekf_ver > 1.76
              if (avg_unc_pr <= 3.94) {
                return 1.03 + DT_IMPURITY_FACTOR * 0.007;
              }
              else { // avg_unc_pr > 3.94
                return 5.26;
              }
            }
          }
        }
      }
      else { // f_cn0_avg > 36.75
        if (pl_ekf_hor <= 22.14) {
          if (pl_ekf_hor <= 3.69) {
            if (f_postCodeSTD <= 1.02) {
              if (u_ppp_use_pr_num_float <= 8.5) {
                return 4.42 + DT_IMPURITY_FACTOR * 3.432;
              }
              else { // u_ppp_use_pr_num_float > 8.5
                return 1.29 + DT_IMPURITY_FACTOR * 0.724;
              }
            }
            else { // f_postCodeSTD > 1.02
              if (f_postPhaseSTD <= 0.03) {
                return 3.83 + DT_IMPURITY_FACTOR * 1.351;
              }
              else { // f_postPhaseSTD > 0.03
                return 13.40;
              }
            }
          }
          else { // pl_ekf_hor > 3.69
            if (f_cn0_avg <= 37.25) {
              if (avg_unc_pr <= 4.84) {
                return 5.26;
              }
              else { // avg_unc_pr > 4.84
                return 86.09;
              }
            }
            else { // f_cn0_avg > 37.25
              if (f_cn0_std <= 8.53) {
                return 8.12;
              }
              else { // f_cn0_std > 8.53
                return 15.41;
              }
            }
          }
        }
        else { // pl_ekf_hor > 22.14
          if (f_cn0_std <= 7.78) {
            if (f_cmc_avg <= 0.42) {
              return 43.95;
            }
            else { // f_cmc_avg > 0.42
              return 28.45;
            }
          }
          else { // f_cn0_std > 7.78
            if (pl_ekf_hor <= 28.4) {
              if (pl_ekf_ver <= 13.85) {
                return 68.26;
              }
              else { // pl_ekf_ver > 13.85
                return 64.54;
              }
            }
            else { // pl_ekf_hor > 28.4
              if (pl_ekf_hor <= 34.82) {
                return 57.40;
              }
              else { // pl_ekf_hor > 34.82
                return 52.67;
              }
            }
          }
        }
      }
    }
  }
}

static double predict_fix_0(integ_PppFeature_t* gpz_pppFeature) {
  uint8_t closeSkyCount = ppp_IntegCountSetBits(gpz_pppFeature->t_closeSkyCount);
  float f_vdop = gpz_pppFeature->z_dops.f_vdop;
  float f_ambPool_pdop = gpz_pppFeature->f_ambPool_pdop;
  uint8_t u_nb = gpz_pppFeature->u_nb;
  float f_ambPool_adop = gpz_pppFeature->f_ambPool_adop;
  float f_cn0_std = gpz_pppFeature->f_cn0_std;
  double d_lambda_sigma_2 = gpz_pppFeature->d_lambda_sigma[1];
  float f_fixedCodeSTD = gpz_pppFeature->f_fixedCodeSTD;
  float u_ppp_use_sat_num_fix = gpz_pppFeature->u_ppp_use_sat_num_fix;
  float f_fixedPhaseSTD = gpz_pppFeature->f_fixedPhaseSTD;
  uint8_t u_MeasInUseCount = gpz_pppFeature->z_SvStatus.u_MeasInUseCount;
  float f_age = gpz_pppFeature->f_age;
  uint8_t u_SvTrackCount = gpz_pppFeature->z_SvStatus.u_SvTrackCount;
  int32_t q_QRcheckStatus = (int32_t)gpz_pppFeature->q_QRcheckStatus;
  float avg_unc_pr = gpz_pppFeature->f_avg_unc_pr;
  float f_cmc_avg = gpz_pppFeature->f_cmc_avg;
  uint8_t u_CN040 = gpz_pppFeature->u_CN040;
  uint8_t u_MeasTrackCount = gpz_pppFeature->z_SvStatus.u_MeasTrackCount;
  uint8_t u_fixedTotalSatCount = gpz_pppFeature->u_fixedTotalSatCount;
  float pl_ekf_ver = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];
  float f_gdop = gpz_pppFeature->z_dops.f_gdop;
  float f_cmc_std = gpz_pppFeature->f_cmc_std;
  float f_ratio = gpz_pppFeature->f_ratio;
  uint8_t u_satMeasCount = gpz_pppFeature->u_satMeasCount;
  double d_lambda_sigma_1 = gpz_pppFeature->d_lambda_sigma[0];
  uint8_t u_SvInUseCount = gpz_pppFeature->z_SvStatus.u_SvInUseCount;

  if (u_CN040 <= 12.5) {
    if (u_MeasInUseCount <= 33.5) {
      if (f_vdop <= 1.12) {
        if (f_cmc_avg <= 2.58) {
          if (pl_ekf_ver <= 0.15) {
            if (f_ambPool_adop <= 6.18) {
              return 0.60;
            }
            else { // f_ambPool_adop > 6.18
              if (d_lambda_sigma_1 <= 153.96) {
                return 0.43;
              }
              else { // d_lambda_sigma_1 > 153.96
                return 0.37;
              }
            }
          }
          else { // pl_ekf_ver > 0.15
            if (d_lambda_sigma_2 <= 78.29) {
              if (closeSkyCount <= 19.0) {
                return 0.06;
              }
              else { // closeSkyCount > 19.0
                return 0.02;
              }
            }
            else { // d_lambda_sigma_2 > 78.29
              if (u_satMeasCount <= 13.5) {
                return 0.13;
              }
              else { // u_satMeasCount > 13.5
                return 0.09;
              }
            }
          }
        }
        else { // f_cmc_avg > 2.58
          if (f_cn0_std <= 9.64) {
            if (f_fixedCodeSTD <= 1.12) {
              return 1.15;
            }
            else { // f_fixedCodeSTD > 1.12
              if (f_ambPool_pdop <= 3.53) {
                return 1.91 + DT_IMPURITY_FACTOR * 0.012;
              }
              else { // f_ambPool_pdop > 3.53
                return 2.57 + DT_IMPURITY_FACTOR * 0.539;
              }
            }
          }
          else { // f_cn0_std > 9.64
            if (f_cmc_avg <= 5.96) {
              return 3.29 + DT_IMPURITY_FACTOR * 0.635;
            }
            else { // f_cmc_avg > 5.96
              if (f_age <= 24.85) {
                return 2.37 + DT_IMPURITY_FACTOR * 0.395;
              }
              else { // f_age > 24.85
                return 2.70;
              }
            }
          }
        }
      }
      else { // f_vdop > 1.12
        if (f_vdop <= 1.63) {
          if (u_CN040 <= 7.5) {
            if (f_vdop <= 1.6) {
              if (u_MeasTrackCount <= 51.5) {
                return 4.14 + DT_IMPURITY_FACTOR * 0.037;
              }
              else { // u_MeasTrackCount > 51.5
                return 3.84 + DT_IMPURITY_FACTOR * 0.169;
              }
            }
            else { // f_vdop > 1.6
              if (u_SvTrackCount <= 32.5) {
                return 2.94;
              }
              else { // u_SvTrackCount > 32.5
                return 3.45 + DT_IMPURITY_FACTOR * 0.036;
              }
            }
          }
          else { // u_CN040 > 7.5
            if (avg_unc_pr <= 7.16) {
              if (f_vdop <= 1.22) {
                return 10.94;
              }
              else { // f_vdop > 1.22
                return 3.31 + DT_IMPURITY_FACTOR * 0.738;
              }
            }
            else { // avg_unc_pr > 7.16
              if (pl_ekf_ver <= 0.12) {
                return 3.05 + DT_IMPURITY_FACTOR * 0.604;
              }
              else { // pl_ekf_ver > 0.12
                return 2.05 + DT_IMPURITY_FACTOR * 0.452;
              }
            }
          }
        }
        else { // f_vdop > 1.63
          if (pl_ekf_ver <= 0.11) {
            if (f_cmc_std <= 4.39) {
              if (f_cn0_std <= 10.01) {
                return 4.73 + DT_IMPURITY_FACTOR * 0.056;
              }
              else { // f_cn0_std > 10.01
                return 5.78;
              }
            }
            else { // f_cmc_std > 4.39
              if (f_ratio <= 2.06) {
                return 8.01;
              }
              else { // f_ratio > 2.06
                return 5.88;
              }
            }
          }
          else { // pl_ekf_ver > 0.11
            if (u_SvInUseCount <= 17.5) {
              if (u_CN040 <= 6.5) {
                return 3.68 + DT_IMPURITY_FACTOR * 0.216;
              }
              else { // u_CN040 > 6.5
                return 2.44 + DT_IMPURITY_FACTOR * 0.834;
              }
            }
            else { // u_SvInUseCount > 17.5
              if (d_lambda_sigma_2 <= 224.06) {
                return 8.09;
              }
              else { // d_lambda_sigma_2 > 224.06
                return 8.06;
              }
            }
          }
        }
      }
    }
    else { // u_MeasInUseCount > 33.5
      if (f_cmc_avg <= 3.57) {
        if (f_fixedCodeSTD <= 2.42) {
          if (avg_unc_pr <= 16.45) {
            if (d_lambda_sigma_2 <= 1907.16) {
              if (f_cmc_avg <= 3.37) {
                return 0.09 + DT_IMPURITY_FACTOR * 0.018;
              }
              else { // f_cmc_avg > 3.37
                return 0.48 + DT_IMPURITY_FACTOR * 0.241;
              }
            }
            else { // d_lambda_sigma_2 > 1907.16
              return 1.93 + DT_IMPURITY_FACTOR * 2.671;
            }
          }
          else { // avg_unc_pr > 16.45
            return 1.94 + DT_IMPURITY_FACTOR * 3.468;
          }
        }
        else { // f_fixedCodeSTD > 2.42
          return 2.21 + DT_IMPURITY_FACTOR * 4.352;
        }
      }
      else { // f_cmc_avg > 3.57
        if (u_MeasInUseCount <= 34.5) {
          return 3.89 + DT_IMPURITY_FACTOR * 3.010;
        }
        else { // u_MeasInUseCount > 34.5
          if (avg_unc_pr <= 11.11) {
            if (f_cmc_avg <= 4.32) {
              if (f_cmc_std <= 3.55) {
                return 0.27 + DT_IMPURITY_FACTOR * 0.006;
              }
              else { // f_cmc_std > 3.55
                return 0.08;
              }
            }
            else { // f_cmc_avg > 4.32
              if (f_age <= 6.15) {
                return 1.16;
              }
              else { // f_age > 6.15
                return 1.03 + DT_IMPURITY_FACTOR * 0.030;
              }
            }
          }
          else { // avg_unc_pr > 11.11
            if (f_cmc_std <= 3.24) {
              if (u_fixedTotalSatCount <= 9.5) {
                return 1.81 + DT_IMPURITY_FACTOR * 0.044;
              }
              else { // u_fixedTotalSatCount > 9.5
                return 1.18 + DT_IMPURITY_FACTOR * 0.272;
              }
            }
            else { // f_cmc_std > 3.24
              if (u_ppp_use_sat_num_fix <= 10.5) {
                return 0.98 + DT_IMPURITY_FACTOR * 0.070;
              }
              else { // u_ppp_use_sat_num_fix > 10.5
                return 1.45 + DT_IMPURITY_FACTOR * 0.043;
              }
            }
          }
        }
      }
    }
  }
  else { // u_CN040 > 12.5
    if (f_cn0_std <= 7.78) {
      if (f_fixedPhaseSTD <= 0.01) {
        if (u_nb <= 9.5) {
          if (f_fixedCodeSTD <= 1.0) {
            if (u_MeasTrackCount <= 37.5) {
              if (pl_ekf_ver <= 0.17) {
                return 1.72 + DT_IMPURITY_FACTOR * 0.049;
              }
              else { // pl_ekf_ver > 0.17
                return 0.03;
              }
            }
            else { // u_MeasTrackCount > 37.5
              if (f_fixedPhaseSTD <= 0.01) {
                return 0.09 + DT_IMPURITY_FACTOR * 0.029;
              }
              else { // f_fixedPhaseSTD > 0.01
                return 0.17 + DT_IMPURITY_FACTOR * 0.078;
              }
            }
          }
          else { // f_fixedCodeSTD > 1.0
            if (u_SvTrackCount <= 28.5) {
              if (u_MeasTrackCount <= 39.5) {
                return 3.24 + DT_IMPURITY_FACTOR * 0.649;
              }
              else { // u_MeasTrackCount > 39.5
                return 1.87 + DT_IMPURITY_FACTOR * 0.428;
              }
            }
            else { // u_SvTrackCount > 28.5
              if (u_CN040 <= 19.5) {
                return 0.98 + DT_IMPURITY_FACTOR * 0.619;
              }
              else { // u_CN040 > 19.5
                return 0.35 + DT_IMPURITY_FACTOR * 0.283;
              }
            }
          }
        }
        else { // u_nb > 9.5
          if (f_ratio <= 1.96) {
            if (f_fixedCodeSTD <= 1.01) {
              if (f_cmc_avg <= 4.97) {
                return 0.06 + DT_IMPURITY_FACTOR * 0.022;
              }
              else { // f_cmc_avg > 4.97
                return 0.16 + DT_IMPURITY_FACTOR * 0.065;
              }
            }
            else { // f_fixedCodeSTD > 1.01
              if (q_QRcheckStatus <= 0.5) {
                return 0.62 + DT_IMPURITY_FACTOR * 0.650;
              }
              else { // q_QRcheckStatus > 0.5
                return 0.14 + DT_IMPURITY_FACTOR * 0.061;
              }
            }
          }
          else { // f_ratio > 1.96
            if (u_SvInUseCount <= 25.5) {
              if (pl_ekf_ver <= 0.06) {
                return 0.04;
              }
              else { // pl_ekf_ver > 0.06
                return 0.05 + DT_IMPURITY_FACTOR * 0.010;
              }
            }
            else { // u_SvInUseCount > 25.5
              if (f_ratio <= 38.87) {
                return 0.02;
              }
              else { // f_ratio > 38.87
                return 0.13;
              }
            }
          }
        }
      }
      else { // f_fixedPhaseSTD > 0.01
        if (u_fixedTotalSatCount <= 9.5) {
          if (f_fixedCodeSTD <= 0.49) {
            if (f_fixedCodeSTD <= 0.37) {
              if (f_ambPool_adop <= 7.7) {
                return 0.12 + DT_IMPURITY_FACTOR * 0.025;
              }
              else { // f_ambPool_adop > 7.7
                return 0.23 + DT_IMPURITY_FACTOR * 0.065;
              }
            }
            else { // f_fixedCodeSTD > 0.37
              if (f_cmc_avg <= 0.26) {
                return 1.71;
              }
              else { // f_cmc_avg > 0.26
                return 0.26 + DT_IMPURITY_FACTOR * 0.088;
              }
            }
          }
          else { // f_fixedCodeSTD > 0.49
            if (u_SvTrackCount <= 26.5) {
              if (d_lambda_sigma_1 <= 53.43) {
                return 4.35 + DT_IMPURITY_FACTOR * 0.037;
              }
              else { // d_lambda_sigma_1 > 53.43
                return 1.72 + DT_IMPURITY_FACTOR * 0.045;
              }
            }
            else { // u_SvTrackCount > 26.5
              if (f_ratio <= 2.04) {
                return 0.53 + DT_IMPURITY_FACTOR * 0.349;
              }
              else { // f_ratio > 2.04
                return 0.21 + DT_IMPURITY_FACTOR * 0.098;
              }
            }
          }
        }
        else { // u_fixedTotalSatCount > 9.5
          if (u_MeasInUseCount <= 29.5) {
            if (avg_unc_pr <= 6.18) {
              if (f_cmc_avg <= 1.05) {
                return 1.69;
              }
              else { // f_cmc_avg > 1.05
                return 0.28 + DT_IMPURITY_FACTOR * 0.121;
              }
            }
            else { // avg_unc_pr > 6.18
              if (f_ambPool_adop <= 1.5) {
                return 4.68;
              }
              else { // f_ambPool_adop > 1.5
                return 4.69;
              }
            }
          }
          else { // u_MeasInUseCount > 29.5
            if (f_ratio <= 1.68) {
              if (f_fixedCodeSTD <= 1.52) {
                return 0.19 + DT_IMPURITY_FACTOR * 0.102;
              }
              else { // f_fixedCodeSTD > 1.52
                return 0.97 + DT_IMPURITY_FACTOR * 0.801;
              }
            }
            else { // f_ratio > 1.68
              if (u_MeasInUseCount <= 59.5) {
                return 0.07 + DT_IMPURITY_FACTOR * 0.017;
              }
              else { // u_MeasInUseCount > 59.5
                return 1.46 + DT_IMPURITY_FACTOR * 0.522;
              }
            }
          }
        }
      }
    }
    else { // f_cn0_std > 7.78
      if (u_CN040 <= 17.5) {
        if (d_lambda_sigma_2 <= 3401.46) {
          if (f_cmc_std <= 2.52) {
            if (f_gdop <= 1.4) {
              if (u_ppp_use_sat_num_fix <= 12.5) {
                return 0.12 + DT_IMPURITY_FACTOR * 0.015;
              }
              else { // u_ppp_use_sat_num_fix > 12.5
                return 2.81 + DT_IMPURITY_FACTOR * 0.229;
              }
            }
            else { // f_gdop > 1.4
              if (u_SvTrackCount <= 26.5) {
                return 2.89;
              }
              else { // u_SvTrackCount > 26.5
                return 0.60 + DT_IMPURITY_FACTOR * 0.467;
              }
            }
          }
          else { // f_cmc_std > 2.52
            if (f_fixedCodeSTD <= 0.99) {
              if (d_lambda_sigma_1 <= 232.06) {
                return 0.54 + DT_IMPURITY_FACTOR * 0.228;
              }
              else { // d_lambda_sigma_1 > 232.06
                return 1.46 + DT_IMPURITY_FACTOR * 0.594;
              }
            }
            else { // f_fixedCodeSTD > 0.99
              if (d_lambda_sigma_2 <= 933.6) {
                return 2.20 + DT_IMPURITY_FACTOR * 0.571;
              }
              else { // d_lambda_sigma_2 > 933.6
                return 1.08 + DT_IMPURITY_FACTOR * 0.640;
              }
            }
          }
        }
        else { // d_lambda_sigma_2 > 3401.46
          if (f_ambPool_adop <= 1.67) {
            if (f_ratio <= 1.25) {
              if (f_ambPool_adop <= 1.48) {
                return 4.67;
              }
              else { // f_ambPool_adop > 1.48
                return 4.67;
              }
            }
            else { // f_ratio > 1.25
              if (f_ambPool_adop <= 1.64) {
                return 4.68;
              }
              else { // f_ambPool_adop > 1.64
                return 4.69;
              }
            }
          }
          else { // f_ambPool_adop > 1.67
            if (f_cmc_std <= 1.63) {
              if (d_lambda_sigma_1 <= 2745.93) {
                return 4.69;
              }
              else { // d_lambda_sigma_1 > 2745.93
                return 4.70;
              }
            }
            else { // f_cmc_std > 1.63
              if (d_lambda_sigma_1 <= 2690.93) {
                return 4.71;
              }
              else { // d_lambda_sigma_1 > 2690.93
                return 4.71;
              }
            }
          }
        }
      }
      else { // u_CN040 > 17.5
        if (pl_ekf_ver <= 0.07) {
          if (u_CN040 <= 21.5) {
            if (f_ambPool_pdop <= 4.09) {
              if (u_SvInUseCount <= 20.5) {
                return 0.60 + DT_IMPURITY_FACTOR * 0.424;
              }
              else { // u_SvInUseCount > 20.5
                return 1.64 + DT_IMPURITY_FACTOR * 1.105;
              }
            }
            else { // f_ambPool_pdop > 4.09
              if (d_lambda_sigma_1 <= 65.21) {
                return 0.44 + DT_IMPURITY_FACTOR * 0.267;
              }
              else { // d_lambda_sigma_1 > 65.21
                return 3.65 + DT_IMPURITY_FACTOR * 0.678;
              }
            }
          }
          else { // u_CN040 > 21.5
            if (f_ambPool_pdop <= 1.94) {
              if (f_vdop <= 1.17) {
                return 0.11 + DT_IMPURITY_FACTOR * 0.100;
              }
              else { // f_vdop > 1.17
                return 0.80 + DT_IMPURITY_FACTOR * 0.381;
              }
            }
            else { // f_ambPool_pdop > 1.94
              if (f_fixedCodeSTD <= 1.05) {
                return 0.41 + DT_IMPURITY_FACTOR * 0.395;
              }
              else { // f_fixedCodeSTD > 1.05
                return 1.01 + DT_IMPURITY_FACTOR * 0.724;
              }
            }
          }
        }
        else { // pl_ekf_ver > 0.07
          if (f_ratio <= 2.22) {
            if (f_fixedCodeSTD <= 1.44) {
              if (f_ambPool_adop <= 2.46) {
                return 0.11 + DT_IMPURITY_FACTOR * 0.019;
              }
              else { // f_ambPool_adop > 2.46
                return 0.45 + DT_IMPURITY_FACTOR * 0.325;
              }
            }
            else { // f_fixedCodeSTD > 1.44
              if (u_MeasInUseCount <= 26.5) {
                return 2.31 + DT_IMPURITY_FACTOR * 0.451;
              }
              else { // u_MeasInUseCount > 26.5
                return 0.79 + DT_IMPURITY_FACTOR * 0.507;
              }
            }
          }
          else { // f_ratio > 2.22
            if (f_ambPool_adop <= 8.27) {
              if (f_ambPool_pdop <= 6.59) {
                return 0.08 + DT_IMPURITY_FACTOR * 0.014;
              }
              else { // f_ambPool_pdop > 6.59
                return 0.28 + DT_IMPURITY_FACTOR * 0.058;
              }
            }
            else { // f_ambPool_adop > 8.27
              if (f_cmc_avg <= 2.57) {
                return 0.51 + DT_IMPURITY_FACTOR * 0.174;
              }
              else { // f_cmc_avg > 2.57
                return 0.12 + DT_IMPURITY_FACTOR * 0.024;
              }
            }
          }
        }
      }
    }
  }
}

/**
  * @brief horizontal protection level by Bagging DT model, max value from 10 trees model and EKF sigma
  * @param[in]
  * @return
  */
double integ_DecisionTreePredictHorFloat(integ_PppFeature_t* gpz_pppFeature) {

  double* pd_predict_list = OS_MALLOC_FAST(sizeof(double) * 7);
  double d_pred = 0.0;
  if (NULL == pd_predict_list)
  {
    return 0.0;
  }

  /* predict value from 10 trees */
  pd_predict_list[0] = predict_float_0(gpz_pppFeature);
  pd_predict_list[1] = predict_float_1(gpz_pppFeature);
  pd_predict_list[2] = predict_float_2(gpz_pppFeature);
  pd_predict_list[3] = predict_float_3(gpz_pppFeature);
  pd_predict_list[4] = predict_float_4(gpz_pppFeature);
  pd_predict_list[5] = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  pd_predict_list[6] = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];


  /* Bagging, max value from 10 trees model and pl_ekf*/
  d_pred = vector_dbl_max(pd_predict_list, 7);

  /* 0.5 BIAS */
  d_pred += 0.5;

  OS_FREE(pd_predict_list);
  return d_pred;
}

/**
 * @brief horizontal protection level by Bagging DT model, max value from 10 trees model and EKF sigma
 * @param[in]
 * @return
 */
double integ_DecisionTreePredictHorFix(integ_PppFeature_t* gpz_pppFeature) {

  double* pd_predict_list = OS_MALLOC_FAST(sizeof(double) * 3);
  double d_pred = 0.0;
  if (NULL == pd_predict_list)
  {
    return 0.0;
  }

  /* predict value from 10 trees */
  pd_predict_list[0] = predict_fix_0(gpz_pppFeature);
  pd_predict_list[1] = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[0];
  pd_predict_list[2] = gpz_pppFeature->z_pl_ekf_result.f_hv_protection_level[1];


  /* Bagging, max value from 10 trees model and pl_ekf*/
  d_pred = vector_dbl_max(pd_predict_list, 3);

  /* 0.5 BIAS */
  d_pred += 0.5;

  OS_FREE(pd_predict_list);
  return d_pred;
}